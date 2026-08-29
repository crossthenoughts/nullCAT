// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// DeviceForceModel - the control-loading force field (device families:
// shifter, pedal; later rudder/stick rows reuse it unchanged).
//
// Pure engine, CommissioningMode-style: knows nothing about drives, SOEM,
// or Qt. MotionController feeds the axis's own position (motor-shaft revs
// from the homed reference) plus dt; the model returns a signed torque in
// % of rated. RT-safe: fixed state, no allocation, no locking.
//
// Layer model:
//   L0 geometry + L1 character come from DeviceParams (config; presets
//     fill it; the web curve editor edits its node arrays).
//   L2 state modifiers (DeviceStateMods) arrive PER CYCLE and default to
//     inert - the NULLCATX wire feeds them later (grind/blocking/trim);
//     nothing here changes shape when that lands.
//   L3 guards run inside step(), always, in fixed order after the field:
//     thermal dwell -> slew cap -> velocity fold -> clamp. State can
//     never override them.
//
// Frame convention: the MODEL works entirely in the HOME frame (positions
// exactly as configured: neutral, detents, stops). `dir` maps the model's
// force onto the motor's torque sign at the very last step - so a
// mirrored mechanical build flips ONE number, never the geometry.
//
// Curve convention: y = the force RESISTING displacement at x (the force
// the hand feels), sampled piecewise-linear, ends clamped. The engine
// applies the restoring sign. An empty curve contributes nothing.
// ============================================================

#include "Config.h"      // DeviceParams, CurveNode
#include "WaveSynth.h"
#include <cmath>
#include <vector>

// L2 state modifiers. Defaults are inert; the state layer (wire-driven)
// writes these per cycle. Generic on purpose: trim = neutralShift,
// airspeed loading = forceScale, grind/buffet = the texture pair.
struct DeviceStateMods
{
    double forceScale    = 1.0;
    double neutralShift  = 0.0;   // revs, added to DeviceParams::neutralRev
    double textureAmpPct = 0.0;   // overlay amplitude (% of rated)
    double textureFreqHz = 0.0;
};

class DeviceForceModel
{
public:
    void configure(const DeviceParams& p, double dtSec)
    {
        m_p  = p;
        m_dt = (dtSec > 0.0) ? dtSec : 0.002;
        reset(0.0);
    }

    // Seed at engage: no output step on the first cycle, texture phase fresh.
    void reset(double posRev)
    {
        m_prevPos  = posRev;
        m_havePrev = false;
        m_vel      = 0.0;
        m_lastOut  = 0.0;
        m_dwellSec = 0.0;
        m_relaxed  = false;
        m_tex.reset();
    }

    bool thermallyRelaxed() const { return m_relaxed; }

    // posRev: motor revs from home. Returns motor torque in % of rated,
    // guard chain applied. Deterministic for a given input sequence.
    double step(double posRev, const DeviceStateMods& mods)
    {
        // ---- velocity estimate, low-passed (sign preserved) ----
        double vRaw = 0.0;
        if (m_havePrev)
            vRaw = (posRev - m_prevPos) / m_dt;
        m_prevPos  = posRev;
        m_havePrev = true;
        const double a = (m_p.velLpfHz > 0.0)
            ? std::min(1.0, 2.0 * wavesynth::kPi * m_p.velLpfHz * m_dt) : 1.0;
        m_vel += a * (vRaw - m_vel);

        // ---- L0/L1 force field (home frame) ----
        const double neutral = m_p.neutralRev + mods.neutralShift;
        double x = posRev - neutral;

        // Lash: a zero-force free-play band about neutral; displacement is
        // measured from the band's edge beyond it.
        if (m_p.lashRev > 0.0)
        {
            if (std::fabs(x) <= m_p.lashRev) x = 0.0;
            else x -= (x > 0.0 ? m_p.lashRev : -m_p.lashRev);
        }

        // Centring spring: curve y resists displacement; restoring sign here.
        double f = -restoring(m_p.springCurve, x);

        // Nearest detent (positions are home-frame; profile x is relative to
        // the detent centre). Only the nearest one acts.
        if (!m_p.detents.empty() && !m_p.detentCurve.empty())
        {
            double best = m_p.detents[0];
            for (double d : m_p.detents)
                if (std::fabs(posRev - d) < std::fabs(posRev - best)) best = d;
            f += -restoring(m_p.detentCurve, posRev - best);
        }

        // Soft end stops: stiff spring past the stop, extra damping inside it.
        if (posRev > m_p.stopMaxRev)
            f += -m_p.stopSpring * (posRev - m_p.stopMaxRev) - m_p.stopDamp * m_vel;
        else if (posRev < m_p.stopMinRev)
            f += -m_p.stopSpring * (posRev - m_p.stopMinRev) - m_p.stopDamp * m_vel;

        // Viscous damping everywhere.
        f -= m_p.dampPctPerRevS * m_vel;

        // ---- L2 modifiers ----
        f *= mods.forceScale;
        if (mods.textureAmpPct > 0.0 && mods.textureFreqHz > 0.0)
            f += mods.textureAmpPct * m_tex.step(mods.textureFreqHz, m_dt);

        // ---- L3 guards, fixed order ----
        // Thermal dwell: sustained near-ceiling output eases to zero until
        // demand drops (pre-empts drive i2t). Hysteresis band of 10 points.
        if (m_p.thermalDwellSec > 0.0)
        {
            const double band = m_p.maxForcePct * m_p.thermalPct / 100.0;
            if (std::fabs(f) >= band) m_dwellSec += m_dt;
            else if (std::fabs(f) < m_p.maxForcePct * (m_p.thermalPct - 10.0) / 100.0)
            { m_dwellSec = 0.0; m_relaxed = false; }
            if (m_dwellSec >= m_p.thermalDwellSec) m_relaxed = true;
            if (m_relaxed) f = 0.0;
        }

        // Slew cap (a feel knob at device defaults, still a safety envelope
        // for a garbage modifier or curve edit landing mid-session).
        {
            const double maxStep = m_p.slewPctPerSec * m_dt;
            f = std::max(m_lastOut - maxStep, std::min(m_lastOut + maxStep, f));
        }

        // Velocity fold (anti-runaway, AFTER the slew so the fold is never
        // rate-limited downward): output folds to zero across a half-knee
        // band above foldRpm.
        if (m_p.foldRpm > 0.0)
        {
            const double rpm  = std::fabs(m_vel) * 60.0;
            const double band = 0.5 * m_p.foldRpm;
            if (rpm >= m_p.foldRpm + band) f = 0.0;
            else if (rpm > m_p.foldRpm)
                f *= 1.0 - (rpm - m_p.foldRpm) / band;
        }

        // Model output clamp. Above this only the drive's own 0x6072
        // limit still caps (the axis torqueMaxPct is belt machinery and
        // is NOT applied on the device path).
        f = std::max(-m_p.maxForcePct, std::min(m_p.maxForcePct, f));

        m_lastOut = f;
        // `dir` maps model force onto motor torque sign, at the very end.
        return m_p.dir * f;
    }

    // Piecewise-linear sample; ends clamped; empty curve = 0.
    static double sampleCurve(const std::vector<CurveNode>& c, double x)
    {
        if (c.empty()) return 0.0;
        if (x <= c.front().x) return c.front().y;
        if (x >= c.back().x)  return c.back().y;
        for (size_t i = 1; i < c.size(); ++i)
            if (x <= c[i].x)
            {
                const double t = (x - c[i - 1].x) / (c[i].x - c[i - 1].x);
                return c[i - 1].y + t * (c[i].y - c[i - 1].y);
            }
        return c.back().y;
    }

private:
    // Curve y resists displacement: force magnitude at |offset|, signed to
    // oppose the offset's direction. A curve with signed x (asymmetric
    // pedal preload) is sampled directly; the sign convention still holds
    // because the drawn y opposes motion away from neutral.
    static double restoring(const std::vector<CurveNode>& c, double x)
    {
        if (c.empty()) return 0.0;
        const bool signedCurve = (c.front().x < 0.0);
        if (signedCurve) return sampleCurve(c, x);
        return (x >= 0.0) ? sampleCurve(c, x) : -sampleCurve(c, -x);
    }

    DeviceParams m_p;
    double m_dt      = 0.002;
    double m_prevPos = 0.0;
    double m_vel     = 0.0;
    bool   m_havePrev = false;
    double m_lastOut = 0.0;
    double m_dwellSec = 0.0;
    bool   m_relaxed  = false;
    wavesynth::Oscillator m_tex;
};
