// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// SlipModel - the per-wheel tyre slip synthesiser behind the Lateral slip
// and Longitudinal slip tiles.
//
// Four wheels, two components. Each wheel carries its own severity for
// component A and component B (lateral: A = front scrub, B = rear slide;
// longitudinal: A = lock judder, B = spin tramp) and its own attack/
// release ramp, so the inside front locking alone judders ALONE. The
// oscillators are per AXLE, not per wheel: a car's body is one stiff
// structure, and a locking front corner shakes the whole front with a bias
// to that side, never the two front corners against each other. With an
// oscillator per wheel the two front posts of a rig drifted out of phase
// and rocked it side to side (found on the bench); now the two wheels of
// an axle share one carrier phase and roughness and differ only in level.
// A route asks for a Part (all, an axle, a corner) and gets the strongest
// wheel of that part per component: selecting, never summing.
//
// Severity staging, from the real thing: as slip grows past the limit the
// stick-slip chatter slows and roughens (squeal, moan, shudder). The
// carrier falls by kStageDrop at full severity and the jitter grows. The
// attack is the tile's attack ms (Longitudinal: 8, a tyre lets go in
// milliseconds; Lateral: 30 by default, a softer way in); the release is
// the usual fade.
//
// RT-safe: fixed arrays, no allocation, pure arithmetic.
// ============================================================

#include "HapticsTypes.h"
#include "NcxTokens.h"      // Wheel order
#include "WaveSynth.h"
#include <algorithm>
#include <cstdint>

namespace haptics {

namespace slip_k {
    constexpr double kAttackSec  = 0.008;   // the default attack (SlipParams::attackMs): a tyre lets go in ~5-10 ms
    constexpr double kReleaseSec = 0.120;
    constexpr double kStageDrop  = 0.35;    // carrier at full severity = (1 - this) x set hz
    constexpr double kStageRough = 0.6;     // jitter grows by this share at full severity
    constexpr double kMinHz      = 2.0;
    // The front tyres' screech and pulse (the Lateral tile's).
    constexpr double kScreechRough = 0.6;   // screech is always a rough texture, never a tone
    constexpr double kPulseSpread  = 0.25;  // each pulse cycle up to this much longer or shorter
    constexpr double kPulseDepthVar = 0.3;  // ...and up to this much shallower
    constexpr double kPulse2nd     = 0.3;   // a lopsided shove: this much second harmonic
    constexpr double kPulseNorm    = 1.0 / 1.25; // ...scaled back to a peak of about 1
}

class SlipModel
{
public:
    struct Out
    {
        double a = 0.0, b = 0.0;          // samples, -1..1 x mix x level
        double aHz = 0.0, bHz = 0.0;      // carriers (unjittered) for sink derating
        double aEnv = 0.0, bEnv = 0.0;    // the non-oscillatory part (level x mix)
        double aPhase = 0.0, bPhase = 0.0;// oscillator phase, radians: a shaker plays sin(harm x phase)
        // The fronts' screech (c) and pulse (d), where the law drives them.
        double c = 0.0, d = 0.0, cHz = 0.0, dHz = 0.0, cEnv = 0.0, cPhase = 0.0;
        double dEnv = 0.0, dPhase = 0.0;  // the pulse's envelope and phase: a shaker plays it at its harmonic
    };

    // Per wheel, per cycle, BEFORE step(): component severities 0..1.
    // A wheel not driven this cycle releases.
    void drive(int wheel, double a, double b)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        W& w = m_w[wheel];
        w.tA = std::max(0.0, std::min(1.0, a));
        w.tB = std::max(0.0, std::min(1.0, b));
    }

    // Per wheel, per cycle, after drive(): the screech and pulse levels 0..1
    // (the Lateral tile's fronts; never driven = none).
    void driveExtra(int wheel, double screech, double pulse)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        W& w = m_w[wheel];
        w.tC = std::max(0.0, std::min(1.0, screech));
        w.tD = std::max(0.0, std::min(1.0, pulse));
    }

    // Reshape a wheel's driven severities after drive() and before step()
    // (the ABS slip link): component A held between floorA and capA, then
    // each scaled (kA, kB).
    void shape(int wheel, double floorA, double capA, double kA, double kB)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        W& w = m_w[wheel];
        w.tA = std::max(0.0, std::min(1.0, std::min(capA, std::max(w.tA, floorA)) * kA));
        w.tB = std::max(0.0, std::min(1.0, w.tB * kB));
    }

    // Carrier multipliers applied on top of the set hz (lock judder falls
    // with road speed; 1 = as set).
    void setCarrierScale(double aScale, double bScale)
    {
        m_aScale = std::max(0.1, aScale);
        m_bScale = std::max(0.1, bScale);
    }

    void step(double dtSec, const EffectParams& p, const SlipParams& s)
    {
        // How fast a slide builds: the tile's attack ms (Longitudinal keeps
        // the default, a tyre letting go in milliseconds).
        const double attack = std::max(0.001, s.attackMs / 1000.0);
        for (W& w : m_w)
        {
            ramp(w.lA, w.tA, dtSec, attack);
            ramp(w.lB, w.tB, dtSec, attack);
            ramp(w.lC, w.tC, dtSec, attack);
            ramp(w.lD, w.tD, dtSec, attack);
            w.tA = w.tB = w.tC = w.tD = 0.0;    // a law that stops driving = release
        }
        // One oscillator per axle and component, staged by the axle's
        // stronger wheel; each wheel scales the shared waveform by its own
        // level, so the two wheels of an axle always move together.
        for (int a = 0; a < kAxles; ++a)
        {
            Axle& x = m_axle[a];
            const W& w0 = m_w[2 * a];
            const W& w1 = m_w[2 * a + 1];
            x.uA = unit(std::max(w0.lA, w1.lA), s.aHz * m_aScale, s.aMix, p.jitter, x.oscA, x.rng, dtSec, x.hzA);
            x.uB = unit(std::max(w0.lB, w1.lB), s.bHz * m_bScale, s.bMix, p.jitter, x.oscB, x.rng, dtSec, x.hzB);
            x.uC = unit(std::max(w0.lC, w1.lC), s.screechHz, s.screech, std::max(p.jitter, slip_k::kScreechRough), x.oscC, x.rng, dtSec, x.hzC, 0.0);
            x.uD = pulseUnit(x, std::max(w0.lD, w1.lD), s.pulseHz, s.pulse, dtSec);
        }
        for (int i = 0; i < WHEEL_COUNT; ++i)
        {
            W& w = m_w[i];
            const Axle& x = m_axle[axleOf(i)];
            w.hzA = (w.lA >= 1e-4) ? x.hzA : 0.0;
            w.hzB = (w.lB >= 1e-4) ? x.hzB : 0.0;
            w.sA  = (w.hzA > 0.0) ? x.uA * w.lA * s.aMix : 0.0;
            w.sB  = (w.hzB > 0.0) ? x.uB * w.lB * s.bMix : 0.0;
            w.envA = (w.hzA > 0.0) ? w.lA * s.aMix : 0.0;
            w.envB = (w.hzB > 0.0) ? w.lB * s.bMix : 0.0;
            w.hzC = (w.lC >= 1e-4) ? x.hzC : 0.0;
            w.hzD = (w.lD >= 1e-4) ? x.hzD : 0.0;
            w.sC  = (w.hzC > 0.0) ? x.uC * w.lC * s.screech : 0.0;
            w.sD  = (w.hzD > 0.0) ? x.uD * w.lD * s.pulse : 0.0;
            w.envC = (w.hzC > 0.0) ? w.lC * s.screech : 0.0;
            w.envD = (w.hzD > 0.0) ? w.lD * s.pulse * x.dDepth * slip_k::kPulseNorm : 0.0;
        }
    }

    // The strongest wheel of the part, per component.
    Out outputFor(Part part) const
    {
        int first = 0, last = WHEEL_COUNT - 1;
        switch (part)
        {
            case Part::Front: first = WheelFL; last = WheelFR; break;
            case Part::Rear:  first = WheelRL; last = WheelRR; break;
            case Part::FL:    first = last = WheelFL; break;
            case Part::FR:    first = last = WheelFR; break;
            case Part::RL:    first = last = WheelRL; break;
            case Part::RR:    first = last = WheelRR; break;
            default: break;
        }
        Out o;
        double bestA = 0.0, bestB = 0.0, bestC = 0.0, bestD = 0.0;
        for (int i = first; i <= last; ++i)
        {
            const W& w = m_w[i];
            const Axle& x = m_axle[axleOf(i)];
            if (w.lA > bestA) { bestA = w.lA; o.a = w.sA; o.aHz = w.hzA; o.aEnv = w.envA; o.aPhase = x.oscA.phase; }
            if (w.lB > bestB) { bestB = w.lB; o.b = w.sB; o.bHz = w.hzB; o.bEnv = w.envB; o.bPhase = x.oscB.phase; }
            if (w.lC > bestC) { bestC = w.lC; o.c = w.sC; o.cHz = w.hzC; o.cEnv = w.envC; o.cPhase = x.oscC.phase; }
            if (w.lD > bestD) { bestD = w.lD; o.d = w.sD; o.dHz = w.hzD; o.dEnv = w.envD; o.dPhase = x.dPhase; }
        }
        return o;
    }

    // One wheel's sample this cycle (tests: the two wheels of an axle are
    // in phase).
    double wheelSampleA(int wheel) const { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].sA : 0.0; }
    double wheelSampleB(int wheel) const { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].sB : 0.0; }

    // Highest smoothed level anywhere, screech and pulse included (status /
    // wave; the layer renders nothing below it).
    double level() const
    {
        double m = 0.0;
        for (const W& w : m_w) m = std::max(m, std::max(std::max(w.lA, w.lB), std::max(w.lC, w.lD)));
        return m;
    }
    double wheelScreech(int wheel) const { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].lC : 0.0; }
    double wheelPulse(int wheel) const   { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].lD : 0.0; }

    double wheelLevel(int wheel) const
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return 0.0;
        return std::max(m_w[wheel].lA, m_w[wheel].lB);
    }

    void clear()
    {
        for (W& w : m_w) { w = W{}; }
        for (int a = 0; a < kAxles; ++a) { m_axle[a] = Axle{}; m_axle[a].rng = kSeeds[a]; }
    }

private:
    struct W
    {
        double tA = 0.0, tB = 0.0, tC = 0.0, tD = 0.0;   // driven this cycle (C screech, D pulse)
        double lA = 0.0, lB = 0.0, lC = 0.0, lD = 0.0;   // smoothed
        double sA = 0.0, sB = 0.0, sC = 0.0, sD = 0.0;   // this cycle's samples
        double hzA = 0.0, hzB = 0.0, hzC = 0.0, hzD = 0.0; // this cycle's carriers
        double envA = 0.0, envB = 0.0, envC = 0.0, envD = 0.0;   // this cycle's envelopes (level x mix)
    };
    // Front axle (FL, FR) and rear axle (RL, RR): one oscillator per
    // component, its unit sample and carrier this cycle.
    static constexpr int kAxles = 2;
    static constexpr uint64_t kSeeds[kAxles] = { 0x9E3779B97F4A7C15ull, 0xD1B54A32D192ED03ull };
    struct Axle
    {
        wavesynth::Oscillator oscA, oscB;
        double uA = 0.0, uB = 0.0;
        double hzA = 0.0, hzB = 0.0;
        uint64_t rng = 0x9E3779B97F4A7C15ull;
        // Screech oscillator; the pulse's phase, this cycle's rate and depth.
        wavesynth::Oscillator oscC;
        double uC = 0.0, uD = 0.0, hzC = 0.0, hzD = 0.0;
        double dPhase = 0.0, dRate = 1.0, dDepth = 1.0;
    };
    static int axleOf(int wheel) { return (wheel == WheelFL || wheel == WheelFR) ? 0 : 1; }

    static void ramp(double& level, double target, double dtSec, double attackSec)
    {
        using namespace slip_k;
        const double rate = (target > level) ? dtSec / attackSec : dtSec / kReleaseSec;
        level += std::max(-rate, std::min(rate, target - level));
    }

    // The axle's waveform at unit amplitude, staged by its stronger wheel.
    static double unit(double level, double setHz, double mix, double jitter,
                       wavesynth::Oscillator& osc, uint64_t& rng, double dtSec, double& hzOut,
                       double stageDrop = slip_k::kStageDrop)
    {
        using namespace slip_k;
        hzOut = 0.0;
        if (level < 1e-4 || mix <= 0.0 || setHz <= 0.0) return 0.0;
        const double carrier = std::max(kMinHz, setHz * (1.0 - stageDrop * level));
        hzOut = carrier;
        double f = carrier;
        const double jit = jitter * (1.0 + kStageRough * level);
        if (jit > 0.0)
        {
            rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
            const double r = static_cast<double>(rng & 0xFFFF) / 65535.0;
            f *= 1.0 + jit * (r - 0.5);
        }
        return osc.step(f, dtSec);
    }

    // The pulse: a slow, lopsided shove, each cycle a little longer or
    // shorter and shallower than the last, so it never reads as a metronome.
    static double pulseUnit(Axle& x, double level, double hz, double mix, double dtSec)
    {
        using namespace slip_k;
        x.hzD = 0.0;
        if (level < 1e-4 || mix <= 0.0 || hz <= 0.0) { x.dPhase = 0.0; return 0.0; }
        x.hzD = hz * x.dRate;
        x.dPhase += 2.0 * wavesynth::kPi * x.hzD * dtSec;
        if (x.dPhase >= 2.0 * wavesynth::kPi)
        {
            x.dPhase -= 2.0 * wavesynth::kPi;
            x.rng ^= x.rng << 13; x.rng ^= x.rng >> 7; x.rng ^= x.rng << 17;
            const double r1 = static_cast<double>(x.rng & 0xFFFF) / 65535.0;
            const double r2 = static_cast<double>((x.rng >> 16) & 0xFFFF) / 65535.0;
            x.dRate  = 1.0 + kPulseSpread * (2.0 * r1 - 1.0);
            x.dDepth = 1.0 - kPulseDepthVar * r2;
        }
        return x.dDepth * kPulseNorm * (std::sin(x.dPhase) + kPulse2nd * std::sin(2.0 * x.dPhase));
    }

    W      m_w[WHEEL_COUNT];
    Axle   m_axle[kAxles] = { makeAxle(kSeeds[0]), makeAxle(kSeeds[1]) };
    static Axle makeAxle(uint64_t seed) { Axle x; x.rng = seed; return x; }
    double m_aScale = 1.0, m_bScale = 1.0;
};

} // namespace haptics
