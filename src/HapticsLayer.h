// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// HapticsLayer - one-shot haptic transients, routable across axes.
//
// A separate layer that RIDES ON TOP of the per-axis force pipeline and
// never modifies it: sources (detent capture, wall contact) fire an event;
// the layer synthesizes a short enveloped sine burst; a per-event routing
// table says which torque-mode axes feel it and at what gain, so a shifter
// click can also thump the belt tensioners. The owner (MotionController)
// adds overlayFor(axis) to each torque command AFTER its normal guards and
// clamps the sum inside the axis's existing limits - this layer can never
// exceed what the guard rails already allow.
//
// RT rules, by construction: fixed pools, no allocation, no locks, no
// logging, no mailbox. fire() and step()/overlayFor() are called from the
// same RT thread; the web Test button arrives via the existing command
// queue like every other command. Params and routes are SNAPSHOTTED into
// the event at fire time, so a mid-burst config edit never tears.
//
// Envelope: raised-cosine (wavesynth::envelope), exactly zero at both
// ends - a burst can never step the torque command on entry or exit.
// Amplitude may be scaled at fire time (impact velocity), so a gentle
// detent entry clicks softly and a slam clicks hard.
// ============================================================

#include "HapticsTypes.h"
#include "EngineModel.h"
#include "SlipModel.h"
#include "RoadModel.h"
#include "WaveSynth.h"
#include <algorithm>
#include <cstdint>

namespace haptics {

class Layer
{
public:
    // Set/replace the tuning for one event type (config apply; not RT).
    void configure(EventType t, const EffectParams& p)
    {
        m_params[static_cast<int>(t)] = p;
    }

    const EffectParams& params(EventType t) const
    {
        return m_params[static_cast<int>(t)];
    }

    // Fire one transient. scale in [0,1] multiplies the configured
    // amplitude (impact velocity). Inert when the effect's ampPct is 0 or
    // every route is dead. When the pool is full the event is dropped (a
    // missed click is harmless; a stalled RT loop is not).
    void fire(EventType t, double scale = 1.0)
    {
        const EffectParams& p = m_params[static_cast<int>(t)];
        if (p.ampPct <= 0.0 || p.durMs <= 0.0) return;
        if (scale <= 0.0) return;
        if (scale > 1.0) scale = 1.0;

        Event* e = nullptr;
        for (Event& c : m_events) if (!c.active) { e = &c; break; }
        if (!e) return;                         // pool full: drop, never block

        // Snapshot params + routes at fire time.
        e->ampPct = p.ampPct * scale;
        e->freqHz = (p.freqHz > 0.0) ? p.freqHz : 90.0;
        e->durSec = p.durMs / 1000.0;
        e->tSec   = 0.0;
        bool anyRoute = false;
        for (int i = 0; i < MAX_ROUTES; ++i)
        {
            const bool ok = p.routes[i].axis >= 0 && p.routes[i].axis < MAX_HAPTIC_AXES
                            && p.routes[i].gain > 0.0;
            e->routes[i].axis = ok ? p.routes[i].axis : -1;
            e->routes[i].gain = ok ? p.routes[i].gain : 0.0;
            anyRoute |= ok;
        }
        if (!anyRoute) return;                  // fully unrouted: never activate
        e->active = true;
        ++m_fired;
    }

    // Advance every active transient and continuous effect by one cycle
    // and refresh the per-axis overlay sums. Call ONCE per control cycle
    // (after driveFx calls), before overlayFor().
    void step(double dtSec)
    {
        for (double& o : m_overlay) o = 0.0;
        if (dtSec <= 0.0) return;

        // One-shot transients.
        for (Event& e : m_events)
        {
            if (!e.active) continue;
            e.tSec += dtSec;
            if (e.tSec >= e.durSec) { e.active = false; continue; }
            // Raised-cosine envelope (zero at both ends) on a sine carrier.
            const double env = wavesynth::envelope(e.tSec, e.durSec, e.durSec * 0.25);
            const double v   = e.ampPct * env
                             * std::sin(2.0 * wavesynth::kPi * e.freqHz * e.tSec);
            for (const Route& r : e.routes)
                if (r.axis >= 0) m_overlay[r.axis] += v * r.gain * sinkScale(r.axis, e.ampPct * r.gain, e.freqHz);
        }

        // Continuous effects: attack/release-smoothed level on a free-running
        // oscillator (phase-continuous through frequency changes), optional
        // per-cycle carrier jitter for texture-class effects.
        for (int i = 0; i < FX_TYPE_COUNT; ++i)
        {
            Fx& f = m_fx[i];
            const EffectParams& p = m_fxParams[i];
            const bool slipSlot = slipFor(static_cast<FxType>(i)) != nullptr;
            // Road replay: per-corner suspension travel when the sim sends
            // it (the law drives the corners); the texture oscillator is the
            // fallback and runs below when it does not.
            if (i == static_cast<int>(FxType::Road) && (m_roadDriven || m_road.active()))
            {
                m_roadDriven = false;
                if (p.ampPct <= 0.0) { m_road.clear(); f.level = 0.0; continue; }
                m_road.step(dtSec, m_roadParams);
                f.level = m_road.level();
                f.targetLevel = 0.0;
                if (f.level < 1e-4) continue;
                for (const Route& r : p.routes)
                {
                    if (r.axis < 0 || r.axis >= MAX_HAPTIC_AXES || r.gain <= 0.0) continue;
                    const double s = m_road.outputFor(r.part);
                    if (s == 0.0) continue;
                    // No single carrier in a replay: derate a position sink
                    // at a representative bump rate; the owner's sum guard
                    // holds the axis limits regardless.
                    m_overlay[r.axis] += p.ampPct * s * r.gain * sinkScale(r.axis, p.ampPct * r.gain, road_k::kDerateHz);
                }
                continue;
            }
            if (!slipSlot)
            {
                // Level ramp: ~50 ms attack, ~120 ms release. (The slip
                // model ramps per wheel itself.)
                const double rate = (f.targetLevel > f.level) ? dtSec / 0.050 : dtSec / 0.120;
                f.level += std::max(-rate, std::min(rate, f.targetLevel - f.level));
                if (p.ampPct <= 0.0 || f.level < 1e-4) continue;
            }
            else if (p.ampPct <= 0.0)
            {
                slipFor(static_cast<FxType>(i))->clear();
                f.level = 0.0;
                continue;
            }

            // The engine slot runs the pulse-train synth, not the oscillator.
            if (i == static_cast<int>(FxType::RpmVibe))
            {
                const EngineOut eo = m_engine.step(dtSec, p, m_engineParams, f.level, f.rng);
                const double v = eo.total();
                for (const Route& r : p.routes)
                {
                    if (r.axis < 0 || r.axis >= MAX_HAPTIC_AXES || r.gain <= 0.0) continue;
                    if (m_sinkKind[r.axis] == SinkKind::Position)
                    {
                        // Each component derated by what this axis can follow
                        // at ITS carrier: the idle rock survives on a vertical
                        // where the thumps and buzz are tiny.
                        const double ask = p.ampPct * r.gain;
                        double pv = eo.rock   * sinkScale(r.axis, ask, eo.rockHz)
                                  + eo.thumps * sinkScale(r.axis, ask, eo.thumpHz)
                                  + eo.buzz   * sinkScale(r.axis, ask, eo.buzzHz);
                        pv = std::max(-eo.cap, std::min(eo.cap, pv));
                        m_overlay[r.axis] += pv * r.gain;
                    }
                    else if (v != 0.0)
                        m_overlay[r.axis] += v * r.gain;
                }
                continue;
            }

            // The two slip slots run the per-wheel model: each route takes
            // the strongest wheel of its part, per component, derated per
            // component carrier on a position sink.
            if (SlipModel* sm = slipFor(static_cast<FxType>(i)))
            {
                sm->step(dtSec, p, *slipParamsFor(static_cast<FxType>(i)));
                f.level = sm->level();
                if (f.level < 1e-4) continue;
                for (const Route& r : p.routes)
                {
                    if (r.axis < 0 || r.axis >= MAX_HAPTIC_AXES || r.gain <= 0.0) continue;
                    const SlipModel::Out o = sm->outputFor(r.part);
                    const double ask = p.ampPct * r.gain;
                    const double v = p.ampPct * (o.a * sinkScale(r.axis, ask, o.aHz)
                                               + o.b * sinkScale(r.axis, ask, o.bHz));
                    if (v != 0.0) m_overlay[r.axis] += v * r.gain;
                }
                continue;
            }

            double freq = f.freqHz;
            if (freq < 0.5) continue;   // no usable carrier = silence, not DC
            if (p.jitter > 0.0)
            {
                // xorshift PRNG: pure arithmetic, RT-safe. Roughens the
                // carrier so skid/road read as texture, not a tone.
                f.rng ^= f.rng << 13; f.rng ^= f.rng >> 7; f.rng ^= f.rng << 17;
                const double r = static_cast<double>(f.rng & 0xFFFF) / 65535.0; // 0..1
                freq *= 1.0 + p.jitter * (r - 0.5);
            }
            const double v = p.ampPct * f.level * f.osc.step(freq, dtSec);
            for (const Route& r : p.routes)
                if (r.axis >= 0 && r.axis < MAX_HAPTIC_AXES && r.gain > 0.0)
                    m_overlay[r.axis] += v * r.gain * sinkScale(r.axis, p.ampPct * r.gain, f.freqHz);
        }

        // Master trim last: per-effect settings stay untouched underneath.
        const double mg = m_muted ? 0.0 : m_masterGain;
        if (mg != 1.0) for (double& o : m_overlay) o *= mg;
    }

    // Torque overlay (% of rated) for one axis this cycle. The caller adds
    // it to the axis command and clamps the SUM inside the axis limits.
    double overlayFor(int axis) const
    {
        return (axis >= 0 && axis < MAX_HAPTIC_AXES) ? m_overlay[axis] : 0.0;
    }

    // ---- sinks -----------------------------------------------------------
    // What each axis index IS as a destination. Torque (the default): the
    // overlay is % of rated torque, a route gain is a plain multiplier.
    // Position: a route gain is mm at 100% effect amplitude, the overlay is
    // (% x gain) and the owner divides by 100 to get mm; every contribution
    // is first derated to what the axis can physically follow at the
    // effect's carrier - min(cap, vBudget/w, aBudget/w^2), the commissioning
    // sweep's rule - so a 35 Hz texture on a heavy vertical arrives at the
    // physics amplitude instead of a demand the drive would clip, while the
    // 13 Hz idle rock comes through. Config apply; not RT.
    void setSinkKind(int axis, SinkKind k)
    {
        if (axis >= 0 && axis < MAX_HAPTIC_AXES) m_sinkKind[axis] = k;
    }
    void setPositionLimits(int axis, double vBudgetMmS, double aBudgetMmS2, double capMm)
    {
        if (axis < 0 || axis >= MAX_HAPTIC_AXES) return;
        m_posV[axis]   = std::max(0.0, vBudgetMmS);
        m_posA[axis]   = std::max(0.0, aBudgetMmS2);
        m_posCap[axis] = std::max(0.0, capMm);
    }
    SinkKind sinkKind(int axis) const
    {
        return (axis >= 0 && axis < MAX_HAPTIC_AXES) ? m_sinkKind[axis] : SinkKind::Torque;
    }
    // Largest peak offset (mm) a position axis can take at a carrier.
    double positionAllowedMm(int axis, double hz) const
    {
        if (axis < 0 || axis >= MAX_HAPTIC_AXES) return 0.0;
        double allowed = m_posCap[axis];
        if (hz > 0.0)
        {
            const double w = 2.0 * wavesynth::kPi * hz;
            allowed = std::min(allowed, std::min(m_posV[axis] / w, m_posA[axis] / (w * w)));
        }
        return allowed;
    }

    // ---- continuous effects -------------------------------------------------

    void configureFx(FxType t, const EffectParams& p)
    {
        m_fxParams[static_cast<int>(t)] = p;
    }

    const EffectParams& fxParams(FxType t) const
    {
        return m_fxParams[static_cast<int>(t)];
    }

    // Engine description + mix (config apply; not RT).
    void configureEngine(const EngineParams& e) { m_engineParams = e; }
    const EngineParams& engineParams() const   { return m_engineParams; }
    double learnedMaxRpm() const               { return m_engine.learnedMaxRpm(); }

    // ---- per-wheel slip (the Lockup = longitudinal and Skid = lateral
    // slots). Config apply for the params; the law drives wheels per cycle.
    void configureSlip(FxType t, const SlipParams& s)
    {
        if (SlipParams* sp = slipParamsFor(t)) *sp = s;
    }
    const SlipParams& slipParams(FxType t) const
    {
        return (t == FxType::Skid) ? m_slipLatParams : m_slipLonParams;
    }
    // Component severities (0..1) for one wheel this cycle. Lateral: a =
    // scrub, b = slide. Longitudinal: a = lock, b = spin.
    void driveSlip(FxType t, int wheel, double a, double b)
    {
        if (SlipModel* sm = slipFor(t)) sm->drive(wheel, a, b);
    }
    void setSlipCarrierScale(FxType t, double aScale, double bScale)
    {
        if (SlipModel* sm = slipFor(t)) sm->setCarrierScale(aScale, bScale);
    }
    double slipWheelLevel(FxType t, int wheel) const
    {
        const SlipModel* sm = (t == FxType::Skid) ? &m_slipLat : (t == FxType::Lockup) ? &m_slipLon : nullptr;
        return sm ? sm->wheelLevel(wheel) : 0.0;
    }
    static bool isSlipSlot(FxType t) { return t == FxType::Skid || t == FxType::Lockup; }

    // ---- per-corner road replay (the Road slot). Config apply for the
    // params; the law drives the corners per cycle when the sim sends
    // suspension velocities, and does not when it sends roadNoise.
    void configureRoad(const RoadParams& r) { m_roadParams = r; }
    const RoadParams& roadParams() const   { return m_roadParams; }
    void driveRoad(int wheel, double velMmS)
    {
        m_road.drive(wheel, velMmS);
        m_roadDriven = true;
    }
    double roadWheelTravelMm(int wheel) const { return m_road.wheelTravelMm(wheel); }
    bool   roadReplaying() const              { return m_road.active(); }

    // Drive one continuous effect for THIS cycle: level 0..1 (silence to
    // full configured amplitude) and the carrier frequency to use (RpmVibe
    // passes rpm/60 x order; others pass their configured freqHz). Called
    // every cycle by the owner BEFORE step(); a level not driven this
    // cycle decays on the release ramp, so a crashed source fades out
    // rather than droning.
    void driveFx(FxType t, double level, double freqHz)
    {
        Fx& f = m_fx[static_cast<int>(t)];
        f.targetLevel = (level < 0.0) ? 0.0 : (level > 1.0 ? 1.0 : level);
        // No driven frequency = the effect's configured carrier.
        f.freqHz = (freqHz > 0.0) ? freqHz : m_fxParams[static_cast<int>(t)].freqHz;
    }

    // ---- master trim (RT-thread writes only, like everything here) ----
    // gain scales EVERY overlay (0..2); mute is a hard zero. Both applied
    // at the final sum, so per-effect settings stay untouched underneath.
    void setMasterGain(double g) { m_masterGain = (g < 0.0) ? 0.0 : (g > 2.0 ? 2.0 : g); }
    void setMuted(bool m)        { m_muted = m; }
    bool muted() const           { return m_muted; }

    // Live level of one continuous effect (0..1 smoothed) - status surface.
    double fxLevel(int i) const
    { return (i >= 0 && i < FX_TYPE_COUNT) ? m_fx[i].level : 0.0; }

    // What the effect is actually PUTTING OUT, not what it is being driven
    // with: zero when its amplitude is 0, it has no route with gain, or the
    // layer is muted. The web wave draws this, so a tile only animates when
    // something can be felt (a bench session read a full wave on an effect
    // at amp 0 / no route as "fires visually, nothing felt").
    double fxOutputLevel(int i) const
    {
        if (i < 0 || i >= FX_TYPE_COUNT || m_muted || m_masterGain <= 0.0) return 0.0;
        const EffectParams& p = m_fxParams[i];
        if (p.ampPct <= 0.0 || !hasRoute(p)) return 0.0;
        return m_fx[i].level;
    }

    static bool hasRoute(const EffectParams& p)
    {
        for (const Route& r : p.routes)
            if (r.axis >= 0 && r.axis < MAX_HAPTIC_AXES && r.gain > 0.0) return true;
        return false;
    }

    // ---- pulse-train engine (replaces the generic oscillator for the
    // RpmVibe slot). A real engine FIRES rather than hums: each firing is
    // a short damped thump at rpm/60 x cylinders/2. Low rpm = discrete
    // chunky pulses; rising rpm merges them into buzz by physics, not by
    // crossfade. load scales pulse strength (lugging hits harder than
    // coasting), the lope amount (EffectParams.jitter) roughens idle
    // per-pulse, and the limiter flag DROPS pulses in bursts - a limiter
    // cuts firings, so the stumble is missing events, exactly as felt. ----
    void driveEngine(double level, double fireHz, double load01, bool limiterOn)
    {
        Fx& f = m_fx[static_cast<int>(FxType::RpmVibe)];
        f.targetLevel = (level < 0.0) ? 0.0 : (level > 1.0 ? 1.0 : level);
        m_engine.drive(fireHz, load01, limiterOn);
    }

    // Kill every active transient instantly (e-stop, park, loop stop).
    void clearAll()
    {
        for (Event& e : m_events) e.active = false;
        for (Fx& f : m_fx) { f.targetLevel = 0.0; f.level = 0.0; f.osc.reset(); }
        m_engine.clear();
        m_slipLat.clear();
        m_slipLon.clear();
        m_road.clear();
        m_roadDriven = false;
        for (double& o : m_overlay) o = 0.0;
    }

    bool anyActive() const
    {
        for (const Event& e : m_events) if (e.active) return true;
        return false;
    }

    // Total transients fired since boot (status/UI activity dot).
    uint64_t fireCount() const { return m_fired; }

private:
    struct Event
    {
        bool   active = false;
        double ampPct = 0.0, freqHz = 0.0, durSec = 0.0, tSec = 0.0;
        Route  routes[MAX_ROUTES];
    };

    struct Fx
    {
        double targetLevel = 0.0;   // driven per cycle by the owner
        double level       = 0.0;   // attack/release-smoothed
        double freqHz      = 30.0;
        wavesynth::Oscillator osc;
        uint64_t rng = 0x9E3779B97F4A7C15ull;   // xorshift state
    };

    // Route scale for one contribution: 1 on a torque sink; on a position
    // sink, allowed / asked where asked = (ampPct x gain) / 100 mm.
    double sinkScale(int axis, double askedPctTimesGain, double hz) const
    {
        if (axis < 0 || axis >= MAX_HAPTIC_AXES || m_sinkKind[axis] != SinkKind::Position) return 1.0;
        const double askedMm = askedPctTimesGain / 100.0;
        if (askedMm <= 0.0) return 0.0;
        return std::min(1.0, positionAllowedMm(axis, hz) / askedMm);
    }

    SlipModel* slipFor(FxType t)
    {
        return (t == FxType::Skid) ? &m_slipLat : (t == FxType::Lockup) ? &m_slipLon : nullptr;
    }
    const SlipParams* slipParamsFor(FxType t) const
    {
        return (t == FxType::Skid) ? &m_slipLatParams : (t == FxType::Lockup) ? &m_slipLonParams : nullptr;
    }
    SlipParams* slipParamsFor(FxType t)
    {
        return (t == FxType::Skid) ? &m_slipLatParams : (t == FxType::Lockup) ? &m_slipLonParams : nullptr;
    }

    SinkKind     m_sinkKind[MAX_HAPTIC_AXES] = {};
    double       m_posV[MAX_HAPTIC_AXES] = {}, m_posA[MAX_HAPTIC_AXES] = {}, m_posCap[MAX_HAPTIC_AXES] = {};
    EngineModel  m_engine;
    EngineParams m_engineParams;
    SlipModel    m_slipLat, m_slipLon;
    SlipParams   m_slipLatParams{ 1.0, 25.0, 1.0, 11.0, 7.0 };
    SlipParams   m_slipLonParams{ 1.0,  9.0, 1.0, 10.0, 0.8 };
    RoadModel    m_road;
    RoadParams   m_roadParams;
    bool         m_roadDriven = false;   // a law drove corners this cycle
    EffectParams m_params[EVENT_TYPE_COUNT];
    EffectParams m_fxParams[FX_TYPE_COUNT];
    Event        m_events[MAX_EVENTS];
    Fx           m_fx[FX_TYPE_COUNT];
    double       m_overlay[MAX_HAPTIC_AXES] = {};
    double       m_masterGain = 1.0;
    bool         m_muted      = false;
    uint64_t     m_fired = 0;
};

} // namespace haptics
