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

#include "WaveSynth.h"
#include <algorithm>
#include <cstdint>

namespace haptics {

static constexpr int MAX_EVENTS          = 8;   // concurrent transients
static constexpr int MAX_HAPTIC_AXES     = 10;  // == MAX_DRIVES
static constexpr int MAX_ROUTES          = MAX_HAPTIC_AXES;  // an effect may route to EVERY axis

// Transients: one-shot bursts. GearShift is the sim-driven one (fired on a
// gear-channel change - the thunk of a shift ringing through the chassis).
enum class EventType { DetentClick = 0, GearShift = 1, COUNT };
static constexpr int EVENT_TYPE_COUNT = static_cast<int>(EventType::COUNT);

// Telemetry-driven CONTINUOUS effects (SimHub-ShakeIt class, rendered as
// servo torque). Each is an oscillator whose LEVEL (0..1) is driven per
// cycle by the owner from the NULLCATX channels; the layer adds
// attack/release smoothing so effects fade in and out instead of
// clicking, and the same routing model as the transients. An unbound or
// zero channel drives level 0 = silence; channel staleness (500 ms
// fail-safe) must drive all levels to 0 at the owner.
enum class FxType { RpmVibe = 0, AbsPulse = 1, Lockup = 2, Skid = 3, Road = 4,
                    Limiter = 5, TcPulse = 6, Kerb = 7, COUNT };
static constexpr int FX_TYPE_COUNT = static_cast<int>(FxType::COUNT);

// One destination: explicit axis index and a gain multiplier.
// gain 0 or axis -1 = slot unused.
struct Route
{
    int    axis = -1;
    double gain = 0.0;
};

// Per-event-type tuning (rig config). ampPct 0 = the effect is off.
// Transients use freqHz + durMs; continuous effects use freqHz as their
// carrier (durMs ignored) except RpmVibe, which uses `order` (carrier =
// rpm/60 x order, the engine's firing frequency) and ignores freqHz.
// jitter (0..1) roughens the carrier per cycle - skid and road feel like
// texture, not a tone.
//
// Routing is one flat model for every effect: an explicit per-axis gain
// table, empty by default (an unconfigured effect reaches nothing). No
// implicit destinations of any kind.
struct EffectParams
{
    double ampPct = 0.0;    // % of rated torque at full scale
    double freqHz = 90.0;   // burst/texture carrier
    double durMs  = 18.0;   // burst length (transients only)
    double order  = 2.0;    // RpmVibe only: carrier = rpm/60 x order
    double jitter = 0.0;    // 0..1 carrier roughness (Skid/Road)
    Route  routes[MAX_ROUTES] = {};
};

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
                if (r.axis >= 0) m_overlay[r.axis] += v * r.gain;
        }

        // Continuous effects: attack/release-smoothed level on a free-running
        // oscillator (phase-continuous through frequency changes), optional
        // per-cycle carrier jitter for texture-class effects.
        for (int i = 0; i < FX_TYPE_COUNT; ++i)
        {
            Fx& f = m_fx[i];
            const EffectParams& p = m_fxParams[i];
            // Level ramp: ~50 ms attack, ~120 ms release.
            const double rate = (f.targetLevel > f.level) ? dtSec / 0.050 : dtSec / 0.120;
            f.level += std::max(-rate, std::min(rate, f.targetLevel - f.level));
            if (p.ampPct <= 0.0 || f.level < 1e-4) continue;

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
                    m_overlay[r.axis] += v * r.gain;
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

    // ---- continuous effects -------------------------------------------------

    void configureFx(FxType t, const EffectParams& p)
    {
        m_fxParams[static_cast<int>(t)] = p;
    }

    const EffectParams& fxParams(FxType t) const
    {
        return m_fxParams[static_cast<int>(t)];
    }

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

    // Kill every active transient instantly (e-stop, park, loop stop).
    void clearAll()
    {
        for (Event& e : m_events) e.active = false;
        for (Fx& f : m_fx) { f.targetLevel = 0.0; f.level = 0.0; f.osc.reset(); }
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
