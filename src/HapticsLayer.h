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
// carrier (durMs ignored). RpmVibe (the engine) uses freqHz as the THUMP
// carrier of each firing (default 30 Hz, 0 = default), cylinders for the
// firing density, and jitter as the idle lope amount.
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
    double order  = 2.0;    // ENGINE: buzz carrier order (x crank rate), 0 = auto from max rpm; unused elsewhere
    double jitter = 0.0;    // 0..1 carrier roughness; ENGINE: idle-lope amount
    double cylinders = 4.0; // ENGINE only: firing rate = rpm/60 x cylinders/2
    double rock  = 1.0;     // ENGINE only: crank-rate rock component gain 0..1
    double thump = 1.0;     // ENGINE only: firing thump component gain 0..1
    double buzz  = 1.0;     // ENGINE only: rpm-following vibration gain 0..1 (carrier = crank x order)
    double litres = 2.0;    // ENGINE only: total displacement; per-cylinder size sets impulse weight
    double layout = 0.0;    // ENGINE only: 0 inline, 1 V, 2 flat/boxer, 3 Wankel (cylinders = rotors)
    double maxRpm = 0.0;    // ENGINE only: redline; 0 = learn it (peak hold, limiter snap)
    double limHit = 1.0;    // ENGINE only: limiter hammer strength 0..2 (each return hit, 1 = full load)
    double limHz  = 12.0;   // ENGINE only: limiter cut rate Hz (6..25)
    double limJit = 0.0;    // ENGINE only: limiter cut-timing irregularity 0..1
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

            // The engine slot runs the pulse-train synth, not the oscillator.
            if (i == static_cast<int>(FxType::RpmVibe))
            {
                const double v = stepEngine(f, p, dtSec);
                if (v != 0.0)
                    for (const Route& r : p.routes)
                        if (r.axis >= 0 && r.axis < MAX_HAPTIC_AXES && r.gain > 0.0)
                            m_overlay[r.axis] += v * r.gain;
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
        m_eng.fireHz   = (fireHz > 0.0) ? fireHz : 0.0;
        m_eng.load     = (load01 < 0.0) ? 0.0 : (load01 > 1.0 ? 1.0 : load01);
        m_eng.limiter  = limiterOn;
    }

    // Kill every active transient instantly (e-stop, park, loop stop).
    void clearAll()
    {
        for (Event& e : m_events) e.active = false;
        for (Fx& f : m_fx) { f.targetLevel = 0.0; f.level = 0.0; f.osc.reset(); }
        for (double& t : m_eng.pulseT) t = 1e9;
        m_eng.firePhase = 0.0; m_eng.crankPhase = 0.0; m_eng.rockPhase = 0.0;
        m_eng.cutPhase = 0.0; m_eng.wasCut = false; m_eng.buzzPhase = 0.0; m_eng.beatPhase = 0.0; m_eng.revScale = 1.0;
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

    // ---- engine model state ----------------------------------------------
    // What you feel from an engine at idle is not the firing frequency (a
    // V8 at 800 rpm fires at 53 Hz, a buzz); it is the block rocking on its
    // mounts at crank rate (13 Hz) with the firing as texture on top, and a
    // big cam adds a half-order lope. So: firings are thumps with a FIXED
    // low carrier (EffectParams.freqHz, default 30 Hz) that overlap in a
    // small pool; a crank-rate rock rides underneath, full at idle and gone
    // by ~2500 rpm; the lope amount (EffectParams.jitter) is a per-revolution
    // random unevenness plus half-order content. Cylinders only set firing
    // density, which is the right physics. The limiter is a CUT GATE: the
    // ECU drops whole bursts of firings (~12 Hz, half on / half off) and the
    // engine comes back at full load, which is the bounce; random single
    // misfires only read as "a different rpm".
    static constexpr int ENGINE_PULSES = 4;
    struct Engine
    {
        double fireHz = 0.0, load = 0.5;
        bool   limiter = false;
        double firePhase  = 0.0;     // 0..1 per firing
        double crankPhase = 0.0;     // 0..1 per revolution (per-rev lope draw)
        double rockPhase  = 0.0;     // 0..1 per TWO revolutions (half-order)
        double revScale   = 1.0;     // this revolution's unevenness
        double cutPhase   = 0.0;     // limiter gate 0..1
        double cutRate    = 12.0;    // this cut cycle's rate (jittered per cycle)
        bool   wasCut     = false;   // previous cycle inside a cut (return-hit edge)
        double buzzPhase  = 0.0;     // rpm-following vibration carrier 0..1
        double beatPhase  = 0.0;     // rotary idle beat 0..1
        double learnedMax = 7000.0;  // redline learned from the stream (peak hold; limiter hit snaps it)
        double pulseDur   = 0.033;
        double pulseT[ENGINE_PULSES]   = { 1e9, 1e9, 1e9, 1e9 };
        double pulseAmp[ENGINE_PULSES] = { 0.0, 0.0, 0.0, 0.0 };
    };
    Engine       m_eng;

    // One cycle of the engine model; returns the overlay value (% of rated,
    // before routing gain). f.level is the attack/release-smoothed drive
    // level, p the RpmVibe params.
    double stepEngine(Fx& f, const EffectParams& p, double dtSec)
    {
        Engine& E = m_eng;
        const double thumpHz = (p.freqHz >= 5.0) ? p.freqHz : 30.0;
        E.pulseDur = 1.0 / thumpHz;                       // one carrier cycle
        const double cyl     = std::max(1.0, p.cylinders);
        const int    layout  = static_cast<int>(p.layout + 0.5);
        const bool   rotary  = (layout == 3);
        // Four-stroke: each cylinder fires every 2 revs. Wankel: each rotor
        // fires once per eccentric-shaft rev (3 faces per rotor rev, shaft
        // turns 3x), so "cylinders" = rotors and density is 1 per rotor.
        const double crankHz = rotary ? E.fireHz / cyl : E.fireHz / (cyl / 2.0);

        // Engine size, two physical factors from litres + cylinders:
        //  heavy   - displacement PER CYLINDER sets each firing impulse
        //            (0.5 L/cyl = 1.0; a 1.0 L triple 0.67, a 6.5 L V8 1.6,
        //            a 1.6 L V6 0.55, a 6.0 L V12 1.0). Scales thumps and idle
        //            lumpiness. A sub-litre engine is light and busy, a big
        //            block hits hard, whatever the cylinder count.
        //  balance - inherent shake by cylinder count: a triple has a strong
        //            first-order rocking couple, a four shakes, a six is
        //            smooth, a V12 is turbine-smooth. Scales the rock only.
        //            Layout scales it: a V engine is mechanically smoother than
        //            an inline of the same count, a flat/boxer cancels most of
        //            its primary shake, a Wankel has no reciprocating mass at all.
        const double perCyl  = std::max(0.05, p.litres) / cyl;
        const double heavy   = std::max(0.4, std::min(1.6, perCyl / 0.5));
        const double layoutK = (layout == 1) ? 0.85 : (layout == 2) ? 0.7 : 1.0;
        const double balance = rotary ? 0.25
                             : layoutK * std::max(0.3, std::min(1.3, 2.3 / std::sqrt(cyl)));   // inline 3:1.3 4:1.15 6:0.94 8:0.81 12:0.66 16:0.58

        // Redline: set on the tile, or LEARNED from the stream - peak hold
        // of the rpm seen (seeded at 7000 so nothing is wrong before the
        // first full-throttle run) and snapped exactly the first time the
        // limiter flag comes in (the plugin raises it at 98.5% of the car's
        // max), down as well as up, so a car change corrects itself on its
        // first limiter hit. The top-end laws below scale to this, so a
        // 6000 rpm V8 and a 16000 rpm V12 both use the whole effect.
        const double rpmNow = crankHz * 60.0;
        if (E.fireHz >= 0.5)
        {
            if (rpmNow > E.learnedMax) E.learnedMax = rpmNow;
            if (E.limiter && rpmNow > 1000.0) E.learnedMax = rpmNow / 0.985;
        }
        const double maxRpm = (p.maxRpm > 0.0) ? p.maxRpm : E.learnedMax;
        const double x      = std::max(0.0, std::min(1.0, rpmNow / std::max(1000.0, maxRpm)));   // 0..1 of redline

        // Rotary beat: the rotors' combustion is uneven at idle (port
        // overlap, the classic rough "brap"), felt as a slow 2-3 Hz swell
        // and fade of the firing pulses that smooths out by ~2500 rpm.
        double beat = 1.0;
        if (rotary && E.fireHz >= 0.5)
        {
            E.beatPhase += 2.5 * dtSec;
            if (E.beatPhase >= 1.0) E.beatPhase -= (double)(int)E.beatPhase;
            const double depth = 0.5 * std::max(0.0, std::min(1.0, (42.0 - crankHz) / 28.0));
            beat = 1.0 + depth * std::sin(2.0 * wavesynth::kPi * E.beatPhase);
        }

        // Limiter cut gate.
        bool cut = false;
        if (E.limiter && E.fireHz >= 0.5)
        {
            // Rate per cut cycle: limHz, jittered by limJit (+-40% at 1) so a
            // rough limiter stumbles irregularly instead of a clean metronome.
            E.cutPhase += E.cutRate * dtSec;
            if (E.cutPhase >= 1.0)
            {
                E.cutPhase -= (double)(int)E.cutPhase;
                const double base = std::max(4.0, std::min(30.0, p.limHz > 0.0 ? p.limHz : 12.0));
                f.rng ^= f.rng << 13; f.rng ^= f.rng >> 7; f.rng ^= f.rng << 17;
                const double r = (double)(f.rng & 0xFFFF) / 65535.0;
                E.cutRate = base * (1.0 + 0.8 * std::max(0.0, std::min(1.0, p.limJit)) * (r - 0.5));
            }
            cut = (E.cutPhase < 0.5);
        }
        else { E.cutPhase = 0.0; E.cutRate = std::max(4.0, std::min(30.0, p.limHz > 0.0 ? p.limHz : 12.0)); }
        // The return from a cut is its own event: the engine catches again
        // with a lurch. It gets a hit at the thump carrier whatever the rpm
        // (the per-firing thumps have faded out up here), scaled by limHit,
        // so "limiter x" does what it says where the limiter actually lives.
        const bool returnHit = E.wasCut && !cut;
        E.wasCut = cut;

        if (E.fireHz >= 0.5)
        {
            E.crankPhase += crankHz * dtSec;
            if (E.crankPhase >= 1.0)
            {
                E.crankPhase -= (double)(int)E.crankPhase;
                f.rng ^= f.rng << 13; f.rng ^= f.rng >> 7; f.rng ^= f.rng << 17;
                const double r = (double)(f.rng & 0xFFFF) / 65535.0;
                E.revScale = 1.0 + p.jitter * (r - 0.5) * 2.0;   // +-lope
            }
            E.rockPhase += 0.5 * crankHz * dtSec;
            if (E.rockPhase >= 1.0) E.rockPhase -= (double)(int)E.rockPhase;

            E.firePhase += E.fireHz * dtSec;
            if (E.firePhase >= 1.0)
            {
                E.firePhase -= (double)(int)E.firePhase;
                if (!cut)
                {
                    // Free slot, else the one closest to finishing.
                    int slot = 0;
                    for (int s = 1; s < ENGINE_PULSES; ++s)
                        if (E.pulseT[s] > E.pulseT[slot]) slot = s;
                    // Under the limiter the engine comes back hard: full load.
                    const double hit = E.limiter ? std::max(0.0, std::min(2.0, p.limHit)) : (0.35 + 0.65 * E.load);
                    // Thumps overlap once firings come faster than one
                    // carrier cycle; normalise so the merged buzz keeps the
                    // configured amplitude instead of stacking past it.
                    const double overlap = E.fireHz * E.pulseDur;
                    const double norm    = 1.0 / std::max(1.0, std::sqrt(overlap));
                    // Once firings can no longer be resolved (more than ~4 per
                    // thump cycle) they are not felt as thumps any more: fade
                    // them out and let the buzz carry the engine.
                    const double resolve = std::max(0.0, std::min(1.0, 1.0 - (overlap - 4.0) / 8.0));
                    E.pulseAmp[slot] = p.ampPct * f.level * 0.6 * p.thump * heavy * beat * hit * norm * resolve * E.revScale;
                    E.pulseT[slot]   = 0.0;
                }
            }
        }

        if (returnHit && p.thump > 0.0)
        {
            int slot = 0;
            for (int s = 1; s < ENGINE_PULSES; ++s)
                if (E.pulseT[s] > E.pulseT[slot]) slot = s;
            E.pulseAmp[slot] = p.ampPct * f.level * 0.6 * p.thump * heavy
                             * std::max(0.0, std::min(2.0, p.limHit));
            E.pulseT[slot]   = 0.0;
        }

        // Sum the thumps in flight (they may overlap at high firing rates).
        double v = 0.0;
        for (int s = 0; s < ENGINE_PULSES; ++s)
        {
            if (E.pulseT[s] >= E.pulseDur) continue;
            const double env = wavesynth::envelope(E.pulseT[s], E.pulseDur, E.pulseDur * 0.30);
            v += E.pulseAmp[s] * env
               * std::sin(2.0 * wavesynth::kPi * (E.pulseT[s] / E.pulseDur));
            E.pulseT[s] += dtSec;
        }

        // Crank-rate rock: full below ~840 rpm, gone above ~2500 rpm, and
        // it sags during a limiter cut like the real thing.
        if (E.fireHz >= 0.5 && !cut)
        {
            const double fade = std::max(0.0, std::min(1.0, (42.0 - crankHz) / 28.0));
            if (fade > 0.0)
            {
                const double ph   = 2.0 * wavesynth::kPi * E.rockPhase;   // one cycle = 2 revs
                const double rock = std::sin(2.0 * ph)                      // crank rate
                                  + 0.6 * p.jitter * std::sin(ph);          // half-order lope
                // A loaded block rocks harder than a coasting one.
                v += p.ampPct * f.level * 0.4 * p.rock * heavy * balance * fade * (0.7 + 0.3 * E.load) * E.revScale * rock;
            }
        }

        // Buzz: the rest of the rev range. Above idle the block stops
        // rocking and what you feel is the firing-order vibration through
        // the structure, pitch rising with rpm. At 6000 rpm on a V8 that is
        // 400 Hz, beyond any actuator here, so the TRUE relationship (pitch
        // proportional to rpm) is kept at a lower order the actuator can
        // carry: carrier = crank rate x order (default 1: 13 Hz at 800 rpm
        // to ~117 Hz at 7000), clamped to 150 Hz. Level rises from ~1200 rpm
        // to full at ~3600 and with throttle, taking over as the rock fades,
        // and keeps growing to the limiter; it sags in every limiter cut.
        if (E.fireHz >= 0.5 && p.buzz > 0.0)
        {
            // Order: set on the tile, or AUTO so the redline lands at the top
            // of the band (120 Hz): pitch stays proportional to rpm and the
            // whole rev range fits whatever the engine revs to (a 7000 rpm
            // V8 gets ~1.0, a 16000 rpm V12 ~0.45, a 5500 rpm diesel ~1.3).
            const double order = (p.order > 0.0) ? p.order
                               : std::max(0.25, std::min(4.0, 120.0 / (maxRpm / 60.0)));
            const double carrier = std::max(8.0, std::min(150.0, crankHz * order));
            E.buzzPhase += carrier * dtSec;
            if (E.buzzPhase >= 1.0) E.buzzPhase -= (double)(int)E.buzzPhase;
            // Level: in from 15% of redline, full body by 55%, and still
            // building to the top so the last part of the range is the
            // angriest, as it is.
            const double rise = std::max(0.0, std::min(1.0, (x - 0.15) / 0.40));
            if (rise > 0.0 && !cut)
            {
                const double lvl = rise * (0.75 + 0.25 * x) * (0.5 + 0.5 * E.load)
                                 * (E.limiter ? std::max(0.0, std::min(2.0, p.limHit)) : 1.0);
                v += p.ampPct * f.level * 0.5 * p.buzz * (0.6 + 0.4 * heavy) * lvl
                   * std::sin(2.0 * wavesynth::kPi * E.buzzPhase);
            }
        }
        // The engine never exceeds its own amplitude; the axis clamp is the
        // guard rail above this, not the shaping.
        // On the limiter the hit may run up to limHit x, so the ceiling follows.
        const double cap = p.ampPct * f.level * (E.limiter ? std::max(1.0, std::min(2.0, p.limHit)) : 1.0);
        return std::max(-cap, std::min(cap, v));
    }
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
