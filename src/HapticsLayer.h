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
#include <cstdint>

namespace haptics {

static constexpr int MAX_EVENTS          = 8;   // concurrent transients
static constexpr int MAX_ROUTES          = 4;   // destinations per event type
static constexpr int MAX_HAPTIC_AXES     = 10;  // == MAX_DRIVES
static constexpr int ROUTE_SOURCE_AXIS   = -2;  // "the axis that fired it"

enum class EventType { DetentClick = 0, COUNT };
static constexpr int EVENT_TYPE_COUNT = static_cast<int>(EventType::COUNT);

// One destination: axis index (or ROUTE_SOURCE_AXIS) and a gain multiplier.
// gain 0 or axis -1 = slot unused.
struct Route
{
    int    axis = -1;
    double gain = 0.0;
};

// Per-event-type tuning (rig config). ampPct 0 = the effect is off.
struct EffectParams
{
    double ampPct = 0.0;    // % of rated torque at full scale
    double freqHz = 90.0;   // burst carrier
    double durMs  = 18.0;   // burst length
    Route  routes[MAX_ROUTES] = { { ROUTE_SOURCE_AXIS, 1.0 } };
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

    // Fire one transient. sourceAxis resolves ROUTE_SOURCE_AXIS routes;
    // scale in [0,1] multiplies the configured amplitude (impact velocity).
    // Inert when the effect's ampPct is 0 or every route is dead. When the
    // pool is full the event is dropped (a missed click is harmless; a
    // stalled RT loop is not).
    void fire(EventType t, int sourceAxis, double scale = 1.0)
    {
        const EffectParams& p = m_params[static_cast<int>(t)];
        if (p.ampPct <= 0.0 || p.durMs <= 0.0) return;
        if (scale <= 0.0) return;
        if (scale > 1.0) scale = 1.0;

        Event* e = nullptr;
        for (Event& c : m_events) if (!c.active) { e = &c; break; }
        if (!e) return;                         // pool full: drop, never block

        // Snapshot params + resolved routes at fire time.
        e->ampPct = p.ampPct * scale;
        e->freqHz = (p.freqHz > 0.0) ? p.freqHz : 90.0;
        e->durSec = p.durMs / 1000.0;
        e->tSec   = 0.0;
        bool anyRoute = false;
        for (int i = 0; i < MAX_ROUTES; ++i)
        {
            int axis = p.routes[i].axis;
            if (axis == ROUTE_SOURCE_AXIS) axis = sourceAxis;
            const bool ok = axis >= 0 && axis < MAX_HAPTIC_AXES
                            && p.routes[i].gain > 0.0;
            e->routes[i].axis = ok ? axis : -1;
            e->routes[i].gain = ok ? p.routes[i].gain : 0.0;
            anyRoute |= ok;
        }
        if (!anyRoute) return;                  // fully unrouted: never activate
        e->active = true;
        ++m_fired;
    }

    // Advance every active event by one cycle and refresh the per-axis
    // overlay sums. Call ONCE per control cycle, before overlayFor().
    void step(double dtSec)
    {
        for (double& o : m_overlay) o = 0.0;
        if (dtSec <= 0.0) return;
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
    }

    // Torque overlay (% of rated) for one axis this cycle. The caller adds
    // it to the axis command and clamps the SUM inside the axis limits.
    double overlayFor(int axis) const
    {
        return (axis >= 0 && axis < MAX_HAPTIC_AXES) ? m_overlay[axis] : 0.0;
    }

    // Kill every active transient instantly (e-stop, park, loop stop).
    void clearAll()
    {
        for (Event& e : m_events) e.active = false;
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

    EffectParams m_params[EVENT_TYPE_COUNT];
    Event        m_events[MAX_EVENTS];
    double       m_overlay[MAX_HAPTIC_AXES] = {};
    uint64_t     m_fired = 0;
};

} // namespace haptics
