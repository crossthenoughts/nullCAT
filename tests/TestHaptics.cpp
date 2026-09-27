// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestHaptics.cpp - unit tests for the haptic transient layer and the
// force model's detent-capture trigger.
//
// Pure-logic suite (no drives, no Qt Test). Pins: fire/step/overlay
// lifecycle, envelope boundedness (zero start and end, bounded middle),
// routing incl. ROUTE_SOURCE_AXIS resolution and gains, amp-0 inertness,
// pool exhaustion behaviour (drop, never block), clearAll, and the
// edge-triggered detent capture in DeviceForceModel (fires once on entry,
// never on dwell, re-fires on re-entry, never on the seed step, and
// works with an empty detent CURVE - a pure-haptic gate).
// ============================================================

#include "../src/HapticsLayer.h"
#include "../src/DeviceForceModel.h"
#include <cstdio>
#include <cmath>

static int g_pass = 0, g_fail = 0;
static void CHECK(bool ok, const char* what)
{
    if (ok) { ++g_pass; }
    else    { ++g_fail; std::printf("FAIL: %s\n", what); }
}

using haptics::Layer;
using haptics::EffectParams;
using haptics::EventType;
using haptics::ROUTE_SOURCE_AXIS;

static EffectParams click(double amp = 10.0, double freq = 100.0, double ms = 20.0)
{
    EffectParams p;
    p.ampPct = amp; p.freqHz = freq; p.durMs = ms;
    return p;   // default route: { ROUTE_SOURCE_AXIS, 1.0 }
}

int main()
{
    const double DT = 0.0005;   // 2 kHz

    // ================= lifecycle + envelope =================
    {
        Layer L;
        L.configure(EventType::DetentClick, click());
        CHECK(!L.anyActive(), "fresh layer: nothing active");
        L.fire(EventType::DetentClick, 3);
        CHECK(L.anyActive(), "fire activates an event");
        CHECK(L.fireCount() == 1, "fireCount counts");

        // First step: envelope is ~0 at t->0 (raised cosine), never a snap.
        L.step(DT);
        CHECK(std::fabs(L.overlayFor(3)) < 1.0, "no torque snap on the first cycle");

        // Mid-burst: overlay must actually appear, bounded by amp.
        double peak = 0.0;
        for (int i = 0; i < 40; ++i) { L.step(DT); peak = std::max(peak, std::fabs(L.overlayFor(3))); }
        CHECK(peak > 1.0, "burst produces torque mid-envelope");
        CHECK(peak <= 10.0 + 1e-9, "burst never exceeds configured amplitude");

        // After duration: inactive, overlay exactly zero.
        for (int i = 0; i < 40; ++i) L.step(DT);
        CHECK(!L.anyActive(), "event expires after durMs");
        CHECK(L.overlayFor(3) == 0.0, "expired event contributes exactly zero");
    }

    // ================= routing =================
    {
        Layer L;
        EffectParams p = click(20.0);
        p.routes[0] = { ROUTE_SOURCE_AXIS, 1.0 };   // own axis, full
        p.routes[1] = { 5, 0.5 };                   // belt at half gain
        L.configure(EventType::DetentClick, p);
        L.fire(EventType::DetentClick, 2);
        double own = 0.0, belt = 0.0;
        for (int i = 0; i < 30; ++i)
        {
            L.step(DT);
            own  = std::max(own,  std::fabs(L.overlayFor(2)));
            belt = std::max(belt, std::fabs(L.overlayFor(5)));
        }
        CHECK(own > 1.0,                        "ROUTE_SOURCE_AXIS resolves to the firing axis");
        CHECK(belt > 0.5,                       "explicit route receives the burst");
        CHECK(std::fabs(belt - own * 0.5) < 0.2, "route gain scales the burst");
        CHECK(L.overlayFor(0) == 0.0,           "unrouted axis stays silent");
    }

    // ================= inertness + scale + bad input =================
    {
        Layer L;                                    // default config: amp 0
        L.fire(EventType::DetentClick, 1);
        CHECK(!L.anyActive(), "amp 0 (the shipped default) is fully inert");

        L.configure(EventType::DetentClick, click());
        L.fire(EventType::DetentClick, 1, 0.0);
        CHECK(!L.anyActive(), "scale 0 fires nothing");
        L.fire(EventType::DetentClick, 1, 7.0);     // clamps to 1
        double peak = 0.0;
        for (int i = 0; i < 40; ++i) { L.step(DT); peak = std::max(peak, std::fabs(L.overlayFor(1))); }
        CHECK(peak <= 10.0 + 1e-9, "scale clamps at 1.0");

        EffectParams unrouted = click();
        unrouted.routes[0] = { -1, 0.0 };
        Layer L2; L2.configure(EventType::DetentClick, unrouted);
        L2.fire(EventType::DetentClick, 1);
        CHECK(!L2.anyActive(), "fully unrouted event never activates");

        CHECK(L.overlayFor(-1) == 0.0 && L.overlayFor(99) == 0.0,
              "out-of-range axis reads zero, no crash");
    }

    // ================= pool exhaustion + clearAll =================
    {
        Layer L;
        L.configure(EventType::DetentClick, click(10.0, 100.0, 50.0));
        for (int i = 0; i < 20; ++i) L.fire(EventType::DetentClick, 1);
        CHECK(L.fireCount() == haptics::MAX_EVENTS,
              "pool full: extra fires drop (never block, never overwrite)");
        L.clearAll();
        CHECK(!L.anyActive(), "clearAll kills every event");
        L.step(DT);
        CHECK(L.overlayFor(1) == 0.0, "cleared pool contributes zero");
    }

    // ================= model trigger: detent capture =================
    {
        DeviceParams p;
        p.springCurve  = { {0.0, 0.0}, {0.07, 50.0} };
        p.detentCurve  = { {-0.02, 60.0}, {0.0, 0.0}, {0.02, -60.0} };
        p.detents      = { 0.05 };
        p.stopMinRev   = -0.07;
        p.stopMaxRev   =  0.07;
        p.velLpfHz     = 0.0;
        p.slewPctPerSec = 1e9;
        p.maxForcePct  = 300.0;

        DeviceForceModel m; m.configure(p, 0.0005);
        DeviceStateMods inert;
        HapticTriggers t;

        // Seed step INSIDE the gate: must not fire (engaged in-gate).
        m.reset(0.05);
        t = {}; m.step(0.05, inert, &t);
        CHECK(!t.detentEnter, "seed step never fires, even inside the gate");

        // Leave, then re-enter: exactly one edge.
        t = {}; m.step(0.02, inert, &t);
        CHECK(!t.detentEnter, "outside the capture radius: no trigger");
        t = {}; m.step(0.046, inert, &t);
        CHECK(t.detentEnter, "entering the capture radius fires");
        CHECK(t.detentVel > 0.99, "fast entry saturates the velocity scale");
        t = {}; m.step(0.05, inert, &t);
        CHECK(!t.detentEnter, "dwelling in the gate never re-fires");

        // Null-sink cycles still advance the edge state: step out with no
        // sink, back in with one - the entry must still read as an edge.
        m.step(0.0, inert);                       // leave, nobody listening
        t = {}; m.step(0.05, inert, &t);
        CHECK(t.detentEnter, "edge state advances through null-sink cycles");

        // Slow entry scales down.
        m.step(0.02, inert);
        t = {};
        for (double x = 0.02; !t.detentEnter && x < 0.06; x += 0.0002)
            m.step(x, inert, &t);
        CHECK(t.detentEnter && t.detentVel < 0.5, "gentle entry gets a gentle scale");

        // Pure-haptic gate: empty detent CURVE still triggers.
        DeviceParams ph = p; ph.detentCurve.clear();
        DeviceForceModel mh; mh.configure(ph, 0.0005);
        mh.reset(0.0); mh.step(0.0, inert);
        t = {}; mh.step(0.05, inert, &t);
        CHECK(t.detentEnter, "gates with no holding force still click");

        // No detents at all: never triggers.
        DeviceParams pn = p; pn.detents.clear();
        DeviceForceModel mn; mn.configure(pn, 0.0005);
        mn.reset(0.0); mn.step(0.0, inert);
        t = {}; mn.step(0.05, inert, &t);
        CHECK(!t.detentEnter, "no detents: no trigger, ever");
    }

    std::printf("\nTestHaptics: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
