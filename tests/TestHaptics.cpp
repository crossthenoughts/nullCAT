// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestHaptics.cpp - unit tests for the haptic transient layer and the
// force model's detent-capture trigger.
//
// Pure-logic suite (no drives, no Qt Test). Pins: fire/step/overlay
// lifecycle, envelope boundedness (zero start and end, bounded middle),
// routing gains across explicit axes, amp-0 inertness,
// pool exhaustion behaviour (drop, never block), clearAll, and the
// edge-triggered detent capture in DeviceForceModel (fires once on entry,
// never on dwell, re-fires on re-entry, never on the seed step, and
// works with an empty detent CURVE - a pure-haptic gate).
// ============================================================

#include "../src/HapticsLayer.h"
#include "../src/DeviceForceModel.h"
#include <cstdio>
#include <cmath>
#include <utility>
#include <vector>

static int g_pass = 0, g_fail = 0;
static void CHECK(bool ok, const char* what)
{
    if (ok) { ++g_pass; }
    else    { ++g_fail; std::printf("FAIL: %s\n", what); }
}

using haptics::Layer;
using haptics::EffectParams;
using haptics::EventType;

static EffectParams click(double amp = 10.0, double freq = 100.0, double ms = 20.0)
{
    EffectParams p;
    p.ampPct = amp; p.freqHz = freq; p.durMs = ms;
    p.routes[0] = { 3, 1.0 };   // struct default is EMPTY routes: route to axis 3
    return p;
}

int main()
{
    const double DT = 0.0005;   // 2 kHz

    // ================= lifecycle + envelope =================
    {
        Layer L;
        L.configure(EventType::DetentClick, click());
        CHECK(!L.anyActive(), "fresh layer: nothing active");
        L.fire(EventType::DetentClick);
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
        p.routes[0] = { 2, 1.0 };                   // full gain
        p.routes[1] = { 5, 0.5 };                   // belt at half gain
        L.configure(EventType::DetentClick, p);
        L.fire(EventType::DetentClick);
        double own = 0.0, belt = 0.0;
        for (int i = 0; i < 30; ++i)
        {
            L.step(DT);
            own  = std::max(own,  std::fabs(L.overlayFor(2)));
            belt = std::max(belt, std::fabs(L.overlayFor(5)));
        }
        CHECK(own > 1.0,                        "full-gain route receives the burst");
        CHECK(belt > 0.5,                       "explicit route receives the burst");
        CHECK(std::fabs(belt - own * 0.5) < 0.2, "route gain scales the burst");
        CHECK(L.overlayFor(0) == 0.0,           "unrouted axis stays silent");
    }

    // ================= inertness + scale + bad input =================
    {
        Layer L;                                    // default config: amp 0
        L.fire(EventType::DetentClick);
        CHECK(!L.anyActive(), "amp 0 (the shipped default) is fully inert");

        L.configure(EventType::DetentClick, click());
        L.fire(EventType::DetentClick, 0.0);
        CHECK(!L.anyActive(), "scale 0 fires nothing");
        L.fire(EventType::DetentClick, 7.0);        // clamps to 1
        double peak = 0.0;
        for (int i = 0; i < 40; ++i) { L.step(DT); peak = std::max(peak, std::fabs(L.overlayFor(3))); }
        CHECK(peak > 1.0 && peak <= 10.0 + 1e-9, "scale clamps at 1.0");

        EffectParams unrouted = click();
        unrouted.routes[0] = { -1, 0.0 };
        Layer L2; L2.configure(EventType::DetentClick, unrouted);
        L2.fire(EventType::DetentClick);
        CHECK(!L2.anyActive(), "fully unrouted event never activates");

        CHECK(L.overlayFor(-1) == 0.0 && L.overlayFor(99) == 0.0,
              "out-of-range axis reads zero, no crash");
    }

    // ================= pool exhaustion + clearAll =================
    {
        Layer L;
        L.configure(EventType::DetentClick, click(10.0, 100.0, 50.0));
        for (int i = 0; i < 20; ++i) L.fire(EventType::DetentClick);
        CHECK(L.fireCount() == haptics::MAX_EVENTS,
              "pool full: extra fires drop (never block, never overwrite)");
        L.clearAll();
        CHECK(!L.anyActive(), "clearAll kills every event");
        L.step(DT);
        CHECK(L.overlayFor(1) == 0.0, "cleared pool contributes zero");
    }

    // ================= continuous effects =================
    {
        using haptics::FxType;
        Layer L;
        EffectParams p;
        p.ampPct = 10.0; p.freqHz = 35.0; p.jitter = 0.0;
        p.routes[0] = { 4, 1.0 };                    // explicit axis
        L.configureFx(FxType::Skid, p);

        // Undriven: silent.
        L.step(DT);
        CHECK(L.overlayFor(4) == 0.0, "fx undriven: silent");

        // Driven at full level: ramps in (attack), reaches near amp.
        double peak = 0.0;
        for (int i = 0; i < 400; ++i)                // 200 ms at 2 kHz
        { L.driveFx(FxType::Skid, 1.0, 0.0); L.step(DT); peak = std::max(peak, std::fabs(L.overlayFor(4))); }
        CHECK(peak > 8.0, "fx reaches near full amplitude when driven");
        CHECK(peak <= 10.0 + 1e-9, "fx never exceeds configured amplitude");

        // First cycles after drive start must be small (attack ramp).
        Layer L2; L2.configureFx(FxType::Skid, p);
        L2.driveFx(FxType::Skid, 1.0, 0.0); L2.step(DT);
        CHECK(std::fabs(L2.overlayFor(4)) < 1.0, "fx attack ramps, never snaps on");

        // Stop driving: release ramp decays to silence (fail-safe fade).
        for (int i = 0; i < 800; ++i) { L.driveFx(FxType::Skid, 0.0, 0.0); L.step(DT); }
        CHECK(L.overlayFor(4) == 0.0 || std::fabs(L.overlayFor(4)) < 0.05,
              "fx released: fades to silence");

        // Half level scales.
        Layer L3; L3.configureFx(FxType::Skid, p);
        double half = 0.0;
        for (int i = 0; i < 400; ++i)
        { L3.driveFx(FxType::Skid, 0.5, 0.0); L3.step(DT); half = std::max(half, std::fabs(L3.overlayFor(4))); }
        CHECK(half > 3.0 && half < 6.0, "fx level scales amplitude");

        // Amp 0 config: driven or not, silent.
        Layer L4;                                     // defaults: amp 0
        L4.driveFx(FxType::Road, 1.0, 0.0); L4.step(DT);
        CHECK(!std::fabs(L4.overlayFor(4)), "fx amp 0 (shipped default) is inert");

        // No usable carrier (freq 0 config, none driven): silence, not DC.
        EffectParams pz = p; pz.freqHz = 0.0;
        Layer L5; L5.configureFx(FxType::Skid, pz);
        for (int i = 0; i < 200; ++i) { L5.driveFx(FxType::Skid, 1.0, 0.0); L5.step(DT); }
        CHECK(L5.overlayFor(4) == 0.0, "fx with no carrier is silent, never DC");

        // Driven frequency is honored (Lockup slot; the RpmVibe slot is
        // the pulse-train engine and ignores a driven carrier by design).
        EffectParams pr; pr.ampPct = 10.0; pr.freqHz = 0.0; pr.routes[0] = { 2, 1.0 };
        Layer L6; L6.configureFx(FxType::Lockup, pr);
        double vibe = 0.0;
        for (int i = 0; i < 400; ++i)
        { L6.driveFx(FxType::Lockup, 1.0, 120.0); L6.step(DT); vibe = std::max(vibe, std::fabs(L6.overlayFor(2))); }
        CHECK(vibe > 8.0, "driven carrier frequency is honored");
        L6.clearAll(); L6.step(DT);
        CHECK(L6.overlayFor(2) == 0.0, "clearAll silences continuous effects instantly");

        // Jitter keeps the output bounded.
        EffectParams pj = p; pj.jitter = 1.0;
        Layer L7; L7.configureFx(FxType::Road, pj);
        double jp = 0.0;
        for (int i = 0; i < 400; ++i)
        { L7.driveFx(FxType::Road, 1.0, 0.0); L7.step(DT); jp = std::max(jp, std::fabs(L7.overlayFor(4))); }
        CHECK(jp > 5.0 && jp <= 10.0 + 1e-9, "full jitter stays bounded by amp");
    }

    // ================= pulse-train engine =================
    {
        using haptics::FxType;
        EffectParams p;
        p.ampPct = 10.0; p.jitter = 0.0; p.cylinders = 8.0;
        p.routes[0] = { 6, 1.0 };

        // Run one second of the engine at a firing rate (V8: fireHz =
        // rpm/60 x 4) and return the raw peak plus a 20 ms moving average,
        // which strips the 30 Hz thumps and leaves the crank-rate rock.
        struct Run { double peak = 0.0; std::vector<double> lp; };
        auto run = [&](double fireHz, double load, bool lim) {
            Layer L; L.configureFx(FxType::RpmVibe, p);
            Run r; std::vector<double> raw;
            for (int i = 0; i < 2000; ++i)                  // 1 s at 2 kHz
            {
                L.driveEngine(1.0, fireHz, load, lim);
                L.step(DT);
                const double v = L.overlayFor(6);
                raw.push_back(v);
                r.peak = std::max(r.peak, std::fabs(v));
            }
            for (size_t i = 40; i < raw.size(); ++i)
            { double s = 0.0; for (size_t k = i - 40; k < i; ++k) s += raw[k]; r.lp.push_back(s / 40.0); }
            return r;
        };
        auto crossings = [](const std::vector<double>& x) {
            int n = 0; for (size_t i = 1; i < x.size(); ++i) if ((x[i] >= 0.0) != (x[i-1] >= 0.0)) ++n; return n; };
        auto lpEnergy = [](const std::vector<double>& x) {
            double e = 0.0; for (double v : x) e += std::fabs(v); return e; };

        // V8 idle, 800 rpm: fireHz 53.3, crank 13.3 Hz. The felt content is
        // the crank-rate rock: the low-passed signal crosses zero ~27 times
        // a second (2 per rev), not at the 53 Hz firing rate.
        const Run idle = run(53.3, 1.0, false);
        CHECK(idle.peak > 5.0,            "engine produces torque at idle");
        CHECK(idle.peak <= 10.0 + 1e-9,   "engine bounded by its own amp");
        const int xIdle = crossings(idle.lp);
        CHECK(xIdle >= 20 && xIdle <= 34, "idle rock runs at crank rate (~27 crossings/s), not firing rate");

        // 6000 rpm: fireHz 400, crank 100 Hz. The rock is gone, the thumps
        // merge into a continuous buzz with no long gaps, still bounded.
        const Run high = run(400.0, 1.0, false);
        CHECK(high.peak <= 10.0 + 1e-9,   "high rpm still bounded by amp (overlap normalised)");
        CHECK(lpEnergy(high.lp) < 0.3 * lpEnergy(idle.lp), "rock fades out by high rpm");
        {
            Layer Lh; Lh.configureFx(FxType::RpmVibe, p);
            int g = 0, maxGap = 0;
            for (int i = 0; i < 2000; ++i)
            {
                Lh.driveEngine(1.0, 400.0, 1.0, false); Lh.step(DT);
                if (i > 200 && std::fabs(Lh.overlayFor(6)) < 0.01) { ++g; maxGap = std::max(maxGap, g); }
                else g = 0;
            }
            CHECK(maxGap <= 4, "high rpm: no long gaps - pulses merge into buzz");
        }

        // Fewer cylinders = sparser firings, same character: a 4-cyl at
        // 800 rpm (fireHz 26.7) still rocks at the same crank rate.
        {
            EffectParams p4 = p; p4.cylinders = 4.0;
            Layer L4; L4.configureFx(FxType::RpmVibe, p4);
            std::vector<double> raw, lp;
            for (int i = 0; i < 2000; ++i) { L4.driveEngine(1.0, 26.7, 1.0, false); L4.step(DT); raw.push_back(L4.overlayFor(6)); }
            for (size_t i = 40; i < raw.size(); ++i)
            { double s = 0.0; for (size_t k = i - 40; k < i; ++k) s += raw[k]; lp.push_back(s / 40.0); }
            const int x4 = crossings(lp);
            CHECK(x4 >= 20 && x4 <= 34,   "4-cyl idle rocks at the same crank rate");
        }

        // Component mix: rock 0 leaves only thumps (low-passed signal
        // collapses), thump 0 leaves only the rock (no gaps, no 30 Hz
        // content), both 0 is silence. Set-by-feel on the tile.
        {
            auto lpOf = [&](const EffectParams& q, double fireHz) {
                Layer L; L.configureFx(FxType::RpmVibe, q);
                std::vector<double> raw, lp; double pk = 0.0;
                for (int i = 0; i < 2000; ++i) { L.driveEngine(1.0, fireHz, 1.0, false); L.step(DT);
                    raw.push_back(L.overlayFor(6)); pk = std::max(pk, std::fabs(raw.back())); }
                for (size_t i = 40; i < raw.size(); ++i)
                { double s = 0.0; for (size_t k = i - 40; k < i; ++k) s += raw[k]; lp.push_back(s / 40.0); }
                return std::make_pair(pk, lpEnergy(lp));
            };
            EffectParams noRock = p;  noRock.rock  = 0.0;
            EffectParams noThump = p; noThump.thump = 0.0;
            EffectParams none = p;    none.rock = 0.0; none.thump = 0.0;
            const auto full = lpOf(p, 53.3), nr = lpOf(noRock, 53.3), nt = lpOf(noThump, 53.3), z = lpOf(none, 53.3);
            CHECK(nr.second < 0.15 * full.second, "rock x 0: crank-rate content gone, thumps remain");
            CHECK(nr.first > 2.0,                 "rock x 0: thumps still produce torque");
            CHECK(nt.second > 0.8 * full.second,  "thump x 0: the rock is untouched");
            CHECK(z.first == 0.0,                 "rock 0 + thump 0: silence");
        }

        // Load: coasting hits softer than full load.
        const Run coast = run(53.3, 0.0, false);
        CHECK(coast.peak > 1.0 && coast.peak < idle.peak * 0.75, "coasting hits softer than full load");

        // Limiter: whole bursts are cut (~12 Hz gate, half off), so energy
        // drops AND the output has real holes of >= 30 ms, which is the
        // bounce. Random single misfires never produce holes that long.
        auto energy = [&](bool lim){
            Layer Le; Le.configureFx(FxType::RpmVibe, p);
            double e = 0.0; int g = 0, maxGap = 0;
            for (int i = 0; i < 4000; ++i)
            { Le.driveEngine(1.0, 60.0, 1.0, lim); Le.step(DT);
              const double a = std::fabs(Le.overlayFor(6)); e += a;
              if (a < 0.01) { ++g; maxGap = std::max(maxGap, g); } else g = 0; }
            return std::make_pair(e, maxGap);
        };
        const auto on = energy(true), off = energy(false);
        CHECK(on.first < off.first * 0.8, "limiter cuts firings: energy falls");
        CHECK(on.second >= 50,            "limiter leaves holes of >= 25 ms (the bounce)");
        CHECK(off.second < 50,            "no such holes without the limiter");

        // No firing rate = silence even when driven.
        Layer Lz; Lz.configureFx(FxType::RpmVibe, p);
        for (int i = 0; i < 200; ++i) { Lz.driveEngine(1.0, 0.0, 1.0, false); Lz.step(DT); }
        CHECK(Lz.overlayFor(6) == 0.0, "engine with no rpm is silent");
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
