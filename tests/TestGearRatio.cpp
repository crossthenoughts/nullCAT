// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestGearRatio.cpp - unit tests for the gear-ratio learner and the
// revmatch let-in it feeds.
//
// Pins: learning + confidence in seconds (the same at any control rate),
// the sample guards (speed/rpm/clutch/brake/neutral/stale, the gear held,
// rpm/speed steady: no launch or wheelspin), a one-frame junk gear teaches
// nothing, outlier-driven relearn (car change), gears that contradict each
// other are not used, cache adoption (fills undriven gears, never from an
// out-of-order set), fast rejection of a wrong adoption, and the
// DeviceStateLayer revmatch consumption. Plain-main, no Qt.
// ============================================================

#include "../src/GearRatioLearner.h"
#include <cstdio>
#include <cmath>

static int g_pass = 0, g_fail = 0;
static void CHECK(bool ok, const char* what)
{
    if (ok) { ++g_pass; }
    else    { ++g_fail; std::printf("FAIL: %s\n", what); }
}

static NcxValues nv(double rpm, double sp, double gear, double clutch = 0.0, double brake = 0.0)
{
    NcxValues v; v.fresh = true;
    v.have[NcxValues::Rpm] = v.have[NcxValues::SpeedKmh] =
    v.have[NcxValues::Gear] = v.have[NcxValues::ClutchPct] = v.have[NcxValues::BrakePct] = true;
    v.val[NcxValues::Rpm] = rpm; v.val[NcxValues::SpeedKmh] = sp;
    v.val[NcxValues::Gear] = gear; v.val[NcxValues::ClutchPct] = clutch; v.val[NcxValues::BrakePct] = brake;
    return v;
}

static double g_dt = 0.0005;   // the Pi's 2 kHz loop (one block runs at 500 Hz)

// Drive gear g at ratio r (speed 100 km/h) for sec seconds.
static void drive(GearRatioLearner& L, int g, double r, double sec)
{
    const int n = static_cast<int>(sec / g_dt + 0.5);
    for (int i = 0; i < n; ++i) L.step(nv(r * 100.0, 100.0, g), g_dt);
}

int main()
{
    const double HOLD = GearRatioLearner::HOLD_SEC, CONF = GearRatioLearner::CONFIDENT_SEC;

    // ---------- learning + confidence, in seconds, at any loop rate ----------
    for (const double dt : { 0.0005, 0.002 })
    {
        g_dt = dt;
        GearRatioLearner L;
        drive(L, 3, 40.0, HOLD + CONF - 0.1);
        CHECK(!L.ratios().known[3], "not usable before the confidence time (the gear held, then 2 s agreeing)");
        drive(L, 3, 40.0, 0.2);
        CHECK(L.ratios().known[3] && std::fabs(L.ratios().r[3] - 40.0) < 0.5,
              "gear ratio learned and usable after enough agreeing seconds");
        CHECK(L.gearSessionConfident(3) && L.dirty(),
              "session-confident + dirty (worth persisting)");
        CHECK(!L.ratios().known[2], "undriven gears stay unknown");
    }
    g_dt = 0.0005;

    // ---------- sample guards ----------
    {
        GearRatioLearner L;
        const int n = static_cast<int>(10.0 / g_dt);
        for (int i = 0; i < n; ++i) L.step(nv(4000.0, 10.0, 3), g_dt);              // too slow
        for (int i = 0; i < n; ++i) L.step(nv(4000.0, 100.0, 3, 80.0), g_dt);       // clutch down
        for (int i = 0; i < n; ++i) L.step(nv(4000.0, 100.0, 3, 0.0, 40.0), g_dt);  // braking
        for (int i = 0; i < n; ++i) L.step(nv(4000.0, 100.0, 0), g_dt);             // neutral
        for (int i = 0; i < n; ++i) L.step(nv(4000.0, 100.0, -1), g_dt);            // reverse
        NcxValues stale = nv(4000.0, 100.0, 3); stale.fresh = false;
        for (int i = 0; i < n; ++i) L.step(stale, g_dt);                            // dead stream
        bool any = false;
        for (int g = 1; g < MAX_GEARS; ++g) if (L.ratios().known[g] || L.gearRatio(g) > 0.0) any = true;
        CHECK(!any, "slow/clutch-down/braking/neutral/reverse/stale samples never learn");
    }

    // ---------- a launch (rpm on the limiter, speed climbing) is no ratio ----------
    {
        GearRatioLearner L;
        const int n = static_cast<int>(4.0 / g_dt);
        for (int i = 0; i < n; ++i)
        {
            const double sp = 20.0 + 15.0 * (i * g_dt);   // 20 to 80 km/h in 4 s, the wheels spinning up the revs
            L.step(nv(7000.0, sp, 1), g_dt);
        }
        CHECK(!L.ratios().known[1] && L.gearRatio(1) == 0.0, "a launch with rpm running away from the speed teaches 1st nothing");
    }
    // ...and from 1st held at the line: the wheels spinning past 15 km/h
    // must not pass as steady on the first samples, launch after launch.
    {
        GearRatioLearner L;
        for (int k = 0; k < 150; ++k)
        {
            for (int i = 0; i < static_cast<int>(1.0 / g_dt); ++i) L.step(nv(3000.0, 0.0, 1), g_dt);   // held at the line
            for (int i = 0; i < static_cast<int>(1.5 / g_dt); ++i)
            {
                const double sp = 10.0 + 30.0 * (i * g_dt);   // the revs pinned, the speed climbing
                L.step(nv(7000.0, sp, 1), g_dt);
            }
        }
        CHECK(L.gearRatio(1) == 0.0 && !L.gearSessionConfident(1), "150 launches from 1st held at the line teach 1st nothing");
    }
    // Off the brake in a held gear: the first samples after are no ratio either.
    {
        GearRatioLearner L;
        for (int i = 0; i < static_cast<int>(2.0 / g_dt); ++i) L.step(nv(2000.0, 100.0, 3, 0.0, 60.0), g_dt);   // braking in 3rd
        for (int i = 0; i < static_cast<int>(0.1 / g_dt); ++i) L.step(nv(2000.0, 100.0, 3), g_dt);              // off the brake, 0.1 s
        CHECK(L.gearRatio(3) == 0.0, "the moment off the brake in a held gear seeds nothing");
    }

    // ---------- a one-frame junk gear teaches nothing ----------
    {
        GearRatioLearner L;
        drive(L, 4, 40.0, 3.0);
        L.step(nv(4000.0, 100.0, 2), 1.0 / 60.0);   // the sim's junk "2" for one 60 Hz frame
        drive(L, 4, 40.0, 3.0);
        CHECK(L.gearRatio(2) == 0.0 && !L.ratios().known[2], "a one-frame junk gear seeds nothing");
        CHECK(L.ratios().known[4], "...and the real gear carries on");
    }

    // ---------- car change: sustained disagreement relearns ----------
    {
        GearRatioLearner L;
        drive(L, 3, 40.0, HOLD + CONF + 0.1);
        drive(L, 3, 60.0, GearRatioLearner::RELEARN_SEC - 0.5);
        CHECK(std::fabs(L.ratios().r[3] - 40.0) < 0.5,
              "brief disagreement (a few seconds) does not move the estimate");
        // (A step from 40 to 60 is no steady ratio for the first ~0.8 s:
        // the disagreement counts from there.)
        drive(L, 3, 60.0, 0.6 + 1.0 + CONF + 0.1);
        CHECK(L.ratios().known[3] && std::fabs(L.ratios().r[3] - 60.0) < 0.5,
              "sustained disagreement relearns the gear (new car)");
    }

    // ---------- gears that contradict each other are not used ----------
    {
        GearRatioLearner L;
        drive(L, 2, 33.7, HOLD + CONF + 0.1);   // 2nd learned from garbage...
        drive(L, 3, 46.2, HOLD + CONF + 0.1);   // ...below 3rd
        CHECK(!L.ratios().known[2] && !L.ratios().known[3],
              "a higher gear with the higher ratio: neither is used");
        CHECK(!L.gearSessionConfident(2) && !L.gearSessionConfident(3),
              "...nor persisted");
        drive(L, 4, 40.0, HOLD + CONF + 0.1);
        CHECK(!L.ratios().known[4], "a gear a lower confident gear contradicts is not used either (4th at 40 above 2nd's 33.7)");
    }

    // ---------- cache adoption fills undriven gears ----------
    CachedCar car{};
    car.r[1] = 120.0; car.has[1] = true;
    car.r[2] = 85.0;  car.has[2] = true;
    car.r[3] = 60.0;  car.has[3] = true;
    car.r[4] = 45.0;  car.has[4] = true;
    {
        GearRatioLearner L;
        L.setCache(&car, 1);
        drive(L, 1, 120.5, HOLD + CONF + 0.1);
        CHECK(!L.ratios().known[3],
              "one confident gear is not identity - no adoption yet");
        drive(L, 2, 85.2, HOLD + CONF + 0.1);
        CHECK(L.ratios().known[3] && std::fabs(L.ratios().r[3] - 60.0) < 1e-9 &&
              L.ratios().known[4] && std::fabs(L.ratios().r[4] - 45.0) < 1e-9,
              "two matching gears adopt the remembered car's other ratios");
        CHECK(!L.gearSessionConfident(3),
              "adopted values are usable but NOT session-confident (never persisted)");

        // ---------- a wrong adopted value dies fast, alone ----------
        drive(L, 3, 70.0, HOLD + GearRatioLearner::ADOPTED_DROP_SEC + 0.2);
        CHECK(std::fabs(L.ratios().r[3] - 70.0) < 0.5,
              "an adopted value contradicted by live driving is replaced quickly");
        CHECK(L.ratios().known[4] && std::fabs(L.ratios().r[4] - 45.0) < 1e-9,
              "the other adopted gears survive (the match may still be right)");
    }

    // ---------- an out-of-order remembered set is never adopted ----------
    {
        // The wrong pair (5th above 4th) away from the gears driven, so only
        // the set's own order can refuse it.
        CachedCar bad{};
        bad.r[1] = 120.0; bad.has[1] = true;
        bad.r[2] = 85.0;  bad.has[2] = true;
        bad.r[3] = 60.0;  bad.has[3] = true;
        bad.r[4] = 45.0;  bad.has[4] = true;
        bad.r[5] = 50.0;  bad.has[5] = true;
        GearRatioLearner L;
        L.setCache(&bad, 1);
        drive(L, 1, 120.0, HOLD + CONF + 0.1);
        drive(L, 2, 85.0, HOLD + CONF + 0.1);
        CHECK(!L.ratios().known[3], "a remembered set out of gear order adopts nothing");
    }

    // ---------- session relearn (car change) drops every adoption ----------
    {
        GearRatioLearner L;
        L.setCache(&car, 1);
        drive(L, 1, 120.0, HOLD + CONF + 0.1);
        drive(L, 2, 85.0, HOLD + CONF + 0.1);
        CHECK(L.ratios().known[4], "adopted (setup)");
        drive(L, 1, 100.0, HOLD + GearRatioLearner::RELEARN_SEC + 0.1);   // new car in gear 1
        CHECK(!L.ratios().known[4],
              "relearning a session gear drops the old car's adopted ratios");
    }

    // ---------- revmatch let-in through DeviceStateLayer ----------
    {
        DeviceParams p;
        p.detents        = { -0.05, 0.0, 0.05 };
        p.clutchBitePct  = 25.0;
        p.blockGain      = 1.0;
        p.blockStartRev  = 0.01;
        p.rpmMatchPct    = 8.0;
        DeviceStateLayer layer; layer.configure(p);
        GearRatios rat{};
        rat.r[3] = 60.0; rat.known[3] = true;

        // Mid-shift (0.02 rev out of the 0.0 detent), clutch up, in gear 2.
        // Destination gear 3 wants 60 * 100 = 6000 rpm at 100 km/h.
        DeviceStateMods m = layer.step(0.02, nv(6000.0, 100.0, 2, 5.0), &rat);
        CHECK(m.forceScale == 1.0, "revmatched clutchless shift lets in (no block)");
        m = layer.step(0.02, nv(4000.0, 100.0, 2, 5.0), &rat);
        CHECK(m.forceScale > 1.0, "wrong rpm: the block stands");
        m = layer.step(0.02, nv(6000.0, 3.0, 2, 5.0), &rat);
        CHECK(m.forceScale > 1.0, "near-standstill: no meaningful match, block stands");
        GearRatios none{};
        m = layer.step(0.02, nv(6000.0, 100.0, 2, 5.0), &none);
        CHECK(m.forceScale > 1.0, "destination ratio unknown: block stands");
        DeviceParams off = p; off.rpmMatchPct = 0.0;
        DeviceStateLayer loff; loff.configure(off);
        m = loff.step(0.02, nv(6000.0, 100.0, 2, 5.0), &rat);
        CHECK(m.forceScale > 1.0, "rpmMatchPct 0: revmatch disabled, block stands");
    }

    std::printf("TestGearRatio: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
