// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestGearRatio.cpp - unit tests for the gear-ratio learner and the
// revmatch let-in it feeds.
//
// Pins: learning + confidence, the sample guards (speed/rpm/clutch/
// neutral/stale), outlier-driven relearn (car change), cache adoption
// (fills undriven gears), fast rejection of a wrong adoption, and the
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

static NcxValues nv(double rpm, double sp, double gear, double clutch = 0.0)
{
    NcxValues v; v.fresh = true;
    v.have[NcxValues::Rpm] = v.have[NcxValues::SpeedKmh] =
    v.have[NcxValues::Gear] = v.have[NcxValues::ClutchPct] = true;
    v.val[NcxValues::Rpm] = rpm; v.val[NcxValues::SpeedKmh] = sp;
    v.val[NcxValues::Gear] = gear; v.val[NcxValues::ClutchPct] = clutch;
    return v;
}

// Drive gear g at ratio r (speed 100 km/h) for n agreeing cycles.
static void drive(GearRatioLearner& L, int g, double r, int n)
{
    for (int i = 0; i < n; ++i) L.step(nv(r * 100.0, 100.0, g));
}

int main()
{
    const int N = GearRatioLearner::CONFIDENT_N;

    // ---------- learning + confidence ----------
    {
        GearRatioLearner L;
        drive(L, 3, 40.0, N - 1);
        CHECK(!L.ratios().known[3], "not usable before the confidence threshold");
        drive(L, 3, 40.0, 1);
        CHECK(L.ratios().known[3] && std::fabs(L.ratios().r[3] - 40.0) < 0.5,
              "gear ratio learned and usable after enough agreeing samples");
        CHECK(L.gearSessionConfident(3) && L.dirty(),
              "session-confident + dirty (worth persisting)");
        CHECK(!L.ratios().known[2], "undriven gears stay unknown");
    }

    // ---------- sample guards ----------
    {
        GearRatioLearner L;
        for (int i = 0; i < N; ++i) L.step(nv(4000.0, 10.0, 3));         // too slow
        for (int i = 0; i < N; ++i) L.step(nv(4000.0, 100.0, 3, 80.0));  // clutch down
        for (int i = 0; i < N; ++i) L.step(nv(4000.0, 100.0, 0));        // neutral
        for (int i = 0; i < N; ++i) L.step(nv(4000.0, 100.0, -1));       // reverse
        NcxValues stale = nv(4000.0, 100.0, 3); stale.fresh = false;
        for (int i = 0; i < N; ++i) L.step(stale);                       // dead stream
        bool any = false;
        for (int g = 1; g < MAX_GEARS; ++g) if (L.ratios().known[g]) any = true;
        CHECK(!any, "slow/clutch-down/neutral/reverse/stale samples never learn");
    }

    // ---------- car change: sustained disagreement relearns ----------
    {
        GearRatioLearner L;
        drive(L, 3, 40.0, N);
        drive(L, 3, 60.0, GearRatioLearner::OUTLIER_RELEARN - 1);
        CHECK(std::fabs(L.ratios().r[3] - 40.0) < 0.5,
              "brief disagreement (wheelspin-scale) does not move the estimate");
        drive(L, 3, 60.0, 1 + N);
        CHECK(L.ratios().known[3] && std::fabs(L.ratios().r[3] - 60.0) < 0.5,
              "sustained disagreement relearns the gear (new car)");
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
        drive(L, 1, 120.5, N);
        CHECK(!L.ratios().known[3],
              "one confident gear is not identity - no adoption yet");
        drive(L, 2, 85.2, N);
        CHECK(L.ratios().known[3] && std::fabs(L.ratios().r[3] - 60.0) < 1e-9 &&
              L.ratios().known[4] && std::fabs(L.ratios().r[4] - 45.0) < 1e-9,
              "two matching gears adopt the remembered car's other ratios");
        CHECK(!L.gearSessionConfident(3),
              "adopted values are usable but NOT session-confident (never persisted)");

        // ---------- a wrong adopted value dies fast, alone ----------
        for (int i = 0; i < GearRatioLearner::ADOPTED_DROP; ++i)
            L.step(nv(90.0 * 100.0, 100.0, 3));
        CHECK(std::fabs(L.ratios().r[3] - 90.0) < 0.5,
              "an adopted value contradicted by live driving is replaced quickly");
        CHECK(L.ratios().known[4] && std::fabs(L.ratios().r[4] - 45.0) < 1e-9,
              "the other adopted gears survive (the match may still be right)");
    }

    // ---------- session relearn (car change) drops every adoption ----------
    {
        GearRatioLearner L;
        L.setCache(&car, 1);
        drive(L, 1, 120.0, N);
        drive(L, 2, 85.0, N);
        CHECK(L.ratios().known[4], "adopted (setup)");
        drive(L, 1, 100.0, GearRatioLearner::OUTLIER_RELEARN);   // new car in gear 1
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
