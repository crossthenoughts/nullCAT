// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestHomingSeat.cpp - regression test for the deinit "seat" mode of HomingSequence.
//
// Proves the three invariants the seat-mode flag must hold (see the design review):
//   1. ADDITIVE: normal homing (seatMode=false) is byte-identical to seat homing
//      (seatMode=true) for the ENTIRE shared path - enable, settle, search-to-torque - //      diverging at EXACTLY ONE cycle: the hardstop. Same search, same detection cycle,
//      bit-identical commanded/actual trace up to and including that cycle.
//   3. NO HOME REFERENCE: seat mode never enters Backoff and never reaches Complete, and
//      establishes NO home offset (getHomeOffset() stays 0). Normal homing does both.
//   (+ seat mode holds ON the hardstop; normal homing backs off to ~backoff position.)
//
// Hardware-free: drives HomingSequence via MockA6Drive (no SOEM, no NIC, no Qt).
//   g++ -std=c++17 -I src tests/TestHomingSeat.cpp src/A6Drive.cpp src/Logging.cpp -o /tmp/ths && /tmp/ths
// ============================================================
#include "HomingSequence.h"
#include "MockA6Drive.h"
#include "Config.h"
#include "SeatPreflight.h"

#include <cstdio>
#include <vector>
#include <cstddef>

static int g_fail = 0, g_pass = 0;
static void check(bool ok, const char* name)
{
    if (ok) { ++g_pass; std::printf("[PASS] %s\n", name); }
    else    { ++g_fail; std::printf("[FAIL] %s\n", name); }
}

struct Trace
{
    std::vector<HomingSequence::State> states;
    std::vector<double>                actuals;   // raw actual position after each step()
    double               homeOffset = 0.0;        // HomingSequence's internal offset (0 if never set)
    HomingSequence::State finalState = HomingSequence::State::Idle;
    bool sawBackoff = false, sawComplete = false, sawSeated = false;
};

static DriveConfig makeCfg()
{
    DriveConfig c;
    c.homeDirection  = "negative";   // search downward (toward the min/bottom stop)
    c.invertDir      = true;         // foldback fixture: retract = raw negative (see TestHomingSequence)
    c.homingTorquePct = 25;          // the rig's threshold
    c.homingBackoffMm = 1.0;         // normal homing backs off 1.0mm; seat must NOT
    c.homingSpeed  = 50.0;
    c.strokeMm        = 80.0;
    return c;
}

// Run one homing to a terminal state against a fresh mock + a hardstop at 0.0.
static Trace runHoming(bool seatMode, double holdTorque = 0.0)
{
    Trace t;
    MockA6Drive drive;
    drive.configure(/*slave*/1, /*startPos*/5.0, /*maxVelPerCycle*/0.05);
    drive.setHardstop(/*pos*/0.0, /*isMinLimit*/true, /*torquePct*/50.0);  // 50% >= 25% threshold
    // Seated-driver model: static load-holding torque present the whole run
    // (rig-measured ~30-40% with a driver aboard). The detector must key on
    // DEVIATION from the settled baseline -- |torque|>=25% false-triggers here.
    drive.setSimHoldTorque(holdTorque);

    HomingSequence hs;
    hs.configure(makeCfg(), /*cycleTimeSec*/0.0005);
    hs.start(&drive, seatMode);

    const int maxIters = 8000;
    for (int k = 0; k < maxIters; ++k)
    {
        HomingSequence::State s = hs.step(&drive);
        t.states.push_back(s);
        t.actuals.push_back(drive.getActualPositionRaw());
        if (s == HomingSequence::State::Backoff)  t.sawBackoff  = true;
        if (s == HomingSequence::State::Complete) t.sawComplete = true;
        if (s == HomingSequence::State::Seated)   t.sawSeated   = true;

        if (s == HomingSequence::State::Complete || s == HomingSequence::State::Error ||
            s == HomingSequence::State::FatalError)
        {
            t.finalState = s;
            break;
        }
        if (s == HomingSequence::State::Seated)
        {
            // Confirm it keeps holding on the stop for a few more cycles, then stop.
            for (int h = 0; h < 5; ++h)
            {
                HomingSequence::State s2 = hs.step(&drive);
                t.states.push_back(s2);
                t.actuals.push_back(drive.getActualPositionRaw());
            }
            t.finalState = HomingSequence::State::Seated;
            break;
        }
    }
    t.homeOffset = hs.getHomeOffset();
    return t;
}

// First index where state == target, or -1.
static int firstIndexOf(const Trace& t, HomingSequence::State target)
{
    for (size_t k = 0; k < t.states.size(); ++k)
        if (t.states[k] == target) return static_cast<int>(k);
    return -1;
}

int main()
{
    const Trace norm = runHoming(/*seatMode=*/false);
    const Trace seat = runHoming(/*seatMode=*/true);

    std::printf("== normal homing reaches the reference (back-off + offset) ==\n");
    check(norm.finalState == HomingSequence::State::Complete, "normal -> Complete");
    check(norm.sawBackoff, "normal performed back-off");
    check(norm.homeOffset > 0.5, "normal established a home offset (~1.0)");
    check(!norm.actuals.empty() && norm.actuals.back() > 0.5,
          "normal final position is the backed-off home (~1.0mm), off the stop");

    std::printf("== seat homing holds on the stop, no reference (req 3) ==\n");
    check(seat.finalState == HomingSequence::State::Seated, "seat -> Seated");
    check(!seat.sawBackoff,  "seat NEVER entered Backoff");
    check(!seat.sawComplete, "seat NEVER reached Complete");
    check(seat.homeOffset == 0.0, "seat established NO home offset (stays 0)");
    check(!seat.actuals.empty() && seat.actuals.back() < 0.05,
          "seat final position is ON the stop (~0.0mm)");

    std::printf("== shared path is byte-identical, diverging only at the hardstop (req 1) ==\n");
    const int transN = firstIndexOf(norm, HomingSequence::State::Backoff);  // normal diverges here
    const int transS = firstIndexOf(seat, HomingSequence::State::Seated);   // seat diverges here
    check(transN > 0, "normal reached the hardstop (Backoff) cycle");
    check(transS > 0, "seat reached the hardstop (Seated) cycle");
    check(transN == transS, "both modes detect the hardstop on the SAME cycle");

    bool prefixIdentical = (transN > 0 && transN == transS);
    if (prefixIdentical)
    {
        for (int k = 0; k <= transN; ++k)
        {
            // bit-exact: same code, same inputs -> same doubles
            if (norm.actuals[k] != seat.actuals[k]) { prefixIdentical = false; break; }
            // states identical for every cycle BEFORE the divergence cycle
            if (k < transN && norm.states[k] != seat.states[k]) { prefixIdentical = false; break; }
        }
    }
    check(prefixIdentical, "actual-position trace bit-identical through the hardstop cycle");

    bool divergeState = (transN == transS) &&
                        norm.states[transN] == HomingSequence::State::Backoff &&
                        seat.states[transN] == HomingSequence::State::Seated;
    check(divergeState, "the ONE divergence is exactly Backoff (normal) vs Seated (seat)");

    std::printf("== seated driver: holding torque must not fake the hardstop ==\n");
    // Rig incident (Jul 4): with a driver seated, static holding torque ~30-40%
    // exceeded the bare 25%% threshold -> "HARDSTOP FOUND ... distance=0.004mm" at
    // the PARK position -> seat de-energized ~1.5mm in the air and the rig dropped.
    // With baseline-relative detection both modes must search the full 5mm to the
    // REAL stop and land there.
    {
        const Trace n35 = runHoming(/*seatMode=*/false, /*holdTorque=*/35.0);
        const Trace s35 = runHoming(/*seatMode=*/true,  /*holdTorque=*/35.0);
        double nMin = 1e9, sMin = 1e9;
        for (double a : n35.actuals) nMin = std::min(nMin, a);
        for (double a : s35.actuals) sMin = std::min(sMin, a);
        check(n35.finalState == HomingSequence::State::Complete, "seated normal -> Complete");
        check(nMin < 0.1,  "seated normal searched to the REAL stop (no false trip at 5mm)");
        check(n35.homeOffset > 0.5 && n35.homeOffset < 1.5,
              "seated normal home offset is at the true stop (+backoff), not mid-air");
        check(s35.finalState == HomingSequence::State::Seated, "seated seat -> Seated");
        check(sMin < 0.1,  "seated seat pressed to the REAL stop before de-energize");
        check(!s35.actuals.empty() && s35.actuals.back() < 0.05,
              "seated seat holds ON the stop (rig cannot drop on de-energize)");
    }

    // ---- The de-init seat pass's bus check, one frame at a time: a frame
    // every slave worked on is good, one only some worked on is partial (the
    // pass seats the axes still in OP), no reply is dead. The pass used to
    // read every mismatch as dead and skipped the whole seat on a rig with
    // one drive off. ----
    {
        check(classifySeatProbe(18, 18) == SeatProbe::Good,    "seat probe: the expected working counter is a good frame");
        check(classifySeatProbe(6, 18)  == SeatProbe::Partial, "seat probe: some slaves worked on it = partial, not dead");
        check(classifySeatProbe(17, 18) == SeatProbe::Partial, "seat probe: one slave short = partial");
        check(classifySeatProbe(19, 18) == SeatProbe::Partial, "seat probe: more than expected is not a clean frame");
        check(classifySeatProbe(0, 18)  == SeatProbe::Dead,    "seat probe: a reply no slave worked on = dead");
        check(classifySeatProbe(-1, 18) == SeatProbe::Dead,    "seat probe: no reply = dead");
        check(classifySeatProbe(0, 0)   == SeatProbe::Dead,    "seat probe: nothing expected, nothing back = dead, never good");

        // The verdict from the window's counts (18 is the shipped target).
        check(seatPlanFor(18, 0, 18)   == SeatPlan::Seat,    "seat plan: 18 good frames seat normally");
        check(seatPlanFor(0, 500, 18)  == SeatPlan::Partial, "seat plan: a window of partial frames seats the reachable axes");
        check(seatPlanFor(0, 240, 18)  == SeatPlan::Partial, "seat plan: half the window lost to a delivery pause is still partial");
        check(seatPlanFor(0, 10, 18)   == SeatPlan::Skip,    "seat plan: a few partial frames are not a partial bus");
        check(seatPlanFor(17, 0, 18)   == SeatPlan::Skip,    "seat plan: one good frame short of 18 with nothing else skips");

        // During the press, relief and dwell.
        check(seatFrameOk(18, 18, 0),   "seat frame: a healthy bus wants every slave");
        check(!seatFrameOk(6, 18, 0),   "seat frame: a healthy bus losing slaves is a bad frame");
        check(!seatFrameOk(-1, 18, 0),  "seat frame: no reply is a bad frame");
        check(seatFrameOk(6, 18, 6),    "seat frame: a partial bus as the pre-flight found it is fine (no abort while seating)");
        check(seatFrameOk(9, 18, 6),    "seat frame: a partial bus with a slave back is fine");
        check(!seatFrameOk(3, 18, 6),   "seat frame: a partial bus getting worse is a bad frame");
        check(!seatFrameOk(-1, 18, 6),  "seat frame: no reply on a partial bus is a bad frame");
        check(!seatFrameOk(19, 18, 0),  "seat frame: more than expected on a healthy bus is not a clean frame");

        // Verdict boundaries: Seat wins, 18 partial is enough, the two counts never add up.
        check(seatPlanFor(18, 500, 18) == SeatPlan::Seat,    "seat plan: 18 good frames seat normally whatever else came");
        check(seatPlanFor(0, 18, 18)   == SeatPlan::Partial, "seat plan: exactly 18 partial frames is a partial bus");
        check(seatPlanFor(10, 10, 18)  == SeatPlan::Skip,    "seat plan: good and partial frames are not summed");

        // The tally and the floor a partial seat is held to.
        {
            SeatTally t;
            t.add(18, 18); t.add(-1, 18); t.add(9, 18); t.add(6, 18); t.add(9, 18);
            check(t.good == 1 && t.dead == 1 && t.partial == 3 && t.probes == 5, "seat tally: counts every probe once by kind");
            check(t.partialFloor(18) == 9, "seat tally: the floor is the latest partial level, not an earlier glitch");
            SeatTally none;
            none.add(18, 18); none.add(-1, 18);
            check(none.partialFloor(18) == 0, "seat tally: no partial frame, no floor (a healthy-bus seat)");
            SeatTally high;
            high.add(19, 18);
            check(high.partialFloor(18) == 18, "seat tally: the floor never sits above a full frame");
        }
    }

    std::printf("\n==== %d passed, %d failed ====\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
