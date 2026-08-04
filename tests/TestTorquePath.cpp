// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestTorquePath.cpp — belt / torque-mode safety invariants for MotionController.
//
// The belt torque command is computed by MotionController::process() from the axis
// state + telemetry input, independent of the drive object, so this drives the controller
// directly with drives=nullptr (no MockA6Drive, no SOEM, no NIC, no Qt).
//
// It locks the invariants the belt path MUST hold so the blend refactor that follows
// (and any future edit) cannot silently regress them. Torque mode shipped broken for
// its entire life once before precisely because nothing asserted "select torque =>
// correct command". These are that assertion.
//
// Each cycle uses a FRESH MotionOutput, exactly as ControlLoop does (the real system
// never carries torque across cycles), so "0 on stop/estop" is tested as the system
// actually sees it.
//
//   g++ -std=c++17 -I src tests/TestTorquePath.cpp src/MotionController.cpp \
//       src/A6Drive.cpp src/Logging.cpp -o /tmp/ttp && /tmp/ttp
// ============================================================
#include "MotionController.h"
#include "TelemetryInput.h"
#include "Config.h"

#include <cstdio>
#include <cmath>

static int g_fail = 0, g_pass = 0;
static void check(bool ok, const char* name)
{
    if (ok) { ++g_pass; std::printf("[PASS] %s\n", name); }
    else    { ++g_fail; std::printf("[FAIL] %s\n", name); }
}
static void approx(double got, double want, double tol, const char* name)
{
    bool ok = std::fabs(got - want) <= tol;
    if (ok) { ++g_pass; std::printf("[PASS] %s (got %.4f)\n", name, got); }
    else    { ++g_fail; std::printf("[FAIL] %s (got %.4f, want %.4f +/- %.4f)\n", name, got, want, tol); }
}

// Single belt/torque axis. Belt skips homing and (today) goes ONLINE directly on
// unpark. Short unpark/blend/park times so the state machine settles in a few cycles.
static AppConfig makeBeltConfig(double minPct = 5.0, double maxPct = 50.0)
{
    DriveConfig dc;
    dc.slaveIndex          = 1;
    dc.axisType            = "belt";
    dc.mode                = "torque";
    dc.torqueMinPct        = minPct;
    dc.torqueMaxPct        = maxPct;
    dc.strokeMm            = 100.0;
    dc.countsPerMm         = 100.0;
    dc.maxVelocityMmS      = 200.0;
    dc.maxAccelerationMmS2 = 2000.0;
    dc.maxJerkMmS3         = 20000.0;
    dc.unparkTimeSec       = 0.1;
    dc.parkTimeSec         = 0.1;
    dc.onlineHoldTimeoutSec = 5.0;

    AppConfig cfg;
    cfg.controlLoopHz       = 100;     // 10 ms cycle
    cfg.numDrives           = 1;
    cfg.blendTimeSec        = 0.1;
    cfg.blendMaxVelocityMmS = 20.0;
    cfg.drives.push_back(dc);
    return cfg;
}

static TelemetryData sim(double raw, bool valid = true)
{
    TelemetryData sd{};
    sd.valid        = valid;
    sd.numPositions = 1;
    sd.positions[0] = raw;
    sd.packetType   = TelemetryPacketType::Motion;
    return sd;
}

// Run `cycles` of process() with the given input, fresh output each cycle (as
// ControlLoop does). Returns the LAST cycle's commanded torque for axis 0.
static double run(MotionController& mc, const TelemetryData& sd, int cycles)
{
    MotionOutput out{};
    for (int i = 0; i < cycles; ++i) { out = MotionOutput{}; mc.process(sd, out, nullptr, 0); }
    return out.torques[0];
}

// Option A ruling (Jul 20 2026): belts tension ONLY via TensionBelts -- unpark
// (auto or explicit) leaves them slack-parked. Tests online the belt exactly as
// every real surface does: through the command queue.
static void tensionBelt(MotionController& mc)
{
    MotionCommand c;
    c.type = MotionCommand::Type::TensionBelts;
    mc.enqueueCommand(c);   // drained on the next process() cycle
}

// Configure a belt axis and bring it to a settled ONLINE state (any blend-in complete).
static void onlineBelt(MotionController& mc, const AppConfig& cfg)
{
    mc.configure(cfg);
    mc.setEmergencyStop(false);
    tensionBelt(mc);
    run(mc, sim(0.0), 200);   // settle ONLINE (blend-in complete)
}

int main()
{
    Logger::instance().setMinLevel(LogLevel::LVL_ERROR);   // keep the test output clean

    // ---- ONLINE mapping + clamping (the core "select torque => correct command") ----
    {
        MotionController mc; onlineBelt(mc, makeBeltConfig(5.0, 50.0));
        approx(run(mc, sim(0.0),     3),  5.0, 0.01, "raw=0      -> torqueMin (belt stays taut)");
        approx(run(mc, sim(65535.0), 3), 50.0, 0.01, "raw=65535  -> torqueMax");
        approx(run(mc, sim(32768.0), 3), 27.5, 0.10, "raw=mid    -> ~midpoint (linear)");
        approx(run(mc, sim(70000.0), 3), 50.0, 0.01, "raw>65535  -> clamped to torqueMax");
        approx(run(mc, sim(-500.0),  3),  5.0, 0.01, "raw<0      -> clamped to torqueMin");
    }

    // ---- min/max are honoured from config (not hard-coded) ----
    {
        MotionController mc; onlineBelt(mc, makeBeltConfig(10.0, 30.0));
        approx(run(mc, sim(0.0),     3), 10.0, 0.01, "config min respected (10%)");
        approx(run(mc, sim(65535.0), 3), 30.0, 0.01, "config max respected (30%)");
    }
    // Peak reachable: % is of RATED torque, so torqueMaxPct may exceed 100 (up to ~300,
    // bounded by the drive's 0x6072). The old setTargetTorque ±100% clamp made this
    // unreachable; the controller mapping itself must pass it through. (8 cycles: the
    // slew cap legitimately paces the 145% climb at 30%/cycle.)
    {
        MotionController mc; onlineBelt(mc, makeBeltConfig(5.0, 150.0));
        approx(run(mc, sim(65535.0), 8), 150.0, 0.01, "max>100% reachable (150% of rated)");
    }

    // ---- E-STOP cuts torque to 0 even with full sim input ----
    {
        MotionController mc; onlineBelt(mc, makeBeltConfig());
        mc.setEmergencyStop(true);
        approx(run(mc, sim(65535.0), 3), 0.0, 1e-9, "e-stop -> torque 0 (max sim input ignored)");
        // Held through the estop ramp into PARKED, torque must stay 0 the whole time.
        approx(run(mc, sim(65535.0), 100), 0.0, 1e-9, "e-stop held -> torque stays 0");
    }

    // ---- STOP / PARK eases the belt to slack (ends at 0), not an instant cut ----
    {
        MotionController mc; onlineBelt(mc, makeBeltConfig());
        run(mc, sim(65535.0), 20);            // track at max so lastTension is high
        mc.startPark();
        double mid = run(mc, sim(65535.0), 5);   // mid-ramp (parkTime=0.1s -> 10cy)
        check(mid > 0.0 && mid < 50.0, "park eases (mid-ramp torque between 0 and tracked)");
        approx(run(mc, sim(65535.0), 40), 0.0, 1e-9, "park completes -> torque 0 (belt slack)");
    }

    // ---- Blend-in: tension ramps from 0 on enable, never snaps to full ----
    {
        MotionController mc;
        mc.configure(makeBeltConfig());
        mc.setEmergencyStop(false);
        tensionBelt(mc);                       // PARKED -> BLENDING -> ONLINE (Option A path)
        double c1 = run(mc, sim(65535.0), 1);  // first blend cycle: fraction of full only
        check(c1 < 10.0, "enable cycle 1 stays far below full (no snap to full)");
        double settled = run(mc, sim(65535.0), 200);
        approx(settled, 50.0, 0.01, "after blend -> full tracked tension");
    }

    // ---- Stale telemetry: staged standby (hold -> ease to min -> park to 0) ----
    // HOLD_SEC=2s, onlineHoldTimeout=5s, 10ms cycle. Continuous stale stream.
    {
        MotionController mc; onlineBelt(mc, makeBeltConfig());
        run(mc, sim(65535.0), 20);                       // track at max (lastTension=50)
        approx(run(mc, sim(0.0, false), 50),  50.0, 0.5, "stale 0.5s -> hold last tension (phase 1)");
        approx(run(mc, sim(0.0, false), 300),  5.0, 0.5, "stale 3.5s -> ease to torqueMin (phase 2)");
        approx(run(mc, sim(0.0, false), 250),  0.0, 1e-9, "stale 6s   -> parked, slack (phase 3)");
    }

    // ================= Belt guards =================
    // 100Hz cycle: slew 3000%/s = 30%/cycle. encoderCountsPerRev=1000, countsPerMm=100
    // -> unitsPerRev=10; overspeed 600rpm = 100 units/s = 1 unit/cycle.

    // ---- Slew cap: envelope on steps, transparent to ripple ----
    {
        AppConfig cfg = makeBeltConfig(5.0, 50.0);
        MotionController mc; onlineBelt(mc, cfg);           // settled at min (5%)
        approx(run(mc, sim(65535.0), 1), 35.0, 0.01, "slew: instant max step paced to +30%/cycle");
        run(mc, sim(65535.0), 8);                            // settle at 50
        // 10%-amplitude alternation (well under 30%/cycle) must pass UNATTENUATED:
        // 50% <-> 40% raw values; two consecutive cycles land exactly on target.
        double rawFor40 = (40.0 - 5.0) / 45.0 * 65535.0;
        approx(run(mc, sim(rawFor40), 1), 40.0, 0.05, "slew: ripple passes unattenuated (down)");
        approx(run(mc, sim(65535.0), 1), 50.0, 0.05, "slew: ripple passes unattenuated (up)");
    }

    // ---- Overspeed: sustained spin trips (latched slack); transient flick doesn't ----
    {
        AppConfig cfg = makeBeltConfig(5.0, 50.0);
        cfg.drives[0].encoderCountsPerRev = 1000.0;          // unitsPerRev = 10
        cfg.drives[0].beltMaxTravelRevs   = 0.0;             // isolate OVERSPEED (travel cap
                                                             // would win the race at 3 revs)
        MotionController mc; mc.configure(cfg); mc.setEmergencyStop(false);
        A6Drive simDrive;                                     // sim mode: no PDO ptrs
        A6Drive* drv[1] = { &simDrive };
        tensionBelt(mc);
        MotionOutput out{};
        for (int c = 0; c < 200; ++c) { out = MotionOutput{}; mc.process(sim(0.0), out, drv, 1); }
        approx(out.torques[0], 5.0, 0.01, "overspeed rig: ONLINE at min with sim drive");

        // Transient flick: 2 units/cycle (1200rpm) for 5 cycles (50ms) then stop -> no trip.
        double pos = simDrive.getActualPositionRaw();
        for (int c = 0; c < 5;  ++c) { pos += 2.0; simDrive.setSimPosition(pos);
            out = MotionOutput{}; mc.process(sim(0.0), out, drv, 1); }
        for (int c = 0; c < 30; ++c) { out = MotionOutput{}; mc.process(sim(0.0), out, drv, 1); }
        approx(out.torques[0], 5.0, 0.01, "overspeed: 50ms flick does NOT trip (persistence)");

        // Sustained spin: 1200rpm for 250ms (25 cycles) -> trip -> torque 0, latched.
        for (int c = 0; c < 25; ++c) { pos += 2.0; simDrive.setSimPosition(pos);
            out = MotionOutput{}; mc.process(sim(0.0), out, drv, 1); }
        approx(out.torques[0], 0.0, 1e-9, "overspeed: sustained spin trips -> torque 0");
        for (int c = 0; c < 50; ++c) { out = MotionOutput{}; mc.process(sim(65535.0), out, drv, 1); }
        approx(out.torques[0], 0.0, 1e-9, "overspeed: LATCHED slack (max demand ignored until Tension)");
        check(mc.getMotionStatus().beltGuard[0] == 1, "overspeed: guard=1 published for the card");
        // Option A: unpark must NOT clear a guard-trip latch -- only TensionBelts may.
        mc.startUnpark(drv, 1);
        for (int c = 0; c < 200; ++c) { out = MotionOutput{}; mc.process(sim(0.0), out, drv, 1); }
        approx(out.torques[0], 0.0, 1e-9, "overspeed: unpark does NOT re-tension a tripped belt");
        check(mc.getMotionStatus().beltGuard[0] == 1, "overspeed: guard latch survives unpark");
        tensionBelt(mc);
        for (int c = 0; c < 200; ++c) { out = MotionOutput{}; mc.process(sim(0.0), out, drv, 1); }
        approx(out.torques[0], 5.0, 0.01, "overspeed: explicit Tension re-tensions");
        check(mc.getMotionStatus().beltGuard[0] == 0, "overspeed: guard flag clears on tension-up");
    }

    // ---- Net-travel cap: SLOW continuous wind (below rpm threshold) still trips ----
    {
        AppConfig cfg = makeBeltConfig(5.0, 50.0);
        cfg.drives[0].encoderCountsPerRev = 1000.0;          // unitsPerRev = 10
        MotionController mc; mc.configure(cfg); mc.setEmergencyStop(false);
        A6Drive simDrive; A6Drive* drv[1] = { &simDrive };
        tensionBelt(mc);
        MotionOutput out{};
        for (int c = 0; c < 200; ++c) { out = MotionOutput{}; mc.process(sim(0.0), out, drv, 1); }
        // 0.5 units/cycle = 300rpm -- BELOW the 600rpm threshold (overspeed silent).
        // Travel cap 3 revs = 30 units = 60 cycles of slow continuous winding.
        double pos = simDrive.getActualPositionRaw();
        for (int c = 0; c < 100; ++c) { pos += 0.5; simDrive.setSimPosition(pos);
            out = MotionOutput{}; mc.process(sim(0.0), out, drv, 1); }
        approx(out.torques[0], 0.0, 1e-9, "travel cap: sub-threshold continuous wind trips (latched slack)");
        check(mc.getMotionStatus().beltGuard[0] == 2, "travel cap: guard=2 published for the card");
        // Re-tension re-seeds the reference: hand-pulled spool position is the new zero.
        mc.enqueueCommand({MotionCommand::Type::TensionBelts, -1});
        for (int c = 0; c < 200; ++c) { out = MotionOutput{}; mc.process(sim(0.0), out, drv, 1); }
        approx(out.torques[0], 5.0, 0.01, "travel cap: tension-up re-seeds reference, resumes");
    }

    // ---- Guards armed during BLENDING (spin-up during the tension ramp trips) ----
    {
        AppConfig cfg = makeBeltConfig(5.0, 50.0);
        cfg.drives[0].encoderCountsPerRev = 1000.0;
        AppConfig cfg2 = cfg; cfg2.blendTimeSec = 2.0;       // long blend: trip happens inside it
        MotionController mc; mc.configure(cfg2); mc.setEmergencyStop(false);
        A6Drive simDrive; A6Drive* drv[1] = { &simDrive };
        tensionBelt(mc);
        MotionOutput out{};
        for (int c = 0; c < 15; ++c) { out = MotionOutput{}; mc.process(sim(0.0), out, drv, 1); }  // into UNPARKING/BLENDING
        double pos = simDrive.getActualPositionRaw();
        for (int c = 0; c < 60; ++c) { pos += 2.0; simDrive.setSimPosition(pos);                    // 1200rpm sustained
            out = MotionOutput{}; mc.process(sim(0.0), out, drv, 1); }
        approx(out.torques[0], 0.0, 1e-9, "BLENDING-armed: runaway during tension ramp trips");
    }

    // ---- Velocity fold: master-side CST speed limiter (the Er06.0 fix) ----
    // Drive-side objects (0x607F, C03.47/48) provably don't restrain CST speed on
    // the A6; the fold is THE enforced limit. unitsPerRev=10 @100Hz: 1 unit/cycle
    // = 600rpm. Knee at default 800rpm, band = 200rpm (25%), zero at 1000rpm.
    {
        AppConfig cfg = makeBeltConfig(5.0, 50.0);
        cfg.drives[0].encoderCountsPerRev = 1000.0;          // unitsPerRev = 10
        cfg.drives[0].beltOverspeedRpm    = 0.0;             // isolate the fold (no trip)
        cfg.drives[0].beltMaxTravelRevs   = 0.0;
        MotionController mc; mc.configure(cfg); mc.setEmergencyStop(false);
        A6Drive simDrive; A6Drive* drv[1] = { &simDrive };
        tensionBelt(mc);
        MotionOutput out{};
        for (int c = 0; c < 200; ++c) { out = MotionOutput{}; mc.process(sim(65535.0), out, drv, 1); }
        approx(out.torques[0], 50.0, 0.01, "fold: below knee -> full tension (haptics untouched)");

        // Spin at 900rpm (1.5 units/cycle): knee 800, band 200 -> scale 0.5.
        double pos = simDrive.getActualPositionRaw();
        for (int c = 0; c < 40; ++c) { pos += 1.5; simDrive.setSimPosition(pos);
            out = MotionOutput{}; mc.process(sim(65535.0), out, drv, 1); }
        approx(out.torques[0], 25.0, 1.5, "fold: 900rpm (knee+half band) -> tension halved");

        // Spin at 1200rpm (2 units/cycle): beyond knee+band -> folded to zero.
        for (int c = 0; c < 40; ++c) { pos += 2.0; simDrive.setSimPosition(pos);
            out = MotionOutput{}; mc.process(sim(65535.0), out, drv, 1); }
        approx(out.torques[0], 0.0, 0.5, "fold: beyond knee+band -> tension folded to ~0");

        // Speed recovers -> tension ramps back up under the slew cap (no snap).
        // One cycle at 100Hz allows exactly slew*dt = 3000*0.01 = 30% from the
        // folded 0 -- proves the ramp is slew-governed, not an instant snap.
        out = MotionOutput{}; mc.process(sim(65535.0), out, drv, 1);
        approx(out.torques[0], 30.0, 0.5, "fold: recovery is slew-governed (30%/cycle at 100Hz)");
        for (int c = 0; c < 200; ++c) { out = MotionOutput{}; mc.process(sim(65535.0), out, drv, 1); }
        approx(out.torques[0], 50.0, 0.01, "fold: full tension restored once speed is sane");
    }

    // ---- Relaxer: sustained near-max dwell eases to min; released when demand drops ----
    {
        AppConfig cfg = makeBeltConfig(5.0, 50.0);
        cfg.drives[0].beltRelaxerSec = 2.0;                  // band = 80% of max = 40%
        MotionController mc; onlineBelt(mc, cfg);
        approx(run(mc, sim(65535.0), 150), 50.0, 0.01, "relaxer: <2s at max still full tension");
        approx(run(mc, sim(65535.0), 100), 5.0, 0.01, "relaxer: >2s sustained max -> eased to min");
        check(mc.getMotionStatus().beltGuard[0] == 3, "relaxer: guard=3 (relaxed) published for the card");
        // Demand drops below the release band (70% of max = 35%) -> resumes tracking.
        double rawFor20 = (20.0 - 5.0) / 45.0 * 65535.0;
        approx(run(mc, sim(rawFor20), 10), 20.0, 0.05, "relaxer: released when demand drops");
    }

    // ---- Belt slack/tension (don-doff): belt-scoped, position axes untouched ----
    {
        // Two axes: vertical (index 0, stays PARKED-unhomed) + belt (index 1).
        AppConfig cfg = makeBeltConfig(5.0, 50.0);
        DriveConfig vert;
        vert.slaveIndex = 1; vert.axisType = "linear_vertical"; vert.mode = "csp";
        vert.strokeMm = 100.0; vert.countsPerMm = 100.0;
        cfg.drives[0].slaveIndex = 2;
        cfg.drives.insert(cfg.drives.begin(), vert);
        cfg.numDrives = 2;

        MotionController mc; mc.configure(cfg); mc.setEmergencyStop(false);
        mc.startUnpark();                                   // verticals only (Option A)
        mc.enqueueCommand({MotionCommand::Type::TensionBelts, -1});   // belt onlines via Tension
        MotionOutput out{};
        TelemetryData sd = sim(65535.0); sd.numPositions = 2; sd.positions[1] = 65535.0;
        // Long settle: the vertical must finish slewing to its (constant) target so
        // "untouched" is testable as "a settled tracker stays settled".
        for (int c = 0; c < 3000; ++c) { out = MotionOutput{}; mc.process(sd, out, nullptr, 0); }
        approx(out.torques[1], 50.0, 0.01, "don-doff rig: belt ONLINE at max");
        const double vertPosBefore = out.positions[0];

        mc.enqueueCommand({MotionCommand::Type::SlackBelts, -1});
        for (int c = 0; c < 30; ++c) { out = MotionOutput{}; mc.process(sd, out, nullptr, 0); }
        approx(out.torques[1], 0.0, 1e-9, "SlackBelts -> belt slack (torque 0) despite max demand");
        approx(out.positions[0], vertPosBefore, 1e-6, "SlackBelts -> position axis completely untouched");

        mc.enqueueCommand({MotionCommand::Type::TensionBelts, -1});
        for (int c = 0; c < 300; ++c) { out = MotionOutput{}; mc.process(sd, out, nullptr, 0); }
        approx(out.torques[1], 50.0, 0.01, "TensionBelts -> blends back to live tension");
        approx(out.positions[0], vertPosBefore, 1e-6, "TensionBelts -> position axis still untouched");

        // TensionBelts refused under e-stop (belt stays slack).
        mc.enqueueCommand({MotionCommand::Type::SlackBelts, -1});
        for (int c = 0; c < 30; ++c) { out = MotionOutput{}; mc.process(sd, out, nullptr, 0); }
        mc.setEmergencyStop(true);
        for (int c = 0; c < 5;  ++c) { out = MotionOutput{}; mc.process(sd, out, nullptr, 0); }
        mc.enqueueCommand({MotionCommand::Type::TensionBelts, -1});
        for (int c = 0; c < 30; ++c) { out = MotionOutput{}; mc.process(sd, out, nullptr, 0); }
        approx(out.torques[1], 0.0, 1e-9, "TensionBelts refused while e-stopped (stays slack)");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
