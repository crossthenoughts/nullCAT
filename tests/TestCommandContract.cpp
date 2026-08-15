// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestCommandContract.cpp - pins Docs/COMMAND_CONTRACT.md at the engine layer.
//
// The escaped-bug class in this project is CONTRACT bugs (torque mode shipped
// broken for its entire life because nothing asserted "select torque => CST").
// This suite asserts, for every enqueued MotionCommand, the OBSERVABLE state
// change the contract table promises - including the silent engine-layer
// refusals (e.g. TensionBelts under e-stop) that the HTTP layer cannot see.
//
// Same harness style as TestTorquePath: drives=nullptr, fresh MotionOutput per
// cycle, no SOEM/NIC/Qt. Commands go through enqueueCommand() exactly as the
// WebServer does - never by calling the handlers directly.
//
//   ctest -R CommandContract
// ============================================================
#include "MotionController.h"
#include "TelemetryInput.h"
#include "Config.h"

#include <cstdio>
#include <cmath>

static int g_fail = 0, g_pass = 0, g_pending = 0;
static void check(bool ok, const char* name)
{
    if (ok) { ++g_pass; std::printf("[PASS] %s\n", name); }
    else    { ++g_fail; std::printf("[FAIL] %s\n", name); }
}
// A contract row whose intended behavior is AWAITING TIM'S RULING (found
// drifted on first run, Jul 20 2026): reported, never fails the build.
// Once ruled, promote to check() (and fix engine or table accordingly).
static void pending(bool matchesProposedContract, const char* name)
{
    ++g_pending;
    std::printf("[PENDING] %s -- engine currently %s the proposed contract\n",
                name, matchesProposedContract ? "MATCHES" : "VIOLATES");
}

// One belt (torque) axis + one vertical (CSP) axis - the contract rows differ
// per axis kind (belts never home; verticals never take belt commands).
static AppConfig makeRigConfig()
{
    DriveConfig vert;
    vert.slaveIndex   = 1;
    vert.axisType     = "linear_vertical";
    vert.mode         = "csp";
    vert.strokeMm     = 100.0;
    vert.countsPerMm  = 100.0;
    vert.maxVelocityMmS = 200.0;
    vert.maxAccelerationMmS2 = 2000.0;
    vert.maxJerkMmS3  = 20000.0;
    vert.unparkTimeSec = 0.1;
    vert.parkTimeSec   = 0.1;
    vert.onlineHoldTimeoutSec = 5.0;

    DriveConfig belt = vert;
    belt.slaveIndex   = 2;
    belt.axisType     = "belt";
    belt.mode         = "torque";
    belt.torqueMinPct = 5.0;
    belt.torqueMaxPct = 50.0;

    AppConfig cfg;
    cfg.controlLoopHz       = 100;
    cfg.numDrives           = 2;
    cfg.blendTimeSec        = 0.1;
    cfg.blendMaxVelocityMmS = 20.0;
    cfg.drives.push_back(vert);
    cfg.drives.push_back(belt);
    return cfg;
}

static TelemetryData sim(double raw, bool valid = true)
{
    TelemetryData sd{};
    sd.valid        = valid;
    sd.numPositions = 2;
    sd.positions[0] = 0.5;   // vertical mid-stroke
    sd.positions[1] = raw;   // belt demand
    sd.packetType   = TelemetryPacketType::Motion;
    return sd;
}

static MotionOutput runCycles(MotionController& mc, const TelemetryData& sd, int n)
{
    MotionOutput out{};
    for (int i = 0; i < n; ++i) { out = MotionOutput{}; mc.process(sd, out, nullptr, 0); }
    return out;
}

static bool cmd(MotionController& mc, MotionCommand::Type t)
{
    MotionCommand c;
    c.type = t;
    return mc.enqueueCommand(c);
}

int main()
{
    const int BELT = 1;   // axis index of the torque axis in makeRigConfig()

    // ---- TensionBelts / SlackBelts pair ------------------------------------
    {
        MotionController mc;
        mc.configure(makeRigConfig());
        runCycles(mc, sim(0.0), 50);   // settle

        check(cmd(mc, MotionCommand::Type::TensionBelts), "TensionBelts enqueues");
        MotionOutput out = runCycles(mc, sim(0.3), 400);
        check(out.torques[BELT] > 0.0, "TensionBelts: belt commands torque > 0");

        check(cmd(mc, MotionCommand::Type::SlackBelts), "SlackBelts enqueues");
        out = runCycles(mc, sim(0.3), 400);
        check(out.torques[BELT] == 0.0, "SlackBelts: belt torque commands 0");
        check(mc.getAxisState(BELT) == AxisMotionState::PARKED
              || mc.getAxisState(BELT) == AxisMotionState::PARKING,
              "SlackBelts: belt axis parked/parking");

        // Idempotency: repeating the achieved command changes nothing.
        cmd(mc, MotionCommand::Type::SlackBelts);
        out = runCycles(mc, sim(0.3), 100);
        check(out.torques[BELT] == 0.0, "SlackBelts is idempotent (still 0)");
    }

    // ---- Contract row: TensionBelts REFUSED under e-stop (silent) ----------
    {
        MotionController mc;
        mc.configure(makeRigConfig());
        runCycles(mc, sim(0.0), 50);
        cmd(mc, MotionCommand::Type::SlackBelts);
        runCycles(mc, sim(0.0), 200);

        mc.setEmergencyStop(true);
        runCycles(mc, sim(0.0), 50);
        cmd(mc, MotionCommand::Type::TensionBelts);
        MotionOutput out = runCycles(mc, sim(0.5), 400);
        check(out.torques[BELT] == 0.0,
              "TensionBelts under e-stop: refused, torque stays 0");
        check(mc.getAxisState(BELT) != AxisMotionState::ONLINE
              && mc.getAxisState(BELT) != AxisMotionState::BLENDING,
              "TensionBelts under e-stop: axis never goes online");

        // RULED Option A (Jul 20): release restores PERMISSION, not state --
        // the belt stays slack-parked until an explicit TensionBelts.
        mc.setEmergencyStop(false);
        out = runCycles(mc, sim(0.5), 400);
        check(out.torques[BELT] == 0.0,
              "e-stop release alone does not re-tension (explicit command required)");

        // And the same command after release IS honored.
        cmd(mc, MotionCommand::Type::TensionBelts);
        out = runCycles(mc, sim(0.5), 400);
        check(out.torques[BELT] > 0.0,
              "TensionBelts after e-stop release: honored");
    }

    // ---- E-stop always honored, from any state ------------------------------
    {
        MotionController mc;
        mc.configure(makeRigConfig());
        runCycles(mc, sim(0.0), 50);
        cmd(mc, MotionCommand::Type::TensionBelts);
        runCycles(mc, sim(0.5), 400);

        mc.setEmergencyStop(true);
        MotionOutput out = runCycles(mc, sim(0.9), 400);
        check(out.torques[BELT] == 0.0, "e-stop: belt torque forced to 0");
        check(out.positions[0] == out.positions[0],   // finite (not NaN)
              "e-stop: vertical output stays finite");
    }

    // ---- StartPark / StartUnpark pair (belt axis; verticals need homing) ----
    {
        MotionController mc;
        mc.configure(makeRigConfig());
        runCycles(mc, sim(0.0), 50);
        cmd(mc, MotionCommand::Type::TensionBelts);
        runCycles(mc, sim(0.3), 400);

        check(cmd(mc, MotionCommand::Type::StartPark), "StartPark enqueues");
        MotionOutput out = runCycles(mc, sim(0.3), 600);
        check(out.torques[BELT] == 0.0, "StartPark: belt eased to 0");
        check(mc.getAxisState(BELT) == AxisMotionState::PARKED,
              "StartPark: belt PARKED");

        // Idempotent: parking a parked rig is a no-op, never an error.
        check(cmd(mc, MotionCommand::Type::StartPark), "StartPark re-enqueues when parked");
        out = runCycles(mc, sim(0.3), 100);
        check(mc.getAxisState(BELT) == AxisMotionState::PARKED,
              "StartPark idempotent: still PARKED");
    }

    // ---- StartHoming: belt axes never home (contract row) -------------------
    {
        MotionController mc;
        mc.configure(makeRigConfig());
        runCycles(mc, sim(0.0), 50);
        AxisMotionState beltBefore = mc.getAxisState(BELT);
        cmd(mc, MotionCommand::Type::StartHoming);
        runCycles(mc, sim(0.0), 100);
        // RULED Option A (Jul 20): the homing pipeline's auto-unpark excludes
        // torque axes -- /api/home can never tension a belt.
        check(mc.getAxisState(BELT) == beltBefore,
              "StartHoming: belt axis state unchanged (belts never home)");
    }

    std::printf("\n%d passed, %d failed, %d pending ruling\n", g_pass, g_fail, g_pending);
    return g_fail ? 1 : 0;
}
