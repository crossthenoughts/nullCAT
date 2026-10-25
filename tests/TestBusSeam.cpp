// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
//
// TestBusSeam - the master and the control loop on a fake bus
// (tests/FakeBus.h, the seam in src/BusTransport.h). What ran only on a
// rig until now:
//   S-1 OP on the fake bus: the drives bound through the real PDO
//       assignment, a frame carries the expected working counter, a drive
//       reads its slave's statusword;
//   S-2 the pump carries out a stop asked from another thread: every
//       enabled slave sees the quick stop within a few frames;
//   S-3 the pump leaves a fault reset for the loop: nothing while it owns
//       the bus, the loop's next frame takes it;
//   S-4 the control loop enables the drives on its real drive path; an
//       e-stop takes them out at once and nothing enables them again while
//       tripped, the input held or released (the path a 10 ms blip on the
//       GPIO input took on 2026-10-25, when the release re-homed the rig);
//   S-5 Stop EtherCAT after a trip: the seat pass's e-stop guard (it
//       enables nothing);
//   S-6 the drive's own quick stop (a hardware e-stop wired to the drive)
//       trips the controller with the drive as its source; its release
//       frees the input, the trip holds;
//   S-7 the seat pass untripped does enable the post on this bus (so
//       S-5's "enables nothing" can fail).
// Windows are generous (a slow emulated arm64 CI runner): a waitFor ends
// as soon as its condition holds.
#include "FakeBus.h"
#include "../src/EtherCATMaster.h"
#include "../src/ControlLoop.h"
#include "../src/MotionController.h"
#include "../src/TelemetryInput.h"
#include "../src/Config.h"
#include "../src/Logging.h"
#include <chrono>
#include <cstdio>
#include <thread>

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const char* what)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    (ok ? g_pass : g_fail)++;
}

template <class F>
static bool waitFor(F done, int ms)
{
    for (int t = 0; t < ms; t += 5)
    {
        if (done()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return done();
}

static void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// A post (position, homes against its lower stop) and a belt (torque).
static AppConfig makeCfg()
{
    DriveConfig post;
    post.slaveIndex          = 1;
    post.name                = "post";
    post.axisType            = "linear_vertical";
    post.mode                = "csp";
    post.strokeMm            = 100.0;
    post.countsPerMm         = 100.0;
    post.homeDirection       = "negative";
    post.homingSpeed         = 50.0;
    post.homingBackoffMm     = 1.5;
    post.homingTorquePct     = 25;
    post.parkMode            = "endstop";
    post.maxVelocityMmS      = 200.0;
    post.maxAccelerationMmS2 = 2000.0;
    post.maxJerkMmS3         = 20000.0;
    post.unparkTimeSec       = 0.1;
    post.parkTimeSec         = 0.1;

    DriveConfig belt;
    belt.slaveIndex          = 2;
    belt.name                = "belt";
    belt.axisType            = "belt";
    belt.mode                = "torque";
    belt.strokeMm            = 100.0;
    belt.countsPerMm         = 100.0;
    belt.maxVelocityMmS      = 200.0;
    belt.maxAccelerationMmS2 = 2000.0;
    belt.maxJerkMmS3         = 20000.0;
    belt.unparkTimeSec       = 0.1;
    belt.parkTimeSec         = 0.1;
    belt.torqueMinPct        = 5.0;
    belt.torqueMaxPct        = 50.0;

    AppConfig cfg;
    cfg.simulationMode      = false;
    cfg.controlLoopHz       = 500;
    cfg.numDrives           = 2;
    cfg.blendTimeSec        = 0.1;
    cfg.blendMaxVelocityMmS = 20.0;
    cfg.drives.push_back(post);
    cfg.drives.push_back(belt);
    return cfg;
}

int main()
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    Logger::instance().setMinLevel(LogLevel::LVL_ERROR);
    const AppConfig cfg = makeCfg();

    FakeBus bus({ false, true });
    bus.with(1, [](FakeBus::Slave& s) { s.hardLo = 0; s.hardHi = 20000; s.pos = 5000; });

    EtherCATMaster master;
    master.applyConfig(cfg);

    // ---- S-1 OP on the fake bus ----
    check(master.attachTestBus(&bus).ok, "S-1 OP on the fake bus");
    check(master.isOperational() && !master.isSimulation() && master.getDriveCount() == 2,
          "S-1 ...two drives, and not simulation (the real drive paths run)");
    {
        const long f0 = bus.frames();
        master.sendReceive();
        check(bus.frames() == f0 + 1, "S-1 sendReceive puts one frame on the bus");
    }
    master.getDrive(0)->updateStatus();
    check(master.getDrive(0)->getState() == DriveState::ReadyToSwitchOn,
          "S-1 a drive reads its slave through the PDO assignment (ready to switch on: the init's 0x0006)");

    // ---- S-2 the pump carries out a stop ----
    // The test thread owns the bus first and enables both drives.
    for (int c = 0; c < 400 && !bus.allEnabled(); ++c)
    {
        master.sendReceive();
        for (int i = 0; i < master.getDriveCount(); ++i) master.getDrive(i)->stepEnableStateMachine();
    }
    check(bus.allEnabled(), "S-2 the drives enable on the fake bus (the DS402 sequence)");
    {
        const long f0 = bus.frames();
        master.startPump();
        check(waitFor([&] { return bus.frames() > f0 + 20; }, 2000), "S-2 the pump runs frames");
        check(bus.allEnabled(), "S-2 ...holding the drives enabled");
        master.requestStopAll();
        check(waitFor([&] { return !bus.anyEnabled(); }, 2000),
              "S-2 a stop asked from another thread while the pump owns the bus reaches every drive");
        int qs = 0;
        for (int s = 1; s <= 2; ++s) bus.with(s, [&](FakeBus::Slave& x) { qs += x.quickStops; });
        check(qs == 2, "S-2 ...as a quick stop (0x0003) on each");
        master.stopPump();
    }

    // ---- S-3 the pump leaves a fault reset for the loop ----
    {
        bus.with(1, [](FakeBus::Slave& s) { s.state = FakeBus::St::Fault; s.faultCode = 0x2312; });
        const long f0 = bus.frames();
        master.startPump();
        check(waitFor([&] { return bus.frames() > f0 + 10; }, 2000), "S-3 the pump runs frames");
        master.requestFaultResetAll();
        const long f1 = bus.frames();
        sleepMs(150);
        check(bus.frames() > f1 + 10 && !master.getDrive(0)->isFaultResetPending() && !master.getDrive(1)->isFaultResetPending(),
              "S-3 the pump, still running frames, leaves a fault reset alone (it waits for the loop)");
        master.stopPump();
        master.sendReceive();
        check(master.getDrive(0)->isFaultResetPending() && master.getDrive(1)->isFaultResetPending(),
              "S-3 ...the loop's next frame takes it");
    }

    // ---- S-4 the control loop: an e-stop, and nothing enables again ----
    MotionController motion;
    motion.configure(cfg);
    TelemetryInput telemetry;   // no socket: no telemetry
    ControlLoop loop;
    loop.setComponents(&master, &telemetry, &motion);
    loop.setConfig(cfg);
    // Start clean: a fresh OP (the fault and its pending reset above gone).
    bus.with(1, [](FakeBus::Slave& s) { s.state = FakeBus::St::SwitchOnDisabled; s.faultCode = 0; });
    check(master.attachTestBus(&bus).ok, "S-4 OP again on the fake bus");
    motion.configure(cfg);
    check(loop.start(), "S-4 the control loop starts on the fake bus");
    check(waitFor([&] { return bus.state(2) == FakeBus::St::OperationEnabled; }, 3000),
          "S-4 the loop enables the belt's drive on its real drive path");
    motion.setEmergencyStop(true, MotionController::EstopSource::Web);
    master.requestStopAll();
    check(waitFor([&] { return !bus.anyEnabled(); }, 2000), "S-4 an e-stop takes every drive out");
    bus.forbidEnable(true);
    const long fw = bus.frames();
    sleepMs(800);
    motion.setEmergencyStop(false, MotionController::EstopSource::Web);   // the input released
    sleepMs(800);
    check(loop.isRunning() && bus.frames() > fw + 100, "S-4 the loop ran frames all through the window");
    check(bus.forbiddenEnabledFrames() == 0,
          "S-4 ...and nothing enables a drive while tripped, the input held or released");
    check(motion.isEstopTripped() && motion.isEmergencyStop(), "S-4 the trip holds after the release");
    loop.stop();
    loop.waitForStop();

    // ---- S-5 Stop EtherCAT after a trip: no seat ----
    {
        const int en = bus.enables();
        const auto t0 = std::chrono::steady_clock::now();
        loop.seatThenStop();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
        check(bus.enables() == en && bus.forbiddenEnabledFrames() == 0,
              "S-5 Stop EtherCAT after a trip: the seat pass enables nothing");
        check(ms < 1000, "S-5 ...and returns at once");
    }
    master.shutdown();
    bus.forbidEnable(false);

    // ---- S-6 the drive's own quick stop trips the controller ----
    {
        check(master.attachTestBus(&bus).ok && motion.clearEmergencyTrip(),
              "S-6 (setup) OP again, the trip cleared as a successful initialize clears it");
        motion.configure(cfg);
        check(loop.start(), "S-6 the loop starts again");
        check(waitFor([&] { return bus.state(2) == FakeBus::St::OperationEnabled; }, 3000),
              "S-6 the belt's drive enables again after the re-initialize");
        bus.with(2, [](FakeBus::Slave& s) { s.holdQuickStop = true; });
        check(waitFor([&] { return motion.isEstopTripped(); }, 2000)
              && motion.estopTripSource() == MotionController::EstopSource::Drive,
              "S-6 the drive's own quick stop trips the controller (the drive as its source)");
        bus.with(2, [](FakeBus::Slave& s) { s.holdQuickStop = false; });
        check(waitFor([&] { return !motion.isEstopInputHeld(); }, 2000) && motion.isEstopTripped(),
              "S-6 ...its release frees the input; the trip holds");
        loop.stop();
        loop.waitForStop();
        master.shutdown();
    }

    // ---- S-7 the seat pass, untripped, does run on this bus ----
    {
        check(master.attachTestBus(&bus).ok && motion.clearEmergencyTrip(), "S-7 (setup) OP, not tripped");
        motion.configure(cfg);
        const int en = bus.enables();
        loop.seatThenStop();
        check(bus.enables() > en, "S-7 untripped, the seat pass enables the post (S-5's check can fail)");
        master.shutdown();
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
