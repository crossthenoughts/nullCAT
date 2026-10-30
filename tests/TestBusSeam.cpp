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
//       S-5's "enables nothing" can fail);
//   S-8 the pump's first frame a period after the loop's last; the pump
//       and its OP bridge, held up after a frame, drop the frames they
//       missed instead of sending them back to back (the Pi's drives left
//       OP that way at the hand-over to the loop), and the report counts
//       them;
//   S-9 replies short of the expected working counter while the pump
//       holds the bus (the drives out of OP), and frames with no reply,
//       each in its report;
//   S-10 the schedule rule itself: late by less than a period less the
//       least gap keeps it, later drops whole periods on the same grid;
//   S-11 the bus never falls quiet across a hand-over to the pump: the
//       gaps from the loop's last frame to the bridge, inside it, to the
//       pump and inside it are all on the pump's report and short; on
//       Linux the bridge and the pump run on one thread for the bus
//       session (the dispatch thread), nothing created at the hand-over
//       (a new thread's stack took 13 ms to lock on the Pi, the drives'
//       sync loss).
// Windows are generous (a slow emulated arm64 CI runner): a waitFor ends
// as soon as its condition holds.
#include "FakeBus.h"
#include "../src/EtherCATMaster.h"
#include "../src/ControlLoop.h"
#include "../src/MotionController.h"
#include "../src/TelemetryInput.h"
#include "../src/Config.h"
#include "../src/Logging.h"
#include "../src/PlatformRT.h"
#include <chrono>
#include <cstdio>
#include <thread>
#include <vector>

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

    // ---- S-8 the pump held up: no frames back to back after it ----
    // The Pi's drives left OP with a sync error at the hand-over to the
    // loop (2026-10-23, 2026-10-29 twice): the pump, late, caught up by
    // sending its missed frames back to back.
    {
        // The frame after a hold-up: at least half the least gap (1 ms at
        // this config's 500 Hz) after it, where catching up sends it within
        // microseconds; slowness only widens it.
        const auto afterHoldUp = [](const std::vector<int64_t>& t) -> int64_t
        {
            for (size_t i = 1; i + 1 < t.size(); ++i)
                if (t[i] - t[i - 1] >= 8000000) return t[i + 1] - t[i];
            return -1;
        };
        check(master.attachTestBus(&bus).ok, "S-8 (setup) OP");
        bus.recordFrameTimes(true);
        master.sendReceive();   // the loop's last frame
        master.startPump();
        check(waitFor([&] { return bus.frameTimes().size() >= 3; }, 2000), "S-8 the pump takes over");
        {
            const std::vector<int64_t> t = bus.frameTimes();
            check(t.size() >= 2 && t[1] - t[0] >= 1000000,
                  "S-8 its first frame comes a period after the loop's last, not straight after it");
        }
        const long f0 = bus.frames();
        check(waitFor([&] { return bus.frames() > f0 + 10; }, 2000), "S-8 the pump runs frames");
        bus.recordFrameTimes(true);
        bus.holdUpAfterNextFrame(9);   // four and a half of its 2 ms periods
        check(waitFor([&] { return bus.frameTimes().size() >= 6; }, 3000), "S-8 ...and goes on after a hold-up");
        master.stopPump();
        check(afterHoldUp(bus.frameTimes()) >= 500000,
              "S-8 after a hold-up the pump drops the frames it missed: its next one is not straight after");
        check(master.lastPumpReport().missedWakes >= 1 && master.lastPumpReport().droppedFrames >= 3,
              "S-8 ...and its report counts the late wake and the frames dropped");
        // The OP bridge (the pump's first frames, on the dispatch thread) the same.
        bus.recordFrameTimes(true);
        bus.holdUpAfterNextFrame(9);
        master.startPump();
        check(waitFor([&] { return bus.frameTimes().size() >= 6; }, 3000), "S-8 the bridge goes on after a hold-up");
        master.stopPump();
        check(afterHoldUp(bus.frameTimes()) >= 500000,
              "S-8 ...and drops the frames it missed: its next one is not straight after");
        bus.recordFrameTimes(false);
    }

    // ---- S-11 no quiet bus at a hand-over, nothing born there ----
    {
        check(master.attachTestBus(&bus).ok, "S-11 (setup) OP");
        bus.recordFrameTimes(true);
        master.sendReceive();   // the loop's last frame
        master.startPump();
        check(waitFor([&] { return bus.frameTimes().size() >= 14; }, 4000), "S-11 the bridge (10 frames) and the pump run");
        master.stopPump();
        const EtherCATMaster::PumpReport r = master.lastPumpReport();
        std::printf("  S-11 gaps (ms): to the bridge %.2f, in it %.2f, to the pump %.2f, in it %.2f; bridge short %d\n",
                    r.gapToBridgeUs / 1000.0, r.bridgeMaxGapUs / 1000.0, r.gapToPumpUs / 1000.0, r.pumpMaxGapUs / 1000.0, r.bridgeShort);
        // About 2 ms each (the bridge starts a period on); 15 ms is the
        // Pi's fault and a slow runner's worst, so the bound sits between.
        check(r.gapToBridgeUs >= 0.0 && r.gapToBridgeUs < 15000.0 && r.bridgeMaxGapUs >= 0.0 && r.bridgeMaxGapUs < 15000.0,
              "S-11 the bridge follows the loop's last frame and keeps its cadence (every gap under 15 ms)");
        check(r.gapToPumpUs >= 0.0 && r.gapToPumpUs < 15000.0 && r.pumpMaxGapUs >= 0.0 && r.pumpMaxGapUs < 15000.0,
              "S-11 the pump follows the bridge and keeps its cadence (every gap under 15 ms)");
        check(r.bridgeShort == 0, "S-11 the bridge's replies in full, on record");
        const std::vector<size_t> who = bus.frameThreads();
#ifndef _WIN32
        // Linux: one thread for the bridge and the pump, and the same one
        // next time: the dispatch thread, for the bus session.
        bool one = who.size() >= 14;
        for (size_t i = 2; i < who.size() && one; ++i) one = (who[i] == who[1]);
        check(one && who.size() >= 2 && who[1] != who[0], "S-11 Linux: the bridge and the pump run on one thread, not the loop's, nothing created at the hand-over");
        bus.recordFrameTimes(true);
        master.startPump();
        check(waitFor([&] { return bus.frameTimes().size() >= 3; }, 3000), "S-11 Linux: a second pump runs");
        master.stopPump();
        const std::vector<size_t> again = bus.frameThreads();
        check(!again.empty() && who.size() >= 2 && again.back() == who[1], "S-11 Linux: ...on the same thread as the first (the dispatch thread lives for the session)");
#else
        // Windows: the bridge on the dispatch thread, the pump on its own
        // (the Npcap warm-up), as before.
        check(who.size() >= 14 && who[1] != who[0] && who.back() != who[1], "S-11 Windows: the bridge and the pump on their own threads, as before");
#endif
        bus.recordFrameTimes(false);
        master.shutdown();
        check(master.attachTestBus(&bus).ok, "S-11 (teardown) OP again for the sections that follow");
    }

    // ---- S-9 the drives leaving OP while the pump holds them are on record ----
    {
        const EtherCATMaster::PumpReport clean = master.lastPumpReport();
        check(clean.shortFromSec < 0.0 && clean.shortReplies == 0 && clean.heldSec > 0.0,
              "S-9 a pump with every reply in full reports none short");
        const long f0 = bus.frames();
        master.startPump();
        check(waitFor([&] { return bus.frames() > f0 + 10; }, 2000), "S-9 the pump runs frames");
        bus.replyWkc(2);   // both slaves out of OP: their inputs only
        const long f1 = bus.frames();
        check(waitFor([&] { return bus.frames() > f1 + 10; }, 2000), "S-9 ...the replies short");
        bus.replyWkc(-1);  // then no reply at all
        const long f2 = bus.frames();
        check(waitFor([&] { return bus.frames() > f2 + 10; }, 2000), "S-9 ...then none");
        master.stopPump();
        bus.replyWkc(FakeBus::kFullReplies);
        const EtherCATMaster::PumpReport r = master.lastPumpReport();
        check(r.shortFromSec > 0.0 && r.shortWkc == 2 && r.shortReplies >= 10,
              "S-9 the pump's report has when its replies went short, their WKC and how many");
        check(r.lostFrames >= 10 && r.lostFromSec > r.shortFromSec && !r.crashed,
              "S-9 ...and the frames with no reply apart from them (a lost frame is not a drive out of OP)");
        master.shutdown();
    }

    // ---- S-10 the pump's schedule rule (PlatformRT::skipMissedPeriods) ----
    {
        const int64_t p = PlatformRT::periodCounts(2000.0), gap = p / 4;
        const PlatformRT::Timestamp t0 = PlatformRT::now();
        const auto at = [&t0](int64_t counts) { PlatformRT::Timestamp t = t0; PlatformRT::addCounts(t, counts); return t; };
        PlatformRT::Timestamp d = t0;
        check(PlatformRT::skipMissedPeriods(d, at(p / 2), p, gap) == 0 && PlatformRT::elapsedMicros(d, t0) == 0.0,
              "S-10 a wake late by less than a period less the gap keeps the schedule");
        d = t0;
        check(PlatformRT::skipMissedPeriods(d, at(p * 4 / 5), p, gap) == 1 && PlatformRT::elapsedMicros(d, at(p)) == 0.0,
              "S-10 a wake within the gap of the next frame drops that one, on the same grid");
        d = t0;
        check(PlatformRT::skipMissedPeriods(d, at(3 * p), p, gap) == 3 && PlatformRT::elapsedMicros(d, at(3 * p)) == 0.0,
              "S-10 three periods late drops three, the grid kept (no catching up, no new phase)");
        PlatformRT::Timestamp later = t0;
        PlatformRT::addCounts(later, 3 * PlatformRT::periodCounts(1e6));   // 3 s: past a 32-bit long of ns
        check(PlatformRT::elapsedMicros(later, t0) == 3e6, "S-10 a deadline moves on by seconds exactly");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
