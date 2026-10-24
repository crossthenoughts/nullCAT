// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestPumpLifecycle.cpp  (Build 222 Phase 3)
//
// Tests for EtherCATMaster pump dispatcher lifecycle.
// All tests run in SIMULATION MODE - no hardware required.
// Covers:
//   PL-1  5 sequential startPump/stopPump cycles - no crash, no deadlock
//   PL-2  startPump() then shutdown() - no deadlock, isInitialized() = false
//   PL-3  shutdown() + re-initialize() - dispatch thread restarts, pump works
//   PL-4  Multiple startPump() posts while pump already active - no double-launch
// ============================================================

#include <QtTest>
#include <chrono>
#include <thread>

#include "../src/EtherCATMaster.h"
#include "../src/Config.h"
#include "../src/Logging.h"

static AppConfig makeTwoDriveConfig()
{
    DriveConfig d1;
    d1.slaveIndex          = 1;
    d1.axisType            = "linear_vertical";
    d1.strokeMm            = 100.0;
    d1.homingSpeed      = 10.0;
    d1.homingBackoffMm     = 1.5;
    d1.homingTorquePct     = 25;
    d1.homeDirection       = "negative";
    d1.parkMode            = "endstop";
    d1.maxVelocityMmS      = 200.0;
    d1.maxAccelerationMmS2 = 2000.0;
    d1.maxJerkMmS3         = 20000.0;
    d1.countsPerMm         = 100.0;
    d1.ballscrewPitch      = 5.0;

    DriveConfig d2 = d1;
    d2.slaveIndex = 2;

    AppConfig cfg;
    cfg.numDrives     = 2;
    cfg.controlLoopHz = 500;
    cfg.dcSyncOffsetNs = 125000;
    cfg.pdoWatchdogMs  = 100;
    cfg.drives = {d1, d2};
    for (auto& d : cfg.drives) d.followingErrorWindowMm = 30.0;   // per-axis (rig) since Slice 2
    return cfg;
}

class TestPumpLifecycle : public QObject
{
    Q_OBJECT

private Q_SLOTS:

    void initTestCase()
    {
        Logger::instance().init("", false);
    }

    // PL-1: 5 sequential startPump/stopPump cycles - dispatch thread handles all without crash.
    // In sim mode startPumpBody() is a no-op, so this tests the dispatch CV cycle itself.
    void test_PL1_sequentialStartStop()
    {
        EtherCATMaster master;
        master.setSimulationMode(true);
        master.applyConfig(makeTwoDriveConfig());
        QVERIFY(master.initializeAndEnterOp("sim"));

        for (int i = 0; i < 5; ++i)
        {
            master.startPump();
            // Give dispatch thread time to wake and process
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            master.stopPump();
        }

        // No crash reaching here is the pass condition.
        QVERIFY(master.isInitialized());
    }

    // PL-2: startPump() then shutdown() - no deadlock, master goes to uninitialized.
    void test_PL2_startPumpThenShutdown()
    {
        EtherCATMaster master;
        master.setSimulationMode(true);
        master.applyConfig(makeTwoDriveConfig());
        QVERIFY(master.initializeAndEnterOp("sim"));

        master.startPump();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        master.shutdown();  // must not deadlock

        QVERIFY(!master.isInitialized());
        QVERIFY(!master.isOperational());
    }

    // PL-3: dispatch thread restart path.
    // init → shutdown (joins first dispatch thread) → re-init (must restart dispatch
    // thread via the !joinable() path in initialize()) → startPump() → second shutdown()
    // (joins the re-created dispatch thread - this join is the observable that proves
    // startPumpDispatchThread() fired in initialize()).
    // In sim mode startPumpBody() is a no-op, so m_pumpActive is never set;
    // the second shutdown()'s join is the only way to verify the thread was alive.
    void test_PL3_shutdownAndReinitPump()
    {
        EtherCATMaster master;
        master.setSimulationMode(true);
        master.applyConfig(makeTwoDriveConfig());
        QVERIFY(master.initializeAndEnterOp("sim"));

        // First shutdown - dispatch thread joined here
        master.shutdown();
        QVERIFY(!master.isInitialized());

        // Re-init - dispatch thread must be restarted by initialize()
        QVERIFY(master.initializeAndEnterOp("sim"));
        QVERIFY(master.isOperational());

        // Post a pump request to the restarted dispatch thread
        master.startPump();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // Second shutdown - joining the re-created dispatch thread is the observable
        // that initialize() correctly restarted it via startPumpDispatchThread().
        master.shutdown();
        QVERIFY(!master.isInitialized());
        QVERIFY(!master.isOperational());
    }

    // PL-4: Multiple startPump() calls while pump already active - no double-launch.
    // hasPumpCrashed() must remain false (sim no-op can't crash, but active flag
    // must not be set twice causing an orphaned thread).
    void test_PL4_multipleStartPumpWhileActive()
    {
        EtherCATMaster master;
        master.setSimulationMode(true);
        master.applyConfig(makeTwoDriveConfig());
        QVERIFY(master.initializeAndEnterOp("sim"));

        // Call startPump() 10 times in rapid succession
        for (int i = 0; i < 10; ++i)
            master.startPump();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        QVERIFY(!master.hasPumpCrashed());
        QVERIFY(master.isInitialized());
    }

    // PL-5: a stop asked from another thread (the web server, the GPIO
    // panel) waits for the bus owner and goes out with its next frame:
    // sendReceive(), the RT loop's one way to the bus, carries it out.
    void test_PL5_stopRequestCarriedOutByTheOwner()
    {
        // A fake process image for each drive (sim drives have none), so
        // what the owner writes can be read back; declared first, so it
        // outlives the master that points into it.
        struct Slot { uint16_t cw = 0x000F; int8_t mode = 8; int32_t target = 0; uint16_t sw = 0x0237; int8_t shown = 8; int32_t actual = 0; };
        Slot image[2];
        EtherCATMaster master;
        master.setSimulationMode(true);
        master.applyConfig(makeTwoDriveConfig());
        QVERIFY(master.initializeAndEnterOp("sim"));
        QCOMPARE(master.getDriveCount(), 2);
        for (int i = 0; i < 2; ++i)
            master.getDrive(i)->setPDOPointers(&image[i].cw, &image[i].mode, &image[i].target,
                                               &image[i].sw, &image[i].shown, &image[i].actual);
        QVERIFY(!master.stopRequestPending());
        std::thread asker([&] { master.requestStopAll(); master.requestFaultResetAll(); });
        asker.join();
        QVERIFY2(master.stopRequestPending() && master.faultResetRequestPending(), "the asking thread carried nothing out itself");
        QVERIFY2(image[0].cw == 0x000F && image[1].cw == 0x000F, "...and wrote no controlword");
        QVERIFY2(!master.getDrive(0)->isFaultResetPending(), "...nor started a fault reset");
        master.sendReceive();
        QVERIFY2(!master.stopRequestPending() && !master.faultResetRequestPending(), "the owner's next frame took both");
        QVERIFY2(image[0].cw == 0x0003 && image[1].cw == 0x0003, "...disabled every drive (A6Drive::disableOperation)");
        QVERIFY2(master.getDrive(0)->isFaultResetPending() && master.getDrive(1)->isFaultResetPending(),
                 "...and started the fault reset on every drive");
        image[0].cw = image[1].cw = 0x000F;
        master.sendReceive();
        QVERIFY2(image[0].cw == 0x000F && image[1].cw == 0x000F, "a stop is carried out once");
        master.requestStopAll();
        master.shutdown();
        QVERIFY2(!master.stopRequestPending(), "shutdown disables every drive itself: nothing left over");
        for (int i = 0; i < 2; ++i)
            if (A6Drive* d = master.getDrive(i)) d->setPDOPointers(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    }
};

QTEST_MAIN(TestPumpLifecycle)
#include "TestPumpLifecycle.moc"
