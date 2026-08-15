// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestB222E.cpp  (Build 222E)
//
// Unit tests for threading audit fixes C1 and C3.
// All tests run in SIMULATION MODE - no hardware required.
//
// Tests:
//   E-1  C1: stopPump() after rapid startPump() - pump is not active on return
//   E-2  C1: concurrent startPump/stopPump from two threads - no deadlock
//   E-3  C3: setRtLoopActive(true) blocks startPumpBody() - isPumpActive() stays false
//   E-4  C3: setRtLoopActive(false) after block - startPump() succeeds (sim no-op, dispatch clears)
// ============================================================

#include <QtTest>
#include <chrono>
#include <thread>
#include <atomic>

#include "../src/EtherCATMaster.h"
#include "../src/Config.h"
#include "../src/Logging.h"

static AppConfig makeSimConfig()
{
    DriveConfig d1;
    d1.slaveIndex          = 1;
    d1.axisType            = "linear_vertical";
    d1.strokeMm            = 100.0;
    d1.homingSpeedMmS      = 10.0;
    d1.homingBackoffMm     = 1.5;
    d1.homingTorquePct     = 25;
    d1.homeDirection       = "negative";
    d1.homeMode            = "endstop";
    d1.maxVelocityMmS      = 200.0;
    d1.maxAccelerationMmS2 = 2000.0;
    d1.maxJerkMmS3         = 20000.0;
    d1.countsPerMm         = 100.0;
    d1.ballscrewPitch      = 5.0;

    AppConfig cfg;
    cfg.numDrives     = 1;
    cfg.controlLoopHz = 500;
    cfg.dcSyncOffsetNs = 125000;
    cfg.pdoWatchdogMs  = 100;
    cfg.drives = {d1};
    cfg.drives[0].followingErrorWindowMm = 30.0;   // per-axis (rig) since Slice 2
    return cfg;
}

class TestB222E : public QObject
{
    Q_OBJECT

private Q_SLOTS:

    void initTestCase()
    {
        Logger::instance().init("", false);
    }

    // E-1: C1 - stopPump() called immediately after startPump() must not leave
    // pump active. In sim mode startPumpBody() is a no-op, so the dispatch thread
    // wakes, calls startPumpBody() (returns immediately), clears m_dispatchBusy.
    // stopPump() must spin until m_dispatchBusy clears, then observe m_pumpActive=false.
    void test_E1_stopPumpWaitsForDispatch()
    {
        EtherCATMaster master;
        master.setSimulationMode(true);
        master.applyConfig(makeSimConfig());
        QVERIFY(master.initializeAndEnterOp("sim"));

        for (int i = 0; i < 20; ++i)
        {
            master.startPump();
            master.stopPump();  // must not return while dispatch is mid-flight
            // After stopPump() returns, pump must not be active.
            QVERIFY2(!master.isPumpActive(),
                qPrintable(QString("Iteration %1: isPumpActive() true after stopPump()").arg(i)));
        }

        master.shutdown();
    }

    // E-2: C1 - concurrent startPump/stopPump from two threads must not deadlock
    // and must not leave the master in an inconsistent state.
    void test_E2_concurrentStartStop()
    {
        EtherCATMaster master;
        master.setSimulationMode(true);
        master.applyConfig(makeSimConfig());
        QVERIFY(master.initializeAndEnterOp("sim"));

        std::atomic<bool> done{false};

        // T-start: hammers startPump() repeatedly
        std::thread tStart([&]() {
            for (int i = 0; i < 100 && !done.load(); ++i)
            {
                master.startPump();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });

        // T-stop: hammers stopPump() repeatedly
        std::thread tStop([&]() {
            for (int i = 0; i < 100 && !done.load(); ++i)
            {
                master.stopPump();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });

        tStart.join();
        tStop.join();
        done.store(true);

        // Final state: master still initialized, no crash
        QVERIFY(master.isInitialized());
        QVERIFY(!master.hasPumpCrashed());

        master.shutdown();
    }

    // E-3: C3 - setRtLoopActive(true) prevents startPumpBody() from launching T5.
    // In sim mode startPumpBody() bails at m_rtLoopActive check (before sim check),
    // so m_dispatchBusy is set by startPump() and cleared by T4 quickly.
    // isPumpActive() must remain false.
    void test_E3_rtLoopActiveBlocksPump()
    {
        EtherCATMaster master;
        master.setSimulationMode(true);
        master.applyConfig(makeSimConfig());
        QVERIFY(master.initializeAndEnterOp("sim"));

        master.setRtLoopActive(true);

        master.startPump();

        // Give dispatch thread time to wake, enter startPumpBody(), bail, clear m_dispatchBusy
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Pump must not have started
        QVERIFY2(!master.isPumpActive(), "isPumpActive() true after setRtLoopActive(true) guard");

        master.setRtLoopActive(false);
        master.shutdown();
    }

    // E-4: C3 - after clearing setRtLoopActive(false), startPump() goes through
    // normally (sim no-op, but dispatch cycle completes and m_dispatchBusy clears).
    // stopPump() must return cleanly.
    void test_E4_rtLoopActiveReleaseAllowsPump()
    {
        EtherCATMaster master;
        master.setSimulationMode(true);
        master.applyConfig(makeSimConfig());
        QVERIFY(master.initializeAndEnterOp("sim"));

        master.setRtLoopActive(true);
        master.startPump();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        QVERIFY(!master.isPumpActive());

        // Now release the guard and start again
        master.setRtLoopActive(false);
        master.startPump();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        // stopPump() must not deadlock (m_dispatchBusy cleared by T4)
        master.stopPump();
        QVERIFY(!master.isPumpActive());

        master.shutdown();
    }
};

QTEST_MAIN(TestB222E)
#include "TestB222E.moc"
