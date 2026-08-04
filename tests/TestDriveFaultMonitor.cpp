// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestDriveFaultMonitor.cpp  (Build 63 — TF-1)
//
// Unit tests for DriveFaultMonitor, the per-drive fault state
// machine extracted from ControlLoop.
//
// Tests cover:
//   TF-1-1: Normal operation (no fault) -- no disable, no lockout
//   TF-1-2: Fault → reset success → allClearRehome + seedPosition
//   TF-1-3: Fault → MAX_FAULT_RETRIES → lockout
//   TF-1-4: Self-recovery via stable-cycles (B57 fix path)
//   TF-1-5: Healthy-cycles reset clears retry counter
//   TF-1-6: E-stop release (reset()) clears all state
//   TF-1-7: User clearLockout() allows retry after lockout
//   TF-1-8: FaultReactionActive is handled (skipEnableSM, no reset started)
//   TF-1-9: Multi-drive allClearRehome fires only when ALL faults clear
// ============================================================

#include <QtTest>
#include "../src/DriveFaultMonitor.h"
#include "../src/MockA6Drive.h"
#include "../src/Logging.h"

class TestDriveFaultMonitor : public QObject
{
    Q_OBJECT

private slots:

    void initTestCase()
    {
        // TF-6: suppress log output during tests
        Logger::instance().setMinLevel(LogLevel::LVL_CRITICAL);
    }

    // ---- TF-1-1: Normal enable path -- no fault ----
    // Every cycle the monitor gets OperationEnabled → should return empty result.
    void normalOperation_noFault()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);

        DriveFaultMonitor mon;
        mon.configure(1);

        for (uint64_t cycle = 1; cycle <= 200; ++cycle)
        {
            auto r = mon.step(0, &mock, DriveState::OperationEnabled, cycle);
            QVERIFY(!r.disable);
            QVERIFY(!r.firstFaultSeen);
            QVERIFY(!r.allClearRehome);
            QVERIFY(!r.lockoutJustOccurred);
            QVERIFY(!r.seedPosition);
            QVERIFY(!r.skipEnableSM);
        }

        QVERIFY(!mon.isLocked(0));
        QVERIFY(!mon.isFaultSeen(0));
        QCOMPARE(mon.getRetryCount(0), 0);
    }

    // ---- TF-1-2: Fault → reset succeeds → seedPosition + allClearRehome ----
    void faultResetSuccess_seedAndRehome()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.injectFault();

        DriveFaultMonitor mon;
        mon.configure(1);

        // First step with Fault: firstFaultSeen
        auto r1 = mon.step(0, &mock, DriveState::Fault, 1);
        QVERIFY(r1.firstFaultSeen);
        QVERIFY(!r1.disable);      // not locked out yet
        QVERIFY(r1.skipEnableSM);  // still in fault handling
        QVERIFY(mon.isFaultSeen(0));

        // Mock's stepFaultReset() takes 3 calls to clear
        // (m_faultResetStep increments to 3 → clears on the 3rd step call)
        // But startFaultReset() is called by the monitor on the first Fault step,
        // so subsequent steps call stepFaultReset.
        // Step 2 and 3 — still pending
        auto r2 = mon.step(0, &mock, DriveState::Fault, 2);
        QVERIFY(!r2.firstFaultSeen); // already seen
        QVERIFY(!r2.disable);
        QVERIFY(r2.skipEnableSM);

        // Step 3 — cleared
        auto r3 = mon.step(0, &mock, DriveState::Fault, 3);
        QVERIFY(r3.seedPosition);
        QVERIFY(r3.allClearRehome);
        QVERIFY(!r3.disable);
        QVERIFY(!mon.isFaultSeen(0));
        QCOMPARE(mon.getRetryCount(0), 1);
    }

    // ---- TF-1-3: Fault repeats → MAX_FAULT_RETRIES → lockout ----
    void faultRepeat_lockoutAfterMaxRetries()
    {
        DriveFaultMonitor mon;
        mon.configure(1);

        bool lockoutSeen = false;
        uint64_t cycle = 0;

        for (int attempt = 0; attempt < DriveFaultMonitor::MAX_FAULT_RETRIES + 2; ++attempt)
        {
            // Fresh mock each round — each mock starts with faultResetStep=0
            MockA6Drive mock;
            mock.configure(1, 0.0, 0.05);
            mock.injectFault();

            // Drive stuck in Fault — run enough steps to complete one reset cycle
            // (3 stepFaultReset calls) then inject fault again
            for (int s = 0; s < 5; ++s)
            {
                ++cycle;
                auto r = mon.step(0, &mock, DriveState::Fault, cycle);
                if (r.lockoutJustOccurred)
                {
                    lockoutSeen = true;
                    QVERIFY(r.disable);
                    QVERIFY(mon.isLocked(0));
                    goto done;
                }
                if (r.seedPosition)
                {
                    // fault cleared this round — re-inject for next attempt
                    mock.injectFault();
                    break;
                }
            }
        }
        done:
        QVERIFY2(lockoutSeen, "Expected lockoutJustOccurred before reaching MAX_FAULT_RETRIES+2 attempts");
        QVERIFY(mon.isLocked(0));

        // Subsequent steps must return disable immediately
        MockA6Drive mock2;
        mock2.configure(1, 0.0, 0.05);
        auto r = mon.step(0, &mock2, DriveState::OperationEnabled, ++cycle);
        QVERIFY(r.disable);
        QVERIFY(!r.firstFaultSeen);
    }

    // ---- TF-1-4: Self-recovery via stable-cycles (B57 path) ----
    // Drive was in Fault, then spontaneously recovers to OperationEnabled
    // without going through stepFaultReset(). Must clear after FAULT_CLEAR_STABLE_CYCLES.
    void selfRecovery_stableCyclesClearsFaultSeen()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.injectFault();

        DriveFaultMonitor mon;
        mon.configure(1);

        // Inject fault to set faultSeen
        auto r0 = mon.step(0, &mock, DriveState::Fault, 1);
        QVERIFY(r0.firstFaultSeen);
        QVERIFY(mon.isFaultSeen(0));

        // Spontaneous recovery -- no stepFaultReset, just OperationEnabled
        mock.clearFault();

        // Run FAULT_CLEAR_STABLE_CYCLES cycles (c=2..101): stableCycles reaches 99 max.
        // opEnabledSince is set to 2 on the first call, so stableCycles = c - 2.
        // At c=101: stableCycles=99, still below the 100 threshold.
        const uint64_t stableNeeded = DriveFaultMonitor::FAULT_CLEAR_STABLE_CYCLES;
        for (uint64_t c = 2; c < 2 + stableNeeded; ++c)
        {
            auto r = mon.step(0, &mock, DriveState::OperationEnabled, c);
            QVERIFY(!r.allClearRehome);
        }
        QVERIFY(mon.isFaultSeen(0));  // not cleared yet

        // One more cycle: stableCycles = 100, threshold reached → fires
        uint64_t finalCycle = 2 + stableNeeded;
        auto rFinal = mon.step(0, &mock, DriveState::OperationEnabled, finalCycle);
        QVERIFY(rFinal.allClearRehome);
        QVERIFY(!mon.isFaultSeen(0));
    }

    // ---- TF-1-5: Healthy-cycles reset clears retry counter ----
    void healthyCycles_resetsRetryCounter()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.injectFault();

        DriveFaultMonitor mon;
        mon.configure(1);

        // One fault-reset cycle (3 steps)
        for (int s = 1; s <= 3; ++s)
            mon.step(0, &mock, DriveState::Fault, s);

        QCOMPARE(mon.getRetryCount(0), 1);

        // Fault cleared -- now run HEALTHY_CYCLES_RESET + 1 cycles at OperationEnabled
        mock.clearFault();
        const uint64_t healthyNeeded = DriveFaultMonitor::HEALTHY_CYCLES_RESET;
        for (uint64_t c = 4; c <= 4 + healthyNeeded; ++c)
            mon.step(0, &mock, DriveState::OperationEnabled, c);

        QCOMPARE(mon.getRetryCount(0), 0);
    }

    // ---- TF-1-6: reset() clears all state ----
    void reset_clearsAllState()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.injectFault();

        DriveFaultMonitor mon;
        mon.configure(2);

        mon.step(0, &mock, DriveState::Fault, 1);
        QVERIFY(mon.isFaultSeen(0));

        mon.reset();

        QVERIFY(!mon.isFaultSeen(0));
        QVERIFY(!mon.isLocked(0));
        QCOMPARE(mon.getRetryCount(0), 0);
        QVERIFY(!mon.isAnyFaultSeen());
    }

    // ---- TF-1-7: clearLockout() allows retry after lockout ----
    void clearLockout_allowsRetry()
    {
        DriveFaultMonitor mon;
        mon.configure(1);

        // Drive locked out
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.injectFault();

        bool lockoutSeen = false;
        for (uint64_t c = 1; c <= 200 && !lockoutSeen; ++c)
        {
            auto r = mon.step(0, &mock, DriveState::Fault, c);
            if (r.lockoutJustOccurred) lockoutSeen = true;
        }
        QVERIFY(lockoutSeen);
        QVERIFY(mon.isLocked(0));

        // Clear lockout
        mon.clearLockout(0);
        QVERIFY(!mon.isLocked(0));
        QCOMPARE(mon.getRetryCount(0), 0);
        QVERIFY(!mon.isFaultSeen(0));

        // Next step with no fault → proceed normally
        mock.clearFault();
        auto r = mon.step(0, &mock, DriveState::OperationEnabled, 201);
        QVERIFY(!r.disable);
        QVERIFY(!r.skipEnableSM);
    }

    // ---- TF-1-8: FaultReactionActive → skipEnableSM, no reset started ----
    void faultReactionActive_noResetStarted()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);

        DriveFaultMonitor mon;
        mon.configure(1);

        // FaultReactionActive is a transient state -- no reset should start
        auto r = mon.step(0, &mock, DriveState::FaultReactionActive, 1);
        QVERIFY(r.firstFaultSeen);
        QVERIFY(r.skipEnableSM);
        QVERIFY(!r.disable);

        QCOMPARE(mon.getRetryCount(0), 0);  // no reset attempted

        // Next cycle still FaultReactionActive
        auto r2 = mon.step(0, &mock, DriveState::FaultReactionActive, 2);
        QVERIFY(!r2.firstFaultSeen);  // already seen
        QVERIFY(r2.skipEnableSM);
        QCOMPARE(mon.getRetryCount(0), 0);  // still no reset
    }

    // ---- TF-1-9: Multi-drive allClearRehome only when ALL clear ----
    void multiDrive_allClearRehomeFiringOrder()
    {
        MockA6Drive mock0, mock1;
        mock0.configure(1, 0.0, 0.05);
        mock1.configure(2, 0.0, 0.05);
        mock0.injectFault();
        mock1.injectFault();

        DriveFaultMonitor mon;
        mon.configure(2);

        // Both drives in fault
        mon.step(0, &mock0, DriveState::Fault, 1);
        mon.step(1, &mock1, DriveState::Fault, 1);

        QVERIFY(mon.isFaultSeen(0));
        QVERIFY(mon.isFaultSeen(1));

        // Drive 0 clears after 3 stepFaultReset calls (cycle 2 and 3 have already
        // incremented faultResetStep once each in step 1, so cycle 3 completes it)
        bool drive0Cleared = false;
        for (uint64_t c = 2; c <= 5 && !drive0Cleared; ++c)
        {
            auto r = mon.step(0, &mock0, DriveState::Fault, c);
            if (r.seedPosition)
            {
                // allClearRehome must NOT fire -- drive 1 still in fault
                QVERIFY2(!r.allClearRehome,
                    "allClearRehome fired while drive 1 still faulted");
                drive0Cleared = true;
            }
        }
        QVERIFY2(drive0Cleared, "Drive 0 did not clear within 5 cycles");
        QVERIFY(!mon.isFaultSeen(0));
        QVERIFY( mon.isFaultSeen(1));

        // Now clear drive 1
        bool drive1Cleared = false;
        for (uint64_t c = 6; c <= 10 && !drive1Cleared; ++c)
        {
            auto r = mon.step(1, &mock1, DriveState::Fault, c);
            if (r.seedPosition)
            {
                QVERIFY2(r.allClearRehome,
                    "allClearRehome did not fire when last drive cleared");
                drive1Cleared = true;
            }
        }
        QVERIFY2(drive1Cleared, "Drive 1 did not clear within 10 cycles");
        QVERIFY(!mon.isFaultSeen(0));
        QVERIFY(!mon.isFaultSeen(1));
    }
};

QTEST_MAIN(TestDriveFaultMonitor)
#include "TestDriveFaultMonitor.moc"
