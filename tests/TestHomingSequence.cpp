// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestHomingSequence.cpp  (Build 55 — P2-2)
//
// Unit tests for HomingSequence state machine.
// Exercises every state and error path using MockA6Drive.
//
// Test cycle time: 10ms (cycleTimeSec=0.01).
// Timeouts: enable=10s(1000c), homing=60s(6000c), backoff=10s(1000c).
// Torque confirm: 50 consecutive cycles above threshold.
// ============================================================

#include <QtTest>
#include "../src/HomingSequence.h"
#include "../src/MockA6Drive.h"
#include "../src/Config.h"
#include "../src/Logging.h"

// Build a DriveConfig wired for homing tests.
// speed=5mm/s, cycle=0.01s → 0.05mm per cycle
// backoff=1.5mm, torquePct=25%, dir=negative, stroke=100mm
static DriveConfig makeConfig(const std::string& dir = "negative",
                              double strokeMm    = 100.0,
                              double speedMmS    = 400.0,   // 0.2mm/cycle in-harness (see homingStepMm)
                              double backoffMm   = 1.5,
                              int    torquePct   = 25)
{
    DriveConfig c;
    c.homeDirection    = dir;
    c.strokeMm         = strokeMm;
    c.homingSpeedMmS   = speedMmS;
    c.homingBackoffMm  = backoffMm;
    c.homingTorquePct  = torquePct;
    return c;
}

// Run HomingSequence::step() up to maxCycles.
// Returns the final state. Stops early if Complete/Error/FatalError reached.
static HomingSequence::State runUntilDone(HomingSequence& hs, A6Drive* drive, int maxCycles)
{
    HomingSequence::State s = HomingSequence::State::Idle;
    for (int i = 0; i < maxCycles; ++i)
    {
        s = hs.step(drive);
        if (s == HomingSequence::State::Complete  ||
            s == HomingSequence::State::Error      ||
            s == HomingSequence::State::FatalError)
            break;
    }
    return s;
}

class TestHomingSequence : public QObject
{
    Q_OBJECT

private slots:

    void initTestCase()
    {
        // TF-6: suppress log output during tests
        Logger::instance().setMinLevel(LogLevel::LVL_CRITICAL);
    }

    // ---- P2-2-1: Normal homing completes successfully ----
    void normalHomingCompletes()
    {
        // Hardstop at -50mm (homing negative from 0).
        // maxVelPerCycle = 0.05mm → reaches stop in ~1000 cycles.
        // torque = 50% → exceeds threshold=25% immediately at stop.
        // TORQUE_CONFIRM_CYCLES=50 → 50 cycles at stop to confirm.
        // Backoff 1.5mm at 0.05mm/cycle → 30 cycles.
        // Total expected: ~1080 cycles, well within 3000 cycle limit.
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.setHardstop(-50.0, /*isMinLimit=*/true, 50.0);

        HomingSequence hs;
        hs.configure(makeConfig(), 0.01);
        hs.start(&mock);

        HomingSequence::State s = runUntilDone(hs, &mock, 3500);

        QCOMPARE((int)s, (int)HomingSequence::State::Complete);
        QVERIFY(hs.isComplete());
        QVERIFY(!hs.isFatalError());

        // Home offset should be near the backoff position (-48.5mm ≈ hardstopPos + backoffMm)
        double expectedOffset = -50.0 + 1.5;  // hardstop + backoff
        QVERIFY(std::abs(hs.getHomeOffset() - expectedOffset) < 0.2);

        // Hardstop position recorded
        QVERIFY(std::abs(hs.getHardstopPos() - (-50.0)) < 0.2);
    }

    // ---- P2-2-2: Torque search timeout → FatalError ----
    // Drive can move freely, never hits a hardstop.
    // But stroke guard fires first (150mm / 0.5mm/cycle = 300 cycles < timeout).
    // Use fast speed to separate from timeout, or no hardstop → timeout at 6000c.
    void torqueSearchTimeout()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.001);  // very slow: 0.001mm/cycle → will never reach stroke limit
        // No hardstop → torque stays zero → timeout after 6000 cycles (60s at 0.01s)

        HomingSequence hs;
        hs.configure(makeConfig("negative", 100.0, 0.001), 0.01);  // tiny speed matches mock
        hs.start(&mock);

        HomingSequence::State s = runUntilDone(hs, &mock, 7000);

        QCOMPARE((int)s, (int)HomingSequence::State::FatalError);
        QVERIFY(hs.isFatalError());
    }

    // ---- P2-2-3: Stroke guard fires before timeout ----
    // Speed 1000 → step = 1000*REF_DT(0.0005) = 0.5mm/cycle. No hardstop.
    // strokeMm=100 → limit=150mm. 150/0.5 = 300 cycles. Timeout is 6000 cycles.
    // The distance guard is checked BEFORE the timeout every cycle, so lengthening
    // the timeout only widens the separation this test relies on.
    void strokeGuardTriggers()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.5);   // 0.5mm/cycle = 50mm/s at 0.01s cycle
        // No hardstop -- drive runs freely into stroke limit

        DriveConfig cfg = makeConfig("negative", 100.0, 1000.0); // strokeMm=100 → limit=150mm
        HomingSequence hs;
        hs.configure(cfg, 0.01);
        hs.start(&mock);

        HomingSequence::State s = runUntilDone(hs, &mock, 2000);

        // Must abort with FatalError due to stroke guard, well before timeout (3000c)
        QCOMPARE((int)s, (int)HomingSequence::State::FatalError);

        // Verify it fired on stroke guard, not timeout (should stop around cycle 300)
        QVERIFY(hs.getCycleCount() < 1000);
    }

    // ---- P2-2-4: Drive fault during torque search → FatalError ----
    void faultDuringTorqueSearch()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.setHardstop(-50.0, true, 50.0);

        HomingSequence hs;
        hs.configure(makeConfig(), 0.01);
        hs.start(&mock);

        // Run until CSPTorqueSearch is active.
        // B59: 3 cycles to enable (default m_enableCyclesToComplete=3) +
        //       10 cycles in SettlingDrive (SETTLE_CYCLES) = 13 minimum.
        for (int i = 0; i < 15; ++i) hs.step(&mock);
        QCOMPARE((int)hs.getState(), (int)HomingSequence::State::CSPTorqueSearch);

        // Inject fault mid-search
        mock.injectFault();

        HomingSequence::State s = runUntilDone(hs, &mock, 100);
        QCOMPARE((int)s, (int)HomingSequence::State::FatalError);
    }

    // ---- P2-2-5: Drive fault during backoff → FatalError ----
    void faultDuringBackoff()
    {
        // Close hardstop at -2mm so we reach Backoff quickly (2/0.05 = 40 cycles + 50 confirm = 90c)
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.setHardstop(-2.0, true, 50.0);

        HomingSequence hs;
        hs.configure(makeConfig(), 0.01);
        hs.start(&mock);

        // Run until we reach Backoff state
        int limit = 500;
        while (hs.getState() != HomingSequence::State::Backoff && limit-- > 0)
            hs.step(&mock);

        QCOMPARE((int)hs.getState(), (int)HomingSequence::State::Backoff);

        // Inject fault during backoff
        mock.injectFault();

        HomingSequence::State s = runUntilDone(hs, &mock, 100);
        QCOMPARE((int)s, (int)HomingSequence::State::FatalError);
    }

    // ---- P2-2-6: Enable timeout → FatalError ----
    // Drive never reaches OperationEnabled.
    void enableTimeout()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.setEnableCycles(999999);   // never enables within test run

        HomingSequence hs;
        hs.configure(makeConfig(), 0.01);
        hs.start(&mock);

        // Enable timeout = 10s / 0.01s = 1000 cycles
        HomingSequence::State s = runUntilDone(hs, &mock, 1500);
        QCOMPARE((int)s, (int)HomingSequence::State::FatalError);
    }

    // ---- P2-2-7: FatalError is terminal — step() does not restart ----
    // Key validation for P1-1: after FatalError, further step() calls must
    // return FatalError immediately without changing any state.
    void fatalErrorDoesNotRestart()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.setEnableCycles(999999);   // trigger enable timeout → FatalError

        HomingSequence hs;
        hs.configure(makeConfig(), 0.01);
        hs.start(&mock);

        // Wait for FatalError
        runUntilDone(hs, &mock, 1500);
        QCOMPARE((int)hs.getState(), (int)HomingSequence::State::FatalError);

        int cyclesBefore = hs.getCycleCount();

        // Call step() 20 more times -- must stay in FatalError, no progression
        for (int i = 0; i < 20; ++i)
        {
            HomingSequence::State s = hs.step(&mock);
            QCOMPARE((int)s, (int)HomingSequence::State::FatalError);
        }

        // Cycle count must not have advanced (early return guard active)
        QCOMPARE(hs.getCycleCount(), cyclesBefore);
    }

    // ---- P2-2-8: Positive homing direction ----
    // Verify dirSign is applied correctly when homeDirection="positive".
    void positiveDirectionHoming()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.setHardstop(50.0, /*isMinLimit=*/false, 50.0);  // stop at +50mm

        DriveConfig cfg = makeConfig("positive");
        HomingSequence hs;
        hs.configure(cfg, 0.01);
        hs.start(&mock);

        HomingSequence::State s = runUntilDone(hs, &mock, 3500);

        QCOMPARE((int)s, (int)HomingSequence::State::Complete);
        // Hardstop near +50mm
        QVERIFY(std::abs(hs.getHardstopPos() - 50.0) < 0.5);
    }

    // ---- P3-1: Stroke limit fires FatalError when no hardstop found ----
    // homeDirection=negative, strokeMm=50mm → limit=75mm.
    // No hardstop set → drive moves negatively indefinitely.
    // After ~1500 TorqueSearch cycles (75mm / 0.05mm) the stroke guard fires.
    // FatalError must be reached before the 2000-cycle limit.
    void strokeLimitExceeded_fatalError()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        // No hardstop -- drive moves forever in negative direction

        HomingSequence hs;
        // strokeMm=50 → m_strokeLimitMm = 75mm
        hs.configure(makeConfig("negative", 50.0), 0.01);
        hs.start(&mock);

        // Run up to 2000 cycles: 3 enable + 10 settle + 1500 search = 1513 expected
        HomingSequence::State s = runUntilDone(hs, &mock, 2000);

        QCOMPARE((int)s, (int)HomingSequence::State::FatalError);
        QVERIFY(hs.isFatalError());

        // Distance at FatalError must be >= 75mm (stroke limit)
        // (allow a cycle or two of overshoot before the check triggers)
        double distTraveled = std::abs(mock.getSimActualPos() - 0.0);
        QVERIFY2(distTraveled >= 74.0,
            qPrintable(QString("Expected >=74mm traveled at FatalError, got %1mm")
                .arg(distTraveled, 0, 'f', 2)));
    }

    // ---- P3-2: isQuickStopActive() reflects injected QSA state ----
    void quickStopActive_detectedCorrectly()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);

        QVERIFY(!mock.isQuickStopActive());
        QVERIFY(!mock.isFault());

        mock.injectQuickStop();
        mock.updateStatus();

        QVERIFY(mock.isQuickStopActive());
        QCOMPARE((int)mock.getState(), (int)DriveState::QuickStopActive);
        QVERIFY(!mock.isFault());  // QSA is NOT a fault state

        // stepEnableStateMachine must return false (and not write any CW)
        bool enabled = mock.stepEnableStateMachine();
        QVERIFY(!enabled);

        mock.clearQuickStop();
        mock.updateStatus();

        QVERIFY(!mock.isQuickStopActive());
        QCOMPARE((int)mock.getState(), (int)DriveState::SwitchOnDisabled);
    }
};  // end class TestHomingSequence

QTEST_MAIN(TestHomingSequence)
#include "TestHomingSequence.moc"
