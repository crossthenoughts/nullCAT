// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestCommissioningFlow.cpp - integration tests for the commissioning
// TESTING state machine through MotionController with MockA6Drive.
//
// The engine itself is covered by TestCommissioning; THIS file pins the
// wiring that must not regress: the entry rails (unhomed / unparked /
// telemetry-active / belt refusals), the closed-loop happy path (PARKED ->
// TESTING -> results -> park-out), and every cancel path (user park,
// e-stop, following-error abort). Cycle time 100Hz, mock chase rate set
// per test: fast enough to track for happy paths, deliberately too slow
// to prove the ferr abort rail.
// ============================================================

#include <QtTest>
#include "../src/MotionController.h"
#include "../src/MockA6Drive.h"
#include "../src/Config.h"
#include "../src/Logging.h"
#include <cstring>

// Single vertical CSP axis, foldback frame, endstop park (parkPos = 1.5mm)
// so commissioning entry starts ~48.5mm from centre and Centering is real.
static AppConfig makeCfg()
{
    DriveConfig dc;
    dc.slaveIndex           = 1;
    dc.axisType             = "linear_vertical";
    dc.mode                 = "csp";
    dc.invertDir            = true;    // foldback fixture (see TestHomingSequence)
    dc.parkMode             = "endstop";
    dc.strokeMm             = 100.0;
    dc.homingSpeed          = 400.0;
    dc.homingBackoffMm      = 1.5;
    dc.homingTorquePct      = 25;
    dc.homeDirection        = "negative";
    dc.maxVelocityMmS       = 200.0;
    dc.maxAccelerationMmS2  = 2000.0;
    dc.maxJerkMmS3          = 20000.0;
    dc.unparkTimeSec        = 0.1;
    dc.parkTimeSec          = 0.1;
    dc.countsPerMm          = 100.0;
    dc.ballscrewPitch       = 5.0;

    AppConfig cfg;
    cfg.controlLoopHz       = 100;
    cfg.numDrives           = 1;
    cfg.blendTimeSec        = 0.1;
    cfg.blendMaxVelocityMmS = 20.0;
    cfg.drives.push_back(dc);
    return cfg;
}

// One closed-loop cycle: mock chases its target, controller runs, and the
// controller's output is applied through the frame-aware setter once homed
// (homing writes its own raw targets) -- the same shape ControlLoop has.
static void cycleOnce(MotionController& mc, MockA6Drive& mock,
                      const TelemetryData& td = TelemetryData{})
{
    A6Drive* drives[1] = { &mock };
    mock.updateStatus();
    MotionOutput out{};
    mc.process(td, out, drives, 1);
    if (mc.isAxisHomed(0)) mock.setTargetPosition(out.positions[0]);
}

static bool runToState(MotionController& mc, MockA6Drive& mock,
                       AxisMotionState want, int maxCycles)
{
    for (int i = 0; i < maxCycles; ++i)
    {
        cycleOnce(mc, mock);
        if (mc.getAxisState(0) == want) return true;
    }
    return false;
}

// Home, then park, then settle until the mock has physically reached the
// commanded park position (commissioning entry measures from there).
static bool homeAndPark(MotionController& mc, MockA6Drive& mock)
{
    mc.startHoming();
    if (!runToState(mc, mock, AxisMotionState::ONLINE, 20000)) return false;
    mc.startPark();
    if (!runToState(mc, mock, AxisMotionState::PARKED, 2000)) return false;
    for (int i = 0; i < 300; ++i) cycleOnce(mc, mock);   // let the mock catch up
    return true;
}

// A one-segment tone plan on axis 0: 2 Hz, 5 mm -- comfortably inside the
// axis budgets and the fast mock's 100 mm/s chase rate.
static CommissioningPlan tonePlan(double durationSec = 1.5)
{
    CommissioningPlan plan;
    plan.numSegments = 1;
    plan.seg[0].freqHz = 2.0;
    plan.seg[0].durationSec = durationSec;
    plan.seg[0].rampSec = 0.3;
    plan.seg[0].ampMm[0] = 5.0;
    std::snprintf(plan.seg[0].label, sizeof(plan.seg[0].label), "2.0 Hz");
    std::snprintf(plan.title, sizeof(plan.title), "flow test");
    return plan;
}

// A valid 16-bit centre telemetry frame (second channel non-zero so the
// frame is not treated as all-zeros no-data).
static TelemetryData centreFrame()
{
    TelemetryData td{};
    td.valid        = true;
    td.numPositions = 2;
    td.positions[0] = 32767.0;
    td.positions[1] = 32767.0;
    td.packetType   = TelemetryPacketType::Motion;
    return td;
}

class TestCommissioningFlow : public QObject
{
    Q_OBJECT

private slots:

    void initTestCase()
    {
        Logger::instance().setMinLevel(LogLevel::LVL_CRITICAL);
    }

    // ---- Entry rail: unhomed axes refuse, states untouched ----
    void refusesWhenUnhomed()
    {
        MockA6Drive mock; mock.configure(1, 0.0, 1.0);
        MotionController mc; mc.configure(makeCfg());

        QVERIFY(mc.requestCommissioningStart(tonePlan()));
        for (int i = 0; i < 20; ++i) cycleOnce(mc, mock);

        QCOMPARE(mc.getAxisState(0), AxisMotionState::PARKED);
        CommissioningStatus st = mc.getCommissioningStatus();
        QVERIFY(!st.active);
        QVERIFY2(std::strstr(st.reason, "rehome") != nullptr
              || std::strstr(st.reason, "not homed") != nullptr,
                 qPrintable(QString("reason was: %1").arg(st.reason)));
    }

    // ---- Entry rail: a live telemetry stream refuses ----
    void refusesWhenTelemetryActive()
    {
        MockA6Drive mock; mock.configure(1, 0.0, 1.0);
        mock.setHardstop(-2.0, true, 50.0);
        MotionController mc; mc.configure(makeCfg());
        QVERIFY(homeAndPark(mc, mock));

        const TelemetryData td = centreFrame();
        for (int i = 0; i < 10; ++i) cycleOnce(mc, mock, td);

        QVERIFY(mc.requestCommissioningStart(tonePlan()));
        for (int i = 0; i < 20; ++i) cycleOnce(mc, mock);   // stream stops; <2s quiet

        CommissioningStatus st = mc.getCommissioningStatus();
        QVERIFY(!st.active);
        QVERIFY2(std::strstr(st.reason, "telemetry") != nullptr,
                 qPrintable(QString("reason was: %1").arg(st.reason)));
        QCOMPARE(mc.getAxisState(0), AxisMotionState::PARKED);
    }

    // ---- Entry rail: belt/torque axes are never testable ----
    void refusesBeltAxis()
    {
        AppConfig cfg = makeCfg();
        cfg.drives[0].axisType = "belt";
        cfg.drives[0].mode     = "torque";
        MockA6Drive mock; mock.configure(1, 0.0, 1.0);
        MotionController mc; mc.configure(cfg);

        QVERIFY(mc.requestCommissioningStart(tonePlan()));
        for (int i = 0; i < 20; ++i) cycleOnce(mc, mock);

        CommissioningStatus st = mc.getCommissioningStatus();
        QVERIFY(!st.active);
        QVERIFY2(std::strstr(st.reason, "not testable") != nullptr,
                 qPrintable(QString("reason was: %1").arg(st.reason)));
    }

    // ---- Happy path: PARKED -> TESTING -> results -> parked again ----
    void happyPathRunsAndParksOut()
    {
        MockA6Drive mock; mock.configure(1, 0.0, 1.0);   // 100 mm/s chase
        mock.setHardstop(-2.0, true, 50.0);
        MotionController mc; mc.configure(makeCfg());
        QVERIFY(homeAndPark(mc, mock));

        QVERIFY(mc.requestCommissioningStart(tonePlan()));
        QVERIFY(runToState(mc, mock, AxisMotionState::TESTING, 50));

        // Centering ~2s + segment 1.5s + return; 100Hz -> generous bound.
        bool parked = false;
        for (int i = 0; i < 3000 && !parked; ++i)
        {
            cycleOnce(mc, mock);
            parked = (mc.getAxisState(0) == AxisMotionState::PARKED);
        }
        QVERIFY2(parked, "test did not park out after completion");

        CommissioningStatus st = mc.getCommissioningStatus();
        QVERIFY(!st.active);
        QVERIFY2(!st.aborted, qPrintable(QString("aborted: %1").arg(st.reason)));
        QCOMPARE(st.resultCount, 1);
        const CommissioningAxisResult& ar = st.results[0].axis[0];
        QVERIFY(ar.tested);
        // The fast mock is a 100 mm/s rate-limited follower; at 2 Hz / 5 mm
        // (peak 63 mm/s) it tracks with some lag -- ratio must be sane.
        QVERIFY2(ar.actAmpMm / ar.cmdAmpMm > 0.5 && ar.actAmpMm / ar.cmdAmpMm < 1.1,
                 qPrintable(QString("ratio %1").arg(ar.actAmpMm / ar.cmdAmpMm)));
        // Completing a test must NOT demand a rehome (nothing faulted).
        QVERIFY(!mc.needsRehome());
    }

    // ---- Cancel path: user park during a test cancels cleanly ----
    void userParkCancels()
    {
        MockA6Drive mock; mock.configure(1, 0.0, 1.0);
        mock.setHardstop(-2.0, true, 50.0);
        MotionController mc; mc.configure(makeCfg());
        QVERIFY(homeAndPark(mc, mock));

        QVERIFY(mc.requestCommissioningStart(tonePlan(10.0)));
        QVERIFY(runToState(mc, mock, AxisMotionState::TESTING, 50));
        for (int i = 0; i < 100; ++i) cycleOnce(mc, mock);

        mc.startPark();
        QVERIFY(runToState(mc, mock, AxisMotionState::PARKED, 2000));

        CommissioningStatus st = mc.getCommissioningStatus();
        QVERIFY(!st.active);
        QVERIFY(st.aborted);
        QVERIFY2(std::strstr(st.reason, "taken over") != nullptr,
                 qPrintable(QString("reason was: %1").arg(st.reason)));
    }

    // ---- Cancel path: e-stop during a test cancels and stops in place ----
    void estopCancels()
    {
        MockA6Drive mock; mock.configure(1, 0.0, 1.0);
        mock.setHardstop(-2.0, true, 50.0);
        MotionController mc; mc.configure(makeCfg());
        QVERIFY(homeAndPark(mc, mock));

        QVERIFY(mc.requestCommissioningStart(tonePlan(10.0)));
        QVERIFY(runToState(mc, mock, AxisMotionState::TESTING, 50));
        for (int i = 0; i < 100; ++i) cycleOnce(mc, mock);

        mc.setEmergencyStop(true);
        cycleOnce(mc, mock);
        QCOMPARE(mc.getAxisState(0), AxisMotionState::ESTOPPING);
        QVERIFY(runToState(mc, mock, AxisMotionState::PARKED, 200));

        CommissioningStatus st = mc.getCommissioningStatus();
        QVERIFY(!st.active);
        QVERIFY(st.aborted);
        mc.setEmergencyStop(false);   // release for teardown symmetry
    }

    // ---- Abort rail end-to-end: an axis that cannot follow aborts and
    //      the rig still ends PARKED through the normal park-out ----
    void ferrAbortParksOut()
    {
        MockA6Drive mock; mock.configure(1, 0.0, 0.05);   // 5 mm/s: far too slow
        mock.setHardstop(-2.0, true, 50.0);
        MotionController mc; mc.configure(makeCfg());
        QVERIFY(homeAndPark(mc, mock));

        QVERIFY(mc.requestCommissioningStart(tonePlan(10.0)));
        QVERIFY(runToState(mc, mock, AxisMotionState::TESTING, 50));

        bool parked = false;
        for (int i = 0; i < 5000 && !parked; ++i)
        {
            cycleOnce(mc, mock);
            parked = (mc.getAxisState(0) == AxisMotionState::PARKED);
        }
        QVERIFY2(parked, "aborted test did not park out");

        CommissioningStatus st = mc.getCommissioningStatus();
        QVERIFY(st.aborted);
        QVERIFY2(std::strstr(st.reason, "following error") != nullptr,
                 qPrintable(QString("reason was: %1").arg(st.reason)));
    }

    // ---- A second test after a completed one starts cleanly ----
    void secondRunStartsClean()
    {
        MockA6Drive mock; mock.configure(1, 0.0, 1.0);
        mock.setHardstop(-2.0, true, 50.0);
        MotionController mc; mc.configure(makeCfg());
        QVERIFY(homeAndPark(mc, mock));

        for (int run = 0; run < 2; ++run)
        {
            QVERIFY(mc.requestCommissioningStart(tonePlan()));
            QVERIFY(runToState(mc, mock, AxisMotionState::TESTING, 50));
            bool parked = false;
            for (int i = 0; i < 3000 && !parked; ++i)
            {
                cycleOnce(mc, mock);
                parked = (mc.getAxisState(0) == AxisMotionState::PARKED);
            }
            QVERIFY(parked);
            for (int i = 0; i < 300; ++i) cycleOnce(mc, mock);
        }
        CommissioningStatus st = mc.getCommissioningStatus();
        QVERIFY(!st.aborted);
        QCOMPARE(st.resultCount, 1);   // fresh run: results reset, one row
    }
};

QTEST_MAIN(TestCommissioningFlow)
#include "TestCommissioningFlow.moc"
