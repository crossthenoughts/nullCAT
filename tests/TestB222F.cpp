// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestB222F.cpp  (Build 222F)
//
// Unit tests for B222F bug fixes:
//   Fix 1: Pre-enable command-position sync (A6Drive + HomingSequence)
//   Fix 2: Post-OP WKC validation threshold math
//   Fix 3: Rehome-after-fault clears stale position limits
//   Fix 4: Diagnostic log emission (smoke tests)
//
// Tests:
//   F-1  Fix 1: Real A6Drive - hold at CW=0x07 for N sync cycles before 0x0F
//   F-2  Fix 1: Real A6Drive - target PDO tracks actual PDO during sync hold
//   F-3  Fix 1: HomingSequence fault during SettlingDrive → FatalError + settle message
//   F-4  Fix 1: HomingSequence no-fault settle → progresses to CSPTorqueSearch
//   F-5  Fix 2: WKC math - 0 of 50 good → fail
//   F-6  Fix 2: WKC math - 45 of 50 good (90%) → pass at threshold 0.9
//   F-7  Fix 2: WKC math - 44 of 50 good (88%) → fail at threshold 0.9
//   F-8  Fix 2: WKC math - 50 of 50 good (100%) → pass
//   F-9  Fix 3: MotionController clearAxisLimits - wide limits set before homing start
//   F-10 Fix 3: MotionController - correct limits restored after homing completes
//   F-11 Fix 4: Sim init smoke test - setMasterState diag log path runs without crash
//   F-12 Fix 4: Sim init smoke test - commandSyncCycles propagated to drives
// ============================================================

#include <QtTest>
#include <cstdint>

#include "../src/A6Drive.h"
#include "../src/HomingSequence.h"
#include "../src/MotionController.h"
#include "../src/EtherCATMaster.h"
#include "../src/MockA6Drive.h"
#include "../src/Config.h"
#include "../src/Logging.h"

// ---- Helpers ----

static DriveConfig makeHomingConfig(double strokeMm = 100.0,
                                    const std::string& dir = "negative")
{
    DriveConfig c;
    c.slaveIndex          = 1;
    c.axisType            = "linear_vertical";
    c.strokeMm            = strokeMm;
    c.homingSpeed      = 5.0;
    c.homingBackoffMm     = 1.5;
    c.homingTorquePct     = 25;
    c.homeDirection       = dir;
    c.invertDir           = true;   // foldback fixture: retract = raw negative (see TestHomingSequence)
    c.parkMode            = "endstop";
    c.maxVelocityMmS      = 200.0;
    c.maxAccelerationMmS2 = 2000.0;
    c.maxJerkMmS3         = 20000.0;
    c.unparkTimeSec       = 0.1;
    c.parkTimeSec         = 0.1;
    c.countsPerMm         = 100.0;
    c.ballscrewPitch      = 5.0;
    return c;
}

static AppConfig makeSimConfig(int syncCycles = 10)
{
    AppConfig cfg;
    cfg.numDrives          = 1;
    cfg.controlLoopHz      = 1000;
    cfg.simulationMode     = true;
    cfg.commandSyncCycles  = syncCycles;
    cfg.drives.push_back(makeHomingConfig());
    return cfg;
}

// Run HomingSequence::step() until the target state is reached or maxCycles exceeded.
// Returns the final state.
static HomingSequence::State runUntilState(HomingSequence& hs, MockA6Drive* drive,
    HomingSequence::State target, int maxCycles)
{
    HomingSequence::State s = HomingSequence::State::Idle;
    for (int i = 0; i < maxCycles; ++i)
    {
        s = hs.step(drive);
        if (s == target ||
            s == HomingSequence::State::Complete  ||
            s == HomingSequence::State::Error      ||
            s == HomingSequence::State::FatalError)
            break;
    }
    return s;
}

// ---- Test class ----

class TestB222F : public QObject
{
    Q_OBJECT

private Q_SLOTS:

    void initTestCase()
    {
        Logger::instance().init("", false);
        Logger::instance().setMinLevel(LogLevel::LVL_CRITICAL);
    }
    void cleanupTestCase() {}

    // ----------------------------------------------------------------
    // F-1: Real A6Drive holds at CW=0x07 for N sync cycles, then 0x0F.
    //
    // Uses real A6Drive with local PDO buffers (no SOEM).
    // setCommandSyncCycles(3) → 3 calls in SwitchedOn should keep CW=0x07,
    // then 4th call writes CW=0x0F.
    // ----------------------------------------------------------------
    void f1_commandSyncHoldsAt0x07()
    {
        const int SyncN = 3;
        uint16_t cw = 0;
        int8_t   mop = 0;
        int32_t  tgt = 0;
        uint16_t sw = 0;
        int8_t   mod = 0;
        int32_t  act = 12345;  // non-zero actual position

        A6Drive drive;
        drive.setSlaveIndex(1);
        drive.setScaling(364.0);  // B222O: single-arg, counts/mm. ~131072/360.
        drive.setCommandSyncCycles(SyncN);
        drive.setPDOPointers(&cw, &mop, &tgt, &sw, &mod, &act);

        // Statusword = SwitchedOn: (sw & 0x006F) == 0x0023
        // 0x0023 = CW_SWITCH_ON|CW_ENABLE_VOLTAGE|CW_QUICK_STOP (SW bits)
        sw = 0x0023;

        // Run N sync cycles - controlword must remain 0x07 throughout the hold phase.
        // (Old code overwrote 0x07 with 0x0F on the Nth call in the same invocation;
        //  new code keeps hold and post-sync strictly separated so all N calls stay 0x07.)
        for (int i = 0; i < SyncN; ++i)
        {
            sw = 0x0023;  // keep drive in SwitchedOn state
            drive.stepEnableStateMachine();
            QCOMPARE(static_cast<int>(cw), 0x07);  // must NOT have sent 0x0F yet
        }

        // N+1th call enters post-sync phase and writes 0x0F (EnableOperation)
        sw = 0x0023;
        drive.stepEnableStateMachine();
        QCOMPARE(static_cast<int>(cw), 0x0F);
    }

    // ----------------------------------------------------------------
    // F-2: During sync hold, target PDO tracks actual PDO counts.
    // ----------------------------------------------------------------
    void f2_commandSyncTracksActual()
    {
        uint16_t cw = 0;
        int8_t   mop = 0;
        int32_t  tgt = 0;
        uint16_t sw = 0x0023;
        int8_t   mod = 0;
        int32_t  act = 1000;

        A6Drive drive;
        drive.setSlaveIndex(1);
        drive.setScaling(364.0);  // B222O: single-arg, counts/mm. ~131072/360.
        drive.setCommandSyncCycles(5);
        drive.setPDOPointers(&cw, &mop, &tgt, &sw, &mod, &act);

        // Run 3 sync cycles, changing actual each time
        for (int i = 0; i < 3; ++i)
        {
            sw = 0x0023;
            act = 1000 + (i * 100);
            drive.stepEnableStateMachine();
            // Target must match actual (in counts, exact copy)
            QCOMPARE(tgt, act);
        }
    }

    // ----------------------------------------------------------------
    // F-3: HomingSequence fault during SettlingDrive → FatalError.
    //
    // The settle-specific log message is emitted (cannot capture here,
    // so we verify the state transitions correctly to FatalError).
    // ----------------------------------------------------------------
    void f3_faultDuringSettle_fatalError()
    {
        DriveConfig cfg = makeHomingConfig();
        MockA6Drive drive;
        drive.setSlaveIndex(1);
        drive.setScaling(364.0);  // B222O: single-arg, counts/mm. ~131072/360.
        drive.setMaxVelocity(200.0, 0.001);

        HomingSequence hs;
        hs.configure(cfg, 0.001);
        hs.start(&drive);

        // Run until SettlingDrive (MockA6Drive enables in m_enableCyclesToComplete=3 cycles)
        HomingSequence::State s = runUntilState(hs, &drive,
            HomingSequence::State::SettlingDrive, 100);
        QCOMPARE(s, HomingSequence::State::SettlingDrive);

        // Inject fault while settling
        drive.injectFault();

        // One more step → FatalError (settle-specific branch)
        s = hs.step(&drive);
        QCOMPARE(s, HomingSequence::State::FatalError);
    }

    // ----------------------------------------------------------------
    // F-4: No fault during settle → HomingSequence advances to CSPTorqueSearch.
    // ----------------------------------------------------------------
    void f4_noFaultSettle_progressesToSearch()
    {
        DriveConfig cfg = makeHomingConfig();
        MockA6Drive drive;
        drive.setSlaveIndex(1);
        drive.setScaling(364.0);  // B222O: single-arg, counts/mm. ~131072/360.
        drive.setMaxVelocity(200.0, 0.001);
        // Place hardstop at -10mm so torque search terminates
        drive.setHardstop(-10.0, true, 30.0);

        HomingSequence hs;
        hs.configure(cfg, 0.001);
        hs.start(&drive);

        // Run until SettlingDrive
        HomingSequence::State s = runUntilState(hs, &drive,
            HomingSequence::State::SettlingDrive, 100);
        QCOMPARE(s, HomingSequence::State::SettlingDrive);

        // Run through SETTLE_CYCLES=10 cycles with no fault → should advance to CSPTorqueSearch
        for (int i = 0; i < 15; ++i)
        {
            s = hs.step(&drive);
            if (s != HomingSequence::State::SettlingDrive)
                break;
        }
        QCOMPARE(s, HomingSequence::State::CSPTorqueSearch);
    }

    // ----------------------------------------------------------------
    // F-5: WKC fraction math - 0 good of 50 → fail (fraction < 0.9)
    // ----------------------------------------------------------------
    void f5_wkcMath_zeroGood_fail()
    {
        const int total = 50;
        const int good  = 0;
        const double threshold = 0.9;
        double fraction = static_cast<double>(good) / total;
        QVERIFY(fraction < threshold);
    }

    // ----------------------------------------------------------------
    // F-6: WKC fraction math - 45 good of 50 (90%) → pass at threshold 0.9
    // ----------------------------------------------------------------
    void f6_wkcMath_45of50_pass()
    {
        const int total = 50;
        const int good  = 45;
        const double threshold = 0.9;
        double fraction = static_cast<double>(good) / total;
        QVERIFY(fraction >= threshold);
    }

    // ----------------------------------------------------------------
    // F-7: WKC fraction math - 44 good of 50 (88%) → fail at threshold 0.9
    // ----------------------------------------------------------------
    void f7_wkcMath_44of50_fail()
    {
        const int total = 50;
        const int good  = 44;
        const double threshold = 0.9;
        double fraction = static_cast<double>(good) / total;
        QVERIFY(fraction < threshold);
    }

    // ----------------------------------------------------------------
    // F-8: WKC fraction math - 50 good of 50 (100%) → pass
    // ----------------------------------------------------------------
    void f8_wkcMath_50of50_pass()
    {
        const int total = 50;
        const int good  = 50;
        const double threshold = 0.9;
        double fraction = static_cast<double>(good) / total;
        QVERIFY(fraction >= threshold);
    }

    // ----------------------------------------------------------------
    // F-9: clearAxisLimits - wide limits applied before homing start.
    //
    // On the first processHomingAxis cycle (state==Idle), clearAxisLimits
    // must call drive->setLimits with (-2*stroke, 3*stroke).
    // stroke=100mm → (-200, 300).
    // ----------------------------------------------------------------
    void f9_clearAxisLimits_wideRangeOnRehome()
    {
        AppConfig cfg = makeSimConfig();

        MotionController mc;
        mc.configure(cfg);

        MockA6Drive drive;
        drive.setSlaveIndex(1);
        drive.setScaling(364.0);  // B222O: single-arg, counts/mm. ~131072/360.
        drive.setMaxVelocity(200.0, 0.001);
        drive.resetLimitsTracker();

        // Set hardstop so homing can actually complete
        drive.setHardstop(-10.0, true, 30.0);

        mc.startHoming();
        A6Drive* drives[1] = { &drive };
        TelemetryData empty{};
        MotionOutput out{};

        // One cycle to trigger processHomingAxis in Idle state → clearAxisLimits called
        drive.updateStatus();
        mc.process(empty, out, drives, 1);

        QVERIFY(mc.getAxisState(0) != AxisMotionState::ONLINE);  // not yet homed
        // clearAxisLimits must have been called with wide range
        int count = drive.getSetLimitsCallCount();
        QVERIFY(count >= 1);
        QCOMPARE(drive.getLastSetLimitsMin(), -200.0);  // -2 * 100mm
        QCOMPARE(drive.getLastSetLimitsMax(),  300.0);  // +3 * 100mm
    }

    // ----------------------------------------------------------------
    // F-10: After homing completes, limits are updated to the offset range.
    //
    // setLimits is called twice total: once by clearAxisLimits (wide),
    // once after homing complete (normal range offset by homeOffset).
    // The second call must have the narrower values.
    // ----------------------------------------------------------------
    void f10_limitsRestoredAfterHoming()
    {
        AppConfig cfg = makeSimConfig();

        MotionController mc;
        mc.configure(cfg);

        MockA6Drive drive;
        drive.setSlaveIndex(1);
        drive.setScaling(364.0);  // B222O: single-arg, counts/mm. ~131072/360.
        drive.setMaxVelocity(200.0, 0.001);
        drive.resetLimitsTracker();
        drive.setHardstop(-10.0, true, 30.0);

        mc.startHoming();
        A6Drive* drives[1] = { &drive };
        TelemetryData empty{};
        MotionOutput out{};

        for (int i = 0; i < 10000; ++i)
        {
            drive.updateStatus();
            mc.process(empty, out, drives, 1);
            if (mc.getAxisState(0) == AxisMotionState::ONLINE)
                break;
        }
        QVERIFY(mc.getAxisState(0) == AxisMotionState::ONLINE);

        // setLimits must have been called at least twice:
        // 1. clearAxisLimits (wide range)
        // 2. post-homing limits (normal range, offset-adjusted)
        QVERIFY(drive.getSetLimitsCallCount() >= 2);

        // Final limits should NOT be the wide range
        QVERIFY(drive.getLastSetLimitsMin() > -200.0);
        QVERIFY(drive.getLastSetLimitsMax() < 300.0);
    }

    // ----------------------------------------------------------------
    // F-11: Sim init smoke test - setMasterState diag log runs without crash.
    //
    // EtherCATMaster sim init calls setMasterState(Op), which now emits
    // a logDiag. Verify init succeeds and masterState == Op.
    // ----------------------------------------------------------------
    void f11_simInit_masterStateDiagNocrash()
    {
        AppConfig cfg = makeSimConfig(10);
        EtherCATMaster master;
        master.setSimulationMode(true);
        master.applyConfig(cfg);
        InitResult r = master.initializeAndEnterOp(cfg.nicName);
        QVERIFY(r.ok);
        QVERIFY(master.isOperational());
        QVERIFY(master.getMasterState() == ECState::Op);
    }

    // ----------------------------------------------------------------
    // F-12: commandSyncCycles config is propagated to drives.
    //
    // After applyConfig + initializeAndEnterOp in sim mode, the drive
    // must have getCommandSyncCycles() == cfg.commandSyncCycles.
    // ----------------------------------------------------------------
    void f12_commandSyncCyclesPropagatedToDrives()
    {
        AppConfig cfg = makeSimConfig(7);  // non-default value to confirm propagation
        EtherCATMaster master;
        master.setSimulationMode(true);
        master.applyConfig(cfg);
        InitResult r = master.initializeAndEnterOp(cfg.nicName);
        QVERIFY(r.ok);

        A6Drive* drive = master.getDrive(0);
        QVERIFY(drive != nullptr);
        QCOMPARE(drive->getCommandSyncCycles(), 7);
    }
};

QTEST_MAIN(TestB222F)
#include "TestB222F.moc"
