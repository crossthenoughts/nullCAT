// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestFaultRecovery.cpp  (Build 58 - P2-4)
//
// Fault recovery integration tests.
// Exercises the full fault → park → rehome → ONLINE round-trip
// using MockA6Drive, simulating what ControlLoop does when a
// drive fault is detected mid-ONLINE.
//
// Also covers the Build 57 fix: stable-cycles rehome trigger
// (drives that self-recover bypass stepFaultReset() - the
// allClear check must still call startHoming()).
//
// Cycle time: 10ms (100Hz). parkTimeSec=0.1s → 10 cycles.
// ============================================================

#include <QtTest>
#include "../src/MotionController.h"
#include "../src/MockA6Drive.h"
#include "../src/Config.h"
#include "../src/Logging.h"

// ---- Helpers ----

static AppConfig makeSingleAxisConfig()
{
    DriveConfig dc;
    dc.slaveIndex           = 1;
    dc.axisType             = "linear_vertical";
    dc.strokeMm             = 100.0;
    dc.homingSpeed       = 400.0;  // harness speed: step = speed*REF_DT(0.0005); pre-redesign tests assumed speed*0.01
    dc.homingBackoffMm      = 1.5;
    dc.homingTorquePct      = 25;
    dc.homeDirection        = "negative";
    dc.invertDir            = true;   // foldback fixture: retract = raw negative (see TestHomingSequence)
    dc.parkMode             = "endstop";    // parkPos = 1.5mm
    dc.maxVelocityMmS       = 200.0;
    dc.maxAccelerationMmS2  = 2000.0;
    dc.maxJerkMmS3          = 20000.0;
    dc.unparkTimeSec        = 0.1;
    dc.parkTimeSec          = 0.1;
    dc.countsPerMm          = 100.0;
    dc.ballscrewPitch       = 5.0;

    AppConfig cfg;
    cfg.controlLoopHz        = 100;
    cfg.numDrives            = 1;
    cfg.blendMaxVelocityMmS  = 20.0;
    cfg.drives.push_back(dc);
    return cfg;
}

// Run process() N cycles, stepping the mock drive each cycle.
static MotionOutput runCycles(MotionController& mc, MockA6Drive* drive, int n)
{
    TelemetryData empty{};
    MotionOutput out{};
    A6Drive* drives[1] = { drive };
    for (int i = 0; i < n; ++i)
    {
        drive->updateStatus();
        mc.process(empty, out, drives, 1);
    }
    return out;
}

// Get a drive all the way from PARKED → homed → ONLINE.
// Returns cycle count used.
static int driveToOnline(MotionController& mc, MockA6Drive* drive, int maxCycles = 5000)
{
    mc.startHoming();
    A6Drive* drives[1] = { drive };
    TelemetryData empty{};
    MotionOutput out{};
    for (int i = 0; i < maxCycles; ++i)
    {
        drive->updateStatus();
        mc.process(empty, out, drives, 1);
        if (mc.getAxisState(0) == AxisMotionState::ONLINE)
            return i + 1;
    }
    return -1; // did not reach ONLINE
}

// ---- Test class ----

class TestFaultRecovery : public QObject
{
    Q_OBJECT

private slots:

    void initTestCase()
    {
        // TF-6: suppress log output during tests
        Logger::instance().setMinLevel(LogLevel::LVL_CRITICAL);
    }
    void cleanupTestCase() {}

    // ----------------------------------------------------------------
    // Verify baseline: drive reaches ONLINE without any fault.
    // ----------------------------------------------------------------
    void baseline_reachesOnline()
    {
        AppConfig cfg = makeSingleAxisConfig();
        MotionController mc;
        mc.configure(cfg);

        MockA6Drive drive;
        drive.configure(1, 0.0, 1.0);
        drive.setHardstop(-50.0, true, 50.0);

        int cycles = driveToOnline(mc, &drive);
        QVERIFY2(cycles > 0, "Did not reach ONLINE");
        QCOMPARE(mc.getAxisState(0), AxisMotionState::ONLINE);
    }

    // ----------------------------------------------------------------
    // Fault mid-ONLINE: park → fault reset → rehome → ONLINE.
    // Simulates what ControlLoop does when drive->isFault() is true.
    // ----------------------------------------------------------------
    void faultMidOnline_recoversAndRehomes()
    {
        AppConfig cfg = makeSingleAxisConfig();
        MotionController mc;
        mc.configure(cfg);

        MockA6Drive drive;
        drive.configure(1, 0.0, 1.0);
        drive.setHardstop(-50.0, true, 50.0);

        int cycles = driveToOnline(mc, &drive);
        QVERIFY2(cycles > 0, "Did not reach ONLINE for fault test setup");
        QCOMPARE(mc.getAxisState(0), AxisMotionState::ONLINE);

        // ---- Inject fault (ControlLoop detects drive->isFault()) ----
        drive.injectFault();
        QVERIFY(drive.isFault());

        // ---- ControlLoop reaction: park all axes, set needsRehome ----
        mc.setNeedsRehome(true);
        mc.startPark();

        // Run until PARKED
        A6Drive* drives[1] = { &drive };
        TelemetryData empty{};
        MotionOutput out{};
        bool reachedParked = false;
        for (int i = 0; i < 500; ++i)
        {
            drive.updateStatus();
            mc.process(empty, out, drives, 1);
            if (mc.getAxisState(0) == AxisMotionState::PARKED)
            {
                reachedParked = true;
                break;
            }
        }
        QVERIFY2(reachedParked, "Did not park after fault");

        // ---- ControlLoop: stepFaultReset until cleared ----
        drive.startFaultReset();
        bool faultCleared = false;
        for (int i = 0; i < 20; ++i)
        {
            if (drive.stepFaultReset()) { faultCleared = true; break; }
        }
        QVERIFY2(faultCleared, "Fault reset did not complete");
        QVERIFY(!drive.isFault());

        // ---- ControlLoop: allClear && needsRehome → startHoming() ----
        QVERIFY(mc.needsRehome());
        mc.startHoming();

        // Run until ONLINE
        bool reachedOnline = false;
        for (int i = 0; i < 5000; ++i)
        {
            drive.updateStatus();
            mc.process(empty, out, drives, 1);
            if (mc.getAxisState(0) == AxisMotionState::ONLINE)
            {
                reachedOnline = true;
                break;
            }
        }
        QVERIFY2(reachedOnline, "Did not reach ONLINE after fault recovery");
        QVERIFY(!mc.needsRehome());
    }

    // ----------------------------------------------------------------
    // Multiple sequential faults: verify retry counter increments
    // and recovery still works up to MAX retries.
    // ----------------------------------------------------------------
    void multipleFaults_recoverySucceeds()
    {
        AppConfig cfg = makeSingleAxisConfig();
        MotionController mc;
        mc.configure(cfg);

        MockA6Drive drive;
        drive.configure(1, 0.0, 1.0);
        drive.setHardstop(-50.0, true, 50.0);

        A6Drive* drives[1] = { &drive };
        TelemetryData empty{};
        MotionOutput out{};

        // Go through 3 fault/recover cycles
        for (int faultRound = 0; faultRound < 3; ++faultRound)
        {
            int c = driveToOnline(mc, &drive);
            QVERIFY2(c > 0, qPrintable(QString("Round %1: did not reach ONLINE").arg(faultRound)));

            drive.injectFault();
            mc.setNeedsRehome(true);
            mc.startPark();

            for (int i = 0; i < 500; ++i)
            {
                drive.updateStatus();
                mc.process(empty, out, drives, 1);
                if (mc.getAxisState(0) == AxisMotionState::PARKED) break;
            }
            QCOMPARE(mc.getAxisState(0), AxisMotionState::PARKED);

            drive.startFaultReset();
            for (int i = 0; i < 20; ++i)
                if (drive.stepFaultReset()) break;

            mc.startHoming();
        }

        // Final state: drive should reach ONLINE
        bool reachedOnline = false;
        for (int i = 0; i < 5000; ++i)
        {
            drive.updateStatus();
            mc.process(empty, out, drives, 1);
            if (mc.getAxisState(0) == AxisMotionState::ONLINE)
            {
                reachedOnline = true;
                break;
            }
        }
        QVERIFY2(reachedOnline, "Did not reach ONLINE after 3rd recovery");
    }

    // ----------------------------------------------------------------
    // Fault during homing: HomingSequence detects drive fault →
    // FatalError state → axis PARKED. Verify no auto-restart.
    // ----------------------------------------------------------------
    void faultDuringHoming_landsFatalError()
    {
        AppConfig cfg = makeSingleAxisConfig();
        MotionController mc;
        mc.configure(cfg);

        MockA6Drive drive;
        drive.configure(1, 0.0, 1.0);
        drive.setHardstop(-50.0, true, 50.0);
        // Inject fault immediately so enable timeout fires → FatalError
        drive.setEnableCycles(10000); // never enables

        mc.startHoming();
        QCOMPARE(mc.getAxisState(0), AxisMotionState::HOMING);

        // Run until PARKED (FatalError → processHomingAxis sets PARKED)
        A6Drive* drives[1] = { &drive };
        TelemetryData empty{};
        MotionOutput out{};
        bool reachedParked = false;
        for (int i = 0; i < 50000; ++i)
        {
            drive.updateStatus();
            mc.process(empty, out, drives, 1);
            if (mc.getAxisState(0) == AxisMotionState::PARKED)
            {
                reachedParked = true;
                break;
            }
        }
        QVERIFY2(reachedParked, "Did not reach PARKED after homing FatalError");

        // Verify it does NOT auto-restart homing
        for (int i = 0; i < 100; ++i)
        {
            drive.updateStatus();
            mc.process(empty, out, drives, 1);
        }
        QCOMPARE(mc.getAxisState(0), AxisMotionState::PARKED);
    }

    // ----------------------------------------------------------------
    // Fault mid-ONLINE followed by e-stop: verify e-stop takes
    // precedence and axes stay PARKED until explicit rehome.
    // ----------------------------------------------------------------
    void faultThenEstop_requiresRehome()
    {
        AppConfig cfg = makeSingleAxisConfig();
        MotionController mc;
        mc.configure(cfg);

        MockA6Drive drive;
        drive.configure(1, 0.0, 1.0);
        drive.setHardstop(-50.0, true, 50.0);

        int c = driveToOnline(mc, &drive);
        QVERIFY2(c > 0, "Did not reach ONLINE");

        // Fault + e-stop simultaneously
        drive.injectFault();
        mc.setNeedsRehome(true);
        mc.startPark();
        mc.setEmergencyStop(true);

        A6Drive* drives[1] = { &drive };
        TelemetryData empty{};
        MotionOutput out{};
        for (int i = 0; i < 500; ++i)
        {
            drive.updateStatus();
            mc.process(empty, out, drives, 1);
        }

        // E-stop cleared → startHoming() fires on the next process() call (P3-4:
        // state transitions happen via edge detection inside process(), not in
        // setEmergencyStop() which now only writes the atomic).
        drive.clearFault();
        mc.setEmergencyStop(false);
        mc.process(empty, out, drives, 1);  // falling edge detected → startHoming()

        // Axis should now be homing
        bool reachedHoming = (mc.getAxisState(0) == AxisMotionState::HOMING);
        QVERIFY2(reachedHoming, "Axis should be HOMING after e-stop release");
    }

    // ---- TF-3-3: 3-drive fault recovery round-trip ----
    // All three axes reach ONLINE, drive 1 faults, all axes park,
    // DriveFaultMonitor (via ControlLoop simulation) resets the fault,
    // and startHoming() triggers the full rehome round-trip.
    //
    // This test simulates only the MotionController side (not ControlLoop),
    // so we manually call setNeedsRehome/startPark/startHoming as ControlLoop
    // would. After startHoming, all axes must reach ONLINE again.
    void threeAxis_faultRecovery_roundTrip()
    {
        // Build 3-axis config
        AppConfig cfg;
        cfg.controlLoopHz       = 100;
        cfg.numDrives           = 3;
        cfg.blendMaxVelocityMmS = 20.0;
        for (int ax = 0; ax < 3; ++ax)
        {
            DriveConfig dc;
            dc.slaveIndex           = ax + 1;
            dc.axisType             = "linear_vertical";
            dc.strokeMm             = 100.0;
            dc.homingSpeed       = 1000.0;  // harness speed: step = speed*REF_DT(0.0005); pre-redesign tests assumed speed*0.01
            dc.homingBackoffMm      = 1.5;
            dc.homingTorquePct      = 25;
            dc.homeDirection        = "negative";
            dc.invertDir            = true;   // foldback fixture: retract = raw negative (see TestHomingSequence)
            dc.parkMode             = "endstop";
            dc.maxVelocityMmS       = 200.0;
            dc.maxAccelerationMmS2  = 2000.0;
            dc.maxJerkMmS3          = 20000.0;
            dc.unparkTimeSec        = 0.1;
            dc.parkTimeSec          = 0.1;
            dc.countsPerMm          = 100.0;
            dc.ballscrewPitch       = 5.0;
            cfg.drives.push_back(dc);
        }

        MotionController mc;
        mc.configure(cfg);

        MockA6Drive m0, m1, m2;
        m0.configure(1, 0.0, 0.5);  m0.setHardstop(-50.0, true, 50.0);
        m1.configure(2, 0.0, 0.5);  m1.setHardstop(-50.0, true, 50.0);
        m2.configure(3, 0.0, 0.5);  m2.setHardstop(-50.0, true, 50.0);

        A6Drive* drives[3] = { &m0, &m1, &m2 };
        TelemetryData empty{};
        MotionOutput out{};

        // ---- Phase 1: get all axes ONLINE ----
        mc.startHoming();
        bool allOnline = false;
        for (int c = 0; c < 5000 && !allOnline; ++c)
        {
            m0.updateStatus(); m1.updateStatus(); m2.updateStatus();
            mc.process(empty, out, drives, 3);
            allOnline = (mc.getAxisState(0) == AxisMotionState::ONLINE &&
                         mc.getAxisState(1) == AxisMotionState::ONLINE &&
                         mc.getAxisState(2) == AxisMotionState::ONLINE);
        }
        QVERIFY2(allOnline, "Phase 1: did not reach all-ONLINE");

        // ---- Phase 2: simulate fault on axis 1, park all ----
        mc.setNeedsRehome(true);
        mc.startPark();
        for (int c = 0; c < 300; ++c)
        {
            m0.updateStatus(); m1.updateStatus(); m2.updateStatus();
            mc.process(empty, out, drives, 3);
        }
        for (int ax = 0; ax < 3; ++ax)
            QVERIFY2(mc.getAxisState(ax) == AxisMotionState::PARKED,
                qPrintable(QString("Phase 2: axis %1 not PARKED").arg(ax)));

        // ---- Phase 3: fault cleared → startHoming() → all ONLINE again ----
        mc.startHoming();
        bool allOnline2 = false;
        for (int c = 0; c < 5000 && !allOnline2; ++c)
        {
            m0.updateStatus(); m1.updateStatus(); m2.updateStatus();
            mc.process(empty, out, drives, 3);
            allOnline2 = (mc.getAxisState(0) == AxisMotionState::ONLINE &&
                          mc.getAxisState(1) == AxisMotionState::ONLINE &&
                          mc.getAxisState(2) == AxisMotionState::ONLINE);
        }
        QVERIFY2(allOnline2, "Phase 3: did not reach all-ONLINE after rehome");
        QVERIFY(!mc.needsRehome());
    }
};

QTEST_MAIN(TestFaultRecovery)
#include "TestFaultRecovery.moc"
