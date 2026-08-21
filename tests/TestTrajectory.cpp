// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestTrajectory.cpp  (Build 63 - TF-4)
//
// Tests for the s-curve trajectory planner exercised indirectly
// through MotionController::process() while axes are ONLINE.
//
// stepTrajectory() is private; we observe its effect through
// MotionOutput::positions[] each cycle.
//
// Cycle time: 10ms (100Hz).
// Axis config: strokeMm=100, centerPos=50, min=0, max=100.
// maxVelocityMmS=200 → max Δpos per cycle = 2.0mm
// blendMaxVelocityMmS=20 → max Δpos during BLENDING = 0.2mm/cycle
//
// Tests:
//   TF-4-1: Velocity not exceeded each cycle during motion
//   TF-4-2: Position converges to target within tolerance
//   TF-4-3: Position limit clamping (output never exits [0, strokeMm])
//   TF-4-4: velCap honoured during BLENDING phase
// ============================================================

#include <QtTest>
#include "../src/MotionController.h"
#include "../src/MockA6Drive.h"
#include "../src/Config.h"
#include <cmath>
#include <algorithm>

// ---- Helpers ----

static AppConfig makeTrajectoryConfig(double maxVelMmS     = 200.0,
                                      double blendVelMmS   = 20.0,
                                      double strokeMm      = 100.0,
                                      double cycleHz       = 100.0)
{
    DriveConfig dc;
    dc.slaveIndex           = 1;
    dc.axisType             = "linear_vertical";
    dc.strokeMm             = strokeMm;
    dc.homingSpeed       = 50.0;    // fast homing
    dc.homingBackoffMm      = 1.5;
    dc.homingTorquePct      = 25;
    dc.homeDirection        = "negative";
    dc.parkMode             = "endstop";
    dc.maxVelocityMmS       = maxVelMmS;
    dc.maxAccelerationMmS2  = 2000.0;
    dc.maxJerkMmS3          = 20000.0;
    dc.unparkTimeSec        = 0.1;
    dc.parkTimeSec          = 0.1;
    dc.countsPerMm          = 100.0;
    dc.ballscrewPitch       = 5.0;

    AppConfig cfg;
    cfg.controlLoopHz       = static_cast<int>(cycleHz);
    cfg.numDrives           = 1;
    cfg.blendMaxVelocityMmS = blendVelMmS;
    cfg.drives.push_back(dc);
    return cfg;
}

// Build a TelemetryData packet commanding an offset (mm relative to center)
// as a 16-bit wire value -- the ONLY scaling the engine accepts since 0.9.3:
// 0..65535 with 32767 = centerPos, full scale = half a stroke. The fixtures
// in this file all use strokeMm=100, so mm-from-centre maps as
// 32767 * (1 + mm/50). Offsets beyond +-50mm produce out-of-range wire
// values; the engine clamps the resulting target to [0, strokeMm], which is
// exactly what the clamping test exercises.
static TelemetryData makeMotionPacket(double mmFromCenter)
{
    TelemetryData d;
    d.valid        = true;
    d.numPositions = 1;
    d.positions[0] = 32767.0 * (1.0 + mmFromCenter / 50.0);
    d.packetType   = TelemetryPacketType::Motion;
    return d;
}

// Get axis to ONLINE. Returns cycle count used, or -1 on failure.
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
    return -1;
}

class TestTrajectory : public QObject
{
    Q_OBJECT

private slots:

    void initTestCase()
    {
        // TF-6: suppress log output during tests
        Logger::instance().setMinLevel(LogLevel::LVL_CRITICAL);
    }

    // ---- TF-4-1: Velocity not exceeded each cycle ----
    // Feed a step input (center → max stroke) while ONLINE.
    // The per-cycle position delta must never exceed maxVelocityMmS * dt.
    void velocityNotExceeded()
    {
        const double maxVelMmS = 200.0;
        const double dt        = 0.01;           // 100Hz
        const double maxDelta  = maxVelMmS * dt; // 2.0mm per cycle (hard limit)

        AppConfig cfg = makeTrajectoryConfig(maxVelMmS);
        MotionController mc;
        mc.configure(cfg);

        MockA6Drive mock;
        mock.configure(1, 0.0, 0.5);
        mock.setHardstop(50.0, false, 50.0);   // inline fixture: retract = raw positive, stop at +50 (reversed frame)

        int c = driveToOnline(mc, &mock, 5000);
        QVERIFY2(c > 0, "Did not reach ONLINE before velocity test");

        A6Drive* drives[1] = { &mock };
        MotionOutput out{};

        // Command a step to the maximum positive position (raw=+50 → center+50=100mm)
        TelemetryData cmd = makeMotionPacket(50.0);

        double prevPos = -1.0;
        double maxObserved = 0.0;

        for (int i = 0; i < 1000; ++i)
        {
            mock.updateStatus();
            mc.process(cmd, out, drives, 1);

            double pos = out.positions[0];
            if (prevPos >= 0.0)
            {
                double delta = std::abs(pos - prevPos);
                maxObserved = std::max(maxObserved, delta);
                // Allow one LSB of floating-point slop
                QVERIFY2(delta <= maxDelta + 1e-6,
                    qPrintable(QString("Velocity exceeded: delta=%1mm (max=%2mm) at cycle %3")
                        .arg(delta, 0, 'f', 4).arg(maxDelta, 0, 'f', 4).arg(i)));
            }
            prevPos = pos;
        }
    }

    // ---- TF-4-2: Position convergence ----
    // After commanding a target, the trajectory must converge to within
    // 0.1mm of the setpoint within 3 seconds (300 cycles at 100Hz).
    void positionConvergesToTarget()
    {
        AppConfig cfg = makeTrajectoryConfig();
        MotionController mc;
        mc.configure(cfg);

        MockA6Drive mock;
        mock.configure(1, 0.0, 0.5);
        mock.setHardstop(50.0, false, 50.0);   // inline fixture: retract = raw positive, stop at +50 (reversed frame)

        int c = driveToOnline(mc, &mock, 5000);
        QVERIFY2(c > 0, "Did not reach ONLINE");

        A6Drive* drives[1] = { &mock };
        MotionOutput out{};

        // Command center+40mm = 90mm (raw=+40 from center of 50mm)
        const double targetRaw = 40.0;   // raw mm from center
        const double centerPos = 50.0;   // set from strokeMm=100
        const double expected  = centerPos + targetRaw; // 90mm

        TelemetryData cmd = makeMotionPacket(targetRaw);

        for (int i = 0; i < 300; ++i)
        {
            mock.updateStatus();
            mc.process(cmd, out, drives, 1);
        }

        double finalPos = out.positions[0];
        double err = std::abs(finalPos - expected);
        QVERIFY2(err < 0.1,
            qPrintable(QString("Did not converge: final=%1mm, target=%2mm, err=%3mm")
                .arg(finalPos, 0, 'f', 3).arg(expected, 0, 'f', 3).arg(err, 0, 'f', 3)));
    }

    // ---- TF-4-3: Position limit clamping ----
    // Command a position beyond strokeMm (raw = 100mm from center → clamped to maxPos=100).
    // Output must stay within [0, strokeMm] on every cycle.
    void positionLimitClamping()
    {
        const double strokeMm = 100.0;
        AppConfig cfg = makeTrajectoryConfig(200.0, 20.0, strokeMm);
        MotionController mc;
        mc.configure(cfg);

        MockA6Drive mock;
        mock.configure(1, 0.0, 0.5);
        mock.setHardstop(50.0, false, 50.0);   // inline fixture: retract = raw positive, stop at +50 (reversed frame)

        int c = driveToOnline(mc, &mock, 5000);
        QVERIFY2(c > 0, "Did not reach ONLINE");

        A6Drive* drives[1] = { &mock };
        MotionOutput out{};

        // Command far beyond maxPos: +500mm from centre is an out-of-range
        // 16-bit wire value; the engine clamps the target to maxPos.
        TelemetryData cmd = makeMotionPacket(500.0);  // clamped at maxPos=100

        for (int i = 0; i < 500; ++i)
        {
            mock.updateStatus();
            mc.process(cmd, out, drives, 1);

            double pos = out.positions[0];
            QVERIFY2(pos >= 0.0 - 1e-6 && pos <= strokeMm + 1e-6,
                qPrintable(QString("Position %1mm outside [0, %2mm] at cycle %3")
                    .arg(pos, 0, 'f', 3).arg(strokeMm, 0, 'f', 1).arg(i)));
        }
    }

    // ---- All-zeros frame is no-data, not a command ----
    // In 16-bit, 0 means full deflection to one end -- but a frame where
    // EVERY channel is exactly 0 is the signature of a telemetry tool idling
    // at menu, not a motion command. Reading it literally would slam the
    // whole rig to one end of stroke. The engine treats such a frame as
    // no-data: the axis holds (stale-telemetry phase 1) instead of moving.
    // A frame with ANY nonzero channel remains a real command, including a
    // genuine single-axis zero alongside live channels.
    void allZerosFrame_isNoData_notFullDeflection()
    {
        AppConfig cfg = makeTrajectoryConfig();
        MotionController mc;
        mc.configure(cfg);

        MockA6Drive mock;
        mock.configure(1, 0.0, 0.5);
        mock.setHardstop(50.0, false, 50.0);   // inline fixture (see above)

        int c = driveToOnline(mc, &mock, 5000);
        QVERIFY2(c > 0, "Did not reach ONLINE");

        A6Drive* drives[1] = { &mock };
        MotionOutput out{};

        // Track at centre on real frames first.
        TelemetryData centre = makeMotionPacket(0.0);   // wire 32767
        for (int i = 0; i < 300; ++i) { mock.updateStatus(); mc.process(centre, out, drives, 1); }
        const double before = out.positions[0];
        QVERIFY2(std::abs(before - 50.0) < 0.5,
            qPrintable(QString("setup: expected ~centre, got %1").arg(before)));

        // Now feed all-zero frames for one second of cycles. A literal 16-bit
        // reading would command 0mm (full one end); no-data must hold.
        TelemetryData zeros;
        zeros.valid        = true;
        zeros.numPositions = 1;
        zeros.positions[0] = 0.0;
        zeros.packetType   = TelemetryPacketType::Motion;
        for (int i = 0; i < 100; ++i) { mock.updateStatus(); mc.process(zeros, out, drives, 1); }

        QVERIFY2(std::abs(out.positions[0] - before) < 0.5,
            qPrintable(QString("all-zeros frame moved the axis: %1 -> %2 (must hold)")
                .arg(before).arg(out.positions[0])));
    }

    // ---- TF-4-4: velCap honoured during BLENDING ----
    // Right after homing, the axis enters BLENDING state and stepTrajectory()
    // is called with velCap = blendMaxVelocityMmS = 20mm/s.
    // Per-cycle delta during BLENDING must not exceed 20 * 0.01 = 0.2mm.
    void velCap_honoured_duringBlending()
    {
        const double blendVelMmS = 20.0;
        const double dt          = 0.01;
        const double maxDelta    = blendVelMmS * dt;  // 0.2mm

        AppConfig cfg = makeTrajectoryConfig(200.0, blendVelMmS);
        MotionController mc;
        mc.configure(cfg);

        MockA6Drive mock;
        mock.configure(1, 0.0, 0.5);
        mock.setHardstop(50.0, false, 50.0);   // inline fixture: retract = raw positive, stop at +50 (reversed frame)

        // Homing puts the axis in BLENDING after completing.
        mc.startHoming();

        A6Drive* drives[1] = { &mock };
        TelemetryData empty{};
        MotionOutput out{};

        double prevPos  = -1.0;
        bool   seenBlending = false;
        bool   leftBlending = false;
        double maxBlendDelta = 0.0;

        for (int i = 0; i < 3000 && !leftBlending; ++i)
        {
            mock.updateStatus();
            mc.process(empty, out, drives, 1);

            AxisMotionState st = mc.getAxisState(0);

            if (st == AxisMotionState::BLENDING)
            {
                seenBlending = true;
                double pos = out.positions[0];
                if (prevPos >= 0.0)
                {
                    double delta = std::abs(pos - prevPos);
                    maxBlendDelta = std::max(maxBlendDelta, delta);
                    QVERIFY2(delta <= maxDelta + 1e-6,
                        qPrintable(QString("BLENDING velocity exceeded: delta=%1mm (cap=%2mm) at cycle %3")
                            .arg(delta, 0, 'f', 4).arg(maxDelta, 0, 'f', 4).arg(i)));
                }
                prevPos = pos;
            }
            else if (seenBlending)
            {
                leftBlending = true;
            }
        }

        QVERIFY2(seenBlending, "BLENDING state was never observed -- check homing setup");
    }
};

QTEST_MAIN(TestTrajectory)
#include "TestTrajectory.moc"
