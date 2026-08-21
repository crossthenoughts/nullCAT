// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestMotionController.cpp  (Build 55 - P2-3)
//
// State machine tests for MotionController.
// Covers homing completion, FatalError handling, ESTOPPING,
// and PARKED behaviour. Uses MockA6Drive throughout.
//
// Cycle time: 10ms (100Hz). ESTOP_RAMP_SEC = 0.5s → 50 cycles.
// parkPos: parkMode="endstop" → ac.parkPos = homingBackoffMm = 1.5mm.
// ============================================================

#include <QtTest>
#include "../src/MotionController.h"
#include "../src/MockA6Drive.h"
#include "../src/Config.h"
#include "../src/Logging.h"

// ---- Helpers ----

// Build a minimal AppConfig for a single linear axis.
// parkMode="endstop" → parkPos = homingBackoffMm = 1.5mm.
static AppConfig makeSingleAxisConfig(double cycleHz   = 100.0,
                                      double strokeMm  = 100.0,
                                      double speedMmS  = 400.0,   // 0.2mm/cycle in-harness (see homingStepMm)
                                      double backoffMm = 1.5,
                                      int    torquePct = 25)
{
    DriveConfig dc;
    dc.slaveIndex           = 1;
    dc.axisType             = "linear_vertical";
    dc.strokeMm             = strokeMm;
    dc.homingSpeed       = speedMmS;
    dc.homingBackoffMm      = backoffMm;
    dc.homingTorquePct      = torquePct;
    dc.homeDirection        = "negative";
    dc.invertDir            = true;   // foldback fixture: retract = raw negative (see TestHomingSequence)
    dc.parkMode             = "endstop";    // parkPos = backoffMm = 1.5mm
    dc.maxVelocityMmS       = 200.0;
    dc.maxAccelerationMmS2  = 2000.0;
    dc.maxJerkMmS3          = 20000.0;
    dc.unparkTimeSec        = 0.1;
    dc.parkTimeSec          = 0.1;
    dc.countsPerMm          = 100.0;
    dc.ballscrewPitch       = 5.0;

    AppConfig cfg;
    cfg.controlLoopHz = static_cast<int>(cycleHz);
    cfg.numDrives     = 1;
    cfg.drives.push_back(dc);
    return cfg;
}

// Run process() N cycles with a single drive.
static MotionOutput runCycles(MotionController& mc, A6Drive* drive, int n)
{
    TelemetryData empty{};
    MotionOutput out{};
    A6Drive* drives[1] = { drive };
    for (int i = 0; i < n; ++i)
        mc.process(empty, out, drives, 1);
    return out;
}

// Run until axis reaches targetState or maxCycles expires.
// Returns true if targetState reached within maxCycles.
static bool runUntilState(MotionController& mc, A6Drive* drive,
                           AxisMotionState targetState, int maxCycles)
{
    TelemetryData empty{};
    A6Drive* drives[1] = { drive };
    for (int i = 0; i < maxCycles; ++i)
    {
        MotionOutput out{};
        mc.process(empty, out, drives, 1);
        if (mc.getAxisState(0) == targetState) return true;
    }
    return false;
}

// ---- Config re-apply helpers ------------------------------------------------
// configure() branches on BOTH axisType ("belt" vs everything else) and mode
// ("torque" / "pp" / csp), and the parkPos it derives additionally depends on
// parkMode. The re-apply tests below sweep that whole matrix rather than belts
// alone, because a regression in any one branch looks identical from the UI:
// "I changed a setting and it did nothing until I restarted the app."
static AppConfig makeAxisConfig(const char* axisType, const char* mode,
                                const char* parkMode, double strokeMm,
                                double backoffMm, double torqueMinPct = 5.0,
                                double torqueMaxPct = 50.0)
{
    DriveConfig dc;
    dc.slaveIndex           = 1;
    dc.invertDir            = true;   // foldback fixture: retract = raw negative (see TestHomingSequence)
    dc.axisType             = axisType;
    dc.mode                 = mode;
    dc.parkMode             = parkMode;
    dc.strokeMm             = strokeMm;
    dc.homingBackoffMm      = backoffMm;
    dc.homeDirection        = "negative";
    dc.homingSpeed       = 400.0;
    dc.homingTorquePct      = 25;
    dc.maxVelocityMmS       = 200.0;
    dc.maxAccelerationMmS2  = 2000.0;
    dc.maxJerkMmS3          = 20000.0;
    dc.unparkTimeSec        = 0.1;
    dc.parkTimeSec          = 0.1;
    dc.onlineHoldTimeoutSec = 5.0;
    dc.countsPerMm          = 100.0;
    dc.torqueMinPct         = torqueMinPct;
    dc.torqueMaxPct         = torqueMaxPct;

    AppConfig cfg;
    cfg.controlLoopHz       = 100;
    cfg.numDrives           = 1;
    cfg.blendTimeSec        = 0.1;
    cfg.blendMaxVelocityMmS = 20.0;
    cfg.drives.push_back(dc);
    return cfg;
}

// Cycles spent searching before the axis reaches ONLINE, under whichever config
// is currently in force. parkPos is NOT usable as the probe here: a PARKED axis
// holds its ACTUAL position while un-homed, and holds the home position (not
// parkPos) once homed (B222P). Homing speed, by contrast, flows straight through
// configure() -> HomingSequence::configure(), so a MotionController still holding
// a stale AxisConfig searches at the old speed and the cycle count does not move.
// Returns -1 if ONLINE is never reached.
static int cyclesToOnline(MotionController& mc, MockA6Drive& mock, int maxCycles = 20000)
{
    mc.startHoming();
    TelemetryData empty{};
    A6Drive* drives[1] = { &mock };
    for (int i = 0; i < maxCycles; ++i)
    {
        MotionOutput out{};
        mc.process(empty, out, drives, 1);
        if (mc.getAxisState(0) == AxisMotionState::ONLINE) return i;
    }
    return -1;
}

// Settled belt tension under the config in force. Belts tension ONLY via the
// command queue (the path every real surface uses), never via unpark.
static double settledBeltTorque(MotionController& mc)
{
    MotionCommand c;
    c.type = MotionCommand::Type::TensionBelts;
    mc.enqueueCommand(c);

    TelemetryData sd{};
    sd.valid        = true;
    // Belt demand zero, plus a 16-bit centre on a second channel so the frame
    // is not ALL-zero (the engine treats an all-zero frame as no-data).
    sd.numPositions = 2;
    sd.positions[0] = 0.0;
    sd.positions[1] = 32767.0;
    sd.packetType   = TelemetryPacketType::Motion;

    MotionOutput out{};
    for (int i = 0; i < 200; ++i) { out = MotionOutput{}; mc.process(sd, out, nullptr, 0); }
    return out.torques[0];
}

// ============================================================

class TestMotionController : public QObject
{
    Q_OBJECT

private slots:

    void initTestCase()
    {
        // TF-6: suppress log output during tests
        Logger::instance().setMinLevel(LogLevel::LVL_CRITICAL);
    }

    // ============================================================
    // Config re-apply: a second configure() fully replaces the first.
    //
    // Both EtherCAT init paths -- the Qt Initialize button (MainWindow) and
    // /api/init (WebServer) -- call configure() with the reloaded AppConfig, so
    // that a rig.json edit saved while EtherCAT was up reaches the engine on
    // Stop -> Initialize rather than needing an application restart. That is the
    // contract the config page now states; these tests pin the engine behaviour
    // it rests on, across every axis type and drive mode.
    // ============================================================

    // Position axes, per (axisType, mode, parkMode). Halving the search speed
    // must lengthen the search: if the re-apply is lost, the controller keeps
    // homing at the original speed and the count is unchanged.
    void reapply_positionAxis_takesNewHomingSpeed_data()
    {
        QTest::addColumn<QString>("axisType");
        QTest::addColumn<QString>("mode");
        QTest::addColumn<QString>("parkMode");

        QTest::newRow("linear_vertical/csp/endstop")   << "linear_vertical"   << "csp" << "endstop";
        QTest::newRow("linear_horizontal/csp/center")  << "linear_horizontal" << "csp" << "center";
        QTest::newRow("linear_vertical/pp/endstop")    << "linear_vertical"   << "pp"  << "endstop";
        QTest::newRow("linear_horizontal/pp/center")   << "linear_horizontal" << "pp"  << "center";
    }

    void reapply_positionAxis_takesNewHomingSpeed()
    {
        QFETCH(QString, axisType);
        QFETCH(QString, mode);
        QFETCH(QString, parkMode);

        const QByteArray at = axisType.toUtf8(), md = mode.toUtf8(), hm = parkMode.toUtf8();

        AppConfig fastCfg = makeAxisConfig(at.constData(), md.constData(), hm.constData(), 100.0, 1.5);
        fastCfg.drives[0].homingSpeed = 400.0;

        MotionController mc;
        mc.configure(fastCfg);

        MockA6Drive fastMock;
        fastMock.configure(1, 0.0, 0.05);
        fastMock.setHardstop(-2.0, true, 50.0);
        const int fastCycles = cyclesToOnline(mc, fastMock);
        QVERIFY2(fastCycles > 0, "axis never reached ONLINE under the first config");

        AppConfig slowCfg = fastCfg;
        slowCfg.drives[0].homingSpeed = 100.0;        // 4x slower search
        mc.configure(slowCfg);

        MockA6Drive slowMock;
        slowMock.configure(1, 0.0, 0.05);
        slowMock.setHardstop(-2.0, true, 50.0);
        const int slowCycles = cyclesToOnline(mc, slowMock);
        QVERIFY2(slowCycles > 0, "axis never reached ONLINE after re-apply");

        QVERIFY2(slowCycles > fastCycles,
                 qPrintable(QString("re-applied homing speed had no effect: %1 cycles then %2")
                            .arg(fastCycles).arg(slowCycles)));
    }

    // The reported symptom was belt settings specifically: tension limits are
    // consumed only through configure(), so a stale AxisConfig silently keeps
    // commanding the old tension.
    void reapply_beltTorque_takesNewTensionLimits()
    {
        MotionController mc;
        mc.setEmergencyStop(false);

        mc.configure(makeAxisConfig("belt", "torque", "endstop", 100.0, 1.5, 5.0, 50.0));
        const double lowFloor = settledBeltTorque(mc);

        mc.configure(makeAxisConfig("belt", "torque", "endstop", 100.0, 1.5, 20.0, 50.0));
        const double highFloor = settledBeltTorque(mc);

        QVERIFY2(highFloor > lowFloor,
                 qPrintable(QString("belt tension floor did not re-apply: %1 then %2")
                            .arg(lowFloor).arg(highFloor)));
    }

    // Belts are marked homed without a search; position axes must run the real
    // one. Switching axisType therefore has to rebind that branch too -- a stale
    // AxisConfig would leave a re-typed axis claiming a home reference it has
    // never established.
    void reapply_switchingAxisTypeRebindsHomingBranch()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.setHardstop(-2.0, true, 50.0);

        MotionController mc;
        mc.configure(makeAxisConfig("belt", "torque", "endstop", 100.0, 3.0));
        mc.startHoming();
        runCycles(mc, &mock, 5);
        QVERIFY2(mc.isAxisHomed(0), "belt should be marked homed without a search");

        mc.configure(makeAxisConfig("linear_vertical", "csp", "endstop", 100.0, 3.0));
        QVERIFY2(!mc.isAxisHomed(0), "configure() must clear the home reference");

        mc.startHoming();
        QCOMPARE((int)mc.getAxisState(0), (int)AxisMotionState::HOMING);
    }

    // Why both init paths guard the call on the loop being stopped: configure()
    // reseats every axis and drops the home reference. Pin it so the guard's
    // rationale cannot quietly stop being true.
    void reapply_resetsEveryAxisToParkedAndUnhomed()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.setHardstop(-2.0, true, 50.0);

        MotionController mc;
        mc.configure(makeSingleAxisConfig(100.0));
        mc.startHoming();
        QVERIFY(runUntilState(mc, &mock, AxisMotionState::ONLINE, 5000));
        QVERIFY(mc.isAxisHomed(0));

        mc.configure(makeSingleAxisConfig(100.0));
        QCOMPARE((int)mc.getAxisState(0), (int)AxisMotionState::PARKED);
        QVERIFY(!mc.isAxisHomed(0));
        QVERIFY(mc.needsRehome());
    }

    // ============================================================
    // Home frame direction: an inline axis (invertDir=false, retract = raw
    // positive) homes against a high-raw stop, and everything afterwards must
    // move AWAY from that stop. On pre-0.9.2 code the engineering frame had no
    // sign, so this exact sequence homed correctly and then commanded the
    // unpark HALF A STROKE THROUGH the hardstop -- the field failure that
    // motivated the frame fix. These two tests fail on that code.
    // ============================================================

    void inlineAxis_unparkMovesAwayFromStop_neverThroughIt()
    {
        AppConfig cfg = makeAxisConfig("linear_vertical", "csp", "endstop", 100.0, 1.5);
        cfg.drives[0].invertDir = false;            // inline: retract = raw positive

        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.setHardstop(2.0, /*isMinLimit=*/false, 50.0);   // retracted stop at +2mm raw

        MotionController mc;
        mc.configure(cfg);
        mc.startHoming();

        // Home -> auto-unpark -> ONLINE, tracking the peak raw excursion.
        // Unlike the state-only tests, this one closes the loop the way
        // ControlLoop does: motion output is applied to the drive via the
        // frame-aware setTargetPosition() (homing writes its own raw targets,
        // so output is applied only once the axis is homed), and updateStatus()
        // lets the mock chase, so the mock's RAW position is the machine truth.
        TelemetryData empty{};
        A6Drive* drives[1] = { &mock };
        double maxRaw = -1e9;
        bool online = false;
        for (int i = 0; i < 8000 && !online; ++i)
        {
            mock.updateStatus();
            MotionOutput out{};
            mc.process(empty, out, drives, 1);
            if (mc.isAxisHomed(0)) mock.setTargetPosition(out.positions[0]);
            maxRaw = std::max(maxRaw, mock.getActualPositionRaw());
            online = (mc.getAxisState(0) == AxisMotionState::ONLINE);
        }
        QVERIFY2(online, "inline axis never reached ONLINE");

        // Let the unpark/blend settle, still tracking the excursion. The mock
        // chases at 0.05mm/cycle, so the ~50mm move to centre needs ~1000
        // cycles; 1500 gives margin.
        for (int i = 0; i < 1500; ++i)
        {
            mock.updateStatus();
            MotionOutput out{};
            mc.process(empty, out, drives, 1);
            mock.setTargetPosition(out.positions[0]);
            maxRaw = std::max(maxRaw, mock.getActualPositionRaw());
        }

        // Never through the stop (homing presses it by design; nothing after may).
        QVERIFY2(maxRaw < 2.0 + 0.6,
                 qPrintable(QString("axis passed the hardstop: peak raw %1 vs stop 2.0").arg(maxRaw)));

        // And genuinely AWAY from it: centre = eng 50 = raw homeOffset - 50 = -49.5.
        const double raw = mock.getActualPositionRaw();
        QVERIFY2(raw < -40.0,
                 qPrintable(QString("axis did not move away from the stop: raw %1, want ~-49.5").arg(raw)));
        QVERIFY2(std::abs(mock.getActualPosition() - 50.0) < 2.0,
                 qPrintable(QString("eng position %1, want ~50 (centre)").arg(mock.getActualPosition())));
    }

    void inlineAxis_driveLimits_onCorrectSideOfStop()
    {
        AppConfig cfg = makeAxisConfig("linear_vertical", "csp", "endstop", 100.0, 1.5);
        cfg.drives[0].invertDir = false;

        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.setHardstop(2.0, /*isMinLimit=*/false, 50.0);
        mock.resetLimitsTracker();

        MotionController mc;
        mc.configure(cfg);
        mc.startHoming();
        QVERIFY(runUntilState(mc, &mock, AxisMotionState::ONLINE, 8000));

        // Post-homing raw clamp window must sit AWAY from the stop:
        // [homeOffset - stroke, homeOffset] = ~[-99.5, 0.5]. Pre-0.9.2 this was
        // [homeOffset, homeOffset + stroke] -- entirely on the far side of the
        // stop, disarming the drive-level overtravel clamp exactly where it
        // was needed.
        QVERIFY(mock.getSetLimitsCallCount() >= 1);
        QVERIFY2(std::abs(mock.getLastSetLimitsMax() - 0.5) < 0.3,
                 qPrintable(QString("limits max %1, want ~0.5").arg(mock.getLastSetLimitsMax())));
        QVERIFY2(std::abs(mock.getLastSetLimitsMin() - (-99.5)) < 0.3,
                 qPrintable(QString("limits min %1, want ~-99.5").arg(mock.getLastSetLimitsMin())));
    }

    // ---- P2-3-1: configure() starts PARKED, startHoming() → HOMING ----
    void initialStateIsParked_startHomingTransitionsToHoming()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);

        MotionController mc;
        mc.configure(makeSingleAxisConfig());

        // After configure(), axis is always PARKED (not auto-homed)
        QCOMPARE((int)mc.getAxisState(0), (int)AxisMotionState::PARKED);

        mc.startHoming();
        QCOMPARE((int)mc.getAxisState(0), (int)AxisMotionState::HOMING);
    }

    // ---- P2-3-2: Homing completes → home offset applied → auto-unparks to ONLINE ----
    // After homing, MotionController auto-unparks so PARKED is transient.
    // Verify home offset was applied and axis reaches ONLINE.
    void homingCompletes_offsetApplied_reachesOnline()
    {
        MockA6Drive mock;
        // 100Hz, 0.01s cycle, 0.05mm/cycle. Hardstop at -2mm: reached in ~88c + backoff.
        mock.configure(1, 0.0, 0.05);
        mock.setHardstop(-2.0, true, 50.0);

        MotionController mc;
        mc.configure(makeSingleAxisConfig(100.0));
        mc.startHoming();

        QCOMPARE((int)mc.getAxisState(0), (int)AxisMotionState::HOMING);

        // After homing: PARKED → auto-unpark → UNPARKING → BLENDING → ONLINE
        bool reached = runUntilState(mc, &mock, AxisMotionState::ONLINE, 5000);

        QVERIFY2(reached, "Axis did not reach ONLINE after homing within cycle limit");

        // Home offset must have been applied to the drive
        QVERIFY(mock.isHomeOffsetSet());

        // Axis must be marked homed
        QVERIFY(mc.isAxisHomed(0));
    }

    // ---- P2-3-3: FatalError → PARKED, no auto-restart (P1-1 validation) ----
    void fatalError_doesNotRestart()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);
        mock.setEnableCycles(999999);  // never enables → HomingSequence FatalError at 10s

        MotionController mc;
        mc.configure(makeSingleAxisConfig(100.0));
        mc.startHoming();

        // Enable timeout = 10s / 0.01s = 1000 cycles. Run to 2000 to be safe.
        bool parked = runUntilState(mc, &mock, AxisMotionState::PARKED, 2000);
        QVERIFY2(parked, "Axis did not reach PARKED after FatalError within cycle limit");

        // Must NOT auto-restart homing after FatalError. Run 200 more cycles.
        runCycles(mc, &mock, 200);
        QCOMPARE((int)mc.getAxisState(0), (int)AxisMotionState::PARKED);
    }

    // ---- P2-3-4: PARKED + un-homed holds actual raw position (B75 safety) ----
    // B75: before homing, homeOffset is unknown. Commanding parkPos (1.5mm)
    // against a raw-position drive would cause a sudden jump → ER871/87.
    // Un-homed PARKED must hold the drive's actual raw position instead.
    void parked_unhomedHoldsActualPosition()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);  // drive starts at raw pos 0.0

        MotionController mc;
        mc.configure(makeSingleAxisConfig(100.0));
        // Axis is PARKED + un-homed after configure (no homing triggered)

        MotionOutput out = runCycles(mc, &mock, 5);

        // Must hold actual raw position (0.0), NOT parkPos (1.5mm)
        QVERIFY2(std::abs(out.positions[0]) < 0.01,
                 qPrintable(QString("un-homed PARKED should hold actual pos 0.0, got %1")
                     .arg(out.positions[0])));
    }

    // ---- P2-3-5: ESTOPPING holds position in place (P1-2 validation) ----
    // Axis must NOT move toward parkPos during ESTOPPING.
    // This was the bug: delta = parkPos - currentPos caused a motion command.
    void estopping_holdsPosition()
    {
        MockA6Drive mock;
        // Mock position = 30mm (well away from parkPos=1.5mm)
        mock.configure(1, 30.0, 0.05);

        MotionController mc;
        mc.configure(makeSingleAxisConfig(100.0));
        mc.startHoming();  // axis now HOMING (not PARKED) so estop will fire

        // Run 1 cycle so rt.currentPos is seeded from mock's actual position (30mm)
        runCycles(mc, &mock, 1);

        // P3-4: setEmergencyStop() only writes the atomic; state transition happens
        // inside process() on the next cycle via estop edge detection.
        mc.setEmergencyStop(true);

        // Capture initial position from the first ESTOPPING cycle (edge fires here)
        TelemetryData empty{};
        MotionOutput firstOut{};
        A6Drive* drives[1] = { &mock };
        mc.process(empty, firstOut, drives, 1);
        QCOMPARE((int)mc.getAxisState(0), (int)AxisMotionState::ESTOPPING);
        double heldPos = firstOut.positions[0];

        // Run up to 40 more cycles (within 50c ESTOP_RAMP_SEC window)
        // and verify position does NOT drift toward parkPos (1.5mm)
        for (int i = 0; i < 40; ++i)
        {
            MotionOutput out{};
            mc.process(empty, out, drives, 1);
            if (mc.getAxisState(0) != AxisMotionState::ESTOPPING) break;

            QVERIFY2(std::abs(out.positions[0] - heldPos) < 0.01,
                     qPrintable(QString("Position drifted during ESTOPPING: "
                         "cycle %1, pos=%2mm, expected=%3mm")
                         .arg(i + 1).arg(out.positions[0]).arg(heldPos)));
        }

        // After ESTOP_RAMP_SEC (50 total cycles) → PARKED
        bool parked = runUntilState(mc, &mock, AxisMotionState::PARKED, 60);
        QVERIFY2(parked, "Axis did not reach PARKED after ESTOPPING ramp");
    }

    // ---- P2-3-6: ESTOPPING explicitly does NOT move toward parkPos ----
    // Complementary to test 5: directly check position is not near parkPos.
    void estopping_doesNotMoveTowardPark()
    {
        MockA6Drive mock;
        mock.configure(1, 30.0, 0.0);  // frozen at 30mm

        MotionController mc;
        mc.configure(makeSingleAxisConfig(100.0));
        mc.startHoming();

        // Seed rt.currentPos from mock
        runCycles(mc, &mock, 1);

        // P3-4: state transition happens in process() on the rising edge
        mc.setEmergencyStop(true);
        TelemetryData empty{};
        A6Drive* drives[1] = { &mock };
        MotionOutput edgeOut{};
        mc.process(empty, edgeOut, drives, 1);  // edge detected → ESTOPPING
        QCOMPARE((int)mc.getAxisState(0), (int)AxisMotionState::ESTOPPING);

        // Run 10 cycles within ESTOP window
        for (int i = 0; i < 10; ++i)
        {
            MotionOutput out{};
            mc.process(empty, out, drives, 1);
            if (mc.getAxisState(0) != AxisMotionState::ESTOPPING) break;

            // Must stay near 30mm, must NOT approach parkPos (1.5mm)
            double distFromStart = std::abs(out.positions[0] - 30.0);
            QVERIFY2(distFromStart < 0.01,
                     qPrintable(QString("Position moved during ESTOPPING: "
                         "pos=%1mm (moved %2mm from start)")
                         .arg(out.positions[0]).arg(distFromStart)));

            double distFromPark = std::abs(out.positions[0] - 1.5);
            QVERIFY2(distFromPark > 10.0,
                     qPrintable(QString("Position moved toward parkPos during ESTOPPING: "
                         "pos=%1mm (parkPos=1.5mm)")
                         .arg(out.positions[0])));
        }
    }

    // ---- An unrecognised parkMode must NOT instant-home ----
    // The retired "gravity" mode declared an axis homed at wherever it was
    // resting, with no search. It is gone; a leftover config carrying it (or any
    // other unknown value) must fall through to the REAL torque endstop search,
    // so the axis earns a reference instead of inventing one. Pinning the safe
    // direction: after a cycle the axis is still searching, not homed.
    void unknownHomeMode_doesNotInstantHome_runsRealSearch()
    {
        MockA6Drive mock;
        mock.configure(1, -5.0, 0.0);   // frozen at a raw position, as before

        DriveConfig dc;
        dc.slaveIndex          = 1;
        dc.axisType            = "linear_vertical";
        dc.strokeMm            = 100.0;
        dc.parkMode            = "gravity";   // retired mode = now just "unknown"
        dc.homeDirection       = "negative";
        dc.invertDir           = true;   // foldback fixture: retract = raw negative (see TestHomingSequence)
        dc.homingSpeed      = 400.0;
        dc.homingBackoffMm     = 1.5;
        dc.homingTorquePct     = 25;
        dc.maxVelocityMmS      = 200.0;
        dc.maxAccelerationMmS2 = 2000.0;
        dc.maxJerkMmS3         = 20000.0;
        dc.unparkTimeSec       = 0.1;
        dc.parkTimeSec         = 0.1;
        dc.countsPerMm         = 100.0;
        dc.ballscrewPitch      = 5.0;
        AppConfig cfg;
        cfg.controlLoopHz = 100;
        cfg.numDrives = 1;
        cfg.drives.push_back(dc);

        MotionController mc;
        mc.configure(cfg);
        mc.startHoming();

        TelemetryData empty{};
        MotionOutput out{};
        A6Drive* drives[1] = { &mock };
        mc.process(empty, out, drives, 1);

        // The old behaviour was homed==true after exactly this one cycle.
        QVERIFY2(!mc.isAxisHomed(0),
                 "Unknown parkMode instant-homed -- the retired gravity "
                 "short-circuit is still present");
        QCOMPARE(mc.getAxisState(0), AxisMotionState::HOMING);
    }

    // ---- Unpark must refuse an axis that was never homed ----
    // Normal flow cannot reach this (loop stop re-arms needsRehome, loop start
    // homes, park/unpark need a running loop). A homing FatalError can: that axis
    // stays PARKED+unhomed while its peers finish and also sit PARKED, so the
    // rig reads "all parked" and the toggle offers Unpark. Unpark ramps to
    // centerPos, which for an unhomed axis is a target in a frame with no
    // relation to the machine -- i.e. straight at an endstop. It must hold.
    void unpark_refusesUnhomedAxis()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.0);

        DriveConfig dc;
        dc.slaveIndex          = 1;
        dc.axisType            = "linear_horizontal";
        dc.strokeMm            = 100.0;
        dc.parkMode            = "center";
        dc.homeDirection       = "negative";
        dc.invertDir           = true;   // foldback fixture: retract = raw negative (see TestHomingSequence)
        dc.homingSpeed      = 400.0;
        dc.homingBackoffMm     = 1.5;
        dc.homingTorquePct     = 25;
        dc.maxVelocityMmS      = 200.0;
        dc.maxAccelerationMmS2 = 2000.0;
        dc.maxJerkMmS3         = 20000.0;
        dc.unparkTimeSec       = 0.1;
        dc.parkTimeSec         = 0.1;
        dc.countsPerMm         = 100.0;
        dc.ballscrewPitch      = 5.0;
        AppConfig cfg;
        cfg.controlLoopHz = 100;
        cfg.numDrives = 1;
        cfg.drives.push_back(dc);

        MotionController mc;
        mc.configure(cfg);
        // configure() leaves the axis PARKED and homed==false -- exactly the
        // state a FatalError axis is left in. No homing is run here.
        QVERIFY(!mc.isAxisHomed(0));
        QCOMPARE(mc.getAxisState(0), AxisMotionState::PARKED);

        mc.enqueueCommand({ MotionCommand::Type::StartUnpark, -1 });
        TelemetryData empty{};
        MotionOutput out{};
        A6Drive* drives[1] = { &mock };
        for (int c = 0; c < 20; ++c) mc.process(empty, out, drives, 1);

        QVERIFY2(mc.getAxisState(0) == AxisMotionState::PARKED,
                 "Unhomed axis left PARKED -- unpark must refuse it");
    }

    // ---- TF-5-2: Belt axis skipped by startHoming(), marked homed immediately ----
    // Belt axes have axisType="belt" -- startHoming() skips them and marks homed=true.
    // They should proceed directly to ONLINE without an unpark interpolation.
    void beltAxis_skippedByHoming_markedHomed()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.0);

        DriveConfig dc;
        dc.slaveIndex          = 1;
        dc.axisType            = "belt";
        dc.strokeMm            = 360.0;
        dc.parkMode            = "center";
        dc.homeDirection       = "negative";
        dc.invertDir           = true;   // foldback fixture: retract = raw negative (see TestHomingSequence)
        dc.homingSpeed      = 400.0;  // harness speed: step = speed*REF_DT(0.0005); pre-redesign tests assumed speed*0.01
        dc.homingBackoffMm     = 0.0;
        dc.homingTorquePct     = 25;
        dc.maxVelocityMmS      = 200.0;
        dc.maxAccelerationMmS2 = 2000.0;
        dc.maxJerkMmS3         = 20000.0;
        dc.unparkTimeSec       = 0.1;
        dc.parkTimeSec         = 0.1;
        dc.countsPerMm         = 27.78;
        dc.ballscrewPitch      = 5.0;
        AppConfig cfg;
        cfg.controlLoopHz = 100;
        cfg.numDrives = 1;
        cfg.drives.push_back(dc);

        MotionController mc;
        mc.configure(cfg);

        // startHoming() skips belt axes and marks homed=true immediately.
        mc.startHoming();

        // Belt axis is not put into HOMING state -- it's marked homed=true directly.
        QVERIFY(mc.isAxisHomed(0));

        // After homing, all axes are considered homed → auto-unpark fires.
        // Belt unpark transitions straight to PARKED (no UNPARKING interpolation).
        // Eventually, allAxesReady() should return true.
        // Run up to 300 cycles (30s at 100Hz) for any state transitions to complete.
        bool ready = false;
        TelemetryData empty{};
        A6Drive* drives[1] = { &mock };
        for (int i = 0; i < 300; ++i)
        {
            MotionOutput out{};
            mc.process(empty, out, drives, 1);
            if (mc.allAxesReady()) { ready = true; break; }
        }
        QVERIFY2(ready, "Belt axis did not reach allAxesReady() within cycle limit");
    }

    // ---- TF-3-1: 3-drive allAxesHomed() and allAxesReady() ----
    // All three axes must individually home before allAxesHomed() returns true.
    // allAxesReady() must also wait until all three complete unpark.
    void threeAxis_allAxesHomedAndReady()
    {
        // Build a 3-axis config (all linear_vertical, endstop homing)
        AppConfig cfg;
        cfg.controlLoopHz = 100;
        cfg.numDrives     = 3;
        for (int ax = 0; ax < 3; ++ax)
        {
            DriveConfig dc;
            dc.slaveIndex           = ax + 1;
            dc.axisType             = "linear_vertical";
            dc.strokeMm             = 100.0;
            dc.homingSpeed       = 1000.0; // fast: 0.5mm/cycle (1000*REF_DT) → reaches stop quickly
            dc.homingBackoffMm      = 1.5;
            dc.homingTorquePct      = 25;
            dc.homeDirection        = "negative";
            dc.invertDir           = true;   // foldback fixture: retract = raw negative (see TestHomingSequence)
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

        // Three mocks with hardstops at -50mm
        MockA6Drive m0, m1, m2;
        m0.configure(1, 0.0, 0.5);  m0.setHardstop(-50.0, true, 50.0);
        m1.configure(2, 0.0, 0.5);  m1.setHardstop(-50.0, true, 50.0);
        m2.configure(3, 0.0, 0.5);  m2.setHardstop(-50.0, true, 50.0);

        mc.startHoming();

        A6Drive* drives[3] = { &m0, &m1, &m2 };
        TelemetryData empty{};
        MotionOutput out{};

        bool homedAll = false;
        bool readyAll = false;
        for (int cycle = 0; cycle < 5000; ++cycle)
        {
            m0.updateStatus(); m1.updateStatus(); m2.updateStatus();
            mc.process(empty, out, drives, 3);

            if (!homedAll && mc.allAxesHomed())
                homedAll = true;

            if (homedAll && mc.allAxesReady())
            {
                readyAll = true;
                break;
            }
        }

        QVERIFY2(homedAll, "allAxesHomed() never returned true for 3-axis config");
        QVERIFY2(readyAll, "allAxesReady() never returned true after all axes homed");

        for (int ax = 0; ax < 3; ++ax)
            QVERIFY2(mc.isAxisHomed(ax),
                qPrintable(QString("Axis %1 not individually homed").arg(ax)));
    }

    // ---- TF-3-2: Fault on one axis while others ONLINE ----
    // One drive faults mid-ONLINE. MotionController must set needsRehome=true
    // and park all axes. The faulted drive recovering alone is not enough --
    // all axes are PARKED waiting for explicit startHoming().
    void threeAxis_faultOnOneAxis_allPark()
    {
        AppConfig cfg;
        cfg.controlLoopHz = 100;
        cfg.numDrives     = 3;
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
            dc.invertDir           = true;   // foldback fixture: retract = raw negative (see TestHomingSequence)
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

        // Get all axes to ONLINE
        mc.startHoming();
        TelemetryData empty{};
        MotionOutput out{};
        bool allOnline = false;
        for (int cycle = 0; cycle < 5000 && !allOnline; ++cycle)
        {
            m0.updateStatus(); m1.updateStatus(); m2.updateStatus();
            mc.process(empty, out, drives, 3);
            allOnline = (mc.getAxisState(0) == AxisMotionState::ONLINE &&
                         mc.getAxisState(1) == AxisMotionState::ONLINE &&
                         mc.getAxisState(2) == AxisMotionState::ONLINE);
        }
        QVERIFY2(allOnline, "Did not reach all-ONLINE within 5000 cycles");

        // Fault axis 1 mid-ONLINE
        mc.setNeedsRehome(true);
        mc.startPark();

        // Run park cycle
        for (int cycle = 0; cycle < 200; ++cycle)
        {
            m0.updateStatus(); m1.updateStatus(); m2.updateStatus();
            mc.process(empty, out, drives, 3);
        }

        // All axes must be PARKED (not homed) -- needsRehome=true
        QVERIFY(mc.needsRehome());
        for (int ax = 0; ax < 3; ++ax)
        {
            AxisMotionState st = mc.getAxisState(ax);
            QVERIFY2(st == AxisMotionState::PARKED,
                qPrintable(QString("Axis %1 state after park = %2, expected PARKED")
                    .arg(ax).arg((int)st)));
        }
    }

    // ---- B222P regression: post-homing PARKED holds at home position, not parkPos ----
    //
    // For parkMode="center", parkPos = strokeMm/2 (e.g. 40mm for an 80mm
    // stroke). Prior to B222P, the PARKED state output unconditionally
    // commanded ac.parkPos every cycle once rt.homed was true. The cycle
    // right after homing-complete saw the target jump from ~0mm (the
    // backoff position) to 40mm in 2ms -- a 40mm-in-one-cycle step that
    // tripped the drive's velocity loop and produced Er87.1 on every cold
    // homing of a center-mode axis.
    //
    // Reproducible only in multi-axis configs: with one axis the
    // auto-unpark fires immediately and PARKED is a one-cycle transient.
    // With multiple axes, the first-to-home sits in PARKED for hundreds
    // of cycles while the others search -- and that's where the bug bit.
    void postHoming_centerMode_doesNotJumpToParkPos()
    {
        // Two-axis config, parkMode="center", strokeMm=80 -> parkPos=40.
        AppConfig cfg;
        cfg.controlLoopHz = 100;
        cfg.numDrives     = 2;
        for (int ax = 0; ax < 2; ++ax)
        {
            DriveConfig dc;
            dc.slaveIndex           = ax + 1;
            dc.axisType             = "linear_vertical";
            dc.strokeMm             = 80.0;
            dc.homingSpeed       = 400.0;  // harness speed: step = speed*REF_DT(0.0005); pre-redesign tests assumed speed*0.01
            dc.homingBackoffMm      = 1.5;
            dc.homingTorquePct      = 25;
            dc.homeDirection        = "negative";
            dc.invertDir           = true;   // foldback fixture: retract = raw negative (see TestHomingSequence)
            dc.parkMode             = "center";    // parkPos = 40.0
            dc.maxVelocityMmS       = 200.0;
            dc.maxAccelerationMmS2  = 2000.0;
            dc.maxJerkMmS3          = 20000.0;
            dc.unparkTimeSec        = 3.0;
            dc.parkTimeSec          = 3.0;
            dc.countsPerMm          = 100.0;
            dc.ballscrewPitch       = 5.0;
            cfg.drives.push_back(dc);
        }

        MotionController mc;
        mc.configure(cfg);

        // Axis 0: hardstop at -1mm (homes fast, ~20 cycles to stop).
        // Axis 1: hardstop at -30mm (homes slow, ~600 cycles to stop).
        // Window between the two completions is where the bug manifests.
        MockA6Drive m0, m1;
        m0.configure(1, 0.0, 0.05);  m0.setHardstop(-1.0,  true, 50.0);
        m1.configure(2, 0.0, 0.05);  m1.setHardstop(-30.0, true, 50.0);

        mc.startHoming();

        A6Drive* drives[2] = { &m0, &m1 };
        TelemetryData empty{};
        MotionOutput out{};

        // Run until axis 0 enters PARKED (homing complete on axis 0).
        bool axis0Parked = false;
        int  cyclesToPark0 = 0;
        for (int cycle = 0; cycle < 5000 && !axis0Parked; ++cycle)
        {
            m0.updateStatus(); m1.updateStatus();
            mc.process(empty, out, drives, 2);
            if (mc.getAxisState(0) == AxisMotionState::PARKED && mc.isAxisHomed(0))
            {
                axis0Parked = true;
                cyclesToPark0 = cycle;
            }
        }
        QVERIFY2(axis0Parked, "Axis 0 never reached PARKED post-homing");

        // Axis 1 must still be HOMING (otherwise the test isn't exercising
        // the PARKED-while-other-axis-searching window).
        QCOMPARE((int)mc.getAxisState(1), (int)AxisMotionState::HOMING);

        // Capture axis 0's commanded position over the next 30 cycles. Each
        // cycle motion-out.positions[0] should stay close to the homing
        // completion position (=0 in engineering, the backoff point). It must
        // NOT jump to 40 (parkPos = strokeMm/2 for parkMode=center).
        double maxCommandedAbs = 0.0;
        for (int k = 0; k < 30; ++k)
        {
            m0.updateStatus(); m1.updateStatus();
            mc.process(empty, out, drives, 2);
            // Once auto-unpark fires the trajectory starts ramping from 0 to
            // centerPos, so once axis 0 leaves PARKED we stop measuring.
            if (mc.getAxisState(0) != AxisMotionState::PARKED) break;
            double v = std::abs(out.positions[0]);
            if (v > maxCommandedAbs) maxCommandedAbs = v;
        }

        // Hard threshold: motion-out.positions[0] must be < 5mm during the
        // PARKED-after-homing window. The old bug had it at parkPos=40
        // immediately. Anything significantly > 0 in this window means we've
        // regressed.
        QVERIFY2(maxCommandedAbs < 5.0,
            qPrintable(QString("Post-homing PARKED axis commanded %1mm "
                "(should be near 0, parkPos=40 indicates B222P regression). "
                "axis0 reached PARKED at cycle %2.")
                .arg(maxCommandedAbs, 0, 'f', 3).arg(cyclesToPark0)));
    }

    // ---- B64-1: enqueueCommand StartHoming is drained and executed by process() ----
    // Enqueue StartHoming from the "UI thread" before calling process().
    // After one process() cycle the axis must be in HOMING state.
    void cmdQueue_startHoming_executedByProcess()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);

        MotionController mc;
        mc.configure(makeSingleAxisConfig());

        // Enqueue before process() - simulates UI button press
        bool ok = mc.enqueueCommand({MotionCommand::Type::StartHoming, -1});
        QVERIFY(ok);

        // One process() call drains the queue and calls startHoming()
        TelemetryData empty{};
        MotionOutput out{};
        A6Drive* drives[1] = { &mock };
        mc.process(empty, out, drives, 1);

        QCOMPARE((int)mc.getAxisState(0), (int)AxisMotionState::HOMING);
    }

    // ---- B64-2: enqueueCommand StartPark is drained correctly ----
    void cmdQueue_startPark_executedByProcess()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.5);
        mock.setHardstop(-50.0, true, 50.0);

        MotionController mc;
        mc.configure(makeSingleAxisConfig());

        // Get to ONLINE first
        mc.startHoming();
        bool reached = runUntilState(mc, &mock, AxisMotionState::ONLINE, 5000);
        QVERIFY2(reached, "Did not reach ONLINE");
        QCOMPARE((int)mc.getAxisState(0), (int)AxisMotionState::ONLINE);

        // Enqueue park
        bool ok = mc.enqueueCommand({MotionCommand::Type::StartPark, 0});
        QVERIFY(ok);

        // One process() drains and calls startPark()
        TelemetryData empty{};
        MotionOutput out{};
        A6Drive* drives[1] = { &mock };
        mock.updateStatus();
        mc.process(empty, out, drives, 1);

        // Axis should have transitioned to PARKING
        AxisMotionState st = mc.getAxisState(0);
        QVERIFY2(st == AxisMotionState::PARKING || st == AxisMotionState::PARKED,
            qPrintable(QString("Expected PARKING or PARKED, got %1").arg((int)st)));
    }

    // ---- B64-3: Full queue returns false without crashing ----
    // SpscQueue<T,32> holds N-1=31 items (one slot reserved as full sentinel).
    // Verify the 32nd push returns false.
    void cmdQueue_full_returnsFalse()
    {
        MotionController mc;
        mc.configure(makeSingleAxisConfig());

        // Push 31 commands (max capacity for a 32-slot SPSC ring)
        int pushed = 0;
        for (int i = 0; i < 31; ++i)
        {
            if (mc.enqueueCommand({MotionCommand::Type::StartHoming, -1}))
                ++pushed;
        }
        QCOMPARE(pushed, 31);

        // 32nd must fail (queue full)
        bool overflow = mc.enqueueCommand({MotionCommand::Type::StartHoming, -1});
        QVERIFY(!overflow);
    }

    // ---- B64-4: getMotionStatus() snapshot reflects current state ----
    // After startHoming via queue and one process() cycle, the snapshot
    // must report HOMING and needsRehome=false (homing just started).
    void motionStatus_snapshot_reflectsState()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.05);

        MotionController mc;
        mc.configure(makeSingleAxisConfig());

        // Before any process() the snapshot has numDrives=0 (never published)
        // After one idle process() it should reflect PARKED.
        TelemetryData empty{};
        MotionOutput out{};
        A6Drive* drives[1] = { &mock };
        mc.process(empty, out, drives, 1);

        MotionStatus ms = mc.getMotionStatus();
        QCOMPARE(ms.numDrives, 1);
        QCOMPARE((int)ms.axisState[0], (int)AxisMotionState::PARKED);
        QVERIFY(ms.needsRehome);

        // Enqueue homing, run one cycle, check snapshot updated
        mc.enqueueCommand({MotionCommand::Type::StartHoming, -1});
        mock.updateStatus();
        mc.process(empty, out, drives, 1);

        ms = mc.getMotionStatus();
        QCOMPARE((int)ms.axisState[0], (int)AxisMotionState::HOMING);
        QVERIFY(!ms.homed[0]);
    }

    // ---- B64-5: axisStateName includes BLENDING (regression for missing case) ----
    // Verify getAxisStateName() returns "BLENDING" and not "UNKNOWN" when in that state.
    void axisStateName_blending_notUnknown()
    {
        MockA6Drive mock;
        mock.configure(1, 0.0, 0.5);
        mock.setHardstop(-50.0, true, 50.0);

        MotionController mc;
        mc.configure(makeSingleAxisConfig());

        mc.startHoming();
        TelemetryData empty{};
        MotionOutput out{};
        A6Drive* drives[1] = { &mock };

        bool sawBlending = false;
        for (int i = 0; i < 3000 && !sawBlending; ++i)
        {
            mock.updateStatus();
            mc.process(empty, out, drives, 1);
            if (mc.getAxisState(0) == AxisMotionState::BLENDING)
            {
                MotionStatus ms = mc.getMotionStatus();
                QVERIFY2(ms.axisStateName[0] == "BLENDING",
                    qPrintable(QString("Expected 'BLENDING', got '%1'").arg(QString::fromStdString(ms.axisStateName[0]))));
                sawBlending = true;
            }
        }
        QVERIFY2(sawBlending, "BLENDING state was never observed");
    }
};

QTEST_MAIN(TestMotionController)
#include "TestMotionController.moc"
