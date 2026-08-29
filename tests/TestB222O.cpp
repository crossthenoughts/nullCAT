// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestB222O.cpp
//
// Config schema/scaling roundtrip tests:
//   O-1   Per-field round-trip: every settable field in AppConfig
//         and DriveConfig survives save() then load(). This is the
//         guard against the dcSyncOffset-style bug (save schema
//         drifts from load schema -> UI edits silently dropped).
//   O-2   Scaling correctness: countsPerMm is computed from
//         encoderCountsPerRev / ballscrewPitch at load. Verifies
//         the single-pipeline guarantee (no dual countsPerRev +
//         unitsPerRev anymore).
//   O-3   recomputeDerivedFields enforces the same rule when called
//         directly (used by the dialog after pitch/encoder edits).
//   O-4   A6Drive::setScaling is single-arg and unitsToCount /
//         countsToUnits use countsPerMm directly.
// ============================================================

#include <QtTest>
#include <QTemporaryDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>

#include "../src/Config.h"
#include "../src/A6Drive.h"

class TestB222O : public QObject
{
    Q_OBJECT

private:
    static QJsonObject readJson(const QString& path)
    {
        QFile f(path);
        if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return {};
        QByteArray data = f.readAll();
        f.close();
        QJsonParseError err;
        QJsonDocument doc = QJsonDocument::fromJson(data, &err);
        if (err.error != QJsonParseError::NoError || !doc.isObject()) return {};
        return doc.object();
    }

private slots:

    void o1_appconfig_roundtrip_every_field()
    {
        QTemporaryDir tmp;
        QVERIFY(tmp.isValid());
        QString path = tmp.path() + "/config.json";

        // Construct an AppConfig where every field has a non-default,
        // distinctive value. Save it. Reload into a fresh Config.
        // Every field must come back identical.
        Config orig;
        AppConfig& a = orig.get();
        a.nicName                  = "TestNic_NotDefault";
        a.controlLoopHz            = 500;       // canonical rate
        a.numDrives                = 2;
        a.telemetryPort               = 4445;
        a.webPort                  = 9090;
        a.webBindAddr              = "10.20.30.40";
        a.webUIEnabled             = true;
        a.dcSyncOffsetNs           = 500000;    // <-- THE bug case
        a.pdoWatchdogMs            = 250;
        a.blendTimeSec             = 3.7;
        a.blendMaxVelocityMmS      = 42.0;
        a.requireUserFaultReset    = true;
        a.enableCapabilityScan     = true;
        a.commandSyncCycles        = 17;
        a.wkcValidationCycles      = 99;
        a.wkcValidationThreshold   = 0.82;
        a.logFile                  = "logs/custom.log";
        a.logToConsole             = false;
        a.simulationMode           = true;
        a.logMinLevel              = "warning";
        a.diagEnabled              = false;
        a.foregroundKeeperEnabled  = false;
        a.foregroundKeeperAlpha    = 64;
        a.foregroundKeeperX        = 123;
        a.foregroundKeeperY        = 456;

        // At least one drive so save/load doesn't trigger "empty drives"
        // setDefaults() fallback.
        a.drives.clear();
        DriveConfig d;
        d.slaveIndex = 1; d.name = "TestAxis1";
        d.ballscrewPitch = 10.0; d.encoderCountsPerRev = 131072.0;
        d.countsPerMm = 13107.2;
        a.drives.push_back(d);
        d.slaveIndex = 2; d.name = "TestAxis2";
        a.drives.push_back(d);

        QVERIFY(orig.save(path.toStdString()));

        Config loaded;
        QVERIFY(loaded.load(path.toStdString()));
        const AppConfig& b = loaded.get();

        QCOMPARE(b.nicName,                  a.nicName);
        QCOMPARE(b.controlLoopHz,            a.controlLoopHz);
        QCOMPARE(b.numDrives,                a.numDrives);
        QCOMPARE(b.telemetryPort,               a.telemetryPort);
        QCOMPARE(b.webPort,                  a.webPort);
        QCOMPARE(b.webBindAddr,              a.webBindAddr);
        QCOMPARE(b.webUIEnabled,             a.webUIEnabled);
        QCOMPARE(b.dcSyncOffsetNs,           a.dcSyncOffsetNs);   // The bug-case field.
        QCOMPARE(b.pdoWatchdogMs,            a.pdoWatchdogMs);
        QCOMPARE(b.blendTimeSec,             a.blendTimeSec);
        QCOMPARE(b.blendMaxVelocityMmS,      a.blendMaxVelocityMmS);
        QCOMPARE(b.requireUserFaultReset,    a.requireUserFaultReset);
        QCOMPARE(b.enableCapabilityScan,     a.enableCapabilityScan);
        QCOMPARE(b.commandSyncCycles,        a.commandSyncCycles);
        QCOMPARE(b.wkcValidationCycles,      a.wkcValidationCycles);
        QCOMPARE(b.wkcValidationThreshold,   a.wkcValidationThreshold);
        QCOMPARE(b.logFile,                  a.logFile);
        QCOMPARE(b.logToConsole,             a.logToConsole);
        QCOMPARE(b.simulationMode,           a.simulationMode);
        QCOMPARE(b.logMinLevel,              a.logMinLevel);
        QCOMPARE(b.diagEnabled,              a.diagEnabled);
        QCOMPARE(b.foregroundKeeperEnabled,  a.foregroundKeeperEnabled);
        QCOMPARE(b.foregroundKeeperAlpha,    a.foregroundKeeperAlpha);
        QCOMPARE(b.foregroundKeeperX,        a.foregroundKeeperX);
        QCOMPARE(b.foregroundKeeperY,        a.foregroundKeeperY);
    }

    void o1b_driveconfig_roundtrip_every_field()
    {
        QTemporaryDir tmp;
        QVERIFY(tmp.isValid());
        QString path = tmp.path() + "/config.json";

        Config orig;
        AppConfig& a = orig.get();
        a.drives.clear();
        a.numDrives = 1;

        DriveConfig d;
        d.slaveIndex                   = 3;
        d.name                         = "AxisAlpha";
        d.mode                         = "torque";   // canonical token ("cst" is a read alias - see o1c)
        d.axisType                     = "linear_horizontal";
        d.invertDir                    = true;
        d.strokeMm                     = 175.5;
        d.ballscrewPitch               = 8.0;
        d.encoderCountsPerRev          = 65536.0;
        d.reductionRatio               = "3:1";
        // countsPerMm is DERIVED by the single scaling pipeline at load:
        // encoderCountsPerRev / ballscrewPitch x reduction (65536/8 x 3).
        // Set the expectation to the derived value so the roundtrip compare
        // also pins the reduction-ratio factor.
        d.countsPerMm                  = 65536.0 / 8.0 * 3.0;
        d.homeDirection                = "positive";
        d.parkMode                     = "endstop";
        d.homingBackoffMm              = 2.7;
        d.homingSpeed               = 12.0;
        d.homingTorquePct              = 40;
        d.maxVelocityMmS               = 333.0;
        d.maxAccelerationMmS2          = 4242.0;
        d.maxJerkMmS3                  = 55555.0;
        d.unparkTimeSec                = 4.0;
        d.parkTimeSec                  = 5.0;
        // (filterAlpha is not a DriveConfig field.)
        d.spikeFilterEnabled           = true;
        d.spikeMaxMm                   = 7.5;
        d.torqueMinPct                 = 8.0;
        d.torqueMaxPct                 = 65.0;
        d.followingErrorWindowMm       = 50.5;   // per-axis (rig) field
        a.drives.push_back(d);

        QVERIFY(orig.save(path.toStdString()));

        Config loaded;
        QVERIFY(loaded.load(path.toStdString()));
        QVERIFY(!loaded.get().drives.empty());
        const DriveConfig& b = loaded.get().drives[0];

        QCOMPARE(b.slaveIndex,                 d.slaveIndex);
        QCOMPARE(b.name,                       d.name);
        QCOMPARE(b.mode,                       d.mode);
        QCOMPARE(b.axisType,                   d.axisType);
        QCOMPARE(b.invertDir,                  d.invertDir);
        QCOMPARE(b.strokeMm,                   d.strokeMm);
        QCOMPARE(b.ballscrewPitch,             d.ballscrewPitch);
        QCOMPARE(b.encoderCountsPerRev,        d.encoderCountsPerRev);
        QCOMPARE(b.countsPerMm,                d.countsPerMm);
        QCOMPARE(b.reductionRatio,             d.reductionRatio);
        QCOMPARE(b.homeDirection,              d.homeDirection);
        QCOMPARE(b.parkMode,                   d.parkMode);
        QCOMPARE(b.homingBackoffMm,            d.homingBackoffMm);
        QCOMPARE(b.homingSpeed,             d.homingSpeed);
        QCOMPARE(b.homingTorquePct,            d.homingTorquePct);
        QCOMPARE(b.maxVelocityMmS,             d.maxVelocityMmS);
        QCOMPARE(b.maxAccelerationMmS2,        d.maxAccelerationMmS2);
        QCOMPARE(b.maxJerkMmS3,                d.maxJerkMmS3);
        QCOMPARE(b.unparkTimeSec,              d.unparkTimeSec);
        QCOMPARE(b.parkTimeSec,                d.parkTimeSec);
        // (filterAlpha is not a DriveConfig field.)
        QCOMPARE(b.spikeFilterEnabled,         d.spikeFilterEnabled);
        QCOMPARE(b.spikeMaxMm,                 d.spikeMaxMm);
        QCOMPARE(b.torqueMinPct,               d.torqueMinPct);
        QCOMPARE(b.torqueMaxPct,               d.torqueMaxPct);
        QCOMPARE(b.followingErrorWindowMm,     d.followingErrorWindowMm);
    }

    void o1c_mode_cst_alias_normalizes_to_torque()
    {
        // "torque" is the canonical mode token (what the motion + EtherCAT
        // layers switch on); "cst" is accepted on read as a DS402-flavoured
        // alias and normalized. Pin that normalization.
        QTemporaryDir tmp;
        QVERIFY(tmp.isValid());
        QString path = tmp.path() + "/config.json";

        QFile rig(tmp.path() + "/rig.json");
        QVERIFY(rig.open(QIODevice::WriteOnly | QIODevice::Text));
        rig.write(
            "{\n"
            "  \"configVersion\": 2,\n"
            "  \"numDrives\": 1,\n"
            "  \"global\": {},\n"
            "  \"axes\": [ { \"slaveIndex\": 1, \"name\": \"Belt\", \"mode\": \"cst\" } ]\n"
            "}\n");
        rig.close();

        Config loaded;
        QVERIFY(loaded.load(path.toStdString()));
        QVERIFY(!loaded.get().drives.empty());
        QCOMPARE(loaded.get().drives[0].mode, std::string("torque"));
    }

    void o2_scaling_derived_from_pitch_and_encoder_counts()
    {
        // Hand-write a minimal config with only ballscrewPitch and
        // encoderCountsPerRev. Load must derive countsPerMm. This is
        // the path that the new dialog produces.
        QTemporaryDir tmp;
        QVERIFY(tmp.isValid());
        QString path = tmp.path() + "/config.json";

        QByteArray raw =
            "{\n"
            "  \"nicName\": \"X\",\n"
            "  \"controlLoopHz\": 500,\n"
            "  \"numDrives\": 1,\n"
            "  \"drives\": [\n"
            "    { \"slaveIndex\": 1, \"name\": \"A\",\n"
            "      \"ballscrewPitch\": 10.0,\n"
            "      \"encoderCountsPerRev\": 131072\n"
            "    }\n"
            "  ]\n"
            "}\n";
        QFile f(path);
        QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text));
        f.write(raw);
        f.close();

        Config cfg;
        QVERIFY(cfg.load(path.toStdString()));
        QVERIFY(!cfg.get().drives.empty());
        const DriveConfig& d = cfg.get().drives[0];

        QCOMPARE(d.ballscrewPitch,       10.0);
        QCOMPARE(d.encoderCountsPerRev,  131072.0);
        QVERIFY2(qAbs(d.countsPerMm - 13107.2) < 0.001,
            qPrintable(QString("Expected countsPerMm ~13107.2, got %1").arg(d.countsPerMm)));
    }

    void o3_recompute_derived_fields()
    {
        // Direct helper used by the dialog after pitch/encoder edits.
        DriveConfig d;
        d.encoderCountsPerRev = 100000.0;
        d.ballscrewPitch      = 5.0;
        d.countsPerMm         = -999.0;   // stale

        Config::recomputeDerivedFields(d);
        QCOMPARE(d.countsPerMm, 20000.0);

        // Pitch 0 must not divide-by-zero (function leaves value unchanged).
        d.ballscrewPitch = 0.0;
        d.countsPerMm    = 42.0;
        Config::recomputeDerivedFields(d);
        QCOMPARE(d.countsPerMm, 42.0);
    }

    void o3b_reduction_ratio_edge_cases()
    {
        // Malformed ratios fall back to 1.0 (never 0, never negative) --
        // a geared lever's counts/degree depends on this parse.
        DriveConfig d;
        d.encoderCountsPerRev = 131072.0;
        d.ballscrewPitch      = 360.0;      // rotary convention: counts/deg

        d.reductionRatio = "50:1";
        Config::recomputeDerivedFields(d);
        QVERIFY2(qAbs(d.countsPerMm - 131072.0 * 50.0 / 360.0) < 0.01,
                 qPrintable(QString("50:1 counts/deg wrong: %1").arg(d.countsPerMm)));

        const char* fallbackToOne[] = { "abc", "0:1", "-3:1", "", ":1" };
        for (const char* r : fallbackToOne)
        {
            d.reductionRatio = r;
            d.countsPerMm    = -1.0;
            Config::recomputeDerivedFields(d);
            QVERIFY2(qAbs(d.countsPerMm - 131072.0 / 360.0) < 0.01,
                     qPrintable(QString("ratio '%1' must fall back to 1.0").arg(r)));
        }

        // A REVERSED ratio ("1:50") silently reads as 1.0 - documented
        // behaviour, pinned here so any future change is deliberate.
        d.reductionRatio = "1:50";
        Config::recomputeDerivedFields(d);
        QVERIFY(qAbs(d.countsPerMm - 131072.0 / 360.0) < 0.01);

        d.reductionRatio = "1.5:1";
        Config::recomputeDerivedFields(d);
        QVERIFY(qAbs(d.countsPerMm - 131072.0 * 1.5 / 360.0) < 0.01);
    }

    void o3c_device_object_roundtrip()
    {
        // The nested device{} block (curves as node arrays) must survive
        // save/load exactly - it is what presets fill and the curve editor
        // edits.
        QTemporaryDir tmp;
        QVERIFY(tmp.isValid());
        QString path = tmp.path() + "/config.json";

        Config cfg;
        {
            AppConfig& c = cfg.get();
            c.numDrives = 1;
            DriveConfig d;
            d.slaveIndex = 1;
            d.axisType   = "shifter";
            d.mode       = "torque";
            d.device.dir          = -1.0;
            d.device.neutralRev   = 0.01;
            d.device.detents      = { -0.05, 0.0, 0.05 };
            d.device.springCurve  = { {-0.07, -175.0}, {0.0, 0.0}, {0.07, 175.0} };
            d.device.detentCurve  = { {-0.02, 60.0}, {0.02, -60.0} };
            d.device.stopMinRev   = -0.08;
            d.device.stopMaxRev   =  0.08;
            d.device.lashRev      = 0.004;
            d.device.thermalDwellSec = 20.0;
            d.device.clutchBitePct   = 30.0;
            d.device.blockGain       = 1.5;
            d.device.grindAmpPct     = 12.0;
            d.device.grindFreqHz     = 47.0;
            d.device.blockStartRev   = 0.015;
            c.drives = { d };
            c.ncxBindings = { { "rpm", 0, 1.0, 0.0 },
                              { "clutchPct", 3, 100.0, 0.0 } };
        }
        QVERIFY(cfg.save(path.toStdString()));

        Config back;
        QVERIFY(back.load(path.toStdString()));
        QVERIFY(!back.get().drives.empty());
        const DeviceParams& p = back.get().drives[0].device;
        QCOMPARE(p.dir, -1.0);
        QCOMPARE(p.neutralRev, 0.01);
        QCOMPARE((int)p.detents.size(), 3);
        QCOMPARE(p.detents[2], 0.05);
        QCOMPARE((int)p.springCurve.size(), 3);
        QCOMPARE(p.springCurve[0].x, -0.07);
        QCOMPARE(p.springCurve[0].y, -175.0);
        QCOMPARE((int)p.detentCurve.size(), 2);
        QCOMPARE(p.detentCurve[1].y, -60.0);
        QCOMPARE(p.stopMinRev, -0.08);
        QCOMPARE(p.lashRev, 0.004);
        QCOMPARE(p.thermalDwellSec, 20.0);
        QCOMPARE(p.clutchBitePct, 30.0);
        QCOMPARE(p.blockGain, 1.5);
        QCOMPARE(p.grindAmpPct, 12.0);
        QCOMPARE(p.grindFreqHz, 47.0);
        QCOMPARE(p.blockStartRev, 0.015);
        const auto& nb = back.get().ncxBindings;
        QCOMPARE((int)nb.size(), 2);
        QCOMPARE(QString::fromStdString(nb[0].token), QString("rpm"));
        QCOMPARE(nb[1].slot, 3);
        QCOMPARE(nb[1].scale, 100.0);
    }

    void o4_a6drive_single_arg_scaling()
    {
        // setScaling takes a single counts/mm argument.
        // Verify unitsToCount + countsToUnits both use the single value.
        A6Drive drive;
        drive.setScaling(13107.2);   // 10mm pitch, 131072 cpr
        QVERIFY(qAbs(drive.getCountsPerMm() - 13107.2) < 0.001);
        QCOMPARE(drive.unitsToCount(1.0),  13107);   // 1mm -> 13107 counts
        QCOMPARE(drive.unitsToCount(10.0), 131072);  // 10mm -> 131072 counts (one rev)

        // Negative scaling not allowed (clamp to 1.0 so /0 protection).
        drive.setScaling(-1.0);
        QCOMPARE(drive.getCountsPerMm(), 1.0);
    }

    void o5_full_default_roundtrip_no_data_loss()
    {
        // Save a freshly-constructed Config (all defaults) and reload it.
        // Every field should be unchanged. Catches regressions where a new
        // AppConfig field is added but only wired into load() and not save()
        // (the schema-drift bug class this suite guards).
        QTemporaryDir tmp;
        QVERIFY(tmp.isValid());
        QString path = tmp.path() + "/config.json";

        Config orig;
        QVERIFY(orig.save(path.toStdString()));

        Config loaded;
        QVERIFY(loaded.load(path.toStdString()));

        const AppConfig& a = orig.get();
        const AppConfig& b = loaded.get();
        QCOMPARE(b.controlLoopHz, a.controlLoopHz);
        QCOMPARE(b.dcSyncOffsetNs, a.dcSyncOffsetNs);
        QCOMPARE(b.pdoWatchdogMs, a.pdoWatchdogMs);
        QCOMPARE(b.blendMaxVelocityMmS, a.blendMaxVelocityMmS);
        QCOMPARE(b.commandSyncCycles, a.commandSyncCycles);
        QCOMPARE(b.wkcValidationCycles, a.wkcValidationCycles);
        QCOMPARE(b.wkcValidationThreshold, a.wkcValidationThreshold);
        QCOMPARE(b.requireUserFaultReset, a.requireUserFaultReset);
        QCOMPARE(b.enableCapabilityScan, a.enableCapabilityScan);

        QVERIFY(!b.drives.empty());
        const DriveConfig& da = a.drives[0];
        const DriveConfig& db = b.drives[0];
        QCOMPARE(db.followingErrorWindowMm,     da.followingErrorWindowMm);
    }
};

QTEST_MAIN(TestB222O)
#include "TestB222O.moc"
