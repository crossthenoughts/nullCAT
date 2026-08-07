// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestConfigTwoFile.cpp  (Slice 2 — host/rig config split)
//
// Verifies the two-file (host.json + rig.json) load/save:
//   - cold start (no config of any kind) writes a VALID two-file set
//   - reload round-trips
//   - single-writer isolation: saveRig() never touches host.json and
//     saveHost() never touches rig.json  (the structural race guarantee)
//   - merge-on-save preservation: unknown/user keys in either file
//     (top-level host, rig per-axis, "_comment" annotations) survive a
//     save; axes merge by slaveIndex on add/remove
// ============================================================

#include <QtTest>
#include <QTemporaryDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include "../src/Config.h"

class TestConfigTwoFile : public QObject
{
    Q_OBJECT

    static QString anchor(const QTemporaryDir& d) { return d.path() + "/config.json"; }

    static QJsonObject readObj(const QString& path)
    {
        QFile f(path);
        if (!f.open(QIODevice::ReadOnly)) return {};
        return QJsonDocument::fromJson(f.readAll()).object();
    }
    static void writeText(const QString& path, const QByteArray& bytes)
    {
        QFile f(path); QVERIFY(f.open(QIODevice::WriteOnly)); f.write(bytes); f.close();
    }

private slots:
    // Cold start: empty dir -> load() must leave a valid host.json + rig.json.
    void coldStart_writesValidTwoFiles()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config cfg;
        QVERIFY(cfg.load(anchor(dir).toStdString()));

        QVERIFY2(QFile::exists(dir.path() + "/host.json"), "host.json not created on cold start");
        QVERIFY2(QFile::exists(dir.path() + "/rig.json"),  "rig.json not created on cold start");

        QJsonObject host = readObj(dir.path() + "/host.json");
        QCOMPARE(host.value("configVersion").toInt(), 2);
        QVERIFY(host.contains("nicName"));
        QVERIFY(host.contains("webBindAddr"));
        QVERIFY(host.contains("telemetryBindAddr"));   // new host field

        QJsonObject rig = readObj(dir.path() + "/rig.json");
        QCOMPARE(rig.value("configVersion").toInt(), 2);
        QVERIFY(rig.value("global").isObject());
        QVERIFY(rig.value("axes").isArray());
        QVERIFY2(rig.value("axes").toArray().size() >= 1, "cold-start rig has no default axis");

        // The produced config validates.
        QVERIFY(cfg.get().validate().empty());
    }

    // host.json alone (the Pi installer seeds one; rig.json does not exist
    // yet) must keep its values. Regression: load() used to call the full
    // setDefaults() when drives were empty, resetting the entire AppConfig
    // and wiping the host fields it had just read — every fresh Pi install
    // lost its seeded nicName/webBindAddr on first boot.
    void hostOnly_preservesHostFields()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/host.json",
                  "{\n"
                  "    \"nicName\": \"eth0\",\n"
                  "    \"webBindAddr\": \"0.0.0.0\"\n"
                  "}\n");

        Config cfg;
        QVERIFY(cfg.load(anchor(dir).toStdString()));
        QCOMPARE(QString::fromStdString(cfg.get().nicName), QString("eth0"));
        QCOMPARE(QString::fromStdString(cfg.get().webBindAddr), QString("0.0.0.0"));
        QVERIFY2(!cfg.get().drives.empty(), "default drive not seeded");

        // The normalize-save (what main() does right after load) must write
        // the values back, not the compiled defaults.
        QVERIFY(cfg.save(anchor(dir).toStdString()));
        QJsonObject host = readObj(dir.path() + "/host.json");
        QCOMPARE(host.value("nicName").toString(), QString("eth0"));
        QCOMPARE(host.value("webBindAddr").toString(), QString("0.0.0.0"));
    }

    // Reload from the freshly-written pair round-trips to a usable config.
    void coldStart_reloadRoundtrips()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config a; QVERIFY(a.load(anchor(dir).toStdString()));

        Config b; QVERIFY(b.load(anchor(dir).toStdString()));
        QVERIFY(!b.get().drives.empty());
        QCOMPARE(b.get().configVersion, 2);
        QCOMPARE(QString::fromStdString(b.get().conditioningMode), QString("bypass"));
    }

    // Single-writer isolation: saveRig() must not rewrite host.json (and vice versa).
    void saveRig_leavesHostUntouched()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));

        QFile hf(dir.path() + "/host.json"); QVERIFY(hf.open(QIODevice::ReadOnly));
        const QByteArray host0 = hf.readAll(); hf.close();

        cfg.get().conditioningMode = "interpolate";   // a rig field
        QVERIFY(cfg.saveRig(anchor(dir).toStdString()));

        QFile hf2(dir.path() + "/host.json"); QVERIFY(hf2.open(QIODevice::ReadOnly));
        const QByteArray host1 = hf2.readAll(); hf2.close();
        QCOMPARE(host1, host0);   // host.json byte-identical after a rig save

        QJsonObject rig = readObj(dir.path() + "/rig.json");
        QCOMPARE(rig.value("global").toObject().value("conditioningMode").toString(), QString("interpolate"));
    }

    void saveHost_leavesRigUntouched()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));

        QFile rf(dir.path() + "/rig.json"); QVERIFY(rf.open(QIODevice::ReadOnly));
        const QByteArray rig0 = rf.readAll(); rf.close();

        cfg.get().nicName = "Ethernet 7";   // a host field
        QVERIFY(cfg.saveHost(anchor(dir).toStdString()));

        QFile rf2(dir.path() + "/rig.json"); QVERIFY(rf2.open(QIODevice::ReadOnly));
        const QByteArray rig1 = rf2.readAll(); rf2.close();
        QCOMPARE(rig1, rig0);   // rig.json byte-identical after a host save

        QCOMPARE(readObj(dir.path() + "/host.json").value("nicName").toString(), QString("Ethernet 7"));
    }

    // /api/rig validation core: a valid rig body passes; an invalid one is caught.
    void validateRigBody_acceptsValidRejectsInvalid()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));  // writes host.json + rig.json

        QFile rf(dir.path() + "/rig.json"); QVERIFY(rf.open(QIODevice::ReadOnly));
        const QByteArray good = rf.readAll(); rf.close();
        auto errsGood = Config::validateRigBody(anchor(dir).toStdString(), good.toStdString());
        QVERIFY2(errsGood.empty(), errsGood.empty() ? "" : errsGood[0].c_str());

        // Corrupt axis 0 (strokeMm = 0) -> validator must reject.
        QJsonObject rig = QJsonDocument::fromJson(good).object();
        QJsonArray axes = rig.value("axes").toArray();
        QJsonObject a0 = axes.at(0).toObject(); a0["strokeMm"] = 0.0;
        axes.replace(0, a0); rig["axes"] = axes;
        const QByteArray bad = QJsonDocument(rig).toJson();
        QVERIFY(!Config::validateRigBody(anchor(dir).toStdString(), bad.toStdString()).empty());
    }

    // /api/host validation core: a valid host body passes.
    void validateHostBody_acceptsValid()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        QFile hf(dir.path() + "/host.json"); QVERIFY(hf.open(QIODevice::ReadOnly));
        const QByteArray host = hf.readAll(); hf.close();
        auto errs = Config::validateHostBody(anchor(dir).toStdString(), host.toStdString());
        QVERIFY2(errs.empty(), errs.empty() ? "" : errs[0].c_str());
    }

    // followingErrorWindowMm is PER-AXIS: it lands in rig.axes (never
    // rig.global) and per-axis edits round-trip independently.
    void followingError_perAxisRoundtrip()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config a; QVERIFY(a.load(anchor(dir).toStdString()));   // cold start writes the pair

        DriveConfig d2; d2.slaveIndex = 2; d2.name = "A2";
        a.get().drives.push_back(d2);
        a.get().numDrives = 2;
        a.get().drives[0].followingErrorWindowMm = 42.0;
        a.get().drives[1].followingErrorWindowMm = 7.5;
        QVERIFY(a.saveRig(anchor(dir).toStdString()));

        QJsonObject rig = readObj(dir.path() + "/rig.json");
        QVERIFY2(!rig.value("global").toObject().contains("followingErrorWindowMm"),
                 "followingError leaked into rig.global (should be per-axis)");
        QCOMPARE(rig.value("axes").toArray().at(0).toObject().value("followingErrorWindowMm").toDouble(), 42.0);

        Config b; QVERIFY(b.load(anchor(dir).toStdString()));
        QCOMPARE(static_cast<int>(b.get().drives.size()), 2);
        QCOMPARE(b.get().drives[0].followingErrorWindowMm, 42.0);
        QCOMPARE(b.get().drives[1].followingErrorWindowMm, 7.5);
    }

    // ---- Axis defaults: a MISSING key must fall back to the compiled-in
    // DriveConfig default (Config.h), which is what CONFIG_REFERENCE promises.
    // The reader used to spell a second default into every call site, and three
    // of them disagreed with the struct -- homeMode "center" vs "endstop",
    // homingSpeedMmS 5 vs 250, maxAccelerationMmS2 2000 vs 10000 -- so an axis
    // omitting them silently got the stale value. ----
    void axisDefaults_missingKeysUseStructDefaults()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/host.json", "{ \"controlLoopHz\": 500 }\n");
        // Minimal axis: identity only, every tunable omitted.
        writeText(dir.path() + "/rig.json",
            "{ \"configVersion\": 2, \"numDrives\": 1, \"global\": {},"
            "  \"axes\": [ { \"slaveIndex\": 1, \"name\": \"A1\" } ] }\n");

        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        QCOMPARE(static_cast<int>(cfg.get().drives.size()), 1);
        const DriveConfig& d = cfg.get().drives[0];
        const DriveConfig  def;   // compiled-in defaults

        // The three that had diverged -- assert against the struct, not literals,
        // so this test keeps holding if the defaults are retuned later.
        QCOMPARE(QString::fromStdString(d.homeMode), QString::fromStdString(def.homeMode));
        QCOMPARE(d.homingSpeedMmS,      def.homingSpeedMmS);
        QCOMPARE(d.maxAccelerationMmS2, def.maxAccelerationMmS2);
        // Spot-check the rest of the surface is untouched by the reader rewrite.
        QCOMPARE(QString::fromStdString(d.axisType),     QString::fromStdString(def.axisType));
        QCOMPARE(QString::fromStdString(d.homeDirection),QString::fromStdString(def.homeDirection));
        QCOMPARE(d.strokeMm,          def.strokeMm);
        QCOMPARE(d.homingBackoffMm,   def.homingBackoffMm);
        QCOMPARE(d.homingTorquePct,   def.homingTorquePct);
        QCOMPARE(d.maxVelocityMmS,    def.maxVelocityMmS);
        QCOMPARE(d.maxJerkMmS3,       def.maxJerkMmS3);
        QCOMPARE(d.unparkTimeSec,     def.unparkTimeSec);
        QCOMPARE(d.parkTimeSec,       def.parkTimeSec);
        QCOMPARE(d.torqueMinPct,      def.torqueMinPct);
        QCOMPARE(d.torqueMaxPct,      def.torqueMaxPct);
        QCOMPARE(d.spikeFilterEnabled,def.spikeFilterEnabled);
        // Positional defaults are index-derived, not struct-derived.
        QCOMPARE(d.slaveIndex, 1);
        QCOMPARE(QString::fromStdString(d.name), QString("A1"));
    }

    // Explicit values must still win over the defaults (the rewrite must not
    // have turned any read into a no-op).
    void axisDefaults_explicitValuesStillWin()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/host.json", "{ \"controlLoopHz\": 500 }\n");
        writeText(dir.path() + "/rig.json",
            "{ \"configVersion\": 2, \"numDrives\": 1, \"global\": {},"
            "  \"axes\": [ { \"slaveIndex\": 3, \"name\": \"Belt\","
            "                \"mode\": \"cst\", \"axisType\": \"linear_horizontal\","
            "                \"homeMode\": \"center\", \"homeDirection\": \"positive\","
            "                \"strokeMm\": 175.5, \"homingSpeedMmS\": 12.0,"
            "                \"maxAccelerationMmS2\": 4242.0, \"invertDir\": true,"
            "                \"spikeFilterEnabled\": true, \"homingTorquePct\": 40 } ] }\n");

        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        const DriveConfig& d = cfg.get().drives[0];
        QCOMPARE(d.slaveIndex, 3);
        QCOMPARE(QString::fromStdString(d.name), QString("Belt"));
        QCOMPARE(QString::fromStdString(d.mode), QString("torque"));   // "cst" normalised
        QCOMPARE(QString::fromStdString(d.axisType), QString("linear_horizontal"));
        QCOMPARE(QString::fromStdString(d.homeMode), QString("center"));
        QCOMPARE(QString::fromStdString(d.homeDirection), QString("positive"));
        QCOMPARE(d.strokeMm,            175.5);
        QCOMPARE(d.homingSpeedMmS,      12.0);
        QCOMPARE(d.maxAccelerationMmS2, 4242.0);
        QCOMPARE(d.homingTorquePct,     40);
        QCOMPARE(d.invertDir,           true);
        QCOMPARE(d.spikeFilterEnabled,  true);
    }

    // ---- Merge-on-save preservation (the silent-data-loss regression class:
    // a save writing only the modelled schema would drop hand-added keys). ----

    // Unknown top-level host.json key survives a save that changes a modelled
    // host field.
    void mergeOnSave_unknownHostKeyPreserved()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/host.json",
            "{ \"nicName\": \"TestNic\", \"controlLoopHz\": 500,"
            "  \"experimentalFutureField\": \"keep-me\" }\n");
        writeText(dir.path() + "/rig.json",
            "{ \"configVersion\": 2, \"numDrives\": 1, \"global\": {},"
            "  \"axes\": [ { \"slaveIndex\": 1, \"name\": \"A1\", \"strokeMm\": 100.0 } ] }\n");

        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        cfg.get().nicName = "ChangedNic";
        QVERIFY(cfg.saveHost(anchor(dir).toStdString()));

        QJsonObject host = readObj(dir.path() + "/host.json");
        QVERIFY2(host.contains("experimentalFutureField"),
                 "Unknown host.json key dropped by saveHost() — merge regression.");
        QCOMPARE(host.value("experimentalFutureField").toString(), QString("keep-me"));
        QCOMPARE(host.value("nicName").toString(), QString("ChangedNic"));
    }

    // "_comment" user annotations survive: top-level in host.json and
    // per-axis in rig.json (QJsonDocument can't keep real comments, so
    // users annotate with keys).
    void mergeOnSave_commentKeysPreserved()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/host.json",
            "{ \"_comment\": \"NIC name from ipconfig /all\","
            "  \"nicName\": \"Ethernet 2\", \"controlLoopHz\": 500 }\n");
        writeText(dir.path() + "/rig.json",
            "{ \"configVersion\": 2, \"numDrives\": 1, \"global\": {},"
            "  \"axes\": [ { \"slaveIndex\": 1, \"name\": \"Heave\", \"strokeMm\": 150.0,"
            "                \"_comment\": \"Hand-tuned — do not auto-adjust\" } ] }\n");

        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        QVERIFY(cfg.save(anchor(dir).toStdString()));

        QJsonObject host = readObj(dir.path() + "/host.json");
        QVERIFY2(host.contains("_comment"), "Top-level host _comment dropped.");
        QCOMPARE(host.value("_comment").toString(), QString("NIC name from ipconfig /all"));

        QJsonArray axes = readObj(dir.path() + "/rig.json").value("axes").toArray();
        QCOMPARE(axes.size(), 1);
        QVERIFY2(axes.at(0).toObject().contains("_comment"), "Per-axis _comment dropped.");
        QCOMPARE(axes.at(0).toObject().value("_comment").toString(),
                 QString("Hand-tuned — do not auto-adjust"));
    }

    // Unknown host keys survive when only an UNRELATED modelled field changed.
    void mergeOnSave_unknownHostKeysSurviveUnrelatedChange()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/host.json",
            "{ \"nicName\": \"NicX\", \"controlLoopHz\": 500,"
            "  \"customDebugFlag\": true, \"experimentalOffsetUs\": 42 }\n");
        writeText(dir.path() + "/rig.json",
            "{ \"configVersion\": 2, \"numDrives\": 1, \"global\": {},"
            "  \"axes\": [ { \"slaveIndex\": 1, \"name\": \"A1\", \"strokeMm\": 100.0 } ] }\n");

        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        cfg.get().telemetryPort = 5555;
        QVERIFY(cfg.saveHost(anchor(dir).toStdString()));

        QJsonObject host = readObj(dir.path() + "/host.json");
        QCOMPARE(host.value("telemetryPort").toInt(), 5555);
        QVERIFY2(host.contains("customDebugFlag"), "Unknown bool host key dropped.");
        QCOMPARE(host.value("customDebugFlag").toBool(), true);
        QVERIFY2(host.contains("experimentalOffsetUs"), "Unknown int host key dropped.");
        QCOMPARE(host.value("experimentalOffsetUs").toInt(), 42);
    }

    // Per-axis unknown fields in rig.json survive a save that changes modelled
    // per-axis fields (mergeAxes preserves by slaveIndex).
    void mergeOnSave_unknownPerAxisFieldPreserved()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/host.json",
            "{ \"nicName\": \"NicY\", \"controlLoopHz\": 500 }\n");
        writeText(dir.path() + "/rig.json",
            "{ \"configVersion\": 2, \"numDrives\": 1, \"global\": {},"
            "  \"axes\": [ { \"slaveIndex\": 1, \"name\": \"A1\", \"strokeMm\": 100.0,"
            "                \"futureExperimentalField\": \"keep me too\","
            "                \"customGain\": 1.23 } ] }\n");

        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        QVERIFY(!cfg.get().drives.empty());
        cfg.get().drives[0].name     = "A1-renamed";
        cfg.get().drives[0].strokeMm = 175.0;
        QVERIFY(cfg.saveRig(anchor(dir).toStdString()));

        QJsonArray axes = readObj(dir.path() + "/rig.json").value("axes").toArray();
        QCOMPARE(axes.size(), 1);
        QJsonObject a0 = axes.at(0).toObject();
        QCOMPARE(a0.value("name").toString(),  QString("A1-renamed"));
        QCOMPARE(a0.value("strokeMm").toDouble(), 175.0);
        QVERIFY2(a0.contains("futureExperimentalField"), "Unknown per-axis string dropped.");
        QCOMPARE(a0.value("futureExperimentalField").toString(), QString("keep me too"));
        QVERIFY2(a0.contains("customGain"), "Unknown per-axis number dropped.");
        QCOMPARE(a0.value("customGain").toDouble(), 1.23);
    }

    // webUIEnabled is a HOST field: it must land in host.json and round-trip
    // both ways.
    void webUIEnabled_roundtripsInHost()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));   // cold start
        cfg.get().webUIEnabled = true;
        QVERIFY(cfg.saveHost(anchor(dir).toStdString()));

        QCOMPARE(readObj(dir.path() + "/host.json").value("webUIEnabled").toBool(), true);

        Config b; QVERIFY(b.load(anchor(dir).toStdString()));
        QCOMPARE(b.get().webUIEnabled, true);

        b.get().webUIEnabled = false;
        QVERIFY(b.saveHost(anchor(dir).toStdString()));
        QCOMPARE(readObj(dir.path() + "/host.json").value("webUIEnabled").toBool(), false);
    }

    // Axes merge by slaveIndex: removed axes disappear, added axes appear,
    // and unknown fields on RETAINED axes survive.
    void axesMerge_addRemoveBySlaveIndex()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/host.json",
            "{ \"nicName\": \"NicZ\", \"controlLoopHz\": 500 }\n");
        writeText(dir.path() + "/rig.json",
            "{ \"configVersion\": 2, \"numDrives\": 2, \"global\": {},"
            "  \"axes\": ["
            "    { \"slaveIndex\": 1, \"name\": \"A1\", \"strokeMm\": 100.0, \"_keep\": \"x\" },"
            "    { \"slaveIndex\": 2, \"name\": \"A2\", \"strokeMm\": 100.0, \"_keep\": \"y\" } ] }\n");

        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        QCOMPARE(static_cast<int>(cfg.get().drives.size()), 2);

        cfg.get().drives.erase(cfg.get().drives.begin() + 1);   // remove axis 2
        DriveConfig added; added.slaveIndex = 3; added.name = "A3-added";
        cfg.get().drives.push_back(added);                       // add axis 3
        cfg.get().numDrives = static_cast<int>(cfg.get().drives.size());
        QVERIFY(cfg.saveRig(anchor(dir).toStdString()));

        QJsonObject rig = readObj(dir.path() + "/rig.json");
        QJsonArray axes = rig.value("axes").toArray();
        QCOMPARE(axes.size(), 2);

        QJsonObject a0 = axes.at(0).toObject();
        QCOMPARE(a0.value("slaveIndex").toInt(), 1);
        QVERIFY2(a0.contains("_keep"), "Unknown field on retained axis dropped.");
        QCOMPARE(a0.value("_keep").toString(), QString("x"));

        QCOMPARE(axes.at(1).toObject().value("slaveIndex").toInt(), 3);
        QCOMPARE(axes.at(1).toObject().value("name").toString(), QString("A3-added"));

        for (int i = 0; i < axes.size(); ++i)
            QVERIFY2(axes.at(i).toObject().value("slaveIndex").toInt() != 2,
                     "Removed slaveIndex=2 entry leaked back into rig.json.");
        QCOMPARE(rig.value("numDrives").toInt(), 2);
    }
};

QTEST_MAIN(TestConfigTwoFile)
#include "TestConfigTwoFile.moc"
