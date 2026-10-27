// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestConfigTwoFile.cpp  (Slice 2 - host/rig config split)
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
#include "../src/PlatformRT.h"
#include "../src/EffectStatus.h"
#include "../src/HapticsProfiles.h"
#include "../src/SupportRedact.h"

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
    static QByteArray readBytes(const QString& path)
    {
        QFile f(path);
        if (!f.open(QIODevice::ReadOnly)) return {};
        return f.readAll();
    }

private slots:
    // A rig.json that is there but cannot be read (truncated by a crash
    // mid-write, the way a bench Pi lost its tuning) is a hard error: load
    // fails naming the file, the file is left byte for byte as it was, the
    // other file too, and the config in memory keeps what it had. Running
    // on defaults instead let the boot normalise write them over the file.
    // Reverting Config::load's Unreadable check (the "hard stop" block)
    // makes load() return true here and the second QVERIFY fails.
    void unreadableRig_loadFailsAndTouchesNothing()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));   // cold start writes the pair
        cfg.get().drives[0].strokeMm = 123.0;
        QVERIFY(cfg.saveRig(anchor(dir).toStdString()));
        QVERIFY(cfg.load(anchor(dir).toStdString()));
        QCOMPARE(cfg.get().drives[0].strokeMm, 123.0);

        // Truncate rig.json mid-object, as a crash during a write would.
        const QByteArray whole = readBytes(dir.path() + "/rig.json");
        QVERIFY(whole.size() > 100);
        const QByteArray cut = whole.left(whole.size() / 2);
        writeText(dir.path() + "/rig.json", cut);
        const QByteArray host0 = readBytes(dir.path() + "/host.json");

        QVERIFY2(!cfg.load(anchor(dir).toStdString()), "an unreadable rig.json must fail the load, not read as defaults");
        QVERIFY2(cfg.lastError().find("rig.json") != std::string::npos, cfg.lastError().c_str());
        QCOMPARE(readBytes(dir.path() + "/rig.json"), cut);     // byte-identical: nothing written over it
        QCOMPARE(readBytes(dir.path() + "/host.json"), host0);
        QCOMPARE(cfg.get().drives[0].strokeMm, 123.0);          // the set in memory is kept

        // The saves refuse too: the merge would have nothing to keep, and
        // the file is the user's to fix.
        QVERIFY(!cfg.saveRig(anchor(dir).toStdString()));
        QCOMPARE(readBytes(dir.path() + "/rig.json"), cut);
        // A zero-byte file is the common crash signature: the same hard stop.
        writeText(dir.path() + "/rig.json", QByteArray());
        QVERIFY(!cfg.load(anchor(dir).toStdString()));
        QCOMPARE(readBytes(dir.path() + "/rig.json"), QByteArray());

        // host.json unreadable with a good rig.json: the same, naming host.json.
        writeText(dir.path() + "/rig.json", whole);
        writeText(dir.path() + "/host.json", "{ \"nicName\": ");
        QVERIFY(!cfg.load(anchor(dir).toStdString()));
        QVERIFY2(cfg.lastError().find("host.json") != std::string::npos, cfg.lastError().c_str());
        QCOMPARE(readBytes(dir.path() + "/host.json"), QByteArray("{ \"nicName\": "));
        QCOMPARE(readBytes(dir.path() + "/rig.json"), whole);
        QVERIFY(!cfg.saveHost(anchor(dir).toStdString()));
        QCOMPARE(readBytes(dir.path() + "/host.json"), QByteArray("{ \"nicName\": "));
        // Both good again: the load is back.
        writeText(dir.path() + "/host.json", host0);
        QVERIFY(cfg.load(anchor(dir).toStdString()));
        QVERIFY(cfg.lastError().empty());
    }

    // A hand-edited rig past the strip's ranges loads (never refused), with
    // the values that would hurt clamped to their range, one warning each
    // naming the file and the field, and the set then validates clean. The
    // file itself is left as written. Reverting clampHapticsOnLoad (or
    // the call in Config::load) leaves amp at 500 and no warnings.
    void outOfRange_loadClampsAndWarns()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config seed; QVERIFY(seed.load(anchor(dir).toStdString()));   // cold start: a valid pair
        QJsonObject rig = readObj(dir.path() + "/rig.json");
        QJsonObject g = rig.value("global").toObject();
        QJsonObject h = g.value("haptics").toObject();
        QJsonObject abs = h.value("abs").toObject();
        abs["ampPct"] = 500.0;
        QJsonArray routes; { QJsonObject ro; ro["shaker"] = 0; ro["gain"] = 9.0; ro["harm"] = 12; routes.append(ro); }
        abs["routes"] = routes;
        h["abs"] = abs; g["haptics"] = h; rig["global"] = g;
        const QByteArray text = QJsonDocument(rig).toJson();
        writeText(dir.path() + "/rig.json", text);

        Config cfg;
        QVERIFY2(cfg.load(anchor(dir).toStdString()), "an out-of-range value must never refuse the load");
        const haptics::EffectParams& p = cfg.get().hapticsFx[static_cast<size_t>(haptics::Effect::Abs)];
        QCOMPARE(p.ampPct, 100.0);
        QCOMPARE(p.routes[0].shaker, 0);
        QCOMPARE(p.routes[0].gain, 2.0);
        QCOMPARE(p.routes[0].harm, 8);
        const std::vector<std::string>& w = cfg.loadWarnings();
        QCOMPARE(static_cast<int>(w.size()), 3);
        for (const std::string& line : w) QVERIFY2(line.find("rig.json: haptics.abs.") == 0, line.c_str());
        QVERIFY(w[0].find("ampPct") != std::string::npos && w[0].find("500") != std::string::npos);
        QVERIFY(w[1].find("routes[0].harm") != std::string::npos && w[1].find("12") != std::string::npos);
        QVERIFY(w[2].find("routes[0].gain") != std::string::npos && w[2].find("9") != std::string::npos);
        const auto errs = cfg.get().validate();
        QVERIFY2(errs.empty(), errs.empty() ? "" : errs[0].c_str());
        QCOMPARE(readBytes(dir.path() + "/rig.json"), text);   // left as written

        // The rest of the clamped set: a position axis's haptic max and a
        // route to it in mm, the master gain and the position budget; and
        // a value the clamp does not touch is still reported, with its
        // file, and runs as read (the load succeeds).
        rig = readObj(dir.path() + "/rig.json"); g = rig.value("global").toObject(); h = g.value("haptics").toObject();
        QJsonObject road = h.value("road").toObject();
        QJsonArray rr; { QJsonObject ro; ro["axis"] = 0; ro["gain"] = 50.0; rr.append(ro); } road["routes"] = rr;
        h["road"] = road; h["masterGain"] = 5.0; h["positionBudget"] = -1.0;
        g["haptics"] = h; rig["global"] = g;
        QJsonArray axes = rig.value("axes").toArray();
        QJsonObject a0 = axes.at(0).toObject(); a0["hapticsMaxMm"] = 40.0; axes.replace(0, a0); rig["axes"] = axes;
        writeText(dir.path() + "/rig.json", QJsonDocument(rig).toJson());
        writeText(dir.path() + "/host.json", "{ \"controlLoopHz\": 123 }\n");
        Config more;
        QVERIFY(more.load(anchor(dir).toStdString()));
        QCOMPARE(more.get().drives[0].hapticsMaxMm, 10.0);
        QCOMPARE(more.get().hapticsFx[static_cast<size_t>(haptics::Effect::Road)].routes[0].gain, 10.0);   // capped by the axis's max
        QCOMPARE(more.get().hapticsMasterGain, 2.0);
        QCOMPARE(more.get().hapticsPositionBudget, 0.0);
        const std::vector<std::string>& w2 = more.loadWarnings();
        bool hz = false, maxMm = false, gain = false, master = false, budget = false;
        for (const std::string& line : w2)
        {
            if (line.find("host.json: controlLoopHz") == 0) hz = true;
            if (line.find("rig.json: axes[0].hapticsMaxMm") == 0) maxMm = true;
            if (line.find("rig.json: haptics.road.routes[0].gain") == 0) gain = true;
            if (line.find("rig.json: haptics.masterGain") == 0) master = true;
            if (line.find("rig.json: haptics.positionBudget") == 0) budget = true;
        }
        QVERIFY2(hz && maxMm && gain && master && budget, "every clamp and the validate error must be reported with its file");
        QCOMPARE(more.get().controlLoopHz, 123);   // runs as read
    }

    // The clamp never reaches the file. The two paths that load the set and
    // save it back: the Pi's boot normalise (load as written, save, then
    // clamp for running) and a server-side apply (load as written, change
    // what it owns, save; run a clamped copy). Both used to load clamped
    // and so wrote an amp of 500 back as 100, and dropped a route whose
    // gain clamped to 0.
    void outOfRange_neverWrittenBack()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config seed; QVERIFY(seed.load(anchor(dir).toStdString()));   // cold start: a valid pair
        QJsonObject rig = readObj(dir.path() + "/rig.json");
        QJsonObject g = rig.value("global").toObject();
        QJsonObject h = g.value("haptics").toObject();
        QJsonObject abs = h.value("abs").toObject();
        abs["ampPct"] = 500.0;
        // A position route on an axis whose haptic max is 0: the clamp takes
        // its gain to 0, and the writer drops a route at 0.
        QJsonArray routes; { QJsonObject ro; ro["axis"] = 0; ro["gain"] = 1.5; routes.append(ro); }
        abs["routes"] = routes;
        h["abs"] = abs; g["haptics"] = h; rig["global"] = g;
        QJsonArray axes = rig.value("axes").toArray();
        QJsonObject a0 = axes.at(0).toObject(); a0["hapticsMaxMm"] = 0.0; axes.replace(0, a0); rig["axes"] = axes;
        writeText(dir.path() + "/rig.json", QJsonDocument(rig).toJson());
        const auto absInFile = [&]() {
            return readObj(dir.path() + "/rig.json").value("global").toObject().value("haptics").toObject().value("abs").toObject();
        };

        // Boot normalise.
        Config boot;
        QVERIFY(boot.load(anchor(dir).toStdString(), /*forRun=*/false));
        QCOMPARE(boot.get().hapticsFx[static_cast<size_t>(haptics::Effect::Abs)].ampPct, 500.0);
        QVERIFY2(boot.loadWarnings().empty(), "an as-written load clamps nothing and so warns of nothing");
        QVERIFY(boot.save(anchor(dir).toStdString()));
        boot.clampForRun();
        QCOMPARE(boot.get().hapticsFx[static_cast<size_t>(haptics::Effect::Abs)].ampPct, 100.0);   // runs clamped
        QVERIFY(!boot.loadWarnings().empty());
        QCOMPARE(boot.get().hapticsFx[static_cast<size_t>(haptics::Effect::Abs)].routes[0].gain, 0.0);
        QCOMPARE(absInFile().value("ampPct").toDouble(), 500.0);
        QCOMPARE(absInFile().value("routes").toArray().size(), 1);
        QCOMPARE(absInFile().value("routes").toArray().at(0).toObject().value("gain").toDouble(), 1.5);

        // A server-side apply that moves another tile (a car entry's engine).
        Config apply;
        QVERIFY(apply.load(anchor(dir).toStdString(), /*forRun=*/false));
        QJsonObject eng; eng["ampPct"] = 42.0;
        QJsonObject applied; applied["rpmVibe"] = eng;
        Config::readHapticsObject(applied, apply.get());
        AppConfig run = apply.get();
        std::vector<std::string> clamped;
        Config::clampHaptics(run, clamped);
        QVERIFY(!clamped.empty());
        QCOMPARE(run.hapticsFx[static_cast<size_t>(haptics::Effect::Abs)].ampPct, 100.0);
        QVERIFY(apply.saveRig(anchor(dir).toStdString()));
        QCOMPARE(absInFile().value("ampPct").toDouble(), 500.0);
        QCOMPARE(absInFile().value("routes").toArray().size(), 1);
        QCOMPARE(readObj(dir.path() + "/rig.json").value("global").toObject().value("haptics").toObject()
                     .value("rpmVibe").toObject().value("ampPct").toDouble(), 42.0);

        // And a plain load still runs clamped (what the mains run).
        Config plain;
        QVERIFY(plain.load(anchor(dir).toStdString()));
        QCOMPARE(plain.get().hapticsFx[static_cast<size_t>(haptics::Effect::Abs)].ampPct, 100.0);
    }

    // The atomic writer (QSaveFile: a temporary of its own, renamed over
    // the target on commit) must not be stopped by a stale rig.json.tmp
    // beside the file, which an interrupted web save leaves behind, and
    // must leave that file alone. (A truncating write passes this too; the
    // truncation it guards against is a crash-time event, which the
    // unreadable-file test above covers from the other side.)
    void saveRig_landsBesideStaleTmp()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        writeText(dir.path() + "/rig.json.tmp", "garbage left by an interrupted save");
        cfg.get().drives[0].name = "Landed";
        QVERIFY(cfg.saveRig(anchor(dir).toStdString()));
        QJsonObject rig = readObj(dir.path() + "/rig.json");
        QCOMPARE(rig.value("axes").toArray().at(0).toObject().value("name").toString(), QString("Landed"));
        QCOMPARE(readBytes(dir.path() + "/rig.json.tmp"), QByteArray("garbage left by an interrupted save"));
        Config back; QVERIFY(back.load(anchor(dir).toStdString()));
        QCOMPARE(QString::fromStdString(back.get().drives[0].name), QString("Landed"));
    }

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
    // and wiping the host fields it had just read - every fresh Pi install
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

    // The belt cue's source and its g settings survive save -> load; a
    // rig.json without them reads the defaults (the cue stays the motion
    // line's, as before).
    void beltFromG_roundTrip()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config a; QVERIFY(a.load(anchor(dir).toStdString()));
        QVERIFY(!a.get().drives.empty());
        QCOMPARE(QString::fromStdString(a.get().drives[0].beltSource), QString("motion"));
        DriveConfig& d = a.get().drives[0];
        d.beltSource = "g"; d.beltGMode = "accel"; d.beltGFromG = 0.2; d.beltGFullG = 2.5;
        d.beltGCurve = 1.5; d.beltGSmoothMs = 45.0;
        QCOMPARE(QString::fromStdString(d.beltGSway), QString("off"));
        QCOMPARE(QString::fromStdString(d.beltGHeave), QString("off"));
        d.beltGSway = "right"; d.beltGSwayMix = 0.75; d.beltGHeave = "lift"; d.beltGHeaveMix = 2.0;
        QVERIFY(a.save(anchor(dir).toStdString()));
        Config b; QVERIFY(b.load(anchor(dir).toStdString()));
        const DriveConfig& r = b.get().drives[0];
        QCOMPARE(QString::fromStdString(r.beltSource), QString("g"));
        QCOMPARE(QString::fromStdString(r.beltGMode), QString("accel"));
        QCOMPARE(r.beltGFromG, 0.2); QCOMPARE(r.beltGFullG, 2.5);
        QCOMPARE(r.beltGCurve, 1.5); QCOMPARE(r.beltGSmoothMs, 45.0);
        QCOMPARE(QString::fromStdString(r.beltGSway), QString("right"));
        QCOMPARE(QString::fromStdString(r.beltGHeave), QString("lift"));
        QCOMPARE(r.beltGSwayMix, 0.75); QCOMPARE(r.beltGHeaveMix, 2.0);
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

    // The haptics block round-trips exactly: every effect's tuning and routes,
    // the engine description, and the master gain survive save -> load. The
    // Pi file surprised the bench once; this pins the format.
    void haptics_roundTrip()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config a; QVERIFY(a.load(anchor(dir).toStdString()));
        AppConfig& c = a.get();
        for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
        {
            haptics::EffectParams& p = c.hapticsFx[static_cast<size_t>(i)];
            p.ampPct = 10.0 + i; p.freqHz = 20.0 + i; p.durMs = 20.0; p.jitter = 0.05 * i; p.peakPct = 30.0 + i;
            p.routes[0] = { 0, 0.5 + 0.1 * i }; p.routes[1] = { 1, 1.0 };
        }
        c.hapticsEngine.cylinders = 8; c.hapticsEngine.litres = 6.5; c.hapticsEngine.layout = 1;
        c.hapticsEngine.maxRpm = 7000; c.hapticsEngine.rock = 0.7; c.hapticsEngine.thump = 0.6;
        c.hapticsEngine.buzz = 0.9; c.hapticsEngine.order = 0.5; c.hapticsEngine.limHit = 1.2;
        c.hapticsEngine.limHz = 10; c.hapticsEngine.limJit = 0.3;
        c.hapticsMasterGain = 1.3;
        c.hapticsSlipLat = { 0.7, 28.0, 0.4, 9.0, 6.5, 35.0, 1.5, 10.0, 45.0, 0.0, 0.45, 60.0, 40.0, 0.8, 7.5 };
        c.hapticsSlipLon = { 0.9, 8.0, 0.2, 12.0, 0.6 };
        c.hapticsRoad    = { 12.0, 3.5, 0.35, 140.0, 18.0, 2.0, 1.5, 4.0, 18.0, 0.4, 3.0, 65.0, 6.5, 1.0 };
        c.hapticsKerb    = { 30.0, 10.0, 4.0, 12.0, 15.0 };
        c.hapticsAbs     = { 0.6, 0.8, 0.4, 0.5, 55.0, 1.0, 0.45, 0.2, 65.0 };
        c.hapticsTc      = { 0.2, 0.9, 0.0, 0.0, 40.0 };
        c.hapticsAbs.holdMs = 120.0; c.hapticsTc.holdMs = 0.0;   // the TC hold off
        c.hapticsAbs.fromWheels = 2.0;   // on its own
        c.hapticsAbs.lowOff = 0.0;                 // default on: off round-trips
        c.hapticsSlipLon.screech = 0.45; c.hapticsSlipLon.screechPct = 4.5; c.hapticsSlipLon.screechHz = 42.0;   // the edge
        c.hapticsSlipLat.baseMode = 2.0; c.hapticsSlipLat.baseDegF = 2.4; c.hapticsSlipLat.baseDegR = 1.1;      // the lateral baseline fixed
        c.hapticsSurface = { 0.4, 0.8, 1.0, 0.3, 110.0, 0.5 };
        c.hapticsFx[static_cast<size_t>(haptics::Effect::Surface)].routes[0].part = haptics::Part::RL;
        c.hapticsWheels  = { 2.5, 0.5, 0.8, 1.5 };
        QCOMPARE(c.hapticsWheels.freshOnReset, 1.0);   // the default: on
        c.hapticsWheels.freshOnReset = 0.0;
        c.hapticsImpacts = { 700.0, 3000.0, 2.0 };
        c.hapticsFx[static_cast<size_t>(haptics::Effect::Wheels)].routes[1].part = haptics::Part::Rear;
        c.hapticsFx[static_cast<size_t>(haptics::Effect::Abs)].routes[0].part = haptics::Part::FR;
        c.hapticsFx[static_cast<size_t>(haptics::Effect::Kerb)].routes[1].part = haptics::Part::Front;
        c.hapticsFx[static_cast<size_t>(haptics::Effect::Road)].routes[0].part = haptics::Part::RR;
        c.hapticsDriveline = { 0.8, 9.0, 0.6, 6.5, 1.0, 0.7, 0.4, 55.0 };
        c.hapticsDriveline.ignoreN = 0.0;          // default on: off round-trips
        c.hapticsDriveline.whinePitch = 3.0;       // default below the buzz: above round-trips
        c.hapticsDriveline.diff = 0.4; c.hapticsDriveline.diffType = 1.0; c.hapticsDriveline.diffHz = 9.0; c.hapticsDriveline.driven = 2.0;
        c.hapticsEngine.inertia = 0.3; c.hapticsEngine.turbo = 1; c.hapticsEngine.liftoff = 1.5; c.hapticsEngine.pops = 0.4;
        c.hapticsEngine.tcCut = 0.6;
        // Route parts on the per-wheel tiles survive; "all" is the default.
        c.hapticsFx[static_cast<size_t>(haptics::Effect::Skid)].routes[0].part   = haptics::Part::FL;
        c.hapticsFx[static_cast<size_t>(haptics::Effect::Lockup)].routes[1].part = haptics::Part::Rear;
        // A shaker route beside the axis routes: shaker index, gain, harmonic.
        { haptics::Route sr; sr.shaker = 3; sr.gain = 0.7; sr.harm = 2;
          c.hapticsFx[static_cast<size_t>(haptics::Effect::Kerb)].routes[2] = sr; }
        c.hapticsAxisDelayMs = 12.0;
        QVERIFY(a.saveRig(anchor(dir).toStdString()));

        Config b; QVERIFY(b.load(anchor(dir).toStdString()));
        const AppConfig& r = b.get();
        {
            const haptics::Route& sr = r.hapticsFx[static_cast<size_t>(haptics::Effect::Kerb)].routes[2];
            QCOMPARE(sr.shaker, 3); QCOMPARE(sr.axis, -1); QCOMPARE(sr.gain, 0.7); QCOMPARE(sr.harm, 2);
            QCOMPARE(r.hapticsAxisDelayMs, 12.0);
        }
        for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
        {
            const haptics::EffectParams& p = r.hapticsFx[static_cast<size_t>(i)];
            QCOMPARE(p.ampPct, 10.0 + i); QCOMPARE(p.freqHz, 20.0 + i); QCOMPARE(p.jitter, 0.05 * i);
            QCOMPARE(p.peakPct, 30.0 + i);
            QCOMPARE(p.routes[0].axis, 0); QCOMPARE(p.routes[0].gain, 0.5 + 0.1 * i);
            QCOMPARE(p.routes[1].axis, 1); QCOMPARE(p.routes[1].gain, 1.0);
            QCOMPARE(p.routes[2].axis, -1);
        }
        QCOMPARE(r.hapticsEngine.cylinders, 8.0); QCOMPARE(r.hapticsEngine.litres, 6.5);
        QCOMPARE(r.hapticsEngine.layout, 1.0);    QCOMPARE(r.hapticsEngine.maxRpm, 7000.0);
        QCOMPARE(r.hapticsEngine.rock, 0.7);      QCOMPARE(r.hapticsEngine.thump, 0.6);
        QCOMPARE(r.hapticsEngine.buzz, 0.9);      QCOMPARE(r.hapticsEngine.order, 0.5);
        QCOMPARE(r.hapticsEngine.limHit, 1.2);    QCOMPARE(r.hapticsEngine.limHz, 10.0);
        QCOMPARE(r.hapticsEngine.limJit, 0.3);    QCOMPARE(r.hapticsMasterGain, 1.3);
        QCOMPARE(r.hapticsSlipLat.aMix, 0.7); QCOMPARE(r.hapticsSlipLat.aHz, 28.0); QCOMPARE(r.hapticsSlipLat.bMix, 0.4);
        QCOMPARE(r.hapticsSlipLat.bHz, 9.0);  QCOMPARE(r.hapticsSlipLat.peak, 6.5);
        QCOMPARE(r.hapticsSlipLat.onsetPct, 35.0); QCOMPARE(r.hapticsSlipLat.ease, 1.5);
        QCOMPARE(r.hapticsSlipLat.smoothHz, 10.0); QCOMPARE(r.hapticsSlipLat.attackMs, 45.0);
        QCOMPARE(r.hapticsSlipLat.budget, 0.0);   // saved off, read back off (the default is on)
        QCOMPARE(r.hapticsSlipLon.aMix, 0.9); QCOMPARE(r.hapticsSlipLon.aHz, 8.0);  QCOMPARE(r.hapticsSlipLon.bMix, 0.2);
        QCOMPARE(r.hapticsSlipLon.bHz, 12.0); QCOMPARE(r.hapticsSlipLon.peak, 0.6);
        QVERIFY(r.hapticsFx[static_cast<size_t>(haptics::Effect::Skid)].routes[0].part   == haptics::Part::FL);
        QVERIFY(r.hapticsFx[static_cast<size_t>(haptics::Effect::Skid)].routes[1].part   == haptics::Part::All);
        QVERIFY(r.hapticsFx[static_cast<size_t>(haptics::Effect::Lockup)].routes[1].part == haptics::Part::Rear);
        QCOMPARE(r.hapticsRoad.fullMm, 12.0); QCOMPARE(r.hapticsRoad.hpHz, 3.5);
        QCOMPARE(r.hapticsRoad.surface, 0.35); QCOMPARE(r.hapticsRoad.surfaceKmh, 140.0); QCOMPARE(r.hapticsRoad.surfaceHz, 18.0);
        QCOMPARE(r.hapticsRoad.model, 2.0);  QCOMPARE(r.hapticsRoad.bodyMm, 1.5); QCOMPARE(r.hapticsRoad.bodyHz, 4.0);
        QCOMPARE(r.hapticsRoad.hopHz, 18.0); QCOMPARE(r.hapticsRoad.damping, 0.4); QCOMPARE(r.hapticsRoad.rough, 3.0);
        QCOMPARE(r.hapticsRoad.suspPct, 65.0); QCOMPARE(r.hapticsRoad.suspHz, 6.5); QCOMPARE(r.hapticsRoad.fullLearn, 1.0);
        QCOMPARE(r.hapticsKerb.pitchCm, 30.0); QCOMPARE(r.hapticsKerb.riseMm, 10.0); QCOMPARE(r.hapticsKerb.ribMm, 4.0);
        QCOMPARE(r.hapticsKerb.fullMm, 12.0);  QCOMPARE(r.hapticsKerb.detectMm, 15.0);
        QVERIFY(r.hapticsFx[static_cast<size_t>(haptics::Effect::Kerb)].routes[1].part == haptics::Part::Front);
        QCOMPARE(r.hapticsAbs.sharp, 0.6); QCOMPARE(r.hapticsAbs.spread, 0.8); QCOMPARE(r.hapticsAbs.slow, 0.4);
        QCOMPARE(r.hapticsAbs.buzz, 0.5);  QCOMPARE(r.hapticsAbs.buzzHz, 55.0);
        QCOMPARE(r.hapticsAbs.slipLink, 1.0); QCOMPARE(r.hapticsAbs.lockLink, 0.45); QCOMPARE(r.hapticsAbs.scrubLink, 0.2);
        QCOMPARE(r.hapticsAbs.brakeFull, 65.0);
        QCOMPARE(r.hapticsAbs.holdMs, 120.0); QCOMPARE(r.hapticsTc.holdMs, 0.0);
        QCOMPARE(r.hapticsAbs.fromWheels, 2.0);
        QCOMPARE(r.hapticsAbs.lowOff, 0.0); QCOMPARE(r.hapticsTc.lowOff, 1.0);   // TC carries none
        QCOMPARE(r.hapticsDriveline.ignoreN, 0.0);
        QCOMPARE(r.hapticsDriveline.whinePitch, 3.0);
        QCOMPARE(r.hapticsDriveline.diff, 0.4); QCOMPARE(r.hapticsDriveline.diffType, 1.0);
        QCOMPARE(r.hapticsDriveline.diffHz, 9.0); QCOMPARE(r.hapticsDriveline.driven, 2.0);
        QCOMPARE(r.hapticsSlipLon.screech, 0.45); QCOMPARE(r.hapticsSlipLon.screechPct, 4.5); QCOMPARE(r.hapticsSlipLon.screechHz, 42.0);
        QCOMPARE(r.hapticsSlipLat.screech, 0.45); QCOMPARE(r.hapticsSlipLat.screechPct, 60.0); QCOMPARE(r.hapticsSlipLat.screechHz, 40.0);
        QCOMPARE(r.hapticsSlipLat.pulse, 0.8);    QCOMPARE(r.hapticsSlipLat.pulseHz, 7.5);
        QCOMPARE(r.hapticsSlipLat.baseMode, 2.0); QCOMPARE(r.hapticsSlipLat.baseDegF, 2.4); QCOMPARE(r.hapticsSlipLat.baseDegR, 1.1);
        QCOMPARE(r.hapticsSlipLon.baseMode, 0.0);   // Longitudinal carries no baseline
        {   // the baseline is a choice: 1.5 is none of them
            const auto names = [](const AppConfig& x) {
                for (const std::string& e : x.validate()) if (e.find("baseMode") != std::string::npos) return true;
                return false; };
            AppConfig bad = r; bad.hapticsSlipLat.baseMode = 1.5;
            QVERIFY(names(bad));
            bad.hapticsSlipLat.baseMode = 1.0;
            QVERIFY(!names(bad));
        }
        QCOMPARE(r.hapticsTc.sharp, 0.2);  QCOMPARE(r.hapticsTc.spread, 0.9);
        QCOMPARE(r.hapticsTc.slow, 0.0);   QCOMPARE(r.hapticsTc.buzz, 0.0);   // TC carries neither
        QVERIFY(r.hapticsFx[static_cast<size_t>(haptics::Effect::Abs)].routes[0].part == haptics::Part::FR);
        QCOMPARE(r.hapticsSurface.stones, 0.4);  QCOMPARE(r.hapticsSurface.crunch, 0.8); QCOMPARE(r.hapticsSurface.studs, 1.0);
        QCOMPARE(r.hapticsSurface.puddles, 0.3); QCOMPARE(r.hapticsSurface.aquaKmh, 110.0); QCOMPARE(r.hapticsSurface.smooth, 0.5);
        QVERIFY(r.hapticsFx[static_cast<size_t>(haptics::Effect::Surface)].routes[0].part == haptics::Part::RL);
        QCOMPARE(r.hapticsImpacts.fromMmS, 700.0); QCOMPARE(r.hapticsImpacts.fullMmS, 3000.0); QCOMPARE(r.hapticsImpacts.heaveG, 2.0);
        QCOMPARE(r.hapticsWheels.balance, 2.5); QCOMPARE(r.hapticsWheels.flat, 0.5); QCOMPARE(r.hapticsWheels.judder, 0.8); QCOMPARE(r.hapticsWheels.puncture, 1.5);
        QCOMPARE(r.hapticsWheels.freshOnReset, 0.0);   // saved off, read back off (the default is on)
        QVERIFY(r.hapticsFx[static_cast<size_t>(haptics::Effect::Wheels)].routes[1].part == haptics::Part::Rear);
        QVERIFY(r.hapticsFx[static_cast<size_t>(haptics::Effect::Road)].routes[0].part == haptics::Part::RR);
        QCOMPARE(r.hapticsDriveline.clutch, 0.8); QCOMPARE(r.hapticsDriveline.clutchHz, 9.0);
        QCOMPARE(r.hapticsDriveline.lug, 0.6);    QCOMPARE(r.hapticsDriveline.lugHz, 6.5);
        QCOMPARE(r.hapticsDriveline.gearbox, 1.0); QCOMPARE(r.hapticsDriveline.whine, 0.7);
        QCOMPARE(r.hapticsDriveline.shunt, 0.4);   QCOMPARE(r.hapticsDriveline.shuntHz, 55.0);
        QCOMPARE(r.hapticsEngine.inertia, 0.3);   QCOMPARE(r.hapticsEngine.turbo, 1.0);
        QCOMPARE(r.hapticsEngine.liftoff, 1.5);   QCOMPARE(r.hapticsEngine.pops, 0.4);
        QCOMPARE(r.hapticsEngine.tcCut, 0.6);

        // The file carries the tiles under their current keys with the
        // readable slip fields, and the old keys are gone.
        QJsonObject rig = readObj(dir.path() + "/rig.json");
        QJsonObject h = rig.value("global").toObject().value("haptics").toObject();
        QVERIFY(h.contains("slipLat") && h.contains("slipLon"));
        QVERIFY(!h.contains("skid") && !h.contains("lockup"));
        QCOMPARE(h.value("slipLat").toObject().value("scrubHz").toDouble(), 28.0);
        QCOMPARE(h.value("slipLon").toObject().value("peakRatio").toDouble(), 0.6);
        QCOMPARE(h.value("slipLat").toObject().value("routes").toArray().at(0).toObject().value("part").toString(), QString("fl"));
    }

    // A 0.9.6 rig file saved the two slip tiles as "skid" and "lockup" with
    // one carrier each. Their amplitude, routes and carrier must come back
    // under the new tiles so a tuned rig keeps feeling the same after the
    // update, and the next save writes the new keys.
    void haptics_legacySlipKeysMigrate()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config a; QVERIFY(a.load(anchor(dir).toStdString()));
        QJsonObject rig = readObj(dir.path() + "/rig.json");
        QJsonObject g = rig.value("global").toObject();
        QJsonObject h = g.value("haptics").toObject();
        h.remove("slipLat"); h.remove("slipLon");
        QJsonObject skid;   skid["ampPct"] = 35.0;   skid["freqHz"] = 40.0; skid["jitter"] = 0.5;
        QJsonArray sr; { QJsonObject ro; ro["axis"] = 3; ro["gain"] = 1.0; sr.append(ro); } skid["routes"] = sr;
        QJsonObject lockup; lockup["ampPct"] = 60.0; lockup["freqHz"] = 7.0; lockup["jitter"] = 0.2;
        QJsonArray lr; { QJsonObject ro; ro["axis"] = 0; ro["gain"] = 0.8; lr.append(ro); } lockup["routes"] = lr;
        h["skid"] = skid; h["lockup"] = lockup;
        g["haptics"] = h; rig["global"] = g;
        const QByteArray legacyText = QJsonDocument(rig).toJson();
        writeText(dir.path() + "/rig.json", legacyText);

        Config b; QVERIFY(b.load(anchor(dir).toStdString()));
        // The one rewrite a load makes keeps the file as it was beside it
        // (rig.json.bak, byte for byte). Reverting the backupFile call in
        // Config::load fails the next line.
        QCOMPARE(readBytes(dir.path() + "/rig.json.bak"), legacyText);
        const AppConfig& r = b.get();
        const haptics::EffectParams& lat = r.hapticsFx[static_cast<size_t>(haptics::Effect::Skid)];
        const haptics::EffectParams& lon = r.hapticsFx[static_cast<size_t>(haptics::Effect::Lockup)];
        QCOMPARE(lat.ampPct, 35.0); QCOMPARE(lat.jitter, 0.5);
        QCOMPARE(lat.routes[0].axis, 3); QCOMPARE(lat.routes[0].gain, 1.0);
        QVERIFY(lat.routes[0].part == haptics::Part::All);
        QCOMPARE(r.hapticsSlipLat.aHz, 40.0);           // the old carrier became the scrub carrier
        QCOMPARE(r.hapticsSlipLat.bHz, 11.0);           // the slide keeps its default
        QCOMPARE(lon.ampPct, 60.0); QCOMPARE(lon.jitter, 0.2);
        QCOMPARE(lon.routes[0].axis, 0); QCOMPARE(lon.routes[0].gain, 0.8);
        QCOMPARE(r.hapticsSlipLon.aHz, 7.0);            // the old carrier became the lock carrier

        // The load itself rewrote the file under the new keys (the web serves
        // rig.json as is, so it must never see the retired keys), keeping
        // the migrated tuning.
        QJsonObject h2 = readObj(dir.path() + "/rig.json").value("global").toObject().value("haptics").toObject();
        QVERIFY(h2.contains("slipLat") && h2.contains("slipLon"));
        QVERIFY(!h2.contains("skid") && !h2.contains("lockup"));
        QCOMPARE(h2.value("slipLat").toObject().value("ampPct").toDouble(), 35.0);
        QCOMPARE(h2.value("slipLat").toObject().value("scrubHz").toDouble(), 40.0);
        QCOMPARE(h2.value("slipLon").toObject().value("routes").toArray().at(0).toObject().value("gain").toDouble(), 0.8);
    }

    // The sticky per-sim effect record: channel-spec matching, delivered /
    // produced per game, transient attribution, persistence beside rig.json.
    void effectStatus_recordsAndPersists()
    {
        using haptics::Effect; using haptics::effectInfo;
        bool have[NcxTok::TokenCount] = {};
        double fx[haptics::FX_TYPE_COUNT] = {};
        uint64_t fired[haptics::EVENT_TYPE_COUNT] = {};

        // Channel spec matching: "a|b", "group*", "~optional". Lateral slip
        // needs road speed as well (silent at a standstill).
        QVERIFY(!EffectStatus::channelsDelivered(effectInfo(Effect::Skid), have));
        have[NcxTok::Skid] = true;
        QVERIFY2(!EffectStatus::channelsDelivered(effectInfo(Effect::Skid), have), "skid without speed is not delivered");
        have[NcxTok::SpeedKmh] = true;
        QVERIFY2(EffectStatus::channelsDelivered(effectInfo(Effect::Skid), have), "skid + speed satisfies slipAngle*|skid, speedKmh");
        have[NcxTok::Skid] = false;
        for (int w = 0; w < 3; ++w) have[NcxTok::SlipAngleFL + w] = true;
        QVERIFY2(!EffectStatus::channelsDelivered(effectInfo(Effect::Skid), have), "three of four wheels is not the group");
        have[NcxTok::SlipAngleRR] = true;
        QVERIFY2(EffectStatus::channelsDelivered(effectInfo(Effect::Skid), have), "all four wheels satisfy slipAngle*");
        QVERIFY2(EffectStatus::channelsDelivered(effectInfo(Effect::DetentClick), have), "no channels needed = delivered");
        have[NcxTok::Rpm] = true;
        QVERIFY2(EffectStatus::channelsDelivered(effectInfo(Effect::Engine), have), "engine needs rpm only (throttle, limiter optional in the law)");

        EffectStatus es;
        const int64_t t0 = 1700000000000LL;
        // Game A: slip channels arrive, nothing plays -> delivered, not produced.
        es.observe("A", have, fx, fired, true, t0);
        QCOMPARE(es.currentGame(), std::string("A"));
        auto rA = es.records("A");
        QVERIFY(rA[(size_t)Effect::Skid].delivered && !rA[(size_t)Effect::Skid].produced);
        QVERIFY(!rA[(size_t)Effect::Road].delivered);
        // The lateral slip plays -> produced.
        fx[(int)haptics::FxType::Skid] = 0.3;
        es.observe("A", have, fx, fired, true, t0 + 1000);
        QVERIFY(es.records("A")[(size_t)Effect::Skid].produced);
        // A gear shift transient fires -> the gearShift tile, not the detent.
        have[NcxTok::Gear] = true;
        fired[(int)haptics::EventType::GearShift] = 1;
        es.observe("A", have, fx, fired, true, t0 + 2000);
        QVERIFY(es.records("A")[(size_t)Effect::GearShift].produced);
        QVERIFY(!es.records("A")[(size_t)Effect::DetentClick].produced);
        // Game B starts: its own record, A untouched; stream gone = nothing changes.
        fx[(int)haptics::FxType::Skid] = 0.0;
        es.observe("B", have, fx, fired, true, t0 + 3000);
        QCOMPARE(es.currentGame(), std::string("B"));
        QVERIFY(es.records("B")[(size_t)Effect::Skid].delivered && !es.records("B")[(size_t)Effect::Skid].produced);
        QVERIFY(es.records("A")[(size_t)Effect::Skid].produced);
        es.observe("", have, fx, fired, false, t0 + 4000);
        QCOMPARE(es.currentGame(), std::string("B"));
        // A detent click on the bench (no stream) is still recorded.
        fired[(int)haptics::EventType::DetentClick] = 1;
        es.observe("", have, fx, fired, false, t0 + 5000);
        QVERIFY(es.records("B")[(size_t)Effect::DetentClick].produced);
        const auto games = es.games();
        QCOMPARE(games.size(), (size_t)2); QCOMPARE(games[0], std::string("B"));

        // Persist and reload beside the config anchor.
        QTemporaryDir dir; QVERIFY(dir.isValid());
        QVERIFY(es.dirty());
        QVERIFY(es.save(anchor(dir).toStdString()));
        QVERIFY(!es.dirty());
        QVERIFY(QFile::exists(dir.path() + "/effectstatus.json"));
        EffectStatus back; QVERIFY(back.load(anchor(dir).toStdString()));
        QCOMPARE(back.currentGame(), std::string("B"));
        QVERIFY(back.records("A")[(size_t)Effect::Skid].produced);
        QVERIFY(back.records("A")[(size_t)Effect::GearShift].produced);
        QVERIFY(back.records("B")[(size_t)Effect::Skid].delivered && !back.records("B")[(size_t)Effect::Skid].produced);
        QCOMPARE(back.records("A")[(size_t)Effect::Skid].lastSeenMs, t0 + 2000);
        back.clear("A");
        QCOMPARE(back.games().size(), (size_t)1);
        back.clearAll();
        QVERIFY(back.games().empty());
    }

    // Haptics profiles: a profile is the rig haptics object exactly (both
    // Config statics round-trip it), the store persists names, bindings and
    // the active one, and the car binding wins over the game's.
    void hapticsProfiles_storeAndRoundTrip()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config a; QVERIFY(a.load(anchor(dir).toStdString()));
        AppConfig& c = a.get();
        c.hapticsFx[static_cast<size_t>(haptics::Effect::Skid)].ampPct = 42.0;
        c.hapticsFx[static_cast<size_t>(haptics::Effect::Skid)].routes[0] = { 2, 1.5, haptics::Part::FL };
        c.hapticsEngine.cylinders = 6; c.hapticsEngine.layout = 1; c.hapticsEngine.turbo = 1;
        c.hapticsMasterGain = 0.9; c.hapticsPositionBudget = 0.25;
        const QJsonObject snap = Config::writeHapticsObject(c);
        QCOMPARE(snap.value("slipLat").toObject().value("ampPct").toDouble(), 42.0);
        QCOMPARE(snap.value("masterGain").toDouble(), 0.9);

        // Read it into a fresh config: everything comes back.
        AppConfig d;
        Config::readHapticsObject(snap, d);
        QCOMPARE(d.hapticsFx[static_cast<size_t>(haptics::Effect::Skid)].ampPct, 42.0);
        QCOMPARE(d.hapticsFx[static_cast<size_t>(haptics::Effect::Skid)].routes[0].axis, 2);
        QCOMPARE(d.hapticsFx[static_cast<size_t>(haptics::Effect::Skid)].routes[0].gain, 1.5);
        QVERIFY(d.hapticsFx[static_cast<size_t>(haptics::Effect::Skid)].routes[0].part == haptics::Part::FL);
        QCOMPARE(d.hapticsEngine.cylinders, 6.0); QCOMPARE(d.hapticsEngine.layout, 1.0); QCOMPARE(d.hapticsEngine.turbo, 1.0);
        QCOMPARE(d.hapticsMasterGain, 0.9); QCOMPARE(d.hapticsPositionBudget, 0.25);
        // A partial object (an older profile) leaves the rest alone.
        QJsonObject partial; QJsonObject kerb; kerb["ampPct"] = 7.0; partial["kerb"] = kerb;
        Config::readHapticsObject(partial, d);
        QCOMPARE(d.hapticsFx[static_cast<size_t>(haptics::Effect::Kerb)].ampPct, 7.0);
        QCOMPARE(d.hapticsFx[static_cast<size_t>(haptics::Effect::Skid)].ampPct, 42.0);
        QCOMPARE(d.hapticsMasterGain, 0.9);

        HapticsProfiles ps;
        ps.put("GT3", snap);
        QJsonObject quiet = snap; quiet["masterGain"] = 0.3;
        ps.put("Quiet", quiet);
        ps.setActive("GT3");
        ps.bind(HapticsProfiles::carKey("Porsche 992 GT3 R"), "GT3");
        ps.bind(HapticsProfiles::gameKey("AMS2"), "Quiet");
        QCOMPARE(ps.profileFor("Porsche 992 GT3 R", "AMS2"), std::string("GT3"));   // car wins
        QCOMPARE(ps.profileFor("Formula Trainer", "AMS2"), std::string("Quiet"));   // game fallback
        QCOMPARE(ps.profileFor("Formula Trainer", "AC"), std::string(""));          // nothing bound
        QCOMPARE(ps.profileFor("", ""), std::string(""));
        ps.bind("car:x", "NoSuchProfile");
        QVERIFY(ps.bindings().count("car:x") == 0);
        QVERIFY(ps.save(anchor(dir).toStdString()));
        QVERIFY(QFile::exists(dir.path() + "/profiles.json"));

        HapticsProfiles back; QVERIFY(back.load(anchor(dir).toStdString()));
        QCOMPARE(back.names().size(), (size_t)2);
        QCOMPARE(back.active(), std::string("GT3"));
        QCOMPARE(back.get("Quiet").value("masterGain").toDouble(), 0.3);
        QCOMPARE(back.profileFor("Porsche 992 GT3 R", ""), std::string("GT3"));
        QVERIFY(back.remove("GT3"));
        QCOMPARE(back.active(), std::string(""));
        QCOMPARE(back.profileFor("Porsche 992 GT3 R", "AMS2"), std::string("Quiet"));   // the car binding went with it
        QVERIFY(!back.remove("GT3"));
    }

    // A profiles.json that is there but cannot be read keeps the set in
    // memory, fails the load naming the error, and is never written over:
    // a save goes to profiles.json.rejected and reports failure, until a
    // later load reads the file. It used to parse to an empty set and
    // return true, and the next save wrote that emptiness over the user's
    // profiles. Reverting the parse check in HapticsProfiles::load makes
    // the load succeed with an empty set and the "kept" row fail.
    void hapticsProfiles_unreadableFileIsNeverWrittenOver()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        const std::string anchor = (dir.path() + "/config.json").toStdString();
        HapticsProfiles ps;
        QJsonObject h; h["masterGain"] = 0.5;
        ps.put("Mine", h);
        QVERIFY(ps.save(anchor));
        const QByteArray good = readBytes(dir.path() + "/profiles.json");
        QVERIFY(good.size() > 20);
        const QByteArray cut = good.left(good.size() / 2);
        writeText(dir.path() + "/profiles.json", cut);

        HapticsProfiles back;
        back.put("Before", h);
        QVERIFY2(!back.load(anchor), "an unreadable profiles.json must fail the load");
        QVERIFY(!back.lastError().empty());
        QCOMPARE(back.names().size(), (size_t)1);
        QCOMPARE(back.names()[0], std::string("Before"));            // kept
        back.put("After", h);
        QVERIFY2(!back.save(anchor), "a save after a refused load must report failure");
        QCOMPARE(readBytes(dir.path() + "/profiles.json"), cut);       // never written over
        QVERIFY(QFile::exists(dir.path() + "/profiles.json.rejected"));
        QVERIFY(readBytes(dir.path() + "/profiles.json.rejected").contains("\"After\""));
        // The file readable again: the load takes it and saves land.
        writeText(dir.path() + "/profiles.json", good);
        QVERIFY(back.load(anchor));
        QVERIFY(back.lastError().empty());
        QCOMPARE(back.names().size(), (size_t)1);
        QCOMPARE(back.names()[0], std::string("Mine"));
        back.put("Again", h);
        QVERIFY(back.save(anchor));
        QVERIFY(readBytes(dir.path() + "/profiles.json").contains("\"Again\""));
        // No file at all is no error worth more than false, and saves are fine.
        QTemporaryDir empty; QVERIFY(empty.isValid());
        HapticsProfiles fresh;
        QVERIFY(!fresh.load((empty.path() + "/config.json").toStdString()));
        QVERIFY(fresh.lastError().empty());
        fresh.put("New", h);
        QVERIFY(fresh.save((empty.path() + "/config.json").toStdString()));
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
    // of them disagreed with the struct -- parkMode "center" vs "endstop",
    // homingSpeed 5 vs 250, maxAccelerationMmS2 2000 vs 10000 -- so an axis
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
        QCOMPARE(QString::fromStdString(d.parkMode), QString::fromStdString(def.parkMode));
        QCOMPARE(d.homingSpeed,      def.homingSpeed);
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
        QCOMPARE(d.beltSlewPctPerSec, def.beltSlewPctPerSec);
        // Behaviour, not the number: an axis that omits the key must get a cap
        // that passes the motion software's tension cue sharply. The cap acts
        // on the telemetry tension only (the haptics overlay is never slewed);
        // a cue swinging 30% at 100 Hz needs 2*pi*f*A ~ 18850 %/s, which the
        // old default (3000) rounded into a triangle a quarter the size.
        QVERIFY2(d.beltSlewPctPerSec >= 6.283185307 * 100.0 * 30.0,
                 "default belt slew cap blunts a 30% tension cue at 100 Hz");
        QCOMPARE(d.spikeFilterEnabled,def.spikeFilterEnabled);
        // Positional defaults are index-derived, not struct-derived.
        QCOMPARE(d.slaveIndex, 1);
        QCOMPARE(QString::fromStdString(d.name), QString("A1"));
    }

    // Explicit values must still win over the defaults (the rewrite must not
    // have turned any read into a no-op).
    // parkMode/homingSpeed were homeMode/homingSpeedMmS until 0.9.2. A rig.json
    // written before the rename must keep its values: falling back to the struct
    // defaults would move where the axis parks and jump the homing speed, both
    // silently.
    void deprecatedKeys_homeModeAndHomingSpeedMmS_stillRead()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/host.json", "{ \"controlLoopHz\": 500 }\n");
        writeText(dir.path() + "/rig.json",
            "{ \"configVersion\": 2, \"numDrives\": 1, \"global\": {},"
            "  \"axes\": [ { \"slaveIndex\": 1, \"axisType\": \"linear_horizontal\","
            "                \"homeMode\": \"center\", \"homingSpeedMmS\": 8.0 } ] }\n");

        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        const DriveConfig& d = cfg.get().drives[0];
        QCOMPARE(QString::fromStdString(d.parkMode), QString("center"));
        QCOMPARE(d.homingSpeed, 8.0);
    }

    // Both spellings present: the current name wins, so a file mid-migration is
    // not decided by key order.
    void deprecatedKeys_newNameWinsOverOld()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/host.json", "{ \"controlLoopHz\": 500 }\n");
        writeText(dir.path() + "/rig.json",
            "{ \"configVersion\": 2, \"numDrives\": 1, \"global\": {},"
            "  \"axes\": [ { \"slaveIndex\": 1,"
            "                \"homeMode\": \"endstop\", \"parkMode\": \"center\","
            "                \"homingSpeedMmS\": 8.0, \"homingSpeed\": 42.0 } ] }\n");

        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        const DriveConfig& d = cfg.get().drives[0];
        QCOMPARE(QString::fromStdString(d.parkMode), QString("center"));
        QCOMPARE(d.homingSpeed, 42.0);
    }

    void axisDefaults_explicitValuesStillWin()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/host.json", "{ \"controlLoopHz\": 500 }\n");
        writeText(dir.path() + "/rig.json",
            "{ \"configVersion\": 2, \"numDrives\": 1, \"global\": {},"
            "  \"axes\": [ { \"slaveIndex\": 3, \"name\": \"Belt\","
            "                \"mode\": \"cst\", \"axisType\": \"linear_horizontal\","
            "                \"parkMode\": \"center\", \"homeDirection\": \"positive\","
            "                \"strokeMm\": 175.5, \"homingSpeed\": 12.0,"
            "                \"maxAccelerationMmS2\": 4242.0, \"invertDir\": true,"
            "                \"spikeFilterEnabled\": true, \"homingTorquePct\": 40 } ] }\n");

        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        const DriveConfig& d = cfg.get().drives[0];
        QCOMPARE(d.slaveIndex, 3);
        QCOMPARE(QString::fromStdString(d.name), QString("Belt"));
        QCOMPARE(QString::fromStdString(d.mode), QString("torque"));   // "cst" normalised
        QCOMPARE(QString::fromStdString(d.axisType), QString("linear_horizontal"));
        QCOMPARE(QString::fromStdString(d.parkMode), QString("center"));
        QCOMPARE(QString::fromStdString(d.homeDirection), QString("positive"));
        QCOMPARE(d.strokeMm,            175.5);
        QCOMPARE(d.homingSpeed,      12.0);
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
                 "Unknown host.json key dropped by saveHost() - merge regression.");
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
            "                \"_comment\": \"Hand-tuned - do not auto-adjust\" } ] }\n");

        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        QVERIFY(cfg.save(anchor(dir).toStdString()));

        QJsonObject host = readObj(dir.path() + "/host.json");
        QVERIFY2(host.contains("_comment"), "Top-level host _comment dropped.");
        QCOMPARE(host.value("_comment").toString(), QString("NIC name from ipconfig /all"));

        QJsonArray axes = readObj(dir.path() + "/rig.json").value("axes").toArray();
        QCOMPARE(axes.size(), 1);
        QVERIFY2(axes.at(0).toObject().contains("_comment"), "Per-axis _comment dropped.");
        QCOMPARE(axes.at(0).toObject().value("_comment").toString(),
                 QString("Hand-tuned - do not auto-adjust"));
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

    // Unknown keys INSIDE the haptics object survive a save: a tile this
    // build does not know and a setting inside a tile it does (a newer
    // version's, read back by this one), the way the unknown top-level
    // keys do; the tile's own keys land over it. The haptics object used
    // to be replaced whole, so every server-side apply (a car entry, a
    // starting point, a profile) dropped them. Reverting mergeHaptics (the
    // globalSchema["haptics"] line in saveRig) fails the first QVERIFY.
    void mergeOnSave_unknownHapticsKeysPreserved()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config seed; QVERIFY(seed.load(anchor(dir).toStdString()));
        QJsonObject rig = readObj(dir.path() + "/rig.json");
        QJsonObject g = rig.value("global").toObject();
        QJsonObject h = g.value("haptics").toObject();
        QJsonObject future; future["ampPct"] = 12.0; future["_note"] = "a tile this build does not have";
        h["futureTile"] = future;
        QJsonObject abs = h.value("abs").toObject(); abs["futureParam"] = 7.0; abs["_comment"] = "mine"; abs["ampPct"] = 20.0;
        h["abs"] = abs;
        g["haptics"] = h; rig["global"] = g;
        writeText(dir.path() + "/rig.json", QJsonDocument(rig).toJson());

        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        cfg.get().hapticsFx[static_cast<size_t>(haptics::Effect::Abs)].ampPct = 25.0;   // an edit the save carries
        QVERIFY(cfg.saveRig(anchor(dir).toStdString()));
        const QJsonObject h2 = readObj(dir.path() + "/rig.json").value("global").toObject().value("haptics").toObject();
        QVERIFY2(h2.value("futureTile").isObject(), "unknown tile dropped by saveRig");
        QCOMPARE(h2.value("futureTile").toObject().value("ampPct").toDouble(), 12.0);
        const QJsonObject abs2 = h2.value("abs").toObject();
        QVERIFY2(abs2.contains("futureParam"), "unknown key inside a tile dropped by saveRig");
        QCOMPARE(abs2.value("futureParam").toDouble(), 7.0);
        QCOMPARE(abs2.value("_comment").toString(), QString("mine"));
        QCOMPARE(abs2.value("ampPct").toDouble(), 25.0);                  // the schema's value lands
        QVERIFY(abs2.value("routes").isArray());
        // A tile that is not an object in the file is replaced by the schema's.
        QJsonObject rig3 = readObj(dir.path() + "/rig.json");
        QJsonObject g3 = rig3.value("global").toObject(); QJsonObject h3 = g3.value("haptics").toObject();
        h3["kerb"] = 5; g3["haptics"] = h3; rig3["global"] = g3;
        writeText(dir.path() + "/rig.json", QJsonDocument(rig3).toJson());
        Config again; QVERIFY(again.load(anchor(dir).toStdString()));
        QVERIFY(again.saveRig(anchor(dir).toStdString()));
        QVERIFY(readObj(dir.path() + "/rig.json").value("global").toObject().value("haptics").toObject().value("kerb").isObject());
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

    // ---- the control loop's CPU (host.json rtPinCore / rtCore) ----
    // Round-trips through host.json; auto (-1) and a CPU the machine does
    // not have both resolve to the auto core, which is a real CPU of this
    // machine; unpinned resolves to none.
    void rtCore_hostKeysAndResolve()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config c; QVERIFY(c.load(anchor(dir).toStdString()));   // cold start
        QCOMPARE(c.get().rtPinCore, true);
        QCOMPARE(c.get().rtCore, -1);
        QCOMPARE(c.get().asyncSend, true);
        c.get().rtPinCore = false; c.get().rtCore = 14; c.get().asyncSend = false;
        QVERIFY(c.saveHost(anchor(dir).toStdString()));
        Config back; QVERIFY(back.load(anchor(dir).toStdString()));
        QCOMPARE(back.get().rtPinCore, false);
        QCOMPARE(back.get().rtCore, 14);
        QCOMPARE(back.get().asyncSend, false);   // the Windows send path switch rides host.json too
        back.get().rtCore = 999;
        bool flagged = false;
        for (const std::string& e : back.get().validate()) flagged = flagged || e.rfind("rtCore", 0) == 0;
        QVERIFY2(flagged, "an impossible rtCore is a validate() error");

        const int n = PlatformRT::logicalCpuCount();
        const int a = PlatformRT::autoRtCore();
        qInfo("this machine: %d logical CPUs, auto loop CPU %d", n, a);
        QVERIFY(a >= 0 && a < n);
        QCOMPARE(PlatformRT::resolveRtCore(-1, true), a);
        QCOMPARE(PlatformRT::resolveRtCore(n + 3, true), a);   // a CPU this machine lacks: auto
        QCOMPARE(PlatformRT::resolveRtCore(0, true), 0);
        QCOMPARE(PlatformRT::resolveRtCore(5, false), -1);     // not pinned
    }

    // ---- the haptics parameter contract ----
    // Every tunable the strip offers (a HapticsRegistry ParamSpec row) lives
    // in the config and comes back from a save and a load exactly: a row
    // added without its storage, or stored but not read or not written,
    // fails here naming the tile and the key. Two passes with different
    // values, so a value that happens to be the default cannot pass by
    // chance. And every key the writer puts in a tile's object is a
    // registry row of that tile, a key every tile carries, or one kept for
    // older files below: nothing is written that nothing can tune.
    void hapticsContract_everyRegistryRowRoundTrips()
    {
        const auto valueFor = [](const haptics::ParamSpec& p, int i, int k, int pass) -> double
        {
            if (p.opts)   // a choice: an index inside the row's range
            {
                const int n = static_cast<int>(p.max - p.min) + 1;
                return p.min + ((i + k + pass) % std::max(1, n));
            }
            const int s = ((i * 37 + k * 11 + pass * 29) % 89) + 1;   // 1..89
            return p.min + (p.max - p.min) * s / 91.0;
        };
        QStringList bad;
        for (int pass = 0; pass < 2; ++pass)
        {
            QJsonObject in;
            for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
            {
                const haptics::EffectInfo& info = haptics::effectInfo(i);
                QJsonObject o;
                for (int k = 0; k < info.paramCount; ++k) o[info.params[k].key] = valueFor(info.params[k], i, k, pass);
                in[info.key] = o;
            }
            AppConfig c;
            Config::readHapticsObject(in, c);
            const QJsonObject out = Config::writeHapticsObject(c);
            for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
            {
                const haptics::EffectInfo& info = haptics::effectInfo(i);
                const QJsonObject o = out.value(info.key).toObject();
                for (int k = 0; k < info.paramCount; ++k)
                {
                    const char* key = info.params[k].key;
                    const double want = valueFor(info.params[k], i, k, pass);
                    if (!o.contains(key) || o.value(key).toDouble() != want)
                        bad << QString("%1.%2 (pass %3: in %4, out %5)").arg(info.key, key).arg(pass).arg(want)
                                   .arg(o.contains(key) ? QString::number(o.value(key).toDouble()) : QString("absent"));
                }
            }
        }
        QVERIFY2(bad.isEmpty(), qPrintable("registry rows that do not round-trip: " + bad.join("; ")));

        // Every written key accounted for.
        static const char* const kEveryTile[] = { "ampPct", "freqHz", "durMs", "jitter", "peakPct", "routes" };
        const QJsonObject out = Config::writeHapticsObject(AppConfig{});
        QStringList stray;
        for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
        {
            const haptics::EffectInfo& info = haptics::effectInfo(i);
            const QJsonObject o = out.value(info.key).toObject();
            for (auto it = o.begin(); it != o.end(); ++it)
            {
                const std::string key = it.key().toStdString();
                bool known = false;
                for (const char* e : kEveryTile) known = known || key == e;
                for (int k = 0; k < info.paramCount && !known; ++k) known = key == info.params[k].key;
                if (!known) stray << QString("%1.%2").arg(info.key, it.key());
            }
        }
        QVERIFY2(stray.isEmpty(), qPrintable("keys written that no registry row offers: " + stray.join(", ")));
    }

    // The support bundle (the dashboard's Download logs) carries host.json
    // and rig.json with the web password blanked: an escaped quote inside it
    // is part of it, an empty one stays empty, every occurrence goes, the
    // name as a value is not a key, and a shape that cannot be blanked (not
    // a string, never ends) is refused so the file is left out.
    void supportBundle_blanksTheWebPassword()
    {
        // The password: se"sa\me, as JSON writes it.
        std::string t = "{\n  \"webAuthToken\" :  \"se\\\"sa\\\\me\",\n  \"nicName\": \"eth0\"\n}";
        QVERIFY(t.find("se\\\"sa\\\\me") != std::string::npos);   // the check below can fail
        QVERIFY(redactWebAuthToken(t));
        QVERIFY2(t.find("se\\\"sa") == std::string::npos && t.find("sa\\\\me") == std::string::npos, t.c_str());
        QVERIFY(t.find("\"webAuthToken\" :  \"<redacted>\",") != std::string::npos);
        QVERIFY(t.find("\"nicName\": \"eth0\"") != std::string::npos);
        std::string e = "{\"webAuthToken\":\"\"}";
        QVERIFY(redactWebAuthToken(e));
        QCOMPARE(e, std::string("{\"webAuthToken\":\"\"}"));   // no password set: nothing to hide
        std::string two = "{\"a\":{\"webAuthToken\":\"x1\"},\"webAuthToken\":\"x2\",\"note\":\"webAuthToken\"}";
        QVERIFY(redactWebAuthToken(two));
        QVERIFY(two.find("x1") == std::string::npos && two.find("x2") == std::string::npos);
        QVERIFY(two.find("\"note\":\"webAuthToken\"") != std::string::npos);
        std::string none = "{\"nicName\":\"eth0\"}";
        QVERIFY(redactWebAuthToken(none));
        QCOMPARE(none, std::string("{\"nicName\":\"eth0\"}"));
        std::string num = "{\"webAuthToken\": 1234}";
        QVERIFY2(!redactWebAuthToken(num), "a value that is not a string is refused");
        std::string open = "{\"webAuthToken\": \"secret";
        QVERIFY2(!redactWebAuthToken(open), "a value that never ends is refused");
    }

    // The validator holds every registry row to the range and the choices
    // the strip offers (C1 stage 2: from the registry, no hand-kept list):
    // past either end, a row is an error naming the tile and the key, and
    // a choice set to a fraction is one too. A fresh config has none.
    void hapticsContract_validateHoldsEveryRow()
    {
        const auto hapticsErrors = [](const AppConfig& c)
        {
            QStringList out;
            for (const std::string& e : c.validate()) if (e.rfind("haptics.", 0) == 0) out << QString::fromStdString(e);
            return out;
        };
        QVERIFY2(hapticsErrors(AppConfig{}).isEmpty(), qPrintable(hapticsErrors(AppConfig{}).join("; ")));
        QStringList missed;
        for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
        {
            const haptics::EffectInfo& info = haptics::effectInfo(i);
            for (int k = 0; k < info.paramCount; ++k)
            {
                const haptics::ParamSpec& ps = info.params[k];
                const QString want = QString("haptics.%1: %2 ").arg(info.key, ps.key);
                const double span = ps.max - ps.min;
                for (const double bad : { ps.max + span + 1.0, ps.min - span - 1.0 })
                {
                    AppConfig c;
                    double* f = Config::hapticsField(c, i, ps.key);
                    QVERIFY(f);
                    *f = bad;
                    bool named = false;
                    for (const QString& e : hapticsErrors(c)) named = named || e.startsWith(want);
                    if (!named) missed << QString("%1.%2 = %3").arg(info.key, ps.key).arg(bad);
                }
                if (ps.opts && span >= 1.0)
                {
                    AppConfig c;
                    *Config::hapticsField(c, i, ps.key) = ps.min + 0.5;
                    bool named = false;
                    for (const QString& e : hapticsErrors(c)) named = named || (e.startsWith(want) && e.contains("choices"));
                    if (!named) missed << QString("%1.%2 = %3 (a choice)").arg(info.key, ps.key).arg(ps.min + 0.5);
                }
            }
        }
        QVERIFY2(missed.isEmpty(), qPrintable("values the validator let through: " + missed.join("; ")));
    }

    // Shaker shaping lives in rig.json (global.shakerShaping), by shaker
    // number, and only for shakers that are not flat and off. Every field
    // round-trips, host.json never carries it, and a shaker put back to
    // flat leaves the file (the key is always written, so the merge on
    // save cannot keep a stale entry alive). Dropping the shakerShaping
    // block from writeRigGlobal or readRigGlobal fails the first QCOMPARE.
    void shakerShaping_roundTripsInRigOnly()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        haptics::ShakerShape s;
        s.eqOn = true;
        s.band[0].on = true; s.band[0].hz = 18.0; s.band[0].q = 0.9;
        s.band[2].type = haptics::BandType::LowShelf; s.band[2].hz = 45.0; s.band[2].db = -7.5; s.band[2].q = 1.3;
        s.band[4].type = haptics::BandType::Bell; s.band[4].on = true; s.band[4].hz = 120.0; s.band[4].db = 3.0;
        s.comp.on = true; s.comp.thresholdDb = -21.0; s.comp.ratio = 5.5; s.comp.kneeDb = 4.0;
        s.comp.attackMs = 7.0; s.comp.releaseMs = 350.0; s.comp.makeupDb = 2.5;
        cfg.get().shakerShaping[2] = s;
        QVERIFY(cfg.saveRig(anchor(dir).toStdString()));

        const QJsonArray arr = readObj(dir.path() + "/rig.json").value("global").toObject().value("shakerShaping").toArray();
        QCOMPARE(arr.size(), 1);                                   // only the shaped shaker
        QCOMPARE(arr.at(0).toObject().value("shaker").toInt(), 3); // by its number, 1-based
        QCOMPARE(arr.at(0).toObject().value("eq").toObject().value("bands").toArray().at(2).toObject().value("type").toString(), QString("lowshelf"));
        QVERIFY2(!readObj(dir.path() + "/host.json").contains("shakerShaping"), "shaping is the rig's, never host.json's");

        Config back; QVERIFY(back.load(anchor(dir).toStdString()));
        QVERIFY2(back.get().shakerShaping[2] == s, "every shaping field round-trips through rig.json");
        for (size_t i = 0; i < back.get().shakerShaping.size(); ++i)
            if (i != 2) QVERIFY(back.get().shakerShaping[i].isDefault());
        QVERIFY(back.loadWarnings().empty());

        back.get().shakerShaping[2] = haptics::ShakerShape{};
        QVERIFY(back.saveRig(anchor(dir).toStdString()));
        const QJsonObject g = readObj(dir.path() + "/rig.json").value("global").toObject();
        QVERIFY2(g.value("shakerShaping").isArray() && g.value("shakerShaping").toArray().isEmpty(),
                 "a shaker put back to flat leaves the file (the key stays, empty)");
        Config again; QVERIFY(again.load(anchor(dir).toStdString()));
        QVERIFY(again.get().shakerShaping[2].isDefault());
    }

    // A hand-edited rig.json with shaping out of range runs clamped, with a
    // warning naming the shaker; a web save carrying the same is refused
    // (validateRigBody), naming it too. An entry for shaker 9 is dropped.
    void shakerShaping_clampedOnLoadRefusedOnSave()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        Config seed; QVERIFY(seed.load(anchor(dir).toStdString()));
        QJsonObject rig = readObj(dir.path() + "/rig.json");
        QJsonObject g = rig.value("global").toObject();
        QJsonObject eq; eq["on"] = true;
        QJsonArray bands; QJsonObject b0; b0["type"] = "highpass"; b0["hz"] = 5000.0; b0["on"] = true; bands.append(b0);
        eq["bands"] = bands;
        QJsonObject comp; comp["on"] = true; comp["ratio"] = 99.0;
        QJsonObject e; e["shaker"] = 2; e["eq"] = eq; e["comp"] = comp;
        QJsonObject ninth; ninth["shaker"] = 9; ninth["comp"] = comp;   // (not "far": a Windows macro)
        // Shaker 3: only a band type the build does not know, on band 1
        // (a high-pass by default, so writing it back as a bell shows).
        QJsonObject eq3; QJsonArray b3; QJsonObject notch; notch["type"] = "notch"; b3.append(notch); eq3["bands"] = b3;
        QJsonObject e3; e3["shaker"] = 3; e3["eq"] = eq3;
        QJsonArray arr; arr.append(e); arr.append(ninth); arr.append(e3);
        g["shakerShaping"] = arr; rig["global"] = g;
        writeText(dir.path() + "/rig.json", QJsonDocument(rig).toJson());

        const QString body = QString::fromUtf8(QJsonDocument(rig).toJson());
        bool named = false;
        for (const std::string& err : Config::validateRigBody(anchor(dir).toStdString(), body.toStdString()))
            named = named || QString::fromStdString(err).contains("shakerShaping for shaker 2");
        QVERIFY2(named, "a web save with shaping out of range is refused, naming the shaker");
        bool typeNamed = false;
        for (const std::string& err : Config::validateRigBody(anchor(dir).toStdString(), body.toStdString()))
            typeNamed = typeNamed || QString::fromStdString(err).contains("shakerShaping for shaker 3");
        QVERIFY2(typeNamed, "a web save with a band type the build does not know is refused, naming the shaker");

        Config cfg; QVERIFY(cfg.load(anchor(dir).toStdString()));
        const haptics::ShakerShape& s = cfg.get().shakerShaping[1];
        QCOMPARE(s.band[0].hz, haptics::shape_k::kHzMax);
        QCOMPARE(s.comp.ratio, haptics::shape_k::kRatioMax);
        QVERIFY(s.eqOn && s.comp.on);
        bool warned = false;
        for (const std::string& w : cfg.loadWarnings()) warned = warned || QString::fromStdString(w).contains("shakerShaping for shaker 2");
        QVERIFY2(warned, "the clamp on load says which shaker");
        bool typeWarned = false;
        for (const std::string& w : cfg.loadWarnings()) typeWarned = typeWarned || QString::fromStdString(w).contains("shakerShaping for shaker 3");
        QVERIFY2(typeWarned, "a band type the build does not know is in the load warnings, naming the shaker");
        QVERIFY(cfg.get().shakerShaping[2].band[0].type == haptics::BandType::HighPass);   // the band's own type back
        // A load not for a run (the server's own applies, the Pi's start)
        // saved straight back writes the band's own type too, not a bell.
        Config raw; QVERIFY(raw.load(anchor(dir).toStdString(), false));
        QVERIFY(raw.saveRig(anchor(dir).toStdString()));
        QString written;
        for (const QJsonValue& v : readObj(dir.path() + "/rig.json").value("global").toObject().value("shakerShaping").toArray())
            if (v.toObject().value("shaker").toInt() == 3)
                written = v.toObject().value("eq").toObject().value("bands").toArray().at(0).toObject().value("type").toString();
        QVERIFY2(written.isEmpty() || written == "highpass", qPrintable("an unknown band type saved back as " + written));
        for (size_t i = 0; i < cfg.get().shakerShaping.size(); ++i)
            if (i != 1) QVERIFY(cfg.get().shakerShaping[i].isDefault());
    }
};

QTEST_MAIN(TestConfigTwoFile)
#include "TestConfigTwoFile.moc"
