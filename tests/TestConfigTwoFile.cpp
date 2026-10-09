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
#include "../src/EffectStatus.h"
#include "../src/HapticsProfiles.h"

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
        c.hapticsSlipLat = { 0.7, 28.0, 0.4, 9.0, 6.5 };
        c.hapticsSlipLon = { 0.9, 8.0, 0.2, 12.0, 0.6 };
        c.hapticsRoad    = { 12.0, 3.5, 0.35, 140.0, 18.0 };
        c.hapticsFx[static_cast<size_t>(haptics::Effect::Road)].routes[0].part = haptics::Part::RR;
        c.hapticsDriveline = { 0.8, 9.0, 0.6, 6.5, 1.0, 0.7, 0.4, 55.0 };
        c.hapticsEngine.inertia = 0.3; c.hapticsEngine.turbo = 1; c.hapticsEngine.liftoff = 1.5; c.hapticsEngine.pops = 0.4;
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
        QCOMPARE(r.hapticsSlipLon.aMix, 0.9); QCOMPARE(r.hapticsSlipLon.aHz, 8.0);  QCOMPARE(r.hapticsSlipLon.bMix, 0.2);
        QCOMPARE(r.hapticsSlipLon.bHz, 12.0); QCOMPARE(r.hapticsSlipLon.peak, 0.6);
        QVERIFY(r.hapticsFx[static_cast<size_t>(haptics::Effect::Skid)].routes[0].part   == haptics::Part::FL);
        QVERIFY(r.hapticsFx[static_cast<size_t>(haptics::Effect::Skid)].routes[1].part   == haptics::Part::All);
        QVERIFY(r.hapticsFx[static_cast<size_t>(haptics::Effect::Lockup)].routes[1].part == haptics::Part::Rear);
        QCOMPARE(r.hapticsRoad.fullMm, 12.0); QCOMPARE(r.hapticsRoad.hpHz, 3.5);
        QCOMPARE(r.hapticsRoad.surface, 0.35); QCOMPARE(r.hapticsRoad.surfaceKmh, 140.0); QCOMPARE(r.hapticsRoad.surfaceHz, 18.0);
        QVERIFY(r.hapticsFx[static_cast<size_t>(haptics::Effect::Road)].routes[0].part == haptics::Part::RR);
        QCOMPARE(r.hapticsDriveline.clutch, 0.8); QCOMPARE(r.hapticsDriveline.clutchHz, 9.0);
        QCOMPARE(r.hapticsDriveline.lug, 0.6);    QCOMPARE(r.hapticsDriveline.lugHz, 6.5);
        QCOMPARE(r.hapticsDriveline.gearbox, 1.0); QCOMPARE(r.hapticsDriveline.whine, 0.7);
        QCOMPARE(r.hapticsDriveline.shunt, 0.4);   QCOMPARE(r.hapticsDriveline.shuntHz, 55.0);
        QCOMPARE(r.hapticsEngine.inertia, 0.3);   QCOMPARE(r.hapticsEngine.turbo, 1.0);
        QCOMPARE(r.hapticsEngine.liftoff, 1.5);   QCOMPARE(r.hapticsEngine.pops, 0.4);

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
        writeText(dir.path() + "/rig.json", QJsonDocument(rig).toJson());

        Config b; QVERIFY(b.load(anchor(dir).toStdString()));
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
