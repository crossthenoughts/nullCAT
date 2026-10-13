// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestCarTable.cpp - the shipped car table and the user's layer
//
//   - lookup is by game + car as the stream names them, normalised: the
//     game resolves through the file's alias map, case and punctuation
//     in the car string never miss an entry
//   - a name with no entry of its own lands on a near match: the longest
//     entry of the same game it starts with, in whole words, two or more;
//     an exact entry always wins, a one-word prefix or another game never
//   - the user layer wins over stock on the same key; forget restores stock
//   - the car defaults (a car with no entry, follow car on) carry exactly
//     the car-owned keys at a fresh install's values and move nothing else
//   - an entry applied through Config::readHapticsObject moves only the
//     five car-owned tiles and keeps every route, the slip tiles, master
//     gain and the follow switch exactly as they were
//   - save-for-this-car round trip: the car-owned fields come back, routes
//     and learned values do not
//   - the user file survives a save/load, and the shipped resources/
//     cars.json parses with the counts this build ships
// ============================================================

#include <QtTest>
#include <QTemporaryDir>
#include <algorithm>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include "../src/CarTable.h"
#include "../src/Config.h"
#include "../src/HapticsRegistry.h"

class TestCarTable : public QObject
{
    Q_OBJECT

    static void writeText(const QString& path, const QByteArray& bytes)
    {
        QFile f(path); QVERIFY(f.open(QIODevice::WriteOnly)); f.write(bytes); f.close();
    }
    static QByteArray stockFile()
    {
        return R"({
  "games": { "ac": "AssettoCorsa", "ams2": ["Automobilista2", "AMS2"] },
  "cars": {
    "ac:ks_mazda_787b": {
      "name": "Mazda 787B",
      "engine": {"ampPct": 45, "cylinders": 4, "litres": 2.6, "layout": 3, "turbo": 0, "rock": 0.4, "thump": 0.5, "buzz": 1.0, "jitter": 0.2, "inertia": 0.35, "liftoff": 1.0, "pops": 0.8, "limHit": 1.2, "limHz": 14, "limJit": 0.3, "freqHz": 30},
      "limiter": {"ampPct": 35, "freqHz": 14, "jitter": 0.2},
      "driveline": {"ampPct": 35, "gearbox": 1, "whine": 0.6, "shunt": 0.6, "shuntHz": 45, "clutch": 0.8, "clutchHz": 12, "lug": 0.4, "lugHz": 8, "jitter": 0.2},
      "abs": {"ampPct": 0, "freqHz": 12}, "tc": {"ampPct": 0, "freqHz": 15},
      "notes": "four-rotor"
    },
    "ams2:Lotus 98T": {
      "engine": {"ampPct": 50, "cylinders": 6, "litres": 1.5, "layout": 1, "turbo": 1, "liftoff": 1.8},
      "limiter": {"ampPct": 30, "freqHz": 15},
      "driveline": {"ampPct": 30, "gearbox": 1, "whine": 0.9},
      "abs": {"ampPct": 0, "freqHz": 12}, "tc": {"ampPct": 0, "freqHz": 15},
      "notes": "Renault V6 turbo"
    }
  }
})";
    }
    static AppConfig rigWithRoutes()
    {
        AppConfig c;
        DriveConfig d; d.mode = "torque"; d.name = "belt";
        c.drives.push_back(d);
        c.numDrives = 1;
        const auto fx = [&c](haptics::Effect e) -> haptics::EffectParams& { return c.hapticsFx[static_cast<size_t>(e)]; };
        fx(haptics::Effect::Engine).ampPct = 20; fx(haptics::Effect::Engine).routes[0].axis = 0; fx(haptics::Effect::Engine).routes[0].gain = 0.7;
        fx(haptics::Effect::Limiter).routes[0].axis = 0; fx(haptics::Effect::Limiter).routes[0].gain = 0.3;
        fx(haptics::Effect::Driveline).routes[0].axis = 0; fx(haptics::Effect::Driveline).routes[0].gain = 1.1;
        fx(haptics::Effect::Skid).ampPct = 60; fx(haptics::Effect::Skid).routes[0].axis = 0; fx(haptics::Effect::Skid).routes[0].gain = 0.5;
        c.hapticsEngine.cylinders = 8; c.hapticsEngine.maxRpm = 7200; c.hapticsEngine.order = 3;
        c.hapticsSlipLat.peak = 6.5;
        c.hapticsMasterGain = 1.3;
        c.hapticsFollowCar = false;
        return c;
    }

private slots:
    void lookup_normalisesGameAndCar()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/cars.json", stockFile());
        CarTable t;
        QVERIFY(t.loadStock((dir.path() + "/cars.json").toStdString()));
        QCOMPARE(t.stockCount(), 2);
        QCOMPARE(t.userCount(), 0);

        CarTable::Match m = t.find("AssettoCorsa", "ks_mazda_787b");
        QCOMPARE(m.source, std::string("stock"));
        QCOMPARE(m.key, std::string("ac:ks_mazda_787b"));
        QCOMPARE(m.name, std::string("Mazda 787B"));
        QCOMPARE(m.notes, std::string("four-rotor"));

        // The alias map: either spelling of the game, any case; the car
        // string's case and punctuation do not matter either.
        QCOMPARE(t.find("Automobilista2", "Lotus 98T").source, std::string("stock"));
        QCOMPARE(t.find("ams2", "lotus-98t").source, std::string("stock"));
        QCOMPARE(t.find("AMS2", "LOTUS 98T ").key, std::string("ams2:Lotus 98T"));
        QCOMPARE(t.find("assettocorsa", "KS_MAZDA_787B").key, std::string("ac:ks_mazda_787b"));
        // No name in the entry: the key's car half stands in.
        QCOMPARE(t.find("ams2", "Lotus 98T").name, std::string("Lotus 98T"));

        // By the display name too: senders differ (SimHub names the car,
        // a shared-memory reader has the folder id), any of them must land.
        QCOMPARE(t.find("AssettoCorsa", "Mazda 787B").key, std::string("ac:ks_mazda_787b"));
        QCOMPARE(t.find("Assetto Corsa", "mazda-787b").key, std::string("ac:ks_mazda_787b"));
        QCOMPARE(t.find("AC", "MAZDA 787B").source, std::string("stock"));
        QVERIFY(t.find("Automobilista2", "Mazda 787B").source.empty());   // the name belongs to AC
        // findKey is keys only: a display name is not a key.
        QVERIFY(t.findKey("ac:Mazda 787B").source.empty());

        // Misses: wrong game, unknown car, no car at all.
        QVERIFY(t.find("Automobilista2", "ks_mazda_787b").source.empty());
        QVERIFY(t.find("AssettoCorsa", "ks_mazda_mx5").source.empty());
        QVERIFY(t.find("AssettoCorsa", "").source.empty());

        // keyFor files a new entry under the game's short id.
        QCOMPARE(t.keyFor("AssettoCorsa", " ks_ferrari_f40 "), std::string("ac:ks_ferrari_f40"));
        QCOMPARE(t.keyFor("RFactor2", "Some Car"), std::string("rfactor2:Some Car"));

        // The list: both games, sorted by game then name.
        const auto items = t.list();
        QCOMPARE(items.size(), size_t(2));
        QCOMPARE(items[0].game, std::string("ac"));
        QCOMPARE(items[1].game, std::string("ams2"));
    }

    void nearMatch_wholeWordsSameGame()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/cars.json", R"({
  "games": { "ac": "AssettoCorsa", "ams2": ["Automobilista2", "AMS2"] },
  "cars": {
    "ams2:MINI Cooper S 1965": { "name": "MINI Cooper S 1965", "engine": {"ampPct": 42}, "notes": "base" },
    "ams2:MINI Cooper S":      { "name": "MINI Cooper S", "engine": {"ampPct": 40}, "notes": "tier 2" },
    "ams2:Caterham":           { "name": "Caterham", "engine": {"ampPct": 30}, "notes": "one word" },
    "ams2:Lotus 98T":          { "engine": {"ampPct": 50}, "notes": "turbo" },
    "ac:ks_mazda_787b":        { "name": "Mazda 787B", "engine": {"ampPct": 45}, "notes": "four-rotor" }
  }
})");
        CarTable t;
        QVERIFY(t.loadStock((dir.path() + "/cars.json").toStdString()));

        // The sim's "B" after the year: the longest entry it starts with.
        CarTable::Match m = t.find("Automobilista2", "MINI Cooper S 1965 B");
        QCOMPARE(m.key, std::string("ams2:MINI Cooper S 1965"));
        QCOMPARE(m.source, std::string("stock"));
        QVERIFY(m.nearMatch);
        QCOMPARE(m.notes, std::string("base"));
        // Without the year's entry the shorter one is next.
        m = t.find("ams2", "MINI Cooper S 1966");
        QCOMPARE(m.key, std::string("ams2:MINI Cooper S"));
        QVERIFY(m.nearMatch);
        // An exact entry always beats a prefix.
        m = t.find("ams2", "MINI Cooper S 1965");
        QCOMPARE(m.key, std::string("ams2:MINI Cooper S 1965"));
        QVERIFY(!m.nearMatch);
        m = t.find("AMS2", "mini cooper s");
        QCOMPARE(m.key, std::string("ams2:MINI Cooper S"));
        QVERIFY(!m.nearMatch);
        // One word is not a car: a make alone never matches.
        QVERIFY(t.find("ams2", "Caterham Seven 620R").source.empty());
        // Whole words only, never part of one.
        QVERIFY(t.find("ams2", "Lotus 98Turbo Special").source.empty());
        // Another game's entries never match, however close.
        QVERIFY(t.find("AssettoCorsa", "MINI Cooper S 1965 B").source.empty());
        QVERIFY(t.find("rFactor2", "MINI Cooper S 1965 B").source.empty());
        // A content folder id breaks into words at its underscores; a
        // display name matches the same way.
        m = t.find("AssettoCorsa", "ks_mazda_787b_le_mans");
        QCOMPARE(m.key, std::string("ac:ks_mazda_787b"));
        QVERIFY(m.nearMatch);
        m = t.find("AC", "Mazda 787B Le Mans");
        QCOMPARE(m.key, std::string("ac:ks_mazda_787b"));
        QVERIFY(m.nearMatch);
        // A variant suffix on top of the near match still lands.
        m = t.find("ams2", "MINI Cooper S 1965 B - Wet");
        QCOMPARE(m.key, std::string("ams2:MINI Cooper S 1965"));
        QVERIFY(m.nearMatch);
        // Exact only, when asked: no near match.
        QVERIFY(t.find("ams2", "MINI Cooper S 1965 B", false).source.empty());
        // Your own entry for the car itself is exact and wins.
        QJsonObject mine; mine["name"] = "MINI Cooper S 1965 B"; mine["notes"] = "mine";
        t.putUser(t.keyFor("Automobilista2", "MINI Cooper S 1965 B"), mine);
        m = t.find("AMS2", "MINI Cooper S 1965 B");
        QCOMPARE(m.source, std::string("user"));
        QVERIFY(!m.nearMatch);
        // ...and the base car is still the stock base car.
        QCOMPARE(t.find("AMS2", "MINI Cooper S 1965").source, std::string("stock"));
    }

    void defaults_carKeysAtFreshValues()
    {
        const AppConfig fresh;
        const QJsonObject freshH = Config::writeHapticsObject(fresh);
        const QJsonObject d = CarTable::defaultHapticsObject();
        // Exactly the five car tiles, each with exactly its car-owned keys,
        // at a fresh install's values; never a route.
        QCOMPARE(d.size(), 5);
        for (auto it = d.begin(); it != d.end(); ++it)
        {
            const std::vector<std::string> keys = CarTable::carKeys(it.key().toStdString());
            QVERIFY2(!keys.empty(), qPrintable(it.key()));
            const QJsonObject o = it.value().toObject();
            QCOMPARE(o.size(), static_cast<int>(keys.size()));
            for (const std::string& k : keys)
            {
                const QString qk = QString::fromStdString(k);
                QVERIFY2(o.contains(qk), qPrintable(it.key() + "." + qk));
                QVERIFY2(o.value(qk) == freshH.value(it.key()).toObject().value(qk), qPrintable(it.key() + "." + qk));
            }
        }

        // Over a rig carrying another car (the 787B, a front-drive spool
        // left from before): every car-owned key goes back to the fresh
        // install's, every other key, route, the master and the switch stay.
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/cars.json", stockFile());
        CarTable t;
        QVERIFY(t.loadStock((dir.path() + "/cars.json").toStdString()));
        AppConfig c = rigWithRoutes();
        Config::readHapticsObject(CarTable::toHapticsObject(t.find("AssettoCorsa", "ks_mazda_787b").entry), c);
        c.hapticsDriveline.driven = 1.0; c.hapticsDriveline.diffType = 1.0; c.hapticsDriveline.whinePitch = 2.0;
        c.hapticsTc.sharp = 0.9;
        const QJsonObject before = Config::writeHapticsObject(c);
        Config::readHapticsObject(d, c);
        const QJsonObject after = Config::writeHapticsObject(c);
        QCOMPARE(after.keys(), before.keys());
        for (auto it = after.begin(); it != after.end(); ++it)
        {
            if (!it.value().isObject())
            {
                QVERIFY2(it.value() == before.value(it.key()), qPrintable(it.key()));
                continue;
            }
            const std::vector<std::string> car = CarTable::carKeys(it.key().toStdString());
            const QJsonObject o = it.value().toObject(), was = before.value(it.key()).toObject();
            for (auto kt = o.begin(); kt != o.end(); ++kt)
            {
                const bool owned = std::find(car.begin(), car.end(), kt.key().toStdString()) != car.end();
                const QJsonValue want = owned ? freshH.value(it.key()).toObject().value(kt.key()) : was.value(kt.key());
                QVERIFY2(kt.value() == want, qPrintable(it.key() + "." + kt.key()));
            }
        }
        QCOMPARE(c.hapticsEngine.cylinders, fresh.hapticsEngine.cylinders);
        QCOMPARE(c.hapticsDriveline.driven, fresh.hapticsDriveline.driven);
        QCOMPARE(c.hapticsDriveline.whinePitch, 2.0);   // the rig's, not the car's
        QCOMPARE(c.hapticsTc.sharp, 0.9);
        QCOMPARE(c.hapticsFx[static_cast<size_t>(haptics::Effect::Engine)].routes[0].gain, 0.7);
        QCOMPARE(c.hapticsMasterGain, 1.3);
        QCOMPARE(c.hapticsFollowCar, false);
    }

    void userLayer_winsAndForgets()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/cars.json", stockFile());
        const std::string anchor = (dir.path() + "/config.json").toStdString();
        CarTable t;
        QVERIFY(t.loadStock((dir.path() + "/cars.json").toStdString()));
        QVERIFY(!t.loadUser(anchor));   // no file yet is not an error worth more than false

        QJsonObject mine; mine["name"] = "Mazda 787B"; mine["notes"] = "softer";
        QJsonObject eng; eng["ampPct"] = 25; mine["engine"] = eng;
        t.putUser(t.keyFor("AssettoCorsa", "ks_mazda_787b"), mine);
        QCOMPARE(t.userCount(), 1);
        CarTable::Match m = t.find("AssettoCorsa", "KS_MAZDA_787B");
        QCOMPARE(m.source, std::string("user"));
        QCOMPARE(m.notes, std::string("softer"));
        QCOMPARE(m.entry.value("engine").toObject().value("ampPct").toDouble(), 25.0);
        // ...and by name, the user layer still wins over the stock name.
        QCOMPARE(t.find("AssettoCorsa", "Mazda 787B").source, std::string("user"));
        // An entry saved under a sender's display name (no id known) is
        // found by that name and by the stock key's name alike.
        QJsonObject byName; byName["name"] = "Lotus 98T"; byName["notes"] = "mine";
        t.putUser(t.keyFor("Automobilista2", "Lotus 98T"), byName);
        QCOMPARE(t.find("AMS2", "lotus 98t").source, std::string("user"));
        QVERIFY(t.removeUser("ams2:Lotus 98T"));

        // Both layers show in the list (game, then name, then the layer:
        // stock before user for the same car).
        const auto items = t.list();
        QCOMPARE(items.size(), size_t(3));
        QCOMPARE(items[0].source, std::string("stock"));
        QCOMPARE(items[1].source, std::string("user"));
        QCOMPARE(items[1].key, std::string("ac:ks_mazda_787b"));

        // Persisted, reloaded, still wins; forgotten, stock is back.
        QVERIFY(t.saveUser(anchor));
        CarTable t2;
        QVERIFY(t2.loadUser(anchor));
        QVERIFY(t2.loadStock((dir.path() + "/cars.json").toStdString()));
        QCOMPARE(t2.find("AssettoCorsa", "ks_mazda_787b").source, std::string("user"));
        QVERIFY(t2.removeUser("ac:ks_mazda_787b"));
        QVERIFY(!t2.removeUser("ac:ks_mazda_787b"));
        QCOMPARE(t2.find("AssettoCorsa", "ks_mazda_787b").source, std::string("stock"));
        QVERIFY(t2.saveUser(anchor));
        CarTable t3;
        QVERIFY(t3.loadUser(anchor));
        QCOMPARE(t3.userCount(), 0);
    }

    void apply_movesFiveTilesOnly()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        writeText(dir.path() + "/cars.json", stockFile());
        CarTable t;
        QVERIFY(t.loadStock((dir.path() + "/cars.json").toStdString()));
        const CarTable::Match m = t.find("AssettoCorsa", "ks_mazda_787b");

        AppConfig c = rigWithRoutes();
        const QJsonObject h = CarTable::toHapticsObject(m.entry);
        QVERIFY(h.contains(haptics::effectInfo(haptics::Effect::Engine).key));
        QVERIFY(!h.contains("masterGain"));
        QVERIFY(!h.contains("followCar"));
        QVERIFY(!h.contains("slipLat"));
        for (const QString& k : h.keys()) QVERIFY2(!h.value(k).toObject().contains("routes"), "an entry must never carry routes");
        Config::readHapticsObject(h, c);

        const auto fx = [&c](haptics::Effect e) -> const haptics::EffectParams& { return c.hapticsFx[static_cast<size_t>(e)]; };
        // The five tiles took the car's values...
        QCOMPARE(fx(haptics::Effect::Engine).ampPct, 45.0);
        QCOMPARE(fx(haptics::Effect::Engine).freqHz, 30.0);
        QCOMPARE(fx(haptics::Effect::Engine).jitter, 0.2);
        QCOMPARE(c.hapticsEngine.cylinders, 4.0);
        QCOMPARE(c.hapticsEngine.layout, 3.0);
        QCOMPARE(c.hapticsEngine.pops, 0.8);
        QCOMPARE(fx(haptics::Effect::Limiter).ampPct, 35.0);
        QCOMPARE(fx(haptics::Effect::Driveline).ampPct, 35.0);
        QCOMPARE(c.hapticsDriveline.gearbox, 1.0);
        QCOMPARE(c.hapticsDriveline.whine, 0.6);
        QCOMPARE(fx(haptics::Effect::Abs).ampPct, 0.0);
        QCOMPARE(fx(haptics::Effect::Tc).freqHz, 15.0);
        // ...with the learned values back to "learn" (they belong to the car too)...
        QCOMPARE(c.hapticsEngine.maxRpm, 0.0);
        QCOMPARE(c.hapticsEngine.order, 0.0);
        // ...and every route, the other tiles, master gain and the switch untouched.
        QCOMPARE(fx(haptics::Effect::Engine).routes[0].axis, 0);
        QCOMPARE(fx(haptics::Effect::Engine).routes[0].gain, 0.7);
        QCOMPARE(fx(haptics::Effect::Limiter).routes[0].gain, 0.3);
        QCOMPARE(fx(haptics::Effect::Driveline).routes[0].gain, 1.1);
        QCOMPARE(fx(haptics::Effect::Skid).ampPct, 60.0);
        QCOMPARE(fx(haptics::Effect::Skid).routes[0].gain, 0.5);
        QCOMPARE(c.hapticsSlipLat.peak, 6.5);
        QCOMPARE(c.hapticsMasterGain, 1.3);
        QCOMPARE(c.hapticsFollowCar, false);

        // A sparse entry (the 98T names a few engine fields) leaves the
        // fields it does not name as they were.
        const CarTable::Match s = t.find("ams2", "Lotus 98T");
        Config::readHapticsObject(CarTable::toHapticsObject(s.entry), c);
        QCOMPARE(c.hapticsEngine.cylinders, 6.0);
        QCOMPARE(c.hapticsEngine.turbo, 1.0);
        QCOMPARE(c.hapticsEngine.pops, 0.8);   // from the 787B apply, not named by the 98T
        QCOMPARE(fx(haptics::Effect::Engine).routes[0].gain, 0.7);
    }

    void saveForThisCar_roundTrip()
    {
        AppConfig c = rigWithRoutes();
        const QJsonObject e = CarTable::fromHapticsObject(Config::writeHapticsObject(c), "Mine", "my notes");
        QCOMPARE(e.value("name").toString(), QString("Mine"));
        QCOMPARE(e.value("notes").toString(), QString("my notes"));
        const QJsonObject eng = e.value("engine").toObject();
        QCOMPARE(eng.value("ampPct").toDouble(), 20.0);
        QCOMPARE(eng.value("cylinders").toDouble(), 8.0);
        QCOMPARE(eng.value("maxRpm").toDouble(), 7200.0);   // set by hand: kept
        QCOMPARE(eng.value("order").toDouble(), 3.0);
        QVERIFY(!eng.contains("routes"));
        QVERIFY(!eng.contains("durMs"));
        QVERIFY(!eng.contains("peakPct"));
        QVERIFY(!e.value("driveline").toObject().contains("routes"));
        QVERIFY(!e.contains("slipLat"));
        QVERIFY(!e.contains("masterGain"));
        for (const char* k : { "engine", "limiter", "driveline", "abs", "tc" })
            QVERIFY2(e.value(k).isObject(), k);

        // Learned values at "learn" are not written.
        c.hapticsEngine.maxRpm = 0; c.hapticsEngine.order = 0;
        const QJsonObject e2 = CarTable::fromHapticsObject(Config::writeHapticsObject(c), "Mine", "");
        QVERIFY(!e2.value("engine").toObject().contains("maxRpm"));
        QVERIFY(!e2.value("engine").toObject().contains("order"));

        // Applied to a fresh rig, the entry reproduces the car-owned values.
        AppConfig d;
        Config::readHapticsObject(CarTable::toHapticsObject(e), d);
        QCOMPARE(d.hapticsFx[static_cast<size_t>(haptics::Effect::Engine)].ampPct, 20.0);
        QCOMPARE(d.hapticsEngine.cylinders, 8.0);
        QCOMPARE(d.hapticsEngine.maxRpm, 7200.0);
        QCOMPARE(d.hapticsFx[static_cast<size_t>(haptics::Effect::Engine)].routes[0].axis, -1);
        QCOMPARE(d.hapticsMasterGain, 1.0);

        // The diff: what the car has and which wheels it drives are the
        // car's, so an entry that does not say resets them (no spool or
        // front drive carried over from the last car); how strongly it plays
        // is the rig's and stays.
        {
            AppConfig r; r.hapticsDriveline.diff = 0.5; r.hapticsDriveline.diffHz = 9.0;
            r.hapticsDriveline.diffType = 1.0; r.hapticsDriveline.driven = 1.0;   // the last car: a front-drive spool
            QJsonObject entry; QJsonObject dl; dl["gearbox"] = 1.0; entry["driveline"] = dl;
            Config::readHapticsObject(CarTable::toHapticsObject(entry), r);
            QCOMPARE(r.hapticsDriveline.diffType, 0.0); QCOMPARE(r.hapticsDriveline.driven, 0.0);
            QCOMPARE(r.hapticsDriveline.diff, 0.5);     QCOMPARE(r.hapticsDriveline.diffHz, 9.0);
        }

        // The keys the tiles tint as "follows the car" are exactly what an
        // entry stores: every stored key is one, ignore neutral and whine
        // pitch (the rig's) are not, and a tile the table never touches has
        // none.
        {
            AppConfig r = rigWithRoutes(); r.hapticsEngine.maxRpm = 7000; r.hapticsEngine.order = 2;
            const QJsonObject all = CarTable::fromHapticsObject(Config::writeHapticsObject(r), "x", "");
            const std::pair<const char*, const char*> tiles[] = {
                { "engine", haptics::effectInfo(haptics::Effect::Engine).key }, { "limiter", "limiter" },
                { "driveline", "driveline" }, { "abs", "abs" }, { "tc", "tc" } };
            for (const auto& [section, tile] : tiles)
            {
                const std::vector<std::string> keys = CarTable::carKeys(tile);
                const QJsonObject o = all.value(section).toObject();
                QCOMPARE(static_cast<int>(keys.size()), o.size());
                for (const std::string& k : keys) QVERIFY2(o.contains(QString::fromStdString(k)), k.c_str());
            }
            const std::vector<std::string> dl = CarTable::carKeys("driveline");
            QVERIFY(std::find(dl.begin(), dl.end(), "whine") != dl.end());
            QVERIFY(std::find(dl.begin(), dl.end(), "ignoreN") == dl.end());
            QVERIFY(std::find(dl.begin(), dl.end(), "whinePitch") == dl.end());
            QVERIFY(CarTable::carKeys("slipLat").empty());
        }
    }

    void followCar_inRigJsonNotInProfiles()
    {
        AppConfig c;
        QCOMPARE(c.hapticsFollowCar, true);
        c.hapticsFollowCar = false;
        const QJsonObject h = Config::writeHapticsObject(c);
        QVERIFY(h.contains("followCar"));
        QCOMPARE(h.value("followCar").toBool(true), false);
        AppConfig d;
        Config::readHapticsObject(h, d);
        QCOMPARE(d.hapticsFollowCar, false);
        // Absent (an older file, a profile): keeps what the rig has.
        QJsonObject h2 = h; h2.remove("followCar");
        AppConfig e; e.hapticsFollowCar = true;
        Config::readHapticsObject(h2, e);
        QCOMPARE(e.hapticsFollowCar, true);
    }

    void shippedTable_parses()
    {
        const QString path = QString(NULLCAT_SOURCE_DIR) + "/resources/cars.json";
        CarTable t;
        QVERIFY2(t.loadStock(path.toStdString()), "resources/cars.json missing or unreadable");
        QVERIFY2(t.stockCount() >= 450, "the shipped table lost cars");
        QCOMPARE(t.find("AssettoCorsa", "ks_mazda_787b").source, std::string("stock"));
        QCOMPARE(t.find("Automobilista2", "Lotus 98T").source, std::string("stock"));
        QCOMPARE(t.find("Automobilista2", "McLaren MP4/4").source, std::string("stock"));
        // SimHub names AC cars by display name: the shipped names are the
        // game's own (ui_car.json), so that path lands too.
        QCOMPARE(t.find("AssettoCorsa", "Mazda 787B").key, std::string("ac:ks_mazda_787b"));
        QCOMPARE(t.find("AssettoCorsa", "Nissan 370z Nismo").key, std::string("ac:ks_nissan_370z"));
        QCOMPARE(t.find("AssettoCorsa", "Ferrari F40").key, std::string("ac:ferrari_f40"));
        // AMS2 keys are the game's own strings (SimHub's car list), with an
        // aero variant folding onto its base car.
        QCOMPARE(t.find("Automobilista2", "BMW M3 Sport Evo Group A").source, std::string("stock"));
        QCOMPARE(t.find("Automobilista2", "Porsche 963 - Low Downforce").key, std::string("ams2:Porsche 963"));
        QCOMPARE(t.find("Automobilista2", "Formula USA 2023 - Speedway").key, std::string("ams2:Formula USA 2023"));
        QCOMPARE(t.find("Automobilista2", "McLaren MP4_4 - Low Downforce").key, std::string("ams2:McLaren MP4/4"));
        QCOMPARE(t.find("Automobilista2", "Fusca 1 Hot Cars").source, std::string("stock"));
        QVERIFY(t.find("Automobilista2", "Some Car - Low Downforce").source.empty());
        // Names the sim gives that the roster first missed, and the Minis
        // driving their front wheels; another suffix after the Mini's year
        // lands on it as a near match.
        {
            const CarTable::Match b = t.find("Automobilista2", "MINI Cooper S 1965 B");
            QCOMPARE(b.key, std::string("ams2:MINI Cooper S 1965 B"));
            QVERIFY(!b.nearMatch);
            for (const char* mini : { "MINI Cooper S 1965 B", "MINI Cooper S 1965", "MINI Cooper S" })
                QCOMPARE(t.find("Automobilista2", mini).entry.value("driveline").toObject().value("driven").toDouble(-1), 1.0);
            const CarTable::Match c = t.find("Automobilista2", "MINI Cooper S 1965 C");
            QCOMPARE(c.key, std::string("ams2:MINI Cooper S 1965"));
            QVERIFY(c.nearMatch);
            QCOMPARE(t.find("Automobilista2", "Porsche 911 GT3 Cup 4.0").key, std::string("ams2:Porsche 911 GT3 Cup 4.0"));
            QVERIFY(!t.find("Automobilista2", "Porsche 911 GT3 Cup 4.0").nearMatch);
        }
        // Every entry has the five sections, notes, and never routes.
        QFile f(path); QVERIFY(f.open(QIODevice::ReadOnly));
        const QJsonObject cars = QJsonDocument::fromJson(f.readAll()).object().value("cars").toObject();
        for (auto it = cars.begin(); it != cars.end(); ++it)
        {
            const QJsonObject e = it.value().toObject();
            for (const char* k : { "engine", "limiter", "driveline", "abs", "tc" })
            {
                QVERIFY2(e.value(k).isObject(), qPrintable(it.key() + ": " + k));
                QVERIFY2(!e.value(k).toObject().contains("routes"), qPrintable(it.key() + ": routes in " + k));
            }
            QVERIFY2(e.value("notes").isString() && !e.value("notes").toString().isEmpty(), qPrintable(it.key() + ": notes"));
            QVERIFY2(it.key().startsWith("ac:") || it.key().startsWith("ams2:"), qPrintable(it.key()));
        }
    }
};

QTEST_MAIN(TestCarTable)
#include "TestCarTable.moc"
