// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestHapticsPresets.cpp - the shipped starting points (presets.json)
//
//   - the shipped file loads eight starting points in the file's order (four types,
//     then each for Automobilista 2, only its slide sized up),
//     each with its description
//   - applying one through Config::readHapticsObject changes exactly the
//     settings it lists: every route, master gain, the position budget,
//     the follow switch and every car-owned setting stay as they were
//   - every shipped value is inside its setting's range and the result
//     passes AppConfig::validate
//   - the loader drops what a starting point may not set (an unknown tile
//     or setting, routes, the rig's keys, a car-owned setting, a
//     non-number) and a preset left with nothing; a file with a byte order
//     mark keeps its order; a missing file is none
// ============================================================

#include <QtTest>
#include <QTemporaryDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <algorithm>
#include "../src/HapticsPresets.h"
#include "../src/CarTable.h"
#include "../src/Config.h"
#include "../src/HapticsRegistry.h"

class TestHapticsPresets : public QObject
{
    Q_OBJECT

    static QString shippedPath() { return QString(NULLCAT_SOURCE_DIR) + "/resources/presets.json"; }

    // A rig with routes on several tiles, the rig's own keys away from
    // their defaults and a car's settings on the car tiles.
    static AppConfig tunedRig()
    {
        AppConfig c;
        DriveConfig d; d.mode = "torque"; d.name = "belt";
        c.drives.push_back(d);
        c.numDrives = 1;
        const auto fx = [&c](haptics::Effect e) -> haptics::EffectParams& { return c.hapticsFx[static_cast<size_t>(e)]; };
        for (haptics::Effect e : { haptics::Effect::Skid, haptics::Effect::Lockup, haptics::Effect::Road,
                                   haptics::Effect::Kerb, haptics::Effect::Wheels, haptics::Effect::Impacts,
                                   haptics::Effect::Engine, haptics::Effect::Driveline })
        {
            fx(e).ampPct = 40;
            fx(e).routes[0].axis = 0; fx(e).routes[0].gain = 0.6;
        }
        fx(haptics::Effect::Road).routes[0].part = haptics::Part::Front;
        c.hapticsEngine.cylinders = 8; c.hapticsEngine.litres = 6.2; c.hapticsEngine.turbo = 1;
        c.hapticsDriveline.driven = 2; c.hapticsDriveline.gearbox = 1;
        fx(haptics::Effect::Abs).ampPct = 30; fx(haptics::Effect::Tc).ampPct = 20;
        c.hapticsMasterGain = 1.3;
        c.hapticsPositionBudget = 0.6;
        c.hapticsAxisDelayMs = 12;
        c.hapticsFollowCar = false;
        c.hapticsRoad.model = 1;
        return c;
    }

private slots:
    void shipped_eightInFileOrder()
    {
        HapticsPresets p;
        QVERIFY2(p.load(shippedPath().toStdString()), "resources/presets.json missing or unreadable");
        QCOMPARE(p.count(), 8);
        const auto list = p.list();
        const char* names[] = { "Road car", "Cup car", "GT3 / GT", "Open wheeler",
                                "Road car (AMS2)", "Cup car (AMS2)", "GT3 / GT (AMS2)", "Open wheeler (AMS2)" };
        for (size_t i = 0; i < 8; ++i)
        {
            QCOMPARE(list[i].name, std::string(names[i]));
            QVERIFY2(!list[i].description.empty(), names[i]);
            QVERIFY2(!list[i].haptics.isEmpty(), names[i]);
        }
        QCOMPARE(list[0].description, std::string("Street car on road tyres: soft, long-travel suspension, progressive slides, iron brakes. Tuned on Assetto Corsa."));
        QVERIFY(p.has("Open wheeler"));
        QVERIFY(!p.has("open wheeler"));
        QVERIFY(p.get("nope").name.empty());
    }

    // Each Automobilista 2 starting point is its type's with only the slide
    // sized up (its tyres reach their grip at larger slip angles): the same
    // settings, the same values, peak deg larger.
    void ams2_isItsTypeWithLargerPeakDeg()
    {
        HapticsPresets p;
        QVERIFY(p.load(shippedPath().toStdString()));
        for (const char* type : { "Road car", "Cup car", "GT3 / GT", "Open wheeler" })
        {
            const QJsonObject base = p.get(type).haptics;
            const QJsonObject ams2 = p.get(std::string(type) + " (AMS2)").haptics;
            QVERIFY2(!base.isEmpty() && !ams2.isEmpty(), type);
            QCOMPARE(ams2.keys(), base.keys());
            for (const QString& tile : base.keys())
            {
                const QJsonObject b = base.value(tile).toObject(), a = ams2.value(tile).toObject();
                QCOMPARE(a.keys(), b.keys());
                for (const QString& k : b.keys())
                {
                    if (tile == "slipLat" && k == "peakDeg")
                        QVERIFY2(a.value(k).toDouble() > b.value(k).toDouble(), type);
                    else
                        QVERIFY2(a.value(k).toDouble() == b.value(k).toDouble(), qPrintable(QString(type) + " " + tile + "." + k));
                }
            }
        }
    }

    void openWheeler_changesOnlyItsSettings()
    {
        HapticsPresets p;
        QVERIFY(p.load(shippedPath().toStdString()));
        const HapticsPresets::Preset ow = p.get("Open wheeler");
        QCOMPARE(ow.name, std::string("Open wheeler"));

        AppConfig c = tunedRig();
        const QJsonObject before = Config::writeHapticsObject(c);
        Config::readHapticsObject(ow.haptics, c);
        const QJsonObject after = Config::writeHapticsObject(c);

        // Every key of every tile: the preset's value where it names one,
        // else exactly what was there (routes are compared whole).
        QCOMPARE(after.keys(), before.keys());
        int changed = 0;
        for (auto it = after.begin(); it != after.end(); ++it)
        {
            if (!it.value().isObject())
            {
                QVERIFY2(it.value() == before.value(it.key()), qPrintable(it.key()));
                continue;
            }
            const QJsonObject o = it.value().toObject(), was = before.value(it.key()).toObject();
            const QJsonObject set = ow.haptics.value(it.key()).toObject();
            for (auto kt = o.begin(); kt != o.end(); ++kt)
            {
                const bool named = set.contains(kt.key());
                const QJsonValue want = named ? set.value(kt.key()) : was.value(kt.key());
                const bool same = (kt.value().isDouble() && want.isDouble()) ? kt.value().toDouble() == want.toDouble()
                                                                             : kt.value() == want;
                QVERIFY2(same, qPrintable(it.key() + "." + kt.key()));
                if (named) ++changed;
            }
        }
        QCOMPARE(changed, 17);   // 2 + 3 + 7 + 1 + 2 + 2: every listed setting landed

        // The listed values, by name...
        QCOMPARE(c.hapticsSlipLat.peak, 5.5);
        QCOMPARE(c.hapticsSlipLat.attackMs, 5.0);
        QCOMPARE(c.hapticsSlipLon.peak, 0.4);
        QCOMPARE(c.hapticsSlipLon.screechPct, 2.5);   // edgePct is the longitudinal tile's screech %
        QCOMPARE(c.hapticsSlipLon.bMix, 1.0);         // spin x
        QCOMPARE(c.hapticsRoad.fullMm, 18.0);
        QCOMPARE(c.hapticsRoad.damping, 0.5);
        QCOMPARE(c.hapticsKerb.fullMm, 6.5);
        QCOMPARE(c.hapticsWheels.balance, 0.3);
        QCOMPARE(c.hapticsWheels.judder, 0.1);
        QCOMPARE(c.hapticsFx[static_cast<size_t>(haptics::Effect::Impacts)].freqHz, 55.0);
        QCOMPARE(c.hapticsFx[static_cast<size_t>(haptics::Effect::Impacts)].durMs, 40.0);
        // ...and what it never touches: routes, the rig, the car.
        for (haptics::Effect e : { haptics::Effect::Skid, haptics::Effect::Road, haptics::Effect::Engine })
        {
            QCOMPARE(c.hapticsFx[static_cast<size_t>(e)].routes[0].axis, 0);
            QCOMPARE(c.hapticsFx[static_cast<size_t>(e)].routes[0].gain, 0.6);
            QCOMPARE(c.hapticsFx[static_cast<size_t>(e)].ampPct, 40.0);
        }
        QCOMPARE(c.hapticsFx[static_cast<size_t>(haptics::Effect::Road)].routes[0].part, haptics::Part::Front);
        QCOMPARE(c.hapticsRoad.model, 1.0);
        QCOMPARE(c.hapticsMasterGain, 1.3);
        QCOMPARE(c.hapticsPositionBudget, 0.6);
        QCOMPARE(c.hapticsAxisDelayMs, 12.0);
        QCOMPARE(c.hapticsFollowCar, false);
        QCOMPARE(c.hapticsEngine.cylinders, 8.0);
        QCOMPARE(c.hapticsEngine.litres, 6.2);
        QCOMPARE(c.hapticsEngine.turbo, 1.0);
        QCOMPARE(c.hapticsDriveline.driven, 2.0);
        QCOMPARE(c.hapticsDriveline.gearbox, 1.0);
        QCOMPARE(c.hapticsFx[static_cast<size_t>(haptics::Effect::Abs)].ampPct, 30.0);
        QCOMPARE(c.hapticsFx[static_cast<size_t>(haptics::Effect::Tc)].ampPct, 20.0);
    }

    void shipped_valuesInRangeAndValid()
    {
        HapticsPresets p;
        QVERIFY(p.load(shippedPath().toStdString()));
        for (const HapticsPresets::Preset& pr : p.list())
        {
            // Never a rig key, a route or a car-owned setting.
            QVERIFY(!pr.haptics.contains("masterGain"));
            QVERIFY(!pr.haptics.contains("positionBudget"));
            QVERIFY(!pr.haptics.contains("followCar"));
            for (auto it = pr.haptics.begin(); it != pr.haptics.end(); ++it)
            {
                const haptics::EffectInfo* info = haptics::findEffect(it.key().toStdString().c_str());
                QVERIFY2(info, qPrintable(it.key()));
                const std::vector<std::string> car = CarTable::carKeys(it.key().toStdString());
                const QJsonObject o = it.value().toObject();
                for (auto kt = o.begin(); kt != o.end(); ++kt)
                {
                    const std::string key = kt.key().toStdString();
                    const QString where = QString::fromStdString(pr.name) + ": " + it.key() + "." + kt.key();
                    QVERIFY2(std::find(car.begin(), car.end(), key) == car.end(), qPrintable(where + " belongs to the car"));
                    const haptics::ParamSpec* spec = nullptr;
                    for (int k = 0; k < info->paramCount; ++k)
                        if (key == info->params[k].key) spec = &info->params[k];
                    QVERIFY2(spec, qPrintable(where + " is not a setting of the tile"));
                    const double v = kt.value().toDouble();
                    QVERIFY2(v >= spec->min && v <= spec->max, qPrintable(where + " = " + QString::number(v) + " out of range ["
                                                                          + QString::number(spec->min) + ", " + QString::number(spec->max) + "]"));
                }
            }
            // Applied to a fresh one-axis rig and to one with every effect on
            // (so the checks that only run on a live effect run too), it
            // validates.
            for (const bool allOn : { false, true })
            {
                AppConfig c;
                c.drives.push_back(DriveConfig{});
                c.numDrives = 1;
                if (allOn) for (haptics::EffectParams& fx : c.hapticsFx) fx.ampPct = 50;
                const auto base = c.validate();
                QVERIFY2(base.empty(), qPrintable(QString::fromStdString(base.empty() ? std::string() : base.front())));
                Config::readHapticsObject(pr.haptics, c);
                const auto errs = c.validate();
                QVERIFY2(errs.empty(), qPrintable(QString::fromStdString(pr.name + ": " + (errs.empty() ? std::string() : errs.front()))));
            }
        }
    }

    void loader_dropsWhatAPresetMayNotSet()
    {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        const QString path = dir.path() + "/presets.json";
        {
            QFile f(path); QVERIFY(f.open(QIODevice::WriteOnly));
            f.write(R"({
  "_help": "test",
  "presets": {
    "Zeta": {
      "description": "last by name, first in the file",
      "haptics": {
        "road":      { "rough": 3, "routes": [ { "axis": 0, "gain": 1.0 } ], "nope": 1, "fullMm": "deep" },
        "rpmVibe":   { "cylinders": 12, "ampPct": 80 },
        "driveline": { "driven": 2, "lugHz": 9, "whinePitch": 1 },
        "abs":       { "ampPct": 50, "sharp": 0.7 },
        "masterGain": 2,
        "positionBudget": 1,
        "warpDrive": { "ampPct": 10 }
      }
    },
    "Alpha": {
      "description": "only what it may not set",
      "haptics": { "rpmVibe": { "litres": 4 }, "masterGain": 0.5 }
    },
    "Mid café name": {
      "description": "an escaped name",
      "haptics": { "kerb": { "fullMm": 12 } }
    }
  }
})");
        }
        HapticsPresets p;
        QVERIFY(p.load(path.toStdString()));
        const auto list = p.list();
        QCOMPARE(list.size(), size_t(2));   // Alpha set nothing it may: dropped
        QCOMPARE(list[0].name, std::string("Zeta"));
        QCOMPARE(list[1].name, std::string("Mid caf\xc3\xa9 name"));   // the file's é, decoded

        const QJsonObject h = list[0].haptics;
        QCOMPARE(h.keys(), QStringList({ "abs", "driveline", "road" }));
        QCOMPARE(h.value("road").toObject().keys(), QStringList({ "rough" }));   // no routes, nothing unknown, numbers only
        QCOMPARE(h.value("road").toObject().value("rough").toDouble(), 3.0);
        QCOMPARE(h.value("driveline").toObject().keys(), QStringList({ "whinePitch" }));   // driven and lug hz are the car's
        QCOMPARE(h.value("abs").toObject().keys(), QStringList({ "sharp" }));          // ABS amp is the car's
        QVERIFY(!h.contains("rpmVibe"));   // both engine settings it named are the car's

        // Applied, the routes on the road tile are the rig's own still.
        AppConfig c;
        DriveConfig d; d.mode = "torque"; c.drives.push_back(d); c.numDrives = 1;
        c.hapticsFx[static_cast<size_t>(haptics::Effect::Road)].routes[0].axis = 0;
        c.hapticsFx[static_cast<size_t>(haptics::Effect::Road)].routes[0].gain = 0.25;
        Config::readHapticsObject(h, c);
        QCOMPARE(c.hapticsRoad.rough, 3.0);
        QCOMPARE(c.hapticsFx[static_cast<size_t>(haptics::Effect::Road)].routes[0].gain, 0.25);
        QCOMPARE(c.hapticsFx[static_cast<size_t>(haptics::Effect::Road)].routes[1].axis, -1);
        QCOMPARE(c.hapticsMasterGain, 1.0);

        // Saved with a byte order mark, the file keeps its order.
        {
            QFile f(path); QVERIFY(f.open(QIODevice::WriteOnly));
            f.write("\xEF\xBB\xBF{ \"presets\": { \"Zed\": { \"haptics\": { \"kerb\": { \"fullMm\": 9 } } },"
                    " \"Ann\": { \"haptics\": { \"kerb\": { \"fullMm\": 8 } } } } }");
        }
        QVERIFY(p.load(path.toStdString()));
        QCOMPARE(p.count(), 2);
        QCOMPARE(p.list()[0].name, std::string("Zed"));
        QCOMPARE(p.list()[1].name, std::string("Ann"));

        // A missing file, or one that is not JSON: no starting points.
        HapticsPresets none;
        QVERIFY(!none.load((dir.path() + "/missing.json").toStdString()));
        QCOMPARE(none.count(), 0);
        { QFile f(path); QVERIFY(f.open(QIODevice::WriteOnly)); f.write("{ not json"); }
        QVERIFY(!p.load(path.toStdString()));
        QCOMPARE(p.count(), 0);
    }
};

QTEST_MAIN(TestHapticsPresets)
#include "TestHapticsPresets.moc"
