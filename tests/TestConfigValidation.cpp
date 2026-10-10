// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestConfigValidation.cpp  (Build 65 - P4-2)
//
// Unit tests for AppConfig::validate().
// Covers: valid config, drive count bounds, Hz bounds,
// stroke/backoff/speed plausibility, torque pct bounds,
// torque mode pct ranges, multiple errors.
// ============================================================

#include <QtTest>
#include "../src/Config.h"

class TestConfigValidation : public QObject
{
    Q_OBJECT

private:
    // Builds a valid single-drive AppConfig as a baseline
    static AppConfig validConfig()
    {
        AppConfig cfg;
        cfg.controlLoopHz = 1000;
        cfg.numDrives     = 1;

        DriveConfig dc;
        dc.strokeMm         = 100.0;
        dc.homingBackoffMm  = 1.5;
        dc.homingSpeed   = 5.0;
        dc.maxVelocityMmS   = 200.0;
        dc.maxAccelerationMmS2 = 500.0;
        dc.homingTorquePct  = 25;
        dc.mode             = "csp";
        dc.torqueMinPct     = 5.0;
        dc.torqueMaxPct     = 50.0;
        cfg.drives.push_back(dc);

        return cfg;
    }

private slots:
    // Valid config → no errors
    void validConfig_noErrors()
    {
        AppConfig cfg = validConfig();
        auto errors = cfg.validate();
        QVERIFY2(errors.empty(),
            ("Expected no errors, got: " + (errors.empty() ? "" : errors[0])).c_str());
    }

    // Zero drives
    void zeroDrives_error()
    {
        AppConfig cfg = validConfig();
        cfg.drives.clear();
        auto errors = cfg.validate();
        QVERIFY(!errors.empty());
        bool found = false;
        for (const auto& e : errors)
            if (e.find("numDrives") != std::string::npos || e.find("drive") != std::string::npos)
                found = true;
        QVERIFY2(found, "Expected drive count error");
    }

    // controlLoopHz below minimum (100)
    void controlLoopHz_tooLow_error()
    {
        AppConfig cfg = validConfig();
        cfg.controlLoopHz = 50;
        auto errors = cfg.validate();
        QVERIFY(!errors.empty());
        bool found = false;
        for (const auto& e : errors)
            if (e.find("controlLoopHz") != std::string::npos || e.find("Hz") != std::string::npos)
                found = true;
        QVERIFY2(found, "Expected controlLoopHz error");
    }

    // controlLoopHz above maximum (2000)
    void controlLoopHz_tooHigh_error()
    {
        AppConfig cfg = validConfig();
        cfg.controlLoopHz = 3000;
        auto errors = cfg.validate();
        QVERIFY(!errors.empty());
        bool found = false;
        for (const auto& e : errors)
            if (e.find("controlLoopHz") != std::string::npos || e.find("Hz") != std::string::npos)
                found = true;
        QVERIFY2(found, "Expected controlLoopHz error");
    }

    // strokeMm = 0 (physically impossible)
    void strokeMm_zero_error()
    {
        AppConfig cfg = validConfig();
        cfg.drives[0].strokeMm = 0.0;
        auto errors = cfg.validate();
        QVERIFY(!errors.empty());
        bool found = false;
        for (const auto& e : errors)
            if (e.find("stroke") != std::string::npos)
                found = true;
        QVERIFY2(found, "Expected stroke error");
    }

    // backoffMm >= strokeMm (backoff must be less than stroke)
    void backoff_greaterThanStroke_error()
    {
        AppConfig cfg = validConfig();
        cfg.drives[0].homingBackoffMm = 150.0;  // > strokeMm=100
        auto errors = cfg.validate();
        QVERIFY(!errors.empty());
        bool found = false;
        for (const auto& e : errors)
            if (e.find("backoff") != std::string::npos || e.find("stroke") != std::string::npos)
                found = true;
        QVERIFY2(found, "Expected backoff >= stroke error");
    }

    // Rotary lever: a following-error window wider than the arc means the
    // drive-side runaway protection (0x6065) can never trip. Server-side twin
    // of the web editor's arc cap - must be rejected, not silently written.
    void rotaryFerrWindow_overArc_error()
    {
        AppConfig cfg = validConfig();
        cfg.drives[0].axisType = "rotary_lever";
        cfg.drives[0].strokeMm = 40.0;                 // arc, degrees
        cfg.drives[0].followingErrorWindowMm = 100.0;  // linear default: unsafe here
        auto errors = cfg.validate();
        QVERIFY(!errors.empty());
        bool found = false;
        for (const auto& e : errors)
            if (e.find("followingErrorWindow") != std::string::npos
                && e.find("arc") != std::string::npos)
                found = true;
        QVERIFY2(found, "Expected rotary ferr-window arc error");
    }

    // Rotary window within the arc passes; a linear axis keeps the wide
    // default untouched (the cap is rotary geometry, not a general clamp).
    void rotaryFerrWindow_withinArc_ok()
    {
        AppConfig cfg = validConfig();
        cfg.drives[0].axisType = "rotary_lever";
        cfg.drives[0].strokeMm = 40.0;
        cfg.drives[0].followingErrorWindowMm = 20.0;
        QVERIFY2(cfg.validate().empty(), "Rotary window within arc must pass");

        AppConfig lin = validConfig();                 // linear_vertical
        lin.drives[0].followingErrorWindowMm = 100.0;
        QVERIFY2(lin.validate().empty(), "Linear axis keeps the wide default");
    }

    // ---- Control-loading device family (0.9.5) ----
    void deviceAxis_validConfig_passes()
    {
        AppConfig cfg = validConfig();
        DriveConfig& d = cfg.drives[0];
        d.axisType = "shifter";
        d.mode     = "torque";
        d.device.detents     = { -0.05, 0.0, 0.05 };
        d.device.springCurve = { {-0.07, -175.0}, {0.0, 0.0}, {0.07, 175.0} };
        d.device.detentCurve = { {-0.02, 60.0}, {0.0, 0.0}, {0.02, -60.0} };
        auto errors = cfg.validate();
        QVERIFY2(errors.empty(),
            ("Device config must pass, got: " + (errors.empty() ? "" : errors[0])).c_str());
    }

    void deviceAxis_wrongMode_error()
    {
        AppConfig cfg = validConfig();
        cfg.drives[0].axisType = "shifter";
        cfg.drives[0].mode     = "csp";
        bool found = false;
        for (const auto& e : cfg.validate())
            if (e.find("torque") != std::string::npos) found = true;
        QVERIFY2(found, "shifter in csp mode must demand torque mode");
    }

    void deviceAxis_detentOutsideStops_error()
    {
        AppConfig cfg = validConfig();
        cfg.drives[0].axisType = "shifter";
        cfg.drives[0].mode     = "torque";
        cfg.drives[0].device.detents = { 0.5 };   // outside +/-0.07 stops
        bool found = false;
        for (const auto& e : cfg.validate())
            if (e.find("detents") != std::string::npos) found = true;
        QVERIFY2(found, "detent outside the stops must be rejected");
    }

    void deviceAxis_nonMonotonicCurve_error()
    {
        AppConfig cfg = validConfig();
        cfg.drives[0].axisType = "pedal";
        cfg.drives[0].mode     = "torque";
        cfg.drives[0].device.springCurve = { {0.0, 0.0}, {0.05, 50.0}, {0.02, 80.0} };
        bool found = false;
        for (const auto& e : cfg.validate())
            if (e.find("springCurve") != std::string::npos) found = true;
        QVERIFY2(found, "non-monotonic curve x values must be rejected");
    }

    // Deliberate 0.9.5 re-gate: the seven belt guards bind BELT axes only.
    // A non-belt torque axis with out-of-belt-range guard values validates
    // (it never runs the belt guards); before 0.9.5 this errored.
    void ncxBindings_validation()
    {
        AppConfig cfg = validConfig();
        cfg.ncxBindings = { { "rpm", 0, 1.0, 0.0 },
                           { "clutchPct", 3, 100.0, 0.0 } };
        QVERIFY2(cfg.validate().empty(), "well-formed bindings pass");

        cfg.ncxBindings = { { "boostBar", 0, 1.0, 0.0 } };
        bool found = false;
        for (const auto& e : cfg.validate())
            if (e.find("unknown token") != std::string::npos) found = true;
        QVERIFY2(found, "unknown token rejected");

        // Protocol 1.5 widened the wire to 64 slots: 63 is the last valid one.
        cfg.ncxBindings = { { "rpm", 63, 1.0, 0.0 } };
        QVERIFY2(cfg.validate().empty(), "slot 63 accepted (wire carries 0..63)");
        cfg.ncxBindings = { { "rpm", 64, 1.0, 0.0 } };
        found = false;
        for (const auto& e : cfg.validate())
            if (e.find("slot out of range") != std::string::npos) found = true;
        QVERIFY2(found, "slot 64 rejected (wire carries 0..63)");

        // Every 1.3 token is a known name (the registry drives validation).
        cfg.ncxBindings = { { "slipAngleFL", 14, 1.0, 0.0 }, { "suspVelRR", 33, 1.0, 0.0 },
                           { "maxRpm", 13, 1.0, 0.0 }, { "loadRL", 28, 1.0, 0.0 } };
        QVERIFY2(cfg.validate().empty(), "1.3 per-wheel tokens accepted");

        cfg.ncxBindings = { { "rpm", 0, 1.0, 0.0 }, { "rpm", 1, 1.0, 0.0 } };
        found = false;
        for (const auto& e : cfg.validate())
            if (e.find("bound twice") != std::string::npos) found = true;
        QVERIFY2(found, "duplicate token rejected");

        cfg.ncxBindings = { { "rpm", 0, 0.0, 0.0 } };
        found = false;
        for (const auto& e : cfg.validate())
            if (e.find("scale must be nonzero") != std::string::npos) found = true;
        QVERIFY2(found, "zero scale rejected");
    }

    void beltGuardRanges_bindBeltsOnly()
    {
        AppConfig cfg = validConfig();
        cfg.drives[0].axisType         = "shifter";
        cfg.drives[0].mode             = "torque";
        cfg.drives[0].beltOverspeedRpm = 10.0;    // far below the belt floor of 50
        QVERIFY2(cfg.validate().empty(),
                 "belt guard ranges must not bind a non-belt torque axis");

        AppConfig belt = validConfig();
        belt.drives[0].axisType         = "belt";
        belt.drives[0].mode             = "torque";
        belt.drives[0].beltOverspeedRpm = 10.0;
        bool found = false;
        for (const auto& e : belt.validate())
            if (e.find("beltOverspeedRpm") != std::string::npos) found = true;
        QVERIFY2(found, "belt axes keep the belt guard ranges unchanged");
    }

    // homingTorquePct = 0 (must be >= 1)
    void homingTorquePct_zero_error()
    {
        AppConfig cfg = validConfig();
        cfg.drives[0].homingTorquePct = 0;
        auto errors = cfg.validate();
        QVERIFY(!errors.empty());
        bool found = false;
        for (const auto& e : errors)
            if (e.find("torquePct") != std::string::npos || e.find("homing") != std::string::npos)
                found = true;
        QVERIFY2(found, "Expected homingTorquePct error");
    }

    // Multiple errors accumulate
    void multipleErrors_allReported()
    {
        AppConfig cfg = validConfig();
        cfg.controlLoopHz = 0;             // bad Hz
        cfg.drives[0].strokeMm = -1.0;     // bad stroke
        cfg.drives[0].homingBackoffMm = 999.0; // bad backoff
        auto errors = cfg.validate();
        QVERIFY2(errors.size() >= 2, "Expected at least 2 errors");
    }

    // Route gain unit follows the destination: a position axis takes mm
    // up to ITS hapticsMaxMm (the route editor offers the same cap; the
    // old flat [0, 2] refused what the editor allowed), a torque axis or a
    // shaker takes a 0..2 multiplier.
    void routeGain_perDestination()
    {
        AppConfig cfg = validConfig();
        cfg.drives[0].name = "FR"; cfg.drives[0].hapticsMaxMm = 3.0;
        DriveConfig belt = cfg.drives[0]; belt.name = "Belt"; belt.mode = "torque"; belt.axisType = "belt";
        cfg.drives.push_back(belt); cfg.numDrives = 2;
        haptics::EffectParams& p = cfg.hapticsFx[static_cast<size_t>(haptics::Effect::Kerb)];
        p.ampPct = 50.0;
        auto hasErr = [&](const char* needle) {
            for (const auto& e : cfg.validate()) if (e.find(needle) != std::string::npos) return true;
            return false; };

        p.routes[0] = { 0, 2.5 };                      // 2.5 mm on a 3 mm axis
        QVERIFY2(cfg.validate().empty(), "2.5 mm on a position axis with hapticsMaxMm 3 passes");
        p.routes[0] = { 0, 3.5 };
        QVERIFY2(hasErr("haptic max"), "3.5 mm on a 3 mm axis is refused, naming the axis cap");
        p.routes[0] = { 1, 2.5 };                      // x on the belt
        QVERIFY2(hasErr("[0, 2] x"), "2.5 x on a torque axis is refused");
        p.routes[0] = { 1, 2.0 };
        QVERIFY2(cfg.validate().empty(), "2.0 x on a torque axis passes");
        p.routes[0] = haptics::Route{}; p.routes[0].shaker = 0; p.routes[0].gain = 2.5;
        QVERIFY2(hasErr("[0, 2]"), "2.5 x on a shaker is refused");
        p.routes[0] = { 0, -0.5 };
        QVERIFY2(hasErr(">= 0"), "a negative gain is refused");
    }

    // peak % (the channel value that counts as full) is 1..400: past 100
    // for the combined-slip path, where a car can read beyond the wire's
    // "let go" convention.
    void peakPct_range()
    {
        AppConfig cfg = validConfig();
        haptics::EffectParams& p = cfg.hapticsFx[static_cast<size_t>(haptics::Effect::Kerb)];
        QCOMPARE(p.peakPct, 100.0);
        const auto refused = [&cfg]() {
            for (const auto& e : cfg.validate()) if (e.find("peakPct") != std::string::npos) return true;
            return false; };
        p.peakPct = 0.0;   QVERIFY2(refused(), "peakPct 0 is refused");
        p.peakPct = 401.0; QVERIFY2(refused(), "peakPct 401 is refused");
        p.peakPct = 25.0;  QVERIFY(cfg.validate().empty());
        p.peakPct = 250.0; QVERIFY2(cfg.validate().empty(), "peakPct 250 is accepted (combined slip past the convention)");
    }

    // The Road surface carrier has its own range.
    void roadSurfaceHz_range()
    {
        AppConfig cfg = validConfig();
        QCOMPARE(cfg.hapticsRoad.surfaceHz, 12.0);
        cfg.hapticsRoad.surfaceHz = 3.0;
        bool found = false;
        for (const auto& e : cfg.validate()) if (e.find("surfaceHz") != std::string::npos) found = true;
        QVERIFY2(found, "surface hz below 4 is refused");
        cfg.hapticsRoad.surfaceHz = 20.0;
        QVERIFY(cfg.validate().empty());
    }

    // The Road model is one of three; the tyre model's settings and the
    // Kerb's have their ranges.
    void roadModelAndKerb_ranges()
    {
        AppConfig cfg = validConfig();
        QCOMPARE(cfg.hapticsRoad.model, 0.0);   // suspension by default
        QCOMPARE(cfg.hapticsRoad.surface, 0.0); // the old grain off: the roughness moves with the car
        const auto refused = [&cfg](const char* key) {
            for (const auto& e : cfg.validate()) if (e.find(key) != std::string::npos) return true;
            return false; };
        cfg.hapticsRoad.model = 3.0;  QVERIFY2(refused("model"), "road model 3 is refused");
        cfg.hapticsRoad.model = 1.5;  QVERIFY2(refused("model"), "road model 1.5 is refused");
        cfg.hapticsRoad.model = 2.0;  QVERIFY(cfg.validate().empty());
        cfg.hapticsRoad.hopHz = 40.0; QVERIFY2(refused("hopHz"), "hop hz 40 is refused");
        cfg.hapticsRoad.hopHz = 18.0; cfg.hapticsRoad.rough = -1.0; QVERIFY2(refused("rough"), "negative roughness is refused");
        cfg.hapticsRoad.rough = 3.0;  QVERIFY(cfg.validate().empty());
        cfg.hapticsKerb.pitchCm = 2.0;  QVERIFY2(refused("pitchCm"), "a 2 cm rib pitch is refused");
        cfg.hapticsKerb.pitchCm = 30.0; cfg.hapticsKerb.detectMm = 150.0; QVERIFY2(refused("detectMm"), "detect mm 150 is refused");
        cfg.hapticsKerb.detectMm = 15.0; QVERIFY(cfg.validate().empty());
    }

    // The Lateral slip tile's way in: onset %, ease, smooth hz, attack ms.
    void slipLatWayIn_ranges()
    {
        AppConfig cfg = validConfig();
        QCOMPARE(cfg.hapticsSlipLat.onsetPct, 40.0);
        QCOMPARE(cfg.hapticsSlipLat.attackMs, 30.0);
        QCOMPARE(cfg.hapticsSlipLon.attackMs, 8.0);   // Longitudinal keeps the fast attack
        const auto refused = [&cfg](const char* key) {
            for (const auto& e : cfg.validate()) if (e.find(key) != std::string::npos) return true;
            return false; };
        cfg.hapticsSlipLat.onsetPct = 95.0; QVERIFY2(refused("onsetPct"), "onset 95% is refused");
        cfg.hapticsSlipLat.onsetPct = 30.0; cfg.hapticsSlipLat.ease = 3.0; QVERIFY2(refused("ease"), "ease 3 is refused");
        cfg.hapticsSlipLat.ease = 0.5; cfg.hapticsSlipLat.attackMs = 1.0; QVERIFY2(refused("attackMs"), "attack 1 ms is refused");
        cfg.hapticsSlipLat.attackMs = 50.0; cfg.hapticsSlipLat.smoothHz = 40.0; QVERIFY2(refused("smoothHz"), "smooth 40 hz is refused");
        cfg.hapticsSlipLat.smoothHz = 0.0; QVERIFY(cfg.validate().empty());
    }

    // ABS and TC: sharp, spread (both), slow, buzz, buzz hz (ABS).
    void pulse_ranges()
    {
        AppConfig cfg = validConfig();
        QCOMPARE(cfg.hapticsAbs.sharp, 0.3); QCOMPARE(cfg.hapticsAbs.buzz, 0.3);
        QCOMPARE(cfg.hapticsTc.slow, 0.0);   QCOMPARE(cfg.hapticsTc.buzz, 0.0);
        const auto refused = [&cfg](const char* key) {
            for (const auto& e : cfg.validate()) if (e.find(key) != std::string::npos) return true;
            return false; };
        cfg.hapticsAbs.sharp = 1.5;  QVERIFY2(refused("sharp"), "abs sharp 1.5 is refused");
        cfg.hapticsAbs.sharp = 0.5; cfg.hapticsAbs.buzzHz = 10.0; QVERIFY2(refused("buzzHz"), "buzz hz 10 is refused");
        cfg.hapticsAbs.buzzHz = 40.0; cfg.hapticsTc.spread = -0.1; QVERIFY2(refused("spread"), "tc spread -0.1 is refused");
        cfg.hapticsTc.spread = 0.5; QVERIFY(cfg.validate().empty());
    }

    // The frozen-stream guard is off by default and only takes a usable window.
    void ncxFrozenMs_range()
    {
        AppConfig cfg = validConfig();
        QCOMPARE(cfg.ncxFrozenMs, 0);
        QVERIFY(cfg.validate().empty());
        cfg.ncxFrozenMs = 100;
        bool found = false;
        for (const auto& e : cfg.validate()) if (e.find("ncxFrozenMs") != std::string::npos) found = true;
        QVERIFY2(found, "a 100 ms window is refused");
        cfg.ncxFrozenMs = 2000;
        QVERIFY(cfg.validate().empty());
    }
};

QTEST_APPLESS_MAIN(TestConfigValidation)
#include "TestConfigValidation.moc"
