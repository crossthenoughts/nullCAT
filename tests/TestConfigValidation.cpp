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
};

QTEST_APPLESS_MAIN(TestConfigValidation)
#include "TestConfigValidation.moc"
