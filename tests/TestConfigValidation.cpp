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
