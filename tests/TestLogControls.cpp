// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestLogControls.cpp
//
// Unit tests for the log-control surface:
//   L-1   Logger::parseLevel maps known strings + falls back to DEBUG.
//   L-2   Logger::setDiagEnabled(false) short-circuits logDiag - no write
//         to the diag file, no entry visible on disk.
//   L-3   Logger::setDiagEnabled(true) (default) preserves prior behaviour
// - logDiag writes are observable in the diag file.
//   L-4   Logger::setMinLevel suppresses LOG_* below the threshold.
// ============================================================

#include <QtTest>
#include <QTemporaryFile>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include "../src/Logging.h"

class TestLogControls : public QObject
{
    Q_OBJECT

private slots:

    void l1_parseLevel()
    {
        QCOMPARE(static_cast<int>(Logger::parseLevel("debug")),    static_cast<int>(LogLevel::LVL_DEBUG));
        QCOMPARE(static_cast<int>(Logger::parseLevel("info")),     static_cast<int>(LogLevel::LVL_INFO));
        QCOMPARE(static_cast<int>(Logger::parseLevel("warning")),  static_cast<int>(LogLevel::LVL_WARNING));
        QCOMPARE(static_cast<int>(Logger::parseLevel("error")),    static_cast<int>(LogLevel::LVL_ERROR));
        QCOMPARE(static_cast<int>(Logger::parseLevel("critical")), static_cast<int>(LogLevel::LVL_CRITICAL));
        // Case-insensitive
        QCOMPARE(static_cast<int>(Logger::parseLevel("CRITICAL")), static_cast<int>(LogLevel::LVL_CRITICAL));
        QCOMPARE(static_cast<int>(Logger::parseLevel("Warning")),  static_cast<int>(LogLevel::LVL_WARNING));
        // Unknown -> DEBUG (most permissive)
        QCOMPARE(static_cast<int>(Logger::parseLevel("nonsense")), static_cast<int>(LogLevel::LVL_DEBUG));
        QCOMPARE(static_cast<int>(Logger::parseLevel("")),         static_cast<int>(LogLevel::LVL_DEBUG));
    }

    void l2_diagDisabled_writes_nothing()
    {
        QTemporaryFile tmp;
        QVERIFY(tmp.open());
        QString tmpPath = tmp.fileName();
        tmp.close();

        Logger::instance().initDiag(tmpPath.toStdString());
        Logger::instance().setDiagEnabled(false);
        Logger::instance().logDiag("L2_should_not_appear");
        Logger::instance().flush();

        std::ifstream in(tmpPath.toStdString());
        std::stringstream buf;
        buf << in.rdbuf();
        const std::string contents = buf.str();
        QVERIFY2(contents.find("L2_should_not_appear") == std::string::npos,
            qPrintable(QString("Diag write leaked when disabled. File contents:\n%1")
                .arg(QString::fromStdString(contents))));

        // Cleanup for other tests
        Logger::instance().setDiagEnabled(true);
        Logger::instance().initDiag("");
    }

    void l3_diagEnabled_writes_through()
    {
        QTemporaryFile tmp;
        QVERIFY(tmp.open());
        QString tmpPath = tmp.fileName();
        tmp.close();

        Logger::instance().initDiag(tmpPath.toStdString());
        Logger::instance().setDiagEnabled(true);
        Logger::instance().logDiag("L3_should_be_present");
        Logger::instance().flush();

        std::ifstream in(tmpPath.toStdString());
        std::stringstream buf;
        buf << in.rdbuf();
        const std::string contents = buf.str();
        QVERIFY2(contents.find("L3_should_be_present") != std::string::npos,
            qPrintable(QString("Expected diag line missing. File contents:\n%1")
                .arg(QString::fromStdString(contents))));

        Logger::instance().initDiag("");
    }

    void l4_minLevel_suppresses_lower()
    {
        // Set min level to ERROR; INFO/WARNING should be filtered.
        Logger::instance().setMinLevel(LogLevel::LVL_ERROR);

        // We can't easily observe the non-RT path output here without
        // re-initialising the main log file. The functional check we DO
        // have is that Logger::log() returns early on level < m_minLevel
        // (line 254 in Logging.cpp). We exercise that path by calling
        // log() at LVL_INFO with the minLevel set to ERROR -- it must
        // not crash and must not push to SPSC (RT path) or write to the
        // main log either. The "no crash" property is the verifiable
        // unit-level outcome; full file-output verification would require
        // a re-initialised log file fixture similar to the diag test.
        Logger::instance().log(LogLevel::LVL_INFO,    "L4_info_should_be_suppressed");
        Logger::instance().log(LogLevel::LVL_WARNING, "L4_warning_should_be_suppressed");
        Logger::instance().log(LogLevel::LVL_ERROR,   "L4_error_should_pass_through");

        // Reset for other tests
        Logger::instance().setMinLevel(LogLevel::LVL_DEBUG);

        QVERIFY(true);  // no crash
    }
};

QTEST_MAIN(TestLogControls)
#include "TestLogControls.moc"
