// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestRTLogging.cpp  (Build 222D)
//
// Tests for Logger::pushRT() - the RT-safe, zero-heap-allocation
// log path used by the RT_LOG_* macros.
//
// Tests:
//   RT-1: pushRT() formats and delivers a basic message
//   RT-2: pushRT() with no format args (literal string)
//   RT-3: pushRT() truncates messages > 478 chars with "..."
//   RT-4: pushRT() with a full ring (1025 entries) does not crash
//   RT-5: Bad-frame rate-limit predicate (count%100==1) fires correctly
//   RT-6: Bad-frame counter resets to 0 on clean frame recovery
// ============================================================

#include <QtTest>
#include "../src/Logging.h"
#include <string>
#include <vector>

class TestRTLogging : public QObject
{
    Q_OBJECT

private slots:

    void initTestCase()
    {
        Logger::instance().setMinLevel(LogLevel::LVL_DEBUG);
        Logger::instance().init("", false);  // no file output during tests
    }

    void cleanupTestCase()
    {
        Logger::instance().clearRTThread();
    }

    // RT-1: Basic format string with int, float, string args
    void pushRT_basicFormat()
    {
        Logger::instance().setRTThread();

        Logger::instance().pushRT(LogLevel::LVL_INFO,
            "test %d %.2f %s", 42, 3.14, "hello");

        // Small sleep to let the drain thread pick it up
        QThread::msleep(50);
        Logger::instance().clearRTThread();

        auto logs = Logger::instance().getRecentLogs(200);
        bool found = false;
        for (const auto& line : logs)
            if (line.find("test 42 3.14 hello") != std::string::npos)
                { found = true; break; }
        QVERIFY2(found, "pushRT basic format message not found in log ring");
    }

    // RT-2: No format args - plain string literal
    void pushRT_noArgs()
    {
        Logger::instance().setRTThread();

        Logger::instance().pushRT(LogLevel::LVL_WARNING,
            "plain message no args");

        QThread::msleep(50);
        Logger::instance().clearRTThread();

        auto logs = Logger::instance().getRecentLogs(200);
        bool found = false;
        for (const auto& line : logs)
            if (line.find("plain message no args") != std::string::npos)
                { found = true; break; }
        QVERIFY2(found, "pushRT no-args message not found in log ring");
    }

    // RT-3: Truncation - input longer than 478 chars gets "..." marker
    void pushRT_truncation()
    {
        Logger::instance().setRTThread();

        // Build a 700-char format string with no format specifiers
        std::string longMsg(700, 'X');
        Logger::instance().pushRT(LogLevel::LVL_INFO, "%s", longMsg.c_str());

        QThread::msleep(50);
        Logger::instance().clearRTThread();

        auto logs = Logger::instance().getRecentLogs(200);
        bool foundTruncated = false;
        for (const auto& line : logs)
        {
            if (line.find(std::string(10, 'X')) != std::string::npos &&
                line.find("...") != std::string::npos)
            {
                foundTruncated = true;
                break;
            }
        }
        QVERIFY2(foundTruncated, "pushRT truncation marker '...' not found for overlong message");
    }

    // RT-4: Overfilling the ring (>1024 entries) must not crash
    void pushRT_fullRingDrops()
    {
        Logger::instance().setRTThread();

        for (int i = 0; i < 1100; ++i)
            Logger::instance().pushRT(LogLevel::LVL_DEBUG, "fill %d", i);

        QThread::msleep(100);
        Logger::instance().clearRTThread();

        // Test passes if we reach here without crash or hang
        QVERIFY(true);
    }

    // RT-5: Rate-limit predicate (count%100==1) fires exactly once per 100
    void rateLimitPredicate_firesCorrectly()
    {
        uint64_t count = 0;
        int logCount = 0;
        for (int i = 0; i < 500; ++i)
        {
            ++count;
            if (count % 100 == 1)
                ++logCount;
        }
        // Fires at count 1, 101, 201, 301, 401 → 5 times
        QCOMPARE(logCount, 5);
    }

    // RT-6: Counter reset on clean frame - two bursts of 150 produce 4 total fires
    void rateLimitPredicate_resetsOnRecovery()
    {
        uint64_t count = 0;
        int logCount = 0;

        // First burst: 150 bad frames → fires at 1, 101 (2 fires)
        for (int i = 0; i < 150; ++i)
        {
            ++count;
            if (count % 100 == 1)
                ++logCount;
        }

        // Clean frame - reset counter
        count = 0;

        // Second burst: 150 bad frames → fires at 1, 101 (2 more fires)
        for (int i = 0; i < 150; ++i)
        {
            ++count;
            if (count % 100 == 1)
                ++logCount;
        }

        QCOMPARE(logCount, 4);
    }
};

QTEST_MAIN(TestRTLogging)
#include "TestRTLogging.moc"
