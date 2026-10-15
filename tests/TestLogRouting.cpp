// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestLogRouting - which thread's RT_LOG_* lines go where.
//
// The RT log ring is single-producer (SpscQueue.h): the control loop's
// thread. RT_LOG_* is also reached from the haptics idle clock (the
// telemetry receive path and the staged haptics apply while the loop is
// stopped) and from the web and status threads (the channel stream's
// started/stopped lines in getLatestData). Those callers must take the
// ordinary synchronous path, and the ring must be drained whenever the
// logger is up, not only while the loop runs: a shaker-only session never
// starts the loop, and its lines sat in the ring for ever.
//
//   ctest -R LogRouting
// ============================================================
#include "Logging.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

static int g_fail = 0, g_pass = 0;
static void check(bool ok, const char* name)
{
    if (ok) { ++g_pass; std::printf("[PASS] %s\n", name); }
    else    { ++g_fail; std::printf("[FAIL] %s\n", name); }
}
static bool logged(const char* needle)
{
    for (const std::string& line : Logger::instance().getRecentLogs(500))
        if (line.find(needle) != std::string::npos) return true;
    return false;
}
static int loggedCount(const char* prefix)
{
    int n = 0;
    for (const std::string& line : Logger::instance().getRecentLogs(500))
        if (line.find(prefix) != std::string::npos) ++n;
    return n;
}

int main()
{
    Logger& lg = Logger::instance();
    lg.setMinLevel(LogLevel::LVL_DEBUG);
    lg.init("", false);   // no file, no console: the ring buffer is what we read

    // 1. The drain thread is up from init, not from the loop's first start.
    //    (Logger::setRTThread used to start it: a session that never starts
    //    the control loop never drained the ring.)
    check(lg.drainRunning(), "the ring's drain thread runs from Logger::init");

    // 2. No RT thread armed (the loop stopped, or never started): an
    //    RT_LOG_* line from any thread lands at once, synchronously.
    //    Before: pushed into the ring, nobody draining -> never seen.
    RT_LOG_INFO("route-A idle clock line %d", 1);
    check(logged("route-A idle clock line 1"), "RT_LOG_* with no RT thread armed lands synchronously (the idle clock's lines)");

    // 3. The control loop armed on another thread: its own lines ride the
    //    ring; a line from this (web/status/idle) thread still lands at
    //    once, through the ordinary path, never into the ring it does not
    //    own. The check reads the log right after the push, before any
    //    drain cycle (10 ms) could have run.
    {
        std::atomic<int> stage{0};
        std::thread rt([&]
        {
            lg.setRTThread();
            RT_LOG_INFO("route-B ring line from the RT thread");
            stage.store(1);
            while (stage.load() < 2) std::this_thread::yield();
            RT_LOG_INFO("route-D ring line after the web line");
            lg.clearRTThread();
            stage.store(3);
        });
        while (stage.load() < 1) std::this_thread::yield();
        RT_LOG_WARNING("route-C web thread line while the loop runs");
        const bool atOnce = logged("route-C web thread line while the loop runs");
        stage.store(2);
        while (stage.load() < 3) std::this_thread::yield();
        rt.join();
        check(atOnce, "RT_LOG_* from a non-RT thread while the loop runs lands synchronously (not via the ring)");
        check(logged("route-B ring line from the RT thread") && logged("route-D ring line after the web line"),
              "the RT thread's own lines still ride the ring and are drained");
    }

    // 4. clearRTThread leaves nothing behind: every line the RT thread
    //    pushed is in the log by the time it returns (the loop's stop
    //    sequence logs through the ordinary path right after, in order).
    {
        std::thread rt([&]
        {
            lg.setRTThread();
            for (int i = 0; i < 200; ++i) RT_LOG_DEBUG("route-E flush %03d", i);
            lg.clearRTThread();
        });
        rt.join();
        check(loggedCount("route-E flush ") == 200, "clearRTThread returns with the RT thread's lines all drained");
        check(lg.drainRunning(), "...and the drain thread keeps running for the next loop start");
    }

    // 5. The diag ring entries route the same way: an RT_DIAG from a
    //    non-RT thread goes to the diag writer at once (no diag file open
    //    here, so just: it does not crash and does not land in the main
    //    log).
    RT_DIAG("DIAG | route-F | from a non-RT thread");
    check(!logged("route-F"), "RT_DIAG from a non-RT thread never lands in the main log");

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
