// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestWkcMonitor.cpp - unit tests for the WKC error/recovery monitor.
//
// Pins the semantics the recovery stack depends on (V1 review S4):
// time-based trigger threshold from configure(), the exact cycle the
// recovery signal fires, the inRecovery latch (one signal per sustained
// loss), clear-on-good-cycle, and reset(). Pure logic - no SOEM / Qt /
// NIC (ctx is opaque and never dereferenced).
// Run with: ctest -R WkcMonitor   (or run the binary directly).
// ============================================================
#include "WkcMonitor.h"
#include <cstdio>

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("  FAIL: %s  (line %d)\n", (msg), __LINE__); } \
} while (0)

// Feed n consecutive mismatched cycles (wkc 2 vs expected 3).
static void feedErrors(WkcMonitor& m, int n, uint64_t& cycle)
{
    for (int i = 0; i < n; ++i)
        m.check(2, 3, nullptr, 2, cycle++);
}

int main()
{
    uint64_t cycle = 0;

    // ---- Good cycles: true, no state accumulates ----
    {
        WkcMonitor m;
        CHECK(m.check(3, 3, nullptr, 2, cycle++), "matching WKC returns true");
        CHECK(m.getConsecutiveErrors() == 0, "good cycle: consecutive stays 0");
        CHECK(m.getTotalErrors() == 0,       "good cycle: total stays 0");
        CHECK(!m.isInRecovery(),             "good cycle: not in recovery");
    }

    // ---- Mismatch: false + counters ----
    {
        WkcMonitor m;
        CHECK(!m.check(2, 3, nullptr, 2, cycle++), "WKC mismatch returns false (skip outputs)");
        CHECK(m.getConsecutiveErrors() == 1, "mismatch increments consecutive");
        CHECK(m.getTotalErrors() == 1,       "mismatch increments total");
        CHECK(!m.isInRecovery(),             "single blip does not signal recovery");
    }

    // ---- Default threshold (configure never called) = 600 cycles ----
    {
        WkcMonitor m;
        feedErrors(m, 599, cycle);
        CHECK(!m.isInRecovery(),              "default: 599 consecutive - no signal yet");
        CHECK(m.getRecoveryAttempts() == 0,   "default: no attempt before threshold");
        feedErrors(m, 1, cycle);
        CHECK(m.isInRecovery(),               "default: signal fires exactly at 600");
        CHECK(m.getRecoveryAttempts() == 1,   "default: exactly one attempt at threshold");
    }

    // ---- configure(): threshold = max(3, hz * 0.3), int-truncated ----
    {
        WkcMonitor m;
        m.configure(2000);                    // 2000 * 0.3 = 600
        feedErrors(m, 599, cycle);
        CHECK(!m.isInRecovery(),              "2kHz: no signal at 599");
        feedErrors(m, 1, cycle);
        CHECK(m.isInRecovery(),               "2kHz: signal at 600 (~300ms)");
    }
    {
        WkcMonitor m;
        m.configure(500);                     // 500 * 0.3 = 150
        feedErrors(m, 149, cycle);
        CHECK(!m.isInRecovery(),              "500Hz: no signal at 149");
        feedErrors(m, 1, cycle);
        CHECK(m.isInRecovery(),               "500Hz: signal at 150 (~300ms)");
    }
    {
        // A 180 ms delivery pause (the longest seen on the rig PC) must not
        // start a recovery scan.
        WkcMonitor m;
        m.configure(500);
        feedErrors(m, 90, cycle);
        CHECK(!m.isInRecovery(),              "500Hz: a 180ms pause (90 cycles) never signals");
        m.check(3, 3, nullptr, 2, cycle++);
        CHECK(m.getRecoveryAttempts() == 0,   "500Hz: no attempt after the pause clears");
    }
    {
        WkcMonitor m;
        m.configure(5);                       // 5 * 0.3 = 1.5 → floor of 3 applies
        feedErrors(m, 2, cycle);
        CHECK(!m.isInRecovery(),              "5Hz: floor - no signal at 2");
        feedErrors(m, 1, cycle);
        CHECK(m.isInRecovery(),               "5Hz: floor - signal at 3");
    }

    // ---- Latch: sustained loss signals ONCE, not per cycle ----
    {
        WkcMonitor m;
        m.configure(2000);
        feedErrors(m, 1000, cycle);
        CHECK(m.getRecoveryAttempts() == 1,   "latch: 1000 consecutive errors = 1 attempt");
        CHECK(m.getConsecutiveErrors() == 1000,"latch: consecutive keeps counting");
        CHECK(m.getTotalErrors() == 1000,     "latch: total keeps counting");
    }

    // ---- Good cycle clears consecutive + latch; total persists ----
    {
        WkcMonitor m;
        m.configure(2000);
        feedErrors(m, 700, cycle);
        CHECK(m.isInRecovery(),               "setup: in recovery after 700");
        CHECK(m.check(3, 3, nullptr, 2, cycle++), "good frame returns true again");
        CHECK(m.getConsecutiveErrors() == 0,  "good frame clears consecutive");
        CHECK(!m.isInRecovery(),              "good frame clears inRecovery");
        CHECK(m.getTotalErrors() == 700,      "total errors persist across recovery");
        CHECK(m.getRecoveryAttempts() == 1,   "attempts persist across recovery");
    }

    // ---- Blip below threshold + good frame: never signals ----
    {
        WkcMonitor m;
        m.configure(2000);
        for (int burst = 0; burst < 10; ++burst)
        {
            feedErrors(m, 599, cycle);                // one under threshold
            m.check(3, 3, nullptr, 2, cycle++);       // recovers
        }
        CHECK(m.getRecoveryAttempts() == 0,   "repeated sub-threshold bursts never signal");
        CHECK(m.getTotalErrors() == 5990,     "sub-threshold bursts still counted in total");
    }

    // ---- Re-trigger: a SECOND sustained loss signals again ----
    {
        WkcMonitor m;
        m.configure(2000);
        feedErrors(m, 600, cycle);
        m.check(3, 3, nullptr, 2, cycle++);           // recover
        feedErrors(m, 600, cycle);
        CHECK(m.getRecoveryAttempts() == 2,   "second sustained loss signals a second attempt");
        CHECK(m.isInRecovery(),               "second loss latches inRecovery again");
    }

    // ---- reset(): everything back to zero ----
    {
        WkcMonitor m;
        m.configure(2000);
        feedErrors(m, 700, cycle);
        m.reset();
        CHECK(m.getConsecutiveErrors() == 0,  "reset: consecutive cleared");
        CHECK(m.getTotalErrors() == 0,        "reset: total cleared");
        CHECK(m.getRecoveryAttempts() == 0,   "reset: attempts cleared");
        CHECK(!m.isInRecovery(),              "reset: inRecovery cleared");
        CHECK(m.getLongestRun() == 0,         "reset: longest run cleared");
        CHECK(m.getWindowRuns() == 0,         "reset: window runs cleared");
        feedErrors(m, 600, cycle);
        CHECK(m.getRecoveryAttempts() == 1,   "reset: threshold behavior intact afterwards");
    }

    // ---- Minute summary accounting: runs, lost cycles, longest; window rolls ----
    {
        WkcMonitor m;
        m.configure(500);
        CHECK(m.summaryCycles() == 30000,     "500Hz: summary window is 60 s of cycles");
        CHECK(m.notableRunCycles() == 50,     "500Hz: a run of 100 ms (50 cycles) is notable");
        uint64_t used = 0;
        auto run = [&](int bad) { feedErrors(m, bad, cycle); m.check(3, 3, nullptr, 2, cycle++); used += bad + 1; };
        run(1); run(3); run(5);
        CHECK(m.getWindowRuns() == 3,         "window: three runs counted");
        CHECK(m.getWindowLost() == 9,         "window: nine cycles lost");
        CHECK(m.getWindowLongest() == 5,      "window: longest run is 5");
        CHECK(m.getLongestRun() == 5,         "session: longest run is 5");
        CHECK(m.getTotalErrors() == 9,        "session: total is 9");
        // Good cycles up to one short of the window: nothing clears yet.
        while (used < 29999) { m.check(3, 3, nullptr, 2, cycle++); ++used; }
        CHECK(m.getWindowRuns() == 3,         "window: intact one cycle before it rolls");
        m.check(3, 3, nullptr, 2, cycle++);   // the 30000th cycle rolls the window
        CHECK(m.getWindowRuns() == 0,         "window: runs cleared when it rolls");
        CHECK(m.getWindowLost() == 0,         "window: lost cleared when it rolls");
        CHECK(m.getWindowLongest() == 0,      "window: longest cleared when it rolls");
        CHECK(m.getLongestRun() == 5,         "session: longest run survives the roll");
        CHECK(m.getTotalErrors() == 9,        "session: total survives the roll");
        run(2);
        CHECK(m.getWindowRuns() == 1,         "window: counting again after the roll");
        CHECK(m.getWindowLongest() == 2,      "window: new longest after the roll");
        CHECK(m.getLongestRun() == 5,         "session: longest unchanged by a shorter run");
    }

    // ---- A run that spans the window roll counts in both windows ----
    {
        WkcMonitor m;
        m.configure(500);
        for (int i = 0; i < 29998; ++i) m.check(3, 3, nullptr, 2, cycle++);
        feedErrors(m, 2, cycle);              // checks 29999 and 30000: the roll lands on the 2nd
        CHECK(m.getWindowRuns() == 1,         "roll inside a run: the run is counted in the new window");
        CHECK(m.getWindowLongest() == 2,      "roll inside a run: longest carries the run so far");
        CHECK(m.getWindowLost() == 0,         "roll inside a run: lost counts only this window's cycles");
        feedErrors(m, 3, cycle);
        CHECK(m.getWindowLost() == 3,         "roll inside a run: later misses counted");
        CHECK(m.getWindowLongest() == 5,      "roll inside a run: longest keeps growing");
        m.check(3, 3, nullptr, 2, cycle++);
        CHECK(m.getWindowRuns() == 1,         "roll inside a run: still one run");
    }

    std::printf("TestWkcMonitor: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
