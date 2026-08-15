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

    // ---- Default threshold (configure never called) = 30 cycles ----
    {
        WkcMonitor m;
        feedErrors(m, 29, cycle);
        CHECK(!m.isInRecovery(),              "default: 29 consecutive - no signal yet");
        CHECK(m.getRecoveryAttempts() == 0,   "default: no attempt before threshold");
        feedErrors(m, 1, cycle);
        CHECK(m.isInRecovery(),               "default: signal fires exactly at 30");
        CHECK(m.getRecoveryAttempts() == 1,   "default: exactly one attempt at threshold");
    }

    // ---- configure(): threshold = max(3, hz * 0.015), int-truncated ----
    {
        WkcMonitor m;
        m.configure(2000);                    // 2000 * 0.015 = 30
        feedErrors(m, 29, cycle);
        CHECK(!m.isInRecovery(),              "2kHz: no signal at 29");
        feedErrors(m, 1, cycle);
        CHECK(m.isInRecovery(),               "2kHz: signal at 30 (~15ms)");
    }
    {
        WkcMonitor m;
        m.configure(500);                     // 500 * 0.015 = 7.5 → truncates to 7
        feedErrors(m, 6, cycle);
        CHECK(!m.isInRecovery(),              "500Hz: no signal at 6");
        feedErrors(m, 1, cycle);
        CHECK(m.isInRecovery(),               "500Hz: signal at 7 (7.5 truncated, ~14ms)");
    }
    {
        WkcMonitor m;
        m.configure(100);                     // 100 * 0.015 = 1.5 → floor of 3 applies
        feedErrors(m, 2, cycle);
        CHECK(!m.isInRecovery(),              "100Hz: floor - no signal at 2");
        feedErrors(m, 1, cycle);
        CHECK(m.isInRecovery(),               "100Hz: floor - signal at 3");
    }

    // ---- Latch: sustained loss signals ONCE, not per cycle ----
    {
        WkcMonitor m;
        m.configure(2000);
        feedErrors(m, 200, cycle);
        CHECK(m.getRecoveryAttempts() == 1,   "latch: 200 consecutive errors = 1 attempt");
        CHECK(m.getConsecutiveErrors() == 200,"latch: consecutive keeps counting");
        CHECK(m.getTotalErrors() == 200,      "latch: total keeps counting");
    }

    // ---- Good cycle clears consecutive + latch; total persists ----
    {
        WkcMonitor m;
        m.configure(2000);
        feedErrors(m, 40, cycle);
        CHECK(m.isInRecovery(),               "setup: in recovery after 40");
        CHECK(m.check(3, 3, nullptr, 2, cycle++), "good frame returns true again");
        CHECK(m.getConsecutiveErrors() == 0,  "good frame clears consecutive");
        CHECK(!m.isInRecovery(),              "good frame clears inRecovery");
        CHECK(m.getTotalErrors() == 40,       "total errors persist across recovery");
        CHECK(m.getRecoveryAttempts() == 1,   "attempts persist across recovery");
    }

    // ---- Blip below threshold + good frame: never signals ----
    {
        WkcMonitor m;
        m.configure(2000);
        for (int burst = 0; burst < 10; ++burst)
        {
            feedErrors(m, 29, cycle);                 // one under threshold
            m.check(3, 3, nullptr, 2, cycle++);       // recovers
        }
        CHECK(m.getRecoveryAttempts() == 0,   "repeated sub-threshold bursts never signal");
        CHECK(m.getTotalErrors() == 290,      "sub-threshold bursts still counted in total");
    }

    // ---- Re-trigger: a SECOND sustained loss signals again ----
    {
        WkcMonitor m;
        m.configure(2000);
        feedErrors(m, 30, cycle);
        m.check(3, 3, nullptr, 2, cycle++);           // recover
        feedErrors(m, 30, cycle);
        CHECK(m.getRecoveryAttempts() == 2,   "second sustained loss signals a second attempt");
        CHECK(m.isInRecovery(),               "second loss latches inRecovery again");
    }

    // ---- reset(): everything back to zero ----
    {
        WkcMonitor m;
        m.configure(2000);
        feedErrors(m, 50, cycle);
        m.reset();
        CHECK(m.getConsecutiveErrors() == 0,  "reset: consecutive cleared");
        CHECK(m.getTotalErrors() == 0,        "reset: total cleared");
        CHECK(m.getRecoveryAttempts() == 0,   "reset: attempts cleared");
        CHECK(!m.isInRecovery(),              "reset: inRecovery cleared");
        feedErrors(m, 30, cycle);
        CHECK(m.getRecoveryAttempts() == 1,   "reset: threshold behavior intact afterwards");
    }

    std::printf("TestWkcMonitor: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
