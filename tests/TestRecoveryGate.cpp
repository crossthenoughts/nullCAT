// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestRecoveryGate.cpp - pins the recovery thread's mailbox pacing policy.
//
// The invariant under test: between any two access-lock holds the PDO
// pump counter must have ADVANCED - a verified frame, not a scheduler
// hope. Covers: no immediate retry after a failed read; retry only after
// the observed counter advances; global pacing across drives (each read
// waits a fresh PDO cycle); attempt exhaustion; and that failure paths
// leave the gate in a usable state (the recovery thread survives).
// ============================================================

#include "../src/RecoveryGate.h"
#include <cstdio>

static int g_pass = 0, g_fail = 0;
static void CHECK(bool ok, const char* what)
{
    if (ok) { ++g_pass; }
    else    { ++g_fail; std::printf("FAIL: %s\n", what); }
}

int main()
{
    // ---- first attempt is free; failed attempt never retries same-cycle ----
    {
        RecoveryGate g;
        g.arm(1u << 3, 100);
        CHECK(g.nextDue(100) == 3, "armed bit is due immediately (no prior hold)");
        g.noteAttempt(3, 100, false);
        CHECK(g.nextDue(100) == -1, "NO immediate retry at the same counter value");
        CHECK(g.nextDue(101) == 3,  "retry allowed once the counter has advanced");
    }

    // ---- global pacing: a second drive also waits for a fresh cycle ----
    {
        RecoveryGate g;
        g.arm((1u << 1) | (1u << 4), 50);
        const int first = g.nextDue(50);
        CHECK(first == 1, "lowest armed bit attempts first");
        g.noteAttempt(1, 50, true);
        CHECK(g.nextDue(50) == -1,
              "the OTHER drive must wait for a verified PDO frame (global gate)");
        CHECK(g.nextDue(51) == 4, "other drive due after the counter advances");
        g.noteAttempt(4, 51, true);
        CHECK(!g.pending(), "all satisfied");
    }

    // ---- exhaustion: bounded attempts, then a one-time give-up signal ----
    {
        RecoveryGate g;
        g.arm(1u << 0, 10);
        uint64_t c = 10;
        bool gaveUp = false;
        for (int a = 0; a < RecoveryGate::MAX_ATTEMPTS; ++a)
        {
            const int b = g.nextDue(c);
            CHECK(b == 0, "due while attempts remain");
            gaveUp = g.noteAttempt(0, c, false);
            c += 2;   // pump advances between attempts
        }
        CHECK(gaveUp, "final failed attempt reports give-up exactly once");
        CHECK(!g.pending() && g.nextDue(c + 10) == -1,
              "exhausted bit never retries (waits for a fresh fault event)");
    }

    // ---- failure paths leave the gate usable (recovery thread survives) ----
    {
        RecoveryGate g;
        g.noteAttempt(7, 5, false);            // note on an unarmed bit: no-op
        g.noteAttempt(-1, 5, false);           // out of range: no-op
        g.noteAttempt(32, 5, true);            // out of range: no-op
        CHECK(!g.pending(), "stray notes never create phantom work");
        g.arm(1u << 7, 6);
        CHECK(g.nextDue(6) == 7, "gate fully functional after stray notes");
        g.noteAttempt(7, 6, true);
        CHECK(!g.pending(), "clean completion");
    }

    // ---- re-arm during pending keeps the attempt budget (no retry laundering) ----
    {
        RecoveryGate g;
        g.arm(1u << 2, 0);
        g.noteAttempt(2, 0, false);
        g.arm(1u << 2, 1);                     // fault edge fires again mid-retry
        CHECK(g.nextDue(1) == 2, "still due after re-arm");
        g.noteAttempt(2, 1, false);
        bool gaveUp = g.noteAttempt(2, 2, false);
        CHECK(gaveUp, "re-arming did not reset the attempt budget");
    }

    // ---- the shared advance predicate (fault-history inter-read gaps) ----
    {
        CHECK(!RecoveryGate::advanced(9, 9), "same counter = no frame observed");
        CHECK(RecoveryGate::advanced(9, 10), "any increment = a verified frame");
        CHECK(!RecoveryGate::advanced(10, 9), "regression never counts as advance");
    }

    std::printf("TestRecoveryGate: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
