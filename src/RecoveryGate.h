// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// RecoveryGate - retry/pacing policy for the recovery thread's DIRECT
// mailbox reads (panel code, fault history).
//
// The invariant it enforces: between any two access-lock holds by the
// recovery thread, the PDO pump counter must have ADVANCED - i.e. the
// RT loop demonstrably exchanged at least one frame. This turns "PDO
// gets a chance between diagnostic reads" from a scheduler assumption
// (std::mutex guarantees no fairness) into a checked fact, so two
// adjacent 50 ms holds can never stack into a >100 ms PDO gap and trip
// the watchdog during the very fault being diagnosed.
//
// Pure logic, no locking, no time: single-threaded use from the
// recovery thread only. Unit-tested in TestRecoveryGate.
// ============================================================

#include <cstdint>

class RecoveryGate
{
public:
    static constexpr int MAX_ATTEMPTS = 3;

    // Counter-advance predicate shared by every recovery-side pacing
    // decision (retries here, inter-read gaps in the fault-history walk).
    static bool advanced(uint64_t before, uint64_t now) { return now > before; }

    // A new request event (bits = drives). Re-arming a bit that is
    // already pending keeps its attempt count (a burst of fault edges
    // must not grant unlimited retries).
    void arm(uint32_t mask, uint64_t counterNow)
    {
        for (int b = 0; b < 32; ++b)
        {
            if (!(mask & (1u << b)) || m_active[b]) continue;
            m_active[b]   = true;
            m_attempts[b] = 0;
            m_lastAt[b]   = counterNow;   // informational; first attempt is free
        }
    }

    // The next drive allowed an attempt NOW, or -1. Rules:
    //  - a bit's FIRST attempt needs no advance (no prior hold to space from);
    //  - a RETRY needs the counter advanced past that bit's failed attempt;
    //  - GLOBALLY, any attempt needs the counter advanced past the last
    //    attempt on ANY bit (never two adjacent holds, even across drives).
    int nextDue(uint64_t counterNow) const
    {
        if (m_anyAttempts && !advanced(m_lastAnyAt, counterNow)) return -1;
        for (int b = 0; b < 32; ++b)
        {
            if (!m_active[b]) continue;
            if (m_attempts[b] == 0) return b;
            if (advanced(m_lastAt[b], counterNow)) return b;
        }
        return -1;
    }

    // Record an attempt's outcome. Returns true if the bit just gave up
    // (attempt limit reached) so the caller can log the abandonment once.
    bool noteAttempt(int bit, uint64_t counterNow, bool ok)
    {
        if (bit < 0 || bit >= 32 || !m_active[bit]) return false;
        m_anyAttempts = true;
        m_lastAnyAt   = counterNow;
        if (ok) { m_active[bit] = false; return false; }
        m_lastAt[bit] = counterNow;
        if (++m_attempts[bit] >= MAX_ATTEMPTS)
        {
            m_active[bit] = false;
            return true;
        }
        return false;
    }

    bool pending() const
    {
        for (int b = 0; b < 32; ++b) if (m_active[b]) return true;
        return false;
    }

private:
    bool     m_active[32]   = {};
    uint8_t  m_attempts[32] = {};
    uint64_t m_lastAt[32]   = {};
    bool     m_anyAttempts  = false;
    uint64_t m_lastAnyAt    = 0;
};
