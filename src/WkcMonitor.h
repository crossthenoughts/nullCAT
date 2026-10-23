// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// WkcMonitor.h
//
// Working Counter (WKC) error detection and slave recovery.
//
// EtherCAT WKC:
//   Each slave increments the WKC when it processes PDO data.
//   expectedWKC = (outputsWKC * 2) + inputsWKC
//   A mismatch means one or more slaves did not process data - //   typically caused by a dropped slave, cable issue, or
//   slave timeout.
//
// Strategy:
//   1. Track consecutive WKC errors (transient glitches vs
//      sustained loss are handled differently).
//   2. On sustained loss (>= RECOVERY_THRESHOLD consecutive):
//      call ec_recover_slave() / ec_reconfig_slave() for
//      each non-OP slave.
//   3. Expose counters and state to ControlLoop for logging
//      and UI display.
//   4. Log by the minute, not by the cycle: one line per minute that
//      had misses (cycles lost, runs, longest run, session total) and
//      a line for any single outage of 100 ms or more. On Windows the
//      adapter's delivery pauses give ~20 short runs a second; three
//      lines per run was 78,000 lines in 24 minutes (test11, 2026-10-22).
//
// Usage (inside ControlLoop cyclic section):
//
//   int wkc = master->sendReceive();
//   bool ok = m_wkcMonitor.check(wkc, expectedWkc, ctx, slaveCount);
//   if (!ok) { /* skip this cycle's outputs */ }
//
// ============================================================

#include "Logging.h"
#include <algorithm>
#include <cstdint>
#include <atomic>

// SOEM context forward declaration - full headers included by EtherCATMaster.cpp
struct ecx_context;
typedef struct ecx_context ecx_contextt;

class WkcMonitor
{
public:
    WkcMonitor() = default;

    // Recovery trigger is TIME-based, not cycle-based: signal only after
    // ~300ms of CONTINUOUS frame loss (same real-world behavior at any loop
    // rate: 600 cycles at 2kHz, 150 at 500Hz). Blips are already survived by
    // hold-previous-PDO and the drives' own tolerance (100ms PDO watchdog;
    // SYNC0 is slave-generated and keeps firing). The Windows adapter's
    // delivery pauses run up to ~180ms (rig PC, 2026-10-22): with the old
    // 15ms threshold every pause started a scan (443 in 18 minutes), and a
    // scan during a pause is blind (its own state reads time out), so it
    // "recovered" slaves that were never lost. A genuinely dropped slave is
    // still dropped 300ms later; nothing is lost.
    void configure(int controlLoopHz)
    {
        m_hz = std::max(1, controlLoopHz);
        m_recoveryThresholdCycles =
            std::max(3, static_cast<int>(m_hz * RECOVERY_THRESHOLD_SEC));
        m_notableRunCycles = std::max(1, static_cast<int>(m_hz * NOTABLE_RUN_SEC));
        m_summaryCycles    = std::max(1, static_cast<int>(m_hz * SUMMARY_SEC));
    }

    void reset()
    {
        m_consecutiveErrors = 0;
        m_totalErrors = 0;
        m_recoveryAttempts = 0;
        m_lastErrorCycle = 0;
        m_inRecovery = false;
        m_longestRun = 0;
        m_windowCycles = 0;
        clearWindow();
    }

    // Call each PDO cycle with the result of sendReceive()
    // Returns true if WKC is OK (safe to use drive outputs)
    // Returns false if WKC mismatch (skip drive outputs this cycle)
    bool check(int wkc, int expectedWkc, void* ctx, int slaveCount, uint64_t cycleNum)
    {
        ++m_windowCycles;
        const bool ok = (wkc == expectedWkc);
        if (ok)
        {
            if (m_consecutiveErrors > 0) endRun();
        }
        else
        {
            ++m_consecutiveErrors;
            ++m_totalErrors;
            ++m_windowLost;
            m_lastErrorCycle = cycleNum;
            if (m_consecutiveErrors == 1) ++m_windowRuns;
            m_windowLongest = std::max(m_windowLongest, m_consecutiveErrors);
            m_longestRun    = std::max(m_longestRun, m_consecutiveErrors);

            // A long outage reports once a second while it lasts.
            if (m_consecutiveErrors % m_hz == 0)
                RT_LOG_WARNING("WkcMonitor: no valid reply for %.1f s (expected WKC %d, got %d).",
                    static_cast<double>(m_consecutiveErrors) / m_hz, expectedWkc, wkc);

            // Attempt recovery after sustained loss (time-based; see configure())
            if (m_consecutiveErrors >= m_recoveryThresholdCycles && !m_inRecovery)
                attemptRecovery(ctx, slaveCount, cycleNum);
        }
        if (m_windowCycles >= m_summaryCycles) summarise();
        return ok;   // false: the caller holds the previous inputs this cycle
    }

    uint64_t getTotalErrors()       const { return m_totalErrors; }
    int      getConsecutiveErrors() const { return m_consecutiveErrors; }
    int      getRecoveryAttempts()  const { return m_recoveryAttempts; }
    bool     isInRecovery()         const { return m_inRecovery; }
    // Longest run of misses this session (cycles).
    int      getLongestRun()        const { return m_longestRun; }
    // The current summary window (cleared every SUMMARY_SEC of cycles).
    uint64_t getWindowLost()        const { return m_windowLost; }
    int      getWindowRuns()        const { return m_windowRuns; }
    int      getWindowLongest()     const { return m_windowLongest; }
    int      summaryCycles()        const { return m_summaryCycles; }
    int      notableRunCycles()     const { return m_notableRunCycles; }

private:
    int      m_consecutiveErrors = 0;
    uint64_t m_totalErrors = 0;
    int      m_recoveryAttempts = 0;
    uint64_t m_lastErrorCycle = 0;
    bool     m_inRecovery = false;
    int      m_longestRun = 0;
    uint64_t m_windowCycles = 0;
    uint64_t m_windowLost = 0;
    int      m_windowRuns = 0;
    int      m_windowLongest = 0;

    // ~300ms of continuous loss before signalling recovery (see configure()).
    // Default matches a 2kHz loop if configure() is never called.
    static constexpr double RECOVERY_THRESHOLD_SEC = 0.3;
    int m_recoveryThresholdCycles = 600;
    // A single run this long gets its own line when it ends; shorter runs
    // only count towards the minute summary.
    static constexpr double NOTABLE_RUN_SEC = 0.1;
    static constexpr double SUMMARY_SEC     = 60.0;
    int m_hz               = 2000;
    int m_notableRunCycles = 200;
    int m_summaryCycles    = 120000;

    void clearWindow()
    {
        m_windowLost = 0;
        m_windowRuns = 0;
        m_windowLongest = 0;
    }

    // The run of misses just ended: a line only when it was long enough
    // to matter (the drives' own PDO watchdog is 100 ms).
    void endRun()
    {
        if (m_consecutiveErrors >= m_notableRunCycles)
            RT_LOG_WARNING("WkcMonitor: replies back after %d cycles (%.0f ms) without a valid one.",
                m_consecutiveErrors, 1000.0 * m_consecutiveErrors / m_hz);
        m_consecutiveErrors = 0;
        m_inRecovery = false;
    }

    // Once per window: one line if anything was missed, then start over.
    void summarise()
    {
        if (m_windowLost > 0)
            RT_LOG_WARNING("WkcMonitor: last %.0f s: %llu cycles without a valid reply in %d runs "
                "(longest %.0f ms); session total %llu.",
                static_cast<double>(m_windowCycles) / m_hz,
                static_cast<unsigned long long>(m_windowLost), m_windowRuns,
                1000.0 * m_windowLongest / m_hz,
                static_cast<unsigned long long>(m_totalErrors));
        m_windowCycles = 0;
        clearWindow();
        // A run that spans the roll counts in both windows.
        if (m_consecutiveErrors > 0)
        {
            m_windowRuns = 1;
            m_windowLongest = m_consecutiveErrors;
        }
    }
    // Per-slave recovery attempt capping lives in the recovery
    // thread itself (EtherCATMaster::kMaxRecoveryReconfigAttempts);
    // this class only signals that recovery is needed.

    void attemptRecovery(void* ctx, int slaveCount, uint64_t cycleNum)
    {
        ++m_recoveryAttempts;
        m_inRecovery = true;
        RT_LOG_WARNING("WkcMonitor: %d consecutive WKC errors -- recovery delegated to ControlLoop.",
            m_consecutiveErrors);
        // Actual ecx_recover_slave calls happen in ControlLoop.cpp
        // which has full SOEM header access
    }
};
