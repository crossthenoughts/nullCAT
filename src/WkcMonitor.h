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
//   A mismatch means one or more slaves did not process data —
//   typically caused by a dropped slave, cable issue, or
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

// SOEM context forward declaration — full headers included by EtherCATMaster.cpp
struct ecx_context;
typedef struct ecx_context ecx_contextt;

class WkcMonitor
{
public:
    WkcMonitor() = default;

    // Recovery trigger is TIME-based, not cycle-based: signal only after
    // ~15ms of CONTINUOUS frame loss (same real-world behavior at any loop
    // rate: 30 cycles at 2kHz, 7 at 500Hz). Blips are already survived by
    // hold-previous-PDO and the drives' own tolerance (100ms PDO watchdog;
    // SYNC0 is slave-generated and keeps firing) -- a threshold of only a
    // few cycles (1.5ms at 2kHz) guarantees the recovery scan runs DURING
    // bursts, when its own state reads are blind (see doRecoveryScan Branch 4).
    // A genuinely dropped slave is still dropped 15ms later; nothing is lost.
    void configure(int controlLoopHz)
    {
        m_recoveryThresholdCycles =
            std::max(3, static_cast<int>(controlLoopHz * RECOVERY_THRESHOLD_SEC));
    }

    void reset()
    {
        m_consecutiveErrors = 0;
        m_totalErrors = 0;
        m_recoveryAttempts = 0;
        m_lastErrorCycle = 0;
        m_inRecovery = false;
    }

    // Call each PDO cycle with the result of sendReceive()
    // Returns true if WKC is OK (safe to use drive outputs)
    // Returns false if WKC mismatch (skip drive outputs this cycle)
    bool check(int wkc, int expectedWkc, void* ctx, int slaveCount, uint64_t cycleNum)
    {
        if (wkc == expectedWkc)
        {
            // Good cycle
            if (m_consecutiveErrors > 0)
            {
                RT_LOG_INFO("WkcMonitor: WKC recovered after %d consecutive errors.",
                    m_consecutiveErrors);
                m_consecutiveErrors = 0;
                m_inRecovery = false;
            }
            return true;
        }

        // WKC mismatch
        ++m_consecutiveErrors;
        ++m_totalErrors;
        m_lastErrorCycle = cycleNum;

        // Log throttled: first error, then every 100
        if (m_consecutiveErrors == 1 || m_consecutiveErrors % 100 == 0)
        {
            RT_LOG_WARNING("WkcMonitor: WKC mismatch -- expected %d got %d "
                "(consecutive: %d, total: %llu)",
                expectedWkc, wkc, m_consecutiveErrors,
                static_cast<unsigned long long>(m_totalErrors));
        }

        // Attempt recovery after sustained loss (time-based; see configure())
        if (m_consecutiveErrors >= m_recoveryThresholdCycles && !m_inRecovery)
        {
            attemptRecovery(ctx, slaveCount, cycleNum);
        }

        return false;  // Caller should skip drive outputs this cycle
    }

    uint64_t getTotalErrors()       const { return m_totalErrors; }
    int      getConsecutiveErrors() const { return m_consecutiveErrors; }
    int      getRecoveryAttempts()  const { return m_recoveryAttempts; }
    bool     isInRecovery()         const { return m_inRecovery; }

private:
    int      m_consecutiveErrors = 0;
    uint64_t m_totalErrors = 0;
    int      m_recoveryAttempts = 0;
    uint64_t m_lastErrorCycle = 0;
    bool     m_inRecovery = false;

    // ~15ms of continuous loss before signalling recovery (see configure()).
    // Default matches a 2kHz loop if configure() is never called.
    static constexpr double RECOVERY_THRESHOLD_SEC = 0.015;
    int m_recoveryThresholdCycles = 30;
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
