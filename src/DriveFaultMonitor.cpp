// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// DriveFaultMonitor.cpp - see DriveFaultMonitor.h
// ============================================================
#include "DriveFaultMonitor.h"
#include <cstring>

DriveFaultMonitor::DriveFaultMonitor()
{
    std::memset(m_faultSeen,     0, sizeof(m_faultSeen));
    std::memset(m_retryCount,    0, sizeof(m_retryCount));
    std::memset(m_locked,        0, sizeof(m_locked));
    std::memset(m_healthyCycle,  0, sizeof(m_healthyCycle));
    std::memset(m_opEnabledSince,0, sizeof(m_opEnabledSince));
    std::memset(m_resetActive,   0, sizeof(m_resetActive));
}

void DriveFaultMonitor::configure(int numDrives)
{
    m_numDrives = (numDrives <= MAX_DRIVES) ? numDrives : MAX_DRIVES;
    reset();
}

void DriveFaultMonitor::reset()
{
    std::memset(m_faultSeen,     0, sizeof(m_faultSeen));
    std::memset(m_retryCount,    0, sizeof(m_retryCount));
    std::memset(m_locked,        0, sizeof(m_locked));
    std::memset(m_healthyCycle,  0, sizeof(m_healthyCycle));
    std::memset(m_opEnabledSince,0, sizeof(m_opEnabledSince));
    std::memset(m_resetActive,   0, sizeof(m_resetActive));
}

void DriveFaultMonitor::clearLockout(int i)
{
    if (i < 0 || i >= m_numDrives) return;
    m_locked[i]       = false;
    m_retryCount[i]   = 0;
    m_faultSeen[i]    = false;
    m_resetActive[i]  = false;
}

bool DriveFaultMonitor::isAnyFaultSeen() const
{
    for (int j = 0; j < m_numDrives; ++j)
        if (m_faultSeen[j]) return true;
    return false;
}

DriveFaultMonitor::DriveResult DriveFaultMonitor::step(
    int driveIndex, A6Drive* drive, DriveState state, uint64_t cycleCount)
{
    DriveResult r;
    const int i = driveIndex;

    if (i < 0 || i >= m_numDrives || !drive)
        return r;

    // ---- Lockout gate ----
    if (m_locked[i])
    {
        r.disable = true;
        return r;
    }

    // ---- Fault or FaultReactionActive ----
    if (state == DriveState::Fault || state == DriveState::FaultReactionActive)
    {
        // First detection this fault episode
        if (!m_faultSeen[i])
        {
            m_faultSeen[i]   = true;
            m_resetActive[i] = false;
            r.firstFaultSeen = true;
        }

        if (state == DriveState::Fault)
        {
            // Check retry budget
            if (m_retryCount[i] >= MAX_FAULT_RETRIES)
            {
                if (!m_locked[i])
                {
                    m_locked[i]          = true;
                    r.lockoutJustOccurred = true;
                }
                r.disable = true;
                return r;
            }

            // Start reset if not already active
            if (!m_resetActive[i])
            {
                m_resetActive[i] = true;
                ++m_retryCount[i];
                drive->startFaultReset();
            }

            // Step the reset state machine
            bool cleared = drive->stepFaultReset();
            if (cleared)
            {
                m_resetActive[i]    = false;
                m_opEnabledSince[i] = 0;
                m_healthyCycle[i]   = cycleCount;

                // Clear faultSeen now so the allClear check works immediately
                m_faultSeen[i] = false;

                if (!isAnyFaultSeen())
                    r.allClearRehome = true;

                r.seedPosition = true;
                return r;   // caller: seed position then continue
            }
            else if (!drive->isFaultResetPending())
            {
                // Reset attempt timed out
                m_resetActive[i] = false;
            }
        }

        // Still in fault state (FaultReactionActive, or Fault not yet cleared)
        r.skipEnableSM = true;
        return r;
    }

    // ---- Non-fault path ----
    if (m_faultSeen[i])
    {
        if (state == DriveState::OperationEnabled)
        {
            if (m_opEnabledSince[i] == 0)
                m_opEnabledSince[i] = cycleCount;

            uint64_t stableCycles = cycleCount - m_opEnabledSince[i];
            if (stableCycles >= FAULT_CLEAR_STABLE_CYCLES)
            {
                // Self-recovering drives reach here instead of the cleared branch above
                m_faultSeen[i]    = false;
                m_resetActive[i]  = false;
                m_healthyCycle[i] = cycleCount;

                if (!isAnyFaultSeen())
                    r.allClearRehome = true;
            }
        }
        else
        {
            m_opEnabledSince[i] = 0;
        }
    }

    // Healthy-cycle retry counter reset
    if (m_retryCount[i] > 0 &&
        m_healthyCycle[i] > 0 &&
        (cycleCount - m_healthyCycle[i]) > HEALTHY_CYCLES_RESET)
    {
        m_retryCount[i] = 0;
    }

    return r;   // proceed with enable state machine
}
