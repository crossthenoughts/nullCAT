// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// DriveFaultMonitor.h
//
// Per-drive fault state machine extracted from ControlLoop for
// independent testability. Owns all per-drive fault tracking
// arrays (fault seen, retry count, lockout, stable cycles,
// healthy cycle timestamp, fault reset active flag).
//
// Usage per RT cycle (non-emergency-stop path):
//   auto r = m_faultMonitor.step(i, drive, state, cycleCount);
//   if (r.disable)            { drive->disableOperation(); continue; }
//   if (r.firstFaultSeen)     { motion->setNeedsRehome(true); motion->startPark(); }
//   if (r.allClearRehome && motion->needsRehome()) { motion->startHoming(); }
//   if (r.lockoutJustOccurred){ emit faultLockoutOccurred(i, ...); }
//   if (r.seedPosition)       { drive->setTargetPositionRaw(drive->getActualPositionRaw()); }
//   if (r.skipEnableSM)       continue;
//
// Lifetime:
//   call configure(numDrives) once at loop start.
//   call reset() on e-stop rising edge (released) or at loop start.
//   call clearLockout(i) when UI requests per-drive lockout clear.
// ============================================================

#include "A6Drive.h"

#include <cstdint>
#include <cstring>

class DriveFaultMonitor
{
public:
    // ---- Tuneable constants (match ControlLoop values) ----
    static constexpr int      MAX_FAULT_RETRIES         = 5;
    static constexpr uint64_t FAULT_CLEAR_STABLE_CYCLES = 100;
    static constexpr uint64_t HEALTHY_CYCLES_RESET       = 500;
    static constexpr int      MAX_DRIVES                 = 8;

    // ---- Result returned by step() ----
    // All fields default false; only the relevant ones are set.
    struct DriveResult
    {
        // Caller must call drive->disableOperation(); continue;
        bool disable           = false;

        // First time this fault was detected this session.
        // Caller should: motion->setNeedsRehome(true); motion->startPark();
        bool firstFaultSeen    = false;

        // All faults cleared — caller should startHoming() if needsRehome().
        bool allClearRehome    = false;

        // Just hit MAX_FAULT_RETRIES — caller should emit faultLockoutOccurred().
        // disable is also true when this fires.
        bool lockoutJustOccurred = false;

        // Fault cleared via reset or stable-cycles.
        // Caller must: drive->setTargetPositionRaw(drive->getActualPositionRaw()); continue;
        bool seedPosition      = false;

        // Drive is in a fault state still being handled — skip enable SM.
        // (disable=false here means the caller should NOT call disableOperation)
        bool skipEnableSM      = false;
    };

    DriveFaultMonitor();

    // Call once with the actual number of drives before entering the RT loop.
    void configure(int numDrives);

    // Reset all per-drive state (on e-stop release or loop re-start).
    void reset();

    // Clear lockout for one drive (user-requested via UI).
    void clearLockout(int driveIndex);

    // Main per-cycle, per-drive entry point.
    // drive    — used for startFaultReset/stepFaultReset/isFaultResetPending.
    // state    — pre-fetched from drive->getState() this cycle.
    // cycleCount — monotonically increasing RT cycle counter.
    DriveResult step(int driveIndex, A6Drive* drive, DriveState state, uint64_t cycleCount);

    // Accessors for tests and diagnostics.
    bool isLocked     (int i) const { return i >= 0 && i < m_numDrives && m_locked[i]; }
    bool isFaultSeen  (int i) const { return i >= 0 && i < m_numDrives && m_faultSeen[i]; }
    int  getRetryCount(int i) const { return (i >= 0 && i < m_numDrives) ? m_retryCount[i] : 0; }
    bool isAnyFaultSeen() const;

private:
    int m_numDrives = 0;

    bool     m_faultSeen    [MAX_DRIVES] = {};
    int      m_retryCount   [MAX_DRIVES] = {};
    bool     m_locked       [MAX_DRIVES] = {};
    uint64_t m_healthyCycle [MAX_DRIVES] = {};
    uint64_t m_opEnabledSince[MAX_DRIVES] = {};
    bool     m_resetActive  [MAX_DRIVES] = {};
};
