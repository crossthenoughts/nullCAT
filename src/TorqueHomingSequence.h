// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// TorqueHomingSequence - torque-only homing for CST device axes
// (HomingKind::Torque: shifter, pedal; any future control-loading row).
//
// The device families run CST, whose PDO layout deliberately binds the
// position target null - the CSP torque-search (HomingSequence) cannot
// move them. Instead: enable the drive, settle, command a LOW constant
// torque toward a travel stop, and detect arrival from the position
// derivative (the lever stalls against the stop). The raw count at the
// stall is latched; MotionController maps it onto the configured stop
// position (device.homeDir < 0 -> stopMinRev, else stopMaxRev), so the
// device frame needs no A6Drive home-offset plumbing and no mm scaling.
//
// A SIBLING of HomingSequence, deliberately not a refactor of it: the
// CSP path stays untouched. Same design rules: header-only, no SOEM, no
// Qt, RT-safe, every exit narrated.
// ============================================================

#include "A6Drive.h"
#include "Config.h"
#include "Logging.h"
#include <cmath>
#include <cstdint>

class TorqueHomingSequence
{
public:
    enum class State : uint8_t
    { Idle, EnablingDrive, Settle, Search, Confirm, Complete, FatalError };

    void configure(const DeviceParams& p, double encoderCountsPerRev,
                   double cycleTimeSec)
    {
        m_p       = p;
        m_cpr     = (encoderCountsPerRev > 0.0) ? encoderCountsPerRev : 131072.0;
        m_dt      = cycleTimeSec;
        m_state   = State::Idle;
    }

    void reset() { m_state = State::Idle; m_elapsed = 0.0; }

    void start(A6Drive* drive)
    {
        if (!drive) return;
        m_state       = State::EnablingDrive;
        m_elapsed     = 0.0;
        m_stallCycles = 0;
        m_startRaw    = drive->getActualPositionRaw();
        m_prevRaw     = m_startRaw;
        m_homeRaw     = 0.0;
        LOG_INFO(strf("TorqueHoming[%d]: starting stall search (%.0f%% toward %s stop)",
            drive->getSlaveIndex(), m_p.homeTorquePct,
            m_p.homeDir < 0.0 ? "min" : "max"));
    }

    // Advance one cycle. Writes the cycle's torque command to the drive
    // (the sequence OWNS the drive while the axis is HOMING -- ControlLoop
    // skips its normal write block, exactly as with HomingSequence) and
    // returns the same value so the caller can mirror it into
    // output.torques[] for the cards. homeDir is DEVICE-frame (-1 = toward
    // stopMinRev); `dir` maps it onto the motor's torque sign, the same
    // composition DeviceForceModel applies to its output.
    double step(A6Drive* drive)
    {
        if (drive) drive->updateStatus();   // sibling contract: step() is the tick
        const double cmd = stepInner(drive);
        if (drive) drive->setTargetTorque(cmd);
        return cmd;
    }

private:
    double stepInner(A6Drive* drive)
    {
        if (!drive || m_state == State::Idle ||
            m_state == State::Complete || m_state == State::FatalError)
            return 0.0;

        m_elapsed += m_dt;
        const double raw = drive->getActualPositionRaw();

        switch (m_state)
        {
        case State::EnablingDrive:
        {
            if (drive->stepEnableStateMachine())
            {
                m_state   = State::Settle;
                m_elapsed = 0.0;
                LOG_INFO(strf("TorqueHoming[%d]: drive enabled -- settling",
                    drive->getSlaveIndex()));
            }
            else if (m_elapsed > ENABLE_TIMEOUT_SEC)
            {
                LOG_ERROR(strf("TorqueHoming[%d]: enable timeout", drive->getSlaveIndex()));
                m_state = State::FatalError;
            }
            return 0.0;
        }

        case State::Settle:
            if (m_elapsed >= SETTLE_SEC)
            {
                m_state       = State::Search;
                m_elapsed     = 0.0;
                m_startRaw    = raw;
                m_prevRaw     = raw;
                m_stallCycles = 0;
            }
            return 0.0;

        case State::Search:
        case State::Confirm:
        {
            // Travel guard first: never push past 1.5x the configured span.
            const double travelRev = std::fabs(raw - m_startRaw) / m_cpr;
            const double spanRev   = m_p.stopMaxRev - m_p.stopMinRev;
            if (travelRev > 1.5 * spanRev + 0.02)
            {
                LOG_ERROR(strf("TorqueHoming[%d]: TRAVEL GUARD (%.3f rev of %.3f rev span) "
                    "-- check device.homeDir / mechanics", drive->getSlaveIndex(),
                    travelRev, spanRev));
                m_state = State::FatalError;
                return 0.0;
            }
            if (m_elapsed > SEARCH_TIMEOUT_SEC)
            {
                LOG_ERROR(strf("TorqueHoming[%d]: search TIMEOUT (travelled %.3f rev)",
                    drive->getSlaveIndex(), travelRev));
                m_state = State::FatalError;
                return 0.0;
            }

            // Stall detection: per-cycle displacement below the epsilon for
            // a sustained window while torque is applied = the stop.
            const double stepRev = std::fabs(raw - m_prevRaw) / m_cpr;
            m_prevRaw = raw;
            if (stepRev < STALL_EPS_REV) ++m_stallCycles;
            else                          m_stallCycles = 0;

            // Search -> Confirm at the first window; Complete only after the
            // FULL count. (An earlier revision computed the target before the
            // transition, collapsing the Confirm phase to zero extra cycles -
            // spotted in the bench logs as a 0.06 s search.)
            if (m_state == State::Search && m_stallCycles >= STALL_CONFIRM_CYCLES)
                m_state = State::Confirm;   // keep pushing, keep counting
            if (m_state == State::Confirm &&
                m_stallCycles >= STALL_CONFIRM_CYCLES + CONFIRM_EXTRA_CYCLES)
            {
                m_homeRaw = raw;
                m_state   = State::Complete;
                LOG_INFO(strf("TorqueHoming[%d]: STOP FOUND at raw=%.0f "
                    "(travelled %.3f rev, %.2fs) -- home latched",
                    drive->getSlaveIndex(), raw, travelRev, m_elapsed));
                return 0.0;
            }
            return m_p.dir * m_p.homeDir * m_p.homeTorquePct;
        }

        default:
            return 0.0;
        }
    }

public:
    State  getState()  const { return m_state; }
    bool   isComplete()   const { return m_state == State::Complete; }
    bool   isFatalError() const { return m_state == State::FatalError; }
    double getHomeRaw()   const { return m_homeRaw; }
    // The configured position the found stop corresponds to.
    double homeStopRev()  const
    { return (m_p.homeDir < 0.0) ? m_p.stopMinRev : m_p.stopMaxRev; }

    static const char* stateName(State s)
    {
        switch (s)
        {
        case State::Idle:          return "Idle";
        case State::EnablingDrive: return "EnablingDrive";
        case State::Settle:        return "Settle";
        case State::Search:        return "Search";
        case State::Confirm:       return "Confirm";
        case State::Complete:      return "Complete";
        case State::FatalError:    return "FatalError";
        }
        return "?";
    }

private:
    static constexpr double ENABLE_TIMEOUT_SEC   = 5.0;
    static constexpr double SETTLE_SEC           = 0.1;
    static constexpr double SEARCH_TIMEOUT_SEC   = 10.0;
    static constexpr double STALL_EPS_REV        = 0.0005; // ~65 counts/cycle
    static constexpr int    STALL_CONFIRM_CYCLES = 100;
    static constexpr int    CONFIRM_EXTRA_CYCLES = 100;

    DeviceParams m_p;
    double m_cpr     = 131072.0;
    double m_dt      = 0.002;
    State  m_state   = State::Idle;
    double m_elapsed = 0.0;
    double m_startRaw = 0.0;
    double m_prevRaw  = 0.0;
    double m_homeRaw  = 0.0;
    int    m_stallCycles = 0;
};
