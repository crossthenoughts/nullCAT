// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// A6Drive.cpp
//
// DS402 servo drive interface: fixed PDO mapping, enable state
// machine, fault reset, homing, and position/torque targets with
// scaling, limit clamping, and rate limiting.
// ============================================================

#include "A6Drive.h"
#include "Logging.h"

#include <cmath>
#include <algorithm>

#ifdef SOEM_AVAILABLE
extern "C" {
#include "soem/ec_options.h"
#include "soem/ec_type.h"
#include "nicdrv.h"
#include "soem/ec_base.h"
#include "soem/ec_main.h"
#include "soem/ec_coe.h"
}
#endif

A6Drive::A6Drive()
{
    // m_countsPerMm has a sensible default (13107.2 for 10mm pitch).
    // setScaling() overrides it at init from the resolved config.
}

void A6Drive::setSlaveIndex(int slaveIndex)
{
    m_slaveIndex = slaveIndex;
}

void A6Drive::setScaling(double countsPerMm)
{
    // countsPerMm is the single canonical scaling, derived by Config
    // from encoderCountsPerRev / ballscrewPitch.
    m_countsPerMm = (countsPerMm > 0.0) ? countsPerMm : 1.0;
    LOG_INFO(strf("A6Drive[%d]: Scaling = %g counts/mm", m_slaveIndex, m_countsPerMm));
}

void A6Drive::setLimits(double minPos, double maxPos)
{
    m_minPos = minPos;
    m_maxPos = maxPos;
}

void A6Drive::setMaxVelocity(double maxVelocityUnitsPerSec, double cycleTimeSec)
{
    m_maxDeltaPerCycle = maxVelocityUnitsPerSec * cycleTimeSec;
    LOG_INFO(strf("A6Drive[%d]: Rate limiter = %.4f units/cycle (%.1f units/s at %.0f Hz)",
        m_slaveIndex, m_maxDeltaPerCycle, maxVelocityUnitsPerSec, 1.0 / cycleTimeSec));
}

void A6Drive::setPDOPointers(
    uint16_t* controlword,
    int8_t* modeOfOperation,
    int32_t* targetPosition,
    uint16_t* statusword,
    int8_t* modeOfOpDisplay,
    int32_t* actualPosition,
    int16_t* torqueFeedback)
{
    m_pControlword = controlword;
    m_pModeOfOperation = modeOfOperation;
    m_pTargetPosition = targetPosition;
    m_pStatusword = statusword;
    m_pModeOfOpDisplay = modeOfOpDisplay;
    m_pActualPosition = actualPosition;
    m_pTorqueFeedback = torqueFeedback;
    LOG_INFO(strf("A6Drive[%d]: PDO pointers set.%s",
        m_slaveIndex, torqueFeedback ? " (torque feedback available)" : ""));
}

bool A6Drive::configurePDOs(ecx_contextt* ctx)
{
#ifndef SOEM_AVAILABLE
    return true;
#else
    if (!ctx) return false;

    LOG_INFO(strf("A6Drive[%d]: Configuring PDOs (fixed mapping)...", m_slaveIndex));

    int wkc = 0;
    bool ok = true;
    uint8_t zero = 0;
    uint8_t one = 1;

    wkc = ecx_SDOwrite(ctx, (uint16)m_slaveIndex, 0x1C12, 0x00, FALSE, sizeof(zero), &zero, 700000);
    if (wkc <= 0) { LOG_WARNING(strf("A6Drive[%d]: SM2 clear failed (may be ok)", m_slaveIndex)); }

    wkc = ecx_SDOwrite(ctx, (uint16)m_slaveIndex, 0x1C13, 0x00, FALSE, sizeof(zero), &zero, 700000);
    if (wkc <= 0) { LOG_WARNING(strf("A6Drive[%d]: SM3 clear failed (may be ok)", m_slaveIndex)); }

    uint16_t rpdo = 0x1702;
    wkc = ecx_SDOwrite(ctx, (uint16)m_slaveIndex, 0x1C12, 0x01, FALSE, sizeof(rpdo), &rpdo, 700000);
    if (wkc <= 0) { LOG_ERROR(strf("A6Drive[%d]: Failed assign RPDO 1702h", m_slaveIndex)); ok = false; }

    wkc = ecx_SDOwrite(ctx, (uint16)m_slaveIndex, 0x1C12, 0x00, FALSE, sizeof(one), &one, 700000);
    if (wkc <= 0) { LOG_ERROR(strf("A6Drive[%d]: Failed set SM2 count", m_slaveIndex)); ok = false; }

    uint16_t tpdo = 0x1B02;
    wkc = ecx_SDOwrite(ctx, (uint16)m_slaveIndex, 0x1C13, 0x01, FALSE, sizeof(tpdo), &tpdo, 700000);
    if (wkc <= 0) { LOG_ERROR(strf("A6Drive[%d]: Failed assign TPDO 1B02h", m_slaveIndex)); ok = false; }

    wkc = ecx_SDOwrite(ctx, (uint16)m_slaveIndex, 0x1C13, 0x00, FALSE, sizeof(one), &one, 700000);
    if (wkc <= 0) { LOG_ERROR(strf("A6Drive[%d]: Failed set SM3 count", m_slaveIndex)); ok = false; }

    if (ok)
        LOG_INFO(strf("A6Drive[%d]: PDO config OK (RPDO=1702h TPDO=1B02h).", m_slaveIndex));
    else
        LOG_WARNING(strf("A6Drive[%d]: PDO config errors.", m_slaveIndex));

    return ok;
#endif
}

bool A6Drive::configurePDOsStatic(int slaveIndex, ecx_contextt* ctx)
{
#ifndef SOEM_AVAILABLE
    return true;
#else
    if (!ctx) return false;
    LOG_INFO(strf("A6Drive[%d]: Configuring PDOs (fixed mapping)...", slaveIndex));
    bool ok = true;
    int wkc = 0;
    uint8_t zero = 0;
    uint8_t one = 1;

    wkc = ecx_SDOwrite(ctx, (uint16)slaveIndex, 0x1C12, 0x00, FALSE, sizeof(zero), &zero, 700000);
    if (wkc <= 0) LOG_WARNING(strf("A6Drive[%d]: SM2 clear failed (may be ok)", slaveIndex));

    wkc = ecx_SDOwrite(ctx, (uint16)slaveIndex, 0x1C13, 0x00, FALSE, sizeof(zero), &zero, 700000);
    if (wkc <= 0) LOG_WARNING(strf("A6Drive[%d]: SM3 clear failed (may be ok)", slaveIndex));

    uint16_t rpdo = 0x1702;
    wkc = ecx_SDOwrite(ctx, (uint16)slaveIndex, 0x1C12, 0x01, FALSE, sizeof(rpdo), &rpdo, 700000);
    if (wkc <= 0) { LOG_ERROR(strf("A6Drive[%d]: Failed assign RPDO 1702h", slaveIndex)); ok = false; }

    wkc = ecx_SDOwrite(ctx, (uint16)slaveIndex, 0x1C12, 0x00, FALSE, sizeof(one), &one, 700000);
    if (wkc <= 0) { LOG_ERROR(strf("A6Drive[%d]: Failed set SM2 count", slaveIndex)); ok = false; }

    uint16_t tpdo = 0x1B02;
    wkc = ecx_SDOwrite(ctx, (uint16)slaveIndex, 0x1C13, 0x01, FALSE, sizeof(tpdo), &tpdo, 700000);
    if (wkc <= 0) { LOG_ERROR(strf("A6Drive[%d]: Failed assign TPDO 1B02h", slaveIndex)); ok = false; }

    wkc = ecx_SDOwrite(ctx, (uint16)slaveIndex, 0x1C13, 0x00, FALSE, sizeof(one), &one, 700000);
    if (wkc <= 0) { LOG_ERROR(strf("A6Drive[%d]: Failed set SM3 count", slaveIndex)); ok = false; }

    if (ok)
        LOG_INFO(strf("A6Drive[%d]: PDO config OK (RPDO=1702h TPDO=1B02h).", slaveIndex));
    else
        LOG_WARNING(strf("A6Drive[%d]: PDO config errors.", slaveIndex));
    return ok;
#endif
}

bool A6Drive::stepEnableStateMachine()
{
    if (!m_pControlword || !m_pStatusword) return false;
    updateStatus();

    // Reset sync state whenever the drive leaves SwitchedOn. Without this,
    // the sync restarts on every cycle after the drive responds to 0x0F
    // (drive briefly shows SwitchedOn again due to PDO round-trip lag),
    // causing the hold/advance pattern to loop forever.
    if (m_state != DriveState::SwitchedOn && m_commandSyncActive)
    {
        m_commandSyncActive = false;
        m_commandSyncCount  = 0;
    }

    switch (m_state)
    {
    case DriveState::NotReadyToSwitchOn:
        m_enableStallCycles = 0;
        break;

    case DriveState::SwitchOnDisabled:
        writeControlwordRaw(
            DS402Bits::CW_ENABLE_VOLTAGE | DS402Bits::CW_QUICK_STOP);
        m_enableStallCycles = 0;
        break;

    case DriveState::ReadyToSwitchOn:
        writeControlwordRaw(
            DS402Bits::CW_SWITCH_ON | DS402Bits::CW_ENABLE_VOLTAGE | DS402Bits::CW_QUICK_STOP);
        m_enableStallCycles = 0;
        break;

    case DriveState::SwitchedOn:
    {
        // Command-position sync.
        // Hold at 0x07 for m_commandSyncCycles cycles while writing target PDO
        // = actual PDO. This synchronizes the drive's internal command-position
        // register with the encoder reading BEFORE the 0x0F snapshot that can
        // trip following-error if command != actual by >0x6065 (100mm window).
        if (!m_commandSyncActive)
        {
            m_commandSyncActive = true;
            m_commandSyncCount = 0;
            m_commandSyncInitialPos = getActualPositionRaw();
            RT_DIAG("DIAG | command_sync | drive=%d | cycles=%d | initial_actual=%.3f | phase=start",
                m_slaveIndex, m_commandSyncCycles, m_commandSyncInitialPos);
        }
        // Write target = actual every sync cycle (direct counts, no offset/clamping)
        if (m_pTargetPosition)
            *m_pTargetPosition = m_lastActualCounts;

        if (m_commandSyncCount < m_commandSyncCycles)
        {
            // Hold phase: keep CW=0x07, accumulate sync cycles
            writeControlwordRaw(
                DS402Bits::CW_SWITCH_ON | DS402Bits::CW_ENABLE_VOLTAGE | DS402Bits::CW_QUICK_STOP);
            ++m_commandSyncCount;
            if (m_commandSyncCount == m_commandSyncCycles)
            {
                // Log completion once, on the cycle that fills the count
                double finalPos = getActualPositionRaw();
                double drift = std::abs(finalPos - m_commandSyncInitialPos);
                RT_DIAG("DIAG | command_sync | drive=%d | cycles=%d | initial_actual=%.3f | final_actual=%.3f | result=%s",
                    m_slaveIndex, m_commandSyncCycles, m_commandSyncInitialPos,
                    finalPos, drift > 1.0 ? "drift" : "ok");
            }
        }
        else
        {
            // Post-sync phase: keep requesting OperationEnabled every cycle until
            // the drive acknowledges (state leaves SwitchedOn). Do NOT reset
            // m_commandSyncActive here - the reset above (before the switch)
            // handles it when m_state transitions away from SwitchedOn.
            writeControlwordRaw(
                DS402Bits::CW_SWITCH_ON | DS402Bits::CW_ENABLE_VOLTAGE |
                DS402Bits::CW_QUICK_STOP | DS402Bits::CW_ENABLE_OPERATION);
        }
        m_enableStallCycles = 0;
        break;
    }

    case DriveState::OperationEnabled:
        writeControlwordRaw(
            DS402Bits::CW_SWITCH_ON |
            DS402Bits::CW_ENABLE_VOLTAGE |
            DS402Bits::CW_QUICK_STOP |
            DS402Bits::CW_ENABLE_OPERATION);
        return true;

    case DriveState::QuickStopActive:
        // Hardware e-stop is active. Do NOT write a controlword trying to
        // exit QuickStopActive -- the physical button holds this state regardless.
        // ControlLoop detects isQuickStopActive() and triggers software e-stop.
        // When button is released, drive self-transitions to SwitchOnDisabled.
        m_enableStallCycles = 0;
        break;

    case DriveState::FaultReactionActive:
        m_enableStallCycles = 0;
        break;

    case DriveState::Fault:
        break;

    case DriveState::Unknown:
    default:
        if (++m_enableStallCycles % ENABLE_STALL_MAX == 0)
            RT_LOG_WARNING("A6Drive[%d]: State machine stuck in Unknown for %d cycles, SW=0x%04x",
                m_slaveIndex, m_enableStallCycles, m_lastStatusword);
        break;
    }
    return false;
}

bool A6Drive::enterCSPMode()
{
    if (!m_pModeOfOperation) return false;
    *m_pModeOfOperation = static_cast<int8_t>(ModeOfOperation::CyclicSyncPos);
    LOG_INFO(strf("A6Drive[%d]: CSP mode set.", m_slaveIndex));
    return true;
}

void A6Drive::enableOperation()
{
    stepEnableStateMachine();
}

void A6Drive::prepareForPump()
{
    // 0x0006 = Enable Voltage (bit 1) + Quick Stop (bit 2). DS402 "Shutdown" command:
    // requests "Ready to Switch On" state - drive not enabled, no motion. Correct
    // neutral state to hold throughout the pre-OP and OP pumps before the control
    // loop's first enableOperation() call.
    writeControlword(DriveCommand::Shutdown);
}

void A6Drive::disableOperation()
{
    if (!m_pControlword) return;
    writeControlwordRaw(DS402Bits::CW_ENABLE_VOLTAGE | DS402Bits::CW_SWITCH_ON);
}

void A6Drive::startFaultReset()
{
    if (!m_faultResetPending)
    {
        m_faultResetPending = true;
        m_faultResetCycles = 0;
        m_faultResetPulseHigh = false;
        RT_LOG_INFO("A6Drive[%d]: Fault reset sequence started.", m_slaveIndex);
    }
}

bool A6Drive::stepFaultReset()
{
    if (!m_pControlword || !m_pStatusword) return false;

    if (!m_faultResetPending)
        startFaultReset();

    updateStatus();
    ++m_faultResetCycles;

    if (m_faultResetCycles <= 5)
    {
        writeControlwordRaw(0x0000);
        return false;
    }

    if (m_faultResetCycles <= 15)
    {
        writeControlwordRaw(DS402Bits::CW_FAULT_RESET);
        return false;
    }

    if (m_state != DriveState::Fault && m_state != DriveState::FaultReactionActive)
    {
        m_faultResetPending = false;
        m_faultResetCycles = 0;
        RT_LOG_INFO("A6Drive[%d]: Fault cleared -- re-enabling. New state=%d",
            m_slaveIndex, static_cast<int>(m_state));
        return stepEnableStateMachine();
    }

    if (m_faultResetCycles > 100)
    {
        RT_LOG_ERROR("A6Drive[%d]: Fault reset FAILED after %d cycles. SW=0x%04x. Manual intervention required.",
            m_slaveIndex, m_faultResetCycles, m_lastStatusword);
        m_faultResetPending = false;
        m_faultResetCycles = 0;
        return false;
    }

    if (m_faultResetCycles % 20 < 10)
        writeControlwordRaw(DS402Bits::CW_FAULT_RESET);
    else
        writeControlwordRaw(0x0000);

    return false;
}

uint16_t A6Drive::readErrorCode(ecx_contextt* ctx)
{
#ifndef SOEM_AVAILABLE
    return 0;
#else
    if (!ctx) return 0;
    uint16_t errorCode = 0;
    int size = sizeof(errorCode);
    int wkc = ecx_SDOread(ctx, (uint16)m_slaveIndex, 0x603F, 0x00,
        FALSE, &size, &errorCode, 700000);
    if (wkc > 0)
    {
        LOG_ERROR(strf("A6Drive[%d]: Error code from 0x603F = 0x%04x", m_slaveIndex, errorCode));
        return errorCode;
    }
    LOG_WARNING(strf("A6Drive[%d]: Could not read error code (wkc=%d)", m_slaveIndex, wkc));
    return 0;
#endif
}

void A6Drive::startHomingMode(ModeOfOperation mode, ecx_contextt* ctx)
{
    if (!m_pModeOfOperation || !m_pControlword) return;

    *m_pModeOfOperation = static_cast<int8_t>(mode);

    if (mode == ModeOfOperation::SetHomeHere)
    {
        writeControlwordRaw(
            DS402Bits::CW_SWITCH_ON |
            DS402Bits::CW_ENABLE_VOLTAGE |
            DS402Bits::CW_QUICK_STOP |
            DS402Bits::CW_ENABLE_OPERATION |
            DS402Bits::CW_HOMING_START);
        RT_LOG_INFO("A6Drive[%d]: Mode 35 set-home-here triggered.", m_slaveIndex);
    }
    else if (mode == ModeOfOperation::TorqueHoming)
    {
        writeControlwordRaw(
            DS402Bits::CW_SWITCH_ON |
            DS402Bits::CW_ENABLE_VOLTAGE |
            DS402Bits::CW_QUICK_STOP |
            DS402Bits::CW_ENABLE_OPERATION |
            DS402Bits::CW_HOMING_START);
        RT_LOG_INFO("A6Drive[%d]: Torque homing (mode -2) started.", m_slaveIndex);
    }
    else if (mode == ModeOfOperation::Homing)
    {
        writeControlwordRaw(
            DS402Bits::CW_SWITCH_ON |
            DS402Bits::CW_ENABLE_VOLTAGE |
            DS402Bits::CW_QUICK_STOP |
            DS402Bits::CW_ENABLE_OPERATION |
            DS402Bits::CW_HOMING_START);
        RT_LOG_INFO("A6Drive[%d]: Standard homing (mode 6) started.", m_slaveIndex);
    }

    (void)ctx;
}

bool A6Drive::isHomingAttained() const
{
    return (m_lastStatusword & DS402Bits::SW_HOMING_ATTAINED) != 0;
}

bool A6Drive::isHomingError() const
{
    return (m_lastStatusword & DS402Bits::SW_HOMING_ERROR) != 0;
}

void A6Drive::stopHoming()
{
    if (!m_pControlword) return;
    writeControlwordRaw(
        DS402Bits::CW_SWITCH_ON |
        DS402Bits::CW_ENABLE_VOLTAGE |
        DS402Bits::CW_QUICK_STOP |
        DS402Bits::CW_ENABLE_OPERATION);
}

// ============================================================
// setTargetPosition -- WITH home offset, clamping, rate limiting
// Used during normal operation (after homing completes).
// ============================================================
void A6Drive::setTargetPosition(double engineeringUnits)
{
    if (!m_pTargetPosition) return;

    // Apply home offset: caller's 0.0 = backoff position
    double absolute = engineeringUnits + m_homeOffset;

    // Clamp to configured limits
    double clamped = std::max(m_minPos, std::min(m_maxPos, absolute));
    if (clamped != absolute)
    {
        static int clampLogCount = 0;
        if (++clampLogCount <= 10 || clampLogCount % 1000 == 0)
            RT_LOG_WARNING("A6Drive[%d]: Position CLAMPED: requested=%.3f clamped=%.3f (limits [%.3f, %.3f])",
                m_slaveIndex, absolute, clamped, m_minPos, m_maxPos);
    }

    // Rate limiter: reject jumps larger than maxVelocity * cycleTime
    if (m_lastTargetValid)
    {
        double delta = std::abs(clamped - m_lastTargetUnits);
        if (delta > m_maxDeltaPerCycle * 2.0)  // 2x margin for filter transients
        {
            static int rateLimitLogCount = 0;
            if (++rateLimitLogCount <= 10 || rateLimitLogCount % 1000 == 0)
                RT_LOG_WARNING("A6Drive[%d]: Position RATE LIMITED: delta=%.4f max=%.4f units/cycle",
                    m_slaveIndex, delta, m_maxDeltaPerCycle);

            // Limit to max step toward target
            double sign = (clamped > m_lastTargetUnits) ? 1.0 : -1.0;
            clamped = m_lastTargetUnits + sign * m_maxDeltaPerCycle * 2.0;
        }
    }

    m_lastTargetUnits = clamped;
    m_lastTargetValid = true;

    *m_pTargetPosition = unitsToCount(clamped);

    // PP mode: set change-set-immediately (bit 5) and toggle new-setpoint (bit 4)
    // each cycle so the drive continuously updates its internal profile target.
    if (m_ppMode && m_pControlword)
    {
        m_ppNewSetpoint = !m_ppNewSetpoint;
        uint16_t cw = *m_pControlword;
        // Bit 5: change set immediately (always set in cyclic PP)
        cw |= (1u << 5);
        // Bit 4: new setpoint -- toggled each cycle
        if (m_ppNewSetpoint)
            cw |= (1u << 4);
        else
            cw &= ~(1u << 4);
        *m_pControlword = cw;
    }
}

// ============================================================
// setTargetTorque -- torque mode only
// pct is % of motor RATED torque (DS402 0x6071 is per-mille of rated, so x10):
// 100% = rated, up to ~300% = peak. The drive's own 0x6072 (max torque) is the HARD
// ceiling -- it clamps 0x6071 internally regardless of what we command -- so this app
// clamp is only a sanity bound. Clamping at 100 would silently cap the belt at rated
// and make peak unreachable, so allow up to peak and let 0x6072 protect.
// ============================================================
void A6Drive::setTargetTorque(double pct)
{
    if (!m_pTargetTorque) return;
    constexpr double MAX_CMD_TORQUE_PCT = 300.0;   // peak; real ceiling is drive 0x6072
    double clamped = std::max(-MAX_CMD_TORQUE_PCT, std::min(MAX_CMD_TORQUE_PCT, pct));
    *m_pTargetTorque = static_cast<int16_t>(clamped * 10.0);

    // Write CST mode (10) via PDO each cycle.
    // 6060h is in 1702h RPDO at out+12 -- mode is set here, not via SDO.
    if (m_pModeOfOperation)
        *m_pModeOfOperation = 10;

    // 0x607F (max profile velocity) is ALSO part of the 1702h RPDO (out+15).
    // ZERO pins the shaft, so a beltMaxRpm-derived nonzero value is commanded
    // each cycle to hold that invariant -- but a nonzero 0x607F does NOT
    // actually restrain CST speed on the A6. The enforced slack-lunge limit
    // is the MASTER-side velocity fold in MotionController, not this
    // register. (0x8400 here means Er06.0 runaway protection, addressed by
    // C06.20=0 provisioning -- not an overspeed clamp problem.)
    if (m_pMaxProfileVel)
        *m_pMaxProfileVel = m_maxProfileVelCounts;
}

// ============================================================
// setTargetPositionRaw -- NO offset, NO clamping, NO rate limiting
// Used during homing search and backoff when offset isn't set yet.
// PP mode: still needs the new-setpoint handshake or the drive won't move.
// ============================================================
void A6Drive::setTargetPositionRaw(double rawUnits)
{
    if (!m_pTargetPosition) return;
    *m_pTargetPosition = unitsToCount(rawUnits);
    m_lastTargetUnits = rawUnits;
    m_lastTargetValid = true;

    // PP mode: toggle new-setpoint (bit 4) + set change-set-immediately (bit 5)
    // Without this the drive ignores position targets during homing.
    if (m_ppMode && m_pControlword)
    {
        m_ppNewSetpoint = !m_ppNewSetpoint;
        uint16_t cw = *m_pControlword;
        cw |= (1u << 5);
        if (m_ppNewSetpoint) cw |= (1u << 4);
        else                 cw &= ~(1u << 4);
        *m_pControlword = cw;
    }
}

// ============================================================
// getActualPosition -- WITH home offset applied
// Returns position relative to home (0.0 = backoff position)
// ============================================================
double A6Drive::getActualPosition() const
{
    if (!m_pActualPosition)
        return m_simActualPos;
    return countsToUnits(m_lastActualCounts) - m_homeOffset;
}

// ============================================================
// getActualPositionRaw -- NO offset
// Returns raw encoder position in engineering units
// ============================================================
double A6Drive::getActualPositionRaw() const
{
    if (!m_pActualPosition)
        return m_simActualPos;
    return countsToUnits(m_lastActualCounts);
}

void A6Drive::updateStatus()
{
    if (!m_pStatusword || !m_pActualPosition) return;
    // Skip on bad EtherCAT frame -- keep last known good values.
    // A single corrupted or missing frame should never change drive state.
    if (!m_frameValid) return;
    m_lastStatusword = *m_pStatusword;
    m_lastActualCounts = *m_pActualPosition;
    DriveState newState = parseStatusword(m_lastStatusword);
    if (newState != m_state)
    {
        RT_LOG_INFO("A6Drive[%d]: State %d -> %d  (SW=0x%04x)",
            m_slaveIndex, static_cast<int>(m_state), static_cast<int>(newState), m_lastStatusword);
        if (newState == DriveState::Fault)
            RT_LOG_ERROR("A6Drive[%d]: FAULT detected! SW=0x%04x", m_slaveIndex, m_lastStatusword);
        if (newState == DriveState::FaultReactionActive)
            RT_LOG_WARNING("A6Drive[%d]: Fault reaction active. SW=0x%04x", m_slaveIndex, m_lastStatusword);
        m_state = newState;
    }
}

int32_t A6Drive::unitsToCount(double units) const
{
    return static_cast<int32_t>(units * m_countsPerMm);
}

double A6Drive::countsToUnits(int32_t counts) const
{
    if (m_countsPerMm == 0.0) return 0.0;
    return static_cast<double>(counts) / m_countsPerMm;
}

DriveState A6Drive::parseStatusword(uint16_t sw) const
{
    if ((sw & 0x004F) == 0x000F)  return DriveState::FaultReactionActive;
    if (sw & DS402Bits::SW_FAULT) return DriveState::Fault;
    if ((sw & 0x004F) == 0x0000)  return DriveState::NotReadyToSwitchOn;
    if ((sw & 0x006F) == 0x0040)  return DriveState::SwitchOnDisabled;
    if ((sw & 0x006F) == 0x0021)  return DriveState::ReadyToSwitchOn;
    if ((sw & 0x006F) == 0x0023)  return DriveState::SwitchedOn;
    if ((sw & 0x006F) == 0x0027)  return DriveState::OperationEnabled;
    if ((sw & 0x0067) == 0x0007)  return DriveState::QuickStopActive;
    return DriveState::Unknown;
}

void A6Drive::writeControlword(DriveCommand cmd)
{
    if (m_pControlword)
        *m_pControlword = static_cast<uint16_t>(cmd);
}

void A6Drive::writeControlwordRaw(uint16_t cw)
{
    if (m_pControlword)
        *m_pControlword = cw;
}
