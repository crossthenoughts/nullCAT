// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// MockA6Drive.h  (test infrastructure)
//
// Simulates A6Drive behaviour without hardware or SOEM.
// Inherits A6Drive so it can be passed as A6Drive* anywhere.
//
// Simulation model:
//   - DS402 enable state machine: configurable cycles to reach
//     OperationEnabled (default 3).
//   - Position: actual position moves toward target at
//     configurable max mm/cycle (default 0.05).
//   - Mechanical hardstop: clamps actual position, reports a
//     configurable torque % while pushing against it.
//   - Fault injection: injectFault() / clearFault().
//
// Test setup: call configure() to set axis parameters, then
// call setHardstop() and injectFault() as needed from tests.
// ============================================================

#include "A6Drive.h"
#include <cmath>
#include <algorithm>
#include <string>

class MockA6Drive : public A6Drive
{
public:
    MockA6Drive()
    {
        setSlaveIndex(1);
    }

    // ---- Test configuration API ----

    // Call before starting a test.
    // startPos: where the drive is at power-on (raw mm).
    // maxVelPerCycle: max mm movement per step() call.
    void configure(int slaveIdx, double startPos = 0.0, double maxVelPerCycle = 0.05)
    {
        setSlaveIndex(slaveIdx);
        m_actualPos       = startPos;
        m_targetPos       = startPos;
        m_maxVelPerCycle  = maxVelPerCycle;
        m_simState        = DriveState::SwitchOnDisabled;
        m_faultInjected   = false;
        m_qsaInjected     = false;
        m_enableStep      = 0;
        m_faultResetStep  = 0;
        m_hardstopActive        = false;
        m_mockHomeOffset        = 0.0;
        m_mockFrameSign         = 1.0;
        m_mockHomeOffsetSet     = false;
        m_fraInjected           = false;
        m_useIntermediateStates = false;
        m_stepsPerState         = 1;
    }

    // Set a mechanical hardstop.  isMinLimit=true means the stop is at a
    // lower position (homing in negative direction). torquePct reported
    // while pushing against it (default 50 -- well above any threshold).
    void setHardstop(double pos, bool isMinLimit = true, double torquePct = 50.0)
    {
        m_hardstopActive  = true;
        m_hardstopPos     = pos;
        m_hardstopIsMin   = isMinLimit;
        m_hardstopTorque  = torquePct;
    }

    void clearHardstop()
    {
        m_hardstopActive = false;
    }

    // Inject a drive fault. Drive immediately transitions to Fault state
    // on the next updateStatus() call and stays there until clearFault().
    void injectFault()  { m_faultInjected = true; m_fraInjected = false; }
    void clearFault()   { m_faultInjected = false; m_fraInjected = false; m_simState = DriveState::SwitchOnDisabled; m_enableStep = 0; }

    // Inject FaultReactionActive (transient pre-Fault DS402 state).
    // Drive stays in FaultReactionActive until injectFault() or clearFault().
    void injectFaultReactionActive() { m_fraInjected = true; m_faultInjected = false; m_simState = DriveState::FaultReactionActive; }

    // Configure the enable state machine to pass through DS402 intermediate
    // states (NotReadyToSwitchOn → ReadyToSwitchOn → SwitchedOn → OperationEnabled).
    // When enabled, each step advances one state; total cycles = 4 * stepsPerState.
    void setIntermediateStates(bool enable, int stepsPerState = 1)
    {
        m_useIntermediateStates = enable;
        m_stepsPerState = (stepsPerState > 0) ? stepsPerState : 1;
    }

    // Inject/clear hardware QuickStopActive (physical e-stop button).
    void injectQuickStop() { m_qsaInjected = true;  m_simState = DriveState::QuickStopActive; }
    void clearQuickStop()  { m_qsaInjected = false; m_simState = DriveState::SwitchOnDisabled; m_enableStep = 0; }

    // How many stepEnableStateMachine() calls until OperationEnabled.
    void setEnableCycles(int n) { m_enableCyclesToComplete = n; }

    // Direct position read (for test assertions).
    double getSimActualPos() const { return m_actualPos; }

    // ---- A6Drive virtual overrides ----

    // Called once per HomingSequence::step() -- this is the simulation tick.
    void updateStatus() override
    {
        if (m_faultInjected)
        {
            m_simState = DriveState::Fault;
            return;
        }
        if (m_fraInjected)
        {
            m_simState = DriveState::FaultReactionActive;
            return;
        }
        if (m_qsaInjected)
        {
            m_simState = DriveState::QuickStopActive;
            return;
        }

        // Move actual position toward target, clamped to maxVelPerCycle.
        double delta = m_targetPos - m_actualPos;
        if (std::abs(delta) > 1e-9)
        {
            double step = std::min(std::abs(delta), m_maxVelPerCycle);
            m_actualPos += (delta > 0.0 ? step : -step);
        }

        // Apply hardstop: clamp actual position at the physical limit.
        if (m_hardstopActive)
        {
            if (m_hardstopIsMin)
                m_actualPos = std::max(m_actualPos, m_hardstopPos);
            else
                m_actualPos = std::min(m_actualPos, m_hardstopPos);
        }
    }

    // DS402 enable state machine -- returns true when OperationEnabled.
    bool stepEnableStateMachine() override
    {
        if (m_faultInjected || m_simState == DriveState::Fault)
            return false;
        if (m_fraInjected || m_simState == DriveState::FaultReactionActive)
            return false;
        if (m_qsaInjected || m_simState == DriveState::QuickStopActive)
            return false;  // hold still, don't try to exit QSA

        if (m_useIntermediateStates)
        {
            // Step through DS402 states one step at a time.
            // Each state requires m_stepsPerState calls before advancing.
            ++m_enableStep;
            int stateIndex = (m_enableStep - 1) / m_stepsPerState;
            switch (stateIndex)
            {
                case 0: m_simState = DriveState::SwitchOnDisabled; break;
                case 1: m_simState = DriveState::ReadyToSwitchOn;  break;
                case 2: m_simState = DriveState::SwitchedOn;       break;
                default:
                    m_simState = DriveState::OperationEnabled;
                    return true;
            }
            return false;
        }

        ++m_enableStep;
        if (m_enableStep >= m_enableCyclesToComplete)
        {
            m_simState = DriveState::OperationEnabled;
            return true;
        }
        return false;
    }

    // Fault reset -- clears fault after a few cycles.
    bool stepFaultReset() override
    {
        ++m_faultResetStep;
        if (m_faultResetStep >= 3)
        {
            m_faultInjected  = false;
            m_simState       = DriveState::SwitchOnDisabled;
            m_faultResetStep = 0;
            m_enableStep     = 0;
            return true;
        }
        return false;
    }

    // ---- Position (raw, used during homing) ----

    double getActualPositionRaw() const override { return m_actualPos; }
    void   setTargetPositionRaw(double pos) override { m_targetPos = pos; }

    // ---- Position (with home frame, used post-homing) ----
    // Mirrors A6Drive's frame exactly: raw = offset + sign * eng.

    double getActualPosition() const override
    {
        return m_mockFrameSign * (m_actualPos - m_mockHomeOffset);
    }

    void setTargetPosition(double eu) override
    {
        m_targetPos = m_mockHomeOffset + m_mockFrameSign * eu;
    }

    // ---- Home frame ----

    void   setHomeOffset(double offsetUnits, double frameSign = 1.0) override
    {
        m_mockHomeOffset    = offsetUnits;
        m_mockFrameSign     = (frameSign < 0.0) ? -1.0 : 1.0;
        m_mockHomeOffsetSet = true;
    }
    double getHomeOffset()   const override { return m_mockHomeOffset; }
    double getFrameSign()    const override { return m_mockFrameSign; }
    bool   isHomeOffsetSet() const override { return m_mockHomeOffsetSet; }
    void   clearHomeOffset() override { m_mockHomeOffset = 0.0; m_mockFrameSign = 1.0; m_mockHomeOffsetSet = false; }

    // ---- Torque ----

    // Report hardstop torque when actual position is within 0.5mm of stop
    // AND the drive is commanding into the stop.  Zero otherwise.
    // m_simHoldTorque models a STATIC load-holding torque (e.g. a seated driver's
    // weight, ~30-40% on the rig) present the whole time -- the seated-driver
    // homing regression uses it to prove the hardstop detector keys on torque
    // DEVIATION from baseline, not magnitude.
    double getTorquePercent() const override
    {
        if (!m_hardstopActive) return m_simHoldTorque;
        // At hardstop when actual position is clamped there
        bool atStop = (std::abs(m_actualPos - m_hardstopPos) < 0.5);
        // Pushing: target is at or beyond the stop in the stop direction
        bool pushing = m_hardstopIsMin
            ? (m_targetPos <= m_hardstopPos + 0.1)
            : (m_targetPos >= m_hardstopPos - 0.1);
        return m_simHoldTorque + ((atStop && pushing) ? m_hardstopTorque : 0.0);
    }

    void setSimHoldTorque(double pct) { m_simHoldTorque = pct; }

    int16_t getTorqueRaw() const override
    {
        return static_cast<int16_t>(getTorquePercent() * 10.0);
    }

    // ---- Drive state ----

    DriveState getState()             const override { return m_simState; }
    bool       isEnabled()            const override { return m_simState == DriveState::OperationEnabled; }
    bool       isFault()              const override { return m_faultInjected || m_simState == DriveState::Fault; }
    bool       isQuickStopActive()    const override { return m_qsaInjected  || m_simState == DriveState::QuickStopActive; }
    uint16_t   getStatusword() const override
    {
        switch (m_simState)
        {
        case DriveState::OperationEnabled:    return 0x0237;
        case DriveState::Fault:               return 0x0208;
        case DriveState::FaultReactionActive: return 0x000F;  // fault reaction bits
        case DriveState::SwitchOnDisabled:    return 0x0240;
        case DriveState::ReadyToSwitchOn:     return 0x0221;
        case DriveState::SwitchedOn:          return 0x0223;
        case DriveState::QuickStopActive:     return 0x0007;
        default:                              return 0x0000;
        }
    }
    int32_t getActualCounts() const override { return static_cast<int32_t>(m_actualPos * 100.0); }
    int8_t  getModeDisplay()  const override { return 8; }  // CSP

    // setLimits records its calls so tests can assert on them
    void setLimits(double minPos, double maxPos) override {
        m_lastSetLimitsMin = minPos; m_lastSetLimitsMax = maxPos; ++m_setLimitsCallCount;
    }
    double getLastSetLimitsMin()  const { return m_lastSetLimitsMin; }
    double getLastSetLimitsMax()  const { return m_lastSetLimitsMax; }
    int    getSetLimitsCallCount() const { return m_setLimitsCallCount; }
    void   resetLimitsTracker() { m_lastSetLimitsMin = 0.0; m_lastSetLimitsMax = 0.0; m_setLimitsCallCount = 0; }
    void setMaxVelocity(double, double) override {}

private:
    // Simulation state
    DriveState m_simState           = DriveState::SwitchOnDisabled;
    bool       m_faultInjected      = false;
    bool       m_fraInjected        = false;  // FaultReactionActive injection
    bool       m_qsaInjected        = false;  // QuickStopActive injection
    int        m_enableStep         = 0;
    int        m_enableCyclesToComplete = 3;
    int        m_faultResetStep     = 0;
    bool       m_useIntermediateStates = false;  // step through DS402 states
    int        m_stepsPerState         = 1;

    // Position
    double m_actualPos       = 0.0;
    double m_targetPos       = 0.0;
    double m_maxVelPerCycle  = 0.05;  // mm per updateStatus() call

    // Hardstop
    bool   m_hardstopActive  = false;
    double m_hardstopPos     = 0.0;
    bool   m_hardstopIsMin   = true;   // true = stop is a lower bound (negative dir)
    double m_hardstopTorque  = 50.0;   // % to report when pushing against stop
    double m_simHoldTorque   = 0.0;    // static load-holding torque (seated driver model)

    // Home offset (separate from A6Drive's private members -- those aren't accessible)
    double m_mockHomeOffset    = 0.0;
    double m_mockFrameSign     = 1.0;
    bool   m_mockHomeOffsetSet = false;

    // setLimits call tracking (for test assertions)
    double m_lastSetLimitsMin  = 0.0;
    double m_lastSetLimitsMax  = 0.0;
    int    m_setLimitsCallCount = 0;
};
