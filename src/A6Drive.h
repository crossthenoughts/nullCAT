// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// A6Drive.h
//
// DS402 servo drive interface: enable/fault state machines, homing,
// software home offset, position clamping and rate limiting, and
// torque (CST) support.
// ============================================================

#include <cstdint>
#include <string>
#include <atomic>

struct ecx_context;
typedef struct ecx_context ecx_contextt;

enum class DriveState
{
    Unknown = -1,
    NotReadyToSwitchOn = 0,
    SwitchOnDisabled = 1,
    ReadyToSwitchOn = 2,
    SwitchedOn = 3,
    OperationEnabled = 4,
    QuickStopActive = 5,
    FaultReactionActive = 6,
    Fault = 7
};

enum class DriveCommand : uint16_t
{
    Shutdown = 0x0006,
    SwitchOn = 0x0007,
    EnableOperation = 0x000F,
    DisableVoltage = 0x0000,
    QuickStop = 0x0002,
    FaultReset = 0x0080,
    HomingStart = 0x001F,
    HomingStop = 0x000F
};

enum class ModeOfOperation : int8_t
{
    ProfilePosition = 1,
    ProfileVelocity = 3,
    ProfileTorque = 4,
    Homing = 6,
    CyclicSyncPos = 8,
    CyclicSyncVel = 9,
    CyclicSyncTorque = 10,
    TorqueHomingRev = -1,
    TorqueHoming = -2,
    SetHomeHere = 35
};

namespace DS402Bits
{
    static constexpr uint16_t SW_READY_TO_SWITCH_ON = 0x0001;
    static constexpr uint16_t SW_SWITCHED_ON = 0x0002;
    static constexpr uint16_t SW_OPERATION_ENABLED = 0x0004;
    static constexpr uint16_t SW_FAULT = 0x0008;
    static constexpr uint16_t SW_VOLTAGE_ENABLED = 0x0010;
    static constexpr uint16_t SW_QUICK_STOP = 0x0020;
    static constexpr uint16_t SW_SWITCH_ON_DISABLED = 0x0040;
    static constexpr uint16_t SW_WARNING = 0x0080;
    static constexpr uint16_t SW_HOMING_ATTAINED = 0x1000;
    static constexpr uint16_t SW_HOMING_ERROR = 0x2000;

    static constexpr uint16_t CW_SWITCH_ON = 0x0001;
    static constexpr uint16_t CW_ENABLE_VOLTAGE = 0x0002;
    static constexpr uint16_t CW_QUICK_STOP = 0x0004;
    static constexpr uint16_t CW_ENABLE_OPERATION = 0x0008;
    static constexpr uint16_t CW_HOMING_START = 0x0010;
    static constexpr uint16_t CW_FAULT_RESET = 0x0080;
}

class A6Drive
{
public:
    A6Drive();
    // Virtual destructor required for mock subclassing in tests
    virtual ~A6Drive() = default;

    void setSlaveIndex(int slaveIndex);
    virtual int  getSlaveIndex() const { return m_slaveIndex; }
    // Single-arg scaling: one canonical counts/mm value so the host
    // and drive views of the scaling cannot diverge silently.
    void setScaling(double countsPerMm);
    virtual void setLimits(double minPos, double maxPos);
    void setName(const std::string& name) { m_name = name; }
    std::string getName() const { return m_name; }

    // Configure rate limiter (must be called after setScaling)
    virtual void setMaxVelocity(double maxVelocityUnitsPerSec, double cycleTimeSec);

    void setPDOPointers(
        uint16_t* controlword,
        int8_t* modeOfOperation,
        int32_t* targetPosition,
        uint16_t* statusword,
        int8_t* modeOfOpDisplay,
        int32_t* actualPosition,
        int16_t* torqueFeedback = nullptr
    );

    // Null out all PDO pointers before ecx_close() frees the iomap.
    // Must be called in EtherCATMaster::shutdown() after stopPump() so that
    // no RT thread is accessing the pointers when they are cleared.
    void clearPDOPointers()
    {
        m_pControlword     = nullptr;
        m_pModeOfOperation = nullptr;
        m_pTargetPosition  = nullptr;
        m_pStatusword      = nullptr;
        m_pModeOfOpDisplay = nullptr;
        m_pActualPosition  = nullptr;
        m_pTorqueFeedback  = nullptr;
        m_pTargetTorque    = nullptr;
        m_pMaxProfileVel   = nullptr;
        m_pFaultCode       = nullptr;
    }

    bool configurePDOs(ecx_contextt* ctx = nullptr);
    static bool configurePDOsStatic(int slaveIndex, ecx_contextt* ctx);

    virtual bool stepEnableStateMachine();
    bool enterCSPMode();
    void enableOperation();
    void disableOperation();

    virtual bool stepFaultReset();
    bool isFaultResetPending() const { return m_faultResetPending; }
    void startFaultReset();
    uint16_t readErrorCode(ecx_contextt* ctx);

    // Precise fault identity: SDO-read 0x203F (UInt32; low 16 bits = the
    // panel/Er code, e.g. 0x871 = Er87.1). Blocking mailbox transaction --
    // recovery thread only, NEVER the RT loop. Returns 0 on failure.
    // The last successful read is cached for the UI (atomic: written by the
    // recovery thread, read by web/UI threads).
    uint32_t readPanelCode(ecx_contextt* ctx);
    uint32_t getPanelCode() const { return m_panelCode.load(std::memory_order_acquire); }
    void     clearPanelCode()     { m_panelCode.store(0, std::memory_order_release); }

    // Homing mode -- retained for compatibility but mode 35 should NOT
    // be used (it can't return to CSP during operation).
    void startHomingMode(ModeOfOperation mode, ecx_contextt* ctx = nullptr);
    bool isHomingAttained()  const;
    bool isHomingError()     const;
    void stopHoming();

    // ---- Software home frame (offset + direction) ----
    // After homing, call setHomeOffset() with the raw position at the backoff
    // point and the frame sign. The engineering frame is:
    //     raw = homeOffset + frameSign * engineering
    //     eng = frameSign * (raw - homeOffset)
    // frameSign is chosen by homing as the OPPOSITE of the search direction, so
    // engineering position 0.0 is the backoff point and engineering ALWAYS
    // increases away from the homed stop -- regardless of which raw direction
    // the search ran. This is the single place that invariant lives; nothing
    // above this class may reason about raw direction. (Before 0.9.2 the sign
    // did not exist: the frame silently assumed a raw-negative search, and a
    // positive-direction home produced a frame pointing INTO the hardstop --
    // homing succeeded, then unpark drove through the stop.)
    virtual void   setHomeOffset(double offsetUnits, double frameSign = 1.0)
    {
        m_homeOffset = offsetUnits;
        m_frameSign  = (frameSign < 0.0) ? -1.0 : 1.0;
        m_homeOffsetSet = true;
    }
    virtual double getHomeOffset() const { return m_homeOffset; }
    virtual double getFrameSign()  const { return m_frameSign; }
    virtual bool   isHomeOffsetSet() const { return m_homeOffsetSet; }
    virtual void   clearHomeOffset() { m_homeOffset = 0.0; m_frameSign = 1.0; m_homeOffsetSet = false; }

    // Position access WITH offset applied (normal operation)
    virtual void   setTargetPosition(double engineeringUnits);
    virtual double getActualPosition() const;

    // Position access WITHOUT offset (used during homing search/backoff)
    virtual void   setTargetPositionRaw(double rawUnits);
    virtual double getActualPositionRaw() const;

    void setPPMode(bool pp) { m_ppMode = pp; m_ppNewSetpoint = false; }
    bool isPPMode() const { return m_ppMode; }

    // Configure command-position sync cycles.
    // stepEnableStateMachine() holds at 0x07 (SwitchedOn) for this many cycles,
    // writing target PDO = actual PDO each cycle, before transitioning to 0x0F.
    void setCommandSyncCycles(int n) { m_commandSyncCycles = (n > 0) ? n : 1; }
    int  getCommandSyncCycles() const { return m_commandSyncCycles; }

    void setTorqueMode(bool t) { m_torqueMode = t; }
    bool isTorqueMode() const { return m_torqueMode; }
    // Fault code 0x603F: first entry of the 1B01h TPDO (in+0), live PDO read like
    // torque feedback. Logged at fault detection so the panel Er-code is identifiable
    // from the log alone (manual fault table maps 603F <-> Er; e.g. 0x8700=sync/Er74.x,
    // overload family = Er40/41).
    void setFaultCodePDOPointer(uint16_t* p) { m_pFaultCode = p; }
    uint16_t getFaultCode() const { return m_pFaultCode ? *m_pFaultCode : 0; }

    // Torque (1702h) extras: target torque 0x6071 (out+10) AND max profile velocity
    // 0x607F (out+15). 0x607F is PART of the 1702h RPDO, so leaving it unset commands
    // "max velocity = 0" every cycle -- the drive makes torque but permits zero motion
    // (shaft pinned static). setTargetTorque() re-writes it each cycle alongside the
    // 6060h mode reinforce.
    void setTorquePDOPointer(int16_t* p, uint32_t* maxVel = nullptr)
    { m_pTargetTorque = p; m_pMaxProfileVel = maxVel; }

    // 0x607F value commanded each cycle in CST. ZERO pins the shaft
    // (0x607F is part of the 1702h RPDO, so leaving it unset commands
    // "max velocity = 0"); NONZERO values do NOT restrain CST speed on
    // the A6. The enforced speed limit is the MASTER-side velocity fold
    // (MotionController::beltVelocityFold). A beltMaxRpm-derived value is
    // still commanded each cycle: it documents intent and guarantees the
    // never-zero invariant. 0x80000000 = drive-native unlimited.
    // (An 0x8400 fault here is Er06.0 RUNAWAY PROTECTION, not overspeed --
    // addressed by C06.20=0 in torque-drive provisioning.)
    void setMaxProfileVelocityCounts(uint32_t v) { m_maxProfileVelCounts = v; }
    void setTargetTorque(double pct);  // clamped ±300% (0.1%-of-rated DS402 units;
                                       // the drive's 0x6072 is the hard ceiling)

    void   setSimPosition(double pos) { m_simActualPos = pos; }
    void   setSimTarget(double pos) { m_simTargetPos = pos; }
    double getSimTarget() const { return m_simTargetPos; }

    // Set once per RT cycle by ControlLoop after sendReceive().
    // When false, updateStatus() is a no-op -- last known good PDO values
    // are retained so a single bad frame cannot corrupt the state machine
    // or produce a garbage actual position that triggers ER87.x.
    void setFrameValid(bool valid) { m_frameValid = valid; }

    // Write DS402 Shutdown controlword (0x0006) before cyclic exchange starts.
    // Prevents the zeroed iomap from sending 0x0000 (Quick Stop) to the drive during
    // the pre-OP pump. Safe to call any time after setPDOPointers(); no-op if pointer
    // is null (simulation mode or pointers not yet assigned).
    void prepareForPump();

    virtual void   updateStatus();

    // Torque feedback accessor
    // 0x6077 returns torque in 0.1% units, so we divide by 10.
    virtual double getTorquePercent() const
    {
        if (!m_pTorqueFeedback) return 0.0;
        return static_cast<double>(*m_pTorqueFeedback) / 10.0;
    }

    virtual int16_t getTorqueRaw() const
    {
        if (!m_pTorqueFeedback) return 0;
        return *m_pTorqueFeedback;
    }

    virtual DriveState getState()          const { return m_state; }
    virtual bool       isEnabled()         const { return m_state == DriveState::OperationEnabled; }
    virtual bool       isFault()           const { return m_state == DriveState::Fault; }
    virtual bool       isQuickStopActive() const { return m_state == DriveState::QuickStopActive; }
    bool       isInFaultReaction() const { return m_state == DriveState::FaultReactionActive; }
    virtual uint16_t   getStatusword()  const { return m_lastStatusword; }
    virtual int32_t    getActualCounts()const { return m_lastActualCounts; }
    virtual int8_t     getModeDisplay() const { return m_pModeOfOpDisplay ? *m_pModeOfOpDisplay : 0; }

    double getCountsPerMm() const { return m_countsPerMm; }
    // Legacy alias kept for callers that read "counts per unit".
    // Engineering units == mm for linear axes (the only kind in use).
    double getCountsPerUnit() const { return m_countsPerMm; }

    int32_t unitsToCount(double units) const;
    double  countsToUnits(int32_t counts) const;

private:
    int         m_slaveIndex = 1;
    std::string m_name = "A6Drive";

    double m_simActualPos = 0.0;
    double m_simTargetPos = 0.0;
    // Single scaling value -- counts per engineering unit (mm).
    // Default 13107.2 = 131072 counts/rev / 10mm pitch, matches the
    // factory-default rig. Set by setScaling() at init.
    double m_countsPerMm = 13107.2;
    double m_minPos = -180.0;
    double m_maxPos = 180.0;

    // Software home offset
    double m_homeOffset = 0.0;
    double m_frameSign  = 1.0;   // +1: eng increases with raw; -1: reversed (positive-direction home)
    bool   m_homeOffsetSet = false;

    // Rate limiter
    double m_maxDeltaPerCycle = 1e9;   // effectively unlimited until configured
    double m_lastTargetUnits = 0.0;
    bool   m_lastTargetValid = false;

    uint16_t* m_pControlword = nullptr;
    int8_t* m_pModeOfOperation = nullptr;
    int32_t* m_pTargetPosition = nullptr;
    uint16_t* m_pStatusword = nullptr;
    int8_t* m_pModeOfOpDisplay = nullptr;
    int32_t* m_pActualPosition = nullptr;
    int16_t* m_pTorqueFeedback = nullptr;   // 0x6077

    DriveState m_state = DriveState::Unknown;
    uint16_t   m_lastStatusword = 0;
    int32_t    m_lastActualCounts = 0;
    bool       m_frameValid = true;   // set by ControlLoop per-cycle via setFrameValid()

    int  m_faultResetCycles = 0;
    bool m_faultResetPending = false;
    bool m_faultResetPulseHigh = false;

    int  m_enableStallCycles = 0;
    static constexpr int ENABLE_STALL_MAX = 500;

    // Command-position sync state
    int    m_commandSyncCycles     = 10;
    int    m_commandSyncCount      = 0;
    bool   m_commandSyncActive     = false;
    double m_commandSyncInitialPos = 0.0;

    bool m_ppMode = false;
    bool m_ppNewSetpoint = false;  // toggled each cycle in PP mode

    bool     m_torqueMode = false;
    int16_t*  m_pTargetTorque  = nullptr;  // 0x6071, only valid when torque PDO remap succeeded
    uint32_t* m_pMaxProfileVel = nullptr;  // 0x607F in 1702h -- MUST be written (0 = motion locked)
    uint32_t  m_maxProfileVelCounts = 0x80000000u;  // 0x607F value (counts/s); see setter
    uint16_t* m_pFaultCode     = nullptr;  // 0x603F, in+0 of 1B01h (both layouts) -- live read
    std::atomic<uint32_t> m_panelCode{0};  // last 0x203F read (low 16 = Er panel code)

    DriveState parseStatusword(uint16_t sw) const;
    void       writeControlword(DriveCommand cmd);
    void       writeControlwordRaw(uint16_t cw);
};
