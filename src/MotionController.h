// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// MotionController - per-axis motion state machine (PARKED / HOMING /
// UNPARKING / BLENDING / ONLINE / PARKING / ESTOPPING), CSP command
// conditioning, belt torque tensioning with runaway guards, and the
// deinit seat-on-the-stop sequence.
//
// Threading contract:
//   - UI thread must call enqueueCommand() - never startHoming() or
//     startPark() directly.
//   - RT thread (ControlLoop) may call startHoming()/startPark()/
//     setNeedsRehome() directly - they execute immediately.
//   - setEmergencyStop(): atomic-only; state transitions happen inside
//     process() on estop edge detection - no m_axisState write from UI.
//   - RT thread publishes a MotionStatus snapshot at the end of each
//     process(); the UI reads getMotionStatus() - no direct access to
//     m_axisState[], m_runtime[], or m_needsRehome from outside the
//     RT thread.
// ============================================================

#include "TelemetryInput.h"
#include "Config.h"
#include "HomingSequence.h"
#include "SpscQueue.h"
#include "CommandConditioner.h"
#include <array>
#include <atomic>
#include <string>
#include <shared_mutex>

// ---- Command types routed through the SPSC queue (UI → RT) ----
struct MotionCommand
{
    enum class Type : uint8_t
    {
        StartHoming,      // intVal = axis index (-1 = all axes)
        StartPark,        // no args
        StartUnpark,      // no args - return parked axes to standby/online
        SlackBelts,       // torque axes only: ease tension to 0 (don/doff); others untouched
        TensionBelts,     // torque axes only: blend 0 -> live tension; others untouched
    };
    Type type   = Type::StartHoming;
    int  intVal = -1;
};

struct MotionOutput
{
    double   positions[MAX_DRIVES] = {};
    double   torques[MAX_DRIVES]   = {};  // ±100%, only valid for torque mode axes
    int      numDrives             = 0;
    bool     emergencyStop         = false;
};

enum class AxisMotionState
{
    HOMING,
    PARKED,
    UNPARKING,
    BLENDING,
    ONLINE,
    PARKING,
    ESTOPPING
};

// Snapshot of UI-visible motion state published by the RT thread at the end of
// each process() cycle. The UI timer reads this via getMotionStatus() under a
// read lock - the UI never touches m_axisState[], m_runtime[], or
// m_needsRehome directly, so there is no data race on them.
struct MotionStatus
{
    AxisMotionState axisState    [MAX_DRIVES] = {};
    std::string     axisStateName[MAX_DRIVES];   // pre-formatted for display
    bool            homed        [MAX_DRIVES] = {};
    double          accelWinPeakMms2[MAX_DRIVES] = {}; // peak WINDOWED commanded accel mm/s^2 (latched; headroom gauge)
    double          accelClipPct [MAX_DRIVES] = {};  // % cycles the Amax accel clamp bound (clip rate)
    double          accelBindPct [MAX_DRIVES] = {};  // % cycles the relative-braking clamp bound
    bool            needsRehome               = true;
    int             numDrives                 = 0;
    // Belt (torque axes) card telemetry
    double          beltCmdPct[MAX_DRIVES]    = {};  // last commanded tension %
    uint8_t         beltGuard [MAX_DRIVES]    = {};  // 0=ok 1=OVERSPEED trip 2=TRAVEL trip 3=relaxer active
};

class MotionController
{
public:
    MotionController();
    void configure(const AppConfig& config);
    void setEmergencyStop(bool active);
    bool isEmergencyStop() const { return m_emergencyStop.load(); }
    void resetFilters();

    void process(const TelemetryData& telemetryData, MotionOutput& output,
                 A6Drive** drives = nullptr, int numDrives = 0,
                 ecx_contextt* ctx = nullptr);

    void startUnpark(A6Drive** drives = nullptr, int numHwDrives = 0);
    void startPark();

    // Belt don/doff (torque axes ONLY; position axes untouched -- session keeps
    // running). Slack = ease tension to 0 (PARKED = zero torque, spool hand-free);
    // Tension = blend 0 -> live tension. RT thread only; UI enqueues the
    // SlackBelts/TensionBelts commands. TensionBelts is refused while e-stopped.
    void slackBelts();
    void tensionBelts();

private:
    // Belt runaway guards (overspeed persistence + net-travel cap), armed in
    // BLENDING and ONLINE. Returns true if tripped: state latched to PARKED
    // (slack; explicit unpark/tension to resume), torque zeroed this cycle.
    bool beltGuardTripped(int i, A6Drive* d, MotionOutput& output);

    // Master-side CST velocity limiter (ODrive-style): folds commanded tension to
    // zero across a band above beltMaxRpm, using the guard's per-cycle rpm
    // measurement. THE enforced speed limit for torque axes -- the drive-side
    // objects (0x607F, C03.47/48) provably do NOT restrain CST speed on the A6
    // (rig: Er06.0/0x8400 overspeed recurred with both active). One-cycle (500us)
    // reaction caps the slack-take-up lunge at ~beltMaxRpm+50 instead of the
    // ~3600rpm drive fault threshold. Deliberately applied AFTER the slew cap
    // (a safety fold must not be rate-limited downward).
    double beltVelocityFold(int i, double tension) const;

public:

    // Deinit-only soft shutdown ("seat on the stop"). Runs the REAL homing search in
    // seat mode (HomingSequence::start(seatMode=true)) on each VERTICAL, non-torque/PP,
    // non-faulted axis, so it presses DOWN onto its endstop exactly as Home does and holds
    // there -- but with NO back-off and NO home reference established (it is parking on the
    // stop, not homing). Homing is NOT required first; faulted/horizontal/belt axes are
    // skipped (they de-energize as today). ControlLoop drives these via stepSeatHoming()
    // each cycle and de-energizes once allSeatAxesAtStop(). Needs live drive pointers.
    // driveSeatable (optional): per-drive eligibility mask from the deinit-seat
    // bus-health probe (only slaves confirmed reachable get seated) -- nullptr =
    // all eligible (healthy bus).
    void startSeatHoming(A6Drive** drives, int numHwDrives,
                         const bool* driveSeatable = nullptr);
    // Per-cycle step of the seat-mode homing for the axes startSeatHoming() selected.
    // Drives HomingSequence::step() directly -- it deliberately bypasses processHomingAxis()
    // so homing's completion side effects (homeOffset / homed / PARKED) never run.
    void stepSeatHoming(A6Drive** drives, int numHwDrives);
    // True once every seat axis has reached the stop (HomingSequence Seated) or terminated
    // (Error/FatalError -- that axis simply de-energizes/drops, accepted). The ControlLoop
    // seat loop waits on this (bounded by a backstop + HomingSequence's own guards).
    bool allSeatAxesAtStop() const;

    // ---- Phase 2: ease the press off to "resting" before de-energizing ----
    // Plain orchestration on the same seat axes (does NOT touch HomingSequence). After the
    // press, the axes hold ~25% torque into the stop; disabling a loaded axis there trips
    // ErC1.2. startSeatRelief() snapshots each axis; stepSeatRelief() eases it back a homing
    // step per cycle while watching torque, and marks it done when the torque bottoms out
    // (the just-resting point -- stop bears the weight, torque -> ~0) or a small cap is hit,
    // so it stays ON the stop (no drop) at a low, safe-to-disable torque.
    void startSeatRelief(A6Drive** drives, int numHwDrives);
    void stepSeatRelief(A6Drive** drives, int numHwDrives);
    bool allSeatAxesRelieved() const;
    void forceAxisParked(int i)
    {
        if (i >= 0 && i < MAX_DRIVES)
        {
            m_axisState[i] = AxisMotionState::PARKED;
            m_runtime[i].homed = false;
            m_needsRehome = true;
        }
    }

    void startHoming(int axisIndex = -1);

    // ---- UI-thread command path ----
    // The UI thread MUST use enqueueCommand() instead of calling startHoming()
    // or startPark() directly. Commands are executed by the RT thread at the
    // top of process(), eliminating data races on m_axisState[]/m_runtime[].
    // Returns false (and logs a warning) if the queue is full.
    bool enqueueCommand(MotionCommand cmd);

    // ---- UI-safe state snapshot ----
    // Call from the UI thread. Returns a consistent copy of all UI-visible
    // state published by the RT thread at the end of the last process() cycle.
    MotionStatus getMotionStatus() const;

    // ---- RT-thread-only state accessors (called from ControlLoop) ----
    // Do NOT call these from the UI thread - use getMotionStatus() instead.
    AxisMotionState getAxisState(int i) const
    {
        if (i < 0 || i >= MAX_DRIVES) return AxisMotionState::PARKED;
        return m_axisState[i];
    }
    std::string getAxisStateName(int i) const;
    bool    allAxesParked()   const;
    bool    allAxesHomed()    const;
    bool    allAxesReady()    const;
    bool    needsRehome()     const { return m_needsRehome; }
    void    setNeedsRehome(bool v)  { m_needsRehome = v; }

    // Guard diagnostics (per-axis CSP conditioner). resetGuardStats() at loop-start;
    // getGuardStats(i) read live for the cards and at loop-stop for the session log.
    void    resetGuardStats() { for (int i = 0; i < MAX_DRIVES; ++i) m_runtime[i].onlineCond.resetGuardStats(); }
    CommandConditioner::GuardStats getGuardStats(int i) const
    {
        if (i < 0 || i >= MAX_DRIVES) return {};
        return m_runtime[i].onlineCond.getGuardStats();
    }
    bool    isAxisHomed(int i) const;

private:
    struct AxisConfig
    {
        double  strokeMm          = 100.0;
        bool        invertDir     = false;
        std::string axisType      = "linear_vertical";
        double  countsPerMm       = 100.0;
        double  maxVelocityMmS    = 200.0;
        double  maxAccelMmS2      = 2000.0;
        double  maxJerkMmS3       = 60000.0;  // BLENDING s-curve only (ONLINE ignores jerk)
        bool    torqueMode        = false;    // belt tensioner axis (CST)
        bool    ppMode            = false;    // PP drive -- send target direct, drive profiles internally
        // ONLINE CSP tracking filter (critically-damped 2nd-order follower).
        // wn = 2*pi*trackingWnHz -- the single feel knob (smoothness vs latency).
        // The overshoot guarantee comes from the braking-aware velocity clamp
        // (derived from maxAccel), NOT a jerk limit -- see TrackingFilter.h.
        double  trackingWnHz             = 30.0;
        double  torqueMinPct      = 5.0;      // holding torque at telemetry=0 (snug)
        double  torqueMaxPct      = 50.0;     // max torque at telemetry=65535
        // Belt guards (see Config.h for semantics; all MOTOR-side values)
        double  beltSlewPctPerSec = 3000.0;   // d(tension)/dt safety envelope
        double  beltOverspeedRpm  = 600.0;    // sustained-speed trip threshold
        double  beltOverspeedMs   = 200.0;    // persistence before trip
        double  beltMaxTravelRevs = 3.0;      // net-winding cap since tension-up (0 = off)
        double  beltMaxRpm        = 800.0;    // master-side velocity fold-back knee (0 = off)
        double  beltRelaxerSec    = 0.0;      // sustained near-max dwell (0 = off)
        double  beltRelaxerPct    = 80.0;     // % of torqueMaxPct that counts as dwell
        double  beltUnitsPerRev   = 10.0;     // encoderCountsPerRev / countsPerMm (rpm math)
        double  blendMaxVelocityMmS = 20.0; // velocity cap during post-homing blend
        double  unparkTimeSec     = 3.0;
        double  parkTimeSec       = 3.0;
        double  onlineHoldTimeoutSec = 15.0;  // total stale-telemetry hold before park
        bool    spikeFilterEnabled= false;
        double  spikeMaxMm        = 5.0;
        std::string homeMode      = "center";
        double  homingBackoffMm   = 1.5;
        double  homingSpeedMmS    = 5.0;
        int     homingTorquePct   = 25;
        std::string homeDirection = "negative";
        double  minPos            = 0.0;
        double  maxPos            = 100.0;
        double  parkPos           = 1.5;
        double  centerPos         = 50.0;
    };

    // S-curve trajectory planner state per axis
    struct TrajectoryState
    {
        double pos   = 0.0;   // current planned position
        double vel   = 0.0;   // current planned velocity (mm/s)
        double accel = 0.0;   // current planned acceleration (mm/s^2)
        double targetPos = 0.0; // latest waypoint from telemetry
        bool   active = false;
    };

    struct AxisRuntime
    {
        double  filteredPos       = 0.0;
        double  currentPos        = 0.0;
        double  targetPos         = 0.0;
        double  alpha             = 0.1;
        bool    initialized       = false;
        double  interpStart       = 0.0;
        double  interpTarget      = 0.0;
        double  interpElapsed     = 0.0;
        double  interpDuration    = 3.0;
        double  prevPos           = 0.0;
        bool    homed             = false;
        bool    parkHoldLatched   = false;   // unhomed park: capture hold pos once
        double  blendStartPos     = 0.0;
        double  blendElapsed      = 0.0;
        double  blendDuration     = 2.0;
        int     blendExtensions   = 0;
        TrajectoryState traj;     // s-curve planner state
        CommandConditioner onlineCond;   // CSP command conditioner (Bypass/Interpolate/Filter); seeded at handoff
        double  onlineStaleSec    = 0.0;  // time since last valid telemetry frame in ONLINE
        double  lastTension       = 0.0;  // belt: last commanded torque % (stale-hold + park-ramp start)
        // Belt guard runtime (armed through BLENDING + ONLINE; re-seeded each tension-up)
        double  beltPrevRaw       = 0.0;  // last raw position for the shaft-speed derivative
        double  beltRefRaw        = 0.0;  // net-travel reference (position at tension-up)
        double  beltLastRpm       = 0.0;  // latest measured shaft rpm (fold-back input)
        bool    beltHaveRaw       = false;// derivative + reference seeded this session?
        double  beltOverspeedSec  = 0.0;  // time continuously above the overspeed threshold
        double  beltDwellSec      = 0.0;  // time continuously at/above the relaxer band
        bool    beltRelaxed       = false;// relaxer engaged (tension clamped to min)
        uint8_t beltLastTripWhy   = 0;    // 1=OVERSPEED 2=TRAVEL; sticky until next tension-up (card display)
        // Deinit seat phase-2 (ease-off-to-rest) per-axis state -- see startSeatRelief().
        double  seatReliefStartRaw = 0.0; // raw position where the ease-off began (cap reference)
        double  seatReliefMinTrq   = 0.0; // lowest |torque| seen while easing off (bottom detector)
        double  seatReliefLastEased = 0.0; // last ease-off distance that counted as progress
        double  seatReliefStallSec = 0.0; // time with no torque-drop and no movement (plateau detector)
        bool    seatReliefDone     = false;
        bool    onlineHadData     = false; // seen >=1 valid frame this ONLINE session?
                                           // stale-park only arms after the first frame --
                                           // before telemetry ever connects we hold, not park.
    };

    int              m_numDrives    = 1;
    double           m_cycleTimeSec = 0.001;
    double           m_blendTimeSec = 2.0;
    CommandConditioner::Mode m_conditioningMode = CommandConditioner::Mode::Bypass;  // CSP conditioning
    bool             m_needsRehome = true;
    AxisConfig       m_axisConfig[MAX_DRIVES];
    AxisRuntime      m_runtime[MAX_DRIVES];
    AxisMotionState  m_axisState[MAX_DRIVES];
    HomingSequence   m_homing[MAX_DRIVES];
    bool             m_seatActive[MAX_DRIVES] = {};   // axes selected by startSeatHoming() (deinit seat)
    std::atomic<bool> m_emergencyStop{false};
    double           m_estopElapsed = 0.0;
    bool             m_homingUnparkDone = false;
    bool             m_prevEstopState = false;  // estop edge detection in process()

    // Commands from UI thread, drained at top of process() on RT thread.
    // 32 slots is more than sufficient -- typical burst is 1 command per user action.
    SpscQueue<MotionCommand, 32> m_cmdQueue;

    // UI-read status snapshot. Written by RT thread at end of process(),
    // read by UI thread via getMotionStatus(). std::shared_mutex protects the copy.
    mutable std::shared_mutex m_statusLock;
    MotionStatus           m_statusSnapshot;
    static constexpr double ESTOP_RAMP_SEC = 0.5;
    // Stale-telemetry response (after at least one frame): hold in place for
    // ONLINE_STALE_HOLD_SEC to ride out brief dropouts/stutters, then ease to
    // center (standby) until the per-axis onlineHoldTimeoutSec, then park.
    static constexpr double ONLINE_STALE_HOLD_SEC = 2.0;
    static constexpr double READY_TOLERANCE_MM = 2.0;
    static constexpr double BLEND_HANDOVER_THRESHOLD_MM = 5.0;
    static constexpr int    MAX_BLEND_EXTENSIONS = 3;

    // Drain queued commands from UI thread (called at top of process())
    void drainCommands(A6Drive** drives, int numHwDrives);

    // Publish UI-visible state to m_statusSnapshot (called at end of process())
    void publishStatus();

    // CSP command conditioning: dispatch the live target through the selected mode
    // (Bypass / Interpolate / Filter) into the shared guard chain. velCap caps the
    // command velocity (= maxVelocity in ONLINE; the ramped blend cap in BLENDING).
    double conditionCommand(CommandConditioner& cond, double target,
                            double frameSec, double velCap, const AxisConfig& ac);

    // S-curve trajectory step function
    // Advances the trajectory state one cycle toward targetPos
    // with velocity, acceleration, and jerk limits.
    // velCap > 0 overrides ac.maxVelocityMmS (used during BLENDING to avoid
    // violent lurch to live telemetry position after post-homing re-entry).
    double stepTrajectory(TrajectoryState& traj, const AxisConfig& ac,
                          double velCap = -1.0);

    double applySpikeFilter(int i, double input);
    double interpolate(double from, double to, double t);

    void processHomingAxis(int i, A6Drive* drive, MotionOutput& output);
    void clearAxisLimits(int axis, A6Drive* drive);  // widen stale post-fault limits before rehome
};
