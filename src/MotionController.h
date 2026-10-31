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
#include "TorqueHomingSequence.h"
#include "DeviceForceModel.h"
#include "HapticsLayer.h"
#include "HapticLaws.h"
#include "ShakerBank.h"
#include "DeviceStateLayer.h"
#include "GearRatioLearner.h"
#include "SpscQueue.h"
#include "CommandConditioner.h"
#include "CommissioningMode.h"
#include "AxisKind.h"
#include <array>
#include <atomic>
#include <string>
#include <mutex>
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
        EngageDevice,     // device axes only: blend force field in; intVal = axis (-1 = all)
        ReleaseDevice,    // device axes only: ease force to 0 (limp); intVal = axis (-1 = all)
        // The haptics commands below never wait in the queue: enqueueCommand
        // hands them to whichever clock steps the layer next (the RT loop, or
        // the idle clock with the loop stopped: shakers).
        HapticsTest,      // web Test button: intVal = registry effect id; a
                          // transient fires once, a continuous effect previews
                          // at full level ~2 s (routing = effect's table)
        HapticsMute,      // master: intVal 1 = mute every tile, 0 = unmute every tile
        HapticsMuteFx,    // one tile: intVal = registry effect id | 0x100 to mute (without: unmute)
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
    ESTOPPING,
    TESTING      // commissioning excitation (CommissioningMode drives the axis)
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
    // Device axes: live lever position, home-frame revs (web teach
    // capture + card display; meaningful once homed), plus the travel
    // extremes swept since homing (Capture travel).
    double          devPosRev[MAX_DRIVES]        = {};
    double          devPosMin[MAX_DRIVES]        = {};
    double          devPosMax[MAX_DRIVES]        = {};
    // Gear-ratio learner snapshot (device effects). Read off-RT for the
    // status surface and for the car-cache save at shutdown.
    double          gearRatio[MAX_GEARS]          = {};
    bool            gearRatioKnown[MAX_GEARS]     = {};  // usable (learned or adopted)
    bool            gearRatioConfident[MAX_GEARS] = {};  // session-observed (persistable)
    bool            gearRatiosDirty               = false;
    // ...and the car the sim named before this one: its confident ratios,
    // to merge into the cache once per gearRatioPrevSeq step (a car change
    // starts the learner over; without this the old car's set was lost
    // unless the service was shut down in order).
    double          gearRatioPrev[MAX_GEARS]          = {};
    bool            gearRatioPrevConfident[MAX_GEARS] = {};
    bool            gearRatiosPrevDirty               = false;
    uint32_t        gearRatioPrevSeq                  = 0;
    // Haptics surface: total transients fired, master mute, live smoothed
    // level per continuous effect, and which NCX tokens the wire is
    // actually delivering right now (the per-tile channel-health chips).
    uint64_t        hapticsFired                  = 0;
    uint64_t        hapticsFiredBy[haptics::EVENT_TYPE_COUNT] = {};
    bool            hapticsMuted                  = false;   // the master MUTE was pressed (and not UNMUTE since)
    uint64_t        hapticsMuteMask               = 0;       // muted tiles, bit = registry effect id
    double          hapticsFxLevel[haptics::FX_TYPE_COUNT] = {};
    bool            ncxHave[NcxValues::TokenCount]          = {};
    // What each effect is being DRIVEN with (0..1, before amp/route/mute
    // gating) and the per-wheel severities of the two slip tiles plus the
    // Road replay's per-corner travel (mm), each with a session peak that
    // holds until Reset peaks: the tile readouts, so a session's worst
    // slide can be read after the sim gave the screen back. Same for the
    // channel values themselves (the chip shows value and peak).
    // 0 lateral, 1 longitudinal, 2 road mm, 3 on a kerb, 4 flat spot mm,
    // 5 impact ring (peak = hardest knock), 6 ground contact 1/0 (peak = 1
    // once the tyre has been in the air), 7 surface class, 8 water 0..1,
    // 9 floating 0..1, 10 brake disc heat 0..1, 11 punctured 1/0 (the
    // Wheels tile's tyre state; 0 while that tile is off), 12 the lateral
    // slip angle as played (deg: the sim's, or from a slip speed less its
    // baseline), 13 that baseline (deg, learned on the straights).
    static constexpr int HAP_WHEEL_SETS = 14;
    double          hapticsIn     [haptics::FX_TYPE_COUNT] = {};
    double          hapticsInPk   [haptics::FX_TYPE_COUNT] = {};
    // Each tile's triggers since Reset peaks, registry order: a transient's
    // fires, a continuous effect's episodes (its driven level rising past
    // a few percent after falling away); a Test counts none. With the
    // seconds since the last one began (-1: none yet).
    uint32_t        hapticsCount [haptics::EFFECT_COUNT] = {};
    double          hapticsAgoSec[haptics::EFFECT_COUNT] = {};
    // New road heights a second while moving (0: none arriving): the Kerb
    // tile's detector needs the physics rate, not SimHub's ~55.
    double          hapHeightsHz = 0.0;
    double          hapticsWheel  [HAP_WHEEL_SETS][4]      = {};
    double          hapticsWheelPk[HAP_WHEEL_SETS][4]      = {};
    double          ncxVal[NcxValues::TokenCount]          = {};
    double          ncxPk [NcxValues::TokenCount]          = {};
    // ABS: working on the sim's flag (held), from the wheels, and the rate
    // the wheels' dumps come at; the lateral slip's source (0 the skid
    // channel, 1 the sim's slip angle, 2 a combined slip, 3 an angle from a
    // slip speed), its baseline mode (0 learn, 1 learn once, 2 fixed), the
    // baseline's straight-line time, and whether that is enough to call it
    // learned.
    bool            hapAbsFlag = false, hapAbsWheels = false;
    bool            hapAbsFlagSeen = false;   // the sim's ABS flag has been on, this car (from wheels "with the flag")
    double          hapAbsWheelHz = 0.0;
    int             hapLatSrc = 0, hapLatMode = 0;
    double          hapLatBaseSec = 0.0;
    bool            hapLatLearned = false;
    // Slip from the car's g: each axle's sideways grip limit in use (the
    // preset until the car's g moves it; 0 = no lateral source yet) in the
    // lateral source's units (hapGripSrc), the seconds it
    // has learned from, whether the car's sideways g matches its turning
    // (haptics::gripTurnState: -1 not sent, 0 not enough cornering yet, 1
    // yes, 2 no); the braking limit (0 = not learned) and the lock onset in
    // force (slip ratios).
    double          hapGripLat[2] = {}, hapGripSec = 0.0, hapGripLon = 0.0, hapGripLock = 0.0;
    double          hapGripW[2] = {};   // the learned limit's weight in the one in use, front and rear (0 the preset .. 1)
    int             hapGripSrc = 0, hapGripTurn = -1;
    // Road: full mm learned for this car (0 = not yet), and the seconds of
    // driving it has learned over.
    double          hapRoadFullMm = 0.0, hapRoadFullSec = 0.0;
};

class MotionController
{
public:
    MotionController();
    void configure(const AppConfig& config);
    // E-stop. Any e-stop (the web or Qt button, the GPIO input, a drive's
    // own quick stop, the transport failing) TRIPS the controller: the
    // drives stay disabled and nothing that moves the rig (homing, unpark,
    // belt tension, device engage) is honoured until the drives are
    // re-initialized (clearEmergencyTrip, from a successful initialize).
    // Releasing the e-stop only releases its input; it never moves anything.
    enum class EstopSource : int { None = 0, Web = 1, Gpio = 2, App = 3, Drive = 4, Transport = 5 };
    void setEmergencyStop(bool active, EstopSource source = EstopSource::App);
    // Stopped by an e-stop: its input held, or tripped since the last initialize.
    bool isEmergencyStop() const { return m_emergencyStop.load() || m_estopTripped.load(); }
    bool isEstopInputHeld() const { return m_emergencyStop.load(); }
    bool isEstopTripped() const   { return m_estopTripped.load(); }
    EstopSource estopTripSource() const { return static_cast<EstopSource>(m_estopTripSource.load()); }
    // steady_clock ms of the trip (0 = none).
    int64_t estopTripMs() const   { return m_estopTripMs.load(); }
    // After a successful initialize of the drives only. Refused (false) while
    // the e-stop input is still held (release it first), and when an e-stop
    // came after the initialize began (eventsAtInitStart = estopEventCount()
    // read as it started): that e-stop needs an initialize of its own.
    bool clearEmergencyTrip(uint32_t eventsAtInitStart);
    bool clearEmergencyTrip() { return clearEmergencyTrip(estopEventCount()); }
    // Every e-stop, any source, counted.
    uint32_t estopEventCount() const { return m_estopEvents.load(std::memory_order_acquire); }
    static const char* estopSourceName(EstopSource s);
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

    // Device engage/release (shifter/pedal families ONLY; every other axis
    // untouched). PARKED = limp (zero torque, lever moves freely); engage
    // blends the force field in over blendTime; release ramps it back to 0.
    // Same surface pattern as the belt pair: RT thread only, UI enqueues
    // EngageDevice/ReleaseDevice. Engage is refused while e-stopped and for
    // an unhomed device (the force field is home-frame). axisIndex -1 = all
    // device axes.
    void engageDevices(int axisIndex);
    void releaseDevices(int axisIndex);

    // Car cache handoff (main thread, at startup - before the RT loop runs).
    void setCarCache(const std::vector<CachedCar>& cars)
    { m_ratioLearner.setCache(cars.data(), (int)cars.size()); }

    // ---- Haptics live-apply (the tuning path, same idea as devices) ----
    // A rig save stages the new haptics tuning here (web thread, under the
    // lock); the RT thread applies it at the top of the next cycle - amp,
    // frequency, routing and master gain changes never need a re-init.
    void stageHaptics(const AppConfig& c);
    // The sound card behind the shaker routes (owned by main; may be null).
    void setShakerOutput(haptics::ShakerBank* s) { m_shakers = s; }
    // Haptics idle clock tick (a timer thread while the control loop is
    // stopped): laws + layer step + shaker push, nothing else.
    void hapticsIdleTick(const TelemetryData& td);
    void stepHapticsCycle(const TelemetryData& td, bool estop);   // one haptics cycle (both clocks)
    // The layer itself, read-only, for tests that must see what reaches an
    // axis or a shaker rather than a readout that is 0 by construction.
    // Read it only while nothing steps the layer (no loop, no idle clock).
    const haptics::Layer& hapticsLayer() const { return m_haptics; }
    // Shaker tone test: a 40 Hz tone on one channel for a second, picked up
    // by whichever clock is stepping the layer (RT loop or idle clock).
    void requestShakerTone(int channel) { m_toneRequest.store(channel + 1, std::memory_order_release); }

    // ---- Device live-apply (the tuning path) ----
    // A rig save stages new device params here (web thread, under the
    // lock); the RT thread applies them the next cycle the device rests
    // LIMP (PARKED) - feel edits never need a restart. Anything staged
    // while engaged applies on release. Non-device settings keep the
    // restart semantics.
    void stageDeviceParams(int axisIndex, const DeviceParams& p);

    // ---- Belt from g live-apply ----
    // A rig save whose only axis changes are a belt's g shape (surge, sway,
    // heave and their shares, g from, g at max, curve, smoothing) stages
    // the shape here (web thread); the RT thread takes it at the top of a
    // cycle without waiting on the web thread. The belt's source, its
    // torque range and its guards still wait for an Initialize.
    void stageBeltG(int axisIndex, const DriveConfig& dc);
    // A save that changes the axes drops any shape staged by index.
    void clearStagedBeltG();

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
    // The belt's live tension demand from its source (the motion line or
    // the car's g); false when there is none this cycle.
    bool   beltDemand(int i, const TelemetryData& td, double& tension);
    double beltGTension(int i, double x);
    // Belt from g after a crash: the share of the g this cycle (1 = all of
    // it), from the crash watch in the axis's runtime. RT thread only.
    double beltCrashScale(int i, const NcxValues& v, bool teleport);

    // ---- Per-family step functions (Stage D decomposition) ----------------
    // process()'s per-axis switch keeps the family-agnostic frame (e-stop
    // edges, HOMING, PARKED hold, UNPARKING, ESTOPPING, TESTING) and
    // delegates BLENDING/ONLINE/PARKING to these. Each returns the cycle's
    // commanded position and owns its family's state transitions. The
    // family fork inside those cases is the seam future device families
    // (shifter, pedal) extend -- one new stepXxx set per family, never
    // more branches inside process().
    bool   axisFrameUsable(int i, const TelemetryData& td) const;
    // 16-bit wire value -> engineering units: the ONE decode authority
    // (centre-32767 scaling, polarity, spike filter, travel clamp).
    double decodeTargetMm(int i, double raw);
    double stepBeltBlending(int i, AxisMotionState& state, const TelemetryData& td,
                            MotionOutput& output, A6Drive** drives, int numHwDrives);
    double stepPositionBlending(int i, AxisMotionState& state, const TelemetryData& td);
    double stepBeltOnline(int i, AxisMotionState& state, const TelemetryData& td,
                          MotionOutput& output, A6Drive** drives, int numHwDrives);
    void   noteBeltMotionStream(int i, const TelemetryData& td);
    double stepPositionOnline(int i, AxisMotionState& state, const TelemetryData& td);
    double stepBeltParking(int i, AxisMotionState& state, MotionOutput& output);
    double stepPositionParking(int i, AxisMotionState& state);
    // Device family (shifter/pedal): the force field runs every cycle from
    // the axis's OWN position; no telemetry dependency in v1. BLENDING =
    // engage ramp (forceScale 0 -> 1), ONLINE = live field, PARKING =
    // force ramp to 0 -> PARKED (limp).
    double stepDeviceBlending(int i, AxisMotionState& state, const TelemetryData& td,
                              MotionOutput& output, A6Drive** drives, int numHwDrives);
    double stepDeviceOnline(int i, AxisMotionState& state, const TelemetryData& td,
                            MotionOutput& output, A6Drive** drives, int numHwDrives);
    double stepDeviceParking(int i, AxisMotionState& state, MotionOutput& output);
    // Device position in HOME-frame revs: dir maps motor raw onto the
    // device frame (the mirror of DeviceForceModel's output mapping);
    // null drive (sim) reads as resting at neutral.
    double deviceCurrentRev(int i, A6Drive* drive) const;

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
    // Returns false (and logs a warning) if the queue is full. Any number of
    // threads may call it at once (the web server's pool, the GPIO panel,
    // the Qt window): they take turns on m_cmdPushMx. Never the RT thread.
    bool enqueueCommand(MotionCommand cmd);
    // Test seam: the RT drain's own pop, for the multi-producer stress test
    // (TestCommandQueue), which plays the RT thread's part.
    bool popCommandForTest(MotionCommand& out) { return m_cmdQueue.pop(out); }

    // ---- Commissioning test mode (UI thread entry; RT services it) ----
    // The plan is copied under a lock and picked up at the top of the next
    // process() cycle, where the runtime rails are checked (all excited axes
    // homed + PARKED, no rehome pending, no e-stop, telemetry quiet). A
    // refusal shows up in getCommissioningStatus().reason. Returns false only
    // if a test is already active or pending.
    bool requestCommissioningStart(const CommissioningPlan& plan);
    void requestCommissioningStop();
    CommissioningStatus getCommissioningStatus() const
    { return m_commissioning.getStatus(); }

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
    void    resetGuardStats()
    {
        for (int i = 0; i < MAX_DRIVES; ++i) m_runtime[i].onlineCond.resetGuardStats();
        requestHapticPeakReset();
    }
    // Haptic session peaks (tile readouts) re-baseline with the drive
    // peaks: Reset peaks in the header, and every loop start. Consumed by
    // whichever path steps the haptics next (RT loop or idle clock).
    void    requestHapticPeakReset() { m_hapPeakReset.store(true, std::memory_order_release); }
    // The Lateral tile's relearn: its straight-line baseline starts over,
    // taken by whichever path steps the haptics next.
    void    requestLatRelearn() { m_hapLatRelearn.store(true, std::memory_order_release); }
    // The Wheels tile's fresh tyres: flat spots and hot discs gone.
    void    requestFreshTyres() { m_hapFreshTyres.store(true, std::memory_order_release); }
    // A tile's mute as asked (registry effect id), before a clock applies it.
    bool    hapticsTileMuted(int id) const
    {
        return id >= 0 && id < 64 && ((m_hapMuteMask.load(std::memory_order_acquire) >> id) & 1ull);
    }
    CommandConditioner::GuardStats getGuardStats(int i) const
    {
        if (i < 0 || i >= MAX_DRIVES) return {};
        return m_runtime[i].onlineCond.getGuardStats();
    }
    bool    isAxisHomed(int i) const;

private:
    struct AxisConfig
    {
        AxisCaps caps;            // classification authority (see AxisKind.h)
        double  strokeMm          = 100.0;
        bool        invertDir     = false;
        // Effective telemetry polarity: invertDir (linkage reversal) XOR
        // homed-extended (frame reversal). Computed once in configure() so the
        // RT path composes the two signs without re-deriving them per cycle.
        bool        telemetryInvert = false;
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
        double  beltSlewPctPerSec = 20000.0;  // telemetry d(tension)/dt cap (configure() copies the rig's value)
        double  beltOverspeedRpm  = 600.0;    // sustained-speed trip threshold
        double  beltOverspeedMs   = 200.0;    // persistence before trip
        double  beltMaxTravelRevs = 3.0;      // net-winding cap since tension-up (0 = off)
        double  beltMaxRpm        = 800.0;    // master-side velocity fold-back knee (0 = off)
        double  beltRelaxerSec    = 0.0;      // sustained near-max dwell (0 = off)
        double  beltRelaxerPct    = 80.0;     // % of torqueMaxPct that counts as dwell
        // Belt cue from the car's g (Config.h beltSource "g"): what tightens
        // it on each of the car's three axes, 0 off, 1 the value above zero,
        // 2 below zero, 3 either (surge: decel / accel / both, accSurge
        // reading above zero under braking; sway: left / right / both;
        // heave: press / lift / both), and sway's and heave's share against
        // surge's.
        bool    beltFromG         = false;
        int     beltGMode         = 1;
        int     beltGSway         = 0;
        int     beltGHeave        = 0;
        double  beltGSwayMix      = 1.0;
        double  beltGHeaveMix     = 1.0;
        double  beltGFromG        = 0.05;
        double  beltGFullG        = 1.5;
        double  beltGCurve        = 1.0;
        double  beltGSmoothMs     = 30.0;
        double  beltGCrashPct     = 50.0;     // the g's share after a crash while it stays rough (100 = off)
        double  beltUnitsPerRev   = 10.0;     // encoderCountsPerRev / countsPerMm (rpm math)
        double  hapticsMaxMm      = 3.0;      // position axes: largest haptic offset (0 = none)
        double  blendMaxVelocityMmS = 20.0; // velocity cap during post-homing blend
        double  unparkTimeSec     = 3.0;
        double  parkTimeSec       = 3.0;
        double  onlineHoldTimeoutSec = 15.0;  // total stale-telemetry hold before park
        bool    spikeFilterEnabled= false;
        double  spikeMaxMm        = 5.0;
        std::string parkMode      = "center";
        DeviceParams device;      // device families only (shifter/pedal)
        double  homingBackoffMm   = 1.5;
        double  homingSpeed    = 5.0;
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
        // Haptics on a position axis: sumGuard passes cue + overlay through
        // the FULL axis limits so the cue keeps priority (the layer has
        // already derated the overlay to what the axis can follow). Seeded
        // when an overlay first appears, cleared whenever not ONLINE.
        CommandConditioner sumGuard;
        bool    hapActive         = false;
        bool    stepGuardWarned   = false; // the last-line step check has logged this live stretch
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
        // Belt from g, one per axis of the car (surge, sway, heave).
        struct BeltGChan
        {
            double f       = 0.0;   // the g, low-passed
            bool   live    = false; // seeded by a live sample (afresh after a gap or tension-up)
            double badSec  = 0.0;   // how long it has been unusable (a gap past kBeltGReseedSec re-seeds)
            // Heave: any gravity in the sender's value (0, or +-1 g), taken
            // off; learned from its level standing still and steady (the
            // level, and how long it has held), kept while the car drives.
            double grav    = 0.0;
            double parkG   = 0.0, parkSec = 0.0;
            double lastRaw = 0.0;   // the last value sent, and how long it has stayed exactly so
            double sameSec = 0.0;
        };
        BeltGChan beltGCh[3];
        // Belt from g: a teleport's hold-off (seconds left: the g is held
        // off and the belt sits at minimum), and the sender's last speed
        // and road height per wheel (once seen), for the jumps. Cleared
        // with the g filters at tension-up (resetBeltTeleport).
        double  beltTeleportSec  = 0.0;
        double  beltLastSpeedKmh = 0.0;
        double  beltLastRoadMm[4] = {};
        bool    beltSpeedSeen    = false;
        bool    beltRoadSeen[4]  = {};
        void resetBeltTeleport()
        {
            beltTeleportSec = 0.0; beltSpeedSeen = false;
            for (bool& s : beltRoadSeen) s = false;
        }
        // Belt from g: the crash watch (MotionController::beltCrashScale).
        // The g's slow part and its fast part's power (rough, junk frames
        // left out), how long surge has held one value, the most speed over
        // the last 0.6 s in 50 ms buckets, a hit waiting for its speed
        // loss, the look after a crash (its second half's power, junk in),
        // the crash held (its car, how long quiet) and the share it leaves
        // the g. Road state rather than the car's: kept across a car change
        // (the hold ends with the car); cleared at tension-up (resetBeltCrash).
        struct BeltCrash
        {
            double meanS = 0.0, meanW = 0.0, roughPow = 0.0;
            bool   seeded = false;
            double lastS = 0.0, sameSec = 0.0;
            double speedBucket[12] = {};
            int    bucket = 0;
            double bucketSec = 0.0;
            bool   speedSeen = false;
            double hitSec = 0.0, hitKmh = 0.0, hitPeakMs2 = 0.0;
            double lookSec = 0.0, lookPow = 0.0, lookTime = 0.0, lostKmh = 0.0;
            bool   held = false;
            double quietSec = 0.0;
            char   car[NCY_STR_LEN] = {}, carId[NCY_STR_LEN] = {};
            double scale = 1.0;
        };
        BeltCrash beltCrash;
        void resetBeltCrash() { beltCrash = BeltCrash{}; }
        // Belt from g: the motion stream's staleness, which (not the
        // channels') lets the belt slacken by itself, with the posts.
        // Armed once a motion frame is seen this session.
        double  beltMotionStaleSec = 0.0;
        bool    beltMotionSeen     = false;
        uint8_t beltLastTripWhy   = 0;    // 1=OVERSPEED 2=TRAVEL; sticky until next tension-up (card display)
        // Deinit seat phase-2 (ease-off-to-rest) per-axis state -- see startSeatRelief().
        double  seatReliefStartRaw = 0.0; // raw position where the ease-off began (cap reference)
        double  seatReliefMinTrq   = 0.0; // lowest |torque| seen while easing off (bottom detector)
        double  seatReliefLastEased = 0.0; // last ease-off distance that counted as progress
        double  seatReliefStallSec = 0.0; // time with no torque-drop and no movement (plateau detector)
        bool    seatReliefDone     = false;
        bool    onlineHadData     = false; // seen >=1 valid frame this ONLINE session?
        // Device family (shifter/pedal) runtime. The home frame lives HERE,
        // not in A6Drive's home offset: raw counts never pass through the mm
        // scaling, and deviceCurrentRev() is the one converter.
        DeviceForceModel deviceModel;
        DeviceStateLayer deviceState;      // NULLCATX effects (stateless, fail-safe inert)
        double  deviceHomeRaw     = 0.0;   // raw units latched at the found stop
        double  deviceHomeStopRev = 0.0;   // which configured stop that raw maps to
        bool    deviceSeeded      = false; // model reset at engage (fresh, no vel kick)
        double  devPosRev         = 0.0;   // live position, refreshed every cycle
        // Travel sweep since homing: the teach capture reads these (move
        // the lever end to end by hand while limp, press Capture travel).
        double  devPosMin         = 0.0;
        double  devPosMax         = 0.0;
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
    TorqueHomingSequence m_torqueHoming[MAX_DRIVES];   // device axes (HomingKind::Torque)
    NcxMap           m_ncxMap;   // rig-level NULLCATX binding table (strings resolved once)
    haptics::Layer   m_haptics;  // routable haptic layer (RT-owned; see HapticsLayer.h)
    haptics::LawsState m_hapLaws;  // Test preview timers + gear edge state (RT-owned)
    double           m_hapPositionBudget = 0.4;  // share of a position axis's limits haptics may use
    void fireHaptics(const HapticTriggers& t);
    void driveContinuousHaptics(const TelemetryData& td);
    void applyStagedHaptics();
    void publishHapticsStatusLocked();
    void publishLearnerLocked();
    void pushShakers();
    haptics::ShakerBank*   m_shakers = nullptr;
    std::atomic<bool>      m_hapticsBusy{false};
    std::atomic<int>       m_toneRequest{0};   // channel + 1, 0 = none
    // Tile Test and mutes, taken by whichever clock steps the layer next.
    std::atomic<int>       m_hapTestReq{-1};          // registry effect id, -1 = none
    std::atomic<uint64_t>  m_hapMuteMask{0};          // muted tiles, bit = registry effect id
    std::atomic<bool>      m_hapMasterMute{false};
    uint64_t               m_hapMuteApplied = ~0ull;  // what the layer has (clock thread); ~0 = apply next cycle
    void applyHapticMutes();
    void applyHaptics(const AppConfig& c);
    void applyHapticSinks();
    // Live-apply stage (trivially copyable - no allocation on either side):
    // one slot per registry effect plus the engine description.
    struct HapticsStage
    {
        std::array<haptics::EffectParams, haptics::EFFECT_COUNT> fx{};
        haptics::EngineParams engine;
        haptics::SlipParams   slipLat, slipLon;
        haptics::RoadParams   road;
        haptics::KerbParams   kerb;
        haptics::PulseParams  abs, tc;
        haptics::SurfaceParams surface;
        haptics::WheelsParams wheels;
        haptics::ImpactParams impacts;
        haptics::DrivelineParams driveline;
        double masterGain = 1.0;
        double positionBudget = 0.4;
        double axisDelayMs = 0.0;
        double beltFullG = 0.0;
    };
    HapticsStage     m_hapStage;
    // The type of car's belt g at max (haptics beltFullG; 0 = each belt's
    // own). Written where the haptics config lands (configure, the staged
    // apply), read by the belt law on the RT thread.
    double           m_beltTypeFullG = 0.0;
    std::mutex       m_hapStageLock;
    std::atomic<bool> m_hapStagePending{false};
    // Channel-health snapshot for the status surface, the channel values
    // beside it, and the session peaks (haptics-thread owned; copied into
    // the snapshot by publishHapticsStatusLocked).
    bool             m_ncxHaveSnapshot[NcxValues::TokenCount] = {};
    void trackHapticPeaks();
    // Per-car gear-ratio learner (survives configure(): re-init must not
    // forget a session's driving; only setCarCache() reseeds the cache).
    GearRatioLearner m_ratioLearner;
    // Device live-apply staging (see stageDeviceParams). The mask is the
    // lock-free "anything pending?" fast path; the params copy under the
    // lock is brief and only contends with a web save.
    std::mutex            m_devApplyLock;
    DeviceParams          m_devPending[MAX_DRIVES];
    std::atomic<uint16_t> m_devPendingMask{0};
    void applyPendingDeviceParams();
    // The belt-from-g law's settings as one copyable block. configure() and
    // the live apply both go through beltGShapeOf (one place for the
    // clamps and the names).
    struct BeltGShape
    {
        int    mode = 1, sway = 0, heave = 0;
        double swayMix = 1.0, heaveMix = 1.0, fromG = 0.05, fullG = 1.5, curve = 1.0, smoothMs = 30.0;
        double crashPct = 50.0;
    };
    static BeltGShape beltGShapeOf(const DriveConfig& dc);
    static void       putBeltGShape(AxisConfig& ac, const BeltGShape& s);
    std::mutex            m_beltGLock;
    BeltGShape            m_beltGPending[MAX_DRIVES];
    std::atomic<uint16_t> m_beltGMask{0};
    void applyPendingBeltG();
    bool             m_seatActive[MAX_DRIVES] = {};   // axes selected by startSeatHoming() (deinit seat)
    std::atomic<bool> m_emergencyStop{false};   // the e-stop input (held)
    std::atomic<bool> m_estopTripped{false};    // latched by any e-stop, cleared only by clearEmergencyTrip
    std::atomic<int>  m_estopTripSource{0};
    std::atomic<int64_t> m_estopTripMs{0};
    std::atomic<uint32_t> m_estopEvents{0};
    bool             estopHolds() const { return m_emergencyStop.load(std::memory_order_acquire) || m_estopTripped.load(std::memory_order_acquire); }
    double           m_estopElapsed = 0.0;
    bool             m_homingUnparkDone = false;
    bool             m_prevEstopState = false;  // estop edge detection in process()

    // Commands from the UI threads, drained at top of process() on RT thread.
    // 32 slots (31 usable) is more than sufficient -- typical burst is 1
    // command per user action. The queue takes ONE producer: the threads
    // that enqueue (web pool, GPIO panel, Qt) serialise on m_cmdPushMx, so
    // to the queue they are one; the RT thread's pop() never takes it.
    SpscQueue<MotionCommand, 32> m_cmdQueue;
    std::mutex                   m_cmdPushMx;

    // ---- Commissioning test mode state ----
    // Plan handoff: UI writes m_pendingTestPlan under the lock and sets the
    // request bit; the RT thread copies it out in serviceCommissioning().
    CommissioningMode    m_commissioning;
    std::mutex           m_testPlanLock;
    CommissioningPlan    m_pendingTestPlan;
    std::atomic<uint8_t> m_testRequest{0};       // bit0 = start, bit1 = stop
    double m_testOffsets[MAX_DRIVES] = {};       // engine output this cycle
    bool   m_testWasActive = false;              // completion edge detection
    double m_secsSinceTelemetry = 1e9;           // telemetry-quiet entry rail
    void serviceCommissioning(A6Drive** drives, int numHwDrives);

    // UI-read status snapshot. Written by RT thread at end of process(),
    // read by UI thread via getMotionStatus(). std::shared_mutex protects the copy.
    mutable std::shared_mutex m_statusLock;
    MotionStatus           m_statusSnapshot;
    // Alloc-free steady-state publishing: the snapshot's state-name strings
    // are rebuilt ONLY when the state (or homing sub-state) changes. A
    // per-cycle rebuild allocated ~10 strings/cycle on the RT thread, and
    // the process heap lock has no priority inheritance -- a GUI thread
    // alloc storm (exactly what park/unpark clicks trigger: log pane, card
    // repaints) could stall the RT loop inside malloc (2208B: 4-14ms stall
    // bursts trailing every park/unpark event).
    AxisMotionState m_pubState[MAX_DRIVES] = {};
    int             m_pubHomingSub[MAX_DRIVES] = {};
    bool            m_pubNameInit[MAX_DRIVES] = {};
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
    void processDeviceHomingAxis(int i, A6Drive* drive, MotionOutput& output);
    void clearAxisLimits(int axis, A6Drive* drive);  // widen stale post-fault limits before rehome

    double           m_ncxValSnapshot [NcxValues::TokenCount] = {};
    double           m_ncxPk          [NcxValues::TokenCount] = {};
    double           m_hapInPk   [haptics::FX_TYPE_COUNT]    = {};
    double           m_hapWheelPk[MotionStatus::HAP_WHEEL_SETS][4] = {};
    double           m_hapPreviewQuiet[haptics::FX_TYPE_COUNT] = {};   // a Test preview and its tail: no peaks
    // Tile triggers (trackHapticPeaks): counts, when the last began (on
    // the haptics clock), whether a continuous effect is in an episode,
    // and the fire counts already taken.
    uint32_t         m_hapCount    [haptics::EFFECT_COUNT] = {};
    double           m_hapLastSec  [haptics::EFFECT_COUNT] = {};
    bool             m_hapTrigOn   [haptics::EFFECT_COUNT] = {};
    uint64_t         m_hapFiredSeen[haptics::EFFECT_COUNT] = {};
    double           m_hapClockSec = 0.0;
    uint32_t         m_hapCarGen = 0;   // the car the counts are for (LawsState::carGen)
    // The road heights' arrival rate: changes counted over a window while
    // moving, and the last rate.
    double           m_hgtLast = 0.0, m_hgtWinSec = 0.0, m_hgtHz = 0.0, m_hgtGoneSec = 0.0;
    int              m_hgtChanges = 0;
    std::atomic<bool> m_hapPeakReset{false};
    std::atomic<bool> m_hapLatRelearn{false};
    std::atomic<bool> m_hapFreshTyres{false};
};
