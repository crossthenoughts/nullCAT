// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// MotionController.cpp
//
// Per-axis motion state machine (PARKED / HOMING / UNPARKING / BLENDING /
// ONLINE / PARKING / ESTOPPING), CSP command conditioning, belt torque
// tensioning with runaway guards, and the deinit seat-on-the-stop sequence.
// ============================================================

#define _USE_MATH_DEFINES   // M_PI under MSVC; must precede the FIRST <cmath> include,
                            // which arrives transitively via MotionController.h ->
                            // CommandConditioner.h. GCC/glibc expose M_PI regardless, so
                            // this is a no-op on the Pi build.
#include "MotionController.h"
#include "Logging.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <shared_mutex>
#include <thread>

static constexpr double TELEMETRY_CENTER = 32767.0;

// The wire contract (Docs/CONFIG_REFERENCE.md) defines an axis value as
// 16-bit unsigned, 0..65535 with centre 32767 -- and that is the ONLY
// scaling. Until 0.9.3 a per-frame heuristic guessed "any channel above 500
// means 16-bit, otherwise the values are millimetres": unspecced (present
// since the initial commit, predating the contract), re-decided every frame
// (a channel hovering around 500 teleported the target between opposite
// stroke ends on consecutive frames), and cross-channel (one odd channel
// flipped the interpretation of every axis). Removed: values are 16-bit,
// full stop.
//
// One deliberate carve-out replaces the accidental protection the mm branch
// used to provide: a frame where EVERY channel is exactly 0 is treated as
// no-data rather than as a command. In 16-bit, 0 is full deflection to one
// end -- but no real motion frame commands every axis and belt to zero
// simultaneously, while some telemetry tools DO emit all-zeros at menu/idle.
// Reading such a frame literally would slam the whole rig to one end;
// treating it as no-data feeds the existing stale-telemetry standby instead.
static bool telemetryFrameUsable(const TelemetryData& t)
{
    if (!t.valid || t.numPositions <= 0) return false;
    for (int j = 0; j < t.numPositions; ++j)
        if (t.positions[j] != 0.0) return true;   // any nonzero channel = real frame
    return false;
}

// Belt tensioner mapping: telemetry raw 0..65535 -> torqueMin..torqueMax (% of rated),
// clamped. Unidirectional -- the belt never commands below torqueMin while tracking.
static inline double beltTension(double raw, double minPct, double maxPct)
{
    double norm = std::max(0.0, std::min(1.0, raw / 65535.0));
    return minPct + norm * (maxPct - minPct);
}

MotionController::MotionController()
{
    for (int i = 0; i < MAX_DRIVES; ++i)
    {
        m_axisState[i] = AxisMotionState::PARKED;
        m_runtime[i] = AxisRuntime{};
        m_axisConfig[i] = AxisConfig{};
    }
}

// The haptics path (the Layer, the laws, the channel map, the sink kinds)
// is stepped by the idle clock while the loop is stopped, which is when
// configure() runs (a re-initialise, a rig save in the desktop app): hold
// the flag the RT loop and the clock share, so no tick runs on half-written
// settings. This thread is not real-time: a short spin is fine.
namespace {
struct HapticsBusyGuard
{
    std::atomic<bool>& flag;
    explicit HapticsBusyGuard(std::atomic<bool>& f) : flag(f)
    {
        while (flag.exchange(true, std::memory_order_acq_rel)) std::this_thread::yield();
    }
    ~HapticsBusyGuard() { flag.store(false, std::memory_order_release); }
};

// haptics.axisDelayMs in control cycles, for the layer's delay line.
int axisDelayCycles(double ms, double cycleSec)
{
    return static_cast<int>(ms / 1000.0 / std::max(1e-6, cycleSec) + 0.5);
}
}

void MotionController::configure(const AppConfig& config)
{
    HapticsBusyGuard busy(m_hapticsBusy);
    m_numDrives = std::min(config.numDrives, MAX_DRIVES);
    m_cycleTimeSec = 1.0 / std::max(100, config.controlLoopHz);
    m_blendTimeSec = config.blendTimeSec;
    m_conditioningMode = (config.conditioningMode == "interpolate") ? CommandConditioner::Mode::Interpolate
                       : (config.conditioningMode == "filter")      ? CommandConditioner::Mode::Filter
                       :                                              CommandConditioner::Mode::Bypass;
    m_needsRehome = true;
    // A belt shape staged since the config was read is NOT dropped here: the
    // web thread stages only while the running axes are the file's (see
    // WebServer), so it is newer than or the same as what is read now. A
    // save that changes the axes drops it (clearStagedBeltG).

    // Initialize is a fresh start: a command queued while the loop was
    // stopped (a tension or engage pressed while tripped, a script) must
    // not run at the next loop start. The loop is stopped whenever this
    // runs, so this thread is the queue's only consumer.
    {
        MotionCommand stale;
        int dropped = 0;
        while (m_cmdQueue.pop(stale)) ++dropped;
        if (dropped > 0)
            LOG_INFO(strf("MotionController: %d queued command(s) from before the initialize discarded.", dropped));
    }

    // Haptic layer: tuning + routes are rig-global config. applyHaptics is
    // shared with the live-apply staging path (a rig save applies without
    // a re-initialize).
    applyHaptics(config);
    m_hapPositionBudget = std::max(0.0, std::min(1.0, config.hapticsPositionBudget));
    m_beltTypeFullG = std::isfinite(config.hapticsBeltFullG) ? config.hapticsBeltFullG : 0.0;
    // The axis delay too: it was only set by a live apply, so after every
    // start or Initialize the axes played undelayed until the next save.
    m_haptics.setAxisDelayCycles(axisDelayCycles(config.hapticsAxisDelayMs, m_cycleTimeSec));

    // NULLCATX channel bindings (rig-level): resolve token strings once so
    // the RT path only does index lookups.
    m_ncxMap.configure(config.ncxBindings);
    if (m_ncxMap.boundCount() > 0)
        LOG_INFO(strf("MotionController: %d NULLCATX channel binding(s) active.",
                      m_ncxMap.boundCount()));

    for (int i = 0; i < m_numDrives && i < config.drives.size(); ++i)
    {
        const DriveConfig& dc = config.drives[i];
        AxisConfig& ac = m_axisConfig[i];

        ac.strokeMm = dc.strokeMm;
        ac.invertDir = dc.invertDir;
        // Telemetry polarity composes TWO reversals: the linkage (invertDir --
        // foldback moves the platform opposite to actuator extension) and the
        // frame (homing to the extended stop points engineering the other way
        // along the same travel). XOR keeps the platform's response identical
        // whether an axis parks retracted or extended.
        ac.telemetryInvert = (dc.invertDir != (dc.homeDirection == "positive"));
        ac.axisType = dc.axisType;
        ac.countsPerMm = dc.countsPerMm;
        ac.maxVelocityMmS = dc.maxVelocityMmS;
        ac.maxAccelMmS2 = dc.maxAccelerationMmS2;
        ac.maxJerkMmS3 = dc.maxJerkMmS3;   // s-curve planner only; the tracking follower ignores jerk
        ac.unparkTimeSec = dc.unparkTimeSec;
        ac.parkTimeSec = dc.parkTimeSec;
        ac.onlineHoldTimeoutSec = dc.onlineHoldTimeoutSec;
        ac.spikeFilterEnabled = dc.spikeFilterEnabled;
        ac.spikeMaxMm = dc.spikeMaxMm;
        ac.parkMode = dc.parkMode;
        ac.homingBackoffMm = dc.homingBackoffMm;
        ac.homingSpeed = dc.homingSpeed;
        ac.homingTorquePct = dc.homingTorquePct;
        ac.homeDirection = dc.homeDirection;
        ac.caps          = axisCaps(dc.axisType, dc.mode);
        ac.torqueMode    = ac.caps.torqueMode;
        ac.ppMode        = ac.caps.ppMode;
        ac.trackingWnHz             = dc.trackingWnHz;
        ac.torqueMinPct  = dc.torqueMinPct;
        ac.torqueMaxPct  = dc.torqueMaxPct;
        ac.beltSlewPctPerSec = dc.beltSlewPctPerSec;
        ac.beltOverspeedRpm  = dc.beltOverspeedRpm;
        ac.beltOverspeedMs   = dc.beltOverspeedMs;
        ac.beltMaxTravelRevs = dc.beltMaxTravelRevs;
        ac.beltMaxRpm        = dc.beltMaxRpm;
        ac.hapticsMaxMm      = dc.hapticsMaxMm;
        ac.beltRelaxerSec    = dc.beltRelaxerSec;
        ac.beltRelaxerPct    = dc.beltRelaxerPct;
        ac.beltFromG         = (dc.beltSource == "g");
        putBeltGShape(ac, beltGShapeOf(dc));
        // Shaft-rpm math for the overspeed guard: getActualPositionRaw() returns
        // counts/countsPerMm "units", so one motor rev = encoderCountsPerRev/countsPerMm.
        ac.beltUnitsPerRev = (dc.countsPerMm > 0.0)
                           ? (dc.encoderCountsPerRev / dc.countsPerMm) : 10.0;
        ac.blendMaxVelocityMmS = config.blendMaxVelocityMmS;

        bool isBelt = ac.caps.beltType;

        // Device families (shifter/pedal): force engine + torque homing.
        // getActualPositionRaw() is in counts/countsPerMm "units", so the
        // rev conversion shares the belt's units-per-rev factor.
        if (ac.caps.isDevice())
        {
            ac.device = dc.device;
            m_runtime[i].deviceModel.configure(ac.device, m_cycleTimeSec);
            m_runtime[i].deviceState.configure(ac.device);
            m_torqueHoming[i].configure(ac.device, ac.beltUnitsPerRev, m_cycleTimeSec);
            LOG_INFO(strf("MotionController: Axis %d device: dir=%+.0f stops[%.3f, %.3f]rev "
                          "neutral=%.3frev detents=%d maxForce=%.0f%% homeTorque=%.0f%% homeDir=%+.0f",
                          i + 1, ac.device.dir, ac.device.stopMinRev, ac.device.stopMaxRev,
                          ac.device.neutralRev, (int)ac.device.detents.size(),
                          ac.device.maxForcePct, ac.device.homeTorquePct, ac.device.homeDir));
        }

        if (ac.torqueMode && isBelt)
        {
            // Ratio visibility: the strap sees motor torque x reduction. Config
            // validation caps maxPct x ratio at 300%-of-rated; log the effective
            // strap-side ceiling so a ratio change is never silently carried.
            double rf = 1.0;
            if (!dc.reductionRatio.empty()) rf = std::max(1.0, atof(dc.reductionRatio.c_str()));
            LOG_INFO(strf("MotionController: Axis %d belt guards: slew=%.0f%%/s overspeed=%.0frpm/%.0fms "
                          "relaxer=%s | tension %.0f..%.0f%% x %s = strap-side max %.0f%% of rated",
                          i + 1, ac.beltSlewPctPerSec, ac.beltOverspeedRpm, ac.beltOverspeedMs,
                          ac.beltRelaxerSec > 0.0 ? strf("%.0fs@%.0f%%", ac.beltRelaxerSec, ac.beltRelaxerPct).c_str() : "off",
                          ac.torqueMinPct, ac.torqueMaxPct, dc.reductionRatio.c_str(),
                          ac.torqueMaxPct * rf));
            if (ac.beltFromG)
            {
                static const char* const kSurge[4] = { "off", "braking", "acceleration", "both" };
                static const char* const kSway[4]  = { "off", "left", "right", "both" };
                static const char* const kHeave[4] = { "off", "press", "lift", "both" };
                LOG_INFO(strf("MotionController: Axis %d belt from the car's g: surge %s, sway %s x%.2f, "
                              "heave %s x%.2f, %.2f..%.2f g, curve %.2f, smoothed %.0f ms", i + 1,
                              kSurge[ac.beltGMode & 3], kSway[ac.beltGSway & 3], ac.beltGSwayMix,
                              kHeave[ac.beltGHeave & 3], ac.beltGHeaveMix,
                              ac.beltGFromG, ac.beltGFullG, ac.beltGCurve, ac.beltGSmoothMs));
            }
        }

        if (isBelt)
        {
            ac.minPos    = 0.0;
            ac.maxPos    = dc.strokeMm;
            ac.centerPos = 0.0;   // not used for belt
            ac.parkPos   = 0.0;   // snug end -- belt taut but not pulling
        }
        else
        {
            // Offset coordinate system: 0.0 = backoff position, strokeMm = full extension
            ac.minPos = 0.0;
            ac.maxPos = dc.strokeMm;
            ac.centerPos = dc.strokeMm / 2.0;

            // "endstop": park at backoff after homing (vertical axes -- gravity holds at bottom)
            // "center":  park at mid-stroke after homing (horizontal axes -- natural rest position)
            if (dc.parkMode == "center")
                ac.parkPos = ac.centerPos;
            else
                ac.parkPos = dc.homingBackoffMm;  // "endstop" and all other modes
        }

        AxisRuntime& rt = m_runtime[i];
        // rt.alpha is consumed nowhere; it stays in the struct so existing
        // test fixtures need no changes. Candidate for a future cleanup.
        rt.currentPos = ac.parkPos;
        rt.filteredPos = ac.parkPos;
        rt.prevPos = ac.parkPos;
        rt.initialized = true;
        rt.homed = false;

        // Initialize trajectory state
        rt.traj.pos = ac.parkPos;
        rt.traj.vel = 0.0;
        rt.traj.accel = 0.0;
        rt.traj.targetPos = ac.parkPos;
        rt.traj.active = false;

        m_axisState[i] = AxisMotionState::PARKED;

        m_homing[i].configure(dc, m_cycleTimeSec);

        // parkMode and homingSpeed are logged because they are the two settings a
        // homing/park post-mortem always needs and neither was previously in the
        // log: parkMode selects parkPos (nothing else), and homingSpeed is a
        // per-cycle step multiplier rather than true mm/s (see homingStepMm),
        // so it is quoted as the raw setting.
        const char* u = ac.caps.unit;   // "mm" / "deg": rotary logs read in degrees
        LOG_INFO(strf("MotionController: Axis %d '%s' type=%s "
            "stroke=%.1f%s backoff=%.2f%s center=%.1f%s maxV=%.1f%s/s maxA=%.1f%s/s2 maxJ=%.1f%s/s3 "
            "homeDir=%s parkMode=%s homingSpeed=%.0f (setting, not %s/s)",
            i + 1, dc.name.c_str(), dc.axisType.c_str(),
            ac.strokeMm, u, ac.parkPos, u, ac.centerPos, u,
            ac.maxVelocityMmS, u, ac.maxAccelMmS2, u, ac.maxJerkMmS3, u,
            ac.homeDirection.c_str(), dc.parkMode.c_str(), ac.homingSpeed, u));

        // Conditioning mode + (Filter only) the derived knee consequences -- read
        // the cost of the numbers, not just the numbers. (CSP axes only.)
        if (!ac.ppMode && !ac.torqueMode)
        {
            if (m_conditioningMode == CommandConditioner::Mode::Filter)
            {
                const double wn = 2.0 * M_PI * ac.trackingWnHz;
                const double linRegimeMm = ac.maxAccelMmS2 / (wn * wn);
                const double groupDelayMs = (wn > 0.0) ? (2.0 / wn) * 1e3 : 0.0;
                LOG_INFO(strf("MotionController: Axis %d conditioning=FILTER wn=%.1f rad/s (%.1fHz) "
                    "| linear no-overshoot regime |err|<%.3fmm | group delay ~%.1fms",
                    i + 1, wn, ac.trackingWnHz, linRegimeMm, groupDelayMs));
            }
            else
            {
                LOG_INFO(strf("MotionController: Axis %d conditioning=%s (relative braking; "
                    "latency = %s)", i + 1,
                    m_conditioningMode == CommandConditioner::Mode::Bypass ? "BYPASS" : "INTERPOLATE",
                    m_conditioningMode == CommandConditioner::Mode::Bypass ? "lowest"
                                                                          : "~1 frame (1/new_hz)"));
            }
        }
    }

    LOG_INFO(strf("MotionController: Configured %d axes at %dHz",
        m_numDrives, config.controlLoopHz));

    // Haptic sink kinds + position limits read the axis configs filled above.
    applyHapticSinks();
}

void MotionController::setEmergencyStop(bool active, EstopSource source)
{
    // Only atomics here -- safe from any thread. The state transitions
    // (ESTOPPING, then PARKED in place) happen inside process() on the RT
    // thread, so m_axisState[] is never written off it.
    if (active)
    {
        // The count, then the input, then the trip (latched with its first
        // source and time): a clearEmergencyTrip racing this sees the count
        // moved or the input held when it re-checks, and keeps the trip.
        m_estopEvents.fetch_add(1, std::memory_order_acq_rel);
        m_emergencyStop.store(true, std::memory_order_release);
        if (!m_estopTripped.exchange(true, std::memory_order_acq_rel))
        {
            m_estopTripSource.store(static_cast<int>(source), std::memory_order_release);
            m_estopTripMs.store(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_release);
        }
        // RT_LOG routes by thread: lock-free from the RT loop (a drive's
        // quick stop, the transport), the ordinary path from any other.
        RT_LOG_WARNING("MotionController: EMERGENCY STOP (%s)", estopSourceName(source));
    }
    else
    {
        m_emergencyStop.store(false, std::memory_order_release);
        if (m_estopTripped.load(std::memory_order_acquire))
            RT_LOG_INFO("MotionController: Emergency stop input released (%s) -- the drives stay "
                        "stopped until you re-initialize them.", estopSourceName(source));
    }
}

bool MotionController::clearEmergencyTrip(uint32_t eventsAtInitStart)
{
    if (m_emergencyStop.load(std::memory_order_acquire))
    {
        LOG_WARNING("MotionController: E-stop still held -- the trip stays until it is released and the drives are re-initialized.");
        return false;
    }
    if (m_estopEvents.load(std::memory_order_acquire) != eventsAtInitStart)
    {
        LOG_WARNING("MotionController: E-stop during the initialize -- the trip stays; initialize again to clear it.");
        return false;
    }
    if (m_estopTripped.exchange(false, std::memory_order_acq_rel))
    {
        // An e-stop that landed during the clear keeps the trip.
        if (m_emergencyStop.load(std::memory_order_acquire)
            || m_estopEvents.load(std::memory_order_acquire) != eventsAtInitStart)
        {
            m_estopTripped.store(true, std::memory_order_release);
            LOG_WARNING("MotionController: E-stop pressed during the initialize -- the trip stays.");
            return false;
        }
        m_estopTripSource.store(0, std::memory_order_release);
        m_estopTripMs.store(0, std::memory_order_release);
        LOG_INFO("MotionController: E-stop trip cleared by re-initializing the drives.");
    }
    return true;
}

const char* MotionController::estopSourceName(EstopSource s)
{
    switch (s)
    {
    case EstopSource::Web:       return "dashboard e-stop";
    case EstopSource::Gpio:      return "GPIO e-stop input";
    case EstopSource::App:       return "app e-stop";
    case EstopSource::Drive:     return "drive quick stop";
    case EstopSource::Transport: return "EtherCAT transport failure";
    default:                     return "e-stop";
    }
}

bool MotionController::enqueueCommand(MotionCommand cmd)
{
    // The haptics commands are the layer's, and the layer is stepped by the
    // idle clock while the loop is stopped (shakers), which never drains
    // the queue: they go to slots the next haptics cycle takes, either way.
    static_assert(haptics::EFFECT_COUNT <= 64, "the mute mask has a bit per tile");
    switch (cmd.type)
    {
    case MotionCommand::Type::HapticsTest:
        if (cmd.intVal < 0 || cmd.intVal >= haptics::EFFECT_COUNT) return false;
        m_hapTestReq.store(cmd.intVal, std::memory_order_release);
        return true;
    case MotionCommand::Type::HapticsMute:
        // The master: every tile muted, or every tile unmuted (mute all,
        // then unmute one tile: that tile solo).
        m_hapMasterMute.store(cmd.intVal != 0, std::memory_order_release);
        m_hapMuteMask.store(cmd.intVal != 0 ? (haptics::EFFECT_COUNT >= 64 ? ~0ull : ((1ull << haptics::EFFECT_COUNT) - 1)) : 0ull,
                            std::memory_order_release);
        return true;
    case MotionCommand::Type::HapticsMuteFx:
    {
        const int id = cmd.intVal & 0xFF;
        if (id >= haptics::EFFECT_COUNT) return false;
        if (cmd.intVal & 0x100) m_hapMuteMask.fetch_or(1ull << id, std::memory_order_acq_rel);
        else                    m_hapMuteMask.fetch_and(~(1ull << id), std::memory_order_acq_rel);
        return true;
    }
    default:
        break;
    }
    bool pushed;
    {
        std::lock_guard<std::mutex> lk(m_cmdPushMx);   // one producer at a time (SpscQueue's contract)
        pushed = m_cmdQueue.push(cmd);
    }
    if (!pushed)
    {
        LOG_WARNING(strf("MotionController: Command queue full -- command type %d dropped.",
            static_cast<int>(cmd.type)));
        return false;
    }
    return true;
}

void MotionController::drainCommands(A6Drive** /*drives*/, int /*numHwDrives*/)
{
    MotionCommand cmd;
    while (m_cmdQueue.pop(cmd))
    {
        switch (cmd.type)
        {
        case MotionCommand::Type::StartHoming:
            startHoming(cmd.intVal);
            break;
        case MotionCommand::Type::StartPark:
            startPark();
            break;
        case MotionCommand::Type::StartUnpark:
            // drives only used for a log line in startUnpark (seed is rt.currentPos)
            startUnpark(nullptr, 0);
            break;
        case MotionCommand::Type::SlackBelts:
            slackBelts();
            break;
        case MotionCommand::Type::TensionBelts:
            tensionBelts();
            break;
        case MotionCommand::Type::EngageDevice:
            engageDevices(cmd.intVal);
            break;
        case MotionCommand::Type::ReleaseDevice:
            releaseDevices(cmd.intVal);
            break;
        default:   // the haptics commands never reach the queue (enqueueCommand)
            break;
        }
    }
}

// ============================================================
// Commissioning test mode
// ============================================================

bool MotionController::requestCommissioningStart(const CommissioningPlan& plan)
{
    if (m_commissioning.active() ||
        (m_testRequest.load(std::memory_order_acquire) & 0x1))
        return false;
    {
        std::lock_guard<std::mutex> lk(m_testPlanLock);
        m_pendingTestPlan = plan;
    }
    m_testRequest.fetch_or(0x1, std::memory_order_release);
    return true;
}

void MotionController::requestCommissioningStop()
{
    m_testRequest.fetch_or(0x2, std::memory_order_release);
}

void MotionController::serviceCommissioning(A6Drive** drives, int numHwDrives)
{
    const uint8_t req = m_testRequest.exchange(0, std::memory_order_acq_rel);

    if (req & 0x2)
    {
        if (m_commissioning.active())
        {
            RT_LOG_INFO("MotionController: Commissioning test stopped by user.");
            m_commissioning.abort("stopped by user");
        }
    }

    if ((req & 0x1) && !m_commissioning.active())
    {
        CommissioningPlan plan;
        {
            std::lock_guard<std::mutex> lk(m_testPlanLock);
            plan = m_pendingTestPlan;
        }

        // ---- Entry rails. Every axis the plan excites must be homed and
        // PARKED; no rehome pending; no e-stop; telemetry stream quiet. ----
        const char* refusal = nullptr;
        char refusalBuf[96];
        bool excite[MAX_DRIVES] = {};
        for (int s = 0; s < plan.numSegments && s < MAX_TEST_SEGMENTS; ++s)
            for (int i = 0; i < m_numDrives && i < MAX_DRIVES; ++i)
                if (plan.seg[s].ampMm[i] != 0.0) excite[i] = true;

        bool anyExcited = false;
        for (int i = 0; i < m_numDrives; ++i) anyExcited = anyExcited || excite[i];

        // The axis-kind rail comes FIRST: it is stateless, and "belt not
        // testable" is a more useful refusal than the rehome/park rails a
        // belt axis would otherwise trip.
        for (int i = 0; i < m_numDrives && !refusal; ++i)
        {
            if (!excite[i]) continue;
            const AxisConfig& ac = m_axisConfig[i];
            if (ac.torqueMode || ac.ppMode)
            {
                std::snprintf(refusalBuf, sizeof(refusalBuf),
                    "axis %d is a %s axis -- not testable", i + 1,
                    ac.torqueMode ? "belt/torque" : "PP");
                refusal = refusalBuf;
            }
        }

        if (refusal)
            ;   // keep the axis-kind refusal
        else if (plan.numSegments <= 0 || !anyExcited)
            refusal = "empty plan (no axes selected?)";
        else if (estopHolds())
            refusal = "e-stop tripped -- re-initialize the drives";
        else if (m_needsRehome)
            refusal = "rehome required -- home all axes first";
        else if (m_secsSinceTelemetry < 2.0)
            refusal = "telemetry stream active -- stop the game/SimHub feed first";
        else
        {
            for (int i = 0; i < m_numDrives; ++i)
            {
                if (!excite[i]) continue;
                if (!m_runtime[i].homed)
                {
                    std::snprintf(refusalBuf, sizeof(refusalBuf),
                        "rig not homed -- Start Loop homes automatically "
                        "(axis %d not homed)", i + 1);
                    refusal = refusalBuf;
                    break;
                }
                if (m_axisState[i] != AxisMotionState::PARKED)
                {
                    std::snprintf(refusalBuf, sizeof(refusalBuf),
                        "rig not parked -- press Park All, keep the loop "
                        "running, then start the test (axis %d is %s)", i + 1,
                        getAxisStateName(i).c_str());
                    refusal = refusalBuf;
                    break;
                }
            }
        }

        if (refusal)
        {
            RT_LOG_WARNING("MotionController: Commissioning start refused: %s", refusal);
            m_commissioning.setRefusal(refusal);
        }
        else
        {
            CommissioningAxisLimits limits[MAX_DRIVES];
            for (int i = 0; i < m_numDrives; ++i)
            {
                const AxisConfig& ac = m_axisConfig[i];
                limits[i].enabled = excite[i];
                // Usable half-range about centre: the excitation must fit on
                // BOTH sides of centerPos inside [minPos, maxPos].
                limits[i].halfStrokeMm = std::min(ac.centerPos - ac.minPos,
                                                  ac.maxPos - ac.centerPos);
                limits[i].maxVelMmS    = ac.maxVelocityMmS;
                limits[i].maxAccelMmS2 = ac.maxAccelMmS2;
                std::snprintf(limits[i].unit, sizeof(limits[i].unit),
                              "%s", ac.caps.unit);
                // Platform frame, like telemetry: the engine's offsets are
                // multiplied by the same polarity on the way out (TESTING
                // case), so its centering path must start from the offset
                // measured in that frame -- otherwise an inverted axis would
                // step to the mirrored position on test entry.
                limits[i].startOffsetMm = (ac.telemetryInvert ? -1.0 : 1.0)
                                        * (m_runtime[i].currentPos - ac.centerPos);
            }
            m_commissioning.start(plan, limits, m_numDrives, m_cycleTimeSec);
            for (int i = 0; i < m_numDrives; ++i)
            {
                if (!excite[i]) continue;
                m_axisState[i] = AxisMotionState::TESTING;
                m_runtime[i].onlineCond.seedState(m_runtime[i].currentPos, 0.0);
            }
            RT_LOG_INFO("MotionController: Commissioning test '%s' started "
                        "(%d segment(s)).", plan.title, plan.numSegments);
        }
    }

    // ---- Per-cycle service of an armed engine ----
    if (m_commissioning.active())
    {
        // Cancel rail: if any excited axis was taken out of TESTING by a
        // fault park, e-stop, or user park, the engine must not keep
        // commanding -- the axes are no longer listening.
        for (int i = 0; i < m_numDrives; ++i)
        {
            if (m_commissioning.axisEnabled(i) &&
                m_axisState[i] != AxisMotionState::TESTING)
            {
                RT_LOG_WARNING("MotionController: Commissioning cancelled -- "
                               "axis %d left TESTING.", i + 1);
                m_commissioning.cancel("axes taken over (fault/e-stop/park)");
                break;
            }
        }
    }

    const bool stepped = m_commissioning.active()
                       ? m_commissioning.step(m_testOffsets) : false;
    if (!stepped)
        std::memset(m_testOffsets, 0, sizeof(m_testOffsets));

    // Completion edge: engine finished (Done) -> park the TESTING axes via
    // the normal park machinery (parkMode=center holds, endstop moves aside).
    if (m_testWasActive && !m_commissioning.active())
    {
        bool anyTesting = false;
        for (int i = 0; i < m_numDrives; ++i)
            anyTesting = anyTesting || (m_axisState[i] == AxisMotionState::TESTING);
        if (anyTesting)
        {
            RT_LOG_INFO("MotionController: Commissioning test finished -- parking.");
            startPark();
        }
    }
    m_testWasActive = m_commissioning.active();
}

void MotionController::resetFilters()
{
    for (int i = 0; i < MAX_DRIVES; ++i)
    {
        m_runtime[i].filteredPos = m_axisConfig[i].parkPos;
        m_runtime[i].currentPos = m_axisConfig[i].parkPos;
        m_runtime[i].prevPos = m_axisConfig[i].parkPos;
        m_runtime[i].initialized = false;
        m_runtime[i].traj.pos = m_axisConfig[i].parkPos;
        m_runtime[i].traj.vel = 0.0;
        m_runtime[i].traj.accel = 0.0;
        m_runtime[i].traj.targetPos = m_axisConfig[i].parkPos;
        m_runtime[i].traj.active = false;
    }
}

bool MotionController::allAxesParked() const
{
    for (int i = 0; i < m_numDrives; ++i)
        if (m_axisState[i] != AxisMotionState::PARKED)
            return false;
    return true;
}

bool MotionController::allAxesHomed() const
{
    for (int i = 0; i < m_numDrives; ++i)
    {
        if (!m_axisConfig[i].caps.homes) continue;
        // Devices are lifecycle-independent of the rig (like belts): they
        // home via their own deliberate engage flow, so they must never
        // hold the post-homing auto-unpark hostage.
        if (m_axisConfig[i].caps.isDevice()) continue;
        if (!m_runtime[i].homed) return false;
    }
    return true;
}

bool MotionController::allAxesReady() const
{
    for (int i = 0; i < m_numDrives; ++i)
    {
        if (!m_axisConfig[i].caps.homes) continue;
        if (m_axisConfig[i].caps.isDevice()) continue;   // device readiness is its own affair
        if (m_axisState[i] != AxisMotionState::ONLINE &&
            m_axisState[i] != AxisMotionState::BLENDING) return false;
    }
    return true;
}

bool MotionController::isAxisHomed(int i) const
{
    if (i < 0 || i >= MAX_DRIVES) return false;
    return m_runtime[i].homed;
}

void MotionController::startHoming(int axisIndex)
{
    // Nothing moves after an e-stop until the drives are re-initialized:
    // loop-start auto-homing, a fault-recovery rehome and the Home button
    // alike.
    if (estopHolds())
    {
        RT_LOG_WARNING("MotionController: Homing refused -- e-stop tripped; re-initialize the drives to resume.");
        return;
    }
    m_homingUnparkDone = false;
    m_needsRehome = false;
    int start = (axisIndex < 0) ? 0 : axisIndex;
    int end = (axisIndex < 0) ? m_numDrives : axisIndex + 1;

    for (int i = start; i < end; ++i)
    {
        if (i >= m_numDrives) break;
        if (!m_axisConfig[i].caps.homes)
        {
            RT_LOG_INFO("MotionController: Axis %d is belt type -- skipping homing.", i + 1);
            m_runtime[i].homed = true;
            continue;
        }
        // Home-ALL never grabs a device: a homing push must not surprise a
        // hand resting on the lever when the rig comes up (or on e-stop
        // release). Devices home through their own deliberate action - the
        // device button, or an explicitly TARGETED home of that one axis.
        if (axisIndex < 0 && m_axisConfig[i].caps.isDevice())
        {
            RT_LOG_INFO("MotionController: Axis %d is a device -- homes via its "
                        "engage button, not home-all.", i + 1);
            continue;
        }
        // Reset HomingSequence regardless of what state it was in.
        // Without this, if e-stop fires mid-CSPTorqueSearch and clears,
        // processHomingAxis sees state != Idle/Complete/Error and skips
        // start() -- leaving the drive in SwitchOnDisabled while homing
        // tries to run CSPTorqueSearch (positions frozen, torque=0%).
        if (m_axisConfig[i].caps.homingKind == HomingKind::Torque)
            m_torqueHoming[i].reset();
        else
            m_homing[i].reset();
        m_axisState[i] = AxisMotionState::HOMING;
        m_runtime[i].homed = false;
        RT_LOG_INFO("MotionController: Axis %d queued for homing.", i + 1);
    }
}

std::string MotionController::getAxisStateName(int i) const
{
    if (i < 0 || i >= MAX_DRIVES) return "UNKNOWN";
    switch (m_axisState[i])
    {
    case AxisMotionState::HOMING:
    {
        std::string sub = (m_axisConfig[i].caps.homingKind == HomingKind::Torque)
            ? TorqueHomingSequence::stateName(m_torqueHoming[i].getState())
            : HomingSequence::stateName(m_homing[i].getState());
        return "HOMING[" + sub + "]";
    }
    case AxisMotionState::PARKED:    return "PARKED";
    case AxisMotionState::UNPARKING: return "UNPARKING";
    case AxisMotionState::BLENDING:  return "BLENDING";
    case AxisMotionState::ONLINE:    return "ONLINE";
    case AxisMotionState::PARKING:   return "PARKING";
    case AxisMotionState::ESTOPPING: return "E-STOP";
    case AxisMotionState::TESTING:   return "TESTING";
    default:                         return "UNKNOWN";
    }
}

MotionStatus MotionController::getMotionStatus() const
{
    std::shared_lock<std::shared_mutex> lock(m_statusLock);
    return m_statusSnapshot;
}

void MotionController::publishStatus()
{
    // try_lock, never wait: this runs on the RT thread every cycle, and the
    // readers (web poll, Qt timer) hold the shared lock WITHOUT priority
    // inheritance -- if Windows preempts a reader mid-hold, a blocking RT
    // writer stalls for the reader's whole preemption (the 4-11ms
    // "stall -- resynced" cluster in the 2208 logs, ~1/s, tracking the UI
    // poll cadence). Status is best-effort: skip this cycle, publish next.
    std::unique_lock<std::shared_mutex> lock(m_statusLock, std::try_to_lock);
    if (!lock.owns_lock()) return;
    m_statusSnapshot.numDrives   = m_numDrives;
    m_statusSnapshot.needsRehome = m_needsRehome;
    for (int i = 0; i < m_numDrives; ++i)
    {
        m_statusSnapshot.axisState    [i] = m_axisState[i];
        // Rebuild the display string only on a state change -- see the
        // m_pubState comment in the header (RT heap-lock avoidance).
        const AxisMotionState st = m_axisState[i];
        const int sub = (st != AxisMotionState::HOMING) ? -1
                      : (m_axisConfig[i].caps.homingKind == HomingKind::Torque)
                      ? (int)m_torqueHoming[i].getState()
                      : (int)m_homing[i].getState();
        if (!m_pubNameInit[i] || st != m_pubState[i] || sub != m_pubHomingSub[i])
        {
            m_statusSnapshot.axisStateName[i] = getAxisStateName(i);
            m_pubState[i]     = st;
            m_pubHomingSub[i] = sub;
            m_pubNameInit[i]  = true;
        }
        m_statusSnapshot.homed        [i] = m_runtime[i].homed;
        const auto gs = m_runtime[i].onlineCond.getGuardStats();   // CSP accel diag for the cards
        m_statusSnapshot.accelWinPeakMms2[i] = gs.accelWinPeakMms2;
        m_statusSnapshot.accelClipPct    [i] = gs.clipPct;
        m_statusSnapshot.accelBindPct    [i] = gs.bindPct;
        // Belt card telemetry: commanded tension + guard state (trip reason is sticky
        // until the next tension-up, so a trip is visible on the card, not log-only).
        m_statusSnapshot.beltCmdPct[i] = m_runtime[i].lastTension;
        m_statusSnapshot.beltGuard [i] = m_runtime[i].beltLastTripWhy ? m_runtime[i].beltLastTripWhy
                                       : (m_runtime[i].beltRelaxed ? 3 : 0);
        m_statusSnapshot.devPosRev[i]  = m_runtime[i].devPosRev;
        m_statusSnapshot.devPosMin[i]  = m_runtime[i].devPosMin;
        m_statusSnapshot.devPosMax[i]  = m_runtime[i].devPosMax;
    }
    publishHapticsStatusLocked();
    // The learner steps inside the haptics cycle, which the idle clock may
    // still be running for a tick at loop start: read it only holding the
    // flag that cycle takes (a car change mid-read would publish the new
    // count with the old car's set, and the cache would never get the car).
    if (!m_hapticsBusy.exchange(true, std::memory_order_acq_rel))
    {
        publishLearnerLocked();
        m_hapticsBusy.store(false, std::memory_order_release);
    }
}

// Gear-ratio learner snapshot: the status surface reads it live; the car
// cache reads the published copy (a finished car from the mains' poll, the
// current one after the loop stops). Caller holds the status lock AND the
// haptics busy flag (the learner's own thread at that moment).
void MotionController::publishLearnerLocked()
{
    for (int g = 1; g < MAX_GEARS; ++g)
    {
        m_statusSnapshot.gearRatio[g]          = m_ratioLearner.gearRatio(g);
        m_statusSnapshot.gearRatioKnown[g]     = m_ratioLearner.ratios().known[g];
        m_statusSnapshot.gearRatioConfident[g] = m_ratioLearner.gearSessionConfident(g);
        m_statusSnapshot.gearRatioPrev[g]          = m_ratioLearner.prevRatio(g);
        m_statusSnapshot.gearRatioPrevConfident[g] = m_ratioLearner.prevConfident(g);
    }
    m_statusSnapshot.gearRatiosDirty     = m_ratioLearner.dirty();
    m_statusSnapshot.gearRatiosPrevDirty = m_ratioLearner.prevDirty();
    m_statusSnapshot.gearRatioPrevSeq    = m_ratioLearner.prevSeq();
}

// The haptics part of the snapshot, shared by the RT publish and the idle
// clock (which has no axes to publish but keeps the tiles, dots and
// channel chips live while the loop is stopped). Caller holds the lock.
void MotionController::publishHapticsStatusLocked()
{
    m_statusSnapshot.hapticsFired = m_haptics.fireCount();
    for (int i = 0; i < haptics::EVENT_TYPE_COUNT; ++i)
        m_statusSnapshot.hapticsFiredBy[i] = m_haptics.fireCount(static_cast<haptics::EventType>(i));
    m_statusSnapshot.hapticsMuted    = m_hapMasterMute.load(std::memory_order_acquire);
    m_statusSnapshot.hapticsMuteMask = m_hapMuteMask.load(std::memory_order_acquire);
    for (int i = 0; i < haptics::FX_TYPE_COUNT; ++i)
    {
        m_statusSnapshot.hapticsFxLevel[i] = m_haptics.fxOutputLevel(i);   // felt, not driven
        m_statusSnapshot.hapticsIn[i]      = m_haptics.fxLevel(i);         // driven
        m_statusSnapshot.hapticsInPk[i]    = m_hapInPk[i];
    }
    for (int w = 0; w < 4; ++w)
    {
        m_statusSnapshot.hapticsWheel[0][w] = m_haptics.slipWheelLevel(haptics::FxType::Skid, w);
        m_statusSnapshot.hapticsWheel[1][w] = m_haptics.slipWheelLevel(haptics::FxType::Lockup, w);
        m_statusSnapshot.hapticsWheel[2][w] = m_haptics.roadWheelTravelMm(w);
        m_statusSnapshot.hapticsWheel[3][w] = m_haptics.kerbWheelLevel(w);
        m_statusSnapshot.hapticsWheel[4][w] = m_haptics.wheelFlatMm(w);
        m_statusSnapshot.hapticsWheel[5][w] = m_haptics.impactModel().ringLevel(w);
        m_statusSnapshot.hapticsWheel[6][w] = m_haptics.groundContact(w);
        m_statusSnapshot.hapticsWheel[7][w] = m_haptics.surfaceModel().surfaceClass(w);
        m_statusSnapshot.hapticsWheel[8][w] = m_haptics.surfaceModel().water(w);
        m_statusSnapshot.hapticsWheel[9][w] = m_haptics.surfaceModel().floating(w);
        m_statusSnapshot.hapticsWheel[10][w] = m_haptics.wheelsModel().discHeat(w);
        m_statusSnapshot.hapticsWheel[11][w] = m_haptics.wheelsModel().punctured(w) ? 1.0 : 0.0;
        m_statusSnapshot.hapticsWheel[12][w] = m_hapLaws.latAngle[w];
        m_statusSnapshot.hapticsWheel[13][w] = m_hapLaws.latBase[w];
        for (int s = 0; s < MotionStatus::HAP_WHEEL_SETS; ++s)
            m_statusSnapshot.hapticsWheelPk[s][w] = m_hapWheelPk[s][w];
    }
    for (int i = 0; i < NcxValues::TokenCount; ++i)
    {
        m_statusSnapshot.ncxHave[i] = m_ncxHaveSnapshot[i];
        m_statusSnapshot.ncxVal[i]  = m_ncxValSnapshot[i];
        m_statusSnapshot.ncxPk[i]   = m_ncxPk[i];
    }
    m_statusSnapshot.hapAbsFlag    = m_hapLaws.absFlagOn;
    m_statusSnapshot.hapAbsFlagSeen = m_hapLaws.absFlagSeen;
    m_statusSnapshot.hapLatSrc     = m_hapLaws.slipSrc;
    m_statusSnapshot.hapLatMode    = m_hapLaws.latBaseMode;
    m_statusSnapshot.hapAbsWheels  = m_hapLaws.absWheelHoldSec > 0.0;
    m_statusSnapshot.hapAbsWheelHz = m_hapLaws.absWheelHz;
    m_statusSnapshot.hapLatBaseSec = m_hapLaws.latBaseSec;
    m_statusSnapshot.hapLatLearned = m_hapLaws.latBaseSec >= haptics::laws_k::kLatBaseLearnSec;
    m_statusSnapshot.hapRoadFullMm  = m_haptics.roadModel().learnedFullMm();
    m_statusSnapshot.hapRoadFullSec = m_haptics.roadModel().learnedFullSec();
}

// Session peaks of the tile readouts, taken every haptics step (the
// status publish is try-lock and may skip a cycle; a peak must not).
// Peaks hold the magnitude: a signed slip angle peaks at its |value|.
void MotionController::trackHapticPeaks()
{
    if (m_hapPeakReset.exchange(false, std::memory_order_acq_rel))
    {
        for (double& p : m_hapInPk) p = 0.0;
        for (auto& set : m_hapWheelPk) for (double& p : set) p = 0.0;
        for (double& p : m_ncxPk) p = 0.0;
    }
    // A Test preview drives its tile at full, which is no peak from the sim:
    // skip it, and the half second its level takes to fall away after.
    constexpr double kPreviewPeakTailSec = 0.5;
    bool testing[haptics::FX_TYPE_COUNT] = {};
    for (int i = 0; i < haptics::FX_TYPE_COUNT; ++i)
    {
        double& q = m_hapPreviewQuiet[i];
        q = (m_hapLaws.previewSec[i] > 0.0) ? kPreviewPeakTailSec : std::max(0.0, q - m_cycleTimeSec);
        testing[i] = q > 0.0;
    }
    const auto live = [&](haptics::FxType t) { return !testing[static_cast<int>(t)]; };
    for (int i = 0; i < haptics::FX_TYPE_COUNT; ++i)
        if (!testing[i]) m_hapInPk[i] = std::max(m_hapInPk[i], m_haptics.fxLevel(i));
    for (int w = 0; w < 4; ++w)
    {
        using haptics::FxType;
        if (live(FxType::Skid))    m_hapWheelPk[0][w] = std::max(m_hapWheelPk[0][w], m_haptics.slipWheelLevel(FxType::Skid, w));
        if (live(FxType::Lockup))  m_hapWheelPk[1][w] = std::max(m_hapWheelPk[1][w], m_haptics.slipWheelLevel(FxType::Lockup, w));
        if (live(FxType::Road))    m_hapWheelPk[2][w] = std::max(m_hapWheelPk[2][w], std::fabs(m_haptics.roadWheelTravelMm(w)));
        if (live(FxType::Kerb))    m_hapWheelPk[3][w] = std::max(m_hapWheelPk[3][w], m_haptics.kerbWheelLevel(w));
        if (live(FxType::Wheels))
        {
            m_hapWheelPk[4][w]  = std::max(m_hapWheelPk[4][w], m_haptics.wheelFlatMm(w));
            m_hapWheelPk[10][w] = std::max(m_hapWheelPk[10][w], m_haptics.wheelsModel().discHeat(w));
            m_hapWheelPk[11][w] = std::max(m_hapWheelPk[11][w], m_haptics.wheelsModel().punctured(w) ? 1.0 : 0.0);
        }
        if (live(FxType::Impacts)) m_hapWheelPk[5][w] = std::max(m_hapWheelPk[5][w], m_haptics.impactModel().ringLevel(w));
        m_hapWheelPk[6][w] = std::max(m_hapWheelPk[6][w], 1.0 - m_haptics.groundContact(w));   // been in the air
        m_hapWheelPk[7][w] = m_haptics.surfaceModel().surfaceClass(w);                           // (no peak: as now)
        if (live(FxType::Surface))
        {
            m_hapWheelPk[8][w] = std::max(m_hapWheelPk[8][w], m_haptics.surfaceModel().water(w));
            m_hapWheelPk[9][w] = std::max(m_hapWheelPk[9][w], m_haptics.surfaceModel().floating(w));
        }
        if (live(FxType::Skid)) m_hapWheelPk[12][w] = std::max(m_hapWheelPk[12][w], m_hapLaws.latAngle[w]);
        m_hapWheelPk[13][w] = m_hapLaws.latBase[w];                                              // (no peak: as now)
    }
    // A slip channel means nothing at a standstill (a stationary car's slip
    // is a division by almost zero: billions on the wire), so its session
    // peak only counts while the car is moving; the peak is the number a
    // user sets peak % from.
    const bool moving = m_ncxHaveSnapshot[NcxValues::SpeedKmh] && m_ncxValSnapshot[NcxValues::SpeedKmh] > 5.0;
    const auto slipToken = [](int t)
    {
        return t == NcxValues::Skid || t == NcxValues::Lockup
            || (t >= NcxValues::SlipAngleFL && t <= NcxValues::SlipRatioRR)
            || (t >= NcxValues::WheelSlipFL && t <= NcxValues::WheelSlipRR);
    };
    for (int i = 0; i < NcxValues::TokenCount; ++i)
    {
        if (!m_ncxHaveSnapshot[i] || (!moving && slipToken(i))) continue;
        m_ncxPk[i] = std::max(m_ncxPk[i], std::fabs(m_ncxValSnapshot[i]));
    }
}

void MotionController::startUnpark(A6Drive** drives, int numHwDrives)
{
    if (estopHolds())
    {
        RT_LOG_WARNING("MotionController: Unpark refused -- e-stop tripped; re-initialize the drives to resume.");
        return;
    }
    RT_LOG_INFO("MotionController: Starting unpark sequence...");
    for (int i = 0; i < m_numDrives; ++i)
    {
        // Belt/torque axes NEVER tension via unpark (Docs/COMMAND_CONTRACT.md)
        // -- neither the post-homing auto-unpark nor /api/unpark. Otherwise
        // e-stop release and /api/home would become hidden tension triggers.
        // TensionBelts is the belt's one and only re-tension path; it is also
        // the only thing that clears a guard-trip latch. Belts stay slack-
        // parked here.
        if (m_axisConfig[i].torqueMode) continue;

        // Never unpark an axis with no valid position reference. In the normal
        // flow this cannot happen (loop stop re-arms needsRehome, loop start
        // homes, and both park buttons require a running loop), but a homing
        // FatalError leaves that axis PARKED+unhomed while its peers finish and
        // also sit PARKED -- which reads as "all parked", so the toggle offers
        // Unpark. Unparking ramps to centerPos, and for an unhomed axis that
        // target is in a frame with no relation to the machine. Hold instead.
        if (!m_runtime[i].homed)
        {
            RT_LOG_WARNING("MotionController: Axis %d not unparked -- never homed "
                           "(no position reference). Re-home first.", i + 1);
            continue;
        }

        if (m_axisState[i] == AxisMotionState::PARKED)
        {
            AxisRuntime& rt = m_runtime[i];
            AxisConfig& ac = m_axisConfig[i];

            // Seed from rt.currentPos (the position the PARKED state is holding),
            // NOT the drive's actual position: for the last axis to home, the
            // drive can still sit at the raw home position while rt.currentPos
            // already carries the commanded hold point, so seeding from the
            // encoder would command a backward step on the first UNPARKING cycle.
            double startPos = rt.currentPos;
            RT_LOG_INFO("MotionController: Axis %d unpark from rt.currentPos=%.3f (drive actual=%.3f)",
                i + 1, rt.currentPos,
                drives && i < numHwDrives && drives[i] ? drives[i]->getActualPosition() : rt.currentPos);

            rt.interpStart = startPos;
            rt.currentPos = startPos;
            rt.interpTarget = ac.centerPos;
            rt.interpElapsed = 0.0;
            rt.interpDuration = ac.unparkTimeSec;
            m_axisState[i] = AxisMotionState::UNPARKING;
        }
    }
}

void MotionController::startPark()
{
    RT_LOG_INFO("MotionController: Starting park sequence...");
    for (int i = 0; i < m_numDrives; ++i)
    {
        AxisMotionState& state = m_axisState[i];
        // Catch every "in-motion" state, BLENDING included: the auto-unpark on loop
        // start runs UNPARKING -> BLENDING -> ONLINE and the blend lasts seconds, so a
        // Stop Loop can easily land mid-blend. An axis skipped here never reaches
        // PARKED -> "Park timeout" -> axis stranded at centre -> the deinit seat can
        // then false-latch on a mid-stroke gravity-hold and drop the axis.
        if (state == AxisMotionState::ONLINE ||
            state == AxisMotionState::UNPARKING ||
            state == AxisMotionState::BLENDING ||
            state == AxisMotionState::TESTING)
        {
            AxisRuntime& rt = m_runtime[i];
            AxisConfig& ac = m_axisConfig[i];

            if (!ac.caps.positionPark)
            {
                // Ease tension to 0 over parkTime (ramp driven in the PARKING case from
                // rt.lastTension) instead of dropping torque in one step.
                rt.interpElapsed  = 0.0;
                rt.interpDuration = ac.parkTimeSec;
                state             = AxisMotionState::PARKING;
                RT_LOG_INFO("MotionController: Axis %d (belt) parking -- easing slack.", i + 1);
            }
            else
            {
                rt.interpStart = rt.currentPos;
                rt.interpTarget = ac.parkPos;
                rt.interpElapsed = 0.0;
                rt.interpDuration = ac.parkTimeSec;
                state = AxisMotionState::PARKING;
            }
        }
    }
}

// ---- Belt slack/tension: don-doff flow, scoped to torque axes ONLY ------------
// Rig-level command pair (get in -> slack -> buckle up -> tension) that leaves the
// position axes completely untouched -- the session keeps running. Slack reuses the
// belt PARKING ramp (ease tension to 0 over parkTime; PARKED = zero torque, drive
// enabled, spool pulls freely by hand -- CST has no position hold, so donning trips
// nothing). Tension re-enters through BLENDING (0 -> live tension over blendTime),
// the same gentle ramp as session start. Surface-agnostic by design: web button,
// PC app, GPIO pin, or a HID button box all just send the same
// command -- see /api/belts/slack + /api/belts/tension.

void MotionController::slackBelts()
{
    for (int i = 0; i < m_numDrives; ++i)
    {
        // A belt is a belt-TYPED TORQUE axis (Stage C re-key). Mode alone
        // must not grip an axis here: the 0.9.5 device family (shifter,
        // active pedal) is torque-mode but must never be slacked/tensioned
        // by the belts button. The degenerate mismatches (belt type without
        // torque mode and vice versa) become no-ops, the safe direction.
        if (!(m_axisConfig[i].caps.beltType && m_axisConfig[i].torqueMode))
            continue;
        AxisMotionState& state = m_axisState[i];
        if (state == AxisMotionState::ONLINE ||
            state == AxisMotionState::BLENDING ||
            state == AxisMotionState::UNPARKING)
        {
            m_runtime[i].interpElapsed  = 0.0;
            m_runtime[i].interpDuration = m_axisConfig[i].parkTimeSec;
            state = AxisMotionState::PARKING;   // belt PARKING = ease tension to 0
            RT_LOG_INFO("MotionController: Axis %d (belt) slacking for don/doff.", i + 1);
        }
    }
}

void MotionController::tensionBelts()
{
    if (estopHolds())
    {
        RT_LOG_WARNING("MotionController: TensionBelts refused -- e-stop tripped; re-initialize the drives to resume.");
        return;
    }
    for (int i = 0; i < m_numDrives; ++i)
    {
        // A belt is a belt-TYPED TORQUE axis (Stage C re-key). Mode alone
        // must not grip an axis here: the 0.9.5 device family (shifter,
        // active pedal) is torque-mode but must never be slacked/tensioned
        // by the belts button. The degenerate mismatches (belt type without
        // torque mode and vice versa) become no-ops, the safe direction.
        if (!(m_axisConfig[i].caps.beltType && m_axisConfig[i].torqueMode))
            continue;
        AxisMotionState& state = m_axisState[i];
        // PARKED = slack; PARKING = mid-slack (flip back, blend restarts from 0 --
        // the slew cap keeps the transition smooth either way).
        if (state == AxisMotionState::PARKED || state == AxisMotionState::PARKING)
        {
            AxisRuntime& rt = m_runtime[i];
            rt.blendElapsed   = 0.0;
            rt.blendDuration  = m_blendTimeSec;
            rt.lastTension    = 0.0;
            rt.onlineHadData  = false;
            rt.onlineStaleSec = 0.0;
            rt.beltMotionSeen     = false;
            rt.beltMotionStaleSec = 0.0;
            state = AxisMotionState::BLENDING;  // torque branch ramps 0 -> tension
            rt.beltHaveRaw      = false;        // re-seed guards (derivative + travel ref)
            rt.beltOverspeedSec = 0.0;
            rt.beltDwellSec     = 0.0;
            rt.beltRelaxed      = false;
            rt.beltLastTripWhy  = 0;            // card guard flag clears on tension-up
            for (auto& c : rt.beltGCh) c.live = c.meanLive = false;   // the g filters start afresh (not from the last session's braking)
            rt.resetBeltTeleport();   // ...and the teleport test (a speed from the last session is no jump)
            RT_LOG_INFO("MotionController: Axis %d (belt) tensioning -- blending in.", i + 1);
        }
    }
}

// ---- Device engage/release: force field on/off, scoped to device axes ONLY ----
// The device mirror of the belt slack/tension pair. PARKED = limp (zero torque,
// the lever swings freely); engage blends the force field in over blendTime
// (forceScale 0 -> 1 inside stepDeviceBlending); release ramps the last
// commanded force to 0 over parkTime. Surface-agnostic like the belt pair --
// see /api/device/engage + /api/device/release.

void MotionController::engageDevices(int axisIndex)
{
    if (estopHolds())
    {
        RT_LOG_WARNING("MotionController: EngageDevice refused -- e-stop tripped; re-initialize the drives to resume.");
        return;
    }
    int start = (axisIndex < 0) ? 0 : axisIndex;
    int end   = (axisIndex < 0) ? m_numDrives : axisIndex + 1;
    for (int i = start; i < end && i < m_numDrives; ++i)
    {
        if (!m_axisConfig[i].caps.isDevice()) continue;
        if (!m_runtime[i].homed)
        {
            // The deliberate press IS the homing authorization: an unhomed
            // device starts its gentle stall-search here (and ends LIMP -
            // the next press engages). This is the three-state device
            // button: home -> engage -> release.
            if (m_axisState[i] == AxisMotionState::PARKED)
            {
                m_torqueHoming[i].reset();
                m_axisState[i] = AxisMotionState::HOMING;
                RT_LOG_INFO("MotionController: Axis %d (device) homing on engage "
                            "request -- will rest limp when done.", i + 1);
            }
            continue;
        }
        AxisMotionState& state = m_axisState[i];
        // PARKED = limp; PARKING = mid-release (flip back, the blend restarts
        // from 0 -- the model's slew cap keeps the transition smooth).
        if (state == AxisMotionState::PARKED || state == AxisMotionState::PARKING)
        {
            AxisRuntime& rt   = m_runtime[i];
            rt.blendElapsed   = 0.0;
            rt.blendDuration  = m_blendTimeSec;
            rt.lastTension    = 0.0;
            rt.deviceSeeded   = false;   // re-seed the model at the engage position
            state = AxisMotionState::BLENDING;
            RT_LOG_INFO("MotionController: Axis %d (device) engaging -- blending force in.", i + 1);
        }
    }
}

void MotionController::releaseDevices(int axisIndex)
{
    int start = (axisIndex < 0) ? 0 : axisIndex;
    int end   = (axisIndex < 0) ? m_numDrives : axisIndex + 1;
    for (int i = start; i < end && i < m_numDrives; ++i)
    {
        if (!m_axisConfig[i].caps.isDevice()) continue;
        AxisMotionState& state = m_axisState[i];
        if (state == AxisMotionState::ONLINE ||
            state == AxisMotionState::BLENDING)
        {
            m_runtime[i].interpElapsed  = 0.0;
            m_runtime[i].interpDuration = m_axisConfig[i].parkTimeSec;
            state = AxisMotionState::PARKING;   // device PARKING = force ramp to 0
            RT_LOG_INFO("MotionController: Axis %d (device) releasing -- easing to limp.", i + 1);
        }
    }
}

// ---- Belt from g: the shape, and its live apply ---------------------------------

// A g at max held to its range and at least 0.05 g above g from: one rule
// for the belt's own and the type of car's.
static double beltFullGWithin(double fromG, double fullG)
{
    return std::max(fromG + 0.05, std::min(AppConfig::kBeltFullGMax, std::max(AppConfig::kBeltFullGMin, fullG)));
}

MotionController::BeltGShape MotionController::beltGShapeOf(const DriveConfig& dc)
{
    BeltGShape s;
    // Surge: the senders' accSurge reads above zero under braking (as
    // SimHub gives it; the direct read is fitted to SimHub's sign), so
    // "decel" takes the value above zero and "accel" the one below.
    s.mode     = (dc.beltGMode == "off") ? 0 : (dc.beltGMode == "accel") ? 2 : (dc.beltGMode == "both") ? 3 : 1;
    s.sway     = (dc.beltGSway == "left") ? 1 : (dc.beltGSway == "right") ? 2 : (dc.beltGSway == "both") ? 3 : 0;
    s.heave    = (dc.beltGHeave == "press") ? 1 : (dc.beltGHeave == "lift") ? 2 : (dc.beltGHeave == "both") ? 3 : 0;
    s.swayMix  = std::isfinite(dc.beltGSwayMix)  ? std::max(0.0, std::min(3.0, dc.beltGSwayMix))  : 1.0;
    s.heaveMix = std::isfinite(dc.beltGHeaveMix) ? std::max(0.0, std::min(3.0, dc.beltGHeaveMix)) : 1.0;
    // Clamped into their ranges here as well: validation only warns on
    // load, and a curve at or below 0 would ask for more than torque max.
    s.fromG    = std::isfinite(dc.beltGFromG) ? std::max(0.0, std::min(2.0, dc.beltGFromG)) : 0.05;
    s.fullG    = beltFullGWithin(s.fromG, std::isfinite(dc.beltGFullG) ? dc.beltGFullG : 1.5);
    s.curve    = std::isfinite(dc.beltGCurve) ? std::max(0.3, std::min(3.0, dc.beltGCurve)) : 1.0;
    s.smoothMs = std::isfinite(dc.beltGSmoothMs) ? std::max(0.0, std::min(250.0, dc.beltGSmoothMs)) : 30.0;
    return s;
}

void MotionController::putBeltGShape(AxisConfig& ac, const BeltGShape& s)
{
    ac.beltGMode = s.mode; ac.beltGSway = s.sway; ac.beltGHeave = s.heave;
    ac.beltGSwayMix = s.swayMix; ac.beltGHeaveMix = s.heaveMix;
    ac.beltGFromG = s.fromG; ac.beltGFullG = s.fullG; ac.beltGCurve = s.curve; ac.beltGSmoothMs = s.smoothMs;
}

void MotionController::stageBeltG(int axisIndex, const DriveConfig& dc)
{
    // Web thread: only the index here. The RT side checks the axis is a
    // belt of the running config when it takes the stage (the running
    // config belongs to the RT thread and to configure()).
    if (axisIndex < 0 || axisIndex >= MAX_DRIVES) return;
    const BeltGShape s = beltGShapeOf(dc);
    {
        std::lock_guard<std::mutex> lk(m_beltGLock);
        m_beltGPending[axisIndex] = s;
    }
    m_beltGMask.fetch_or(uint16_t(1u << axisIndex), std::memory_order_release);
}

// A save that changes the axes: a shape staged by index no longer names
// the axis it was meant for.
void MotionController::clearStagedBeltG()
{
    m_beltGMask.store(0, std::memory_order_release);
}

// RT thread, top of a cycle. Never waits on the web thread: a save
// mid-copy leaves the shape for the next cycle (the mask clears inside the
// lock, so a stage landing between the copy and the clear is kept). The
// low-pass state carries on (a new smoothing eases from where it is),
// except for a channel switched on: it starts afresh, as at tension-up,
// not from the g it held when it was switched off.
void MotionController::applyPendingBeltG()
{
    if (!m_beltGMask.load(std::memory_order_acquire)) return;
    BeltGShape s[MAX_DRIVES];
    uint16_t mask;
    {
        std::unique_lock<std::mutex> lk(m_beltGLock, std::try_to_lock);
        if (!lk.owns_lock()) return;
        mask = m_beltGMask.exchange(0, std::memory_order_acq_rel);
        for (int i = 0; i < MAX_DRIVES; ++i) if (mask & (1u << i)) s[i] = m_beltGPending[i];
    }
    static const char* const kSurge[4] = { "off", "braking", "acceleration", "both" };
    for (int i = 0; i < m_numDrives; ++i)
    {
        if (!(mask & (1u << i))) continue;
        AxisConfig& ac = m_axisConfig[i];
        if (!ac.caps.beltType) continue;
        const int was[3] = { ac.beltGMode, ac.beltGSway, ac.beltGHeave };
        const int now[3] = { s[i].mode, s[i].sway, s[i].heave };
        for (int k = 0; k < 3; ++k)
            if (was[k] == 0 && now[k] != 0) { m_runtime[i].beltGCh[k].live = false; m_runtime[i].beltGCh[k].meanLive = false; }
        const bool same = ac.beltGMode == s[i].mode && ac.beltGSway == s[i].sway && ac.beltGHeave == s[i].heave
                       && ac.beltGSwayMix == s[i].swayMix && ac.beltGHeaveMix == s[i].heaveMix && ac.beltGFromG == s[i].fromG
                       && ac.beltGFullG == s[i].fullG && ac.beltGCurve == s[i].curve && ac.beltGSmoothMs == s[i].smoothMs;
        putBeltGShape(ac, s[i]);
        if (!same)
            RT_LOG_INFO("MotionController: Axis %d belt from the car's g applied live: surge %s, %.2f..%.2f g, curve %.2f, smoothed %.0f ms",
                        i + 1, kSurge[s[i].mode & 3], s[i].fromG, s[i].fullG, s[i].curve, s[i].smoothMs);
    }
}

// ---- Device live-apply (tuning path) -------------------------------------------

void MotionController::stageDeviceParams(int axisIndex, const DeviceParams& p)
{
    if (axisIndex < 0 || axisIndex >= m_numDrives) return;
    if (!m_axisConfig[axisIndex].caps.isDevice()) return;
    {
        std::lock_guard<std::mutex> lk(m_devApplyLock);
        m_devPending[axisIndex] = p;
    }
    m_devPendingMask.fetch_or(uint16_t(1u << axisIndex), std::memory_order_release);
}

void MotionController::applyPendingDeviceParams()
{
    const uint16_t mask = m_devPendingMask.load(std::memory_order_acquire);
    if (!mask) return;
    for (int i = 0; i < m_numDrives; ++i)
    {
        if (!(mask & (1u << i))) continue;
        // Apply only while LIMP: the force field re-seeds cleanly with the
        // lever at rest; anything staged mid-engage waits for release.
        if (m_axisState[i] != AxisMotionState::PARKED) continue;
        DeviceParams p;
        {
            std::lock_guard<std::mutex> lk(m_devApplyLock);
            p = m_devPending[i];
        }
        m_devPendingMask.fetch_and(uint16_t(~(1u << i)), std::memory_order_release);
        AxisConfig& ac = m_axisConfig[i];
        ac.device = p;
        m_runtime[i].deviceModel.configure(p, m_cycleTimeSec);
        m_runtime[i].deviceState.configure(p);
        m_torqueHoming[i].configure(p, ac.beltUnitsPerRev, m_cycleTimeSec);
        RT_LOG_INFO("MotionController: Axis %d (device) settings applied live.", i + 1);
    }
}

// ---- Belt runaway guards (armed in BLENDING + ONLINE) --------------------------
// Two independent detectors for the same failure (load lost: snapped/detached belt):
//  * Overspeed persistence: |rpm| >= threshold continuously for the window. Catches a
//    fast runaway in ~a fifth of a second; haptic flicks/hand pulls reset the timer.
//  * Net-travel cap: |position - tension-up reference| > N motor revs. Catches what an
//    rpm threshold can't -- a SLOW continuous wind (bare shaft idling at min tension
//    spins below any sane rpm limit). With a belt attached the strap bounds net travel
//    to ~1 rev, so the cap can never fire in legitimate use, at ANY speed.
// Trip: instant torque cut + latched slack (PARKED; explicit tension/unpark resumes).
bool MotionController::beltGuardTripped(int i, A6Drive* d, MotionOutput& output)
{
    if (!d) return false;
    AxisConfig&  ac = m_axisConfig[i];
    AxisRuntime& rt = m_runtime[i];

    const double raw = d->getActualPositionRaw();
    if (!rt.beltHaveRaw)
    {
        rt.beltPrevRaw = raw; rt.beltRefRaw = raw; rt.beltHaveRaw = true;
        return false;
    }

    const double rpm = std::fabs(raw - rt.beltPrevRaw) / ac.beltUnitsPerRev
                     / m_cycleTimeSec * 60.0;
    rt.beltPrevRaw = raw;
    rt.beltLastRpm = rpm;   // fold-back input (see beltVelocityFold)
    const double travelRevs = std::fabs(raw - rt.beltRefRaw) / ac.beltUnitsPerRev;

    const char* why = nullptr;
    if (ac.beltOverspeedRpm > 0.0)
    {
        if      (rpm >= ac.beltOverspeedRpm)       rt.beltOverspeedSec += m_cycleTimeSec;
        else if (rpm <  ac.beltOverspeedRpm * 0.8) rt.beltOverspeedSec  = 0.0;   // hysteresis
        if (rt.beltOverspeedSec * 1000.0 >= ac.beltOverspeedMs) why = "OVERSPEED";
    }
    if (!why && ac.beltMaxTravelRevs > 0.0 && travelRevs >= ac.beltMaxTravelRevs)
        why = "TRAVEL";

    if (!why) return false;

    m_axisState[i]     = AxisMotionState::PARKED;
    rt.lastTension     = 0.0;
    rt.beltHaveRaw     = false;
    rt.beltLastTripWhy = (why[0] == 'O') ? 1 : 2;   // OVERSPEED=1, TRAVEL=2 (card display)
    output.torques[i]  = 0.0;
    RT_LOG_ERROR("MotionController: Axis %d (belt) %s (%.0frpm, %.1f revs net) -- load lost? "
                 "Torque cut, latched slack (unpark/tension to resume).",
                 i + 1, why, rpm, travelRevs);
    return true;
}

// ---- Master-side CST velocity limiter (see header) -----------------------------
// Linear fold: full tension at/below beltMaxRpm, zero at beltMaxRpm + band (band =
// 25% of the knee, >=100rpm). During a slack lunge the loop reacts one cycle
// (500us) after crossing the knee, so speed tops out just above it -- far below
// the drive's ~3600rpm Er06.0 overspeed fault. In normal belt-attached operation
// the spool never approaches the knee, so haptics are untouched. Composes with
// the belt-loss guard: sustained sitting at the knee (belt truly gone) still
// accumulates the overspeed persistence and trips latched slack.
double MotionController::beltVelocityFold(int i, double tension) const
{
    const AxisConfig& ac = m_axisConfig[i];
    if (ac.beltMaxRpm <= 0.0) return tension;
    const double over = m_runtime[i].beltLastRpm - ac.beltMaxRpm;
    if (over <= 0.0) return tension;
    const double band  = std::max(100.0, ac.beltMaxRpm * 0.25);
    const double scale = std::max(0.0, 1.0 - over / band);
    return tension * scale;
}

void MotionController::startSeatHoming(A6Drive** drives, int numHwDrives,
                                       const bool* driveSeatable)
{
    RT_LOG_INFO("MotionController: Deinit seat -- starting seat-mode homing on vertical axes.");
    bool any = false;
    for (int i = 0; i < m_numDrives; ++i)
    {
        m_seatActive[i] = false;
        AxisConfig& ac = m_axisConfig[i];
        A6Drive* d = (drives && i < numHwDrives) ? drives[i] : nullptr;

        // Vertical, CSP position axis on a healthy drive only. Horizontal/belt/torque/PP,
        // sim, or already-faulted axes are skipped -- they de-energize as today and never
        // block allSeatAxesAtStop(). Homing is NOT required first: seat-mode homing is a
        // raw, torque-bounded descent in the configured home direction, no reference needed.
        if (!ac.caps.seatable)  continue;
        if (!d || d->isFault()) continue;

        // Bus-health guard: on a partially degraded bus ControlLoop passes a
        // per-drive eligibility mask (slave confirmed in OP) -- a seat pass on
        // an unreachable slave would run blind on frozen PDO data. nullptr =
        // healthy bus, all eligible.
        if (driveSeatable && i < numHwDrives && !driveSeatable[i])
        {
            RT_LOG_ERROR("MotionController: Axis %d NOT seated -- slave unreachable "
                         "on a degraded bus.", i + 1);
            continue;
        }

        // Run the REAL homing search, in seat mode: presses DOWN to 25% torque exactly as
        // Home, but at the hardstop holds on the stop (no back-off) and sets NO home
        // reference. Driven directly via stepSeatHoming() -> HomingSequence::step(), so
        // processHomingAxis's completion path (homeOffset/homed/PARKED) is never reached.
        m_homing[i].reset();
        m_homing[i].start(d, /*seatMode=*/true);
        m_seatActive[i] = true;
        any = true;
        RT_LOG_INFO("MotionController: Axis %d seat-homing toward stop (de-energize on 25%% torque, no back-off).", i + 1);
    }
    // De-energizing loses the reference, so require a re-home next loop start. (Seat mode
    // itself never establishes a reference, so this is the only thing that sets it.)
    if (any) m_needsRehome = true;
}

void MotionController::stepSeatHoming(A6Drive** drives, int numHwDrives)
{
    for (int i = 0; i < m_numDrives; ++i)
    {
        if (!m_seatActive[i]) continue;
        A6Drive* d = (drives && i < numHwDrives) ? drives[i] : nullptr;
        if (!d) continue;
        m_homing[i].step(d);   // commands the drive (raw, FE-capped) per the homing state machine
    }
}

bool MotionController::allSeatAxesAtStop() const
{
    for (int i = 0; i < m_numDrives; ++i)
    {
        if (!m_seatActive[i]) continue;
        HomingSequence::State s = m_homing[i].getState();
        // Seated = on the stop; Error/FatalError = gave up (that axis just drops, accepted).
        if (s != HomingSequence::State::Seated &&
            s != HomingSequence::State::Error &&
            s != HomingSequence::State::FatalError)
            return false;
    }
    return true;
}

// ---- Phase 2: ease the press off to "resting" (no HomingSequence involvement) ----
// FLOOR  : |torque| this low => clearly resting on the stop, stop easing.
// RISE   : once |torque| climbs this far back above its minimum, the axis has lifted off
//          the stop and is now holding its own weight -> we've passed the resting point.
// CAP_MM : hard limit on ease-off travel so a mis-read can never lift the axis far (the
//          worst-case drop). The bottom-detector normally trips well before this.
static constexpr double SEAT_RELIEF_FLOOR_PCT = 4.0;
static constexpr double SEAT_RELIEF_RISE_PCT  = 3.0;
static constexpr double SEAT_RELIEF_CAP_MM    = 1.0;
// PLATEAU exit: a light axis that never fully unloads onto its stop (the low-weight rear
// actuator, or an operator leaning on the seat) can hold a steady torque inside the
// FLOOR..RISE band while not moving -- none of the three conditions above ever trip, so it
// would ride the de-init cap for the full backstop. If torque stops dropping (no new low
// beyond EPS) AND the axis stops moving (no new ease-off travel beyond EPS) for STALL_SEC,
// it is as rested as it will get -> exit and de-energize. This only makes a stuck axis
// finish sooner; it can never delay one (any exit just de-energizes at the current point).
// EPS values sit just above encoder/torque read noise so jitter can't reset the timer.
static constexpr double SEAT_RELIEF_EPS_PCT   = 0.5;
static constexpr double SEAT_RELIEF_EPS_MM    = 0.02;
static constexpr double SEAT_RELIEF_STALL_SEC = 1.0;

void MotionController::startSeatRelief(A6Drive** drives, int numHwDrives)
{
    for (int i = 0; i < m_numDrives; ++i)
    {
        if (!m_seatActive[i]) continue;
        A6Drive* d = (drives && i < numHwDrives) ? drives[i] : nullptr;
        m_runtime[i].seatReliefDone     = (d == nullptr);              // no drive -> nothing to ease
        m_runtime[i].seatReliefStartRaw = d ? d->getActualPositionRaw() : 0.0;
        m_runtime[i].seatReliefMinTrq   = d ? std::fabs(d->getTorquePercent()) : 0.0;
        m_runtime[i].seatReliefLastEased = 0.0;
        m_runtime[i].seatReliefStallSec  = 0.0;
    }
    RT_LOG_INFO("MotionController: Deinit seat -- easing the press off to resting.");
}

void MotionController::stepSeatRelief(A6Drive** drives, int numHwDrives)
{
    for (int i = 0; i < m_numDrives; ++i)
    {
        if (!m_seatActive[i] || m_runtime[i].seatReliefDone) continue;
        A6Drive* d = (drives && i < numHwDrives) ? drives[i] : nullptr;
        if (!d || d->isFault()) { m_runtime[i].seatReliefDone = true; continue; }

        const double torque = std::fabs(d->getTorquePercent());
        const double actual = d->getActualPositionRaw();
        const double eased  = std::fabs(actual - m_runtime[i].seatReliefStartRaw);

        // Progress = a meaningful new torque low OR meaningful new ease-off travel (both
        // EPS-filtered so read noise never counts). seatReliefMinTrq is updated only on a
        // real drop, so it doubles as the plateau reference and the bottom for lift-off.
        bool progressed = false;
        if (torque < m_runtime[i].seatReliefMinTrq - SEAT_RELIEF_EPS_PCT)
        { m_runtime[i].seatReliefMinTrq = torque; progressed = true; }
        if (eased  > m_runtime[i].seatReliefLastEased + SEAT_RELIEF_EPS_MM)
        { m_runtime[i].seatReliefLastEased = eased; progressed = true; }
        m_runtime[i].seatReliefStallSec = progressed ? 0.0
                                        : m_runtime[i].seatReliefStallSec + m_cycleTimeSec;

        const bool floored   = (torque <= SEAT_RELIEF_FLOOR_PCT);                                // torque essentially gone
        const bool liftedOff = (torque >= m_runtime[i].seatReliefMinTrq + SEAT_RELIEF_RISE_PCT); // bottomed -> lifting off
        const bool travelCap = (eased  >= SEAT_RELIEF_CAP_MM);                                   // ease-off safety cap
        const bool plateau   = (m_runtime[i].seatReliefStallSec >= SEAT_RELIEF_STALL_SEC);       // stuck, as rested as it gets
        if (floored || liftedOff || travelCap || plateau)
        {
            m_runtime[i].seatReliefDone = true;
            d->setTargetPositionRaw(actual);   // hold at the just-resting point
            const char* why = floored ? "floor" : liftedOff ? "lift-off" : travelCap ? "cap" : "plateau";
            RT_LOG_INFO("MotionController: Axis %d eased to rest (%s: torque=%.0f%%, eased=%.2fmm).",
                        i + 1, why, torque, eased);
        }
        else
        {
            // Ease AWAY from the stop (opposite the press / home-search direction), one
            // homing step per cycle so following error stays capped -- same gentle pace.
            // Away from the stop = opposite of the search direction, from the
            // same helper homing itself uses (this was the one site that
            // hand-flipped homeDirection before the sign became centralised).
            const double reliefSign = -HomingSequence::searchSign(
                m_axisConfig[i].invertDir, m_axisConfig[i].homeDirection);
            d->setTargetPositionRaw(actual + reliefSign * m_homing[i].homingStepMm());
        }
    }
}

bool MotionController::allSeatAxesRelieved() const
{
    for (int i = 0; i < m_numDrives; ++i)
        if (m_seatActive[i] && !m_runtime[i].seatReliefDone) return false;
    return true;
}

// ============================================================
// S-curve trajectory planner
//
// Each cycle, advances position toward targetPos with:
//   - Velocity limited to maxVelocityMmS
//   - Acceleration limited to maxAccelMmS2
//   - Jerk (rate of acceleration change) limited to maxJerkMmS3
//
// This produces continuous position, velocity, and acceleration
// profiles. The drive sees smooth force changes instead of
// impulses, resulting in quiet operation.
//
// The approach: at each cycle, compute the desired acceleration
// to reach the target, then clamp the jerk (change in accel)
// to produce smooth transitions. This is simpler than full
// 7-phase s-curve planning and works well for the use case of
// continuously updating waypoints at ~60Hz.
// ============================================================
// ============================================================
// conditionCommand -- dispatch the live target through the selected CSP
// conditioning mode into the shared guard chain. Used by both ONLINE (velCap =
// maxVelocity) and BLENDING (velCap = the ramped blend cap). The guard chain
// (Amax / Vmax / relative braking / stroke) is identical across modes; only the
// pre-clamp command differs. frameSec = the host nominal frame interval.
// ============================================================
double MotionController::conditionCommand(CommandConditioner& cond, double target,
                                          double frameSec, double velCap, const AxisConfig& ac)
{
    const double brakeEps = 4.0 / ac.countsPerMm;
    switch (m_conditioningMode)
    {
    case CommandConditioner::Mode::Bypass:
        return cond.stepBypass(target, m_cycleTimeSec, frameSec, velCap, ac.maxAccelMmS2, brakeEps);
    case CommandConditioner::Mode::Interpolate:
        return cond.stepInterpolate(target, m_cycleTimeSec, frameSec, velCap, ac.maxAccelMmS2, brakeEps);
    case CommandConditioner::Mode::Filter:
    default:
        return cond.stepFilter(target, m_cycleTimeSec, frameSec, 2.0 * M_PI * ac.trackingWnHz,
                               velCap, ac.maxAccelMmS2, brakeEps);
    }
}

double MotionController::stepTrajectory(TrajectoryState& traj, const AxisConfig& ac, double velCap)
{
    double dt = m_cycleTimeSec;
    double maxV = (velCap > 0.0) ? std::min(velCap, ac.maxVelocityMmS) : ac.maxVelocityMmS;
    double maxA = ac.maxAccelMmS2;
    double maxJ = ac.maxJerkMmS3;

    double posError = traj.targetPos - traj.pos;

    double v = traj.vel;
    double a = traj.accel;
    double absV = std::abs(v);
    double errorSign = (posError > 0.0) ? 1.0 : -1.0;
    double absError = std::abs(posError);

    if (absError < 0.001 && absV < 0.01)
    {
        // At target with negligible velocity -- hold
        traj.pos = traj.targetPos;
        traj.vel = 0.0;
        traj.accel = 0.0;
        return traj.pos;
    }

    // Braking-curve controller:
    // The braking curve gives the maximum speed allowed at distance d from target:
    //   brakingVel = sqrt(2 * maxA * d)
    // If current speed exceeds this we MUST decelerate at maxA to stop in time.
    // If current speed is within the curve, accelerate toward target at maxA.
    // Bang-bang acceleration choice avoids the /dt amplification that caused
    // oscillation with the (targetVel - v) / dt formulation.
    double brakingVel = std::sqrt(2.0 * maxA * absError);
    double speedAlongError = v * errorSign;  // positive = moving toward target

    double desiredAccel;
    if (speedAlongError > brakingVel)
        desiredAccel = -errorSign * maxA;   // overspeed -- brake hard
    else
        desiredAccel = errorSign * maxA;    // within curve -- accelerate toward target

    // Clamp acceleration magnitude
    desiredAccel = std::max(-maxA, std::min(maxA, desiredAccel));

    // Apply jerk limit: limit the CHANGE in acceleration per cycle
    double maxJerkPerCycle = maxJ * dt;
    double accelChange = desiredAccel - a;
    if (std::abs(accelChange) > maxJerkPerCycle)
    {
        accelChange = (accelChange > 0.0) ? maxJerkPerCycle : -maxJerkPerCycle;
    }

    // Update acceleration
    a += accelChange;
    a = std::max(-maxA, std::min(maxA, a));

    // Update velocity
    v += a * dt;
    v = std::max(-maxV, std::min(maxV, v));

    // Update position
    double newPos = traj.pos + v * dt;

    // Clamp position to limits
    newPos = std::max(ac.minPos, std::min(ac.maxPos, newPos));

    // If we hit a position limit, zero velocity and acceleration
    if (newPos <= ac.minPos || newPos >= ac.maxPos)
    {
        if ((newPos <= ac.minPos && v < 0.0) || (newPos >= ac.maxPos && v > 0.0))
        {
            v = 0.0;
            a = 0.0;
        }
    }

    traj.pos = newPos;
    traj.vel = v;
    traj.accel = a;

    return newPos;
}

// ============================================================
// Per-family step functions (Stage D decomposition)
//
// Verbatim extractions from the old process() switch -- expression order
// preserved on purpose: the golden-sequence pins in TestMotionController
// hold these bit-identical to the pre-decomposition monolith.
// ============================================================

bool MotionController::axisFrameUsable(int i, const TelemetryData& td) const
{
    return telemetryFrameUsable(td) && i < td.numPositions
        && std::isfinite(td.positions[i]);
}

double MotionController::decodeTargetMm(int i, double raw)
{
    AxisConfig& ac = m_axisConfig[i];
    // 16-bit only (the wire contract): 0..65535, centre 32767 = centerPos.
    double norm = (raw - TELEMETRY_CENTER) / TELEMETRY_CENTER;
    double targetMm = ac.centerPos + norm * (ac.strokeMm / 2.0);
    if (ac.telemetryInvert)
        targetMm = ac.centerPos - (targetMm - ac.centerPos);
    if (ac.spikeFilterEnabled)
        targetMm = applySpikeFilter(i, targetMm);
    return std::max(ac.minPos, std::min(ac.maxPos, targetMm));
}

// A car pulls well under 8 g outside a crash; a value past it is a junk
// frame (a reset, a glitch), not a cue.
static constexpr double kBeltGJunkMs2 = 8.0 * 9.80665;
// One junk frame does not restart the g smoothing; a gap this long does.
static constexpr double kBeltGReseedSec = 0.1;
// A sender that stops sending one of the g channels leaves the parser
// holding its last value while the rest of the stream lives on: a value
// that has not changed in this long reads as no g (the belt eases to
// minimum, ONLINE; never the stale stages, which would park a belt slack
// while the car sits in the pits).
static constexpr double kBeltGFrozenSec = 0.5;
// Heave is felt as its change from the steady part (gravity, a long
// banking, a hill), taken as its mean over about this long.
static constexpr double kBeltGHeaveMeanSec = 2.0;
// A teleport (the sim puts the car somewhere else: a reset, a restart, a
// recovery to the road) brings a g far past any car's that then decays
// for about half a second (the sender's g is the slope of a velocity that
// jumped). The junk limit drops the peak but lets the tail through:
// several g, a full pull on the belt. A teleport shows as a speed jump no
// car makes, or the road under the car jumping in height; either holds
// the g off this long: the belt sits at minimum, the tail goes by, and
// the g is taken afresh after it. A g past the junk limit alone is not a
// sign: a car striking a rock or a tree pulls 10 to 60 g in the sim (AC
// Rally, 2026-10-27), and that is felt.
static constexpr double kBeltGTeleportHoldSec = 1.0;
// The speed changing by more than this from one value the sender sends to
// the next is a teleport (AC Rally 2026-10-27: resets 47 to 93 km/h in one
// frame; the hardest crashes 21 km/h). However long between them: a game
// paused resumes at the speed it stopped at, and a restart comes after
// the stream paused to load it, or with the speed frozen meanwhile. (A
// long dropout while the speed changes that much reads as one too: a
// second at minimum, a release.)
static constexpr double kBeltGTeleportKmh = 30.0;
// The road's height under the car moving more than this on two wheels or
// more at once, standing still (under kBeltGTeleportStillKmh), is a restart
// in place (AC Rally: 36 to 438 m, the speed 0 before and after). Only
// standing still: on the move the speed shows a reset, and a landing from
// a jump can step the contact points. A wheel's height reading exactly 0
// is no value (AC Rally drops one corner's contact point for a frame,
// even at 123 km/h).
static constexpr double kBeltGTeleportRoadMm   = 2000.0;
static constexpr double kBeltGTeleportStillKmh = 10.0;

// The belt's live tension demand this cycle from its source (Config.h
// beltSource): the motion line's slot for this axis, or the car's g.
// False when there is none (no usable frame; the channel stream stale,
// or silent or junk on every g the belt takes): the caller stages the
// standby exactly as for a lost motion line. RT thread only.
bool MotionController::beltDemand(int i, const TelemetryData& td, double& tension)
{
    const AxisConfig& ac = m_axisConfig[i];
    if (!ac.beltFromG)
    {
        if (!axisFrameUsable(i, td)) return false;
        tension = beltTension(td.positions[i], ac.torqueMinPct, ac.torqueMaxPct);
        return true;
    }
    AxisRuntime& rt = m_runtime[i];
    const NcxValues v = m_ncxMap.extract(td);
    static constexpr NcxValues::Token kTok[3] = { NcxValues::AccSurge, NcxValues::AccSway, NcxValues::AccHeave };
    const int    mode[3] = { ac.beltGMode, ac.beltGSway, ac.beltGHeave };
    const double mix[3]  = { 1.0, ac.beltGSwayMix, ac.beltGHeaveMix };
    bool usable[3] = {}, anyUsable = false;
    for (int k = 0; k < 3; ++k)
    {
        if (mode[k] == 0) continue;
        const double a = v.val[kTok[k]];
        usable[k] = v.fresh && v.have[kTok[k]] && std::isfinite(a) && std::fabs(a) <= kBeltGJunkMs2;
        anyUsable = anyUsable || usable[k];
    }
    const double dt = m_cycleTimeSec;
    // A teleport: the speed or the road's height jumping from one value the
    // sender sends to the next (kBeltGTeleport*).
    bool teleport = false, anyPresent = false;
    for (int k = 0; k < 3; ++k)
        if (mode[k] != 0 && v.fresh && v.have[kTok[k]]) anyPresent = true;
    if (v.fresh)
    {
        if (v.have[NcxValues::SpeedKmh] && std::isfinite(v.val[NcxValues::SpeedKmh]))
        {
            const double s = v.val[NcxValues::SpeedKmh];
            if (s != rt.beltLastSpeedKmh)
            {
                if (rt.beltSpeedSeen && std::fabs(s - rt.beltLastSpeedKmh) > kBeltGTeleportKmh) teleport = true;
                rt.beltLastSpeedKmh = s;
                rt.beltSpeedSeen    = true;
            }
        }
        // The road's height under each wheel: two or more jumping at once,
        // standing still.
        const bool still = v.have[NcxValues::SpeedKmh] && std::isfinite(v.val[NcxValues::SpeedKmh])
                        && std::fabs(v.val[NcxValues::SpeedKmh]) < kBeltGTeleportStillKmh;
        int jumped = 0;
        for (int w = 0; w < 4; ++w)
        {
            const int tk = NcxValues::RoadHeightFL + w;
            if (!v.have[tk] || !std::isfinite(v.val[tk]) || v.val[tk] == 0.0) continue;
            const double h = v.val[tk];
            if (h == rt.beltLastRoadMm[w]) continue;
            if (rt.beltRoadSeen[w] && std::fabs(h - rt.beltLastRoadMm[w]) > kBeltGTeleportRoadMm) ++jumped;
            rt.beltLastRoadMm[w] = h;
            rt.beltRoadSeen[w]   = true;
        }
        if (still && jumped >= 2) teleport = true;
    }
    if (teleport)
    {
        if (rt.beltTeleportSec <= 0.0)
            RT_LOG_INFO("MotionController: Axis %d belt: the car teleported (a speed or road-height jump): its g is held off for %.1f s",
                        i + 1, kBeltGTeleportHoldSec);
        rt.beltTeleportSec = kBeltGTeleportHoldSec;
    }
    if (rt.beltTeleportSec > 0.0)
    {
        rt.beltTeleportSec -= dt;
        for (AxisRuntime::BeltGChan& c : rt.beltGCh) { c.live = false; c.meanLive = false; c.badSec = 0.0; }   // afresh after it
        if (!anyPresent) return false;   // the stream gone as well: the standby, as ever
        tension = beltGTension(i, 0.0);
        return true;
    }
    const double smoothA = (ac.beltGSmoothMs > 0.0) ? dt / (ac.beltGSmoothMs / 1000.0 + dt) : 1.0;
    double x = 0.0;
    for (int k = 0; k < 3; ++k)
    {
        if (mode[k] == 0) continue;
        AxisRuntime::BeltGChan& c = rt.beltGCh[k];
        if (!usable[k])
        {
            // Held as it was for a junk frame, dropped after a real gap;
            // heave's mean kept through a pause, dropped after a long one.
            c.badSec += dt;
            if (c.badSec > kBeltGReseedSec)    c.live = false;
            if (c.badSec > kBeltGHeaveMeanSec) c.meanLive = false;
        }
        else
        {
            c.badSec = 0.0;
            const double raw = v.val[kTok[k]];
            const double rawG = raw / 9.80665;
            double g = rawG;
            // Unchanged too long: no g, through the smoothing (no step
            // either way): surge and sway 0, heave its steady part (the
            // value itself while there is none yet, so a parked car's
            // gravity never reads as a g).
            if (raw != c.lastRaw) { c.lastRaw = raw; c.sameSec = 0.0; }
            else if ((c.sameSec += dt) > kBeltGFrozenSec)
                g = (k != 2) ? 0.0 : c.meanLive ? c.mean : rawG;
            if (!c.live) c.f = g;
            else         c.f += (g - c.f) * smoothA;
            if (k == 2)
            {
                if (!c.meanLive)  c.mean = g;
                else if (c.live)  c.mean += (g - c.mean) * (dt / (kBeltGHeaveMeanSec + dt));
                c.meanLive = true;
            }
            c.live = true;
        }
        if (!c.live) continue;
        const double s = (k == 2) ? c.f - c.mean : c.f;
        const double folded = (mode[k] == 1) ? std::max(0.0, s)
                            : (mode[k] == 2) ? std::max(0.0, -s)
                            : std::fabs(s);
        x += mix[k] * folded;
    }
    // Unusable on every g the belt takes: the standby stages, as a lost line
    // (nothing selected never has a demand: the belt waits at minimum).
    if (!anyUsable) return false;
    tension = beltGTension(i, x);
    return true;
}

// Belt from g: x is what tightens it, in g (each axis's g low-passed over
// beltGSmoothMs, folded by its mode, sway and heave at their share),
// mapped from beltGFromG to beltGFullG onto torqueMinPct..torqueMaxPct (a
// straight line at curve 1).
double MotionController::beltGTension(int i, double x)
{
    const AxisConfig& ac = m_axisConfig[i];
    if (!std::isfinite(x)) x = 0.0;
    // The type of car's g at max wins over the belt's own while it is set.
    const double full = (m_beltTypeFullG > 0.0) ? beltFullGWithin(ac.beltGFromG, m_beltTypeFullG) : ac.beltGFullG;
    double u = std::max(0.0, std::min(1.0, (x - ac.beltGFromG) / std::max(1e-6, full - ac.beltGFromG)));
    if (ac.beltGCurve != 1.0) u = std::pow(u, ac.beltGCurve);
    const double t = ac.torqueMinPct + (ac.torqueMaxPct - ac.torqueMinPct) * u;
    // Never outside the belt's own range, whatever the settings say.
    return std::isfinite(t) ? std::max(ac.torqueMinPct, std::min(ac.torqueMaxPct, t)) : ac.torqueMinPct;
}

double MotionController::stepBeltBlending(int i, AxisMotionState& state,
    const TelemetryData& telemetryData, MotionOutput& output,
    A6Drive** drives, int numHwDrives)
{
    AxisConfig& ac = m_axisConfig[i];
    AxisRuntime& rt = m_runtime[i];

    // Runaway guards armed from the first tensioning cycle (a snapped belt
    // can spin up during the blend just as well as ONLINE).
    if (beltGuardTripped(i, (drives && i < numHwDrives) ? drives[i] : nullptr, output))
        return rt.currentPos;

    // Belt: tension blends in over the same window a position axis uses to
    // ease from center into live tracking -- torque ramps 0 -> live tension
    // across blendDuration, so the belt never snaps taut on enable.
    rt.blendElapsed += m_cycleTimeSec;
    double bf = std::max(0.0, std::min(1.0, rt.blendElapsed / rt.blendDuration));
    double tension = ac.torqueMinPct;
    const bool haveData = beltDemand(i, telemetryData, tension);
    if (!haveData) tension = ac.torqueMinPct;
    if (haveData) rt.onlineHadData = true;
    if (ac.beltFromG) noteBeltMotionStream(i, telemetryData);
    // Store the RAMPED value so the ONLINE slew cap starts from what was
    // actually commanded at the blend seam (no step at handoff). Velocity
    // fold applies during the blend too -- a slack lunge doesn't wait for
    // ONLINE.
    rt.lastTension    = beltVelocityFold(i, bf * tension);
    output.torques[i] = rt.lastTension;
    if (bf >= 1.0)
    {
        state = AxisMotionState::ONLINE;
        rt.onlineStaleSec = 0.0;
        // Guards stay armed straight through -- they were seeded at
        // tension-up (BLENDING entry); resetting here would discard the
        // net-travel reference mid-session.
        RT_LOG_INFO("MotionController: Axis %d (belt) blend complete -- ONLINE.", i + 1);
    }
    return rt.currentPos;
}

double MotionController::stepPositionBlending(int i, AxisMotionState& state,
    const TelemetryData& telemetryData)
{
    AxisConfig& ac = m_axisConfig[i];
    AxisRuntime& rt = m_runtime[i];

    rt.blendElapsed += m_cycleTimeSec;
    double blendFactor = rt.blendElapsed / rt.blendDuration;

    // BLENDING runs the SAME tracking filter as ONLINE, with the velocity
    // cap ramped from blendMaxVelocity up to maxVelocity over the blend.
    // Seeded at the blend-start position (center) on entry, the rig glides
    // smoothly from center into live tracking -- no crossfade "miniature
    // replay", no bang-bang oscillation, and no separate ONLINE handoff
    // (the same filter just continues once the cap reaches full).
    double targetMm = rt.currentPos;   // hold if no data
    bool haveData = axisFrameUsable(i, telemetryData);
    if (haveData)
    {
        rt.onlineHadData = true;
        targetMm = decodeTargetMm(i, telemetryData.positions[i]);
    }

    double bf = std::max(0.0, std::min(1.0, blendFactor));
    double velCap = ac.blendMaxVelocityMmS
                  + bf * (ac.maxVelocityMmS - ac.blendMaxVelocityMmS);
    if (velCap < 1.0) velCap = 1.0;

    // Same selected conditioning mode as ONLINE, but with the ramped blend
    // velocity cap -- so the unpark ease-in works in every mode and the
    // BLENDING->ONLINE seam is seamless (same conditioner, cap reaches full).
    double outPos = conditionCommand(rt.onlineCond, targetMm,
                                     telemetryData.nominalFrameSec, velCap, ac);
    outPos = std::max(ac.minPos, std::min(ac.maxPos, outPos));

    if (blendFactor >= 1.0)
    {
        // Cap has reached full -- this IS ONLINE now. The same filter
        // continues next cycle; nothing to re-seed, no discontinuity.
        state = AxisMotionState::ONLINE;
        rt.onlineStaleSec = 0.0;
        RT_LOG_INFO("MotionController: Axis %d blend complete -- ONLINE.", i + 1);
    }
    return outPos;
}

void MotionController::noteBeltMotionStream(int i, const TelemetryData& td)
{
    AxisRuntime& rt = m_runtime[i];
    if (telemetryFrameUsable(td))
    {
        rt.beltMotionSeen     = true;
        rt.beltMotionStaleSec = 0.0;
    }
    else if (rt.beltMotionSeen)
        rt.beltMotionStaleSec += m_cycleTimeSec;
}

double MotionController::stepBeltOnline(int i, AxisMotionState& state,
    const TelemetryData& telemetryData, MotionOutput& output,
    A6Drive** drives, int numHwDrives)
{
    AxisConfig& ac = m_axisConfig[i];
    AxisRuntime& rt = m_runtime[i];

    // Runaway guards (overspeed persistence + net-travel cap) -- see
    // beltGuardTripped(). Armed continuously from tension-up through ONLINE.
    if (beltGuardTripped(i, (drives && i < numHwDrives) ? drives[i] : nullptr, output))
        return rt.currentPos;

    // Unidirectional belt tensioner with the SAME staged standby as a
    // position axis (so a telemetry loss settles instead of holding torque
    // forever):
    //   live frame       -> tension tracks telemetry (min..max)
    //   phase 1 (<HOLD)  -> hold last tension (ride out a stutter)
    //   phase 2 (..to)   -> ease to torqueMin (neutral taut standby)
    //   phase 3 (>=to)   -> PARKING (ramp tension to 0 = slack)
    // From g the belt's stream is the channel stream, which goes quiet
    // whenever the game pauses or loses focus while the motion stream and
    // the posts carry on: there it stops at phase 2 (taut at min) and
    // phase 3 waits for the motion stream to have been stale for the hold,
    // so it slackens with the posts, never on its own.
    if (ac.beltFromG) noteBeltMotionStream(i, telemetryData);
    double tension = ac.torqueMinPct;
    double demand = 0.0;
    const bool haveData = beltDemand(i, telemetryData, demand);
    if (haveData)
    {
        rt.onlineStaleSec = 0.0;
        rt.onlineHadData  = true;
        tension = demand;
    }
    else if (rt.onlineHadData)
    {
        rt.onlineStaleSec += m_cycleTimeSec;
        // From g: the motion stream stale for the hold, and the g gone for
        // a ride-out of its own (a stutter never slackens it).
        const bool slacken = ac.beltFromG
            ? (rt.beltMotionSeen && rt.beltMotionStaleSec >= ac.onlineHoldTimeoutSec
               && rt.onlineStaleSec >= std::min(ONLINE_STALE_HOLD_SEC, ac.onlineHoldTimeoutSec))
            : (rt.onlineStaleSec >= ac.onlineHoldTimeoutSec);
        if (slacken)
        {
            rt.interpElapsed  = 0.0;
            rt.interpDuration = ac.parkTimeSec;
            state             = AxisMotionState::PARKING;   // ramps tension -> 0
            output.torques[i] = rt.lastTension;
            if (ac.beltFromG)
                RT_LOG_WARNING("MotionController: Axis %d (belt from g) motion stream stale "
                    ">%.0fs -- easing slack with the posts.", i + 1, ac.onlineHoldTimeoutSec);
            else
                RT_LOG_WARNING("MotionController: Axis %d (belt) telemetry stale "
                    ">%.0fs -- easing slack.", i + 1, ac.onlineHoldTimeoutSec);
            return rt.currentPos;
        }
        tension = (rt.onlineStaleSec > ONLINE_STALE_HOLD_SEC)
                ? ac.torqueMinPct      // phase 2: ease to neutral taut
                : rt.lastTension;      // phase 1: hold last
    }
    else
    {
        tension = ac.torqueMinPct;     // never had a frame -- min taut, waiting
    }

    // ---- Relaxer (force/thermal-domain, optional): sustained near-max
    // dwell is not a legitimate racing signal -- braking zones last
    // seconds. Ease to min until demand drops; also pre-empts the drive's
    // i2t overload fault (which would park the whole rig mid-session).
    if (ac.beltRelaxerSec > 0.0)
    {
        const double band = ac.torqueMaxPct * ac.beltRelaxerPct / 100.0;
        if (tension >= band) rt.beltDwellSec += m_cycleTimeSec;
        else if (tension < ac.torqueMaxPct * (ac.beltRelaxerPct - 10.0) / 100.0)
        { rt.beltDwellSec = 0.0; if (rt.beltRelaxed) { rt.beltRelaxed = false;
            RT_LOG_INFO("MotionController: Axis %d (belt) relaxer released.", i + 1); } }
        if (!rt.beltRelaxed && rt.beltDwellSec >= ac.beltRelaxerSec)
        { rt.beltRelaxed = true;
            RT_LOG_WARNING("MotionController: Axis %d (belt) tension >=%.0f%% of max for %.0fs "
                           "-- relaxing to min until demand drops.", i + 1, ac.beltRelaxerPct, ac.beltRelaxerSec); }
        if (rt.beltRelaxed) tension = ac.torqueMinPct;
    }

    // ---- Slew cap (command-domain): limits d(tension)/dt of the TELEMETRY
    // tension only. It shapes the motion software's tension cue, spreads a
    // frame that jumps and paces the stall ease and the resume take-up; the
    // haptics overlay is added later in process() with no slew, so this
    // never limits nullCAT's own belt effects (see Config.h). The e-stop path
    // bypasses this (it breaks out above and commands 0 immediately).
    {
        const double maxStep = ac.beltSlewPctPerSec * m_cycleTimeSec;
        tension = std::max(rt.lastTension - maxStep,
                           std::min(rt.lastTension + maxStep, tension));
    }

    // ---- Velocity fold (motion-domain, master-side): AFTER the slew so
    // the safety fold is never rate-limited downward. Recovery is gentle
    // by construction: lastTension stores the folded value, so as speed
    // drops the slew cap governs the ramp back up.
    tension = beltVelocityFold(i, tension);

    rt.lastTension    = tension;
    output.torques[i] = tension;
    return rt.currentPos;
}

double MotionController::stepPositionOnline(int i, AxisMotionState& state,
    const TelemetryData& telemetryData)
{
    AxisConfig& ac = m_axisConfig[i];
    AxisRuntime& rt = m_runtime[i];

    // Decode the telemetry target (shared by CSP and PP paths below).
    // NaN/Inf guard: a non-finite raw sample is treated as "no data" --
    // we never feed garbage into the command stream.
    // Hold if no data: the CUE's own position, never rt.currentPos on CSP.
    // rt.currentPos is the command sent, haptic overlay included; holding
    // it fed the overlay back in every cycle (a 0.1 mm vibration became
    // ~200 mm/s of travel: the dev bench, motion off, unpark, the posts
    // ran wild). PP takes no haptics, so its command is its cue.
    const double cueHold = ac.ppMode ? rt.currentPos : rt.onlineCond.pos;
    double targetMm = cueHold;
    bool haveData = axisFrameUsable(i, telemetryData);
    if (haveData)
    {
        rt.onlineStaleSec = 0.0;
        rt.onlineHadData = true;
        targetMm = decodeTargetMm(i, telemetryData.positions[i]);
    }
    else if (rt.onlineHadData)
    {
        // Telemetry dropped AFTER it was flowing -- staged standby, not an
        // instant park (a paused sim / menu / alt-tab / telemetry stutter must
        // not eject us from ONLINE). Only arms once a frame has been seen, so
        // a freshly-started loop with telemetry not yet connected just holds.
        //   phase 1 (< HOLD_SEC):        hold in place, ride out the hiccup
        //   phase 2 (HOLD_SEC..timeout): ease to center (level standby)
        //   phase 3 (>= onlineHoldTimeout): park
        // Any returning frame resumes ONLINE tracking seamlessly (the filter
        // slews from wherever it is, braking-clamped). targetMm defaults to
        // the cue's hold (phase 1) unless overridden below.
        rt.onlineStaleSec += m_cycleTimeSec;
        if (rt.onlineStaleSec >= ac.onlineHoldTimeoutSec)
        {
            rt.interpStart    = rt.currentPos;
            rt.interpTarget   = ac.parkPos;
            rt.interpElapsed  = 0.0;
            rt.interpDuration = ac.parkTimeSec;
            state             = AxisMotionState::PARKING;
            RT_LOG_WARNING("MotionController: Axis %d telemetry stale "
                ">%.0fs -- parking.", i + 1, ac.onlineHoldTimeoutSec);
            return rt.currentPos;
        }
        else if (rt.onlineStaleSec > ONLINE_STALE_HOLD_SEC)
        {
            targetMm = ac.centerPos;   // ease smoothly to neutral standby
        }
    }

    double outPos;
    if (ac.ppMode)
    {
        // PP mode UNCHANGED: the drive has its own profiler. Without this
        // cap a large telemetry jump (source active at session start with the
        // axis mid-stroke) makes the drive profiler target full stroke in
        // one go -- a violent full-range move. Cap the advance to
        // maxVelocityMmS * cycleTime; the drive smooths on top.
        double maxStep = ac.maxVelocityMmS * m_cycleTimeSec;
        double delta = targetMm - rt.currentPos;
        outPos = (std::abs(delta) > maxStep)
            ? rt.currentPos + std::copysign(maxStep, delta)
            : targetMm;
    }
    else
    {
        // CSP command conditioning: dispatch through the selected mode
        // (Bypass / Interpolate / Filter) into the shared guard chain. At
        // full velocity (maxVelocity) in ONLINE. Counts are formed only at
        // the PDO write (downstream).
        outPos = conditionCommand(rt.onlineCond, targetMm,
                                  telemetryData.nominalFrameSec, ac.maxVelocityMmS, ac);
        // Haptics on a position axis. A route gain on this axis is mm at
        // 100% effect amplitude; the layer has already derated every
        // contribution to what this axis can follow at the effect carrier
        // (min of hapticsMaxMm, the haptic share of maxVelocity / w and of
        // maxAcceleration / w^2 - the commissioning sweep rule), so the sum
        // / 100 is an offset the actuator can reproduce. Added AFTER the
        // cue conditioning (Interpolate/Filter never smear it), capped once
        // more at hapticsMaxMm, and the SUM passes sumGuard with the FULL
        // axis limits so a hard cue always wins and the drive never sees
        // more than its limits. ONLINE only (never BLENDING, homing or
        // parking), never on PP mode.
        // The guard engages from the command actually sent last cycle at the
        // cue's own velocity (no position or velocity seam), is cleared
        // whenever the axis is not ONLINE (process(), after the state
        // machine), and hands back to the cue only once it has landed on it.
        if (ac.hapticsMaxMm > 0.0 && state == AxisMotionState::ONLINE)
        {
            double hapMm = m_haptics.overlayFor(i) / 100.0;
            if (hapMm != 0.0 || rt.hapActive)
            {
                const double brakeEps = 4.0 / std::max(1.0, ac.countsPerMm);
                const double cueOut = outPos;
                if (!rt.hapActive)
                {
                    rt.sumGuard.seedState(rt.currentPos, (cueOut - rt.currentPos) / m_cycleTimeSec);
                    rt.hapActive = true;
                }
                hapMm = std::max(-ac.hapticsMaxMm, std::min(ac.hapticsMaxMm, hapMm));
                outPos = rt.sumGuard.stepBypass(cueOut + hapMm, m_cycleTimeSec, m_cycleTimeSec,
                                                ac.maxVelocityMmS, ac.maxAccelMmS2, brakeEps);
                // Overlay gone and the guard back on the cue: hand the command
                // back. Dropping it the moment the overlay read zero stepped
                // the command by the last overlay in one cycle.
                if (hapMm == 0.0 && std::fabs(outPos - cueOut) <= brakeEps
                    && std::fabs(rt.sumGuard.vel - rt.onlineCond.vel) <= ac.maxAccelMmS2 * m_cycleTimeSec)
                    rt.hapActive = false;
            }
        }
        else rt.hapActive = false;
        outPos = std::max(ac.minPos, std::min(ac.maxPos, outPos));
    }
    return outPos;
}

double MotionController::stepBeltParking(int i, AxisMotionState& state,
    MotionOutput& output)
{
    AxisRuntime& rt = m_runtime[i];

    // Belt: ease tension from its park-entry value (rt.lastTension) to 0
    // over parkTime, then PARKED (slack).
    rt.interpElapsed += m_cycleTimeSec;
    double t = std::min(1.0, rt.interpElapsed / rt.interpDuration);
    output.torques[i] = (1.0 - t) * rt.lastTension;
    if (t >= 1.0)
    {
        state = AxisMotionState::PARKED;
        rt.lastTension = 0.0;
        RT_LOG_INFO("MotionController: Axis %d (belt) PARKED (slack).", i + 1);
    }
    return rt.currentPos;
}

// ---- Device family (shifter/pedal) step functions ------------------------
// The force field is self-contained: it runs from the axis's OWN measured
// position, so there is no telemetry dependency, no stale-park, and no
// conditioning chain -- DeviceForceModel carries its own guard chain
// (thermal dwell -> slew -> velocity fold -> clamp). rt.currentPos is a
// pass-through for devices (positions are ignored by the CST write path);
// rt.lastTension carries the last commanded torque, belt-style, so the
// PARKING ramp starts from what was actually commanded.

double MotionController::deviceCurrentRev(int i, A6Drive* drive) const
{
    const AxisConfig& ac = m_axisConfig[i];
    if (!drive) return ac.device.neutralRev;   // sim: resting at neutral
    const AxisRuntime& rt = m_runtime[i];
    // dir maps motor raw onto the device frame -- the exact mirror of the
    // model's output mapping, so the loop sign is consistent for mirrored
    // builds. beltUnitsPerRev is the family-generic raw-units-per-motor-rev
    // factor (belt-named only for history).
    return rt.deviceHomeStopRev + ac.device.dir *
        (drive->getActualPositionRaw() - rt.deviceHomeRaw) / ac.beltUnitsPerRev;
}

double MotionController::stepDeviceBlending(int i, AxisMotionState& state,
    const TelemetryData& td, MotionOutput& output, A6Drive** drives, int numHwDrives)
{
    AxisRuntime& rt = m_runtime[i];
    A6Drive* d = (drives && i < numHwDrives) ? drives[i] : nullptr;
    const double posRev = deviceCurrentRev(i, d);

    // Seed the model at the engage position: no velocity kick from a stale
    // previous-position, fresh texture phase.
    if (!rt.deviceSeeded)
    {
        rt.deviceModel.reset(posRev);
        rt.deviceSeeded = true;
    }

    rt.blendElapsed += m_cycleTimeSec;
    const double bf = std::max(0.0, std::min(1.0, rt.blendElapsed / rt.blendDuration));

    // State effects compose with the engage ramp: the field (and anything
    // the wire adds to it) fades in together, never snaps.
    DeviceStateMods mods = rt.deviceState.step(posRev, m_ncxMap.extract(td),
                                               &m_ratioLearner.ratios());
    mods.forceScale    *= bf;
    mods.textureAmpPct *= bf;
    HapticTriggers trig;
    const double f = rt.deviceModel.step(posRev, mods, &trig);
    fireHaptics(trig);
    rt.lastTension    = f;
    output.torques[i] = f;

    if (bf >= 1.0)
    {
        state = AxisMotionState::ONLINE;
        RT_LOG_INFO("MotionController: Axis %d (device) engaged -- ONLINE.", i + 1);
    }
    return rt.currentPos;
}

// Model detected, layer synthesizes: velocity scale gets a 0.25 floor so a
// slow, deliberate gate entry still clicks softly instead of going silent.
// Routing is entirely the effect's own axis-gain table.
void MotionController::fireHaptics(const HapticTriggers& t)
{
    if (t.detentEnter)
        m_haptics.fire(haptics::EventType::DetentClick, 0.25 + 0.75 * t.detentVel);
}

// Push one set of haptics tuning into the layer: every registry effect
// into its pool/level slot by kind, the engine description, master gain.
static void applyHapticsTo(haptics::Layer& L, const std::array<haptics::EffectParams, haptics::EFFECT_COUNT>& fx,
                           const haptics::EngineParams& engine, const haptics::SlipParams& slipLat,
                           const haptics::SlipParams& slipLon, const haptics::RoadParams& road,
                           const haptics::KerbParams& kerb, const haptics::PulseParams& abs, const haptics::PulseParams& tc,
                           const haptics::SurfaceParams& surface, const haptics::WheelsParams& wheels, const haptics::ImpactParams& impacts,
                           const haptics::DrivelineParams& driveline, double masterGain)
{
    for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
    {
        const haptics::EffectInfo& info = haptics::effectInfo(i);
        const haptics::EffectParams& p = fx[static_cast<size_t>(i)];
        switch (info.kind)
        {
            case haptics::Kind::Transient:  L.configure(info.event, p);  break;
            case haptics::Kind::Continuous:
            case haptics::Kind::Engine:
            case haptics::Kind::Slip:
            case haptics::Kind::Road:
            case haptics::Kind::Kerb:
            case haptics::Kind::Abs:
            case haptics::Kind::Tc:
            case haptics::Kind::Surface:
            case haptics::Kind::Wheels:
            case haptics::Kind::Impacts:
            case haptics::Kind::Driveline:  L.configureFx(info.fx, p);   break;
        }
    }
    L.configureEngine(engine);
    L.configureSlip(haptics::FxType::Skid,   slipLat);
    L.configureSlip(haptics::FxType::Lockup, slipLon);
    L.configureRoad(road);
    L.configureKerb(kerb);
    L.configurePulse(haptics::FxType::AbsPulse, abs);
    L.configurePulse(haptics::FxType::TcPulse, tc);
    L.configureSurface(surface);
    L.configureWheels(wheels);
    L.configureImpacts(impacts);
    L.configureDriveline(driveline);
    L.setMasterGain(masterGain);
}

void MotionController::applyHaptics(const AppConfig& c)
{
    applyHapticsTo(m_haptics, c.hapticsFx, c.hapticsEngine, c.hapticsSlipLat, c.hapticsSlipLon, c.hapticsRoad, c.hapticsKerb,
                   c.hapticsAbs, c.hapticsTc, c.hapticsSurface, c.hapticsWheels, c.hapticsImpacts, c.hapticsDriveline, c.hapticsMasterGain);
}

// What each axis is as a haptic destination, and for position axes the
// limits the layer derates against: the haptic share of the axis velocity
// and acceleration limits plus its hapticsMaxMm cap. Torque-mode axes and
// devices are torque sinks; CSP axes are position sinks; PP-mode axes take
// no haptics (the drive's own profiler would mangle them).
void MotionController::applyHapticSinks()
{
    for (int i = 0; i < MAX_DRIVES; ++i)
    {
        const AxisConfig& ac = m_axisConfig[i];
        const bool torquey = ac.torqueMode || ac.caps.isDevice();
        if (i >= m_numDrives || torquey || ac.ppMode)
        {
            m_haptics.setSinkKind(i, haptics::SinkKind::Torque);
            m_haptics.setPositionLimits(i, 0.0, 0.0, 0.0);
            continue;
        }
        m_haptics.setSinkKind(i, haptics::SinkKind::Position);
        m_haptics.setPositionLimits(i, m_hapPositionBudget * ac.maxVelocityMmS,
                                       m_hapPositionBudget * ac.maxAccelMmS2,
                                       ac.ppMode ? 0.0 : ac.hapticsMaxMm);
    }
}

// Live-apply: a rig save stages the new haptics config here (web thread,
// under the lock); the RT thread applies it at the top of the next cycle.
// Same pattern as the device-params staging - feel tuning never needs a
// re-initialize. The stage is a plain member (trivially copyable), so
// neither side ever allocates.
void MotionController::stageHaptics(const AppConfig& c)
{
    std::lock_guard<std::mutex> lk(m_hapStageLock);
    m_hapStage.fx         = c.hapticsFx;
    m_hapStage.engine     = c.hapticsEngine;
    m_hapStage.slipLat    = c.hapticsSlipLat;
    m_hapStage.slipLon    = c.hapticsSlipLon;
    m_hapStage.road       = c.hapticsRoad;
    m_hapStage.kerb       = c.hapticsKerb;
    m_hapStage.abs        = c.hapticsAbs;
    m_hapStage.tc         = c.hapticsTc;
    m_hapStage.surface    = c.hapticsSurface;
    m_hapStage.wheels     = c.hapticsWheels;
    m_hapStage.impacts    = c.hapticsImpacts;
    m_hapStage.driveline  = c.hapticsDriveline;
    m_hapStage.masterGain = c.hapticsMasterGain;
    m_hapStage.positionBudget = c.hapticsPositionBudget;
    m_hapStage.axisDelayMs    = c.hapticsAxisDelayMs;
    m_hapStage.beltFullG      = c.hapticsBeltFullG;
    m_hapStagePending.store(true, std::memory_order_release);
}

// Staged live config (rig save, no re-init) lands at the top of a cycle.
void MotionController::applyStagedHaptics()
{
    if (const int t = m_toneRequest.exchange(0, std::memory_order_acq_rel)) m_haptics.startShakerTone(t - 1, 1.0);
    applyHapticMutes();
    // A tile's Test, on this clock's thread (fire()/preview never race the
    // layer). Routing is the effect's own table; the overlay gate keeps
    // non-live axes silent regardless. Transients fire once, continuous and
    // engine effects run a short full-level preview.
    if (const int id = m_hapTestReq.exchange(-1, std::memory_order_acq_rel); id >= 0 && id < haptics::EFFECT_COUNT)
    {
        const haptics::EffectInfo& info = haptics::effectInfo(id);
        if (info.kind == haptics::Kind::Transient)
            m_haptics.fire(info.event, 1.0);
        else
            haptics::startPreview(m_hapLaws, info.fx);
    }
    if (!m_hapStagePending.load(std::memory_order_acquire)) return;
    HapticsStage s;   // trivially copyable, no allocation on this thread
    {
        // Never block this thread on the web thread: if a save is mid-copy
        // the stage waits a cycle. The pending flag clears inside the lock,
        // so a stage landing between the copy and the clear is kept (it
        // was lost until the next save).
        std::unique_lock<std::mutex> lk(m_hapStageLock, std::try_to_lock);
        if (!lk.owns_lock()) return;
        s = m_hapStage;
        m_hapStagePending.store(false, std::memory_order_release);
    }
    applyHapticsTo(m_haptics, s.fx, s.engine, s.slipLat, s.slipLon, s.road, s.kerb, s.abs, s.tc, s.surface, s.wheels, s.impacts, s.driveline, s.masterGain);
    m_hapPositionBudget = std::max(0.0, std::min(1.0, s.positionBudget));
    m_beltTypeFullG = std::isfinite(s.beltFullG) ? s.beltFullG : 0.0;
    m_haptics.setAxisDelayCycles(axisDelayCycles(s.axisDelayMs, m_cycleTimeSec));
    applyHapticSinks();
    RT_LOG_INFO("MotionController: haptics settings applied live.");
}

// The tiles' mutes onto the layer, when they changed (the clock's thread).
void MotionController::applyHapticMutes()
{
    const uint64_t mask = m_hapMuteMask.load(std::memory_order_acquire);
    if (mask == m_hapMuteApplied) return;
    for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
    {
        const haptics::EffectInfo& info = haptics::effectInfo(i);
        const bool m = (mask >> i) & 1ull;
        if (info.kind == haptics::Kind::Transient) m_haptics.setEventMuted(info.event, m);
        else                                       m_haptics.setFxMuted(info.fx, m);
    }
    m_hapMuteApplied = mask;
}

// One haptics cycle, on whichever clock has the path (the RT loop, or the
// idle clock with the loop stopped): the staged settings, the laws (or the
// e-stop clear), the layer, the peaks, the shakers. One place, so the two
// clocks cannot drift apart.
void MotionController::stepHapticsCycle(const TelemetryData& td, bool estop)
{
    applyStagedHaptics();
    // Gear-ratio learner: one observation per cycle from the NULLCATX
    // channels (it filters for meaningful samples itself, in seconds).
    // Rig-level, and on whichever clock is stepping: a shaker-only install
    // never runs the RT loop, and the laws read the ratios it learns.
    if (m_ncxMap.boundCount() > 0)
        m_ratioLearner.step(m_ncxMap.extract(td), m_cycleTimeSec);
    if (estop) m_haptics.clearAll();
    else       driveContinuousHaptics(td);
    m_haptics.step(m_cycleTimeSec);
    trackHapticPeaks();
    pushShakers();
}

// One control cycle's shaker samples into the sound card, when one is
// attached (ShakerOutput::push is lock-free and skips when closed).
void MotionController::pushShakers()
{
    if (m_shakers) m_shakers->push(m_haptics.shakerSamples(), haptics::MAX_SHAKER_OUT);
}

// The haptics idle clock: with the control loop stopped (or no drives
// at all), a timer thread steps just the haptics path at the control
// rate so shakers keep playing. Never touches an axis. Guarded against
// overlapping the RT cycle by a flag both paths take.
void MotionController::hapticsIdleTick(const TelemetryData& td)
{
    if (m_hapticsBusy.exchange(true, std::memory_order_acq_rel)) return;   // the RT loop has it
    // An e-stop silences the shakers with the loop stopped too (the GPIO
    // button or the page's e-stop reach the flag whether or not it runs),
    // until the re-initialize that clears its trip.
    stepHapticsCycle(td, estopHolds());
    {
        std::unique_lock<std::shared_mutex> lock(m_statusLock, std::try_to_lock);
        if (lock.owns_lock()) { publishHapticsStatusLocked(); publishLearnerLocked(); }   // this tick holds the flag
    }
    m_hapticsBusy.store(false, std::memory_order_release);
}

// The effect laws live in HapticLaws.h (one place for every
// channel -> level rule, testable without a controller); this is the
// per-cycle call plus the channel-health snapshot the status surface shows.
void MotionController::driveContinuousHaptics(const TelemetryData& td)
{
    const NcxValues v = m_ncxMap.extract(td);
    if (m_hapLatRelearn.exchange(false, std::memory_order_acq_rel)) haptics::relearnLatBase(m_hapLaws);
    if (m_hapFreshTyres.exchange(false, std::memory_order_acq_rel)) m_haptics.wheelsFreshTyres();
    haptics::driveLaws(m_haptics, m_hapLaws, v, m_cycleTimeSec, &m_ratioLearner.ratios());
    for (int i = 0; i < NcxValues::TokenCount; ++i)
    {
        m_ncxHaveSnapshot[i] = v.fresh && v.have[i];
        m_ncxValSnapshot[i]  = m_ncxHaveSnapshot[i] ? v.val[i] : 0.0;
    }
}

double MotionController::stepDeviceOnline(int i, AxisMotionState& /*state*/,
    const TelemetryData& td, MotionOutput& output, A6Drive** drives, int numHwDrives)
{
    AxisRuntime& rt = m_runtime[i];
    A6Drive* d = (drives && i < numHwDrives) ? drives[i] : nullptr;
    const double posRev = deviceCurrentRev(i, d);
    const DeviceStateMods mods = rt.deviceState.step(posRev, m_ncxMap.extract(td),
                                                     &m_ratioLearner.ratios());
    HapticTriggers trig;
    const double f = rt.deviceModel.step(posRev, mods, &trig);
    fireHaptics(trig);
    rt.lastTension    = f;
    output.torques[i] = f;
    return rt.currentPos;
}

double MotionController::stepDeviceParking(int i, AxisMotionState& state,
    MotionOutput& output)
{
    AxisRuntime& rt = m_runtime[i];

    // Ease the force from its release-entry value to 0 over parkTime, then
    // PARKED (limp). Signed-safe: devices command negative torque too.
    rt.interpElapsed += m_cycleTimeSec;
    const double t = std::min(1.0, rt.interpElapsed / rt.interpDuration);
    output.torques[i] = (1.0 - t) * rt.lastTension;
    if (t >= 1.0)
    {
        state = AxisMotionState::PARKED;
        rt.lastTension  = 0.0;
        rt.deviceSeeded = false;
        RT_LOG_INFO("MotionController: Axis %d (device) released -- PARKED (limp).", i + 1);
    }
    return rt.currentPos;
}

double MotionController::stepPositionParking(int i, AxisMotionState& state)
{
    AxisConfig& ac = m_axisConfig[i];
    AxisRuntime& rt = m_runtime[i];

    rt.interpElapsed += m_cycleTimeSec;
    double t = std::min(1.0, rt.interpElapsed / rt.interpDuration);
    double outPos = interpolate(rt.interpStart, rt.interpTarget, t);
    if (t >= 1.0)
    {
        state = AxisMotionState::PARKED;
        outPos = ac.parkPos;
        RT_LOG_INFO("MotionController: Axis %d PARKED.", i + 1);
    }
    return outPos;
}

// ============================================================
// process() -- main cyclic function
// ============================================================
void MotionController::process(const TelemetryData& telemetryData, MotionOutput& output,
    A6Drive** drives, int numHwDrives, ecx_contextt* /*ctx*/)
{
    // Drain UI commands first -- all m_axisState[] writes from UI-initiated
    // commands (StartHoming, StartPark) happen here on the RT thread.
    drainCommands(drives, numHwDrives);

    // Device live-apply: staged feel/geometry params land on limp devices.
    applyPendingDeviceParams();
    // Belt from g live-apply: a saved g shape lands this cycle.
    applyPendingBeltG();

    // Telemetry-quiet tracking for the commissioning entry rail: a test must
    // not start while a game/SimHub stream is live (the two would fight).
    if (telemetryFrameUsable(telemetryData))
        m_secsSinceTelemetry = 0.0;
    else if (m_secsSinceTelemetry < 1e9)
        m_secsSinceTelemetry += m_cycleTimeSec;

    // Commissioning test mode: start/stop requests, engine step, completion.
    serviceCommissioning(drives, numHwDrives);

    output.numDrives = m_numDrives;
    // The input held, or tripped since the last initialize: either way the
    // drives stay disabled (ControlLoop) and nothing moves.
    const bool estopNow = estopHolds();
    output.emergencyStop = estopNow;

    // Edge-detect estop transitions here so every m_axisState[] write happens on
    // the RT thread -- setEmergencyStop() (any thread) only touches the atomic.
    if (estopNow && !m_prevEstopState)
    {
        // Rising edge: transition all live axes to ESTOPPING.
        RT_LOG_WARNING("MotionController: EMERGENCY STOP - ramping to halt.");
        m_estopElapsed = 0.0;
        for (int i = 0; i < m_numDrives; ++i)
            if (m_axisState[i] != AxisMotionState::PARKED)
                m_axisState[i] = AxisMotionState::ESTOPPING;
    }
    else if (!estopNow && m_prevEstopState)
    {
        // Falling edge: the trip was cleared by re-initializing the drives.
        // Nothing moves from here by itself (it used to re-home and then
        // auto-unpark: a 10 ms blip on the GPIO input moved the rig with no
        // one at the controls). The axes stay parked where the stop left
        // them; the initialize that cleared the trip reset them to unhomed
        // (configure), so the loop start's auto-homing or a Home press is
        // what moves them next.
        RT_LOG_INFO("MotionController: E-stop trip cleared -- the axes stay where they are until homed.");
        m_estopElapsed = 0.0;
    }
    m_prevEstopState = estopNow;

    if (estopNow)
        m_estopElapsed += m_cycleTimeSec;

    // Haptics: apply any staged live config first (rig save, no re-init);
    // e-stop kills everything instantly (no click tail or effect drone
    // rides an emergency ramp); otherwise drive the continuous effects
    // from this cycle's channels and advance the layer once, so
    // overlayFor() below serves this cycle's values.
    // (the idle clock may still hold the layer for one tick at the hand-
    // over when the loop starts; the overlays are then last cycle's)
    if (!m_hapticsBusy.exchange(true, std::memory_order_acq_rel))
    {
        stepHapticsCycle(telemetryData, estopNow);
        m_hapticsBusy.store(false, std::memory_order_release);
    }

    for (int i = 0; i < m_numDrives; ++i)
    {
        AxisConfig& ac = m_axisConfig[i];
        AxisRuntime& rt = m_runtime[i];
        AxisMotionState& state = m_axisState[i];

        double outPos = rt.currentPos;

        // Device axes: refresh the live lever position every cycle,
        // whatever the state - the web teach capture reads it while the
        // device rests limp (PARKED), where no step function runs. The
        // min/max sweep since homing feeds Capture travel.
        if (ac.caps.isDevice())
        {
            rt.devPosRev = deviceCurrentRev(i,
                (drives && i < numHwDrives) ? drives[i] : nullptr);
            if (rt.homed)
            {
                if (rt.devPosRev < rt.devPosMin) rt.devPosMin = rt.devPosRev;
                if (rt.devPosRev > rt.devPosMax) rt.devPosMax = rt.devPosRev;
            }
        }

        switch (state)
        {
        case AxisMotionState::HOMING:
        {
            A6Drive* drive = (drives && i < numHwDrives) ? drives[i] : nullptr;
            processHomingAxis(i, drive, output);
            outPos = rt.currentPos;
            break;
        }

        case AxisMotionState::PARKED:
            if (!rt.homed)
            {
                // Drive not yet homed -- homeOffset invalid, so hold at the
                // physical resting position (not parkPos, which would jump → Er87).
                //
                // CRITICAL: latch that position ONCE. Re-reading the live encoder
                // every cycle and writing it straight back as the target makes
                // the drive chase its own 17-bit encoder noise floor -- a
                // ±1-count dither on 0x607A that sings audibly through the motor
                // while every position plot still reads "stationary". Latched
                // once, the commanded count is bit-identical every cycle.
                if (!rt.parkHoldLatched)
                {
                    A6Drive* pd = (drives && i < numHwDrives) ? drives[i] : nullptr;
                    rt.currentPos = pd ? pd->getActualPositionRaw() : rt.currentPos;
                    rt.parkHoldLatched = true;
                    RT_LOG_INFO("MotionController: Axis %d unhomed park-hold latched at %.4f.",
                                i + 1, rt.currentPos);
                }
                outPos = rt.currentPos;
            }
            else
            {
                // Hold at rt.currentPos as set by the prior state transition --
                // NEVER command ac.parkPos unconditionally here. Right after
                // homing, rt.currentPos is the post-homing position (0.0), so
                // jumping to parkPos would be a |parkPos - 0| step in one cycle:
                // harmless for parkMode=="endstop" (parkPos ~1.5mm, small enough
                // for the drive to track) but tens of mm for "center" -- an
                // instant Er87.1 on every cold homing. The unpark trajectory
                // starts from rt.currentPos and ramps smoothly to centerPos,
                // which is the intended behavior in both modes.
                outPos = rt.currentPos;
            }
            break;

        case AxisMotionState::UNPARKING:
        {
            rt.interpElapsed += m_cycleTimeSec;
            double t = std::min(1.0, rt.interpElapsed / rt.interpDuration);
            outPos = interpolate(rt.interpStart, rt.interpTarget, t);
            if (t >= 1.0)
            {
                rt.blendStartPos = outPos;
                rt.blendElapsed = 0.0;
                rt.blendDuration = m_blendTimeSec;
                rt.blendExtensions = 0;
                rt.filteredPos = outPos;
                rt.initialized = true;

                // Initialize trajectory state at blend start
                rt.traj.pos = outPos;
                rt.traj.vel = 0.0;
                rt.traj.accel = 0.0;
                rt.traj.targetPos = outPos;
                rt.traj.active = true;

                // Seed the ONLINE follower at the blend-start position (vel 0).
                // BLENDING runs the same filter with a ramping velocity cap, so
                // there is no separate planner and no BLENDING->ONLINE handoff.
                rt.onlineCond.seedState(outPos, 0.0);
                rt.onlineHadData = false;
                rt.onlineStaleSec = 0.0;
                rt.beltMotionSeen     = false;
                rt.beltMotionStaleSec = 0.0;
                // Belt guards: re-seed derivative + net-travel reference at tension-up.
                rt.beltHaveRaw      = false;
                rt.beltOverspeedSec = 0.0;
                rt.beltDwellSec     = 0.0;
                rt.beltRelaxed      = false;
                rt.beltLastTripWhy  = 0;
                for (auto& c : rt.beltGCh) c.live = c.meanLive = false;
                rt.resetBeltTeleport();

                state = AxisMotionState::BLENDING;
                RT_LOG_INFO("MotionController: Axis %d BLENDING into live telemetry.", i + 1);
            }
            break;
        }

        case AxisMotionState::BLENDING:
        {
            if (output.emergencyStop)
            {
                state = AxisMotionState::ESTOPPING;
                m_estopElapsed = 0.0;
                outPos = rt.currentPos;
                break;
            }
            // Family fork (see the Stage D seam note in the header): device
            // families first -- they are torque-mode too, so the belt
            // ternary alone would misroute them. Non-device routing is
            // UNCHANGED, including the degenerate torque-mode-without-
            // belt-type configs.
            outPos = ac.caps.isDevice()
                ? stepDeviceBlending(i, state, telemetryData, output, drives, numHwDrives)
                : (ac.torqueMode
                    ? stepBeltBlending(i, state, telemetryData, output, drives, numHwDrives)
                    : stepPositionBlending(i, state, telemetryData));
            break;
        }

        case AxisMotionState::ONLINE:
        {
            if (output.emergencyStop)
            {
                state = AxisMotionState::ESTOPPING;
                m_estopElapsed = 0.0;
                outPos = rt.currentPos;
                break;
            }

            outPos = ac.caps.isDevice()
                ? stepDeviceOnline(i, state, telemetryData, output, drives, numHwDrives)
                : (ac.torqueMode
                    ? stepBeltOnline(i, state, telemetryData, output, drives, numHwDrives)
                    : stepPositionOnline(i, state, telemetryData));
            break;
        }

        case AxisMotionState::PARKING:
        {
            outPos = ac.caps.isDevice()
                ? stepDeviceParking(i, state, output)
                : (ac.torqueMode
                    ? stepBeltParking(i, state, output)
                    : stepPositionParking(i, state));
            break;
        }

        case AxisMotionState::ESTOPPING:
        {
            // Decelerate in place -- do NOT move toward parkPos.
            // Hardware quick-stop has already cut drive output; commanding
            // a position ramp toward park causes unexpected motion on release.
            outPos = rt.currentPos;

            double t = std::min(1.0, m_estopElapsed / ESTOP_RAMP_SEC);
            if (t >= 1.0)
            {
                state = AxisMotionState::PARKED;
                RT_LOG_INFO("MotionController: Axis %d E-STOP complete -- held at pos=%.3f",
                    i + 1, rt.currentPos);
            }
            break;
        }

        case AxisMotionState::TESTING:
        {
            // Commissioning excitation. The engine's offset rides on centre
            // and goes through stepBypass DIRECTLY -- deterministic excitation
            // (Filter/Interpolate smoothing would attenuate the very tones
            // being measured) with the full guard chain (velocity clamp,
            // accel clamp, braking guard) still live. Metrics + the
            // following-error abort rail run on the guarded command vs the
            // measured position, so what is scored is what the drive was
            // actually asked to do.
            // Test offsets are PLATFORM-frame, like telemetry: the same
            // polarity composition (invertDir XOR extended-stop homing) maps
            // them into the engineering frame, so mirrored lever pairs heave
            // together instead of differentially. Metrics are fed in the
            // platform frame too (both cmd and act through the same sign, so
            // amplitude/phase/ferr are unchanged on uninverted axes).
            const double pol = ac.telemetryInvert ? -1.0 : 1.0;
            const double want = ac.centerPos + pol * m_testOffsets[i];
            const double brakeEps = 4.0 / ac.countsPerMm;
            outPos = rt.onlineCond.stepBypass(want, m_cycleTimeSec, m_cycleTimeSec,
                                              ac.maxVelocityMmS, ac.maxAccelMmS2,
                                              brakeEps);
            A6Drive* drive = (drives && i < numHwDrives) ? drives[i] : nullptr;
            const double actual = drive ? drive->getActualPosition() : outPos;
            m_commissioning.recordSample(i, pol * (outPos - ac.centerPos),
                                         pol * (actual - ac.centerPos),
                                         drive ? drive->getTorquePercent() : 0.0);
            break;
        }
        }

        // Re-arm the unhomed park-hold latch once the axis leaves PARKED, so a
        // future unhomed park captures a fresh resting position.
        if (state != AxisMotionState::PARKED)
            rt.parkHoldLatched = false;

        // The haptics guard lives for one ONLINE stretch. Kept across a park
        // and re-home it resumed from a stale position and velocity at the
        // next ONLINE and stepped the posts (the dev bench's Er87).
        if (state != AxisMotionState::ONLINE)
            rt.hapActive = false;

        // Last line: while live telemetry or haptics shape a position axis's
        // command, it can never move further in one cycle than the axis's
        // maxVelocity allows. The guards upstream already hold this; if one
        // ever fails, the step is held to the limit here and logged once,
        // instead of reaching the drive as a jump.
        if (!ac.torqueMode && !ac.caps.isDevice()
            && (state == AxisMotionState::ONLINE || state == AxisMotionState::BLENDING))
        {
            const double maxStep = ac.maxVelocityMmS * m_cycleTimeSec * 1.05 + 1.0 / std::max(1.0, ac.countsPerMm);
            const double step = outPos - rt.currentPos;
            if (std::fabs(step) > maxStep)
            {
                if (!rt.stepGuardWarned)
                {
                    RT_LOG_WARNING("MotionController: Axis %d command step %.3f mm in one cycle held to %.3f mm "
                        "(maxVelocity) -- a guard upstream let it through.", i + 1, step, maxStep);
                    rt.stepGuardWarned = true;
                }
                outPos = rt.currentPos + std::copysign(maxStep, step);
            }
        }
        else rt.stepGuardWarned = false;

        // Haptics overlay: rides ON TOP of whatever the state machine
        // commanded, torque-mode axes only, live states only (a limp or
        // parking axis never buzzes), and the SUM is clamped inside the
        // axis's own limits - the layer can never exceed the guard rails.
        if ((state == AxisMotionState::ONLINE || state == AxisMotionState::BLENDING)
            && (ac.torqueMode || ac.caps.isDevice()))
        {
            double ov = m_haptics.overlayFor(i);
            // A belt folded for overspeed (beltVelocityFold has taken its
            // tension down, or away) takes its haptics down with it: the
            // overlay used to ride over the fold and could keep pulling on
            // a belt the guard had let go of.
            if (!ac.caps.isDevice()) ov = beltVelocityFold(i, ov);
            if (ov != 0.0)
            {
                double t = output.torques[i] + ov;
                if (ac.caps.isDevice())
                {
                    const double cap = ac.device.maxForcePct;
                    t = std::max(-cap, std::min(cap, t));
                }
                else
                {
                    t = std::max(0.0, std::min(ac.torqueMaxPct, t));
                }
                output.torques[i] = t;
            }
        }

        rt.currentPos = outPos;
        rt.prevPos = outPos;
        output.positions[i] = outPos;
    }

    // Post-loop: auto-unpark after homing
    if (!m_homingUnparkDone && allAxesHomed())
    {
        bool anyHomed = false;
        for (int i = 0; i < m_numDrives; ++i)
            if (m_runtime[i].homed) { anyHomed = true; break; }

        if (anyHomed)
        {
            RT_LOG_INFO("MotionController: All axes homed -- auto-unparking.");
            m_homingUnparkDone = true;
            startUnpark(drives, numHwDrives);
        }
    }

    // Publish UI-visible state snapshot (safe - we are on the RT thread,
    // all m_axisState[] / m_runtime[] writes for this cycle are complete).
    publishStatus();
}

// ============================================================
// Homing axis processing
// ============================================================
void MotionController::processHomingAxis(int i, A6Drive* drive, MotionOutput& output)
{
    AxisRuntime& rt = m_runtime[i];
    AxisConfig& ac = m_axisConfig[i];

    // Device families home by torque stall-search, not the CSP endstop
    // search -- their PDO layout has no position command to move with.
    if (ac.caps.homingKind == HomingKind::Torque)
    {
        processDeviceHomingAxis(i, drive, output);
        return;
    }

    bool isSimDrive = (drive == nullptr);

    if (isSimDrive)
    {
        RT_LOG_INFO("MotionController: Axis %d homing in SIM mode -- instant complete.", i + 1);
        rt.currentPos = ac.parkPos;
        rt.homed = true;
        m_axisState[i] = AxisMotionState::PARKED;
        output.positions[i] = rt.currentPos;
        return;
    }

    // (A "gravity" parkMode used to short-circuit here: it declared the axis
    // homed at wherever it happened to be resting, with no search, on the
    // assumption that gravity had already parked a vertical actuator on its
    // bottom stop. It was a leftover from the PP-mode era, never exposed in the
    // UI or documented, and unused. Removed: any unrecognised parkMode now falls
    // through to the real torque-based endstop search below, which is the safe
    // direction -- an axis with no valid reference searches for one instead of
    // inventing one.)

    if (m_homing[i].getState() == HomingSequence::State::Idle ||
        m_homing[i].getState() == HomingSequence::State::Complete ||
        m_homing[i].getState() == HomingSequence::State::Error)
    {
        clearAxisLimits(i, drive);  // widen stale post-fault limits before the search
        m_homing[i].reset();
        m_homing[i].start(drive);
    }

    HomingSequence::State hs = m_homing[i].step(drive);

    rt.currentPos = drive->getActualPositionRaw();

    if (hs == HomingSequence::State::Complete)
    {
        double homeOffset = m_homing[i].getHomeOffset();
        double frameSign  = m_homing[i].getFrameSign();
        drive->setHomeOffset(homeOffset, frameSign);

        RT_LOG_INFO("MotionController: Axis %d home frame applied: offset=%.3f sign=%+.0f",
            i + 1, homeOffset, frameSign);

        // Update drive limits to the homed coordinate window. The engineering
        // window is always [minPos, maxPos] away from the stop; in raw terms
        // its endpoints are offset + sign*minPos and offset + sign*maxPos,
        // which ORDER-SWAP when the frame is reversed -- min/max them, or a
        // positive-direction home would place the clamp window entirely on the
        // far side of the stop (the exact overtravel it exists to prevent).
        const double rawA = homeOffset + frameSign * ac.minPos;
        const double rawB = homeOffset + frameSign * ac.maxPos;
        drive->setLimits(std::min(rawA, rawB), std::max(rawA, rawB));
        RT_LOG_INFO("MotionController: Axis %d drive limits updated to [%.3f, %.3f] (absolute)",
            i + 1, std::min(rawA, rawB), std::max(rawA, rawB));

        rt.homed = true;
        double actualPos = drive->getActualPosition();
        rt.currentPos = actualPos;
        rt.filteredPos = actualPos;
        rt.initialized = true;

        // Initialize trajectory at current position with zero velocity
        rt.traj.pos = actualPos;
        rt.traj.vel = 0.0;
        rt.traj.accel = 0.0;
        rt.traj.targetPos = actualPos;
        rt.traj.active = true;

        m_axisState[i] = AxisMotionState::PARKED;

        RT_LOG_INFO("MotionController: Axis %d homing COMPLETE -- PARKED at offset pos=%.3f (raw=%.3f)",
            i + 1, actualPos, drive->getActualPositionRaw());
    }
    else if (hs == HomingSequence::State::Error)
    {
        RT_LOG_ERROR("MotionController: Axis %d homing ERROR -- axis PARKED at current position.", i + 1);
        rt.currentPos = drive->getActualPositionRaw();
        m_axisState[i] = AxisMotionState::PARKED;
    }
    else if (hs == HomingSequence::State::FatalError)
    {
        // FatalError is terminal -- does NOT auto-restart.
        // Requires explicit startHoming() call (user action) to retry.
        RT_LOG_ERROR("MotionController: Axis %d homing FATAL ERROR -- "
            "manual re-home required. Call startHoming() to retry.", i + 1);
        rt.currentPos = drive->getActualPositionRaw();
        m_axisState[i] = AxisMotionState::PARKED;
    }

    output.positions[i] = rt.currentPos;
}

void MotionController::processDeviceHomingAxis(int i, A6Drive* drive, MotionOutput& output)
{
    AxisRuntime& rt = m_runtime[i];
    AxisConfig&  ac = m_axisConfig[i];

    if (!drive)
    {
        RT_LOG_INFO("MotionController: Axis %d (device) homing in SIM mode -- instant complete.", i + 1);
        rt.deviceHomeRaw     = 0.0;
        rt.deviceHomeStopRev = (ac.device.homeDir < 0.0) ? ac.device.stopMinRev
                                                         : ac.device.stopMaxRev;
        rt.homed       = true;
        rt.lastTension = 0.0;
        m_axisState[i] = AxisMotionState::PARKED;
        return;
    }

    if (m_torqueHoming[i].getState() == TorqueHomingSequence::State::Idle)
        m_torqueHoming[i].start(drive);

    // The sequence writes the drive itself (it owns the drive while the
    // axis is HOMING); the mirror into output.torques[] feeds the cards.
    output.torques[i]   = m_torqueHoming[i].step(drive);
    rt.currentPos       = drive->getActualPositionRaw();
    output.positions[i] = rt.currentPos;

    if (m_torqueHoming[i].isComplete())
    {
        rt.deviceHomeRaw     = m_torqueHoming[i].getHomeRaw();
        rt.deviceHomeStopRev = m_torqueHoming[i].homeStopRev();
        rt.devPosRev   = rt.deviceHomeStopRev;   // lever is AT the stop; the
                             // per-cycle refresh ran before this frame latched
        rt.devPosMin   = rt.devPosRev;           // fresh sweep for Capture travel
        rt.devPosMax   = rt.devPosRev;
        rt.homed       = true;
        rt.lastTension = 0.0;
        m_axisState[i] = AxisMotionState::PARKED;   // PARKED = limp for devices
        RT_LOG_INFO("MotionController: Axis %d (device) homing COMPLETE -- stop %.3frev "
            "latched at raw=%.3f. PARKED (limp); engage to load the force field.",
            i + 1, rt.deviceHomeStopRev, rt.deviceHomeRaw);
    }
    else if (m_torqueHoming[i].isFatalError())
    {
        // Terminal, like the CSP FatalError: axis PARKED unhomed; explicit
        // startHoming() (user action) to retry.
        RT_LOG_ERROR("MotionController: Axis %d (device) homing FATAL ERROR -- "
            "manual re-home required.", i + 1);
        rt.lastTension = 0.0;
        m_axisState[i] = AxisMotionState::PARKED;
    }
}

double MotionController::applySpikeFilter(int i, double input)
{
    double delta = std::abs(input - m_runtime[i].prevPos);
    if (delta > m_axisConfig[i].spikeMaxMm)
    {
        RT_LOG_WARNING("MotionController: Spike on axis %d (%.2fmm), rejected.", i + 1, delta);
        return m_runtime[i].prevPos;
    }
    return input;
}

double MotionController::interpolate(double from, double to, double t)
{
    double s = t * t * (3.0 - 2.0 * t);
    return from + s * (to - from);
}

void MotionController::clearAxisLimits(int axis, A6Drive* drive)
{
    if (!drive) return;
    const AxisConfig& ac = m_axisConfig[axis];
    // Widen limits to 2× stroke below and 3× above so any post-fault actual
    // position is reachable during the new homing search.
    drive->setLimits(-ac.strokeMm * 2.0, ac.strokeMm * 3.0);
}
