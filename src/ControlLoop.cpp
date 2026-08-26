// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// ControlLoop.cpp
//
// Hard real-time cyclic loop: paces the EtherCAT PDO exchange,
// gates state transitions on WKC validity, runs the motion
// controller, and steps the DS402 enable/fault handling per
// cycle. Also hosts the deinit seat-only pass (press the
// vertical axes onto the bottom stop before de-energizing).
// Platform RT primitives (scheduling, timing, affinity) live in
// PlatformRT.h; slave recovery runs on EtherCATMaster's recovery
// thread and is only signalled from here.
// ============================================================

#include "ControlLoop.h"
#include "DriveFaultMonitor.h"
#include "A6FaultCodes.h"
#include "Logging.h"
#include "DcPhaseLock.h"

#ifdef SOEM_AVAILABLE
extern "C" {
#include "soem/ec_options.h"
#include "soem/ec_type.h"
#include "nicdrv.h"
#include "soem/ec_base.h"
#include "soem/ec_main.h"
#include "soem/ec_dc.h"
#include "soem/ec_coe.h"
#include "soem/ec_config.h"
}
#endif

#include "WkcMonitor.h"
#include "PlatformRT.h"
#include <shared_mutex>
#include <cstring>
#include <algorithm>
#include <cmath>

// ---- Deinit seat tuning ----
// SEAT_DWELL_SEC: how long to hold steady ON the stop after the press+relief, before the
// de-energize / OP->INIT switchover. Lets a lightly-damped axis finish settling so it isn't
// mid-ring when the switchover happens (suspected ErC1.2 trigger on the low-drag rear axis).
// The companion seat knobs are the relief thresholds in MotionController.cpp
// (SEAT_RELIEF_FLOOR_PCT / _RISE_PCT / _CAP_MM).
static constexpr double SEAT_DWELL_SEC = 1.0;

// Deinit-seat bus-health guard. Pre-flight: 20
// paced WKC-checked cycles; >=18 good = seat normally; partial-majority = seat
// only slaves confirmed in OP; otherwise skip the seat entirely (the drives'
// own 100ms watchdog has already de-energized them on a dead bus). Mid-phase:
// 0.5s of consecutive bad WKC aborts all remaining seat phases -- a seat
// pass on a dead bus otherwise runs blind on frozen PDO data.
static constexpr int    SEAT_PROBE_CYCLES   = 20;
static constexpr int    SEAT_PROBE_MIN_GOOD = 18;
static constexpr double SEAT_ABORT_SEC      = 0.5;

// Slave recovery lives on EtherCATMaster's recovery thread
// (doRecoveryScan); the RT loop only signals it via
// m_master->signalRecoveryNeeded().

// ============================================================
// ControlLoopWorker
// ============================================================

void ControlLoopWorker::setComponents(
    EtherCATMaster* master,
    TelemetryInput* telemetry,
    MotionController* motion)
{
    m_master = master;
    m_telemetry = telemetry;
    m_motion = motion;
}

void ControlLoopWorker::setConfig(const AppConfig& config)
{
    m_config = config;
    double hz = std::max(100, std::min(4000, config.controlLoopHz));   // Pi: up to 4 kHz
    m_cycleTimeUs = 1e6 / hz;
    // Publish shared status in the ~50Hz band regardless of loop rate
    // (2kHz Pi -> every 40th cycle; 500Hz -> every 10th; <=50Hz -> every cycle).
    m_statusPublishDivider = std::max(1, static_cast<int>(hz / 50.0));
}

void ControlLoopWorker::clearFaultLockout(int driveIndex)
{
    if (driveIndex < 0)
    {
        // Clear all drives
        m_clearLockoutMask.store(~uint32_t(0), std::memory_order_release);
    }
    else if (driveIndex < MAX_DRIVES)
    {
        m_clearLockoutMask.fetch_or(uint32_t(1) << driveIndex, std::memory_order_release);
    }
}

LoopStats ControlLoopWorker::getStats() const
{
    std::shared_lock<std::shared_mutex> lock(m_statusLock);
    return m_stats;
}

DriveStatus ControlLoopWorker::getDriveStatus(int index) const
{
    if (index < 0 || index >= MAX_DRIVES) return {};
    std::shared_lock<std::shared_mutex> lock(m_statusLock);
    return m_driveStatus[index];
}

// ============================================================
// Main Control Loop
// ============================================================
void ControlLoopWorker::run()
{
    m_velPrimed = false;   // re-prime velocity derivation on each (re)start
    // Elevate RT priority, register with MMCSS (Windows) / SCHED_FIFO (Linux),
    // and pin to core 3. Details and fallback warnings logged inside threadSetup().
    PlatformRT::RtHandle rtHandle = PlatformRT::threadSetup(3);

    // Arm the RT-safe log path. From this point, LOG_* calls on this thread
    // push to a lock-free SPSC queue. The drain thread routes them to file + UI.
    Logger::instance().setRTThread();

    PlatformRT::timerBegin();

    RT_LOG_INFO("ControlLoop: Started at %d Hz (cycle: %.1f us)",
        m_config.controlLoopHz, m_cycleTimeUs);

    if (m_master)
    {
        m_master->stopPump();
        m_master->setRtLoopActive(true);  // this thread is now the sole SOEM owner
        RT_LOG_INFO("ControlLoop: Background pump stopped -- control loop is now master.");
    }

    bool m_telemetryMotionActive = false;
    bool prevEstop = false;           // e-stop transition detection
    bool pumpCrashReported = false;   // one-shot pump crash handling
    bool hwEstopActive = false;       // hardware e-stop via QuickStopActive
    // Track whether QuickStopActive was caused by the software e-stop.
    // If true, don't auto-clear when drives exit QuickStop -- the user must
    // explicitly release. Only auto-clear if the e-stop was hardware-initiated
    // (physical button/relay that held QuickStop pin on the drive).
    bool hwEstopWasSoftware = false;

    // ---- Timing setup ----
    PlatformRT::Timestamp now        = PlatformRT::now();
    PlatformRT::Timestamp nextCycleTime = now;
    const int64_t cycleCountsTarget  = PlatformRT::periodCounts(m_cycleTimeUs);

    // ---- DC phase-lock compensator (default OFF) ----
    // When disabled, periodCounts below stays cycleCountsTarget -> the loop
    // free-runs exactly as before (byte-identical). When enabled, a gentle
    // clamped PI trims the per-cycle period to hold the DC sampling phase
    // constant, paying down the ppm walk that slides the frame onto the SYNC0
    // boundary. Seat-mode passes above intentionally do NOT use it (transient
    // teardown). Phase is read after sendReceive(), so it feeds the NEXT cycle's
    // trim -- one cycle of latency, negligible for this bandwidth.
    DcPhaseLock dcLock;
    {
        const int hz = (m_config.controlLoopHz > 0) ? m_config.controlLoopHz : 500;
        DcPhaseLock::Params p;
        p.cycleNs       = 1000000000LL / hz;
        p.nominalCounts = cycleCountsTarget;
        p.dtSec         = 1.0 / (double)hz;
        p.kp            = m_config.dcPhaseLockKp;
        p.ki            = m_config.dcPhaseLockKi;
        p.maxTrimNs     = m_config.dcPhaseLockMaxTrimNs;
        p.warmupCycles  = hz;                       // ~1 s settle before locking
        dcLock.configure(p);
        dcLock.setEnabled(m_config.dcPhaseLockEnabled);
    }
    int64_t dcLockPhase = DcPhaseLock::kNoSample;   // previous cycle's DC phase

    // ---- Statistics ----
    uint64_t cycleCount = 0;
    double   maxJitterUs = 0.0;
    uint64_t lastStatsEmit = 0;
    const uint64_t statsInterval = static_cast<uint64_t>(m_config.controlLoopHz);
    uint64_t badFrameCount = 0;

    // EtherCAT send/receive round-trip time tracking (hardware path only).
    double   rttMinUs = 1e9;
    double   rttMaxUs = 0.0;
    double   rttSumUs = 0.0;
    uint64_t rttCount = 0;

    // ---- WKC monitor ----
    WkcMonitor wkcMonitor;

    // ---- Fault tracking per drive (delegated to DriveFaultMonitor) ----
    DriveFaultMonitor faultMonitor;

    bool justEnabled[MAX_DRIVES] = {};
    memset(justEnabled, 0, sizeof(justEnabled));

    // ---- Following error tracking ----
    double maxFollowingError[MAX_DRIVES] = {};
    memset(maxFollowingError, 0, sizeof(maxFollowingError));

    // ---- Command-dither fingerprint (diag-gated) ----
    // Per-axis, once per second: reversals/s of the commanded count, max single-
    // cycle step, and max second-difference (velStep). A high reversals/s with a
    // tiny maxStep, or a high maxVelStep with low reversals, indicates command
    // dither / a low-send-rate staircase in the 0x607A stream -- a command-
    // cleanliness check per mode. Passive; diagEnabled gates it entirely.
    int32_t cmdPrevCnt[MAX_DRIVES]    = {};
    int     cmdReversals[MAX_DRIVES]  = {};
    int32_t cmdMaxStep[MAX_DRIVES]    = {};
    int32_t cmdPrevStep[MAX_DRIVES]   = {};
    int32_t cmdMaxVelStep[MAX_DRIVES] = {};
    int8_t  cmdPrevSign[MAX_DRIVES]   = {};
    bool    cmdSeen[MAX_DRIVES]       = {};
    bool    cmdHaveStep[MAX_DRIVES]   = {};

    // ---- DC phase walk (sync-lock fingerprint, diag-gated) ----
    int64_t dcPhaseMin = INT64_MAX, dcPhaseMax = INT64_MIN;

    // ---- Drive pointer cache ----
    A6Drive* driveCache[MAX_DRIVES] = {};
    int      numDrives = 0;
    ecx_contextt* ctx = nullptr;

    if (m_master)
    {
        // Cap numDrives to the actual slave count, not the config count
        int configDrives = m_master->getDriveCount();
        int slavesOnBus = m_master->getSlaveCount();
        numDrives = std::min(configDrives, slavesOnBus);
        if (configDrives > slavesOnBus)
        {
            RT_LOG_WARNING("ControlLoop: Config has %d drives but only %d slaves on bus. "
                "Using %d drives.", configDrives, slavesOnBus, numDrives);
        }
        ctx = m_master->getContext();
        for (int i = 0; i < numDrives && i < MAX_DRIVES; ++i)
            driveCache[i] = m_master->getDrive(i);
    }

    faultMonitor.configure(numDrives);
    wkcMonitor.configure(m_config.controlLoopHz);   // time-based recovery trigger (~15ms)

    // ---- Deinit SEAT-ONLY pass ----
    // The loop was stopped; deinit re-engaged this RT thread purely to press the vertical
    // axes onto the bottom stop (so the de-init that follows doesn't free-fall them). No
    // main loop, no auto-home. On completion it hands the drives back to the pump (held ON
    // the stop, still in OP) -- the de-energize itself is done by EtherCATMaster::shutdown()
    // via the proven path, so DC SYNC0 is never starved. SOEM is already claimed above
    // (stopPump + setRtLoopActive). Runs here on the RT thread so RT logging + pacing are
    // valid (RT_LOG is single-producer; must be this thread).
    if (m_seatOnlyMode.load(std::memory_order_acquire))
    {
        // ---- Bus-health pre-flight ----
        // Classify SEAT_PROBE_CYCLES paced frames before pressing anything:
        // a seat sequence on a dead/degraded bus runs blind on frozen PDO
        // data and then de-energizes axes that never reached the stop.
        bool seatSkipped = false;
        bool seatable[MAX_DRIVES];
        const bool* seatMask = nullptr;
        for (int i = 0; i < MAX_DRIVES; ++i) seatable[i] = true;
        if (m_master && !m_master->isSimulation())
        {
            int good = 0, part = 0, dead = 0;
            const int expWkc = m_master->getExpectedWKC();
            for (int p = 0; p < SEAT_PROBE_CYCLES; ++p)
            {
                PlatformRT::waitUntil(nextCycleTime);
                PlatformRT::advancePeriod(nextCycleTime, cycleCountsTarget);
                int wkc = m_master->sendReceive();
                m_master->processCyclicMailbox(1);
                if (wkc == expWkc) ++good; else if (wkc > 0) ++part; else ++dead;
            }
            if (good >= SEAT_PROBE_MIN_GOOD)
            {
                // Healthy bus -- today's path, unchanged.
            }
            else if (part > SEAT_PROBE_CYCLES / 2)
            {
                // Partial bus (frames return, some slaves missing): seat only
                // axes whose slave reads OPERATIONAL. 0x08 = EC_STATE_OPERATIONAL
                // (SOEM headers are not visible in this TU by design).
                uint16_t st[MAX_DRIVES] = {};
                int n = m_master->probeSlaveALStates(st, MAX_DRIVES);
                for (int i = 0; i < numDrives && i < MAX_DRIVES; ++i)
                {
                    int slaveIdx = driveCache[i] ? driveCache[i]->getSlaveIndex() : (i + 1);
                    seatable[i] = (slaveIdx >= 1 && slaveIdx <= n) && (st[slaveIdx - 1] == 0x08);
                }
                seatMask = seatable;
                RT_LOG_WARNING("ControlLoop: Deinit seat on a PARTIAL bus (good=%d partial=%d"
                    " dead=%d of %d) -- seating reachable axes only.",
                    good, part, dead, SEAT_PROBE_CYCLES);
            }
            else
            {
                seatSkipped = true;
                RT_LOG_ERROR("ControlLoop: Deinit seat SKIPPED -- bus unhealthy (good=%d"
                    " partial=%d dead=%d of %d, expectedWKC=%d). Axes were NOT seated;"
                    " check drive power / bus cabling. De-init continues.",
                    good, part, dead, SEAT_PROBE_CYCLES, expWkc);
            }
        }
        // Mid-phase abort tracking: SEAT_ABORT_SEC of consecutive bad WKC ends
        // all remaining seat phases (the drives watchdog at 100ms, so by then
        // the outcome is decided). Single blips reset the counter.
        bool busLost = false;
        int  busLostConsec = 0;
        const int seatAbortConsecBad =
            std::max(1, static_cast<int>(m_config.controlLoopHz * SEAT_ABORT_SEC));

        // Seat = run the REAL homing search (in seat mode) on the eligible vertical axes:
        // the identical enable→settle→search-down-to-25%-torque you see on Home, but at the
        // hardstop it HOLDS on the stop instead of backing off, and establishes NO home
        // reference. We drive HomingSequence::step() directly (stepSeatHoming) -- never
        // process()/processHomingAxis -- so homing's completion side effects (homeOffset,
        // homed, PARKED, auto-restart) can never run. Direction + torque detection are
        // homing's verbatim, so there is no separate press logic to get wrong.
        if (m_motion && !seatSkipped) m_motion->startSeatHoming(driveCache, numDrives, seatMask);

        // Give-up backstop for BOTH the press (reach the stop) and the relief (ease off)
        // phases below. Normal completion is fast -- the press seats in ~1.5s and a
        // resting axis relieves in <1s -- so this only caps a pathological hang. A light
        // axis that never fully unloads onto its stop (e.g. the low-weight rear actuator,
        // or an operator leaning on the seat) can never satisfy the relief's rest test and
        // would otherwise ride this cap; 10s de-energizes it promptly rather than stalling
        // the whole de-init for 30s. Time-based, so it is correct at any loop rate.
        const int maxSeatCycles = static_cast<int>(m_config.controlLoopHz * 10.0);
        int seatCycles = 0;
        while (m_motion && !seatSkipped && !busLost
               && !m_motion->allSeatAxesAtStop() && seatCycles < maxSeatCycles)
        {
            PlatformRT::waitUntil(nextCycleTime);
            PlatformRT::advancePeriod(nextCycleTime, cycleCountsTarget);
            ++seatCycles;

            m_motion->stepSeatHoming(driveCache, numDrives);   // homing search/hold, raw + FE-capped
            if (m_master)
            {
                int wkc = m_master->sendReceive();
                m_master->processCyclicMailbox(1);   // keep the cyclic mailbox serviced (≤1 op/cycle)
                if (m_master->isSimulation() || wkc == m_master->getExpectedWKC())
                    busLostConsec = 0;
                else if (++busLostConsec >= seatAbortConsecBad)
                {
                    busLost = true;
                    RT_LOG_ERROR("ControlLoop: Deinit seat ABORTED -- bus lost mid-seat"
                        " (%d consecutive bad WKC). De-init continues.", busLostConsec);
                }
            }
            updateDriveStatuses();
        }

        if (seatSkipped || busLost)
            ;   // outcome already logged by the guard
        else if (seatCycles >= maxSeatCycles)
            RT_LOG_WARNING("ControlLoop: Deinit seat backstop hit (%d cycles) -- holding on stop.", seatCycles);
        else
            RT_LOG_INFO("ControlLoop: Deinit seat pressed to stop (%d cycles).", seatCycles);

        // Phase 2 -- ease the press off to RESTING before de-energizing. The drives are
        // pressing ~25% into the stop; disabling a loaded axis at that torque trips ErC1.2
        // on the heavy actuators (drive 3, light, tolerates it; 1/2 don't). Ease each axis
        // back a touch until its torque bottoms out at the just-resting point (the stop then
        // bears the weight, torque -> ~0) so de-energize happens at the same low torque the
        // proven park path already disables at -- and the axis stays ON the stop (no drop).
        // This is plain orchestration (a few setTargetPositionRaw + torque watch); it does
        // NOT touch HomingSequence.
        if (m_motion && !seatSkipped && !busLost)
            m_motion->startSeatRelief(driveCache, numDrives);
        int reliefCycles = 0;
        while (m_motion && !seatSkipped && !busLost
               && !m_motion->allSeatAxesRelieved() && reliefCycles < maxSeatCycles)
        {
            PlatformRT::waitUntil(nextCycleTime);
            PlatformRT::advancePeriod(nextCycleTime, cycleCountsTarget);
            ++reliefCycles;

            m_motion->stepSeatRelief(driveCache, numDrives);   // ease off one step + torque watch
            if (m_master)
            {
                int wkc = m_master->sendReceive();
                m_master->processCyclicMailbox(1);
                if (m_master->isSimulation() || wkc == m_master->getExpectedWKC())
                    busLostConsec = 0;
                else if (++busLostConsec >= seatAbortConsecBad)
                {
                    busLost = true;
                    RT_LOG_ERROR("ControlLoop: Deinit seat relief ABORTED -- bus lost mid-seat"
                        " (%d consecutive bad WKC). De-init continues.", busLostConsec);
                }
            }
            updateDriveStatuses();
        }
        if (!seatSkipped && !busLost)
            RT_LOG_INFO("ControlLoop: Seat press eased to rest (%d cycles) -- de-init will de-energize.", reliefCycles);

        // Phase 3 -- DWELL. Hold steady on the stop for SEAT_DWELL_SEC before de-energizing,
        // so a lightly-damped axis (the low-drag rear actuator) finishes ringing/settling
        // before the OP->INIT switchover -- the suspected ErC1.2 trigger on that axis. No
        // new commands: the relieved targets just hold; we keep cycling sendReceive so DC
        // stays alive. TUNABLE: SEAT_DWELL_SEC (top of this file). Other seat knobs are the
        // relief thresholds in MotionController.cpp (SEAT_RELIEF_FLOOR/RISE/CAP).
        const int dwellCycles = static_cast<int>(m_config.controlLoopHz * SEAT_DWELL_SEC);
        for (int d = 0; !seatSkipped && !busLost && d < dwellCycles; ++d)
        {
            PlatformRT::waitUntil(nextCycleTime);
            PlatformRT::advancePeriod(nextCycleTime, cycleCountsTarget);
            if (m_master)
            {
                int wkc = m_master->sendReceive();
                m_master->processCyclicMailbox(1);
                if (m_master->isSimulation() || wkc == m_master->getExpectedWKC())
                    busLostConsec = 0;
                else if (++busLostConsec >= seatAbortConsecBad)
                {
                    busLost = true;
                    RT_LOG_ERROR("ControlLoop: Seat dwell cut short -- bus lost (%d consecutive"
                        " bad WKC). De-init continues.", busLostConsec);
                }
            }
            updateDriveStatuses();
        }
        if (!seatSkipped && !busLost)
            RT_LOG_INFO("ControlLoop: Seat dwell %.1fs complete (%d cycles) -- de-init will de-energize.",
                        SEAT_DWELL_SEC, dwellCycles);

        // Hand back to the background pump EXACTLY like the normal park teardown below:
        // the drives stay enabled, held ON the stop, in OP, and the pump keeps frames
        // flowing so the drive's DC SYNC0 watchdog never starves. The de-init that follows
        // (EtherCATMaster::shutdown) then disables + walks OP->INIT via the proven path.
        // Do NOT de-energize here and return into the no-frame gap before shutdown() --
        // that starved SYNC0 and tripped Er74.1 (no sync) / ErC1.2 (state-switch error).
        if (m_master)
            m_master->setRtLoopActive(false);
        if (m_master && !m_master->isSimulation())
        {
            m_master->startPump();
            RT_LOG_INFO("ControlLoop: Pump restarted after seat -- drives held on the stop until de-init.");
        }
        PlatformRT::threadTeardown(rtHandle);
        PlatformRT::timerEnd();
        return;   // caller (deinit) now runs EtherCATMaster::shutdown(): disable + OP->INIT
    }

    if (m_onLoopStarted) m_onLoopStarted();

    if (m_motion && m_motion->needsRehome())
    {
        RT_LOG_INFO("ControlLoop: Auto-homing on loop start (incremental encoders).");
        m_motion->startHoming();
        faultMonitor.reset();
    }

    if (m_motion) m_motion->resetGuardStats();   // fresh per-axis guard diagnostics this session
    memset(m_peakFollowingError, 0, sizeof(m_peakFollowingError));
    m_statsResetRequested.store(false, std::memory_order_release);

    // ---- Main loop ----
    while (!m_stopRequested.load())
    {
        PlatformRT::waitUntil(nextCycleTime);

        // Capture actual wake time and measure jitter BEFORE advancing the
        // deadline. elapsedMicros(now, nextCycleTime) = how late we woke up
        // relative to the target for THIS cycle. Advancing first was measuring
        // against the NEXT cycle target, giving a constant -2000µs (one period).
        now = PlatformRT::now();
        double jitterUs = PlatformRT::elapsedMicros(now, nextCycleTime);
        maxJitterUs = std::max(maxJitterUs, std::abs(jitterUs));
        ++cycleCount;

        // DC phase-lock: corrected period if enabled, else exactly the nominal.
        int64_t periodCounts = cycleCountsTarget;
        if (dcLock.enabled()) periodCounts = dcLock.update(dcLockPhase);
        PlatformRT::advancePeriod(nextCycleTime, periodCounts);

        // Overrun resync. advancePeriod() marches the deadline in fixed steps
        // and waitUntil() returns immediately while behind, so after a long
        // host stall (Npcap/DPC latency; 31ms observed in the field) the loop
        // would fire the missed cycles BACK-TO-BACK. Two reasons that burst is
        // worse than skipping: the drive's SYNC0 latch samples the LAST target
        // it received, so N burst cycles of commanded motion collapse into one
        // multi-mm latched step at full tracking speed (field logs show the
        // matching following-error spikes); and many frames per SYNC0 interval
        // is the exact per-frame sync-error mechanism the DC-aligned pump
        // cadence exists to prevent (ErC1.1 class). If the deadline has fallen
        // more than 2 full cycles behind, snap it to now and accept the missed
        // cycles as lost -- the axis holds briefly (exactly the bad-frame
        // policy) instead of lunging. Ordinary jitter under 2 cycles still
        // catches up normally, preserving the long-term cadence.
        {
            const double behindUs = PlatformRT::elapsedMicros(now, nextCycleTime);
            const double cycleUs  = 1e6 / std::max(1, m_config.controlLoopHz);
            if (behindUs > 2.0 * cycleUs)
            {
                const int lost = static_cast<int>(behindUs / cycleUs);
                nextCycleTime = now;
                PlatformRT::advancePeriod(nextCycleTime, periodCounts);
                RT_LOG_WARNING("ControlLoop: %.1fms stall -- resynced cadence, "
                               "%d cycle(s) skipped (axes held; no catch-up burst).",
                               behindUs / 1000.0, lost);
            }
        }

        // Soft stats reset (matched set): clear guard stats + peak following-error
        // together, in-loop, without dropping drives. Consumed once per request.
        if (m_statsResetRequested.exchange(false, std::memory_order_acq_rel))
        {
            if (m_motion) m_motion->resetGuardStats();
            memset(m_peakFollowingError, 0, sizeof(m_peakFollowingError));
        }

        // Send/receive process data FIRST
        int lastWkc = -1;
        if (m_master && m_master->isOperational() && !m_master->isSimulation())
        {
            PlatformRT::Timestamp rttT0 = PlatformRT::now();
            lastWkc = m_master->sendReceive();
            double rttUs = PlatformRT::elapsedMicros(PlatformRT::now(), rttT0);
            if (rttUs > 0.0)
            {
                rttMinUs = std::min(rttMinUs, rttUs);
                rttMaxUs = std::max(rttMaxUs, rttUs);
                rttSumUs += rttUs;
                ++rttCount;
            }

            // DC phase walk (sync-lock fingerprint), captured this frame.
            int64_t dcPh = m_master->getDcPhaseNs();
            if (dcPh < dcPhaseMin) dcPhaseMin = dcPh;
            if (dcPh > dcPhaseMax) dcPhaseMax = dcPh;
            dcLockPhase = dcPh;   // feeds next cycle's phase-lock trim

            // Drain pending cyclic-mailbox ops (emergency frames, queued
            // SDOs). limit=1 bounds the worst-case per-cycle latency. No-op
            // until EtherCATMaster::initCyclicMailboxHandler() has
            // registered slaves.
            m_master->processCyclicMailbox(1);
        }

        // Detect pump thread crash immediately (one-shot).
        if (!pumpCrashReported && m_master && !m_master->isSimulation() && m_master->hasPumpCrashed())
        {
            pumpCrashReported = true;
            RT_LOG_ERROR("ControlLoop: EtherCAT pump thread crashed -- triggering emergency stop.");
            if (m_onError)
                m_onError("EtherCAT transport failed: pump thread crashed. Restart required.");
            if (m_motion) m_motion->setEmergencyStop(true);
        }

        // ---- 1. Receive telemetry ----
        TelemetryData telemetryData;
        if (m_telemetry)
        {
            // Bounded drain: each queued datagram costs one recvfrom+parse on
            // this thread, so a flooding sender must not own the cycle. 32 per
            // cycle (16k pkt/s at 500 Hz) clears any real burst and works off a
            // post-stall socket backlog within a few cycles.
            for (int drained = 0; drained < 32 && m_telemetry->receive(); ++drained) {}

            if (m_telemetry->hasData())
            {
                telemetryData = m_telemetry->getLatestData();
                // Nominal frame interval (1/new_hz) for the Interpolate conditioning
                // mode -- always-on estimate, independent of the diag flag.
                telemetryData.nominalFrameSec = m_telemetry->getNominalFrameSec();

                // Telemetry-loss detection. getLatestData() keeps returning the
                // last packet with valid=true after the stream stops, so without
                // this the downstream staleness/standby logic never fires (the
                // axis would hold the frozen last target forever, then lurch on
                // reconnect). Mark stale once no fresh Motion frame has arrived
                // within TELEMETRY_STALE_MS; MotionController's staged standby then
                // owns the hold -> ease-to-center -> park timing from here. The
                // window is short enough to detect a real loss promptly but well
                // above any normal inter-packet gap (>=~10 Hz telemetry).
                constexpr int TELEMETRY_STALE_MS = 300;
                if (!m_telemetry->hasFreshData(TELEMETRY_STALE_MS))
                    telemetryData.valid = false;

                // hasData() is set solely for Motion packets. Motion gating
                // comes from nullCAT's own readiness (auto-enable below)
                // plus the web/GPIO Stop, never from any sender-side
                // lifecycle signal.
            }

            if (!m_telemetryMotionActive && m_motion && m_motion->allAxesReady()
                && m_motion->allAxesHomed())
            {
                m_telemetryMotionActive = true;
                RT_LOG_INFO("ControlLoop: All axes reached center -- motion enabled automatically.");
            }
        }

        // ---- 2. Gate PDO data on WKC validity ----
        // If sendReceive() returned the wrong WKC, one or more EtherCAT
        // frames were lost or corrupted.  Suppress updateStatus() for this
        // cycle so the state machine cannot transition on garbage data and
        // the homing sequence cannot compute a target from a garbage actualPos.
        // Last known-good values are retained until the next clean frame.
        if (m_master && !m_master->isSimulation() && m_master->isOperational())
        {
            bool frameValid = (lastWkc == m_master->getExpectedWKC());
            for (int i = 0; i < numDrives; ++i)
                if (driveCache[i]) driveCache[i]->setFrameValid(frameValid);
            if (!frameValid)
            {
                ++badFrameCount;
                if (badFrameCount % 100 == 1)
                    RT_LOG_WARNING("ControlLoop: Bad frame (wkc=%d expected=%d)"
                        " -- holding previous PDO values this cycle (total bad: %llu)",
                        lastWkc, m_master->getExpectedWKC(),
                        (unsigned long long)badFrameCount);
            }
            else
            {
                badFrameCount = 0;
            }
        }

        // ---- 3. Run motion controller ----
        MotionOutput motionOut;
        if (m_motion)
        {
            m_motion->process(telemetryData, motionOut, driveCache, numDrives, ctx);
        }

        // Reset fault lockouts on e-stop release.
        {
            bool curEstop = motionOut.emergencyStop;
            if (prevEstop && !curEstop)
            {
                faultMonitor.reset();
                for (int i = 0; i < numDrives; ++i)
                    justEnabled[i] = false;
                RT_LOG_INFO("ControlLoop: E-stop released -- fault lockouts and retry counters reset.");
            }
            prevEstop = curEstop;
        }

        // ---- 4. Drive outputs ----
        if (m_master && m_master->isOperational())
        {
            if (m_master->isSimulation())
            {
                for (int i = 0; i < numDrives; ++i)
                {
                    A6Drive* drive = driveCache[i];
                    if (!drive) continue;
                    double target = (i < motionOut.numDrives) ? motionOut.positions[i] : 0.0;
                    drive->setSimPosition(target);
                    drive->setSimTarget(target);
                }
            }
            else
            {
                for (int i = 0; i < numDrives; ++i)
                {
                    A6Drive* drive = driveCache[i];
                    if (!drive) continue;

                    drive->updateStatus();
                    DriveState state = drive->getState();

                    if (motionOut.emergencyStop)
                    {
                        drive->disableOperation();
                        justEnabled[i] = false;
                        continue;
                    }

                    // Check if the UI has requested a lockout clear for this drive.
                    {
                        uint32_t mask = m_clearLockoutMask.load(std::memory_order_acquire);
                        if (mask & (uint32_t(1) << i))
                        {
                            faultMonitor.clearLockout(i);
                            m_clearLockoutMask.fetch_and(~(uint32_t(1) << i), std::memory_order_release);
                            RT_LOG_INFO("ControlLoop: Drive %d fault lockout cleared by user.", drive->getSlaveIndex());
                        }
                    }

                    // ---- Fault state machine (delegated to DriveFaultMonitor) ----
                    {
                        auto fr = faultMonitor.step(i, drive, state, cycleCount);

                        if (fr.firstFaultSeen)
                        {
                            // Decoded in-line: 603F is a coarse CiA402 class shared by
                            // several panel Er codes, so the candidates are named here
                            // rather than sending the operator to the manual. The
                            // decode is a static literal -- RT-safe.
                            RT_LOG_ERROR("ControlLoop: Drive %d FAULT. SW=0x%04x code=0x%04x (603F: %s) (retries so far: %d/%d)",
                                drive->getSlaveIndex(), drive->getStatusword(),
                                drive->getFaultCode(),
                                a6BusFaultCandidates(drive->getFaultCode()),
                                faultMonitor.getRetryCount(i),
                                DriveFaultMonitor::MAX_FAULT_RETRIES);
                            // Precise decode: flag the recovery thread to
                            // SDO-read 0x203F (exact Er panel code) off-RT.
                            // Single atomic fetch_or here -- no mailbox
                            // traffic from the RT path.
                            if (m_master)
                                m_master->requestPanelCodeRead(i);
                            if (m_motion)
                            {
                                m_motion->setNeedsRehome(true);
                                m_motion->startPark();
                                RT_LOG_WARNING("ControlLoop: Drive fault -- parking all axes, rehome required.");
                            }
                        }

                        if (fr.lockoutJustOccurred)
                        {
                            std::string msg = strf("Drive %d locked out after %d fault reset attempts.",
                                drive->getSlaveIndex(), faultMonitor.getRetryCount(i));
                            RT_LOG_ERROR("ControlLoop: Drive %d locked out after %d fault reset attempts. "
                                "Power cycle drive or use Clear Lockout.",
                                drive->getSlaveIndex(), faultMonitor.getRetryCount(i));
                            if (m_config.requireUserFaultReset && m_onFaultLockout)
                                m_onFaultLockout(i, msg);
                        }

                        if (fr.seedPosition)
                        {
                            double actualPos = drive->getActualPositionRaw();
                            drive->setTargetPositionRaw(actualPos);
                            RT_LOG_INFO("ControlLoop: Drive %d fault cleared. "
                                "Target re-seeded to actual pos=%.3f", drive->getSlaveIndex(), actualPos);
                        }

                        if (fr.allClearRehome && m_motion && m_motion->needsRehome())
                        {
                            RT_LOG_INFO("ControlLoop: All faults cleared -- starting rehome.");
                            m_motion->startHoming();
                        }

                        if (fr.disable)
                        {
                            drive->disableOperation();
                            continue;
                        }

                        if (fr.seedPosition)
                            continue;   // seed must be the only command this cycle

                        if (fr.skipEnableSM)
                            continue;
                    }

                    // ---- DS402 Enable State Machine ----
                    bool axisIsHoming = (m_motion &&
                        m_motion->getAxisState(i) == AxisMotionState::HOMING);

                    if (!axisIsHoming)
                    {
                        bool enabled = drive->stepEnableStateMachine();

                        if (enabled)
                        {
                            if (!justEnabled[i])
                            {
                                double actual = drive->getActualPositionRaw();
                                drive->setTargetPositionRaw(actual);

                                justEnabled[i] = true;

                                RT_LOG_INFO("ControlLoop: Drive %d initial target seeded to actual (%.3f)",
                                    drive->getSlaveIndex(), actual);
                            }

                            double pos;
                            if (i < motionOut.numDrives)
                            {
                                pos = motionOut.positions[i];
                            }
                            else
                            {
                                pos = drive->getActualPosition();
                            }

                            if (drive->isTorqueMode())
                                drive->setTargetTorque((i < motionOut.numDrives) ? motionOut.torques[i] : 0.0);
                            else
                                drive->setTargetPosition(pos);

                            // Position-axis diagnostics only: on a torque drive
                            // `pos` is a frozen placeholder while the driver
                            // drags the shaft, so ferr and dither numbers there
                            // are noise dressed as data.
                            if (!drive->isTorqueMode())
                            {
                                double actualPos = drive->getActualPosition();
                                double followErr = std::abs(actualPos - pos);
                                if (followErr > maxFollowingError[i])
                                    maxFollowingError[i] = followErr;        // per-second diag window
                                if (followErr > m_peakFollowingError[i])
                                    m_peakFollowingError[i] = followErr;     // latched for the card (soft-reset)

                                // Command-dither fingerprint: track the commanded count
                                // (0x607A) reversals, max step, and velocity discontinuity.
                                int32_t cnt = (int32_t)std::llround(pos * drive->getCountsPerMm());
                                if (cmdSeen[i])
                                {
                                    int32_t d  = cnt - cmdPrevCnt[i];
                                    int32_t ad = d < 0 ? -d : d;
                                    if (ad > cmdMaxStep[i]) cmdMaxStep[i] = ad;
                                    int8_t sgn = (int8_t)((d > 0) - (d < 0));
                                    if (sgn != 0)
                                    {
                                        if (cmdPrevSign[i] != 0 && sgn != cmdPrevSign[i])
                                            cmdReversals[i]++;
                                        cmdPrevSign[i] = sgn;
                                    }
                                    if (cmdHaveStep[i])
                                    {
                                        int32_t dd  = d - cmdPrevStep[i];
                                        int32_t add = dd < 0 ? -dd : dd;
                                        if (add > cmdMaxVelStep[i]) cmdMaxVelStep[i] = add;
                                    }
                                    cmdPrevStep[i] = d;
                                    cmdHaveStep[i] = true;
                                }
                                cmdPrevCnt[i] = cnt;
                                cmdSeen[i] = true;
                            }
                        }
                    }
                    else
                    {
                        justEnabled[i] = false;
                    }
                }

                // ---- Hardware e-stop detection (QuickStopActive) ----
                {
                    bool anyQSA = false;
                    for (int i = 0; i < numDrives; ++i)
                    {
                        if (driveCache[i] && driveCache[i]->isQuickStopActive())
                        {
                            anyQSA = true;
                            break;
                        }
                    }
                    if (anyQSA && !hwEstopActive)
                    {
                        hwEstopActive = true;
                        // Record whether the software e-stop was already active when
                        // QuickStopActive first appeared.  If yes, our own disableOperation()
                        // caused the QuickStop -- the user must release it manually.
                        // If no, it was a physical hardware button -- auto-clear on release.
                        hwEstopWasSoftware = motionOut.emergencyStop;
                        if (m_motion) m_motion->setEmergencyStop(true);
                        RT_LOG_WARNING("ControlLoop: Hardware e-stop detected (QuickStopActive) -- "
                                    "triggering emergency stop. (sw-initiated=%s)",
                                    hwEstopWasSoftware ? "yes" : "no");
                    }
                    else if (!anyQSA && hwEstopActive)
                    {
                        hwEstopActive = false;
                        if (!hwEstopWasSoftware)
                        {
                            // Physical hardware button was released -- safe to auto-clear.
                            if (m_motion) m_motion->setEmergencyStop(false);
                            RT_LOG_INFO("ControlLoop: Hardware e-stop released -- drives exited QuickStopActive.");
                        }
                        else
                        {
                            // Software e-stop caused the QuickStop. Drives finished QuickStop
                            // and are now in SwitchOnDisabled. User must press Clear E-Stop.
                            RT_LOG_INFO("ControlLoop: Drives exited QuickStopActive (software e-stop still held -- press Clear E-Stop to resume).");
                        }
                    }
                }

                // ---- WKC check ----
                bool wkcOk = wkcMonitor.check(
                    lastWkc, m_master->getExpectedWKC(),
                    ctx, m_master->getSlaveCount(), cycleCount);

#ifdef SOEM_AVAILABLE
                // Recovery is deferred to EtherCATMaster's dedicated thread:
                // the RT loop never stalls on ecx_recover_slave or DC re-arm.
                // It signals an atomic flag and the recovery thread runs the
                // full 4-branch decision tree. Signal at most once per
                // statsInterval window to avoid wake-up storms.
                if (!wkcOk && wkcMonitor.isInRecovery() && ctx && !hwEstopActive)
                {
                    static uint64_t lastRecoverySignal = 0;
                    if (cycleCount - lastRecoverySignal > statsInterval)
                    {
                        lastRecoverySignal = cycleCount;
                        RT_LOG_WARNING("ControlLoop: WKC dipped - signalling recovery thread.");
                        if (m_master)
                            m_master->signalRecoveryNeeded();
                    }
                }
#endif
            }
        }

        // ---- 5. Update shared drive statuses ----
        updateDriveStatuses();

        // ---- 6. Publish stats once per second ----
        if (cycleCount - lastStatsEmit >= statsInterval)
        {
            lastStatsEmit = cycleCount;
            LoopStats stats;
            stats.cycleCount = cycleCount;
            stats.loopHz = static_cast<double>(m_config.controlLoopHz);
            stats.maxJitterUs = maxJitterUs;
            stats.wkcErrors = static_cast<int>(wkcMonitor.getTotalErrors());
            stats.running = true;
            {
                // try_lock: RT thread must never wait on a lock a preempted
                // UI/web reader may hold (no priority inheritance). Stats
                // are re-published next second if this attempt is skipped.
                std::unique_lock<std::shared_mutex> lock(m_statusLock, std::try_to_lock);
                if (lock.owns_lock()) m_stats = stats;
            }
            if (m_onStatsUpdated) m_onStatsUpdated(stats);

            // Per-second following-error telemetry goes to the DIAG stream,
            // not the main log: at one line per drive per second it was ~90%
            // of main-log volume on long sessions, drowning the events the
            // log exists for. The web drive cards carry the live value; the
            // latched peak survives in the session summary at loop exit.
            // Skip PP drives -- their following error is drive-internal
            // profile lag, not a fault.
            for (int i = 0; i < numDrives && i < MAX_DRIVES; ++i)
            {
                bool isPP = (i < (int)m_config.drives.size() &&
                             m_config.drives[i].mode == "pp");
                if (!isPP && maxFollowingError[i] > 0.01)
                {
                    RT_DIAG("DIAG | ferr | drive=%d | max_mm=%.3f | hz=%d",
                        driveCache[i]->getSlaveIndex(), maxFollowingError[i],
                        m_config.controlLoopHz);
                }
                maxFollowingError[i] = 0.0;

                // Command-dither fingerprint (diag-gated): reversals/s + tiny maxStep,
                // or a large maxVelStep, = setpoint dither / low-send-rate staircase
                // (a command-cleanliness signal).
                if (cmdReversals[i] > 0 || cmdMaxVelStep[i] > 1)
                    RT_DIAG("DIAG | cmd | drive=%d | reversals_per_s=%d | maxStep=%d | maxVelStep=%d | hz=%d",
                        driveCache[i]->getSlaveIndex(), cmdReversals[i],
                        cmdMaxStep[i], cmdMaxVelStep[i], m_config.controlLoopHz);
                cmdReversals[i]  = 0;
                cmdMaxStep[i]    = 0;
                cmdMaxVelStep[i] = 0;
            }

            // DC phase walk (diag-gated): a drifting "now" / large 1s span = the
            // loop is NOT phase-locked to the DC reference (frames slide vs SYNC0).
            // A locked loop holds this ~constant.
            if (m_master && m_master->isOperational() && dcPhaseMax >= dcPhaseMin)
                RT_DIAG("DIAG | dc_phase | now_ns=%lld | span_min=%lld | span_max=%lld | span_ns=%lld"
                    " | lock=%d | locked=%d | target_ns=%lld | err_ns=%.0f | trim_ns=%.0f",
                    (long long)m_master->getDcPhaseNs(),
                    (long long)dcPhaseMin, (long long)dcPhaseMax,
                    (long long)(dcPhaseMax - dcPhaseMin),
                    dcLock.enabled() ? 1 : 0, dcLock.locked() ? 1 : 0,
                    (long long)dcLock.targetNs(), dcLock.lastErrNs(), dcLock.lastTrimNs());
            dcPhaseMin = INT64_MAX; dcPhaseMax = INT64_MIN;

            maxJitterUs = 0.0;

            // A17: RTT DIAG - emitted only when sendReceive was actually called.
            if (rttCount > 0)
            {
                RT_DIAG("DIAG | rtt | cycle=%llu | count=%llu | min_us=%.1f | max_us=%.1f | avg_us=%.1f",
                    (unsigned long long)cycleCount, (unsigned long long)rttCount,
                    rttMinUs, rttMaxUs, rttSumUs / static_cast<double>(rttCount));
                rttMinUs = 1e9;
                rttMaxUs = 0.0;
                rttSumUs = 0.0;
                rttCount = 0;
            }

            if (wkcMonitor.getConsecutiveErrors() > 0)
                RT_LOG_WARNING("ControlLoop: WKC errors this second: total=%llu consecutive=%d recoveries=%d",
                    (unsigned long long)wkcMonitor.getTotalErrors(),
                    wkcMonitor.getConsecutiveErrors(),
                    wkcMonitor.getRecoveryAttempts());
        }
    }

    // ---- Park and stop ----
    if (m_motion)
    {
        for (int i = 0; i < numDrives; ++i)
        {
            if (m_motion->getAxisState(i) == AxisMotionState::HOMING)
            {
                m_motion->forceAxisParked(i);
                RT_LOG_INFO("ControlLoop: Axis %d forced PARKED (was HOMING at shutdown).", i);
            }
        }

        // Normal loop stop just parks (the drives stay energized in OP via the pump).
        // Seating onto the bottom stop is NOT done here -- it belongs to the de-init path
        // (ControlLoop::seatThenStop, which runs the homing-based seat then de-energizes),
        // because that is the action that actually removes power. Real hardware + sim both
        // run the same bounded park loop.
        m_motion->startPark();

        const int maxParkCycles = static_cast<int>(m_config.controlLoopHz * 8.0);
        int parkCycles = 0;

        while (!m_motion->allAxesParked() && parkCycles < maxParkCycles)
        {
            PlatformRT::waitUntil(nextCycleTime);
            PlatformRT::advancePeriod(nextCycleTime, cycleCountsTarget);
            ++parkCycles;

            TelemetryData emptyData;
            MotionOutput motionOut;
            m_motion->process(emptyData, motionOut, driveCache, numDrives, ctx);

            if (m_master && m_master->isOperational())
            {
                if (m_master->isSimulation())
                {
                    for (int i = 0; i < numDrives; ++i)
                    {
                        A6Drive* drive = driveCache[i];
                        if (!drive) continue;
                        double target = (i < motionOut.numDrives) ? motionOut.positions[i] : 0.0;
                        double actual = drive->getActualPosition();
                        drive->setSimPosition(actual + 0.1 * (target - actual));
                        drive->setSimTarget(target);
                    }
                }
                else
                {
                    for (int i = 0; i < numDrives; ++i)
                    {
                        A6Drive* drive = driveCache[i];
                        if (!drive) continue;
                        double pos = (i < motionOut.numDrives) ? motionOut.positions[i] : 0.0;
                        drive->setTargetPosition(pos);
                    }
                    m_master->sendReceive();
                }
            }
            updateDriveStatuses();
        }

        if (parkCycles >= maxParkCycles)
            RT_LOG_WARNING("ControlLoop: Park timeout -- stopping anyway.");
        else
            RT_LOG_INFO("ControlLoop: All axes parked after %d cycles.", parkCycles);
    }

    // ---- Belt torque bleed (ErC1.2 guard) ----
    // The RPDO retains the last commanded 0x6071 after this loop stops, and the
    // background pump keeps transmitting it. If any torque drive still carries
    // tension here (park timeout, odd stop path), the drive keeps pulling with the
    // loop stopped, and a later de-init disables it under load -> ErC1.2 (same
    // disable-under-load class as the vertical seat issue). Zero every torque
    // drive's command and push one frame so no torque drive is EVER de-energized
    // (or pump-idled) with tension left in the buffer.
    if (m_master && m_master->isOperational() && !m_master->isSimulation())
    {
        bool bled = false;
        for (int i = 0; i < numDrives; ++i)
            if (driveCache[i] && driveCache[i]->isTorqueMode())
            { driveCache[i]->setTargetTorque(0.0); bled = true; }
        if (bled)
        {
            m_master->sendReceive();
            RT_LOG_INFO("ControlLoop: Belt torque bled to 0 before loop exit (ErC1.2 guard).");
        }
    }

    // ---- Release SOEM ownership, then restart background pump ----
    // Clear m_rtLoopActive before startPump() so startPumpBody() is not blocked.
    if (m_master)
        m_master->setRtLoopActive(false);

    // startPump() posts to the persistent dispatch thread inside
    // EtherCATMaster - safe to call from this dying RT thread.
    if (m_master && !m_master->isSimulation())
    {
        m_master->startPump();
        RT_LOG_INFO("ControlLoop: Background pump restart requested -- drive stays in OP.");
    }

    PlatformRT::threadTeardown(rtHandle);
    PlatformRT::timerEnd();

    RT_LOG_INFO("ControlLoop: Stopping after %llu cycles. Total WKC errors: %llu",
        (unsigned long long)cycleCount, (unsigned long long)wkcMonitor.getTotalErrors());

    // Per-axis conditioning/guard session summary -> standard log (not the soem diag).
    // Off the hot path (loop has exited). Skips axes that never conditioned (PP/torque/
    // never-online -> totalCyc==0).
    if (m_motion)
    {
        for (int i = 0; i < numDrives && i < MAX_DRIVES; ++i)
        {
            const auto g = m_motion->getGuardStats(i);
            if (g.totalCyc == 0) continue;
            LOG_INFO(strf("ControlLoop: Axis %d conditioning summary -- peak accel: windowed=%.0f mm/s^2 "
                "(macroscopic, the headroom gauge), single-cycle=%.0f mm/s^2 (worst corner), demand peak=%.0f%% "
                "of Amax, clipped %.1f%% of cycles | relative-braking binds %.2f%% of cycles, peak clamped "
                "%.1f mm/s | vt accuracy during binds: peak vt error=%.1f mm/s (vt=%.1f mm/s at that bind -- "
                "small error => clamp working on a real transient, large => vt was inaccurate) | peak following "
                "error=%.3f mm",
                driveCache[i]->getSlaveIndex(), g.accelWinPeakMms2, g.accelCmdPeakMms2, g.accelDemandPeakPct,
                g.clipPct, g.bindPct, g.bindPeakClampMms, g.bindVtErrPeakMms, g.bindVtAtErr, m_peakFollowingError[i]));
        }
    }

    // Disarm the RT log path and flush remaining queued entries before exit.
    Logger::instance().clearRTThread();
    {
        std::unique_lock<std::shared_mutex> lock(m_statusLock);
        m_stats.running = false;
    }
    if (m_onLoopStopped) m_onLoopStopped();
}

void ControlLoopWorker::updateDriveStatuses()
{
    if (!m_master) return;
    // Decimated publish: EMAs/velocity below stay per-cycle; the lock + copy +
    // callback run only every m_statusPublishDivider-th call (~50Hz band).
    const bool publishThisCycle =
        (m_statusPublishTick++ % static_cast<uint64_t>(m_statusPublishDivider)) == 0;
    int n = m_master->getDriveCount();
    for (int i = 0; i < n && i < MAX_DRIVES; ++i)
    {
        A6Drive* drive = m_master->getDrive(i);
        if (!drive) continue;

        DriveStatus ds;
        ds.driveIndex = i;
        ds.state = drive->getState();
        ds.actualPos = drive->getActualPosition();
        ds.statusword = drive->getStatusword();
        ds.torquePct = drive->getTorquePercent();

        // Velocity = position derivative, per cycle, lightly EMA-smoothed to tame
        // encoder quantization. Primed on first pass to avoid a start-up spike.
        const double hz = (m_cycleTimeUs > 0.0) ? (1e6 / m_cycleTimeUs) : 0.0;
        if (m_velPrimed)
        {
            const double rawVel = (ds.actualPos - m_prevPos[i]) * hz;
            m_velFilt[i] += 0.2 * (rawVel - m_velFilt[i]);
        }
        m_prevPos[i] = ds.actualPos;
        ds.velocity = m_velFilt[i];
        // Thermal duty proxy: ~60s exponential RMS of torque (heating ~ torque^2).
        // Sustained readings above ~100% of rated mean the drive's i2t clock is running.
        {
            const double dt = (m_cycleTimeUs > 0.0) ? m_cycleTimeUs * 1e-6 : 0.0005;
            const double alpha = dt / 60.0;
            m_trqSqEma[i] += alpha * (ds.torquePct * ds.torquePct - m_trqSqEma[i]);
            ds.torqueRms60 = std::sqrt(std::max(0.0, m_trqSqEma[i]));
        }
        ds.peakFollowingError = m_peakFollowingError[i];

        if (publishThisCycle)
        {
            {
                // try_lock: same no-wait rule as the stats publish above.
                std::unique_lock<std::shared_mutex> lock(m_statusLock, std::try_to_lock);
                if (lock.owns_lock()) m_driveStatus[i] = ds;
            }
            if (m_onDriveStatus) m_onDriveStatus(i, ds);
        }
    }
    m_velPrimed = true;   // subsequent passes derive velocity from the delta
}

void ControlLoopWorker::handleEnableStateMachines(bool /*enableRequested*/) {}

// ============================================================
// ControlLoop -- thread lifecycle
// ============================================================

ControlLoop::ControlLoop() {}

ControlLoop::~ControlLoop()
{
    stop();
    if (m_thread.joinable())
        m_thread.join();
    delete m_worker;
    m_worker = nullptr;
}

void ControlLoop::setComponents(EtherCATMaster* master, TelemetryInput* telemetry, MotionController* motion)
{
    m_master = master; m_telemetry = telemetry; m_motion = motion;
}

void ControlLoop::setConfig(const AppConfig& config) { m_config = config; }

bool ControlLoop::start()
{
    if (m_running.load()) { LOG_WARNING("ControlLoop::start(): already running."); return false; }
    if (!m_master || !m_telemetry || !m_motion) { LOG_ERROR("ControlLoop::start(): components not set."); return false; }
    if (!m_master->isOperational()) { LOG_ERROR("ControlLoop::start(): EtherCAT not in OP."); return false; }

    // The start_control_loop DIAG is emitted here so both UIs (Qt + web)
    // produce the same SOEM-log breadcrumb. The guards above have passed,
    // so this reflects the state immediately before the worker thread
    // spawns.
    {
        int mstate = static_cast<int>(m_master->getMasterState());
        bool op    = m_master->isOperational();
        bool init  = m_master->isInitialized();
        Logger::instance().logDiag(strf(
            "DIAG | start_control_loop | masterState=%d | isOp=%d | isInit=%d",
            mstate, op ? 1 : 0, init ? 1 : 0));
    }

    // A previous run() may have exited while leaving m_thread joinable, and
    // assigning a new std::thread over a joinable handle calls
    // std::terminate() per the C++ standard. The handle stays joinable until
    // explicitly joined even after the worker loop exits, so stop the worker
    // (idempotent) and join before reassigning.
    if (m_worker) m_worker->requestStop();
    if (m_thread.joinable())
        m_thread.join();

    delete m_worker;
    m_worker = new ControlLoopWorker();
    m_worker->setComponents(m_master, m_telemetry, m_motion);
    m_worker->setConfig(m_config);

    // Forward callbacks from worker to ControlLoop's registered callbacks,
    // also updating m_running on start/stop transitions.
    m_worker->setOnLoopStarted([this]() {
        m_running.store(true);
        if (m_onLoopStarted) m_onLoopStarted();
    });
    m_worker->setOnLoopStopped([this]() {
        m_running.store(false);
        if (m_motion) m_motion->setNeedsRehome(true);
        if (m_onLoopStopped) m_onLoopStopped();
    });
    m_worker->setOnStatsUpdated([this](LoopStats s) {
        if (m_onStatsUpdated) m_onStatsUpdated(s);
    });
    m_worker->setOnDriveStatusUpdated([this](int idx, DriveStatus ds) {
        if (m_onDriveStatus) m_onDriveStatus(idx, ds);
    });
    m_worker->setOnError([this](const std::string& msg) {
        if (m_onError) m_onError(msg);
    });
    m_worker->setOnFaultLockout([this](int idx, const std::string& msg) {
        if (m_onFaultLockout) m_onFaultLockout(idx, msg);
    });

    m_thread = std::thread([this]() { m_worker->run(); });
    LOG_INFO("ControlLoop: Thread started.");
    return true;
}

void ControlLoop::stop(bool seatToStop)
{
    if (m_worker) m_worker->requestStop(seatToStop);
}

void ControlLoop::waitForStop()
{
    if (m_worker) m_worker->requestStop();
    if (m_thread.joinable())
        m_thread.join();
}

void ControlLoop::seatThenStop()
{
    // Deinit-path seat: the loop is already stopped (the UI only enables Stop EtherCAT
    // when stopped) and the background pump is holding the drives in OP. Spin up the
    // worker in seat-only mode -- it claims SOEM, presses the vertical axes onto the
    // bottom stop, de-energizes ON the stop, and exits without restarting the pump.
    // Synchronous: returns once seated + de-energized so the caller can shutdown().
    if (m_running.load()) { LOG_WARNING("ControlLoop::seatThenStop(): loop running -- skipping seat."); return; }
    if (!m_master || !m_motion) return;
    if (!m_master->isOperational() || m_master->isSimulation()) return;

    // Never assign over a joinable handle (std::terminate); a prior run()
    // may have exited while leaving m_thread joinable.
    if (m_worker) m_worker->requestStop();
    if (m_thread.joinable()) m_thread.join();

    delete m_worker;
    m_worker = new ControlLoopWorker();
    m_worker->setComponents(m_master, m_telemetry, m_motion);
    m_worker->setConfig(m_config);
    m_worker->setSeatOnlyMode(true);
    // Surface drive status during the press (torque rising) but no loop-started/stopped
    // transitions -- a seat-only pass never enters the running state.
    m_worker->setOnDriveStatusUpdated([this](int idx, DriveStatus ds) {
        if (m_onDriveStatus) m_onDriveStatus(idx, ds);
    });

    m_thread = std::thread([this]() { m_worker->run(); });
    if (m_thread.joinable()) m_thread.join();
    LOG_INFO("ControlLoop: Seat pass complete -- drives held on the stop; de-init will de-energize.");
}

LoopStats   ControlLoop::getStats()                const { return m_worker ? m_worker->getStats() : LoopStats{}; }
DriveStatus ControlLoop::getDriveStatus(int index) const { return m_worker ? m_worker->getDriveStatus(index) : DriveStatus{}; }

void ControlLoop::clearFaultLockout(int driveIndex)
{
    if (m_worker) m_worker->clearFaultLockout(driveIndex);
}

void ControlLoop::requestStatsReset()
{
    if (m_worker) m_worker->requestStatsReset();
}
