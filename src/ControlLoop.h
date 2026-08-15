// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// ControlLoop.h
//
// Real-time cyclic control loop. ControlLoopWorker runs the
// EtherCAT/motion cycle on a high-priority thread;
// ControlLoop manages the thread lifecycle from the GUI
// thread. Qt-free: status and events flow out through
// std::function callbacks, timing via PlatformRT.
// ============================================================

#include "EtherCATMaster.h"
#include "TelemetryInput.h"
#include "MotionController.h"
#include "Config.h"

#include <functional>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <array>
#include <cstdint>
#include <string>

// Per-drive status snapshot shared with GUI
struct DriveStatus
{
    int        driveIndex   = 0;
    DriveState state        = DriveState::Unknown;
    double     actualPos    = 0.0;
    double     targetPos    = 0.0;
    double     velocity     = 0.0;   // mm/s, derived from position (per-cycle, EMA-smoothed)
    double     torquePct    = 0.0;   // % torque feedback (0x6077)
    double     torqueRms60  = 0.0;   // ~60s rolling RMS of torque (%) -- thermal duty proxy
                                     // (heating ~ torque^2; sustained >100 = i2t clock running)
    uint16_t   statusword   = 0;
    double     peakFollowingError = 0.0;  // |actual-commanded| peak since loop-start/soft-reset (mm)
};

// Overall loop statistics
struct LoopStats
{
    uint64_t cycleCount     = 0;
    double   loopHz         = 0.0;
    double   maxJitterUs    = 0.0;
    int      wkcErrors      = 0;
    bool     running        = false;
};

// ============================================================
// ControlLoopWorker -- runs in the high-priority thread
// ============================================================
class ControlLoopWorker
{
public:
    ControlLoopWorker() = default;

    void setComponents(
        EtherCATMaster* master,
        TelemetryInput*    telemetry,
        MotionController* motion
    );

    void setConfig(const AppConfig& config);
    void run();
    // seatToStop is accepted for source compatibility but IGNORED: seating onto the
    // bottom stop happens on the de-init path (ControlLoop::seatThenStop, homing-based),
    // not as part of a loop stop.
    void requestStop(bool /*seatToStop*/ = false)
    {
        m_stopRequested.store(true, std::memory_order_release);
    }

    // Deinit-path SEAT-ONLY pass: when set, run() skips the main loop (and auto-home),
    // presses the vertical axes onto the bottom stop (reverse-homing to torque), then
    // DE-ENERGIZES on the stop and exits WITHOUT restarting the pump. Set before the
    // worker thread is launched; consumed once in run().
    void setSeatOnlyMode(bool b) { m_seatOnlyMode.store(b, std::memory_order_release); }

    LoopStats   getStats()                const;
    DriveStatus getDriveStatus(int index) const;

    // Called from UI thread to clear a drive's fault lockout.
    // Sets an atomic bitmask bit; the RT loop clears driveFaultLocked[i]
    // on the next cycle, allowing the drive to retry.
    // driveIndex = -1 clears all drives.
    void clearFaultLockout(int driveIndex = -1);

    // Soft stats reset (matched set): UI thread requests; the RT loop clears the
    // per-axis guard stats AND the peak following-error together on the next cycle.
    // Lets the user re-baseline accel/bind/follow-error during Amax tuning WITHOUT
    // dropping the drives out of OP (no loop stop / reinit / rehome).
    void requestStatsReset() { m_statsResetRequested.store(true, std::memory_order_release); }

    // Status/event callbacks.
    // Called from RT thread - UI must marshal via QMetaObject::invokeMethod.
    void setOnLoopStarted(std::function<void()> cb)                              { m_onLoopStarted       = std::move(cb); }
    void setOnLoopStopped(std::function<void()> cb)                              { m_onLoopStopped       = std::move(cb); }
    void setOnStatsUpdated(std::function<void(LoopStats)> cb)                    { m_onStatsUpdated      = std::move(cb); }
    void setOnDriveStatusUpdated(std::function<void(int, DriveStatus)> cb)       { m_onDriveStatus       = std::move(cb); }
    void setOnError(std::function<void(const std::string&)> cb)                  { m_onError             = std::move(cb); }
    void setOnFaultLockout(std::function<void(int, const std::string&)> cb)      { m_onFaultLockout      = std::move(cb); }

private:
    EtherCATMaster*   m_master = nullptr;
    TelemetryInput*      m_telemetry = nullptr;
    MotionController* m_motion = nullptr;
    bool m_telemetryMotionActive = false;
    AppConfig m_config;

    std::atomic<bool>     m_stopRequested{false};
    // Deinit-path seat-only pass (see setSeatOnlyMode): run() seats + de-energizes, no main loop.
    std::atomic<bool>     m_seatOnlyMode{false};
    // Bitmask of drive indices whose lockout should be cleared by the RT loop.
    // UI thread sets bits via clearFaultLockout(); RT thread reads and clears them.
    std::atomic<uint32_t> m_clearLockoutMask{0};
    // Soft stats reset request (UI thread sets; RT loop consumes via exchange).
    std::atomic<bool>     m_statsResetRequested{false};
    // Peak following-error per axis, latched since loop-start/soft-reset (RT-only).
    double                m_peakFollowingError[MAX_DRIVES] = {};

    mutable std::shared_mutex m_statusLock;
    LoopStats                 m_stats;
    DriveStatus               m_driveStatus[MAX_DRIVES];

    // Per-cycle velocity derivation (mm/s) with light EMA smoothing.
    double m_prevPos[MAX_DRIVES] = {};
    double m_velFilt[MAX_DRIVES] = {};
    double m_trqSqEma[MAX_DRIVES] = {};  // EMA of torque^2 (tau ~60s) -> DriveStatus.torqueRms60
    bool   m_velPrimed = false;

    // Status-publish decimation: the EMAs above update every cycle, but the
    // m_statusLock write + m_onDriveStatus callback run only every Nth call,
    // targeting the ~50Hz band (divider = max(1, controlLoopHz/50)). Nothing
    // consumes status faster than a few Hz (web polls at 2Hz; UIs use timers),
    // while per-cycle publishing cost 4-6 write-locks/cycle on the RT thread --
    // on Windows (SRWLock, no priority inheritance) a preempted reader could
    // stall the loop for a scheduler quantum: the 30-50ms jitter-spike class.
    // LoopStats (once per second) is deliberately NOT decimated further.
    int      m_statusPublishDivider = 1;
    uint64_t m_statusPublishTick    = 0;

    double m_cycleTimeUs = 1000.0;

    void updateDriveStatuses();
    void handleEnableStateMachines(bool enableRequested);

    std::function<void()>                        m_onLoopStarted;
    std::function<void()>                        m_onLoopStopped;
    std::function<void(LoopStats)>               m_onStatsUpdated;
    std::function<void(int, DriveStatus)>        m_onDriveStatus;
    std::function<void(const std::string&)>      m_onError;
    std::function<void(int, const std::string&)> m_onFaultLockout;
};

// ============================================================
// ControlLoop -- manages the thread lifecycle from the GUI thread
// ============================================================
class ControlLoop
{
public:
    ControlLoop();
    ~ControlLoop();

    void setComponents(
        EtherCATMaster* master,
        TelemetryInput*    telemetry,
        MotionController* motion
    );

    void setConfig(const AppConfig& config);
    bool start();
    // seatToStop is accepted for source compatibility but IGNORED (see
    // ControlLoopWorker::requestStop); use seatThenStop() for the deinit seat.
    void stop(bool seatToStop = false);
    // Requests stop and blocks until the thread exits (bounded by park sequence ~8s max).
    // Safe to call from the UI thread.
    void waitForStop();
    // Deinit-path seat: with the loop ALREADY stopped (drives held in OP by the pump),
    // briefly spin up the worker in seat-only mode to press the vertical axes onto the
    // bottom stop and de-energize there, so the de-energize that follows doesn't
    // free-fall them. Synchronous (joins the worker). No-op in sim / when not OP / when
    // the loop is running. Call immediately before EtherCATMaster::shutdown().
    void seatThenStop();
    bool isRunning() const { return m_running.load(); }

    LoopStats   getStats()                const;
    DriveStatus getDriveStatus(int index) const;

    // Delegate to ControlLoopWorker::clearFaultLockout()
    void clearFaultLockout(int driveIndex = -1);

    // Delegate to ControlLoopWorker::requestStatsReset() (soft stats re-baseline).
    void requestStatsReset();

    // Register callbacks before start(). ControlLoop
    // forwards these to ControlLoopWorker.
    void setOnLoopStarted(std::function<void()> cb)                              { m_onLoopStarted       = std::move(cb); }
    void setOnLoopStopped(std::function<void()> cb)                              { m_onLoopStopped       = std::move(cb); }
    void setOnStatsUpdated(std::function<void(LoopStats)> cb)                    { m_onStatsUpdated      = std::move(cb); }
    void setOnDriveStatusUpdated(std::function<void(int, DriveStatus)> cb)       { m_onDriveStatus       = std::move(cb); }
    void setOnError(std::function<void(const std::string&)> cb)                  { m_onError             = std::move(cb); }
    void setOnFaultLockout(std::function<void(int, const std::string&)> cb)      { m_onFaultLockout      = std::move(cb); }

private:
    std::thread        m_thread;
    ControlLoopWorker* m_worker = nullptr;

    EtherCATMaster*   m_master = nullptr;
    TelemetryInput*      m_telemetry = nullptr;
    MotionController* m_motion = nullptr;

    AppConfig         m_config;
    std::atomic<bool> m_running{false};

    std::function<void()>                        m_onLoopStarted;
    std::function<void()>                        m_onLoopStopped;
    std::function<void(LoopStats)>               m_onStatsUpdated;
    std::function<void(int, DriveStatus)>        m_onDriveStatus;
    std::function<void(const std::string&)>      m_onError;
    std::function<void(int, const std::string&)> m_onFaultLockout;
};
