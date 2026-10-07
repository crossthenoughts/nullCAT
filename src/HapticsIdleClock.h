// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// HapticsIdleClock - steps the haptics layer while the control loop is
// not running, so shakers play with the drives off, or with no drives at
// all (a shaker-only install is a standalone haptics engine fed by the
// same sim channels). A plain timer thread at the control rate; it hands
// over the moment the loop starts (MotionController::hapticsIdleTick is
// guarded against overlapping the RT cycle) and takes back when it stops.
// ============================================================

#include "ControlLoop.h"
#include "MotionController.h"
#include "TelemetryInput.h"
#include <atomic>
#include <chrono>
#include <thread>
#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#else
#  include <pthread.h>
#  include <sched.h>
#endif

class HapticsIdleClock
{
public:
    HapticsIdleClock(MotionController& motion, TelemetryInput& telemetry, ControlLoop& loop, int controlHz)
        : m_motion(motion), m_telemetry(telemetry), m_loop(loop),
          m_period(std::chrono::microseconds(1000000 / std::max(100, controlHz))) {}
    ~HapticsIdleClock() { stop(); }

    void start()
    {
        if (m_running.exchange(true)) return;
        m_thread = std::thread([this]()
        {
            // Elevated but below the RT loop: SCHED_FIFO 10 on Linux (the
            // service has rtprio), HIGHEST on Windows. Absolute-time sleeps
            // keep the long-run rate exact; the shaker servo absorbs jitter.
#ifdef _WIN32
            SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
#else
            { sched_param sp{}; sp.sched_priority = 10; pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp); }
#endif
            auto next = std::chrono::steady_clock::now();
            while (m_running.load(std::memory_order_relaxed))
            {
                next += m_period;
                std::this_thread::sleep_until(next);
                if (m_loop.isRunning()) { next = std::chrono::steady_clock::now(); continue; }   // the RT loop owns the layer and the socket
                // The control loop normally pumps the telemetry socket; with
                // it stopped this clock does (same bounded drain), so the sim
                // channels reach the layer on a rig with no loop running.
                for (int drained = 0; drained < 32 && m_telemetry.receive(); ++drained) {}
                m_motion.hapticsIdleTick(m_telemetry.getLatestData());
            }
        });
    }
    void stop()
    {
        if (!m_running.exchange(false)) return;
        if (m_thread.joinable()) m_thread.join();
    }
    bool running() const { return m_running.load(); }

private:
    MotionController& m_motion;
    TelemetryInput&   m_telemetry;
    ControlLoop&      m_loop;
    std::chrono::microseconds m_period;
    std::atomic<bool> m_running{false};
    std::thread       m_thread;
};
