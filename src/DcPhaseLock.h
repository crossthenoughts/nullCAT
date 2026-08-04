// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// DcPhaseLock.h — DC phase-lock compensator
//
// A gentle, clamped PI compensator that locks the master's cycle phase to the
// EtherCAT DC reference clock by trimming the per-cycle period a few ns at a
// time. Without it the loop (and pump) free-run on the host's own oscillator,
// which differs from the DC reference (a drive's crystal) by a few ppm; the
// sampling instant then WALKS across the cycle (~2.5 ppm = ~5 ns/cycle at
// 500 Hz here) until it slides onto the SYNC0 latch boundary and faults the
// drive (Er74.1) — worst on long OP-holds and long sessions.
//
// Why closed-loop and not a hardcoded drift number:
//   The walk is the difference between two crystals (host vs DC reference). It
//   DIFFERS between systems in magnitude AND sign, and drifts slowly with
//   temperature. Lock-to-acquisition + the integrator self-discover the actual
//   drift and continuously re-track it — correct on every Pi/PC/drive combo
//   with no per-system constant.
//
// Loop-rate independence:
//   The PI is formulated in continuous time (dt = 1/controlLoopHz applied
//   explicitly), so the closed-loop bandwidth is in Hz and ONE set of gains
//   behaves identically at 250/500/1000 Hz. The mod base (cycleNs) and dt both
//   derive from the loop rate; the clamp is an absolute ns bound (a hard,
//   system-independent safety limit).
//
// Safety / opt-in:
//   Default OFF. When disabled the caller bypasses it entirely (the period it
//   feeds to advancePeriod() is the unchanged nominal), so behaviour is
//   byte-identical to a free-running loop. The clamp bounds the per-cycle period
//   perturbation regardless of gains or a bad DC reading, so even a mistuned or
//   wrong-signed loop can only nudge the period by ±maxTrimNs — it cannot run
//   the period away. Pure arithmetic, no allocation/locks: RT-hot-path safe.
//
// Usage per cycle (caller owns one instance per timing thread — no sharing):
//   int64_t period = nominalCounts;
//   if (lock.enabled()) period = lock.update(dcPhaseNs);   // dcPhaseNs = DCtime % cycleNs
//   PlatformRT::advancePeriod(deadline, period);
// ============================================================

#include <cstdint>
#include <cmath>

class DcPhaseLock
{
public:
    static constexpr int64_t kNoSample = INT64_MIN;   // "no DC phase available this cycle"

    struct Params
    {
        int64_t cycleNs       = 2000000;   // cycle / SYNC0 period (phase mod base), ns
        int64_t nominalCounts = 0;         // PlatformRT::periodCounts(cycleTimeUs) — platform tick units
        double  dtSec         = 0.002;     // cycle period in seconds (1/controlLoopHz)
        double  kp            = 2.5;       // PI proportional gain (rad/s-ish; damping)
        double  ki            = 1.6;       // PI integral gain (1/s; ~0.2 Hz bandwidth, critically damped)
        int64_t maxTrimNs     = 10000;     // absolute clamp on |period - nominal|, ns (hard safety bound)
        int64_t targetNs      = -1;        // <0 => lock-to-acquisition (capture first settled sample)
        int     warmupCycles  = 500;       // settle before priming the target (~1 s at 500 Hz)
    };

    void configure(const Params& p)
    {
        m_p = p;
        // ns -> platform count units. nominalCounts corresponds to cycleNs, so
        // counts-per-ns = nominalCounts / cycleNs (1.0 on Linux where a tick is
        // 1 ns; QPF/1e9 on Windows). Computed once; the trim is tiny so this
        // single multiply is all the per-cycle conversion needs.
        m_countsPerNs = (p.cycleNs > 0) ? (double)p.nominalCounts / (double)p.cycleNs : 1.0;
        reset();
    }

    void reset()
    {
        m_integ     = 0.0;
        m_cycles    = 0;
        m_primed    = false;
        m_target    = 0;
        m_lastErr   = 0.0;
        m_lastTrim  = 0.0;
    }

    bool enabled() const     { return m_enabled; }
    void setEnabled(bool e)  { m_enabled = e; }

    // Returns the corrected period (in platform count units) for the next
    // advancePeriod(). Pass kNoSample on cycles with no fresh DC phase (e.g.
    // before OP) — it holds nominal and does not advance internal state.
    int64_t update(int64_t dcPhaseNs)
    {
        if (dcPhaseNs == kNoSample) return m_p.nominalCounts;

        // Settle: let the loop reach steady state before locking onto a target.
        if (m_cycles < m_p.warmupCycles) { ++m_cycles; return m_p.nominalCounts; }

        if (!m_primed)
        {
            // Lock-to-acquisition: hold wherever the loop naturally settled (a
            // known-good operating point), unless an explicit target is set.
            m_target = (m_p.targetNs >= 0) ? m_p.targetNs : dcPhaseNs;
            m_primed = true;
            m_integ  = 0.0;
        }

        // Circular phase error, shortest signed distance in (-cycleNs/2, cycleNs/2].
        int64_t e = dcPhaseNs - m_target;
        const int64_t half = m_p.cycleNs / 2;
        if (e >  half) e -= m_p.cycleNs;
        if (e < -half) e += m_p.cycleNs;

        // PI in continuous time: u (ns/s) is a frequency command; trim = -dt*u.
        // Sign: phase below target (e<0) => lengthen the period (trim>0) to let
        // the DC clock catch back up — verified against the observed -5 ns/cycle
        // walk (master period runs short, so we add time).
        const double u    = m_p.kp * (double)e + m_p.ki * m_integ;
        double       trim = -m_p.dtSec * u;

        const double maxT = (double)m_p.maxTrimNs;
        bool clamped = false;
        if (trim >  maxT) { trim =  maxT; clamped = true; }
        if (trim < -maxT) { trim = -maxT; clamped = true; }

        // Anti-windup: only accumulate when the actuator (period trim) is not
        // saturated, so a transient can't wind the integrator up.
        if (!clamped) m_integ += (double)e * m_p.dtSec;

        m_lastErr  = (double)e;
        m_lastTrim = trim;
        return m_p.nominalCounts + (int64_t)std::llround(trim * m_countsPerNs);
    }

    // Diagnostics (for the diag-gated per-second log).
    bool    locked()    const { return m_primed; }
    int64_t targetNs()  const { return m_target; }
    double  lastErrNs() const { return m_lastErr; }
    double  lastTrimNs()const { return m_lastTrim; }

private:
    Params  m_p;
    bool    m_enabled     = false;
    double  m_countsPerNs = 1.0;
    double  m_integ       = 0.0;
    int     m_cycles      = 0;
    bool    m_primed      = false;
    int64_t m_target      = 0;
    double  m_lastErr     = 0.0;
    double  m_lastTrim    = 0.0;
};
