// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// CommandConditioner.h
//
// Three command-conditioning modes over ONE shared guard chain. A mode only
// chooses how the raw target becomes the pre-clamp command; the guard chain is
// ALWAYS applied and is not part of the mode. Position safety does not depend on
// the mode.
//
//   Bypass      Raw target, guard only. We consume the host motion software's
//               ALREADY-INTERPOLATED output (SimHub/DRSM/FlyPT, ~300-1000 Hz) --
//               NOT game telemetry -- so on a sane UDP rate the stream is already
//               smooth and there is nothing to reconstruct. Lowest latency.
//   Interpolate Target-relative first-order fill toward the latest target over one
//               nominal frame interval. For a LOW UDP SEND rate (a host left at its
//               100 Hz default) -- a send-rate handler, NOT a "slow game" handler.
//               Fills gaps between sparse frames; no low-pass. Latency ~= frameSec.
//   Filter      Exact 2nd-order low-pass (knee = wn). Feel-shaping low-pass for
//               aggressive sources (FlyPT) / comfort. Latency 2/wn.
//
// GUARD CHAIN (always-on, mode-independent safety envelope; stroke min/max is
// applied by the caller): accel clamp (Amax), velocity clamp (Vmax), and a
// TARGET-VELOCITY-AWARE braking clamp. The braking clamp limits the command's
// velocity RELATIVE to the target's own motion (vt): vel <= vt + sqrt(2*amax*err)
// toward the target. Tracking a moving target is therefore NOT penalized -- only
// the closing velocity is bounded -- which avoids the v^2/2amax following-lag that
// an absolute braking clamp imposes (measured ~20-30 ms at 200-300 mm/s). When the
// target-velocity estimate is unavailable (startup) or the target has stopped, vt
// collapses to 0 and the clamp degrades to the conservative ABSOLUTE braking
// clamp, so overshoot protection is uniform at all times.
//
// MODE INTERFACE: every mode keeps commanded (pos, vel) in shared state, so
// getCommandedState()/seedState() are uniform -- the BLENDING->ONLINE handoff
// seeds the live commanded pos/vel into whichever mode is active and never pops.
//
// FRAMING: a command "staircase" is an artifact of a low UDP SEND rate, NOT a
// game-telemetry floor. Filter LOW-PASSES; Interpolate is a DISTINCT algorithm.
// ============================================================

#include <cmath>

struct CommandConditioner
{
    enum class Mode { Bypass, Interpolate, Filter };

    double pos             = 0.0;   // commanded position (engineering units, e.g. mm)
    double vel             = 0.0;   // commanded velocity
    double accel           = 0.0;   // last commanded acceleration (recorded for diagnostics)
    double lastValidTarget = 0.0;   // last finite target seen (NaN/Inf reject)

    // Target-velocity estimate (frame-delta), used by the relative braking clamp.
    double targetVel       = 0.0;   // current estimate (0 = unavailable/stopped -> absolute braking)
    double vtLastTarget    = 0.0;   // last target value at which targetVel was computed
    double vtHoldSec       = 0.0;   // time the target has been unchanged
    bool   vtHave          = false; // have we anchored vtLastTarget yet?

    // vt accuracy: the held vt is a zero-order-hold PREDICTION of the target's velocity
    // (assume it keeps last frame's rate). When the next frame arrives, the realized
    // frame velocity is the ground truth, so |realized - held vt| is the vt-vs-ACTUAL-
    // target-velocity error. We carry the most recent such error so a bind can record
    // how reliable vt was when it engaged -- a bind with small vt error is the clamp
    // doing its job on a genuine fast transient; a bind with large vt error means vt
    // was inaccurate (stale/lagging) when it clamped.
    double vtPredErr       = 0.0;   // most recent |realized - predicted| target velocity (mm/s)
    bool   vtPredHave      = false; // have a prior estimate to measure prediction error against?

    // ---- Guard diagnostics: session-cumulative accumulators, RT-thread-only,
    //      pure arithmetic (no I/O). reset on loop-start; read live (cards) and at
    //      loop-stop (log). The command output never depends on these. ----
    double gAccelCmdPeak   = 0.0;   // peak |COMMANDED accel| (post-guard, mm/s^2) -- the card bar
    double gAccelDemandPeak= 0.0;   // peak |pre-clamp accel demand| / Amax (>1 = how far over) -- log
    long   gTotal          = 0;     // total conditioning cycles
    long   gClip           = 0;     // cycles the Amax clamp bound (demand > Amax) -- the clip rate
    long   gBind           = 0;     // cycles the relative-braking clamp bound
    double gBindPeakClamp  = 0.0;   // peak velocity removed by a bind (mm/s)
    // vt accuracy DURING binds -- the metric that tells a bind apart from a problem.
    double gBindVtErrPeak  = 0.0;   // peak |vt - actual target vel| at a bind (mm/s)
    double gBindVtAtErr    = 0.0;   // the vt in use at that worst-vt-error bind (for context)

    // Windowed (macroscopic) commanded accel -- the CARD headroom gauge. The per-cycle
    // commanded accel pegs at exactly Amax whenever the accel clamp binds even once,
    // which is every telemetry-frame boundary at a high loop rate (the command has a
    // position step in Bypass, a velocity step in Interpolate). So the per-cycle metric
    // reads ~100% at EVERY Amax and can't show headroom. Measuring |dvel| over a short
    // window instead dilutes those single-cycle discontinuities and reports the accel
    // the actuator actually has to produce: it sits BELOW Amax when there is headroom,
    // and only approaches Amax when the command genuinely sustains max accel. Latched
    // peak (drive, pause, then read -- you can't watch live).
    static constexpr double ACCEL_WIN_SEC = 0.020;  // 20 ms macroscopic window: spans >=2 frames even
                                                    // at a 100 Hz host send rate (10 ms frames), so it
                                                    // dilutes per-frame discontinuities on slow sources too
    static constexpr int    ACCEL_WIN_CAP = 128;    // ring capacity >= window/dt at the 4 kHz max loop (=80)
    double gAccelWinPeak   = 0.0;   // latched peak windowed |accel| (mm/s^2) -- the card gauge
    double velHist[ACCEL_WIN_CAP] = {};   // ring of recent commanded velocities
    int    velHistHead     = 0;     // next write index
    int    velHistCount    = 0;     // valid samples (ramps up after a reset/seed)

    struct GuardStats {
        double accelCmdPeakMms2=0;   // peak per-cycle commanded accel (worst single-cycle corner) -- log
        double accelWinPeakMms2=0;   // peak windowed/macroscopic commanded accel -- the card gauge
        double accelDemandPeakPct=0; // peak demand/Amax % (>100 = how far over) -- log
        double clipPct=0;            // % cycles the Amax clamp bound -- card + log
        double bindPct=0;            // % cycles relative-braking bound -- log
        double bindPeakClampMms=0;   // peak velocity the brake clamp removed -- log
        double bindVtErrPeakMms=0;   // peak vt-vs-actual-target-velocity error during binds -- log
        double bindVtAtErr=0;        // vt in use at that worst-vt-error bind -- log
        long   totalCyc=0;
    };
    void resetGuardStats()
    {
        gAccelCmdPeak=gAccelDemandPeak=0.0; gTotal=gClip=gBind=0;
        gBindPeakClamp=gBindVtErrPeak=gBindVtAtErr=0.0;
        gAccelWinPeak=0.0; velHistHead=0; velHistCount=0;
    }
    GuardStats getGuardStats() const
    {
        GuardStats g;
        g.accelCmdPeakMms2   = gAccelCmdPeak;
        g.accelWinPeakMms2   = gAccelWinPeak;
        g.accelDemandPeakPct = gAccelDemandPeak * 100.0;
        g.clipPct            = gTotal ? (100.0 * gClip / gTotal) : 0.0;
        g.bindPct            = gTotal ? (100.0 * gBind / gTotal) : 0.0;
        g.bindPeakClampMms   = gBindPeakClamp;
        g.bindVtErrPeakMms   = gBindVtErrPeak;
        g.bindVtAtErr        = gBindVtAtErr;
        g.totalCyc = gTotal;
        return g;
    }

    // ---- Mode interface: uniform commanded state for handoff continuity ----
    struct State { double pos = 0.0; double vel = 0.0; };
    State  getCommandedState() const { return { pos, vel }; }
    void   seedState(double p, double v)
    {
        pos = p; vel = v; accel = 0.0; lastValidTarget = p;
        targetVel = 0.0; vtLastTarget = p; vtHoldSec = 0.0; vtHave = false;
        vtPredErr = 0.0; vtPredHave = false;
        velHistHead = 0; velHistCount = 0;   // restart the accel window across the seam (latched peak persists)
    }
    void   reset(double p, double v) { seedState(p, v); }   // legacy alias

    // ---- Target-velocity estimate from the frame stream (shared by all modes) ----
    // vt = change in target over one nominal frame interval. Held while the target
    // is unchanged, but ZEROED once it has been held longer than ~2 frame intervals
    // (the target has stopped) so a stale vt cannot itself drive an overshoot.
    // frameSec <= 0 (no estimate yet) -> vt = 0 (absolute braking).
    void updateTargetVel(double target, double dt, double frameSec)
    {
        if (frameSec <= 0.0) { targetVel = 0.0; vtLastTarget = target; vtHoldSec = 0.0; vtHave = true; vtPredHave = false; vtPredErr = 0.0; return; }
        if (!vtHave)         { vtLastTarget = target; targetVel = 0.0; vtHoldSec = 0.0; vtHave = true; vtPredHave = false; vtPredErr = 0.0; return; }
        if (target != vtLastTarget)
        {
            const double realized = (target - vtLastTarget) / frameSec;   // ground truth this frame
            // vt-vs-actual error: how wrong the held vt prediction was. Skipped on the
            // first realized velocity (the prior 0 was an anchor, not a prediction). A
            // stale-low vt (decayed to 0 while the target was held) that mispredicts a
            // resumed fast target shows up here as a large error -- exactly the case the
            // user wants surfaced.
            if (vtPredHave) vtPredErr = std::fabs(realized - targetVel);
            vtPredHave   = true;
            targetVel    = realized;
            vtLastTarget = target;
            vtHoldSec    = 0.0;
        }
        else
        {
            vtHoldSec += dt;
            if (vtHoldSec > 2.0 * frameSec) targetVel = 0.0;   // target stopped -> conservative
        }
    }

    // ---- Shared guard chain (always-on). err = target - pos. brakeEps disables
    //      the braking clamp in a tiny band around the target. ----
    double applyGuard(double candidateVel, double err, double dt,
                      double vmax, double amax, double brakeEps)
    {
        const double velPrev = vel;                        // for post-guard commanded accel
        const double aeff = (candidateVel - vel) / dt;     // pre-clamp accel demand
        // accel demand diagnostics (side-channel; does not affect the command)
        const double demandFrac = (amax > 0.0) ? std::fabs(aeff) / amax : 0.0;
        ++gTotal;
        if (demandFrac > gAccelDemandPeak) gAccelDemandPeak = demandFrac;
        if (demandFrac > 1.0) ++gClip;

        if (aeff >  amax) candidateVel = vel + amax * dt;   // (1) acceleration clamp
        if (aeff < -amax) candidateVel = vel - amax * dt;
        accel = (candidateVel - vel) / dt;
        vel = candidateVel;

        if (vel >  vmax) vel =  vmax;                       // (2) velocity clamp
        if (vel < -vmax) vel = -vmax;

        bool braked = false;                               // did the braking clamp cut vel this cycle?
        if (std::fabs(err) > brakeEps)                      // (3) target-velocity-aware braking
        {
            const double vbrake = std::sqrt(2.0 * amax * std::fabs(err));
            const double cap = (err > 0.0) ? (targetVel + vbrake) : (targetVel - vbrake);
            const bool over = (err > 0.0) ? (vel > cap) : (vel < cap);
            if (over)
            {
                braked = true;
                const double removed = std::fabs(vel - cap);   // bind diagnostics (side-channel)
                ++gBind;
                if (removed > gBindPeakClamp) gBindPeakClamp = removed;
                // vt accuracy at this bind: a bind with a small vt error is the clamp
                // doing its job on a genuine transient; a large vt error means vt was
                // inaccurate when it clamped. vtPredErr is the most recent frame's
                // |realized - predicted| target velocity (held between frames).
                if (vtPredErr > gBindVtErrPeak) { gBindVtErrPeak = vtPredErr; gBindVtAtErr = targetVel; }
                vel = cap;
            }
        }

        // Peak COMMANDED accel, EXCLUDING braking-clamp cycles. The braking clamp
        // snaps velocity onto the safe envelope in one cycle -- a deceleration that is
        // NOT accel-limited, so (vel-velPrev)/dt on a bind cycle is a huge corner that
        // would dominate the peak and blow up as dt shrinks (it's really velRemoved/dt).
        // Those corners are reported separately as the relative-braking bind rate. What
        // remains here is the accel the acceleration clamp governs -- bounded by Amax --
        // which is the useful "how close am I to the Amax limit" headroom gauge.
        if (!braked)
        {
            const double cmdAccel = (vel - velPrev) / dt;
            if (std::fabs(cmdAccel) > gAccelCmdPeak) gAccelCmdPeak = std::fabs(cmdAccel);
        }

        pos += vel * dt;

        // Windowed (macroscopic) commanded accel: |vel_now - vel_(W cycles ago)| / (W*dt),
        // W spanning ~ACCEL_WIN_SEC. Single-cycle frame-boundary corners contribute only
        // amax*dt to the window's dvel, diluted over W cycles, so this does NOT peg at
        // Amax -- it reads the real accel the actuator must produce. Latched peak. Placed
        // AFTER the pos update so the command math stays contiguous -- the windowing is
        // pure side-channel and must not perturb the FP command output.
        {
            int W = (int)std::lround(ACCEL_WIN_SEC / dt);
            if (W < 1) W = 1; else if (W > ACCEL_WIN_CAP) W = ACCEL_WIN_CAP;
            if (velHistCount >= W)
            {
                int idx = velHistHead - W; if (idx < 0) idx += ACCEL_WIN_CAP;
                const double winAccel = (vel - velHist[idx]) / (W * dt);
                if (std::fabs(winAccel) > gAccelWinPeak) gAccelWinPeak = std::fabs(winAccel);
            }
            velHist[velHistHead] = vel;
            if (++velHistHead >= ACCEL_WIN_CAP) velHistHead = 0;
            if (velHistCount < ACCEL_WIN_CAP) ++velHistCount;
        }
        return pos;
    }

    // ---- Bypass: raw target, guard only. On a clean
    //      high-rate stream the clamps never bind -> true passthrough. ----
    double stepBypass(double target, double dt, double frameSec,
                      double vmax, double amax, double brakeEps)
    {
        if (std::isfinite(target)) lastValidTarget = target;
        else                       target = lastValidTarget;
        updateTargetVel(target, dt, frameSec);
        const double err = target - pos;
        return applyGuard(err / dt, err, dt, vmax, amax, brakeEps);  // close the gap; guard limits
    }

    // ---- Interpolate: target-relative first-order fill over one nominal frame
    //      interval (frameSec = 1/new_hz). Asymptotic -> never overshoots before
    //      the next frame, never freezes if a frame is dropped; self-adjusts on
    //      early/late/dropped frames and on UDP-rate changes. frameSec not yet
    //      measurable -> Bypass. ----
    double stepInterpolate(double target, double dt, double frameSec,
                           double vmax, double amax, double brakeEps)
    {
        if (std::isfinite(target)) lastValidTarget = target;
        else                       target = lastValidTarget;
        updateTargetVel(target, dt, frameSec);
        const double err = target - pos;
        const double candidateVel = (frameSec > dt) ? (err / frameSec)   // first-order fill
                                                     : (err / dt);        // startup -> Bypass
        return applyGuard(candidateVel, err, dt, vmax, amax, brakeEps);
    }

    // ---- Filter: exact 2nd-order low-pass step. wn in rad/s. ----
    double stepFilter(double target, double dt, double frameSec, double wn,
                      double vmax, double amax, double brakeEps)
    {
        if (std::isfinite(target)) lastValidTarget = target;
        else                       target = lastValidTarget;
        updateTargetVel(target, dt, frameSec);
        const double err = target - pos;
        const double p   = wn * dt;
        const double e   = std::exp(-p);
        const double candidateVel = e * wn * wn * dt * err + e * (1.0 - p) * vel;
        return applyGuard(candidateVel, err, dt, vmax, amax, brakeEps);
    }
};
