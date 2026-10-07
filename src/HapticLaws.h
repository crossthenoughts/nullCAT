// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// HapticLaws - the sim channels -> effect level laws, all of them, in one
// place and out of the controller.
//
// Each law is a few lines: which NcxValues tokens drive an effect and how
// (a flag, a 0-100 magnitude, an edge). Everything fails safe: a stale
// stream (NcxValues::fresh false), an unbound token or a zero magnitude
// all drive level 0, and the layer's release ramp fades the effect out
// rather than cutting it. Magnitude channels are 0-100 by wire convention.
//
// A web Test preview can force one effect to full level for a short
// time; the timers live in LawsState and decay here on the RT thread.
// Pure functions of their inputs: testable with a hand-built NcxValues
// and no controller.
// ============================================================

#include "HapticsLayer.h"
#include "HapticsRegistry.h"
#include "DeviceStateLayer.h"   // NcxValues
#include <algorithm>
#include <cmath>

namespace haptics {

struct LawsState
{
    double previewSec[FX_TYPE_COUNT] = {};   // >0 = Test preview running for that slot
    int    lastGear = 0;
    bool   gearSeen = false;
    // Rolling factor per wheel for the wheelSpeed fallback: speedKmh /
    // wheelSpeed learned while cruising (no brake, light throttle), so a
    // sender's wheel-speed unit (rev/s, rad/s, km/h) and a staggered tyre
    // set both come out as ratio 0 when rolling. Unknown until learned.
    double rollK[WHEEL_COUNT]     = {};
    bool   rollKnown[WHEEL_COUNT] = {};
};

namespace laws_k {
    constexpr double kEngineAliveRpm   = 400.0;    // alive from just above cranking so idle chunks
    constexpr double kPreviewIdleRpm   = 1100.0;   // the canned idle a Test preview runs
    constexpr double kAbsMinBrakePct   = 10.0;     // ABS needs the brake actually applied
    constexpr double kPreviewSec       = 2.0;      // Test preview length
    // Slip laws
    constexpr double kSlipOnsetFrac    = 0.6;      // lateral: nothing below this share of peak deg (normal cornering)
    constexpr double kLonOnsetRatio    = 0.15;     // longitudinal: peak grip sits around 0.1-0.2 ratio
    constexpr double kSlipMinSpeedKmh  = 5.0;      // ratios mean nothing at a standstill
    constexpr double kLockHzRefKmh     = 80.0;     // lock judder carrier = set hz at this road speed
    constexpr double kLockHzMin        = 0.4, kLockHzMax = 1.4;
    constexpr double kLoadWeightMax    = 1.5;      // loaded tyre weighting cap
    constexpr double kRollLearnMinKmh  = 30.0;
    constexpr double kRollLearnMaxThr  = 30.0;     // % throttle; above this a driven wheel may be slipping
    constexpr double kRollLearnMaxBrk  = 5.0;
    constexpr double kRollLearnRate    = 0.01;     // EWMA step per cycle while cruising
}

// Start a Test preview on a continuous/engine slot.
inline void startPreview(LawsState& st, FxType t)
{
    st.previewSec[static_cast<int>(t)] = laws_k::kPreviewSec;
}

// Drive every continuous effect and the gear-shift transient for THIS
// cycle from the channel values. Call before Layer::step().
inline void driveLaws(Layer& L, LawsState& st, const NcxValues& v, double dtSec)
{
    using namespace laws_k;
    const bool live = v.fresh;

    const auto previewOr = [&](FxType t, double level) -> double
    {
        double& left = st.previewSec[static_cast<int>(t)];
        if (left <= 0.0) return level;
        left -= dtSec;
        return 1.0;
    };
    const auto mag = [&](NcxValues::Token t) -> double
    {
        if (!live || !v.have[t]) return 0.0;
        return std::max(0.0, std::min(1.0, v.val[t] / 100.0));
    };
    const auto flag = [&](NcxValues::Token t) -> double
    { return (live && v.have[t] && v.val[t] > 0.5) ? 1.0 : 0.0; };

    // Engine: firing rate from rpm and the engine description; throttle is
    // the load; the limiter flag drives the cut gate. A preview with no sim
    // runs a canned idle.
    {
        const EngineParams& e = L.engineParams();
        const double cyl = std::max(1.0, e.cylinders);
        const double perRev = (e.layout > 2.5) ? cyl : cyl / 2.0;   // Wankel: one firing per rotor per rev
        double level = 0.0, fireHz = 0.0, load = 0.5;
        const bool lim = flag(NcxValues::Limiter) > 0.5;
        if (live && v.have[NcxValues::Rpm] && v.val[NcxValues::Rpm] > kEngineAliveRpm)
        {
            level  = 1.0;
            fireHz = v.val[NcxValues::Rpm] / 60.0 * perRev;
            if (v.have[NcxValues::ThrottlePct])
                load = std::max(0.0, std::min(1.0, v.val[NcxValues::ThrottlePct] / 100.0));
        }
        const double lv = previewOr(FxType::RpmVibe, level);
        if (lv > 0.0 && fireHz < 0.5)
            fireHz = kPreviewIdleRpm / 60.0 * perRev;
        L.driveEngine(lv, fireHz, load, lim);
    }

    // ABS: only while the sim says ABS is cycling AND the brake is applied
    // (some sims flicker the flag at zero brake).
    {
        const bool on = flag(NcxValues::AbsActive) > 0.5
                        && live && v.have[NcxValues::BrakePct] && v.val[NcxValues::BrakePct] > kAbsMinBrakePct;
        L.driveFx(FxType::AbsPulse, previewOr(FxType::AbsPulse, on ? 1.0 : 0.0), 0.0);
    }

    // Per-wheel slip. Raw physics in, severity out; the loaded tyre
    // weighted up when loads arrive; the old single channels as fallback.
    {
        const auto haveW = [&](int group) -> bool
        {
            for (int w = 0; w < WHEEL_COUNT; ++w) if (!v.have[ncxWheelToken(group, w)]) return false;
            return live;
        };
        double loadW[WHEEL_COUNT] = { 1.0, 1.0, 1.0, 1.0 };
        if (haveW(NcxValues::LoadFL))
        {
            double mean = 0.0;
            for (int w = 0; w < WHEEL_COUNT; ++w) mean += std::max(0.0, v.val[NcxValues::LoadFL + w]);
            mean /= WHEEL_COUNT;
            if (mean > 0.0)
                for (int w = 0; w < WHEEL_COUNT; ++w)
                    loadW[w] = std::max(0.0, std::min(kLoadWeightMax, std::max(0.0, v.val[NcxValues::LoadFL + w]) / mean));
        }
        const double speed = (live && v.have[NcxValues::SpeedKmh]) ? std::max(0.0, v.val[NcxValues::SpeedKmh]) : 0.0;

        // Lateral: slip angle (deg) past the onset share of peak deg.
        {
            const SlipParams& sp = L.slipParams(FxType::Skid);
            const double peak  = std::max(0.5, sp.peak);
            const double onset = kSlipOnsetFrac * peak;
            const bool preview = st.previewSec[static_cast<int>(FxType::Skid)] > 0.0;
            const bool perWheel = haveW(NcxValues::SlipAngleFL);
            const double single = mag(NcxValues::Skid);
            for (int w = 0; w < WHEEL_COUNT; ++w)
            {
                double sev = 0.0;
                if (perWheel)
                {
                    const double ang = std::fabs(v.val[NcxValues::SlipAngleFL + w]);
                    sev = std::max(0.0, std::min(1.0, (ang - onset) / std::max(1e-6, peak - onset)));
                }
                else if (live && v.have[NcxValues::Skid])
                    sev = single;
                sev = std::min(1.0, sev * loadW[w]);
                if (preview) sev = 1.0;
                const bool front = (w == WheelFL || w == WheelFR);
                L.driveSlip(FxType::Skid, w, front ? sev : 0.0, front ? 0.0 : sev);
            }
        }

        // Longitudinal: slip ratio (- locking, + spinning) past the onset
        // ratio up to peak ratio. Ratio from the sim, else from wheel
        // speeds with the learned rolling factor, else the lockup channel.
        {
            const SlipParams& sp = L.slipParams(FxType::Lockup);
            const double peak = std::max(kLonOnsetRatio + 0.05, sp.peak);
            const bool preview = st.previewSec[static_cast<int>(FxType::Lockup)] > 0.0;
            const bool haveRatio = haveW(NcxValues::SlipRatioFL);
            const bool haveWs    = !haveRatio && haveW(NcxValues::WheelSpeedFL) && v.have[NcxValues::SpeedKmh];
            const double single  = mag(NcxValues::Lockup);
            const bool moving = speed > kSlipMinSpeedKmh;

            // Learn the rolling factor while cruising on wheel speeds.
            if (haveWs && speed > kRollLearnMinKmh
                && (!v.have[NcxValues::BrakePct]    || v.val[NcxValues::BrakePct]    < kRollLearnMaxBrk)
                && (!v.have[NcxValues::ThrottlePct] || v.val[NcxValues::ThrottlePct] < kRollLearnMaxThr))
            {
                for (int w = 0; w < WHEEL_COUNT; ++w)
                {
                    const double ws = v.val[NcxValues::WheelSpeedFL + w];
                    if (ws <= 0.0) continue;
                    const double k = speed / ws;
                    if (!st.rollKnown[w]) { st.rollK[w] = k; st.rollKnown[w] = true; }
                    else st.rollK[w] += kRollLearnRate * (k - st.rollK[w]);
                }
            }

            for (int w = 0; w < WHEEL_COUNT; ++w)
            {
                double lock = 0.0, spin = 0.0;
                if (moving && haveRatio)
                {
                    const double r = v.val[NcxValues::SlipRatioFL + w];
                    lock = std::max(0.0, std::min(1.0, (-r - kLonOnsetRatio) / (peak - kLonOnsetRatio)));
                    spin = std::max(0.0, std::min(1.0, ( r - kLonOnsetRatio) / (peak - kLonOnsetRatio)));
                }
                else if (moving && haveWs && st.rollKnown[w])
                {
                    const double r = v.val[NcxValues::WheelSpeedFL + w] * st.rollK[w] / speed - 1.0;
                    lock = std::max(0.0, std::min(1.0, (-r - kLonOnsetRatio) / (peak - kLonOnsetRatio)));
                    spin = std::max(0.0, std::min(1.0, ( r - kLonOnsetRatio) / (peak - kLonOnsetRatio)));
                }
                else if (!haveRatio && !haveWs && live && v.have[NcxValues::Lockup])
                    lock = single;
                lock = std::min(1.0, lock * loadW[w]);
                spin = std::min(1.0, spin * loadW[w]);
                if (preview) { lock = 1.0; spin = 1.0; }
                L.driveSlip(FxType::Lockup, w, lock, spin);
            }
            // Lock judder beats with road speed; spin tramp is a resonance.
            const double hzScale = (speed > 0.0)
                ? std::max(kLockHzMin, std::min(kLockHzMax, speed / kLockHzRefKmh)) : 1.0;
            L.setSlipCarrierScale(FxType::Lockup, hzScale, 1.0);
        }
        // Preview timers for the slip slots decay here (previewOr does it
        // for the oscillator slots).
        for (FxType t : { FxType::Skid, FxType::Lockup })
        {
            double& left = st.previewSec[static_cast<int>(t)];
            if (left > 0.0) left -= dtSec;
        }
    }

    // Magnitude-driven textures (0-100 on the wire).
    L.driveFx(FxType::Road,   previewOr(FxType::Road,   mag(NcxValues::RoadNoise)), 0.0);
    L.driveFx(FxType::Kerb,   previewOr(FxType::Kerb,   mag(NcxValues::Curbs)),     0.0);

    // Flag-driven pulses (0/1 on the wire).
    L.driveFx(FxType::Limiter, previewOr(FxType::Limiter, flag(NcxValues::Limiter)),  0.0);
    L.driveFx(FxType::TcPulse, previewOr(FxType::TcPulse, flag(NcxValues::TcActive)), 0.0);

    // Gear-shift thunk: a transient on every gear-channel CHANGE (up or
    // down). Edge state seeds on first sight and clears on staleness, so a
    // returning stream never fires a stale crossing.
    if (live && v.have[NcxValues::Gear])
    {
        const double g0 = v.val[NcxValues::Gear];
        const int g = static_cast<int>(g0 < 0.0 ? g0 - 0.5 : g0 + 0.5);
        if (st.gearSeen && g != st.lastGear)
            L.fire(EventType::GearShift, 1.0);
        st.lastGear = g;
        st.gearSeen = true;
    }
    else
    {
        st.gearSeen = false;
    }
}

} // namespace haptics
