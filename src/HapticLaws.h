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
#include <cstdlib>

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
    // Driveline shunt: the throttle a moment ago, for the zero-torque
    // crossing; and the dog box's second knock after a shift under power.
    double lastThrottle = 0.0;
    bool   throttleSeen = false;
    double lowThrottleAgo = 1e9, highThrottleAgo = 1e9;
    double dogKnockIn = 0.0;   // seconds until the dog engagement knock (0 = none pending)
};

namespace laws_k {
    constexpr double kEngineAliveRpm   = 30.0;     // anything turning: cranking lumps from the starter up
    constexpr double kOverrunThrottle  = 10.0;     // % throttle at or below which the throttle is shut
    constexpr double kOverrunMinKmh    = 10.0;     // in gear and rolling: the wheels drive the engine
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
    // Driveline laws
    constexpr double kClutchSlipLo     = 15.0;     // % pedal: below this the clutch is up (driving)
    constexpr double kClutchSlipHi     = 85.0;     // % pedal: above this it is floored (open)
    constexpr double kClutchLaunchKmh  = 5.0;      // slower than this with the engine up = a launch (full slip)
    constexpr double kClutchMinRpm     = 500.0;    // nothing from a stalled engine
    constexpr double kLugThrottle      = 50.0;     // % throttle at or above which the engine can be lugged
    constexpr double kLugFromX         = 0.30;     // lug fades out by this fraction of the redline...
    constexpr double kLugFullX         = 0.12;     // ...and is full at this one (just above idle)
    constexpr double kLugMinRpm        = 600.0;    // not while cranking or dying
    // Whine: pitch from the input shaft at an order the actuator carries,
    // stepping per gear (a lower gear meshes slower per engine rev).
    constexpr double kWhineBandTopHz   = 120.0;    // auto order aims the redline here, like the engine buzz
    constexpr double kWhineSamplesPerCycle = 8.0;
    constexpr double kWhineGearStep    = 0.06;     // pitch x (1 - step x (6 - gear)): 1st lowest, 6th highest
    constexpr double kWhineLoadFloor   = 0.3;      // straight-cut gears whine unloaded too, louder under load
    constexpr double kWhineMinRpm      = 500.0;
    // Shunt: the throttle crossing its zero-torque point fast.
    constexpr double kShuntLowPct      = 5.0;
    constexpr double kShuntHighPct     = 20.0;
    constexpr double kShuntWindowSec   = 0.15;     // the crossing has to happen within this
    constexpr double kShuntMinKmh      = 3.0;      // rolling, in gear
    constexpr double kShuntDogGain     = 1.4;      // a dog box has more lash to take up
    // Gearbox-shaped shift: synchro = softer, longer clunk; dog = harder,
    // shorter knock, and under power a second knock when the dogs engage.
    constexpr double kSynchroScale     = 0.75, kSynchroDur = 1.4, kSynchroFreq = 0.8;
    constexpr double kDogScale         = 1.0,  kDogDur     = 0.7, kDogFreq     = 1.3;
    constexpr double kDogPowerThrottle = 60.0;     // % throttle: a shift above this gets the engagement knock
    constexpr double kDogKnockDelaySec = 0.06;
    constexpr double kDogKnockScale    = 0.7;
}

// Start a Test preview on a continuous/engine slot.
inline void startPreview(LawsState& st, FxType t)
{
    st.previewSec[static_cast<int>(t)] = laws_k::kPreviewSec;
}

// Drive every continuous effect and the gear-shift transient for THIS
// cycle from the channel values. Call before Layer::step().
inline void driveLaws(Layer& L, LawsState& st, const NcxValues& v, double dtSec,
                      const GearRatios* ratios = nullptr)
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
    // A 0..100 magnitude channel as severity: full at the effect's peak %
    // (100 = the channel's own full scale), so a property that only ever
    // reaches 25 can still drive a full slide.
    const auto mag = [&](NcxValues::Token t, FxType fx) -> double
    {
        if (!live || !v.have[t]) return 0.0;
        const double peak = std::max(1.0, L.fxParams(fx).peakPct);
        return std::max(0.0, std::min(1.0, v.val[t] / peak));
    };
    const auto flag = [&](NcxValues::Token t) -> double
    { return (live && v.have[t] && v.val[t] > 0.5) ? 1.0 : 0.0; };

    // Engine: firing rate from rpm and the engine description; throttle is
    // the load; the limiter flag drives the cut gate. A preview with no sim
    // runs a canned idle.
    {
        const EngineParams& e = L.engineParams();
        const double cyl = std::max(1.0, e.cylinders);
        const int    lay = static_cast<int>(e.layout + 0.5);
        // Firings per rev: four-stroke cyl/2; Wankel one per rotor; two-stroke
        // every cylinder; electric none, so the motor rpm goes through as is.
        const double perRev = (lay == 3 || lay == 4) ? cyl : (lay == 5) ? 1.0 : cyl / 2.0;
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
        // Boost in bar when the sim sends it (vacuum reads as 0 boost);
        // negative = unknown, the model falls back to rpm for the lift.
        const double boost = (live && v.have[NcxValues::Boost]) ? std::max(0.0, v.val[NcxValues::Boost]) : -1.0;
        // Pit limiter cuts like the rev limiter but must not teach the redline.
        const bool pitLim = flag(NcxValues::PitLimiter) > 0.5;
        // Overrun in gear: throttle shut, a forward or reverse gear selected,
        // rolling: the wheels drive the engine (heavier than a neutral coast).
        bool inGearOverrun = false;
        if (live && v.have[NcxValues::Gear] && v.have[NcxValues::SpeedKmh])
        {
            const double g = v.val[NcxValues::Gear];
            const bool throttleShut = !v.have[NcxValues::ThrottlePct] || v.val[NcxValues::ThrottlePct] <= kOverrunThrottle;
            inGearOverrun = throttleShut && (g >= 0.5 || g <= -0.5) && v.val[NcxValues::SpeedKmh] > kOverrunMinKmh;
        }
        L.driveEngine(lv, fireHz, load, lim, boost, pitLim, inGearOverrun);
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
        // Slip means nothing at a standstill (a stationary car reports
        // slip angles and skid magnitudes that are noise or stale).
        const bool moving = speed > kSlipMinSpeedKmh;

        // Lateral: slip angle (deg) past the onset share of peak deg.
        {
            const SlipParams& sp = L.slipParams(FxType::Skid);
            const double peak  = std::max(0.5, sp.peak);
            const double onset = kSlipOnsetFrac * peak;
            const bool preview = st.previewSec[static_cast<int>(FxType::Skid)] > 0.0;
            const bool perWheel = haveW(NcxValues::SlipAngleFL);
            const double single = mag(NcxValues::Skid, FxType::Skid);
            for (int w = 0; w < WHEEL_COUNT; ++w)
            {
                double sev = 0.0;
                if (moving && perWheel)
                {
                    const double ang = std::fabs(v.val[NcxValues::SlipAngleFL + w]);
                    sev = std::max(0.0, std::min(1.0, (ang - onset) / std::max(1e-6, peak - onset)));
                }
                else if (moving && !perWheel && live && v.have[NcxValues::Skid])
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
            const double single  = mag(NcxValues::Lockup, FxType::Lockup);

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
                else if (moving && !haveRatio && !haveWs && live && v.have[NcxValues::Lockup])
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

    // Road: replay the corners when the sim sends suspension velocities
    // (mm/s, all four); the roadNoise texture otherwise. A preview runs
    // the texture so Test shows something without a sim.
    {
        bool corners = live && st.previewSec[static_cast<int>(FxType::Road)] <= 0.0;
        for (int w = 0; w < WHEEL_COUNT && corners; ++w) corners = v.have[NcxValues::SuspVelFL + w];
        if (corners)
        {
            for (int w = 0; w < WHEEL_COUNT; ++w) L.driveRoad(w, v.val[NcxValues::SuspVelFL + w]);
            L.driveFx(FxType::Road, 0.0, 0.0);
        }
        else
            L.driveFx(FxType::Road, previewOr(FxType::Road, mag(NcxValues::RoadNoise, FxType::Road)), 0.0);
    }

    // Driveline: clutch judder while the pedal is in the slipping band with
    // slip across the clutch (engine rpm against what the gear and road
    // speed say, from the learned ratios; a launch from rest is full slip;
    // an unknown ratio counts as half), scaled by load; lugging wind-up at
    // high throttle and low revs. Nothing in neutral. A preview plays both.
    {
        double judder = 0.0, lug = 0.0;
        const bool preview = st.previewSec[static_cast<int>(FxType::Driveline)] > 0.0;
        if (preview) { judder = 1.0; lug = 1.0; st.previewSec[static_cast<int>(FxType::Driveline)] -= dtSec; }
        else if (live && v.have[NcxValues::Rpm] && v.have[NcxValues::Gear] && v.val[NcxValues::Rpm] > kClutchMinRpm)
        {
            const double rpm  = v.val[NcxValues::Rpm];
            const double g0   = v.val[NcxValues::Gear];
            const int    g    = static_cast<int>(g0 < 0.0 ? g0 - 0.5 : g0 + 0.5);
            const bool   inGear = (g != 0);
            const double load = v.have[NcxValues::ThrottlePct] ? std::max(0.0, std::min(1.0, v.val[NcxValues::ThrottlePct] / 100.0)) : 0.5;
            if (inGear && v.have[NcxValues::ClutchPct])
            {
                const double c = v.val[NcxValues::ClutchPct];
                if (c > kClutchSlipLo && c < kClutchSlipHi)
                {
                    // Window: full mid-band, tapering to the edges.
                    const double mid = 0.5 * (kClutchSlipLo + kClutchSlipHi), half = 0.5 * (kClutchSlipHi - kClutchSlipLo);
                    const double window = std::max(0.0, 1.0 - std::fabs(c - mid) / half);
                    double slip = 0.5;   // ratio unknown: assume some
                    const double speed = v.have[NcxValues::SpeedKmh] ? std::max(0.0, v.val[NcxValues::SpeedKmh]) : -1.0;
                    if (speed >= 0.0 && speed < kClutchLaunchKmh) slip = 1.0;
                    else if (speed >= 0.0 && ratios && g > 0 && g < MAX_GEARS && ratios->known[g])
                    {
                        const double expected = ratios->r[g] * speed;
                        slip = std::max(0.0, std::min(1.0, std::fabs(rpm - expected) / std::max(1.0, rpm)));
                    }
                    judder = window * slip * std::max(0.15, load);
                }
            }
            if (inGear && v.have[NcxValues::ThrottlePct] && v.val[NcxValues::ThrottlePct] >= kLugThrottle && rpm > kLugMinRpm)
            {
                const double maxRpm = (L.engineParams().maxRpm > 0.0) ? L.engineParams().maxRpm : L.learnedMaxRpm();
                const double x = rpm / std::max(1000.0, maxRpm);
                lug = std::max(0.0, std::min(1.0, (kLugFromX - x) / (kLugFromX - kLugFullX))) * load;
            }
        }
        L.driveDriveline(judder, lug);

        // Whine: straight-cut mesh following the input shaft, in gear with
        // the clutch driving; level from load with a floor (unloaded gears
        // still sing). The carrier steps per gear.
        {
            double wl = 0.0, whz = 0.0;
            const DrivelineParams& dp = L.drivelineParams();
            if (preview) { wl = 1.0; whz = 60.0; }
            else if (dp.whine > 0.0 && live && v.have[NcxValues::Rpm] && v.have[NcxValues::Gear]
                     && v.val[NcxValues::Rpm] > kWhineMinRpm)
            {
                const double g0 = v.val[NcxValues::Gear];
                const int    g  = static_cast<int>(g0 < 0.0 ? g0 - 0.5 : g0 + 0.5);
                const bool clutchOpen = v.have[NcxValues::ClutchPct] && v.val[NcxValues::ClutchPct] > kClutchSlipHi;
                if (g != 0 && !clutchOpen)
                {
                    const double maxRpm = (L.engineParams().maxRpm > 0.0) ? L.engineParams().maxRpm : L.learnedMaxRpm();
                    const double bandTop = std::min(kWhineBandTopHz, (1.0 / std::max(1e-4, dtSec)) / kWhineSamplesPerCycle);
                    const double order = bandTop / std::max(1000.0, maxRpm) * 60.0;
                    const double gearK = 1.0 - kWhineGearStep * std::max(0.0, 6.0 - std::min(6.0, static_cast<double>(std::abs(g))));
                    whz = v.val[NcxValues::Rpm] / 60.0 * order * gearK;
                    const double load = v.have[NcxValues::ThrottlePct] ? std::max(0.0, std::min(1.0, v.val[NcxValues::ThrottlePct] / 100.0)) : 0.5;
                    wl = kWhineLoadFloor + (1.0 - kWhineLoadFloor) * load;
                }
            }
            L.driveDrivelineWhine(wl, whz);
        }

        // Shunt: the throttle crossing zero torque fast, in gear and rolling:
        // backlash takes up with a knock, harder in a dog box.
        {
            const DrivelineParams& dp = L.drivelineParams();
            const bool dog = dp.gearbox >= 0.5;
            if (live && v.have[NcxValues::ThrottlePct])
            {
                const double thr = v.val[NcxValues::ThrottlePct];
                if (thr <= kShuntLowPct)  st.lowThrottleAgo  = 0.0; else st.lowThrottleAgo  += dtSec;
                if (thr >= kShuntHighPct) st.highThrottleAgo = 0.0; else st.highThrottleAgo += dtSec;
                const bool inGearRolling = v.have[NcxValues::Gear] && std::fabs(v.val[NcxValues::Gear]) >= 0.5
                                           && (!v.have[NcxValues::SpeedKmh] || v.val[NcxValues::SpeedKmh] > kShuntMinKmh);
                if (st.throttleSeen && inGearRolling && dp.shunt > 0.0)
                {
                    // Tip-in: was at or below low within the window, now at or above high.
                    if (thr >= kShuntHighPct && st.lastThrottle < kShuntHighPct && st.lowThrottleAgo < kShuntWindowSec && st.lowThrottleAgo > 0.0)
                    { L.drivelineShunt(std::min(1.0, (dog ? kShuntDogGain : 1.0))); st.lowThrottleAgo = 1e9; }
                    // Lift: was at or above high within the window, now at or below low.
                    if (thr <= kShuntLowPct && st.lastThrottle > kShuntLowPct && st.highThrottleAgo < kShuntWindowSec && st.highThrottleAgo > 0.0)
                    { L.drivelineShunt(std::min(1.0, 0.8 * (dog ? kShuntDogGain : 1.0))); st.highThrottleAgo = 1e9; }
                }
                st.lastThrottle = thr; st.throttleSeen = true;
            }
            else st.throttleSeen = false;
            if (preview && st.dogKnockIn <= 0.0 && dp.shunt > 0.0 && st.previewSec[static_cast<int>(FxType::Driveline)] > kPreviewSec - dtSec * 1.5)
                L.drivelineShunt(1.0);   // one knock at the start of a preview
        }
    }

    // Magnitude-driven textures (0-100 on the wire).
    L.driveFx(FxType::Kerb,   previewOr(FxType::Kerb,   mag(NcxValues::Curbs, FxType::Kerb)), 0.0);

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
        {
            // Shaped by the gearbox: synchro = a softer, longer clunk; dog =
            // a harder, shorter knock and, shifted under power, a second
            // knock a moment later when the dogs engage.
            const bool dog = L.drivelineParams().gearbox >= 0.5;
            if (dog) L.fire(EventType::GearShift, kDogScale, kDogDur, kDogFreq);
            else     L.fire(EventType::GearShift, kSynchroScale, kSynchroDur, kSynchroFreq);
            const double thr = v.have[NcxValues::ThrottlePct] ? v.val[NcxValues::ThrottlePct] : 0.0;
            if (dog && thr >= kDogPowerThrottle) st.dogKnockIn = kDogKnockDelaySec;
        }
        st.lastGear = g;
        st.gearSeen = true;
    }
    else
    {
        st.gearSeen = false;
    }
    if (st.dogKnockIn > 0.0)
    {
        st.dogKnockIn -= dtSec;
        if (st.dogKnockIn <= 0.0) { L.fire(EventType::GearShift, kDogKnockScale, kDogDur, kDogFreq); st.dogKnockIn = 0.0; }
    }
}

} // namespace haptics
