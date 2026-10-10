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
    // Kerb detection from road heights: each axle's usual left-minus-right
    // (camber, banking), learned off the kerbs, so a tyre stepping up shows.
    double kerbBias[2] = {};
    bool   kerbBiasSeen = false;
    // Lateral slip: the per-wheel slip angle (or combined slip) smoothed,
    // and which source it is (0 none, 1 angle, 2 combined) so a change of
    // source starts the smoothing afresh.
    double slipF[WHEEL_COUNT] = {};
    int    slipSrc = 0;
    // The water standing on the road, built up from the rain where the sim
    // sends rain but no wetness (0 dry .. 1 standing water).
    double waterFilm = 0.0;
    // How long the stream has been quiet (fresh tyres for the Wheels tile
    // once it has been quiet a while).
    double quietSec = 0.0;
};

namespace laws_k {
    constexpr double kEngineAliveRpm   = 30.0;     // anything turning: cranking lumps from the starter up
    constexpr double kOverrunThrottle  = 10.0;     // % throttle at or below which the throttle is shut
    constexpr double kOverrunMinKmh    = 10.0;     // in gear and rolling: the wheels drive the engine
    constexpr double kPreviewIdleRpm   = 1100.0;   // the canned idle a Test preview runs
    constexpr double kAbsMinBrakePct   = 10.0;     // ABS needs the brake actually applied
    constexpr double kPreviewSec       = 2.0;      // Test preview length
    // Slip laws (the lateral onset, curve and smoothing are the tile's:
    // SlipParams onsetPct, ease, smoothHz)
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
    // Road and kerb laws
    constexpr double kPreviewKmh       = 80.0;     // the road speed a road, kerb or ABS Test preview runs at
    constexpr double kPreviewKerbOnSec = 0.6;      // kerb preview: on this long...
    constexpr double kPreviewKerbCycle = 1.0;      // ...in every this long
    constexpr double kKerbMinKmh       = 10.0;     // height detection: rolling
    constexpr double kKerbBiasSec      = 2.0;      // how fast the axle's usual left-right learns
    // Surface laws: the water film from rain.
    constexpr double kFilmFillSec      = 60.0;     // heavy rain stands on the road in about a minute
    constexpr double kFilmDrySec       = 240.0;    // ...and takes a few to dry off
    constexpr double kPreviewPhaseSec  = 0.7;      // a Surface Test: gravel, snow, then a wet road (Wheels: balance, flat, judder)
    // Wheels laws
    constexpr double kFreshTyresSec    = 30.0;     // the stream quiet this long: fresh tyres, cold discs
    constexpr double kPreviewWheelSpread[WHEEL_COUNT] = { 0.0, 0.004, -0.003, 0.007 };   // a Test: each wheel a touch apart
    constexpr double kPreviewFlatMm    = 0.8;
    constexpr double kPreviewBrake     = 0.6;
    // Impacts laws: a Test preview's knocks (wheel -1 = every corner).
    struct PreviewKnock { double atSec; int wheel; double severity, sign; };
    constexpr PreviewKnock kPreviewKnocks[] = {
        { 0.10, WheelFL, 1.0, 1.0 }, { 0.50, WheelFR, 0.6, 1.0 }, { 0.90, WheelRL, 1.0, -1.0 },
        { 1.30, WheelRR, 0.6, 1.0 }, { 1.70, -1, 1.0, 1.0 } };
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
        // Traction control cutting: the engine drops a share of its firings
        // (the TC tile adds the body surge on top).
        const bool tcCut = flag(NcxValues::TcActive) > 0.5;
        L.driveEngine(lv, fireHz, load, lim, boost, pitLim, inGearOverrun, tcCut);
    }

    // ABS: only while the sim says ABS is cycling AND the brake is applied
    // (some sims flicker the flag at zero brake).
    // The road speed slows the cycle towards a stop (unknown: full rate; a
    // Test preview runs at speed).
    {
        const bool on = flag(NcxValues::AbsActive) > 0.5
                        && live && v.have[NcxValues::BrakePct] && v.val[NcxValues::BrakePct] > kAbsMinBrakePct;
        const bool previewing = st.previewSec[static_cast<int>(FxType::AbsPulse)] > 0.0;
        L.driveAbsSpeed(previewing ? kPreviewKmh
                        : (live && v.have[NcxValues::SpeedKmh]) ? std::max(0.0, v.val[NcxValues::SpeedKmh]) : -1.0);
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
        // Each wheel's signed slip ratio where one is known this cycle (from
        // the sim or the wheel speeds), for the grip budget below.
        double ratioW[WHEEL_COUNT] = {};
        bool   ratioOk[WHEEL_COUNT] = {};
        double lonPeak = kLonOnsetRatio + 0.05;

        // Longitudinal: slip ratio (- locking, + spinning) past the onset
        // ratio up to peak ratio. Ratio from the sim, else from wheel speeds
        // with the learned rolling factor, else the lockup channel.
        {
            const SlipParams& sp = L.slipParams(FxType::Lockup);
            const double peak = std::max(kLonOnsetRatio + 0.05, sp.peak);
            lonPeak = peak;
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
                    ratioW[w] = r; ratioOk[w] = true;
                }
                else if (moving && haveWs && st.rollKnown[w])
                {
                    const double r = v.val[NcxValues::WheelSpeedFL + w] * st.rollK[w] / speed - 1.0;
                    lock = std::max(0.0, std::min(1.0, (-r - kLonOnsetRatio) / (peak - kLonOnsetRatio)));
                    spin = std::max(0.0, std::min(1.0, ( r - kLonOnsetRatio) / (peak - kLonOnsetRatio)));
                    ratioW[w] = r; ratioOk[w] = true;
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

        // Lateral: slip angle (deg) from onset % of peak deg up to peak deg,
        // shaped by ease (0 linear; 1 = severity squared, a light scrub
        // first; 2 cubed). Where the sim has no slip angle but a combined
        // slip per wheel (Automobilista 2's slip speed), that, from onset %
        // of peak % up to full at peak %. A combined slip cannot be split
        // into sideways and along, and a spinning or locked tyre is sliding
        // too: it plays here as well as on the longitudinal tile (taking
        // the longitudinal share out cancelled power oversteer on the
        // bench). Either is smoothed at smooth hz first: it arrives ~60
        // times a second, and a slip angle worked out from the tyres'
        // positions (Assetto Corsa) jitters, which near the onset made the
        // slide flicker in and out. Else the single skid channel.
        {
            const SlipParams& sp = L.slipParams(FxType::Skid);
            const double peak  = std::max(0.5, sp.peak);
            const double onsetFrac = std::max(0.0, std::min(0.9, sp.onsetPct / 100.0));
            const double onset = onsetFrac * peak;
            const double ease  = std::max(0.0, std::min(2.0, sp.ease));
            const bool preview = st.previewSec[static_cast<int>(FxType::Skid)] > 0.0;
            const bool perWheel = haveW(NcxValues::SlipAngleFL);
            const bool combined = !perWheel && haveW(NcxValues::WheelSlipFL);
            const double peakC  = std::max(1.0, L.fxParams(FxType::Skid).peakPct);
            const double onsetC = onsetFrac * peakC;
            const double single = mag(NcxValues::Skid, FxType::Skid);
            const int src = perWheel ? 1 : (combined ? 2 : 0);
            const int base = (src == 1) ? NcxValues::SlipAngleFL : NcxValues::WheelSlipFL;
            const double smA = (sp.smoothHz > 0.0) ? 1.0 - std::exp(-2.0 * wavesynth::kPi * sp.smoothHz * dtSec) : 1.0;
            if (src != st.slipSrc)
            {
                for (int w = 0; w < WHEEL_COUNT; ++w)
                    st.slipF[w] = (src != 0) ? std::fabs(v.val[base + w]) : 0.0;
                st.slipSrc = src;
            }
            const auto curve = [ease](double lin) { return std::pow(std::max(0.0, std::min(1.0, lin)), 1.0 + ease); };
            for (int w = 0; w < WHEEL_COUNT; ++w)
            {
                if (src != 0) st.slipF[w] += (std::fabs(v.val[base + w]) - st.slipF[w]) * smA;
                double sev = 0.0;
                if (moving && perWheel)
                    sev = curve((st.slipF[w] - onset) / std::max(1e-6, peak - onset));
                else if (moving && combined)
                    sev = curve((st.slipF[w] - onsetC) / std::max(1e-6, peakC - onsetC));
                else if (moving && !perWheel && !combined && live && v.have[NcxValues::Skid])
                    sev = single;
                sev = std::min(1.0, sev * loadW[w]);
                if (preview) sev = 1.0;
                const bool front = (w == WheelFL || w == WheelFR);
                L.driveSlip(FxType::Skid, w, front ? sev : 0.0, front ? 0.0 : sev);

                // Grip budget: a wheel with both a slip angle and a slip
                // ratio slides when the two TOGETHER reach its limit. Each
                // is measured against its own grip limit (the onset angle;
                // the 0.15 ratio), the two combine as a circle (1 = at the
                // limit), and the slide grows from there to a full slide
                // (blended between the two full points by direction), then
                // plays on each tile by its direction (sideways: Lateral;
                // along: Longitudinal, lock or spin by sign). Pure sideways
                // or pure along gives the same severity as the tiles alone.
                if (sp.budget >= 0.5 && moving && perWheel && ratioOk[w] && !preview
                    && st.previewSec[static_cast<int>(FxType::Lockup)] <= 0.0)
                {
                    const double onsetA = std::max(0.05 * peak, onset);
                    const double aN = st.slipF[w] / onsetA;
                    const double kN = std::fabs(ratioW[w]) / kLonOnsetRatio;
                    const double u  = std::sqrt(aN * aN + kN * kN);
                    double lat = 0.0, lon = 0.0;
                    if (u > 1e-9)
                    {
                        const double wa = aN * aN / (u * u), wk = kN * kN / (u * u);
                        const double uFull = wa * (peak / onsetA) + wk * (lonPeak / kLonOnsetRatio);
                        const double s = curve((u - 1.0) / std::max(1e-6, uFull - 1.0));
                        lat = std::min(1.0, s * (aN / u) * loadW[w]);
                        lon = std::min(1.0, s * (kN / u) * loadW[w]);
                    }
                    L.driveSlip(FxType::Skid, w, front ? lat : 0.0, front ? 0.0 : lat);
                    L.driveSlip(FxType::Lockup, w, ratioW[w] < 0.0 ? lon : 0.0, ratioW[w] > 0.0 ? lon : 0.0);
                }
            }
        }
        // Preview timers for the slip slots decay here (previewOr does it
        // for the oscillator slots).
        for (FxType t : { FxType::Skid, FxType::Lockup })
        {
            double& left = st.previewSec[static_cast<int>(t)];
            if (left > 0.0) left -= dtSec;
        }

        // Wheels: each wheel turns at the car's speed x (1 + its slip ratio)
        // where the ratio is known (from the sim or the wheel speeds, above),
        // else at the car's speed; the ratio and the load share flat-spot a
        // locked wheel; the brake heats the discs. A stream gone quiet for a
        // while (a new session, a new car) brings fresh tyres. A Test preview
        // runs at 80 km/h, the wheels a touch apart: out of round, then a flat
        // on the front left, then hot discs under braking.
        {
            double& previewLeft = st.previewSec[static_cast<int>(FxType::Wheels)];
            if (live) st.quietSec = 0.0;
            else if (st.quietSec < kFreshTyresSec && (st.quietSec += dtSec) >= kFreshTyresSec) L.wheelsFreshTyres();
            if (previewLeft > 0.0)
            {
                const double t = kPreviewSec - previewLeft;
                for (int w = 0; w < WHEEL_COUNT; ++w)
                {
                    L.driveWheelSpeed(w, kPreviewKmh / 3.6 * (1.0 + kPreviewWheelSpread[w]));
                    L.driveWheelLock(w, 0.0, 0.0, 1.0);   // the preview's heat is its own: the discs stay as they are
                }
                const bool flat = t >= kPreviewPhaseSec, judder = t >= 2.0 * kPreviewPhaseSec;
                L.driveWheelsBrake(judder ? kPreviewBrake : 0.0);
                L.driveWheelsPreview(flat ? kPreviewFlatMm : 0.0, judder ? 1.0 : 0.0);
                previewLeft -= dtSec;
            }
            else if (live && v.have[NcxValues::SpeedKmh])
            {
                const double carMs = speed / 3.6;
                for (int w = 0; w < WHEEL_COUNT; ++w)
                {
                    L.driveWheelSpeed(w, ratioOk[w] ? carMs * std::max(0.0, 1.0 + ratioW[w]) : carMs);
                    L.driveWheelLock(w, carMs, ratioOk[w] ? ratioW[w] : 0.0, loadW[w]);
                }
                L.driveWheelsBrake(v.have[NcxValues::BrakePct] ? v.val[NcxValues::BrakePct] / 100.0 : 0.0);
            }
        }
    }

    // Road: everything the models can use goes in and the tile's model
    // picks what it plays. Per corner: suspension velocity (mm/s) or travel
    // (mm), the road height under the tyre (mm), the surface class there;
    // the car: speed, body heave, pitch and roll, wheelbase and track. When
    // the chosen model has nothing to run on, the roadNoise texture. The
    // surface grain (suspension model) rides on top, rising with road speed,
    // full by surface km/h, at the surface mix. A Test preview runs the tyre
    // model at 80 km/h on its own road, the other models the texture.
    {
        double& previewLeft = st.previewSec[static_cast<int>(FxType::Road)];
        const bool previewing = previewLeft > 0.0;
        const RoadParams& rp = L.roadParams();
        const int model = static_cast<int>(rp.model + 0.5);
        const bool feed = live && !previewing;
        const auto haveW = [&](int group) -> bool
        {
            if (!feed) return false;
            for (int w = 0; w < WHEEL_COUNT; ++w) if (!v.have[group + w]) return false;
            return true;
        };
        const bool vel    = haveW(NcxValues::SuspVelFL);
        const bool travel = !vel && haveW(NcxValues::SuspTravelFL);
        const bool height = haveW(NcxValues::RoadHeightFL);
        const bool surf   = haveW(NcxValues::SurfaceFL);
        const bool body   = feed && v.have[NcxValues::AccHeave];
        const bool speed  = feed && v.have[NcxValues::SpeedKmh];
        if (vel)         for (int w = 0; w < WHEEL_COUNT; ++w) L.driveRoad(w, v.val[NcxValues::SuspVelFL + w]);
        else if (travel) for (int w = 0; w < WHEEL_COUNT; ++w) L.driveRoadTravel(w, v.val[NcxValues::SuspTravelFL + w]);
        if (height)      for (int w = 0; w < WHEEL_COUNT; ++w) L.driveRoadHeight(w, v.val[NcxValues::RoadHeightFL + w]);
        for (int w = 0; w < WHEEL_COUNT; ++w)
            L.driveRoadSurface(w, surf ? static_cast<int>(v.val[NcxValues::SurfaceFL + w] + 0.5) : SurfTarmac);
        if (body)
            L.driveChassis(v.val[NcxValues::AccHeave],
                           v.have[NcxValues::PitchDeg] ? v.val[NcxValues::PitchDeg] : 0.0,
                           v.have[NcxValues::RollDeg]  ? v.val[NcxValues::RollDeg]  : 0.0);
        if (live && (v.have[NcxValues::Wheelbase] || v.have[NcxValues::TrackWidth]))
            L.driveGeometry(v.have[NcxValues::Wheelbase]  ? v.val[NcxValues::Wheelbase]  : 0.0,
                            v.have[NcxValues::TrackWidth] ? v.val[NcxValues::TrackWidth] : 0.0);
        if (speed)                         L.driveRoadSpeed(std::max(0.0, v.val[NcxValues::SpeedKmh]) / 3.6);
        else if (previewing && model == 1) L.driveRoadSpeed(kPreviewKmh / 3.6);
        // The tyre model runs on speed alone (its own road); the chassis
        // model needs the body; the suspension model the corners.
        const bool fed = (model == 1) ? (speed || height || vel || travel || previewing)
                       : (model == 2) ? body
                       : (vel || travel);
        double texture = fed ? 0.0 : mag(NcxValues::RoadNoise, FxType::Road);
        if (previewing) { previewLeft -= dtSec; if (model != 1) texture = 1.0; }
        L.driveFx(FxType::Road, texture, 0.0);
        double grain = 0.0;
        if (speed && rp.surface > 0.0)
        {
            const double spd = std::max(0.0, v.val[NcxValues::SpeedKmh]);
            const double full = std::max(1.0, rp.surfaceKmh);
            grain = rp.surface * std::min(1.0, spd / full);
        }
        L.driveRoadGrain(grain);
    }

    // Kerb: how much of each tyre is on a kerb. The sim's surface class
    // under each tyre where it sends one; else its curbs channel (all
    // wheels, full at peak %); else, with detect mm set and the road height
    // under each tyre, a tyre stepped up above its axle mate by detect mm
    // more than usual for that axle (camber and banking learned off the
    // kerbs). A Test preview runs the strip at 80 km/h, on and off.
    {
        double& previewLeft = st.previewSec[static_cast<int>(FxType::Kerb)];
        const KerbParams& kp = L.kerbParams();
        const auto haveW = [&](int group) -> bool
        {
            if (!live) return false;
            for (int w = 0; w < WHEEL_COUNT; ++w) if (!v.have[group + w]) return false;
            return true;
        };
        const double kmh = (live && v.have[NcxValues::SpeedKmh]) ? std::max(0.0, v.val[NcxValues::SpeedKmh]) : 0.0;
        double lvl[WHEEL_COUNT] = {};
        bool heights = false;
        if (previewLeft > 0.0)
        {
            const bool on = std::fmod(kPreviewSec - previewLeft, kPreviewKerbCycle) < kPreviewKerbOnSec;
            for (double& l : lvl) l = on ? 1.0 : 0.0;
            previewLeft -= dtSec;
            L.driveKerbSpeed(kPreviewKmh / 3.6);   // after the road law's speed, so it wins
        }
        else if (haveW(NcxValues::SurfaceFL))
        {
            for (int w = 0; w < WHEEL_COUNT; ++w)
                lvl[w] = (static_cast<int>(v.val[NcxValues::SurfaceFL + w] + 0.5) == SurfKerb) ? 1.0 : 0.0;
        }
        else if (live && v.have[NcxValues::Curbs])
        {
            const double c = mag(NcxValues::Curbs, FxType::Kerb);
            for (double& l : lvl) l = c;
        }
        else if (kp.detectMm > 0.0 && haveW(NcxValues::RoadHeightFL) && kmh > kKerbMinKmh)
        {
            heights = true;
            const double half = 0.5 * kp.detectMm;
            const double a = std::min(1.0, dtSec / kKerbBiasSec);
            for (int ax = 0; ax < 2; ++ax)
            {
                const int l = ax ? WheelRL : WheelFL, r = ax ? WheelRR : WheelFR;
                const double d = v.val[NcxValues::RoadHeightFL + l] - v.val[NcxValues::RoadHeightFL + r];
                if (!st.kerbBiasSeen) st.kerbBias[ax] = d;
                const double ex = d - st.kerbBias[ax];
                // Learns off the kerbs; on one, a tenth as fast (a bias seeded
                // on a kerb still finds its way back).
                st.kerbBias[ax] += ((std::fabs(ex) < half) ? a : 0.1 * a) * (d - st.kerbBias[ax]);
                lvl[ex > 0.0 ? l : r] = std::max(0.0, std::min(1.0, (std::fabs(ex) - half) / half));
            }
        }
        st.kerbBiasSeen = heights;
        for (int w = 0; w < WHEEL_COUNT; ++w) L.driveKerb(w, lvl[w]);
    }

    // Surface: the water on the road under each tyre: the sim's own per
    // tyre where it sends it, else its track wetness, else a film built up
    // from the rain. The surface class and the speed reach the model with
    // the Road tile's inputs above. A Test preview runs gravel, then snow,
    // then a wet road, at 80 km/h.
    {
        double& previewLeft = st.previewSec[static_cast<int>(FxType::Surface)];
        bool wetW = live;
        for (int w = 0; w < WHEEL_COUNT; ++w) wetW = wetW && v.have[NcxValues::WetFL + w];
        const bool wet  = live && v.have[NcxValues::Wet];
        const bool rain = live && v.have[NcxValues::Rain];
        if (rain)
        {
            const double r = std::max(0.0, std::min(1.0, v.val[NcxValues::Rain]));
            st.waterFilm += dtSec * (r * (1.0 - st.waterFilm) / kFilmFillSec - (1.0 - r) * st.waterFilm / kFilmDrySec);
            st.waterFilm = std::max(0.0, std::min(1.0, st.waterFilm));
        }
        double water[WHEEL_COUNT] = {};
        for (int w = 0; w < WHEEL_COUNT; ++w)
            water[w] = wetW ? v.val[NcxValues::WetFL + w] : wet ? v.val[NcxValues::Wet] : rain ? st.waterFilm : 0.0;
        if (previewLeft > 0.0)
        {
            const double t = kPreviewSec - previewLeft;
            const int cls = (t < kPreviewPhaseSec) ? SurfGravel : (t < 2.0 * kPreviewPhaseSec) ? SurfSnow : SurfTarmac;
            for (int w = 0; w < WHEEL_COUNT; ++w)
            {
                L.driveRoadSurface(w, cls);
                water[w] = (cls == SurfTarmac) ? 1.0 : 0.0;
            }
            L.driveSurfaceSpeed(kPreviewKmh / 3.6);
            previewLeft -= dtSec;
        }
        for (int w = 0; w < WHEEL_COUNT; ++w) L.driveWet(w, water[w]);
    }

    // Impacts: each corner's suspension velocity (or its travel, the
    // velocity worked out per sample) and the body's vertical acceleration;
    // the model finds the sudden jumps. A Test preview knocks FL, FR, RL
    // (a top-out), RR, then the whole car.
    {
        double& previewLeft = st.previewSec[static_cast<int>(FxType::Impacts)];
        if (previewLeft > 0.0)
        {
            const double t0 = kPreviewSec - previewLeft, t1 = t0 + dtSec;
            for (const PreviewKnock& k : kPreviewKnocks)
                if (k.atSec >= t0 && k.atSec < t1)
                {
                    if (k.wheel < 0) for (int w = 0; w < WHEEL_COUNT; ++w) L.impactKnock(w, k.severity, k.sign);
                    else L.impactKnock(k.wheel, k.severity, k.sign);
                }
            previewLeft -= dtSec;
        }
        else if (live)
        {
            bool vel = true, travel = true;
            for (int w = 0; w < WHEEL_COUNT; ++w)
            {
                vel    = vel    && v.have[NcxValues::SuspVelFL + w];
                travel = travel && v.have[NcxValues::SuspTravelFL + w];
            }
            if (vel)         for (int w = 0; w < WHEEL_COUNT; ++w) L.driveImpactVelocity(w, v.val[NcxValues::SuspVelFL + w]);
            else if (travel) for (int w = 0; w < WHEEL_COUNT; ++w) L.driveImpactTravel(w, v.val[NcxValues::SuspTravelFL + w]);
            if (v.have[NcxValues::AccHeave]) L.driveImpactHeave(v.val[NcxValues::AccHeave]);
        }
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
