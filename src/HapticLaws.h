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
#include <cstring>

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
    double rollSeedSec = 0.0;   // how long the wheels have looked fit to learn from
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
    // and which source it is (0 none, 1 angle, 2 combined, 3 an angle from
    // a slip speed) so a change of source starts the smoothing afresh.
    double slipF[WHEEL_COUNT] = {};
    int    slipSrc = 0;
    // The water standing on the road, built up from the rain where the sim
    // sends rain but no wetness (0 dry .. 1 standing water).
    double waterFilm = 0.0;
    // How long the stream has been quiet (fresh tyres for the Wheels tile
    // once it has been quiet a while).
    double quietSec = 0.0;
    // Ground contact: the usual corner load (learned) and which tyres are
    // in the air (with hysteresis).
    double loadRef = 0.0;
    bool   air[WHEEL_COUNT] = {};
    // The car the sim last named: a different one starts the per-car
    // learning over (rolling factor, usual load, kerb bias, the engine's
    // redline, the tyres).
    char   lastCar[NCY_STR_LEN] = {};
    bool   carSeen = false;
    // Road speed glitches (a jump to the pits, a reset): the last speed the
    // sim sent, how long ago it changed, and how long the speed stays
    // untrusted after a jump no car can make.
    double lastSpeedKmh = 0.0, speedAgeSec = 0.0, glitchSec = 0.0;
    bool   speedSeen = false;
    double lastPitch = 0.0, lastRoll = 0.0;   // ...and the body's angles a moment ago
    bool   anglesSeen = false;
    // The sim's ABS and TC flags, held over their short gaps; whether the
    // ABS flag is playing this cycle (for the card).
    double absHoldSec = 0.0, tcHoldSec = 0.0;
    bool   absFlagOn = false;
    // ABS from the wheels: per wheel, armed once it dips towards a lock,
    // its lowest ratio since, the time since its last dump; the hold after
    // a dump that counted, and the rate the dumps come at (for the card).
    bool   absArmed[WHEEL_COUNT] = {};
    double absMinR[WHEEL_COUNT] = {}, absSince[WHEEL_COUNT] = { 1e9, 1e9, 1e9, 1e9 };
    double absWheelHoldSec = 0.0, absWheelHz = 0.0;
    // Lateral from a slip speed: each wheel's sideways angle (deg), its
    // straight-line baseline (learned) and the angle as played (smoothed,
    // less the baseline; also a sim's own slip angle), for the card.
    double latRaw[WHEEL_COUNT] = {}, latBase[WHEEL_COUNT] = {}, latAngle[WHEEL_COUNT] = {};
    double latBaseSec = 0.0;   // straight-line time the baseline has learned over
    int    latBaseMode = 0;    // the tile's baseline mode last cycle (leaving fixed starts the learning over)
};

// The Lateral tile's straight-line baseline starts over: its relearn
// button, and a new car.
inline void relearnLatBase(LawsState& st)
{
    for (int w = 0; w < WHEEL_COUNT; ++w) st.latRaw[w] = st.latBase[w] = st.latAngle[w] = 0.0;
    st.latBaseSec = 0.0;
}

namespace laws_k {
    constexpr double kEngineAliveRpm   = 30.0;     // anything turning: cranking lumps from the starter up
    constexpr double kOverrunThrottle  = 10.0;     // % throttle at or below which the throttle is shut
    constexpr double kOverrunMinKmh    = 10.0;     // in gear and rolling: the wheels drive the engine
    constexpr double kPreviewIdleRpm   = 1100.0;   // the canned idle a Test preview runs
    constexpr double kAbsMinBrakePct   = 10.0;     // ABS needs the brake actually applied
    // ABS from the wheels: a wheel dipping this far towards a lock arms; it
    // spinning back up past this (and this much from its lowest) is a dump,
    // and only then can it arm again (a wheel hovering near a lock, or
    // creeping back over several samples, is no dump or one); two dumps
    // this far apart (a 25 to 3 Hz cycle) are ABS working, held this long
    // after (longer than the slowest cycle).
    constexpr double kAbsDipRatio      = 0.06;
    constexpr double kAbsRecoverLevel  = 0.04;
    constexpr double kAbsRecoverRatio  = 0.03;
    constexpr double kAbsMinPeriodSec  = 0.04, kAbsMaxPeriodSec = 0.35;
    constexpr double kAbsWheelHoldSec  = 0.4;
    // Lateral from a slip speed: the straight-line baseline learns while
    // the fronts turn within this of each other above this speed, off the
    // brake, the tyres short of their edge; up over this long (time
    // constant) while it first learns, learned after this much straight
    // (95 %), then up only slowly (a long fast corner the straight test
    // misses moves it little), down quickly (the straight-line angle is
    // the least the car shows), never past this; the road speed under the
    // angle never below this (m/s).
    constexpr double kLatBaseStraight  = 0.004;
    constexpr double kLatBaseMinKmh    = 60.0;
    constexpr double kLatBaseSec       = 2.0;
    constexpr double kLatBaseLearnSec  = 6.0;
    constexpr double kLatBaseUpSec     = 30.0;
    constexpr double kLatBaseDownSec   = 1.0;
    constexpr double kLatBaseMaxDeg    = 6.0;
    constexpr double kLatMinSpeedMs    = 3.0;
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
    constexpr double kRollLearnSec     = 2.0;      // learns over seconds while cruising...
    constexpr double kRollSeedSec      = 0.5;      // ...the first value only after this long fit to learn from
    constexpr double kRollAxleAgree    = 0.04;     // an axle's two wheels within this of each other (not sliding, not a tight corner)
    constexpr double kRollMaxSlipDeg   = 3.0;      // no tyre past this slip angle, where the sim sends them
    constexpr double kRollMaxStep      = 0.05;     // a sample pulls at most this share off what was learned
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
    // Ground contact from load: in the air below this share of the usual
    // corner load, back on the ground above that one.
    constexpr double kAirEnterShare    = 0.03;
    constexpr double kAirExitShare     = 0.08;
    constexpr double kLoadRefSec       = 3.0;      // the usual corner load learns this slowly...
    constexpr double kLoadRefMinShare  = 0.5;      // ...and only while the car carries at least this share of it
    // What the sim sends that cannot be true, held off: a road speed jumping
    // faster than this (about 10 g, over the time since it last changed,
    // counted no shorter than the floor, so two bunched packets are not a
    // jump, and no longer than the ceiling, so a speed held steady a while
    // cannot spread a jump over that while) leaves the speed untrusted for
    // the hold; a body acceleration past about 30 g is dropped; slip angles
    // past 90 degrees, combined slip past this many times its peak % and
    // slip ratios outside lock..spin are capped, so one bad value cannot
    // hold a slide on for seconds through the smoothing.
    constexpr double kGlitchAccelMs2   = 100.0;
    constexpr double kGlitchMinDtSec   = 0.02;
    constexpr double kGlitchMaxDtSec   = 0.1;
    constexpr double kGlitchHoldSec    = 0.5;
    constexpr double kHeaveMaxMs2      = 300.0;
    constexpr double kSlipAngleMaxDeg  = 90.0;
    constexpr double kCombinedMaxX     = 4.0;
    constexpr double kRatioMin         = -1.0, kRatioMax = 3.0;
    constexpr double kGearMin          = -1.0, kGearMax = 12.0;   // a gear outside this is a garbage frame
    constexpr double kAngleJumpDeg     = 8.0;      // pitch or roll jumping this far between two samples: a glitch
    // Wheels laws
    constexpr double kFreshTyresSec    = 600.0;    // a different car brings fresh tyres; failing that, the stream quiet this long (a pause in the pits is not a new session)
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
inline void driveLaws(Layer& L, LawsState& st, const NcxValues& in, double dtSec,
                      const GearRatios* ratios = nullptr)
{
    using namespace laws_k;
    const bool live = in.fresh;

    // The channels as the laws use them: anything the sim sent that cannot
    // be true is capped or dropped here first (see laws_k).
    NcxValues v = in;
    {
        for (int t = 0; t < NcxValues::TokenCount; ++t)
            if (v.have[t] && !std::isfinite(v.val[t])) v.have[t] = false;
        const double combinedMax = kCombinedMaxX * std::max(1.0, L.fxParams(FxType::Skid).peakPct);
        for (int w = 0; w < WHEEL_COUNT; ++w)
        {
            double& a = v.val[NcxValues::SlipAngleFL + w];
            a = std::max(-kSlipAngleMaxDeg, std::min(kSlipAngleMaxDeg, a));
            double& c = v.val[NcxValues::WheelSlipFL + w];
            c = std::max(0.0, std::min(combinedMax, c));
            double& r = v.val[NcxValues::SlipRatioFL + w];
            r = std::max(kRatioMin, std::min(kRatioMax, r));
        }
        if (v.have[NcxValues::AccHeave] && std::fabs(v.val[NcxValues::AccHeave]) > kHeaveMaxMs2)
            v.have[NcxValues::AccHeave] = false;
        if (v.have[NcxValues::Gear] && (v.val[NcxValues::Gear] < kGearMin - 0.5 || v.val[NcxValues::Gear] > kGearMax + 0.5))
            v.have[NcxValues::Gear] = false;
        // The road speed: a jump no car can make leaves it untrusted (as if
        // unsent) for the hold, so nothing reads a jump to the pits as four
        // wheels locked at speed.
        st.glitchSec = std::max(0.0, st.glitchSec - dtSec);
        if (live && v.have[NcxValues::SpeedKmh])
        {
            const double s = v.val[NcxValues::SpeedKmh];
            st.speedAgeSec += dtSec;
            if (!st.speedSeen) { st.lastSpeedKmh = s; st.speedAgeSec = 0.0; st.speedSeen = true; }
            else if (s != st.lastSpeedKmh)
            {
                if (std::fabs(s - st.lastSpeedKmh) / 3.6
                    > kGlitchAccelMs2 * std::min(kGlitchMaxDtSec, std::max(kGlitchMinDtSec, st.speedAgeSec)))
                    st.glitchSec = kGlitchHoldSec;
                st.lastSpeedKmh = s; st.speedAgeSec = 0.0;
            }
        }
        else st.speedSeen = false;
        // The body's angles jumping further than a car turns between two
        // samples (a reset, a jump to the pits) are the same kind of glitch.
        if (live && v.have[NcxValues::PitchDeg] && v.have[NcxValues::RollDeg])
        {
            const double p = v.val[NcxValues::PitchDeg], r = v.val[NcxValues::RollDeg];
            if (st.anglesSeen && (std::fabs(p - st.lastPitch) > kAngleJumpDeg || std::fabs(r - st.lastRoll) > kAngleJumpDeg))
                st.glitchSec = kGlitchHoldSec;
            st.lastPitch = p; st.lastRoll = r; st.anglesSeen = true;
        }
        else st.anglesSeen = false;
        // While untrusted: no speed, and no body (heave, pitch, roll).
        if (st.glitchSec > 0.0)
            for (NcxValues::Token t : { NcxValues::SpeedKmh, NcxValues::AccHeave, NcxValues::PitchDeg, NcxValues::RollDeg })
                v.have[t] = false;
    }

    // A different car: what was learned about the last one starts over.
    if (live && v.car[0] != '\0')
    {
        if (st.carSeen && std::strncmp(v.car, st.lastCar, NCY_STR_LEN) != 0)
        {
            for (int w = 0; w < WHEEL_COUNT; ++w) { st.rollK[w] = 0.0; st.rollKnown[w] = false; st.air[w] = false; }
            st.rollSeedSec = 0.0;
            st.loadRef = 0.0;
            st.kerbBias[0] = st.kerbBias[1] = 0.0; st.kerbBiasSeen = false;
            st.quietSec = 0.0;
            relearnLatBase(st);
            for (int w = 0; w < WHEEL_COUNT; ++w) { st.absArmed[w] = false; st.absSince[w] = 1e9; }
            st.absWheelHoldSec = st.absWheelHz = 0.0;
            L.newCar();
        }
        std::memcpy(st.lastCar, v.car, NCY_STR_LEN);
        st.lastCar[NCY_STR_LEN - 1] = '\0';
        st.carSeen = true;
    }

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
    // The sim's TC flag, held over gaps up to the TC tile's hold ms, like
    // ABS's below: it flickers as traction control cuts in and out, and
    // each flicker restarted the engine's cut and the TC tile's pulse (a
    // stutter that felt like the rig). Hold 0: the flag as sent.
    const bool tcFlag = flag(NcxValues::TcActive) > 0.5;
    if (tcFlag) st.tcHoldSec = std::max(0.0, L.pulseParams(FxType::TcPulse).holdMs) / 1000.0;
    else        st.tcHoldSec = std::max(0.0, st.tcHoldSec - dtSec);
    const double tcOn = (tcFlag || st.tcHoldSec > 0.0) ? 1.0 : 0.0;

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
        const bool tcCut = tcOn > 0.5;
        L.driveEngine(lv, fireHz, load, lim, boost, pitLim, inGearOverrun, tcCut);
    }

    // Ground contact per tyre: the sim's own flag where it sends one
    // (onGround*), else the tyre's load (load*): a tyre carrying almost
    // nothing of the usual corner load is in the air (with hysteresis, so a
    // light tyre does not flicker). Unknown = on the ground. The Road,
    // Surface and Wheels tiles take it through the layer; slip and kerb
    // below. A punctured tyre (deflated*) goes to the Wheels tile.
    double ground[WHEEL_COUNT] = { 1.0, 1.0, 1.0, 1.0 };
    {
        bool flags = live, loads = live, defl = live;
        for (int w = 0; w < WHEEL_COUNT; ++w)
        {
            flags = flags && v.have[NcxValues::OnGroundFL + w];
            loads = loads && v.have[NcxValues::LoadFL + w];
            defl  = defl  && v.have[NcxValues::DeflatedFL + w];
        }
        if (flags)
        {
            for (int w = 0; w < WHEEL_COUNT; ++w) st.air[w] = v.val[NcxValues::OnGroundFL + w] < 0.5;
        }
        else if (loads)
        {
            double mean = 0.0;
            for (int w = 0; w < WHEEL_COUNT; ++w) mean += std::max(0.0, v.val[NcxValues::LoadFL + w]);
            mean /= WHEEL_COUNT;
            // The usual corner load (aero included), learned while the car is
            // mostly on the ground.
            if (st.loadRef <= 0.0) st.loadRef = mean;
            else if (mean > kLoadRefMinShare * st.loadRef)
                st.loadRef += std::min(1.0, dtSec / kLoadRefSec) * (mean - st.loadRef);
            for (int w = 0; w < WHEEL_COUNT; ++w)
            {
                const double share = (st.loadRef > 0.0) ? std::max(0.0, v.val[NcxValues::LoadFL + w]) / st.loadRef : 1.0;
                if (st.air[w]) { if (share > kAirExitShare) st.air[w] = false; }
                else if (share < kAirEnterShare) st.air[w] = true;
            }
        }
        else for (bool& a : st.air) a = false;
        for (int w = 0; w < WHEEL_COUNT; ++w)
        {
            ground[w] = st.air[w] ? 0.0 : 1.0;
            L.driveGround(w, ground[w]);
            L.driveDeflated(w, defl ? v.val[NcxValues::DeflatedFL + w] : 0.0);
        }
    }

    // Each wheel's signed slip ratio where one is known this cycle (from the
    // sim or the wheel speeds): the grip budget, the Wheels tile and ABS
    // from the wheels read it after the slip laws.
    double ratioW[WHEEL_COUNT] = {};
    bool   ratioOk[WHEEL_COUNT] = {};

    // Per-wheel slip. Raw physics in, severity out; the loaded tyre
    // weighted up when loads arrive; the old single channels as fallback.
    // A tyre in the air does not slip.
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

            // Learn the rolling factor while cruising on wheel speeds: over
            // seconds, and only while each axle's two wheels turn together
            // (none sliding, spinning or rolling backwards, no tight
            // corner) and, where the sim sends slip angles, no tyre is
            // sliding. A sample pulls at most kRollMaxStep off what was
            // learned, and the first value is taken only after half a
            // second of that, so a spin with the pedals off cannot teach it
            // (it did, and the rears then read as spinning for seconds).
            bool cruising = haveWs && speed > kRollLearnMinKmh
                && (!v.have[NcxValues::BrakePct]    || v.val[NcxValues::BrakePct]    < kRollLearnMaxBrk)
                && (!v.have[NcxValues::ThrottlePct] || v.val[NcxValues::ThrottlePct] < kRollLearnMaxThr);
            // A sender may count forward rotation as negative (Automobilista
            // 2 does): the size of the wheel speed is what counts. Learning
            // on its sign alone left Automobilista 2 with no rolling factor,
            // so no longitudinal slip at all.
            double kw[WHEEL_COUNT] = {};
            for (int w = 0; w < WHEEL_COUNT && cruising; ++w)
            {
                const double ws = std::fabs(v.val[NcxValues::WheelSpeedFL + w]);
                if (ws < 1e-6) cruising = false;
                else kw[w] = speed / ws;
            }
            cruising = cruising && std::fabs(kw[WheelFL] / kw[WheelFR] - 1.0) <= kRollAxleAgree
                                && std::fabs(kw[WheelRL] / kw[WheelRR] - 1.0) <= kRollAxleAgree;
            if (cruising && haveW(NcxValues::SlipAngleFL))
                for (int w = 0; w < WHEEL_COUNT; ++w)
                    if (std::fabs(v.val[NcxValues::SlipAngleFL + w]) > kRollMaxSlipDeg) cruising = false;
            // Where the lateral angle comes from a slip speed, a tyre sliding
            // sideways past what the car shows on a straight is a corner too
            // (a long coasting corner taught the corner's wheel speed split,
            // and the corner then counted as straight for the lateral
            // baseline). Last cycle's angle, and only once the baseline is
            // known (learned, or fixed): before it the straight-line angle
            // reads as a slide.
            if (cruising && st.slipSrc == 3 && (st.latBaseSec >= kLatBaseLearnSec || st.latBaseMode == 2))
                for (int w = 0; w < WHEEL_COUNT; ++w)
                    if (st.latRaw[w] - st.latBase[w] > kRollMaxSlipDeg) cruising = false;
            st.rollSeedSec = cruising ? st.rollSeedSec + dtSec : 0.0;
            if (cruising)
            {
                const double a = std::min(1.0, dtSec / kRollLearnSec);
                for (int w = 0; w < WHEEL_COUNT; ++w)
                {
                    if (!st.rollKnown[w])
                    {
                        if (st.rollSeedSec >= kRollSeedSec) { st.rollK[w] = kw[w]; st.rollKnown[w] = true; }
                        continue;
                    }
                    const double k = std::max((1.0 - kRollMaxStep) * st.rollK[w], std::min((1.0 + kRollMaxStep) * st.rollK[w], kw[w]));
                    st.rollK[w] += a * (k - st.rollK[w]);
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
                    // The road speed has no sign: a wheel turning backwards
                    // with the car (spun round, rolling back) is rolling,
                    // not locked.
                    const double r = std::fabs(v.val[NcxValues::WheelSpeedFL + w]) * st.rollK[w] / speed - 1.0;
                    lock = std::max(0.0, std::min(1.0, (-r - kLonOnsetRatio) / (peak - kLonOnsetRatio)));
                    spin = std::max(0.0, std::min(1.0, ( r - kLonOnsetRatio) / (peak - kLonOnsetRatio)));
                    ratioW[w] = r; ratioOk[w] = true;
                }
                else if (moving && !haveRatio && !haveWs && live && v.have[NcxValues::Lockup])
                    lock = single;
                lock = std::min(1.0, lock * loadW[w]) * ground[w];
                spin = std::min(1.0, spin * loadW[w]) * ground[w];
                if (preview) { lock = 1.0; spin = 1.0; }
                L.driveSlip(FxType::Lockup, w, lock, spin);
            }

            // The edge: the tyre working short of its limit, braking or
            // driving (a TC holds the driven wheels at 2 to 7 %, well short
            // of the spin texture's 15 %, so nothing was felt until the rear
            // let go): a fine grain from edge % (the tile's own) up to full
            // at the onset, eased in. From wheel speeds a wheel's ratio also
            // carries the corner's geometry (the outer wheel turns faster
            // than the road speed, the inner slower: +-5 % in a hairpin), so
            // there the axle's mean ratio is used: a hairpin's split cancels,
            // braking or driving the axle does not.
            for (int w = 0; w < WHEEL_COUNT; ++w)
            {
                double edge = 0.0;
                if (moving && ratioOk[w])
                {
                    const int m = w ^ 1;   // the axle's other wheel
                    const double r = (haveWs && ratioOk[m]) ? 0.5 * (ratioW[w] + ratioW[m]) : ratioW[w];
                    const double from = std::max(0.001, std::min(kLonOnsetRatio - 0.01, sp.screechPct / 100.0));
                    const double l = std::max(0.0, std::min(1.0, (std::fabs(r) - from) / (kLonOnsetRatio - from)));
                    edge = std::min(1.0, l * l * (3.0 - 2.0 * l) * loadW[w]) * ground[w];
                }
                if (preview) edge = 1.0;
                L.driveSlipExtra(FxType::Lockup, w, edge, 0.0);
            }
            // Lock judder beats with road speed; spin tramp is a resonance.
            const double hzScale = (speed > 0.0)
                ? std::max(kLockHzMin, std::min(kLockHzMax, speed / kLockHzRefKmh)) : 1.0;
            L.setSlipCarrierScale(FxType::Lockup, hzScale, 1.0);
        }

        // Lateral: slip angle (deg) from onset % of peak deg up to peak deg,
        // shaped by ease (0 linear; 1 = severity squared, a light scrub
        // first; 2 cubed). Where the sim has no slip angle but a slip speed
        // per wheel (Automobilista 2), the sideways angle worked out from it
        // (below); else a combined slip per wheel (Assetto Corsa's), from
        // onset % of peak % up to full at peak %. A combined slip cannot be split
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
            // A slip SPEED per wheel (m/s, sideways and along together:
            // Automobilista 2), taken apart: the along part is the wheel's
            // slip ratio x the road speed, what is left is sideways, and as
            // an angle (asin of it over the road speed) it plays through the
            // slip angle path, less each wheel's straight-line angle (toe,
            // the tyre's own creep), learned on the straights. As a plain
            // combined slip it grew with road speed: 2.5 deg's worth on
            // every straight, the scrub strongest at 250 km/h.
            const bool fromSpeed = !perWheel && haveW(NcxValues::SlipSpeedFL) && v.have[NcxValues::SpeedKmh];
            // The baseline (the tile's): learned on the straights (0),
            // learned once per car and then held (1), or fixed at base deg F
            // and R (2). Leaving fixed starts the learning over: the fixed
            // values were never learned.
            const int baseMode = std::max(0, std::min(2, static_cast<int>(sp.baseMode + 0.5)));
            if (st.latBaseMode == 2 && baseMode != 2) relearnLatBase(st);
            st.latBaseMode = baseMode;
            if (baseMode == 2)
                for (int w = 0; w < WHEEL_COUNT; ++w)
                    st.latBase[w] = std::max(0.0, std::min(kLatBaseMaxDeg, (w == WheelFL || w == WheelFR) ? sp.baseDegF : sp.baseDegR));
            if (fromSpeed)
            {
                const double vms = std::max(kLatMinSpeedMs, speed / 3.6);
                // Straight: the fronts turning together once their rolling
                // factors are known (without them nothing says the car is
                // straight), off the brake, every tyre short of its edge.
                bool straight = speed > kLatBaseMinKmh && st.rollKnown[WheelFL] && st.rollKnown[WheelFR]
                                && ratioOk[WheelFL] && ratioOk[WheelFR]
                                && v.have[NcxValues::WheelSpeedFL] && v.have[NcxValues::WheelSpeedFR]
                                && (!v.have[NcxValues::BrakePct] || v.val[NcxValues::BrakePct] < kRollLearnMaxBrk);
                if (straight)
                {
                    const double fl = std::fabs(v.val[NcxValues::WheelSpeedFL]) * st.rollK[WheelFL];
                    const double fr = std::fabs(v.val[NcxValues::WheelSpeedFR]) * st.rollK[WheelFR];
                    straight = std::fabs(fr - fl) / std::max(1e-6, 0.5 * (fr + fl)) < kLatBaseStraight;
                    for (int w = 0; w < WHEEL_COUNT && straight; ++w)
                        if (!ratioOk[w] || std::fabs(ratioW[w]) > kLonOnsetRatio / 3.0) straight = false;
                }
                // Fixed, or learned once and held: no learning.
                if (baseMode == 2 || (baseMode == 1 && st.latBaseSec >= kLatBaseLearnSec)) straight = false;
                if (straight) st.latBaseSec += dtSec;
                const double up   = std::min(1.0, dtSec / ((st.latBaseSec < kLatBaseLearnSec) ? kLatBaseSec : kLatBaseUpSec));
                const double down = std::min(1.0, dtSec / kLatBaseDownSec);
                // A sender with slip ratios or wheel speeds whose ratio is not
                // known yet (the rolling factor still learning after a car
                // change): that wheel's angle is unknown, not all sideways
                // (a race start's wheelspin read as a full slide).
                const bool alongSent = haveW(NcxValues::SlipRatioFL) || haveW(NcxValues::WheelSpeedFL);
                for (int w = 0; w < WHEEL_COUNT; ++w)
                {
                    if (alongSent && !ratioOk[w]) { st.latRaw[w] = 0.0; continue; }
                    const double ss    = std::fabs(v.val[NcxValues::SlipSpeedFL + w]);
                    const double along = ratioOk[w] ? std::fabs(ratioW[w]) * vms : 0.0;
                    const double side  = std::sqrt(std::max(0.0, ss * ss - along * along));
                    st.latRaw[w] = std::asin(std::min(1.0, side / vms)) * 180.0 / wavesynth::kPi;
                    if (straight)
                    {
                        const double t = std::min(kLatBaseMaxDeg, st.latRaw[w]);
                        st.latBase[w] += ((t > st.latBase[w]) ? up : down) * (t - st.latBase[w]);
                    }
                }
            }
            const bool combined = !perWheel && !fromSpeed && haveW(NcxValues::WheelSlipFL);
            const double peakC  = std::max(1.0, L.fxParams(FxType::Skid).peakPct);
            const double onsetC = onsetFrac * peakC;
            const double single = mag(NcxValues::Skid, FxType::Skid);
            // 1 the sim's slip angle, 3 the angle from a slip speed (both deg),
            // 2 a combined slip (peak %), 0 the single skid channel.
            const int src = perWheel ? 1 : fromSpeed ? 3 : combined ? 2 : 0;
            const bool angled = (src == 1 || src == 3);
            double in[WHEEL_COUNT] = {};
            for (int w = 0; w < WHEEL_COUNT; ++w)
                in[w] = (src == 1) ? std::fabs(v.val[NcxValues::SlipAngleFL + w])
                      : (src == 3) ? std::max(0.0, st.latRaw[w] - st.latBase[w])
                      : (src == 2) ? std::fabs(v.val[NcxValues::WheelSlipFL + w]) : 0.0;
            const double smA = (sp.smoothHz > 0.0) ? 1.0 - std::exp(-2.0 * wavesynth::kPi * sp.smoothHz * dtSec) : 1.0;
            if (src != st.slipSrc)
            {
                for (int w = 0; w < WHEEL_COUNT; ++w) st.slipF[w] = in[w];
                st.slipSrc = src;
            }
            const auto curve = [ease](double lin) { return std::pow(std::max(0.0, std::min(1.0, lin)), 1.0 + ease); };
            for (int w = 0; w < WHEEL_COUNT; ++w)
            {
                if (src != 0) st.slipF[w] += (in[w] - st.slipF[w]) * smA;
                st.latAngle[w] = angled ? st.slipF[w] : 0.0;
                double sev = 0.0;
                if (moving && angled)
                    sev = curve((st.slipF[w] - onset) / std::max(1e-6, peak - onset));
                else if (moving && combined)
                    sev = curve((st.slipF[w] - onsetC) / std::max(1e-6, peakC - onsetC));
                else if (moving && src == 0 && live && v.have[NcxValues::Skid])
                    sev = single;
                sev = std::min(1.0, sev * loadW[w]) * ground[w];
                if (preview) sev = 1.0;
                const bool front = (w == WheelFL || w == WheelFR);
                L.driveSlip(FxType::Skid, w, front ? sev : 0.0, front ? 0.0 : sev);
                double latFinal = sev;   // the slide as finally driven (the grip budget may replace it)

                // Grip budget: a wheel with both a slip angle and a slip
                // ratio slides when the two TOGETHER reach its limit. Each
                // is measured against its own grip limit (the onset angle;
                // the 0.15 ratio), the two combine as a circle (1 = at the
                // limit), and the slide grows from there to a full slide
                // (blended between the two full points by direction), then
                // plays on each tile by its direction (sideways: Lateral;
                // along: Longitudinal, lock or spin by sign). Pure sideways
                // or pure along gives the same severity as the tiles alone.
                if (sp.budget >= 0.5 && moving && angled && ratioOk[w] && !preview
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
                        // The way in (ease) is the Lateral tile's: by
                        // direction, so a slide straight along rises as the
                        // Longitudinal tile's does, in a straight line (with
                        // the lateral ease on it too, a 50% wheelspin played
                        // at a few %).
                        const double lin = std::max(0.0, std::min(1.0, (u - 1.0) / std::max(1e-6, uFull - 1.0)));
                        const double s   = std::pow(lin, 1.0 + ease * wa);
                        lat = std::min(1.0, s * (aN / u) * loadW[w]) * ground[w];
                        lon = std::min(1.0, s * (kN / u) * loadW[w]) * ground[w];
                    }
                    L.driveSlip(FxType::Skid, w, front ? lat : 0.0, front ? 0.0 : lat);
                    L.driveSlip(FxType::Lockup, w, ratioW[w] < 0.0 ? lon : 0.0, ratioW[w] > 0.0 ? lon : 0.0);
                    latFinal = lat;
                }

                // The fronts' screech, the tyres starting to squeal before
                // they let go: from screech % of peak to full at peak, eased
                // in (smoothstep), the loaded tyre weighted up; and the pulse,
                // the slow stick-slip shove, as strong as the slide itself.
                if (front)
                {
                    double screech = 0.0;
                    if (moving)
                    {
                        const double x = angled ? st.slipF[w] / peak
                                       : combined ? st.slipF[w] / peakC
                                       : (live && v.have[NcxValues::Skid]) ? single : 0.0;
                        const double from = std::max(0.0, std::min(0.95, sp.screechPct / 100.0));
                        const double l = std::max(0.0, std::min(1.0, (x - from) / (1.0 - from)));
                        screech = std::min(1.0, l * l * (3.0 - 2.0 * l) * loadW[w]) * ground[w];
                    }
                    if (preview) screech = 1.0;
                    L.driveSlipExtra(FxType::Skid, w, screech, latFinal);
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
        // locked wheel; the brake heats the discs. A different car brings
        // fresh tyres (above); failing that, a stream quiet for ten minutes
        // (a pause in the pits keeps them). A Test preview
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

    // ABS: only while ABS is working AND the brake is applied (some sims
    // flicker the flag at zero brake), and as hard as the driver brakes:
    // each dump and re-apply swings the deceleration more at high pressure,
    // so light trail braking is a light pulse, full by full at brake % (10
    // = always full). The road speed slows the cycle towards a stop
    // (unknown: full rate; a Test preview runs at speed).
    // Working = the sim's flag, held over gaps up to the tile's hold ms
    // (Assetto Corsa raises it in blips, about every other sample through a
    // stop, which chopped the cycle into fragments; 0 = the flag as sent);
    // or, with from wheels on, the wheels themselves: a sim that reports
    // its ABS only now and then (Automobilista 2: 7 % of the braking) still
    // shows it in each wheel dipping towards a lock and spinning back up as
    // its valve dumps. Two dumps on a wheel at an ABS rhythm (25 to 3 Hz)
    // are ABS working, and each dump times that corner's pulse (its cycle
    // pulled into step with the dumps, at the rate they come). A wheel that
    // locks and stays locked (no ABS), or hovers near a lock (threshold
    // braking), never dumps; a tyre in the air is not counted.
    {
        const PulseParams& pp = L.pulseParams(FxType::AbsPulse);
        const bool absFlag = flag(NcxValues::AbsActive) > 0.5;
        if (absFlag) st.absHoldSec = std::max(0.0, pp.holdMs) / 1000.0;
        else         st.absHoldSec = std::max(0.0, st.absHoldSec - dtSec);
        const bool braking = live && v.have[NcxValues::BrakePct] && v.val[NcxValues::BrakePct] > kAbsMinBrakePct;
        const bool rolling = live && v.have[NcxValues::SpeedKmh] && v.val[NcxValues::SpeedKmh] > kSlipMinSpeedKmh;
        st.absWheelHoldSec = std::max(0.0, st.absWheelHoldSec - dtSec);
        for (int w = 0; w < WHEEL_COUNT; ++w)
        {
            st.absSince[w] += dtSec;
            if (!(pp.fromWheels >= 0.5 && braking && rolling && ratioOk[w] && ground[w] > 0.5))
            {
                st.absArmed[w] = false; st.absSince[w] = 1e9;
                continue;
            }
            const double r = ratioW[w];
            if (!st.absArmed[w])
            {
                if (r < -kAbsDipRatio) { st.absArmed[w] = true; st.absMinR[w] = r; }
                continue;
            }
            st.absMinR[w] = std::min(st.absMinR[w], r);
            if (r > -kAbsRecoverLevel && r > st.absMinR[w] + kAbsRecoverRatio)
            {
                const double period = st.absSince[w];
                if (period >= kAbsMinPeriodSec && period <= kAbsMaxPeriodSec)
                {
                    st.absWheelHoldSec = kAbsWheelHoldSec;
                    st.absWheelHz += 0.3 * (1.0 / period - st.absWheelHz);
                    L.absSync(w, period);
                }
                st.absSince[w] = 0.0; st.absArmed[w] = false;
            }
        }
        st.absFlagOn = (absFlag || st.absHoldSec > 0.0) && braking;
        const bool on = st.absFlagOn || (st.absWheelHoldSec > 0.0 && braking);
        if (!on) L.absUnsync();   // ABS off: the next stop starts free, not on a stale timing
        double level = 0.0;
        if (on)
        {
            const double full = std::max(kAbsMinBrakePct, pp.brakeFull);
            level = (full <= kAbsMinBrakePct + 1e-9) ? 1.0
                  : std::max(0.0, std::min(1.0, (v.val[NcxValues::BrakePct] - kAbsMinBrakePct) / (full - kAbsMinBrakePct)));
        }
        const bool previewing = st.previewSec[static_cast<int>(FxType::AbsPulse)] > 0.0;
        L.driveAbsSpeed(previewing ? kPreviewKmh
                        : (live && v.have[NcxValues::SpeedKmh]) ? std::max(0.0, v.val[NcxValues::SpeedKmh]) : -1.0);
        L.driveFx(FxType::AbsPulse, previewOr(FxType::AbsPulse, level), 0.0);
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
    // under each tyre where it sends one; else the stronger of its curbs
    // channel (all wheels, full at peak %) and, with detect mm set and the
    // road height under each tyre, a tyre stepped up above its axle mate by
    // detect mm more than usual for that axle (camber and banking learned
    // off the kerbs). A Test preview runs the strip at 80 km/h, on and off.
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
        else
        {
            // The game's kerb channel (Competizione, EVO) on every wheel,
            // and with detect mm set and the road heights arriving, a tyre
            // stepping up past its axle mate: the stronger of the two. A
            // numbered line pads a kerb channel it does not have with 0, so
            // the channel must never shut the detector out (it did, in
            // Assetto Corsa).
            if (live && v.have[NcxValues::Curbs])
            {
                const double c = mag(NcxValues::Curbs, FxType::Kerb);
                for (double& l : lvl) l = c;
            }
            if (kp.detectMm > 0.0 && haveW(NcxValues::RoadHeightFL) && kmh > kKerbMinKmh)
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
                    // Learns off the kerbs; on one, a tenth as fast (a bias
                    // seeded on a kerb still finds its way back).
                    st.kerbBias[ax] += ((std::fabs(ex) < half) ? a : 0.1 * a) * (d - st.kerbBias[ax]);
                    const int up = ex > 0.0 ? l : r;
                    lvl[up] = std::max(lvl[up], std::max(0.0, std::min(1.0, (std::fabs(ex) - half) / half)));
                }
            }
        }
        st.kerbBiasSeen = heights;
        for (int w = 0; w < WHEEL_COUNT; ++w) L.driveKerb(w, lvl[w] * ground[w]);   // a tyre in the air is on no kerb
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
    L.driveFx(FxType::TcPulse, previewOr(FxType::TcPulse, tcOn), 0.0);

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
