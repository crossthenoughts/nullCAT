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
    // The gear the box is in (the last one knocked for), and a new value
    // waiting to hold long enough to be a gear (a junk frame is not).
    int    lastGear = 0;
    bool   gearSeen = false;
    int    gearCand = 0;
    double gearCandSec = 0.0;
    double shiftOpenSec = -1.0;   // ignore neutral: since the box left its gear into N (-1 = not in a shift)
    bool   dogAtLanding = false;  // ...a dog box left under power: its engagement knock comes as the gear goes in
    // The engine speed last sent that could be true (a junk frame's rpm is
    // replaced by it).
    double lastRpm = 0.0, rpmAgeSec = 0.0;
    bool   rpmSeen = false;
    double engineHoldHz = 0.0;   // the engine's last live firing rate: held while the stream is away
    double maxRpmVal = 0.0, maxRpmHeldSec = 0.0;   // the sim's max rpm, and how long it has held (the redline seed)
    // Rolling factor per wheel for the wheelSpeed fallback: speedKmh /
    // wheelSpeed learned while cruising (no brake, light throttle), so a
    // sender's wheel-speed unit (rev/s, rad/s, km/h) and a staggered tyre
    // set both come out as ratio 0 when rolling. Unknown until learned.
    double rollK[WHEEL_COUNT]     = {};
    bool   rollKnown[WHEEL_COUNT] = {};
    // The first value: samples taken while the wheels looked fit to learn
    // from, however short the moments (a race car is rarely at light
    // throttle for long), one per kRollSeedEverySec of that time.
    static constexpr int ROLL_SEED_SLOTS = 16;
    double rollSeedSec = 0.0;      // time the wheels have looked fit to learn from, in all
    double rollSeedNextSec = 0.0;  // ...when the next sample is due
    int    rollSeedN = 0;
    double rollSeedK[WHEEL_COUNT][ROLL_SEED_SLOTS] = {};
    double rollCruiseSec = 0.0;    // cruising time with a wheel still unknown (the seed's axle agreement widens after a while)
    double rollOffSec = 0.0;       // how long the samples have disagreed with what was learned, one way, without a break
    int    rollOffSign[WHEEL_COUNT] = {};   // ...which way each wheel was off (+1 faster, -1 slower, 0 inside)
    bool   rollOpen = false;               // ...escaped: learning until every wheel is back well inside
    // Which axle the engine drives, as the wheels show it: seconds of full
    // throttle with that axle running ahead of the other (0 front, 1 rear).
    double drivenSec[2] = {};
    double leadCruise = 0.0, leadCruiseSec = 0.0;   // ...against how far the rears run ahead of the fronts cruising
    // Clutch judder: how long the pedal has been in the slipping band.
    double clutchBandSec = 0.0;
    // Driveline shunt: the throttle a moment ago, for the zero-torque
    // crossing; and the dog box's second knock after a shift under power.
    double lastThrottle = 0.0;
    bool   throttleSeen = false;
    double lowThrottleAgo = 1e9, highThrottleAgo = 1e9;
    double dogKnockIn = 0.0;   // seconds until the dog engagement knock (0 = none pending)
    // Kerb detection from road heights, per axle: the last few sim samples
    // of the left-minus-right height difference with their times (a jump
    // across them is a kerb's edge or ribs), the jumps' fading envelope,
    // and the kerb under way: on or not, how long, the difference before it
    // began, and its side from the first jump (0 none, 1 left, 2 right).
    static constexpr int KERB_HIST = 32;
    bool   kerbSeen = false;
    double kerbClock = 0.0;
    double kerbHistT[2][KERB_HIST] = {}, kerbHistD[2][KERB_HIST] = {};
    int    kerbHistN[2] = {}, kerbHistHead[2] = {};
    double kerbEnv[2] = {};
    bool   kerbOn[2] = {};
    double kerbOnSec[2] = {}, kerbRef[2] = {}, kerbQuietSec[2] = {};
    double kerbLastRef[2] = {}, kerbSinceOff[2] = { 1e9, 1e9 };   // the last kerb: where it began, how long since it ended
    int    kerbLastSide[2] = {};
    int    kerbSide[2] = {};
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
    // Fresh tyres on a reset (the Wheels tile's freshOnReset): the pit flag
    // last sent and how long the car has been out since it was last up;
    // how long since it rose after a run out, since the road speed jumped
    // to a standstill between two live samples, and since the stream came
    // back after a gap; a jump waiting to hold (no pit flag sent); the lap
    // in force and a different one waiting to hold; each tyre's wear
    // against its highest in the last second and how long a fall from it
    // has held; the guard between two refreshes.
    bool   pitSeen = false, pitWas = false;
    double onTrackSec = 0.0, pitRoseAgo = 1e9;
    double pitFirstAgo = 1e9;   // since the flag was first sent set (a sender that sends it only once it has seen it set)
    double jumpAgo = 1e9, resumeAgo = 1e9;
    bool   wasLive = false;
    bool   jumpHolding = false;
    double jumpHoldSec = 0.0;
    bool   lapSeen = false;
    long   lastLap = 0, lapCand = 0;
    double lapCandSec = 0.0;
    bool   wearSeen[WHEEL_COUNT] = {};
    double wearHi[WHEEL_COUNT] = {}, wearHiSec[WHEEL_COUNT] = {}, wearLowSec[WHEEL_COUNT] = {};
    double freshGuardSec = 0.0;
    // Ground contact: the usual corner load (learned) and which tyres are
    // in the air (with hysteresis).
    double loadRef = 0.0;
    bool   air[WHEEL_COUNT] = {};
    // The car the sim last named: a different one starts the per-car
    // learning over (rolling factor, usual load, kerb detector, the engine's
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
    bool   absFlagSeen = false;   // the sim's ABS flag has been on, this car: its ABS works (the wheels may fill its gaps)
    // ABS from the wheels: per wheel, armed once it dips towards a lock,
    // its lowest ratio since, the time since its last dump; the hold after
    // a dump that counted, and the rate the dumps come at (for the card).
    bool   absArmed[WHEEL_COUNT] = {};
    double absMinR[WHEEL_COUNT] = {}, absSince[WHEEL_COUNT] = { 1e9, 1e9, 1e9, 1e9 };
    double absWheelHoldSec = 0.0, absWheelHz = 0.0;
    // Lateral from a slip speed: each wheel's sideways angle (deg), its
    // straight-line baseline (learned) and the angle as played (smoothed,
    // less the baseline; also a sim's own slip angle), for the card; and
    // each wheel's straight-line slip along the road (a share of the road
    // speed), learned on the straights.
    double latRaw[WHEEL_COUNT] = {}, latBase[WHEEL_COUNT] = {}, latAngle[WHEEL_COUNT] = {};
    double latCreep[WHEEL_COUNT] = {};
    double latBaseSec = 0.0;   // straight-line time the baseline has learned over
    int    latBaseMode = 0;    // the tile's baseline mode last cycle (leaving fixed starts the learning over)
};

// The Lateral tile's straight-line baseline starts over: its relearn
// button, and a new car.
inline void relearnLatBase(LawsState& st)
{
    for (int w = 0; w < WHEEL_COUNT; ++w) st.latRaw[w] = st.latBase[w] = st.latAngle[w] = st.latCreep[w] = 0.0;
    st.latBaseSec = 0.0;
}

// A different car: everything learned or remembered about the last one
// starts over. By construction, not by list: a fresh state, with only what
// outlives a car change copied back (a Test preview under way, the car's
// name, the road-speed and body-angle glitch trackers, the baseline mode
// the tile was last in), so a field that forgets to opt in is reset,
// never carried over (the gear edge, the water film and the throttle
// crossing were carried over: a knock, puddles and a shunt on the change).
// The Layer's models have their own newCar(). RT-safe: one stack copy.
inline void newCar(LawsState& st)
{
    LawsState fresh;
    std::memcpy(fresh.previewSec, st.previewSec, sizeof fresh.previewSec);
    std::memcpy(fresh.lastCar, st.lastCar, sizeof fresh.lastCar);
    fresh.carSeen      = st.carSeen;
    fresh.lastSpeedKmh = st.lastSpeedKmh; fresh.speedAgeSec = st.speedAgeSec; fresh.glitchSec = st.glitchSec;
    fresh.speedSeen    = st.speedSeen;
    fresh.lastPitch    = st.lastPitch;    fresh.lastRoll = st.lastRoll; fresh.anglesSeen = st.anglesSeen;
    fresh.latBaseMode  = st.latBaseMode;
    st = fresh;
}

namespace laws_k {
    constexpr double kEngineAliveRpm   = 30.0;     // anything turning: cranking lumps from the starter up
    constexpr double kOverrunThrottle  = 10.0;     // % throttle at or below which the throttle is shut
    constexpr double kOverrunMinKmh    = 10.0;     // in gear and rolling: the wheels drive the engine
    constexpr double kPreviewIdleRpm   = 1100.0;   // the canned idle a Test preview runs
    // The Engine Test with no sim: the canned idle for kPreviewIdleSec, then
    // the revs climb to the redline as the load goes from kPreviewLoadFrom to
    // full, and the last kPreviewTopSec sit at the redline on the cut.
    constexpr double kPreviewIdleSec   = 0.3;
    constexpr double kPreviewTopSec    = 0.3;
    constexpr double kPreviewLoadFrom  = 0.5;
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
    // The slip speed's along part: Automobilista 2 measures a rolling
    // tyre's slip against another radius than the one it rolls on, so on a
    // straight each wheel shows a slip along the road (c, a share of the
    // road speed: 3 to 6 % on the fronts, about 1 % on the rears) that read
    // as a sideways angle. c is learned per wheel on the straights while
    // that wheel turns within kLatCreepRatio of the road (near zero torque),
    // over kLatBaseSec at first and kLatCreepSec once learned, kept within
    // these bounds; the along part is then (ratio + c) x the road speed.
    // A wheel spinning or locked past kLatRatioGate: the sideways part
    // cannot be told from it (a launch read as a 90 degree slide).
    // The angle from a slip speed fades in from kLatFadeFromKmh to full
    // at kLatFadeFullKmh (below it the sideways part is mostly noise).
    constexpr double kLatCreepRatio    = 0.01;
    constexpr double kLatCreepSec      = 10.0;
    constexpr double kLatCreepMin      = -0.02, kLatCreepMax = 0.12;
    constexpr double kLatRatioGate     = 0.5;
    constexpr double kLatFadeFromKmh   = 10.0, kLatFadeFullKmh = 25.0;
    constexpr double kPreviewSec       = 2.0;      // Test preview length
    // Slip laws (the lateral onset, curve and smoothing are the tile's:
    // SlipParams onsetPct, ease, smoothHz)
    constexpr double kLonOnsetRatio    = 0.15;     // longitudinal: peak grip sits around 0.1-0.2 ratio
    constexpr double kSlipMinSpeedKmh  = 5.0;      // ratios mean nothing at a standstill
    constexpr double kLockHzRefKmh     = 80.0;     // lock judder carrier = set hz at this road speed
    constexpr double kLockHzMin        = 0.4, kLockHzMax = 1.4;
    constexpr double kLoadWeightMax    = 1.5;      // loaded tyre weighting cap (the Wheels tile's load share)
    constexpr double kRollLearnMinKmh  = 30.0;
    constexpr double kRollLearnMaxThr  = 30.0;     // % throttle; above this a driven wheel may be slipping
    constexpr double kRollLearnMaxBrk  = 5.0;
    constexpr double kRollLearnSec     = 2.0;      // learns over seconds while cruising...
    constexpr double kRollSeedSec      = 0.5;      // ...the first value only after this long fit to learn from, in all
    constexpr double kRollSeedEverySec = 0.05;     // ...one sample per this much of it, wherever it falls (the median of them)
    constexpr double kRollSeedAgree    = 0.02;     // ...each wheel's samples this close to their median...
    constexpr double kRollSeedShare    = 0.75;     // ...this share of them, or keep collecting (a spin's samples disagree)
    constexpr double kRollSeedAxleAgree = 0.01;    // ...each sample with both axles' wheels this close (a pit-exit curve taught a 4.6 % error)...
    constexpr double kRollSeedWidenSec = 20.0;     // ...widening to kRollAxleAgree after this long cruising without a value (left/right tyre stagger)
    constexpr double kRollAxleAgree    = 0.04;     // an axle's two wheels within this of each other (not sliding, not a tight corner)
    constexpr double kRollMaxSlipDeg   = 3.0;      // no tyre past this slip angle, where the sim sends them
    constexpr double kRollPreBaseSlipDeg = 6.0;    // ...an angle from a slip speed before its baseline is known: past this
    constexpr double kRollMaxStep      = 0.05;     // a sample pulls at most this share off what was learned
    // Once learned, a sample further than this off it is ignored (a reset,
    // a slide, a curve), unless they have disagreed this long in all (a
    // wrong first value, a real change), when the pull above applies.
    constexpr double kRollGate         = 0.015;
    constexpr double kRollGateEscapeSec = 5.0;
    constexpr double kRollSettle       = 0.0075;   // ...once escaped, learning until every wheel is this close
    constexpr double kRollClutchLo     = 10.0, kRollClutchHi = 90.0;   // % clutch: slipping between, no learning
    // Engine braking (throttle under this, in gear, the clutch up) turns
    // the driven wheels slower than the road: no learning on that axle.
    // The driven axle is the one running ahead under kDrivenThrottle %
    // between these speeds by kDrivenAhead more than it does cruising, for
    // kDrivenSec in all.
    constexpr double kRollOverrunThr   = 3.0;
    constexpr double kDrivenThrottle   = 80.0;
    constexpr double kDrivenMinKmh     = 30.0, kDrivenMaxKmh = 200.0;
    constexpr double kDrivenAhead      = 0.01;
    constexpr double kDrivenSec        = 2.0;
    constexpr double kDrivenLeadSec    = 1.0;      // the cruising lead: its time constant, and the cruising it needs first
    // Driveline laws
    constexpr double kClutchSlipLo     = 15.0;     // % pedal: below this the clutch is up (driving)
    constexpr double kClutchSlipHi     = 85.0;     // % pedal: above this it is floored (open)
    constexpr double kClutchLaunchKmh  = 5.0;      // slower than this with the engine up = a launch (full slip)
    constexpr double kClutchMinRpm     = 500.0;    // nothing from a stalled engine
    constexpr double kJudderHoldSec    = 0.25;     // above a launch, the pedal held in the slipping band this long (a shift's quick dip is no slip)
    constexpr double kLugThrottle      = 50.0;     // % throttle at or above which the engine can be lugged
    constexpr double kLugFromX         = 0.30;     // lug fades out by this fraction of the redline...
    constexpr double kLugFullX         = 0.12;     // ...and is full at this one (just above idle)
    // The differential, a walking-pace thing: a limited slip full below
    // this road speed and gone by this (a spool's scrub by these), nothing
    // below kDiffMinKmh (two near-stopped wheels' split is noise). The
    // corner's own geometry this far apart starts it, this far is full (a
    // turn of about 10 m radius at a 1.5 m track); the driven wheels may run
    // this much past the free axle's split before it is a spinning wheel,
    // not the corner; a free axle this far apart is the car rotating. A
    // limited slip's preload clunks this much off throttle. The clunks'
    // rate: diff hz once the wheels turn apart at kDiffSlipRefMs, down to
    // this share of it.
    constexpr double kDiffFullKmh      = 8.0,  kDiffGoneKmh = 20.0;
    constexpr double kDiffSpoolFullKmh = 10.0, kDiffSpoolGoneKmh = 30.0;
    constexpr double kDiffMinKmh       = 2.0;
    constexpr double kDiffFromSplit    = 0.05, kDiffFullSplit = 0.15;
    constexpr double kDiffSpoolFromSplit = 0.06, kDiffSpoolFullSplit = 0.15;
    constexpr double kDiffSpinAllow    = 0.03;
    constexpr double kDiffMaxGeoSplit  = 0.4;
    constexpr double kDiffPreload      = 0.5;
    constexpr double kDiffSlipRefMs    = 0.5;
    constexpr double kDiffRateMin      = 0.3;
    constexpr double kLugMinRpm        = 600.0;    // not while cranking or dying
    // Whine: pitch from the input shaft at an order the actuator carries,
    // stepping per gear (a lower gear meshes slower per engine rev).
    // Whine pitch against the engine buzz, 1st gear to 6th, by the tile's
    // whine pitch: with the buzz (70 to 100%: in the top gears the two sit
    // a few Hz apart and beat, a slow pulsing), below it, an octave down,
    // above it. All but the first stay at least 20% off the buzz.
    constexpr double kWhineLow[4]      = { 0.70, 0.62, 0.35, 1.20 };
    constexpr double kWhineHigh[4]     = { 1.00, 0.80, 0.50, 1.50 };
    constexpr double kWhineSamplesPerCycle = 8.0;  // a gear whose redline pitch the loop cannot carry this well drops an octave
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
    constexpr double kGearSettleSec    = 0.045;    // a gear must hold this long to count (a junk frame lasts one 60 Hz sample, 17 ms, up to about 30 with timing jitter)
    constexpr double kNeutralHeldSec   = 0.5;      // ignore neutral: a gear going in within this of leaving the last is the same shift; a neutral held longer is a real one
    // Road and kerb laws
    constexpr double kPreviewKmh       = 80.0;     // the road speed a road, kerb or ABS Test preview runs at
    constexpr double kPreviewKerbOnSec = 0.6;      // kerb preview: on this long...
    constexpr double kPreviewKerbCycle = 1.0;      // ...in every this long
    constexpr double kKerbMinKmh       = 10.0;     // height detection: rolling
    // A kerb is a JUMP in an axle's left-right height difference: the
    // largest change across the sim samples of the last this long (a kerb's
    // edge, its ribs; two samples at 60 Hz, more at a faster rate), the
    // envelope of the jumps fading over this long. The road's camber and
    // banking change over seconds (Imola: 20 to 55 mm, axle by axle), about
    // 1 mm in the window, so they never jump. A kerb lasts from its first
    // jump until the axle is back level with where it was before (the drop
    // off it is the same kerb, on the same side), at most this long.
    constexpr double kKerbJumpSec      = 0.035;
    constexpr double kKerbHoldSec      = 0.25;
    constexpr double kKerbEventMaxSec  = 4.0;
    constexpr double kKerbQuietSec     = 0.4;      // ...or until it has been quiet this long (the camber may have moved)
    constexpr double kKerbDropSec      = 2.0;      // a jump back towards where it began, within this after, is its drop off
    // ABS: no ABS below walking pace (the ABS tile's off < 5 km/h).
    constexpr double kAbsLowOffKmh     = 5.0;
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
    // An engine speed past this, or changing faster than this (rpm/s, over
    // the time since it last changed, from the speed guard's floor: 3000
    // rpm in a 60 Hz sample, 15000 after a tenth of a second, anything after
    // a quarter), is a garbage frame (Automobilista 2 sent 191,105 rpm once,
    // with 386 km/h and gear 128): the last one that could be true stands,
    // and the frame's limiter flag, which the sender set from that rpm,
    // goes with it (it would teach the redline).
    constexpr double kRpmMax           = 40000.0;   // an electric motor revs past 20000
    constexpr double kRpmSlew          = 150000.0;
    constexpr double kRpmMaxAgeSec     = kRpmMax / kRpmSlew;   // the allowance stops growing here: any rpm below kRpmMax is taken
    constexpr double kRpmStopped       = 500.0;    // an engine stopping (a stall, the ignition off) is taken at once
    constexpr double kMaxRpmHoldSec    = 0.5;      // the sim's max rpm seeds the redline once it has held this long
    constexpr double kAngleJumpDeg     = 8.0;      // pitch or roll jumping this far between two samples: a glitch
    // Wheels laws
    constexpr double kFreshTyresSec    = 600.0;    // a different car brings fresh tyres; failing that, the stream quiet this long (a pause in the pits is not a new session)
    // Fresh tyres on a reset (freshOnReset): the road speed jumping from
    // above kJumpFromKmh to below kJumpToKmh from one sample to the next is
    // the car put somewhere, not driven (no car stops like that); the pit
    // flag rising within kFreshPairSec of such a jump, or of the stream
    // coming back after a gap (the pause menu), after the car was out
    // kFreshOutSec, is a return to the pits; a tyre's wear falling
    // kFreshWearDrop below its highest in the last kFreshWearSec is new
    // tyres. A jump with no pit flag sent, a lap number and a fall in wear
    // count once they have held kFreshConfirmSec (one junk sample is not a
    // reset; longer than the 500 ms a sender's last packet stays live). At
    // most one refresh every kFreshGuardSec.
    constexpr double kJumpFromKmh      = 40.0;
    constexpr double kJumpToKmh        = 2.0;
    constexpr double kFreshOutSec      = 5.0;
    constexpr double kFreshPairSec     = 1.0;
    constexpr double kFreshWearDrop    = 0.2;
    constexpr double kFreshWearSec     = 1.0;
    constexpr double kFreshConfirmSec  = 0.75;
    constexpr double kFreshGuardSec    = 2.0;
    constexpr double kPreviewWheelSpread[WHEEL_COUNT] = { 0.0, 0.004, -0.003, 0.007 };   // a Test: each wheel a touch apart
    constexpr double kPreviewFlatMm    = 0.8;
    constexpr double kPreviewBrake     = 0.6;
    // Traction control: nothing to cut without throttle (the sim's flag and
    // its hold ran on after a lift).
    constexpr double kTcMinThrottlePct = 10.0;
    // Impacts laws: slower than this the car is standing (a reset or a
    // spawn drops it onto the ground); a Test preview's knocks (wheel -1 =
    // every corner).
    constexpr double kImpactStillKmh   = 2.0;
    struct PreviewKnock { double atSec; int wheel; double severity, sign; };
    constexpr PreviewKnock kPreviewKnocks[] = {
        { 0.10, WheelFL, 1.0, 1.0 }, { 0.50, WheelFR, 0.6, 1.0 }, { 0.90, WheelRL, 1.0, -1.0 },
        { 1.30, WheelRR, 0.6, 1.0 }, { 1.70, -1, 1.0, 1.0 } };
}

// A surface class as sent (any finite number): clamped to the table before
// the cast (a value past the int range is undefined to cast).
inline int surfaceClass(double sent)
{
    const double c = std::max(0.0, std::min(static_cast<double>(SURFACE_CLASS_COUNT - 1), sent + 0.5));
    return static_cast<int>(c);
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
    bool gearDropped = false;   // this frame's gear was junk (dropped), not unsent
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
        {
            v.have[NcxValues::Gear] = false;
            gearDropped = true;   // a junk frame, not a stream gone: the gear shift keeps its place
        }
        st.rpmAgeSec += dtSec;
        // A different car this frame: its first rpm owes nothing to the last
        // car's (the per-car reset below runs after this).
        if (!live || (v.car[0] != '\0' && st.carSeen && std::strncmp(v.car, st.lastCar, NCY_STR_LEN) != 0))
            st.rpmSeen = false;
        if (live && v.have[NcxValues::Rpm])
        {
            const double rpm = v.val[NcxValues::Rpm];
            bool junk = rpm > kRpmMax;
            // The allowance grows with the time since the last true change,
            // until anything below kRpmMax is taken (a stall from 18,000 or a
            // reset to the pits stood for good at a tenth-second ceiling); an
            // engine stopping is taken at once.
            if (!junk && st.rpmSeen && rpm != st.lastRpm && rpm > kRpmStopped)
                junk = std::fabs(rpm - st.lastRpm) > kRpmSlew * std::min(kRpmMaxAgeSec, std::max(kGlitchMinDtSec, st.rpmAgeSec));
            if (junk)
            {
                if (st.rpmSeen) v.val[NcxValues::Rpm] = st.lastRpm;
                else            v.have[NcxValues::Rpm] = false;
                v.have[NcxValues::Limiter] = false;
            }
            else
            {
                if (!st.rpmSeen || rpm != st.lastRpm) st.rpmAgeSec = 0.0;
                st.lastRpm = rpm; st.rpmSeen = true;
            }
        }
        // The road speed: a jump no car can make leaves it untrusted (as if
        // unsent) for the hold, so nothing reads a jump to the pits as four
        // wheels locked at speed.
        st.glitchSec = std::max(0.0, st.glitchSec - dtSec);
        st.jumpAgo += dtSec;
        if (live && v.have[NcxValues::SpeedKmh])
        {
            const double s = v.val[NcxValues::SpeedKmh];
            st.speedAgeSec += dtSec;
            if (!st.speedSeen) { st.lastSpeedKmh = s; st.speedAgeSec = 0.0; st.speedSeen = true; }
            else if (s != st.lastSpeedKmh)
            {
                // A jump to a standstill between two live samples (fresh
                // tyres on a reset).
                if (st.lastSpeedKmh > kJumpFromKmh && s < kJumpToKmh) st.jumpAgo = 0.0;
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
        // While untrusted: no speed, no body (heave, pitch, roll), and no
        // suspension or road under the tyres (a reset's travel jump played
        // a full-scale road pop and an impact knock).
        if (st.glitchSec > 0.0)
        {
            for (NcxValues::Token t : { NcxValues::SpeedKmh, NcxValues::AccHeave, NcxValues::PitchDeg, NcxValues::RollDeg })
                v.have[t] = false;
            for (int w = 0; w < WHEEL_COUNT; ++w)
            {
                v.have[NcxValues::SuspVelFL + w] = false;
                v.have[NcxValues::SuspTravelFL + w] = false;
                v.have[NcxValues::RoadHeightFL + w] = false;
            }
        }
    }

    // A different car: what was learned about the last one starts over.
    if (live && v.car[0] != '\0')
    {
        if (st.carSeen && std::strncmp(v.car, st.lastCar, NCY_STR_LEN) != 0)
        {
            newCar(st);
            L.newCar();
        }
        std::memcpy(st.lastCar, v.car, NCY_STR_LEN);
        st.lastCar[NCY_STR_LEN - 1] = '\0';
        st.carSeen = true;
    }
    // The sim's own max rpm seeds the learned redline (once per car), once
    // it has held steady for kMaxRpmHoldSec: a sender's may lag the car's
    // name, and the last car's would stick for the session.
    if (live && v.have[NcxValues::MaxRpm] && v.val[NcxValues::MaxRpm] <= kRpmMax)
    {
        const double m = v.val[NcxValues::MaxRpm];
        if (std::fabs(m - st.maxRpmVal) <= 1.0) st.maxRpmHeldSec += dtSec;
        else { st.maxRpmVal = m; st.maxRpmHeldSec = 0.0; }
        if (st.maxRpmHeldSec >= kMaxRpmHoldSec) L.seedMaxRpm(m);
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
    // No throttle, nothing for a traction control to cut: off the pedal the
    // flag and its hold are ignored (Automobilista 2's ran on after a lift,
    // seconds of TC with the throttle shut).
    const bool tcThrottle = !(live && v.have[NcxValues::ThrottlePct]) || v.val[NcxValues::ThrottlePct] >= kTcMinThrottlePct;
    const bool tcFlag = flag(NcxValues::TcActive) > 0.5 && tcThrottle;
    if (tcFlag) st.tcHoldSec = std::max(0.0, L.pulseParams(FxType::TcPulse).holdMs) / 1000.0;
    else        st.tcHoldSec = tcThrottle ? std::max(0.0, st.tcHoldSec - dtSec) : 0.0;
    const double tcOn = (tcFlag || st.tcHoldSec > 0.0) ? 1.0 : 0.0;

    // Engine: firing rate from rpm and the engine description; throttle is
    // the load; the limiter flag drives the cut gate. A Test with no sim
    // runs a sweep (below).
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
            st.engineHoldHz = fireHz;
        }
        // No stream, or no rpm in it: unknown, not an engine stopping. It
        // keeps turning at its last rate with the level at 0, so the release
        // ramp fades it (the sim reporting 0 rpm while live is a stall and
        // kicks; every pause used to, the SimHub plugin goes quiet then).
        else if (!live || !v.have[NcxValues::Rpm])
            fireHz = st.engineHoldHz;
        const double previewLeft = st.previewSec[static_cast<int>(FxType::RpmVibe)];   // before previewOr counts it down
        const double lv = previewOr(FxType::RpmVibe, level);
        // A Test with no live rpm runs its own sweep, never the rate held
        // from the last stream (the sim quit at 7000 rpm previewed at 7000):
        // the canned idle (the idle rock), then the revs climbing to the
        // redline under a rising load, ending on the cut. The cut comes in
        // through the pit limiter's input (it cuts like the rev limiter but
        // never snaps the redline), and the whole sweep is driven as a
        // preview, so the engine learns nothing from it. With live rpm the
        // Test plays the live engine, as before.
        bool previewing = false, previewCut = false;
        if (lv > 0.0 && level <= 0.0 && previewLeft > 0.0)
        {
            const double top  = std::max(kPreviewIdleRpm, (e.maxRpm > 0.0) ? e.maxRpm : L.learnedMaxRpm());
            const double t    = kPreviewSec - previewLeft;
            const double span = kPreviewSec - kPreviewIdleSec - kPreviewTopSec;
            const double x    = (span > 0.0) ? std::max(0.0, std::min(1.0, (t - kPreviewIdleSec) / span)) : 1.0;
            fireHz     = (kPreviewIdleRpm + (top - kPreviewIdleRpm) * x) / 60.0 * perRev;
            load       = kPreviewLoadFrom + (1.0 - kPreviewLoadFrom) * x;
            previewing = true;
            previewCut = t >= kPreviewSec - kPreviewTopSec;
        }
        // Live rpm too low for a firing every two seconds (a single cylinder
        // below 60 rpm, starting or stalling): the canned idle stands in, as
        // it always has. Only a running Test with no live rpm sweeps.
        else if (lv > 0.0 && fireHz < 0.5)
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
        L.driveEngine(lv, fireHz, load, lim, boost, pitLim || previewCut, inGearOverrun, tcCut, previewing);
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
        // The slip's load weight only takes away: the loaded tyre of each
        // axle counts as 1, the lighter one by its share of it (weighting the
        // loaded tyre up played Assetto Corsa, which sends loads, about 1.4
        // times stronger than a sim that does not). The Wheels tile's load
        // share is against the mean of all four.
        double loadW[WHEEL_COUNT] = { 1.0, 1.0, 1.0, 1.0 };
        double loadShare[WHEEL_COUNT] = { 1.0, 1.0, 1.0, 1.0 };
        if (haveW(NcxValues::LoadFL))
        {
            double load[WHEEL_COUNT], mean = 0.0;
            for (int w = 0; w < WHEEL_COUNT; ++w) mean += (load[w] = std::max(0.0, v.val[NcxValues::LoadFL + w]));
            mean /= WHEEL_COUNT;
            for (int w = 0; w < WHEEL_COUNT; ++w)
            {
                const double axleMax = std::max(load[w], load[w ^ 1]);
                if (axleMax > 0.0) loadW[w] = load[w] / axleMax;
                if (mean > 0.0) loadShare[w] = std::min(kLoadWeightMax, load[w] / mean);
            }
        }
        const double speed = (live && v.have[NcxValues::SpeedKmh]) ? std::max(0.0, v.val[NcxValues::SpeedKmh]) : 0.0;
        // Slip means nothing at a standstill (a stationary car reports
        // slip angles and skid magnitudes that are noise or stale).
        const bool moving = speed > kSlipMinSpeedKmh;
        double lonPeak = kLonOnsetRatio + 0.05;
        double lockW[WHEEL_COUNT] = {}, spinW[WHEEL_COUNT] = {};   // the Longitudinal tile's own, for the grip budget

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
            // learned. The first value is the median of samples taken over
            // half a second of that in all, however short the moments (it
            // waited for half a second without a break, and a 312T driven
            // hard in Assetto Corsa went minutes with no longitudinal slip
            // at all), taken only once the samples agree, so a spin with
            // the pedals off cannot teach it (it did, and the rears then
            // read as spinning for seconds), and with both axles' wheels
            // within kRollSeedAxleAgree. Once learned, a sample far off it
            // is ignored for a while (kRollGate), and engine braking teaches
            // the driven axle nothing (its wheels turn slower than the road).
            bool cruising = haveWs && speed > kRollLearnMinKmh
                && (!v.have[NcxValues::BrakePct]    || v.val[NcxValues::BrakePct]    < kRollLearnMaxBrk)
                && (!v.have[NcxValues::ThrottlePct] || v.val[NcxValues::ThrottlePct] < kRollLearnMaxThr)
                && !(v.have[NcxValues::ClutchPct] && v.val[NcxValues::ClutchPct] > kRollClutchLo
                                                  && v.val[NcxValues::ClutchPct] < kRollClutchHi);
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
            // known (learned, or fixed); before it, past a wider limit (a
            // slide moved the rears 1.3 % before the baseline was learned).
            if (cruising && st.slipSrc == 3)
            {
                const bool baseKnown = st.latBaseSec >= kLatBaseLearnSec || st.latBaseMode == 2;
                for (int w = 0; w < WHEEL_COUNT; ++w)
                    if (baseKnown ? st.latRaw[w] - st.latBase[w] > kRollMaxSlipDeg : st.latRaw[w] > kRollPreBaseSlipDeg)
                        cruising = false;
            }
            // Engine braking: the driven axle (the one found running ahead
            // under full throttle) turns slower than the road; it neither
            // learns nor counts as disagreeing.
            const bool overrun = v.have[NcxValues::ThrottlePct] && v.val[NcxValues::ThrottlePct] < kRollOverrunThr
                              && !(v.have[NcxValues::ClutchPct] && v.val[NcxValues::ClutchPct] >= kRollClutchHi)
                              && !(v.have[NcxValues::Gear] && std::fabs(v.val[NcxValues::Gear]) < 0.5);
            const auto skipAxle = [&](int w)
            {
                const int ax = w < 2 ? 0 : 1;
                return overrun && st.drivenSec[ax] > kDrivenSec && st.drivenSec[ax] > st.drivenSec[ax ^ 1];
            };
            // The gate: a wheel off what was learned by more than kRollGate.
            // Only one-sided, steady disagreement counts towards the escape
            // (a wrong first value, a real change): an axle's two wheels off
            // opposite ways is a corner and a wheel changing its way is
            // noise, either starts it over (corners lap after lap added up
            // and taught their split); moments the wheels agree count it
            // back down (a value just outside the gate, its samples now in,
            // now out, still gets there).
            // (A wheel left out for engine braking keeps the way it was
            // off: the others agreeing says nothing about it.)
            const bool calm = cruising;   // fit to learn from, the gate aside (the driven axle's lead reads it)
            if (cruising)
            {
                int sign[WHEEL_COUNT] = {};
                bool off = false, flip = false, held = false;
                for (int w = 0; w < WHEEL_COUNT; ++w)
                {
                    if (!st.rollKnown[w]) continue;
                    if (skipAxle(w)) { sign[w] = st.rollOffSign[w]; held = held || sign[w] != 0; continue; }
                    const double d = kw[w] / st.rollK[w] - 1.0;
                    if (std::fabs(d) > kRollGate) { sign[w] = d > 0.0 ? 1 : -1; off = true; }
                    if (sign[w] != 0 && st.rollOffSign[w] != 0 && sign[w] != st.rollOffSign[w]) flip = true;
                }
                const bool corner = sign[WheelFL] * sign[WheelFR] < 0 || sign[WheelRL] * sign[WheelRR] < 0;
                if (corner || flip) st.rollOffSec = 0.0;
                else if (off)   st.rollOffSec = std::min(kRollGateEscapeSec, st.rollOffSec + dtSec);
                else if (!held) st.rollOffSec = std::max(0.0, st.rollOffSec - dtSec);   // agreeing counts it back down
                for (int w = 0; w < WHEEL_COUNT; ++w) st.rollOffSign[w] = sign[w];
                // Once escaped, it stays open until every wheel looked at is
                // well inside the gate (a value hovering just outside it was
                // refused again and stalled a wrong first value for minutes).
                // A corner never teaches.
                if (st.rollOffSec >= kRollGateEscapeSec) st.rollOpen = true;
                if (st.rollOpen)
                {
                    bool settled = true;
                    for (int w = 0; w < WHEEL_COUNT; ++w)
                        if (st.rollKnown[w] && !skipAxle(w) && std::fabs(kw[w] / st.rollK[w] - 1.0) > kRollSettle) settled = false;
                    if (settled) { st.rollOpen = false; st.rollOffSec = 0.0; }
                }
                if (corner || (off && !st.rollOpen)) cruising = false;
            }
            if (cruising)
            {
                bool anyUnknown = false;
                for (int w = 0; w < WHEEL_COUNT; ++w) anyUnknown = anyUnknown || !st.rollKnown[w];
                if (anyUnknown) st.rollCruiseSec += dtSec;
                // A car with left/right tyre stagger never has its axles
                // within 1 %: after a while the usual 4 % will do.
                const double seedAgree = (st.rollCruiseSec < kRollSeedWidenSec) ? kRollSeedAxleAgree : kRollAxleAgree;
                const bool axlesTogether = std::fabs(kw[WheelFL] / kw[WheelFR] - 1.0) <= seedAgree
                                        && std::fabs(kw[WheelRL] / kw[WheelRR] - 1.0) <= seedAgree;
                if (axlesTogether) st.rollSeedSec += dtSec;
                if (anyUnknown && axlesTogether && st.rollSeedSec >= st.rollSeedNextSec)
                {
                    const int slot = st.rollSeedN % LawsState::ROLL_SEED_SLOTS;
                    for (int w = 0; w < WHEEL_COUNT; ++w) st.rollSeedK[w][slot] = kw[w];
                    ++st.rollSeedN;
                    st.rollSeedNextSec = st.rollSeedSec + kRollSeedEverySec;
                    if (st.rollSeedSec >= kRollSeedSec)
                    {
                        const int n = std::min(st.rollSeedN, LawsState::ROLL_SEED_SLOTS);
                        for (int w = 0; w < WHEEL_COUNT; ++w)
                        {
                            if (st.rollKnown[w]) continue;
                            double s[LawsState::ROLL_SEED_SLOTS];
                            for (int i = 0; i < n; ++i) s[i] = st.rollSeedK[w][i];
                            std::nth_element(s, s + n / 2, s + n);
                            const double med = s[n / 2];
                            int agree = 0;
                            for (int i = 0; i < n; ++i) if (std::fabs(st.rollSeedK[w][i] / med - 1.0) <= kRollSeedAgree) ++agree;
                            if (agree >= kRollSeedShare * n) { st.rollK[w] = med; st.rollKnown[w] = true; }
                        }
                    }
                }
                const double a = std::min(1.0, dtSec / kRollLearnSec);
                for (int w = 0; w < WHEEL_COUNT; ++w)
                {
                    if (!st.rollKnown[w] || skipAxle(w)) continue;
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
                lockW[w] = lock; spinW[w] = spin;
            }
            // Which axle the engine drives: the one that runs further ahead
            // of the other under full throttle than it does cruising, by a
            // tyre's working slip (further is wheelspin or a spin, which
            // says nothing). Against the cruising lead, not zero: a rolling
            // factor learned wrong (a Sigma's fronts seeded 6 % off in a
            // pit-exit curve) read as that axle driving, and it then never
            // learned on a lift. The rolling factor's engine-braking rule
            // above reads it.
            if (live && ratioOk[WheelFL] && ratioOk[WheelFR] && ratioOk[WheelRL] && ratioOk[WheelRR])
            {
                const double lead = 0.5 * (ratioW[WheelRL] + ratioW[WheelRR]) - 0.5 * (ratioW[WheelFL] + ratioW[WheelFR]);
                if (calm)
                {
                    st.leadCruise = (st.leadCruiseSec > 0.0) ? st.leadCruise + std::min(1.0, dtSec / kDrivenLeadSec) * (lead - st.leadCruise) : lead;
                    st.leadCruiseSec += dtSec;
                }
                if (st.leadCruiseSec >= kDrivenLeadSec && v.have[NcxValues::ThrottlePct] && v.val[NcxValues::ThrottlePct] > kDrivenThrottle
                    && speed > kDrivenMinKmh && speed < kDrivenMaxKmh)
                {
                    const double d = lead - st.leadCruise;
                    if (d > kDrivenAhead && d < kLonOnsetRatio) st.drivenSec[1] += dtSec;
                    if (-d > kDrivenAhead && -d < kLonOnsetRatio) st.drivenSec[0] += dtSec;
                }
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
            // slip ratio plus its straight-line slip (c, learned on the
            // straights) x the road speed, what is left is sideways, and as
            // an angle (asin of it over the road speed) it plays through the
            // slip angle path, less what is left of each wheel's straight-
            // line angle (toe), learned on the straights too. As a plain
            // combined slip it grew with road speed: 2.5 deg's worth on
            // every straight, the scrub strongest at 250 km/h. Taking c off
            // as an angle instead read the fronts at zero under braking and
            // a corner about c too low, and a driven wheel under power as a
            // false angle.
            const bool fromSpeed = !perWheel && haveW(NcxValues::SlipSpeedFL) && v.have[NcxValues::SpeedKmh];
            // The baseline (the tile's): learned on the straights (0),
            // learned once per car and then held (1), or fixed at base deg F
            // and R (2). Leaving fixed starts the learning over: the fixed
            // values were never learned.
            const int baseMode = std::max(0, std::min(2, static_cast<int>(sp.baseMode + 0.5)));
            if (st.latBaseMode == 2 && baseMode != 2) relearnLatBase(st);
            st.latBaseMode = baseMode;
            // Fixed takes the set angles off and learns nothing, the
            // straight-line slip along the road included.
            if (baseMode == 2)
                for (int w = 0; w < WHEEL_COUNT; ++w)
                {
                    st.latBase[w] = std::max(0.0, std::min(kLatBaseMaxDeg, (w == WheelFL || w == WheelFR) ? sp.baseDegF : sp.baseDegR));
                    st.latCreep[w] = 0.0;
                }
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
                // Fixed: no learning.
                if (baseMode == 2) straight = false;
                // Learned once and held: no learning.
                if (baseMode == 1 && st.latBaseSec >= kLatBaseLearnSec) straight = false;
                if (straight) st.latBaseSec += dtSec;
                const bool   learned = st.latBaseSec >= kLatBaseLearnSec;
                const double up    = std::min(1.0, dtSec / (learned ? kLatBaseUpSec : kLatBaseSec));
                const double down  = std::min(1.0, dtSec / kLatBaseDownSec);
                const double creep = std::min(1.0, dtSec / (learned ? kLatCreepSec : kLatBaseSec));
                // A sender with slip ratios or wheel speeds whose ratio is not
                // known yet (the rolling factor still learning after a car
                // change): that wheel's angle is unknown, not all sideways
                // (a race start's wheelspin read as a full slide); so is one
                // spinning or locked past kLatRatioGate.
                const bool alongSent = haveW(NcxValues::SlipRatioFL) || haveW(NcxValues::WheelSpeedFL);
                for (int w = 0; w < WHEEL_COUNT; ++w)
                {
                    if (alongSent && !ratioOk[w]) { st.latRaw[w] = 0.0; continue; }
                    if (ratioOk[w] && std::fabs(ratioW[w]) > kLatRatioGate) { st.latRaw[w] = 0.0; continue; }
                    const double ss = std::fabs(v.val[NcxValues::SlipSpeedFL + w]);
                    const double r  = ratioOk[w] ? ratioW[w] : 0.0;
                    // c and the baseline learn only while this wheel turns
                    // with the road (no torque through it: under power the
                    // rear baseline started at 2.3 to 2.9 deg against 0.5),
                    // on the car's shared straight-line clock. (A clock per
                    // wheel left a rear-driven car's rears on the fast rates
                    // for minutes, their along slip drifting high and their
                    // corners reading low, and taught a Mini's fronts worse,
                    // on the recordings.)
                    const bool rolling = straight && std::fabs(r) < kLatCreepRatio;
                    if (rolling)
                        st.latCreep[w] += creep * (std::max(kLatCreepMin, std::min(kLatCreepMax, ss / vms - r)) - st.latCreep[w]);
                    const double along = std::fabs(r + st.latCreep[w]) * vms;
                    const double side  = std::sqrt(std::max(0.0, ss * ss - along * along));
                    st.latRaw[w] = std::asin(std::min(1.0, side / vms)) * 180.0 / wavesynth::kPi;
                    if (rolling)
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
            // An angle from a slip speed fades in with road speed (below
            // walking-pace speeds a slow car's sideways part is mostly
            // noise: a GTP played 75 % of its time at 5 to 30 km/h as a slide).
            const double latFade = (src == 3)
                ? std::max(0.0, std::min(1.0, (speed - kLatFadeFromKmh) / (kLatFadeFullKmh - kLatFadeFromKmh))) : 1.0;
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
                sev = std::min(1.0, sev * loadW[w]) * ground[w] * latFade;
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
                // or pure along gives the same severity as the tiles alone,
                // and it never plays less than the tiles alone (with the
                // onset at 0 any angle read as the whole budget, and lock
                // and spin went nearly silent).
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
                        lat = std::min(1.0, s * (aN / u) * loadW[w]) * ground[w] * latFade;
                        lon = std::min(1.0, s * (kN / u) * loadW[w]) * ground[w];
                    }
                    lat = std::max(lat, sev);
                    const double lock = std::max(ratioW[w] < 0.0 ? lon : 0.0, lockW[w]);
                    const double spin = std::max(ratioW[w] > 0.0 ? lon : 0.0, spinW[w]);
                    L.driveSlip(FxType::Skid, w, front ? lat : 0.0, front ? 0.0 : lat);
                    L.driveSlip(FxType::Lockup, w, lock, spin);
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
                        screech = std::min(1.0, l * l * (3.0 - 2.0 * l) * loadW[w]) * ground[w] * latFade;
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
            // Fresh tyres on a reset (the tile's freshOnReset), from the
            // sim's own signs where it sends them (protocol 1.9):
            //  a. the pit flag rising after the car was out, within a second
            //     of the road speed jumping to a standstill or of the stream
            //     coming back after a gap (the pause menu, from speed or from
            //     a standstill): put back in the pits or the garage. Also the
            //     flag sent for the first time, set, within a second of a
            //     jump. Driving in keeps the tyres, and so does a stop that
            //     does not change them;
            //  b. the lap number going back and holding: a restart (the first
            //     one sent only sets where it starts; 0 or less is no lap);
            //  c. a tyre's wear falling and holding: new tyres at a stop (the
            //     highest is kept across a gap: wear only grows in a stint);
            //  d. no pit flag sent: the jump to a standstill between two live
            //     samples, the car then standing (a stream that went away and
            //     came back is not a jump).
            // Live samples only: one not sent is unknown, never 0.
            {
                st.pitRoseAgo += dtSec; st.resumeAgo += dtSec; st.pitFirstAgo += dtSec;
                if (live && !st.wasLive) st.resumeAgo = 0.0;
                st.wasLive = live;
                st.freshGuardSec = std::max(0.0, st.freshGuardSec - dtSec);
                bool reset = false;
                const bool pitSent = live && v.have[NcxValues::InPit];
                if (pitSent)
                {
                    const bool inBox = v.val[NcxValues::InPit] > 0.5;
                    if (st.pitSeen && inBox && !st.pitWas && st.onTrackSec >= kFreshOutSec) st.pitRoseAgo = 0.0;
                    // The flag sent for the first time, already set: a sender
                    // that sends it only once it has seen it set (the return
                    // itself, from a race started on the grid). Counted with
                    // a jump only: a session starting in the garage is not a
                    // return.
                    if (!st.pitSeen && inBox) st.pitFirstAgo = 0.0;
                    st.onTrackSec = inBox ? 0.0 : st.onTrackSec + dtSec;
                    st.pitWas = inBox; st.pitSeen = true;
                }
                if (st.pitRoseAgo <= kFreshPairSec && std::min(st.jumpAgo, st.resumeAgo) <= kFreshPairSec)
                {
                    reset = true;                                                // a
                    st.pitRoseAgo = st.jumpAgo = st.resumeAgo = 1e9;
                }
                if (st.pitFirstAgo <= kFreshPairSec && st.jumpAgo <= kFreshPairSec)
                {
                    reset = true;                                                // a, first flag
                    st.pitFirstAgo = st.jumpAgo = 1e9;
                }
                if (live && v.have[NcxValues::Lap])
                {
                    const long lap = std::lround(v.val[NcxValues::Lap]);
                    if (lap > 0)
                    {
                        if (!st.lapSeen) { st.lastLap = lap; st.lapSeen = true; }
                        else if (lap == st.lastLap) st.lapCandSec = 0.0;
                        else
                        {
                            if (lap != st.lapCand) { st.lapCand = lap; st.lapCandSec = 0.0; }
                            if ((st.lapCandSec += dtSec) >= kFreshConfirmSec)
                            {
                                if (lap < st.lastLap) reset = true;              // b
                                st.lastLap = lap; st.lapCandSec = 0.0;
                            }
                        }
                    }
                }
                // The wear as a group (all four tyres or none, as the protocol
                // has every per-wheel group).
                bool wearSent = live;
                for (int w = 0; w < WHEEL_COUNT; ++w) wearSent = wearSent && v.have[NcxValues::TyreWearFL + w];
                for (int w = 0; w < WHEEL_COUNT; ++w)
                {
                    if (!wearSent) continue;
                    const double x = v.val[NcxValues::TyreWearFL + w];
                    if (!st.wearSeen[w]) { st.wearHi[w] = x; st.wearHiSec[w] = st.wearLowSec[w] = 0.0; st.wearSeen[w] = true; }
                    else if (st.wearHi[w] - x > kFreshWearDrop)
                    {
                        if ((st.wearLowSec[w] += dtSec) >= kFreshConfirmSec)
                        {
                            reset = true;                                        // c
                            st.wearHi[w] = x; st.wearHiSec[w] = st.wearLowSec[w] = 0.0;
                        }
                    }
                    else
                    {
                        st.wearLowSec[w] = 0.0;
                        if (x >= st.wearHi[w] || st.wearHiSec[w] >= kFreshWearSec) { st.wearHi[w] = x; st.wearHiSec[w] = 0.0; }
                        else st.wearHiSec[w] += dtSec;
                    }
                }
                if (live && !pitSent && st.jumpAgo == 0.0) { st.jumpHolding = true; st.jumpHoldSec = 0.0; }
                else if (st.jumpHolding)
                {
                    // The raw speed: the jump itself leaves it untrusted in v
                    // for the glitch hold.
                    const bool standing = live && !pitSent && in.have[NcxValues::SpeedKmh]
                                          && std::isfinite(in.val[NcxValues::SpeedKmh]) && in.val[NcxValues::SpeedKmh] < kJumpToKmh;
                    if (!standing) st.jumpHolding = false;
                    else if ((st.jumpHoldSec += dtSec) >= kFreshConfirmSec)
                    {
                        reset = true;                                            // d
                        st.jumpHolding = false;
                    }
                }
                if (reset && st.freshGuardSec <= 0.0 && L.wheelsParams().freshOnReset >= 0.5)
                {
                    L.wheelsFreshTyres();
                    st.freshGuardSec = kFreshGuardSec;
                }
            }
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
                    L.driveWheelLock(w, carMs, ratioOk[w] ? ratioW[w] : 0.0, loadShare[w]);
                }
                L.driveWheelsBrake(v.have[NcxValues::BrakePct] ? v.val[NcxValues::BrakePct] / 100.0 : 0.0);
            }
            // The brake still goes on while a speed jump leaves the speed
            // untrusted: the pads' grab must not start over afterwards.
            else if (live && v.have[NcxValues::BrakePct])
                L.driveWheelsBrake(v.val[NcxValues::BrakePct] / 100.0);
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
    // braking), never dumps; a tyre in the air is not counted. From wheels
    // "with the flag": the wheels fill the gaps of a flag that has shown
    // this car's ABS working, or stand in for a sender with no flag; a flag
    // that never comes on is ABS switched off (Assetto Corsa, aid off: a
    // driver pumping the pedal through two lock-ups read as ABS). "On its
    // own": the wheels alone, for a game whose flag is always 0 (the SimHub
    // plugin sends 0 where SimHub has none).
    {
        const PulseParams& pp = L.pulseParams(FxType::AbsPulse);
        const bool absFlag = flag(NcxValues::AbsActive) > 0.5;
        if (absFlag) { st.absHoldSec = std::max(0.0, pp.holdMs) / 1000.0; st.absFlagSeen = true; }
        else         st.absHoldSec = std::max(0.0, st.absHoldSec - dtSec);
        const bool wheelsTrusted = !(live && v.have[NcxValues::AbsActive]) || st.absFlagSeen;
        // No ABS at all with the tile's amp at 0, through its slip link
        // either (a car with no ABS has its amp at 0: the link went on
        // judder-locking the tyres through half of every stop); and, with
        // off < 5 km/h, none below walking pace (a sim's flag held on at a
        // standstill with the brake down played ABS in the pits).
        // (The speed as sent: a low one is safe to trust for this even while
        // a jump has left it untrusted for everything else.)
        const bool fitted  = L.fxParams(FxType::AbsPulse).ampPct > 0.0;
        const bool tooSlow = pp.lowOff >= 0.5 && live && in.have[NcxValues::SpeedKmh] && std::isfinite(in.val[NcxValues::SpeedKmh])
                             && in.val[NcxValues::SpeedKmh] < kAbsLowOffKmh;
        const bool braking = fitted && !tooSlow && live && v.have[NcxValues::BrakePct] && v.val[NcxValues::BrakePct] > kAbsMinBrakePct;
        const bool rolling = live && v.have[NcxValues::SpeedKmh] && v.val[NcxValues::SpeedKmh] > kSlipMinSpeedKmh;
        st.absWheelHoldSec = std::max(0.0, st.absWheelHoldSec - dtSec);
        for (int w = 0; w < WHEEL_COUNT; ++w)
        {
            st.absSince[w] += dtSec;
            if (!(pp.fromWheels >= 0.5 && (wheelsTrusted || pp.fromWheels >= 1.5) && braking && rolling && ratioOk[w] && ground[w] > 0.5))
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
        const double absLevel = previewOr(FxType::AbsPulse, level);   // (previewOr runs the Test timer: call it either way)
        L.driveFx(FxType::AbsPulse, fitted ? absLevel : 0.0, 0.0);      // amp 0: not even a Test drives the slip link
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
            L.driveRoadSurface(w, surf ? surfaceClass(v.val[NcxValues::SurfaceFL + w]) : SurfTarmac);
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
    // road height under each tyre, a kerb's edge or ribs: the axle's
    // left-right height difference jumping by half of detect mm or more
    // within a few hundredths of a second (full at detect mm), the rumble
    // fading a quarter second after the last jump, on the side the first
    // jump rose (to the drop off it). The road's own camber and banking change
    // over seconds and never count (measured against a slowly learned usual
    // difference, they read as a kerb most of a lap at Imola). A Test
    // preview runs the strip at 80 km/h, on and off.
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
                lvl[w] = (surfaceClass(v.val[NcxValues::SurfaceFL + w]) == SurfKerb) ? 1.0 : 0.0;
        }
        else
        {
            // The game's kerb channel (Competizione, EVO) on every wheel,
            // and with detect mm set and the road heights arriving, a tyre
            // jumping against its axle mate (a kerb's edge or ribs): the stronger of the two. A
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
                const double half  = 0.5 * kp.detectMm;
                const double fade  = std::exp(-dtSec / kKerbHoldSec);
                st.kerbClock += dtSec;
                for (int ax = 0; ax < 2; ++ax)
                {
                    const int l = ax ? WheelRL : WheelFL, r = ax ? WheelRR : WheelFR;
                    const double d = v.val[NcxValues::RoadHeightFL + l] - v.val[NcxValues::RoadHeightFL + r];
                    const int K = LawsState::KERB_HIST;
                    if (!st.kerbSeen) { st.kerbHistN[ax] = 0; st.kerbEnv[ax] = 0.0; st.kerbOn[ax] = false; st.kerbSide[ax] = 0;
                                        st.kerbQuietSec[ax] = 0.0; st.kerbLastSide[ax] = 0; st.kerbSinceOff[ax] = 1e9; }
                    // A new sim sample (the held value changed) joins the
                    // history; the jump: the largest change from any sample
                    // of the last kKerbJumpSec.
                    const int last = (st.kerbHistHead[ax] + K - 1) % K;
                    if (st.kerbHistN[ax] == 0 || st.kerbHistD[ax][last] != d)
                    {
                        st.kerbHistT[ax][st.kerbHistHead[ax]] = st.kerbClock;
                        st.kerbHistD[ax][st.kerbHistHead[ax]] = d;
                        st.kerbHistHead[ax] = (st.kerbHistHead[ax] + 1) % K;
                        st.kerbHistN[ax] = std::min(K, st.kerbHistN[ax] + 1);
                    }
                    // A past sample counts while it was still the value
                    // held some time in the window: the sample that replaced
                    // it arrived inside it.
                    double jump = 0.0, from = d;
                    for (int i = 2; i <= st.kerbHistN[ax]; ++i)
                    {
                        const int succ = (st.kerbHistHead[ax] + K - (i - 1)) % K;
                        if (st.kerbClock - st.kerbHistT[ax][succ] > kKerbJumpSec) break;
                        const double dk = st.kerbHistD[ax][(st.kerbHistHead[ax] + K - i) % K];
                        if (std::fabs(d - dk) > jump) { jump = std::fabs(d - dk); from = dk; }
                    }
                    st.kerbEnv[ax] = std::max(jump, st.kerbEnv[ax] * fade);
                    const double k = std::max(0.0, std::min(1.0, (st.kerbEnv[ax] - half) / half));
                    // The kerb under way: its side is the first jump's (the
                    // left tyre rising against the right: left; a side
                    // learned from the camber lagged a corner's camber and
                    // put it on the wrong wheel), held until the axle is back
                    // level with where it was before, so the drop off a
                    // raised kerb rumbles the same wheel.
                    // While on one, the axle swinging past where it began to
                    // the other side is a kerb under the other wheel (a
                    // chicane); quiet for kKerbQuietSec, it is over even if
                    // the camber has moved meanwhile.
                    if (st.kerbOn[ax] && jump > half)
                    {
                        const double off = d - st.kerbRef[ax];
                        if ((st.kerbSide[ax] == 1 && off < -half) || (st.kerbSide[ax] == 2 && off > half))
                        { st.kerbSide[ax] = (off > 0.0) ? 1 : 2; st.kerbRef[ax] = from; st.kerbOnSec[ax] = 0.0; }
                    }
                    // A new jump: the drop off the last kerb when it comes
                    // soon after and swings back towards where that kerb
                    // began (the same wheel); else a kerb of its own, on the
                    // side the jump rose.
                    st.kerbSinceOff[ax] += dtSec;
                    if (!st.kerbOn[ax] && jump > half)
                    {
                        const bool dropOff = st.kerbLastSide[ax] != 0 && st.kerbSinceOff[ax] < kKerbDropSec
                                             && std::fabs(from - st.kerbLastRef[ax]) > half
                                             && std::fabs(d - st.kerbLastRef[ax]) < std::fabs(from - st.kerbLastRef[ax]);
                        st.kerbOn[ax] = true; st.kerbOnSec[ax] = 0.0;
                        st.kerbRef[ax]  = dropOff ? st.kerbLastRef[ax] : from;
                        st.kerbSide[ax] = dropOff ? st.kerbLastSide[ax] : ((d > from) ? 1 : 2);
                    }
                    if (st.kerbOn[ax])
                    {
                        st.kerbOnSec[ax] += dtSec;
                        st.kerbQuietSec[ax] = (k <= 0.0) ? st.kerbQuietSec[ax] + dtSec : 0.0;
                        if ((k <= 0.0 && std::fabs(d - st.kerbRef[ax]) <= half) || st.kerbQuietSec[ax] > kKerbQuietSec
                            || st.kerbOnSec[ax] > kKerbEventMaxSec)
                        {
                            st.kerbLastRef[ax] = st.kerbRef[ax]; st.kerbLastSide[ax] = st.kerbSide[ax]; st.kerbSinceOff[ax] = 0.0;
                            st.kerbOn[ax] = false; st.kerbSide[ax] = 0; st.kerbQuietSec[ax] = 0.0;
                        }
                    }
                    if (k <= 0.0) continue;
                    if (st.kerbSide[ax] != 2) lvl[l] = std::max(lvl[l], k);
                    if (st.kerbSide[ax] != 1) lvl[r] = std::max(lvl[r], k);
                }
            }
        }
        st.kerbSeen = heights;
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
    // the model finds the sudden jumps, never with the car standing (a reset
    // or a spawn drops it onto the ground: the speed as sent, which a reset
    // leaves untrusted for everything else). A Test preview knocks FL, FR,
    // RL (a top-out), RR, then the whole car.
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
            if (in.have[NcxValues::SpeedKmh] && std::isfinite(in.val[NcxValues::SpeedKmh]) && in.val[NcxValues::SpeedKmh] < kImpactStillKmh)
                L.impactsStill();
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
        if (preview) { judder = 1.0; lug = 1.0; st.clutchBandSec = 0.0; st.previewSec[static_cast<int>(FxType::Driveline)] -= dtSec; }
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
                    st.clutchBandSec += dtSec;
                    // Window: full mid-band, tapering to the edges.
                    const double mid = 0.5 * (kClutchSlipLo + kClutchSlipHi), half = 0.5 * (kClutchSlipHi - kClutchSlipLo);
                    const double window = std::max(0.0, 1.0 - std::fabs(c - mid) / half);
                    double slip = 0.5;   // ratio unknown: assume some
                    const double speed = v.have[NcxValues::SpeedKmh] ? std::max(0.0, v.val[NcxValues::SpeedKmh]) : -1.0;
                    const bool launch = speed >= 0.0 && speed < kClutchLaunchKmh;
                    if (launch) slip = 1.0;
                    else if (speed >= 0.0 && ratios && g > 0 && g < MAX_GEARS && ratios->known[g])
                    {
                        const double expected = ratios->r[g] * speed;
                        slip = std::max(0.0, std::min(1.0, std::fabs(rpm - expected) / std::max(1.0, rpm)));
                    }
                    // Rolling, the pedal held in the band a moment: a shift's
                    // quick dip (an auto-clutch, a heel-and-toe) is no slip to
                    // judder over (it shuddered on every shift at speed).
                    if (launch || st.clutchBandSec >= kJudderHoldSec)
                        judder = window * slip * std::max(0.15, load);
                }
                else st.clutchBandSec = 0.0;
            }
            else st.clutchBandSec = 0.0;
            // (An electric motor has no lugging: full torque from a standstill.)
            const bool electric = static_cast<int>(L.engineParams().layout + 0.5) == 5;
            if (inGear && !electric && v.have[NcxValues::ThrottlePct] && v.val[NcxValues::ThrottlePct] >= kLugThrottle && rpm > kLugMinRpm)
            {
                const double maxRpm = (L.engineParams().maxRpm > 0.0) ? L.engineParams().maxRpm : L.learnedMaxRpm();
                const double x = rpm / std::max(1000.0, maxRpm);
                lug = std::max(0.0, std::min(1.0, (kLugFromX - x) / (kLugFromX - kLugFullX))) * load;
            }
        }
        else st.clutchBandSec = 0.0;   // no stream, a stalled engine: a later dip starts its wait afresh
        L.driveDriveline(judder, lug);

        // The differential (diff x 0 = off, an open diff). A clutch-type
        // limited slip clunks when the driven wheels are made to turn apart
        // at walking pace: full lock out of the pits, turning round after an
        // off, a hairpin at a crawl (full below kDiffFullKmh, gone by
        // kDiffGoneKmh), harder under power (off it: its preload). On a
        // racing lap a healthy diff is silent: its plates are clamped or
        // slip smoothly, and the balance it gives reaches the driver through
        // the motion and the wheel. Only the corner's own geometry counts:
        // the free axle's split plus kDiffSpinAllow, so a spinning inside
        // wheel or one skipping over a kerb is no diff, and a free axle past
        // kDiffMaxGeoSplit is the car rotating. That needs the wheel speeds
        // (a sim's own slip ratios leave the geometry out): without them it
        // stays silent. A spool lets the driven wheels turn only as one, so
        // the inside tyre scrubs and hops instead, by the free axle's split.
        // The clunks come evenly, faster the faster the wheels turn apart.
        {
            const DrivelineParams& dp = L.drivelineParams();
            double dl = 0.0, rate = 0.0;
            // Not under braking (a lock-up or ABS working is no diff), and no
            // drive through it in neutral or with the clutch down: the
            // preload alone.
            const bool braking = v.have[NcxValues::BrakePct] && v.val[NcxValues::BrakePct] > kAbsMinBrakePct;
            if (preview && dp.diff > 0.0) { dl = 1.0; rate = dp.diffHz; }
            else if (dp.diff > 0.0 && live && v.have[NcxValues::SpeedKmh] && !braking)
            {
                const double kmh   = std::max(0.0, v.val[NcxValues::SpeedKmh]);
                const bool   spool = dp.diffType >= 0.5;
                const int    driven = static_cast<int>(dp.driven + 0.5);   // 0 rear, 1 front, 2 all
                // An axle's two wheels apart, from the wheel speeds (the
                // corner's geometry is in them).
                const auto split = [&](int ax, double& s) -> bool
                {
                    const int l = ax ? WheelRL : WheelFL, r = ax ? WheelRR : WheelFR;
                    if (!v.have[NcxValues::WheelSpeedFL + l] || !v.have[NcxValues::WheelSpeedFL + r]) return false;
                    // The rolling factors only as a pair (a tyre set's stagger
                    // is in them): one known and one not mixed a road speed
                    // with the sender's raw unit, a 10 % split on a straight.
                    const bool both = st.rollKnown[l] && st.rollKnown[r];
                    const double a = std::fabs(v.val[NcxValues::WheelSpeedFL + l]) * (both ? st.rollK[l] : 1.0);
                    const double b = std::fabs(v.val[NcxValues::WheelSpeedFL + r]) * (both ? st.rollK[r] : 1.0);
                    if (a + b < 1e-6) return false;
                    s = std::fabs(a - b) / (0.5 * (a + b));
                    // A wheel locking is no diff.
                    if ((ratioOk[l] && ratioW[l] < -kLonOnsetRatio) || (ratioOk[r] && ratioW[r] < -kLonOnsetRatio)) return false;
                    s *= std::min(ground[l], ground[r]);
                    return true;
                };
                // s: how far apart the corner turns the diff's wheels; geo:
                // the free axle's split (the corner, or the car rotating).
                double s = 0.0, geo = 0.0; bool ok = false;
                double front = 0.0, rear = 0.0;
                const bool okF = split(0, front), okR = split(1, rear);
                if (!spool && driven <= 1)
                {
                    const double dSplit = driven == 0 ? rear : front;
                    geo = driven == 0 ? front : rear;
                    ok  = okF && okR;
                    s   = std::min(dSplit, geo + kDiffSpinAllow);
                }
                else if (!spool)   // all driven: the axle further apart is held to the other's corner
                {
                    geo = std::min(front, rear);
                    ok  = okF && okR;
                    s   = std::min(std::max(front, rear), geo + kDiffSpinAllow);
                }
                else if (driven <= 1)   // a spool: the free axle says how tight
                {
                    geo = s = driven == 0 ? front : rear;
                    ok  = driven == 0 ? okF : okR;
                }
                if (ok && kmh >= kDiffMinKmh && geo <= kDiffMaxGeoSplit)
                {
                    const double full   = spool ? kDiffSpoolFullKmh : kDiffFullKmh;
                    const double gone   = spool ? kDiffSpoolGoneKmh : kDiffGoneKmh;
                    const double from   = spool ? kDiffSpoolFromSplit : kDiffFromSplit;
                    const double to     = spool ? kDiffSpoolFullSplit : kDiffFullSplit;
                    const double speedK = std::max(0.0, std::min(1.0, (gone - kmh) / (gone - full)));
                    const double splitK = std::max(0.0, std::min(1.0, (s - from) / (to - from)));
                    const bool   drive  = !(v.have[NcxValues::Gear] && std::fabs(v.val[NcxValues::Gear]) < 0.5)
                                       && !(v.have[NcxValues::ClutchPct] && v.val[NcxValues::ClutchPct] >= kClutchSlipHi);
                    const double thr    = (drive && v.have[NcxValues::ThrottlePct]) ? std::max(0.0, std::min(1.0, v.val[NcxValues::ThrottlePct] / 100.0)) : 0.0;
                    const double torqueK = spool ? 1.0 : kDiffPreload + (1.0 - kDiffPreload) * thr;
                    dl   = speedK * splitK * torqueK;
                    // How fast the wheels turn apart (m/s) sets the rate.
                    const double apartMs = kmh / 3.6 * s;
                    rate = std::max(1.0, dp.diffHz) * std::max(kDiffRateMin, std::min(1.0, apartMs / kDiffSlipRefMs));
                }
            }
            L.driveDiff(dl, rate);
        }

        // Whine: straight-cut mesh following the input shaft, in gear with
        // the clutch driving; level from load with a floor (unloaded gears
        // still sing). The carrier is placed against the engine buzz by the
        // whine pitch choice and steps per gear.
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
                    const EngineParams& ep = L.engineParams();
                    const double rpm    = v.val[NcxValues::Rpm];
                    const double maxRpm = (ep.maxRpm > 0.0) ? ep.maxRpm : L.learnedMaxRpm();
                    const int    pitch  = std::max(0, std::min(3, static_cast<int>(dp.whinePitch + 0.5)));
                    const double gearX  = (std::min(6.0, static_cast<double>(std::abs(g))) - 1.0) / 5.0;
                    const double ratio  = kWhineLow[pitch] + (kWhineHigh[pitch] - kWhineLow[pitch]) * gearX;
                    // A pitch the loop cannot carry drops an octave, chosen
                    // from the gear's pitch at the redline (the revs now, past
                    // it), so it holds through the rev range: picked from the
                    // revs now it fell an octave mid-rev (above the buzz on a
                    // 500 Hz loop turned below it from 4700 rpm).
                    const double top = (1.0 / std::max(1e-4, dtSec)) / kWhineSamplesPerCycle;
                    const double ref = EngineModel::buzzHz(ep, std::max(rpm, maxRpm) / 60.0, maxRpm, dtSec) * ratio;
                    double octave = 1.0;
                    while (ref * octave > top) octave *= 0.5;
                    whz = EngineModel::buzzHz(ep, rpm / 60.0, maxRpm, dtSec) * ratio * octave;
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

    // Gear-shift thunk: a transient on every change of the gear the box is
    // in (up or down), once the new gear has held kGearSettleSec: a sim
    // reports a junk gear for a frame mid-shift (Automobilista 2 read 5, N,
    // 3, 4 for a 5-4 downshift: three knocks). With ignore neutral (the
    // Driveline tile's) the shift knocks once, as the box leaves its gear
    // (the N the sim shows mid-shift, 130 to 190 ms in Automobilista 2,
    // settled), and the gear it lands in within kNeutralHeldSec is the same
    // shift: no knock of its own (knocking there came 200 ms late), only a
    // dog box's engagement knock as the dogs go in, if it left under power
    // or goes in under power (a blipped downshift).
    // A neutral held longer is a real one: the gear after it knocks. Off,
    // going into neutral and into the next gear knock separately. A junk
    // gear frame (dropped above) leaves it where it was. Edge state seeds on
    // first sight and clears on staleness, so a returning stream never
    // fires a stale crossing.
    if (live && v.have[NcxValues::Gear])
    {
        const double g0 = v.val[NcxValues::Gear];
        const int g = static_cast<int>(g0 < 0.0 ? g0 - 0.5 : g0 + 0.5);
        if (!st.gearSeen) { st.lastGear = st.gearCand = g; st.gearCandSec = 0.0; st.shiftOpenSec = -1.0; st.dogAtLanding = false; }
        else if (g != st.gearCand) { st.gearCand = g; st.gearCandSec = 0.0; }
        else st.gearCandSec += dtSec;
        if (st.shiftOpenSec >= 0.0) st.shiftOpenSec += dtSec;
        const bool ignoreN = L.drivelineParams().ignoreN >= 0.5;
        if (st.gearSeen && st.gearCand != st.lastGear && st.gearCandSec >= kGearSettleSec)
        {
            // Shaped by the gearbox: synchro = a softer, longer clunk; dog =
            // a harder, shorter knock and, shifted under power, a second
            // knock when the dogs engage (a moment later, or as the gear goes
            // in after a neutral).
            const bool dog = L.drivelineParams().gearbox >= 0.5;
            const double thr = v.have[NcxValues::ThrottlePct] ? v.val[NcxValues::ThrottlePct] : 0.0;
            const bool departing = ignoreN && st.gearCand == 0;
            const bool landing   = ignoreN && st.lastGear == 0 && st.shiftOpenSec >= 0.0 && st.shiftOpenSec < kNeutralHeldSec;
            if (landing)
            {
                if (dog && (st.dogAtLanding || thr >= kDogPowerThrottle)) L.fire(EventType::GearShift, kDogKnockScale, kDogDur, kDogFreq);
            }
            else
            {
                if (dog) L.fire(EventType::GearShift, kDogScale, kDogDur, kDogFreq);
                else     L.fire(EventType::GearShift, kSynchroScale, kSynchroDur, kSynchroFreq);
                if (dog && thr >= kDogPowerThrottle && !departing) st.dogKnockIn = kDogKnockDelaySec;
            }
            st.dogAtLanding = departing && dog && thr >= kDogPowerThrottle;
            st.shiftOpenSec = departing ? 0.0 : -1.0;
            st.lastGear = st.gearCand;
        }
        st.gearSeen = true;
    }
    else if (!(live && gearDropped))
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
