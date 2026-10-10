// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// HapticsTypes - the plain types shared by the haptics layer, the engine
// model, the registry and the config. No behaviour here.
// ============================================================

namespace haptics {

static constexpr int MAX_EVENTS      = 8;   // concurrent transients
static constexpr int MAX_HAPTIC_AXES = 10;  // == MAX_DRIVES
static constexpr int MAX_ROUTES      = MAX_HAPTIC_AXES + 4;  // every axis plus a few shakers
static constexpr int MAX_SHAKER_OUT  = 8;    // shaker channels on the sound card

// Transients: one-shot bursts. GearShift is the sim-driven one (fired on a
// gear-channel change - the thunk of a shift ringing through the chassis).
enum class EventType { DetentClick = 0, GearShift = 1, COUNT };
static constexpr int EVENT_TYPE_COUNT = static_cast<int>(EventType::COUNT);

// Telemetry-driven CONTINUOUS effects. RpmVibe is the engine model; Lockup
// and Skid are the two per-wheel slip models (longitudinal and lateral);
// the rest are oscillators whose LEVEL (0..1) is driven per cycle by a law.
enum class FxType { RpmVibe = 0, AbsPulse = 1, Lockup = 2, Skid = 3, Road = 4,
                    Limiter = 5, TcPulse = 6, Kerb = 7, Driveline = 8, COUNT };
static constexpr int FX_TYPE_COUNT = static_cast<int>(FxType::COUNT);

// What an axis index is as a routing destination. Torque: the overlay is %
// of rated torque. Position: the overlay is (% x gain) with gain in mm at
// 100% amplitude, derated per effect to what the axis can follow.
enum class SinkKind { Torque = 0, Position = 1 };

// Which wheels a route carries, for the per-wheel effects (slip). All = the
// strongest wheel; Front/Rear = the strongest of that axle; a corner = that
// wheel only. Effects without wheels ignore it.
enum class Part { All = 0, Front, Rear, FL, FR, RL, RR, COUNT };
static constexpr int PART_COUNT = static_cast<int>(Part::COUNT);
inline const char* partKey(Part p)
{
    static const char* const k[PART_COUNT] = { "all", "front", "rear", "fl", "fr", "rl", "rr" };
    const int i = static_cast<int>(p);
    return (i >= 0 && i < PART_COUNT) ? k[i] : "all";
}

// One destination: explicit axis index, a gain multiplier and, for the
// per-wheel effects, which wheels it carries. gain 0 or axis -1 = unused.
// A route is one destination: an axis (index) OR a shaker channel (index),
// a gain, for the per-wheel effects which wheels it carries, and for a
// shaker the harmonic of the effect's carrier it plays (1 = as is, 2 =
// double: a 9 Hz belt effect lands on the shaker at 18 Hz, phase-locked).
// gain 0, or axis and shaker both -1, = unused.
struct Route
{
    int    axis   = -1;
    double gain   = 0.0;
    Part   part   = Part::All;
    int    shaker = -1;
    int    harm   = 1;
};

// The two per-wheel slip tiles share one shape: two components (A on some
// wheels, B on others), each with its own mix and carrier. Lateral: A =
// front scrub, B = rear slide. Longitudinal: A = lock judder, B = spin
// tramp. peak is the tyre-limit scale the raw channel is normalised
// against (lateral: slip angle in degrees at full severity; longitudinal:
// slip ratio at full severity).
struct SlipParams
{
    double aMix = 1.0;
    double aHz  = 25.0;
    double bMix = 1.0;
    double bHz  = 11.0;
    double peak = 8.0;
    // How a slide comes in (the Lateral slip tile's; Longitudinal uses
    // attackMs only). onsetPct: the slide starts at this share of peak (of
    // peak deg, or of peak % on the combined-slip path); ease: 0 rises
    // linearly to full, 1 starts gently (severity squared), 2 gentler
    // still; smoothHz: the per-wheel slip angle or slip smoothed (sims send
    // it ~60 times a second, and a derived angle jitters), 0 = off;
    // attackMs: how fast a wheel's slide builds.
    double onsetPct = 40.0;
    double ease     = 1.0;
    double smoothHz = 8.0;
    double attackMs = 8.0;
};

// The Road tile's replay settings (used when per-corner suspension
// velocities or travel arrive; the synthesised texture from roadNoise
// otherwise). fullMm: suspension travel that is 100% amplitude (25: a real
// race car's suspension moves tens of mm over kerbs; smaller values pinned
// every bump at full). hpHz: high-pass on the travel, removing the slow
// body motion the cue already produces. surface: the tarmac grain under a
// rolling car, a rough texture on its own carrier (surfaceHz, low enough
// for a position axis to carry) whose level rises with road speed (full
// by surfaceKmh) on top of either path. Off by default since the
// suspension model's roughness (rough x) moves with the car by itself.
struct RoadParams
{
    double fullMm     = 25.0;
    double hpHz       = 2.0;
    double surface    = 0.0;    // mix 0..1
    double surfaceKmh = 100.0;  // full by this road speed
    double surfaceHz  = 12.0;   // the grain's carrier
    // Which model plays the road (RoadModel.h): 0 suspension (replay the
    // sim's suspension travel plus the road's fine roughness through the
    // corner), 1 tyre (a quarter car per corner on the road under that
    // tyre: the sim's real road where it gives one, plus the roughness),
    // 2 chassis (the sim's own body heave, pitch and roll in the band the
    // motion cue leaves out).
    double model      = 0.0;
    double bodyMm     = 1.0;    // tyre/chassis: body movement that counts as 100%
    double bodyHz     = 3.0;    // tyre, suspension roughness: the car's body bounce (race car 3-5, road car 1-1.5)
    double hopHz      = 16.0;   // tyre, suspension roughness: wheel hop (12-20)
    double damping    = 0.3;    // tyre, suspension roughness: the bounce's damping ratio
    double rough      = 2.0;    // tyre, suspension: road roughness, 1 = a smooth public road (ISO 8608 class A), 0 = none
};

// The Kerb tile: a rumble strip under the tyre that is on one. Ribs at
// pitch spacing play at speed / pitch, the step up onto the kerb at rise,
// both through the tyre (RoadModel.h KerbModel).
struct KerbParams
{
    double pitchCm  = 25.0;   // rib spacing
    double riseMm   = 8.0;    // the step up onto the kerb
    double ribMm    = 3.0;    // rib height
    double fullMm   = 10.0;   // kerb movement that counts as 100% on a position axis
    double detectMm = 0.0;    // no surface type from the sim: a tyre this much above its axle mate is on a kerb (0 = off)
};

// The ABS and TC tiles' pulse (PulseModel.h). Both: sharp (the drop's
// edge, 0 round to 1 a knock) and spread (each cycle a little different
// and, for ABS, each corner on its own rate). ABS also slows as the car
// slows and carries the pump's buzz; TC keeps slow and buzz at 0.
struct PulseParams
{
    double sharp  = 0.3;
    double spread = 0.5;
    double slow   = 0.5;    // ABS: how far the cycle slows towards a stop (0 = never, 0.5 = half rate at a standstill)
    double buzz   = 0.3;    // ABS: the pump and valve buzz mix 0..1
    double buzzHz = 40.0;   // ...its carrier
    // ABS slip link (0 off, 1 on): while ABS works, each corner's valve
    // cycle shapes that wheel's slip tiles. lockLink: the lock texture,
    // held near (never at) full lock and surging with the cycle; scrubLink:
    // the lateral slip easing after each dump. Spin is untouched.
    double slipLink  = 0.0;
    double lockLink  = 0.7;
    double scrubLink = 0.3;
};

// The Driveline tile: clutch judder and lugging wind-up, each a mix and
// a carrier (the driveline's own resonances).
struct DrivelineParams
{
    double clutch   = 1.0;
    double clutchHz = 10.0;
    double lug      = 1.0;
    double lugHz    = 7.0;
    double gearbox  = 0.0;    // 0 synchro (road H-pattern), 1 dog (sequential / dogbox)
    double whine    = 0.0;    // straight-cut gear whine mix 0..1 (0 = helical, quiet)
    double shunt    = 0.5;    // driveline shunt (backlash take-up) on throttle tip-in / lift, 0..1
    double shuntHz  = 40.0;   // ...its knock carrier
};

// Per-effect tuning shared by every effect kind. ampPct 0 = the effect is
// off. Transients use freqHz + durMs; continuous effects use freqHz as
// their carrier; the engine uses freqHz as its thump carrier and jitter
// as the idle lope (its own description lives in EngineParams).
// jitter (0..1) roughens the carrier per cycle so skid, road and kerb feel
// like texture rather than a tone.
//
// Routing is one flat model for every effect: an explicit per-axis gain
// table, empty by default (an unconfigured effect reaches nothing).
struct EffectParams
{
    double ampPct = 0.0;    // % of rated torque at full scale
    double freqHz = 90.0;   // burst/texture carrier (engine: thump carrier)
    double durMs  = 18.0;   // burst length (transients only)
    double jitter = 0.0;    // 0..1 carrier roughness (engine: lope)
    // Effects fed by a 0..100 magnitude channel (kerb, road texture, the
    // slip tiles' single-channel fallback): the channel value that counts
    // as full severity. A property that peaks at 25 in a burnout gets
    // peakPct 25, same idea as the per-wheel tiles' peak deg / peak ratio.
    double peakPct = 100.0;
    Route  routes[MAX_ROUTES] = {};
};

} // namespace haptics
