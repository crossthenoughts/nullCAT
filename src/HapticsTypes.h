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
static constexpr int MAX_ROUTES      = MAX_HAPTIC_AXES;  // an effect may route to EVERY axis

// Transients: one-shot bursts. GearShift is the sim-driven one (fired on a
// gear-channel change - the thunk of a shift ringing through the chassis).
enum class EventType { DetentClick = 0, GearShift = 1, COUNT };
static constexpr int EVENT_TYPE_COUNT = static_cast<int>(EventType::COUNT);

// Telemetry-driven CONTINUOUS effects. RpmVibe is the engine model; Lockup
// and Skid are the two per-wheel slip models (longitudinal and lateral);
// the rest are oscillators whose LEVEL (0..1) is driven per cycle by a law.
enum class FxType { RpmVibe = 0, AbsPulse = 1, Lockup = 2, Skid = 3, Road = 4,
                    Limiter = 5, TcPulse = 6, Kerb = 7, COUNT };
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
struct Route
{
    int    axis = -1;
    double gain = 0.0;
    Part   part = Part::All;
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
    Route  routes[MAX_ROUTES] = {};
};

} // namespace haptics
