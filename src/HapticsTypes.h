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

// Telemetry-driven CONTINUOUS effects. RpmVibe is the engine model; the
// rest are oscillators whose LEVEL (0..1) is driven per cycle by a law.
enum class FxType { RpmVibe = 0, AbsPulse = 1, Lockup = 2, Skid = 3, Road = 4,
                    Limiter = 5, TcPulse = 6, Kerb = 7, COUNT };
static constexpr int FX_TYPE_COUNT = static_cast<int>(FxType::COUNT);

// One destination: explicit axis index and a gain multiplier.
// gain 0 or axis -1 = slot unused.
struct Route
{
    int    axis = -1;
    double gain = 0.0;
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
