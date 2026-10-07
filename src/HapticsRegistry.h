// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// HapticsRegistry - THE list of haptic effects, in one place.
//
// Everything that needs to know "which effects exist" reads this table:
// config read/write/validation, the live-apply stage, the controller's
// apply, the web Test endpoint, and /api/haptics/schema (which is how the
// browser builds its tiles, so it carries no copy of its own). Adding an
// ordinary effect is one row here plus one law in HapticLaws.h; adding one
// with physics of its own is a row plus a model file (see EngineModel.h).
//
// Keys are the config/API names and must never change once shipped (they
// are what rig.json files in the field contain).
// ============================================================

#include "HapticsTypes.h"
#include "EngineModel.h"
#include <array>
#include <cstring>

namespace haptics {

enum class Effect { DetentClick = 0, GearShift, Engine, Abs, Lockup, Skid, Road,
                    Limiter, Tc, Kerb, COUNT };
static constexpr int EFFECT_COUNT = static_cast<int>(Effect::COUNT);

enum class Kind { Transient, Continuous, Engine };

// One tunable the UI shows for an effect: config key, label, range, step,
// and for a choice an options list ("a|b|c", value = index) instead.
struct ParamSpec
{
    const char* key;
    const char* label;
    double      min, max, step;
    const char* opts;   // nullptr for a number
};

struct EffectInfo
{
    Effect       id;
    const char*  key;          // config / API key
    const char*  label;        // tile title
    Kind         kind;
    EventType    event;        // transients: which pool slot (else COUNT)
    FxType       fx;           // continuous + engine: which level slot (else COUNT)
    const char*  channels[3];  // NcxValues tokens the law needs (nullptr-terminated)
    EffectParams defaults;
    const ParamSpec* params;
    int          paramCount;
    const char*  tip;
};

namespace registry_detail {

constexpr ParamSpec kTransientParams[] = {
    { "ampPct", "amp %",     0,   100, 1,    nullptr },
    { "freqHz", "freq hz",   10,  500, 5,    nullptr },
    { "durMs",  "length ms", 5,   100, 1,    nullptr },
};
constexpr ParamSpec kPulseParams[] = {          // ABS, TC: periodic, no texture
    { "ampPct", "amp %",     0,   100, 1,    nullptr },
    { "freqHz", "freq hz",   4,   60,  1,    nullptr },
};
constexpr ParamSpec kLowTextureParams[] = {     // lockup: low carrier, roughened
    { "ampPct", "amp %",     0,   100, 1,    nullptr },
    { "freqHz", "freq hz",   4,   60,  1,    nullptr },
    { "jitter", "jitter",    0,   1,   0.05, nullptr },
};
constexpr ParamSpec kTextureParams[] = {        // skid, road, limiter, kerb
    { "ampPct", "amp %",     0,   100, 1,    nullptr },
    { "freqHz", "freq hz",   10,  120, 1,    nullptr },
    { "jitter", "jitter",    0,   1,   0.05, nullptr },
};
constexpr ParamSpec kEngineParams[] = {
    { "ampPct",    "amp %",                 0,   100,   1,    nullptr },
    { "cylinders", "cyl / rotors",          1,   16,    1,    nullptr },
    { "litres",    "litres",                0.1, 30,    0.1,  nullptr },
    { "layout",    "layout",                0,   3,     1,    "inline|V|flat / boxer|wankel" },
    { "maxRpm",    "max rpm (0=learn)",     0,   30000, 100,  nullptr },
    { "rock",      "rock x",                0,   1,     0.1,  nullptr },
    { "thump",     "thump x",               0,   1,     0.1,  nullptr },
    { "buzz",      "buzz x",                0,   1,     0.1,  nullptr },
    { "order",     "buzz order (0=auto)",   0,   4,     0.25, nullptr },
    { "limHit",    "limiter x",             0,   2,     0.1,  nullptr },
    { "limHz",     "limiter hz",            4,   30,    1,    nullptr },
    { "limJit",    "limiter jit",           0,   1,     0.05, nullptr },
    { "freqHz",    "thump hz",              10,  80,    1,    nullptr },
    { "jitter",    "lope",                  0,   1,     0.05, nullptr },
};

// EffectParams{ampPct, freqHz, durMs, jitter}; routes default empty.
constexpr EffectInfo kEffects[EFFECT_COUNT] = {
    { Effect::DetentClick, "detentClick", "Detent click", Kind::Transient, EventType::DetentClick, FxType::COUNT,
      { nullptr }, { 0.0, 90.0, 18.0, 0.0 }, kTransientParams, 3,
      "One short click as the lever settles into a gate, scaled by entry speed." },
    { Effect::GearShift,   "gearShift",   "Gear shift",   Kind::Transient, EventType::GearShift, FxType::COUNT,
      { "gear", nullptr }, { 0.0, 60.0, 25.0, 0.0 }, kTransientParams, 3,
      "A thunk on every gear change, ringing through the chassis. Needs the gear channel." },
    { Effect::Engine,      "rpmVibe",     "Engine",       Kind::Engine, EventType::COUNT, FxType::RpmVibe,
      { "rpm", "throttlePct", "limiter" }, { 0.0, 30.0, 0.0, 0.15 }, kEngineParams, 14,
      "Engine: the block rocking at crank rate at idle (lumpy, fades out by ~2500 rpm) with each firing as a "
      "low thump on top. Above idle the rock hands over to the buzz: the firing-order vibration with pitch "
      "rising with rpm, kept at an order the actuator can carry; level grows with rpm and throttle up to the "
      "limiter. Describe the engine (cyl/rotors, litres, layout) and set rock x, thump x and buzz x by feel; "
      "max rpm 0 learns the redline while you drive. Throttle loads it; the limiter cuts whole bursts of "
      "firings for the bounce, with its own strength, rate and roughness. Needs rpm (throttle and limiter optional)." },
    { Effect::Abs,         "abs",         "ABS",          Kind::Continuous, EventType::COUNT, FxType::AbsPulse,
      { "brakePct", "absActive", nullptr }, { 0.0, 12.0, 0.0, 0.0 }, kPulseParams, 2,
      "Pulses while ABS cycles under braking. Needs the absActive and brakePct channels." },
    { Effect::Lockup,      "lockup",      "Lockup",       Kind::Continuous, EventType::COUNT, FxType::Lockup,
      { "lockup", nullptr }, { 0.0, 9.0, 0.0, 0.2 }, kLowTextureParams, 3,
      "Wheel-lock judder, scaled by the lockup channel (0-100)." },
    { Effect::Skid,        "skid",        "Skid",         Kind::Continuous, EventType::COUNT, FxType::Skid,
      { "skid", nullptr }, { 0.0, 35.0, 0.0, 0.5 }, kTextureParams, 3,
      "Tyre-slip rumble, scaled by the skid channel (0-100)." },
    { Effect::Road,        "road",        "Road",         Kind::Continuous, EventType::COUNT, FxType::Road,
      { "roadNoise", nullptr }, { 0.0, 28.0, 0.0, 0.6 }, kTextureParams, 3,
      "Surface feel, scaled by the roadNoise channel (0-100)." },
    { Effect::Limiter,     "limiter",     "Limiter",      Kind::Continuous, EventType::COUNT, FxType::Limiter,
      { "limiter", nullptr }, { 0.0, 12.0, 0.0, 0.15 }, kTextureParams, 3,
      "Extra hammer on top of the engine effect while the limiter is in (the engine effect already cuts "
      "bursts of firings for the bounce). Keep it slow, ~10-15 hz. The plugin computes the flag from rpm vs the car max." },
    { Effect::Tc,          "tc",          "TC pulse",     Kind::Continuous, EventType::COUNT, FxType::TcPulse,
      { "tcActive", nullptr }, { 0.0, 15.0, 0.0, 0.0 }, kPulseParams, 2,
      "Traction control cutting. Needs the tcActive channel." },
    { Effect::Kerb,        "kerb",        "Kerb",         Kind::Continuous, EventType::COUNT, FxType::Kerb,
      { "curbs", nullptr }, { 0.0, 40.0, 0.0, 0.4 }, kTextureParams, 3,
      "Kerb-strip rumble, scaled by the curbs channel (0-100). Bind curbsProp in the plugin." },
};

} // namespace registry_detail

// Per-effect defaults from the table, for seeding AppConfig.
inline std::array<EffectParams, EFFECT_COUNT> defaultEffectParams()
{
    std::array<EffectParams, EFFECT_COUNT> out{};
    for (int i = 0; i < EFFECT_COUNT; ++i) out[static_cast<size_t>(i)] = registry_detail::kEffects[i].defaults;
    return out;
}

inline const EffectInfo& effectInfo(Effect e)
{
    return registry_detail::kEffects[static_cast<int>(e)];
}
inline const EffectInfo& effectInfo(int i)
{
    return registry_detail::kEffects[(i < 0 || i >= EFFECT_COUNT) ? 0 : i];
}

// Lookup by config/API key; nullptr if unknown.
inline const EffectInfo* findEffect(const char* key)
{
    if (!key) return nullptr;
    for (const EffectInfo& e : registry_detail::kEffects)
        if (std::strcmp(e.key, key) == 0) return &e;
    return nullptr;
}

// The continuous/engine effect that owns a level slot (status arrays are
// indexed by FxType), or nullptr for an unused slot.
inline const EffectInfo* effectForFx(FxType t)
{
    for (const EffectInfo& e : registry_detail::kEffects)
        if (e.fx == t) return &e;
    return nullptr;
}

} // namespace haptics
