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
                    Limiter, Tc, Kerb, Driveline, COUNT };
static constexpr int EFFECT_COUNT = static_cast<int>(Effect::COUNT);

// Slip: a per-wheel model (SlipModel.h) whose routes carry a Part. Road:
// the texture oscillator with a per-corner replay (RoadModel.h) when the
// sim sends suspension velocities; routes carry a Part too.
enum class Kind { Transient, Continuous, Engine, Slip, Road, Driveline };
inline bool kindHasParts(Kind k) { return k == Kind::Slip || k == Kind::Road; }

// One tunable the UI shows for an effect: config key, label, range, step,
// and for a choice an options list ("a|b|c", value = index) instead.
struct ParamSpec
{
    const char* key;
    const char* label;
    double      min, max, step;
    const char* opts;   // nullptr for a number
};

// Channel entries name the NcxValues tokens a law needs. Syntax, read by
// the web for its tick/cross chips: "a|b" = any of these satisfies it;
// a trailing '*' = any of the four wheel tokens of that group (slipAngle*
// = slipAngleFL..RR); a leading '~' = optional (never shown as missing).
struct EffectInfo
{
    Effect       id;
    const char*  key;          // config / API key
    const char*  label;        // tile title
    Kind         kind;
    EventType    event;        // transients: which pool slot (else COUNT)
    FxType       fx;           // continuous + engine + slip: which level slot (else COUNT)
    const char*  channels[6];  // nullptr-terminated, see above
    EffectParams defaults;
    const ParamSpec* params;
    int          paramCount;
    const char*  tip;
    // Slip only: the config keys of SlipParams {aMix, aHz, bMix, bHz, peak}
    // and the tile's defaults for them; nullptr / unused otherwise.
    const char* const* slipKeys;
    SlipParams   slipDefaults;
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
constexpr ParamSpec kSlipLatParams[] = {        // lateral slip: scrub (fronts) + slide (rears)
    { "ampPct",  "amp %",      0,   100, 1,    nullptr },
    { "scrub",   "scrub x",    0,   1,   0.1,  nullptr },
    { "scrubHz", "scrub hz",   8,   60,  1,    nullptr },
    { "slide",   "slide x",    0,   1,   0.1,  nullptr },
    { "slideHz", "slide hz",   4,   30,  1,    nullptr },
    { "peakDeg", "peak deg",   2,   20,  0.5,  nullptr },
    { "jitter",  "jitter",     0,   1,   0.05, nullptr },
};
constexpr ParamSpec kSlipLonParams[] = {        // longitudinal slip: lock judder + spin tramp
    { "ampPct",    "amp %",      0,   100, 1,    nullptr },
    { "lock",      "lock x",     0,   1,   0.1,  nullptr },
    { "lockHz",    "lock hz",    4,   30,  1,    nullptr },
    { "spin",      "spin x",     0,   1,   0.1,  nullptr },
    { "spinHz",    "spin hz",    4,   30,  1,    nullptr },
    { "peakRatio", "peak ratio", 0.2, 2,   0.05, nullptr },
    { "jitter",    "jitter",     0,   1,   0.05, nullptr },
};
constexpr const char* kSlipLatKeys[5] = { "scrub", "scrubHz", "slide", "slideHz", "peakDeg" };
constexpr const char* kSlipLonKeys[5] = { "lock", "lockHz", "spin", "spinHz", "peakRatio" };
constexpr ParamSpec kRoadParams[] = {           // road: texture carrier + per-corner replay settings
    { "ampPct", "amp %",     0,   100, 1,    nullptr },
    { "freqHz", "freq hz",   10,  120, 1,    nullptr },
    { "jitter", "jitter",    0,   1,   0.05, nullptr },
    { "fullMm", "full mm",   0.5, 50,  0.5,  nullptr },
    { "hpHz",   "cut hz",    0.5, 10,  0.5,  nullptr },
};
constexpr ParamSpec kDrivelineParams[] = {      // driveline: clutch judder + lugging wind-up
    { "ampPct",   "amp %",      0,   100, 1,    nullptr },
    { "clutch",   "clutch x",   0,   1,   0.1,  nullptr },
    { "clutchHz", "clutch hz",  4,   20,  0.5,  nullptr },
    { "lug",      "lug x",      0,   1,   0.1,  nullptr },
    { "lugHz",    "lug hz",     3,   15,  0.5,  nullptr },
    { "gearbox",  "gearbox",    0,   1,   1,    "synchro|dog" },
    { "whine",    "whine x",    0,   1,   0.1,  nullptr },
    { "shunt",    "shunt x",    0,   1,   0.1,  nullptr },
    { "shuntHz",  "shunt hz",   20,  80,  1,    nullptr },
    { "jitter",   "jitter",     0,   1,   0.05, nullptr },
};
constexpr ParamSpec kTextureParams[] = {        // limiter, kerb
    { "ampPct", "amp %",     0,   100, 1,    nullptr },
    { "freqHz", "freq hz",   10,  120, 1,    nullptr },
    { "jitter", "jitter",    0,   1,   0.05, nullptr },
};
constexpr ParamSpec kEngineParams[] = {
    { "ampPct",    "amp %",                 0,   100,   1,    nullptr },
    { "cylinders", "cyl / rotors",          1,   16,    1,    nullptr },
    { "litres",    "litres",                0.1, 30,    0.1,  nullptr },
    { "layout",    "layout",                0,   5,     1,    "inline|V|flat / boxer|wankel|two-stroke|electric" },
    { "maxRpm",    "max rpm (0=learn)",     0,   30000, 100,  nullptr },
    { "rock",      "rock x",                0,   1,     0.1,  nullptr },
    { "thump",     "thump x",               0,   1,     0.1,  nullptr },
    { "buzz",      "buzz x",                0,   1,     0.1,  nullptr },
    { "order",     "buzz order (0=auto)",   0,   4,     0.25, nullptr },
    { "limHit",    "limiter x",             0,   2,     0.1,  nullptr },
    { "limHz",     "limiter hz",            4,   30,    1,    nullptr },
    { "limJit",    "limiter jit",           0,   1,     0.05, nullptr },
    { "inertia",   "inertia x",             0,   1,     0.1,  nullptr },
    { "turbo",     "turbo",                 0,   1,     1,    "no|yes" },
    { "liftoff",   "lift-off x",            0,   2,     0.1,  nullptr },
    { "pops",      "pops x",                0,   1,     0.1,  nullptr },
    { "freqHz",    "thump hz",              10,  80,    1,    nullptr },
    { "jitter",    "lope",                  0,   1,     0.05, nullptr },
};

// EffectParams{ampPct, freqHz, durMs, jitter}; routes default empty.
constexpr EffectInfo kEffects[EFFECT_COUNT] = {
    { Effect::DetentClick, "detentClick", "Detent click", Kind::Transient, EventType::DetentClick, FxType::COUNT,
      { nullptr }, { 0.0, 90.0, 18.0, 0.0 }, kTransientParams, 3,
      "One short click as the lever settles into a gate, scaled by entry speed.", nullptr, {} },
    { Effect::GearShift,   "gearShift",   "Gear shift",   Kind::Transient, EventType::GearShift, FxType::COUNT,
      { "gear", nullptr }, { 0.0, 60.0, 25.0, 0.0 }, kTransientParams, 3,
      "A thunk on every gear change, ringing through the chassis. Needs the gear channel.", nullptr, {} },
    { Effect::Engine,      "rpmVibe",     "Engine",       Kind::Engine, EventType::COUNT, FxType::RpmVibe,
      { "rpm", "~throttlePct", "~limiter", "~boost" }, { 0.0, 30.0, 0.0, 0.15 }, kEngineParams, 18,
      "Engine: the block rocking at crank rate at idle (lumpy, fades out by ~2500 rpm) with each firing as a "
      "low thump on top. Above idle the rock hands over to the buzz: the firing-order vibration with pitch "
      "rising with rpm, kept at an order the actuator can carry; level grows with rpm and throttle up to the "
      "limiter. Describe the engine (cyl/rotors, litres, layout) and set rock x, thump x and buzz x by feel; "
      "max rpm 0 learns the redline while you drive. Throttle loads it; the limiter cuts whole bursts of "
      "firings for the bounce, with its own strength, rate and roughness. Inertia is the rpm-squared shake that stays "
      "when you lift and fades as the revs fall; lift-off is the pop (or, with turbo, the blow-off whoosh and "
      "flutter, scaled by boost when the sim sends it) as the throttle snaps shut up the band; pops is the "
      "fuel-cut crackle on the overrun. Needs rpm (throttle, limiter and boost optional).",
      nullptr, {} },
    { Effect::Abs,         "abs",         "ABS",          Kind::Continuous, EventType::COUNT, FxType::AbsPulse,
      { "brakePct", "absActive", nullptr }, { 0.0, 12.0, 0.0, 0.0 }, kPulseParams, 2,
      "Pulses while ABS cycles under braking. Needs the absActive and brakePct channels.", nullptr, {} },
    { Effect::Lockup,      "slipLon",     "Longitudinal slip", Kind::Slip, EventType::COUNT, FxType::Lockup,
      { "slipRatio*|wheelSpeed*|lockup", "speedKmh", "~load*", nullptr }, { 0.0, 9.0, 0.0, 0.2 }, kSlipLonParams, 7,
      "Each wheel's tread slipping along the road, per wheel. Lock: a wheel turning slower than the car under "
      "braking, a heavy judder whose beat falls with road speed. Spin: a driven wheel turning faster than the car, "
      "the axle tramping at its own resonance. Severity rises from the slip ratio past the tyre's limit up to peak "
      "ratio; the loaded tyre is weighted up when wheel loads arrive. Route with a part (a corner, an axle, all) so "
      "the inside front locking judders that corner. Needs per-wheel slip ratios, or wheel speeds (the rolling "
      "factor is learned while cruising), or the single lockup channel as a fallback.",
      kSlipLonKeys, { 1.0, 9.0, 1.0, 10.0, 0.8 } },
    { Effect::Skid,        "slipLat",     "Lateral slip", Kind::Slip, EventType::COUNT, FxType::Skid,
      { "slipAngle*|skid", "~load*", nullptr }, { 0.0, 25.0, 0.0, 0.5 }, kSlipLatParams, 7,
      "The tyres sliding sideways, per wheel. Scrub: the fronts pushing wide, a fine fast texture. Slide: the "
      "rears stepping out, an irregular slower chatter. Severity rises from the slip angle past the tyre's limit "
      "up to peak deg; the carrier slows and roughens as it goes (squeal, moan, shudder) and the onset is abrupt. "
      "The loaded tyre is weighted up when wheel loads arrive. Route with a part (a corner, an axle, all). Needs "
      "per-wheel slip angles, or the single skid channel as a fallback.",
      kSlipLatKeys, { 1.0, 25.0, 1.0, 11.0, 7.0 } },
    { Effect::Road,        "road",        "Road",         Kind::Road, EventType::COUNT, FxType::Road,
      { "suspVel*|roadNoise", nullptr }, { 0.0, 28.0, 0.0, 0.6 }, kRoadParams, 5,
      "The road surface. With per-corner suspension velocities from the sim it REPLAYS the road: each corner's "
      "travel, with the slow body motion cut away (cut hz) so only the bumps remain, at their real timing; full mm "
      "is the bump that counts as 100%. Route with a part so each actuator plays its own corner. Without them, a "
      "texture at freq hz scaled by the roadNoise channel (0-100).", nullptr, {} },
    { Effect::Limiter,     "limiter",     "Limiter",      Kind::Continuous, EventType::COUNT, FxType::Limiter,
      { "limiter", nullptr }, { 0.0, 12.0, 0.0, 0.15 }, kTextureParams, 3,
      "Extra hammer on top of the engine effect while the limiter is in (the engine effect already cuts "
      "bursts of firings for the bounce). Keep it slow, ~10-15 hz. The plugin computes the flag from rpm vs the car max.",
      nullptr, {} },
    { Effect::Tc,          "tc",          "TC pulse",     Kind::Continuous, EventType::COUNT, FxType::TcPulse,
      { "tcActive", nullptr }, { 0.0, 15.0, 0.0, 0.0 }, kPulseParams, 2,
      "Traction control cutting. Needs the tcActive channel.", nullptr, {} },
    { Effect::Kerb,        "kerb",        "Kerb",         Kind::Continuous, EventType::COUNT, FxType::Kerb,
      { "curbs", nullptr }, { 0.0, 40.0, 0.0, 0.4 }, kTextureParams, 3,
      "Kerb-strip rumble, scaled by the curbs channel (0-100). Bind curbsProp in the plugin.", nullptr, {} },
    { Effect::Driveline,   "driveline",   "Driveline",    Kind::Driveline, EventType::COUNT, FxType::Driveline,
      { "clutchPct", "rpm", "gear", "~speedKmh", "~throttlePct" }, { 0.0, 10.0, 0.0, 0.2 }, kDrivelineParams, 10,
      "The transmission when the engine and the wheels disagree. Clutch: a slipping clutch grabbing and releasing "
      "at a launch or a bad downshift, the whole driveline shuddering at clutch hz, from how much slip (engine rpm "
      "against what the gear and road speed say, using the learned ratios), how much load, and how far into the "
      "slipping band the pedal is. Lug: full throttle at too few revs winding the driveline up and letting go at "
      "lug hz, fading as the revs climb out of it. Gearbox shapes the gear-shift thunk: a synchro box clunks "
      "softly, a dog box knocks hard and, shifted under power, knocks again as the dogs engage. Whine is "
      "straight-cut gears meshing, its pitch stepping with every gear and its level with load. Shunt is the "
      "backlash taking up when the throttle snaps open or shut, in gear and rolling. Nothing in neutral. Needs "
      "clutch, rpm and gear (speed and throttle make it exact).", nullptr, {} },
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

// Keys an effect was saved under before its current key (0.9.6 rig files:
// "skid" became slipLat, "lockup" became slipLon). Config read falls back
// to these; nullptr when there is none.
inline const char* legacyEffectKey(Effect e)
{
    switch (e)
    {
        case Effect::Skid:   return "skid";
        case Effect::Lockup: return "lockup";
        default:             return nullptr;
    }
}

// SlipParams field by index, matching EffectInfo::slipKeys order.
inline double& slipField(SlipParams& s, int i)
{
    switch (i)
    {
        case 0: return s.aMix;
        case 1: return s.aHz;
        case 2: return s.bMix;
        case 3: return s.bHz;
        default: return s.peak;
    }
}
inline double slipField(const SlipParams& s, int i)
{
    return slipField(const_cast<SlipParams&>(s), i);
}

inline Part partFromKey(const char* k)
{
    if (!k) return Part::All;
    for (int i = 0; i < PART_COUNT; ++i)
        if (std::strcmp(partKey(static_cast<Part>(i)), k) == 0) return static_cast<Part>(i);
    return Part::All;
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
