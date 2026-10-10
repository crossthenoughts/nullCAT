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
                    Limiter, Tc, Kerb, Driveline, Surface, COUNT };
static constexpr int EFFECT_COUNT = static_cast<int>(Effect::COUNT);

// Slip: a per-wheel model (SlipModel.h) whose routes carry a Part. Road:
// the texture oscillator with a per-corner replay (RoadModel.h) when the
// sim sends suspension velocities; routes carry a Part too.
enum class Kind { Transient, Continuous, Engine, Slip, Road, Driveline, Kerb, Abs, Tc, Surface };
inline bool kindHasParts(Kind k) { return k == Kind::Slip || k == Kind::Road || k == Kind::Kerb || k == Kind::Abs || k == Kind::Surface; }

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
    // Slip only: the config keys of SlipParams by slipField index
    // (SLIP_KEY_COUNT of them, a null key = not this tile's) and the tile's
    // defaults for them; nullptr / unused otherwise.
    const char* const* slipKeys;
    SlipParams   slipDefaults;
};

constexpr int SLIP_KEY_COUNT = 9;   // SlipParams fields with a config key (slipField)

namespace registry_detail {

constexpr ParamSpec kTransientParams[] = {
    { "ampPct", "amp %",     0,   100, 1,    nullptr },
    { "freqHz", "freq hz",   10,  500, 5,    nullptr },
    { "durMs",  "length ms", 5,   100, 1,    nullptr },
};
constexpr ParamSpec kAbsParams[] = {            // ABS: the corners' valves, the pump
    { "ampPct", "amp %",     0,   100, 1,    nullptr },
    { "freqHz", "freq hz",   4,   60,  1,    nullptr },   // the cycle rate at speed
    { "sharp",  "sharp",     0,   1,   0.05, nullptr },   // the dump's edge: 0 round, 1 a knock
    { "spread", "spread",    0,   1,   0.05, nullptr },   // each corner its own rate, each cycle a little different
    { "slow",   "slow x",    0,   1,   0.05, nullptr },   // the cycle slowing towards a stop
    { "buzz",   "buzz x",    0,   1,   0.05, nullptr },   // the pump and valves
    { "buzzHz", "buzz hz",   15,  120, 1,    nullptr },
    { "slipLink",  "slip link", 0, 1,  1,    "off|on" },  // the corners' valves shape the slip tiles
    { "lockLink",  "lock x",    0, 1,  0.05, nullptr },   // ...the lock texture surging with each cycle
    { "scrubLink", "scrub x",   0, 1,  0.05, nullptr },   // ...the lateral slip easing after each dump
};
constexpr ParamSpec kTcParams[] = {             // TC: the cuts' surge
    { "ampPct", "amp %",     0,   100, 1,    nullptr },
    { "freqHz", "freq hz",   4,   60,  1,    nullptr },
    { "sharp",  "sharp",     0,   1,   0.05, nullptr },   // the cut's edge: 0 round, 1 a knock
    { "spread", "spread",    0,   1,   0.05, nullptr },   // each cut a little different in timing and depth
};
constexpr ParamSpec kSlipLatParams[] = {        // lateral slip: scrub (fronts) + slide (rears)
    { "ampPct",  "amp %",      0,   100, 1,    nullptr },
    { "scrub",   "scrub x",    0,   1,   0.1,  nullptr },
    { "scrubHz", "scrub hz",   8,   60,  1,    nullptr },
    { "slide",   "slide x",    0,   1,   0.1,  nullptr },
    { "slideHz", "slide hz",   4,   30,  1,    nullptr },
    { "peakDeg", "peak deg",   2,   20,  0.5,  nullptr },
    { "onsetPct","onset %",    0,   90,  5,    nullptr },   // the slide starts at this share of peak
    { "ease",    "ease",       0,   2,   0.1,  nullptr },   // 0 linear, 1 a gentle start, 2 gentler
    { "smoothHz","smooth hz",  0,   30,  1,    nullptr },   // slip angle smoothing, 0 off
    { "attackMs","attack ms",  2,   200, 1,    nullptr },   // how fast a slide builds
    { "jitter",  "jitter",     0,   1,   0.05, nullptr },
    { "peakPct", "peak %",     1,   400, 1,    nullptr },   // combined-slip and skid-channel paths
};
constexpr ParamSpec kSlipLonParams[] = {        // longitudinal slip: lock judder + spin tramp
    { "ampPct",    "amp %",      0,   100, 1,    nullptr },
    { "lock",      "lock x",     0,   1,   0.1,  nullptr },
    { "lockHz",    "lock hz",    4,   30,  1,    nullptr },
    { "spin",      "spin x",     0,   1,   0.1,  nullptr },
    { "spinHz",    "spin hz",    4,   30,  1,    nullptr },
    { "peakRatio", "peak ratio", 0.2, 2,   0.05, nullptr },
    { "jitter",    "jitter",     0,   1,   0.05, nullptr },
    { "peakPct",   "peak %",     1,   400, 1,    nullptr },   // lockup channel fallback only
};
// SlipParams fields by index (slipField). A null key = that tile does not
// carry the field (Longitudinal keeps its fixed onset and curve).
constexpr const char* kSlipLatKeys[SLIP_KEY_COUNT] = { "scrub", "scrubHz", "slide", "slideHz", "peakDeg",
                                                       "onsetPct", "ease", "smoothHz", "attackMs" };
constexpr const char* kSlipLonKeys[SLIP_KEY_COUNT] = { "lock", "lockHz", "spin", "spinHz", "peakRatio",
                                                       nullptr, nullptr, nullptr, nullptr };
constexpr ParamSpec kRoadParams[] = {           // road: the model, its settings, the texture fallback
    { "ampPct",     "amp %",       0,   100, 1,    nullptr },
    { "model",      "model",       0,   2,   1,    "suspension|tyre|chassis" },
    { "bodyMm",     "body mm",     0.1, 20,  0.1,  nullptr },   // tyre, chassis: body movement = 100%
    { "hpHz",       "cut hz",      0.5, 10,  0.5,  nullptr },
    { "rough",      "rough x",     0,   20,  0.1,  nullptr },   // tyre, suspension: road roughness
    { "bodyHz",     "body hz",     0.8, 8,   0.1,  nullptr },   // tyre, suspension roughness: the car's body bounce
    { "hopHz",      "hop hz",      6,   30,  0.5,  nullptr },   // tyre, suspension roughness: wheel hop
    { "damping",    "damping",     0.05, 1.5, 0.05, nullptr },  // tyre, suspension roughness: damping ratio
    { "fullMm",     "full mm",     0.5, 50,  0.5,  nullptr },   // suspension: travel = 100%
    { "surface",    "surface x",   0,   1,   0.05, nullptr },   // suspension: tarmac grain rising with road speed
    { "surfaceHz",  "surface hz",  4,   40,  1,    nullptr },   // ...on its own carrier
    { "surfaceKmh", "surface km/h", 20, 300, 5,    nullptr },   // ...full by this speed
    { "freqHz",     "freq hz",     10,  120, 1,    nullptr },   // the roadNoise texture fallback
    { "jitter",     "jitter",      0,   1,   0.05, nullptr },
    { "peakPct",    "peak %",      1,   100, 1,    nullptr },   // roadNoise texture fallback only
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
constexpr ParamSpec kTextureParams[] = {        // limiter (flag-driven: nothing to trim)
    { "ampPct", "amp %",     0,   100, 1,    nullptr },
    { "freqHz", "freq hz",   10,  120, 1,    nullptr },
    { "jitter", "jitter",    0,   1,   0.05, nullptr },
};
constexpr ParamSpec kKerbParams[] = {           // kerb: the rumble strip under the tyre
    { "ampPct",   "amp %",     0,   100, 1,    nullptr },
    { "pitchCm",  "pitch cm",  5,   100, 1,    nullptr },   // rib spacing: hums at speed / pitch
    { "riseMm",   "rise mm",   0,   50,  0.5,  nullptr },   // the step up onto the kerb
    { "ribMm",    "rib mm",    0,   20,  0.5,  nullptr },   // rib height
    { "fullMm",   "full mm",   0.5, 50,  0.5,  nullptr },   // kerb movement = 100% on a post
    { "jitter",   "jitter",    0,   1,   0.05, nullptr },   // rib to rib unevenness
    { "peakPct",  "peak %",    1,   100, 1,    nullptr },   // the single curbs channel
    { "detectMm", "detect mm", 0,   100, 1,    nullptr },   // no surface type: a tyre this far above its mate is on a kerb (0 off)
};
constexpr ParamSpec kSurfaceParams[] = {        // surface: stones, crunch, studs, puddles
    { "ampPct",  "amp %",      0,   100, 1,    nullptr },
    { "stones",  "stones x",   0,   1,   0.05, nullptr },   // gravel, sand, dirt: stones struck
    { "crunch",  "crunch x",   0,   1,   0.05, nullptr },   // snow (sand, mud) underfoot
    { "studs",   "studs",      0,   1,   1,    "off|on" },  // studded tyres on ice and snow
    { "puddles", "puddle x",   0,   1,   0.05, nullptr },   // the tug into a puddle, the bite out
    { "aquaKmh", "aqua km/h",  40,  250, 5,    nullptr },   // a tyre floats in standing water from here
    { "smooth",  "smooth x",   0,   1,   0.05, nullptr },   // how far water fills the road's texture
};
constexpr ParamSpec kEngineParams[] = {    { "ampPct",    "amp %",                 0,   100,   1,    nullptr },
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
      "A thunk on every gear change, ringing through the chassis: a short burst at freq hz on a belt or a shaker, "
      "one jolt (about 12 hz, longer for a synchro clunk, shorter for a dog knock) on a position axis, which cannot "
      "carry a buzz. Needs the gear channel.", nullptr, {} },
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
    { Effect::Abs,         "abs",         "ABS",          Kind::Abs, EventType::COUNT, FxType::AbsPulse,
      { "brakePct", "absActive", "~speedKmh", nullptr }, { 0.0, 12.0, 0.0, 0.0 }, kAbsParams, 10,
      "ABS working under braking: at each corner the valve dumps the brake pressure and rebuilds it, about freq hz "
      "times a second, so the car's deceleration judders. sharp is the dump's edge (0 round, 1 a hard knock); "
      "spread lets each corner run on its own rate and each cycle differ, so the four drift in and out of step and "
      "the car grumbles rather than beating; slow x slows the cycle as the car slows (the thump-thump before a "
      "stop: 0.5 = half rate at a standstill); buzz x / hz is the pump and valves underneath, best on a belt or "
      "shaker. Route with a part so a post carries its own corner. Slip link on: while ABS works each corner's "
      "valve shapes that wheel's slip tiles, the lock texture held at the edge of locking and surging with each "
      "cycle (lock x), the lateral slip easing after each dump (scrub x); this works with ABS's own amp at 0 too. "
      "Needs the absActive and brakePct channels (speed for slow x).",
      nullptr, {} },
    { Effect::Lockup,      "slipLon",     "Longitudinal slip", Kind::Slip, EventType::COUNT, FxType::Lockup,
      { "slipRatio*|wheelSpeed*|lockup", "speedKmh", "~load*", nullptr }, { 0.0, 9.0, 0.0, 0.2 }, kSlipLonParams, 8,
      "Each wheel's tread slipping along the road, per wheel. Lock: a wheel turning slower than the car under "
      "braking, a heavy judder whose beat falls with road speed. Spin: a driven wheel turning faster than the car, "
      "the tread chattering at the contact patch (16 hz; a live axle tramps lower, around 10). Severity rises from "
      "the slip ratio past the tyre's limit up to peak ratio; the loaded tyre is weighted up when wheel loads "
      "arrive. The two wheels of an axle move together, the corner told by how hard. Route with a part (a corner, "
      "an axle, all). Needs per-wheel slip ratios, or wheel speeds (the rolling factor is learned while "
      "cruising), or the single lockup channel as a fallback (peak % is the channel value that counts as a full "
      "slide on that path).",
      kSlipLonKeys, { 1.0, 9.0, 1.0, 16.0, 0.8 } },
    { Effect::Skid,        "slipLat",     "Lateral slip", Kind::Slip, EventType::COUNT, FxType::Skid,
      { "slipAngle*|wheelSlip*|skid", "speedKmh", "~load*", nullptr }, { 0.0, 20.0, 0.0, 0.5 }, kSlipLatParams, 12,
      "The tyres sliding, per wheel. Scrub: the fronts pushing wide, a fine texture. Slide: the rears stepping "
      "out, an irregular slower chatter. A slide starts at onset % of peak deg and is full at peak deg; ease "
      "shapes the way in (0 straight up, 1 a light scrub first, 2 lighter still), smooth hz steadies the slip "
      "angle the sim sends, attack ms is how fast a slide builds. The carrier slows and roughens as the slide "
      "grows (squeal, moan, shudder). The loaded tyre is weighted up when wheel loads arrive; the two wheels of "
      "an axle move together. Route with a part (a corner, an axle, all). Needs per-wheel slip angles; else the "
      "per-wheel combined slip (Automobilista 2: any sliding tyre, spinning and locking included), from onset % "
      "of peak % up to full at peak % (the chip shows the value arriving and its peak); else the single skid "
      "channel. Nothing at a standstill.",
      kSlipLatKeys, { 1.0, 20.0, 1.0, 11.0, 7.0, 40.0, 1.0, 8.0, 30.0 } },
    { Effect::Road,        "road",        "Road",         Kind::Road, EventType::COUNT, FxType::Road,
      { "roadHeight*|suspVel*|suspTravel*|roadNoise", "speedKmh", "~surface*", "~accHeave", nullptr },
      { 0.0, 16.0, 0.0, 0.6 }, kRoadParams, 15,
      "The road as the car's body feels it, above the band the motion cue covers (cut hz), per corner: route with "
      "a part so each post plays its own corner. A post moves the body's movement, a belt or shaker pushes with "
      "its acceleration. Model TYRE: a quarter car at each corner (body hz, hop hz, damping) running over the "
      "road under that tyre: the sim's real road where it sends one, else its suspension, plus roughness laid "
      "out by distance (rough x, more on bumpy roads, gravel, grass), so it rises in pitch with speed and the "
      "rears meet each bump a wheelbase after the fronts. Model CHASSIS: the sim's own body heave, pitch and roll "
      "in that band, spread to the corners. body mm is the body movement that counts as 100% (set it to your "
      "route gain to play the body 1:1). Model SUSPENSION (the default): the sim's suspension travel itself, full "
      "at full mm, plus the road's fine roughness the sim does not model (rough x, through the tyre and the "
      "corner's spring and damper: body hz, hop hz, damping), rising with speed, rougher on gravel, grass and "
      "cobbles; rough x 0 = the sim's travel alone. The old surface grain (surface x, hz, km/h) is still there, "
      "off by default. Without per-corner data, a texture at freq hz from the roadNoise channel, full at peak %.",
      nullptr, {} },
    { Effect::Limiter,     "limiter",     "Limiter",      Kind::Continuous, EventType::COUNT, FxType::Limiter,
      { "limiter", nullptr }, { 0.0, 12.0, 0.0, 0.15 }, kTextureParams, 3,
      "Extra hammer on top of the engine effect while the limiter is in (the engine effect already cuts "
      "bursts of firings for the bounce). Keep it slow, ~10-15 hz. The plugin computes the flag from rpm vs the car max.",
      nullptr, {} },
    { Effect::Tc,          "tc",          "TC pulse",     Kind::Tc, EventType::COUNT, FxType::TcPulse,
      { "tcActive", nullptr }, { 0.0, 15.0, 0.0, 0.0 }, kTcParams, 4,
      "Traction control cutting: the engine effect already stutters (TC drops a share of the firings in irregular "
      "bursts, more with more throttle); this adds the body surge of each cut, a loss of drive and a recovery about "
      "freq hz times a second. sharp is the cut's edge (0 round, 1 a hard knock), spread makes each cut a little "
      "different in timing and depth. Route it to surge and the belt. Needs the tcActive channel.", nullptr, {} },
    { Effect::Kerb,        "kerb",        "Kerb",         Kind::Kerb, EventType::COUNT, FxType::Kerb,
      { "surface*|curbs|roadHeight*", "speedKmh", nullptr }, { 0.0, 16.0, 0.0, 0.4 }, kKerbParams, 8,
      "A rumble strip under the tyre that is on one: ribs every pitch cm, so they hum at speed / pitch and the hum "
      "follows speed, each rib a little different (jitter), and the step up onto the kerb (rise mm). A post gets "
      "the kerb's height (a thud on and off; it cannot carry the hum), a belt or shaker the ribs and a kick on and "
      "off, stronger with speed. Route with a part so the corner on the kerb rumbles. Which tyre is on a kerb comes "
      "from the sim's surface type (Automobilista 2), else the curbs channel (Competizione, EVO: the game's own; "
      "full at peak %, all wheels), else, with detect mm set, a tyre riding that far above its axle mate on a "
      "real road profile (Assetto Corsa).",
      nullptr, {} },
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
    { Effect::Surface,     "surface",     "Surface",      Kind::Surface, EventType::COUNT, FxType::Surface,
      { "surface*", "speedKmh", "~rain|wet", nullptr }, { 0.0, 60.0, 0.0, 0.0 }, kSurfaceParams, 7,
      "What the ground does under each tyre beyond the road's shape, per corner (route with a part). Stones: "
      "on gravel, sand and dirt the tyres strike stones, more of them the faster you go, a sharp tick on a belt "
      "or shaker and a small bump on a post. Crunch: snow compacting under the tread (sand lighter, mud a low "
      "squelch). Studs: studded tyres biting on ice and snow, a rough buzz. Puddles: on a wet road, puddles "
      "laid out along it; the water drags at a tyre as it enters (stronger with speed), above aqua km/h in "
      "standing water the tyre floats and the road under it goes quiet, and it bites as it comes out. smooth x "
      "is how far water fills the Road tile's texture. Needs the surface under each tyre (Automobilista 2) "
      "and the wetness or the rain.", nullptr, {} },
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
        case 5: return s.onsetPct;
        case 6: return s.ease;
        case 7: return s.smoothHz;
        case 8: return s.attackMs;
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
