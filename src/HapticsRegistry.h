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
#include <iterator>   // std::size: each tile's param count is its table's size
#include <cstring>

namespace haptics {

enum class Effect { DetentClick = 0, GearShift, Engine, Abs, Lockup, Skid, Road,
                    Limiter, Tc, Kerb, Driveline, Surface, Wheels, Impacts, COUNT };
static constexpr int EFFECT_COUNT = static_cast<int>(Effect::COUNT);

// Slip: a per-wheel model (SlipModel.h) whose routes carry a Part. Road:
// the texture oscillator with a per-corner replay (RoadModel.h) when the
// sim sends suspension velocities; routes carry a Part too.
enum class Kind { Transient, Continuous, Engine, Slip, Road, Driveline, Kerb, Abs, Tc, Surface, Wheels, Impacts };
inline bool kindHasParts(Kind k)
{
    return k == Kind::Slip || k == Kind::Road || k == Kind::Kerb || k == Kind::Abs || k == Kind::Surface
        || k == Kind::Wheels || k == Kind::Impacts;
}

// One tunable the UI shows for an effect: config key, label, range, step,
// and for a choice an options list ("a|b|c", value = index) instead.
// flags: what else the setting is (kParam* below).
struct ParamSpec
{
    const char* key;
    const char* label;
    double      min, max, step;
    const char* opts;        // nullptr for a number
    unsigned    flags = 0;
};

// The setting belongs to the car, not the rig: a car table entry sets it
// (follow car, pull), "save for this car" keeps it, a starting point leaves
// it alone, and the tile tints it. (A profile is the whole set: it carries
// these too.) CarTable::carKeys() reads these flags; nothing else keeps a
// list of them.
constexpr unsigned kParamCar = 1u << 0;

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

constexpr int SLIP_KEY_COUNT = 18;  // SlipParams fields with a config key (slipField)

namespace registry_detail {

constexpr ParamSpec kTransientParams[] = {
    { "ampPct", "amp %",     0,   100, 1,    nullptr },
    { "freqHz", "freq hz",   10,  500, 5,    nullptr },
    { "durMs",  "length ms", 5,   100, 1,    nullptr },
};
constexpr ParamSpec kAbsParams[] = {            // ABS: the corners' valves, the pump
    { "ampPct", "amp %",     0,   100, 1,    nullptr, kParamCar },
    { "freqHz", "freq hz",   4,   60,  1,    nullptr, kParamCar },   // the cycle rate at speed
    { "sharp",  "sharp",     0,   1,   0.05, nullptr },   // the dump's edge: 0 round, 1 a knock
    { "spread", "spread",    0,   1,   0.05, nullptr },   // each corner its own rate, each cycle a little different
    { "slow",   "slow x",    0,   1,   0.05, nullptr },   // the cycle slowing towards a stop
    { "buzz",   "buzz x",    0,   1,   0.05, nullptr },   // the pump and valves
    { "buzzHz", "buzz hz",   15,  120, 1,    nullptr },
    { "brakeFull", "full at brake %", 10, 100, 5, nullptr }, // light braking, a light pulse; full by this pedal
    { "holdMs",    "hold ms",   0, 500, 10,   nullptr },  // the sim's flag held over gaps this long (0 off)
    { "fromWheels","from wheels", 0, 2, 1,   "off|with the flag|on its own" },  // ABS also from the wheels dipping and recovering, the pulse timed to them
    { "lowOff",    "off < 5 km/h", 0, 1, 1,  "off|on" },  // no ABS below walking pace, whatever the flag says
    { "slipLink",  "slip link", 0, 1,  1,    "off|on" },  // the corners' valves shape the slip tiles
    { "lockLink",  "lock x",    0, 1,  0.05, nullptr },   // ...the lock texture surging with each cycle
    { "scrubLink", "scrub x",   0, 1,  0.05, nullptr },   // ...the lateral slip easing after each dump
};
constexpr ParamSpec kTcParams[] = {             // TC: the cuts' surge
    { "ampPct", "amp %",     0,   100, 1,    nullptr, kParamCar },
    { "freqHz", "freq hz",   4,   60,  1,    nullptr, kParamCar },
    { "sharp",  "sharp",     0,   1,   0.05, nullptr },   // the cut's edge: 0 round, 1 a knock
    { "spread", "spread",    0,   1,   0.05, nullptr },   // each cut a little different in timing and depth
    { "holdMs", "hold ms",   0,   500, 10,   nullptr },   // the sim's flag held over gaps this long (0 off)
};
constexpr ParamSpec kSlipLatParams[] = {        // lateral slip: scrub (fronts) + slide (rears)
    { "ampPct",  "amp %",      0,   100, 1,    nullptr },
    { "scrub",   "scrub x",    0,   1,   0.1,  nullptr },
    { "scrubHz", "scrub hz",   8,   60,  1,    nullptr },
    { "screech",   "screech x",   0,  1,   0.05, nullptr },   // fronts: the tyres starting to squeal, before they let go
    { "screechPct","screech %",   0,  95,  5,    nullptr },   // ...from this share of peak deg
    { "screechHz", "screech hz",  10, 120, 1,    nullptr },   // ...on its own carrier
    { "pulse",     "pulse x",     0,  1,   0.05, nullptr },   // fronts past the limit: the slow stick-slip shove
    { "pulseHz",   "pulse hz",    2,  15,  0.5,  nullptr },
    { "slide",   "slide x",    0,   1,   0.1,  nullptr },
    { "slideHz", "slide hz",   4,   30,  1,    nullptr },
    { "peakDeg", "peak deg",   2,   20,  0.5,  nullptr },
    { "onsetPct","onset %",    0,   90,  5,    nullptr },   // the slide starts at this share of peak
    { "ease",    "ease",       0,   2,   0.1,  nullptr },   // 0 linear, 1 a gentle start, 2 gentler
    { "smoothHz","smooth hz",  0,   30,  1,    nullptr },   // slip angle smoothing, 0 off
    { "attackMs","attack ms",  2,   200, 1,    nullptr },   // how fast a slide builds
    { "budget",  "grip budget", 0,  1,   1,    "off|on" },  // cornering and braking/driving share one grip
    { "baseMode", "baseline",  0,   2,   1,    "learn|learn once|fixed" },   // slip-speed path: the straight-line angle taken off
    { "baseDegF", "base deg F", 0,  6,   0.1,  nullptr },   // ...fixed: the fronts'
    { "baseDegR", "base deg R", 0,  6,   0.1,  nullptr },   // ...fixed: the rears'
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
    { "edge",      "edge x",     0,   1,   0.05, nullptr },   // the tyres working short of their limit
    { "edgePct",   "edge from %", 0.5, 14, 0.5,  nullptr },   // ...from this slip ratio % (full at the 15 % limit)
    { "edgeHz",    "edge hz",    10,  120, 1,    nullptr },   // ...on its own carrier
    { "jitter",    "jitter",     0,   1,   0.05, nullptr },
    { "peakPct",   "peak %",     1,   400, 1,    nullptr },   // lockup channel fallback only
};
// SlipParams fields by index (slipField). A null key = that tile does not
// carry the field (Longitudinal keeps its fixed onset and curve).
constexpr const char* kSlipLatKeys[SLIP_KEY_COUNT] = { "scrub", "scrubHz", "slide", "slideHz", "peakDeg",
                                                       "onsetPct", "ease", "smoothHz", "attackMs", "budget",
                                                       "screech", "screechPct", "screechHz", "pulse", "pulseHz",
                                                       "baseMode", "baseDegF", "baseDegR" };
constexpr const char* kSlipLonKeys[SLIP_KEY_COUNT] = { "lock", "lockHz", "spin", "spinHz", "peakRatio",
                                                       nullptr, nullptr, nullptr, nullptr, nullptr,
                                                       "edge", "edgePct", "edgeHz", nullptr, nullptr,
                                                       nullptr, nullptr, nullptr };
constexpr ParamSpec kRoadParams[] = {           // road: the model, its settings, the texture fallback
    { "ampPct",     "amp %",       0,   100, 1,    nullptr },
    { "model",      "model",       0,   2,   1,    "suspension|tyre|chassis" },
    { "bodyMm",     "body mm",     0.1, 50,  0.5,  nullptr },   // tyre, chassis: body movement = 100%
    { "hpHz",       "cut hz",      0.5, 10,  0.5,  nullptr },
    { "rough",      "rough x",     0,   20,  0.1,  nullptr },   // tyre, suspension: road roughness
    { "suspPct",    "susp %",      0,   100, 5,    nullptr },   // tyre, suspension: the game's own movement
    { "suspHz",     "susp cut hz", 0,   15,  0.5,  nullptr },   // ...its steep cut (0 = off)
    { "bodyHz",     "body hz",     0.8, 8,   0.1,  nullptr },   // tyre, suspension roughness: the car's body bounce
    { "hopHz",      "hop hz",      6,   30,  0.5,  nullptr },   // tyre, suspension roughness: wheel hop
    { "damping",    "damping",     0.05, 1.5, 0.05, nullptr },  // tyre, suspension roughness: damping ratio
    { "fullMm",     "full mm",     0.5, 50,  0.5,  nullptr },   // suspension: travel = 100%
    { "fullLearn",  "learn full mm", 0, 1,   1,    "off|on" },  // ...learned per car from its suspension instead
    { "surface",    "surface x",   0,   1,   0.05, nullptr },   // suspension: tarmac grain rising with road speed
    { "surfaceHz",  "surface hz",  4,   40,  1,    nullptr },   // ...on its own carrier
    { "surfaceKmh", "surface km/h", 20, 300, 5,    nullptr },   // ...full by this speed
    { "freqHz",     "freq hz",     10,  120, 1,    nullptr },   // the roadNoise texture fallback
    { "jitter",     "jitter",      0,   1,   0.05, nullptr },
    { "peakPct",    "peak %",      1,   100, 1,    nullptr },   // roadNoise texture fallback only
};
constexpr ParamSpec kDrivelineParams[] = {      // driveline: clutch judder + lugging wind-up
    { "ampPct",   "amp %",      0,   100, 1,    nullptr, kParamCar },
    { "clutch",   "clutch x",   0,   1,   0.1,  nullptr, kParamCar },
    { "clutchHz", "clutch hz",  4,   20,  0.5,  nullptr, kParamCar },
    { "lug",      "lug x",      0,   1,   0.1,  nullptr, kParamCar },
    { "lugHz",    "lug hz",     3,   15,  0.5,  nullptr, kParamCar },
    { "gearbox",  "gearbox",    0,   1,   1,    "synchro|dog", kParamCar },
    { "ignoreN",  "ignore neutral", 0, 1, 1,   "off|on" },   // the Gear shift knock: N passed through is no shift
    { "diff",     "diff x",     0,   1,   0.05, nullptr, kParamCar },   // the differential's clunk (0 = off: an open diff)
    { "diffType", "diff",       0,   1,   1,    "limited slip|spool", kParamCar },
    { "diffHz",   "diff hz",    2,   30,  1,    nullptr, kParamCar },   // the clunks' rate when the wheels turn apart briskly
    { "driven",   "driven",     0,   2,   1,    "rear|front|all", kParamCar },
    { "whine",    "whine x",    0,   1,   0.1,  nullptr, kParamCar },
    { "whinePitch", "whine pitch", 0, 3,  1,    "with the buzz|below the buzz|octave down|above the buzz" },
    { "shunt",    "shunt x",    0,   1,   0.1,  nullptr, kParamCar },
    { "shuntHz",  "shunt hz",   20,  80,  1,    nullptr, kParamCar },
    { "jitter",   "jitter",     0,   1,   0.05, nullptr, kParamCar },
};
constexpr ParamSpec kTextureParams[] = {        // limiter (flag-driven: nothing to trim)
    { "ampPct", "amp %",     0,   100, 1,    nullptr, kParamCar },
    { "freqHz", "freq hz",   5,   120, 1,    nullptr, kParamCar },   // from 5: a truck's or a kart's slow limiter hammer
    { "jitter", "jitter",    0,   1,   0.05, nullptr, kParamCar },
};
constexpr ParamSpec kKerbParams[] = {           // kerb: the rumble strip under the tyre
    { "ampPct",   "amp %",     0,   100, 1,    nullptr },
    { "pitchCm",  "pitch cm",  5,   100, 1,    nullptr },   // rib spacing: hums at speed / pitch
    { "ribMm",    "rib mm",    0,   20,  0.5,  nullptr },   // rib height
    { "fullMm",   "full mm",   0.5, 50,  0.5,  nullptr },   // kerb movement = 100% on a post
    { "jitter",   "jitter",    0,   1,   0.05, nullptr },   // rib to rib unevenness
    { "peakPct",  "peak %",    1,   100, 1,    nullptr },   // the game's own kerb vibration
};
constexpr ParamSpec kSurfaceParams[] = {        // surface: stones, crunch, studs, puddles
    { "ampPct",  "amp %",      0,   100, 1,    nullptr },
    { "stones",  "stones x",   0,   1,   0.05, nullptr },   // gravel, sand, dirt: stones struck
    { "stonesPerM", "stones per m", 0.05, 2, 0.05, nullptr },   // how many a tyre strikes per metre of gravel
    { "crunch",  "crunch x",   0,   1,   0.05, nullptr },   // snow (sand, mud) underfoot
    { "studs",   "studs",      0,   1,   1,    "off|on" },  // studded tyres on ice and snow
    { "puddles", "puddle x",   0,   1,   0.05, nullptr },   // the tug into a puddle, the bite out
    { "aquaKmh", "aqua km/h",  40,  250, 5,    nullptr },   // a tyre floats in standing water from here
    { "smooth",  "smooth x",   0,   1,   0.05, nullptr },   // how far water fills the road's texture
    { "assume",  "no surface sent", 0, 4, 1,   "tarmac|gravel|dirt|snow|sand" },   // the ground when the sim sends none
};
constexpr ParamSpec kWheelsParams[] = {         // wheels: out of round, flat spots, brake judder
    { "ampPct",  "amp %",      0,   100, 1,    nullptr },
    { "balance", "balance x",  0,   4,   0.1,  nullptr },   // the wheel out of balance (and a touch out of round)
    { "flat",    "flat x",     0,   4,   0.1,  nullptr },   // flat spots from lock-ups
    { "judder",  "judder x",   0,   1,   0.05, nullptr },   // hot brake discs under braking
    { "puncture","puncture x", 0,   2,   0.1,  nullptr },   // a punctured tyre, when the sim says so
    { "bite",    "bite x",     0,   1,   0.05, nullptr },   // the pads grabbing the disc as the brake goes on
    { "drag",    "drag x",     0,   1,   0.05, nullptr },   // ...and rubbing while it stays on
    { "biteHz",  "bite hz",    10,  200, 1,    nullptr },   // the grab's grain at 100 km/h, following the wheel's speed
    { "dragHz",  "drag hz",    10,  200, 1,    nullptr },   // ...and the rub's
    { "freshOnReset", "fresh tyres on pit/restart", 0, 1, 1, "off|on" },   // a restart, a jump to the pits, a tyre change
};
constexpr ParamSpec kImpactParams[] = {         // impacts: the edge of a hit through the suspension
    { "ampPct",  "amp %",      0,   100,  1,    nullptr },
    { "freqHz",  "ring hz",    15,  120,  1,    nullptr },   // the knock's ring on a belt or shaker
    { "durMs",   "ring ms",    10,  200,  1,    nullptr },   // ...and how fast it dies
    { "fromMmS", "from mm/s",  100, 3000, 50,   nullptr },   // a suspension velocity jump that starts a knock
    { "fullMmS", "full mm/s",  300, 8000, 100,  nullptr },   // ...and a full one
    { "heaveG",  "heave g",    0,   5,    0.1,  nullptr },   // a vertical g jump that knocks the whole car (0 off)
    { "hitG",    "hit g",      0,   10,   0.1,  nullptr },   // a horizontal g jump (a wall, a contact) that knocks the whole car (0 off)
};
constexpr ParamSpec kEngineParams[] = {    { "ampPct",    "amp %",                 0,   100,   1,    nullptr, kParamCar },
    { "cylinders", "cyl / rotors",          1,   16,    1,    nullptr, kParamCar },
    { "litres",    "litres",                0.1, 30,    0.1,  nullptr, kParamCar },
    { "layout",    "layout",                0,   5,     1,    "inline|V|flat / boxer|wankel|two-stroke|electric", kParamCar },
    { "maxRpm",    "max rpm (0=learn)",     0,   30000, 100,  nullptr, kParamCar },
    { "rock",      "rock x",                0,   1,     0.1,  nullptr, kParamCar },
    { "thump",     "thump x",               0,   1,     0.1,  nullptr, kParamCar },
    { "buzz",      "buzz x",                0,   1,     0.1,  nullptr, kParamCar },
    { "order",     "buzz order (0=auto)",   0,   8,     0.25, nullptr },   // the rig's, not the car's: a PC wants about 0.75, the Pi 0
    { "limHit",    "limiter x",             0,   2,     0.1,  nullptr, kParamCar },
    { "limHz",     "limiter hz",            4,   30,    1,    nullptr, kParamCar },
    { "limJit",    "limiter jit",           0,   1,     0.05, nullptr, kParamCar },
    { "inertia",   "inertia x",             0,   1,     0.1,  nullptr, kParamCar },
    { "turbo",     "turbo",                 0,   1,     1,    "no|yes", kParamCar },
    { "liftoff",   "lift-off x",            0,   2,     0.1,  nullptr, kParamCar },
    { "pops",      "pops x",                0,   1,     0.1,  nullptr, kParamCar },
    { "tcCut",     "TC cut x",              0,   1,     0.1,  nullptr, kParamCar },   // traction control dropping firings (0 = ignore TC)
    { "foldback",  "foldback",              0,   2,     1,    "wrap (as now)|stop|keep rising" },   // firings faster than the loop can play
    { "freqHz",    "thump hz",              10,  80,    1,    nullptr, kParamCar },
    { "jitter",    "lope",                  0,   1,     0.05, nullptr, kParamCar },
};

// EffectParams{ampPct, freqHz, durMs, jitter}; routes default empty.
constexpr EffectInfo kEffects[EFFECT_COUNT] = {
    { Effect::DetentClick, "detentClick", "Detent click", Kind::Transient, EventType::DetentClick, FxType::COUNT,
      { nullptr }, { 0.0, 90.0, 18.0, 0.0 }, kTransientParams, static_cast<int>(std::size(kTransientParams)),
      "One short click as the lever settles into a gate, scaled by entry speed.", nullptr, {} },
    { Effect::GearShift,   "gearShift",   "Gear shift",   Kind::Transient, EventType::GearShift, FxType::COUNT,
      { "gear", nullptr }, { 0.0, 60.0, 25.0, 0.0 }, kTransientParams, static_cast<int>(std::size(kTransientParams)),
      "A thunk on every gear change, ringing through the chassis: a short burst at freq hz on a belt or a shaker, "
      "one jolt (about 12 hz, longer for a synchro clunk, shorter for a dog knock) on a position axis, which cannot "
      "carry a buzz. A gear counts once it has held a few hundredths of a second (a sim's one-frame junk gear "
      "mid-shift is none); with the Driveline tile's ignore neutral, one knock per shift as the box leaves its gear "
      "(the neutral passed through and the gear it lands in are the same shift), and the gearbox there shapes the "
      "knock. Needs the gear channel.", nullptr, {} },
    { Effect::Engine,      "rpmVibe",     "Engine",       Kind::Engine, EventType::COUNT, FxType::RpmVibe,
      { "rpm", "~throttlePct", "~limiter", "~boost" }, { 0.0, 30.0, 0.0, 0.15 }, kEngineParams, static_cast<int>(std::size(kEngineParams)),
      "Engine: the block rocking at crank rate at idle (lumpy, fades out by ~2500 rpm) with each firing as a "
      "low thump on top. Above idle the rock hands over to the buzz: the firing-order vibration with pitch "
      "rising with rpm, kept at an order the actuator can carry; level grows with rpm and throttle up to the "
      "limiter. Describe the engine (cyl/rotors, litres, layout) and set rock x, thump x and buzz x by feel; "
      "max rpm 0 learns the redline while you drive. Throttle loads it; the limiter cuts whole bursts of "
      "firings for the bounce, with its own strength, rate and roughness. Inertia is the rpm-squared shake that stays "
      "when you lift and fades as the revs fall; lift-off is the pop (or, with turbo, the blow-off whoosh and "
      "flutter, scaled by boost when the sim sends it) as the throttle snaps shut up the band; pops is the "
      "fuel-cut crackle on the overrun; TC cut is how hard the engine stutters while the sim's traction "
      "control cuts (0, the default: not at all, as a modern TC trims torque smoothly; toward 1 for an "
      "ignition-cut TC). Needs rpm (throttle, limiter and boost optional).",
      nullptr, {} },
    { Effect::Abs,         "abs",         "ABS",          Kind::Abs, EventType::COUNT, FxType::AbsPulse,
      { "brakePct", "absActive", "~speedKmh", nullptr }, { 0.0, 12.0, 0.0, 0.0 }, kAbsParams, static_cast<int>(std::size(kAbsParams)),
      "ABS working under braking: at each corner the valve dumps the brake pressure and rebuilds it, about freq hz "
      "times a second, so the car's deceleration judders. sharp is the dump's edge (0 round, 1 a hard knock); "
      "spread lets each corner run on its own rate and each cycle differ, so the four drift in and out of step and "
      "the car grumbles rather than beating; slow x slows the cycle as the car slows (the thump-thump before a "
      "stop: 0.5 = half rate at a standstill); buzz x / hz is the pump and valves underneath, best on a belt or "
      "shaker. It pulses as hard as you brake: light trail braking a light pulse, full at full at brake % (10 = "
      "always full). hold ms keeps the sim's ABS flag on over gaps that short (some sims blip it), so a stop "
      "plays one steady cycle (0 = off). From wheels on: ABS also counts as working when the wheels show it (each "
      "one dipping towards a lock and spinning back up as its valve dumps), and each corner's pulse is timed to "
      "those dumps; for sims that report ABS only now and then. Route with a part so a post carries its own corner. Slip link on: while ABS works each corner's "
      "valve shapes that wheel's slip tiles, the lock texture held at the edge of locking and surging with each "
      "cycle (lock x), the lateral slip easing after each dump (scrub x); amp 0 is no ABS at all, the slip link included. "
      "Needs the absActive and brakePct channels (speed for slow x).",
      nullptr, {} },
    { Effect::Lockup,      "slipLon",     "Longitudinal slip", Kind::Slip, EventType::COUNT, FxType::Lockup,
      { "slipRatio*|wheelSpeed*|lockup", "speedKmh", "~load*", nullptr }, { 0.0, 9.0, 0.0, 0.2 }, kSlipLonParams, static_cast<int>(std::size(kSlipLonParams)),
      "Each wheel's tread slipping along the road, per wheel. Lock: a wheel turning slower than the car under "
      "braking, a heavy judder whose beat falls with road speed. Spin: a driven wheel turning faster than the car, "
      "the tread chattering at the contact patch (16 hz; a live axle tramps lower, around 10). Severity rises from "
      "the slip ratio past the tyre's limit up to peak ratio; the lighter tyre of an axle plays weaker when wheel loads "
      "arrive. The two wheels of an axle move together, the corner told by how hard. Route with a part (a corner, "
      "an axle, all). Needs per-wheel slip ratios, or wheel speeds (the rolling factor is learned while "
      "cruising), or the single lockup channel as a fallback (peak % is the channel value that counts as a full "
      "slide on that path). Edge: every tyre working short of its limit, braking or driving (a traction control "
      "holds the driven wheels at a few % of slip, well short of the spin texture): a fine grain from edge from % "
      "of slip ratio up to full at the limit, on its own carrier (edge hz), so you feel the rear putting the power "
      "down and the fronts near locking before anything lets go.",
      kSlipLonKeys, { 1.0, 9.0, 1.0, 16.0, 0.8, 40.0, 1.0, 8.0, 8.0, 0.0, 0.3, 3.0, 35.0, 0.0, 6.0, 0.0, 0.0, 0.0 } },
    { Effect::Skid,        "slipLat",     "Lateral slip", Kind::Slip, EventType::COUNT, FxType::Skid,
      { "slipAngle*|slipSpeed*|wheelSlip*|skid", "speedKmh", "~load*", nullptr }, { 0.0, 20.0, 0.0, 0.5 }, kSlipLatParams, static_cast<int>(std::size(kSlipLatParams)),
      "The tyres sliding, per wheel. Scrub: the fronts pushing wide, a fine texture. Screech: the fronts starting "
      "to squeal before they let go, a light texture from screech % of peak deg, full at peak deg, on its own "
      "carrier (screech hz). Pulse: fronts past the limit sticking and slipping in a slow, irregular shove at "
      "pulse hz, as strong as the scrub; slow enough for a seat actuator to carry. Slide: the rears stepping "
      "out, an irregular slower chatter. A slide starts at onset % of peak deg and is full at peak deg; ease "
      "shapes the way in (0 straight up, 1 a light scrub first, 2 lighter still), smooth hz steadies the slip "
      "angle the sim sends, attack ms is how fast a slide builds. Grip budget on: a tyre has one grip for "
      "cornering and braking or driving together, so where the sim sends both a slip angle and a slip ratio (or "
      "wheel speeds) a wheel slides when the two together reach its limit (trail braking, power out of a corner), "
      "and the slide plays here or on Longitudinal slip by its direction. The carrier slows and roughens as the slide "
      "grows (squeal, moan, shudder). The lighter tyre of an axle plays weaker when wheel loads arrive; the two wheels of "
      "an axle move together. Route with a part (a corner, an axle, all). Needs per-wheel slip angles; else a "
      "per-wheel slip speed (Automobilista 2, Le Mans Ultimate), turned into an angle less the straight-line angle learned on the "
      "straights (baseline: learn, learn once, or fixed at base deg F / R; the tile shows both, relearn starts it "
      "over); else the per-wheel combined slip (any sliding tyre, spinning and locking included), from onset % of "
      "peak % up to full at peak % (the chip shows the value arriving and its peak); else the single skid channel. "
      "Nothing at a standstill.",
      kSlipLatKeys, { 1.0, 20.0, 1.0, 11.0, 7.0, 40.0, 1.0, 8.0, 30.0, 1.0, 0.3, 50.0, 30.0, 0.6, 6.0, 0.0, 0.0, 0.0 } },
    { Effect::Road,        "road",        "Road",         Kind::Road, EventType::COUNT, FxType::Road,
      { "roadHeight*|suspVel*|suspTravel*|roadNoise", "speedKmh", "~surface*", "~accHeave", nullptr },
      { 0.0, 16.0, 0.0, 0.6 }, kRoadParams, static_cast<int>(std::size(kRoadParams)),
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
      "cobbles; rough x 0 = the sim's travel alone. Both: susp % is how much of the sim's own movement plays "
      "(its suspension, or its road under the tyre) beside the roughness, and susp cut hz a steep cut on that "
      "alone (0 = off): the body's slow movement the motion already gives drops away, the road's bumps stay. "
      "The old surface grain (surface x, hz, km/h) is still there, "
      "off by default. Without per-corner data, a texture at freq hz from the roadNoise channel, full at peak %.",
      nullptr, {} },
    { Effect::Limiter,     "limiter",     "Limiter",      Kind::Continuous, EventType::COUNT, FxType::Limiter,
      { "limiter", nullptr }, { 0.0, 12.0, 0.0, 0.15 }, kTextureParams, static_cast<int>(std::size(kTextureParams)),
      "Extra hammer on top of the engine effect while the limiter is in (the engine effect already cuts "
      "bursts of firings for the bounce). Keep it slow, ~10-15 hz. The plugin computes the flag from rpm vs the car max.",
      nullptr, {} },
    { Effect::Tc,          "tc",          "TC pulse",     Kind::Tc, EventType::COUNT, FxType::TcPulse,
      { "tcActive", nullptr }, { 0.0, 15.0, 0.0, 0.0 }, kTcParams, static_cast<int>(std::size(kTcParams)),
      "Traction control cutting: the body surge of each cut, a loss of drive and a recovery about freq hz times a "
      "second (the Engine tile's TC cut x adds an ignition-cut stutter if you want one). sharp is the cut's edge "
      "(0 round, 1 a hard knock), spread makes each cut a little different in timing and depth; hold ms keeps the "
      "sim's TC flag on over gaps that short (it flickers as TC cuts in and out), so TC working plays as one "
      "(0 = off). Nothing with the throttle shut: there is nothing to cut. Route it to surge and the belt. Needs "
      "the tcActive channel.", nullptr, {} },
    { Effect::Kerb,        "kerb",        "Rumble strip", Kind::Kerb, EventType::COUNT, FxType::Kerb,
      { "surface*|curbs", "speedKmh", nullptr }, { 0.0, 16.0, 0.0, 0.4 }, kKerbParams, static_cast<int>(std::size(kKerbParams)),
      "The rumble of a kerb's ribs under the tyre the game says is on one: ribs every pitch cm, so they hum at "
      "speed / pitch and the hum follows speed, each rib a little different (jitter). The car riding up onto "
      "the kerb is the motion's. A belt or shaker gets the ribs and a kick on and off, stronger with speed; a "
      "post a small bump on and off (it cannot carry the hum). Route with a part so the corner on the kerb "
      "rumbles. Plays only in a game that says when a tyre is on a kerb: its surface type under each tyre "
      "(Automobilista 2, Le Mans Ultimate, BeamNG.drive), else its own kerb vibration (Competizione, Rally: one "
      "value for the car, so every wheel, full at peak %). A game that says neither (Assetto Corsa) has no "
      "rumble strip here: the motion, Road and Impacts carry its kerbs.",
      nullptr, {} },
    { Effect::Driveline,   "driveline",   "Driveline",    Kind::Driveline, EventType::COUNT, FxType::Driveline,
      { "clutchPct", "rpm", "gear", "~speedKmh", "~throttlePct" }, { 0.0, 10.0, 0.0, 0.2 }, kDrivelineParams, static_cast<int>(std::size(kDrivelineParams)),
      "The transmission when the engine and the wheels disagree. Clutch: a slipping clutch grabbing and releasing "
      "at a launch or a bad downshift, the whole driveline shuddering at clutch hz, from how much slip (engine rpm "
      "against what the gear and road speed say, using the learned ratios), how much load, and how far into the "
      "slipping band the pedal is; rolling, the pedal held there a quarter second first (a shift's quick dip is "
      "no slip). Lug: full throttle at too few revs winding the driveline up and letting go at "
      "lug hz, fading as the revs climb out of it. Gearbox shapes the gear-shift thunk: a synchro box clunks "
      "softly, a dog box knocks hard and, shifted under power, knocks again as the dogs engage. Whine is "
      "straight-cut gears meshing, its pitch stepping with every gear and its level with load. Shunt is the "
      "backlash taking up when the throttle snaps open or shut, in gear and rolling. Clutch, lug, whine and shunt: "
      "nothing in neutral. Diff (0 = off): a limited-slip's plates clunking as the driven wheels are made to turn "
      "apart at walking pace (full lock out of the pits, turning round), harder under power (in neutral or with "
      "the clutch down, its preload alone), never under braking; or a spool's inside tyre scrubbing below 30 km/h. "
      "Silent on a racing lap, as a healthy diff is. It needs per-wheel wheel speeds. Needs clutch, "
      "rpm and gear (speed and throttle make it exact).", nullptr, {} },
    { Effect::Surface,     "surface",     "Surface",      Kind::Surface, EventType::COUNT, FxType::Surface,
      { "~surface*", "speedKmh", "~rain|wet", nullptr }, { 0.0, 60.0, 0.0, 0.0 }, kSurfaceParams, static_cast<int>(std::size(kSurfaceParams)),
      "What the ground does under each tyre beyond the road's shape, per corner (route with a part). Stones: "
      "on gravel, sand and dirt the tyres strike stones, more of them the faster you go, a sharp tick on a belt "
      "or shaker and a small bump on a post. Crunch: snow compacting under the tread (sand lighter, mud a low "
      "squelch). Studs: studded tyres biting on ice and snow, a rough buzz. Puddles: on a wet road, puddles "
      "laid out along it; the water drags at a tyre as it enters (stronger with speed), above aqua km/h in "
      "standing water the tyre floats and the road under it goes quiet, and it bites as it comes out. smooth x "
      "is how far water fills the Road tile's texture. Plays the surface under each tyre the sim sends "
      "(Automobilista 2, Le Mans Ultimate), else the ground set under no surface sent (gravel for a rally type of "
      "car; tarmac, nothing to play, for the rest), and the wetness or the rain.", nullptr, {} },
    { Effect::Wheels,      "wheels",      "Wheels",       Kind::Wheels, EventType::COUNT, FxType::Wheels,
      { "speedKmh", "~slipRatio*|wheelSpeed*", "~brakePct", "~load*", "~deflated*" }, { 0.0, 12.0, 0.0, 0.0 }, kWheelsParams, static_cast<int>(std::size(kWheelsParams)),
      "What each wheel does as it turns, felt the way a car passes it on: through the corner's tyre, spring and "
      "damper (the Road tile's body hz, hop hz and damping), a smooth shake at the wheel's own rate rather than a "
      "knock per revolution. Balance: the wheel slightly out of balance (and a touch out of round), a shake that "
      "grows steadily with speed. Flat: a lock-up grinds a flat spot into the tyre where it slid, deeper the "
      "longer and harder it slid, and it thumps softly once a revolution from then on, wearing slowly round "
      "again. Judder: discs made hot by hard braking pulse the braking force twice a revolution while you "
      "brake (best on a belt, surge or a shaker). Puncture: when the sim says a tyre is deflated (Automobilista "
      "2, Le Mans Ultimate), the wheel rides on its folded sidewall, a heavy thump once a revolution. Bite and drag: the pads on "
      "the turning disc, a grab as the brake goes on and a rub while it stays on, each a rough grain at its own "
      "hz (at 100 km/h, following the wheel's speed), at the caliper rather than through the suspension; "
      "nothing from a locked wheel. A wheel in the air does none of the rest. Each wheel turns at its own "
      "speed, so the corners drift in and out of step; route with a part so each post plays its own corner. "
      "Fresh tyres wipes the flats and cools the discs. Needs the speed; per-wheel slip ratios or wheel speeds "
      "for each wheel's own speed and the lock-ups, the brake for the judder and the pads.",
      nullptr, {} },
    { Effect::Impacts,     "impacts",     "Impacts",      Kind::Impacts, EventType::COUNT, FxType::Impacts,
      { "suspVel*|suspTravel*|accHeave|accSurge|accSway", nullptr }, { 0.0, 40.0, 50.0, 0.0 }, kImpactParams, static_cast<int>(std::size(kImpactParams)),
      "The edge of a hit, which the motion cue rounds off and never sends to a belt or shaker: a corner's "
      "suspension stopping or starting dead between two sim samples (the bump stop on a big compression, a "
      "landing, a kerb strike, the suspension topping out), from from mm/s of sudden change, full at full mm/s; "
      "and a sudden jump in the body's vertical g (heave g, 0 = off), or in its g along and across the car (a "
      "wall, a contact with another car: hit g, 0 = off), on every corner; never with the car standing "
      "(a reset drops it onto the ground). A belt or shaker gets a "
      "knock ringing at ring hz and dying over ring ms; a post one short jolt, up for a compression, down for a "
      "top-out (keep its route small: the motion already moves the body). Route with a part so a post knocks "
      "for its own corner. Needs the suspension velocity or travel per corner, or the body's accelerations "
      "(hits need the SimHub plugin 1.9).",
      nullptr, {} },
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
        case 9: return s.budget;
        case 10: return s.screech;
        case 11: return s.screechPct;
        case 12: return s.screechHz;
        case 13: return s.pulse;
        case 14: return s.pulseHz;
        case 15: return s.baseMode;
        case 16: return s.baseDegF;
        case 17: return s.baseDegR;
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
