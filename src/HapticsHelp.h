// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// HapticsHelp - the hover text for every haptic setting: what it does, a
// sensible range, and starting points for a road car, a GT3 car and a
// single-seater. Served with /api/haptics/schema (each param's "help"); the
// strip shows it when the pointer rests on a setting. Keyed by the tile's
// key and the setting's key; a setting many tiles share (amp %, jitter,
// peak %) has one common text under "*" unless a tile needs its own.
// Starting points, not answers: every rig and every driver differs.
// ============================================================

#include <cstring>

namespace haptics {

struct ParamHelp { const char* effect; const char* key; const char* text; };

namespace help_detail {

constexpr ParamHelp kHelp[] = {
    // ---- shared ----
    { "*", "ampPct",
      "How strong the effect is at full severity, 0 to 100 (0 = off). Set it by feel on your rig: start "
      "around 40 to 60 and raise it until the effect is clear without hiding the others." },
    { "*", "jitter",
      "Roughens the carrier so it reads as a texture, not a tone. 0 to 1; 0.2 to 0.5 suits most. The same "
      "for any car." },
    { "*", "peakPct",
      "Only for the single-channel fallback: the channel value that counts as full severity, 1 to 400. "
      "Drive a few laps, read the peak on the channel chip and set it there." },
    { "*", "freqHz",
      "The carrier: lower is a heavier, slower shake, higher a finer buzz. A seat actuator carries up to "
      "about 20 Hz well; belts and shakers go higher." },
    { "*", "durMs",
      "How long each burst lasts, 5 to 100 ms." },

    // ---- detent click ----
    { "detentClick", "freqHz", "The click's pitch, 10 to 500 Hz. 60 to 120 for a crisp click on a shifter." },
    { "detentClick", "durMs", "The click's length, 5 to 100 ms. 12 to 25 for a short, clean click." },

    // ---- gear shift ----
    { "gearShift", "freqHz",
      "The thunk's ring on a belt or shaker, 10 to 500 Hz (a seat actuator always gets one jolt). "
      "Road car: 40 to 60. GT3: 60 to 80. Single-seater: 70 to 100 (a sharper knock)." },
    { "gearShift", "durMs",
      "How long the thunk rings, 5 to 100 ms. Road car: 30 to 40 (a soft clunk). GT3: 20 to 30. "
      "Single-seater: 15 to 25." },

    // ---- engine ----
    { "rpmVibe", "cylinders",
      "Cylinders (or rotors for a Wankel), 1 to 16. Road car: 4. GT3: 6 (flat-six) or 8 (V8). "
      "Single-seater: 4 (F3, F4) or 6 (a modern F1 V6)." },
    { "rpmVibe", "litres",
      "Engine size, 0.1 to 30 litres: bigger cylinders thump harder. Road car: 1.6 to 2.5. GT3: 4.0 "
      "(flat-six) to 6.3 (V8). Single-seater: 1.6 (F1 V6) to 3.4 (F3)." },
    { "rpmVibe", "layout",
      "How the cylinders sit, which sets how smooth the engine is. Road car: inline. GT3: flat / boxer "
      "(Porsche) or V. Single-seater: V (F1) or inline (F3, F4)." },
    { "rpmVibe", "maxRpm",
      "The redline, 0 to 30000 (0 = learn it while you drive, which works well). Road car: 6000 to 7000. "
      "GT3: 7500 to 9500. Single-seater: 7500 (F3) to 15000 (F1)." },
    { "rpmVibe", "rock",
      "The block rocking at idle, a slow lumpy sway that fades by about 2500 rpm, 0 to 1. Road car: 0.4 to "
      "0.6. GT3: 0.6 to 0.9. Single-seater: 0.3 to 0.5 (they idle high)." },
    { "rpmVibe", "thump",
      "Each firing as a low thump, 0 to 1. Road car: 0.3 to 0.5. GT3: 0.6 to 0.9 (a V8 thumps hardest). "
      "Single-seater: 0.3 to 0.5." },
    { "rpmVibe", "buzz",
      "The high buzz above idle, rising with revs, 0 to 1. Road car: 0.3 to 0.5. GT3: 0.6 to 0.8. "
      "Single-seater: 0.7 to 1 (a screaming engine)." },
    { "rpmVibe", "order",
      "The buzz's pitch per engine revolution, 0 to 4 (0 = automatic, which suits most). Lower it (0.25 to "
      "0.5) for a high-revving engine so the top of the range still fits what the actuator can carry." },
    { "rpmVibe", "limHit",
      "How hard the rev limiter bounces, 0 to 2. Road car: 0.6 to 1 (a soft cut). GT3: 1 to 1.5. "
      "Single-seater: 1.2 to 2 (a hard cut)." },
    { "rpmVibe", "limHz",
      "How fast the limiter bounces, 4 to 30 Hz. Road car: 6 to 10. GT3: 10 to 15. Single-seater: 15 to 25." },
    { "rpmVibe", "limJit",
      "How uneven the limiter bounce is, 0 to 1. 0.2 to 0.4 for most; higher for a rough old car." },
    { "rpmVibe", "inertia",
      "The shake that stays when you lift at high revs and fades as they fall, 0 to 1. Road car: 0.2 to 0.4. "
      "GT3: 0.4 to 0.7. Single-seater: 0.5 to 0.8." },
    { "rpmVibe", "turbo",
      "Turbocharged: a lift-off is a blow-off whoosh and flutter instead of a pop. Road car: often yes. GT3: "
      "no for most (yes for some, like the Ferrari 296). Single-seater: yes for F1, no for F3 and F4." },
    { "rpmVibe", "liftoff",
      "The pop (or turbo whoosh) when you snap the throttle shut high in the revs, 0 to 2. Road car: 0.3 to "
      "0.6. GT3: 0.8 to 1.2. Single-seater: 0.6 to 1." },
    { "rpmVibe", "pops",
      "The crackle on the overrun, 0 to 1. Road car: 0 to 0.2. GT3: 0.4 to 0.8. Single-seater: 0.2 to 0.5." },
    { "rpmVibe", "tcCut",
      "How hard the engine stutters while the sim's traction control cuts, 0 to 1 (0 = not at all). A modern "
      "TC trims torque smoothly; an ignition-cut TC stutters. Road car: 0. GT3: 0 to 0.3. Single-seater: 0. "
      "Rally or an older car with a cut-based TC: 0.4 to 0.8. The TC tile plays the drive easing either way." },
    { "rpmVibe", "freqHz",
      "The weight of each firing thump, 10 to 80 Hz: lower is heavier. Road car: 30 to 40. GT3: 22 to 30. "
      "Single-seater: 30 to 45." },
    { "rpmVibe", "jitter",
      "The idle lope, how uneven the idle is, 0 to 1. Road car: 0.05 to 0.15. GT3: 0.2 to 0.4 (a lumpy cam). "
      "Single-seater: 0.1 to 0.2." },

    // ---- ABS ----
    { "abs", "freqHz",
      "How many times a second each corner's valve dumps and rebuilds the pressure, 4 to 60 Hz. Road car: 8 to "
      "12 (older systems slower). GT3: 12 to 18. Single-seater: most have no ABS." },
    { "abs", "sharp",
      "The dump's edge, 0 to 1: 0 a round surge, 1 a hard knock. Road car: 0.2 to 0.4. GT3: 0.4 to 0.6. "
      "Single-seater: no ABS." },
    { "abs", "spread",
      "Each corner on its own rate and each cycle a little different, 0 to 1: the car grumbles rather than "
      "beats. 0.4 to 0.7 for most cars; 0 gives one regular pulse." },
    { "abs", "slow",
      "How far the cycle slows towards a stop, 0 to 1 (0.5 = half rate at a standstill). 0.4 to 0.6 for "
      "most cars." },
    { "abs", "buzz",
      "The pump and valves buzzing underneath, 0 to 1. Best on a belt or shaker. Road car: 0.3 to 0.5 (you "
      "hear and feel the pump). GT3: 0.1 to 0.3. Single-seater: no ABS." },
    { "abs", "buzzHz",
      "The pump buzz's pitch, 15 to 120 Hz. 35 to 50 for most cars." },
    { "abs", "brakeFull",
      "ABS pulses as hard as you brake: nothing at 10% pedal, full at this brake %, 10 to 100 (10 = always "
      "full). 70 to 85 for most; lower it if light trail braking should still be clearly felt." },
    { "abs", "holdMs",
      "Keeps the sim's ABS flag on over gaps this short, 0 to 500 ms (0 = off, the flag as sent). Some sims "
      "(Assetto Corsa) blip the flag through a stop, which chops the pulse into fragments; 100 to 200 joins "
      "them into one steady cycle. The same for any car: it depends on the sim." },
    { "abs", "slipLink",
      "While ABS works, each corner's valve shapes that wheel's slip tiles (the lock texture held at the edge "
      "of locking, the lateral slip easing after each dump). Works with ABS's own amp at 0." },
    { "abs", "lockLink",
      "With slip link on: how much the lock texture surges with each valve cycle, 0 to 1. 0.5 to 0.8." },
    { "abs", "scrubLink",
      "With slip link on: how much the lateral slip eases after each dump, 0 to 1. 0.2 to 0.4." },

    // ---- longitudinal slip ----
    { "slipLon", "lock",
      "A wheel locking under braking, a heavy judder, 0 to 1. Road car: 0.8 to 1. GT3: 0.8 to 1. "
      "Single-seater: 1 (no ABS: lock-ups are part of driving it)." },
    { "slipLon", "lockHz",
      "The lock judder's pitch at 80 km/h (it falls as the car slows), 4 to 30 Hz. Road car: 8 to 12. GT3: "
      "12 to 18. Single-seater: 14 to 20." },
    { "slipLon", "spin",
      "A driven wheel spinning, the tread chattering, 0 to 1. Road car: 0.6 to 0.8. GT3: 0.8 to 1. "
      "Single-seater: 0.8 to 1." },
    { "slipLon", "spinHz",
      "The spin chatter's pitch, 4 to 30 Hz (a live axle tramps lower, around 10). Road car: 10 to 14. GT3: "
      "14 to 18. Single-seater: 16 to 20." },
    { "slipLon", "peakRatio",
      "The slip ratio that counts as a full lock or spin, 0.2 to 2 (nothing plays inside 0.15). Road car: 0.6 "
      "to 0.8. GT3: 0.4 to 0.6. Single-seater: 0.3 to 0.5 (slicks let go at less slip)." },

    // ---- lateral slip ----
    { "slipLat", "scrub",
      "The fronts pushing wide (understeer), a fine texture, 0 to 1. 0.8 to 1 for most cars." },
    { "slipLat", "scrubHz",
      "The scrub texture's pitch, 8 to 60 Hz. A seat actuator carries 15 to 20 best; belts and shakers can "
      "go to 30. Road car: 15 to 20. GT3: 18 to 22. Single-seater: 20 to 25." },
    { "slipLat", "screech",
      "The fronts starting to squeal before they let go: a light texture from screech % of peak deg, full at "
      "peak deg, 0 to 1. The early cue that you are near the limit. 0.2 to 0.4 for most." },
    { "slipLat", "screechPct",
      "Where the screech starts, as a share of peak deg, 0 to 95%. 40 to 60 for most; lower it for an earlier "
      "warning." },
    { "slipLat", "screechHz",
      "The screech's pitch, 10 to 120 Hz: best above the scrub, on a belt or shaker (a seat actuator carries "
      "little of it). 25 to 40 for most." },
    { "slipLat", "pulse",
      "Fronts past the limit sticking and slipping in a slow, irregular shove, as strong as the scrub, 0 to "
      "1. Slow enough for a seat actuator to carry. 0.4 to 0.8 for most." },
    { "slipLat", "pulseHz",
      "How fast the understeer pulse comes, 2 to 15 Hz. Road car: 4 to 6 (soft tyres, slower). GT3: 5 to 7. "
      "Single-seater: 6 to 9." },
    { "slipLat", "slide",
      "The rears stepping out (oversteer), an irregular chatter, 0 to 1. 0.8 to 1 for most cars." },
    { "slipLat", "slideHz",
      "The rear slide chatter's pitch, 4 to 30 Hz. Road car: 9 to 12. GT3: 12 to 16. Single-seater: 14 to 18." },
    { "slipLat", "peakDeg",
      "The slip angle (degrees) that counts as a full slide, 2 to 20. Road car: 10 to 14 (street tyres slide "
      "progressively). GT3: 8 to 10. Single-seater: 6 to 8." },
    { "slipLat", "onsetPct",
      "Where a slide starts, as a share of peak deg, 0 to 90%. 0 with a high ease gives the most progressive "
      "way in; 30 to 50 keeps ordinary cornering quiet. GT3 on a bench: 0 with ease 2 felt right." },
    { "slipLat", "ease",
      "The shape of the way in, 0 to 2: 0 rises straight to full, 1 starts gently (severity squared), 2 "
      "gentler still. 1 to 2 for most; 2 lets you feel the load come and go as you adjust your line." },
    { "slipLat", "smoothHz",
      "Smooths the slip angle each wheel reports (it arrives about 60 times a second and can jitter), 0 to 30 "
      "Hz (0 = off). Lower is calmer but lags; higher follows faster. 8 to 12 for most." },
    { "slipLat", "attackMs",
      "How fast a slide builds once it starts, 2 to 200 ms. 2 to 10 for a sharp, responsive feel; 30 to 60 "
      "for a softer one. Road car: 20 to 40. GT3: 2 to 15. Single-seater: 2 to 10." },
    { "slipLat", "budget",
      "Grip budget: a tyre has one grip for cornering and for braking or driving, so trail braking and power "
      "out of a corner slide earlier, as they do in a car. On for most." },
    { "slipLat", "peakPct",
      "For sims without a slip angle (combined slip, the skid channel): the value that counts as a full slide, "
      "1 to 400. Read the chip's peak after a few laps." },

    // ---- road ----
    { "road", "model",
      "How the road is made. Suspension (the default) replays the sim's suspension and adds the road's fine "
      "roughness; tyre runs a model of each corner over the road; chassis plays the sim's own body movement. "
      "Suspension suits most sims." },
    { "road", "bodyMm",
      "Tyre and chassis models: the body movement (mm) that counts as 100%, 0.1 to 20. 0.5 to 1.5 for most; "
      "set it to your route gain to play the body 1:1." },
    { "road", "hpHz",
      "Cut: road movement slower than this is left to the motion cue, 0.5 to 10 Hz. 1.5 to 3 for most; higher "
      "if the haptics fight your motion software." },
    { "road", "rough",
      "The road's fine roughness, laid out by distance so it rises with speed, 0 to 20 (1 = a smooth public "
      "road, 0 = the sim's road alone). Road car: 1.5 to 3. GT3: 1.5 to 2.5. Single-seater: 1 to 2." },
    { "road", "suspPct",
      "Suspension and tyre models: how much of the sim's own movement plays (its suspension, or its road under "
      "the tyre) beside the roughness, 0 to 100 (0 = the roughness alone). 100 keeps it all. With susp cut hz "
      "set: road car 60 to 80 (long soft travel is mostly body), GT3 80 to 100, single-seater 100." },
    { "road", "suspHz",
      "A steep cut on the sim's own movement alone, 0 to 15 Hz (0 = off: cut hz only). Body movement well "
      "below it (dive, roll, crests, the motion's job) drops away fast; bumps above it stay. Set it above the "
      "car's body bounce: road car 3 to 4, GT3 5 to 7, single-seater 7 to 9." },
    { "road", "bodyHz",
      "The body's bounce on its springs, 0.8 to 8 Hz. Road car: 1.2 to 1.6. GT3: 3 to 4. Single-seater: 4 to 6." },
    { "road", "hopHz",
      "The wheel's hop on its tyre, 6 to 30 Hz. Road car: 10 to 13. GT3: 14 to 17. Single-seater: 18 to 22." },
    { "road", "damping",
      "How quickly the body settles after a bump, 0.05 to 1.5 (damping ratio). Road car: 0.2 to 0.3 (soft). "
      "GT3: 0.3 to 0.45. Single-seater: 0.4 to 0.6." },
    { "road", "fullMm",
      "Suspension model: the suspension travel (mm) that counts as 100%, 0.5 to 50. Road car: 40 to 50 (long "
      "travel). GT3: 25 to 35. Single-seater: 15 to 20 (stiff, short travel)." },
    { "road", "surface",
      "The old tarmac grain, a texture rising with speed on top of the road, 0 to 1 (off by default: the "
      "roughness does its job). 0.2 to 0.5 if you want extra texture." },
    { "road", "surfaceHz", "The tarmac grain's pitch, 4 to 40 Hz. 10 to 16 for a seat actuator." },
    { "road", "surfaceKmh", "The speed by which the tarmac grain is at full, 20 to 300 km/h. 100 to 150 for most." },
    { "road", "freqHz", "The roadNoise texture's pitch when the sim sends no per-corner road, 10 to 120 Hz. 14 to 20." },

    // ---- limiter ----
    { "limiter", "freqHz",
      "The extra hammer on the rev limiter, 10 to 120 Hz. Keep it slow: 10 to 15. Road car: 10. GT3: 12 to 15. "
      "Single-seater: 15 to 20." },

    // ---- TC ----
    { "tc", "freqHz",
      "How many traction-control cuts a second, 4 to 60 Hz. Road car: 5 to 10. GT3: 10 to 20. Single-seater: "
      "most have no TC." },
    { "tc", "sharp",
      "Each cut's edge, 0 to 1: 0 a round surge, 1 a hard knock. Road car: 0.1 to 0.3. GT3: 0.3 to 0.5." },
    { "tc", "spread",
      "Each cut a little different in timing and depth, 0 to 1. 0.3 to 0.6 for most." },
    { "tc", "holdMs",
      "Keeps the sim's TC flag on over gaps this short, 0 to 500 ms (0 = off, the flag as sent). The flag "
      "flickers as traction control cuts in and out, which plays as a stutter; 100 to 200 makes TC working "
      "one continuous effect. The same for any car: it depends on the sim." },

    // ---- kerb ----
    { "kerb", "pitchCm",
      "Rib spacing on the rumble strip, 5 to 100 cm: the hum's pitch is speed divided by this. Set by the "
      "track, not the car: 20 to 30 for most circuit kerbs, 40 to 60 for road rumble strips." },
    { "kerb", "riseMm",
      "The step up onto the kerb, 0 to 50 mm. 5 to 10 for flat circuit kerbs, 15 to 30 for sausage kerbs." },
    { "kerb", "ribMm", "Rib height, 0 to 20 mm. 2 to 5 for most circuit kerbs." },
    { "kerb", "fullMm",
      "Kerb movement (mm) that counts as 100% on a seat actuator, 0.5 to 50. Road car: 12 to 15 (soft, rides "
      "it out). GT3: 8 to 12. Single-seater: 5 to 8 (stiff, feels every rib)." },
    { "kerb", "detectMm",
      "For sims with no surface type (Assetto Corsa): a tyre this many mm above its axle partner is on a "
      "kerb (from half of it, full at it), 0 to 100 (0 = off). 12 to 20 for raised kerbs; 8 to 10 if low "
      "kerbs do not register. Flat painted kerbs have no height to find." },

    // ---- driveline ----
    { "driveline", "clutch",
      "A slipping clutch grabbing and releasing at a launch or a bad downshift, 0 to 1. Road car: 0.8 to 1. "
      "GT3: 0.5 to 0.8. Single-seater: 0.4 to 0.6 (only at the start)." },
    { "driveline", "clutchHz", "The clutch judder's pitch, 4 to 20 Hz. 8 to 12 for most." },
    { "driveline", "lug",
      "Full throttle at too few revs, the driveline winding up, 0 to 1. Road car: 0.8 to 1. GT3: 0.5 to 0.8. "
      "Single-seater: 0.3 to 0.5." },
    { "driveline", "lugHz", "The lugging wind-up's pitch, 3 to 15 Hz. 5 to 8 for most." },
    { "driveline", "gearbox",
      "Shapes the gear-shift thunk. Road car: synchro (a soft clunk). GT3: dog (a hard knock; a second knock "
      "on a shift under power). Single-seater: dog." },
    { "driveline", "whine",
      "Straight-cut gears singing, rising with each gear, 0 to 1 (0 = quiet helical gears). Road car: 0. "
      "GT3: 0.3 to 0.6. Single-seater: 0.4 to 0.8." },
    { "driveline", "shunt",
      "Backlash taking up when the throttle snaps open or shut, 0 to 1. Road car: 0.3 to 0.5. GT3: 0.4 to 0.7. "
      "Single-seater: 0.5 to 0.8." },
    { "driveline", "shuntHz", "The shunt knock's pitch, 20 to 80 Hz. 35 to 50 for most." },

    // ---- surface ----
    { "surface", "stones",
      "Stones struck on gravel, sand and dirt, busier with speed, 0 to 1. 0.5 to 0.8 for most cars (rally: up "
      "to 1)." },
    { "surface", "crunch", "Snow (and sand, mud) compacting under the tread, 0 to 1. 0.4 to 0.7." },
    { "surface", "studs", "Studded tyres buzzing on ice and snow. On only for rally cars on studs." },
    { "surface", "puddles",
      "The water dragging at a tyre into a puddle and biting coming out, 0 to 1. 0.5 to 0.8 for most." },
    { "surface", "aquaKmh",
      "The speed a tyre floats at in standing water, 40 to 250 km/h. Road car: 80 to 90. GT3 on wets: 120 to "
      "140. Single-seater on wets: 140 to 160. Any car on slicks: 60 to 70." },
    { "surface", "smooth",
      "How far water fills the road's texture under a tyre, 0 to 1. 0.6 to 0.8 for most." },

    // ---- wheels ----
    { "wheels", "balance",
      "The wheel slightly out of balance, a shake growing with speed, 0 to 4 (1 = a typical wheel). Road car: "
      "1 to 1.5. GT3: 0.5 to 1. Single-seater: 0.3 to 0.6." },
    { "wheels", "flat",
      "Flat spots ground in by your lock-ups, a soft thump once a revolution, 0 to 4 (x their depth). 1 for "
      "most; raise it to feel lighter flats." },
    { "wheels", "judder",
      "Hot brake discs pulsing the braking force, 0 to 1. Fore and aft: route it to a belt, surge or a shaker. "
      "Road car: 0.4 to 0.7 (iron discs). GT3: 0.2 to 0.4. Single-seater: 0 to 0.2 (carbon brakes)." },
    { "wheels", "puncture",
      "A punctured tyre (when the sim says so) riding on its sidewall, a heavy thump once a revolution, 0 to "
      "2. 1 for most." },

    // ---- impacts ----
    { "impacts", "freqHz",
      "Ring hz: the knock's ring on a belt or shaker, 15 to 120 Hz. Road car: 30 to 40. GT3: 40 to 50. "
      "Single-seater: 50 to 60 (a stiffer chassis rings higher)." },
    { "impacts", "durMs",
      "Ring ms: how fast the knock dies, 10 to 200 ms. Road car: 60 to 80. GT3: 40 to 60. Single-seater: 30 "
      "to 50." },
    { "impacts", "fromMmS",
      "A sudden change in a corner's suspension speed (mm/s) that starts a knock, 100 to 3000. Raise it if "
      "rough tracks knock all the time. Road car: 400 to 600. GT3: 500 to 800. Single-seater: 800 to 1200 "
      "(stiff cars jolt routinely)." },
    { "impacts", "fullMmS",
      "The sudden change that is a full knock, 300 to 8000 mm/s. Road car: 2000 to 2500. GT3: 2500 to 3500. "
      "Single-seater: 3500 to 5000." },
    { "impacts", "heaveG",
      "A jump in the body's vertical g that knocks every corner (a landing), 0 to 5 (0 = off). 1.5 to 2.5 for "
      "most; rally: 1 to 1.5." },
};

} // namespace help_detail

// The hover text for a tile's setting: its own, else the shared one, else
// nullptr.
inline const char* paramHelp(const char* effect, const char* key)
{
    if (!effect || !key) return nullptr;
    const char* shared = nullptr;
    for (const ParamHelp& h : help_detail::kHelp)
    {
        if (std::strcmp(h.key, key) != 0) continue;
        if (std::strcmp(h.effect, effect) == 0) return h.text;
        if (h.effect[0] == '*' && h.effect[1] == '\0') shared = h.text;
    }
    return shared;
}

} // namespace haptics
