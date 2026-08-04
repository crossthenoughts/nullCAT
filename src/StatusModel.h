// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// StatusModel.h — shared, platform-neutral status model
//
// One canonical status truth for every renderer (web cards, PC/Qt rows,
// NeoPixel/HID box). It takes the engine's existing snapshot (AxisMotionState +
// DriveState/statusword + system flags) and produces, per axis, a canonical
// indicator bucket + verbatim detail text + retained RAW state, plus an aggregate
// and a single state->colour/blink table.
//
// This codifies the model the web UI implements (app.js card logic +
// app.css .s-* palette + logo precedence) so behaviour is unchanged on the web and
// the PC surface can adopt the identical model.
//
// No Qt, no web, no hardware deps — pure derivation over engine enums.
// ============================================================

#include <cstdint>
#include <string>
#include "A6Drive.h"          // DriveState (DS402 electrical) — header is dep-light
#include "MotionController.h"  // AxisMotionState

namespace status {

// Six canonical indicator buckets — the colour vocabulary the web already uses
// (s-op / s-home / s-park / s-off / s-fault / s-estop).
enum class Indicator { RUNNING, BUSY, IDLE, OFFLINE, FAULT, ESTOP };

// Per-axis canonical result. `state`+`text` are what a renderer shows; the raw
// fields are retained so a later model-only refinement can un-fold the current
// BLENDING->RUNNING / UNPARKING->IDLE merges without touching any renderer.
struct AxisIndicator
{
    Indicator       state      = Indicator::OFFLINE;
    std::string     text;                 // verbatim text to display (electrical or motion)
    bool            fault      = false;   // true for FAULT (per-axis fault flag)
    int             axisIndex  = -1;
    // ---- retained raw inputs (for future un-folding + detail) ----
    AxisMotionState rawMotion  = AxisMotionState::PARKED;
    DriveState      rawDrive   = DriveState::Unknown;
    uint16_t        statusword = 0;
};

// State -> colour/blink, defined ONCE as data. Renderers map these into their
// medium (web: webClass; Qt: hex + pattern; NeoPixel: hex + pattern). A renderer
// given a state has no colour choice -> the surfaces cannot diverge.
struct Style
{
    const char* name;      // "RUNNING" ...
    const char* webClass;  // "s-op" ... (matches web/app.css)
    const char* hexLight;  // light theme  (matches --ok/--warn/--danger/--ink-soft)
    const char* hexDark;   // dark theme
    const char* pattern;   // "solid" | "solid-dim" | "pulse" | "blink"
};

// The single colour/blink table. Deterministic: same state -> same style always.
const Style& styleOf(Indicator s);

// Stable lowercase token for JSON/transport (e.g. "running","fault").
const char* indicatorToken(Indicator s);

// ---- Per-axis derivation (verbatim from the current web app.js card logic) ----
// 1. electrical fault (decoded from statusword) wins -> FAULT
// 2. else, if loop running AND the motion state is "meaningful"
//    (HOMING / PARKED / PARKING / UNPARKING / ESTOPPING) -> the motion state
// 3. else -> the electrical (DS402) state
// motionName is the engine's pre-formatted axisStateName (e.g. "HOMING[search]")
// so sub-state detail is preserved in `text`.
// hasStatusword distinguishes "no electrical data" (drive not OP -> text empty,
// OFFLINE) from "statusword == 0" (NOT READY) -- matches the web's `'sw' in d`.
AxisIndicator deriveAxis(int axisIndex,
                         AxisMotionState motion, const std::string& motionName,
                         bool hasStatusword, uint16_t statusword, DriveState rawDrive,
                         bool loopRunning, bool estop);

// ---- Aggregate / summary (codifies the current web logo precedence) ----
// ESTOP > FAULT (any axis) > RUNNING (loop running) > OFFLINE.
Indicator deriveAggregate(const AxisIndicator* axes, int count,
                          bool loopRunning, bool estop);

// ---- Rig-level boolean aggregates ----
// THE single definition of the booleans that drive /api/status fields and the
// toggle endpoints' server-side resolution (WebServer::buildStatusJson +
// park-toggle + belts-toggle). A second hand-derived copy would let those
// surfaces disagree about "parked"/"slack".
// Semantics — quirks included — are pinned by TestStatusModel; change the
// rules here and in the tests together, never at a call site.

struct MotionAggregates
{
    bool allParked    = false;  // numDrives > 0 AND every axis PARKED
    bool anyHoming    = false;  // any axis HOMING
    // Any axis mid-transition: PARKING / UNPARKING / HOMING / ESTOPPING.
    // park-toggle refuses while true (toggles are no-ops, never reversals).
    // NOTE: BLENDING is deliberately NOT in this set (pinned legacy behavior:
    // park-toggle during a blend resolves "park", and startPark handles
    // BLENDING axes).
    bool transitional = false;
};
MotionAggregates deriveMotionAggregates(const AxisMotionState* states, int numDrives);

struct BeltAggregates
{
    bool hasBelts     = false;  // any config axis in torque mode
    bool beltsSlack   = false;  // hasBelts AND every torque axis PARKED
    // Any TORQUE axis mid-transition: PARKING / BLENDING / UNPARKING /
    // ESTOPPING. belts-toggle refuses while true.
    bool transitional = false;
};
// isTorque[i] = config drive i is mode "torque". numConfig may exceed
// numStates; a torque axis beyond the engine snapshot counts as slack
// (preserved quirk of the original derivations).
BeltAggregates deriveBeltAggregates(const AxisMotionState* states, int numStates,
                                    const bool* isTorque, int numConfig);

} // namespace status
