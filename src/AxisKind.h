// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// AxisKind - THE axis classification authority.
//
// Every "what kind of axis is this" decision in the engine goes through
// axisCaps(axisType, mode) instead of scattered string compares, so adding
// a device kind (rotary lever today; H-shifter / active brake in the
// torque-device family later) is one row here, not a codebase sweep.
// TestAxisKind pins every field against the legacy string-compare
// expressions across the full type x mode matrix - the refactor is
// behaviour-identical by construction, and future edits that change a
// classification fail that golden test loudly.
//
// Deliberate non-client: Config.cpp's validator keeps its own
// (mode=="cst"||mode=="torque") tests - "cst" is a legacy config alias
// tolerated at the validation layer only; the engine has never honoured
// it, and routing it through here would silently widen engine behaviour.
// ============================================================

#include <string>
#include <cstdint>

// The family is what the per-cycle fork in MotionController::process()
// switches on: Position axes track telemetry targets, Belt renders the
// tensioner's unidirectional force map, and the control-loading device
// families (Shifter, Pedal) render a signed force field from their own
// encoder position. One new device = one new family value + one row below.
enum class AxisFamily : uint8_t { Position, Belt, Shifter, Pedal };

// How the axis establishes its position reference. Position axes run the
// CSP torque-search HomingSequence; device families are CST (no position
// PDO), so they run the torque-only stall search instead; belts have no
// position reference at all.
enum class HomingKind : uint8_t { None, Csp, Torque };

struct AxisCaps
{
    // ---- primitive shape (mirrors the config strings) ----
    bool beltType   = false;   // axisType == "belt"
    bool horizontal = false;   // axisType == "linear_horizontal"
    bool rotary     = false;   // axisType == "rotary_lever" (unit = degrees)
    bool torqueMode = false;   // mode == "torque"
    bool ppMode     = false;   // mode == "pp"

    // ---- family + engine semantics ----
    AxisFamily family     = AxisFamily::Position;
    HomingKind homingKind = HomingKind::Csp;
    bool homes        = true;  // participates in homing + homed/ready gates
                               // (belts skip homing and are exempt from both)
    bool positionPark = true;  // parks via position interp (belts and devices
                               // ease force to zero instead)
    bool seatable     = false; // deinit seat pass: vertical linear CSP only
    // DECLARED, not inferred: this axis's mechanics hold position when the
    // drive is de-energised (self-locking gearbox or brake). Rotary levers
    // declare true (the 50:1 box); a capstan-driven shifter is back-drivable
    // and declares false. Consumed by the seat-pass eligibility below.
    bool holdsPositionUnpowered = false;
    // Commissioning classification: 0 = vertical role (pitch/roll/heave),
    // 1 = horizontal (solo segment), 2 = excluded (torque/belt devices).
    int  commissioningKind = 0;
    const char* unit = "mm";   // display unit for logs/UI

    bool isDevice() const
    { return family == AxisFamily::Shifter || family == AxisFamily::Pedal; }
};

inline AxisCaps axisCaps(const std::string& axisType, const std::string& mode)
{
    AxisCaps c;
    c.beltType   = (axisType == "belt");
    c.horizontal = (axisType == "linear_horizontal");
    c.rotary     = (axisType == "rotary_lever");
    c.torqueMode = (mode == "torque");
    c.ppMode     = (mode == "pp");

    const bool shifter = (axisType == "shifter");
    const bool pedal   = (axisType == "pedal");
    c.family = c.beltType ? AxisFamily::Belt
             : shifter    ? AxisFamily::Shifter
             : pedal      ? AxisFamily::Pedal
             : AxisFamily::Position;

    c.homingKind = c.beltType       ? HomingKind::None
                 : c.isDevice()     ? HomingKind::Torque
                 : HomingKind::Csp;
    c.homes        = (c.homingKind != HomingKind::None);
    c.positionPark = !c.beltType && !c.isDevice();
    // Self-locking mechanics, declared per row: today only the geared
    // rotary lever. Devices on back-drivable capstans stay false.
    c.holdsPositionUnpowered = c.rotary;
    // Seat: literal legacy gate -- axisType must BE "linear_vertical" (an
    // unknown type string was never seatable), and not torque/PP. An axis
    // whose mechanics hold unpowered never needs seating (declared above;
    // linear_vertical declares false, so this clause changes nothing today).
    c.seatable = (axisType == "linear_vertical") && !c.torqueMode && !c.ppMode
              && !c.holdsPositionUnpowered;
    c.commissioningKind = (c.torqueMode || c.beltType) ? 2
                        : c.horizontal ? 1 : 0;
    // Device force models work in motor-shaft revolutions.
    c.unit = c.isDevice() ? "rev" : c.rotary ? "deg" : "mm";
    return c;
}
