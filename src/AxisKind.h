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

struct AxisCaps
{
    // ---- primitive shape (mirrors the config strings) ----
    bool beltType   = false;   // axisType == "belt"
    bool horizontal = false;   // axisType == "linear_horizontal"
    bool rotary     = false;   // axisType == "rotary_lever" (unit = degrees)
    bool torqueMode = false;   // mode == "torque"
    bool ppMode     = false;   // mode == "pp"

    // ---- engine semantics ----
    bool homes        = true;  // participates in homing + homed/ready gates
                               // (belts skip homing and are exempt from both)
    bool positionPark = true;  // parks via position interp (belts ease tension)
    bool seatable     = false; // deinit seat pass: vertical linear CSP only
    // Commissioning classification: 0 = vertical role (pitch/roll/heave),
    // 1 = horizontal (solo segment), 2 = excluded (torque/belt devices).
    int  commissioningKind = 0;
    const char* unit = "mm";   // display unit for logs/UI
};

inline AxisCaps axisCaps(const std::string& axisType, const std::string& mode)
{
    AxisCaps c;
    c.beltType   = (axisType == "belt");
    c.horizontal = (axisType == "linear_horizontal");
    c.rotary     = (axisType == "rotary_lever");
    c.torqueMode = (mode == "torque");
    c.ppMode     = (mode == "pp");

    c.homes        = !c.beltType;
    c.positionPark = !c.beltType;
    // Seat: literal legacy gate -- axisType must BE "linear_vertical" (an
    // unknown type string was never seatable), and not torque/PP.
    c.seatable = (axisType == "linear_vertical") && !c.torqueMode && !c.ppMode;
    c.commissioningKind = (c.torqueMode || c.beltType) ? 2
                        : c.horizontal ? 1 : 0;
    c.unit = c.rotary ? "deg" : "mm";
    return c;
}
