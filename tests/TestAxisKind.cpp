// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestAxisKind.cpp - golden test for the axis classification authority.
//
// axisCaps() replaced scattered string compares across MotionController,
// EtherCATMaster, and WebServer. This test pins every capability field
// against the LEGACY expressions (copied verbatim from the pre-refactor
// call sites) over the full axisType x mode matrix, including unknown
// strings - the refactor is behaviour-identical by construction, and any
// future edit that changes a classification fails here loudly.
// Run with: ctest -R AxisKind
// ============================================================
#include "AxisKind.h"
#include <cstdio>
#include <cstring>
#include <string>

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("  FAIL: %s  (line %d)\n", (msg), __LINE__); } \
} while (0)

int main()
{
    const char* types[] = { "linear_vertical", "linear_horizontal",
                            "rotary_lever", "belt", "linear_center",
                            "", "bogus_type" };
    const char* modes[] = { "csp", "pp", "torque", "cst", "" };

    for (const char* atc : types)
    for (const char* mdc : modes)
    {
        const std::string at = atc, md = mdc;
        const AxisCaps c = axisCaps(at, md);
        char msg[128];

        // ---- legacy expressions, verbatim from the pre-refactor sites ----

        // MotionController configure(): dc.axisType == "belt"
        const bool legIsBelt = (at == "belt");
        // MotionController allAxesHomed/allAxesReady/homing skip:
        // belts exempt (axisType == "belt" -> continue)
        const bool legHomingExempt = (at == "belt");
        // MotionController startPark(): belts ease tension, not position
        const bool legBeltPark = (at == "belt");
        // MotionController seat pass: linear_vertical AND not torque/PP
        const bool legSeat = (at == "linear_vertical")
                          && !(md == "torque") && !(md == "pp");
        // EtherCATMaster init position: belt -> 0, else homingBackoff
        const bool legInitZero = (at == "belt");
        // WebServer commissioning meta kind
        const int legKind = (md == "torque" || at == "belt") ? 2
                          : (at == "linear_horizontal") ? 1 : 0;
        // MotionController mode flags
        const bool legTorque = (md == "torque");   // NB: legacy "cst" alias
        const bool legPp     = (md == "pp");       // is validation-layer only

        std::snprintf(msg, sizeof(msg), "beltType [%s/%s]", atc, mdc);
        CHECK(c.beltType == legIsBelt, msg);
        std::snprintf(msg, sizeof(msg), "homes [%s/%s]", atc, mdc);
        CHECK(!c.homes == legHomingExempt, msg);
        std::snprintf(msg, sizeof(msg), "positionPark [%s/%s]", atc, mdc);
        CHECK(!c.positionPark == legBeltPark, msg);
        std::snprintf(msg, sizeof(msg), "seatable [%s/%s]", atc, mdc);
        CHECK(c.seatable == legSeat, msg);
        std::snprintf(msg, sizeof(msg), "init-zero [%s/%s]", atc, mdc);
        CHECK(!c.homes == legInitZero, msg);
        std::snprintf(msg, sizeof(msg), "commissioningKind [%s/%s]", atc, mdc);
        CHECK(c.commissioningKind == legKind, msg);
        std::snprintf(msg, sizeof(msg), "torqueMode [%s/%s]", atc, mdc);
        CHECK(c.torqueMode == legTorque, msg);
        std::snprintf(msg, sizeof(msg), "ppMode [%s/%s]", atc, mdc);
        CHECK(c.ppMode == legPp, msg);
    }

    // ---- semantic invariants (new, not legacy-derived) ----
    CHECK(std::strcmp(axisCaps("rotary_lever", "csp").unit, "deg") == 0,
          "rotary unit is degrees");
    CHECK(std::strcmp(axisCaps("linear_vertical", "csp").unit, "mm") == 0,
          "linear unit is mm");
    CHECK(axisCaps("rotary_lever", "csp").commissioningKind == 0,
          "rotary lever takes the vertical commissioning role");
    CHECK(!axisCaps("rotary_lever", "csp").seatable,
          "rotary lever is never seated (50:1 boxes hold on disable)");
    CHECK(axisCaps("belt", "torque").commissioningKind == 2,
          "belt torque axis excluded from commissioning");
    // Unknown future type: behaves as a generic position axis, never seated.
    const AxisCaps u = axisCaps("hexapod_strut", "csp");
    CHECK(u.homes && u.positionPark && !u.seatable && u.commissioningKind == 0,
          "unknown type = generic position axis, not seatable");

    // ---- Stage C: the "is a belt" command/status predicate ----
    // The belts controls (slack/tension, aggregates, Qt button) key on
    // beltType && torqueMode. Across the whole matrix this equals the old
    // mode-only gate everywhere EXCEPT the two degenerate mismatches, where
    // the new predicate is false (a no-op, the safe direction): a torque
    // axis of a non-belt type (the 0.9.5 shifter/pedal family) must never
    // be gripped by the belts button, and a belt-typed position axis can't
    // accept torque commands.
    for (const char* t : { "linear_vertical", "linear_horizontal",
                           "rotary_lever", "belt", "hexapod_strut" })
        for (const char* m : { "csp", "pp", "torque" })
        {
            const AxisCaps c = axisCaps(t, m);
            const bool isBelt = c.beltType && c.torqueMode;
            const bool oldGate = (std::strcmp(m, "torque") == 0);
            const bool matched = (std::strcmp(t, "belt") == 0) == oldGate;
            if (matched)
                CHECK(isBelt == oldGate, "belt predicate == legacy gate on matched combos");
            else
                CHECK(!isBelt, "mismatched type/mode is never a belt (safe no-op)");
        }

    std::printf("TestAxisKind: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
