// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestTelemetryParse.cpp — pins the telemetry UDP wire-format semantics.
//
// parsePacket runs ON THE RT THREAD at telemetry rate as a zero-allocation
// span walker; this suite is the byte-for-byte wire contract any rewrite
// must pass unchanged.
//
// Wire truth (rig-verified): every field after the NULLCAT header is an
// axis value — there is NO timestamp field on the wire (timestampMs is
// always 0).
//
//   ctest -R TelemetryParse
// ============================================================
#include "TelemetryInput.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>

static int g_fail = 0, g_pass = 0;
static void check(bool ok, const char* name)
{
    if (ok) { ++g_pass; std::printf("[PASS] %s\n", name); }
    else    { ++g_fail; std::printf("[FAIL] %s\n", name); }
}

static TelemetryData parse(const char* line, bool* okOut = nullptr)
{
    TelemetryData d{};
    bool ok = TelemetryInput::parsePacket(line, (int)std::strlen(line), d);
    if (okOut) *okOut = ok;
    return d;
}

int main()
{
    // ---- canonical motion line ----
    {
        bool ok; TelemetryData d = parse("NULLCAT,10.5,-5.2,0.0,22.1", &ok);
        check(ok && d.valid, "canonical: accepted");
        check(d.packetType == TelemetryPacketType::Motion, "canonical: Motion type");
        check(d.numPositions == 4, "canonical: 4 positions");
        check(std::fabs(d.positions[0] - 10.5) < 1e-12 &&
              std::fabs(d.positions[1] + 5.2)  < 1e-12 &&
              std::fabs(d.positions[3] - 22.1) < 1e-12, "canonical: values exact");
        check(d.timestampMs == 0, "canonical: timestampMs always 0 (no wire field)");
    }

    // ---- legacy lifecycle tokens are gone: plain rejects, never motion ----
    {
        bool ok; parse("SIMHUB_START", &ok);
        check(!ok, "reject: legacy SIMHUB_START (no comma, wrong header)");
        parse("simhub_stop", &ok);
        check(!ok, "reject: legacy simhub_stop");
        TelemetryData d = parse("NULLCAT_START", &ok);
        check(!ok && d.numPositions == 0, "reject: underscore token is not a motion packet");
    }

    // ---- header tolerance ----
    {
        bool ok; TelemetryData d = parse("nullcat,1.0", &ok);
        check(ok && d.numPositions == 1, "header: case-insensitive");
        d = parse("  NULLCAT , 1.0 , 2.0 \n", &ok);
        check(ok && d.numPositions == 2, "header+fields: whitespace tolerated");
        d = parse("NULL CAT,3.5", &ok);   // embedded whitespace in header is stripped
        check(ok && d.numPositions == 1 && std::fabs(d.positions[0] - 3.5) < 1e-12,
              "header: embedded whitespace stripped (legacy tolerance)");
    }

    // ---- rejects ----
    {
        bool ok; parse("GARBAGE,1,2", &ok);
        check(!ok, "reject: wrong header");
        parse("NULLCAT", &ok);
        check(!ok, "reject: no comma");
        parse("", &ok);
        check(!ok, "reject: empty line");
        TelemetryData d = parse("NULLCAT,", &ok);
        check(!ok && d.numPositions == 0, "reject: header with no fields (valid=false)");
        parse("NULLCAT,abc,xyz", &ok);
        check(!ok, "reject: no numeric fields at all");
    }

    // ---- field-level tolerance (pinned quirks of the original parser) ----
    {
        bool ok; TelemetryData d = parse("NULLCAT,1.0,,2.0", &ok);
        check(ok && d.numPositions == 2 &&
              std::fabs(d.positions[1] - 2.0) < 1e-12,
              "empty field skipped, later fields still parsed (positions COMPACT)");
        d = parse("NULLCAT,1.0,abc,2.0", &ok);
        check(ok && d.numPositions == 2, "non-numeric field skipped (compacting)");
        d = parse("NULLCAT,1.0,2.0,", &ok);
        check(ok && d.numPositions == 2, "trailing comma harmless");
        d = parse("NULLCAT,1.5e2,-3E-1", &ok);
        check(ok && d.numPositions == 2 &&
              std::fabs(d.positions[0] - 150.0) < 1e-12 &&
              std::fabs(d.positions[1] + 0.3)   < 1e-12, "scientific notation via strtod");
    }

    // ---- clamp at MAX_DRIVES ----
    {
        std::string line = "NULLCAT";
        for (int i = 0; i < 14; ++i) line += "," + std::to_string(i + 0.5);
        bool ok; TelemetryData d = parse(line.c_str(), &ok);
        check(ok && d.numPositions == MAX_DRIVES, "field count clamped at MAX_DRIVES");
        check(std::fabs(d.positions[MAX_DRIVES - 1] - (MAX_DRIVES - 1 + 0.5)) < 1e-12,
              "clamp keeps the FIRST MAX_DRIVES fields in order");
    }

    // ---- long-but-legal numeric field (headroom for the fixed-buffer rewrite) ----
    {
        bool ok; TelemetryData d = parse("NULLCAT,00000000000000000000000001.53125", &ok);
        check(ok && d.numPositions == 1 && std::fabs(d.positions[0] - 1.53125) < 1e-12,
              "40-char numeric field parses (fixed buffer must fit realistic max)");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
