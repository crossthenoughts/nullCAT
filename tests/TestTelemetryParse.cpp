// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestTelemetryParse.cpp - pins the telemetry UDP wire-format semantics.
//
// parsePacket runs ON THE RT THREAD at telemetry rate as a zero-allocation
// span walker; this suite is the byte-for-byte wire contract any rewrite
// must pass unchanged.
//
// Wire truth (rig-verified): every field after the NULLCAT header is an
// axis value - there is NO timestamp field on the wire (timestampMs is
// always 0).
//
//   ctest -R TelemetryParse
// ============================================================
#include "TelemetryInput.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <thread>
#include <chrono>

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

    // ---- NULLCATX raw-channel packets (0.9.5 device state wire) ----
    {
        bool ok; TelemetryData d = parse("NULLCATX,6500,142.3,3,0", &ok);
        check(ok && d.packetType == TelemetryPacketType::Ncx,
              "NULLCATX header parses as an Ncx packet");
        check(d.numNcx == 4 &&
              std::fabs(d.ncx[0] - 6500.0) < 1e-12 &&
              std::fabs(d.ncx[1] - 142.3)  < 1e-12,
              "channels land in ncx[], in order");
        check(d.numPositions == 0,
              "an Ncx packet carries NO motion positions");

        d = parse("nullcatx,1.0", &ok);
        check(ok && d.packetType == TelemetryPacketType::Ncx,
              "NULLCATX header is case-insensitive like NULLCAT");
        d = parse("NULLCATX,1.0,,abc,2.0", &ok);
        check(ok && d.numNcx == 2, "Ncx fields skip-and-compact like motion fields");
        parse("NULLCATXX,1.0", &ok);
        check(!ok, "reject: NULLCATXX is not a header");
        parse("NULLCATX", &ok);
        check(!ok, "reject: Ncx header with no comma");

        std::string line = "NULLCATX";
        for (int i = 0; i < 60; ++i) line += "," + std::to_string(i + 0.25);
        d = parse(line.c_str(), &ok);
        check(ok && d.numNcx == MAX_NCX_CHANNELS && MAX_NCX_CHANNELS == 48
              && std::fabs(d.ncx[47] - 47.25) < 1e-12,
              "channel count clamped at MAX_NCX_CHANNELS (48 since protocol 1.3), first 48 kept");
        check(!d.ncxFresh,
              "parsePacket never sets ncxFresh (getLatestData owns freshness)");
    }

    // ---- NULLCATY named packets (protocol 1.3) ----
    {
        bool ok; TelemetryData d = parse("NULLCATY,rpm=6500,slipAngleFL=-4.25, suspVelRR = 120 ,game=AMS2,car=Formula Ultimate Gen2", &ok);
        check(ok && d.packetType == TelemetryPacketType::Ncy, "NULLCATY header parses as an Ncy packet");
        check(d.ncyHave[NcxTok::Rpm] && std::fabs(d.ncy[NcxTok::Rpm] - 6500.0) < 1e-12,
              "a named token lands under its token index");
        check(d.ncyHave[NcxTok::SlipAngleFL] && std::fabs(d.ncy[NcxTok::SlipAngleFL] + 4.25) < 1e-12,
              "per-wheel tokens are named exactly (slipAngleFL)");
        check(d.ncyHave[NcxTok::SuspVelRR] && std::fabs(d.ncy[NcxTok::SuspVelRR] - 120.0) < 1e-12,
              "whitespace around key, '=' and value is ignored");
        check(!d.ncyHave[NcxTok::SpeedKmh] && !d.ncyHave[NcxTok::Gear],
              "tokens the line did not name are NOT present");
        check(std::string(d.game) == "AMS2" && std::string(d.car) == "Formula Ultimate Gen2",
              "game= and car= strings are copied (spaces kept)");
        check(d.numNcx == 0 && d.numPositions == 0, "an Ncy packet carries no slots and no positions");

        d = parse("NULLCATY,boostBar=1.2,RPM=100,rpm,=5,rpm=abc,gear=3", &ok);
        check(ok && d.ncyHave[NcxTok::Gear] && !d.ncyHave[NcxTok::Rpm],
              "unknown keys, wrong case, missing '=', empty key and non-numeric values are skipped; valid pairs still land");
        parse("NULLCATY,boostBar=1.2", &ok);
        check(!ok, "reject: an Ncy packet with nothing recognised is not valid");
        parse("NULLCATY", &ok);
        check(!ok, "reject: Ncy header with no comma");
        d = parse("nullcaty,game=x", &ok);
        check(ok && std::string(d.game) == "x", "NULLCATY header is case-insensitive");

        std::string longCar = "NULLCATY,car=";
        for (int i = 0; i < 100; ++i) longCar += 'c';
        d = parse(longCar.c_str(), &ok);
        check(ok && std::strlen(d.car) == NCY_STR_LEN - 1, "an over-long name is truncated to the store, never overrun");
    }

    // ---- frozen-stream guard: OFF by default ----
    // An idling car at a standstill sends exactly the same numbers for as
    // long as it idles (integer idle rpm, zero speed, zero pedals), so the
    // default is to keep trusting an unchanging stream.
    {
        TelemetryInput ti;
        auto feed = [&](const char* line) { TelemetryData p; TelemetryInput::parsePacket(line, (int)std::strlen(line), p); ti.ingest(p); };
        auto wait = [](int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); };
        check(ti.frozenMs() == 0, "the guard is off unless host.json turns it on");
        feed("NULLCATX,2000,0,0");
        for (int i = 0; i < 12; ++i) { wait(200); feed("NULLCATX,2000,0,0"); }
        TelemetryData s = ti.getLatestData();
        check(s.ncxFresh && !s.ncxFrozen, "guard off: an idling car's unchanging stream stays fresh past 2 s");
    }

    // ---- frozen-stream guard (ingest path, no socket), turned on ----
    // Packets that keep arriving with every value unchanged for 2 s are a
    // paused sim or a sender replaying its last frame: the stream reads as
    // NOT fresh (effects release) and as frozen, until a value changes.
    // An identity-only NULLCATY line is neither a change nor a repeat; a
    // NULLCATY value merged over the X stream counts like any other.
    {
        TelemetryInput ti;
        ti.setFrozenMs(NCX_FROZEN_MS);
        auto feed = [&](const char* line) { TelemetryData p; TelemetryInput::parsePacket(line, (int)std::strlen(line), p); ti.ingest(p); };
        auto wait = [](int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); };

        feed("NULLCATX,5000,100,3");
        TelemetryData s = ti.getLatestData();
        check(s.ncxFresh && !s.ncxFrozen, "fresh on the first packet");

        // Repeats for ~2.3 s (well past NCX_FROZEN_MS), keeping the packet
        // cadence under the 500 ms window.
        for (int i = 0; i < 12; ++i) { wait(200); feed("NULLCATX,5000,100,3"); }
        s = ti.getLatestData();
        check(!s.ncxFresh && s.ncxFrozen, "identical packets for 2 s: not fresh, frozen");
        check(s.numNcx == 3 && s.ncx[0] == 5000.0, "the values are still there, just not trusted");

        feed("NULLCATX,5001,100,3");
        s = ti.getLatestData();
        check(s.ncxFresh && !s.ncxFrozen, "one changed value thaws the stream at once");

        // Identity-only lines in between do not count as changes.
        for (int i = 0; i < 12; ++i) { wait(200); feed("NULLCATX,5001,100,3"); feed("NULLCATY,game=X,car=Y"); }
        s = ti.getLatestData();
        check(!s.ncxFresh && s.ncxFrozen, "identity-only lines do not keep a frozen stream fresh");

        // A changed NULLCATY value does.
        feed("NULLCATY,rpm=5002");
        s = ti.getLatestData();
        check(s.ncxFresh && !s.ncxFrozen, "a changed named value thaws the stream");

        // Stopped: no packet for 500 ms is neither fresh nor frozen.
        wait(600);
        s = ti.getLatestData();
        check(!s.ncxFresh && !s.ncxFrozen, "no packets for 500 ms: stopped, not frozen");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
