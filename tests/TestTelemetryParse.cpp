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
#include <filesystem>

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
        // Slots are positional (B16): a bad field keeps its slot and is
        // marked absent, so slot 3 is still 2.0 and the rpm/speed/gear
        // bindings never slide onto a neighbour's value. Reverting the
        // "if (isNcx)" branch of parsePacket to the compacting path gives
        // numNcx == 2 with 2.0 in slot 1: every check below fails.
        d = parse("NULLCATX,1.0,,abc,2.0", &ok);
        check(ok && d.numNcx == 4, "Ncx: a bad field keeps its slot (numNcx counts to the last present slot)");
        check(d.ncxHave[0] && std::fabs(d.ncx[0] - 1.0) < 1e-12, "Ncx: slot 0 present");
        check(!d.ncxHave[1] && d.ncx[1] == 0.0, "Ncx: an empty field is an absent slot (value 0)");
        check(!d.ncxHave[2] && d.ncx[2] == 0.0, "Ncx: a non-numeric field is an absent slot (value 0)");
        check(d.ncxHave[3] && std::fabs(d.ncx[3] - 2.0) < 1e-12, "Ncx: the field after the gap is STILL slot 3");
        d = parse("NULLCATX,7,8,", &ok);
        check(ok && d.numNcx == 2 && !d.ncxHave[2], "Ncx: a trailing empty field adds no slot");
        parse("NULLCATX,,abc,", &ok);
        check(!ok, "reject: an Ncx line with no present slot at all");
        d = parse("NULLCAT,1.0,,abc,2.0", &ok);
        check(ok && d.numPositions == 2 && std::fabs(d.positions[1] - 2.0) < 1e-12,
              "the motion line still compacts (axis values, pinned legacy behaviour)");
        parse("NULLCATXX,1.0", &ok);
        check(!ok, "reject: NULLCATXX is not a header");
        parse("NULLCATX", &ok);
        check(!ok, "reject: Ncx header with no comma");

        std::string line = "NULLCATX";
        for (int i = 0; i < 120; ++i) line += "," + std::to_string(i + 0.25);
        d = parse(line.c_str(), &ok);
        check(ok && d.numNcx == MAX_NCX_CHANNELS && MAX_NCX_CHANNELS == 96
              && std::fabs(d.ncx[95] - 95.25) < 1e-12,
              "channel count clamped at MAX_NCX_CHANNELS (96 since protocol 1.7), first 96 kept");
        check(NcxTok::TokenCount <= MAX_NCX_CHANNELS, "every token has a slot on the numbered line");
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
        check(d.carId[0] == '\0', "no carId= on the line: the id is empty (an older plugin)");
        check(d.numNcx == 0 && d.numPositions == 0, "an Ncy packet carries no slots and no positions");

        d = parse("NULLCATY,game=AssettoCorsa,car=Mazda 787B,carId=ks_mazda_787b", &ok);
        check(ok && std::string(d.car) == "Mazda 787B" && std::string(d.carId) == "ks_mazda_787b",
              "carId= lands beside car= (SimHub CarId: the content folder for AC)");

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

    // ---- The telemetry recorder: every datagram, in order, to a file ----
    {
        namespace fs = std::filesystem;
        // Under the working (build) folder, on a real disk: the Pi's /tmp is a
        // RAM disk under the recorder's 1 GB minimum.
        const fs::path dir = fs::current_path() / "nullcat-rec-test";
        fs::remove_all(dir);
        TelemetryRecorder rec;
        rec.push("NULLCATY,rpm=1\n", 15);   // off: ignored
        check(rec.status().packets == 0 && rec.status().dropped == 0, "recorder: off takes nothing (one atomic load)");
        std::string why;
        check(rec.start(dir.string(), why), "recorder: starts into a new folder");
        if (!why.empty()) std::printf("  recorder refused: %s\n", why.c_str());
        std::string why2;
        check(!rec.start(dir.string(), why2) && !why2.empty(), "recorder: a second start while recording is refused");
        char line[64];
        for (int i = 0; i < 100; ++i)
        {
            const int n = std::snprintf(line, sizeof line, "NULLCATY,rpm=%d\n", i);
            rec.push(line, n);
        }
        const std::string file = rec.status().file;
        rec.stop();
        const TelemetryRecorder::Status st = rec.status();
        check(!st.on && st.packets == 100 && st.dropped == 0, "recorder: 100 datagrams written, none dropped");
        std::FILE* f = std::fopen((dir / file).string().c_str(), "rb");
        int lines = 0, ok = 0; char buf[256];
        if (f)
        {
            while (std::fgets(buf, sizeof buf, f))
            {
                if (lines > 0)
                {
                    const char* tab = std::strchr(buf, '\t');
                    char want[64]; std::snprintf(want, sizeof want, "NULLCATY,rpm=%d\n", lines - 1);
                    if (tab && std::strcmp(tab + 1, want) == 0) ++ok;
                }
                ++lines;
            }
            std::fclose(f);
        }
        check(lines == 101 && ok == 100, "recorder: a header, then each datagram in order as ms<TAB>datagram (its own line ending dropped)");
        const auto countLines = [&](const std::string& name) {
            int n = 0; char b[256];
            if (std::FILE* g = std::fopen((dir / name).string().c_str(), "rb")) { while (std::fgets(b, sizeof b, g)) ++n; std::fclose(g); }
            return n; };
        // A flood from another thread (the receive path) while the writer
        // runs: never blocks, and every datagram is either written or counted.
        // Started again within the same second: a new file, the first kept.
        check(rec.start(dir.string(), why), "recorder: starts again after a stop");
        const std::string file2 = rec.status().file;
        std::thread producer([&] { for (int i = 0; i < 5000; ++i) rec.push("NULLCATY,speedKmh=100\n", 22); });
        producer.join();
        rec.stop();
        const TelemetryRecorder::Status s2 = rec.status();
        std::printf("  recorder flood: %llu written, %llu dropped\n", static_cast<unsigned long long>(s2.packets), static_cast<unsigned long long>(s2.dropped));
        check(s2.packets + s2.dropped == 5000, "recorder: under a flood every datagram is written or counted as dropped");
        check(s2.packets >= TelemetryRecorder::kSlots && countLines(file2) == static_cast<int>(s2.packets) + 1,
              "recorder: ...at least a ring's worth written, every one of them in the file");
        check(file2 != file && countLines(file) == 101, "recorder: a restart in the same second writes a new file and keeps the last");
        fs::remove_all(dir);
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
