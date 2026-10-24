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
#include <vector>
#include <algorithm>
#include <atomic>

static int g_fail = 0, g_pass = 0;
static void check(bool ok, const char* name)
{
    if (ok) { ++g_pass; std::printf("[PASS] %s\n", name); }
    else    { ++g_fail; std::printf("[FAIL] %s\n", name); }
}

// Park a thread wherever it is (a stand-in for the scheduler taking it
// off the core mid-call) and let it go again. Windows: SuspendThread, with
// a GetThreadContext to be sure it has actually stopped. Linux: a signal
// whose handler waits on a semaphore (sem_wait is async-signal-safe).
#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  include <windows.h>
static void parkThread(std::thread& t)
{
    HANDLE h = static_cast<HANDLE>(t.native_handle());
    SuspendThread(h);
    CONTEXT ctx; ctx.ContextFlags = CONTEXT_CONTROL;
    GetThreadContext(h, &ctx);   // returns only once the thread is really stopped
}
static void unparkThread(std::thread& t) { ResumeThread(static_cast<HANDLE>(t.native_handle())); }
#else
#  include <csignal>
#  include <semaphore.h>
#  include <pthread.h>
static sem_t g_parkSem;
static std::atomic<bool> g_parked{false};
static void parkHandler(int) { g_parked.store(true); sem_wait(&g_parkSem); g_parked.store(false); }
static void parkThread(std::thread& t)
{
    static bool armed = false;
    if (!armed) { sem_init(&g_parkSem, 0, 0); std::signal(SIGUSR1, parkHandler); armed = true; }
    pthread_kill(t.native_handle(), SIGUSR1);
    while (!g_parked.load()) std::this_thread::yield();
}
static void unparkThread(std::thread&) { sem_post(&g_parkSem); }
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <unistd.h>
#endif

// One datagram to the loopback port a TelemetryInput is listening on
// (the socket library is already up: initialize() started it).
static void sendUdp(int port, const char* line)
{
    sockaddr_in to{};
    to.sin_family = AF_INET;
    to.sin_port   = htons(static_cast<uint16_t>(port));
    to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
#ifdef _WIN32
    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sendto(s, line, static_cast<int>(std::strlen(line)), 0, reinterpret_cast<sockaddr*>(&to), sizeof(to));
    closesocket(s);
#else
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sendto(s, line, std::strlen(line), 0, reinterpret_cast<sockaddr*>(&to), sizeof(to));
    close(s);
#endif
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

    // ---- protocol 1.9: the pit flag, the lap and the tyre wear, by name,
    // at slots 75..80 (appended: nothing before them moved); a line
    // without them leaves them absent ----
    {
        check(NcxTok::SlipSpeedRR == 74 && NcxTok::InPit == 75 && NcxTok::Lap == 76 && NcxTok::TyreWearFL == 77
              && NcxTok::TyreWearRR == 80 && NcxTok::TokenCount == 81,
              "protocol 1.9: inPit, lap and tyreWearFL..RR take slots 75..80, after slipSpeedRR");
        check(std::string(ncxTokenName(NcxTok::InPit)) == "inPit" && std::string(ncxTokenName(NcxTok::Lap)) == "lap"
              && std::string(ncxTokenName(NcxTok::TyreWearFL)) == "tyreWearFL" && std::string(ncxTokenName(NcxTok::TyreWearRR)) == "tyreWearRR",
              "protocol 1.9: the names are the wire's");
        bool ok; TelemetryData d = parse("NULLCATY,inPit=1,lap=12,tyreWearFL=0.25,tyreWearFR=0.3,tyreWearRL=0.35,tyreWearRR=0.4", &ok);
        check(ok && d.ncyHave[NcxTok::InPit] && d.ncy[NcxTok::InPit] == 1.0 && d.ncyHave[NcxTok::Lap] && d.ncy[NcxTok::Lap] == 12.0,
              "protocol 1.9: inPit and lap land by name");
        check(d.ncyHave[NcxTok::TyreWearFL] && d.ncyHave[NcxTok::TyreWearRR] && std::fabs(d.ncy[NcxTok::TyreWearRR] - 0.4) < 1e-12,
              "protocol 1.9: the tyre wear lands per wheel by name");
        d = parse("NULLCATY,rpm=5000,speedKmh=100", &ok);
        check(ok && !d.ncyHave[NcxTok::InPit] && !d.ncyHave[NcxTok::Lap] && !d.ncyHave[NcxTok::TyreWearFL],
              "protocol 1.9: a 1.8 line leaves them absent (unknown, never 0)");
    }

    // ---- non-finite values never land (A5) ----
    // strtod happily reads "nan", "inf", "infinity" and overflows to inf;
    // a sender that prints a NaN property (SimHub does, as "NaN") would
    // otherwise push it through every law and freeze the status page.
    // Reverting the std::isfinite(val) in either strtod branch of
    // parsePacket fails these.
    {
        bool ok; TelemetryData d = parse("NULLCATX,nan,2", &ok);
        check(ok && d.numNcx == 2 && !d.ncxHave[0] && d.ncx[0] == 0.0 && d.ncxHave[1],
              "Ncx: a NaN field is an absent slot (value 0), the slot after it still lands");
        d = parse("NULLCATX,1e999,-inf,infinity,3", &ok);
        check(ok && d.numNcx == 4 && !d.ncxHave[0] && !d.ncxHave[1] && !d.ncxHave[2] && d.ncxHave[3],
              "Ncx: an overflow, -inf and infinity are absent slots");
        bool allFinite = true;
        for (int i = 0; i < d.numNcx; ++i) allFinite = allFinite && std::isfinite(d.ncx[i]);
        check(allFinite, "Ncx: every stored slot value is finite, present or not");
        parse("NULLCATX,nan,inf", &ok);
        check(!ok, "reject: an Ncx line with nothing finite on it");
        d = parse("NULLCATY,rpm=nan,speedKmh=inf,gear=3", &ok);
        check(ok && !d.ncyHave[NcxTok::Rpm] && !d.ncyHave[NcxTok::SpeedKmh] && d.ncyHave[NcxTok::Gear],
              "Ncy: NaN and inf tokens are NOT present, the finite one lands");
        check(d.ncy[NcxTok::Rpm] == 0.0 && d.ncy[NcxTok::SpeedKmh] == 0.0, "Ncy: an absent token's value stays 0");
        parse("NULLCATY,rpm=NaN", &ok);
        check(!ok, "reject: an Ncy line whose only value is NaN lands nothing");
        d = parse("NULLCAT,1.0,nan,2.0", &ok);
        check(ok && d.numPositions == 2 && std::fabs(d.positions[1] - 2.0) < 1e-12,
              "motion: a NaN field is skipped like any bad field (compacting), never an axis target");
        // Through the store: a NaN token ingested is absent in the snapshot.
        TelemetryInput ti;
        TelemetryData p; TelemetryInput::parsePacket("NULLCATY,rpm=nan,gear=2", 23, p); ti.ingest(p);
        TelemetryData s = ti.getLatestData();
        check(!s.ncyHave[NcxTok::Rpm] && s.ncyHave[NcxTok::Gear] && std::isfinite(s.ncy[NcxTok::Rpm]),
              "snapshot: a NaN token is absent (have=false) and its value finite");
    }

    // ---- named tokens do not outlive the game or car that sent them (B1) ----
    // A NULLCATY line may carry any subset of tokens (PROTOCOL.md), so a
    // token holds its value between lines; the hold used to end only after
    // a 500 ms gap, so a game or car switch with no gap kept the old game's
    // surface*, slipSpeed*, wet*, onGround*, deflated* (tokens the new game
    // never sends) frozen at their last values. Reverting the identity
    // change check in ingest() fails the "clears" checks below.
    {
        TelemetryInput ti;
        auto feed = [&](const char* line) { TelemetryData p; TelemetryInput::parsePacket(line, (int)std::strlen(line), p); ti.ingest(p); };
        feed("NULLCATY,rpm=1");
        feed("NULLCATY,game=AMS2,car=Formula V8");
        TelemetryData s = ti.getLatestData();
        check(s.ncyHave[NcxTok::Rpm], "the first identity line (none known before) keeps the values already received");
        feed("NULLCATY,rpm=5000,surfaceFL=3,surfaceFR=3,surfaceRL=3,surfaceRR=3,slipSpeedFL=1.5");
        s = ti.getLatestData();
        check(s.ncyHave[NcxTok::SurfaceFL] && s.ncyHave[NcxTok::SlipSpeedFL], "AMS2's own tokens are present");
        feed("NULLCATY,game=AMS2,car=Formula V8");
        s = ti.getLatestData();
        check(s.ncyHave[NcxTok::SurfaceFL] && s.ncyHave[NcxTok::Rpm], "the identity line repeating (once a second) keeps every value");
        feed("NULLCATY,game=AssettoCorsa,car=Mazda 787B,carId=ks_mazda_787b");   // a new game, no gap
        s = ti.getLatestData();
        check(!s.ncyHave[NcxTok::SurfaceFL] && !s.ncyHave[NcxTok::SurfaceRR] && !s.ncyHave[NcxTok::SlipSpeedFL] && !s.ncyHave[NcxTok::Rpm],
              "a game switch with no gap clears every named token");
        check(std::string(s.game) == "AssettoCorsa" && std::string(s.car) == "Mazda 787B" && std::string(s.carId) == "ks_mazda_787b",
              "...and the new identity is in place");
        feed("NULLCATY,rpm=3000,deflatedFL=1");
        s = ti.getLatestData();
        check(s.ncyHave[NcxTok::Rpm] && s.ncyHave[NcxTok::DeflatedFL] && !s.ncyHave[NcxTok::SurfaceFL],
              "the new game's values land; the old game's stay gone");
        feed("NULLCATY,game=AssettoCorsa,car=Porsche 911 GT3 R,carId=ks_porsche_911_gt3_r_2016");   // same game, new car
        s = ti.getLatestData();
        check(!s.ncyHave[NcxTok::DeflatedFL] && !s.ncyHave[NcxTok::Rpm], "a car change in the same game clears every named token too");
        feed("NULLCATY,wet=0.5");
        feed("NULLCATY,game=AssettoCorsa,car=Porsche 911 GT3 R,carId=ks_porsche_911_gt3_r_2016");
        s = ti.getLatestData();
        check(s.ncyHave[NcxTok::Wet], "the same car named again keeps its values");
        feed("NULLCATY,game=AssettoCorsa,car=Porsche 911 GT3 R");   // an older plugin: no id on the line
        s = ti.getLatestData();
        check(s.ncyHave[NcxTok::Wet] && s.carId[0] == '\0', "a line without carId= drops the stale id (as before) and is not a car change");
        // Two cars with the same name, told apart by their id alone (a mod
        // and the original, both "Porsche 911 GT3 R"): a change of id is a
        // car change. Breaks with the carId clause of the check removed.
        feed("NULLCATY,game=AssettoCorsa,car=Porsche 911 GT3 R,carId=ks_porsche_911_gt3_r_2016");
        feed("NULLCATY,wet=0.25");
        feed("NULLCATY,game=AssettoCorsa,car=Porsche 911 GT3 R,carId=rss_gtm_lux_v8");
        s = ti.getLatestData();
        check(!s.ncyHave[NcxTok::Wet] && std::string(s.carId) == "rss_gtm_lux_v8",
              "the same car name with another id is a car change: every named token clears");
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

    // ---- the receive path waits for nobody (B13) ----
    // The RT loop stores packets (ingest) and takes the snapshot
    // (getLatestData) every cycle; the web and status threads take the
    // same snapshot at their own pace. Neither side may make the RT side
    // wait: a status reader preempted while holding a lock would stall
    // the control cycle for as long as the scheduler left it there. The
    // store is a seqlock: the writer never waits, a reader that caught a
    // write in progress retries. With the old std::mutex (revert
    // TelemetryInput's store to a lock_guard) the writer's median ingest
    // under a hammering reader sits well above the threshold: measured
    // here before the change. The torn-read checks guard the seqlock's
    // own correctness (a mutex would pass them too).
    {
        TelemetryInput ti;
        TelemetryData motion; TelemetryInput::parsePacket("NULLCAT,1,1,1,1,1,1,1,1,1,1", 28, motion);
        TelemetryData ncx;    TelemetryInput::parsePacket("NULLCATX,1,1,1,1,1,1,1,1,1,1,1,1", 32, ncx);
        auto nowUs = [] { return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now().time_since_epoch()).count(); };
        auto median = [](std::vector<double> v) { std::sort(v.begin(), v.end()); return v.empty() ? 0.0 : v[v.size() / 2]; };
        auto maxOf  = [](const std::vector<double>& v) { double m = 0; for (double x : v) m = std::max(m, x); return m; };
        auto consistent = [](const TelemetryData& d)
        {
            if (!d.valid) return true;
            for (int i = 1; i < d.numPositions; ++i) if (d.positions[i] != d.positions[0]) return false;
            for (int i = 1; i < d.numNcx; ++i) if (d.ncx[i] != d.ncx[0]) return false;
            return true;
        };

        // 1. A status reader hammering getLatestData while the receive path stores.
        {
            std::atomic<bool> stop{false}; std::atomic<long> torn{0}, reads{0};
            std::thread reader([&] { while (!stop.load()) { TelemetryData d = ti.getLatestData(); ++reads; if (!consistent(d)) ++torn; } });
            std::vector<double> us; us.reserve(40000);
            for (int k = 1; k <= 40000; ++k)
            {
                for (double& p : motion.positions) p = k;
                for (int i = 0; i < ncx.numNcx; ++i) ncx.ncx[i] = k;
                const double t0 = nowUs();
                ti.ingest((k & 1) ? motion : ncx);
                us.push_back(nowUs() - t0);
            }
            stop.store(true); reader.join();
            std::printf("  B13 writer under a hammering reader: median %.2f us, max %.1f us over %zu stores (%ld reads, %ld torn)\n",
                        median(us), maxOf(us), us.size(), reads.load(), torn.load());
            check(median(us) < 10.0, "B13 the receive path's store never waits for a status reader (median under 10 us)");
            check(torn.load() == 0 && reads.load() > 0, "B13 the status reader never sees a torn snapshot");
        }
        // 2. The RT loop's own snapshot while another thread floods the store
        //    (the idle clock's drain during the loop-start hand-over). The
        //    flood is the real receive path's shape: a parse per packet
        //    (outside the write window) then the store; a loop of bare
        //    stores would be one long window, which no socket produces.
        {
            std::atomic<bool> stop{false};
            std::thread writer([&] {
                char line[160]; int k = 0;
                while (!stop.load())
                {
                    k = (k + 1) % 1000000;   // keeps the line under the buffer whatever the machine's speed
                    const int n = std::snprintf(line, sizeof line, "NULLCAT,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", k, k, k, k, k, k, k, k, k, k);
                    TelemetryData p; if (TelemetryInput::parsePacket(line, n, p)) ti.ingest(p);
                }
            });
            std::vector<double> us; us.reserve(40000); long torn = 0, fallback = 0;
            for (int k = 0; k < 40000; ++k)
            {
                const double t0 = nowUs();
                const TelemetryData d = ti.getLatestData();
                us.push_back(nowUs() - t0);
                if (!consistent(d)) ++torn;
                if (!d.valid) ++fallback;
            }
            stop.store(true); writer.join();
            std::printf("  B13 reader under a flooding writer: median %.2f us, max %.1f us (%ld torn, %ld fallbacks)\n",
                        median(us), maxOf(us), torn, fallback);
            check(median(us) < 10.0, "B13 the snapshot never waits for the store (median under 10 us)");
            check(torn == 0, "B13 ...and is never torn");
            // The bounded retry gives up only when the writer is preempted
            // inside its window (a loaded test machine does that now and
            // then: 0.05 to 2 % of reads measured, the count is printed
            // above); against a running writer the pre-spin sees the window
            // close. A reader that could not cope at all would give up on
            // most reads, so the bound here is loose on purpose.
            check(fallback < 4000, "B13 ...and the bounded retry gives up on well under 10 % of reads against a flooding writer");
        }
        // 3. A status reader stopped dead in the middle of its copy (the
        //    scheduler does that for a whole time slice on a busy box) must
        //    not stall the receive path. The reader thread is parked at a
        //    random point of its loop, almost all of which is the copy, and
        //    a store is timed while it sits there; a helper lets it go 20 ms
        //    later. With a mutex around the store (git show aa4daf3:
        //    src/TelemetryInput.cpp, the lock_guard in ingest and
        //    getLatestData) the store waited for the reader whenever it was
        //    caught inside its copy: 11 of 20 trials, maximum 36.8 ms, on
        //    the dev laptop. The seqlock's store takes under a microsecond
        //    whatever the reader is doing.
        {
            std::atomic<bool> stop{false}; std::atomic<long> sink{0};
            std::thread reader([&] { while (!stop.load()) { const TelemetryData d = ti.getLatestData(); sink.fetch_add(d.numPositions, std::memory_order_relaxed); } });
            double maxMs = 0.0; int parkedInsideCopy = 0;
            for (int trial = 0; trial < 20; ++trial)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                parkThread(reader);
                std::thread release([&] { std::this_thread::sleep_for(std::chrono::milliseconds(20)); unparkThread(reader); });
                const double t0 = nowUs();
                ti.ingest(motion);
                const double dtMs = (nowUs() - t0) / 1000.0;
                maxMs = std::max(maxMs, dtMs);
                if (dtMs > 5.0) ++parkedInsideCopy;
                release.join();
            }
            stop.store(true); reader.join();
            std::printf("  B13 store with the reader parked mid-copy: max %.3f ms over 20 trials (%d waited for it)\n", maxMs, parkedInsideCopy);
            check(maxMs < 5.0, "B13 a reader stopped dead mid-copy never stalls the receive path's store");
        }
    }

    // ---- one thread at a time on the socket (B14) ----
    // The RT loop drains the socket every cycle; the haptics idle clock
    // drains it while the loop is stopped. At loop start the loop's
    // running flag flips before its first drain, and the idle clock reads
    // the flag before its own drain of up to 32 datagrams, so for a moment
    // both were in receive(): two recvfrom calls into the one receive
    // buffer, two parsers over it, two writers of the store. receive()
    // now takes an owner flag: the second caller gets false and drains
    // next tick. Two threads drain a pre-filled loopback socket here; the
    // packet callback (inside receive) checks it is never entered twice
    // at once and that every frame it sees is whole. Reverting the
    // m_receiving exchange at the top of receive() fails the first two.
    {
        TelemetryInput ti;
        check(ti.initialize(0, "127.0.0.1"), "B14 a loopback socket on a port of the system's choosing");
        const int port = ti.port();
        check(port > 0, "B14 the bound port is reported");
        std::atomic<int> inside{0}, overlaps{0}, delivered{0}, torn{0};
        ti.setOnNewData([&](const TelemetryData& d)
        {
            if (inside.fetch_add(1) != 0) ++overlaps;
            for (int i = 1; i < d.numPositions; ++i) if (d.positions[i] != d.positions[0]) { ++torn; break; }
            ++delivered;
            const auto until = std::chrono::steady_clock::now() + std::chrono::microseconds(30);   // a parse's worth of work
            while (std::chrono::steady_clock::now() < until) {}
            inside.fetch_sub(1);
        });
        // Fewer than the smallest default receive buffer holds: each small
        // datagram costs a whole kernel buffer, so the Pi's default takes
        // about 220 of these and 300 overflowed it before either thread read.
        const int sent = 150;
        char line[64];
        for (int k = 1; k <= sent; ++k) { std::snprintf(line, sizeof line, "NULLCAT,%d,%d,%d,%d,%d,%d", k, k, k, k, k, k); sendUdp(port, line); }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        auto drain = [&]
        {
            const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (std::chrono::steady_clock::now() < until)
            {
                if (ti.receive()) continue;
                if (delivered.load() >= sent) break;
                std::this_thread::yield();
            }
        };
        std::thread a(drain), b(drain);
        a.join(); b.join();
        std::printf("  B14 two drainers: %d of %d delivered, %d overlaps, %d torn\n", delivered.load(), sent, overlaps.load(), torn.load());
        check(overlaps.load() == 0, "B14 two threads draining the socket never run the receive path at once");
        check(torn.load() == 0, "B14 ...and no frame is torn");
        check(delivered.load() >= sent * 95 / 100, "B14 ...and the datagrams arrive (the second caller backs off, it does not lose them)");
        ti.shutdown();
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
