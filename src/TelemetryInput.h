// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <mutex>
#include <functional>
#include <type_traits>
#include <atomic>
#include <cstdint>
#include <chrono>
#include <string>
#include "NcxTokens.h"
#include "TelemetryRecorder.h"

static constexpr int MAX_DRIVES = 10;

enum class TelemetryPacketType
{
    Motion,   // NULLCAT,<axis1>,...  - motion data (drives the axes)
    Ncx,      // NULLCATX,<ch0>,...   - raw sim telemetry channels (device
              //   state effects; meaning is assigned by the ncxBindings
              //   config, never by the wire)
    Ncy,      // NULLCATY,<key>=<value>,... - named channels (token names
              //   as keys, canonical units, no bindings) and the sim
              //   identity strings (game, car)
    Invalid
};

static constexpr int MAX_NCX_CHANNELS = 96;   // protocol 1.7: every token has a slot, with room to grow
static_assert(NcxTok::TokenCount <= MAX_NCX_CHANNELS, "every channel token needs a slot on the numbered line");
static constexpr int NCY_STR_LEN      = 48;   // game / car names, NUL-terminated
// Frozen-stream guard (OFF unless host.json sets ncxFrozenMs): packets
// still arriving but no numeric value has changed for this long = a
// paused sim or a sender replaying its last frame; the stream is treated
// as not fresh (effects release) until a value changes again. Off by
// default because an idling car at a standstill is exactly as still as a
// paused one (integer idle rpm, zero speed, zero pedals), and the SimHub
// plugin already goes silent on pause. NCX_FROZEN_MS is the window a
// sender without a pause fail-safe would use.
static constexpr int64_t NCX_FROZEN_MS = 2000;

struct TelemetryData
{
    uint64_t        timestampMs           = 0;
    int             numPositions          = 0;
    double          positions[MAX_DRIVES] = {};
    bool            valid                 = false;
    TelemetryPacketType packetType           = TelemetryPacketType::Invalid;
    double          nominalFrameSec       = -1.0;  // host new-frame interval (1/new_hz); -1 = not yet measured
    // NULLCATX channels, merged in by getLatestData() from the latest Ncx
    // packet (the two streams run at independent rates; an Ncx packet never
    // touches the motion fields and vice versa). ncxFresh is the fail-safe:
    // consumers go inert when the stream stops (<500 ms window).
    // Slots are POSITIONAL: field k on the line is slot k, always. A field
    // that is empty or not a number keeps its slot (ncxHave[k] false, value
    // 0) so every later slot still means what the sender's order says;
    // numNcx counts up to the last slot that is present.
    int             numNcx                   = 0;
    double          ncx[MAX_NCX_CHANNELS]    = {};
    bool            ncxHave[MAX_NCX_CHANNELS] = {};
    bool            ncxFresh                 = false;
    bool            ncxFrozen                = false;   // arriving but unchanged for NCX_FROZEN_MS
    // NULLCATY named values: per token, present + value. A parsed Ncy
    // packet carries only the keys it named; getLatestData() merges them
    // into the persistent per-token store (a 1 Hz identity line and a fast
    // channel line coexist). Same freshness window as the X stream.
    bool            ncyHave[NcxTok::TokenCount] = {};
    double          ncy[NcxTok::TokenCount]     = {};
    char            game[NCY_STR_LEN]           = {};
    char            car[NCY_STR_LEN]            = {};    // the car as the sim shows it (SimHub CarModel)
    char            carId[NCY_STR_LEN]          = {};    // its stable id (SimHub CarId: the content folder for AC); "" from an older plugin
    // The sender's direct source, from its identity line (protocol 1.9):
    // 'y' on, 'n' off, '-' none for this game, 'o' only (on, and it cannot
    // switch: a game's own sender), 0 never said.
    char            direct                      = 0;
};
static_assert(std::is_trivially_copyable<TelemetryData>::value, "TelemetryData is copied with memcpy by the snapshot reader");

// ============================================================
// Threads.
//
//   receive path   receive() -> parsePacket -> ingest. One caller at a
//                  time: the RT loop while it runs, the haptics idle
//                  clock while it does not. Both call receive(); the
//                  first one in owns the socket until it returns and the
//                  other gets false and drains next tick, so the two can
//                  never read into the one receive buffer at once during
//                  the hand-over at loop start. This is the store's only
//                  writer. Never blocks, never allocates, never logs
//                  through anything that can block.
//   snapshot       getLatestData(). The receive path's own thread every
//                  cycle, plus the web and status threads whenever they
//                  like. A seqlock: the writer bumps a sequence number
//                  around its store (odd = writing) and waits for nobody;
//                  a reader copies, then checks the number did not move
//                  and retries if it did. A reader on the RT side only
//                  ever meets a write in progress during the hand-over,
//                  and gives up after a bounded number of tries (the
//                  frame is reported not valid: one cycle's hold).
//   control        initialize()/shutdown()/setFrozenMs()/setOnNewData()
//                  from the main thread before the loops start; the
//                  recorder's start/stop from the web thread (its own
//                  lock-free hand-off, see TelemetryRecorder.h).
// ============================================================

class TelemetryInput
{
public:
    TelemetryInput();
    ~TelemetryInput();

    // bindAddr semantics: "" or "0.0.0.0" = platform
    // default (Linux: any interface; Windows: loopback - the telemetry source,
    // e.g. SimHub, usually runs locally). An explicit address binds that
    // interface only - EXCEPT "127.0.0.1"/"localhost" on Linux, auto-corrected
    // to any-interface with a warning: loopback telemetry into the Pi has no
    // use case, and existing host.json files carry 127.0.0.1 as a stale UI
    // placeholder (honoring it literally would silently kill PC→Pi telemetry
    // on update). A headless Windows NUC sets its LAN IP (or 0.0.0.0) so the
    // separate game PC's telemetry can reach it (plus a firewall inbound-UDP
    // allow).
    bool initialize(int port, const std::string& bindAddr = "");
    bool receive();
    // A whole copy of the latest frame. The RT loop and the idle clock
    // (wait false) never wait: a writer preempted inside its store gives
    // them an empty frame (not valid, no channels) for one cycle. A status
    // or web thread passes wait true and retries until it has a whole one,
    // so a collision never reads as the stream stopping (which would clear
    // the car identity there and rerun the profile and car table logic).
    TelemetryData getLatestData(bool wait = false) const;
    bool hasData()        const { return m_hasData.load(); }
    bool hasRecentData()  const
    {
        if (!m_hasData.load()) return false;
        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        return (now - m_lastMotionPacketMs.load()) < 1500;
    }
    // Prompt telemetry-loss check with a caller-chosen window. The control loop
    // uses this to flip TelemetryData.valid=false soon after the stream stops --
    // getLatestData() otherwise keeps returning the last packet (valid=true)
    // forever, so without it the staleness/standby logic never fires. The 1500 ms
    // hasRecentData() is deliberately separate (it drives the UI RX indicator).
    bool hasFreshData(int withinMs) const
    {
        if (!m_hasData.load()) return false;
        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        return (now - m_lastMotionPacketMs.load()) < withinMs;
    }
    bool isInitialized()  const { return m_socket != INVALID_SOCKET_VALUE; }
    // The port the socket is bound to (what initialize() was given, or the
    // system's choice when it was given 0).
    int  port()           const { return m_port; }
    // NULLCATX stream indicator (web status): a channel packet arrived recently.
    // Same 1500 ms window as the motion RX indicator; the 500 ms EFFECT
    // freshness in getLatestData() is deliberately tighter (fail-safe first,
    // indicator second).
    bool hasRecentNcx() const
    {
        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const int64_t last = m_lastNcxPacketMs.load();
        return last != 0 && (now - last) < 1500;
    }
    void shutdown();

    // Direct telemetry (protocol 1.9). What the channel sender last said
    // about its direct source ('y', 'n', '-', 'o', or 0 when it never said; it
    // is cleared when the stream comes back after a gap, so read it with
    // hasRecentNcx()), and a line back to it: NULLCATC,... sent from
    // this socket (the telemetry port, its answer path through a firewall)
    // to the address its NULLCATY lines come from. Any thread but the RT
    // loop; false when no sender is known or the socket is closed.
    char channelSenderDirect() const { return m_senderDirect.load(std::memory_order_acquire); }
    bool sendToChannelSender(const char* line) const;

    // More than one channel sender (protocol 1.9 `direct`): a sender whose
    // identity line says direct=on (or only) is the channel source while
    // it keeps sending, and every other sender's NULLCATX and NULLCATY
    // lines are set aside (motion lines never are), so two senders for one
    // game (a game's own nullCAT mod and the SimHub plugin) never mix their
    // values or flip the car between their names. Its own lines keep it the
    // source after any gap; it lets go when it says off or none, when
    // another sender's line comes kDirectHoldMs after its last (the game
    // closed, the mod switched off), or sooner, kDirectQuietMs or more
    // after its last, when a set-aside line names another game than the
    // direct sender's own (names compared without case or punctuation:
    // BeamNgDrive is BeamNG.drive), from any sender: another game started
    // (2026-10-30: Le Mans Ultimate's lines after Assetto Corsa Rally's came
    // from a new socket and waited 38 s for the hold to run out). Until the
    // direct sender has named its game, the first sender to name one in a
    // set-aside line naming another than it did counts instead. The others
    // are used again. A game paused in a menu sends nothing for as long as
    // it sits there (BeamNG 2026-10-29: 42 s, and twice about 2 minutes)
    // while the SimHub plugin goes on sending its own read of the same
    // game, which took over at 5 s and flipped the car. A sender whose
    // hold ran out takes the source back with its first line (before its
    // identity names its source again). Another sender saying direct=on
    // takes over at once. Each change of source starts the store clean,
    // positional slots included. sender = where the line came from (the
    // m_ncySender encoding), nowMs the time it arrived. True: hand the
    // line on; false: set aside. Receive path only (receive() asks before
    // anything is stored or recorded; public for tests, with their own
    // senders and clock).
    static constexpr int64_t kDirectHoldMs  = 300000;
    static constexpr int64_t kDirectQuietMs = 5000;
    bool admitChannelLine(const TelemetryData& parsed, uint64_t sender, int64_t nowMs);
    // Any thread: another sender's channel line was set aside in the last
    // 1.5 s (the status says so beside direct telemetry).
    bool channelLinesSetAside() const
    {
        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const int64_t last = m_setAsideMs.load(std::memory_order_acquire);
        return last != 0 && (now - last) < 1500;
    }

    // UDP telemetry-rate diagnostic. Measured on the receive path, ACTIVE ONLY
    // WHEN DIAGNOSTICS ARE ENABLED (zero cost otherwise). Returns -1 when off or
    // before the first 1 s window completes. arrival = packets/s pulled from the
    // socket; new = frames/s carrying a changed value; holdPct = % that are holds
    // (the host resending the last value). new_hz ~ the host motion software's
    // effective new-frame rate at our socket (its interpolated output resampled to
    // the configured UDP rate) -- NOT the game's telemetry rate.
    double getUdpArrivalHz() const { return m_udpArrivalHz.load(std::memory_order_relaxed); }
    double getUdpNewHz()     const { return m_udpNewHz.load(std::memory_order_relaxed); }
    double getUdpHoldPct()   const { return m_udpHoldPct.load(std::memory_order_relaxed); }

    // Nominal new-frame interval (seconds) for the Interpolate mode, measured
    // always-on (independent of the diag flag). -1 until the EWMA converges
    // (>=8 new frames), so the caller degrades to Bypass during startup.
    double getNominalFrameSec() const
    {
        return (m_newFrameCount.load(std::memory_order_relaxed) >= 8)
            ? m_frameIntervalEwmaMs.load(std::memory_order_relaxed) / 1000.0
            : -1.0;
    }

    // Invoked from the receive path for every parsed packet. Register before
    // the receive() loop starts.
    void setOnNewData(std::function<void(const TelemetryData&)> cb)  { m_onNewData = std::move(cb); }
    // No lifecycle callbacks by design: sender lifecycle signals are
    // intentionally ignored (see receive()).

    // Parse one UDP line into a TelemetryData. Pure function of its inputs -     // public static so TestTelemetryParse can pin the wire-format semantics
    // (this runs on the RT thread at telemetry rate; no heap).
    static bool parsePacket(const char* buf, int len, TelemetryData& out);
    // Store one parsed packet exactly as receive() would (no socket needed):
    // motion packets into the motion snapshot, channel packets into the
    // channel store with the frozen-stream bookkeeping. Public for tests;
    // it is the store's writer, so one thread at a time (receive() holds
    // the owner flag around it).
    bool ingest(const TelemetryData& parsed);

    // Frozen-stream guard window in ms; 0 (the default) = off. Set once at
    // startup from host.json (ncxFrozenMs).
    void    setFrozenMs(int64_t ms) { m_frozenMs.store(ms < 0 ? 0 : ms); }
    int64_t frozenMs() const        { return m_frozenMs.load(); }

    // Every datagram as it arrived, to a file (start/stop from the web
    // thread; receive() only hands it a copy, see TelemetryRecorder.h).
    TelemetryRecorder&       recorder()       { return m_recorder; }
    const TelemetryRecorder& recorder() const { return m_recorder; }

private:
    TelemetryRecorder m_recorder;
    static constexpr uintptr_t INVALID_SOCKET_VALUE = static_cast<uintptr_t>(~0);
    uintptr_t m_socket            = INVALID_SOCKET_VALUE;
    bool      m_winsockInitialized= false;
    int       m_port              = 4444;

    // The store: written by the receive path only (inside the seqlock
    // window), copied whole by getLatestData().
    struct Store
    {
        TelemetryData motion;                       // the last motion packet as parsed
        // Latest NULLCATX channels, deliberately separate from the motion
        // frame so neither stream clobbers the other.
        int     ncxCount = 0;
        double  ncxVals[MAX_NCX_CHANNELS] = {};
        bool    ncxHave[MAX_NCX_CHANNELS] = {};
        // Persistent NULLCATY store: each Y packet merges the keys it
        // names; cleared when the channel stream has gone stale (so a
        // returning sender never resurrects old values) and when a line
        // names a different game, car or car id (so one game's tokens
        // never play under the next).
        bool    ncyHave[NcxTok::TokenCount] = {};
        double  ncyVals[NcxTok::TokenCount] = {};
        char    game[NCY_STR_LEN]  = {};
        char    car[NCY_STR_LEN]   = {};
        char    carId[NCY_STR_LEN] = {};
        char    direct = 0;
    };
    static_assert(std::is_trivially_copyable<Store>::value, "the snapshot reader copies the store with memcpy");
    Store m_store;
    std::atomic<uint32_t> m_seq{0};              // odd while the receive path writes m_store
    static constexpr int kSnapshotTries = 16;    // a reader's bounded retry (each try is one copy)
    void storeBegin();                           // receive path: open the write window
    void storeEnd();                             //               close it
    bool snapshot(Store& out) const;             // any thread: a whole copy, false if the retry ran out
    std::atomic<bool>  m_hasData{false};
    // The receive path's owner flag: one thread at a time in receive().
    std::atomic<bool>  m_receiving{false};

    static constexpr int RECV_BUF_SIZE = 4096;
    char m_recvBuf[RECV_BUF_SIZE] = {};

    // Receive-thread only. Counts consecutive parse failures for the
    // rate-limited reject log (fires at 1, 101, ...); reset on a clean parse.
    int m_parseFailCount = 0;

    std::atomic<int64_t> m_lastMotionPacketMs{0};

    std::atomic<int64_t> m_lastNcxPacketMs{0};
    // The channel sender: bit 48 set when known, its IPv4 address (network
    // order) above its port (network order). Written by the receive path.
    std::atomic<uint64_t> m_ncySender{0};
    std::atomic<char>     m_senderDirect{0};
    // admitChannelLine: the direct sender (0 = none), its last channel
    // line, the game it names (empty until it does), the game the
    // set-aside lines first named under its hold and the sender that named
    // it, the sender whose hold last ran out, and a change of source for
    // the next ingest to start the store clean on (receive path only);
    // when a line was last set aside.
    uint64_t              m_directSender   = 0;
    int64_t               m_directLastMs   = 0;
    char                  m_directGame[NCY_STR_LEN] = {};
    char                  m_asideGame[NCY_STR_LEN] = {};
    uint64_t              m_asideSender    = 0;
    uint64_t              m_lastHolder     = 0;
    bool                  m_setAsideLogged = false;
    bool                  m_sourceChanged  = false;
    std::atomic<int64_t>  m_setAsideMs{0};
    std::atomic<int64_t> m_lastNcxChangeMs{0};   // last packet whose numbers differed
    std::atomic<int64_t> m_frozenMs{0};          // guard window, 0 = off
    mutable std::atomic<int> m_ncxStreamState{0}; // 0 stopped, 1 live, 2 frozen (getLatestData, any thread)

    // UDP-rate diagnostic: receive-thread-only counters + published atomics.
    void    updateUdpRate(const TelemetryData& d);
    double  m_udpPrevPos[MAX_DRIVES] = {};
    int     m_udpPrevN       = 0;
    bool    m_udpHavePrev    = false;
    int     m_udpArrival     = 0;
    int     m_udpNew         = 0;
    int     m_udpHold        = 0;
    int64_t m_udpWindowStartMs = 0;
    std::atomic<double> m_udpArrivalHz{-1.0};
    std::atomic<double> m_udpNewHz{-1.0};
    std::atomic<double> m_udpHoldPct{-1.0};
    // Always-on nominal frame-interval estimate (for the Interpolate mode).
    std::atomic<double> m_frameIntervalEwmaMs{0.0};
    std::atomic<int>    m_newFrameCount{0};
    int64_t             m_lastNewFrameMs = 0;   // receive-thread only

    std::function<void(const TelemetryData&)> m_onNewData;
};
