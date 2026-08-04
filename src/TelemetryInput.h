// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <mutex>
#include <functional>
#include <atomic>
#include <cstdint>
#include <chrono>
#include <string>

static constexpr int MAX_DRIVES = 10;

enum class TelemetryPacketType
{
    Motion,   // NULLCAT,<axis1>,...  — motion data (the only accepted packet)
    Invalid
};

struct TelemetryData
{
    uint64_t        timestampMs           = 0;
    int             numPositions          = 0;
    double          positions[MAX_DRIVES] = {};
    bool            valid                 = false;
    TelemetryPacketType packetType           = TelemetryPacketType::Invalid;
    double          nominalFrameSec       = -1.0;  // host new-frame interval (1/new_hz); -1 = not yet measured
};

class TelemetryInput
{
public:
    TelemetryInput();
    ~TelemetryInput();

    // bindAddr semantics: "" or "0.0.0.0" = platform
    // default (Linux: any interface; Windows: loopback — the telemetry source,
    // e.g. SimHub, usually runs locally). An explicit address binds that
    // interface only — EXCEPT "127.0.0.1"/"localhost" on Linux, auto-corrected
    // to any-interface with a warning: loopback telemetry into the Pi has no
    // use case, and existing host.json files carry 127.0.0.1 as a stale UI
    // placeholder (honoring it literally would silently kill PC→Pi telemetry
    // on update). A headless Windows NUC sets its LAN IP (or 0.0.0.0) so the
    // separate game PC's telemetry can reach it (plus a firewall inbound-UDP
    // allow).
    bool initialize(int port, const std::string& bindAddr = "");
    bool receive();
    TelemetryData getLatestData() const;
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
    void shutdown();

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

    // Parse one UDP line into a TelemetryData. Pure function of its inputs —
    // public static so TestTelemetryParse can pin the wire-format semantics
    // (this runs on the RT thread at telemetry rate; no heap).
    static bool parsePacket(const char* buf, int len, TelemetryData& out);

private:
    static constexpr uintptr_t INVALID_SOCKET_VALUE = static_cast<uintptr_t>(~0);
    uintptr_t m_socket            = INVALID_SOCKET_VALUE;
    bool      m_winsockInitialized= false;
    int       m_port              = 4444;

    mutable std::mutex m_dataMutex;
    TelemetryData         m_latestData;
    std::atomic<bool>  m_hasData{false};

    static constexpr int RECV_BUF_SIZE = 4096;
    char m_recvBuf[RECV_BUF_SIZE] = {};

    std::atomic<int64_t> m_lastMotionPacketMs{0};

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
