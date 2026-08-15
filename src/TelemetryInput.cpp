// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TelemetryInput.cpp
// UDP telemetry reception (SimHub, SimTools, FlyPT Mover - any custom UDP sender).
//
// Cross-platform: Winsock on Windows, BSD sockets on
// Linux. Only the socket I/O (initialize/receive/shutdown) is
// platform-guarded; the CSV parser is plain std C++.
//
// Windows: link ws2_32. Linux: no extra link (libc sockets).
// ============================================================

#include "TelemetryInput.h"
#include "Logging.h"

#ifdef _WIN32
  // Include Winsock BEFORE any other Windows headers
  #define WIN32_LEAN_AND_MEAN
  #define NOMINMAX
  #include <winsock2.h>
  #include <ws2tcpip.h>
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  #include <fcntl.h>
  #include <sys/time.h>
  #include <cerrno>
  // Winsock-style aliases so the shared call sites stay readable.
  typedef int SOCKET;
  static const int INVALID_SOCKET = -1;
  static const int SOCKET_ERROR   = -1;
  static inline int closesocket(int s) { return ::close(s); }
  static inline int WSAGetLastError()  { return errno; }
#endif

#include <cstring>
#include <cstdlib>
#include <cctype>
#include <cstdint>
#include <string>
#include <algorithm>

// ============================================================
// Constructor / Destructor
// ============================================================

TelemetryInput::TelemetryInput()
{
    memset(m_recvBuf, 0, sizeof(m_recvBuf));
}

TelemetryInput::~TelemetryInput()
{
    shutdown();
}

// ============================================================
// Initialize socket and bind
// ============================================================

bool TelemetryInput::initialize(int port, const std::string& bindAddr)
{
    m_port = port;

#ifdef _WIN32
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != 0)
    {
        LOG_ERROR(strf("TelemetryInput: WSAStartup failed: %d", result));
        return false;
    }
    m_winsockInitialized = true;
#endif

    // Create UDP socket
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET)
    {
        LOG_ERROR(strf("TelemetryInput: socket() failed: %d", WSAGetLastError()));
#ifdef _WIN32
        WSACleanup();
        m_winsockInitialized = false;
#endif
        return false;
    }

    // Enable SO_REUSEADDR so multiple instances can bind (optional)
    int optval = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&optval), sizeof(optval));

    // Set non-blocking mode + a short receive timeout (10 ms safety).
#ifdef _WIN32
    u_long nonblocking = 1;
    ioctlsocket(sock, FIONBIO, &nonblocking);

    DWORD timeout = 10;  // ms
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
               reinterpret_cast<const char*>(&timeout), sizeof(timeout));
#else
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    struct timeval timeout{};
    timeout.tv_sec  = 0;
    timeout.tv_usec = 10 * 1000;  // 10 ms
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
               reinterpret_cast<const char*>(&timeout), sizeof(timeout));
#endif

    // Bind address (telemetryBindAddr): "" / "0.0.0.0" = platform default -     // Windows loopback (telemetry source usually local), Linux any-interface (the PC
    // sends over the LAN/point-to-point link). An explicit address restricts
    // reception to that interface (e.g. pin the Pi to the wired PC link, or
    // open a headless Windows NUC to a separate game PC.s telemetry).
    // COMPAT: "127.0.0.1"/"localhost" on Linux is a stale UI placeholder in
    // existing host.json files, and loopback telemetry into the Pi has no use
    // case - auto-correct to any-interface with a warning instead of silently
    // killing PC→Pi telemetry on update.
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(port));

    std::string effective = bindAddr;
#ifndef _WIN32
    if (effective == "127.0.0.1" || effective == "localhost")
    {
        LOG_WARNING("TelemetryInput: telemetryBindAddr=127.0.0.1 is loopback-only and has no use "
                    "on Linux (stale placeholder?) -- binding all interfaces instead. "
                    "Set a specific LAN IP to restrict, or clear the field for auto.");
        effective.clear();
    }
#endif
    if (effective.empty() || effective == "0.0.0.0")
    {
#ifdef _WIN32
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   // 127.0.0.1 (telemetry source local)
#else
        addr.sin_addr.s_addr = htonl(INADDR_ANY);        // 0.0.0.0
#endif
        if (effective == "0.0.0.0")
            addr.sin_addr.s_addr = htonl(INADDR_ANY);    // explicit any (NUC opens up)
    }
    else if (inet_pton(AF_INET, effective.c_str(), &addr.sin_addr) != 1)
    {
        LOG_WARNING(strf("TelemetryInput: telemetryBindAddr '%s' is not a valid IPv4 address -- "
                         "falling back to platform default.", effective.c_str()));
#ifdef _WIN32
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
#else
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
#endif
    }

    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR)
    {
        LOG_ERROR(strf("TelemetryInput: bind() failed on port %d: error %d",
            port, WSAGetLastError()));
        closesocket(sock);
#ifdef _WIN32
        WSACleanup();
        m_winsockInitialized = false;
#endif
        return false;
    }

    m_socket = static_cast<uintptr_t>(sock);

    // Log the address ACTUALLY bound (not a hardcoded guess) so a NUC/remote
    // setup can be verified from the log alone.
    {
        char boundStr[INET_ADDRSTRLEN] = "?";
        inet_ntop(AF_INET, &addr.sin_addr, boundStr, sizeof(boundStr));
        LOG_INFO(strf("TelemetryInput: Listening for telemetry on UDP %s:%d", boundStr, port));
    }
    return true;
}

// ============================================================
// Receive (non-blocking)
// ============================================================

bool TelemetryInput::receive()
{
    if (m_socket == INVALID_SOCKET_VALUE) return false;

    SOCKET sock = static_cast<SOCKET>(m_socket);

    sockaddr_in senderAddr{};
#ifdef _WIN32
    int senderAddrLen = sizeof(senderAddr);
#else
    socklen_t senderAddrLen = sizeof(senderAddr);
#endif

    int bytes = static_cast<int>(recvfrom(
        sock,
        m_recvBuf,
        RECV_BUF_SIZE - 1,   // Leave room for null terminator
        0,
        reinterpret_cast<sockaddr*>(&senderAddr),
        &senderAddrLen
    ));

    if (bytes == SOCKET_ERROR)
    {
        int err = WSAGetLastError();
#ifdef _WIN32
        if (err == WSAEWOULDBLOCK)
            return false;  // No data available - normal for non-blocking
#else
        if (err == EWOULDBLOCK || err == EAGAIN)
            return false;  // No data available - normal for non-blocking
#endif
        LOG_WARNING(strf("TelemetryInput: recvfrom() error: %d", err));
        return false;
    }

    if (bytes <= 0) return false;

    // Null-terminate the received data
    m_recvBuf[bytes] = '\0';

    TelemetryData parsed;
    if (!parsePacket(m_recvBuf, bytes, parsed))
    {
        // Limit logged length to 80 chars
        int logLen = std::min(bytes, 80);
        char tmp[81];
        memcpy(tmp, m_recvBuf, logLen);
        tmp[logLen] = '\0';
        LOG_DEBUG(strf("TelemetryInput: Failed to parse packet: '%s'", tmp));
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_dataMutex);
        m_latestData = parsed;
    }

    // Only timestamp motion packets for the receiving indicator
    if (parsed.packetType == TelemetryPacketType::Motion)
    {
        auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        m_lastMotionPacketMs.store(static_cast<int64_t>(nowMs));
        updateUdpRate(parsed);   // telemetry-rate diagnostic (diag-gated)
    }

    // Only Motion packets set m_hasData: nullCAT gates motion on its own
    // readiness (all axes homed + at center) + the web/GPIO Stop, never on
    // any sender-side lifecycle signal.
    m_hasData.store(parsed.packetType == TelemetryPacketType::Motion);

    if (m_onNewData)
        m_onNewData(parsed);

    return true;
}

// ============================================================
// updateUdpRate - UDP telemetry-rate diagnostic (receive thread only).
// Active only when diagnostics are enabled (otherwise one atomic-bool check and
// out). Per Motion packet: count arrival, and classify NEW vs HOLD by comparing
// the raw values to the previous packet. A held frame is BIT-IDENTICAL (the sender
// resends the same string), so the epsilon only needs to reject float-repr noise
// and is kept far below 1 LSB -- it must never reclassify slow real motion as a
// hold. Rolls up once per second into the published atomics + a gated DIAG line.
// ============================================================
void TelemetryInput::updateUdpRate(const TelemetryData& d)
{
    const int64_t nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    // ---- ALWAYS-ON: new-vs-hold detection + nominal frame-interval EWMA. Feeds the
    //      Interpolate conditioning mode, so it must run regardless of the diag flag.
    //      Cheap: a few compares + one EWMA step, no allocation. ----
    const double EPS = 1e-6;
    bool changed = !m_udpHavePrev || d.numPositions != m_udpPrevN;
    for (int i = 0; !changed && i < d.numPositions && i < MAX_DRIVES; ++i)
    {
        double diff = d.positions[i] - m_udpPrevPos[i];
        if (diff < 0) diff = -diff;
        if (diff > EPS) changed = true;
    }
    for (int i = 0; i < d.numPositions && i < MAX_DRIVES; ++i)
        m_udpPrevPos[i] = d.positions[i];
    m_udpPrevN = d.numPositions;
    m_udpHavePrev = true;

    if (changed)
    {
        if (m_lastNewFrameMs != 0)
        {
            const double interval = static_cast<double>(nowMs - m_lastNewFrameMs);
            if (interval > 0.0 && interval < 1000.0)   // ignore the first sample / dropouts > 1 s
            {
                double ewma   = m_frameIntervalEwmaMs.load(std::memory_order_relaxed);
                const int cnt = m_newFrameCount.load(std::memory_order_relaxed);
                ewma = (cnt == 0) ? interval : ewma + 0.1 * (interval - ewma);
                m_frameIntervalEwmaMs.store(ewma, std::memory_order_relaxed);
                m_newFrameCount.store(cnt + 1, std::memory_order_relaxed);
            }
        }
        m_lastNewFrameMs = nowMs;
    }

    // ---- DIAG-GATED: arrival/new/hold rates + the DIAG line ----
    if (!Logger::instance().isDiagEnabled())
    {
        if (m_udpArrivalHz.load(std::memory_order_relaxed) >= 0.0)
        {
            m_udpArrivalHz.store(-1.0, std::memory_order_relaxed);
            m_udpNewHz.store(-1.0, std::memory_order_relaxed);
            m_udpHoldPct.store(-1.0, std::memory_order_relaxed);
            m_udpArrival = m_udpNew = m_udpHold = 0;
            m_udpWindowStartMs = 0;
        }
        return;
    }
    if (m_udpWindowStartMs == 0) m_udpWindowStartMs = nowMs;
    ++m_udpArrival;
    if (changed) ++m_udpNew; else ++m_udpHold;

    const int64_t elapsed = nowMs - m_udpWindowStartMs;
    if (elapsed >= 1000)
    {
        const double secs = elapsed / 1000.0;
        const double arrHz = m_udpArrival / secs;
        const double newHz = m_udpNew / secs;
        const double holdPct = m_udpArrival ? (100.0 * m_udpHold / m_udpArrival) : 0.0;
        m_udpArrivalHz.store(arrHz, std::memory_order_relaxed);
        m_udpNewHz.store(newHz, std::memory_order_relaxed);
        m_udpHoldPct.store(holdPct, std::memory_order_relaxed);
        const double interpLatMs = (m_newFrameCount.load(std::memory_order_relaxed) >= 8)
                                 ? m_frameIntervalEwmaMs.load(std::memory_order_relaxed) : -1.0;
        RT_DIAG("DIAG | udp | arrival_hz=%.0f | new_hz=%.0f | hold_pct=%.0f | interp_lat_ms=%.1f",
            arrHz, newHz, holdPct, interpLatMs);
        m_udpArrival = m_udpNew = m_udpHold = 0;
        m_udpWindowStartMs = nowMs;
    }
}

// ============================================================
// Get Latest Data (thread-safe copy)
// ============================================================

TelemetryData TelemetryInput::getLatestData() const
{
    std::lock_guard<std::mutex> lock(m_dataMutex);
    return m_latestData;
}

// ============================================================
// Shutdown
// ============================================================

void TelemetryInput::shutdown()
{
    if (m_socket != INVALID_SOCKET_VALUE)
    {
        closesocket(static_cast<SOCKET>(m_socket));
        m_socket = INVALID_SOCKET_VALUE;
        LOG_INFO("TelemetryInput: Socket closed.");
    }

#ifdef _WIN32
    if (m_winsockInitialized)
    {
        WSACleanup();
        m_winsockInitialized = false;
    }
#else
    (void)m_winsockInitialized;
#endif
}

// ============================================================
// Parse Packet  (zero-allocation span tokenizer)
//
// Wire format (CSV line from any motion software's custom UDP output,
// e.g. SimHub's Generic UDP output):
//   NULLCAT,<pos0>,<pos1>,...,<posN>\n   -- every field after the header is
//                                          an axis value. There is NO
//                                          timestamp field on the wire
//                                          (timestampMs is always 0).
//
// This runs ON THE RT THREAD at telemetry rate, so: no heap, no
// std::string. Pointer/length spans throughout; each numeric field is
// copied into a fixed stack buffer only because strtod needs a NUL
// terminator. Fields longer than kMaxFieldLen-1 chars are treated as
// garbage and skipped -- no legitimate numeric approaches that length
// (TestTelemetryParse pins a 40-char field as parseable).
//
// Semantics are pinned byte-for-byte by TestTelemetryParse.
// ============================================================

bool TelemetryInput::parsePacket(const char* buf, int len, TelemetryData& out)
{
    // Trim the whole line (span bounds only, no copy).
    const char* p = buf;
    while (len > 0 && std::isspace(static_cast<unsigned char>(p[0])))       { ++p; --len; }
    while (len > 0 && std::isspace(static_cast<unsigned char>(p[len - 1]))) { --len; }

    // ---- Motion data ----
    out.numPositions = 0;
    out.packetType   = TelemetryPacketType::Motion;

    // Header token: everything before the first comma must read "NULLCAT"
    // case-insensitively with embedded whitespace ignored (tolerance pinned
    // by TestTelemetryParse). No comma at all = not our packet.
    const char* comma = static_cast<const char*>(std::memchr(p, ',', static_cast<size_t>(len)));
    if (!comma) return false;
    {
        const char* ref = "NULLCAT";
        for (const char* h = p; h < comma; ++h)
        {
            const unsigned char c = static_cast<unsigned char>(*h);
            if (std::isspace(c)) continue;
            if (*ref == '\0' ||
                std::toupper(c) != static_cast<unsigned char>(*ref)) return false;
            ++ref;
        }
        if (*ref != '\0') return false;
    }

    // Fields: comma-separated numerics; empty/garbage fields are skipped and
    // the position array COMPACTS (pinned behavior).
    constexpr int kMaxFieldLen = 64;
    const char* end = p + len;
    const char* f   = comma + 1;
    while (f <= end && out.numPositions < MAX_DRIVES)
    {
        const char* fc = static_cast<const char*>(
            std::memchr(f, ',', static_cast<size_t>(end - f)));
        const char* b  = fc ? fc : end;
        const char* a  = f;
        while (a < b && std::isspace(static_cast<unsigned char>(a[0])))  ++a;
        while (b > a && std::isspace(static_cast<unsigned char>(b[-1]))) --b;

        const int flen = static_cast<int>(b - a);
        if (flen > 0 && flen < kMaxFieldLen)
        {
            char fld[kMaxFieldLen];   // stack only; NUL terminator for strtod
            std::memcpy(fld, a, static_cast<size_t>(flen));
            fld[flen] = '\0';
            char* endp = nullptr;
            const double val = std::strtod(fld, &endp);
            if (endp != fld)
                out.positions[out.numPositions++] = val;
        }

        if (!fc) break;
        f = fc + 1;
    }

    out.timestampMs = 0;
    out.valid = (out.numPositions > 0);
    return out.valid;
}
