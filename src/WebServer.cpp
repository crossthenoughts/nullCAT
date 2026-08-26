// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// WebServer.cpp - embedded HTTP + WebSocket control/status
// server. Endpoint contracts and threading notes in WebServer.h.
// ============================================================

// httplib uses platform sockets - suppress Windows warnings
#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  define NOMINMAX
#  define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif

#include "WebServer.h"
#include "httplib.h"
#include "Logging.h"
#include "StatusModel.h"   // shared canonical status (additive emit)
#include "A6FaultCodes.h"  // decoded fault names on the drive cards
#include "CommissioningMode.h"  // test-mode plan builders (/api/test/*)
#include "AxisKind.h"           // axis classification authority
#include <QJsonDocument>   // /api/test/start body parsing
#include <QJsonObject>
#include <QJsonArray>
#include "SdoWorker.h"     // IGBT temp readout for the drive cards (OP-time round-robin poll)
#include <vector>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <chrono>
#include <sys/stat.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <algorithm>

// Local interface enumeration for the Host-header allowlist.
#ifdef _WIN32
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  include <iphlpapi.h>
#  pragma comment(lib, "iphlpapi.lib")
#else
#  include <ifaddrs.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#endif

// ---- JSON helpers (no external dependency) ----

static std::string jsonStr(const std::string& s)
{
    // Minimal JSON string escaping
    std::string out;
    out.reserve(s.size() + 2);
    out += '"';
    for (char c : s)
    {
        if      (c == '"')  out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else if (c == '\n') out += "\\n";
        else if (c == '\r') out += "\\r";
        else                out += c;
    }
    out += '"';
    return out;
}

static std::string jsonBool(bool v)  { return v ? "true" : "false"; }
static std::string jsonInt(int v)    { return std::to_string(v); }
static std::string jsonDouble(double v, int prec = 2)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "%.*f", prec, v);
    return buf;
}

// Sibling file in the same directory as the config.json anchor (host.json / rig.json).
static std::string siblingFile(const std::string& anchor, const char* name)
{
    auto pos = anchor.find_last_of("/\\");
    std::string dir = (pos == std::string::npos) ? std::string() : anchor.substr(0, pos + 1);
    return dir + name;
}

// Who owns host.json on THIS build. Keyed off HAS_QT_CONFIG (a native config UI
// is compiled in), NOT the OS - so a future headless x86/Windows NUC build,
// which omits the Qt config UI, correctly reports "web" and lets the browser
// edit host fields. PC (Qt) = "native"; headless (Pi, or headless NUC) = "web".
static const char* hostOwner()
{
#ifdef HAS_QT_CONFIG
    return "native";
#else
    return "web";
#endif
}

// ============================================================

// ============================================================
// Host-header allowlist (DNS-rebinding defense; see WebServer.h)
// ============================================================

std::string WebServer::hostHeaderName(const std::string& hostHeader)
{
    std::string h = hostHeader;
    // Trim surrounding whitespace.
    while (!h.empty() && (h.front() == ' ' || h.front() == '\t')) h.erase(0, 1);
    while (!h.empty() && (h.back()  == ' ' || h.back()  == '\t')) h.pop_back();
    if (h.empty()) return h;

    if (h.front() == '[')
    {
        // Bracketed IPv6 literal: "[::1]" or "[::1]:8080".
        const size_t close = h.find(']');
        h = (close == std::string::npos) ? std::string() : h.substr(1, close - 1);
    }
    else
    {
        // Strip ":port" - but a bare (non-bracketed) IPv6 literal has multiple
        // colons and carries no port; leave it whole.
        const size_t first = h.find(':');
        if (first != std::string::npos && h.find(':', first + 1) == std::string::npos)
            h.erase(first);
    }
    std::transform(h.begin(), h.end(), h.begin(),
                   [](unsigned char c) { return (char)std::tolower(c); });
    return h;
}

bool WebServer::isPrivateClientAddr(const std::string& addr)
{
    in6_addr a6{};
    if (inet_pton(AF_INET6, addr.c_str(), &a6) == 1)
    {
        if (IN6_IS_ADDR_LOOPBACK(&a6) || IN6_IS_ADDR_LINKLOCAL(&a6)) return true;
        const unsigned char* b = (const unsigned char*)&a6;
        if ((b[0] & 0xFE) == 0xFC) return true;                    // ULA fc00::/7
        if (IN6_IS_ADDR_V4MAPPED(&a6))                             // ::ffff:a.b.c.d
            return isPrivateClientAddr(std::to_string(b[12]) + "." + std::to_string(b[13])
                                       + "." + std::to_string(b[14]) + "." + std::to_string(b[15]));
        return false;
    }
    in_addr a4{};
    if (inet_pton(AF_INET, addr.c_str(), &a4) != 1) return false;  // unparseable: fail closed
    const uint32_t ip = ntohl(a4.s_addr);
    return (ip >> 24) == 127                                       // loopback
        || (ip >> 24) == 10                                        // 10/8
        || (ip >> 20) == (172u << 4 | 1u)                          // 172.16/12
        || (ip >> 16) == (192u << 8 | 168u)                        // 192.168/16
        || (ip >> 16) == (169u << 8 | 254u);                       // link-local
}

std::vector<std::string> WebServer::collectLocalAddrs()
{
    std::vector<std::string> out;
#ifdef _WIN32
    ULONG sz = 16 * 1024;
    std::vector<unsigned char> buf(sz);
    auto* aa = reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buf.data());
    if (GetAdaptersAddresses(AF_UNSPEC, GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST
                             | GAA_FLAG_SKIP_DNS_SERVER, nullptr, aa, &sz) == NO_ERROR)
    {
        for (auto* ad = aa; ad; ad = ad->Next)
            for (auto* ua = ad->FirstUnicastAddress; ua; ua = ua->Next)
            {
                char host[NI_MAXHOST] = {};
                if (getnameinfo(ua->Address.lpSockaddr, (socklen_t)ua->Address.iSockaddrLength,
                                host, sizeof(host), nullptr, 0, NI_NUMERICHOST) == 0)
                    out.emplace_back(host);
            }
    }
#else
    ifaddrs* ifa0 = nullptr;
    if (getifaddrs(&ifa0) == 0)
    {
        for (ifaddrs* ifa = ifa0; ifa; ifa = ifa->ifa_next)
        {
            if (!ifa->ifa_addr) continue;
            char host[64] = {};
            if (ifa->ifa_addr->sa_family == AF_INET)
                inet_ntop(AF_INET, &((sockaddr_in*)ifa->ifa_addr)->sin_addr, host, sizeof(host));
            else if (ifa->ifa_addr->sa_family == AF_INET6)
                inet_ntop(AF_INET6, &((sockaddr_in6*)ifa->ifa_addr)->sin6_addr, host, sizeof(host));
            else
                continue;
            if (host[0]) out.emplace_back(host);
        }
        freeifaddrs(ifa0);
    }
#endif
    for (auto& s : out)
    {
        // Drop an IPv6 scope suffix ("fe80::1%eth0") - Host headers never carry one.
        const size_t pct = s.find('%');
        if (pct != std::string::npos) s.erase(pct);
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c) { return (char)std::tolower(c); });
    }
    return out;
}

bool WebServer::hostAllowed(const std::string& hostHeader)
{
    const std::string name = hostHeaderName(hostHeader);
    if (name.empty()) return false;   // HTTP/1.1 requires Host; absent = fail closed

    if (name == "localhost" || name == "127.0.0.1" || name == "::1") return true;

    // This machine's own hostname is always allowed, bare and as the mDNS
    // ".local" form -- browsing the controller by its advertised name must
    // work with zero configuration.
    static const std::string ownHost = []{
        char hn[256] = {};
        if (gethostname(hn, sizeof(hn) - 1) != 0) return std::string();
        std::string h(hn);
        std::transform(h.begin(), h.end(), h.begin(),
                       [](unsigned char c) { return (char)std::tolower(c); });
        return h;
    }();
    if (!ownHost.empty() && (name == ownHost || name == ownHost + ".local")) return true;
    if (!m_bindAddr.empty() && m_bindAddr != "0.0.0.0" && name == m_bindAddr) return true;
    for (const auto& h : m_extraHosts)
        if (name == hostHeaderName(h)) return true;

    std::lock_guard<std::mutex> lk(m_hostCacheMutex);
    auto inCache = [&] {
        return std::find(m_localAddrs.begin(), m_localAddrs.end(), name) != m_localAddrs.end();
    };
    if (inCache()) return true;
    // Miss: the interface set may have changed (DHCP renew, USB NIC replug -     // the eth1 class of event). Rescan at most once per 5s, then re-check.
    const auto now = std::chrono::steady_clock::now();
    if (now - m_lastAddrScan > std::chrono::seconds(5))
    {
        m_lastAddrScan = now;
        m_localAddrs   = collectLocalAddrs();
        if (inCache()) return true;
    }
    return false;
}

WebServer::WebServer() = default;

WebServer::~WebServer()
{
    stop();
}

void WebServer::setComponents(MotionController* motion,
                               ControlLoop*      loop,
                               EtherCATMaster*   master,
                               const AppConfig*  config)
{
    m_motion = motion;
    m_loop   = loop;
    m_master = master;
    m_config = config;
}

// ============================================================
// buildStatusJson
// ============================================================
std::string WebServer::buildStatusJson() const
{
    // Gather state from thread-safe accessors only
    bool estop      = m_motion ? m_motion->isEmergencyStop() : false;
    bool loopRunning = m_loop  ? m_loop->isRunning()         : false;
    bool masterOp   = m_master ? m_master->isOperational()   : false;

    LoopStats stats;
    if (m_loop) stats = m_loop->getStats();

    MotionStatus ms;
    if (m_motion) ms = m_motion->getMotionStatus();

    // Rig-level aggregates: SINGLE SOURCE in StatusModel --
    // the same derivations resolve the toggle endpoints, so the dashboard
    // and a physical toggle button can never disagree about parked/slack.
    const status::MotionAggregates magg =
        status::deriveMotionAggregates(ms.axisState, ms.numDrives);
    bool torqueMask[MAX_DRIVES] = {};
    const int nCfg = torqueModeMask(torqueMask);
    const status::BeltAggregates bagg =
        status::deriveBeltAggregates(ms.axisState, ms.numDrives, torqueMask, nCfg);
    const bool anyHoming = magg.anyHoming;
    const bool allParked = magg.allParked;
    const bool hasBelts  = bagg.hasBelts;
    const bool beltsSlack = bagg.beltsSlack;

    // Cards come from config so they render before Initialize/Start; slavesFound
    // is the live count from the master. Per-drive live fields are gated on the loop.
    int numDrives   = m_config ? static_cast<int>(m_config->drives.size()) : ms.numDrives;
    if (numDrives == 0) numDrives = ms.numDrives;
    int slavesFound = m_master ? m_master->getSlaveCount() : 0;

    // Unix-ms timestamp so the browser can detect stale data
    int64_t tsMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // ---- Build JSON manually ----
    std::string s;
    s.reserve(512);
    s += "{";
    s += "\"ts\":"           + std::to_string(tsMs)            + ",";
    s += "\"loopRunning\":"  + jsonBool(loopRunning)           + ",";
    s += "\"masterOp\":"     + jsonBool(masterOp)             + ",";
    s += "\"estop\":"        + jsonBool(estop)                + ",";
    s += "\"needsRehome\":"  + jsonBool(ms.needsRehome)       + ",";
    s += "\"homing\":"       + jsonBool(anyHoming)            + ",";
    s += "\"parked\":"       + jsonBool(allParked)            + ",";
    s += "\"hasBelts\":"     + jsonBool(hasBelts)             + ",";
    s += "\"beltsSlack\":"   + jsonBool(beltsSlack)           + ",";
    s += "\"numDrives\":"    + jsonInt(numDrives)             + ",";
    s += "\"slavesFound\":"  + jsonInt(slavesFound)           + ",";
    s += "\"loopHz\":"       + jsonDouble(stats.loopHz, 1)    + ",";
    s += "\"maxJitterUs\":"  + jsonDouble(stats.maxJitterUs, 1) + ",";
    s += "\"wkcErrors\":"    + jsonInt(stats.wkcErrors)       + ",";
    s += "\"initBusy\":"   + jsonBool(m_initBusy.load())          + ",";
    s += "\"telemetryReceiving\":" + jsonBool(m_telemetry && m_telemetry->hasRecentData()) + ",";
    s += "\"telemetryInit\":"      + jsonBool(m_telemetry && m_telemetry->isInitialized()) + ",";
    // UDP telemetry-rate diagnostic (-1 when diagnostics off / no window yet).
    s += "\"udpArrivalHz\":" + jsonDouble(m_telemetry ? m_telemetry->getUdpArrivalHz() : -1.0, 0) + ",";
    s += "\"udpNewHz\":"     + jsonDouble(m_telemetry ? m_telemetry->getUdpNewHz()     : -1.0, 0) + ",";
    s += "\"udpHoldPct\":"   + jsonDouble(m_telemetry ? m_telemetry->getUdpHoldPct()   : -1.0, 0) + ",";
    s += "\"drives\":[";
    // Canonical status (shared StatusModel): per-drive indicator collected here so
    // the aggregate below is computed by the model's single precedence rule, not a
    // second copy. Additive -- existing fields (sw/state/...) are untouched, so the
    // current web renderer is unaffected until it is refactored to consume `ind`.
    std::vector<status::AxisIndicator> inds;
    inds.reserve(numDrives);
    for (int i = 0; i < numDrives; ++i)
    {
        // index first (no leading comma); every later field is comma-prefixed,
        // so omitting the live block when the loop is stopped stays valid JSON.
        s += "{\"index\":" + jsonInt(i);

        // config fields - always present, so cards render before Initialize
        if (m_config && i < static_cast<int>(m_config->drives.size()))
        {
            const DriveConfig& dc = m_config->drives[i];
            s += ",\"name\":"           + jsonStr(dc.name);
            s += ",\"axisType\":"       + jsonStr(dc.axisType);
            s += ",\"mode\":"           + jsonStr(dc.mode);   // csp/pp/torque - card hides position rows in torque
            s += ",\"invertDir\":"      + jsonBool(dc.invertDir);
            s += ",\"strokeMm\":"       + jsonDouble(dc.strokeMm, 1);
            s += ",\"ballscrewPitch\":" + jsonDouble(dc.ballscrewPitch, 2);
            s += ",\"reductionRatio\":" + jsonStr(dc.reductionRatio);  // rotary card shows the gear ratio
            s += ",\"maxAccel\":"       + jsonDouble(dc.maxAccelerationMmS2, 0); // Amax, for the accel bar
        }

        // electrical/PDO fields - available whenever the master is OP, including
        // during init/enable before the loop runs (statusword, position, torque
        // read straight from the drive's live PDO).
        if (masterOp && m_master && i < m_master->getDriveCount() && m_master->getDrive(i))
        {
            A6Drive* dr = m_master->getDrive(i);
            s += ",\"sw\":"  + jsonInt(dr->getStatusword());
            s += ",\"pos\":" + jsonDouble(dr->getActualPosition(), 3);
            s += ",\"trq\":" + jsonDouble(dr->getTorquePercent(), 1);
            // IGBT temperature from the SdoWorker's OP-time round-robin poll (~15s/drive).
            if (m_config && i < static_cast<int>(m_config->drives.size()))
            {
                SdoWorker* sw = m_master->sdoWorker();
                int slave = m_config->drives[i].slaveIndex;
                if (sw && sw->tempValid(static_cast<uint16_t>(slave)))
                    s += ",\"igbtC\":" + jsonDouble(sw->tempLatestC(static_cast<uint16_t>(slave)), 0);
            }
        }
        // motion-layer + loop-computed fields - only while the control loop runs
        if (loopRunning)
        {
            if (i < ms.numDrives)
            {
                s += ",\"state\":" + jsonStr(ms.axisStateName[i]);
                s += ",\"homed\":" + jsonBool(ms.homed[i]);
                s += ",\"accelPeakMms2\":" + jsonDouble(ms.accelWinPeakMms2[i], 0); // peak WINDOWED commanded accel (headroom gauge)
                s += ",\"accelClipPct\":"  + jsonDouble(ms.accelClipPct[i], 1);     // % cycles accel clamp bound
                s += ",\"accelBindPct\":"  + jsonDouble(ms.accelBindPct[i], 1);     // % cycles braking clamp bound
            }
            DriveStatus ds;
            if (m_loop) ds = m_loop->getDriveStatus(i);
            s += ",\"vel\":"        + jsonDouble(ds.velocity, 2);
            s += ",\"target\":"     + jsonDouble(ds.targetPos, 3);
            s += ",\"followErrPeak\":" + jsonDouble(ds.peakFollowingError, 3);      // latched, soft-reset
            // Torque-axis card telemetry: commanded tension, thermal duty RMS, shaft
            // rpm (velocity is in pseudo-mm units; one rev = encCountsPerRev/countsPerMm),
            // and guard state (sticky trip reason so a trip is card-visible, not log-only).
            if (m_config && i < static_cast<int>(m_config->drives.size())
                && m_config->drives[i].mode == "torque")
            {
                const DriveConfig& tdc = m_config->drives[i];
                s += ",\"cmdTrq\":" + jsonDouble((i < ms.numDrives) ? ms.beltCmdPct[i] : 0.0, 1);
                s += ",\"rms\":"    + jsonDouble(ds.torqueRms60, 0);
                double unitsPerRev = (tdc.countsPerMm > 0.0)
                                   ? tdc.encoderCountsPerRev / tdc.countsPerMm : 0.0;
                if (unitsPerRev > 0.0)
                    s += ",\"rpm\":" + jsonDouble(ds.velocity / unitsPerRev * 60.0, 0);
                static const char* GUARD[] = { "", "overspeed", "travel", "relaxed" };
                uint8_t g = (i < ms.numDrives && ms.beltGuard[i] <= 3) ? ms.beltGuard[i] : 0;
                s += ",\"guard\":" + jsonStr(GUARD[g]);
            }
        }

        // Canonical indicator (additive) - derived from the same inputs the web uses
        // (statusword when OP, motion state when the loop runs). state/text/fault +
        // the shared colour class/pattern, so any renderer shows the same thing.
        bool hasSw = (masterOp && m_master && i < m_master->getDriveCount() && m_master->getDrive(i));
        uint16_t sw = hasSw ? m_master->getDrive(i)->getStatusword() : 0;
        AxisMotionState mst = (loopRunning && i < ms.numDrives) ? ms.axisState[i]
                                                                : AxisMotionState::PARKED;
        DriveState rawDrive = (loopRunning && m_loop) ? m_loop->getDriveStatus(i).state
                                                      : DriveState::Unknown;
        status::AxisIndicator ind = status::deriveAxis(
            i, mst,
            (loopRunning && i < ms.numDrives) ? ms.axisStateName[i] : std::string(),
            hasSw, sw, rawDrive, loopRunning, estop);
        const status::Style& stl = status::styleOf(ind.state);
        s += ",\"ind\":{\"state\":"  + jsonStr(status::indicatorToken(ind.state))
           + ",\"text\":"    + jsonStr(ind.text)
           + ",\"fault\":"   + jsonBool(ind.fault)
           + ",\"cls\":"     + jsonStr(stl.webClass)
           + ",\"pattern\":" + jsonStr(stl.pattern);
        // Decoded fault identity (additive). Precise Er name when the
        // recovery thread's one-shot 0x203F read has landed; otherwise the
        // coarse 603F class + candidate list from the live PDO word.
        if (ind.fault && hasSw)
        {
            A6Drive* dp = m_master->getDrive(i);
            uint16_t bus   = dp ? dp->getFaultCode() : 0;
            uint32_t panel = dp ? dp->getPanelCode() : 0;
            const A6FaultInfo* fi = a6PanelFault(static_cast<uint16_t>(panel & 0xFFFF));
            if (fi)
                s += ",\"faultCode\":" + jsonStr(fi->er)
                   + ",\"faultText\":" + jsonStr(std::string(fi->name)
                        + (fi->resettable ? "" : " -- not resettable, power cycle"));
            else if (bus != 0)
                s += ",\"faultCode\":" + jsonStr(strf("0x%04x", bus))
                   + ",\"faultText\":" + jsonStr(a6BusFaultCandidates(bus));
        }
        s += "}";
        inds.push_back(ind);

        s += "}";
        if (i < numDrives - 1) s += ",";
    }
    s += "]";

    // Aggregate / summary indicator - single precedence rule, in the model.
    status::Indicator agg = status::deriveAggregate(inds.data(), static_cast<int>(inds.size()),
                                                    loopRunning, estop);
    const status::Style& aggStl = status::styleOf(agg);
    s += ",\"aggregate\":{\"state\":" + jsonStr(status::indicatorToken(agg))
       + ",\"cls\":"     + jsonStr(aggStl.webClass)
       + ",\"pattern\":" + jsonStr(aggStl.pattern) + "}";
    s += "}";
    return s;
}

// ============================================================
// readWebFile
// ============================================================
std::string WebServer::readWebFile(const std::string& filename) const
{
    std::string path = m_webRoot + "/" + filename;
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) return {};
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

const char* WebServer::mimeType(const std::string& filename)
{
    if (filename.size() >= 5 && filename.substr(filename.size() - 5) == ".html") return "text/html";
    if (filename.size() >= 3 && filename.substr(filename.size() - 3) == ".js")   return "application/javascript";
    if (filename.size() >= 4 && filename.substr(filename.size() - 4) == ".css")  return "text/css";
    if (filename.size() >= 5 && filename.substr(filename.size() - 5) == ".json") return "application/json";
    return "text/plain";
}

// ============================================================
// EtherCAT bring-up requests (shared by web endpoints + GPIO panel)
// ============================================================
void WebServer::setRefusal(const char* why)
{
    std::lock_guard<std::mutex> lk(m_initMutex);
    m_lastRefusal = why;
    LOG_WARNING(std::string("WebServer: request refused -- ") + why);
}

std::string WebServer::lastRefusal() const
{
    std::lock_guard<std::mutex> lk(m_initMutex);
    return m_lastRefusal.empty() ? std::string("request refused") : m_lastRefusal;
}

bool WebServer::requestInit()
{
    if (!m_master || !m_config)
    { setRefusal("components not ready (no master/config)"); return false; }
    if (m_master->isOperational())
    { setRefusal("already operational -- run Stop EtherCAT first"); return false; }
    if (m_initBusy.load())
    { setRefusal("an init/de-init is already in progress"); return false; }

    m_initBusy.store(true);
    {
        std::lock_guard<std::mutex> lk(m_initMutex);
        m_initLastError.clear();
        m_lastRefusal.clear();
        m_initRequested = true;
    }
    m_initCv.notify_one();
    return true;
}

bool WebServer::requestDeinit()
{
    if (!m_master)
    { setRefusal("components not ready (no master)"); return false; }
    if (!m_master->isInitialized())
    { setRefusal("EtherCAT is not initialized"); return false; }
    if (m_initBusy.load())
    { setRefusal("an init/de-init is already in progress"); return false; }

    m_initBusy.store(true);
    {
        std::lock_guard<std::mutex> lk(m_initMutex);
        m_lastRefusal.clear();
        m_deinitRequested = true;
    }
    m_initCv.notify_one();
    return true;
}

// ============================================================
// start / stop
// ============================================================
bool WebServer::start()
{
    if (m_running.load()) return true;

    // Pre-create the init worker thread so it is warm and scheduled by the
    // time the user clicks Init. Avoids 1-5ms cold thread creation at click
    // time, which can push SOEM init past its timing margin and fail it.
    m_initThreadStop = false;
    m_initThread = std::thread([this]()
    {
#ifdef _WIN32
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
#endif
        while (true)
        {
            std::unique_lock<std::mutex> lk(m_initMutex);
            m_initCv.wait(lk, [this]{ return m_initRequested || m_deinitRequested || m_initThreadStop; });
            if (m_initThreadStop) break;
            bool doInit   = m_initRequested;
            bool doDeinit = m_deinitRequested;
            m_initRequested   = false;
            m_deinitRequested = false;
            lk.unlock();

            if (doDeinit)
            {
                // Quiet the mailbox FIRST, while the loop still runs: the SDO
                // worker's in-flight transfer (temp poll) completes under the
                // live handler and the whole teardown below is mailbox-silent.
                // Stopping later (shutdown) risked an op dying half-conversed
                // when the loop had already exited.
                if (m_master) m_master->stopSdoWorker();
                // Inverse of Initialize. The control loop hands PDO cycling to
                // the background pump when it stops, so the drives sit in OP
                // with no clean way down. shutdown() disables the drives, walks
                // the slaves OP→INIT and closes the NIC - drives leave OP
                // without a DC-sync fault. Stop the loop first so the RT thread
                // has released SOEM before we tear the master down. masterOp
                // goes false afterwards, which re-enables Initialize.
                if (m_loop && m_loop->isRunning())
                {
                    // Defensive (the UI only enables Stop EtherCAT when stopped): a plain
                    // stop -- the axes park and stay energized in OP under the pump. Seating
                    // is NOT done here; it belongs to the de-energize step below.
                    m_loop->stop();
                    m_loop->waitForStop();
                }
                // Loop is now stopped, drives held in OP by the pump. Seat the vertical axes
                // onto the bottom stop (homing-based) and de-energize ON the stop, so the
                // OP→INIT teardown below doesn't free-fall them (the 1.5mm drop/thunk).
                if (m_loop) m_loop->seatThenStop();
                if (m_master) m_master->shutdown();
            }
            else if (doInit)
            {
                // Mirror the Qt Initialize button: re-apply the (possibly reloaded)
                // config to the motion controller so a rig.json save made while
                // EtherCAT was up takes effect on THIS init instead of needing an
                // application restart. Both init entry points must do this -- an
                // operator who edits and initialises entirely from the web UI never
                // touches the Qt button. Guarded on the loop being stopped:
                // configure() reseats axes to parkPos and clears homed/arms rehome,
                // which must never run under a live RT thread.
                if (m_motion && m_config && !(m_loop && m_loop->isRunning()))
                    m_motion->configure(*m_config);

                std::string nicName = m_config ? m_config->nicName : "";
                bool ok = m_master && m_master->initializeAndEnterOp(nicName);
                if (!ok)
                {
                    std::lock_guard<std::mutex> elk(m_initMutex);
                    m_initLastError = m_master ? m_master->getLastError() : "No master";
                }
            }
            m_initBusy.store(false);
        }
    });

    m_running.store(true);
    m_thread = std::thread([this]()
    {
        // The process runs at HIGH_PRIORITY_CLASS for the RT thread's benefit.
        // Explicitly drop this thread to below-normal so the web server's
        // accept/send loop cannot compete with the EtherCAT RT thread.
#ifdef _WIN32
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
        httplib::Server svr;
        m_svr.store(&svr);

        // Serve any static asset straight from web/ (logo.svg, fonts/*.woff2, …).
        // Explicit Get() handlers below take precedence; this is the fallback.
        svr.set_mount_point("/", m_webRoot);

        // ---- Static files ----
        // no-store: browser must not cache these files between builds.
        svr.Get("/", [this](const httplib::Request&, httplib::Response& res)
        {
            std::string body = readWebFile("index.html");
            if (body.empty())
            {
                res.set_content("<h1>nullCAT</h1><p>web/index.html not found.</p>", "text/html");
                return;
            }
            res.set_header("Cache-Control", "no-store");
            res.set_content(body, "text/html");
        });

        svr.Get("/app.js", [this](const httplib::Request&, httplib::Response& res)
        {
            std::string body = readWebFile("app.js");
            if (body.empty()) { res.status = 404; return; }
            res.set_header("Cache-Control", "no-store");
            res.set_content(body, "application/javascript");
        });

        svr.Get("/app.css", [this](const httplib::Request&, httplib::Response& res)
        {
            std::string body = readWebFile("app.css");
            if (body.empty()) { res.status = 404; return; }
            res.set_header("Cache-Control", "no-store");
            res.set_content(body, "text/css");
        });

        // ---- REST: recent log lines ----
        // Returns up to ?n=N lines (default 100, max 500) as a JSON array.
        svr.Get("/api/logs", [](const httplib::Request& req, httplib::Response& res)
        {
            int n = 100;
            if (req.has_param("n"))
            {
                try { n = std::stoi(req.get_param_value("n")); } catch (...) {}
                n = std::max(1, std::min(n, 500));
            }

            auto lines = Logger::instance().getRecentLogs(n);
            std::string json = "[";
            for (size_t i = 0; i < lines.size(); ++i)
            {
                json += jsonStr(lines[i]);
                if (i + 1 < lines.size()) json += ",";
            }
            json += "]";

            res.set_content(json, "application/json");
        });

        // ---- REST: status snapshot ----
        svr.Get("/api/status", [this](const httplib::Request&, httplib::Response& res)
        {
            res.set_content(buildStatusJson(), "application/json");
        });

        // Serve one of the split config files (host.json / rig.json) verbatim.
        auto serveConfigFile = [this](const char* name, httplib::Response& res)
        {
            if (m_configPath.empty()) { res.status = 404; res.set_content("{\"ok\":false,\"error\":\"no config path\"}", "application/json"); return; }
            std::ifstream f(siblingFile(m_configPath, name), std::ios::binary);
            if (!f) { res.status = 404; res.set_content(std::string("{\"ok\":false,\"error\":\"") + name + " not found\"}", "application/json"); return; }
            std::stringstream ss; ss << f.rdbuf();
            res.set_content(ss.str(), "application/json");
        };

        // GET /api/meta - surface-ownership hint for the web UI. hostOwner =
        // "native" when a native config UI owns host.json, "web" when headless.
        // The web shows/edits the host section only when it owns it.
        svr.Get("/api/meta", [this](const httplib::Request&, httplib::Response& res)
        {
            // Pending-restart truth is SERVER-owned so it
            // survives page reloads and every client agrees. A namespace is
            // "pending" when its file was modified after this process started
            // (one stat() per file per poll; meta is polled at UI cadence only).
            auto pendingSince = [this](const char* which) -> bool
            {
                if (m_configPath.empty()) return false;
                struct stat st{};
                if (::stat(siblingFile(m_configPath, which).c_str(), &st) != 0) return false;
                return st.st_mtime > m_processStart;
            };
            const bool rigPend  = pendingSince("rig.json");
            const bool hostPend = pendingSince("host.json");
            res.set_content(std::string("{\"hostOwner\":\"") + hostOwner()
                            + "\",\"configVersion\":2"
                            + ",\"rigPendingRestart\":"  + (rigPend  ? "true" : "false")
                            + ",\"hostPendingRestart\":" + (hostPend ? "true" : "false")
                            + "}",
                            "application/json");
        });

        // GET /api/rig - the portable rig config (axes + global feel).
        svr.Get("/api/rig",  [serveConfigFile](const httplib::Request&, httplib::Response& res) { serveConfigFile("rig.json",  res); });
        // GET /api/host - the per-machine host config (read-only display on PC).
        svr.Get("/api/host", [serveConfigFile](const httplib::Request&, httplib::Response& res) { serveConfigFile("host.json", res); });

        // ---- Button bindings ----
        // buttons.json is the THIRD namespace: per-machine (this host's box),
        // but WEB-OWNED ON BOTH PLATFORMS (unlike host.json, native-owned on
        // PC) - the wizard must save exactly where host.json is read-only.
        // Missing file = empty map, not an error.
        svr.Get("/api/buttons", [this](const httplib::Request&, httplib::Response& res)
        {
            std::ifstream f(m_configPath.empty() ? std::string()
                            : siblingFile(m_configPath, "buttons.json"), std::ios::binary);
            if (!f) { res.set_content("{\"configVersion\":1,\"bindings\":[]}", "application/json"); return; }
            std::stringstream ss; ss << f.rdbuf();
            res.set_content(ss.str(), "application/json");
        });

        // ---- REST: commands ----
        // No CORS headers anywhere: the UI is served same-origin and needs
        // none, and their absence is the first layer keeping arbitrary web
        // pages on the LAN from reading responses cross-origin.
        auto postCmd = [&](const std::string& path,
                           std::function<void(const httplib::Request&, httplib::Response&)> fn)
        {
            svr.Post(path, [fn](const httplib::Request& req, httplib::Response& res)
            {
                fn(req, res);
            });
        };

        // Helper: build ok/error JSON response
        auto okResp = [](httplib::Response& res)
        {
            res.set_content("{\"ok\":true}", "application/json");
        };
        auto errResp = [](httplib::Response& res, const std::string& msg)
        {
            res.set_content("{\"ok\":false,\"error\":" + jsonStr(msg) + "}",
                "application/json");
            res.status = 400;
        };

        // /api/init - async EtherCAT init. Returns immediately with {"ok":true,"status":"starting"}
        // or {"ok":false,"error":"..."} if already initializing/operational.
        // Progress is visible via the initBusy + masterOp fields in /api/status and WS push.
        postCmd("/api/init", [this, errResp](const httplib::Request&, httplib::Response& res)
        {
            if (!requestInit())
            {
                errResp(res, lastRefusal());
                return;
            }
            res.set_content("{\"ok\":true,\"status\":\"starting\"}", "application/json");
        });

        // /api/deinit - async EtherCAT de-init (inverse of /api/init). Stops the
        // loop if running, then brings the drives OP→INIT and closes the master,
        // so they leave OP cleanly (no DC-sync fault) and Initialize re-enables.
        postCmd("/api/deinit", [this, errResp](const httplib::Request&, httplib::Response& res)
        {
            if (!requestDeinit())
            {
                errResp(res, lastRefusal());
                return;
            }
            res.set_content("{\"ok\":true,\"status\":\"stopping\"}", "application/json");
        });

        // Shared: validate `body` for namespace `which` against the other
        // namespace on disk, then atomically replace <which>.json. Restart to
        // apply. Single writer per file: rig = web (both platforms); host =
        // web only on headless builds (refused here when natively owned).
        auto writeConfigNamespace = [this, errResp](const char* which,
            const std::vector<std::string>& errs, const std::string& body, httplib::Response& res)
        {
            if (!errs.empty())
            {
                std::string m; for (auto& e : errs) m += (m.empty() ? "" : "; ") + e;
                errResp(res, m);
                return;
            }
            const std::string path = siblingFile(m_configPath, which);
            const std::string tmp  = path + ".tmp";
            { std::ofstream o(tmp, std::ios::binary | std::ios::trunc);
              if (!o) { errResp(res, "Cannot write temp file."); return; } o << body; }
            // std::filesystem::rename is an atomic replace-if-exists on BOTH
            // platforms (MSVC -> MoveFileEx(MOVEFILE_REPLACE_EXISTING), Linux ->
            // rename(2)). Plain std::rename fails with EEXIST on the Windows CRT
            // when the target exists, which would break every save after the
            // first on a Windows build.
            std::error_code ec;
            std::filesystem::rename(tmp, path, ec);
            if (ec)
            { std::error_code ec2; std::filesystem::remove(tmp, ec2); errResp(res, "Save failed (rename)."); return; }
            LOG_INFO(std::string("WebServer: ") + which + " updated via web - restart to apply.");
            res.set_content("{\"ok\":true,\"restartRequired\":true}", "application/json");
        };

        // POST /api/rig - the web owns rig.json on both platforms.
        postCmd("/api/rig", [this, errResp, writeConfigNamespace](const httplib::Request& req, httplib::Response& res)
        {
            if (m_configPath.empty()) { errResp(res, "No config path configured."); return; }
            writeConfigNamespace("rig.json", Config::validateRigBody(m_configPath, req.body), req.body, res);
        });

        // POST /api/host - only honored on headless builds (hostOwner == "web").
        // When a native UI owns host.json, refuse so a browser cannot become a
        // second writer of host.json - single-writer enforced server-side, not
        // merely hidden in the UI.
        postCmd("/api/host", [this, errResp, writeConfigNamespace](const httplib::Request& req, httplib::Response& res)
        {
            if (std::string(hostOwner()) == "native")
            {
                res.status = 403;
                res.set_content("{\"ok\":false,\"error\":\"host config is managed by the native app on this machine\"}",
                                "application/json");
                return;
            }
            if (m_configPath.empty()) { errResp(res, "No config path configured."); return; }
            writeConfigNamespace("host.json", Config::validateHostBody(m_configPath, req.body), req.body, res);
        });

        // /api/start - start the control loop (drive must already be operational)
        postCmd("/api/start", [this, okResp, errResp](const httplib::Request&, httplib::Response& res)
        {
            if (!m_loop) { errResp(res, "Components not ready."); return; }
            if (!m_master || !m_master->isOperational()) { errResp(res, "EtherCAT not operational. Run /api/init first."); return; }
            if (m_loop->isRunning()) { okResp(res); return; }
            m_loop->start();
            okResp(res);
        });

        postCmd("/api/stop", [this, okResp](const httplib::Request&, httplib::Response& res)
        {
            if (m_onStopRequested) m_onStopRequested();
            okResp(res);
        });

        postCmd("/api/estop", [this, okResp, errResp](const httplib::Request&, httplib::Response& res)
        {
            if (!m_motion || !m_master) { errResp(res, "Components not ready."); return; }
            m_motion->setEmergencyStop(true);
            m_master->disableAllDrives();   // immediately halt drives via EtherCAT
            okResp(res);
        });

        postCmd("/api/estop/release", [this, okResp](const httplib::Request&, httplib::Response& res)
        {
            if (m_motion) m_motion->setEmergencyStop(false);
            okResp(res);
        });

        postCmd("/api/home", [this, okResp, errResp](const httplib::Request& req, httplib::Response& res)
        {
            if (!m_motion) { errResp(res, "Motion controller not ready."); return; }
            MotionCommand cmd;
            cmd.type   = MotionCommand::Type::StartHoming;
            cmd.intVal = -1;                    // default: all axes
            // Optional {"axis": N} homes one axis (1-based, chain order).
            // Hexapods need this: six coupled legs torque-searching at once
            // can trip each other's thresholds, so legs home one at a time.
            if (!req.body.empty())
            {
                QJsonParseError pe;
                const QJsonDocument doc = QJsonDocument::fromJson(
                    QByteArray(req.body.c_str(), (int)req.body.size()), &pe);
                if (pe.error == QJsonParseError::NoError && doc.isObject())
                {
                    const int axis = doc.object().value("axis").toInt(0);
                    if (axis != 0)
                    {
                        const int n = m_config ? (int)m_config->drives.size() : 0;
                        if (axis < 1 || axis > n)
                        { errResp(res, "axis out of range."); return; }
                        cmd.intVal = axis - 1;
                    }
                }
            }
            m_motion->enqueueCommand(cmd);
            okResp(res);
        });

        postCmd("/api/park", [this, okResp, errResp](const httplib::Request&, httplib::Response& res)
        {
            if (!m_motion) { errResp(res, "Motion controller not ready."); return; }
            MotionCommand cmd;
            cmd.type = MotionCommand::Type::StartPark;
            m_motion->enqueueCommand(cmd);
            okResp(res);
        });

        postCmd("/api/unpark", [this, okResp, errResp](const httplib::Request&, httplib::Response& res)
        {
            if (!m_motion) { errResp(res, "Motion controller not ready."); return; }
            MotionCommand cmd;
            cmd.type = MotionCommand::Type::StartUnpark;
            m_motion->enqueueCommand(cmd);
            okResp(res);
        });

        // Belt don/doff (torque axes only; position axes untouched). Two explicit
        // idempotent endpoints -- NOT a toggle -- so a physical button bound to
        // "tension" (GPIO panel or HID button box) can never slack you mid-lap
        // because UI state drifted.
        postCmd("/api/belts/slack", [this, okResp, errResp](const httplib::Request&, httplib::Response& res)
        {
            if (!m_motion) { errResp(res, "Motion controller not ready."); return; }
            MotionCommand cmd;
            cmd.type = MotionCommand::Type::SlackBelts;
            m_motion->enqueueCommand(cmd);
            okResp(res);
        });
        postCmd("/api/belts/tension", [this, okResp, errResp](const httplib::Request&, httplib::Response& res)
        {
            if (!m_motion) { errResp(res, "Motion controller not ready."); return; }
            MotionCommand cmd;
            cmd.type = MotionCommand::Type::TensionBelts;
            m_motion->enqueueCommand(cmd);
            okResp(res);
        });

        // GPIO panel LED self-test (no-op if the panel/mode has no LEDs).
        postCmd("/api/gpio/ledtest", [this, okResp, errResp](const httplib::Request&, httplib::Response& res)
        {
            if (!m_onLedTest) { errResp(res, "GPIO panel not active."); return; }
            m_onLedTest();
            okResp(res);
        });

        postCmd("/api/reset-fault", [this, okResp, errResp](const httplib::Request&, httplib::Response& res)
        {
            if (!m_master || !m_loop) { errResp(res, "Components not ready."); return; }
            // Two-step: clear physical drive faults via EtherCAT, then clear software lockout
            m_master->resetAllFaults();
            m_loop->clearFaultLockout(-1);
            okResp(res);
        });

        // ---- Commissioning test mode ----
        // The browser sends a compact spec (mode + axis selections + params);
        // the plan is built HERE with the CommissioningMode builders so amp
        // percentages, mixing weights, and note parsing have one C++
        // implementation (unit-tested), not a JS twin. The RT thread applies
        // its own entry rails (homed/PARKED/telemetry-quiet) when it picks
        // the plan up -- a 200 here means "queued", not "running"; poll
        // /api/test/status for the verdict.
        postCmd("/api/test/start", [this, okResp, errResp](const httplib::Request& req, httplib::Response& res)
        {
            if (!m_motion) { errResp(res, "Motion controller not ready."); return; }
            if (!(m_loop && m_loop->isRunning()))
            { errResp(res, "Control loop not running -- initialize and start first."); return; }

            QJsonParseError pe;
            const QJsonDocument doc = QJsonDocument::fromJson(
                QByteArray(req.body.c_str(), (int)req.body.size()), &pe);
            if (pe.error != QJsonParseError::NoError || !doc.isObject())
            { errResp(res, "Invalid JSON body."); return; }
            const QJsonObject o = doc.object();

            // Axis metadata: kind + stroke from config, selection + cycle
            // roles (front/rear, left/right) from the request.
            CommissioningAxisMeta meta[MAX_DRIVES] = {};
            const int n = m_config
                ? std::min((int)m_config->drives.size(), (int)MAX_DRIVES) : 0;
            for (int i = 0; i < n; ++i)
            {
                const DriveConfig& dc = m_config->drives[i];
                meta[i].kind = (uint8_t)axisCaps(dc.axisType, dc.mode).commissioningKind;
                meta[i].halfStrokeMm = dc.strokeMm / 2.0;
            }
            for (const QJsonValue& v : o.value("axes").toArray())
            {
                const QJsonObject a = v.toObject();
                const int i = a.value("i").toInt(-1);
                if (i < 0 || i >= n) continue;
                meta[i].selected  = a.value("sel").toBool(false);
                meta[i].frontRear = (int8_t)a.value("fr").toInt(0);
                meta[i].leftRight = (int8_t)a.value("lr").toInt(0);
                // Optional explicit mixing weights [wPitch, wRoll, wHeave]:
                // presence switches this axis off the legacy corner roles
                // (buildCycle clamps to [-1, 1]).
                const QJsonArray w = a.value("w").toArray();
                if (w.size() == 3)
                {
                    meta[i].useWeights = true;
                    meta[i].wPitch = w.at(0).toDouble(0.0);
                    meta[i].wRoll  = w.at(1).toDouble(0.0);
                    meta[i].wHeave = w.at(2).toDouble(0.0);
                }
            }

            const std::string mode = o.value("mode").toString().toStdString();
            CommissioningPlan plan;
            int built = -1;
            if (mode == "cycle")
            {
                CommissioningCycleParams p;
                p.enPitch  = o.value("enPitch").toBool(true);
                p.enRoll   = o.value("enRoll").toBool(true);
                p.enHeave  = o.value("enHeave").toBool(true);
                p.enHoriz  = o.value("enHoriz").toBool(true);
                p.pitchPct = o.value("pitchPct").toDouble(30.0);
                p.rollPct  = o.value("rollPct").toDouble(30.0);
                p.heavePct = o.value("heavePct").toDouble(40.0);
                p.horizPct = o.value("horizPct").toDouble(30.0);
                p.freqHz   = o.value("freqHz").toDouble(0.2);
                p.cycles   = o.value("cycles").toInt(2);
                built = CommissioningMode::buildCycle(p, meta, n, plan);
            }
            else if (mode == "tone")
            {
                built = CommissioningMode::buildTone(
                    o.value("freqHz").toDouble(25.0),
                    o.value("pct").toDouble(2.0),
                    o.value("durationSec").toDouble(5.0), meta, n, plan);
            }
            else if (mode == "sweep")
            {
                built = CommissioningMode::buildSweep(
                    o.value("f0").toDouble(5.0),
                    o.value("f1").toDouble(50.0),
                    o.value("stepHz").toDouble(2.5),
                    o.value("dwellSec").toDouble(2.0),
                    o.value("pct").toDouble(2.0), meta, n, plan);
            }
            else if (mode == "song")
            {
                built = CommissioningMode::buildSong(
                    o.value("notes").toString().toStdString().c_str(),
                    o.value("beatSec").toDouble(0.24),
                    o.value("pct").toDouble(2.0), meta, n, plan);
            }
            else if (mode == "step")
            {
                built = CommissioningMode::buildStep(
                    o.value("pct").toDouble(5.0),
                    o.value("holdSec").toDouble(3.0), meta, n, plan);
            }
            else { errResp(res, "Unknown test mode."); return; }

            if (built < 0) { errResp(res, "Invalid test parameters (check note names)."); return; }
            if (built == 0)
            { errResp(res, "No testable axes selected (belts are not testable; "
                           "cycle needs front/rear + left/right roles or "
                           "per-axis mixing weights)."); return; }
            if (!m_motion->requestCommissioningStart(plan))
            { errResp(res, "A test is already running."); return; }
            okResp(res);
        });

        postCmd("/api/test/stop", [this, okResp, errResp](const httplib::Request&, httplib::Response& res)
        {
            if (!m_motion) { errResp(res, "Motion controller not ready."); return; }
            m_motion->requestCommissioningStop();
            okResp(res);
        });

        svr.Get("/api/test/status", [this](const httplib::Request&, httplib::Response& res)
        {
            if (!m_motion)
            { res.set_content("{\"active\":false}", "application/json"); return; }
            const CommissioningStatus st = m_motion->getCommissioningStatus();
            std::string s = "{\"active\":" + jsonBool(st.active)
                + ",\"done\":"    + jsonBool(st.done)
                + ",\"aborted\":" + jsonBool(st.aborted)
                + ",\"phase\":"   + jsonStr(st.phase)
                + ",\"reason\":"  + jsonStr(st.reason)
                + ",\"title\":"   + jsonStr(st.title)
                + ",\"segIdx\":"      + std::to_string(st.segIdx)
                + ",\"numSegments\":" + std::to_string(st.numSegments)
                + ",\"progressPct\":" + strf("%.1f", st.progressPct)
                + ",\"results\":[";
            for (int r = 0; r < st.resultCount; ++r)
            {
                const CommissioningSegResult& sr = st.results[r];
                if (r) s += ",";
                s += "{\"label\":" + jsonStr(sr.label)
                   + ",\"kind\":"   + std::to_string((int)sr.kind)
                   + ",\"freqHz\":" + strf("%.2f", sr.freqHz) + ",\"axes\":[";
                bool first = true;
                for (int i = 0; i < MAX_DRIVES; ++i)
                {
                    const CommissioningAxisResult& ar = sr.axis[i];
                    if (!ar.tested) continue;
                    if (!first) s += ",";
                    first = false;
                    const double ratio = (ar.cmdAmpMm > 1e-6)
                        ? ar.actAmpMm / ar.cmdAmpMm : 0.0;
                    s += "{\"i\":" + std::to_string(i)
                       + ",\"cmdAmp\":"   + strf("%.3f", ar.cmdAmpMm)
                       + ",\"actAmp\":"   + strf("%.3f", ar.actAmpMm)
                       + ",\"ratio\":"    + strf("%.3f", ratio)
                       + ",\"phaseDeg\":" + strf("%.1f", ar.phaseDeg)
                       + ",\"ferrRms\":"  + strf("%.3f", ar.ferrRmsMm)
                       + ",\"ferrPeak\":" + strf("%.3f", ar.ferrPeakMm)
                       + ",\"trqRms\":"   + strf("%.1f", ar.trqRmsPct)
                       + ",\"derated\":"  + jsonBool(ar.derated);
                    if (sr.kind == 1)
                    {
                        s += ",\"osPct\":"    + strf("%.1f", ar.overshootPct)
                           + ",\"riseMs\":"   + strf("%.0f", ar.riseMs)
                           + ",\"settleMs\":" + strf("%.0f", ar.settleMs);
                    }
                    else
                    {
                        s += ",\"trqAmp\":" + strf("%.2f", ar.trqAmpPct);
                        // Load/inertia indicator: torque amplitude per unit of
                        // measured acceleration amplitude (% rated per m/s^2).
                        // Flat across a sweep = mass-dominated; a peak marks a
                        // resonance worth a drive-side notch.
                        const double wf = 2.0 * 3.14159265358979 * sr.freqHz;
                        const double accel = ar.actAmpMm / 1000.0 * wf * wf;
                        if (accel > 1e-6)
                            s += ",\"trqPerAcc\":" + strf("%.3f", ar.trqAmpPct / accel);
                    }
                    s += "}";
                }
                s += "]}";
            }
            s += "]}";
            res.set_content(s, "application/json");
        });

        // ---- Toggle endpoints: ONE physical button per stateful pair
        // (mirrors the dashboard and the GPIO panel's park
        // button). Resolution happens HERE against canonical engine state
        // (server-side: cannot drift), with two guards a single button needs
        // that a pair doesn't:
        //   1. transitions are NO-OPS, never reversals - a toggle only acts
        //      from a settled state (press during PARKING must not unpark);
        //   2. a per-toggle cooldown swallows double-press/bounce flip-flops.
        // Discrete endpoints stay for the dashboard, scripts, and legacy maps.
        auto toggleReady = [](std::atomic<int64_t>& lastMs) -> bool
        {
            const int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            if (now - lastMs.load() < 1500) return false;
            lastMs.store(now);
            return true;
        };
        auto resolvedResp = [](httplib::Response& res, const char* action)
        {
            res.set_content(std::string("{\"ok\":true,\"resolved\":\"") + action + "\"}",
                            "application/json");
        };

        postCmd("/api/init-toggle", [this, errResp, toggleReady, resolvedResp](const httplib::Request&, httplib::Response& res)
        {
            static std::atomic<int64_t> last{0};
            if (!toggleReady(last)) { errResp(res, "Toggle cooldown."); return; }
            // requestInit/requestDeinit already refuse while busy - guard 1 is
            // inherited for the whole bring-up/teardown window.
            if (m_master && m_master->isOperational())
            {
                if (!requestDeinit()) { errResp(res, lastRefusal()); return; }
                resolvedResp(res, "deinit");
            }
            else
            {
                if (!requestInit()) { errResp(res, lastRefusal()); return; }
                resolvedResp(res, "init");
            }
        });

        postCmd("/api/run-toggle", [this, errResp, toggleReady, resolvedResp](const httplib::Request&, httplib::Response& res)
        {
            if (!m_loop) { errResp(res, "Components not ready."); return; }
            static std::atomic<int64_t> last{0};
            if (!toggleReady(last)) { errResp(res, "Toggle cooldown."); return; }
            if (m_loop->isRunning())
            {
                if (m_onStopRequested) m_onStopRequested();
                resolvedResp(res, "stop");
            }
            else
            {
                if (!m_master || !m_master->isOperational()) { errResp(res, "EtherCAT not operational. Run init first."); return; }
                m_loop->start();
                resolvedResp(res, "start");
            }
        });

        postCmd("/api/park-toggle", [this, errResp, toggleReady, resolvedResp](const httplib::Request&, httplib::Response& res)
        {
            if (!m_motion) { errResp(res, "Motion controller not ready."); return; }
            static std::atomic<int64_t> last{0};
            if (!toggleReady(last)) { errResp(res, "Toggle cooldown."); return; }
            MotionStatus ms = m_motion->getMotionStatus();
            // Single-source aggregates (StatusModel): identical
            // derivation to /api/status, pinned by TestStatusModel.
            const status::MotionAggregates agg =
                status::deriveMotionAggregates(ms.axisState, ms.numDrives);
            if (agg.transitional) { errResp(res, "Transitioning -- toggle ignored."); return; }
            MotionCommand c;
            c.type = agg.allParked ? MotionCommand::Type::StartUnpark : MotionCommand::Type::StartPark;
            m_motion->enqueueCommand(c);
            resolvedResp(res, agg.allParked ? "unpark" : "park");
        });

        postCmd("/api/belts-toggle", [this, errResp, toggleReady, resolvedResp](const httplib::Request&, httplib::Response& res)
        {
            if (!m_motion) { errResp(res, "Motion controller not ready."); return; }
            static std::atomic<int64_t> last{0};
            if (!toggleReady(last)) { errResp(res, "Toggle cooldown."); return; }
            MotionStatus ms = m_motion->getMotionStatus();
            // Single-source aggregates (StatusModel): identical
            // derivation to /api/status, pinned by TestStatusModel.
            bool torqueMask[MAX_DRIVES] = {};
            const int nCfg = torqueModeMask(torqueMask);
            const status::BeltAggregates agg =
                status::deriveBeltAggregates(ms.axisState, ms.numDrives, torqueMask, nCfg);
            const bool beltsSlack = agg.beltsSlack;
            if (!agg.hasBelts)    { errResp(res, "No torque axes on this rig."); return; }
            if (agg.transitional) { errResp(res, "Belt transitioning -- toggle ignored."); return; }
            // Make the e-stop refusal VISIBLE here (the engine would refuse
            // silently): a blind press must not read as accepted.
            if (beltsSlack && m_motion->isEmergencyStop())
            { errResp(res, "E-stop active -- tension refused."); return; }
            MotionCommand c;
            c.type = beltsSlack ? MotionCommand::Type::TensionBelts : MotionCommand::Type::SlackBelts;
            m_motion->enqueueCommand(c);
            resolvedResp(res, beltsSlack ? "belts/tension" : "belts/slack");
        });

        // ---- Button bindings - save (hot-applies, no restart) and
        // the capture flow for the web wizard. The bindable-command set is
        // enforced server-side in Config::validateButtonsBody; a crafted POST
        // cannot bind restart/shutdown/estop-release.
        postCmd("/api/buttons", [this, errResp](const httplib::Request& req, httplib::Response& res)
        {
            if (m_configPath.empty()) { errResp(res, "No config path configured."); return; }
            auto errs = Config::validateButtonsBody(req.body);
            if (!errs.empty())
            {
                std::string m; for (auto& e : errs) m += (m.empty() ? "" : "; ") + e;
                errResp(res, m);
                return;
            }
            const std::string path = siblingFile(m_configPath, "buttons.json");
            const std::string tmp  = path + ".tmp";
            { std::ofstream o(tmp, std::ios::binary | std::ios::trunc);
              if (!o) { errResp(res, "Cannot write temp file."); return; } o << req.body; }
            std::error_code ec;
            std::filesystem::rename(tmp, path, ec);
            if (ec)
            { std::error_code ec2; std::filesystem::remove(tmp, ec2); errResp(res, "Save failed (rename)."); return; }
            if (m_buttonHooks.bindingsChanged) m_buttonHooks.bindingsChanged();
            LOG_INFO("WebServer: buttons.json updated via web - hot-applied.");
            res.set_content("{\"ok\":true,\"hotApplied\":true}", "application/json");
        });

        postCmd("/api/buttons/listen", [this, okResp, errResp](const httplib::Request&, httplib::Response& res)
        {
            if (!m_buttonHooks.armCapture) { errResp(res, "No button-capture backend on this host."); return; }
            if (!m_buttonHooks.armCapture()) { errResp(res, "Capture backend refused (busy?)."); return; }
            okResp(res);
        });

        svr.Get("/api/buttons/capture", [this](const httplib::Request&, httplib::Response& res)
        {
            res.set_content(m_buttonHooks.pollCapture ? m_buttonHooks.pollCapture()
                                                      : "{\"captured\":false,\"backend\":\"none\"}",
                            "application/json");
        });

        // Soft stats reset: re-baseline the per-axis tuning metrics (peak commanded
        // accel, Amax clip rate, relative-braking binds, peak following-error) as a
        // matched set WITHOUT dropping drives out of OP. Used while tuning Amax live.
        postCmd("/api/resetstats", [this, okResp, errResp](const httplib::Request&, httplib::Response& res)
        {
            if (!m_loop) { errResp(res, "Components not ready."); return; }
            m_loop->requestStatsReset();
            okResp(res);
        });

        // Graceful shutdown - watchdog will NOT relaunch (exit code 0)
        // Power OFF the machine (clean OS shutdown). On Linux the non-root service
        // calls `systemctl poweroff` via a NOPASSWD sudoers rule (see
        // pi/nullcat-poweroff.sudoers). Detached + delayed so the HTTP response
        // flushes first. On Windows this just exits the app (no poweroff).
        // Destructive endpoints carry one extra gate: the CLIENT address must
        // be private (loopback/LAN). Even a Host-allowlisted request from a
        // non-private source cannot power off or restart the controller.
        auto privateClientOnly = [](const httplib::Request& req, httplib::Response& res) -> bool
        {
            if (WebServer::isPrivateClientAddr(req.remote_addr)) return true;
            res.status = 403;
            res.set_content("{\"ok\":false,\"error\":\"local network clients only\"}",
                            "application/json");
            return false;
        };
        postCmd("/api/shutdown", [this, privateClientOnly](const httplib::Request& req, httplib::Response& res)
        {
            if (!privateClientOnly(req, res)) return;
            res.set_content("{\"ok\":true,\"status\":\"powering off\"}", "application/json");
#ifdef __linux__
            LOG_INFO("WebServer: shutdown requested via web UI - powering off.");
            std::thread([] {
                std::this_thread::sleep_for(std::chrono::milliseconds(600));
                if (std::system("sudo -n systemctl poweroff") != 0)
                    (void)std::system("sudo -n poweroff");
            }).detach();
#else
            if (m_onExitRequested) m_onExitRequested(0);
#endif
        });

        // Restart - watchdog WILL relaunch after 500ms (exit code 2)
        postCmd("/api/restart", [this, privateClientOnly](const httplib::Request& req, httplib::Response& res)
        {
            if (!privateClientOnly(req, res)) return;
            res.set_content("{\"ok\":true}", "application/json");
            if (m_onExitRequested) m_onExitRequested(2);
        });

        // ---- WebSocket: 10Hz state push ----
        // The handler runs for the lifetime of each client connection.
        // We push JSON at 10Hz; client messages are not expected.
        svr.WebSocket("/ws",
            [this](const httplib::Request&, httplib::ws::WebSocket& ws)
            {
                while (ws.is_open() && m_running.load())
                {
                    ws.send(buildStatusJson());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            });

        // Host-header allowlist, fail-closed BEFORE any route (including the
        // static mount): the DNS-rebinding defense. A rebinding page's request
        // carries its own hostname in Host, which never matches this machine.
        svr.set_pre_routing_handler(
            [this](const httplib::Request& req, httplib::Response& res) -> httplib::Server::HandlerResponse
            {
                if (hostAllowed(req.get_header_value("Host")))
                    return httplib::Server::HandlerResponse::Unhandled;
                res.status = 421;   // Misdirected Request
                res.set_content("{\"ok\":false,\"error\":\"Host not allowed\"}",
                                "application/json");
                return httplib::Server::HandlerResponse::Handled;
            });

        // Retry binding in case the port is still held from a previous crash.
        // Windows holds TCP ports in TIME_WAIT for ~30s after an unclean close.
        bool bound = false;
        for (int attempt = 0; attempt < 5 && m_running.load(); ++attempt)
        {
            if (attempt > 0)
            {
                LOG_WARNING(strf("WebServer: Port %d busy, retrying in 2s... (%d/5)",
                    m_port, attempt + 1));
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
            if (svr.bind_to_port(m_bindAddr.c_str(), m_port))
            {
                bound = true;
                break;
            }
        }

        if (!bound)
        {
            LOG_ERROR(strf("WebServer: Failed to bind to %s:%d after 5 attempts.",
                m_bindAddr.c_str(), m_port));
            m_svr.store(nullptr);
            m_running.store(false);
            return;
        }

        LOG_INFO(strf("WebServer: Listening on http://%s:%d", m_bindAddr.c_str(), m_port));

        svr.listen_after_bind();

        m_svr.store(nullptr);
        m_running.store(false);
        LOG_INFO("WebServer: Stopped.");
    });

    return true;
}

void WebServer::stop()
{
    m_running.store(false);
    // Signal svr.listen() to return - without this the server thread
    // blocks forever and join() hangs, causing a crash on app close.
    if (auto* svr = m_svr.load())
        svr->stop();
    if (m_thread.joinable()) m_thread.join();

    // Shut down the pre-created init worker thread
    {
        std::lock_guard<std::mutex> lk(m_initMutex);
        m_initThreadStop = true;
    }
    m_initCv.notify_one();
    if (m_initThread.joinable()) m_initThread.join();
}
