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
#include <cmath>
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
#include "DriveProvisioner.h" // role check: drive-resident params vs axis role (PreOp, read-only)
#include <vector>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <chrono>
#include <sys/stat.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <regex>
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

static long long fileMtime(const std::string& path)
{
    struct stat st{};
    return (::stat(path.c_str(), &st) == 0) ? static_cast<long long>(st.st_mtime) : 0;
}

static bool readWholeFile(const std::string& path, std::string& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    out.assign((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return true;
}

// Structural JSON equality (key order and number formatting do not count),
// so a save that changes nothing is recognised as such and not written.
static bool jsonEqual(const std::string& a, const std::string& b)
{
    const QJsonDocument da = QJsonDocument::fromJson(QByteArray::fromStdString(a));
    const QJsonDocument db = QJsonDocument::fromJson(QByteArray::fromStdString(b));
    if (da.isNull() || db.isNull()) return false;
    return da == db;
}

// True when two rig bodies differ ONLY in the parts the running process
// applies live on save (haptics tuning + per-device feel), i.e. nothing that
// needs an Initialize changed. Unparseable bodies count as "needs init".
static bool rigDiffIsLiveOnly(const std::string& oldBody, const std::string& newBody)
{
    auto strip = [](const std::string& s, QJsonObject& out) -> bool
    {
        const QJsonDocument d = QJsonDocument::fromJson(QByteArray::fromStdString(s));
        if (!d.isObject()) return false;
        out = d.object();
        QJsonObject g = out.value("global").toObject();
        g.remove("haptics");
        out.insert("global", g);
        QJsonArray axes = out.value("axes").toArray();
        for (int i = 0; i < axes.size(); ++i)
        {
            QJsonObject a = axes.at(i).toObject();
            a.remove("device");
            axes.replace(i, a);
        }
        out.insert("axes", axes);
        return true;
    };
    QJsonObject a, b;
    if (!strip(oldBody, a) || !strip(newBody, b)) return false;
    return a == b;
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
    bool beltMask[MAX_DRIVES] = {};
    const int nCfg = beltAxisMask(beltMask);
    const status::BeltAggregates bagg =
        status::deriveBeltAggregates(ms.axisState, ms.numDrives, beltMask, nCfg);
    bool devMask[MAX_DRIVES] = {};
    deviceAxisMask(devMask);
    const status::DeviceAggregates dagg =
        status::deriveDeviceAggregates(ms.axisState, ms.numDrives, ms.homed, devMask, nCfg);
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
    // Device toggle state (shared derivation with /api/device-toggle).
    s += "\"hasDevices\":"   + jsonBool(dagg.hasDevices)      + ",";
    s += "\"devEngaged\":"   + jsonBool(dagg.anyEngaged)      + ",";
    s += "\"devAllHomed\":"  + jsonBool(dagg.allHomed)        + ",";
    s += "\"numDrives\":"    + jsonInt(numDrives)             + ",";
    s += "\"slavesFound\":"  + jsonInt(slavesFound)           + ",";
    s += "\"loopHz\":"       + jsonDouble(stats.loopHz, 1)    + ",";
    s += "\"maxJitterUs\":"  + jsonDouble(stats.maxJitterUs, 1) + ",";
    s += "\"wkcErrors\":"    + jsonInt(stats.wkcErrors)       + ",";
    s += "\"initBusy\":"   + jsonBool(m_initBusy.load())          + ",";
    s += "\"telemetryReceiving\":" + jsonBool(m_telemetry && m_telemetry->hasRecentData()) + ",";
    s += "\"telemetryInit\":"      + jsonBool(m_telemetry && m_telemetry->isInitialized()) + ",";
    // NULLCATX channel stream indicator (device state effects wire) + how
    // many gear ratios the learner currently considers usable.
    s += "\"ncxRx\":"              + jsonBool(m_telemetry && m_telemetry->hasRecentNcx()) + ",";
    {
        // Haptics surface: fired counter, mute, live fx levels, and which
        // NCX tokens the wire is delivering (per-tile channel-health chips;
        // array order = NcxValues::Token order = the protocol registry).
        s += "\"hapFired\":" + std::to_string(ms.hapticsFired) + ",";
        s += "\"hapMuted\":" + jsonBool(ms.hapticsMuted) + ",";
        s += "\"hapFx\":[";
        for (int i = 0; i < haptics::FX_TYPE_COUNT; ++i)
            s += (i ? "," : "") + jsonDouble(ms.hapticsFxLevel[i], 3);
        // Tile readouts: what each effect is driven with (hapIn, with its
        // session peak hapInPk), the per-wheel sets [lateral severity,
        // longitudinal severity, road travel mm] now and at peak, and the
        // channel values with their peaks (same order as ncxHave).
        s += "],\"hapIn\":[";
        for (int i = 0; i < haptics::FX_TYPE_COUNT; ++i)
            s += (i ? "," : "") + jsonDouble(ms.hapticsIn[i], 3);
        s += "],\"hapInPk\":[";
        for (int i = 0; i < haptics::FX_TYPE_COUNT; ++i)
            s += (i ? "," : "") + jsonDouble(ms.hapticsInPk[i], 3);
        s += "],\"hapWheels\":[";
        for (int k = 0; k < MotionStatus::HAP_WHEEL_SETS; ++k)
        {
            s += (k ? ",[" : "[");
            for (int w = 0; w < 4; ++w) s += (w ? "," : "") + jsonDouble(ms.hapticsWheel[k][w], 3);
            s += "]";
        }
        s += "],\"hapWheelsPk\":[";
        for (int k = 0; k < MotionStatus::HAP_WHEEL_SETS; ++k)
        {
            s += (k ? ",[" : "[");
            for (int w = 0; w < 4; ++w) s += (w ? "," : "") + jsonDouble(ms.hapticsWheelPk[k][w], 3);
            s += "]";
        }
        s += "],\"ncxHave\":[";
        for (int i = 0; i < NcxValues::TokenCount; ++i)
            s += std::string(i ? "," : "") + (ms.ncxHave[i] ? "true" : "false");
        s += "],\"ncxVal\":[";
        for (int i = 0; i < NcxValues::TokenCount; ++i)
            s += (i ? "," : "") + jsonDouble(ms.ncxVal[i], 3);
        s += "],\"ncxPk\":[";
        for (int i = 0; i < NcxValues::TokenCount; ++i)
            s += (i ? "," : "") + jsonDouble(ms.ncxPk[i], 3);
        s += "],";
        // Sim identity from NULLCATY (empty until a sender names it).
        if (m_telemetry)
        {
            const TelemetryData td = m_telemetry->getLatestData();
            s += "\"simGame\":" + jsonStr((td.ncxFresh || td.ncxFrozen) ? td.game : "") + ",";
            s += "\"simCar\":"  + jsonStr((td.ncxFresh || td.ncxFrozen) ? td.car  : "") + ",";
            // Packets arriving but every value unchanged for 2 s (paused sim):
            // effects are released; the header shows it instead of "receiving".
            s += "\"ncxFrozen\":" + jsonBool(td.ncxFrozen) + ",";
        }
        // Shaker sink: the sound card state and per-channel output levels.
        {
            s += "\"shakers\":{";
            if (m_shakers)
            {
                s += "\"enabled\":" + jsonBool(m_config && m_config->shakersEnabled) + ",";
                s += "\"open\":" + jsonBool(m_shakers->anyOpen()) + ",";
                s += "\"channels\":" + jsonInt(m_shakers->totalChannels()) + ",";
                s += "\"devices\":[";
                bool first = true;
                for (const haptics::ShakerBank::Info& i : m_shakers->info())
                {
                    s += std::string(first ? "" : ",") + "{";
                    first = false;
                    s += "\"name\":" + jsonStr(i.open ? i.name : i.wanted) + ",";
                    s += "\"open\":" + jsonBool(i.open) + ",";
                    s += "\"error\":" + jsonStr(i.error) + ",";
                    s += "\"first\":" + jsonInt(i.firstShaker) + ",";
                    s += "\"channels\":" + jsonInt(i.channels) + ",";
                    s += "\"rate\":" + jsonInt(i.sampleRate) + ",";
                    s += "\"bufferMs\":" + jsonDouble(i.bufferMs, 1) + ",";
                    s += "\"underruns\":" + std::to_string(i.underruns) + ",";
                    s += "\"fill\":" + jsonInt(i.fill) + ",";
                    s += "\"levels\":[";
                    for (size_t c = 0; c < i.levels.size(); ++c)
                        s += (c ? "," : "") + jsonDouble(std::fabs(i.levels[c]), 3);
                    s += "]}";
                }
                s += "]";
            }
            else s += "\"enabled\":false,\"open\":false,\"channels\":0,\"devices\":[]";
            s += "},";
        }
        // Sticky per-sim effect dots for the current game, registry order:
        // 0 never delivered, 1 delivered but never produced, 2 has produced.
        {
            const std::string game = m_effectStatus.currentGame();
            const EffectStatus::Records rec = m_effectStatus.records(game);
            s += "\"hapProfile\":" + jsonStr(m_profiles.active()) + ",";
            // The car table's view of the current car: its entry (key,
            // name, where from), whether that entry is what the tiles
            // carry, and a generation that bumps on every apply so a page
            // can refresh its copy of the config.
            {
                std::lock_guard<std::mutex> lk(m_carMx);
                s += "\"hapCar\":" + jsonStr(m_carState.car) + ",";
                s += "\"hapCarId\":" + jsonStr(m_carState.carId) + ",";
                s += "\"hapCarKey\":" + jsonStr(m_carState.match.key) + ",";
                s += "\"hapCarName\":" + jsonStr(m_carState.match.name) + ",";
                s += "\"hapCarSrc\":" + jsonStr(m_carState.match.source) + ",";
                s += std::string("\"hapCarApplied\":") + (m_carState.applied ? "true" : "false") + ",";
            }
            s += "\"hapCarGen\":" + jsonInt(m_carGen.load()) + ",";
            s += "\"hapGame\":" + jsonStr(game) + ",\"hapDots\":[";
            for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
            {
                const EffectRecord& r = rec[static_cast<size_t>(i)];
                s += std::string(i ? "," : "") + (r.produced ? "2" : (r.delivered ? "1" : "0"));
            }
            s += "],";
        }
    }
    {
        int known = 0;
        for (int g = 1; g < MAX_GEARS; ++g) if (ms.gearRatioKnown[g]) ++known;
        s += "\"gearsKnown\":" + jsonInt(known) + ",";
    }
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
                // Device axes: live lever position in home-frame revs
                // (web teach capture). Meaningful once homed.
                if (m_config && i < static_cast<int>(m_config->drives.size()))
                {
                    const std::string& at = m_config->drives[i].axisType;
                    if (at == "shifter" || at == "pedal")
                    {
                        s += ",\"devRev\":" + jsonDouble(ms.devPosRev[i], 4);
                        s += ",\"devMin\":" + jsonDouble(ms.devPosMin[i], 4);
                        s += ",\"devMax\":" + jsonDouble(ms.devPosMax[i], 4);
                    }
                }
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
        // Decoded fault identity (additive): the 603F class from the live
        // PDO word plus the candidate Er codes it covers. The exact sub-code
        // is on the drive panel; nothing here reads the mailbox.
        if (ind.fault && hasSw)
        {
            A6Drive* dp = m_master->getDrive(i);
            uint16_t bus = dp ? dp->getFaultCode() : 0;
            if (bus != 0)
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
                // Mirror the Qt Initialize button: reload the config from DISK
                // (headless has no file watcher - without this a Pi re-init
                // silently applied the boot-time config, so web saves looked
                // dead until a service restart), then re-apply it to the
                // motion controller so a rig.json save made while EtherCAT
                // was up takes effect on THIS init. Guarded on the loop being
                // stopped: configure() reseats axes to parkPos and clears
                // homed/arms rehome, which must never run under a live RT
                // thread.
                if (m_motion && m_config && !(m_loop && m_loop->isRunning()))
                {
                    if (m_configReloader && !m_configReloader())
                        LOG_WARNING("WebServer: config reload from disk failed -- "
                                    "initializing with the in-memory config.");
                    else
                    {
                        // rig.json is now what this process runs: the pill
                        // for it goes out (host.json still needs a restart).
                        m_rigNeedsInit.store(false);
                        if (!m_configPath.empty())
                            m_rigKnownMtime.store(fileMtime(siblingFile(m_configPath, "rig.json")));
                    }
                    m_motion->configure(*m_config);
                }

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

    // Effect status sampler: a few Hz, below-normal priority, independent
    // of any browser. Loads the remembered file first.
    if (!m_configPath.empty()) { m_effectStatus.load(m_configPath); m_profiles.load(m_configPath); }
    // The car table: the shipped file (missing = an empty table, nothing
    // else changes) and the user's layer beside the config.
    if (!m_stockCarsPath.empty() && !m_cars.loadStock(m_stockCarsPath))
        LOG_WARNING(strf("Haptics: car table %s not found or unreadable; the strip will not follow the car.",
                         m_stockCarsPath.c_str()));
    if (!m_configPath.empty()) m_cars.loadUser(m_configPath);
    if (m_cars.stockCount() || m_cars.userCount())
        LOG_INFO(strf("Haptics: car table loaded, %d cars shipped, %d of yours.",
                      m_cars.stockCount(), m_cars.userCount()));
    m_statusThread = std::thread([this]()
    {
#ifdef _WIN32
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
        int ticks = 0;
        while (m_running.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            sampleEffectStatus();
            if (++ticks % 40 == 0 && !m_configPath.empty() && m_effectStatus.dirty())   // every 10 s
                m_effectStatus.save(m_configPath);
        }
    });

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

        // NOTE: no pre-routing handler here - httplib supports exactly ONE
        // (set_pre_routing_handler replaces), and it is registered further
        // down with the Host-header defense + the optional web auth gate.

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

        // ---- REST: provisioning role check ----
        // Reads the drive-RESIDENT role-critical params (profile torqueOnly[],
        // C06.20 runaway protection first) from every configured slave and
        // compares each against what that axis's role expects: the torque
        // value on torque axes, the declared factory default on position
        // axes. Catches state that stayed with the metal through a physical
        // drive swap, replacement, or factory reset. Read-only, PreOp with
        // the loop stopped (same regime as the provision tool) - refused
        // otherwise, so it can never contend with the RT exchange.
        svr.Get("/api/provision/rolecheck", [this](const httplib::Request&, httplib::Response& res)
        {
            const auto fail = [&res](const std::string& msg)
            { res.set_content("{\"ok\":false,\"error\":" + jsonStr(msg) + "}", "application/json"); };

            if (!m_master || !m_config) { fail("controller not available"); return; }
            if (!m_master->isInitialized())
            { fail("Initialize EtherCAT first (loop stopped) - the check reads each drive over the bus."); return; }
            if (m_master->isOperational() || m_master->isRtLoopActive())
            { fail("Stop the control loop first - the check runs with the bus in PreOp only."); return; }

            prov::Profile prof;
            std::string perr;
            if (!prof.load("drive_profiles/nullcat_a6.json", perr))
            { fail("drive profile not found (drive_profiles/nullcat_a6.json): " + perr); return; }
            if (prof.torqueOnly.empty())
            { fail("this drive profile declares no role-critical params"); return; }

            prov::Provisioner pv(*m_master);
            std::string s = "{\"ok\":true,\"rows\":[";
            bool first = true;
            for (size_t i = 0; i < m_config->drives.size(); ++i)
            {
                const DriveConfig& d = m_config->drives[i];
                if (d.slaveIndex < 1) continue;
                const bool torqueAxis = (d.mode == "torque");
                prov::Result rr = pv.roleCheck(d.slaveIndex, prof, torqueAxis);
                const std::string head =
                    "{\"name\":" + jsonStr(d.name) +
                    ",\"slave\":" + std::to_string(d.slaveIndex) +
                    ",\"role\":\"" + (torqueAxis ? "torque" : "position") + "\"";
                if (rr.writes.empty())          // per-slave refusal (e.g. index out of range)
                {
                    if (!first) s += ","; first = false;
                    s += head + ",\"error\":" + jsonStr(rr.message) + "}";
                    continue;
                }
                for (const auto& w : rr.writes)
                {
                    if (!first) s += ","; first = false;
                    s += head +
                         ",\"param\":"    + jsonStr(w.panel) +
                         ",\"coe\":"      + jsonStr(w.coe) +
                         ",\"expected\":" + std::to_string(static_cast<long long>(w.wrote)) +
                         ",\"actual\":"   + std::to_string(static_cast<long long>(w.readback)) +
                         ",\"match\":"    + (w.verified ? "true" : "false") + "}";
                }
            }
            s += "]}";
            res.set_content(s, "application/json");
        });

        // ---- REST: recent log lines ----
        // Returns up to ?n=N lines (default 100, max 500) as a JSON array.
        // One-click support bundle: everything a remote report needs, as a
        // single text file DOWNLOADED to whatever machine runs the browser
        // (no SSH, no scp, no path knowledge). Serves the on-disk logs
        // directly - identical on Pi and PC, no journalctl involved.
        svr.Get("/api/logbundle", [this](const httplib::Request&, httplib::Response& res)
        {
            const auto tailFile = [](const std::string& path, size_t capBytes) -> std::string
            {
                std::ifstream f(path, std::ios::binary | std::ios::ate);
                if (!f) return "(not found: " + path + ")\n";
                const std::streamoff sz = f.tellg();
                const std::streamoff take = std::min<std::streamoff>(sz, (std::streamoff)capBytes);
                f.seekg(sz - take);
                std::string out((size_t)take, '\0');
                f.read(&out[0], take);
                if (take < sz) out.insert(0, "...(older lines truncated)...\n");
                return out;
            };
            const auto wholeFile = [](const std::string& path) -> std::string
            {
                std::ifstream f(path, std::ios::binary);
                if (!f) return "(not found)\n";
                std::stringstream ss; ss << f.rdbuf(); return ss.str();
            };

#ifndef NULLCAT_VERSION
#define NULLCAT_VERSION "dev"
#endif
            std::string s;
            s.reserve(1 << 20);
            {
                char ts[64] = "?";
                std::time_t t = std::time(nullptr);
                std::strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
                s += "==== nullCAT support bundle ====\nversion: " NULLCAT_VERSION "\nplatform: ";
#ifdef _WIN32
                s += "windows";
#else
                s += "linux";
#endif
                s += "\ncaptured: "; s += ts; s += "\n\n";
            }
            s += "==== recent log ring (last 500 lines) ====\n";
            for (const auto& l : Logger::instance().getRecentLogs(500)) { s += l; s += "\n"; }
            if (!m_configPath.empty() && m_config)
            {
                const std::string appLog = siblingFile(m_configPath, m_config->logFile.c_str());
                std::string soemLog = appLog;
                const auto dot = soemLog.find_last_of('.');
                if (dot != std::string::npos) soemLog.insert(dot, "_soem");
                s += "\n==== app.log tail ====\n";
                s += tailFile(appLog, 512 * 1024);
                s += "\n==== soem log tail ====\n";
                s += tailFile(soemLog, 256 * 1024);
                s += "\n==== rig.json ====\n";
                s += wholeFile(siblingFile(m_configPath, "rig.json"));
                s += "\n==== host.json ====\n";
                s += wholeFile(siblingFile(m_configPath, "host.json"));
            }
            res.set_header("Content-Disposition",
                           "attachment; filename=\"nullcat-support-bundle.txt\"");
            res.set_header("Cache-Control", "no-store");
            res.set_content(s, "text/plain; charset=utf-8");
        });

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
            // Pending truth is SERVER-owned so it survives page reloads and
            // every client agrees: a web save that changed something this
            // process has not picked up (see WebServer.h), or a file whose
            // mtime is newer than the last one we wrote/loaded (external edit).
            auto pending = [this](const char* which, bool flag, long long known) -> bool
            {
                if (flag) return true;
                if (m_configPath.empty()) return false;
                const long long mt = fileMtime(siblingFile(m_configPath, which));
                return mt > (known ? known : static_cast<long long>(m_processStart));
            };
            const bool rigPend  = pending("rig.json",  m_rigNeedsInit.load(),     m_rigKnownMtime.load());
            const bool hostPend = pending("host.json", m_hostNeedsRestart.load(), m_hostKnownMtime.load());
#ifndef NULLCAT_VERSION
#define NULLCAT_VERSION "dev"
#endif
            res.set_content(std::string("{\"hostOwner\":\"") + hostOwner()
                            + "\",\"configVersion\":2"
                            + ",\"version\":\"" NULLCAT_VERSION "\""
#ifdef _WIN32
                            + ",\"platform\":\"windows\""
#else
                            + ",\"platform\":\"linux\""
#endif
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
        // namespace on disk, then atomically replace <which>.json. Single
        // writer per file: rig = web (both platforms); host = web only on
        // headless builds (refused there when natively owned). A body that
        // is structurally identical to the file is NOT written (no mtime
        // bump, no pending pill): a bench session lost an afternoon to the
        // pill lighting on every save and every restart being a first init.
        // Returns -1 on failure (response already set), 0 unchanged, 1 written.
        auto writeConfigNamespace = [this, errResp](const char* which,
            const std::vector<std::string>& errs, const std::string& body,
            httplib::Response& res, std::string* oldBodyOut) -> int
        {
            if (!errs.empty())
            {
                std::string m; for (auto& e : errs) m += (m.empty() ? "" : "; ") + e;
                errResp(res, m);
                return -1;
            }
            const std::string path = siblingFile(m_configPath, which);
            std::string oldBody;
            const bool haveOld = readWholeFile(path, oldBody);
            if (oldBodyOut) *oldBodyOut = haveOld ? oldBody : std::string();
            if (haveOld && jsonEqual(oldBody, body))
            {
                LOG_INFO(std::string("WebServer: ") + which + " save received - no changes, not written.");
                return 0;
            }
            const std::string tmp  = path + ".tmp";
            { std::ofstream o(tmp, std::ios::binary | std::ios::trunc);
              if (!o) { errResp(res, "Cannot write temp file."); return -1; } o << body; }
            // std::filesystem::rename is an atomic replace-if-exists on BOTH
            // platforms (MSVC -> MoveFileEx(MOVEFILE_REPLACE_EXISTING), Linux ->
            // rename(2)). Plain std::rename fails with EEXIST on the Windows CRT
            // when the target exists, which would break every save after the
            // first on a Windows build.
            // On Windows the replace fails with a sharing violation while
            // another reader (AV scan, the native app's file watcher) has the
            // target open for a few tens of ms after the previous write; a
            // short retry turns that into a non-event instead of a refused save.
            std::error_code ec;
            for (int attempt = 0; attempt < 8; ++attempt)
            {
                std::filesystem::rename(tmp, path, ec);
                if (!ec) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            if (ec)
            { std::error_code ec2; std::filesystem::remove(tmp, ec2); errResp(res, "Save failed (rename)."); return -1; }
            return 1;
        };

        // POST /api/rig - the web owns rig.json on both platforms.
        postCmd("/api/rig", [this, errResp, writeConfigNamespace](const httplib::Request& req, httplib::Response& res)
        {
            if (m_configPath.empty()) { errResp(res, "No config path configured."); return; }
            std::string oldBody;
            const int wr = writeConfigNamespace("rig.json", Config::validateRigBody(m_configPath, req.body),
                                                req.body, res, &oldBody);
            if (wr < 0) return;
            if (wr == 0)
            {
                res.set_content(std::string("{\"ok\":true,\"changed\":false,\"needsInit\":")
                                + (m_rigNeedsInit.load() ? "true" : "false") + "}", "application/json");
                return;
            }
            // What changed decides the pill: haptics + device feel apply live
            // below; anything else waits for the next Initialize.
            const bool liveOnly = rigDiffIsLiveOnly(oldBody, req.body);
            if (!liveOnly) m_rigNeedsInit.store(true);
            m_rigKnownMtime.store(fileMtime(siblingFile(m_configPath, "rig.json")));
            LOG_INFO(liveOnly
                ? "WebServer: rig.json updated via web - haptics/device tuning only, applied live (no re-initialize needed)."
                : "WebServer: rig.json updated via web - applies on the next Initialize.");
            // Device live-apply: stage each device axis's fresh params with
            // the motion controller - they land the moment that device is
            // (or next becomes) limp. Feel tuning never needs a restart.
            if (m_motion)
            {
                std::vector<DriveConfig> axes;
                if (Config::parseRigBodyAxes(req.body, axes))
                    for (size_t i = 0; i < axes.size() && i < MAX_DRIVES; ++i)
                        if (axisCaps(axes[i].axisType, axes[i].mode).isDevice())
                            m_motion->stageDeviceParams((int)i, axes[i].device);
                // Haptics live-apply: reload the just-written config from
                // disk and stage its haptics tuning - amp/frequency/routing
                // edits take effect immediately, no re-initialize. (A save
                // is rare; the disk round-trip keeps one parser as truth.)
                Config fresh;
                if (fresh.load(m_configPath))
                    m_motion->stageHaptics(fresh.get());
            }
            res.set_content(std::string("{\"ok\":true,\"changed\":true,\"needsInit\":")
                            + (m_rigNeedsInit.load() ? "true" : "false") + "}", "application/json");
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
            const int wr = writeConfigNamespace("host.json", Config::validateHostBody(m_configPath, req.body),
                                                req.body, res, nullptr);
            if (wr < 0) return;
            if (wr == 1)
            {
                m_hostNeedsRestart.store(true);
                m_hostKnownMtime.store(fileMtime(siblingFile(m_configPath, "host.json")));
                LOG_INFO("WebServer: host.json updated via web - applies on the next service/app restart.");
            }
            res.set_content(std::string("{\"ok\":true,\"changed\":") + (wr == 1 ? "true" : "false")
                            + ",\"needsRestart\":" + (m_hostNeedsRestart.load() ? "true" : "false") + "}",
                            "application/json");
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

        // Device engage/release (shifter/pedal families ONLY; every other
        // axis untouched -- the RT side filters on caps.isDevice()). Same
        // idempotent two-endpoint shape as the belt pair, for the same
        // reason: a physical button bound to one of these can never do the
        // opposite because UI state drifted. Optional {"axis": N} targets
        // one device (1-based, chain order); default all device axes.
        const auto deviceCmd = [this, okResp, errResp](
            MotionCommand::Type type, const httplib::Request& req, httplib::Response& res)
        {
            if (!m_motion) { errResp(res, "Motion controller not ready."); return; }
            MotionCommand cmd;
            cmd.type   = type;
            cmd.intVal = -1;
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
        };
        postCmd("/api/device/engage", [deviceCmd](const httplib::Request& req, httplib::Response& res)
        { deviceCmd(MotionCommand::Type::EngageDevice, req, res); });
        postCmd("/api/device/release", [deviceCmd](const httplib::Request& req, httplib::Response& res)
        { deviceCmd(MotionCommand::Type::ReleaseDevice, req, res); });

        // Haptics Test button: fires the detent click (effect=detentClick) or
        // previews one continuous effect at full level for ~2 s. Enqueued so
        // the layer is only ever touched from the RT thread.
        postCmd("/api/haptics/test", [this, okResp, errResp](const httplib::Request& req, httplib::Response& res)
        {
            if (!m_motion) { errResp(res, "Motion controller not ready."); return; }
            // Body: {"effect": "<registry key>"}.
            const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(req.body));
            const std::string key = doc.isObject() ? doc.object().value("effect").toString().toStdString() : std::string();
            const haptics::EffectInfo* info = haptics::findEffect(key.c_str());
            if (!info) { errResp(res, "Unknown effect."); return; }

            // The effect as SAVED has to be able to produce something, and
            // one of the axes it routes to has to be live, or the button is
            // a mystery (a bench session read a full wave on an effect at
            // amp 0 / no route as "fires visually, nothing felt"). Disk is
            // truth: unsaved edits in the browser are not what would play.
            Config saved;
            if (!saved.load(m_configPath)) { errResp(res, "Cannot read the saved config."); return; }
            const AppConfig& sc = saved.get();
            const haptics::EffectParams& ep = sc.hapticsFx[static_cast<size_t>(info->id)];
            if (ep.ampPct <= 0.0)
            { errResp(res, "Effect amplitude is 0 in the saved config: set amp % and Save first."); return; }
            if (!haptics::Layer::hasRoute(ep))
            { errResp(res, "Effect has no route in the saved config: open its route chip, set a gain, then Save."); return; }
            if (sc.hapticsMasterGain <= 0.0)
            { errResp(res, "Master gain is 0: nothing can be felt."); return; }

            const MotionStatus ms = m_motion->getMotionStatus();
            bool routedLive = false;
            std::string routedNames;
            for (const haptics::Route& r : ep.routes)
            {
                if (r.axis < 0 || r.gain <= 0.0) continue;
                if (static_cast<size_t>(r.axis) < sc.drives.size())
                    routedNames += (routedNames.empty() ? "" : ", ") + sc.drives[r.axis].name;
                // numDrives is 0 until the loop runs: nothing is live then.
                if (r.axis < ms.numDrives
                    && (ms.axisState[r.axis] == AxisMotionState::ONLINE
                        || ms.axisState[r.axis] == AxisMotionState::BLENDING))
                    routedLive = true;
            }
            if (!routedLive)
            { errResp(res, "Routed axis not live (" + routedNames + "): start the loop, then engage the device or tension the belt."); return; }

            MotionCommand cmd; cmd.type = MotionCommand::Type::HapticsTest; cmd.intVal = static_cast<int>(info->id);
            if (!m_motion->enqueueCommand(cmd)) { errResp(res, "Command queue full."); return; }
            okResp(res);
        });

        // GET /api/haptics/schema - the effect table (HapticsRegistry.h) as
        // JSON: key, label, kind, channels, the tunables to show with their
        // ranges, and the defaults. The browser builds its tiles from this,
        // so it carries no copy of the effect list.
        svr.Get("/api/haptics/schema", [](const httplib::Request&, httplib::Response& res)
        {
            QJsonArray effects;
            for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
            {
                const haptics::EffectInfo& info = haptics::effectInfo(i);
                QJsonObject e;
                e["key"]   = info.key;
                e["label"] = info.label;
                e["kind"]  = (info.kind == haptics::Kind::Transient) ? "transient"
                           : (info.kind == haptics::Kind::Engine)    ? "engine"
                           : (info.kind == haptics::Kind::Slip)      ? "slip"
                           : (info.kind == haptics::Kind::Road)      ? "road"
                           : (info.kind == haptics::Kind::Kerb)      ? "kerb"
                           : (info.kind == haptics::Kind::Abs)       ? "abs"
                           : (info.kind == haptics::Kind::Tc)        ? "tc"
                           : (info.kind == haptics::Kind::Surface)   ? "surface"
                           : (info.kind == haptics::Kind::Wheels)    ? "wheels"
                           : (info.kind == haptics::Kind::Impacts)   ? "impacts"
                           : (info.kind == haptics::Kind::Driveline) ? "driveline" : "continuous";
                if (info.kind != haptics::Kind::Transient) e["fxIdx"] = static_cast<int>(info.fx);
                // Per-wheel effects: routes carry a part (route editor shows the selector).
                e["parts"] = haptics::kindHasParts(info.kind);
                QJsonArray ch;
                for (const char* c : info.channels) { if (!c) break; ch.append(c); }
                e["channels"] = ch;
                QJsonArray params;
                for (int k = 0; k < info.paramCount; ++k)
                {
                    const haptics::ParamSpec& ps = info.params[k];
                    QJsonObject p;
                    p["key"] = ps.key; p["label"] = ps.label;
                    p["min"] = ps.min; p["max"] = ps.max; p["step"] = ps.step;
                    if (ps.opts)
                    {
                        QJsonArray opts;
                        for (const QString& o : QString(ps.opts).split('|')) opts.append(o);
                        p["opts"] = opts;
                    }
                    params.append(p);
                }
                e["params"] = params;
                QJsonObject d;
                d["ampPct"] = info.defaults.ampPct; d["freqHz"] = info.defaults.freqHz;
                d["durMs"]  = info.defaults.durMs;  d["jitter"] = info.defaults.jitter;
                if (info.kind == haptics::Kind::Engine)
                {
                    const haptics::EngineParams ed;
                    d["cylinders"] = ed.cylinders; d["litres"] = ed.litres; d["layout"] = ed.layout;
                    d["maxRpm"] = ed.maxRpm; d["rock"] = ed.rock; d["thump"] = ed.thump; d["buzz"] = ed.buzz;
                    d["order"] = ed.order; d["limHit"] = ed.limHit; d["limHz"] = ed.limHz; d["limJit"] = ed.limJit;
                    d["inertia"] = ed.inertia; d["turbo"] = ed.turbo; d["liftoff"] = ed.liftoff; d["pops"] = ed.pops;
                }
                else if (info.kind == haptics::Kind::Slip)
                {
                    for (int k = 0; k < haptics::SLIP_KEY_COUNT; ++k)
                        if (info.slipKeys[k]) d[info.slipKeys[k]] = haptics::slipField(info.slipDefaults, k);
                }
                else if (info.kind == haptics::Kind::Road)
                {
                    const haptics::RoadParams rd;
                    d["fullMm"] = rd.fullMm; d["hpHz"] = rd.hpHz;
                    d["surface"] = rd.surface; d["surfaceKmh"] = rd.surfaceKmh; d["surfaceHz"] = rd.surfaceHz;
                    d["model"] = rd.model; d["bodyMm"] = rd.bodyMm; d["bodyHz"] = rd.bodyHz;
                    d["hopHz"] = rd.hopHz; d["damping"] = rd.damping; d["rough"] = rd.rough;
                }
                else if (info.kind == haptics::Kind::Kerb)
                {
                    const haptics::KerbParams kd;
                    d["pitchCm"] = kd.pitchCm; d["riseMm"] = kd.riseMm; d["ribMm"] = kd.ribMm;
                    d["fullMm"] = kd.fullMm; d["detectMm"] = kd.detectMm;
                }
                else if (info.kind == haptics::Kind::Surface)
                {
                    const haptics::SurfaceParams sd;
                    d["stones"] = sd.stones; d["crunch"] = sd.crunch; d["studs"] = sd.studs;
                    d["puddles"] = sd.puddles; d["aquaKmh"] = sd.aquaKmh; d["smooth"] = sd.smooth;
                }
                else if (info.kind == haptics::Kind::Wheels)
                {
                    const haptics::WheelsParams wd;
                    d["balance"] = wd.balance; d["flat"] = wd.flat; d["judder"] = wd.judder;
                }
                else if (info.kind == haptics::Kind::Impacts)
                {
                    const haptics::ImpactParams md;
                    d["fromMmS"] = md.fromMmS; d["fullMmS"] = md.fullMmS; d["heaveG"] = md.heaveG;
                }
                else if (info.kind == haptics::Kind::Abs || info.kind == haptics::Kind::Tc)
                {
                    const AppConfig dc;   // the tiles' defaults (TC has no slow or buzz)
                    const haptics::PulseParams& q = (info.kind == haptics::Kind::Abs) ? dc.hapticsAbs : dc.hapticsTc;
                    d["sharp"] = q.sharp; d["spread"] = q.spread;
                    if (info.kind == haptics::Kind::Abs)
                    {
                        d["slow"] = q.slow; d["buzz"] = q.buzz; d["buzzHz"] = q.buzzHz;
                        d["slipLink"] = q.slipLink; d["lockLink"] = q.lockLink; d["scrubLink"] = q.scrubLink;
                    }
                }
                else if (info.kind == haptics::Kind::Driveline)
                {
                    const haptics::DrivelineParams dd;
                    d["clutch"] = dd.clutch; d["clutchHz"] = dd.clutchHz; d["lug"] = dd.lug; d["lugHz"] = dd.lugHz;
                    d["gearbox"] = dd.gearbox; d["whine"] = dd.whine; d["shunt"] = dd.shunt; d["shuntHz"] = dd.shuntHz;
                }
                e["defaults"] = d;
                e["tip"] = info.tip;
                effects.append(e);
            }
            // The sim channel token registry (NcxTokens.h): the bindings
            // editor's list, the status ncxHave[] order, and what the chip
            // syntax ("slipAngle*", "a|b", "~optional") expands against.
            QJsonArray tokens;
            for (int t = 0; t < NcxTok::TokenCount; ++t) tokens.append(ncxTokenName(t));
            QJsonArray parts;
            for (int i = 0; i < haptics::PART_COUNT; ++i) parts.append(haptics::partKey(static_cast<haptics::Part>(i)));
            QJsonObject root;
            root["effects"] = effects; root["masterGainDefault"] = 1.0;
            root["tokens"] = tokens; root["maxSlots"] = MAX_NCX_CHANNELS; root["parts"] = parts;
            res.set_content(QJsonDocument(root).toJson(QJsonDocument::Compact).toStdString(), "application/json");
        });

        // GET /api/haptics/status?game=<name> - the sticky per-sim record:
        // every game seen (most recent first), the one asked for (default
        // the current one) with per-effect delivered / produced / lastSeen.
        svr.Get("/api/haptics/status", [this](const httplib::Request& req, httplib::Response& res)
        {
            const std::string game = req.has_param("game") ? req.get_param_value("game") : m_effectStatus.currentGame();
            const EffectStatus::Records rec = m_effectStatus.records(game);
            QJsonArray games;
            for (const std::string& g : m_effectStatus.games()) games.append(QString::fromStdString(g));
            QJsonObject effects;
            for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
            {
                const EffectRecord& r = rec[static_cast<size_t>(i)];
                QJsonObject e;
                e["delivered"] = r.delivered; e["produced"] = r.produced;
                e["lastSeen"]  = static_cast<double>(r.lastSeenMs);
                effects[haptics::effectInfo(i).key] = e;
            }
            QJsonObject root;
            root["games"] = games; root["game"] = QString::fromStdString(game);
            root["current"] = QString::fromStdString(m_effectStatus.currentGame());
            root["effects"] = effects;
            res.set_content(QJsonDocument(root).toJson(QJsonDocument::Compact).toStdString(), "application/json");
        });
        // POST /api/haptics/status/clear - body {"game":"<name>"} forgets one
        // game's record; {"all":true} forgets everything.
        postCmd("/api/haptics/status/clear", [this, okResp](const httplib::Request& req, httplib::Response& res)
        {
            const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(req.body));
            const QJsonObject o = doc.isObject() ? doc.object() : QJsonObject();
            if (o.value("all").toBool(false)) m_effectStatus.clearAll();
            else m_effectStatus.clear(o.value("game").toString().toStdString());
            if (!m_configPath.empty()) m_effectStatus.save(m_configPath);
            okResp(res);
        });

        // Haptics profiles. GET lists them with the active one, the bindings
        // and what the stream currently names. Save snapshots the SAVED set
        // under a name; load copies a profile over the live set (applied
        // live); delete forgets one; bind ties the current car (or a given
        // car/game) to a profile; unbind drops a key.
        svr.Get("/api/haptics/profiles", [this](const httplib::Request&, httplib::Response& res)
        {
            QJsonArray names;
            for (const std::string& n : m_profiles.names()) names.append(QString::fromStdString(n));
            QJsonObject binds;
            for (const auto& kv : m_profiles.bindings()) binds[QString::fromStdString(kv.first)] = QString::fromStdString(kv.second);
            QJsonObject root;
            root["profiles"] = names;
            root["active"]   = QString::fromStdString(m_profiles.active());
            root["bindings"] = binds;
            std::string car, game;
            if (m_telemetry) { const TelemetryData td = m_telemetry->getLatestData(); if (td.ncxFresh || td.ncxFrozen) { car = td.car; game = td.game; } }
            root["car"] = QString::fromStdString(car); root["game"] = QString::fromStdString(game);
            res.set_content(QJsonDocument(root).toJson(QJsonDocument::Compact).toStdString(), "application/json");
        });
        const auto profileName = [](const httplib::Request& req) -> std::string
        {
            const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(req.body));
            std::string n = doc.isObject() ? doc.object().value("name").toString().trimmed().toStdString() : std::string();
            if (n.size() > 48) n.resize(48);
            return n;
        };
        postCmd("/api/haptics/profiles/save", [this, okResp, errResp, profileName](const httplib::Request& req, httplib::Response& res)
        {
            const std::string name = profileName(req);
            if (name.empty()) { errResp(res, "Give the profile a name."); return; }
            std::string err;
            if (!snapshotProfile(name, err)) { errResp(res, "Could not save the profile: " + err); return; }
            okResp(res);
        });
        postCmd("/api/haptics/profiles/load", [this, okResp, errResp, profileName](const httplib::Request& req, httplib::Response& res)
        {
            const std::string name = profileName(req);
            std::string err;
            if (!loadProfile(name, err)) { errResp(res, "Could not load the profile: " + err); return; }
            LOG_INFO(strf("Haptics: profile '%s' loaded from the web.", name.c_str()));
            okResp(res);
        });
        postCmd("/api/haptics/profiles/delete", [this, okResp, errResp, profileName](const httplib::Request& req, httplib::Response& res)
        {
            const std::string name = profileName(req);
            if (!m_profiles.remove(name)) { errResp(res, "Unknown profile."); return; }
            if (!m_configPath.empty()) m_profiles.save(m_configPath);
            okResp(res);
        });
        postCmd("/api/haptics/profiles/bind", [this, okResp, errResp, profileName](const httplib::Request& req, httplib::Response& res)
        {
            // Body: {"name": profile, "car": "<name>"} or {"name", "game": "<name>"};
            // with neither, the car the stream currently names (else its game).
            const std::string name = profileName(req);
            if (!m_profiles.has(name)) { errResp(res, "Unknown profile."); return; }
            const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(req.body));
            const QJsonObject o = doc.isObject() ? doc.object() : QJsonObject();
            std::string key;
            if (o.contains("car") && !o.value("car").toString().isEmpty())       key = HapticsProfiles::carKey(o.value("car").toString().toStdString());
            else if (o.contains("game") && !o.value("game").toString().isEmpty()) key = HapticsProfiles::gameKey(o.value("game").toString().toStdString());
            else if (m_telemetry)
            {
                const TelemetryData td = m_telemetry->getLatestData();
                if (td.car[0])       key = HapticsProfiles::carKey(td.car);
                else if (td.game[0]) key = HapticsProfiles::gameKey(td.game);
            }
            if (key.empty()) { errResp(res, "The sim has not named a car or game yet: nothing to bind to."); return; }
            m_profiles.bind(key, name);
            if (!m_configPath.empty()) m_profiles.save(m_configPath);
            res.set_content("{\"ok\":true,\"key\":" + jsonStr(key) + "}", "application/json");
        });
        postCmd("/api/haptics/profiles/unbind", [this, okResp, errResp](const httplib::Request& req, httplib::Response& res)
        {
            const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(req.body));
            const std::string key = doc.isObject() ? doc.object().value("key").toString().toStdString() : std::string();
            if (key.empty()) { errResp(res, "Which binding?"); return; }
            m_profiles.unbind(key);
            if (!m_configPath.empty()) m_profiles.save(m_configPath);
            okResp(res);
        });

        // ---- Car table: the current car's entry and the whole list; pull
        // (apply an entry: the current car's, or any by key, as a starting
        // point for a car the table lacks); save the strip's car-owned
        // values under the current car; forget that user entry.
        svr.Get("/api/cars", [this](const httplib::Request&, httplib::Response& res)
        {
            QJsonObject root;
            {
                std::lock_guard<std::mutex> lk(m_carMx);
                root["car"]     = QString::fromStdString(m_carState.car);
                root["carId"]   = QString::fromStdString(m_carState.carId);
                root["game"]    = QString::fromStdString(m_carState.game);
                root["key"]     = QString::fromStdString(m_carState.match.key);
                root["name"]    = QString::fromStdString(m_carState.match.name);
                root["source"]  = QString::fromStdString(m_carState.match.source);
                root["notes"]   = QString::fromStdString(m_carState.match.notes);
                root["applied"] = m_carState.applied;
            }
            root["follow"] = followCarEnabled();
            root["stock"]  = m_cars.stockCount();
            root["user"]   = m_cars.userCount();
            root["gen"]    = m_carGen.load();
            res.set_content(QJsonDocument(root).toJson(QJsonDocument::Compact).toStdString(), "application/json");
        });
        svr.Get("/api/cars/list", [this](const httplib::Request&, httplib::Response& res)
        {
            QJsonArray arr;
            for (const CarTable::Item& it : m_cars.list())
            {
                QJsonObject o;
                o["key"] = QString::fromStdString(it.key);  o["name"]   = QString::fromStdString(it.name);
                o["game"] = QString::fromStdString(it.game); o["source"] = QString::fromStdString(it.source);
                arr.append(o);
            }
            QJsonObject root; root["cars"] = arr;
            res.set_content(QJsonDocument(root).toJson(QJsonDocument::Compact).toStdString(), "application/json");
        });
        postCmd("/api/cars/pull", [this, errResp](const httplib::Request& req, httplib::Response& res)
        {
            // Body: {"key": "<entry>"} applies that entry; {} applies the
            // current car's own entry.
            const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(req.body));
            const std::string key = doc.isObject() ? doc.object().value("key").toString().toStdString() : std::string();
            CarTable::Match m;
            if (!key.empty())
            {
                m = m_cars.findKey(key);
                if (m.source.empty()) { errResp(res, "No such entry in the car table."); return; }
            }
            else
            {
                std::string car;
                { std::lock_guard<std::mutex> lk(m_carMx); m = m_carState.match; car = m_carState.car; }
                if (car.empty()) { errResp(res, "The sim has not named a car yet."); return; }
                if (m.source.empty()) { errResp(res, "No entry for " + car + " in the car table: pick one from the list as a starting point."); return; }
            }
            std::string err;
            if (!applyCarEntry(m, err)) { errResp(res, "Could not apply the car preset: " + err); return; }
            {
                std::lock_guard<std::mutex> lk(m_carMx);
                // Applied by hand: the tiles carry this entry now, which is
                // the car's own only when it was the car's own.
                m_carState.applied = key.empty() || (m.key == m_carState.match.key);
            }
            LOG_INFO(strf("Haptics: car preset '%s' (%s) applied from the web.", m.key.c_str(), m.source.c_str()));
            res.set_content("{\"ok\":true,\"key\":" + jsonStr(m.key) + ",\"name\":" + jsonStr(m.name)
                            + ",\"source\":" + jsonStr(m.source) + "}", "application/json");
        });
        postCmd("/api/cars/save", [this, errResp](const httplib::Request& req, httplib::Response& res)
        {
            // The SAVED strip's car-owned values under the current car's
            // key, in the user layer. Name and notes: the body's, else the
            // existing entry's (user, then stock), else the car's name.
            if (m_configPath.empty()) { errResp(res, "No config path configured."); return; }
            std::string car, carId, game;
            CarTable::Match cur;
            { std::lock_guard<std::mutex> lk(m_carMx); car = m_carState.car; carId = m_carState.carId; game = m_carState.game; cur = m_carState.match; }
            if (car.empty()) { errResp(res, "The sim has not named a car yet: nothing to save under."); return; }
            const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(req.body));
            const QJsonObject body = doc.isObject() ? doc.object() : QJsonObject();
            std::string name  = body.value("name").toString().trimmed().toStdString();
            std::string notes = body.contains("notes") ? body.value("notes").toString().toStdString() : cur.notes;
            if (name.empty()) name = cur.name.empty() ? car : cur.name;
            if (name.size() > 64) name.resize(64);
            if (notes.size() > 4000) notes.resize(4000);
            Config cfg;
            if (!cfg.load(m_configPath)) { errResp(res, "Cannot read the config."); return; }
            const std::string key = carKeyFor(game, car, carId);
            m_cars.putUser(key, CarTable::fromHapticsObject(Config::writeHapticsObject(cfg.get()), name, notes));
            if (!m_cars.saveUser(m_configPath)) { errResp(res, "Cannot write cars.local.json."); return; }
            {
                std::lock_guard<std::mutex> lk(m_carMx);
                m_carState.match   = findCar(game, car, carId);
                m_carState.applied = true;
            }
            LOG_INFO(strf("Haptics: car preset '%s' saved to your layer.", key.c_str()));
            res.set_content("{\"ok\":true,\"key\":" + jsonStr(key) + "}", "application/json");
        });
        postCmd("/api/cars/forget", [this, errResp, okResp](const httplib::Request& req, httplib::Response& res)
        {
            // Body: {"key": "<entry>"}, else the current car's user entry.
            if (m_configPath.empty()) { errResp(res, "No config path configured."); return; }
            const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(req.body));
            std::string key = doc.isObject() ? doc.object().value("key").toString().toStdString() : std::string();
            std::string car, carId, game;
            { std::lock_guard<std::mutex> lk(m_carMx); car = m_carState.car; carId = m_carState.carId; game = m_carState.game; }
            if (key.empty())
            {
                if (car.empty()) { errResp(res, "The sim has not named a car yet."); return; }
                // Yours for this car: filed under the id, or under the
                // name by an older plugin.
                key = carKeyFor(game, car, carId);
                if (!carId.empty() && m_cars.findKey(key).source != "user") key = m_cars.keyFor(game, car);
            }
            if (!m_cars.removeUser(key)) { errResp(res, "No entry of yours for " + key + "."); return; }
            m_cars.saveUser(m_configPath);
            {
                std::lock_guard<std::mutex> lk(m_carMx);
                if (!car.empty()) { m_carState.match = findCar(game, car, carId); m_carState.applied = false; }
            }
            okResp(res);
        });

        // Shakers: the playback devices the backend can see (for the host
        // settings picker) and a per-channel tone test (40 Hz for a second,
        // through the whole chain, with or without the control loop).
        svr.Get("/api/shakers/devices", [this](const httplib::Request&, httplib::Response& res)
        {
            QJsonArray devs;
            if (m_shakers)
                for (const haptics::ShakerDeviceInfo& d : m_shakers->listDevices())
                {
                    QJsonObject o; o["name"] = QString::fromStdString(d.name); o["default"] = d.isDefault;
                    devs.append(o);
                }
            QJsonObject root; root["devices"] = devs;
            res.set_content(QJsonDocument(root).toJson(QJsonDocument::Compact).toStdString(), "application/json");
        });
        postCmd("/api/shakers/test", [this, okResp, errResp](const httplib::Request& req, httplib::Response& res)
        {
            if (!m_motion) { errResp(res, "Motion controller not ready."); return; }
            if (!m_shakers || !m_shakers->anyOpen())
            { errResp(res, "No shaker output is open: enable shakers in the host settings and restart."); return; }
            const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(req.body));
            const int ch = doc.isObject() ? doc.object().value("channel").toInt(-1) : -1;
            if (ch < 0 || ch >= m_shakers->totalChannels()) { errResp(res, "Channel out of range."); return; }
            if (!m_shakers->channelLive(ch)) { errResp(res, "That channel's device is not open."); return; }
            m_motion->requestShakerTone(ch);
            okResp(res);
        });

        // Haptics master mute (runtime only, not persisted): body {"on":true|false}.
        postCmd("/api/haptics/mute", [this, okResp, errResp](const httplib::Request& req, httplib::Response& res)
        {
            if (!m_motion) { errResp(res, "Motion controller not ready."); return; }
            MotionCommand cmd; cmd.type = MotionCommand::Type::HapticsMute;
            cmd.intVal = (req.body.find("true") != std::string::npos) ? 1 : 0;
            if (!m_motion->enqueueCommand(cmd)) { errResp(res, "Command queue full."); return; }
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
                           "cycle needs front/rear + left/right roles)."); return; }
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
            bool beltMask[MAX_DRIVES] = {};
            const int nCfg = beltAxisMask(beltMask);
            const status::BeltAggregates agg =
                status::deriveBeltAggregates(ms.axisState, ms.numDrives, beltMask, nCfg);
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

        // The three-state device button: unhomed devices home (the press IS
        // the deliberate authorization - devices never home with the rig),
        // homed+limp engage, any engaged releases. One resolution for web,
        // HID binds, and the GPIO panel via the shared StatusModel derivation.
        postCmd("/api/device-toggle", [this, errResp, toggleReady, resolvedResp](const httplib::Request&, httplib::Response& res)
        {
            if (!m_motion) { errResp(res, "Motion controller not ready."); return; }
            static std::atomic<int64_t> last{0};
            if (!toggleReady(last)) { errResp(res, "Toggle cooldown."); return; }
            MotionStatus ms = m_motion->getMotionStatus();
            bool devMask[MAX_DRIVES] = {};
            const int nCfg = deviceAxisMask(devMask);
            const status::DeviceAggregates agg =
                status::deriveDeviceAggregates(ms.axisState, ms.numDrives, ms.homed, devMask, nCfg);
            if (!agg.hasDevices)   { errResp(res, "No device axes on this rig."); return; }
            if (agg.transitional)  { errResp(res, "Device transitioning -- toggle ignored."); return; }
            if (!agg.anyEngaged && m_motion->isEmergencyStop())
            { errResp(res, "E-stop active -- device engage refused."); return; }
            MotionCommand c;
            c.type   = agg.anyEngaged ? MotionCommand::Type::ReleaseDevice
                                      : MotionCommand::Type::EngageDevice;
            c.intVal = -1;
            m_motion->enqueueCommand(c);
            resolvedResp(res, agg.anyEngaged ? "device/release"
                             : agg.allHomed  ? "device/engage" : "device/home");
        });

        // ---- User device presets: hand-crafted feels saved by name ----
        // devicepresets.json beside the configs. Authored state like the
        // car cache: never merged into rig.json, ferried by the installer
        // and updater, survives everything. Applying one is a web-side
        // act (it fills the axis's device object; Save persists).
        svr.Get("/api/devpresets", [this](const httplib::Request&, httplib::Response& res)
        {
            const std::string path = siblingFile(m_configPath, "devicepresets.json");
            std::ifstream f(path, std::ios::binary);
            if (!f) { res.set_content("{\"presets\":{}}", "application/json"); return; }
            std::stringstream ss; ss << f.rdbuf();
            res.set_content(ss.str(), "application/json");
        });
        postCmd("/api/devpresets", [this, errResp](const httplib::Request& req, httplib::Response& res)
        {
            if (m_configPath.empty()) { errResp(res, "No config path configured."); return; }
            QJsonParseError pe;
            const QJsonDocument doc = QJsonDocument::fromJson(
                QByteArray(req.body.c_str(), (int)req.body.size()), &pe);
            if (pe.error != QJsonParseError::NoError || !doc.isObject())
            { errResp(res, "Invalid JSON."); return; }
            const QString name = doc.object().value("name").toString().trimmed();
            const QJsonValue dev = doc.object().value("device");
            const bool remove = doc.object().value("remove").toBool(false);
            if (name.isEmpty() || name.size() > 64)
            { errResp(res, "Preset name must be 1..64 characters."); return; }
            if (!remove && !dev.isObject())
            { errResp(res, "Missing device object."); return; }

            const std::string path = siblingFile(m_configPath, "devicepresets.json");
            QJsonObject root, presets;
            {
                std::ifstream f(path, std::ios::binary);
                if (f) { std::stringstream ss; ss << f.rdbuf();
                    const QJsonDocument d0 = QJsonDocument::fromJson(
                        QByteArray(ss.str().c_str(), (int)ss.str().size()));
                    if (d0.isObject()) root = d0.object(); }
            }
            presets = root.value("presets").toObject();
            if (remove) presets.remove(name); else presets[name] = dev.toObject();
            root["presets"] = presets;
            const QByteArray out = QJsonDocument(root).toJson(QJsonDocument::Indented);
            const std::string tmp = path + ".tmp";
            { std::ofstream o(tmp, std::ios::binary | std::ios::trunc);
              if (!o) { errResp(res, "Cannot write temp file."); return; }
              o.write(out.constData(), out.size()); }
            std::error_code ec;
            std::filesystem::rename(tmp, path, ec);
            if (ec) { std::error_code ec2; std::filesystem::remove(tmp, ec2);
                      errResp(res, "Save failed (rename)."); return; }
            res.set_content("{\"ok\":true}", "application/json");
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
            // The haptic tile peaks go with the drive peaks; asked for
            // directly as well so the idle clock (loop stopped) honours it.
            if (m_motion) m_motion->requestHapticPeakReset();
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

        // Pi click-updater. Never bindable; private clients only; refused
        // outright while the bus or loop is up (park, stop, de-init first).
        // The BROWSER does the is-there-a-newer-release check itself
        // (GitHub's API is CORS-friendly); this endpoint only launches the
        // systemd unit that performs the update. Progress is observed the
        // blunt honest way: the service restarts under the updater and
        // /api/meta answers with the new version when it is done.
        postCmd("/api/update/start", [this, privateClientOnly, errResp](const httplib::Request& req, httplib::Response& res)
        {
            if (!privateClientOnly(req, res)) return;
            if (!m_onUpdateStart)
            { errResp(res, "Updates are not supported on this build (Windows updates ship as a release zip)."); return; }
            if (m_master && m_master->isOperational())
            { errResp(res, "Stop EtherCAT before updating."); return; }
            if (m_loop && m_loop->isRunning())
            { errResp(res, "Stop the control loop before updating."); return; }
            QJsonParseError pe;
            const QJsonDocument doc = QJsonDocument::fromJson(
                QByteArray(req.body.c_str(), (int)req.body.size()), &pe);
            const std::string version =
                (pe.error == QJsonParseError::NoError && doc.isObject())
                ? doc.object().value("version").toString().toStdString() : "";
            static const std::regex kVer("^[0-9]+\\.[0-9]+\\.[0-9]+$");
            if (!std::regex_match(version, kVer))
            { errResp(res, "Bad or missing version."); return; }
            const std::string err = m_onUpdateStart(version);
            if (!err.empty()) { errResp(res, err.c_str()); return; }
            res.set_content("{\"ok\":true}", "application/json");
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

        // THE pre-routing handler. httplib's set_pre_routing_handler REPLACES
        // (last registration wins, no chaining), so every pre-route check
        // must live in this one lambda - a second registration elsewhere
        // silently disables the first (the auth gate shipped dead that way
        // once; TestHttpContract now pins both checks).
        //
        // 1. Host-header allowlist, fail-closed BEFORE any route (including
        //    the static mount): the DNS-rebinding defense. A rebinding page's
        //    request carries its own hostname in Host, which never matches
        //    this machine.
        // 2. Optional web auth (host.json webAuthToken; empty = off): every
        //    /api/* request must carry the token; the static page stays
        //    reachable so the password prompt can render. Constant-time
        //    compare; boot-time config like the other host settings.
        svr.set_pre_routing_handler(
            [this](const httplib::Request& req, httplib::Response& res) -> httplib::Server::HandlerResponse
            {
                if (!hostAllowed(req.get_header_value("Host")))
                {
                    res.status = 421;   // Misdirected Request
                    res.set_content("{\"ok\":false,\"error\":\"Host not allowed\"}",
                                    "application/json");
                    return httplib::Server::HandlerResponse::Handled;
                }

                if (m_config && !m_config->webAuthToken.empty()
                    && req.path.rfind("/api/", 0) == 0)
                {
                    const std::string& want = m_config->webAuthToken;
                    const std::string  got  = req.get_header_value("X-Nullcat-Auth");
                    unsigned char acc = (got.size() == want.size()) ? 0 : 1;
                    for (size_t i = 0; i < got.size() && i < want.size(); ++i)
                        acc |= static_cast<unsigned char>(got[i] ^ want[i]);
                    if (acc != 0)
                    {
                        res.status = 401;
                        res.set_content("{\"ok\":false,\"error\":\"auth required\"}",
                                        "application/json");
                        return httplib::Server::HandlerResponse::Handled;
                    }
                }

                return httplib::Server::HandlerResponse::Unhandled;
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

// One sample of the published status into the sticky per-sim record, and
// the profile auto-switch: when the stream names a car or game with a
// binding, that profile is loaded (once per identity change).
void WebServer::sampleEffectStatus()
{
    if (!m_motion) return;
    const MotionStatus ms = m_motion->getMotionStatus();
    bool live = false;
    std::string game, car, carId;
    if (m_telemetry)
    {
        const TelemetryData td = m_telemetry->getLatestData();
        live  = td.ncxFresh;
        game  = td.game;
        car   = td.car;
        carId = td.carId;
    }
    const int64_t nowMs = static_cast<int64_t>(std::time(nullptr)) * 1000;
    m_effectStatus.observe(game, ms.ncxHave, ms.hapticsFxLevel, ms.hapticsFiredBy, live, nowMs);

    if (live && (!car.empty() || !game.empty()))
    {
        const std::string ident = car + "|" + carId + "|" + game;
        if (ident != m_lastIdentity)
        {
            m_lastIdentity = ident;
            LOG_INFO(strf("Haptics: the sim names car '%s'%s%s%s in game '%s'.", car.c_str(),
                          carId.empty() ? "" : " (id '", carId.c_str(), carId.empty() ? "" : "')", game.c_str()));
            const std::string want = m_profiles.profileFor(car, game);
            if (!want.empty() && want != m_profiles.active())
            {
                std::string err;
                if (loadProfile(want, err))
                    LOG_INFO(strf("Haptics: profile '%s' loaded for %s%s%s.", want.c_str(),
                                  car.empty() ? "" : car.c_str(), (!car.empty() && !game.empty()) ? " / " : "",
                                  game.empty() ? "" : game.c_str()));
                else
                    LOG_WARNING(strf("Haptics: profile '%s' could not be loaded: %s", want.c_str(), err.c_str()));
            }
            // Then the car table: a profile bound to THIS CAR is the user's
            // word on it and wins; a game-bound profile set the rig up and
            // the car's entry goes on top of it.
            const bool carBound = !car.empty() && m_profiles.bindings().count(HapticsProfiles::carKey(car)) > 0;
            carIdentity(game, car, carId, !carBound && followCarEnabled());
        }
    }
    else if (!live)
        m_lastIdentity.clear();   // a returning stream looks its profile up again
}

// The table entry for what the sim names: by the stable id when the
// plugin sends one, else by the shown name (an older plugin, or a game
// whose id is its name).
CarTable::Match WebServer::findCar(const std::string& game, const std::string& car, const std::string& carId) const
{
    if (!carId.empty())
    {
        CarTable::Match m = m_cars.find(game, carId);
        if (!m.source.empty()) return m;
    }
    return m_cars.find(game, car);
}

// Where an entry saved for this car is filed: under the id when there is
// one, so it matches the shipped keys and survives a display-name change.
std::string WebServer::carKeyFor(const std::string& game, const std::string& car, const std::string& carId) const
{
    return m_cars.keyFor(game, carId.empty() ? car : carId);
}

// A new car (or game) from the stream: look its entry up for the strip to
// show, and when the strip follows the car, apply it.
void WebServer::carIdentity(const std::string& game, const std::string& car, const std::string& carId, bool mayApply)
{
    CarTable::Match m = findCar(game, car, carId);
    const std::string ident = game + "|" + car + "|" + carId;
    bool applied = false;
    if (ident == m_carAppliedIdent)
    {
        // The same car, back after a dropout: the tiles already carry
        // what they carried (the entry, or the user's edits since).
        std::lock_guard<std::mutex> lk(m_carMx);
        applied = m_carState.applied;
    }
    else if (mayApply && !m.source.empty())
    {
        std::string err;
        applied = applyCarEntry(m, err);
        if (applied)
        {
            m_carAppliedIdent = ident;
            LOG_INFO(strf("Haptics: car preset '%s' (%s) applied for %s.", m.key.c_str(), m.source.c_str(), car.c_str()));
        }
        else
            LOG_WARNING(strf("Haptics: car preset '%s' could not be applied: %s", m.key.c_str(), err.c_str()));
    }
    else
        m_carAppliedIdent.clear();   // a car the table did not act on
    std::lock_guard<std::mutex> lk(m_carMx);
    m_carState.match   = m;
    m_carState.car     = car;
    m_carState.carId   = carId;
    m_carState.game    = game;
    m_carState.applied = applied;
}

// The follow switch lives in rig.json's haptics object (saved from the
// strip, applied live like the rest of it); read it from the file so a
// save from the web is honoured on the next car without a restart.
bool WebServer::followCarEnabled() const
{
    if (m_configPath.empty()) return false;
    Config cfg;
    if (!cfg.load(m_configPath)) return false;
    return cfg.get().hapticsFollowCar;
}

bool WebServer::applyCarEntry(const CarTable::Match& m, std::string& err)
{
    if (m.source.empty()) { err = "no entry"; return false; }
    std::lock_guard<std::mutex> lk(m_profilesIo);
    if (!applyHapticsObjectLocked(CarTable::toHapticsObject(m.entry), err)) return false;
    m_carGen.fetch_add(1);
    return true;
}

// Merge a haptics object (a whole profile, or a car entry's five tiles)
// over the live set: into rig.json (the working copy), then staged live.
// Serialised with saves from the web by m_profilesIo, held by the caller.
bool WebServer::applyHapticsObjectLocked(const QJsonObject& h, std::string& err)
{
    if (m_configPath.empty()) { err = "no config path"; return false; }
    Config cfg;
    if (!cfg.load(m_configPath)) { err = "cannot read the config"; return false; }
    Config::readHapticsObject(h, cfg.get());
    const auto errs = cfg.get().validate();
    if (!errs.empty()) { err = "fails validation: " + errs.front(); return false; }
    if (!cfg.saveRig(m_configPath)) { err = "cannot write rig.json"; return false; }
    m_rigKnownMtime.store(fileMtime(siblingFile(m_configPath, "rig.json")));
    if (m_motion) m_motion->stageHaptics(cfg.get());
    return true;
}

// Copy a profile over the live haptics set. The follow-car switch is the
// rig's, not the profile's: a profile never flips it.
bool WebServer::loadProfile(const std::string& name, std::string& err)
{
    if (m_configPath.empty()) { err = "no config path"; return false; }
    if (!m_profiles.has(name)) { err = "unknown profile"; return false; }
    std::lock_guard<std::mutex> lk(m_profilesIo);
    QJsonObject h = m_profiles.get(name);
    h.remove("followCar");
    if (!applyHapticsObjectLocked(h, err)) { err = "profile " + err; return false; }
    m_profiles.setActive(name);
    m_profiles.save(m_configPath);
    return true;
}

// Snapshot the SAVED haptics set under a name.
bool WebServer::snapshotProfile(const std::string& name, std::string& err)
{
    if (m_configPath.empty()) { err = "no config path"; return false; }
    std::lock_guard<std::mutex> lk(m_profilesIo);
    Config cfg;
    if (!cfg.load(m_configPath)) { err = "cannot read the config"; return false; }
    QJsonObject h = Config::writeHapticsObject(cfg.get());
    h.remove("followCar");
    m_profiles.put(name, h);
    m_profiles.setActive(name);
    if (!m_profiles.save(m_configPath)) { err = "cannot write profiles.json"; return false; }
    return true;
}

void WebServer::stop()
{
    m_running.store(false);
    if (m_statusThread.joinable()) m_statusThread.join();
    if (!m_configPath.empty() && m_effectStatus.dirty()) m_effectStatus.save(m_configPath);
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
