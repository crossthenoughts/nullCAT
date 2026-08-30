// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// WebServer.h
//
// Embedded HTTP + WebSocket server using cpp-httplib.
// Runs in its own thread at normal priority - completely
// isolated from the RT loop by the command queue.
//
// Serves (core set; config, toggle and lifecycle endpoints are
// registered in WebServer.cpp):
//   GET  /          → web/index.html
//   GET  /app.js    → web/app.js
//   GET  /api/status → JSON snapshot (polled fallback)
//   WS   /ws        → 10Hz state push (JSON)
//   POST /api/start      → start the control loop
//   POST /api/stop       → stop control loop
//   POST /api/estop      → set emergency stop
//   POST /api/estop/release → clear emergency stop
//   POST /api/home       → enqueue StartHoming (all axes)
//   POST /api/park       → enqueue StartPark
//   POST /api/reset-fault → clear fault lockout
//
// Thread safety:
//   All state reads go through MotionController::getMotionStatus()
//   (shared_mutex protected snapshot) and ControlLoop::getStats()
//   (shared_mutex). Commands go through enqueueCommand() or the
//   atomic setEmergencyStop(). No direct access to RT state.
// ============================================================

#include "MotionController.h"
#include "AxisKind.h"
#include "ControlLoop.h"
#include "EtherCATMaster.h"
#include "Config.h"
#include "Logging.h"

#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <mutex>
#include <ctime>
#include <chrono>
#include <vector>

// Forward declaration - avoids pulling the heavy httplib.h into every TU
// that includes WebServer.h.
namespace httplib { class Server; }

class WebServer
{
public:
    WebServer();
    ~WebServer();

    // Call before start(). webRoot is the path to the web/ directory.
    void setComponents(MotionController* motion,
                       ControlLoop*      loop,
                       EtherCATMaster*   master,
                       const AppConfig*  config);

    void setWebRoot(const std::string& path)    { m_webRoot    = path; }
    // Path to config.json - enables GET/POST /api/config (passthrough save).
    void setConfigPath(const std::string& path) { m_configPath = path; }
    // Telemetry handle - lets /api/status report the receiving indicator.
    void setTelemetry(TelemetryInput* s)              { m_telemetry     = s; }
    void setPort(int port)                      { m_port       = port; }
    // Bind address. Defaults to 127.0.0.1 (loopback - no firewall prompt).
    // Set to "0.0.0.0" to allow remote access (RPi headless, phone, etc.)
    void setBindAddr(const std::string& addr)   { m_bindAddr   = addr; }
    // Extra Host-header names the operator allows (host.json webAllowedHosts,
    // e.g. an mDNS name). Local interface addresses + localhost forms are
    // always allowed; this only appends.
    void setAllowedHosts(std::vector<std::string> hosts) { m_extraHosts = std::move(hosts); }

    // ---- Host-header validation helpers (public statics: unit-tested) ----
    // Hostname part of a Host header, lowercased, port stripped. Handles
    // "name", "name:port", "1.2.3.4:port", "[::1]:port".
    static std::string hostHeaderName(const std::string& hostHeader);
    // True for loopback, RFC1918, IPv4 link-local, IPv6 loopback/link-local/
    // ULA - i.e. a client on the local machine or a private LAN. Gates the
    // destructive endpoints (/api/shutdown, /api/restart): they must never be
    // driveable from a non-private source address.
    static bool isPrivateClientAddr(const std::string& addr);

    void setOnStopRequested(std::function<void()> cb)         { m_onStopRequested  = std::move(cb); }
    // GPIO panel LED self-test trigger (wired to GpioPanel in main_headless).
    void setOnLedTest(std::function<void()> cb)               { m_onLedTest       = std::move(cb); }
    // Reload the AppConfig from disk (host.json/rig.json). Called by the
    // init worker before re-applying config to the motion controller, so a
    // web save made while EtherCAT was up takes effect on the NEXT
    // Initialize. The Qt build reloads via its file watcher; the headless
    // build has no watcher, so WITHOUT this hook a Pi re-init silently
    // applied the boot-time config - saved axes/settings looked dead until
    // a full service restart (found on the bench, 0.9.5 beta).
    void setConfigReloader(std::function<bool()> cb)          { m_configReloader   = std::move(cb); }
    // exitCode 0 = clean quit (watchdog does not relaunch)
    // exitCode 2 = restart requested (watchdog relaunches after 500ms)
    void setOnExitRequested(std::function<void(int)> cb)  { m_onExitRequested     = std::move(cb); }

    // Pi click-updater hook: launch the updater for a bare "x.y.z" version
    // string (endpoint-sanitized; the hook must sanitize again). Returns ""
    // on success or a human-readable error. Left unset on builds that
    // update by other means (Windows: the release zip) -- the endpoint
    // then refuses with a clear message.
    void setUpdateStarter(std::function<std::string(const std::string&)> fn)
    { m_onUpdateStart = std::move(fn); }

    bool start();
    void stop();
    bool isRunning() const { return m_running.load(); }

    // Trigger the async EtherCAT init / de-init on the shared worker thread.
    // Used by the web endpoints AND the GPIO panel (single init path). Return
    // false if not currently possible (components missing, already
    // operational/initialized, or a request is in flight).
    bool requestInit();
    bool requestDeinit();
    // Why the last requestInit/requestDeinit returned false. Set on every
    // refusal path so the caller can report ONE true reason instead of a
    // list of maybes.
    std::string lastRefusal() const;

    // Per-drive BELT mask from config, consumed by the StatusModel
    // aggregate derivations (buildStatusJson + belts-toggle). A belt is a
    // belt-typed torque axis (Stage C re-key): torque mode alone must not
    // classify an axis as a belt, or the 0.9.5 device family (shifter,
    // pedal) would be gripped by the belts controls.
    // Returns the number of config drives written (clamped to MAX_DRIVES).
    int beltAxisMask(bool out[]) const
    {
        int n = m_config ? static_cast<int>(m_config->drives.size()) : 0;
        if (n > MAX_DRIVES) n = MAX_DRIVES;
        for (int i = 0; i < n; ++i)
        {
            const AxisCaps c = axisCaps(m_config->drives[i].axisType,
                                        m_config->drives[i].mode);
            out[i] = c.beltType && c.torqueMode;
        }
        return n;
    }

    // Per-drive DEVICE mask (shifter/pedal families), the device-toggle's
    // twin of beltAxisMask.
    int deviceAxisMask(bool out[]) const
    {
        int n = m_config ? static_cast<int>(m_config->drives.size()) : 0;
        if (n > MAX_DRIVES) n = MAX_DRIVES;
        for (int i = 0; i < n; ++i)
            out[i] = axisCaps(m_config->drives[i].axisType,
                              m_config->drives[i].mode).isDevice();
        return n;
    }

    // Platform-specific physical-button backend.
    // The web wizard's capture flow and the hot-apply notification route
    // through these; the WebServer itself stays platform-agnostic. Unset
    // hooks degrade gracefully (capture reports no backend; save still works).
    struct ButtonBackendHooks
    {
        std::function<bool()>        armCapture;      // arm one-shot capture of the next press
        std::function<std::string()> pollCapture;     // JSON {"captured":...}
        std::function<void()>        bindingsChanged; // buttons.json saved -> hot-apply
    };
    void setButtonHooks(ButtonBackendHooks h) { m_buttonHooks = std::move(h); }
    bool isInitBusy() const { return m_initBusy.load(); }

    int port() const { return m_port; }

private:
    MotionController* m_motion  = nullptr;
    ControlLoop*      m_loop    = nullptr;
    EtherCATMaster*   m_master  = nullptr;
    const AppConfig*  m_config  = nullptr;
    TelemetryInput*      m_telemetry  = nullptr;

    std::string m_webRoot   = "web";
    std::string m_configPath;          // config.json path (for /api/config)
    // /api/meta reports <ns>PendingRestart when a config
    // file's mtime is newer than this (saved, but this process hasn't reloaded
    // it). Server-owned so the pill survives page reloads / multiple clients.
    time_t      m_processStart = ::time(nullptr);
    ButtonBackendHooks m_buttonHooks;
    int         m_port      = 8080;
    std::string m_bindAddr  = "127.0.0.1";

    std::thread       m_thread;
    std::atomic<bool> m_running{false};

    // Set inside the server thread once svr is constructed; used by stop()
    // to call svr.stop() and unblock svr.listen().
    std::atomic<httplib::Server*> m_svr{nullptr};

    // Async init - pre-created thread waits for signal, runs initializeAndEnterOp,
    // then waits again. Avoids cold thread creation at click time.
    std::thread            m_initThread;
    std::atomic<bool>      m_initBusy{false};
    std::string            m_initLastError;
    std::string            m_lastRefusal;      // guarded by m_initMutex
    mutable std::mutex     m_initMutex;   // mutable: lastRefusal() is const
    std::condition_variable m_initCv;
    bool                   m_initRequested{false};
    bool                   m_deinitRequested{false};  // de-init: drives OP→INIT, master closed
    bool                   m_initThreadStop{false};

    std::function<void()>        m_onStopRequested;
    std::function<void()>        m_onLedTest;
    std::function<bool()>        m_configReloader;
    std::function<void(int)> m_onExitRequested;
    std::function<std::string(const std::string&)> m_onUpdateStart;

    // ---- Host-header allowlist (DNS-rebinding defense) ----
    // A request is served only when its Host header names this machine:
    // localhost forms, the configured bind address, any address currently on
    // a local interface, or an operator-listed name. A rebinding page carries
    // its OWN hostname in Host, which can never match. Fail-closed: rejected
    // requests get 421 before any handler runs.
    bool hostAllowed(const std::string& hostHeader);
    // Current local interface addresses (getifaddrs / GetAdaptersAddresses).
    static std::vector<std::string> collectLocalAddrs();
    std::mutex               m_hostCacheMutex;
    std::vector<std::string> m_localAddrs;       // cached; rescanned on miss
    std::chrono::steady_clock::time_point m_lastAddrScan{};
    std::vector<std::string> m_extraHosts;       // webAllowedHosts (host.json)

    void setRefusal(const char* why);

    // Builds the JSON status string pushed over WebSocket and returned by /api/status
    std::string buildStatusJson() const;

    // Reads a file from m_webRoot and returns its contents.
    // Returns empty string if the file cannot be opened.
    std::string readWebFile(const std::string& filename) const;

    // Returns MIME type for common web extensions.
    static const char* mimeType(const std::string& filename);
};
