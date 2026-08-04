// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// main_headless.cpp — NullCAT-Pi headless entry point
//
// Linux daemon equivalent of src/main.cpp, minus the Qt GUI and the
// Windows process-hardening. Wires the same backend objects
// (Config -> EtherCATMaster -> TelemetryInput -> MotionController ->
// ControlLoop -> WebServer) and runs until SIGINT/SIGTERM.
//
// The web UI is the control surface here: it owns init/start/stop, so
// this main only constructs, starts the web server, and blocks. No Qt
// event loop is needed (WebServer/httplib and ControlLoop run their own
// threads; TelemetryInput is polled by the RT loop).
//
// RT scheduling (SCHED_FIFO, core pin, mlockall) is applied inside
// ControlLoop via PlatformRT. That needs the rtprio/memlock limits
// (provisioned) and CAP_NET_RAW on this binary for SOEM raw sockets.
// ============================================================

#include "Config.h"
#include "Logging.h"
#include "EtherCATMaster.h"
#include "TelemetryInput.h"
#include "MotionController.h"
#include "ControlLoop.h"
#include "WebServer.h"
#include "EvdevButtons.h"
#include "A6Drive.h"
#include "GpioPanel.h"   // header is libgpiod-free; usage guarded by HAVE_LIBGPIOD

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <unistd.h>

// ---- shutdown signalling ----
static std::atomic<bool>    g_stop{false};
static std::atomic<int>     g_exitCode{0};
static std::condition_variable g_cv;
static std::mutex           g_cvMu;

// Async-signal-safe: just set the flag. The main loop polls it on a
// timed wait, so we deliberately do NOT call cv.notify here.
static void onSignal(int) { g_stop.store(true); }

// Called from normal threads (e.g. the web server) to request exit.
static void requestExit(int code)
{
    g_exitCode.store(code);
    g_stop.store(true);
    g_cv.notify_all();
}

// Resolve the directory containing this executable (for config.json + web/).
static std::string exeDir()
{
    char buf[4096];
    ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n > 0)
    {
        buf[n] = '\0';
        std::string p(buf);
        auto slash = p.find_last_of('/');
        if (slash != std::string::npos) return p.substr(0, slash);
    }
    return ".";
}

int main(int argc, char* argv[])
{
    const std::string dir = exeDir();

    // ---- config anchor: argv[1] override, else the executable's directory.
    // The anchor names the directory holding host.json/rig.json; no flat
    // config.json is read. Anchoring at exeDir unconditionally keeps the
    // config location independent of the launch directory.
    std::string cfgPath;
    if (argc > 1)
        cfgPath = argv[1];
    else
        cfgPath = dir + "/config.json";

    Config config;
    if (!config.load(cfgPath))
        fprintf(stderr, "Warning: could not load %s (%s). Using defaults.\n",
                cfgPath.c_str(), config.lastError().c_str());

    // Normalize the on-disk config to the full schema: fills defaults and writes
    // the complete drives[] array, so the web config editor always has every
    // field to edit (and a fresh/minimal config.json self-heals on first boot).
    config.save(cfgPath);

    const AppConfig& cfg = config.get();

    // ---- logger ----
    std::string logPath = dir + "/" + cfg.logFile;
    Logger::instance().init(logPath, cfg.logToConsole);
    {
        auto dot = logPath.find_last_of('.');
        std::string diagPath = (dot != std::string::npos)
            ? logPath.substr(0, dot) + "_soem.log"
            : logPath + "_soem.log";
        Logger::instance().initDiag(diagPath);
    }
    Logger::instance().setMinLevel(Logger::parseLevel(cfg.logMinLevel));
    Logger::instance().setDiagEnabled(cfg.diagEnabled);

    LOG_INFO("==========================================================");
    LOG_INFO("NullCAT-Pi headless controller starting...");
    LOG_INFO(strf("Config: %s", cfgPath.c_str()));
    LOG_INFO(strf("NIC: %s", cfg.nicName.empty() ? "(not set)" : cfg.nicName.c_str()));
    if (cfg.simulationMode)
        LOG_INFO("*** SIMULATION MODE — no EtherCAT hardware required ***");
    LOG_INFO(strf("Drives: %d, Loop Hz: %d, Telemetry port: %d",
        cfg.numDrives, cfg.controlLoopHz, cfg.telemetryPort));
    LOG_INFO("==========================================================");

    // ---- backend objects (same wiring as src/main.cpp) ----
    EtherCATMaster master;
    master.setSimulationMode(cfg.simulationMode);
    master.applyConfig(cfg);

    TelemetryInput telemetry;
    if (!telemetry.initialize(cfg.telemetryPort, cfg.telemetryBindAddr))
        LOG_WARNING(strf("TelemetryInput: failed to bind UDP port %d. "
                         "Telemetry will not be received.", cfg.telemetryPort));

    MotionController motion;
    motion.configure(cfg);

    ControlLoop loop;
    loop.setComponents(&master, &telemetry, &motion);
    loop.setConfig(cfg);

    WebServer webServer;
    webServer.setComponents(&motion, &loop, &master, &config.get());
    webServer.setWebRoot(dir + "/web");
    webServer.setConfigPath(cfgPath);
    webServer.setTelemetry(&telemetry);
    webServer.setPort(cfg.webPort > 0 ? cfg.webPort : 8080);
    webServer.setBindAddr(cfg.webBindAddr);
    webServer.setAllowedHosts(cfg.webAllowedHosts);
    webServer.setOnStopRequested([&loop]()
    {
        if (loop.isRunning())
            loop.stop();
    });
    webServer.setOnExitRequested([](int code) { requestExit(code); });

    // Button backend (EvdevButtons.h): capture + runtime reader for the web
    // binding wizard. A thread in THIS process — no new service; it blocks
    // on a condvar unless bindings exist or a capture is armed.
    EvdevButtons buttons;
    {
        WebServer::ButtonBackendHooks bh;
        bh.armCapture      = [&buttons]() { return buttons.armCapture(); };
        bh.pollCapture     = [&buttons]() { return buttons.pollCapture(); };
        bh.bindingsChanged = [&buttons]() { buttons.bindingsChanged(); };
        webServer.setButtonHooks(std::move(bh));
    }

    // Headless: the web UI is the ONLY control surface, so always start it.
    webServer.start();
    buttons.start(dir + "/buttons.json", webServer.port());
    LOG_INFO(strf("WebServer listening on %s:%d", cfg.webBindAddr.c_str(), webServer.port()));
    if (cfg.webBindAddr == "127.0.0.1")
        LOG_WARNING("webBindAddr=127.0.0.1 — web UI only reachable locally. "
                    "Set webBindAddr to 0.0.0.0 (or the eth1 IP) in host.json for PC access.");
    if (cfg.nicName.empty())
        LOG_WARNING("nicName not set in host.json — set it (e.g. eth0) before EtherCAT init.");

    // ---- optional GPIO control panel (Pi appliance) ----
#ifdef HAVE_LIBGPIOD
    GpioPanel gpioPanel;
    // gpioMode: "off" | "estop" | "estop_led" | "full"
    const std::string gpioMode = cfg.gpioMode;
    if (gpioMode != "off")
    {
        PanelPins pins;
        pins.chip     = cfg.gpioChip;
        pins.estop    = static_cast<unsigned>(cfg.gpioEstopPin);
        pins.engage   = static_cast<unsigned>(cfg.gpioEngagePin);
        pins.park     = static_cast<unsigned>(cfg.gpioParkPin);
        pins.ledRun   = static_cast<unsigned>(cfg.gpioLedRunPin);
        pins.ledReady = static_cast<unsigned>(cfg.gpioLedReadyPin);
        pins.ledFault = static_cast<unsigned>(cfg.gpioLedFaultPin);
        pins.useLeds    = (gpioMode == "estop_led" || gpioMode == "full");
        pins.useButtons = (gpioMode == "full");

        // Buttons fire the same actions as the web endpoints.
        PanelActions acts;
        acts.init  = [&webServer]() { webServer.requestInit(); };
        acts.start = [&loop, &master]()
        {
            if (master.isOperational() && !loop.isRunning())
                loop.start();
        };
        acts.stop = [&loop]()
        {
            if (loop.isRunning())
                loop.stop();
        };
        acts.park = [&motion]()
        {
            MotionCommand c;
            c.type = MotionCommand::Type::StartPark;
            motion.enqueueCommand(c);
        };
        acts.unpark = [&motion]()
        {
            MotionCommand c;
            c.type = MotionCommand::Type::StartUnpark;
            motion.enqueueCommand(c);
        };
        acts.estop = [&motion, &master]()
        {
            motion.setEmergencyStop(true);
            master.disableAllDrives();
        };
        acts.estopRelease = [&motion]() { motion.setEmergencyStop(false); };

        auto getStatus = [&master, &loop, &motion, &webServer]() -> PanelStatus
        {
            PanelStatus s;
            s.masterOp    = master.isOperational();
            s.loopRunning = loop.isRunning();
            s.estop       = motion.isEmergencyStop();
            s.initBusy    = webServer.isInitBusy();
            if (s.masterOp)
                for (int i = 0; i < master.getDriveCount(); ++i)
                {
                    A6Drive* d = master.getDrive(i);
                    if (d && d->isFault()) { s.fault = true; break; }
                }
            // Motion phase for LED patterns: homing = any axis homing,
            // parked = all axes parked.
            const MotionStatus mst = motion.getMotionStatus();
            bool allParked = (mst.numDrives > 0);
            for (int i = 0; i < mst.numDrives && i < MAX_DRIVES; ++i)
            {
                if (mst.axisState[i] == AxisMotionState::HOMING) s.homing = true;
                if (mst.axisState[i] != AxisMotionState::PARKED) allParked = false;
            }
            s.parked = allParked;
            return s;
        };

        webServer.setOnLedTest([&gpioPanel]() { gpioPanel.requestLedTest(); });

        if (gpioPanel.start(pins, getStatus, acts))
            LOG_INFO(strf("GPIO control panel enabled (mode=%s).", gpioMode.c_str()));
        else
            LOG_WARNING("GPIO control panel failed to start — continuing without it.");
    }
    else
    {
        LOG_INFO("GPIO control panel disabled (gpioMode=off).");
    }
#endif

    // ---- signal handlers ----
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    LOG_INFO("Ready. Ctrl-C (SIGINT) to exit.");

    // ---- block until shutdown requested ----
    {
        std::unique_lock<std::mutex> lk(g_cvMu);
        while (!g_stop.load())
            g_cv.wait_for(lk, std::chrono::milliseconds(250),
                          [] { return g_stop.load(); });
    }

    // ---- ordered shutdown (mirrors src/main.cpp) ----
    LOG_INFO("Shutdown requested.");
#ifdef HAVE_LIBGPIOD
    gpioPanel.stop();   // turns LEDs off + releases the lines
#endif
    if (loop.isRunning())
    {
        LOG_INFO("Stopping control loop...");
        loop.waitForStop();
    }
    buttons.stop();
    webServer.stop();
    if (master.isInitialized())
    {
        LOG_INFO("Closing EtherCAT master...");
        master.shutdown();
    }
    telemetry.shutdown();

    LOG_INFO(strf("Exited with code %d.", g_exitCode.load()));
    return g_exitCode.load();
}
