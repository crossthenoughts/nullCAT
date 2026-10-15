// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// main.cpp
// EtherCAT Motion Controller - StepperOnline A6 Drives
//
// Entry point: initializes Qt, loads config, creates all
// backend objects, connects them to the GUI, and runs the
// event loop.
//
// IMPORTANT: This application must be run as Administrator
// on Windows for SOEM raw socket access.
// ============================================================

#include <QApplication>
#include <QMessageBox>
#include <QTimer>

#include "MainWindow.h"
#include "Config.h"
#include "Logging.h"
#include "EtherCATMaster.h"
#include "TelemetryInput.h"
#include "MotionController.h"
#include "CarCache.h"
#include "ShakerBank.h"
#include "HapticsIdleClock.h"
#include "ControlLoop.h"
#include "WebServer.h"
#include "DirectInputButtons.h"
#include "ForegroundKeeper.h"
#include <memory>
#include <windows.h>

int main(int argc, char* argv[])
{
    // ---- Qt Application ----
    QApplication app(argc, argv);
    // Prevent zombie process from blocking relaunch
    HANDLE hMutex = CreateMutexA(nullptr, TRUE, "nullCAT_SingleInstance");
    if (GetLastError() == ERROR_ALREADY_EXISTS)
    {
        QMessageBox::critical(nullptr, "Already Running",
            "nullCAT is already running.\n"
            "Close the existing instance first.");
        CloseHandle(hMutex);
        return 1;
    }
    app.setApplicationName("nullCAT");
#ifndef NULLCAT_VERSION
#define NULLCAT_VERSION "dev"
#endif
    app.setApplicationVersion(NULLCAT_VERSION);
    app.setOrganizationName("nullCAT");

    // ---- Process-level RT hardening (Windows) ----
    // HIGH_PRIORITY_CLASS: elevates scheduler priority for the whole process,
    // not just the RT thread, so the OS doesn't starve it between EtherCAT frames.
    SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);

    // Disable Windows 11 Efficiency Mode / power throttling for this process.
    // MMCSS handles the RT thread, but this ensures the main + web threads
    // aren't throttled by the scheduler's EcoQoS policy.
    //
    // IGNORE_TIMER_RESOLUTION: Windows 11 silently IGNORES a background
    // process's timeBeginPeriod(1) -- the request the RT loop's hybrid
    // sleep/busy-wait depends on -- honouring it only while the process owns
    // the foreground window. The moment focus moves to SimHub or the game
    // (i.e. exactly when the operator parks/unparks), the effective timer
    // snaps to the 15.625ms default quantum and one Sleep() overshoots by a
    // full quantum. Field logs show the signature to the microsecond:
    // ~15.5-15.6ms RT stalls clustered on park/unpark events. This flag opts
    // out of that behaviour so timeBeginPeriod holds while unfocused
    // (StateMask 0 = disable the throttling policy). Complements the
    // ForegroundKeeper, which defends against the same policy family by
    // other means.
    PROCESS_POWER_THROTTLING_STATE pts = {};
    pts.Version     = PROCESS_POWER_THROTTLING_CURRENT_VERSION;
#ifndef PROCESS_POWER_THROTTLING_IGNORE_TIMER_RESOLUTION
#define PROCESS_POWER_THROTTLING_IGNORE_TIMER_RESOLUTION 0x4
#endif
    pts.ControlMask = PROCESS_POWER_THROTTLING_EXECUTION_SPEED
                    | PROCESS_POWER_THROTTLING_IGNORE_TIMER_RESOLUTION;
    pts.StateMask   = 0;  // 0 = disable throttling
    SetProcessInformation(GetCurrentProcess(),
        ProcessPowerThrottling, &pts, sizeof(pts));

    // ---- Load Configuration ----
    Config config;

    // The config anchor is always next to the executable. Only the anchor's
    // DIRECTORY matters: Config::load() reads host.json + rig.json siblings
    // (the legacy flat file is a migration fallback only, never written).
    // No working-directory fallback: it made the config location depend on
    // where the app was launched from, silently splitting host.json across
    // directories.
    QString exeDir  = QCoreApplication::applicationDirPath();
    QString cfgPath = exeDir + "/config.json";

    if (!config.load(cfgPath.toStdString()))
    {
        // A host.json or rig.json that is there but cannot be read: refuse
        // to start rather than run on defaults, which the first settings
        // save would write over the file. The logger is not up yet, so the
        // reason goes to stderr and a dialog; fix or move the file it names.
        fprintf(stderr, "nullCAT: refusing to start: %s\n", config.lastError().c_str());
        QMessageBox::critical(nullptr, "Config cannot be read",
            QString::fromStdString(config.lastError()) + "\n\nnullCAT will not start until the file is fixed or moved.");
        CloseHandle(hMutex);
        return 1;
    }

    // ---- Initialize Logger ----
    const AppConfig& cfg = config.get();
    std::string logPath = exeDir.toStdString() + "/" + cfg.logFile;
    Logger::instance().init(logPath, cfg.logToConsole);
    {
        auto dot = logPath.find_last_of('.');
        std::string diagPath = (dot != std::string::npos)
            ? logPath.substr(0, dot) + "_soem.log"
            : logPath + "_soem.log";
        Logger::instance().initDiag(diagPath);
    }

    // Apply log-level + diag-enable from config. logMinLevel filters
    // LOG_*/RT_LOG_*; diagEnabled gates logDiag() entirely. Both default to
    // their permissive setting (debug, true), so a config-less startup logs
    // everything.
    Logger::instance().setMinLevel(Logger::parseLevel(cfg.logMinLevel));
    Logger::instance().setDiagEnabled(cfg.diagEnabled);
    // What the config load clamped or found out of range (the files are
    // left as written): into the log now that there is one.
    for (const std::string& w : config.loadWarnings()) LOG_WARNING("Config: " + w);

    LOG_INFO("==========================================================");
    LOG_INFO("nullCAT starting...");
    LOG_INFO(strf("Config: host.json + rig.json in %s",
                  exeDir.toStdString().c_str()));
    LOG_INFO(strf("NIC: %s", cfg.nicName.empty() ? "(not set)" : cfg.nicName.c_str()));
    if (cfg.simulationMode)
    LOG_INFO("*** SIMULATION MODE ENABLED - no EtherCAT hardware required ***");
    LOG_INFO(strf("Drives: %d, Loop Hz: %d, Telemetry Port: %d",
        cfg.numDrives, cfg.controlLoopHz, cfg.telemetryPort));
    LOG_INFO("==========================================================");

    // ---- Create Backend Objects ----

    // EtherCAT master - apply all config before any init attempt
    EtherCATMaster master;
    master.setSimulationMode(cfg.simulationMode);
    master.applyConfig(cfg);

    // Telemetry UDP input
    TelemetryInput telemetry;
    telemetry.setFrozenMs(cfg.ncxFrozenMs);
    if (!telemetry.initialize(cfg.telemetryPort, cfg.telemetryBindAddr))
    {
        LOG_WARNING(strf("TelemetryInput: Failed to bind UDP port %d. "
                         "Telemetry will not be received.", cfg.telemetryPort));
        // Non-fatal: the app can still run manually
    }

    // Motion controller
    MotionController motion;
    motion.configure(cfg);

    // Remembered per-car gear ratios (device effects). Loaded before the
    // RT loop ever runs; saved at shutdown from the last learner snapshot.
    CarCache carCache;
    carCache.load(cfgPath.toStdString());
    motion.setCarCache(carCache.cars());

    // Control loop
    ControlLoop loop;
    loop.setComponents(&master, &telemetry, &motion);
    loop.setConfig(cfg);

    // ---- Web Server ----
    WebServer webServer;
    webServer.setComponents(&motion, &loop, &master, &config.get());
    {
        // Serve web/ relative to the executable directory
        std::string webRoot = exeDir.toStdString() + "/web";
        webServer.setWebRoot(webRoot);
        webServer.setCarTablePath(exeDir.toStdString() + "/cars.json");
        webServer.setPresetsPath(exeDir.toStdString() + "/presets.json");
    }
    webServer.setPort(cfg.webPort > 0 ? cfg.webPort : 8080);
    webServer.setBindAddr(cfg.webBindAddr);
    // Web-triggered re-init applies the SAVED config (the Qt file watcher
    // usually reloads first anyway; this makes the guarantee unconditional).
    webServer.setConfigReloader([&config, cfgPath]() -> bool
    {
        if (!config.load(cfgPath.toStdString())) { LOG_WARNING("Config: " + config.lastError()); return false; }
        for (const std::string& w : config.loadWarnings()) LOG_WARNING("Config: " + w);
        return true;
    });
    // Extra Host-header names the operator allows (host.json webAllowedHosts,
    // e.g. an mDNS name). Local interface addresses are always allowed.
    webServer.setAllowedHosts(cfg.webAllowedHosts);
    // Enable the web config editor (GET/POST /api/config) and the telemetry
    // receiving indicator. Without setConfigPath, /api/config 404s and the
    // web UI has no working config surface - defeating PC↔web parity.
    webServer.setConfigPath(cfgPath.toStdString());
    webServer.setTelemetry(&telemetry);

    // Shaker outputs: the sound card behind the shaker routes, and the idle
    // clock that keeps the haptics layer stepping while the control loop is
    // stopped (shakers with the drives off, or a rig with no drives at all).
    haptics::ShakerBank shakers;
    if (cfg.shakersEnabled)
    {
        std::vector<haptics::ShakerDeviceSpec> specs;
        for (const AppConfig::AudioDevice& d : cfg.shakerDevices()) specs.push_back({ d.device, d.channels });
        shakers.open(specs, cfg.controlLoopHz);
        for (const haptics::ShakerBank::Info& i : shakers.info())
        {
            if (i.open)
                LOG_INFO(strf("Shakers %d-%d: '%s', %d channel(s) at %d Hz, latency %.1f ms (card buffer %.1f ms).",
                              i.firstShaker + 1, i.firstShaker + i.channels, i.name.c_str(), i.channels, i.sampleRate,
                              i.latencyMs, i.bufferMs));
            else
                LOG_WARNING(strf("Shakers %d-%d: %s -- those routes stay silent.",
                                 i.firstShaker + 1, i.firstShaker + i.channels, i.error.c_str()));
            if (i.open && i.channels < i.channelsWanted)
                LOG_WARNING(strf("Shakers %d-%d: the card has %d of the %d channels asked for; shakers %d-%d stay silent.",
                                 i.firstShaker + 1, i.firstShaker + i.channelsWanted, i.channels, i.channelsWanted,
                                 i.firstShaker + i.channels + 1, i.firstShaker + i.channelsWanted));
        }
    }
    motion.setShakerOutput(&shakers);
    webServer.setShakerOutput(&shakers);
    HapticsIdleClock idleClock(motion, telemetry, loop, cfg.controlLoopHz);
    if (cfg.shakersEnabled) idleClock.start();
    webServer.setOnStopRequested([&loop]()
    {
        if (loop.isRunning())
            loop.stop();
    });
    webServer.setOnExitRequested([](int exitCode)
    {
        QCoreApplication::exit(exitCode);
    });
    // webUI defaults OFF; the Qt UI is the primary control surface on
    // Windows. Enable explicitly in host config (webUIEnabled=true) for the
    // HTTP dashboard.
    // Button backend (DirectInputButtons): USB game-controller capture +
    // runtime reader for the web binding wizard. Only meaningful with the web UI
    // up (the wizard lives there; runtime presses POST to the local server). The
    // thread sleeps unless bindings exist or a capture is armed; its dtor stops it.
    // Declared here so it outlives the run and destructs before webServer.
    DirectInputButtons buttons;
    if (cfg.webUIEnabled)
    {
        WebServer::ButtonBackendHooks bh;
        bh.armCapture      = [&buttons]() { return buttons.armCapture(); };
        bh.pollCapture     = [&buttons]() { return buttons.pollCapture(); };
        bh.bindingsChanged = [&buttons]() { buttons.bindingsChanged(); };
        webServer.setButtonHooks(std::move(bh));

        webServer.start();
        LOG_INFO(strf("WebServer started on port %d", webServer.port()));

        // buttons.json is the sibling of config.json (same dir), matching the
        // WebServer's siblingFile(configPath, "buttons.json") save path.
        buttons.start(exeDir.toStdString() + "/buttons.json", webServer.port());
    }
    else
    {
        LOG_INFO("WebServer disabled (config: webUIEnabled=false). Qt UI is the only control surface.");
    }

    // Keep the process classified as foreground-active so Windows 11
    // doesn't apply background throttling when the user clicks away to
    // another app. See ForegroundKeeper.h for the rationale. unique_ptr so
    // the widget lifetime matches main()'s scope and Qt cleanly destroys it
    // when the app exits.
    std::unique_ptr<ForegroundKeeper> fgKeeper;
    if (cfg.foregroundKeeperEnabled)
    {
        fgKeeper = std::make_unique<ForegroundKeeper>(
            cfg.foregroundKeeperX,
            cfg.foregroundKeeperY,
            cfg.foregroundKeeperAlpha);
    }
    else
    {
        LOG_INFO("ForegroundKeeper: disabled via config (foregroundKeeperEnabled=false). "
                 "RT thread may suffer background throttling when the main window loses focus.");
    }

    // ---- Create and Show Main Window ----
    // Pass &fgKeeper so the Application Settings dialog can rebuild it
    // live when the user toggles foregroundKeeperEnabled or tunes alpha/X/Y.
    MainWindow window;
    window.setComponents(&master, &telemetry, &motion, &loop, &config, &fgKeeper, &webServer);
    window.show();

    LOG_INFO("GUI initialized. Ready.");

    if (cfg.nicName.empty())
    {
        LOG_WARNING("NIC name not configured. Please edit config.json and set 'nicName'.");
    }

    // A car the sim moved away from: its learned gear ratios into the
    // cache now, not at shutdown (a box that is powered off rather than
    // closed would lose every car of the session). The learner publishes
    // each finished car once; the cache takes each once.
    auto rememberFinishedCar = [&]()
    {
        const MotionStatus ms = motion.getMotionStatus();
        carCache.takeFinished(ms.gearRatioPrevSeq, ms.gearRatioPrev, ms.gearRatioPrevConfident, ms.gearRatiosPrevDirty);
        if (carCache.flush(cfgPath.toStdString())) LOG_INFO("CarCache: learned gear ratios saved.");
    };
    QTimer carCacheTimer;
    QObject::connect(&carCacheTimer, &QTimer::timeout, rememberFinishedCar);
    carCacheTimer.start(250);

    // ---- Run Qt Event Loop ----
    int exitCode = app.exec();
    carCacheTimer.stop();

    // ---- Cleanup (also done in MainWindow::closeEvent, but belt+suspenders) ----
    if (loop.isRunning())
    {
        LOG_INFO("Shutdown: Stopping control loop...");
        loop.waitForStop();
    }

    // Stop the web server before SOEM/PDO teardown so its worker thread
    // can't dereference PDO pointers cleared by master.shutdown().
    webServer.stop();

    if (master.isInitialized())
    {
        LOG_INFO("Shutdown: Closing EtherCAT master...");
        master.shutdown();
    }

    idleClock.stop();
    shakers.close();
    telemetry.shutdown();

    // Persist the session's learned gear ratios (loop stopped, snapshot
    // final): a car finished since the last poll, then the car being driven.
    rememberFinishedCar();
    {
        const MotionStatus ms = motion.getMotionStatus();
        carCache.takeSession(ms.gearRatio, ms.gearRatioConfident, ms.gearRatiosDirty);
        if (carCache.flush(cfgPath.toStdString(), /*force=*/true)) LOG_INFO("CarCache: session gear ratios remembered.");
        else if (carCache.unsaved()) LOG_WARNING("CarCache: learned gear ratios could not be saved.");
    }

    LOG_INFO(strf("Application exited with code %d.", exitCode));
    return exitCode;
}
