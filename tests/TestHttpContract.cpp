// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestHttpContract.cpp - pins Docs/COMMAND_CONTRACT.md at the HTTP layer.
//
// TestCommandContract proves MotionCommand => engine state. THIS suite proves
// the WIRING above it: real POSTs to the real WebServer routes, against the
// full simulation-mode stack assembled exactly as pi/main_headless.cpp does
// (EtherCATMaster sim + TelemetryInput + MotionController + ControlLoop +
// WebServer). This is the layer where the project's escaped-bug class lives
// (endpoint string != engine action, field-accepted-but-never-applied): the
// torque-mode UI-string bug would have failed here on day one.
//
// Contract facts exercised (see COMMAND_CONTRACT.md):
//   - HTTP-layer guards produce ok:false BEFORE the engine is touched
//     (/api/start before init).
//   - ok:true means ACCEPTED, not DONE: outcome truth is polled from
//     /api/status (e.g. TensionBelts under e-stop returns ok yet the belt
//     stays slack - the silent engine refusal, visible only via status).
//   - Option A ruling: after /api/start (auto-home + auto-unpark in sim) and
//     after e-stop release, the belt stays SLACK until explicit tension.
//
// Assertions are dumb substring checks on the status JSON - deliberately: the
// web UI itself consumes these exact tokens.
//
//   ctest -R HttpContract
// ============================================================
#include "EtherCATMaster.h"
#include "TelemetryInput.h"
#include "MotionController.h"
#include "ControlLoop.h"
#include "WebServer.h"
#include "Config.h"
#include "Logging.h"
#include "httplib.h"

#include <cstdio>
#include <string>
#include <thread>
#include <chrono>

static int g_fail = 0, g_pass = 0;
static void check(bool ok, const char* name)
{
    if (ok) { ++g_pass; std::printf("[PASS] %s\n", name); }
    else    { ++g_fail; std::printf("[FAIL] %s\n", name); }
}

static const char* HOST = "127.0.0.1";
static const int   PORT = 18742;

static std::string get(const char* path)
{
    httplib::Client cli(HOST, PORT);
    cli.set_read_timeout(2, 0);
    auto r = cli.Get(path);
    return r ? r->body : std::string();
}

static std::string post(const char* path, const char* body = "")
{
    httplib::Client cli(HOST, PORT);
    cli.set_read_timeout(5, 0);
    auto r = cli.Post(path, body, "application/json");
    return r ? r->body : std::string();
}

static bool has(const std::string& body, const char* token)
{
    return body.find(token) != std::string::npos;
}

// Poll /api/status until it contains `token` (or timeout). The contract says
// outcome truth is status polling - the tests obey their own contract.
static bool waitStatus(const char* token, int timeoutMs)
{
    for (int t = 0; t < timeoutMs; t += 50)
    {
        if (has(get("/api/status"), token)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
}

// BELT-ONLY rig: sim drives cannot complete a torque-hardstop homing search
// (no sim torque model, and homing targets bypass the sim actuals) -- a KNOWN
// sim-fidelity gap (backlog: sim homing model). A vertical axis would sit in
// HOMING forever and the toggle transition-guard would (correctly) refuse
// everything. Belts skip homing entirely, so a belt-only rig has no stuck
// states and every assertion here stays meaningful; vertical-axis engine
// behavior is covered by TestCommandContract at the core layer.
static AppConfig makeCfg()
{
    DriveConfig belt;
    belt.slaveIndex = 1;
    belt.axisType   = "belt";
    belt.mode       = "torque";
    belt.strokeMm = 100.0; belt.countsPerMm = 100.0;
    belt.maxVelocityMmS = 200.0; belt.maxAccelerationMmS2 = 2000.0;
    belt.maxJerkMmS3 = 20000.0;
    belt.unparkTimeSec = 0.1; belt.parkTimeSec = 0.1;
    belt.onlineHoldTimeoutSec = 60.0;   // no-data standby must not race the toggle tests
    belt.torqueMinPct = 5.0;
    belt.torqueMaxPct = 50.0;

    AppConfig cfg;
    cfg.simulationMode = true;
    cfg.controlLoopHz  = 100;
    cfg.numDrives      = 1;
    cfg.blendTimeSec   = 0.1;
    cfg.blendMaxVelocityMmS = 20.0;
    cfg.webBindAddr    = HOST;
    cfg.webPort        = PORT;
    cfg.telemetryPort     = 18743;
    cfg.drives.push_back(belt);
    return cfg;
}

int main()
{
    Logger::instance().setMinLevel(LogLevel::LVL_ERROR);
    AppConfig cfg = makeCfg();

    // ---- assemble the stack exactly as pi/main_headless.cpp does ----
    EtherCATMaster master;
    master.setSimulationMode(true);
    master.applyConfig(cfg);

    TelemetryInput telemetry;
    telemetry.initialize(cfg.telemetryPort);   // bind failure tolerated (no UDP used here)

    MotionController motion;
    motion.configure(cfg);

    ControlLoop loop;
    loop.setComponents(&master, &telemetry, &motion);
    loop.setConfig(cfg);

    WebServer web;
    web.setComponents(&motion, &loop, &master, &cfg);
    web.setTelemetry(&telemetry);
    web.setPort(PORT);
    web.setBindAddr(HOST);
    web.setOnStopRequested([&loop]() { if (loop.isRunning()) loop.stop(); });

    if (!web.start())
    {
        std::printf("[FAIL] WebServer failed to start on %s:%d\n", HOST, PORT);
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // ---- surface up before engine: status + meta respond ----
    check(has(get("/api/status"), "\"loopRunning\""), "GET /api/status responds pre-init");
    check(has(get("/api/meta"),   "hostOwner"),       "GET /api/meta exposes hostOwner");
    check(has(get("/api/meta"),   "\"rigPendingRestart\":false"),
          "GET /api/meta: pending-restart flags present, false with no saves");

    // ---- Phase 5 button endpoints: graceful degradation is contract ----
    // (no config path and no capture backend wired in this harness)
    check(has(get("/api/buttons"), "\"bindings\":[]"),
          "GET /api/buttons: empty map when no buttons.json");
    check(has(post("/api/buttons/listen"), "\"ok\":false"),
          "POST /api/buttons/listen: refused with no capture backend");
    check(has(get("/api/buttons/capture"), "\"captured\":false"),
          "GET /api/buttons/capture: no-backend reports captured:false");

    // ---- HTTP-layer guard: start before init is ok:false ----
    check(has(post("/api/start"), "\"ok\":false"),
          "POST /api/start before init: refused at the HTTP layer");

    // ---- init (sim) -> masterOp ----
    check(has(post("/api/init"), "\"ok\":true"), "POST /api/init accepted");
    check(waitStatus("\"masterOp\":true", 15000), "status reaches masterOp:true");

    // ---- start loop -> auto-home -> Option A: belt stays SLACK ----
    check(has(post("/api/start"), "\"ok\":true"), "POST /api/start accepted");
    check(waitStatus("\"loopRunning\":true", 5000), "status reaches loopRunning:true");
    check(waitStatus("\"hasBelts\":true", 2000),    "status: hasBelts:true");
    std::this_thread::sleep_for(std::chrono::seconds(2));   // let auto-home/unpark finish
    check(has(get("/api/status"), "\"beltsSlack\":true"),
          "Option A: belt still SLACK after start/auto-unpark");

    // ---- explicit tension is the one belt trigger ----
    check(has(post("/api/belts/tension"), "\"ok\":true"), "POST /api/belts/tension accepted");
    check(waitStatus("\"beltsSlack\":false", 5000), "tension: beltsSlack goes false");

    // ---- toggle endpoints (one button per pair; server-resolved + guarded) ----
    // Runs BEFORE the e-stop section: e-stop release triggers a re-home, and
    // sim drives have no hardstop torque model (getTorquePercent()==0 without
    // PDO pointers) so that homing never completes -- a KNOWN sim-fidelity
    // gap (backlog: sim hardstop model). The transition guard then correctly
    // refuses toggles forever after. Here everything is settled.
    // Belt is TENSIONED but may still be BLENDING from the re-tension above --
    // and the transition guard makes toggles NO-OPS during transitions (by
    // design). Settle first, then exercise resolution + cooldown.
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(800));   // blend -> ONLINE
        std::string r = post("/api/belts-toggle");
        check(has(r, "\"resolved\":\"belts/slack\""), "belts-toggle from tensioned: resolves slack");
        check(has(post("/api/belts-toggle"), "\"ok\":false"),
              "belts-toggle immediately again: cooldown refuses");
        check(waitStatus("\"beltsSlack\":true", 5000), "belts-toggle: belt goes slack");
        std::this_thread::sleep_for(std::chrono::milliseconds(1700));  // cooldown + settle PARKED
        check(has(post("/api/belts-toggle"), "\"resolved\":\"belts/tension\""),
              "belts-toggle after cooldown: resolves tension");
        check(waitStatus("\"beltsSlack\":false", 5000), "belts-toggle: belt re-tensions");
    }
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(800));   // blend -> ONLINE (settled)
        std::string r = post("/api/park-toggle");
        check(has(r, "\"resolved\":\"park\""), "park-toggle from online: resolves park");
        check(has(post("/api/park-toggle"), "\"ok\":false"),
              "park-toggle immediately again: cooldown refuses");
        check(waitStatus("\"parked\":true", 10000), "park-toggle: rig parks");
    }

    // ---- update endpoint: refused without an updater hook (this harness
    // sets none, exactly like the Windows build), never silently accepted ----
    check(has(post("/api/update/start", "{\"version\":\"9.9.9\"}"), "\"ok\":false"),
          "update/start without an updater hook: visibly refused");

    // ---- per-axis home body ({"axis":N}, 1-based; no body = all axes) ----
    check(has(post("/api/home", "{\"axis\":1}"), "\"ok\":true"),
          "home axis 1: accepted (belt rig: engine skips, belts never home)");
    check(has(post("/api/home", "{\"axis\":7}"), "\"ok\":false"),
          "home axis 7: out of range, refused at the HTTP layer");
    check(has(post("/api/home", "not json"), "\"ok\":true"),
          "home with junk body: falls back to home-all (accepted)");


    // ---- e-stop; tension ACCEPTED but silently refused (contract fact #1/#2) ----
    check(has(post("/api/estop"), "\"ok\":true"), "POST /api/estop accepted");
    check(waitStatus("\"estop\":true", 3000), "status: estop:true");
    check(waitStatus("\"beltsSlack\":true", 5000), "e-stop slacks the belt");
    check(has(post("/api/belts/tension"), "\"ok\":true"),
          "tension under e-stop: ok:true (ACCEPTED, not done)");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    check(has(get("/api/status"), "\"beltsSlack\":true"),
          "tension under e-stop: belt STAYS slack (silent engine refusal)");

    // ---- release restores permission, not state (Option A) ----
    check(has(post("/api/estop/release"), "\"ok\":true"), "POST /api/estop/release accepted");
    check(waitStatus("\"estop\":false", 3000), "status: estop:false");
    std::this_thread::sleep_for(std::chrono::seconds(2));   // re-home/auto-unpark window
    check(has(get("/api/status"), "\"beltsSlack\":true"),
          "Option A: belt still slack after e-stop release");
    check(has(post("/api/belts/tension"), "\"ok\":true"), "re-tension accepted after release");
    check(waitStatus("\"beltsSlack\":false", 5000), "re-tension takes effect");

    // ---- B1 security hardening: Host allowlist + no CORS ----
    // Pure classifier checks (no server involved).
    check(WebServer::hostHeaderName("LOCALHOST:8080") == "localhost", "hostHeaderName strips port + lowercases");
    check(WebServer::hostHeaderName("[::1]:8080") == "::1",           "hostHeaderName handles bracketed IPv6");
    check(WebServer::hostHeaderName("fe80::1") == "fe80::1",          "hostHeaderName leaves bare IPv6 whole");
    check(WebServer::isPrivateClientAddr("127.0.0.1"),   "private: loopback");
    check(WebServer::isPrivateClientAddr("192.168.50.5"),"private: RFC1918 /16");
    check(WebServer::isPrivateClientAddr("10.1.2.3"),    "private: RFC1918 /8");
    check(WebServer::isPrivateClientAddr("172.31.0.9"),  "private: RFC1918 /12");
    check(WebServer::isPrivateClientAddr("::1"),         "private: IPv6 loopback");
    check(!WebServer::isPrivateClientAddr("8.8.8.8"),    "public IPv4 rejected");
    check(!WebServer::isPrivateClientAddr("172.32.0.1"), "172.32/12 boundary rejected");
    check(!WebServer::isPrivateClientAddr(""),           "unparseable addr fails closed");

    // Wire checks: a foreign Host is rejected BEFORE any handler, and the
    // rejection carries no state change. The engine is live here (estop off).
    {
        httplib::Client cli(HOST, PORT);
        cli.set_read_timeout(2, 0);
        httplib::Headers evil = {{"Host", "evil.example.com"}};
        auto r = cli.Get("/api/status", evil);
        check(r && r->status == 421, "foreign Host on GET /api/status -> 421");
        auto p = cli.Post("/api/estop", evil, "", "application/json");
        check(p && p->status == 421, "foreign Host on POST /api/estop -> 421");
        check(has(get("/api/status"), "\"estop\":false"),
              "foreign-Host estop POST changed NOTHING (engine untouched)");
        auto sd = cli.Post("/api/shutdown", evil, "", "application/json");
        check(sd && sd->status == 421, "foreign Host on POST /api/shutdown -> 421 (never reaches handler)");
        // The machine's own hostname (bare + .local mDNS form) is always
        // allowed with zero configuration.
        {
            char hn[256] = {};
            if (gethostname(hn, sizeof(hn) - 1) == 0 && hn[0])
            {
                httplib::Headers own = {{"Host", std::string(hn) + ".local:8080"}};
                auto r2 = cli.Get("/api/status", own);
                check(r2 && r2->status == 200, "own hostname .local form accepted (mDNS)");
            }
        }
        // Legit local Host still fully served, with no CORS header anywhere.
        auto ok = cli.Get("/api/status");
        check(ok && ok->status == 200, "Host 127.0.0.1:port accepted");
        check(ok && !ok->has_header("Access-Control-Allow-Origin"),
              "no Access-Control-Allow-Origin on any response");
    }

    // ---- stop -> deinit ----
    check(has(post("/api/stop"), "\"ok\":true"), "POST /api/stop accepted");
    check(waitStatus("\"loopRunning\":false", 10000), "status reaches loopRunning:false");
    check(has(post("/api/deinit"), "\"ok\":true"), "POST /api/deinit accepted");
    check(waitStatus("\"masterOp\":false", 15000), "status reaches masterOp:false");

    // ---- teardown ----
    if (loop.isRunning()) loop.stop();
    web.stop();
    master.shutdown();

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
