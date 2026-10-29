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
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <filesystem>

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

// A POST with extra headers, status code included (the 409 contract).
struct Reply { int status = 0; std::string body; };
static Reply postHdr(const char* path, const std::string& body, const httplib::Headers& headers)
{
    httplib::Client cli(HOST, PORT);
    cli.set_read_timeout(5, 0);
    auto r = cli.Post(path, headers, body, "application/json");
    Reply out;
    if (r) { out.status = r->status; out.body = r->body; }
    return out;
}

// GET /api/rig: the body and the apply generation it belongs to
// (X-Nullcat-Gen), -1 when the header is missing.
static bool getRig(std::string& body, int& gen)
{
    httplib::Client cli(HOST, PORT);
    cli.set_read_timeout(2, 0);
    auto r = cli.Get("/api/rig");
    if (!r) return false;
    body = r->body;
    gen  = r->has_header("X-Nullcat-Gen") ? std::atoi(r->get_header_value("X-Nullcat-Gen").c_str()) : -1;
    return r->status == 200;
}

// The status's hapCarGen, -1 when missing.
static int genNow()
{
    const std::string st = get("/api/status");
    const size_t a = st.find("\"hapCarGen\":");
    return (a == std::string::npos) ? -1 : std::atoi(st.c_str() + a + 12);
}

// A freshly started server answers once its thread has bound the port,
// which on a loaded machine is a second or two after start() returns.
static bool waitUp(int port, int timeoutMs)
{
    for (int t = 0; t < timeoutMs; t += 100)
    {
        httplib::Client cli(HOST, port);
        cli.set_read_timeout(1, 0);
        auto r = cli.Get("/");
        if (r) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

// The rig body with one haptics key (top level of the haptics object, or
// inside a tile) set to a number, for the save tests.
static std::string rigWith(const std::string& body, const char* tile, const char* key, double value)
{
    QJsonObject root = QJsonDocument::fromJson(QByteArray::fromStdString(body)).object();
    QJsonObject g = root.value("global").toObject();
    QJsonObject h = g.value("haptics").toObject();
    if (tile) { QJsonObject t = h.value(tile).toObject(); t[key] = value; h[tile] = t; }
    else h[key] = value;
    g["haptics"] = h; root["global"] = g;
    return QJsonDocument(root).toJson().toStdString();
}

// The same for a top-level haptics switch.
static std::string rigWithBool(const std::string& body, const char* key, bool value)
{
    QJsonObject root = QJsonDocument::fromJson(QByteArray::fromStdString(body)).object();
    QJsonObject g = root.value("global").toObject();
    QJsonObject h = g.value("haptics").toObject();
    h[key] = value;
    g["haptics"] = h; root["global"] = g;
    return QJsonDocument(root).toJson().toStdString();
}

// A body with its spaces removed (the file is indented, a space after each colon).
static std::string compact(std::string s)
{
    s.erase(std::remove(s.begin(), s.end(), ' '), s.end());
    s.erase(std::remove(s.begin(), s.end(), '\n'), s.end());
    return s;
}

// Name a car the way a plugin does it (a NULLCATY line fed straight in)
// while the status is polled for `want`.
static bool nameCar(TelemetryInput& telemetry, const char* game, const char* car, const char* want, int timeoutMs)
{
    for (int t = 0, rpm = 2000; t < timeoutMs; t += 100, rpm += 7)
    {
        const std::string line = std::string("NULLCATY,game=") + game + ",car=" + car + ",rpm=" + std::to_string(rpm);
        TelemetryData td;
        if (TelemetryInput::parsePacket(line.c_str(), static_cast<int>(line.size()), td)) telemetry.ingest(td);
        if (has(get("/api/status"), want)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
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

// One tile's mute from the status's hapMute[] (registry order), -1 when
// the list is missing or short.
static int muteAt(const std::string& st, int idx)
{
    const size_t a = st.find("\"hapMute\":[");
    if (a == std::string::npos) return -1;
    size_t p = a + 11;
    for (int i = 0; i < idx; ++i)
    {
        p = st.find(',', p);
        if (p == std::string::npos) return -1;
        ++p;
    }
    if (st.compare(p, 4, "true") == 0) return 1;
    if (st.compare(p, 5, "false") == 0) return 0;
    return -1;
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

    // A scratch config directory, as the mains give the server: the
    // haptics Test endpoint reads the saved rig, profiles snapshot into
    // and load from it, and the sticky status file lands beside it.
    const std::filesystem::path cfgDir = std::filesystem::temp_directory_path() / "nullcat-http-contract";
    std::filesystem::remove_all(cfgDir);
    std::filesystem::create_directories(cfgDir);
    { Config seed; seed.load((cfgDir / "config.json").string()); }   // cold start writes host.json + rig.json
    // The limiter routed to axis 1 in the saved rig, so a Test of it gets
    // as far as the live check (the Test reads the saved rig).
    {
        QFile f(QString::fromStdString((cfgDir / "rig.json").string()));
        if (f.open(QIODevice::ReadOnly))
        {
            QJsonObject root = QJsonDocument::fromJson(f.readAll()).object();
            f.close();
            QJsonObject g = root.value("global").toObject(), h = g.value("haptics").toObject();
            QJsonObject lim = h.value("limiter").toObject();
            QJsonArray routes; { QJsonObject ro; ro["axis"] = 0; ro["gain"] = 0.5; routes.append(ro); }
            lim["ampPct"] = 30.0; lim["routes"] = routes;
            h["limiter"] = lim; g["haptics"] = h; root["global"] = g;
            if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) { f.write(QJsonDocument(root).toJson()); f.close(); }
        }
    }
    // A two-car shipped table beside it, as the mains find cars.json
    // beside the executable.
    {
        std::ofstream t(cfgDir / "cars.json");
        t << "{\"games\":{\"ac\":\"AssettoCorsa\"},\"cars\":{"
             "\"ac:ks_mazda_787b\":{\"name\":\"Mazda 787B\",\"engine\":{\"ampPct\":45,\"cylinders\":4,\"layout\":3},"
             "\"limiter\":{\"ampPct\":35},\"driveline\":{\"ampPct\":35,\"gearbox\":1},\"abs\":{\"ampPct\":0},\"tc\":{\"ampPct\":0},"
             "\"notes\":\"four-rotor\"},"
             "\"ac:ks_ferrari_f40\":{\"name\":\"Ferrari F40\",\"engine\":{\"ampPct\":46,\"cylinders\":8,\"turbo\":1},"
             "\"limiter\":{\"ampPct\":30},\"driveline\":{\"ampPct\":30},\"abs\":{\"ampPct\":0},\"tc\":{\"ampPct\":0},\"notes\":\"twin turbo\"}}}";
    }
    // The shipped starting points beside it too, the file itself.
    std::error_code presetsErr;
    std::filesystem::copy_file(std::filesystem::path(NULLCAT_SOURCE_DIR) / "resources" / "presets.json",
                               cfgDir / "presets.json", std::filesystem::copy_options::overwrite_existing, presetsErr);

    WebServer web;
    web.setComponents(&motion, &loop, &master, &cfg);
    web.setTelemetry(&telemetry);
    web.setConfigPath((cfgDir / "config.json").string());
    web.setCarTablePath((cfgDir / "cars.json").string());
    web.setPresetsPath((cfgDir / "presets.json").string());
    web.setPort(PORT);
    web.setBindAddr(HOST);
    web.setOnStopRequested([&loop]() { if (loop.isRunning()) loop.stop(); });

    if (!web.start())
    {
        std::printf("[FAIL] WebServer failed to start on %s:%d\n", HOST, PORT);
        return 1;
    }
    check(waitUp(PORT, 10000), "WebServer answers on its port");

    // ---- surface up before engine: status + meta respond ----
    check(has(get("/api/status"), "\"loopRunning\""), "GET /api/status responds pre-init");
    // The hand-built status must be JSON whatever the numbers and strings
    // hold: a non-finite number prints as 0 and a control character in a
    // string is escaped (reverting either in WebServer::jsonDouble /
    // jsonStr fails the unit rows; "nan" or a raw tab in the text made the
    // page's poll loop swallow the whole status).
    {
        QJsonParseError pe;
        const std::string st = get("/api/status");
        QJsonDocument::fromJson(QByteArray(st.c_str(), (int)st.size()), &pe);
        check(pe.error == QJsonParseError::NoError, "GET /api/status is parseable JSON");
        check(WebServer::jsonDouble(std::nan(""), 3) == "0.000" && WebServer::jsonDouble(INFINITY, 1) == "0.0"
              && WebServer::jsonDouble(-INFINITY, 0) == "0" && WebServer::jsonDouble(1.5, 2) == "1.50",
              "jsonDouble: nan and inf print as 0, finite values as before");
        check(WebServer::jsonStr("a\tb\x01" "c\"d\\e\n") == "\"a\\tb\\u0001c\\\"d\\\\e\\n\"",
              "jsonStr: control characters, quote and backslash escaped");
        const std::string esc = WebServer::jsonStr("\x1f" "x\x7f");
        QJsonParseError pe2;
        const QJsonDocument back = QJsonDocument::fromJson(QByteArray(("[" + esc + "]").c_str()), &pe2);
        check(pe2.error == QJsonParseError::NoError && back.array().at(0).toString() == QString::fromUtf8("\x1f" "x\x7f"),
              "jsonStr: every escape parses back to the same text");
    }
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

    // ---- haptics surface: schema, Test by key with reasons, mute ----
    // The schema is the effect table (HapticsRegistry.h) the browser builds
    // its tiles from; Test refuses with a reason against the SAVED config
    // (every effect ships off, so amp 0 is the first refusal); mute is a
    // runtime command that always succeeds.
    check(has(get("/api/haptics/schema"), "\"key\":\"rpmVibe\"") && has(get("/api/haptics/schema"), "\"opts\""),
          "GET /api/haptics/schema lists the effects with their tunables");
    // The schema also carries the channel token registry (the page's only
    // copy), the wire's slot count and the route parts of the slip tiles.
    check(has(get("/api/haptics/schema"), "\"tokens\":[\"rpm\",\"speedKmh\"")
          && has(get("/api/haptics/schema"), "\"suspVelRR\"")
          && has(get("/api/haptics/schema"), "\"maxSlots\":96")
          && has(get("/api/haptics/schema"), "\"parts\":[\"all\",\"front\",\"rear\",\"fl\",\"fr\",\"rl\",\"rr\"]")
          && has(get("/api/haptics/schema"), "\"key\":\"slipLat\"")
          && has(get("/api/haptics/schema"), "\"label\":\"Lateral slip\"")
          && has(get("/api/haptics/schema"), "\"kind\":\"slip\"") && has(get("/api/haptics/schema"), "\"kind\":\"kerb\""),
          "GET /api/haptics/schema carries tokens, maxSlots, parts and the slip and kerb kinds");
    // Every tile's defaults: each key it offers and the keys every tile
    // carries (peakPct was missing), equal to what a fresh config saves.
    {
        const QJsonArray fx = QJsonDocument::fromJson(QByteArray::fromStdString(get("/api/haptics/schema"))).object().value("effects").toArray();
        const QJsonObject fresh = Config::writeHapticsObject(AppConfig{});
        bool all = fx.size() == haptics::EFFECT_COUNT;
        std::string bad;
        for (const QJsonValue& v : fx)
        {
            const QJsonObject e = v.toObject();
            const QJsonObject d = e.value("defaults").toObject();
            QJsonObject want = fresh.value(e.value("key").toString()).toObject();
            want.remove("routes");
            if (d != want) { all = false; bad += e.value("key").toString().toStdString() + " "; }
            for (const QJsonValue& p : e.value("params").toArray())
                if (!d.contains(p.toObject().value("key").toString())) { all = false; bad += e.value("key").toString().toStdString() + "." + p.toObject().value("key").toString().toStdString() + " "; }
            if (!d.contains("peakPct")) { all = false; bad += e.value("key").toString().toStdString() + ".peakPct "; }
        }
        check(all, ("GET /api/haptics/schema: every tile's defaults are a fresh config's, every offered key and peakPct included " + bad).c_str());
    }
    check(has(post("/api/haptics/test", "{\"effect\":\"slipLat\"}"), "amplitude is 0"),
          "POST /api/haptics/test: a saved effect at amp 0 is refused with the reason");
    {
        // Routed, but the loop is not running: the refusal names each
        // destination with its state and the one thing to do (it used to
        // tell a parked seat axis to "engage the device or tension the belt").
        const std::string r = post("/api/haptics/test", "{\"effect\":\"limiter\"}");
        check(has(r, "\"ok\":false") && has(r, ": loop not running") && has(r, "start the loop")
              && !has(r, "tension the belt") && !has(r, "engage the device"),
              ("POST /api/haptics/test: routed with the loop stopped, the refusal names the state and only the fix it needs " + r).c_str());
    }
    check(has(post("/api/haptics/test", "{\"effect\":\"skid\"}"), "Unknown effect"),
          "POST /api/haptics/test: the retired skid key is not an API name");
    check(has(post("/api/haptics/test", "{\"effect\":\"nope\"}"), "Unknown effect"),
          "POST /api/haptics/test: an unknown key is refused");
    check(has(post("/api/haptics/mute", "{\"on\":true}"), "\"ok\":true")
          && has(post("/api/haptics/mute", "{\"on\":false}"), "\"ok\":true"),
          "POST /api/haptics/mute on/off accepted");
    check(has(post("/api/haptics/mute", "{\"effect\":\"driveline\",\"on\":true}"), "\"ok\":true")
          && has(post("/api/haptics/mute", "{\"effect\":\"driveline\",\"on\":false}"), "\"ok\":true")
          && has(post("/api/haptics/mute", "{\"effect\":\"nope\",\"on\":true}"), "Unknown effect"),
          "POST /api/haptics/mute one tile accepted, an unknown tile refused");
    check(has(get("/api/status"), "\"hapMute\":[false,"), "GET /api/status carries the tiles' mute list");
    check(has(post("/api/haptics/mute", "{\"effect\":\"driveline\",\"on\":1}"), "\"ok\":true")
          && has(post("/api/haptics/mute", "{\"effect\":\"driveline\",\"on\":0}"), "\"ok\":true"),
          "POST /api/haptics/mute takes 1/0 as well as true/false");
    // A body without a boolean or numeric "on" is refused at the HTTP
    // layer (it used to fall back to a substring search for "true", so
    // "garbage" un-muted the rig). Whether it also changed nothing is read
    // back once the loop publishes the mute state, below.
    check(has(post("/api/haptics/mute", "garbage"), "\"ok\":false")
          && has(post("/api/haptics/mute", "{}"), "\"ok\":false")
          && has(post("/api/haptics/mute", "{\"on\":\"yes\"}"), "\"ok\":false")
          && has(post("/api/haptics/mute", "{\"effect\":\"driveline\"}"), "\"ok\":false"),
          "POST /api/haptics/mute refuses a body without a boolean or numeric on");
    {
        // The settings that follow the car, per tile, from the registry's
        // car flag (in registry order, so read as a set): the Driveline
        // tile's include its amplitude, jitter and clutch, the Engine
        // tile's its cylinders but never the buzz order (the rig's: a PC
        // wants about 0.75, the Pi 0, whatever the car), and a tile the car
        // table never touches has none.
        const QJsonArray fx = QJsonDocument::fromJson(QByteArray::fromStdString(get("/api/haptics/schema")))
                                  .object().value("effects").toArray();
        bool drive = false, engine = false, none = false;
        for (const QJsonValue& v : fx)
        {
            const QJsonObject e = v.toObject();
            const QJsonArray ck = e.value("carKeys").toArray();
            const QString key = e.value("key").toString();
            if (key == "driveline")
                drive = ck.contains(QJsonValue("ampPct")) && ck.contains(QJsonValue("jitter"))
                        && ck.contains(QJsonValue("clutch"));
            if (key == "rpmVibe")
                engine = ck.contains(QJsonValue("cylinders")) && !ck.contains(QJsonValue("order"));
            if (e.contains("carKeys") && ck.isEmpty()) none = true;
        }
        check(drive && engine && none && has(get("/api/haptics/schema"), "\"id\":0"),
              "GET /api/haptics/schema marks the settings that follow the car, per tile, the buzz order not among them");
    }
    {
        // The Wheels tile's fresh tyres on pit/restart: a switch with its
        // hover text, on by default, and the rig's (not the car's).
        const QJsonArray fx = QJsonDocument::fromJson(QByteArray::fromStdString(get("/api/haptics/schema")))
                                  .object().value("effects").toArray();
        bool sw = false, on = false, rig = false;
        for (const QJsonValue& v : fx)
        {
            const QJsonObject e = v.toObject();
            if (e.value("key").toString() != "wheels") continue;
            for (const QJsonValue& p : e.value("params").toArray())
            {
                const QJsonObject o = p.toObject();
                if (o.value("key").toString() == "freshOnReset")
                    sw = o.value("opts").toArray().size() == 2 && !o.value("help").toString().isEmpty();
            }
            on  = e.value("defaults").toObject().value("freshOnReset").toDouble() == 1.0;
            rig = e.contains("carKeys") && !e.value("carKeys").toArray().contains(QJsonValue("freshOnReset"));
        }
        check(sw && on && rig,
              "GET /api/haptics/schema: the Wheels tile's fresh tyres on pit/restart is a switch, on by default, the rig's");
    }
    {
        // ...and the ones a type of car (a starting point) sets, from the
        // shipped presets.json: the Lateral tile's slide size, nothing on
        // the Engine tile (the car's), never a car-owned key.
        const QJsonArray fx = QJsonDocument::fromJson(QByteArray::fromStdString(get("/api/haptics/schema")))
                                  .object().value("effects").toArray();
        bool latPeak = false, engineNone = false, disjoint = true;
        for (const QJsonValue& v : fx)
        {
            const QJsonObject e = v.toObject();
            const QJsonArray tk = e.value("typeKeys").toArray(), ck = e.value("carKeys").toArray();
            if (e.value("key").toString() == "slipLat") latPeak = tk.contains(QJsonValue("peakDeg"));
            if (e.value("key").toString() == "rpmVibe") engineNone = e.contains("typeKeys") && tk.isEmpty();
            for (const QJsonValue& k : tk) if (ck.contains(k)) disjoint = false;
        }
        check(latPeak && engineNone && disjoint,
              "GET /api/haptics/schema marks the settings a starting point sets (typeKeys), none of them the car's");
    }
    // Sticky per-sim effect record: readable empty, clearable, and the
    // status carries the dots for the current game in registry order.
    check(has(get("/api/haptics/status"), "\"games\":[") && has(get("/api/haptics/status"), "\"effects\":{"),
          "GET /api/haptics/status returns the games list and the per-effect record");
    check(has(post("/api/haptics/status/clear", "{\"all\":true}"), "\"ok\":true"),
          "POST /api/haptics/status/clear accepted");
    {
        // ...and read back: no game remembered, every dot grey.
        std::string dots = "\"hapDots\":[";
        for (int i = 0; i < haptics::EFFECT_COUNT; ++i) dots += (i ? ",0" : "0");
        dots += "]";
        check(has(get("/api/haptics/status"), "\"games\":[]") && has(get("/api/status"), dots.c_str()),
              "status/clear read back: no games remembered, every sim dot grey");
    }
    check(has(get("/api/status"), "\"hapDots\":[") && has(get("/api/status"), "\"hapGame\":")
          && has(get("/api/status"), "\"ncxFrozen\":false") && has(get("/api/status"), "\"hapProfile\":"),
          "GET /api/status carries hapDots, hapGame, hapProfile and ncxFrozen");
    // Tile readouts: driven levels with peaks, the three per-wheel sets
    // (lateral, longitudinal, road mm) now and at peak, channel values
    // and peaks in token order.
    {
        const std::string st = get("/api/status");
        check(has(st, "\"hapIn\":[") && has(st, "\"hapInPk\":[")
              && has(st, "\"hapWheels\":[[") && has(st, "\"hapWheelsPk\":[[")
              && has(st, "\"ncxVal\":[") && has(st, "\"ncxPk\":["),
              "GET /api/status carries the tile readouts (hapIn/Pk, hapWheels/Pk, ncxVal/Pk)");
        // Each tile's triggers (none yet: 0, and -1 for the seconds since)
        // and the road heights' rate, the whole still one JSON object.
        check(has(st, "\"hapHeightsHz\":") && has(st, "\"hapCount\":[0,0,") && has(st, "\"hapAgo\":[-1.0,")
              && QJsonDocument::fromJson(QByteArray::fromStdString(st)).isObject(),
              "GET /api/status carries each tile's trigger count and seconds since (none yet) and the heights' rate, as valid JSON");
        check(has(post("/api/resetstats", "{}"), "\"ok\":true"), "POST /api/resetstats accepted (clears the haptic peaks too)");
        check(has(st, "\"hapLatSrc\":") && has(st, "\"hapLatLearned\":") && has(st, "\"hapAbs\":{") && has(st, "\"record\":{"),
              "GET /api/status carries the lateral source and baseline state, ABS's source and the recording");
        check(has(post("/api/haptics/relearn", "{}"), "\"ok\":true"), "POST /api/haptics/relearn accepted (the lateral baseline starts over)");
        check(has(post("/api/haptics/fresh-tyres", "{}"), "\"ok\":true"), "POST /api/haptics/fresh-tyres accepted (flat spots and hot discs gone)");
    }
    // Shakers (off in this host config): the status says so, the device list
    // answers, and the tone test is refused with the reason.
    check(has(get("/api/status"), "\"shakers\":{\"enabled\":false"),
          "GET /api/status reports the shaker sink state");
    check(has(get("/api/shakers/devices"), "\"devices\":["),
          "GET /api/shakers/devices lists playback devices (possibly none)");
    check(has(post("/api/shakers/test", "{\"channel\":0}"), "No shaker output is open"),
          "POST /api/shakers/test refuses with a reason when shakers are off");
    check(has(post("/api/shakers/reopen", "{}"), "\"ok\":false") && has(post("/api/shakers/reopen", "{}"), "Shakers are off"),
          "POST /api/shakers/reopen refuses with a reason when shakers are off");
    // Device presets: a save lands, and a devicepresets.json that is there
    // but cannot be read (cut short) is refused and left as it is: it used
    // to read as empty, and the save wrote one preset over all the others.
    {
        const std::filesystem::path dp = cfgDir / "devicepresets.json";
        check(has(post("/api/devpresets", "{\"name\":\"Mine\",\"device\":{\"a\":1}}"), "\"ok\":true")
              && has(get("/api/devpresets"), "\"Mine\""), "POST /api/devpresets saves a preset, GET lists it");
        const std::string cut = "{\"presets\":{\"Mine\":{\"a\":1}";
        { std::ofstream t(dp, std::ios::binary | std::ios::trunc); t << cut; }
        const std::string r = post("/api/devpresets", "{\"name\":\"New\",\"device\":{\"b\":2}}");
        std::string after;
        { std::ifstream i(dp, std::ios::binary); after.assign(std::istreambuf_iterator<char>(i), std::istreambuf_iterator<char>()); }
        check(has(r, "\"ok\":false") && has(r, "cannot be read") && after == cut,
              "POST /api/devpresets over an unreadable file: refused with the reason, the file untouched");
        std::filesystem::remove(dp);
    }
    // Host settings: the page posts only the fields it shows. Every other
    // host.json key (the GPIO pins, the e-stop pin among them, the allowed
    // host names, the loop CPU) stays as it was: a save used to replace the
    // file with the page's object, and those went back to their defaults.
    {
        const std::filesystem::path hp = cfgDir / "host.json";
        const auto readText = [](const std::filesystem::path& p)
        { std::ifstream i(p, std::ios::binary); return std::string(std::istreambuf_iterator<char>(i), std::istreambuf_iterator<char>()); };
        const std::string original = readText(hp);
        QJsonObject h = QJsonDocument::fromJson(QByteArray::fromStdString(original)).object();
        h["gpioEstopPin"] = 5;
        h["rtCore"] = 7;
        QJsonArray allowed; allowed.append(QStringLiteral("nullcat.local")); h["webAllowedHosts"] = allowed;
        { std::ofstream o(hp, std::ios::binary | std::ios::trunc); o << QJsonDocument(h).toJson().toStdString(); }
        const std::string r = post("/api/host", "{\"configVersion\":2,\"nicName\":\"eth9\",\"logMinLevel\":\"info\"}");
        const QJsonObject after = QJsonDocument::fromJson(QByteArray::fromStdString(readText(hp))).object();
        check(has(r, "\"ok\":true") && after.value("nicName").toString() == "eth9" && after.value("logMinLevel").toString() == "info",
              "POST /api/host: the page's fields land");
        check(after.value("gpioEstopPin").toInt() == 5 && after.value("rtCore").toInt() == 7
              && after.value("webAllowedHosts").toArray().size() == 1,
              "POST /api/host: every host key the page does not send stays (e-stop pin, loop CPU, allowed hosts)");
        { std::ofstream o(hp, std::ios::binary | std::ios::trunc); o << original; }   // as it was for the rest of the run
    }
    // The dashboard's Download logs carries host.json with the web password
    // blanked (a bundle is made to be shared), escaped quote and all.
    {
        const std::filesystem::path hp = cfgDir / "host.json";
        const auto readText = [](const std::filesystem::path& p)
        { std::ifstream i(p, std::ios::binary); return std::string(std::istreambuf_iterator<char>(i), std::istreambuf_iterator<char>()); };
        const std::string original = readText(hp);
        QJsonObject h = QJsonDocument::fromJson(QByteArray::fromStdString(original)).object();
        h["webAuthToken"] = QStringLiteral("open\"sesame");
        { std::ofstream o(hp, std::ios::binary | std::ios::trunc); o << QJsonDocument(h).toJson().toStdString(); }
        const std::string b = get("/api/logbundle");
        check(has(b, "==== host.json ====") && has(b, "<redacted>") && !has(b, "sesame"),
              "GET /api/logbundle: host.json is in, its web password blanked");
        // A key written with an escape (a hand edit) is the same key to the
        // parser: the file is left out rather than shared.
        { std::ofstream o(hp, std::ios::binary | std::ios::trunc); o << "{\"web\\u0041uthToken\": \"hidden1\"}"; }
        const std::string b2 = get("/api/logbundle");
        check(has(b2, "==== host.json ====") && has(b2, "(left out") && !has(b2, "hidden1"),
              "GET /api/logbundle: a password under an escaped key name leaves host.json out");
        { std::ofstream o(hp, std::ios::binary | std::ios::trunc); o << original; }
    }
    // Profiles: save snapshots the saved set, it lists, loads (the saved
    // rig's haptics object is the snapshot again, an edit made in between
    // undone), binds to a car name (naming that car loads it), and
    // deletes; an unknown name is refused.
    int genAfterProfile = -1;
    {
        const auto hapticsOf = [](const std::string& body) -> QJsonObject
        {
            QJsonObject h = QJsonDocument::fromJson(QByteArray::fromStdString(body)).object()
                                .value("global").toObject().value("haptics").toObject();
            h.remove("followCar");   // the rig's, never a profile's
            return h;
        };
        const QJsonObject snapshot = hapticsOf(get("/api/rig"));
        check(has(post("/api/haptics/profiles/save", "{\"name\":\"Bench\"}"), "\"ok\":true"),
              "POST /api/haptics/profiles/save snapshots the saved haptics set");
        check(has(get("/api/haptics/profiles"), "\"profiles\":[\"Bench\"]") && has(get("/api/haptics/profiles"), "\"active\":\"Bench\""),
              "GET /api/haptics/profiles lists it as active");
        // An edit after the snapshot, then the load: the file is the snapshot again.
        {
            std::string body; int gen = -1;
            check(getRig(body, gen), "GET /api/rig for an edit after the snapshot");
            const Reply r = postHdr("/api/rig", rigWith(body, nullptr, "masterGain", 0.6), {{"X-Nullcat-Gen", std::to_string(gen)}});
            check(r.status == 200 && hapticsOf(get("/api/rig")) != snapshot, "the edit (master gain 0.6) lands and differs from the snapshot");
        }
        const int genBefore = genNow();
        check(has(post("/api/haptics/profiles/load", "{\"name\":\"Bench\"}"), "\"ok\":true"),
              "POST /api/haptics/profiles/load applies it");
        check(hapticsOf(get("/api/rig")) == snapshot && has(get("/api/status"), "\"hapProfile\":\"Bench\""),
              "profile load read back: the saved haptics object is the snapshot again, the status names it");
        check(genNow() == genBefore + 1, "a profile load moves the apply generation (a page refreshes its copy)");
        genAfterProfile = genNow();
        check(has(post("/api/haptics/profiles/load", "{\"name\":\"nope\"}"), "\"ok\":false"),
              "POST /api/haptics/profiles/load refuses an unknown name");
        check(has(post("/api/haptics/profiles/bind", "{\"name\":\"Bench\",\"car\":\"Test Car\"}"), "\"key\":\"car:Test Car\""),
              "POST /api/haptics/profiles/bind ties a car to it");
        check(has(get("/api/haptics/profiles"), "\"car:Test Car\":\"Bench\""),
              "GET /api/haptics/profiles shows the binding");
        // A second profile, so Bench is not the active one: whether the
        // binding loads it when the sim names the bound car is checked
        // after the car table rows below (naming a car sets the car row).
        check(has(post("/api/haptics/profiles/save", "{\"name\":\"Other\"}"), "\"ok\":true")
              && has(get("/api/haptics/profiles"), "\"active\":\"Other\""),
              "a second profile saved is the active one");
    }
    // Car table: the shipped file beside the config lists, an entry pulled
    // by key lands in the saved rig (five tiles only, routes untouched),
    // the status carries the car fields, and with no car named by the sim
    // the current-car actions refuse with a reason.
    {
        check(has(get("/api/cars"), "\"stock\":2") && has(get("/api/cars"), "\"follow\":true")
              && has(get("/api/cars"), "\"car\":\"\""),
              "GET /api/cars reports the table size, the follow switch and no current car");
        check(has(get("/api/cars/list"), "\"key\":\"ac:ks_ferrari_f40\"") && has(get("/api/cars/list"), "\"name\":\"Mazda 787B\"")
              && has(get("/api/cars/list"), "\"source\":\"stock\""),
              "GET /api/cars/list carries every entry with key, name and layer");
        const std::string st = get("/api/status");
        // The apply generation has not moved since the Bench profile load
        // above (a rewrite of the live set like a pull is).
        check(has(st, "\"hapCar\":\"\"") && has(st, "\"hapCarKey\":\"\"") && has(st, "\"hapCarSrc\":\"\"")
              && has(st, "\"hapCarApplied\":false") && has(st, "\"hapCarGen\":") && genNow() == genAfterProfile,
              "GET /api/status carries the car table fields (hapCar/Key/Src/Applied/Gen, where the profile load left it)");
        const int gen0 = genNow();
        check(has(post("/api/cars/pull", "{\"key\":\"ac:ks_ferrari_f40\"}"), "\"name\":\"Ferrari F40\""),
              "POST /api/cars/pull applies an entry by key");
        // /api/rig is the file itself (indented, a space after each colon).
        std::string rig = get("/api/rig");
        rig.erase(std::remove(rig.begin(), rig.end(), ' '), rig.end());
        check(has(rig, "\"cylinders\":8") && has(rig, "\"turbo\":1") && has(rig, "\"followCar\":true"),
              "the pulled entry is in the saved rig (engine description) and the follow switch stayed");
        check(genNow() == gen0 + 1, "GET /api/status hapCarGen bumped on the apply");
        check(has(post("/api/cars/pull", "{\"key\":\"ac:nope\"}"), "\"ok\":false"),
              "POST /api/cars/pull refuses an unknown key");
        check(has(post("/api/cars/pull", "{}"), "has not named a car"),
              "POST /api/cars/pull with no car named by the sim refuses with the reason");
        check(has(post("/api/cars/save", "{}"), "has not named a car"),
              "POST /api/cars/save with no car named by the sim refuses with the reason");
        check(has(post("/api/cars/forget", "{\"key\":\"ac:ks_ferrari_f40\"}"), "\"ok\":false"),
              "POST /api/cars/forget refuses when there is no entry of yours");
        check(has(st, "\"hapCarDefault\":false") && has(get("/api/cars"), "\"defaults\":false"),
              "GET /api/status and /api/cars say whether the car tiles are on the defaults");
    }
    // A page saves the copy of the rig it loaded, with the apply
    // generation that copy belongs to (X-Nullcat-Gen from GET /api/rig).
    // A save against a stale generation (the F40 pull above rewrote the
    // haptics after an older copy was loaded) is refused with 409 and
    // nothing is written; the current generation saves. Reverting the
    // X-Nullcat-Gen check in POST /api/rig makes the first save land (200)
    // and the "untouched" row fail. A request without the header (a
    // script) is taken as is.
    {
        std::string body; int gen = -1;
        check(getRig(body, gen) && gen >= 1 && gen == genNow(),
              "GET /api/rig carries X-Nullcat-Gen equal to the status's hapCarGen");
        const std::string edited = rigWith(body, nullptr, "masterGain", 0.9);
        const Reply stale = postHdr("/api/rig", edited, {{"X-Nullcat-Gen", std::to_string(gen - 1)}});
        check(stale.status == 409 && has(stale.body, "\"ok\":false") && has(stale.body, "nothing was saved"),
              "POST /api/rig against a stale generation: 409 with the reason");
        check(!has(compact(get("/api/rig")), "\"masterGain\":0.9"), "...and the file is untouched");
        const Reply fresh = postHdr("/api/rig", edited, {{"X-Nullcat-Gen", std::to_string(gen)}});
        check(fresh.status == 200 && has(fresh.body, "\"ok\":true") && has(fresh.body, "\"changed\":true"),
              "POST /api/rig against the current generation: saved");
        check(has(compact(get("/api/rig")), "\"masterGain\":0.9"), "...and the file carries the edit");
        check(genNow() == gen + 1 && has(fresh.body, ("\"gen\":" + std::to_string(gen + 1)).c_str()),
              "a web save that changed the file moves the generation, and the reply carries the new one");
        // A second page that loaded the same copy before that save: its save
        // would write the first page's edit away, so it is refused too.
        const Reply other = postHdr("/api/rig", rigWith(body, nullptr, "masterGain", 0.7), {{"X-Nullcat-Gen", std::to_string(gen)}});
        check(other.status == 409 && has(compact(get("/api/rig")), "\"masterGain\":0.9"),
              "a second page saving the copy it loaded before another page's save: 409, the first save stands");
        const Reply bare = postHdr("/api/rig", rigWith(body, nullptr, "masterGain", 0.8), {});
        check(bare.status == 200 && has(bare.body, "\"ok\":true") && has(compact(get("/api/rig")), "\"masterGain\":0.8"),
              "POST /api/rig without the header (a script) is taken as is");
    }
    // Shaker shaping: the schema comes from ShakerShape.h; the live preview
    // refuses with a reason while shakers are off; a save that changes only
    // global.shakerShaping applies live (no Initialize asked for) and the
    // file carries it; one out of range is refused, naming the shaker.
    // Taking shakerShaping out of rigDiffIsLiveOnly's strip fails the live
    // check here (and the belt block's first check below).
    {
        const std::string sc = get("/api/shakers/schema");
        check(has(sc, "\"bands\":5") && has(sc, "\"types\":[\"highpass\",\"lowshelf\",\"bell\",\"highshelf\",\"lowpass\"]") && has(sc, "\"default\":{"),
              "GET /api/shakers/schema: five bands, the band types, the flat default");
        check(has(post("/api/shakers/shape", "{\"channel\":0,\"shape\":{}}"), "Shakers are off"),
              "POST /api/shakers/shape refuses with a reason when shakers are off");
        const auto withShaping = [](const std::string& b, double ratio)
        {
            QJsonObject root = QJsonDocument::fromJson(QByteArray::fromStdString(b)).object();
            QJsonObject g = root.value("global").toObject();
            QJsonObject comp; comp["on"] = true; comp["ratio"] = ratio;
            QJsonObject e; e["shaker"] = 1; e["comp"] = comp;
            QJsonArray arr; arr.append(e); g["shakerShaping"] = arr; root["global"] = g;
            return QJsonDocument(root).toJson().toStdString();
        };
        std::string body; int gen = -1;
        check(getRig(body, gen), "GET /api/rig for the shaping edits");
        const Reply bad = postHdr("/api/rig", withShaping(body, 99.0), {{"X-Nullcat-Gen", std::to_string(gen)}});
        check(bad.status == 400 && has(bad.body, "shakerShaping for shaker 1"),
              "a save with shaping out of range is refused, naming the shaker");
        const Reply ok = postHdr("/api/rig", withShaping(body, 4.0), {{"X-Nullcat-Gen", std::to_string(gen)}});
        check(ok.status == 200 && has(ok.body, "\"changed\":true") && has(ok.body, "\"needsInit\":false")
              && has(get("/api/meta"), "\"rigPendingRestart\":false"),
              "a save that changes only the shaker shaping applies live: no Initialize asked for");
        check(has(compact(get("/api/rig")), "\"shakerShaping\":[{"), "...and rig.json carries it");
    }
    // A belt's g shape applies live on save (staged with the motion
    // controller); its torque range still waits for an Initialize. Taking
    // the beltG keys out of the live set makes the first row fail.
    {
        const auto axisWith = [](const std::string& b, const char* key, double v)
        {
            QJsonObject root = QJsonDocument::fromJson(QByteArray::fromStdString(b)).object();
            QJsonArray axes = root.value("axes").toArray();
            QJsonObject a = axes.at(0).toObject(); a[key] = v; axes.replace(0, a);
            root["axes"] = axes;
            return QJsonDocument(root).toJson().toStdString();
        };
        std::string body; int gen = -1;
        check(getRig(body, gen), "GET /api/rig for the belt g edits");
        const Reply g = postHdr("/api/rig", axisWith(body, "beltGFullG", 2.0), {{"X-Nullcat-Gen", std::to_string(gen)}});
        check(g.status == 200 && has(g.body, "\"changed\":true") && has(g.body, "\"needsInit\":false")
              && has(get("/api/meta"), "\"rigPendingRestart\":false"),
              "a save that moves only a belt's g at max applies live: no Initialize asked for");
        check(getRig(body, gen), "GET /api/rig again");
        const Reply t = postHdr("/api/rig", axisWith(body, "torqueMaxPct", 40.0), {{"X-Nullcat-Gen", std::to_string(gen)}});
        check(t.status == 200 && has(t.body, "\"needsInit\":true"),
              "a save that moves the belt's torque max still asks for an Initialize");
    }
    // Starting points: the shipped file beside the config lists its four
    // types in the file's order; apply refuses an unknown name and merges a
    // known one into the saved rig (its own settings only: the engine the
    // F40 pull left, the follow switch and the routes stay), bumping the
    // apply generation so every page refreshes.
    {
        check(!presetsErr, "the shipped resources/presets.json is staged beside the config");
        const std::string ps = get("/api/haptics/presets");
        const size_t a = ps.find("\"name\":\"Road car\""), b = ps.find("\"name\":\"Cup car\""),
                     c = ps.find("\"name\":\"GT3 / GT\""), d = ps.find("\"name\":\"Open wheeler\""),
                     e = ps.find("\"name\":\"Road car (AMS2)\""), f = ps.find("\"name\":\"Open wheeler (AMS2)\"");
        check(a != std::string::npos && b != std::string::npos && c != std::string::npos && d != std::string::npos
              && e != std::string::npos && f != std::string::npos
              && a < b && b < c && c < d && d < e && e < f && has(ps, "\"description\":\"Street car on road tyres"),
              "GET /api/haptics/presets lists the starting points in order (the four types, then for Automobilista 2), with their descriptions");
        check(has(post("/api/haptics/presets/apply", "{\"name\":\"nope\"}"), "\"ok\":false"),
              "POST /api/haptics/presets/apply refuses an unknown name");
        check(has(post("/api/haptics/presets/apply", "{}"), "\"ok\":false"),
              "POST /api/haptics/presets/apply refuses no name");
        const int gen0 = genNow();
        check(has(post("/api/haptics/presets/apply", "{\"name\":\"Open wheeler\"}"), "\"ok\":true"),
              "POST /api/haptics/presets/apply applies a known one");
        std::string rig = get("/api/rig");
        rig.erase(std::remove(rig.begin(), rig.end(), ' '), rig.end());
        check(has(rig, "\"peakDeg\":5.5") && has(rig, "\"fullMm\":18") && has(rig, "\"cylinders\":8")
              && has(rig, "\"turbo\":1") && has(rig, "\"followCar\":true"),
              "the starting point is in the saved rig; the pulled car's engine and the follow switch stayed");
        check(genNow() == gen0 + 1, "GET /api/status hapCarGen bumped on the starting point");
        // The same starting point again changes nothing: no write, and the
        // generation stays (it used to move and refuse a page's save, 409,
        // for nothing).
        check(has(post("/api/haptics/presets/apply", "{\"name\":\"Open wheeler\"}"), "\"ok\":true") && genNow() == gen0 + 1,
              "POST /api/haptics/presets/apply of the one already applied: accepted, the generation stays");
    }
    // Follow car: a car the table has no entry for puts the car tiles on
    // the defaults (not the last car's: the F40 pulled above), the status
    // saying so, in a game the table has nothing for (BeamNG: an Assetto
    // Corsa car without ABS left its ABS off) as in one it covers; a name
    // that starts with an entry's lands on that entry as a near match.
    // Named the way a plugin does it, a NULLCATY line, fed straight in
    // while the status is polled. Breaks with the defaults kept to the
    // games the table covers.
    {
        // Each car waited for by its name first: two cars in a row on the
        // defaults read alike, so the flag alone would not say the second
        // has been taken.
        check(nameCar(telemetry, "iRacing", "Some Car", "\"hapCar\":\"Some Car\"", 10000)
              && waitStatus("\"hapCarDefault\":true", 5000),
              "follow car: a car in a game the table has nothing for gets the defaults too (status hapCarDefault)");
        {
            std::string r = get("/api/rig");
            r.erase(std::remove(r.begin(), r.end(), ' '), r.end());
            check(!has(r, "\"cylinders\":8") && has(r, "\"followCar\":true"),
                  "...the defaults replaced the last car's engine (the F40's) in the saved rig");
        }
        check(nameCar(telemetry, "AssettoCorsa", "Mystery Car", "\"hapCar\":\"Mystery Car\"", 10000)
              && waitStatus("\"hapCarDefault\":true", 5000),
              "follow car: a car with no entry puts the car tiles on the defaults (status hapCarDefault)");
        std::string rig = get("/api/rig");
        rig.erase(std::remove(rig.begin(), rig.end(), ' '), rig.end());
        check(!has(rig, "\"cylinders\":8") && has(rig, "\"peakDeg\":5.5") && has(rig, "\"followCar\":true"),
              "the defaults replaced the last car's engine in the saved rig; the starting point and the switch stayed");
        const std::string cars = get("/api/cars");
        check(has(cars, "\"car\":\"Mystery Car\"") && has(cars, "\"source\":\"\"") && has(cars, "\"defaults\":true"),
              "GET /api/cars: the car has no entry and the defaults are in use");
        // Save for this car files the saved five tiles under the current
        // car in the user layer (cars.local.json): the entry is then the
        // car's own, applied, and listed; forget drops it and the row
        // shows no entry again (the tiles keep what they carried).
        check(has(post("/api/cars/save", "{\"name\":\"My Mystery\",\"notes\":\"mine\"}"), "\"key\":\"ac:Mystery Car\""),
              "POST /api/cars/save files an entry under the current car");
        check(std::filesystem::exists(cfgDir / "cars.local.json"), "...in cars.local.json beside the config");
        {
            const std::string c2 = get("/api/cars");
            check(has(c2, "\"source\":\"user\"") && has(c2, "\"name\":\"My Mystery\"") && has(c2, "\"notes\":\"mine\"")
                  && has(c2, "\"applied\":true") && has(c2, "\"defaults\":false") && has(c2, "\"user\":1"),
                  "GET /api/cars: the saved entry is the car's own, applied, not the defaults");
            check(has(get("/api/cars/list"), "\"name\":\"My Mystery\"") && has(get("/api/cars/list"), "\"source\":\"user\""),
                  "GET /api/cars/list carries it as yours");
            check(has(get("/api/status"), "\"hapCarSrc\":\"user\"") && has(get("/api/status"), "\"hapCarApplied\":true"),
                  "GET /api/status carries the saved entry");
        }
        check(has(post("/api/cars/forget", "{}"), "\"ok\":true"), "POST /api/cars/forget drops the entry of yours");
        {
            const std::string c3 = get("/api/cars");
            check(has(c3, "\"source\":\"\"") && has(c3, "\"applied\":false") && has(c3, "\"defaults\":false") && has(c3, "\"user\":0")
                  && !has(get("/api/cars/list"), "\"name\":\"My Mystery\""),
                  "GET /api/cars after forget: no entry, not applied, not the defaults; the list no longer carries it");
        }
        check(nameCar(telemetry, "AssettoCorsa", "Mazda 787B Le Mans", "\"hapCarKey\":\"ac:ks_mazda_787b\"", 10000)
              && has(get("/api/cars"), "\"near\":true") && has(get("/api/status"), "\"hapCarDefault\":false")
              && has(get("/api/status"), "\"hapCarApplied\":true"),
              "follow car: a name that starts with an entry's is a near match, applied");
        // A car on the defaults whose sender describes it (protocol 1.11,
        // the BeamNG mod): its engine goes on the engine tile and its type
        // of car is applied as the starting point, once: the same car
        // described otherwise later changes nothing. Breaks with
        // senderCarFacts not called, or applying every time.
        {
            const auto describe = [&](const char* cyl, const char* want) {
                for (int t = 0, rpm = 3000; t < 10000; t += 100, rpm += 7)
                {
                    const std::string line = std::string("NULLCATY,game=BeamNG.drive,car=bastion sport_M,carId=bastion/sport_M,engCylinders=")
                        + cyl + ",engLayout=1,engLitres=5.2,engTurbo=0,engLimitHz=7,carType=1,rpm=" + std::to_string(rpm);
                    TelemetryData td;
                    if (TelemetryInput::parsePacket(line.c_str(), static_cast<int>(line.size()), td)) telemetry.ingest(td);
                    if (has(get("/api/cars"), want)) return true;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                return false; };
            check(describe("8", "\"described\":true") && has(get("/api/cars"), "\"typed\":true") && has(get("/api/cars"), "\"defaults\":true"),
                  "follow car: a car on the defaults takes what its sender says about it (GET /api/cars described, typed)");
            std::string r = get("/api/rig");
            r.erase(std::remove(r.begin(), r.end(), ' '), r.end());
            check(has(r, "\"cylinders\":8") && has(r, "\"layout\":1") && has(r, "\"litres\":5.2") && has(r, "\"limHz\":7"),
                  "...its engine on the engine tile (8 cylinders, V, 5.2 litres, limiter 7 hz)");
            check(has(r, "\"beltFullG\":1.4"), "...and its type of car (cup) applied as the starting point (belt g at max 1.4)");
            describe("6", "\"typed\":true");
            std::this_thread::sleep_for(std::chrono::milliseconds(1200));   // two status samples
            std::string r2 = get("/api/rig");
            r2.erase(std::remove(r2.begin(), r2.end(), ' '), r2.end());
            check(has(r2, "\"cylinders\":8") && !has(r2, "\"cylinders\":6"), "...once: the same car described otherwise later changes nothing");
        }
        // A save that moves a setting the car table owns (the engine's amp)
        // means the tiles no longer carry the entry: the car row stops
        // claiming it. One that moves nothing the car owns (the master
        // gain) leaves the claim. Reverting the carOwnedKeysChanged block
        // in POST /api/rig leaves hapCarApplied true after the first save.
        {
            std::string body; int gen = -1;
            check(getRig(body, gen), "GET /api/rig for the car-owned edit");
            const Reply r = postHdr("/api/rig", rigWith(body, "rpmVibe", "ampPct", 33.0), {{"X-Nullcat-Gen", std::to_string(gen)}});
            check(r.status == 200 && has(r.body, "\"ok\":true"), "a save moving the engine's amp (car-owned) lands");
            check(has(get("/api/status"), "\"hapCarApplied\":false") && has(get("/api/cars"), "\"applied\":false"),
                  "...and the car row no longer claims the entry is in force");
            check(has(post("/api/cars/pull", "{}"), "\"ok\":true") && has(get("/api/status"), "\"hapCarApplied\":true"),
                  "pull car preset: the entry is in force again");
            // A profile replaces the whole set, the car's tiles included.
            check(has(post("/api/haptics/profiles/load", "{\"name\":\"Other\"}"), "\"ok\":true")
                  && has(get("/api/status"), "\"hapCarApplied\":false"),
                  "a profile load: the car row no longer claims the entry is in force");
            check(has(post("/api/cars/pull", "{}"), "\"ok\":true") && has(get("/api/status"), "\"hapCarApplied\":true"),
                  "...and a pull puts it back in force");
            check(getRig(body, gen), "GET /api/rig for the rig-owned edit");
            const Reply r2 = postHdr("/api/rig", rigWith(body, nullptr, "masterGain", 0.7), {{"X-Nullcat-Gen", std::to_string(gen)}});
            check(r2.status == 200 && has(r2.body, "\"ok\":true") && has(get("/api/status"), "\"hapCarApplied\":true"),
                  "a save moving only the master gain leaves the claim");
        }
        // Follow car off: a covered car with an entry arriving is matched
        // for the row but nothing is applied: the generation and the file
        // stay as they are. Then on again.
        {
            std::string body; int gen = -1;
            check(getRig(body, gen), "GET /api/rig to untick follow car");
            const Reply off = postHdr("/api/rig", rigWithBool(body, "followCar", false), {{"X-Nullcat-Gen", std::to_string(gen)}});
            check(off.status == 200 && has(off.body, "\"ok\":true") && has(get("/api/cars"), "\"follow\":false"),
                  "follow car unticked and saved: /api/cars reports follow:false");
            const std::string before = get("/api/rig");
            const int genOff = genNow();
            check(nameCar(telemetry, "AssettoCorsa", "Ferrari F40", "\"hapCarKey\":\"ac:ks_ferrari_f40\"", 10000),
                  "follow car off: a covered car with an entry is matched for the row");
            std::this_thread::sleep_for(std::chrono::milliseconds(600));
            check(genNow() == genOff && get("/api/rig") == before && has(get("/api/status"), "\"hapCarApplied\":false")
                  && has(get("/api/status"), "\"hapCarDefault\":false"),
                  "...but nothing is applied: hapCarGen unchanged, /api/rig untouched, the row says not applied");
            check(getRig(body, gen), "GET /api/rig to tick follow car again");
            const Reply on = postHdr("/api/rig", rigWithBool(body, "followCar", true), {{"X-Nullcat-Gen", std::to_string(gen)}});
            check(on.status == 200 && has(on.body, "\"ok\":true") && has(get("/api/cars"), "\"follow\":true"),
                  "follow car ticked again");
        }
        // The profile binding fires: with Other active, the sim naming the
        // bound Test Car (iRacing: no car of the table's) loads Bench by
        // itself; then both profiles go, the binding with them.
        check(nameCar(telemetry, "iRacing", "Test Car", "\"hapProfile\":\"Bench\"", 10000),
              "the sim naming the bound car loads its profile (status hapProfile)");
        check(has(post("/api/haptics/profiles/delete", "{\"name\":\"Bench\"}"), "\"ok\":true")
              && has(post("/api/haptics/profiles/delete", "{\"name\":\"Other\"}"), "\"ok\":true")
              && has(get("/api/haptics/profiles"), "\"profiles\":[]") && !has(get("/api/haptics/profiles"), "car:Test Car"),
              "POST /api/haptics/profiles/delete forgets them and the binding");
        // The 787B again (a car change: the table acted on none since),
        // so the restart check at the end finds it remembered.
        check(nameCar(telemetry, "AssettoCorsa", "Mazda 787B Le Mans", "\"hapCarApplied\":true", 10000),
              "the near match applied again after the other cars");
        std::this_thread::sleep_for(std::chrono::milliseconds(700));   // the stream goes quiet before the loop starts
    }

    // ---- init (sim) -> masterOp ----
    check(has(post("/api/init"), "\"ok\":true"), "POST /api/init accepted");
    check(waitStatus("\"masterOp\":true", 15000), "status reaches masterOp:true");

    // ---- start loop -> auto-home -> Option A: belt stays SLACK ----
    check(has(post("/api/start"), "\"ok\":true"), "POST /api/start accepted");
    check(waitStatus("\"loopRunning\":true", 5000), "status reaches loopRunning:true");
    check(waitStatus("\"hasBelts\":true", 2000),    "status: hasBelts:true");

    // ---- mutes read back (the loop publishes hapMuted / hapMute[]) ----
    // The master mute lands; a garbage body, {} and a string "on" are
    // refused AND leave it muted (reverting the "on" type check in the
    // mute handler un-mutes on "garbage": the old fallback found no "true"
    // and sent on=false); one tile's mute lands in hapMute[] at its
    // registry index (driveline = 10); unmute brings everything back.
    {
        check(has(post("/api/haptics/mute", "{\"on\":true}"), "\"ok\":true") && waitStatus("\"hapMuted\":true", 3000),
              "mute on: status reads hapMuted:true");
        check(muteAt(get("/api/status"), 10) == 1, "mute on: every tile muted (driveline's hapMute[10] true)");
        check(has(post("/api/haptics/mute", "garbage"), "\"ok\":false")
              && has(post("/api/haptics/mute", "{}"), "\"ok\":false")
              && has(post("/api/haptics/mute", "{\"on\":\"yes\"}"), "\"ok\":false"),
              "garbage mute bodies refused while muted");
        std::this_thread::sleep_for(std::chrono::milliseconds(400));   // several publishes
        check(has(get("/api/status"), "\"hapMuted\":true") && muteAt(get("/api/status"), 10) == 1,
              "garbage mute bodies changed NOTHING: still muted");
        check(has(post("/api/haptics/mute", "{\"on\":false}"), "\"ok\":true") && waitStatus("\"hapMuted\":false", 3000),
              "mute off: status reads hapMuted:false");
        check(muteAt(get("/api/status"), 10) == 0, "mute off: the tiles are back (hapMute[10] false)");
        check(has(post("/api/haptics/mute", "{\"effect\":\"driveline\",\"on\":true}"), "\"ok\":true"),
              "one tile's mute accepted");
        bool tileMuted = false;
        for (int t = 0; t < 3000 && !tileMuted; t += 50)
        {
            tileMuted = (muteAt(get("/api/status"), 10) == 1);
            if (!tileMuted) std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        check(tileMuted && has(get("/api/status"), "\"hapMuted\":false") && muteAt(get("/api/status"), 9) == 0,
              "one tile's mute reads back in hapMute[] alone (master and the kerb tile unmuted)");
        check(has(post("/api/haptics/mute", "{\"effect\":\"driveline\",\"on\":0}"), "\"ok\":true"), "one tile's unmute accepted");
        bool tileBack = false;
        for (int t = 0; t < 3000 && !tileBack; t += 50)
        {
            tileBack = (muteAt(get("/api/status"), 10) == 0);
            if (!tileBack) std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        check(tileBack, "one tile's unmute reads back");
    }
    // ---- relearn and fresh tyres read back (the loop publishes them) ----
    // Nothing drives here, so the fields can only read as reset: the
    // lateral baseline at 0 s and not learned after relearn; the Wheels
    // tile's flat spots (hapWheels set 4) all 0 after fresh tyres.
    {
        check(has(post("/api/haptics/relearn", "{}"), "\"ok\":true") && waitStatus("\"hapLatBaseSec\":0.0", 3000)
              && has(get("/api/status"), "\"hapLatLearned\":false"),
              "relearn read back: the lateral baseline at 0 s, not learned");
        const auto wheelSet = [](const std::string& st, int k) -> std::string
        {
            size_t p = st.find("\"hapWheels\":[");
            if (p == std::string::npos) return std::string();
            p += 13;
            for (int i = 0; i < k; ++i) { p = st.find('[', p + 1); if (p == std::string::npos) return std::string(); }
            const size_t e = st.find(']', p);
            return (e == std::string::npos) ? std::string() : st.substr(p, e - p + 1);
        };
        check(has(post("/api/haptics/fresh-tyres", "{}"), "\"ok\":true"), "POST /api/haptics/fresh-tyres accepted");
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        check(wheelSet(get("/api/status"), 4) == "[0.000,0.000,0.000,0.000]",
              "fresh tyres read back: every flat spot 0 (hapWheels set 4)");
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));   // let auto-home/unpark finish
    check(has(get("/api/status"), "\"beltsSlack\":true"),
          "Option A: belt still SLACK after start/auto-unpark");

    // ---- explicit tension is the one belt trigger ----
    check(has(post("/api/belts/tension"), "\"ok\":true"), "POST /api/belts/tension accepted");
    check(waitStatus("\"beltsSlack\":false", 5000), "tension: beltsSlack goes false");

    // ---- 0.9.5 device endpoints: surface shape ----
    // This rig has no device axis, so accept = enqueue and the RT-side
    // family filter no-ops -- what these rows pin is the ROUTE (it exists,
    // it answers) and the HTTP-side axis validation. The engine-layer
    // behaviour rows live in TestCommandContract.
    check(has(post("/api/device/engage"), "\"ok\":true"),
          "POST /api/device/engage accepted");
    check(has(post("/api/device/release"), "\"ok\":true"),
          "POST /api/device/release accepted");
    check(has(post("/api/device/engage", "{\"axis\":99}"), "\"ok\":false"),
          "POST /api/device/engage axis out of range: refused");
    check(has(post("/api/device-toggle"), "No device axes"),
          "POST /api/device-toggle on a device-less rig: visible refusal");
    check(has(get("/api/status"), "\"hasDevices\":false"),
          "status: hasDevices false on a device-less rig");

    // ---- toggle endpoints (one button per pair; server-resolved + guarded) ----
    // Runs BEFORE the e-stop section, which trips the rig and walks it back
    // through Stop EtherCAT and Initialize. Here everything is settled.
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


    // ---- direct telemetry: the rig's choice reads back; junk is refused ----
    check(has(post("/api/telemetry/direct", "not json"), "\"ok\":false")
          && has(post("/api/telemetry/direct", "{\"on\":\"yes\"}"), "\"ok\":false"),
          "direct telemetry: a body without a boolean \"on\" is refused");
    check(has(get("/api/status"), "\"directWant\":\"\""), "direct telemetry: the refusals changed nothing (left to the sender)");
    check(has(post("/api/telemetry/direct", "{\"on\":true}"), "\"direct\":\"on\"")
          && has(get("/api/status"), "\"directWant\":\"on\""),
          "direct telemetry on: reads back");
    check(has(post("/api/telemetry/direct", "{\"on\":false}"), "\"direct\":\"off\"")
          && has(get("/api/status"), "\"directWant\":\"off\""),
          "direct telemetry off: reads back");
    check(has(get("/api/status"), "\"directSender\":\""), "status carries what the sender says");
    check(has(get("/api/status"), "\"directSetAside\":false"), "status carries whether another sender is set aside (none here)");
    // The kerbs channel by name (a game's own kerb signal) reads true; the
    // numbered line's slot (0 in Assetto Corsa whatever the game has) does
    // not count. Breaks with curbsNamed read from the merged channels.
    {
        const auto feed = [&](const char* line) {
            TelemetryData td;
            if (TelemetryInput::parsePacket(line, static_cast<int>(std::strlen(line)), td)) telemetry.ingest(td); };
        feed("NULLCATX,4000,100,3,0,50,0,0,0,0,0,0,0,0,7500");
        check(has(get("/api/status"), "\"curbsNamed\":false"), "status: a numbered line's kerbs slot is no kerb channel of the game's");
        feed("NULLCATY,curbs=40,rpm=4000");
        check(has(get("/api/status"), "\"curbsNamed\":true"), "status: the kerbs channel by name is the game's own");
    }

    // ---- e-stop; tension ACCEPTED but silently refused (contract fact #1/#2) ----
    check(has(post("/api/estop"), "\"ok\":true"), "POST /api/estop accepted");
    check(waitStatus("\"estop\":true", 3000), "status: estop:true");
    check(waitStatus("\"beltsSlack\":true", 5000), "e-stop slacks the belt");
    check(has(post("/api/belts/tension"), "\"ok\":true"),
          "tension under e-stop: ok:true (ACCEPTED, not done)");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    check(has(get("/api/status"), "\"beltsSlack\":true"),
          "tension under e-stop: belt STAYS slack (silent engine refusal)");

    // ---- the trip names what stopped the rig and when ----
    {
        const std::string st = get("/api/status");
        check(has(st, "\"estopInput\":true") && has(st, "\"estopTripped\":true")
              && has(st, "\"estopSource\":\"dashboard e-stop\"") && has(st, "\"estopTripAgoSec\":"),
              "status: the trip carries its source and age");
    }

    // ---- release frees the input only: the trip holds ----
    check(has(post("/api/estop/release"), "\"ok\":true"), "POST /api/estop/release accepted");
    check(waitStatus("\"estopInput\":false", 3000), "status: estopInput:false after release");
    {
        const std::string st = get("/api/status");
        check(has(st, "\"estop\":true") && has(st, "\"estopTripped\":true"),
              "release alone: still tripped (estop:true)");
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));   // nothing may move in this window
    check(has(get("/api/status"), "\"beltsSlack\":true"),
          "belt still slack after e-stop release");
    check(has(post("/api/belts/tension"), "\"ok\":true"),
          "tension after release: ok:true (ACCEPTED, the engine refuses)");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    check(has(get("/api/status"), "\"beltsSlack\":true"),
          "tension after release, before re-initialize: belt STAYS slack");
    check(has(post("/api/belts-toggle"), "re-initialize"),
          "belts-toggle while tripped: visible refusal naming the fix");

    // ---- a stopped loop will not start while tripped ----
    check(has(post("/api/stop"), "\"ok\":true"), "POST /api/stop accepted (tripped)");
    check(waitStatus("\"loopRunning\":false", 10000), "tripped: the loop stops");
    check(has(post("/api/start"), "re-initialize"), "start while tripped: refused, names the fix");
    check(has(post("/api/run-toggle"), "re-initialize"), "run-toggle while tripped: refused, names the fix");
    check(has(get("/api/status"), "\"loopRunning\":false"), "the refused starts left the loop stopped");

    // ---- only a successful re-initialize clears it ----
    check(has(post("/api/deinit"), "\"ok\":true"), "POST /api/deinit accepted (tripped)");
    check(waitStatus("\"masterOp\":false", 15000), "tripped: masterOp false");
    check(has(get("/api/status"), "\"estopTripped\":true"), "Stop EtherCAT alone does not clear the trip");
    check(has(post("/api/init"), "\"ok\":true"), "re-initialize accepted");
    check(waitStatus("\"masterOp\":true", 15000), "re-initialize: masterOp true");
    check(waitStatus("\"estopTripped\":false", 3000) && has(get("/api/status"), "\"estop\":false"),
          "a successful re-initialize clears the trip");
    check(has(post("/api/start"), "\"ok\":true"), "start after re-initialize: accepted");
    check(waitStatus("\"loopRunning\":true", 5000), "start after re-initialize: loop runs");

    // ---- the cleared trip restores permission, not state (Option A) ----
    std::this_thread::sleep_for(std::chrono::seconds(2));   // auto-home window
    check(has(get("/api/status"), "\"beltsSlack\":true"),
          "Option A: belt still slack after the re-initialize");
    check(has(post("/api/belts/tension"), "\"ok\":true"), "re-tension accepted after re-initialize");
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

    // ---- optional web auth: a set token gates every /api/*; static exempt ----
    // The token is BOOT-TIME config in production (host.json, read once,
    // restart to change), so the test mirrors that exactly: a second server
    // instance born with the token set, on its own port. (An earlier version
    // mutated the shared cfg while the first server's threads were live - a
    // data race x86 forgave and arm64 rightly did not.)
    {
        AppConfig acfg = makeCfg();
        acfg.webAuthToken = "sesame";
        WebServer aweb;
        aweb.setComponents(&motion, &loop, &master, &acfg);
        aweb.setPort(PORT + 1);
        aweb.setBindAddr(HOST);
        if (!aweb.start())
        {
            check(false, "auth-enabled WebServer starts");
        }
        else
        {
            check(waitUp(PORT + 1, 10000), "auth-enabled WebServer answers on its port");
            httplib::Client cli(HOST, PORT + 1);
            cli.set_read_timeout(2, 0);
            auto r1 = cli.Get("/api/status");
            check(r1 && r1->status == 401 && has(r1->body, "auth required"),
                  "auth on: GET /api/* without token is 401");
            auto r2 = cli.Get("/api/status", httplib::Headers{{"X-Nullcat-Auth", "sesame"}});
            check(r2 && r2->status == 200 && has(r2->body, "\"loopRunning\""),
                  "auth on: correct token passes");
            auto r3 = cli.Get("/api/status", httplib::Headers{{"X-Nullcat-Auth", "sesamE"}});
            check(r3 && r3->status == 401, "auth on: wrong token is 401");
            auto r4 = cli.Post("/api/stop", "", "application/json");
            check(r4 && r4->status == 401, "auth on: POST without token is 401");
            auto r5 = cli.Get("/");
            check(r5 && r5->status != 401, "auth on: static page stays reachable (prompt can render)");
            aweb.stop();
        }
    }
    check(has(get("/api/status"), "\"loopRunning\""),
          "no-token server (shipped default) stays open throughout");

    // ---- a restart is no new car ----
    // The car the table last acted on, and whether its tiles carry its
    // entry or the defaults, is remembered in carstate.json beside the
    // config. A second server over the same directory (what a restart is),
    // fed the same car, applies nothing: hapCarGen stays where it started,
    // the rig file is untouched (the hand-set engine amp stays) and the row
    // reads as remembered; a different car is a car change as ever.
    // Reverting the memory (carIdentity's "same car" branch reading
    // m_carMem, loaded by loadCarMemory) re-applies the near-match entry on
    // the second server: the generation moves and the engine's amp goes back
    // to the entry's 45. The restarted server's generation is its own, so a
    // page's save from before the restart is refused (it used to restart at
    // 0, where an old page's copy could match it).
    {
        std::string body; int gen = -1;
        check(getRig(body, gen), "GET /api/rig before the restart");
        const Reply r = postHdr("/api/rig", rigWith(body, "rpmVibe", "ampPct", 37.0), {{"X-Nullcat-Gen", std::to_string(gen)}});
        check(r.status == 200 && has(r.body, "\"ok\":true"), "a hand-set engine amp (37) saved before the restart");
        check(waitStatus("\"hapCarApplied\":false", 2000), "...which clears the row's applied");
        const int genOld = genNow();
        web.stop();
        check(std::filesystem::exists(cfgDir / "carstate.json"), "carstate.json is beside the config");

        WebServer web2;
        web2.setComponents(&motion, &loop, &master, &cfg);
        web2.setTelemetry(&telemetry);
        web2.setConfigPath((cfgDir / "config.json").string());
        web2.setCarTablePath((cfgDir / "cars.json").string());
        web2.setPresetsPath((cfgDir / "presets.json").string());
        web2.setPort(PORT);
        web2.setBindAddr(HOST);
        if (!web2.start()) check(false, "the second server (a restart) starts");
        else
        {
            check(waitUp(PORT, 10000), "the second server (a restart) answers on the port");
            const int gen2 = genNow();
            check(gen2 > 0 && gen2 != genOld && has(get("/api/status"), "\"hapCar\":\"\""),
                  "the restarted server starts with no car and a generation of its own (not the old run's)");
            const Reply oldPage = postHdr("/api/rig", rigWith(body, nullptr, "masterGain", 0.5), {{"X-Nullcat-Gen", std::to_string(genOld)}});
            check(oldPage.status == 409 && !has(compact(get("/api/rig")), "\"masterGain\":0.5"),
                  "a page's save carrying the generation from before the restart: 409, nothing written");
            check(nameCar(telemetry, "AssettoCorsa", "Mazda 787B Le Mans", "\"hapCarKey\":\"ac:ks_mazda_787b\"", 10000),
                  "the same car after the restart is matched again");
            std::this_thread::sleep_for(std::chrono::milliseconds(600));   // two samples: an apply would have landed
            check(genNow() == gen2, "...but nothing is applied: hapCarGen stays where it started");
            check(has(compact(get("/api/rig")), "\"ampPct\":37") && has(get("/api/status"), "\"hapCarApplied\":false")
                  && has(get("/api/status"), "\"hapCarDefault\":false"),
                  "...the hand-set amp stays and the row reads as remembered (not applied)");
            check(nameCar(telemetry, "AssettoCorsa", "Ferrari F40", "\"hapCarApplied\":true", 10000) && genNow() == gen2 + 1,
                  "a different car after the restart is applied as ever");

            // ---- direct telemetry across the restart, and end to end ----
            // The choice (off, set before the restart) is remembered; the
            // sender is asked, from the telemetry port to the port its
            // identity came from, while it reports otherwise, at most about
            // once a second, and never while it agrees or has none.
            check(has(get("/api/status"), "\"directWant\":\"off\""),
                  "direct telemetry: the choice made before the restart is remembered (senderstate.json)");
            {
                const int c = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
                timeval tmo{0, 300000};
                setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, &tmo, sizeof(tmo));
                sockaddr_in me{};
                me.sin_family = AF_INET;
                me.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
                bind(c, reinterpret_cast<sockaddr*>(&me), sizeof(me));
                sockaddr_in to{};
                to.sin_family = AF_INET;
                to.sin_port = htons(static_cast<uint16_t>(cfg.telemetryPort));
                to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
                // The loop is stopped here: the test drains the socket.
                const auto say = [&](const char* direct)
                {
                    const std::string l = std::string("NULLCATY,game=AssettoCorsa,car=Ferrari F40,direct=") + direct;
                    sendto(c, l.data(), l.size(), 0, reinterpret_cast<sockaddr*>(&to), sizeof(to));
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    while (telemetry.receive()) {}
                };
                // Asks heard over a window, the sender saying `direct` throughout.
                const auto asksOver = [&](const char* direct, int ms, std::string& last)
                {
                    int asks = 0;
                    for (int t = 0; t < ms; t += 320)
                    {
                        say(direct);
                        char b[128];
                        const ssize_t n = recv(c, b, sizeof(b) - 1, 0);
                        if (n > 0) { ++asks; last.assign(b, static_cast<size_t>(n)); }
                    }
                    return asks;
                };
                std::string last;
                say("off");
                check(has(post("/api/telemetry/direct", "{\"on\":true}"), "\"direct\":\"on\""), "direct telemetry on (end to end)");
                const int asked = asksOver("off", 3000, last);
                check(asked >= 2 && asked <= 5 && last == "NULLCATC,direct=on\n",
                      "the sender reporting off is asked for on, about once a second");
                check(has(get("/api/status"), "\"directSender\":\"off\""), "status: what the sender says");
                asksOver("on", 700, last);   // it switched: any ask still in flight lands here
                check(asksOver("on", 2000, last) == 0, "the sender reporting on (as asked): never asked");
                // A second sender's channel line while this one says on:
                // set aside, and the status says so (it reads false before).
                {
                    check(has(get("/api/status"), "\"directSetAside\":false"), "status: no sender set aside yet");
                    const int c2 = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
                    bind(c2, reinterpret_cast<sockaddr*>(&me), sizeof(me));
                    const std::string l = "NULLCATY,game=BeamNgDrive,car=scintilla,rpm=1234,direct=none";
                    sendto(c2, l.data(), l.size(), 0, reinterpret_cast<sockaddr*>(&to), sizeof(to));
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    while (telemetry.receive()) {}
                    check(has(get("/api/status"), "\"directSetAside\":true") && has(get("/api/status"), "\"directSender\":\"on\""),
                          "status: another sender's line set aside while the direct one sends, and the direct one still reported");
                    close(c2);
                }
                check(asksOver("none", 2000, last) == 0, "a sender with no direct source for the game: never asked");
                check(has(post("/api/telemetry/direct", "{\"on\":false}"), "\"direct\":\"off\""), "direct telemetry off (end to end)");
                check(asksOver("on", 2000, last) >= 1 && last == "NULLCATC,direct=off\n",
                      "the sender reporting on is asked for off");
                // A sender that is only direct (a game's own mod) cannot
                // switch: the switch off never asks it, and the status says
                // what it is.
                asksOver("only", 700, last);   // an ask still in flight lands here
                check(asksOver("only", 2000, last) == 0 && has(get("/api/status"), "\"directSender\":\"only\""),
                      "a sender saying only is never asked, with the switch off; the status reads only");
                close(c);
            }
            web2.stop();
        }
    }

    // ---- teardown ----
    if (loop.isRunning()) loop.stop();
    web.stop();
    master.shutdown();
    std::filesystem::remove_all(cfgDir);

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
