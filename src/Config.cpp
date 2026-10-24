// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// Config.cpp - configuration schema, persistence, validation.
//
// All schema knowledge lives in this file. load() and save() are
// driven from the same canonical field list so they can never
// diverge.
//
// Public interface (Config.h) is Qt-free. Qt JSON is an internal
// implementation detail.
// ============================================================
#include "Config.h"
#include "AxisKind.h"
#include "TelemetryInput.h"   // MAX_NCX_CHANNELS
#include <cstring>

#include <QString>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include <QHash>
#include <QSaveFile>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <thread>

// ---------- helpers ----------------------------------------------------------

static std::string toLower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return s;
}

// Reduction ratio is stored as "N:1" (motor revs per output rev), e.g. "2:1"
// or "1.5:1". Returns N as a factor; anything unparseable falls back to 1.0.
// A gearbox of N:1 means the motor turns N times per output (ballscrew) rev,
// so there are N x encoderCountsPerRev counts per output rev -> per pitch mm.
static double reductionFactor(const std::string& ratio)
{
    auto colon = ratio.find(':');
    std::string lead = (colon == std::string::npos) ? ratio : ratio.substr(0, colon);
    try { double v = std::stod(lead); return (v > 0.0) ? v : 1.0; }
    catch (...) { return 1.0; }
}

// ---- host.json: per-machine fields (NIC, network, timing, services, FK, GPIO) ----
// The host namespace travels with the controlling machine, not the rig. On PC
// it is Qt-owned; on the headless Pi it is web-owned. Single writer per platform.
static void writeHostConfig(const AppConfig& c, QJsonObject& obj)
{
    obj["configVersion"]            = c.configVersion;
    obj["nicName"]                  = QString::fromStdString(c.nicName);
    obj["controlLoopHz"]            = c.controlLoopHz;
    obj["rtPinCore"]                = c.rtPinCore;
    obj["rtCore"]                   = c.rtCore;
    obj["asyncSend"]                = c.asyncSend;
    obj["tempPollSec"]              = c.tempPollSec;
    obj["telemetryPort"]               = c.telemetryPort;
    obj["telemetryBindAddr"]           = QString::fromStdString(c.telemetryBindAddr);
    obj["ncxFrozenMs"]              = c.ncxFrozenMs;
    obj["webPort"]                  = c.webPort;
    obj["webBindAddr"]              = QString::fromStdString(c.webBindAddr);
    obj["webAuthToken"]             = QString::fromStdString(c.webAuthToken);
    { QJsonArray a; for (const auto& h : c.webAllowedHosts) a.append(QString::fromStdString(h));
      obj["webAllowedHosts"] = a; }
    obj["webUIEnabled"]             = c.webUIEnabled;
    obj["webShowDevices"]           = c.webShowDevices;
    obj["shakersEnabled"]           = c.shakersEnabled;
    obj["audioDevice"]              = QString::fromStdString(c.audioDevice);
    obj["audioChannels"]            = c.audioChannels;
    if (!c.audioDevices.empty())
    {
        QJsonArray devs;
        for (const AppConfig::AudioDevice& d : c.audioDevices)
        { QJsonObject o; o["device"] = QString::fromStdString(d.device); o["channels"] = d.channels; devs.append(o); }
        obj["audioDevices"] = devs;
    }
    obj["dcSyncOffsetNs"]           = c.dcSyncOffsetNs;
    obj["dcPhaseLockEnabled"]       = c.dcPhaseLockEnabled;
    obj["dcPhaseLockKp"]            = c.dcPhaseLockKp;
    obj["dcPhaseLockKi"]            = c.dcPhaseLockKi;
    obj["dcPhaseLockMaxTrimNs"]     = c.dcPhaseLockMaxTrimNs;
    obj["pdoWatchdogMs"]            = c.pdoWatchdogMs;
    obj["enableCapabilityScan"]     = c.enableCapabilityScan;
    obj["commandSyncCycles"]        = c.commandSyncCycles;
    obj["sync0RecycleRounds"]       = c.sync0RecycleRounds;
    obj["wkcValidationCycles"]      = c.wkcValidationCycles;
    obj["wkcValidationThreshold"]   = c.wkcValidationThreshold;
    obj["logFile"]                  = QString::fromStdString(c.logFile);
    obj["logToConsole"]             = c.logToConsole;
    obj["simulationMode"]           = c.simulationMode;
    obj["logMinLevel"]              = QString::fromStdString(c.logMinLevel);
    obj["diagEnabled"]              = c.diagEnabled;
    obj["foregroundKeeperEnabled"]  = c.foregroundKeeperEnabled;
    obj["foregroundKeeperAlpha"]    = c.foregroundKeeperAlpha;
    obj["foregroundKeeperX"]        = c.foregroundKeeperX;
    obj["foregroundKeeperY"]        = c.foregroundKeeperY;
    obj["gpioMode"]                 = QString::fromStdString(c.gpioMode);
    obj["gpioEnabled"]              = c.gpioEnabled;
    obj["gpioChip"]                 = QString::fromStdString(c.gpioChip);
    obj["gpioEstopPin"]             = c.gpioEstopPin;
    obj["gpioEngagePin"]            = c.gpioEngagePin;
    obj["gpioParkPin"]              = c.gpioParkPin;
    obj["gpioBeltPin"]              = c.gpioBeltPin;
    obj["gpioDevicePin"]            = c.gpioDevicePin;
    obj["gpioLedRunPin"]            = c.gpioLedRunPin;
    obj["gpioLedReadyPin"]          = c.gpioLedReadyPin;
    obj["gpioLedFaultPin"]          = c.gpioLedFaultPin;
}

static void readHostConfig(const QJsonObject& obj, AppConfig& c)
{
    if (obj.contains("nicName"))                  c.nicName                  = obj.value("nicName").toString().toStdString();
    if (obj.contains("controlLoopHz"))            c.controlLoopHz            = obj.value("controlLoopHz").toInt(500);
    if (obj.contains("rtPinCore"))                c.rtPinCore                = obj.value("rtPinCore").toBool(true);
    if (obj.contains("rtCore"))                   c.rtCore                   = obj.value("rtCore").toInt(-1);
    if (obj.contains("asyncSend"))                c.asyncSend                = obj.value("asyncSend").toBool(true);
    if (obj.contains("tempPollSec"))              c.tempPollSec              = obj.value("tempPollSec").toDouble(15.0);
    if (obj.contains("telemetryPort"))               c.telemetryPort               = obj.value("telemetryPort").toInt(4444);
    if (obj.contains("telemetryBindAddr"))           c.telemetryBindAddr           = obj.value("telemetryBindAddr").toString("").toStdString();  // "" = platform default
    if (obj.contains("ncxFrozenMs"))              c.ncxFrozenMs              = obj.value("ncxFrozenMs").toInt(0);
    if (obj.contains("webPort"))                  c.webPort                  = obj.value("webPort").toInt(8080);
    if (obj.contains("webBindAddr"))              c.webBindAddr              = obj.value("webBindAddr").toString("127.0.0.1").toStdString();
    if (obj.contains("webAuthToken"))             c.webAuthToken             = obj.value("webAuthToken").toString("").toStdString();  // "" = auth off
    if (obj.contains("webAllowedHosts"))
    {
        c.webAllowedHosts.clear();
        for (const auto& v : obj.value("webAllowedHosts").toArray())
            if (!v.toString().isEmpty()) c.webAllowedHosts.push_back(v.toString().toStdString());
    }
    if (obj.contains("webUIEnabled"))             c.webUIEnabled             = obj.value("webUIEnabled").toBool(false);
    if (obj.contains("webShowDevices"))           c.webShowDevices           = obj.value("webShowDevices").toBool(false);
    if (obj.contains("shakersEnabled"))           c.shakersEnabled           = obj.value("shakersEnabled").toBool(false);
    if (obj.contains("audioDevice"))              c.audioDevice              = obj.value("audioDevice").toString("").toStdString();
    if (obj.contains("audioChannels"))            c.audioChannels            = obj.value("audioChannels").toInt(2);
    if (obj.contains("audioDevices"))
    {
        c.audioDevices.clear();
        for (const QJsonValue& v : obj.value("audioDevices").toArray())
        {
            const QJsonObject o = v.toObject();
            AppConfig::AudioDevice d;
            d.device   = o.value("device").toString("").toStdString();
            d.channels = o.value("channels").toInt(2);
            c.audioDevices.push_back(d);
        }
    }
    if (obj.contains("dcSyncOffsetNs"))           c.dcSyncOffsetNs           = obj.value("dcSyncOffsetNs").toInt(0);
    if (obj.contains("dcPhaseLockEnabled"))       c.dcPhaseLockEnabled       = obj.value("dcPhaseLockEnabled").toBool(false);
    if (obj.contains("dcPhaseLockKp"))            c.dcPhaseLockKp            = obj.value("dcPhaseLockKp").toDouble(2.5);
    if (obj.contains("dcPhaseLockKi"))            c.dcPhaseLockKi            = obj.value("dcPhaseLockKi").toDouble(1.6);
    if (obj.contains("dcPhaseLockMaxTrimNs"))     c.dcPhaseLockMaxTrimNs     = obj.value("dcPhaseLockMaxTrimNs").toInt(10000);
    if (obj.contains("pdoWatchdogMs"))            c.pdoWatchdogMs            = obj.value("pdoWatchdogMs").toInt(100);
    if (obj.contains("enableCapabilityScan"))     c.enableCapabilityScan     = obj.value("enableCapabilityScan").toBool(false);
    if (obj.contains("commandSyncCycles"))        c.commandSyncCycles        = obj.value("commandSyncCycles").toInt(10);
    if (obj.contains("sync0RecycleRounds"))       c.sync0RecycleRounds       = obj.value("sync0RecycleRounds").toInt(2);
    if (obj.contains("wkcValidationCycles"))      c.wkcValidationCycles      = obj.value("wkcValidationCycles").toInt(50);
    if (obj.contains("wkcValidationThreshold"))   c.wkcValidationThreshold   = obj.value("wkcValidationThreshold").toDouble(0.9);
    if (obj.contains("logFile"))                  c.logFile                  = obj.value("logFile").toString("logs/app.log").toStdString();
    if (obj.contains("logToConsole"))             c.logToConsole             = obj.value("logToConsole").toBool(true);
    if (obj.contains("simulationMode"))           c.simulationMode           = obj.value("simulationMode").toBool(false);
    if (obj.contains("logMinLevel"))              c.logMinLevel              = obj.value("logMinLevel").toString("debug").toStdString();
    if (obj.contains("diagEnabled"))              c.diagEnabled              = obj.value("diagEnabled").toBool(true);
    if (obj.contains("foregroundKeeperEnabled"))  c.foregroundKeeperEnabled  = obj.value("foregroundKeeperEnabled").toBool(true);
    if (obj.contains("foregroundKeeperAlpha"))    c.foregroundKeeperAlpha    = obj.value("foregroundKeeperAlpha").toInt(255);
    if (obj.contains("foregroundKeeperX"))        c.foregroundKeeperX        = obj.value("foregroundKeeperX").toInt(0);
    if (obj.contains("foregroundKeeperY"))        c.foregroundKeeperY        = obj.value("foregroundKeeperY").toInt(0);
    if (obj.contains("gpioEnabled"))              c.gpioEnabled              = obj.value("gpioEnabled").toBool(false);
    if (obj.contains("gpioMode"))                 c.gpioMode                 = toLower(obj.value("gpioMode").toString("off").toStdString());
    else if (c.gpioEnabled)                       c.gpioMode                 = "full";   // migrate legacy gpioEnabled=true
    if (obj.contains("gpioChip"))                 c.gpioChip                 = obj.value("gpioChip").toString("gpiochip0").toStdString();
    if (obj.contains("gpioEstopPin"))             c.gpioEstopPin             = obj.value("gpioEstopPin").toInt(17);
    if (obj.contains("gpioEngagePin"))            c.gpioEngagePin            = obj.value("gpioEngagePin").toInt(27);
    if (obj.contains("gpioParkPin"))              c.gpioParkPin              = obj.value("gpioParkPin").toInt(22);
    if (obj.contains("gpioBeltPin"))              c.gpioBeltPin              = obj.value("gpioBeltPin").toInt(4);
    if (obj.contains("gpioDevicePin"))            c.gpioDevicePin            = obj.value("gpioDevicePin").toInt(18);
    if (obj.contains("gpioLedRunPin"))            c.gpioLedRunPin            = obj.value("gpioLedRunPin").toInt(23);
    if (obj.contains("gpioLedReadyPin"))          c.gpioLedReadyPin          = obj.value("gpioLedReadyPin").toInt(24);
    if (obj.contains("gpioLedFaultPin"))          c.gpioLedFaultPin          = obj.value("gpioLedFaultPin").toInt(25);
}

// ---- rig.json "global": per-rig values shared across all axes ----
// (conditioning/blend feel, fault-reset policy, following-error window). These
// travel with the rig. Per-axis values live in rig.json "axes" (writeDriveConfig).
static void writeRigGlobal(const AppConfig& c, QJsonObject& obj)
{
    obj["conditioningMode"]         = QString::fromStdString(c.conditioningMode);
    obj["blendTimeSec"]             = c.blendTimeSec;
    obj["blendMaxVelocityMmS"]      = c.blendMaxVelocityMmS;
    obj["requireUserFaultReset"]    = c.requireUserFaultReset;
    // NULLCATX channel bindings. ALWAYS written (an empty array included):
    // saveRig merges schema keys over the existing file, so omitting the
    // key when empty would make a removed binding immortal on disk.
    {
        QJsonArray arr;
        for (const NcxBinding& b : c.ncxBindings)
        {
            QJsonObject o;
            o["token"]  = QString::fromStdString(b.token);
            o["slot"]   = b.slot;
            o["scale"]  = b.scale;
            o["offset"] = b.offset;
            arr.append(o);
        }
        obj["ncxBindings"] = arr;
    }
    // Haptic effects (HapticsRegistry.h). Same always-write rule as
    // ncxBindings: schema merge would make a deleted route immortal.
    obj["haptics"] = Config::writeHapticsObject(c);
}

// ---- haptics settings: one map from config key to field ------------------
// A tile's object in rig.json (and in a profile) carries exactly the keys
// every tile has (EffectParams) plus the tile's own registry rows
// (HapticsRegistry.h ParamSpec), each stored in one field of AppConfig.
// These tables are that storage, one row per field; read and write walk
// the registry and look each key up here, so a setting is added by its
// registry row and its line below, and TestConfigTwoFile's contract test
// fails for a row without one.
namespace {
template <class S> struct FieldRow { const char* key; double S::* field; };

template <class S, size_t N>
double* findField(S& s, const FieldRow<S> (&rows)[N], const char* key)
{
    for (const FieldRow<S>& r : rows)
        if (std::strcmp(r.key, key) == 0) return &(s.*r.field);
    return nullptr;
}

using haptics::EffectParams; using haptics::EngineParams; using haptics::RoadParams; using haptics::KerbParams;
using haptics::SurfaceParams; using haptics::WheelsParams; using haptics::ImpactParams; using haptics::PulseParams;
using haptics::DrivelineParams;

// Every tile (the engine's freqHz is its thump carrier, its jitter the lope).
constexpr FieldRow<EffectParams> kCommonFields[] = {
    { "ampPct", &EffectParams::ampPct }, { "freqHz", &EffectParams::freqHz }, { "durMs", &EffectParams::durMs },
    { "jitter", &EffectParams::jitter }, { "peakPct", &EffectParams::peakPct },
};
constexpr FieldRow<EngineParams> kEngineFields[] = {
    { "cylinders", &EngineParams::cylinders }, { "litres", &EngineParams::litres }, { "layout", &EngineParams::layout },
    { "maxRpm", &EngineParams::maxRpm }, { "rock", &EngineParams::rock }, { "thump", &EngineParams::thump },
    { "buzz", &EngineParams::buzz }, { "order", &EngineParams::order }, { "limHit", &EngineParams::limHit },
    { "limHz", &EngineParams::limHz }, { "limJit", &EngineParams::limJit }, { "inertia", &EngineParams::inertia },
    { "turbo", &EngineParams::turbo }, { "liftoff", &EngineParams::liftoff }, { "pops", &EngineParams::pops },
    { "tcCut", &EngineParams::tcCut }, { "foldback", &EngineParams::foldback },
};
constexpr FieldRow<RoadParams> kRoadFields[] = {
    { "fullMm", &RoadParams::fullMm }, { "hpHz", &RoadParams::hpHz }, { "surface", &RoadParams::surface },
    { "surfaceKmh", &RoadParams::surfaceKmh }, { "surfaceHz", &RoadParams::surfaceHz }, { "model", &RoadParams::model },
    { "bodyMm", &RoadParams::bodyMm }, { "bodyHz", &RoadParams::bodyHz }, { "hopHz", &RoadParams::hopHz },
    { "damping", &RoadParams::damping }, { "rough", &RoadParams::rough }, { "suspPct", &RoadParams::suspPct },
    { "suspHz", &RoadParams::suspHz }, { "fullLearn", &RoadParams::fullLearn },
};
constexpr FieldRow<KerbParams> kKerbFields[] = {
    { "pitchCm", &KerbParams::pitchCm }, { "riseMm", &KerbParams::riseMm }, { "ribMm", &KerbParams::ribMm },
    { "fullMm", &KerbParams::fullMm }, { "detectMm", &KerbParams::detectMm },
};
constexpr FieldRow<SurfaceParams> kSurfaceFields[] = {
    { "stones", &SurfaceParams::stones }, { "crunch", &SurfaceParams::crunch }, { "studs", &SurfaceParams::studs },
    { "puddles", &SurfaceParams::puddles }, { "aquaKmh", &SurfaceParams::aquaKmh }, { "smooth", &SurfaceParams::smooth },
};
constexpr FieldRow<WheelsParams> kWheelsFields[] = {
    { "balance", &WheelsParams::balance }, { "flat", &WheelsParams::flat }, { "judder", &WheelsParams::judder },
    { "puncture", &WheelsParams::puncture }, { "bite", &WheelsParams::bite }, { "drag", &WheelsParams::drag },
    { "biteHz", &WheelsParams::biteHz }, { "dragHz", &WheelsParams::dragHz },
};
constexpr FieldRow<ImpactParams> kImpactFields[] = {
    { "fromMmS", &ImpactParams::fromMmS }, { "fullMmS", &ImpactParams::fullMmS }, { "heaveG", &ImpactParams::heaveG },
};
// ABS and TC share the struct; each tile's registry rows say which it uses.
constexpr FieldRow<PulseParams> kPulseFields[] = {
    { "sharp", &PulseParams::sharp }, { "spread", &PulseParams::spread }, { "holdMs", &PulseParams::holdMs },
    { "slow", &PulseParams::slow }, { "buzz", &PulseParams::buzz }, { "buzzHz", &PulseParams::buzzHz },
    { "slipLink", &PulseParams::slipLink }, { "lockLink", &PulseParams::lockLink }, { "scrubLink", &PulseParams::scrubLink },
    { "brakeFull", &PulseParams::brakeFull }, { "fromWheels", &PulseParams::fromWheels }, { "lowOff", &PulseParams::lowOff },
};
constexpr FieldRow<DrivelineParams> kDrivelineFields[] = {
    { "clutch", &DrivelineParams::clutch }, { "clutchHz", &DrivelineParams::clutchHz }, { "lug", &DrivelineParams::lug },
    { "lugHz", &DrivelineParams::lugHz }, { "gearbox", &DrivelineParams::gearbox }, { "whine", &DrivelineParams::whine },
    { "whinePitch", &DrivelineParams::whinePitch }, { "shunt", &DrivelineParams::shunt }, { "shuntHz", &DrivelineParams::shuntHz },
    { "ignoreN", &DrivelineParams::ignoreN }, { "diff", &DrivelineParams::diff }, { "diffType", &DrivelineParams::diffType },
    { "diffHz", &DrivelineParams::diffHz }, { "driven", &DrivelineParams::driven },
};

// Each key the tile's object carries, in the order written: the common
// keys, then the tile's registry rows that are not one of them.
template <class F>
void forEachHapticsKey(const haptics::EffectInfo& info, F f)
{
    for (const FieldRow<EffectParams>& r : kCommonFields) f(r.key);
    for (int k = 0; k < info.paramCount; ++k)
    {
        const char* key = info.params[k].key;
        bool common = false;
        for (const FieldRow<EffectParams>& r : kCommonFields) common = common || std::strcmp(r.key, key) == 0;
        if (!common) f(key);
    }
}
} // namespace

double* Config::hapticsField(AppConfig& c, int effect, const char* key)
{
    if (effect < 0 || effect >= haptics::EFFECT_COUNT || !key) return nullptr;
    const haptics::EffectInfo& info = haptics::effectInfo(effect);
    if (double* f = findField(c.hapticsFx[static_cast<size_t>(effect)], kCommonFields, key)) return f;
    switch (info.kind)
    {
        case haptics::Kind::Engine:    return findField(c.hapticsEngine, kEngineFields, key);
        case haptics::Kind::Road:      return findField(c.hapticsRoad, kRoadFields, key);
        case haptics::Kind::Kerb:      return findField(c.hapticsKerb, kKerbFields, key);
        case haptics::Kind::Surface:   return findField(c.hapticsSurface, kSurfaceFields, key);
        case haptics::Kind::Wheels:    return findField(c.hapticsWheels, kWheelsFields, key);
        case haptics::Kind::Impacts:   return findField(c.hapticsImpacts, kImpactFields, key);
        case haptics::Kind::Abs:       return findField(c.hapticsAbs, kPulseFields, key);
        case haptics::Kind::Tc:        return findField(c.hapticsTc, kPulseFields, key);
        case haptics::Kind::Driveline: return findField(c.hapticsDriveline, kDrivelineFields, key);
        case haptics::Kind::Slip:
        {
            // The slip tiles' own table: SlipParams by slipField index.
            haptics::SlipParams& s = (info.id == haptics::Effect::Skid) ? c.hapticsSlipLat : c.hapticsSlipLon;
            for (int k = 0; k < haptics::SLIP_KEY_COUNT; ++k)
                if (info.slipKeys[k] && std::strcmp(info.slipKeys[k], key) == 0) return &haptics::slipField(s, k);
            return nullptr;
        }
        default:                       return nullptr;   // transients and plain textures: the common keys only
    }
}

const double* Config::hapticsField(const AppConfig& c, int effect, const char* key)
{
    return hapticsField(const_cast<AppConfig&>(c), effect, key);   // the same lookup, read only
}

// The haptics object exactly as rig.json carries it: one object per effect
// under its registry key (the engine description and mix inside the engine
// object, keys unchanged since 0.9.6), plus masterGain and positionBudget.
// Also what a haptics profile IS (HapticsProfiles), so a profile and the
// live set never drift apart in shape.
QJsonObject Config::writeHapticsObject(const AppConfig& c)
{
    QJsonObject h;
    for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
    {
        const haptics::EffectInfo& info = haptics::effectInfo(i);
        const haptics::EffectParams& p = c.hapticsFx[static_cast<size_t>(i)];
        QJsonObject o;
        forEachHapticsKey(info, [&](const char* key) { if (const double* f = hapticsField(c, i, key)) o[key] = *f; });
        QJsonArray r;
        for (const haptics::Route& rt : p.routes)
            if ((rt.axis != -1 || rt.shaker != -1) && rt.gain > 0.0)
            {
                QJsonObject ro; ro["gain"] = rt.gain;
                if (rt.shaker >= 0) { ro["shaker"] = rt.shaker; if (rt.harm != 1) ro["harm"] = rt.harm; }
                else ro["axis"] = rt.axis;
                if (haptics::kindHasParts(info.kind) && rt.part != haptics::Part::All) ro["part"] = haptics::partKey(rt.part);
                r.append(ro);
            }
        o["routes"] = r;
        h[info.key] = o;
    }
    h["masterGain"] = c.hapticsMasterGain;
    h["positionBudget"] = c.hapticsPositionBudget;
    h["axisDelayMs"] = c.hapticsAxisDelayMs;
    h["followCar"] = c.hapticsFollowCar;
    return h;
}

static void readRigGlobal(const QJsonObject& obj, AppConfig& c)
{
    if (obj.contains("conditioningMode"))         c.conditioningMode         = obj.value("conditioningMode").toString("bypass").toStdString();
    if (obj.contains("blendTimeSec"))             c.blendTimeSec             = obj.value("blendTimeSec").toDouble(2.0);
    if (obj.contains("blendMaxVelocityMmS"))      c.blendMaxVelocityMmS      = obj.value("blendMaxVelocityMmS").toDouble(20.0);
    if (obj.contains("requireUserFaultReset"))    c.requireUserFaultReset    = obj.value("requireUserFaultReset").toBool(false);
    if (obj.contains("ncxBindings"))
    {
        c.ncxBindings.clear();
        for (const QJsonValue& v : obj.value("ncxBindings").toArray())
        {
            const QJsonObject o = v.toObject();
            NcxBinding b;
            b.token  = o.value("token").toString("").toStdString();
            b.slot   = o.value("slot").toInt(0);
            b.scale  = o.value("scale").toDouble(1.0);
            b.offset = o.value("offset").toDouble(0.0);
            c.ncxBindings.push_back(b);
        }
    }
    if (obj.contains("haptics"))
        Config::readHapticsObject(obj.value("haptics").toObject(), c);
}

// Read a haptics object (rig.json's, or a profile's) into the config.
// Missing effects keep what c already has, so a partial object (an old
// file, a profile saved before an effect existed) never zeroes the rest;
// a missing or non-numeric key keeps its field as it was too.
void Config::readHapticsObject(const QJsonObject& h, AppConfig& c)
{
    for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
    {
        const haptics::EffectInfo& info = haptics::effectInfo(i);
        // A 0.9.6 file carries the slip tiles under their old keys
        // ("skid", "lockup"): read those when the new key is absent so
        // an amplitude and its routes survive the rename.
        const char* legacy = haptics::legacyEffectKey(info.id);
        const bool  haveNew = h.contains(info.key);
        if (!haveNew && !(legacy && h.contains(legacy))) continue;
        const QJsonObject o = h.value(haveNew ? info.key : legacy).toObject();
        haptics::EffectParams& p = c.hapticsFx[static_cast<size_t>(i)];
        forEachHapticsKey(info, [&](const char* key) { if (double* f = hapticsField(c, i, key)) *f = o.value(key).toDouble(*f); });
        // Routes are explicit axis indices only. Negative axis values are
        // dropped on read: early 0.9.6 files carried a since-removed
        // "own axis" (-2) route default.
        if (o.contains("routes"))
        {
            for (haptics::Route& rt : p.routes) rt = haptics::Route{};
            int n = 0;
            for (const QJsonValue& v : o.value("routes").toArray())
            {
                if (n >= haptics::MAX_ROUTES) break;
                const QJsonObject ro = v.toObject();
                const int axis   = ro.value("axis").toInt(-1);
                const int shaker = ro.value("shaker").toInt(-1);
                if (axis < 0 && shaker < 0) continue;
                p.routes[n].axis   = (shaker >= 0) ? -1 : axis;
                p.routes[n].shaker = shaker;
                // Read as written: validate() refuses a harm outside
                // 1..8 on a save, and load() clamps it with a warning.
                p.routes[n].harm   = ro.value("harm").toInt(1);
                p.routes[n].gain   = ro.value("gain").toDouble(0.0);
                p.routes[n].part   = haptics::partFromKey(ro.value("part").toString("all").toStdString().c_str());
                ++n;
            }
        }
        // Old single-carrier slip file: its freqHz was the one carrier;
        // carry it onto the component the old effect was (skid = texture =
        // scrub, lockup = judder = lock).
        if (info.kind == haptics::Kind::Slip && !haveNew && o.contains("freqHz"))
        {
            haptics::SlipParams& s = (info.id == haptics::Effect::Skid) ? c.hapticsSlipLat : c.hapticsSlipLon;
            s.aHz = o.value("freqHz").toDouble(s.aHz);
        }
    }
    if (h.contains("masterGain"))  c.hapticsMasterGain = h.value("masterGain").toDouble(1.0);
    if (h.contains("positionBudget")) c.hapticsPositionBudget = h.value("positionBudget").toDouble(0.4);
    if (h.contains("axisDelayMs")) c.hapticsAxisDelayMs = h.value("axisDelayMs").toDouble(0.0);
    if (h.contains("followCar"))   c.hapticsFollowCar = h.value("followCar").toBool(true);
}

// ---- Control-loading "device" object (families shifter/pedal) --------------
// Curves serialize as [[x,y],...] node arrays - the exact structure the web
// curve editor edits and a preset ships.
static QJsonArray writeCurve(const std::vector<CurveNode>& c)
{
    QJsonArray a;
    for (const CurveNode& n : c)
    { QJsonArray p; p.append(n.x); p.append(n.y); a.append(p); }
    return a;
}
static void readCurve(const QJsonValue& v, std::vector<CurveNode>& out)
{
    if (!v.isArray()) return;   // absent/wrong shape: keep caller's value
    out.clear();
    for (const QJsonValue& e : v.toArray())
    {
        const QJsonArray p = e.toArray();
        if (p.size() != 2) continue;
        out.push_back({ p.at(0).toDouble(), p.at(1).toDouble() });
    }
}

static void writeDeviceParams(const DeviceParams& p, QJsonObject& o)
{
    o["dir"]             = p.dir;
    o["neutralRev"]      = p.neutralRev;
    o["springCurve"]     = writeCurve(p.springCurve);
    o["detentCurve"]     = writeCurve(p.detentCurve);
    { QJsonArray a; for (double d : p.detents) a.append(d); o["detents"] = a; }
    o["stopMinRev"]      = p.stopMinRev;
    o["stopMaxRev"]      = p.stopMaxRev;
    o["stopSpring"]      = p.stopSpring;
    o["stopDamp"]        = p.stopDamp;
    o["lashRev"]         = p.lashRev;
    o["dampPctPerRevS"]  = p.dampPctPerRevS;
    o["frictionPct"]     = p.frictionPct;
    o["breakoutScale"]   = p.breakoutScale;
    o["velLpfHz"]        = p.velLpfHz;
    o["maxForcePct"]     = p.maxForcePct;
    o["homeTorquePct"]   = p.homeTorquePct;
    o["homeDir"]         = p.homeDir;
    o["slewPctPerSec"]   = p.slewPctPerSec;
    o["thermalDwellSec"] = p.thermalDwellSec;
    o["thermalPct"]      = p.thermalPct;
    o["foldRpm"]         = p.foldRpm;
    o["clutchBitePct"]   = p.clutchBitePct;
    o["blockGain"]       = p.blockGain;
    o["grindAmpPct"]     = p.grindAmpPct;
    o["grindFreqHz"]     = p.grindFreqHz;
    o["blockStartRev"]   = p.blockStartRev;
    o["rpmMatchPct"]     = p.rpmMatchPct;
}
static void writeDriveConfig(const DriveConfig& d, QJsonObject& obj)
{
    obj["slaveIndex"]                = d.slaveIndex;
    obj["name"]                      = QString::fromStdString(d.name);
    obj["mode"]                      = QString::fromStdString(d.mode);
    obj["axisType"]                  = QString::fromStdString(d.axisType);
    obj["invertDir"]                 = d.invertDir;
    obj["strokeMm"]                  = d.strokeMm;
    obj["ballscrewPitch"]            = d.ballscrewPitch;
    obj["encoderCountsPerRev"]       = d.encoderCountsPerRev;
    obj["countsPerMm"]               = d.countsPerMm;
    obj["reductionRatio"]            = QString::fromStdString(d.reductionRatio);
    obj["homeDirection"]             = QString::fromStdString(d.homeDirection);
    obj["parkMode"]                  = QString::fromStdString(d.parkMode);
    obj["homingBackoffMm"]           = d.homingBackoffMm;
    obj["homingSpeed"]            = d.homingSpeed;
    obj["homingTorquePct"]           = d.homingTorquePct;
    obj["maxVelocityMmS"]            = d.maxVelocityMmS;
    obj["maxAccelerationMmS2"]       = d.maxAccelerationMmS2;
    obj["maxJerkMmS3"]               = d.maxJerkMmS3;
    obj["followingErrorWindowMm"]    = d.followingErrorWindowMm;
    obj["trackingWnHz"]              = d.trackingWnHz;
    obj["unparkTimeSec"]             = d.unparkTimeSec;
    obj["onlineHoldTimeoutSec"]      = d.onlineHoldTimeoutSec;
    obj["parkTimeSec"]               = d.parkTimeSec;
    obj["spikeFilterEnabled"]        = d.spikeFilterEnabled;
    obj["spikeMaxMm"]                = d.spikeMaxMm;
    obj["torqueMinPct"]              = d.torqueMinPct;
    obj["torqueMaxPct"]              = d.torqueMaxPct;
    obj["beltSlewPctPerSec"]         = d.beltSlewPctPerSec;
    obj["beltOverspeedRpm"]          = d.beltOverspeedRpm;
    obj["beltOverspeedMs"]           = d.beltOverspeedMs;
    obj["beltMaxTravelRevs"]         = d.beltMaxTravelRevs;
    obj["beltMaxRpm"]                = d.beltMaxRpm;
    obj["hapticsMaxMm"]              = d.hapticsMaxMm;
    obj["beltRelaxerSec"]            = d.beltRelaxerSec;
    obj["beltRelaxerPct"]            = d.beltRelaxerPct;
    // Nested device object: written for the control-loading families only,
    // so ordinary axes never carry an unused block.
    if (axisCaps(d.axisType, d.mode).isDevice())
    {
        QJsonObject dev;
        writeDeviceParams(d.device, dev);
        obj["device"] = dev;
    }
}

// Present-only readers. A key that is ABSENT leaves the value the caller
// supplied -- which is the compiled-in default from DriveConfig (Config.h),
// since loadAxesArray default-constructs before every call.
//
// The previous form spelled a default into every call site
// (obj.value(k).toDouble(2000.0)), creating a SECOND set of defaults that was
// free to drift from the struct -- and had: parkMode "center" vs "endstop",
// homingSpeed 5 vs 250, maxAccelerationMmS2 2000 vs 10000. An axis missing
// those keys silently got the stale value, defeating the configured default
// and contradicting CONFIG_REFERENCE ("missing keys fall back to compiled-in
// defaults"). Passing the current value as the fallback also means a null or
// wrong-typed entry keeps the default instead of collapsing to 0/"".
static void rdStr(const QJsonObject& o, const char* k, std::string& v, bool lower = false)
{
    if (!o.contains(k)) return;
    std::string s = o.value(k).toString(QString::fromStdString(v)).toStdString();
    v = lower ? toLower(s) : s;
}
static void rdDbl (const QJsonObject& o, const char* k, double& v) { if (o.contains(k)) v = o.value(k).toDouble(v); }
static void rdInt (const QJsonObject& o, const char* k, int&    v) { if (o.contains(k)) v = o.value(k).toInt(v); }
static void rdBool(const QJsonObject& o, const char* k, bool&   v) { if (o.contains(k)) v = o.value(k).toBool(v); }

static void readDeviceParams(const QJsonObject& o, DeviceParams& p)
{
    rdDbl(o, "dir",             p.dir);
    rdDbl(o, "neutralRev",      p.neutralRev);
    readCurve(o.value("springCurve"), p.springCurve);
    readCurve(o.value("detentCurve"), p.detentCurve);
    if (o.contains("detents") && o.value("detents").isArray())
    {
        p.detents.clear();
        for (const QJsonValue& v : o.value("detents").toArray())
            p.detents.push_back(v.toDouble());
    }
    rdDbl(o, "stopMinRev",      p.stopMinRev);
    rdDbl(o, "stopMaxRev",      p.stopMaxRev);
    rdDbl(o, "stopSpring",      p.stopSpring);
    rdDbl(o, "stopDamp",        p.stopDamp);
    rdDbl(o, "lashRev",         p.lashRev);
    rdDbl(o, "dampPctPerRevS",  p.dampPctPerRevS);
    rdDbl(o, "frictionPct",     p.frictionPct);
    rdDbl(o, "breakoutScale",   p.breakoutScale);
    rdDbl(o, "velLpfHz",        p.velLpfHz);
    rdDbl(o, "maxForcePct",     p.maxForcePct);
    rdDbl(o, "homeTorquePct",   p.homeTorquePct);
    rdDbl(o, "homeDir",         p.homeDir);
    rdDbl(o, "slewPctPerSec",   p.slewPctPerSec);
    rdDbl(o, "thermalDwellSec", p.thermalDwellSec);
    rdDbl(o, "thermalPct",      p.thermalPct);
    rdDbl(o, "foldRpm",         p.foldRpm);
    rdDbl(o, "clutchBitePct",   p.clutchBitePct);
    rdDbl(o, "blockGain",       p.blockGain);
    rdDbl(o, "grindAmpPct",     p.grindAmpPct);
    rdDbl(o, "grindFreqHz",     p.grindFreqHz);
    rdDbl(o, "blockStartRev",   p.blockStartRev);
    rdDbl(o, "rpmMatchPct",     p.rpmMatchPct);
}

static void readDriveConfig(const QJsonObject& obj, int idx, DriveConfig& d)
{
    // Positional defaults: these two depend on the axis index, not the struct.
    d.slaveIndex = idx + 1;
    d.name       = QString("Drive %1").arg(idx + 1).toStdString();
    rdInt(obj, "slaveIndex", d.slaveIndex);
    rdStr(obj, "name",       d.name);

    rdStr(obj, "mode", d.mode, /*lower=*/true);
    // Canonical torque-mode string is "torque" (what the motion + EtherCAT
    // paths check). Older configs / the DS402 term used "cst" -- normalise it
    // so every downstream check sees one value.
    if (d.mode == "cst") d.mode = "torque";
    rdStr (obj, "axisType",               d.axisType, true);
    rdBool(obj, "invertDir",              d.invertDir);
    rdDbl (obj, "strokeMm",               d.strokeMm);
    rdDbl (obj, "ballscrewPitch",         d.ballscrewPitch);
    rdDbl (obj, "encoderCountsPerRev",    d.encoderCountsPerRev);
    rdStr (obj, "reductionRatio",         d.reductionRatio);
    rdStr (obj, "homeDirection",          d.homeDirection, true);
    // parkMode was called homeMode until 0.9.2. Read the old key first so an
    // existing rig.json keeps its park position, then let the new key win if
    // both are present. Without this the field would silently fall back to the
    // struct default and move where the axis parks.
    rdStr (obj, "homeMode",               d.parkMode, true);   // deprecated alias
    rdStr (obj, "parkMode",               d.parkMode, true);
    rdDbl (obj, "homingBackoffMm",        d.homingBackoffMm);
    rdDbl (obj, "homingSpeedMmS",      d.homingSpeed);        // deprecated alias (see parkMode above)
    rdDbl (obj, "homingSpeed",         d.homingSpeed);
    rdInt (obj, "homingTorquePct",        d.homingTorquePct);
    rdDbl (obj, "maxVelocityMmS",         d.maxVelocityMmS);
    rdDbl (obj, "maxAccelerationMmS2",    d.maxAccelerationMmS2);
    rdDbl (obj, "maxJerkMmS3",            d.maxJerkMmS3);
    rdDbl (obj, "followingErrorWindowMm", d.followingErrorWindowMm);
    rdDbl (obj, "trackingWnHz",           d.trackingWnHz);
    rdDbl (obj, "unparkTimeSec",          d.unparkTimeSec);
    rdDbl (obj, "onlineHoldTimeoutSec",   d.onlineHoldTimeoutSec);
    rdDbl (obj, "parkTimeSec",            d.parkTimeSec);
    rdBool(obj, "spikeFilterEnabled",     d.spikeFilterEnabled);
    rdDbl (obj, "spikeMaxMm",             d.spikeMaxMm);
    rdDbl (obj, "torqueMinPct",           d.torqueMinPct);
    rdDbl (obj, "torqueMaxPct",           d.torqueMaxPct);
    rdDbl (obj, "beltSlewPctPerSec",      d.beltSlewPctPerSec);
    rdDbl (obj, "beltOverspeedRpm",       d.beltOverspeedRpm);
    rdDbl (obj, "beltOverspeedMs",        d.beltOverspeedMs);
    rdDbl (obj, "beltMaxTravelRevs",      d.beltMaxTravelRevs);
    rdDbl (obj, "beltMaxRpm",             d.beltMaxRpm);
    rdDbl (obj, "hapticsMaxMm",           d.hapticsMaxMm);
    rdDbl (obj, "beltRelaxerSec",         d.beltRelaxerSec);
    rdDbl (obj, "beltRelaxerPct",         d.beltRelaxerPct);
    if (obj.contains("device") && obj.value("device").isObject())
        readDeviceParams(obj.value("device").toObject(), d.device);

    // countsPerMm is derived from encoderCountsPerRev * reduction /
    // ballscrewPitch for linear axes. If the file explicitly stores countsPerMm
    // we respect that value here (round-trip safety for hand-edited rigs), but
    // recomputeDerivedFields() re-derives it after load, so the derivation is
    // the single source of truth.
    const double derived = (d.ballscrewPitch > 0.0)
        ? (d.encoderCountsPerRev * reductionFactor(d.reductionRatio) / d.ballscrewPitch)
        : 13107.2;
    if (obj.contains("countsPerMm"))
        d.countsPerMm = obj.value("countsPerMm").toDouble(derived);
    else
        d.countsPerMm = derived;
}

// ---------- Config -----------------------------------------------------------

Config::Config()
{
    setDefaults();
}

void Config::setDefaults()
{
    m_config = AppConfig{};
    seedDefaultDrive(m_config);
}

// Seed the default Drive 1 WITHOUT resetting the rest of the config. load()
// calls this when host.json exists but rig.json does not (a fresh install
// with a seeded host.json): a full setDefaults() there would wipe the host
// fields that were just read.
void Config::seedDefaultDrive(AppConfig& c)
{
    DriveConfig d;
    d.slaveIndex = 1;
    d.name       = "Drive 1";
    recomputeDerivedFields(d);
    c.drives.clear();
    c.drives.push_back(d);
    c.numDrives = 1;
}

void Config::recomputeDerivedFields(DriveConfig& d)
{
    if (d.ballscrewPitch > 0.0)
        d.countsPerMm = d.encoderCountsPerRev * reductionFactor(d.reductionRatio)
                        / d.ballscrewPitch;
}

// ---------- two-file (host.json + rig.json) helpers --------------------------

// The directory that holds the split files, derived from the anchor-path
// anchor path the callers already pass.
static QString configDir(const std::string& anchorPath)
{
    return QFileInfo(QString::fromStdString(anchorPath)).absolutePath();
}
static QString hostFile(const QString& dir) { return dir + "/host.json"; }
static QString rigFile (const QString& dir) { return dir + "/rig.json";  }

// A JSON object file is one of three things to a load: not there (a cold
// start, or the missing half of the pair), there but not readable as a JSON
// object (truncated by a crash mid-write, a hand edit gone wrong; why in
// err), or read. The two failures must never be confused: an absent file
// gets defaults written, an unreadable one must not.
enum class JsonRead { Absent, Unreadable, Ok };
static JsonRead readJsonObject(const QString& path, QJsonObject& out, QString* err = nullptr)
{
    QFile f(path);
    if (!f.exists()) return JsonRead::Absent;
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (err) *err = f.errorString();
        return JsonRead::Unreadable;
    }
    const QByteArray data = f.readAll();
    f.close();
    QJsonParseError pe;
    const QJsonDocument doc = QJsonDocument::fromJson(data, &pe);
    if (pe.error != QJsonParseError::NoError)
    {
        if (err) *err = pe.errorString() + QString(" at offset %1").arg(pe.offset);
        return JsonRead::Unreadable;
    }
    if (!doc.isObject())
    {
        if (err) *err = "not a JSON object";
        return JsonRead::Unreadable;
    }
    out = doc.object();
    return JsonRead::Ok;
}
// Written to a temporary file beside the target and renamed over it on
// commit (QSaveFile), so a crash or a full disk mid-write leaves the old
// file whole, never a truncated one: a truncated rig.json once read as
// "no config" and the next start wrote defaults over the rig's tuning. A
// stale <name>.tmp left by an interrupted web save is no obstacle
// (QSaveFile names its own temporary). On Windows the rename can fail for
// a few tens of ms while another reader (the native app's file watcher,
// a virus scan) holds the target open; a short retry absorbs that.
static bool writeJsonObject(const QString& path, const QJsonObject& obj)
{
    const QByteArray text = QJsonDocument(obj).toJson(QJsonDocument::Indented);
    for (int attempt = 0; attempt < 8; ++attempt)
    {
        if (attempt > 0) std::this_thread::sleep_for(std::chrono::milliseconds(50));
        QSaveFile f(path);
        if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) continue;
        f.write(text);
        if (f.commit()) return true;
    }
    return false;
}
// A copy of a file beside it as <name>.bak, replacing an older one: taken
// before load() rewrites a file (the retired-key migration), so the file
// as the user had it is never gone.
static bool backupFile(const QString& path)
{
    const QString bak = path + ".bak";
    QFile::remove(bak);
    return QFile::copy(path, bak);
}
// Overlay schema keys onto dst, leaving unknown keys already in dst untouched
// (merge-on-save: user annotations / future-build fields survive).
static void overlayKeys(QJsonObject& dst, const QJsonObject& schema)
{
    for (auto it = schema.constBegin(); it != schema.constEnd(); ++it)
        dst[it.key()] = it.value();
}
// The haptics object merges per tile: a key inside a tile this build does
// not know (a newer version's setting, read back by an older build, or a
// hand annotation) survives a save the way the unknown top-level keys do;
// a tile's schema keys (routes included, always written) land over it. A
// tile that is not an object in the file is replaced. The retired keys
// (0.9.6 "skid"/"lockup") are dropped: the migration rewrite means them
// to be, and the web serves the file as is.
static QJsonObject mergeHaptics(const QJsonObject& existing, const QJsonObject& schema)
{
    QJsonObject out = existing;
    for (auto it = schema.constBegin(); it != schema.constEnd(); ++it)
    {
        if (it.value().isObject() && out.value(it.key()).isObject())
        {
            QJsonObject tile = out.value(it.key()).toObject();
            overlayKeys(tile, it.value().toObject());
            out[it.key()] = tile;
        }
        else out[it.key()] = it.value();
    }
    for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
        if (const char* legacy = haptics::legacyEffectKey(haptics::effectInfo(i).id)) out.remove(legacy);
    return out;
}
// Merge a per-axis array by slaveIndex so unknown per-axis keys survive.
static QJsonArray mergeAxes(const QJsonArray& existing, const QJsonArray& schema)
{
    QHash<int, QJsonObject> bySlave;
    for (const QJsonValue& v : existing)
    {
        QJsonObject o = v.toObject();
        int si = o.value("slaveIndex").toInt(-1);
        if (si > 0) bySlave.insert(si, o);
    }
    QJsonArray out;
    for (const QJsonValue& v : schema)
    {
        QJsonObject s = v.toObject();
        int si = s.value("slaveIndex").toInt(-1);
        QJsonObject target = bySlave.value(si, QJsonObject());
        for (auto it = s.constBegin(); it != s.constEnd(); ++it)
            target[it.key()] = it.value();
        out.append(target);
    }
    return out;
}

// Populate a drives[] vector from a rig "axes" array.
// Free function so Config.h stays Qt-free.
static void loadAxesArray(const QJsonArray& axes, std::vector<DriveConfig>& drives)
{
    drives.clear();
    int idx = 0;
    for (const QJsonValue& v : axes)
    {
        if (idx >= 10) break;
        if (!v.isObject()) continue;
        DriveConfig d;
        readDriveConfig(v.toObject(), idx, d);
        Config::recomputeDerivedFields(d);
        drives.push_back(d);
        ++idx;
    }
}

// ---- validate-and-warn on load -----------------------------------------------
// A hand-edited file can carry a value past the range the strip offers.
// The web refuses such a body on save; a load takes the file as it is,
// so the values that would hurt past their range are clamped to it, each
// with a warning that names the file and the field: an amplitude of 500 %
// (five times what the strip can ask for), a route gain of 9 x, a shaker
// harmonic of 12, the master gain, the position budget and an axis's
// haptic max. Everything else out of range is reported by validate() and
// runs as read. The file is left as written, so the warning repeats at
// every start until it is fixed: only the set that runs is clamped, and
// every caller that saves the set back loads it without (load's forRun).
// One small function, so a registry-driven clamp over every ParamSpec row
// can replace it later.
void Config::clampHaptics(AppConfig& c, std::vector<std::string>& warnings)
{
    const auto clampD = [&warnings](double& v, double lo, double hi, const std::string& field)
    {
        if (v >= lo && v <= hi) return;
        const double was = v;
        v = std::max(lo, std::min(hi, v));
        char buf[200];
        snprintf(buf, sizeof(buf), "rig.json: %s is %g, outside [%g, %g]: running at %g", field.c_str(), was, lo, hi, v);
        warnings.push_back(buf);
    };
    const auto clampI = [&warnings](int& v, int lo, int hi, const std::string& field)
    {
        if (v >= lo && v <= hi) return;
        const int was = v;
        v = std::max(lo, std::min(hi, v));
        warnings.push_back("rig.json: " + field + " is " + std::to_string(was) + ", outside [" + std::to_string(lo)
                           + ", " + std::to_string(hi) + "]: running at " + std::to_string(v));
    };
    // The axes first: a position route's gain is capped by its axis's max.
    for (size_t i = 0; i < c.drives.size(); ++i)
        clampD(c.drives[i].hapticsMaxMm, 0.0, 10.0, "axes[" + std::to_string(i) + "].hapticsMaxMm");
    for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
    {
        const haptics::EffectInfo& info = haptics::effectInfo(i);
        haptics::EffectParams& p = c.hapticsFx[static_cast<size_t>(i)];
        const std::string pfx = std::string("haptics.") + info.key + ".";
        double ampLo = 0.0, ampHi = 100.0;   // the registry's amp % row, every tile's
        for (int k = 0; k < info.paramCount; ++k)
            if (std::strcmp(info.params[k].key, "ampPct") == 0) { ampLo = info.params[k].min; ampHi = info.params[k].max; }
        clampD(p.ampPct, ampLo, ampHi, pfx + "ampPct");
        for (int n = 0; n < haptics::MAX_ROUTES; ++n)
        {
            haptics::Route& r = p.routes[n];
            if (r.axis == -1 && r.shaker == -1 && r.gain <= 0.0) continue;   // unused slot
            const std::string rpfx = pfx + "routes[" + std::to_string(n) + "].";
            clampI(r.harm, 1, 8, rpfx + "harm");
            // The gain's unit follows the destination (as validate() reads
            // it): a shaker or a torque axis takes a multiplier up to 2, a
            // position axis millimetres up to its own haptic max.
            double hi = 2.0;
            if (r.shaker < 0 && r.axis >= 0 && static_cast<size_t>(r.axis) < c.drives.size()
                && c.drives[static_cast<size_t>(r.axis)].mode != "torque")
                hi = c.drives[static_cast<size_t>(r.axis)].hapticsMaxMm;
            clampD(r.gain, 0.0, hi, rpfx + "gain");
        }
    }
    clampD(c.hapticsMasterGain, 0.0, 2.0, "haptics.masterGain");
    clampD(c.hapticsPositionBudget, 0.0, 1.0, "haptics.positionBudget");
}

// Which file a validate() error belongs to, for the warning that reports
// it: the host fields the validator checks are few and named first in
// their message; everything else is the rig's.
static const char* fileForError(const std::string& e)
{
    static const char* const kHostFields[] = { "controlLoopHz", "rtCore", "ncxFrozenMs", "pdoWatchdogMs", "sync0RecycleRounds",
                                               "audioChannels", "audioDevices" };
    for (const char* f : kHostFields)
        if (e.rfind(f, 0) == 0) return "host.json";
    return "rig.json";
}

void Config::clampForRun()
{
    clampHaptics(m_config, m_loadWarnings);
    for (const std::string& e : m_config.validate())
        m_loadWarnings.push_back(std::string(fileForError(e)) + ": " + e);
}

bool Config::load(const std::string& anchorPath, bool forRun)
{
    const QString dir    = configDir(anchorPath);
    m_loadWarnings.clear();

    QJsonObject hostObj, rigObj;
    QString hostErr, rigErr;
    const JsonRead host = readJsonObject(hostFile(dir), hostObj, &hostErr);
    const JsonRead rig  = readJsonObject(rigFile(dir),  rigObj,  &rigErr);

    // A file that is there but cannot be read is a hard stop, never a
    // reason to run on defaults: the mains normalise the config to disk
    // right after loading it, and every server-side apply saves, so
    // defaults here would be written over whatever the file still holds.
    // The config in memory stays as it was (a reload from the web keeps
    // running on the set it had); the file is left for the user to fix.
    if (host == JsonRead::Unreadable || rig == JsonRead::Unreadable)
    {
        const bool h = (host == JsonRead::Unreadable);
        m_lastError = (h ? hostFile(dir) : rigFile(dir)).toStdString() + " cannot be read ("
                    + (h ? hostErr : rigErr).toStdString()
                    + "): fix or move the file; nothing has been written over it";
        return false;
    }

    if (host == JsonRead::Ok || rig == JsonRead::Ok)
    {
        // ---- native two-file format (the steady state) ----
        AppConfig c;
        if (host == JsonRead::Ok) readHostConfig(hostObj, c);
        bool rewriteRig = false;
        if (rig == JsonRead::Ok)
        {
            c.configVersion = rigObj.value("configVersion").toInt(2);
            readRigGlobal(rigObj.value("global").toObject(), c);
            loadAxesArray(rigObj.value("axes").toArray(), c.drives);
            int nd = rigObj.value("numDrives").toInt(static_cast<int>(c.drives.size()));
            c.numDrives = std::max(1, std::min(nd, static_cast<int>(c.drives.size())));
            // A file carrying a haptic effect under a retired key (0.9.6
            // "skid"/"lockup") was read through the migration; rewrite it
            // once under the current keys so the web, which serves the file
            // as is, shows the same tuning the controller runs and a later
            // save cannot write defaults over it.
            const QJsonObject h = rigObj.value("global").toObject().value("haptics").toObject();
            for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
            {
                const haptics::EffectInfo& info = haptics::effectInfo(i);
                const char* legacy = haptics::legacyEffectKey(info.id);
                if (legacy && h.contains(legacy)) rewriteRig = true;
            }
        }
        if (c.drives.empty()) seedDefaultDrive(c);
        m_config = c;
        m_lastError.clear();
        // The one rewrite a load makes keeps the file as it was beside it,
        // and writes the values as read (the clamp below is in memory only).
        if (rewriteRig && backupFile(rigFile(dir))) saveRig(anchorPath);
        // Clamp and warn, never refuse: the rig runs at the edge of its
        // range instead of beyond it or not at all, and the log says so.
        if (forRun) clampForRun();
        return true;
    }

    // ---- no split files: cold start ----
    // The legacy flat config.json migration was retired pre-release: every
    // deployment is a fresh install of the two-file schema, so a missing pair
    // simply means first boot.
    setDefaults();
    m_config.configVersion = 2;

    // Persist the split pair so the directory always ends up with valid
    // two-file output.
    saveHost(anchorPath);
    saveRig(anchorPath);
    m_lastError.clear();
    return true;
}

// host.json - single writer (Qt on PC, web on Pi). Merge-on-save preserves
// unknown keys already in the file. A file that is there but unreadable is
// never written over (the merge would have nothing to keep): the save
// fails, as the load of it did.
bool Config::saveHost(const std::string& anchorPath) const
{
    const QString path = hostFile(configDir(anchorPath));
    QJsonObject existing;
    if (readJsonObject(path, existing) == JsonRead::Unreadable) return false;   // absent: a fresh write
    QJsonObject schema;
    writeHostConfig(m_config, schema);
    overlayKeys(existing, schema);
    return writeJsonObject(path, existing);
}

// rig.json - single writer (the web UI on both platforms). { configVersion,
// numDrives, global:{...}, axes:[...] }. Unknown keys in global and per-axis
// survive (merge-on-save); axes merge by slaveIndex. Same rule as saveHost
// for a file that is there but unreadable: never written over.
bool Config::saveRig(const std::string& anchorPath) const
{
    const QString path = rigFile(configDir(anchorPath));
    QJsonObject existing;
    if (readJsonObject(path, existing) == JsonRead::Unreadable) return false;

    QJsonObject globalMerged = existing.value("global").toObject();
    QJsonObject globalSchema;
    writeRigGlobal(m_config, globalSchema);
    globalSchema["haptics"] = mergeHaptics(globalMerged.value("haptics").toObject(), globalSchema.value("haptics").toObject());
    overlayKeys(globalMerged, globalSchema);

    QJsonArray axesSchema;
    for (const DriveConfig& d : m_config.drives)
    {
        QJsonObject o;
        writeDriveConfig(d, o);
        axesSchema.append(o);
    }
    QJsonArray axesMerged = mergeAxes(existing.value("axes").toArray(), axesSchema);

    existing["configVersion"] = m_config.configVersion;
    existing["numDrives"]     = m_config.numDrives;
    existing["global"]        = globalMerged;
    existing["axes"]          = axesMerged;
    return writeJsonObject(path, existing);
}

// Transitional convenience: write both. Prefer saveHost()/saveRig() so each
// file has a single writer per platform.
bool Config::save(const std::string& anchorPath) const
{
    const bool h = saveHost(anchorPath);
    const bool r = saveRig(anchorPath);
    return h && r;
}

// ---- web endpoint validators ------------------------------------------------
// Build a full AppConfig from one proposed namespace body + the OTHER namespace
// already on disk, then run the single unified validator. Cross-field checks
// (e.g. controlLoopHz host vs per-axis trackingWnHz rig) stay meaningful.

std::vector<std::string> Config::validateRigBody(const std::string& anchorPath,
                                                 const std::string& rigJsonBody)
{
    AppConfig c;
    QJsonObject hostObj;
    if (readJsonObject(hostFile(configDir(anchorPath)), hostObj) == JsonRead::Ok)
        readHostConfig(hostObj, c);

    QJsonParseError pe;
    QJsonDocument doc = QJsonDocument::fromJson(QString::fromStdString(rigJsonBody).toUtf8(), &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject())
        return { std::string("Invalid rig JSON: ") + pe.errorString().toStdString() };

    QJsonObject rig = doc.object();
    readRigGlobal(rig.value("global").toObject(), c);
    loadAxesArray(rig.value("axes").toArray(), c.drives);
    c.numDrives = std::max(1, std::min(rig.value("numDrives").toInt(static_cast<int>(c.drives.size())),
                                       static_cast<int>(c.drives.size())));
    if (c.drives.empty()) return { std::string("rig has no axes") };
    return c.validate();
}

// Parse a rig body's axes into DriveConfigs (no validation - callers run
// this only on a body validateRigBody already accepted). Feeds the device
// live-apply path: the web server stages each device axis's fresh params
// with the motion controller after a successful save.
bool Config::parseRigBodyAxes(const std::string& rigJsonBody,
                              std::vector<DriveConfig>& out)
{
    QJsonParseError pe;
    QJsonDocument doc = QJsonDocument::fromJson(
        QString::fromStdString(rigJsonBody).toUtf8(), &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject()) return false;
    out.clear();
    loadAxesArray(doc.object().value("axes").toArray(), out);
    return !out.empty();
}

std::vector<std::string> Config::validateHostBody(const std::string& anchorPath,
                                                  const std::string& hostJsonBody)
{
    AppConfig c;
    QJsonObject rigObj;
    if (readJsonObject(rigFile(configDir(anchorPath)), rigObj) == JsonRead::Ok)
    {
        readRigGlobal(rigObj.value("global").toObject(), c);
        loadAxesArray(rigObj.value("axes").toArray(), c.drives);
        c.numDrives = std::max(1, std::min(rigObj.value("numDrives").toInt(static_cast<int>(c.drives.size())),
                                           static_cast<int>(c.drives.size())));
    }
    if (c.drives.empty())   // host-only validation still needs >=1 axis for the validator
    {
        DriveConfig d; d.slaveIndex = 1; recomputeDerivedFields(d);
        c.drives.push_back(d); c.numDrives = 1;
    }

    QJsonParseError pe;
    QJsonDocument doc = QJsonDocument::fromJson(QString::fromStdString(hostJsonBody).toUtf8(), &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject())
        return { std::string("Invalid host JSON: ") + pe.errorString().toStdString() };

    readHostConfig(doc.object(), c);
    return c.validate();
}

bool Config::isBindableCommand(const std::string& cmd)
{
    // Mirrors Docs/COMMAND_CONTRACT.md "Box v1" column. estop/release,
    // restart, and shutdown are DELIBERATELY absent -- server-side gate,
    // independent of any UI's list. Toggle tokens: one button per stateful
    // pair, resolved server-side with transition + cooldown guards. The
    // discrete halves stay valid (dashboard, scripts, legacy buttons.json
    // maps) but the wizard lists toggles.
    static const char* kBindable[] = {
        "init-toggle", "run-toggle", "park-toggle", "belts-toggle",
        "device-toggle",
        "home", "estop", "reset-fault",
        "init", "deinit", "start", "stop", "park", "unpark",
        "belts/slack", "belts/tension",
    };
    for (const char* b : kBindable) if (cmd == b) return true;
    return false;
}

std::vector<std::string> Config::validateButtonsBody(const std::string& body)
{
    std::vector<std::string> errors;
    QJsonParseError pe;
    QJsonDocument doc = QJsonDocument::fromJson(QString::fromStdString(body).toUtf8(), &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject())
        return { std::string("Invalid buttons JSON: ") + pe.errorString().toStdString() };

    QJsonValue bv = doc.object().value("bindings");
    if (!bv.isArray()) return { "buttons.json: 'bindings' array missing" };

    int i = 0;
    for (const QJsonValue& v : bv.toArray())
    {
        std::string pfx = "binding " + std::to_string(i++) + ": ";
        if (!v.isObject()) { errors.push_back(pfx + "not an object"); continue; }
        QJsonObject o = v.toObject();
        std::string cmd = o.value("cmd").toString().toStdString();
        if (!isBindableCommand(cmd))
            errors.push_back(pfx + "'" + cmd + "' is not a bindable command");
        if (!o.value("code").isDouble() || o.value("code").toInt(-1) < 0)
            errors.push_back(pfx + "'code' must be a non-negative integer");
        if (o.value("vendor").toString().isEmpty() || o.value("product").toString().isEmpty())
            errors.push_back(pfx + "'vendor'/'product' must be non-empty strings");
    }
    return errors;
}

// ============================================================
// AppConfig::validate()
// ============================================================
std::vector<std::string> AppConfig::validate() const
{
    std::vector<std::string> errors;

    if (controlLoopHz < 100 || controlLoopHz > 4000)
        errors.push_back("controlLoopHz=" + std::to_string(controlLoopHz) +
                         " out of range [100, 4000]");
    else if (1000000000 % controlLoopHz != 0)
        // SYNC0 period = 1e9 / Hz (integer ns) - must divide evenly or DC sync
        // drifts from the loop. Valid: 250/500/1000/2000/4000 Hz.
        errors.push_back("controlLoopHz=" + std::to_string(controlLoopHz) +
                         " must divide 1e9 evenly (use 250/500/1000/2000/4000)");

    // A CPU this machine does not have falls back to auto at start (with a
    // warning); only an impossible number is an error here.
    if (rtCore < -1 || rtCore > 255)
        errors.push_back("rtCore=" + std::to_string(rtCore) + " out of range [-1 (auto), 255]");

    if (conditioningMode != "bypass" && conditioningMode != "interpolate" && conditioningMode != "filter")
        errors.push_back("conditioningMode='" + conditioningMode +
                         "' must be bypass | interpolate | filter");

    // 0 = off; a window shorter than a packet period could never mean
    // "nothing changed", so anything on starts at 500 ms.
    if (ncxFrozenMs != 0 && (ncxFrozenMs < 500 || ncxFrozenMs > 60000))
        errors.push_back("ncxFrozenMs=" + std::to_string(ncxFrozenMs) +
                         " must be 0 (off) or in [500, 60000]");

    if (numDrives < 1 || numDrives > 10)
        errors.push_back("numDrives=" + std::to_string(numDrives) +
                         " out of range [1, 10]");

    if (static_cast<int>(drives.size()) < numDrives)
        errors.push_back("drives array has " + std::to_string(drives.size()) +
                         " entries but numDrives=" + std::to_string(numDrives));

    if (pdoWatchdogMs < 0)
        errors.push_back("pdoWatchdogMs must be >= 0");

    if (sync0RecycleRounds < 0 || sync0RecycleRounds > 5)
        errors.push_back("sync0RecycleRounds out of range [0, 5] (0 disables)");

    int checkDrives = std::min(numDrives, static_cast<int>(drives.size()));
    for (int i = 0; i < checkDrives; ++i)
    {
        const DriveConfig& d = drives[i];
        std::string pfx = "Drive " + std::to_string(i + 1) + ": ";

        // encoderCountsPerRev is required for EVERY mode: torque/belt axes consume
        // it too (0x607F velocity-limit counts and the overspeed guard's rpm math
        // divide by it -- a saved 0 would pin the shaft and poison the guard).
        if (d.encoderCountsPerRev <= 0.0)
            errors.push_back(pfx + "encoderCountsPerRev must be > 0");

        if (d.followingErrorWindowMm < 0.0)
            errors.push_back(pfx + "followingErrorWindowMm must be >= 0");

        // Rotary levers: 0x6065 is the drive-side runaway protection, and on a
        // geared lever the engineering unit is degrees. A window wider than the
        // axis's own arc can never trip (the linear default of 100 becomes
        // ~1.8M counts at 50:1), so the protection is disabled in practice.
        // Server-side twin of the web editor's arc cap (app.js dynMax) --
        // geometry, not a magic number.
        if (axisCaps(d.axisType, d.mode).rotary && d.strokeMm > 0.0
            && d.followingErrorWindowMm > d.strokeMm)
            errors.push_back(pfx + "followingErrorWindowMm (" +
                             std::to_string(d.followingErrorWindowMm) +
                             " deg) exceeds the axis arc (" +
                             std::to_string(d.strokeMm) + " deg) -- a rotary "
                             "window wider than its own travel disables "
                             "drive-side runaway protection");

        // Linear/homing/tracking requirements apply only to position axes. The
        // web UI hides all of these fields for torque mode (app.js axisApplicable:
        // pos-only / csp-only), and a belt tensioner legitimately has no
        // ballscrew, stroke, or homing -- ungated, a belt axis with those fields
        // at 0 could never be saved and there would be no field in the UI to fix.
        const bool torqueAxis = (d.mode == "cst" || d.mode == "torque");
        if (!torqueAxis)
        {
        if (d.strokeMm <= 0.0)
            errors.push_back(pfx + "strokeMm must be > 0 (got " +
                             std::to_string(d.strokeMm) + ")");

        if (d.ballscrewPitch <= 0.0)
            errors.push_back(pfx + "ballscrewPitch must be > 0 (got " +
                             std::to_string(d.ballscrewPitch) + ")");

        if (d.countsPerMm <= 0.0)
            errors.push_back(pfx + "countsPerMm must be > 0 (derived from "
                             "encoderCountsPerRev / ballscrewPitch)");

        if (d.homingBackoffMm <= 0.0)
            errors.push_back(pfx + "homingBackoffMm must be > 0");

        if (d.strokeMm > 0.0 && d.homingBackoffMm >= d.strokeMm)
            errors.push_back(pfx + "homingBackoffMm (" +
                             std::to_string(d.homingBackoffMm) +
                             ") must be < strokeMm (" +
                             std::to_string(d.strokeMm) + ")");

        if (d.maxVelocityMmS <= 0.0)
            errors.push_back(pfx + "maxVelocityMmS must be > 0");
        else if (d.maxVelocityMmS > 10000.0)
            errors.push_back(pfx + "maxVelocityMmS must be <= 10000");

        if (d.homingSpeed <= 0.0)
            errors.push_back(pfx + "homingSpeed must be > 0");

        // homingSpeed is a homing-command number, not true mm/s (the achieved
        // homing velocity is the drive's position-loop gain x per-cycle step, see
        // HomingSequence::homingStepMm). So it is NOT bounded by maxVelocityMmS;
        // the per-cycle following error is hard-capped by HOMING_MAX_STEP_MM
        // instead. Keep only a generous sanity ceiling here.
        if (d.homingSpeed > 5000.0)
            errors.push_back(pfx + "homingSpeed must be <= 5000");

        if (d.homingTorquePct < 1 || d.homingTorquePct > 100)
            errors.push_back(pfx + "homingTorquePct=" +
                             std::to_string(d.homingTorquePct) +
                             " out of range [1, 100]");

        if (d.maxAccelerationMmS2 <= 0.0)
            errors.push_back(pfx + "maxAccelerationMmS2 must be > 0");
        else if (d.maxAccelerationMmS2 > 100000.0)
            errors.push_back(pfx + "maxAccelerationMmS2 must be <= 100000");

        // maxJerkMmS3 is retired: its only consumer was the BLENDING s-curve
        // planner, now replaced by the tracking filter (the follower ignores jerk).
        // The field lingers only as a dormant config key for round-trip compatibility;
        // it constrains nothing, so it is no longer validated and no longer bounds Amax.

        // Filter-mode knee ceiling. The exact integrator is stable to the loop
        // Nyquist, but the cap is a MUSICAL/actuator-bandwidth limit, not a
        // stability one: >~125 Hz is shaker/tactile territory, not main-platform,
        // and only risks passing telemetry glitches. Cap at 125 Hz regardless of
        // loop rate (clamped to the loop Nyquist on very slow loops).
        const double wnCap = std::min(125.0, controlLoopHz / 2.0);
        if (d.trackingWnHz <= 0.0)
            errors.push_back(pfx + "trackingWnHz must be > 0");
        else if (d.trackingWnHz > wnCap)
            errors.push_back(pfx + "trackingWnHz=" + std::to_string(d.trackingWnHz) +
                             " exceeds the " + std::to_string(wnCap) +
                             " Hz knee ceiling (actuator-bandwidth cap)");
        }

        if (d.onlineHoldTimeoutSec <= 0.0)
            errors.push_back(pfx + "onlineHoldTimeoutSec must be > 0");

        const AxisCaps kcaps = axisCaps(d.axisType, d.mode);
        if (d.mode == "cst" || d.mode == "torque")
        {
            // Torque values are % of RATED (100 = rated, ~300 = peak); the drive's
            // 0x6072 is the hard ceiling. Peak-region operation is legitimate for
            // belt haptics (rig-calibrated: ~7Nm on a 30mm barrel at 1:1 = firm,
            // not hazardous); the drive's i2t + the relaxer bound the dwell.
            if (d.torqueMinPct < 0.0 || d.torqueMinPct > 300.0)
                errors.push_back(pfx + "torqueMinPct out of range [0, 300]");
            if (d.torqueMaxPct < 0.0 || d.torqueMaxPct > 300.0)
                errors.push_back(pfx + "torqueMaxPct out of range [0, 300]");
            if (d.torqueMinPct >= d.torqueMaxPct)
                errors.push_back(pfx + "torqueMinPct must be < torqueMaxPct");
        }
        // The seven belt guards are BELT-shaped (load-lost physics, strap
        // winding); their ranges bind belt axes only. The device families
        // carry their own guard set inside device{} (validated below), so a
        // shifter is no longer forced into, e.g., a mandatory overspeed trip
        // whose model is "snapped strap free-spins".
        if (kcaps.beltType && (d.mode == "cst" || d.mode == "torque"))
        {
            if (d.beltSlewPctPerSec < 100.0)
                errors.push_back(pfx + "beltSlewPctPerSec must be >= 100 (20000 recommended)");
            if (d.beltOverspeedRpm < 50.0 || d.beltOverspeedRpm > 6000.0)
                errors.push_back(pfx + "beltOverspeedRpm out of range [50, 6000]");
            if (d.beltOverspeedMs < 20.0 || d.beltOverspeedMs > 5000.0)
                errors.push_back(pfx + "beltOverspeedMs out of range [20, 5000]");
            if (d.beltMaxTravelRevs != 0.0 && (d.beltMaxTravelRevs < 0.5 || d.beltMaxTravelRevs > 100.0))
                errors.push_back(pfx + "beltMaxTravelRevs must be 0 (off) or in [0.5, 100]");
            if (d.beltMaxRpm != 0.0 && (d.beltMaxRpm < 100.0 || d.beltMaxRpm > 3000.0))
                errors.push_back(pfx + "beltMaxRpm must be 0 (unlimited) or in [100, 3000]");
            if (d.hapticsMaxMm < 0.0 || d.hapticsMaxMm > 10.0)
                errors.push_back(pfx + "hapticsMaxMm out of range [0, 10]");
            if (d.beltRelaxerSec != 0.0 && (d.beltRelaxerSec < 2.0 || d.beltRelaxerSec > 120.0))
                errors.push_back(pfx + "beltRelaxerSec must be 0 (off) or in [2, 120]");
            if (d.beltRelaxerPct < 20.0 || d.beltRelaxerPct > 100.0)
                errors.push_back(pfx + "beltRelaxerPct out of range [20, 100]");
            // Ratio safety: the strap sees motor torque x reduction, so a
            // torqueMaxPct tuned at 1:1 must be re-derived when the ratio changes.
            // Cap the STRAP-side ceiling at 300%-of-rated-at-1:1 equivalent.
            {
                double rf = 1.0;
                if (!d.reductionRatio.empty())
                    rf = std::max(1.0, std::atof(d.reductionRatio.c_str()));
                if (d.torqueMaxPct * rf > 300.0)
                    errors.push_back(pfx + "torqueMaxPct x reduction exceeds 300% of rated at the strap -- "
                                           "reduce torqueMaxPct to compensate for the gearing");
            }
        }

        // ---- Control-loading device families (shifter/pedal) ----
        if (kcaps.isDevice())
        {
            const DeviceParams& p = d.device;
            if (d.mode != "torque")
                errors.push_back(pfx + "device family axes (shifter/pedal) require mode \"torque\"");
            if (p.dir != 1.0 && p.dir != -1.0)
                errors.push_back(pfx + "device.dir must be +1 or -1");
            if (p.homeDir != 1.0 && p.homeDir != -1.0)
                errors.push_back(pfx + "device.homeDir must be +1 or -1");
            if (p.stopMinRev >= p.stopMaxRev)
                errors.push_back(pfx + "device.stopMinRev must be < device.stopMaxRev");
            if (p.neutralRev < p.stopMinRev || p.neutralRev > p.stopMaxRev)
                errors.push_back(pfx + "device.neutralRev must lie between the stops");
            for (double det : p.detents)
                if (det < p.stopMinRev || det > p.stopMaxRev)
                { errors.push_back(pfx + "device.detents entries must lie between the stops"); break; }
            auto monotonicX = [](const std::vector<CurveNode>& c)
            {
                for (size_t i = 1; i < c.size(); ++i)
                    if (c[i].x <= c[i - 1].x) return false;
                return true;
            };
            if (!monotonicX(p.springCurve))
                errors.push_back(pfx + "device.springCurve x values must be strictly increasing");
            if (!monotonicX(p.detentCurve))
                errors.push_back(pfx + "device.detentCurve x values must be strictly increasing");
            if (p.maxForcePct <= 0.0 || p.maxForcePct > 300.0)
                errors.push_back(pfx + "device.maxForcePct out of range (0, 300]");
            if (p.homeTorquePct < 5.0 || p.homeTorquePct > 100.0)
                errors.push_back(pfx + "device.homeTorquePct out of range [5, 100]");
            if (p.slewPctPerSec < 100.0)
                errors.push_back(pfx + "device.slewPctPerSec must be >= 100");
            if (p.thermalDwellSec != 0.0 && (p.thermalDwellSec < 1.0 || p.thermalDwellSec > 300.0))
                errors.push_back(pfx + "device.thermalDwellSec must be 0 (off) or in [1, 300]");
            if (p.thermalPct < 20.0 || p.thermalPct > 100.0)
                errors.push_back(pfx + "device.thermalPct out of range [20, 100]");
            if (p.foldRpm != 0.0 && (p.foldRpm < 100.0 || p.foldRpm > 6000.0))
                errors.push_back(pfx + "device.foldRpm must be 0 (off) or in [100, 6000]");
            if (p.lashRev < 0.0 || p.lashRev >= (p.stopMaxRev - p.stopMinRev) / 2.0)
                errors.push_back(pfx + "device.lashRev must be >= 0 and smaller than half the travel");
            if (p.velLpfHz < 0.0)
                errors.push_back(pfx + "device.velLpfHz must be >= 0");
            if (p.dampPctPerRevS < 0.0 || p.stopDamp < 0.0 || p.stopSpring < 0.0)
                errors.push_back(pfx + "device damping/stop values must be >= 0");
            // State-layer effect character (all inert at defaults).
            if (p.clutchBitePct < 0.0 || p.clutchBitePct > 100.0)
                errors.push_back(pfx + "device.clutchBitePct out of range [0, 100]");
            if (p.blockGain < 0.0 || p.blockGain > 10.0)
                errors.push_back(pfx + "device.blockGain out of range [0, 10]");
            if (p.grindAmpPct < 0.0 || p.grindAmpPct > 100.0)
                errors.push_back(pfx + "device.grindAmpPct out of range [0, 100]");
            if (p.grindFreqHz <= 0.0 || p.grindFreqHz > 500.0)
                errors.push_back(pfx + "device.grindFreqHz out of range (0, 500]");
            if (p.blockStartRev < 0.0 || p.blockStartRev > (p.stopMaxRev - p.stopMinRev))
                errors.push_back(pfx + "device.blockStartRev must be >= 0 and within the travel");
            if (p.rpmMatchPct < 0.0 || p.rpmMatchPct > 50.0)
                errors.push_back(pfx + "device.rpmMatchPct out of range [0, 50]");
            if (p.frictionPct < 0.0 || p.frictionPct > 50.0)
                errors.push_back(pfx + "device.frictionPct out of range [0, 50]");
            if (p.breakoutScale < 0.2 || p.breakoutScale > 10.0)
                errors.push_back(pfx + "device.breakoutScale out of range [0.2, 10]");
        }
    }

    // ---- Haptic effect layer (rig global) ----
    // Every tile's settings are held to its registry rows (HapticsRegistry.h
    // ParamSpec: the ranges and the choices the strip offers), through the
    // same key-to-field table the config reads and writes with: the file
    // can hold what the strip can set, and a setting added to the registry
    // is checked with no line here. Then the keys every tile carries where
    // a tile offers no row for them, the routes, and the one rule between
    // two settings (an impact's full above its start).
    {
        const auto num = [](double v) { char b[32]; std::snprintf(b, sizeof b, "%g", v); return std::string(b); };
        for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
        {
            const haptics::EffectInfo& info = haptics::effectInfo(i);
            const haptics::EffectParams& p = hapticsFx[static_cast<size_t>(i)];
            const std::string pfx = std::string("haptics.") + info.key + ": ";
            for (int k = 0; k < info.paramCount; ++k)
            {
                const haptics::ParamSpec& ps = info.params[k];
                const double* f = Config::hapticsField(*this, i, ps.key);
                if (!f) continue;   // every row has its field (TestConfigTwoFile's contract test)
                if (!(*f >= ps.min && *f <= ps.max))   // a NaN fails too
                    errors.push_back(pfx + ps.key + " out of range [" + num(ps.min) + ", " + num(ps.max) + "]");
                else if (ps.opts && *f != std::floor(*f))
                    errors.push_back(pfx + ps.key + " must be one of its choices (" + ps.opts + ": 0, 1, ...)");
            }
            const auto offers = [&info](const char* key)
            {
                for (int k = 0; k < info.paramCount; ++k)
                    if (std::strcmp(info.params[k].key, key) == 0) return true;
                return false;
            };
            // Carried by every tile, offered by some: where the tile offers
            // no row for it, held to its widest use.
            if (!offers("freqHz") && p.ampPct > 0.0 && p.freqHz > 0.0 && (p.freqHz < 4.0 || p.freqHz > 500.0))
                errors.push_back(pfx + "freqHz out of range [4, 500]");
            if (!offers("jitter") && (p.jitter < 0.0 || p.jitter > 1.0))
                errors.push_back(pfx + "jitter out of range [0, 1]");
            // Up to 400: on the combined-slip path 100 is only the wire's
            // convention for "let go", and a car can read well past it.
            if (!offers("peakPct") && (p.peakPct < 1.0 || p.peakPct > 400.0))
                errors.push_back(pfx + "peakPct out of range [1, 400]");
            for (const haptics::Route& r : p.routes)
            {
                if (r.axis == -1 && r.shaker == -1 && r.gain <= 0.0) continue;   // unused slot
                if (r.shaker >= haptics::MAX_SHAKER_OUT)
                    errors.push_back(pfx + "route shaker out of range");
                if (r.harm < 1 || r.harm > 8)
                    errors.push_back(pfx + "route harm out of range [1, 8]");
                if (r.shaker < 0 && (r.axis < 0 || r.axis >= haptics::MAX_HAPTIC_AXES))
                    errors.push_back(pfx + "route axis out of range");
                if (r.gain < 0.0)
                    errors.push_back(pfx + "route gain must be >= 0");
                // The gain's unit follows the destination: a torque axis
                // or a shaker takes a plain multiplier (0..2); a position
                // axis takes millimetres, capped by that axis's own
                // hapticsMaxMm (the route editor offers the same cap).
                else if (r.shaker >= 0 || r.axis < 0 || static_cast<size_t>(r.axis) >= drives.size())
                {
                    if (r.gain > 2.0) errors.push_back(pfx + "route gain out of range [0, 2]");
                }
                else
                {
                    const DriveConfig& d = drives[static_cast<size_t>(r.axis)];
                    if (d.mode == "torque")
                    {
                        if (r.gain > 2.0)
                            errors.push_back(pfx + "route gain to " + d.name + " out of range [0, 2] x");
                    }
                    else if (r.gain > d.hapticsMaxMm)
                        errors.push_back(pfx + "route gain to " + d.name + " is " + std::to_string(r.gain).substr(0, 4)
                                         + " mm, above that axis's haptic max of " + std::to_string(d.hapticsMaxMm).substr(0, 4) + " mm");
                }
            }
            if (info.kind == haptics::Kind::Impacts && hapticsImpacts.fullMmS <= hapticsImpacts.fromMmS)
                errors.push_back(pfx + "fullMmS must be above fromMmS");
        }
        if (hapticsMasterGain < 0.0 || hapticsMasterGain > 2.0)
            errors.push_back("haptics.masterGain out of range [0, 2]");
        if (hapticsPositionBudget < 0.0 || hapticsPositionBudget > 1.0)
            errors.push_back("haptics.positionBudget out of range [0, 1]");
        if (hapticsAxisDelayMs < 0.0 || hapticsAxisDelayMs > 60.0)
            errors.push_back("haptics.axisDelayMs out of range [0, 60]");
        if (audioChannels < 1 || audioChannels > haptics::MAX_SHAKER_OUT)
            errors.push_back("audioChannels out of range [1, 8]");
        {
            int total = 0;
            for (size_t i = 0; i < audioDevices.size(); ++i)
            {
                if (audioDevices[i].channels < 1 || audioDevices[i].channels > haptics::MAX_SHAKER_OUT)
                    errors.push_back("audioDevices[" + std::to_string(i) + "].channels out of range [1, 8]");
                total += audioDevices[i].channels;
            }
            if (total > haptics::MAX_SHAKER_OUT)
                errors.push_back("audioDevices: more than 8 shaker channels in total");
        }
    }

    // ---- NULLCATX channel bindings (rig global) ----
    {
        std::vector<std::string> seen;
        for (size_t i = 0; i < ncxBindings.size(); ++i)
        {
            const NcxBinding& b = ncxBindings[i];
            const std::string pfx = "ncxBindings[" + std::to_string(i) + "]: ";
            if (ncxTokenIndexN(b.token.data(), static_cast<int>(b.token.size())) < 0)
            {
                std::string list;
                for (int t = 0; t < NcxTok::TokenCount; ++t) list += std::string(t ? ", " : "") + ncxTokenName(t);
                errors.push_back(pfx + "unknown token \"" + b.token + "\" (" + list + ")");
            }
            if (b.slot < 0 || b.slot >= MAX_NCX_CHANNELS)
                errors.push_back(pfx + "slot out of range [0, " + std::to_string(MAX_NCX_CHANNELS - 1) + "]");
            if (b.scale == 0.0)
                errors.push_back(pfx + "scale must be nonzero");
            for (const std::string& s : seen)
                if (s == b.token)
                { errors.push_back(pfx + "token \"" + b.token + "\" bound twice"); break; }
            seen.push_back(b.token);
        }
    }

    return errors;
}
