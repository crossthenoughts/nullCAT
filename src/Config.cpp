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

#include <QString>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include <QHash>

#include <algorithm>
#include <cstdlib>
#include <cctype>
#include <stdexcept>

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
    obj["tempPollSec"]              = c.tempPollSec;
    obj["telemetryPort"]               = c.telemetryPort;
    obj["telemetryBindAddr"]           = QString::fromStdString(c.telemetryBindAddr);
    obj["webPort"]                  = c.webPort;
    obj["webBindAddr"]              = QString::fromStdString(c.webBindAddr);
    { QJsonArray a; for (const auto& h : c.webAllowedHosts) a.append(QString::fromStdString(h));
      obj["webAllowedHosts"] = a; }
    obj["webUIEnabled"]             = c.webUIEnabled;
    obj["webShowDevices"]           = c.webShowDevices;
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
    obj["gpioLedRunPin"]            = c.gpioLedRunPin;
    obj["gpioLedReadyPin"]          = c.gpioLedReadyPin;
    obj["gpioLedFaultPin"]          = c.gpioLedFaultPin;
}

static void readHostConfig(const QJsonObject& obj, AppConfig& c)
{
    if (obj.contains("nicName"))                  c.nicName                  = obj.value("nicName").toString().toStdString();
    if (obj.contains("controlLoopHz"))            c.controlLoopHz            = obj.value("controlLoopHz").toInt(500);
    if (obj.contains("tempPollSec"))              c.tempPollSec              = obj.value("tempPollSec").toDouble(15.0);
    if (obj.contains("telemetryPort"))               c.telemetryPort               = obj.value("telemetryPort").toInt(4444);
    if (obj.contains("telemetryBindAddr"))           c.telemetryBindAddr           = obj.value("telemetryBindAddr").toString("").toStdString();  // "" = platform default
    if (obj.contains("webPort"))                  c.webPort                  = obj.value("webPort").toInt(8080);
    if (obj.contains("webBindAddr"))              c.webBindAddr              = obj.value("webBindAddr").toString("127.0.0.1").toStdString();
    if (obj.contains("webAllowedHosts"))
    {
        c.webAllowedHosts.clear();
        for (const auto& v : obj.value("webAllowedHosts").toArray())
            if (!v.toString().isEmpty()) c.webAllowedHosts.push_back(v.toString().toStdString());
    }
    if (obj.contains("webUIEnabled"))             c.webUIEnabled             = obj.value("webUIEnabled").toBool(false);
    if (obj.contains("webShowDevices"))           c.webShowDevices           = obj.value("webShowDevices").toBool(false);
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
}

static void readRigGlobal(const QJsonObject& obj, AppConfig& c)
{
    if (obj.contains("conditioningMode"))         c.conditioningMode         = obj.value("conditioningMode").toString("bypass").toStdString();
    if (obj.contains("blendTimeSec"))             c.blendTimeSec             = obj.value("blendTimeSec").toDouble(2.0);
    if (obj.contains("blendMaxVelocityMmS"))      c.blendMaxVelocityMmS      = obj.value("blendMaxVelocityMmS").toDouble(20.0);
    if (obj.contains("requireUserFaultReset"))    c.requireUserFaultReset    = obj.value("requireUserFaultReset").toBool(false);
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
    o["velLpfHz"]        = p.velLpfHz;
    o["maxForcePct"]     = p.maxForcePct;
    o["homeTorquePct"]   = p.homeTorquePct;
    o["homeDir"]         = p.homeDir;
    o["slewPctPerSec"]   = p.slewPctPerSec;
    o["thermalDwellSec"] = p.thermalDwellSec;
    o["thermalPct"]      = p.thermalPct;
    o["foldRpm"]         = p.foldRpm;
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
    rdDbl(o, "velLpfHz",        p.velLpfHz);
    rdDbl(o, "maxForcePct",     p.maxForcePct);
    rdDbl(o, "homeTorquePct",   p.homeTorquePct);
    rdDbl(o, "homeDir",         p.homeDir);
    rdDbl(o, "slewPctPerSec",   p.slewPctPerSec);
    rdDbl(o, "thermalDwellSec", p.thermalDwellSec);
    rdDbl(o, "thermalPct",      p.thermalPct);
    rdDbl(o, "foldRpm",         p.foldRpm);
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
    seedDefaultDrive();
}

// Seed the default Drive 1 WITHOUT resetting m_config. load() calls this when
// host.json exists but rig.json does not (a fresh install with a seeded
// host.json): a full setDefaults() there would wipe the host fields that were
// just read.
void Config::seedDefaultDrive()
{
    DriveConfig d;
    d.slaveIndex = 1;
    d.name       = "Drive 1";
    recomputeDerivedFields(d);
    m_config.drives.clear();
    m_config.drives.push_back(d);
    m_config.numDrives = 1;
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

static bool readJsonObject(const QString& path, QJsonObject& out)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return false;
    QByteArray data = f.readAll();
    f.close();
    QJsonParseError pe;
    QJsonDocument doc = QJsonDocument::fromJson(data, &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject()) return false;
    out = doc.object();
    return true;
}
static bool writeJsonObject(const QString& path, const QJsonObject& obj)
{
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return false;
    f.write(QJsonDocument(obj).toJson(QJsonDocument::Indented));
    f.close();
    return true;
}
// Overlay schema keys onto dst, leaving unknown keys already in dst untouched
// (merge-on-save: user annotations / future-build fields survive).
static void overlayKeys(QJsonObject& dst, const QJsonObject& schema)
{
    for (auto it = schema.constBegin(); it != schema.constEnd(); ++it)
        dst[it.key()] = it.value();
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

bool Config::load(const std::string& anchorPath)
{
    const QString dir    = configDir(anchorPath);

    QJsonObject hostObj, rigObj;
    const bool haveHost = readJsonObject(hostFile(dir), hostObj);
    const bool haveRig  = readJsonObject(rigFile(dir),  rigObj);

    if (haveHost || haveRig)
    {
        // ---- native two-file format (the steady state) ----
        m_config = AppConfig{};
        if (haveHost) readHostConfig(hostObj, m_config);
        if (haveRig)
        {
            m_config.configVersion = rigObj.value("configVersion").toInt(2);
            readRigGlobal(rigObj.value("global").toObject(), m_config);
            loadAxesArray(rigObj.value("axes").toArray(), m_config.drives);
            int nd = rigObj.value("numDrives").toInt(static_cast<int>(m_config.drives.size()));
            m_config.numDrives = std::max(1, std::min(nd, static_cast<int>(m_config.drives.size())));
        }
        if (m_config.drives.empty()) seedDefaultDrive();
        m_lastError.clear();
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
// unknown keys already in the file.
bool Config::saveHost(const std::string& anchorPath) const
{
    const QString path = hostFile(configDir(anchorPath));
    QJsonObject existing;
    readJsonObject(path, existing);            // ignore failure: empty -> fresh write
    QJsonObject schema;
    writeHostConfig(m_config, schema);
    overlayKeys(existing, schema);
    return writeJsonObject(path, existing);
}

// rig.json - single writer (the web UI on both platforms). { configVersion,
// numDrives, global:{...}, axes:[...] }. Unknown keys in global and per-axis
// survive (merge-on-save); axes merge by slaveIndex.
bool Config::saveRig(const std::string& anchorPath) const
{
    const QString path = rigFile(configDir(anchorPath));
    QJsonObject existing;
    readJsonObject(path, existing);

    QJsonObject globalMerged = existing.value("global").toObject();
    QJsonObject globalSchema;
    writeRigGlobal(m_config, globalSchema);
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
    if (readJsonObject(hostFile(configDir(anchorPath)), hostObj))
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

std::vector<std::string> Config::validateHostBody(const std::string& anchorPath,
                                                  const std::string& hostJsonBody)
{
    AppConfig c;
    QJsonObject rigObj;
    if (readJsonObject(rigFile(configDir(anchorPath)), rigObj))
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

    if (conditioningMode != "bypass" && conditioningMode != "interpolate" && conditioningMode != "filter")
        errors.push_back("conditioningMode='" + conditioningMode +
                         "' must be bypass | interpolate | filter");

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
                errors.push_back(pfx + "beltSlewPctPerSec must be >= 100 (haptics need ~1000+; 3000 recommended)");
            if (d.beltOverspeedRpm < 50.0 || d.beltOverspeedRpm > 6000.0)
                errors.push_back(pfx + "beltOverspeedRpm out of range [50, 6000]");
            if (d.beltOverspeedMs < 20.0 || d.beltOverspeedMs > 5000.0)
                errors.push_back(pfx + "beltOverspeedMs out of range [20, 5000]");
            if (d.beltMaxTravelRevs != 0.0 && (d.beltMaxTravelRevs < 0.5 || d.beltMaxTravelRevs > 100.0))
                errors.push_back(pfx + "beltMaxTravelRevs must be 0 (off) or in [0.5, 100]");
            if (d.beltMaxRpm != 0.0 && (d.beltMaxRpm < 100.0 || d.beltMaxRpm > 3000.0))
                errors.push_back(pfx + "beltMaxRpm must be 0 (unlimited) or in [100, 3000]");
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
        }
    }

    return errors;
}
