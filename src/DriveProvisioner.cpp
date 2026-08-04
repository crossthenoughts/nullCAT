// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// DriveProvisioner.cpp — see DriveProvisioner.h
// ============================================================
#include "DriveProvisioner.h"
#include "EtherCATMaster.h"
#include "Logging.h"

#include "soem/ec_options.h"
#include "soem/ec_type.h"
#include "nicdrv.h"
#include "soem/ec_base.h"
#include "soem/ec_main.h"
#include "soem/ec_coe.h"

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QString>

#include <fstream>
#include <ctime>
#include <cstdio>

namespace prov {

namespace {

constexpr int kSdoTimeoutUs = 700000;

// "0x2001:0x41" -> index/sub. Returns false on malformed input.
bool parseCoE(const std::string& s, uint16_t& index, uint8_t& sub)
{
    auto colon = s.find(':');
    if (colon == std::string::npos) return false;
    try {
        index = static_cast<uint16_t>(std::stoul(s.substr(0, colon), nullptr, 16));
        sub   = static_cast<uint8_t >(std::stoul(s.substr(colon + 1), nullptr, 16));
    } catch (...) { return false; }
    return true;
}

bool typeInfo(const std::string& t, int& bytes, bool& isSigned)
{
    if      (t == "u8")  { bytes = 1; isSigned = false; }
    else if (t == "u16") { bytes = 2; isSigned = false; }
    else if (t == "u32") { bytes = 4; isSigned = false; }
    else if (t == "i16") { bytes = 2; isSigned = true;  }
    else if (t == "i32") { bytes = 4; isSigned = true;  }
    else return false;
    return true;
}

// Parse one JSON object into a Param. `valueKey` lets inertia use "baseline".
bool parseParam(const QJsonObject& o, Param& p, std::string& err, const char* valueKey = "value")
{
    p.coe   = o.value("coe").toString().toStdString();
    p.panel = o.value("panel").toString().toStdString();
    p.name  = o.value("name").toString().toStdString();
    if (!parseCoE(p.coe, p.index, p.sub)) { err = "bad coe: " + p.coe; return false; }
    if (!typeInfo(o.value("type").toString("u16").toStdString(), p.bytes, p.isSigned)) {
        err = "bad type for " + p.coe; return false;
    }
    p.value = static_cast<int64_t>(o.value(valueKey).toDouble());
    return true;
}

} // namespace

bool Profile::load(const std::string& path, std::string& err)
{
    QFile f(QString::fromStdString(path));
    if (!f.open(QIODevice::ReadOnly)) { err = "cannot open " + path; return false; }
    QJsonParseError pe;
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
        err = "JSON parse error in " + path + ": " + pe.errorString().toStdString();
        return false;
    }
    QJsonObject root = doc.object();
    name = root.value("name").toString().toStdString();

    if (!parseParam(root.value("unlock").toObject(), unlock, err)) return false;
    if (!parseParam(root.value("store").toObject(),  store,  err)) return false;

    QJsonObject carrier = root.value("carrier").toObject();
    if (carrier.contains("doubler")) {
        if (!parseParam(carrier.value("doubler").toObject(), carrierDoubler, err)) return false;
        hasCarrier = true;
    }

    QJsonObject perAxis = root.value("perAxis").toObject();
    if (perAxis.contains("inertia")) {
        if (!parseParam(perAxis.value("inertia").toObject(), inertiaBaseline, err, "baseline")) return false;
        hasInertia = true;
    }

    QJsonArray arr = root.value("clone").toArray();
    if (arr.isEmpty()) { err = "clone[] is empty"; return false; }
    for (const QJsonValue& v : arr) {
        Param p;
        if (!parseParam(v.toObject(), p, err)) return false;
        clone.push_back(p);
    }

    // torqueOnly[] is optional (older profiles lack it).
    for (const QJsonValue& v : root.value("torqueOnly").toArray()) {
        Param p;
        if (!parseParam(v.toObject(), p, err)) return false;
        torqueOnly.push_back(p);
    }
    return true;
}

// ---- SOEM access -------------------------------------------------------------

bool Provisioner::writeParam(int slave, const Param& p, std::string& err)
{
    ecx_contextt* ctx = m_master.getContext();
    if (!ctx) { err = "no SOEM context"; return false; }
    uint8_t buf[8] = {0};
    for (int i = 0; i < p.bytes; ++i) buf[i] = static_cast<uint8_t>((p.value >> (8 * i)) & 0xFF);
    int wkc = ecx_SDOwrite(ctx, static_cast<uint16>(slave), p.index, p.sub, FALSE,
                           p.bytes, buf, kSdoTimeoutUs);
    if (wkc <= 0) { err = p.coe + " write wkc=" + std::to_string(wkc); return false; }
    return true;
}

bool Provisioner::readParam(int slave, const Param& p, int64_t& out, std::string& err)
{
    ecx_contextt* ctx = m_master.getContext();
    if (!ctx) { err = "no SOEM context"; return false; }
    uint8_t buf[8] = {0};
    int sz = p.bytes;
    int wkc = ecx_SDOread(ctx, static_cast<uint16>(slave), p.index, p.sub, FALSE,
                          &sz, buf, kSdoTimeoutUs);
    if (wkc <= 0) { err = p.coe + " read wkc=" + std::to_string(wkc); return false; }
    uint64_t u = 0;
    for (int i = 0; i < p.bytes; ++i) u |= static_cast<uint64_t>(buf[i]) << (8 * i);
    if (p.isSigned) {
        if      (p.bytes == 1) out = static_cast<int8_t >(u);
        else if (p.bytes == 2) out = static_cast<int16_t>(u);
        else                   out = static_cast<int32_t>(u);
    } else {
        out = static_cast<int64_t>(u);
    }
    return true;
}

bool Provisioner::preconditionOk(int slave, std::string& why)
{
    if (!m_master.isInitialized())      { why = "master not initialised"; return false; }
    if (m_master.isRtLoopActive())      { why = "RT loop is running — stop the control loop first"; return false; }
    if (m_master.isOperational())       { why = "master is in OP — provisioning requires PreOp (stop the loop)"; return false; }
    if (slave < 1 || slave > m_master.getSlaveCount()) { why = "slave index out of range"; return false; }
    return true;
}

std::string Provisioner::writeBackupFile(int slave, const std::vector<Param>& params)
{
    std::time_t t = std::time(nullptr);
    char ts[32]; std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&t));
    char path[160];
    std::snprintf(path, sizeof(path), "logs/provision_backup_slave%d_%s.txt", slave, ts);

    std::ofstream of(path);
    if (!of.is_open()) { LOG_WARNING(strf("DriveProvisioner: could not open backup file %s", path)); return {}; }
    of << "# nullCAT provisioning before-state backup\n";
    of << "# slave=" << slave << " time=" << ts << "\n";
    of << "# coe | panel | name | value-before\n";
    for (const Param& p : params) {
        int64_t v = 0; std::string err;
        if (readParam(slave, p, v, err)) of << p.coe << " | " << p.panel << " | " << p.name << " | " << v << "\n";
        else                             of << p.coe << " | " << p.panel << " | " << p.name << " | READ_FAILED(" << err << ")\n";
    }
    of.close();
    LOG_INFO(strf("DriveProvisioner: before-state backup written to %s", path));
    return path;
}

// ---- provision ---------------------------------------------------------------

Result Provisioner::provision(int slave, const Profile& p, bool torqueAxis)
{
    Result r;
    std::string why;
    if (!preconditionOk(slave, why)) { r.message = "refused: " + why; return r; }

    std::lock_guard<std::mutex> xfer(m_master.sdoTransferMutex());

    // The apply-list: clone + baseline inertia, plus torqueOnly[] (C06.20 runaway
    // protection = 0) when the rig config says this slave is a torque axis. All are
    // public OD entries (no unlock needed). The carrier (R22.38 / 0x2022) is
    // deliberately NOT touched here — it is not SDO-writable on this drive even after
    // the C00.31=1107 unlock, so 16 kHz is a manual panel step. See provision_main
    // --fresh note.
    std::vector<Param> apply = p.clone;
    if (p.hasInertia) apply.push_back(p.inertiaBaseline);
    if (torqueAxis)
        apply.insert(apply.end(), p.torqueOnly.begin(), p.torqueOnly.end());

    // Durable before-state backup.
    r.backupFile = writeBackupFile(slave, apply);

    // Write the config set.
    for (const Param& w : apply) {
        WriteResult wr; wr.coe = w.coe; wr.panel = w.panel; wr.name = w.name; wr.wrote = w.value;
        std::string e;
        if (!writeParam(slave, w, e)) LOG_WARNING(strf("DriveProvisioner: %s write failed: %s", w.coe.c_str(), e.c_str()));
        r.writes.push_back(wr);
    }

    // Readback-verify the config set.
    bool allVerified = true;
    for (size_t i = 0; i < apply.size(); ++i) {
        int64_t rb = 0; std::string e;
        if (readParam(slave, apply[i], rb, e)) { r.writes[i].readback = rb; r.writes[i].verified = (rb == apply[i].value); }
        else                                   { r.writes[i].readback = 0;  r.writes[i].verified = false; }
        if (!r.writes[i].verified) {
            allVerified = false;
            LOG_WARNING(strf("DriveProvisioner: VERIFY MISMATCH %s (%s): wrote %lld read %lld",
                             apply[i].coe.c_str(), apply[i].panel.c_str(),
                             (long long)apply[i].value, (long long)r.writes[i].readback));
        }
    }

    // STORE only if every CONFIG readback verified (carrier is NOT part of the gate).
    if (allVerified) {
        std::string e;
        if (writeParam(slave, p.store, e)) {
            r.stored = true; r.ok = true;
            r.message = strf("slave %d: %zu config params written + verified, stored to NVRAM. POWER-CYCLE the drive, then --verify.",
                             slave, apply.size());
            LOG_INFO("DriveProvisioner: " + r.message);
        } else {
            r.message = "all config verified but STORE write failed: " + e;
            LOG_ERROR("DriveProvisioner: " + r.message);
        }
    } else {
        r.ok = false; r.stored = false;
        r.message = strf("slave %d: config readback verify FAILED — NVRAM NOT written. Before-state: %s",
                         slave, r.backupFile.c_str());
        LOG_ERROR("DriveProvisioner: " + r.message);
    }
    return r;
}

// ---- verify (post power-cycle) -----------------------------------------------

Result Provisioner::verify(int slave, const Profile& p, bool torqueAxis)
{
    Result r;
    std::string why;
    if (!preconditionOk(slave, why)) { r.message = "refused: " + why; return r; }

    std::lock_guard<std::mutex> xfer(m_master.sdoTransferMutex());

    std::vector<Param> check = p.clone;
    if (p.hasInertia) check.push_back(p.inertiaBaseline);
    if (torqueAxis)
        check.insert(check.end(), p.torqueOnly.begin(), p.torqueOnly.end());
    // carrier (0x2022) is a panel-only step, so it is NOT part of the config verify set.

    bool all = true;
    for (const Param& c : check) {
        WriteResult wr; wr.coe = c.coe; wr.panel = c.panel; wr.name = c.name; wr.wrote = c.value;
        int64_t rb = 0; std::string e;
        if (readParam(slave, c, rb, e)) { wr.readback = rb; wr.verified = (rb == c.value); }
        else                            { wr.verified = false; }
        if (!wr.verified) all = false;
        r.writes.push_back(wr);
    }
    r.ok = all;
    r.message = all ? strf("slave %d: all %zu params match the profile.", slave, check.size())
                    : strf("slave %d: MISMATCH — drive does not match the profile (see per-param).", slave);
    return r;
}

} // namespace prov
