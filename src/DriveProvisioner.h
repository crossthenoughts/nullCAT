// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// DriveProvisioner.h — drive-side parameter provisioning (PreOp, SDO).
//
// Writes the verified nullCAT parameter set into a drive's vendor object dictionary
// and persists it to NVRAM. Runs ONLY in PreOp with the RT loop stopped (same regime
// as the init-time 0x6072/0x6065 writes). Blocking ecx_SDOwrite/read under the
// master's sdoTransferMutex. No RT / hot-path involvement.
//
// Safety model:
//  1. STORE (0x1010:01="save") is issued ONLY if every readback verifies — any
//     mismatch aborts before NVRAM is touched.
//  2. The store signature is the CiA-301 "save" magic (0x65766173), profile-driven.
//  3. The carrier doubler (R22.38 / 0x2022) is NOT written here. It is not SDO-writable
//     on this drive even after the C00.31=1107 unlock (writes return wkc=0), so the
//     16 kHz step is done by hand on the drive panel. unlock/carrier stay in the profile
//     for reference only.
//  4. A durable before-state backup FILE is written before any change.
//  5. Writes parameters only — no inertia auto-ID motion (that is a separate
//     guarded step). Inertia is written at its baseline value.
//  6. The profile's clone[] values are generated from a known-good drive dump.
//
// Two-phase: provision() writes+stores; the operator power-cycles the drive; verify()
// reads everything back to confirm it persisted.
// ============================================================

#include <cstdint>
#include <string>
#include <vector>

class EtherCATMaster;

namespace prov {

// One object-dictionary entry to write/verify.
struct Param
{
    uint16_t    index    = 0;
    uint8_t     sub      = 0;
    int         bytes    = 2;        // 1 / 2 / 4
    bool        isSigned = false;
    int64_t     value    = 0;        // target value
    std::string coe;                 // "0x2001:0x41" (for logging)
    std::string panel;               // "C01.40"
    std::string name;
};

struct Profile
{
    std::string        name;
    Param              unlock;                 // C00.31 = 1107 (gates hidden carrier obj)
    Param              store;                   // 0x1010:01 = 0x65766173 ("save")
    Param              carrierDoubler;          // R22.38 = 1
    bool               hasCarrier  = false;
    Param              inertiaBaseline;         // C00.06 = 100
    bool               hasInertia  = false;
    std::vector<Param> clone;                   // the 18 verified params
    std::vector<Param> torqueOnly;               // extra one-time writes for torque axes
                                                 // (C06.20 runaway protection = 0)

    // Load + validate from the shipped JSON. Returns false + err on any problem.
    bool load(const std::string& path, std::string& err);
};

struct WriteResult
{
    std::string coe, panel, name;
    int64_t     wrote = 0, readback = 0;
    bool        verified = false;
};

struct Result
{
    bool                     ok = false;
    std::string              message;
    std::string              backupFile;   // durable before-state backup path
    std::vector<WriteResult> writes;        // per-param wrote/readback/verified
    bool                     stored = false;
};

class Provisioner
{
public:
    explicit Provisioner(EtherCATMaster& master) : m_master(master) {}

    // Provision one slave. Refuses unless: master initialised, at PreOp/SafeOp (not OP),
    // RT loop stopped. Sequence: precondition -> backup file -> write clone (+ baseline
    // inertia, + torqueOnly[] iff torqueAxis) -> readback-verify ALL -> STORE only if
    // zero mismatch. torqueAxis comes from the rig config's mode for this slave
    // (provision_main resolves it). The carrier (R22.38) is NOT written — it is not
    // SDO-writable on this drive; 16 kHz is a manual panel step.
    Result provision(int slave, const Profile& p, bool torqueAxis = false);

    // Readback clone[] (+ inertia, + torqueOnly[] iff torqueAxis) and compare to the
    // profile. For the post-power-cycle confirmation. Does NOT write anything.
    Result verify(int slave, const Profile& p, bool torqueAxis = false);

private:
    EtherCATMaster& m_master;

    bool preconditionOk(int slave, std::string& why);
    bool writeParam(int slave, const Param& p, std::string& err);          // mutex held by caller
    bool readParam (int slave, const Param& p, int64_t& out, std::string& err);
    std::string writeBackupFile(int slave, const std::vector<Param>& params);
};

} // namespace prov
