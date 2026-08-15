// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// provision_main.cpp - nullCAT standalone drive-provisioning tool.
//
// Run with the master service STOPPED (it owns the NIC), exactly like slaveinfo:
//   sudo systemctl stop nullcat-pi
//   sudo ./provision eth0 4 --fresh
//   (power-cycle the drive)
//   sudo ./provision eth0 4 --verify
//
// It brings the bus to SafeOp (master.initialize, NOT OP), then runs the
// DriveProvisioner sequence (backup -> write -> readback-verify-all -> STORE only on
// zero mismatch). No motion, no control loop. The 16 kHz carrier (R22.38) is a manual
// panel step (not SDO-writable) - --fresh just prints the reminder. See DriveProvisioner.h.
// ============================================================
#include "EtherCATMaster.h"
#include "DriveProvisioner.h"
#include "Config.h"
#include "Logging.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include <limits.h>
#include <libgen.h>

static std::string exeDir()
{
    char buf[PATH_MAX];
    ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return ".";
    buf[n] = 0;
    return std::string(::dirname(buf));
}

static const char* st(bool b) { return b ? "OK" : "**FAIL**"; }

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        printf("nullCAT drive provisioning tool\n\n");
        printf("Usage: %s <ifname> <slave> [--fresh] [--verify] [--config PATH] [--profile PATH]\n\n", argv[0]);
        printf("  <slave>    1-based slave index on the bus (run slaveinfo to confirm).\n");
        printf("  --fresh    Brand-new out-of-box drive: prints the manual 16 kHz carrier\n");
        printf("             panel step at the end (the carrier is not SDO-writable, so the\n");
        printf("             tool never touches it). The config params write the same either way.\n");
        printf("  --verify   Read back + compare only, no writes. Run AFTER the power-cycle.\n\n");
        printf("Stop the master service first (it owns the NIC):  sudo systemctl stop nullcat-pi\n");
        return 2;
    }

    std::string ifname = argv[1];
    int  slave   = std::atoi(argv[2]);
    bool fresh   = false;
    bool verify  = false;
    std::string dir      = exeDir();
    std::string cfgPath  = dir + "/config.json";
    std::string profPath = dir + "/drive_profiles/nullcat_a6.json";

    for (int i = 3; i < argc; ++i)
    {
        if      (!std::strcmp(argv[i], "--fresh"))                      fresh  = true;
        else if (!std::strcmp(argv[i], "--verify"))                     verify = true;
        else if (!std::strcmp(argv[i], "--config")  && i + 1 < argc)    cfgPath  = argv[++i];
        else if (!std::strcmp(argv[i], "--profile") && i + 1 < argc)    profPath = argv[++i];
        else { printf("Unknown argument: %s\n", argv[i]); return 2; }
    }

    Logger::instance().init(dir + "/logs/provision.log", true);

    // ---- load + validate the profile ----
    prov::Profile profile;
    std::string err;
    if (!profile.load(profPath, err))
    {
        printf("ERROR: profile load failed (%s): %s\n", profPath.c_str(), err.c_str());
        return 1;
    }
    printf("Profile: %s  (%zu clone params, carrier=%s, inertia-baseline=%s)\n",
           profile.name.c_str(), profile.clone.size(),
           profile.hasCarrier ? "yes" : "no", profile.hasInertia ? "yes" : "no");

    // ---- config (for DC/sync init params); NIC overridden by the arg ----
    Config config;
    if (!config.load(cfgPath))
        printf("Note: config load (%s): %s -- using defaults\n", cfgPath.c_str(), config.lastError().c_str());
    AppConfig cfg = config.get();
    cfg.nicName = ifname;

    // ---- bring the bus to SafeOp (NOT OP) ----
    EtherCATMaster master;
    master.applyConfig(cfg);
    printf("Initializing EtherCAT on %s (to SafeOp, no OP, no loop)...\n", ifname.c_str());
    InitResult ir = master.initialize(ifname);
    if (!ir.ok)
    {
        printf("ERROR: EtherCAT init failed: %s\n", ir.detail.c_str());
        master.shutdown();
        return 1;
    }
    printf("Init OK. %d slave(s) on bus. Target = slave %d.\n\n", master.getSlaveCount(), slave);

    // ---- resolve the target slave's axis mode from the rig config ----
    // torqueOnly[] profile params (C06.20 runaway protection = 0) apply only to
    // torque axes; a slave missing from the config is treated as position.
    bool torqueAxis = false;
    for (const DriveConfig& d : cfg.drives)
        if (d.slaveIndex == slave) { torqueAxis = (d.mode == "torque"); break; }
    if (!profile.torqueOnly.empty())
        printf("Axis mode per rig config: %s -> torqueOnly params (%zu) %s.\n\n",
               torqueAxis ? "TORQUE" : "position (or not in config)",
               profile.torqueOnly.size(),
               torqueAxis ? "INCLUDED" : "skipped");

    // ---- run the provisioner ----
    prov::Provisioner provr(master);
    prov::Result res = verify
        ? provr.verify(slave, profile, torqueAxis)
        : provr.provision(slave, profile, torqueAxis);

    printf("%-14s %-9s %-10s %-10s %s\n", "CoE", "panel", "wrote", "readback", "verified");
    printf("---------------------------------------------------------------\n");
    for (const auto& w : res.writes)
        printf("%-14s %-9s %-10lld %-10lld %s\n",
               w.coe.c_str(), w.panel.c_str(),
               (long long)w.wrote, (long long)w.readback, st(w.verified));

    printf("\n%s\n", res.message.c_str());
    if (!verify && !res.backupFile.empty()) printf("Before-state backup: %s\n", res.backupFile.c_str());
    if (!verify) printf("STORED to NVRAM: %s%s\n", st(res.stored),
                        res.stored ? "  -> POWER-CYCLE the drive, then re-run with --verify" : "");

    // 16 kHz carrier is a manual panel step (not SDO-writable) - remind for a fresh drive.
    if (fresh && !verify && res.stored)
        printf("\nFresh drive: set the 16 kHz carrier on the PANEL (this tool does not):\n"
               "    C00.31 = 1107 (unlock), then R22.38 = 1, save, power-cycle.\n"
               "    Skip if this drive should match others still on the 8 kHz base.\n");

    master.shutdown();
    return res.ok ? 0 : 1;
}
