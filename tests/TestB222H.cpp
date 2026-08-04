// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestB222H.cpp  (Build 222H)
//
// Unit tests for B222H fixes:
//   H-1a/1b  A1a+A1b: checkSlaveErrorStateCached — ALStatusCode pre-check
//            before configMap (A1a) and pre-OP SDO burst (A1b)
//   H-8      A8: mbx_rl field added to slave_state and pre_config_map_slave DIAG
//   H-15     A15: ecx_err2string format in drainElistImpl;
//            ec_ALstatuscode2string in logOPFailureDiagnostics (code inspection)
//   H-17     A17: per-cycle RTT tracking in ControlLoop; DIAG suppressed in sim mode
//   H-18     A18: OP transition early-exit (hardware-only; QSKIP in unit tests)
//   H-1cDiag A1c-diag: DC margin sample in pre-OP pump (hardware-only; QSKIP)
//
// Tests H-1a/1b-1..4, H-8a/b, H-15 require SOEM_AVAILABLE (ecx_contextt
// must be fully defined for heap allocation and ecx_pusherror/ecx_err2string).
// Tests H-1 sim, H-17a/b do not require SOEM.
// ============================================================

#include <QtTest>
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>
#include <fstream>
#include <thread>
#include <chrono>

#include "../src/EtherCATMaster.h"
#include "../src/Logging.h"
#include "../src/PlatformRT.h"
#include "../src/Config.h"
#include "../src/ControlLoop.h"

#ifdef SOEM_AVAILABLE
#  include "ec_options.h"
#  include "osal.h"
#  include "ec_type.h"
#  include "nicdrv.h"
#  include "ec_main.h"
// ec_print.h NOT included here: direct inclusion causes ecx_pusherror crash in this executable.
// Root cause: ec_print.h pulls Pcap32/WinPcap headers which redefine CRITICAL_SECTION fields,
// changing the sizeof(ecx_portt) seen by the test vs the soem.lib binary → struct layout mismatch.
// Forward-declare only the one function needed directly in tests.
extern "C" { char* ec_ALstatuscode2string(uint16 ALstatuscode); }
#endif

// ---- Helpers ----

static AppConfig makeSimConfig(int hz = 1000)
{
    AppConfig cfg;
    cfg.numDrives      = 1;
    cfg.controlLoopHz  = hz;
    cfg.simulationMode = true;

    DriveConfig d;
    d.slaveIndex          = 1;
    d.axisType            = "linear_vertical";
    d.strokeMm            = 100.0;
    d.homingSpeedMmS      = 5.0;
    d.homingBackoffMm     = 1.5;
    d.homingTorquePct     = 25;
    d.homeDirection       = "negative";
    d.homeMode            = "endstop";
    d.maxVelocityMmS      = 200.0;
    d.maxAccelerationMmS2 = 2000.0;
    d.maxJerkMmS3         = 20000.0;
    d.unparkTimeSec       = 0.1;
    d.parkTimeSec         = 0.1;
    d.countsPerMm         = 100.0;
    d.ballscrewPitch      = 5.0;
    cfg.drives.push_back(d);
    return cfg;
}

static std::string getTempDir()
{
    const char* t = std::getenv("TEMP");
    return std::string(t ? t : "C:/Temp");
}

// ---- Test class ----

class TestB222H : public QObject
{
    Q_OBJECT

private Q_SLOTS:

    void initTestCase()
    {
        Logger::instance().init("", false);
        Logger::instance().setMinLevel(LogLevel::LVL_CRITICAL);
    }

    void cleanupTestCase()
    {
        // Close any diag file left open by tests.
        Logger::instance().initDiag("");
    }

    // ----------------------------------------------------------------
    // H-1a/1b-1: checkSlaveErrorStateForTest returns true when all slave
    // ALstatuscode fields are zero. FPRD fallback for cache=0 will fail
    // (wkc<=0, no real port) and is correctly treated as clean.
    // ----------------------------------------------------------------
    void h1_allSlavesClean()
    {
#ifdef SOEM_AVAILABLE
        std::unique_ptr<ecx_contextt> ctxPtr(new ecx_contextt());
        memset(ctxPtr.get(), 0, sizeof(ecx_contextt));
        ecx_contextt& ctx = *ctxPtr;

        int failedSlave = -1;
        std::string detail;
        bool ok = EtherCATMaster::checkSlaveErrorStateForTest(
            &ctx, 1, "test_clean", failedSlave, detail);

        QVERIFY2(ok, "Expected true when all ALstatuscode=0");
        QCOMPARE(failedSlave, -1);
        QVERIFY2(detail.empty(), "Expected empty detail on clean check");
#else
        QSKIP("SOEM not available — skipping (ecx_contextt not fully defined)");
#endif
    }

    // ----------------------------------------------------------------
    // H-1a/1b-2: Returns false and populates failedSlave/detail when
    // the SOEM cached ALstatuscode is non-zero (bypasses FPRD call).
    // ----------------------------------------------------------------
    void h1_cachedDirty_returnsFalse()
    {
#ifdef SOEM_AVAILABLE
        std::unique_ptr<ecx_contextt> ctxPtr(new ecx_contextt());
        memset(ctxPtr.get(), 0, sizeof(ecx_contextt));
        ecx_contextt& ctx = *ctxPtr;
        ctx.slavelist[1].ALstatuscode = 0x0027;  // SYNC starting time set incorrectly

        int failedSlave = -1;
        std::string detail;
        bool ok = EtherCATMaster::checkSlaveErrorStateForTest(
            &ctx, 1, "test_dirty", failedSlave, detail);

        QVERIFY2(!ok, "Expected false when slave 1 ALstatuscode=0x0027");
        QCOMPARE(failedSlave, 1);
        QVERIFY2(detail.find("0x0027") != std::string::npos,
                 "Expected hex code 0x0027 in detail string");
#else
        QSKIP("SOEM not available");
#endif
    }

    // ----------------------------------------------------------------
    // H-1a/1b-3: With 3 slaves, the check stops at the first dirty
    // slave (slave 2) — slave 3's error code is never inspected.
    // ----------------------------------------------------------------
    void h1_middleSlaveDirty_stopsAtFirst()
    {
#ifdef SOEM_AVAILABLE
        std::unique_ptr<ecx_contextt> ctxPtr(new ecx_contextt());
        memset(ctxPtr.get(), 0, sizeof(ecx_contextt));
        ecx_contextt& ctx = *ctxPtr;
        ctx.slavelist[2].ALstatuscode = 0x001B;  // Invalid AL control
        ctx.slavelist[3].ALstatuscode = 0x002F;  // Invalid communication settings

        int failedSlave = -1;
        std::string detail;
        bool ok = EtherCATMaster::checkSlaveErrorStateForTest(
            &ctx, 3, "test_middle", failedSlave, detail);

        QVERIFY2(!ok, "Expected false with slave 2 dirty");
        QCOMPARE(failedSlave, 2);
        QVERIFY2(detail.find("0x001b") != std::string::npos ||
                 detail.find("0x001B") != std::string::npos,
                 "Expected hex code 0x001B in detail string");
#else
        QSKIP("SOEM not available");
#endif
    }

    // ----------------------------------------------------------------
    // H-1a/1b-4: detail string contains the SOEM-formatted AL description
    // from ec_ALstatuscode2string, not just the raw hex code.
    // ----------------------------------------------------------------
    void h1_detailContainsAlDescription()
    {
#ifdef SOEM_AVAILABLE
        std::unique_ptr<ecx_contextt> ctxPtr(new ecx_contextt());
        memset(ctxPtr.get(), 0, sizeof(ecx_contextt));
        ecx_contextt& ctx = *ctxPtr;
        // 0x001B = "Invalid AL control" per SOEM ec_print.c table
        ctx.slavelist[1].ALstatuscode = 0x001B;

        int failedSlave = -1;
        std::string detail;
        EtherCATMaster::checkSlaveErrorStateForTest(
            &ctx, 1, "test_desc", failedSlave, detail);

        const char* soemDesc = ec_ALstatuscode2string(0x001B);
        QVERIFY2(soemDesc && soemDesc[0] != '\0',
                 "ec_ALstatuscode2string returned null or empty for 0x001B");
        QVERIFY2(detail.find(soemDesc) != std::string::npos,
                 "Expected SOEM AL description in detail string");
#else
        QSKIP("SOEM not available");
#endif
    }

    // ----------------------------------------------------------------
    // H-1 sim: A1a + A1b changes do not affect the simulation init path.
    // In sim mode initialize() returns success before any stage is called,
    // so the new ecx_readstate + checkSlaveErrorStateCached calls are
    // not reached.
    // ----------------------------------------------------------------
    void h1_simInitUnaffected()
    {
        AppConfig cfg = makeSimConfig();
        EtherCATMaster master;
        master.setSimulationMode(true);
        master.applyConfig(cfg);
        InitResult r = master.initializeAndEnterOp(cfg.nicName);
        QVERIFY(r.ok);
        QVERIFY(master.isOperational());
        QVERIFY(master.getMasterState() == ECState::Op);
    }

    // ----------------------------------------------------------------
    // H-8a: dumpSlaveStateForTest DIAG output includes the mbx_rl= key
    // (A8: mbx_rl was missing from slave_state DIAG before B222H).
    // ----------------------------------------------------------------
    void h8_mbxRlInSlaveStateDiag()
    {
#ifdef SOEM_AVAILABLE
        std::unique_ptr<ecx_contextt> ctxPtr(new ecx_contextt());
        memset(ctxPtr.get(), 0, sizeof(ecx_contextt));
        ecx_contextt& ctx = *ctxPtr;

        std::string diagPath = getTempDir() + "/TestB222H_h8a.diag";
        Logger::instance().initDiag(diagPath);
        EtherCATMaster::dumpSlaveStateForTest(&ctx, 1, "test_mbxrl");
        Logger::instance().flush();
        Logger::instance().initDiag("");  // close diag file

        std::ifstream f(diagPath);
        QVERIFY2(f.is_open(), "Could not open temp diag file");
        std::string content((std::istreambuf_iterator<char>(f)),
                             std::istreambuf_iterator<char>());
        QVERIFY2(content.find("mbx_rl=") != std::string::npos,
                 "Expected 'mbx_rl=' in slave_state DIAG");
#else
        QSKIP("SOEM not available");
#endif
    }

    // ----------------------------------------------------------------
    // H-8b: dumpSlaveStateForTest emits the correct mbx_rl value.
    // ----------------------------------------------------------------
    void h8_mbxRlValueCorrect()
    {
#ifdef SOEM_AVAILABLE
        std::unique_ptr<ecx_contextt> ctxPtr(new ecx_contextt());
        memset(ctxPtr.get(), 0, sizeof(ecx_contextt));
        ecx_contextt& ctx = *ctxPtr;
        ctx.slavelist[1].mbx_rl = 42;

        std::string diagPath = getTempDir() + "/TestB222H_h8b.diag";
        Logger::instance().initDiag(diagPath);
        EtherCATMaster::dumpSlaveStateForTest(&ctx, 1, "test_mbxrl_val");
        Logger::instance().flush();
        Logger::instance().initDiag("");

        std::ifstream f(diagPath);
        QVERIFY2(f.is_open(), "Could not open temp diag file");
        std::string content((std::istreambuf_iterator<char>(f)),
                             std::istreambuf_iterator<char>());
        QVERIFY2(content.find("mbx_rl=42") != std::string::npos,
                 "Expected 'mbx_rl=42' in slave_state DIAG");
#else
        QSKIP("SOEM not available");
#endif
    }

    // ----------------------------------------------------------------
    // H-15: drainElistImpl DIAG uses ecx_err2string (A15 formatter).
    //
    // Writes an SDO error directly into the elist ring buffer (bypassing
    // ecx_pusherror, which requires Windows CS to be primed) and drains it.
    // Verifies that the DIAG line uses the A15 unified format (numeric etype +
    // ecx_err2string) rather than the old explicit "index=0x%04x:%02x | abort=…"
    // format that was emitted for non-EMERGENCY entries before A15.
    //
    // AbortCode=0 is intentional: ec_sdoerror2string(0) returns nullptr →
    // safeErr2String catches the strncpy crash → DIAG emits "(fmt_error)" rather
    // than crashing. The assertion checks that the old "index=0x6040" explicit
    // field is absent, confirming A15's formatter replaced the old branch.
    // ----------------------------------------------------------------
    void h15_elistDiagUsesFormattedString()
    {
#ifdef SOEM_AVAILABLE
        // ecx_pusherror on a zeroed ctx crashes: the SOEM binary's ecx_pusherror
        // requires Windows CS database to be primed by a prior InitializeCriticalSection
        // call (which doesn't happen in this test's isolation context). Write directly
        // to the elist ring buffer fields instead to simulate what ecx_pusherror does,
        // bypassing the CS-related crash path entirely.
        std::unique_ptr<ecx_contextt> ctxPtr(new ecx_contextt());
        memset(ctxPtr.get(), 0, sizeof(ecx_contextt));
        ecx_contextt& ctx = *ctxPtr;

        ec_errort entry{};
        entry.Etype     = EC_ERR_TYPE_SDO_ERROR;
        entry.Slave     = 1;
        entry.Index     = 0x6040;
        entry.SubIdx    = 0;
        entry.AbortCode = 0;  // ec_sdoerror2string(0)=null → safeErr2String catches, logs "(fmt_error)"
        // Direct elist write (simulates ecx_pusherror without its CS requirement):
        ctx.elist.head = 1;
        ctx.elist.Error[1] = entry;
        // tail stays 0 → ecx_iserror returns true (head != tail)

        QVERIFY2(ctx.elist.head != ctx.elist.tail, "elist should have an entry after direct write");

        std::string diagPath = getTempDir() + "/TestB222H_h15.diag";
        Logger::instance().initDiag(diagPath);
        EtherCATMaster::drainElistForTest(&ctx, "test_fmt");
        Logger::instance().flush();
        Logger::instance().initDiag("");

        std::ifstream f(diagPath);
        QVERIFY2(f.is_open(), "Could not open temp diag file");
        std::string content((std::istreambuf_iterator<char>(f)),
                             std::istreambuf_iterator<char>());

        QVERIFY2(content.find("DIAG | elist") != std::string::npos &&
                 content.find("test_fmt") != std::string::npos,
                 "Expected 'DIAG | elist' line for 'test_fmt' in diag output");

        // Old format (pre-A15) for non-EMERGENCY types emitted "index=0x6040:00 | abort=..."
        // New format (A15) uses ecx_err2string via safeErr2String and emits "(fmt_error)"
        // when ecx_err2string crashes (AbortCode=0 is not in SOEM's table → null → crash).
        QVERIFY2(content.find("index=0x6040") == std::string::npos,
                 "Unexpected old 'index=0x6040' field — A15 replaced explicit format with ecx_err2string");
#else
        QSKIP("SOEM not available — skipping elist formatter test");
#endif
    }

    // ----------------------------------------------------------------
    // H-17a: ControlLoopWorker runs briefly in sim mode without crashing.
    //
    // Verifies that A17 RTT tracking code (added rttMinUs/rttMaxUs/rttSumUs/
    // rttCount variables + instrumented sendReceive block) does not introduce
    // any regression in the sim path.
    // ----------------------------------------------------------------
    void h17_loopRunsWithoutCrash()
    {
        AppConfig cfg = makeSimConfig(100);  // 100Hz → 10ms/cycle

        EtherCATMaster master;
        master.setSimulationMode(true);
        master.applyConfig(cfg);
        InitResult r = master.initializeAndEnterOp(cfg.nicName);
        QVERIFY2(r.ok, "Sim init failed before ControlLoopWorker test");

        ControlLoopWorker worker;
        worker.setComponents(&master, nullptr, nullptr);
        worker.setConfig(cfg);

        std::thread t([&]() { worker.run(); });
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // ~10 cycles
        worker.requestStop();
        t.join();
        // Reaching here means the loop ran and stopped cleanly.
    }

    // ----------------------------------------------------------------
    // H-17b: No 'DIAG | rtt' is emitted in simulation mode.
    //
    // In sim mode the sendReceive() guard (!m_master->isSimulation()) is false,
    // so rttCount stays 0. The RTT DIAG block is guarded by (rttCount > 0)
    // and must not emit even after the stats section fires (>= statsInterval cycles).
    //
    // Runs at 100Hz for 1200ms (>= statsInterval=100 cycles, so stats fire once).
    // ----------------------------------------------------------------
    void h17_noRttDiagInSimMode()
    {
        AppConfig cfg = makeSimConfig(100);  // 100Hz → statsInterval=100 cycles

        EtherCATMaster master;
        master.setSimulationMode(true);
        master.applyConfig(cfg);
        InitResult r = master.initializeAndEnterOp(cfg.nicName);
        QVERIFY2(r.ok, "Sim init failed before RTT DIAG test");

        std::string diagPath = getTempDir() + "/TestB222H_h17.diag";
        Logger::instance().initDiag(diagPath);

        ControlLoopWorker worker;
        worker.setComponents(&master, nullptr, nullptr);
        worker.setConfig(cfg);

        std::thread t([&]() { worker.run(); });
        std::this_thread::sleep_for(std::chrono::milliseconds(1200));  // > 100 cycles
        worker.requestStop();
        t.join();

        Logger::instance().flush();
        Logger::instance().initDiag("");

        std::ifstream f(diagPath);
        QVERIFY2(f.is_open(), "Could not open temp diag file for RTT check");
        std::string content((std::istreambuf_iterator<char>(f)),
                             std::istreambuf_iterator<char>());
        QVERIFY2(content.find("DIAG | rtt") == std::string::npos,
                 "Unexpected 'DIAG | rtt' in sim-mode log — rttCount should stay 0");
    }

    // ----------------------------------------------------------------
    // H-18: A18 OP transition early-exit is hardware-only.
    //
    // The stageOPTransition loop is not reachable in simulation mode
    // (initialize() returns immediately). Verified by code inspection
    // that the every-100-cycle ecx_readstate + allOpNow check is correct.
    // ----------------------------------------------------------------
    void h18_hardwareOnlySkip()
    {
        QSKIP("A18 OP transition early-exit requires live hardware — verified by code inspection");
    }

    // ----------------------------------------------------------------
    // H-1c-diag: DC margin sample is hardware-only.
    //
    // The dc_margin_sample DIAG inside stagePreOpPump (ECT_REG_DCSYSTIME
    // FPRD every 500 cycles) is not reachable in simulation mode.
    // Verified by code inspection.
    // ----------------------------------------------------------------
    void h1cDiag_hardwareOnlySkip()
    {
        QSKIP("A1c-diag dc_margin_sample requires live hardware — verified by code inspection");
    }
};

QTEST_MAIN(TestB222H)
#include "TestB222H.moc"
