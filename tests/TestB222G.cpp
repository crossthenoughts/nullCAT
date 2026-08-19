// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestB222G.cpp  (Build 222G)
//
// Unit tests for B222G fixes:
//   G-1  P1: Pre-config-init Npcap drain - sim init regression
//   G-3a P3: Logger::flush() method exists and runs without crash
//   G-3b P3: Logger::flush() with open file - content is readable after flush
//   G-3c P3: PlatformRT::safeCall catches SEH and runs flush without crash
//   G-4  P4: drainElist bails at MAX_DRAIN_ENTRIES=100 on corrupted elist
//
// P2 (error message update) is verified by code inspection only - // the crash path requires a live NIC and cannot be unit-tested.
// ============================================================

#include <QtTest>
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>
#include <fstream>

#include "../src/EtherCATMaster.h"
#include "../src/Logging.h"
#include "../src/PlatformRT.h"
#include "../src/Config.h"

// Full SOEM definition needed to allocate ecx_contextt on the stack in G-4.
// The forward declaration in A6Drive.h is not sufficient for stack allocation.
// Include ec_main.h directly rather than the aggregate soem.h to avoid
// the ec_soe.h EC_SOE_MAXNAME dependency ordering issue.
#ifdef SOEM_AVAILABLE
#  include "ec_options.h"
#  include "osal.h"
#  include "ec_type.h"
#  include "nicdrv.h"
#  include "ec_main.h"
#endif

#ifdef _WIN32
#  include <windows.h>
#endif

// ---- Helpers ----

static AppConfig makeSimConfig()
{
    AppConfig cfg;
    cfg.numDrives      = 1;
    cfg.controlLoopHz  = 1000;
    cfg.simulationMode = true;

    DriveConfig d;
    d.slaveIndex          = 1;
    d.axisType            = "linear_vertical";
    d.strokeMm            = 100.0;
    d.homingSpeed      = 5.0;
    d.homingBackoffMm     = 1.5;
    d.homingTorquePct     = 25;
    d.homeDirection       = "negative";
    d.parkMode            = "endstop";
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

// ---- Test class ----

class TestB222G : public QObject
{
    Q_OBJECT

private Q_SLOTS:

    void initTestCase()
    {
        Logger::instance().init("", false);
        Logger::instance().setMinLevel(LogLevel::LVL_CRITICAL);
    }
    void cleanupTestCase() {}

    // ----------------------------------------------------------------
    // G-1: P1 regression - sim init still succeeds after B222G changes.
    //
    // The pre-config-init drain is in the hardware-only code path and
    // cannot be unit-tested without a live NIC. This test confirms the
    // sim path (which bypasses NIC init entirely) is unaffected.
    // Hardware drain placement verified by inspection: drain block lies
    // between "ecx_init() OK" log and safeConfigInit() call.
    // ----------------------------------------------------------------
    void g1_simInitUnaffectedByDrainChange()
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
    // G-3a: Logger::flush() method exists and runs without crash.
    // ----------------------------------------------------------------
    void g3a_loggerFlushExists()
    {
        try
        {
            Logger::instance().flush();
        }
        catch (...)
        {
            QFAIL("Logger::flush() threw an unexpected exception");
        }
    }

    // ----------------------------------------------------------------
    // G-3b: Logger::flush() with open file - written content is readable.
    //
    // writeEntry() already calls flush() per-write, but this test
    // verifies the explicit flush() API leaves the file in a consistent
    // state where a subsequent reader can see the content.
    // ----------------------------------------------------------------
    void g3b_loggerFlushWithOpenFile()
    {
        const char* tmpEnv = std::getenv("TEMP");
        std::string tmpPath = std::string(tmpEnv ? tmpEnv : "C:/Temp")
                              + "/TestB222G_flush_test.log";

        Logger::instance().init(tmpPath, false);
        Logger::instance().setMinLevel(LogLevel::LVL_DEBUG);
        LOG_INFO("TestB222G: flush test sentinel");
        Logger::instance().flush();

        std::ifstream f(tmpPath);
        QVERIFY2(f.is_open(), "Temp log file could not be opened after flush");

        std::string content((std::istreambuf_iterator<char>(f)),
                             std::istreambuf_iterator<char>());
        QVERIFY2(content.find("flush test sentinel") != std::string::npos,
                 "Sentinel line not found in flushed log file");

        // Restore silent logger for remaining tests
        Logger::instance().init("", false);
        Logger::instance().setMinLevel(LogLevel::LVL_CRITICAL);
    }

    // ----------------------------------------------------------------
    // G-3c: PlatformRT::safeCall catches SEH; flush ran without crash.
    //
    // Raises a non-fatal exception code (0xdeadbeef) inside safeCall.
    // Verifies: (a) exception is caught, (b) code is returned correctly,
    // (c) the Logger::flush() + fflush calls in __except did not crash.
    // Skipped on non-Windows where safeCall is a passthrough.
    // ----------------------------------------------------------------
    void g3c_safecallFlushOnException()
    {
#ifdef _WIN32
        uint32_t exCode = 0;
        bool ok = PlatformRT::safeCall([]()
        {
            RaiseException(0xdeadbeef, 0, 0, nullptr);
        }, &exCode);
        QVERIFY(!ok);
        QCOMPARE(exCode, static_cast<uint32_t>(0xdeadbeef));
        // Reaching here means __except ran (including Logger::flush()) without crash.
#else
        QSKIP("SEH not available on this platform");
#endif
    }

    // ----------------------------------------------------------------
    // G-4: drainElist bails at MAX_DRAIN_ENTRIES=100 on corrupted elist.
    //
    // EC_MAXELIST=64, so a valid ring holds at most 64 entries. Setting
    // elist.head=200 (> 64) simulates corruption: tail cycles 0..64..0
    // and can never equal 200, so ecx_iserror() always returns true.
    // Without the cap the loop would be infinite. With the cap it bails
    // at 100 and emits a LOG_WARNING containing "bailed at 100 entries".
    // ----------------------------------------------------------------
    void g4_drainElistBailsAtMaxEntries()
    {
#ifdef SOEM_AVAILABLE
        // Push EC_MAXELIST=64 entries via ecx_pusherror into a fresh context.
        // Then drain with a cap of 30 (< 64), which forces the bailout path to
        // fire at count=30. This exercises the real SOEM ring rather than trying
        // to corrupt it with out-of-range head values (which SOEM's modular math
        // collapses to EC_MAXELIST%N apparent entries).
        //
        // ecx_contextt holds slavelist[200] + grouplist arrays and exceeds 100 KB -         // too large for the stack (default 1 MB shared with Qt's own frames).
        // Heap-allocate so the test doesn't overflow the stack.
        std::unique_ptr<ecx_contextt> ctxPtr(new ecx_contextt());
        memset(ctxPtr.get(), 0, sizeof(ecx_contextt));
        ecx_contextt& ctx = *ctxPtr;

        ec_errort entry{};
        entry.Etype = EC_ERR_TYPE_SDO_ERROR;
        for (int i = 0; i < EC_MAXELIST; ++i)
            ecx_pusherror(&ctx, &entry);

        // Verify the ring has entries before draining
        QVERIFY2(ecx_iserror(&ctx), "elist should have entries after ecx_pusherror");

        Logger::instance().setMinLevel(LogLevel::LVL_DEBUG);

        EtherCATMaster::drainElistForTestCapped(&ctx, "test_bailout", 30);

        Logger::instance().setMinLevel(LogLevel::LVL_CRITICAL);

        auto logs = Logger::instance().getRecentLogs(300);
        bool foundBailout = false;
        for (const auto& line : logs)
        {
            if (line.find("bailed at 30 entries") != std::string::npos)
            {
                foundBailout = true;
                break;
            }
        }
        QVERIFY2(foundBailout, "Expected 'bailed at 30 entries' warning not found in ring buffer");
#else
        QSKIP("SOEM not available - skipping elist bailout test");
#endif
    }
};

QTEST_MAIN(TestB222G)
#include "TestB222G.moc"
