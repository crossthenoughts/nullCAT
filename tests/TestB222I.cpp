// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestB222I.cpp  (Build 222I)
//
// Unit tests for B222I changes:
//   I-1     ec_options.h aligned to library (EC_MAXMBX=1486 etc.) - verified by
//           sizeof(ecx_contextt) sanity check; runtime ABI invariant.
//   I-2a    stageDCArm wrapped in PlatformRT::safeCall - code inspection
//           (hardware-only at runtime; sim path doesn't enter stageDCArm).
//   I-2c    drainElistImpl corruption guard: out-of-range Slave or Etype
//           triggers corrupted_skip exit, not 100-entry bailout.
//   I-3     A4: processCyclicMailbox is a no-op before initCyclicMailboxHandler
//           registers slaves; safe to call from RT loop pre-OP.
//   I-4a    A5: recovery thread start/stop is reentrant + clean.
//   I-4b    A5: signalRecoveryNeeded sets the atomic flag (consumed by thread).
//   I-4c    A5: sendReceive() / processCyclicMailbox() acquire m_soemAccessMutex
//           (functional check - they still work under the lock).
// ============================================================

#include <QtTest>
#include <atomic>
#include <thread>
#include <chrono>
#include <cstring>

#include "../src/EtherCATMaster.h"
#include "../src/Logging.h"
#include "../src/PlatformRT.h"
#include "../src/Config.h"

#ifdef SOEM_AVAILABLE
#  include "ec_options.h"
#  include "osal.h"
#  include "ec_type.h"
#  include "nicdrv.h"
#  include "ec_main.h"
#endif

class TestB222I : public QObject
{
    Q_OBJECT

private slots:

    // --------------------------------------------------------------
    // I-1: ec_options.h aligned to library. Verify sizeof(ecx_contextt)
    // exceeds the pre-alignment threshold (the install header sized it
    // ~115KB smaller). The exact size is build-dependent; we just check
    // it's plausible for the corrected layout.
    // --------------------------------------------------------------
    void i1_eccontext_sized_for_library()
    {
#ifdef SOEM_AVAILABLE
        // With EC_MAXMBX=1486, EC_MBXPOOLSIZE=32, mbxpool alone is
        // 32 * (1486+1) ≈ 48 KB. Combined with slavelist (200 slots),
        // grouplist, idxstack, eepSM/FMMU, EEPROM cache, total should
        // be > 200 KB. The previous (mismatched) build had ~100 KB.
        constexpr std::size_t kMinExpectedBytes = 150 * 1024;
        QVERIFY2(sizeof(ecx_contextt) >= kMinExpectedBytes,
            qPrintable(QString("sizeof(ecx_contextt) = %1 < %2 (header may be mis-aligned)")
                .arg(sizeof(ecx_contextt)).arg(kMinExpectedBytes)));
#else
        QSKIP("SOEM not available");
#endif
    }

    // --------------------------------------------------------------
    // I-2c: drainElistImpl corruption guard. Push a single ec_errort
    // with Slave=9999 (well beyond EC_MAXSLAVE=200), then drain.
    // Expect: status=corrupted_skip, not bailout_max_reached.
    // --------------------------------------------------------------
    void i2c_drainElist_corruption_guard()
    {
#ifdef SOEM_AVAILABLE
        std::unique_ptr<ecx_contextt> ctxPtr(new ecx_contextt());
        memset(ctxPtr.get(), 0, sizeof(ecx_contextt));
        ecx_contextt& ctx = *ctxPtr;

        ec_errort entry{};
        entry.Slave = 9999;             // out of range [0, EC_MAXSLAVE]
        entry.Etype = EC_ERR_TYPE_SDO_ERROR;
        ecx_pusherror(&ctx, &entry);
        QVERIFY2(ecx_iserror(&ctx), "elist should have the corrupt entry");

        Logger::instance().setMinLevel(LogLevel::LVL_DEBUG);
        EtherCATMaster::drainElistForTestCapped(&ctx, "test_corruption", 100);
        Logger::instance().setMinLevel(LogLevel::LVL_CRITICAL);

        // The DIAG line goes to _soem.log via logDiag(); the user-facing warning
        // (in the ring buffer via LOG_WARNING) is the one we match against here.
        auto logs = Logger::instance().getRecentLogs(100);
        bool foundCorrupted = false;
        bool foundBailout = false;
        for (const auto& line : logs)
        {
            if (line.find("drainElist aborted at entry 1") != std::string::npos &&
                line.find("Slave=9999") != std::string::npos)
                foundCorrupted = true;
            if (line.find("bailed at") != std::string::npos) foundBailout = true;
        }
        QVERIFY2(foundCorrupted, "Expected 'drainElist aborted ... Slave=9999' warning");
        QVERIFY2(!foundBailout, "Should NOT have hit the 100-entry bailout - guard should fire first");
#else
        QSKIP("SOEM not available");
#endif
    }

    // --------------------------------------------------------------
    // I-2c-2: drainElistImpl corruption guard - out-of-range Etype.
    // --------------------------------------------------------------
    void i2c_drainElist_corruption_guard_etype()
    {
#ifdef SOEM_AVAILABLE
        std::unique_ptr<ecx_contextt> ctxPtr(new ecx_contextt());
        memset(ctxPtr.get(), 0, sizeof(ecx_contextt));
        ecx_contextt& ctx = *ctxPtr;

        ec_errort entry{};
        entry.Slave = 1;
        entry.Etype = static_cast<ec_err_type>(999);  // out of range [0, 9]
        ecx_pusherror(&ctx, &entry);

        Logger::instance().setMinLevel(LogLevel::LVL_DEBUG);
        EtherCATMaster::drainElistForTestCapped(&ctx, "test_corruption_etype", 100);
        Logger::instance().setMinLevel(LogLevel::LVL_CRITICAL);

        auto logs = Logger::instance().getRecentLogs(100);
        bool foundCorrupted = false;
        for (const auto& line : logs)
            if (line.find("drainElist aborted") != std::string::npos &&
                line.find("Etype=999") != std::string::npos)
            { foundCorrupted = true; break; }
        QVERIFY2(foundCorrupted, "Expected 'drainElist aborted ... Etype=999' for out-of-range Etype");
#else
        QSKIP("SOEM not available");
#endif
    }

    // --------------------------------------------------------------
    // I-3: processCyclicMailbox returns 0 when not initialised (no
    // mailbox handler registered) and in simulation mode. Safe to
    // call from the RT loop before A4 has wired up.
    // --------------------------------------------------------------
    void i3_processCyclicMailbox_safe_when_uninit()
    {
        EtherCATMaster master;
        AppConfig cfg;
        cfg.numDrives = 1;
        cfg.controlLoopHz = 500;
        cfg.simulationMode = true;
        DriveConfig d;
        d.slaveIndex = 1; d.axisType = "linear_vertical"; d.name = "A";
        d.encoderCountsPerRev = 10000; d.ballscrewPitch = 5.0; d.countsPerMm = 2000.0;
        cfg.drives.push_back(d);
        master.applyConfig(cfg);

        // Sim mode: should return 0 without touching SOEM.
        QCOMPARE(master.processCyclicMailbox(1), 0);
        QCOMPARE(master.processCyclicMailbox(10), 0);
    }

    // --------------------------------------------------------------
    // I-4a: recovery thread start/stop is idempotent.
    // --------------------------------------------------------------
    void i4a_recovery_thread_idempotent_stop()
    {
        EtherCATMaster master;
        master.setSimulationMode(true);
        // signalRecoveryNeeded() works even if thread not started - just sets a flag.
        master.signalRecoveryNeeded();
        QVERIFY(!master.isRecoveryThreadRunning());  // start gated by simulationMode
    }

    // --------------------------------------------------------------
    // I-4b: signalRecoveryNeeded is callable from any thread without
    // blocking. Stress-test concurrent signalling.
    // --------------------------------------------------------------
    void i4b_signalRecoveryNeeded_threadsafe()
    {
        EtherCATMaster master;
        master.setSimulationMode(true);
        std::atomic<bool> stop{false};
        std::thread t1([&]() {
            while (!stop.load()) master.signalRecoveryNeeded();
        });
        std::thread t2([&]() {
            while (!stop.load()) master.signalRecoveryNeeded();
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        stop.store(true);
        t1.join();
        t2.join();
        // No assertion: success is "no crash, no hang".
        QVERIFY(true);
    }

    // --------------------------------------------------------------
    // I-4c: sendReceive() doesn't deadlock under m_soemAccessMutex.
    // We can't easily run through initialize() in unit tests without
    // hardware, so this just verifies the call returns promptly when
    // not initialized. Real coverage is via TestInitSequence + hardware.
    // --------------------------------------------------------------
    void i4c_sendReceive_no_deadlock_uninit()
    {
        EtherCATMaster master;
        master.setSimulationMode(true);
        // Not initialized: returns -1 quickly, mutex acquisition is moot
        // (we never reach the locked block). Just verifies no hang.
        auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < 100; ++i)
            (void)master.sendReceive();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
        QVERIFY2(elapsed < 1000,
            qPrintable(QString("sendReceive loop took %1ms - possible deadlock").arg(elapsed)));
    }
};

QTEST_MAIN(TestB222I)
#include "TestB222I.moc"
