// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestB222J.cpp  (Build 222J)
//
// Unit tests for B222J changes:
//   J-6a    A6: ds402ErrorCodeString covers expected DS402/CiA 402 codes
//   J-6b    A6: unknown codes return "unknown" rather than null/crashing
//   (J-6c, the readDriveFaultHistory placeholder, removed with the reads)
//   J-3     A3: dumpSlaveState DIAG line includes coembxoverrun=N
// ============================================================

#include <QtTest>
#include <cstdint>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>
#include <memory>
#include <QTemporaryFile>

#include "../src/EtherCATMaster.h"
#include "../src/Logging.h"
#include "../src/Config.h"

#ifdef SOEM_AVAILABLE
#  include "ec_options.h"
#  include "osal.h"
#  include "ec_type.h"
#  include "nicdrv.h"
#  include "ec_main.h"
#endif

class TestB222J : public QObject
{
    Q_OBJECT

private slots:

    // --------------------------------------------------------------
    // J-6a: ds402ErrorCodeString returns expected descriptions for
    // the common DS402 / CiA 402 error codes the recovery thread is
    // most likely to encounter. Spot-check a representative subset.
    // --------------------------------------------------------------
    void j6a_ds402ErrorCodeString_known()
    {
        // No error
        QCOMPARE(QString(EtherCATMaster::ds402ErrorCodeString(0x0000)),
                 QString("no error"));
        // DC link undervoltage - common on rig power-cycle
        QCOMPARE(QString(EtherCATMaster::ds402ErrorCodeString(0x3220)),
                 QString("DC link undervoltage"));
        // Following error window exceeded - what we're MOST likely to see
        // in motion if drive integration falls behind
        QCOMPARE(QString(EtherCATMaster::ds402ErrorCodeString(0x8611)),
                 QString("following error window exceeded"));
        // Sync error - what B222H R2 actually faulted on (SW=0xd74e)
        QCOMPARE(QString(EtherCATMaster::ds402ErrorCodeString(0x8700)),
                 QString("sync error"));
        // Drive overtemperature
        QCOMPARE(QString(EtherCATMaster::ds402ErrorCodeString(0x4210)),
                 QString("drive / motor overtemperature"));
    }

    // --------------------------------------------------------------
    // J-6b: unknown codes get a stable "unknown" string, not null
    // or a crash. The recovery scan logs raw hex anyway, so the
    // description is just for human-readability - but it must never
    // be null (would NULL-deref strncpy / printf).
    // --------------------------------------------------------------
    void j6b_ds402ErrorCodeString_unknown()
    {
        QVERIFY(EtherCATMaster::ds402ErrorCodeString(0x9999) != nullptr);
        QCOMPARE(QString(EtherCATMaster::ds402ErrorCodeString(0x9999)),
                 QString("unknown"));
        QCOMPARE(QString(EtherCATMaster::ds402ErrorCodeString(0xABCD)),
                 QString("unknown"));
    }

    // --------------------------------------------------------------
    // J-3: dumpSlaveState DIAG line must include coembxoverrun=N.
    // Construct a fake slave with coembxoverrun=42, call the test-
    // exposed dumpSlaveStateForTest, scan recent diag output for the
    // expected field.
    // --------------------------------------------------------------
    void j3_dumpSlaveState_includes_coembxoverrun()
    {
#ifdef SOEM_AVAILABLE
        // Heap-allocate so we don't blow the test thread's stack
        // (ecx_contextt is >200KB after the B222I ABI alignment).
        std::unique_ptr<ecx_contextt> ctxPtr(new ecx_contextt());
        memset(ctxPtr.get(), 0, sizeof(ecx_contextt));
        ecx_contextt& ctx = *ctxPtr;

        // Fake a slave with a non-zero overrun count.
        ctx.slavelist[1].state          = 0x08;       // OP
        ctx.slavelist[1].mbx_l          = 256;
        ctx.slavelist[1].mbx_rl         = 256;
        ctx.slavelist[1].mbx_proto      = 0x000C;
        ctx.slavelist[1].Obytes         = 12;
        ctx.slavelist[1].Ibytes         = 28;
        ctx.slavelist[1].hasdc          = TRUE;
        ctx.slavelist[1].DCcycle        = 2000000;
        ctx.slavelist[1].DCshift        = 500000;
        ctx.slavelist[1].coembxoverrun  = 42;

        // dumpSlaveState writes via logDiag() which targets m_diagFile
        // (separate from the ring buffer that getRecentLogs returns).
        // Point Logger at a temp diag file, write, read back.
        QTemporaryFile tmp;
        QVERIFY(tmp.open());
        QString tmpPath = tmp.fileName();
        tmp.close();  // close handle so Logger can open it for writing

        Logger::instance().initDiag(tmpPath.toStdString());
        EtherCATMaster::dumpSlaveStateForTest(&ctx, 1, "j3_test");
        // Flush so the write is visible on read-back.
        Logger::instance().flush();

        std::ifstream in(tmpPath.toStdString());
        std::stringstream buf;
        buf << in.rdbuf();
        std::string contents = buf.str();
        QVERIFY2(contents.find("coembxoverrun=42") != std::string::npos,
            qPrintable(QString("Expected 'coembxoverrun=42' in diag file. Got:\n%1")
                .arg(QString::fromStdString(contents))));
        QVERIFY2(contents.find("slave_state") != std::string::npos,
            "Expected 'slave_state' tag in diag file");
#else
        QSKIP("SOEM not available - dumpSlaveState test requires SOEM headers");
#endif
    }
};

QTEST_MAIN(TestB222J)
#include "TestB222J.moc"
