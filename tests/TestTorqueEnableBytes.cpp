// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestTorqueEnableBytes.cpp
//
// Pins the bytes a torque drive (1702h RPDO) is given BEFORE and AT the
// enable instant. The first-init belt lunge (0.9.6 bench, 2026-10-01) was a
// zero-filled IOmap on a process's first enable: 0x0F went out with mode 0,
// 0x607F 0 and target position 0 because setTargetTorque() only runs once
// the drive reports OperationEnabled. Every torque test before this one
// used MockA6Drive, which has no IOmap at all, so the wire bytes were never
// under test.
//
// Tests:
//   T-1 primeTorqueCommand() at pointer setup fills mode/torque/clamp
//   T-2 A zero-filled IOmap is repaired on the pre-enable (SwitchedOn) cycles
//       and the 0x0F controlword never goes out with mode 0 or clamp 0
//   T-3 A position drive is untouched by primeTorqueCommand()
//   T-4 setTargetTorque() keeps the same invariant after enable
// ============================================================

#include <QtTest>
#include <cstdint>
#include <cstring>

#include "../src/A6Drive.h"
#include "../src/Logging.h"

namespace
{
// 1702h RPDO (19 bytes) / 1B01h TPDO (28 bytes), laid out exactly as
// EtherCATMaster wires them (see the torque branch of the PDO pointer setup).
struct TorqueIoMap
{
    uint8_t out[19];
    uint8_t in[28];
    TorqueIoMap() { std::memset(out, 0, sizeof(out)); std::memset(in, 0, sizeof(in)); }

    uint16_t* cw()       { return reinterpret_cast<uint16_t*>(out + 0);  }
    int8_t*   mode()     { return reinterpret_cast<int8_t*>(out + 12);   }
    int16_t*  torque()   { return reinterpret_cast<int16_t*>(out + 10);  }
    uint32_t* maxVel()   { return reinterpret_cast<uint32_t*>(out + 15); }
    uint16_t* sw()       { return reinterpret_cast<uint16_t*>(in + 2);   }
    int32_t*  actual()   { return reinterpret_cast<int32_t*>(in + 4);    }
    int16_t*  torqueFb() { return reinterpret_cast<int16_t*>(in + 8);    }
};

constexpr uint32_t kClamp = 1747626;   // 800 rpm at 131072 counts/rev

void wireTorqueDrive(A6Drive& d, TorqueIoMap& m)
{
    d.setSlaveIndex(4);
    d.setPDOPointers(m.cw(), m.mode(), nullptr, m.sw(), nullptr, m.actual(), m.torqueFb());
    d.setTorquePDOPointer(m.torque(), m.maxVel());
    d.setTorqueMode(true);
    d.setMaxProfileVelocityCounts(kClamp);
}
} // namespace

class TestTorqueEnableBytes : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase()
    {
        Logger::instance().setMinLevel(LogLevel::LVL_CRITICAL);
    }

    // T-1: the master primes the command right after wiring the pointers,
    // so the very first frame of the pre-OP pump is already a valid command.
    void t1_primeAtPointerSetup()
    {
        TorqueIoMap m;
        A6Drive d;
        wireTorqueDrive(d, m);
        QCOMPARE(static_cast<int>(*m.mode()), 0);        // zero-filled, as on a fresh process

        d.primeTorqueCommand();

        QCOMPARE(static_cast<int>(*m.mode()), 10);       // CST
        QCOMPARE(static_cast<int>(*m.torque()), 0);      // no torque before enable
        QCOMPARE(*m.maxVel(), kClamp);                   // never the "pinned" zero
    }

    // T-2: even if the IOmap is zeroed AFTER setup, the pre-enable cycles
    // repair it, and the controlword that first carries ENABLE_OPERATION
    // (0x0F) goes out alongside a valid command. This is the exact instant
    // the belt faulted on the bench.
    void t2_zeroFilledIoMapRepairedBeforeEnable()
    {
        TorqueIoMap m;
        A6Drive d;
        wireTorqueDrive(d, m);
        d.setCommandSyncCycles(3);
        *m.actual() = -458000;                           // belt wound ~3.5 rev from zero
        std::memset(m.out, 0, sizeof(m.out));            // the fresh-process state

        // Walk the DS402 enable sequence the way the drive answers it.
        *m.sw() = 0x0040;                                // SwitchOnDisabled
        d.stepEnableStateMachine();
        *m.sw() = 0x0021;                                // ReadyToSwitchOn
        d.stepEnableStateMachine();

        // SwitchedOn: the hold phase. From here on the command must be valid.
        *m.sw() = 0x0023;
        bool sawEnable = false;
        for (int i = 0; i < 8 && !sawEnable; ++i)
        {
            d.stepEnableStateMachine();
            QCOMPARE(static_cast<int>(*m.mode()), 10);
            QCOMPARE(*m.maxVel(), kClamp);
            QCOMPARE(static_cast<int>(*m.torque()), 0);
            if ((*m.cw() & 0x000F) == 0x000F) sawEnable = true;
        }
        QVERIFY2(sawEnable, "state machine never requested OperationEnabled");

        // At the 0x0F instant the bytes the drive snapshots are valid.
        QCOMPARE(static_cast<int>(*m.cw() & 0x000F), 0x0F);
        QCOMPARE(static_cast<int>(*m.mode()), 10);
        QCOMPARE(*m.maxVel(), kClamp);
        QCOMPARE(static_cast<int>(*m.torque()), 0);
    }

    // T-3: a position drive (1701h, no torque pointers) is left alone.
    void t3_positionDriveUntouched()
    {
        uint16_t cw = 0; int8_t mop = 0; int32_t tgt = 777; uint16_t sw = 0x0023;
        int8_t mod = 0; int32_t act = 777;
        A6Drive d;
        d.setSlaveIndex(1);
        d.setPDOPointers(&cw, &mop, &tgt, &sw, &mod, &act);

        d.primeTorqueCommand();
        d.stepEnableStateMachine();

        QCOMPARE(static_cast<int>(mop), 0);              // mode byte untouched
        QCOMPARE(tgt, act);                              // position seeding still works
    }

    // T-4: once enabled, the per-cycle torque write keeps the invariant.
    void t4_setTargetTorqueKeepsInvariant()
    {
        TorqueIoMap m;
        A6Drive d;
        wireTorqueDrive(d, m);
        d.primeTorqueCommand();
        std::memset(m.out, 0, sizeof(m.out));            // hostile: someone zeroed it

        d.setTargetTorque(12.5);

        QCOMPARE(static_cast<int>(*m.mode()), 10);
        QCOMPARE(*m.maxVel(), kClamp);
        QCOMPARE(static_cast<int>(*m.torque()), 125);    // 0.1% units
    }
};

QTEST_MAIN(TestTorqueEnableBytes)
#include "TestTorqueEnableBytes.moc"
