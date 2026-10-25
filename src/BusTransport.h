// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// BusTransport.h - the test seam under the EtherCAT master
//
// The cyclic bus as the master's frame paths use it: one exchange of the
// process image, the distributed clock it carries, each slave's slice of
// the image and its AL state. On a real rig the master talks to SOEM
// directly and none of this is used. A test attaches a fake bus
// (EtherCATMaster::attachTestBus, tests/FakeBus.h): the master goes to OP
// on it with its drives bound through the real PDO assignment, so the
// pump, the OP bridge, sendReceive, the stop and fault-reset requests and
// the ControlLoop's real drive paths run without hardware. The hardware
// init sequence (configuration, mapping, DC, the OP transition) stays
// SOEM's alone.
// ============================================================

#include <cstdint>

class BusTransport
{
public:
    virtual ~BusTransport() = default;

    virtual int slaveCount() const = 0;
    // The working counter a whole frame carries.
    virtual int expectedWkc() const = 0;
    // One frame: the outputs out, the inputs in. The working counter, or
    // -1 with *exception set when the send crashed. Bus owner only.
    virtual int exchange(uint32_t* exception) = 0;
    // The distributed clock as of the last frame, ns.
    virtual int64_t dcTimeNs() const = 0;
    // Slave i's (1-based) slice of the process image, as SOEM's slavelist
    // has it: the outputs (master to slave) and inputs, and the outputs'
    // size (it tells the PDO layout apart: 19 bytes = 1702h torque).
    virtual uint8_t* outputs(int slave) = 0;
    virtual uint8_t* inputs(int slave) = 0;
    virtual int outputBytes(int slave) const = 0;
    // Each slave's AL state (and status code), as EtherCATMaster::
    // probeSlaveALStates reports it; the count written.
    virtual int probeAlStates(uint16_t* states, int maxSlaves, uint16_t* codes) = 0;
};
