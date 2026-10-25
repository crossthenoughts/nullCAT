// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// FakeBus.h - a fake EtherCAT bus of A6-like DS402 slaves
//
// The test side of the seam in src/BusTransport.h. Each slave carries the
// process image the master's PDO assignment expects (1701h position: 12
// bytes out; 1702h torque: 19 bytes out; 1B01h in: fault code | statusword
// | actual position | torque) and answers each frame as an A6 does:
//   - the DS402 state machine on the controlword, with the statusword
//     values the drives report in the rig's logs (0x1650 switch-on
//     disabled .. 0x1637 operation enabled, 0x1617 quick stop active,
//     0x1638 fault);
//   - a quick stop (enable voltage, quick-stop bit low: 0x0003) from
//     operation enabled goes to quick stop active, then switch-on disabled
//     a frame later (the A6's quick-stop option);
//   - the actual position follows the target while enabled (CSP), between
//     two hardstops, with torque feedback rising past them;
//   - what a test injects: a fault (a sticky one survives the reset), the
//     drive's own quick stop held (a hardware e-stop wired to the drive),
//     a vertical axis sagging while disabled, and an overcurrent fault on
//     enabling a motor that is still moving (the 2026-10-25 posts).
// It records what a test asks about: frames, entries into operation
// enabled, frames enabled while a test says enabling is forbidden, quick
// stops and fault-reset pulses seen.
//
// exchange() runs on whichever thread owns the bus (the pump, the RT
// loop); the test thread reads the model through the accessors. One mutex
// covers both.
// ============================================================

#include "../src/BusTransport.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <vector>

class FakeBus : public BusTransport
{
public:
    enum class St { SwitchOnDisabled, ReadyToSwitchOn, SwitchedOn, OperationEnabled, QuickStopActive, Fault };

    struct Slave
    {
        bool     torqueLayout = false;
        uint8_t  out[32] = {};
        uint8_t  in[32]  = {};
        St       state = St::SwitchOnDisabled;
        int32_t  pos = 0;                // counts
        int32_t  lastMove = 0;           // counts moved last frame
        int32_t  hardLo = -1000000, hardHi = 1000000;
        uint16_t faultCode = 0;
        uint16_t prevCw = 0;
        int      qsaFrames = 0;
        // injected
        bool     faultSticky = false;    // a fault reset does not clear it
        bool     holdQuickStop = false;  // the drive's own quick stop held (a hardware e-stop)
        int32_t  sagPerFrame = 0;        // counts per frame down while not enabled
        bool     overcurrentOnMovingEnable = false;
        // recorded
        int      enables = 0;            // entries into operation enabled
        int      quickStops = 0;         // quick-stop controlwords seen while enabled
        int      faultResetPulses = 0;
        int      forbiddenEnabledFrames = 0;
    };

    explicit FakeBus(const std::vector<bool>& torqueLayout)
    {
        m_s.resize(torqueLayout.size());
        for (size_t i = 0; i < torqueLayout.size(); ++i) m_s[i].torqueLayout = torqueLayout[i];
        for (Slave& s : m_s) publish(s);
    }

    // ---- BusTransport ----
    int slaveCount() const override { return static_cast<int>(m_s.size()); }
    int expectedWkc() const override { return 3 * slaveCount(); }
    int exchange(uint32_t* exception) override
    {
        if (exception) *exception = 0;
        std::lock_guard<std::mutex> lk(m_mx);
        for (Slave& s : m_s) step(s);
        ++m_frames;
        m_dcNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        return expectedWkc();
    }
    int64_t dcTimeNs() const override { return m_dcNs.load(); }
    uint8_t* outputs(int slave) override { return valid(slave) ? m_s[slave - 1].out : nullptr; }
    uint8_t* inputs(int slave) override  { return valid(slave) ? m_s[slave - 1].in  : nullptr; }
    int outputBytes(int slave) const override { return valid(slave) ? (m_s[slave - 1].torqueLayout ? 19 : 12) : 0; }
    int probeAlStates(uint16_t* states, int maxSlaves, uint16_t* codes) override
    {
        const int n = std::min(maxSlaves, slaveCount());
        for (int i = 0; i < n; ++i) { states[i] = 0x08; if (codes) codes[i] = 0; }   // OP
        return n;
    }

    // ---- the test's side ----
    long frames() const { return m_frames.load(); }
    St state(int slave) { std::lock_guard<std::mutex> lk(m_mx); return m_s[slave - 1].state; }
    bool anyEnabled()
    {
        std::lock_guard<std::mutex> lk(m_mx);
        for (const Slave& s : m_s) if (s.state == St::OperationEnabled) return true;
        return false;
    }
    bool allEnabled()
    {
        std::lock_guard<std::mutex> lk(m_mx);
        for (const Slave& s : m_s) if (s.state != St::OperationEnabled) return false;
        return true;
    }
    int enables()
    {
        std::lock_guard<std::mutex> lk(m_mx);
        int n = 0;
        for (const Slave& s : m_s) n += s.enables;
        return n;
    }
    // Every frame from now on with a slave enabled counts against the test.
    void forbidEnable(bool on) { std::lock_guard<std::mutex> lk(m_mx); m_forbid = on; }
    int forbiddenEnabledFrames()
    {
        std::lock_guard<std::mutex> lk(m_mx);
        int n = 0;
        for (const Slave& s : m_s) n += s.forbiddenEnabledFrames;
        return n;
    }
    template <class F> void with(int slave, F f) { std::lock_guard<std::mutex> lk(m_mx); f(m_s[slave - 1]); }

private:
    bool valid(int slave) const { return slave >= 1 && slave <= slaveCount(); }

    static uint16_t rd16(const uint8_t* p) { uint16_t v; std::memcpy(&v, p, 2); return v; }
    static int32_t  rd32(const uint8_t* p) { int32_t v;  std::memcpy(&v, p, 4); return v; }
    static void wr16(uint8_t* p, uint16_t v) { std::memcpy(p, &v, 2); }
    static void wr32(uint8_t* p, int32_t v)  { std::memcpy(p, &v, 4); }

    static uint16_t statusword(St st)
    {
        switch (st)
        {
        case St::SwitchOnDisabled: return 0x1650;
        case St::ReadyToSwitchOn:  return 0x1631;
        case St::SwitchedOn:       return 0x1633;
        case St::OperationEnabled: return 0x1637;
        case St::QuickStopActive:  return 0x1617;
        case St::Fault:            return 0x1638;
        }
        return 0x1650;
    }

    void publish(Slave& s)
    {
        wr16(s.in + 0, s.state == St::Fault ? s.faultCode : uint16_t(0));
        wr16(s.in + 2, statusword(s.state));
        wr32(s.in + 4, s.pos);
        int16_t torque = 0;
        if (s.state == St::OperationEnabled)
        {
            if (s.torqueLayout)
            {
                int16_t t; std::memcpy(&t, s.out + 10, 2);
                torque = t;
            }
            else
            {
                const int32_t target = rd32(s.out + 2);
                const int32_t past = target < s.hardLo ? s.hardLo - target : target > s.hardHi ? target - s.hardHi : 0;
                torque = static_cast<int16_t>(std::min<int32_t>(3000, past * 5));   // 0.1 % units
            }
        }
        wr16(s.in + 8, static_cast<uint16_t>(torque));
    }

    void fault(Slave& s, uint16_t code) { s.state = St::Fault; s.faultCode = code; }

    void step(Slave& s)
    {
        const uint16_t cw = rd16(s.out + 0);
        const bool resetEdge = (cw & 0x0080) && !(s.prevCw & 0x0080);
        if (resetEdge) ++s.faultResetPulses;
        const St was = s.state;

        if (s.holdQuickStop && s.state != St::Fault)
            s.state = St::QuickStopActive;
        else if (s.state == St::Fault)
        {
            if (resetEdge && !s.faultSticky) { s.state = St::SwitchOnDisabled; s.faultCode = 0; }
        }
        else if (s.state == St::QuickStopActive)
        {
            if (++s.qsaFrames > 1) s.state = St::SwitchOnDisabled;   // quick-stop option: then disabled
        }
        else if (!(cw & 0x0002))
            s.state = St::SwitchOnDisabled;                          // disable voltage
        else if (!(cw & 0x0004))
        {
            // Quick stop (0x0003).
            if (s.state == St::OperationEnabled) { s.state = St::QuickStopActive; s.qsaFrames = 0; ++s.quickStops; }
            else s.state = St::SwitchOnDisabled;
        }
        else if ((cw & 0x0007) == 0x0006)
        {
            if (s.state == St::SwitchOnDisabled || s.state == St::SwitchedOn || s.state == St::OperationEnabled)
                s.state = St::ReadyToSwitchOn;                       // shutdown
        }
        else if ((cw & 0x000F) == 0x0007)
        {
            if (s.state == St::ReadyToSwitchOn || s.state == St::OperationEnabled)
                s.state = St::SwitchedOn;                            // switch on / disable operation
        }
        else if ((cw & 0x000F) == 0x000F)
        {
            if (s.state == St::SwitchedOn)
            {
                if (s.overcurrentOnMovingEnable && s.lastMove != 0)
                    fault(s, 0x2312);                                // enabling a motor that still moves
                else
                    s.state = St::OperationEnabled;
            }
            else if (s.state == St::ReadyToSwitchOn)
                s.state = St::SwitchedOn;
        }
        if (s.state != St::QuickStopActive) s.qsaFrames = 0;
        if (s.state == St::OperationEnabled && was != St::OperationEnabled) ++s.enables;
        if (s.state == St::OperationEnabled && m_forbid) ++s.forbiddenEnabledFrames;

        // Motion: CSP while enabled (position layout), a sag while not.
        const int32_t before = s.pos;
        if (s.state == St::OperationEnabled && !s.torqueLayout)
        {
            int32_t target = rd32(s.out + 2);
            if (target < s.hardLo) target = s.hardLo;
            if (target > s.hardHi) target = s.hardHi;
            s.pos = target;
        }
        else if (s.state != St::OperationEnabled && s.sagPerFrame != 0)
        {
            s.pos -= s.sagPerFrame;
            if (s.pos < s.hardLo) s.pos = s.hardLo;
        }
        s.lastMove = s.pos - before;
        s.prevCw = cw;
        publish(s);
    }

    std::vector<Slave>   m_s;
    std::mutex           m_mx;
    std::atomic<long>    m_frames{0};
    std::atomic<int64_t> m_dcNs{0};
    bool                 m_forbid = false;
};
