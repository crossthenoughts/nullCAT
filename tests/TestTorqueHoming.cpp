// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestTorqueHoming.cpp - unit tests for the torque-only homing sequence
// (device families: shifter, pedal).
//
// Plain-main suite driving TorqueHomingSequence against MockA6Drive in
// torque-response mode. The tests run with encoderCountsPerRev = 1 so the
// mock's position IS revs and the physics stays legible. Covers both stop
// directions, already-at-stop start, the travel guard, enable timeout,
// and the torque command contract (zero outside Search/Confirm).
// ============================================================

#include "../src/TorqueHomingSequence.h"
#include "../src/MockA6Drive.h"
#include <cstdio>
#include <cmath>

static int g_pass = 0, g_fail = 0;
static void CHECK(bool ok, const char* what)
{
    if (ok) { ++g_pass; }
    else    { ++g_fail; std::printf("FAIL: %s\n", what); }
}

static DeviceParams shifterParams()
{
    DeviceParams p;
    p.stopMinRev    = -0.07;
    p.stopMaxRev    =  0.07;
    p.homeTorquePct = 30.0;
    p.homeDir       = -1.0;
    return p;
}

// Drive the sequence to a terminal state (or cycle budget), feeding the
// returned torque back into the mock physics each cycle.
static int runToEnd(TorqueHomingSequence& seq, MockA6Drive& mock, int maxCycles)
{
    int n = 0;
    while (n < maxCycles &&
           !seq.isComplete() && !seq.isFatalError())
    {
        const double t = seq.step(&mock);
        mock.setSimCommandedTorque(t);
        mock.updateStatus();
        ++n;
    }
    return n;
}

int main()
{
    const double DT = 0.002;   // the 2ms pump

    // ---------- happy path: min stop ----------
    {
        MockA6Drive mock; mock.configure(1, 0.0);
        mock.setTorqueResponse(0.0001);            // 30% -> 0.003 rev/cycle
        mock.setHardstop(-0.05, true);
        TorqueHomingSequence seq;
        seq.configure(shifterParams(), 1.0, DT);
        seq.start(&mock);
        runToEnd(seq, mock, 5000);
        CHECK(seq.isComplete(), "min-stop search completes");
        CHECK(std::fabs(seq.getHomeRaw() - (-0.05)) < 1e-9,
              "home latched at the physical stop");
        CHECK(seq.homeStopRev() == -0.07,
              "homeDir=-1 maps the stop to stopMinRev");
        CHECK(seq.step(&mock) == 0.0, "zero torque after Complete");
    }

    // ---------- happy path: max stop ----------
    {
        MockA6Drive mock; mock.configure(1, 0.0);
        mock.setTorqueResponse(0.0001);
        mock.setHardstop(0.06, false);
        DeviceParams p = shifterParams();
        p.homeDir = 1.0;
        TorqueHomingSequence seq;
        seq.configure(p, 1.0, DT);
        seq.start(&mock);
        runToEnd(seq, mock, 5000);
        CHECK(seq.isComplete(), "max-stop search completes");
        CHECK(std::fabs(seq.getHomeRaw() - 0.06) < 1e-9,
              "home latched at the max stop");
        CHECK(seq.homeStopRev() == 0.07,
              "homeDir=+1 maps the stop to stopMaxRev");
    }

    // ---------- already at the stop when homing starts ----------
    // The common case: the lever rests against the stop it homes toward.
    {
        MockA6Drive mock; mock.configure(1, -0.05);
        mock.setTorqueResponse(0.0001);
        mock.setHardstop(-0.05, true);
        TorqueHomingSequence seq;
        seq.configure(shifterParams(), 1.0, DT);
        seq.start(&mock);
        const int n = runToEnd(seq, mock, 5000);
        CHECK(seq.isComplete(), "starting at the stop still completes");
        CHECK(std::fabs(seq.getHomeRaw() - (-0.05)) < 1e-9,
              "home latched at the start position");
        CHECK(n < 600, "already-at-stop completes promptly");
    }

    // ---------- travel guard: no stop found ----------
    // Wrong homeDir / decoupled mechanics: the axis runs away. The guard
    // trips at 1.5x the configured span, well before the timeout.
    {
        MockA6Drive mock; mock.configure(1, 0.0);
        mock.setTorqueResponse(0.0001);            // no hardstop set
        TorqueHomingSequence seq;
        seq.configure(shifterParams(), 1.0, DT);
        seq.start(&mock);
        runToEnd(seq, mock, 5000);
        CHECK(seq.isFatalError(), "runaway trips the travel guard");
        CHECK(std::fabs(mock.getSimActualPos()) < 0.30,
              "guard stops the push near 1.5x span, not at timeout");
        CHECK(seq.step(&mock) == 0.0, "zero torque after FatalError");
    }

    // ---------- enable timeout ----------
    {
        MockA6Drive mock; mock.configure(1, 0.0);
        mock.injectFault();                        // enable SM never succeeds
        TorqueHomingSequence seq;
        seq.configure(shifterParams(), 1.0, DT);
        seq.start(&mock);
        int n = 0;
        while (n < 5000 && !seq.isFatalError())
        { seq.step(&mock); mock.updateStatus(); ++n; }
        CHECK(seq.isFatalError(), "drive that will not enable = FatalError");
        CHECK(n >= 2400 && n <= 2600, "enable timeout fires at ~5s");
    }

    // ---------- torque command contract ----------
    {
        MockA6Drive mock; mock.configure(1, 0.0);
        mock.setTorqueResponse(0.0001);
        mock.setHardstop(-0.05, true);
        TorqueHomingSequence seq;
        seq.configure(shifterParams(), 1.0, DT);
        CHECK(seq.step(&mock) == 0.0, "Idle commands zero torque");
        seq.start(&mock);
        bool zeroWhileEnabling = true, sawSearchTorque = false, torqueOk = true;
        int n = 0;
        while (n < 5000 && !seq.isComplete() && !seq.isFatalError())
        {
            const double t = seq.step(&mock);
            const auto s = seq.getState();
            if (s == TorqueHomingSequence::State::EnablingDrive ||
                s == TorqueHomingSequence::State::Settle)
            { if (t != 0.0) zeroWhileEnabling = false; }
            if (s == TorqueHomingSequence::State::Search ||
                s == TorqueHomingSequence::State::Confirm)
            {
                if (t == -30.0) sawSearchTorque = true;
                else if (t != 0.0) torqueOk = false;   // only the final latch cycle is 0
            }
            mock.setSimCommandedTorque(t);
            mock.updateStatus();
            ++n;
        }
        CHECK(seq.isComplete(), "contract run completes");
        CHECK(zeroWhileEnabling, "zero torque while enabling and settling");
        CHECK(sawSearchTorque && torqueOk,
              "search commands exactly homeDir * homeTorquePct");
    }

    std::printf("TestTorqueHoming: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
