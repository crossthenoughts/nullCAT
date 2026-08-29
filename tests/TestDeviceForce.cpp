// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestDeviceForce.cpp - unit tests for the control-loading force engine.
//
// Pure-logic suite (no drives, no Qt Test): pins the curve sampler, the
// force-field terms (spring, lash, detent capture, stops, damping), the
// L2 modifiers, the L3 guard order and behaviour, the dir output mapping,
// and determinism. Runs on every platform.
// ============================================================

#include "../src/DeviceForceModel.h"
#include <cstdio>
#include <cmath>

static int g_pass = 0, g_fail = 0;
static void CHECK(bool ok, const char* what)
{
    if (ok) { ++g_pass; }
    else    { ++g_fail; std::printf("FAIL: %s\n", what); }
}

static DeviceParams baseParams()
{
    DeviceParams p;
    p.springCurve = { {0.0, 0.0}, {0.07, 175.0} };   // unsigned x: mirrored
    p.detentCurve = { {-0.02, 60.0}, {0.0, 0.0}, {0.02, -60.0} };
    p.detents     = { 0.0 };
    p.stopMinRev  = -0.07;
    p.stopMaxRev  =  0.07;
    p.dampPctPerRevS = 0.0;    // most tests want the static field only
    p.velLpfHz    = 0.0;       // no filter lag in unit tests
    p.slewPctPerSec = 1e9;     // effectively off unless a test sets it
    p.maxForcePct = 300.0;
    return p;
}

int main()
{
    const double DT = 0.0005;   // 2 kHz, the Pi force-loop rate

    // ================= curve sampler =================
    {
        std::vector<CurveNode> c;
        CHECK(DeviceForceModel::sampleCurve(c, 0.5) == 0.0, "empty curve = 0");
        c = { {0.0, 10.0} };
        CHECK(DeviceForceModel::sampleCurve(c, -1) == 10.0 &&
              DeviceForceModel::sampleCurve(c, 1) == 10.0, "single node clamps both ways");
        c = { {0.0, 0.0}, {1.0, 100.0} };
        CHECK(std::fabs(DeviceForceModel::sampleCurve(c, 0.25) - 25.0) < 1e-12,
              "linear interpolation");
        CHECK(DeviceForceModel::sampleCurve(c, 2.0) == 100.0, "end clamp high");
    }

    // ================= spring restores, lash is dead ==============
    {
        DeviceParams p = baseParams();
        p.detents.clear(); p.detentCurve.clear();
        DeviceForceModel m; m.configure(p, DT);
        m.reset(0.0);
        DeviceStateMods inert;
        CHECK(m.step(0.035, inert) < 0.0, "positive displacement -> negative (restoring) torque");
        m.reset(0.0);
        CHECK(m.step(-0.035, inert) > 0.0, "negative displacement -> positive torque");

        p.lashRev = 0.01;
        DeviceForceModel ml; ml.configure(p, DT); ml.reset(0.0);
        CHECK(ml.step(0.005, inert) == 0.0, "inside the lash band: zero force");
        ml.reset(0.0);
        const double fL = ml.step(0.03, inert);
        DeviceForceModel mn; p.lashRev = 0.0; mn.configure(p, DT); mn.reset(0.0);
        const double fN = mn.step(0.02, inert);
        CHECK(std::fabs(fL - fN) < 1e-9, "beyond lash: displacement measured from the band edge");
    }

    // ================= detent capture (nearest only) ==============
    // Same resist-convention as the spring: y drawn positive at positive
    // rel means the profile RESISTS leaving the detent, i.e. captures.
    {
        DeviceParams p = baseParams();
        p.springCurve.clear();
        p.detentCurve = { {-0.02, -60.0}, {0.0, 0.0}, {0.02, 60.0} };
        p.detents = { -0.05, 0.0, 0.05 };
        DeviceForceModel m; m.configure(p, DT); m.reset(0.0);
        DeviceStateMods inert;
        // At +0.04 the NEAREST detent is +0.05 (rel = -0.01): capture pulls
        // the lever UP toward it (positive force).
        CHECK(m.step(0.04, inert) > 0.0, "approaching a detent from below pulls in");
        m.reset(0.0);
        // At +0.01 the nearest is 0.0 (rel = +0.01): capture pulls back DOWN.
        CHECK(m.step(0.01, inert) < 0.0, "leaving a detent pulls back toward it");
        // An INVERTED profile is equally legal config: it pushes through
        // (the over-centre click) - only the feel differs.
        p.detentCurve = { {-0.02, 60.0}, {0.0, 0.0}, {0.02, -60.0} };
        DeviceForceModel mo; mo.configure(p, DT); mo.reset(0.0);
        CHECK(mo.step(0.01, inert) > 0.0, "over-centre profile pushes through");
    }

    // ================= stops + damping ==================
    {
        DeviceParams p = baseParams();
        p.springCurve.clear(); p.detents.clear(); p.detentCurve.clear();
        DeviceForceModel m; m.configure(p, DT); m.reset(0.0);
        DeviceStateMods inert;
        CHECK(m.step(0.08, inert) < -100.0, "past the stop: hard wall force");
        p.dampPctPerRevS = 50.0;
        DeviceForceModel md; md.configure(p, DT); md.reset(0.0);
        md.step(0.0, inert);
        const double fv = md.step(0.001, inert);   // moving +2 rev/s
        CHECK(fv < 0.0, "viscous damping opposes velocity");
    }

    // ================= L2 modifiers =================
    {
        DeviceParams p = baseParams();
        p.detents.clear(); p.detentCurve.clear();
        DeviceForceModel m; m.configure(p, DT); m.reset(0.0);
        DeviceStateMods half; half.forceScale = 0.5;
        DeviceStateMods full;
        const double fFull = m.step(0.03, full);
        m.reset(0.0);
        const double fHalf = m.step(0.03, half);
        CHECK(std::fabs(fHalf - 0.5 * fFull) < 1e-9, "forceScale scales the field");

        DeviceStateMods trim; trim.neutralShift = 0.03;
        m.reset(0.03);
        CHECK(m.step(0.03, trim) == 0.0, "neutralShift moves the rest point (trim)");

        DeviceStateMods tex; tex.textureAmpPct = 10.0; tex.textureFreqHz = 40.0;
        DeviceForceModel mt; mt.configure(p, DT); mt.reset(0.0);
        bool nonzero = false;
        double peak = 0.0;
        for (int i = 0; i < 200; ++i)
        {
            const double f = mt.step(0.0, tex);   // at neutral: pure texture
            if (f != 0.0) nonzero = true;
            peak = std::max(peak, std::fabs(f));
        }
        CHECK(nonzero && peak <= 10.0 + 1e-9 && peak > 5.0,
              "texture overlay oscillates within its amplitude");
    }

    // ================= L3 guards =================
    {
        DeviceParams p = baseParams();
        p.detents.clear(); p.detentCurve.clear();
        p.slewPctPerSec = 1000.0;    // 0.5 %/cycle at 2 kHz
        DeviceForceModel m; m.configure(p, DT); m.reset(0.0);
        DeviceStateMods inert;
        const double f1 = m.step(0.05, inert);   // wants ~ -125 instantly
        CHECK(std::fabs(f1) <= 1000.0 * DT + 1e-9, "slew cap bounds the first step");

        p = baseParams();
        p.detents.clear(); p.detentCurve.clear();
        p.foldRpm = 120.0;           // 2 rev/s
        DeviceForceModel mf; mf.configure(p, DT); mf.reset(0.0);
        DeviceStateMods it;
        double pos = 0.0;
        double fFast = 0.0;
        for (int i = 0; i < 400; ++i) { pos += 0.005; fFast = mf.step(pos, it); }  // 10 rev/s
        CHECK(fFast == 0.0, "velocity fold zeroes output at runaway speed");

        p = baseParams();
        p.detents.clear(); p.detentCurve.clear();
        p.thermalDwellSec = 0.05;    // 100 cycles at 2 kHz
        p.thermalPct      = 20.0;
        p.maxForcePct     = 200.0;
        DeviceForceModel mh; mh.configure(p, DT); mh.reset(0.0);
        double fHot = -1.0;
        for (int i = 0; i < 200; ++i) fHot = mh.step(0.05, it);   // ~125% sustained
        CHECK(fHot == 0.0 && mh.thermallyRelaxed(),
              "thermal dwell eases sustained near-max force to zero");
    }

    // ================= dir mapping + determinism =================
    {
        DeviceParams p = baseParams();
        p.detents.clear(); p.detentCurve.clear();
        DeviceForceModel a; a.configure(p, DT); a.reset(0.0);
        p.dir = -1.0;
        DeviceForceModel b; b.configure(p, DT); b.reset(0.0);
        DeviceStateMods inert;
        const double fa = a.step(0.03, inert);
        const double fb = b.step(0.03, inert);
        CHECK(std::fabs(fa + fb) < 1e-12, "dir=-1 flips ONLY the output torque sign");

        p.dir = 1.0;
        DeviceForceModel d1; d1.configure(p, DT);
        DeviceForceModel d2; d2.configure(p, DT);
        d1.reset(0.0); d2.reset(0.0);
        bool identical = true;
        double pos = 0.0;
        DeviceStateMods mods; mods.textureAmpPct = 5.0; mods.textureFreqHz = 33.0;
        for (int i = 0; i < 5000; ++i)
        {
            pos = 0.05 * std::sin(0.01 * i);
            if (d1.step(pos, mods) != d2.step(pos, mods)) { identical = false; break; }
        }
        CHECK(identical, "bit-identical across instances for identical inputs");
    }

    std::printf("TestDeviceForce: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
