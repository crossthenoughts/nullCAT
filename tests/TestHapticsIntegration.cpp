// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestHapticsIntegration.cpp
//
// The ring AROUND the haptics layer, driven through MotionController::process()
// with no drive object (the belt torque command is controller-computed). The
// layer itself is pinned by TestHaptics; this pins the lines that decide
// whether anything the layer produces ever reaches a drive, and under what
// limits - the most safety-relevant behaviour in the feature, which had no
// test (the stuck-engine and cannot-save bench bugs both lived out here).
//
//   I-1  PARKED (belt slack): a live, full-scale skid channel produces NO
//        torque - the overlay only applies to ONLINE/BLENDING axes
//   I-2  ONLINE: the same channel modulates the belt torque (an oscillation
//        rides on the tension), and the SUM never leaves [0, torqueMaxPct]
//        even with amp 100 and route gain 2
//   I-3  Live-apply: stageHaptics() from the web thread lands on the next
//        cycle (effect off -> on) without a re-initialize
//   I-4  Stale stream: ncxFresh false fades the effect to nothing
//   I-5  Mute (command queue) silences it; unmute brings it back
//   I-6  E-stop clears it instantly, and the belt torque is 0
//   I-7  Laws: a hand-built NcxValues drives the expected effect levels
//        (ABS needs the brake; flags; magnitudes; gear edge fires once)
// ============================================================

#include "MotionController.h"
#include "TelemetryInput.h"
#include "Config.h"
#include "HapticLaws.h"
#include "MockA6Drive.h"

#include <cstdio>
#include <cmath>
#include <algorithm>

static int g_fail = 0, g_pass = 0;
static void check(bool ok, const char* name)
{
    if (ok) { ++g_pass; std::printf("[PASS] %s\n", name); }
    else    { ++g_fail; std::printf("[FAIL] %s\n", name); }
}

static AppConfig makeBeltConfig(double skidAmp, double routeGain)
{
    DriveConfig dc;
    dc.slaveIndex          = 1;
    dc.name                = "Belt";
    dc.axisType            = "belt";
    dc.mode                = "torque";
    dc.torqueMinPct        = 5.0;
    dc.torqueMaxPct        = 50.0;
    dc.strokeMm            = 100.0;
    dc.countsPerMm         = 100.0;
    dc.maxVelocityMmS      = 200.0;
    dc.maxAccelerationMmS2 = 2000.0;
    dc.maxJerkMmS3         = 20000.0;
    dc.unparkTimeSec       = 0.1;
    dc.parkTimeSec         = 0.1;
    dc.onlineHoldTimeoutSec = 5.0;

    AppConfig cfg;
    cfg.controlLoopHz       = 500;     // 2 ms cycle: a 35 Hz skid is well resolved
    cfg.numDrives           = 1;
    cfg.blendTimeSec        = 0.1;
    cfg.blendMaxVelocityMmS = 20.0;
    cfg.drives.push_back(dc);
    // Default bindings (all 13 tokens, slot = index) ship with AppConfig.
    haptics::EffectParams& skid = cfg.hapticsFx[static_cast<size_t>(haptics::Effect::Skid)];
    skid.ampPct    = skidAmp;
    skid.routes[0] = { 0, routeGain };
    return cfg;
}

// Motion frame (belt demand zero) + a NULLCATX frame with the skid channel.
static TelemetryData frame(double skidPct, bool fresh, double raw0 = 0.0)
{
    TelemetryData sd{};
    sd.valid        = true;
    sd.numPositions = 2;
    sd.positions[0] = raw0;
    sd.positions[1] = 32767.0;
    sd.packetType   = TelemetryPacketType::Motion;
    sd.numNcx       = 13;
    sd.ncx[static_cast<int>(NcxValues::Skid)] = skidPct;
    sd.ncxFresh     = fresh;
    return sd;
}

struct Span { double lo = 1e9, hi = -1e9; double width() const { return hi - lo; } };

// Run cycles; return the torque span over the LAST `measure` cycles.
static Span run(MotionController& mc, const TelemetryData& sd, int cycles, int measure)
{
    Span sp; MotionOutput out{};
    for (int i = 0; i < cycles; ++i)
    {
        out = MotionOutput{};
        mc.process(sd, out, nullptr, 0);
        if (i >= cycles - measure)
        {
            sp.lo = std::min(sp.lo, out.torques[0]);
            sp.hi = std::max(sp.hi, out.torques[0]);
        }
    }
    return sp;
}

static void tensionBelt(MotionController& mc)
{
    MotionCommand c; c.type = MotionCommand::Type::TensionBelts; mc.enqueueCommand(c);
}

int main()
{
    Logger::instance().setMinLevel(LogLevel::LVL_ERROR);

    // ---- I-1: parked = no haptic torque, whatever the channel says ----
    {
        MotionController mc;
        mc.configure(makeBeltConfig(50.0, 1.0));
        mc.setEmergencyStop(false);
        const Span sp = run(mc, frame(100.0, true), 300, 200);
        check(mc.getAxisState(0) == AxisMotionState::PARKED, "I-1 belt is PARKED (slack) before tensioning");
        check(sp.hi <= 1e-9 && sp.lo >= -1e-9, "I-1 PARKED: full-scale skid channel produces no torque");
    }

    // ---- I-2: online = modulation inside the axis limits ----
    {
        MotionController mc;
        mc.configure(makeBeltConfig(100.0, 2.0));   // deliberately over the top
        mc.setEmergencyStop(false);
        tensionBelt(mc);
        run(mc, frame(0.0, true), 400, 1);           // settle ONLINE, effect silent
        check(mc.getAxisState(0) == AxisMotionState::ONLINE, "I-2 belt is ONLINE after tensioning");
        const Span quiet = run(mc, frame(0.0, true), 200, 100);
        const Span loud  = run(mc, frame(100.0, true), 300, 100);
        check(quiet.width() < 0.5,            "I-2 skid 0: belt torque steady");
        check(loud.width()  > 5.0,            "I-2 skid 100: an oscillation rides on the tension");
        check(loud.lo >= -1e-9 && loud.hi <= 50.0 + 1e-9,
              "I-2 the SUM stays inside [0, torqueMaxPct] with amp 100 x gain 2");
    }

    // ---- I-3: live-apply lands on the next cycle ----
    {
        MotionController mc;
        mc.configure(makeBeltConfig(0.0, 1.0));      // effect off in the running config
        mc.setEmergencyStop(false);
        tensionBelt(mc);
        run(mc, frame(100.0, true), 400, 1);
        const Span before = run(mc, frame(100.0, true), 200, 100);
        mc.stageHaptics(makeBeltConfig(50.0, 1.0));  // "web thread": a rig save
        const Span after  = run(mc, frame(100.0, true), 200, 100);
        check(before.width() < 0.5,           "I-3 before the save: silent");
        check(after.width()  > 5.0,           "I-3 after stageHaptics: the effect plays, no re-initialize");
    }

    // ---- I-4: stale stream fades out ----
    {
        MotionController mc;
        mc.configure(makeBeltConfig(50.0, 1.0));
        mc.setEmergencyStop(false);
        tensionBelt(mc);
        run(mc, frame(100.0, true), 500, 1);
        const Span live  = run(mc, frame(100.0, true), 200, 100);
        const Span stale = run(mc, frame(100.0, false), 250, 100);   // 0.5 s stale
        check(live.width()  > 5.0,            "I-4 fresh stream: effect plays");
        check(stale.width() < 0.5,            "I-4 stale stream: effect faded to nothing");
    }

    // ---- I-5: mute / unmute through the command queue ----
    {
        MotionController mc;
        mc.configure(makeBeltConfig(50.0, 1.0));
        mc.setEmergencyStop(false);
        tensionBelt(mc);
        run(mc, frame(100.0, true), 500, 1);
        MotionCommand m; m.type = MotionCommand::Type::HapticsMute; m.intVal = 1; mc.enqueueCommand(m);
        const Span muted = run(mc, frame(100.0, true), 200, 100);
        m.intVal = 0; mc.enqueueCommand(m);
        const Span back  = run(mc, frame(100.0, true), 200, 100);
        check(muted.width() < 0.5,            "I-5 mute: silent");
        check(back.width()  > 5.0,            "I-5 unmute: plays again");
    }

    // ---- I-6: e-stop clears instantly ----
    {
        MotionController mc;
        mc.configure(makeBeltConfig(50.0, 1.0));
        mc.setEmergencyStop(false);
        tensionBelt(mc);
        run(mc, frame(100.0, true), 500, 1);
        mc.setEmergencyStop(true);
        const Span es = run(mc, frame(100.0, true), 50, 40);
        check(es.hi <= 1e-9 && es.lo >= -1e-9, "I-6 e-stop: belt torque 0, no haptic overlay");
    }

    // ---- I-7: the laws, with no controller at all ----
    {
        haptics::Layer L;
        haptics::LawsState st;
        haptics::EffectParams p; p.ampPct = 50.0; p.freqHz = 12.0; p.routes[0] = { 0, 1.0 };
        for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
        {
            const haptics::EffectInfo& info = haptics::effectInfo(i);
            if (info.kind == haptics::Kind::Transient) L.configure(info.event, p);
            else                                       L.configureFx(info.fx, p);
        }
        NcxValues v{}; v.fresh = true;
        for (int i = 0; i < NcxValues::TokenCount; ++i) v.have[i] = true;
        const double dt = 0.002;
        auto settle = [&](int n) { for (int i = 0; i < n; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); } };

        // ABS flag with no brake: nothing. With brake: pulses.
        v.val[NcxValues::AbsActive] = 1.0; v.val[NcxValues::BrakePct] = 0.0;  settle(100);
        check(L.fxLevel(static_cast<int>(haptics::FxType::AbsPulse)) < 0.05, "I-7 ABS flag alone (no brake) stays silent");
        v.val[NcxValues::BrakePct] = 60.0; settle(100);
        check(L.fxLevel(static_cast<int>(haptics::FxType::AbsPulse)) > 0.9,  "I-7 ABS flag + brake drives the pulse");

        // Magnitude channel: 50 -> level 0.5.
        v.val[NcxValues::Skid] = 50.0; settle(200);
        check(std::fabs(L.fxLevel(static_cast<int>(haptics::FxType::Skid)) - 0.5) < 0.05, "I-7 skid 50 drives level 0.5");

        // Flag channel: TC.
        v.val[NcxValues::TcActive] = 1.0; settle(100);
        check(L.fxLevel(static_cast<int>(haptics::FxType::TcPulse)) > 0.9, "I-7 tcActive drives the TC pulse");

        // Gear edge: a change fires ONE transient; a steady gear fires none;
        // the first sighting after staleness never fires.
        v.val[NcxValues::Gear] = 3.0; settle(10);
        const uint64_t n0 = L.fireCount();
        v.val[NcxValues::Gear] = 4.0; settle(10);
        const uint64_t n1 = L.fireCount();
        settle(100);
        const uint64_t n2 = L.fireCount();
        check(n1 == n0 + 1, "I-7 gear change fires one gear-shift transient");
        check(n2 == n1,     "I-7 steady gear fires nothing more");
        v.fresh = false; settle(10); v.fresh = true; v.val[NcxValues::Gear] = 2.0; settle(10);
        check(L.fireCount() == n2, "I-7 a returning stream never fires a stale crossing");

        // Stale: every level fades.
        v.fresh = false; settle(300);
        check(L.fxLevel(static_cast<int>(haptics::FxType::Skid)) < 0.01
              && L.fxLevel(static_cast<int>(haptics::FxType::TcPulse)) < 0.01, "I-7 stale stream fades every effect");
    }

    // ---- P-1..P-4: a POSITION (CSP) axis as a routing destination ----
    // Route gain on a position axis is mm at 100% amplitude. The offset is
    // tracked inside the haptic share of the axis limits (what the actuator
    // can follow), capped at hapticsMaxMm, and the sum passes the full axis
    // guard. A low-frequency effect comes through at its physics amplitude;
    // a high-frequency one is attenuated by the acceleration budget, never
    // clipped by the drive. PARKED produces nothing.
    {
        auto makePosConfig = [](double skidAmp, double gainMm, double freqHz, double capMm, double budget) {
            DriveConfig dc;
            dc.slaveIndex = 1; dc.name = "Seat"; dc.axisType = "linear_vertical";
            dc.strokeMm = 100.0; dc.homingSpeed = 400.0; dc.homingBackoffMm = 1.5; dc.homingTorquePct = 25;
            dc.homeDirection = "negative"; dc.invertDir = true; dc.parkMode = "endstop";
            dc.maxVelocityMmS = 200.0; dc.maxAccelerationMmS2 = 2000.0; dc.maxJerkMmS3 = 20000.0;
            dc.unparkTimeSec = 0.1; dc.parkTimeSec = 0.1; dc.countsPerMm = 100.0; dc.ballscrewPitch = 5.0;
            dc.hapticsMaxMm = capMm;
            AppConfig cfg;
            cfg.controlLoopHz = 500; cfg.numDrives = 1; cfg.drives.push_back(dc);
            cfg.hapticsPositionBudget = budget;
            haptics::EffectParams& skid = cfg.hapticsFx[static_cast<size_t>(haptics::Effect::Skid)];
            skid.ampPct = skidAmp; skid.freqHz = freqHz; skid.jitter = 0.0;
            skid.routes[0] = { 0, gainMm };
            return cfg;
        };
        // Drive the controller with a mock drive (homes against a hardstop,
        // auto-unparks, settles ONLINE); return the position span over the
        // last `measure` cycles.
        auto posRun = [](MotionController& mc, MockA6Drive& mock, const TelemetryData& sd, int cycles, int measure) {
            Span sp; A6Drive* drives[1] = { &mock }; MotionOutput out{};
            for (int i = 0; i < cycles; ++i)
            {
                out = MotionOutput{};
                mc.process(sd, out, drives, 1);
                if (i >= cycles - measure) { sp.lo = std::min(sp.lo, out.positions[0]); sp.hi = std::max(sp.hi, out.positions[0]); }
            }
            return sp;
        };
        auto bringOnline = [&](MotionController& mc, MockA6Drive& mock) {
            mc.setEmergencyStop(false);
            mock.setHardstop(-2.0, true, 50.0);
            mc.startHoming();
            TelemetryData empty{}; A6Drive* drives[1] = { &mock };
            for (int i = 0; i < 20000 && mc.getAxisState(0) != AxisMotionState::ONLINE; ++i)
            { MotionOutput out{}; mc.process(empty, out, drives, 1); }
            return mc.getAxisState(0) == AxisMotionState::ONLINE;
        };

        // P-1: 5 Hz skid at 2 mm gain. Budget 0.4 x 2000 mm/s^2 allows
        // A <= 800/(2*pi*5)^2 = 0.81 mm at 5 Hz, so the offset should
        // swing ~+-0.8 mm around the cue (less than the 2 mm asked, more
        // than the 3 mm cap could ever bite).
        {
            MotionController mc; MockA6Drive mock;
            mc.configure(makePosConfig(100.0, 2.0, 5.0, 3.0, 0.4));
            check(bringOnline(mc, mock), "P-1 position axis reaches ONLINE");
            const Span quiet = posRun(mc, mock, frame(0.0, true, 32767.0), 600, 100);
            const Span loud  = posRun(mc, mock, frame(100.0, true, 32767.0), 1000, 400);
            check(quiet.width() < 0.05,                          "P-1 skid 0: position command steady");
            // 0.4 x 2000 / (2 pi 5)^2 = 0.81 mm peak -> ~1.62 mm swing.
            check(loud.width() > 1.4 && loud.width() < 1.9,       "P-1 5 Hz skid: ~+-0.8 mm swing (accel budget / w^2), not the 2 mm asked");
        }
        // P-2: the same at 35 Hz: the accel budget allows only ~0.017 mm,
        // so the swing is small - the drive is never asked for more than it
        // can follow.
        {
            MotionController mc; MockA6Drive mock;
            mc.configure(makePosConfig(100.0, 2.0, 35.0, 3.0, 0.4));
            check(bringOnline(mc, mock), "P-2 position axis reaches ONLINE");
            posRun(mc, mock, frame(0.0, true, 32767.0), 600, 1);
            const Span loud = posRun(mc, mock, frame(100.0, true, 32767.0), 1000, 400);
            // 0.4 x 2000 / (2 pi 35)^2 = 0.0165 mm peak -> ~0.033 mm swing.
            check(loud.width() > 0.02 && loud.width() < 0.06,     "P-2 35 Hz skid: attenuated to what the axis can follow (~0.03 mm swing)");
        }
        // P-3: hapticsMaxMm caps it: 1 Hz (budget allows 20 mm) at 5 mm
        // gain, cap 0.5 mm -> ~+-0.5 mm.
        {
            MotionController mc; MockA6Drive mock;
            mc.configure(makePosConfig(100.0, 5.0, 1.0, 0.5, 0.4));
            check(bringOnline(mc, mock), "P-3 position axis reaches ONLINE");
            posRun(mc, mock, frame(0.0, true, 32767.0), 600, 1);
            const Span loud = posRun(mc, mock, frame(100.0, true, 32767.0), 2000, 1000);
            check(loud.width() > 0.9 && loud.width() < 1.1,       "P-3 hapticsMaxMm caps the offset at +-0.5 mm");
        }
        // P-4: hapticsMaxMm 0 = this axis takes no haptics; PARKED takes none.
        {
            MotionController mc; MockA6Drive mock;
            mc.configure(makePosConfig(100.0, 2.0, 5.0, 0.0, 0.4));
            check(bringOnline(mc, mock), "P-4 position axis reaches ONLINE");
            posRun(mc, mock, frame(0.0, true, 32767.0), 600, 1);
            const Span loud = posRun(mc, mock, frame(100.0, true, 32767.0), 600, 200);
            check(loud.width() < 0.05,                            "P-4 hapticsMaxMm 0: no haptics on this axis");
            MotionController mc2; MockA6Drive mock2;
            mc2.configure(makePosConfig(100.0, 2.0, 5.0, 3.0, 0.4));
            mc2.setEmergencyStop(false);
            const Span parked = posRun(mc2, mock2, frame(100.0, true), 600, 200);
            check(mc2.getAxisState(0) == AxisMotionState::PARKED, "P-4 un-homed axis is PARKED");
            check(parked.width() < 0.05,                          "P-4 PARKED: no haptics");
        }
    }

    std::printf("TestHapticsIntegration: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
