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
        // The 0.9.6 token set only: the per-wheel 1.3 tokens stay unbound so
        // the single skid/lockup channels take their fallback path here.
        for (int i = 0; i <= NcxValues::Curbs; ++i) v.have[i] = true;
        const double dt = 0.002;
        auto settle = [&](int n) { for (int i = 0; i < n; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); } };

        // ABS flag with no brake: nothing. With brake: pulses.
        v.val[NcxValues::AbsActive] = 1.0; v.val[NcxValues::BrakePct] = 0.0;  settle(100);
        check(L.fxLevel(static_cast<int>(haptics::FxType::AbsPulse)) < 0.05, "I-7 ABS flag alone (no brake) stays silent");
        v.val[NcxValues::BrakePct] = 60.0; settle(100);
        check(L.fxLevel(static_cast<int>(haptics::FxType::AbsPulse)) > 0.9,  "I-7 ABS flag + brake drives the pulse");

        // Single skid channel (fallback): 50 -> every wheel at 0.5 -> tile level 0.5.
        v.val[NcxValues::Skid] = 50.0; settle(200);
        check(std::fabs(L.fxLevel(static_cast<int>(haptics::FxType::Skid)) - 0.5) < 0.05, "I-7 skid 50 drives level 0.5");
        check(std::fabs(L.slipWheelLevel(haptics::FxType::Skid, WheelRR) - 0.5) < 0.05, "I-7 single skid channel reaches every wheel");
        v.val[NcxValues::Skid] = 0.0; settle(300);

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

    // ---- I-8: the per-wheel slip laws (protocol 1.3 raw physics in) ----
    {
        using haptics::FxType;
        haptics::Layer L;
        haptics::LawsState st;
        haptics::EffectParams p; p.ampPct = 50.0; p.routes[0] = { 0, 1.0 };
        L.configureFx(FxType::Skid, p);   L.configureSlip(FxType::Skid,   { 1.0, 25.0, 1.0, 11.0, 7.0 });
        L.configureFx(FxType::Lockup, p); L.configureSlip(FxType::Lockup, { 1.0, 9.0, 1.0, 10.0, 0.8 });
        NcxValues v{}; v.fresh = true;
        const double dt = 0.002;
        auto settle = [&](int n) { for (int i = 0; i < n; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); } };
        auto lvl = [&](FxType t, int w) { return L.slipWheelLevel(t, w); };
        auto setW = [&](int group, double fl, double fr, double rl, double rr)
        {
            const double x[4] = { fl, fr, rl, rr };
            for (int w = 0; w < 4; ++w) { v.have[group + w] = true; v.val[group + w] = x[w]; }
        };
        v.have[NcxValues::SpeedKmh] = true; v.val[NcxValues::SpeedKmh] = 100.0;

        // Lateral: slip angle past 60% of peak deg. Peak 7: 4.2 deg = onset,
        // 7 deg = full; 5.6 deg = halfway. Only FL slides.
        setW(NcxValues::SlipAngleFL, 7.0, 0.0, 0.0, 0.0); settle(100);
        check(lvl(FxType::Skid, WheelFL) > 0.95 && lvl(FxType::Skid, WheelFR) < 0.01 && lvl(FxType::Skid, WheelRR) < 0.01,
              "I-8 lateral: 7 deg on FL alone drives FL alone");
        setW(NcxValues::SlipAngleFL, -5.6, 3.0, 0.0, 0.0); settle(100);
        check(std::fabs(lvl(FxType::Skid, WheelFL) - 0.5) < 0.05, "I-8 lateral: -5.6 deg (sign ignored) = halfway");
        check(lvl(FxType::Skid, WheelFR) < 0.01, "I-8 lateral: 3 deg is normal cornering, nothing");
        // A single skid channel is ignored once per-wheel angles arrive.
        v.have[NcxValues::Skid] = true; v.val[NcxValues::Skid] = 100.0; settle(100);
        check(lvl(FxType::Skid, WheelRR) < 0.01, "I-8 lateral: per-wheel angles win over the single channel");
        v.have[NcxValues::Skid] = false;

        // Load weighting: the loaded tyre is weighted up (capped at 1.5x).
        setW(NcxValues::SlipAngleFL, 5.6, 5.6, 0.0, 0.0);
        setW(NcxValues::LoadFL, 6000.0, 1000.0, 2500.0, 2500.0);   // mean 3000: FL 2x -> 1.5 cap, FR 0.33
        settle(100);
        check(std::fabs(lvl(FxType::Skid, WheelFL) - 0.75) < 0.05, "I-8 load: the loaded FL at 0.5 x 1.5 = 0.75");
        check(std::fabs(lvl(FxType::Skid, WheelFR) - 0.167) < 0.05, "I-8 load: the unloaded FR at 0.5 x 0.33");
        for (int w = 0; w < 4; ++w) v.have[NcxValues::LoadFL + w] = false;
        setW(NcxValues::SlipAngleFL, 0.0, 0.0, 0.0, 0.0); settle(300);

        // Longitudinal from slip ratio: - locks, + spins; onset 0.15, full at
        // peak 0.8. FL -0.8 = full lock; RR +0.475 = halfway spin.
        setW(NcxValues::SlipRatioFL, -0.8, 0.0, 0.0, 0.475); settle(100);
        check(lvl(FxType::Lockup, WheelFL) > 0.95, "I-8 longitudinal: ratio -0.8 on FL = full lock on FL");
        check(std::fabs(lvl(FxType::Lockup, WheelRR) - 0.5) < 0.05, "I-8 longitudinal: ratio +0.475 on RR = halfway spin");
        check(lvl(FxType::Lockup, WheelFR) < 0.01, "I-8 longitudinal: a rolling wheel is silent");
        // Nothing at a standstill: ratios mean nothing there.
        v.val[NcxValues::SpeedKmh] = 2.0; settle(300);
        check(lvl(FxType::Lockup, WheelFL) < 0.01, "I-8 longitudinal: silent below 5 km/h");
        v.val[NcxValues::SpeedKmh] = 100.0;
        for (int w = 0; w < 4; ++w) v.have[NcxValues::SlipRatioFL + w] = false;
        settle(300);

        // Wheel speeds in a sender's own unit: the rolling factor is learned
        // while cruising (brake 0, throttle < 30, > 30 km/h), then the ratio
        // comes from wheelSpeed x k / speed - 1.
        v.have[NcxValues::ThrottlePct] = true; v.val[NcxValues::ThrottlePct] = 10.0;
        v.have[NcxValues::BrakePct]    = true; v.val[NcxValues::BrakePct]    = 0.0;
        setW(NcxValues::WheelSpeedFL, 10.0, 10.0, 11.0, 11.0);   // rev/s; rears on taller tyres
        settle(1500);                                             // learn: k = 10 fronts, 9.09 rears
        check(lvl(FxType::Lockup, WheelRL) < 0.01, "I-8 wheel speed: staggered tyres read as rolling after learning");
        v.val[NcxValues::BrakePct] = 80.0;
        setW(NcxValues::WheelSpeedFL, 5.0, 10.0, 11.0, 11.0);    // FL half speed: ratio -0.5 -> (0.5-0.15)/0.65 = 0.54
        settle(100);
        check(std::fabs(lvl(FxType::Lockup, WheelFL) - 0.54) < 0.06, "I-8 wheel speed: FL at half speed under braking = 0.54 lock");
        check(lvl(FxType::Lockup, WheelFR) < 0.01, "I-8 wheel speed: the other front rolls");
        // A braking frame never retrains the factor (FL would otherwise learn 20).
        settle(500);
        check(std::fabs(lvl(FxType::Lockup, WheelFL) - 0.54) < 0.06, "I-8 wheel speed: the factor is not learned while braking");
        for (int w = 0; w < 4; ++w) v.have[NcxValues::WheelSpeedFL + w] = false;
        settle(300);

        // Single lockup channel: the fallback when neither ratios nor wheel
        // speeds arrive. 100 -> every wheel fully locked, no spin.
        v.have[NcxValues::Lockup] = true; v.val[NcxValues::Lockup] = 100.0; settle(100);
        check(lvl(FxType::Lockup, WheelRR) > 0.95, "I-8 single lockup channel reaches every wheel");

        // Engine law: the pit limiter flag cuts the engine (holes in the
        // output) but the learned redline stays put; an engine joined
        // running needs no catch; cranking rpm plays (lumps, not silence).
        {
            // A fresh layer: the slip and road routes above share axis 0.
            haptics::Layer L; haptics::LawsState st;
            auto settleE = [&](int n) { for (int i = 0; i < n; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); } };
            for (int t = 0; t < NcxValues::TokenCount; ++t) v.have[t] = false;
            haptics::EffectParams ep; ep.ampPct = 60.0; ep.freqHz = 30.0; ep.routes[0] = { 0, 1.0 };
            haptics::EngineParams eg; eg.maxRpm = 0.0; eg.inertia = 0.0; eg.liftoff = 0.0;
            L.configureFx(FxType::RpmVibe, ep); L.configureEngine(eg);
            (void)settleE;
            v.have[NcxValues::Rpm] = true; v.val[NcxValues::Rpm] = 1500.0;
            v.have[NcxValues::ThrottlePct] = true; v.val[NcxValues::ThrottlePct] = 40.0;
            v.have[NcxValues::PitLimiter] = true; v.val[NcxValues::PitLimiter] = 1.0;
            const double seed = L.learnedMaxRpm();
            int holes = 0; bool inHole = false;
            for (int i = 0; i < 1000; ++i)
            {
                haptics::driveLaws(L, st, v, dt); L.step(dt);
                const bool z = std::fabs(L.overlayFor(0)) <= 1e-9;
                if (i > 200 && z && !inHole) ++holes;
                inHole = z;
            }
            check(holes >= 6, "I-8 engine: the pit limiter token cuts the engine in bursts");
            check(std::fabs(L.learnedMaxRpm() - seed) < 1e-9, "I-8 engine: the pit limiter never teaches the redline");
            v.val[NcxValues::PitLimiter] = 0.0;
            v.val[NcxValues::Rpm] = 250.0; v.val[NcxValues::ThrottlePct] = 0.0;
            double crank = 0.0;
            for (int i = 0; i < 400; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); crank = std::max(crank, std::fabs(L.overlayFor(0))); }
            check(crank > 0.5, "I-8 engine: 250 rpm on the starter plays cranking lumps");
        }

        // Driveline law: judder only with the clutch in the slipping band and
        // slip across it, scaled by load; a launch from rest is full slip; a
        // known ratio makes it exact; nothing with the clutch up or in
        // neutral. Lug only at high throttle and low revs in gear.
        {
            haptics::Layer L; haptics::LawsState st;
            for (int t = 0; t < NcxValues::TokenCount; ++t) v.have[t] = false;
            haptics::EffectParams dp; dp.ampPct = 100.0; dp.jitter = 0.0; dp.routes[0] = { 0, 1.0 };
            haptics::EngineParams eg; eg.maxRpm = 7000.0;
            L.configureFx(FxType::Driveline, dp); L.configureDriveline({ 1.0, 10.0, 1.0, 7.0 }); L.configureEngine(eg);
            auto set = [&](NcxValues::Token t, double val) { v.have[t] = true; v.val[t] = val; };
            GearRatios ratios; ratios.r[2] = 60.0; ratios.known[2] = true;   // 2nd gear: 60 rpm per km/h
            auto settleD = [&](int n) { for (int i = 0; i < n; ++i) { haptics::driveLaws(L, st, v, dt, &ratios); L.step(dt); } };
            auto judder = [&]() { return L.drivelineJudderLevel(); };
            auto lug    = [&]() { return L.drivelineLugLevel(); };

            // Launch: 1st gear, 3000 rpm, standing still, clutch mid-band, full throttle.
            set(NcxValues::Rpm, 3000.0); set(NcxValues::Gear, 1.0); set(NcxValues::SpeedKmh, 0.0);
            set(NcxValues::ClutchPct, 50.0); set(NcxValues::ThrottlePct, 100.0);
            settleD(200);
            check(judder() > 0.9, "I-8 driveline: a launch with the clutch mid-band judders fully");
            set(NcxValues::ClutchPct, 5.0); settleD(300);
            check(judder() < 0.02, "I-8 driveline: clutch up = no judder");
            set(NcxValues::ClutchPct, 95.0); settleD(300);
            check(judder() < 0.02, "I-8 driveline: clutch floored (open) = no judder");
            set(NcxValues::ClutchPct, 50.0); set(NcxValues::Gear, 0.0); settleD(300);
            check(judder() < 0.02, "I-8 driveline: neutral = no judder");
            // Rolling in 2nd with a known ratio: 60 km/h wants 3600 rpm; at
            // 3600 no slip, at 5400 half of the rpm is slip.
            set(NcxValues::Gear, 2.0); set(NcxValues::SpeedKmh, 60.0); set(NcxValues::Rpm, 3600.0); settleD(300);
            check(judder() < 0.02, "I-8 driveline: matched revs through a slipping-band clutch = no judder");
            set(NcxValues::Rpm, 5400.0); settleD(300);
            check(std::fabs(judder() - 0.33) < 0.06, "I-8 driveline: 1800 rpm over the ratio at 5400 = a third slip");
            set(NcxValues::ThrottlePct, 0.0); settleD(300);
            check(std::fabs(judder() - 0.05) < 0.03, "I-8 driveline: off throttle the judder drops to the floor");

            // Lug: in gear, throttle 100, 1200 rpm of a 7000 redline (x 0.17).
            set(NcxValues::ClutchPct, 0.0); set(NcxValues::ThrottlePct, 100.0); set(NcxValues::Rpm, 1200.0); set(NcxValues::SpeedKmh, 20.0);
            settleD(300);
            check(lug() > 0.6, "I-8 driveline: full throttle at 1200 rpm lugs");
            set(NcxValues::Rpm, 3000.0); settleD(300);
            check(lug() < 0.02, "I-8 driveline: at 3000 rpm the lug is gone");
            set(NcxValues::Rpm, 1200.0); set(NcxValues::ThrottlePct, 20.0); settleD(300);
            check(lug() < 0.02, "I-8 driveline: light throttle at low revs does not lug");
            set(NcxValues::ThrottlePct, 100.0); set(NcxValues::Gear, 0.0); settleD(300);
            check(lug() < 0.02, "I-8 driveline: no lug in neutral");
        }

        // Driveline: straight-cut whine, shunt and the gearbox-shaped shift.
        {
            haptics::Layer L; haptics::LawsState st;
            for (int t = 0; t < NcxValues::TokenCount; ++t) v.have[t] = false;
            haptics::EffectParams dp; dp.ampPct = 100.0; dp.jitter = 0.0; dp.routes[0] = { 0, 1.0 };
            haptics::DrivelineParams dd; dd.whine = 1.0; dd.shunt = 1.0; dd.shuntHz = 40.0; dd.gearbox = 0.0;
            haptics::EngineParams eg; eg.maxRpm = 7000.0;
            L.configureFx(FxType::Driveline, dp); L.configureDriveline(dd); L.configureEngine(eg);
            haptics::EffectParams gs; gs.ampPct = 100.0; gs.freqHz = 60.0; gs.durMs = 25.0; gs.routes[0] = { 1, 1.0 };
            L.configure(haptics::EventType::GearShift, gs);
            auto set = [&](NcxValues::Token t, double val) { v.have[t] = true; v.val[t] = val; };
            auto settleD = [&](int n) { for (int i = 0; i < n; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); } };
            auto hzOnAxis0 = [&](int n) {
                std::vector<double> o;
                for (int i = 0; i < n; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); o.push_back(L.overlayFor(0)); }
                int xr = 0; for (size_t i = 1; i < o.size(); ++i) if ((o[i] >= 0.0) != (o[i-1] >= 0.0)) ++xr;
                return xr / 2.0 / (o.size() * dt); };
            // 3500 rpm (half the redline), 3rd gear, full throttle: the whine
            // sits at the auto order for 3500 rpm x the 3rd-gear factor
            // (band top 62.5 Hz at 500 Hz loop: 62.5 x 0.5 x (1 - 0.06 x 3) = 25.6 Hz).
            set(NcxValues::Rpm, 3500.0); set(NcxValues::Gear, 3.0); set(NcxValues::ThrottlePct, 100.0); set(NcxValues::ClutchPct, 0.0);
            settleD(200);
            check(L.drivelineWhineLevel() > 0.95, "I-8 whine: full throttle in gear whines at full level");
            const double hz3 = hzOnAxis0(1000);
            check(std::fabs(hz3 - 25.6) < 1.5, "I-8 whine: the pitch follows rpm at the auto order (3rd gear)");
            set(NcxValues::Gear, 4.0); settleD(200);
            const double hz4 = hzOnAxis0(1000);
            check(hz4 > hz3 + 1.0, "I-8 whine: a higher gear steps the pitch up at the same rpm");
            set(NcxValues::ThrottlePct, 0.0); settleD(300);
            check(std::fabs(L.drivelineWhineLevel() - 0.3) < 0.05, "I-8 whine: off throttle it drops to the unloaded floor, not silence");
            set(NcxValues::ClutchPct, 95.0); settleD(300);
            check(L.drivelineWhineLevel() < 0.02, "I-8 whine: clutch open = no load path, no whine");
            set(NcxValues::ClutchPct, 0.0); set(NcxValues::Gear, 0.0); settleD(300);
            check(L.drivelineWhineLevel() < 0.02, "I-8 whine: neutral is silent");

            // Shunt: in 3rd, rolling, the throttle snapping from 0 to 80 within
            // a few ms knocks once; creeping up over a second does not; lifting
            // fast knocks again; nothing in neutral.
            auto knocks = [&](int n) {
                int bursts = 0; bool in = false;
                for (int i = 0; i < n; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); const bool on = std::fabs(L.overlayFor(0)) > 1e-9; if (on && !in) ++bursts; in = on; }
                return bursts; };
            dd.whine = 0.0; L.configureDriveline(dd);
            set(NcxValues::Gear, 3.0); set(NcxValues::SpeedKmh, 60.0); set(NcxValues::ThrottlePct, 0.0); settleD(200);
            set(NcxValues::ThrottlePct, 80.0);
            check(knocks(200) == 1, "I-8 shunt: a fast tip-in knocks once");
            for (int i = 1; i <= 50; ++i) { set(NcxValues::ThrottlePct, 80.0 - i * 1.6); settleD(10); }   // 500 ms creep down to 0
            check(knocks(100) == 0, "I-8 shunt: a slow lift does not knock");
            set(NcxValues::ThrottlePct, 80.0); settleD(200);
            set(NcxValues::ThrottlePct, 0.0);
            check(knocks(200) == 1, "I-8 shunt: a fast lift knocks once");
            set(NcxValues::Gear, 0.0); settleD(200); set(NcxValues::ThrottlePct, 80.0);
            check(knocks(200) == 0, "I-8 shunt: nothing in neutral");

            // Gearbox-shaped shift: the synchro clunk is longer and softer than
            // the dog knock; a dog shift under power gets a second knock ~60 ms
            // later; a dog shift off throttle does not.
            auto shiftBurst = [&](double gearbox, double throttle, double& peak, double& lenMs, int& count) {
                dd.gearbox = gearbox; L.configureDriveline(dd);
                set(NcxValues::Gear, 2.0); set(NcxValues::ThrottlePct, throttle); set(NcxValues::SpeedKmh, 60.0); settleD(300);
                set(NcxValues::Gear, 3.0);
                peak = 0.0; lenMs = 0.0; count = 0; bool in = false; int first = -1, last = -1;
                for (int i = 0; i < 150; ++i)
                {
                    haptics::driveLaws(L, st, v, dt); L.step(dt);
                    const double a = std::fabs(L.overlayFor(1));
                    peak = std::max(peak, a);
                    const bool on = a > 1e-9;
                    if (on && !in) { ++count; if (first < 0) first = i; }
                    if (on) last = i; if (count == 1 && on) lenMs = (last - first + 1) * dt * 1000.0;
                    in = on;
                } };
            double pS, lS, pD, lD; int cS, cD, cD0;
            double pD0, lD0;
            shiftBurst(0.0, 20.0, pS, lS, cS);
            shiftBurst(1.0, 20.0, pD0, lD0, cD0);
            shiftBurst(1.0, 100.0, pD, lD, cD);
            check(cS == 1 && cD0 == 1, "I-8 shift: one thunk per gear change (synchro, and dog off throttle)");
            check(pD0 > pS * 1.2 && lD0 < lS * 0.7, "I-8 shift: the dog knock is harder and shorter than the synchro clunk");
            check(cD == 2, "I-8 shift: a dog shift under power gets the engagement knock too");
        }

        // Road: per-corner suspension velocities replay the corners and the
        // roadNoise magnitude is ignored; without them roadNoise drives the
        // texture; a stale stream releases the replay.
        haptics::EffectParams rp; rp.ampPct = 100.0; rp.freqHz = 28.0; rp.routes[0] = { 0, 1.0 };
        L.configureFx(FxType::Road, rp); L.configureRoad({ 8.0, 2.0 });
        v.have[NcxValues::RoadNoise] = true; v.val[NcxValues::RoadNoise] = 100.0;
        setW(NcxValues::SuspVelFL, 400.0, 0.0, 0.0, 0.0); settle(50);
        check(L.roadReplaying() && L.roadWheelTravelMm(WheelFL) > 1.0 && L.roadWheelTravelMm(WheelRR) == 0.0,
              "I-8 road: suspension velocities replay the corners (FL moves, RR does not)");
        check(L.fxLevel(static_cast<int>(FxType::Road)) > 0.1 && L.fxLevel(static_cast<int>(FxType::Road)) <= 1.0,
              "I-8 road: tile level follows the biggest corner, roadNoise ignored");
        for (int w = 0; w < 4; ++w) v.have[NcxValues::SuspVelFL + w] = false;
        settle(400);
        check(!L.roadReplaying() && L.fxLevel(static_cast<int>(FxType::Road)) > 0.9,
              "I-8 road: without corners the roadNoise texture drives the tile");
        v.fresh = false; settle(300);
        check(L.fxLevel(static_cast<int>(FxType::Road)) < 0.01, "I-8 road: stale stream fades it");
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
            // Lateral slip tile, scrub only. The single skid channel at 100
            // drives full severity, where the staged carrier is 65% of the
            // set hz: set it so the carrier lands at freqHz.
            cfg.hapticsSlipLat = { 1.0, freqHz / 0.65, 0.0, 11.0, 7.0 };
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
