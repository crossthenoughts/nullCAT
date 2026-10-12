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
#include <cstring>
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

// Motion frame (belt demand zero) + a NULLCATX frame with the skid channel,
// rolling at 80 km/h (the slip laws are silent at a standstill).
static TelemetryData frame(double skidPct, bool fresh, double raw0 = 0.0)
{
    TelemetryData sd{};
    sd.valid        = true;
    sd.numPositions = 2;
    sd.positions[0] = raw0;
    sd.positions[1] = 32767.0;
    sd.packetType   = TelemetryPacketType::Motion;
    sd.numNcx       = 13;
    sd.ncx[static_cast<int>(NcxValues::SpeedKmh)] = 80.0;
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
        // As hard as the driver brakes: from nothing at 10% to full at full at
        // brake % (80 by default).
        v.val[NcxValues::BrakePct] = 45.0; settle(100);
        check(std::fabs(L.fxLevel(static_cast<int>(haptics::FxType::AbsPulse)) - 0.5) < 0.02, "I-7 ABS at 45% brake pulses at half strength");
        v.val[NcxValues::BrakePct] = 85.0; settle(100);
        check(L.fxLevel(static_cast<int>(haptics::FxType::AbsPulse)) > 0.99,  "I-7 ABS flag + firm brake drives the full pulse");
        {
            haptics::PulseParams always = L.pulseParams(haptics::FxType::AbsPulse); always.brakeFull = 10.0;
            L.configurePulse(haptics::FxType::AbsPulse, always);
            v.val[NcxValues::BrakePct] = 20.0; settle(100);
            check(L.fxLevel(static_cast<int>(haptics::FxType::AbsPulse)) > 0.99, "I-7 ABS full at brake % 10: full strength whatever the pedal");
            always.brakeFull = 80.0; L.configurePulse(haptics::FxType::AbsPulse, always);
            v.val[NcxValues::BrakePct] = 85.0; settle(100);
        }
        // Assetto Corsa blips the flag, about every other sample through a
        // stop: held over the gaps, the pulse runs on as one.
        {
            double lo = 1.0;
            for (int i = 0; i < 300; ++i)
            {
                v.val[NcxValues::AbsActive] = ((i / 8) % 2 == 0) ? 1.0 : 0.0;   // 16 ms on, 16 ms off
                haptics::driveLaws(L, st, v, dt); L.step(dt);
                if (i > 20) lo = std::min(lo, L.fxLevel(static_cast<int>(haptics::FxType::AbsPulse)));
            }
            check(lo > 0.99, "I-7 ABS: a flag blipping every other sample holds one steady pulse");
            v.val[NcxValues::AbsActive] = 0.0; settle(150);
            check(L.fxLevel(static_cast<int>(haptics::FxType::AbsPulse)) < 0.05, "I-7 ABS: the flag gone for good, it lets go");
            v.val[NcxValues::AbsActive] = 1.0; settle(10);
        }
        // The road speed slows the ABS cycle (slow x 0.5: half the 12 Hz set
        // at a standstill, 62.5% at 20 km/h, full from 80).
        check(std::fabs(L.absModel().rateHz() - 6.0) < 0.01, "I-7 ABS: at a standstill the cycle runs at half rate");
        v.val[NcxValues::SpeedKmh] = 20.0; settle(5);
        check(std::fabs(L.absModel().rateHz() - 12.0 * 0.625) < 0.01, "I-7 ABS: at 20 km/h it has slowed to 62.5%");
        // (Settled past the half second a speed jump no car can make, 20 to
        // 100 in a sample, leaves the speed untrusted.)
        v.val[NcxValues::SpeedKmh] = 100.0; settle(300);
        check(std::fabs(L.absModel().rateHz() - 12.0) < 0.01, "I-7 ABS: at speed, the rate set");
        v.val[NcxValues::SpeedKmh] = 0.0;

        // Single skid channel (fallback): 50 -> every wheel at 0.5 -> tile
        // level 0.5. Only while rolling: a stationary car's skid value is
        // noise (the dev bench read a held value at a standstill).
        v.val[NcxValues::Skid] = 50.0; settle(200);
        check(L.fxLevel(static_cast<int>(haptics::FxType::Skid)) < 0.01, "I-7 skid at a standstill is silent");
        v.val[NcxValues::SpeedKmh] = 60.0; settle(450);   // past the speed jump's hold
        check(std::fabs(L.fxLevel(static_cast<int>(haptics::FxType::Skid)) - 0.5) < 0.05, "I-7 skid 50 drives level 0.5");
        check(std::fabs(L.slipWheelLevel(haptics::FxType::Skid, WheelRR) - 0.5) < 0.05, "I-7 single skid channel reaches every wheel");
        // peak %: the channel value that counts as a full slide. 25 ->
        // skid 50 is clamped to full, skid 12.5 is halfway. Kerb the same.
        {
            haptics::EffectParams q = p; q.peakPct = 25.0;
            L.configureFx(haptics::FxType::Skid, q); L.configureFx(haptics::FxType::Kerb, q);
            settle(200);
            check(L.slipWheelLevel(haptics::FxType::Skid, WheelFL) > 0.95, "I-7 peak % 25: skid 50 is a full slide");
            v.val[NcxValues::Skid] = 12.5; settle(300);
            check(std::fabs(L.slipWheelLevel(haptics::FxType::Skid, WheelFL) - 0.5) < 0.05, "I-7 peak % 25: skid 12.5 is halfway");
            v.val[NcxValues::Curbs] = 25.0; settle(200);
            check(L.fxLevel(static_cast<int>(haptics::FxType::Kerb)) > 0.95, "I-7 peak % 25: curbs 25 is a full kerb");
            v.val[NcxValues::Curbs] = 0.0;
            L.configureFx(haptics::FxType::Skid, p); L.configureFx(haptics::FxType::Kerb, p);
        }
        v.val[NcxValues::Skid] = 0.0; settle(300);

        // Flag channel: TC.
        v.val[NcxValues::TcActive] = 1.0; settle(100);
        check(L.fxLevel(static_cast<int>(haptics::FxType::TcPulse)) > 0.9, "I-7 tcActive drives the TC pulse");
        // The TC flag flickers as traction control cuts in and out: held
        // over the gaps, one steady pulse; gone for good, it lets go.
        {
            double lo = 1.0;
            for (int i = 0; i < 300; ++i)
            {
                v.val[NcxValues::TcActive] = ((i / 8) % 2 == 0) ? 1.0 : 0.0;   // 16 ms on, 16 ms off
                haptics::driveLaws(L, st, v, dt); L.step(dt);
                if (i > 20) lo = std::min(lo, L.fxLevel(static_cast<int>(haptics::FxType::TcPulse)));
            }
            check(lo > 0.99, "I-7 TC: a flickering flag holds one steady pulse");
            v.val[NcxValues::TcActive] = 0.0; settle(150);
            check(L.fxLevel(static_cast<int>(haptics::FxType::TcPulse)) < 0.05, "I-7 TC: the flag gone for good, it lets go");
            // hold ms 0: the flag as the sim sends it.
            haptics::PulseParams raw = L.pulseParams(haptics::FxType::TcPulse); raw.holdMs = 0.0;
            L.configurePulse(haptics::FxType::TcPulse, raw);
            double lo0 = 1.0;
            for (int i = 0; i < 300; ++i)
            {
                v.val[NcxValues::TcActive] = ((i / 8) % 2 == 0) ? 1.0 : 0.0;
                haptics::driveLaws(L, st, v, dt); L.step(dt);
                if (i > 20) lo0 = std::min(lo0, L.fxLevel(static_cast<int>(haptics::FxType::TcPulse)));
            }
            check(lo0 < 0.5, "I-7 TC: hold ms 0 plays the flag as sent (flickering)");
            raw.holdMs = 150.0; L.configurePulse(haptics::FxType::TcPulse, raw);
            v.val[NcxValues::TcActive] = 1.0; settle(10);
        }

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
        // Lateral: onset 60% of peak, linear, no smoothing, the fast attack:
        // the mapping itself (the tile's onset, ease, smoothing and attack
        // have their own checks below).
        L.configureFx(FxType::Skid, p);   L.configureSlip(FxType::Skid,   { 1.0, 25.0, 1.0, 11.0, 7.0, 60.0, 0.0, 0.0, 8.0 });
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

        // Combined slip per wheel (protocol 1.4, Automobilista 2's slip
        // speed): the lateral source where no slip angle exists, from onset %
        // of peak % (here 60 of 100) to full at peak %. A combined slip
        // cannot be split, and a locked or spinning tyre is sliding too: it
        // plays here as well as on the longitudinal tile. Per-wheel angles
        // win over it when both arrive.
        for (int w = 0; w < 4; ++w) v.have[NcxValues::SlipAngleFL + w] = false;
        setW(NcxValues::WheelSlipFL, 100.0, 80.0, 50.0, 0.0); settle(100);
        check(lvl(FxType::Skid, WheelFL) > 0.95 && std::fabs(lvl(FxType::Skid, WheelFR) - 0.5) < 0.05
              && lvl(FxType::Skid, WheelRL) < 0.01 && lvl(FxType::Skid, WheelRR) < 0.01,
              "I-8 combined slip: 100 on FL = full, 80 on FR = halfway, 50 (below the onset) = nothing");
        setW(NcxValues::SlipRatioFL, -0.8, 0.0, 0.0, 0.0); settle(100);   // FL fully locked
        check(lvl(FxType::Lockup, WheelFL) > 0.95 && lvl(FxType::Skid, WheelFL) > 0.95,
              "I-8 combined slip: a locked FL plays as lock AND as a slide (a sliding tyre is sliding)");
        for (int w = 0; w < 4; ++w) v.have[NcxValues::SlipRatioFL + w] = false;
        setW(NcxValues::SlipAngleFL, 0.0, 0.0, 0.0, 0.0); settle(300);
        check(lvl(FxType::Skid, WheelFL) < 0.01, "I-8 combined slip: slip angles present (and small) win over it");
        for (int w = 0; w < 4; ++w) { v.have[NcxValues::SlipAngleFL + w] = false; v.have[NcxValues::WheelSlipFL + w] = false; }
        settle(300);

        // The Lateral tile's way in: onset %, ease, smooth hz, attack ms.
        {
            haptics::Layer T; haptics::LawsState ts;
            T.configureFx(FxType::Skid, p);
            NcxValues tv{}; tv.fresh = true;
            tv.have[NcxValues::SpeedKmh] = true; tv.val[NcxValues::SpeedKmh] = 100.0;
            auto setA = [&](double fl) { for (int w = 0; w < 4; ++w) { tv.have[NcxValues::SlipAngleFL + w] = true; tv.val[NcxValues::SlipAngleFL + w] = (w == 0) ? fl : 0.0; } };
            auto runT = [&](int n) { for (int i = 0; i < n; ++i) { haptics::driveLaws(T, ts, tv, dt); T.step(dt); } };
            auto lv = [&]() { return T.slipWheelLevel(FxType::Skid, WheelFL); };
            // Onset 40% of 7 deg = 2.8 deg; linear, halfway at 4.9 deg.
            T.configureSlip(FxType::Skid, { 1.0, 25.0, 1.0, 11.0, 7.0, 40.0, 0.0, 0.0, 8.0 });
            setA(4.9); runT(100);
            check(std::fabs(lv() - 0.5) < 0.05, "I-8 onset 40%: halfway from 2.8 to 7 deg is half a slide");
            setA(2.5); runT(150);
            check(lv() < 0.01, "I-8 onset 40%: below 2.8 deg nothing");
            // Ease 1: severity squared, the same angle comes in at a quarter.
            T.configureSlip(FxType::Skid, { 1.0, 25.0, 1.0, 11.0, 7.0, 40.0, 1.0, 0.0, 8.0 });
            setA(4.9); runT(100);
            check(std::fabs(lv() - 0.25) < 0.04, "I-8 ease 1: halfway in is a quarter (a light scrub first)");
            setA(7.0); runT(100);
            check(lv() > 0.95, "I-8 ease 1: still full at peak deg");
            // Smoothing: an angle flickering around the onset (a derived
            // angle at 60 Hz) reads as its peaks with a fast attack and a slow
            // release; smoothed it reads as its average.
            auto flicker = [&](double smoothHz) {
                T.clearAll(); haptics::LawsState fs; ts = fs;
                T.configureSlip(FxType::Skid, { 1.0, 25.0, 1.0, 11.0, 7.0, 40.0, 0.0, smoothHz, 8.0 });
                double sum = 0.0; int n = 0;
                for (int k = 0; k < 600; ++k)
                {
                    setA(((k / 4) % 2) ? 4.6 : 2.0);                    // mean 3.3 deg: linear severity 0.12
                    haptics::driveLaws(T, ts, tv, dt); T.step(dt);
                    if (k >= 300) { sum += lv(); ++n; }
                }
                return sum / n;
            };
            const double raw = flicker(0.0), smooth = flicker(8.0);
            check(raw > 0.3, "I-8 smooth 0: a flickering angle plays as its peaks (the chunky onset)");
            check(std::fabs(smooth - 0.12) < 0.05, "I-8 smooth 8 hz: it plays as its average");
            // Attack: 30 ms to build a full slide.
            T.configureSlip(FxType::Skid, { 1.0, 25.0, 1.0, 11.0, 7.0, 40.0, 0.0, 0.0, 30.0 });
            setA(0.0); runT(300);
            setA(7.0); runT(5);
            check(lv() > 0.2 && lv() < 0.5, "I-8 attack 30 ms: 10 ms in, still building");
            runT(15);
            check(lv() > 0.95, "I-8 attack 30 ms: full by 40 ms");
        }

        // Grip budget: a wheel with both a slip angle and a slip ratio slides
        // when the two together reach its limit. Onset 40% of 7 deg = 2.8 deg,
        // the ratio's limit 0.15; linear ease, no smoothing, fast attack.
        {
            auto budgetRun = [&](double budget, double angle, double ratio, double& lat, double& lon, double ease = 0.0) {
                haptics::Layer B; haptics::LawsState bs;
                B.configureFx(FxType::Skid, p); B.configureFx(FxType::Lockup, p);
                haptics::SlipParams lp{ 1.0, 25.0, 1.0, 11.0, 7.0, 40.0, ease, 0.0, 8.0 }; lp.budget = budget;
                B.configureSlip(FxType::Skid, lp); B.configureSlip(FxType::Lockup, { 1.0, 9.0, 1.0, 10.0, 0.8 });
                NcxValues bv{}; bv.fresh = true;
                bv.have[NcxValues::SpeedKmh] = true; bv.val[NcxValues::SpeedKmh] = 100.0;
                for (int w = 0; w < 4; ++w)
                {
                    bv.have[NcxValues::SlipAngleFL + w] = true; bv.val[NcxValues::SlipAngleFL + w] = (w == 0) ? angle : 0.0;
                    bv.have[NcxValues::SlipRatioFL + w] = true; bv.val[NcxValues::SlipRatioFL + w] = (w == 0) ? ratio : 0.0;
                }
                for (int i = 0; i < 100; ++i) { haptics::driveLaws(B, bs, bv, dt); B.step(dt); }
                lat = B.slipWheelLevel(FxType::Skid, WheelFL); lon = B.slipWheelLevel(FxType::Lockup, WheelFL);
            };
            double lat = 0.0, lon = 0.0;
            budgetRun(0.0, 2.8, -0.15, lat, lon);
            check(lat < 0.01 && lon < 0.01, "I-8 budget off: angle and braking each at its own limit, nothing");
            budgetRun(1.0, 2.8, -0.15, lat, lon);
            check(lat > 0.05 && lon > 0.05, "I-8 budget on: the two together are past the limit, a slide on both tiles (trail braking)");
            budgetRun(1.0, 4.9, 0.0, lat, lon);
            check(std::fabs(lat - 0.5) < 0.05 && lon < 0.01, "I-8 budget on: pure sideways is the Lateral tile as before");
            budgetRun(1.0, 0.0, -0.475, lat, lon);
            check(lat < 0.01 && std::fabs(lon - 0.5) < 0.05, "I-8 budget on: pure braking is the Longitudinal tile as before");
            // The Lateral tile's ease (2: cubed) shapes sideways only: a
            // slide straight along stays the Longitudinal tile's straight line.
            budgetRun(1.0, 0.0, -0.475, lat, lon, 2.0);
            check(std::fabs(lon - 0.5) < 0.05, "I-8 budget on, ease 2: pure braking is still the Longitudinal tile's 0.5");
            budgetRun(1.0, 4.9, 0.0, lat, lon, 2.0);
            check(std::fabs(lat - 0.125) < 0.03, "I-8 budget on, ease 2: pure sideways is cubed, as the Lateral tile");
        }

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
            // The tile's level (what the wave and the readout follow) must
            // fall back to 0 once the knock is over: on the bench the
            // Driveline wave stayed "live" after the first shunt.
            check(L.fxLevel(static_cast<int>(FxType::Driveline)) < 1e-6,
                  "I-8 shunt: the reported level is 0 again after the knock (no stuck wave)");
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
        // (Model 0, the suspension replay, roughness off: these are about the
        // sim's travel. The roughness, the tyre and chassis models: below and
        // in TestHaptics.)
        const auto susp = [](double full, double hp, double surface = 0.2, double kmh = 100.0)
        {
            haptics::RoadParams r; r.fullMm = full; r.hpHz = hp; r.surface = surface; r.surfaceKmh = kmh; r.model = 0;
            r.rough = 0.0;
            return r;
        };
        haptics::EffectParams rp; rp.ampPct = 100.0; rp.freqHz = 28.0; rp.routes[0] = { 0, 1.0 };
        L.configureFx(FxType::Road, rp); L.configureRoad(susp(8.0, 2.0));
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

        // Suspension TRAVEL (protocol 1.4, Assetto Corsa): the change in
        // travel enters the same high-passed replay; velocities win when
        // both arrive; a held sample adds nothing more.
        v.fresh = true; v.have[NcxValues::RoadNoise] = false;
        setW(NcxValues::SuspTravelFL, 0.0, 0.0, 0.0, 0.0); settle(20);
        setW(NcxValues::SuspTravelFL, 6.0, 0.0, 0.0, 0.0); settle(5);
        check(L.roadReplaying() && L.roadWheelTravelMm(WheelFL) > 4.0 && L.roadWheelTravelMm(WheelRR) == 0.0,
              "I-8 road: a 6 mm step in FL travel replays on FL");
        settle(400);
        check(L.roadWheelTravelMm(WheelFL) < 1.0, "I-8 road: travel held still is cut away by cut hz (only the bump remains)");
        setW(NcxValues::SuspVelFL, 0.0, 0.0, 0.0, 400.0); settle(50);
        check(L.roadWheelTravelMm(WheelRR) > 1.0, "I-8 road: velocities win over travel when both arrive");
        for (int w = 0; w < 4; ++w) { v.have[NcxValues::SuspVelFL + w] = false; v.have[NcxValues::SuspTravelFL + w] = false; }
        settle(400);

        // Surface: the tarmac grain under a rolling car, a texture rising
        // with road speed (surface x at surface km/h and above), on top of
        // either path; nothing at a standstill.
        v.have[NcxValues::SpeedKmh] = true; v.val[NcxValues::SpeedKmh] = 80.0;
        L.configureRoad(susp(8.0, 2.0, 0.1, 100.0)); settle(300);
        check(std::fabs(L.fxLevel(static_cast<int>(FxType::Road)) - 0.08) < 0.02,
              "I-8 surface: 0.1 x at 80 of 100 km/h = level 0.08 with nothing else driving the tile");
        L.configureRoad(susp(8.0, 2.0, 0.5, 50.0)); settle(300);
        check(std::fabs(L.fxLevel(static_cast<int>(FxType::Road)) - 0.5) < 0.03,
              "I-8 surface: 0.5 x, full from 50 km/h = level 0.5 at 80");
        setW(NcxValues::SuspVelFL, 400.0, 0.0, 0.0, 0.0); settle(50);
        check(L.roadReplaying() && L.fxLevel(static_cast<int>(FxType::Road)) >= 0.5,
              "I-8 surface: the grain stays under a replayed bump");
        for (int w = 0; w < 4; ++w) v.have[NcxValues::SuspVelFL + w] = false;
        v.val[NcxValues::SpeedKmh] = 0.0; settle(400);
        check(L.fxLevel(static_cast<int>(FxType::Road)) < 0.01, "I-8 surface: nothing at a standstill");
        L.configureRoad(susp(8.0, 2.0, 0.0, 100.0)); v.val[NcxValues::SpeedKmh] = 100.0; settle(300);
        check(L.fxLevel(static_cast<int>(FxType::Road)) < 0.01, "I-8 surface: surface x 0 = off");
    }

    // ---- I-9: the road's tyre and chassis models and the kerb, through the laws ----
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.002, PI = 3.14159265358979;
        haptics::Layer L; haptics::LawsState st;
        NcxValues v{}; v.fresh = true;
        auto settle = [&](int n) { for (int i = 0; i < n; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); } };
        auto peakOn = [&](haptics::Layer& X, haptics::LawsState& s, NcxValues& in, int axis, int n)
        { double pk = 0.0; for (int i = 0; i < n; ++i) { haptics::driveLaws(X, s, in, dt); X.step(dt); pk = std::max(pk, std::fabs(X.overlayFor(axis))); } return pk; };

        haptics::EffectParams rp; rp.ampPct = 100.0; rp.freqHz = 28.0; rp.jitter = 0.0; rp.routes[0] = { 0, 1.0 };
        L.configureFx(FxType::Road, rp);
        haptics::RoadParams tyre; tyre.model = 1; tyre.rough = 2.0; tyre.bodyMm = 1.0;
        L.configureRoad(tyre);

        // Tyre: speed alone runs the model on its own road; a car standing
        // still feels nothing (and the roadNoise texture stands down: the
        // model has what it needs).
        v.have[NcxValues::SpeedKmh] = true; v.val[NcxValues::SpeedKmh] = 100.0;
        v.have[NcxValues::RoadNoise] = true; v.val[NcxValues::RoadNoise] = 100.0;
        check(peakOn(L, st, v, 0, 1500) > 1.0, "I-9 road tyre: speed alone runs the model on its own road");
        v.val[NcxValues::SpeedKmh] = 0.0; settle(1500);
        check(peakOn(L, st, v, 0, 500) < 0.5, "I-9 road tyre: nothing under a car standing still, no texture either");

        // Chassis: without the body the texture plays; with it, the body's heave.
        haptics::RoadParams chassis; chassis.model = 2; L.configureRoad(chassis);
        v.val[NcxValues::SpeedKmh] = 80.0; settle(300);
        check(L.fxLevel(static_cast<int>(FxType::Road)) > 0.9, "I-9 road chassis: no body data, the roadNoise texture plays");
        v.have[NcxValues::AccHeave] = true; v.val[NcxValues::AccHeave] = 0.0; settle(400);
        check(L.fxLevel(static_cast<int>(FxType::Road)) < 0.05, "I-9 road chassis: with the body arriving, the texture stands down");
        double pk = 0.0;
        for (int i = 0; i < 1500; ++i)
        {
            v.val[NcxValues::AccHeave] = 3.0 * std::sin(2.0 * PI * 6.0 * i * dt);
            haptics::driveLaws(L, st, v, dt); L.step(dt); pk = std::max(pk, std::fabs(L.overlayFor(0)));
        }
        check(pk > 5.0, "I-9 road chassis: the body's heave plays");

        // Preview with no sim: the tyre model at 80 km/h on its own road.
        {
            haptics::Layer P; haptics::LawsState ps; NcxValues none{};
            P.configureFx(FxType::Road, rp); P.configureRoad(tyre);
            haptics::startPreview(ps, FxType::Road);
            check(peakOn(P, ps, none, 0, 900) > 1.0, "I-9 road preview: the tyre model runs at 80 km/h on its own road");
        }

        // Kerb from the surface under each tyre: FL on a kerb rumbles the FL
        // route, at speed / pitch, and not the RR route; it lets go as it leaves.
        haptics::Layer K; haptics::LawsState ks;
        haptics::EffectParams kp; kp.ampPct = 100.0; kp.jitter = 0.0;
        kp.routes[0] = { 0, 1.0, Part::FL }; kp.routes[1] = { 1, 1.0, Part::RR };
        K.configureFx(FxType::Kerb, kp); K.configureKerb(haptics::KerbParams{});
        NcxValues kv{}; kv.fresh = true;
        kv.have[NcxValues::SpeedKmh] = true; kv.val[NcxValues::SpeedKmh] = 72.0;   // 20 m/s: ribs at 80 Hz
        auto surf = [&](int fl, int fr, int rl, int rr)
        { const int s[4] = { fl, fr, rl, rr }; for (int w = 0; w < 4; ++w) { kv.have[NcxValues::SurfaceFL + w] = true; kv.val[NcxValues::SurfaceFL + w] = s[w]; } };
        surf(SurfKerb, SurfTarmac, SurfTarmac, SurfTarmac);
        std::vector<double> o; double pkRR = 0.0;
        for (int i = 0; i < 600; ++i)
        {
            haptics::driveLaws(K, ks, kv, dt); K.step(dt);
            if (i >= 100) o.push_back(K.overlayFor(0));
            pkRR = std::max(pkRR, std::fabs(K.overlayFor(1)));
        }
        int xr = 0; for (size_t i = 1; i < o.size(); ++i) if ((o[i] >= 0.0) != (o[i - 1] >= 0.0)) ++xr;
        check(K.kerbWheelLevel(WheelFL) > 0.99 && K.kerbWheelLevel(WheelRR) == 0.0, "I-9 kerb: the surface class puts FL on the kerb, not RR");
        check(std::fabs(xr / 2.0 / (o.size() * dt) - 80.0) < 5.0, "I-9 kerb: the FL route hums at speed / pitch");
        check(pkRR < 1e-9, "I-9 kerb: the RR route is silent");
        check(K.fxLevel(static_cast<int>(FxType::Kerb)) > 0.99, "I-9 kerb: the tile level shows the tyre on the kerb");
        surf(SurfTarmac, SurfTarmac, SurfTarmac, SurfTarmac);
        for (int i = 0; i < 50; ++i) { haptics::driveLaws(K, ks, kv, dt); K.step(dt); }
        check(K.kerbWheelLevel(WheelFL) == 0.0, "I-9 kerb: off the kerb, it lets go");

        // Kerb from the road heights (detect mm): the axle's usual
        // left-right (banking) is not a kerb; a tyre stepping up past it is.
        haptics::KerbParams kd; kd.detectMm = 20.0; K.configureKerb(kd);
        for (int w = 0; w < 4; ++w) kv.have[NcxValues::SurfaceFL + w] = false;
        auto heights = [&](double fl, double fr, double rl, double rr)
        { const double h[4] = { fl, fr, rl, rr }; for (int w = 0; w < 4; ++w) { kv.have[NcxValues::RoadHeightFL + w] = true; kv.val[NcxValues::RoadHeightFL + w] = h[w]; } };
        heights(30.0, 0.0, 30.0, 0.0);
        for (int i = 0; i < 500; ++i) { haptics::driveLaws(K, ks, kv, dt); K.step(dt); }
        check(K.kerbWheelLevel(WheelFL) == 0.0 && K.kerbWheelLevel(WheelFR) == 0.0, "I-9 kerb detect: banking alone is not a kerb");
        heights(60.0, 0.0, 30.0, 0.0);
        for (int i = 0; i < 50; ++i) { haptics::driveLaws(K, ks, kv, dt); K.step(dt); }
        check(K.kerbWheelLevel(WheelFL) > 0.99 && K.kerbWheelLevel(WheelFR) == 0.0 && K.kerbWheelLevel(WheelRL) == 0.0,
              "I-9 kerb detect: FL 30 mm above its usual is on a kerb");
        heights(30.0, 0.0, 30.0, 0.0);
        for (int i = 0; i < 50; ++i) { haptics::driveLaws(K, ks, kv, dt); K.step(dt); }
        heights(30.0, 0.0, 30.0, 25.0);
        for (int i = 0; i < 50; ++i) { haptics::driveLaws(K, ks, kv, dt); K.step(dt); }
        check(K.kerbWheelLevel(WheelRR) > 0.99 && K.kerbWheelLevel(WheelFL) == 0.0, "I-9 kerb detect: RR stepping up 25 mm is on a kerb");
        // A numbered line pads its kerb channel with 0 (Assetto Corsa sends
        // none): that 0 must not shut the detector out.
        kv.have[NcxValues::Curbs] = true; kv.val[NcxValues::Curbs] = 0.0;
        heights(30.0, 0.0, 30.0, 0.0);
        for (int i = 0; i < 50; ++i) { haptics::driveLaws(K, ks, kv, dt); K.step(dt); }
        heights(60.0, 0.0, 30.0, 0.0);
        for (int i = 0; i < 50; ++i) { haptics::driveLaws(K, ks, kv, dt); K.step(dt); }
        check(K.kerbWheelLevel(WheelFL) > 0.99, "I-9 kerb detect: a kerb channel stuck at 0 does not shut the detector out");
        kv.have[NcxValues::Curbs] = false;

        // Preview with no sim: the strip at 80 km/h, on 0.6 s in every 1 s.
        {
            haptics::Layer P; haptics::LawsState ps; NcxValues none{};
            P.configureFx(FxType::Kerb, kp); P.configureKerb(haptics::KerbParams{});
            haptics::startPreview(ps, FxType::Kerb);
            double fl = 0.0;
            for (int i = 0; i < 150; ++i) { haptics::driveLaws(P, ps, none, dt); P.step(dt); fl = std::max(fl, std::fabs(P.overlayFor(0))); }
            check(P.kerbWheelLevel(WheelFL) > 0.99 && fl > 5.0, "I-9 kerb preview: on a kerb at first, the FL route rumbling");
            for (int i = 0; i < 250; ++i) { haptics::driveLaws(P, ps, none, dt); P.step(dt); }
            check(P.kerbWheelLevel(WheelFL) == 0.0, "I-9 kerb preview: off it at 0.8 s");
            for (int i = 0; i < 200; ++i) { haptics::driveLaws(P, ps, none, dt); P.step(dt); }
            check(P.kerbWheelLevel(WheelFL) > 0.99, "I-9 kerb preview: back on at 1.2 s");
        }
    }

    // ---- I-10: the Surface tile through the laws ----
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.002;
        haptics::Layer L; haptics::LawsState st;
        haptics::EffectParams sp; sp.ampPct = 100.0; sp.routes[0] = { 0, 1.0, Part::FL }; sp.routes[1] = { 1, 1.0, Part::FR };
        L.configureFx(FxType::Surface, sp);
        NcxValues v{}; v.fresh = true;
        v.have[NcxValues::SpeedKmh] = true; v.val[NcxValues::SpeedKmh] = 72.0;   // 20 m/s
        auto surf = [&](int fl, int fr, int rl, int rr)
        { const int s[4] = { fl, fr, rl, rr }; for (int w = 0; w < 4; ++w) { v.have[NcxValues::SurfaceFL + w] = true; v.val[NcxValues::SurfaceFL + w] = s[w]; } };
        // FL on gravel, FR on tarmac: stones on FL's route, nothing on FR's.
        surf(SurfGravel, SurfTarmac, SurfTarmac, SurfTarmac);
        double pk0 = 0.0, pk1 = 0.0;
        for (int i = 0; i < 1500; ++i)
        {
            haptics::driveLaws(L, st, v, dt); L.step(dt);
            pk0 = std::max(pk0, std::fabs(L.overlayFor(0))); pk1 = std::max(pk1, std::fabs(L.overlayFor(1)));
        }
        check(L.surfaceModel().stonesStruck(WheelFL) > 50 && L.surfaceModel().stonesStruck(WheelFR) == 0,
              "I-10 surface: the gravel under FL strikes stones, the tarmac under FR none");
        check(pk0 > 1.0 && pk1 < 1e-9, "I-10 surface: FL's route plays them, FR's is silent");

        // Rain builds a water film (a minute of heavy rain: 63%); the sim's
        // own wetness wins over it; per tyre wins over that.
        surf(SurfTarmac, SurfTarmac, SurfTarmac, SurfTarmac);
        v.have[NcxValues::Rain] = true; v.val[NcxValues::Rain] = 1.0;
        for (int i = 0; i < 30000; ++i) haptics::driveLaws(L, st, v, dt);
        check(std::fabs(st.waterFilm - 0.632) < 0.03, "I-10 surface: a minute of heavy rain stands on the road (63%)");
        check(std::fabs(L.surfaceModel().wetness(WheelFL) - st.waterFilm) < 1e-9, "I-10 surface: with rain alone, the film is the wetness");
        v.have[NcxValues::Wet] = true; v.val[NcxValues::Wet] = 0.2;
        haptics::driveLaws(L, st, v, dt); L.step(dt);
        check(std::fabs(L.surfaceModel().wetness(WheelRR) - 0.2) < 1e-9, "I-10 surface: the sim's wetness wins over the rain");
        const double perW[4] = { 0.9, 0.0, 0.0, 0.5 };
        for (int w = 0; w < 4; ++w) { v.have[NcxValues::WetFL + w] = true; v.val[NcxValues::WetFL + w] = perW[w]; }
        haptics::driveLaws(L, st, v, dt); L.step(dt);
        check(std::fabs(L.surfaceModel().wetness(WheelFL) - 0.9) < 1e-9 && std::fabs(L.surfaceModel().wetness(WheelRR) - 0.5) < 1e-9,
              "I-10 surface: per tyre wins over the track's wetness");
        v.val[NcxValues::Rain] = 0.0;
        for (int w = 0; w < 4; ++w) v.have[NcxValues::WetFL + w] = false;
        v.have[NcxValues::Wet] = false;
        const double film = st.waterFilm;
        for (int i = 0; i < 30000; ++i) haptics::driveLaws(L, st, v, dt);
        check(st.waterFilm < film * 0.85 && st.waterFilm > film * 0.7, "I-10 surface: after the rain the road dries slowly");

        // A Test preview with no sim: gravel stones, then snow, then puddles.
        {
            haptics::Layer P; haptics::LawsState ps; NcxValues none{};
            P.configureFx(FxType::Surface, sp);
            haptics::startPreview(ps, FxType::Surface);
            for (int i = 0; i < 300; ++i) { haptics::driveLaws(P, ps, none, dt); P.step(dt); }
            check(P.surfaceModel().stonesStruck(WheelFL) > 10, "I-10 surface preview: gravel first, stones struck");
            double snow = 0.0;
            for (int i = 0; i < 400; ++i) { haptics::driveLaws(P, ps, none, dt); P.step(dt); if (i > 100 && i < 350) snow = std::max(snow, std::fabs(P.overlayFor(0))); }
            check(snow > 1.0, "I-10 surface preview: then snow crunching");
            double wet = 0.0;
            for (int i = 0; i < 300; ++i) { haptics::driveLaws(P, ps, none, dt); P.step(dt); wet = std::max(wet, P.surfaceModel().wetness(WheelFL)); }
            check(wet > 0.99, "I-10 surface preview: then a wet road");
        }
    }

    // ---- I-11: the Wheels tile through the laws ----
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.002;
        haptics::Layer L; haptics::LawsState st;
        haptics::EffectParams wp; wp.ampPct = 100.0; wp.routes[0] = { 0, 1.0, Part::FL }; wp.routes[1] = { 1, 1.0, Part::FR };
        L.configureFx(FxType::Wheels, wp);
        haptics::WheelsParams q; q.balance = 0.0; q.judder = 0.0; L.configureWheels(q);
        NcxValues v{}; v.fresh = true;
        v.have[NcxValues::SpeedKmh] = true; v.val[NcxValues::SpeedKmh] = 72.0;   // 20 m/s
        v.have[NcxValues::BrakePct] = true; v.val[NcxValues::BrakePct] = 100.0;
        auto ratios = [&](double fl, double fr, double rl, double rr)
        { const double r[4] = { fl, fr, rl, rr }; for (int w = 0; w < 4; ++w) { v.have[NcxValues::SlipRatioFL + w] = true; v.val[NcxValues::SlipRatioFL + w] = r[w]; } };
        // FL locked for a second: a flat ground into FL alone.
        ratios(-1.0, -0.1, -0.1, -0.1);
        for (int i = 0; i < 500; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
        check(std::fabs(L.wheelFlatMm(WheelFL) - 0.36) < 0.01 && L.wheelFlatMm(WheelFR) == 0.0,
              "I-11 wheels: a second locked at 72 km/h grinds a 0.36 mm flat into FL alone (from 0.1 s locked)");
        // Rolling on: FL's route thumps once a revolution, FR's is silent.
        ratios(0.0, 0.0, 0.0, 0.0); v.val[NcxValues::BrakePct] = 0.0;
        double pk0 = 0.0, pk1 = 0.0;
        for (int i = 0; i < 1500; ++i)
        {
            haptics::driveLaws(L, st, v, dt); L.step(dt);
            if (i > 500) { pk0 = std::max(pk0, std::fabs(L.overlayFor(0))); pk1 = std::max(pk1, std::fabs(L.overlayFor(1))); }
        }
        check(pk0 > 2.0 && pk1 < 1e-9, "I-11 wheels: the flat thumps on FL's route, FR's is silent");
        check(L.fxLevel(static_cast<int>(FxType::Wheels)) > 0.02, "I-11 wheels: the tile level shows it");
        // A minute's pause (the pits, a menu) keeps the flat; so does the
        // sim naming the car it was already in.
        v.fresh = false;
        for (int i = 0; i < 30000; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
        check(L.wheelFlatMm(WheelFL) > 0.3, "I-11 wheels: a minute's pause keeps the flat");
        v.fresh = true; std::strcpy(v.car, "Car A");
        for (int i = 0; i < 100; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
        check(L.wheelFlatMm(WheelFL) > 0.3, "I-11 wheels: the sim naming the car keeps the flat");
        // A different car: fresh tyres, and the per-car learning starts over.
        st.rollKnown[WheelFL] = true; st.loadRef = 1234.0;
        std::strcpy(v.car, "Car B");
        haptics::driveLaws(L, st, v, dt); L.step(dt);
        check(L.wheelFlatMm(WheelFL) == 0.0 && !st.rollKnown[WheelFL] && st.loadRef == 0.0,
              "I-11 wheels: a different car brings fresh tyres and forgets the last car's learning");
        // No identity at all: ten minutes quiet brings fresh tyres.
        ratios(-1.0, 0.0, 0.0, 0.0); v.val[NcxValues::BrakePct] = 100.0;
        for (int i = 0; i < 250; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
        check(L.wheelFlatMm(WheelFL) > 0.1, "I-11 wheels: a new lock-up grinds a new flat");
        v.fresh = false;
        for (int i = 0; i < 290000; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
        check(L.wheelFlatMm(WheelFL) > 0.1, "I-11 wheels: quiet for under ten minutes keeps it");
        for (int i = 0; i < 11000; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
        check(L.wheelFlatMm(WheelFL) == 0.0, "I-11 wheels: quiet for ten minutes, fresh tyres");

        // A Test preview with no sim: out of round, then a flat on the front
        // left, then hot discs under braking, at 80 km/h.
        {
            haptics::Layer P; haptics::LawsState ps; NcxValues none{};
            P.configureFx(FxType::Wheels, wp); P.configureWheels(haptics::WheelsParams{});
            haptics::startPreview(ps, FxType::Wheels);
            double run = 0.0, flat = 0.0, judder = 0.0;
            for (int i = 0; i < 1000; ++i)
            {
                haptics::driveLaws(P, ps, none, dt); P.step(dt);
                const double o = std::fabs(P.overlayFor(0));
                if (i > 100 && i < 340) run = std::max(run, o);
                else if (i > 450 && i < 690) flat = std::max(flat, o);
                else if (i > 750) judder = std::max(judder, o);
            }
            std::printf("  I-11 wheels preview peaks: balance %.2f, flat %.2f, judder %.2f\n", run, flat, judder);
            check(run > 0.5, "I-11 wheels preview: out of balance first");
            check(flat > 1.5 * run, "I-11 wheels preview: then a flat on the front left");
            check(judder > flat, "I-11 wheels preview: then hot discs under braking");
            check(P.wheelFlatMm(WheelFL) == 0.0, "I-11 wheels preview: the tyres' own state untouched");
        }
    }

    // ---- I-13: ground contact through the laws ----
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.002;
        haptics::Layer L; haptics::LawsState st;
        haptics::EffectParams lat; lat.ampPct = 100.0; lat.routes[0] = { 0, 1.0, Part::FL }; lat.routes[1] = { 1, 1.0, Part::FR };
        L.configureFx(FxType::Skid, lat);
        haptics::EffectParams kb; kb.ampPct = 100.0; kb.routes[0] = { 2, 1.0, Part::FL };
        L.configureFx(FxType::Kerb, kb); L.configureKerb(haptics::KerbParams{});
        haptics::EffectParams wh; wh.ampPct = 100.0; wh.routes[0] = { 3, 1.0, Part::RR };
        L.configureFx(FxType::Wheels, wh);
        haptics::WheelsParams wq; wq.balance = 0.0; wq.flat = 0.0; wq.judder = 0.0; L.configureWheels(wq);
        NcxValues v{}; v.fresh = true;
        v.have[NcxValues::SpeedKmh] = true; v.val[NcxValues::SpeedKmh] = 100.0;
        auto setW = [&](int group, double a, double b, double c, double d)
        { const double x[4] = { a, b, c, d }; for (int w = 0; w < 4; ++w) { v.have[group + w] = true; v.val[group + w] = x[w]; } };
        setW(NcxValues::SlipAngleFL, 9.0, 9.0, 0.0, 0.0);          // both fronts sliding
        setW(NcxValues::SurfaceFL, SurfKerb, SurfKerb, SurfTarmac, SurfTarmac);
        setW(NcxValues::LoadFL, 3000.0, 3000.0, 3000.0, 3000.0);
        auto run = [&](int n) { for (int i = 0; i < n; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); } };
        run(1000);
        check(L.slipWheelLevel(FxType::Skid, WheelFL) > 0.5 && L.kerbWheelLevel(WheelFL) > 0.99,
              "I-13 ground: FL on the ground slides and rumbles on the kerb");
        // FL unloaded (in the air): its slip and kerb stop, FR carries on.
        setW(NcxValues::LoadFL, 0.0, 3000.0, 3000.0, 3000.0);
        run(200);
        check(L.slipWheelLevel(FxType::Skid, WheelFL) < 0.01 && L.kerbWheelLevel(WheelFL) < 0.01,
              "I-13 ground: FL with no load is in the air: no slide, no kerb");
        check(L.slipWheelLevel(FxType::Skid, WheelFR) > 0.5 && L.kerbWheelLevel(WheelFR) > 0.99,
              "I-13 ground: FR on the ground carries on");
        // Hysteresis: a light tyre (5% of its usual load) stays where it was.
        setW(NcxValues::LoadFL, 150.0, 3000.0, 3000.0, 3000.0);
        run(100);
        check(st.air[WheelFL], "I-13 ground: 5% load after the air is still in the air (no flicker)");
        setW(NcxValues::LoadFL, 400.0, 3000.0, 3000.0, 3000.0);
        run(100);
        check(!st.air[WheelFL], "I-13 ground: 13% load is back on the ground");
        // The sim's own flag wins over the load (AMS2 sends no loads).
        setW(NcxValues::LoadFL, 3000.0, 3000.0, 3000.0, 3000.0);
        setW(NcxValues::OnGroundFL, 1.0, 0.0, 1.0, 1.0);
        run(200);
        check(L.slipWheelLevel(FxType::Skid, WheelFR) < 0.01 && L.slipWheelLevel(FxType::Skid, WheelFL) > 0.5,
              "I-13 ground: onGround* wins: FR flagged in the air goes quiet though it carries load");
        // A punctured tyre thumps on its route.
        setW(NcxValues::DeflatedFL, 0.0, 0.0, 0.0, 1.0);
        double pk = 0.0;
        for (int i = 0; i < 1500; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); if (i > 500) pk = std::max(pk, std::fabs(L.overlayFor(3))); }
        check(pk > 10.0, "I-13 ground: deflatedRR thumps the RR route (the Wheels tile's puncture)");
    }

    // ---- I-17: Automobilista 2 the way it really sends ----
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.0005;
        const double PI = 3.14159265358979;
        haptics::Layer L; haptics::LawsState st;
        haptics::EffectParams fx; fx.ampPct = 100.0; fx.routes[0] = { 0, 1.0, Part::FL };
        L.configureFx(FxType::Lockup, fx); L.configureFx(FxType::Skid, fx); L.configureFx(FxType::AbsPulse, fx);
        haptics::SlipParams lat = haptics::effectInfo(haptics::Effect::Skid).slipDefaults;
        lat.peak = 9.0; lat.onsetPct = 40.0; lat.ease = 0.0; lat.smoothHz = 3.0; lat.budget = 0.0;
        L.configureSlip(FxType::Skid, lat);
        // Longitudinal as a rig gets it (the registry's: edge 0.3 from 3 %);
        // a bare Layer's own defaults have no edge.
        L.configureSlip(FxType::Lockup, haptics::effectInfo(haptics::Effect::Lockup).slipDefaults);
        NcxValues v{}; v.fresh = true;
        auto setW = [&](int group, double a, double b, double c, double d)
        { const double x[4] = { a, b, c, d }; for (int w = 0; w < 4; ++w) { v.have[group + w] = true; v.val[group + w] = x[w]; } };
        auto run = [&](double secs) { const int n = static_cast<int>(secs / dt); for (int i = 0; i < n; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); } };
        // Rad/s for a 0.33 m wheel at kmh, NEGATIVE going forward (as AMS2 sends).
        auto wsAt = [](double kmh) { return -(kmh / 3.6) / 0.33; };
        v.have[NcxValues::BrakePct] = true; v.val[NcxValues::BrakePct] = 0.0;
        v.have[NcxValues::ThrottlePct] = true; v.val[NcxValues::ThrottlePct] = 0.0;
        v.have[NcxValues::SpeedKmh] = true; v.val[NcxValues::SpeedKmh] = 200.0;
        setW(NcxValues::WheelSpeedFL, wsAt(200), wsAt(200), wsAt(200), wsAt(200));
        // Before the rolling factor is known the along part is unknown: the
        // wheel's angle is too, not all sideways (a start's wheelspin read as
        // a full slide).
        const double spin = 55.556 * 0.2;   // 20 % wheelspin's worth of slip speed, all along
        setW(NcxValues::SlipSpeedFL, spin, spin, spin, spin);
        for (int i = 0; i < 100; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
        check(!st.rollKnown[WheelFL] && st.latRaw[WheelFL] == 0.0 && L.slipWheelLevel(FxType::Skid, WheelFL) < 1e-6,
              "I-17: no rolling factor yet: the slip speed plays nothing (unknown, not a slide)");
        // A straight at 200 km/h: 5 deg's worth of slip speed (55.6 m/s x
        // sin 5 = 4.84 m/s) on every wheel, the tyres' own creep and toe:
        // past the 3.6 deg onset, so it would slide without the baseline.
        const double base = 55.556 * std::sin(5.0 * PI / 180.0);
        setW(NcxValues::SlipSpeedFL, base, base, base, base);
        run(15.0);   // the baseline learns over seconds (a lap of straights)
        check(st.rollKnown[WheelFL] && st.rollKnown[WheelRR], "I-17: negative wheel speeds (Automobilista 2) still teach the rolling factor");
        std::printf("  I-17 straight-line baseline FL %.2f deg after %.1f s of straights\n", st.latBase[WheelFL], st.latBaseSec);
        check(std::fabs(st.latBase[WheelFL] - 5.0) < 0.3 && st.latBaseSec > haptics::laws_k::kLatBaseLearnSec, "I-17: the straight-line slip (5 deg) is learned as the baseline");
        check(L.slipWheelLevel(FxType::Skid, WheelFL) < 0.02, "I-17: ...so a 200 km/h straight plays no slide (it played 0.2 as a combined slip)");
        // A long coasting corner at 100 km/h (the fronts 2 % apart), the
        // fronts at 13 deg of slip speed: 13 - 5 = 8 deg as played. Five
        // seconds: the rolling factor must not learn the corner's split (it
        // did, and the corner then counted as straight for the baseline).
        v.val[NcxValues::SpeedKmh] = 100.0;
        setW(NcxValues::WheelSpeedFL, wsAt(99), wsAt(101), wsAt(99.5), wsAt(100.5));
        const double side = 27.778 * std::sin(13.0 * PI / 180.0);
        setW(NcxValues::SlipSpeedFL, side, side, base / 2.0, base / 2.0);
        const double baseBefore = st.latBase[WheelFL];
        run(5.0);
        std::printf("  I-17 corner: FL angle as played %.2f deg, lateral %.2f\n", st.latAngle[WheelFL], L.slipWheelLevel(FxType::Skid, WheelFL));
        check(std::fabs(st.latAngle[WheelFL] - 8.0) < 0.4, "I-17: in a corner the slip speed plays as its sideways angle less the baseline");
        check(L.slipWheelLevel(FxType::Skid, WheelFL) > 0.5, "I-17: ...a real slide on the front");
        check(std::fabs(st.latBase[WheelFL] - baseBefore) < 1e-9, "I-17: the baseline does not learn in a long corner");
        // Braking in a straight line: the fronts at -10 % slip ratio. Their
        // slip speed is that along slip (2.78 m/s) and the baseline's
        // sideways part; taken apart, no slide.
        v.val[NcxValues::BrakePct] = 60.0;
        setW(NcxValues::WheelSpeedFL, wsAt(90), wsAt(90), wsAt(100), wsAt(100));
        {
            const double along = 0.1 * 27.778, sideB = 27.778 * std::sin(5.0 * PI / 180.0);
            const double sf = std::sqrt(along * along + sideB * sideB);
            setW(NcxValues::SlipSpeedFL, sf, sf, sideB, sideB);
        }
        run(1.0);
        check(L.slipWheelLevel(FxType::Skid, WheelFL) < 0.05, "I-17: braking straight, the along slip is taken out: no lateral slide");
        // The edge: the fronts at 10 % braking slip are working (3 % to 15 %).
        check(L.slipWheelEdge(WheelFL) > 0.3 && L.slipWheelEdge(WheelRL) < 0.01, "I-17: the edge grain: the braked fronts at 10 % work, the rolling rears do not");
        // Driving out: the rears at +5 % (a traction control holding them);
        // edge from 3 % feels it, edge from 8 % does not.
        v.val[NcxValues::BrakePct] = 0.0; v.val[NcxValues::ThrottlePct] = 100.0;
        setW(NcxValues::WheelSpeedFL, wsAt(100), wsAt(100), wsAt(105), wsAt(105));
        setW(NcxValues::SlipSpeedFL, base / 2.0, base / 2.0, 1.4, 1.4);
        run(0.5);
        const double edge5 = L.slipWheelEdge(WheelRL);
        haptics::SlipParams lon = L.slipParams(FxType::Lockup); lon.screechPct = 8.0; L.configureSlip(FxType::Lockup, lon);
        run(0.5);
        std::printf("  I-17 edge on the rears at +5 %%: %.2f from 3 %%, %.2f from 8 %%\n", edge5, L.slipWheelEdge(WheelRL));
        check(edge5 > 0.05 && L.slipWheelLevel(FxType::Lockup, WheelRL) < 0.01, "I-17: a traction control's 5 % is felt as the edge, not as wheelspin");
        check(L.slipWheelEdge(WheelRL) < 0.01, "I-17: edge from % moves where it starts");
        lon.screechPct = 3.0; L.configureSlip(FxType::Lockup, lon);
        v.val[NcxValues::ThrottlePct] = 0.0;
        // A hairpin coasting: from the wheel speeds the outer wheels read
        // +5 %, the inner -5 % (the corner's geometry, no slip): the axle's
        // mean is 0, so no edge.
        setW(NcxValues::WheelSpeedFL, wsAt(95), wsAt(105), wsAt(95), wsAt(105));
        run(0.5);
        check(L.slipWheelEdge(WheelFL) < 0.01 && L.slipWheelEdge(WheelFR) < 0.01 && L.slipWheelEdge(WheelRR) < 0.01,
              "I-17: a hairpin's wheel speed split (+-5 %) is no edge");

        // ABS from the wheels: no flag (Automobilista 2 rarely sends it); the
        // front wheels hunting near lock under 80 % brake, the wheel speeds
        // at 60 Hz. maxStep: the FL pulse's largest step from one cycle to
        // the next after the first 0.2 s (a pulse jumping to each dump
        // stepped by up to 2).
        haptics::PulseParams pp; pp.brakeFull = 80.0;
        double maxStep = 0.0;
        auto hunt = [&](double secs, auto ratioAt) {
            const int n = static_cast<int>(secs / dt); double held = 0.0, last = L.absModel().channel(WheelFL); int lastK = -1;
            maxStep = 0.0;
            for (int i = 0; i < n; ++i)
            {
                const double t = i * dt; const int k = static_cast<int>(t * 60.0);
                if (k != lastK) { lastK = k; held = ratioAt(t); }
                const double wf = wsAt(100.0 * (1.0 + held));
                setW(NcxValues::WheelSpeedFL, wf, wf, wsAt(100), wsAt(100));
                haptics::driveLaws(L, st, v, dt); L.step(dt);
                const double now = L.absModel().channel(WheelFL);
                if (t > 0.2) maxStep = std::max(maxStep, std::fabs(now - last));
                last = now;
            } };
        const auto abs10   = [&](double t) { return -0.07 + 0.05 * std::cos(2.0 * PI * 10.0 * t); };   // -0.02 .. -0.12 at 10 Hz
        // 8 Hz: a quick deep dip, back up over about 4 samples, then a spell
        // rolling near the road speed before the pressure builds again.
        const auto slow8   = [&](double t) { const double u = t * 8.0 - std::floor(t * 8.0);
                                             return (u < 0.15) ? -0.02 - 0.33 * u / 0.15 : (u < 0.7) ? -0.35 + 0.33 * (u - 0.15) / 0.55 : -0.02; };
        const auto hover15 = [&](double t) { return -0.09 + 0.02 * std::cos(2.0 * PI * 15.0 * t); };   // threshold braking, the road shaking it
        const auto locked  = [&](double) { return -1.0; };
        const auto absOff  = [&]() { return st.absWheelHoldSec == 0.0 && L.fxLevel(static_cast<int>(FxType::AbsPulse)) < 0.01; };
        v.have[NcxValues::AbsActive] = true; v.val[NcxValues::AbsActive] = 0.0;
        v.val[NcxValues::BrakePct] = 80.0;
        setW(NcxValues::SlipSpeedFL, 0.0, 0.0, 0.0, 0.0);
        pp.fromWheels = 0.0; L.configurePulse(FxType::AbsPulse, pp);
        hunt(1.0, abs10);
        check(absOff(), "I-17: from wheels off, no flag: no ABS");
        pp.fromWheels = 1.0; L.configurePulse(FxType::AbsPulse, pp);
        hunt(1.0, abs10);
        std::printf("  I-17 ABS from the wheels: %.1f Hz detected, level %.2f, largest pulse step %.3f\n",
                    st.absWheelHz, L.fxLevel(static_cast<int>(FxType::AbsPulse)), maxStep);
        check(st.absWheelHoldSec > 0.0 && std::fabs(st.absWheelHz - 10.0) < 1.5, "I-17: from wheels on: the wheels hunting at 10 Hz are ABS working, its rate found");
        check(L.fxLevel(static_cast<int>(FxType::AbsPulse)) > 0.99, "I-17: ...and the ABS pulse plays (80 % brake = full)");
        check(L.absModel().synced(WheelFL) && !L.absModel().synced(WheelRL), "I-17: the front corners' pulse is timed to their wheels (the rolling rears are not)");
        check(maxStep < 0.1, "I-17: ...pulled into step with them, never jumping to each dump");
        st.absWheelHz = 0.0;
        hunt(1.0, slow8);
        std::printf("  I-17 ABS, a deep dip back up over 8 samples: %.1f Hz detected\n", st.absWheelHz);
        check(st.absWheelHoldSec > 0.0 && std::fabs(st.absWheelHz - 8.0) < 1.0, "I-17: a wheel spinning back up over several samples is one dump a cycle (the rate stays true)");
        hunt(1.0, hover15);
        check(absOff(), "I-17: threshold braking, the wheel hovering near a lock, is no ABS");
        setW(NcxValues::OnGroundFL, 0.0, 0.0, 1.0, 1.0);
        hunt(1.0, abs10);
        check(absOff(), "I-17: the fronts in the air (a kerb hop) are no ABS");
        for (int w = 0; w < 4; ++w) v.have[NcxValues::OnGroundFL + w] = false;
        hunt(1.0, locked);
        check(absOff(), "I-17: a wheel locked and staying locked (no ABS) is no ABS");
        hunt(0.5, [&](double) { return 0.0; });
        check(absOff() && !L.absModel().synced(WheelFL), "I-17: ...nor its release; ABS off frees the corners");
        v.val[NcxValues::BrakePct] = 0.0;
    }

    // ---- I-14: the fronts' screech and pulse through the laws ----
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.002;
        haptics::Layer L; haptics::LawsState st;
        haptics::EffectParams lat; lat.ampPct = 100.0; lat.routes[0] = { 0, 1.0, Part::FL };
        L.configureFx(FxType::Skid, lat);
        haptics::SlipParams sp = haptics::effectInfo(haptics::Effect::Skid).slipDefaults;
        sp.peak = 8.0; sp.onsetPct = 40.0; sp.ease = 1.0; sp.smoothHz = 0.0; sp.attackMs = 2.0; sp.budget = 0.0;
        sp.screech = 1.0; sp.screechPct = 20.0; sp.pulse = 1.0;
        L.configureSlip(FxType::Skid, sp);
        NcxValues v{}; v.fresh = true;
        v.have[NcxValues::SpeedKmh] = true; v.val[NcxValues::SpeedKmh] = 100.0;
        auto ang = [&](double fl, double rl)
        { const double a[4] = { fl, 0.0, rl, 0.0 }; for (int w = 0; w < 4; ++w) { v.have[NcxValues::SlipAngleFL + w] = true; v.val[NcxValues::SlipAngleFL + w] = a[w]; } };
        auto run = [&](int n) { for (int i = 0; i < n; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); } };
        // 2.5 deg: past screech % (1.6) but short of the onset (3.2): the
        // tyre squeals, nothing slides yet.
        ang(2.5, 0.0); run(200);
        check(L.slipWheelScreech(WheelFL) > 0.02 && L.slipWheelLevel(FxType::Skid, WheelFL) < 1e-6 && L.slipWheelPulse(WheelFL) < 1e-6,
              "I-14 screech: the front squeals before it slides");
        // 7 deg: sliding; the pulse follows the slide, the screech near full.
        ang(7.0, 7.0); run(200);
        check(std::fabs(L.slipWheelPulse(WheelFL) - L.slipWheelLevel(FxType::Skid, WheelFL)) < 0.05 && L.slipWheelPulse(WheelFL) > 0.4,
              "I-14 pulse: as strong as the front's slide");
        check(L.slipWheelScreech(WheelFL) > 0.85, "I-14 screech: near full near the peak");
        check(L.slipWheelScreech(WheelRL) == 0.0 && L.slipWheelPulse(WheelRL) == 0.0, "I-14: the rears neither screech nor pulse (their slide is the slide)");
    }

    // ---- I-16: a Test preview is no session peak ----
    {
        MotionController mc;
        mc.configure(makeBeltConfig(50.0, 1.0));
        mc.setEmergencyStop(false);
        tensionBelt(mc);
        run(mc, frame(0.0, true), 400, 1);           // ONLINE, the skid channel at 0
        mc.requestHapticPeakReset();
        MotionCommand t; t.type = MotionCommand::Type::HapticsTest; t.intVal = static_cast<int>(haptics::Effect::Skid);
        mc.enqueueCommand(t);
        run(mc, frame(0.0, true), 8000, 1);          // the preview, and well past it
        const int fx = static_cast<int>(haptics::FxType::Skid);
        MotionStatus ms = mc.getMotionStatus();
        check(ms.hapticsInPk[fx] < 0.01 && ms.hapticsWheelPk[0][WheelFL] < 0.01,
              "I-16: a Test preview leaves the session peaks alone");
        run(mc, frame(40.0, true), 600, 1);          // the sim's own slide
        ms = mc.getMotionStatus();
        check(ms.hapticsInPk[fx] > 0.3, "I-16: the sim's own slide still sets the peak");
    }

    // ---- I-15: what the sim sends that cannot be true ----
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.0005;
        haptics::Layer L; haptics::LawsState st;
        haptics::EffectParams fx; fx.ampPct = 100.0; fx.routes[0] = { 0, 1.0, Part::FL };
        L.configureFx(FxType::Lockup, fx); L.configureFx(FxType::Skid, fx); L.configureFx(FxType::Wheels, fx);
        haptics::WheelsParams wq; wq.balance = 0.0; wq.judder = 0.0; L.configureWheels(wq);   // flat 1
        haptics::SlipParams sp = haptics::effectInfo(haptics::Effect::Skid).slipDefaults;
        sp.onsetPct = 0.0; sp.ease = 0.0; sp.smoothHz = 1.0; sp.budget = 0.0;
        L.configureSlip(FxType::Skid, sp);
        NcxValues v{}; v.fresh = true;
        auto setW = [&](int group, double x) { for (int w = 0; w < 4; ++w) { v.have[group + w] = true; v.val[group + w] = x; } };
        v.have[NcxValues::SpeedKmh] = true; v.val[NcxValues::SpeedKmh] = 120.0;
        v.have[NcxValues::BrakePct] = true; v.val[NcxValues::BrakePct] = 0.0;
        v.have[NcxValues::ThrottlePct] = true; v.val[NcxValues::ThrottlePct] = 0.0;
        setW(NcxValues::WheelSpeedFL, 104.0);   // rad/s at 120 km/h (Automobilista 2: wheel speeds, no ratio)
        setW(NcxValues::WheelSlipFL, 10.0);     // a little combined slip
        // Samples 1/60 s apart, held between.
        auto run = [&](double secs) { const int n = static_cast<int>(secs / dt); for (int i = 0; i < n; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); } };
        run(2.0);   // cruising: the rolling factor learned
        const double k0 = st.rollK[WheelFL];
        check(st.rollKnown[WheelFL] && L.slipWheelLevel(FxType::Lockup, WheelFL) < 1e-6, "I-15: cruising learns the rolling factor, nothing locked");
        // A jump to the pits: one sample at 579 km/h, then back.
        v.val[NcxValues::SpeedKmh] = 579.0; run(1.0 / 60.0);
        double lockPk = L.slipWheelLevel(FxType::Lockup, WheelFL);
        v.val[NcxValues::SpeedKmh] = 120.0;
        for (int i = 0; i < 400; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); lockPk = std::max(lockPk, L.slipWheelLevel(FxType::Lockup, WheelFL)); }
        check(st.glitchSec > 0.0, "I-15: a road speed jump no car can make leaves the speed untrusted a moment");
        check(lockPk < 1e-6 && L.wheelFlatMm(WheelFL) == 0.0 && L.wheelFlatMm(WheelRR) == 0.0,
              "I-15: the jump is no lock-up on any wheel and grinds no flat");
        check(std::fabs(st.rollK[WheelFL] - k0) < 1e-9, "I-15: nor does it teach the rolling factor");
        run(1.0);
        check(st.glitchSec == 0.0, "I-15: trusted again half a second later");
        // A real lock-up still grinds a flat.
        setW(NcxValues::WheelSpeedFL, 0.0); v.val[NcxValues::BrakePct] = 100.0;
        run(0.5);
        check(L.slipWheelLevel(FxType::Lockup, WheelFL) > 0.5 && L.wheelFlatMm(WheelFL) > 0.1, "I-15: a real lock-up still locks and grinds");
        setW(NcxValues::WheelSpeedFL, 104.0); v.val[NcxValues::BrakePct] = 0.0;
        run(1.0);
        // A spin with the pedals off: the car still doing 80 km/h, every
        // wheel nearly stopped. It must not teach the rolling factor (it
        // did, and back on the straight the rears read as spinning for
        // seconds).
        v.val[NcxValues::SpeedKmh] = 80.0; setW(NcxValues::WheelSpeedFL, 2.0);
        run(1.0);
        check(std::fabs(st.rollK[WheelRL] / k0 - 1.0) < 0.03, "I-15: a spin with the pedals off teaches the rolling factor nothing");
        v.val[NcxValues::SpeedKmh] = 120.0; setW(NcxValues::WheelSpeedFL, 104.0); v.val[NcxValues::ThrottlePct] = 100.0;
        // From 0.75 s on (the spin's own lock-up faded; the speed jump's
        // hold over): it used to read as spinning for many seconds.
        double after = 0.0;
        for (int i = 0; i < 6000; ++i)
        {
            haptics::driveLaws(L, st, v, dt); L.step(dt);
            if (i >= 1500) for (int w = 0; w < 4; ++w) after = std::max(after, L.slipWheelLevel(FxType::Lockup, w));
        }
        check(after < 0.01, "I-15: ...so back on the straight at full throttle nothing reads as spinning");
        v.val[NcxValues::ThrottlePct] = 0.0;
        // Spun round and rolling backwards: the wheels turn backwards with
        // the car. Rolling, not locked: no slip, no flat.
        const double flatBefore = L.wheelFlatMm(WheelRL);
        setW(NcxValues::WheelSpeedFL, -70.0); v.val[NcxValues::SpeedKmh] = 80.0;
        double back = 0.0;
        for (int i = 0; i < 2000; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); for (int w = 0; w < 4; ++w) back = std::max(back, L.slipWheelLevel(FxType::Lockup, w)); }
        check(back < 0.01 && L.wheelFlatMm(WheelRL) <= flatBefore + 1e-9, "I-15: rolling backwards is rolling, not four locked wheels");
        setW(NcxValues::WheelSpeedFL, 104.0); v.val[NcxValues::SpeedKmh] = 120.0;
        run(1.0);
        // A garbage gear (150, seen once) is dropped: no shift thunks.
        haptics::EffectParams gp; gp.ampPct = 100.0; gp.freqHz = 20.0; gp.durMs = 60.0; gp.routes[0] = { 2, 1.0, Part::All };
        L.configure(haptics::EventType::GearShift, gp);
        v.have[NcxValues::Gear] = true; v.val[NcxValues::Gear] = 4.0; run(0.2);
        const uint64_t shifts0 = L.fireCount(haptics::EventType::GearShift);
        v.val[NcxValues::Gear] = 150.0; run(1.0 / 60.0);
        v.val[NcxValues::Gear] = 4.0;   run(0.2);
        check(L.fireCount(haptics::EventType::GearShift) == shifts0, "I-15: a garbage gear (150) fires no shift");
        // One combined-slip sample of three billion (Automobilista 2's slip
        // speed, now and then): capped, it cannot hold the slide on.
        v.val[NcxValues::WheelSlipFL] = 3.0e9; run(1.0 / 60.0);
        v.val[NcxValues::WheelSlipFL] = 10.0; run(0.3);
        std::printf("  I-15 lateral 0.3 s after a 3e9 slip sample: %.3f\n", L.slipWheelLevel(FxType::Skid, WheelFL));
        check(L.slipWheelLevel(FxType::Skid, WheelFL) < 0.25, "I-15: a single absurd slip sample does not hold a slide on");
        // A body acceleration of 290 g (one sample) knocks nothing.
        haptics::EffectParams ip; ip.ampPct = 100.0; ip.freqHz = 40.0; ip.durMs = 50.0; ip.routes[0] = { 1, 1.0, Part::FL };
        L.configureFx(FxType::Impacts, ip);
        v.have[NcxValues::AccHeave] = true; v.val[NcxValues::AccHeave] = 9.8; run(0.2);
        v.val[NcxValues::AccHeave] = 2880.0; run(1.0 / 60.0);
        v.val[NcxValues::AccHeave] = 9.8;
        double ipk = 0.0;
        for (int i = 0; i < 400; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); ipk = std::max(ipk, std::fabs(L.overlayFor(1))); }
        check(ipk < 1e-9, "I-15: a 290 g body acceleration sample knocks nothing");
        // The body's angles jumping 20 deg in one sample (a reset) are a
        // glitch: speed and body untrusted a moment, then back.
        v.have[NcxValues::PitchDeg] = v.have[NcxValues::RollDeg] = true;
        v.val[NcxValues::PitchDeg] = 1.0; v.val[NcxValues::RollDeg] = 0.5; run(0.6);
        check(st.glitchSec == 0.0, "I-15: steady angles are no glitch");
        v.val[NcxValues::PitchDeg] = 21.0; run(1.0 / 60.0);
        check(st.glitchSec > 0.0, "I-15: pitch jumping 20 deg in a sample is a glitch");
        v.val[NcxValues::PitchDeg] = 1.0; run(1.0);
        check(st.glitchSec == 0.0, "I-15: ...trusted again after the hold");
        v.have[NcxValues::PitchDeg] = v.have[NcxValues::RollDeg] = false;
    }

    // ---- I-12: the Impacts tile through the laws ----
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.002;
        haptics::Layer L; haptics::LawsState st;
        haptics::EffectParams ip; ip.ampPct = 100.0; ip.freqHz = 40.0; ip.durMs = 50.0;
        ip.routes[0] = { 0, 1.0, Part::FL }; ip.routes[1] = { 1, 1.0, Part::FR };
        L.configureFx(FxType::Impacts, ip); L.configureImpacts(haptics::ImpactParams{});
        NcxValues v{}; v.fresh = true;
        auto vel = [&](double fl) { for (int w = 0; w < 4; ++w) { v.have[NcxValues::SuspVelFL + w] = true; v.val[NcxValues::SuspVelFL + w] = (w == WheelFL) ? fl : 0.0; } };
        vel(0.0);
        for (int i = 0; i < 100; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
        vel(2000.0);   // FL compresses at 2 m/s from one sample to the next: a kerb strike
        double pk0 = 0.0, pk1 = 0.0;
        for (int i = 0; i < 100; ++i)
        {
            haptics::driveLaws(L, st, v, dt); L.step(dt);
            pk0 = std::max(pk0, std::fabs(L.overlayFor(0))); pk1 = std::max(pk1, std::fabs(L.overlayFor(1)));
        }
        check(L.impactModel().knocks(WheelFL) == 1 && pk0 > 50.0 && pk1 < 1e-9,
              "I-12 impacts: a sudden 2 m/s compression on FL knocks FL's route, FR's is silent");
        // Travel instead (Assetto Corsa), and the body's heave.
        haptics::Layer T; haptics::LawsState ts; T.configureFx(FxType::Impacts, ip); T.configureImpacts(haptics::ImpactParams{});
        NcxValues tv{}; tv.fresh = true;
        for (int i = 0; i < 600; ++i)
        {
            const double held = std::floor(i * dt * 60.0) / 60.0;
            for (int w = 0; w < 4; ++w) { tv.have[NcxValues::SuspTravelFL + w] = true; tv.val[NcxValues::SuspTravelFL + w] = (w == WheelRR && held >= 0.5) ? 30.0 : 0.0; }
            tv.have[NcxValues::AccHeave] = true; tv.val[NcxValues::AccHeave] = (held >= 1.0) ? 40.0 : 0.0;
            haptics::driveLaws(T, ts, tv, dt); T.step(dt);
        }
        check(T.impactModel().knocks(WheelRR) >= 1 && T.impactModel().knocks(WheelFR) == 1,
              "I-12 impacts: RR's travel jumping 30 mm in a sample knocks RR; a 4 g heave jump knocks every corner");

        // A Test preview with no sim: FL, FR, RL (down), RR, then the whole car.
        {
            haptics::Layer P; haptics::LawsState ps; NcxValues none{};
            P.configureFx(FxType::Impacts, ip); P.configureImpacts(haptics::ImpactParams{});
            haptics::startPreview(ps, FxType::Impacts);
            double flPk = 0.0;
            for (int i = 0; i < 1000; ++i) { haptics::driveLaws(P, ps, none, dt); P.step(dt); if (i < 150) flPk = std::max(flPk, std::fabs(P.overlayFor(0))); }
            check(flPk > 50.0, "I-12 impacts preview: FL knocks first");
            check(P.impactModel().knocks(WheelFL) == 2 && P.impactModel().knocks(WheelRL) == 2 && P.impactModel().knocks(WheelRR) == 2,
                  "I-12 impacts preview: each corner once, then the whole car");
        }
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

        // P-5..P-7: the dev bench fault. Motion off (no positions on the
        // wire), sim channels live, haptics on the posts: unparked, the hold
        // fed the overlay back in every cycle and the posts ran at 400 mm/s
        // with instant reversals; a park and re-home later the stale guard
        // stepped them into Er87. Every cycle's command step must stay
        // inside maxVelocity, a reversal must pass through a stop (the
        // acceleration limit), and the hold must not wander.
        struct StepWatch
        {
            double prev = 0.0, prevStep = 0.0; bool have = false, haveStep = false;
            double maxStep = 0.0, maxRevStep = 0.0;
            void see(double p)
            {
                if (have)
                {
                    const double s = p - prev;
                    maxStep = std::max(maxStep, std::fabs(s));
                    if (haveStep && s * prevStep < 0.0) maxRevStep = std::max(maxRevStep, std::fabs(s));
                    prevStep = s; haveStep = true;
                }
                prev = p; have = true;
            }
        };
        const double kDt = 1.0 / 500.0;
        const double kVmaxStep = 200.0 * kDt * 1.001 + 1e-9;            // maxVelocity 200 mm/s, one cycle
        const double kRevStep  = 2000.0 * kDt * kDt * 1.001 + 0.011;     // a reversal: from a stop, Amax 2000, + a count
        auto watchRun = [](MotionController& mc, MockA6Drive& mock, const TelemetryData& sd, int cycles, StepWatch& w, Span* sp)
        {
            A6Drive* drives[1] = { &mock };
            for (int i = 0; i < cycles; ++i)
            {
                MotionOutput out{}; mc.process(sd, out, drives, 1);
                w.see(out.positions[0]);
                if (sp) { sp->lo = std::min(sp->lo, out.positions[0]); sp->hi = std::max(sp->hi, out.positions[0]); }
            }
        };
        TelemetryData noMotion = frame(100.0, true);
        noMotion.numPositions = 0;                                        // motion off, channels on
        TelemetryData noMotionQuiet = frame(0.0, true);
        noMotionQuiet.numPositions = 0;
        {
            MotionController mc; MockA6Drive mock;
            mc.configure(makePosConfig(100.0, 2.0, 12.0, 3.0, 0.4));
            check(bringOnline(mc, mock), "P-5 position axis reaches ONLINE with motion off");
            StepWatch w; Span settle, held;
            watchRun(mc, mock, noMotion, 500, w, &settle);
            watchRun(mc, mock, noMotion, 2500, w, &held);
            // 0.4 x 2000 / (2 pi 12)^2 = 0.14 mm: the post vibrates +-0.14 mm
            // about the hold, and the hold itself does not move.
            check(held.width() < 0.4, "P-5 motion off: the overlay vibrates about the hold, it does not build up");
            check(std::fabs(0.5 * (held.lo + held.hi) - 0.5 * (settle.lo + settle.hi)) < 0.1, "P-5 motion off: the hold does not wander");
            check(w.maxStep <= kVmaxStep, "P-5 motion off: no command step past maxVelocity");
            check(w.maxRevStep <= kRevStep, "P-5 motion off: no instant reversal");
        }
        {
            MotionController mc; MockA6Drive mock;
            mc.configure(makePosConfig(100.0, 2.0, 12.0, 3.0, 0.4));
            check(bringOnline(mc, mock), "P-6 position axis reaches ONLINE");
            StepWatch w;
            watchRun(mc, mock, noMotion, 1500, w, nullptr);
            mc.startPark();
            watchRun(mc, mock, noMotion, 600, w, nullptr);
            check(mc.getAxisState(0) == AxisMotionState::PARKED, "P-6 parked with haptics live");
            { A6Drive* drives[1] = { &mock }; mc.startUnpark(drives, 1); }
            // Watched from BLENDING on (the unpark itself is a timed move to
            // centre, 0.1 s here, faster than maxVelocity by design).
            StepWatch back;
            for (int i = 0; i < 3000 && mc.getAxisState(0) != AxisMotionState::ONLINE; ++i)
            {
                A6Drive* drives[1] = { &mock }; MotionOutput out{};
                mc.process(noMotion, out, drives, 1);
                if (mc.getAxisState(0) == AxisMotionState::BLENDING || mc.getAxisState(0) == AxisMotionState::ONLINE)
                    back.see(out.positions[0]);
            }
            check(mc.getAxisState(0) == AxisMotionState::ONLINE, "P-6 back ONLINE after a park, haptics still live");
            watchRun(mc, mock, noMotion, 1000, back, nullptr);
            check(back.maxStep <= kVmaxStep, "P-6 the haptics guard starts fresh: no step entering ONLINE again");
            check(back.maxRevStep <= kRevStep, "P-6 ...and no instant reversal");
        }
        {
            MotionController mc; MockA6Drive mock;
            mc.configure(makePosConfig(100.0, 2.0, 5.0, 3.0, 0.4));
            check(bringOnline(mc, mock), "P-7 position axis reaches ONLINE");
            StepWatch w;
            for (int k = 0; k < 20; ++k)                                 // the overlay stopping and starting
            {
                watchRun(mc, mock, frame(100.0, true, 32767.0), 37, w, nullptr);
                watchRun(mc, mock, frame(0.0, true, 32767.0), 23, w, nullptr);
            }
            watchRun(mc, mock, noMotionQuiet, 400, w, nullptr);
            check(w.maxStep <= kVmaxStep, "P-7 the overlay going to zero hands back without a step");
            check(w.maxRevStep <= kRevStep, "P-7 ...and without an instant reversal");
        }
    }

    std::printf("TestHapticsIntegration: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
