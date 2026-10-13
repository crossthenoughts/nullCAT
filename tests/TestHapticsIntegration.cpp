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
#include <vector>

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

        // The master mutes every tile; unmuting one under it plays that
        // tile alone (a solo), the master still reading muted; the master's
        // UNMUTE clears every tile.
        const uint64_t skidBit = 1ull << static_cast<int>(haptics::Effect::Skid);
        m.intVal = 1; mc.enqueueCommand(m);
        run(mc, frame(100.0, true), 50, 1);
        MotionStatus ms = mc.getMotionStatus();
        check(ms.hapticsMuted && (ms.hapticsMuteMask & skidBit) && (ms.hapticsMuteMask & (1ull << static_cast<int>(haptics::Effect::Engine))),
              "I-5 master mute: every tile reads muted");
        MotionCommand f; f.type = MotionCommand::Type::HapticsMuteFx; f.intVal = static_cast<int>(haptics::Effect::Skid);   // unmute
        mc.enqueueCommand(f);
        const Span solo = run(mc, frame(100.0, true), 200, 100);
        ms = mc.getMotionStatus();
        check(solo.width() > 5.0 && ms.hapticsMuted && !(ms.hapticsMuteMask & skidBit),
              "I-5 solo: one tile unmuted under the master plays, the master still muted");
        f.intVal = static_cast<int>(haptics::Effect::Skid) | 0x100; mc.enqueueCommand(f);
        const Span off = run(mc, frame(100.0, true), 200, 100);
        check(off.width() < 0.5, "I-5 tile mute: that tile silent again");
        m.intVal = 0; mc.enqueueCommand(m);
        const Span all = run(mc, frame(100.0, true), 200, 100);
        ms = mc.getMotionStatus();
        check(all.width() > 5.0 && !ms.hapticsMuted && ms.hapticsMuteMask == 0, "I-5 master unmute: every tile back");
    }

    // ---- I-5b: with the loop stopped the idle clock steps the layer
    // (shakers): tile mutes reach it too (the command queue is drained by
    // the RT loop only) ----
    {
        MotionController mc;
        mc.configure(makeBeltConfig(50.0, 1.0));
        const int fx = static_cast<int>(haptics::FxType::Skid);
        for (int i = 0; i < 300; ++i) mc.hapticsIdleTick(frame(100.0, true));
        const double live = mc.getMotionStatus().hapticsFxLevel[fx];
        MotionCommand f; f.type = MotionCommand::Type::HapticsMuteFx; f.intVal = static_cast<int>(haptics::Effect::Skid) | 0x100;
        mc.enqueueCommand(f);
        for (int i = 0; i < 5; ++i) mc.hapticsIdleTick(frame(100.0, true));
        const double muted = mc.getMotionStatus().hapticsFxLevel[fx];
        check(live > 0.5 && muted == 0.0, "I-5b loop stopped: a tile mute reaches the layer on the idle clock (its felt level 0)");
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

        // ABS flag with no brake: nothing. With brake: pulses. (The pulse's
        // own behaviour from a standstill up, so off < 5 km/h is unticked
        // here; I-17 covers it.)
        { haptics::PulseParams q = L.pulseParams(haptics::FxType::AbsPulse); q.lowOff = 0.0; L.configurePulse(haptics::FxType::AbsPulse, q); }
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

        // Flag channel: TC (on the throttle: there is nothing to cut without).
        const bool hadThrTc = v.have[NcxValues::ThrottlePct]; const double thrTc = v.val[NcxValues::ThrottlePct];
        v.have[NcxValues::ThrottlePct] = true; v.val[NcxValues::ThrottlePct] = 60.0;
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
            // Off the throttle there is nothing for a traction control to
            // cut: the flag (and its hold) is ignored with the pedal up
            // (Automobilista 2's ran on after a lift).
            const bool hadThr = v.have[NcxValues::ThrottlePct]; const double thr0 = v.val[NcxValues::ThrottlePct];
            raw.holdMs = 500.0; L.configurePulse(haptics::FxType::TcPulse, raw);   // a long hold: the lift must cut it short
            v.have[NcxValues::ThrottlePct] = true; v.val[NcxValues::ThrottlePct] = 0.0; settle(100);   // 0.2 s
            check(L.fxLevel(static_cast<int>(haptics::FxType::TcPulse)) < 0.05, "I-7 TC: the flag on with the throttle shut plays no TC (its hold cut short)");
            raw.holdMs = 150.0; L.configurePulse(haptics::FxType::TcPulse, raw);
            v.val[NcxValues::ThrottlePct] = 60.0; settle(100);
            check(L.fxLevel(static_cast<int>(haptics::FxType::TcPulse)) > 0.9, "I-7 TC: ...on the throttle it does");
            v.have[NcxValues::ThrottlePct] = hadThr; v.val[NcxValues::ThrottlePct] = thr0; settle(10);
        }
        v.have[NcxValues::ThrottlePct] = hadThrTc; v.val[NcxValues::ThrottlePct] = thrTc;

        // Gear edge: a change fires ONE transient (once the new gear has
        // held 45 ms); a steady gear fires none; the first sighting after
        // staleness never fires.
        v.val[NcxValues::Gear] = 3.0; settle(40);
        const uint64_t n0 = L.fireCount();
        v.val[NcxValues::Gear] = 4.0; settle(30);
        const uint64_t n1 = L.fireCount();
        settle(100);
        const uint64_t n2 = L.fireCount();
        check(n1 == n0 + 1, "I-7 gear change fires one gear-shift transient");
        check(n2 == n1,     "I-7 steady gear fires nothing more");
        v.fresh = false; settle(10); v.fresh = true; v.val[NcxValues::Gear] = 2.0; settle(60);
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

        // Load weighting only takes away: the loaded tyre of each axle
        // counts as 1, the lighter one by its share of it (weighting the
        // loaded tyre up played a sim that sends loads 1.4x stronger).
        setW(NcxValues::SlipAngleFL, 5.6, 5.6, 0.0, 0.0);
        setW(NcxValues::LoadFL, 6000.0, 1000.0, 2500.0, 2500.0);   // the front axle: FR at a sixth of FL
        settle(100);
        check(std::fabs(lvl(FxType::Skid, WheelFL) - 0.5) < 0.05, "I-8 load: the loaded FL as without loads (0.5), never weighted up");
        check(std::fabs(lvl(FxType::Skid, WheelFR) - 0.083) < 0.03, "I-8 load: the unloaded FR at 0.5 x 1/6");
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
            auto budgetRun = [&](double budget, double angle, double ratio, double& lat, double& lon, double ease = 0.0, double onsetPct = 40.0) {
                haptics::Layer B; haptics::LawsState bs;
                B.configureFx(FxType::Skid, p); B.configureFx(FxType::Lockup, p);
                haptics::SlipParams lp{ 1.0, 25.0, 1.0, 11.0, 7.0, onsetPct, ease, 0.0, 8.0 }; lp.budget = budget;
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
            // Onset 0 (the budget's angle limit a twentieth of peak): an
            // ordinary 3 deg of cornering took the whole budget and a lock
            // under it went nearly silent. Never less than each tile alone.
            double latAlone = 0.0, lonAlone = 0.0, lat0 = 0.0, lon0 = 0.0;
            budgetRun(0.0, 3.0, -0.475, latAlone, lonAlone, 2.0, 0.0);
            budgetRun(1.0, 3.0, -0.475, lat0, lon0, 2.0, 0.0);
            std::printf("  I-8 budget, onset 0: tiles alone lat %.2f lon %.2f, with the budget lat %.2f lon %.2f\n", latAlone, lonAlone, lat0, lon0);
            check(lonAlone > 0.45 && lon0 >= lonAlone - 0.01 && lat0 >= latAlone - 0.01,
                  "I-8 budget on, onset 0: a lock in a corner plays at least as the Longitudinal tile alone");
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
            // The sim's own max rpm seeds the learned redline, once per car:
            // a truck's 3,700 no longer waits for its limiter.
            // (Once it has held for half a second: a sender's may lag the
            // car's name.)
            std::strncpy(v.car, "Truck", NCY_STR_LEN - 1);
            v.val[NcxValues::Rpm] = 900.0;
            v.have[NcxValues::MaxRpm] = true; v.val[NcxValues::MaxRpm] = 3700.0;
            settleE(100);
            check(L.learnedMaxRpm() > 5000.0, "I-8 engine: the sim's max rpm is not taken at once");
            settleE(200);
            check(std::fabs(L.learnedMaxRpm() - 3700.0) < 1e-9, "I-8 engine: ...once it has held, it seeds the redline");
            v.val[NcxValues::Rpm] = 3800.0; settleE(1000);
            check(std::fabs(L.learnedMaxRpm() - 3800.0) < 1.0, "I-8 engine: ...the peak hold still refines it");
            v.val[NcxValues::Rpm] = 900.0; settleE(50);
            v.val[NcxValues::MaxRpm] = 5000.0; settleE(500);
            check(std::fabs(L.learnedMaxRpm() - 3800.0) < 1.0, "I-8 engine: ...once per car (a later max rpm does not pull it back)");
            // A new car whose max rpm arrives a moment after its name: the
            // last car's is not taken.
            std::strncpy(v.car, "F2004", NCY_STR_LEN - 1);
            v.val[NcxValues::MaxRpm] = 5000.0; settleE(50);
            v.val[NcxValues::MaxRpm] = 18800.0; settleE(300);
            check(std::fabs(L.learnedMaxRpm() - 18800.0) < 1e-9, "I-8 engine: a new car seeds from its own max rpm, not the last car's lagging one");
            // Never below what the car has already shown.
            std::strncpy(v.car, "GT3", NCY_STR_LEN - 1);
            v.have[NcxValues::MaxRpm] = false; v.val[NcxValues::Rpm] = 9500.0; settleE(300);
            v.have[NcxValues::MaxRpm] = true; v.val[NcxValues::MaxRpm] = 9000.0; settleE(300);
            check(std::fabs(L.learnedMaxRpm() - 9500.0) < 1.0, "I-8 engine: the seed never lowers the redline below the revs already seen");
            v.have[NcxValues::MaxRpm] = false; v.car[0] = '\0';
        }
        // The idle rock follows the square root of heavy x balance: a smooth
        // V12 (a 250 GTO) still idles with a felt shake, against a small
        // four (a Mini). It had a third of the four's; now over half, and
        // still less.
        {
            auto idleRock = [&](double cyl, double litres, double layout)
            {
                haptics::Layer L; haptics::LawsState st;
                for (int t = 0; t < NcxValues::TokenCount; ++t) v.have[t] = false;
                haptics::EffectParams ep; ep.ampPct = 60.0; ep.freqHz = 30.0; ep.routes[0] = { 0, 1.0 };
                haptics::EngineParams eg; eg.cylinders = cyl; eg.litres = litres; eg.layout = layout; eg.maxRpm = 7500.0;
                eg.thump = 0.0; eg.buzz = 0.0; eg.inertia = 0.0; eg.liftoff = 0.0;
                L.configureFx(FxType::RpmVibe, ep); L.configureEngine(eg);
                v.have[NcxValues::Rpm] = true; v.val[NcxValues::Rpm] = 800.0;
                v.have[NcxValues::ThrottlePct] = true; v.val[NcxValues::ThrottlePct] = 0.0;
                double pk = 0.0;
                for (int i = 0; i < 2000; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); if (i > 1000) pk = std::max(pk, std::fabs(L.overlayFor(0))); }
                return pk;
            };
            const double v12 = idleRock(12.0, 3.0, 1.0), i4 = idleRock(4.0, 1.3, 0.0);
            std::printf("  I-8 idle rock: V12 3.0 L %.2f, inline 4 1.3 L %.2f (%.0f %%)\n", v12, i4, 100.0 * v12 / i4);
            check(v12 > 0.5 * i4 && v12 < i4, "I-8 engine: a smooth V12 idles with over half a small four's rock, and still less");
            for (int t = 0; t < NcxValues::TokenCount; ++t) v.have[t] = false;
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
            // A shift's quick dip at speed (an auto-clutch, a heel-and-toe):
            // 0.15 s in the band is no slip to judder over; held, it is.
            set(NcxValues::ThrottlePct, 100.0); set(NcxValues::ClutchPct, 0.0); settleD(200);
            double dipPk = 0.0;
            set(NcxValues::ClutchPct, 50.0);
            for (int i = 0; i < 75; ++i) { settleD(1); dipPk = std::max(dipPk, judder()); }   // 0.15 s
            set(NcxValues::ClutchPct, 0.0); settleD(200);
            check(dipPk < 0.02, "I-8 driveline: a quick clutch dip at speed (a shift) does not judder");
            set(NcxValues::ClutchPct, 50.0); settleD(300);
            check(judder() > 0.25, "I-8 driveline: the clutch held in the band at speed, slipping, judders");

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
            haptics::DrivelineParams dd; dd.whine = 1.0; dd.whinePitch = 0.0; dd.shunt = 1.0; dd.shuntHz = 40.0; dd.gearbox = 0.0;
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

            // Whine pitch against the engine buzz (the tile's choice). With
            // the buzz it sits at 70 to 100% of it, so the top gears beat
            // against it (the old model, a slow pulsing on the belt); below,
            // octave down and above stay 20% or more off it in every gear,
            // and a pitch the loop cannot carry drops an octave.
            {
                set(NcxValues::Rpm, 6000.0); set(NcxValues::ThrottlePct, 100.0); set(NcxValues::ClutchPct, 0.0);
                const double buzz = haptics::EngineModel::buzzHz(eg, 6000.0 / 60.0, 7000.0, dt);
                double nearest[4] = { 9.0, 9.0, 9.0, 9.0 }; bool carried = true;
                for (int pch = 0; pch < 4; ++pch)
                {
                    dd.whinePitch = pch; L.configureDriveline(dd);
                    for (int g = 1; g <= 6; ++g)
                    {
                        set(NcxValues::Gear, g); settleD(20);
                        nearest[pch] = std::min(nearest[pch], std::fabs(L.drivelineWhineHz() / buzz - 1.0));
                        carried = carried && L.drivelineWhineHz() > 1.0 && L.drivelineWhineHz() <= (1.0 / dt) / 8.0 + 1e-9;
                    }
                }
                std::printf("  I-8 whine pitch: nearest to the %.1f Hz buzz with %.2f below %.2f octave %.2f above %.2f\n",
                            buzz, nearest[0], nearest[1], nearest[2], nearest[3]);
                check(nearest[0] < 0.01, "I-8 whine pitch: with the buzz, 6th sits on it (the old model)");
                check(nearest[1] >= 0.199 && nearest[2] >= 0.199 && nearest[3] >= 0.199,
                      "I-8 whine pitch: below, octave down and above stay 20%+ off the buzz in every gear");
                check(carried, "I-8 whine pitch: never past what the loop can carry");
                // The octave is the gear's, chosen at the redline: an rpm
                // sweep never steps the pitch down (picked from the revs now,
                // above the buzz fell an octave mid-rev on this 500 Hz loop).
                bool rising = true;
                for (int pch = 0; pch < 4; ++pch)
                {
                    dd.whinePitch = pch; L.configureDriveline(dd);
                    for (int g = 1; g <= 6; ++g)
                    {
                        // Back to 1000 rpm and held there first: 7000 to 1000
                        // in one sample is a junk frame to the rpm guard.
                        set(NcxValues::Gear, g); set(NcxValues::Rpm, 1000.0); settleD(300);
                        double last = L.drivelineWhineHz();
                        for (double rpm = 1250.0; rpm <= 7000.0; rpm += 250.0)
                        {
                            set(NcxValues::Rpm, rpm); settleD(2);
                            rising = rising && L.drivelineWhineHz() >= last - 1e-9;
                            last = L.drivelineWhineHz();
                        }
                    }
                }
                check(rising, "I-8 whine pitch: an rpm sweep never steps the pitch down (every choice and gear)");
                // On the Pi's 2 kHz loop, above sits above the buzz in every gear.
                const double dt2 = 0.0005;
                dd.whinePitch = 3.0; L.configureDriveline(dd);
                double lowest = 9.0;
                for (int g = 1; g <= 6; ++g)
                {
                    set(NcxValues::Gear, g); set(NcxValues::Rpm, 6500.0);
                    for (int i = 0; i < 4; ++i) { haptics::driveLaws(L, st, v, dt2); L.step(dt2); }
                    lowest = std::min(lowest, L.drivelineWhineHz() / haptics::EngineModel::buzzHz(eg, 6500.0 / 60.0, 7000.0, dt2));
                }
                check(lowest >= 1.199, "I-8 whine pitch: above the buzz sits above it on a 2 kHz loop");
                dd.whinePitch = 0.0; L.configureDriveline(dd);
                set(NcxValues::Rpm, 3500.0); set(NcxValues::Gear, 0.0); settleD(300);
            }

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

        // Kerb from the road heights (detect mm): a kerb is a JUMP in an
        // axle's left-right height difference (its edge, its ribs). The
        // road's banking, and its camber changing over seconds (Imola: 20 to
        // 55 mm, which read as a kerb most of a lap against a slowly learned
        // usual difference), are not. Sim samples at 60 Hz, the laws at 500.
        haptics::KerbParams kd; kd.detectMm = 20.0; K.configureKerb(kd);
        for (int w = 0; w < 4; ++w) kv.have[NcxValues::SurfaceFL + w] = false;
        auto heights = [&](double fl, double fr, double rl, double rr)
        { const double h[4] = { fl, fr, rl, rr }; for (int w = 0; w < 4; ++w) { kv.have[NcxValues::RoadHeightFL + w] = true; kv.val[NcxValues::RoadHeightFL + w] = h[w]; } };
        // Drive secs with the heights a function of time, held per 60 Hz
        // sample; the largest kerb level seen per wheel after `from` s.
        double kerbPk[4] = {};
        auto ride = [&](double secs, double from, auto at) {
            for (double& p : kerbPk) p = 0.0;
            const int n = static_cast<int>(secs / dt); int lastK = -1;
            for (int i = 0; i < n; ++i)
            {
                const double t = i * dt; const int k = static_cast<int>(t * 60.0);
                if (k != lastK) { lastK = k; at(k / 60.0); }
                haptics::driveLaws(K, ks, kv, dt); K.step(dt);
                if (t >= from) for (int w = 0; w < 4; ++w) kerbPk[w] = std::max(kerbPk[w], K.kerbWheelLevel(w));
            } };
        ride(1.0, 0.0, [&](double) { heights(30.0, 0.0, 30.0, 0.0); });
        check(kerbPk[WheelFL] == 0.0 && kerbPk[WheelFR] == 0.0, "I-9 kerb detect: banking alone is not a kerb");
        ride(2.0, 0.0, [&](double t) { heights(30.0 + 25.0 * t, 0.0, 30.0 + 25.0 * t, 0.0); });   // 50 mm of camber in 2 s, both axles
        check(kerbPk[WheelFL] == 0.0 && kerbPk[WheelFR] == 0.0 && kerbPk[WheelRL] == 0.0, "I-9 kerb detect: the camber changing over seconds is no kerb");
        // A ribbed kerb under FL: 20 mm ribs every other sample for 0.6 s.
        ride(0.6, 0.05, [&](double t) { const int k = static_cast<int>(t * 60.0 + 0.5); heights((k % 2) ? 100.0 : 80.0, 0.0, 80.0, 0.0); });
        check(kerbPk[WheelFL] > 0.99 && kerbPk[WheelFR] == 0.0 && kerbPk[WheelRL] == 0.0, "I-9 kerb detect: ribs under FL put FL on a kerb, not FR or the rears");
        check(K.kerbWheelLevel(WheelFL) > 0.99, "I-9 kerb detect: ...and keep it there while the ribs go on");
        ride(1.0, 0.6, [&](double) { heights(80.0, 0.0, 80.0, 0.0); });
        check(kerbPk[WheelFL] == 0.0, "I-9 kerb detect: off the ribs it lets go within half a second");
        // Ribs under FR straight after the camber has moved (a side taken
        // from a slowly learned camber lagged it and put the kerb on FL).
        ride(2.0, 0.0, [&](double t) { heights(80.0 + 25.0 * t, 0.0, 80.0, 0.0); });
        ride(0.6, 0.05, [&](double t) { const int k = static_cast<int>(t * 60.0 + 0.5); heights(130.0, (k % 2) ? 20.0 : 0.0, 80.0, 0.0); });
        check(kerbPk[WheelFR] > 0.99 && kerbPk[WheelFL] == 0.0, "I-9 kerb detect: ribs under FR after a camber change are FR's");
        ride(1.0, 0.0, [&](double) { heights(130.0, 0.0, 80.0, 0.0); });
        // A flat raised kerb under FL: up 30 mm for a second, then off. The
        // drop off rumbles FL again, not FR.
        ride(0.1, 0.0, [&](double) { heights(160.0, 0.0, 80.0, 0.0); });
        check(kerbPk[WheelFL] > 0.99 && kerbPk[WheelFR] == 0.0, "I-9 kerb detect: up onto a flat kerb, FL");
        ride(1.0, 0.0, [&](double) { heights(160.0, 0.0, 80.0, 0.0); });
        ride(0.1, 0.0, [&](double) { heights(130.0, 0.0, 80.0, 0.0); });
        check(kerbPk[WheelFL] > 0.99 && kerbPk[WheelFR] == 0.0, "I-9 kerb detect: ...and off it, FL again (the same kerb)");
        ride(1.0, 0.0, [&](double) { heights(130.0, 0.0, 80.0, 0.0); });
        ride(2.0, 0.0, [&](double t) { heights(130.0 - 25.0 * t, 0.0, 80.0, 0.0); });   // camber back, slowly
        check(kerbPk[WheelFL] == 0.0 && kerbPk[WheelFR] == 0.0, "I-9 kerb detect: the camber easing back is no kerb");
        // A chicane: a kerb under FL, up and off, then one under FR.
        ride(0.3, 0.0, [&](double) { heights(110.0, 0.0, 80.0, 0.0); });
        check(kerbPk[WheelFL] > 0.99 && kerbPk[WheelFR] == 0.0, "I-9 kerb detect: a chicane: FL's kerb first");
        ride(0.3, 0.0, [&](double) { heights(80.0, 0.0, 80.0, 0.0); });
        ride(0.6, 0.0, [&](double) { heights(80.0, 0.0, 80.0, 0.0); });
        ride(0.3, 0.0, [&](double) { heights(80.0, 30.0, 80.0, 0.0); });
        check(kerbPk[WheelFR] > 0.99 && kerbPk[WheelFL] == 0.0, "I-9 kerb detect: ...then FR's is FR's");
        ride(0.3, 0.0, [&](double) { heights(80.0, 0.0, 80.0, 0.0); });
        ride(1.0, 0.0, [&](double) { heights(80.0, 0.0, 80.0, 0.0); });
        // A single step up under RR (a raised kerb's edge): on at once, gone
        // within half a second on its flat top.
        ride(0.1, 0.0, [&](double) { heights(80.0, 0.0, 80.0, 25.0); });
        check(kerbPk[WheelRR] > 0.99 && kerbPk[WheelRL] == 0.0 && kerbPk[WheelFL] == 0.0, "I-9 kerb detect: RR stepping up 25 mm is on a kerb");
        ride(1.0, 0.5, [&](double) { heights(80.0, 0.0, 80.0, 25.0); });
        check(kerbPk[WheelRR] == 0.0, "I-9 kerb detect: ...for a moment: its flat top is no rumble");
        // A numbered line pads its kerb channel with 0 (Assetto Corsa sends
        // none): that 0 must not shut the detector out.
        kv.have[NcxValues::Curbs] = true; kv.val[NcxValues::Curbs] = 0.0;
        ride(0.5, 0.0, [&](double) { heights(80.0, 0.0, 80.0, 0.0); });
        ride(0.1, 0.0, [&](double) { heights(110.0, 0.0, 80.0, 0.0); });
        check(kerbPk[WheelFL] > 0.99, "I-9 kerb detect: a kerb channel stuck at 0 does not shut the detector out");
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
        // The slip speed as Automobilista 2 makes it: a rolling tyre slips
        // along the road by c of the road speed (its own creep: measured
        // against another radius than the one it rolls on) on top of its
        // slip ratio, and sideways by the slip angle. c here 6 % on the
        // fronts, 1 % on the rears.
        const double cF = 0.06, cR = 0.01;
        auto ams = [&](double kmh, double ratio, double c, double deg)
        {
            const double vms = kmh / 3.6, side = vms * std::sin(deg * PI / 180.0), along = (ratio + c) * vms;
            return std::sqrt(side * side + along * along);
        };
        auto amsAll = [&](double kmh, double rF, double rR, double degF, double degR)
        { setW(NcxValues::SlipSpeedFL, ams(kmh, rF, cF, degF), ams(kmh, rF, cF, degF), ams(kmh, rR, cR, degR), ams(kmh, rR, cR, degR)); };
        // A straight at 200 km/h, every tyre rolling: its creep alone, 3.4
        // deg's worth on the fronts (past the 3.6 deg onset with the old
        // reading's noise, and a slide on every straight).
        amsAll(200.0, 0.0, 0.0, 0.0, 0.0);
        run(40.0);   // learned over seconds (a few laps of straights)
        check(st.rollKnown[WheelFL] && st.rollKnown[WheelRR], "I-17: negative wheel speeds (Automobilista 2) still teach the rolling factor");
        std::printf("  I-17 straight-line slip FL %.3f RL %.3f of road speed, angle left %.2f deg, after %.1f s of straights\n",
                    st.latCreep[WheelFL], st.latCreep[WheelRL], st.latBase[WheelFL], st.latBaseSec);
        check(std::fabs(st.latCreep[WheelFL] - cF) < 0.002 && std::fabs(st.latCreep[WheelRL] - cR) < 0.002
              && st.latBaseSec > haptics::laws_k::kLatBaseLearnSec, "I-17: each tyre's straight-line slip along the road is learned");
        check(st.latAngle[WheelFL] < 0.3 && st.latBase[WheelFL] < 0.5, "I-17: ...and nothing is left as an angle on the straight");
        check(L.slipWheelLevel(FxType::Skid, WheelFL) < 0.02, "I-17: ...so a 200 km/h straight plays no slide (it played 0.2 as a combined slip)");
        // A long coasting corner at 100 km/h (the fronts 2 % apart), the
        // fronts at 8 deg. Five seconds: the rolling factor must not learn
        // the corner's split (it did, and the corner then counted as
        // straight for the baseline).
        v.val[NcxValues::SpeedKmh] = 100.0;
        setW(NcxValues::WheelSpeedFL, wsAt(99), wsAt(101), wsAt(99.5), wsAt(100.5));
        setW(NcxValues::SlipSpeedFL, ams(100, -0.01, cF, 8.0), ams(100, 0.01, cF, 8.0), ams(100, -0.005, cR, 0.0), ams(100, 0.005, cR, 0.0));
        const double baseBefore = st.latBase[WheelFL], creepBefore = st.latCreep[WheelFL];
        run(5.0);
        std::printf("  I-17 corner: FL angle as played %.2f deg, lateral %.2f\n", st.latAngle[WheelFL], L.slipWheelLevel(FxType::Skid, WheelFL));
        check(std::fabs(st.latAngle[WheelFL] - 8.0) < 0.4, "I-17: in a corner the slip speed plays as its sideways angle (taking c off as an angle read it about c too low)");
        check(L.slipWheelLevel(FxType::Skid, WheelFL) > 0.5, "I-17: ...a real slide on the front");
        check(std::fabs(st.latBase[WheelFL] - baseBefore) < 1e-9 && std::fabs(st.latCreep[WheelFL] - creepBefore) < 1e-9,
              "I-17: nothing is learned in a long corner");
        // Braking in a straight line: the fronts at -10 % slip ratio, their
        // slip speed only along the road; taken apart, no slide.
        v.val[NcxValues::BrakePct] = 60.0;
        setW(NcxValues::WheelSpeedFL, wsAt(90), wsAt(90), wsAt(100), wsAt(100));
        amsAll(100.0, -0.1, 0.0, 0.0, 0.0);
        run(1.0);
        check(L.slipWheelLevel(FxType::Skid, WheelFL) < 0.05, "I-17: braking straight, the along slip is taken out: no lateral slide");
        // The edge: the fronts at 10 % braking slip are working (3 % to 15 %).
        check(L.slipWheelEdge(WheelFL) > 0.3 && L.slipWheelEdge(WheelRL) < 0.01, "I-17: the edge grain: the braked fronts at 10 % work, the rolling rears do not");
        // Braking into a corner: the fronts at -10 % and 6 deg. Taking c off
        // as an angle read the fronts at 0 under braking.
        amsAll(100.0, -0.1, 0.0, 6.0, 0.0);
        run(1.0);
        std::printf("  I-17 braking into a corner: FL angle as played %.2f deg\n", st.latAngle[WheelFL]);
        check(std::fabs(st.latAngle[WheelFL] - 6.0) < 0.5, "I-17: braking into a corner, the fronts' angle is read (it read 0)");
        // Driving out: the rears at +5 % (a traction control holding them);
        // edge from 3 % feels it, edge from 8 % does not.
        v.val[NcxValues::BrakePct] = 0.0; v.val[NcxValues::ThrottlePct] = 100.0;
        setW(NcxValues::WheelSpeedFL, wsAt(100), wsAt(100), wsAt(105), wsAt(105));
        amsAll(100.0, 0.0, 0.05, 0.0, 0.0);
        run(0.5);
        check(L.slipWheelLevel(FxType::Skid, WheelRL) < 0.02, "I-17: under power on a straight, the driven rears show no false angle");
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
        // A launch: 15 km/h, the rears spinning at +80 %. No angle can be
        // told from that (a launch read as a 90 deg slide).
        v.val[NcxValues::SpeedKmh] = 15.0; v.val[NcxValues::ThrottlePct] = 100.0;
        setW(NcxValues::WheelSpeedFL, wsAt(15), wsAt(15), wsAt(27), wsAt(27));
        amsAll(15.0, 0.0, 0.8, 0.0, 0.0);
        run(2.0);   // past the hold after the road speed's jump
        check(st.latRaw[WheelRL] == 0.0 && L.slipWheelLevel(FxType::Skid, WheelRL) < 0.01, "I-17: a launch's wheelspin is no slide");
        // A real 10 deg slide on the rears fades in with road speed: a third
        // at 15 km/h, nothing at 9 (full from 25).
        v.val[NcxValues::ThrottlePct] = 0.0;
        auto rearSlideAt = [&](double kmh)
        {
            v.val[NcxValues::SpeedKmh] = kmh;
            setW(NcxValues::WheelSpeedFL, wsAt(kmh), wsAt(kmh), wsAt(kmh), wsAt(kmh));
            amsAll(kmh, 0.0, 0.0, 0.0, 10.0);
            run(2.0);
            return L.slipWheelLevel(FxType::Skid, WheelRL);
        };
        const double slide30 = rearSlideAt(30.0), slide15 = rearSlideAt(15.0), slide9 = rearSlideAt(9.0);
        std::printf("  I-17 a 10 deg rear slide at 30 / 15 / 9 km/h: %.2f / %.2f / %.2f\n", slide30, slide15, slide9);
        check(slide30 > 0.9 && std::fabs(slide15 / slide30 - 1.0 / 3.0) < 0.08 && slide9 < 0.01,
              "I-17: at walking pace the slide fades out (gone by 10 km/h)");
        v.val[NcxValues::SpeedKmh] = 100.0;

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
        // A flag sent but never on is the car's ABS switched off (Assetto
        // Corsa, aid off): a driver pumping the pedal through lock-ups is
        // not read as ABS.
        hunt(1.0, abs10);
        check(absOff(), "I-17: from wheels with the flag, the sim's flag sent but never on (ABS off): the wheels are not ABS");
        // On its own: a game whose flag is always 0 (the SimHub plugin
        // sends 0 where SimHub has none): the wheels alone.
        pp.fromWheels = 2.0; L.configurePulse(FxType::AbsPulse, pp);
        hunt(1.0, abs10);
        check(st.absWheelHoldSec > 0.0, "I-17: from wheels on its own: the wheels are ABS working, flag or none");
        pp.fromWheels = 1.0; L.configurePulse(FxType::AbsPulse, pp);
        v.val[NcxValues::BrakePct] = 0.0; run(0.6); v.val[NcxValues::BrakePct] = 80.0;
        hunt(0.5, [&](double) { return 0.0; });
        // The flag seen once (Automobilista 2 sends it now and then; here
        // off the brake, so it plays nothing itself): the wheels fill its
        // gaps from then on.
        v.val[NcxValues::BrakePct] = 0.0;
        v.val[NcxValues::AbsActive] = 1.0; run(1.0 / 60.0); v.val[NcxValues::AbsActive] = 0.0;
        run(0.3);
        v.val[NcxValues::BrakePct] = 80.0;
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
        // A sim that sends no ABS flag at all: the wheels are all there is.
        {
            haptics::Layer L2; haptics::LawsState st2;
            L2.configureFx(FxType::AbsPulse, fx); L2.configurePulse(FxType::AbsPulse, pp);
            NcxValues v2 = v; v2.have[NcxValues::AbsActive] = false;
            const int n = static_cast<int>(2.5 / dt); int lastK = -1; double held = 0.0;
            for (int i = 0; i < n; ++i)
            {
                // A second cruising (the rolling factor), then the stop.
                const double t = i * dt; const int k = static_cast<int>(t * 60.0);
                v2.val[NcxValues::BrakePct] = (t < 1.0) ? 0.0 : 80.0;
                if (k != lastK) { lastK = k; held = (t < 1.0) ? 0.0 : abs10(t); }
                const double wf = wsAt(100.0 * (1.0 + held));
                v2.val[NcxValues::WheelSpeedFL] = wf; v2.val[NcxValues::WheelSpeedFR] = wf;
                haptics::driveLaws(L2, st2, v2, dt); L2.step(dt);
            }
            check(st2.absWheelHoldSec > 0.0, "I-17: no ABS flag sent at all: the wheels are ABS working");
        }

        // Off below walking pace: the sim's flag held on at 3 km/h with the
        // brake down (the BMW in the pits) plays no ABS; unticked it does.
        // (The speed jump is a glitch the guard holds for half a second.)
        v.val[NcxValues::AbsActive] = 1.0;
        v.val[NcxValues::SpeedKmh] = 3.0; setW(NcxValues::WheelSpeedFL, wsAt(3), wsAt(3), wsAt(3), wsAt(3));
        pp.fromWheels = 0.0; pp.lowOff = 1.0; L.configurePulse(FxType::AbsPulse, pp);
        double absPk = 0.0;   // over the whole second: the speed jump's half-second hold included
        for (int i = 0; i < static_cast<int>(1.0 / dt); ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); absPk = std::max(absPk, L.fxLevel(static_cast<int>(FxType::AbsPulse))); }
        check(absPk < 0.01, "I-17: ABS flag on at 3 km/h: nothing below 5 km/h, through the speed jump's hold too");
        pp.lowOff = 0.0; L.configurePulse(FxType::AbsPulse, pp);
        run(0.5);
        check(L.fxLevel(static_cast<int>(FxType::AbsPulse)) > 0.99, "I-17: ...off < 5 km/h unticked, the flag plays");
        // Amp 0 is no ABS at all: its slip link does not judder the tyres
        // (a car without ABS, its entry's amp 0, the tickboxes left on from
        // the last car). Kept amp with no routes: felt through the tyres only.
        v.val[NcxValues::SpeedKmh] = 100.0; setW(NcxValues::WheelSpeedFL, wsAt(100), wsAt(100), wsAt(100), wsAt(100));
        pp.lowOff = 1.0; pp.slipLink = 1.0; L.configurePulse(FxType::AbsPulse, pp);
        haptics::EffectParams off = fx; off.ampPct = 0.0; L.configureFx(FxType::AbsPulse, off);
        double lockPk = 0.0;
        for (int i = 0; i < static_cast<int>(1.0 / dt); ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); if (i * dt > 0.6) lockPk = std::max(lockPk, L.slipWheelLevel(FxType::Lockup, WheelRL)); }
        check(L.fxLevel(static_cast<int>(FxType::AbsPulse)) < 0.01 && lockPk < 0.01, "I-17: ABS amp 0: no ABS, and no lock judder through its slip link");
        haptics::startPreview(st, FxType::AbsPulse);
        lockPk = 0.0;
        for (int i = 0; i < static_cast<int>(1.0 / dt); ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); lockPk = std::max(lockPk, L.slipWheelLevel(FxType::Lockup, WheelRL)); }
        check(lockPk < 0.01, "I-17: ...nor a Test of it");
        haptics::EffectParams tyresOnly = fx; tyresOnly.routes[0] = haptics::Route{}; L.configureFx(FxType::AbsPulse, tyresOnly);
        lockPk = 0.0;
        for (int i = 0; i < static_cast<int>(1.0 / dt); ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); if (i * dt > 0.3) lockPk = std::max(lockPk, L.slipWheelLevel(FxType::Lockup, WheelRL)); }
        check(lockPk > 0.4, "I-17: ABS amp kept, no routes: felt through the tyres only (the slip link at the edge of locking)");
        L.configureFx(FxType::AbsPulse, fx);
        pp.slipLink = 0.0; L.configurePulse(FxType::AbsPulse, pp);
        v.val[NcxValues::AbsActive] = 0.0;
        v.val[NcxValues::BrakePct] = 0.0;
    }

    // ---- I-18: the Lateral baseline: learn, learn once, fixed, relearn ----
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.002;
        const double PI = 3.14159265358979;
        haptics::Layer L; haptics::LawsState st;
        haptics::EffectParams fx; fx.ampPct = 100.0; fx.routes[0] = { 0, 1.0, Part::FL };
        L.configureFx(FxType::Lockup, fx); L.configureFx(FxType::Skid, fx);
        haptics::SlipParams lat = haptics::effectInfo(haptics::Effect::Skid).slipDefaults;
        L.configureSlip(FxType::Skid, lat);
        NcxValues v{}; v.fresh = true;
        std::strncpy(v.car, "Car A", NCY_STR_LEN - 1);
        auto setW = [&](int group, double x) { for (int w = 0; w < 4; ++w) { v.have[group + w] = true; v.val[group + w] = x; } };
        auto run = [&](double secs) { const int n = static_cast<int>(secs / dt); for (int i = 0; i < n; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); } };
        auto straightAt = [&](double deg) { setW(NcxValues::SlipSpeedFL, 55.556 * std::sin(deg * PI / 180.0)); };
        auto mode = [&](double m, double f = 0.0, double r = 0.0) { lat.baseMode = m; lat.baseDegF = f; lat.baseDegR = r; L.configureSlip(FxType::Skid, lat); };
        v.have[NcxValues::BrakePct] = true; v.val[NcxValues::BrakePct] = 0.0;
        v.have[NcxValues::ThrottlePct] = true; v.val[NcxValues::ThrottlePct] = 0.0;
        v.have[NcxValues::SpeedKmh] = true; v.val[NcxValues::SpeedKmh] = 200.0;
        setW(NcxValues::WheelSpeedFL, -(200.0 / 3.6) / 0.33);
        // A straight's slip speed, rolling, is the tyres' slip along the
        // road (c): learned as that, nothing left as an angle.
        const double c4 = std::sin(4.0 * PI / 180.0), c2 = std::sin(2.0 * PI / 180.0);
        straightAt(4.0);
        run(40.0);
        std::printf("  I-18 learned: along %.4f (true %.4f), angle left %.2f, played %.2f deg\n",
                    st.latCreep[WheelFL], c4, st.latBase[WheelFL], st.latAngle[WheelFL]);
        check(std::fabs(st.latCreep[WheelFL] - c4) < 0.002 && st.latBase[WheelFL] < 0.6 && st.latAngle[WheelFL] < 0.3
              && st.latBaseSec >= haptics::laws_k::kLatBaseLearnSec,
              "I-18 learn: a straight's slip is learned as the tyres' own slip along the road");
        haptics::relearnLatBase(st);
        check(st.latCreep[WheelFL] == 0.0 && st.latBase[WheelFL] == 0.0 && st.latBaseSec == 0.0, "I-18 relearn: the learning starts over");
        run(40.0);
        check(std::fabs(st.latCreep[WheelFL] - c4) < 0.002, "I-18 relearn: ...and learns again from the next straights");
        // Learn once: learned, then held whatever the straights say.
        mode(1.0);
        const double held = st.latCreep[WheelFL], heldBase = st.latBase[WheelFL];
        straightAt(2.0);
        run(5.0);
        check(st.latCreep[WheelFL] == held && st.latBase[WheelFL] == heldBase, "I-18 learn once: once learned it is held (a lighter straight does not move it)");
        // Learn: it follows the lighter straight, and plays nothing while
        // it does.
        mode(0.0);
        run(30.0);
        check(std::fabs(st.latCreep[WheelFL] - c2) < 0.003 && st.latAngle[WheelFL] < 0.3, "I-18 learn: it follows a lighter straight, playing nothing on it");
        // Fixed: base deg F and R, no learning; the angle played is the
        // straight's 2 deg less them.
        mode(2.0, 1.5, 0.5);
        const double secBefore = st.latBaseSec;
        run(3.0);
        check(st.latBase[WheelFL] == 1.5 && st.latBase[WheelFR] == 1.5 && st.latBase[WheelRL] == 0.5 && st.latBase[WheelRR] == 0.5,
              "I-18 fixed: base deg F on the fronts, R on the rears");
        check(std::fabs(st.latAngle[WheelFL] - 0.5) < 0.1 && std::fabs(st.latAngle[WheelRL] - 1.5) < 0.1 && st.latBaseSec == secBefore
              && st.latCreep[WheelFL] == 0.0,
              "I-18 fixed: the angle played is the sim's less them, nothing learned (no slip along the road either)");
        // Back to learn: the fixed values were never learned, so it starts over.
        mode(0.0);
        run(dt);
        check(st.latBaseSec < 0.01 && st.latBase[WheelFL] < 0.01, "I-18: leaving fixed starts the learning over");
        run(10.0);
        check(st.latCreep[WheelFL] > 0.5 * c2, "I-18: ...and it learns again");
        // A new car starts it over too.
        std::strncpy(v.car, "Car B", NCY_STR_LEN - 1);
        run(dt);
        check(st.latBaseSec < 0.01 && st.latBase[WheelFL] < 0.01 && st.latCreep[WheelFL] < 0.001, "I-18: a new car starts the learning over");

        // A front-driven car: on the straights its fronts drive (+3 %) and
        // roll only on a lift. Each wheel learns on its own clock, so the
        // fronts' slip along the road is learned on the lifts and they show
        // no angle under power (one clock for all four ran out on the
        // power: 4 deg of scrub on every straight, learn once for good).
        for (double m : { 0.0, 1.0 })
        {
            std::strncpy(v.car, m == 0.0 ? "Mini learn" : "Mini learn once", NCY_STR_LEN - 1);
            mode(m);
            const double kmh = 150.0, vms = kmh / 3.6, cF = 0.05, cR = 0.01;
            v.val[NcxValues::SpeedKmh] = kmh;
            auto drive = [&](double frontAhead, double throttle)
            {
                v.val[NcxValues::ThrottlePct] = throttle;
                v.val[NcxValues::WheelSpeedFL] = v.val[NcxValues::WheelSpeedFR] = -vms * (1.0 + frontAhead) / 0.33;
                v.val[NcxValues::WheelSpeedRL] = v.val[NcxValues::WheelSpeedRR] = -vms / 0.33;
                v.val[NcxValues::SlipSpeedFL] = v.val[NcxValues::SlipSpeedFR] = (frontAhead + cF) * vms;
                v.val[NcxValues::SlipSpeedRL] = v.val[NcxValues::SlipSpeedRR] = cR * vms;
            };
            drive(0.0, 0.0);  run(1.0);   // out of the pits, coasting
            for (int lap = 0; lap < 8; ++lap) { drive(0.03, 100.0); run(6.0); drive(0.0, 0.0); run(1.5); }
            drive(0.03, 100.0); run(2.0);
            std::printf("  I-18 front-driven (%s): along FL %.4f (true %.2f), RL %.4f, FL under power %.2f deg, learned %.1f s\n",
                        m == 0.0 ? "learn" : "learn once", st.latCreep[WheelFL], cF, st.latCreep[WheelRL], st.latAngle[WheelFL], st.latBaseSec);
            check(std::fabs(st.latCreep[WheelFL] - cF) < 0.005 && std::fabs(st.latCreep[WheelRL] - cR) < 0.002,
                  "I-18 front-driven: the driving fronts learn their slip along the road on the lifts");
            check(st.latAngle[WheelFL] < 1.0 && L.slipWheelLevel(FxType::Skid, WheelFL) < 0.02,
                  "I-18 front-driven: ...and show no scrub under power on the straight");
        }
        mode(0.0);
    }

    // ---- I-19: the Wheels tile's pads: the grab and the drag ----
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.0005;
        haptics::Layer L; haptics::LawsState st;
        haptics::EffectParams wp; wp.ampPct = 100.0; wp.routes[0] = { 0, 1.0, Part::FL };   // a belt: a force
        L.configureFx(FxType::Wheels, wp);
        haptics::WheelsParams q; q.balance = 0.0; q.flat = 0.0; q.judder = 0.0; q.puncture = 0.0;
        q.bite = 1.0; q.drag = 0.0; q.biteHz = 40.0; q.dragHz = 80.0;
        L.configureWheels(q);
        NcxValues v{}; v.fresh = true;
        v.have[NcxValues::SpeedKmh] = true; v.val[NcxValues::SpeedKmh] = 100.0;
        v.have[NcxValues::BrakePct] = true; v.val[NcxValues::BrakePct] = 0.0;
        // Peak |output| over a window, and its zero crossings (the pitch).
        auto watch = [&](double secs, double& pk, int& xings) {
            pk = 0.0; xings = 0; double last = 0.0;
            for (int i = 0; i < static_cast<int>(secs / dt); ++i)
            {
                haptics::driveLaws(L, st, v, dt); L.step(dt);
                const double o = L.overlayFor(0);
                pk = std::max(pk, std::fabs(o));
                if (i > 0 && (o >= 0.0) != (last >= 0.0) && std::fabs(o - last) > 1e-9) ++xings;
                last = o;
            } };
        double pk = 0.0; int xr = 0;
        watch(1.0, pk, xr);
        check(pk < 1e-6, "I-19 pads: no brake, nothing");
        v.val[NcxValues::BrakePct] = 60.0;
        double grabPk = 0.0; watch(0.3, grabPk, xr);
        const double grabHz = xr / 2.0 / 0.3;
        double laterPk = 0.0; watch(1.0, laterPk, xr); watch(0.3, laterPk, xr);   // a second on, the pedal held
        std::printf("  I-19 grab peak %.1f (pitch %.0f Hz at 100 km/h), a second later %.2f\n", grabPk, grabHz, laterPk);
        check(grabPk > 30.0, "I-19 pads: the brake going on grabs");
        check(std::fabs(grabHz - 40.0) < 10.0, "I-19 pads: ...at bite hz at 100 km/h (dithered)");
        check(laterPk < 0.1 * grabPk, "I-19 pads: ...fading once the pedal holds (drag 0)");
        q.drag = 1.0; L.configureWheels(q);
        double dragPk = 0.0; watch(0.5, dragPk, xr);
        const double dragHz = xr / 2.0 / 0.5;
        check(dragPk > 20.0 && std::fabs(dragHz - 80.0) < 20.0, "I-19 pads: the drag rubs on while the brake stays on, at drag hz");
        v.val[NcxValues::SpeedKmh] = 50.0; watch(0.6, pk, xr);   // past the speed jump's hold
        watch(0.5, pk, xr);
        check(std::fabs(xr / 2.0 / 0.5 - 40.0) < 12.0, "I-19 pads: at 50 km/h the drag's pitch is half");
        // No step on or off: the grain eases in from a zero crossing and out.
        {
            v.val[NcxValues::SpeedKmh] = 100.0; v.val[NcxValues::BrakePct] = 0.0;
            watch(1.0, pk, xr);   // past the speed jump's hold, the pads off
            double last = L.overlayFor(0), maxStep = 0.0, peak = 0.0;
            v.val[NcxValues::BrakePct] = 60.0;
            for (int i = 0; i < static_cast<int>(0.3 / dt); ++i)
            {
                haptics::driveLaws(L, st, v, dt); L.step(dt);
                const double o = L.overlayFor(0);
                maxStep = std::max(maxStep, std::fabs(o - last)); peak = std::max(peak, std::fabs(o)); last = o;
            }
            v.val[NcxValues::BrakePct] = 0.0;
            for (int i = 0; i < static_cast<int>(0.1 / dt); ++i)
            {
                haptics::driveLaws(L, st, v, dt); L.step(dt);
                const double o = L.overlayFor(0);
                maxStep = std::max(maxStep, std::fabs(o - last)); last = o;
            }
            std::printf("  I-19 pads: largest step %.1f of a %.1f peak\n", maxStep, peak);
            check(peak > 30.0 && maxStep < 0.4 * peak, "I-19 pads: the grab and its release step nowhere");
        }
        // The pitch never asks the loop for fewer than 8 samples a cycle: at
        // 300 km/h a bite hz of 200 would be 600 Hz, which aliased.
        {
            q.biteHz = 200.0; q.drag = 0.0; L.configureWheels(q);
            v.val[NcxValues::SpeedKmh] = 300.0; watch(1.0, pk, xr);
            v.val[NcxValues::BrakePct] = 60.0; watch(0.2, pk, xr);
            const double hz = xr / 2.0 / 0.2;
            std::printf("  I-19 pads: bite hz 200 at 300 km/h plays at %.0f Hz (the cap %.0f)\n", hz, 1.0 / (dt * 8.0));
            check(hz <= 1.0 / (dt * 8.0) * 1.15, "I-19 pads: the pitch is capped where the loop can draw it");
            v.val[NcxValues::BrakePct] = 60.0; v.val[NcxValues::SpeedKmh] = 50.0; watch(1.0, pk, xr);
            q.biteHz = 40.0; q.drag = 1.0; L.configureWheels(q);
        }
        // A locked wheel's disc is not turning under the pads: nothing.
        for (int w = 0; w < 4; ++w) { v.have[NcxValues::SlipRatioFL + w] = true; v.val[NcxValues::SlipRatioFL + w] = -1.0; }
        watch(0.2, pk, xr); watch(0.5, pk, xr);   // (after the grain has eased out)
        check(pk < 1e-6, "I-19 pads: a locked wheel gives nothing");
    }

    // ---- I-20: the Driveline tile's differential ----
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.0005;
        haptics::Layer L; haptics::LawsState st;
        haptics::EffectParams dp; dp.ampPct = 100.0; dp.routes[0] = { 0, 1.0, Part::All };   // a belt: a force
        L.configureFx(FxType::Driveline, dp);
        haptics::DrivelineParams d; d.clutch = 0.0; d.lug = 0.0; d.whine = 0.0; d.shunt = 0.0;
        d.diff = 1.0; d.diffType = 0.0; d.diffHz = 8.0; d.driven = 0.0;
        L.configureDriveline(d);
        NcxValues v{}; v.fresh = true;
        v.have[NcxValues::SpeedKmh] = true; v.val[NcxValues::SpeedKmh] = 10.0;
        v.have[NcxValues::ThrottlePct] = true; v.val[NcxValues::ThrottlePct] = 60.0;
        auto ratios = [&](double fl, double fr, double rl, double rr)
        { const double x[4] = { fl, fr, rl, rr }; for (int w = 0; w < 4; ++w) { v.have[NcxValues::SlipRatioFL + w] = true; v.val[NcxValues::SlipRatioFL + w] = x[w]; } };
        // Clacks over a window: how many, their spacing's spread, the peak.
        auto listen = [&](double secs, int& clacks, double& spread, double& pk) {
            // A clack: output after at least 5 ms of silence.
            clacks = 0; pk = 0.0; int quiet = 1000; double last = -1.0; std::vector<double> gaps;
            for (int i = 0; i < static_cast<int>(secs / dt); ++i)
            {
                haptics::driveLaws(L, st, v, dt); L.step(dt);
                const double o = std::fabs(L.overlayFor(0));
                pk = std::max(pk, o);
                if (o < 1e-9) { ++quiet; continue; }
                if (quiet >= 10) { ++clacks; if (last >= 0.0) gaps.push_back(i * dt - last); last = i * dt; }
                quiet = 0;
            }
            double m = 0.0; for (double g : gaps) m += g; m = gaps.empty() ? 0.0 : m / gaps.size();
            double var = 0.0; for (double g : gaps) var += (g - m) * (g - m);
            spread = (gaps.size() > 1 && m > 0.0) ? std::sqrt(var / gaps.size()) / m : 0.0;
        };
        // Wheel speeds as Assetto Corsa and Automobilista 2 send them (the
        // corner's geometry in them), m/s at the tread.
        auto speeds = [&](double fl, double fr, double rl, double rr)
        { const double x[4] = { fl, fr, rl, rr }; for (int w = 0; w < 4; ++w) { v.have[NcxValues::WheelSpeedFL + w] = true; v.val[NcxValues::WheelSpeedFL + w] = x[w]; } };
        auto noSpeeds = [&]() { for (int w = 0; w < 4; ++w) v.have[NcxValues::WheelSpeedFL + w] = false; };
        // The road speed and each axle's outside wheels this share apart.
        auto corner = [&](double kmh, double frontApart, double rearApart)
        {
            const double c = kmh / 3.6;
            v.val[NcxValues::SpeedKmh] = kmh;
            speeds((1.0 - 0.5 * frontApart) * c, (1.0 + 0.5 * frontApart) * c,
                   (1.0 - 0.5 * rearApart) * c, (1.0 + 0.5 * rearApart) * c);
        };
        v.have[NcxValues::Gear] = true; v.val[NcxValues::Gear] = 1.0;
        v.have[NcxValues::BrakePct] = true; v.val[NcxValues::BrakePct] = 0.0;
        int n = 0; double spread = 0.0, pk = 0.0;
        corner(10.0, 0.0, 0.0);
        listen(1.0, n, spread, pk);
        check(n == 0, "I-20 diff: driving straight, the plates hold: nothing");
        // Full lock at 10 km/h under 60 % throttle (out of the pits, turning
        // round): the outside wheels 20 % faster than the inside ones.
        corner(10.0, 0.20, 0.20);
        listen(2.0, n, spread, pk);
        std::printf("  I-20 diff: %d clunks in 2 s (spacing spread %.0f %%), peak %.1f\n", n, 100.0 * spread, pk);
        check(n >= 12 && n <= 20, "I-20 diff: a walking-pace full-lock turn under power clunks at diff hz");
        check(spread > 0.03 && spread < 0.15, "I-20 diff: ...evenly, like a ratchet, but not a tone");
        const double onPower = L.diffLevel();
        v.val[NcxValues::ThrottlePct] = 0.0; listen(1.0, n, spread, pk);
        check(L.diffLevel() > 0.5 * onPower && L.diffLevel() < 0.75 * onPower, "I-20 diff: off the throttle, the preload's lighter clunk");
        v.val[NcxValues::ThrottlePct] = 60.0;
        v.val[NcxValues::Gear] = 0.0; listen(1.0, n, spread, pk);
        check(L.diffLevel() < 0.75 * onPower, "I-20 diff: in neutral, no drive through it: the preload alone");
        v.val[NcxValues::Gear] = 1.0;
        v.val[NcxValues::BrakePct] = 50.0; listen(0.5, n, spread, pk); listen(1.0, n, spread, pk);
        check(n == 0, "I-20 diff: under braking (a lock-up, ABS), nothing");
        v.val[NcxValues::BrakePct] = 0.0;
        // Wheels turning apart slowly (5 km/h, 10 %): fewer clunks.
        corner(5.0, 0.10, 0.10); listen(0.5, n, spread, pk); listen(2.0, n, spread, pk);
        check(n >= 2 && n <= 8, "I-20 diff: the wheels turning apart slowly, the clunks come slower");
        // A racing hairpin: 25 km/h, 8 % apart. A healthy diff is silent.
        v.val[NcxValues::Gear] = 2.0;
        corner(25.0, 0.08, 0.08); listen(1.0, n, spread, pk); listen(1.0, n, spread, pk);
        check(n == 0, "I-20 diff: a racing hairpin at 25 km/h, nothing");
        corner(100.0, 0.20, 0.20); listen(1.0, n, spread, pk);
        check(n == 0, "I-20 diff: at speed, nothing");
        v.val[NcxValues::Gear] = 1.0;
        // The inside rear spinning on the way out (driven 30 % apart, the
        // free fronts together): the corner's geometry is not in it.
        corner(10.0, 0.0, 0.30); listen(0.5, n, spread, pk); listen(1.0, n, spread, pk);
        check(n == 0, "I-20 diff: a spinning inside wheel is no diff");
        // The car rotating (the free axle 50 % apart): a spin, nothing.
        corner(10.0, 0.50, 0.50); listen(0.5, n, spread, pk); listen(1.0, n, spread, pk);
        check(n == 0, "I-20 diff: a spinning car is no diff");
        // Two near-stopped wheels' split is noise.
        corner(1.5, 0.20, 0.20); listen(0.5, n, spread, pk); listen(1.0, n, spread, pk);
        check(n == 0, "I-20 diff: below 2 km/h, nothing");
        corner(10.0, 0.20, 0.20); listen(0.5, n, spread, pk);
        d.diff = 0.0; L.configureDriveline(d); listen(1.0, n, spread, pk);
        check(n == 0, "I-20 diff: diff x 0 (an open diff), nothing");
        // A sim's own slip ratios (Competizione) leave the geometry out:
        // with no wheel speeds, nothing.
        d.diff = 1.0; L.configureDriveline(d);
        noSpeeds(); ratios(0.0, 0.0, 0.08, 0.0);
        listen(0.5, n, spread, pk); listen(1.0, n, spread, pk);
        check(n == 0, "I-20 diff: from a sim's slip ratios alone, nothing");
        for (int w = 0; w < 4; ++w) v.have[NcxValues::SlipRatioFL + w] = false;
        // The clunks never cut each other short, even at the fastest rate.
        {
            corner(10.0, 0.20, 0.20);
            d.diffHz = 30.0; L.configureDriveline(d);
            double last = 0.0, maxStep = 0.0, peak = 0.0;
            for (int i = 0; i < static_cast<int>(1.0 / dt); ++i)
            {
                haptics::driveLaws(L, st, v, dt); L.step(dt);
                const double o = L.overlayFor(0);
                maxStep = std::max(maxStep, std::fabs(o - last)); peak = std::max(peak, std::fabs(o)); last = o;
            }
            check(peak > 10.0 && maxStep < 0.25 * peak, "I-20 diff: at diff hz 30 no clunk is cut short (no step)");
            d.diffHz = 8.0; L.configureDriveline(d);
        }
        // Front-driven: the rears are the free axle. The inside front
        // spinning on the way out is no diff; a full-lock turn is.
        d.driven = 1.0; L.configureDriveline(d);
        corner(10.0, 0.35, 0.02); listen(0.5, n, spread, pk); listen(1.0, n, spread, pk);
        check(n == 0, "I-20 diff: front-driven, the inside front spinning is no diff");
        corner(10.0, 0.20, 0.20); listen(2.0, n, spread, pk);
        check(n >= 12, "I-20 diff: front-driven, a walking-pace full-lock turn clunks");
        // All driven: the axle further apart is held to the other's corner.
        d.driven = 2.0; L.configureDriveline(d);
        corner(10.0, 0.20, 0.25); listen(2.0, n, spread, pk);
        check(n >= 12, "I-20 diff: all driven, a walking-pace full-lock turn clunks");
        corner(10.0, 0.01, 0.30); listen(0.5, n, spread, pk); listen(1.0, n, spread, pk);
        check(n == 0, "I-20 diff: all driven, one axle spinning is no diff");
        d.driven = 0.0;
        // A spool: the rears turn as one (no split), the inside tyre scrubs;
        // how tight the corner is comes from the free fronts' speeds.
        d.diffType = 1.0; L.configureDriveline(d);
        corner(10.0, 0.20, 0.0);
        listen(2.0, n, spread, pk);
        check(n >= 12, "I-20 diff: a spool in a tight turn scrubs and hops (from the free axle)");
        corner(25.0, 0.20, 0.0); listen(2.0, n, spread, pk);
        check(n >= 3, "I-20 diff: ...still at 25 km/h, fading (gone by 30)");
        corner(35.0, 0.20, 0.0); listen(0.5, n, spread, pk); listen(1.0, n, spread, pk);
        check(n == 0, "I-20 diff: ...and gone above 30 km/h");
        corner(10.0, 0.0, 0.0); listen(1.0, n, spread, pk); listen(1.0, n, spread, pk);
        check(n == 0, "I-20 diff: ...and straight, nothing");
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
        // The gear as the sims really send it: a neutral for a moment in
        // every shift, and Automobilista 2's one-frame junk gear (a 5-4
        // downshift read 5, N, 3, 4: three knocks). A synchro box, so one
        // knock per shift (no dog engagement knock).
        {
            haptics::DrivelineParams dpar = L.drivelineParams(); dpar.gearbox = 0.0; dpar.ignoreN = 1.0; L.configureDriveline(dpar);
            const auto shifts = [&]() { return L.fireCount(haptics::EventType::GearShift); };
            v.have[NcxValues::ThrottlePct] = true; v.val[NcxValues::ThrottlePct] = 0.0;
            uint64_t s0 = shifts();
            v.val[NcxValues::Gear] = 0.0; run(0.06);
            check(shifts() - s0 == 1, "I-15 gear: ignore neutral knocks as the box leaves 4 (45 ms into the neutral), not when 5 goes in");
            run(0.09); v.val[NcxValues::Gear] = 5.0; run(0.3);
            check(shifts() - s0 == 1, "I-15 gear: 4, N for 150 ms, 5 is one shift (ignore neutral)");
            s0 = shifts();
            v.val[NcxValues::Gear] = 0.0; run(0.18); v.val[NcxValues::Gear] = 3.0; run(1.0 / 60.0); v.val[NcxValues::Gear] = 4.0; run(0.3);
            check(shifts() - s0 == 1, "I-15 gear: 5, N, a one-frame junk 3, then 4 is one shift");
            s0 = shifts();
            v.val[NcxValues::Gear] = 2.0; run(1.0 / 60.0); v.val[NcxValues::Gear] = 4.0; run(0.3);
            check(shifts() == s0, "I-15 gear: a one-frame junk gear on its own is no shift");
            dpar.ignoreN = 0.0; L.configureDriveline(dpar);
            s0 = shifts();
            v.val[NcxValues::Gear] = 0.0; run(0.15); v.val[NcxValues::Gear] = 5.0; run(0.3);
            check(shifts() - s0 == 2, "I-15 gear: ignore neutral off, going into N knocks too");
            dpar.ignoreN = 1.0; L.configureDriveline(dpar);
            // A junk frame (gear 128, dropped) in the middle of a real shift
            // does not swallow it.
            s0 = shifts();
            v.val[NcxValues::Gear] = 4.0; run(0.02); v.val[NcxValues::Gear] = 128.0; run(1.0 / 60.0);
            v.val[NcxValues::Gear] = 4.0; run(0.3);
            check(shifts() - s0 == 1, "I-15 gear: a junk frame mid-shift does not swallow the shift");
            // A neutral held half a second is a real one: out of gear knocks,
            // and selecting the same gear again knocks too.
            s0 = shifts();
            v.val[NcxValues::Gear] = 0.0; run(0.7); v.val[NcxValues::Gear] = 4.0; run(0.3);
            check(shifts() - s0 == 2, "I-15 gear: out of gear into a real neutral knocks, and back into the same gear knocks");
            // A dog box shifted under power: the shift knocks as the box
            // leaves its gear and the engagement knock comes as the next gear
            // goes in (not 60 ms after the first, in the middle of the
            // neutral); a lift-off shift has no engagement knock.
            dpar.gearbox = 1.0; L.configureDriveline(dpar);
            v.val[NcxValues::ThrottlePct] = 100.0;
            s0 = shifts();
            v.val[NcxValues::Gear] = 0.0; run(0.14);
            check(shifts() - s0 == 1, "I-15 gear: dog box under power, one knock through the neutral (none mid-neutral)");
            v.val[NcxValues::Gear] = 5.0; run(0.3);
            check(shifts() - s0 == 2, "I-15 gear: ...and the engagement knock as 5 goes in");
            v.val[NcxValues::ThrottlePct] = 0.0;
            s0 = shifts();
            v.val[NcxValues::Gear] = 0.0; run(0.15); v.val[NcxValues::Gear] = 4.0; run(0.3);
            check(shifts() - s0 == 1, "I-15 gear: dog box off throttle, one knock (no engagement knock)");
            // A blipped downshift: off the throttle as it leaves, on it as the
            // next gear goes in: the dogs engage under power, so it knocks.
            s0 = shifts();
            v.val[NcxValues::Gear] = 0.0; run(0.15); v.val[NcxValues::ThrottlePct] = 80.0; v.val[NcxValues::Gear] = 3.0; run(0.3);
            check(shifts() - s0 == 2, "I-15 gear: dog box, blipped downshift: the engagement knock as the gear goes in");
            // Ignore neutral switched on mid-shift (after the knock into N):
            // the gear going in knocks as with it off, no third knock.
            v.val[NcxValues::ThrottlePct] = 0.0;
            dpar.gearbox = 0.0; dpar.ignoreN = 0.0; L.configureDriveline(dpar);
            s0 = shifts();
            v.val[NcxValues::Gear] = 0.0; run(0.1);
            dpar.ignoreN = 1.0; L.configureDriveline(dpar);
            v.val[NcxValues::Gear] = 4.0; run(0.3);
            check(shifts() - s0 == 2, "I-15 gear: ignore neutral switched on mid-shift: two knocks, as it was off when the shift began");
        }
        // A junk engine speed (Automobilista 2 sent 191,105 rpm once, with
        // the plugin's limiter flag set from it) is not taken: the last one
        // that could be true stands, and the flag goes with it. Nor an rpm
        // changing faster than an engine can within one sample; held a
        // moment, a real change stands.
        { haptics::EffectParams lp; lp.ampPct = 100.0; lp.freqHz = 17.0; lp.routes[0] = { 2, 1.0, Part::All }; L.configureFx(FxType::Limiter, lp); }
        v.have[NcxValues::Rpm] = true; v.val[NcxValues::Rpm] = 6000.0;
        v.have[NcxValues::Limiter] = true; v.val[NcxValues::Limiter] = 1.0; run(1.0 / 60.0);
        check(L.fxLevel(static_cast<int>(FxType::Limiter)) > 0.2, "I-15 rpm: a true frame's limiter flag plays (the control)");
        v.val[NcxValues::Limiter] = 0.0; run(0.3);
        v.val[NcxValues::Rpm] = 191105.0; v.val[NcxValues::Limiter] = 1.0; run(1.0 / 60.0);
        check(st.rpmSeen && st.lastRpm == 6000.0, "I-15 rpm: a 191,105 rpm junk frame is not taken");
        check(L.fxLevel(static_cast<int>(FxType::Limiter)) < 0.01, "I-15 rpm: ...nor its limiter flag");
        v.val[NcxValues::Limiter] = 0.0;
        v.val[NcxValues::Rpm] = 6100.0; run(1.0 / 60.0);
        v.val[NcxValues::Rpm] = 12000.0; run(1.0 / 60.0);
        check(st.lastRpm == 6100.0, "I-15 rpm: 6100 to 12000 in one sample is junk");
        run(0.2);
        check(st.lastRpm == 12000.0, "I-15 rpm: ...held a moment, it stands");
        // An F1 engine at 18,000: a stall (or the ignition off) is taken at
        // once; a reset to the pits idling at 1500 within a quarter second
        // (a tenth-second ceiling held it at 18,000 for good).
        v.val[NcxValues::Rpm] = 18000.0; run(0.4);
        v.val[NcxValues::Rpm] = 0.0; run(1.0 / 60.0);
        check(st.lastRpm == 0.0, "I-15 rpm: 18,000 to a stall is taken at once");
        v.val[NcxValues::Rpm] = 18000.0; run(0.4);
        v.val[NcxValues::Rpm] = 1500.0; run(0.3);
        check(st.lastRpm == 1500.0, "I-15 rpm: 18,000 to 1500 (a reset to idle) is taken within a quarter second");
        v.have[NcxValues::Limiter] = false;
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

    // ---- I-21: the rolling factor from short moments ----
    {
        using haptics::FxType;
        const double dt = 0.002;
        auto fresh = []() {
            NcxValues v{}; v.fresh = true;
            v.have[NcxValues::SpeedKmh] = v.have[NcxValues::ThrottlePct] = v.have[NcxValues::BrakePct] = true;
            v.val[NcxValues::SpeedKmh] = 120.0; v.val[NcxValues::BrakePct] = 0.0;
            for (int w = 0; w < 4; ++w) { v.have[NcxValues::WheelSpeedFL + w] = true; v.val[NcxValues::WheelSpeedFL + w] = 104.0; }
            return v; };
        // A race car: light throttle only in short moments (a lift, a
        // breath), flat out between with the rears 4 % ahead of the road
        // (a 312T in Assetto Corsa went minutes with none of it learned).
        {
            haptics::Layer L; haptics::LawsState st; NcxValues v = fresh();
            haptics::EffectParams lp; lp.ampPct = 100.0; lp.routes[0] = { 0, 1.0 }; L.configureFx(FxType::Lockup, lp);
            double knownAt = -1.0;
            for (int i = 0; i < static_cast<int>(6.0 / dt); ++i)
            {
                const double t = i * dt, u = std::fmod(t, 0.5);
                const bool lift = u < 0.08;
                v.val[NcxValues::ThrottlePct] = lift ? 20.0 : 100.0;
                v.val[NcxValues::WheelSpeedRL] = v.val[NcxValues::WheelSpeedRR] = lift ? 104.0 : 104.0 * 1.04;
                haptics::driveLaws(L, st, v, dt); L.step(dt);
                if (knownAt < 0.0 && st.rollKnown[0] && st.rollKnown[1] && st.rollKnown[2] && st.rollKnown[3]) knownAt = t;
            }
            std::printf("  I-21 rolling factor from 80 ms lifts every half second: known at %.1f s\n", knownAt);
            check(knownAt > 0.0 && knownAt < 4.0, "I-21: short moments add up: the rolling factor is learned (it waited for half a second without a break)");
            check(std::fabs(st.rollK[WheelRL] * 104.0 / 120.0 - 1.0) < 0.01, "I-21: ...the rolling one, not the spinning one");
        }
        // Samples that disagree (a wheel's speed all over the place from one
        // sample to the next) seed nothing; steady ones beside them do.
        {
            haptics::Layer L; haptics::LawsState st; NcxValues v = fresh();
            v.val[NcxValues::ThrottlePct] = 20.0;
            uint64_t rng = 12345;
            for (int i = 0; i < static_cast<int>(2.0 / dt); ++i)
            {
                if (i % 8 == 0)
                {
                    rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
                    const double k = 1.0 + 0.30 * (static_cast<double>(rng & 0xFFFF) / 65535.0 - 0.5);   // +-15 %, a wheel all over the place
                    v.val[NcxValues::WheelSpeedRL] = v.val[NcxValues::WheelSpeedRR] = 104.0 * k;
                }
                haptics::driveLaws(L, st, v, dt); L.step(dt);
            }
            check(!st.rollKnown[WheelRL], "I-21: wheel speeds that disagree from sample to sample teach nothing");
            check(st.rollKnown[WheelFL], "I-21: ...the steady fronts are learned");
        }
        auto runFor = [&](haptics::Layer& L, haptics::LawsState& st, NcxValues& v, double secs)
        { for (int i = 0; i < static_cast<int>(secs / dt); ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); } };
        auto setWs = [](NcxValues& v, double fl, double fr, double rl, double rr)
        { const double x[4] = { fl, fr, rl, rr }; for (int w = 0; w < 4; ++w) v.val[NcxValues::WheelSpeedFL + w] = 104.0 * x[w]; };
        // A curve (each axle's wheels 2 % apart, within the 4 % that counts
        // as turning together) teaches no first value: a pit-exit curve left
        // a front wheel 4.6 % off.
        {
            haptics::Layer L; haptics::LawsState st; NcxValues v = fresh();
            v.val[NcxValues::ThrottlePct] = 20.0;
            setWs(v, 0.99, 1.01, 0.99, 1.01);
            runFor(L, st, v, 3.0);
            check(!st.rollKnown[WheelFL] && !st.rollKnown[WheelRR], "I-21: a curve teaches no first value");
            setWs(v, 1.0, 1.0, 1.0, 1.0);
            runFor(L, st, v, 2.0);
            check(st.rollKnown[WheelFL] && std::fabs(st.rollK[WheelFL] * 104.0 / 120.0 - 1.0) < 0.002, "I-21: ...the straight after it does");
            // Once learned, a wheel 3 % off (a reset, a slide) is ignored...
            const double k0 = st.rollK[WheelFR];
            setWs(v, 1.0, 1.03, 1.0, 1.0);
            runFor(L, st, v, 3.0);
            check(std::fabs(st.rollK[WheelFR] / k0 - 1.0) < 0.001, "I-21: once learned, a moment 3 % off is ignored (a reset left a wheel 3 % off for a minute)");
            // ...unless it keeps disagreeing (a wrong first value, a real change).
            runFor(L, st, v, 6.0);
            check(st.rollK[WheelFR] / k0 < 0.985, "I-21: ...unless it keeps disagreeing for 5 s: then it is learned");
        }
        // Coasting corners lap after lap, each axle's wheels 3.5 % apart
        // (inside the 4 % that counts as together), flat out between: the
        // disagreement never adds up to the escape (it did by the third
        // lap and taught the corner's split).
        {
            haptics::Layer L; haptics::LawsState st; NcxValues v = fresh();
            v.val[NcxValues::ThrottlePct] = 20.0;
            runFor(L, st, v, 2.0);
            const double kFL = st.rollK[WheelFL], kFR = st.rollK[WheelFR];
            for (int lap = 0; lap < 6; ++lap)
            {
                v.val[NcxValues::ThrottlePct] = 20.0; setWs(v, 0.9825, 1.0175, 0.9825, 1.0175); runFor(L, st, v, 2.0);
                v.val[NcxValues::ThrottlePct] = 100.0; setWs(v, 1.0, 1.0, 1.0, 1.0); runFor(L, st, v, 5.0);
            }
            std::printf("  I-21 six cornering laps: FL %+.2f %%, FR %+.2f %%\n", 100.0 * (st.rollK[WheelFL] / kFL - 1.0), 100.0 * (st.rollK[WheelFR] / kFR - 1.0));
            check(std::fabs(st.rollK[WheelFL] / kFL - 1.0) < 0.001 && std::fabs(st.rollK[WheelFR] / kFR - 1.0) < 0.001,
                  "I-21: corners lap after lap never teach their split");
        }
        // Engine braking on every lift (the driven rears 2 % slow) is
        // left out of the disagreement: it no longer holds the gate open,
        // so a reset moment afterwards (a front 3 % off for half a second)
        // is still ignored.
        {
            haptics::Layer L; haptics::LawsState st; NcxValues v = fresh();
            v.val[NcxValues::ThrottlePct] = 20.0;
            runFor(L, st, v, 2.0);
            v.have[NcxValues::Gear] = true; v.val[NcxValues::Gear] = 4.0;
            v.have[NcxValues::ClutchPct] = true; v.val[NcxValues::ClutchPct] = 0.0;
            for (int lap = 0; lap < 5; ++lap)
            {
                v.val[NcxValues::ThrottlePct] = 100.0; setWs(v, 1.0, 1.0, 1.02, 1.02); runFor(L, st, v, 3.0);
                v.val[NcxValues::ThrottlePct] = 0.0;   setWs(v, 1.0, 1.0, 0.98, 0.98); runFor(L, st, v, 3.0);
            }
            const double kFR = st.rollK[WheelFR];
            v.val[NcxValues::ThrottlePct] = 20.0; setWs(v, 1.0, 1.03, 1.0, 1.0); runFor(L, st, v, 0.5);
            check(st.rollOffSec < haptics::laws_k::kRollGateEscapeSec && std::fabs(st.rollK[WheelFR] / kFR - 1.0) < 0.001,
                  "I-21: engine braking does not hold the gate open: a later reset moment is still ignored");
        }
        // The clutch slipping (10 to 90 %) teaches nothing.
        {
            haptics::Layer L; haptics::LawsState st; NcxValues v = fresh();
            v.val[NcxValues::ThrottlePct] = 20.0;
            v.have[NcxValues::ClutchPct] = true; v.val[NcxValues::ClutchPct] = 50.0;
            runFor(L, st, v, 2.0);
            check(!st.rollKnown[WheelFL], "I-21: the clutch slipping teaches nothing");
            v.val[NcxValues::ClutchPct] = 0.0;
            runFor(L, st, v, 2.0);
            check(st.rollKnown[WheelFL], "I-21: ...the clutch up does");
        }
        // Engine braking turns the driven wheels slower than the road: once
        // the driven axle is found (the one running ahead under full
        // throttle), it learns nothing on a closed throttle in gear (the
        // driven rears learned up to 1.2 % high). The free axle still learns.
        {
            haptics::Layer L; haptics::LawsState st; NcxValues v = fresh();
            v.val[NcxValues::ThrottlePct] = 20.0;
            runFor(L, st, v, 2.0);
            v.val[NcxValues::ThrottlePct] = 100.0;
            setWs(v, 1.0, 1.0, 1.02, 1.02);
            runFor(L, st, v, 3.0);
            check(st.drivenSec[1] > haptics::laws_k::kDrivenSec && st.drivenSec[0] == 0.0, "I-21: the driven rears are found under full throttle");
            v.have[NcxValues::Gear] = true; v.val[NcxValues::Gear] = 4.0;
            v.have[NcxValues::ClutchPct] = true; v.val[NcxValues::ClutchPct] = 0.0;
            v.val[NcxValues::ThrottlePct] = 0.0;
            setWs(v, 1.005, 1.005, 0.99, 0.99);
            const double kR = st.rollK[WheelRL], kF = st.rollK[WheelFL];
            runFor(L, st, v, 3.0);
            check(st.rollK[WheelRL] == kR, "I-21: engine braking teaches the driven rears nothing");
            check(st.rollK[WheelFL] < kF * 0.997, "I-21: ...the free fronts still learn");
        }
        // A Sigma at pit exit: both fronts 6 % slow while the first value is
        // taken (they seeded 6 % off), then racing: flat out with the rears
        // 2 % ahead, lifts in gear (engine braking the rears 1.5 %), a
        // little part throttle. The driven rears are found against the
        // cruising lead (the wrong fronts read as driving, and then never
        // learned on a lift), and the fronts are put right within a minute
        // (it took 270 s).
        {
            haptics::Layer L; haptics::LawsState st; NcxValues v = fresh();
            const double truth = 120.0 / 104.0;
            v.val[NcxValues::ThrottlePct] = 8.0;
            setWs(v, 0.94, 0.94, 1.0, 1.0);
            runFor(L, st, v, 2.0);
            check(st.rollKnown[WheelFL] && st.rollK[WheelFL] / truth > 1.05, "I-21 pit exit: the fronts seeded 6 % off");
            v.have[NcxValues::Gear] = true; v.val[NcxValues::Gear] = 3.0;
            v.have[NcxValues::ClutchPct] = true; v.val[NcxValues::ClutchPct] = 0.0;
            for (int lap = 0; lap < 10; ++lap)
            {
                v.val[NcxValues::ThrottlePct] = 100.0; setWs(v, 1.0, 1.0, 1.02, 1.02);   runFor(L, st, v, 3.0);
                v.val[NcxValues::ThrottlePct] = 0.0;   setWs(v, 1.0, 1.0, 0.985, 0.985); runFor(L, st, v, 2.0);
                v.val[NcxValues::ThrottlePct] = 15.0;  setWs(v, 1.0, 1.0, 1.0, 1.0);     runFor(L, st, v, 1.0);
            }
            std::printf("  I-21 pit-exit seed: driven F %.1f s R %.1f s, FL %+.2f %% off after 60 s\n",
                        st.drivenSec[0], st.drivenSec[1], 100.0 * (st.rollK[WheelFL] / truth - 1.0));
            check(st.drivenSec[1] > haptics::laws_k::kDrivenSec && st.drivenSec[0] < st.drivenSec[1], "I-21 pit exit: the rears are found driven, not the wrong fronts");
            check(std::fabs(st.rollK[WheelFL] / truth - 1.0) < 0.01 && std::fabs(st.rollK[WheelRL] / truth - 1.0) < 0.005,
                  "I-21 pit exit: ...and the fronts are put right within a minute, the rears untouched by engine braking");
        }
        // Automobilista 2: a slide before the straight-line baseline is
        // learned teaches nothing (a slide moved the rears 1.3 %).
        {
            haptics::Layer L; haptics::LawsState st; NcxValues v = fresh();
            v.val[NcxValues::ThrottlePct] = 20.0;
            for (int w = 0; w < 4; ++w) { v.have[NcxValues::SlipSpeedFL + w] = true; v.val[NcxValues::SlipSpeedFL + w] = 0.0; }
            runFor(L, st, v, 2.0);
            check(st.rollKnown[WheelRL] && st.latBaseSec < haptics::laws_k::kLatBaseLearnSec, "I-21: Automobilista 2, the rolling factor known, the baseline not yet");
            const double kR = st.rollK[WheelRL];
            setWs(v, 1.0, 1.0, 1.012, 1.012);
            v.val[NcxValues::SlipSpeedRL] = v.val[NcxValues::SlipSpeedRR] = (120.0 / 3.6) * std::sin(8.0 * 3.14159265358979 / 180.0);
            runFor(L, st, v, 3.0);
            // (One cycle in, before the angle is known, it may move a hair.)
            check(std::fabs(st.rollK[WheelRL] / kR - 1.0) < 1e-4, "I-21: ...an 8 deg slide teaches the rolling factor nothing");
        }
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
        // Standing still (a reset or a spawn drops the car onto the ground):
        // no knock; moving off, the first sample is no jump from a stale one;
        // moving, a jump knocks as ever.
        {
            haptics::Layer S; haptics::LawsState ss; S.configureFx(FxType::Impacts, ip); S.configureImpacts(haptics::ImpactParams{});
            NcxValues sv{}; sv.fresh = true;
            sv.have[NcxValues::SpeedKmh] = true; sv.val[NcxValues::SpeedKmh] = 0.0;
            auto svel = [&](double fl, int n) {
                for (int w = 0; w < 4; ++w) { sv.have[NcxValues::SuspVelFL + w] = true; sv.val[NcxValues::SuspVelFL + w] = (w == WheelFL) ? fl : 0.0; }
                for (int i = 0; i < n; ++i) { haptics::driveLaws(S, ss, sv, dt); S.step(dt); } };
            svel(0.0, 100); svel(2000.0, 100);
            check(S.impactModel().knocks(WheelFL) == 0, "I-12 impacts: standing still, a 2 m/s drop (a reset) knocks nothing");
            // Moving off with the corner where it stood: the last sample
            // taken standing is the one it jumps from (were they skipped,
            // the jump from 0 would knock).
            sv.val[NcxValues::SpeedKmh] = 40.0; svel(2000.0, 100);
            check(S.impactModel().knocks(WheelFL) == 0, "I-12 impacts: moving off, no knock from a stale sample");
            svel(0.0, 100);
            check(S.impactModel().knocks(WheelFL) == 1, "I-12 impacts: moving, the jump knocks");
        }

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
