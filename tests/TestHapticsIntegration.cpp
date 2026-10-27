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
#include <memory>
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
    for (int i = 0; i < sd.numNcx; ++i) sd.ncxHave[i] = true;
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

    // ---- I-2b: a belt folded for overspeed takes its haptics down too ----
    // The velocity fold takes a belt's tension away past the knee; the
    // overlay rode over it and could keep pulling on a belt the guard had
    // let go of. Breaks with the beltVelocityFold call on the overlay
    // removed (MotionController's haptics overlay block).
    {
        struct SpinDrive : MockA6Drive { double raw = 0.0; double getActualPositionRaw() const override { return raw; } };
        AppConfig cfg = makeBeltConfig(100.0, 1.0);
        cfg.drives[0].beltMaxRpm      = 200.0;      // folded away by 300 rpm (band = 100)
        cfg.drives[0].beltOverspeedRpm = 100000.0;   // the trips stay out of it: the fold alone
        cfg.drives[0].beltMaxTravelRevs = 1000.0;
        MotionController mc; SpinDrive mock; A6Drive* drives[1] = { &mock };
        mc.configure(cfg);
        mc.setEmergencyStop(false);
        tensionBelt(mc);
        const double unitsPerRev = cfg.drives[0].encoderCountsPerRev / cfg.drives[0].countsPerMm;
        auto spin = [&](double rpm, double skid, int cycles, Span& sp) {
            MotionOutput out{}; sp = Span{};
            for (int i = 0; i < cycles; ++i)
            {
                mock.raw += rpm / 60.0 * unitsPerRev * (1.0 / 500.0);
                out = MotionOutput{};
                mc.process(frame(skid, true), out, drives, 1);
                if (i >= cycles / 2) { sp.lo = std::min(sp.lo, out.torques[0]); sp.hi = std::max(sp.hi, out.torques[0]); }
            }
        };
        Span sp;
        spin(0.0, 0.0, 400, sp);
        check(mc.getAxisState(0) == AxisMotionState::ONLINE, "I-2b belt is ONLINE");
        spin(0.0, 100.0, 300, sp);
        check(sp.hi > 5.0, "I-2b at rest the skid rides on the tension");
        spin(400.0, 100.0, 300, sp);
        std::printf("  I-2b belt folded at 400 rpm with the skid at 100: torque %.3f to %.3f\n", sp.lo, sp.hi);
        check(sp.hi < 1e-6, "I-2b folded for overspeed, the haptics are folded with the tension: no torque at all");
        spin(0.0, 100.0, 300, sp);
        check(sp.hi > 5.0, "I-2b ...and back once the shaft slows");
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
        // The belt's overlay is what would reach the axis: the readouts
        // report 0 for a muted tile by construction and proved nothing
        // (this passed with the mute guard deleted).
        double live = 0.0;
        for (int i = 0; i < 300; ++i) { mc.hapticsIdleTick(frame(100.0, true)); live = std::max(live, std::fabs(mc.hapticsLayer().overlayFor(0))); }
        const double liveLvl = mc.getMotionStatus().hapticsFxLevel[fx];
        MotionCommand f; f.type = MotionCommand::Type::HapticsMuteFx; f.intVal = static_cast<int>(haptics::Effect::Skid) | 0x100;
        mc.enqueueCommand(f);
        double muted = 0.0;
        for (int i = 0; i < 50; ++i) { mc.hapticsIdleTick(frame(100.0, true)); if (i >= 5) muted = std::max(muted, std::fabs(mc.hapticsLayer().overlayFor(0))); }
        check(liveLvl > 0.5 && live > 1.0, "I-5b loop stopped: the skid plays on the idle clock (overlay on the belt)");
        check(muted == 0.0, "I-5b loop stopped: a tile mute reaches the layer on the idle clock: nothing reaches the belt");
    }

    // ---- I-5c: the gear-ratio learner runs on the idle clock too (a
    // shaker-only install never runs the RT loop, and the laws read the
    // ratios it learns); a car the sim moves away from is published once
    // for the cache, with its count stepped ----
    {
        MotionController mc;
        mc.configure(makeBeltConfig(50.0, 1.0));
        // 2 ms cycles: 3rd held at 6000 rpm / 100 km/h for 4 s (confident
        // needs 2 s of agreeing samples after the 0.5 s hold).
        TelemetryData td = frame(0.0, true);
        td.ncx[static_cast<int>(NcxValues::Rpm)]      = 6000.0;
        td.ncx[static_cast<int>(NcxValues::SpeedKmh)] = 100.0;
        td.ncx[static_cast<int>(NcxValues::Gear)]     = 3.0;
        std::snprintf(td.car, NCY_STR_LEN, "car_a");
        for (int i = 0; i < 2000; ++i) mc.hapticsIdleTick(td);
        // Then 2nd at the same speed.
        td.ncx[static_cast<int>(NcxValues::Rpm)]  = 8000.0;
        td.ncx[static_cast<int>(NcxValues::Gear)] = 2.0;
        for (int i = 0; i < 2000; ++i) mc.hapticsIdleTick(td);
        MotionStatus ms = mc.getMotionStatus();
        check(ms.gearRatioConfident[3] && std::fabs(ms.gearRatio[3] - 60.0) < 1.0
              && ms.gearRatioConfident[2] && std::fabs(ms.gearRatio[2] - 80.0) < 1.0,
              "I-5c loop stopped: the learner learns on the idle clock (3rd 60, 2nd 80 rpm per km/h, confident)");
        check(ms.gearRatioPrevSeq == 0, "I-5c: no car finished yet: the finished count is 0");

        // The sim names another car: car_a's set is published for the
        // cache once (the count steps once), the slots start over.
        std::snprintf(td.car, NCY_STR_LEN, "car_b");
        td.ncx[static_cast<int>(NcxValues::Rpm)]  = 5000.0;
        td.ncx[static_cast<int>(NcxValues::Gear)] = 4.0;
        for (int i = 0; i < 50; ++i) mc.hapticsIdleTick(td);
        ms = mc.getMotionStatus();
        check(ms.gearRatioPrevSeq == 1 && ms.gearRatiosPrevDirty
              && ms.gearRatioPrevConfident[3] && std::fabs(ms.gearRatioPrev[3] - 60.0) < 1.0
              && ms.gearRatioPrevConfident[2] && std::fabs(ms.gearRatioPrev[2] - 80.0) < 1.0,
              "I-5c car change: the finished car's confident set is published with the count at 1");
        check(!ms.gearRatioConfident[3] && !ms.gearRatioConfident[2],
              "I-5c car change: the new car starts with no confident gear");
        for (int i = 0; i < 2000; ++i) mc.hapticsIdleTick(td);
        ms = mc.getMotionStatus();
        check(ms.gearRatioPrevSeq == 1, "I-5c: driving the new car does not step the finished count again");
    }

    // ---- I-5d: a car driven earlier in the session (A, then B, then A
    // again) adopts what it taught: two of its gears driven again bring
    // the third back without driving it (the learner's cache was only the
    // startup copy, so A relearned every gear from nothing) ----
    {
        MotionController mc;
        mc.configure(makeBeltConfig(50.0, 1.0));
        TelemetryData td = frame(0.0, true);
        td.ncx[static_cast<int>(NcxValues::SpeedKmh)] = 100.0;
        const auto drive = [&](const char* car, double gear, double rpm, int cycles)
        {
            std::snprintf(td.car, NCY_STR_LEN, "%s", car);
            td.ncx[static_cast<int>(NcxValues::Gear)] = gear;
            td.ncx[static_cast<int>(NcxValues::Rpm)]  = rpm;
            for (int i = 0; i < cycles; ++i) mc.hapticsIdleTick(td);
        };
        drive("car_a", 2.0, 8000.0, 2000);   // 80 rpm per km/h
        drive("car_a", 3.0, 6000.0, 2000);   // 60
        drive("car_a", 4.0, 4500.0, 2000);   // 45
        MotionStatus ms = mc.getMotionStatus();
        check(ms.gearRatioConfident[2] && ms.gearRatioConfident[3] && ms.gearRatioConfident[4],
              "I-5d car A: 2nd, 3rd and 4th learned");
        drive("car_b", 5.0, 3000.0, 2000);   // another car: A is finished
        ms = mc.getMotionStatus();
        check(!ms.gearRatioKnown[4], "I-5d car B: A's 4th is not B's");
        drive("car_a", 2.0, 8000.0, 2000);   // A again: 2nd and 3rd only
        drive("car_a", 3.0, 6000.0, 2000);
        ms = mc.getMotionStatus();
        check(ms.gearRatioKnown[4] && std::fabs(ms.gearRatio[4] - 45.0) < 1.0,
              "I-5d car A again: its 4th comes back from this session without being driven");
    }

    // ---- I-5e: an e-stop silences the shakers with the loop stopped too
    // (the idle clock stepped the layer as if there were none) ----
    {
        MotionController mc;
        mc.configure(makeBeltConfig(50.0, 1.0));
        double live = 0.0, stopped = 0.0;
        for (int i = 0; i < 300; ++i) { mc.hapticsIdleTick(frame(100.0, true)); live = std::max(live, std::fabs(mc.hapticsLayer().overlayFor(0))); }
        mc.setEmergencyStop(true);
        for (int i = 0; i < 50; ++i) { mc.hapticsIdleTick(frame(100.0, true)); if (i >= 1) stopped = std::max(stopped, std::fabs(mc.hapticsLayer().overlayFor(0))); }
        mc.setEmergencyStop(false);
        check(live > 1.0 && stopped == 0.0, "I-5e loop stopped: an e-stop clears the layer on the idle clock");
    }

    // ---- I-5f: the axis delay applies from configure(), not only from a
    // live apply (after every start or Initialize the axes played
    // undelayed until the next save) ----
    {
        AppConfig cfg = makeBeltConfig(50.0, 1.0);   // 500 Hz: 2 ms a cycle
        cfg.hapticsAxisDelayMs = 12.0;
        MotionController mc;
        mc.configure(cfg);
        check(mc.hapticsLayer().axisDelayCycles() == 6, "I-5f configure(): axis delay 12 ms is 6 cycles at 500 Hz");
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
        v.val[NcxValues::SpeedKmh] = 100.0; st.lastSpeedKmh = 100.0;   // the setup's jump is no glitch (the hold would drop the suspension below)
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
            // With the Engine tile's amplitude at 0 the redline still learns
            // (the Driveline's whine and lug read it; a 15,000 rpm engine
            // lugged between 840 and 2,100 rpm on the 7,000 guess). Breaks
            // with the learnRedline call in the Layer removed.
            {
                haptics::EffectParams off = ep; off.ampPct = 0.0; L.configureFx(FxType::RpmVibe, off);
                std::strncpy(v.car, "Silent", NCY_STR_LEN - 1);
                v.have[NcxValues::MaxRpm] = false; v.val[NcxValues::Rpm] = 900.0; settleE(10);
                v.val[NcxValues::Rpm] = 12000.0; settleE(600);
                check(std::fabs(L.learnedMaxRpm() - 12000.0) < 1.0, "I-8 engine: the redline is learned with the tile's amplitude at 0");
                L.configureFx(FxType::RpmVibe, ep);
            }
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
        // The stream stopping (a pause, a menu, a dropout) is not the engine
        // stopping: the output fades on the release ramp, no stall kick (it
        // kicked at 1.5x on every pause: the law drove 0 rpm, the model read
        // a stall). The sim reporting 0 rpm while live is a stall and kicks.
        // Breaks with HapticLaws.h's engineHoldHz reverted to driving 0.
        {
            haptics::Layer L; haptics::LawsState st;
            for (int t = 0; t < NcxValues::TokenCount; ++t) v.have[t] = false;
            haptics::EffectParams ep; ep.ampPct = 50.0; ep.freqHz = 30.0; ep.routes[0] = { 0, 1.0 };
            haptics::EngineParams eg; eg.cylinders = 8; eg.litres = 4.0; eg.maxRpm = 7000.0; eg.inertia = 0.0; eg.liftoff = 0.0;
            L.configureFx(FxType::RpmVibe, ep); L.configureEngine(eg);
            v.have[NcxValues::Rpm] = true; v.val[NcxValues::Rpm] = 3000.0;
            v.have[NcxValues::ThrottlePct] = true; v.val[NcxValues::ThrottlePct] = 30.0;
            for (int i = 0; i < 1000; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
            check(L.engineRunning() && L.engineStalls() == 0, "I-8 engine drop: running at 3000 rpm");
            double atDrop = 0.0;
            for (int i = 0; i < 20; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); atDrop = std::max(atDrop, std::fabs(L.overlayFor(0))); }
            v.fresh = false;   // the stream stops
            double peakAfter = 0.0, lvlEnd = 1.0;
            for (int i = 0; i < 300; ++i)
            {
                haptics::driveLaws(L, st, v, dt); L.step(dt);
                peakAfter = std::max(peakAfter, std::fabs(L.overlayFor(0)));
                if (i == 299) lvlEnd = L.fxLevel(static_cast<int>(FxType::RpmVibe));
            }
            std::printf("  I-8 engine drop: peak %.1f before, %.1f in the 0.6 s after, level at the end %.3f, stalls %llu\n",
                        atDrop, peakAfter, lvlEnd, (unsigned long long)L.engineStalls());
            check(L.engineStalls() == 0, "I-8 engine drop: the stream stopping is no stall");
            check(peakAfter <= atDrop * 1.05, "I-8 engine drop: ...nothing louder than before plays after it");
            check(lvlEnd < 0.01, "I-8 engine drop: ...and it has faded out");
            // Back and running, then the sim says the engine is off: that is a
            // stall, and it kicks.
            v.fresh = true; v.val[NcxValues::Rpm] = 3000.0;
            for (int i = 0; i < 500; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
            v.val[NcxValues::Rpm] = 0.0;
            double kick = 0.0;
            for (int i = 0; i < 100; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); kick = std::max(kick, std::fabs(L.overlayFor(0))); }
            check(L.engineStalls() == 1 && !L.engineRunning() && kick > atDrop, "I-8 engine drop: the sim reporting 0 rpm while live is a stall, and it kicks");
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
            // An electric motor pulls from a standstill: no lugging, ever
            // (it shuddered at lug hz on every pull-away). Breaks with the
            // electric test in the lug law removed.
            haptics::EngineParams ev = eg; ev.layout = 5.0; L.configureEngine(ev);
            set(NcxValues::Gear, 1.0); set(NcxValues::Rpm, 1200.0); settleD(300);
            check(lug() < 0.02, "I-8 driveline: an electric car does not lug");
            L.configureEngine(eg);
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
        st.lastSpeedKmh = v.val[NcxValues::SpeedKmh];   // the blocks above moved v's speed on their own state: no glitch here
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
        check(L.surfaceModel().stonesStruck(WheelFL) > 10 && L.surfaceModel().stonesStruck(WheelFR) == 0,
              "I-10 surface: the gravel under FL strikes stones, the tarmac under FR none");
        check(pk0 > 1.0 && pk1 < 1e-9, "I-10 surface: FL's route plays them, FR's is silent");

        // No surface type from the sim: the ground the tile assumes. Gravel
        // strikes stones on every tyre; the default (tarmac) none; a sim
        // that says tarmac wins over an assumed gravel.
        {
            haptics::Layer A; haptics::LawsState as;
            A.configureFx(FxType::Surface, sp);
            haptics::SurfaceParams q; q.assume = 1.0;   // gravel
            A.configureSurface(q);
            NcxValues n{}; n.fresh = true;
            n.have[NcxValues::SpeedKmh] = true; n.val[NcxValues::SpeedKmh] = 72.0;
            for (int i = 0; i < 1500; ++i) { haptics::driveLaws(A, as, n, dt); A.step(dt); }
            check(A.surfaceModel().stonesStruck(WheelFL) > 10 && A.surfaceModel().stonesStruck(WheelRR) > 10,
                  "I-10 surface: no surface sent, gravel assumed: stones on every tyre");
            haptics::Layer T; haptics::LawsState ts;
            T.configureFx(FxType::Surface, sp);
            for (int i = 0; i < 1500; ++i) { haptics::driveLaws(T, ts, n, dt); T.step(dt); }
            check(T.surfaceModel().stonesStruck(WheelFL) == 0, "I-10 surface: no surface sent, nothing assumed: tarmac, no stones");
            haptics::Layer S; haptics::LawsState ss;
            S.configureFx(FxType::Surface, sp); S.configureSurface(q);
            for (int w = 0; w < 4; ++w) { n.have[NcxValues::SurfaceFL + w] = true; n.val[NcxValues::SurfaceFL + w] = SurfTarmac; }
            for (int i = 0; i < 1500; ++i) { haptics::driveLaws(S, ss, n, dt); S.step(dt); }
            check(S.surfaceModel().stonesStruck(WheelFL) == 0, "I-10 surface: the sim saying tarmac wins over an assumed gravel");
        }
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
            for (int i = 0; i < 340; ++i) { haptics::driveLaws(P, ps, none, dt); P.step(dt); }
            check(P.surfaceModel().stonesStruck(WheelFL) >= 2, "I-10 surface preview: gravel first, stones struck");
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
        // Only one wheel of an axle with a rolling factor (the other's
        // samples never agreed): the pair is used as a pair or not at all.
        // Half-scaled, a straight read as a 10 % split and the diff clunked
        // out of every pit lane. Breaks with the "both" test reverted.
        d.diffType = 0.0; L.configureDriveline(d);
        for (int w = 0; w < 4; ++w) { st.rollK[w] = 1.1; st.rollKnown[w] = true; }
        st.rollKnown[WheelRR] = false;   // the sender's raw unit on one side
        corner(10.0, 0.0, 0.0); listen(0.5, n, spread, pk); listen(1.0, n, spread, pk);
        check(n == 0, "I-20 diff: one wheel of the axle without a rolling factor: no false split on a straight");
        for (int w = 0; w < 4; ++w) st.rollKnown[w] = false;
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

    // ---- I-30: each tile's trigger count, and the road heights' rate ----
    {
        MotionController mc;
        AppConfig cfg = makeBeltConfig(50.0, 1.0);   // 500 Hz: a cycle is 2 ms
        haptics::EffectParams& gs = cfg.hapticsFx[static_cast<size_t>(haptics::Effect::GearShift)];
        gs.ampPct = 50.0; gs.routes[0] = { 0, 1.0 };   // a fire only counts when it plays
        mc.configure(cfg);
        mc.setEmergencyStop(false);
        tensionBelt(mc);
        run(mc, frame(0.0, true), 400, 1);
        mc.requestHapticPeakReset();
        run(mc, frame(0.0, true), 10, 1);
        const int skid = static_cast<int>(haptics::Effect::Skid), gear = static_cast<int>(haptics::Effect::GearShift);
        MotionStatus ms = mc.getMotionStatus();
        check(ms.hapticsCount[skid] == 0 && ms.hapticsAgoSec[skid] < 0.0, "I-30: no slide yet: none counted, none ago");
        // Three slides, each followed by grip: three triggers, the last
        // begun 1.8 s ago (its 0.6 s and the 1.2 s of grip after).
        for (int k = 0; k < 3; ++k) { run(mc, frame(60.0, true), 300, 1); run(mc, frame(0.0, true), 600, 1); }
        ms = mc.getMotionStatus();
        check(ms.hapticsCount[skid] == 3, "I-30: three slides, three triggers");
        check(ms.hapticsAgoSec[skid] > 1.6 && ms.hapticsAgoSec[skid] < 2.0, "I-30: the last slide began 1.8 s ago");
        // A Test is no trigger: the continuous preview, and a transient's fire.
        MotionCommand t; t.type = MotionCommand::Type::HapticsTest; t.intVal = skid;
        mc.enqueueCommand(t);
        run(mc, frame(0.0, true), 8000, 1);
        t.intVal = gear; mc.enqueueCommand(t);
        run(mc, frame(0.0, true), 50, 1);
        ms = mc.getMotionStatus();
        check(ms.hapticsCount[skid] == 3 && ms.hapticsCount[gear] == 0, "I-30: a Test (a preview, a fire) is no trigger");
        // A transient's own fires count, each one.
        auto& L = const_cast<haptics::Layer&>(mc.hapticsLayer());
        L.fire(haptics::EventType::GearShift); run(mc, frame(0.0, true), 5, 1);
        L.fire(haptics::EventType::GearShift); run(mc, frame(0.0, true), 5, 1);
        ms = mc.getMotionStatus();
        check(ms.hapticsCount[gear] == 2 && ms.hapticsAgoSec[gear] >= 0.0 && ms.hapticsAgoSec[gear] < 0.05, "I-30: two shifts, two triggers, the last just now");
        // Reset peaks counts from then.
        mc.requestHapticPeakReset();
        run(mc, frame(0.0, true), 10, 1);
        ms = mc.getMotionStatus();
        check(ms.hapticsCount[skid] == 0 && ms.hapticsCount[gear] == 0 && ms.hapticsAgoSec[skid] < 0.0, "I-30: Reset peaks starts the counts over");
        // Only what plays counts: a muted tile's slide is none (as a muted
        // transient's fire is none), and once unmuted the next one counts.
        auto muteFx = [&](int id, bool on) { MotionCommand m; m.type = MotionCommand::Type::HapticsMuteFx; m.intVal = id | (on ? 0x100 : 0); mc.enqueueCommand(m); };
        muteFx(skid, true);
        run(mc, frame(60.0, true), 300, 1); run(mc, frame(0.0, true), 600, 1);
        check(mc.getMotionStatus().hapticsCount[skid] == 0, "I-30: a muted tile's slide is no trigger");
        muteFx(skid, false);
        run(mc, frame(60.0, true), 300, 1); run(mc, frame(0.0, true), 600, 1);
        check(mc.getMotionStatus().hapticsCount[skid] == 1, "I-30: unmuted, the next slide counts");
        // A Test that cannot play (its tile muted in the same cycle) takes
        // nothing off: a real fire counted in that cycle still counts.
        L.fire(haptics::EventType::GearShift);       // played: the tile is not muted yet
        muteFx(gear, true);
        t.intVal = gear; mc.enqueueCommand(t);       // the next cycle mutes it, then the Test cannot play
        run(mc, frame(0.0, true), 5, 1);
        muteFx(gear, false);
        run(mc, frame(0.0, true), 5, 1);
        check(mc.getMotionStatus().hapticsCount[gear] == 1, "I-30: a Test that could not play leaves a real fire in its cycle counted");
        // Car A then car B: the counts are the car's, B starts at none.
        TelemetryData ca = frame(60.0, true), ga = frame(0.0, true);
        std::strcpy(ca.car, "Car A"); std::strcpy(ga.car, "Car A");
        run(mc, ga, 50, 1);
        for (int k = 0; k < 2; ++k) { run(mc, ca, 300, 1); run(mc, ga, 600, 1); }
        check(mc.getMotionStatus().hapticsCount[skid] == 3, "I-30: car A's two slides count (on top of the last)");
        TelemetryData cb = ca, gb = ga;
        std::strcpy(cb.car, "Car B"); std::strcpy(gb.car, "Car B");
        run(mc, gb, 50, 1);
        ms = mc.getMotionStatus();
        check(ms.hapticsCount[skid] == 0 && ms.hapticsAgoSec[skid] < 0.0, "I-30: car B starts with none counted");
        run(mc, cb, 300, 1); run(mc, gb, 600, 1);
        check(mc.getMotionStatus().hapticsCount[skid] == 1, "I-30: car B's own slide counts");

        // The road heights' rate while moving: a new value every 9th cycle
        // reads about 55 a second (SimHub's), every cycle 500; none sent, 0.
        TelemetryData sd = frame(0.0, true);
        for (int w = 0; w < 4; ++w) sd.ncyHave[NcxTok::RoadHeightFL + w] = true;
        auto heights = [&](int every, int cycles)
        {
            for (int i = 0; i < cycles; ++i)
            {
                if (i % every == 0) for (int w = 0; w < 4; ++w) sd.ncy[NcxTok::RoadHeightFL + w] = 10000.0 + i;
                MotionOutput out{}; mc.process(sd, out, nullptr, 0);
            }
            return mc.getMotionStatus().hapHeightsHz;
        };
        const double slow = heights(9, 1500);
        check(std::fabs(slow - 500.0 / 9.0) < 3.0, "I-30: road heights arriving 55 a second read 55");
        const double fast = heights(1, 1500);
        check(std::fabs(fast - 500.0) < 5.0, "I-30: a new height every cycle reads 500");
        // Stopped, the last rate stands (a parked car's height does not move).
        sd.ncx[static_cast<int>(NcxValues::SpeedKmh)] = 0.0;
        for (int i = 0; i < 1500; ++i) { MotionOutput out{}; mc.process(sd, out, nullptr, 0); }
        check(std::fabs(mc.getMotionStatus().hapHeightsHz - 500.0) < 5.0, "I-30: stopped, the last rate stands");
        sd.ncx[static_cast<int>(NcxValues::SpeedKmh)] = 80.0;
        // One frame without heights is no stop; half a second without is.
        for (int w = 0; w < 4; ++w) sd.ncyHave[NcxTok::RoadHeightFL + w] = false;
        { MotionOutput out{}; mc.process(sd, out, nullptr, 0); }
        check(std::fabs(mc.getMotionStatus().hapHeightsHz - 500.0) < 5.0, "I-30: a single frame without heights keeps the rate");
        for (int i = 0; i < 300; ++i) { MotionOutput out{}; mc.process(sd, out, nullptr, 0); }
        check(mc.getMotionStatus().hapHeightsHz == 0.0, "I-30: no heights for half a second reads 0");
        std::printf("  I-30: heights %.1f / %.1f a second\n", slow, fast);
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
        // A wheel just outside the gate, its samples now in, now out (a
        // value learned 1.9 % off, the samples scattering 1.2 to 2.2 %):
        // the time it agrees counts back down instead of starting over, so
        // it still gets there (it stalled for minutes), and once open it
        // learns until well inside.
        {
            haptics::Layer L; haptics::LawsState st; NcxValues v = fresh();
            v.val[NcxValues::ThrottlePct] = 20.0;
            runFor(L, st, v, 2.0);
            const double k0 = st.rollK[WheelFR];
            for (int i = 0; i < 400; ++i)   // 40 s: 70 % of the samples 2.2 % off, 30 % 1.2 %
            {
                setWs(v, 1.0, (i % 10 < 7) ? 1.022 : 1.012, 1.0, 1.0);
                runFor(L, st, v, 0.1);
            }
            const double want = k0 / (0.7 * 1.022 + 0.3 * 1.012);
            std::printf("  I-21 hovering just outside the gate: FR %+.2f %% off the scatter's mean after 40 s\n", 100.0 * (st.rollK[WheelFR] / want - 1.0));
            check(std::fabs(st.rollK[WheelFR] / want - 1.0) < 0.006 && !st.rollOpen, "I-21: a wheel hovering just outside the gate is still put right, and the gate closes again");
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

    // ---- I-22: a different car starts everything over, quietly ----
    // Car A driven hard (4th, 6000 rpm, throttle, rain, learned ratios and
    // wheel sizes), then the sim names car B in 1st with its engine off (0
    // rpm, as a car spawns) and the throttle shut: no gear knock, no shunt,
    // no stall kick, no puddles, and every learner fresh. Breaks with
    // newCar(st) reverted to the old hand-kept list (the gear edge, the
    // throttle crossing and the water film carried over) or
    // EngineModel::forgetCar not clearing running (car A's running engine
    // meeting car B's 0 rpm reads as a stall).
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.002;
        haptics::Layer L; haptics::LawsState st;
        haptics::EffectParams fx; fx.ampPct = 100.0; fx.routes[0] = { 0, 1.0 };
        L.configure(haptics::EventType::GearShift, fx); L.configureFx(FxType::Driveline, fx);
        L.configureFx(FxType::RpmVibe, fx); L.configureFx(FxType::Surface, fx); L.configureFx(FxType::Lockup, fx);
        haptics::DrivelineParams dp; dp.shunt = 1.0; dp.gearbox = 1.0; L.configureDriveline(dp);
        haptics::EngineParams eg; eg.maxRpm = 0.0; L.configureEngine(eg);
        haptics::SurfaceParams sq; sq.puddles = 1.0; L.configureSurface(sq);
        NcxValues v{}; v.fresh = true;
        auto set = [&](NcxValues::Token t, double x) { v.have[t] = true; v.val[t] = x; };
        auto run = [&](double secs) { for (int i = 0; i < static_cast<int>(secs / dt); ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); } };
        std::strncpy(v.car, "Car A", NCY_STR_LEN - 1);
        set(NcxValues::Rpm, 6000.0); set(NcxValues::Gear, 4.0); set(NcxValues::ThrottlePct, 100.0);
        set(NcxValues::SpeedKmh, 150.0); set(NcxValues::ClutchPct, 0.0); set(NcxValues::BrakePct, 0.0);
        set(NcxValues::Rain, 1.0);
        for (int w = 0; w < 4; ++w) { set(static_cast<NcxValues::Token>(NcxValues::WheelSpeedFL + w), 150.0 / 3.6 / 0.33); }
        run(3.0);
        set(NcxValues::ThrottlePct, 20.0); run(3.0);   // cruising: the wheel sizes learn
        check(st.gearSeen && st.rollKnown[WheelFL] && st.waterFilm > 0.0 && L.engineRunning(),
              "I-22: car A learned (gear seen, rolling factor, a water film, the engine running)");
        const uint64_t knocks = L.fireCount(haptics::EventType::GearShift), stalls = L.engineStalls();
        const double shuntBefore = L.drivelineShuntLevel();
        // Car B: 1st, engine off, throttle shut, speed and wheels at a crawl.
        std::strncpy(v.car, "Car B", NCY_STR_LEN - 1);
        set(NcxValues::Rpm, 0.0); set(NcxValues::Gear, 1.0); set(NcxValues::ThrottlePct, 0.0);
        set(NcxValues::SpeedKmh, 5.0); st.lastSpeedKmh = 5.0;
        for (int w = 0; w < 4; ++w) v.val[NcxValues::WheelSpeedFL + w] = 5.0 / 3.6 / 0.33;
        double shuntPeak = 0.0, filmAfterChange = 1.0;
        for (int i = 0; i < static_cast<int>(1.0 / dt); ++i)
        {
            haptics::driveLaws(L, st, v, dt); L.step(dt);
            if (i == 0) filmAfterChange = st.waterFilm;   // (the rain still falls: it builds again from here)
            shuntPeak = std::max(shuntPeak, L.drivelineShuntLevel());
        }
        check(L.fireCount(haptics::EventType::GearShift) == knocks, "I-22: no gear knock on the car change");
        check(L.engineStalls() == stalls, "I-22: no stall kick on the car change");
        check(shuntPeak <= shuntBefore + 1e-9 && shuntPeak < 0.05, "I-22: no shunt from the old car's throttle on the car change");
        check(filmAfterChange < 0.01, "I-22: the water film starts over");
        check(!st.rollKnown[WheelFL] && st.latBaseSec == 0.0 && st.loadRef == 0.0 && !st.absFlagSeen && st.drivenSec[1] == 0.0,
              "I-22: the learners start over (rolling factor, baseline, usual load, ABS flag, driven axle)");
    }

    // ---- I-22b: a car change that lands somewhere else in the same sample
    // (a new car put in the pits) is still a speed glitch: newCar keeps the
    // road-speed tracker. Breaks with the tracker left out of newCar's
    // copy-back (a fresh state seeds on the jump and sees none) ----
    {
        const double dt = 0.002;
        haptics::Layer L; haptics::LawsState st;
        NcxValues v{}; v.fresh = true;
        v.have[NcxValues::SpeedKmh] = true; v.val[NcxValues::SpeedKmh] = 150.0;
        std::strncpy(v.car, "Car A", NCY_STR_LEN - 1);
        for (int i = 0; i < static_cast<int>(1.0 / dt); ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
        check(st.glitchSec == 0.0, "I-22b: car A at a steady 150 km/h: no glitch");
        std::strncpy(v.car, "Car B", NCY_STR_LEN - 1);
        v.val[NcxValues::SpeedKmh] = 0.0;
        haptics::driveLaws(L, st, v, dt); L.step(dt);
        check(st.glitchSec > 0.0, "I-22b: car B in the pits in the same sample: the speed jump is a glitch");
    }

    // ---- I-23: a reset's jump plays no road pop and no impact knock ----
    // Breaks with the glitch hold's token list reverted to speed and body
    // only (HapticLaws.h: SuspVel*/SuspTravel*/RoadHeight* were left in).
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.002;
        haptics::Layer L; haptics::LawsState st;
        haptics::EffectParams rp; rp.ampPct = 100.0; rp.routes[0] = { 0, 1.0, Part::All };
        haptics::EffectParams ip; ip.ampPct = 100.0; ip.freqHz = 40.0; ip.durMs = 50.0; ip.routes[0] = { 1, 1.0, Part::All };
        L.configureFx(FxType::Road, rp); L.configureFx(FxType::Impacts, ip);
        haptics::RoadParams rq; rq.model = 0; rq.rough = 0.0; rq.fullMm = 25.0; L.configureRoad(rq);
        L.configureImpacts(haptics::ImpactParams{});
        NcxValues v{}; v.fresh = true;
        v.have[NcxValues::SpeedKmh] = true; v.val[NcxValues::SpeedKmh] = 120.0;
        for (int w = 0; w < 4; ++w) { v.have[NcxValues::SuspTravelFL + w] = true; v.val[NcxValues::SuspTravelFL + w] = 0.0; }
        auto run = [&](double secs, double& roadPk) { for (int i = 0; i < static_cast<int>(secs / dt); ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); roadPk = std::max(roadPk, std::fabs(L.overlayFor(0))); } };
        double pk = 0.0; run(1.0, pk);
        const uint32_t knocks = L.impactModel().knocks(WheelFL);
        // The reset: the car lands in the pits at 0 km/h with the suspension
        // 60 mm from where it was, in one sample.
        v.val[NcxValues::SpeedKmh] = 0.0;
        for (int w = 0; w < 4; ++w) v.val[NcxValues::SuspTravelFL + w] = 60.0;
        pk = 0.0; run(0.4, pk);
        std::printf("  I-23 reset: road peak after the jump %.2f, knocks %u (was %u)\n", pk, L.impactModel().knocks(WheelFL), knocks);
        check(st.glitchSec > 0.0, "I-23: the jump to the pits is a glitch");
        check(pk < 1.0, "I-23: the suspension jump plays no road pop");
        check(L.impactModel().knocks(WheelFL) == knocks, "I-23: ...and no impact knock");
    }

    // ---- I-23b: the same jump at speed. I-23 lands at 0 km/h, where
    // impacts are off anyway, and its travel had sat still long enough to
    // count as no sample: its knock check passes either way. Here the car
    // is still moving after the jump (120 to 60 km/h in one sample: a
    // glitch) and the suspension is live up to it. Breaks with the glitch
    // hold's token list reverted (the 60 mm step knocks every corner) ----
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.002;
        haptics::Layer L; haptics::LawsState st;
        haptics::EffectParams ip; ip.ampPct = 100.0; ip.freqHz = 40.0; ip.durMs = 50.0; ip.routes[0] = { 0, 1.0, Part::All };
        L.configureFx(FxType::Impacts, ip);
        L.configureImpacts(haptics::ImpactParams{});
        NcxValues v{}; v.fresh = true;
        v.have[NcxValues::SpeedKmh] = true; v.val[NcxValues::SpeedKmh] = 120.0;
        for (int w = 0; w < 4; ++w) v.have[NcxValues::SuspTravelFL + w] = true;
        double t = 0.0;
        for (int i = 0; i < static_cast<int>(1.0 / dt); ++i, t += dt)
        {
            for (int w = 0; w < 4; ++w) v.val[NcxValues::SuspTravelFL + w] = 2.0 * std::sin(2.0 * 3.14159265358979 * 3.0 * t);   // a gentle live road
            haptics::driveLaws(L, st, v, dt); L.step(dt);
        }
        uint32_t before = 0;
        for (int w = 0; w < 4; ++w) before += L.impactModel().knocks(w);
        v.val[NcxValues::SpeedKmh] = 60.0;
        for (int i = 0; i < static_cast<int>(0.4 / dt); ++i)
        {
            for (int w = 0; w < 4; ++w) v.val[NcxValues::SuspTravelFL + w] = 60.0 + 0.1 * std::sin(2.0 * 3.14159265358979 * 3.0 * (t += dt));
            haptics::driveLaws(L, st, v, dt); L.step(dt);
        }
        uint32_t after = 0;
        for (int w = 0; w < 4; ++w) after += L.impactModel().knocks(w);
        check(st.glitchSec > 0.0, "I-23b: 120 to 60 km/h in one sample is a glitch");
        check(after == before, "I-23b: the suspension's 60 mm jump at speed plays no impact knock");
    }

    // ---- I-8c: the Engine tile's Test with the sim gone runs the canned
    // idle, not the rate held from the last stream (quit at 7000 rpm, the
    // preview played at 7000) ----
    {
        using haptics::FxType;
        const double dt = 0.002;
        haptics::Layer L; haptics::LawsState st;
        haptics::EffectParams fx; fx.ampPct = 100.0; fx.routes[0] = { 0, 1.0 };
        L.configureFx(FxType::RpmVibe, fx);
        NcxValues v{}; v.fresh = true;
        v.have[NcxValues::Rpm] = true; v.val[NcxValues::Rpm] = 7000.0;
        for (int i = 0; i < static_cast<int>(1.0 / dt); ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
        const double hzLive = L.engineFireHz();
        v.fresh = false;   // the sim quits
        for (int i = 0; i < static_cast<int>(1.0 / dt); ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
        haptics::startPreview(st, FxType::RpmVibe);
        for (int i = 0; i < 20; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
        const double hzTest = L.engineFireHz();
        std::printf("  I-8c engine firing rate: live %.1f Hz, Test with the sim gone %.1f Hz\n", hzLive, hzTest);
        check(hzLive > 100.0 && hzTest > 0.0 && hzTest < 0.3 * hzLive,
              "I-8c the Engine Test with the stream gone plays the canned idle, not the held 7000 rpm");
    }

    // ---- I-8d: the Engine Test with no stream sweeps from the canned idle
    // to the redline, and teaches the redline nothing (a truck tested
    // before its max rpm arrived used to be left with the 7000 guess) ----
    {
        using haptics::FxType;
        const double dt = 0.002;
        const double perRev = 4.0;   // a V8 four-stroke: four firings per crank rev
        double hzAt10 = 0.0; bool cutAt10 = true, cutAt18 = false;
        auto sweep = [&](double maxRpm, double& hzAt01, double& hzAt18, haptics::Layer& L)
        {
            haptics::LawsState st;
            haptics::EffectParams fx; fx.ampPct = 100.0; fx.routes[0] = { 0, 1.0 };
            L.configureFx(FxType::RpmVibe, fx);
            haptics::EngineParams ep; ep.cylinders = 8.0; ep.layout = 1.0; ep.maxRpm = maxRpm;
            L.configureEngine(ep);
            NcxValues v{}; v.fresh = false;   // no sim
            haptics::startPreview(st, FxType::RpmVibe);
            const int n = static_cast<int>(2.0 / dt);
            for (int i = 0; i < n; ++i)
            {
                haptics::driveLaws(L, st, v, dt); L.step(dt);
                if (i == static_cast<int>(0.1 / dt)) hzAt01 = L.engineFireHz();
                if (i == static_cast<int>(1.0 / dt)) { hzAt10 = L.engineFireHz(); cutAt10 = L.engineLimiterOn(); }
                if (i == static_cast<int>(1.8 / dt)) { hzAt18 = L.engineFireHz(); cutAt18 = L.engineLimiterOn(); }
            }
        };
        {
            haptics::Layer L; double a = 0.0, b = 0.0;
            const double seed = L.learnedMaxRpm();
            sweep(0.0, a, b, L);
            std::printf("  I-8d engine Test sweep: %.1f Hz at 0.1 s, %.1f Hz at 1.8 s (redline guess %.0f rpm)\n", a, b, seed);
            check(std::fabs(a - 1100.0 / 60.0 * perRev) < 0.01, "I-8d the Engine Test opens on the canned idle");
            check(std::fabs(b - seed / 60.0 * perRev) < 0.01, "I-8d ...and ends at the redline it has (the guess before anything is seen)");
            check(std::fabs(hzAt10 - (1100.0 + 0.5 * (seed - 1100.0)) / 60.0 * perRev) < 0.05,
                  "I-8d ...climbing straight between them (half way at 1.0 s)");
            check(!cutAt10 && cutAt18, "I-8d ...on the cut only at the top");
            check(std::fabs(L.learnedMaxRpm() - seed) < 1e-9, "I-8d the sweep teaches the redline nothing");
            L.seedMaxRpm(3700.0);
            check(std::fabs(L.learnedMaxRpm() - 3700.0) < 1e-9,
                  "I-8d a truck's max rpm arriving after the Test still seeds its own redline");
        }
        {
            haptics::Layer L; double a = 0.0, b = 0.0;
            sweep(5200.0, a, b, L);
            check(std::fabs(b - 5200.0 / 60.0 * perRev) < 0.01, "I-8d the sweep ends at the tile's own redline when one is set");
        }
        {
            // With live rpm the Test plays the live engine, as before.
            haptics::Layer L; haptics::LawsState st;
            haptics::EffectParams fx; fx.ampPct = 100.0; fx.routes[0] = { 0, 1.0 };
            L.configureFx(FxType::RpmVibe, fx);
            haptics::EngineParams ep; ep.cylinders = 8.0; ep.layout = 1.0; L.configureEngine(ep);
            NcxValues v{}; v.fresh = true;
            v.have[NcxValues::Rpm] = true; v.val[NcxValues::Rpm] = 3000.0;
            for (int i = 0; i < 200; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
            haptics::startPreview(st, FxType::RpmVibe);
            for (int i = 0; i < static_cast<int>(1.8 / dt); ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
            check(std::fabs(L.engineFireHz() - 3000.0 / 60.0 * perRev) < 0.01,
                  "I-8d a Test with live rpm plays the live engine, not the sweep");
        }
        {
            // A single cylinder crawling below 60 rpm with no Test (a kart
            // starting or stalling): the canned idle stands in as it always
            // has; it never gets the sweep's redline or cut.
            haptics::Layer L; haptics::LawsState st;
            haptics::EffectParams fx; fx.ampPct = 100.0; fx.routes[0] = { 0, 1.0 };
            L.configureFx(FxType::RpmVibe, fx);
            haptics::EngineParams ep; ep.cylinders = 1.0; ep.layout = 0.0; L.configureEngine(ep);
            NcxValues v{}; v.fresh = true;
            v.have[NcxValues::Rpm] = true; v.val[NcxValues::Rpm] = 45.0;
            for (int i = 0; i < 100; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
            check(std::fabs(L.engineFireHz() - 1100.0 / 60.0 * 0.5) < 1e-9 && !L.engineLimiterOn(),
                  "I-8d a live single cylinder below 60 rpm keeps the canned idle (no sweep without a Test)");
        }
        {
            // The sim comes back coasting in the middle of a Test (load high
            // in the sweep, throttle shut now): the hand-off is no lift-off.
            haptics::Layer L; haptics::LawsState st;
            haptics::EffectParams fx; fx.ampPct = 100.0; fx.routes[0] = { 0, 1.0 };
            L.configureFx(FxType::RpmVibe, fx);
            haptics::EngineParams ep; ep.cylinders = 8.0; ep.layout = 1.0; L.configureEngine(ep);
            NcxValues v{}; v.fresh = false;
            haptics::startPreview(st, FxType::RpmVibe);
            for (int i = 0; i < static_cast<int>(1.2 / dt); ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
            const uint64_t before = L.engineLiftOffs();
            v.fresh = true;
            v.have[NcxValues::Rpm] = true;         v.val[NcxValues::Rpm] = 6000.0;
            v.have[NcxValues::ThrottlePct] = true; v.val[NcxValues::ThrottlePct] = 0.0;
            for (int i = 0; i < 200; ++i) { haptics::driveLaws(L, st, v, dt); L.step(dt); }
            check(L.engineLiftOffs() == before, "I-8d the sim returning mid-Test fires no lift-off pop");
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
            sv.val[NcxValues::SpeedKmh] = 40.0; ss.lastSpeedKmh = 40.0; svel(2000.0, 100);   // (moving off as the setup, not a reset: the glitch hold stays off)
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
        // P-8: two tiles on one post: each is derated alone, and their SUM is
        // held at hapticsMaxMm (the cap is on the summed overlay, before the
        // guard). Skid at 1 Hz x 5 mm (would be +-0.5 capped alone) plus the
        // lock-up at 4 Hz x 5 mm (the budget allows 1.27 mm): together they
        // ask for up to 1.8 mm and get 0.5. Breaks with the clamp of hapMm to
        // hapticsMaxMm in stepPositionOnline removed.
        {
            MotionController mc; MockA6Drive mock;
            AppConfig cfg = makePosConfig(100.0, 5.0, 1.0, 0.5, 0.4);
            haptics::EffectParams& lock = cfg.hapticsFx[static_cast<size_t>(haptics::Effect::Lockup)];
            lock.ampPct = 100.0; lock.jitter = 0.0; lock.routes[0] = { 0, 5.0 };
            cfg.hapticsSlipLon = { 1.0, 4.0, 0.0, 10.0, 0.8 };   // lock at 4 Hz (the carrier floor), spin off
            mc.configure(cfg);
            check(bringOnline(mc, mock), "P-8 position axis reaches ONLINE");
            posRun(mc, mock, frame(0.0, true, 32767.0), 600, 1);
            TelemetryData both = frame(100.0, true, 32767.0);
            both.ncx[static_cast<int>(NcxValues::Lockup)] = 100.0;
            const Span loud = posRun(mc, mock, both, 2000, 1000);
            std::printf("  P-8 two tiles on one post, cap 0.5 mm: swing %.2f mm\n", loud.width());
            check(loud.width() > 0.9 && loud.width() < 1.15,      "P-8 two tiles on one post: the sum is held at +-hapticsMaxMm");
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

    // ---- I-24: fresh tyres on a reset (the Wheels tile's freshOnReset,
    // protocol 1.9). A jump back to the pits, a restart and new tyres at a
    // stop wipe the flats; with no pit flag sent, a jump to a standstill
    // does. Driving into the pits, a stop without new tyres, a stream that
    // came back, a jump onto the track with the pit flag down, one junk
    // sample, a car change and the setting off keep them. At most one
    // refresh every 2 s. ----
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.002;
        struct Rig { haptics::Layer L; haptics::LawsState st; NcxValues v{}; };
        // pit: send the pit flag (down), lap 3 and each tyre at 0.3 wear.
        auto make = [&](bool on, bool pit) {
            auto r = std::make_unique<Rig>();
            haptics::EffectParams wp; wp.ampPct = 100.0; wp.routes[0] = { 0, 1.0, Part::FL };
            r->L.configureFx(FxType::Wheels, wp);
            haptics::WheelsParams q; q.balance = 0.0; q.judder = 0.0; q.freshOnReset = on ? 1.0 : 0.0;
            r->L.configureWheels(q);
            NcxValues& v = r->v;
            v.fresh = true;
            v.have[NcxValues::SpeedKmh] = v.have[NcxValues::BrakePct] = true;
            for (int w = 0; w < 4; ++w) v.have[NcxValues::SlipRatioFL + w] = true;
            if (pit)
            {
                v.have[NcxValues::InPit] = v.have[NcxValues::Lap] = true; v.val[NcxValues::Lap] = 3.0;
                for (int w = 0; w < 4; ++w) { v.have[NcxValues::TyreWearFL + w] = true; v.val[NcxValues::TyreWearFL + w] = 0.3; }
            }
            return r;
        };
        auto run = [&](Rig& r, double sec) {
            for (int i = 0, n = static_cast<int>(sec / dt + 0.5); i < n; ++i) { haptics::driveLaws(r.L, r.st, r.v, dt); r.L.step(dt); }
        };
        auto speed = [&](Rig& r, double kmh) { r.v.val[NcxValues::SpeedKmh] = kmh; };
        // A lock-up at 72 km/h grinds a flat into FL; then rolling on at
        // that speed, the pit flag down.
        auto grind = [&](Rig& r) {
            r.v.val[NcxValues::InPit] = 0.0;
            speed(r, 72.0); r.v.val[NcxValues::BrakePct] = 0.0; run(r, 1.0);
            r.v.val[NcxValues::BrakePct] = 100.0;
            for (int w = 0; w < 4; ++w) r.v.val[NcxValues::SlipRatioFL + w] = (w == 0) ? -1.0 : -0.1;
            run(r, 0.5);
            r.v.val[NcxValues::BrakePct] = 0.0;
            for (int w = 0; w < 4; ++w) r.v.val[NcxValues::SlipRatioFL + w] = 0.0;
            run(r, 0.2);
            return r.L.wheelFlatMm(WheelFL) > 0.1;
        };
        auto flat = [&](Rig& r) { return r.L.wheelFlatMm(WheelFL); };
        // A short lock-up at the current speed: a small new flat.
        auto quickLock = [&](Rig& r) {
            r.v.val[NcxValues::BrakePct] = 100.0; r.v.val[NcxValues::SlipRatioFL] = -1.0; run(r, 0.4);
            r.v.val[NcxValues::BrakePct] = 0.0;   r.v.val[NcxValues::SlipRatioFL] = 0.0;  run(r, 0.05);
            return r.L.wheelFlatMm(WheelFL) > 0.02;
        };

        // a. A jump back to the pits: 150 km/h to a standstill with the pit
        // flag rising, after six seconds out.
        {
            auto r = make(true, true);
            check(grind(*r), "I-24 setup: a lock-up grinds a flat");
            speed(*r, 150.0); run(*r, 6.0);
            speed(*r, 0.0); r->v.val[NcxValues::InPit] = 1.0; run(*r, 0.1);
            check(flat(*r) == 0.0, "I-24 a: a jump back to the pits brings fresh tyres");
            // The pit flag a few samples behind the jump, and ahead of it.
            check(grind(*r), "I-24 setup: a new flat");
            speed(*r, 150.0); run(*r, 6.0);
            speed(*r, 0.0); run(*r, 0.05); r->v.val[NcxValues::InPit] = 1.0; run(*r, 0.1);
            check(flat(*r) == 0.0, "I-24 a: ...with the pit flag a few samples behind the jump");
            check(grind(*r), "I-24 setup: a new flat");
            speed(*r, 150.0); run(*r, 6.0);
            r->v.val[NcxValues::InPit] = 1.0; run(*r, 0.05); speed(*r, 0.0); run(*r, 0.1);
            check(flat(*r) == 0.0, "I-24 a: ...and ahead of it");
            // From the pause menu: the stream away for a few seconds, then
            // back with the car in the pits.
            check(grind(*r), "I-24 setup: a new flat");
            speed(*r, 150.0); run(*r, 6.0);
            r->v.fresh = false; run(*r, 3.0);
            r->v.fresh = true; speed(*r, 0.0); r->v.val[NcxValues::InPit] = 1.0; run(*r, 0.1);
            check(flat(*r) == 0.0, "I-24 a: ...and from the pause menu, the stream away meanwhile");
            // ...and from a standstill (stuck in the gravel), through the
            // pause menu: no speed jump at all.
            check(grind(*r), "I-24 setup: a new flat");
            speed(*r, 150.0); run(*r, 6.0);
            for (int i = 0; i < 1000; ++i) { speed(*r, 150.0 * (1.0 - i / 1000.0)); run(*r, dt); }
            speed(*r, 0.0); run(*r, 2.0);
            r->v.fresh = false; run(*r, 3.0);
            r->v.fresh = true; r->v.val[NcxValues::InPit] = 1.0; run(*r, 0.1);
            check(flat(*r) == 0.0, "I-24 a: ...and from a standstill through the pause menu");
            // Out of the pits for under 5 s, then the same jump back: not
            // counted (the pit flag must follow a run out).
            auto q = make(true, true);
            q->v.val[NcxValues::InPit] = 1.0; run(*q, 0.1);
            check(grind(*q), "I-24 setup: a flat");
            speed(*q, 150.0); run(*q, 2.0);
            speed(*q, 0.0); q->v.val[NcxValues::InPit] = 1.0; run(*q, 0.1);
            check(flat(*q) > 0.1, "I-24 a: the pit flag rising under 5 s after leaving the pits does not count");
        }
        // Driving into the pits, and a stop that does not change the tyres,
        // keep them; then new tyres at that stop (c) wipe them.
        {
            auto r = make(true, true);
            check(grind(*r), "I-24 setup: a flat");
            speed(*r, 60.0); run(*r, 6.0);
            for (int i = 0; i < 1500; ++i) { speed(*r, 60.0 * (1.0 - i / 1500.0)); run(*r, dt); }   // 3 s to a stop
            speed(*r, 0.0); r->v.val[NcxValues::InPit] = 1.0; run(*r, 10.0);
            check(flat(*r) > 0.1, "I-24: driving into the pits and stopping keeps the flat");
            for (int w = 0; w < 4; ++w) r->v.val[NcxValues::TyreWearFL + w] = 0.25;   // a reading a touch lower
            run(*r, 1.0);
            check(flat(*r) > 0.1, "I-24: a stop without new tyres keeps it (a wear reading 0.05 lower is not new tyres)");
            r->v.val[NcxValues::InPit] = 0.0;
            for (int i = 0; i < 1500; ++i) { speed(*r, 60.0 * i / 1500.0); run(*r, dt); }
            check(flat(*r) > 0.1, "I-24: ...and driving out keeps it");
            // The pit flag up at the pit lane entry, at speed, then the stop.
            speed(*r, 60.0); run(*r, 6.0);
            r->v.val[NcxValues::InPit] = 1.0;
            for (int i = 0; i < 1500; ++i) { speed(*r, 60.0 * (1.0 - i / 1500.0)); run(*r, dt); }
            run(*r, 2.0);
            check(flat(*r) > 0.1, "I-24: the pit flag up at the pit lane entry keeps it too");
            // c. New tyres at the stop.
            r->v.val[NcxValues::TyreWearFR] = 0.0; run(*r, 0.9);
            check(flat(*r) == 0.0, "I-24 c: one tyre's wear falling to new brings fresh tyres");
            // ...and across a gap in the stream (the highest wear is kept).
            check(grind(*r), "I-24 setup: a new flat");
            r->v.fresh = false; run(*r, 2.0);
            r->v.fresh = true; r->v.val[NcxValues::TyreWearRL] = 0.0; run(*r, 0.9);
            check(flat(*r) == 0.0, "I-24 c: ...and across a gap in the stream");
        }
        // b. A restart: the lap number going back; going up is a lap.
        {
            auto r = make(true, true);
            check(grind(*r), "I-24 setup: a flat");
            r->v.val[NcxValues::Lap] = 4.0; run(*r, 0.9);
            check(flat(*r) > 0.1, "I-24 b: the next lap keeps it");
            r->v.val[NcxValues::Lap] = 2.0; run(*r, 0.9);
            check(flat(*r) == 0.0, "I-24 b: the lap number going back (a restart) brings fresh tyres");
            // At most one refresh every 2 s: a second restart a second later
            // leaves the new flat; new tyres after the guard wipe it.
            check(quickLock(*r), "I-24 setup: a new flat within the guard");
            r->v.val[NcxValues::Lap] = 1.0; run(*r, 0.9);
            check(flat(*r) > 0.02, "I-24 guard: a second restart within 2 s leaves the new flat");
            run(*r, 1.5); r->v.val[NcxValues::TyreWearFL] = 0.0; run(*r, 0.9);
            check(flat(*r) == 0.0, "I-24 guard: ...new tyres after it wipe it");
        }
        // One junk sample is not a reset: 0 km/h for one sample at speed
        // (no pit flag), a lower lap or a worn-to-new tyre for one sample.
        // A lap of 0, even held (a menu), is no lap.
        {
            auto r = make(true, false);
            check(grind(*r), "I-24 setup: a flat");
            speed(*r, 150.0); run(*r, 1.0); speed(*r, 0.0); run(*r, 0.016); speed(*r, 150.0); run(*r, 1.0);
            check(flat(*r) > 0.1, "I-24 junk: one 0 km/h sample at speed (no pit flag) keeps the flat");
            auto p = make(true, true);
            check(grind(*p), "I-24 setup: a flat");
            p->v.val[NcxValues::Lap] = 5.0; run(*p, 1.0);
            p->v.val[NcxValues::Lap] = 2.0; run(*p, 0.016); p->v.val[NcxValues::Lap] = 5.0; run(*p, 1.0);
            check(flat(*p) > 0.1, "I-24 junk: one lower lap sample keeps the flat");
            p->v.val[NcxValues::Lap] = 0.0; run(*p, 2.0); p->v.val[NcxValues::Lap] = 5.0; run(*p, 1.0);
            check(flat(*p) > 0.1, "I-24 junk: a lap of 0, even held (a menu), is no lap");
            p->v.val[NcxValues::TyreWearFL] = 0.0; run(*p, 0.016); p->v.val[NcxValues::TyreWearFL] = 0.3; run(*p, 1.0);
            check(flat(*p) > 0.1, "I-24 junk: one wear sample at new keeps the flat");
        }
        // A sender that sends the pit flag only once it has seen it set: the
        // first flag, set, with the jump back to the pits (or a few samples
        // after it) is a return; a session starting in the garage is not.
        {
            auto r = make(true, false);
            check(grind(*r), "I-24 setup: a flat");
            speed(*r, 150.0); run(*r, 6.0);
            speed(*r, 0.0); r->v.have[NcxValues::InPit] = true; r->v.val[NcxValues::InPit] = 1.0; run(*r, 0.1);
            check(flat(*r) == 0.0, "I-24 a: the pit flag first sent set, with the jump back to the pits, is a return");
            auto q = make(true, false);
            check(grind(*q), "I-24 setup: a flat");
            speed(*q, 150.0); run(*q, 6.0);
            speed(*q, 0.0); run(*q, 0.05); q->v.have[NcxValues::InPit] = true; q->v.val[NcxValues::InPit] = 1.0; run(*q, 0.1);
            check(flat(*q) == 0.0, "I-24 a: ...and a few samples after the jump");
            auto g = make(true, false);
            check(grind(*g), "I-24 setup: a flat");
            speed(*g, 150.0); run(*g, 2.0);
            g->v.fresh = false; run(*g, 3.0);
            g->v.fresh = true; speed(*g, 0.0); g->v.have[NcxValues::InPit] = true; g->v.val[NcxValues::InPit] = 1.0; run(*g, 1.0);
            check(flat(*g) > 0.1, "I-24: a session starting in the garage (the first flag, set, after a gap) is not a return");
        }
        // The last packet before the sender stops stays live for 500 ms: a
        // junk lower lap on it is no restart. Wear is read as a group: three
        // tyres at new with the fourth unsent is no tyre change; all four is.
        {
            auto p = make(true, true);
            check(grind(*p), "I-24 setup: a flat");
            p->v.val[NcxValues::Lap] = 5.0; run(*p, 1.0);
            p->v.val[NcxValues::Lap] = 2.0; run(*p, 0.5);
            p->v.fresh = false; run(*p, 1.0);
            p->v.fresh = true; p->v.val[NcxValues::Lap] = 5.0; run(*p, 1.0);
            check(flat(*p) > 0.1, "I-24 junk: a lower lap on the last packet before the stream stops is no restart");
            p->v.have[NcxValues::TyreWearRR] = false;
            for (int w = 0; w < 3; ++w) p->v.val[NcxValues::TyreWearFL + w] = 0.0;
            run(*p, 1.0);
            check(flat(*p) > 0.1, "I-24 c: three tyres at new with the fourth unsent is no tyre change");
            p->v.have[NcxValues::TyreWearRR] = true; p->v.val[NcxValues::TyreWearRR] = 0.0; run(*p, 1.0);
            check(flat(*p) == 0.0, "I-24 c: ...all four at new is");
        }
        // Car A to Car B: what was known of A's laps and wear starts over
        // (the change itself brings fresh tyres; nothing of A's may then
        // wipe B's first flat).
        {
            auto r = make(true, true);
            std::strcpy(r->v.car, "Car A");
            r->v.val[NcxValues::Lap] = 10.0;
            for (int w = 0; w < 4; ++w) r->v.val[NcxValues::TyreWearFL + w] = 0.6;
            run(*r, 1.0);
            std::strcpy(r->v.car, "Car B");   // its lap and wear not sent yet
            r->v.have[NcxValues::Lap] = false;
            for (int w = 0; w < 4; ++w) r->v.have[NcxValues::TyreWearFL + w] = false;
            check(grind(*r), "I-24 setup: Car B's first flat");
            r->v.have[NcxValues::Lap] = true; r->v.val[NcxValues::Lap] = 3.0;
            for (int w = 0; w < 4; ++w) { r->v.have[NcxValues::TyreWearFL + w] = true; r->v.val[NcxValues::TyreWearFL + w] = 0.1; }
            run(*r, 1.0);
            check(flat(*r) > 0.1, "I-24 car change: Car A's lap and wear are no restart or new tyres for Car B");
        }
        // d. No pit flag sent: a jump to a standstill between two live
        // samples; not across a gap in the stream. With the pit flag sent
        // and down, the same jump (onto the track) keeps the tyres.
        {
            auto r = make(true, false);
            check(grind(*r), "I-24 setup: a flat");
            speed(*r, 150.0); run(*r, 1.0); speed(*r, 0.0); run(*r, 0.9);
            check(flat(*r) == 0.0, "I-24 d: with no pit flag, a jump to a standstill brings fresh tyres");
            check(grind(*r), "I-24 setup: a new flat");
            speed(*r, 150.0); run(*r, 3.0);
            r->v.fresh = false; run(*r, 1.0);
            r->v.fresh = true; speed(*r, 0.0); run(*r, 1.0);
            check(flat(*r) > 0.1, "I-24 d: the stream gone and back at a standstill is not a jump");
            auto t = make(true, true);
            check(grind(*t), "I-24 setup: a flat");
            speed(*t, 150.0); run(*t, 6.0); speed(*t, 0.0); run(*t, 2.0);
            check(flat(*t) > 0.1, "I-24: with the pit flag sent and down, a jump to a standstill (onto the track) keeps it");
        }
        // The setting off: none of them.
        {
            auto r = make(false, true);
            check(grind(*r), "I-24 setup: a flat");
            speed(*r, 150.0); run(*r, 6.0);
            speed(*r, 0.0); r->v.val[NcxValues::InPit] = 1.0; run(*r, 1.0);
            r->v.val[NcxValues::Lap] = 1.0; run(*r, 1.0);
            r->v.val[NcxValues::TyreWearFL] = 0.0; run(*r, 1.0);
            check(flat(*r) > 0.1, "I-24 off: no jump to the pits, restart or new tyres refreshes them");
            auto d = make(false, false);
            check(grind(*d), "I-24 setup: a flat");
            speed(*d, 150.0); run(*d, 1.0); speed(*d, 0.0); run(*d, 1.0);
            check(flat(*d) > 0.1, "I-24 off: ...nor a jump to a standstill with no pit flag");
        }
    }

    // ---- I-25: the belt from the car's g (beltSource "g") ----
    // The tension follows the longitudinal g (accSurge) in a straight line
    // from g from to g at max onto torque min..max, on braking, on
    // acceleration or on either; the motion line's value for the axis is
    // unused; a value past 8 g is a junk frame; the channel stream gone (or
    // silent on accSurge) stages the standby as a lost motion line does;
    // e-stop is 0; a belt left on "motion" ignores the g.
    {
        const double G = 9.80665;
        auto gFrame = [](double accMs2, bool fresh, bool sent = true, double raw0 = 65535.0) {
            TelemetryData sd = frame(0.0, fresh, raw0);
            sd.ncyHave[NcxTok::AccSurge] = sent; sd.ncy[NcxTok::AccSurge] = accMs2;
            return sd;
        };
        auto belt = [&](const char* mode, double smoothMs, const char* source = "g") {
            AppConfig cfg = makeBeltConfig(0.0, 1.0);    // no haptics: the cue alone (torque 5..50)
            cfg.drives[0].beltSource = source; cfg.drives[0].beltGMode = mode;
            cfg.drives[0].beltGFromG = 0.1; cfg.drives[0].beltGFullG = 1.1;
            cfg.drives[0].beltGSmoothMs = smoothMs;
            auto mc = std::make_unique<MotionController>();
            mc->configure(cfg); mc->setEmergencyStop(false);
            tensionBelt(*mc);
            run(*mc, gFrame(0.0, true), 400, 1);
            return mc;
        };
        auto at = [&](MotionController& mc, double accG) { return run(mc, gFrame(accG * G, true), 200, 1).hi; };
        {
            auto mc = belt("decel", 0.0);
            check(mc->getAxisState(0) == AxisMotionState::ONLINE, "I-25 setup: a belt from g reaches ONLINE");
            check(std::fabs(at(*mc, 0.0) - 5.0) < 1e-6, "I-25 decel: no g, torque min (the motion line at full scale is unused)");
            check(std::fabs(at(*mc, 0.6) - 27.5) < 1e-6, "I-25 decel: half way from g from to g at max is half way up the range (a straight line)");
            check(std::fabs(at(*mc, 1.1) - 50.0) < 1e-6, "I-25 decel: g at max, torque max");
            check(std::fabs(at(*mc, 3.0) - 50.0) < 1e-6, "I-25 decel: past g at max, held at torque max");
            check(std::fabs(at(*mc, -1.0) - 5.0) < 1e-6, "I-25 decel: acceleration leaves it at min");
            check(std::fabs(at(*mc, 0.05) - 5.0) < 1e-6, "I-25 decel: inside the dead band, min");
        }
        {
            // Live apply (a rig save that moves only the g shape): the next
            // cycle pulls to the new scale, with no Initialize and the belt
            // still ONLINE; a new surge choice lands the same way.
            auto mc = belt("decel", 0.0);
            check(std::fabs(at(*mc, 0.6) - 27.5) < 1e-6, "I-25 live: before, g at max 1.1");
            DriveConfig d = makeBeltConfig(0.0, 1.0).drives[0];
            d.beltSource = "g"; d.beltGMode = "decel"; d.beltGFromG = 0.1; d.beltGFullG = 2.1; d.beltGSmoothMs = 0.0;
            mc->stageBeltG(0, d);
            const double after = run(*mc, gFrame(0.6 * G, true), 1, 1).hi;
            check(std::fabs(after - (5.0 + 45.0 * 0.25)) < 1e-6 && mc->getAxisState(0) == AxisMotionState::ONLINE,
                  "I-25 live: a staged g at max of 2.1 pulls a quarter of the range at 0.6 g on the next cycle, still ONLINE");
            d.beltGMode = "accel";
            mc->stageBeltG(0, d);
            check(std::fabs(at(*mc, 0.6) - 5.0) < 1e-6 && std::fabs(at(*mc, -0.6) - (5.0 + 45.0 * 0.25)) < 1e-6,
                  "I-25 live: a staged surge on acceleration takes over from braking");
        }
        {
            // The type of car's belt g at max (haptics beltFullG) wins over
            // the belt's own while it is set, and a save of it lands live
            // through the haptics stage; 0 hands back to the belt's own.
            AppConfig cfg = makeBeltConfig(0.0, 1.0);
            cfg.drives[0].beltSource = "g"; cfg.drives[0].beltGMode = "decel";
            cfg.drives[0].beltGFromG = 0.1; cfg.drives[0].beltGFullG = 1.1; cfg.drives[0].beltGSmoothMs = 0.0;
            cfg.hapticsBeltFullG = 2.1;
            MotionController mc; mc.configure(cfg); mc.setEmergencyStop(false); tensionBelt(mc);
            run(mc, gFrame(0.0, true), 400, 1);
            check(std::fabs(at(mc, 0.6) - (5.0 + 45.0 * 0.25)) < 1e-6,
                  "I-25 type: the type of car's g at max (2.1) is in force over the belt's own (1.1)");
            cfg.hapticsBeltFullG = 0.0; mc.stageHaptics(cfg);
            run(mc, gFrame(0.0, true), 50, 1);   // the g moves (an unchanged g for 0.5 s reads as none)
            check(std::fabs(at(mc, 0.6) - 27.5) < 1e-6 && mc.getAxisState(0) == AxisMotionState::ONLINE,
                  "I-25 type: saved back to 0, the belt's own g at max again, live");
        }
        {
            auto mc = belt("accel", 0.0);
            check(std::fabs(at(*mc, -0.6) - 27.5) < 1e-6 && std::fabs(at(*mc, 1.0) - 5.0) < 1e-6,
                  "I-25 accel: acceleration tightens, braking does not");
            auto mb = belt("both", 0.0);
            check(std::fabs(at(*mb, -0.6) - 27.5) < 1e-6 && std::fabs(at(*mb, 0.6) - 27.5) < 1e-6,
                  "I-25 both: either way tightens alike");
        }
        {
            // A curve of 2: at half the g share, a quarter of the range.
            AppConfig cfg = makeBeltConfig(0.0, 1.0);
            cfg.drives[0].beltSource = "g"; cfg.drives[0].beltGMode = "decel";
            cfg.drives[0].beltGFromG = 0.1; cfg.drives[0].beltGFullG = 1.1; cfg.drives[0].beltGSmoothMs = 0.0;
            cfg.drives[0].beltGCurve = 2.0;
            MotionController mc; mc.configure(cfg); mc.setEmergencyStop(false); tensionBelt(mc);
            run(mc, gFrame(0.0, true), 400, 1);
            check(std::fabs(at(mc, 0.6) - (5.0 + 45.0 * 0.25)) < 1e-6, "I-25 curve 2: half the g share gives a quarter of the range");
        }
        {
            auto mc = belt("decel", 30.0);
            const double s1 = run(*mc, gFrame(1.1 * G, true), 15, 1).hi;   // 30 ms at 2 ms a cycle
            check(s1 > 5.0 + 0.5 * 45.0 && s1 < 5.0 + 0.75 * 45.0, "I-25 smoothing 30 ms: 30 ms after a step, about two thirds of the way");
            check(std::fabs(run(*mc, gFrame(1.1 * G, true), 200, 1).hi - 50.0) < 0.05, "I-25 smoothing: then it settles at torque max");
        }
        {
            // One junk frame does not restart the smoothing: the next g is
            // eased into, not jumped to.
            auto mc = belt("decel", 30.0);
            run(*mc, gFrame(0.6 * G, true), 200, 1);
            run(*mc, gFrame(20.0 * G, true), 8, 1);                      // one 16 ms junk frame
            const double next = run(*mc, gFrame(1.1 * G, true), 1, 1).hi;
            check(next > 27.0 && next < 31.0, "I-25 smoothing: one junk frame does not restart it (the next g is eased into)");
            // A real gap does: back from 3 s of silence at no g, the belt
            // starts at min, not from the braking before the gap.
            run(*mc, gFrame(1.1 * G, true), 200, 1);
            run(*mc, gFrame(1.1 * G, false), 1500, 1);
            const double back = run(*mc, gFrame(0.0, true), 1, 1).hi;
            check(std::fabs(back - 5.0) < 1e-6, "I-25 smoothing: after a gap it starts afresh (min at no g, not the braking before)");
        }
        {
            // Tension-up starts the smoothing afresh: slacked while braking
            // hard, tensioned again with the car still, the blend never
            // climbs from the old braking.
            auto mc = belt("decel", 250.0);
            run(*mc, gFrame(2.0 * G, true), 150, 1);
            { MotionCommand c; c.type = MotionCommand::Type::SlackBelts; mc->enqueueCommand(c); }
            run(*mc, gFrame(2.0 * G, true), 200, 1);
            tensionBelt(*mc);
            const Span up = run(*mc, gFrame(0.0, true), 100, 100);
            check(up.hi <= 5.0 + 1e-6, "I-25 tension-up starts the g smoothing afresh (no pull from the braking before the slack)");
        }
        {
            // The blend (tension-up) follows the g too: at no g it rises to
            // min only, whatever the motion line says for the axis.
            AppConfig cfg = makeBeltConfig(0.0, 1.0);
            cfg.drives[0].beltSource = "g"; cfg.drives[0].beltGSmoothMs = 0.0;
            MotionController mc; mc.configure(cfg); mc.setEmergencyStop(false); tensionBelt(mc);
            const Span blend = run(mc, gFrame(0.0, true, true, 65535.0), 60, 60);
            check(blend.hi <= 5.0 + 1e-6, "I-25 the tension-up blend follows the g (the motion line at full scale is unused)");
        }
        {
            // An accSurge frozen while the rest of the stream lives on (a
            // sender that stopped sending it) reads as no g: min, ONLINE.
            auto mc = belt("decel", 0.0);
            const double frozen = run(*mc, gFrame(1.1 * G, true), 400, 1).hi;   // 0.8 s unchanged
            check(std::fabs(frozen - 5.0) < 1e-6 && mc->getAxisState(0) == AxisMotionState::ONLINE,
                  "I-25 an accSurge unchanged for over 0.5 s reads as no g: min, still ONLINE");
            check(std::fabs(at(*mc, 0.6) - 27.5) < 1e-6, "I-25 ...and a changing g takes over again");
        }
        {
            // Settings out of range (validation only warns on load): a
            // curve below 0 never asks for more than torque max.
            AppConfig cfg = makeBeltConfig(0.0, 1.0);
            cfg.drives[0].beltSource = "g"; cfg.drives[0].beltGSmoothMs = 0.0; cfg.drives[0].beltGCurve = -1.0;
            MotionController mc; mc.configure(cfg); mc.setEmergencyStop(false); tensionBelt(mc);
            const Span still = run(mc, gFrame(0.0, true), 400, 300);
            const Span hard  = run(mc, gFrame(0.8 * G, true), 200, 200);
            check(still.hi <= 5.0 + 1e-6 && hard.hi <= 50.0 + 1e-6, "I-25 a curve below 0 never asks for more than torque max");
        }
        {
            auto mc = belt("decel", 0.0);
            const double held = run(*mc, gFrame(0.6 * G, true), 100, 1).hi;
            const double junk = run(*mc, gFrame(20.0 * G, true), 5, 1).hi;
            check(std::fabs(junk - held) < 1e-6, "I-25 a value past 8 g is a junk frame: the tension holds");
            run(*mc, gFrame(0.6 * G, true), 50, 1);
            const double stale = run(*mc, gFrame(0.6 * G, false), 1250, 1).hi;   // 2.5 s: past the 2 s hold
            check(std::fabs(stale - 5.0) < 1e-6 && mc->getAxisState(0) == AxisMotionState::ONLINE,
                  "I-25 the channel stream gone: past the hold, torque min (never the last g)");
            auto mu = belt("decel", 0.0);
            run(*mu, gFrame(0.6 * G, true), 100, 1);
            const double unsent = run(*mu, gFrame(0.6 * G, true, false), 1250, 1).hi;
            check(std::fabs(unsent - 5.0) < 1e-6, "I-25 a live stream without accSurge is no g: torque min past the hold");
        }
        {
            auto mc = belt("decel", 0.0);
            run(*mc, gFrame(1.1 * G, true), 100, 1);
            mc->setEmergencyStop(true);
            const Span e = run(*mc, gFrame(1.1 * G, true), 50, 10);
            check(e.hi <= 1e-9 && e.lo >= -1e-9, "I-25 e-stop: torque 0 whatever the g");
        }
        {
            auto mc = belt("decel", 0.0, "motion");
            const double t = run(*mc, gFrame(1.1 * G, true, true, 0.0), 200, 1).hi;
            check(std::fabs(t - 5.0) < 1e-6, "I-25 a belt left on motion follows the motion line (0 = min), whatever the g");
        }
    }

    // ---- I-27: Impacts from the body's g along and across the car ----
    // A wall at speed knocks, and so does a crash that sheds speed faster
    // than a reset check allows when the car's g confirms it; a junk value
    // (past 1000 m/s^2) does not; a reset's jump (the speed glitch) with a
    // spike in the same sample does not; one junk sample under hard braking
    // does not; a car standing does not; hit g 0 does not.
    {
        using haptics::FxType; using haptics::Part;
        const double dt = 0.002, G = 9.80665;
        struct Rig { haptics::Layer L; haptics::LawsState st; NcxValues v{}; };
        auto make = [&](double hitG) {
            auto r = std::make_unique<Rig>();
            haptics::EffectParams ip; ip.ampPct = 100.0; ip.freqHz = 40.0; ip.durMs = 50.0; ip.routes[0] = { 0, 1.0, Part::All };
            r->L.configureFx(FxType::Impacts, ip);
            haptics::ImpactParams q; q.hitG = hitG; r->L.configureImpacts(q);
            r->v.fresh = true;
            r->v.have[NcxValues::SpeedKmh] = true; r->v.val[NcxValues::SpeedKmh] = 120.0;
            r->v.have[NcxValues::AccSurge] = r->v.have[NcxValues::AccSway] = true;
            return r;
        };
        // One 60 Hz sample of surge (g) at speed (km/h), then 0.3 s more
        // of the same; knocks on FL after.
        auto sample = [&](Rig& r, double surgeG, double kmh) {
            r.v.val[NcxValues::AccSurge] = surgeG * G; r.v.val[NcxValues::SpeedKmh] = kmh;
            for (int i = 0; i < static_cast<int>(0.3 / dt); ++i) { haptics::driveLaws(r.L, r.st, r.v, dt); r.L.step(dt); }
            return r.L.impactModel().knocks(WheelFL);
        };
        {
            auto r = make(2.0);
            sample(*r, 0.0, 120.0);
            check(sample(*r, -8.0, 120.0) == 1, "I-27: an 8 g wall at speed knocks");
            check(sample(*r, 0.0, 120.0) == 2, "I-27: ...and the 8 g back is a knock of its own");
        }
        {
            auto r = make(2.0);
            sample(*r, 0.0, 120.0);
            const uint32_t k = sample(*r, 150.0, 120.0);  // 1471 m/s^2: junk
            check(k == 0 && sample(*r, 0.0, 120.0) == 0, "I-27: a value past 1000 m/s^2 (about 100 g) is junk: no knock, nor on the way back");
        }
        {
            auto r = make(2.0);
            sample(*r, 0.0, 120.0);
            r->v.val[NcxValues::AccSurge] = -8.0 * G; r->v.val[NcxValues::SpeedKmh] = 60.0;   // 120 to 60 in one sample: a glitch
            for (int i = 0; i < static_cast<int>(0.3 / dt); ++i) { haptics::driveLaws(r->L, r->st, r->v, dt); r->L.step(dt); }
            check(r->L.impactModel().knocks(WheelFL) == 0, "I-27: a reset's jump with a spike in the same sample knocks nothing");
        }
        // A 60 Hz stream: speed and the g change every sample, as a sim's do.
        // brake(r, secs, g): braking at g from the current speed; then one
        // sample of (dv km/h, surge g, sway g).
        auto brake = [&](Rig& r, double secs, double gBrake, double swayG = 0.3) {
            const int samples = static_cast<int>(secs * 60.0);
            for (int k = 0; k < samples; ++k)
            {
                r.v.val[NcxValues::SpeedKmh] -= gBrake * G / 60.0 * 3.6;
                r.v.val[NcxValues::AccSurge] = -gBrake * G + 0.01 * (k % 3);
                r.v.val[NcxValues::AccSway]  = swayG * G + 0.01 * (k % 2);
                for (int i = 0; i < static_cast<int>(1.0 / 60.0 / dt); ++i) { haptics::driveLaws(r.L, r.st, r.v, dt); r.L.step(dt); }
            }
        };
        auto one = [&](Rig& r, double dvKmh, double surgeG, double swayG) {
            r.v.val[NcxValues::SpeedKmh] -= dvKmh;
            r.v.val[NcxValues::AccSurge] = surgeG * G; r.v.val[NcxValues::AccSway] = swayG * G;
            for (int i = 0; i < static_cast<int>(1.0 / 60.0 / dt); ++i) { haptics::driveLaws(r.L, r.st, r.v, dt); r.L.step(dt); }
        };
        {
            // A crash: 36 km/h shed in one sample with the car's g saying so
            // (50 g): not a reset, and a knock (it was a glitch, and past
            // the old 300 m/s^2 cap, junk).
            auto r = make(2.0);
            r->v.val[NcxValues::SpeedKmh] = 150.0;
            brake(*r, 1.0, 1.0);
            one(*r, 36.0, -50.0, 0.3);
            brake(*r, 0.3, 1.0);
            check(r->st.glitchSec == 0.0, "I-27: 36 km/h shed in a sample with 50 g along the car is a crash, not a reset");
            check(r->L.impactModel().knocks(WheelFL) >= 1, "I-27: ...and it knocks");
        }
        {
            // A reset at speed: 110 km/h gone in a sample (to the pits), the
            // car's g 8 g, far short of what that would take: a reset,
            // nothing knocks (an 8 g jump would).
            auto r = make(2.0);
            r->v.val[NcxValues::SpeedKmh] = 150.0;
            brake(*r, 1.0, 1.0);
            one(*r, 110.0, -8.0, 0.3);
            brake(*r, 0.3, 0.0, 0.3);
            check(r->st.glitchSec > 0.0, "I-27: a reset (110 km/h gone in a sample at 8 g) is a reset");
            check(r->L.impactModel().knocks(WheelFL) == 0, "I-27: ...and knocks nothing");
        }
        {
            // Hard braking (2.5 g) with one junk surge sample: the missing
            // value is no jump to 0 and back.
            auto r = make(2.0);
            r->v.val[NcxValues::SpeedKmh] = 200.0;
            brake(*r, 1.0, 2.5);
            one(*r, 0.7, 2000.0 / G, 0.3);   // 2000 m/s^2: junk
            brake(*r, 0.5, 2.5);
            check(r->L.impactModel().knocks(WheelFL) == 0, "I-27: one junk surge sample under hard braking knocks nothing");
        }
        {
            auto r = make(2.0);
            sample(*r, 0.0, 1.0);
            check(sample(*r, -8.0, 1.0) == 0, "I-27: a car standing does not knock");
            auto z = make(0.0);
            sample(*z, 0.0, 120.0);
            check(sample(*z, -8.0, 120.0) == 0, "I-27: hit g 0 = off");
        }
    }

    // ---- I-26: the belt from g, cornering (accSway) and heave (accHeave) ----
    // Each axis of the car folded by its own choice, sway and heave at their
    // share, added up and mapped as braking alone is; heave counts as its
    // change from its own running mean (gravity and a steady hill never
    // tighten); a channel junk for a frame holds, unsent or gone it adds
    // nothing while another g the belt takes still lives; every g gone is
    // the standby stages; nothing selected sits at min, ONLINE.
    {
        const double G = 9.80665;
        // A frame with all three g; each one changes by a hair every other
        // cycle (a live sender never repeats a float exactly) unless its bit
        // in still is set (then it is held exactly, as a sender that stopped).
        int tick = 0;
        auto f3 = [&](double surge, double sway, double heave, unsigned unsent = 0, unsigned still = 0) {
            TelemetryData sd = frame(0.0, true, 65535.0);
            const double e = (++tick & 1) ? 1e-4 : 0.0;
            sd.ncyHave[NcxTok::AccSurge] = !(unsent & 1); sd.ncy[NcxTok::AccSurge] = surge + ((still & 1) ? 0.0 : e);
            sd.ncyHave[NcxTok::AccSway]  = !(unsent & 2); sd.ncy[NcxTok::AccSway]  = sway  + ((still & 2) ? 0.0 : e);
            sd.ncyHave[NcxTok::AccHeave] = !(unsent & 4); sd.ncy[NcxTok::AccHeave] = heave + ((still & 4) ? 0.0 : e);
            return sd;
        };
        // Cycles of one g each; the torque on the last one.
        auto hold = [&](MotionController& mc, double surgeG, double swayG, double heaveG, int cycles,
                        unsigned unsent = 0, unsigned still = 0) {
            double t = 0.0;
            for (int c = 0; c < cycles; ++c) t = run(mc, f3(surgeG * G, swayG * G, heaveG * G, unsent, still), 1, 1).hi;
            return t;
        };
        auto belt = [&](const char* surge, const char* sway, double swayMix, const char* heave, double heaveMix) {
            AppConfig cfg = makeBeltConfig(0.0, 1.0);    // no haptics: the cue alone (torque 5..50)
            DriveConfig& d = cfg.drives[0];
            d.beltSource = "g"; d.beltGMode = surge;
            d.beltGSway = sway; d.beltGSwayMix = swayMix; d.beltGHeave = heave; d.beltGHeaveMix = heaveMix;
            d.beltGFromG = 0.1; d.beltGFullG = 1.1; d.beltGSmoothMs = 0.0;
            auto mc = std::make_unique<MotionController>();
            mc->configure(cfg); mc->setEmergencyStop(false);
            tensionBelt(*mc);
            hold(*mc, 0.0, 0.0, 1.0, 400);               // gravity on heave, the car still
            return mc;
        };
        const auto near = [](double a, double b) { return std::fabs(a - b) < 1e-3; };
        {
            auto mc = belt("off", "left", 1.0, "off", 1.0);
            check(mc->getAxisState(0) == AxisMotionState::ONLINE, "I-26 setup: a belt on cornering alone reaches ONLINE");
            check(near(hold(*mc, 0.0, 0.6, 1.0, 100), 27.5), "I-26 sway left: 0.6 g one way is half way up (as braking is)");
            check(near(hold(*mc, 0.0, -0.6, 1.0, 100), 5.0), "I-26 sway left: the other way leaves it at min");
            check(near(hold(*mc, 1.1, 0.0, 1.0, 100), 5.0), "I-26 braking off: braking alone leaves it at min");
            auto mr = belt("off", "right", 1.0, "off", 1.0);
            check(near(hold(*mr, 0.0, -0.6, 1.0, 100), 27.5) && near(hold(*mr, 0.0, 0.6, 1.0, 100), 5.0),
                  "I-26 sway right: the other side tightens it");
            auto mb = belt("off", "both", 1.0, "off", 1.0);
            check(near(hold(*mb, 0.0, -0.6, 1.0, 100), 27.5) && near(hold(*mb, 0.0, 0.6, 1.0, 100), 27.5),
                  "I-26 sway both: either side tightens it alike");
        }
        {
            // Braking and cornering add up, cornering at its share.
            auto mc = belt("decel", "left", 0.5, "off", 1.0);
            check(near(hold(*mc, 0.4, 0.4, 1.0, 100), 27.5), "I-26 braking 0.4 g + cornering 0.4 g at a share of 0.5 = 0.6 g");
            auto mo = belt("decel", "off", 1.0, "off", 1.0);
            check(near(hold(*mo, 0.6, 1.0, 1.0, 100), 27.5), "I-26 cornering off: the cornering g adds nothing");
        }
        {
            // Heave: the change from its mean, not the g itself.
            auto ml = belt("off", "off", 1.0, "lift", 1.0);
            check(near(hold(*ml, 0.0, 0.0, 1.0, 1500), 5.0), "I-26 heave: gravity alone (1 g, 3 s) never tightens it");
            const double lift = hold(*ml, 0.0, 0.0, 0.4, 1);
            check(std::fabs(lift - 27.5) < 0.1, "I-26 heave lift: 0.6 g below the mean (a crest) is half way up");
            const double settled = hold(*ml, 0.0, 0.0, 0.4, 10000);
            check(near(settled, 5.0), "I-26 heave: a steady new g (a hill, a banking) fades back to min");
            auto mp = belt("off", "off", 1.0, "press", 1.0);
            check(near(hold(*mp, 0.0, 0.0, 0.4, 1), 5.0), "I-26 heave press: a crest leaves it at min");
            auto mp2 = belt("off", "off", 1.0, "press", 2.0);
            check(std::fabs(hold(*mp2, 0.0, 0.0, 1.3, 1) - 27.5) < 0.1, "I-26 heave press at a share of 2: 0.3 g into a compression = 0.6 g");
        }
        {
            // One g the belt takes, unsent: the others still drive it.
            auto mc = belt("decel", "left", 1.0, "off", 1.0);
            const double t = hold(*mc, 0.6, 0.0, 1.0, 1250, 2);
            check(near(t, 27.5) && mc->getAxisState(0) == AxisMotionState::ONLINE,
                  "I-26 cornering unsent: braking still drives the belt (no standby)");
            // ...and the only one it takes, unsent: the standby stages.
            auto ms = belt("off", "left", 1.0, "off", 1.0);
            hold(*ms, 0.0, 0.6, 1.0, 100);
            const double gone = hold(*ms, 0.0, 0.6, 1.0, 1250, 2);
            check(near(gone, 5.0) && ms->getAxisState(0) == AxisMotionState::ONLINE, "I-26 cornering alone, unsent: min past the hold");
        }
        {
            // A junk cornering frame holds the cornering g; a run of them drops it.
            auto mc = belt("decel", "left", 1.0, "off", 1.0);
            check(near(hold(*mc, 0.6, 0.4, 1.0, 100), 45.5), "I-26 braking 0.6 g + cornering 0.4 g = 1.0 g");
            check(near(hold(*mc, 0.6, 20.0, 1.0, 5), 45.5), "I-26 a junk cornering frame: the cornering g holds");
            check(near(hold(*mc, 0.6, 20.0, 1.0, 200), 27.5), "I-26 junk on cornering past 0.1 s: braking alone");
            // A cornering g held exactly (a sender that stopped sending it) is no g.
            hold(*mc, 0.6, 0.4, 1.0, 100);
            check(near(hold(*mc, 0.6, 0.4, 1.0, 400, 0, 2), 27.5), "I-26 cornering frozen over 0.5 s: no cornering g");
        }
        {
            // Parked in the pits: heave exactly 1 g (gravity, as a sender
            // that rounds it), the stream gone 0.3 s (a pause), then
            // driving off at about 1 g: the belt stays at min (it was
            // seeded at 0 g after the gap, a 1 g press).
            auto mc = belt("off", "off", 1.0, "both", 1.0);
            hold(*mc, 0.0, 0.0, 1.0, 1000, 0, 7);
            TelemetryData gap = f3(0.0, 0.0, G, 0, 7); gap.ncxFresh = false;
            run(*mc, gap, 150, 1);
            double hi = 0.0;
            for (int c = 0; c < 100; ++c) hi = std::max(hi, hold(*mc, 0.0, 0.0, 1.0, 1, 0, 7));
            for (int c = 0; c < 300; ++c) hi = std::max(hi, hold(*mc, 0.0, 0.0, 1.0 + 0.01 * std::sin(c * 0.1), 1));
            check(hi < 5.0 + 0.5, "I-26 parked with gravity, a pause, then driving off: no pull from gravity");
        }
        {
            // A pause on the crest of a bump: heave's steady part is kept
            // through it, so back at 1 g after the bump the belt is at min,
            // not pulled for seconds by a mean started on the crest.
            auto mc = belt("off", "off", 1.0, "both", 1.0);
            hold(*mc, 0.0, 0.0, 1.0, 1000);
            hold(*mc, 0.0, 0.0, 1.5, 3);
            TelemetryData gap = f3(0.0, 0.0, 1.5 * G); gap.ncxFresh = false;
            run(*mc, gap, 150, 1);
            hold(*mc, 0.0, 0.0, 1.5, 1);
            hold(*mc, 0.0, 0.0, 1.0, 50);
            double hi = 0.0;
            for (int c = 0; c < 150; ++c) hi = std::max(hi, hold(*mc, 0.0, 0.0, 1.0, 1));
            check(hi < 5.0 + 0.5, "I-26 a pause on a bump's crest: heave's steady part kept, back at min after it");
        }
        {
            // The smoothing works on cornering as on braking: 30 ms after a
            // step to 0.6 g, about two thirds of the way.
            AppConfig cfg = makeBeltConfig(0.0, 1.0);
            DriveConfig& d = cfg.drives[0];
            d.beltSource = "g"; d.beltGMode = "off"; d.beltGSway = "left";
            d.beltGFromG = 0.1; d.beltGFullG = 1.1; d.beltGSmoothMs = 30.0;
            MotionController mc; mc.configure(cfg); mc.setEmergencyStop(false); tensionBelt(mc);
            hold(mc, 0.0, 0.0, 1.0, 400);
            const double t = hold(mc, 0.0, 0.6, 1.0, 15);
            check(t > 14.0 && t < 21.0, "I-26 smoothing 30 ms on cornering: 30 ms after a step, about two thirds of the way");
        }
        {
            // A channel switched back on live starts afresh: sway taken off
            // with its low-pass high from a corner, then put back on with no
            // cornering, pulls nothing (it used to pull from the old corner
            // for up to the smoothing time). Surge stays on throughout.
            AppConfig cfg = makeBeltConfig(0.0, 1.0);
            DriveConfig& d = cfg.drives[0];
            d.beltSource = "g"; d.beltGMode = "decel"; d.beltGSway = "left";
            d.beltGFromG = 0.1; d.beltGFullG = 1.1; d.beltGSmoothMs = 250.0;
            MotionController mc; mc.configure(cfg); mc.setEmergencyStop(false); tensionBelt(mc);
            hold(mc, 0.0, 0.0, 1.0, 100);
            const double corner = hold(mc, 0.0, 1.0, 1.0, 200);   // 0.4 s of corner: the low-pass well up
            DriveConfig off = d; off.beltGSway = "off";
            mc.stageBeltG(0, off);
            hold(mc, 0.0, 0.0, 1.0, 10);
            mc.stageBeltG(0, d);
            const double back = hold(mc, 0.0, 0.0, 1.0, 1);
            check(corner > 25.0 && near(back, 5.0), "I-26 live: sway switched back on with no cornering pulls nothing (starts afresh, not from the corner before)");
        }
        {
            // Nothing selected: min, and ONLINE (not the standby stages).
            auto mc = belt("off", "off", 1.0, "off", 1.0);
            const double t = hold(*mc, 1.1, 1.0, 0.2, 3000);
            check(near(t, 5.0) && mc->getAxisState(0) == AxisMotionState::ONLINE,
                  "I-26 nothing selected: min, still ONLINE after 6 s");
        }
        {
            // Tension-up starts the heave mean afresh: slacked at 1 g,
            // tensioned again on a steady 1.5 g, the belt stays at min.
            auto mc = belt("off", "off", 1.0, "both", 1.0);
            hold(*mc, 0.0, 0.0, 1.0, 1000);
            { MotionCommand c; c.type = MotionCommand::Type::SlackBelts; mc->enqueueCommand(c); }
            hold(*mc, 0.0, 0.0, 1.0, 200);
            tensionBelt(*mc);
            double hi = 0.0;
            for (int c = 0; c < 100; ++c) hi = std::max(hi, hold(*mc, 0.0, 0.0, 1.5, 1));
            check(hi <= 5.0 + 1e-3, "I-26 tension-up starts the heave mean afresh (no pull from the g before the slack)");
        }
    }

    // ---- I-29: the belt from g lets a teleport go by, never a crash ----
    // A reset, a restart or a recovery puts the car elsewhere: the sender's
    // g spikes far past a car's and decays over about half a second (AC
    // Rally 2026-10-27: 12 g on a restart, 19 g on a reset from 96 km/h),
    // and the speed jumps (47 to 93 km/h in a frame) or the road under the
    // car jumps in height (70 to 128 m). Either holds the g off for a
    // second: the belt sits at min through the tail (under 8 g, several),
    // then follows again. A crash is felt: its speed falls by at most about
    // 20 km/h a frame, and the tail past its junk frames pulls. Real
    // braking, even a sudden 1.5 g, never trips it. Taking the hold-off out
    // of beltDemand fails the three tail checks; making a g past 8 g a
    // teleport fails the crash check.
    {
        const double G = 9.80665;
        int tick = 0;
        double speed = 0.0, roadMm = 455000.0;
        auto f = [&](double accG) {
            TelemetryData sd = frame(0.0, true, 65535.0);
            ++tick;
            sd.ncx[static_cast<int>(NcxValues::SpeedKmh)] = speed + ((tick & 1) ? 1e-3 : 0.0);
            sd.ncyHave[NcxTok::AccSurge] = true;
            sd.ncy[NcxTok::AccSurge] = accG * G + ((tick & 1) ? 1e-4 : 0.0);
            for (int w = 0; w < 4; ++w) { sd.ncyHave[NcxTok::RoadHeightFL + w] = true; sd.ncy[NcxTok::RoadHeightFL + w] = roadMm + ((tick & 1) ? 0.1 : 0.0); }
            return sd;
        };
        auto hiFor = [&](MotionController& mc, double sec, double accG) {
            double hi = -1e9;
            const int cycles = std::max(1, static_cast<int>(sec / 0.002 + 0.5));
            for (int c = 0; c < cycles; ++c) hi = std::max(hi, run(mc, f(accG), 1, 1).hi);
            return hi;
        };
        auto lastFor = [&](MotionController& mc, double sec, double accG) {
            double t = 0.0;
            const int cycles = std::max(1, static_cast<int>(sec / 0.002 + 0.5));
            for (int c = 0; c < cycles; ++c) t = run(mc, f(accG), 1, 1).hi;
            return t;
        };
        auto belt = [&]() {
            AppConfig cfg = makeBeltConfig(0.0, 1.0);    // torque 5..50
            DriveConfig& d = cfg.drives[0];
            d.beltSource = "g"; d.beltGMode = "decel";
            d.beltGFromG = 0.1; d.beltGFullG = 1.1; d.beltGSmoothMs = 0.0;
            auto mc = std::make_unique<MotionController>();
            mc->configure(cfg);
            tensionBelt(*mc);
            hiFor(*mc, 0.8, 0.0);
            return mc;
        };
        // The decaying tail of a spike, frame by frame from `from` g: its
        // highest tension.
        auto tail = [&](MotionController& mc, double from) {
            double hi = -1e9;
            for (double g = from; g > 0.3; g *= 0.85) hi = std::max(hi, hiFor(mc, 0.02, g));
            return hi;
        };
        // A restart: the speed to 0 in one frame, a 12 g spike, its tail.
        {
            speed = 67.0;
            auto mc = belt();
            check(mc->getAxisState(0) == AxisMotionState::ONLINE && std::fabs(lastFor(*mc, 0.2, 0.9) - 41.0) < 1e-3, "I-29 setup: braking at 0.9 g, 41 %");
            speed = 0.0;
            hiFor(*mc, 0.01, 12.0);
            check(tail(*mc, 7.9) <= 5.0 + 1e-3, "I-29 a restart (the speed to 0 in a frame) holds the belt at min through the spike's tail below 8 g");
            hiFor(*mc, 0.7, 0.0);
            check(std::fabs(lastFor(*mc, 0.1, 0.6) - 27.5) < 1e-3 && mc->getAxisState(0) == AxisMotionState::ONLINE,
                  "I-29 ...and a second on, the belt follows the g again, ONLINE");
        }
        // A restart as AC Rally does it: the stream pauses 0.4 s to load the
        // stage, then comes back at 0 km/h with the spike (67 km/h before).
        {
            speed = 67.0;
            auto mc = belt();
            lastFor(*mc, 0.2, 0.9);
            for (int c = 0; c < 200; ++c) { TelemetryData sd = f(0.9); sd.ncxFresh = false; run(*mc, sd, 1, 1); }
            speed = 0.0;
            hiFor(*mc, 0.01, 0.0);
            hiFor(*mc, 0.01, 12.0);
            check(tail(*mc, 7.9) <= 5.0 + 1e-3, "I-29 a restart across a pause (67 km/h before, 0 after) holds the belt at min through the spike's tail");
        }
        // A pause that resumes as it stopped is no teleport: the belt pulls
        // at once.
        {
            speed = 100.0;
            auto mc = belt();
            lastFor(*mc, 0.2, 0.9);
            for (int c = 0; c < 500; ++c) { TelemetryData sd = f(0.9); sd.ncxFresh = false; run(*mc, sd, 1, 1); }
            speed = 99.8;
            check(std::fabs(lastFor(*mc, 0.006, 1.0) - 45.5) < 1e-3, "I-29 a pause that resumes as it stopped is no teleport: the belt pulls at once");
        }
        // AC Rally's restart menu: the speed frozen at 46.54 km/h for half a
        // second while the lines keep coming, then 0.01 with the spike.
        {
            speed = 46.54;
            auto mc = belt();
            for (int c = 0; c < 250; ++c) { TelemetryData sd = f(0.0); sd.ncx[static_cast<int>(NcxValues::SpeedKmh)] = 46.54; run(*mc, sd, 1, 1); }
            speed = 0.01;
            double hi = hiFor(*mc, 0.01, 3.0);
            hi = std::max(hi, hiFor(*mc, 0.03, 9.4));
            check(std::max(hi, tail(*mc, 7.9)) <= 5.0 + 1e-3, "I-29 a speed frozen, then a jump to 0 (the restart menu): held at min from the jump on");
        }
        // A reset on the move: 96 to 46 km/h in a frame with a 5.6 g sample.
        {
            speed = 96.0;
            auto mc = belt();
            lastFor(*mc, 0.2, 0.9);
            speed = 46.0;
            const double hi = std::max(hiFor(*mc, 0.01, 5.6), tail(*mc, 5.0));
            check(hi <= 5.0 + 1e-3, "I-29 a reset on the move (a speed jump no car makes) holds the belt at min through the g it brings");
        }
        // A restart standing still: the speed stays 0, the road under the
        // car jumps 70 m.
        {
            speed = 0.0; roadMm = 455000.0;
            auto mc = belt();
            roadMm = 525000.0;
            hiFor(*mc, 0.01, 12.0);
            check(tail(*mc, 7.9) <= 5.0 + 1e-3, "I-29 a restart standing still (the road's height jumps 70 m) holds the belt at min too");
        }
        // AC Rally drops one corner's contact point to 0 for a frame, even
        // at speed: no teleport, the braking pulls on. Nor is a landing
        // (all four heights step 5 m at 80 km/h): a restart in place is
        // only standing still.
        {
            speed = 100.0; roadMm = 463000.0;
            auto mc = belt();
            lastFor(*mc, 0.2, 0.9);
            TelemetryData sd = f(0.9); sd.ncy[NcxTok::RoadHeightFL] = 0.0;
            run(*mc, sd, 1, 1);
            check(std::fabs(lastFor(*mc, 0.05, 1.0) - 45.5) < 1e-3, "I-29 one corner's height reading 0 for a frame at speed is no teleport");
            speed = 80.0; roadMm = 458000.0;
            check(std::fabs(lastFor(*mc, 0.05, 1.0) - 45.5) < 1e-3, "I-29 a landing (every height steps 5 m at 80 km/h) is no teleport");
        }
        // A crash: 62 to 42 km/h in a frame (a rock), a 20 g frame (junk,
        // held), then a tail from 7 g: felt.
        {
            speed = 62.0; roadMm = 455000.0;
            auto mc = belt();
            lastFor(*mc, 0.2, 0.0);
            speed = 42.0;
            hiFor(*mc, 0.01, 20.0);
            check(tail(*mc, 7.0) >= 50.0 - 1e-3, "I-29 a crash is no teleport: its tail pulls the belt to max");
        }
        // Real braking: the speed falls a little each frame; a sudden 1.5 g
        // (a hard hit, under the junk limit) is felt at once.
        {
            speed = 120.0;
            auto mc = belt();
            hiFor(*mc, 0.2, 0.0);
            speed = 119.6;
            check(std::fabs(lastFor(*mc, 0.006, 1.5) - 50.0) < 1e-3, "I-29 real braking, even a sudden 1.5 g, pulls within 6 ms (the belt's slew; no hold-off)");
            double lo = 1e9;
            for (int c = 0; c < 100; ++c) { speed -= 0.1; lo = std::min(lo, lastFor(*mc, 0.002, 1.0)); }
            check(std::fabs(lo - 45.5) < 1e-3, "I-29 ...and keeps following while the speed falls as a car's does");
        }
    }

    // ---- I-28: the belt from g slackens with the posts, never on its own ----
    // The channel stream goes quiet whenever the game pauses or loses focus
    // while the motion stream (and the posts) carry on: the belt eases to
    // min and stays taut however long that lasts. It slackens by itself
    // only once the motion stream has been stale for the hold too (5 s
    // here), after a 2 s ride-out of the g's own; a motion stream never seen
    // this session never slackens it. A belt on "motion" keeps its ladder.
    {
        const double G = 9.80665;
        int tick = 0;
        // The g changes by a hair every other cycle (a live sender never
        // repeats a float exactly, and a frozen one reads as no g).
        auto f = [&](double accG, bool chanFresh, bool motionLive) {
            TelemetryData sd = frame(0.0, chanFresh, 65535.0);
            sd.valid = motionLive;
            sd.ncyHave[NcxTok::AccSurge] = true;
            sd.ncy[NcxTok::AccSurge] = accG * G + ((++tick & 1) ? 1e-4 : 0.0);
            return sd;
        };
        auto runFor = [&](MotionController& mc, double sec, double accG, bool chanFresh, bool motionLive) {
            double t = 0.0;
            const int cycles = static_cast<int>(sec / 0.002 + 0.5);
            for (int c = 0; c < cycles; ++c) t = run(mc, f(accG, chanFresh, motionLive), 1, 1).hi;
            return t;
        };
        auto belt = [&](const char* source, bool motionLive = true) {
            AppConfig cfg = makeBeltConfig(0.0, 1.0);    // torque 5..50, onlineHoldTimeoutSec 5 s
            DriveConfig& d = cfg.drives[0];
            d.beltSource = source; d.beltGMode = "decel";
            d.beltGFromG = 0.1; d.beltGFullG = 1.1; d.beltGSmoothMs = 0.0;
            auto mc = std::make_unique<MotionController>();
            mc->configure(cfg);
            tensionBelt(*mc);
            runFor(*mc, 0.8, 0.6, true, motionLive);
            return mc;
        };
        auto online = [](MotionController& mc) { return mc.getAxisState(0) == AxisMotionState::ONLINE; };
        {
            auto mc = belt("g");
            check(online(*mc) && std::fabs(runFor(*mc, 0.1, 0.6, true, true) - 27.5) < 1e-3,
                  "I-28 setup: a belt from g tracking braking");
            const double paused = runFor(*mc, 60.0, 0.6, false, true);   // a minute in the pause menu
            check(online(*mc) && std::fabs(paused - 5.0) < 1e-6,
                  "I-28 the g gone a minute, the motion stream live: taut at min, still ONLINE");
            check(std::fabs(runFor(*mc, 0.1, 0.6, true, true) - 27.5) < 1e-3,
                  "I-28 ...and the g takes over again when the game resumes (no re-tension)");
        }
        {
            auto mc = belt("g");
            const double early = runFor(*mc, 4.0, 0.6, false, false);
            check(online(*mc) && std::fabs(early - 5.0) < 1e-6,
                  "I-28 both streams gone 4 s: taut at min, ONLINE (inside the 5 s hold)");
            runFor(*mc, 1.5, 0.6, false, false);
            check(!online(*mc), "I-28 both streams gone past the hold: the belt slackens with the posts");
            const double slack = runFor(*mc, 1.0, 0.6, false, false);
            check(slack <= 1e-9 && mc->getAxisState(0) == AxisMotionState::PARKED,
                  "I-28 ...to slack (PARKED, torque 0)");
        }
        {
            auto mc = belt("g");
            const double live = runFor(*mc, 20.0, 0.6, true, false);
            check(online(*mc) && std::fabs(live - 27.5) < 1e-3,
                  "I-28 the motion stream gone 20 s, the g live: the belt keeps following the g");
            const double ride = runFor(*mc, 1.5, 0.6, false, false);
            check(online(*mc) && std::fabs(ride - 27.5) < 1e-3,
                  "I-28 ...then the g gone too: held through the 2 s ride-out");
            runFor(*mc, 1.0, 0.6, false, false);
            check(!online(*mc), "I-28 ...past it, it slackens (the motion stream long stale)");
        }
        {
            auto mc = belt("g", false);
            const double t = runFor(*mc, 30.0, 0.6, false, false);
            check(online(*mc) && std::fabs(t - 5.0) < 1e-6,
                  "I-28 no motion stream this session: the g gone 30 s, taut at min, never slack by itself");
        }
        {
            auto mc = belt("motion");
            runFor(*mc, 5.5, 0.6, true, false);
            check(!online(*mc), "I-28 a belt on motion: the motion line gone past the hold, it slackens as before");
        }
    }

    std::printf("TestHapticsIntegration: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
