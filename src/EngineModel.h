// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// EngineModel - the engine haptic effect, as physics rather than a tone.
//
// What you feel from an engine is three things, and the model is those
// three things mixed:
//   ROCK   the block rocking on its mounts at crank rate. Full at idle,
//          gone by ~2500 rpm. A triple shakes, a V is smoother than an
//          inline, a boxer cancels most of it, a Wankel has none.
//   THUMPS each firing as a short low thump (fixed carrier, "thump hz"),
//          overlapping in a small pool as rpm rises. Displacement per
//          cylinder sets how hard each one hits. Once firings come faster
//          than can be resolved they fade and the buzz carries the engine.
//   BUZZ   the firing-order vibration through the structure, pitch
//          proportional to rpm at an order the actuator can carry, level
//          rising from ~15% of the redline to full at ~55% and building
//          to the top.
// The limiter is a CUT GATE (whole bursts of firings dropped, the engine
// catching again with a lurch on each return), not random misfires, which
// only ever read as "a different rpm". The redline is set or LEARNED
// (peak hold, snapped exactly on the first limiter hit).
//
// Header-only and RT-safe like the layer: fixed state, no allocation.
// Every constant that shapes the feel is named below with its units.
// ============================================================

#include "HapticsTypes.h"
#include "WaveSynth.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace haptics {

// Engine description + mix, on top of the effect's shared EffectParams
// (ampPct, freqHz = thump carrier, jitter = lope, routes).
struct EngineParams
{
    double cylinders = 4.0;   // cylinders, or rotors when layout is Wankel (1..16)
    double litres    = 2.0;   // total displacement (0.1..30)
    double layout    = 0.0;   // 0 inline, 1 V, 2 flat/boxer, 3 Wankel
    double maxRpm    = 0.0;   // redline; 0 = learn (peak hold, limiter snap)
    double rock      = 1.0;   // mix 0..1
    double thump     = 1.0;   // mix 0..1
    double buzz      = 1.0;   // mix 0..1
    double order     = 0.0;   // buzz carrier order x crank; 0 = auto from the redline
    double limHit    = 1.0;   // limiter return-hit strength 0..2 (1 = full load)
    double limHz     = 12.0;  // limiter cut rate 4..30 Hz
    double limJit    = 0.0;   // limiter cut-timing irregularity 0..1
};

namespace engine_k {
    constexpr double kThumpDefaultHz      = 30.0;   // thump carrier when freqHz is unset
    constexpr double kThumpMinHz          = 5.0;    // below this freqHz means "unset"
    constexpr double kThumpMix            = 0.6;    // share of amp a full thump takes
    constexpr double kRockMix             = 0.4;    // share of amp the rock takes
    constexpr double kBuzzMix             = 0.5;    // share of amp the buzz takes
    constexpr double kUnitPerCylLitres    = 0.5;    // 0.5 L/cyl = unit impulse weight
    constexpr double kHeavyMin            = 0.4, kHeavyMax = 1.6;
    constexpr double kBalanceNum          = 2.3;    // balance = num / sqrt(cyl)
    constexpr double kBalanceMin          = 0.3, kBalanceMax = 1.3;
    constexpr double kBalanceV            = 0.85;   // V smoother than inline
    constexpr double kBalanceFlat         = 0.7;    // boxer cancels most primary shake
    constexpr double kBalanceWankel       = 0.25;   // no reciprocating mass
    constexpr double kRockFullBelowHz     = 14.0;   // crank Hz: rock full below (840 rpm)
    constexpr double kRockGoneAboveHz     = 42.0;   // crank Hz: rock gone above (2520 rpm)
    constexpr double kRockLoadFloor       = 0.7;    // rock = floor + (1-floor) x load
    constexpr double kHalfOrderLope       = 0.6;    // half-order content x lope
    constexpr double kCoastHit            = 0.35;   // thump strength at closed throttle
    constexpr double kResolveFullOverlap  = 4.0;    // thumps fully felt up to this many per carrier cycle
    constexpr double kResolveFadeSpan     = 8.0;    // ...and gone this many beyond it
    constexpr double kBuzzBandTopHz       = 120.0;  // auto order aims the redline here
    constexpr double kBuzzCarrierMinHz    = 8.0, kBuzzCarrierMaxHz = 150.0;
    constexpr double kBuzzOrderMin        = 0.25, kBuzzOrderMax = 4.0;
    constexpr double kBuzzInAt            = 0.15;   // fraction of redline the buzz comes in
    constexpr double kBuzzFullBy          = 0.55;   // ...and is full-bodied by
    constexpr double kBuzzTopGrowth       = 0.25;   // extra level growth from full-bodied to redline
    constexpr double kBuzzLoadFloor       = 0.5;    // buzz = floor + (1-floor) x load
    constexpr double kBuzzHeavyFloor      = 0.6;    // buzz = floor + (1-floor) x heavy
    constexpr double kLearnSeedRpm        = 7000.0; // redline guess before anything is seen
    constexpr double kLimiterFlagAt       = 0.985;  // the plugin raises the flag at this fraction of max
    constexpr double kLearnMinRpm         = 1000.0; // ignore limiter flags below this
    constexpr double kCutDuty             = 0.5;    // fraction of each cut cycle that is silent
    constexpr double kCutJitSpan          = 0.8;    // +-40% at limJit 1
    constexpr double kCutHzMin            = 4.0, kCutHzMax = 30.0;
    constexpr double kBeatHz              = 2.5;    // rotary idle beat
    constexpr double kBeatDepth           = 0.5;    // ...at idle, fading with the rock
    constexpr int    kPulses              = 4;      // thump pool
}

// One cycle's output, kept as its three components with their carriers so
// a sink that has to derate by frequency (a position axis) can scale each
// part by what the actuator can follow at THAT part's frequency: the 13 Hz
// idle rock survives on a vertical where the 30 Hz thumps are tiny.
// total() is what a torque sink takes.
struct EngineOut
{
    double rock = 0.0, thumps = 0.0, buzz = 0.0;     // % of rated, before routing gain
    double rockHz = 0.0, thumpHz = 30.0, buzzHz = 0.0;
    double cap = 0.0;                                 // the effect's own amplitude ceiling this cycle
    double total() const { return std::max(-cap, std::min(cap, rock + thumps + buzz)); }
};

class EngineModel
{
public:
    // Per-cycle drive from the owner's law: firing rate, throttle load, limiter flag.
    void drive(double fireHz, double load01, bool limiterOn)
    {
        m_fireHz  = (fireHz > 0.0) ? fireHz : 0.0;
        m_load    = std::max(0.0, std::min(1.0, load01));
        m_limiter = limiterOn;
    }

    void clear()
    {
        for (double& t : m_pulseT) t = 1e9;
        m_firePhase = m_crankPhase = m_rockPhase = m_cutPhase = m_buzzPhase = m_beatPhase = 0.0;
        m_wasCut = false; m_revScale = 1.0;
    }

    double learnedMaxRpm() const { return m_learnedMax; }

    // One cycle. level = the effect's attack/release-smoothed drive level;
    // p the shared params (ampPct, freqHz, jitter); e the engine params;
    // rng the caller's xorshift state. Returns the value before routing
    // gain, in % of rated, already capped at the effect's own amplitude.
    EngineOut step(double dtSec, const EffectParams& p, const EngineParams& e, double level, uint64_t& rng)
    {
        using namespace engine_k;
        EngineOut out;
        const double thumpHz = (p.freqHz >= kThumpMinHz) ? p.freqHz : kThumpDefaultHz;
        m_pulseDur = 1.0 / thumpHz;
        out.thumpHz = thumpHz;
        const double cyl    = std::max(1.0, e.cylinders);
        const int    layout = static_cast<int>(e.layout + 0.5);
        const bool   rotary = (layout == 3);
        // Four-stroke: each cylinder fires every 2 revs. Wankel: each rotor
        // fires once per eccentric-shaft rev, so "cylinders" = rotors.
        const double crankHz = rotary ? m_fireHz / cyl : m_fireHz / (cyl / 2.0);

        const double perCyl  = std::max(0.05, e.litres) / cyl;
        const double heavy   = std::max(kHeavyMin, std::min(kHeavyMax, perCyl / kUnitPerCylLitres));
        const double layoutK = (layout == 1) ? kBalanceV : (layout == 2) ? kBalanceFlat : 1.0;
        const double balance = rotary ? kBalanceWankel
                             : layoutK * std::max(kBalanceMin, std::min(kBalanceMax, kBalanceNum / std::sqrt(cyl)));

        // Redline: set, or learned (peak hold; limiter flag snaps it exactly).
        const double rpmNow = crankHz * 60.0;
        if (m_fireHz >= 0.5)
        {
            if (rpmNow > m_learnedMax) m_learnedMax = rpmNow;
            if (m_limiter && rpmNow > kLearnMinRpm) m_learnedMax = rpmNow / kLimiterFlagAt;
        }
        const double maxRpm = (e.maxRpm > 0.0) ? e.maxRpm : m_learnedMax;
        const double x      = std::max(0.0, std::min(1.0, rpmNow / std::max(kLearnMinRpm, maxRpm)));
        const double rockFade = std::max(0.0, std::min(1.0,
            (kRockGoneAboveHz - crankHz) / (kRockGoneAboveHz - kRockFullBelowHz)));

        // Rotary idle beat: uneven combustion felt as a slow swell and fade.
        double beat = 1.0;
        if (rotary && m_fireHz >= 0.5)
        {
            m_beatPhase = wrap(m_beatPhase + kBeatHz * dtSec);
            beat = 1.0 + kBeatDepth * rockFade * std::sin(2.0 * wavesynth::kPi * m_beatPhase);
        }

        // Limiter cut gate, rate jittered per cut cycle.
        const double cutBase = std::max(kCutHzMin, std::min(kCutHzMax, e.limHz > 0.0 ? e.limHz : 12.0));
        bool cut = false;
        if (m_limiter && m_fireHz >= 0.5)
        {
            m_cutPhase += m_cutRate * dtSec;
            if (m_cutPhase >= 1.0)
            {
                m_cutPhase = wrap(m_cutPhase);
                m_cutRate  = cutBase * (1.0 + kCutJitSpan * std::max(0.0, std::min(1.0, e.limJit)) * (rand01(rng) - 0.5));
            }
            cut = (m_cutPhase < kCutDuty);
        }
        else { m_cutPhase = 0.0; m_cutRate = cutBase; }
        const bool returnHit = m_wasCut && !cut;
        m_wasCut = cut;
        const double limHit = std::max(0.0, std::min(2.0, e.limHit));

        if (m_fireHz >= 0.5)
        {
            m_crankPhase += crankHz * dtSec;
            if (m_crankPhase >= 1.0)
            {
                m_crankPhase = wrap(m_crankPhase);
                m_revScale = 1.0 + p.jitter * (rand01(rng) - 0.5) * 2.0;   // +-lope per rev
            }
            m_rockPhase = wrap(m_rockPhase + 0.5 * crankHz * dtSec);

            m_firePhase += m_fireHz * dtSec;
            if (m_firePhase >= 1.0)
            {
                m_firePhase = wrap(m_firePhase);
                if (!cut)
                {
                    const double hit     = m_limiter ? limHit : (kCoastHit + (1.0 - kCoastHit) * m_load);
                    const double overlap = m_fireHz * m_pulseDur;
                    const double norm    = 1.0 / std::max(1.0, std::sqrt(overlap));
                    const double resolve = std::max(0.0, std::min(1.0, 1.0 - (overlap - kResolveFullOverlap) / kResolveFadeSpan));
                    firePulse(p.ampPct * level * kThumpMix * e.thump * heavy * beat * hit * norm * resolve * m_revScale);
                }
            }
        }
        // The return from a cut is its own event: the engine catches again
        // with a lurch, at any rpm, scaled by limHit.
        if (returnHit && e.thump > 0.0)
            firePulse(p.ampPct * level * kThumpMix * e.thump * heavy * limHit);

        // Sum the thumps in flight.
        for (int s = 0; s < kPulses; ++s)
        {
            if (m_pulseT[s] >= m_pulseDur) continue;
            const double env = wavesynth::envelope(m_pulseT[s], m_pulseDur, m_pulseDur * 0.30);
            out.thumps += m_pulseAmp[s] * env * std::sin(2.0 * wavesynth::kPi * (m_pulseT[s] / m_pulseDur));
            m_pulseT[s] += dtSec;
        }

        // Rock: crank rate + half-order lope, loaded, sagging in a cut.
        if (m_fireHz >= 0.5 && !cut && rockFade > 0.0)
        {
            const double ph   = 2.0 * wavesynth::kPi * m_rockPhase;   // one cycle = 2 revs
            const double rock = std::sin(2.0 * ph) + kHalfOrderLope * p.jitter * std::sin(ph);
            out.rock = p.ampPct * level * kRockMix * e.rock * heavy * balance * rockFade
                     * (kRockLoadFloor + (1.0 - kRockLoadFloor) * m_load) * m_revScale * rock;
            out.rockHz = crankHz;
        }

        // Buzz: pitch proportional to rpm at an order the actuator can carry.
        if (m_fireHz >= 0.5 && e.buzz > 0.0)
        {
            const double order   = (e.order > 0.0) ? e.order
                                 : std::max(kBuzzOrderMin, std::min(kBuzzOrderMax, kBuzzBandTopHz / (maxRpm / 60.0)));
            const double carrier = std::max(kBuzzCarrierMinHz, std::min(kBuzzCarrierMaxHz, crankHz * order));
            m_buzzPhase = wrap(m_buzzPhase + carrier * dtSec);
            out.buzzHz = carrier;
            const double rise = std::max(0.0, std::min(1.0, (x - kBuzzInAt) / (kBuzzFullBy - kBuzzInAt)));
            if (rise > 0.0 && !cut)
            {
                const double lvl = rise * (1.0 - kBuzzTopGrowth + kBuzzTopGrowth * x)
                                 * (kBuzzLoadFloor + (1.0 - kBuzzLoadFloor) * m_load)
                                 * (m_limiter ? limHit : 1.0);
                out.buzz = p.ampPct * level * kBuzzMix * e.buzz * (kBuzzHeavyFloor + (1.0 - kBuzzHeavyFloor) * heavy) * lvl
                         * std::sin(2.0 * wavesynth::kPi * m_buzzPhase);
            }
        }

        // The engine never exceeds its own amplitude (x limHit on the limiter);
        // the axis clamp is the guard rail above this, not the shaping.
        out.cap = p.ampPct * level * (m_limiter ? std::max(1.0, limHit) : 1.0);
        return out;
    }

private:
    static double wrap(double ph) { return ph - static_cast<double>(static_cast<int>(ph)); }
    static double rand01(uint64_t& s)
    {
        s ^= s << 13; s ^= s >> 7; s ^= s << 17;
        return static_cast<double>(s & 0xFFFF) / 65535.0;
    }
    void firePulse(double amp)
    {
        int slot = 0;   // free slot, else the one closest to finishing
        for (int s = 1; s < engine_k::kPulses; ++s)
            if (m_pulseT[s] > m_pulseT[slot]) slot = s;
        m_pulseAmp[slot] = amp;
        m_pulseT[slot]   = 0.0;
    }

    double m_fireHz = 0.0, m_load = 0.5;
    bool   m_limiter = false;
    double m_firePhase = 0.0, m_crankPhase = 0.0, m_rockPhase = 0.0;
    double m_revScale = 1.0;
    double m_cutPhase = 0.0, m_cutRate = 12.0;
    bool   m_wasCut = false;
    double m_buzzPhase = 0.0, m_beatPhase = 0.0;
    double m_learnedMax = engine_k::kLearnSeedRpm;
    double m_pulseDur = 0.033;
    double m_pulseT[engine_k::kPulses]   = { 1e9, 1e9, 1e9, 1e9 };
    double m_pulseAmp[engine_k::kPulses] = { 0.0, 0.0, 0.0, 0.0 };
};

} // namespace haptics
