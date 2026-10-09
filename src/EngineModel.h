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
    double layout    = 0.0;   // 0 inline, 1 V, 2 flat/boxer, 3 Wankel, 4 two-stroke, 5 electric
    double maxRpm    = 0.0;   // redline; 0 = learn (peak hold, limiter snap)
    double rock      = 1.0;   // mix 0..1
    double thump     = 1.0;   // mix 0..1
    double buzz      = 1.0;   // mix 0..1
    double order     = 0.0;   // buzz carrier order x crank; 0 = auto from the redline
    double limHit    = 1.0;   // limiter return-hit strength 0..2 (1 = full load)
    double limHz     = 12.0;  // limiter cut rate 4..30 Hz
    double limJit    = 0.0;   // limiter cut-timing irregularity 0..1
    double inertia   = 0.5;   // mix 0..1: load-independent rpm^2 shake (what remains on a lift)
    double turbo     = 0.0;   // 0 no, 1 yes: the lift-off burst is a blow-off whoosh + flutter
    double liftoff   = 1.0;   // lift-off transient strength 0..2
    double pops      = 0.0;   // overrun pops 0..1 (fuel-cut crackle; 0 for a road car)
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
    constexpr double kBalanceTwoStroke    = 1.3;    // light flywheel, rigid mounts: shakes above its size
    constexpr double kTwoStrokeHeavy      = 1.5;    // a firing every rev hits harder per cc
    constexpr double kElectricLoadFloor   = 0.2;    // whine = floor + (1-floor) x load (regen whines too)
    constexpr double kElectricFullX       = 0.3;    // whine level full-bodied by this fraction of max rpm
    constexpr double kRockFullBelowHz     = 14.0;   // crank Hz: rock full below (840 rpm)
    constexpr double kRockGoneAboveHz     = 42.0;   // crank Hz: rock gone above (2520 rpm)
    constexpr double kRockLoadFloor       = 0.7;    // rock = floor + (1-floor) x load
    constexpr double kHalfOrderLope       = 0.6;    // half-order content x lope
    constexpr double kCoastHit            = 0.35;   // thump strength at closed throttle
    constexpr double kResolveFullOverlap  = 4.0;    // thumps fully felt up to this many per carrier cycle
    constexpr double kResolveFadeSpan     = 8.0;    // ...and gone this many beyond it
    constexpr double kBuzzBandTopHz       = 120.0;  // auto order aims the redline here (at 2 kHz and up)
    constexpr double kBuzzSamplesPerCycle = 8.0;    // the band top never asks for fewer samples per cycle than this
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
    // Traction control: the ECU cutting a SHARE of the firings in bursts
    // (spark or fuel to some cylinders), not all of them like the limiter:
    // a stutter whose depth rises with how hard the driver pushes.
    constexpr double kTcCutHz             = 15.0;   // burst rate
    constexpr double kTcCutJit            = 0.8;    // +-40% per burst: irregular
    constexpr double kTcDuty              = 0.5;    // share of each burst cycle that cuts
    constexpr double kTcDepthMin          = 0.3;    // share of firings dropped at light throttle...
    constexpr double kTcDepthMax          = 0.85;   // ...and at full throttle
    constexpr double kTcReturnHit         = 0.6;    // the drive coming back after a burst, x limHit x depth
    constexpr double kBeatHz              = 2.5;    // rotary idle beat
    constexpr double kBeatDepth           = 0.5;    // ...at idle, fading with the rock
    constexpr int    kPulses              = 4;      // thump pool
    // Inertia: pistons and rods reversing, rpm^2, no throttle in it.
    constexpr double kInertiaMix          = 0.5;    // share of amp a full inertia takes at the redline
    // Boost: cylinder pressure under boost makes each firing heavier.
    constexpr double kBoostRefBar         = 1.0;    // 1 bar of boost = kBoostGain more hit
    constexpr double kBoostGain           = 0.6;
    constexpr double kBoostHitMax         = 2.0;
    // Lift-off: a throttle drop from above kLiftFrom to below kLiftTo within
    // kLiftWindowSec, above kLiftMinX of the redline, is the lift event.
    constexpr double kLiftFrom            = 0.6, kLiftTo = 0.1;
    constexpr double kLiftWindowSec       = 0.15;
    constexpr double kLiftMinX            = 0.5;
    constexpr double kLiftPopHit          = 0.5;    // NA: one soft pop x liftoff
    constexpr double kLiftWhooshHit       = 1.0;    // turbo: the blow-off whoosh x liftoff
    constexpr double kLiftWhooshLen       = 2.5;    // ...this many thump lengths long
    constexpr int    kFlutterPulses       = 7;      // turbo: the compressor flutter after the whoosh
    constexpr double kFlutterHz           = 16.0;   // "stututu" chop rate
    constexpr double kFlutterHit          = 0.7;    // first flutter pulse x liftoff, decaying
    constexpr double kFlutterDecay        = 0.72;   // per pulse
    constexpr double kLiftHoldoffSec      = 0.6;    // one lift event per this long
    // Overrun pops: sparse random firings in the exhaust on a closed throttle.
    constexpr double kPopsRateHz          = 9.0;    // mean rate at pops 1 and the redline
    constexpr double kPopsMinX            = 0.3;
    constexpr double kPopsHitMin          = 0.3, kPopsHitMax = 1.0;
    // Overrun in gear: the wheels drive the engine against closed throttle
    // (compression pumping), heavier and rougher than a neutral coast-down.
    constexpr double kOverrunHit          = 0.5;    // thump hit, throttle shut, in gear and rolling
    constexpr double kOverrunRough        = 0.25;   // extra per-firing irregularity in that state
    // Starting and stopping. Below kCatchRpm with no combustion yet the
    // starter turns the engine over: slow compression lumps. The catch is
    // the first firings taking hold (a lurch, then an uneven flare). Below
    // kDyingRpm a running engine shudders harder as it dies, and stops
    // with one last kick.
    constexpr double kCatchRpm            = 500.0;
    constexpr double kCrankHit            = 0.6;    // compression lump x heavy while cranking
    constexpr double kCrankRock           = 0.5;    // the starter rocking the block
    constexpr double kCatchHit            = 1.4;    // the lurch as it catches
    constexpr double kCatchFlareSec       = 0.6;    // uneven fast idle after the catch
    constexpr double kCatchFlareLope      = 0.5;    // ...this much extra lope
    constexpr double kDyingRpm            = 650.0;  // a running engine below this is dying
    constexpr double kDyingHitGain        = 1.2;    // thumps up to (1 + this) x as it dies
    constexpr double kDyingLope           = 0.6;    // ...and this much extra lope
    constexpr double kStallKick           = 1.5;    // the last kick as it stops
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
    // For a shaker playing a harmonic: the rock as envelope + phase (radians
    // of the 2-rev cycle) + lope share, the buzz (and inertia) as envelope
    // + phase. Thumps are pulses and play as they are.
    double rockEnv = 0.0, rockPhase = 0.0, rockLope = 0.0;
    double buzzEnv = 0.0, buzzPhase = 0.0;
    // rock at a harmonic k: env x (sin(2k ph) + lope sin(k ph)); buzz: env x sin(k x buzzPhase).
    double rockAt(int k) const { return rockEnv * (std::sin(2.0 * k * rockPhase) + rockLope * std::sin(k * rockPhase)); }
    double buzzAt(int k) const { return buzzEnv * std::sin(k * buzzPhase); }
    double total() const { return std::max(-cap, std::min(cap, rock + thumps + buzz)); }
};

class EngineModel
{
public:
    // Per-cycle drive from the owner's law: firing rate, throttle load,
    // limiter flag, boost in bar (negative = the sim does not say).
    // pitLimiter: cuts like the rev limiter but never teaches the redline.
    // inGearOverrun: throttle shut while the wheels drive the engine (in
    // gear, rolling): heavier, rougher overrun than a neutral coast-down.
    // tcCut: traction control is cutting this cycle (a share of the firings
    // drops in irregular bursts, deeper with throttle).
    void drive(double fireHz, double load01, bool limiterOn, double boostBar = -1.0,
               bool pitLimiter = false, bool inGearOverrun = false, bool tcCut = false)
    {
        m_fireHz     = (fireHz > 0.0) ? fireHz : 0.0;
        m_load       = std::max(0.0, std::min(1.0, load01));
        m_limiter    = limiterOn || pitLimiter;
        m_revLimiter = limiterOn;
        m_boost      = (boostBar >= 0.0) ? boostBar : -1.0;
        m_inGearOverrun = inGearOverrun;
        m_tc         = tcCut;
    }

    void clear()
    {
        for (double& t : m_pulseT) t = 1e9;
        m_firePhase = m_crankPhase = m_rockPhase = m_cutPhase = m_buzzPhase = m_beatPhase = 0.0;
        m_wasCut = false; m_revScale = 1.0;
        m_tcPhase = 0.0; m_tcRate = engine_k::kTcCutHz; m_tcWasCut = false;
        m_hiLoadAgo = 1e9; m_liftHoldoff = 0.0; m_flutterLeft = 0; m_flutterT = 0.0; m_popWait = 0.0;
        m_running = false; m_sawCranking = false; m_catchFlare = 0.0;
    }

    // Lift-off events seen (tests, status).
    uint64_t liftOffCount() const { return m_liftOffs; }
    uint64_t tcDropCount() const  { return m_tcDrops; }    // firings traction control dropped
    bool     running() const      { return m_running; }   // combustion has taken hold
    uint64_t catchCount() const   { return m_catches; }
    uint64_t stallCount() const   { return m_stalls; }

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
        const bool   rotary = (layout == 3), twoStroke = (layout == 4), electric = (layout == 5);
        // Four-stroke: each cylinder fires every 2 revs. Wankel: each rotor
        // fires once per eccentric-shaft rev, so "cylinders" = rotors. Two-
        // stroke: every cylinder every rev. Electric: no firing at all, the
        // law passes motor rpm straight through (one "firing" per rev).
        const double crankHz = (rotary || twoStroke) ? m_fireHz / cyl : electric ? m_fireHz : m_fireHz / (cyl / 2.0);

        const double perCyl  = std::max(0.05, e.litres) / cyl;
        const double heavy   = std::max(kHeavyMin, std::min(kHeavyMax, perCyl / kUnitPerCylLitres)) * (twoStroke ? kTwoStrokeHeavy : 1.0);
        const double layoutK = (layout == 1) ? kBalanceV : (layout == 2) ? kBalanceFlat : twoStroke ? kBalanceTwoStroke : 1.0;
        const double balance = rotary ? kBalanceWankel : electric ? 0.0
                             : layoutK * std::max(kBalanceMin, std::min(kBalanceMax, kBalanceNum / std::sqrt(cyl)));

        // Redline: set, or learned (peak hold; the REV limiter flag snaps it
        // exactly; a pit limiter cuts but teaches nothing).
        const double rpmNow = crankHz * 60.0;
        if (m_fireHz >= 0.5)
        {
            if (rpmNow > m_learnedMax) m_learnedMax = rpmNow;
            if (m_revLimiter && rpmNow > kLearnMinRpm) m_learnedMax = rpmNow / kLimiterFlagAt;
        }

        // Running state: cranking below kCatchRpm until it catches; dying
        // below kDyingRpm once running; stopped with a kick when the revs
        // reach zero (or the sim drops them there).
        const bool turning = m_fireHz >= 0.5;
        bool cranking = false, catchNow = false, stallNow = false;
        if (electric) { m_running = turning; m_sawCranking = false; }
        else if (!m_running)
        {
            // The catch is only an event when the starter was seen first: an
            // engine already running when the stream begins just runs.
            if (turning && rpmNow >= kCatchRpm)
            {
                m_running = true;
                if (m_sawCranking) { catchNow = true; ++m_catches; m_catchFlare = kCatchFlareSec; }
                m_sawCranking = false;
            }
            else if (turning) { cranking = true; m_sawCranking = true; }
        }
        else if (!turning || rpmNow < kCatchRpm * 0.5)
        {
            // Stopped: one last kick, whether it died slowly or the sim
            // simply reported zero.
            m_running = false; stallNow = true; ++m_stalls;
        }
        const double dying = (m_running && rpmNow < kDyingRpm)
                           ? std::max(0.0, std::min(1.0, (kDyingRpm - rpmNow) / (kDyingRpm - kCatchRpm * 0.5))) : 0.0;
        if (m_catchFlare > 0.0) m_catchFlare -= dtSec;
        const double extraLope = (m_catchFlare > 0.0 ? kCatchFlareLope : 0.0) + kDyingLope * dying
                               + (m_inGearOverrun && m_load <= kLiftTo ? kOverrunRough : 0.0);
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
        if (m_limiter && m_fireHz >= 0.5 && !electric)
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

        // Traction control: bursts at an irregular rate, and within a burst
        // each firing dropped with a probability that rises with throttle.
        // The limiter, when it is in, already cuts everything.
        bool tcInCut = false;
        const double tcDepth = kTcDepthMin + (kTcDepthMax - kTcDepthMin) * m_load;
        if (m_tc && !m_limiter && m_fireHz >= 0.5 && !electric)
        {
            m_tcPhase += m_tcRate * dtSec;
            if (m_tcPhase >= 1.0)
            {
                m_tcPhase = wrap(m_tcPhase);
                m_tcRate  = kTcCutHz * (1.0 + kTcCutJit * (rand01(rng) - 0.5));
            }
            tcInCut = (m_tcPhase < kTcDuty);
        }
        else { m_tcPhase = 0.0; m_tcRate = kTcCutHz; }
        const bool tcReturn = m_tcWasCut && !tcInCut && m_tc;
        m_tcWasCut = tcInCut;

        if (m_fireHz >= 0.5 && !electric)
        {
            m_crankPhase += crankHz * dtSec;
            if (m_crankPhase >= 1.0)
            {
                m_crankPhase = wrap(m_crankPhase);
                m_revScale = 1.0 + std::min(1.0, p.jitter + extraLope) * (rand01(rng) - 0.5) * 2.0;   // +-lope per rev
            }
            m_rockPhase = wrap(m_rockPhase + 0.5 * crankHz * dtSec);

            m_firePhase += m_fireHz * dtSec;
            if (m_firePhase >= 1.0)
            {
                m_firePhase = wrap(m_firePhase);
                const bool tcDrop = tcInCut && !cranking && rand01(rng) < tcDepth;
                if (tcDrop) ++m_tcDrops;
                if (cranking)
                {
                    // No combustion yet: each compression stroke is a slow lump.
                    firePulse(p.ampPct * level * kThumpMix * e.thump * heavy * kCrankHit * m_revScale);
                }
                else if (!cut && !tcDrop)
                {
                    // Boost: more air per firing = a heavier hit, on top of load.
                    const double boostUp = (m_boost > 0.0) ? std::min(kBoostHitMax, 1.0 + kBoostGain * m_boost / kBoostRefBar) : 1.0;
                    // Closed throttle: in gear the wheels pump the engine
                    // (heavier), in neutral it freewheels (lighter).
                    const double floorHit = (m_inGearOverrun ? kOverrunHit : kCoastHit);
                    const double hit     = m_limiter ? limHit
                                         : std::min(kBoostHitMax, (floorHit + (1.0 - floorHit) * m_load) * boostUp)
                                           * (1.0 + kDyingHitGain * dying);
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
        // ...and so is the drive coming back after a traction-control burst,
        // lighter, as deep as the cut was.
        if (tcReturn && e.thump > 0.0)
            firePulse(p.ampPct * level * kThumpMix * e.thump * heavy * limHit * tcDepth * kTcReturnHit);
        // The catch (first firings taking hold) and the stall kick are
        // single heavy events in the thump pool.
        if (catchNow && e.thump > 0.0) firePulse(p.ampPct * level * kThumpMix * e.thump * heavy * kCatchHit, 1.5);
        if (stallNow && e.thump > 0.0) firePulse(p.ampPct * level * kThumpMix * e.thump * heavy * kStallKick, 2.0);

        // Lift-off: the throttle snapping shut at high rpm. Naturally
        // aspirated: one soft pop as combustion stops. Turbo: the boost
        // dumping (a longer, heavier whoosh) followed by the compressor
        // flutter, each scaled by the boost the sim reports or, without
        // it, by how far up the band the lift happened.
        {
            if (m_load >= kLiftFrom) m_hiLoadAgo = 0.0; else m_hiLoadAgo += dtSec;
            if (m_liftHoldoff > 0.0) m_liftHoldoff -= dtSec;
            const bool liftEdge = m_load <= kLiftTo && m_hiLoadAgo < kLiftWindowSec && m_hiLoadAgo > 0.0
                                  && x >= kLiftMinX && m_liftHoldoff <= 0.0 && m_fireHz >= 0.5;
            if (liftEdge && e.liftoff > 0.0 && !electric)
            {
                ++m_liftOffs;
                m_liftHoldoff = kLiftHoldoffSec;
                m_hiLoadAgo   = 1e9;
                const double how = (m_boost >= 0.0) ? std::min(1.0, m_boost / kBoostRefBar) : x;
                const double base = p.ampPct * level * kThumpMix * heavy * std::min(2.0, e.liftoff) * how;
                if (e.turbo >= 0.5)
                {
                    firePulse(base * kLiftWhooshHit, kLiftWhooshLen);
                    m_flutterLeft = kFlutterPulses;
                    m_flutterT    = 1.0 / kFlutterHz;    // first flutter pulse after the whoosh starts
                    m_flutterAmp  = base * kFlutterHit;
                }
                else
                    firePulse(base * kLiftPopHit);
            }
            if (m_flutterLeft > 0)
            {
                m_flutterT -= dtSec;
                if (m_flutterT <= 0.0)
                {
                    firePulse(m_flutterAmp);
                    m_flutterAmp *= kFlutterDecay;
                    m_flutterT   += 1.0 / kFlutterHz;
                    --m_flutterLeft;
                }
            }
        }

        // Overrun pops: with the throttle shut and the revs up, sparse random
        // firings in the exhaust, more often the higher the revs.
        if (e.pops > 0.0 && m_load <= kLiftTo && x >= kPopsMinX && m_fireHz >= 0.5 && !m_limiter && !electric)
        {
            m_popWait -= dtSec;
            if (m_popWait <= 0.0)
            {
                const double rate = kPopsRateHz * std::min(1.0, e.pops) * x;
                m_popWait = (0.3 + 1.4 * rand01(rng)) / std::max(0.5, rate);   // random spacing around 1/rate
                const double hit = kPopsHitMin + (kPopsHitMax - kPopsHitMin) * rand01(rng);
                firePulse(p.ampPct * level * kThumpMix * heavy * std::min(1.0, e.pops) * hit);
            }
        }
        else m_popWait = 0.0;

        // Sum the thumps in flight.
        for (int s = 0; s < kPulses; ++s)
        {
            const double dur = m_pulseDur * m_pulseLen[s];
            if (m_pulseT[s] >= dur) continue;
            const double env = wavesynth::envelope(m_pulseT[s], dur, dur * 0.30);
            out.thumps += m_pulseAmp[s] * env * std::sin(2.0 * wavesynth::kPi * (m_pulseT[s] / dur));
            m_pulseT[s] += dtSec;
        }

        // Rock: crank rate + half-order lope, loaded, sagging in a cut. While
        // cranking it is the starter rocking the block, lighter and unloaded.
        if (m_fireHz >= 0.5 && !cut && rockFade > 0.0)
        {
            const double ph   = 2.0 * wavesynth::kPi * m_rockPhase;   // one cycle = 2 revs
            const double lope = std::min(1.0, p.jitter + extraLope);
            const double rock = std::sin(2.0 * ph) + (twoStroke ? 0.0 : kHalfOrderLope * lope * std::sin(ph));
            const double loadK = cranking ? kCrankRock : (kRockLoadFloor + (1.0 - kRockLoadFloor) * m_load) * (1.0 + kDyingHitGain * 0.5 * dying);
            out.rock = p.ampPct * level * kRockMix * e.rock * heavy * balance * rockFade * loadK * m_revScale * rock;
            out.rockHz = crankHz;
            out.rockEnv = p.ampPct * level * kRockMix * e.rock * heavy * balance * rockFade * loadK * m_revScale;
            out.rockPhase = ph; out.rockLope = twoStroke ? 0.0 : kHalfOrderLope * lope;
        }

        // Buzz: pitch proportional to rpm at an order the actuator can carry.
        if (m_fireHz >= 0.5 && (e.buzz > 0.0 || e.inertia > 0.0))
        {
            // The auto band top follows the control rate: 120 Hz on a 2 kHz
            // loop, 62 Hz on a 500 Hz PC loop (4 samples per cycle at 120 Hz
            // was a coarse, buzzy stair). Set order by hand to override.
            const double bandTop = std::min(kBuzzBandTopHz, (1.0 / std::max(1e-4, dtSec)) / kBuzzSamplesPerCycle);
            const double order   = (e.order > 0.0) ? e.order
                                 : std::max(kBuzzOrderMin, std::min(kBuzzOrderMax, bandTop / (maxRpm / 60.0)));
            const double carrier = std::max(kBuzzCarrierMinHz, std::min(kBuzzCarrierMaxHz, crankHz * order));
            m_buzzPhase = wrap(m_buzzPhase + carrier * dtSec);
            out.buzzHz = carrier;
            out.buzzPhase = 2.0 * wavesynth::kPi * m_buzzPhase;
            // Electric: the motor whine, there from the first turn, carried by
            // load (regen whines too, at the floor), no firing-order band to
            // come in at and no cut. Otherwise the firing-order buzz.
            const double rise = electric
                              ? std::max(0.0, std::min(1.0, x / kElectricFullX))
                              : std::max(0.0, std::min(1.0, (x - kBuzzInAt) / (kBuzzFullBy - kBuzzInAt)));
            const double s    = std::sin(2.0 * wavesynth::kPi * m_buzzPhase);
            if (electric && rise > 0.0 && e.buzz > 0.0)
            {
                const double lvl = rise * (kElectricLoadFloor + (1.0 - kElectricLoadFloor) * m_load);
                out.buzzEnv = p.ampPct * level * kBuzzMix * e.buzz * lvl;
                out.buzz = out.buzzEnv * s;
            }
            else if (rise > 0.0 && !cut && e.buzz > 0.0)
            {
                const double lvl = rise * (1.0 - kBuzzTopGrowth + kBuzzTopGrowth * x)
                                 * (kBuzzLoadFloor + (1.0 - kBuzzLoadFloor) * m_load)
                                 * (m_limiter ? limHit : 1.0);
                out.buzzEnv = p.ampPct * level * kBuzzMix * e.buzz * (kBuzzHeavyFloor + (1.0 - kBuzzHeavyFloor) * heavy) * lvl;
                out.buzz = out.buzzEnv * s;
            }
            // Inertia: the reciprocating mass reversing, rpm^2 and nothing to
            // do with the throttle, so it is what remains on a lift and it
            // dissipates with the square of the falling revs. Keeps going
            // through a limiter cut (the engine still spins). Balance as
            // for the rock: a four shakes, a six or a twelve barely.
            if (e.inertia > 0.0 && !electric)
            {
                const double inEnv = p.ampPct * level * kInertiaMix * std::min(1.0, e.inertia) * balance * x * x;
                out.buzz += inEnv * s;
                out.buzzEnv += inEnv;
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
    // lenMul stretches one pulse (the blow-off whoosh) past the thump length.
    void firePulse(double amp, double lenMul = 1.0)
    {
        int slot = 0;   // free slot, else the one closest to finishing
        for (int s = 1; s < engine_k::kPulses; ++s)
            if (m_pulseT[s] / m_pulseLen[s] > m_pulseT[slot] / m_pulseLen[slot]) slot = s;
        m_pulseAmp[slot] = amp;
        m_pulseLen[slot] = std::max(0.25, lenMul);
        m_pulseT[slot]   = 0.0;
    }

    double m_fireHz = 0.0, m_load = 0.5;
    bool   m_limiter = false;
    double m_firePhase = 0.0, m_crankPhase = 0.0, m_rockPhase = 0.0;
    double m_revScale = 1.0;
    double m_cutPhase = 0.0, m_cutRate = 12.0;
    bool   m_wasCut = false;
    bool   m_tc = false, m_tcWasCut = false;
    double m_tcPhase = 0.0, m_tcRate = engine_k::kTcCutHz;
    uint64_t m_tcDrops = 0;
    double m_buzzPhase = 0.0, m_beatPhase = 0.0;
    double m_learnedMax = engine_k::kLearnSeedRpm;
    double m_pulseDur = 0.033;
    double m_pulseT[engine_k::kPulses]   = { 1e9, 1e9, 1e9, 1e9 };
    double m_pulseAmp[engine_k::kPulses] = { 0.0, 0.0, 0.0, 0.0 };
    double m_pulseLen[engine_k::kPulses] = { 1.0, 1.0, 1.0, 1.0 };   // x m_pulseDur
    double   m_boost = -1.0;
    double   m_hiLoadAgo = 1e9, m_liftHoldoff = 0.0;
    int      m_flutterLeft = 0;
    double   m_flutterT = 0.0, m_flutterAmp = 0.0;
    double   m_popWait = 0.0;
    uint64_t m_liftOffs = 0;
    bool     m_revLimiter = false, m_inGearOverrun = false;
    bool     m_running = false, m_sawCranking = false;
    double   m_catchFlare = 0.0;
    uint64_t m_catches = 0, m_stalls = 0;
};

} // namespace haptics
