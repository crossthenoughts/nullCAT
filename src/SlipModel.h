// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// SlipModel - the per-wheel tyre slip synthesiser behind the Lateral slip
// and Longitudinal slip tiles.
//
// Four wheels, two components. Each wheel carries its own severity for
// component A and component B (lateral: A = front scrub, B = rear slide;
// longitudinal: A = lock judder, B = spin tramp), its own attack/release
// ramp and its own oscillator, so the inside front locking alone judders
// ALONE. A route then asks for a Part (all, an axle, a corner) and gets
// the strongest wheel of that part per component: selecting, never
// summing, so two sines at nearly the same carrier can never comb.
//
// Severity staging, from the real thing: as slip grows past the limit the
// stick-slip chatter slows and roughens (squeal, moan, shudder). The
// carrier falls by kStageDrop at full severity and the jitter grows. The
// onset is abrupt (kAttackSec) because a tyre lets go in milliseconds;
// the release is the usual fade.
//
// RT-safe: fixed arrays, no allocation, pure arithmetic.
// ============================================================

#include "HapticsTypes.h"
#include "NcxTokens.h"      // Wheel order
#include "WaveSynth.h"
#include <algorithm>
#include <cstdint>

namespace haptics {

namespace slip_k {
    constexpr double kAttackSec  = 0.008;   // a tyre lets go in ~5-10 ms
    constexpr double kReleaseSec = 0.120;
    constexpr double kStageDrop  = 0.35;    // carrier at full severity = (1 - this) x set hz
    constexpr double kStageRough = 0.6;     // jitter grows by this share at full severity
    constexpr double kMinHz      = 2.0;
}

class SlipModel
{
public:
    struct Out
    {
        double a = 0.0, b = 0.0;          // samples, -1..1 x mix x level
        double aHz = 0.0, bHz = 0.0;      // carriers (unjittered) for sink derating
        double aEnv = 0.0, bEnv = 0.0;    // the non-oscillatory part (level x mix)
        double aPhase = 0.0, bPhase = 0.0;// oscillator phase, radians: a shaker plays sin(harm x phase)
    };

    // Per wheel, per cycle, BEFORE step(): component severities 0..1.
    // A wheel not driven this cycle releases.
    void drive(int wheel, double a, double b)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        W& w = m_w[wheel];
        w.tA = std::max(0.0, std::min(1.0, a));
        w.tB = std::max(0.0, std::min(1.0, b));
    }

    // Carrier multipliers applied on top of the set hz (lock judder falls
    // with road speed; 1 = as set).
    void setCarrierScale(double aScale, double bScale)
    {
        m_aScale = std::max(0.1, aScale);
        m_bScale = std::max(0.1, bScale);
    }

    void step(double dtSec, const EffectParams& p, const SlipParams& s)
    {
        using namespace slip_k;
        for (W& w : m_w)
        {
            ramp(w.lA, w.tA, dtSec);
            ramp(w.lB, w.tB, dtSec);
            w.tA = w.tB = 0.0;    // a law that stops driving = release
            w.sA = component(w.lA, s.aHz * m_aScale, s.aMix, p.jitter, w.oscA, w.rng, dtSec, w.hzA);
            w.sB = component(w.lB, s.bHz * m_bScale, s.bMix, p.jitter, w.oscB, w.rng, dtSec, w.hzB);
            w.envA = (w.hzA > 0.0) ? w.lA * s.aMix : 0.0;
            w.envB = (w.hzB > 0.0) ? w.lB * s.bMix : 0.0;
        }
    }

    // The strongest wheel of the part, per component.
    Out outputFor(Part part) const
    {
        int first = 0, last = WHEEL_COUNT - 1;
        switch (part)
        {
            case Part::Front: first = WheelFL; last = WheelFR; break;
            case Part::Rear:  first = WheelRL; last = WheelRR; break;
            case Part::FL:    first = last = WheelFL; break;
            case Part::FR:    first = last = WheelFR; break;
            case Part::RL:    first = last = WheelRL; break;
            case Part::RR:    first = last = WheelRR; break;
            default: break;
        }
        Out o;
        double bestA = 0.0, bestB = 0.0;
        for (int i = first; i <= last; ++i)
        {
            const W& w = m_w[i];
            if (w.lA > bestA) { bestA = w.lA; o.a = w.sA; o.aHz = w.hzA; o.aEnv = w.envA; o.aPhase = w.oscA.phase; }
            if (w.lB > bestB) { bestB = w.lB; o.b = w.sB; o.bHz = w.hzB; o.bEnv = w.envB; o.bPhase = w.oscB.phase; }
        }
        return o;
    }

    // Highest smoothed severity anywhere (status / wave).
    double level() const
    {
        double m = 0.0;
        for (const W& w : m_w) m = std::max(m, std::max(w.lA, w.lB));
        return m;
    }

    double wheelLevel(int wheel) const
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return 0.0;
        return std::max(m_w[wheel].lA, m_w[wheel].lB);
    }

    void clear()
    {
        for (W& w : m_w) { w = W{}; }
    }

private:
    struct W
    {
        double tA = 0.0, tB = 0.0;    // driven this cycle
        double lA = 0.0, lB = 0.0;    // smoothed severity
        double sA = 0.0, sB = 0.0;    // this cycle's samples
        double hzA = 0.0, hzB = 0.0;  // this cycle's carriers
        double envA = 0.0, envB = 0.0;// this cycle's envelopes (level x mix)
        wavesynth::Oscillator oscA, oscB;
        uint64_t rng = 0x9E3779B97F4A7C15ull;
    };

    static void ramp(double& level, double target, double dtSec)
    {
        using namespace slip_k;
        const double rate = (target > level) ? dtSec / kAttackSec : dtSec / kReleaseSec;
        level += std::max(-rate, std::min(rate, target - level));
    }

    static double component(double level, double setHz, double mix, double jitter,
                            wavesynth::Oscillator& osc, uint64_t& rng, double dtSec, double& hzOut)
    {
        using namespace slip_k;
        hzOut = 0.0;
        if (level < 1e-4 || mix <= 0.0 || setHz <= 0.0) return 0.0;
        const double carrier = std::max(kMinHz, setHz * (1.0 - kStageDrop * level));
        hzOut = carrier;
        double f = carrier;
        const double jit = jitter * (1.0 + kStageRough * level);
        if (jit > 0.0)
        {
            rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
            const double r = static_cast<double>(rng & 0xFFFF) / 65535.0;
            f *= 1.0 + jit * (r - 0.5);
        }
        return osc.step(f, dtSec) * level * mix;
    }

    W      m_w[WHEEL_COUNT];
    double m_aScale = 1.0, m_bScale = 1.0;
};

} // namespace haptics
