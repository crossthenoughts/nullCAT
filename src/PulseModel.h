// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// PulseModel - the ABS and TC tiles.
//
// Each cycle is a drop and a recovery (brake pressure dumped then rebuilt;
// drive cut then restored), so the car's deceleration or push saw-tooths.
// What makes a real one feel real rather than a metronome:
//   sharp   the edge of the drop: 0 a round wave (the drop takes half the
//           cycle), 1 the tile's knock (ABS dumps in a quarter, TC cuts in
//           a third);
//   spread  each cycle a little longer or shorter and shallower than the
//           last, and (ABS) each corner's valve on its own rate, so the
//           four drift in and out of step and the car grumbles instead of
//           beating;
//   slow    (ABS) the cycle slowing as the car slows, the thump-thump
//           before a stop (full rate from kSlowFullKmh up);
//   buzz    (ABS) the pump and valves under it, a fine rough buzz.
// ABS runs four channels (the corners) and a route takes a Part: one
// corner plays its own pulse, an axle or all plays its corners averaged, as
// the body feels the sum of the wheels' braking (in step the full pulse,
// drifting apart they partly cancel: the grumble). spread 0 keeps the four
// in step, one pulse. TC runs one channel. Every channel averages to zero,
// so a position axis does not drift.
//
// RT-safe: fixed arrays, pure arithmetic, no allocation.
// ============================================================

#include "HapticsTypes.h"
#include "NcxTokens.h"      // Wheel order
#include "WaveSynth.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace haptics {

namespace pulse_k {
    constexpr double kAbsDrop    = 0.25;   // sharp 1: ABS dumps in a quarter of the cycle
    constexpr double kTcDrop     = 0.33;   // sharp 1: TC cuts in a third
    constexpr double kPeriodVar  = 0.3;    // spread 1: each cycle up to +-30% long
    constexpr double kDepthVar   = 0.35;   // spread 1: each cycle up to 35% shallower
    // spread 1: each corner's own rate around freq hz, and where in its
    // cycle it starts (FL, FR, RL, RR)
    constexpr double kCornerRate[WHEEL_COUNT]  = { -0.15, 0.12, -0.07, 0.18 };
    constexpr double kCornerPhase[WHEEL_COUNT] = { 0.0, 0.5, 0.25, 0.75 };
    constexpr double kSlowFullKmh = 80.0;  // full cycle rate from here up
    constexpr double kBuzzJitter  = 0.3;   // the pump is not a tone
}

// One cycle at u (0..1 through it): from +1 down to -1 over `drop` of the
// cycle, back up over the rest, both halves cosine-shaped.
inline double pulseShape(double u, double drop)
{
    u -= std::floor(u);
    drop = std::max(0.05, std::min(0.95, drop));
    return (u < drop) ? std::cos(wavesynth::kPi * u / drop)
                      : -std::cos(wavesynth::kPi * (u - drop) / (1.0 - drop));
}

// The drop's share of the cycle for an edge: sharp 0 = half (round), 1 =
// the tile's own knock.
inline double pulseDrop(double sharp, double knockDrop)
{
    const double s = std::max(0.0, std::min(1.0, sharp));
    return 0.5 - s * (0.5 - knockDrop);
}

class PulseModel
{
public:
    // channels: 4 for ABS (the corners), 1 for TC.
    PulseModel(int channels, double knockDrop) : m_n(std::max(1, std::min(static_cast<int>(WHEEL_COUNT), channels))), m_knockDrop(knockDrop)
    { clear(); }

    // Road speed (km/h); negative = unknown (no slowing). Per cycle.
    void driveSpeed(double kmh) { m_kmh = std::isfinite(kmh) ? kmh : -1.0; }

    void step(double dtSec, double freqHz, const PulseParams& q)
    {
        using namespace pulse_k;
        const double spread = std::max(0.0, std::min(1.0, q.spread));
        const double slow   = std::max(0.0, std::min(1.0, q.slow));
        const double speedK = (m_kmh < 0.0) ? 1.0 : 1.0 - slow * (1.0 - std::min(1.0, m_kmh / kSlowFullKmh));
        m_rateHz = std::max(0.5, freqHz) * speedK;
        const double drop = pulseDrop(q.sharp, m_knockDrop);
        for (int c = 0; c < m_n; ++c)
        {
            Ch& ch = m_ch[c];
            const double cornerK = (m_n > 1) ? 1.0 + spread * kCornerRate[c] : 1.0;
            ch.u += m_rateHz * cornerK * ch.periodK * dtSec;
            if (ch.u >= 1.0)
            {
                ch.u -= std::floor(ch.u);
                ch.periodK = 1.0 / (1.0 + spread * kPeriodVar * (2.0 * rand01(ch.rng) - 1.0));
                ch.depth   = 1.0 - spread * kDepthVar * rand01(ch.rng);
            }
            ch.out = ch.depth * pulseShape(ch.u + ((m_n > 1) ? spread * kCornerPhase[c] : 0.0), drop);
        }
        // The pump and valves: a rough buzz at buzz hz.
        m_buzzHz = 0.0; m_buzzOut = 0.0;
        if (q.buzz > 0.0 && q.buzzHz > 0.0)
        {
            const double f = q.buzzHz * (1.0 + kBuzzJitter * (rand01(m_buzzRng) - 0.5));
            m_buzzOut = std::min(1.0, q.buzz) * m_buzzOsc.step(f, dtSec);
            m_buzzHz = q.buzzHz;
        }
    }

    // The pulse for a route's part (ABS: the corners of the part averaged;
    // a single corner plays its own; TC: its one channel).
    double pulseFor(Part part) const
    {
        if (m_n == 1) return m_ch[0].out;
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
        double sum = 0.0;
        for (int c = first; c <= last; ++c) sum += m_ch[c].out;
        return sum / static_cast<double>(last - first + 1);
    }
    double channel(int c) const { return (c >= 0 && c < m_n) ? m_ch[c].out : 0.0; }
    double rateHz() const  { return m_rateHz; }
    double buzz() const    { return m_buzzOut; }
    double buzzHz() const  { return m_buzzHz; }
    double buzzPhase() const { return m_buzzOsc.phase; }

    void clear()
    {
        uint64_t seed = 0xA5A5F00DCAFEBEEFull;
        for (Ch& ch : m_ch) { ch = Ch{}; ch.rng = seed; ch.u = 0.0; seed = seed * 6364136223846793005ull + 1442695040888963407ull; }
        m_buzzOsc = wavesynth::Oscillator{}; m_buzzRng = 0x2545F4914F6CDD1Dull;
        m_rateHz = m_buzzHz = m_buzzOut = 0.0;
    }

private:
    static double rand01(uint64_t& r)
    {
        r ^= r << 13; r ^= r >> 7; r ^= r << 17;
        return static_cast<double>(r & 0xFFFFFF) / 16777215.0;
    }
    struct Ch { double u = 0.0, periodK = 1.0, depth = 1.0, out = 0.0; uint64_t rng = 1; };
    Ch     m_ch[WHEEL_COUNT];
    int    m_n;
    double m_knockDrop;
    double m_kmh = -1.0, m_rateHz = 0.0;
    wavesynth::Oscillator m_buzzOsc;
    uint64_t m_buzzRng = 0x2545F4914F6CDD1Dull;
    double m_buzzHz = 0.0, m_buzzOut = 0.0;
};

} // namespace haptics
