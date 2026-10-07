// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// DrivelineModel - the Driveline tile: what the transmission does to the
// car when the engine and the wheels disagree.
//
//   JUDDER  a slipping clutch grabbing and releasing: the whole driveline
//           shuddering at its own resonance (8-12 Hz) while the clutch is
//           part-way engaged with slip across it, at a launch or a bad
//           downshift. Severity from how much slip, how much load, and how
//           far into the slipping band the pedal is.
//   LUG     the engine bogged: full throttle at too few revs winds the
//           driveline up and lets it go, a slow shudder (5-9 Hz) that
//           fades as the revs climb out of it.
// Both are whole-car (no parts). Each has a mix and a carrier; the shared
// jitter roughens them. Levels ramp like the other continuous effects.
//
// RT-safe: fixed state, pure arithmetic.
// ============================================================

#include "HapticsTypes.h"
#include "WaveSynth.h"
#include <algorithm>
#include <cstdint>

namespace haptics {

namespace driveline_k {
    constexpr double kAttackSec  = 0.05;
    constexpr double kReleaseSec = 0.15;
    constexpr double kMinHz      = 2.0;
}

struct DrivelineOut
{
    double judder = 0.0, lug = 0.0;       // -1..1 x mix x level
    double judderHz = 0.0, lugHz = 0.0;   // carriers for sink derating
};

class DrivelineModel
{
public:
    // Per cycle, BEFORE step(): severities 0..1. Not driven = releasing.
    void drive(double judder, double lug)
    {
        m_tJ = std::max(0.0, std::min(1.0, judder));
        m_tL = std::max(0.0, std::min(1.0, lug));
    }

    void step(double dtSec, const EffectParams& p, const DrivelineParams& d)
    {
        ramp(m_lJ, m_tJ, dtSec); ramp(m_lL, m_tL, dtSec);
        m_tJ = m_tL = 0.0;
        m_out = DrivelineOut{};
        m_out.judder = component(m_lJ, d.clutchHz, d.clutch, p.jitter, m_oscJ, m_rng, dtSec, m_out.judderHz);
        m_out.lug    = component(m_lL, d.lugHz,    d.lug,    p.jitter, m_oscL, m_rng, dtSec, m_out.lugHz);
    }

    const DrivelineOut& out() const { return m_out; }
    double level() const { return std::max(m_lJ, m_lL); }
    double judderLevel() const { return m_lJ; }
    double lugLevel() const    { return m_lL; }
    void clear() { *this = DrivelineModel{}; }

private:
    static void ramp(double& level, double target, double dtSec)
    {
        using namespace driveline_k;
        const double rate = (target > level) ? dtSec / kAttackSec : dtSec / kReleaseSec;
        level += std::max(-rate, std::min(rate, target - level));
    }
    static double component(double level, double setHz, double mix, double jitter,
                            wavesynth::Oscillator& osc, uint64_t& rng, double dtSec, double& hzOut)
    {
        using namespace driveline_k;
        hzOut = 0.0;
        if (level < 1e-4 || mix <= 0.0 || setHz <= 0.0) return 0.0;
        const double carrier = std::max(kMinHz, setHz);
        hzOut = carrier;
        double f = carrier;
        if (jitter > 0.0)
        {
            rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
            const double r = static_cast<double>(rng & 0xFFFF) / 65535.0;
            f *= 1.0 + jitter * (r - 0.5);
        }
        return osc.step(f, dtSec) * level * mix;
    }

    double m_tJ = 0.0, m_tL = 0.0, m_lJ = 0.0, m_lL = 0.0;
    wavesynth::Oscillator m_oscJ, m_oscL;
    uint64_t m_rng = 0x9E3779B97F4A7C15ull;
    DrivelineOut m_out;
};

} // namespace haptics
