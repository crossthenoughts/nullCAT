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
//   WHINE   straight-cut gears meshing: a tone whose pitch follows the
//           input shaft (the law sets the carrier from rpm and the gear,
//           so it steps on every shift) and whose level follows load.
//   SHUNT   backlash taking up when the throttle crosses zero torque: one
//           knock at shunt hz, harder in a dog box, scaled by how fast.
// All whole-car (no parts). Mixes and carriers per component; the shared
// jitter roughens the oscillators. Levels ramp like the other continuous
// effects; the shunt is a pulse.
//
// RT-safe: fixed state, pure arithmetic.
// ============================================================

#include "HapticsTypes.h"
#include "WaveSynth.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace haptics {

namespace driveline_k {
    constexpr double kAttackSec  = 0.05;
    constexpr double kReleaseSec = 0.15;
    constexpr double kMinHz      = 2.0;
}

struct DrivelineOut
{
    double judder = 0.0, lug = 0.0, whine = 0.0, shunt = 0.0;   // -1..1 x mix x level (shunt: a pulse)
    double judderHz = 0.0, lugHz = 0.0, whineHz = 0.0, shuntHz = 0.0;   // carriers for sink derating
    double judderEnv = 0.0, lugEnv = 0.0, whineEnv = 0.0;        // level x mix
    double judderPhase = 0.0, lugPhase = 0.0, whinePhase = 0.0;  // radians, for a shaker's harmonic
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
    // Whine: level 0..1 and the carrier (Hz) the law derived from rpm and
    // the gear this cycle. Not driven = releasing.
    void driveWhine(double level, double hz)
    {
        m_tW = std::max(0.0, std::min(1.0, level));
        m_whineHz = std::max(0.0, hz);
    }
    // Shunt: one knock of the given strength (0..1 x mix), now.
    void shunt(double strength)
    {
        int slot = 0;
        for (int i = 1; i < kShuntPool; ++i) if (m_shT[i] > m_shT[slot]) slot = i;
        m_shAmp[slot] = std::max(0.0, std::min(1.0, strength));
        m_shT[slot]   = 0.0;
    }

    void step(double dtSec, const EffectParams& p, const DrivelineParams& d)
    {
        ramp(m_lJ, m_tJ, dtSec); ramp(m_lL, m_tL, dtSec);
        m_tJ = m_tL = 0.0;
        m_out = DrivelineOut{};
        m_out.judder = component(m_lJ, d.clutchHz, d.clutch, p.jitter, m_oscJ, m_rng, dtSec, m_out.judderHz);
        m_out.lug    = component(m_lL, d.lugHz,    d.lug,    p.jitter, m_oscL, m_rng, dtSec, m_out.lugHz);
        m_out.judderEnv = (m_out.judderHz > 0.0) ? m_lJ * d.clutch : 0.0; m_out.judderPhase = m_oscJ.phase;
        m_out.lugEnv    = (m_out.lugHz > 0.0)    ? m_lL * d.lug    : 0.0; m_out.lugPhase    = m_oscL.phase;
        // Whine: the carrier is the law's, the mix is the tile's; a tone, so
        // no jitter on it.
        ramp(m_lW, m_tW, dtSec); m_tW = 0.0;
        m_out.whine = component(m_lW, m_whineHz, d.whine, 0.0, m_oscW, m_rng, dtSec, m_out.whineHz);
        m_out.whineEnv = (m_out.whineHz > 0.0) ? m_lW * d.whine : 0.0; m_out.whinePhase = m_oscW.phase;
        // Shunt pulses: a damped knock, one carrier cycle long.
        const double shuntHz = std::max(driveline_k::kMinHz, d.shuntHz);
        const double dur = 1.0 / shuntHz;
        m_out.shuntHz = shuntHz;
        for (int i = 0; i < kShuntPool; ++i)
        {
            if (m_shT[i] >= dur) continue;
            const double env = wavesynth::envelope(m_shT[i], dur, dur * 0.3);
            m_out.shunt += m_shAmp[i] * d.shunt * env * std::sin(2.0 * wavesynth::kPi * (m_shT[i] / dur));
            m_shT[i] += dtSec;
        }
    }

    const DrivelineOut& out() const { return m_out; }
    double level() const
    {
        double sh = 0.0; for (int i = 0; i < kShuntPool; ++i) if (m_shT[i] < 1.0) sh = std::max(sh, m_shAmp[i]);
        return std::max(std::max(m_lJ, m_lL), std::max(m_lW, sh));
    }
    double judderLevel() const { return m_lJ; }
    double lugLevel() const    { return m_lL; }
    double whineLevel() const  { return m_lW; }
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

    static constexpr int kShuntPool = 2;
    double m_tJ = 0.0, m_tL = 0.0, m_lJ = 0.0, m_lL = 0.0;
    double m_tW = 0.0, m_lW = 0.0, m_whineHz = 0.0;
    double m_shT[kShuntPool] = { 1e9, 1e9 }, m_shAmp[kShuntPool] = { 0.0, 0.0 };
    wavesynth::Oscillator m_oscJ, m_oscL, m_oscW;
    uint64_t m_rng = 0x9E3779B97F4A7C15ull;
    DrivelineOut m_out;
};

} // namespace haptics
