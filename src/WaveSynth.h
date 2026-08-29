// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// WaveSynth - shared waveform primitives for the excitation engines.
//
// Consumed by CommissioningMode (test excitation) and DeviceForceModel
// (control-loading textures: grind, buffet). One vocabulary, tested once:
// the commissioning suite's envelope/continuity checks cover these
// expressions for every consumer. RT-safe by construction: pure
// arithmetic, fixed state, no allocation.
// ============================================================

#include <cmath>

namespace wavesynth
{

constexpr double kPi = 3.14159265358979323846;

// Raised-cosine ramp in/out with a unity hold. Guarantees exactly 0 at
// t <= 0 and t >= dur, so an enveloped segment can never step. (Moved
// verbatim from CommissioningMode 0.9.3; expression order preserved.)
inline double envelope(double t, double dur, double ramp)
{
    if (t <= 0.0 || t >= dur) return 0.0;
    if (2.0 * ramp > dur) ramp = dur / 2.0;
    if (t < ramp)        return 0.5 * (1.0 - std::cos(kPi * t / ramp));
    if (t > dur - ramp)  return 0.5 * (1.0 - std::cos(kPi * (dur - t) / ramp));
    return 1.0;
}

// Free-running sine oscillator: phase accumulates per step, so the
// frequency can change every cycle (a wire-driven texture) without a
// waveform discontinuity. Returns sin(phase) in [-1, 1].
struct Oscillator
{
    double phase = 0.0;
    double step(double freqHz, double dtSec)
    {
        phase += 2.0 * kPi * freqHz * dtSec;
        if (phase >= 2.0 * kPi) phase -= 2.0 * kPi;
        return std::sin(phase);
    }
    void reset() { phase = 0.0; }
};

} // namespace wavesynth
