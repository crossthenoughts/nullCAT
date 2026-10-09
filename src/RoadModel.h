// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// RoadModel - per-corner road replay behind the Road tile.
//
// When the sim sends suspension velocity for all four corners, the Road
// effect stops synthesising a texture and REPLAYS the road: each corner's
// velocity is integrated into travel through a leaky integrator (a first-
// order high-pass on the integral), so the slow body motion the motion
// cue already produces (heave, pitch, roll, under ~2 Hz) is removed and
// only the bumps remain, with their real timing and shape. Routed with a
// Part, a four-post rig's FL actuator replays the FL corner.
//
// Output per wheel is travel / fullMm, clamped to +-1: a route gain then
// means "this much at a fullMm bump", exactly like every other effect.
//
// RT-safe: fixed arrays, pure arithmetic.
// ============================================================

#include "HapticsTypes.h"
#include "NcxTokens.h"      // Wheel order
#include "WaveSynth.h"
#include <algorithm>
#include <cmath>

namespace haptics {

namespace road_k {
    constexpr double kMinHpHz    = 0.5;
    constexpr double kMinFullMm  = 0.5;
    constexpr double kReleaseSec = 0.120;   // fade when the stream stops driving
    constexpr double kDerateHz   = 8.0;     // representative bump rate for position-sink derating
}

class RoadModel
{
public:
    // Per wheel, per cycle, BEFORE step(): suspension velocity, mm/s, signed.
    void drive(int wheel, double velMmS)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        if (!std::isfinite(velMmS)) velMmS = 0.0;
        m_w[wheel].v = velMmS;
        m_w[wheel].dx = 0.0;
        m_w[wheel].driven = true;
    }
    // ...or suspension travel, mm, signed, for sims that give position
    // and not velocity (Assetto Corsa). The change since the last sample
    // enters the same high-passed integral, so a held sample (the sender
    // runs slower than the loop) adds nothing until the next one lands.
    void driveTravel(int wheel, double travelMm)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        if (!std::isfinite(travelMm)) return;
        W& w = m_w[wheel];
        w.dx = w.haveTravel ? (travelMm - w.lastTravel) : 0.0;
        w.lastTravel = travelMm; w.haveTravel = true;
        w.v = 0.0;
        w.driven = true;
    }

    void step(double dtSec, const RoadParams& p)
    {
        using namespace road_k;
        const double hp   = std::max(kMinHpHz, p.hpHz);
        const double leak = std::exp(-2.0 * wavesynth::kPi * hp * dtSec);
        const double full = std::max(kMinFullMm, p.fullMm);
        for (W& w : m_w)
        {
            if (w.driven)
            {
                w.x = w.x * leak + w.v * dtSec + w.dx;
                w.gate = 1.0;
            }
            else
            {
                // Not driven this cycle (stream gone): the travel decays on
                // the same leak and the gate fades, so nothing holds.
                w.x *= leak;
                w.gate = std::max(0.0, w.gate - dtSec / kReleaseSec);
                w.haveTravel = false;
            }
            w.driven = false; w.dx = 0.0;
            w.s = std::max(-1.0, std::min(1.0, w.x / full)) * w.gate;
        }
    }

    // The corner with the most travel among the part's wheels.
    double outputFor(Part part) const
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
        double best = 0.0;
        for (int i = first; i <= last; ++i)
            if (std::fabs(m_w[i].s) > std::fabs(best)) best = m_w[i].s;
        return best;
    }

    double level() const
    {
        double m = 0.0;
        for (const W& w : m_w) m = std::max(m, std::fabs(w.s));
        return m;
    }

    double wheelTravelMm(int wheel) const
    {
        return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].x : 0.0;
    }

    bool active() const
    {
        for (const W& w : m_w) if (w.gate > 0.0) return true;
        return false;
    }

    void clear() { for (W& w : m_w) w = W{}; }

private:
    struct W
    {
        double v = 0.0;       // driven velocity this cycle
        double dx = 0.0;      // driven travel change this cycle (travel path)
        double x = 0.0;       // high-passed travel, mm
        double s = 0.0;       // this cycle's sample, -1..1
        double gate = 0.0;    // 1 while driven, fades when not
        double lastTravel = 0.0;
        bool   haveTravel = false;
        bool   driven = false;
    };
    W m_w[WHEEL_COUNT];
};

} // namespace haptics
