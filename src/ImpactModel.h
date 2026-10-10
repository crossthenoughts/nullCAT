// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// ImpactModel - the Impacts tile: the edge of a hit through the suspension.
//
// The motion cue already moves the rig with a landing, a bump-stop hit or
// a kerb strike, but rounded off: telemetry arrives ~60 times a second and
// cueing filters, rate-limits and clips spikes. What it cannot pass on is
// the EDGE, the few milliseconds in which a corner's suspension velocity
// changes abruptly, and it never reaches a belt or a shaker. This model
// finds those edges and plays only them:
//   corner   a jump in a corner's suspension velocity between two sim
//            samples (the wheel stopped dead on its bump stop, a landing,
//            a kerb strike, the suspension topping out), from from mm/s,
//            full at full mm/s;
//   body     a jump in the body's vertical acceleration (a landing, a
//            heavy compression) from heave g, full at 3 x that, on every
//            corner.
// A knock is a ring at freq hz decaying over ring ms on a belt or shaker
// (it starts from zero and rises within a quarter cycle: the edge), and one
// short jolt on a post, a raised cosine back to where it started: up when
// a compression stopped dead or started (the bump stop, a landing, a kerb),
// down when an extension did (topping out, dropping into a hole). Each
// corner knocks at most every kMinGapSec unless the new hit is clearly
// harder.
//
// RT-safe: fixed arrays, pure arithmetic, no allocation.
// ============================================================

#include "HapticsTypes.h"
#include "NcxTokens.h"      // Wheel order
#include "RoadModel.h"      // RoadOut, partWheels
#include "WaveSynth.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace haptics {

namespace impact_k {
    constexpr double kMinGapSec   = 0.08;    // a corner knocks at most this often...
    constexpr double kHarderBy    = 1.5;     // ...unless the new hit is this much harder than what still rings
    constexpr double kThudHz      = 15.0;    // a post: one jolt at this rate
    constexpr double kG           = 9.80665;
    constexpr double kHeaveFullX  = 3.0;     // the body: full at this x heave g
    constexpr double kMinSampleSec = 0.002;  // a travel sample closer than this to the last is not a new sample
    constexpr double kMaxSampleSec = 0.25;   // ...nor one after a gap this long (the stream stalled)
    constexpr double kSampleLearn = 0.2;     // the sim's sample interval, learned from the travel's changes
    constexpr double kStillX      = 2.5;     // travel unchanged this many intervals after moving = stopped (one dropped frame is not)
    constexpr double kLevelSec    = 0.15;    // the tile's level: peak, falling over this
}

class ImpactModel
{
public:
    struct Out { double pos = 0.0, force = 0.0; };

    ImpactModel() { clear(); }

    // ---- inputs, per cycle, BEFORE step() (the held sim values; a new
    // sample is told by its value changing) ----
    // A corner's suspension velocity, mm/s (compression positive).
    void driveVelocity(int wheel, double mmS)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT || !std::isfinite(mmS)) return;
        C& c = m_c[wheel];
        c.velIn = mmS; c.velDriven = true;
    }
    // ...or its suspension travel, mm (compression positive), for sims that
    // give position and not velocity: the velocity is worked out per sample.
    void driveTravel(int wheel, double mm)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT || !std::isfinite(mm)) return;
        C& c = m_c[wheel];
        c.travelIn = mm; c.travelDriven = true;
    }
    // The body's vertical acceleration, m/s^2.
    void driveHeave(double ms2)
    {
        if (!std::isfinite(ms2)) return;
        m_heaveIn = ms2; m_heaveDriven = true;
    }
    // A knock on one corner now, severity 0..1, sign +1 compression / -1
    // top-out (a Test preview; the laws' own hits come through the inputs).
    void knock(int wheel, double severity, double sign)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        start(m_c[wheel], std::max(0.0, std::min(1.0, severity)), sign < 0.0 ? -1.0 : 1.0, true);
    }

    void step(double dtSec, const EffectParams& p, const ImpactParams& q)
    {
        using namespace impact_k;
        if (dtSec <= 0.0) return;
        const double from = std::max(1.0, q.fromMmS);
        const double full = std::max(from + 1.0, q.fullMmS);
        // The body: a jump in vertical acceleration between two samples.
        if (m_heaveDriven)
        {
            if (m_heaveSeen && m_heaveIn != m_heavePrev && q.heaveG > 0.0)
            {
                const double jumpG = std::fabs(m_heaveIn - m_heavePrev) / kG;
                const double s = (jumpG - q.heaveG) / ((kHeaveFullX - 1.0) * q.heaveG);
                if (s > 0.0)
                    for (C& c : m_c) start(c, std::min(1.0, s), (m_heaveIn > m_heavePrev) ? 1.0 : -1.0, false);
            }
            m_heavePrev = m_heaveIn; m_heaveSeen = true;
        }
        else m_heaveSeen = false;
        m_heaveDriven = false;

        const double ringHz = std::max(5.0, p.freqHz);
        const double ringTau = std::max(0.005, p.durMs / 1000.0);
        double inst = 0.0;
        for (C& c : m_c)
        {
            c.sinceSample += dtSec;
            c.sinceKnock  += dtSec;
            // The corner: one velocity per sim sample, from the sim or worked
            // out from its travel; a knock on a jump from the last sample.
            bool sample = false; double v = 0.0;
            if (c.velDriven)
            {
                c.travelSeen = false;
                if (!c.velSeen || c.velIn != c.velLast) { sample = true; v = c.velIn; }
                c.velLast = c.velIn; c.velSeen = true;
            }
            else if (c.travelDriven)
            {
                c.velSeen = false;
                if (!c.travelSeen) { c.travelLast = c.travelIn; c.travelSeen = true; c.sinceSample = 0.0; c.haveV = false; }
                else if (c.travelIn != c.travelLast && c.sinceSample >= kMinSampleSec)
                {
                    if (c.sinceSample <= kMaxSampleSec)
                    {
                        sample = true; v = (c.travelIn - c.travelLast) / c.sinceSample;
                        if (c.sinceSample < 3.0 * c.dtSample) c.dtSample += kSampleLearn * (c.sinceSample - c.dtSample);
                    }
                    else c.haveV = false;   // after a stall, start afresh
                    c.travelLast = c.travelIn; c.sinceSample = 0.0;
                }
                // Travel held still past the sample rate: the suspension is
                // not moving. After moving, that is a stop (a bump stop
                // leaves the travel where it is, so no new value arrives to
                // say so); from the start, a known rest, so the first move
                // (a landing from full droop) is a jump too.
                else if (c.sinceSample > kStillX * c.dtSample)
                {
                    if (!c.haveV) { c.vLast = 0.0; c.haveV = true; }
                    else if (c.vLast != 0.0) { sample = true; v = 0.0; }
                }
            }
            else { c.velSeen = c.travelSeen = c.haveV = false; }
            if (sample)
            {
                if (c.haveV)
                {
                    // Which way the body is kicked: by the movement that
                    // stopped or started, whichever was the faster. A
                    // compression stopped dead (the bump stop) or started
                    // (a landing, a kerb) kicks it up; an extension stopped
                    // (topping out) or started (dropping into a hole), down.
                    const double jump = v - c.vLast;
                    const double s = (std::fabs(jump) - from) / (full - from);
                    const double moved = (std::fabs(v) >= std::fabs(c.vLast)) ? v : c.vLast;
                    if (s > 0.0) start(c, std::min(1.0, s), moved >= 0.0 ? 1.0 : -1.0, false);
                }
                c.vLast = v; c.haveV = true;
            }
            c.velDriven = c.travelDriven = false;

            // The knock: a ring from zero (the edge) decaying over ring ms;
            // a post's jolt, one raised cosine at kThudHz.
            c.out = Out{};
            c.eNow = 0.0;
            if (c.env > 0.0)
            {
                c.t += dtSec;
                c.eNow = c.env * std::exp(-c.t / ringTau);
                c.phase += 2.0 * wavesynth::kPi * ringHz * dtSec;
                c.out.force = c.sign * c.eNow * std::sin(c.phase);
                if (c.t < 1.0 / kThudHz)
                    c.out.pos = c.sign * c.env * 0.5 * (1.0 - std::cos(2.0 * wavesynth::kPi * kThudHz * c.t));
                if (c.eNow < 1e-4 && c.t >= 1.0 / kThudHz) { c.env = 0.0; c.eNow = 0.0; }
            }
            inst = std::max(inst, std::max(std::fabs(c.out.pos), std::fabs(c.out.force)));
        }
        m_level = std::max(inst, m_level * std::exp(-dtSec / kLevelSec));
    }

    // The strongest corner of the part (a hit is felt whole, not averaged).
    Out outputFor(Part part) const
    {
        int first, last; partWheels(part, first, last);
        Out o;
        for (int i = first; i <= last; ++i)
        {
            if (std::fabs(m_c[i].out.pos)   > std::fabs(o.pos))   o.pos   = m_c[i].out.pos;
            if (std::fabs(m_c[i].out.force) > std::fabs(o.force)) o.force = m_c[i].out.force;
        }
        return o;
    }
    double   level() const             { return m_level; }
    uint32_t knocks(int wheel) const   { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_c[wheel].count : 0u; }
    double   lastSeverity(int wheel) const { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_c[wheel].lastSev : 0.0; }

    void clear()
    {
        for (C& c : m_c) c = C{};
        m_heaveIn = m_heavePrev = 0.0; m_heaveDriven = m_heaveSeen = false;
        m_level = 0.0;
    }

private:
    struct C
    {
        // inputs
        double velIn = 0.0, velLast = 0.0, travelIn = 0.0, travelLast = 0.0;
        bool   velDriven = false, travelDriven = false, velSeen = false, travelSeen = false, haveV = false;
        double sinceSample = 0.0, vLast = 0.0, dtSample = 1.0 / 60.0;
        // the knock
        double env = 0.0, eNow = 0.0, t = 0.0, phase = 0.0, sign = 1.0, sinceKnock = 1e9, lastSev = 0.0;
        uint32_t count = 0;
        Out    out;
    };

    void start(C& c, double s, double sign, bool force)
    {
        using namespace impact_k;
        if (s <= 0.0) return;
        if (!force && c.sinceKnock < kMinGapSec && s < kHarderBy * c.eNow) return;
        c.env = s; c.t = 0.0; c.phase = 0.0; c.sign = sign; c.sinceKnock = 0.0; c.lastSev = s; ++c.count;
    }

    C m_c[WHEEL_COUNT];
    double m_heaveIn = 0.0, m_heavePrev = 0.0;
    bool   m_heaveDriven = false, m_heaveSeen = false;
    double m_level = 0.0;
};

} // namespace haptics
