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
//            heavy compression) from heave g, full at 3 x that, or in its
//            acceleration along and across the car together (a wall, a
//            car's contact) from hit g, full at 3 x that, on every corner
//            (the stronger of the two). A jump is the change from the
//            sample before; on a feed faster than about 120 Hz (learned
//            from its samples), the change since the value current 12 ms
//            before, so it knocks as a 60 Hz feed does.
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
    constexpr double kHeaveFullX  = 3.0;     // the body: full at this x heave g...
    constexpr double kHitFullX    = 3.0;     // ...and at this x hit g
    constexpr double kJumpWindowSec = 0.012; // the body's jump on a fast feed: the change since the value current this long ago
    constexpr double kFastFeedSec = 0.008;   // ...a feed whose samples come closer than this (learned); slower, the sample before
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
    // The body's acceleration along and across the car, m/s^2 (0 for one
    // the sim does not send), and which of them it sent (1 along, 2
    // across): a different set from the last starts the jumps afresh (a
    // value missing for a frame is no jump to 0).
    void driveHorizontal(double surgeMs2, double swayMs2, int present = 3)
    {
        if (!std::isfinite(surgeMs2) || !std::isfinite(swayMs2)) return;
        m_surgeIn = surgeMs2; m_swayIn = swayMs2; m_horizDriven = true; m_horizSet = present;
    }
    // Per cycle, BEFORE step(): the car is standing still. The inputs are
    // tracked as ever (so the first move after has a true last sample to
    // jump from) but nothing knocks: a car dropped onto the ground at a
    // reset or a spawn knocked once at a standstill.
    void still() { m_still = true; }
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
        const bool   still = m_still;
        m_still = false;
        m_now += dtSec;
        // The body: a jump in vertical acceleration, or in the horizontal
        // one (along and across together), knocks every corner; the
        // stronger of the two plays, the heave's way or up for a hit.
        double bodyS = 0.0, bodySign = 1.0, ja = 0.0, jb = 0.0;
        if (!m_heaveDriven) m_heave.clear();
        else if (m_heave.sample(m_now, m_heaveIn, 0.0, ja, jb) && q.heaveG > 0.0)
        {
            const double s = (std::fabs(ja) / kG - q.heaveG) / ((kHeaveFullX - 1.0) * q.heaveG);
            if (s > bodyS) { bodyS = s; bodySign = (ja > 0.0) ? 1.0 : -1.0; }
        }
        if (!m_horizDriven || m_horizSet != m_horizSetLast) m_horiz.clear();
        m_horizSetLast = m_horizDriven ? m_horizSet : 0;
        if (m_horizDriven && m_horiz.sample(m_now, m_surgeIn, m_swayIn, ja, jb) && q.hitG > 0.0)
        {
            const double s = (std::hypot(ja, jb) / kG - q.hitG) / ((kHitFullX - 1.0) * q.hitG);
            if (s > bodyS) { bodyS = s; bodySign = 1.0; }
        }
        m_bodySince += dtSec;
        if (bodyS > 0.0 && !still)
        {
            const double s = std::min(1.0, bodyS);
            // The same edge again on a faster feed's next sample (within the
            // window of the body knock it started): that knock grows to it,
            // one knock, not one restarted on every sample of the edge.
            if (m_bodySince < kJumpWindowSec)
            {
                for (C& c : m_c)
                    if (c.body && c.env > 0.0 && s > c.env) { c.env = s; c.lastSev = s; }
            }
            else
            {
                bool started = false;
                for (C& c : m_c)
                    if (start(c, s, bodySign, false)) { c.body = true; started = true; }
                if (started) m_bodySince = 0.0;
            }
        }
        m_heaveDriven = m_horizDriven = false;

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
                    if (s > 0.0 && !still) start(c, std::min(1.0, s), moved >= 0.0 ? 1.0 : -1.0, false);
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
    double   ringLevel(int wheel) const    { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_c[wheel].eNow : 0.0; }   // the knock still ringing

    void clear()
    {
        for (C& c : m_c) c = C{};
        m_heaveIn = m_surgeIn = m_swayIn = 0.0; m_heaveDriven = m_horizDriven = false;
        m_heave = Track{}; m_horiz = Track{}; m_now = 0.0; m_bodySince = 1e9; m_horizSet = m_horizSetLast = 0;
        m_still = false;
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
        bool   body = false;   // the knock ringing is the body's (a faster feed's next sample may grow it)
        uint32_t count = 0;
        Out    out;
    };

    // True when the knock starts (or, started this same cycle, grows to it).
    bool start(C& c, double s, double sign, bool force)
    {
        using namespace impact_k;
        if (s <= 0.0) return false;
        // A knock started this same cycle (the body's, then the corner's):
        // one knock, the stronger of the two.
        const bool again = (c.env > 0.0 && c.t == 0.0);
        if (!force)
        {
            if (again) { if (s <= c.env) return false; }
            else if (c.sinceKnock < kMinGapSec && s < kHarderBy * c.eNow) return false;
        }
        c.env = s; c.t = 0.0; c.phase = 0.0; c.sign = sign; c.sinceKnock = 0.0; c.lastSev = s;
        c.body = false;   // the body's loop marks its own
        if (!again) ++c.count;
        return true;
    }

    // The body's samples (one or two values): a new sample is told by its
    // value changing. Its jump is the change from the sample before; on a
    // fast feed (samples learned to come closer than kFastFeedSec), the
    // largest change from the samples since the one that was current
    // kJumpWindowSec ago, that one included: the change over the span a
    // 60 Hz sample covers, not its smaller steps. A 60 Hz feed whose
    // samples happen to arrive close together is still sample to sample.
    struct Track
    {
        static constexpr int kN = 16;
        double t[kN] = {}, a[kN] = {}, b[kN] = {};
        int n = 0, head = 0;
        double interval = 1.0 / 60.0;   // learned time between samples (kept through a gap)
        void clear() { n = 0; head = 0; }
        // True for a new sample after the first, with the jump in (ja, jb).
        bool sample(double now, double va, double vb, double& ja, double& jb)
        {
            if (n > 0 && va == a[head] && vb == b[head]) return false;
            if (n > 0)
            {
                const double gap = now - t[head];
                if (gap > 0.0 && gap < impact_k::kMaxSampleSec) interval += impact_k::kSampleLearn * (gap - interval);
            }
            const double window = (interval < impact_k::kFastFeedSec) ? impact_k::kJumpWindowSec : 0.0;
            ja = jb = 0.0;
            double best = -1.0;
            for (int k = 0; k < n; ++k)
            {
                const int i = (head - k + kN) % kN;
                const double da = va - a[i], db = vb - b[i], d = da * da + db * db;
                if (d > best) { best = d; ja = da; jb = db; }
                if (now - t[i] >= window) break;   // current a window ago (or the sample before): the last that counts
            }
            const bool jumped = (n > 0);
            head = (head + 1) % kN; t[head] = now; a[head] = va; b[head] = vb;
            n = std::min(kN, n + 1);
            return jumped;
        }
    };

    C m_c[WHEEL_COUNT];
    double m_heaveIn = 0.0, m_surgeIn = 0.0, m_swayIn = 0.0, m_now = 0.0;
    double m_bodySince = 1e9;   // since the last body knock started
    int    m_horizSet = 0, m_horizSetLast = 0;   // which horizontal values were sent (1 along, 2 across)
    bool   m_heaveDriven = false, m_horizDriven = false;
    Track  m_heave, m_horiz;
    bool   m_still = false;   // this cycle: standing still (no knocks)
    double m_level = 0.0;
};

} // namespace haptics
