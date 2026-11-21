// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// RoadFullLearner - the Road tile's full mm learned for each car (the
// suspension model, with learn full mm).
//
// Each corner's travel as the model plays it (after cut hz and susp cut
// hz, before susp %) goes into a histogram of log-spaced bins from
// kFullLearnLoMm to kFullLearnHiMm, pooled over the four corners while the
// car rolls above kFullLearnMinKmh with the tyre on the ground (RoadModel
// decides that). What this session's driving says is its kFullLearnQ
// quantile / kFullLearnShare, once kFullLearnMinSec of driving is in it.
// Measured on 25 recorded sessions at the GT3 starting point's cuts (cut hz
// 3, susp cut hz 6): the 99th percentile is 2.6 mm for Assetto Corsa's
// Porsche GT3 R, which at this share is the 29 mm that starting point sets
// by hand; across the cars 1.3 to 4.1 mm, and Automobilista 2's cars move
// about half as far as Assetto Corsa's.
//
// The value in use is a weighted mean (of the logs, by seconds) of three
// things: the full mm as set (kFullLearnSetSec, so a new car starts from it
// and leaves it as the driving comes in), what was remembered for this car
// from earlier sessions (seed(), up to kFullLearnSeedMaxSec), and this
// session's driving (the seconds in the histogram). Taken straight from the
// histogram the value moved 2 to 6 times within one recorded drive (4 to 26
// mm in one car): its first minutes are an out lap, and five minutes of
// memory followed whichever part of the track came last. The histogram now
// remembers about kFullLearnMemorySec of driving (older driving fades out
// slowly: another track, a car the sim does not name).
//
// carMm() / carSec() are the car's own evidence without the set value:
// what RoadFullMemory keeps for the car between sessions.
//
// RT-safe: a fixed array, pure arithmetic, no allocation.
// ============================================================

#include "NcxTokens.h"      // WHEEL_COUNT
#include <algorithm>
#include <cmath>

namespace haptics {

namespace road_k {
    constexpr double kMinFullMm  = 0.5;
    constexpr int    kFullLearnBins    = 160;   // 5 % wide
    constexpr double kFullLearnLoMm    = 0.05, kFullLearnHiMm = 128.0;
    constexpr double kFullLearnMinKmh  = 30.0;
    constexpr double kFullLearnQ       = 0.99;
    constexpr double kFullLearnShare   = 0.09;
    constexpr double kFullLearnMinSec  = 20.0;
    constexpr double kFullLearnEverySec = 0.5;
    constexpr double kFullLearnMemorySec = 1800.0;
    constexpr double kFullLearnMaxMm   = 50.0;   // the field's own range
    constexpr double kFullLearnSetSec  = 60.0;   // the full mm as set counts as this much driving
    constexpr double kFullLearnSeedMaxSec = 1800.0;  // what was remembered for a car counts as this much at most
}

struct RoadFullLearner
{
    // One corner's travel as the model plays it (mm), for dtSec of driving.
    void add(double mm, double dtSec)
    {
        using namespace road_k;
        if (!std::isfinite(mm)) return;
        const double a = std::fabs(mm);
        const int b = (a <= kFullLearnLoMm) ? 0
                    : std::min(kFullLearnBins - 1, static_cast<int>(std::log(a / kFullLearnLoMm)
                                                    / std::log(kFullLearnHiMm / kFullLearnLoMm) * kFullLearnBins));
        m_hist[b] += dtSec;
        m_mass += dtSec;
        m_sec  += dtSec;
    }

    // Every cycle, after the corners: the value worked out now and then (it
    // moves slowly), the older driving fading as it goes. setFullMm: the
    // full mm as set.
    void update(double setFullMm)
    {
        using namespace road_k;
        if (setFullMm != m_lastSet) { m_lastSet = setFullMm; m_dirty = true; }   // it is one of the three the value is made of
        if (m_sec < m_nextSec && !m_dirty) return;
        const bool tick = m_sec >= m_nextSec;
        m_dirty = false;
        // This session's driving, once there is enough of it to say anything.
        const bool have = m_sec >= kFullLearnMinSec * WHEEL_COUNT;
        const double wr  = have ? m_mass / WHEEL_COUNT : 0.0;
        const double est = have ? clampMm(quantileMm(kFullLearnQ) / kFullLearnShare) : 0.0;
        const double ws  = m_seedMm > 0.0 ? std::min(kFullLearnSeedMaxSec, m_seedSec) : 0.0;
        const double wc  = ws + wr;
        if (wc <= 0.0) { m_carMm = m_carSec = m_valueMm = 0.0; }
        else
        {
            const double carLog = ((ws > 0.0 ? ws * std::log(m_seedMm) : 0.0) + (wr > 0.0 ? wr * std::log(est) : 0.0)) / wc;
            m_carMm  = clampMm(std::exp(carLog));
            m_carSec = std::min(kFullLearnSeedMaxSec, wc);
            const double set = clampMm(setFullMm);
            m_valueMm = clampMm(std::exp((kFullLearnSetSec * std::log(set) + wc * carLog) / (kFullLearnSetSec + wc)));
        }
        if (tick)
        {
            m_nextSec = m_sec + kFullLearnEverySec * WHEEL_COUNT;
            const double keep = std::exp(-kFullLearnEverySec / kFullLearnMemorySec);
            for (double& b : m_hist) b *= keep;
            m_mass *= keep;
        }
    }

    // A different car, or new cuts (the millimetres are after them): its
    // suspension starts over, and nothing is remembered for it.
    void forget()
    {
        for (double& b : m_hist) b = 0.0;
        m_mass = m_sec = m_nextSec = 0.0;
        m_seedMm = m_seedSec = 0.0;
        m_carMm = m_carSec = m_valueMm = 0.0;
        m_dirty = false;
    }

    // What was remembered for this car: counts beside this session's
    // driving from the next update() on. Leaves the driving so far alone.
    void seed(double mm, double sec)
    {
        if (!std::isfinite(mm) || !std::isfinite(sec) || mm <= 0.0 || sec <= 0.0) return;
        m_seedMm = clampMm(mm); m_seedSec = sec;
        m_dirty = true;
    }

    // Full mm for this car (0 = not yet: nothing remembered and too little
    // driving), and this session's driving in it, seconds.
    double valueMm() const   { return m_valueMm; }
    double drivenSec() const { return m_sec / WHEEL_COUNT; }
    // The car's own evidence (what was remembered and this session's
    // driving, without the full mm as set), and the seconds it stands for.
    double carMm() const  { return m_carMm; }
    double carSec() const { return m_carSec; }
    bool   seeded() const { return m_seedMm > 0.0; }

    // The travel (mm) at quantile q of this session's driving (0 if none).
    double quantileMm(double q) const
    {
        using namespace road_k;
        if (m_mass <= 0.0) return 0.0;
        const double want = std::max(0.0, std::min(1.0, q)) * m_mass;
        const double step = std::log(kFullLearnHiMm / kFullLearnLoMm) / kFullLearnBins;
        double acc = 0.0;
        for (int b = 0; b < kFullLearnBins; ++b)
        {
            if (acc + m_hist[b] >= want && m_hist[b] > 0.0)
            {
                const double f = (want - acc) / m_hist[b];   // within the bin, geometric
                return kFullLearnLoMm * std::exp((b + f) * step);
            }
            acc += m_hist[b];
        }
        return kFullLearnHiMm;
    }

private:
    static double clampMm(double mm) { return std::max(road_k::kMinFullMm, std::min(road_k::kFullLearnMaxMm, mm)); }

    // The travel histogram (seconds per bin, the four corners pooled, older
    // driving fading) and its sum; the driving learned over (corner
    // seconds, never fading: the gate and the clock); when the value is
    // next worked out; what was remembered; the results.
    double m_hist[road_k::kFullLearnBins] = {};
    double m_mass = 0.0, m_sec = 0.0, m_nextSec = 0.0;
    double m_seedMm = 0.0, m_seedSec = 0.0;
    double m_carMm = 0.0, m_carSec = 0.0, m_valueMm = 0.0;
    double m_lastSet = 0.0;
    bool   m_dirty = false;
};

} // namespace haptics
