// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// GripLimitLearner - where a tyre's grip runs out, learned from the
// car's own g, for the slip laws' onset (HapticLaws.h).
//
// A tyre's grip grows with its slip up to a limit and then stops: past
// it the car's g no longer rises however far the tyre slides. So the
// limit shows in what the sim sends: sort the slip into bins by size,
// keep the car's g in each, and the limit is where the g reaches the
// top. On the replays (2026-10-30, 30 cars in 6 titles) each car's limit
// held per axle within about 0.5 deg between the two halves of a session
// (Assetto Corsa, Automobilista 2, Le Mans Ultimate), and it differed up
// to 2.5 x between titles and 7 x between BeamNG.drive cars: no one
// fixed onset fits them all.
//
//   - BINS log-spaced bins of the slip (from x0, each step x the last);
//     each keeps the seconds seen and a running mean of the g there (over
//     its first seconds, then its last kMeanSec), so old driving fades.
//   - Every kEvalSec: the bins with minBinSec or more, smoothed over
//     three; the plateau is the highest (kMinPlateauG at least); the
//     limit is the smallest bin reaching kKneeShare of it (its middle).
//   - The g has to be seen climbing to it: kClimbBins bins below it at
//     kClimbLo to kClimbHi of the plateau (a slip that does not follow the
//     g, corners at one g whatever the angle and straights at none, is a
//     step, and taught a limit at the first cornering bin: a full slide in
//     every corner).
//   - Past its peak, where the g falls away (to kFallShare of the
//     plateau): the far end of the tyre's best (fall; 0 = not seen). A tyre
//     with a long flat top (Le Mans Ultimate's GT3 fronts: 90 % of the top
//     at 1.9 deg, the top at 5.6, falling past 8) has it far past its knee.
//   - The evidence for the limit: the seconds spent at or past it
//     (nearSec); weight() leans the limit in use from a preset to the
//     learned one by it, half way at kEvidenceHalfSec (5 s: Le Mans
//     Ultimate's rears spend about 6 s a minute there; at 30 s they played
//     3 to 5 % of the time on the replays, against 17 to 26 % on the
//     learned limit as it was).
//   - Trusted only once the car has been past it, pastSec in the bins at
//     kPastX x the limit or more (a driver who has not found the limit
//     yet shows no plateau); after that it eases to each new estimate
//     over kEaseSec, so one odd corner cannot move it far.
// Which samples count is the caller's (steady cornering, straight
// braking). A different car starts it over (newCar builds a fresh
// LawsState).
//
// RT-safe: fixed arrays, no allocation, no strings.
// ============================================================

#include <algorithm>
#include <cmath>

struct GripLimitLearner
{
    static constexpr int    BINS         = 36;
    static constexpr double kMeanSec     = 30.0;
    static constexpr double kEvalSec     = 0.5;
    static constexpr double kKneeShare   = 0.9;
    static constexpr double kMinPlateauG = 0.4;
    static constexpr double kPastX       = 1.3;
    static constexpr double kEaseSec     = 20.0;
    static constexpr int    kMinBins     = 4;
    static constexpr double kClimbLo     = 0.3, kClimbHi = 0.85;
    static constexpr int    kClimbBins   = 2;
    static constexpr double kFallShare   = 0.97;
    static constexpr double kEvidenceHalfSec = 5.0;

    explicit GripLimitLearner(double from = 0.25, double ratio = 1.13) : x0(from), step(ratio) {}
    // The bins by what is learned: a slip angle (deg, 0.25 to about 20),
    // a combined slip (1 to about 390), a slip ratio (0.004 to about 0.6).
    static GripLimitLearner angle()    { return GripLimitLearner(0.25, 1.13); }
    static GripLimitLearner combined() { return GripLimitLearner(1.0, 1.18); }
    static GripLimitLearner ratio()    { return GripLimitLearner(0.004, 1.15); }

    double x0, step;                    // bin i spans x0 x step^i .. x0 x step^(i+1)
    double sec[BINS] = {}, g[BINS] = {};
    double limit  = 0.0;                // the slip at the limit; 0 = not learned yet
    double fall   = 0.0;                // past the peak, where the g has fallen away; 0 = not seen
    double nearSec = 0.0;               // seconds of samples at or past the limit (its evidence)
    double fedSec = 0.0;                // seconds of samples in all (for the tile)
    double evalIn = 0.0;

    bool learned() const { return limit > 0.0; }
    // How far to trust the learned limit over a preset: 0 not learned, then
    // half way at kEvidenceHalfSec at or past it, towards 1.
    double weight() const { return learned() ? nearSec / (nearSec + kEvidenceHalfSec) : 0.0; }
    double centre(int i) const { return x0 * std::pow(step, i + 0.5); }

    // One sample: the slip and the car's g (both positive), held dtSec.
    void add(double x, double gAbs, double dtSec)
    {
        if (!(x >= x0) || !(gAbs >= 0.0) || !(dtSec > 0.0)) return;
        const int i = static_cast<int>(std::floor(std::log(x / x0) / std::log(step)));
        if (i < 0 || i >= BINS) return;
        sec[i] += dtSec;
        const double w = std::max(dtSec / sec[i], dtSec / kMeanSec);   // (sec has this sample: the first sets the mean)
        g[i] += std::min(1.0, w) * (gAbs - g[i]);
        fedSec += dtSec;
    }

    // Every cycle; looks again every kEvalSec.
    void tick(double dtSec, double minBinSec, double pastSec)
    {
        if ((evalIn += dtSec) < kEvalSec) return;
        evalIn = 0.0;
        int idx[BINS]; int n = 0;
        for (int i = 0; i < BINS; ++i) if (sec[i] >= minBinSec) idx[n++] = i;
        if (n < kMinBins) return;
        double sm[BINS]; double plateau = 0.0;
        for (int k = 0; k < n; ++k)
        {
            const int a = std::max(0, k - 1), b = std::min(n - 1, k + 1);
            double s = 0.0;
            for (int j = a; j <= b; ++j) s += g[idx[j]];
            sm[k] = s / (b - a + 1);
            plateau = std::max(plateau, sm[k]);
        }
        if (plateau < kMinPlateauG) return;
        int knee = -1;
        for (int k = 0; k < n && knee < 0; ++k) if (sm[k] >= kKneeShare * plateau) knee = k;
        if (knee < 0) return;
        int climb = 0;
        for (int k = 0; k < knee; ++k)
            if (g[idx[k]] >= kClimbLo * plateau && g[idx[k]] <= kClimbHi * plateau) ++climb;
        if (climb < kClimbBins) return;
        const double at = centre(idx[knee]);
        double past = 0.0;
        for (int i = 0; i < BINS; ++i) if (centre(i) >= kPastX * at) past += sec[i];
        if (past < pastSec) return;
        const double ease = std::min(1.0, kEvalSec / kEaseSec);
        limit = learned() ? limit + ease * (at - limit) : at;
        double atLimit = 0.0;
        for (int i = 0; i < BINS; ++i) if (centre(i) >= limit) atLimit += sec[i];
        nearSec = atLimit;
        int peak = 0;
        for (int k = 1; k < n; ++k) if (sm[k] > sm[peak]) peak = k;
        for (int k = peak + 1; k < n; ++k)
            if (sm[k] <= kFallShare * plateau)
            {
                const double f = centre(idx[k]);
                fall = (fall > 0.0) ? fall + ease * (f - fall) : f;
                break;
            }
    }
};
