// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// GearRatioLearner - self-learning per-car gear ratios from the
// NULLCATX channel stream, for the ratio-aware device effects
// (revmatch let-in first; anything else that needs "what rpm would
// gear G be at right now" reuses it).
//
// The wire carries no car identity, so identity IS the ratio set:
//   - While driving in a forward gear with the clutch up and enough
//     speed, rpm/speedKmh is that gear's ratio (rpm per km/h). Each
//     gear's estimate seeds on first sight and then EWMA-tracks
//     agreeing samples; a gear is CONFIDENT after enough of them.
//   - Sustained disagreement (car change, new setup) relearns that
//     gear from scratch and drops every adopted value - stale ratios
//     from the previous car must never linger on gears not yet driven.
//   - The cache: once two session gears are confident, they are
//     matched against remembered cars (all comparable gears within
//     tolerance); a match ADOPTS the remembered ratios for gears not
//     yet driven this session. Adopted values still track live samples
//     and are never written back to the cache (only session-observed
//     gears persist - a wrong adoption cannot propagate).
//
// RT-safe: fixed state, no allocation after setCache(), no strings.
// Persistence lives in CarCache (off the RT thread); the learner only
// holds the in-memory table.
// ============================================================

#include "DeviceStateLayer.h"   // NcxValues, GearRatios, MAX_GEARS
#include <cmath>
#include <cstdint>

static constexpr int MAX_CACHED_CARS = 64;

// One remembered car: the ratios it was observed with.
struct CachedCar
{
    double r[MAX_GEARS]   = {};
    bool   has[MAX_GEARS] = {};
};

class GearRatioLearner
{
public:
    // Learning thresholds. Ratios are rpm per km/h (typical spread ~20
    // in top gear to ~120 in first), so relative tolerances are safe.
    static constexpr double SPEED_MIN_KMH   = 15.0;  // below this rpm/speed is noise
    static constexpr double RPM_MIN         = 500.0;
    static constexpr double CLUTCH_MAX_PCT  = 50.0;  // clutch mostly up = driving
    static constexpr double EWMA_ALPHA      = 0.05;
    static constexpr double OUTLIER_PCT     = 0.10;  // sample vs estimate disagreement
    static constexpr int    OUTLIER_RELEARN = 300;   // consecutive outliers -> new car
    static constexpr int    ADOPTED_DROP    = 25;    // adopted value: dies much faster
    static constexpr int    CONFIDENT_N     = 400;   // agreeing samples -> confident
    static constexpr int    COUNT_CAP       = 10000;
    static constexpr double MATCH_TOL       = 0.06;  // cache-match tolerance
    static constexpr int    ADOPT_MIN_GEARS = 2;     // session gears needed to adopt

    void setCache(const CachedCar* cars, int n)
    {
        m_cacheN = 0;
        for (int i = 0; i < n && i < MAX_CACHED_CARS; ++i) m_cache[m_cacheN++] = cars[i];
    }

    void step(const NcxValues& v)
    {
        if (!v.fresh) return;
        if (!v.have[NcxValues::Rpm] || !v.have[NcxValues::SpeedKmh] ||
            !v.have[NcxValues::Gear]) return;
        const int g = (int)std::lround(v.val[NcxValues::Gear]);
        if (g < 1 || g >= MAX_GEARS) return;             // neutral/reverse: no ratio
        const double sp  = v.val[NcxValues::SpeedKmh];
        const double rpm = v.val[NcxValues::Rpm];
        if (sp < SPEED_MIN_KMH || rpm < RPM_MIN) return;
        if (v.have[NcxValues::ClutchPct] &&
            v.val[NcxValues::ClutchPct] > CLUTCH_MAX_PCT) return;   // slipping: not a ratio

        const double r = rpm / sp;
        Slot& s = m_g[g];
        if (s.count == 0 && !s.adopted)
        {
            s.ratio = r; s.count = 1; s.outliers = 0;
        }
        else if (std::fabs(r - s.ratio) <= OUTLIER_PCT * s.ratio)
        {
            // Agreeing sample: track it. An adopted value being confirmed
            // by live driving keeps its adopted (usable) status while the
            // live count builds toward session confidence.
            s.ratio += EWMA_ALPHA * (r - s.ratio);
            if (s.count < COUNT_CAP) ++s.count;
            s.outliers = 0;
            if (s.count == CONFIDENT_N)
            {
                m_dirty = true;
                tryAdopt();
            }
        }
        else
        {
            // Disagreement. An ADOPTED value is only a hypothesis: a short
            // burst of disagreement kills it (that gear alone - the match
            // may still be right for the others). A session-LEARNED value
            // takes sustained disagreement (a different car), and that
            // also drops every adopted value - they belonged to the old
            // identity.
            const int limit = s.adopted ? ADOPTED_DROP : OUTLIER_RELEARN;
            if (++s.outliers >= limit)
            {
                const bool wasAdopted = s.adopted;
                s = Slot{}; s.ratio = r; s.count = 1;
                if (!wasAdopted)
                {
                    for (int i = 1; i < MAX_GEARS; ++i)
                        if (m_g[i].adopted) m_g[i] = Slot{};
                    m_adoptedOnce = false;
                }
            }
        }

        publish();
    }

    const GearRatios& ratios() const { return m_view; }

    // Persistence interface (read off-RT via the published status copy).
    double   gearRatio(int g)        const { return (g > 0 && g < MAX_GEARS) ? m_g[g].ratio : 0.0; }
    uint16_t gearCount(int g)        const { return (g > 0 && g < MAX_GEARS) ? m_g[g].count : 0; }
    // Session-confident = enough LIVE agreeing samples, however the slot
    // was seeded. Only these persist to the cache, so a wrong adoption can
    // never write itself back.
    bool     gearSessionConfident(int g) const
    { return g > 0 && g < MAX_GEARS && m_g[g].count >= CONFIDENT_N; }
    bool     dirty() const { return m_dirty; }

private:
    struct Slot
    {
        double   ratio    = 0.0;
        uint16_t count    = 0;
        uint16_t outliers = 0;
        bool     adopted  = false;
    };

    void publish()
    {
        for (int g = 1; g < MAX_GEARS; ++g)
        {
            m_view.r[g]     = m_g[g].ratio;
            m_view.known[g] = m_g[g].adopted || m_g[g].count >= CONFIDENT_N;
        }
    }

    void tryAdopt()
    {
        if (m_adoptedOnce) return;
        int confident = 0;
        for (int g = 1; g < MAX_GEARS; ++g)
            if (gearSessionConfident(g)) ++confident;
        if (confident < ADOPT_MIN_GEARS) return;

        for (int c = 0; c < m_cacheN; ++c)
        {
            const CachedCar& car = m_cache[c];
            bool ok = true; int shared = 0;
            for (int g = 1; g < MAX_GEARS && ok; ++g)
            {
                if (!gearSessionConfident(g) || !car.has[g]) continue;
                ++shared;
                if (std::fabs(m_g[g].ratio - car.r[g]) > MATCH_TOL * car.r[g]) ok = false;
            }
            if (!ok || shared < ADOPT_MIN_GEARS) continue;

            for (int g = 1; g < MAX_GEARS; ++g)
                if (car.has[g] && m_g[g].count < CONFIDENT_N && !m_g[g].adopted)
                { m_g[g].ratio = car.r[g]; m_g[g].adopted = true; m_g[g].outliers = 0; }
            m_adoptedOnce = true;
            return;
        }
    }

    Slot       m_g[MAX_GEARS];
    GearRatios m_view;
    CachedCar  m_cache[MAX_CACHED_CARS];
    int        m_cacheN      = 0;
    bool       m_adoptedOnce = false;
    bool       m_dirty       = false;
};
