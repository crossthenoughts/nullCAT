// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// GearRatioLearner - self-learning per-car gear ratios from the
// NULLCATX channel stream, for the ratio-aware device effects
// (revmatch let-in first) and the Driveline tile's clutch judder
// ("what rpm would gear G be at right now").
//
// The wire carries no car identity, so identity IS the ratio set:
//   - While driving steadily in a forward gear (the gear held a moment,
//     clutch up, off the brake, rpm/speed not running away: no shift, no
//     launch, no wheelspin), rpm/speedKmh is that gear's ratio (rpm per
//     km/h). Each gear's estimate seeds on first sight and then tracks
//     agreeing samples; a gear is CONFIDENT after enough seconds of them.
//   - Sustained disagreement (car change, new setup) relearns that
//     gear from scratch and drops every adopted value - stale ratios
//     from the previous car must never linger on gears not yet driven.
//   - A higher gear has a lower ratio: gears that contradict each other
//     are not used (or persisted) until driving sorts out which is wrong.
//   - The cache: once two session gears are confident, they are
//     matched against remembered cars (all comparable gears within
//     tolerance); a match ADOPTS the remembered ratios for gears not
//     yet driven this session. Adopted values still track live samples
//     and are never written back to the cache (only session-observed
//     gears persist - a wrong adoption cannot propagate).
// Every threshold is in seconds of qualifying samples, so the control
// rate does not matter: counted in cycles, a gear was confident after
// 0.2 s at 2 kHz and a launch's wheelspin relearned it (2nd gear learned
// at half its ratio, then remembered in the cache).
//
// RT-safe: fixed state, no allocation after setCache(), no strings.
// Persistence lives in CarCache (off the RT thread); the learner only
// holds the in-memory table.
// ============================================================

#include "DeviceStateLayer.h"   // NcxValues, GearRatios, MAX_GEARS
#include <algorithm>
#include <cstring>
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
    static constexpr double SPEED_MIN_KMH    = 15.0;  // below this rpm/speed is noise
    static constexpr double RPM_MIN          = 500.0;
    static constexpr double CLUTCH_MAX_PCT   = 20.0;  // clutch up = driving through it
    static constexpr double BRAKE_MAX_PCT    = 5.0;   // braking: a locking wheel, a downshift's revs
    static constexpr double HOLD_SEC         = 0.5;   // the gear held this long with every sample fit (no shift, no junk frame, no clutch/brake/crawl)
    static constexpr double STEADY_TAU_SEC   = 0.25;  // rpm/speed's own smoothing...
    static constexpr double STEADY_PCT       = 0.02;  // ...a sample this far off it is no ratio (a launch, wheelspin)
    static constexpr double TRACK_TAU_SEC    = 1.0;   // an agreeing estimate's time constant
    static constexpr double OUTLIER_PCT      = 0.10;  // sample vs estimate disagreement
    static constexpr double RELEARN_SEC      = 3.0;   // sustained disagreement -> new car
    static constexpr double ADOPTED_DROP_SEC = 0.3;   // an adopted value dies much faster
    static constexpr double CONFIDENT_SEC    = 2.0;   // agreeing seconds -> confident
    static constexpr double AGREE_CAP_SEC    = 600.0;
    static constexpr double MATCH_TOL        = 0.06;  // cache-match tolerance
    static constexpr int    ADOPT_MIN_GEARS  = 2;     // session gears needed to adopt

    // Ratios that fall with every gear (gaps allowed): a set where a
    // higher gear's ratio is not below a lower one's has a wrong value.
    static bool inOrder(const double r[MAX_GEARS], const bool has[MAX_GEARS])
    {
        double last = 0.0;
        for (int g = 1; g < MAX_GEARS; ++g)
        {
            if (!has[g]) continue;
            if (last > 0.0 && r[g] >= last) return false;
            last = r[g];
        }
        return true;
    }

    void setCache(const CachedCar* cars, int n)
    {
        m_cacheN = 0;
        for (int i = 0; i < n && i < MAX_CACHED_CARS; ++i) m_cache[m_cacheN++] = cars[i];
    }

    void step(const NcxValues& v, double dtSec)
    {
        if (!v.fresh || dtSec <= 0.0) { m_steadyOk = false; m_heldSec = 0.0; return; }
        // The sim naming a different car: this car's ratios are finished
        // with (stashed for the cache) and the slots start over. Without
        // that, every gear the new car had not yet driven kept the old
        // car's ratio as known, and revmatch and the clutch judder read it.
        if (v.car[0] != '\0')
        {
            if (m_carSeen && std::strncmp(v.car, m_car, NCY_STR_LEN) != 0) finishCar();
            std::memcpy(m_car, v.car, NCY_STR_LEN);
            m_car[NCY_STR_LEN - 1] = '\0';
            m_carSeen = true;
        }
        if (!v.have[NcxValues::Rpm] || !v.have[NcxValues::SpeedKmh] ||
            !v.have[NcxValues::Gear]) return;
        const double gRaw = v.val[NcxValues::Gear];
        if (!std::isfinite(gRaw) || gRaw < -100.0 || gRaw > 100.0) return;   // junk: the laws drop it, this path sees the raw frame
        const int g = (int)std::lround(gRaw);
        // The gear held: any change (a shift, a one-frame junk gear) starts
        // the wait again.
        if (g != m_lastGear) { m_lastGear = g; m_heldSec = 0.0; m_steadyOk = false; }
        else m_heldSec += dtSec;
        if (g < 1 || g >= MAX_GEARS) return;             // neutral/reverse: no ratio
        const double sp  = v.val[NcxValues::SpeedKmh];
        const double rpm = v.val[NcxValues::Rpm];
        // Any sample that cannot be a ratio starts the wait again too: the
        // first samples after one would otherwise pass as steady before the
        // smoothing had anything to compare with (a launch in a held 1st,
        // the wheels spinning past 15 km/h, seeded 1st at four times its
        // ratio; launch after launch made it confident).
        const auto reject = [&]() { m_steadyOk = false; m_heldSec = 0.0; };
        if (sp < SPEED_MIN_KMH || rpm < RPM_MIN) { reject(); return; }
        if (v.have[NcxValues::ClutchPct] &&
            v.val[NcxValues::ClutchPct] > CLUTCH_MAX_PCT) { reject(); return; }   // slipping: not a ratio
        if (v.have[NcxValues::BrakePct] &&
            v.val[NcxValues::BrakePct] > BRAKE_MAX_PCT) { reject(); return; }

        const double r = rpm / sp;
        // Steady: rpm/speed near its own recent value (a launch or a wheel
        // spinning up runs away from it).
        if (!m_steadyOk) { m_steady = r; m_steadyOk = true; }
        else m_steady += std::min(1.0, dtSec / STEADY_TAU_SEC) * (r - m_steady);
        if (m_heldSec < HOLD_SEC || std::fabs(r - m_steady) > STEADY_PCT * m_steady) return;

        Slot& s = m_g[g];
        if (!s.seeded && !s.adopted)
        {
            s.ratio = r; s.seeded = true; s.agreeSec = 0.0; s.outlierSec = 0.0;
        }
        else if (std::fabs(r - s.ratio) <= OUTLIER_PCT * s.ratio)
        {
            // Agreeing sample: track it. An adopted value being confirmed
            // by live driving keeps its adopted (usable) status while the
            // live time builds toward session confidence.
            s.ratio += std::min(1.0, dtSec / TRACK_TAU_SEC) * (r - s.ratio);
            const bool was = s.agreeSec >= CONFIDENT_SEC;
            s.agreeSec = std::min(AGREE_CAP_SEC, s.agreeSec + dtSec);
            s.outlierSec = 0.0;
            if (!was && s.agreeSec >= CONFIDENT_SEC)
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
            s.outlierSec += dtSec;
            if (s.outlierSec >= (s.adopted ? ADOPTED_DROP_SEC : RELEARN_SEC))
            {
                const bool wasAdopted = s.adopted;
                s = Slot{}; s.ratio = r; s.seeded = true;
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
    double   gearAgreeSec(int g)     const { return (g > 0 && g < MAX_GEARS) ? m_g[g].agreeSec : 0.0; }
    // Session-confident = enough LIVE agreeing seconds, however the slot
    // was seeded, and no other confident gear contradicting it. Only these
    // persist to the cache, so a wrong adoption can never write itself
    // back.
    bool     gearSessionConfident(int g) const
    { return g > 0 && g < MAX_GEARS && m_g[g].agreeSec >= CONFIDENT_SEC && consistent(g, false); }
    bool     dirty() const { return m_dirty; }
    // The last car finished with (the sim named another): its session-
    // confident ratios, whether they were new, and a count that steps once
    // per finished car so a reader merges each set into the cache once.
    double   prevRatio(int g)     const { return (g > 0 && g < MAX_GEARS) ? m_prev.r[g] : 0.0; }
    bool     prevConfident(int g) const { return g > 0 && g < MAX_GEARS && m_prev.has[g]; }
    bool     prevDirty()          const { return m_prevDirty; }
    uint32_t prevSeq()            const { return m_prevSeq; }

private:
    // A different car: stash what this one taught (for the cache), then
    // start the slots over. The adoption from the cache runs again for the
    // new car once it has two confident gears of its own.
    void finishCar()
    {
        CachedCar done{};
        bool any = false;
        for (int g = 1; g < MAX_GEARS; ++g)
        {
            done.has[g] = gearSessionConfident(g);
            done.r[g]   = done.has[g] ? m_g[g].ratio : 0.0;
            any = any || done.has[g];
        }
        if (any) { m_prev = done; m_prevDirty = m_dirty; ++m_prevSeq; }
        for (Slot& s : m_g) s = Slot{};
        m_adoptedOnce = false; m_dirty = false;
        m_lastGear = -99; m_heldSec = 0.0; m_steadyOk = false;
        publish();
    }
    struct Slot
    {
        double ratio      = 0.0;
        double agreeSec   = 0.0;
        double outlierSec = 0.0;
        bool   seeded     = false;
        bool   adopted    = false;
    };

    bool liveConfident(int g) const { return m_g[g].agreeSec >= CONFIDENT_SEC; }
    // Gear g against every other gear that counts (live-confident, and with
    // adopted ones too when they count for use): lower gears above it,
    // higher ones below.
    bool consistent(int g, bool withAdopted) const
    {
        for (int h = 1; h < MAX_GEARS; ++h)
        {
            if (h == g || !(liveConfident(h) || (withAdopted && m_g[h].adopted))) continue;
            if (h < g && !(m_g[h].ratio > m_g[g].ratio)) return false;
            if (h > g && !(m_g[h].ratio < m_g[g].ratio)) return false;
        }
        return true;
    }

    void publish()
    {
        for (int g = 1; g < MAX_GEARS; ++g)
        {
            m_view.r[g]     = m_g[g].ratio;
            m_view.known[g] = (m_g[g].adopted || liveConfident(g)) && consistent(g, true);
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
            if (!inOrder(car.r, car.has)) continue;   // a remembered set with a wrong value
            bool ok = true; int shared = 0;
            for (int g = 1; g < MAX_GEARS && ok; ++g)
            {
                if (!gearSessionConfident(g) || !car.has[g]) continue;
                ++shared;
                if (std::fabs(m_g[g].ratio - car.r[g]) > MATCH_TOL * car.r[g]) ok = false;
            }
            if (!ok || shared < ADOPT_MIN_GEARS) continue;

            for (int g = 1; g < MAX_GEARS; ++g)
                if (car.has[g] && !liveConfident(g) && !m_g[g].adopted)
                { m_g[g].ratio = car.r[g]; m_g[g].adopted = true; m_g[g].seeded = true; m_g[g].outlierSec = 0.0; }
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
    int        m_lastGear    = -99;
    double     m_heldSec     = 0.0;
    double     m_steady      = 0.0;
    bool       m_steadyOk    = false;
    char       m_car[NCY_STR_LEN] = {};
    bool       m_carSeen     = false;
    CachedCar  m_prev{};
    bool       m_prevDirty   = false;
    uint32_t   m_prevSeq     = 0;
};
