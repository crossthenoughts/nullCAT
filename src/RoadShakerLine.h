// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// RoadShakerLine - what the Road tile sends to its shaker routes, each
// part by itself, for the tile's "to shakers" line:
//
//   bumps    the model's output, or the tone that stands in for it when
//            the model has nothing to run on (the roadNoise fallback, a
//            Test), x shaker bumps %;
//   grain    the surface grain's tone, x shaker grain %;
//   texture  the road's fine texture (RoadTexture.h), at texture x.
//
// Each reads the largest route's share of a shaker's full scale, held and
// falling over kHoldSec (a 25 to 90 Hz signal read as it is would cross
// zero a hundred times a second). The Layer notes what it sends as it
// sends it and rolls the cycle over at the start of the next; the texture
// also gives its own level (before the tile's amp and the route's gain),
// held the same way, for the tile's level.
//
// RT-safe: a few doubles.
// ============================================================

#include <algorithm>
#include <cmath>

namespace haptics {

struct RoadShakerLine
{
    enum Part { Bumps = 0, Grain = 1, Texture = 2, PART_COUNT = 3 };
    static constexpr double kHoldSec = 0.5;

    // A shaker route's share of a part, from its percent setting.
    static double share(double pct)
    {
        return std::isfinite(pct) ? std::max(0.0, std::min(100.0, pct)) / 100.0 : 1.0;
    }

    // One sample as a shaker got it (-1..1 nominal, route gain included).
    void note(Part part, double sample)   { m_now[part] = std::max(m_now[part], std::fabs(sample)); }
    // The texture's own sample on a route that carries it.
    void noteTexture(double sample)       { m_texNow = std::max(m_texNow, std::fabs(sample)); }

    // The cycle just noted becomes what is held; a muted tile sent nothing.
    void roll(double dtSec, bool muted)
    {
        const double fall = std::exp(-dtSec / kHoldSec);
        for (int k = 0; k < PART_COUNT; ++k)
        {
            m_held[k] = std::max(m_held[k] * fall, muted ? 0.0 : m_now[k]);
            m_now[k] = 0.0;
        }
        m_texHeld = std::max(m_texHeld * fall, m_texNow);
        m_texNow = 0.0;
    }

    double sent(int part) const    { return (part >= 0 && part < PART_COUNT) ? m_held[part] : 0.0; }
    double textureLevel() const    { return m_texHeld; }
    void   clear()                 { *this = RoadShakerLine{}; }

private:
    double m_now[PART_COUNT] = {}, m_held[PART_COUNT] = {};
    double m_texNow = 0.0, m_texHeld = 0.0;
};

} // namespace haptics
