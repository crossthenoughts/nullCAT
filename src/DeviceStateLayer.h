// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// DeviceStateLayer - the L2 state layer for device axes: turns bound
// NULLCATX channel values into per-cycle DeviceStateMods for the force
// model. The MODEL is character (config); THIS is situation (wire).
//
// Structure, per the layer contract in DeviceForceModel.h:
//   - The wire carries dumb numbers; NcxMap resolves them into semantic
//     tokens using the rig's ncxBindings ONCE at configure time (no
//     string work on the RT path).
//   - step() is const and stateless: same inputs, same mods, trivially
//     testable, nothing to reset at engage.
//   - FAIL-SAFE FIRST: a stale channel stream (ncxFresh false), an
//     unbound token, or an effect left at its inert default all yield
//     inert mods - the device falls back to its plain configured feel,
//     never to a stuck effect.
//
// v1 effects (thresholds in DeviceParams):
//   - Clutch blocking: clutch not pressed while the lever is being moved
//     out of its detent -> the field stiffens (forceScale 1 + blockGain).
//   - Gear grind: the same blocked push adds the grind texture.
// Ratio-aware effects (revmatch let-in, pop-out) need the per-car
// rpm-per-speed cache and land on top of this without reshaping it.
// ============================================================

#include "Config.h"           // DeviceParams, NcxBinding
#include "DeviceForceModel.h" // DeviceStateMods
#include "TelemetryInput.h"   // TelemetryData (ncx channels)
#include <cmath>
#include <string>
#include <vector>

// Semantic channel values after binding resolution.
struct NcxValues
{
    enum Token { Rpm, SpeedKmh, Gear, ClutchPct, ThrottlePct, TokenCount };
    bool   fresh = false;             // channel stream alive (<500 ms)
    bool   have[TokenCount] = {};     // token bound AND present in the packet
    double val[TokenCount]  = {};
};

// Gear ratios (rpm per km/h), index 1..8; produced by GearRatioLearner,
// consumed here for the revmatch let-in. known = usable now (learned this
// session or adopted from the car cache).
static constexpr int MAX_GEARS = 9;   // index 1..8 used; 0 unused
struct GearRatios
{
    double r[MAX_GEARS]     = {};
    bool   known[MAX_GEARS] = {};
};

inline int ncxTokenIndex(const std::string& t)
{
    if (t == "rpm")         return NcxValues::Rpm;
    if (t == "speedKmh")    return NcxValues::SpeedKmh;
    if (t == "gear")        return NcxValues::Gear;
    if (t == "clutchPct")   return NcxValues::ClutchPct;
    if (t == "throttlePct") return NcxValues::ThrottlePct;
    return -1;
}

// Pre-resolved binding table: strings die at configure time.
class NcxMap
{
public:
    void configure(const std::vector<NcxBinding>& bindings)
    {
        m_n = 0;
        for (const NcxBinding& b : bindings)
        {
            const int tok = ncxTokenIndex(b.token);
            if (tok < 0 || b.slot < 0 || b.slot >= MAX_NCX_CHANNELS) continue;
            if (m_n >= NcxValues::TokenCount) break;
            m_e[m_n++] = { tok, b.slot, b.scale, b.offset };
        }
    }

    NcxValues extract(const TelemetryData& td) const
    {
        NcxValues v;
        v.fresh = td.ncxFresh;
        for (int i = 0; i < m_n; ++i)
        {
            if (m_e[i].slot >= td.numNcx) continue;
            v.have[m_e[i].token] = true;
            v.val[m_e[i].token]  = td.ncx[m_e[i].slot] * m_e[i].scale + m_e[i].offset;
        }
        return v;
    }

    int boundCount() const { return m_n; }

private:
    struct Entry { int token; int slot; double scale; double offset; };
    Entry m_e[NcxValues::TokenCount] = {};
    int   m_n = 0;
};

class DeviceStateLayer
{
public:
    void configure(const DeviceParams& p) { m_p = p; }

    DeviceStateMods step(double posRev, const NcxValues& v,
                         const GearRatios* ratios = nullptr) const
    {
        DeviceStateMods m;                 // inert defaults
        if (!v.fresh) return m;            // stream dead -> plain feel

        // ---- Clutch blocking + gear grind ----
        // clutchPct convention: 0 = pedal up (clutch driving), 100 =
        // floored (disengaged). Blocking needs the clutch DRIVING while
        // the lever is pushed out of its detent past blockStartRev.
        if (m_p.clutchBitePct > 0.0 && v.have[NcxValues::ClutchPct] &&
            !m_p.detents.empty())
        {
            const bool clutchDriving = v.val[NcxValues::ClutchPct] < m_p.clutchBitePct;
            double best = m_p.detents[0];
            for (double d : m_p.detents)
                if (std::fabs(posRev - d) < std::fabs(posRev - best)) best = d;
            const double rel = std::fabs(posRev - best);
            if (clutchDriving && rel > m_p.blockStartRev &&
                !revmatched(v, ratios))
            {
                if (m_p.blockGain > 0.0)
                    m.forceScale = 1.0 + m_p.blockGain;
                if (m_p.grindAmpPct > 0.0)
                {
                    m.textureAmpPct = m_p.grindAmpPct;
                    m.textureFreqHz = m_p.grindFreqHz;
                }
            }
        }
        return m;
    }

private:
    // Revmatch let-in: a clutchless shift goes in when the engine speed
    // already agrees with where a NEIGHBOUR gear would put it at the
    // current road speed (a 1D lever moves one gear at a time, so the
    // destination is current gear +/- 1; matching either lets it in).
    // Needs learned ratios - unknown destination = the block stands, and
    // near-standstill there is no meaningful match (clutchless into 1st
    // at a stop grinds, as it should).
    bool revmatched(const NcxValues& v, const GearRatios* ratios) const
    {
        if (m_p.rpmMatchPct <= 0.0 || !ratios) return false;
        if (!v.have[NcxValues::Rpm] || !v.have[NcxValues::SpeedKmh] ||
            !v.have[NcxValues::Gear]) return false;
        const double sp = v.val[NcxValues::SpeedKmh];
        if (sp < 5.0) return false;
        const int g = (int)(v.val[NcxValues::Gear] < 0.0
                          ? v.val[NcxValues::Gear] - 0.5
                          : v.val[NcxValues::Gear] + 0.5);
        for (int t = g - 1; t <= g + 1; t += 2)
        {
            if (t < 1 || t >= MAX_GEARS || !ratios->known[t]) continue;
            const double want = ratios->r[t] * sp;
            if (want > 0.0 &&
                std::fabs(v.val[NcxValues::Rpm] - want) <= m_p.rpmMatchPct / 100.0 * want)
                return true;
        }
        return false;
    }

    DeviceParams m_p;
};
