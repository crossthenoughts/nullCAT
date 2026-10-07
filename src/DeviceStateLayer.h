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
#include <cstring>
#include <string>
#include <vector>

// Semantic channel values after binding resolution (NcxTokens.h is the
// registry). The 0.9.6 tokens feed the haptic effect suite: brakePct/
// absActive gate the ABS pulse, skid/lockup/roadNoise are 0-100
// magnitudes computed sender-side. The 1.3 per-wheel tokens are RAW
// physics (slip angle, slip ratio, wheel speed, load, suspension velocity)
// that nullCAT turns into effect levels itself, so any sender stays a
// dumb template. An unbound or absent channel leaves its effect inert.
struct NcxValues : NcxTok
{
    bool   fresh = false;             // channel stream alive (<500 ms)
    bool   have[TokenCount] = {};     // token bound AND present in the packet
    double val[TokenCount]  = {};
    char   game[NCY_STR_LEN] = {};    // NULLCATY identity, empty when unsent
    char   car[NCY_STR_LEN]  = {};
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
    return ncxTokenIndexN(t.data(), static_cast<int>(t.size()));
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
        // Named (NULLCATY) values are canonical units by contract: no
        // scale/offset, and they win over a slot binding for the same token.
        for (int t = 0; t < NcxTok::TokenCount; ++t)
            if (td.ncyHave[t]) { v.have[t] = true; v.val[t] = td.ncy[t]; }
        std::memcpy(v.game, td.game, NCY_STR_LEN);
        std::memcpy(v.car,  td.car,  NCY_STR_LEN);
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
