// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// HapticLaws - the sim channels -> effect level laws, all of them, in one
// place and out of the controller.
//
// Each law is a few lines: which NcxValues tokens drive an effect and how
// (a flag, a 0-100 magnitude, an edge). Everything fails safe: a stale
// stream (NcxValues::fresh false), an unbound token or a zero magnitude
// all drive level 0, and the layer's release ramp fades the effect out
// rather than cutting it. Magnitude channels are 0-100 by wire convention.
//
// A web Test preview can force one effect to full level for a short
// time; the timers live in LawsState and decay here on the RT thread.
// Pure functions of their inputs: testable with a hand-built NcxValues
// and no controller.
// ============================================================

#include "HapticsLayer.h"
#include "HapticsRegistry.h"
#include "DeviceStateLayer.h"   // NcxValues
#include <algorithm>

namespace haptics {

struct LawsState
{
    double previewSec[FX_TYPE_COUNT] = {};   // >0 = Test preview running for that slot
    int    lastGear = 0;
    bool   gearSeen = false;
};

namespace laws_k {
    constexpr double kEngineAliveRpm   = 400.0;    // alive from just above cranking so idle chunks
    constexpr double kPreviewIdleRpm   = 1100.0;   // the canned idle a Test preview runs
    constexpr double kAbsMinBrakePct   = 10.0;     // ABS needs the brake actually applied
    constexpr double kPreviewSec       = 2.0;      // Test preview length
}

// Start a Test preview on a continuous/engine slot.
inline void startPreview(LawsState& st, FxType t)
{
    st.previewSec[static_cast<int>(t)] = laws_k::kPreviewSec;
}

// Drive every continuous effect and the gear-shift transient for THIS
// cycle from the channel values. Call before Layer::step().
inline void driveLaws(Layer& L, LawsState& st, const NcxValues& v, double dtSec)
{
    using namespace laws_k;
    const bool live = v.fresh;

    const auto previewOr = [&](FxType t, double level) -> double
    {
        double& left = st.previewSec[static_cast<int>(t)];
        if (left <= 0.0) return level;
        left -= dtSec;
        return 1.0;
    };
    const auto mag = [&](NcxValues::Token t) -> double
    {
        if (!live || !v.have[t]) return 0.0;
        return std::max(0.0, std::min(1.0, v.val[t] / 100.0));
    };
    const auto flag = [&](NcxValues::Token t) -> double
    { return (live && v.have[t] && v.val[t] > 0.5) ? 1.0 : 0.0; };

    // Engine: firing rate from rpm and the engine description; throttle is
    // the load; the limiter flag drives the cut gate. A preview with no sim
    // runs a canned idle.
    {
        const EngineParams& e = L.engineParams();
        const double cyl = std::max(1.0, e.cylinders);
        const double perRev = (e.layout > 2.5) ? cyl : cyl / 2.0;   // Wankel: one firing per rotor per rev
        double level = 0.0, fireHz = 0.0, load = 0.5;
        const bool lim = flag(NcxValues::Limiter) > 0.5;
        if (live && v.have[NcxValues::Rpm] && v.val[NcxValues::Rpm] > kEngineAliveRpm)
        {
            level  = 1.0;
            fireHz = v.val[NcxValues::Rpm] / 60.0 * perRev;
            if (v.have[NcxValues::ThrottlePct])
                load = std::max(0.0, std::min(1.0, v.val[NcxValues::ThrottlePct] / 100.0));
        }
        const double lv = previewOr(FxType::RpmVibe, level);
        if (lv > 0.0 && fireHz < 0.5)
            fireHz = kPreviewIdleRpm / 60.0 * perRev;
        L.driveEngine(lv, fireHz, load, lim);
    }

    // ABS: only while the sim says ABS is cycling AND the brake is applied
    // (some sims flicker the flag at zero brake).
    {
        const bool on = flag(NcxValues::AbsActive) > 0.5
                        && live && v.have[NcxValues::BrakePct] && v.val[NcxValues::BrakePct] > kAbsMinBrakePct;
        L.driveFx(FxType::AbsPulse, previewOr(FxType::AbsPulse, on ? 1.0 : 0.0), 0.0);
    }

    // Magnitude-driven textures (0-100 on the wire).
    L.driveFx(FxType::Lockup, previewOr(FxType::Lockup, mag(NcxValues::Lockup)),    0.0);
    L.driveFx(FxType::Skid,   previewOr(FxType::Skid,   mag(NcxValues::Skid)),      0.0);
    L.driveFx(FxType::Road,   previewOr(FxType::Road,   mag(NcxValues::RoadNoise)), 0.0);
    L.driveFx(FxType::Kerb,   previewOr(FxType::Kerb,   mag(NcxValues::Curbs)),     0.0);

    // Flag-driven pulses (0/1 on the wire).
    L.driveFx(FxType::Limiter, previewOr(FxType::Limiter, flag(NcxValues::Limiter)),  0.0);
    L.driveFx(FxType::TcPulse, previewOr(FxType::TcPulse, flag(NcxValues::TcActive)), 0.0);

    // Gear-shift thunk: a transient on every gear-channel CHANGE (up or
    // down). Edge state seeds on first sight and clears on staleness, so a
    // returning stream never fires a stale crossing.
    if (live && v.have[NcxValues::Gear])
    {
        const double g0 = v.val[NcxValues::Gear];
        const int g = static_cast<int>(g0 < 0.0 ? g0 - 0.5 : g0 + 0.5);
        if (st.gearSeen && g != st.lastGear)
            L.fire(EventType::GearShift, 1.0);
        st.lastGear = g;
        st.gearSeen = true;
    }
    else
    {
        st.gearSeen = false;
    }
}

} // namespace haptics
