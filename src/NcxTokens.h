// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// NcxTokens - the sim channel token registry (Docs/PROTOCOL.md), the one
// table every consumer reads: the parser (NULLCATY key=value lines name
// tokens directly), the bindings (NULLCATX slot -> token), config
// validation, the status surface and the web (served, never copied).
//
// Order IS the protocol's recommended slot order and the status array
// order; append only.
// ============================================================

#include <cstring>

struct NcxTok
{
    enum Token
    {
        Rpm, SpeedKmh, Gear, ClutchPct, ThrottlePct,
        BrakePct, AbsActive, Skid, Lockup, RoadNoise,
        Limiter, TcActive, Curbs,
        // protocol 1.3
        MaxRpm,
        SlipAngleFL, SlipAngleFR, SlipAngleRL, SlipAngleRR,     // deg, signed
        SlipRatioFL, SlipRatioFR, SlipRatioRL, SlipRatioRR,     // ratio, signed (- locking, + spinning)
        WheelSpeedFL, WheelSpeedFR, WheelSpeedRL, WheelSpeedRR, // any unit; rolling factor learned
        LoadFL, LoadFR, LoadRL, LoadRR,                         // N (any unit; relative)
        SuspVelFL, SuspVelFR, SuspVelRL, SuspVelRR,             // mm/s, signed
        Boost,                                                  // bar, turbo boost (negative = vacuum)
        PitLimiter,                                             // 0 or 1; pit limiter engaged
        // protocol 1.4
        WheelSlipFL, WheelSlipFR, WheelSlipRL, WheelSlipRR,     // combined slip per wheel, 0..100 (100 = let go); lateral source where no slip angle exists
        SuspTravelFL, SuspTravelFR, SuspTravelRL, SuspTravelRR, // suspension travel, mm, signed (the road from sims that give position, not velocity)
        // protocol 1.5
        RoadHeightFL, RoadHeightFR, RoadHeightRL, RoadHeightRR, // height of the road under each tyre, mm, world (hills included; nullCAT filters them)
        SurfaceFL, SurfaceFR, SurfaceRL, SurfaceRR,             // surface class under each tyre (SurfaceClass below)
        AccHeave,                                               // body vertical acceleration, m/s^2, up positive
        PitchDeg, RollDeg,                                      // body pitch and roll, degrees
        Wheelbase, TrackWidth,                                  // car geometry, m (contact patch to contact patch)
        // protocol 1.6
        Rain,                                                   // rain falling, 0..1 (0 dry weather, 1 the heaviest)
        Wet,                                                    // water on the road, 0..1 (0 dry, 1 standing water), where the sim knows it
        WetFL, WetFR, WetRL, WetRR,                             // ...under each tyre, where the sim knows it (wins over wet)
        TokenCount
    };
};

// What the ground under a tyre is, as a sender reports it (protocol 1.5;
// snow to water since 1.6). Senders map their sim's own surface names onto
// these.
enum SurfaceClass { SurfTarmac = 0, SurfBumpy = 1, SurfKerb = 2, SurfGravel = 3, SurfGrass = 4,
                    SurfDirt = 5, SurfCobbles = 6, SurfSnow = 7, SurfIce = 8, SurfSand = 9,
                    SurfMud = 10, SurfWater = 11, SURFACE_CLASS_COUNT = 12 };

// Wheel order used by every per-wheel token group.
enum Wheel { WheelFL = 0, WheelFR = 1, WheelRL = 2, WheelRR = 3, WHEEL_COUNT = 4 };

inline const char* ncxTokenName(int t)
{
    static const char* const kNames[NcxTok::TokenCount] = {
        "rpm", "speedKmh", "gear", "clutchPct", "throttlePct",
        "brakePct", "absActive", "skid", "lockup", "roadNoise",
        "limiter", "tcActive", "curbs",
        "maxRpm",
        "slipAngleFL", "slipAngleFR", "slipAngleRL", "slipAngleRR",
        "slipRatioFL", "slipRatioFR", "slipRatioRL", "slipRatioRR",
        "wheelSpeedFL", "wheelSpeedFR", "wheelSpeedRL", "wheelSpeedRR",
        "loadFL", "loadFR", "loadRL", "loadRR",
        "suspVelFL", "suspVelFR", "suspVelRL", "suspVelRR",
        "boost", "pitLimiter",
        "wheelSlipFL", "wheelSlipFR", "wheelSlipRL", "wheelSlipRR",
        "suspTravelFL", "suspTravelFR", "suspTravelRL", "suspTravelRR",
        "roadHeightFL", "roadHeightFR", "roadHeightRL", "roadHeightRR",
        "surfaceFL", "surfaceFR", "surfaceRL", "surfaceRR",
        "accHeave", "pitchDeg", "rollDeg", "wheelbase", "trackWidth",
        "rain", "wet", "wetFL", "wetFR", "wetRL", "wetRR",
    };
    return (t >= 0 && t < NcxTok::TokenCount) ? kNames[t] : "";
}

// Token index for a name span (not NUL-terminated); -1 when unknown.
// Exact, case-sensitive: the names are the wire contract.
inline int ncxTokenIndexN(const char* s, int n)
{
    if (n <= 0) return -1;
    for (int t = 0; t < NcxTok::TokenCount; ++t)
    {
        const char* name = ncxTokenName(t);
        if (static_cast<int>(std::strlen(name)) == n && std::memcmp(name, s, static_cast<size_t>(n)) == 0)
            return t;
    }
    return -1;
}

// First token of each per-wheel group; the wheel index adds to it.
inline int ncxWheelToken(int group, int wheel) { return group + wheel; }
