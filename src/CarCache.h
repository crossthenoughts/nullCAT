// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// CarCache - persistence for the gear-ratio learner (carcache.json,
// beside host.json/rig.json). NOT config: it is learned machine state,
// so it is never merged, validated, or edited - just remembered.
//
// Runs on the MAIN thread only: loaded once at startup (fed to
// MotionController::setCarCache before the RT loop runs) and fed from
// the published learner snapshot (MotionStatus): a car the sim moved
// away from as the mains poll, the car being driven at shutdown. A car
// entry is a ratio set; merge() matches a session's observations against
// the remembered cars exactly the way the learner adopts (>= 2 shared
// gears, all within tolerance), updates the match (moving it to the
// front - most-recently-driven order), or remembers a new car. Only
// SESSION-CONFIDENT gears are ever written, so an adopted value can
// never persist itself.
// ============================================================

#include "GearRatioLearner.h"   // CachedCar, MAX_GEARS, tolerances
#include <cstdint>
#include <string>
#include <vector>

class CarCache
{
public:
    // anchorPath: the same config anchor the mains hand to Config::load.
    bool load(const std::string& anchorPath);
    bool save(const std::string& anchorPath) const;

    // From the learner's published status; each returns true when the
    // cache changed (the caller saves).
    //   takeSession:  the car being driven (MotionStatus gearRatio*): at
    //                 shutdown, when the snapshot is final. dirty = the
    //                 session taught something the cache did not have.
    //   takeFinished: the car the sim moved away from (gearRatioPrev*),
    //                 published with a count that steps once per finished
    //                 car: merged once per step, from a periodic poll, so
    //                 a box that is powered off rather than stopped still
    //                 keeps every car but the last.
    bool takeSession(const double r[MAX_GEARS], const bool confident[MAX_GEARS], bool dirty);
    bool takeFinished(uint32_t seq, const double r[MAX_GEARS], const bool confident[MAX_GEARS], bool dirty);

    bool merge(const double r[MAX_GEARS], const bool confident[MAX_GEARS]);

    const std::vector<CachedCar>& cars() const { return m_cars; }

private:
    std::vector<CachedCar> m_cars;   // front = most recently driven
    uint32_t m_finishedSeq = 0;      // the last gearRatioPrevSeq taken
};
