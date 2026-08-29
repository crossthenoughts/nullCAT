// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// CarCache - persistence for the gear-ratio learner (carcache.json,
// beside host.json/rig.json). NOT config: it is learned machine state,
// so it is never merged, validated, or edited - just remembered.
//
// Runs on the MAIN thread only: loaded once at startup (fed to
// MotionController::setCarCache before the RT loop runs) and saved at
// shutdown from the last published learner snapshot. A car entry is a
// ratio set; merge() matches a session's observations against the
// remembered cars exactly the way the learner adopts (>= 2 shared
// gears, all within tolerance), updates the match (moving it to the
// front - most-recently-driven order), or remembers a new car. Only
// SESSION-CONFIDENT gears are ever written, so an adopted value can
// never persist itself.
// ============================================================

#include "GearRatioLearner.h"   // CachedCar, MAX_GEARS, tolerances
#include <string>
#include <vector>

class CarCache
{
public:
    // anchorPath: the same config anchor the mains hand to Config::load.
    bool load(const std::string& anchorPath);
    bool save(const std::string& anchorPath) const;

    void merge(const double r[MAX_GEARS], const bool confident[MAX_GEARS]);

    const std::vector<CachedCar>& cars() const { return m_cars; }

private:
    std::vector<CachedCar> m_cars;   // front = most recently driven
};
