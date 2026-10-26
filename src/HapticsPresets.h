// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// HapticsPresets - the shipped starting points (presets.json beside the
// executable, refreshed by every update, like cars.json).
//
// A starting point is a named, partial haptics object for a type of car
// (road car, cup car, GT3, open wheeler): only the settings that depend
// on the type (slip, road, kerb, wheels, impacts, and beltFullG, the g a
// belt on the car's g reaches its max at), under the same tile and
// setting names as rig.json's haptics object. Applying one is a merge over
// the live set (Config::readHapticsObject) that leaves everything it does
// not name exactly as it was. It never carries routes, master gain, the
// position budget or a setting the car table owns (CarTable::carKeys):
// those are the rig's and the car's, so the loader drops any of them a
// file names, with a log line, as it does an unknown tile or setting.
// Unlike a profile (the whole strip, routes included, so rig-specific and
// never shipped) a starting point fits every rig.
//
// The order is the file's. A missing or unreadable file is no presets.
//
// Loaded once at start; read by the web thread only.
// ============================================================

#include <QJsonObject>
#include <mutex>
#include <string>
#include <vector>

class HapticsPresets
{
public:
    struct Preset
    {
        std::string name;
        std::string description;
        QJsonObject haptics;   // the partial object, unknown and foreign keys dropped
    };

    bool load(const std::string& path);   // presets.json; false = none (missing or unreadable)

    std::vector<Preset> list() const;      // in file order
    bool   has(const std::string& name) const;
    Preset get(const std::string& name) const;   // empty name when unknown
    int    count() const;

    // A file's haptics object reduced to what a starting point may set:
    // known tiles, their own settings, numbers only, never routes or a
    // car-owned setting. Each thing dropped is logged against the preset.
    static QJsonObject clean(const QJsonObject& h, const std::string& preset);

private:
    std::vector<Preset> m_presets;
    mutable std::mutex  m_mx;
};
