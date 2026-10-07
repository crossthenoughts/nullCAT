// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// HapticsProfiles - named sets of the whole haptics layer (every tile's
// settings and routes, the engine description, master, position budget:
// the rig.json haptics object exactly), kept in profiles.json beside the
// rig config, with bindings from a car or a game name to a profile so
// the layer can switch itself when the sim names what is running.
//
// The live set in rig.json stays the working copy: saving a profile
// snapshots it, loading one copies the profile over it (and the owner
// applies it live). Bindings: "car:<name>" is looked up first, then
// "game:<name>"; no binding = no switching.
//
// Main/web thread only, like CarCache and EffectStatus.
// ============================================================

#include <QJsonObject>
#include <map>
#include <mutex>
#include <string>
#include <vector>

class HapticsProfiles
{
public:
    bool load(const std::string& anchorPath);
    bool save(const std::string& anchorPath) const;

    std::vector<std::string> names() const;                 // sorted
    bool        has(const std::string& name) const;
    QJsonObject get(const std::string& name) const;         // empty when unknown
    void        put(const std::string& name, const QJsonObject& haptics);
    bool        remove(const std::string& name);            // also drops its bindings

    std::string active() const;
    void        setActive(const std::string& name);

    // Bindings. key = "car:<name>" or "game:<name>".
    void        bind(const std::string& key, const std::string& profile);
    void        unbind(const std::string& key);
    std::map<std::string, std::string> bindings() const;
    // The profile a car/game pair should run: the car binding wins, then
    // the game's; "" when neither is bound.
    std::string profileFor(const std::string& car, const std::string& game) const;

    static std::string carKey(const std::string& car)   { return "car:" + car; }
    static std::string gameKey(const std::string& game) { return "game:" + game; }

private:
    std::map<std::string, QJsonObject> m_profiles;
    std::map<std::string, std::string> m_bindings;
    std::string m_active;
    mutable std::mutex m_mx;
};
