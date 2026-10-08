// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// CarTable - how each car feels: the shipped table (cars.json beside the
// executable, refreshed by every update) and the user's own layer
// (cars.local.json beside the config, ferried like profiles.json).
//
// An entry carries only what belongs to the CAR: the engine's description
// and character, the limiter, driveline, ABS and TC amplitudes and
// carriers, a display name and free-text notes (the real-world figures
// the values came from). Never routes, master gain, the position budget
// or any other rig tuning: applying an entry is a merge over the live
// haptics set (Config::readHapticsObject) that leaves everything the
// entry does not name exactly as it was. A profile replaces the whole
// strip; a car entry moves five tiles' amps and character.
//
// Keys: "<game>:<car>". The game is the short id from the file's "games"
// map (ac, ams2, ...; the stream's SimHub game name resolves to it), the
// car is what the sim names (SimHub CarModel: the content folder id for
// AC, the in-game name for AMS2). Lookups normalise both halves (lower
// case, letters and digits only) so case and punctuation in the stream
// never miss an entry. The user layer wins over stock on the same key.
//
// Main/web thread only, like HapticsProfiles.
// ============================================================

#include <QJsonObject>
#include <map>
#include <mutex>
#include <string>
#include <vector>

class CarTable
{
public:
    struct Match
    {
        std::string key;      // the entry's key as written in its file
        std::string name;
        std::string notes;
        std::string source;   // "stock", "user", or "" when nothing matched
        QJsonObject entry;
    };
    struct Item { std::string key, name, game, source; };

    bool loadStock(const std::string& path);         // cars.json
    bool loadUser(const std::string& anchorPath);    // cars.local.json beside the config
    bool saveUser(const std::string& anchorPath) const;

    int stockCount() const;
    int userCount() const;

    // The entry for what the sim names: the user layer first, then stock.
    Match find(const std::string& game, const std::string& car) const;
    // An entry by its key (either layer, user first), e.g. one picked from
    // the list to use as a starting point for another car.
    Match findKey(const std::string& key) const;
    // Every entry, user and stock, sorted by game then name.
    std::vector<Item> list() const;

    // The key an entry for this game/car is filed under: the game's short
    // id when the table knows the game, else the game name itself.
    std::string keyFor(const std::string& game, const std::string& car) const;
    void putUser(const std::string& key, const QJsonObject& entry);
    bool removeUser(const std::string& key);

    // An entry as the partial haptics object Config::readHapticsObject
    // merges: the engine under its registry key, limiter, driveline, abs,
    // tc; never routes. maxRpm and buzz order fall back to "learn" unless
    // the entry sets them (they belong to the car too).
    static QJsonObject toHapticsObject(const QJsonObject& entry);
    // The reverse: the car-owned fields of a haptics object as an entry.
    static QJsonObject fromHapticsObject(const QJsonObject& haptics,
                                         const std::string& name, const std::string& notes);
    // Lower case, letters and digits only.
    static std::string normalise(const std::string& s);

private:
    struct Row { std::string key; QJsonObject entry; };
    std::map<std::string, Row> m_stock, m_user;          // by normalised key
    std::map<std::string, std::string> m_games;          // normalised game name -> short id
    mutable std::mutex m_mx;

    // Both halves normalised, the game resolved through the alias map.
    // Callers hold m_mx.
    std::string normaliseKey(const std::string& key) const;
    static bool readRows(const QJsonObject& root, std::map<std::string, Row>& into,
                         std::map<std::string, std::string>* games);
};
