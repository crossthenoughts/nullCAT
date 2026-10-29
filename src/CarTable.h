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
// map (ac, ams2, ...; whatever the sender calls the game resolves to it),
// the car is its stable id in that sim (the content folder for AC, the
// in-game name for AMS2). Each entry also carries the car's display
// "name" as the sim shows it, and a lookup matches EITHER: senders differ
// (SimHub names the car, a shared-memory reader has the folder id, a
// hand-written feeder has whatever it has) and the table must recognise
// the car from any of them. Lookups normalise both halves (lower case,
// letters and digits only) so case and punctuation never miss an entry.
// The user layer wins over stock whichever way it matched. A name with no
// entry of its own can still land on a near match: the longest entry of
// the same game that the name starts with, in whole words, at least two
// ("MINI Cooper S 1965 B" is the "MINI Cooper S 1965"). A long name the
// sim cut short lands on the one entry of the same game that begins with
// it (AC EVO's "BMW M3 E30 Sport Evo (Evolution" is the "... (Evolution
// III)").
//
// Thread-safe: its own mutex guards the table (the web thread and the
// status thread, following the car, both use it).
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
        bool        nearMatch = false;   // the name only starts with this entry's
    };
    struct Item { std::string key, name, game, source; };

    bool loadStock(const std::string& path);         // cars.json
    bool loadUser(const std::string& anchorPath);    // cars.local.json beside the config
    // Refused (false) while the file is there but could not be read at
    // load: a save would write this layer (empty, or only what was added
    // since) over every car in it. userError() says why.
    bool saveUser(const std::string& anchorPath) const;
    std::string userError() const;

    int stockCount() const;
    int userCount() const;

    // The entry for what the sender names, by its key (stable id) or its
    // display name: the user layer first, then stock; then the base car of
    // a " - " variant; then, with nearOk, a long name the sim cut short
    // (the one entry of the same game whose name starts with it), then a
    // near match (the longest entry of the same game the name starts
    // with, whole words, at least two).
    Match find(const std::string& game, const std::string& car, bool nearOk = true) const;
    // An entry by its key only (either layer, user first), e.g. one picked
    // from the list to use as a starting point for another car.
    Match findKey(const std::string& key) const;
    // Every entry, user and stock, sorted by game then name.
    std::vector<Item> list() const;
    // Whether either layer has any entry for this game (its name or alias).
    bool coversGame(const std::string& game) const;
    // Whether either layer has any entry at all: with a table, a car it has
    // no entry for (in any game) gets the defaults when the strip follows
    // the car; with none (no cars.json), the user's settings stay.
    bool anyEntries() const;

    // The key an entry for this game/car is filed under: the game's short
    // id when the table knows the game, else the game name itself.
    std::string keyFor(const std::string& game, const std::string& car) const;
    void putUser(const std::string& key, const QJsonObject& entry);
    bool removeUser(const std::string& key);

    // An entry as the partial haptics object Config::readHapticsObject
    // merges: the engine under its registry key, limiter, driveline, abs,
    // tc, each cut down to the keys the car owns (carKeys): never routes,
    // never a rig-owned setting however the entry was written. maxRpm
    // falls back to "learn" unless the entry sets it (it belongs to the car
    // too). The buzz order is the rig's and never crosses.
    static QJsonObject toHapticsObject(const QJsonObject& entry);
    // The reverse: the car-owned fields of a haptics object as an entry.
    static QJsonObject fromHapticsObject(const QJsonObject& haptics,
                                         const std::string& name, const std::string& notes);
    // Every car-owned setting at its default (what a fresh install has,
    // each tile's strength at the table's typical one), as the partial
    // haptics object readHapticsObject merges: what a car with no entry
    // gets when the strip follows the car, so it never keeps the last
    // car's engine, gearbox, ABS or TC.
    static QJsonObject defaultHapticsObject();
    // What the sender says about the car's engine (protocol 1.11:
    // engCylinders, engLayout, engLitres, engTurbo, engLimitHz, by name),
    // as the partial haptics object readHapticsObject merges: the engine
    // tile's cylinders, layout, litres, turbo and limiter hz, each clamped
    // to its range and only where sent. Empty when it says nothing. For a
    // car with no entry: an entry, the table's or yours, always wins.
    static QJsonObject engineFromSender(const bool have[], const double val[]);
    // The starting point a sender's type of car (carType) names: Road car,
    // Cup car, GT3 / GT, Open wheeler, Rally gravel, Rally tarmac; "" for
    // a type it does not know.
    static std::string startingPointForCarType(double carType);
    // The settings of one tile (its haptics key) that belong to the car:
    // what a car's entry sets and "save for this car" stores. Empty for a
    // tile the table never touches.
    static std::vector<std::string> carKeys(const std::string& tileKey);
    // Lower case, letters and digits only.
    static std::string normalise(const std::string& s);

private:
    struct Row { std::string key; QJsonObject entry; };
    struct Layer
    {
        std::map<std::string, Row>         rows;     // by normalised key
        std::map<std::string, std::string> byName;   // normalised "<game>:<name>" -> normalised key
    };
    Layer m_stock, m_user;
    std::string m_userError;   // cars.local.json there but unreadable at load ("" = fine)
    std::map<std::string, std::string> m_games;          // normalised game name -> short id
    mutable std::mutex m_mx;

    // Both halves normalised, the game resolved through the alias map.
    // Callers hold m_mx.
    std::string normaliseKey(const std::string& key) const;
    void   rebuildNames(Layer& layer) const;
    const Row* lookup(const Layer& layer, const std::string& nk) const;   // key, then name
    Match findNear(const std::string& game, const std::string& car) const;
    // A long name the sim cut short: the one entry of the game it begins.
    Match findCut(const std::string& game, const std::string& car) const;
    static constexpr size_t kCutMinChars = 30;   // the shortest cut seen: AC EVO's 31 (30 if it ends on a space)
    static Match toMatch(const Row& row, const char* source);
    static bool readRows(const QJsonObject& root, std::map<std::string, Row>& into,
                         std::map<std::string, std::string>* games);
};
