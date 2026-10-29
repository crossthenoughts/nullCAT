// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#include "CarTable.h"
#include "Config.h"
#include "HapticsRegistry.h"
#include "NcxTokens.h"
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iterator>
#include <set>

namespace {
QString userPath(const std::string& anchorPath)
{
    return QFileInfo(QString::fromStdString(anchorPath)).absolutePath() + "/cars.local.json";
}
QJsonObject readJsonFile(const QString& path, bool& ok)
{
    QFile f(path);
    ok = f.open(QIODevice::ReadOnly);
    if (!ok) return QJsonObject();
    const QJsonDocument d = QJsonDocument::fromJson(f.readAll());
    ok = d.isObject();
    return d.object();
}
const char* engineKey() { return haptics::effectInfo(haptics::Effect::Engine).key; }
}

std::string CarTable::normalise(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (unsigned char c : s)
        if (std::isalnum(c)) out.push_back(static_cast<char>(std::tolower(c)));
    return out;
}

std::string CarTable::normaliseKey(const std::string& key) const
{
    const size_t colon = key.find(':');
    if (colon == std::string::npos) return normalise(key);
    std::string game = normalise(key.substr(0, colon));
    const auto it = m_games.find(game);
    if (it != m_games.end()) game = it->second;
    return game + ":" + normalise(key.substr(colon + 1));
}

// The "cars" object of a file (or the whole file minus its housekeeping
// keys, for a hand-written one) into a layer; "games" into the alias map.
bool CarTable::readRows(const QJsonObject& root, std::map<std::string, Row>& into,
                        std::map<std::string, std::string>* games)
{
    if (games && root.value("games").isObject())
    {
        const QJsonObject g = root.value("games").toObject();
        for (auto it = g.begin(); it != g.end(); ++it)
        {
            const std::string id = normalise(it.key().toStdString());
            if (id.empty()) continue;
            (*games)[id] = id;
            if (it.value().isString()) (*games)[normalise(it.value().toString().toStdString())] = id;
            else if (it.value().isArray())
                for (const QJsonValue& v : it.value().toArray())
                    if (v.isString()) (*games)[normalise(v.toString().toStdString())] = id;
        }
    }
    QJsonObject cars;
    if (root.value("cars").isObject()) cars = root.value("cars").toObject();
    else
    {
        cars = root;
        cars.remove("games"); cars.remove("_help");
    }
    int n = 0;
    for (auto it = cars.begin(); it != cars.end(); ++it)
    {
        if (!it.value().isObject()) continue;
        Row r; r.key = it.key().toStdString(); r.entry = it.value().toObject();
        into[r.key] = r;   // keyed by the written key for now; re-keyed by the caller
        ++n;
    }
    return n > 0;
}

// The name index of a layer: "<game>:<display name>" normalised, pointing
// at the entry's key. A name that is also some entry's key is not
// indexed (the key is the authority); a name shared by two entries goes
// to the first in key order.
void CarTable::rebuildNames(Layer& layer) const
{
    layer.byName.clear();
    for (const auto& kv : layer.rows)
    {
        const std::string name = kv.second.entry.value("name").toString().toStdString();
        if (name.empty()) continue;
        const size_t colon = kv.first.find(':');
        const std::string game = colon == std::string::npos ? std::string() : kv.first.substr(0, colon);
        const std::string nn = game + ":" + normalise(name);
        if (nn == kv.first || layer.rows.count(nn)) continue;
        layer.byName.emplace(nn, kv.first);
    }
}

const CarTable::Row* CarTable::lookup(const Layer& layer, const std::string& nk) const
{
    if (const auto it = layer.rows.find(nk); it != layer.rows.end()) return &it->second;
    if (const auto in = layer.byName.find(nk); in != layer.byName.end())
        if (const auto it = layer.rows.find(in->second); it != layer.rows.end()) return &it->second;
    return nullptr;
}

bool CarTable::loadStock(const std::string& path)
{
    bool ok = false;
    const QJsonObject root = readJsonFile(QString::fromStdString(path), ok);
    if (!ok) return false;
    std::map<std::string, Row> rows;
    std::map<std::string, std::string> games;
    readRows(root, rows, &games);
    std::lock_guard<std::mutex> lk(m_mx);
    m_games = games;
    m_stock.rows.clear();
    for (auto& kv : rows) m_stock.rows[normaliseKey(kv.first)] = kv.second;
    rebuildNames(m_stock);
    // The user layer was keyed before the alias map existed: re-key it.
    std::map<std::string, Row> user;
    for (auto& kv : m_user.rows) user[normaliseKey(kv.second.key)] = kv.second;
    m_user.rows = user;
    rebuildNames(m_user);
    return true;
}

bool CarTable::loadUser(const std::string& anchorPath)
{
    const QString path = userPath(anchorPath);
    bool ok = false;
    const QJsonObject root = readJsonFile(path, ok);
    if (!ok)
    {
        // Absent: no user layer yet. There but unreadable (cut short, a
        // hand edit gone wrong): kept as it is, and saveUser refuses until
        // it is fixed or moved and nullCAT restarted.
        std::lock_guard<std::mutex> lk(m_mx);
        m_userError = QFileInfo::exists(path)
            ? path.toStdString() + " cannot be read: fix or move the file and restart (nothing is written over it)"
            : std::string();
        return false;
    }
    std::map<std::string, Row> rows;
    readRows(root, rows, nullptr);
    std::lock_guard<std::mutex> lk(m_mx);
    m_userError.clear();
    m_user.rows.clear();
    for (auto& kv : rows) m_user.rows[normaliseKey(kv.first)] = kv.second;
    rebuildNames(m_user);
    return true;
}

std::string CarTable::userError() const { std::lock_guard<std::mutex> lk(m_mx); return m_userError; }

bool CarTable::saveUser(const std::string& anchorPath) const
{
    QJsonObject root, cars;
    {
        std::lock_guard<std::mutex> lk(m_mx);
        if (!m_userError.empty()) return false;   // never written over (see loadUser)
        for (const auto& kv : m_user.rows) cars[QString::fromStdString(kv.second.key)] = kv.second.entry;
    }
    root["_help"] = "nullCAT car table, your layer: how a car's engine, limiter, driveline, ABS and TC feel, "
                    "saved from the Haptics panel with 'save as mine'. Wins over the shipped cars.json "
                    "for the same car. Never carries routes.";
    root["cars"] = cars;
    QSaveFile f(userPath(anchorPath));
    if (!f.open(QIODevice::WriteOnly)) return false;
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return f.commit();
}

int CarTable::stockCount() const { std::lock_guard<std::mutex> lk(m_mx); return static_cast<int>(m_stock.rows.size()); }
int CarTable::userCount() const  { std::lock_guard<std::mutex> lk(m_mx); return static_cast<int>(m_user.rows.size()); }

std::string CarTable::keyFor(const std::string& game, const std::string& car) const
{
    std::lock_guard<std::mutex> lk(m_mx);
    std::string id = normalise(game);
    const auto it = m_games.find(id);
    if (it != m_games.end()) id = it->second;
    std::string c = car;
    while (!c.empty() && std::isspace(static_cast<unsigned char>(c.back()))) c.pop_back();
    size_t s = 0;
    while (s < c.size() && std::isspace(static_cast<unsigned char>(c[s]))) ++s;
    return id + ":" + c.substr(s);
}

CarTable::Match CarTable::toMatch(const Row& row, const char* source)
{
    Match m;
    m.key    = row.key;
    m.entry  = row.entry;
    m.source = source;
    m.name   = row.entry.value("name").toString().toStdString();
    m.notes  = row.entry.value("notes").toString().toStdString();
    if (m.name.empty())
    {
        const size_t colon = row.key.find(':');
        m.name = colon == std::string::npos ? row.key : row.key.substr(colon + 1);
    }
    return m;
}

CarTable::Match CarTable::find(const std::string& game, const std::string& car, bool nearOk) const
{
    if (car.empty()) return Match{};
    {
        const std::string key = keyFor(game, car);
        std::lock_guard<std::mutex> lk(m_mx);
        const std::string nk = normaliseKey(key);
        if (const Row* r = lookup(m_user, nk))  return toMatch(*r, "user");
        if (const Row* r = lookup(m_stock, nk)) return toMatch(*r, "stock");
    }
    // An aero or track variant of a car the sim names with a suffix
    // ("Porsche 963 - Low Downforce", "Formula USA 2023 - Speedway") is
    // the same car to the engine and driveline: fall back to the base.
    const size_t dash = car.rfind(" - ");
    if (dash != std::string::npos && dash > 0)
        if (Match m = find(game, car.substr(0, dash), false); !m.source.empty()) return m;
    if (!nearOk) return Match{};
    // A long name cut short first: the one entry that begins with all of
    // it is more specific than a shorter entry it begins with
    // ("Giulietta QV Launch Edition 201" is the Launch Edition, not the
    // "Giulietta QV").
    if (Match m = findCut(game, car); !m.source.empty()) return m;
    return findNear(game, car);
}

// A name the sim cut short: AC EVO hands its car names over in a field of
// 31 characters ("BMW M3 E30 Sport Evo (Evolution" for "BMW M3 E30 Sport
// Evo (Evolution III)"). The one car of the same game whose name starts
// with it is the car. Only for a name as long as a cut one (kCutMinChars
// characters: a full name that merely begins a longer car's, "... GT3
// Evo" before "... GT3 Evo2", is left to the near match), and only when
// exactly one car fits; your own row for it before stock's.
CarTable::Match CarTable::findCut(const std::string& game, const std::string& car) const
{
    if (car.size() < kCutMinChars) return Match{};
    const std::string n = normalise(car);
    std::lock_guard<std::mutex> lk(m_mx);
    std::string id = normalise(game);
    if (const auto it = m_games.find(id); it != m_games.end()) id = it->second;
    const std::string prefix = id + ":" + n;
    const auto begins = [&prefix](const std::string& s) { return s.compare(0, prefix.size(), prefix) == 0; };
    // The rows whose name begins with it: by the name index, and by the key
    // (the index leaves out a name that is its own key, as AMS2's and AC
    // Rally's are; a content id never begins with a car's name).
    const auto fits = [&](const Layer& layer) -> std::set<std::string>
    {
        std::set<std::string> keys;
        for (auto it = layer.byName.lower_bound(prefix); it != layer.byName.end() && begins(it->first); ++it)
            keys.insert(it->second);
        for (auto it = layer.rows.lower_bound(prefix); it != layer.rows.end() && begins(it->first); ++it)
            keys.insert(it->first);
        return keys;
    };
    // One car across both layers (your copy of a stock car is the same
    // key), then your row for it before stock's.
    std::set<std::string> keys = fits(m_user);
    for (const std::string& k : fits(m_stock)) keys.insert(k);
    if (keys.size() != 1) return Match{};
    const Row* row = nullptr;
    const char* source = "user";
    if (const auto u = m_user.rows.find(*keys.begin()); u != m_user.rows.end()) row = &u->second;
    else if (const auto s = m_stock.rows.find(*keys.begin()); s != m_stock.rows.end()) { row = &s->second; source = "stock"; }
    if (!row) return Match{};
    Match m = toMatch(*row, source);
    m.nearMatch = true;
    return m;
}

// A near match: a car the sim names with something after a name the table
// has ("MINI Cooper S 1965 B", "Porsche 911 GT3 Cup 4.0 Evo") is taken as
// that car. Words are what lies between spaces or underscores (the spaces
// of a content folder id), each normalised like the rest of the lookup;
// the longest prefix of whole words wins, and it must be at least two
// words so a make alone ("Porsche ...") never matches a car. Only the
// same game's entries: the game is part of every key and name.
CarTable::Match CarTable::findNear(const std::string& game, const std::string& car) const
{
    std::vector<std::string> words;
    std::string word;
    for (const char c : car + " ")
    {
        if (c == '_' || std::isspace(static_cast<unsigned char>(c)))
        {
            word = normalise(word);
            if (!word.empty()) words.push_back(word);
            word.clear();
        }
        else word.push_back(c);
    }
    if (words.size() < 3) return Match{};   // two words for the entry, one more for the suffix
    std::lock_guard<std::mutex> lk(m_mx);
    std::string id = normalise(game);
    if (const auto it = m_games.find(id); it != m_games.end()) id = it->second;
    for (size_t k = words.size() - 1; k >= 2; --k)
    {
        std::string nk = id + ":";
        for (size_t i = 0; i < k; ++i) nk += words[i];
        const Row* r = lookup(m_user, nk);
        const char* source = "user";
        if (!r) { r = lookup(m_stock, nk); source = "stock"; }
        if (r)
        {
            Match m = toMatch(*r, source);
            m.nearMatch = true;
            return m;
        }
    }
    return Match{};
}

CarTable::Match CarTable::findKey(const std::string& key) const
{
    std::lock_guard<std::mutex> lk(m_mx);
    const std::string nk = normaliseKey(key);
    if (const auto it = m_user.rows.find(nk); it != m_user.rows.end())   return toMatch(it->second, "user");
    if (const auto it = m_stock.rows.find(nk); it != m_stock.rows.end()) return toMatch(it->second, "stock");
    return Match{};
}

std::vector<CarTable::Item> CarTable::list() const
{
    std::vector<Item> out;
    const auto add = [&out](const std::map<std::string, Row>& layer, const char* source)
    {
        for (const auto& kv : layer)
        {
            Item it;
            it.key    = kv.second.key;
            it.source = source;
            const size_t colon = it.key.find(':');
            it.game   = colon == std::string::npos ? std::string() : it.key.substr(0, colon);
            it.name   = kv.second.entry.value("name").toString().toStdString();
            if (it.name.empty()) it.name = colon == std::string::npos ? it.key : it.key.substr(colon + 1);
            out.push_back(it);
        }
    };
    {
        std::lock_guard<std::mutex> lk(m_mx);
        add(m_user.rows, "user");
        add(m_stock.rows, "stock");
    }
    std::sort(out.begin(), out.end(), [](const Item& a, const Item& b)
    {
        if (a.game != b.game) return a.game < b.game;
        const std::string an = normalise(a.name), bn = normalise(b.name);
        if (an != bn) return an < bn;
        return a.source < b.source;
    });
    return out;
}

bool CarTable::coversGame(const std::string& game) const
{
    std::lock_guard<std::mutex> lk(m_mx);
    std::string id = normalise(game);
    if (const auto it = m_games.find(id); it != m_games.end()) id = it->second;
    const std::string prefix = id + ":";
    const auto any = [&prefix](const Layer& layer)
    {
        const auto it = layer.rows.lower_bound(prefix);   // rows are keyed "<game>:<car>", sorted
        return it != layer.rows.end() && it->first.compare(0, prefix.size(), prefix) == 0;
    };
    return any(m_user) || any(m_stock);
}

QJsonObject CarTable::engineFromSender(const bool have[], const double val[])
{
    const auto clampTo = [](double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); };
    QJsonObject eng;
    if (have[NcxTok::EngCylinders] && std::isfinite(val[NcxTok::EngCylinders]))
        eng["cylinders"] = std::round(clampTo(val[NcxTok::EngCylinders], 1.0, 16.0));
    if (have[NcxTok::EngLayout] && std::isfinite(val[NcxTok::EngLayout]))
        eng["layout"] = std::round(clampTo(val[NcxTok::EngLayout], 0.0, 5.0));
    if (have[NcxTok::EngLitres] && std::isfinite(val[NcxTok::EngLitres]) && val[NcxTok::EngLitres] > 0.0)
        eng["litres"] = std::round(clampTo(val[NcxTok::EngLitres], 0.1, 30.0) * 10.0) / 10.0;
    if (have[NcxTok::EngTurbo] && std::isfinite(val[NcxTok::EngTurbo]))
        eng["turbo"] = val[NcxTok::EngTurbo] >= 0.5 ? 1.0 : 0.0;
    if (have[NcxTok::EngLimitHz] && std::isfinite(val[NcxTok::EngLimitHz]) && val[NcxTok::EngLimitHz] > 0.0)
        eng["limHz"] = std::round(clampTo(val[NcxTok::EngLimitHz], 4.0, 30.0));
    QJsonObject h;
    if (!eng.isEmpty()) h[engineKey()] = eng;
    return h;
}

std::string CarTable::startingPointForCarType(double carType)
{
    if (!std::isfinite(carType)) return "";
    static const char* const kTypes[] = { "Road car", "Cup car", "GT3 / GT", "Open wheeler", "Rally gravel", "Rally tarmac" };
    const long t = std::lround(carType);
    return (t >= 0 && t < static_cast<long>(std::size(kTypes))) ? kTypes[t] : "";
}

bool CarTable::anyEntries() const
{
    std::lock_guard<std::mutex> lk(m_mx);
    return !m_user.rows.empty() || !m_stock.rows.empty();
}

void CarTable::putUser(const std::string& key, const QJsonObject& entry)
{
    std::lock_guard<std::mutex> lk(m_mx);
    Row r; r.key = key; r.entry = entry;
    m_user.rows[normaliseKey(key)] = r;
    rebuildNames(m_user);
}

bool CarTable::removeUser(const std::string& key)
{
    std::lock_guard<std::mutex> lk(m_mx);
    const bool gone = m_user.rows.erase(normaliseKey(key)) > 0;
    if (gone) rebuildNames(m_user);
    return gone;
}

QJsonObject CarTable::toHapticsObject(const QJsonObject& entry)
{
    QJsonObject h;
    // Only the keys the car owns cross over (carKeys): a hand-written
    // entry carrying a rig-owned key (an ABS "sharp", the driveline's
    // "whinePitch", a route) used to move the rig's setting with the car.
    const auto section = [&](const char* from, const char* to)
    {
        if (!entry.value(from).isObject()) return;
        const QJsonObject src = entry.value(from).toObject();
        QJsonObject o;
        for (const std::string& k : carKeys(to))
        {
            const QString qk = QString::fromStdString(k);
            if (src.contains(qk)) o[qk] = src.value(qk);
        }
        h[to] = o;
    };
    section("engine", engineKey());
    if (h.contains(engineKey()))
    {
        QJsonObject o = h.value(engineKey()).toObject();
        if (!o.contains("maxRpm")) o["maxRpm"] = 0.0;
        h[engineKey()] = o;
    }
    section("limiter",   "limiter");
    section("driveline", "driveline");
    // What diff the car has and which wheels it drives are the car's: an
    // entry that does not say is a rear-driven limited slip (no other car's
    // spool or front drive carried over). How strongly the diff plays (diff
    // x, diff hz) is the car's too, but an entry that does not say leaves it
    // as it was.
    if (h.contains("driveline"))
    {
        QJsonObject o = h.value("driveline").toObject();
        if (!o.contains("diffType")) o["diffType"] = 0.0;
        if (!o.contains("driven"))   o["driven"]   = 0.0;
        h["driveline"] = o;
    }
    section("abs",       "abs");
    section("tc",        "tc");
    return h;
}

// The car-owned settings of a tile: the registry rows flagged kParamCar
// (the rest of a tile, and every other tile, is the rig's).
std::vector<std::string> CarTable::carKeys(const std::string& tileKey)
{
    std::vector<std::string> out;
    for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
    {
        const haptics::EffectInfo& info = haptics::effectInfo(i);
        if (tileKey != info.key) continue;
        for (int p = 0; p < info.paramCount; ++p)
            if (info.params[p].flags & haptics::kParamCar) out.push_back(info.params[p].key);
    }
    return out;
}

QJsonObject CarTable::fromHapticsObject(const QJsonObject& haptics,
                                        const std::string& name, const std::string& notes)
{
    const auto pick = [&](const char* from)
    {
        QJsonObject o;
        const QJsonObject src = haptics.value(from).toObject();
        for (const std::string& k : carKeys(from))
        {
            const QString key = QString::fromStdString(k);
            if (src.contains(key)) o[key] = src.value(key);
        }
        // Learned values are not part of a car's description.
        if (o.value("maxRpm").toDouble(0.0) <= 0.0) o.remove("maxRpm");
        return o;
    };
    QJsonObject e;
    e["name"]      = QString::fromStdString(name);
    e["engine"]    = pick(engineKey());
    e["limiter"]   = pick("limiter");
    e["driveline"] = pick("driveline");
    e["abs"]       = pick("abs");
    e["tc"]        = pick("tc");
    e["notes"]     = QString::fromStdString(notes);
    return e;
}

// The car-owned settings of a fresh install, through the same pick and
// merge an entry takes, so it carries exactly the keys an entry can; each
// tile at the table's typical strength (the median of the shipped entries
// that play it, measured 2026-10-14), not a fresh install's 0, which
// silenced the engine, gearbox, ABS and TC of every car with no entry.
QJsonObject CarTable::defaultHapticsObject()
{
    QJsonObject h = toHapticsObject(fromHapticsObject(Config::writeHapticsObject(AppConfig()), "", ""));
    const struct { const char* tile; double amp; } kTypical[] = {
        { engineKey(), 48.0 }, { "limiter", 30.0 }, { "driveline", 34.0 }, { "abs", 40.0 }, { "tc", 40.0 } };
    for (const auto& t : kTypical)
    {
        QJsonObject o = h.value(t.tile).toObject();
        o["ampPct"] = t.amp;
        h[t.tile] = o;
    }
    return h;
}
