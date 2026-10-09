// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#include "CarTable.h"
#include "HapticsRegistry.h"
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <algorithm>
#include <cctype>

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
    bool ok = false;
    const QJsonObject root = readJsonFile(userPath(anchorPath), ok);
    if (!ok) return false;
    std::map<std::string, Row> rows;
    readRows(root, rows, nullptr);
    std::lock_guard<std::mutex> lk(m_mx);
    m_user.rows.clear();
    for (auto& kv : rows) m_user.rows[normaliseKey(kv.first)] = kv.second;
    rebuildNames(m_user);
    return true;
}

bool CarTable::saveUser(const std::string& anchorPath) const
{
    QJsonObject root, cars;
    {
        std::lock_guard<std::mutex> lk(m_mx);
        for (const auto& kv : m_user.rows) cars[QString::fromStdString(kv.second.key)] = kv.second.entry;
    }
    root["_help"] = "nullCAT car table, your layer: how a car's engine, limiter, driveline, ABS and TC feel, "
                    "saved from the Haptics strip with 'save for this car'. Wins over the shipped cars.json "
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

CarTable::Match CarTable::find(const std::string& game, const std::string& car) const
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
    if (dash != std::string::npos && dash > 0) return find(game, car.substr(0, dash));
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
    const auto section = [&](const char* from, const char* to)
    {
        if (!entry.value(from).isObject()) return;
        QJsonObject o = entry.value(from).toObject();
        o.remove("routes");
        h[to] = o;
    };
    section("engine", engineKey());
    if (h.contains(engineKey()))
    {
        QJsonObject o = h.value(engineKey()).toObject();
        if (!o.contains("maxRpm")) o["maxRpm"] = 0.0;
        if (!o.contains("order"))  o["order"]  = 0.0;
        h[engineKey()] = o;
    }
    section("limiter",   "limiter");
    section("driveline", "driveline");
    section("abs",       "abs");
    section("tc",        "tc");
    return h;
}

QJsonObject CarTable::fromHapticsObject(const QJsonObject& haptics,
                                        const std::string& name, const std::string& notes)
{
    static const char* const kEngine[]    = { "ampPct", "freqHz", "jitter", "cylinders", "litres", "layout",
                                              "maxRpm", "order", "rock", "thump", "buzz", "limHit", "limHz",
                                              "limJit", "inertia", "turbo", "liftoff", "pops", nullptr };
    static const char* const kLimiter[]   = { "ampPct", "freqHz", "jitter", nullptr };
    static const char* const kDriveline[] = { "ampPct", "jitter", "clutch", "clutchHz", "lug", "lugHz",
                                              "gearbox", "whine", "shunt", "shuntHz", nullptr };
    static const char* const kPulse[]     = { "ampPct", "freqHz", nullptr };
    const auto pick = [&](const char* from, const char* const* keys)
    {
        QJsonObject o;
        const QJsonObject src = haptics.value(from).toObject();
        for (const char* const* k = keys; *k; ++k)
            if (src.contains(*k)) o[*k] = src.value(*k);
        // Learned values are not part of a car's description.
        if (o.value("maxRpm").toDouble(0.0) <= 0.0) o.remove("maxRpm");
        if (o.value("order").toDouble(0.0)  <= 0.0) o.remove("order");
        return o;
    };
    QJsonObject e;
    e["name"]      = QString::fromStdString(name);
    e["engine"]    = pick(engineKey(), kEngine);
    e["limiter"]   = pick("limiter",   kLimiter);
    e["driveline"] = pick("driveline", kDriveline);
    e["abs"]       = pick("abs",       kPulse);
    e["tc"]        = pick("tc",        kPulse);
    e["notes"]     = QString::fromStdString(notes);
    return e;
}
