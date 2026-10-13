// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#include "HapticsPresets.h"
#include "CarTable.h"
#include "HapticsRegistry.h"
#include "Logging.h"
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <algorithm>
#include <cctype>

namespace {
// The keys of one member object of a JSON text's top-level object, in the
// order they are written: QJsonObject sorts its keys, and the starting
// points are offered in the file's order. Only ever run on text the JSON
// parser has already accepted, so it walks well-formed JSON; the keys come
// back decoded by that parser.
class KeyOrder
{
public:
    explicit KeyOrder(const QByteArray& text) : s(text) {}

    std::vector<QString> of(const QString& member)
    {
        std::vector<QString> out;
        ws();
        if (!eat('{')) return out;
        for (;;)
        {
            ws();
            if (peek() != '"') return out;
            const QString key = str();
            ws(); if (!eat(':')) return out; ws();
            if (key == member && eat('{'))
            {
                for (;;)
                {
                    ws();
                    if (peek() != '"') return out;
                    out.push_back(str());
                    ws(); if (!eat(':')) return out;
                    ws(); skipValue(); ws();
                    if (!eat(',')) return out;
                }
            }
            skipValue(); ws();
            if (!eat(',')) return out;
        }
    }

private:
    const QByteArray& s;
    int i = 0;
    char peek() const { return i < s.size() ? s[i] : '\0'; }
    bool eat(char c)  { if (peek() != c) return false; ++i; return true; }
    void ws()         { while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i; }
    void skipString()
    {
        ++i;
        while (i < s.size() && s[i] != '"') { if (s[i] == '\\') ++i; ++i; }
        ++i;
    }
    QString str()
    {
        const int start = i;
        skipString();
        const QJsonDocument d = QJsonDocument::fromJson(QByteArray("[") + s.mid(start, i - start) + "]");
        return d.array().at(0).toString();
    }
    // One value: a string, a number or word, or a whole object or array.
    void skipValue()
    {
        int depth = 0;
        while (i < s.size())
        {
            const char c = s[i];
            if (c == '"') { skipString(); if (depth == 0) return; continue; }
            if (c == '{' || c == '[') ++depth;
            else if (c == '}' || c == ']')
            {
                if (depth == 0) return;
                if (--depth == 0) { ++i; return; }
            }
            else if (depth == 0 && (c == ',' || std::isspace(static_cast<unsigned char>(c)))) return;
            ++i;
        }
    }
};
}

QJsonObject HapticsPresets::clean(const QJsonObject& h, const std::string& preset)
{
    QJsonObject out;
    for (auto it = h.begin(); it != h.end(); ++it)
    {
        const std::string tile = it.key().toStdString();
        const haptics::EffectInfo* info = haptics::findEffect(tile.c_str());
        if (!info || !it.value().isObject())
        {
            LOG_WARNING(strf("Haptics: starting point '%s': '%s' is not a tile (master gain, the position budget "
                             "and the like are the rig's), ignored.", preset.c_str(), tile.c_str()));
            continue;
        }
        const std::vector<std::string> car = CarTable::carKeys(tile);
        const QJsonObject src = it.value().toObject();
        QJsonObject o;
        for (auto kt = src.begin(); kt != src.end(); ++kt)
        {
            const std::string key = kt.key().toStdString();
            bool known = false;
            for (int k = 0; k < info->paramCount && !known; ++k) known = (key == info->params[k].key);
            const char* why = (key == "routes")                                  ? "routes are the rig's"
                            : !known                                             ? "is not one of its settings"
                            : (std::find(car.begin(), car.end(), key) != car.end()) ? "belongs to the car (the car table sets it)"
                            : !kt.value().isDouble()                             ? "is not a number"
                            : nullptr;
            if (why)
            {
                LOG_WARNING(strf("Haptics: starting point '%s': %s.%s %s, ignored.", preset.c_str(), tile.c_str(),
                                 key.c_str(), why));
                continue;
            }
            o[kt.key()] = kt.value();
        }
        if (!o.isEmpty()) out[it.key()] = o;
    }
    return out;
}

bool HapticsPresets::load(const std::string& path)
{
    std::vector<Preset> out;
    QFile f(QString::fromStdString(path));
    if (f.open(QIODevice::ReadOnly))
    {
        const QByteArray text = f.readAll();
        QJsonParseError pe{};
        const QJsonDocument doc = QJsonDocument::fromJson(text, &pe);
        if (!doc.isObject())
            LOG_WARNING(strf("Haptics: starting points %s unreadable (%s): none offered.", path.c_str(),
                             pe.errorString().toStdString().c_str()));
        else
        {
            const QJsonObject ps = doc.object().value("presets").toObject();
            std::vector<QString> names = KeyOrder(text).of("presets");
            for (auto it = ps.begin(); it != ps.end(); ++it)   // the walk missed none, but never lose one
                if (std::find(names.begin(), names.end(), it.key()) == names.end()) names.push_back(it.key());
            for (const QString& n : names)
            {
                const std::string name = n.toStdString();
                if (!ps.contains(n) || std::any_of(out.begin(), out.end(), [&](const Preset& p) { return p.name == name; }))
                    continue;
                const QJsonObject o = ps.value(n).toObject();
                Preset p;
                p.name        = name;
                p.description = o.value("description").toString().toStdString();
                p.haptics     = clean(o.value("haptics").toObject(), name);
                if (name.empty() || p.haptics.isEmpty())
                {
                    LOG_WARNING(strf("Haptics: starting point '%s' sets nothing a starting point may set, ignored.", name.c_str()));
                    continue;
                }
                out.push_back(p);
            }
        }
    }
    std::lock_guard<std::mutex> lk(m_mx);
    m_presets = out;
    return !m_presets.empty();
}

std::vector<HapticsPresets::Preset> HapticsPresets::list() const
{
    std::lock_guard<std::mutex> lk(m_mx);
    return m_presets;
}

bool HapticsPresets::has(const std::string& name) const
{
    return !get(name).name.empty();
}

HapticsPresets::Preset HapticsPresets::get(const std::string& name) const
{
    std::lock_guard<std::mutex> lk(m_mx);
    for (const Preset& p : m_presets)
        if (p.name == name) return p;
    return Preset{};
}

int HapticsPresets::count() const
{
    std::lock_guard<std::mutex> lk(m_mx);
    return static_cast<int>(m_presets.size());
}
