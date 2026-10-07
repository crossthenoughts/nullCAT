// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#include "EffectStatus.h"
#include "HapticsTypes.h"
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <algorithm>
#include <cstring>

namespace {

QString statusPath(const std::string& anchorPath)
{
    return QFileInfo(QString::fromStdString(anchorPath)).absolutePath() + "/effectstatus.json";
}

constexpr double kProducedLevel = 0.02;   // output level that counts as "felt"

// One channel entry satisfied? "a|b" any alternative; "group*" every wheel
// token of the group; a plain name is that token.
bool entryDelivered(const char* entry, const bool* have)
{
    std::string e = entry;
    size_t pos = 0;
    while (pos <= e.size())
    {
        const size_t bar = e.find('|', pos);
        const std::string alt = e.substr(pos, bar == std::string::npos ? std::string::npos : bar - pos);
        if (!alt.empty())
        {
            if (alt.back() == '*')
            {
                const std::string group = alt.substr(0, alt.size() - 1);
                bool any = false, all = true;
                for (int t = 0; t < NcxTok::TokenCount; ++t)
                {
                    const char* name = ncxTokenName(t);
                    if (std::strncmp(name, group.c_str(), group.size()) != 0) continue;
                    any = true;
                    if (!have[t]) all = false;
                }
                if (any && all) return true;
            }
            else
            {
                const int t = ncxTokenIndexN(alt.c_str(), static_cast<int>(alt.size()));
                if (t >= 0 && have[t]) return true;
            }
        }
        if (bar == std::string::npos) break;
        pos = bar + 1;
    }
    return false;
}

} // namespace

bool EffectStatus::channelsDelivered(const haptics::EffectInfo& info, const bool* have)
{
    for (const char* c : info.channels)
    {
        if (!c) break;
        if (c[0] == '~') continue;                // optional: never required
        if (!entryDelivered(c, have)) return false;
    }
    return true;
}

void EffectStatus::observe(const std::string& game, const bool* have, const double* fxOut,
                           const uint64_t* firedBy, bool streamLive, int64_t nowMs)
{
    std::lock_guard<std::mutex> lk(m_mx);
    // Transient edges are attributed to whichever game is current; the
    // counters are tracked even when no stream is live so a detent click
    // on a bench (no sim) is still counted under the unnamed sender.
    bool firedEdge[haptics::EVENT_TYPE_COUNT] = {};
    for (int i = 0; i < haptics::EVENT_TYPE_COUNT; ++i)
    {
        firedEdge[i] = m_haveFired && firedBy[i] > m_lastFiredBy[i];
        m_lastFiredBy[i] = firedBy[i];
    }
    m_haveFired = true;
    if (!streamLive)
    {
        // Nothing arriving: only a transient with no channel need (the
        // detent click) can still be felt; record it under the last game.
        for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
        {
            const haptics::EffectInfo& info = haptics::effectInfo(i);
            if (info.kind != haptics::Kind::Transient || info.channels[0] != nullptr) continue;
            if (!firedEdge[static_cast<int>(info.event)]) continue;
            Game& g = m_games[m_current];
            EffectRecord& r = g.rec[static_cast<size_t>(i)];
            r.delivered = true; r.produced = true; r.lastSeenMs = nowMs; g.lastSeenMs = nowMs;
            m_dirty = true;
        }
        return;
    }
    if (game != m_current) { m_current = game; m_dirty = true; }
    Game& g = m_games[m_current];
    g.lastSeenMs = nowMs;
    for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
    {
        const haptics::EffectInfo& info = haptics::effectInfo(i);
        EffectRecord& r = g.rec[static_cast<size_t>(i)];
        const bool delivered = channelsDelivered(info, have);
        bool produced = false;
        if (info.kind == haptics::Kind::Transient)
            produced = firedEdge[static_cast<int>(info.event)];
        else
            produced = fxOut[static_cast<int>(info.fx)] > kProducedLevel;
        if (delivered && !r.delivered) { r.delivered = true; m_dirty = true; }
        if (produced  && !r.produced)  { r.produced  = true; m_dirty = true; }
        if (delivered || produced) r.lastSeenMs = nowMs;
    }
}

std::string EffectStatus::currentGame() const
{
    std::lock_guard<std::mutex> lk(m_mx);
    return m_current;
}

std::vector<std::string> EffectStatus::games() const
{
    std::lock_guard<std::mutex> lk(m_mx);
    std::vector<std::pair<int64_t, std::string>> v;
    for (const auto& kv : m_games) v.push_back({ kv.second.lastSeenMs, kv.first });
    std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    std::vector<std::string> out;
    for (const auto& p : v) out.push_back(p.second);
    return out;
}

EffectStatus::Records EffectStatus::records(const std::string& game) const
{
    std::lock_guard<std::mutex> lk(m_mx);
    const auto it = m_games.find(game);
    return it == m_games.end() ? Records{} : it->second.rec;
}

void EffectStatus::clear(const std::string& game)
{
    std::lock_guard<std::mutex> lk(m_mx);
    if (m_games.erase(game)) m_dirty = true;
}

void EffectStatus::clearAll()
{
    std::lock_guard<std::mutex> lk(m_mx);
    if (!m_games.empty()) m_dirty = true;
    m_games.clear();
}

bool EffectStatus::load(const std::string& anchorPath)
{
    QFile f(statusPath(anchorPath));
    if (!f.open(QIODevice::ReadOnly)) return false;
    const QJsonObject root = QJsonDocument::fromJson(f.readAll()).object();
    std::lock_guard<std::mutex> lk(m_mx);
    m_games.clear();
    const QJsonObject games = root.value("games").toObject();
    for (auto it = games.begin(); it != games.end(); ++it)
    {
        const QJsonObject go = it.value().toObject();
        Game g;
        g.lastSeenMs = static_cast<int64_t>(go.value("lastSeen").toDouble(0.0));
        const QJsonObject eff = go.value("effects").toObject();
        for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
        {
            const QJsonObject eo = eff.value(haptics::effectInfo(i).key).toObject();
            EffectRecord& r = g.rec[static_cast<size_t>(i)];
            r.delivered  = eo.value("delivered").toBool(false);
            r.produced   = eo.value("produced").toBool(false);
            r.lastSeenMs = static_cast<int64_t>(eo.value("lastSeen").toDouble(0.0));
        }
        m_games[it.key().toStdString()] = g;
    }
    m_current = root.value("current").toString().toStdString();
    m_dirty = false;
    return true;
}

bool EffectStatus::save(const std::string& anchorPath) const
{
    QJsonObject root, games;
    {
        std::lock_guard<std::mutex> lk(m_mx);
        for (const auto& kv : m_games)
        {
            QJsonObject go, eff;
            go["lastSeen"] = static_cast<double>(kv.second.lastSeenMs);
            for (int i = 0; i < haptics::EFFECT_COUNT; ++i)
            {
                const EffectRecord& r = kv.second.rec[static_cast<size_t>(i)];
                if (!r.delivered && !r.produced) continue;
                QJsonObject eo;
                eo["delivered"] = r.delivered; eo["produced"] = r.produced;
                eo["lastSeen"]  = static_cast<double>(r.lastSeenMs);
                eff[haptics::effectInfo(i).key] = eo;
            }
            go["effects"] = eff;
            games[QString::fromStdString(kv.first)] = go;
        }
        root["games"]   = games;
        root["current"] = QString::fromStdString(m_current);
        root["_help"]   = "nullCAT remembers, per sim, which haptic effects it has ever received channels for and which have ever played. Not config: delete to forget.";
        m_dirty = false;
    }
    QSaveFile f(statusPath(anchorPath));
    if (!f.open(QIODevice::WriteOnly)) return false;
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return f.commit();
}
