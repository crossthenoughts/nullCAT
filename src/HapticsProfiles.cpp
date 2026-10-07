// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#include "HapticsProfiles.h"
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QSaveFile>

namespace {
QString profilesPath(const std::string& anchorPath)
{
    return QFileInfo(QString::fromStdString(anchorPath)).absolutePath() + "/profiles.json";
}
}

bool HapticsProfiles::load(const std::string& anchorPath)
{
    QFile f(profilesPath(anchorPath));
    if (!f.open(QIODevice::ReadOnly)) return false;
    const QJsonObject root = QJsonDocument::fromJson(f.readAll()).object();
    std::lock_guard<std::mutex> lk(m_mx);
    m_profiles.clear(); m_bindings.clear();
    const QJsonObject ps = root.value("profiles").toObject();
    for (auto it = ps.begin(); it != ps.end(); ++it)
        if (it.value().isObject()) m_profiles[it.key().toStdString()] = it.value().toObject();
    const QJsonObject bs = root.value("bindings").toObject();
    for (auto it = bs.begin(); it != bs.end(); ++it)
        if (it.value().isString() && m_profiles.count(it.value().toString().toStdString()))
            m_bindings[it.key().toStdString()] = it.value().toString().toStdString();
    m_active = root.value("active").toString().toStdString();
    if (!m_profiles.count(m_active)) m_active.clear();
    return true;
}

bool HapticsProfiles::save(const std::string& anchorPath) const
{
    QJsonObject root, ps, bs;
    {
        std::lock_guard<std::mutex> lk(m_mx);
        for (const auto& kv : m_profiles) ps[QString::fromStdString(kv.first)] = kv.second;
        for (const auto& kv : m_bindings) bs[QString::fromStdString(kv.first)] = QString::fromStdString(kv.second);
        root["active"] = QString::fromStdString(m_active);
    }
    root["profiles"] = ps;
    root["bindings"] = bs;
    root["_help"] = "nullCAT haptics profiles: named copies of the rig's haptics settings, and which car or game each one is for. Edited from the Haptics strip.";
    QSaveFile f(profilesPath(anchorPath));
    if (!f.open(QIODevice::WriteOnly)) return false;
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return f.commit();
}

std::vector<std::string> HapticsProfiles::names() const
{
    std::lock_guard<std::mutex> lk(m_mx);
    std::vector<std::string> out;
    for (const auto& kv : m_profiles) out.push_back(kv.first);
    return out;
}

bool HapticsProfiles::has(const std::string& name) const
{
    std::lock_guard<std::mutex> lk(m_mx);
    return m_profiles.count(name) > 0;
}

QJsonObject HapticsProfiles::get(const std::string& name) const
{
    std::lock_guard<std::mutex> lk(m_mx);
    const auto it = m_profiles.find(name);
    return it == m_profiles.end() ? QJsonObject() : it->second;
}

void HapticsProfiles::put(const std::string& name, const QJsonObject& haptics)
{
    std::lock_guard<std::mutex> lk(m_mx);
    m_profiles[name] = haptics;
}

bool HapticsProfiles::remove(const std::string& name)
{
    std::lock_guard<std::mutex> lk(m_mx);
    if (!m_profiles.erase(name)) return false;
    for (auto it = m_bindings.begin(); it != m_bindings.end();)
        it = (it->second == name) ? m_bindings.erase(it) : std::next(it);
    if (m_active == name) m_active.clear();
    return true;
}

std::string HapticsProfiles::active() const
{
    std::lock_guard<std::mutex> lk(m_mx);
    return m_active;
}

void HapticsProfiles::setActive(const std::string& name)
{
    std::lock_guard<std::mutex> lk(m_mx);
    m_active = m_profiles.count(name) ? name : std::string();
}

void HapticsProfiles::bind(const std::string& key, const std::string& profile)
{
    std::lock_guard<std::mutex> lk(m_mx);
    if (!m_profiles.count(profile) || key.size() < 5) return;
    m_bindings[key] = profile;
}

void HapticsProfiles::unbind(const std::string& key)
{
    std::lock_guard<std::mutex> lk(m_mx);
    m_bindings.erase(key);
}

std::map<std::string, std::string> HapticsProfiles::bindings() const
{
    std::lock_guard<std::mutex> lk(m_mx);
    return m_bindings;
}

std::string HapticsProfiles::profileFor(const std::string& car, const std::string& game) const
{
    std::lock_guard<std::mutex> lk(m_mx);
    if (!car.empty())
    {
        const auto it = m_bindings.find(carKey(car));
        if (it != m_bindings.end()) return it->second;
    }
    if (!game.empty())
    {
        const auto it = m_bindings.find(gameKey(game));
        if (it != m_bindings.end()) return it->second;
    }
    return std::string();
}
