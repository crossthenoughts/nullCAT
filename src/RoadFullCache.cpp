// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#include "RoadFullCache.h"
#include "RoadFullLearner.h"   // the full mm's range
#include "Logging.h"

#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <cmath>
#include <cstddef>
#include <cstring>

using haptics::RoadFullEntry;

static QString cachePath(const std::string& anchorPath)
{
    return QFileInfo(QString::fromStdString(anchorPath)).absolutePath()
         + "/roadfull.json";
}

bool RoadFullCache::valid(const RoadFullEntry& e)
{
    using namespace haptics::road_k;
    return e.key != 0
        && std::isfinite(e.mm) && e.mm >= kMinFullMm && e.mm <= kFullLearnMaxMm
        && std::isfinite(e.sec) && e.sec > 0.0
        && std::isfinite(e.hpHz) && e.hpHz >= 0.0 && std::isfinite(e.suspHz) && e.suspHz >= 0.0;
}

bool RoadFullCache::load(const std::string& anchorPath)
{
    m_cars.clear();
    QFile f(cachePath(anchorPath));
    if (!f.open(QIODevice::ReadOnly)) return false;   // absent = nothing remembered, fine
    QJsonParseError pe;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject())
    {
        LOG_WARNING("RoadFullCache: roadfull.json unreadable -- starting empty.");
        return false;
    }
    int dropped = 0;
    for (const QJsonValue& v : doc.object().value("cars").toArray())
    {
        const QJsonObject o = v.toObject();
        RoadFullEntry e;
        bool ok = false;
        e.key    = o.value("key").toString().toULongLong(&ok, 16);
        e.mm     = o.value("mm").toDouble(0.0);
        e.sec    = o.value("sec").toDouble(0.0);
        e.hpHz   = o.value("hpHz").toDouble(-1.0);
        e.suspHz = o.value("suspHz").toDouble(-1.0);
        const QByteArray name = o.value("car").toString().toUtf8();
        std::strncpy(e.car, name.constData(), haptics::kRoadFullCarLen - 1);
        e.car[haptics::kRoadFullCarLen - 1] = '\0';
        bool twice = false;
        for (const RoadFullEntry& have : m_cars) twice = twice || have.key == e.key;
        if (!ok || !valid(e) || twice) { ++dropped; continue; }
        if ((int)m_cars.size() >= haptics::RoadFullMemory::kCars) break;
        m_cars.push_back(e);
    }
    if (!m_cars.empty())
        LOG_INFO(strf("RoadFullCache: the road's full mm remembered for %d car(s).", (int)m_cars.size()));
    if (dropped > 0)
        LOG_INFO(strf("RoadFullCache: %d entry(ies) that could not be read left out.", dropped));
    return true;
}

bool RoadFullCache::save(const std::string& anchorPath) const
{
    QJsonArray cars;
    for (const RoadFullEntry& e : m_cars)
    {
        QJsonObject o;
        o["key"]    = QString::number(static_cast<qulonglong>(e.key), 16).rightJustified(16, QLatin1Char('0'));
        o["car"]    = QString::fromUtf8(e.car);
        o["mm"]     = std::round(e.mm * 100.0) / 100.0;
        o["sec"]    = std::round(e.sec);
        o["hpHz"]   = e.hpHz;
        o["suspHz"] = e.suspHz;
        cars.append(o);
    }
    QJsonObject root;
    root["_documentation"] =
        "The Road tile's full mm learned for each car (learn full mm): its "
        "value, the seconds of driving behind it and the cut hz and susp cut "
        "hz it was learned under. Machine state, not configuration: safe to "
        "delete, never edited by hand.";
    root["cars"] = cars;

    // Written while running (each finished car), so a box powered off
    // mid-save is expected: a temporary file renamed over the old one
    // leaves the old file whole.
    QSaveFile f(cachePath(anchorPath));
    if (!f.open(QIODevice::WriteOnly)
        || f.write(QJsonDocument(root).toJson(QJsonDocument::Indented)) < 0
        || !f.commit())
    {
        LOG_WARNING("RoadFullCache: could not write roadfull.json.");
        return false;
    }
    return true;
}

bool RoadFullCache::takeSession(const RoadFullEntry& e)
{
    const bool changed = merge(e);
    if (changed) m_unsaved = true;
    return changed;
}

bool RoadFullCache::takeFinished(uint32_t seq, const RoadFullEntry& e)
{
    if (seq == m_finishedSeq) return false;   // this car is already in (or there was never one)
    m_finishedSeq = seq;
    return takeSession(e);
}

bool RoadFullCache::poll(double nowSec, uint32_t finishedSeq, const RoadFullEntry& finished, const RoadFullEntry& current)
{
    bool changed = takeFinished(finishedSeq, finished);
    if (!m_polled) { m_polled = true; m_nextSessionSec = nowSec + kSessionEverySec; }
    else if (nowSec >= m_nextSessionSec)
    {
        m_nextSessionSec = nowSec + kSessionEverySec;
        changed = takeSession(current) || changed;
    }
    return changed;
}

bool RoadFullCache::flush(const std::string& anchorPath, double nowSec, bool force)
{
    if (!m_unsaved) return false;
    if (!force && m_wrote && nowSec < m_nextWriteSec) return false;
    m_wrote = true; m_nextWriteSec = nowSec + kWriteEverySec;   // written or not
    if (!save(anchorPath)) return false;
    m_unsaved = false;
    return true;
}

bool RoadFullCache::merge(const RoadFullEntry& in)
{
    if (!valid(in)) return false;
    RoadFullEntry e = in;
    e.car[haptics::kRoadFullCarLen - 1] = '\0';
    for (size_t i = 0; i < m_cars.size(); ++i)
    {
        if (m_cars[i].key != e.key) continue;
        // The same to what the file keeps (mm to 0.01, whole seconds): no
        // write for a car that was only sat in.
        const RoadFullEntry& have = m_cars[i];
        const bool same = std::fabs(have.mm - e.mm) < 0.005 && std::fabs(have.sec - e.sec) < 0.5
                       && haptics::roadFullSameCuts(have, e.hpHz, e.suspHz);
        if (same && i == 0) return false;
        m_cars.erase(m_cars.begin() + static_cast<std::ptrdiff_t>(i));
        m_cars.insert(m_cars.begin(), e);                 // most recently driven first
        return true;
    }
    m_cars.insert(m_cars.begin(), e);
    while ((int)m_cars.size() > haptics::RoadFullMemory::kCars) m_cars.pop_back();
    return true;
}
