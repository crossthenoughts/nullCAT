// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#include "CarCache.h"
#include "Logging.h"

#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <cmath>

static QString cachePath(const std::string& anchorPath)
{
    return QFileInfo(QString::fromStdString(anchorPath)).absolutePath()
         + "/carcache.json";
}

bool CarCache::load(const std::string& anchorPath)
{
    m_cars.clear();
    QFile f(cachePath(anchorPath));
    if (!f.open(QIODevice::ReadOnly)) return false;   // absent = empty cache, fine
    QJsonParseError pe;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject())
    {
        LOG_WARNING("CarCache: carcache.json unreadable -- starting empty.");
        return false;
    }
    int dropped = 0;
    for (const QJsonValue& v : doc.object().value("cars").toArray())
    {
        const QJsonObject o = v.toObject();
        const QJsonArray ra = o.value("ratios").toArray();
        CachedCar car;
        bool any = false;
        for (int g = 1; g < MAX_GEARS && g < ra.size(); ++g)
        {
            const double r = ra[g].toDouble(0.0);
            if (r > 0.0) { car.r[g] = r; car.has[g] = true; any = true; }
        }
        // A set that does not fall with every gear holds a wrong value
        // (learned from a launch or wheelspin before the learner was
        // timed in seconds): forgotten, the car is relearned.
        if (any && !GearRatioLearner::inOrder(car.r, car.has)) { ++dropped; continue; }
        if (any) m_cars.push_back(car);
        if ((int)m_cars.size() >= MAX_CACHED_CARS) break;
    }
    if (!m_cars.empty())
        LOG_INFO(strf("CarCache: %d remembered car(s) loaded.", (int)m_cars.size()));
    if (dropped > 0)
        LOG_INFO(strf("CarCache: %d remembered car(s) with ratios out of gear order forgotten.", dropped));
    return true;
}

bool CarCache::save(const std::string& anchorPath) const
{
    QJsonArray cars;
    for (const CachedCar& car : m_cars)
    {
        QJsonArray ra;
        for (int g = 0; g < MAX_GEARS; ++g)
            ra.append(car.has[g] ? car.r[g] : 0.0);
        QJsonObject o;
        o["ratios"] = ra;
        cars.append(o);
    }
    QJsonObject root;
    root["_documentation"] =
        "Learned per-car gear ratios (rpm per km/h, index = gear). Machine "
        "state, not configuration: safe to delete, never edited by hand.";
    root["cars"] = cars;

    // Written while running (each finished car), so a box powered off
    // mid-save is expected: a temporary file renamed over the old one
    // leaves the old file whole.
    QSaveFile f(cachePath(anchorPath));
    if (!f.open(QIODevice::WriteOnly)
        || f.write(QJsonDocument(root).toJson(QJsonDocument::Indented)) < 0
        || !f.commit())
    {
        LOG_WARNING("CarCache: could not write carcache.json.");
        return false;
    }
    return true;
}

bool CarCache::takeSession(const double r[MAX_GEARS], const bool confident[MAX_GEARS], bool dirty)
{
    if (!dirty) return false;   // nothing the cache did not already know
    const bool changed = merge(r, confident);
    if (changed) m_unsaved = true;
    return changed;
}

bool CarCache::flush(const std::string& anchorPath, bool force)
{
    if (!m_unsaved) return false;
    if (!force && m_retryWait > 0) { --m_retryWait; return false; }
    if (!save(anchorPath)) { m_retryWait = 40; return false; }   // ~10 s of 250 ms polls
    m_unsaved = false; m_retryWait = 0;
    return true;
}

bool CarCache::takeFinished(uint32_t seq, const double r[MAX_GEARS],
                            const bool confident[MAX_GEARS], bool dirty)
{
    if (seq == m_finishedSeq) return false;   // this set is already in (or there was never one)
    m_finishedSeq = seq;
    return takeSession(r, confident, dirty);
}

bool CarCache::merge(const double r[MAX_GEARS], const bool confident[MAX_GEARS])
{
    int n = 0;
    for (int g = 1; g < MAX_GEARS; ++g) if (confident[g]) ++n;
    if (n < GearRatioLearner::ADOPT_MIN_GEARS) return false;   // too little to identify a car

    // Find the remembered car this session matches (same rule as adoption:
    // every shared gear within tolerance, at least two shared).
    int match = -1;
    for (size_t c = 0; c < m_cars.size() && match < 0; ++c)
    {
        const CachedCar& car = m_cars[c];
        bool ok = true; int shared = 0;
        for (int g = 1; g < MAX_GEARS && ok; ++g)
        {
            if (!confident[g] || !car.has[g]) continue;
            ++shared;
            if (std::fabs(r[g] - car.r[g]) > GearRatioLearner::MATCH_TOL * car.r[g])
                ok = false;
        }
        if (ok && shared >= GearRatioLearner::ADOPT_MIN_GEARS) match = (int)c;
    }

    {
        // The session's own gears must fall with every gear.
        bool has[MAX_GEARS] = {};
        for (int g = 1; g < MAX_GEARS; ++g) has[g] = confident[g];
        if (!GearRatioLearner::inOrder(r, has)) return false;
    }
    CachedCar car = (match >= 0) ? m_cars[match] : CachedCar{};
    for (int g = 1; g < MAX_GEARS; ++g)
        if (confident[g]) { car.r[g] = r[g]; car.has[g] = true; }
    // A remembered gear the session contradicts (out of order with what
    // was just driven) goes; the session's gears stand.
    for (int g = 1; g < MAX_GEARS; ++g)
    {
        if (confident[g] || !car.has[g]) continue;
        for (int h = 1; h < MAX_GEARS; ++h)
        {
            if (!confident[h]) continue;
            if ((h < g && !(r[h] > car.r[g])) || (h > g && !(r[h] < car.r[g]))) { car.has[g] = false; car.r[g] = 0.0; break; }
        }
    }

    if (match >= 0) m_cars.erase(m_cars.begin() + match);
    m_cars.insert(m_cars.begin(), car);                  // most recently driven first
    while ((int)m_cars.size() > MAX_CACHED_CARS) m_cars.pop_back();
    return true;
}
