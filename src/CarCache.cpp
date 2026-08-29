// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#include "CarCache.h"
#include "Logging.h"

#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
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
        if (any) m_cars.push_back(car);
        if ((int)m_cars.size() >= MAX_CACHED_CARS) break;
    }
    if (!m_cars.empty())
        LOG_INFO(strf("CarCache: %d remembered car(s) loaded.", (int)m_cars.size()));
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

    QFile f(cachePath(anchorPath));
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        LOG_WARNING("CarCache: could not write carcache.json.");
        return false;
    }
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return true;
}

void CarCache::merge(const double r[MAX_GEARS], const bool confident[MAX_GEARS])
{
    int n = 0;
    for (int g = 1; g < MAX_GEARS; ++g) if (confident[g]) ++n;
    if (n < GearRatioLearner::ADOPT_MIN_GEARS) return;   // too little to identify a car

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

    CachedCar car = (match >= 0) ? m_cars[match] : CachedCar{};
    for (int g = 1; g < MAX_GEARS; ++g)
        if (confident[g]) { car.r[g] = r[g]; car.has[g] = true; }

    if (match >= 0) m_cars.erase(m_cars.begin() + match);
    m_cars.insert(m_cars.begin(), car);                  // most recently driven first
    while ((int)m_cars.size() > MAX_CACHED_CARS) m_cars.pop_back();
}
