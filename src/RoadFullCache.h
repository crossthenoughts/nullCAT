// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// RoadFullCache - persistence for the Road tile's learned full mm per car
// (roadfull.json, beside host.json/rig.json). NOT config: it is learned
// machine state, so it is never merged, validated against a schema or
// edited - just remembered. The same shape as CarCache (the gear ratios).
//
// Runs on the MAIN thread only: loaded once at startup (fed to
// MotionController::setRoadFullCache before the RT loop runs) and fed
// from the published snapshot (MotionStatus roadFull*): a car the sim
// moved away from as the mains poll, the car being driven now and then
// and at shutdown. An entry is one car (RoadFullMemory.h: its key, its
// full mm, the seconds of driving behind it, the cuts it was learned
// under, its name); the most recently driven first,
// RoadFullMemory::kCars of them at most.
//
// Its two waits are in seconds on the caller's clock (any monotonic one),
// not in calls: the mains happen to poll every 250 ms.
// ============================================================

#include "RoadFullMemory.h"
#include <cstdint>
#include <string>
#include <vector>

class RoadFullCache
{
public:
    // The car being driven is taken this often by poll(): a Pi is powered
    // off, not stopped, and one car is often driven all evening, so waiting
    // for a car change or the exit would never write it.
    static constexpr double kSessionEverySec = 120.0;
    // The file is written this often at most (forced writes aside),
    // whatever is fed in (two senders naming different cars in turn), and a
    // failed write is tried again after as long, so a file held by another
    // program does not flood the log.
    static constexpr double kWriteEverySec = 10.0;

    // anchorPath: the same config anchor the mains hand to Config::load.
    bool load(const std::string& anchorPath);
    bool save(const std::string& anchorPath) const;

    // From the published status; each returns true when the cache changed
    // (flush() writes it).
    //   takeSession:  the car being driven (MotionStatus roadFullCar): at
    //                 shutdown, when the snapshot is final.
    //   takeFinished: the car the sim moved away from (roadFullPrev),
    //                 published with a count that steps once per finished
    //                 car: merged once per step.
    //   poll:         the mains' periodic call: the finished car at once,
    //                 the car being driven every kSessionEverySec.
    bool takeSession(const haptics::RoadFullEntry& e);
    bool takeFinished(uint32_t seq, const haptics::RoadFullEntry& e);
    bool poll(double nowSec, uint32_t finishedSeq, const haptics::RoadFullEntry& finished, const haptics::RoadFullEntry& current);
    // Write what the takes changed, kWriteEverySec apart at most; a failed
    // write stays pending. force (shutdown) always tries. True when it wrote.
    bool flush(const std::string& anchorPath, double nowSec, bool force = false);
    bool unsaved() const { return m_unsaved; }

    // One car in: true when it is new or differs from what was kept.
    bool merge(const haptics::RoadFullEntry& e);

    const std::vector<haptics::RoadFullEntry>& cars() const { return m_cars; }

    // An entry worth keeping: a car, a full mm inside the field's range,
    // some driving behind it, cuts that are numbers.
    static bool valid(const haptics::RoadFullEntry& e);

private:
    std::vector<haptics::RoadFullEntry> m_cars;   // front = most recently driven
    uint32_t m_finishedSeq = 0;      // the last roadFullPrevSeq taken
    bool     m_unsaved     = false;  // a take changed the cache and no write has landed since
    bool     m_polled      = false;  // poll() has run: its clock has started
    double   m_nextSessionSec = 0.0; // when poll() next takes the car being driven
    double   m_nextWriteSec   = 0.0; // when flush() may next write
    bool     m_wrote       = false;  // flush() has tried a write: the wait above counts
};
