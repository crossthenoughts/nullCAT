// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// RoadFullMemory - the Road tile's learned full mm, kept per car.
//
// RoadFullLearner works a car's full mm out of its driving; this is what
// carries it from one drive of the car to the next. A fixed table of cars
// (CarKey.h: the game, the car's name and its id as one key), each with its
// own full mm, the seconds of driving behind it and the cuts it was learned
// under (its millimetres are after them: under other cuts it says nothing).
//
// Owned by the haptics clock while the loop runs, exactly as the gear
// ratio learner's cache is: load() from the main thread at start-up, before
// the loop ever runs; step() once a cycle after the laws. When the car
// named changes, the car left behind is already in the table (it is
// written every cycle, so nothing depends on who notices the change
// first), it is published as finished (finished(), finishedSeq(): the main
// thread's RoadFullCache writes it to roadfull.json), and the new car's
// entry, if it has one under the cuts in use, is handed back for the
// learner to start from. Going back to a car within one session finds what
// this session learned.
//
// The learner belongs to one car under one pair of cuts at a time: on
// every change of key, and of the cuts, it starts over (changed), from what
// was remembered or from nothing. The laws start it over for another name
// or id themselves, and the layer for new cuts; the same name in another
// game is another car here too. A change of cuts keeps what was remembered
// until the new cuts have learned something of their own, so cuts changed
// and put back find the car's value again.
//
// RT-safe: fixed arrays, no allocation; a scan of kCars entries on a
// change, one entry written per cycle.
// ============================================================

#include "CarKey.h"
#include <cmath>
#include <cstdint>
#include <cstring>

namespace haptics {

static constexpr int kRoadFullCarLen = kCarKeyStrLen;   // the car's name

struct RoadFullEntry
{
    uint64_t key  = 0;                    // the car (carIdentityKey); 0 = no car
    double   mm   = 0.0, sec = 0.0;       // its own full mm, and the seconds of driving behind it
    double   hpHz = 0.0, suspHz = 0.0;    // the cuts it was learned under
    char     car[kRoadFullCarLen] = {};   // the car's name, for whoever reads the file
};

// The cuts of an entry are the cuts in use.
inline bool roadFullSameCuts(const RoadFullEntry& e, double hpHz, double suspHz)
{
    return std::fabs(e.hpHz - hpHz) < 1e-6 && std::fabs(e.suspHz - suspHz) < 1e-6;
}

class RoadFullMemory
{
public:
    static constexpr int kCars = 64;

    // Start-up, before the loop runs: the remembered cars, most recently
    // driven first (more than kCars: the rest are left out).
    void load(const RoadFullEntry* e, int n)
    {
        for (Slot& s : m_slot) s = Slot{};
        m_clock = 0; m_key = 0; m_idx = -1; m_cur = RoadFullEntry{};
        for (int i = 0; i < n && i < kCars; ++i)
        {
            if (e[i].key == 0 || !(e[i].mm > 0.0) || !(e[i].sec > 0.0)) continue;
            m_slot[i].e = e[i];
            m_slot[i].used = static_cast<uint32_t>(n - i);   // the first given is the last to be dropped
        }
        m_clock = static_cast<uint32_t>(n > 0 ? n : 0);
    }

    // changed: another car is named than last cycle, or the cuts are others
    // (the learner starts over). any: it is remembered under the cuts in
    // use (mm, sec).
    struct Seed { bool changed = false, any = false; double mm = 0.0, sec = 0.0; };

    // Once a cycle, after the laws. key: the car named now (0 = none yet),
    // car: its name. carMm / carSec: the learner's own evidence for it now
    // (0 = nothing yet). hpHz / suspHz: the cuts in use. Returns whether
    // the car or the cuts have just changed, and what to start the learner
    // from when the car is remembered under these cuts.
    Seed step(uint64_t key, const char* car, double carMm, double carSec, double hpHz, double suspHz)
    {
        Seed seed;
        const bool otherCar  = key != m_key;
        const bool otherCuts = !otherCar && key != 0 && !(std::fabs(hpHz - m_hpHz) < 1e-6 && std::fabs(suspHz - m_suspHz) < 1e-6);
        m_hpHz = hpHz; m_suspHz = suspHz;
        if (otherCar || otherCuts)
        {
            // The car left behind: in the table as of the last cycle.
            if (otherCar && m_key != 0 && m_cur.mm > 0.0) { m_finished = m_cur; ++m_finishedSeq; }
            seed.changed = true;
            m_key = key; m_idx = -1;
            m_cur = RoadFullEntry{}; m_cur.key = key;
            if (car) { std::strncpy(m_cur.car, car, kRoadFullCarLen - 1); m_cur.car[kRoadFullCarLen - 1] = '\0'; }
            if (key != 0)
            {
                if (otherCar) ++m_clock;
                for (int i = 0; i < kCars; ++i)
                    if (m_slot[i].e.key == key) { m_idx = i; m_slot[i].used = m_clock; break; }
                if (m_idx >= 0 && m_slot[m_idx].e.mm > 0.0 && roadFullSameCuts(m_slot[m_idx].e, hpHz, suspHz))
                {
                    seed.any = true; seed.mm = m_slot[m_idx].e.mm; seed.sec = m_slot[m_idx].e.sec;
                    m_cur.mm = seed.mm; m_cur.sec = seed.sec; m_cur.hpHz = hpHz; m_cur.suspHz = suspHz;
                }
            }
            return seed;   // the learner has the car's value from the next cycle
        }
        if (m_key == 0 || !(carMm > 0.0) || !std::isfinite(carMm) || !std::isfinite(carSec)) return seed;
        m_cur.mm = carMm; m_cur.sec = carSec; m_cur.hpHz = hpHz; m_cur.suspHz = suspHz;
        if (m_idx < 0)
        {
            // A car not in the table yet: an empty slot, else the one
            // longest unused.
            int pick = 0;
            for (int i = 0; i < kCars; ++i)
            {
                if (m_slot[i].e.key == 0) { pick = i; break; }
                if (m_slot[i].used < m_slot[pick].used) pick = i;
            }
            m_idx = pick; m_slot[pick].used = m_clock;
        }
        m_slot[m_idx].e = m_cur;
        return seed;
    }

    // For the main thread (through the status snapshot): the car being
    // driven, and the last car left behind with a count that steps once per
    // car, so each is written once.
    const RoadFullEntry& current() const  { return m_cur; }
    const RoadFullEntry& finished() const { return m_finished; }
    uint32_t finishedSeq() const          { return m_finishedSeq; }

    // The entry remembered for a car (tests): null when it has none.
    const RoadFullEntry* find(uint64_t key) const
    {
        for (const Slot& s : m_slot) if (key != 0 && s.e.key == key) return &s.e;
        return nullptr;
    }

private:
    struct Slot { RoadFullEntry e; uint32_t used = 0; };
    Slot          m_slot[kCars];
    uint32_t      m_clock = 0;            // steps on every car change: the larger, the more recently driven
    uint64_t      m_key = 0;              // the car being driven
    double        m_hpHz = 0.0, m_suspHz = 0.0;   // the cuts in use last cycle
    int           m_idx = -1;             // ...its slot (-1 = not in the table yet)
    RoadFullEntry m_cur, m_finished;
    uint32_t      m_finishedSeq = 0;
};

} // namespace haptics
