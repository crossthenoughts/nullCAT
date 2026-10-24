// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// SeatPreflight.h - one frame of the de-init seat pass's bus check.
//
// The seat pass presses the vertical axes onto their stops before de-init
// de-energises them, so it first checks the bus a frame at a time:
//   Good    every slave did its work on the frame (the expected working
//           counter);
//   Partial the frame came back but only some slaves worked on it (a drive
//           powered off or out of OP, the ring returning from the last
//           slave it reaches): the pass can still seat the axes whose
//           slaves are in OP;
//   Dead    no reply in time, or a reply no slave worked on.
// Takes the RAW working counter of the exchange (EtherCATMaster::
// lastRawWkc(), -1 = no reply): sendReceive() itself returns -1 for every
// mismatch, which made a partial bus read as a dead one and skipped the
// whole seat. No SOEM types, so the rule is unit tested on its own.
// ============================================================

enum class SeatProbe { Good, Partial, Dead };

inline SeatProbe classifySeatProbe(int rawWkc, int expectedWkc)
{
    if (expectedWkc > 0 && rawWkc == expectedWkc) return SeatProbe::Good;
    if (rawWkc > 0) return SeatProbe::Partial;   // above expected counts here too: not a clean frame
    return SeatProbe::Dead;
}

// The pre-flight's verdict from its counts. A healthy bus stops at minGood
// good frames; a partial bus needs as many partial frames in the window.
// (Not "more partial than dead": one Windows delivery pause can fill half
// the window, and the per-slave state read decides which axes seat anyway.)
enum class SeatPlan { Seat, Partial, Skip };

inline SeatPlan seatPlanFor(int good, int partial, int minGood)
{
    if (good >= minGood)    return SeatPlan::Seat;
    if (partial >= minGood) return SeatPlan::Partial;
    return SeatPlan::Skip;
}

// The pre-flight's running tally, one probe at a time.
struct SeatTally
{
    int good = 0, partial = 0, dead = 0, probes = 0;
    int lastPartial = 0;   // the working counter of the latest partial frame

    void add(int rawWkc, int expectedWkc)
    {
        ++probes;
        switch (classifySeatProbe(rawWkc, expectedWkc))
        {
            case SeatProbe::Good:    ++good; break;
            case SeatProbe::Partial: ++partial; lastPartial = rawWkc; break;
            case SeatProbe::Dead:    ++dead; break;
        }
    }

    // The floor seatFrameOk holds a partial seat to: the level the bus showed
    // last before the slave states are read (an earlier glitch must not
    // lower it), never above a full frame. 0 = no partial frame seen.
    int partialFloor(int expectedWkc) const
    {
        return (lastPartial < expectedWkc) ? lastPartial : expectedWkc;
    }
};

// During the press, relief and dwell: is the bus still as the pre-flight
// found it? On a healthy bus (partialFloor 0) every slave must work on the
// frame, as before. On a partial bus, the frame must carry at least the
// work the pre-flight saw (SeatTally::partialFloor): every frame there
// misses the slaves that are gone, so only a frame showing the bus getting
// worse counts towards the abort.
inline bool seatFrameOk(int rawWkc, int expectedWkc, int partialFloor)
{
    if (partialFloor <= 0) return classifySeatProbe(rawWkc, expectedWkc) == SeatProbe::Good;
    return rawWkc >= partialFloor;
}
