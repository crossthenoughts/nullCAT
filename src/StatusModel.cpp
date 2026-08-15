// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// StatusModel.cpp - see StatusModel.h for the model contract.
//
// The derivation reproduces web/app.js byte-for-byte (the reference
// implementation) so the web is unchanged and the PC surface can match it.
// ============================================================
#include "StatusModel.h"
#include <cstdio>

namespace status {

// ---- colour/blink table (matches web/app.css .s-* + palette tokens) ----
const Style& styleOf(Indicator s)
{
    // light/dark hex are the --ok/--warn/--danger/--ink-soft tokens from app.css.
    static const Style kRunning { "RUNNING", "s-op",    "#2f7d4f", "#33c75a", "solid"     };
    static const Style kBusy    { "BUSY",    "s-home",  "#b07d18", "#ffb12e", "pulse"     };
    static const Style kIdle    { "IDLE",    "s-park",  "#6f685a", "#8b94a0", "solid"     };
    static const Style kOffline { "OFFLINE", "s-off",   "#6f685a", "#8b94a0", "solid-dim" };
    static const Style kFault   { "FAULT",   "s-fault", "#bb3322", "#ff4d3d", "blink"     };
    static const Style kEstop   { "ESTOP",   "s-estop", "#bb3322", "#ff4d3d", "pulse"     };
    switch (s)
    {
    case Indicator::RUNNING: return kRunning;
    case Indicator::BUSY:    return kBusy;
    case Indicator::IDLE:    return kIdle;
    case Indicator::FAULT:   return kFault;
    case Indicator::ESTOP:   return kEstop;
    case Indicator::OFFLINE:
    default:                 return kOffline;
    }
}

const char* indicatorToken(Indicator s)
{
    switch (s)
    {
    case Indicator::RUNNING: return "running";
    case Indicator::BUSY:    return "busy";
    case Indicator::IDLE:    return "idle";
    case Indicator::FAULT:   return "fault";
    case Indicator::ESTOP:   return "estop";
    case Indicator::OFFLINE:
    default:                 return "offline";
    }
}

namespace {

struct Electrical { Indicator state; const char* text; bool fault; };

// DS402 statusword (0x6041) decode - byte-identical to web app.js ds402().
// Decoding the RAW statusword (not DriveState) guarantees the web renders the
// same text/colour before and after consuming the model.
Electrical decodeElectrical(uint16_t sw)
{
    const uint16_t s = sw;
    if ((s & 0x4F) == 0x08 || (s & 0x4F) == 0x0F) return { Indicator::FAULT,   "FAULT",          true  };
    if ((s & 0x6F) == 0x07)                        return { Indicator::FAULT,   "QUICK STOP",     true  };
    if ((s & 0x6F) == 0x27)                        return { Indicator::RUNNING, "OP ENABLED",     false };
    if ((s & 0x6F) == 0x23)                        return { Indicator::BUSY,    "SWITCHED ON",    false };
    if ((s & 0x6F) == 0x21)                        return { Indicator::IDLE,    "READY",          false };
    if ((s & 0x4F) == 0x40)                        return { Indicator::OFFLINE, "SW-ON DISABLED", false };
    if ((s & 0x4F) == 0x00)                        return { Indicator::OFFLINE, "NOT READY",      false };
    return { Indicator::OFFLINE, nullptr, false };   // unknown -> caller formats hex
}

// "Meaningful" motion states = the web's outcomes for our axisStateName values
// under its /park|hom|oper|run|fault|estop|fatal/ regex. NOTE: the engine's E-STOP
// name is "E-STOP" -- with the hyphen, "e-stop" does NOT match /estop/ in app.js,
// so an ESTOPPING axis falls through to the ELECTRICAL path there (the QuickStop
// statusword -> "QUICK STOP"/s-fault). We match that exactly, so ESTOPPING is NOT
// meaningful here. ONLINE and BLENDING also fall through to electrical (their raw
// state is retained for later un-folding). Verified by the equivalence sweep in
// tests/TestStatusModel.cpp against a port of the app.js reference.
bool isMeaningfulMotion(AxisMotionState m)
{
    switch (m)
    {
    case AxisMotionState::HOMING:
    case AxisMotionState::PARKED:
    case AxisMotionState::PARKING:
    case AxisMotionState::UNPARKING:
        return true;
    default:                       // ONLINE, BLENDING, ESTOPPING -> electrical
        return false;
    }
}

Indicator motionBucket(AxisMotionState m)
{
    switch (m)
    {
    case AxisMotionState::HOMING:    return Indicator::BUSY;
    case AxisMotionState::PARKED:
    case AxisMotionState::PARKING:
    case AxisMotionState::UNPARKING: return Indicator::IDLE;
    default:                         return Indicator::RUNNING;  // not reached (guarded)
    }
}

} // namespace

AxisIndicator deriveAxis(int axisIndex,
                         AxisMotionState motion, const std::string& motionName,
                         bool hasStatusword, uint16_t statusword, DriveState rawDrive,
                         bool loopRunning, bool /*estop*/)
{
    AxisIndicator r;
    r.axisIndex  = axisIndex;
    r.rawMotion  = motion;
    r.rawDrive   = rawDrive;
    r.statusword = statusword;

    Electrical e = hasStatusword ? decodeElectrical(statusword)
                                 : Electrical{ Indicator::OFFLINE, "", false };

    // 1) electrical fault wins (only when we actually have electrical data)
    if (hasStatusword && e.fault)
    {
        r.state = Indicator::FAULT;
        r.text  = e.text ? e.text : "FAULT";
        r.fault = true;
        return r;
    }

    // 2) motion state, when the loop is running and the state is meaningful
    if (loopRunning && isMeaningfulMotion(motion))
    {
        r.state = motionBucket(motion);
        r.text  = motionName;
        r.fault = (r.state == Indicator::FAULT);
        return r;
    }

    // 3) electrical fallback. With no statusword this is OFFLINE with empty text
    // (the web shows "-" for it) -- NOT a fabricated "NOT READY".
    r.state = e.state;
    if (!hasStatusword)
    {
        r.text = "";        // no electrical data; renderer shows placeholder
    }
    else if (e.text)
    {
        r.text = e.text;
    }
    else
    {
        char buf[8];
        std::snprintf(buf, sizeof(buf), "0x%04X", statusword);
        r.text = buf;
    }
    r.fault = e.fault;
    return r;
}

Indicator deriveAggregate(const AxisIndicator* axes, int count,
                          bool loopRunning, bool estop)
{
    if (estop) return Indicator::ESTOP;
    for (int i = 0; i < count; ++i)
        if (axes[i].fault) return Indicator::FAULT;     // any axis faulted
    if (loopRunning) return Indicator::RUNNING;
    return Indicator::OFFLINE;
}


// ---- Rig-level boolean aggregates ------------------------------------------
// Single source for the booleans consumed by buildStatusJson, park-toggle and
// belts-toggle; quirks are deliberate. Do not "improve" here without
// updating TestStatusModel in the same commit.

MotionAggregates deriveMotionAggregates(const AxisMotionState* states, int numDrives)
{
    MotionAggregates a;
    a.allParked = (numDrives > 0);
    for (int i = 0; i < numDrives && i < MAX_DRIVES; ++i)
    {
        const AxisMotionState st = states[i];
        if (st == AxisMotionState::HOMING) a.anyHoming = true;
        if (st != AxisMotionState::PARKED) a.allParked = false;
        if (st == AxisMotionState::PARKING || st == AxisMotionState::UNPARKING ||
            st == AxisMotionState::HOMING  || st == AxisMotionState::ESTOPPING)
            a.transitional = true;
    }
    return a;
}

BeltAggregates deriveBeltAggregates(const AxisMotionState* states, int numStates,
                                    const bool* isTorque, int numConfig)
{
    BeltAggregates b;
    bool slack = true;
    for (int i = 0; i < numConfig && i < MAX_DRIVES; ++i)
    {
        if (!isTorque[i]) continue;
        b.hasBelts = true;
        if (i < numStates)
        {
            const AxisMotionState st = states[i];
            if (st != AxisMotionState::PARKED) slack = false;
            if (st == AxisMotionState::PARKING   || st == AxisMotionState::BLENDING ||
                st == AxisMotionState::UNPARKING || st == AxisMotionState::ESTOPPING)
                b.transitional = true;
        }
    }
    b.beltsSlack = b.hasBelts && slack;
    return b;
}

} // namespace status
