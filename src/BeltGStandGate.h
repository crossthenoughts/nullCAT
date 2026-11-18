// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// BeltGStandGate: the belt from g takes no g from a car that is standing
// with the throttle shut.
//
// A game loading a map or putting a car down sends g no car makes while
// the car stands: AC EVO drops it onto its wheels and then sends single
// lines of -2.5 g surge (twice in 8 s, engine off), BeamNG 1.6 s of noise
// to -9 g after a new car, AMS2 -38 g and LMU -1.1 g right after a load
// (recordings, 2026-10 and 2026-11). Each was a pull into the shoulders:
// the car stands, so no speed jump holds the g off (the teleport rule),
// under 10 km/h the fade after a gap does not arm (a launch must be felt),
// and most are under the 8 g junk limit. A wrecked car's noise at a
// standstill is the same case.
//
// A standing car's g is real only when it pulls away, and then the driver
// is on the throttle. So: while STANDING and the throttle is at or under
// kThrottlePct, the g is left out (the belt sits at minimum, ONLINE).
//   Standing: the speed under kStandKmh for kStandSec; or the first speed
//   after the stream starts again (a load, a pause, a reset, tension-up)
//   under kGoKmh. Left at kGoKmh.
//   The throttle opens it at once: a launch pulls with no delay.
//   The brake never opens it (AC EVO holds the brake at 100 % in the
//   garage). A sender that sends no throttle or no speed: never closed.
// Lost with it: off the throttle, a bump, a shunt or braking from a crawl,
// until the car has passed kGoKmh again. A car that passes kGoKmh with the
// throttle still shut (rolling, or driven by the game) gets its g back as
// it is at that moment, with no fade.
// What it leans on: the speed is the car's own over the ground (the
// plugin and the BeamNG mod send that); a sender giving the wheels' speed
// would read a long lock-up under braking as standing. And a throttle
// that is sent but stuck at 0 loses a launch's g up to kGoKmh (the log
// line beltDemand writes for a stand says so).
//
// Pure, RT-safe, no allocation. Owner: MotionController::beltDemand.
// ============================================================
#include "BeltGFadeIn.h"

#include <cmath>

struct BeltGStandGate
{
    static constexpr double kStandKmh    = 3.0;    // under this...
    static constexpr double kStandSec    = 0.3;    // ...for this long: standing
    // At or over this: driving. The fade's own line (BeltGFadeIn): under it
    // a g coming back is the gate's to judge, at or over it the fade's.
    static constexpr double kGoKmh       = BeltGFadeIn::kMinKmh;
    static constexpr double kThrottlePct = 5.0;    // over this the driver is pulling away: the g counts

    bool   standing = false;
    double slowSec  = 0.0;     // how long the speed has stayed under kStandKmh
    bool   fresh    = true;    // the stream has just (re)started: the first speed decides

    // The stream stopped or the car was put somewhere else (not fresh, a
    // teleport's hold-off, tension-up): the next speed decides afresh.
    void restart() { fresh = true; slowSec = 0.0; }

    // One cycle with the latest speed and throttle (each: is it sent, its
    // value). True while the g is to be left out.
    bool step(bool haveKmh, double kmh, bool haveThrottle, double throttlePct, double dt)
    {
        // No speed: unknown, never closed; when it is back, the first decides.
        if (!haveKmh || !std::isfinite(kmh)) { standing = false; slowSec = 0.0; fresh = true; return false; }
        const double v = std::fabs(kmh);
        if (fresh) { fresh = false; standing = v < kGoKmh; slowSec = 0.0; }
        if (v >= kGoKmh)        { standing = false; slowSec = 0.0; }
        else if (v < kStandKmh) { slowSec += dt; if (slowSec >= kStandSec) standing = true; }
        else                    slowSec = 0.0;
        return standing && haveThrottle && std::isfinite(throttlePct) && throttlePct <= kThrottlePct;
    }
};
