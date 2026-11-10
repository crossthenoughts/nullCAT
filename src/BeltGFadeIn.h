// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// BeltGFadeIn: the belt from g easing back in after its g was away.
//
// When the g comes back (the game unpaused, the end of a teleport's
// hold-off, a sender that held every g still moving again) its first value
// used to land at once: the belt's slew cap moves min to max in about 2 ms,
// so a resume mid-corner or under braking was a full pull into the
// shoulders, and AMS2's rolling start (the car put on track at 50 km/h
// already accelerating) a jerk. Once armed, the tension crossfades from
// where the belt was (held through a short pause, at min after a long one
// or a hold-off) to the live demand over kSec, the way the enable blend
// eases a belt in, whatever the size of the step. A demand under the belt's
// last tension ends it at once: easing off is never slowed, and a pause
// that resumes as it stopped has next to nothing to fade. A demand equal to
// it does not (a cycle with no usable g holds the last tension, as one junk
// frame or a torn snapshot does: the fade carries on through it).
//
// Not armed (the owner, MotionController::beltDemand, decides the gap; this
// decides the speed): a car under kMinKmh when its g comes back is pulling
// away, felt at once. Pure, RT-safe, no allocation.
// ============================================================
#include <algorithm>
#include <cmath>

struct BeltGFadeIn
{
    static constexpr double kSec    = 0.4;    // the crossfade's length
    static constexpr double kMinKmh = 10.0;   // under this the car is pulling away: no fade

    bool   on    = false;
    double from  = 0.0;   // the tension the belt had when the g came back
    double sec   = 0.0;   // how far into the crossfade

    // The g came back after a gap, the belt at fromTension: fade, unless
    // the car stands. A speed the sender does not send fades (the safe side).
    void arm(bool haveSpeed, double kmh, double fromTension)
    {
        if (haveSpeed && std::isfinite(kmh) && std::fabs(kmh) < kMinKmh) return;
        on = true; from = fromTension; sec = 0.0;
    }
    void clear() { on = false; }

    // The tension this cycle for a demand: crossfaded in from `from` while
    // on; over once the crossfade is done, or the demand is under the last
    // tension (easing off is never slowed; not a number ends it too).
    double apply(double demand, double last, double dt)
    {
        if (!on) return demand;
        sec += dt;
        if (!(demand >= last) || sec >= kSec) { on = false; return demand; }
        const double w = sec / kSec;
        return std::min(demand, std::max(last, from + (demand - from) * w));
    }
};
