// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// RoadChassisSigns - which way a sender's roll and pitch turn (the Road
// tile's chassis model). The protocol leaves the sign of rollDeg and
// pitchDeg to the sender; the model needs to know whether a positive roll
// lifts the car's left side and a positive pitch its nose.
//
// The suspension tells: its travel is compression positive (the protocol),
// and a corner of the body moves against its own spring's compression. On a
// flat road exactly so (the body rolls onto the outer springs); over bumps
// the body is driven through spring and damper, which turns the relation by
// up to a quarter cycle but never past it. So with the SAME filter on both
// sides, roll against the left-minus-right compression, and pitch against
// front-minus-rear, correlate negatively when positive means left up and
// nose up, and positively when it means the other way.
//
// The evidence is taken in stretches of kSignStretchSec, each stretch's own
// correlation counting once whatever its size, and the mean of them over
// the whole drive (never fading) is what decides: a crash or a rollover is
// a few stretches among hundreds, where summed by size it outweighed the
// drive. Measured so on twelve recorded drives (six games, three senders)
// in the model's band: roll -0.40 to -0.74 on every one; pitch -0.16 to
// -0.67 on eleven and -0.02 on one (a prototype in Le Mans Ultimate through
// SimHub, whose pitch says nothing either way).
//
// A second counts only while both of its sides move (past kSignMinDeg and
// kSignMinMm): a pitch that is not sent, a suspension that stands still or
// a value held through a pause is no evidence, however long it lasts.
// RoadModel feeds it only while the car rolls above kSignMinKmh on all
// four tyres (a car standing in its pit says nothing about the road: on
// one recorded drive its noise took a sign the driving then had to turn
// round). A sign is taken once kSignMinSec of evidence shows a mean of
// kSignCorr or more, and then stands until the whole drive says the
// opposite as clearly; an axis that never shows it stays out of the model.
// forget() on another game (the convention is the sender's for that game).
//
// The learner this replaces took the suspension's own polarity from the
// heave first, through filters that were not the same on both sides: on the
// same drives it read the polarity backwards and changed its mind up to 14
// times in a quarter of an hour, flipping the roll and pitch each time.
//
// RT-safe: a handful of sums, pure arithmetic.
// ============================================================

#include <cmath>

namespace haptics {

namespace road_k {
    constexpr double kSignMinSec     = 20.0;   // evidence before a sign is taken
    constexpr double kSignStretchSec = 2.0;    // ...taken this much at a time, each stretch counting once
    constexpr double kSignMinKmh     = 30.0;   // evidence only from a car rolling faster than this
    constexpr double kSignCorr       = 0.1;    // the mean correlation that takes a sign, or turns one round
    constexpr double kSignMinDeg     = 1e-4;   // an angle moving less than this (in the model's band) is not moving
    constexpr double kSignMinMm      = 1e-3;   // ...and a suspension difference under this
}

struct RoadChassisSigns
{
    // One cycle: the body's roll and pitch, and the suspension's
    // left-minus-right and front-minus-rear compression, all four through
    // the same filter.
    void add(double roll, double pitch, double leftMinusRight, double frontMinusRear, double dtSec)
    {
        if (!(dtSec > 0.0) || !std::isfinite(roll) || !std::isfinite(pitch)
            || !std::isfinite(leftMinusRight) || !std::isfinite(frontMinusRear)) return;
        m_r.add(roll, leftMinusRight, dtSec, m_signR);
        m_p.add(pitch, frontMinusRear, dtSec, m_signP);
    }

    // Another game: its sender's convention is to be found again.
    void forget() { *this = RoadChassisSigns{}; }

    // +1: a positive roll lifts the left side / a positive pitch lifts the
    // nose. -1: the other way. 0: not known yet.
    int roll() const  { return m_signR; }
    int pitch() const { return m_signP; }
    // The seconds of evidence behind each, and what it says so far (the
    // mean of the finished stretches' correlations).
    double rollSec() const   { return m_r.seconds(); }
    double pitchSec() const  { return m_p.seconds(); }
    double rollCorr() const  { return m_r.mean(); }
    double pitchCorr() const { return m_p.mean(); }

private:
    struct Axis
    {
        double xy = 0.0, xx = 0.0, yy = 0.0, sec = 0.0;   // the stretch being filled
        double sum = 0.0; int n = 0;                       // the finished stretches' correlations

        void add(double x, double y, double dt, int& sign)
        {
            using namespace road_k;
            if (!(std::fabs(x) > kSignMinDeg && std::fabs(y) > kSignMinMm)) return;   // one side still: no evidence
            xy += dt * x * y; xx += dt * x * x; yy += dt * y * y; sec += dt;
            if (sec < kSignStretchSec) return;
            const double d = std::sqrt(xx * yy);
            sum += d > 1e-30 ? xy / d : 0.0; ++n;
            xy = xx = yy = sec = 0.0;
            if (n * kSignStretchSec < kSignMinSec) return;
            // The body against the compression: opposite = the positive side is up.
            const double m = mean();
            if (m <= -kSignCorr)     sign = 1;
            else if (m >= kSignCorr) sign = -1;
        }
        double mean() const    { return n > 0 ? sum / n : 0.0; }
        double seconds() const { return n * road_k::kSignStretchSec + sec; }
    };
    Axis m_r, m_p;
    int  m_signR = 0, m_signP = 0;
};

} // namespace haptics
