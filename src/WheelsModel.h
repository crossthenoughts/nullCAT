// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// WheelsModel - the Wheels tile: what each wheel does as it turns.
//
// A wheel is never perfectly balanced or round, and a lock-up grinds a
// flat into it. What reaches the seat is not a thump per revolution (that
// reads as a square wheel) but the corner's response, through a quarter
// car (the Road tile's body hz, hop hz, damping), to what the wheel does
// once per revolution:
//   balance  the wheel slightly out of balance: a force on the hub
//            growing with the wheel's rate squared, so it rises steadily
//            with speed, plus a touch of out of round (a road input, once
//            and twice a revolution, the part felt at low speed);
//   flat     a flat spot ground in by a lock-up: a short dip once per
//            revolution, as long as the contact patch, deeper the further
//            and harder the wheel slid locked, wearing slowly round again;
//   judder   hot brake discs varying in thickness: the braking force
//            pulsing twice per revolution, only under braking with hot
//            discs (felt fore and aft: belts, surge, shakers).
// A force F on the hub (per unit unsprung mass) enters the quarter car as
// the road input that makes the same tyre force: F x unsprung / tyre rate.
// Each wheel turns at its own speed, so the corners drift in and out of
// step: a part carrying several corners plays them together (their mean,
// as the body feels them) and beats.
//
// Disc heat is estimated from the brake and the speed (the sim's brake
// temperatures are not on the wire).
//
// RT-safe: fixed arrays, pure arithmetic, no allocation.
// ============================================================

#include "HapticsTypes.h"
#include "NcxTokens.h"      // Wheel order
#include "RoadModel.h"      // QuarterCar, RoadOut, partWheels, roadSoftKnee
#include "WaveSynth.h"
#include <algorithm>
#include <cmath>

namespace haptics {

namespace wheels_k {
    constexpr double kCircM       = 2.0;     // rolling circumference (a 0.32 m tyre)
    constexpr double kImbalanceMm = 0.08;    // balance x 1: the imbalance's offset per unsprung mass (15 g at 0.2 m on 40 kg)
    constexpr double kRunoutMm    = 0.02;    // balance x 1: this far out of round, once per revolution
    constexpr double kRunout2     = 0.3;     // ...and this share of it twice per revolution
    constexpr double kSpotRev     = 0.08;    // a flat through the contact patch spans this share of a revolution
    constexpr double kFlatMaxMm   = 2.0;     // the deepest flat a tyre takes
    constexpr double kFlatRate    = 0.02;    // mm of flat per metre slid locked (x load share): a 20 m slide = 0.4 mm
    constexpr double kFlatWearM   = 30000.0; // a flat wears towards round over this many metres (1/e)
    constexpr double kPunctureMm  = 2.0;     // puncture x 1: the dip where the wheel rides on its folded sidewall
    constexpr double kPunctureRev = 0.25;    // ...spanning this share of a revolution
    constexpr double kPunctureFold = 0.15;   // ...and an uneven second fold, this share of it, twice a revolution
    constexpr double kLockRatio   = 0.5;     // a wheel at slip ratio -0.5 or below is locked and grinding
    constexpr double kLockMinMs   = 1.0;     // ...while the car is moving
    constexpr double kHeatRate    = 0.001;   // disc heat per (brake 0..1 x m/s) per second: a hard stop from 300 km/h = 0.2
    constexpr double kCoolSec     = 40.0;    // cooling: heat x (0.2 + speed / 30 m/s) / this per second
    constexpr double kHotFrom     = 0.3;     // discs judder above this heat, full by 1
    constexpr double kJudderMinBrake = 0.05; // under braking
    constexpr double kPosFullMm   = 0.3;     // a post's 100%: the body moving this much
    constexpr double kForceRefHz  = 8.0;     // a force sink's 100% = kPosFullMm moved at this rate
    constexpr double kLevelSec    = 0.2;     // the tile's level: peak, falling over this
    constexpr double kPhase[WHEEL_COUNT] = { 0.0, 0.37, 0.71, 0.13 };   // each wheel its own out-of-round
    constexpr double kImbalancePhase = 0.6;  // where the imbalance sits against the out of round (radians)
    constexpr double kMassRatioWheel = road_k::kMassRatio;   // unsprung / sprung, as the quarter car
}

class WheelsModel
{
public:
    WheelsModel() { clear(); }

    // ---- inputs, per cycle, BEFORE step() ----
    // The wheel's own speed over the road, m/s.
    void driveWheelSpeed(int wheel, double ms)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT || !std::isfinite(ms)) return;
        m_w[wheel].v = std::max(0.0, ms); m_w[wheel].driven = true;
    }
    // For flat-spotting: the car's speed (m/s), the wheel's slip ratio
    // (negative = locking) and its load share (1 = an average corner).
    void driveLock(int wheel, double carMs, double slipRatio, double loadShare)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        W& w = m_w[wheel];
        w.carMs = std::isfinite(carMs) ? std::max(0.0, carMs) : 0.0;
        w.ratio = std::isfinite(slipRatio) ? slipRatio : 0.0;
        w.load  = std::isfinite(loadShare) ? std::max(0.0, std::min(2.0, loadShare)) : 1.0;
    }
    // The brake pedal, 0..1.
    void driveBrake(double brake01) { m_brake = std::isfinite(brake01) ? std::max(0.0, std::min(1.0, brake01)) : 0.0; }
    // How far the tyre touches the ground, 0 in the air .. 1 (not driven this
    // cycle = on the ground): in the air there is no road for it to roll
    // over, no braking force to pulse and nothing to grind a flat into.
    void driveGround(int wheel, double contact)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        m_w[wheel].groundIn = std::isfinite(contact) ? std::max(0.0, std::min(1.0, contact)) : 1.0;
    }
    // The tyre punctured (deflated), 0 or 1, as the sim says.
    void driveDeflated(int wheel, double deflated)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        m_w[wheel].deflated = std::isfinite(deflated) && deflated >= 0.5;
    }
    // A Test preview: at least this flat on the front left and this disc
    // heat, this cycle only (the tyres' own state is untouched).
    void drivePreview(double flatMm, double heat) { m_prevFlat = std::max(0.0, flatMm); m_prevHeat = std::max(0.0, heat); }

    void step(double dtSec, const WheelsParams& q, const RoadParams& rp)
    {
        using namespace wheels_k;
        if (dtSec <= 0.0) return;
        const QuarterCar::K k = QuarterCar::params(rp.bodyHz, rp.hopHz, rp.damping);
        const double outA  = std::exp(-2.0 * wavesynth::kPi * std::max(road_k::kMinHpHz, rp.hpHz) * dtSec);
        const double aFull = kPosFullMm * std::pow(2.0 * wavesynth::kPi * kForceRefHz, 2.0);
        const double balance = std::max(0.0, q.balance), flatX = std::max(0.0, q.flat), judder = std::max(0.0, q.judder);
        const double hubToRoad = kMassRatioWheel / std::max(1e-9, k.kt);   // a hub force as the road input making the same tyre force
        double inst = 0.0;
        m_rateHz = 0.0;
        for (int i = 0; i < WHEEL_COUNT; ++i)
        {
            W& w = m_w[i];
            const double v = w.driven ? w.v : 0.0;
            w.driven = false;
            w.groundF += (w.groundIn - w.groundF) * std::min(1.0, dtSec / road_k::kGroundEaseSec);
            w.groundIn = 1.0;
            // The wheel turning.
            w.rev += v / kCircM * dtSec;
            w.rev -= std::floor(w.rev);
            m_rateHz = std::max(m_rateHz, v / kCircM);
            // Flat-spotting: locked and sliding, the flat where the tread
            // sat on the road when the wheel locked.
            const bool grinding = (w.ratio <= -kLockRatio) && (w.carMs > kLockMinMs) && (w.groundF >= 0.5);
            if (grinding)
            {
                if (!w.wasGrinding) w.spotAt = w.rev;
                w.flatMm = std::min(kFlatMaxMm, w.flatMm + kFlatRate * w.load * w.carMs * dtSec);
            }
            w.wasGrinding = grinding;
            w.flatMm *= std::exp(-v * dtSec / kFlatWearM);
            // Disc heat: in with the brake and the speed, out faster at speed.
            w.heat += dtSec * (kHeatRate * m_brake * w.carMs - w.heat * (0.2 + w.carMs / 30.0) / kCoolSec);
            w.heat = std::max(0.0, std::min(1.0, w.heat));

            // The road this wheel makes as it turns (mm): the imbalance's
            // force on the hub (offset x wheel rate squared), out of round.
            const double ph = 2.0 * wavesynth::kPi * (w.rev + kPhase[i]);
            const double om = 2.0 * wavesynth::kPi * v / kCircM;
            double zr = balance * (kImbalanceMm * om * om * hubToRoad * std::sin(ph + kImbalancePhase)
                                   + kRunoutMm * (std::sin(ph) + kRunout2 * std::sin(2.0 * ph + 1.3 * kPhase[i])));
            const double flatMm = (i == WheelFL) ? std::max(w.flatMm, m_prevFlat) : w.flatMm;
            if (flatMm > 0.0 && flatX > 0.0)
            {
                double d = w.rev - w.spotAt; d -= std::floor(d); if (d > 0.5) d -= 1.0;   // revs from the spot, -0.5..0.5
                if (std::fabs(d) < 0.5 * kSpotRev)
                    zr -= flatX * flatMm * 0.5 * (1.0 + std::cos(2.0 * wavesynth::kPi * d / kSpotRev));
            }
            // A puncture: the wheel rides on the folded sidewall, a deep
            // broad dip once a revolution with an uneven second fold.
            if (w.deflated && q.puncture > 0.0)
            {
                double d = w.rev + kPhase[i]; d -= std::floor(d); if (d > 0.5) d -= 1.0;
                if (std::fabs(d) < 0.5 * kPunctureRev)
                    zr -= q.puncture * kPunctureMm * 0.5 * (1.0 + std::cos(2.0 * wavesynth::kPi * d / kPunctureRev));
                zr -= q.puncture * kPunctureMm * kPunctureFold * (1.0 + std::cos(2.0 * ph + 2.1));
            }
            // In the air the wheel rolls over nothing.
            zr *= w.groundF;
            // The corner starts at rest on whatever the wheel stands on.
            if (!w.seeded) { w.qc.zs = w.qc.zu = w.zsPrev = zr; w.seeded = true; }
            w.qc.step(dtSec, zr, k);
            // Above cut hz: the body's movement and its acceleration.
            w.zsHp = outA * (w.zsHp + w.qc.zs - w.zsPrev); w.zsPrev = w.qc.zs;
            w.asHp = outA * (w.asHp + w.qc.as - w.asPrev); w.asPrev = w.qc.as;
            double force = w.asHp / aFull;
            // Brake judder: the braking force pulsing twice per revolution.
            const double heat = std::max(w.heat, m_prevHeat);
            const double hot  = std::max(0.0, std::min(1.0, (heat - kHotFrom) / (1.0 - kHotFrom)));
            if (judder > 0.0 && hot > 0.0 && m_brake > kJudderMinBrake && v > kLockMinMs)
                force += judder * hot * m_brake * w.groundF * std::sin(2.0 * ph + kPhase[i]);
            w.out.pos   = roadSoftKnee(w.zsHp / kPosFullMm);
            w.out.force = roadSoftKnee(force);
            inst = std::max(inst, std::max(std::fabs(w.out.pos), std::fabs(w.out.force)));
        }
        m_level = std::max(inst, m_level * std::exp(-dtSec / kLevelSec));
        m_prevFlat = m_prevHeat = 0.0;
    }

    // The part's corners together: their mean, as the body feels them.
    RoadOut outputFor(Part part) const
    {
        int first, last; partWheels(part, first, last);
        RoadOut o;
        for (int i = first; i <= last; ++i) { o.pos += m_w[i].out.pos; o.force += m_w[i].out.force; }
        const double n = static_cast<double>(last - first + 1);
        o.pos /= n; o.force /= n;
        return o;
    }
    double level() const              { return m_level; }
    double rateHz() const             { return m_rateHz; }   // the fastest wheel's revolutions per second
    double flatMm(int wheel) const    { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].flatMm : 0.0; }
    double discHeat(int wheel) const  { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].heat : 0.0; }
    double revolution(int wheel) const{ return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].rev : 0.0; }

    // New tyres and cold discs.
    void clear()
    {
        for (W& w : m_w) w = W{};
        m_brake = m_level = m_rateHz = m_prevFlat = m_prevHeat = 0.0;
    }

private:
    struct W
    {
        double v = 0.0, carMs = 0.0, ratio = 0.0, load = 1.0;
        bool   driven = false, wasGrinding = false, seeded = false, deflated = false;
        double groundIn = 1.0, groundF = 1.0;   // touching the ground (driven, eased): 0 in the air
        double rev = 0.0, spotAt = 0.0, flatMm = 0.0, heat = 0.0;
        QuarterCar qc;
        double zsHp = 0.0, zsPrev = 0.0, asHp = 0.0, asPrev = 0.0;
        RoadOut out;
    };
    W m_w[WHEEL_COUNT];
    double m_brake = 0.0, m_level = 0.0, m_rateHz = 0.0;
    double m_prevFlat = 0.0, m_prevHeat = 0.0;
};

} // namespace haptics
