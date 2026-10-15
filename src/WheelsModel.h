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
//            discs (felt fore and aft: belts, surge, shakers);
//   bite     the pads on the turning disc: a rough grain at bite hz (at
//   drag     100 km/h, following each wheel's own speed), swelling as the
//            pedal goes on or is squeezed harder (the grab) and settling
//            to a steady rub (the drag), stronger at speed and on the
//            loaded tyre, nothing from a locked wheel; each cycle a little
//            longer or shorter and shallower, so it reads as friction, not
//            a tone. It acts at the caliper, not through the suspension:
//            straight to a belt or shaker, and a fine movement of its own
//            on a post (derated at its own pitch).
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
#include <cstdint>

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
    constexpr double kLockHoldSec = 0.1;     // ...for this long (a single bad sample is no lock-up)
    constexpr double kHeatRate    = 0.001;   // disc heat per (brake 0..1 x m/s) per second: a hard stop from 300 km/h = 0.2
    constexpr double kCoolSec     = 40.0;    // cooling: heat x (0.2 + speed / 30 m/s) / this per second
    constexpr double kHotFrom     = 0.3;     // discs judder above this heat, full by 1
    constexpr double kJudderMinBrake = 0.05; // under braking
    // The pads: the grain's pitch is bite hz at this wheel speed, following
    // it (never below the floor); the grab builds with the pedal's rise (a
    // press from nothing to half pedal in a moment: full) and fades over
    // this long; the grain grows with the square root of speed, full from
    // the reference; each cycle up to this much longer or shorter and this
    // much shallower; a post's share of it (of its 100 %).
    constexpr double kBiteRefMs     = 100.0 / 3.6;
    constexpr double kBiteMinHz     = 4.0;
    constexpr double kGrabPerPedal  = 2.0;
    constexpr double kGrabSec       = 0.25;
    constexpr double kBiteMinBrake  = 0.02;
    constexpr double kBitePeriodVar = 0.25, kBiteDepthVar = 0.3;
    constexpr double kBiteGrit      = 0.35;  // a second harmonic: the pads' grit
    constexpr double kBitePosShare  = 0.1;   // a post: this share of its 100 % at full
    constexpr double kPadRampSec    = 0.008; // the grains ease in and out over this (no click)
    constexpr double kPadSamplesPerCycle = 8.0;   // ...and never ask the loop for fewer samples a cycle
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
    // This cycle only, like the brake below: not driven = not known (the
    // stream stopped mid lock-up grinds nothing in).
    void driveLock(int wheel, double carMs, double slipRatio, double loadShare)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        W& w = m_w[wheel];
        w.carMs = std::isfinite(carMs) ? std::max(0.0, carMs) : 0.0;
        w.ratio = std::isfinite(slipRatio) ? slipRatio : 0.0;
        w.load  = std::isfinite(loadShare) ? std::max(0.0, std::min(2.0, loadShare)) : 1.0;
    }
    // The brake pedal, 0..1, this cycle.
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
        const double bite = std::max(0.0, std::min(1.0, q.bite)), drag = std::max(0.0, std::min(1.0, q.drag));
        const double hubToRoad = kMassRatioWheel / std::max(1e-9, k.kt);   // a hub force as the road input making the same tyre force
        double inst = 0.0;
        m_rateHz = m_grabHz = m_dragHz = 0.0;
        // The grab: the pedal going on, or squeezed harder, fading after.
        const double rise = std::max(0.0, m_brake - m_brakePrev);
        m_brakePrev = m_brake;
        m_grab = (m_brake < kBiteMinBrake) ? 0.0
               : std::min(1.0, m_grab * std::exp(-dtSec / kGrabSec) + kGrabPerPedal * rise);
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
            // Flat-spotting: locked and sliding a moment, the flat where the
            // tread sat on the road when the wheel locked.
            const bool locked = (w.ratio <= -kLockRatio) && (w.carMs > kLockMinMs) && (w.groundF >= 0.5);
            w.lockSec = locked ? w.lockSec + dtSec : 0.0;
            const bool grinding = w.lockSec >= kLockHoldSec;
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
            // The pads: how far the disc is turning against the car (a locked
            // wheel's disc stands still under them) and the tyre's load share.
            const double turning = (w.carMs > kLockMinMs) ? std::max(0.0, std::min(1.0, v / w.carMs)) : (v > kLockMinMs ? 1.0 : 0.0);
            const double loadK   = std::max(0.3, std::min(1.5, w.load));
            w.carMs = 0.0; w.ratio = 0.0; w.load = 1.0;   // this cycle's, used

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
            // The pads on the disc: the grab and the drag, each a dithered
            // grain at its own pitch following this wheel's speed, at the
            // caliper (not through the suspension).
            // (Off, each eases out rather than stopping dead.)
            {
                const double k = (m_brake >= kBiteMinBrake && v > kLockMinMs)
                               ? std::sqrt(std::min(1.0, v / kBiteRefMs)) * turning * loadK : 0.0;
                force += grain(w.grab, w.rng, bite * m_grab * k, q.biteHz * v / kBiteRefMs, dtSec, m_grabHz);
                force += grain(w.drag, w.rng, drag * m_brake * k, q.dragHz * v / kBiteRefMs, dtSec, m_dragHz);
            }
            w.out.pos   = roadSoftKnee(w.zsHp / kPosFullMm);
            w.out.force = roadSoftKnee(force);
            inst = std::max(inst, std::max(std::fabs(w.out.pos), std::fabs(w.out.force)));
        }
        m_level = std::max(inst, m_level * std::exp(-dtSec / kLevelSec));
        m_prevFlat = m_prevHeat = m_brake = 0.0;
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
    // The pads' grains for a post (a movement of their own, each derated at
    // its pitch): the part's corners' mean. drag = false: the grab.
    double padFor(Part part, bool drag) const
    {
        int first, last; partWheels(part, first, last);
        double s = 0.0;
        for (int i = first; i <= last; ++i) s += drag ? m_w[i].drag.pos : m_w[i].grab.pos;
        return s / static_cast<double>(last - first + 1);
    }
    double padHz(bool drag) const     { return drag ? m_dragHz : m_grabHz; }   // the fastest wheel's grain this cycle (0 = none)
    double level() const              { return m_level; }
    double rateHz() const             { return m_rateHz; }   // the fastest wheel's revolutions per second
    double flatMm(int wheel) const    { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].flatMm : 0.0; }
    double discHeat(int wheel) const  { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].heat : 0.0; }
    bool   punctured(int wheel) const { return wheel >= 0 && wheel < WHEEL_COUNT && m_w[wheel].deflated; }
    double revolution(int wheel) const{ return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].rev : 0.0; }

    // Fresh tyres and cold discs, rolling on (the wheels keep turning: no
    // step in the output).
    void freshTyres()
    {
        for (W& w : m_w) { w.flatMm = w.spotAt = w.heat = w.lockSec = 0.0; w.wasGrinding = false; }
        m_prevFlat = m_prevHeat = 0.0;
    }
    // Everything back to rest (a new car, the tile off).
    void clear()
    {
        uint64_t seed = 0x9E3779B97F4A7C15ull;
        for (W& w : m_w) { w = W{}; w.rng = seed; seed = seed * 6364136223846793005ull + 1442695040888963407ull; }
        m_brake = m_level = m_rateHz = m_prevFlat = m_prevHeat = 0.0;
        m_brakePrev = m_grab = m_grabHz = m_dragHz = 0.0;
    }

private:
    // One of the pads' grains on one wheel: phase, this cycle's length and
    // depth (each cycle a little different), a post's share of it.
    struct Grain { double ph = 0.0, k = 1.0, d = 1.0, pos = 0.0, e = 0.0; };   // e: the eased strength
    // Advance a grain at hz (never below the floor) and give its force at
    // strength e (0 = silent), noting the pitch for a post's derating.
    // The strength eases over kPadRampSec each way (no step on or off); once
    // silent the phase starts again at a zero crossing. The pitch never asks
    // for fewer than kPadSamplesPerCycle loop samples a cycle (a fast car at
    // a high bite hz aliased into a lower tone), the grit dropped when its
    // harmonic would.
    static double grain(Grain& g, uint64_t& rng, double target, double hz, double dtSec, double& maxHz)
    {
        using namespace wheels_k;
        g.e += (std::max(0.0, std::min(1.0, target)) - g.e) * std::min(1.0, dtSec / kPadRampSec);
        if (g.e < 1e-4) { g.e = 0.0; g.ph = 0.0; g.pos = 0.0; return 0.0; }
        const double top = 1.0 / (std::max(1e-6, dtSec) * kPadSamplesPerCycle);
        hz = std::max(kBiteMinHz, std::min(top, hz));
        g.ph += hz * g.k * dtSec;
        if (g.ph >= 1.0)
        {
            g.ph -= std::floor(g.ph);
            g.k = 1.0 + kBitePeriodVar * (2.0 * wavesynth::rand01(rng) - 1.0);
            g.d = 1.0 - kBiteDepthVar * wavesynth::rand01(rng);
        }
        const double a = 2.0 * wavesynth::kPi * g.ph;
        const bool   grit = 2.0 * hz * g.k <= top;
        const double s = grit ? (std::sin(a) + kBiteGrit * std::sin(2.0 * a + 0.7)) / (1.0 + kBiteGrit) : std::sin(a);
        const double f = g.e * g.d * s;
        g.pos = kBitePosShare * f;
        maxHz = std::max(maxHz, hz);
        return f;
    }
    struct W
    {
        double v = 0.0, carMs = 0.0, ratio = 0.0, load = 1.0;
        bool   driven = false, wasGrinding = false, seeded = false, deflated = false;
        double groundIn = 1.0, groundF = 1.0;   // touching the ground (driven, eased): 0 in the air
        double rev = 0.0, spotAt = 0.0, flatMm = 0.0, heat = 0.0, lockSec = 0.0;
        QuarterCar qc;
        double zsHp = 0.0, zsPrev = 0.0, asHp = 0.0, asPrev = 0.0;
        Grain grab, drag;   // the pads: the grab and the drag
        uint64_t rng = 1;
        RoadOut out;
    };
    W m_w[WHEEL_COUNT];
    double m_brake = 0.0, m_level = 0.0, m_rateHz = 0.0;
    double m_prevFlat = 0.0, m_prevHeat = 0.0;
    double m_brakePrev = 0.0, m_grab = 0.0, m_grabHz = 0.0, m_dragHz = 0.0;
};

} // namespace haptics
