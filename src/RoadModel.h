// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// RoadModel - the Road tile's three models, and KerbModel, the Kerb tile's
// rumble strip.
//
// What a driver feels of the road is the BODY moving: the road under each
// tyre lifts the wheel, the tyre swallows bumps shorter than its contact
// patch, the spring and damper pass part of the wheel's motion to the
// body, and the body's own mass smooths it. A seat post that replays the
// body's motion above the band the motion cue already covers feels like
// the road; one that replays suspension travel directly turns every sharp
// edge into a pop (felt acceleration grows with frequency squared).
//
//   0 SUSPENSION  each corner's suspension travel (or velocity,
//                 integrated), high-passed at cut hz, /fullMm, plus the
//                 road's fine roughness the sim does not model: the same
//                 random road as model 1, laid out by distance, through the
//                 tyre's envelope and a quarter car, its SUSPENSION movement
//                 (wheel against body) added to the sim's. rough x 0 = the
//                 sim's travel alone.
//   1 TYRE        a quarter car per corner (sprung mass, spring, damper,
//                 unsprung mass, tyre: body hz, hop hz, damping) driven by
//                 the road under that tyre: the sim's real road where it
//                 sends one (roadHeight*, high-passed to drop the hills),
//                 else the suspension travel as the road input; plus a
//                 random road laid out by DISTANCE (ISO 8608 roughness, x
//                 rough, x the surface class under the tyre), so its
//                 frequency follows speed by itself and the rear wheels
//                 meet the fronts' bumps again wheelbase / speed later. The
//                 tyre envelopes bumps shorter than its patch (a low-pass at
//                 speed / patch). Out: the body's motion above cut hz.
//   2 CHASSIS     the sim's own body: heave acceleration integrated twice,
//                 pitch and roll, above cut hz, spread to the corners by
//                 the car's wheelbase and track (rigid body). The signs of
//                 pitch and roll against the corners are LEARNED from the
//                 per-wheel suspension (no rig geometry asked of anyone):
//                 until learned, heave only.
//
// Every model yields per wheel a POSITION sample (body displacement / the
// full scale, for seat posts) and a FORCE sample (body acceleration, for
// belts and shakers, scaled so a body movement of full scale at 8 Hz is
// 100%): a post is told how far, a force device how hard. Past 60% of
// full both bend over a soft knee towards 1 rather than clip.
//
// RT-safe: fixed arrays, pure arithmetic, no allocation.
// ============================================================

#include "HapticsTypes.h"
#include "NcxTokens.h"      // Wheel order, SurfaceClass
#include "WaveSynth.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace haptics {

namespace road_k {
    constexpr double kMinHpHz    = 0.5;
    constexpr double kMinFullMm  = 0.5;
    constexpr double kReleaseSec = 0.120;   // fade when the stream stops driving
    constexpr double kDerateHz   = 8.0;     // representative bump rate for position-sink derating
    constexpr double kKnee       = 0.6;     // linear up to this share of full, then a soft knee
    constexpr double kSmoothHz   = 20.0;    // one-pole low-pass on the suspension replay
    // Tyre model.
    constexpr double kRoadHpHz   = 0.5;     // hills off the real road profile
    constexpr double kPatchM     = 0.15;    // contact patch length: envelops shorter bumps
    constexpr double kMassRatio  = 0.15;    // unsprung / sprung mass
    constexpr double kSubStepSec = 0.001;   // quarter-car integration step ceiling
    constexpr double kProfileStepM = 0.02;  // the random road's grid
    constexpr int    kProfileN     = 512;   // 10.24 m of it held
    constexpr double kProfileLeakM = 20.0;  // its longest wavelength
    constexpr double kIsoSigmaMm   = 1.777; // ISO 8608 class A: sqrt(2 pi^2 n0^2 Gd(n0)), mm / sqrt(m)
    constexpr double kLRCorr       = 0.5;   // left/right correlation of the road
    constexpr double kDefaultWheelbaseM = 2.7, kDefaultTrackM = 1.6;
    constexpr double kMinSpeedMs   = 1.0;   // the road does not move under a car slower than this
    constexpr double kForceRefHz   = 8.0;   // a force sink's 100% = full scale moved at this rate
    constexpr double kInputLpHz    = 25.0;  // sim values arrive ~60 times a second, held: smooth the steps
    // Roughness per surface class (tarmac, bumpy, kerb, gravel, grass, dirt,
    // cobbles, snow, ice, sand, mud, water), x the tarmac road: its LONG
    // undulations and its SHORT wavelengths (under kSurfWaveM, the surface
    // itself) apart, so grass undulates softly, gravel is coarse, snow fills
    // the bumps and ice is glassy. Kerb ribs are the Kerb tile's; stones,
    // crunch and puddles the Surface tile's. A change of surface eases in
    // over kSurfEaseSec.
    constexpr double kSurfLong[SURFACE_CLASS_COUNT]  = { 1.0, 2.0, 1.0, 1.5, 2.5, 2.0, 1.0, 0.7, 0.5, 1.5, 2.0, 1.0 };
    constexpr double kSurfShort[SURFACE_CLASS_COUNT] = { 1.0, 3.0, 1.0, 6.0, 1.5, 4.0, 4.0, 0.8, 0.2, 2.5, 2.0, 0.3 };
    constexpr double kSurfWaveM   = 2.0;
    constexpr double kSurfEaseSec = 0.1;
    // Chassis model.
    constexpr double kLearnSec   = 5.0;     // sign learning memory
    constexpr double kLearnCorr  = 0.3;     // correlation that sets a sign
    // Kerb model.
    constexpr double kKerbAttackSec  = 0.008;   // onto the kerb: abrupt
    constexpr double kKerbReleaseSec = 0.025;   // off it: nearly as abrupt
    constexpr double kKerbForceHpHz  = 10.0;    // a force device feels the ribs and the step's edges, not the step
    constexpr double kKerbRefSpeedMs = 22.2;    // 80 km/h: the ribs at full strength
    constexpr double kKerbPosHpHz    = 3.0;     // a post: a thud on and off, not a held lift along the kerb
}

// Linear to kKnee, then bending smoothly (matching slope) towards 1.
inline double roadSoftKnee(double x)
{
    using namespace road_k;
    const double a = std::fabs(x);
    if (a <= kKnee) return x;
    const double y = kKnee + (1.0 - kKnee) * std::tanh((a - kKnee) / (1.0 - kKnee));
    return (x < 0.0) ? -y : y;
}

// Wheels of a Part.
inline void partWheels(Part part, int& first, int& last)
{
    first = 0; last = WHEEL_COUNT - 1;
    switch (part)
    {
        case Part::Front: first = WheelFL; last = WheelFR; break;
        case Part::Rear:  first = WheelRL; last = WheelRR; break;
        case Part::FL:    first = last = WheelFL; break;
        case Part::FR:    first = last = WheelFR; break;
        case Part::RL:    first = last = WheelRL; break;
        case Part::RR:    first = last = WheelRR; break;
        default: break;
    }
}

// One output sample per sink kind.
struct RoadOut { double pos = 0.0, force = 0.0; };

// A quarter car: sprung mass on spring + damper, unsprung mass on the tyre.
// States in mm; parameters per unit sprung mass, from body hz, wheel-hop hz
// and damping ratio (the mass ratio fixed at kMassRatio).
struct QuarterCar
{
    double zs = 0.0, vs = 0.0, zu = 0.0, vu = 0.0, as = 0.0;

    struct K { double ks = 0.0, kt = 0.0, cs = 0.0; };
    static K params(double bodyHz, double hopHz, double damping)
    {
        using namespace road_k;
        const double mu = kMassRatio;
        const double w1 = 2.0 * wavesynth::kPi * std::max(0.3, bodyHz);
        // The body and wheel-hop modes cannot be any closer than this for a
        // real spring/tyre pair: hop is lifted to the lowest feasible value.
        const double w2min = 2.0 * w1 / std::sqrt(mu) * 1.02;
        const double w2 = std::max(w2min, 2.0 * wavesynth::kPi * hopHz);
        const double S = mu * w2 * w2, P = mu * w1 * w1 * w2 * w2;
        const double disc = std::sqrt(std::max(0.0, S * S - 4.0 * P));
        K k;
        k.ks = 0.5 * (S - disc);     // the spring is the softer of the two
        k.kt = 0.5 * (S + disc);     // the tyre the stiffer
        // The damper sits across the spring alone, and in the body's bounce
        // the wheel moves with the body (the tyre gives too: by r = 1 -
        // w1^2 / ks of it), so only w1^2 / ks of the body's motion works
        // the damper. Sized so the BOUNCE has the damping ratio asked (what
        // a driver feels as how quickly it settles), not the spring alone.
        const double rel = std::max(0.2, std::min(1.0, w1 * w1 / std::max(1e-9, k.ks)));
        const double r   = 1.0 - rel;
        k.cs = 2.0 * std::max(0.0, damping) * w1 * (1.0 + mu * r * r) / (rel * rel);
        return k;
    }

    void step(double dt, double zr, const K& k)
    {
        using namespace road_k;
        const int n = std::max(1, static_cast<int>(std::ceil(dt / kSubStepSec)));
        const double h = dt / n;
        for (int i = 0; i < n; ++i)
        {
            const double f  = k.ks * (zu - zs) + k.cs * (vu - vs);   // suspension on the body (per unit sprung mass)
            as = f;
            const double au = (-f + k.kt * (zr - zu)) / kMassRatio;
            vs += as * h; vu += au * h;
            zs += vs * h; zu += vu * h;
        }
    }
};

class RoadModel
{
public:
    // ---- inputs, per cycle, BEFORE step() ----
    // Suspension velocity, mm/s, signed.
    void drive(int wheel, double velMmS)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        if (!std::isfinite(velMmS)) velMmS = 0.0;
        m_w[wheel].v = velMmS;
        m_w[wheel].dx = 0.0;
        m_w[wheel].driven = true;
    }
    // ...or suspension travel, mm, signed, for sims that give position
    // and not velocity. The change since the last sample enters the same
    // high-passed integral, so a held sample adds nothing.
    void driveTravel(int wheel, double travelMm)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        if (!std::isfinite(travelMm)) return;
        W& w = m_w[wheel];
        w.dx = w.haveTravel ? (travelMm - w.lastTravel) : 0.0;
        w.lastTravel = travelMm; w.haveTravel = true;
        w.v = 0.0;
        w.driven = true;
    }
    // The road under the tyre, mm, world height (hills included).
    void driveHeight(int wheel, double heightMm)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT || !std::isfinite(heightMm)) return;
        m_w[wheel].h = heightMm; m_w[wheel].heightDriven = true;
    }
    // What the ground under the tyre is (SurfaceClass).
    void driveSurface(int wheel, int cls)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        m_w[wheel].surface = (cls >= 0 && cls < SURFACE_CLASS_COUNT) ? cls : SurfTarmac;
    }
    void driveSpeed(double speedMs)
    {
        m_speed = std::isfinite(speedMs) ? std::max(0.0, speedMs) : 0.0;
        m_speedDriven = true;
    }
    // The water under the tyre (0..1) and how far it floats on it (0..1),
    // from the Surface tile; smooth: how far water fills the texture.
    void driveWater(int wheel, double water, double floating)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        m_w[wheel].water    = std::isfinite(water)    ? std::max(0.0, std::min(1.0, water))    : 0.0;
        m_w[wheel].floating = std::isfinite(floating) ? std::max(0.0, std::min(1.0, floating)) : 0.0;
    }
    void setWaterSmooth(double smooth) { m_smooth = std::max(0.0, std::min(1.0, smooth)); }
    // The body: vertical acceleration m/s^2, pitch and roll degrees.
    void driveChassis(double accHeaveMs2, double pitchDeg, double rollDeg)
    {
        if (!std::isfinite(accHeaveMs2) || !std::isfinite(pitchDeg) || !std::isfinite(rollDeg)) return;
        m_acc = accHeaveMs2; m_pitch = pitchDeg; m_roll = rollDeg; m_chassisDriven = true;
    }
    void driveGeometry(double wheelbaseM, double trackM)
    {
        if (wheelbaseM > 1.0 && wheelbaseM < 6.0) m_wheelbase = wheelbaseM;
        if (trackM > 0.8 && trackM < 3.0)         m_track = trackM;
    }

    static double softKnee(double x) { return roadSoftKnee(x); }

    void step(double dtSec, const RoadParams& p)
    {
        using namespace road_k;
        if (dtSec <= 0.0) return;
        const int model = static_cast<int>(p.model + 0.5);
        const double hp   = std::max(kMinHpHz, p.hpHz);
        const double leak = std::exp(-2.0 * wavesynth::kPi * hp * dtSec);

        // The suspension integral runs whatever the model: it is model 0's
        // output, model 1's road input where the sim sends no road, and
        // model 2's reference for its signs.
        bool susp = false, height = false;
        for (W& w : m_w)
        {
            if (w.driven) w.x = w.x * leak + w.v * dtSec + w.dx;
            else        { w.x *= leak; w.haveTravel = false; }
            w.suspDriven = w.driven;
            susp   = susp   || w.driven;
            height = height || w.heightDriven;
            w.driven = false; w.dx = 0.0;
        }
        // Live while the model's own inputs arrive.
        const bool anyDriven = (model == 1) ? (susp || height || m_speedDriven)
                             : (model == 2) ? m_chassisDriven
                             : susp;

        if (model == 1)      stepTyre(dtSec, p, hp);
        else if (model == 2) stepChassis(dtSec, p, hp, leak);
        else                 stepSuspension(dtSec, p, hp);

        // Gate: on while anything drives the model, fading when it stops.
        for (W& w : m_w)
        {
            if (anyDriven) w.gate = 1.0;
            else           w.gate = std::max(0.0, w.gate - dtSec / kReleaseSec);
            w.out.pos *= w.gate; w.out.force *= w.gate;
            w.heightDriven = false;
        }
        m_speedDriven = m_chassisDriven = false;
    }

    // The strongest corner of the part, per sink kind.
    RoadOut outputFor(Part part) const
    {
        int first, last; partWheels(part, first, last);
        RoadOut o;
        for (int i = first; i <= last; ++i)
        {
            if (std::fabs(m_w[i].out.pos)   > std::fabs(o.pos))   o.pos   = m_w[i].out.pos;
            if (std::fabs(m_w[i].out.force) > std::fabs(o.force)) o.force = m_w[i].out.force;
        }
        return o;
    }

    double level() const
    {
        double m = 0.0;
        for (const W& w : m_w) m = std::max(m, std::fabs(w.out.pos));
        return m;
    }

    // What the model plays at that corner, in mm (the tile's per-wheel line).
    double wheelTravelMm(int wheel) const
    {
        return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].mm : 0.0;
    }

    bool active() const
    {
        for (const W& w : m_w) if (w.gate > 0.0) return true;
        return false;
    }

    void clear()
    {
        for (W& w : m_w) w = W{};
        m_dist = 0.0; m_genIdx = -1; m_c = m_iL = m_iR = 0.0;
        for (auto& side : m_prof) for (float& z : side) z = 0.0f;
        m_speed = 0.0; m_speedDriven = m_chassisDriven = false;
        m_vh = m_zh = m_aHp = m_aPrev = 0.0;
        m_accF = m_pitchF = m_rollF = 0.0;
        m_rollHp = m_rollPrev = m_pitchHp = m_pitchPrev = 0.0;
        m_cH = m_eH = m_eM = m_cR = m_eR = m_eLR = m_cP = m_eP = m_eFB = 0.0;
        m_signH = 1; m_signR = m_signP = 0; m_pol = 0;
        m_chassisSeen = false;
    }

    // ---- test hooks ----
    double profileAt(int side, double distM) const
    {
        using namespace road_k;
        if (m_genIdx < 0) return 0.0;
        double idx = distM / kProfileStepM;
        const double newest = static_cast<double>(m_genIdx), oldest = std::max(0.0, newest - (kProfileN - 2));
        idx = std::max(oldest, std::min(newest, idx));
        const int64_t i0 = static_cast<int64_t>(std::floor(idx));
        const double  fr = idx - static_cast<double>(i0);
        const int64_t i1 = std::min<int64_t>(i0 + 1, m_genIdx);
        const float* z = m_prof[side & 1];
        return z[i0 % kProfileN] * (1.0 - fr) + z[i1 % kProfileN] * fr;
    }
    int    learnedPolarity() const  { return m_pol; }
    int    learnedRollSign() const  { return m_signR; }
    int    learnedPitchSign() const { return m_signP; }
    double distance() const         { return m_dist; }

private:
    struct W
    {
        // inputs
        double v = 0.0, dx = 0.0, lastTravel = 0.0, h = 0.0;
        int    surface = SurfTarmac;
        bool   haveTravel = false, driven = false, suspDriven = false, heightDriven = false;
        // suspension replay
        double x = 0.0, y = 0.0;
        // tyre model
        double hpH = 0.0, hpHPrev = 0.0; bool hpHSeen = false;   // the real road, hills off
        double realF = 0.0;                                      // ...its held samples smoothed
        double zrF = 0.0;                                        // after the tyre's envelope
        QuarterCar qc;
        double zsHp = 0.0, zsPrev = 0.0, asHp = 0.0, asPrev = 0.0;
        // the random road under this tyre (both models)
        double rBase = 0.0, rShort = 0.0, rPrev = 0.0, rGainL = 1.0, rGainS = 1.0; bool rSeen = false;
        double water = 0.0, floating = 0.0;   // from the Surface tile: water under the tyre, the tyre floating on it
        // suspension model's roughness: its quarter car and output
        double synF = 0.0, dHp = 0.0, dPrev = 0.0;
        QuarterCar qr;
        // out
        RoadOut out;
        double mm = 0.0, gate = 0.0;
    };

    void stepSuspension(double dtSec, const RoadParams& p, double hpHz)
    {
        using namespace road_k;
        const double full = std::max(kMinFullMm, p.fullMm);
        const double smooth = 1.0 - std::exp(-2.0 * wavesynth::kPi * kSmoothHz * dtSec);
        const bool rough = p.rough > 0.0;
        double envA = 0.0, outA = 0.0, wb = 0.0;
        QuarterCar::K k;
        if (rough)
        {
            advanceRoad(dtSec);
            wb   = (m_wheelbase > 0.0) ? m_wheelbase : kDefaultWheelbaseM;
            envA = 1.0 - std::exp(-2.0 * wavesynth::kPi * envelopeHz(dtSec) * dtSec);
            outA = std::exp(-2.0 * wavesynth::kPi * hpHz * dtSec);
            k    = QuarterCar::params(p.bodyHz, p.hopHz, p.damping);
        }
        for (int i = 0; i < WHEEL_COUNT; ++i)
        {
            W& w = m_w[i];
            double add = 0.0;
            if (rough)
            {
                // The roughness through the tyre and the corner: what the
                // suspension does over it (the wheel against the body).
                w.synF += (roughAt(i, dtSec, p, wb) - w.synF) * envA;
                w.qr.step(dtSec, w.synF, k);
                const double d = w.qr.zu - w.qr.zs;
                w.dHp = outA * (w.dHp + d - w.dPrev); w.dPrev = d;
                add = w.dHp;
            }
            const double total = w.x + add;
            w.y += (softKnee(total / full) - w.y) * smooth;
            w.out.pos = w.out.force = w.y;
            w.mm = total;
        }
    }

    // The random road moves under the car with the distance it covers.
    void advanceRoad(double dtSec)
    {
        using namespace road_k;
        if (m_speed > kMinSpeedMs) { m_dist += m_speed * dtSec; generateTo(m_dist); }
        else if (m_genIdx < 0) generateTo(m_dist);
    }
    // The tyre envelopes bumps shorter than its contact patch.
    double envelopeHz(double dtSec) const
    {
        using namespace road_k;
        return std::max(5.0, std::min(0.45 / dtSec, m_speed / kPatchM));
    }

    // The random road under wheel i this cycle, mm: the profile at its
    // distance (the rears a wheelbase behind), drift off, x rough x, its
    // long undulations and short wavelengths each x the surface class's
    // gain (eased). Scaling the whole profile by one gain stepped the road
    // by tens of mm at a change of surface. Water fills the short
    // wavelengths (smooth x water); a tyre floating on it (aquaplaning) has
    // no road under it at all.
    double roughAt(int i, double dtSec, const RoadParams& p, double wb)
    {
        using namespace road_k;
        W& w = m_w[i];
        const bool front = (i == WheelFL || i == WheelFR);
        const int  side  = (i == WheelFL || i == WheelRL) ? 0 : 1;
        const double raw = profileAt(side, front ? m_dist : m_dist - wb);
        const double wetK   = 1.0 - m_smooth * w.water;
        const double floatK = 1.0 - w.floating;
        const double gL = kSurfLong[w.surface] * floatK;
        const double gS = kSurfShort[w.surface] * wetK * floatK;
        if (!w.rSeen) { w.rPrev = raw; w.rBase = w.rShort = 0.0; w.rGainL = gL; w.rGainS = gS; w.rSeen = true; }
        const double baseA  = std::exp(-2.0 * wavesynth::kPi * kRoadHpHz * dtSec);
        const double shortA = std::exp(-2.0 * wavesynth::kPi * std::max(kRoadHpHz, m_speed / kSurfWaveM) * dtSec);
        const double dRaw = raw - w.rPrev; w.rPrev = raw;
        w.rBase  = baseA  * (w.rBase  + dRaw);
        w.rShort = shortA * (w.rShort + dRaw);
        const double ease = std::min(1.0, dtSec / kSurfEaseSec);
        w.rGainL += (gL - w.rGainL) * ease;
        w.rGainS += (gS - w.rGainS) * ease;
        return std::max(0.0, p.rough) * (w.rGainL * w.rBase + (w.rGainS - w.rGainL) * w.rShort);
    }

    void stepTyre(double dtSec, const RoadParams& p, double hpHz)
    {
        using namespace road_k;
        advanceRoad(dtSec);
        const double wb = (m_wheelbase > 0.0) ? m_wheelbase : kDefaultWheelbaseM;
        const double fc = envelopeHz(dtSec);
        const double envA  = 1.0 - std::exp(-2.0 * wavesynth::kPi * fc * dtSec);
        const double roadA = std::exp(-2.0 * wavesynth::kPi * kRoadHpHz * dtSec);
        const double outA  = std::exp(-2.0 * wavesynth::kPi * hpHz * dtSec);
        const double inA   = 1.0 - std::exp(-2.0 * wavesynth::kPi * kInputLpHz * dtSec);
        const QuarterCar::K k = QuarterCar::params(p.bodyHz, p.hopHz, p.damping);
        const double bodyMm = std::max(0.05, p.bodyMm);
        const double aFull  = bodyMm * std::pow(2.0 * wavesynth::kPi * kForceRefHz, 2.0);
        for (int i = 0; i < WHEEL_COUNT; ++i)
        {
            W& w = m_w[i];
            // The real road where the sim sends one, else the suspension.
            double real = 0.0;
            if (w.heightDriven)
            {
                if (!w.hpHSeen) { w.hpHPrev = w.h; w.hpH = 0.0; w.hpHSeen = true; }
                w.hpH = roadA * (w.hpH + w.h - w.hpHPrev);
                w.hpHPrev = w.h;
                real = w.hpH;
            }
            else
            {
                w.hpHSeen = false;
                real = w.x;
            }
            w.realF += (real - w.realF) * inA;
            const double syn = roughAt(i, dtSec, p, wb);
            w.zrF += (w.realF + syn - w.zrF) * envA;
            w.qc.step(dtSec, w.zrF, k);
            // Above cut hz: what the motion cue does not already do.
            w.zsHp = outA * (w.zsHp + w.qc.zs - w.zsPrev); w.zsPrev = w.qc.zs;
            w.asHp = outA * (w.asHp + w.qc.as - w.asPrev); w.asPrev = w.qc.as;
            w.out.pos   = softKnee(w.zsHp / bodyMm);
            w.out.force = softKnee(w.asHp / aFull);
            w.mm = w.zsHp;
        }
    }

    void stepChassis(double dtSec, const RoadParams& p, double hpHz, double leak)
    {
        using namespace road_k;
        const double outA = std::exp(-2.0 * wavesynth::kPi * hpHz * dtSec);
        const double accA = std::exp(-2.0 * wavesynth::kPi * 0.5 * hpHz * dtSec);
        const double bodyMm = std::max(0.05, p.bodyMm);
        const double aFull  = bodyMm * std::pow(2.0 * wavesynth::kPi * kForceRefHz, 2.0);
        if (m_chassisDriven)
        {
            // The held ~60 Hz samples smoothed first.
            const double inA = 1.0 - std::exp(-2.0 * wavesynth::kPi * kInputLpHz * dtSec);
            if (!m_chassisSeen) { m_accF = m_acc; m_rollF = m_roll; m_pitchF = m_pitch; }
            m_accF   += (m_acc   - m_accF)   * inA;
            m_rollF  += (m_roll  - m_rollF)  * inA;
            m_pitchF += (m_pitch - m_pitchF) * inA;
            const double a = m_accF * 1000.0;   // mm/s^2
            if (!m_chassisSeen) { m_aPrev = a; m_rollPrev = m_rollF; m_pitchPrev = m_pitchF; m_chassisSeen = true; }
            // Heave: gravity and drift off, then integrated twice (each
            // integral leaking at cut hz, so only the band above stays).
            m_aHp = accA * (m_aHp + a - m_aPrev); m_aPrev = a;
            m_vh = m_vh * leak + m_aHp * dtSec;
            m_zh = m_zh * leak + m_vh * dtSec;
            m_rollHp  = outA * (m_rollHp  + m_rollF  - m_rollPrev);  m_rollPrev  = m_rollF;
            m_pitchHp = outA * (m_pitchHp + m_pitchF - m_pitchPrev); m_pitchPrev = m_pitchF;
        }
        else
        {
            m_chassisSeen = false;
            m_vh *= leak; m_zh *= leak; m_aHp *= leak; m_rollHp *= leak; m_pitchHp *= leak;
        }
        learnSigns(dtSec);
        const double halfWb = 500.0 * ((m_wheelbase > 0.0) ? m_wheelbase : kDefaultWheelbaseM);   // mm
        const double halfTr = 500.0 * ((m_track > 0.0) ? m_track : kDefaultTrackM);
        const double roll  = m_rollHp  * wavesynth::kPi / 180.0;
        const double pitch = m_pitchHp * wavesynth::kPi / 180.0;
        for (int i = 0; i < WHEEL_COUNT; ++i)
        {
            W& w = m_w[i];
            const double xw = (i == WheelFL || i == WheelFR) ? halfWb : -halfWb;
            const double yw = (i == WheelFL || i == WheelRL) ? halfTr : -halfTr;
            const double z = m_signH * m_zh + m_signR * roll * yw + m_signP * pitch * xw;
            w.out.pos   = softKnee(z / bodyMm);
            w.out.force = softKnee(m_signH * m_aHp / aFull);
            w.mm = z;
        }
    }

    // Pitch and roll against the corners, from the per-wheel suspension.
    // The spring is what moves the body: k x compression = m x body
    // acceleration = -m w^2 x body displacement, so a corner's body
    // displacement runs OPPOSITE to its suspension compression, over a bump
    // and in a corner alike. Heave is as sent (the protocol: up positive);
    // its correlation with the mean suspension gives the suspension's own
    // polarity (compression or extension positive), and roll and pitch take
    // their signs against the side and end differences through it. Until
    // the polarity is known: heave only.
    void learnSigns(double dtSec)
    {
        using namespace road_k;
        bool susp = true;
        for (const W& w : m_w) susp = susp && w.suspDriven;
        if (!susp || !m_chassisDriven) return;
        const double a  = std::min(1.0, dtSec / kLearnSec);
        const double M  = 0.25 * (m_w[0].x + m_w[1].x + m_w[2].x + m_w[3].x);
        const double LR = 0.5 * (m_w[WheelFL].x + m_w[WheelRL].x - m_w[WheelFR].x - m_w[WheelRR].x);
        const double FB = 0.5 * (m_w[WheelFL].x + m_w[WheelFR].x - m_w[WheelRL].x - m_w[WheelRR].x);
        const auto upd = [a](double& c, double& e1, double& e2, double x1, double x2)
        { c += a * (x1 * x2 - c); e1 += a * (x1 * x1 - e1); e2 += a * (x2 * x2 - e2); };
        upd(m_cH, m_eH, m_eM,  m_zh,      M);
        upd(m_cR, m_eR, m_eLR, m_rollHp,  LR);
        upd(m_cP, m_eP, m_eFB, m_pitchHp, FB);
        // +1 / -1 for a clear correlation, 0 for none.
        const auto sign = [](double c, double e1, double e2) -> int
        {
            const double d = std::sqrt(e1 * e2);
            if (d < 1e-12) return 0;
            const double r = c / d;
            return (r > kLearnCorr) ? 1 : (r < -kLearnCorr) ? -1 : 0;
        };
        const int pol = sign(m_cH, m_eH, m_eM);
        if (pol != 0) m_pol = pol;
        if (m_pol == 0) return;
        const int r = sign(m_cR, m_eR, m_eLR), p = sign(m_cP, m_eP, m_eFB);
        if (r != 0) m_signR = r * m_pol;
        if (p != 0) m_signP = p * m_pol;
    }

    // The random road: a leaky random walk on a distance grid (an ISO 8608
    // slope of -2), common to both sides in part.
    void generateTo(double distM)
    {
        using namespace road_k;
        const int64_t need = static_cast<int64_t>(std::floor(distM / kProfileStepM)) + 1;
        if (need - m_genIdx > kProfileN) m_genIdx = need - kProfileN;   // a jump: start the window fresh
        const double keep = 1.0 - kProfileStepM / kProfileLeakM;
        const double sd   = kIsoSigmaMm * std::sqrt(kProfileStepM);
        const double cw = std::sqrt(kLRCorr), iw = std::sqrt(1.0 - kLRCorr);
        while (m_genIdx < need)
        {
            ++m_genIdx;
            m_c  = m_c  * keep + sd * gauss();
            m_iL = m_iL * keep + sd * gauss();
            m_iR = m_iR * keep + sd * gauss();
            const int64_t slot = m_genIdx % kProfileN;
            m_prof[0][slot] = static_cast<float>(cw * m_c + iw * m_iL);
            m_prof[1][slot] = static_cast<float>(cw * m_c + iw * m_iR);
        }
    }
    double gauss()
    {
        double s = 0.0;
        for (int i = 0; i < 4; ++i)
        {
            m_rng ^= m_rng << 13; m_rng ^= m_rng >> 7; m_rng ^= m_rng << 17;
            s += static_cast<double>(m_rng & 0xFFFFFF) / 16777215.0;
        }
        return (s - 2.0) * 1.7320508075688772;   // unit variance
    }

    W m_w[WHEEL_COUNT];
    // tyre model
    double  m_speed = 0.0, m_dist = 0.0;
    bool    m_speedDriven = false;
    double  m_wheelbase = 0.0, m_track = 0.0;
    double  m_smooth = 0.7;          // water fills the texture (SurfaceParams::smooth)
    int64_t m_genIdx = -1;
    double  m_c = 0.0, m_iL = 0.0, m_iR = 0.0;
    float   m_prof[2][road_k::kProfileN] = {};
    uint64_t m_rng = 0x2545F4914F6CDD1Dull;
    // chassis model
    double m_acc = 0.0, m_pitch = 0.0, m_roll = 0.0;
    double m_accF = 0.0, m_pitchF = 0.0, m_rollF = 0.0;
    bool   m_chassisDriven = false, m_chassisSeen = false;
    double m_aHp = 0.0, m_aPrev = 0.0, m_vh = 0.0, m_zh = 0.0;
    double m_rollHp = 0.0, m_rollPrev = 0.0, m_pitchHp = 0.0, m_pitchPrev = 0.0;
    double m_cH = 0.0, m_eH = 0.0, m_eM = 0.0, m_cR = 0.0, m_eR = 0.0, m_eLR = 0.0, m_cP = 0.0, m_eP = 0.0, m_eFB = 0.0;
    int    m_signH = 1, m_signR = 0, m_signP = 0;
    int    m_pol = 0;   // the suspension against the body: -1 compression positive, +1 extension positive, 0 unknown
};

// ============================================================
// KerbModel - a rumble strip under the tyre that is on one.
//
// Ribs every pitch, laid out by distance, so they hum at speed / pitch
// (60 to 110 Hz at racing speeds) and the hum rises and falls with speed;
// each rib a little different in height (jitter); the step up onto the
// kerb at rise. All through the tyre's envelope. A seat post can carry the
// step (a thud on and off) but not the hum: it gets the kerb's height,
// smoothed, above kKerbPosHpHz (riding along a kerb does not hold the seat
// up; the motion cue has the car's own lift where the sim models it). A belt or shaker gets the ribs and the step's edges (a kick
// on, a kick off), stronger with speed, which is the on/off sound of a
// kerb. Abrupt in and out: a tyre is on a kerb or it is not.
// ============================================================
class KerbModel
{
public:
    // Per wheel, per cycle, BEFORE step(): how much of the tyre is on a
    // kerb (0..1). Not driven = coming off.
    void drive(int wheel, double level)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        m_w[wheel].target = std::max(0.0, std::min(1.0, std::isfinite(level) ? level : 0.0));
    }
    void driveSpeed(double speedMs) { m_speed = std::isfinite(speedMs) ? std::max(0.0, speedMs) : 0.0; }

    void step(double dtSec, const EffectParams& p, const KerbParams& k)
    {
        using namespace road_k;
        if (dtSec <= 0.0) return;
        const double v = m_speed;
        const double pitchM = std::max(0.05, k.pitchCm / 100.0);
        const double fc = std::max(5.0, std::min(0.45 / dtSec, v / kPatchM));
        const double envA = 1.0 - std::exp(-2.0 * wavesynth::kPi * fc * dtSec);
        const double smA  = 1.0 - std::exp(-2.0 * wavesynth::kPi * kSmoothHz * dtSec);
        const double hpA  = std::exp(-2.0 * wavesynth::kPi * kKerbForceHpHz * dtSec);
        const double posA = std::exp(-2.0 * wavesynth::kPi * kKerbPosHpHz * dtSec);
        const double full = std::max(kMinFullMm, k.fullMm);
        const double ribRef = std::max(0.5, k.ribMm);
        const double speedK = std::max(0.2, std::min(1.5, v / kKerbRefSpeedMs));
        const double jit = std::max(0.0, std::min(1.0, p.jitter));
        for (W& w : m_w)
        {
            const double rate = (w.target > w.lvl) ? dtSec / kKerbAttackSec : dtSec / kKerbReleaseSec;
            w.lvl += std::max(-rate, std::min(rate, w.target - w.lvl));
            w.target = 0.0;
            // Ribs pass under the tyre with the distance covered.
            w.phase += 2.0 * wavesynth::kPi * v * dtSec / (pitchM * w.pitchK);
            while (w.phase >= 2.0 * wavesynth::kPi)
            {
                w.phase -= 2.0 * wavesynth::kPi;
                w.ribH   = 1.0 - 0.4 * jit * rand01(w.rng);
                w.pitchK = 1.0 + 0.2 * jit * (rand01(w.rng) - 0.5);
            }
            const double rib = 0.5 * (1.0 - std::cos(w.phase));
            const double z = w.lvl * (k.riseMm + k.ribMm * w.ribH * rib);   // mm
            w.zf += (z - w.zf) * envA;
            w.zs += (w.zf - w.zs) * smA;
            w.zp = posA * (w.zp + w.zs - w.zsPrev); w.zsPrev = w.zs;
            w.hp = hpA * (w.hp + w.zf - w.zfPrev); w.zfPrev = w.zf;
            w.out.pos   = roadSoftKnee(w.zp / full);
            w.out.force = roadSoftKnee(w.hp / ribRef * speedK);
            w.mm = w.zp;
        }
    }

    RoadOut outputFor(Part part) const
    {
        int first, last; partWheels(part, first, last);
        RoadOut o;
        for (int i = first; i <= last; ++i)
        {
            if (std::fabs(m_w[i].out.pos)   > std::fabs(o.pos))   o.pos   = m_w[i].out.pos;
            if (std::fabs(m_w[i].out.force) > std::fabs(o.force)) o.force = m_w[i].out.force;
        }
        return o;
    }
    double level() const
    {
        double m = 0.0;
        for (const W& w : m_w) m = std::max(m, w.lvl);
        return m;
    }
    double wheelLevel(int wheel) const { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].lvl : 0.0; }
    double wheelMm(int wheel) const    { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].mm : 0.0; }
    bool   active() const
    {
        for (const W& w : m_w) if (w.lvl > 1e-4 || std::fabs(w.zp) > 1e-3 || std::fabs(w.hp) > 1e-3) return true;
        return false;
    }
    void clear()
    {
        uint64_t seed = 0x9E3779B97F4A7C15ull;
        for (W& w : m_w) { w = W{}; w.rng = seed; seed = seed * 6364136223846793005ull + 1442695040888963407ull; }
        m_speed = 0.0;
    }
    KerbModel() { clear(); }

private:
    static double rand01(uint64_t& r)
    {
        r ^= r << 13; r ^= r >> 7; r ^= r << 17;
        return static_cast<double>(r & 0xFFFFFF) / 16777215.0;
    }
    struct W
    {
        double target = 0.0, lvl = 0.0, phase = 0.0, ribH = 1.0, pitchK = 1.0;
        double zf = 0.0, zs = 0.0, zsPrev = 0.0, zp = 0.0, hp = 0.0, zfPrev = 0.0, mm = 0.0;
        RoadOut out;
        uint64_t rng = 0x9E3779B97F4A7C15ull;
    };
    W m_w[WHEEL_COUNT];
    double m_speed = 0.0;
};

} // namespace haptics
