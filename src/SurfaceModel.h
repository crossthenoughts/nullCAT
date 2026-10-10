// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// SurfaceModel - the Surface tile: what the ground does under each tyre
// beyond the road's shape (the Road tile carries that, its roughness
// shaped per surface class and smoothed by water from here).
//
//   stones   gravel, sand, dirt: stones struck by the tyre at random, so
//            many per metre of travel (busier with speed), each a size of
//            its own and now and then a big one: a sharp tick on a belt or
//            shaker, a small bump through the tyre on a post;
//   crunch   snow compacting under the tread (and sand, and a lower mud
//            squelch): granular noise, rising with speed;
//   studs    ice and snow with studded tyres: the studs biting, a rough
//            buzz at speed over the stud spacing;
//   puddles  on a wet road, puddles laid out along it per side (the rears
//            meet a puddle a wheelbase after the fronts): the water drags
//            at the tyre as it enters (a tug, stronger with speed squared);
//            deep enough and fast enough the tyre floats on it
//            (aquaplaning: the road under it goes quiet), and bites again
//            as it comes out.
//
// Per wheel, per cycle the law drives the surface class, the water on the
// road under the tyre (0 dry .. 1 standing water) and the road speed. Each
// wheel yields a FORCE sample (belts, shakers) and a POSITION sample
// (posts), and the water the Road tile should smooth by.
//
// RT-safe: fixed arrays and pools, pure arithmetic, no allocation.
// ============================================================

#include "HapticsTypes.h"
#include "NcxTokens.h"      // Wheel order, SurfaceClass
#include "WaveSynth.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace haptics {

namespace surface_k {
    // Stones struck per metre of travel under one tyre, by class (tarmac,
    // bumpy, kerb, gravel, grass, dirt, cobbles, snow, ice, sand, mud, water).
    constexpr double kStonesPerM[SURFACE_CLASS_COUNT] = { 0.0, 0.05, 0.0, 2.0, 0.1, 0.4, 0.0, 0.0, 0.0, 0.6, 0.2, 0.0 };
    constexpr double kBigStoneShare = 0.12;   // one stone in eight is a big one...
    constexpr double kBigStoneK     = 2.5;    // ...this much bigger
    constexpr double kTickMinHz = 60.0, kTickMaxHz = 110.0;   // a stone's ring on the floor
    constexpr double kTickSec   = 0.008;      // ...dying away this fast
    constexpr double kBumpM     = 0.05;       // a stone lifts the tyre over this much road
    constexpr double kBumpMm    = 1.0;        // ...by this much, a typical stone
    constexpr double kPosFullMm = 3.0;        // a post's 100%
    constexpr int    kPool      = 6;          // stones live at once per wheel
    // Crunch by class; snow full, sand light, mud (its own lower band).
    constexpr double kCrunch[SURFACE_CLASS_COUNT] = { 0.0, 0.0, 0.0, 0.15, 0.0, 0.1, 0.0, 1.0, 0.0, 0.35, 0.6, 0.0 };
    constexpr double kCrunchLoHz = 25.0, kCrunchHiHz = 60.0;
    constexpr double kMudLoHz    = 12.0, kMudHiHz    = 30.0;
    constexpr double kCrunchFullMs = 16.7;    // full by 60 km/h
    // Studs: rows passing the contact patch.
    constexpr double kStudSpacingM = 0.25;
    constexpr double kStudMinHz = 20.0, kStudMaxHz = 100.0;
    constexpr double kStudJitter = 0.4;
    // Puddles.
    constexpr double kCellM     = 4.0;        // one chance of a puddle per this much road, per side
    constexpr double kPuddleChance = 0.6;     // x wet squared
    constexpr double kPuddleMinM = 1.0, kPuddleMaxM = 3.5;
    constexpr double kEdgeM     = 0.3;        // a puddle's edge
    constexpr double kFilm      = 0.3;        // a wet road's film, as a share of standing water
    constexpr double kTugRefMs  = 25.0;       // the drag tug at full strength at 90 km/h
    constexpr double kTugSec    = 0.06, kBiteSec = 0.04;
    constexpr double kAquaDepth = 0.3;        // water this deep or more can float a tyre
    constexpr double kFloatInSec = 0.05, kFloatOutSec = 0.03;
    constexpr double kDefaultWheelbaseM = 2.7;
    constexpr double kMinSpeedMs = 1.0;
    constexpr double kGroundEaseSec = 0.02;   // a tyre leaving or meeting the ground eases over this
}

class SurfaceModel
{
public:
    SurfaceModel() { clear(); }

    // ---- inputs, per cycle, BEFORE step() ----
    void driveSurface(int wheel, int cls)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        m_w[wheel].cls = (cls >= 0 && cls < SURFACE_CLASS_COUNT) ? cls : SurfTarmac;
    }
    // Water on the road under the tyre: 0 dry, 1 standing water.
    void driveWet(int wheel, double wet)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        m_w[wheel].wet = std::isfinite(wet) ? std::max(0.0, std::min(1.0, wet)) : 0.0;
    }
    // Road speed, m/s. Not driven this cycle = standing still (a stream
    // that stops must not keep stones flying).
    void driveSpeed(double speedMs) { m_speed = std::isfinite(speedMs) ? std::max(0.0, speedMs) : 0.0; m_speedDriven = true; }
    void driveGeometry(double wheelbaseM) { if (wheelbaseM > 1.0 && wheelbaseM < 6.0) m_wheelbase = wheelbaseM; }
    // How far the tyre touches the ground, 0 in the air .. 1 (not driven this
    // cycle = on the ground): a tyre in the air strikes no stones, crunches
    // nothing and meets no puddle.
    void driveGround(int wheel, double contact)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        m_w[wheel].groundIn = std::isfinite(contact) ? std::max(0.0, std::min(1.0, contact)) : 1.0;
    }

    void step(double dtSec, const SurfaceParams& q)
    {
        using namespace surface_k;
        if (dtSec <= 0.0) return;
        const double v = m_speedDriven ? m_speed : 0.0;
        m_speedDriven = false;
        if (v > kMinSpeedMs) m_dist += v * dtSec;
        const double wb = (m_wheelbase > 0.0) ? m_wheelbase : kDefaultWheelbaseM;
        const double crunchK = std::min(1.0, v / kCrunchFullMs);
        const double studHz = std::max(kStudMinHz, std::min(kStudMaxHz, v / kStudSpacingM));
        const double aquaMs = std::max(5.0, q.aquaKmh / 3.6);
        m_level = 0.0;
        for (int i = 0; i < WHEEL_COUNT; ++i)
        {
            W& w = m_w[i];
            double force = 0.0, posMm = 0.0;
            w.groundF += (w.groundIn - w.groundF) * std::min(1.0, dtSec / kGroundEaseSec);
            w.groundIn = 1.0;
            const bool touching = w.groundF >= 0.5;

            // Stones: a Poisson stream, so many per metre.
            const double rate = kStonesPerM[w.cls] * v;        // per second
            if (touching && rate > 0.0 && q.stones > 0.0 && rand01(w.rng) < rate * dtSec)
            {
                Stone& s = w.pool[w.next]; w.next = (w.next + 1) % kPool;
                s.t = 0.0;
                s.amp = (0.3 + 0.7 * rand01(w.rng)) * ((rand01(w.rng) < kBigStoneShare) ? kBigStoneK : 1.0);
                s.hz = kTickMinHz + (kTickMaxHz - kTickMinHz) * rand01(w.rng);
                s.bumpSec = std::max(0.01, kBumpM / std::max(kMinSpeedMs, v));
                s.live = true;
                ++w.struck;
            }
            for (Stone& s : w.pool)
            {
                if (!s.live) continue;
                s.t += dtSec;
                const double tick = s.amp * std::exp(-s.t / kTickSec) * std::sin(2.0 * wavesynth::kPi * s.hz * s.t);
                const double bump = (s.t < s.bumpSec) ? s.amp * kBumpMm * 0.5 * (1.0 - std::cos(2.0 * wavesynth::kPi * s.t / s.bumpSec)) : 0.0;
                force += q.stones * tick;
                posMm += q.stones * bump;
                if (s.t > std::max(8.0 * kTickSec, s.bumpSec)) s.live = false;
            }

            // Crunch: granular noise in the class's band.
            const double cr = kCrunch[w.cls] * q.crunch * crunchK;
            const bool mud = (w.cls == SurfMud);
            const double lo = mud ? kMudLoHz : kCrunchLoHz, hi = mud ? kMudHiHz : kCrunchHiHz;
            const double n = 2.0 * rand01(w.rng) - 1.0;
            const double hpA = std::exp(-2.0 * wavesynth::kPi * lo * dtSec);
            const double lpA = 1.0 - std::exp(-2.0 * wavesynth::kPi * hi * dtSec);
            w.nLp += (n - w.nLp) * lpA;
            w.nHp = hpA * (w.nHp + w.nLp - w.nPrev); w.nPrev = w.nLp;
            // The band keeps only a small share of the white noise's spread;
            // scaled back up so 1 x is a clear crunch.
            if (cr > 0.0) force += cr * 4.0 * w.nHp;

            // Studs on ice and snow.
            if (q.studs >= 0.5 && (w.cls == SurfIce || w.cls == SurfSnow) && v > kMinSpeedMs)
            {
                const double f = studHz * (1.0 + kStudJitter * (rand01(w.rng) - 0.5));
                force += 0.6 * std::min(1.0, v / kCrunchFullMs) * w.studOsc.step(f, dtSec);
            }

            // Puddles: the water under this tyre where it is now.
            const bool front = (i == WheelFL || i == WheelFR);
            const int side = (i == WheelFL || i == WheelRL) ? 0 : 1;
            const double x = front ? m_dist : m_dist - wb;
            double full = 0.0;
            const double puddle = (w.wet > 0.0 && x > 0.0) ? puddleDepth(side, x, w.wet, &full) : 0.0;
            const double water = std::max(kFilm * w.wet, puddle);
            // Into a puddle: the drag tug, with speed squared, as deep as the
            // puddle is (not its edge, where the tyre crosses in).
            if (touching && puddle > 0.1 && w.puddlePrev <= 0.1 && v > kMinSpeedMs)
            {
                w.tugT = 0.0;
                w.tugAmp = q.puddles * full * std::min(1.5, (v / kTugRefMs) * (v / kTugRefMs));
            }
            w.puddlePrev = puddle;
            if (w.tugT >= 0.0)
            {
                w.tugT += dtSec;
                if (w.tugT < kTugSec)
                {
                    const double s = w.tugAmp * std::sin(wavesynth::kPi * w.tugT / kTugSec);
                    force -= s;
                    posMm -= 0.5 * kBumpMm * s;
                }
                else w.tugT = -1.0;
            }
            // Aquaplaning: deep enough water above the speed it floats at
            // (lower in deeper water: aqua km/h is for standing water).
            const double floatAt = aquaMs * (1.0 + (1.0 - water));
            const bool floats = (water >= kAquaDepth) && (v > floatAt);
            const double fr = floats ? dtSec / kFloatInSec : -dtSec / kFloatOutSec;
            const double was = w.floatLvl;
            w.floatLvl = std::max(0.0, std::min(1.0, w.floatLvl + fr));
            // ...and the bite as it comes out of the water.
            if (was > 0.5 && w.floatLvl <= 0.5)
            {
                w.biteT = 0.0;
                w.biteAmp = q.puddles * std::min(1.0, v / kTugRefMs);
            }
            if (w.biteT >= 0.0)
            {
                w.biteT += dtSec;
                if (w.biteT < kBiteSec)
                {
                    const double s = w.biteAmp * std::sin(wavesynth::kPi * w.biteT / kBiteSec);
                    force += s;
                    posMm += 0.5 * kBumpMm * s;
                }
                else w.biteT = -1.0;
            }
            w.water = water;

            w.out.force = knee(force) * w.groundF;
            w.out.pos   = knee(posMm / kPosFullMm) * w.groundF;
            const double a = std::max(std::fabs(w.out.force), std::fabs(w.out.pos));
            w.env = std::max(a, w.env * std::exp(-dtSec / 0.15));
            m_level = std::max(m_level, w.env);
        }
    }

    struct Out { double pos = 0.0, force = 0.0; };
    // The strongest corner of the part, per sink kind.
    Out outputFor(Part part) const
    {
        int first = 0, last = WHEEL_COUNT - 1;
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
        Out o;
        for (int i = first; i <= last; ++i)
        {
            if (std::fabs(m_w[i].out.pos)   > std::fabs(o.pos))   o.pos   = m_w[i].out.pos;
            if (std::fabs(m_w[i].out.force) > std::fabs(o.force)) o.force = m_w[i].out.force;
        }
        return o;
    }

    // For the Road tile: the water under each tyre (0..1) and how far the
    // tyre floats on it (0..1).
    double water(int wheel) const    { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].water : 0.0; }
    double floating(int wheel) const { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].floatLvl : 0.0; }
    double wetness(int wheel) const  { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].wet : 0.0; }   // as driven
    double level() const             { return m_level; }
    uint64_t stonesStruck(int wheel) const { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].struck : 0; }
    double distance() const          { return m_dist; }

    // The puddle under one side at distance x (0 none .. depth, its edges
    // ramped; *full = the puddle's depth away from its edges): laid out per
    // cell from a hash of the cell, so it is the same puddle every time a
    // wheel passes that bit of road.
    static double puddleDepth(int side, double x, double wet, double* full = nullptr)
    {
        using namespace surface_k;
        const int64_t cell = static_cast<int64_t>(std::floor(x / kCellM));
        uint64_t h = static_cast<uint64_t>(cell) * 0x9E3779B97F4A7C15ull ^ (side ? 0xD1B54A32D192ED03ull : 0x2545F4914F6CDD1Dull);
        const double u1 = hash01(h), u2 = hash01(h), u3 = hash01(h), u4 = hash01(h);
        if (full) *full = 0.0;
        if (u1 >= kPuddleChance * wet * wet) return 0.0;
        const double len = kPuddleMinM + (kPuddleMaxM - kPuddleMinM) * u3;
        const double start = cell * kCellM + u2 * (kCellM - len);
        const double in = x - start;
        if (in < 0.0 || in > len) return 0.0;
        const double edge = std::min(1.0, std::min(in, len - in) / kEdgeM);
        const double depth = wet * (0.4 + 0.6 * u4);
        if (full) *full = depth;
        return edge * depth;
    }

    void clear()
    {
        uint64_t seed = 0x6A09E667F3BCC909ull;
        for (W& w : m_w) { w = W{}; w.rng = seed; seed = seed * 6364136223846793005ull + 1442695040888963407ull; }
        m_dist = 0.0; m_speed = 0.0; m_level = 0.0; m_speedDriven = false;
    }

private:
    static double rand01(uint64_t& r)
    {
        r ^= r << 13; r ^= r >> 7; r ^= r << 17;
        return static_cast<double>(r & 0xFFFFFF) / 16777215.0;
    }
    static double hash01(uint64_t& h)
    {
        h += 0x9E3779B97F4A7C15ull;
        uint64_t z = h;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        z ^= z >> 31;
        return static_cast<double>(z >> 11) / 9007199254740992.0;
    }
    // Linear to 0.6, then bending towards 1.
    static double knee(double x)
    {
        const double a = std::fabs(x);
        if (a <= 0.6) return x;
        const double y = 0.6 + 0.4 * std::tanh((a - 0.6) / 0.4);
        return (x < 0.0) ? -y : y;
    }
    struct Stone { bool live = false; double t = 0.0, amp = 0.0, hz = 0.0, bumpSec = 0.0; };
    struct W
    {
        int cls = SurfTarmac;
        double wet = 0.0, water = 0.0;
        Stone pool[surface_k::kPool];
        int next = 0;
        double nLp = 0.0, nHp = 0.0, nPrev = 0.0;
        wavesynth::Oscillator studOsc;
        double puddlePrev = 0.0, tugT = -1.0, tugAmp = 0.0, biteT = -1.0, biteAmp = 0.0, floatLvl = 0.0;
        double groundIn = 1.0, groundF = 1.0;   // touching the ground (driven, eased): 0 in the air
        Out out;
        double env = 0.0;
        uint64_t rng = 1;
        uint64_t struck = 0;
    };
    W m_w[WHEEL_COUNT];
    double m_dist = 0.0, m_speed = 0.0, m_wheelbase = 0.0, m_level = 0.0;
    bool   m_speedDriven = false;
};

} // namespace haptics
