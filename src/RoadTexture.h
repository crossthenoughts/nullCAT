// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// RoadTexture - the Road tile's texture for shakers: the fine grain of the
// road running under each tyre, above the band the Road models play.
//
// The models (RoadModel.h) play how the suspension and the body MOVE: a
// position, smoothed above 20 Hz, which a seat post can follow and a shaker
// can do next to nothing with. What a cabin feels of a road at speed is
// finer: a random buzz from about 25 Hz up, stronger the faster the car
// goes, different under each wheel. Two sources, per wheel:
//
//   game    the sim's own suspension speed in the band (texture low hz to
//           texture high hz, two second-order cuts at each end). A damper
//           pushes on the body in proportion to that speed, so the speed
//           itself is the force sample. Only as far up as the sender's
//           samples can carry: kRateShare of the rate they arrive at, read
//           from how often the corners' values change. A sender too slow
//           for an octave of the band gives nothing (60 samples a second
//           cannot hold 25 Hz and up, and 90 hold too little of it: on the
//           recordings such senders sat on the edge and the source kept
//           changing). Only from a rolling car: a car dropped onto its
//           suspension as it loads in is not road.
//   speed   made up: noise in the same band (one second-order cut at each
//           end), its level rising with road speed (full by texture km/h),
//           each wheel its own, x the surface under the tyre (less what
//           water fills, nothing from a tyre floating on it or in the air).
//
// texture from: auto takes the game's where the sender is fast enough and
// its suspension is arriving, the made-up one otherwise; game and speed
// force one. A Test preview plays the made-up one at the preview speed.
//
// A big slow bump leaves a little hash in the band: the loop sees each of
// the sender's samples up to one cycle late, and a late step is noise at
// every pitch. About a hundredth of the bump at a 2 kHz loop, about four
// times that at 500 Hz.
//
// Out: per wheel a force sample -1..1 (soft knee), for shaker routes only.
// Nothing is learned about the car. The sender's rate is kept while its
// suspension arrives and read afresh after kRateForgetSec without it, and
// for another game (forgetSender).
//
// RT-safe: fixed arrays, pure arithmetic, no allocation.
// ============================================================

#include "HapticsTypes.h"
#include "NcxTokens.h"      // Wheel order, SurfaceClass
#include "RoadModel.h"      // HighPass2, LowPass2, roadSoftKnee, partWheels, roadShortGain
#include "WaveSynth.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace haptics {

namespace roadtex_k {
    // Suspension speed in the band that is 100 % at texture x 1. Measured
    // on the recordings (replay through this struct, .local/replay/tex.cpp):
    // a circuit car at speed runs 8 to 16 mm/s rms in 25 to 90 Hz with
    // kerbs and joints several times that, so x 1 sits near 0.1 rms with
    // room above it.
    constexpr double kFullMmS      = 100.0;
    constexpr double kMadeRms      = 0.12;    // the made-up texture's rms at full speed on tarmac, at texture x 1 (the game's, as above)
    constexpr double kRateShare    = 0.4;     // the game's detail is taken up to this share of its sample rate
    constexpr double kMinTopRatio  = 2.0;     // ...and counts as carried when that reaches this x texture low hz (an octave of band: 125 samples a second at 25 Hz)
    constexpr double kUsableHyst   = 0.8;     // (once carried, until it falls under this share of that: the rate read wanders with how the samples arrive)
    constexpr int    kRateMinSamples = 16;    // changes seen before the rate is believed
    constexpr double kRateEma      = 0.01;    // the share one more interval moves it by (a hundred samples: arrivals come in bursts)
    constexpr double kRateMinSec   = 0.0002;  // an interval is taken within these...
    constexpr double kRateGapSec   = 0.05;    // ...a longer one is a pause, not a sample period
    constexpr double kRateForgetSec = 2.0;    // no suspension for this long: the next sender may be another
    constexpr double kAttackSec    = 0.050;   // a source coming in
    constexpr double kReleaseSec   = 0.120;   // ...and going (the stream stopping, the other source taking over)
    constexpr double kRollFromKmh  = 5.0;     // the game's own: nothing from a car slower than this...
    constexpr double kRollFullKmh  = 15.0;    // ...all of it from this on
    // Floors for a value that got past the settings' own ranges
    // (HapticsRegistry.h), not those ranges.
    constexpr double kMinLoHz      = 5.0;
    constexpr double kMinBandRatio = 1.2;     // texture high hz is at least this x texture low hz
    constexpr double kMinKmhFull   = 20.0;
    constexpr double kNoiseBw      = 1.1107;  // a second-order Butterworth edge's noise bandwidth, x its corner (pi / (2 sqrt 2))
    constexpr double kTravelStaleSec = 0.1;   // a travel sender: no change for this long is no movement
}

class RoadTexture
{
public:
    // What is playing (the tile's line): nothing asked for, the game's own,
    // the made-up one, or asked for with nothing to make it from.
    enum Source { Off = 0, Game = 1, Made = 2, Waiting = 3 };
    // ...by name, as the status carries it (the page reads the name).
    static const char* sourceName(int s)
    {
        return s == Game ? "game" : s == Made ? "made" : s == Waiting ? "waiting" : "off";
    }

    // ---- inputs, per cycle, BEFORE step() ----
    // Suspension speed, mm/s, signed.
    void drive(int wheel, double velMmS)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT || !std::isfinite(velMmS)) return;
        W& w = m_w[wheel];
        w.raw = w.v = velMmS;
        w.haveTravel = false;
        w.driven = true;
    }
    // ...or suspension travel, mm, for sims that give no speed: the speed
    // between one change of the value and the next. The time between the
    // two is what the loop saw (up to a cycle out), or, once the sender's
    // rate is believed, the nearest whole number of its periods.
    void driveTravel(int wheel, double travelMm)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT || !std::isfinite(travelMm)) return;
        W& w = m_w[wheel];
        if (!w.haveTravel) { w.v = 0.0; w.sinceTravel = 0.0; }
        else if (travelMm != w.raw)
        {
            double span = w.sinceTravel;
            if (m_rateN >= roadtex_k::kRateMinSamples && m_interval > 0.0)
                span = std::max(1.0, std::floor(span / m_interval + 0.5)) * m_interval;
            w.v = (span > 0.0) ? (travelMm - w.raw) / span : 0.0;
            w.sinceTravel = 0.0;
        }
        w.raw = travelMm;
        w.haveTravel = true;
        w.driven = true;
    }
    void driveSpeed(double speedMs)
    {
        m_speed = std::isfinite(speedMs) ? std::max(0.0, speedMs) : 0.0;
        m_speedDriven = true;
    }
    // A Test preview: the made-up texture at this speed, whatever is set.
    void drivePreview(double speedMs)
    {
        driveSpeed(speedMs);
        m_preview = true;
    }
    void driveSurface(int wheel, int cls)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        m_w[wheel].surface = (cls >= 0 && cls < SURFACE_CLASS_COUNT) ? cls : SurfTarmac;
    }
    // How far the tyre touches the ground, 0 in the air .. 1 (not driven
    // this cycle = on the ground).
    void driveGround(int wheel, double contact)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        m_w[wheel].groundIn = std::isfinite(contact) ? std::max(0.0, std::min(1.0, contact)) : 1.0;
    }
    // The water under the tyre (0..1) and how far it floats on it (0..1),
    // from the Surface tile; smooth: how far water fills the texture.
    void driveWater(int wheel, double water, double floating)
    {
        if (wheel < 0 || wheel >= WHEEL_COUNT) return;
        m_w[wheel].water    = std::isfinite(water)    ? std::max(0.0, std::min(1.0, water))    : 0.0;
        m_w[wheel].floating = std::isfinite(floating) ? std::max(0.0, std::min(1.0, floating)) : 0.0;
    }
    void setWaterSmooth(double smooth) { m_smooth = std::isfinite(smooth) ? std::max(0.0, std::min(1.0, smooth)) : 0.0; }

    void step(double dtSec, const RoadParams& p)
    {
        using namespace roadtex_k;
        if (dtSec <= 0.0) return;

        // The sender's rate, whether or not the texture is on (the tile
        // says what it could carry before it is turned up).
        bool fresh = false, anySusp = false;
        for (W& w : m_w)
        {
            if (!w.driven)
            {
                // Not sent this cycle: what comes next is a first value
                // again (the jump across a gap is not a speed).
                w.haveTravel = false; w.rawSeen = false;
                continue;
            }
            anySusp = true;
            if (!w.rawSeen || w.raw != w.rawPrev) fresh = true;
            w.rawPrev = w.raw; w.rawSeen = true;
            if (w.haveTravel)
            {
                w.sinceTravel += dtSec;
                if (w.sinceTravel > kTravelStaleSec) w.v = 0.0;
            }
        }
        m_since += dtSec;
        if (fresh)
        {
            if (m_rateSeen && m_since <= kRateGapSec)
            {
                const double iv = std::max(kRateMinSec, m_since);
                if (m_rateN < kRateMinSamples) { m_interval = (m_interval * m_rateN + iv) / (m_rateN + 1); ++m_rateN; }
                else m_interval += (iv - m_interval) * kRateEma;
            }
            m_rateSeen = true;
            m_since = 0.0;
        }
        if (anySusp) m_quietSec = 0.0;
        else if ((m_quietSec += dtSec) > kRateForgetSec) forgetSender();

        // The band, and whether the sender's samples can carry it: an
        // octave of it, or all of it where less is asked for.
        const double lo   = std::max(kMinLoHz, std::isfinite(p.texLoHz) ? p.texLoHz : kMinLoHz);
        const double hi   = std::min(std::max(lo * kMinBandRatio, std::isfinite(p.texHiHz) ? p.texHiHz : 0.0), 0.45 / dtSec);
        const double rate = rateHz();
        const double top  = std::min(hi, kRateShare * rate);
        const double need = std::min(hi, kMinTopRatio * lo);
        m_usable = rate > 0.0 && kRateShare * rate >= need * (m_usable ? kUsableHyst : 1.0);

        const double x = std::isfinite(p.tex) ? std::max(0.0, p.tex) : 0.0;
        if (x <= 0.0)
        {
            rest();
            m_source = Off;
            endCycle();
            return;
        }

        const int  from     = static_cast<int>(std::max(0.0, std::min(2.0, std::isfinite(p.texFrom) ? p.texFrom : 0.0)) + 0.5);   // auto, game, speed
        const bool gameNow  = m_usable && anySusp;   // the sim's own can play this cycle
        const bool wantGame = !m_preview && from != 2 && gameNow;
        const bool wantMade = m_speedDriven && (m_preview || from == 2 || (from == 0 && !gameNow));
        ease(m_wGame, wantGame ? 1.0 : 0.0, dtSec);
        ease(m_wMade, wantMade ? 1.0 : 0.0, dtSec);

        const double kmh     = m_speed * 3.6;
        const double share   = std::min(1.0, kmh / std::max(kMinKmhFull, std::isfinite(p.texKmh) ? p.texKmh : kMinKmhFull));
        // The sim's own, only from a rolling car (a sender with no road
        // speed: nothing to tell by, so all of it).
        const double roll    = m_speedDriven ? std::max(0.0, std::min(1.0, (kmh - kRollFromKmh) / (kRollFullKmh - kRollFromKmh))) : 1.0;
        // Unit rms for noise through the two cuts: their noise bandwidth is
        // kNoiseBw x (hi - lo) x hi^4 / (hi^4 - lo^4).
        const double h4 = hi * hi * hi * hi, l4 = lo * lo * lo * lo;
        const double norm    = 1.0 / std::sqrt(std::max(1e-9, 2.0 * kNoiseBw * (hi - lo) * h4 / (h4 - l4) * dtSec));
        const double surfA   = std::min(1.0, dtSec / road_k::kSurfEaseSec);
        const double groundA = std::min(1.0, dtSec / road_k::kGroundEaseSec);
        const double topHz   = std::max(lo * kMinBandRatio, top);

        for (W& w : m_w)
        {
            w.groundF += (w.groundIn - w.groundF) * groundA;
            // The sim's own: its suspension speed in the band. Both cuts are
            // steep (two second-order ones each): the bumps just under the
            // band are several times the texture, and above it sit the
            // images of the sender's held samples.
            double g = 0.0;
            if (m_wGame > 0.0 || wantGame)
            {
                double y = w.hpA.step(w.v, lo, dtSec);
                y = w.hpB.step(y, lo, dtSec);
                g = w.lpB.step(w.lpA.step(y, topHz, dtSec), topHz, dtSec) / kFullMmS * roll;
            }
            else { w.hpA.seen = w.hpB.seen = false; w.lpA.reset(); w.lpB.reset(); }
            // Made up: noise in the band at unit rms, x the speed's share,
            // x the surface (the square root of the Road models' short
            // wavelength gain: their six times rougher gravel road would
            // sit a shaker on its knee), nothing from a tyre in the air.
            double m = 0.0;
            const double sg = std::sqrt(std::max(0.0, roadShortGain(w.surface, w.water, w.floating, m_smooth)));
            if (m_wMade > 0.0 || wantMade)
            {
                w.surfGain += (sg - w.surfGain) * surfA;
                const double n = (wavesynth::rand01(w.rng) - 0.5) * 3.4641016151377544;   // unit variance
                const double y = w.nlp.step(w.nhp.step(n, lo, dtSec), hi, dtSec);
                m = y * norm * kMadeRms * share * w.surfGain * w.groundF;
            }
            else { w.nhp.seen = false; w.nlp.reset(); w.surfGain = sg; }
            w.out = roadSoftKnee(x * (m_wGame * g + m_wMade * m));
        }
        m_source = (m_wGame > 0.01 && m_wGame >= m_wMade) ? Game : (m_wMade > 0.01) ? Made : Waiting;
        endCycle();
    }

    // ---- outputs, AFTER step() ----
    // A part of several wheels plays the mean of its corners (as the Road
    // models do: RoadModel::outputFor).
    double outputFor(Part part) const
    {
        int first, last; partWheels(part, first, last);
        double s = 0.0;
        for (int i = first; i <= last; ++i) s += m_w[i].out;
        return s / static_cast<double>(last - first + 1);
    }
    double wheelOut(int wheel) const { return (wheel >= 0 && wheel < WHEEL_COUNT) ? m_w[wheel].out : 0.0; }
    double level() const
    {
        double m = 0.0;
        for (const W& w : m_w) m = std::max(m, std::fabs(w.out));
        return m;
    }
    bool active() const { return m_wGame > 0.0 || m_wMade > 0.0; }

    int    source() const   { return m_source; }
    // The suspension samples a second the sender is giving (0 = not known yet).
    double rateHz() const   { return (m_rateN >= roadtex_k::kRateMinSamples && m_interval > 0.0) ? 1.0 / m_interval : 0.0; }
    // Whether those can carry the band as set (the last step's answer, on
    // or off).
    bool   gameUsable() const { return m_usable; }

    // Another sender may be next (another game, or no suspension for a
    // while): its rate is read afresh. What plays is not touched.
    void forgetSender()
    {
        m_since = 0.0; m_interval = 0.0; m_rateN = 0; m_rateSeen = false; m_usable = false; m_quietSec = 0.0;
    }

    void clear()
    {
        for (int i = 0; i < WHEEL_COUNT; ++i) { m_w[i] = W{}; m_w[i].rng = kSeed[i]; }   // each wheel its own noise
        m_speed = 0.0; m_speedDriven = m_preview = false;
        forgetSender();
        m_wGame = m_wMade = 0.0; m_source = Off;
    }

    RoadTexture() { clear(); }

private:
    static constexpr uint64_t kSeed[WHEEL_COUNT] = { 0x9E3779B97F4A7C15ull, 0xC2B2AE3D27D4EB4Full, 0x165667B19E3779F9ull, 0x27D4EB2F165667C5ull };

    struct W
    {
        // inputs
        double raw = 0.0, rawPrev = 0.0, v = 0.0, sinceTravel = 0.0;
        bool   haveTravel = false, driven = false, rawSeen = false;
        int    surface = SurfTarmac;
        double groundIn = 1.0, groundF = 1.0;
        double water = 0.0, floating = 0.0;
        // the game's path
        HighPass2 hpA, hpB;
        LowPass2  lpA, lpB;
        // the made-up path
        uint64_t  rng = 1;
        HighPass2 nhp;
        LowPass2  nlp;
        double    surfGain = 1.0;
        // out
        double out = 0.0;
    };

    static void ease(double& w, double target, double dtSec)
    {
        const double rate = dtSec / ((target > w) ? roadtex_k::kAttackSec : roadtex_k::kReleaseSec);
        w += std::max(-rate, std::min(rate, target - w));
    }
    // Nothing playing: every filter starts afresh the next time.
    void rest()
    {
        m_wGame = m_wMade = 0.0;
        for (W& w : m_w)
        {
            w.out = 0.0;
            w.hpA.seen = w.hpB.seen = w.nhp.seen = false;
            w.lpA.reset(); w.lpB.reset(); w.nlp.reset();
        }
    }
    // The laws drive every input every cycle: one not driven is not there.
    void endCycle()
    {
        for (W& w : m_w) { w.driven = false; w.groundIn = 1.0; }
        m_speedDriven = false; m_preview = false;
    }

    W      m_w[WHEEL_COUNT];
    double m_speed = 0.0, m_smooth = 0.0;
    bool   m_speedDriven = false, m_preview = false;
    double m_since = 0.0, m_interval = 0.0, m_quietSec = 0.0;
    int    m_rateN = 0;
    bool   m_rateSeen = false, m_usable = false;
    double m_wGame = 0.0, m_wMade = 0.0;
    int    m_source = Off;
};

} // namespace haptics
