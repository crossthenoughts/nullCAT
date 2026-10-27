// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// ShakerShape - per-shaker tone and dynamics shaping at the sound card: a
// five-band parametric EQ and a compressor, between ShakerDsp's smoothing
// and DC block and its soft clip (so the soft clip stays the last word on
// level). The settings are rig.json's global.shakerShaping, and the web
// page's live preview while one is edited; both arrive on the web thread
// and reach the audio thread through ShakerDsp's stage (a try-lock there,
// never a wait). Pure arithmetic, nothing allocated: TestShakerDsp drives
// it without a device.
//
// Moves glide. The audio side walks its working settings toward the
// staged ones with a ~20 ms time constant (frequency and width on a log
// scale, gains in dB) and works the filters out again every 32 frames
// while they move, so dragging a band or the threshold never steps the
// output: a step through a bass shaker is a thump. A band, the EQ or the
// compressor switched on or off cross-fades over the same time; a band
// that changes type fades out, swaps, and fades back in. Fully off, the
// shaper hands its input straight back, so a shaker nobody has shaped
// plays exactly what it played before shaping existed.
//
// Bass, not music: the compressor's level detector holds the peak with a
// release (default 200 ms) far longer than a 20-40 Hz cycle, so it sets a
// gain per passage rather than bending each cycle of the wave.
// ============================================================

#include <algorithm>
#include <cmath>
#include <cstring>

namespace haptics {

static constexpr int SHAPE_BANDS = 5;

// The order is the stored and API order of the type names below.
enum class BandType : int { HighPass = 0, LowShelf = 1, Bell = 2, HighShelf = 3, LowPass = 4 };
static constexpr int BAND_TYPES = 5;
inline const char* bandTypeName(BandType t)
{
    static const char* const k[BAND_TYPES] = { "highpass", "lowshelf", "bell", "highshelf", "lowpass" };
    const int i = static_cast<int>(t);
    return (i >= 0 && i < BAND_TYPES) ? k[i] : "bell";
}
// false when the name is none of them (t untouched).
inline bool bandTypeFromName(const char* s, BandType& t)
{
    for (int i = 0; i < BAND_TYPES; ++i)
        if (s && std::strcmp(s, bandTypeName(static_cast<BandType>(i))) == 0) { t = static_cast<BandType>(i); return true; }
    return false;
}
// Bells and shelves have a gain; the pass types do not.
inline bool bandHasGain(BandType t) { return t == BandType::Bell || t == BandType::LowShelf || t == BandType::HighShelf; }

struct ShapeBand
{
    bool     on   = true;
    BandType type = BandType::Bell;
    double   hz   = 40.0;
    double   db   = 0.0;     // bells and shelves (the pass types ignore it)
    double   q    = 1.0;     // a bell's width, a shelf's slope, a pass filter's corner peak
    bool operator==(const ShapeBand& o) const { return on == o.on && type == o.type && hz == o.hz && db == o.db && q == o.q; }
    bool operator!=(const ShapeBand& o) const { return !(*this == o); }
};

struct ShapeComp
{
    bool   on          = false;
    double thresholdDb = -18.0;
    double ratio       = 3.0;
    double kneeDb      = 6.0;
    double attackMs    = 10.0;
    double releaseMs   = 200.0;
    double makeupDb    = 0.0;
    bool operator==(const ShapeComp& o) const
    {
        return on == o.on && thresholdDb == o.thresholdDb && ratio == o.ratio && kneeDb == o.kneeDb
            && attackMs == o.attackMs && releaseMs == o.releaseMs && makeupDb == o.makeupDb;
    }
    bool operator!=(const ShapeComp& o) const { return !(*this == o); }
};

// One shaker's shaping. The default is flat and off: the EQ off, its two
// end bands (a high-pass and a low-pass) off, three bells on at 0 dB
// across where shakers live, the compressor off.
struct ShakerShape
{
    bool      eqOn = false;
    ShapeBand band[SHAPE_BANDS] = {
        { false, BandType::HighPass,  20.0, 0.0, 0.71 },
        { true,  BandType::Bell,      30.0, 0.0, 1.0  },
        { true,  BandType::Bell,      50.0, 0.0, 1.0  },
        { true,  BandType::Bell,      80.0, 0.0, 1.0  },
        { false, BandType::LowPass,  150.0, 0.0, 0.71 },
    };
    ShapeComp comp;
    bool operator==(const ShakerShape& o) const
    {
        if (eqOn != o.eqOn || comp != o.comp) return false;
        for (int i = 0; i < SHAPE_BANDS; ++i) if (band[i] != o.band[i]) return false;
        return true;
    }
    bool operator!=(const ShakerShape& o) const { return !(*this == o); }
    bool isDefault() const { return *this == ShakerShape{}; }
};

namespace shape_k {
    // Limits (the config's clamp, the web's preview, the page's controls).
    constexpr double kHzMin      = 10.0,  kHzMax      = 300.0;
    constexpr double kDbMin      = -24.0, kDbMax      = 12.0;
    constexpr double kQMin       = 0.3,   kQMax       = 8.0;
    constexpr double kThrMin     = -48.0, kThrMax     = 0.0;
    constexpr double kRatioMin   = 1.0,   kRatioMax   = 20.0;
    constexpr double kKneeMin    = 0.0,   kKneeMax    = 18.0;
    constexpr double kAttackMin  = 1.0,   kAttackMax  = 100.0;
    constexpr double kReleaseMin = 20.0,  kReleaseMax = 2000.0;
    constexpr double kMakeupMin  = 0.0,   kMakeupMax  = 18.0;
    // Glide: the time constant of a move, and how often (frames) the
    // filters are worked out again while one is under way.
    constexpr double kGlideSec   = 0.020;
    constexpr int    kBlock      = 32;
    // A filter's corner stays below this fraction of the card's rate.
    constexpr double kMaxHzOfRate = 0.45;
    // The editor's test signals (ShakerDsp plays them at the card's rate):
    // a slow logarithmic sweep, and a tone at a band's frequency that
    // follows the band while it is dragged. The level is the 40 Hz test
    // burst's, so the three compare.
    constexpr double kSweepFromHz = 10.0, kSweepToHz = 200.0, kSweepSec = 10.0;
    constexpr double kToneSec     = 3.0;
    constexpr double kTestLevel   = 0.5;
    constexpr double kTestFadeSec = 0.010;
    constexpr double kToneGlideSec = 0.030;
}

// Clamp every field to its limits (a value that is not finite takes the
// default's). True when anything changed.
inline bool sanitize(ShakerShape& s)
{
    using namespace shape_k;
    const ShakerShape d;
    bool changed = false;
    auto fix = [&changed](double& v, double lo, double hi, double def)
    {
        const double was = v;
        if (!std::isfinite(v)) v = def;
        v = std::max(lo, std::min(hi, v));
        if (!(v == was)) changed = true;
    };
    for (int i = 0; i < SHAPE_BANDS; ++i)
    {
        ShapeBand& b = s.band[i];
        const int t = static_cast<int>(b.type);
        if (t < 0 || t >= BAND_TYPES) { b.type = d.band[i].type; changed = true; }
        fix(b.hz, kHzMin, kHzMax, d.band[i].hz);
        fix(b.db, kDbMin, kDbMax, d.band[i].db);
        fix(b.q,  kQMin,  kQMax,  d.band[i].q);
    }
    ShapeComp& c = s.comp;
    fix(c.thresholdDb, kThrMin,     kThrMax,     d.comp.thresholdDb);
    fix(c.ratio,       kRatioMin,   kRatioMax,   d.comp.ratio);
    fix(c.kneeDb,      kKneeMin,    kKneeMax,    d.comp.kneeDb);
    fix(c.attackMs,    kAttackMin,  kAttackMax,  d.comp.attackMs);
    fix(c.releaseMs,   kReleaseMin, kReleaseMax, d.comp.releaseMs);
    fix(c.makeupDb,    kMakeupMin,  kMakeupMax,  d.comp.makeupDb);
    return changed;
}

// A second-order section, normalised (a0 = 1).
struct Biquad { double b0 = 1.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0; };

// The band's filter at a sample rate (the Audio EQ Cookbook forms). The
// page draws its curve from the same formulas (web/app.js shkBandResp).
inline Biquad bandCoeffs(BandType t, double hz, double db, double q, double fs)
{
    const double pi = 3.14159265358979323846;
    hz = std::max(1.0, std::min(hz, shape_k::kMaxHzOfRate * fs));
    q  = std::max(0.05, q);
    const double w0 = 2.0 * pi * hz / fs, cw = std::cos(w0), sw = std::sin(w0);
    const double alpha = sw / (2.0 * q);
    const double A = std::pow(10.0, db / 40.0);
    double b0, b1, b2, a0, a1, a2;
    switch (t)
    {
    case BandType::HighPass:
        b0 = (1.0 + cw) / 2.0; b1 = -(1.0 + cw); b2 = b0; a0 = 1.0 + alpha; a1 = -2.0 * cw; a2 = 1.0 - alpha; break;
    case BandType::LowPass:
        b0 = (1.0 - cw) / 2.0; b1 = 1.0 - cw;    b2 = b0; a0 = 1.0 + alpha; a1 = -2.0 * cw; a2 = 1.0 - alpha; break;
    case BandType::LowShelf:
    {
        const double sa = 2.0 * std::sqrt(A) * alpha;
        b0 =       A * ((A + 1.0) - (A - 1.0) * cw + sa);
        b1 = 2.0 * A * ((A - 1.0) - (A + 1.0) * cw);
        b2 =       A * ((A + 1.0) - (A - 1.0) * cw - sa);
        a0 =            (A + 1.0) + (A - 1.0) * cw + sa;
        a1 =    -2.0 * ((A - 1.0) + (A + 1.0) * cw);
        a2 =            (A + 1.0) + (A - 1.0) * cw - sa;
        break;
    }
    case BandType::HighShelf:
    {
        const double sa = 2.0 * std::sqrt(A) * alpha;
        b0 =        A * ((A + 1.0) + (A - 1.0) * cw + sa);
        b1 = -2.0 * A * ((A - 1.0) + (A + 1.0) * cw);
        b2 =        A * ((A + 1.0) + (A - 1.0) * cw - sa);
        a0 =             (A + 1.0) - (A - 1.0) * cw + sa;
        a1 =      2.0 * ((A - 1.0) - (A + 1.0) * cw);
        a2 =             (A + 1.0) - (A - 1.0) * cw - sa;
        break;
    }
    case BandType::Bell:
    default:
        b0 = 1.0 + alpha * A; b1 = -2.0 * cw; b2 = 1.0 - alpha * A;
        a0 = 1.0 + alpha / A; a1 = -2.0 * cw; a2 = 1.0 - alpha / A;
        break;
    }
    Biquad c;
    c.b0 = b0 / a0; c.b1 = b1 / a0; c.b2 = b2 / a0; c.a1 = a1 / a0; c.a2 = a2 / a0;
    return c;
}

// The compressor's static curve: gain change in dB (0 or below) for a
// level in dB, with a soft knee kneeDb wide around the threshold.
inline double compGainDb(double levelDb, double thresholdDb, double ratio, double kneeDb)
{
    const double over = levelDb - thresholdDb;
    const double slope = 1.0 / std::max(1.0, ratio) - 1.0;   // 0 at 1:1, toward -1 at high ratios
    if (kneeDb > 0.0 && 2.0 * std::fabs(over) <= kneeDb)
    {
        const double k = over + kneeDb / 2.0;
        return slope * k * k / (2.0 * kneeDb);
    }
    return over > 0.0 ? slope * over : 0.0;
}

// One shaker's shaper, run by the audio thread: setTarget() when the stage
// brings new settings, process() per frame. The on/off fades (each band's,
// the EQ's, the compressor's) move every frame; the settings move every
// kBlock frames, where the filters are worked out again.
class ShakerShaper
{
public:
    // A new sample rate (the card opened): states cleared, the working
    // settings jump to the target on the next setTarget(snap).
    void reset(double fs)
    {
        const double r = std::max(8000.0, fs);
        *this = ShakerShaper{};
        m_fs = r;
        m_glide  = 1.0 - std::exp(-static_cast<double>(shape_k::kBlock) / (shape_k::kGlideSec * m_fs));
        m_glide1 = 1.0 - std::exp(-1.0 / (shape_k::kGlideSec * m_fs));
    }

    // The settings to move toward (sanitised by the caller); bypass is the
    // page's compare: EQ and compressor fade out while it is held. snap:
    // no glide (a card just opened).
    void setTarget(const ShakerShape& s, bool bypass, bool snap)
    {
        m_tgt = s; m_bypass = bypass;
        if (snap)
        {
            for (int i = 0; i < SHAPE_BANDS; ++i)
            {
                Band& b = m_band[i];
                b.type = s.band[i].type; b.hz = s.band[i].hz; b.db = s.band[i].db; b.q = s.band[i].q;
                b.z1 = b.z2 = 0.0;
                b.c = bandCoeffs(b.type, b.hz, b.db, b.q, m_fs);
            }
            aimMixes();   // after the types: a band is not 'changing type' when it lands on its own
            for (Band& b : m_band) b.mix = b.mixTgt;
            m_eqMix = m_eqMixTgt; m_compMix = m_compMixTgt;
            m_thr = s.comp.thresholdDb; m_ratio = s.comp.ratio; m_knee = s.comp.kneeDb; m_makeup = s.comp.makeupDb;
            m_env = 0.0; m_grDb = 0.0;
        }
        else aimMixes();
        m_aAtt = std::exp(-1.0 / (s.comp.attackMs  * 1e-3 * m_fs));
        m_aRel = std::exp(-1.0 / (s.comp.releaseMs * 1e-3 * m_fs));
        m_settled = false;
        m_idle = false;
        m_left = 0;   // the next process() looks at the targets at once
    }

    double process(double x)
    {
        if (--m_left <= 0) { m_left = shape_k::kBlock; if (!m_settled) update(); }
        if (m_idle) return x;
        double y = x;
        if (m_eqMix != m_eqMixTgt) m_eqMix += m_glide1 * (m_eqMixTgt - m_eqMix);
        if (m_eqMix > 0.0)
        {
            double e = x;
            for (Band& b : m_band)
            {
                if (b.mix != b.mixTgt) b.mix += m_glide1 * (b.mixTgt - b.mix);
                if (b.mix <= 0.0) continue;
                // Direct form II transposed.
                const double f = b.c.b0 * e + b.z1;
                b.z1 = b.c.b1 * e - b.c.a1 * f + b.z2;
                b.z2 = b.c.b2 * e - b.c.a2 * f;
                e = (b.mix >= 1.0) ? f : e + b.mix * (f - e);
            }
            y = (m_eqMix >= 1.0) ? e : x + m_eqMix * (e - x);
        }
        if (m_compMix != m_compMixTgt) m_compMix += m_glide1 * (m_compMixTgt - m_compMix);
        if (m_compMix > 0.0)
        {
            const double lvl = std::fabs(y);
            m_env = (lvl > m_env) ? m_aAtt * m_env + (1.0 - m_aAtt) * lvl : m_aRel * m_env + (1.0 - m_aRel) * lvl;
            const double levelDb = 20.0 * std::log10(std::max(m_env, 1e-6));
            m_grDb = compGainDb(levelDb, m_thr, m_ratio, m_knee);
            const double g = std::pow(10.0, (m_grDb + m_makeup) / 20.0);
            const double c = y * g;
            y = (m_compMix >= 1.0) ? c : y + m_compMix * (c - y);
        }
        return y;
    }

    // The compressor's gain reduction just now, dB (0 or more; 0 when off).
    double grDb() const { return m_compMix > 0.0 ? -m_grDb * m_compMix : 0.0; }
    // Nothing on and nothing fading: process() is the identity.
    bool idle() const { return m_idle; }

private:
    struct Band
    {
        BandType type = BandType::Bell;
        double hz = 40.0, db = 0.0, q = 1.0;
        double mix = 0.0, mixTgt = 0.0;
        double z1 = 0.0, z2 = 0.0;
        Biquad c;
    };

    // Where each fade is headed. A band changing type heads for 0 first.
    void aimMixes()
    {
        const bool eqWant = m_tgt.eqOn && !m_bypass;
        for (int i = 0; i < SHAPE_BANDS; ++i)
        {
            Band& b = m_band[i];
            const bool swapping = (b.type != m_tgt.band[i].type);
            b.mixTgt = (eqWant && m_tgt.band[i].on && !swapping) ? 1.0 : 0.0;
        }
        m_eqMixTgt   = eqWant ? 1.0 : 0.0;
        m_compMixTgt = (m_tgt.comp.on && !m_bypass) ? 1.0 : 0.0;
    }
    // Walk v toward t by the glide step; snap when close. True when it moved.
    bool glideLin(double& v, double t, double eps) const
    {
        if (v == t) return false;
        v += m_glide * (t - v);
        if (std::fabs(t - v) < eps) v = t;
        return true;
    }
    bool glideLog(double& v, double t) const
    {
        if (v == t) return false;
        v = std::exp(std::log(v) + m_glide * (std::log(t) - std::log(v)));
        if (std::fabs(v / t - 1.0) < 1e-4) v = t;
        return true;
    }
    // A fade that process() moves: done when it is there (snapped here).
    static bool fadeDone(double& v, double t)
    {
        if (std::fabs(t - v) < 1e-4) { v = t; return true; }
        return false;
    }
    void update()
    {
        bool moving = false;
        for (int i = 0; i < SHAPE_BANDS; ++i)
        {
            Band& b = m_band[i];
            const ShapeBand& t = m_tgt.band[i];
            if (b.type != t.type)
            {
                // Fading out to change type; once out, the new type takes
                // over from rest, at its own settings, and fades in.
                if (fadeDone(b.mix, b.mixTgt) && b.mix == 0.0)
                {
                    b.type = t.type; b.hz = t.hz; b.db = t.db; b.q = t.q; b.z1 = b.z2 = 0.0;
                    b.c = bandCoeffs(b.type, b.hz, b.db, b.q, m_fs);
                    aimMixes();
                }
                moving = true;
                continue;
            }
            bool c = glideLog(b.hz, t.hz);
            c = glideLin(b.db, t.db, 1e-3) || c;
            c = glideLog(b.q, t.q) || c;
            if (c) b.c = bandCoeffs(b.type, b.hz, b.db, b.q, m_fs);
            moving = moving || c || !fadeDone(b.mix, b.mixTgt);
            if (b.mix == 0.0 && b.mixTgt == 0.0) b.z1 = b.z2 = 0.0;   // at rest: starts clean when it comes back
        }
        moving = !fadeDone(m_eqMix, m_eqMixTgt) || moving;
        moving = !fadeDone(m_compMix, m_compMixTgt) || moving;
        moving = glideLin(m_thr,    m_tgt.comp.thresholdDb, 1e-3) || moving;
        moving = glideLog(m_ratio,  m_tgt.comp.ratio)             || moving;
        moving = glideLin(m_knee,   m_tgt.comp.kneeDb, 1e-3)      || moving;
        moving = glideLin(m_makeup, m_tgt.comp.makeupDb, 1e-3)    || moving;
        if (m_eqMix == 0.0 && m_eqMixTgt == 0.0) for (Band& b : m_band) b.z1 = b.z2 = 0.0;
        if (m_compMix == 0.0 && m_compMixTgt == 0.0) { m_env = 0.0; m_grDb = 0.0; }
        m_settled = !moving;
        m_idle = (m_eqMix == 0.0 && m_eqMixTgt == 0.0 && m_compMix == 0.0 && m_compMixTgt == 0.0);
    }

    double m_fs = 48000.0, m_glide = 0.03, m_glide1 = 0.001;
    ShakerShape m_tgt;
    bool   m_bypass = false;
    Band   m_band[SHAPE_BANDS];
    double m_eqMix = 0.0, m_eqMixTgt = 0.0, m_compMix = 0.0, m_compMixTgt = 0.0;
    double m_thr = -18.0, m_ratio = 3.0, m_knee = 6.0, m_makeup = 0.0;
    double m_aAtt = 0.0, m_aRel = 0.0, m_env = 0.0, m_grDb = 0.0;
    int    m_left = 0;
    bool   m_settled = true, m_idle = true;
};

} // namespace haptics
