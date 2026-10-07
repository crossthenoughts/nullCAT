// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// ShakerDsp - the path from the haptics layer to a sound card, minus the
// sound card. The RT loop pushes ONE sample per channel per control cycle
// (a lock-free ring each); the audio thread renders 48 kHz frames from
// them: a fill-level servo keeps the ring half full by trimming the
// resample step a fraction of a percent (the EtherCAT clock and the USB
// crystal drift tens of ppm against each other), linear interpolation
// between control-rate samples, a second-order low-pass to take the
// stair-steps off, a DC block (voice coils and LRAs hate offset), a
// tanh soft clip, and a per-channel gain.
//
// Underrun (the RT loop stalled, or the loop stopped and the idle clock
// has not taken over): the last sample is held and faded over a few
// milliseconds, a pause rather than a click. Overrun (the audio side
// stalled): the oldest samples are dropped, the cycle is never blocked.
//
// Deterministic: render() is pure arithmetic on the rings, so the whole
// chain is unit-tested without a device (ShakerOutput owns the device).
// ============================================================

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace haptics {

static constexpr int MAX_SHAKERS   = 8;
static constexpr int SHAKER_RING   = 256;     // control-rate samples per channel (~128 ms at 2 kHz)

namespace shaker_k {
    constexpr double kLpfHz        = 300.0;   // takes the stair-steps off a 2 kHz feed
    constexpr double kDcHz         = 5.0;     // DC block corner
    // Fill servo: PROPORTIONAL only. The ring level already integrates the
    // rate mismatch, so an integrating trim on top would hunt; a straight
    // gain settles with a small steady offset (100 ppm of drift parks the
    // ring ~5% off centre), which is exactly what we want.
    constexpr double kServoMaxTrim = 0.005;   // +-0.5% resample step at an empty/full ring (covers a jittery idle clock)
    constexpr double kFadeSec      = 0.004;   // underrun fade
    // Soft clip: straight through to the knee, then tanh into the last
    // 30% so the output can never exceed 1.0 and a small signal is untouched.
    constexpr double kClipKnee     = 0.7;
}

class ShakerDsp
{
public:
    // controlHz: the rate the RT loop pushes at; outHz: the device rate.
    void configure(double controlHz, double outHz, int channels)
    {
        m_controlHz = std::max(50.0, controlHz);
        m_outHz     = std::max(8000.0, outHz);
        m_channels  = std::max(1, std::min(MAX_SHAKERS, channels));
        m_stepNom   = m_controlHz / m_outHz;    // control samples per output frame
        for (int c = 0; c < MAX_SHAKERS; ++c) { m_gain[c] = 1.0f; m_ring[c].clear(); m_st[c] = State{}; }
        // Biquad low-pass (Butterworth) at kLpfHz for the output rate.
        const double w0 = 2.0 * 3.14159265358979 * shaker_k::kLpfHz / m_outHz;
        const double cw = std::cos(w0), sw = std::sin(w0), alpha = sw / (2.0 * 0.7071);
        const double a0 = 1.0 + alpha;
        m_b0 = (1.0 - cw) / 2.0 / a0; m_b1 = (1.0 - cw) / a0; m_b2 = m_b0;
        m_a1 = -2.0 * cw / a0;        m_a2 = (1.0 - alpha) / a0;
        m_dcR = 1.0 - 2.0 * 3.14159265358979 * shaker_k::kDcHz / m_outHz;
        m_fadeStep = 1.0 / (shaker_k::kFadeSec * m_outHz);
    }

    int    channels() const  { return m_channels; }
    double controlHz() const { return m_controlHz; }
    void   setGain(int c, double g) { if (c >= 0 && c < MAX_SHAKERS) m_gain[c] = static_cast<float>(std::max(0.0, std::min(4.0, g))); }

    // RT side: one sample per channel (-1..1 nominal), never blocks.
    void push(const double* samples, int n)
    {
        for (int c = 0; c < m_channels; ++c)
            m_ring[c].push(static_cast<float>(c < n ? samples[c] : 0.0));
    }

    // Audio side: interleaved frames for every channel.
    void render(float* out, int frames)
    {
        for (int c = 0; c < m_channels; ++c) renderChannel(c, out + c, frames, m_channels);
    }

    // Diagnostics.
    int      fill(int c) const      { return (c >= 0 && c < MAX_SHAKERS) ? m_ring[c].size() : 0; }
    uint64_t underruns() const      { return m_underruns.load(std::memory_order_relaxed); }
    double   trim() const           { return m_trim; }
    float    lastOut(int c) const   { return (c >= 0 && c < MAX_SHAKERS) ? m_st[c].lastOut : 0.0f; }

private:
    // Single-producer single-consumer ring of floats.
    struct Ring
    {
        float buf[SHAKER_RING] = {};
        std::atomic<uint32_t> head{0}, tail{0};   // head = write, tail = read
        void clear() { head.store(0); tail.store(0); }
        int size() const
        {
            return static_cast<int>(head.load(std::memory_order_acquire) - tail.load(std::memory_order_acquire));
        }
        void push(float v)
        {
            const uint32_t h = head.load(std::memory_order_relaxed);
            if (static_cast<int>(h - tail.load(std::memory_order_acquire)) >= SHAKER_RING)
                tail.fetch_add(1, std::memory_order_acq_rel);   // full: drop the oldest
            buf[h % SHAKER_RING] = v;
            head.store(h + 1, std::memory_order_release);
        }
        bool peek(int offset, float& v) const
        {
            const uint32_t t = tail.load(std::memory_order_relaxed);
            if (static_cast<int>(head.load(std::memory_order_acquire) - t) <= offset) return false;
            v = buf[(t + static_cast<uint32_t>(offset)) % SHAKER_RING];
            return true;
        }
        void pop() { tail.fetch_add(1, std::memory_order_acq_rel); }
    };
    struct State
    {
        double frac   = 0.0;     // position between ring[0] and ring[1]
        float  held   = 0.0f;    // last good sample (underrun hold)
        double fade   = 1.0;     // 1 = feeding, ramps to 0 on underrun
        double z1 = 0.0, z2 = 0.0;    // biquad
        double dcX = 0.0, dcY = 0.0;  // DC block
        float  lastOut = 0.0f;
    };

    void renderChannel(int c, float* out, int frames, int stride)
    {
        using namespace shaker_k;
        Ring& r = m_ring[c]; State& s = m_st[c];
        for (int i = 0; i < frames; ++i)
        {
            // Servo on channel 0 (all channels are pushed together): aim for
            // half a ring; trim the step within +-kServoMaxTrim.
            if (c == 0)
            {
                const double err = (r.size() - SHAKER_RING / 2) / static_cast<double>(SHAKER_RING / 2);
                m_trim = std::max(-kServoMaxTrim, std::min(kServoMaxTrim, kServoMaxTrim * err));
            }
            const double step = m_stepNom * (1.0 + m_trim);
            float a, b; double x;
            if (r.peek(0, a) && r.peek(1, b))
            {
                x = a + (b - a) * s.frac;
                s.held = static_cast<float>(x);
                s.fade = std::min(1.0, s.fade + m_fadeStep);
                s.frac += step;
                while (s.frac >= 1.0 && r.size() > 1) { r.pop(); s.frac -= 1.0; }
                if (s.frac >= 1.0) s.frac = 0.999;
            }
            else
            {
                // Underrun: hold and fade out.
                if (s.fade >= 1.0 - 1e-9) m_underruns.fetch_add(1, std::memory_order_relaxed);
                s.fade = std::max(0.0, s.fade - m_fadeStep);
                x = s.held * s.fade;
            }
            // Biquad LPF (direct form II transposed).
            const double y = m_b0 * x + s.z1;
            s.z1 = m_b1 * x - m_a1 * y + s.z2;
            s.z2 = m_b2 * x - m_a2 * y;
            // DC block.
            const double d = y - s.dcX + m_dcR * s.dcY;
            s.dcX = y; s.dcY = d;
            // Gain, then the soft clip.
            const double g = d * m_gain[c];
            const double m = std::fabs(g);
            const double v = (m <= kClipKnee) ? g
                           : std::copysign(kClipKnee + (1.0 - kClipKnee) * std::tanh((m - kClipKnee) / (1.0 - kClipKnee)), g);
            s.lastOut = static_cast<float>(v);
            out[i * stride] = s.lastOut;
        }
    }

    double m_controlHz = 2000.0, m_outHz = 48000.0, m_stepNom = 2000.0 / 48000.0;
    int    m_channels = 2;
    double m_trim = 0.0;
    double m_b0 = 0, m_b1 = 0, m_b2 = 0, m_a1 = 0, m_a2 = 0, m_dcR = 0.999, m_fadeStep = 0.005;
    float  m_gain[MAX_SHAKERS] = { 1, 1, 1, 1, 1, 1, 1, 1 };
    Ring   m_ring[MAX_SHAKERS];
    State  m_st[MAX_SHAKERS];
    std::atomic<uint64_t> m_underruns{0};
};

} // namespace haptics
