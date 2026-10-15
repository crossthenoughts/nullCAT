// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// ShakerDsp - the path from the haptics layer to a sound card, minus the
// sound card. The RT loop pushes ONE frame per control cycle (every
// channel's sample together, one lock-free ring of frames, so the channels
// travel in step and can never drift apart); the audio thread renders
// 48 kHz frames from them: a fill-level servo keeps the ring at its target
// by trimming the resample step a fraction of a percent (the EtherCAT
// clock and the USB crystal drift tens of ppm against each other), linear
// interpolation between control-rate frames, a second-order low-pass to
// take the stair-steps off, a DC block (voice coils and LRAs hate offset),
// a tanh soft clip, and a per-channel gain. A peak meter per channel for
// the status line.
//
// Underrun (the RT loop stalled, or the loop stopped and the idle clock
// has not taken over): the last frame is held and faded over a few
// milliseconds, a pause rather than a click. Overrun (the audio side
// stalled): new frames are dropped while the ring is full, the cycle is
// never blocked, and the audio side skips to the newest when it wakes.
// A pushed sample that is not finite is taken as 0 at the ring, so the
// recursive filters never see one.
//
// Deterministic: render() is pure arithmetic on the ring, so the whole
// chain is unit-tested without a device (ShakerOutput owns the device).
// ============================================================

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace haptics {

static constexpr int MAX_SHAKERS   = 8;
static constexpr int SHAKER_RING   = 256;     // control-rate frames (~128 ms at 2 kHz): room, not latency

namespace shaker_k {
    // The ring is held this full: enough to ride out the audio thread
    // taking a period's frames in one go and waking a little late, and no
    // more, since every frame waiting in it is latency. At least this
    // long, and at least this many device periods.
    constexpr double kTargetSec     = 0.012;
    constexpr double kTargetPeriods = 2.5;
    constexpr int    kMinTarget     = 4;
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
    // Level meter: the peak, released over this time constant, so a status
    // read at any moment sees the level, not one point on the waveform.
    constexpr double kMeterReleaseSec = 0.3;
    // A ring within this of full at a callback: the audio side stalled and
    // the producer has been dropping frames; skip to the target.
    constexpr int    kSkipMargin   = 4;
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
        m_stepNom   = m_controlHz / m_outHz;    // control frames per output frame
        m_ring.clear(); m_read = Read{};
        for (int c = 0; c < MAX_SHAKERS; ++c) { m_gain[c] = 1.0f; m_st[c] = State{}; m_meter[c].store(0.0f, std::memory_order_relaxed); }
        // Biquad low-pass (Butterworth) at kLpfHz for the output rate.
        const double w0 = 2.0 * 3.14159265358979 * shaker_k::kLpfHz / m_outHz;
        const double cw = std::cos(w0), sw = std::sin(w0), alpha = sw / (2.0 * 0.7071);
        const double a0 = 1.0 + alpha;
        m_b0 = (1.0 - cw) / 2.0 / a0; m_b1 = (1.0 - cw) / a0; m_b2 = m_b0;
        m_a1 = -2.0 * cw / a0;        m_a2 = (1.0 - alpha) / a0;
        m_dcR = 1.0 - 2.0 * 3.14159265358979 * shaker_k::kDcHz / m_outHz;
        m_fadeStep = 1.0 / (shaker_k::kFadeSec * m_outHz);
        m_meterR = std::exp(-1.0 / (shaker_k::kMeterReleaseSec * m_outHz));
        retarget();
    }
    // The device's period, frames per callback (0 = unknown): the ring is
    // held at least kTargetPeriods of them.
    void setPeriodFrames(int frames) { m_periodFrames = std::max(0, frames); retarget(); }

    int    channels() const  { return m_channels; }
    double controlHz() const { return m_controlHz; }
    int    target() const    { return m_target; }                        // control frames the ring is held at
    double targetMs() const  { return 1000.0 * m_target / m_controlHz; } // ...as time: the ring's share of the latency
    void   setGain(int c, double g) { if (c >= 0 && c < MAX_SHAKERS) m_gain[c] = static_cast<float>(std::max(0.0, std::min(4.0, g))); }

    // RT side: one sample per channel (-1..1 nominal), one frame, never blocks.
    void push(const double* samples, int n) { m_ring.push(samples, n, m_channels); }

    // Audio side: interleaved frames for every channel.
    void render(float* out, int frames)
    {
        using namespace shaker_k;
        const int ch = m_channels;
        Read& rd = m_read;
        // Woken after a stall to a full ring: the newest frames, at the
        // target, not ones a stall old (only this side moves the read
        // position, so no slot is ever written while it is read).
        if (const int n = m_ring.size(); n >= SHAKER_RING - kSkipMargin) { m_ring.skip(n - m_target); rd.frac = 0.0; }
        for (int i = 0; i < frames; ++i)
        {
            // Servo: aim for the target; trim the step within +-kServoMaxTrim.
            const double err = (m_ring.size() - m_target) / static_cast<double>(m_target);
            m_trim = std::max(-kServoMaxTrim, std::min(kServoMaxTrim, kServoMaxTrim * err));
            const double step = m_stepNom * (1.0 + m_trim);
            const float* a = nullptr; const float* b = nullptr;
            const bool feeding = m_ring.pair(a, b);
            if (feeding)
            {
                rd.fade = std::min(1.0, rd.fade + m_fadeStep);
            }
            else
            {
                // Underrun: hold and fade out.
                if (rd.fade >= 1.0 - 1e-9) m_underruns.fetch_add(1, std::memory_order_relaxed);
                rd.fade = std::max(0.0, rd.fade - m_fadeStep);
            }
            for (int c = 0; c < ch; ++c)
            {
                State& s = m_st[c];
                double x;
                if (feeding) { x = a[c] + (b[c] - a[c]) * rd.frac; s.held = static_cast<float>(x); }
                else         x = s.held * rd.fade;
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
                s.peak = std::max(std::fabs(v), s.peak * m_meterR);
                out[i * ch + c] = static_cast<float>(v);
            }
            if (feeding)
            {
                rd.frac += step;
                while (rd.frac >= 1.0 && m_ring.size() > 1) { m_ring.pop(); rd.frac -= 1.0; }
                if (rd.frac >= 1.0) rd.frac = 0.999;
            }
        }
        for (int c = 0; c < ch; ++c) m_meter[c].store(static_cast<float>(m_st[c].peak), std::memory_order_relaxed);
    }

    // Diagnostics.
    int      fill(int c) const      { return (c >= 0 && c < MAX_SHAKERS) ? m_ring.size() : 0; }
    uint64_t underruns() const      { return m_underruns.load(std::memory_order_relaxed); }
    double   trim() const           { return m_trim; }
    // Output level of a channel, 0..1: its recent peak (any thread).
    float    level(int c) const     { return (c >= 0 && c < MAX_SHAKERS) ? m_meter[c].load(std::memory_order_relaxed) : 0.0f; }

private:
    void retarget()
    {
        using namespace shaker_k;
        const double want = std::max(kTargetSec * m_controlHz, kTargetPeriods * m_periodFrames * m_stepNom);
        m_target = std::max(kMinTarget, std::min(SHAKER_RING / 2, static_cast<int>(std::ceil(want))));
    }

    // Single-producer single-consumer ring of frames (every channel's
    // sample for one control cycle in one slot, published together). Only
    // the consumer moves the tail: a full ring drops the producer's new
    // frame, so the producer never writes a slot the consumer may be
    // reading (it writes at head, at least two past the two it reads).
    struct Ring
    {
        float buf[SHAKER_RING][MAX_SHAKERS] = {};
        std::atomic<uint32_t> head{0}, tail{0};   // head = write, tail = read
        void clear() { head.store(0); tail.store(0); }
        int size() const
        {
            return static_cast<int>(head.load(std::memory_order_acquire) - tail.load(std::memory_order_acquire));
        }
        void push(const double* samples, int n, int channels)
        {
            const uint32_t h = head.load(std::memory_order_relaxed);
            if (static_cast<int>(h - tail.load(std::memory_order_acquire)) >= SHAKER_RING) return;   // full: dropped
            float* slot = buf[h % SHAKER_RING];
            // The filters downstream are recursive: one NaN or inf in
            // would stay in their state for the rest of the run. A sample
            // that is not finite counts as 0 (TestShakerDsp S-10).
            for (int c = 0; c < channels; ++c)
                slot[c] = (c < n && std::isfinite(samples[c])) ? static_cast<float>(samples[c]) : 0.0f;
            head.store(h + 1, std::memory_order_release);
        }
        // The two frames at the read position (false: not both there yet).
        bool pair(const float*& a, const float*& b) const
        {
            const uint32_t t = tail.load(std::memory_order_relaxed);
            if (static_cast<int>(head.load(std::memory_order_acquire) - t) < 2) return false;
            a = buf[t % SHAKER_RING];
            b = buf[(t + 1) % SHAKER_RING];
            return true;
        }
        void pop()       { tail.store(tail.load(std::memory_order_relaxed) + 1, std::memory_order_release); }
        void skip(int n) { if (n > 0) tail.store(tail.load(std::memory_order_relaxed) + static_cast<uint32_t>(n), std::memory_order_release); }
    };
    // The read position, shared by every channel.
    struct Read
    {
        double frac = 0.0;     // position between ring[0] and ring[1]
        double fade = 1.0;     // 1 = feeding, ramps to 0 on underrun
    };
    struct State
    {
        float  held   = 0.0f;    // last good sample (underrun hold)
        double z1 = 0.0, z2 = 0.0;    // biquad
        double dcX = 0.0, dcY = 0.0;  // DC block
        double peak = 0.0;            // meter (audio thread)
    };

    double m_controlHz = 2000.0, m_outHz = 48000.0, m_stepNom = 2000.0 / 48000.0;
    int    m_channels = 2;
    int    m_periodFrames = 0, m_target = 24;
    double m_trim = 0.0;
    double m_b0 = 0, m_b1 = 0, m_b2 = 0, m_a1 = 0, m_a2 = 0, m_dcR = 0.999, m_fadeStep = 0.005, m_meterR = 0.9999;
    float  m_gain[MAX_SHAKERS] = { 1, 1, 1, 1, 1, 1, 1, 1 };
    Ring   m_ring;
    Read   m_read;
    State  m_st[MAX_SHAKERS];
    std::atomic<float> m_meter[MAX_SHAKERS] = {};
    std::atomic<uint64_t> m_underruns{0};
};

} // namespace haptics
