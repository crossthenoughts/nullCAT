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
// Shaping (ShakerShape.h): each channel's EQ and compressor sit after the
// DC block and before the gain and soft clip. Their settings and the
// editor's test signals are staged by the web thread under m_stageMx and
// taken at the top of render() with a try-lock: the audio thread never
// waits, and a stage it misses is taken on the next callback (a few ms).
// The stage survives configure() (a card opened again): the channels pick
// their shaping up again, at once rather than gliding.
//
// Test signals (the editor's sweep and band tone) are made here at the
// card's rate, not in the haptics layer at the loop rate: a 150 Hz tone
// fed at the PC's 500 Hz would be three samples a cycle. They go in ahead
// of the low-pass, so they pass through the channel's shaping like any
// effect. The 40 Hz test burst stays in the haptics layer: it checks the
// whole path from the control loop.
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

#include "ShakerShape.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>

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
        // Shaping and test signals at this rate. The staged settings are
        // kept (a card opened again keeps its shaping; ShakerBank stages a
        // new card's before it opens) and taken on the first render,
        // snapped: there is nothing to glide from.
        for (int c = 0; c < MAX_SHAKERS; ++c)
        {
            m_shaper[c].reset(m_outHz); m_test[c] = TestGen{};
            m_gr[c].store(0.0f, std::memory_order_relaxed); m_testHz[c].store(0.0f, std::memory_order_relaxed);
        }
        m_testFadeStep = 1.0 / (shape_k::kTestFadeSec * m_outHz);
        m_sweepMul     = std::exp(std::log(shape_k::kSweepToHz / shape_k::kSweepFromHz) / (shape_k::kSweepSec * m_outHz));
        m_toneGlide    = 1.0 - std::exp(-1.0 / (shape_k::kToneGlideSec * m_outHz));
        m_snapNext = true;
        m_stageDirty.store(true, std::memory_order_release);
        retarget();
    }

    // Web thread: channel c's shaping (sanitised here), and the page's
    // compare (bypass: EQ and compressor fade out while it is held).
    void stageShape(int c, const ShakerShape& shape, bool bypass)
    {
        if (c < 0 || c >= MAX_SHAKERS) return;
        ShakerShape s = shape; sanitize(s);
        std::lock_guard<std::mutex> lk(m_stageMx);
        m_stagedShape[c] = s; m_stagedBypass[c] = bypass;
        m_stageDirty.store(true, std::memory_order_release);
    }
    // Web thread: an editor test signal on channel c. kind: TestStop,
    // TestSweep (kSweepFromHz to kSweepToHz over kSweepSec) or TestTone
    // (hz for kToneSec; asked again while it plays, it glides to the new
    // hz and plays on from there).
    enum TestKind { TestStop = 0, TestSweep = 1, TestTone = 2 };
    void startTest(int c, int kind, double hz)
    {
        if (c < 0 || c >= MAX_SHAKERS) return;
        if (!std::isfinite(hz)) hz = 40.0;
        std::lock_guard<std::mutex> lk(m_stageMx);
        TestReq& r = m_stagedTest[c];
        r.kind = (kind == TestSweep || kind == TestTone) ? kind : TestStop;
        r.hz   = std::max(shape_k::kHzMin, std::min(shape_k::kHzMax, hz));
        ++r.seq;
        m_stageDirty.store(true, std::memory_order_release);
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
        takeStage();
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
                if (m_test[c].kind != TestStop) x += testSample(m_test[c]);
                // Biquad LPF (direct form II transposed).
                const double y = m_b0 * x + s.z1;
                s.z1 = m_b1 * x - m_a1 * y + s.z2;
                s.z2 = m_b2 * x - m_a2 * y;
                // DC block.
                const double d = y - s.dcX + m_dcR * s.dcY;
                s.dcX = y; s.dcY = d;
                // The channel's EQ and compressor (the identity when off).
                const double sh = m_shaper[c].process(d);
                if (!m_shaper[c].idle() || s.grPeak > 0.0)
                {
                    s.grPeak = std::max(m_shaper[c].grDb(), s.grPeak * m_meterR);
                    if (s.grPeak < 1e-4) s.grPeak = 0.0;   // settles at 0, not in ever smaller numbers
                }
                // Gain, then the soft clip.
                const double g = sh * m_gain[c];
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
        for (int c = 0; c < ch; ++c)
        {
            m_meter[c].store(static_cast<float>(m_st[c].peak), std::memory_order_relaxed);
            m_gr[c].store(static_cast<float>(m_st[c].grPeak < 1e-3 ? 0.0 : m_st[c].grPeak), std::memory_order_relaxed);
            m_testHz[c].store(m_test[c].kind != TestStop ? static_cast<float>(m_test[c].hz) : 0.0f, std::memory_order_relaxed);
        }
    }

    // Diagnostics.
    int      fill(int c) const      { return (c >= 0 && c < MAX_SHAKERS) ? m_ring.size() : 0; }
    uint64_t underruns() const      { return m_underruns.load(std::memory_order_relaxed); }
    double   trim() const           { return m_trim; }
    // Output level of a channel, 0..1: its recent peak (any thread).
    float    level(int c) const     { return (c >= 0 && c < MAX_SHAKERS) ? m_meter[c].load(std::memory_order_relaxed) : 0.0f; }
    // The compressor's gain reduction on a channel, dB (0 or more): its
    // recent peak, released like the level meter (any thread).
    float    gainReductionDb(int c) const { return (c >= 0 && c < MAX_SHAKERS) ? m_gr[c].load(std::memory_order_relaxed) : 0.0f; }
    // The test signal playing on a channel, Hz (0 = none; any thread).
    float    testHz(int c) const    { return (c >= 0 && c < MAX_SHAKERS) ? m_testHz[c].load(std::memory_order_relaxed) : 0.0f; }

private:
    struct TestGen
    {
        int    kind = TestStop;
        double hz = 0.0, hzTgt = 0.0, phase = 0.0, fade = 0.0, left = 0.0;
        bool   stopping = false;
    };
    struct TestReq { int kind = TestStop; double hz = 40.0; uint32_t seq = 0; };
    // Audio side, top of render(): the web thread's latest settings and
    // test requests, if it is not writing them this instant.
    void takeStage()
    {
        if (!m_stageDirty.load(std::memory_order_acquire)) return;
        std::unique_lock<std::mutex> lk(m_stageMx, std::try_to_lock);
        if (!lk.owns_lock()) return;   // being written: the next callback takes it
        m_stageDirty.store(false, std::memory_order_relaxed);
        for (int c = 0; c < MAX_SHAKERS; ++c)
        {
            m_shaper[c].setTarget(m_stagedShape[c], m_stagedBypass[c], m_snapNext);
            const TestReq& r = m_stagedTest[c];
            if (r.seq == m_testSeen[c]) continue;
            m_testSeen[c] = r.seq;
            if (m_snapNext) continue;   // a request from before the card opened is old news
            TestGen& g = m_test[c];
            if (r.kind == TestStop) { if (g.kind != TestStop) g.stopping = true; }
            else if (r.kind == TestTone && g.kind == TestTone && !g.stopping) { g.hzTgt = r.hz; g.left = shape_k::kToneSec; }
            else
            {
                const bool playing = (g.kind != TestStop);
                g.kind = r.kind; g.stopping = false;
                g.hz = g.hzTgt = (r.kind == TestSweep) ? shape_k::kSweepFromHz : r.hz;
                g.left = (r.kind == TestSweep) ? shape_k::kSweepSec : shape_k::kToneSec;
                if (!playing) { g.phase = 0.0; g.fade = 0.0; }
            }
        }
        m_snapNext = false;
    }
    // One frame of a channel's test signal; fades in and out over
    // kTestFadeSec, phase-continuous through a glide.
    double testSample(TestGen& g)
    {
        if (g.kind == TestSweep) g.hz = std::min(shape_k::kSweepToHz, g.hz * m_sweepMul);
        else                     g.hz += m_toneGlide * (g.hzTgt - g.hz);
        const bool ending = g.stopping || g.left <= shape_k::kTestFadeSec;
        g.fade = ending ? std::max(0.0, g.fade - m_testFadeStep) : std::min(1.0, g.fade + m_testFadeStep);
        const double twoPi = 6.283185307179586;
        g.phase += twoPi * g.hz / m_outHz;
        if (g.phase >= twoPi) g.phase -= twoPi;
        const double v = shape_k::kTestLevel * g.fade * std::sin(g.phase);
        g.left -= 1.0 / m_outHz;
        if (ending && g.fade <= 0.0) g = TestGen{};
        return v;
    }

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
            // A finite double beyond float range would still land as inf:
            // clamp first (far past full scale; the soft clip owns the rest).
            for (int c = 0; c < channels; ++c)
                slot[c] = (c < n && std::isfinite(samples[c])) ? static_cast<float>(std::max(-16.0, std::min(16.0, samples[c]))) : 0.0f;
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
        double grPeak = 0.0;          // gain-reduction meter, dB (audio thread)
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
    // Shaping and test signals: the audio side's working state ...
    ShakerShaper m_shaper[MAX_SHAKERS];
    TestGen      m_test[MAX_SHAKERS];
    uint32_t     m_testSeen[MAX_SHAKERS] = {};
    bool         m_snapNext = true;
    double       m_testFadeStep = 0.002, m_sweepMul = 1.0, m_toneGlide = 0.001;
    std::atomic<float> m_gr[MAX_SHAKERS] = {}, m_testHz[MAX_SHAKERS] = {};
    // ... and the web thread's stage (m_stageMx; the audio side try-locks).
    std::mutex   m_stageMx;
    ShakerShape  m_stagedShape[MAX_SHAKERS];
    bool         m_stagedBypass[MAX_SHAKERS] = {};
    TestReq      m_stagedTest[MAX_SHAKERS];
    std::atomic<bool> m_stageDirty{false};
};

} // namespace haptics
