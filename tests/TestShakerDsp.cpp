// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
//
// TestShakerDsp - the control-rate -> 48 kHz shaker chain, no device:
//   S-1  a 40 Hz sine pushed at 2 kHz comes out at 40 Hz, near unity, with
//        no DC and no stair-steps (interpolation); a 600 Hz tone is cut
//        to 0.18 and a 100 Hz one untouched (the 300 Hz low-pass);
//   S-2  the fill servo parks the ring 0.32 frames off against a producer
//        that runs 100 ppm fast, and against one 100 ppm slow (measured);
//   S-3  an underrun (producer stops) holds and fades to silence; counted
//        once; refilling resumes; S-3b pins the fade's 4 ms timing;
//   S-4  a producer that floods never blocks and never corrupts (new
//        frames dropped while full), the ring stays bounded, and the next
//        render skips to the newest frames;
//   S-5  a constant input is blocked to zero (DC block); overdrive soft
//        clips below 1.0; per-channel gain; channels are independent;
//   S-6  the ring is held at ~12 ms (and at least 2.5 device periods), not
//        half its size: a pushed edge reaches the output that soon;
//   S-7  channels stay in step: after minutes of a bursty, drifting
//        producer an edge on every channel lands on the same frame;
//   S-8  the level meter reads a tone's level whenever it is read;
//   S-9  after an audio stall that filled the ring, the next callback
//        skips to the newest frames: the latency is back at once;
//   S-10 a NaN or inf sample from the producer never poisons the filters.
// Shaping (ShakerShape.h), measured as gain against the same chain unshaped:
//   S-11 a shaker with no shaping, or shaping switched off, plays exactly
//        what the chain played before shaping existed (bit for bit);
//   S-12 each band type does what it says: a bell's gain at its centre, a
//        shelf's gain well past its corner, a pass filter's -3 dB at its
//        corner and 24 dB down two octaves out, the far side untouched;
//   S-13 the compressor: a steady tone 12 dB over a 4:1 threshold comes
//        out 9 dB down (the meter says 9 dB), makeup adds on top; attack
//        and release take their time, not none;
//   S-14 moves glide: a band's gain raised 12 dB, a band changing type,
//        the EQ switched on and off, all while a tone plays, never step
//        the output (a step through a bass shaker is a thump);
//   S-15 a card opened again keeps its shaping;
//   S-16 the editor's test signals: the sweep climbs 10 to 200 Hz on its
//        log schedule and ends, a band tone plays its frequency, follows
//        a new one without a gap, and stops on request;
//   S-17 a stage written from another thread while the card renders (the
//        page dragging a band): never a non-finite sample, the last stage
//        wins; bad values (NaN, out of range) are clamped, never played.
//   S-18 the cost: eight channels with every band and the compressor on,
//        timed against the same eight unshaped. Printed, not checked: CI
//        runs the arm64 build under emulation, many times slower than a Pi
//        (measured on a Pi 2026-10-27: 5.8 % of one core for all eight).
#include "../src/ShakerDsp.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <vector>

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const char* what)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    (ok ? g_pass : g_fail)++;
}
static void approx(double got, double want, double tol, const char* what)
{
    const bool ok = std::fabs(got - want) <= tol;
    if (!ok) std::printf("       got %.5f want %.5f +/- %.5f\n", got, want, tol);
    check(ok, what);
}

using haptics::ShakerDsp;
static const double kPi = 3.14159265358979;

// Drive the chain: every (controlHz / outHz) frames the producer pushes one
// control sample of the generator; the consumer renders in blocks of 240.
struct Harness
{
    ShakerDsp dsp;
    double controlHz, outHz, ratio;   // ratio > 1 = producer fast
    double tCtrl = 0.0;               // producer time
    double acc   = 0.0;               // producer schedule accumulator in output frames
    std::vector<float> out;
    double fillSum = 0.0, trimSum = 0.0; int samples = 0;   // per rendered block, for averages
    Harness(double cHz, double oHz, int ch, double producerRatio = 1.0) : controlHz(cHz), outHz(oHz), ratio(producerRatio)
    { dsp.configure(cHz, oHz, ch); }
    // keep = false: render without recording (a long run in a few MB).
    template <class Gen> void run(int frames, Gen gen, bool produce = true, bool keep = true)
    {
        const int ch = dsp.channels();
        std::vector<float> blk;
        int done = 0;
        while (done < frames)
        {
            const int n = std::min(240, frames - done);
            if (produce)
            {
                // Producer: as many control samples as fit in n output frames.
                acc += n * (controlHz * ratio / outHz);
                while (acc >= 1.0)
                {
                    double s[haptics::MAX_SHAKERS] = {};
                    gen(tCtrl, s);
                    dsp.push(s, ch);
                    tCtrl += 1.0 / controlHz;
                    acc -= 1.0;
                }
            }
            blk.assign(static_cast<size_t>(n * ch), 0.0f);
            dsp.render(blk.data(), n);
            if (keep) out.insert(out.end(), blk.begin(), blk.end());
            done += n;
            fillSum += dsp.fill(0); trimSum += dsp.trim(); ++samples;
        }
    }
    // Channel c samples from frame a to b.
    std::vector<double> chan(int c, size_t a, size_t b) const
    {
        std::vector<double> v; const int ch = dsp.channels();
        for (size_t i = a; i < b && i * ch + c < out.size(); ++i) v.push_back(out[i * ch + c]);
        return v;
    }
};
static double rms(const std::vector<double>& v) { double s = 0; for (double x : v) s += x * x; return v.empty() ? 0.0 : std::sqrt(s / v.size()); }
static double mean(const std::vector<double>& v) { double s = 0; for (double x : v) s += x; return v.empty() ? 0.0 : s / v.size(); }
static double peak(const std::vector<double>& v) { double m = 0; for (double x : v) m = std::max(m, std::fabs(x)); return m; }
static double hzOf(const std::vector<double>& v, double fs)
{
    int xr = 0; for (size_t i = 1; i < v.size(); ++i) if ((v[i] >= 0) != (v[i-1] >= 0)) ++xr;
    return xr / 2.0 / (v.size() / fs);
}
// Largest sample-to-sample jump: stair-steps from a 2 kHz feed would be
// ~(2 pi 40 / 2000) = 0.126 per control sample at unity; a smooth 40 Hz
// sine at 48 kHz steps 0.0052 per frame.
static double maxStep(const std::vector<double>& v) { double m = 0; for (size_t i = 1; i < v.size(); ++i) m = std::max(m, std::fabs(v[i] - v[i-1])); return m; }

int main()
{
    // Prime: the target's worth of silence so the servo starts there.
    auto prime = [](Harness& h) { double z[haptics::MAX_SHAKERS] = {}; for (int i = 0; i < h.dsp.target(); ++i) h.dsp.push(z, h.dsp.channels()); };

    // ---- S-1 ----
    {
        Harness h(2000.0, 48000.0, 2); prime(h);
        h.run(72000, [](double t, double* s) { s[0] = 0.8 * std::sin(2 * kPi * 40.0 * t); s[1] = 0.0; });
        const auto v = h.chan(0, 24000, 72000);   // one full second: +-0.5 Hz resolution
        approx(hzOf(v, 48000.0), 40.0, 0.75, "S-1 a 40 Hz control-rate sine comes out at 40 Hz");
        // 0.8 is just past the 0.7 knee: 0.7 + 0.3 tanh(0.333) = 0.796.
        approx(peak(v), 0.796, 0.02, "S-1 level near unity (0.8 in, 0.796 out past the soft knee)");
        approx(mean(v), 0.0, 0.005, "S-1 no DC on the output");
        // The step size pins the interpolation (a zero-order hold would
        // step 0.126 per control frame); the low-pass is pinned below.
        check(maxStep(v) < 0.02, "S-1 no stair-steps (linear interpolation between the 2 kHz frames)");
        check(rms(h.chan(1, 24000, 72000)) < 1e-4, "S-1 the silent channel stays silent");
        check(h.dsp.underruns() == 0, "S-1 no underruns with a steady producer");
        // The low-pass: a 600 Hz tone pushed at 2 kHz (3.3 frames per
        // cycle) comes through the 300 Hz second-order Butterworth at
        // 0.24 of its level, and the interpolator's own roll-off at 600 Hz
        // is 0.74: 0.18 together. Without the filter it would be 0.74
        // (with the interpolation images at 1400 and 2600 Hz on top), so
        // this fails if the biquad is bypassed. A 100 Hz tone, well
        // inside the band, passes at 0.98.
        Harness hi(2000.0, 48000.0, 1); prime(hi);
        hi.run(48000, [](double t, double* s) { s[0] = 0.5 * std::sin(2 * kPi * 600.0 * t); });
        const double g600 = rms(hi.chan(0, 24000, 48000)) * std::sqrt(2.0) / 0.5;
        Harness mid(2000.0, 48000.0, 1); prime(mid);
        mid.run(48000, [](double t, double* s) { s[0] = 0.5 * std::sin(2 * kPi * 100.0 * t); });
        const double g100 = rms(mid.chan(0, 24000, 48000)) * std::sqrt(2.0) / 0.5;
        std::printf("  S-1 gain at 100 Hz %.3f, at 600 Hz %.3f (low-pass 300 Hz)\n", g100, g600);
        check(g600 > 0.10 && g600 < 0.30, "S-1 the 300 Hz low-pass: a 600 Hz tone comes through at about 0.18 of its level");
        check(g100 > 0.93 && g100 < 1.03, "S-1 ...and a 100 Hz tone untouched");
    }

    // ---- S-2 ----
    {
        // The harness pushes in blocks of 10 and reads the fill at the low
        // point of that ripple, so the exact-rate run is the baseline the
        // drifted runs are judged against (not the target itself).
        Harness exact(2000.0, 48000.0, 1, 1.0); prime(exact);
        exact.run(48000 * 20, [](double t, double* s) { s[0] = 0.5 * std::sin(2 * kPi * 30.0 * t); });
        const double baseFill = exact.fillSum / exact.samples, baseTrim = exact.trimSum / exact.samples;
        Harness fast(2000.0, 48000.0, 1, 1.0 + 100e-6); prime(fast);
        fast.run(48000 * 20, [](double t, double* s) { s[0] = 0.5 * std::sin(2 * kPi * 30.0 * t); });
        // Proportional servo: a fast producer parks the ring a little ABOVE
        // the target (the offset that makes the trim positive: 100 ppm of
        // 0.5% full trim = 2% of the target, half a sample); the per-block
        // producer ripple (10 samples) is larger than that offset, so judge
        // by the averages over the run, not the last sample.
        const int tgt = exact.dsp.target();
        const double fastFill = fast.fillSum / fast.samples, fastTrim = fast.trimSum / fast.samples;
        Harness slow(2000.0, 48000.0, 1, 1.0 - 100e-6); prime(slow);
        slow.run(48000 * 20, [](double t, double* s) { s[0] = 0.5 * std::sin(2 * kPi * 30.0 * t); });
        const double slowFill = slow.fillSum / slow.samples, slowTrim = slow.trimSum / slow.samples;
        std::printf("  S-2 target %d: fill exact %.2f, fast %.2f, slow %.2f; trim exact %.1f ppm, fast %.1f, slow %.1f\n",
                    tgt, baseFill, fastFill, slowFill, 1e6 * baseTrim, 1e6 * fastTrim, 1e6 * slowTrim);
        // What the proportional servo holds, measured at the block ends the
        // harness samples (the low point of its 10-frame push ripple, so
        // the exact producer reads 4.4 frames under the target and a trim
        // of about -900 ppm there; the average over a block is the drift):
        // 100 ppm of drift parks the ring 0.32 frames off the exact
        // producer's level and moves the block-end trim by 70 ppm. The
        // band used to be half to twice the target, which a servo holding
        // nothing at all would have passed for a while.
        check(std::fabs(fastFill - (baseFill + 0.32)) < 0.15, "S-2 a 100 ppm fast producer parks the ring 0.32 frames above the exact producer's level");
        check(std::fabs((fastTrim - baseTrim) - 70e-6) < 20e-6, "S-2 ...with the block-end trim 70 ppm higher");
        check(std::fabs(slowFill - (baseFill - 0.32)) < 0.15, "S-2 a 100 ppm slow producer parks the ring 0.32 frames below");
        check(std::fabs((slowTrim - baseTrim) + 70e-6) < 20e-6, "S-2 ...with the block-end trim 70 ppm lower");
        check(std::fabs(baseFill - (tgt - 4.4)) < 0.5, "S-2 the exact producer's ring reads the target less the harness ripple's low point (4.4 frames)");
        check(exact.dsp.underruns() == 0, "S-2 an exact producer never underruns");
        check(fast.dsp.underruns() == 0 && slow.dsp.underruns() == 0, "S-2 neither drift causes an underrun");
        // A 500 Hz control loop (PC) works the same.
        Harness pc(500.0, 48000.0, 1); prime(pc);
        pc.run(48000 * 2, [](double t, double* s) { s[0] = 0.5 * std::sin(2 * kPi * 20.0 * t); });
        approx(hzOf(pc.chan(0, 48000, 96000), 48000.0), 20.0, 0.5, "S-2 a 500 Hz feed renders the same carrier");
    }

    // ---- S-3 ----
    {
        Harness h(2000.0, 48000.0, 1); prime(h);
        h.run(24000, [](double t, double* s) { s[0] = 0.7 * std::sin(2 * kPi * 25.0 * t); });
        h.run(24000, [](double, double* s) { s[0] = 0.0; }, false);   // producer stops
        // The ring (its ~12 ms target) drains first, then the 4 ms fade,
        // then the 5 Hz DC block's own tail (~32 ms time constant) rings
        // down: silence by ~250 ms after the stop.
        const auto gap = h.chan(0, 24000 + 12000, 48000);
        check(peak(gap) < 1e-3, "S-3 an underrun fades to silence");
        check(h.dsp.underruns() == 1, "S-3 one underrun counted for one stall");
        h.run(24000, [](double t, double* s) { s[0] = 0.7 * std::sin(2 * kPi * 25.0 * t); });
        check(rms(h.chan(0, 48000 + 12000, 72000)) > 0.3, "S-3 the chain resumes when the producer returns");
    }
    // ---- S-3b: the fade's timing ----
    {
        // The underrun fade is kFadeSec = 4 ms from the held level to 0.
        // Seen at the output through the 300 Hz low-pass (about 1 ms of
        // delay) and the 5 Hz DC block: 2 ms into the fade the output is
        // still most of the held level, by 6 ms it is a small residue.
        // Neither a hard cut (which the low-pass alone smears over about
        // 2 ms, so "no click" passed with the fade removed) nor a plain
        // hold (which the DC block bleeds off over 30 ms) gives both. A
        // 50 Hz tone, pushed frame by frame so the producer stops exactly
        // at a peak (the held sample is the full level), rendered in 1 ms
        // blocks from the moment the ring runs dry. Removing the fade
        // (rd.fade pinned at 1 in render) fails the 6 ms check at about
        // 0.8 of the level; an instant cut (m_fadeStep = 1) fails the
        // 2 ms check at about 0.05.
        ShakerDsp d; d.configure(2000.0, 48000.0, 1);
        const double A = 0.7;
        { double z[haptics::MAX_SHAKERS] = {}; for (int i = 0; i < d.target(); ++i) d.push(z, 1); }   // prime to the target
        auto pushN = [&](int n0, int n1) { for (int n = n0; n < n1; ++n) { double s[haptics::MAX_SHAKERS] = { A * std::sin(2 * kPi * 50.0 * n / 2000.0) }; d.push(s, 1); } };
        std::vector<float> blk(48);
        // Settle: a second of tone, pushed in tens between 240-frame renders.
        int n = 0;
        for (int b = 0; b < 200; ++b) { pushN(n, n + 10); n += 10; std::vector<float> big(240); d.render(big.data(), 240); }
        check(d.underruns() == 0, "S-3b the settled chain has not underrun");
        // Top up to the peak: 50 Hz at 2 kHz peaks every 40 frames at
        // frame 10 + 40k; push up to and including the next such frame.
        const int peakFrame = ((n + 39 - 10) / 40) * 40 + 10;
        pushN(n, peakFrame + 1); n = peakFrame + 1;
        // Render in 1 ms blocks until the ring runs dry, keeping the
        // output; then 10 more ms.
        std::vector<double> out; int dryAt = -1;
        for (int b = 0; b < 60 && dryAt < 0; ++b)
        {
            d.render(blk.data(), 48);
            for (float x : blk) out.push_back(x);
            if (d.underruns() > 0) dryAt = static_cast<int>(out.size()) - 48;   // the block in which it ran dry
        }
        check(dryAt >= 0, "S-3b the ring ran dry once the producer stopped");
        for (int b = 0; b < 10; ++b) { d.render(blk.data(), 48); for (float x : blk) out.push_back(x); }
        // The held level: the output's peak over the last 2 ms before it
        // ran dry (the tone at its peak, through the DC block).
        auto blockPeak = [&](int ms) { double m = 0; for (int i = dryAt + 48 * ms; i < dryAt + 48 * (ms + 1) && i < (int)out.size(); ++i) m = std::max(m, std::fabs(out[i])); return m; };
        double held = 0; for (int i = std::max(0, dryAt - 96); i < dryAt; ++i) held = std::max(held, std::fabs(out[i]));
        std::printf("  S-3b held %.3f; after the ring ran dry: 1 ms %.3f, 2 ms %.3f, 3 ms %.3f, 4 ms %.3f, 5 ms %.3f, 6 ms %.3f, 8 ms %.3f (of %.3f)\n",
                    held, blockPeak(1), blockPeak(2), blockPeak(3), blockPeak(4), blockPeak(5), blockPeak(6), blockPeak(8), A);
        check(held > 0.5 * A, "S-3b the producer stopped at the tone's peak (the held sample is near full level)");
        check(blockPeak(2) > 0.35 * held, "S-3b 2 ms into the underrun the output is still most of the held level (no hard cut)");
        check(blockPeak(6) < 0.30 * held, "S-3b 6 ms in it is down to a residue (the 4 ms fade, not a hold the DC block bleeds off)");
    }

    // ---- S-4 ----
    {
        Harness h(2000.0, 48000.0, 1);
        double s[haptics::MAX_SHAKERS] = { 0.5 };
        for (int i = 0; i < 10000; ++i) h.dsp.push(s, 1);
        check(h.dsp.fill(0) == haptics::SHAKER_RING, "S-4 a flooding producer never grows the ring past its size");
        std::vector<float> blk(480, 0.0f);
        h.dsp.render(blk.data(), 480);
        bool finite = true; for (float x : blk) finite = finite && std::isfinite(x) && std::fabs(x) <= 1.0f;
        check(finite, "S-4 rendering after a flood is finite and in range");
        // The first render after a flood skips to the newest frames: the
        // ring is back at its target less what the render consumed (480
        // output frames = 20 control frames), not a full ring (256 less
        // 20) of stale latency. Reverting the skip at the top of render()
        // leaves 236 here.
        std::printf("  S-4 fill after the first render following a flood: %d (target %d)\n", h.dsp.fill(0), h.dsp.target());
        check(h.dsp.fill(0) <= h.dsp.target(), "S-4 the first render after a flood skips to the newest frames (fill at or under the target)");
    }

    // ---- S-5 ----
    {
        Harness h(2000.0, 48000.0, 3); prime(h);
        h.dsp.setGain(1, 0.5);
        h.run(96000, [](double t, double* s) { s[0] = 0.6; s[1] = 0.6 * std::sin(2 * kPi * 40.0 * t); s[2] = 3.0 * std::sin(2 * kPi * 40.0 * t); });
        approx(peak(h.chan(0, 72000, 96000)), 0.0, 0.01, "S-5 a constant input is blocked to zero (no DC to the coil)");
        const double p1 = peak(h.chan(1, 72000, 96000));
        approx(p1, 0.3, 0.02, "S-5 per-channel gain 0.5 halves the drive (below the knee: untouched)");
        const double p2 = peak(h.chan(2, 72000, 96000));
        check(p2 <= 1.0 + 1e-9 && p2 > 0.98, "S-5 an overdriven input soft-clips at full scale, never above");
        approx(mean(h.chan(2, 72000, 96000)), 0.0, 0.01, "S-5 the clipped channel carries no DC");
    }

    // ---- S-6 ----
    {
        // The target: ~12 ms at 2 kHz (24-25 samples), at least 2.5 device
        // periods (a 10 ms period: 25 ms), and the same in time at 500 Hz.
        ShakerDsp d; d.configure(2000.0, 48000.0, 1); d.setPeriodFrames(240);
        check(d.target() >= 24 && d.target() <= 26, "S-6 2 kHz, 5 ms periods: the ring is held at ~12 ms (24-26 samples)");
        d.setPeriodFrames(480);
        approx(d.targetMs(), 25.0, 0.6, "S-6 10 ms periods: at least 2.5 of them");
        ShakerDsp p; p.configure(500.0, 48000.0, 1); p.setPeriodFrames(240);
        check(p.targetMs() >= 12.0 && p.targetMs() <= 14.5, "S-6 500 Hz: the same ~12 ms");
        // An edge pushed into a settled chain reaches the output within the
        // target plus the filters, not 64 ms later.
        Harness h(2000.0, 48000.0, 1); h.dsp.setPeriodFrames(240); prime(h);
        h.run(48000, [](double, double* s) { s[0] = 0.0; });   // settle on silence
        const double edgeAt = h.tCtrl;
        h.run(4800, [&](double t, double* s) { s[0] = (t >= edgeAt) ? 0.5 : 0.0; });
        const auto v = h.chan(0, 48000, 48000 + 4800);
        size_t first = v.size();
        for (size_t i = 0; i < v.size(); ++i) if (std::fabs(v[i]) > 0.1) { first = i; break; }
        const double delayMs = 1000.0 * first / 48000.0;
        std::printf("  S-6 edge delay through the ring and filters: %.1f ms (target %.1f ms)\n", delayMs, h.dsp.targetMs());
        check(delayMs < 16.0, "S-6 a pushed edge reaches the output within ~15 ms");
    }

    // ---- S-7 ----
    {
        // Channels stay in step: four channels, a producer that pushes in
        // bursts (ten frames per device period, the RT loop against a
        // 240-frame callback) and runs 100 ppm fast or slow, for three
        // minutes. Then an edge pushed on every channel in the same frame
        // must come out on the same output frame on all four, and still
        // within the ring's ~12 ms (channels 2 and up once drifted about
        // 120 ms behind channel 1 in two minutes).
        for (const double ratio : { 1.0 + 100e-6, 1.0 - 100e-6 })
        {
            Harness h(2000.0, 48000.0, 4, ratio); h.dsp.setPeriodFrames(240); prime(h);
            h.run(48000 * 180, [](double t, double* s) { for (int c = 0; c < 4; ++c) s[c] = 0.3 * std::sin(2 * kPi * (30.0 + 10.0 * c) * t); }, true, false);
            h.run(48000, [](double, double* s) { for (int c = 0; c < 4; ++c) s[c] = 0.0; });   // settle on silence
            const size_t from = h.out.size() / 4;
            const double edgeAt = h.tCtrl;
            h.run(4800, [&](double t, double* s) { for (int c = 0; c < 4; ++c) s[c] = (t >= edgeAt) ? 0.5 : 0.0; });
            size_t first[4];
            for (int c = 0; c < 4; ++c)
            {
                const auto v = h.chan(c, from, from + 4800);
                first[c] = v.size();
                for (size_t i = 0; i < v.size(); ++i) if (std::fabs(v[i]) > 0.1) { first[c] = i; break; }
            }
            const bool same = first[1] == first[0] && first[2] == first[0] && first[3] == first[0];
            const double delayMs = 1000.0 * first[0] / 48000.0;
            std::printf("  S-7 %s producer, after 3 min: edge at %.2f ms on ch1, %.2f / %.2f / %.2f on ch2-4\n",
                        ratio > 1.0 ? "fast" : "slow", delayMs, 1000.0 * first[1] / 48000.0, 1000.0 * first[2] / 48000.0, 1000.0 * first[3] / 48000.0);
            check(same, ratio > 1.0 ? "S-7 fast producer: after 3 min every channel lands on the same frame"
                                    : "S-7 slow producer: after 3 min every channel lands on the same frame");
            check(delayMs < 16.0, ratio > 1.0 ? "S-7 fast producer: still within ~15 ms after 3 min"
                                              : "S-7 slow producer: still within ~15 ms after 3 min");
        }
    }

    // ---- S-8 ----
    {
        // The level meter: a 0.5 tone on one channel reads its level at any
        // moment it is read (not one point on the wave: a status read once
        // lands anywhere from 0 to the peak), the quiet channels read
        // nothing, and the reading releases after the tone stops.
        Harness h(2000.0, 48000.0, 4); h.dsp.setPeriodFrames(240); prime(h);
        h.run(24000, [](double t, double* s) { s[1] = 0.5 * std::sin(2 * kPi * 40.0 * t); });
        double lo = 1.0, hi = 0.0, other = 0.0;
        for (int k = 0; k < 37; ++k)   // read after blocks of 37 frames: every phase of the wave
        {
            h.run(37, [](double t, double* s) { s[1] = 0.5 * std::sin(2 * kPi * 40.0 * t); });
            lo = std::min(lo, static_cast<double>(h.dsp.level(1))); hi = std::max(hi, static_cast<double>(h.dsp.level(1)));
            other = std::max({ other, static_cast<double>(h.dsp.level(0)), static_cast<double>(h.dsp.level(2)), static_cast<double>(h.dsp.level(3)) });
        }
        check(lo > 0.45 && hi < 0.52, "S-8 a 0.5 tone reads 0.5 whenever the meter is read");
        check(other < 0.01, "S-8 the silent channels read silent");
        h.run(48000 * 2, [](double, double* s) { s[1] = 0.0; });
        check(h.dsp.level(1) < 0.01, "S-8 the reading releases once the tone stops");
    }

    // ---- S-9 ----
    {
        // The audio side stalls 200 ms (the producer keeps pushing: the
        // ring fills and new frames are dropped); on waking it skips to the
        // newest, so an edge pushed right after lands within the ring's
        // ~12 ms, not a full ring (128 ms) later.
        Harness h(2000.0, 48000.0, 2); h.dsp.setPeriodFrames(240); prime(h);
        h.run(48000, [](double, double* s) { s[0] = s[1] = 0.0; });
        double z[haptics::MAX_SHAKERS] = {};
        for (int i = 0; i < 400; ++i) { h.dsp.push(z, 2); h.tCtrl += 1.0 / 2000.0; }
        check(h.dsp.fill(0) == haptics::SHAKER_RING, "S-9 a stall fills the ring (new frames dropped, none overwritten)");
        const size_t from = h.out.size() / 2;
        const double edgeAt = h.tCtrl;
        h.run(4800, [&](double t, double* s) { s[0] = s[1] = (t >= edgeAt) ? 0.5 : 0.0; });
        const auto v = h.chan(0, from, from + 4800);
        size_t first = v.size();
        for (size_t i = 0; i < v.size(); ++i) if (std::fabs(v[i]) > 0.1) { first = i; break; }
        const double delayMs = 1000.0 * first / 48000.0;
        std::printf("  S-9 edge delay right after a stall: %.1f ms\n", delayMs);
        check(delayMs < 25.0, "S-9 after a stall the next callback skips to the newest: the delay is back at once (not a full ring, 128 ms)");
    }

    // ---- S-10 ----
    {
        // One frame with a NaN on one channel and an inf on the other (a
        // law that divided by zero, a learner's first sample): the biquad
        // and the DC block are recursive, so a NaN that gets in stays in
        // for the rest of the run, on that channel, silently. The input
        // is guarded: a non-finite sample counts as 0. Reverting the
        // std::isfinite in Ring::push (ShakerDsp.h) fails every check here.
        Harness h(2000.0, 48000.0, 2); prime(h);
        auto tone = [](double t, double* s) { s[0] = 0.5 * std::sin(2 * kPi * 40.0 * t); s[1] = 0.5 * std::sin(2 * kPi * 50.0 * t); };
        h.run(24000, tone);
        double bad[haptics::MAX_SHAKERS] = { std::nan(""), INFINITY };
        h.dsp.push(bad, 2); h.tCtrl += 1.0 / 2000.0;
        // A finite double past float range (a law dividing by a tiny
        // number) is the same poison once cast: clamped before the cast.
        double huge[haptics::MAX_SHAKERS] = { 1e300, -1e300 };
        h.dsp.push(huge, 2); h.tCtrl += 1.0 / 2000.0;
        h.run(48000, tone);
        bool finite = true;
        for (size_t i = 24000 * 2; i < h.out.size(); ++i) finite = finite && std::isfinite(h.out[i]);
        check(finite, "S-10 every sample rendered after a NaN/inf frame is finite");
        const auto v0 = h.chan(0, 48000, 72000), v1 = h.chan(1, 48000, 72000);
        approx(peak(v0), 0.5, 0.03, "S-10 the channel that took the NaN plays its tone at level half a second later");
        approx(peak(v1), 0.5, 0.03, "S-10 the channel that took the inf plays its tone at level too");
        approx(hzOf(v0, 48000.0), 40.0, 1.0, "S-10 ...at the right frequency");
        check(h.dsp.level(0) > 0.45 && h.dsp.level(0) < 0.55, "S-10 the level meter reads the tone, not NaN");
    }

    // ================= shaping =================
    using haptics::ShakerShape; using haptics::BandType;
    // A shape with only band b set (EQ on), the other bands flat.
    auto oneBand = [](int b, BandType t, double hz, double db, double q)
    {
        ShakerShape s; s.eqOn = true;
        for (int i = 0; i < haptics::SHAPE_BANDS; ++i) { s.band[i].on = false; }
        s.band[b].on = true; s.band[b].type = t; s.band[b].hz = hz; s.band[b].db = db; s.band[b].q = q;
        return s;
    };
    // Gain (dB) a shape gives a tone: the shaped chain against the plain one,
    // RMS over the second half of a second (settled).
    auto gainAt = [&](const ShakerShape& s, double toneHz, double amp)
    {
        Harness plain(2000.0, 48000.0, 1), shaped(2000.0, 48000.0, 1);
        prime(plain); prime(shaped);
        shaped.dsp.stageShape(0, s, false);
        auto tone = [toneHz, amp](double t, double* v) { v[0] = amp * std::sin(2 * kPi * toneHz * t); };
        plain.run(48000, tone); shaped.run(48000, tone);
        return 20.0 * std::log10(rms(shaped.chan(0, 24000, 48000)) / rms(plain.chan(0, 24000, 48000)));
    };

    // ---- S-11 ----
    {
        // Same input through three chains: never staged, staged the default
        // (all off), and staged a full shape then switched off (glided out).
        Harness a(2000.0, 48000.0, 2), b(2000.0, 48000.0, 2), c(2000.0, 48000.0, 2);
        prime(a); prime(b); prime(c);
        b.dsp.stageShape(0, ShakerShape{}, false); b.dsp.stageShape(1, ShakerShape{}, false);
        ShakerShape on = oneBand(1, BandType::Bell, 40.0, 6.0, 1.0); on.comp.on = true;
        c.dsp.stageShape(0, on, false);
        auto gen = [](double t, double* v) { v[0] = 0.4 * std::sin(2 * kPi * 40.0 * t) + 0.1 * std::sin(2 * kPi * 7.0 * t); v[1] = 0.3 * std::sin(2 * kPi * 55.0 * t); };
        a.run(24000, gen); b.run(24000, gen); c.run(24000, gen);
        c.dsp.stageShape(0, ShakerShape{}, false);    // off again: glides out
        a.run(48000, gen); b.run(48000, gen); c.run(48000, gen);
        bool same = a.out.size() == b.out.size();
        for (size_t i = 0; same && i < a.out.size(); ++i) same = (a.out[i] == b.out[i]);
        check(same, "S-11 a shaker staged the default shaping plays exactly what an unshaped one plays (every sample)");
        bool sameAfter = true;
        for (size_t i = 60000 * 2; i < a.out.size(); ++i) sameAfter = sameAfter && (a.out[i] == c.out[i]);
        check(sameAfter, "S-11 shaping switched off: once it has faded out, the output is the unshaped chain's again, bit for bit");
    }

    // ---- S-12 ----
    {
        const double bell = gainAt(oneBand(1, BandType::Bell, 40.0, 6.0, 1.0), 40.0, 0.2);
        const double bellFar = gainAt(oneBand(1, BandType::Bell, 40.0, 6.0, 2.0), 160.0, 0.2);
        const double bellCut = gainAt(oneBand(2, BandType::Bell, 50.0, -12.0, 2.0), 50.0, 0.2);
        std::printf("  S-12 bell +6 @40: %.2f dB at 40 Hz, %.2f at 160 Hz; bell -12 @50: %.2f dB\n", bell, bellFar, bellCut);
        approx(bell, 6.0, 0.25, "S-12 a +6 dB bell at 40 Hz gives a 40 Hz tone +6 dB");
        approx(bellFar, 0.0, 0.4, "S-12 ...and leaves a tone two octaves away alone (Q 2)");
        approx(bellCut, -12.0, 0.4, "S-12 a -12 dB bell cuts its centre 12 dB (a shaker's resonance notched)");
        const double lsLow = gainAt(oneBand(0, BandType::LowShelf, 60.0, 6.0, 0.71), 10.0, 0.2);
        const double lsHigh = gainAt(oneBand(0, BandType::LowShelf, 60.0, 6.0, 0.71), 250.0, 0.2);
        const double hsHigh = gainAt(oneBand(4, BandType::HighShelf, 40.0, -6.0, 0.71), 200.0, 0.2);
        const double hsLow = gainAt(oneBand(4, BandType::HighShelf, 40.0, -6.0, 0.71), 10.0, 0.2);
        std::printf("  S-12 low shelf +6 @60: %.2f dB at 10 Hz, %.2f at 250; high shelf -6 @40: %.2f at 200, %.2f at 10\n", lsLow, lsHigh, hsHigh, hsLow);
        approx(lsLow, 6.0, 0.7, "S-12 a +6 dB low shelf at 60 Hz lifts 10 Hz by 6 dB");
        approx(lsHigh, 0.0, 0.7, "S-12 ...and leaves 250 Hz alone");
        approx(hsHigh, -6.0, 0.7, "S-12 a -6 dB high shelf at 40 Hz takes 6 dB off 200 Hz");
        approx(hsLow, 0.0, 0.7, "S-12 ...and leaves 10 Hz alone");
        const double hpCorner = gainAt(oneBand(0, BandType::HighPass, 40.0, 0.0, 0.7071), 40.0, 0.2);
        const double hpBelow = gainAt(oneBand(0, BandType::HighPass, 40.0, 0.0, 0.7071), 10.0, 0.2);
        const double hpAbove = gainAt(oneBand(0, BandType::HighPass, 40.0, 0.0, 0.7071), 200.0, 0.2);
        const double lpCorner = gainAt(oneBand(4, BandType::LowPass, 60.0, 0.0, 0.7071), 60.0, 0.2);
        const double lpAbove = gainAt(oneBand(4, BandType::LowPass, 60.0, 0.0, 0.7071), 240.0, 0.2);
        std::printf("  S-12 high-pass 40: %.2f dB at 40, %.2f at 10, %.2f at 200; low-pass 60: %.2f at 60, %.2f at 240\n", hpCorner, hpBelow, hpAbove, lpCorner, lpAbove);
        approx(hpCorner, -3.0, 0.3, "S-12 a high-pass at 40 Hz is 3 dB down at 40 Hz");
        approx(hpBelow, -24.0, 1.5, "S-12 ...and 24 dB down two octaves below (12 dB an octave)");
        approx(hpAbove, 0.0, 0.3, "S-12 ...and passes 200 Hz");
        approx(lpCorner, -3.0, 0.3, "S-12 a low-pass at 60 Hz is 3 dB down at 60 Hz");
        approx(lpAbove, -24.0, 1.5, "S-12 ...and 24 dB down two octaves above");
    }

    // ---- S-13 ----
    {
        ShakerShape s; s.comp.on = true; s.comp.thresholdDb = -18.0; s.comp.ratio = 4.0; s.comp.kneeDb = 0.0;
        s.comp.attackMs = 5.0; s.comp.releaseMs = 300.0; s.comp.makeupDb = 0.0;
        // A 40 Hz tone at 0.5 (-6 dBFS): 12 dB over, 4:1 -> 3 dB over, out at -15 dBFS.
        Harness h(2000.0, 48000.0, 1); prime(h); h.dsp.stageShape(0, s, false);
        auto loud = [](double t, double* v) { v[0] = 0.5 * std::sin(2 * kPi * 40.0 * t); };
        h.run(48000, loud);
        const double outDb = 20.0 * std::log10(peak(h.chan(0, 24000, 48000)));
        std::printf("  S-13 4:1 at -18: a -6 dBFS tone comes out at %.2f dBFS, meter %.2f dB\n", outDb, h.dsp.gainReductionDb(0));
        approx(outDb, -15.0, 0.6, "S-13 a tone 12 dB over a 4:1 threshold comes out 3 dB over it");
        approx(h.dsp.gainReductionDb(0), 9.0, 0.6, "S-13 ...and the gain-reduction meter reads the 9 dB taken off");
        ShakerShape mk = s; mk.comp.makeupDb = 6.0;
        Harness m(2000.0, 48000.0, 1); prime(m); m.dsp.stageShape(0, mk, false);
        m.run(48000, loud);
        approx(20.0 * std::log10(peak(m.chan(0, 24000, 48000))), -9.0, 0.6, "S-13 6 dB of makeup puts it back up 6 dB");
        ShakerShape quiet = s;
        Harness q(2000.0, 48000.0, 1), qp(2000.0, 48000.0, 1); prime(q); prime(qp); q.dsp.stageShape(0, quiet, false);
        auto soft = [](double t, double* v) { v[0] = 0.05 * std::sin(2 * kPi * 40.0 * t); };   // -26 dBFS, under
        q.run(48000, soft); qp.run(48000, soft);
        approx(peak(q.chan(0, 24000, 48000)) / peak(qp.chan(0, 24000, 48000)), 1.0, 0.01, "S-13 a tone under the threshold passes untouched");

        // Timing: quiet, then loud (attack 10 ms), then quiet again (release 200 ms).
        ShakerShape tm = s; tm.comp.attackMs = 10.0; tm.comp.releaseMs = 200.0;
        Harness t(2000.0, 48000.0, 1); prime(t); t.dsp.stageShape(0, tm, false);
        auto stepTone = [](double tt, double* v) { const double a = (tt >= 1.0 && tt < 2.0) ? 0.5 : 0.05; v[0] = a * std::sin(2 * kPi * 40.0 * tt); };
        t.run(48000 * 3, stepTone);
        // Where the producer's t = 1.0 lands in the output: the ring's delay.
        size_t edge = 0; { const auto v = t.chan(0, 40000, 60000); for (size_t i = 0; i < v.size(); ++i) if (std::fabs(v[i]) > 0.1) { edge = 40000 + i; break; } }
        const double firstCycle  = peak(t.chan(0, edge, edge + 1200));          // the first 25 ms of loud
        const double settledLoud = peak(t.chan(0, edge + 24000, edge + 36000));
        // Right after the loud passage the quiet tone comes in with the DC
        // block still settling from the drop, so it is measured against the
        // unshaped chain at the same moment, not against its settled level.
        Harness pl(2000.0, 48000.0, 1); prime(pl); pl.run(48000 * 3, stepTone);
        const double justAfter   = peak(t.chan(0, edge + 48000 + 600, edge + 48000 + 1800));    // 12-37 ms into quiet
        const double plainAfter  = peak(pl.chan(0, edge + 48000 + 600, edge + 48000 + 1800));
        const double settledQuiet = peak(t.chan(0, edge + 48000 + 36000, edge + 48000 + 48000));
        std::printf("  S-13 timing: loud first 25 ms %.3f, settled %.3f; quiet just after %.4f (unshaped %.4f), settled %.4f\n", firstCycle, settledLoud, justAfter, plainAfter, settledQuiet);
        check(edge > 0, "S-13 the loud passage found in the output");
        check(firstCycle > 1.3 * settledLoud, "S-13 attack takes time: the first cycle of a loud passage gets through above the settled level");
        check(justAfter < 0.6 * plainAfter, "S-13 release takes time: right after the loud passage the quiet tone is still held down (more than 4 dB)");
        approx(settledQuiet, 0.05, 0.005, "S-13 ...and after the release the quiet tone is back at its own level");
    }

    // ---- S-14 ----
    {
        // A 40 Hz tone at 0.1; moves made mid-run. A smooth 40 Hz sine at
        // amplitude A steps at most 2 pi 40 A / 48000 per frame; a move that
        // jumped would step by a large share of A at once.
        auto worstStepRatio = [&](const ShakerShape& before, const ShakerShape& after, double settledAmp)
        {
            Harness h(2000.0, 48000.0, 1); prime(h); h.dsp.stageShape(0, before, false);
            auto tone = [](double t, double* v) { v[0] = 0.1 * std::sin(2 * kPi * 40.0 * t); };
            h.run(24000, tone);
            h.dsp.stageShape(0, after, false);
            h.run(24000, tone);
            const double smooth = 2 * kPi * 40.0 * settledAmp / 48000.0;
            return maxStep(h.chan(0, 23000, 48000)) / smooth;
        };
        ShakerShape flat = oneBand(1, BandType::Bell, 40.0, 0.0, 1.0);
        ShakerShape up12 = oneBand(1, BandType::Bell, 40.0, 12.0, 1.0);
        const double rGain = worstStepRatio(flat, up12, 0.1 * 3.98);
        ShakerShape hp = oneBand(1, BandType::HighPass, 30.0, 0.0, 0.71);
        const double rType = worstStepRatio(up12, hp, 0.1 * 3.98);
        ShakerShape off = up12; off.eqOn = false;
        const double rOn  = worstStepRatio(off, up12, 0.1 * 3.98);
        const double rOff = worstStepRatio(up12, off, 0.1 * 3.98);
        std::printf("  S-14 worst step vs a smooth sine: gain +12 %.2f, type swap %.2f, EQ on %.2f, EQ off %.2f\n", rGain, rType, rOn, rOff);
        check(rGain < 1.3, "S-14 a band raised 12 dB while a tone plays glides: no step in the output");
        check(rType < 1.3, "S-14 a band changing type fades out and back in: no step");
        check(rOn < 1.3 && rOff < 1.3, "S-14 the EQ switched on, and off, cross-fades: no step");
        // The same move without the glide is what the check is for: snapped
        // (a card opened with the shape already staged), the gain jumps.
        Harness j(2000.0, 48000.0, 1); prime(j); j.dsp.stageShape(0, flat, false);
        auto tone = [](double t, double* v) { v[0] = 0.1 * std::sin(2 * kPi * 40.0 * t); };
        j.run(24000, tone);
        j.dsp.stageShape(0, up12, false); j.dsp.configure(2000.0, 48000.0, 1); prime(j);
        j.run(24000, tone);
        approx(peak(j.chan(0, 36000, 48000)), 0.1 * 3.98, 0.03, "S-14 (a snapped shape lands at its full gain at once)");
    }

    // ---- S-15 ----
    {
        ShakerShape s = oneBand(1, BandType::Bell, 40.0, 6.0, 1.0);
        Harness h(2000.0, 48000.0, 1); prime(h); h.dsp.stageShape(0, s, false);
        auto tone = [](double t, double* v) { v[0] = 0.1 * std::sin(2 * kPi * 40.0 * t); };
        h.run(24000, tone);
        h.dsp.configure(2000.0, 44100.0, 1); h.outHz = 44100.0; prime(h);   // opened again, at another rate
        h.out.clear();
        h.run(44100, tone);
        approx(peak(h.chan(0, 22050, 44100)), 0.1 * 2.0, 0.01, "S-15 a card opened again (at another rate) keeps its shaping: +6 dB still");
    }

    // ---- S-16 ----
    {
        Harness h(2000.0, 48000.0, 1); prime(h);
        auto silent = [](double, double* v) { v[0] = 0.0; };
        h.run(4800, silent);
        h.dsp.startTest(0, ShakerDsp::TestSweep, 0.0);
        h.run(48000 * 5, silent);
        const double hzAt5 = h.dsp.testHz(0);
        const double want5 = 10.0 * std::pow(20.0, 0.5);
        const double measured = hzOf(h.chan(0, 4800 + 48000 * 5 - 9600, 4800 + 48000 * 5), 48000.0);
        std::printf("  S-16 sweep at 5 s: %.1f Hz reported, %.1f measured (log schedule %.1f)\n", hzAt5, measured, want5);
        approx(hzAt5, want5, 1.0, "S-16 halfway through the sweep it is at the geometric middle of 10-200 Hz");
        approx(measured, want5, 3.0, "S-16 ...and that is what plays");
        approx(peak(h.chan(0, 4800 + 48000 * 4, 4800 + 48000 * 5)), 0.5, 0.05, "S-16 at the test burst's level");
        h.run(48000 * 6, silent);
        check(h.dsp.testHz(0) == 0.0f, "S-16 the sweep ends by itself");
        check(peak(h.chan(0, h.out.size() - 4800, h.out.size())) < 1e-3, "S-16 ...in silence");

        Harness t(2000.0, 48000.0, 1); prime(t);
        t.run(4800, silent);
        t.dsp.startTest(0, ShakerDsp::TestTone, 60.0);
        t.run(48000, silent);
        approx(hzOf(t.chan(0, 4800 + 24000, 4800 + 48000), 48000.0), 60.0, 1.0, "S-16 a band tone plays the band's frequency");
        t.dsp.startTest(0, ShakerDsp::TestTone, 80.0);   // the band dragged
        t.run(48000, silent);
        approx(hzOf(t.chan(0, 4800 + 48000 + 24000, 4800 + 96000), 48000.0), 80.0, 1.0, "S-16 ...follows the band when it moves");
        check(maxStep(t.chan(0, 4800 + 47000, 4800 + 52000)) < 1.3 * 2 * kPi * 80.0 * 0.5 / 48000.0, "S-16 ...without a jump (one tone gliding, not a restart)");
        t.dsp.startTest(0, ShakerDsp::TestTone, 80.0);   // again at 2 s: it plays on past its first 3 s
        t.run(48000 * 5 / 2, silent);
        check(t.dsp.testHz(0) > 0.0f, "S-16 asked again, the tone plays on past its first three seconds");
        t.dsp.startTest(0, ShakerDsp::TestStop, 0.0);
        t.run(960, silent);
        check(peak(t.chan(0, t.out.size() - 240, t.out.size())) < 0.01 && t.dsp.testHz(0) == 0.0f, "S-16 stop fades it out within 20 ms (34 dB down; what is left is the DC block settling)");
    }

    // ---- S-17 ----
    {
        Harness h(2000.0, 48000.0, 2); prime(h);
        auto tone = [](double t, double* v) { v[0] = 0.2 * std::sin(2 * kPi * 40.0 * t); v[1] = 0.2 * std::sin(2 * kPi * 40.0 * t); };
        std::atomic<bool> go{true};
        std::thread page([&h, &go, oneBand]()
        {
            int i = 0;
            while (go.load()) { h.dsp.stageShape(0, oneBand(1, BandType::Bell, 30.0 + (i % 50), (i % 13) - 6.0, 1.0), (i % 7) == 0); ++i; }
        });
        h.run(48000 * 2, tone);
        go.store(false); page.join();
        h.dsp.stageShape(0, oneBand(1, BandType::Bell, 40.0, 6.0, 1.0), false);
        h.out.clear();
        h.run(48000, tone);
        bool finite = true; for (float v : h.out) finite = finite && std::isfinite(v);
        check(finite, "S-17 a stage written from another thread while the card renders: every sample finite");
        approx(peak(h.chan(0, 24000, 48000)), 0.4, 0.02, "S-17 ...and the last stage is what plays");
        ShakerShape bad; bad.eqOn = true; bad.band[1].hz = std::nan(""); bad.band[1].db = 1e9; bad.band[2].q = -3.0;
        bad.comp.on = true; bad.comp.ratio = INFINITY; bad.comp.attackMs = 0.0;
        ShakerShape clean = bad; check(haptics::sanitize(clean), "S-17 out-of-range and non-finite values are found");
        check(clean.band[1].hz == ShakerShape{}.band[1].hz && clean.band[1].db == haptics::shape_k::kDbMax && clean.band[2].q == haptics::shape_k::kQMin
              && clean.comp.ratio == ShakerShape{}.comp.ratio && clean.comp.attackMs == haptics::shape_k::kAttackMin,
              "S-17 ...and clamped (NaN takes the default, the rest the nearest limit)");
        Harness b(2000.0, 48000.0, 1); prime(b); b.dsp.stageShape(0, bad, false);
        b.run(48000, tone);
        bool fin2 = true; for (float v : b.out) fin2 = fin2 && std::isfinite(v);
        check(fin2 && peak(b.chan(0, 24000, 48000)) <= 1.0, "S-17 a shape with bad values staged straight to the card plays finite and bounded");
    }

    // ---- S-18 ----
    {
        ShakerShape full; full.eqOn = true;
        for (int i = 0; i < haptics::SHAPE_BANDS; ++i) full.band[i].on = true;
        full.band[1].db = -6.0; full.band[2].db = 3.0; full.band[3].db = -3.0;
        full.comp.on = true; full.comp.thresholdDb = -24.0;
        auto timeIt = [&](bool shaped)
        {
            Harness h(2000.0, 48000.0, 8); prime(h);
            if (shaped) for (int c = 0; c < 8; ++c) h.dsp.stageShape(c, full, false);
            auto gen = [](double t, double* v) { for (int c = 0; c < 8; ++c) v[c] = 0.4 * std::sin(2 * kPi * (30.0 + 5 * c) * t); };
            const auto t0 = std::chrono::steady_clock::now();
            h.run(48000 * 10, gen, true, false);   // ten seconds of audio, not kept
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        };
        const double plain = timeIt(false), shaped = timeIt(true);
        std::printf("  S-18 ten seconds of eight channels: %.1f ms unshaped, %.1f ms fully shaped (%.2f %% of a core for the shaping)\n",
                    plain * 1000.0, shaped * 1000.0, (shaped - plain) / 10.0 * 100.0);
    }

    std::printf("\nTestShakerDsp: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
