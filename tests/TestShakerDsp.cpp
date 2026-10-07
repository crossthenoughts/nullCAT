// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
//
// TestShakerDsp - the control-rate -> 48 kHz shaker chain, no device:
//   S-1  a 40 Hz sine pushed at 2 kHz comes out at 40 Hz, near unity, with
//        no DC and no stair-steps (second-order LPF);
//   S-2  the fill servo holds the ring near half full against a producer
//        that runs 100 ppm fast, and against one 100 ppm slow;
//   S-3  an underrun (producer stops) holds and fades to silence in a few
//        ms, no click; counted once; refilling resumes;
//   S-4  a producer that floods never blocks and never corrupts (oldest
//        dropped), the ring stays bounded;
//   S-5  a constant input is blocked to zero (DC block); overdrive soft
//        clips below 1.0; per-channel gain; channels are independent.
#include "../src/ShakerDsp.h"
#include <cmath>
#include <cstdio>
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
    template <class Gen> void run(int frames, Gen gen, bool produce = true)
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
            out.insert(out.end(), blk.begin(), blk.end());
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
    // Prime: half a ring of silence so the servo starts centred.
    auto prime = [](Harness& h) { double z[haptics::MAX_SHAKERS] = {}; for (int i = 0; i < haptics::SHAKER_RING / 2; ++i) h.dsp.push(z, h.dsp.channels()); };

    // ---- S-1 ----
    {
        Harness h(2000.0, 48000.0, 2); prime(h);
        h.run(72000, [](double t, double* s) { s[0] = 0.8 * std::sin(2 * kPi * 40.0 * t); s[1] = 0.0; });
        const auto v = h.chan(0, 24000, 72000);   // one full second: +-0.5 Hz resolution
        approx(hzOf(v, 48000.0), 40.0, 0.75, "S-1 a 40 Hz control-rate sine comes out at 40 Hz");
        // 0.8 is just past the 0.7 knee: 0.7 + 0.3 tanh(0.333) = 0.796.
        approx(peak(v), 0.796, 0.02, "S-1 level near unity (0.8 in, 0.796 out past the soft knee)");
        approx(mean(v), 0.0, 0.005, "S-1 no DC on the output");
        check(maxStep(v) < 0.02, "S-1 no stair-steps (the low-pass smooths the 2 kHz feed)");
        check(rms(h.chan(1, 24000, 72000)) < 1e-4, "S-1 the silent channel stays silent");
        check(h.dsp.underruns() == 0, "S-1 no underruns with a steady producer");
    }

    // ---- S-2 ----
    {
        // The harness pushes in blocks of 10 and reads the fill at the low
        // point of that ripple, so the exact-rate run is the baseline the
        // drifted runs are judged against (not the half-ring mark).
        Harness exact(2000.0, 48000.0, 1, 1.0); prime(exact);
        exact.run(48000 * 20, [](double t, double* s) { s[0] = 0.5 * std::sin(2 * kPi * 30.0 * t); });
        const double baseFill = exact.fillSum / exact.samples, baseTrim = exact.trimSum / exact.samples;
        Harness fast(2000.0, 48000.0, 1, 1.0 + 100e-6); prime(fast);
        fast.run(48000 * 20, [](double t, double* s) { s[0] = 0.5 * std::sin(2 * kPi * 30.0 * t); });
        // Proportional servo: a fast producer parks the ring a little ABOVE
        // half (the offset that makes the trim positive); the per-block
        // producer ripple (10 samples) is larger than that offset, so judge
        // by the averages over the run, not the last sample.
        const double fastFill = fast.fillSum / fast.samples, fastTrim = fast.trimSum / fast.samples;
        check(fastFill > haptics::SHAKER_RING / 4 && fastFill < 3 * haptics::SHAKER_RING / 4, "S-2 a 100 ppm fast producer: the ring stays near half over 20 s");
        check(fastTrim > baseTrim && fastFill > baseFill + 1.0, "S-2 the servo trimmed the step up to match it (ring parked higher than at the exact rate)");
        Harness slow(2000.0, 48000.0, 1, 1.0 - 100e-6); prime(slow);
        slow.run(48000 * 20, [](double t, double* s) { s[0] = 0.5 * std::sin(2 * kPi * 30.0 * t); });
        const double slowFill = slow.fillSum / slow.samples, slowTrim = slow.trimSum / slow.samples;
        check(slowFill > haptics::SHAKER_RING / 4 && slowFill < 3 * haptics::SHAKER_RING / 4, "S-2 a 100 ppm slow producer: the ring stays near half over 20 s");
        check(slowTrim < baseTrim && slowFill < baseFill - 1.0, "S-2 the servo trimmed the step down to match it (ring parked lower than at the exact rate)");
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
        // Half a ring (128 samples at 2 kHz = 64 ms = 3072 frames) drains
        // first, then the 4 ms fade, then the 5 Hz DC block's own tail
        // (~32 ms time constant) rings down: silence by ~250 ms after the stop.
        const auto gap = h.chan(0, 24000 + 12000, 48000);
        check(peak(gap) < 1e-3, "S-3 an underrun fades to silence");
        check(h.dsp.underruns() == 1, "S-3 one underrun counted for one stall");
        const auto fade = h.chan(0, 24000, 24000 + 3600);
        check(maxStep(fade) < 0.02, "S-3 the fade has no click");
        h.run(24000, [](double t, double* s) { s[0] = 0.7 * std::sin(2 * kPi * 25.0 * t); });
        check(rms(h.chan(0, 48000 + 12000, 72000)) > 0.3, "S-3 the chain resumes when the producer returns");
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

    std::printf("\nTestShakerDsp: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
