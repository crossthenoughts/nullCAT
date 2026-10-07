// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestHaptics.cpp - unit tests for the haptic transient layer and the
// force model's detent-capture trigger.
//
// Pure-logic suite (no drives, no Qt Test). Pins: fire/step/overlay
// lifecycle, envelope boundedness (zero start and end, bounded middle),
// routing gains across explicit axes, amp-0 inertness,
// pool exhaustion behaviour (drop, never block), clearAll, and the
// edge-triggered detent capture in DeviceForceModel (fires once on entry,
// never on dwell, re-fires on re-entry, never on the seed step, and
// works with an empty detent CURVE - a pure-haptic gate).
// ============================================================

#include "../src/HapticsLayer.h"
#include "../src/DeviceForceModel.h"
#include <cstdio>
#include <cmath>
#include <tuple>
#include <utility>
#include <vector>

static int g_pass = 0, g_fail = 0;
static void CHECK(bool ok, const char* what)
{
    if (ok) { ++g_pass; }
    else    { ++g_fail; std::printf("FAIL: %s\n", what); }
}
static void approx(double got, double want, double tol, const char* what)
{
    if (std::fabs(got - want) <= tol) { ++g_pass; }
    else { ++g_fail; std::printf("FAIL: %s (got %.5f, want %.5f +/- %.5f)\n", what, got, want, tol); }
}

using haptics::Layer;
using haptics::EffectParams;
using haptics::EngineParams;
// Engine tests bundle the shared params with the engine description.
struct EP : EffectParams { EngineParams eng; };
using haptics::EventType;

static EffectParams click(double amp = 10.0, double freq = 100.0, double ms = 20.0)
{
    EffectParams p;
    p.ampPct = amp; p.freqHz = freq; p.durMs = ms;
    p.routes[0] = { 3, 1.0 };   // struct default is EMPTY routes: route to axis 3
    return p;
}

int main()
{
    const double DT = 0.0005;   // 2 kHz

    // ================= lifecycle + envelope =================
    {
        Layer L;
        L.configure(EventType::DetentClick, click());
        CHECK(!L.anyActive(), "fresh layer: nothing active");
        L.fire(EventType::DetentClick);
        CHECK(L.anyActive(), "fire activates an event");
        CHECK(L.fireCount() == 1, "fireCount counts");

        // First step: envelope is ~0 at t->0 (raised cosine), never a snap.
        L.step(DT);
        CHECK(std::fabs(L.overlayFor(3)) < 1.0, "no torque snap on the first cycle");

        // Mid-burst: overlay must actually appear, bounded by amp.
        double peak = 0.0;
        for (int i = 0; i < 40; ++i) { L.step(DT); peak = std::max(peak, std::fabs(L.overlayFor(3))); }
        CHECK(peak > 1.0, "burst produces torque mid-envelope");
        CHECK(peak <= 10.0 + 1e-9, "burst never exceeds configured amplitude");

        // After duration: inactive, overlay exactly zero.
        for (int i = 0; i < 40; ++i) L.step(DT);
        CHECK(!L.anyActive(), "event expires after durMs");
        CHECK(L.overlayFor(3) == 0.0, "expired event contributes exactly zero");
    }

    // ================= routing =================
    {
        Layer L;
        EffectParams p = click(20.0);
        p.routes[0] = { 2, 1.0 };                   // full gain
        p.routes[1] = { 5, 0.5 };                   // belt at half gain
        L.configure(EventType::DetentClick, p);
        L.fire(EventType::DetentClick);
        double own = 0.0, belt = 0.0;
        for (int i = 0; i < 30; ++i)
        {
            L.step(DT);
            own  = std::max(own,  std::fabs(L.overlayFor(2)));
            belt = std::max(belt, std::fabs(L.overlayFor(5)));
        }
        CHECK(own > 1.0,                        "full-gain route receives the burst");
        CHECK(belt > 0.5,                       "explicit route receives the burst");
        CHECK(std::fabs(belt - own * 0.5) < 0.2, "route gain scales the burst");
        CHECK(L.overlayFor(0) == 0.0,           "unrouted axis stays silent");
    }

    // ================= inertness + scale + bad input =================
    {
        Layer L;                                    // default config: amp 0
        L.fire(EventType::DetentClick);
        CHECK(!L.anyActive(), "amp 0 (the shipped default) is fully inert");

        L.configure(EventType::DetentClick, click());
        L.fire(EventType::DetentClick, 0.0);
        CHECK(!L.anyActive(), "scale 0 fires nothing");
        L.fire(EventType::DetentClick, 7.0);        // clamps to 1
        double peak = 0.0;
        for (int i = 0; i < 40; ++i) { L.step(DT); peak = std::max(peak, std::fabs(L.overlayFor(3))); }
        CHECK(peak > 1.0 && peak <= 10.0 + 1e-9, "scale clamps at 1.0");

        EffectParams unrouted = click();
        unrouted.routes[0] = { -1, 0.0 };
        Layer L2; L2.configure(EventType::DetentClick, unrouted);
        L2.fire(EventType::DetentClick);
        CHECK(!L2.anyActive(), "fully unrouted event never activates");

        CHECK(L.overlayFor(-1) == 0.0 && L.overlayFor(99) == 0.0,
              "out-of-range axis reads zero, no crash");
    }

    // ================= pool exhaustion + clearAll =================
    {
        Layer L;
        L.configure(EventType::DetentClick, click(10.0, 100.0, 50.0));
        for (int i = 0; i < 20; ++i) L.fire(EventType::DetentClick);
        CHECK(L.fireCount() == haptics::MAX_EVENTS,
              "pool full: extra fires drop (never block, never overwrite)");
        L.clearAll();
        CHECK(!L.anyActive(), "clearAll kills every event");
        L.step(DT);
        CHECK(L.overlayFor(1) == 0.0, "cleared pool contributes zero");
    }

    // ================= continuous effects =================
    {
        using haptics::FxType;
        Layer L;
        EffectParams p;
        p.ampPct = 10.0; p.freqHz = 35.0; p.jitter = 0.0;
        p.routes[0] = { 4, 1.0 };                    // explicit axis
        L.configureFx(FxType::Road, p);

        // Undriven: silent.
        L.step(DT);
        CHECK(L.overlayFor(4) == 0.0, "fx undriven: silent");

        // Driven at full level: ramps in (attack), reaches near amp.
        double peak = 0.0;
        for (int i = 0; i < 400; ++i)                // 200 ms at 2 kHz
        { L.driveFx(FxType::Road, 1.0, 0.0); L.step(DT); peak = std::max(peak, std::fabs(L.overlayFor(4))); }
        CHECK(peak > 8.0, "fx reaches near full amplitude when driven");
        CHECK(peak <= 10.0 + 1e-9, "fx never exceeds configured amplitude");

        // First cycles after drive start must be small (attack ramp).
        Layer L2; L2.configureFx(FxType::Road, p);
        L2.driveFx(FxType::Road, 1.0, 0.0); L2.step(DT);
        CHECK(std::fabs(L2.overlayFor(4)) < 1.0, "fx attack ramps, never snaps on");

        // Stop driving: release ramp decays to silence (fail-safe fade).
        for (int i = 0; i < 800; ++i) { L.driveFx(FxType::Road, 0.0, 0.0); L.step(DT); }
        CHECK(L.overlayFor(4) == 0.0 || std::fabs(L.overlayFor(4)) < 0.05,
              "fx released: fades to silence");

        // Half level scales.
        Layer L3; L3.configureFx(FxType::Road, p);
        double half = 0.0;
        for (int i = 0; i < 400; ++i)
        { L3.driveFx(FxType::Road, 0.5, 0.0); L3.step(DT); half = std::max(half, std::fabs(L3.overlayFor(4))); }
        CHECK(half > 3.0 && half < 6.0, "fx level scales amplitude");

        // Amp 0 config: driven or not, silent.
        Layer L4;                                     // defaults: amp 0
        L4.driveFx(FxType::Road, 1.0, 0.0); L4.step(DT);
        CHECK(!std::fabs(L4.overlayFor(4)), "fx amp 0 (shipped default) is inert");

        // No usable carrier (freq 0 config, none driven): silence, not DC.
        EffectParams pz = p; pz.freqHz = 0.0;
        Layer L5; L5.configureFx(FxType::Road, pz);
        for (int i = 0; i < 200; ++i) { L5.driveFx(FxType::Road, 1.0, 0.0); L5.step(DT); }
        CHECK(L5.overlayFor(4) == 0.0, "fx with no carrier is silent, never DC");

        // Driven frequency is honored (Lockup slot; the RpmVibe slot is
        // the pulse-train engine and ignores a driven carrier by design).
        EffectParams pr; pr.ampPct = 10.0; pr.freqHz = 0.0; pr.routes[0] = { 2, 1.0 };
        Layer L6; L6.configureFx(FxType::Kerb, pr);
        double vibe = 0.0;
        for (int i = 0; i < 400; ++i)
        { L6.driveFx(FxType::Kerb, 1.0, 120.0); L6.step(DT); vibe = std::max(vibe, std::fabs(L6.overlayFor(2))); }
        CHECK(vibe > 8.0, "driven carrier frequency is honored");
        L6.clearAll(); L6.step(DT);
        CHECK(L6.overlayFor(2) == 0.0, "clearAll silences continuous effects instantly");

        // Jitter keeps the output bounded.
        EffectParams pj = p; pj.jitter = 1.0;
        Layer L7; L7.configureFx(FxType::Road, pj);
        double jp = 0.0;
        for (int i = 0; i < 400; ++i)
        { L7.driveFx(FxType::Road, 1.0, 0.0); L7.step(DT); jp = std::max(jp, std::fabs(L7.overlayFor(4))); }
        CHECK(jp > 5.0 && jp <= 10.0 + 1e-9, "full jitter stays bounded by amp");
    }

    // ================= pulse-train engine =================
    {
        using haptics::FxType;
        EP p;
        p.ampPct = 10.0; p.jitter = 0.0; p.eng.cylinders = 8.0; p.eng.litres = 4.0;   // 0.5 L/cyl = unit weight
        p.freqHz = 30.0;                                                   // the shipped thump carrier
        p.routes[0] = { 6, 1.0 };

        // Run one second of the engine at a firing rate (V8: fireHz =
        // rpm/60 x 4) and return the raw peak plus a 20 ms moving average,
        // which strips the 30 Hz thumps and leaves the crank-rate rock.
        struct Run { double peak = 0.0; std::vector<double> lp; };
        auto run = [&](double fireHz, double load, bool lim) {
            Layer L; L.configureFx(FxType::RpmVibe, p); L.configureEngine(p.eng);
            Run r; std::vector<double> raw;
            for (int i = 0; i < 2000; ++i)                  // 1 s at 2 kHz
            {
                L.driveEngine(1.0, fireHz, load, lim);
                L.step(DT);
                const double v = L.overlayFor(6);
                raw.push_back(v);
                r.peak = std::max(r.peak, std::fabs(v));
            }
            for (size_t i = 40; i < raw.size(); ++i)
            { double s = 0.0; for (size_t k = i - 40; k < i; ++k) s += raw[k]; r.lp.push_back(s / 40.0); }
            return r;
        };
        auto crossings = [](const std::vector<double>& x) {
            int n = 0; for (size_t i = 1; i < x.size(); ++i) if ((x[i] >= 0.0) != (x[i-1] >= 0.0)) ++n; return n; };
        auto lpEnergy = [](const std::vector<double>& x) {
            double e = 0.0; for (double v : x) e += std::fabs(v); return e; };

        // V8 idle, 800 rpm: fireHz 53.3, crank 13.3 Hz. The felt content is
        // the crank-rate rock: the low-passed signal crosses zero ~27 times
        // a second (2 per rev), not at the 53 Hz firing rate.
        const Run idle = run(53.3, 1.0, false);
        CHECK(idle.peak > 5.0,            "engine produces torque at idle");
        CHECK(idle.peak <= 10.0 + 1e-9,   "engine bounded by its own amp");
        const int xIdle = crossings(idle.lp);
        CHECK(xIdle >= 20 && xIdle <= 34, "idle rock runs at crank rate (~27 crossings/s), not firing rate");

        // 6000 rpm: fireHz 400, crank 100 Hz. The rock is gone, the thumps
        // merge into a continuous buzz with no long gaps, still bounded.
        const Run high = run(400.0, 1.0, false);
        CHECK(high.peak <= 10.0 + 1e-9,   "high rpm still bounded by amp (overlap normalised)");
        CHECK(lpEnergy(high.lp) < 0.3 * lpEnergy(idle.lp), "rock fades out by high rpm");
        {
            Layer Lh; Lh.configureFx(FxType::RpmVibe, p); Lh.configureEngine(p.eng);
            int g = 0, maxGap = 0;
            for (int i = 0; i < 2000; ++i)
            {
                Lh.driveEngine(1.0, 400.0, 1.0, false); Lh.step(DT);
                if (i > 200 && std::fabs(Lh.overlayFor(6)) < 0.01) { ++g; maxGap = std::max(maxGap, g); }
                else g = 0;
            }
            CHECK(maxGap <= 4, "high rpm: no long gaps - pulses merge into buzz");
        }

        // Fewer cylinders = sparser firings, same character: a 4-cyl at
        // 800 rpm (fireHz 26.7) still rocks at the same crank rate.
        {
            EP p4 = p; p4.eng.cylinders = 4.0;
            Layer L4; L4.configureFx(FxType::RpmVibe, p4); L4.configureEngine(p4.eng);
            std::vector<double> raw, lp;
            for (int i = 0; i < 2000; ++i) { L4.driveEngine(1.0, 26.7, 1.0, false); L4.step(DT); raw.push_back(L4.overlayFor(6)); }
            for (size_t i = 40; i < raw.size(); ++i)
            { double s = 0.0; for (size_t k = i - 40; k < i; ++k) s += raw[k]; lp.push_back(s / 40.0); }
            const int x4 = crossings(lp);
            CHECK(x4 >= 20 && x4 <= 34,   "4-cyl idle rocks at the same crank rate");
        }

        // Component mix: rock 0 leaves only thumps (low-passed signal
        // collapses), thump 0 leaves only the rock (no gaps, no 30 Hz
        // content), both 0 is silence. Set-by-feel on the tile.
        {
            auto lpOf = [&](const EP& q, double fireHz) {
                Layer L; L.configureFx(FxType::RpmVibe, q); L.configureEngine(q.eng);
                std::vector<double> raw, lp; double pk = 0.0;
                for (int i = 0; i < 2000; ++i) { L.driveEngine(1.0, fireHz, 1.0, false); L.step(DT);
                    raw.push_back(L.overlayFor(6)); pk = std::max(pk, std::fabs(raw.back())); }
                for (size_t i = 40; i < raw.size(); ++i)
                { double s = 0.0; for (size_t k = i - 40; k < i; ++k) s += raw[k]; lp.push_back(s / 40.0); }
                return std::make_pair(pk, lpEnergy(lp));
            };
            EP noRock = p;  noRock.eng.rock  = 0.0;
            EP noThump = p; noThump.eng.thump = 0.0;
            EP none = p;    none.eng.rock = 0.0; none.eng.thump = 0.0;
            const auto full = lpOf(p, 53.3), nr = lpOf(noRock, 53.3), nt = lpOf(noThump, 53.3), z = lpOf(none, 53.3);
            CHECK(nr.second < 0.15 * full.second, "rock x 0: crank-rate content gone, thumps remain");
            CHECK(nr.first > 2.0,                 "rock x 0: thumps still produce torque");
            CHECK(nt.second > 0.8 * full.second,  "thump x 0: the rock is untouched");
            CHECK(z.first == 0.0,                 "rock 0 + thump 0: silence");
        }

        // The rest of the rev range: at 4000 rpm (V8 fireHz 267, crank 67 Hz)
        // the buzz carries the engine - real level, carrier at crank x order
        // (order 1 here: 67 Hz, ~134 raw zero crossings/s), rising with
        // throttle; and it is silent at idle so the rock is untouched.
        {
            EP pb = p; pb.eng.order = 1.0; pb.eng.thump = 0.0; pb.eng.rock = 0.0;   // buzz alone
            auto buzzRun = [&](double fireHz, double load) {
                Layer L; L.configureFx(FxType::RpmVibe, pb); L.configureEngine(pb.eng);
                std::vector<double> raw; double pk = 0.0;
                for (int i = 0; i < 2000; ++i) { L.driveEngine(1.0, fireHz, load, false); L.step(DT);
                    raw.push_back(L.overlayFor(6)); pk = std::max(pk, std::fabs(raw.back())); }
                return std::make_pair(pk, crossings(raw));
            };
            const auto mid = buzzRun(266.7, 1.0), midCoast = buzzRun(266.7, 0.0), idleB = buzzRun(53.3, 1.0);
            CHECK(mid.first > 3.0,                       "4000 rpm: the buzz carries real level");
            CHECK(mid.second >= 110 && mid.second <= 160, "4000 rpm: carrier at crank x order (67 Hz)");
            CHECK(midCoast.first < mid.first * 0.7,      "buzz grows with throttle");
            CHECK(idleB.first < 0.5,                     "buzz is silent at idle (rock territory)");
            EP p2 = pb; p2.eng.order = 2.0;
            Layer L2; L2.configureFx(FxType::RpmVibe, p2); L2.configureEngine(p2.eng);
            std::vector<double> r2;
            for (int i = 0; i < 2000; ++i) { L2.driveEngine(1.0, 266.7, 1.0, false); L2.step(DT); r2.push_back(L2.overlayFor(6)); }
            const int x2 = crossings(r2);
            CHECK(x2 >= 230 && x2 <= 310,                "buzz order 2 doubles the carrier (133 Hz)");
        }

        // Redline scaling. Auto order puts the redline at the top of the
        // band whatever the engine revs to; max rpm is set on the tile or
        // LEARNED (peak hold, limiter snap); level keeps building to the top;
        // thumps fade out once firings cannot be resolved.
        {
            auto buzzOnly = [&](double cyl, double maxRpm, double order, double rpm, bool lim, int warmCycles) {
                EP q = p; q.eng.cylinders = cyl; q.eng.maxRpm = maxRpm; q.eng.order = order; q.eng.rock = 0.0; q.eng.thump = 0.0;
                Layer L; L.configureFx(FxType::RpmVibe, q); L.configureEngine(q.eng);
                const double fireHz = rpm / 60.0 * cyl / 2.0;
                for (int i = 0; i < warmCycles; ++i) { L.driveEngine(1.0, fireHz, 1.0, lim); L.step(DT); }
                std::vector<double> raw; double pk = 0.0;
                for (int i = 0; i < 2000; ++i) { L.driveEngine(1.0, fireHz, 1.0, false); L.step(DT);
                    raw.push_back(L.overlayFor(6)); pk = std::max(pk, std::fabs(raw.back())); }
                return std::make_pair(pk, crossings(raw));
            };
            // V8, manual redline 7000, 4000 rpm: auto order ~1.03 -> ~69 Hz.
            const auto v8mid = buzzOnly(8, 7000, 0, 4000, false, 0);
            CHECK(v8mid.second >= 115 && v8mid.second <= 160, "auto order: 7000 rpm V8 at 4000 rpm sits mid-band (~69 Hz)");
            // V12, manual redline 16000, 14000 rpm: auto order ~0.45 -> ~105 Hz,
            // not clamped at the top.
            const auto f1 = buzzOnly(12, 16000, 0, 14000, false, 0);
            CHECK(f1.second >= 190 && f1.second <= 232, "auto order: 16000 rpm V12 at 14000 rpm near the band top (~105 Hz)");
            CHECK(f1.first > 3.0,                        "16000 rpm V12 carries real level at 14000");
            // Learned: no redline set, seeded 7000; 14000 rpm seen -> the
            // peak hold raises it and the carrier lands at the band top.
            const auto learned = buzzOnly(12, 0, 0, 14000, false, 400);
            CHECK(learned.second >= 225 && learned.second <= 260, "learned redline: peak hold rescales the band (~120 Hz)");
            // Limiter snap: bouncing at 6000 on a 7000-seeded engine pulls
            // the redline DOWN to ~6090, so 6000 is now ~98% of it (band top).
            const auto snapped = buzzOnly(8, 0, 0, 6000, true, 400);
            CHECK(snapped.second >= 225 && snapped.second <= 260, "limiter snap: redline follows the limiter hit, down as well as up");
            // Top end builds: 95% of redline is stronger than 60%.
            const auto p60 = buzzOnly(8, 7000, 1, 4200, false, 0), p95 = buzzOnly(8, 7000, 1, 6650, false, 0);
            CHECK(p95.first > p60.first * 1.05,          "level keeps building to the redline");
            // Thumps fade: a V8 at 6000 rpm (400 firings/s) has no resolvable thumps.
            EP t = p; t.eng.rock = 0.0; t.eng.buzz = 0.0; t.eng.maxRpm = 7000;
            Layer Lt; Lt.configureFx(FxType::RpmVibe, t); Lt.configureEngine(t.eng);
            double tpk = 0.0;
            for (int i = 0; i < 2000; ++i) { Lt.driveEngine(1.0, 400.0, 1.0, false); Lt.step(DT); tpk = std::max(tpk, std::fabs(Lt.overlayFor(6))); }
            CHECK(tpk < 0.5,                             "thumps fade out where firings cannot be resolved");
        }

        // Limiter controls on the engine: limiter x scales the return hit,
        // limiter hz sets the cut rate (count of silent holes per second),
        // limiter jit makes the cut timing irregular.
        {
            auto limRun = [&](double hit, double hz, double jit) {
                EP q = p; q.eng.limHit = hit; q.eng.limHz = hz; q.eng.limJit = jit; q.eng.rock = 0.0; q.eng.buzz = 0.0;
                Layer L; L.configureFx(FxType::RpmVibe, q); L.configureEngine(q.eng);
                double e = 0.0; int holes = 0, g = 0; std::vector<int> lens;
                for (int i = 0; i < 4000; ++i)                    // 2 s
                { L.driveEngine(1.0, 60.0, 1.0, true); L.step(DT);
                  const double a = std::fabs(L.overlayFor(6)); e += a;
                  if (a < 0.01) ++g; else { if (g >= 40) { ++holes; lens.push_back(g); } g = 0; } }
                return std::make_tuple(e, holes, lens);
            };
            const auto base = limRun(1.0, 12.0, 0.0), hard = limRun(1.5, 12.0, 0.0), slow = limRun(1.0, 6.0, 0.0);
            CHECK(std::get<0>(hard) > std::get<0>(base) * 1.3, "limiter x scales the return hit");
            // Where the limiter actually lives: redline rpm on a V8 (467
            // firings/s, per-firing thumps faded out). limiter x must still
            // scale what is felt there - the buzz and the return hit.
            auto redline = [&](double hit) {
                EP q = p; q.eng.limHit = hit; q.eng.maxRpm = 7000;
                Layer L; L.configureFx(FxType::RpmVibe, q); L.configureEngine(q.eng);
                double e = 0.0;
                for (int i = 0; i < 4000; ++i) { L.driveEngine(1.0, 466.7, 1.0, true); L.step(DT); e += std::fabs(L.overlayFor(6)); }
                return e;
            };
            CHECK(redline(1.5) > redline(1.0) * 1.3, "limiter x scales the bounce at redline rpm too");
            CHECK(std::get<1>(base) >= 18 && std::get<1>(base) <= 26, "limiter hz 12: ~24 cuts in 2 s");
            CHECK(std::get<1>(slow) >= 8 && std::get<1>(slow) <= 13,  "limiter hz 6: ~12 cuts in 2 s");
            const auto rough = limRun(1.0, 12.0, 1.0);
            const auto& L = std::get<2>(rough);
            int mn = 1 << 30, mx = 0; for (int x : L) { mn = std::min(mn, x); mx = std::max(mx, x); }
            CHECK(!L.empty() && mx > mn + 20,       "limiter jit: cut lengths vary");
        }

        // Engine size and layout. Same 800 rpm idle, thumps only (rock and
        // buzz off), so the peak is the firing impulse: a 6.5 L V8 hits
        // harder than a 2.0 L four, which hits harder than a 1.0 L triple;
        // and the rock (alone) is strongest on the triple, weaker on the V8,
        // weakest on the V12 - a many-cylinder engine is smooth.
        {
            auto peakAt = [&](double cyl, double litres, double layout, bool thumpsOnly) {
                EP q = p; q.eng.cylinders = cyl; q.eng.litres = litres; q.eng.layout = layout;
                if (thumpsOnly) { q.eng.rock = 0.0; q.eng.buzz = 0.0; } else { q.eng.thump = 0.0; q.eng.buzz = 0.0; }
                Layer L; L.configureFx(FxType::RpmVibe, q); L.configureEngine(q.eng);
                const double fireHz = (layout > 2.5) ? 800.0 / 60.0 * cyl : 800.0 / 60.0 * cyl / 2.0;
                double pk = 0.0;
                for (int i = 0; i < 2000; ++i) { L.driveEngine(1.0, fireHz, 1.0, false); L.step(DT);
                    pk = std::max(pk, std::fabs(L.overlayFor(6))); }
                return pk;
            };
            // Same eight cylinders, three displacements: the impulse follows
            // litres per cylinder. And a 1.0 L triple hits softer than a big V8.
            const double big = peakAt(8, 6.5, 1, true), mid = peakAt(8, 4.0, 1, true), small = peakAt(8, 2.7, 1, true);
            CHECK(big > mid && mid > small,    "thump weight follows displacement per cylinder (6.5 > 4.0 > 2.7 L V8)");
            CHECK(peakAt(3, 1.0, 0, true) < big, "a 1.0 L triple hits softer than a 6.5 L V8");
            // Balance alone: equal 0.5 L per cylinder, inline layout.
            const double r3 = peakAt(3, 1.5, 0, false), r8 = peakAt(8, 4.0, 0, false), r12 = peakAt(12, 6.0, 0, false);
            CHECK(r3 > r8 && r8 > r12,         "rock follows inherent balance (triple > eight > twelve at equal size)");
            const double flat4 = peakAt(4, 2.0, 2, false), in4 = peakAt(4, 2.0, 0, false);
            CHECK(flat4 < in4,                 "a boxer rocks less than the inline of the same count");
        }

        // Wankel: cylinders = rotors, one firing per rotor per shaft rev (a
        // 2-rotor at 800 rpm fires at 26.7 Hz, like a four), near-zero rock,
        // and the idle beat: the thump peaks swell and fade at ~2.5 Hz.
        {
            EP w = p; w.eng.cylinders = 2; w.eng.litres = 1.3; w.eng.layout = 3; w.eng.rock = 1.0; w.eng.buzz = 0.0; w.eng.thump = 1.0;
            EP wr = w; wr.eng.thump = 0.0;                      // rock alone
            Layer Lr; Lr.configureFx(FxType::RpmVibe, wr); Lr.configureEngine(wr.eng);
            double rockPk = 0.0;
            for (int i = 0; i < 2000; ++i) { Lr.driveEngine(1.0, 26.7, 1.0, false); Lr.step(DT); rockPk = std::max(rockPk, std::fabs(Lr.overlayFor(6))); }
            CHECK(rockPk < 2.0,                "Wankel: almost no reciprocating rock");
            EP wt = w; wt.eng.rock = 0.0;                       // thumps alone: the beat
            Layer Lt; Lt.configureFx(FxType::RpmVibe, wt); Lt.configureEngine(wt.eng);
            double hi = 0.0, lo = 1e9;                                // peak per 100 ms window
            for (int win = 0; win < 20; ++win)
            {
                double pk = 0.0;
                for (int i = 0; i < 200; ++i) { Lt.driveEngine(1.0, 26.7, 1.0, false); Lt.step(DT); pk = std::max(pk, std::fabs(Lt.overlayFor(6))); }
                if (win >= 4) { hi = std::max(hi, pk); lo = std::min(lo, pk); }
            }
            CHECK(hi > lo * 1.5,               "Wankel idle beat: firing pulses swell and fade");
        }

        // Load: coasting hits softer than full load.
        const Run coast = run(53.3, 0.0, false);
        CHECK(coast.peak > 1.0 && coast.peak < idle.peak * 0.75, "coasting hits softer than full load");

        // Limiter: whole bursts are cut (~12 Hz gate, half off) and the engine
        // catches again with a lurch on each return. So the output has real
        // holes of >= 25 ms AND hits at least as hard as off the limiter.
        // Random single misfires never produce holes that long.
        auto limShape = [&](bool lim){
            Layer Le; Le.configureFx(FxType::RpmVibe, p); Le.configureEngine(p.eng);
            double pk = 0.0; int g = 0, maxGap = 0;
            for (int i = 0; i < 4000; ++i)
            { Le.driveEngine(1.0, 60.0, 1.0, lim); Le.step(DT);
              const double a = std::fabs(Le.overlayFor(6)); pk = std::max(pk, a);
              if (a < 0.01) { ++g; maxGap = std::max(maxGap, g); } else g = 0; }
            return std::make_pair(pk, maxGap);
        };
        const auto on = limShape(true), off = limShape(false);
        CHECK(on.second >= 50,             "limiter leaves holes of >= 25 ms (the bounce)");
        CHECK(off.second < 50,             "no such holes without the limiter");
        CHECK(on.first >= off.first * 0.9, "the return lurch hits at least as hard as normal running");

        // No firing rate = silence even when driven.
        Layer Lz; Lz.configureFx(FxType::RpmVibe, p); Lz.configureEngine(p.eng);
        for (int i = 0; i < 200; ++i) { Lz.driveEngine(1.0, 0.0, 1.0, false); Lz.step(DT); }
        CHECK(Lz.overlayFor(6) == 0.0, "engine with no rpm is silent");
    }

    // ================= position sinks: derating by carrier =================
    // A route to a position axis is mm at 100% amplitude, derated to what
    // the axis can follow at the effect's carrier: min(cap, vB/w, aB/w^2).
    {
        using haptics::FxType;
        using haptics::SinkKind;
        Layer L;
        L.setSinkKind(0, SinkKind::Position);
        L.setPositionLimits(0, 80.0, 800.0, 3.0);      // 0.4 x (200 mm/s, 2000 mm/s^2), cap 3 mm
        approx(L.positionAllowedMm(0, 5.0),  0.8106, 0.01, "allowed at 5 Hz = aB / w^2 (0.81 mm)");
        approx(L.positionAllowedMm(0, 35.0), 0.0165, 0.002, "allowed at 35 Hz = aB / w^2 (0.017 mm)");
        approx(L.positionAllowedMm(0, 0.1),  3.0,    1e-9, "allowed at 0.1 Hz = the cap (velocity and accel both allow more)");
        approx(L.positionAllowedMm(0, 0.0),  3.0,    1e-9, "no carrier = the cap");
        CHECK(L.sinkKind(1) == SinkKind::Torque, "axes default to torque sinks");

        // Skid at 5 Hz, amp 100, gain 2 mm asked -> overlay peak = 0.81 mm x 100.
        EffectParams p; p.ampPct = 100.0; p.freqHz = 5.0; p.jitter = 0.0; p.routes[0] = { 0, 2.0 };
        L.configureFx(FxType::Road, p);
        double pk = 0.0;
        for (int i = 0; i < 2000; ++i) { L.driveFx(FxType::Road, 1.0, 0.0); L.step(DT); pk = std::max(pk, std::fabs(L.overlayFor(0))); }
        approx(pk / 100.0, 0.81, 0.03, "position sink: 5 Hz skid derated from 2 mm to 0.81 mm");

        // The same route on a torque sink is not derated at all.
        Layer T; T.configureFx(FxType::Road, p);
        double tk = 0.0;
        for (int i = 0; i < 2000; ++i) { T.driveFx(FxType::Road, 1.0, 0.0); T.step(DT); tk = std::max(tk, std::fabs(T.overlayFor(0))); }
        approx(tk, 200.0, 2.0, "torque sink: the same route is amp x gain, undetrated");

        // Engine on a vertical: the 13 Hz idle rock survives (crank-rate
        // crossings still there) while the 30 Hz thumps are derated hard.
        EP e; e.ampPct = 10.0; e.jitter = 0.0; e.eng.cylinders = 8.0; e.eng.litres = 4.0; e.freqHz = 30.0;
        e.routes[0] = { 0, 3.0 };
        Layer V; V.setSinkKind(0, SinkKind::Position); V.setPositionLimits(0, 80.0, 800.0, 3.0);
        V.configureFx(FxType::RpmVibe, e); V.configureEngine(e.eng);
        std::vector<double> raw, lp;
        for (int i = 0; i < 2000; ++i) { V.driveEngine(1.0, 53.3, 1.0, false); V.step(DT); raw.push_back(V.overlayFor(0)); }
        for (size_t i = 40; i < raw.size(); ++i) { double s = 0.0; for (size_t k = i - 40; k < i; ++k) s += raw[k]; lp.push_back(s / 40.0); }
        int xr = 0; for (size_t i = 1; i < lp.size(); ++i) if ((lp[i] >= 0.0) != (lp[i-1] >= 0.0)) ++xr;
        CHECK(xr >= 20 && xr <= 34, "engine on a position sink: the crank-rate rock survives the derating");
        double peakMm = 0.0; for (double v : raw) peakMm = std::max(peakMm, std::fabs(v) / 100.0);
        // Asked 0.3 mm (10% x 3 mm); the rock at 13 Hz is allowed ~0.115 mm and
        // is ~a third of the effect, so a few hundredths of a mm come through.
        CHECK(peakMm <= 0.3 + 1e-9 && peakMm > 0.02, "engine on a position sink: inside the asked 0.3 mm, not silent");
    }

    // ================= per-wheel slip model (Lateral / Longitudinal) =================
    {
        using haptics::FxType; using haptics::Part; using haptics::SlipParams; using haptics::SinkKind;
        // Lateral tile: scrub (fronts) and slide (rears). Only FL driven:
        // a route for the FL corner carries it, the FR corner and the rear
        // axle carry nothing, "front" and "all" carry it (strongest wheel).
        EffectParams p; p.ampPct = 100.0; p.jitter = 0.0;
        p.routes[0] = { 0, 1.0, Part::FL };
        p.routes[1] = { 1, 1.0, Part::FR };
        p.routes[2] = { 2, 1.0, Part::Rear };
        p.routes[3] = { 3, 1.0, Part::Front };
        p.routes[4] = { 4, 1.0, Part::All };
        SlipParams sp{ 1.0, 25.0, 1.0, 11.0, 7.0 };
        Layer L; L.configureFx(FxType::Skid, p); L.configureSlip(FxType::Skid, sp);
        double pk[5] = {};
        for (int i = 0; i < 1000; ++i)
        {
            L.driveSlip(FxType::Skid, WheelFL, 1.0, 0.0);
            L.step(DT);
            for (int a = 0; a < 5; ++a) pk[a] = std::max(pk[a], std::fabs(L.overlayFor(a)));
        }
        CHECK(pk[0] > 90.0, "slip: the FL corner route carries the FL wheel at full amp");
        CHECK(pk[1] < 1e-9,  "slip: the FR corner route carries nothing when only FL slips");
        CHECK(pk[2] < 1e-9,  "slip: the rear axle route carries nothing when only a front slips");
        CHECK(pk[3] > 90.0, "slip: the front axle route carries its strongest wheel");
        CHECK(pk[4] > 90.0, "slip: the all route carries the strongest wheel anywhere");
        CHECK(L.fxLevel(static_cast<int>(FxType::Skid)) > 0.99, "slip: tile level follows the strongest wheel");

        // Abrupt onset: full within ~10 ms of the drive starting (a tyre lets
        // go in milliseconds), release on the usual fade.
        Layer A; A.configureFx(FxType::Skid, p); A.configureSlip(FxType::Skid, sp);
        int cyclesToFull = -1;
        for (int i = 0; i < 200; ++i)
        {
            A.driveSlip(FxType::Skid, WheelRL, 0.0, 1.0); A.step(DT);
            if (cyclesToFull < 0 && A.slipWheelLevel(FxType::Skid, WheelRL) > 0.95) cyclesToFull = i;
        }
        CHECK(cyclesToFull >= 0 && cyclesToFull * DT <= 0.012, "slip: onset reaches full inside ~10 ms");
        int cyclesToGone = -1;
        for (int i = 0; i < 1000; ++i)
        {
            A.step(DT);   // not driven = releasing
            if (cyclesToGone < 0 && A.slipWheelLevel(FxType::Skid, WheelRL) < 0.05) cyclesToGone = i;
        }
        CHECK(cyclesToGone > 0 && cyclesToGone * DT > 0.08 && cyclesToGone * DT < 0.2, "slip: release fades over ~120 ms");

        // Severity staging: the carrier at full severity is (1 - 0.35) x set hz.
        // Count zero crossings of a rear slide at 11 Hz over 2 s: ~7.15 Hz -> ~28-29.
        Layer S; S.configureFx(FxType::Skid, p); S.configureSlip(FxType::Skid, sp);
        std::vector<double> o;
        for (int i = 0; i < 4000; ++i) { S.driveSlip(FxType::Skid, WheelRR, 0.0, 1.0); S.step(DT); if (i >= 200) o.push_back(S.overlayFor(2)); }
        int xr = 0; for (size_t i = 1; i < o.size(); ++i) if ((o[i] >= 0.0) != (o[i-1] >= 0.0)) ++xr;
        const double hzSeen = xr / 2.0 / ((o.size()) * DT);
        approx(hzSeen, 11.0 * 0.65, 0.6, "slip: carrier at full severity slows to 65% of set hz (staging)");
        Layer H; H.configureFx(FxType::Skid, p); H.configureSlip(FxType::Skid, sp);
        o.clear();
        for (int i = 0; i < 4000; ++i) { H.driveSlip(FxType::Skid, WheelRR, 0.0, 0.2); H.step(DT); if (i >= 200) o.push_back(H.overlayFor(2)); }
        xr = 0; for (size_t i = 1; i < o.size(); ++i) if ((o[i] >= 0.0) != (o[i-1] >= 0.0)) ++xr;
        approx(xr / 2.0 / (o.size() * DT), 11.0 * 0.93, 0.6, "slip: light severity keeps most of the set hz");

        // Mix knobs: slide x 0 silences the rears, scrub still plays.
        SlipParams noSlide = sp; noSlide.bMix = 0.0;
        Layer M; M.configureFx(FxType::Skid, p); M.configureSlip(FxType::Skid, noSlide);
        double rear = 0.0, front = 0.0;
        for (int i = 0; i < 500; ++i)
        {
            M.driveSlip(FxType::Skid, WheelRR, 0.0, 1.0); M.driveSlip(FxType::Skid, WheelFL, 1.0, 0.0); M.step(DT);
            rear = std::max(rear, std::fabs(M.overlayFor(2))); front = std::max(front, std::fabs(M.overlayFor(3)));
        }
        CHECK(rear < 1e-9 && front > 90.0, "slip: slide x 0 silences the rears only");

        // Longitudinal: lock (a) and spin (b) on the same wheel are two
        // carriers; the lock carrier scale follows road speed.
        EffectParams q; q.ampPct = 100.0; q.jitter = 0.0; q.routes[0] = { 0, 1.0, Part::All };
        SlipParams lon{ 1.0, 9.0, 0.0, 10.0, 0.8 };
        Layer K; K.configureFx(FxType::Lockup, q); K.configureSlip(FxType::Lockup, lon);
        K.setSlipCarrierScale(FxType::Lockup, 0.5, 1.0);
        o.clear();
        for (int i = 0; i < 4000; ++i) { K.driveSlip(FxType::Lockup, WheelFL, 1.0, 0.0); K.step(DT); if (i >= 200) o.push_back(K.overlayFor(0)); }
        xr = 0; for (size_t i = 1; i < o.size(); ++i) if ((o[i] >= 0.0) != (o[i-1] >= 0.0)) ++xr;
        approx(xr / 2.0 / (o.size() * DT), 9.0 * 0.5 * 0.65, 0.5, "slip: lock carrier = set hz x speed scale x staging");

        // Position sink: each component derated at ITS carrier. A 25 Hz
        // scrub asked at 2 mm on the 80 mm/s / 800 mm/s^2 axis is allowed
        // 800/w^2 at the staged carrier (~16 Hz) = ~0.08 mm.
        Layer P; P.setSinkKind(0, SinkKind::Position); P.setPositionLimits(0, 80.0, 800.0, 3.0);
        EffectParams r; r.ampPct = 100.0; r.jitter = 0.0; r.routes[0] = { 0, 2.0, Part::All };
        P.configureFx(FxType::Skid, r); P.configureSlip(FxType::Skid, sp);
        double pmm = 0.0;
        for (int i = 0; i < 2000; ++i) { P.driveSlip(FxType::Skid, WheelFL, 1.0, 0.0); P.step(DT); pmm = std::max(pmm, std::fabs(P.overlayFor(0)) / 100.0); }
        const double wS = 2.0 * 3.14159265358979 * 25.0 * 0.65;
        approx(pmm, 800.0 / (wS * wS), 0.01, "slip: position sink derates the scrub at its staged carrier");

        // amp 0 = the model is cleared and nothing plays.
        EffectParams z = p; z.ampPct = 0.0;
        Layer Z; Z.configureFx(FxType::Skid, z); Z.configureSlip(FxType::Skid, sp);
        double zk = 0.0;
        for (int i = 0; i < 200; ++i) { Z.driveSlip(FxType::Skid, WheelFL, 1.0, 1.0); Z.step(DT); zk = std::max(zk, std::fabs(Z.overlayFor(0))); }
        CHECK(zk < 1e-9 && Z.fxLevel(static_cast<int>(FxType::Skid)) == 0.0, "slip: amp 0 plays nothing and reports level 0");
    }

    // ================= model trigger: detent capture =================
    {
        DeviceParams p;
        p.springCurve  = { {0.0, 0.0}, {0.07, 50.0} };
        p.detentCurve  = { {-0.02, 60.0}, {0.0, 0.0}, {0.02, -60.0} };
        p.detents      = { 0.05 };
        p.stopMinRev   = -0.07;
        p.stopMaxRev   =  0.07;
        p.velLpfHz     = 0.0;
        p.slewPctPerSec = 1e9;
        p.maxForcePct  = 300.0;

        DeviceForceModel m; m.configure(p, 0.0005);
        DeviceStateMods inert;
        HapticTriggers t;

        // Seed step INSIDE the gate: must not fire (engaged in-gate).
        m.reset(0.05);
        t = {}; m.step(0.05, inert, &t);
        CHECK(!t.detentEnter, "seed step never fires, even inside the gate");

        // Leave, then re-enter: exactly one edge.
        t = {}; m.step(0.02, inert, &t);
        CHECK(!t.detentEnter, "outside the capture radius: no trigger");
        t = {}; m.step(0.046, inert, &t);
        CHECK(t.detentEnter, "entering the capture radius fires");
        CHECK(t.detentVel > 0.99, "fast entry saturates the velocity scale");
        t = {}; m.step(0.05, inert, &t);
        CHECK(!t.detentEnter, "dwelling in the gate never re-fires");

        // Null-sink cycles still advance the edge state: step out with no
        // sink, back in with one - the entry must still read as an edge.
        m.step(0.0, inert);                       // leave, nobody listening
        t = {}; m.step(0.05, inert, &t);
        CHECK(t.detentEnter, "edge state advances through null-sink cycles");

        // Slow entry scales down.
        m.step(0.02, inert);
        t = {};
        for (double x = 0.02; !t.detentEnter && x < 0.06; x += 0.0002)
            m.step(x, inert, &t);
        CHECK(t.detentEnter && t.detentVel < 0.5, "gentle entry gets a gentle scale");

        // Pure-haptic gate: empty detent CURVE still triggers.
        DeviceParams ph = p; ph.detentCurve.clear();
        DeviceForceModel mh; mh.configure(ph, 0.0005);
        mh.reset(0.0); mh.step(0.0, inert);
        t = {}; mh.step(0.05, inert, &t);
        CHECK(t.detentEnter, "gates with no holding force still click");

        // No detents at all: never triggers.
        DeviceParams pn = p; pn.detents.clear();
        DeviceForceModel mn; mn.configure(pn, 0.0005);
        mn.reset(0.0); mn.step(0.0, inert);
        t = {}; mn.step(0.05, inert, &t);
        CHECK(!t.detentEnter, "no detents: no trigger, ever");
    }

    std::printf("\nTestHaptics: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
