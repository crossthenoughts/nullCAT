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
// The existing engine checks measure the three firing components in
// isolation (silences, cut holes), so the bundle starts with the
// load-independent inertia OFF; its own block below turns it on.
struct EP : EffectParams { EngineParams eng; EP() { eng.inertia = 0.0; } };
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

        // Driven frequency is honored (Limiter slot; the RpmVibe slot is
        // the pulse-train engine and ignores a driven carrier by design).
        EffectParams pr; pr.ampPct = 10.0; pr.freqHz = 0.0; pr.routes[0] = { 2, 1.0 };
        Layer L6; L6.configureFx(FxType::Limiter, pr);
        double vibe = 0.0;
        for (int i = 0; i < 400; ++i)
        { L6.driveFx(FxType::Limiter, 1.0, 120.0); L6.step(DT); vibe = std::max(vibe, std::fabs(L6.overlayFor(2))); }
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

    // ================= engine: lift-off, inertia, boost, pops =================
    {
        using haptics::EngineModel; using haptics::EngineOut;
        const double pi = 3.14159265358979;
        auto rms = [](const std::vector<double>& v, size_t a, size_t b) {
            double s = 0.0; for (size_t i = a; i < b && i < v.size(); ++i) s += v[i] * v[i];
            return std::sqrt(s / std::max<size_t>(1, std::min(b, v.size()) - a)); };
        // A V8 at 7000 rpm full throttle, then the throttle snaps shut and
        // the revs fall linearly to 3500 over 2 s.
        auto run = [&](EngineParams e, bool liftThenDescend, double boost, std::vector<double>& thumps, std::vector<double>& buzz) {
            EngineModel m; uint64_t rng = 7;
            EffectParams p; p.ampPct = 100.0; p.freqHz = 30.0; p.jitter = 0.0;
            e.cylinders = 8.0; e.litres = 5.0; e.maxRpm = 7000.0;
            double t = 0.0;
            for (int i = 0; i < 6000; ++i, t += DT)
            {
                double rpm = 7000.0, load = 1.0;
                if (liftThenDescend && t >= 1.0) { load = 0.0; rpm = 7000.0 - 3500.0 * std::min(1.0, (t - 1.0) / 2.0); }
                m.drive(rpm / 60.0 * 4.0, load, false, boost);
                const EngineOut eo = m.step(DT, p, e, 1.0, rng);
                thumps.push_back(eo.thumps); buzz.push_back(eo.buzz);
            }
            return m.liftOffCount();
        };
        std::vector<double> th, bz;
        // Inertia: with buzz off, the buzz-band output after the lift is the
        // inertia alone; it is there at 7000 and ~1/4 of that at 3500.
        EngineParams e; e.buzz = 0.0; e.inertia = 1.0; e.liftoff = 0.0;
        run(e, true, -1.0, th, bz);
        const double atTop = rms(bz, 2200, 2600), atHalf = rms(bz, 5700, 6000);
        CHECK(atTop > 10.0, "engine: inertia carries the high-rpm shake after the lift (buzz off)");
        approx(atHalf / std::max(1e-9, atTop), 0.25, 0.08, "engine: inertia fades with rpm^2 as the revs fall");
        // Inertia off: the buzz band is silent on the lift with buzz off.
        th.clear(); bz.clear(); e.inertia = 0.0;
        run(e, true, -1.0, th, bz);
        CHECK(rms(bz, 2200, 2600) < 1e-9, "engine: no inertia = nothing in the band after a lift with buzz off");

        // Lift-off, naturally aspirated: exactly one event on the edge, a
        // pop in the thump pool right after the lift, none without a lift.
        th.clear(); bz.clear();
        EngineParams na; na.inertia = 0.0; na.liftoff = 1.0; na.turbo = 0.0; na.thump = 0.0;
        const uint64_t lifts = run(na, true, -1.0, th, bz);
        CHECK(lifts == 1, "engine: one lift-off event per throttle drop edge");
        CHECK(rms(th, 2000, 2200) > 2.0 && rms(th, 3000, 3400) < 1e-9, "engine: NA lift-off is one pop, then silence (thumps off)");
        th.clear(); bz.clear();
        CHECK(run(na, false, -1.0, th, bz) == 0 && rms(th, 2000, 2400) < 1e-9, "engine: no lift, no pop");
        // Turbo: the whoosh plus a flutter train lasting ~0.4 s, heavier than the pop.
        th.clear(); bz.clear();
        EngineParams tb = na; tb.turbo = 1.0;
        run(tb, true, 1.0, th, bz);
        CHECK(rms(th, 2000, 2200) > 6.0 && rms(th, 2500, 2800) > 0.5 && rms(th, 4000, 4400) < 1e-9,
              "engine: turbo lift-off is a whoosh and ~0.4 s of flutter, then silence");
        // Lift-off strength follows boost when the sim sends it: 0.2 bar is a fifth of 1 bar.
        std::vector<double> th2, bz2;
        run(tb, true, 0.2, th2, bz2);
        approx(rms(th2, 2000, 2200) / std::max(1e-9, rms(th, 2000, 2200)), 0.2, 0.05, "engine: lift-off scales with reported boost");

        // Boost makes each firing heavier on throttle: at 1500 rpm (where the
        // thumps are still resolved) 1 bar reads 1.6x the no-boost hit.
        auto thumpRms = [&](double boost) {
            EngineModel m; uint64_t rng = 3;
            EffectParams p; p.ampPct = 100.0; p.freqHz = 30.0; p.jitter = 0.0;
            EngineParams bt; bt.inertia = 0.0; bt.liftoff = 0.0; bt.cylinders = 8.0; bt.litres = 5.0; bt.maxRpm = 7000.0;
            std::vector<double> v;
            for (int i = 0; i < 4000; ++i) { m.drive(1500.0 / 60.0 * 4.0, 1.0, false, boost); v.push_back(m.step(DT, p, bt, 1.0, rng).thumps); }
            return rms(v, 1000, 4000); };
        approx(thumpRms(1.0) / std::max(1e-9, thumpRms(0.0)), 1.6, 0.1, "engine: 1 bar of boost = 1.6x heavier firings");

        // Pops: with pops on, the overrun carries sparse pulses; on throttle none extra.
        EngineParams pp; pp.inertia = 0.0; pp.liftoff = 0.0; pp.thump = 0.0; pp.pops = 1.0;
        th.clear(); bz.clear(); run(pp, true, -1.0, th, bz);
        CHECK(rms(th, 400, 1800) < 1e-9, "engine: no pops on throttle");
        CHECK(rms(th, 2400, 5600) > 1.0, "engine: pops crackle on the overrun");
        int bursts = 0; for (size_t i = 2401; i < 5600; ++i) if (std::fabs(th[i]) > 1e-9 && std::fabs(th[i-1]) <= 1e-9) ++bursts;
        CHECK(bursts >= 4 && bursts <= 40, "engine: pops are sparse separate events, not a tone");
    }

    // ================= engine: cranking, catch, stall, pit limiter, overrun =================
    {
        using haptics::EngineModel; using haptics::EngineOut;
        auto rms = [](const std::vector<double>& v, size_t a, size_t b) {
            double s = 0.0; for (size_t i = a; i < b && i < v.size(); ++i) s += v[i] * v[i];
            return std::sqrt(s / std::max<size_t>(1, std::min(b, v.size()) - a)); };
        EffectParams p; p.ampPct = 100.0; p.freqHz = 30.0; p.jitter = 0.0;
        EngineParams e; e.inertia = 0.0; e.liftoff = 0.0; e.cylinders = 4.0; e.litres = 2.0; e.maxRpm = 7000.0;
        const double perRev = 2.0;   // 4-stroke four: firings per rev

        // Start: 1.5 s on the starter at 250 rpm (slow lumps, no combustion),
        // then the catch at 1200 rpm, then idle. Running state and one catch.
        {
            EngineModel m; uint64_t rng = 5; std::vector<double> th, rk;
            for (int i = 0; i < 6000; ++i)
            {
                const double t = i * DT;
                const double rpm = t < 1.5 ? 250.0 : (t < 1.8 ? 1200.0 : 850.0);
                m.drive(rpm / 60.0 * perRev, 0.0, false);
                const EngineOut eo = m.step(DT, p, e, 1.0, rng);
                th.push_back(eo.thumps); rk.push_back(eo.rock);
                if (i == 2000) CHECK(!m.running(), "engine: cranking at 250 rpm is not running yet");
            }
            CHECK(rms(th, 400, 2800) > 1.0, "engine: cranking produces compression lumps");
            int lumps = 0; for (size_t i = 401; i < 2800; ++i) if (std::fabs(th[i]) > 1e-9 && std::fabs(th[i-1]) <= 1e-9) ++lumps;
            // 250 rpm x 2 firings/rev = 8.3 lumps/s over 1.2 s = ~10, each the
            // 33 ms thump apart, so they read as separate events.
            CHECK(lumps >= 7 && lumps <= 13, "engine: cranking lumps come at the compression rate (~8 per second)");
            CHECK(m.running() && m.catchCount() == 1, "engine: the engine catches once as the revs rise through 500");
            CHECK(rms(th, 3000, 3200) > rms(th, 1000, 1200) * 1.5, "engine: the catch is a heavier lurch than a cranking lump");
        }

        // Stall: running at idle, the revs collapse over 1 s to zero. The
        // thumps get heavier as it dies, one last kick lands at the stop,
        // and the model reads stopped; a restart later catches again.
        {
            EngineModel m; uint64_t rng = 9; std::vector<double> th;
            auto at = [&](double rpm, int cycles) { for (int c = 0; c < cycles; ++c) { m.drive(rpm / 60.0 * perRev, 0.0, false); th.push_back(m.step(DT, p, e, 1.0, rng).thumps); } };
            at(250.0, 400); at(1200.0, 400); at(850.0, 1600);                   // crank, catch, idle
            size_t stopAt = 0;
            for (int i = 0; i < 2000; ++i)                                       // dying over 1 s
            {
                at(850.0 * (1.0 - i / 2000.0), 1);
                if (!stopAt && m.stallCount() == 1) stopAt = th.size() - 1;
            }
            // Down to 250 rpm it is dying (heavier than idle); the stop (and
            // the kick) land when the revs fall through 250.
            auto peak = [&](size_t a, size_t b) { double m2 = 0.0; for (size_t i = a; i < b && i < th.size(); ++i) m2 = std::max(m2, std::fabs(th[i])); return m2; };
            CHECK(peak(3400, std::min<size_t>(3700, stopAt)) > peak(1400, 2400) * 1.3, "engine: a dying engine's shudders hit harder than its idle thumps");
            CHECK(m.stallCount() == 1 && !m.running() && stopAt > 3600, "engine: it stops once as the revs fall through 250");
            CHECK(rms(th, stopAt, stopAt + 150) > rms(th, stopAt + 400, stopAt + 600) + 1.0, "engine: the stall kick lands at the stop, then silence");
            at(0.0, 400); at(250.0, 600); at(1000.0, 400);
            CHECK(m.catchCount() == 2 && m.running(), "engine: a restart catches again");
            // An engine already running when the stream begins just runs:
            // no catch lurch out of nowhere.
            EngineModel j; uint64_t rj = 2; std::vector<double> tj;
            for (int i = 0; i < 400; ++i) { j.drive(3000.0 / 60.0 * perRev, 0.3, false); tj.push_back(j.step(DT, p, e, 1.0, rj).thumps); }
            CHECK(j.running() && j.catchCount() == 0, "engine: joining a running engine is not a catch event");
        }

        // Pit limiter: cuts like the rev limiter but never teaches the redline.
        {
            EngineModel a, b; uint64_t rng = 11; std::vector<double> va, vb;
            EngineParams le = e; le.maxRpm = 0.0;   // learning
            for (int i = 0; i < 2000; ++i)
            {
                a.drive(1500.0 / 60.0 * perRev, 0.5, false, -1.0, true,  false);  // pit limiter at 1500
                b.drive(1500.0 / 60.0 * perRev, 0.5, false, -1.0, false, false);  // plain 1500
                va.push_back(a.step(DT, p, le, 1.0, rng).thumps);
                vb.push_back(b.step(DT, p, le, 1.0, rng).thumps);
            }
            int holes = 0; bool inHole = false;
            for (size_t i = 400; i < va.size(); ++i) { const bool z = std::fabs(va[i]) <= 1e-9; if (z && !inHole) ++holes; inHole = z; }
            CHECK(holes >= 8, "engine: the pit limiter cuts firings in bursts");
            approx(a.learnedMaxRpm(), b.learnedMaxRpm(), 1e-9, "engine: the pit limiter teaches nothing about the redline");
            CHECK(a.learnedMaxRpm() >= 7000.0 - 1e-9, "engine: the learned redline stays at its seed under a pit limiter");
        }

        // Overrun: throttle shut at 4000 rpm, in gear and rolling is heavier
        // than the same in neutral (the wheels pump the engine).
        {
            auto overrun = [&](bool inGear) {
                EngineModel m; uint64_t rng = 13; std::vector<double> th;
                for (int i = 0; i < 3000; ++i) { m.drive(4000.0 / 60.0 * perRev, 0.0, false, -1.0, false, inGear); th.push_back(m.step(DT, p, e, 1.0, rng).thumps); }
                return rms(th, 1000, 3000); };
            const double gear = overrun(true), neutral = overrun(false);
            CHECK(gear > neutral * 1.3 && neutral > 0.0, "engine: overrun in gear is distinctly heavier than a neutral coast-down");
        }
    }

    // ================= driveline model =================
    {
        using haptics::FxType; using haptics::DrivelineParams; using haptics::SinkKind;
        EffectParams p; p.ampPct = 100.0; p.jitter = 0.0; p.routes[0] = { 0, 1.0 };
        DrivelineParams d{ 1.0, 10.0, 1.0, 7.0 };
        // Judder alone at 10 Hz: count the cycles; lug alone at 7 Hz.
        auto hzOf = [&](double judder, double lug) {
            Layer L; L.configureFx(FxType::Driveline, p); L.configureDriveline(d);
            std::vector<double> o;
            for (int i = 0; i < 4200; ++i) { L.driveDriveline(judder, lug); L.step(DT); if (i >= 200) o.push_back(L.overlayFor(0)); }
            int xr = 0; for (size_t i = 1; i < o.size(); ++i) if ((o[i] >= 0.0) != (o[i-1] >= 0.0)) ++xr;
            double pk = 0.0; for (double v : o) pk = std::max(pk, std::fabs(v));
            return std::make_pair(xr / 2.0 / (o.size() * DT), pk); };
        auto j = hzOf(1.0, 0.0), l = hzOf(0.0, 1.0);
        approx(j.first, 10.0, 0.3, "driveline: judder plays at clutch hz");
        approx(l.first, 7.0, 0.3,  "driveline: lug plays at lug hz");
        CHECK(j.second > 95.0 && l.second > 95.0, "driveline: each component reaches amp x mix at full severity");
        CHECK(hzOf(0.0, 0.0).second < 1e-9, "driveline: nothing when neither is driven");
        DrivelineParams noLug = d; noLug.lug = 0.0;
        { Layer L; L.configureFx(FxType::Driveline, p); L.configureDriveline(noLug); double pk = 0.0;
          for (int i = 0; i < 1000; ++i) { L.driveDriveline(0.0, 1.0); L.step(DT); pk = std::max(pk, std::fabs(L.overlayFor(0))); }
          CHECK(pk < 1e-9, "driveline: lug x 0 silences the lug"); }
        // Position sink: the 10 Hz judder is derated at 10 Hz.
        { Layer P; P.setSinkKind(0, SinkKind::Position); P.setPositionLimits(0, 80.0, 800.0, 3.0);
          EffectParams r = p; r.routes[0] = { 0, 2.0 };
          P.configureFx(FxType::Driveline, r); P.configureDriveline(d);
          double pk = 0.0; for (int i = 0; i < 2000; ++i) { P.driveDriveline(1.0, 0.0); P.step(DT); pk = std::max(pk, std::fabs(P.overlayFor(0)) / 100.0); }
          const double w = 2.0 * 3.14159265358979 * 10.0;
          approx(pk, std::min(2.0, 800.0 / (w * w)), 0.02, "driveline: position sink derates the judder at its carrier"); }
    }

    // ================= engine layouts: two-stroke, electric =================
    {
        using haptics::EngineModel; using haptics::EngineOut;
        auto rms = [](const std::vector<double>& v, size_t a, size_t b) {
            double s = 0.0; for (size_t i = a; i < b && i < v.size(); ++i) s += v[i] * v[i];
            return std::sqrt(s / std::max<size_t>(1, std::min(b, v.size()) - a)); };
        EffectParams p; p.ampPct = 100.0; p.freqHz = 30.0; p.jitter = 0.3;
        // Two-stroke single (a kart) at 3000 rpm fires once per rev: twice as
        // many thumps as a four-stroke single, and no half-order content in
        // the rock (count rock zero crossings: crank rate only).
        auto thumpsPerSec = [&](double layout, double perRev) {
            EngineModel m; uint64_t rng = 4; EffectParams q = p; q.jitter = 0.0; q.freqHz = 80.0;   // 12.5 ms thumps: 50/s still leaves gaps
            EngineParams e; e.cylinders = 1.0; e.litres = 0.125; e.layout = layout; e.maxRpm = 14000.0; e.inertia = 0.0; e.liftoff = 0.0;
            std::vector<double> th;
            for (int i = 0; i < 4000; ++i) { m.drive(3000.0 / 60.0 * perRev, 0.8, false); th.push_back(m.step(DT, q, e, 1.0, rng).thumps); }
            int bursts = 0; for (size_t i = 2001; i < th.size(); ++i) if (std::fabs(th[i]) > 1e-9 && std::fabs(th[i-1]) <= 1e-9) ++bursts;
            return bursts / 1.0; };
        // (the law supplies fireHz = rpm/60 x perRev: four-stroke single 0.5, two-stroke 1)
        const double four = thumpsPerSec(0.0, 0.5), two = thumpsPerSec(4.0, 1.0);
        approx(four, 25.0, 3.0, "engine: a four-stroke single at 3000 rpm thumps 25 times a second");
        approx(two,  50.0, 4.0, "engine: a two-stroke single at 3000 rpm thumps 50 times a second");
        {
            auto rockHalfOrder = [&](double layout, double perRev) {
                EngineModel m; uint64_t rng = 6;
                EngineParams e; e.cylinders = 1.0; e.litres = 0.125; e.layout = layout; e.maxRpm = 14000.0; e.thump = 0.0; e.buzz = 0.0; e.inertia = 0.0;
                EffectParams q = p; q.jitter = 1.0;   // full lope: the half-order term is at its largest
                std::vector<double> rk;
                for (int i = 0; i < 4000; ++i) { m.drive(700.0 / 60.0 * perRev, 0.5, false); rk.push_back(m.step(DT, q, e, 1.0, rng).rock); }
                // Crank rate 11.7 Hz: a pure crank-rate rock crosses zero ~23/s;
                // half-order content shifts crossings off the even spacing.
                // Measure the spread of crossing intervals instead.
                std::vector<double> gaps; double last = -1;
                for (size_t i = 2001; i < rk.size(); ++i) if ((rk[i] >= 0.0) != (rk[i-1] >= 0.0)) { if (last >= 0) gaps.push_back((i - last) * DT); last = i; }
                double mean = 0; for (double g : gaps) mean += g; mean /= std::max<size_t>(1, gaps.size());
                double var = 0; for (double g : gaps) var += (g - mean) * (g - mean); var /= std::max<size_t>(1, gaps.size());
                return std::sqrt(var) / mean; };
            const double fourSpread = rockHalfOrder(0.0, 0.5), twoSpread = rockHalfOrder(4.0, 1.0);
            CHECK(fourSpread > 0.08, "engine: a four-stroke at full lope has half-order unevenness in its rock");
            CHECK(twoSpread < fourSpread * 0.5, "engine: a two-stroke has no half-order lope (every rev alike)");
        }
        // Electric: no thumps, no rock, no inertia; a whine from the first
        // turn that rises with load, no cuts under a limiter flag, no
        // lift-off burst, and no cranking machinery.
        {
            EngineModel m; uint64_t rng = 8;
            EngineParams e; e.layout = 5.0; e.maxRpm = 20000.0; e.inertia = 1.0; e.liftoff = 1.0; e.turbo = 1.0; e.pops = 1.0;
            std::vector<double> th, rk, bz;
            auto run = [&](double rpm, double load, bool lim, int n) { for (int i = 0; i < n; ++i) { m.drive(rpm / 60.0, load, lim); const EngineOut eo = m.step(DT, p, e, 1.0, rng); th.push_back(eo.thumps); rk.push_back(eo.rock); bz.push_back(eo.buzz); } };
            run(250.0, 0.5, false, 1000);        // what would be cranking on a piston engine
            run(2000.0, 0.2, false, 2000);       // light load
            run(2000.0, 1.0, false, 2000);       // full load
            run(20000.0, 1.0, true, 2000);       // "limiter" flag at max rpm
            run(20000.0, 0.0, false, 2000);      // lift from max
            CHECK(rms(th, 0, th.size()) < 1e-9 && rms(rk, 0, rk.size()) < 1e-9, "engine: electric has no thumps and no rock, ever");
            CHECK(rms(bz, 200, 1000) > 0.5, "engine: electric whines from the first turn (250 rpm)");
            CHECK(rms(bz, 3500, 5000) > rms(bz, 1500, 3000) * 2.0, "engine: the whine rises with load");
            // A cut = a run of silence longer than a carrier cycle (a sine's own
            // zero crossings are single samples).
            int holes = 0, run0 = 0;
            for (size_t i = 5200; i < 7000; ++i) { if (std::fabs(bz[i]) <= 1e-6) { if (++run0 == 10) ++holes; } else run0 = 0; }
            CHECK(holes == 0, "engine: electric has a soft limiter, no cut bursts");
            CHECK(rms(th, 7000, 7400) < 1e-9, "engine: no blow-off or pops on an electric lift");
            CHECK(rms(bz, 7400, 9000) > 0.2 && rms(bz, 7400, 9000) < rms(bz, 5200, 7000), "engine: regen whine on the lift, quieter than under load");
            CHECK(m.catchCount() == 0 && m.stallCount() == 0, "engine: electric never catches or stalls");
        }
    }

    // ================= shaker routes: scale, harmonic, tone, delay =================
    {
        using haptics::FxType; using haptics::Route; using haptics::MAX_SHAKER_OUT;
        auto hzOf = [](const std::vector<double>& v, double dt) {
            int xr = 0; for (size_t i = 1; i < v.size(); ++i) if ((v[i] >= 0.0) != (v[i-1] >= 0.0)) ++xr;
            return xr / 2.0 / (v.size() * dt); };
        auto pk = [](const std::vector<double>& v) { double m = 0; for (double x : v) m = std::max(m, std::fabs(x)); return m; };
        // A 9 Hz continuous effect at amp 100: the belt route gets 100% and
        // shaker 0 (gain 1) gets a full-scale 1.0 sample at 9 Hz; shaker 1
        // at harm 2 plays 18 Hz, phase-locked (zero crossings of the
        // fundamental are zero crossings of the harmonic).
        EffectParams p; p.ampPct = 100.0; p.freqHz = 9.0; p.jitter = 0.0;
        p.routes[0] = { 0, 1.0 };
        Route s0; s0.shaker = 0; s0.gain = 1.0; s0.harm = 1; p.routes[1] = s0;
        Route s1; s1.shaker = 1; s1.gain = 0.5; s1.harm = 2; p.routes[2] = s1;
        Layer L; L.configureFx(FxType::Limiter, p);
        std::vector<double> belt, sh0, sh1;
        for (int i = 0; i < 4000; ++i)
        {
            L.driveFx(FxType::Limiter, 1.0, 0.0); L.step(DT);
            if (i >= 400) { belt.push_back(L.overlayFor(0)); sh0.push_back(L.shakerSample(0)); sh1.push_back(L.shakerSample(1)); }
        }
        approx(pk(belt), 100.0, 1.0, "shaker: the belt route is unchanged (amp x gain, % of rated)");
        approx(pk(sh0), 1.0, 0.01, "shaker: 100% amp x gain 1 is a full-scale 1.0 sample");
        approx(hzOf(sh0, DT), 9.0, 0.3, "shaker: harm 1 plays the effect's carrier");
        approx(hzOf(sh1, DT), 18.0, 0.4, "shaker: harm 2 plays twice the carrier");
        approx(pk(sh1), 0.5, 0.01, "shaker: the route gain scales the sample");
        int locked = 0, cross = 0;
        for (size_t i = 1; i < sh0.size(); ++i)
            if ((sh0[i] >= 0.0) != (sh0[i-1] >= 0.0)) { ++cross; if (std::fabs(sh1[i]) < 0.08) ++locked; }
        CHECK(cross > 30 && locked >= cross - 2, "shaker: the harmonic is phase-locked to the fundamental");
        CHECK(L.shakerSample(2) == 0.0 && L.shakerSample(MAX_SHAKER_OUT) == 0.0, "shaker: unrouted channels are silent, out-of-range reads 0");
        CHECK(Layer::hasRoute(p), "shaker: a shaker-only route counts as routed");
        EffectParams onlyShaker = p; onlyShaker.routes[0] = {};
        CHECK(Layer::hasRoute(onlyShaker), "shaker: an effect routed to shakers alone is routed");

        // Slip: the rear slide on a shaker at harm 3 plays three times the
        // staged carrier (11 x 0.65 = 7.15 Hz -> 21.5 Hz).
        {
            EffectParams q; q.ampPct = 100.0; q.jitter = 0.0;
            Route r; r.shaker = 0; r.gain = 1.0; r.harm = 3; r.part = haptics::Part::Rear; q.routes[0] = r;
            Layer S; S.configureFx(FxType::Skid, q); S.configureSlip(FxType::Skid, { 1.0, 25.0, 1.0, 11.0, 7.0 });
            std::vector<double> o;
            for (int i = 0; i < 4200; ++i) { S.driveSlip(FxType::Skid, WheelRR, 0.0, 1.0); S.step(DT); if (i >= 200) o.push_back(S.shakerSample(0)); }
            approx(hzOf(o, DT), 11.0 * 0.65 * 3.0, 1.0, "shaker: a slip component plays at its staged carrier x harm");
            approx(pk(o), 1.0, 0.02, "shaker: full severity x mix 1 = full scale");
        }
        // Engine: rock on a shaker at harm 2 doubles the crank-rate rock.
        {
            EP e; e.ampPct = 100.0; e.jitter = 0.0; e.eng.cylinders = 8.0; e.eng.litres = 5.0; e.eng.thump = 0.0; e.eng.buzz = 0.0; e.eng.rock = 1.0;
            Route r1; r1.shaker = 0; r1.gain = 1.0; r1.harm = 1; e.routes[0] = r1;
            Route r2; r2.shaker = 1; r2.gain = 1.0; r2.harm = 2; e.routes[1] = r2;
            Layer G; G.configureFx(FxType::RpmVibe, e); G.configureEngine(e.eng);
            std::vector<double> a, b;
            for (int i = 0; i < 4000; ++i) { G.driveEngine(1.0, 800.0 / 60.0 * 4.0, 0.5, false); G.step(DT); if (i >= 1000) { a.push_back(G.shakerSample(0)); b.push_back(G.shakerSample(1)); } }
            approx(hzOf(a, DT), 13.33, 1.0, "shaker: engine rock at harm 1 is the crank rate");
            approx(hzOf(b, DT), 26.67, 1.5, "shaker: engine rock at harm 2 is twice the crank rate");
        }
        // Tone test: 40 Hz at 0.5 on one channel for the asked time, then gone.
        {
            Layer T; std::vector<double> o;
            T.startShakerTone(3, 0.5);
            for (int i = 0; i < 2000; ++i) { T.step(DT); if (i < 1000) o.push_back(T.shakerSample(3)); }
            approx(hzOf(o, DT), 40.0, 1.0, "shaker: the tone test plays 40 Hz on the channel");
            approx(pk(o), 0.5, 0.01, "shaker: ...at half scale");
            CHECK(!T.shakerToneActive() && T.shakerSample(3) == 0.0 && T.shakerSample(0) == 0.0, "shaker: the tone stops after its time and touched no other channel");
        }
        // Alignment delay: 10 cycles of delay shift the axis overlay by 10
        // cycles and leave the shaker sample where it was.
        {
            EffectParams d; d.ampPct = 100.0; d.freqHz = 20.0; d.jitter = 0.0;
            d.routes[0] = { 0, 1.0 }; Route sr; sr.shaker = 0; sr.gain = 1.0; d.routes[1] = sr;
            Layer A; A.configureFx(FxType::Limiter, d); A.setAxisDelayCycles(10);
            std::vector<double> ax, sh;
            for (int i = 0; i < 2000; ++i) { A.driveFx(FxType::Limiter, 1.0, 0.0); A.step(DT); ax.push_back(A.overlayFor(0) / 100.0); sh.push_back(A.shakerSample(0)); }
            double best = 1e9; int bestLag = -1;
            for (int lag = 0; lag < 40; ++lag) { double err = 0; for (size_t i = 500; i < 1900; ++i) err += std::fabs(ax[i] - sh[i - lag]); if (err < best) { best = err; bestLag = lag; } }
            CHECK(bestLag == 10, "shaker: the axis alignment delay holds the axis overlay back by the set cycles");
            A.setAxisDelayCycles(0);
            A.driveFx(FxType::Limiter, 1.0, 0.0); A.step(DT);
            approx(A.overlayFor(0) / 100.0, A.shakerSample(0), 1e-9, "shaker: delay 0 = the axis and the shaker see the same sample");
        }
    }

    // ================= engine buzz band vs loop rate =================
    {
        // The automatic buzz order aims the redline at 120 Hz on a 2 kHz
        // loop but at loopHz/8 on slower loops (62.5 Hz at 500 Hz), so a
        // PC build never synthesises a carrier with 4 samples per cycle.
        auto buzzAtRedline = [](double dt) {
            haptics::EngineModel m; uint64_t rng = 1;
            EffectParams p; p.ampPct = 50.0; p.freqHz = 30.0; p.jitter = 0.0;
            EngineParams e; e.cylinders = 8.0; e.litres = 5.0; e.maxRpm = 7000.0;
            double hz = 0.0;
            for (int i = 0; i < 400; ++i)
            {
                m.drive(7000.0 / 60.0 * 4.0, 1.0, false);     // 7000 rpm, V8: 466.7 firings/s
                hz = m.step(dt, p, e, 1.0, rng).buzzHz;
            }
            return hz;
        };
        approx(buzzAtRedline(0.0005), 120.0, 1.0, "engine: auto buzz band tops at 120 Hz on a 2 kHz loop");
        approx(buzzAtRedline(0.002),  62.5,  1.0, "engine: auto buzz band tops at loopHz/8 on a 500 Hz loop");
    }

    // ================= per-corner road replay =================
    {
        using haptics::FxType; using haptics::Part; using haptics::RoadParams; using haptics::SinkKind;
        // A 10 Hz, 500 mm/s peak suspension velocity on FL is a travel of
        // V/sqrt(w^2 + wc^2) = 500/(2 pi sqrt(10^2 + 2^2)) = 7.80 mm peak
        // through the 2 Hz leaky integrator; with full mm 16 that is 0.49 of
        // full, inside the knee (linear), and the 20 Hz smoothing passes
        // 1/sqrt(1 + (10/20)^2) = 0.894 of it.
        EffectParams p; p.ampPct = 100.0; p.jitter = 0.0;
        p.routes[0] = { 0, 1.0, Part::FL };
        p.routes[1] = { 1, 1.0, Part::RR };
        p.routes[2] = { 2, 1.0, Part::All };
        RoadParams rp{ 16.0, 2.0 }; rp.model = 0;   // the suspension replay
        const double kTravel = 500.0 / (2.0 * 3.14159265358979 * std::sqrt(104.0));
        const double kSmooth10 = 1.0 / std::sqrt(1.0 + 0.25);
        Layer L; L.configureFx(FxType::Road, p); L.configureRoad(rp);
        double pk[3] = {}; double t = 0.0;
        for (int i = 0; i < 4000; ++i, t += DT)
        {
            for (int w = 0; w < 4; ++w) L.driveRoad(w, w == WheelFL ? 500.0 * std::sin(2.0 * 3.14159265358979 * 10.0 * t) : 0.0);
            L.step(DT);
            if (i >= 2000) for (int a = 0; a < 3; ++a) pk[a] = std::max(pk[a], std::fabs(L.overlayFor(a)));
        }
        approx(pk[0], 100.0 * kTravel / 16.0 * kSmooth10, 3.0, "road: FL corner replays FL travel (full mm scaled, smoothed)");
        CHECK(pk[1] < 1e-9, "road: the RR corner route carries nothing when only FL moves");
        approx(pk[2], pk[0], 1e-9, "road: the all route carries the biggest corner");
        CHECK(L.roadReplaying(), "road: replay is active while corners are driven");

        // The high-pass removes slow body motion: a steady 50 mm/s (the
        // cue's heave) settles to a bounded travel of V/(2 pi hp) = 4 mm,
        // not a runaway integral; at 10 Hz the same velocity gives far less.
        Layer H; H.configureFx(FxType::Road, p); H.configureRoad(rp);
        double steady = 0.0;
        for (int i = 0; i < 8000; ++i)
        {
            for (int w = 0; w < 4; ++w) H.driveRoad(w, 50.0);
            H.step(DT);
            steady = H.roadWheelTravelMm(WheelFL);
        }
        approx(steady, 50.0 / (2.0 * 3.14159265358979 * 2.0), 0.2, "road: a constant velocity settles at V/(2 pi cut hz), never runs away");

        // Not driven = the replay releases and the texture path is back:
        // after the gate fades, driveFx on the Road slot plays the oscillator.
        Layer R; R.configureFx(FxType::Road, p); R.configureRoad(rp);
        for (int i = 0; i < 400; ++i) { for (int w = 0; w < 4; ++w) R.driveRoad(w, 300.0 * std::sin(i * 0.3)); R.step(DT); }
        for (int i = 0; i < 1000; ++i) R.step(DT);
        CHECK(!R.roadReplaying(), "road: replay releases when corners stop arriving");
        EffectParams q = p; q.freqHz = 28.0; q.routes[0] = { 0, 1.0 };
        R.configureFx(FxType::Road, q);
        double tex = 0.0;
        for (int i = 0; i < 1000; ++i) { R.driveFx(FxType::Road, 1.0, 0.0); R.step(DT); tex = std::max(tex, std::fabs(R.overlayFor(0))); }
        CHECK(tex > 90.0, "road: the roadNoise texture plays when no corners are sent");

        // Position sink: the FL replay asked at 2 mm is derated at the
        // representative 8 Hz: allowed = 800/(2 pi 8)^2 = 0.317 mm.
        Layer P; P.setSinkKind(0, SinkKind::Position); P.setPositionLimits(0, 80.0, 800.0, 3.0);
        EffectParams r; r.ampPct = 100.0; r.jitter = 0.0; r.routes[0] = { 0, 2.0, Part::FL };
        P.configureFx(FxType::Road, r); P.configureRoad(rp);
        double pmm = 0.0; t = 0.0;
        for (int i = 0; i < 4000; ++i, t += DT)
        {
            for (int w = 0; w < 4; ++w) P.driveRoad(w, 500.0 * std::sin(2.0 * 3.14159265358979 * 10.0 * t));
            P.step(DT);
            if (i >= 2000) pmm = std::max(pmm, std::fabs(P.overlayFor(0)) / 100.0);
        }
        const double w8 = 2.0 * 3.14159265358979 * 8.0;
        approx(pmm, (kTravel / 16.0 * kSmooth10) * std::min(2.0, 800.0 / (w8 * w8)), 0.02, "road: position sink derates the replay at the bump rate");

        // The knee: linear to 0.6 of full, then bending towards 1 with the
        // same slope at the knee; never a clip, never past 1.
        using haptics::RoadModel;
        approx(RoadModel::softKnee(0.5),  0.5,   1e-12, "road knee: linear below 0.6 of full");
        approx(RoadModel::softKnee(1.0),  0.6 + 0.4 * std::tanh(1.0), 1e-9, "road knee: a full-mm bump plays at 0.905, not clipped flat");
        approx(RoadModel::softKnee(-1.0), -(0.6 + 0.4 * std::tanh(1.0)), 1e-9, "road knee: symmetric");
        CHECK(RoadModel::softKnee(4.0) < 1.0 && RoadModel::softKnee(4.0) > 0.999, "road knee: a 4x bump approaches 1, never past it");
        CHECK(std::fabs(RoadModel::softKnee(0.6001) - RoadModel::softKnee(0.5999)) < 3e-4, "road knee: continuous at the knee");

        // amp 0 plays nothing and clears.
        EffectParams z = p; z.ampPct = 0.0;
        Layer Z; Z.configureFx(FxType::Road, z); Z.configureRoad(rp);
        double zk = 0.0;
        for (int i = 0; i < 200; ++i) { for (int w = 0; w < 4; ++w) Z.driveRoad(w, 400.0); Z.step(DT); zk = std::max(zk, std::fabs(Z.overlayFor(0))); }
        CHECK(zk < 1e-9 && Z.fxLevel(static_cast<int>(FxType::Road)) == 0.0, "road: amp 0 plays nothing");
    }

    // ================= road: the tyre and chassis models; the kerb =================
    {
        using haptics::Part; using haptics::RoadParams; using haptics::KerbParams; using haptics::RoadOut;
        using haptics::RoadModel; using haptics::KerbModel; using haptics::QuarterCar;
        const double PI = 3.14159265358979;
        auto rms = [](const std::vector<double>& v, size_t from)
        { double s = 0.0; size_t n = 0; for (size_t i = from; i < v.size(); ++i) { s += v[i] * v[i]; ++n; } return n ? std::sqrt(s / n) : 0.0; };

        // Quarter car: the body resonates at body hz and is isolated from a
        // fast bump; the wheel follows a slow road and not a fast one.
        {
            const QuarterCar::K k = QuarterCar::params(3.0, 16.0, 0.3);
            auto gain = [&](double f, bool body) {
                QuarterCar q; double pk = 0.0; const double dt = 0.0005; const int n = static_cast<int>(6.0 / dt);
                for (int i = 0; i < n; ++i) { q.step(dt, std::sin(2.0 * PI * f * i * dt), k); if (i > n / 2) pk = std::max(pk, std::fabs(body ? q.zs : q.zu)); }
                return pk; };
            double bestF = 0.0, best = 0.0;
            for (double f = 1.0; f <= 8.0; f += 0.25) { const double g = gain(f, true); if (g > best) { best = g; bestF = f; } }
            approx(bestF, 3.0, 0.5, "road tyre: the body resonates at body hz");
            CHECK(best > 1.3, "road tyre: ...lifting more than the road does there (damping 0.3)");
            CHECK(best < 3.0, "road tyre: ...and settling: damping 0.3 is the bounce's own, not floaty");
            CHECK(gain(30.0, true) < 0.2, "road tyre: the body is isolated from a fast bump");
            CHECK(std::fabs(gain(1.0, false) - 1.0) < 0.15 && gain(60.0, false) < 0.5, "road tyre: the wheel follows a slow road, not a fast one");
        }

        // Tyre model, its own road: laid out by distance, so the rears meet
        // the fronts' road a wheelbase / speed later; rough x scales it, so
        // does the surface class under the tyre; it stands still with the car.
        {
            RoadParams rp; rp.model = 1; rp.rough = 2.0; rp.bodyMm = 50.0;   // a big full scale: the raw mm are linear
            const double dt = 0.0005, v = 25.0;
            auto run = [&](RoadModel& m, double speed, int n, std::vector<double>* fl, std::vector<double>* rl) {
                for (int i = 0; i < n; ++i)
                {
                    m.driveSpeed(speed); m.step(dt, rp);
                    if (fl) fl->push_back(m.wheelTravelMm(WheelFL));
                    if (rl) rl->push_back(m.wheelTravelMm(WheelRL));
                } };
            RoadModel m; m.driveGeometry(2.5, 1.6);
            std::vector<double> fl, rl; run(m, v, 8000, &fl, &rl);
            int bestLag = -1; double bestErr = 1e18;
            for (int lag = 150; lag < 250; ++lag)
            { double e = 0.0; for (size_t i = 4000; i < 7900; ++i) e += std::fabs(rl[i] - fl[i - lag]); if (e < bestErr) { bestErr = e; bestLag = lag; } }
            approx(bestLag * dt, 2.5 / v, 0.002, "road tyre: the rears meet the fronts' road a wheelbase / speed later");
            CHECK(rms(fl, 4000) > 0.01, "road tyre: its own road moves the body");
            CHECK(m.distance() > 99.0 && m.distance() < 101.0, "road tyre: the road moves by the distance the car covers");

            RoadParams r4 = rp; r4.rough = 4.0;
            RoadModel a, b; a.driveGeometry(2.5, 1.6); b.driveGeometry(2.5, 1.6);
            std::vector<double> fa, fb;
            for (int i = 0; i < 6000; ++i)
            { a.driveSpeed(v); a.step(dt, rp); b.driveSpeed(v); b.step(dt, r4); fa.push_back(a.wheelTravelMm(WheelFL)); fb.push_back(b.wheelTravelMm(WheelFL)); }
            approx(rms(fb, 2000) / rms(fa, 2000), 2.0, 0.01, "road tyre: rough x 2 is twice the road");

            std::vector<double> still; run(m, 0.0, 8000, &still, nullptr);
            CHECK(rms(still, 6000) < 0.01 * rms(fl, 4000), "road tyre: the road stands still with the car");

            // The tile's wave draws what is felt: a playing model counts.
            EffectParams fp; fp.ampPct = 100.0; fp.routes[0] = { 0, 1.0 };
            Layer F; F.configureFx(haptics::FxType::Road, fp); F.configureRoad(rp);
            for (int i = 0; i < 4000; ++i) { F.driveRoadSpeed(v); F.step(DT); }
            CHECK(F.fxOutputLevel(static_cast<int>(haptics::FxType::Road)) > 0.0, "road tyre: the felt level counts the model, not only the texture");
        }

        // Suspension model's roughness: the sim's travel plus the road's fine
        // roughness through the corner (the wheel against the body). rough 0
        // = the travel alone, exactly; it rises with speed, stops with the
        // car, reaches the rears a wheelbase later, is rougher on gravel
        // (the short wavelengths only), and a change of surface does not
        // thump, in this model or the tyre model.
        {
            const double dt = 0.0005;
            RoadParams sp; sp.model = 0; sp.rough = 2.0; sp.fullMm = 200.0;   // a big full scale: the raw mm are linear
            auto rough = [&](double speed, double roughX, int surface, int n, std::vector<double>* fl, std::vector<double>* rl) {
                RoadModel m; m.driveGeometry(2.5, 1.6); RoadParams q = sp; q.rough = roughX;
                for (int i = 0; i < n; ++i)
                {
                    for (int w = 0; w < 4; ++w) { m.driveTravel(w, 0.0); m.driveSurface(w, surface); }
                    m.driveSpeed(speed); m.step(dt, q);
                    if (fl) fl->push_back(m.wheelTravelMm(WheelFL));
                    if (rl) rl->push_back(m.wheelTravelMm(WheelRL));
                }
            };
            std::vector<double> none, slow, fast, fastRl, gravel, stopped;
            rough(25.0, 0.0, SurfTarmac, 6000, &none, nullptr);
            rough(8.0,  2.0, SurfTarmac, 8000, &slow, nullptr);
            rough(25.0, 2.0, SurfTarmac, 8000, &fast, &fastRl);
            rough(25.0, 2.0, SurfGravel, 8000, &gravel, nullptr);
            rough(0.0,  2.0, SurfTarmac, 6000, &stopped, nullptr);
            double noneMax = 0.0; for (double x : none) noneMax = std::max(noneMax, std::fabs(x));
            CHECK(noneMax == 0.0, "road suspension: rough x 0 = the sim's travel alone, nothing added");
            CHECK(rms(fast, 3000) > 0.05, "road suspension: the road's roughness moves the suspension at speed");
            CHECK(rms(fast, 3000) > 1.5 * rms(slow, 3000), "road suspension: rougher with speed");
            CHECK(rms(stopped, 2000) < 1e-9, "road suspension: nothing from the road while the car stands still");
            CHECK(rms(gravel, 3000) > 1.5 * rms(fast, 3000), "road suspension: gravel under the tyre is clearly rougher");
            int bestLag = -1; double bestErr = 1e18;
            for (int lag = 150; lag < 250; ++lag)
            { double e = 0.0; for (size_t i = 4000; i < 7900; ++i) e += std::fabs(fastRl[i] - fast[i - lag]); if (e < bestErr) { bestErr = e; bestLag = lag; } }
            approx(bestLag * dt, 2.5 / 25.0, 0.002, "road suspension: the rears meet the fronts' roughness a wheelbase / speed later");

            // Onto gravel at speed: no thump at the change, in either model.
            for (int model : { 0, 1 })
            {
                RoadParams q = sp; q.model = model; q.bodyMm = 200.0;
                RoadModel m; m.driveGeometry(2.5, 1.6);
                double atChange = 0.0, onGravel = 0.0;
                for (int i = 0; i < 16000; ++i)
                {
                    const int s = (i >= 8000) ? SurfGravel : SurfTarmac;
                    for (int w = 0; w < 4; ++w) { m.driveTravel(w, 0.0); m.driveSurface(w, s); }
                    m.driveSpeed(25.0); m.step(dt, q);
                    const double a = std::fabs(m.wheelTravelMm(WheelFL));
                    if (i >= 8000 && i < 8600) atChange = std::max(atChange, a);
                    if (i >= 10000) onGravel = std::max(onGravel, a);
                }
                CHECK(atChange <= 1.5 * onGravel, model == 0 ? "road suspension: onto gravel without a thump"
                                                             : "road tyre: onto gravel without a thump");
            }
        }

        // Tyre model, the sim's road: a 10 mm step under FL lifts the FL
        // body (rough 0: nothing else); a steady climb is not a bump.
        {
            RoadParams rp; rp.model = 1; rp.rough = 0.0; rp.bodyMm = 50.0;
            const double dt = 0.0005;
            RoadModel m; double pkFL = 0.0, pkRR = 0.0;
            for (int i = 0; i < 6000; ++i)
            {
                for (int w = 0; w < 4; ++w) m.driveHeight(w, (w == WheelFL && i >= 2000) ? 10.0 : 0.0);
                m.driveSpeed(20.0); m.step(dt, rp);
                pkFL = std::max(pkFL, std::fabs(m.wheelTravelMm(WheelFL)));
                pkRR = std::max(pkRR, std::fabs(m.wheelTravelMm(WheelRR)));
            }
            CHECK(pkFL > 2.0, "road tyre: a step in the road under FL moves the FL body");
            CHECK(pkRR < 1e-9, "road tyre: ...and nothing at RR");
            RoadModel h; double pkH = 0.0;
            for (int i = 0; i < 20000; ++i)
            {
                for (int w = 0; w < 4; ++w) h.driveHeight(w, 100.0 * i * dt);   // climbing 100 mm/s
                h.driveSpeed(20.0); h.step(dt, rp);
                if (i >= 10000) pkH = std::max(pkH, std::fabs(h.wheelTravelMm(WheelFL)));
            }
            CHECK(pkH < 0.05, "road tyre: a steady climb is a hill, not a bump");
        }

        // Chassis model: the body's heave acceleration integrated twice above
        // cut hz. A 6 Hz, 2 m/s^2 heave is 2000 / (2 pi 6)^2 = 1.41 mm, less
        // the filters' share at 6 Hz (0.986 x 0.949^2 x 0.972 = 0.863).
        {
            RoadParams rp; rp.model = 2; rp.bodyMm = 50.0; rp.hpHz = 2.0;
            const double dt = 0.0005, w6 = 2.0 * PI * 6.0;
            RoadModel m; double pk = 0.0, pkRR = 0.0;
            for (int i = 0; i < 12000; ++i)
            {
                m.driveChassis(2.0 * std::sin(w6 * i * dt), 0.0, 0.0); m.step(dt, rp);
                if (i > 6000) { pk = std::max(pk, std::fabs(m.wheelTravelMm(WheelFL))); pkRR = std::max(pkRR, std::fabs(m.wheelTravelMm(WheelRR))); }
            }
            approx(pk, 2000.0 / (w6 * w6) * 0.863, 0.06, "road chassis: heave acceleration integrates to the body's movement");
            approx(pkRR, pk, 1e-9, "road chassis: signs not learned yet = heave only, every corner the same");

            // Signs from the suspension. The body (heave Z at 5 Hz, roll R at
            // 4 Hz, left up positive) moves opposite to its springs'
            // compression, so a compression-positive sim sends each corner's
            // travel against the body above it.
            auto learn = [&](double suspSign, double rollSign, int& pol, int& roll, double& flMinusFr) {
                RoadModel c; RoadParams cp = rp;
                const double w5 = 2.0 * PI * 5.0, w4 = 2.0 * PI * 4.0;
                double diff = 0.0;
                for (int i = 0; i < 24000; ++i)
                {
                    const double t = i * dt;
                    const double z = 1.0 * std::sin(w5 * t);                    // body heave, mm
                    const double r = 0.3 * std::sin(w4 * t);                    // body roll, deg, left up
                    const double side = r * PI / 180.0 * 800.0;                  // left corner up, mm
                    const double comp[4] = { -(z + side), -(z - side), -(z + side), -(z - side) };
                    for (int w = 0; w < 4; ++w) c.driveTravel(w, suspSign * comp[w]);
                    c.driveChassis(-w5 * w5 * z / 1000.0, 0.0, rollSign * r);
                    c.step(dt, cp);
                    if (i >= 20000) diff = std::max(diff, std::fabs(c.wheelTravelMm(WheelFL) - c.wheelTravelMm(WheelFR)));
                }
                pol = c.learnedPolarity(); roll = c.learnedRollSign(); flMinusFr = diff;
            };
            int pol = 0, roll = 0; double d = 0.0;
            learn(1.0, 1.0, pol, roll, d);
            CHECK(pol == -1 && roll == 1, "road chassis: compression-positive suspension, left-up roll: learned as such");
            CHECK(d > 2.0, "road chassis: once learned, roll moves the left and right corners apart");
            learn(-1.0, 1.0, pol, roll, d);
            CHECK(pol == 1 && roll == 1, "road chassis: extension-positive suspension: the same roll sign");
            learn(1.0, -1.0, pol, roll, d);
            CHECK(pol == -1 && roll == -1, "road chassis: a sim whose roll is right-up positive is turned round");
        }

        // Kerb: ribs every pitch hum at speed / pitch on a force sink; on
        // and off within a few ms; a post gets a thud on and off, not a held
        // lift; other corners nothing; stronger with speed.
        {
            EffectParams p; p.ampPct = 100.0; p.jitter = 0.0;
            const KerbParams k;   // pitch 25 cm, rise 8, rib 3, full 10
            const double dt = 0.0005;
            auto run = [&](double speed, std::vector<double>& f, std::vector<double>& pos, std::vector<double>& lvl, double& rr) {
                KerbModel m; rr = 0.0;
                for (int i = 0; i < 6000; ++i)   // 3 s: FL on the kerb from 0.5 to 2.0 s
                {
                    const double t = i * dt;
                    m.driveSpeed(speed);
                    m.drive(WheelFL, (t >= 0.5 && t < 2.0) ? 1.0 : 0.0);
                    m.step(dt, p, k);
                    const RoadOut o = m.outputFor(Part::FL);
                    f.push_back(o.force); pos.push_back(o.pos); lvl.push_back(m.wheelLevel(WheelFL));
                    const RoadOut q = m.outputFor(Part::RR);
                    rr = std::max(rr, std::max(std::fabs(q.force), std::fabs(q.pos)));
                }
            };
            std::vector<double> f, pos, lvl; double rr = 0.0;
            run(20.0, f, pos, lvl, rr);
            int xr = 0; for (size_t i = 1601; i < 3600; ++i) if ((f[i] >= 0.0) != (f[i - 1] >= 0.0)) ++xr;
            approx(xr / 2.0, 20.0 / 0.25, 3.0, "kerb: the ribs hum at speed / pitch (80 Hz at 20 m/s, 25 cm)");
            CHECK(lvl[1020] > 0.99 && lvl[4060] < 1e-9, "kerb: on within 10 ms, off within 30 ms");
            double on = 0.0, mid = 0.0, off = 0.0;
            for (size_t i = 1000; i < 1400; ++i) on  = std::max(on,  pos[i]);
            for (size_t i = 3000; i < 3990; ++i) mid = std::max(mid, std::fabs(pos[i]));
            for (size_t i = 4000; i < 4400; ++i) off = std::min(off, pos[i]);
            CHECK(on > 0.5 && off < -0.4, "kerb post: a thud up as the tyre steps on, one down as it steps off");
            CHECK(mid < 0.3 * on, "kerb post: not a held lift while riding along it");
            CHECK(rr < 1e-9, "kerb: the corners off the kerb carry nothing");
            std::vector<double> fs, ps, ls; double rs = 0.0;
            run(8.0, fs, ps, ls, rs);
            CHECK(rms(f, 1600) > 2.0 * rms(fs, 1600), "kerb: the ribs are stronger at speed");
        }
    }

    // ================= Surface: stones, crunch, studs, puddles =================
    {
        using haptics::SurfaceModel; using haptics::SurfaceParams; using haptics::Part; using haptics::RoadModel;
        using haptics::RoadParams;
        const double dt = 0.0005;
        auto rmsOf = [](const std::vector<double>& v, size_t from)
        { double s = 0.0; size_t n = 0; for (size_t i = from; i < v.size(); ++i) { s += v[i] * v[i]; ++n; } return n ? std::sqrt(s / n) : 0.0; };
        // Run the model on one surface for every wheel; FL's force trace.
        auto runS = [&](SurfaceModel& m, int cls, double wet, double speed, const SurfaceParams& q, int n, std::vector<double>* force) {
            for (int i = 0; i < n; ++i)
            {
                for (int w = 0; w < 4; ++w) { m.driveSurface(w, cls); m.driveWet(w, wet); }
                if (speed >= 0.0) m.driveSpeed(speed);
                m.step(dt, q);
                if (force) force->push_back(m.outputFor(Part::FL).force);
            }
        };
        const SurfaceParams q;   // stones 0.6, crunch 0.5, studs off, puddles 0.6, aqua 90, smooth 0.7

        // Stones: so many per metre, so more with speed; none on tarmac,
        // none at a standstill or once the speed stops arriving.
        {
            SurfaceModel g; runS(g, SurfGravel, 0.0, 20.0, q, 8000, nullptr);       // 4 s at 20 m/s: 80 m
            approx(static_cast<double>(g.stonesStruck(WheelFL)), 2.0 * 80.0, 40.0, "surface stones: two per metre of gravel under a tyre");
            SurfaceModel s; runS(s, SurfGravel, 0.0, 5.0, q, 8000, nullptr);
            CHECK(s.stonesStruck(WheelFL) < g.stonesStruck(WheelFL) / 2, "surface stones: fewer at walking pace");
            SurfaceModel t; std::vector<double> ft; runS(t, SurfTarmac, 0.0, 20.0, q, 4000, &ft);
            CHECK(t.stonesStruck(WheelFL) == 0 && rmsOf(ft, 0) == 0.0, "surface: dry tarmac has nothing for this tile");
            SurfaceModel u; runS(u, SurfGravel, 0.0, -1.0, q, 4000, nullptr);
            CHECK(u.stonesStruck(WheelFL) == 0, "surface stones: no speed arriving, no stones");
            SurfaceModel z; std::vector<double> fz; SurfaceParams q0 = q; q0.stones = 0.0;
            runS(z, SurfGravel, 0.0, 20.0, q0, 4000, &fz);
            CHECK(rmsOf(fz, 0) < 0.05, "surface stones: stones x 0, (almost) nothing on gravel");
        }
        // Crunch on snow, studs on ice.
        {
            SurfaceModel sn; std::vector<double> fs; runS(sn, SurfSnow, 0.0, 20.0, q, 6000, &fs);
            CHECK(rmsOf(fs, 2000) > 0.03, "surface crunch: snow crunches under the tread");
            SurfaceParams qs = q; qs.studs = 1.0;
            SurfaceModel ice; std::vector<double> fi; runS(ice, SurfIce, 0.0, 20.0, q, 4000, &fi);
            SurfaceModel st;  std::vector<double> fst; runS(st, SurfIce, 0.0, 20.0, qs, 4000, &fst);
            CHECK(rmsOf(fi, 0) == 0.0, "surface: ice without studs is silent here");
            CHECK(rmsOf(fst, 1000) > 0.1, "surface studs: studded tyres buzz on ice");
        }
        // Puddles: laid out along the road, the same puddle every pass; a
        // dry road has none; wet, about a third of the road.
        {
            int wetCells = 0, n = 0;
            for (double x = 0.0; x < 2000.0; x += 0.1, ++n)
            {
                if (SurfaceModel::puddleDepth(0, x, 1.0) > 0.0) ++wetCells;
                if (SurfaceModel::puddleDepth(0, x, 0.0) > 0.0) { wetCells = -100000; break; }
            }
            const double cover = static_cast<double>(wetCells) / n;
            CHECK(cover > 0.15 && cover < 0.5, "surface puddles: a wet road is puddled about a third of the way");
            CHECK(SurfaceModel::puddleDepth(1, 123.4, 0.8) == SurfaceModel::puddleDepth(1, 123.4, 0.8), "surface puddles: the same puddle every pass");

            // The rears meet a puddle a wheelbase after the fronts.
            SurfaceModel m; m.driveGeometry(2.5);
            std::vector<double> wf, wr;
            for (int i = 0; i < 12000; ++i)
            {
                for (int w = 0; w < 4; ++w) { m.driveSurface(w, SurfTarmac); m.driveWet(w, 1.0); }
                m.driveSpeed(20.0); m.step(dt, q);
                wf.push_back(m.water(WheelFL)); wr.push_back(m.water(WheelRL));
            }
            int bestLag = -1; double bestErr = 1e18;
            for (int lag = 200; lag < 300; ++lag)
            { double e = 0.0; for (size_t i = 6000; i < 11900; ++i) e += std::fabs(wr[i] - wf[i - lag]); if (e < bestErr) { bestErr = e; bestLag = lag; } }
            approx(bestLag * dt, 2.5 / 20.0, 0.002, "surface puddles: the rears meet the fronts' puddles a wheelbase later");

            // Into a puddle the water drags (a tug); none with puddle x 0.
            SurfaceModel a; std::vector<double> fa; runS(a, SurfTarmac, 1.0, 20.0, q, 8000, &fa);
            SurfaceParams qn = q; qn.puddles = 0.0;
            SurfaceModel b; std::vector<double> fb; runS(b, SurfTarmac, 1.0, 20.0, qn, 8000, &fb);
            double minA = 0.0; for (double x : fa) minA = std::min(minA, x);
            CHECK(minA < -0.1 && rmsOf(fb, 0) == 0.0, "surface puddles: a tug as a tyre enters a puddle at speed; none at puddle x 0");

            // Aquaplaning: above aqua km/h in deep water the tyre floats, at
            // a slow pace it never does.
            auto floated = [&](double speed) {
                SurfaceModel f; double most = 0.0;
                for (int i = 0; i < 16000; ++i)
                {
                    for (int w = 0; w < 4; ++w) { f.driveSurface(w, SurfTarmac); f.driveWet(w, 1.0); }
                    f.driveSpeed(speed); f.step(dt, q);
                    most = std::max(most, f.floating(WheelFL));
                }
                return most; };
            CHECK(floated(32.0) > 0.99, "surface aquaplaning: at 115 km/h a deep puddle floats the tyre");
            CHECK(floated(15.0) == 0.0, "surface aquaplaning: at 54 km/h it never does");
        }
        // The Road tile's roughness follows the surface: ice glassy, water
        // smoothing the texture, a floating tyre with no road under it.
        {
            RoadParams rp; rp.model = 0; rp.rough = 2.0; rp.fullMm = 200.0;
            auto travel = [&](int cls, double water, double floating) {
                RoadModel m; m.driveGeometry(2.5, 1.6); std::vector<double> v;
                for (int i = 0; i < 8000; ++i)
                {
                    for (int w = 0; w < 4; ++w) { m.driveTravel(w, 0.0); m.driveSurface(w, cls); m.driveWater(w, water, floating); }
                    m.driveSpeed(25.0); m.step(dt, rp);
                    v.push_back(m.wheelTravelMm(WheelFL));
                }
                return rmsOf(v, 3000); };
            const double tar = travel(SurfTarmac, 0.0, 0.0);
            CHECK(travel(SurfIce, 0.0, 0.0) < 0.6 * tar, "road surface: ice is smoother than tarmac");
            CHECK(travel(SurfTarmac, 1.0, 0.0) < tar, "road surface: standing water smooths the texture");
            CHECK(travel(SurfTarmac, 1.0, 1.0) < 0.05 * tar, "road surface: a floating tyre has no road under it");
        }
    }

    // ================= Wheels: out of round, flat spots, brake judder =================
    {
        using haptics::WheelsModel; using haptics::WheelsParams; using haptics::RoadParams; using haptics::Part;
        const double dt = 0.0005;
        const RoadParams rp;   // body 3 Hz, hop 16 Hz, damping 0.3, cut 2 Hz
        auto rmsOf = [](const std::vector<double>& v, size_t from)
        { double s = 0.0; size_t n = 0; for (size_t i = from; i < v.size(); ++i) { s += v[i] * v[i]; ++n; } return n ? std::sqrt(s / n) : 0.0; };
        // Roll every wheel at speed x (1 + spread[w]), brake b; a part's force trace.
        auto roll = [&](WheelsModel& m, const WheelsParams& q, double speed, const double spread[4], double brake,
                        int n, Part part, std::vector<double>* force) {
            for (int i = 0; i < n; ++i)
            {
                for (int w = 0; w < 4; ++w) { m.driveWheelSpeed(w, speed * (1.0 + spread[w])); m.driveLock(w, speed, 0.0, 1.0); }
                m.driveBrake(brake);
                m.step(dt, q, rp);
                if (force) force->push_back(m.outputFor(part).force);
            }
        };
        const double same[4] = { 0.0, 0.0, 0.0, 0.0 };
        WheelsParams balOnly; balOnly.flat = 0.0; balOnly.judder = 0.0;   // balance 1

        // Out of balance: grows steadily with speed (the imbalance's force
        // with the wheel's rate squared), through the hop (16 Hz: 32 m/s).
        {
            const double v[6] = { 10.0, 20.0, 27.0, 32.0, 45.0, 60.0 };
            double r[6] = {}, p[6] = {};
            for (int k = 0; k < 6; ++k)
            {
                WheelsModel m; std::vector<double> f, pp;
                for (int i = 0; i < 12000; ++i)
                {
                    for (int w = 0; w < 4; ++w) { m.driveWheelSpeed(w, v[k]); m.driveLock(w, v[k], 0.0, 1.0); }
                    m.step(dt, balOnly, rp);
                    f.push_back(m.outputFor(Part::FL).force); pp.push_back(m.outputFor(Part::FL).pos);
                }
                r[k] = rmsOf(f, 6000); p[k] = rmsOf(pp, 6000);
            }
            std::printf("  wheels balance x1, force rms / pos rms by km/h:");
            for (int k = 0; k < 6; ++k) std::printf(" %.0f %.3f/%.3f", v[k] * 3.6, r[k], p[k]);
            std::printf("\n");
            bool rising = true; for (int k = 1; k < 6; ++k) rising = rising && r[k] > r[k - 1];
            CHECK(rising && r[5] > 6.0 * r[0], "wheels balance: stronger and stronger with speed");
            CHECK(r[1] > 0.02 && r[5] < 0.6, "wheels balance x 1: subtle in town, clear at speed, not pinned");
        }
        // The corners drift in and out of step: all four together beat, one
        // corner on its own is steady.
        {
            const double apart[4] = { 0.0, 0.02, 0.04, 0.06 };
            WheelsModel a; std::vector<double> fa; roll(a, balOnly, 30.0, apart, 0.0, 24000, Part::All, &fa);
            WheelsModel c; std::vector<double> fc; roll(c, balOnly, 30.0, apart, 0.0, 24000, Part::FL, &fc);
            auto swing = [&](const std::vector<double>& f) {
                double lo = 1e9, hi = 0.0;
                for (size_t s = 8000; s + 400 <= f.size(); s += 400)
                { std::vector<double> win(f.begin() + static_cast<long>(s), f.begin() + static_cast<long>(s + 400)); const double x = rmsOf(win, 0); lo = std::min(lo, x); hi = std::max(hi, x); }
                return hi / std::max(1e-9, lo); };
            CHECK(swing(fa) > 2.0, "wheels: the four corners together beat as they drift in and out of step");
            CHECK(swing(fc) < 1.3, "wheels: one corner on its own is steady");
        }
        // Flat spots: a locked, sliding wheel grinds a flat, deeper with the
        // distance slid; a wheel at the edge of locking (ABS) does not.
        {
            WheelsParams q; q.balance = 0.0; q.judder = 0.0;
            WheelsModel m;
            for (int i = 0; i < 2000; ++i)   // FL locked for 1 s at 20 m/s
            {
                for (int w = 0; w < 4; ++w) { m.driveWheelSpeed(w, w == WheelFL ? 0.0 : 20.0); m.driveLock(w, 20.0, w == WheelFL ? -1.0 : 0.0, 1.0); }
                m.step(dt, q, rp);
            }
            approx(m.flatMm(WheelFL), 0.02 * 20.0, 0.01, "wheels flat: 1 s locked at 20 m/s grinds 0.4 mm");
            CHECK(m.flatMm(WheelFR) == 0.0, "wheels flat: only the locked wheel");
            std::vector<double> f, fr; WheelsModel* pm = &m;
            for (int i = 0; i < 8000; ++i)
            {
                for (int w = 0; w < 4; ++w) { pm->driveWheelSpeed(w, 20.0); pm->driveLock(w, 20.0, 0.0, 1.0); }
                pm->step(dt, q, rp);
                f.push_back(pm->outputFor(Part::FL).force); fr.push_back(pm->outputFor(Part::FR).force);
            }
            approx(m.flatMm(WheelFL), 0.4, 0.01, "wheels flat: rolling on does not deepen it");
            // Once a revolution: at 20 m/s a 2 m tyre turns every 0.1 s (200 samples).
            std::vector<double> diff; for (size_t i = 4000; i + 200 < f.size(); ++i) diff.push_back(f[i] - f[i + 200]);
            const double flatRms = rmsOf(f, 4000);
            std::printf("  wheels flat 0.4 mm at 72 km/h: force rms %.3f, peak", flatRms);
            double pk = 0.0; for (size_t i = 4000; i < f.size(); ++i) pk = std::max(pk, std::fabs(f[i])); std::printf(" %.3f\n", pk);
            CHECK(flatRms > 0.02, "wheels flat: a flat spot is felt");
            CHECK(rmsOf(diff, 0) < 0.1 * flatRms, "wheels flat: once a revolution, the same thump every time round");
            CHECK(rmsOf(fr, 4000) < 1e-9, "wheels flat: the round tyres carry nothing");

            WheelsModel abs;
            for (int i = 0; i < 4000; ++i)
            {
                for (int w = 0; w < 4; ++w) { abs.driveWheelSpeed(w, 17.0); abs.driveLock(w, 20.0, -0.15, 1.0); }
                abs.step(dt, q, rp);
            }
            CHECK(abs.flatMm(WheelFL) == 0.0, "wheels flat: a wheel kept at the edge of locking grinds no flat");
        }
        // Brake judder: only with hot discs, only under braking, twice a
        // revolution.
        {
            WheelsParams q; q.balance = 0.0; q.flat = 0.0;   // judder 0.5
            WheelsModel cold; std::vector<double> fc; roll(cold, q, 30.0, same, 1.0, 2000, Part::FL, &fc);
            CHECK(rmsOf(fc, 0) < 1e-9, "wheels judder: cold discs do not judder");
            WheelsModel hot; roll(hot, q, 50.0, same, 1.0, 20000, Part::FL, nullptr);   // 10 s hard on the brakes at 180 km/h
            CHECK(hot.discHeat(WheelFL) > 0.35, "wheels judder: long hard braking heats the discs");
            std::vector<double> fh; roll(hot, q, 30.0, same, 1.0, 2000, Part::FL, &fh);
            int xr = 0; for (size_t i = 1; i < fh.size(); ++i) if ((fh[i] >= 0.0) != (fh[i - 1] >= 0.0)) ++xr;
            CHECK(rmsOf(fh, 0) > 0.02, "wheels judder: hot discs pulse the braking force");
            approx(xr / 2.0, 2.0 * 30.0 / 2.0 * 1.0, 3.0, "wheels judder: twice a revolution (30 Hz at 30 m/s)");
            std::vector<double> fo; roll(hot, q, 30.0, same, 0.0, 2000, Part::FL, &fo);
            CHECK(rmsOf(fo, 200) < 1e-6, "wheels judder: off the brake, nothing");
        }
        // Nothing turning, nothing felt; a Test preview leaves the tyres as
        // they were.
        {
            WheelsParams q;
            WheelsModel m; std::vector<double> f;
            for (int i = 0; i < 4000; ++i) { m.step(dt, q, rp); f.push_back(m.outputFor(Part::All).force); }
            CHECK(rmsOf(f, 0) == 0.0 && m.level() == 0.0, "wheels: no speed arriving, nothing");
            for (int i = 0; i < 2000; ++i)
            {
                for (int w = 0; w < 4; ++w) m.driveWheelSpeed(w, 22.0);
                m.driveBrake(0.6); m.drivePreview(0.3, 1.0); m.step(dt, q, rp);
            }
            CHECK(m.level() > 0.05, "wheels preview: a flat and hot discs play");
            CHECK(m.flatMm(WheelFL) == 0.0 && m.discHeat(WheelFL) < 0.01, "wheels preview: the tyres' own flat and heat untouched");
        }
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

    // ================= bench pass 2: what a rig can actually feel =================
    {
        using haptics::FxType; using haptics::Part; using haptics::SinkKind;

        // Axle phase lock: FL locking hard and FR half as hard, each on its
        // own post, must move the two posts TOGETHER (same sign every cycle,
        // FR at half), never against each other, even with a rough carrier.
        {
            Layer L;
            EffectParams p; p.ampPct = 100.0; p.jitter = 0.5;
            p.routes[0] = { 0, 1.0, Part::FL };
            p.routes[1] = { 1, 1.0, Part::FR };
            L.configureFx(FxType::Lockup, p); L.configureSlip(FxType::Lockup, { 1.0, 12.0, 1.0, 16.0, 0.8 });
            int against = 0, both = 0; double ratioSum = 0.0;
            for (int i = 0; i < 6000; ++i)
            {
                L.driveSlip(FxType::Lockup, WheelFL, 1.0, 0.0);
                L.driveSlip(FxType::Lockup, WheelFR, 0.5, 0.0);
                L.step(DT);
                const double a = L.overlayFor(0), b = L.overlayFor(1);
                if (i > 400 && std::fabs(a) > 5.0 && std::fabs(b) > 1.0)
                {
                    ++both; ratioSum += b / a;
                    if ((a > 0.0) != (b > 0.0)) ++against;
                }
            }
            CHECK(both > 1000 && against == 0, "axle lock: the two front posts never move against each other");
            approx(both ? ratioSum / both : 0.0, 0.5, 0.02, "axle lock: the lighter corner plays the same wave at its own level");
        }

        // A transient on a position axis is one thud it can carry (about
        // 12 hz, ~83 ms), not the 60 hz burst a belt gets; a shaker route
        // now plays it too (fire() used to drop shaker routes).
        {
            Layer L;
            L.setSinkKind(0, SinkKind::Position); L.setPositionLimits(0, 125.0, 10000.0, 3.0);
            EffectParams g; g.ampPct = 100.0; g.freqHz = 60.0; g.durMs = 25.0;
            g.routes[0] = { 0, 1.0 };           // position axis, 1 mm at 100 %
            g.routes[1] = { 1, 1.0 };           // torque axis
            haptics::Route sh; sh.axis = -1; sh.shaker = 0; sh.gain = 1.0; g.routes[2] = sh;
            L.configure(haptics::EventType::GearShift, g);
            L.fire(haptics::EventType::GearShift);
            double pos = 0.0, tq = 0.0, shk = 0.0; int lastPos = -1, lastTq = -1;
            for (int i = 0; i < 400; ++i)
            {
                L.step(DT);
                if (std::fabs(L.overlayFor(0)) > 1e-6) { pos = std::max(pos, std::fabs(L.overlayFor(0)) / 100.0); lastPos = i; }
                if (std::fabs(L.overlayFor(1)) > 1e-6) { tq = std::max(tq, std::fabs(L.overlayFor(1))); lastTq = i; }
                shk = std::max(shk, std::fabs(L.shakerSample(0)));
            }
            CHECK(pos > 0.9 && pos <= 1.0001, "thud: the gear shift moves a position axis its full 1 mm (a 60 hz burst would get 0.07)");
            approx((lastPos + 1) * DT * 1000.0, 1000.0 / 12.0, 2.0, "thud: one cycle at 12 hz on the position axis");
            CHECK(tq > 90.0 && (lastTq + 1) * DT * 1000.0 <= 25.5, "thud: the torque axis still gets the 25 ms burst");
            CHECK(shk > 0.5, "thud: a shaker route plays the transient");
        }

        // ABS and TC are a fast drop and a slower recovery each cycle,
        // averaging to zero (a position axis does not drift).
        {
            using haptics::Layer;
            approx(Layer::dropRecover(0.0, 0.25), 1.0, 1e-12, "abs shape: a cycle starts at the top");
            approx(Layer::dropRecover(2.0 * 3.14159265358979 * 0.25, 0.25), -1.0, 1e-9, "abs shape: at the bottom a quarter of the way through");
            approx(Layer::dropRecover(2.0 * 3.14159265358979 * 0.625, 0.25), 0.0, 1e-9, "abs shape: half way back up at five eighths");
            for (FxType t : { FxType::AbsPulse, FxType::TcPulse })
            {
                Layer L; EffectParams p; p.ampPct = 100.0; p.freqHz = 10.0; p.routes[0] = { 0, 1.0 };
                L.configureFx(t, p);
                L.configurePulse(t, { 1.0, 0.0, 0.0, 0.0, 40.0 });   // the knock, no spread, no buzz: one clean pulse
                double sum = 0.0, pk = 0.0, prev = 0.0; int falling = 0, n = 0;
                for (int i = 0; i < 8000; ++i)
                {
                    L.driveFx(t, 1.0, 0.0); L.step(DT);
                    const double x = L.overlayFor(0);
                    if (i >= 4000) { sum += x; pk = std::max(pk, std::fabs(x)); if (x < prev) ++falling; ++n; }
                    prev = x;
                }
                const double share = static_cast<double>(falling) / n;
                const bool abs = (t == FxType::AbsPulse);
                CHECK(std::fabs(sum / n) < 0.02 * pk, abs ? "abs shape: averages to zero" : "tc shape: averages to zero");
                approx(share, abs ? 0.25 : 0.33, 0.02, abs ? "abs shape: falls in a quarter of each cycle" : "tc shape: falls in a third of each cycle");
            }
        }

        // ABS and TC: sharp, spread, slow x, buzz (PulseModel.h).
        {
            using haptics::Layer; using haptics::PulseParams; using haptics::Part; using haptics::Route;
            // Run a pulse tile with one torque route per given part; return
            // each route's output trace.
            auto runPulse = [&](FxType t, const PulseParams& q, double kmh, std::vector<Part> parts, int n) {
                Layer L; EffectParams p; p.ampPct = 100.0; p.freqHz = 10.0;
                for (size_t k = 0; k < parts.size(); ++k) { Route r; r.axis = static_cast<int>(k); r.gain = 1.0; r.part = parts[k]; p.routes[k] = r; }
                L.configureFx(t, p); L.configurePulse(t, q);
                std::vector<std::vector<double>> out(parts.size());
                for (int i = 0; i < n; ++i)
                {
                    L.driveFx(t, 1.0, 0.0); if (t == FxType::AbsPulse) L.driveAbsSpeed(kmh); L.step(DT);
                    if (i >= 2000) for (size_t k = 0; k < parts.size(); ++k) out[k].push_back(L.overlayFor(static_cast<int>(k)));
                }
                return out;
            };
            auto fallShare = [](const std::vector<double>& v) {
                int falling = 0; for (size_t i = 1; i < v.size(); ++i) if (v[i] < v[i - 1]) ++falling;
                return static_cast<double>(falling) / (v.size() - 1); };
            auto crossings = [](const std::vector<double>& v) {
                std::vector<int> at; for (size_t i = 1; i < v.size(); ++i) if (v[i - 1] < 0.0 && v[i] >= 0.0) at.push_back(static_cast<int>(i));
                return at; };
            auto mean = [](const std::vector<double>& v) { double s = 0.0; for (double x : v) s += x; return s / v.size(); };

            // sharp: 0 round (falls in half the cycle), 1 the knock.
            approx(fallShare(runPulse(FxType::AbsPulse, { 0.0, 0.0, 0.0, 0.0, 40.0 }, -1.0, { Part::All }, 10000)[0]), 0.5, 0.02,
                   "abs sharp 0: a round wave, falling half of each cycle");
            approx(fallShare(runPulse(FxType::AbsPulse, { 0.5, 0.0, 0.0, 0.0, 40.0 }, -1.0, { Part::All }, 10000)[0]), 0.375, 0.02,
                   "abs sharp 0.5: between round and the knock");
            approx(fallShare(runPulse(FxType::TcPulse, { 0.0, 0.0, 0.0, 0.0, 40.0 }, -1.0, { Part::All }, 10000)[0]), 0.5, 0.02,
                   "tc sharp 0: a round wave too");

            // spread: 0 = every cycle the same length; 1 = they vary.
            auto cycleVar = [&](double spread) {
                const auto c = crossings(runPulse(FxType::TcPulse, { 1.0, spread, 0.0, 0.0, 40.0 }, -1.0, { Part::All }, 16000)[0]);
                std::vector<double> len; for (size_t i = 1; i < c.size(); ++i) len.push_back(c[i] - c[i - 1]);
                const double m = mean(len); double s = 0.0; for (double x : len) s += (x - m) * (x - m);
                return std::sqrt(s / len.size()) / m; };
            CHECK(cycleVar(0.0) < 0.02, "tc spread 0: every cut the same length");
            CHECK(cycleVar(1.0) > 0.08, "tc spread 1: each cut a little different");

            // ABS corners: spread 0 in step (a corner = the whole car);
            // spread 1 each on its own rate, so they drift in and out of step.
            auto corr = [&](const std::vector<double>& a, const std::vector<double>& b) {
                double ab = 0.0, aa = 0.0, bb = 0.0;
                for (size_t i = 0; i < a.size(); ++i) { ab += a[i] * b[i]; aa += a[i] * a[i]; bb += b[i] * b[i]; }
                return ab / std::sqrt(aa * bb); };
            const auto inStep = runPulse(FxType::AbsPulse, { 1.0, 0.0, 0.0, 0.0, 40.0 }, -1.0, { Part::FL, Part::FR, Part::All }, 10000);
            const auto apart  = runPulse(FxType::AbsPulse, { 1.0, 1.0, 0.0, 0.0, 40.0 }, -1.0, { Part::FL, Part::FR, Part::All }, 10000);
            CHECK(corr(inStep[0], inStep[1]) > 0.999 && corr(inStep[0], inStep[2]) > 0.999, "abs spread 0: the corners in step, one pulse");
            CHECK(std::fabs(corr(apart[0], apart[1])) < 0.5, "abs spread 1: FL and FR drift in and out of step");
            double pkCorner = 0.0, pkAll = 0.0;
            for (double x : apart[0]) pkCorner = std::max(pkCorner, std::fabs(x));
            for (double x : apart[2]) pkAll    = std::max(pkAll,    std::fabs(x));
            CHECK(pkAll <= 100.0 + 1e-9 && pkCorner > 90.0, "abs: all plays the corners together, never more than one full pulse");
            CHECK(std::fabs(mean(apart[0])) < 0.03 * pkCorner && std::fabs(mean(apart[2])) < 0.03 * pkCorner,
                  "abs: each corner and the sum average to zero (a post does not drift)");

            // slow x: the cycle slows towards a stop; unknown speed = full rate.
            auto rate = [&](double kmh) {
                return static_cast<double>(crossings(runPulse(FxType::AbsPulse, { 1.0, 0.0, 0.5, 0.0, 40.0 }, kmh, { Part::FL }, 22000)[0]).size()) / 10.0; };
            approx(rate(100.0), 10.0, 0.3, "abs slow: full rate at speed");
            approx(rate(20.0), 10.0 * 0.625, 0.3, "abs slow 0.5: 20 km/h runs at 62.5% (half rate at a standstill)");
            approx(rate(-1.0), 10.0, 0.3, "abs slow: speed unknown, full rate");

            // buzz: the pump and valves under the pulse, at buzz hz.
            const auto withBuzz = runPulse(FxType::AbsPulse, { 1.0, 0.0, 0.0, 1.0, 40.0 }, -1.0, { Part::All }, 10000);
            std::vector<double> diff(withBuzz[0].size());
            for (size_t i = 0; i < diff.size(); ++i) diff[i] = withBuzz[0][i] - inStep[2][i];
            approx(static_cast<double>(crossings(diff).size()) / 4.0, 40.0, 4.0, "abs buzz: a buzz at buzz hz under the pulse");

            // Slip link: while ABS works each corner's valve shapes that
            // wheel's slip tiles. ABS's own amp 0 (felt only through the slip
            // tiles); the sim shows no lock on FL, a full scrub on FL, full
            // spin on RR.
            auto linked = [&](double link, double simLock, std::vector<double>& lockFL, std::vector<double>& scrubFL,
                              std::vector<double>& spinRR, std::vector<double>& valveFL) {
                Layer L; EffectParams a; a.ampPct = 0.0; a.freqHz = 10.0;
                EffectParams s; s.ampPct = 100.0; s.routes[0] = { 0, 1.0 };
                L.configureFx(FxType::AbsPulse, a); L.configureFx(FxType::Lockup, s); L.configureFx(FxType::Skid, s);
                PulseParams q{ 1.0, 0.0, 0.0, 0.0, 40.0 }; q.slipLink = link; q.lockLink = 0.7; q.scrubLink = 0.3;
                L.configurePulse(FxType::AbsPulse, q);
                for (int i = 0; i < 6000; ++i)
                {
                    L.driveFx(FxType::AbsPulse, 1.0, 0.0);
                    for (int w = 0; w < 4; ++w) L.driveSlip(FxType::Lockup, w, w == WheelFL ? simLock : 0.0, w == WheelRR ? 1.0 : 0.0);
                    L.driveSlip(FxType::Skid, WheelFL, 1.0, 0.0);
                    L.step(DT);
                    if (i >= 2000)
                    {
                        lockFL.push_back(L.slipWheelLevel(FxType::Lockup, WheelFL));
                        scrubFL.push_back(L.slipWheelLevel(FxType::Skid, WheelFL));
                        spinRR.push_back(L.slipWheelLevel(FxType::Lockup, WheelRR));
                        valveFL.push_back(L.absModel().channel(WheelFL));
                    }
                }
            };
            auto lo = [](const std::vector<double>& v) { double m = 1e9; for (double x : v) m = std::min(m, x); return m; };
            auto hi = [](const std::vector<double>& v) { double m = -1e9; for (double x : v) m = std::max(m, x); return m; };
            std::vector<double> lk, sc, sp, vf;
            linked(1.0, 0.0, lk, sc, sp, vf);
            CHECK(hi(lk) > 0.4 && hi(lk) <= 0.6 + 1e-9 && lo(lk) < 0.3, "abs slip link: the lock texture surges each cycle at the edge of locking, never locked");
            auto ccorr = [&](const std::vector<double>& a, const std::vector<double>& b) {
                const double ma = mean(a), mb = mean(b); double ab = 0.0, aa = 0.0, bb = 0.0;
                for (size_t i = 0; i < a.size(); ++i) { ab += (a[i] - ma) * (b[i] - mb); aa += (a[i] - ma) * (a[i] - ma); bb += (b[i] - mb) * (b[i] - mb); }
                return ab / std::sqrt(aa * bb); };
            CHECK(ccorr(lk, vf) > 0.5, "abs slip link: ...in step with that corner's valve");
            CHECK(hi(sc) > 0.95 && lo(sc) < 0.85, "abs slip link: the scrub eases after each dump and returns");
            CHECK(lo(sp) > 0.95, "abs slip link: spin is untouched");
            std::vector<double> lk2, sc2, sp2, vf2;
            linked(1.0, 1.0, lk2, sc2, sp2, vf2);
            CHECK(hi(lk2) <= 0.6 + 1e-9, "abs slip link: a wheel the sim shows locked stays at the edge while ABS works");
            std::vector<double> lk0, sc0, sp0, vf0;
            linked(0.0, 0.0, lk0, sc0, sp0, vf0);
            CHECK(hi(lk0) < 1e-9 && lo(sc0) > 0.95, "abs slip link off: the slip tiles are the sim's alone");
        }

        // Traction control cuts a share of the engine's firings, more with
        // throttle; nothing without it.
        {
            auto drops = [&](bool tc, double load) {
                Layer L; EffectParams e; e.ampPct = 100.0; e.freqHz = 30.0; e.routes[0] = { 0, 1.0 };
                haptics::EngineParams ep; ep.cylinders = 4; ep.maxRpm = 8000;
                L.configureFx(FxType::RpmVibe, e); L.configureEngine(ep);
                for (int i = 0; i < 4000; ++i) { L.driveEngine(1.0, 200.0, load, false, -1.0, false, false, tc); L.step(DT); }
                return static_cast<double>(L.engineTcDrops());
            };
            const double none = drops(false, 1.0), hard = drops(true, 1.0), light = drops(true, 0.1);
            CHECK(none == 0.0, "tc: no traction control, no dropped firings");
            // 2 s at 200 firings/s, half of each burst cycle cutting: 200 at
            // risk; full throttle drops 85 % of those, light throttle ~35 %.
            approx(hard, 200.0 * 0.85, 40.0, "tc: full throttle drops most of the firings inside a burst");
            CHECK(light > 20.0 && light < hard * 0.6, "tc: light throttle cuts shallower");
        }
    }

    std::printf("\nTestHaptics: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
