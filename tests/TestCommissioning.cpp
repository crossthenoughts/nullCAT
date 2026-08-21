// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestCommissioning.cpp - unit tests for the commissioning excitation
// engine + identification metrics (CommissioningMode).
//
// Proves: note parsing, plan builders (cycle mixing weights, sweep ladder,
// song sequencing), amplitude derating against velocity budgets, envelope
// continuity (no command steps, ever), centering from a park offset,
// Goertzel amplitude/phase accuracy against an analytic first-order plant,
// and the following-error abort rail with smooth return-to-centre.
// No SOEM / Qt / hardware. Run with: ctest -R Commissioning
// ============================================================
#include "CommissioningMode.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <complex>

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("  FAIL: %s  (line %d)\n", (msg), __LINE__); } \
} while (0)

static constexpr double DT = 0.002;   // 500 Hz
static constexpr double TAU = 6.28318530717958647692;

// Standard 4-corner + horizontal + belt rig for the builder tests.
static void rigMeta(CommissioningAxisMeta* m)
{
    for (int i = 0; i < MAX_DRIVES; ++i) m[i] = CommissioningAxisMeta{};
    // 0..3: vertical corners FL FR RL RR, 100mm stroke
    const int8_t fr[4] = { +1, +1, -1, -1 };
    const int8_t lr[4] = { +1, -1, +1, -1 };
    for (int i = 0; i < 4; ++i)
    {
        m[i].selected = true; m[i].kind = 0; m[i].halfStrokeMm = 50.0;
        m[i].frontRear = fr[i]; m[i].leftRight = lr[i];
    }
    // 4: horizontal (TL), 5: belt
    m[4].selected = true; m[4].kind = 1; m[4].halfStrokeMm = 40.0;
    m[5].selected = true; m[5].kind = 2; m[5].halfStrokeMm = 50.0;
}

static void defaultLimits(CommissioningAxisLimits* l, int n)
{
    for (int i = 0; i < MAX_DRIVES; ++i) l[i] = CommissioningAxisLimits{};
    for (int i = 0; i < n; ++i)
    {
        l[i].enabled = true;
        l[i].halfStrokeMm = 50.0;
        l[i].maxVelMmS = 200.0;
        l[i].maxAccelMmS2 = 2000.0;
        l[i].startOffsetMm = 0.0;
    }
}

int main()
{
    // ================= note parsing =================
    {
        double beats = 0.0;
        CHECK(std::fabs(CommissioningMode::noteToFreqHz("a4", &beats) - 440.0) < 0.01,
              "a4 = 440 Hz");
        CHECK(beats == 1.0, "default 1 beat");
        CHECK(std::fabs(CommissioningMode::noteToFreqHz("e1", &beats) - 41.203) < 0.01,
              "e1 = 41.2 Hz");
        CHECK(std::fabs(CommissioningMode::noteToFreqHz("c#2", &beats) - 69.296) < 0.01,
              "c#2 = 69.3 Hz");
        CHECK(CommissioningMode::noteToFreqHz("g1:2", &beats) > 0.0 && beats == 2.0,
              "g1:2 parses 2 beats");
        CHECK(CommissioningMode::noteToFreqHz("r", &beats) == 0.0, "r = rest");
        CHECK(CommissioningMode::noteToFreqHz("-", &beats) == 0.0, "- = rest");
        CHECK(CommissioningMode::noteToFreqHz("h3", &beats) < 0.0, "h3 invalid");
        CHECK(CommissioningMode::noteToFreqHz("e", &beats) < 0.0, "octave-less invalid");
        CHECK(CommissioningMode::noteToFreqHz("e1:0", &beats) < 0.0, "zero beats invalid");
    }

    // ================= builders =================
    CommissioningAxisMeta meta[MAX_DRIVES];
    {
        rigMeta(meta);
        CommissioningPlan plan;
        CommissioningCycleParams p;   // defaults: all movements on
        const int nseg = CommissioningMode::buildCycle(p, meta, 6, plan);
        // pitch + roll + heave + 1 horizontal solo = 4 segments
        CHECK(nseg == 4, "cycle: 4 segments (pitch/roll/heave/horiz)");
        CHECK(std::strcmp(plan.seg[0].label, "pitch") == 0, "seg0 = pitch");
        // pitch: front +, rear -, horizontal + belt untouched
        CHECK(plan.seg[0].ampMm[0] > 0 && plan.seg[0].ampMm[1] > 0, "pitch: front +");
        CHECK(plan.seg[0].ampMm[2] < 0 && plan.seg[0].ampMm[3] < 0, "pitch: rear -");
        CHECK(plan.seg[0].ampMm[4] == 0 && plan.seg[0].ampMm[5] == 0,
              "pitch: horiz + belt silent");
        CHECK(std::fabs(plan.seg[0].ampMm[0] - 0.30 * 50.0) < 1e-9,
              "pitch amp = 30% of half-stroke");
        // roll: left +, right -
        CHECK(plan.seg[1].ampMm[0] > 0 && plan.seg[1].ampMm[1] < 0, "roll: L+ R-");
        // heave: all verticals +
        CHECK(plan.seg[2].ampMm[0] > 0 && plan.seg[2].ampMm[3] > 0, "heave: all +");
        // horizontal solo
        CHECK(plan.seg[3].ampMm[4] > 0 && plan.seg[3].ampMm[0] == 0,
              "horiz solo: only axis 5");
        CHECK(std::fabs(plan.seg[3].ampMm[4] - 0.30 * 40.0) < 1e-9,
              "horiz amp = 30% of its half-stroke");

        // no roles set -> pitch/roll drop out, heave + horiz remain
        CommissioningAxisMeta m2[MAX_DRIVES];
        rigMeta(m2);
        for (int i = 0; i < 4; ++i) { m2[i].frontRear = 0; m2[i].leftRight = 0; }
        const int n2 = CommissioningMode::buildCycle(p, m2, 6, plan);
        CHECK(n2 == 2, "cycle without roles: heave + horiz only");
    }
    {
        rigMeta(meta);
        CommissioningPlan plan;
        int n = CommissioningMode::buildSweep(5.0, 15.0, 5.0, 2.0, 2.0, meta, 6, plan);
        CHECK(n == 3, "sweep 5..15 step 5 = 3 rungs");
        CHECK(std::fabs(plan.seg[1].freqHz - 10.0) < 1e-9, "rung 2 = 10 Hz");
        CHECK(std::strcmp(plan.seg[2].label, "15.0 Hz") == 0, "rung labels");
        CHECK(plan.seg[0].ampMm[5] == 0.0, "sweep: belt silent");
        CHECK(CommissioningMode::buildSweep(15, 5, 5, 2, 2, meta, 6, plan) == -1,
              "backwards sweep rejected");

        n = CommissioningMode::buildTone(25.0, 2.0, 5.0, meta, 6, plan);
        CHECK(n == 1, "tone: one segment");
        CHECK(std::fabs(plan.seg[0].ampMm[0] - 0.02 * 50.0) < 1e-9, "tone amp 2%");

        n = CommissioningMode::buildSong("e1 e1 g1:2 r c2", 0.5, 2.0, meta, 6, plan);
        CHECK(n == 5, "song: 5 segments");
        CHECK(std::fabs(plan.seg[2].durationSec - 1.0) < 1e-9, "g1:2 = 2 beats = 1s");
        CHECK(plan.seg[3].ampMm[0] == 0.0, "rest is silent");
        CHECK(std::strcmp(plan.seg[3].label, "rest") == 0, "rest label");
        CHECK(CommissioningMode::buildSong("e1 x9", 0.5, 2.0, meta, 6, plan) == -1,
              "bad note rejected");

        // nothing selected -> 0
        CommissioningAxisMeta none[MAX_DRIVES] = {};
        CHECK(CommissioningMode::buildTone(25, 2, 5, none, 6, plan) == 0,
              "no axes -> 0 segments");
    }

    // ================= derating =================
    {
        CommissioningMode eng;
        CommissioningPlan plan;
        plan.numSegments = 1;
        plan.seg[0].freqHz = 30.0;
        plan.seg[0].durationSec = 3.0;
        plan.seg[0].rampSec = 0.3;
        plan.seg[0].ampMm[0] = 10.0;         // way over the velocity budget
        std::snprintf(plan.seg[0].label, sizeof(plan.seg[0].label), "30 Hz");
        CommissioningAxisLimits lim[MAX_DRIVES];
        defaultLimits(lim, 1);
        eng.start(plan, lim, 1, DT);

        double out[MAX_DRIVES];
        double maxOut = 0.0;
        int guard = 0;
        while (eng.step(out) && ++guard < 500000)
        {
            eng.recordSample(0, out[0], out[0], 0.0);   // perfect plant
            maxOut = std::max(maxOut, std::fabs(out[0]));
        }
        // The binding budget at 30 Hz is ACCELERATION, not velocity:
        // 0.8*2000/(2*pi*30)^2 ~ 0.045mm vs 0.8*200/(2*pi*30) ~ 0.849mm.
        const double w30 = TAU * 30.0;
        const double cap = std::min(0.9 * 50.0,
                          std::min(0.8 * 200.0 / w30, 0.8 * 2000.0 / (w30 * w30)));
        CHECK(maxOut < cap * 1.02, "30 Hz amplitude derated to the binding budget");
        CHECK(maxOut > cap * 0.9,  "derated tone actually ran");
        CommissioningStatus st = eng.getStatus();
        CHECK(st.resultCount == 1, "one result row");
        CHECK(st.results[0].axis[0].derated, "derated flag set");
        CHECK(std::fabs(st.results[0].axis[0].cmdAmpMm - cap) < 0.1 * cap,
              "measured cmd amplitude = derated cap");
        CHECK(std::strcmp(st.phase, "done") == 0 && !st.aborted, "clean completion");
    }

    // ================= centering + continuity =================
    {
        CommissioningMode eng;
        CommissioningPlan plan;
        plan.numSegments = 1;
        plan.seg[0].freqHz = 1.0;
        plan.seg[0].durationSec = 2.0;
        plan.seg[0].rampSec = 0.3;
        plan.seg[0].ampMm[0] = 20.0;
        CommissioningAxisLimits lim[MAX_DRIVES];
        defaultLimits(lim, 1);
        lim[0].startOffsetMm = -48.0;        // endstop-parked axis
        eng.start(plan, lim, 1, DT);

        double out[MAX_DRIVES];
        double prev = -48.0, maxDelta = 0.0, firstOut = 1e9, lastOut = 1e9;
        int guard = 0;
        while (eng.step(out) && ++guard < 500000)
        {
            if (firstOut > 1e8) firstOut = out[0];
            maxDelta = std::max(maxDelta, std::fabs(out[0] - prev));
            prev = out[0];
            lastOut = out[0];
            eng.recordSample(0, out[0], out[0], 0.0);
        }
        CHECK(std::fabs(firstOut - (-48.0)) < 0.5, "starts at the park offset");
        CHECK(std::fabs(lastOut) < 0.05, "ends at centre");
        // Largest legitimate per-cycle move: centering peak ~pi/2 * 48/T mm/s
        // (T >= 48/25 s) ~ 39mm/s -> 0.08mm per 2ms. Sine at 1 Hz, 20mm:
        // 126mm/s -> 0.25mm. Anything above 1mm would be a step.
        CHECK(maxDelta < 1.0, "no command steps anywhere in the run");
    }

    // ================= Goertzel accuracy vs analytic plant =================
    {
        CommissioningMode eng;
        CommissioningPlan plan;
        plan.numSegments = 1;
        const double f = 10.0;
        plan.seg[0].freqHz = f;
        plan.seg[0].durationSec = 4.0;
        plan.seg[0].rampSec = 0.4;
        plan.seg[0].ampMm[0] = 2.0;
        CommissioningAxisLimits lim[MAX_DRIVES];
        defaultLimits(lim, 1);
        eng.start(plan, lim, 1, DT);

        // First-order plant y[n] = a x[n] + (1-a) y[n-1]
        const double a = 0.2;
        double y = 0.0;
        double out[MAX_DRIVES];
        int guard = 0;
        while (eng.step(out) && ++guard < 500000)
        {
            y = a * out[0] + (1.0 - a) * y;
            eng.recordSample(0, out[0], y, 0.0);
        }
        // Analytic H(e^jw) = a / (1 - (1-a) e^-jw)
        const double w = TAU * f * DT;
        const std::complex<double> H = a
            / (1.0 - (1.0 - a) * std::exp(std::complex<double>(0.0, -w)));
        const double expRatio = std::abs(H);
        const double expPhase = std::arg(H) * 180.0 / 3.14159265358979323846;

        CommissioningStatus st = eng.getStatus();
        CHECK(st.resultCount == 1, "plant run produced a result");
        const CommissioningAxisResult& ar = st.results[0].axis[0];
        const double ratio = ar.actAmpMm / ar.cmdAmpMm;
        char msg[96];
        std::snprintf(msg, sizeof(msg), "Goertzel ratio %.4f vs analytic %.4f",
                      ratio, expRatio);
        CHECK(std::fabs(ratio - expRatio) < 0.03 * expRatio, msg);
        std::snprintf(msg, sizeof(msg), "Goertzel phase %.2f vs analytic %.2f",
                      ar.phaseDeg, expPhase);
        CHECK(std::fabs(ar.phaseDeg - expPhase) < 3.0, msg);
        CHECK(ar.ferrRmsMm > 0.0, "plant lag shows as following error");
    }

    // ================= following-error abort rail =================
    {
        CommissioningMode eng;
        CommissioningPlan plan;
        plan.numSegments = 1;
        plan.seg[0].freqHz = 1.0;
        plan.seg[0].durationSec = 10.0;
        plan.seg[0].rampSec = 0.3;
        plan.seg[0].ampMm[0] = 30.0;
        plan.ferrAbortMm = 10.0;
        CommissioningAxisLimits lim[MAX_DRIVES];
        defaultLimits(lim, 1);
        eng.start(plan, lim, 1, DT);

        double out[MAX_DRIVES];
        double prev = 0.0, maxDelta = 0.0, lastOut = 1e9;
        int cycles = 0;
        while (eng.step(out) && ++cycles < 500000)
        {
            eng.recordSample(0, out[0], 0.0, 0.0);   // stuck axis: no motion
            maxDelta = std::max(maxDelta, std::fabs(out[0] - prev));
            prev = out[0];
            lastOut = out[0];
        }
        CommissioningStatus st = eng.getStatus();
        CHECK(st.aborted, "stuck axis aborts the test");
        CHECK(std::strstr(st.reason, "following error") != nullptr, "reason names ferr");
        CHECK(cycles < 5000, "abort came promptly (not after the full 10s)");
        CHECK(std::fabs(lastOut) < 0.05, "output eased back to centre after abort");
        CHECK(maxDelta < 1.0, "abort return had no command step");
    }

    // ================= multi-segment completion + rests =================
    {
        rigMeta(meta);
        CommissioningPlan plan;
        const int n = CommissioningMode::buildSong("e1 r g1", 0.4, 2.0, meta, 6, plan);
        CHECK(n == 3, "3-token song");
        CommissioningMode eng;
        CommissioningAxisLimits lim[MAX_DRIVES];
        defaultLimits(lim, 6);
        eng.start(plan, lim, 6, DT);
        double out[MAX_DRIVES];
        int guard = 0;
        while (eng.step(out) && ++guard < 500000)
            for (int i = 0; i < 6; ++i)
                if (eng.axisEnabled(i)) eng.recordSample(i, out[i], out[i], 0.0);
        CommissioningStatus st = eng.getStatus();
        CHECK(st.resultCount == 3, "all 3 segments have results");
        CHECK(!st.aborted && std::strcmp(st.phase, "done") == 0, "song completed");
        // rest row: nothing measured, no NaN
        CHECK(st.results[1].axis[0].actAmpMm == 0.0
              && st.results[1].axis[0].phaseDeg == 0.0, "rest row is clean zeros");
    }

    // ================= external cancel =================
    {
        CommissioningMode eng;
        CommissioningPlan plan;
        plan.numSegments = 1;
        plan.seg[0].freqHz = 1.0;
        plan.seg[0].durationSec = 5.0;
        plan.seg[0].ampMm[0] = 10.0;
        CommissioningAxisLimits lim[MAX_DRIVES];
        defaultLimits(lim, 1);
        eng.start(plan, lim, 1, DT);
        double out[MAX_DRIVES];
        for (int k = 0; k < 100; ++k) eng.step(out);
        eng.cancel("axes taken over");
        CHECK(!eng.active(), "cancel is immediate");
        CHECK(!eng.step(out), "no output after cancel");
        CommissioningStatus st = eng.getStatus();
        CHECK(st.aborted && std::strstr(st.reason, "taken over") != nullptr,
              "cancel reason recorded");
    }

    std::printf("TestCommissioning: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
