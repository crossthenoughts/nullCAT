// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestTrackingFilter.cpp — host-side acceptance tests (T1..T5) for the
// critically-damped ONLINE tracking filter with braking-aware velocity clamp.
// Standalone (no Qt):
//   g++ -std=c++17 -O2 -ffp-contract=off -I ../src TestTrackingFilter.cpp -o /tmp/ttf && /tmp/ttf
//
// -ffp-contract=off is REQUIRED for the bit-identity Regression test. It keeps the
// floating-point deterministic so identical math compiles to identical bits: the golden
// inline reference and CommandConditioner.stepFilter() then match to the last bit
// (maxDiff == 0). With the default (-ffp-contract=fast) the compiler may fuse a*b+c into
// an FMA in one path but not the other when the inlined applyGuard body changes (e.g. the
// side-channel windowed-accel buffer), shifting the result ~1e-12 mm and tripping the
// zero-tolerance check. The flag fixes the COMPILER non-determinism; the test keeps its
// zero tolerance, because catching one-bit perturbations before anyone argues whether
// they matter is the whole point of this tripwire.
// ============================================================
#include "CommandConditioner.h"
#include <cstdio>
#include <cmath>
#include <cstdint>
// NOTE: this GCC ignores `#pragma STDC FP_CONTRACT OFF`, so the bit-identity Regression
// test's determinism comes from the -ffp-contract=off BUILD FLAG (see header). Don't add
// the pragma back thinking it guards anything -- it doesn't here.

static const double HZ      = 2000.0;          // control loop
static const double DT      = 1.0 / HZ;
static const double WN      = 2.0 * M_PI * 30.0; // trackingWnHz = 30
static const double VMAX    = 500.0;           // mm/s
static const double AMAX    = 5000.0;          // mm/s^2
static const double CPMM    = 13107.2;         // counts/mm
static const double BRAKE_EPS = 4.0 / CPMM;    // a few counts (matches MotionController)
static const double NOFRAME   = -1.0;   // frameSec sentinel: no estimate -> absolute braking

static inline int32_t counts(double mm) { return (int32_t)std::llround(mm * CPMM); }
static int pass = 0, fail = 0;
static void check(const char* name, bool ok, const char* detail)
{
    printf("  [%s] %-42s %s\n", ok ? "PASS" : "FAIL", name, detail);
    if (ok) ++pass; else ++fail;
}
static void note(const char* name, const char* detail)
{
    printf("  [CHAR] %-42s %s\n", name, detail);
}

// ---- T1a: micro-step in the LINEAR regime (no clamp binds) -> true zeta=1 ----
static void T1_linear()
{
    const double step = 0.5 * (AMAX / (WN*WN));            // ~0.07 mm
    CommandConditioner f; f.reset(0, 0);
    double maxPos = 0, prev = 0; bool mono = true;
    for (int n = 0; n < (int)(0.1*HZ); ++n) {
        double p = f.stepFilter(step, DT, NOFRAME, WN, VMAX, AMAX, BRAKE_EPS);
        if (p > maxPos) maxPos = p;
        if (p < prev - 1e-12) mono = false;
        prev = p;
    }
    double overCounts = (maxPos - step) * CPMM;
    char d[140];
    snprintf(d, sizeof(d), "step=%.3fmm overshoot=%.4f counts mono=%s (proves zeta=1 in linear regime)",
             step, overCounts, mono?"yes":"no");
    check("T1a linear-regime micro-step: zero overshoot", overCounts <= 1.0 && mono, d);
}

// ---- T1b: step at every amplitude -> braking-aware clamp makes overshoot
//          independent of amplitude and at the discrete-time floor (~2*Amax*dt^2,
//          a few microns), saturated regime included. Tolerance 0.01mm. ----
static void T1_step(double step)
{
    CommandConditioner f; f.reset(0, 0);
    double maxPos = -1e9; int settleCyc = -1;
    const int N = (int)(0.6 * HZ);
    const double settleTol = step * 0.01;                 // 1% band
    for (int n = 0; n < N; ++n) {
        double p = f.stepFilter(step, DT, NOFRAME, WN, VMAX, AMAX, BRAKE_EPS);
        if (p > maxPos) maxPos = p;
        if (std::fabs(p - step) <= settleTol) { if (settleCyc < 0) settleCyc = n; } else settleCyc = -1;
    }
    double overMm = maxPos - step;
    bool settled = settleCyc >= 0;
    const double TOL_MM = 0.01;                            // 10 microns ~ 3x the discrete floor
    char d[200];
    snprintf(d, sizeof(d), "step=%5.1fmm overshoot=%6.1f counts (%.4fmm) settled=%s",
             step, overMm*CPMM, overMm, settled?"yes":"no");
    check("T1b step: overshoot at discrete floor (<=0.01mm)", overMm <= TOL_MM && settled, d);
}

// ---- T2: low UDP send-rate stress (50 Hz ZOH of a 2 Hz sine) -- a host left at a
//      sparse send rate, NOT a game-telemetry floor. Filtered velocity-discontinuity
//      << raw. (The Filter mode smooths it; Interpolate is the proper fix -- step 2.) ----
static void T2()
{
    CommandConditioner f; f.reset(0, 0);
    const double amp = 25.0, fs = 2.0, frame = 50.0, frameDt = 1.0/frame;
    const int N = (int)(2.0 * HZ);
    double target = 0, tAcc = 0;
    int32_t prevCnt = 0, prevStep = 0; bool haveStep = false, havePrev = false;
    int32_t rawMaxStep = 0, filtMaxVelStep = 0; int reversals = 0; int8_t prevSign = 0;
    int32_t rawPrevCnt = 0; bool rawHave = false;
    for (int n = 0; n < N; ++n) {
        double t = n * DT;
        tAcc += DT;
        if (tAcc >= frameDt) { tAcc -= frameDt; target = amp * std::sin(2*M_PI*fs*t); }  // ZOH
        if (rawHave) { int32_t rs = std::abs(counts(target) - rawPrevCnt); if (rs > rawMaxStep) rawMaxStep = rs; }
        rawPrevCnt = counts(target); rawHave = true;
        double p = f.stepFilter(target, DT, NOFRAME, WN, VMAX, AMAX, BRAKE_EPS);
        int32_t c = counts(p);
        if (havePrev) {
            int32_t step = c - prevCnt;
            if (haveStep) { int32_t vs = std::abs(step - prevStep); if (vs > filtMaxVelStep) filtMaxVelStep = vs; }
            int8_t sgn = (int8_t)((step>0)-(step<0));
            if (sgn != 0) { if (prevSign != 0 && sgn != prevSign) ++reversals; prevSign = sgn; }
            prevStep = step; haveStep = true;
        }
        prevCnt = c; havePrev = true;
    }
    char d[200];
    double pct = rawMaxStep ? 100.0*filtMaxVelStep/rawMaxStep : 0;
    snprintf(d, sizeof(d), "rawMaxStep=%d filtMaxVelStep=%d (%.3f%% of raw, <5%%) reversals=%d (peaks~8/2s)",
             rawMaxStep, filtMaxVelStep, pct, reversals);
    check("T2 staircase: smoothing + no chatter", pct < 5.0 && reversals <= 12, d);
}

// ---- T3: target ramps then stops mid-approach; no accel chatter ----
static void T3()
{
    CommandConditioner f; f.reset(0, 0);
    const int N = (int)(1.0 * HZ);
    double target = 0; int accSignChanges = 0; int8_t prevAccSign = 0;
    for (int n = 0; n < N; ++n) {
        double t = n * DT;
        if (t < 0.2) target = 100.0 * t;
        else if (t < 0.4) target = 20.0;
        else target = 20.0 - 50.0*(t-0.4);
        f.stepFilter(target, DT, NOFRAME, WN, VMAX, AMAX, BRAKE_EPS);
        int8_t s = (int8_t)((f.accel>1e-6)-(f.accel<-1e-6));
        if (s != 0) { if (prevAccSign != 0 && s != prevAccSign) ++accSignChanges; prevAccSign = s; }
    }
    char d[120]; snprintf(d, sizeof(d), "accel sign changes=%d (target has ~2-3 reversals)", accSignChanges);
    check("T3 moving target: no accel chatter", accSignChanges <= 4, d);
}

// ---- T4: staircase with +/-3ms frame-arrival jitter; no velocity spikes ----
static void T4()
{
    CommandConditioner f; f.reset(0, 0);
    const double amp = 25.0, fs = 2.0, frame = 50.0, frameDt = 1.0/frame;
    const int N = (int)(2.0 * HZ);
    double target = 0, nextFrame = 0; uint32_t rng = 12345;
    int32_t prevCnt = 0, prevStep = 0; bool haveStep=false, havePrev=false; int32_t filtMaxVelStep = 0;
    for (int n = 0; n < N; ++n) {
        double t = n * DT;
        if (t >= nextFrame) {
            target = amp * std::sin(2*M_PI*fs*t);
            rng = rng*1664525u + 1013904223u;
            double jit = ((rng >> 8) / (double)0xFFFFFF - 0.5) * 0.006;  // +/-3 ms
            nextFrame = t + frameDt + jit;
        }
        double p = f.stepFilter(target, DT, NOFRAME, WN, VMAX, AMAX, BRAKE_EPS);
        int32_t c = counts(p);
        if (havePrev) { int32_t step=c-prevCnt; if (haveStep){int32_t vs=std::abs(step-prevStep); if(vs>filtMaxVelStep) filtMaxVelStep=vs;} prevStep=step; haveStep=true; }
        prevCnt=c; havePrev=true;
    }
    int32_t bound = (int32_t)std::llround(AMAX * DT * CPMM * 1.5);
    char d[120]; snprintf(d, sizeof(d), "filtMaxVelStep=%d (bound %d = 1.5*Amax*dt)", filtMaxVelStep, bound);
    check("T4 jitter: no velocity spikes", filtMaxVelStep <= bound, d);
}

// ---- T5: held (latched) command -> bit-identical count over 10,000 cycles ----
static void T5()
{
    double latched = 37.123456;
    int32_t c0 = counts(latched); bool ident = true;
    for (int n = 0; n < 10000; ++n) if (counts(latched) != c0) { ident = false; break; }
    char d[80]; snprintf(d, sizeof(d), "latched count=%d identical over 10000 cycles", c0);
    check("T5 latch: bit-identical parked command", ident, d);
}

// ---- Glitch: a single-frame multi-mm jump must decelerate TO the jump target,
//      never past it (the residual the braking clamp closes vs the old build). ----
static void Glitch(double jump)
{
    CommandConditioner f; f.reset(0, 0);
    double maxPos = -1e9;
    for (int n = 0; n < (int)(0.6*HZ); ++n) {
        double p = f.stepFilter(jump, DT, NOFRAME, WN, VMAX, AMAX, BRAKE_EPS);
        if (p > maxPos) maxPos = p;
    }
    char d[150];
    snprintf(d, sizeof(d), "jump=%.0fmm peak overshoot=%.1f counts (~%.1f microns; was mm of momentum before)",
             jump, (maxPos - jump)*CPMM, (maxPos - jump)*1000.0);
    note("Glitch: large jump decelerates to target", d);
}

// ---- Exact integrator across the wn*dt range the knee cap allows ----
// The analytic ZOH form must be overshoot-free in the linear regime and stable at
// wn*dt = 0.39 (~loopHz/16, the old Euler ceiling), 1.0, and 2.0. We exercise each
// via the loop/knee pair that yields that wn*dt.
static void ExactStep(double loopHz, double kneeHz)
{
    const double dt = 1.0/loopHz, wn = 2.0*M_PI*kneeHz, brakeEps = 4.0/CPMM;
    CommandConditioner f; f.reset(0, 0);
    double maxAbs = 0, maxPos = -1e9; int settleCyc = -1; const double step = 10.0;
    for (int n = 0; n < (int)(0.6*loopHz); ++n) {
        double p = f.stepFilter(step, dt, NOFRAME, wn, VMAX, AMAX, brakeEps);
        if (std::fabs(p) > maxAbs) maxAbs = std::fabs(p);
        if (p > maxPos) maxPos = p;
        if (std::fabs(p - step) <= step*0.01) { if (settleCyc < 0) settleCyc = n; } else settleCyc = -1;
    }
    bool diverged = !std::isfinite(maxAbs) || maxAbs > 100.0;
    double overMm = maxPos - step;
    char d[200];
    snprintf(d, sizeof(d), "loop=%.0f knee=%.0f wn*dt=%.2f -> %s overshoot=%.3fmm settled=%s",
             loopHz, kneeHz, wn*dt, diverged?"DIVERGED":"stable", overMm, settleCyc>=0?"yes":"no");
    check("Exact: stable + overshoot-free across wn*dt", !diverged && overMm <= 0.01 && settleCyc >= 0, d);
}

// ---- Unconditional stability: unclamped at wn*dt = pi (past the old Euler limit) ----
static void StabilityDemo()
{
    const double dt = 1.0/500.0, wn = 2.0*M_PI*250.0;   // wn*dt = pi ~ 3.14
    const double BIG = 1e12;                              // disable clamps -> pure linear integrator
    CommandConditioner f; f.reset(0, 0);
    double maxAbs = 0;
    for (int n = 0; n < 2000; ++n) {
        double p = f.stepFilter(1.0, dt, NOFRAME, wn, BIG, BIG, BIG);
        if (std::fabs(p) > maxAbs) maxAbs = std::fabs(p);
    }
    bool bounded = std::isfinite(maxAbs) && maxAbs < 100.0;
    char d[160];
    snprintf(d, sizeof(d), "wn*dt=3.14 unclamped step=1mm -> maxAbs=%.3g mm (%s)",
             maxAbs, bounded?"bounded":"DIVERGED");
    check("Exact: unconditionally stable at wn*dt=pi", bounded, d);
}

// ---- Higher knee tracks tighter (less lag/attenuation) -- the whole point ----
static void ExactLag(double kneeHz)
{
    const double loopHz = 2000.0, dt = 1.0/loopHz, wn = 2.0*M_PI*kneeHz, brakeEps = 4.0/CPMM;
    const double amp = 1.0, fs = 5.0;   // gentle 1mm 5Hz cue, stays in the linear regime
    CommandConditioner f; f.reset(0, 0);
    double se = 0; int cnt = 0;
    for (int n = 0; n < (int)(2.0*loopHz); ++n) {
        double t = n*dt, tgt = amp*std::sin(2.0*M_PI*fs*t);
        double p = f.stepFilter(tgt, dt, NOFRAME, wn, VMAX, AMAX, brakeEps);
        if (t > 0.5) { se += (p-tgt)*(p-tgt); ++cnt; }
    }
    char d[160];
    snprintf(d, sizeof(d), "knee=%.0fHz: RMS error vs 5Hz cue = %.3fmm (lower = tighter, less lag)",
             kneeHz, std::sqrt(se/cnt));
    note("Filter knee vs tracking tightness", d);
}

// ---- Composite: Bypass/Interpolate pass HF content; Filter low-passes it ----
// 2 + 40 + 80 Hz on a 1000 Hz stream. HF amplitudes are kept inside the Vmax/Amax
// envelope (A*omega < Vmax, A*omega^2 < Amax) so the test isolates the CONDITIONING
// (low-pass vs not) from the hard limits. Per-component output amplitude via sin/cos
// correlation. Bypass/Interpolate must pass 40/80 Hz; Filter@30Hz must attenuate them.
static void Composite()
{
    const double frameSec=0.001; const int fc=(int)(frameSec/DT+0.5);   // 1000Hz frames
    const double A2=5.0, f2=2.0, A40=0.05, f40=40.0, A80=0.015, f80=80.0;
    struct Res { double a2,a40,a80; };
    auto run=[&](int mode)->Res{
        CommandConditioner f; f.reset(0,0);
        double tgt=0, c2=0,s2=0,c40=0,s40=0,c80=0,s80=0; int cnt=0;
        for(int n=0;n<(int)(4.0*HZ);++n){
            double t=n*DT;
            double sig=A2*std::sin(2*M_PI*f2*t)+A40*std::sin(2*M_PI*f40*t)+A80*std::sin(2*M_PI*f80*t);
            if (n%fc==0) tgt=sig;
            double p = (mode==0) ? f.stepBypass(tgt,DT,frameSec,VMAX,AMAX,BRAKE_EPS)
                     : (mode==1) ? f.stepInterpolate(tgt,DT,frameSec,VMAX,AMAX,BRAKE_EPS)
                     :             f.stepFilter(tgt,DT,frameSec,2*M_PI*30.0,VMAX,AMAX,BRAKE_EPS);
            if(t>0.5){
                c2+=p*std::cos(2*M_PI*f2*t); s2+=p*std::sin(2*M_PI*f2*t);
                c40+=p*std::cos(2*M_PI*f40*t); s40+=p*std::sin(2*M_PI*f40*t);
                c80+=p*std::cos(2*M_PI*f80*t); s80+=p*std::sin(2*M_PI*f80*t);
                ++cnt;
            }
        }
        return { 2*std::sqrt(c2*c2+s2*s2)/cnt, 2*std::sqrt(c40*c40+s40*s40)/cnt, 2*std::sqrt(c80*c80+s80*s80)/cnt };
    };
    Res b=run(0), i=run(1), fl=run(2);
    printf("    input mm: 2Hz=%.3f 40Hz=%.3f 80Hz=%.3f\n", A2,A40,A80);
    printf("      Bypass      2Hz=%.3f 40Hz=%.3f 80Hz=%.3f\n", b.a2,b.a40,b.a80);
    printf("      Interpolate 2Hz=%.3f 40Hz=%.3f 80Hz=%.3f\n", i.a2,i.a40,i.a80);
    printf("      Filter@30Hz 2Hz=%.3f 40Hz=%.3f 80Hz=%.3f\n", fl.a2,fl.a40,fl.a80);
    bool pass = b.a40>0.7*A40 && b.a80>0.6*A80          // Bypass passes HF
             && i.a40>0.7*A40 && i.a80>0.6*A80          // Interpolate passes HF
             && fl.a40<0.6*A40 && fl.a80<0.35*A80;      // Filter attenuates HF
    check("Composite: Bypass/Interpolate pass HF, Filter low-passes", pass, "see amplitudes above");
}

// ---- Regression: degraded (vt=0) path == old ABSOLUTE braking, bit-identical ----
// applyGuard now uses TARGET-VELOCITY-AWARE (relative) braking. With vt=0 (the
// degraded path: no frame-interval estimate, i.e. frameSec=NOFRAME) it must collapse
// EXACTLY to the old absolute braking clamp -- that is the safety-equivalence that
// matters. Compared against a golden transcript of the pre-refactor exact inline
// (absolute braking). The relative path (vt!=0) is the new feature, tested above.
static void Regression()
{
    // Golden reference = pre-refactor exact inline w/ ABSOLUTE braking (commit 57f70e0).
    struct Ref {
        double pos=0, vel=0, accel=0, lastValidTarget=0;
        void reset(double p,double v){pos=p;vel=v;accel=0;lastValidTarget=p;}
        double step(double target,double dt,double wn,double vmax,double amax,double brakeEps){
            if (std::isfinite(target)) lastValidTarget=target; else target=lastValidTarget;
            const double err=target-pos;
            const double p=wn*dt;
            const double e=std::exp(-p);
            double newVel = e*wn*wn*dt*err + e*(1.0-p)*vel;
            const double aeff=(newVel-vel)/dt;
            if (aeff> amax) newVel=vel+amax*dt;
            if (aeff<-amax) newVel=vel-amax*dt;
            accel=(newVel-vel)/dt;
            vel=newVel;
            if (vel> vmax) vel= vmax;
            if (vel<-vmax) vel=-vmax;
            if (std::fabs(err)>brakeEps){
                const double vbrake=std::sqrt(2.0*amax*std::fabs(err));
                if (err>0.0){ if(vel> vbrake) vel= vbrake; }
                else        { if(vel<-vbrake) vel=-vbrake; }
            }
            pos+=vel*dt;
            return pos;
        }
    };
    Ref r; r.reset(0,0);
    CommandConditioner f; f.reset(0,0);
    int mism=0; double maxDiff=0;
    for (int n=0;n<20000;++n){
        const double t=n*DT;
        double target;
        if (n%1500==750)            target = std::nan("");                       // NaN reject path
        else if (n%800 < 5)         target = (n%1600 < 800 ? 90.0 : -90.0);      // large jumps -> saturate amax/vmax/braking
        else target = 30.0*std::sin(2*M_PI*2.0*t) + 8.0*std::sin(2*M_PI*37.0*t)  // composite -> linear + brakeEps band
                    + 2.0*std::sin(2*M_PI*91.0*t);
        const double pr = r.step(target, DT, WN, VMAX, AMAX, BRAKE_EPS);
        const double pf = f.stepFilter(target, DT, NOFRAME, WN, VMAX, AMAX, BRAKE_EPS);
        if (pr != pf) { ++mism; double dd=std::fabs(pr-pf); if(dd>maxDiff)maxDiff=dd; }
    }
    char d[160];
    snprintf(d,sizeof(d),"20000 cycles, mismatches=%d maxDiff=%.3g (must be exactly 0)", mism, maxDiff);
    check("Regression: step() bit-identical to pre-refactor inline", mism==0, d);
}

// ---- Relative braking: tracking a moving target is not penalized (no v^2/2a lag) ----
static void RelBraking(double v)
{
    const double frameSec = 0.001; const int fc = (int)(frameSec/DT + 0.5);   // 1000Hz frames (ZOH)
    CommandConditioner rel; rel.reset(0,0);
    CommandConditioner ab;  ab.reset(0,0);
    double tgt=0, lagRel=0, lagAbs=0;
    for (int n=0;n<40000;++n){
        double t=n*DT;
        if (n%fc==0) tgt = v*t;
        double pr = rel.stepBypass(tgt, DT, frameSec, VMAX, AMAX, BRAKE_EPS);  // relative (vt~=v)
        double pa = ab .stepBypass(tgt, DT, NOFRAME,  VMAX, AMAX, BRAKE_EPS);  // absolute (vt=0)
        lagRel = tgt - pr; lagAbs = tgt - pa;
    }
    char d[200];
    snprintf(d,sizeof(d),"v=%5.0f mm/s | relative lag=%.3fmm (%.1fms) vs absolute lag=%.3fmm (%.1fms)",
             v, lagRel, lagRel/v*1000, lagAbs, lagAbs/v*1000);
    check("Relative braking: no v^2/2a tracking lag", lagRel < lagAbs*0.25 && lagRel < 0.3, d);
}

// ---- Relative braking: a SUDDEN target stop overshoots only by the MOMENTUM FLOOR ----
// Physics: zero-lag tracking means the command carries velocity v, which needs v^2/2amax
// to dissipate. So an INSTANT stop (a glitch/freeze -- smooth telemetry decelerates
// gradually and never triggers this) overshoots by ~v^2/2amax, bounded (stroke-clamped,
// staleness-recovered) -- NOT runaway. This is the inherent price of zero-lag tracking;
// absolute braking trades it for always-on lag. We assert the bound holds.
static void OvershootStop(double v)
{
    const double frameSec=0.005; const int fc=(int)(frameSec/DT+0.5);  // 200Hz
    CommandConditioner f; f.reset(0,0);
    double tgt=0, stopAt=0, maxOver=0; bool stopped=false;
    for (int n=0;n<8000;++n){
        double t=n*DT;
        if (t>=0.3 && !stopped){ stopAt=v*0.3; stopped=true; }
        if (n%fc==0) tgt = stopped ? stopAt : v*t;          // ramp, then an instantly-held value
        double p=f.stepBypass(tgt, DT, frameSec, VMAX, AMAX, BRAKE_EPS);
        if (stopped){ double o=p-stopAt; if(o>maxOver)maxOver=o; }
    }
    const double floor = v*v/(2*AMAX);
    char d[180]; snprintf(d,sizeof(d),"v=%.0f instant stop: overshoot=%.3fmm (momentum floor v^2/2a=%.3fmm) -- bounded",
                          v, maxOver, floor);
    check("Relative braking: instant-stop overshoot bounded to momentum floor", maxOver <= floor*1.3 + 0.5, d);
}

// ---- vt accuracy DURING binds: a bind is only a problem if vt was inaccurate ----
// A relative-braking bind is not inherently bad -- if vt is tracking the real target
// velocity well and the clamp engages on a genuine fast transient, that is the clamp
// doing its job. The session-end diagnostic must therefore report the peak vt-vs-actual-
// target-velocity error DURING binds, so the two cases can be told apart. We feed two
// sinusoidal streams that both drive the clamp into binding: a low-frequency one whose
// per-frame velocity the held-vt prediction tracks accurately, and a high-frequency one
// whose per-frame velocity swings so fast that the ZOH prediction is badly wrong. The
// metric must read small in the first and large in the second.
static void VtAccuracyDuringBinds()
{
    const double frameSec = 0.01; const int fc = (int)(frameSec/DT + 0.5);   // 100Hz frames
    const double AMP = 30.0;
    auto run = [&](double fHz){
        CommandConditioner c; c.reset(0,0);
        double tgt = 0.0;
        for (int n=0; n<20000; ++n){
            double t = n*DT;
            if (n%fc==0) tgt = AMP * std::sin(2.0*M_PI*fHz*t);
            c.stepBypass(tgt, DT, frameSec, VMAX, AMAX, BRAKE_EPS);
        }
        return c.getGuardStats();
    };
    auto lo = run(1.0);    // smooth: per-frame velocity changes slowly -> vt accurate
    auto hi = run(15.0);   // volatile: huge per-frame velocity swings -> vt inaccurate
    char d[256];
    snprintf(d,sizeof(d),"smooth 1Hz: %.1f%% binds, peak vt err=%.1f mm/s | volatile 15Hz: %.1f%% binds, "
             "peak vt err=%.1f mm/s (vt=%.0f at that bind)",
             lo.bindPct, lo.bindVtErrPeakMms, hi.bindPct, hi.bindVtErrPeakMms, hi.bindVtAtErr);
    // Both must actually bind; the smooth stream's binds carry a small vt error (clamp
    // working on a real transient); the volatile stream's binds expose a large vt error.
    bool ok = lo.bindPct > 0.0 && hi.bindPct > 0.0
              && lo.bindVtErrPeakMms < 40.0
              && hi.bindVtErrPeakMms > 3.0 * lo.bindVtErrPeakMms
              && hi.bindVtErrPeakMms > 100.0;
    check("vt accuracy: small err on smooth binds, large on volatile", ok, d);
}

// ---- Windowed accel reads the REAL (macroscopic) accel, not the per-cycle Amax peg ----
// A low-rate frame stream into a high loop rate makes the per-cycle commanded accel peg at
// exactly Amax on every frame-boundary velocity step -- so the per-cycle metric reads ~Amax
// at EVERY Amax and is useless for headroom. The windowed metric dilutes those single-cycle
// corners over ~10 ms and must report the actual motion's acceleration, well below a generous
// Amax. Warm up then resetGuardStats() (mirrors a soft reset) so the start-from-rest
// transient isn't latched, then measure steady state.
static void WindowedAccel()
{
    const double frameSec = 0.01; const int fc = (int)(frameSec/DT + 0.5);   // 100 Hz frames into 2 kHz loop
    const double AMX = 50000.0;                                  // generous -- far above the real motion accel
    const double fHz = 2.0, AMP = 30.0;
    const double realAccel = AMP*(2*M_PI*fHz)*(2*M_PI*fHz);      // sine peak accel A*w^2 ~ 4738 mm/s^2
    CommandConditioner c; c.reset(0,0);
    double tgt=0.0;
    for (int n=0;n<4000;++n){ const double t=n*DT; if(n%fc==0) tgt=AMP*std::sin(2*M_PI*fHz*t);
        c.stepInterpolate(tgt,DT,frameSec,VMAX,AMX,BRAKE_EPS); }
    c.resetGuardStats();                                         // drop the start-from-rest transient
    for (int n=4000;n<24000;++n){ const double t=n*DT; if(n%fc==0) tgt=AMP*std::sin(2*M_PI*fHz*t);
        c.stepInterpolate(tgt,DT,frameSec,VMAX,AMX,BRAKE_EPS); }
    auto g=c.getGuardStats();
    char d[220];
    snprintf(d,sizeof(d),"per-cycle peak=%.0f (pegs ~Amax %.0f), windowed peak=%.0f (real A*w^2=%.0f), win/Amax=%.2f",
             g.accelCmdPeakMms2, AMX, g.accelWinPeakMms2, realAccel, g.accelWinPeakMms2/AMX);
    const bool ok = g.accelCmdPeakMms2 > 0.9*AMX                 // per-cycle pegs at Amax (the broken metric)
                 && g.accelWinPeakMms2 < 0.4*AMX                 // windowed shows real headroom (NOT pegged)
                 && g.accelWinPeakMms2 > 0.4*realAccel
                 && g.accelWinPeakMms2 < 3.0*realAccel;          // windowed ~ the actual motion accel
    check("Windowed accel: macroscopic motion accel, not the per-cycle Amax peg", ok, d);
}

// ---- Bypass: passthrough on a clean high-rate stream (guard does NOT rate-limit) ----
static void BypassClean()
{
    const double frameSec=0.001; const int fc=(int)(frameSec/DT+0.5);  // 1000Hz
    CommandConditioner f; f.reset(0,0);
    double tgt=0, maxLag=0;
    for(int n=0;n<(int)(2.0*HZ);++n){
        double t=n*DT;
        if (n%fc==0) tgt = 30.0*std::sin(2*M_PI*2.0*t);   // peak v ~377mm/s
        double p=f.stepBypass(tgt, DT, frameSec, VMAX, AMAX, BRAKE_EPS);
        if (t > 0.3) { double lag=std::fabs(tgt-p); if(lag>maxLag)maxLag=lag; }  // skip startup catch-up
    }
    char d[160]; snprintf(d,sizeof(d),"1000Hz clean stream (peak v~377mm/s), steady state: max|cmd-target|=%.3fmm", maxLag);
    check("Bypass: passthrough on clean stream (no silent rate-limit)", maxLag < 0.5, d);
}

// ---- Interpolate: low UDP send-rate (100Hz) stream -> smooth, no staircase ----
static void InterpSmooth()
{
    const double frameSec=0.01; const int fc=(int)(frameSec/DT+0.5);  // 100Hz
    CommandConditioner f; f.reset(0,0);
    double tgt=0; int32_t prevCnt=0,prevStep=0; bool hp=false,hs=false; int32_t maxVelStep=0;
    for(int n=0;n<(int)(2.0*HZ);++n){
        double t=n*DT;
        if (n%fc==0) tgt = 25.0*std::sin(2*M_PI*2.0*t);
        double p=f.stepInterpolate(tgt, DT, frameSec, VMAX, AMAX, BRAKE_EPS);
        int32_t c=counts(p);
        if(hp){int32_t s=c-prevCnt; if(hs){int32_t vs=std::abs(s-prevStep); if(vs>maxVelStep)maxVelStep=vs;} prevStep=s; hs=true;}
        prevCnt=c; hp=true;
    }
    int32_t bound=(int)(AMAX*DT*CPMM*1.5);
    char d[160]; snprintf(d,sizeof(d),"100Hz stream: filtMaxVelStep=%d (bound %d) -- gaps filled, no staircase", maxVelStep, bound);
    check("Interpolate: low-rate stream smooths cleanly", maxVelStep <= bound, d);
}

// ---- Interpolate: dropped + early frames produce no velocity spike ----
static void InterpJitter()
{
    const double frameSec=0.01; const int fc=(int)(frameSec/DT+0.5);
    CommandConditioner f; f.reset(0,0);
    double tgt=0; int32_t prevCnt=0,prevStep=0; bool hp=false,hs=false; int32_t maxVelStep=0;
    for(int n=0;n<(int)(3.0*HZ);++n){
        double t=n*DT;
        bool deliver=(n%fc==0);
        if (n%(fc*5)==0)      deliver=false;   // drop every 5th frame
        if (n%(fc*7)==(fc/2)) deliver=true;    // an early frame (mid-fill)
        if (deliver) tgt = 20.0*std::sin(2*M_PI*1.5*t);
        double p=f.stepInterpolate(tgt, DT, frameSec, VMAX, AMAX, BRAKE_EPS);
        int32_t c=counts(p);
        if(hp){int32_t s=c-prevCnt; if(hs){int32_t vs=std::abs(s-prevStep); if(vs>maxVelStep)maxVelStep=vs;} prevStep=s; hs=true;}
        prevCnt=c; hp=true;
    }
    int32_t bound=(int)(AMAX*DT*CPMM*2.0);
    char d[160]; snprintf(d,sizeof(d),"dropped+early frames: maxVelStep=%d (bound %d)", maxVelStep, bound);
    check("Interpolate: dropped/early frames -> no velocity spike", maxVelStep <= bound, d);
}

// ---- Interpolate: startup (no interval estimate) degrades to Bypass, bit-identical ----
static void InterpStartup()
{
    CommandConditioner a; a.reset(0,0);
    CommandConditioner b; b.reset(0,0);
    bool ident=true;
    for(int n=0;n<2000;++n){
        double t=n*DT, tgt=15.0*std::sin(2*M_PI*3.0*t) + (n%500<5?60.0:0.0);
        double pi=a.stepInterpolate(tgt, DT, NOFRAME, VMAX, AMAX, BRAKE_EPS);
        double pb=b.stepBypass(tgt, DT, NOFRAME, VMAX, AMAX, BRAKE_EPS);
        if (pi!=pb) ident=false;
    }
    check("Interpolate: startup (no interval) == Bypass (bit-identical)", ident, "frameSec<=dt -> Bypass fallback");
}

int main()
{
    printf("CommandConditioner acceptance (wn=%.1f rad/s = 30Hz, dt=%.4gs, Vmax=%.0f Amax=%.0f)\n",
           WN, DT, VMAX, AMAX);
    printf("--- refactor neutrality (must be bit-identical) ---\n");
    Regression();
    T1_linear();
    printf("--- T1b: braking-aware clamp -> overshoot-free at every amplitude (saturated incl.) ---\n");
    T1_step(2.0); T1_step(5.0); T1_step(50.0); T1_step(100.0);
    printf("--- glitch characterization: momentum past a multi-mm jump is eliminated ---\n");
    Glitch(10.0); Glitch(50.0);
    printf("--- T2..T5: realistic continuous-telemetry tracking (the actual ONLINE input) ---\n");
    T2(); T3(); T4(); T5();
    printf("--- Exact integrator across wn*dt = 0.39 / 1.0 / 2.0 (old Euler ceiling and beyond) ---\n");
    ExactStep(2000.0, 124.0);   // wn*dt = 0.39 (~loopHz/16, old Euler cap)
    ExactStep(2000.0, 318.0);   // wn*dt = 1.0
    ExactStep(500.0, 159.0);    // wn*dt = 2.0 (PC-class loop, well past Euler)
    StabilityDemo();            // unclamped at wn*dt = pi: bounded (Euler would diverge)
    printf("    (lower RMS = tighter tracking / less lag -- why a higher knee surfaces detail)\n");
    ExactLag(30.0); ExactLag(80.0); ExactLag(125.0);
    printf("--- target-velocity-aware (relative) braking: low-latency + bounded overshoot ---\n");
    RelBraking(100.0); RelBraking(200.0); RelBraking(300.0);
    OvershootStop(200.0); OvershootStop(300.0);
    VtAccuracyDuringBinds();
    WindowedAccel();
    BypassClean();
    printf("--- Interpolate mode (send-rate handler) ---\n");
    InterpSmooth(); InterpJitter(); InterpStartup();
    printf("--- 3-mode composite: HF passthrough vs low-pass ---\n");
    Composite();
    printf("\n%d passed, %d failed\n", pass, fail);
    return fail ? 1 : 0;
}
