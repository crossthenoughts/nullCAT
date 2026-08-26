// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// CommissioningMode - rig excitation engine + identification metrics.
//
// Generates deterministic per-axis position offsets (mm, about each axis's
// centre) for commissioning tests: motion cycles (pitch/roll/heave/
// horizontal), single vibration tones, stepped frequency sweeps, and note
// sequences ("songs"). While a test runs it measures, per segment per axis:
//   - commanded vs actual amplitude at the excitation frequency (Goertzel)
//   - phase lag at the excitation frequency
//   - RMS + peak following error, torque ripple RMS
// which together give Bode points (bandwidth, resonances) at the current
// drive tune -- the scientific replacement for by-feel evaluation.
//
// Design rules:
//   - The engine knows NOTHING about drives, SOEM, or Qt. MotionController
//     feeds it dt + actuals and applies its offsets through the normal
//     ONLINE guard chain (accel clamp, velocity clamp, braking guard), so
//     every safety net stays live during a test.
//   - step()/recordSample() are RT-safe: fixed-size state, no allocation,
//     no locking on the hot path. Results publish at segment boundaries
//     via try_lock (retried next cycle if contended).
//   - All excitation is amplitude-enveloped (raised-cosine ramps) so every
//     segment starts and ends at zero offset -- no steps, ever.
//   - Amplitudes are derated per axis to fit stroke, velocity, and accel
//     budgets BEFORE the test runs; the derated flag is reported so a
//     "quieter than asked" result is never mistaken for a rig problem.
//
// Safety rails enforced HERE: amplitude/velocity/accel derating, following-
// error abort (sustained + instantaneous), smooth return-to-centre on abort.
// Rails enforced by MotionController: homed-only entry, PARKED-only entry,
// telemetry-quiet entry, guard chain on the output, fault/e-stop cancel.
// ============================================================

#include "TelemetryInput.h"   // MAX_DRIVES
#include <cstdint>
#include <mutex>

// ---- Plan (built by the UI thread, copied into the engine at start) ----

struct CommissioningSegment
{
    // kind 0 = enveloped sine (all ramped tests). kind 1 = step-and-hold:
    // the raw offset jumps to ampMm and holds -- the guard chain turns the
    // jump into the steepest SAFE profile, and the step metrics (overshoot,
    // rise, settling) are measured against that guarded command.
    uint8_t kind       = 0;
    double freqHz      = 1.0;
    double durationSec = 2.0;
    double rampSec     = 0.3;              // raised-cosine ramp in AND out (sine only)
    double ampMm[MAX_DRIVES] = {};         // signed: negative = antiphase
    char   label[24]   = {};               // "pitch", "12.5 Hz", "e1", ...
};

static constexpr int MAX_TEST_SEGMENTS = 96;

struct CommissioningPlan
{
    CommissioningSegment seg[MAX_TEST_SEGMENTS];
    int    numSegments = 0;
    double ferrAbortMm = 10.0;             // sustained following-error abort
    char   title[32]   = {};
};

// Per-axis facts the engine needs, snapshot by MotionController at start.
struct CommissioningAxisLimits
{
    bool   enabled       = false;          // axis participates in this test
    double halfStrokeMm  = 50.0;
    double maxVelMmS     = 200.0;
    double maxAccelMmS2  = 2000.0;
    double startOffsetMm = 0.0;            // currentPos - centerPos at entry
    char   unit[4]       = "mm";           // display unit ("deg" on rotary levers)
};

// ---- Axis metadata for the plan builders (UI thread only) ----
struct CommissioningAxisMeta
{
    bool    selected     = false;
    uint8_t kind         = 0;              // 0 = vertical, 1 = horizontal, 2 = belt/other
    double  halfStrokeMm = 50.0;
    int8_t  frontRear    = 0;              // +1 front, -1 rear, 0 unknown (cycle mixing)
    int8_t  leftRight    = 0;              // +1 left,  -1 right, 0 unknown
    // Signed mixing weights for the cycle movements, clamped to [-1, 1] by
    // buildCycle. With useWeights set they replace the role pair entirely:
    // pitch uses wPitch, roll wRoll, heave wHeave. Legacy role clients
    // (useWeights=false) are bit-identical to before: pitch = frontRear,
    // roll = leftRight, heave = +1. Needed because a hexapod's paired-leg
    // geometry projects pitch/roll/heave onto each leg as cosines that the
    // +/-1 corner roles cannot express.
    bool   useWeights = false;
    double wPitch = 0.0, wRoll = 0.0, wHeave = 0.0;
};

struct CommissioningCycleParams
{
    bool   enPitch = true,  enRoll = true,  enHeave = true,  enHoriz = true;
    double pitchPct = 30.0, rollPct = 30.0, heavePct = 40.0, horizPct = 30.0;
    double freqHz  = 0.2;                  // gentle full-body motion
    int    cycles  = 2;                    // sine cycles per movement
};

// ---- Results ----

struct CommissioningAxisResult
{
    bool   tested    = false;
    bool   derated   = false;              // amplitude reduced to fit budgets
    double cmdAmpMm  = 0.0;                // measured amplitude of the command
    double actAmpMm  = 0.0;                // measured amplitude of the response
    double phaseDeg  = 0.0;                // response phase relative to command (negative = lag)
    double ferrRmsMm = 0.0;
    double ferrPeakMm= 0.0;
    double trqRmsPct = 0.0;                // torque ripple about its mean
    // Sine rows: torque amplitude AT the excitation frequency (DC-removed
    // projection). trqAmpPct / (actAmpMm*(2*pi*f)^2) is the load/inertia
    // indicator -- flat across a sweep = mass-dominated, a peak = resonance.
    double trqAmpPct = 0.0;
    // Step rows: classical step metrics vs the held target.
    double overshootPct = 0.0;
    double riseMs    = 0.0;                // 10% -> 90% of target
    double settleMs  = 0.0;                // last time outside the 2% band
};

struct CommissioningSegResult
{
    char    label[24] = {};
    uint8_t kind      = 0;                 // mirrors CommissioningSegment::kind
    double  freqHz    = 0.0;
    CommissioningAxisResult axis[MAX_DRIVES];
};

struct CommissioningStatus
{
    bool   active   = false;
    bool   done     = false;               // a run finished since the last start
    bool   aborted  = false;
    char   phase[16]  = {};                // "idle","centering","running","returning","done"
    char   reason[96] = {};                // abort/refusal reason
    char   title[32]  = {};
    int    segIdx      = 0;
    int    numSegments = 0;
    double progressPct = 0.0;              // whole-test progress estimate
    int    resultCount = 0;
    CommissioningSegResult results[MAX_TEST_SEGMENTS];
};

// ============================================================

class CommissioningMode
{
public:
    // ---- Control (RT thread; MotionController::process) ----
    // Copies the plan, derates amplitudes against the limits, arms Centering.
    void start(const CommissioningPlan& plan,
               const CommissioningAxisLimits* limits, int numAxes, double dtSec);

    // Engine-driven abort: eases offsets back to zero, then Done(aborted).
    void abort(const char* reason);

    // External cancel: the axes were taken over (fault park / e-stop / user
    // park), so DON'T keep commanding -- drop straight to Done(aborted).
    void cancel(const char* reason);

    // Advance one cycle; fills outOffsetMm[numAxes] (mm about centre) for
    // enabled axes (0 for others). Returns true while output should be applied.
    bool step(double* outOffsetMm);

    // Metrics + following-error rail. Call per enabled axis AFTER the offset
    // has been through the guard chain: applied = what was actually commanded
    // (mm about centre), actual = measured position (mm about centre).
    void recordSample(int axis, double appliedOffsetMm, double actualOffsetMm,
                      double torquePct);

    bool active() const { return m_phase == Phase::Centering
                              || m_phase == Phase::Running
                              || m_phase == Phase::Returning; }
    bool axisEnabled(int i) const
    { return i >= 0 && i < MAX_DRIVES && m_limits[i].enabled; }

    // ---- Status (any thread) ----
    CommissioningStatus getStatus() const;

    // Refusal note for a start that never reached the engine (validation
    // failed on the RT side). Shows up in getStatus().reason.
    void setRefusal(const char* reason);

    // ---- Plan builders (UI thread; return segment count, -1 on error) ----
    static int buildCycle(const CommissioningCycleParams& p,
                          const CommissioningAxisMeta* meta, int numAxes,
                          CommissioningPlan& out);
    static int buildTone(double freqHz, double pct, double durationSec,
                         const CommissioningAxisMeta* meta, int numAxes,
                         CommissioningPlan& out);
    static int buildSweep(double f0Hz, double f1Hz, double stepHz,
                          double dwellSec, double pct,
                          const CommissioningAxisMeta* meta, int numAxes,
                          CommissioningPlan& out);
    // Step-and-hold on every selected non-belt axis (overshoot/rise/settle).
    static int buildStep(double pct, double holdSec,
                         const CommissioningAxisMeta* meta, int numAxes,
                         CommissioningPlan& out);
    // notes: whitespace-separated tokens "e1", "g1:2" (duration in beats),
    // "r"/"-" = rest. Note names c..b with # or b accidentals, octave digit.
    static int buildSong(const char* notes, double beatSec, double pct,
                         const CommissioningAxisMeta* meta, int numAxes,
                         CommissioningPlan& out);
    static double noteToFreqHz(const char* tok, double* beatsOut);  // 0 = rest, -1 = parse error

private:
    enum class Phase : uint8_t { Idle, Centering, Running, Returning, Done };

    void enterSegment(int idx);
    void finishSegment();                   // fold Goertzel state into results
    void publishResults();                  // try_lock swap to the read buffer
    static double envelope(double t, double dur, double ramp);

    Phase  m_phase = Phase::Idle;
    bool   m_aborted = false;
    char   m_reason[96] = {};
    double m_dt = 0.002;

    CommissioningPlan       m_plan;
    CommissioningAxisLimits m_limits[MAX_DRIVES];
    int    m_numAxes = 0;

    // Phase runtime
    double m_t = 0.0;                       // time within current phase/segment
    int    m_segIdx = 0;
    double m_centerT = 3.0;                 // centering duration
    double m_returnT = 1.0;                 // returning duration
    double m_returnFrom[MAX_DRIVES] = {};   // offsets captured at abort
    double m_lastOut[MAX_DRIVES] = {};      // last emitted offsets

    // Following-error rail. The plan-level ferrAbortMm is tuned for
    // 100mm-class linear strokes; m_ferrAbort[] is the per-axis effective
    // threshold, capped at 20% of the axis's usable half-range (floored at
    // 2.0 so noise cannot abort). A default linear axis (half-range 50)
    // keeps exactly the plan value; a 40-deg lever rails at 4 deg instead
    // of a quarter of its arc.
    double m_ferrAbort[MAX_DRIVES] = {};
    int    m_ferrOverCycles[MAX_DRIVES] = {};
    static constexpr int    FERR_SUSTAIN_CYCLES = 25;   // 50ms @ 500Hz
    static constexpr double FERR_HARD_FACTOR    = 2.0;  // instant abort at 2x

    // Per-segment metric accumulators (reset in enterSegment)
    struct AxisAcc
    {
        // Goertzel state for command + actual at the segment frequency
        double cS1 = 0, cS2 = 0, aS1 = 0, aS2 = 0;
        double coeff = 0, w = 0;
        long   n = 0;                       // samples inside the hold window
        double ferrSq = 0, ferrPeak = 0;
        double trqSum = 0, trqSqSum = 0;
        long   trqN = 0;
        bool   derated = false;
        // Torque projection at the segment frequency (direct DFT bin with a
        // rotating phasor, so the strong static torque -- gravity hold on a
        // vertical axis -- can be removed via the DC bin at finish).
        double tRe = 0, tIm = 0, oRe = 0, oIm = 0;
        double cosK = 1, sinK = 0, cw = 1, sw = 0;
        // Step metrics runtime
        double maxP = 0;                    // max progress actual/target
        double t10 = -1, t90 = -1;          // rise-time crossings
        double lastOob = 0;                 // last time outside the settle band
        double lastAct = 0;                 // final actual (settle accuracy)
    };
    AxisAcc m_acc[MAX_DRIVES];

    // Results: RT writes m_resWork, publishes into m_resShared under try_lock.
    CommissioningSegResult m_resWork[MAX_TEST_SEGMENTS];
    int  m_resCount = 0;
    bool m_publishPending = false;

    mutable std::mutex m_statusLock;        // guards m_resShared + status fields copied out
    CommissioningSegResult m_resShared[MAX_TEST_SEGMENTS];
    int  m_resSharedCount = 0;
    char m_sharedPhase[16]  = {};
    char m_sharedReason[96] = {};
    char m_sharedTitle[32]  = {};
    bool m_sharedActive = false, m_sharedDone = false, m_sharedAborted = false;
    int  m_sharedSegIdx = 0, m_sharedNumSegments = 0;
    double m_sharedProgress = 0.0;
    void publishLightStatus();              // phase/progress under try_lock (cheap fields)
};
