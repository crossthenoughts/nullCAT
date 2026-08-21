// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// CommissioningMode.cpp - excitation engine + identification metrics.
// See CommissioningMode.h for the design contract. No SOEM, no Qt, no
// logging, no allocation on the RT path.
// ============================================================

#include "CommissioningMode.h"
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cctype>
#include <algorithm>

namespace
{
constexpr double PI = 3.14159265358979323846;

double clampd(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}
} // namespace

// ============================================================
// Envelope: raised-cosine ramp in/out, unity hold. Guarantees the offset
// is exactly 0 at t<=0 and t>=dur -- segments can never step.
// ============================================================
double CommissioningMode::envelope(double t, double dur, double ramp)
{
    if (t <= 0.0 || t >= dur) return 0.0;
    if (2.0 * ramp > dur) ramp = dur / 2.0;
    if (t < ramp)        return 0.5 * (1.0 - std::cos(PI * t / ramp));
    if (t > dur - ramp)  return 0.5 * (1.0 - std::cos(PI * (dur - t) / ramp));
    return 1.0;
}

// ============================================================
// Control
// ============================================================

void CommissioningMode::start(const CommissioningPlan& plan,
                              const CommissioningAxisLimits* limits,
                              int numAxes, double dtSec)
{
    m_plan    = plan;
    m_numAxes = std::min(numAxes, MAX_DRIVES);
    m_dt      = (dtSec > 0.0) ? dtSec : 0.002;
    for (int i = 0; i < MAX_DRIVES; ++i)
        m_limits[i] = (i < m_numAxes) ? limits[i] : CommissioningAxisLimits{};

    m_plan.ferrAbortMm = clampd(m_plan.ferrAbortMm, 2.0, 25.0);

    // ---- Amplitude derating: fit every segment inside the stroke,
    // velocity, and acceleration budgets of each axis. sin excitation:
    // vel_pk = A*2*pi*f, acc_pk = A*(2*pi*f)^2.
    // The derated flag is per (segment, axis) and lands in the results.
    for (int s = 0; s < m_plan.numSegments; ++s)
    {
        CommissioningSegment& sg = m_plan.seg[s];
        const double w = 2.0 * PI * std::max(0.01, sg.freqHz);
        for (int i = 0; i < m_numAxes; ++i)
        {
            if (!m_limits[i].enabled) { sg.ampMm[i] = 0.0; continue; }
            double cap = 0.9 * m_limits[i].halfStrokeMm;
            cap = std::min(cap, 0.8 * m_limits[i].maxVelMmS / w);
            cap = std::min(cap, 0.8 * m_limits[i].maxAccelMmS2 / (w * w));
            // Clamp ONTO the cap; enterSegment() recomputes the same cap and
            // flags amplitudes sitting on it as derated for the results.
            if (std::fabs(sg.ampMm[i]) > cap)
                sg.ampMm[i] = (sg.ampMm[i] < 0.0 ? -cap : cap);
        }
    }

    // Centering: cosine-ease every enabled axis from its park offset to 0.
    double maxDist = 0.0;
    for (int i = 0; i < m_numAxes; ++i)
        if (m_limits[i].enabled)
            maxDist = std::max(maxDist, std::fabs(m_limits[i].startOffsetMm));
    m_centerT = clampd(maxDist / 25.0, 1.0, 8.0);   // ~25mm/s mean approach

    m_t = 0.0;
    m_segIdx = 0;
    m_resCount = 0;
    m_publishPending = false;
    m_aborted = false;
    m_reason[0] = '\0';
    std::memset(m_lastOut, 0, sizeof(m_lastOut));
    std::memset(m_returnFrom, 0, sizeof(m_returnFrom));
    std::memset(m_ferrOverCycles, 0, sizeof(m_ferrOverCycles));
    m_phase = Phase::Centering;

    {
        // start() runs on the RT thread but only at test entry -- a plain
        // lock here is fine (no cyclic contention exists yet).
        std::lock_guard<std::mutex> lk(m_statusLock);
        m_resSharedCount = 0;
        std::strncpy(m_sharedTitle, m_plan.title, sizeof(m_sharedTitle) - 1);
        m_sharedTitle[sizeof(m_sharedTitle) - 1] = '\0';
        m_sharedReason[0] = '\0';
        m_sharedDone = false;
        m_sharedAborted = false;
    }
}

void CommissioningMode::abort(const char* reason)
{
    if (m_phase != Phase::Centering && m_phase != Phase::Running) return;
    m_aborted = true;
    std::snprintf(m_reason, sizeof(m_reason), "%s", reason ? reason : "aborted");
    std::memcpy(m_returnFrom, m_lastOut, sizeof(m_returnFrom));
    double maxAbs = 0.0;
    for (int i = 0; i < m_numAxes; ++i)
        maxAbs = std::max(maxAbs, std::fabs(m_returnFrom[i]));
    m_returnT = clampd(maxAbs / 20.0, 0.3, 3.0);
    m_t = 0.0;
    m_phase = Phase::Returning;
}

void CommissioningMode::cancel(const char* reason)
{
    if (m_phase == Phase::Idle || m_phase == Phase::Done) return;
    m_aborted = true;
    std::snprintf(m_reason, sizeof(m_reason), "%s", reason ? reason : "cancelled");
    m_phase = Phase::Done;
    publishResults();
    publishLightStatus();
}

void CommissioningMode::setRefusal(const char* reason)
{
    std::lock_guard<std::mutex> lk(m_statusLock);
    std::snprintf(m_sharedReason, sizeof(m_sharedReason), "%s",
                  reason ? reason : "refused");
}

// ============================================================
// Per-cycle step
// ============================================================

bool CommissioningMode::step(double* outOffsetMm)
{
    for (int i = 0; i < m_numAxes; ++i) outOffsetMm[i] = 0.0;

    switch (m_phase)
    {
    case Phase::Idle:
    case Phase::Done:
        return false;

    case Phase::Centering:
    {
        m_t += m_dt;
        const double u = clampd(m_t / m_centerT, 0.0, 1.0);
        const double k = 0.5 * (1.0 + std::cos(PI * u));   // 1 -> 0
        for (int i = 0; i < m_numAxes; ++i)
            if (m_limits[i].enabled)
                outOffsetMm[i] = m_limits[i].startOffsetMm * k;
        if (m_t >= m_centerT)
        {
            enterSegment(0);
        }
        break;
    }

    case Phase::Running:
    {
        if (m_segIdx >= m_plan.numSegments)
        {
            // No segments (defensive) -- go home.
            std::memcpy(m_returnFrom, m_lastOut, sizeof(m_returnFrom));
            m_returnT = 0.3;
            m_t = 0.0;
            m_phase = Phase::Returning;
            break;
        }
        const CommissioningSegment& sg = m_plan.seg[m_segIdx];
        m_t += m_dt;
        const double env = envelope(m_t, sg.durationSec, sg.rampSec);
        const double s   = std::sin(2.0 * PI * sg.freqHz * m_t);
        for (int i = 0; i < m_numAxes; ++i)
            if (m_limits[i].enabled)
                outOffsetMm[i] = sg.ampMm[i] * env * s;

        if (m_t >= sg.durationSec)
        {
            finishSegment();
            if (m_segIdx + 1 < m_plan.numSegments)
            {
                enterSegment(m_segIdx + 1);
            }
            else
            {
                // Normal completion: offsets are already 0 (envelope).
                std::memset(m_returnFrom, 0, sizeof(m_returnFrom));
                m_returnT = 0.3;
                m_t = 0.0;
                m_phase = Phase::Returning;
            }
        }
        break;
    }

    case Phase::Returning:
    {
        m_t += m_dt;
        const double u = clampd(m_t / m_returnT, 0.0, 1.0);
        const double k = 0.5 * (1.0 + std::cos(PI * u));   // 1 -> 0
        for (int i = 0; i < m_numAxes; ++i)
            if (m_limits[i].enabled)
                outOffsetMm[i] = m_returnFrom[i] * k;
        if (m_t >= m_returnT)
        {
            m_phase = Phase::Done;
            for (int i = 0; i < m_numAxes; ++i) outOffsetMm[i] = 0.0;
        }
        break;
    }
    }

    std::memcpy(m_lastOut, outOffsetMm, sizeof(double) * (size_t)m_numAxes);
    publishResults();
    publishLightStatus();
    return m_phase == Phase::Centering || m_phase == Phase::Running
        || m_phase == Phase::Returning;
}

// ============================================================
// Metrics + following-error rail
// ============================================================

void CommissioningMode::recordSample(int axis, double appliedOffsetMm,
                                     double actualOffsetMm, double torquePct)
{
    if (axis < 0 || axis >= m_numAxes || !m_limits[axis].enabled) return;
    if (m_phase == Phase::Idle || m_phase == Phase::Done) return;

    // ---- Following-error abort rail (active in every moving phase) ----
    const double ferr = std::fabs(appliedOffsetMm - actualOffsetMm);
    if (ferr > m_plan.ferrAbortMm * FERR_HARD_FACTOR)
    {
        char why[96];
        std::snprintf(why, sizeof(why),
            "axis %d following error %.1fmm (hard limit %.1fmm)",
            axis + 1, ferr, m_plan.ferrAbortMm * FERR_HARD_FACTOR);
        abort(why);
        return;
    }
    if (ferr > m_plan.ferrAbortMm)
    {
        if (++m_ferrOverCycles[axis] >= FERR_SUSTAIN_CYCLES)
        {
            char why[96];
            std::snprintf(why, sizeof(why),
                "axis %d following error %.1fmm sustained (limit %.1fmm)",
                axis + 1, ferr, m_plan.ferrAbortMm);
            abort(why);
            return;
        }
    }
    else
    {
        m_ferrOverCycles[axis] = 0;
    }

    // ---- Segment metrics (hold window only) ----
    if (m_phase != Phase::Running || m_segIdx >= m_plan.numSegments) return;
    const CommissioningSegment& sg = m_plan.seg[m_segIdx];
    double ramp = sg.rampSec;
    if (2.0 * ramp > sg.durationSec) ramp = sg.durationSec / 2.0;
    if (m_t < ramp || m_t > sg.durationSec - ramp) return;

    AxisAcc& a = m_acc[axis];
    // Goertzel recurrence for both channels
    {
        double s = appliedOffsetMm + a.coeff * a.cS1 - a.cS2;
        a.cS2 = a.cS1; a.cS1 = s;
        s = actualOffsetMm + a.coeff * a.aS1 - a.aS2;
        a.aS2 = a.aS1; a.aS1 = s;
        a.n++;
    }
    a.ferrSq  += ferr * ferr;
    a.ferrPeak = std::max(a.ferrPeak, ferr);
    a.trqSum  += torquePct;
    a.trqSqSum+= torquePct * torquePct;
    a.trqN++;
}

// ============================================================
// Segment bookkeeping
// ============================================================

void CommissioningMode::enterSegment(int idx)
{
    m_segIdx = idx;
    m_t = 0.0;
    m_phase = Phase::Running;
    const CommissioningSegment& sg = m_plan.seg[idx];
    const double w = 2.0 * PI * sg.freqHz * m_dt;   // rad/sample
    for (int i = 0; i < MAX_DRIVES; ++i)
    {
        m_acc[i] = AxisAcc{};
        m_acc[i].w     = w;
        m_acc[i].coeff = 2.0 * std::cos(w);
        // Derated flag: recompute the cap the same way start() did and mark
        // axes whose amplitude sits ON the cap (start() clamped them there).
        if (i < m_numAxes && m_limits[i].enabled && sg.ampMm[i] != 0.0)
        {
            const double wf = 2.0 * PI * std::max(0.01, sg.freqHz);
            double cap = 0.9 * m_limits[i].halfStrokeMm;
            cap = std::min(cap, 0.8 * m_limits[i].maxVelMmS / wf);
            cap = std::min(cap, 0.8 * m_limits[i].maxAccelMmS2 / (wf * wf));
            m_acc[i].derated = std::fabs(std::fabs(sg.ampMm[i]) - cap) < 1e-9;
        }
    }
}

void CommissioningMode::finishSegment()
{
    if (m_resCount >= MAX_TEST_SEGMENTS) return;
    const CommissioningSegment& sg = m_plan.seg[m_segIdx];
    CommissioningSegResult& r = m_resWork[m_resCount];
    std::memcpy(r.label, sg.label, sizeof(r.label));
    r.freqHz = sg.freqHz;
    for (int i = 0; i < MAX_DRIVES; ++i)
    {
        CommissioningAxisResult& ar = r.axis[i];
        ar = CommissioningAxisResult{};
        if (i >= m_numAxes || !m_limits[i].enabled) continue;
        ar.tested  = (sg.ampMm[i] != 0.0);
        ar.derated = m_acc[i].derated;
        const AxisAcc& a = m_acc[i];
        if (a.n > 8)
        {
            const double cw = std::cos(a.w), sw = std::sin(a.w);
            const double cRe = a.cS1 - a.cS2 * cw, cIm = a.cS2 * sw;
            const double aRe = a.aS1 - a.aS2 * cw, aIm = a.aS2 * sw;
            ar.cmdAmpMm = 2.0 / (double)a.n * std::sqrt(cRe * cRe + cIm * cIm);
            ar.actAmpMm = 2.0 / (double)a.n * std::sqrt(aRe * aRe + aIm * aIm);
            if (ar.cmdAmpMm > 1e-6)
            {
                double ph = (std::atan2(aIm, aRe) - std::atan2(cIm, cRe))
                            * 180.0 / PI;
                while (ph > 180.0)  ph -= 360.0;
                while (ph < -180.0) ph += 360.0;
                ar.phaseDeg = ph;
            }
            ar.ferrRmsMm  = std::sqrt(a.ferrSq / (double)a.n);
            ar.ferrPeakMm = a.ferrPeak;
        }
        if (a.trqN > 0)
        {
            const double mean = a.trqSum / (double)a.trqN;
            const double ms   = a.trqSqSum / (double)a.trqN - mean * mean;
            ar.trqRmsPct = ms > 0.0 ? std::sqrt(ms) : 0.0;
        }
    }
    m_resCount++;
    m_publishPending = true;
}

// ============================================================
// Publication (RT -> readers). try_lock: if the web thread is mid-copy we
// simply retry next cycle -- the RT loop never blocks.
// ============================================================

void CommissioningMode::publishResults()
{
    if (!m_publishPending) return;
    std::unique_lock<std::mutex> lk(m_statusLock, std::try_to_lock);
    if (!lk.owns_lock()) return;
    for (int s = m_resSharedCount; s < m_resCount; ++s)
        m_resShared[s] = m_resWork[s];
    m_resSharedCount = m_resCount;
    m_publishPending = false;
}

void CommissioningMode::publishLightStatus()
{
    std::unique_lock<std::mutex> lk(m_statusLock, std::try_to_lock);
    if (!lk.owns_lock()) return;
    const char* ph = "idle";
    switch (m_phase)
    {
    case Phase::Centering: ph = "centering"; break;
    case Phase::Running:   ph = "running";   break;
    case Phase::Returning: ph = "returning"; break;
    case Phase::Done:      ph = "done";      break;
    case Phase::Idle:      ph = "idle";      break;
    }
    std::snprintf(m_sharedPhase, sizeof(m_sharedPhase), "%s", ph);
    m_sharedActive  = active();
    m_sharedDone    = (m_phase == Phase::Done);
    m_sharedAborted = m_aborted;
    std::snprintf(m_sharedReason, sizeof(m_sharedReason), "%s", m_reason);
    m_sharedSegIdx = m_segIdx;
    m_sharedNumSegments = m_plan.numSegments;
    double prog = 0.0;
    if (m_plan.numSegments > 0)
    {
        if (m_phase == Phase::Running)
        {
            const double segFrac = (m_plan.seg[m_segIdx].durationSec > 0.0)
                ? clampd(m_t / m_plan.seg[m_segIdx].durationSec, 0.0, 1.0) : 0.0;
            prog = 100.0 * (m_segIdx + segFrac) / m_plan.numSegments;
        }
        else if (m_phase == Phase::Returning || m_phase == Phase::Done)
        {
            prog = 100.0;
        }
    }
    m_sharedProgress = prog;
}

CommissioningStatus CommissioningMode::getStatus() const
{
    CommissioningStatus st;
    std::lock_guard<std::mutex> lk(m_statusLock);
    st.active  = m_sharedActive;
    st.done    = m_sharedDone;
    st.aborted = m_sharedAborted;
    std::memcpy(st.phase,  m_sharedPhase,  sizeof(st.phase));
    std::memcpy(st.reason, m_sharedReason, sizeof(st.reason));
    std::memcpy(st.title,  m_sharedTitle,  sizeof(st.title));
    st.segIdx      = m_sharedSegIdx;
    st.numSegments = m_sharedNumSegments;
    st.progressPct = m_sharedProgress;
    st.resultCount = m_resSharedCount;
    for (int s = 0; s < m_resSharedCount; ++s)
        st.results[s] = m_resShared[s];
    return st;
}

// ============================================================
// Plan builders (UI thread)
// ============================================================

namespace
{
void setLabel(CommissioningSegment& sg, const char* text)
{
    std::snprintf(sg.label, sizeof(sg.label), "%s", text);
}
} // namespace

int CommissioningMode::buildCycle(const CommissioningCycleParams& p,
                                  const CommissioningAxisMeta* meta, int numAxes,
                                  CommissioningPlan& out)
{
    out = CommissioningPlan{};
    std::snprintf(out.title, sizeof(out.title), "motion cycle");
    const double freq  = clampd(p.freqHz, 0.05, 1.0);
    const int    cyc   = std::max(1, std::min(10, p.cycles));
    const double dur   = cyc / freq;
    const double ramp  = std::min(1.0, dur * 0.15);
    numAxes = std::min(numAxes, MAX_DRIVES);

    struct Movement { const char* name; bool en; double pct; int which; };
    const Movement movements[] = {
        { "pitch", p.enPitch, p.pitchPct, 0 },
        { "roll",  p.enRoll,  p.rollPct,  1 },
        { "heave", p.enHeave, p.heavePct, 2 },
    };

    for (const Movement& mv : movements)
    {
        if (!mv.en || mv.pct <= 0.0) continue;
        CommissioningSegment sg;
        sg.freqHz = freq;
        sg.durationSec = dur;
        sg.rampSec = ramp;
        setLabel(sg, mv.name);
        bool any = false;
        for (int i = 0; i < numAxes; ++i)
        {
            const CommissioningAxisMeta& m = meta[i];
            if (!m.selected || m.kind != 0) continue;   // verticals only
            double weight = 0.0;
            if (mv.which == 0)      weight = (double)m.frontRear;
            else if (mv.which == 1) weight = (double)m.leftRight;
            else                    weight = 1.0;        // heave: in phase
            if (weight == 0.0) continue;
            sg.ampMm[i] = mv.pct / 100.0 * m.halfStrokeMm * weight;
            any = true;
        }
        if (any && out.numSegments < MAX_TEST_SEGMENTS)
            out.seg[out.numSegments++] = sg;
    }

    // Horizontal axes each get their own movement segment (surge/sway/TL
    // have no pitch/roll/heave meaning -- exercise them one at a time).
    if (p.enHoriz && p.horizPct > 0.0)
    {
        for (int i = 0; i < numAxes; ++i)
        {
            const CommissioningAxisMeta& m = meta[i];
            if (!m.selected || m.kind != 1) continue;
            if (out.numSegments >= MAX_TEST_SEGMENTS) break;
            CommissioningSegment sg;
            sg.freqHz = freq;
            sg.durationSec = dur;
            sg.rampSec = ramp;
            char lbl[24];
            std::snprintf(lbl, sizeof(lbl), "axis %d", i + 1);
            setLabel(sg, lbl);
            sg.ampMm[i] = p.horizPct / 100.0 * m.halfStrokeMm;
            out.seg[out.numSegments++] = sg;
        }
    }
    return out.numSegments;
}

int CommissioningMode::buildTone(double freqHz, double pct, double durationSec,
                                 const CommissioningAxisMeta* meta, int numAxes,
                                 CommissioningPlan& out)
{
    out = CommissioningPlan{};
    std::snprintf(out.title, sizeof(out.title), "vibration tone");
    const double f   = clampd(freqHz, 0.5, 80.0);
    const double dur = clampd(durationSec, 1.0, 60.0);
    CommissioningSegment sg;
    sg.freqHz = f;
    sg.durationSec = dur;
    sg.rampSec = std::min(0.3, dur * 0.2);
    char lbl[24];
    std::snprintf(lbl, sizeof(lbl), "%.1f Hz", f);
    setLabel(sg, lbl);
    bool any = false;
    numAxes = std::min(numAxes, MAX_DRIVES);
    for (int i = 0; i < numAxes; ++i)
    {
        if (!meta[i].selected || meta[i].kind == 2) continue;
        sg.ampMm[i] = clampd(pct, 0.0, 100.0) / 100.0 * meta[i].halfStrokeMm;
        any = any || sg.ampMm[i] != 0.0;
    }
    if (!any) return 0;
    out.seg[out.numSegments++] = sg;
    return out.numSegments;
}

int CommissioningMode::buildSweep(double f0Hz, double f1Hz, double stepHz,
                                  double dwellSec, double pct,
                                  const CommissioningAxisMeta* meta, int numAxes,
                                  CommissioningPlan& out)
{
    out = CommissioningPlan{};
    std::snprintf(out.title, sizeof(out.title), "frequency sweep");
    if (stepHz <= 0.0 || f1Hz < f0Hz) return -1;
    const double dwell = clampd(dwellSec, 1.0, 10.0);
    numAxes = std::min(numAxes, MAX_DRIVES);
    for (double f = f0Hz; f <= f1Hz + 1e-9; f += stepHz)
    {
        if (out.numSegments >= MAX_TEST_SEGMENTS) break;
        CommissioningSegment sg;
        sg.freqHz = clampd(f, 0.5, 80.0);
        sg.durationSec = dwell;
        sg.rampSec = std::min(0.25, dwell * 0.2);
        char lbl[24];
        std::snprintf(lbl, sizeof(lbl), "%.1f Hz", sg.freqHz);
        setLabel(sg, lbl);
        bool any = false;
        for (int i = 0; i < numAxes; ++i)
        {
            if (!meta[i].selected || meta[i].kind == 2) continue;
            sg.ampMm[i] = clampd(pct, 0.0, 100.0) / 100.0 * meta[i].halfStrokeMm;
            any = any || sg.ampMm[i] != 0.0;
        }
        if (!any) return 0;
        out.seg[out.numSegments++] = sg;
    }
    return out.numSegments;
}

double CommissioningMode::noteToFreqHz(const char* tok, double* beatsOut)
{
    if (beatsOut) *beatsOut = 1.0;
    if (!tok || !*tok) return -1.0;

    // split "name:beats"
    char name[16] = {};
    const char* colon = std::strchr(tok, ':');
    size_t nameLen = colon ? (size_t)(colon - tok) : std::strlen(tok);
    if (nameLen == 0 || nameLen >= sizeof(name)) return -1.0;
    std::memcpy(name, tok, nameLen);
    if (colon && beatsOut)
    {
        double b = std::atof(colon + 1);
        if (b <= 0.0 || b > 16.0) return -1.0;
        *beatsOut = b;
    }

    if (std::strcmp(name, "r") == 0 || std::strcmp(name, "-") == 0)
        return 0.0;   // rest

    static const int SEMI[] = { 9, 11, 0, 2, 4, 5, 7 };   // a b c d e f g
    const char c0 = (char)std::tolower((unsigned char)name[0]);
    if (c0 < 'a' || c0 > 'g') return -1.0;
    int semi = SEMI[c0 - 'a'];
    size_t pos = 1;
    if (name[pos] == '#') { semi += 1; ++pos; }
    else if (name[pos] == 'b') { semi -= 1; ++pos; }
    if (!std::isdigit((unsigned char)name[pos])) return -1.0;
    int octave = std::atoi(name + pos);
    if (octave < 0 || octave > 8) return -1.0;
    const int midi = 12 * (octave + 1) + semi;
    return 440.0 * std::pow(2.0, (midi - 69) / 12.0);
}

int CommissioningMode::buildSong(const char* notes, double beatSec, double pct,
                                 const CommissioningAxisMeta* meta, int numAxes,
                                 CommissioningPlan& out)
{
    out = CommissioningPlan{};
    std::snprintf(out.title, sizeof(out.title), "song");
    if (!notes) return -1;
    const double beat = clampd(beatSec, 0.1, 2.0);
    numAxes = std::min(numAxes, MAX_DRIVES);

    const char* p = notes;
    while (*p && out.numSegments < MAX_TEST_SEGMENTS)
    {
        while (*p && std::isspace((unsigned char)*p)) ++p;
        if (!*p) break;
        char tok[24] = {};
        int tl = 0;
        while (*p && !std::isspace((unsigned char)*p) && tl < 23) tok[tl++] = *p++;
        // overlong token: consume the rest, then reject
        if (*p && !std::isspace((unsigned char)*p)) return -1;

        double beats = 1.0;
        const double f = noteToFreqHz(tok, &beats);
        if (f < 0.0) return -1;

        CommissioningSegment sg;
        sg.durationSec = beats * beat;
        sg.rampSec = std::min(0.05, sg.durationSec * 0.25);
        if (f == 0.0)
        {
            sg.freqHz = 1.0;                       // rest: amp stays 0
            setLabel(sg, "rest");
        }
        else
        {
            sg.freqHz = clampd(f, 0.5, 80.0);
            setLabel(sg, tok);
            bool any = false;
            for (int i = 0; i < numAxes; ++i)
            {
                if (!meta[i].selected || meta[i].kind == 2) continue;
                sg.ampMm[i] = clampd(pct, 0.0, 100.0) / 100.0
                            * meta[i].halfStrokeMm;
                any = any || sg.ampMm[i] != 0.0;
            }
            if (!any) return 0;
        }
        out.seg[out.numSegments++] = sg;
    }
    return out.numSegments;
}
