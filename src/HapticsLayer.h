// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// HapticsLayer - one-shot haptic transients, routable across axes.
//
// A separate layer that RIDES ON TOP of the per-axis force pipeline and
// never modifies it: sources (detent capture, wall contact) fire an event;
// the layer synthesizes a short enveloped sine burst; a per-event routing
// table says which torque-mode axes feel it and at what gain, so a shifter
// click can also thump the belt tensioners. The owner (MotionController)
// adds overlayFor(axis) to each torque command AFTER its normal guards and
// clamps the sum inside the axis's existing limits - this layer can never
// exceed what the guard rails already allow.
//
// RT rules, by construction: fixed pools, no allocation, no locks, no
// logging, no mailbox. fire() and step()/overlayFor() are called from the
// same RT thread; the web Test button arrives via the existing command
// queue like every other command. Params and routes are SNAPSHOTTED into
// the event at fire time, so a mid-burst config edit never tears.
//
// Envelope: raised-cosine (wavesynth::envelope), exactly zero at both
// ends - a burst can never step the torque command on entry or exit.
// Amplitude may be scaled at fire time (impact velocity), so a gentle
// detent entry clicks softly and a slam clicks hard.
// ============================================================

#include "HapticsTypes.h"
#include "EngineModel.h"
#include "SlipModel.h"
#include "RoadModel.h"
#include "DrivelineModel.h"
#include "PulseModel.h"
#include "WaveSynth.h"
#include <algorithm>
#include <cstdint>

namespace haptics {

class Layer
{
public:
    // Set/replace the tuning for one event type (config apply; not RT).
    void configure(EventType t, const EffectParams& p)
    {
        m_params[static_cast<int>(t)] = p;
    }

    const EffectParams& params(EventType t) const
    {
        return m_params[static_cast<int>(t)];
    }

    // Fire one transient. scale in [0,1] multiplies the configured
    // amplitude (impact velocity). Inert when the effect's ampPct is 0 or
    // every route is dead. When the pool is full the event is dropped (a
    // missed click is harmless; a stalled RT loop is not).
    // durScale / freqScale let a law shape one firing (a dog box's shift
    // is a shorter, sharper knock than a synchro's clunk) without touching
    // the saved params.
    void fire(EventType t, double scale = 1.0, double durScale = 1.0, double freqScale = 1.0)
    {
        const EffectParams& p = m_params[static_cast<int>(t)];
        if (p.ampPct <= 0.0 || p.durMs <= 0.0) return;
        if (scale <= 0.0) return;
        if (scale > 1.0) scale = 1.0;

        Event* e = nullptr;
        for (Event& c : m_events) if (!c.active) { e = &c; break; }
        if (!e) return;                         // pool full: drop, never block

        // Snapshot params + routes at fire time.
        const double durK = std::max(0.25, std::min(4.0, durScale));
        e->ampPct = p.ampPct * scale;
        e->freqHz = ((p.freqHz > 0.0) ? p.freqHz : 90.0) * std::max(0.25, std::min(4.0, freqScale));
        e->durSec = p.durMs / 1000.0 * durK;
        e->tSec   = 0.0;
        // A position axis cannot carry a 60 Hz burst (a few hundredths of a
        // mm): it gets the same event as one thud at a rate it can follow,
        // stretched with the burst (a synchro's clunk is softer and longer
        // than a dog's knock).
        e->thudHz  = std::max(kThudMinHz, std::min(kThudMaxHz, kThudHz / durK));
        e->thudSec = 1.0 / e->thudHz;
        e->lifeSec = e->durSec;
        bool anyRoute = false;
        for (int i = 0; i < MAX_ROUTES; ++i)
        {
            const Route& r = p.routes[i];
            const bool toAxis   = r.axis >= 0 && r.axis < MAX_HAPTIC_AXES && r.gain > 0.0;
            const bool toShaker = !toAxis && r.shaker >= 0 && r.shaker < MAX_SHAKER_OUT && r.gain > 0.0;
            e->routes[i] = Route{};
            if (toAxis)   { e->routes[i].axis = r.axis; e->routes[i].gain = r.gain;
                            if (m_sinkKind[r.axis] == SinkKind::Position) e->lifeSec = std::max(e->lifeSec, e->thudSec); }
            if (toShaker) { e->routes[i].shaker = r.shaker; e->routes[i].harm = r.harm; e->routes[i].gain = r.gain; }
            anyRoute |= toAxis || toShaker;
        }
        if (!anyRoute) return;                  // fully unrouted: never activate
        e->active = true;
        ++m_fired;
        ++m_firedBy[static_cast<int>(t)];
    }

    // Advance every active transient and continuous effect by one cycle
    // and refresh the per-axis overlay sums. Call ONCE per control cycle
    // (after driveFx calls), before overlayFor().
    void step(double dtSec)
    {
        for (double& o : m_overlay) o = 0.0;
        for (double& o : m_shaker)  o = 0.0;
        if (dtSec <= 0.0) return;

        // One-shot transients.
        for (Event& e : m_events)
        {
            if (!e.active) continue;
            e.tSec += dtSec;
            if (e.tSec >= e.lifeSec) { e.active = false; continue; }
            // Raised-cosine envelope (zero at both ends) on a sine carrier.
            const double env = wavesynth::envelope(e.tSec, e.durSec, e.durSec * 0.25);
            const double v   = e.ampPct * env
                             * std::sin(2.0 * wavesynth::kPi * e.freqHz * e.tSec);
            // The position-sink rendering: one enveloped cycle at the thud rate.
            const double thud = e.ampPct * wavesynth::envelope(e.tSec, e.thudSec, e.thudSec * 0.25)
                              * std::sin(2.0 * wavesynth::kPi * e.thudHz * e.tSec);
            for (const Route& r : e.routes)
            {
                if (r.shaker >= 0) { toShaker(r, e.ampPct * env * std::sin(2.0 * wavesynth::kPi * e.freqHz * r.harm * e.tSec)); continue; }
                if (r.axis < 0) continue;
                if (m_sinkKind[r.axis] == SinkKind::Position)
                    m_overlay[r.axis] += thud * r.gain * sinkScale(r.axis, e.ampPct * r.gain, e.thudHz);
                else
                    m_overlay[r.axis] += v * r.gain;
            }
        }

        // Continuous effects: attack/release-smoothed level on a free-running
        // oscillator (phase-continuous through frequency changes), optional
        // per-cycle carrier jitter for texture-class effects.
        for (int i = 0; i < FX_TYPE_COUNT; ++i)
        {
            Fx& f = m_fx[i];
            const EffectParams& p = m_fxParams[i];
            const bool slipSlot = slipFor(static_cast<FxType>(i)) != nullptr;
            // Road replay: per-corner suspension travel when the sim sends
            // it (the law drives the corners). The texture oscillator below
            // runs as well: it carries the roadNoise fallback when there is
            // no replay, and the surface grain (the law's level) always, so
            // the bumps ride on the tarmac texture rather than replacing it.
            if (i == static_cast<int>(FxType::Road))
            {
                m_roadLevel = 0.0;
                if (m_roadDriven || m_road.active())
                {
                    m_roadDriven = false;
                    if (p.ampPct <= 0.0) m_road.clear();
                    else
                    {
                        m_road.step(dtSec, m_roadParams);
                        m_roadLevel = m_road.level();
                        if (m_roadLevel >= 1e-4 || m_road.active())
                            for (const Route& r : p.routes)
                            {
                                if (r.gain <= 0.0) continue;
                                // A post is told how far (body displacement), a
                                // belt or shaker how hard (body acceleration).
                                const RoadOut o = m_road.outputFor(r.part);
                                if (r.shaker >= 0) { if (o.force != 0.0) toShaker(r, p.ampPct * o.force); continue; }
                                if (r.axis < 0 || r.axis >= MAX_HAPTIC_AXES) continue;
                                // No single carrier in a replay: derate a position sink
                                // at a representative bump rate; the owner's sum guard
                                // holds the axis limits regardless.
                                if (m_sinkKind[r.axis] == SinkKind::Position)
                                    m_overlay[r.axis] += p.ampPct * o.pos * r.gain * sinkScale(r.axis, p.ampPct * r.gain, road_k::kDerateHz);
                                else
                                    m_overlay[r.axis] += p.ampPct * o.force * r.gain;
                            }
                    }
                }
                // The surface grain (the suspension model only: the tyre
                // model's road already carries its roughness): its own rough
                // oscillator at surface hz, whole-car.
                {
                    if (static_cast<int>(m_roadParams.model + 0.5) != 0) m_surfTarget = 0.0;
                    const double rate = (m_surfTarget > m_surfLevel) ? dtSec / 0.050 : dtSec / 0.120;
                    m_surfLevel += std::max(-rate, std::min(rate, m_surfTarget - m_surfLevel));
                    m_surfTarget = 0.0;   // the law drives it every cycle
                    const double sHz = std::max(2.0, m_roadParams.surfaceHz);
                    if (p.ampPct > 0.0 && m_surfLevel >= 1e-4)
                    {
                        double f = sHz;
                        const double jit = std::max(p.jitter, kSurfaceMinJitter);
                        m_surfRng ^= m_surfRng << 13; m_surfRng ^= m_surfRng >> 7; m_surfRng ^= m_surfRng << 17;
                        f *= 1.0 + jit * (static_cast<double>(m_surfRng & 0xFFFF) / 65535.0 - 0.5);
                        const double v = p.ampPct * m_surfLevel * m_surfOsc.step(f, dtSec);
                        for (const Route& r : p.routes)
                        {
                            if (r.gain <= 0.0) continue;
                            if (r.shaker >= 0) { toShaker(r, p.ampPct * m_surfLevel * std::sin(r.harm * m_surfOsc.phase)); continue; }
                            if (r.axis < 0 || r.axis >= MAX_HAPTIC_AXES) continue;
                            m_overlay[r.axis] += v * r.gain * sinkScale(r.axis, p.ampPct * r.gain, sHz);
                        }
                    }
                }
                // ...and on to the texture path with the law's level.
            }
            // Kerb: the rumble strip under whichever tyres are on one.
            if (i == static_cast<int>(FxType::Kerb))
            {
                if (p.ampPct <= 0.0) { m_kerb.clear(); f.level = 0.0; continue; }
                m_kerb.step(dtSec, p, m_kerbParams);
                f.level = m_kerb.level();
                if (!m_kerb.active()) continue;
                for (const Route& r : p.routes)
                {
                    if (r.gain <= 0.0) continue;
                    const RoadOut o = m_kerb.outputFor(r.part);
                    if (r.shaker >= 0) { if (o.force != 0.0) toShaker(r, p.ampPct * o.force); continue; }
                    if (r.axis < 0 || r.axis >= MAX_HAPTIC_AXES) continue;
                    if (m_sinkKind[r.axis] == SinkKind::Position)
                        m_overlay[r.axis] += p.ampPct * o.pos * r.gain * sinkScale(r.axis, p.ampPct * r.gain, road_k::kDerateHz);
                    else
                        m_overlay[r.axis] += p.ampPct * o.force * r.gain;
                }
                continue;
            }
            if (i == static_cast<int>(FxType::Driveline))
            {
                if (p.ampPct <= 0.0) { m_driveline.clear(); f.level = 0.0; continue; }
                m_driveline.step(dtSec, p, m_drivelineParams);
                f.level = m_driveline.level();
                f.targetLevel = 0.0;
                if (f.level < 1e-4) continue;
                const DrivelineOut& o = m_driveline.out();
                for (const Route& r : p.routes)
                {
                    if (r.gain <= 0.0) continue;
                    if (r.shaker >= 0)
                    {
                        toShaker(r, p.ampPct * (o.judderEnv * std::sin(r.harm * o.judderPhase) + o.lugEnv * std::sin(r.harm * o.lugPhase)
                                              + o.whineEnv * std::sin(r.harm * o.whinePhase) + o.shunt));
                        continue;
                    }
                    if (r.axis < 0 || r.axis >= MAX_HAPTIC_AXES) continue;
                    const double ask = p.ampPct * r.gain;
                    const double v = p.ampPct * (o.judder * sinkScale(r.axis, ask, o.judderHz)
                                               + o.lug    * sinkScale(r.axis, ask, o.lugHz)
                                               + o.whine  * sinkScale(r.axis, ask, o.whineHz)
                                               + o.shunt  * sinkScale(r.axis, ask, o.shuntHz));
                    if (v != 0.0) m_overlay[r.axis] += v * r.gain;
                }
                continue;
            }
            if (!slipSlot)
            {
                // Level ramp: ~50 ms attack, ~120 ms release. (The slip
                // model ramps per wheel itself.)
                const double rate = (f.targetLevel > f.level) ? dtSec / 0.050 : dtSec / 0.120;
                f.level += std::max(-rate, std::min(rate, f.targetLevel - f.level));
                if (p.ampPct <= 0.0 || f.level < 1e-4) continue;
            }
            else if (p.ampPct <= 0.0)
            {
                slipFor(static_cast<FxType>(i))->clear();
                f.level = 0.0;
                continue;
            }

            // The engine slot runs the pulse-train synth, not the oscillator.
            if (i == static_cast<int>(FxType::RpmVibe))
            {
                const EngineOut eo = m_engine.step(dtSec, p, m_engineParams, f.level, f.rng);
                const double v = eo.total();
                for (const Route& r : p.routes)
                {
                    if (r.gain <= 0.0) continue;
                    if (r.shaker >= 0)
                    {
                        // Rock and buzz at the route's harmonic, thumps as they are.
                        const double sv = (r.harm == 1) ? v
                                        : std::max(-eo.cap, std::min(eo.cap, eo.rockAt(r.harm) + eo.thumps + eo.buzzAt(r.harm)));
                        toShaker(r, sv);
                        continue;
                    }
                    if (r.axis < 0 || r.axis >= MAX_HAPTIC_AXES) continue;
                    if (m_sinkKind[r.axis] == SinkKind::Position)
                    {
                        // Each component derated by what this axis can follow
                        // at ITS carrier: the idle rock survives on a vertical
                        // where the thumps and buzz are tiny.
                        const double ask = p.ampPct * r.gain;
                        double pv = eo.rock   * sinkScale(r.axis, ask, eo.rockHz)
                                  + eo.thumps * sinkScale(r.axis, ask, eo.thumpHz)
                                  + eo.buzz   * sinkScale(r.axis, ask, eo.buzzHz);
                        pv = std::max(-eo.cap, std::min(eo.cap, pv));
                        m_overlay[r.axis] += pv * r.gain;
                    }
                    else if (v != 0.0)
                        m_overlay[r.axis] += v * r.gain;
                }
                continue;
            }

            // ABS and TC run the pulse model (PulseModel.h): ABS one channel
            // per corner, a route playing its part's corners, with the pump
            // buzz under it; TC one channel. Each derated at its own rate on
            // a position sink.
            if (i == static_cast<int>(FxType::AbsPulse) || i == static_cast<int>(FxType::TcPulse))
            {
                const bool abs = (i == static_cast<int>(FxType::AbsPulse));
                PulseModel& pm = abs ? m_absModel : m_tcModel;
                pm.step(dtSec, f.freqHz, abs ? m_absParams : m_tcParams);
                const double lvl = p.ampPct * f.level;
                const double bz = pm.buzz(), bzHz = pm.buzzHz();
                for (const Route& r : p.routes)
                {
                    if (r.gain <= 0.0) continue;
                    const double pulse = pm.pulseFor(r.part);
                    if (r.shaker >= 0)
                    {
                        toShaker(r, lvl * (pulse + (bzHz > 0.0 ? bz : 0.0)));
                        continue;
                    }
                    if (r.axis < 0 || r.axis >= MAX_HAPTIC_AXES) continue;
                    const double ask = p.ampPct * r.gain;
                    double v = lvl * pulse * sinkScale(r.axis, ask, pm.rateHz());
                    if (bzHz > 0.0) v += lvl * bz * sinkScale(r.axis, ask, bzHz);
                    m_overlay[r.axis] += v * r.gain;
                }
                continue;
            }

            // The two slip slots run the per-wheel model: each route takes
            // the strongest wheel of its part, per component, derated per
            // component carrier on a position sink.
            if (SlipModel* sm = slipFor(static_cast<FxType>(i)))
            {
                sm->step(dtSec, p, *slipParamsFor(static_cast<FxType>(i)));
                f.level = sm->level();
                if (f.level < 1e-4) continue;
                for (const Route& r : p.routes)
                {
                    if (r.gain <= 0.0) continue;
                    const SlipModel::Out o = sm->outputFor(r.part);
                    if (r.shaker >= 0)
                    {
                        toShaker(r, p.ampPct * (o.aEnv * std::sin(r.harm * o.aPhase) + o.bEnv * std::sin(r.harm * o.bPhase)));
                        continue;
                    }
                    if (r.axis < 0 || r.axis >= MAX_HAPTIC_AXES) continue;
                    const double ask = p.ampPct * r.gain;
                    const double v = p.ampPct * (o.a * sinkScale(r.axis, ask, o.aHz)
                                               + o.b * sinkScale(r.axis, ask, o.bHz));
                    if (v != 0.0) m_overlay[r.axis] += v * r.gain;
                }
                continue;
            }

            double freq = f.freqHz;
            if (freq < 0.5) continue;   // no usable carrier = silence, not DC
            if (p.jitter > 0.0)
            {
                // xorshift PRNG: pure arithmetic, RT-safe. Roughens the
                // carrier so skid/road read as texture, not a tone.
                f.rng ^= f.rng << 13; f.rng ^= f.rng >> 7; f.rng ^= f.rng << 17;
                const double r = static_cast<double>(f.rng & 0xFFFF) / 65535.0; // 0..1
                freq *= 1.0 + p.jitter * (r - 0.5);
            }
            const double v = p.ampPct * f.level * f.osc.step(freq, dtSec);
            for (const Route& r : p.routes)
            {
                if (r.gain <= 0.0) continue;
                if (r.shaker >= 0)
                {
                    toShaker(r, p.ampPct * f.level * std::sin(r.harm * f.osc.phase));
                    continue;
                }
                if (r.axis >= 0 && r.axis < MAX_HAPTIC_AXES)
                    m_overlay[r.axis] += v * r.gain * sinkScale(r.axis, p.ampPct * r.gain, f.freqHz);
            }
        }

        // Shaker tone test: a 40 Hz sine on one channel for a moment.
        if (m_toneLeft > 0.0 && m_toneChannel >= 0 && m_toneChannel < MAX_SHAKER_OUT)
        {
            m_toneLeft -= dtSec;
            m_shaker[m_toneChannel] += 0.5 * m_toneOsc.step(40.0, dtSec);
        }

        // Master trim last: per-effect settings stay untouched underneath.
        const double mg = m_muted ? 0.0 : m_masterGain;
        if (mg != 1.0) { for (double& o : m_overlay) o *= mg; for (double& o : m_shaker) o *= mg; }

        // Axis alignment delay: the fast sinks (torque, position) can be
        // held back a few ms so they land together with the slower shaker
        // path. The haptic overlay only, never the motion cue.
        if (m_delayCycles > 0)
        {
            for (int a = 0; a < MAX_HAPTIC_AXES; ++a)
            {
                m_delayBuf[a][m_delayHead] = static_cast<float>(m_overlay[a]);
                const int readAt = (m_delayHead + MAX_DELAY_CYCLES - m_delayCycles) % MAX_DELAY_CYCLES;
                m_overlay[a] = m_delayBuf[a][readAt];
            }
            m_delayHead = (m_delayHead + 1) % MAX_DELAY_CYCLES;
        }
    }

    // Shaker channel sample for this cycle, -1..1 nominal (100% amp x gain
    // 1 = full scale; the DSP soft-clips beyond).
    double shakerSample(int i) const
    {
        return (i >= 0 && i < MAX_SHAKER_OUT) ? m_shaker[i] : 0.0;
    }
    const double* shakerSamples() const { return m_shaker; }

    // Tone test (RT thread, via the command queue).
    void startShakerTone(int channel, double sec) { m_toneChannel = channel; m_toneLeft = sec; m_toneOsc.reset(); }
    bool shakerToneActive() const { return m_toneLeft > 0.0; }

    // Axis alignment delay in control cycles (0 = none; config apply).
    void setAxisDelayCycles(int cycles)
    {
        m_delayCycles = std::max(0, std::min(MAX_DELAY_CYCLES - 1, cycles));
        for (auto& row : m_delayBuf) for (float& v : row) v = 0.0f;
        m_delayHead = 0;
    }
    int axisDelayCycles() const { return m_delayCycles; }

    // Torque overlay (% of rated) for one axis this cycle. The caller adds
    // it to the axis command and clamps the SUM inside the axis limits.
    double overlayFor(int axis) const
    {
        return (axis >= 0 && axis < MAX_HAPTIC_AXES) ? m_overlay[axis] : 0.0;
    }

    // ---- sinks -----------------------------------------------------------
    // What each axis index IS as a destination. Torque (the default): the
    // overlay is % of rated torque, a route gain is a plain multiplier.
    // Position: a route gain is mm at 100% effect amplitude, the overlay is
    // (% x gain) and the owner divides by 100 to get mm; every contribution
    // is first derated to what the axis can physically follow at the
    // effect's carrier - min(cap, vBudget/w, aBudget/w^2), the commissioning
    // sweep's rule - so a 35 Hz texture on a heavy vertical arrives at the
    // physics amplitude instead of a demand the drive would clip, while the
    // 13 Hz idle rock comes through. Config apply; not RT.
    void setSinkKind(int axis, SinkKind k)
    {
        if (axis >= 0 && axis < MAX_HAPTIC_AXES) m_sinkKind[axis] = k;
    }
    void setPositionLimits(int axis, double vBudgetMmS, double aBudgetMmS2, double capMm)
    {
        if (axis < 0 || axis >= MAX_HAPTIC_AXES) return;
        m_posV[axis]   = std::max(0.0, vBudgetMmS);
        m_posA[axis]   = std::max(0.0, aBudgetMmS2);
        m_posCap[axis] = std::max(0.0, capMm);
    }
    SinkKind sinkKind(int axis) const
    {
        return (axis >= 0 && axis < MAX_HAPTIC_AXES) ? m_sinkKind[axis] : SinkKind::Torque;
    }
    // Largest peak offset (mm) a position axis can take at a carrier.
    double positionAllowedMm(int axis, double hz) const
    {
        if (axis < 0 || axis >= MAX_HAPTIC_AXES) return 0.0;
        double allowed = m_posCap[axis];
        if (hz > 0.0)
        {
            const double w = 2.0 * wavesynth::kPi * hz;
            allowed = std::min(allowed, std::min(m_posV[axis] / w, m_posA[axis] / (w * w)));
        }
        return allowed;
    }

    // ---- continuous effects -------------------------------------------------

    void configureFx(FxType t, const EffectParams& p)
    {
        m_fxParams[static_cast<int>(t)] = p;
    }

    const EffectParams& fxParams(FxType t) const
    {
        return m_fxParams[static_cast<int>(t)];
    }

    // Engine description + mix (config apply; not RT).
    void configureEngine(const EngineParams& e) { m_engineParams = e; }
    const EngineParams& engineParams() const   { return m_engineParams; }
    double learnedMaxRpm() const               { return m_engine.learnedMaxRpm(); }

    // ---- per-wheel slip (the Lockup = longitudinal and Skid = lateral
    // slots). Config apply for the params; the law drives wheels per cycle.
    void configureSlip(FxType t, const SlipParams& s)
    {
        if (SlipParams* sp = slipParamsFor(t)) *sp = s;
    }
    const SlipParams& slipParams(FxType t) const
    {
        return (t == FxType::Skid) ? m_slipLatParams : m_slipLonParams;
    }
    // Component severities (0..1) for one wheel this cycle. Lateral: a =
    // scrub, b = slide. Longitudinal: a = lock, b = spin.
    void driveSlip(FxType t, int wheel, double a, double b)
    {
        if (SlipModel* sm = slipFor(t)) sm->drive(wheel, a, b);
    }
    void setSlipCarrierScale(FxType t, double aScale, double bScale)
    {
        if (SlipModel* sm = slipFor(t)) sm->setCarrierScale(aScale, bScale);
    }
    double slipWheelLevel(FxType t, int wheel) const
    {
        const SlipModel* sm = (t == FxType::Skid) ? &m_slipLat : (t == FxType::Lockup) ? &m_slipLon : nullptr;
        return sm ? sm->wheelLevel(wheel) : 0.0;
    }
    static bool isSlipSlot(FxType t) { return t == FxType::Skid || t == FxType::Lockup; }

    // ---- per-corner road replay (the Road slot). Config apply for the
    // params; the law drives the corners per cycle when the sim sends
    // suspension velocities, and does not when it sends roadNoise.
    void configureRoad(const RoadParams& r) { m_roadParams = r; }
    const RoadParams& roadParams() const   { return m_roadParams; }
    void driveRoad(int wheel, double velMmS)
    {
        m_road.drive(wheel, velMmS);
        m_roadDriven = true;
    }
    void driveRoadTravel(int wheel, double travelMm)
    {
        m_road.driveTravel(wheel, travelMm);
        m_roadDriven = true;
    }
    double roadWheelTravelMm(int wheel) const { return m_road.wheelTravelMm(wheel); }
    double roadReplayLevel() const            { return m_roadLevel; }
    bool   roadReplaying() const              { return m_road.active(); }
    // The road under each tyre (mm, world), the surface class there, the
    // car's speed, its body and geometry: the tyre and chassis models.
    void driveRoadHeight(int wheel, double heightMm) { m_road.driveHeight(wheel, heightMm); m_roadDriven = true; }
    void driveRoadSurface(int wheel, int cls)        { m_road.driveSurface(wheel, cls); }
    void driveRoadSpeed(double speedMs)              { m_road.driveSpeed(speedMs); m_kerb.driveSpeed(speedMs); m_roadDriven = true; }
    void driveChassis(double accHeaveMs2, double pitchDeg, double rollDeg)
    {
        m_road.driveChassis(accHeaveMs2, pitchDeg, rollDeg); m_roadDriven = true;
    }
    void driveGeometry(double wheelbaseM, double trackM) { m_road.driveGeometry(wheelbaseM, trackM); }
    const RoadModel& roadModel() const               { return m_road; }
    // The surface grain's level 0..1 for THIS cycle (the law: the surface
    // mix x how far up to surface km/h the car is). Not driven = releasing.
    void   driveRoadGrain(double level)       { m_surfTarget = std::max(0.0, std::min(1.0, level)); }
    double roadGrainLevel() const             { return m_surfLevel; }

    // ---- kerb (the rumble strip). Config apply for the params; the law
    // says per wheel, per cycle, how much of the tyre is on a kerb.
    void configureKerb(const KerbParams& k)   { m_kerbParams = k; }
    const KerbParams& kerbParams() const      { return m_kerbParams; }
    void   driveKerb(int wheel, double level) { m_kerb.drive(wheel, level); }
    void   driveKerbSpeed(double speedMs)     { m_kerb.driveSpeed(speedMs); }   // driveRoadSpeed sets it too
    double kerbWheelLevel(int wheel) const    { return m_kerb.wheelLevel(wheel); }
    const KerbModel& kerbModel() const        { return m_kerb; }

    // ---- driveline (clutch judder + lugging wind-up). Config apply for
    // the params; the law drives the two severities per cycle.
    // ---- ABS and TC (the pulse model). Config apply for the params; the
    // laws drive the level with driveFx and, for ABS, the road speed.
    void configurePulse(FxType t, const PulseParams& q)
    {
        if (t == FxType::AbsPulse) m_absParams = q;
        else if (t == FxType::TcPulse) m_tcParams = q;
    }
    const PulseParams& pulseParams(FxType t) const { return (t == FxType::TcPulse) ? m_tcParams : m_absParams; }
    void driveAbsSpeed(double kmh)                 { m_absModel.driveSpeed(kmh); }   // negative = unknown
    const PulseModel& absModel() const             { return m_absModel; }
    const PulseModel& tcModel() const              { return m_tcModel; }

    void configureDriveline(const DrivelineParams& d) { m_drivelineParams = d; }
    const DrivelineParams& drivelineParams() const   { return m_drivelineParams; }
    void driveDriveline(double judder, double lug)    { m_driveline.drive(judder, lug); }
    void driveDrivelineWhine(double level, double hz) { m_driveline.driveWhine(level, hz); }
    void drivelineShunt(double strength)              { m_driveline.shunt(strength); }
    double drivelineWhineLevel() const                { return m_driveline.whineLevel(); }
    double drivelineJudderLevel() const               { return m_driveline.judderLevel(); }
    double drivelineLugLevel() const                  { return m_driveline.lugLevel(); }

    // Drive one continuous effect for THIS cycle: level 0..1 (silence to
    // full configured amplitude) and the carrier frequency to use (RpmVibe
    // passes rpm/60 x order; others pass their configured freqHz). Called
    // every cycle by the owner BEFORE step(); a level not driven this
    // cycle decays on the release ramp, so a crashed source fades out
    // rather than droning.
    void driveFx(FxType t, double level, double freqHz)
    {
        Fx& f = m_fx[static_cast<int>(t)];
        f.targetLevel = (level < 0.0) ? 0.0 : (level > 1.0 ? 1.0 : level);
        // No driven frequency = the effect's configured carrier.
        f.freqHz = (freqHz > 0.0) ? freqHz : m_fxParams[static_cast<int>(t)].freqHz;
    }

    // ---- master trim (RT-thread writes only, like everything here) ----
    // gain scales EVERY overlay (0..2); mute is a hard zero. Both applied
    // at the final sum, so per-effect settings stay untouched underneath.
    void setMasterGain(double g) { m_masterGain = (g < 0.0) ? 0.0 : (g > 2.0 ? 2.0 : g); }
    void setMuted(bool m)        { m_muted = m; }
    bool muted() const           { return m_muted; }

    // Live level of one continuous effect (0..1 smoothed) - status surface.
    // The Road slot shows the louder of its replay and its texture.
    double fxLevel(int i) const
    {
        if (i < 0 || i >= FX_TYPE_COUNT) return 0.0;
        return (i == static_cast<int>(FxType::Road)) ? std::max(m_fx[i].level, std::max(m_roadLevel, m_surfLevel))
                                                     : m_fx[i].level;
    }

    // What the effect is actually PUTTING OUT, not what it is being driven
    // with: zero when its amplitude is 0, it has no route with gain, or the
    // layer is muted. The web wave draws this, so a tile only animates when
    // something can be felt (a bench session read a full wave on an effect
    // at amp 0 / no route as "fires visually, nothing felt").
    double fxOutputLevel(int i) const
    {
        if (i < 0 || i >= FX_TYPE_COUNT || m_muted || m_masterGain <= 0.0) return 0.0;
        const EffectParams& p = m_fxParams[i];
        if (p.ampPct <= 0.0 || !hasRoute(p)) return 0.0;
        return fxLevel(i);   // Road: its model or grain as well as the texture
    }

    static bool hasRoute(const EffectParams& p)
    {
        for (const Route& r : p.routes)
            if (r.gain > 0.0 && ((r.axis >= 0 && r.axis < MAX_HAPTIC_AXES) || (r.shaker >= 0 && r.shaker < MAX_SHAKER_OUT))) return true;
        return false;
    }

    // ---- pulse-train engine (replaces the generic oscillator for the
    // RpmVibe slot). A real engine FIRES rather than hums: each firing is
    // a short damped thump at rpm/60 x cylinders/2. Low rpm = discrete
    // chunky pulses; rising rpm merges them into buzz by physics, not by
    // crossfade. load scales pulse strength (lugging hits harder than
    // coasting), the lope amount (EffectParams.jitter) roughens idle
    // per-pulse, and the limiter flag DROPS pulses in bursts - a limiter
    // cuts firings, so the stumble is missing events, exactly as felt. ----
    // tcCut: traction control is cutting: the engine drops a share of its
    // firings in bursts, like a shallower, irregular limiter.
    void driveEngine(double level, double fireHz, double load01, bool limiterOn, double boostBar = -1.0,
                     bool pitLimiter = false, bool inGearOverrun = false, bool tcCut = false)
    {
        Fx& f = m_fx[static_cast<int>(FxType::RpmVibe)];
        f.targetLevel = (level < 0.0) ? 0.0 : (level > 1.0 ? 1.0 : level);
        m_engine.drive(fireHz, load01, limiterOn, boostBar, pitLimiter, inGearOverrun, tcCut);
    }
    uint64_t engineTcDrops() const  { return m_engine.tcDropCount(); }
    uint64_t engineLiftOffs() const { return m_engine.liftOffCount(); }
    bool     engineRunning() const  { return m_engine.running(); }

    // Kill every active transient instantly (e-stop, park, loop stop).
    void clearAll()
    {
        for (Event& e : m_events) e.active = false;
        for (Fx& f : m_fx) { f.targetLevel = 0.0; f.level = 0.0; f.osc.reset(); }
        m_engine.clear();
        m_slipLat.clear();
        m_slipLon.clear();
        m_road.clear();
        m_roadDriven = false;
        m_kerb.clear();
        m_surfLevel = m_surfTarget = 0.0; m_surfOsc.reset();
        m_driveline.clear();
        m_absModel.clear();
        m_tcModel.clear();
        for (double& o : m_overlay) o = 0.0;
        for (double& o : m_shaker)  o = 0.0;
        m_toneLeft = 0.0;
    }

    bool anyActive() const
    {
        for (const Event& e : m_events) if (e.active) return true;
        return false;
    }

    // Total transients fired since boot (status/UI activity dot).
    uint64_t fireCount() const { return m_fired; }
    uint64_t fireCount(EventType t) const { return m_firedBy[static_cast<int>(t)]; }

    // One cycle of the ABS / TC waveform at phase ph (radians): from +1 down
    // to -1 over the first `drop` share of the cycle, back up to +1 over the
    // rest, both halves cosine-shaped so the wave is smooth and averages to
    // zero (a position axis does not drift). PulseModel.h's pulseShape.
    static double dropRecover(double ph, double drop)
    {
        return pulseShape(ph / (2.0 * wavesynth::kPi), drop);
    }

private:
    struct Event
    {
        bool   active = false;
        double ampPct = 0.0, freqHz = 0.0, durSec = 0.0, tSec = 0.0;
        double thudHz = 12.0, thudSec = 1.0 / 12.0, lifeSec = 0.0;   // position-sink rendering
        Route  routes[MAX_ROUTES];
    };

    // A transient on a position axis: one cycle at this rate (a gear change
    // through a seat is a jolt, not a buzz), stretched or shortened with
    // the burst's length scale within these bounds.
    static constexpr double kThudHz = 12.0, kThudMinHz = 6.0, kThudMaxHz = 20.0;
    // The surface grain is always rough: tarmac is not a tone.
    static constexpr double kSurfaceMinJitter = 0.5;

    struct Fx
    {
        double targetLevel = 0.0;   // driven per cycle by the owner
        double level       = 0.0;   // attack/release-smoothed
        double freqHz      = 30.0;
        wavesynth::Oscillator osc;
        uint64_t rng = 0x9E3779B97F4A7C15ull;   // xorshift state
    };

    // Route scale for one contribution: 1 on a torque sink; on a position
    // sink, allowed / asked where asked = (ampPct x gain) / 100 mm.
    double sinkScale(int axis, double askedPctTimesGain, double hz) const
    {
        if (axis < 0 || axis >= MAX_HAPTIC_AXES || m_sinkKind[axis] != SinkKind::Position) return 1.0;
        const double askedMm = askedPctTimesGain / 100.0;
        if (askedMm <= 0.0) return 0.0;
        return std::min(1.0, positionAllowedMm(axis, hz) / askedMm);
    }

    SlipModel* slipFor(FxType t)
    {
        return (t == FxType::Skid) ? &m_slipLat : (t == FxType::Lockup) ? &m_slipLon : nullptr;
    }
    const SlipParams* slipParamsFor(FxType t) const
    {
        return (t == FxType::Skid) ? &m_slipLatParams : (t == FxType::Lockup) ? &m_slipLonParams : nullptr;
    }
    SlipParams* slipParamsFor(FxType t)
    {
        return (t == FxType::Skid) ? &m_slipLatParams : (t == FxType::Lockup) ? &m_slipLonParams : nullptr;
    }

    // Add one route's value (in % of rated, like an axis overlay) to its
    // shaker channel as a -1..1 sample.
    void toShaker(const Route& r, double pct)
    {
        if (r.shaker >= 0 && r.shaker < MAX_SHAKER_OUT) m_shaker[r.shaker] += pct * r.gain / 100.0;
    }

    static constexpr int MAX_DELAY_CYCLES = 128;   // 64 ms at 2 kHz, 256 ms at 500 Hz
    double       m_shaker[MAX_SHAKER_OUT] = {};
    int          m_toneChannel = -1;
    double       m_toneLeft = 0.0;
    wavesynth::Oscillator m_toneOsc;
    int          m_delayCycles = 0, m_delayHead = 0;
    float        m_delayBuf[MAX_HAPTIC_AXES][MAX_DELAY_CYCLES] = {};
    SinkKind     m_sinkKind[MAX_HAPTIC_AXES] = {};
    double       m_posV[MAX_HAPTIC_AXES] = {}, m_posA[MAX_HAPTIC_AXES] = {}, m_posCap[MAX_HAPTIC_AXES] = {};
    EngineModel  m_engine;
    EngineParams m_engineParams;
    SlipModel    m_slipLat, m_slipLon;
    SlipParams   m_slipLatParams{ 1.0, 20.0, 1.0, 11.0, 7.0 };
    SlipParams   m_slipLonParams{ 1.0,  9.0, 1.0, 16.0, 0.8 };
    RoadModel    m_road;
    RoadParams   m_roadParams;
    KerbModel    m_kerb;
    KerbParams   m_kerbParams;
    DrivelineModel  m_driveline;
    DrivelineParams m_drivelineParams;
    PulseModel   m_absModel{ WHEEL_COUNT, pulse_k::kAbsDrop };
    PulseModel   m_tcModel{ 1, pulse_k::kTcDrop };
    PulseParams  m_absParams;
    PulseParams  m_tcParams{ 0.3, 0.5, 0.0, 0.0, 40.0 };
    bool         m_roadDriven = false;   // a law drove corners this cycle
    double       m_roadLevel  = 0.0;     // the replay's level this cycle (the texture has its own)
    double       m_surfTarget = 0.0, m_surfLevel = 0.0;   // the surface grain (Road slot)
    wavesynth::Oscillator m_surfOsc;
    uint64_t     m_surfRng = 0xA0761D6478BD642Full;
    EffectParams m_params[EVENT_TYPE_COUNT];
    EffectParams m_fxParams[FX_TYPE_COUNT];
    Event        m_events[MAX_EVENTS];
    Fx           m_fx[FX_TYPE_COUNT];
    double       m_overlay[MAX_HAPTIC_AXES] = {};
    double       m_masterGain = 1.0;
    bool         m_muted      = false;
    uint64_t     m_fired = 0;
    uint64_t     m_firedBy[EVENT_TYPE_COUNT] = {};
};

} // namespace haptics
