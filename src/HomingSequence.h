// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// HomingSequence.h
//
// Purely CSP-based torque homing:
//   1. Enable drive (if not already enabled)
//   2. CSP torque search: increment from actual position
//   3. Hardstop found via torque threshold
//   4. Backoff: move away from hardstop
//   5. Record home offset = raw position at backoff point
//   6. Complete -- no mode switches, no SDOs
//
// The drive stays in CSP mode (set during init) throughout.
// All position commands use raw accessors (no offset) since
// the offset hasn't been set yet during homing.
// ============================================================

#include "A6Drive.h"
#include "AxisKind.h"
#include "Config.h"
#include "Logging.h"

#include <string>
#include <cmath>
#include <algorithm>

class HomingSequence
{
public:
    enum class State
    {
        Idle,
        EnablingDrive,
        SettlingDrive,   // hold at actual pos for SETTLE_CYCLES before torque search
        CSPTorqueSearch,
        Backoff,
        Complete,
        Error,
        FatalError,  // terminal state -- does not auto-restart, requires explicit startHoming()
        Seated       // deinit SEAT mode only: held ON the hardstop, ready to de-energize.
                     // NOT homed -- no back-off, no home reference established.
    };

    HomingSequence() = default;

    // Raw-frame direction of the homing search, derived from the axis mechanics
    // and the (rare) far-side override -- the ONE place this is computed:
    //
    //   invertDir      false = inline actuator (retract = raw positive)
    //                  true  = foldback linkage (retract = raw negative)
    //   homeDirection  "negative" (default) = home to the RETRACTED stop
    //                  "positive"           = home to the EXTENDED stop
    //                  (travel-frame names: negative travel = retract)
    //
    // The frame sign the axis runs under afterwards is the opposite of this
    // (engineering increases AWAY from whichever stop was homed), so even a
    // wrong invertDir only homes to the unintended end -- the frame is still
    // built away from the stop that was actually found, and the axis cannot
    // be commanded through it.
    static double searchSign(bool invertDir, const std::string& homeDirection)
    {
        const double retractRaw = invertDir ? -1.0 : 1.0;
        return (homeDirection == "positive") ? -retractRaw : retractRaw;
    }

    void configure(const DriveConfig& cfg, double cycleTimeSec)
    {
        m_backoffMm = cfg.homingBackoffMm;
        m_speedMmS = cfg.homingSpeed;
        m_torquePct = cfg.homingTorquePct;
        m_homeDir  = cfg.homeDirection;
        m_cycleTimeSec = cycleTimeSec;
        m_state = State::Idle;
        m_dirSign = searchSign(cfg.invertDir, cfg.homeDirection);
        m_unit = axisCaps(cfg.axisType, cfg.mode).unit;   // "mm" / "deg" for logs
        m_seatMode = false;   // normal homing unless start(seatMode=true) is used
        // Abort torque search if travel exceeds 1.5x configured stroke. The
        // stroke itself is kept so a timeout can report how far the search got
        // as a fraction of the axis, not just how long it ran.
        m_strokeMm      = cfg.strokeMm;
        m_strokeLimitMm = cfg.strokeMm * 1.5;
    }

    // seatMode=true: identical search down to the hardstop at the torque threshold, but at
    // the hardstop it HOLDS on the stop (State::Seated) instead of backing off + completing.
    // It establishes NO home reference -- it is parking on the stop, not homing.
    void start(A6Drive* drive, bool seatMode = false)
    {
        if (!drive) return;
        m_seatMode = seatMode;
        m_state = State::EnablingDrive;
        m_elapsed = 0.0;
        m_cycleCount = 0;
        m_torqueConfirmCount = 0;
        m_startPos = drive->getActualPositionRaw();
        m_hardstopPos = 0.0;
        m_homeOffset = 0.0;
        m_peakTorque = 0.0;
        m_baselineTorque = 0.0;
        m_settleTorqueSum = 0.0;
        m_lastCommandedPos = m_startPos;
        m_lastCommandedValid = false;
        LOG_INFO(strf("HomingSequence[%d]: Starting CSP torque homing. "
            "backoff=%.2f%s speed=%.1f%s/s torqueThreshold=%d%% dir=%s",
            drive->getSlaveIndex(), m_backoffMm, m_unit, m_speedMmS, m_unit,
            m_torquePct, m_homeDir.c_str()));
    }

    void reset()
    {
        m_state = State::Idle;
        m_elapsed = 0.0;
        m_cycleCount = 0;
        m_torqueConfirmCount = 0;
        m_homeOffset = 0.0;
        m_seatMode = false;
    }

    // Per-cycle homing increment (mm, unsigned magnitude).
    //
    // Homing commands target = actual + this step every cycle, so the drive's
    // following error is structurally capped at one step even when stalled
    // against the hardstop -- that self-limiting property is what keeps the
    // press gentle and Er87 off the table. The achieved velocity is therefore
    // (drive position-loop gain) x step, NOT step/dt.
    //
    // Two deliberate properties:
    //   1. Loop-rate independent -- the step uses a fixed reference dt
    //      (HOMING_REF_DT_SEC), not the live loop period, so homing speed
    //      does not change when controlLoopHz changes.
    //   2. SPEED_SCALE is an empirical calibration. Because achieved velocity
    //      depends on the unknown drive gain, the displayed mm/s only equals
    //      real mm/s once this is trimmed against a measured home (watch the
    //      Vel readout on the drive card). Raise it to make homing faster.
    // HOMING_MAX_STEP_MM hard-caps the per-cycle following error so no typo in
    // speed/scale can ever push it into the drive's FE fault window.
    double homingStepMm() const
    {
        double s = m_speedMmS * HOMING_REF_DT_SEC * HOMING_SPEED_SCALE;
        return std::min(s, HOMING_MAX_STEP_MM);
    }

    // Homing issues no SDO traffic -- pure cyclic PDO commands.
    State step(A6Drive* drive)
    {
        if (!drive || m_state == State::Idle ||
            m_state == State::Complete || m_state == State::Error ||
            m_state == State::FatalError)
            return m_state;

        ++m_cycleCount;
        m_elapsed += m_cycleTimeSec;
        drive->updateStatus();

        if (drive->isFault() && m_state != State::EnablingDrive)
        {
            if (m_state == State::SettlingDrive)
                LOG_ERROR(strf("HomingSequence[%d]: Drive faulted during settle (SW=0x%04x) -- "
                    "pre-enable sync was not sufficient. Aborting homing.",
                    drive->getSlaveIndex(), drive->getStatusword()));
            else
                LOG_ERROR(strf("HomingSequence[%d]: Drive FAULT during homing! SW=0x%04x torque=%.1f%%",
                    drive->getSlaveIndex(), drive->getStatusword(),
                    drive->getTorquePercent()));
            // FatalError -- does not auto-restart. Requires explicit startHoming().
            m_state = State::FatalError;
            return m_state;
        }

        switch (m_state)
        {
            // ---------------------------------------------------
        case State::EnablingDrive:
        {
            if (drive->isFault())
            {
                drive->stepFaultReset();
            }
            else
            {
                bool enabled = drive->stepEnableStateMachine();
                if (enabled)
                {
                    // Seed target to actual and enter SettlingDrive.
                    // Do NOT start torque search immediately -- the drive's
                    // position loop needs a few cycles to adopt the seeded
                    // target. Jumping straight to CSPTorqueSearch on the
                    // same cycle causes a 1-cycle following error spike
                    // (Er87.x) because the drive still holds its pre-enable
                    // reference while the PDO already carries a new target.
                    double actualPos = drive->getActualPositionRaw();
                    m_startPos = actualPos;
                    drive->setTargetPositionRaw(actualPos);
                    m_settleCount = 0;
                    m_state = State::SettlingDrive;
                    LOG_INFO(strf("HomingSequence[%d]: Drive enabled -- settling at pos=%.3f before search.",
                        drive->getSlaveIndex(), actualPos));
                }
            }
            if (m_elapsed > ENABLE_TIMEOUT_SEC)
            {
                LOG_ERROR(strf("HomingSequence[%d]: Timeout waiting for drive enable (%.0fs). "
                    "Check drive power and EtherCAT state.",
                    drive->getSlaveIndex(), ENABLE_TIMEOUT_SEC));
                m_state = State::FatalError;
            }
            break;
        }

        // ---------------------------------------------------
        // Settle state -- hold at seeded target for
        // SETTLE_CYCLES before commanding any movement.
        // Gives the drive's position loop time to adopt the
        // new reference without a following error spike.
        // ---------------------------------------------------
        case State::SettlingDrive:
        {
            double actualPos = drive->getActualPositionRaw();
            drive->setTargetPositionRaw(actualPos);  // track actual, don't move
            m_lastCommandedPos = actualPos;           // keep spike guard in sync
            // Accumulate the SIGNED static torque while holding still. Its mean is
            // the load-holding baseline: ~0% unloaded, 30-40% with a driver seated.
            m_settleTorqueSum += drive->getTorquePercent();
            ++m_settleCount;
            if (m_settleCount >= SETTLE_CYCLES)
            {
                m_startPos = actualPos;
                m_lastCommandedPos = actualPos;
                m_lastCommandedValid = true;
                m_elapsed = 0.0;
                m_cycleCount = 0;
                m_torqueConfirmCount = 0;
                m_peakTorque = 0.0;
                m_baselineTorque = m_settleTorqueSum / SETTLE_CYCLES;
                m_state = State::CSPTorqueSearch;
                LOG_INFO(strf("HomingSequence[%d]: Drive settled -- starting CSP torque search "
                    "(holding baseline %.1f%%; hardstop = baseline %+d%%).",
                    drive->getSlaveIndex(), m_baselineTorque, m_torquePct));
            }
            break;
        }

        // ---------------------------------------------------
        // CSP torque search: command increments FROM ACTUAL
        // position each cycle. This keeps commanded position
        // close to actual even when motor stalls, preventing
        // following error faults.
        //
        // Uses raw position accessors -- no offset applied.
        // ---------------------------------------------------
        case State::CSPTorqueSearch:
        {
            // Hardstop detection keys on torque DEVIATION from the settled holding
            // baseline, not magnitude. With a driver seated, the static holding
            // torque alone is 30-40%, so a plain |torque| >= threshold false-triggers
            // instantly at the park position and the seat de-energizes in mid-air.
            // Contacting the stop transfers the load OFF the motor, so
            // torque MUST swing away from the holding baseline by the press amount --
            // the deviation reads the same ~25% loaded or unloaded, and unloaded
            // (baseline ~0) this reduces to a plain magnitude threshold.
            double torque = std::abs(drive->getTorquePercent() - m_baselineTorque);
            if (torque > m_peakTorque)
                m_peakTorque = torque;

            // Track from actual position -- not a blind increment
            double actualPos = drive->getActualPositionRaw();
            double stepMm = homingStepMm() * m_dirSign;
            double nextPos = actualPos + stepMm;

            // Spike guard -- if the PDO data was corrupted (bad EtherCAT
            // frame), actualPos can be garbage, making nextPos huge.  The WKC
            // gate in ControlLoop already suppresses updateStatus() on bad
            // frames, but this is a second line of defence.  If the proposed
            // nextPos deviates from the last commanded position by more than
            // SPIKE_MULTIPLIER × one normal step, continue from last commanded
            // position instead -- the drive never sees the garbage value.
            if (m_lastCommandedValid)
            {
                double maxStep = homingStepMm() * SPIKE_MULTIPLIER;
                double jumpFromLast = std::abs(nextPos - m_lastCommandedPos);
                if (jumpFromLast > maxStep)
                {
                    LOG_WARNING(strf(
                        "HomingSequence[%d]: Position spike %.1fmm (limit %.1fmm)"
                        " -- likely bad PDO frame, continuing from last commanded pos",
                        drive->getSlaveIndex(), jumpFromLast, maxStep));
                    nextPos = m_lastCommandedPos + stepMm;
                }
            }
            m_lastCommandedPos = nextPos;
            m_lastCommandedValid = true;
            drive->setTargetPositionRaw(nextPos);

            // Check torque against threshold
            if (torque >= static_cast<double>(m_torquePct))
            {
                ++m_torqueConfirmCount;
                if (m_torqueConfirmCount >= TORQUE_CONFIRM_CYCLES)
                {
                    m_hardstopPos = actualPos;
                    double distMoved = std::abs(m_hardstopPos - m_startPos);

                    LOG_INFO(strf("HomingSequence[%d]: HARDSTOP FOUND at pos=%.3f "
                        "(torque=%.1f%%, peak=%.1f%%, distance=%.3fmm, cycles=%d)",
                        drive->getSlaveIndex(), m_hardstopPos,
                        torque, m_peakTorque, distMoved, m_cycleCount));

                    // Hold at current actual position (shared by both modes).
                    drive->setTargetPositionRaw(actualPos);

                    // *** The ONLY divergence between normal homing and seat mode. ***
                    // Everything above (enable, settle, search to torque) is identical.
                    if (m_seatMode)
                    {
                        // Deinit seat: finish HERE, holding on the stop, ready to de-energize.
                        // No back-off, and deliberately NO home reference (m_homeOffset stays 0,
                        // never reaches Complete) -- this is parking on the stop, not homing.
                        m_state = State::Seated;
                        LOG_INFO(strf("HomingSequence[%d]: SEAT -- on hardstop, holding for "
                            "de-energize (no back-off, no home reference).",
                            drive->getSlaveIndex()));
                    }
                    else
                    {
                        m_backoffTarget = m_hardstopPos + (-m_dirSign * m_backoffMm);
                        m_elapsed = 0.0;
                        m_state = State::Backoff;

                        LOG_INFO(strf("HomingSequence[%d]: Backing off %.2f%s to %.3f",
                            drive->getSlaveIndex(), m_backoffMm, m_unit, m_backoffTarget));
                    }
                }
            }
            else
            {
                m_torqueConfirmCount = 0;
            }

            // Progress logging every 2 seconds
            if (m_cycleCount % static_cast<int>(2.0 / m_cycleTimeSec) == 0)
            {
                double dist = std::abs(actualPos - m_startPos);
                LOG_INFO(strf("HomingSequence[%d]: Searching... pos=%.3f "
                    "torque=%.1f%% peak=%.1f%% distance=%.3f%s",
                    drive->getSlaveIndex(), actualPos, torque, m_peakTorque, dist, m_unit));
            }

            // Stroke guard -- abort if distance exceeds 1.5x configured stroke.
            // Prevents prolonged hardstop press if homeDirection is wrong in config.
            double distTraveled = std::abs(drive->getActualPositionRaw() - m_startPos);
            if (distTraveled > m_strokeLimitMm)
            {
                LOG_ERROR(strf("HomingSequence[%d]: STROKE LIMIT exceeded during torque search "
                    "(traveled=%.1f%s, limit=%.1f%s). Check homeDirection in config.",
                    drive->getSlaveIndex(), distTraveled, m_unit, m_strokeLimitMm, m_unit));
                m_state = State::FatalError;
            }
            else if (m_elapsed > HOMING_TIMEOUT_SEC)
            {
                // Report the MEASURED rate, not one derived from the speed setting:
                // that setting is a per-cycle step multiplier whose real-world speed
                // depends on the loop rate, so only distance/elapsed is trustworthy.
                // Travelled-vs-stroke separates the two failure modes at a glance --
                // "still crawling toward the stop" vs "never moved".
                const double measuredMmS = (m_elapsed > 0.0) ? (distTraveled / m_elapsed) : 0.0;
                LOG_ERROR(strf("HomingSequence[%d]: CSP torque search TIMEOUT after %.0fs -- "
                    "travelled %.1f%s of %.1f%s stroke (%.2f%s/s measured; homingSpeed setting=%.0f). "
                    "Peak torque=%.1f%% (threshold=%d%%). If it was still moving, raise homingSpeed; "
                    "if it barely moved, check motor connection, torque threshold, and homing direction.",
                    drive->getSlaveIndex(), HOMING_TIMEOUT_SEC,
                    distTraveled, m_unit, m_strokeMm, m_unit, measuredMmS, m_unit, m_speedMmS,
                    m_peakTorque, m_torquePct));
                m_state = State::FatalError;
            }
            break;
        }

        // ---------------------------------------------------
        // Backoff: track from actual position each cycle.
        // Use reasonable settling tolerance (50um).
        // Limit step to remaining distance to prevent overshoot.
        //
        // Uses raw position accessors -- no offset applied.
        // ---------------------------------------------------
        case State::Backoff:
        {
            double currentPos = drive->getActualPositionRaw();
            double remaining = std::abs(m_backoffTarget - currentPos);

            if (remaining > BACKOFF_TOLERANCE_MM)
            {
                double step = homingStepMm();
                double dir = (m_backoffTarget > currentPos) ? 1.0 : -1.0;
                // Limit step to remaining distance to prevent overshoot
                double moveAmt = std::min(step, remaining);
                double nextPos = currentPos + dir * moveAmt;
                drive->setTargetPositionRaw(nextPos);
            }
            else
            {
                // Within tolerance -- backoff complete
                drive->setTargetPositionRaw(m_backoffTarget);

                // Record the home offset: raw position at the backoff point
                // After this, position 0.0 in offset coordinates = this physical location
                m_homeOffset = drive->getActualPositionRaw();

                LOG_INFO(strf("HomingSequence[%d]: Backoff complete. pos=%.3f (error=%.4f%s) "
                    "homeOffset=%.3f",
                    drive->getSlaveIndex(), currentPos, remaining, m_unit, m_homeOffset));

                LOG_INFO(strf("HomingSequence[%d]: COMPLETE. No mode switch required -- "
                    "CSP mode active from init. total_cycles=%d elapsed=%.2fs",
                    drive->getSlaveIndex(), m_cycleCount, m_elapsed));

                m_state = State::Complete;
            }

            if (m_elapsed > BACKOFF_TIMEOUT_SEC)
            {
                LOG_ERROR(strf("HomingSequence[%d]: Backoff TIMEOUT. pos=%.3f target=%.3f remaining=%.4f%s",
                    drive->getSlaveIndex(), currentPos, m_backoffTarget, remaining, m_unit));
                m_state = State::FatalError;
            }
            break;
        }

        // ---------------------------------------------------
        // Seated (deinit seat mode terminal): keep holding ON the hardstop at zero
        // following error until the caller de-energizes. No back-off, no home offset,
        // no homed flag -- intentionally not a homing reference.
        // ---------------------------------------------------
        case State::Seated:
        {
            drive->setTargetPositionRaw(drive->getActualPositionRaw());
            break;
        }

        default:
            break;
        }

        return m_state;
    }

    State       getState()       const { return m_state; }
    bool        isComplete()     const { return m_state == State::Complete; }
    bool        isError()        const { return m_state == State::Error; }
    bool        isFatalError()   const { return m_state == State::FatalError; }
    double      getHardstopPos() const { return m_hardstopPos; }
    double      getHomeOffset()  const { return m_homeOffset; }
    // Frame sign for A6Drive::setHomeOffset(): opposite of the search
    // direction, so engineering increases away from the homed stop.
    double      getFrameSign()   const { return -m_dirSign; }
    int         getCycleCount()  const { return m_cycleCount; }

    static std::string stateName(State s)
    {
        switch (s)
        {
        case State::Idle:             return "IDLE";
        case State::EnablingDrive:    return "ENABLING";
        case State::SettlingDrive:    return "SETTLING";
        case State::CSPTorqueSearch:  return "TORQUE_SEARCH";
        case State::Backoff:          return "BACKOFF";
        case State::Complete:         return "COMPLETE";
        case State::Error:            return "ERROR";
        case State::FatalError:       return "FATAL_ERROR";
        case State::Seated:           return "SEATED";
        default:                      return "UNKNOWN";
        }
    }

private:
    State   m_state = State::Idle;
    double  m_cycleTimeSec = 0.001;
    double  m_backoffMm = 1.5;
    double  m_speedMmS = 5.0;      // NOT true mm/s -- a per-cycle step multiplier (see homingStepMm)
    const char* m_unit = "mm";     // display unit for logs ("deg" on rotary levers)
    double  m_strokeMm = 100.0;    // configured stroke, for timeout diagnostics
    int     m_torquePct = 25;
    std::string m_homeDir = "negative";
    double  m_dirSign = -1.0;
    bool    m_seatMode = false;   // deinit seat: hold on hardstop instead of back-off (see start())
    double  m_elapsed = 0.0;
    int     m_cycleCount = 0;
    double  m_startPos = 0.0;
    double  m_hardstopPos = 0.0;
    double  m_backoffTarget = 0.0;
    double  m_homeOffset = 0.0;     // recorded at backoff completion
    double  m_peakTorque = 0.0;
    double  m_baselineTorque  = 0.0;  // settled SIGNED holding torque (driver weight); hardstop = deviation from this
    double  m_settleTorqueSum = 0.0;  // accumulator over SettlingDrive for the baseline mean
    int     m_torqueConfirmCount = 0;
    double  m_strokeLimitMm = 150.0;  // set in configure() as strokeMm * 1.5
    int     m_settleCount   = 0;      // cycles spent in SettlingDrive
    double  m_lastCommandedPos   = 0.0;   // spike guard -- last raw target sent to drive
    bool    m_lastCommandedValid = false; // false until first commanded position is set

    // Homing step shaping (see homingStepMm()).
    // REF_DT fixes the reference period (0.0005 s = 2 kHz) so the step is
    // independent of the live loop rate. SPEED_SCALE is the empirical
    // true-mm/s calibration (trim on the rig). MAX_STEP_MM is a
    // structural cap on per-cycle following error so Er87 cannot be provoked.
    static constexpr double HOMING_REF_DT_SEC   = 0.0005;
    static constexpr double HOMING_SPEED_SCALE  = 1.0;
    static constexpr double HOMING_MAX_STEP_MM  = 2.0;

    // Timeouts
    static constexpr double ENABLE_TIMEOUT_SEC = 10.0;
    // Wall-clock, and deliberately NOT derived from stroke or homingSpeed: the
    // speed setting is a per-cycle step multiplier, not true mm/s (homingStepMm),
    // so any "expected traverse time" computed from it would be fiction. 60s
    // covers a long axis at a slow setting; the distance guard above is what
    // bounds travel, and it is checked first, so raising this costs waiting
    // time on a broken axis -- never extra movement.
    static constexpr double HOMING_TIMEOUT_SEC = 60.0;
    static constexpr double BACKOFF_TIMEOUT_SEC = 10.0;

    // Backoff completion tolerance -- 50um is realistic for servo settling
    static constexpr double BACKOFF_TOLERANCE_MM = 0.05;

    // Torque confirmation: 50 consecutive cycles above threshold (50ms at 1kHz)
    static constexpr int TORQUE_CONFIRM_CYCLES = 50;

    // Spike guard multiplier -- maximum allowed deviation from last
    // commanded position expressed as a multiple of one cycle's movement.
    // 20× at 30mm/s 500Hz = 20 × 0.06mm = 1.2mm max.  Any larger jump
    // is treated as a corrupted PDO reading.
    static constexpr double SPIKE_MULTIPLIER = 20.0;

    // Settle cycles after enable before starting torque search.
    // 10 cycles at 1kHz = 10ms -- enough for the position loop to adopt
    // the seeded target without a following error spike on first movement.
    static constexpr int SETTLE_CYCLES = 10;
};
