// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// Config.h - application + per-axis configuration structures.
//
// Scaling pipeline:
//   countsPerMm = encoderCountsPerRev * reduction / ballscrewPitch
//
//   Computed at config load and any time the dialog mutates either
//   field. A6Drive consumes countsPerMm directly via the single-arg
//   setScaling(double countsPerMm) - one scaling pipeline, one
//   source of truth.
//
// AppConfig::validate() returns a list of error strings.
// Empty = valid. Called between load() and initialize().
// ============================================================

#include <string>
#include <vector>

struct DriveConfig
{
    int         slaveIndex        = 1;
    std::string name              = "";

    // Drive mode set during init (csp, pp, torque)
    std::string mode              = "csp";

    // Axis type
    std::string axisType          = "linear_vertical";

    bool        invertDir         = false;

    // Linear actuator geometry
    double  strokeMm          = 100.0;
    double  ballscrewPitch    = 10.0;        // mm per motor rev

    // Encoder counts per motor revolution (A6/AS715N family = 131072).
    // Exposed as an Advanced field so future drives with different
    // encoder resolutions can be supported without code changes.
    double  encoderCountsPerRev = 131072.0;

    // Derived scaling: countsPerMm = encoderCountsPerRev / ballscrewPitch.
    // Computed at load() and on dialog edit. Read directly by
    // EtherCATMaster + A6Drive - single source of truth.
    double  countsPerMm       = 13107.2;

    // Belt / rotational
    std::string reductionRatio    = "1:1";

    // Homing
    std::string homeDirection     = "negative";
    // parkMode selects the PARK position only; it has no effect on homing,
    // which always runs the torque search in homeDirection. "center" parks at
    // mid-stroke, anything else parks at homingBackoffMm. (Was "homeMode"
    // until 0.9.2, which read as though it chose how the axis homed.)
    std::string parkMode          = "endstop";
    double  homingBackoffMm   = 1.5;
    // NOT true mm/s: a per-cycle step multiplier applied against a fixed
    // reference dt, so the achieved speed also depends on loop rate and drive
    // gain (see HomingSequence::homingStepMm). Too low a value can exhaust the
    // search timeout on a long axis.
    double  homingSpeed    = 250.0;
    int     homingTorquePct   = 25;

    // Motion limits
    double  maxVelocityMmS    = 200.0;

    // S-curve trajectory parameters (BLENDING planner only; the ONLINE follower
    // ignores jerk). Default jerk satisfies the reversal-time inequality
    // jmax >= 2*amax^2/vmax for the default envelope (2*2000^2/200 = 40000).
    double  maxAccelerationMmS2 = 10000.0;
    double  maxJerkMmS3         = 60000.0;

    // Drive-side following-error window (0x6065), PER AXIS. Written to the drive
    // over SDO during SafeOp (counts = followingErrorWindowMm * countsPerMm).
    // A per-axis drive characteristic, so it travels with the rig. 0 disables.
    double  followingErrorWindowMm = 100.0;

    // ONLINE CSP tracking filter (critically-damped 2nd-order follower).
    // trackingWnHz is the corner frequency AND the only feel knob (smoothness vs
    // responsiveness); wn = 2*pi*trackingWnHz, group delay ~= 2/wn (~10.6ms at
    // 30Hz). The no-overshoot guarantee comes from the braking-aware velocity
    // clamp derived from maxAccelerationMmS2, not a jerk limit -- so maxJerkMmS3
    // is NOT wired to this filter (it still drives the BLENDING s-curve planner).
    double  trackingWnHz             = 30.0;

    // Soft start/stop
    double  unparkTimeSec     = 3.0;
    double  parkTimeSec       = 3.0;

    // ONLINE stale-telemetry response: total seconds to hold before parking.
    // Drop < ~2s is ridden out in place; beyond that the axis eases to center
    // (level standby); at this timeout it parks. Resumes ONLINE on any new frame.
    double  onlineHoldTimeoutSec = 15.0;

    // Filtering
    bool    spikeFilterEnabled= false;
    double  spikeMaxMm        = 5.0;

    // Torque mode (CST) - belt tensioner. Values are % of motor RATED torque
    // (100 = rated, up to ~300 = peak). telemetry raw 0..65535 maps linearly to
    // [torqueMinPct, torqueMaxPct]; the belt never drops below torqueMinPct while
    // tracking. The drive's 0x6072 (max torque) is the HARD ceiling above these.
    double torqueMinPct = 5.0;
    double torqueMaxPct = 50.0;

    // Belt guards (torque mode only). NOTE all values are MOTOR-side: with a
    // reduction ratio R the strap sees torque x R and the motor spins R x faster
    // for the same strap speed, so revisit these (and torqueMax) when R changes.
    //  - Slew: safety envelope on d(tension)/dt, NOT an effect shaper. Haptics need
    //    ~2pi*f*A %/s (30Hz +-5% ~ 940); 3000 passes everything the telemetry source can send while
    //    stretching a single garbage frame's 0->300% step over ~100ms.
    //  - Overspeed: sustained shaft speed = lost load (snapped/detached belt). Trip =
    //    |rpm| > beltOverspeedRpm continuously for beltOverspeedMs -> torque 0 +
    //    latched slack (PARKED; ONLY an explicit TensionBelts re-tensions --
    //    unpark never touches belts). Transient haptic flicks and hand pulls
    //    reset the timer; they never persist.
    //  - Relaxer: sustained near-max dwell guard (force/thermal axis). Tension >=
    //    beltRelaxerPct% of torqueMaxPct for beltRelaxerSec -> ease to torqueMin
    //    until demand drops below (pct-10)%. 0 sec = disabled. Also pre-empts the
    //    drive's i2t overload fault (which would park the whole rig).
    //  - Travel cap: cumulative NET winding since tension-up. An rpm threshold can
    //    miss a slow continuous spin (bare shaft at min tension idles below any sane
    //    rpm limit); net travel can't -- with a belt attached the strap bounds net
    //    winding to ~1 rev, so >beltMaxTravelRevs motor revs = load lost at ANY speed.
    //    Reference re-seeds on every tension-up/unpark.
    //  - Max rpm (MASTER-SIDE velocity fold, beltVelocityFold): caps the
    //    slack-take-up lunge. A momentary slack (lean/shift/pull-release) makes the
    //    motor a free shaft at commanded tension. Commanded tension folds linearly
    //    to zero across a band above beltMaxRpm, reacting one RT cycle after the
    //    knee -- speed tops out just above it (800rpm = 1.26 m/s strap on a 30mm
    //    barrel, ample take-up). A snapped belt sits at the knee until the
    //    overspeed guard's persistence trips latched slack.
    //
    // The drive fault 0x8400 (panel Er06.0) is the drive's RUNAWAY
    // PROTECTION, not overspeed: a CST belt is by design dragged by the
    // driver against its torque, which the check reads as
    // motion-inconsistent-with-torque (manual p181). Remedied by C06.20=0 on
    // torque drives -- a ONE-TIME provisioning/panel write (see
    // drive_profiles torqueOnly), NOT app-managed. The drive-side speed objects
    // (0x607F, C03.47/48) do not restrain CST on the A6 (C03.47/48 are
    // local-mode-only per manual p239); the fold is the enforced speed limit.
    double beltSlewPctPerSec = 3000.0;
    double beltOverspeedRpm  = 600.0;
    double beltOverspeedMs   = 200.0;
    double beltMaxTravelRevs = 3.0;     // 0 = disabled
    double beltMaxRpm        = 800.0;   // fold knee; 0 = fold off (NOT recommended)
    double beltRelaxerSec    = 0.0;     // 0 = disabled
    double beltRelaxerPct    = 80.0;

    // NOTE: drive-side safety limits (max torque 0x6072, position 0x607D,
    // following-error 0x6065) are intentionally NOT app-managed. They are set
    // once on the drive itself. The app's protection comes from the drive's
    // own following-error fault, software stroke clamping, and the motion
    // limits above.
};

struct AppConfig
{
    // Schema version: 2 = split host.json (per-machine) + rig.json
    // (portable rig: global + per-axis). The only schema this build reads.
    int         configVersion  = 2;

    std::string nicName        = "";
    int         controlLoopHz  = 500;
    int         numDrives      = 1;

    int     telemetryPort     = 4444;
    // host: UDP bind address for telemetry input. "127.0.0.1" for the
    // same-PC case (loopback); "0.0.0.0" / link-NIC IP for a dedicated NUC
    // fed by a separate game PC. Per-machine, so it lives in host.json.
    std::string telemetryBindAddr = "127.0.0.1";

    double  blendTimeSec          = 2.0;
    double  blendMaxVelocityMmS   = 20.0;
    int     dcSyncOffsetNs        = 0;

    // DC phase-lock compensator. Default OFF - when disabled the loop and
    // pump free-run exactly as before (byte-identical). When enabled, a
    // gentle clamped PI trims the per-cycle period to lock the sampling phase to
    // the DC reference clock, eliminating the ppm-drift walk that slides the
    // frame onto the SYNC0 boundary and faults the drive on long holds.
    // Self-discovers the per-system drift (any magnitude/sign) and re-tracks it
    // with temperature; gains are loop-rate-normalized so one set is valid on
    // every Pi/PC and at any controlLoopHz. Per-machine RT tuning -> host.json.
    bool    dcPhaseLockEnabled    = false;
    double  dcPhaseLockKp         = 2.5;     // PI proportional (damping)
    double  dcPhaseLockKi         = 1.6;     // PI integral (~0.2 Hz bandwidth, critically damped)
    int     dcPhaseLockMaxTrimNs  = 10000;   // absolute clamp on period trim, ns (hard safety bound)

    // CSP command-conditioning mode (global; the input quality is shared across all
    // axes). "bypass" = raw target -> guard (lowest latency; for a smooth high-rate
    // host stream). "interpolate" = fill gaps for a low UDP send rate. "filter" =
    // feel-shaping 2nd-order low-pass (aggressive sources / comfort; adds latency).
    // Default bypass on both platforms -- the host already delivers an interpolated
    // stream. trackingWnHz (the knee) only applies in filter mode.
    std::string conditioningMode  = "bypass";
    int     pdoWatchdogMs         = 100;
    // IGBT temperature poll interval, seconds PER DRIVE (0 = disabled).
    // Host setting -- the non-RT SdoWorker round-robins a CoE read of 0x2040:0x31
    // (AS715N power-stage temp) at this cadence. Per-machine, so it lives in host.json.
    // Platform-gated default: OFF on Windows (PC/NUC), where the SDO poll's mailbox
    // traffic destabilises DC sync on the margin-limited RT loop (verified: a 4-drive
    // rig ran 36 min clean with it off vs repeated 0x8700 sync drops with it on).
    // Linux/Pi keeps the 15s telemetry poll -- its RT headroom absorbs the cost.
    // An explicit tempPollSec in host.json still wins on either platform.
#ifdef _WIN32
    double  tempPollSec           = 0.0;
#else
    double  tempPollSec           = 15.0;
#endif

    std::vector<DriveConfig> drives;

    bool    requireUserFaultReset = false;
    // Runs 24+ SDO reads per drive during init -- useful for first-time setup,
    // leave off in normal operation to reduce init time and crash risk.
    bool    enableCapabilityScan  = false;

    // Cycles to hold controlword at 0x07 (SwitchedOn) while syncing
    // target PDO to actual position before transitioning to 0x0F (OperationEnabled).
    int    commandSyncCycles        = 10;

    // Post-OP WKC validation window and pass threshold.
    int    wkcValidationCycles      = 50;
    double wkcValidationThreshold   = 0.9;

    int         webPort        = 8080;
    std::string webBindAddr    = "127.0.0.1";
    // Extra Host-header names the web server accepts (e.g. an mDNS name like
    // "nullcat.local"). Local interface addresses and localhost forms are
    // always accepted; this list only appends. Part of the Host-header
    // allowlist that blocks DNS-rebinding.
    std::vector<std::string> webAllowedHosts;
    // The web server defaults OFF on Windows installs where the Qt UI is
    // the primary control surface. Set to true to expose the HTTP/WebSocket
    // dashboard at webBindAddr:webPort. Not exposed in the Qt UI; edit
    // config.json directly to enable.
    bool        webUIEnabled   = false;

    std::string logFile        = "logs/app.log";
    bool    logToConsole   = true;
    bool    simulationMode = false;

    // Log-level + DIAG enable toggles.
    std::string logMinLevel    = "debug";   // debug | info | warning | error | critical
    bool        diagEnabled    = true;

    // Foreground keeper.
    bool foregroundKeeperEnabled = true;
    int  foregroundKeeperAlpha   = 255;   // 1..255 (clamped)
    int  foregroundKeeperX       = 0;
    int  foregroundKeeperY       = 0;

    // GPIO control panel (Pi appliance). Pins are BCM line numbers on gpioChip.
    // Consumed only by the Pi build (GpioPanel); ignored on the PC build.
    // gpioMode selects which components are wired/used:
    //   "off"        - panel disabled
    //   "estop"      - E-STOP input only (no LEDs, no buttons)
    //   "estop_led"  - E-STOP + 3 status LEDs
    //   "full"       - E-STOP + ENGAGE/PARK buttons + 3 LEDs
    // gpioEnabled is legacy/back-compat: a config with gpioEnabled=true and no
    // gpioMode is migrated to "full" on load.
    std::string gpioMode        = "off";
    bool        gpioEnabled     = false;
    std::string gpioChip        = "gpiochip0";
    int         gpioEstopPin    = 17;   // input,  NC mushroom (open = e-stop)
    int         gpioEngagePin   = 27;   // input,  momentary
    int         gpioParkPin     = 22;   // input,  momentary
    int         gpioLedRunPin   = 23;   // output, green
    int         gpioLedReadyPin = 24;   // output, amber
    int         gpioLedFaultPin = 25;   // output, red

    // Physical plausibility checks.
    // Returns empty vector if valid; otherwise one string per error.
    std::vector<std::string> validate() const;
};

class Config
{
public:
    Config();

    // anchorPath is the legacy config.json path; its DIRECTORY holds the
    // split files host.json + rig.json. load() reads the split pair, migrating
    // a legacy config.json or writing fresh defaults (cold start) so the dir
    // always ends up with a valid two-file set.
    bool load(const std::string& anchorPath);
    // Writes BOTH files (transitional convenience). Single-writer callers
    // should prefer saveHost()/saveRig() so each file has exactly one writer
    // per platform (Qt owns host on PC; the web owns rig on both) - that is
    // what makes the cross-surface write-race structurally impossible.
    bool save(const std::string& anchorPath) const;
    bool saveHost(const std::string& anchorPath) const;   // host.json only
    bool saveRig(const std::string& anchorPath) const;    // rig.json only

    const AppConfig& get() const { return m_config; }
    AppConfig& get()             { return m_config; }

    std::string lastError() const { return m_lastError; }

    // Recompute derived fields (countsPerMm) for every drive.
    // Called by load() automatically; the dialog also calls this after
    // mutating ballscrewPitch / encoderCountsPerRev so downstream code
    // (EtherCATMaster, MotionController) reads a consistent value.
    static void recomputeDerivedFields(DriveConfig& d);

    // Validate a proposed rig.json / host.json body (string) against the OTHER
    // namespace already on disk, using the unified validator. The dir is taken
    // from anchorPath. Returns one string per error; empty = valid. Writes
    // nothing - used by the web /api/rig and /api/host endpoints before save.
    static std::vector<std::string> validateRigBody(const std::string& anchorPath,
                                                    const std::string& rigJsonBody);
    static std::vector<std::string> validateHostBody(const std::string& anchorPath,
                                                     const std::string& hostJsonBody);

    // Button-binding contract.
    // The bindable set is enforced SERVER-SIDE: a crafted POST can never bind
    // restart/shutdown/estop-release, regardless of what any UI offers.
    static bool isBindableCommand(const std::string& cmd);
    // Validate a proposed buttons.json body: {"configVersion":1,"bindings":
    // [{"cmd":"belts/tension","vendor":"16c0","product":"05e1","code":288,
    //   "label":"..."}]}. Returns one string per error; empty = valid.
    static std::vector<std::string> validateButtonsBody(const std::string& body);

private:
    AppConfig   m_config;
    std::string m_lastError;

    void setDefaults();
    void seedDefaultDrive();
};
