# Config reference

nullCAT reads two config files from the directory next to the executable,
plus an optional third for button bindings:

- **host.json**: per-machine settings (NIC, network, web server, logging,
  GPIO panel). Travels with the controlling computer, not the rig. On the
  Windows/Qt build the desktop app owns this file and the web UI shows it
  read-only; on a headless build the web UI owns it.
- **rig.json**: portable rig config. Mechanics, geometry, motion feel and
  per-axis tuning. Safe to copy between a PC and a Pi. Owned by the web UI
  on both platforms.
- **buttons.json**: HID button bindings. Web-owned on both platforms,
  hot-applied on save (no restart).

Reference copies with every key at its compiled-in default ship as
`resources/host.reference.json` and `resources/rig.reference.json`. Keys
prefixed with `_` are documentation and are ignored by the parser. Missing
keys fall back to compiled-in defaults, so a config file only needs the
values you changed. Saves are merge-on-save: hand-edited keys and unknown
fields survive a save from the UI.

There is no other config file: a `config.json` next to the executable is
ignored (the flat-file migration from pre-release builds was retired).

## host.json

### Network, telemetry, web

| Field | Type | Default | Notes |
|---|---|---|---|
| `nicName` | string | `""` | EtherCAT adapter (device path on Linux, e.g. `eth0`; adapter description on Windows). Empty = auto-detect the first non-loopback adapter. Use a dedicated NIC for the EtherCAT segment. The Pi installer seeds `eth0` (the onboard port). |
| `controlLoopHz` | int | `500` | RT loop rate, clamped 100 to 4000. The Pi reference platform runs 2000. On Windows, 500 is the validated rate; 1000 was unstable in testing. |
| `tempPollSec` | double | Linux `15.0`, Windows `0.0` | Drive IGBT temperature poll period for the web drive cards. `0` disables. Polled over SDO in OP, round-robin, one drive at a time. |
| `telemetryPort` | int | `4444` | UDP port for telemetry input. |
| `telemetryBindAddr` | string | `"127.0.0.1"` | Telemetry bind address. The default suits the common Windows case (telemetry app on the same PC). A dedicated NUC sets its LAN IP (or `0.0.0.0`) so the game PC can reach it -- see NUC_DEPLOYMENT.md. On Linux/Pi a loopback value is auto-corrected to all-interfaces with a warning (the game PC is always a separate machine there); empty or `0.0.0.0` = platform default. |
| `webPort` | int | `8080` | HTTP + WebSocket port (honored when `webUIEnabled` is true). |
| `webBindAddr` | string | `"127.0.0.1"` | `"0.0.0.0"` allows access from other machines on the LAN. The Pi installer seeds `0.0.0.0` into host.json (a headless controller is browsed from another machine); the compiled default suits the same-PC Windows case. |
| `webAllowedHosts` | string[] | `[]` | Extra Host-header names the web server accepts, e.g. a router-assigned DNS alias. Localhost forms, the machine's own hostname (bare and `.local`), the bind address, and all local interface addresses are always accepted; this list only appends. Part of the fail-closed Host allowlist that blocks DNS rebinding (see KNOWN_LIMITATIONS, security section). |
| `webUIEnabled` | bool | `false` | Off by default on Windows installs where the Qt UI is the primary control surface; the headless build runs with it on. On the Qt build, toggle it with the main window's web on/off button (which persists the choice here); it is not in the Settings dialog. |
| `webShowDevices` | bool | `false` | Shows the Devices section in the web UI (force devices: shifter, active pedal) and adds the device axis types to the axis editor. The tickbox lives under Advanced in the web config; Pi builds only (the row is hidden elsewhere). Purely a UI switch: device axes in rig.json work regardless. |
| `shakersEnabled` | bool | `false` | Drive bass shakers from a USB sound card: opens the device(s) below at start and keeps the haptics layer stepping while the control loop is stopped (so shakers work with no drives). Read once at startup. |
| `audioDevice` | string | `""` | The playback device for shakers: part of its name (case does not matter), `""` for the system default, `"null"` for no hardware (the null backend). Device 1 when `audioDevices` is absent. |
| `audioChannels` | int | `2` | How many of that device's outputs are shakers, 1 to 8. |
| `audioDevices` | array | absent | More than one card: `[{"device": "<name>", "channels": 2}, ...]`, shaker numbers running across the devices in order, 8 in total at most. Replaces the two fields above when present. |

### EtherCAT timing and init

| Field | Type | Default | Notes |
|---|---|---|---|
| `dcSyncOffsetNs` | int | `0` | DC SYNC0 offset relative to cycle start, ns. Platform practice: Pi runs `0`; Windows builds run `500000` (500 us) to ride out scheduler jitter. |
| `dcPhaseLockEnabled` | bool | `false` | DC phase-lock compensator. A gentle clamped PI that trims the loop period to hold the DC sampling phase constant. Deliberately dormant: shipped off, present for future need. Do not enable without validating on your rig. |
| `dcPhaseLockKp` | double | `2.5` | Phase-lock proportional gain (only read when enabled). |
| `dcPhaseLockKi` | double | `1.6` | Phase-lock integral gain. |
| `dcPhaseLockMaxTrimNs` | int | `10000` | Per-cycle period trim clamp, ns. |
| `pdoWatchdogMs` | int | `100` | Per-slave PDO watchdog timeout, ms. The drives de-energize themselves if frames stop for this long. |
| `enableCapabilityScan` | bool | `false` | Runs ~24 SDO reads per drive during init. Useful on first bring-up, otherwise leave off. |
| `commandSyncCycles` | int | `10` | Cycles to hold controlword 0x07 while syncing target to actual position before the 0x0F (Operation Enabled) transition. |
| `sync0RecycleRounds` | int | `2` | Wedged-SYNC0 recovery during init: drives whose pulse unit fails to start after a cold power-on are cycled PreOP / disarm / re-arm / SafeOP and re-checked, up to this many rounds (about half a second per affected drive, narrated in the log). `0` disables; a failed init then advises retrying Initialize manually. Range 0 to 5. |
| `wkcValidationCycles` | int | `50` | Post-OP working-counter validation window, cycles. |
| `wkcValidationThreshold` | double | `0.9` | Fraction of the window that must return the expected WKC. |

### Logging and runtime

| Field | Type | Default | Notes |
|---|---|---|---|
| `logFile` | string | `"logs/app.log"` | Main log path. The SOEM diag stream goes to `<name>_soem.log` alongside. |
| `logToConsole` | bool | `true` | Mirror log output to stdout/stderr. |
| `logMinLevel` | string | `"debug"` | Suppress entries below this level: `debug`, `info`, `warning`, `error`, `critical` (case-insensitive). |
| `diagEnabled` | bool | `true` | Gates the DIAG stream (RTT, DC phase, command fingerprints). On the RT thread DIAG lines go through the lock-free log ring, so the overhead when enabled is small; `false` suppresses them entirely. |
| `simulationMode` | bool | `false` | Run the whole engine with no hardware. Simulated drives obey commands and ramp synthetic actuals. Simulation cannot home (see KNOWN_LIMITATIONS). |

### Foreground keeper (Windows only, ignored on Linux)

Windows 11 throttles processes without a visible foreground window, and
process priority alone does not prevent it. The keeper is a 1x1 topmost
window that keeps the process classified as foreground-active. SimHub and
DRSM use the same technique; each app's keeper coexists fine.

| Field | Type | Default | Notes |
|---|---|---|---|
| `foregroundKeeperEnabled` | bool | `true` | Master switch. |
| `foregroundKeeperAlpha` | int | `255` | Window opacity, 1 to 255. Avoid 0: fully transparent windows can be stripped from the foreground list, defeating the purpose. |
| `foregroundKeeperX` | int | `0` | Pixel position (top-left of primary screen by default, away from the corner SimHub/DRSM typically use). |
| `foregroundKeeperY` | int | `0` | |

### GPIO control panel (Pi only, ignored on Windows)

Physical panel wired to the Pi header: e-stop mushroom, engage/park
buttons, status LEDs. Pins are BCM line numbers on `gpioChip`. The panel
stop is a software stop; the safety device is the hardware latch wired to
the drives (see SAFETY.md).

| Field | Type | Default | Notes |
|---|---|---|---|
| `gpioMode` | string | `"off"` | `off`, `estop` (mushroom only), `estop_led` (mushroom + LEDs), `full` (buttons + LEDs). |
| `gpioChip` | string | `"gpiochip0"` | |
| `gpioEstopPin` | int | `17` | Level-sampled; mushroom should be a latching NC type. |
| `gpioEngagePin` | int | `27` | The RUN button: offline = Initialize (start auto-chains), stopped = Start, running = Stop. `0` = not fitted. |
| `gpioParkPin` | int | `22` | Park/unpark toggle. `0` = not fitted. |
| `gpioBeltPin` | int | `4` | Belt slack/tension toggle (header pin 7, directly above the estop/run/park run at 11/13/15). `0` = not fitted. |
| `gpioDevicePin` | int | `18` | Device home/engage/release toggle (header pin 12). `0` = not fitted. Any subset of buttons may be wired - compact panels just zero the rest. |
| `gpioLedRunPin` | int | `23` | |
| `gpioLedReadyPin` | int | `24` | |
| `gpioLedFaultPin` | int | `25` | |

(`gpioEnabled` is a legacy key: `true` migrates to `gpioMode: "full"` on
load.)

## rig.json

Top level: `configVersion`, `numDrives` (1 to 10, must match `axes[]`),
`global` object, `axes` array.

### global

| Field | Type | Default | Notes |
|---|---|---|---|
| `conditioningMode` | string | `"bypass"` | CSP command conditioning. `bypass`: raw target into the guard chain, lowest latency, right for a smooth high-rate host stream. `interpolate`: first-order gap-fill for a low UDP send rate (latency about one frame). `filter`: 2nd-order low-pass feel shaping for aggressive sources or comfort (latency `2/wn`, see `trackingWnHz`). The guard chain (Vmax, Amax, relative braking, stroke) is always active in every mode. No effect on PP or torque axes. |
| `blendTimeSec` | double | `2.0` | Smoothing time constant for cross-axis blended motion. |
| `blendMaxVelocityMmS` | double | `20.0` | Cap on the blended-axis velocity contribution. |
| `requireUserFaultReset` | bool | `false` | If true, faulted drives need a manual reset from the UI before motion resumes. Safety policy, so it travels with the rig. |
| `ncxBindings` | array | all 36 tokens, slots 0 to 35 | NULLCATX channel bindings for the device and haptic effects. Each entry is `{"token", "slot", "scale", "offset"}`: the wire's numbered channel `slot` (0 to 47) maps onto the semantic `token`, with `value = raw * scale + offset`. Tokens: `rpm`, `speedKmh`, `gear`, `clutchPct` (0 = pedal up, 100 = floored), `throttlePct`, `brakePct`, `absActive`, `skid`, `lockup`, `roadNoise`, `limiter`, `tcActive`, `curbs`, `maxRpm`, and the per-wheel groups `slipAngleFL..RR`, `slipRatioFL..RR`, `wheelSpeedFL..RR`, `loadFL..RR`, `suspVelFL..RR`, `boost` and `pitLimiter` (see Docs/PROTOCOL.md for units). Each token binds at most once. The default matches the SimHub plugin's channel order, so it needs no editing; the Sim channels section of the web Setup view edits it and can reset it. An unbound token leaves the effects that use it silent. Bindings only apply to NULLCATX; a NULLCATY line names its tokens directly in canonical units. |
| `haptics` | object | all effects off | Haptic effect layer tuning, one entry per effect plus `masterGain` and `positionBudget`. See below; normally edited on the web Haptics strip. |

### global.haptics

One object per effect: `detentClick`, `gearShift`, `rpmVibe` (the engine), `abs`, `slipLon` (longitudinal slip), `slipLat` (lateral slip), `road`, `limiter`, `tc`, `kerb`, `driveline`, plus `masterGain` (number, 0 to 2, default `1`, scales every effect) and `positionBudget` (number, 0 to 1, default `0.4`, the share of each position axis's velocity and acceleration limits that haptics may use; see `hapticsMaxMm` below) and `axisDelayMs` (number, 0 to 60, default `0`: holds the axis haptic overlays back by this much so they land together with the slower shaker path; the motion cue is never delayed). Every effect is off until it has an amplitude above 0 and at least one route. Saves apply live, with no re-initialize.

| Field | Type | Default | Notes |
|---|---|---|---|
| `ampPct` | double | `0` | Effect strength at full scale (0 to 100). On a torque axis, % of rated torque; on a position axis, % of the route's gain in mm. 0 = off. |
| `freqHz` | double | per effect | Carrier frequency (4 to 500 Hz). For the engine this is the weight of each firing thump (default 30). |
| `durMs` | double | per effect | Length of a one-shot effect (detent click, gear shift), 5 to 100 ms. |
| `jitter` | double | per effect | 0 to 1. Roughens the carrier so skid, road and kerb feel like texture rather than a tone. For the engine it is the idle lope (per-revolution unevenness). |
| `routes` | array | `[]` | Where the effect goes: `[{"axis": <index>, "gain": <number>, "part": <string>}]` or `[{"shaker": <index>, "gain": <number>, "harm": <int>}]`, up to 14 entries, any axis or shaker channel. A shaker route's `gain` is full scale at 100 % amplitude (1 = full, soft-clipped above), `harm` (1 to 8, default 1) the harmonic of the effect's carrier it plays, phase-locked. On a torque axis (belt, device) `gain` is a multiplier, 0 to 2. On a position (CSP) axis `gain` is the offset in mm at 100 % amplitude, 0 to that axis's `hapticsMaxMm`. `part` (the two slip effects and road only, default `"all"`): which wheels this axis carries, `all`, `front`, `rear`, `fl`, `fr`, `rl`, `rr`; the strongest of them plays. No routes = the effect reaches nothing. |

Slip-only fields. Both tiles are per-wheel models with two components, each with a mix (0 to 1) and a carrier; the route's `part` picks the wheels. `freqHz` is kept in the object but unused by these two.

| Effect | Field | Default | Notes |
|---|---|---|---|
| `slipLat` | `scrub`, `scrubHz` | `1`, `25` | The fronts pushing wide: mix and carrier (8 to 60 Hz). |
| `slipLat` | `slide`, `slideHz` | `1`, `11` | The rears stepping out: mix and carrier (4 to 30 Hz). |
| `slipLat` | `peakDeg` | `7` | Slip angle (degrees) at full severity, 2 to 20. Nothing plays below 60 % of it. |
| `slipLon` | `lock`, `lockHz` | `1`, `9` | A wheel locking under braking: mix and carrier (4 to 30 Hz) at 80 km/h; the carrier follows road speed. |
| `slipLon` | `spin`, `spinHz` | `1`, `10` | A driven wheel spinning: mix and carrier (4 to 30 Hz), fixed. |
| `slipLon` | `peakRatio` | `0.8` | Slip ratio at full severity, 0.2 to 2. Nothing plays inside 0.15. |

Road-only fields, used when the sim sends `suspVelFL..RR` (the tile then replays each corner's suspension travel instead of a texture; `freqHz` and `jitter` apply to the texture fallback only):

| Field | Default | Notes |
|---|---|---|
| `fullMm` | `8` | Suspension travel (mm) that is 100 % amplitude, 0.5 to 50. |
| `hpHz` | `2` | High-pass on the travel (Hz), 0.5 to 10: removes the slow body motion the motion cue already produces, leaving the bumps. |

Driveline-only fields (`driveline`):

| Field | Default | Notes |
|---|---|---|
| `clutch`, `clutchHz` | `1`, `10` | Clutch judder: mix (0 to 1) and carrier (4 to 20 Hz). Plays while the clutch pedal is between 15 % and 85 % with slip across it (engine rpm against the learned ratio for the gear times road speed; a launch under 5 km/h is full slip), scaled by throttle. |
| `lug`, `lugHz` | `1`, `7` | Lugging wind-up: mix (0 to 1) and carrier (3 to 15 Hz). Plays in gear at 50 % throttle or more between 12 % and 30 % of the redline, full at the low end. |
| `gearbox` | `0` | 0 synchro, 1 dog. Shapes the gear-shift thunk (synchro: 0.75 x, 1.4 x longer, lower; dog: full, 0.7 x shorter, higher, plus an engagement knock 60 ms after a shift at 60 % throttle or more). |
| `whine` | `0` | Straight-cut gear whine mix (0 to 1): pitch from rpm at the engine buzz's auto order stepping 6 % per gear, level 0.3 + 0.7 x throttle, in gear with the clutch driving. |
| `shunt`, `shuntHz` | `0.5`, `40` | Backlash take-up knock (mix 0 to 1, carrier 20 to 80 Hz) when the throttle crosses 5 % and 20 % within 150 ms, rolling in gear; 1.4 x in a dog box, 0.8 x on a lift. |

A 0.9.6 file that still carries these two effects as `skid` and `lockup` is read through those keys once (amplitude, routes, jitter and the old carrier onto `scrubHz` / `lockHz`) and rewritten under the new keys on the next load.

Engine-only fields (`rpmVibe`):

| Field | Default | Notes |
|---|---|---|
| `cylinders` | `4` | Cylinders, or rotors when `layout` is Wankel (1 to 16). Sets how often it fires. |
| `litres` | `2` | Total displacement (0.1 to 30). Litres per cylinder set how hard each firing hits. |
| `layout` | `0` | 0 inline, 1 V, 2 flat/boxer, 3 Wankel, 4 two-stroke, 5 electric. Sets how much the block shakes and how the engine fires (two-stroke: every cylinder every rev; electric: no firing, a whine). |
| `maxRpm` | `0` | Redline. 0 = learned while you drive (highest rpm seen, set exactly on the first limiter hit). |
| `rock`, `thump`, `buzz` | `1` | Mix of the three parts (0 to 1): the idle rock, the firing thumps, the rpm-following buzz. |
| `order` | `0` | Buzz pitch as a multiple of crank speed (0.25 to 8). 0 = automatic, so the redline lands at the top of the range. |
| `limHit` | `1` | Limiter: strength of each return hit (0 to 2). |
| `limHz` | `12` | Limiter: cut rate (4 to 30 Hz). |
| `limJit` | `0` | Limiter: irregularity of the cut timing (0 to 1). |
| `inertia` | `0.5` | Mix of the rpm-squared reciprocating shake (0 to 1): load-independent, what remains on a lift. |
| `turbo` | `0` | 0 no, 1 yes. Shapes the lift-off event: a pop, or the blow-off whoosh and compressor flutter. |
| `liftoff` | `1` | Lift-off event strength (0 to 2), scaled by `boost` when the sim sends it. |
| `pops` | `0` | Overrun pops (0 to 1): fuel-cut crackle on a shut throttle above 30 % of the redline. |

### axes[] (one object per drive)

| Field | Type | Default | Notes |
|---|---|---|---|
| `slaveIndex` | int | position | 1-based EtherCAT bus position. |
| `name` | string | `"Drive N"` | Label for UI and logs. |
| `mode` | string | `"csp"` | DS402 mode at init. `csp`: cyclic sync position, strict tracking. `pp`: the drive's internal profile generator hunts the target, a softer motion character. `torque`: CST, for belt tensioners and device axes (shifter/pedal require it). (`cst` in old configs is normalised to `torque`.) |
| `axisType` | string | `"linear_vertical"` | `linear_vertical`, `linear_horizontal`, `rotary_lever`, `belt`, `shifter`, `pedal`. The device types (`shifter`, `pedal`) are force-feedback devices: torque mode only, homed by a gentle stall search against a travel stop, engaged/released via their own commands, never touched by belt or unpark commands. Their feel lives in the nested `device` object below. For `rotary_lever` the engineering unit is DEGREES at the lever shaft: `strokeMm` = arc travel in degrees, velocities/accels in deg/s and deg/s2, `homingBackoffMm` in degrees. Internally `ballscrewPitch` is pinned to 360 (360 units per output rev - the web editor forces and hides it) and `reductionRatio` carries the gearbox (e.g. `"50:1"`), giving counts/deg = encoderCountsPerRev x ratio / 360. Motion path, homing, guards, and commissioning are unit-agnostic and behave identically. |
| `invertDir` | bool | `false` | The axis's mechanical polarity: tick (true) for a foldback linkage, leave off for an inline actuator. Determines which motor direction is "retract", so it sets the homing search direction AND the telemetry response together (one mechanical reversal flips both). If a new axis homes toward the wrong end, this is the setting to flip. Since 0.9.2; before that it only reversed the telemetry response. |
| `strokeMm` | double | `100.0` | Usable travel (linear types). Homing's stroke guard trips at 1.5x this if the hardstop is never found. |
| `ballscrewPitch` | double | `10.0` | mm per motor revolution (linear types). |
| `encoderCountsPerRev` | double | `131072.0` | Encoder resolution (A6 family: 131072). Change only for a different drive family. |
| `countsPerMm` | double | derived | `encoderCountsPerRev * reduction / ballscrewPitch`, recomputed at load and on edit. Single source of scaling truth. Do not set by hand. |
| `reductionRatio` | string | `"1:1"` | Gear or belt reduction (rotational and belt types). |
| `homeDirection` | string | `"negative"` | Which stop to home to, in travel terms: `negative` (default) homes to the **retracted** stop, `positive` to the **extended** stop (the rare park-extended case, e.g. an inline actuator you want resting extended). The raw motor direction is derived from this together with `invertDir` -- this field is no longer a raw direction. After homing, position always counts away from whichever stop was found, so a misconfigured axis can home to the unintended end but can never be commanded through it. |
| `parkMode` | string | `"endstop"` | `endstop` (park at the backoff point after homing - verticals, where gravity holds the axis at the bottom) or `center` (park at mid-stroke - the natural rest for a horizontal axis). Both run the same torque-based endstop search; this only sets where the axis parks. Any other value behaves as `endstop`. Named `homeMode` before 0.9.2, which read as though it chose how the axis homed; the old key is still accepted. |
| `homingBackoffMm` | double | `1.5` | Back-off from the hardstop after detection. |
| `homingSpeed` | double | `250.0` | Homing approach speed command. **Not mm/s**: a per-cycle step multiplier applied against a fixed reference dt, so the achieved speed also depends on loop rate and drive gain. Too slow faults the search (torque never builds); values below about 10 are not usable on the reference hardware, and a low value on a long axis can exhaust the 60s search timeout. The timeout message reports the measured mm/s. Named `homingSpeedMmS` before 0.9.2; the old key is still accepted. |
| `homingTorquePct` | int | `25` | Torque threshold for hardstop detection, percent of rated. |
| `maxVelocityMmS` | double | `200.0` | Per-axis velocity cap. |
| `maxAccelerationMmS2` | double | `10000.0` | Acceleration cap. Also the source of the tracking filter's braking clamp. |
| `hapticsMaxMm` | double | `3.0` | Position (CSP) axes: the largest offset a haptic effect may add to the axis's position, 0 to 10 mm. `0` = this axis takes no haptics. Effects are additionally held to what the axis can follow at their carrier frequency: the smaller of this cap, `positionBudget x maxVelocityMmS / (2 pi f)` and `positionBudget x maxAccelerationMmS2 / (2 pi f)^2`. The offset rides on the motion cue, which keeps priority, and the sum is still bound by the axis limits. Only while ONLINE; ignored for torque and PP axes. |
| `maxJerkMmS3` | double | `60000.0` | Retired, ignored. Loaded and saved for round-trip compatibility only; its consumer (the blending s-curve planner's jerk stage) no longer constrains anything. Safe to leave at any value. |
| `followingErrorWindowMm` | double | `100.0` | Per-axis drive-side following-error window, written to object 0x6065 during init (counts = mm x countsPerMm). 100 mm is deliberately wide to tolerate PDO noise without false trips; since 0.9.5 the written value is additionally clamped to the axis's own travel (a window wider than the stroke could never trip, disabling the protection), logged when it happens. The config value itself is kept. |
| `trackingWnHz` | double | `30.0` | The one feel knob for the ONLINE tracking filter (critically damped 2nd-order follower, only active in `filter` conditioning mode). Group delay is about `2/wn`: 10.6 ms at 30 Hz. Raise for tighter tracking, lower for more smoothing. Capped at 125 Hz. No overshoot at any amplitude: the guarantee comes from the braking-aware velocity clamp, not from jerk. |
| `unparkTimeSec` | double | `3.0` | Unpark trajectory duration. |
| `parkTimeSec` | double | `3.0` | Park trajectory duration. |
| `onlineHoldTimeoutSec` | double | `15.0` | Staged response to lost telemetry. Short dropouts (under ~2 s) are ridden out in place. Beyond that the axis eases to center, and at this timeout it parks. Any returning frame resumes tracking. Arms only after the first valid frame. |
| `spikeFilterEnabled` | bool | `false` | Reject single-frame jumps larger than `spikeMaxMm`. |
| `spikeMaxMm` | double | `5.0` | |
| `torqueMinPct` | double | `5.0` | Torque axes: floor tension while tracking, percent of rated. |
| `torqueMaxPct` | double | `50.0` | Torque axes: tension at full-scale telemetry. The drive's own 0x6072 limit is the hard ceiling above this. |
| `beltSlewPctPerSec` | double | `3000.0` | Belt guard: safety envelope on tension rate of change. Passes every real haptic effect; stretches a single garbage frame's full-scale step over ~100 ms. |
| `beltOverspeedRpm` | double | `600.0` | Belt guard: sustained shaft speed above this means the load is lost (snapped or detached belt). |
| `beltOverspeedMs` | double | `200.0` | Persistence for the overspeed trip. Transient flicks reset the timer. Trip result: torque zero, latched slack; only an explicit belt tension command re-tensions. |
| `beltMaxTravelRevs` | double | `3.0` | Belt guard: cumulative net winding since tension-up. Catches a slow continuous spin that stays under any rpm threshold. `0` disables. |
| `beltMaxRpm` | double | `800.0` | Master-side velocity fold knee. Commanded tension folds to zero across a band above this speed, capping the slack-take-up lunge. `0` disables the fold (not recommended). This fold is the enforced speed limit on A6 CST axes; the drive-side speed objects do not restrain CST (see DRIVE_TUNING.md). |
| `beltRelaxerSec` | double | `0.0` | Belt guard: sustained near-max dwell. Tension at or above `beltRelaxerPct`% of `torqueMaxPct` for this long eases to minimum until demand drops. Pre-empts the drive's i2t overload fault. `0` disables. |
| `beltRelaxerPct` | double | `80.0` | |

`rotationCount` in old rig files is ignored on load and no longer saved.

### axes[].device (device axes only)

Present only on `shifter`/`pedal` axes: the force feel and the torque
homing. All positions are motor revolutions in the HOME frame (exactly as
configured: neutral, detents, stops); `dir` alone maps a mirrored
mechanical build. Unlike every other setting, the `device` object
LIVE-APPLIES on save the moment that device is limp (staged if engaged,
landing on release) - feel tuning never needs a restart. Curves are `[[x, y], ...]` node arrays sampled
piecewise-linear with ends clamped; `y` is the force RESISTING
displacement at `x` (percent of rated torque). A curve whose first `x` is
negative is sampled as drawn (asymmetric); otherwise it is mirrored about
zero. The web Devices section ships starter presets for the whole object.

| Field | Type | Default | Notes |
|---|---|---|---|
| `dir` | double | `1` | `+1` or `-1`: maps the model's force onto the motor's torque sign (and motor position onto the device frame). Flip for a mirrored build; never edit the geometry for that. |
| `neutralRev` | double | `0.0` | Rest position, revs from home. |
| `springCurve` | nodes | `[]` | Centring force vs displacement from neutral. Empty contributes nothing. |
| `detents` | double[] | `[]` | Detent centre positions (home frame). Only the nearest one acts. |
| `detentCurve` | nodes | `[]` | Force profile relative to a detent centre. Drawn rising through zero it captures; inverted it pushes through (over-centre click). |
| `stopMinRev` / `stopMaxRev` | double | `-0.07` / `0.07` | Soft end stops. Homing lands on the `homeDir` stop and latches the frame there. |
| `stopSpring` | double | `20000` | Stop wall stiffness, %/rev. |
| `stopDamp` | double | `60` | Extra damping inside the stop, %/(rev/s). |
| `lashRev` | double | `0.0` | Free-play band about neutral (worn-linkage feel). |
| `dampPctPerRevS` | double | `15` | Viscous damping everywhere. |
| `frictionPct` | double | `0.0` | Dry (Coulomb) friction opposing motion - the mechanical-linkage feel viscous damping alone cannot give. `0` = off. |
| `breakoutScale` | double | `1.0` | Detent force multiplier while pulling OUT of a slot: into gear easy, out of gear firm. `1` = symmetric. |
| `velLpfHz` | double | `40` | Velocity estimate low-pass. |
| `maxForcePct` | double | `100` | Model output clamp, % of rated. Above it only the drive's own 0x6072 torque limit still caps (`torqueMinPct`/`torqueMaxPct` are belt fields; the device path never reads them). |
| `homeTorquePct` | double | `30` | Homing push, % of rated (5 to 100). Keep low: it presses a mechanism against its own stop. |
| `homeDir` | double | `-1` | Which stop homing pushes toward, in the DEVICE frame: `-1` = `stopMinRev`, `+1` = `stopMaxRev`. |
| `slewPctPerSec` | double | `20000` | Output slew cap. A feel knob at the default; still the safety envelope for a bad curve edit landing mid-session. |
| `thermalDwellSec` | double | `0.0` | Sustained near-ceiling output for this long eases to zero until demand drops (pre-empts drive i2t). `0` disables. |
| `thermalPct` | double | `80` | Fraction of `maxForcePct` that counts as near-ceiling. |
| `foldRpm` | double | `0.0` | Anti-runaway velocity fold knee (same idea as `beltMaxRpm`). `0` disables. |
| `clutchBitePct` | double | `0.0` | State effect (needs a bound `clutchPct` channel): clutch reading below this = the clutch is driving, so moving the lever out of gear blocks and grinds. `0` disables all clutch logic. If the stream stops, every effect drops out and the plain feel remains. |
| `blockGain` | double | `0.0` | Extra force scale while blocked (the whole field stiffens by `1 + blockGain`). `0` = off. |
| `grindAmpPct` | double | `0.0` | Grind texture amplitude while blocked and pushing, % of rated. `0` = off. |
| `grindFreqHz` | double | `33` | Grind texture frequency. |
| `blockStartRev` | double | `0.01` | How far out of the nearest detent the lever must be before block/grind engage (a settled lever in gear is never affected). |
| `rpmMatchPct` | double | `0.0` | Revmatch let-in window, % of the target rpm. With `rpm`, `speedKmh`, and `gear` channels bound, a clutchless shift goes IN (no block, no grind) when the engine speed is already within this window of what the destination gear needs at the current road speed. The per-gear ratios are learned automatically while you drive and remembered per car in `carcache.json` (machine state beside the configs - safe to delete, never hand-edited). `0` disables. |

## Telemetry wire format (UDP input)

What the controller accepts on `telemetryPort`, for anyone integrating motion
software other than SimHub (any tool that can send a custom UDP string
works). One packet per datagram; the parser's exact semantics are pinned
by `TestTelemetryParse`.

```
NULLCAT,<axis1>,<axis2>,...,<axisN>
```

- **Header (required):** everything before the **first comma** must spell
  `NULLCAT` - case-insensitive, embedded whitespace ignored (`null cat ,`
  parses). Anything else, or a packet with no comma, is rejected whole.
- **Values:** comma-separated **decimal** numbers (`32767`, `48000`,
  scientific notation all fine - C `strtod` rules). Bare hexadecimal like
  `7FFF` does NOT parse (a leading digit is read as decimal, letters are
  garbage) - configure the sender for decimal output.
- **Scaling: 16-bit UNSIGNED decimal, and ONLY that.** `0..65535` with
  `32767` = axis centre; full scale spans the axis's configured `strokeMm`
  (so the same wire value moves a 230mm surge proportionally further than
  a 100mm heave). This is SimHub's *Decimal (string)* / 16-bit mode; in
  tools offering a signed/unsigned choice (FlyPT Mover), pick UNSIGNED -
  signed values parse without an error and every axis then sits
  mis-centred against an endstop. Sanity check: an idle rig should be
  sending values near `32767` on every channel. Out-of-range values are
  clamped to the ends of travel. Before 0.9.3 an undocumented
  heuristic guessed "values under 500 are millimetres"; that guess is gone -
  small numbers now mean what 16-bit says they mean (near the low end of
  travel), so a sender emitting raw mm offsets must be reconfigured.
- **All-zeros frames are ignored:** a frame where *every* value is exactly
  `0` is treated as no telemetry (feeding the normal telemetry-loss standby)
  rather than as a command to slam every axis to one end - some telemetry
  tools emit all-zeros while idling at a menu. A frame with any nonzero
  value is a real command, including genuine zeros on individual axes.
- **Axis order:** value 1 feeds your first configured axis (chain order),
  value 2 the second, and so on. At most **10** values are read; extras
  are ignored.
- **Every slot must be present:** an empty or unparseable field is
  *skipped and the remaining values shift down* onto the wrong axes.
  Always send a value for every axis.
- **No lifecycle tokens:** the controller gates motion on its own
  readiness, never on the sender's lifecycle - send only motion lines.
- No timestamp field, no trailing newline required. Telemetry is
  considered lost after ~300 ms without a fresh motion packet; axes hold
  center until it resumes.

### NULLCATX channel packets (device and haptic effects)

A second line format on the SAME port feeds the force-device effects
(shifter/pedal - see the Devices section of the web UI):

    NULLCATX,<ch0>,<ch1>,...,<chN>

- Same parsing rules as motion lines (decimal CSV, case-insensitive
  header, skipped fields compact). At most **48** channels are read.
- The channels are plain numbers in whatever units the sender has; the
  rig's `ncxBindings` config says which channel means what and rescales
  it. Send raw sim values (rpm, speed, gear, clutch) - no shaping on the
  sender side.
- Independent of the motion stream: either runs without the other, at
  its own rate (100 to 400 Hz is plenty). A NULLCATX packet never counts
  as motion telemetry - it cannot hold off the motion-loss standby.
- If the channel stream stops for 500 ms, every state effect drops out
  and devices fall back to their plain configured feel.
- The same channels can be sent by name instead, with no bindings:
  `NULLCATY,rpm=6500,gear=3,slipAngleFL=4.2,...`, plus `game=` and
  `car=` for the sim identity. See Docs/PROTOCOL.md.

## Tips

- The reference JSONs ship next to the executable, so you can check every
  default on the target machine without the repo.
- After changing config from the web UI, the header shows a
  pending-restart pill until the service restarts; saves do not hot-apply
  (buttons.json is the exception).
- To minimise RT-thread logging cost while keeping error visibility:
  `"logMinLevel": "warning"`, `"diagEnabled": false`.
