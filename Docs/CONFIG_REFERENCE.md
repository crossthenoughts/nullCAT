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
| `gpioEngagePin` | int | `27` | |
| `gpioParkPin` | int | `22` | |
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

### axes[] (one object per drive)

| Field | Type | Default | Notes |
|---|---|---|---|
| `slaveIndex` | int | position | 1-based EtherCAT bus position. |
| `name` | string | `"Drive N"` | Label for UI and logs. |
| `mode` | string | `"csp"` | DS402 mode at init. `csp`: cyclic sync position, strict tracking. `pp`: the drive's internal profile generator hunts the target, a softer motion character. `torque`: CST, for belt tensioners. (`cst` in old configs is normalised to `torque`.) |
| `axisType` | string | `"linear_vertical"` | `linear_vertical`, `linear_center`, `belt`, `rotational_*`. |
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
| `maxJerkMmS3` | double | `60000.0` | Retired, ignored. Loaded and saved for round-trip compatibility only; its consumer (the blending s-curve planner's jerk stage) no longer constrains anything. Safe to leave at any value. |
| `followingErrorWindowMm` | double | `100.0` | Per-axis drive-side following-error window, written to object 0x6065 during init (counts = mm x countsPerMm). 100 mm is deliberately wide to tolerate PDO noise without false trips. |
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
- **Values:** comma-separated **decimal** numbers (`32767`, `1.5`, `-0.3`,
  scientific notation all fine - C `strtod` rules). Bare hexadecimal like
  `7FFF` does NOT parse (a leading digit is read as decimal, letters are
  garbage) - configure the sender for decimal output. 16-bit unsigned
  center-at-32767 is the tested scaling (SimHub's *Decimal (string)* /
  16-bit mode).
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

## Tips

- The reference JSONs ship next to the executable, so you can check every
  default on the target machine without the repo.
- After changing config from the web UI, the header shows a
  pending-restart pill until the service restarts; saves do not hot-apply
  (buttons.json is the exception).
- To minimise RT-thread logging cost while keeping error visibility:
  `"logMinLevel": "warning"`, `"diagEnabled": false`.
