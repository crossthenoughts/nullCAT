# Changelog

Notable changes to nullCAT. Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/);
versioning is [Semantic Versioning](https://semver.org/) - while on `0.x`, the
middle number carries breaking changes and the last carries fixes.

## [0.9.4] - unreleased

### Fixed
- Rotary axes: the following-error window is now capped to the axis arc
  server-side, not only in the web editor. A hand-edited rig.json can no
  longer disable drive-side runaway protection on a lever.

### Changed
- RT loop hardening: the telemetry drain is bounded per cycle, rejected UDP
  packets no longer allocate on the RT thread, and following-error/dither
  diagnostics no longer run against torque axes (their numbers there were
  noise).

### Internal
- Five logic test suites (axis classification, fault codes, commissioning
  engine and flow, config roundtrip) now also build and run on the Linux CI
  job; the tracking-filter suite gains the -ffp-contract=off flag its
  bit-identity regression requires.

## [0.9.3] - 2026-08-25

### Improved
- **Init reliability, substantially.** Field logs (nine failed inits in a
  row) showed a perfect correlation: every slave the pre-OP SYNC0 guard
  "re-armed" refused OP; every slave left alone reached OP. Two defects:
  register reads were not wkc-checked, so a dropped read produced garbage
  margins and false re-arms of healthy syncs; and after a re-arm the OP
  request went out inside the ~100ms window before the new SYNC0 start,
  so the drive's SafeOp-to-OP sync check could only fail. Reads are now
  verified (never act on garbage), a re-arm is followed by a wait for the
  new start, a verification that the pulse unit actually began advancing,
  and a 700ms settle pump before OP is requested. Field results are much
  better but not yet a guaranteed first-try OP; the new per-slave verdict
  lines ("revived" vs "STILL DEAD after re-arm" with power-cycle advice)
  identify what remains.
- **Park/unpark hitching, further reduced.** Two RT-thread waits were
  removed on top of the Windows 11 timer fix below: status publishes no
  longer block on a lock a preempted web/UI reader may hold (Windows
  locks have no priority inheritance; publishes are now try-lock,
  skip-on-contention), and the publish path no longer allocates in steady
  state (state-name strings rebuild only on a state change, so the RT
  loop can no longer stall inside the process heap lock during GUI
  allocation storms - which is exactly what a park/unpark click
  triggers). Field results: better, subtler, not yet fully gone.

### Added
- **Rotary lever axis type (`rotary_lever`)** - first-class support for
  crank-arm actuators (lever 6DOF / hexapod rigs, geared rotary axes).
  The engineering unit is degrees at the lever shaft: the config editor
  and drive cards show arc travel, deg/s, deg/s2 and counts/degree; the
  gear ratio list extends to 100:1 (hand-edited values such as "63:1"
  are shown and preserved, never clobbered); the internal
  pitch-equals-360 convention is forced and hidden. Save-time clamps are
  rotary-aware, so a lever config survives the web editor. The motion
  path, homing, guards, and commissioning are unit-agnostic and needed
  no changes.
- **Commissioning results now land in the log** as one TESTRESULT line per
  axis per segment (amplitudes, ratio, phase, following error, torque
  metrics, step overshoot/rise/settle), so rig logs carry the measured
  data for later analysis, not just start/finish markers.
- **"What this means" read-out under the commissioning results**: plain-
  language per-axis verdicts derived from the numbers - usable bandwidth,
  resonances worth a notch filter, lash/compliance hints, hot vs soft
  step response, and axes notably softer than their peers.
- The default song is now the complete two-phrase statement of "5 Council
  Action Committee" by The Brown Stripe (any resemblance to other riffs is
  aspirational): eighth-note grid, explicit rests between plucks, main
  phrase answered by the bounce variant, and the whole thing transposed
  into the 20-33Hz band the actuators can actually reproduce - the sweep
  data showed the original octave (31-49Hz) came out as undifferentiated
  buzz.
- **Commissioning test mode: exercise and measure the rig without a game
  attached.** A new web-UI panel (Commissioning Tests) runs four test
  types through the normal guarded motion path: a motion cycle (pitch /
  roll / heave on role-assigned vertical axes, then each horizontal axis
  solo), a single vibration tone, a stepped frequency sweep, and a note
  sequence ("song" - basslines in octaves 0-1 sit in the actuators'
  20-60Hz voice). Every segment is measured: commanded vs actual
  amplitude at the excitation frequency (Goertzel), phase lag, RMS/peak
  following error, and torque ripple - so a sweep produces Bode points
  (bandwidth, resonances) for the current drive tune, and before/after
  sweeps turn tuning changes into a measured comparison instead of a
  feel test. Safety is enforced, not advisory: tests only start with the
  loop running, every tested axis homed and parked, and the telemetry
  stream quiet; belt and PP axes are never testable; amplitudes derate
  automatically to stroke/velocity/acceleration budgets (flagged in the
  results); all excitation is ramp-enveloped so motion cannot step; the
  guard chain stays live; and a sustained following error aborts the run
  and re-parks the rig. See Docs/COMMISSIONING.md.
- **Commissioning: step-response test and a load/inertia indicator.** A
  fifth test type jumps each axis to a held target (the guard chain sets
  the slew) and measures overshoot, 10-90% rise time, and 2%-band
  settling time - the classic before/after probe for gain changes. Sine
  segments additionally report the torque amplitude at the excitation
  frequency (static gravity-hold removed) and torque-per-acceleration,
  so a sweep now shows where the load stops behaving like pure mass -
  resonances show up as a peak in that column.
- **Drive faults are now decoded to their exact A6 Er codes.** Previously a
  fault logged only the raw 0x603F bus code, which is a coarse CiA402 class
  shared by many distinct faults (0xFF00 alone covers six different Er
  codes) - identifying the actual fault meant walking to the rig and
  reading the drive's panel. Now: the fault log line names the candidate Er
  codes for the bus class inline, and a one-shot SDO read of 0x203F (the
  precise panel code) runs on the recovery thread the moment a fault is
  seen, logging the exact Er code, its meaning, and whether it is
  resettable or needs a power cycle. The web drive card shows the decoded
  fault name while the drive is faulted. The full fault and alarm tables
  from the A6-EC manual (Er01.0 through ErC2.0 plus the ALF alarm class)
  ship in `A6FaultCodes.h` with a unit test (`TestFaultCodes`) proving
  panel-code uniqueness and table consistency. The read is a single
  mailbox transaction per fault event, not a poll - it cannot destabilise
  DC sync the way the old temperature polling did.

### Changed
- **Axis classification centralised (internal, behaviour-identical).**
  Every "what kind of axis is this" decision - homing exemption, park
  style, deinit-seat eligibility, commissioning role, display unit - now
  goes through one capability map (`AxisKind.h`) instead of scattered
  string compares. A golden test (`TestAxisKind`) pins every field
  against the legacy expressions across the full type x mode matrix, so
  existing rigs classify byte-identically and the future device family
  (H-shifter, active brake) becomes one added row, not a codebase sweep.
- **Telemetry is 16-bit only; the millimetre-guessing heuristic is gone.**
  The wire contract has always documented one scaling (0..65535, centre
  32767, full scale = the axis's configured stroke), but an unspecced
  heuristic dating to the project's first commit re-guessed the format on
  every frame: any channel above 500 meant "treat ALL channels as 16-bit",
  otherwise values were read as millimetres. A channel hovering around 500
  teleported targets between opposite stroke ends on consecutive frames, and
  one odd channel flipped the interpretation of every axis. Values are now
  read as 16-bit, full stop. A sender emitting raw mm offsets must be
  reconfigured to 16-bit output (SimHub's Decimal / 16-bit mode - what the
  setup guides have always specified).
- **All-zeros telemetry frames are treated as no-data.** In 16-bit, zero
  means full deflection to one end, so a frame where every channel is 0
  would command the whole rig to one end of travel at once - but no real
  motion frame looks like that, while some telemetry tools do emit
  all-zeros at menu/idle. Such frames now feed the normal telemetry-loss
  standby (hold, then ease to centre, then park) instead of being obeyed.
  This deliberately replaces the accidental protection the removed
  heuristic used to provide for that case.
- **Windows CI is now blocking.** Green across many consecutive runs since
  the Npcap SDK and delay-load fixes; a red Windows build now fails the
  workflow instead of being advisory.

### Fixed
- **The park/unpark motion hitch: Windows 11 was coarsening the RT timer
  whenever nullCAT lost window focus.** Windows 11 silently ignores a
  background process's 1ms timer request, honouring it only for the
  foreground window - and the operator is focused on SimHub or the game at
  exactly the moments the rig parks and unparks. The effective timer
  snapped to the 15.625ms default quantum, one RT sleep overshot by a full
  quantum (field logs show ~15.5ms stalls, the quantum signature to the
  microsecond, clustered on park/unpark events), and the loop's catch-up
  burst turned the lost cycles into a visible step mid-move. The process
  now opts out via PROCESS_POWER_THROTTLING_IGNORE_TIMER_RESOLUTION, so
  the 1ms timer holds while unfocused; together with the stall resync
  below, a residual stall becomes a brief hold instead of a lurch.
- **The PDO watchdog write is retried, and a persistent failure is loud.**
  One session showed 9 intermittent single-datagram failures configuring
  the watchdog - the mechanism that makes a drive drop torque if the host
  crashes - leaving those drives holding position indefinitely on a host
  crash, flagged only by a one-line warning. The two registers are now
  written together with 3 attempts, and a drive that still fails gets an
  ERROR naming the consequence (hardware e-stop is the only remaining stop
  for that axis).
- **The init shepherd spreads its nudges across the whole OP window.** The
  first cut nudged every 100ms and spent all 10 nudges inside the first
  1.1s; a multi-slave stall then sat un-nudged for the remaining 4s of the
  window. Nudges now fire every 500ms, spanning the full 5s. (Field logs
  also show that when SEVERAL slaves stall together the condition is
  bus/host-wide rather than per-slave hesitation - the per-slave nudge
  trail is what characterises those.)
- **A multi-millisecond host stall no longer triggers a catch-up burst.**
  The RT loop's deadline marched in fixed steps, so after a long stall
  (Npcap/DPC latency; 31ms observed in field logs) it fired all the missed
  cycles back-to-back. The drive's SYNC0 latch samples the last target it
  received, so the burst collapsed many cycles of commanded motion into one
  multi-millimetre latched step at full tracking speed - and many frames
  per SYNC0 interval is the same per-frame sync-error mechanism the
  DC-aligned pump cadence exists to prevent. A stall of more than two
  cycles now resyncs the cadence to the present and logs the skipped
  cycles; the axes hold briefly (the same policy as a bad frame) instead of
  lunging. Ordinary jitter still catches up normally.
- **The per-second `ferr`/`cmd` DIAG lines now carry the loop rate**
  (`hz=`), so a log alone is enough to interpret step sizes - the missing
  rate previously led to clamp saturation being misread as a limit
  violation.
- **Qt window opens readable and remembers its size.** First run opens at
  430x780 (the old 390x682 predates several UI additions and opened with
  the panel contracted); afterwards the window restores whatever size and
  position it last had, saved on a confirmed quit.

## [0.9.2] - 2026-08-19

Multi-drive init made reliable, a coordinate-frame fix that makes it
impossible to command an axis through its endstop, and honest names for the
axis-direction settings. Rig-verified across a five-drive session: homing,
unpark and telemetry response correct on both foldback and inline actuators.

### Fixed
- **Multi-drive EtherCAT init no longer fails because one slave hesitated.**
  The OP request was a single broadcast: a slave that missed it or silently
  declined (the A6's SafeOp-to-OP step is DC-sensitive) sat at SAFE-OP with no
  error code for the full 5s window and failed the whole init -- each added
  drive was another independent chance of that, which is how 5 drives reached
  roughly one failure in ten. The init pump now shepherds stragglers on its
  existing 100ms check: a slave at SAFE-OP+ERROR gets the AL error
  acknowledged, a slave parked at plain SAFE-OP gets a per-slave OP
  re-request, bounded (10 nudges) and logged per slave. Register writes only,
  SYNC0 untouched, healthy boots log nothing. A slave below SAFE-OP, or one
  that exhausts its nudges, still fails init exactly as before.
- **An axis homed in the positive raw direction can no longer be driven
  through its endstop.** The engineering frame had no direction sign: it
  silently assumed homing searched raw-negative, so an axis whose retract
  direction is raw-positive (an inline actuator on a rig wired like the
  reference foldbacks) homed correctly and was then commanded half a stroke
  THROUGH the hardstop on unpark. Position now always counts away from
  whichever stop was homed, the post-homing drive clamp window lands on the
  correct side of that stop (it previously landed entirely on the far side,
  disarming the overtravel protection exactly where it was needed), and even
  a misconfigured axis can only home to the unintended end, never be pushed
  past it.

### Changed
- **Config keys renamed to what they actually do**: `homeMode` is now
  `parkMode` (it selects the park position and never affected homing) and
  `homingSpeedMmS` is now `homingSpeed` (the value is a step multiplier, not
  mm/s). Both old keys are still read, so existing `rig.json` files keep
  working; when both spellings are present the new one wins.
- **`invertDir` now means what it always said: the axis's mechanical
  polarity.** Tick it (labelled "Foldback" in the web UI) for a foldback
  linkage, leave it off for an inline actuator. It sets the homing search
  direction and the telemetry response together (previously it flipped the
  telemetry response only, which is why an inline axis could not be made to
  retract to home). `homeDirection` is now a travel-frame stop selector:
  `negative` (default) homes to the retracted stop, `positive` to the
  extended stop for park-extended setups. Existing foldback configs
  (`invertDir: true`, `homeDirection: negative`) behave identically.

## [0.9.1] - 2026-08-15

Config changes now reach the engine without restarting the application, the
desktop panel gains its missing Park control, and axis defaults come from one
place. Verified on the rig across a two-hour session covering belt and axis
configuration changes with no application restart.

### Added
- **Qt UI: Park / Unpark button.** The desktop panel was the only control
  surface without one (the engine, the web UI and the HID `park-toggle`
  binding all had it). Sits above the Belts toggle; label shows the action,
  and it is disabled mid-transition so it cannot reverse a park, unpark or
  home already in flight.

### Changed
- **Web config page: the Save bar is now sticky** and Button bindings are
  collapsed by default. The save button used to sit below a tall, full-width
  bindings panel that has its own "Save bindings" button, so it was easy to
  save the bindings and believe the config was saved too. The two write
  different files: bindings hot-apply, config needs a restart.
- **Axis defaults now come from the compiled-in values** in `Config.h`, as
  `Docs/CONFIG_REFERENCE.md` has always claimed. The `rig.json` reader carried
  its own second set of defaults which had drifted from the struct in three
  places.

  > **If your `rig.json` omits any of these keys, the effective value changes:**
  > `homeMode` `center` → `endstop`, `homingSpeedMmS` `5` → `250`,
  > `maxAccelerationMmS2` `2000` → `10000`. A rig configured through the web UI
  > is unaffected - it writes every key explicitly. A hand-written or partial
  > `rig.json` is not: in particular, homing would approach the hardstop 50×
  > faster than before. Check your axes before the first run on this version.

- **Homing search timeout raised 30s → 60s**, and the timeout now says what it
  saw. A long axis at a slow `homingSpeed` could exhaust a fixed 30s of
  wall-clock before ever reaching its hardstop, aborting to `FatalError` on a
  perfectly good rig. The timeout is deliberately not derived from stroke and
  speed: `homingSpeed` is a per-cycle step multiplier, not true mm/s, so any
  "expected traverse time" computed from it would be fiction. The distance
  guard (1.5× stroke) is checked first on every cycle and is unchanged, so a
  longer timeout costs waiting time on a broken axis - never extra travel.
- **Homing timeout and axis-configure logging now carry the fields a
  post-mortem needs.** The timeout was the only abort that reported no
  distance; it now gives travelled-vs-stroke and the *measured* mm/s, which
  separates "still crawling toward the stop, raise the speed" from "barely
  moved, check the mechanics". The per-axis configure line gains `homeMode`
  and `homingSpeed`, neither of which appeared anywhere in the log before -   `homeMode` selects the park position and nothing else, so its absence made
  park-behaviour reports impossible to diagnose from a log alone.

### Removed
- **`homeMode: "gravity"`.** It declared an axis homed at wherever it happened
  to be resting, with no search, on the assumption that gravity had already
  parked a vertical actuator on its bottom stop. A leftover from the PP-mode
  era; never exposed in the UI, never documented, unused. Any unrecognised
  `homeMode` now falls through to the real torque-based endstop search, so a
  config still carrying `"gravity"` gets safer rather than broken.

### Fixed
- **Config changes apply on Initialize, not only on an application restart.**
  A `rig.json` save made while EtherCAT was up was reloaded into memory but
  never reached the motion controller: both init paths re-applied the config to
  the EtherCAT master alone, so drive/PDO setup picked the change up while every
  motion-owned value kept whatever it was given at startup - belt tension limits
  and guards, stroke, velocity/accel/jerk, homing parameters, spike filter,
  tracking, park/unpark times, conditioning mode. The UI's "Stop &
  Re-initialize to apply" was therefore false; only closing and reopening the
  app applied them. Both entry points now re-apply - the Qt Initialize button
  and `/api/init`, since an operator who edits and initialises from the web UI
  never touches the Qt button.

  > Unchanged on the Pi: it has no `rig.json` reload path, so the headless
  > daemon still needs a service restart to pick up a config change.

- **Unpark now refuses an axis that was never homed.** Not reachable in the
  normal flow (stopping the loop re-arms the rehome, starting it homes, and
  park/unpark require a running loop), but a homing fatal error left that axis
  parked-and-unhomed while its peers also ended parked - which reads as "all
  parked", so the toggle offered Unpark. Unpark ramps toward mid-stroke, and
  for an unhomed axis that target is in a coordinate frame unrelated to the
  machine.
- **Windows builds from a clean checkout.** The Npcap SDK's `Include`
  directory was never on the compile path, so SOEM's `nicdrv.h` could not find
  `pcap.h`. It built only on machines whose SOEM tree happened to vendor the
  pcap headers. `BUILD_INSTRUCTIONS.md` also now covers the SDK as a separate
  download from the Npcap runtime, and requires **1.16 or newer** - 1.13 and
  older redefine `bpf_program`/`bpf_insn` and fail to compile against SOEM 2.x.
- **Release archives use ZIP-spec path separators.** `Compress-Archive` writes
  backslashes, which Windows tools tolerate but some non-Windows tools turn
  into literal backslashes in filenames.

### Internal
- Windows CI is green: Npcap SDK and the pinned SOEM build are cached (the
  download is retried with backoff, since npcap.com intermittently blocks CI
  address ranges), and the SOEM-linking test binaries delay-load `wpcap.dll`
  so they run on a machine that has the SDK but not the Npcap driver.
  `nullCAT.exe` itself still links it normally, so a missing Npcap fails
  immediately and visibly for an end user.

## [0.9.0] - 2026-08-04

First public beta. Windows desktop build (Qt + Npcap/SOEM) and the Raspberry Pi
headless daemon share one motion core.

- EtherCAT master (SOEM) with DS402 drive management: staged init, DC-sync
  (SYNC0), fault monitoring and recovery, and provisioning tooling for
  STEPPERONLINE A6-EC drives.
- Motion: UDP telemetry (e.g. SimHub) → per-axis tracking filter → CSP position
  drives (heave/pitch/roll) and CST torque drives (belt tensioners), with homing
  against hardstops, parking, e-stop staging and telemetry-loss standby.
- Control surface: embedded web UI (status, config, drive cards, HID button
  bindings), a compact Qt desktop panel on Windows, and an optional GPIO control
  panel on the Pi.

[0.9.3]: https://github.com/crossthenoughts/nullCAT/compare/v0.9.2...v0.9.3
[0.9.2]: https://github.com/crossthenoughts/nullCAT/compare/v0.9.1...v0.9.2
[0.9.1]: https://github.com/crossthenoughts/nullCAT/compare/v0.9.0...v0.9.1
[0.9.0]: https://github.com/crossthenoughts/nullCAT/releases/tag/v0.9.0
