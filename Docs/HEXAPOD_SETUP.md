# Hexapod setup

How to bring up a six-leg hexapod on nullCAT. The walkthrough is written
for **rotary-lever hexapods** (crank arms on geared servos, `rotary_lever`
axes); a classic **linear hexapod** (six identical ballscrew struts) uses
the same flow with `linear_vertical` axes and millimetre units throughout.

Read [SAFETY.md](../SAFETY.md) first. Six coupled legs can pinch, crush,
and tip; commission with the platform empty and a hand near the hardware
e-stop.

## The division of labour

**The sender does the mixing.** Your motion software (FlyPT Mover, or
SimHub with per-axis assignments) owns the hexapod's pose-to-leg
kinematics and sends one 16-bit target per leg over the normal telemetry
packet (`NULLCAT,<leg1>,...,<leg6>`, axis order = chain order). nullCAT
follows each leg through its own guard chain; it deliberately contains no
inverse kinematics. The wire contract is in
[CONFIG_REFERENCE.md](CONFIG_REFERENCE.md).

## Axis configuration

Start from [`deploy/rig.hexapod.example.json`](../deploy/rig.hexapod.example.json)
or build the axes in the web editor (add one leg, fill it in, then
"Apply to all" and adjust per leg). The rotary specifics:

- **Units are degrees at the lever shaft.** Arc travel replaces stroke,
  speeds are deg/s, and the editor shows counts/degree.
- **Gear ratio** (`reductionRatio`): the lever's gearbox, e.g. `50:1`.
  Hand-edited values such as `63:1` are preserved.
- **Following-error window** must sit within the leg's own arc (default
  seed 20 deg). This is enforced at save AND at load: a window wider than
  the arc would disable the drive's runaway protection, so a config
  carrying the linear default is rejected with a clear error.
- **`parkMode: center`** - a lever has no gravity-defined rest position.
- **`invertDir` per leg**: mirrored leg pairs are wired opposite, so
  expect it to alternate. Verify each leg during first homing (below)
  and flip any leg that searches toward the wrong end.
- **Self-locking gearboxes**: nullCAT skips the deinit "seat" pass on
  rotary axes on the assumption that a high-ratio box holds position when
  de-energised. Verify yours does, or fit a drive-managed brake, before
  trusting the platform to stay put at power-off.

## Homing a coupled platform

A hexapod's legs are not independent axes: the platform couples them,
and self-locking gearboxes mean the stationary legs have no compliance.
**Never drive one leg through a large arc with the platform attached** -
the mechanism binds at the joint limits, and a torque search that binds
either false-trips (homing to a position that is not the stop) or
strains the joints. Three regimes:

- **Direction verification, first bring-up: pushrods disconnected.**
  Use the per-axis **Home** button beside each axis row in the
  Commissioning Tests panel (`POST /api/home` with `{"axis": N}` for
  scripts) with the leg unloaded. Watch the first few degrees of the
  search; a leg that turns the wrong way needs `invertDir` flipped, then
  stop the search. Unloaded there may be no true hardstop - letting it
  run until the stroke guard aborts is harmless.
- **Actual homing, platform attached: all legs together** - the normal
  homing on loop start. Every leg searches the same travel direction, so
  the platform sinks toward its all-retracted pose (a legal pose by
  construction). Watch the first run: coupling can trip a neighbour's
  threshold early as legs arrive at their stops at different moments.
  An unlevel platform at park or inconsistent home offsets are the
  symptoms; a slightly higher `homingTorquePct` or a slower
  `homingSpeed` is the fix.
- **Single-leg re-home near park (for example after a fault): safe with
  the platform attached**, because the remaining search distance is only
  the backoff couple of degrees. This is the per-axis button's
  with-platform use.

## Commissioning a hexapod

nullCAT does no kinematics, and that holds during testing too: **it never
generates platform pitch or roll on a hexapod**. Mixing a tilt onto six
coupled legs needs the real leg geometry, and getting it wrong binds the
mechanism - that test belongs to your motion software, which owns the
kinematics (use its manual pose / test controls with nullCAT simply
following each leg). What the built-in tests give you:

- **Heave** is geometry-free and safe: every leg moves in phase by the
  same amount, the same legal pose family as homing and parking. Rotary
  legs take part in the motion cycle's heave automatically and are never
  given pitch/roll roles. (Linear-strut hexapods: leave the Front/Rear
  and Left/Right selectors unset for the same behaviour.)
- **Per-leg characterisation**: tones, sweeps, and steps excite the
  SELECTED legs in phase - select all legs for heave-mode excitation, or
  a single leg to characterise it solo. Solo amplitudes should stay
  small (a few percent): on a coupled platform one leg's excursion is
  absorbed by joint clearance alone.

Results rows are tagged with each axis's unit, and the following-error
abort rail scales to each leg's arc automatically. See
[COMMISSIONING.md](COMMISSIONING.md).

## Limits worth knowing

- Up to 10 axes per controller; 6 levers fit with room for belts.
- Larger topologies than the 4-drive reference rig are field-tested
  territory (see [KNOWN_LIMITATIONS.md](../KNOWN_LIMITATIONS.md)); your
  reports are the data the project needs.
- Simulation mode cannot home, so a six-leg homing sequence can only be
  rehearsed on hardware.
