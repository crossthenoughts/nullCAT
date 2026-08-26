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
  speeds are deg/s, and the editor shows counts/degree. The internal
  `ballscrewPitch = 360` convention is forced and hidden.
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

## First homing: one leg at a time

Six coupled legs torque-searching simultaneously can trip each other's
thresholds (the platform couples them mechanically). Home legs
individually the first time: the **Commissioning Tests** panel has a
per-axis **Home** button beside each axis row (`POST /api/home` with
`{"axis": N}` for scripts). Watch each leg search toward its intended
stop; a leg that searches the wrong way needs its `invertDir` flipped.
Once every leg's direction is verified, the normal all-axes homing on
loop start is fine.

## Commissioning a hexapod

The motion-cycle test's Front/Rear + Left/Right roles describe four-post
rigs. A hexapod needs **custom mixing weights**: tick "Custom mixing
weights" in the Commissioning Tests panel and give each leg a signed
pitch / roll / heave weight in [-1, 1].

Starting point for an evenly spread six-leg layout (legs at 60-degree
spacing, azimuth measured from the nose): `wPitch = cos(azimuth)`,
`wRoll = sin(azimuth)`, `wHeave = 1`. For legs at 30/90/150/210/270/330
degrees that gives:

| Leg | azimuth | wPitch | wRoll | wHeave |
|---|---|---|---|---|
| 1 FL | 330 | 0.87 | -0.5 | 1 |
| 2 FR | 30 | 0.87 | 0.5 | 1 |
| 3 ML | 270 | 0.0 | -1.0 | 1 |
| 4 MR | 90 | 0.0 | 1.0 | 1 |
| 5 RL | 210 | -0.87 | -0.5 | 1 |
| 6 RR | 150 | -0.87 | 0.5 | 1 |

These are feel-check weights, not an IK substitute - your motion
software's profile remains the authority for game motion. Run the first
cycle at low amplitude (10-15%) and watch: pitch should tilt the
platform nose-wise, roll side-wise, heave lift it evenly. A movement
that fights itself means a sign is wrong - flip that leg's weight (or
revisit its `invertDir`, which the tests respect the same way telemetry
does).

Sweeps, tones, steps, and the results table all work per leg exactly as
on a linear rig; results rows are tagged with each axis's unit and the
following-error abort rail scales to the leg's arc automatically. See
[COMMISSIONING.md](COMMISSIONING.md).

## Limits worth knowing

- Up to 10 axes per controller; 6 levers fit with room for belts.
- Larger topologies than the 4-drive reference rig are field-tested
  territory (see [KNOWN_LIMITATIONS.md](../KNOWN_LIMITATIONS.md)); your
  reports are the data the project needs.
- Simulation mode cannot home, so a six-leg homing sequence can only be
  rehearsed on hardware.
