# Commissioning Tests

nullCAT can exercise the rig without a game or SimHub attached: motion
cycles for visual checkout, vibration tones, stepped frequency sweeps, and
note sequences. Every run also measures how faithfully each axis tracked
the excitation, which turns "does the tune feel right?" into numbers.

The panel lives in the web UI under **Commissioning Tests** (enable
`webUIEnabled` on Windows if the panel is needed there).

## Prerequisites (enforced, not advisory)

A test refuses to start unless ALL of these hold:

- The control loop is running and every tested axis is **homed** and
  **PARKED** (no rehome pending, no e-stop).
- **No telemetry is streaming.** Stop the game/SimHub feed first; a live
  stream and a test would fight over the axes. Anything received in the
  last 2 seconds counts as streaming.
- Belt (torque) and PP axes are never testable.

The refusal reason appears in the panel status line.

## What runs

1. **Centering** - each tested axis eases from its park position to its
   configured centre (cosine profile, ~25 mm/s).
2. **Segments** - the test's excitation, one segment at a time. Every
   segment is amplitude-enveloped (raised-cosine ramps), so motion always
   starts and ends at zero offset. No steps, ever.
3. **Return + park** - offsets ease back to zero and the axes park through
   the normal park machinery (`parkMode` respected).

## Safety rails

- **Amplitude derating.** Requested amplitudes are clamped per axis to fit
  90% of the usable half-stroke about centre, 80% of `maxVelocity`
  (amplitude x 2*pi*f), and 80% of `maxAccel` (amplitude x (2*pi*f)^2).
  High-frequency tones therefore run at small amplitudes by physics:
  at 30 Hz with the default 2000 mm/s^2 budget the cap is ~0.05 mm.
  Derated results are flagged `drtd` in the table - a quiet tone is a
  budget, not a rig problem.
- **Guard chain live.** Commands go through the same velocity/accel clamps
  and braking guard as game telemetry.
- **Following-error abort.** Command-vs-actual error sustained for 50 ms
  (or double it instantaneously) aborts the test and eases the rig back
  to centre. The threshold is per axis: 10 units, capped at 20% of the
  axis's usable half-range, so a short rotary arc rails proportionally
  (a 40 deg lever aborts at 4 deg, not a quarter of its travel). Units
  follow the axis: mm on linear axes, degrees on rotary levers. Any
  drive fault or e-stop cancels immediately.

## Test types

- **Motion cycle** - gentle full-body movements at ~0.2 Hz: pitch (front
  verticals up while rears go down), roll (left vs right), heave (all
  verticals together), then each horizontal axis on its own. Per-movement
  enable + amplitude (% of half-stroke). Pitch/roll need each vertical
  axis assigned a Front/Rear and Left/Right role in the panel - the guess
  from the axis name can be overridden and is remembered by the browser.
- **Tone** - a single sine at a chosen frequency/amplitude/duration.
- **Sweep** - a ladder of tones from A to B Hz in fixed steps, dwelling on
  each. This is the Bode-plot generator (see below).
- **Step** - the raw target jumps to +amp and holds. The guard chain sets
  the actual slew (maxVelocity/maxAccel), and the axis is measured against
  the held target: overshoot %, 10-90% rise time, and settling time into
  the 2% band. The classic before/after probe for gain changes: rising
  overshoot = too hot, long settle without overshoot = too soft.
- **Song** - a note sequence. Notes `c0`-`b8` with `#`/`b`, `:n` for a
  length in beats, `r` for a rest, e.g. `a0:2.5 r:0.5 a0:0.75 r:0.25 c1`.
  Actuators speak best around 20-33 Hz (higher notes come out as buzz on a
  typical tune - see your own sweep results), so write basslines in
  octaves 0-1 and keep the low end above `c0`. The default is "5 Council
  Action Committee" by The Brown Stripe, whose resemblance to any famous
  riff is entirely coincidental and, on current evidence, legally safe.

## Reading the results table

One row per segment per axis, measured over the steady part of each
segment (ramps excluded):

| column | meaning |
|---|---|
| cmd / act | commanded vs measured amplitude at the segment frequency (Goertzel), in the axis's unit (mm linear, degrees rotary). Step rows: held target vs final settled value |
| ratio | act/cmd. 1.0 = perfect tracking; falling ratio = approaching bandwidth. Step rows: settle accuracy |
| phase° | response phase relative to command; more negative = more lag |
| ferr rms / pk | following error over the segment (mm) |
| trq σ% | torque ripple (std dev about its mean) - a hunting/vibration indicator |
| trq@f% | torque amplitude AT the excitation frequency, static hold removed |
| trq/acc | trq@f per unit of measured acceleration (% rated per m/s²) - the load/inertia indicator: flat across a sweep = mass-dominated, a peak marks a resonance |
| os% / rise ms / settle ms | step rows only: overshoot, 10-90% rise, settling into the 2% band |

A sweep's rows ARE Bode points: the frequency where ratio drops to ~0.7
is the axis bandwidth at the current tune; a ratio bump above 1.0 with a
phase kink marks a resonance (a candidate for a drive notch filter);
rising trq σ% without a matching ratio rise suggests hunting. Comparing
two sweeps before/after a tuning change is the scientific way to decide
whether the change helped.
