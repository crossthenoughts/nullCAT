# Force Devices (shifter, active pedal)

A device axis turns a servo into a force-feedback control: an H-pattern
or sequential shifter with real gates and detents, or an active pedal
with a programmable resistance curve. The motor renders force against
your hand or foot; it never follows motion telemetry.

Device support is new in 0.9.5 and aimed at the Pi build. Treat it as
experimental: start with low forces and nothing attached to the lever
that can hurt you.

## How a device behaves

- **Devices never move with the rig.** Starting the loop, Home All, and
  e-stop release leave every device untouched and limp - a homing push
  must never surprise a hand resting on the lever. A device does things
  only from its OWN button.
- **The device button is three-state.** First press **homes**: a gentle
  push against one travel stop at low torque until it stalls there (that
  stop becomes the reference), then the device rests **limp** - the
  motor applies no torque and the lever moves freely. The next press
  **engages** (the force feel fades in over the blend time), and an
  engaged press **releases** back to limp. Limp is the safe state; it
  also returns on park and e-stop.
- The same button exists on the web device card, as a bindable HID
  command (`device-toggle`), and as a GPIO panel button - all resolving
  identically.
- The rig's Park button also releases every device.

## Setup

1. On the web UI, open Configuration, tick **Device controls** (under
   Advanced), and Save. The Devices section appears. (Pi builds only.)
2. Add an axis and set its type to **shifter** or **pedal**. Torque mode
   is selected automatically and a starter feel is filled in. Save and
   restart, then Initialize and Start.
3. **Teach the travel by sweep.** Press the device button once - it
   homes and rests limp. Move the lever end to end by hand (the card
   shows the live position and the swept range), then press **Capture
   travel**. That is the mechanism learned; it never needs teaching
   again.
4. **Derive the layout.** Pick a layout and press **Derive layout**:
   - **H / sequential**: neutral at centre, one engagement throw value
     placing the fore/aft gates. (An H-pattern's side-to-side select is
     purely mechanical - the force axis only ever sees the fore/aft
     line, the same for every column.)
   - **Selector (auto)**: splits the range into N slots - an automatic-
     style lever (P/N/D/...) on the same hardware. Which slot means what
     is the game's business.
   - **Custom**: type gates and neutral directly (throttle-quadrant
     style detent placement).
   Use **Set neutral here** if the rest position is off-centre. Save.
5. **Tune the feel live.** Pick a preset, drag the curve nodes
   (double-click adds or removes a node; the dot shows where the lever
   sits right now), adjust friction/breakout/damping - and just Save:
   device settings apply the moment the device is limp, no restart. If
   it was engaged when you saved, they land on release. When a feel is
   right, **Save as preset** keeps it by name, safe from anything.

A mirrored build (motor on the other side) flips ONE setting:
`device.dir` (the Mirror field). Never rewrite the geometry for that.

## Sim-driven effects

With a sim feeding raw telemetry, the shifter can refuse and grind
shifts made without the clutch. Two pieces:

**The channel stream.** A `NULLCATX` UDP line carries raw values (rpm,
speed, gear, clutch, throttle) to the same port as motion telemetry:

- **SimHub:** install the nullCAT Channel Exporter plugin from
  `integrations/simhub/` in the repository. It sends the line above,
  nothing else.
- **FlyPT Mover / other tools:** any tool that can compose a text UDP
  line from telemetry fields works. Send
  `NULLCATX,<rpm>,<speedKmh>,<gear>,<clutchPct>,<throttlePct>` at
  100 Hz or more (clutch: 0 = pedal up, 100 = floored).

**The bindings.** Tell the rig which channel means what with
`ncxBindings` in rig.json (example in the plugin README). The Devices
section shows whether the stream is being received.

**The effects.** On a shifter with detents configured:

- `clutchBitePct`: with the clutch reading below this (pedal up, clutch
  driving), moving the lever out of gear is *blocked* - the whole feel
  stiffens by `blockGain`.
- `grindAmpPct` / `grindFreqHz`: pushing against that blocked gate
  grinds.

- `rpmMatchPct`: the revmatch window. With rpm, speed, and gear bound,
  the controller learns each gear's rpm-per-speed ratio while you drive
  (no car database, no setup - it identifies returning cars by their
  ratios and remembers them across sessions in `carcache.json`). A
  clutchless shift then goes IN when your blip has the engine within
  this window of what the next gear needs - and grinds when it does
  not. Give it a lap of normal driving in at least two gears before
  expecting let-ins.

All of these are off by default. If the channel stream stops for half a
second, every effect drops out and the plain feel remains - a lost
connection can never lock your shifter.

## Troubleshooting

- **Homing trips the travel guard**: `homeDir` points at the wrong stop,
  or the stops don't match the mechanism. Fix the geometry.
- **Forces feel mirrored** (pushes when it should pull): flip
  `device.dir`.
- **Effects never trigger**: check the Devices section says the channel
  stream is *receiving*, and that `clutchPct` is bound. A clutch channel
  arriving as 0..1 needs `"scale": 100`.
- **Lever oscillates or buzzes at rest**: lower the spring slope near
  neutral, raise `dampPctPerRevS`, or add a little `lashRev`.
- **No detent feel, just spring and walls**: the gates are probably at
  (or too near) neutral, hiding under the centring spring. Gates belong
  at the ENGAGEMENT positions - re-derive the layout. Also check the
  detent force peaks above the spring's value at the gate position, or
  the lever will pop out of gear.
- **Shifts feel rubbery**: raise `breakoutScale` (out-of-gear firmness),
  add a few % `frictionPct`, and steepen the detent profile just off
  centre.
