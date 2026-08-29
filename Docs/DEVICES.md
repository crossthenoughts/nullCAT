# Force Devices (shifter, active pedal)

A device axis turns a servo into a force-feedback control: an H-pattern
or sequential shifter with real gates and detents, or an active pedal
with a programmable resistance curve. The motor renders force against
your hand or foot; it never follows motion telemetry.

Device support is new in 0.9.5 and aimed at the Pi build. Treat it as
experimental: start with low forces and nothing attached to the lever
that can hurt you.

## How a device behaves

- **Homing** is gentle: the axis pushes against one of its travel stops
  at a low, configurable torque until it stalls there, and that stop
  becomes its reference. No high-speed moves.
- **After homing the device is limp.** The motor applies no torque and
  the lever moves freely. This is its safe state - it returns to it on
  release, on park, and on e-stop.
- **Engage** loads the force feel (spring, gates, detents), fading it in
  over the blend time so nothing snaps. **Release** fades it back out.
- The rig's Park button also releases every device.

## Setup

1. On the web UI, open Configuration, tick **Device controls** (under
   Advanced), and Save. The Devices section appears. (Pi builds only.)
2. Add an axis and set its type to **shifter** or **pedal**. Torque mode
   is selected automatically and a starter feel is filled in.
3. Set the geometry - easiest by hand. Home the device (it rests limp),
   then in its Devices card: hold the lever at one end and press **Set
   min here**, the other end and **Set max here**, the rest position and
   **Set neutral here**, and each gear slot with **Add gate here**. Save.
   The values are motor revolutions in the homed frame; you can also
   type them directly into the fields. `Home toward` picks which stop
   homing pushes against - wrong direction means homing pushes the wrong
   way (the travel guard will stop it, but get it right first).
4. Save, restart, initialize, home. The device card shows its state;
   press **Engage** when you want the feel live.
5. Pick a preset as a starting point, then shape the feel directly: the
   two curves under each device (centring spring, detent profile) edit
   by dragging their nodes - double-click adds or removes a node. Save
   to persist. The full parameter list is in CONFIG_REFERENCE.md (the
   `device` object).

A mirrored build (motor on the other side) flips ONE setting:
`device.dir`. Never rewrite the geometry for that.

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
