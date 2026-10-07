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
speed, gear, clutch, throttle, and the haptic channels below) to the
same port as motion telemetry:

- **SimHub:** install the nullCAT Channel Exporter plugin from
  `integrations/simhub/` in the repository. It sends all 13 channels;
  see its README for the settings file.
- **FlyPT Mover / other tools:** any tool that can compose a text UDP
  line from telemetry fields works. Send
  `NULLCATX,<rpm>,<speedKmh>,<gear>,<clutchPct>,<throttlePct>` at
  100 Hz or more (clutch: 0 = pedal up, 100 = floored), adding the
  haptic channels in the order given in `Docs/PROTOCOL.md` if your
  tool can supply them.

**The bindings.** The rig already knows which channel means what: the
default bindings match the SimHub plugin's order. Change them only if
your sender uses a different order, in the Sim channels section of the
web Setup view (it has a Reset to defaults button). The Devices section
shows whether the stream is being received.

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

## Haptics

Haptics are short vibrations and textures that ride on top of the
normal feel of any axis: the belt tensioners, devices like the shifter,
and the position axes that move the rig. They are always kept inside
each axis's own limits. They live on the **Haptics** strip in the
Operate view, which appears once Experimental features is ticked in the
host settings.

### Setting one up

Every effect starts off. To make one work it needs three things:

1. **Amplitude** above 0 (`amp %`, a share of the axis's rated torque).
2. **A route**: click the "not routed" link on the tile and give the
   belt, the shifter, or any axis a gain above 0. One effect can go to
   several axes.
3. **Save**: the strip has its own Save bar. Changes apply immediately,
   with no re-initialize.

The routed axis must also be live (belt tensioned, device engaged,
position axis online) to be felt. **Test** plays the effect for a
moment and, if nothing could be felt, says why: amplitude 0, no route,
or the routed axis not live. The tile's waveform only moves while the
effect is actually producing force, and its shape follows the amplitude
and frequency you set.

**Master** scales every effect at once; **Mute** silences the whole
layer without changing any settings (press again to bring it back).

### Routing to a position axis

A torque axis takes a plain gain (x). A position axis takes its gain in
millimetres: the offset the effect adds to the axis's position at 100 %
amplitude, so a route of `0.8` on the surge axis with the effect at
50 % is a 0.4 mm wobble. The offset rides on top of the motion cue and
the cue always keeps priority; the axis's own velocity and acceleration
limits still apply to the sum.

Two things set how much a position axis will actually give:

- **Haptic max** in the axis editor (default 3 mm) is the hard ceiling
  on the offset. Set it to 0 and that axis takes no haptics at all.
- **pos budget** on the Master tile (default 0.4) is the share of each
  axis's velocity and acceleration limits that haptics may use.

A linear actuator cannot swing 3 mm at 90 Hz, so each effect is derated
to what the axis can follow at its carrier: an effect is held to the
smaller of the cap, `budget x max velocity / (2 pi f)` and
`budget x max acceleration / (2 pi f)^2`. The route editor shows the
result for every position axis as "up to X mm at F Hz" before you save
or play anything. Low-frequency effects (the engine's idle rock, a 5 Hz
skid) come through at useful amplitudes; a 90 Hz buzz on a surge axis
is a fraction of a millimetre and is better sent to a belt. Routed
effects are only felt while the axis is online and tracking; PP-mode
axes take no haptics.

### The effects

Each tile lists the sim channels it needs, with a tick when the channel
is arriving and a cross when it is not. A missing channel leaves that
effect silent.

| Effect | What you feel | Channels |
|---|---|---|
| Detent click | the lever dropping into a gate | none (from the shifter itself) |
| Gear shift | a thunk on every gear change | gear |
| Engine | the engine running, from idle to the limiter (below) | rpm, throttle, limiter |
| ABS | a regular pulse while ABS works and the brake is on | brake, ABS active |
| Lockup | brake lockup judder | lockup |
| Skid | tyre slip texture | skid |
| Road | road surface texture | road |
| Limiter | an extra hammer while on the rev limiter | limiter |
| TC pulse | a pulse while traction control cuts | TC active |
| Kerb | kerb strip rumble | kerbs |

With the SimHub plugin, rpm, speed, gear, clutch, throttle, brake, ABS,
limiter and traction control arrive automatically. Skid, lockup, road
and kerbs need a SimHub property named in the plugin's settings file
(see the plugin README).

`freq hz` sets the carrier: lower is a heavier, slower shake, higher is
a finer buzz. `jitter` roughens it so slip, road and kerbs feel like
texture rather than a tone.

### The engine

Describe the engine and the effect follows it:

- **cyl / rotors**, **litres**, **layout** (inline, V, flat/boxer,
  Wankel). Bigger cylinders hit harder; more cylinders and a V or flat
  layout run smoother; a Wankel has no shake at idle but an uneven beat.
- **max rpm**: the redline. Leave it at 0 and it is learned while you
  drive (set exactly the first time you hit the limiter).
- **rock x / thump x / buzz x** mix the three parts, 0 to switch one
  off: the block rocking at idle, the individual firings, and the
  vibration that rises in pitch and strength up the rev range.
- **buzz order**: 0 picks the pitch automatically so the redline sits
  at the top of the range; set it by hand to move it.
- **thump hz**: the weight of each firing (lower = heavier).
- **lope**: idle unevenness; 0.15 to 0.3 for a big-cam V8.

**The limiter.** On the limiter the engine cuts whole bursts of firings
and comes back hard, which is the bounce. Three controls:

- **limiter x**: how hard each return hits (1 = full load).
- **limiter hz**: how often it cuts. Lower (6 to 10) for big, slow
  engines and heavy flywheels, a lazy "bap bap bap"; around 12 for most
  cars; higher (15 to 25) for small high-revving engines and race ECUs,
  a tight stutter. If it feels like the engine is just running at a
  different speed, go lower; if it feels like separate knocks, go
  higher.
- **limiter jit**: 0 for a clean modern ECU, 0.2 to 0.4 for a road car,
  0.5 and up for something rough like a carburettor classic or a rotary.

The separate **Limiter** tile adds an extra hammer on top; keep it at
around 12 Hz so it lands in time with the cuts.

If the channel stream stops for half a second, every effect fades out.

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
