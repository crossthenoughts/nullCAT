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

Each tile also carries a small **sim dot** on the right of its name.
nullCAT remembers, for every game that has sent channels, whether each
effect's channels have ever arrived and whether the effect has ever
played: grey means that game has never sent what the effect needs,
amber means the channels arrive but the effect has never played
(amplitude 0, no route, or something the game never triggers), green
means it has played. The memory survives restarts, so a sim's coverage
is known without a lap in it. The Master tile names the game the dots
are for, lets you look at another remembered game, and can forget one.
A sender that does not name its game (see PROTOCOL.md, `NULLCATY`) is
remembered as an unnamed sender.

### Shakers

Bass shakers and other tactile transducers hang off a USB sound card and
take the same effects as the axes. In the host settings tick
**Shakers**, name the **Audio device** (part of its name; empty for the
system default; `null` for a bench with no card) and how many of its
outputs are **Shaker channels**; a second card (a second stereo dongle
is the usual way to four shakers) goes in **Audio device 2**, its
channels numbered after the first card's. Restart, and the status line
under those fields names each card with its sample rate, buffer and
underrun count, with a **test** button per channel (40 Hz for a second).

Every effect's route editor then lists **Shaker 1..N** below the axes.
A shaker route has a gain (1 = full scale at 100 % amplitude; the output
soft-clips, never clips hard) and a **harmonic**: x1 plays the effect's
carrier as it is, x2 doubles it, and so on, phase-locked to the same
oscillator. That is how a 9 Hz belt effect lands on a shaker at 18 Hz
where the shaker has output, while the belt keeps the 9 Hz. Pulses
(thumps, clicks) and the road replay play as they are.

The shaker path is a few milliseconds behind the belts (the card's
buffer, 15 to 25 ms on most cards). **axis delay ms** on the Master tile
holds the axis effects back by that much so everything lands together;
the motion cue itself is never delayed. Shakers keep playing with the
control loop stopped and on a rig with no drives at all, so a shaker
install on its own is a complete haptics engine for the sim channels.
Underruns in the status line mean the sender or the machine stalled;
the chain holds the last sample and fades rather than clicking.

### Profiles

A profile is a named copy of everything on the strip: every tile's
settings and routes, the engine description, master and pos budget. The
**profile** row on the Master tile holds them. **Save as** snapshots the
saved settings under a name (Save the strip first if you have edits);
**load** copies the chosen profile over the live settings and applies it
at once; **delete** forgets one. **Use for this car** ties the car the
sim is naming right now (its game when it names no car) to the chosen
profile, and from then on nullCAT loads that profile by itself whenever
that car runs, with a line in the log. A profile with no binding is
manual only. Profiles live in `profiles.json` beside the rig config.

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
| Longitudinal slip | a wheel locking under braking, or spinning under power, per wheel (below) | per-wheel slip ratio or wheel speeds, road speed; or the single lockup channel |
| Lateral slip | the tyres sliding sideways: the fronts scrubbing wide, the rears stepping out, per wheel (below) | per-wheel slip angles; or the single skid channel |
| Road | the road surface: each corner's bumps replayed from the suspension, per wheel (below); or a texture | per-corner suspension velocity; or the single road channel |
| Limiter | an extra hammer while on the rev limiter | limiter |
| TC pulse | a pulse while traction control cuts | TC active |
| Kerb | kerb strip rumble | kerbs |
| Driveline | a slipping clutch juddering at a launch, the engine lugging at low revs (below) | clutch, rpm, gear (speed, throttle) |

With the SimHub plugin, rpm, speed, gear, clutch, throttle, brake, ABS,
limiter and traction control arrive automatically. The per-wheel slip
channels, road and kerbs need SimHub properties named in the plugin's
settings file (see the plugin README). Any other motion software that
can send a text line can feed the same channels (see PROTOCOL.md).

`freq hz` sets the carrier: lower is a heavier, slower shake, higher is
a finer buzz. `jitter` roughens it so slip, road and kerbs feel like
texture rather than a tone.

### Tyre slip

The two slip tiles work per wheel: each of the four tyres has its own
severity, and the effect plays on whichever axis you route it to with a
**part**: the route editor gives every axis a selector, `ALL` (the
strongest wheel anywhere), `FRONT` or `REAR` (the stronger wheel of that
axle), or one corner (`FL`, `FR`, `RL`, `RR`). On a four-post rig route
each vertical actuator to its own corner and the inside front locking up
judders that corner and nothing else. On a three-actuator rig route the
rear actuator to `REAR`; a belt or shaker takes `ALL`.

**Lateral slip** is the tyres sliding sideways. The fronts give
**scrub**: a fine, fast texture as they push wide. The rears give
**slide**: an irregular, slower chatter as the back steps out. Each has
its own mix and carrier. **peak deg** is the slip angle at which a tyre is
fully gone (7 for most cars; lower for slicks, higher for road tyres);
nothing plays below about 60 % of it, which is normal cornering. As slip
grows the carrier slows and roughens on its own (squeal, moan, shudder)
and the onset is abrupt, because that is what a tyre letting go feels
like.

**Longitudinal slip** is the tread slipping along the road. **lock** is a
wheel turning slower than the car under braking: a heavy judder whose
beat falls with road speed. **spin** is a driven wheel turning faster
than the car: the axle tramping at its own resonance. **peak ratio** is
the slip ratio at full severity (0.8 by default; a locked wheel is -1);
nothing plays inside the first 0.15, where the tyre is still gripping.

When the sim sends wheel loads, the loaded tyre's slip is weighted up and
the unloaded one's down, so the outside tyre in a corner shakes harder.

Where the sim only gives one overall slip value, the single `skid` and
`lockup` channels still work and feed all four wheels at once; the parts
then simply select the same thing. Where it gives wheel speeds but no
slip ratios, nullCAT learns each wheel's rolling factor while you cruise
(brake off, light throttle, above 30 km/h) and works the ratio out
itself, so a sender's wheel-speed unit and staggered tyre sizes need no
setting up.

### Road

With per-corner suspension velocities from the sim (most titles have
them), the Road tile stops making a texture and **replays the road**:
each corner's suspension travel, with the slow body motion that the
motion cue already produces cut away so only the bumps remain, at their
real timing and shape. Route it with a part like the slip tiles: on a
four-post rig each actuator plays its own corner, so a kerb under the
right-front arrives at the right-front. Two settings: **full mm** is the
bump that counts as 100 % amplitude (8 by default; lower for a stiff
race car, higher for a rally car), and **cut hz** is where the slow
motion is cut (2 by default; raise it if the seat follows body roll the
cue is already doing, lower it to let longer undulations through).

Without the per-corner channels the tile plays a texture at `freq hz`
scaled by the single `road` channel, as before.

### Driveline

What the transmission does when the engine and the wheels disagree.
**clutch x / clutch hz** is clutch judder: with the pedal part-way
through its travel and slip across the clutch, the whole driveline
shudders at its resonance (around 10 Hz). nullCAT works the slip out
from the engine speed against what the gear and road speed say it should
be, using the gear ratios it learns while you drive; a launch from rest
is full slip, and more throttle makes it harder. **lug x / lug hz** is
the engine bogged: full throttle at too few revs winds the driveline up
and lets it go in a slow shudder that fades as the revs climb out of it.
Neither plays in neutral. Needs the clutch, rpm and gear channels, with
speed and throttle making it exact.

### The engine

Describe the engine and the effect follows it:

- **cyl / rotors**, **litres**, **layout** (inline, V, flat/boxer,
  Wankel, two-stroke, electric). Bigger cylinders hit harder; more
  cylinders and a V or flat layout run smoother; a Wankel has no shake
  at idle but an uneven beat. A **two-stroke** (karts, old bikes) fires
  every cylinder every rev, hits harder for its size and has no idle
  lope: a 125 cc kart single is cyl 1, litres 0.125, two-stroke.
  **Electric** has no firing at all: just the motor whine from the first
  turn, rising with load (and quieter under regen when you lift), a soft
  limiter, no cranking or stall; the rpm channel is the motor speed.
- **max rpm**: the redline. Leave it at 0 and it is learned while you
  drive (set exactly the first time you hit the limiter).
- **rock x / thump x / buzz x** mix the three parts, 0 to switch one
  off: the block rocking at idle, the individual firings, and the
  vibration that rises in pitch and strength up the rev range.
- **buzz order**: 0 picks the pitch automatically so the redline sits
  at the top of the range (120 Hz on a Pi, lower on a slower PC loop so
  the buzz stays smooth); set it by hand to move it.
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
around 12 Hz so it lands in time with the cuts. The pit limiter cuts
the same way when the sim sends its flag, without touching the learned
redline.

**Lifting off.** When the throttle snaps shut up the band the combustion
stops but the engine keeps shaking: that is **inertia x**, the pistons
and rods reversing, which rises with the square of the revs, stays on a
lift and dissipates as they fall. The lift itself is an event,
**lift-off x**: with **turbo** off a soft pop as combustion stops; with
it on the boost dumping as a whoosh followed by the compressor flutter,
scaled by the boost the sim reports (or by how far up the band you
lifted when it does not). **pops x** adds the fuel-cut crackle on the
overrun for a sports or rally car; leave it at 0 for a road car.

With the throttle shut, in gear and rolling the wheels drive the engine
against closed throttle, which is heavier and rougher than the same revs
falling in neutral.

**Starting and stopping.** On the starter the engine turns slowly with
no combustion: slow compression lumps. It catches on the way up through
about 500 rpm with a lurch and an uneven fast idle for a moment. Below
about 650 rpm a running engine shudders harder as it dies and stops with
one last kick; restart it and it catches again.

If the channel stream stops for half a second, every effect fades out.
The same happens when packets keep arriving but nothing in them changes
for two seconds (a paused sim, or a sender repeating its last frame):
the Devices section then says *frozen* instead of *receiving*, and the
log notes each time the stream starts, stops, freezes or resumes. The
SimHub plugin also goes quiet by itself while the game is paused.

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
