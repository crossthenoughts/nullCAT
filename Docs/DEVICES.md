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

### The car table

A profile is a copy of your rig's whole strip. The car table is the other
half: what belongs to the car. nullCAT ships an entry for every car in
Assetto Corsa (Kunos content and DLC) and Automobilista 2, carrying the
engine's description and character (cylinders, litres, layout, turbo,
rock, thump, buzz, lope, inertia, lift-off, pops, the limiter bounce), the
limiter, the driveline (gearbox type, whine, shunt, clutch, lug) and
whether the real car has ABS and TC, with the amplitudes of those five
tiles. Nothing else: an entry never carries routes, master gain, the pos
budget or the slip, road and kerb tiles. Those are your rig's, and they
stay exactly as you set them when an entry is applied. So the rig-level
trim lives in the routes (a 0.3 gain on the Engine route instead of 0.7)
and the car-level feel moves with the car.

The **car** row on the Master tile shows the car the sim is naming and
what the table has for it (`Mazda 787B · stock`, `· yours` for your own
entry, `· no entry`). **Pull car preset** applies that entry to the five
tiles. **Follow car** (on by default) does the same by itself whenever the
sim names a car the table knows; untick it and the table only acts when
asked. A profile bound to the car wins over the table.

**Preset** is a search box over the whole table: when your car has no
entry, pick the nearest one (another GT3, the same engine in a different
body), **apply** it, tune to taste, then **save for this car**. That files
the saved five tiles under the current car in your own layer
(`cars.local.json` beside the rig config, kept across updates), which
wins over the shipped entry from then on; **forget** drops it. The Engine
tile names the matched car and folds its notes under it: the real-world
figures and reasoning behind the values, there to refer to when tuning.
The shipped table is `cars.json` beside the program; every update replaces
it, so put your own tuning in your layer, not in that file.

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

As a rule of thumb for a seat on linear actuators: what you feel through
the cushion lives below about 20 Hz. With pos budget 0.5 and an axis
limited to 20 m/s^2, an effect gets about 2.5 mm at 10 Hz, 1.8 at 12,
0.9 at 17, 0.6 at 20 and 0.3 at 28 Hz, and below about 0.3 mm a body
stops noticing. The tile defaults meant for position axes sit at or
below 20 Hz for that reason; the finer carriers (a 32 Hz scrub, a 40 Hz
kerb) belong on a belt or a shaker. A route gain above what the axis can
give at the carrier buys nothing: 1 mm at 17 Hz is already the limit
there, 2 to 3 mm only counts below about 10 Hz. A transient such as the
gear shift is a short 60 Hz burst on a belt, which a position axis
cannot carry at all, so there it plays as one jolt instead: a single
cycle at about 12 Hz (longer for a synchro clunk, shorter for a dog
knock) at the route's full amplitude.

### The effects

Each tile lists the sim channels it needs, with a tick when the channel
is arriving and a cross when it is not. A missing channel leaves that
effect silent.

| Effect | What you feel | Channels |
|---|---|---|
| Detent click | the lever dropping into a gate | none (from the shifter itself) |
| Gear shift | a thunk on every gear change (one jolt on a position axis) | gear |
| Engine | the engine running, from idle to the limiter, stuttering when traction control cuts (below) | rpm, throttle, limiter (TC active) |
| ABS | the brake pressure dumped and rebuilt at each corner while ABS works: the car judders and grumbles, slowing towards a stop, with the pump buzzing under it (below) | brake, ABS active (speed) |
| Longitudinal slip | a wheel locking under braking, or spinning under power, per wheel (below) | per-wheel slip ratio or wheel speeds, road speed; or the single lockup channel |
| Lateral slip | the tyres sliding sideways: the fronts scrubbing wide, the rears stepping out, per wheel (below) | per-wheel slip angles, or the sim's per-wheel combined slip; or the single skid channel |
| Road | the road through the tyres and suspension, per corner (below): the sim's suspension movement with the road's fine roughness added, or the body's movement over the road, or the sim's own body movement | road speed, with the road height, surface or suspension per corner; or the body's heave, pitch and roll; or the single road channel |
| Limiter | an extra hammer while on the rev limiter | limiter |
| TC pulse | the body surge of each traction-control cut: a loss of drive and a recovery, each cut a little different (below; the Engine tile stutters as well) | TC active |
| Kerb | the rumble strip under a tyre on a kerb: a hum that rises and falls with speed, a thud on and off (below) | the surface under each tyre, or the kerbs channel, or the road height under each tyre |
| Driveline | a slipping clutch juddering at a launch, the engine lugging at low revs (below) | clutch, rpm, gear (speed, throttle) |

With the SimHub plugin, rpm, speed, gear, clutch, throttle, brake, ABS,
limiter and traction control arrive automatically, and for Assetto
Corsa, Competizione, EVO, Rally and Automobilista 2 so do the per-wheel
channels: the plugin knows which raw fields each of those sims exposes
and sends them by itself, so the tiles have their input with nothing to
set up. Kerb works by itself in Automobilista 2 (the surface under each
tyre), Competizione and EVO (the game's own kerb channel); in Assetto
Corsa set its **detect mm** (below). For another sim the per-wheel
channels, road and kerbs need SimHub properties named in the plugin's
settings file (see the plugin README). Any other motion software that can send a text line
can feed the same channels (see PROTOCOL.md).

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

The two wheels of an axle always move together: a car's body is one
stiff structure, and a locking front corner shakes the whole front with
a bias to that side, never the two front corners against each other. So
both front posts play the same wave, the harder-slipping corner bigger,
and a front-left lock reads as a judder leaning left rather than the rig
rocking side to side.

**Lateral slip** is the tyres sliding. The fronts give **scrub**: a fine
texture as they push wide (20 Hz by default, which a seat actuator can
still carry; a belt or shaker takes a finer one). The rears give
**slide**: an irregular, slower chatter as the back steps out. Each has
its own mix and carrier. **peak deg** is the slip angle at which a tyre is
fully gone (7 for most cars; lower for slicks, higher for road tyres);
the slide starts at **onset %** of it (40 by default; below that is
normal cornering). **ease** shapes the way in: 0 rises straight up to a
full slide at peak deg, 1 (the default) starts as a light scrub and
builds, 2 starts lighter still. **smooth hz** steadies the slip angle the
sim sends (8 by default; 0 = as sent): it arrives about 60 times a
second, and Assetto Corsa's is worked out from the tyres' positions, so
unsmoothed it flickers around the onset and plays in lumps. **attack
ms** is how fast a slide builds (30 by default; lower for a snappier
break-away). As slip grows the carrier slows and roughens on its own
(squeal, moan, shudder).

**Longitudinal slip** is the tread slipping along the road. **lock** is a
wheel turning slower than the car under braking: a heavy judder whose
beat falls with road speed. **spin** is a driven wheel turning faster
than the car: the tread chattering and skipping at the contact patch (16
Hz by default; a car with a live rear axle tramps lower, set spin hz
around 10 for one). **peak ratio** is
the slip ratio at full severity (0.8 by default; a locked wheel is -1);
nothing plays inside the first 0.15, where the tyre is still gripping.

When the sim sends wheel loads, the loaded tyre's slip is weighted up and
the unloaded one's down, so the outside tyre in a corner shakes harder.

Where the sim has no slip angle but a combined slip per wheel
(Automobilista 2's tyre slip speed; Assetto Corsa's wheel slip where its
slip angle is not available), Lateral slip takes that instead. A combined slip cannot be split into sideways and
along, and a spinning or locked tyre is sliding too, so on that path the
Lateral tile plays for any sliding tyre (power oversteer, a lock-up) and
the Longitudinal tile adds its judder or tramp on top. It starts at
**onset %** of `peak %` and is full at `peak %`; the plugin scales Assetto
Corsa's value so that a clear slide (about 6 on the game's own scale)
reads 100. Cars differ, so drive a few hard laps, read the peak on the
tile's chip (`wheelSlip* 12/140`), and set `peak %` near that peak (up to
400). Where the sim
only gives one overall slip value, the single `skid` and `lockup`
channels still work and feed all four wheels at once; the parts then
simply select the same thing. **peak %** there is the channel value that
counts as a full slide (100 by default): a property that only ever
reaches 25 in a burnout gets `peak %` 25 and the tile reaches full
severity like the per-wheel tiles do. Where it gives wheel speeds
but no slip ratios, nullCAT learns each wheel's rolling factor while you
cruise (brake off, light throttle, above 30 km/h) and works the ratio out
itself, so a sender's wheel-speed unit and staggered tyre sizes need no
setting up. Nothing plays below 5 km/h: slip means nothing at a
standstill.

### Reading what the effects are fed

The Test button plays an effect at **full** severity (1.00 on every
wheel), which is the reference, not what a lap gives. To see what a lap
actually gives, each tile reads out its input:

- the channel chip shows the channel actually feeding the tile with its
  live value and its session peak (`skid 23/41 ✓`); for a per-wheel
  group the largest wheel now and the largest peak (`wheelSlip* 12/140
  ✓`). Slip peaks only count while the car is moving, since a car at a
  standstill reports nonsense slip;
- the two slip tiles, Road and Kerb show a per-wheel line, `FL 0.00/0.82
  FR ... ` (now/peak: severity for slip, mm for Road, how much of the
  tyre is on a kerb for Kerb);
- the other continuous tiles show `in 0.12/0.60`, the level they are
  driven with now and at peak;
- the wave carries a tag: `test` during a Test, `live 0.31` while the sim
  drives it. A wave that keeps moving after a Test with nothing felt is
  live input at a small level, not a stuck effect.

The peaks hold until **Reset peaks** in the header (the same button that
re-baselines the drive cards), so you can alt-tab out of the sim after a
slide and read what it did. Only the trim that acts on the live input
path reads as live: with per-wheel data arriving, `peak deg` / `peak
ratio` are it and `peak %` is dimmed; with a single channel it is the
other way round. On Road the settings the chosen model does not use are
dimmed, and on Kerb the ones for the paths not in use; hover a dimmed
setting to see why. Drive, read the peak, set the trim, Save.

### Road

What you feel of the road in a real car comes through the tyres and the
suspension: the road lifts each wheel, the tyre swallows anything
shorter than its contact patch, the spring and damper work, and the body
above moves with what they pass on. The Road tile plays that, above the
band the motion cue already covers, per corner. Route it with a part
like the slip tiles: on a four-post rig each actuator plays its own
corner, so a bump under the right-front arrives at the right-front and,
a wheelbase later, at the right-rear.

**model** picks how it is made. Try them; choose whichever feels more
like the car.

- **suspension** (the default): each corner's suspension movement from
  the sim, replayed, plus the road's fine roughness that a sim's physics
  does not model: a random road laid out along the track, run through
  the tyre and the corner's spring and damper, so it rises with speed by
  itself, reaches the rears a wheelbase after the fronts and stops when
  the car does. **full mm** is the suspension movement that counts as
  100 % (25 by default). **rough x** sets the roughness (1 is a smooth
  public road, 2 by default, more for a bumpy track, 0 for the sim's
  movement alone); where the sim says what is under the tyre
  (Automobilista 2) it is rougher on bumpy tarmac, cobbles, grass, dirt
  and gravel. **body hz** (how fast the body bounces on its springs: 3
  by default for a race car, 1 to 1.5 for a road car), **hop hz** (how
  fast the wheel hops on its tyre, 16) and **damping** (how quickly a
  bounce dies away, 0.3) shape how the corner passes the roughness on.
- **tyre**: the same model of each corner, but playing the body's
  movement over the road under that tyre: the sim's own road where it
  sends one (Assetto Corsa: the height of the road under each tyre, with
  the hills taken out), else its suspension movement, plus the same
  roughness.
- **chassis**: the sim's own body movement, its vertical acceleration,
  pitch and roll, in the same band, spread to the corners by the car's
  wheelbase and track. Which way the sim's pitch and roll turn is learned
  from the suspension in the first few seconds of driving; until then it
  plays the up and down alone, the same at every corner.

A seat actuator is moved by the movement (how far); a belt or shaker
pushes with its acceleration (how hard). For tyre and chassis, **body
mm** is the body movement that counts as 100 % (1 by default): with a
seat actuator routed at a gain of G mm and amp at 100 %, a body mm of G
plays the body's movement one to one. **cut hz** is where the slow
motion is cut (2 by default; raise it if the seat follows body roll the
cue is already doing, lower it to let longer undulations through).
Movements past about 60 % of full bend over a soft knee towards 100 %
instead of clipping, so a big hit lands as a thump rather than a snap.
The per-wheel line shows each corner's movement and its peak in mm.

Where the sim sends nothing the chosen model can use, the tile plays the
single `road` channel's texture instead, at **freq hz**, full at **peak
%**. The suspension model also keeps the older **surface** grain: a rough
texture on its own carrier, **surface hz** (12 by default), rising with
road speed to full by **surface km/h** (100), at the **surface x** mix.
It is off by default (0), since the roughness now does its job.

### Kerb

A rumble strip under the tyre that is on one. The ribs pass under the
tyre every **pitch cm** (25 by default), so they hum at road speed over
the pitch, about 90 times a second at 80 km/h, and the hum climbs and
falls with speed; each rib is a little different (**jitter**). **rise
mm** is the step up onto the kerb (8 by default) and **rib mm** the
ribs' height (3). A seat actuator cannot carry the hum: it gets a thud
as the tyre steps on and another as it steps off, scaled by **full mm**
(the kerb movement that counts as 100 %, 10 by default), and riding
along the kerb does not hold the seat up. A belt or shaker gets the ribs,
stronger with speed, and a kick on and a kick off: the sound a kerb
makes as you come on and off it. On and off are abrupt, as in a car.
Route it with a part so the corner on the kerb rumbles.

Which tyre is on a kerb comes from the sim: the surface under each tyre
(Automobilista 2); else the game's own kerb channel (Competizione, EVO:
all wheels, full at **peak %**); else, in a sim that sends the road
height under each tyre (Assetto Corsa), a tyre standing more than
**detect mm** above its partner on the same axle, beyond what the
track's camber and banking usually give there. Detect mm is 0 (off) by
default; around 15 finds raised kerbs. A flat painted kerb cannot be
found that way. The Test button runs the strip at 80 km/h, on and off.

### ABS and TC

**ABS** works at each corner: the valve dumps the brake pressure and
rebuilds it, about **freq hz** times a second, so the car's deceleration
judders. Four settings shape how it feels:

- **sharp**: the dump's edge, 0 a round wave, 1 a hard knock (0.3 by
  default).
- **spread**: each corner runs on its own rate and each cycle is a
  little different, so the four drift in and out of step and the car
  grumbles rather than beating like a metronome (0.5 by default; 0 = the
  four in step, one clean pulse).
- **slow x**: the cycle slows as the car slows, the thump-thump just
  before a stop (0.5 by default: half rate at a standstill, full from
  80 km/h; 0 = never).
- **buzz x / buzz hz**: the pump and valves underneath, a fine rough
  buzz (0.3 at 40 Hz by default). A seat actuator can barely carry it; it
  is for a belt or a shaker.

Route ABS with a part like the slip tiles: a post on one corner plays
that corner's valve; an axle or all plays its corners together, the full
pulse when they are in step and less as they drift apart, as the car's
body feels the four wheels' braking added up.

Tick **slip link** to tie ABS to the slip tiles. While ABS works, each
corner's valve then shapes that wheel's slip: the lock texture on the
Longitudinal tile stays at the edge of locking (ABS never lets the wheel
lock) and surges with every cycle, by **lock x** (0.7 by default); the
Lateral tile's scrub and slide ease after each dump, when the tyre gets
its grip back sideways, by **scrub x** (0.3). Spin is left alone. It works
with the ABS tile's own amp at 0 too, if you want ABS felt only through
the tyres. The sim's slip data arrives too slowly to carry the valves'
rhythm itself, so this puts it back.
**TC pulse** is the body surge of each traction-control cut: drive lost,
then back, about **freq hz** times a second, with the same **sharp** and
**spread** (each cut a little different in timing and depth).
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

**gearbox** says what kind of box the car has and shapes the Gear shift
tile's thunk: a *synchro* road box clunks softly and a little longer; a
*dog* box (sequential or an H-pattern dogbox) knocks hard and short,
and a shift under power gets a second knock a moment later as the dogs
engage. **whine x** is straight-cut gears meshing: a tone
whose pitch follows the input shaft and steps on every shift, loud under
load and quieter on the overrun; leave it at 0 for a helical road box.
**shunt x / shunt hz** is the driveline's backlash taking up when the
throttle snaps open or shut while rolling in gear, one knock each way,
harder in a dog box; a slow throttle change makes no knock.

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

**Traction control** is the same mechanism, shallower: while the sim
says TC is cutting, the engine drops a share of its firings in
irregular bursts (about 15 a second), more of them the harder you are
on the throttle, and catches each time a burst ends. That stutter is
what a race car's TC feels like; the **TC pulse** tile adds the body
surge of each cut on top (route it to surge and the belt).

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
The SimHub plugin goes quiet by itself while the game is paused, which
is what makes that work. A sender that cannot go quiet and keeps
repeating its last frame through a pause can be covered by the
frozen-stream guard: `ncxFrozenMs` in host.json (off by default) treats
a stream whose values have not changed for that many milliseconds as
stopped; the Devices section then says *frozen* instead of *receiving*.
It is off by default because a car idling at a standstill sends exactly
the same numbers for as long as it idles, and the guard would mute the
engine after two seconds in the pits.

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
