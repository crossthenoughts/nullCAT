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
  `integrations/simhub/` in the repository. It sends the channels it can
  read from the running game, out of the protocol's 86 (protocol 1.9;
  the list grows as titles are added, see `Docs/PROTOCOL.md`). The
  per-wheel ones are built in for Assetto Corsa, Competizione, EVO, Rally,
  Automobilista 2 and Le Mans Ultimate; other games need a property bound
  in its settings file (see its README).
- **FlyPT Mover / other tools:** any tool that can compose a text UDP
  line from telemetry fields works. Send
  `NULLCATX,<rpm>,<speedKmh>,<gear>,<clutchPct>,<throttlePct>` at
  100 Hz or more (clutch: 0 = pedal up, 100 = floored), adding the
  haptic channels in the order given in `Docs/PROTOCOL.md` if your
  tool can supply them.

**The bindings.** Only a sender that uses the numbered `NULLCATX` line
alone needs them (FlyPT Mover and the like): the SimHub plugin also sends
every channel by name, so there is nothing to bind for it. The default
bindings follow the order in `Docs/PROTOCOL.md`; change them only if
your sender uses a different order, in the Sim channels section at the
bottom of the web Setup view (folded until you open it; it has a Reset
to defaults button). The Devices section shows whether the stream is
being received.

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
position axis online), or the shaker's sound card open, to be felt.
**Test** plays the effect for a moment and, if nothing could be felt,
says why: amplitude 0, no route, or nothing it routes to is live. The tile's waveform only moves while the
effect is actually producing force, and its shape follows the amplitude
and frequency you set.

Each effect's row says what it is doing, under its name, with a lamp:
**playing**; **ready** (fed, nothing to play right now); **armed · none
yet** (an event that has not happened this session, such as ABS on a car
you have not braked hard in, or one that has none); **no input** (this
game does not send what the effect needs, named); **not in this game**
(dimmed: the Rumble strip in a game that does not say when a tyre is on a
kerb); **unrouted** (turned up, routed nowhere); **off**
(amplitude 0); **muted**; **no game** (no telemetry arriving). The
events (TC, ABS, limiter, kerb, impacts, gear shift, detent click) also
show how many times they have gone off and how long ago the last one was,
counted since **Reset peaks** or the loop start. A bar under each wave
shows how much the effect is putting out, with the recent peak. The
**Now** line at the top of the panel counts what is playing and names
anything that needs a look. The effects are grouped by what sets them:
**Car** (the car table: engine, limiter, driveline, TC, ABS), **Type of
car** (the starting point: the slips, road, wheels, surface, kerb and
impacts), then **Other**.

**Master** scales every effect at once. Each tile has an **M** button in
its corner that mutes just that effect: it goes silent, but its settings
and readouts stay as they are and no other tile changes. **Mute** on the
Master tile mutes every tile and **Unmute** brings them all back, so to
hear one effect on its own, press Mute, then that tile's M. Mutes are not
saved: a restart brings everything back.

An effect's editor also carries a small **sim dot** beside its state.
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
take the same effects as the axes. On the Pi, in the web page under
**Setup**, **Shakers / audio**, press **Shakers** (top right) so it reads
on, name **Card 1** (part of its name; **(default)** for the system
default; **no hardware** for a bench with no card) and how many of its
outputs are shakers (**ch**); a second card (a second stereo dongle is
the usual way to four shakers) goes in **Card 2**, its channels numbered
after the first card's. Each row shows the shakers its channels become.
On the PC the same settings are in the desktop app's **Application
Settings**, under **Shakers**: pick each card from the list (the sound
outputs Windows has, by name, so a headset and a shaker card are told
apart) and its channel count. Restart nullCAT, and the status under the
cards names each one with its sample rate, latency, buffer and underrun
count, and every shaker's strip has a play key (40 Hz for a second) to
check which shaker is which.

On Windows nullCAT takes each shaker card for itself (exclusive mode):
no Windows mixer, no app volume, nothing else playing through it, and a
shorter buffer. It also holds the card at full volume, unmuted, every
channel, the way the Pi sets a card to full line level, and puts it back
if anything turns it down or mutes it while nullCAT runs. Set the volume
on the amps. Shakers that feel faint on Windows are worth checking for
another program using the card: SimHub was found binding a shaker card
on some starts, even with it no longer bound there, and muting it.
Before the first run, in Windows:
- the card must be allowed for apps (Settings, Sound, the card, **Allow
  apps and Windows to use this device for audio**), or nullCAT cannot
  open it;
- in the classic Sound panel (`mmsys.cpl`), set the card up with at least
  as many channels as you have shakers (**Configure**: quadraphonic, 5.1
  or 7.1); exclusive mode plays the card in the format set there
  (**Properties**, **Advanced**, where **Allow applications to take
  exclusive control** must stay ticked).

The shaker line shows **[exclusive]**, or **[shared]** with the reason
when another program holds the card (then Windows' mixer and volumes
apply again). At start the log says what the card was at when nullCAT
set it to full, and the shaker line counts any time it was turned down
while running: a count that keeps climbing means another program is
still fighting over the card.

Every effect's route editor then lists **Shaker 1..N** below the axes.
A shaker route has a gain (1 = full scale at 100 % amplitude; the output
soft-clips, never clips hard) and a **harmonic**: x1 plays the effect's
carrier as it is, x2 doubles it, and so on, phase-locked to the same
oscillator. That is how a 9 Hz belt effect (or a 12 Hz ABS cycle) lands
on a shaker at 18 Hz (36 Hz at x3) where the shaker has output, while
the belt keeps its own rate. Road, Rumble strip, Surface, Wheels and Impacts
have no single carrier to multiply: their shaker routes play as they
are (the editor shows "as is"). On a per-corner tile a shaker route also
takes a part, like an axis route: give each shaker its own corner, an
axle, or all of them.

The shaker path is a little behind the belts (the card's buffer plus
the hand-off, about 28 ms on a typical USB card; the shaker line shows
it). **axis delay ms** on the Master tile
holds the axis effects back by that much so everything lands together;
the motion cue itself is never delayed. Shakers keep playing with the
control loop stopped and on a rig with no drives at all, so a shaker
install on its own is a complete haptics engine for the sim channels.
Underruns in the status line mean the sender or the machine stalled;
the chain holds the last sample and fades rather than clicking.

The channel count on the status line is the card's own. Ask a stereo
dongle for four shakers and it reads "2 of 4 channels: shakers 3-4
silent": shakers 3 and 4 on that card are not live (the route editor
says so) and stay silent rather than playing into shakers 1 and 2. If a
card goes away while nullCAT runs (unplugged, disabled, or the system
output moved to another device), the status line says so and the
shakers on it stay silent; the other cards play on. Once the card is
back, press **Reopen** beside the cards' status (it shows while a card is
not open) to open every card again with the same settings, no restart.

#### Shaping each shaker

Each shaker has its own EQ and compressor, in the web page under
**Setup**, **Shakers / audio**. Every shaker has a strip there: its
level in dBFS, the gain its compressor is taking off beside it (amber,
filling from the top), its EQ curve, its 40 Hz test (the play key), and
an **EQ** and a **CMP** key that put its EQ or its compressor in or out
(lamp lit: in). Click a strip to open that shaker's editor below the
strips, the EQ and the compressor side by side (one above the other on a
narrow screen); the arrows step to the next shaker.

- **EQ**: five bands, drawn as one curve. Drag a numbered point to move
  its band (across for the frequency, up and down for the gain), roll the
  wheel over the curve to make the selected band narrower or wider,
  double-click a point to switch its band on or off, or type in the
  band's card under the curve. A band is a bell, a low or high shelf, or
  a high-pass or low-pass (12 dB an octave). A flat EQ starts with the two end bands
  off and three bells at 0 dB.
- **Compressor**: threshold, ratio, knee, attack, release and makeup,
  with its curve, a dot on it where it is working now and the gain it is
  taking off. It is made for bass:
  its release (200 ms to start with) is longer than a bass cycle, so it
  turns a passage down rather than bending each wave.
- **Test**: the 40 Hz burst, a slow sweep from 10 to 200 Hz over ten
  seconds, and a tone at the selected band's frequency that follows the
  band while you drag it. They play through the shaker's EQ and
  compressor, so you feel what you set, and a mark on the curve shows
  where the sweep or the tone is.

A way to start: play the sweep, note where the shaker rings or booms,
put a bell there and cut it until the sweep feels even. A high-pass just
under the lowest frequency the shaker plays well keeps it from flapping
on what it cannot play. Then the compressor brings the small effects up
against the big hits (a lower threshold and some makeup), or just keeps
the big hits from bottoming the shaker.

Changes play on the shaker as you make them, and **Save** keeps them,
with no restart or re-initialize. Leaving the page, or reloading it,
with changes unsaved puts the saved settings back. **Hold to bypass**
plays the shaker without its EQ and compressor while you hold it, to
compare. **Reset EQ** and **Reset compressor** put each back to where it
starts, out. **Copy to** copies one shaker's EQ and compressor to others
(a pair of seat shakers, say): click the shakers to copy to in the
strips, then **Copy**. The settings belong to the rig, by shaker number,
and the PC sets them in the web page too (its sound cards are set in the
desktop app; the web page shows them). On the PC, with its
500 Hz loop, nothing above 250 Hz reaches the shakers: the curve shades
that part.

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

### Starting points

A profile carries your routes, so it only fits your own rig. A starting
point fits any rig: it sets only what depends on the type of car, and
nullCAT ships four types, on the **Type** row at the top of the Haptics
panel, each
in two versions: one tuned on Assetto Corsa (also the one for other sims
until they get their own) and one for Automobilista 2, marked **(AMS2)**.
Automobilista 2's tyres reach their grip at larger slip angles, so its
versions size the slides up (peak deg 1.2 to 1.4 times); everything else
is the same. Its suspension also moves about half as far as Assetto
Corsa's, so with the same full mm its road plays softer: try **learn full
mm** on the Road tile.

- **Road car**: a street car on road tyres. Soft, long-travel suspension,
  slides that build gradually, iron brakes.
- **Cup car**: a one-make racer on semi-slicks (Porsche Cup, Ginetta,
  classic touring). Firmer, slides earlier, still forgiving.
- **GT3 / GT**: GT3, GTE and prototypes on slicks. Stiff, aero, sharp
  slides, steel brakes. The middle ground to tune from.
- **Open wheeler**: single-seaters and formula cars on slicks. Very stiff,
  little travel, grip lets go at small slip angles, carbon brakes.

Each one sets how the tyres let go (Lateral and Longitudinal slip), how
the road and kerbs move the car (Road, Rumble strip), the wheels' shake and brake
judder (Wheels) and the ring of a knock (Impacts). Picking a type shows
what it is for beside it. Everything else stays exactly as it is: your routes, master gain, pos
budget, and the engine, limiter, driveline, ABS and TC (those come from
the car table). So pick the type closest to what you drive, **apply** it,
then pull the car from the car table for its engine, gearbox, ABS and TC,
and tune from there. Applying writes the settings to your rig config at
once, like loading a profile; if you have unsaved edits on the page it
asks first. The starting points are `presets.json` beside the program;
every update replaces it.

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
and the car-level feel moves with the car. On the tiles, the settings a
car's entry can set have a faint tint (your own saved entries carry all of
them; the shipped ones all but the diff's strength and the engine's TC
cut). The settings a starting point sets (the type of car: how the tyres
let go, the suspension, the kerbs, the wheels, the knocks) have a warm
tint of their own. Everything untinted stays put when the car or the
type changes.

The car readout at the top of the Haptics panel shows the car the sim is
naming and what plays for it, in small tags: `stock` or `yours` (whose
entry it is) with `applied` or `not applied`, `near match`, or `no
entry`; and for a car the game describes, `engine from the game` and
`type Road car from the game` (`type Rally tarmac (yours)` once you apply
a type by hand). The **Type** box shows the type in force for the car.
The **Car** row under it has **Re-apply**,
which applies that entry to the five tiles again, and **More** with
**save as mine** and **forget mine**. **Follow car** (on
by default) does it by itself whenever the
sim names a car the table knows; untick it and the table only acts when
asked. A profile bound to the car wins over the table. If the table (or a
starting point, or a profile) moves the tiles while you have unsaved edits
on the strip, the page keeps your edits and says so; Save then refuses,
reloads what the controller has, and asks you to make the edits again,
rather than write your older copy over what was applied. A save of your
own that moves a setting the car owns clears the car row's `applied` (the
tiles are yours now, not the entry's: the row says `not applied`) until
you re-apply it.

A sim sometimes names a car with something after a name the table knows:
a version letter or a year (Automobilista 2 has a `MINI Cooper S 1965 B`
beside its `MINI Cooper S 1965`). When there is no entry under the exact
name, nullCAT takes the entry the name starts with, the longest one, in
whole words and at least two of them, and only from the same game, and the
car row adds `· near match`. **Save as mine** then files your own under
the sim's exact name.

With follow car on, a car the table has no entry for at all, in any game
(one the table covers, or one it has no cars for, such as BeamNG.drive),
does not keep the last car's engine, gearbox, ABS and TC: those five
tiles go back to their defaults: a generic engine (a 2 litre inline
four), gearbox, ABS and TC, each tile at the strength the cars in the
table that have it play it at (ABS and TC play only when they are
working). So an Assetto Corsa car without ABS never leaves the next
game's car with its ABS off. Where the game's own sender says what the
car is (the BeamNG.drive mod does), its engine goes on top of the
defaults: the cylinders, V, inline, boxer or rotary, the size, the turbo
and how fast the limiter cuts, so a V8 plays as a V8; and its type of
car (road, track or cup, race, rally gravel or tarmac) is applied as the
starting point. Both happen once per car: change them and they stay as
you set them. The car row then reads `no entry · engine from the game ·
type Road car from the game`, and `type Rally tarmac (yours)` once you
apply another type. After a restart the same car keeps its
tiles as they are, so what you tuned for it by hand stays, whether the
table gave it its entry or the defaults: nullCAT remembers the car it
last acted on in `carstate.json` beside the rig config (machine state,
safe to delete), and the car row reads as it did before the restart. A
different car is a car change as ever. The car row of a car on the
defaults, with nothing from the game, reads `no entry · defaults`, and
the settings the car table owns are
tinted pink instead of the usual faint tint (hover one to read why) until
you apply a car or save as mine, or the next car comes on. The car still
plays, just not as itself: give it a car like it from the **another car**
box, or set the tiles by hand and save as mine.

**Another car** is a search box over the whole table: when your car has no
entry, pick the nearest one (another GT3, the same engine in a different
body), **apply** it, tune to taste, then **save as mine**. That files
the saved five tiles under the current car in your own layer
(`cars.local.json` beside the rig config, kept across updates), which
wins over the shipped entry from then on; **forget mine** drops it. The Engine
tile names the matched car and folds its notes under it: the real-world
figures and reasoning behind the values, there to refer to when tuning.
The shipped table is `cars.json` beside the program; every update replaces
it, so put your own tuning in your layer, not in that file.

### Recording a session

**record** on the Master tile writes every telemetry packet as it
arrives to a file on the controller, `~/nullcat-recordings` on the Pi (a
`recordings` folder beside the rig config on Windows), named by the time
it started; **stop** closes it. A session recorded this way can be played
back exactly, so a feel that is wrong on one corner of one track can be
looked at packet by packet. The tile shows the time, size and packets
so far. The control loop only hands each packet over; a thread of its
own does the writing, and if it ever falls behind it leaves packets out
of the file (counted on the tile as dropped) rather than hold anything
up. It needs 1 GB free to start and stops by itself at 2 GB.

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
| Rumble strip | the rumble strip under a tyre on a kerb: a hum that rises and falls with speed (below) | the surface under each tyre, or the game's own kerb channel |
| Driveline | a slipping clutch juddering at a launch, the engine lugging at low revs (below) | clutch, rpm, gear (speed, throttle) |
| Surface | what the ground does under each tyre: stones on gravel, snow crunching, studs on ice, puddles dragging and floating the tyre (below) | the surface under each tyre, speed (rain or wetness) |
| Wheels | each wheel turning: slightly out of balance, flat spots from lock-ups, hot brake discs juddering, through the tyre and suspension (below) | speed (per-wheel slip ratio or wheel speeds, brake) |
| Impacts | the edge of a hit: the bump stop, a landing, a kerb strike, a top-out, a wall or another car, as a knock on a belt or shaker and a short jolt on a seat actuator (below) | the suspension per corner, or the body's accelerations |

With the SimHub plugin, rpm, speed, gear, clutch, throttle, brake, ABS,
limiter and traction control arrive automatically, and for Assetto
Corsa, Competizione, EVO, Rally, Automobilista 2 and Le Mans Ultimate so
do the per-wheel channels: the plugin knows which raw fields each of
those sims exposes and sends them by itself, so the tiles have their
input with nothing to set up. Rumble strip works by itself in Automobilista 2,
Le Mans Ultimate and BeamNG.drive (the surface under each tyre), and in
Competizione and Rally (the game's own kerb channel); plain Assetto Corsa
does not say when a tyre is on a kerb, so it has none. For another sim the per-wheel
channels, road and kerbs need SimHub properties named in the plugin's
settings file (see the plugin README). Any other motion software that can send a text line
can feed the same channels (see PROTOCOL.md).

`freq hz` sets the carrier: lower is a heavier, slower shake, higher is
a finer buzz. `jitter` roughens it so slip, road and kerbs feel like
texture rather than a tone.

A tyre in the air goes quiet: no road, surface, kerb or slip through
that corner, and nothing from the Wheels tile, until it lands. nullCAT
knows a tyre is in the air when the sim says so (Automobilista 2) or when
it carries next to none of its usual load (Assetto Corsa, EVO, Rally, Le
Mans Ultimate). It needs no settings.

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

**learn onset** (on by default) finds where this car's tyres let go
instead of using one onset for every car. nullCAT watches the car's
sideways g against how far its tyres slip: the g rises with the slip up
to the tyres' limit and then stops rising, and that limit is learned for
the fronts and the rears apart. Every car has its own (2 to 13 degrees
on the recordings, more on a combined slip), so one fixed onset plays too
early in some cars and too late in others. The slide then starts at
**learned onset %** of the limit (80 by default: a light scrub as the
tyres reach it) and is full at **learned full x** it (2 by default). It
learns in steady corners above 60 km/h and trusts a limit only once you
have driven past it a few times, usually within the first minute or two;
until then onset % and peak deg play, and so they do with learn onset
off. The learn onset row shows the limit found for each axle. It needs
the car's g and turn rate from the sim; a game whose sideways g does not
match the car's turning is not learned from, and the row says so. A
different car is learned afresh.

**grip budget** (on by default) treats each tyre's grip as one budget
shared between cornering and braking or driving, as a real tyre's is.
Where the sim sends both a slip angle and a slip ratio (or wheel speeds:
Assetto Corsa, Competizione, EVO, Rally), a wheel starts to slide when the
two together reach its limit: braking into a corner or powering out of
one slides the tyre earlier than either alone would. The slide then plays
on this tile or on Longitudinal slip by which way the tyre is sliding.
Pure cornering or pure braking feels as before. Untick it to compare.

**Longitudinal slip** is the tread slipping along the road. **lock** is a
wheel turning slower than the car under braking: a heavy judder whose
beat falls with road speed. **spin** is a driven wheel turning faster
than the car: the tread chattering and skipping at the contact patch (16
Hz by default; a car with a live rear axle tramps lower, set spin hz
around 10 for one). **peak ratio** is
the slip ratio at full severity (0.8 by default; a locked wheel is -1);
nothing plays inside the first 0.15, where the tyre is still gripping.
**learn lock onset** (on by default) moves that point for tyres that
brake best with more slip: on gravel, dirt or snow a tyre keeps braking
harder up to 0.1 to 0.25 of slip, where on tarmac it peaks well under
0.1. nullCAT learns this car's limit from straight braking and starts the
lock at 1.5 times it, never earlier than 0.15: on tarmac nothing changes,
and on loose ground the lock no longer plays through ordinary hard
braking. The row shows the limit found.

**edge** is that grip at work: a fine grain while a tyre is braking or
driving hard but short of letting go, so you feel the rear putting the
power down, or the fronts close to locking, before anything slips.
**edge from %** is the slip ratio it starts at (3 % by default) and it is
full at 15 %, where lock and spin take over; lower starts it sooner
(slicks work at less slip than road tyres). **edge x** is its mix (0.3 by
default, 0 = off) and **edge hz** its carrier (35 Hz).

When the sim sends wheel loads, the lighter tyre of each axle plays
weaker by its share of the loaded one's load, so the outside tyre in a
corner shakes harder. The loaded tyre is never weighted up, so a sim that
sends loads plays as strong as one that does not.

Automobilista 2 and Le Mans Ultimate have no slip angle, only how fast
each tyre slides over the road. nullCAT takes out the part that is along the road (the wheel
locking or spinning, and the small slip every tyre shows rolling down a
straight), turns what is left into a slip angle against the road speed,
and plays it like any sim's slip angle, so **peak deg** and **onset %**
mean the same. nullCAT learns each tyre's straight-line slip on the
straights, a few seconds of steady driving above 60 km/h with that tyre
neither driving nor braking, and also any small angle left over (toe), so
the tile plays what the corner adds and stays quiet on the straight.
Below 25 km/h the slide fades out (gone by 10), and a wheel spinning or
locking hard has no angle: there that sim's numbers say little. The tile shows each
tyre's angle as played and, after the slash, the straight-line angle it
has learned (`FL 4.2°/1.8`), with how many seconds of straights it has
learned over. It reads **learning** in amber until it has 6 seconds. A
baseline that stays 0, keeps climbing, or a tile that plays on a
straight, means it is not learning right: worth a recording.

**baseline** picks how that straight-line angle is had. **learn** (the
default) keeps learning it on every straight; once learned it only creeps
up and drops back quickly, so a long fast corner moves it little.
**learn once** learns it for each car and then holds it, so nothing later
in the session can move it. **fixed** learns nothing and takes off
**base deg F** on the fronts and **base deg R** on the rears: read them
off the tile after a learned run. Fixed is one value for every car, and
cars differ, so it suits testing or a session in one car. **relearn** on
the tile throws the learned angle away and learns it again from the next
straights; a new car does that by itself.

Where the sim has no slip angle but a combined slip per wheel
(Assetto Corsa's wheel slip where its slip angle is not available),
Lateral slip takes that instead. A combined slip cannot be split into sideways and
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
cruise (brake off, light throttle, clutch fully in or out, above 30 km/h,
each axle's wheels turning together) and works the ratio out itself, so a
sender's wheel-speed unit and staggered tyre sizes need no setting up.
(Tyres of different sizes left and right, as on an oval car, take a
little longer: about 20 seconds of cruising.) Once learned, a moment that
disagrees with it (a slide, a reset, a curve) is ignored unless the same
wheel keeps disagreeing the same way, 5 seconds more than it agrees (a
wrong first value); then it is learned until it agrees again, and engine braking
teaches the driven wheels nothing (nullCAT finds which wheels are driven
from which ones run ahead under full throttle). Nothing plays below 5
km/h: slip means nothing at a standstill.

### Reading what the effects are fed

The Test button plays an effect at **full** severity (1.00 on every
wheel), which is the reference, not what a lap gives. To see what a lap
actually gives, each tile reads out its input:

- the channel chip shows the channel actually feeding the tile with its
  live value and its session peak (`skid 23/41 ✓`); for a per-wheel
  group the largest wheel now and the largest peak (`wheelSlip* 12/140
  ✓`). Slip peaks only count while the car is moving, since a car at a
  standstill reports nonsense slip;
- the two slip tiles, Road, Rumble strip, Wheels and Impacts show a per-wheel
  line, `FL 0.00/0.82 FR ... ` (now/peak: severity for slip, mm for
  Road, how much of the tyre is on a kerb for Rumble strip, the flat spot in mm
  for Wheels, the knock still ringing and the hardest knock for
  Impacts). On Road a tyre in the air reads `air`;
- Surface shows what each tyre is on as nullCAT reads it: `FL gravel`,
  `FR snow`, `RL tarmac wet`, `RR tarmac float` (aquaplaning) or `air`.
  If the sim says snow and the tile says tarmac, the channel is not
  arriving;
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
dimmed, and on Rumble strip the ones for the paths not in use; hover a dimmed
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
  100 % (25 by default). **learn full mm** (off by default) learns it
  for each car instead, from how far its suspension moves while you drive
  above 30 km/h: after 20 seconds of driving it plays the learned value,
  shown on that row, and full mm is greyed (yours is kept for when it is
  off). Cars differ a lot here, and so do sims: once the slow body
  movement is cut away, an Automobilista 2 car moves about half as far as
  an Assetto Corsa one, so with one full mm its road plays at half
  strength. Learned, every car plays its bumps at a similar strength
  (set so Assetto Corsa's Porsche GT3 R comes out at about the 29 mm of the GT3
  starting point, with its cuts). It learns from the track too: a bumpy one
  teaches a larger full mm than a smooth one, so a bumpy track plays a
  little softer than it would with a fixed value. It remembers about the
  last five minutes of driving, so it follows a change of track; a new car,
  or a change of cut hz or susp cut hz, starts it over. **rough x** sets the roughness (1 is a smooth
  public road, 2 by default, more for a bumpy track, 0 for the sim's
  movement alone); where the sim says what is under the tyre
  (Automobilista 2) it is rougher on bumpy tarmac, cobbles,
  dirt and gravel, undulates softly on grass, fills in on snow and is
  glassy on ice; water on the road smooths it (the Surface tile's
  **smooth x**), and a tyre floating on standing water has no road under
  it at all. **body hz** (how fast the body bounces on its springs: 3
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
mm** is the body movement that counts as 100 % (12 by default). The
body moves several mm over a road while a seat actuator's haptic range
is about a mm, so this scales it down; raise it if the posts sit at
their limits (with a seat actuator routed at a gain of G mm and amp at
100 %, a body mm of G would play the body's movement one to one).
The tyre model joins the sim's samples with straight lines and takes as
much of the sim's own road as their rate can carry (about a quarter of
it: 15 Hz from a 60 Hz feed), the roughness filling in the rest, so a
faster telemetry feed brings more of the real road by itself. **cut hz** is where the slow
motion is cut (2 by default; raise it if the seat follows body roll the
cue is already doing, lower it to let longer undulations through).

For suspension and tyre, the sim's own movement (its suspension, or its
road under the tyre) and the roughness can be set apart. **susp %** is
how much of the sim's movement plays (100 by default; 0 leaves the
roughness alone). **susp cut hz** is a steep cut on the sim's movement
only (0, off, by default): set it above the car's body bounce (3 to 4
for a road car, 5 to 7 for a GT3, 7 to 9 for a single-seater) and the
body's slow dive, roll and crests, which the motion already gives you,
drop away while the track's bumps and the roughness stay. Cut hz is a
gentle cut on everything; susp cut hz removes far more of the slow
movement for the same effect on the bumps.
Movements past about 60 % of full bend over a soft knee towards 100 %
instead of clipping, so a big hit lands as a thump rather than a snap.
The per-wheel line shows each corner's movement and its peak in mm.

Where the sim sends nothing the chosen model can use, the tile plays the
single `road` channel's texture instead, at **freq hz**, full at **peak
%**. The suspension model also keeps the older **surface** grain: a rough
texture on its own carrier, **surface hz** (12 by default), rising with
road speed to full by **surface km/h** (100), at the **surface x** mix.
It is off by default (0), since the roughness now does its job.

### Rumble strip

The rumble of a kerb's ribs under the tyre that is on one: the buzz you
hear and feel as you run over a rumble strip. The ribs pass under the tyre
every **pitch cm** (25 by default), so they hum at road speed over the
pitch, about 90 times a second at 80 km/h, and the hum climbs and falls
with speed; each rib is a little different (**jitter**). **rib mm** is
the ribs' height (3 by default).

The car riding up onto the kerb is your motion's: the sim moves the car
and your motion software carries it. A seat actuator cannot carry the
hum: it gets a small bump as the tyre goes on and another as it comes off,
scaled by **full mm** (the kerb movement that counts as 100 %, 10 by
default), and riding along the kerb does not hold the seat up. A belt or
shaker gets the ribs, stronger with speed, and a kick on and a kick off.
On and off are abrupt, as in a car. Route it with a part so the corner on
the kerb rumbles.

It plays only where the game says when a tyre is on a kerb: the surface
under each tyre (Automobilista 2, Le Mans Ultimate, BeamNG.drive), else
the game's own kerb channel (Competizione, Rally: all wheels, full at
**peak %**). A game that flickers its kerb surface on and off along a
kerb, as Le Mans Ultimate does, still plays one rumble. A game that says
neither (plain Assetto Corsa) has no rumble strip: the tile is dimmed
and reads **not in this game**, and its kerbs come through the motion,
Road and Impacts. The road heights cannot tell a kerb from the road's
own camber at speed, so they are not used for it. The Test button runs
the strip at 80 km/h, on and off.

### Surface

What the ground does under each tyre beyond the road's shape, per
corner (route it with a part like Road).

- **stones x** (0.6): on gravel, sand and dirt the tyres strike stones,
  more of them the faster you go, each a size of its own and now and then
  a big one, and harder with speed (full from 40 km/h; none under
  2 km/h): a sharp tick on a belt or shaker, a small bump on a seat
  actuator.
- **stones per m** (0.35): how many a tyre strikes per metre of gravel
  (sand and dirt fewer, in proportion). Around 0.3 each stone is its own
  tick; past 1 they run together into a rumble at speed.
- **crunch x** (0.5): snow compacting under the tread (sand lighter, mud
  a low squelch), rising with speed.
- **studs**: tick it for studded tyres; on ice and snow the studs bite
  with a rough buzz at speed.
- **puddle x** (0.6): on a wet road the puddles lie along it, the same
  puddle every lap, the rears meeting it a wheelbase after the fronts.
  The water drags at a tyre as it enters (a tug, stronger with speed);
  above **aqua km/h** (90) in standing water the tyre floats and the road
  under it goes quiet, then bites as it comes out of the water.
- **smooth x** (0.7): how far water fills the Road tile's texture.

How wet the road is comes from the sim: its wetness per tyre or for the
track where it sends one, else the rain falling, from which nullCAT lets
the water stand on the road (about a minute of heavy rain to stand, a few
minutes to dry). Automobilista 2 sends the surface under each tyre (snow
and ice included) and its rain; Le Mans Ultimate the surface under each
tyre, its rain and how wet the track is; Competizione its rain and how wet
the track is (no surface: its puddles play, its stones and crunch do not). Test runs gravel, then snow, then a wet
road, at 80 km/h.

### Wheels

What each wheel does as it turns. Nothing here knocks once a revolution
on its own: what each wheel does runs through that corner's tyre, spring
and damper (the Road tile's **body hz**, **hop hz** and **damping**),
the way a car passes it on, so you feel a smooth shake at the wheel's
own rate.

- **balance x** (1): the wheel slightly out of balance, as any wheel is,
  and a touch out of round. The shake grows steadily with speed: barely
  there in town, clear at motorway speed and above. 0 is a perfect
  wheel; raise it for a wheel that has lost a weight or taken a knock.
- **flat x** (1): a wheel that locks and slides grinds a flat spot into
  the tyre where it slid, deeper the further and harder it slid (a
  second locked at 70 km/h is a clear one). From then on it thumps
  softly once a revolution, wearing slowly round over the next tens of
  kilometres. A wheel kept at the edge of locking by ABS grinds
  none. The tile's per-wheel line shows each tyre's flat in mm.
- **judder x** (0.5): brake discs made hot by hard braking pulse the
  braking force twice a revolution while you brake. The heat builds
  from how hard and how fast you brake and cools as you drive, so it
  shows after repeated heavy stops, not on the first. It is a push fore
  and aft: route it to a belt, surge or a shaker.
- **puncture x** (1): when the sim says a tyre is punctured (Automobilista
  2, Le Mans Ultimate), the wheel rides on its folded sidewall: a heavy thump once a
  revolution.
- **bite x** (0.5) and **drag x** (0.3): the pads on the turning disc.
  The bite is the grab as the pedal goes on, or is squeezed harder: a
  coarse grain that swells and fades over a quarter second. The drag is
  the rub that stays while you brake, with the pedal. Each has its own
  pitch at 100 km/h, **bite hz** (35) and **drag hz** (60), following each
  wheel's own speed, so it sinks as the car slows and the corners drift
  apart; each cycle is a little different, so it reads as friction, not a
  tone. Both are stronger at speed and on the loaded tyres (the fronts
  under braking), and a locked wheel gives nothing: its disc is not
  turning under the pads. They act at the caliper rather than through the
  suspension: a belt or shaker gets them as they are, a seat actuator a
  fine movement of its own (a bite hz of 25 to 40 is what a post can
  carry).

Each wheel turns at its own speed (from the slip ratios or wheel speeds
the sim sends), so the corners drift in and out of step: route with a
part so each seat actuator plays its own corner; a route carrying
several corners plays them together and beats as they drift. Fresh
tyres and cold discs come with a different car; a pause in the pits, or
a restart from the grid in the same car, keeps your flats (only ten
minutes without the sim at all starts the tyres over). **fresh tyres**
on the tile wipes the flats and cools the discs there and then; the
tyre line beside it shows what each tyre has. Test runs 80 km/h: out of
balance, then a flat on the front left, then hot discs under braking.

### Impacts

The edge of a hit. Your motion software already moves the rig with a
landing, a big compression or a kerb strike, but rounded off: the sim's
data arrives about 60 times a second and motion cueing smooths and clips
the spikes. What it cannot pass on is the crack of the moment itself, and
it never reaches a belt or a shaker. This tile plays only that.

- A corner's suspension stopping or starting dead from one sim sample to
  the next: the bump stop at the bottom of a big compression, a landing,
  a kerb strike, the suspension topping out over a crest. It starts at
  **from mm/s** (400) of sudden change and is full at **full mm/s**
  (1500). Measured on recorded laps that is about 1 knock a minute for a
  smooth prototype at Bathurst, 6 to 16 for a GT car at Imola, 19 for a
  Group A car at Brands Hatch and 30 to 40 for a 1975 Formula 1 car.
- A sudden jump in the body's vertical g, on every corner: a landing, a
  heavy compression. **heave g** (0.5) is the jump that starts a knock,
  full at three times that; 0 turns it off.
- A sudden jump in the body's g along and across the car, on every
  corner: a wall, a contact with another car. **hit g** (2) is the jump
  that starts a knock, full at three times that; 0 turns it off. Raise it
  if kerbs or gear shifts knock, lower it if light contacts pass
  unnoticed. Needs the SimHub plugin 1.9.

A belt or shaker gets a knock that rings at **ring hz** (40) and dies
over **ring ms** (50). A seat actuator gets one short jolt for its
corner: up when a compression stopped or started (bump stop, landing,
kerb), down when the suspension topped out or dropped away. Keep the
seat routes small: the motion already moves the body, this is only the
edge on top. Route with a part so each post knocks for its own corner.
Test knocks each corner in turn (the rear left topping out), then the
whole car.

Turn **from mm/s** up if rough tracks knock all the time, down if hits
pass unnoticed. A knock is short: on a belt that already carries the
engine, the driveline and the kerbs it is easily lost, so give it a
gain of 1 there, or route it to the seat posts with a part, where it is
a jolt of its own.

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

Some sims report ABS only now and then (Automobilista 2 flags it in a
small part of a stop). Tick **from wheels** and ABS also counts as
working when the wheels show it: a braked wheel dipping towards a lock
and spinning back up as its valve dumps, twice in a row at an ABS
rhythm. Each corner's pulse is then timed to its own wheel's dumps
rather than running at **freq hz**. The tile shows what says ABS is
working right now (the flag, the wheels, or both) and the rate the wheels
are dumping at. It needs wheel speeds or slip ratios; a wheel that stays
locked is a lock-up, not ABS.

Tick **slip link** to tie ABS to the slip tiles. While ABS works, each
corner's valve then shapes that wheel's slip: the lock texture on the
Longitudinal tile stays at the edge of locking (ABS never lets the wheel
lock) and surges with every cycle, by **lock x** (0.7 by default); the
Lateral tile's scrub and slide ease after each dump, when the tyre gets
its grip back sideways, by **scrub x** (0.3). Spin is left alone. The
sim's slip data arrives too slowly to carry the valves' rhythm itself,
so this puts it back. The ABS tile's amp at 0 means no ABS at all, the
slip link and from wheels included: a car without ABS has its amp at 0
in the car table, whatever the tickboxes say from the last car. To feel
ABS only through the tyres, keep its amp and take its routes away.

**off < 5 km/h** (on by default) stops ABS below walking pace, whatever
the sim's flag says: a real ABS lets go before the car stops, and some
sims hold the flag on at a standstill with the brake down.
**TC pulse** is the body surge of each traction-control cut: drive lost,
then back, about **freq hz** times a second, with the same **sharp** and
**spread** (each cut a little different in timing and depth).
### Driveline

What the transmission does when the engine and the wheels disagree.
**clutch x / clutch hz** is clutch judder: with the pedal part-way
through its travel and slip across the clutch, the whole driveline
shudders at its resonance (around 10 Hz). nullCAT works the slip out
from the engine speed against what the gear and road speed say it should
be, using the gear ratios it learns while you drive steadily in each
gear; a launch from rest is full slip, and more throttle makes it harder.
Rolling, the pedal has to stay part-way a quarter of a second first, so a
quick dip during a shift does not judder. **lug x / lug hz** is
the engine bogged: full throttle at too few revs winds the driveline up
and lets it go in a slow shudder that fades as the revs climb out of it.
Neither plays in neutral. Needs the clutch, rpm and gear channels, with
speed and throttle making it exact.

**gearbox** says what kind of box the car has and shapes the Gear shift
tile's thunk: a *synchro* road box clunks softly and a little longer; a
*dog* box (sequential or an H-pattern dogbox) knocks hard and short,
and a shift under power gets a second knock a moment later as the dogs
engage. Sims report neutral for a moment in every shift; with **ignore
neutral** ticked (the default) that is no shift of its own, so each
shift knocks once, as the box leaves its gear (a dog box shifted under
power knocks again as the next gear goes in). Untick it to feel going
into neutral and into the next gear separately. Either way
a gear counts once it has held a few hundredths of a second: a sim's
one-frame junk gear in the middle of a shift is ignored. **whine x** is straight-cut gears meshing: a tone
whose pitch follows the input shaft and steps on every shift, loud under
load and quieter on the overrun; leave it at 0 for a helical road box.
**shunt x / shunt hz** is the driveline's backlash taking up when the
throttle snaps open or shut while rolling in gear, one knock each way,
harder in a dog box; a slow throttle change makes no knock.

**diff x** is the differential (0, off, by default: most road cars have
an open diff, which does nothing you would feel). A **limited slip**
diff's clutch plates resist the driven wheels turning apart; turning at
full lock at walking pace (out of the pits or the paddock, turning round
after an off, a hairpin at a crawl) the outside wheel has to turn much
faster, and the plates stick and let go in an even ratchet of clunks,
harder under power and a little off it. It plays in full below 8 km/h
and is gone by 20. On a racing lap a healthy diff is silent: you feel it
in how the car turns in and drives out, which the motion and the wheel
already give you. Only the corner counts: an inside wheel spinning on
the way out, a wheel skipping over a kerb or a spinning car is no diff. A
**spool** (a locked axle: stock cars, drift cars) lets the wheels turn
only as one, so in a tight turn below 30 km/h the inside tyre scrubs and
hops instead. **diff hz** is how fast the clunks come when the wheels
turn apart briskly (8), fewer as they slow, and **driven** says which
wheels the engine drives (rear by default). It needs the sim to send
wheel speeds. 0.4 to 0.8 suits a race car with a plate diff.

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

**Traction control** can work the same way, shallower: with **TC cut
x** above 0, while the sim says TC is cutting, the engine drops a share
of its firings in irregular bursts (about 15 a second), more of them the
harder you are on the throttle, and catches each time a burst ends. That
stutter is what an ignition-cut TC feels like (rally, older cars); a
modern TC trims the torque smoothly, so TC cut x is 0 by default. The
**TC pulse** tile plays the drive easing with each cut either way (route
it to surge and the belt).

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
