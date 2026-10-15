# Changelog

Notable changes to nullCAT. Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/);
versioning is [Semantic Versioning](https://semver.org/) - while on `0.x`, the
middle number carries breaking changes and the last carries fixes.

## [0.9.8] - unreleased

Development branch. Nothing here is in a release yet.

### Added
- Road tile, suspension model: **learn full mm** (off by default, to try):
  full mm learned for each car from how far its suspension moves while you
  drive, shown on the tile, with your own full mm kept for when it is off.
  On 25 recorded sessions the cars' movement differed by three times, and
  Automobilista 2's cars moved about half as far as Assetto Corsa's, so one
  full mm played their roads at half strength; learned, each plays at a
  similar strength (Assetto Corsa's Porsche GT3 R learns about 29 mm, the GT3
  starting point's, at its cuts). It remembers about five minutes of
  driving; a new car or new cuts start it over.
- **Starting points** for the haptics strip: a **type** row on the Master
  tile with four shipped types of car (**Road car**, **Cup car**, **GT3 /
  GT**, **Open wheeler**), each setting how the tyres let go, how the road
  and kerbs move the car, the wheels and the knocks for that type, in two
  versions: tuned on Assetto Corsa, and **(AMS2)** for Automobilista 2,
  whose tyres reach their grip at larger slip angles (its slides sized up
  1.2 to 1.4 times; on recorded laps a Formula V8's hard scrub
  mid-corner fell from 74 % to 47 % of the time). Applying
  one sets only those settings: your routes, master gain, pos budget and
  the engine, limiter, driveline, ABS and TC stay as they are. Pick the
  type closest to your car, apply it, then pull the car from the car table.
  They ship in `presets.json` beside the program.
- The car table finds a car the sim names with something after a name it
  knows (a version letter or a year, the way Automobilista 2 names a `MINI
  Cooper S 1965 B` beside its `MINI Cooper S 1965`): the entry the name
  starts with, in whole words, from the same game. The car row says `near
  match`.
- A mute on every haptics tile (**M** in its corner): the effect goes
  silent, its settings and readouts stay, no other tile changes. The
  Master tile's **Mute** mutes every tile and **Unmute** brings them all
  back, so Mute then one tile's M plays that effect on its own. Not saved.
- The settings a car's entry can set (follow car, pull, save for this
  car) have a faint tint on the tiles, so it shows what moves with the car
  and what stays.
- Road **model**. **suspension** (the default) replays each corner's
  suspension movement from the sim and adds the road's fine roughness a
  sim does not model: a random road laid out along the track (`rough
  x`), run through the tyre and the corner's spring and damper (`body
  hz`, `hop hz`, `damping`), so it rises with speed, reaches the rears a
  wheelbase after the fronts and is rougher on gravel, grass and cobbles
  where the sim says what is under the tyre (Automobilista 2). **tyre**
  plays the body's movement over the road under each tyre (the sim's own
  road where it sends one, Assetto Corsa) with the same roughness;
  **chassis** plays the sim's own body heave, pitch and roll, learning
  which way they turn from the suspension. A seat actuator gets the
  movement, a belt or shaker its acceleration. The old surface grain is
  still there but off by default: the roughness does its job.
- **Surface** tile: what the ground does under each tyre beyond the
  road's shape. Stones struck on gravel, sand and dirt, busier with
  speed; snow crunching (and a mud squelch); studded tyres buzzing on ice
  and snow; puddles laid out along a wet road, dragging at a tyre as it
  enters, floating it above the aquaplaning speed (the road under it goes
  quiet) and biting as it comes out. The Road tile's roughness now takes
  each surface's character (soft grass undulation, coarse gravel, filled
  snow, glassy ice) and water smooths it. Wetness comes from the sim, or
  is built up from its rain. Protocol 1.6: snow, ice, sand, mud and
  standing water classes, `rain`, `wet`, `wetFL..RR`; the SimHub plugin
  maps Automobilista 2's snow, ice and sand and sends its rain.
- **Wheels** tile: each wheel turning, felt through that corner's tyre,
  spring and damper as a smooth shake at the wheel's own rate rather
  than a knock per revolution. The wheel slightly out of balance, a shake
  growing steadily with speed (**balance x**); flat spots
  ground in by lock-ups, deeper the further a wheel slid, thumping softly
  once a revolution and wearing off (**flat x**, with each tyre's flat
  shown on the tile); hot discs after hard braking pulsing the braking
  force (**judder x**). Each wheel turns at its own speed, so the corners
  drift in and out of step.
- A tyre in the air goes quiet: no road, surface, kerb or slip through
  that corner and nothing from the Wheels tile until it lands, so a jump
  or a crest feels like flying instead of tarmac under you. nullCAT knows
  from the sim (Automobilista 2's tyre flags) or from the tyre carrying
  next to none of its load (the Assetto Corsa family). The Wheels tile
  gains **puncture x**: a punctured tyre (Automobilista 2) rides on its
  folded sidewall, a heavy thump once a revolution. Protocol 1.7: the
  numbered line widens to 96 slots, tokens `onGround*` and `deflated*`.
- Changing car starts over what was learned about the last one: each
  wheel's rolling factor, the usual corner load, the kerb detector,
  the engine's learned redline, and the tyres (no flat spots, cold
  discs). A pause in the pits or a menu no longer gives you fresh tyres;
  without a car name, only ten minutes without the sim does.
- Readouts for the new tiles: Impacts shows each corner's knock now and
  its hardest; Surface shows what each tyre is on (`gravel`, `snow`,
  `tarmac wet`, `float`, `air`); Road shows `air` for a tyre off the
  ground.
- **Impacts** tile: the edge of a hit, which the motion cue rounds off
  and never sends to a belt or shaker. A corner's suspension stopping or
  starting dead between two sim samples (the bump stop, a landing, a kerb
  strike, topping out) or a sudden jump in the body's vertical g knocks:
  a short ring on a belt or shaker, one small jolt on that corner's seat
  actuator.
- ABS and TC feel more like the real thing. ABS works per corner: each
  valve on its own rate, each cycle a little different, so the four
  drift in and out of step and the car grumbles instead of beating
  (**spread**); the dump's edge from round to a knock (**sharp**); the
  cycle slowing towards a stop (**slow x**); the pump and valve buzz
  underneath (**buzz x / hz**). ABS routes take a part, so a post can
  carry its own corner. TC gets **sharp** and **spread** too. The
  defaults are softer and less regular than before; sharp 1 and spread
  0 give the old pulse back.
- ABS **slip link** (a tickbox on the ABS tile): while ABS works, each
  corner's valve shapes that wheel's slip tiles: the lock texture held at
  the edge of locking and surging with every cycle (`lock x`), the
  lateral slip easing after each dump (`scrub x`). Works with ABS's own
  amp at 0. On/off settings on the tiles are tickboxes now (turbo too).
- Slip **grip budget** (on by default, a tickbox on Lateral slip): a
  tyre's grip is one budget shared between cornering and braking or
  driving. Where the sim sends a slip angle and a slip ratio or wheel
  speeds, a wheel slides when the two together reach its limit (trail
  braking, power out of a corner), on whichever tile matches its
  direction; pure cornering or braking is unchanged.
- Lateral slip **screech** and **pulse**, on the front tyres. Screech:
  the tyres starting to squeal before they let go, a fine texture on its
  own carrier coming in from `screech %` of the peak (`screech x`,
  `screech hz`). Pulse: past the limit, the slow stick-slip shove of a
  scrubbing tyre, a few times a second and never quite regular, as strong
  as the slide itself (`pulse x`, `pulse hz`). Both on by default (0.3 and
  0.6); 0 turns either off.
- ABS plays as hard as you brake: light trail braking is a light pulse,
  full by **full at brake %** (80 by default; 10 = always full, as
  before). ABS never plays below 10 % brake, whatever the sim's flag says.
- Road **susp %** and **susp cut hz** (suspension and tyre models): how
  much of the sim's own movement plays beside the roughness, and a steep
  cut on that movement alone. Set the cut above the car's body bounce
  (3 to 4 Hz for a road car, 5 to 7 for a GT3, 7 to 9 for a single-seater)
  and the body's slow dive, roll and crests, which the motion already
  gives you, drop away while the track's bumps and the roughness stay. Off
  by default (100 %, 0 Hz: as before).
- The Wheels tile shows the tyres' state at a glance: flat spots (which
  wheel, how deep), brake discs hot enough to judder and a puncture, in
  amber or red while there is anything, `tyres OK` otherwise.
- Hover help on every haptic setting: what it does, a sensible range, and
  starting points for a road car, a GT3 car and a single-seater. A dimmed
  setting says why first.
- Lateral slip: how a slide comes in is now yours to set. **onset %**
  (where it starts, as a share of peak deg; 40, was a fixed 60), **ease**
  (0 straight up, 1 a light scrub first), **smooth hz** (steadies the
  slip angle the sim sends, which flickered around the onset and played
  in lumps) and **attack ms** (how fast a slide builds; 30, was 8).
- **Kerb** is a rumble strip: ribs every `pitch cm` humming at speed
  over the pitch, a thud on and off on a seat actuator, the ribs and a
  kick on and off on a belt or shaker, per corner (routes take a part
  now). Which tyre is on a kerb comes from the surface under it
  (Automobilista 2), the game's kerb channel (Competizione, EVO), or a
  tyre stepping up past its axle partner in the road heights (Assetto
  Corsa, `detect mm`, off by default). Kerb's `freq hz` is gone: the ribs'
  pitch sets the hum. Test runs it at 80 km/h, on and off.
- Assetto Corsa's slip angle per wheel, worked out by the SimHub plugin
  from each tyre's heading against where its contact patch moved: the
  Lateral slip tile gets a true sideways slide instead of the combined
  slip. Protocol 1.5 widens the channel wire to 64 slots and adds the
  road height and surface under each tyre, the body's heave, pitch and
  roll, and the wheelbase and track.
- Nothing to bind for the sims the SimHub plugin knows: for Assetto
  Corsa, Competizione, EVO, Rally and Automobilista 2 it sends the
  per-wheel channels (wheel speeds, loads, the sim's combined slip, a
  slip ratio where the game has one, suspension travel or velocity) by
  itself, so Longitudinal slip, Lateral slip and Road all have their
  input out of the box. Every channel now travels by name, so the Sim
  channels map in Setup no longer touches a SimHub setup (it is for
  senders on the numbered line). Protocol 1.4 adds `wheelSlip*` (the
  lateral source where a sim has no slip angle) and `suspTravel*` (the
  road from a sim that gives position rather than velocity).
- Road **surface**: the grain of the tarmac under a rolling car, a
  texture on its own carrier (`surface hz`, 12 by default) that rises
  with road speed on top of the replayed bumps, at its own mix (`surface
  x`, 0.2 by default) with `surface km/h` setting where it is full. A
  moving car is no longer silent on a smooth road.
- The car table: an entry for every car in Assetto Corsa (Kunos content
  and DLC) and Automobilista 2 carrying what belongs to the car (the
  engine's description and character, the limiter, the driveline, ABS and
  TC where the real car has them, with those five tiles' amplitudes) and
  notes on the real-world figures behind the values. Never routes, master
  gain or any other tile: applying an entry moves the five car tiles and
  leaves your rig tuning alone. **Follow car** on the Master tile (on by
  default) applies the entry whenever the sim names a car; **pull car
  preset** does it on request; a **preset** search box applies any entry
  as a starting point for a car the table lacks; **save for this car**
  files your tuning under the current car in `cars.local.json`, which
  wins over the shipped `cars.json` and survives updates. The Engine tile
  names the matched car and folds its notes under it. A profile bound to
  the car still wins. Existing users: with follow car on, the Engine,
  Limiter, Driveline, ABS and TC tiles move on the first car change after
  this update; untick it on the Master tile to keep them by hand.
- Tyre slip per wheel: the Skid and Lockup tiles become **Lateral slip**
  (scrub at the fronts, slide at the rears) and **Longitudinal slip**
  (lock under braking, spin under power), each modelled for all four
  wheels from the sim's raw slip angles, slip ratios or wheel speeds,
  weighted by tyre load when the sim sends it. Severity starts where the
  tyre actually lets go (peak deg / peak ratio), the onset is abrupt and
  the carrier slows and roughens as the slide grows. Every route on
  these tiles has a **part**: a corner, an axle or all, so a four-post
  rig feels the inside front lock up on that corner. The old single skid
  and lockup channels still work and feed all four wheels. Saved 0.9.6
  tuning carries over.
- Road replayed per corner: with suspension velocities from the sim the
  Road tile plays each corner's real bumps (the slow body motion the cue
  already does is cut away) instead of a synthetic texture, routed with
  the same part selector so a four-post rig feels a kerb under the wheel
  that hit it. `full mm` and `cut hz` tune it; the texture remains the
  fallback for sims without per-corner data.
- Engine, the lift and the ends of the rev range: a load-independent
  **inertia** shake (rpm squared) that stays on when you lift and fades
  as the revs fall; a **lift-off** event when the throttle snaps shut up
  the band, a pop on a naturally aspirated engine or, with **turbo** on,
  the blow-off whoosh and compressor flutter scaled by the sim's boost;
  **pops** for fuel-cut crackle on the overrun; overrun in gear is
  heavier and rougher than a neutral coast-down; cranking on the starter,
  the catch, dying and the stall kick are modelled below idle; the pit
  limiter cuts like the rev limiter without teaching the redline. New
  tokens `boost` and `pitLimiter`; the plugin sends both automatically.
- Driveline tile grows a gearbox: `gearbox` (synchro or dog) shapes the
  gear-shift thunk (a soft clunk, or a hard short knock with a second
  knock as the dogs engage on a shift under power), `whine x` is
  straight-cut gears meshing (pitch following the input shaft and
  stepping on every shift, loud under load), and `shunt x / hz` is the
  backlash taking up when the throttle snaps open or shut while rolling
  in gear.
- Shakers: bass shakers and other tactile transducers on one or more USB
  sound cards are routing destinations like the axes. Each effect's
  route editor lists Shaker 1..N with a gain and a harmonic (x2 puts a
  9 Hz belt effect at 18 Hz on the shaker, phase-locked). The haptics
  layer keeps stepping and pumping the sim channels while the control
  loop is stopped, so a shaker install with no drives is a complete
  haptics engine for SimHub and the other senders. Per card: its own
  drift servo (the EtherCAT clock and a USB crystal never agree), 300 Hz
  smoothing, DC block and soft clip; an underrun holds and fades, never
  clicks. Host settings: enable, device name(s), channels, a status
  line with a test tone per channel. `axis delay ms` on the Master tile
  holds the axis effects back so belts and shakers land together. The
  null backend (`"audioDevice": "null"`) runs the whole path with no
  card, for a bench or CI. Audio backend: miniaudio (public domain /
  MIT-0, vendored).
- Haptics profiles: named copies of the whole strip (settings, routes,
  the engine, master, pos budget) in `profiles.json`, with save as, load,
  delete and a "use for this car" binding; when the sim names a bound
  car (or game) nullCAT loads that profile itself. Endpoints under
  `/api/haptics/profiles`.
- Engine layouts two-stroke (every cylinder fires every rev, harder for
  its size, no idle lope: karts and old bikes) and electric (no firing,
  the motor whine from the first turn rising with load, regen whine on a
  lift, a soft limiter, no cranking or stall).
- Driveline tile: clutch judder (a slipping clutch grabbing and releasing
  at a launch or a bad downshift, the slip worked out from engine speed
  against the learned gear ratio and road speed) and lugging wind-up
  (full throttle at too few revs), each with a mix and a carrier; nothing
  in neutral.
- Sim dots: every haptics tile shows whether the current game has ever
  sent that effect's channels (grey no, amber yes but it never played,
  green it has played), remembered per game in `effectstatus.json` next
  to the rig config so a sim's coverage is known without a lap. The
  Master tile names the game, can show another remembered one, and can
  forget it (`GET /api/haptics/status`, `POST /api/haptics/status/clear`).
- Stuck-effect guards: a channel stream whose values stop changing for
  two seconds while packets keep arriving (a paused sim, a sender
  repeating its last frame) now releases every effect, the Devices
  section shows "frozen", and the log records each start, stop, freeze
  and resume of the stream. The SimHub plugin stays silent while the
  game is paused. (An rpm vibration once stayed on after the sim had
  closed because the sender kept repeating its last frame.)
- Protocol 1.3: the channel wire takes 48 slots, with per-wheel tokens
  for slip angle, slip ratio, wheel speed, load and suspension velocity
  plus maxRpm; and a new named line, `NULLCATY,key=value,...`, so any
  motion software with a text output (FlyPT, SimTools, a custom feeder)
  can send channels by name with no slot order and tell nullCAT which
  game and car are running. SimHub remains one sender among others.
- SimHub plugin 1.3: sends the car's max rpm and the game and car names,
  and takes optional per-wheel bindings (slip angle, slip ratio, wheel
  speed, load, suspension velocity, each with a unit scale) that go out
  by name only when bound. Older plugins keep working unchanged.
- Haptics on position axes: any effect can now be routed to a CSP axis
  (surge, the verticals) as well as a belt or device. The route's gain
  is in mm at full amplitude, each axis has a Haptic max ceiling (3 mm
  by default, 0 = no haptics on that axis), and a pos budget on the
  Master tile sets the share of the axis's velocity and acceleration
  limits haptics may use. Effects are derated to what the axis can
  follow at their frequency, so a slow engine rock comes through on a
  surge axis while a 90 Hz buzz stays small; the route editor shows
  "up to X mm" per axis before anything is played. The offset rides on
  the motion cue, which keeps priority, and the sum stays inside the
  axis limits. PP-mode axes take no haptics.
- Tile readouts: every effect tile shows what it is being fed. The
  channel chip carries each bound channel's live value and its session
  peak (`skid 23/41`), the two slip tiles and Road show a per-wheel
  now/peak line (Test is 1.00 on every wheel, so a lap reads against it
  directly), the other tiles an `in now/peak`, and the wave is tagged
  `test` or `live x` so a moving wave with nothing felt reads as small
  live input rather than a stuck effect. The peaks hold until **Reset
  peaks** in the header, the same button that re-baselines the drive
  cards, so a slide can be read after the sim gives the screen back.
- `peak %` on the magnitude-fed paths (the slip tiles' single-channel
  fallback, Kerb, Road's texture): the channel value that counts as full
  severity, so a property that only reaches 25 in a burnout still gives
  a full slide. The tile dims whichever trim does not act on the input
  path that is live (per-wheel data or the single channel).
- `ncxFrozenMs` in host.json: the frozen-stream guard is now off by
  default and opt-in with a window, because a car idling at a standstill
  sends identical packets too and the guard muted the engine after two
  seconds in the pits. The SimHub plugin goes quiet on pause by itself.
- Lateral slip in Automobilista 2 plays a real slip angle. The sim gives
  only how fast each tyre slides, and as a plain slip that grew with road
  speed, so the scrub was strongest on the straights. nullCAT now takes
  out the part the wheel's lock or spin explains, turns the rest into an
  angle against the road speed, and learns on the straights the small
  angle every car shows there (toe, the tyre's own creep) to take it off.
  The tile shows each tyre's angle and the learned straight-line angle,
  and whether it is still learning. **baseline** chooses learn, learn
  once (per car, then held) or fixed (**base deg F** and **R**), and
  **relearn** starts it over. Protocol 1.8: `slipSpeedFL..RR`; the SimHub
  plugin sends it from Automobilista 2.
- Longitudinal slip **edge**: a fine grain while a tyre brakes or drives
  hard but short of letting go, so the rear putting the power down (or
  the fronts close to locking) is felt before anything slips. From **edge
  from %** slip (3 by default) to full at 15 %, with its own mix (0.3)
  and carrier (35 Hz).
- ABS **from wheels** (off by default): ABS also counts as working when a
  braked wheel dips towards a lock and spins back up at an ABS rhythm, for
  sims that flag ABS only now and then (Automobilista 2 flagged it in 7 %
  of the braking). Each corner's pulse is then pulled into step with its
  own wheel's dumps. A wheel hovering near a lock, staying locked or in
  the air does not count. The tile shows what says ABS is working (the
  flag, the wheels) and the rate.
- **record** on the Master tile: every telemetry packet to a file on the
  controller (`~/nullcat-recordings` on the Pi), so a session can be
  played back exactly for tuning. A thread of its own writes it; the
  control loop only hands packets over and never waits on it.
- **fresh tyres** on the Wheels tile: wipes the flat spots and cools the
  discs there and then (a restart from the grid in the same car keeps
  them otherwise).
- Driveline tile **diff** (off by default): a limited-slip diff's plates
  clacking as the driven wheels turn apart in slow tight corners, harder
  under power, or a spool's inside tyre scrubbing and hopping; irregular,
  full below 30 km/h and gone by 80 (a spool's by 50). With **diff hz**
  and **driven** (rear, front, all).
- Wheels tile **bite** and **drag**: the pads on the turning disc. The
  bite is the grab as the pedal goes on or is squeezed harder, swelling
  and fading over a quarter second; the drag the rub that stays while you
  brake. Each a dithered grain with its own pitch (**bite hz**, **drag
  hz**, at 100 km/h) following each wheel's speed, stronger at speed and
  on the loaded tyres, nothing from a locked wheel; at the caliper, not
  through the suspension. On by default (bite 0.5, drag 0.3), a saved rig
  included: set them to 0 on the tile if you would rather not.
- Impacts defaults measured on recorded laps: **from mm/s** 400 (was 500),
  **full mm/s** 1500 (was 2500), **heave g** 0.5 (was 1.5, set while the
  plugin sent heave ten times too big: in real units the body almost
  never knocked). About 1 to 40 knocks a minute by car and track. A saved
  rig keeps its own values: change them on the tile.

### Fixed
- A `rig.json` cut short by a crash mid-write (or a hand edit gone wrong)
  read as no config at all, and the next start wrote the defaults over it,
  tuning and all. Such a file now stops nullCAT from starting, with the
  file and the parse error in the log and nothing written over it; every
  config save goes to a temporary file first and is renamed over the old
  one, so a crash mid-save leaves the old file whole; and the one rewrite a
  load makes by itself (the 0.9.6 slip tile keys) keeps the file as it was
  in `rig.json.bak`.
- A hand-edited `rig.json` or `host.json` was never checked on load (only
  a save from the web was), so an amplitude of 500 % ran at 500 %. The
  load now lists every value out of range in the log, with its file and
  field, and clamps the haptics values that would hurt (an effect's amp, a
  route's gain and harmonic, the master gain, the position budget, an
  axis's haptic max) to the range the strip offers; the rest runs as
  written. The file is left as it is.
- A mute request whose body did not say on or off un-muted the rig (it
  looked for the word true and found none). Such a body is now refused
  and changes nothing.
- A number that was not a number (a sender's nan or inf) or a control
  character in a car name made the whole status unreadable to the page,
  which then showed the link as lost. Such a number now reads as 0 and
  every control character is escaped.
- A page with unsaved edits on the Haptics strip could save them over
  what follow car, a pull, a starting point or a profile had applied in
  the meantime (the page only reloads when it has no edits). Every save
  now says which copy of the settings it edited; a stale one is refused,
  nothing is written, and the page reloads the controller's settings and
  asks you to make the edits again. A save of yours that moves a setting
  the car table owns also clears the car row's "applied".
- A car that got its entry from the car table had it applied again at
  every restart, over whatever you had tuned for it by hand (only a car
  on the defaults was remembered). nullCAT now remembers the car the
  table last acted on, and whether it got its entry or the defaults, in
  `carstate.json` beside the rig config: the same car after a restart
  keeps its tiles as they are and its car row reads as before.
- The shaker status line said only "not open" when no card could be
  opened; it now names each card and its own error.
- A `profiles.json` that could not be read (cut short, a hand edit gone
  wrong) read as no profiles at all, and the next profile save wrote that
  emptiness over it. It is now left alone with the reason in the log; a
  profile saved meanwhile goes to `profiles.json.rejected` and the strip
  says so, until the file is fixed or moved and nullCAT restarted.
- A car table entry written by hand with a setting the car does not own
  (an ABS edge, the whine's pitch, the diff's strength) moved that
  setting on the rig when the car came on. Only the settings a car owns
  cross over now; the shipped table is checked to carry nothing else, to
  name every car once, and to validate on a fresh rig.
- A setting inside the haptics part of `rig.json` that this build does
  not know (a newer version's, or a note of your own) was dropped by
  every apply the controller made itself (a car entry, a starting point,
  a profile). They survive now, like the unknown keys elsewhere in the
  file.
- With follow car on, a car the car table had no entry for kept the last
  car's engine, limiter, driveline, ABS and TC (a Mini played a truck's
  12.9 litre engine). In a game the table covers (Assetto Corsa,
  Automobilista 2, or one you have saved a car for) those tiles now go
  back to their defaults for such a car (a generic engine, gearbox, ABS
  and TC, at the strength the cars that have them play them; the same
  car after a restart keeps what you tuned), the car row says `no entry ·
  defaults in use`, and the settings the car table owns are tinted pink
  until you pull a preset car or save for this car. Other games are left
  as they are.
- Automobilista 2's `MINI Cooper S 1965 B` and `Porsche 911 GT3 Cup 4.0`
  had no entry in the car table; both have one now, and the Minis drive
  their front wheels.
- Automobilista 2's slip read wrong. Every tyre rolling down a straight
  shows a small slip along the road (3 to 6 % of the road speed on the
  fronts), and nullCAT took it off as a sideways angle: under braking the
  fronts read no angle at all, corners read 2 to 3.5 degrees light, and a
  driven wheel under power read a false angle (the Mini's fronts scrubbed
  on straights). That slip is now learned per tyre and taken off along the
  road. A launch's wheelspin no longer reads as a 90 degree slide, and the
  slide fades out below 25 km/h, where the sim's numbers say little.
- The rolling factor (wheel speeds) learned wrong in several ways: the
  first value in a curve (a front wheel 4.6 % off), a slide or a reset
  moving it (3 % off for a minute), the driven wheels learning high from
  engine braking (the 250 GTO's rears 1.2 %), a slipping clutch creeping it
  up on each downshift. The first value now needs both axles turning
  together, a moment far off a learned value is ignored unless it keeps
  disagreeing, the clutch slipping teaches nothing, and engine braking
  teaches the driven wheels nothing.
- With the grip budget on and onset at 0, any slip angle took the whole
  budget, so lock and spin went nearly silent. The budget now never plays
  less than each tile alone.
- Assetto Corsa played tyre slip about 1.4 times stronger than
  Automobilista 2 for the same slide: sending tyre loads weighted the
  loaded tyre up. The loaded tyre of each axle now counts as 1 and the
  lighter one less, so both sims play alike. (If slip feels lighter in
  Assetto Corsa, raise the slip tiles' amplitude.)
- The learned redline started at a 7,000 rpm guess and took minutes to
  find a car's real one (83 s for the Corvette GTP; a truck read its 3,700
  rpm as half revs until it hit its limiter). It now starts at the max rpm
  the sim sends.
- With the control loop stopped (shakers only), the haptics **Mute** and
  the tiles' **Test** did nothing until the loop started; and Test refused
  a tile routed only to shakers. Both now work on the shakers either way.
- Shakers on one sound card drifted apart: after a few minutes channels 2
  and up played about a tenth of a second behind channel 1. Every channel
  now travels together and they stay in step.
- The levels on the shaker status line showed one instant of the
  waveform, so a test tone could read anywhere from 0 to its full level
  (a channel looked half as loud when it was not). They now show each
  channel's level.
- Longitudinal slip from wheel speeds was dead in Automobilista 2: the sim
  counts its wheels turning backwards going forward, and nullCAT never
  learned their size. The sign is ignored now (a car rolling backwards
  already read as rolling).
- Road's chassis model and the Impacts tile's heave read the body's
  vertical acceleration about ten times too big in Automobilista 2 and
  Assetto Corsa, which made the chassis model violent: the SimHub plugin
  multiplied SimHub's heave, already in m/s², by g. Update the plugin.
- Hard judder from the Wheels and slip tiles ("square wheels"), seen in
  Automobilista 2. A jump in the sim's road speed no car can make (a
  return to the pits, a reset) read as all four wheels locked at speed and
  ground full flat spots into every tyre in under a second; the speed is
  now ignored for half a second after such a jump, and a flat needs a
  lock-up of at least a tenth of a second. Pausing the sim in the middle
  of a lock-up kept grinding the flats (and heating the discs) for the
  whole pause; it no longer does. Automobilista 2's tyre slip now and then
  reads in the billions, which held the Lateral slip tile at a full slide
  for seconds through its smoothing; slip values are capped before they
  are smoothed. A body acceleration past 30 g is ignored too, and so is a
  gear outside -1 to 12 (a sim sent 150 once: two gear-shift knocks), and
  an engine speed past 40,000 rpm or changing faster than any engine can (Automobilista
  2 sent 191,105 rpm once): the last true one stands.
- The learned gear ratios went wrong and were remembered that way (in
  Automobilista 2 and Assetto Corsa: a 2nd gear learned at half its
  ratio, a 1st at three times it). They are learned only while a gear is
  held steadily, off the brake, with no launch or wheelspin, over seconds
  of driving; a higher gear must have the lower ratio; and remembered
  ratios out of gear order are forgotten. The clutch judder reads its
  slip from them, so it juddered when it should not.
- Clutch judder shuddered on shifts at speed, wherever the clutch dipped
  for a moment (an auto-clutch, a heel-and-toe). Rolling, the clutch now
  has to stay in its slipping range a quarter of a second first; a launch
  judders at once as before.
- Longitudinal slip from wheel speeds could stay silent for minutes in a
  race car (Assetto Corsa): the wheels' rolling size was only learned
  after half a second without a break at light throttle. Short moments
  now add up.
- With **from wheels** on, ABS played on lock-ups in a car with its ABS
  switched off (Assetto Corsa). From wheels is now a choice: **with the
  flag** (what "on" was) counts the wheels only once the sim's own ABS
  flag has been on for that car (the ABS tile says when it is waiting for
  it); **on its own** is the wheels alone, for a game whose flag is always
  0.
- The TC pulse ran on after a lift (Automobilista 2's flag stays on):
  nothing with the throttle shut now.
- An Impacts knock with the car standing, when a reset or a spawn drops
  it onto the ground.
- The Gear shift tile knocked two or three times for one shift: sims
  report neutral for a moment in every shift, and Automobilista 2 a junk
  gear for one frame (a 5-4 downshift read 5, N, 3, 4). A gear now counts
  once it has held 45 ms, and with the Driveline tile's new **ignore
  neutral** (on by default) the neutral passed through is no shift: one
  knock per shift, as the box leaves its gear (knocking when the next gear
  went in came about 200 ms late: Automobilista 2 shows neutral for 130 to
  190 ms in every shift). A dog box shifted under power adds its
  engagement knock as the next gear goes in.
- The Driveline tile's gear whine beat against the Engine tile's buzz when
  both played on one axis: a slow pulsing at speed in the top gears, where
  the two sat a few Hz apart. The new **whine pitch** puts the whine below
  the buzz (the default), an octave down or above it; "with the buzz"
  keeps the old placement. On a control loop slower than about 1.5 kHz
  (a PC) "above" plays an octave down: the loop cannot carry it.
- ABS played through its slip link with the ABS tile's amp at 0, so a car
  without ABS (its car-table amp 0) still judder-locked the tyres through
  half of every stop when slip link was ticked from the last car. Amp 0 is
  now no ABS at all; to feel ABS only through the tyres, keep its amp and
  give it no routes. ABS also played at a standstill with the brake down
  where the sim held its flag on: the ABS tile's new **off < 5 km/h** (on
  by default) stops it below walking pace.
- Longitudinal slip from wheel speeds could read a wheel as spinning or
  locked for many seconds after a spin: with the pedals off in a spin,
  nullCAT learned each wheel's size from the stopped wheels. It now learns
  only while each axle's two wheels turn together and no tyre is sliding,
  over seconds, and no single moment can pull it far off. A car rolling
  backwards after a spin read as four locked wheels (grinding flat spots);
  it reads as rolling now.
- Kerbs from the road heights (`detect mm`) never played in Assetto
  Corsa with the kerb slot bound in the Sim channels map: the plugin sends
  that slot as 0, and the 0 shut the detection out (all you felt was the
  Road tile's suspension going over the kerb). The detection now always
  runs when `detect mm` is set; where the game sends its own kerb channel
  too (Competizione, EVO), the stronger of the two plays. And it finds the
  kerb's edge and ribs: a tyre jumping by half of `detect mm` or more against its axle
  partner within a few hundredths of a second. Measured against a slowly
  learned camber, the track's own camber changes read as a kerb for most
  of a lap at Imola (on recorded laps: 70 % of the driving before, 4 to 7 %
  now, like Automobilista 2's own kerb surfaces).
- Naturally aspirated engines never gave their lift-off pop: the SimHub
  plugin sends a boost of 0 for them, and the pop was scaled by the boost.
  Boost now counts only for an engine set as turbo, so a sim that reports
  a naturally aspirated car's manifold pressure (about 1 bar) no longer
  makes its firings heavier either.
- ABS in Assetto Corsa came through chopped: the game raises its ABS
  flag in blips, about every other sample through a stop, and the ABS
  tile started and stopped its valve cycle with every blip. The flag is
  now held over short gaps (**hold ms** on the ABS tile, 150 by default,
  0 = off), so a stop plays one steady pulse and lets go just after the
  last blip. Brake pressure still sets how hard it pulses.
- Traction control stuttered in a way that felt like a fault in the rig:
  the Engine tile dropped bursts of firings whenever the sim's TC flag
  was on, and that flag flickers as TC cuts in and out. The engine's
  stutter is now **TC cut x** on the Engine tile, 0 by default (a modern
  TC trims torque smoothly; raise it for a rally or older car with an
  ignition-cut TC, saved per car), and the TC tile gets **hold ms** like
  ABS, so TC working plays as one effect.
- Automobilista 2's marbles (the loose rubber off the racing line) read
  as dirt, so running slightly off line struck stones. They read as bumpy
  tarmac now (update the SimHub plugin).
- A Test press set the tiles' session peaks to full, hiding what the sim
  had done. Peaks now skip a Test and the moment it takes to fade.
- Shaker routes: the **x** harmonic did nothing on ABS, TC and the
  Lateral slip pulse (a 12 Hz ABS cycle stayed at 12 Hz); it now plays
  them at that multiple, locked to the cycle. Road, Kerb, Surface, Wheels
  and Impacts have no single carrier to multiply, so their shaker routes
  no longer offer it (they show "as is"). A shaker route on a per-corner
  tile now takes a part like an axis route: a corner per shaker, an axle,
  or all.
- The Road tile's tyre and chassis models sat at their limits most of the
  time ("a ball between two paddles"): their full scale, **body mm**, was
  1 mm while the car's body moves several mm over a road. It is 12 by
  default now (up to 50); set it on a saved rig, which keeps the old 1.
  Chassis: the body's slow roll into a corner and dive under braking now
  go through a steep cut (about half what came through before), and the
  body's angles jumping more than a car can turn between two samples
  (a reset) are ignored for a moment like a speed jump. Tyre: the sim's
  samples are joined by straight lines instead of steps, and the sim's
  own road is taken only as finely as its rate carries (a quarter of it:
  15 Hz from a 60 Hz feed), the roughness filling in the rest, so a
  faster telemetry feed brings more of the real road by itself.
- With the grip budget on, a slide straight along (wheelspin, a lock-up)
  took the Lateral tile's `ease` curve as well: at ease 2 a 50% wheelspin
  played at a few %. The ease shapes the sideways part only now.
- Shakers lagged the motion by about 80 ms: the hand-off between the
  control loop and the sound card kept 64 ms of samples waiting. It now
  keeps about 12 ms, so the shakers land about 28 ms after the motion. The
  shaker status line and the log show the whole delay, not only the
  card's buffer.
- The Surface tile showed `tarmac` under every tyre in games that send no
  surface (Assetto Corsa), as if it were reading the track. It shows `-`
  now, and its hover says the tile has nothing to play in that game.
- Pi: running the test suite on a rig with the control loop live could
  stall the loop and fault the drives, because the tests ran at the
  loop's priority on its processor core. They never do now, and the
  installer refuses to run while the loop is live (Deinitialize first).
- Shakers on the Pi were near silent, the back channels quieter still, and
  late. nullCAT opened the sound card through the system's shared stereo
  layer (it lists the card under several names), squeezing the eight
  shaker channels into two with a buffer of 60 ms and more. It now opens
  the card directly: every channel its own, about 15 ms of buffer (if
  another program holds the card it falls back and says so; the log names
  the device it opened). It also sets that card's output to full line
  level, since many USB boxes start at about -24 dB; set the shaker volume
  on the amps and the route gains.
- Shaker routes vanished: every reload of the page (a car preset applied,
  a profile loaded) dropped them from its copy of the settings, and the
  next Save wrote the haptics back without them. A route drawer left open
  across a reload also kept editing the old copy, so edits never reached
  Save. Both fixed.
- Pi: re-running the installer from a clone after a version change kept
  the previous version's stamp in an existing build folder, so the new
  build installed under the old version's name. A version change now
  re-runs the build's configuration by itself.
- Haptics on the seat actuators with motion off: unparking with no
  motion data from the sim but its channels still arriving made the
  verticals run wild (the haptic movement built up on itself every
  cycle), and the next unpark after a park could fault the drives with
  an excessive position step. The hold no longer includes the haptics,
  the haptics guard starts fresh every time an axis goes live, the
  command can never move faster than the axis's maximum velocity or
  reverse faster than its acceleration allows, and a last check before
  every command holds it to that and logs it if anything upstream ever
  lets a bigger step through.
- The Driveline tile's wave stayed "live" after its first shunt knock
  (Test, or any fast throttle crossing): a finished knock kept counting
  towards the tile's level for ever. The output itself was right; the
  wave and the `in now/pk` readout were not.
- The per-wheel readout line ran off the slip and Road tiles; it is a
  2 x 2 grid now.
- A transient (gear shift, detent click) routed to a shaker never played:
  the shaker routes were dropped when the event fired.

### Changed
- The Driveline tile's diff is now a walking-pace thing, as a real one
  is: a limited slip's clunks come only when the driven wheels are made to
  turn apart in a tight turn at a crawl (full below 8 km/h, gone by 20; a
  spool's scrub and hop by 30), from the corner's own geometry (an inside
  wheel spinning, a wheel skipping over a kerb or a spinning car no longer
  counts), evenly like a ratchet, faster the faster the wheels turn apart
  (diff hz, now 8 by default). It played in every slow racing corner up to
  80 km/h, about 17 random clacks a minute, which felt like a broken car.
  It now needs the sim to send wheel speeds.
- A smooth engine's idle rock is stronger: a V12 (a 250 GTO) now idles
  with a felt shake, over half a small four's, where it had a third.
  Rougher engines are as they were, and the order of engines stays the
  same.
- The Sim channels map in Setup is labelled for what it is: the slot map
  for senders on the numbered line. The SimHub plugin names everything
  it sends and is not affected by it.
- Tyre slip, the two wheels of an axle move together: one carrier phase
  and roughness per axle, each corner at its own level. With a carrier
  per wheel the two front posts of a rig drifted apart and rocked it side
  to side.
- Combined slip (Assetto Corsa, Automobilista 2) plays on the Lateral
  tile for any sliding tyre, spinning and locking included, from a third
  of `peak %` up to full. The plugin scales Assetto Corsa's value so a
  clear slide (about 6 on the game's own scale) reads 100, `peak %` goes
  to 400, and the tile's chip shows the value arriving and its peak (slip
  peaks only count while the car is moving).
- Defaults chosen for what a seat actuator can carry (it gives little
  above about 20 Hz): scrub 20 Hz, spin 16 Hz (tread skipping rather
  than a live axle's 10 Hz tramp), road texture and kerb 16 Hz. Saved
  settings are not changed.
- Road replay: `full mm` defaults to 25, bigger bumps bend over a soft
  knee instead of clipping, and the replay is lightly smoothed, so a kerb
  is a thump rather than a pop.
- Gear shift on a position axis: one jolt at about 12 Hz (longer for a
  synchro, shorter for a dog box) instead of a 60 Hz burst the actuator
  could not move. Belts and shakers keep the burst.
- ABS is a fast pressure dump and a slower rebuild each cycle rather than
  a sine; the TC pulse is a sharp loss of drive and a slower recovery,
  and while TC cuts the Engine tile itself stutters (a share of the
  firings dropped in irregular bursts, deeper with throttle).
- The Master tile is in sections: rig, profiles (the whole strip), car
  table (the five car tiles) and sim dots (display only).
- Route gains are validated per destination: a position axis takes mm
  up to its own Haptic max, a torque axis or shaker a 0 to 2 multiplier.
  The validator used to stop every route at 2 while the route editor
  offered the axis cap, so a 2.5 mm route on a 3 mm axis was refused on
  Save.
- Slip at a standstill: the lateral fallback is speed-gated like the
  longitudinal one (nothing below 5 km/h), so a held skid value from a
  stationary car no longer plays.
- Haptics strip: tiles pack into columns top to bottom, so the tall
  Engine tile sits beside a stack of short ones instead of stretching
  every tile in its row to its own height.
- Pi updater: a new version is assembled beside the live one, the config
  is copied into it, and only then does it take its place and the symlink
  flip; the live folder is never deleted or emptied, and applying a
  version over the folder that is already running it is refused with a
  reason instead. A script test pins the normal update, that refusal and
  the rollback (the remembered sim dots file now travels with the config
  too).
- Engine buzz on slow loops: the automatic buzz order aims the redline at
  120 Hz on the Pi's 2 kHz loop but at an eighth of the loop rate on
  slower ones (62 Hz on a 500 Hz PC loop), so the carrier always has at
  least 8 samples per cycle instead of a coarse 4. Set `buzz order` by
  hand to override, as before.
- The sim channel token list is served to the browser with the haptics
  schema (the Sim channels editor and the tile channel ticks read it from
  there; a hand copy in the page had been dropped during the refactor
  below, which left that editor broken on the development branch only).
- Haptics internals: the effect list is one table the controller serves
  to the browser (GET /api/haptics/schema), so adding an effect is one
  row plus one law instead of edits in eight places; the engine model and
  the channel laws live in their own files; the Test endpoint parses its
  request instead of matching text. Settings files are unchanged. New
  tests pin that effects only reach a live axis and stay inside its
  limits, that saves apply live, that a stale stream fades everything,
  mute and e-stop, the channel laws, the config round-trip, and the
  web strip rendering from the schema.

## [0.9.7] - 2026-10-11

A safety fix for 0.9.6. Nothing else changes; update whenever the rig is
parked.

### Fixed
- Position axes (seat actuators, surge): when the motion data spiked and
  snapped back from one frame to the next (a glitch in the stream, a crash
  spike that got past the sender's filter, a reset in the sim), the
  command could swing past the axis's maximum velocity the other way
  within a single control cycle: a hard jolt, or an excessive position
  increment fault (Er87) on the drive. Stopping is still immediate, but
  reversing now respects the acceleration limit, and nothing exceeds the
  maximum velocity.
- A last line of defence: while the sim drives a position axis, its
  command can never move further in one cycle than the axis's maximum
  velocity allows. If anything upstream ever lets more through, it is held
  to that limit and logged once.

## [0.9.6] - 2026-10-07

0.9.5 was a pre-release that never became an official release. If you
are updating from 0.9.4, everything in the [0.9.5] section below (force
devices, the sim channel stream, revmatch, the self-updating Pi, and
its fixes) is new to you as well.

### Added
- Haptics: a routable effect layer that rides on top of the normal
  feel on any torque axis (devices and belts), always clamped inside
  each axis's own limits. Ten effects, all off by default: detent
  click, gear-shift thunk, engine, ABS pulse, brake lockup, tyre skid,
  road surface, rev-limiter buzz, traction-control pulse and kerb
  rumble. A Haptics strip in the web UI gives each effect a tile with
  its settings, the sim channels it needs (with a live delivered or
  missing mark), a waveform showing what it is actually putting out,
  routing to any torque axis, and a Test button that says why when
  nothing can be felt. Tuning applies live on Save, with no
  re-initialize; a master gain and a Mute button sit on the strip.
- Engine model: at idle you feel the block rocking at crank rate with
  each firing as a low thump on top; above idle a buzz takes over,
  rising in pitch with rpm and building to the limiter. Describe the
  engine (cylinders or rotors, litres, and layout: inline, V,
  flat/boxer or Wankel) and it scales from a sub-litre triple to a
  V12. The redline is set on the tile or learned while you drive.
  "rock x", "thump x" and "buzz x" mix the three parts by feel, and
  the limiter bounce has its own strength, rate and roughness
  controls.
- Wire protocol 1.2 (Docs/PROTOCOL.md): brake, ABS, skid, lockup,
  road, limiter, traction-control and kerb channels. The SimHub plugin
  sends nine channels automatically and ships an annotated settings
  file explaining how to bind the other four (skid, lockup, road,
  kerb) to a SimHub property. Channel bindings ship complete, and a
  Sim channels editor in Setup changes them if you need to.
- Operate and Setup views: Operate keeps the daily surfaces (drives,
  haptics, devices, per-axis config, log); Setup holds the build-time
  ones (host settings, provisioning, commissioning test, bindings,
  updater).
- Optional web password: tick Web password in the host settings, type
  one, save and restart the service; every browser then enters it
  once. Off by default. Plain HTTP, so it keeps out casual access but
  does not encrypt.
- Provisioning role check: Check drive settings reads the
  role-critical settings stored in each drive (runaway protection
  C06.20 first) with the loop stopped, and flags any drive that does
  not match its axis's role, with the panel fix. Those settings travel
  with the physical drive, so a swapped or replaced drive is caught
  before it faults.
- The Pi installer is stable by default: a fresh clone installs the
  latest official release, never a half-finished change. A checked-out
  tag is respected, and --main builds the development branch on
  purpose.
- Every change is now tested by starting the real controller in
  simulation mode and loading the dashboard in a real browser, so a
  page that loads but doesn't work can no longer ship; the telemetry
  input is also stress-tested with malformed data.

### Fixed
- Belt or shifter lunge and fault on the first Initialize after a
  start or restart: a torque drive was switched on before it had been
  given a valid command, so with the belt or lever away from zero it
  lunged and faulted (0xff00). The command is now set before the drive
  is enabled.
- One drive fault no longer takes every drive down. A diagnostic read
  made after a fault stalled the EtherCAT bus, dropped every drive to a
  sync fault and blocked recovery; it has been removed. Drive cards
  still show the fault code; the exact Er code is on the drive panel.
- Haptics changes could not be saved: a haptics-only edit left Save
  disabled. The strip now has its own unsaved marker and Save, and the
  route editor keeps every value you type.
- Saving no longer asks for a restart it doesn't need. Unchanged files
  are not rewritten, and the pending pill says what is actually owed:
  re-initialize for rig settings, restart for host settings; haptics
  and device feel apply live.
- A web save no longer erases the sim channel bindings from rig.json.
- The H layout derive places the gates from the taught travel (just
  inside each stop, neutral at centre); the Throw field is an optional
  override. The old fixed default could put gates outside the real
  travel.
- A refused save now says so on the Devices card, not only beside the
  distant Save button.
- Changing a device's homing direction or mirror warns that the taught
  travel, neutral and gates need re-teaching.

## [0.9.5] - 2026-08-29

### Added
- Force devices, first slice (Pi): two new axis types, `shifter` and
  `pedal`, driven in torque mode by a per-cycle force model (centring
  spring, detents, end stops, damping, free play - all curve-based with
  its own guard chain). A device homes by pushing gently against its
  travel stop, rests limp until engaged, and engages/releases through
  `/api/device/engage|release` - belt and park commands never touch it.
  The web UI gains a Devices section with starter presets and a
  drag-the-nodes editor for the spring and detent curves, enabled by a
  new `webShowDevices` host setting. Bench validation of the feel is
  still ahead of it.
- A second telemetry line format, `NULLCATX,<ch0>,...`, carries raw sim
  values (rpm, speed, gear, clutch) on the same UDP port at its own
  rate; the rig's new `ncxBindings` config maps numbered channels onto
  named tokens. First effects driven by it: clutch-up shift blocking
  and gear grind on shifter axes - inert unless configured, and any
  stall of the channel stream drops every effect back to plain feel.
  A minimal SimHub exporter plugin ships in `integrations/simhub/`;
  any tool that can compose a text UDP line works the same. Setup and
  tuning: Docs/DEVICES.md.
- Revmatch let-in: gear ratios are learned per car while driving (rpm
  per km/h per gear; cars are identified by their ratio set and
  remembered across sessions in carcache.json - no car database), and
  a clutchless shift with the engine blipped to within `rpmMatchPct`
  of the destination gear goes in instead of grinding.
- Devices are lifecycle-independent of the rig: they never home at
  loop start, home-all, or e-stop release. Their button is three-state
  (first press homes and rests limp, next engages, engaged releases),
  identical on the web card, the bindable `device-toggle`, and a new
  GPIO device button; a GPIO belt button joins it, and any panel
  button pin can be set 0 = not fitted for compact builds.
- Device settings live-apply on save while the device is limp - feel
  tuning with no restarts. Travel is taught by sweep (home, waggle the
  lever, Capture travel) and the gate layout derives from a profile:
  H/sequential, N-slot selector (automatic-style lever on the same
  hardware), or custom. Hand-crafted feels save as named presets.
- Shifter feel: dry friction and breakout asymmetry (out of gear firm,
  into gear easy) join the model; presets re-cut with the detents at
  the engagement positions and a gate-shaped spring; the curve editors
  show a live lever dot; the belt guard fields explain what each knob
  does to the belt.
- The Pi updates itself: a Software block in the web UI checks GitHub
  for a newer release and updates on click (rig parked, EtherCAT
  stopped). Releases now ship a prebuilt Pi tarball, installs live as
  version directories under /opt/nullcat with config carried forward,
  and a failed update rolls back automatically (two generations kept).
  Installs made before 0.9.5 adopt the layout with one final manual
  `git pull && ./pi/os-setup/install.sh`.

### Fixed
- The drive-side following-error window (0x6065) is clamped to each
  axis's own travel at the init write - a window wider than the stroke
  could never trip, silently disabling the protection. Configs are not
  modified; the clamp is logged.
- The wedged-SYNC0 recycle now performs the full per-slave rebuild
  (INIT walk + reconfig + fresh arm) that field data showed actually
  clears the wedge, instead of the shallow PreOP walk that did not.
  Unreadable DC margins are labelled as such instead of asserting a
  pulse state that was never observed.
- Re-running install.sh for the version already installed destroyed
  the live config before adopting it (bench edits lost). The installer
  now stashes the config first, and both it and the updater carry
  carcache.json and devicepresets.json forward.
- The EtherCAT init-failure message no longer tells Pi users to check
  Npcap (Windows advice); Linux gets NIC-name/capability guidance.
- Applying a preset no longer overwrites the taught travel, neutral,
  gates, homing direction, or mirror - presets are feel only (a preset
  once wiped a bench-taught travel).
- A telemetry bind address the machine does not own (wrong IP, stale
  DHCP lease, interface not up yet at boot) now falls back to all
  interfaces with a loud log line instead of leaving telemetry dead
  for the whole session; the web header shows SOCKET FAILED when the
  bind genuinely could not happen, and the bind field explains it
  wants THIS machine's address. Found the hard way on the bench.
- The device homing confirm window now actually runs its extra
  confirmation cycles (a threshold was read before the state advanced,
  collapsing the window - visible in logs as a suspiciously fast
  search).
- A browser tab left open across a version update now reloads itself
  (or asks, if there are unsaved edits) instead of silently running
  the previous version's page.
- The recovery thread's diagnostic mailbox reads (drive fault
  history, the precise panel Er code) are now short (50 ms), properly
  serialized against the cyclic exchange, and paced so a VERIFIED PDO
  frame passes between any two reads - diagnosing a drive fault can no
  longer stall the bus into a watchdog cascade. The panel-code read had
  raced the RT exchange entirely unserialized.
- A Download logs button (Logging section) saves a one-file support
  bundle - recent log, app and soem log tails, rig and host config,
  version - to whichever computer runs the browser.
- An unescaped apostrophe in a tooltip string broke the whole web
  dashboard on a fresh load (stuck Disconnected, no controls) - the
  file failed to parse, so no script ran at all. Tabs already open
  from an earlier build kept working, which masked it (#1, thanks
  @Haarie). CI now syntax-checks the web JS so this class cannot
  ship again.

## [0.9.4] - 2026-08-28

### Added
- Hexapod support, first slice: per-axis homing (`/api/home` `{"axis":N}`
  plus buttons in the test panel - for unloaded direction checks and
  near-park re-homes; attached platforms home all legs together), a
  six-lever example config, and Docs/HEXAPOD_SETUP.md. The motion-cycle
  test gives hexapod legs heave and per-leg excitation only: platform
  pitch/roll needs the real leg geometry, so it stays with the motion
  software that owns the kinematics.

### Fixed
- Cold-start init: the post-re-arm SYNC0 health reads are wkc-gated, so
  a dropped read can no longer declare a healthy drive dead - in field
  testing this alone took a cold start from five Initialize attempts to
  one. A wedged pulse unit additionally gets an in-attempt recycle
  (PreOP walk, disarm, re-arm; `sync0RecycleRounds`, default 2, 0
  disables), narrated in the log; a slave it cannot revive gets clear
  advice (fresh init, then power cycle).
- Commissioning refusals now say what to do ("press Park All, keep the
  loop running") instead of naming internal axis states.
- Commissioning offsets now follow axis polarity the way telemetry does,
  so mirrored lever pairs heave together instead of differentially.
  Uninverted rigs are unchanged; on inverted axes test motion mirrors,
  so before/after tuning comparisons should re-baseline on this version
  (step direction affects vertical-axis results via gravity).
- Rotary units are honest end to end: engine and homing logs print
  degrees on levers, commissioning result rows are unit-tagged, and the
  following-error abort rail scales to each axis's travel: min(10, 20%
  of the usable half-range), so a default 100 mm linear axis keeps the
  familiar 10 and shorter axes rail proportionally.
- Rotary axes: the following-error window is now capped to the axis arc
  server-side, not only in the web editor. A hand-edited rig.json can no
  longer disable drive-side runaway protection on a lever.

### Changed
- RT loop hardening: the telemetry drain is bounded per cycle, rejected UDP
  packets no longer allocate on the RT thread, and following-error/dither
  diagnostics no longer run against torque axes (their numbers there were
  noise).

### Internal
- The motion controller's per-cycle monolith is decomposed into
  per-family step functions with a single telemetry-decode authority.
  Byte-identical by proof: three golden-sequence tests captured before
  the change pass bit-exact after it. This is the seam future torque
  device families plug into.
- Belt classification re-keyed: the belts commands and status aggregates
  now key on "belt-typed torque axis" instead of torque mode alone.
  Behaviour-identical (pinned by a bit-exact golden-sequence test);
  groundwork for future torque devices that are not belts.
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

[0.9.4]: https://github.com/crossthenoughts/nullCAT/compare/v0.9.3...v0.9.4
[0.9.3]: https://github.com/crossthenoughts/nullCAT/compare/v0.9.2...v0.9.3
[0.9.2]: https://github.com/crossthenoughts/nullCAT/compare/v0.9.1...v0.9.2
[0.9.1]: https://github.com/crossthenoughts/nullCAT/compare/v0.9.0...v0.9.1
[0.9.0]: https://github.com/crossthenoughts/nullCAT/releases/tag/v0.9.0
