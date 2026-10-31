# nullCAT telemetry wire protocol

The UDP wire between motion/telemetry software (the **sender**: SimHub with
the nullCAT exporter plugin, or any tool that can emit a text line) and the
nullCAT controller (the **receiver**). This document is the contract: the
wire only ever changes here, version-bumped, and senders adapt to it.

**Protocol version: 1.11** (nullCAT 0.9.10). History at the bottom.

nullCAT does not depend on any particular sender. SimHub with the nullCAT
plugin, FlyPT Mover, SimTools, a custom feeder reading the game's shared
memory: anything that can emit a text line over UDP is a sender. The
motion software owns the game; nullCAT owns the rig.

## Transport

- UDP, one datagram per line, plain ASCII. Default port **4444**
  (rig-configurable, `telemetryPort` in host.json).
- No timestamp on the wire; no handshake; no acknowledgements. Senders may
  simply stop - the receiver has staleness fail-safes at every layer and
  parks or idles gracefully.
- One optional line goes the other way (line type 4): the receiver asks a
  sender that says it has a direct source to turn it on or off. A sender
  that never says so never receives it.
- Three line types share the port. Each runs without the others, at its
  own rate.
- A datagram that starts with a space (padding with nothing on it, from a
  sender that fills a fixed-size packet whatever it has to say) is ignored.
  A line may be padded with spaces after its last field.

## Line type 1: motion

```
NULLCAT,<pos0>,<pos1>,...,<posN>\n
```

Every field after the header is one axis value, in the rig's axis order,
**16-bit wire units: 0..65535**. Up to 10 axes are read.

- **Position axes** (linear actuators, rotary levers): centre **32767**
  maps to the axis's configured centre; 0 and 65535 map to centre minus and
  plus half the axis stroke. Per-axis inversion, spike filtering, and
  min/max clamping happen receiver-side - senders never compensate for rig
  mechanics.
- **Belt/tension axes**: the same 0..65535 encoding, but the value is
  tension: **0 = minimum tension** (the configured floor, not slack) and
  65535 = maximum. A belt axis's neutral on the wire is 0, not 32767.
- **Force-device axes** (shifter, active pedal): the slot must be present
  but its value is **ignored** - devices are driven by their own force
  model. Send 32767 as a placeholder.
- Every configured slot must carry a value: empty or malformed fields are
  skipped and the array compacts, which shifts every later axis one slot
  left. Never omit a field.

## Line type 2: channels

```
NULLCATX,<ch0>,<ch1>,...,<chN>\n
```

Up to **96** plain-number channels carrying raw sim values for the device
state effects and the haptic layer. The wire carries numbers only;
**meaning is assigned receiver-side** by the rig's `ncxBindings` config
(each binding: channel slot, token, scale, offset). The recommended
default slot order matches the token table below, making default configs
scale 1, offset 0. A sender may stop at any slot: channels it does not
send are simply absent (their effects stay inert). Slots are counted by
position: a field left empty, or holding something that is not a finite
number (`NaN`, `inf` and an overflow count as not a number), leaves that
one slot absent and the slots after it keep their places. A motion line (`NULLCAT`) is different: there
a bad field is skipped and the values after it close up.

Channel staleness fail-safe: if the channel stream (NULLCATX or NULLCATY)
stops for 500 ms, all channel-driven behaviour (shift blocking, grind,
revmatch, haptic effects) goes inert until it returns. The same applies
when packets keep arriving but no numeric value changes for 2 s (a
paused game, a sender repeating its last frame): the stream is treated
as frozen until a value changes. Senders should simply stop sending
while the game is paused; they need not send anything special.

## Line type 3: named channels

```
NULLCATY,<key>=<value>,<key>=<value>,...\n
```

The same channels by **name**: each key is a token from the registry
below and the value is in that token's canonical unit (no bindings, no
scale or offset). A line may carry any subset of tokens, in any order;
values persist until the stream goes stale, or until a line names a
different game, car or car id (every held value is dropped then, so a
token one game sends and the next does not never carries over). Two
string keys ride on the same line type:

| Key | Value |
|---|---|
| `game` | the running sim's name, free text (up to 47 characters) |
| `car` | the current car as the sim shows it, free text (up to 47 characters) |
| `carId` | optional: the car's stable id in that sim (Assetto Corsa: the content folder, e.g. `ks_mazda_787b`), up to 47 characters. nullCAT's car table is keyed by it where a sim has one; without it the table matches on `car`. A different `carId` under the same `car` is a different car (another build or config of it): what nullCAT learned about the car starts over. An id first sent late, or left off a line, under the same `car` is the same car *(since 1.10)* |
| `direct` | optional: whether the sender is sending from a direct source of its own for this game (a faster feed straight from the sim, as the SimHub plugin's Assetto Corsa read): `on`, `off`, `none` (it has none for this game), or `only` (it is a direct source and has no other, so it cannot switch: a game's own nullCAT mod). Send it with the identity. A sender that says `on` or `off` can be asked to switch (line type 4); one that says `only` or `none` never is. While a sender says `on` or `only` and keeps sending, nullCAT takes the channel lines (line types 2 and 3) from it alone and sets every other sender's aside (motion lines are never set aside), starting its store clean on each change of source; when it says `off` or `none`, when another sender's line comes five minutes after its last (a game paused in a menu sends nothing for as long as it sits there), or, five seconds or more after its last, when a set-aside line from any sender names another `game` than its own (another game started; names are compared on their letters and digits alone, case aside, so `BeamNG.drive` and `BeamNgDrive` are one game), the others are used again (until the direct sender has named its game, only the first sender to name one in a set-aside line naming another counts); a sender whose hold ran out takes the source back with its first line. So a game's own sender and the SimHub plugin can run side by side *(since 1.9; `only` and the setting aside since 1.10)* |

Unknown keys, keys without `=`, and non-numeric values for a numeric
token (`NaN` and `inf` included) are skipped; the rest of the line still
lands. Whitespace around
keys, `=` and values is ignored. A line that lands nothing is rejected.
Keys are case-sensitive.

Use NULLCATY when the sender is a template you write by hand (FlyPT
Mover, SimTools, a custom feeder): there is no slot order to get right,
and the line documents itself. Send the identity (`game`, `car`) on its
own line about once a second and the channels at the data rate. NULLCATX
and NULLCATY may both be used at once; a named value wins over a slot
binding for the same token.

Example lines:

```
NULLCATY,game=Automobilista 2,car=Formula Ultimate Gen2,carId=Formula Ultimate Gen2
NULLCATY,rpm=11480,speedKmh=212.4,gear=5,throttlePct=100,brakePct=0,slipRatioRL=0.08,slipRatioRR=0.31,slipAngleFL=2.1
```

### Token registry (protocol 1.9)

| Slot | Token | Unit / convention |
|---|---|---|
| 0 | `rpm` | engine rpm |
| 1 | `speedKmh` | road speed, km/h |
| 2 | `gear` | numeric; **negative = reverse, 0 = neutral, 1..8 = forward**. Rounded to nearest integer receiver-side. (BeamNG OutGauge senders: subtract 1 from OutGauge's 0=R/1=N/2=first convention.) |
| 3 | `clutchPct` | clutch pedal, 0..100 (100 = fully depressed) |
| 4 | `throttlePct` | throttle, 0..100 |
| 5 | `brakePct` | brake, 0..100 *(since 1.1)* |
| 6 | `absActive` | 0 or 1; ABS currently cycling *(since 1.1)* |
| 7 | `skid` | tyre slip magnitude, 0..100 *(since 1.1)*. Fallback for the lateral slip effect when no per-wheel slip angles are sent: feeds all four wheels. |
| 8 | `lockup` | wheel-lock-under-braking severity, 0..100 *(since 1.1)*. Fallback for the longitudinal slip effect when neither per-wheel slip ratios nor wheel speeds are sent: feeds all four wheels as lock. |
| 9 | `roadNoise` | road surface activity, 0..100 *(since 1.1)* |
| 10 | `limiter` | 0 or 1; engine bouncing off the rev limiter *(since 1.2)* |
| 11 | `tcActive` | 0 or 1; traction control currently cutting *(since 1.2)* |
| 12 | `curbs` | kerb-strip contact magnitude, 0..100 *(since 1.2)* |
| 13 | `maxRpm` | the car's redline, rpm, when the sim exposes it; 0 or absent = nullCAT learns it while you drive *(since 1.3)* |
| 14..17 | `slipAngleFL`, `slipAngleFR`, `slipAngleRL`, `slipAngleRR` | tyre slip angle per wheel, **degrees**, signed (the sign is not used) *(since 1.3)* |
| 18..21 | `slipRatioFL` .. `slipRatioRR` | longitudinal slip ratio per wheel, signed: **negative = the wheel turns slower than the road (locking), positive = faster (spinning)**; -1 = fully locked *(since 1.3)* |
| 22..25 | `wheelSpeedFL` .. `wheelSpeedRR` | wheel rotational speed per wheel in **any unit** (rev/s, rad/s, km/h at the tread): nullCAT learns each wheel's rolling factor against `speedKmh` while cruising, so staggered tyre sizes and unknown radii need no setup. Used when `slipRatio*` is not sent *(since 1.3)* |
| 26..29 | `loadFL` .. `loadRR` | vertical tyre load per wheel, any unit (only the ratio between wheels is used): the loaded tyre's slip is weighted up. Optional *(since 1.3)* |
| 30..33 | `suspVelFL` .. `suspVelRR` | suspension velocity per corner, **mm/s**, signed (positive = compressing). The Road effect replays each corner from these *(since 1.3)* |
| 34 | `boost` | turbo boost, **bar** relative to atmosphere (vacuum negative or 0). Heavier firings under boost and the blow-off on a lift; absent = the engine effect uses rpm instead *(since 1.3)* |
| 35 | `pitLimiter` | 0 or 1; pit limiter engaged. Cuts like the rev limiter without teaching the redline *(since 1.3)* |
| 36..39 | `wheelSlipFL` .. `wheelSlipRR` | the sim's own combined tyre slip magnitude per wheel, scaled so that **100 = the tyre has let go** (Assetto Corsa's `wheelSlip`, which reads about 6 at a clear slide, x 16.7; a slip speed in m/s x 20). The lateral slip effect's source where the sim has no slip angle: any sliding tyre plays from the effect's onset % of its peak % up to full at peak %, so peak % trims the scale *(since 1.4)* |
| 40..43 | `suspTravelFL` .. `suspTravelRR` | suspension travel per corner, **mm**, signed, for sims that give position rather than velocity (Assetto Corsa). The Road effect takes the bumps out of it; `suspVel*` wins when both are sent *(since 1.4)* |
| 44..47 | `roadHeightFL` .. `roadHeightRR` | height of the road under each tyre (its contact point), **mm**, world vertical, hills included: nullCAT takes the hills out. The Road effect's tyre model runs each corner over it; the Rumble strip effect can find kerbs in it *(since 1.5)* |
| 48..51 | `surfaceFL` .. `surfaceRR` | what the ground under each tyre is: **0 tarmac, 1 bumpy tarmac, 2 kerb, 3 gravel, 4 grass, 5 dirt, 6 cobbles**, and since 1.6 **7 snow, 8 ice, 9 sand, 10 mud, 11 standing water**. The sender maps its sim's own surface names onto these. The Rumble strip effect plays a rumble strip under a tyre on 2; the Road effect shapes its road's roughness by class; the Surface effect plays stones, crunch and studs by it *(since 1.5)* |
| 52 | `accHeave` | body vertical acceleration, **m/s²**, up positive, gravity left out (0 standing still; every sender does so). A sender that includes gravity is still read right: the belt from g takes a steady 1 g seen standing still off, and the other uses take only its change *(since 1.5)* |
| 53, 54 | `pitchDeg`, `rollDeg` | body pitch and roll, **degrees**, either sign convention: nullCAT learns which way they turn against the suspension *(since 1.5)* |
| 55, 56 | `wheelbase`, `trackWidth` | the car's wheelbase and track, **m** *(since 1.5)* |
| 57 | `rain` | rain falling, **0..1** (0 dry weather, 1 the heaviest). Where no wetness is sent, nullCAT builds the water standing on the road from it (a minute of heavy rain to stand, several to dry) *(since 1.6)* |
| 58 | `wet` | water on the road, **0..1** (0 dry, 1 standing water), where the sim knows it; wins over `rain` *(since 1.6)* |
| 59..62 | `wetFL` .. `wetRR` | the same under each tyre, where the sim knows it; wins over `wet` *(since 1.6)* |
| 63..66 | `onGroundFL` .. `onGroundRR` | **1** the tyre touches the ground, **0** it is in the air. Where it is not sent, nullCAT tells a tyre in the air from its `load*` (almost none of the usual corner load). A tyre in the air gets no road, surface, kerb or slip *(since 1.7)* |
| 67..70 | `deflatedFL` .. `deflatedRR` | **1** the tyre is punctured, **0** inflated, as the sim says. Feeds the Wheels effect's puncture *(since 1.7)* |
| 71..74 | `slipSpeedFL` .. `slipSpeedRR` | the speed the tyre's contact patch slides over the road, **m/s**, either sign (only the size is used). Where the sim has no slip angle, the lateral slip effect works an angle out of it: it takes out the part `slipRatio*` (or the wheel speeds) already explain along the road, sets what is left against the road speed, and learns on straights the small angle the car always shows there. Wins over `wheelSlip*` for the lateral effect *(since 1.8)* |
| 75 | `inPit` | **1** the car is in its pit box or garage, **0** it is not. Rising after 5 s out, within a second of a jump of the road speed to a standstill or of the stream coming back after a gap (the pause menu), the Wheels effect takes it for a return to the pits: fresh tyres. Driving in does not count. Send it only where the sim really has it: a steady 0 reads as "on track" *(since 1.9)* |
| 76 | `lap` | the current lap number. Going back, and staying back for three quarters of a second, is a restart: fresh tyres. 0 or less is no lap *(since 1.9)* |
| 77..80 | `tyreWearFL` .. `tyreWearRR` | tread worn per tyre, **0..1** (0 new, 1 worn out), where the sim has it. One falling by more than 0.2 and staying there for three quarters of a second is new tyres at a stop: fresh tyres *(since 1.9)* |
| 81, 82 | `accSurge`, `accSway` | the body's acceleration along and across the car, **m/s²**, as the sim's own physics gives it (the same as `accHeave`). For the belt and haptic effects; the motion cue stays the motion software's *(since 1.9)* |
| 83..85 | `yawRate`, `pitchRate`, `rollRate` | the rates the body turns at, **deg/s**, as the sim gives them *(since 1.9)* |
| 86..90 | `engCylinders`, `engLayout`, `engLitres`, `engTurbo`, `engLimitHz` | what the car's engine is, where the sender knows it: the cylinders (or rotors), **1..16**; the layout, **0 inline, 1 V, 2 flat / boxer, 3 rotary, 4 two-stroke, 5 electric**; the displacement, **litres**; **1** turbocharged, **0** not; the rev limiter's cuts a second. Each optional; send them with the identity. For a car the car table has no entry for, with follow car on, nullCAT puts them on the Engine and Limiter tiles over the defaults, once per car (a table entry, stock or the user's, always wins) *(since 1.11)* |
| 91 | `carType` | the type of car, where the sender knows it: **0 road, 1 cup / track, 2 GT / race, 3 open wheeler, 4 rally gravel, 5 rally tarmac**. For a car the car table has no entry for, with follow car on, nullCAT applies the matching starting point once per car; the user can change it after *(since 1.11)* |

Per-wheel groups are always sent as all four or not at all: a group with
a wheel missing is treated as absent. A sender that only has per-axle
data sends the axle's value on both of its wheels.

**Sender-side adaptation rule:** per-game knowledge lives in the sender,
and only that. The sender maps what the current game exposes onto these
tokens, in these units; it does no shaping, thresholding or scaling to
0..100 for the per-wheel tokens, since nullCAT holds the tyre model
(onset, limit, load weighting, staging) and keeps it the same for every
sim. A game that exposes nothing for a token simply does not send it (or
sends 0 on NULLCATX) - the corresponding effect is silently inert, never
wrong. The 0.9.6 magnitude channels (skid, lockup, roadNoise, curbs)
remain as sender-computed 0..100 summaries for senders that have nothing
finer.

## Parser tolerances (pinned by the receiver's test suite)

- Header match is case-insensitive and ignores embedded whitespace.
- Fields are parsed as decimal numbers (integer or floating point).
- Empty or garbage fields are skipped and the array **compacts** (see the
  warning under line type 1).
- Fields longer than 63 characters are treated as garbage.
- Lines that do not start with a recognised header (before the first
  comma) are ignored entirely.

## Rates

Send at a steady fixed rate from a high-priority thread. 100-500 Hz is the
well-trodden range for motion; channels may run slower (60-100 Hz is
plenty). The receiver measures arrival and new-frame rates and hints its
conditioning mode from them.

## Sender templates

FlyPT Mover, SimTools and similar tools have an output module that sends
a text pattern with the game's fields substituted. A NULLCATY pattern
with the fields the tool has is a complete sender; include whichever
tokens exist and leave the rest out. Two lines, one at the data rate and
one slowly for the identity:

```
NULLCATY,rpm={rpm},speedKmh={speed_kmh},gear={gear},throttlePct={throttle_pct},brakePct={brake_pct},clutchPct={clutch_pct},slipAngleFL={slip_angle_fl_deg},slipAngleFR={slip_angle_fr_deg},slipAngleRL={slip_angle_rl_deg},slipAngleRR={slip_angle_rr_deg},slipRatioFL={slip_ratio_fl},slipRatioFR={slip_ratio_fr},slipRatioRL={slip_ratio_rl},slipRatioRR={slip_ratio_rr}
NULLCATY,game={game_name},car={car_name},carId={car_id}
```

Units matter: degrees for slip angles, a signed ratio for slip ratios,
km/h for road speed, 0..100 for the pedals. Wheel speeds and loads can be
in whatever the tool has. The motion line (`NULLCAT,...`) is unchanged
and can come from the same tool.

## Line type 4: receiver to sender

```
NULLCATC,direct=on\n
NULLCATC,direct=off\n
```

The receiver's **direct telemetry** switch. nullCAT sends it from its
telemetry port to the address and port the sender's `NULLCATY` lines
come from (so a firewall passes it as a reply): when the switch changes,
and then at most once a second while the sender's `direct` key says `on`
or `off` and differs from the switch. Nothing is sent to a sender that
never sends `direct`, or says `none`. A sender should accept it only
from the receiver (the address it sends to, or that address's telemetry
port: a receiver with several addresses may answer from another), switch
its direct source, and report the result in its next identity line.
Unknown keys on the line are ignored *(since 1.9)*.

## Version history

- **1.11** (nullCAT 0.9.10): added `engCylinders`, `engLayout`,
  `engLitres`, `engTurbo` and `engLimitHz`, what the car's engine is, and
  `carType`, its type of car, at slots 86..91. For a car the car table has
  no entry for, nullCAT sets its Engine and Limiter tiles and its starting
  point from them. The BeamNG.drive mod sends them. No change to parsing;
  every 1.10 sender remains fully compatible.
- **1.10** (nullCAT 0.9.10): added `direct=only`, a direct source that
  cannot switch (a game's own nullCAT mod, such as BeamNG's). A sender
  saying `direct=on` or `only` is the channel source while it sends, and
  the other senders' channel lines are set aside meanwhile. A different
  `carId` under the same `car` is a different car. A datagram that starts
  with a space is ignored. An older receiver reads `only` as not said;
  every 1.9 sender remains fully compatible.
- **1.9** (nullCAT 0.9.9): added the `direct` identity key and line
  type 4 (`NULLCATC,direct=on|off`, receiver to sender), nullCAT's
  direct telemetry switch. Added `inPit`, `lap` and `tyreWearFL` ..
  `tyreWearRR`, so the Wheels effect brings fresh tyres (no flat spots,
  cold discs) on a restart, a return to the pits or the garage and new
  tyres at a stop, and keeps them through a drive into the pits. A sender
  without the pit flag gets a fallback instead: a jump of the road speed
  to a standstill that holds. The reference SimHub plugin sends the lap for
  every game; the pit flag from Automobilista 2 and the Assetto Corsa
  family from the start, and from other games once SimHub has shown it
  set; the tyre wear from Automobilista 2 and Le Mans Ultimate. Also
  added `accSurge`, `accSway`, `yawRate`, `pitchRate` and `rollRate`, the
  rest of the body's motion as the sim gives it, which the reference plugin
  sends from SimHub's motion data for every game that has it. No change to
  parsing; every 1.8 sender remains fully compatible.
- **1.8** (nullCAT 0.9.8): added `slipSpeedFL` .. `slipSpeedRR`, the
  tyre's sliding speed, which the lateral slip effect turns into a slip
  angle where the sim has none. The reference SimHub plugin sends it from
  Automobilista 2, and now sends `accHeave` as SimHub gives it (already
  m/s²; it was being multiplied by g). No change to parsing; every 1.7
  sender remains fully compatible.
- **1.7** (nullCAT 0.9.8): the channel wire widened from 64 to 96 slots
  (every token keeps a slot, with room to grow). Added `onGroundFL` ..
  `onGroundRR` (a tyre in the air goes quiet) and `deflatedFL` ..
  `deflatedRR` (punctures). The reference SimHub plugin sends both from
  Automobilista 2's tyre flags. No change to parsing; every 1.6 sender
  remains fully compatible.
- **1.6** (nullCAT 0.9.8): surface classes 7 snow, 8 ice, 9 sand, 10 mud,
  11 standing water; tokens `rain`, `wet` and `wetFL` .. `wetRR`, for the
  Surface effect (stones, crunch, studs, puddles and aquaplaning) and the
  Road effect's surface-shaped roughness. The reference SimHub plugin
  maps Automobilista 2's snow, ice and sand and sends its rain. No change
  to parsing; every 1.5 sender remains fully compatible.
- **1.5** (nullCAT 0.9.8): the channel wire widened from 48 to 64 slots.
  Added `roadHeight*` and `surface*` per wheel, `accHeave`, `pitchDeg`,
  `rollDeg`, `wheelbase` and `trackWidth`, for the Road effect's tyre and
  chassis models and the Rumble strip effect. The reference SimHub
  plugin sends them where the sim has them, and now sends Assetto Corsa's
  slip angle per wheel (from each tyre's heading against its direction of
  travel). No change to parsing; every 1.4 sender remains fully
  compatible.
- **1.4** (nullCAT 0.9.8): added the per-wheel tokens `wheelSlip*` (a
  sim's combined slip magnitude, the lateral source where no slip angle
  exists) and `suspTravel*` (suspension position for sims without a
  velocity), and the optional `carId` identity string. The reference
  SimHub plugin now sends every channel by name and carries built-in
  per-game bindings. No change to parsing; every 1.3 sender remains
  fully compatible.
- **1.3** (nullCAT 0.9.8): the channel wire widened from 16 to 48 slots.
  Added the per-wheel raw-physics tokens `slipAngle*`, `slipRatio*`,
  `wheelSpeed*`, `load*`, `suspVel*` and `maxRpm`, consumed by the new
  per-wheel Lateral slip and Longitudinal slip effects (`skid` and
  `lockup` remain as fallbacks). Added line type 3, `NULLCATY`, named
  channels plus the `game` and `car` identity strings. No change to the
  motion line or to NULLCATX parsing; every 1.2 sender remains fully
  compatible.
- **1.2** (nullCAT 0.9.6): added tokens `limiter`, `tcActive`, `curbs` for
  the limiter buzz, TC pulse, and kerb rumble effects. No change to line
  formats or parsing; older senders remain fully compatible.
- **1.1** (nullCAT 0.9.6): added tokens `brakePct`, `absActive`, `skid`,
  `lockup`, `roadNoise` for the haptic effect layer. No change to line
  formats or parsing; senders that predate 1.1 remain fully compatible.
- **1.0** (nullCAT 0.9.5): first written spec of the wire as shipped:
  NULLCAT motion lines, NULLCATX channels with tokens `rpm`, `speedKmh`,
  `gear`, `clutchPct`, `throttlePct`.
