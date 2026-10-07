# nullCAT telemetry wire protocol

The UDP wire between motion/telemetry software (the **sender**: SimHub with
the nullCAT exporter plugin, or any tool that can emit a text line) and the
nullCAT controller (the **receiver**). This document is the contract: the
wire only ever changes here, version-bumped, and senders adapt to it.

**Protocol version: 1.3** (nullCAT 0.9.7). History at the bottom.

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
- Three line types share the port. Each runs without the others, at its
  own rate.

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

Up to **48** plain-number channels carrying raw sim values for the device
state effects and the haptic layer. The wire carries numbers only;
**meaning is assigned receiver-side** by the rig's `ncxBindings` config
(each binding: channel slot, token, scale, offset). The recommended
default slot order matches the token table below, making default configs
scale 1, offset 0. A sender may stop at any slot: channels it does not
send are simply absent (their effects stay inert).

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
values persist until the stream goes stale. Two string keys ride on the
same line type:

| Key | Value |
|---|---|
| `game` | the running sim's name, free text (up to 47 characters) |
| `car` | the current car, free text (up to 47 characters) |

Unknown keys, keys without `=`, and non-numeric values for a numeric
token are skipped; the rest of the line still lands. Whitespace around
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
NULLCATY,game=Automobilista 2,car=Formula Ultimate Gen2
NULLCATY,rpm=11480,speedKmh=212.4,gear=5,throttlePct=100,brakePct=0,slipRatioRL=0.08,slipRatioRR=0.31,slipAngleFL=2.1
```

### Token registry (protocol 1.3)

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
NULLCATY,game={game_name},car={car_name}
```

Units matter: degrees for slip angles, a signed ratio for slip ratios,
km/h for road speed, 0..100 for the pedals. Wheel speeds and loads can be
in whatever the tool has. The motion line (`NULLCAT,...`) is unchanged
and can come from the same tool.

## Version history

- **1.3** (nullCAT 0.9.7): the channel wire widened from 16 to 48 slots.
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
