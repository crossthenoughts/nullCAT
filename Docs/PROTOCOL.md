# nullCAT telemetry wire protocol

The UDP wire between motion/telemetry software (the **sender**: SimHub with
the nullCAT exporter plugin, or any tool that can emit a text line) and the
nullCAT controller (the **receiver**). This document is the contract: the
wire only ever changes here, version-bumped, and senders adapt to it.

**Protocol version: 1.2** (nullCAT 0.9.6). History at the bottom.

## Transport

- UDP, one datagram per line, plain ASCII. Default port **4444**
  (rig-configurable, `telemetryPort` in host.json).
- No timestamp on the wire; no handshake; no acknowledgements. Senders may
  simply stop - the receiver has staleness fail-safes at every layer and
  parks or idles gracefully.
- Two line types share the port. Either runs without the other, at its own
  rate.

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

Up to 16 plain-number channels carrying raw sim values for the device
state effects and the haptic layer. The wire carries numbers only;
**meaning is assigned receiver-side** by the rig's `ncxBindings` config
(each binding: channel slot, token, scale, offset). The recommended
default slot order matches the token table below, making default configs
scale 1, offset 0.

Channel staleness fail-safe: if the NULLCATX stream stops for 500 ms, all
channel-driven behaviour (shift blocking, grind, revmatch, haptic effects)
goes inert until it returns.

### Token registry (protocol 1.2)

| Slot | Token | Unit / convention |
|---|---|---|
| 0 | `rpm` | engine rpm |
| 1 | `speedKmh` | road speed, km/h |
| 2 | `gear` | numeric; **negative = reverse, 0 = neutral, 1..8 = forward**. Rounded to nearest integer receiver-side. (BeamNG OutGauge senders: subtract 1 from OutGauge's 0=R/1=N/2=first convention.) |
| 3 | `clutchPct` | clutch pedal, 0..100 (100 = fully depressed) |
| 4 | `throttlePct` | throttle, 0..100 |
| 5 | `brakePct` | brake, 0..100 *(since 1.1)* |
| 6 | `absActive` | 0 or 1; ABS currently cycling *(since 1.1)* |
| 7 | `skid` | tyre slip magnitude, 0..100 *(since 1.1)* |
| 8 | `lockup` | wheel-lock-under-braking severity, 0..100 *(since 1.1)* |
| 9 | `roadNoise` | road surface activity, 0..100 *(since 1.1)* |
| 10 | `limiter` | 0 or 1; engine bouncing off the rev limiter *(since 1.2)* |
| 11 | `tcActive` | 0 or 1; traction control currently cutting *(since 1.2)* |
| 12 | `curbs` | kerb-strip contact magnitude, 0..100 *(since 1.2)* |

**Sender-side adaptation rule:** per-game knowledge lives in the sender.
The magnitude channels (skid, lockup, roadNoise) are semantic summaries
the sender computes from whatever the current game exposes (e.g. condense
four suspension velocities into one roadNoise magnitude). A game that
exposes nothing for a channel sends 0 - the corresponding effect is
silently inert, never wrong.

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

## Version history

- **1.2** (nullCAT 0.9.6): added tokens `limiter`, `tcActive`, `curbs` for
  the limiter buzz, TC pulse, and kerb rumble effects. No change to line
  formats or parsing; older senders remain fully compatible.
- **1.1** (nullCAT 0.9.6): added tokens `brakePct`, `absActive`, `skid`,
  `lockup`, `roadNoise` for the haptic effect layer. No change to line
  formats or parsing; senders that predate 1.1 remain fully compatible.
- **1.0** (nullCAT 0.9.5): first written spec of the wire as shipped:
  NULLCAT motion lines, NULLCATX channels with tokens `rpm`, `speedKmh`,
  `gear`, `clutchPct`, `throttlePct`.
