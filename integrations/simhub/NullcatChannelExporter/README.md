# nullCAT Channel Exporter (SimHub plugin)

Sends raw sim telemetry to nullCAT for the force-device effects (shifter,
active pedal) and the haptic effect layer. Up to three UDP lines per tick,
nothing else (wire protocol 1.3, see `Docs/PROTOCOL.md` in the nullCAT
repo):

    NULLCATX,<rpm>,<speedKmh>,<gear>,<clutchPct>,<throttlePct>,
             <brakePct>,<absActive>,<skid>,<lockup>,<roadNoise>,
             <limiter>,<tcActive>,<curbs>,<maxRpm>
    NULLCATY,boost=..,pitLimiter=..,slipAngleFL=..,...  (boost, pit limiter, the per-wheel channels you bind)
    NULLCATY,game=..,car=..,carId=..             (once a second)

All the feel and logic lives in nullCAT - this plugin never changes when
effects do. Gear is numeric on the wire: `0` = neutral, `-1` = reverse.
A channel the current game cannot feed is sent as 0 (the classic line) or
not at all (the per-wheel line), which leaves its effect silently inert
on the rig. Boost and the pit limiter travel on the named line rather
than as extra positional slots because their slots (34 and 35) sit past
the per-wheel groups, which the plugin only sends when they are bound;
nothing on the rig side needs setting up for that, the named line maps
itself.

In SimHub's plugin list it appears as **nullCAT Channel Exporter**:
"Sends raw telemetry to nullCAT over UDP: rpm, speed, gear, pedals,
ABS/TC/limiter flags, and optional per-wheel slip, load and suspension
channels for the haptic layer". Version 1.3 of the plugin goes with
nullCAT 0.9.7; it works with 0.9.6 too (the extra channels are simply
ignored there).

SimHub is one sender among others: FlyPT Mover, SimTools or your own
feeder can send the same lines (see the sender templates in
`Docs/PROTOCOL.md`).

## Install

1. Build `NullcatChannelExporter.csproj` (Visual Studio or
   `dotnet build`). If SimHub is not in the default location, pass
   `-p:SimHubPath="D:\SimHub"`.
2. Copy `NullcatChannelExporter.dll` into your SimHub folder.
3. Start SimHub and enable **nullCAT Channel Exporter** when prompted
   (or under Settings, Plugins).

## Point it at your controller

Copy the `NullcatChannelExporter.json` from this folder next to the DLL
and edit it. It is annotated: every `_help...` line explains the setting
below it and is ignored by the plugin. The two settings everyone needs:

```json
"host": "192.168.1.50",
"port": 4444
```

`host` is the Pi or PC running nullCAT; use the same port as your motion
telemetry (nullCAT tells the streams apart by their headers). Without
the file the plugin sends to `127.0.0.1:4444`.

## What is sent without any setup

Twelve channels come from SimHub's standard data and need nothing from
you: rpm, speed, gear, clutch, throttle, brake, ABS active, rev limiter
(computed from rpm vs the car's max), TC active, the car's max rpm,
turbo boost and the pit limiter. They drive the Engine, Gear shift, ABS,
Limiter and TC effects. The game and car names go along once a second.

## Optional: per-wheel slip for the tyre effects

The Lateral slip and Longitudinal slip effects work per wheel. Bind the
game's raw per-wheel fields and nullCAT does the rest (it holds the tyre
model: where slip starts to matter, the limit, load weighting, how the
texture changes as the slide grows). Each group needs all four wheels,
in the order FL, FR, RL, RR:

```json
"slipAngleFLProp": "DataCorePlugin.GameRawData....",
"slipAngleFRProp": "...",
"slipAngleRLProp": "...",
"slipAngleRRProp": "...",
"slipAngleScale":  1
```

| Group | What to bind | Unit on the wire | Used by |
|---|---|---|---|
| `slipAngle*` | tyre slip angle per wheel | degrees (`slipAngleScale: 57.2958` for radians) | Lateral slip |
| `slipRatio*` | longitudinal slip ratio per wheel | signed ratio, -1 = locked | Longitudinal slip |
| `wheelSpeed*` | wheel rotational speed per wheel | any unit; nullCAT learns the rolling factor | Longitudinal slip, when the game has no slip ratio |
| `load*` | vertical tyre load per wheel | any unit (ratios only) | weights the loaded tyre up (optional) |
| `suspVel*` | suspension velocity per corner | mm/s (`suspVelScale: 1000` for m/s) | Road (replays each corner) |

Which fields exist depends on the game. Assetto Corsa, Competizione and
EVO expose slip angle, slip ratio and (AC, EVO) load per wheel; rFactor 2
and Le Mans Ultimate expose the contact-patch velocities and tyre load;
Automobilista 2 exposes wheel speeds (`mTyreRPS`) and suspension
velocities but no slip, so bind `wheelSpeed*` there; iRacing exposes
suspension velocities only. The raw fields are listed in SimHub under
Settings, Properties, filtered on `GameRawData`.

## Optional: the single magnitude channels

Where a game gives nothing per wheel, the four 0-100 magnitudes still
work. `skid` and `lockup` are the fallback for the two slip effects (they
feed all four wheels at once); `road` and `curbs` drive their own effects.

```json
"skidProp":   "",
"lockupProp": "",
"roadProp":   "",
"curbsProp":  ""
```

To find a property name: SimHub, Settings, Properties lists every
property with its live value; copy the exact text, dots included. A name
looks like `PluginName.Section.ValueName`, for example
`DataCorePlugin.GameData.Brake` (that one is the brake, already sent).
You can also build your own with an NCalc formula, and ShakeIt effects can
be exported as properties from the effect's settings; both then appear in
the same list. Empty = not sent and its effect stays silent. Magnitudes
are clamped to 0-100; per-wheel values are sent as they are, times the
group's scale.

## nullCAT side

Nothing to bind: a fresh rig config ships with every channel bound in
this plugin's slot order, and the per-wheel line names its channels
itself. If you have changed the bindings, the Sim channels section of the
web Setup view has a "Reset to defaults" button.

See the Haptics section of `Docs/DEVICES.md` in the nullCAT repository
for what each effect does, which channels it needs, and how to tune it.
