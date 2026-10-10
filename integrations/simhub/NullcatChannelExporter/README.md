# nullCAT Channel Exporter (SimHub plugin)

Sends raw sim telemetry to nullCAT for the force-device effects (shifter,
active pedal) and the haptic effect layer. Up to three UDP lines per tick,
nothing else (wire protocol 1.7, see `Docs/PROTOCOL.md` in the nullCAT
repo):

    NULLCATX,<rpm>,<speedKmh>,<gear>,<clutchPct>,<throttlePct>,
             <brakePct>,<absActive>,<skid>,<lockup>,<roadNoise>,
             <limiter>,<tcActive>,<curbs>,<maxRpm>       (for older receivers)
    NULLCATY,rpm=..,speedKmh=..,...,boost=..,pitLimiter=..,wheelSpeedFL=..,...
             (everything by name: the classic channels, boost, the pit
             limiter, and the per-wheel channels for the running game)
    NULLCATY,game=..,car=..,carId=..             (once a second)

All the feel and logic lives in nullCAT - this plugin never changes when
effects do. Gear is numeric on the wire: `0` = neutral, `-1` = reverse.
Everything goes by name on the second line, so nothing on the rig side
needs mapping for this plugin; the numbered line is kept for receivers
older than 0.9.8. A per-wheel channel the current game cannot feed is not
sent at all, which leaves its effect silently inert on the rig.

In SimHub's plugin list it appears as **nullCAT Channel Exporter**:
"Sends raw telemetry to nullCAT over UDP: rpm, speed, gear, pedals,
ABS/TC/limiter flags, and optional per-wheel slip, load and suspension
channels for the haptic layer". Version 1.7 of the plugin goes with
nullCAT 0.9.8; it works with 0.9.6 and 0.9.7 too (the extra channels are simply
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

Editing the file where it sits: SimHub lives under `Program Files`, and
Windows quietly saves a file you edit there with an ordinary editor into
a private copy (`%LOCALAPPDATA%\VirtualStore\Program Files (x86)\SimHub`)
that SimHub never reads, so your change seems to vanish. Edit the file
somewhere else and copy it into the SimHub folder, or run the editor as
administrator. To check what the plugin actually read, look in SimHub's
log (`Logs\SimHub.txt` in the SimHub folder) for the line
`nullCAT Channel Exporter: sending to ...`, which names the address and
the settings file, or find `NullcatChannelExporter.Target` under
Settings, Properties.

## What is sent without any setup

Twelve channels come from SimHub's standard data and need nothing from
you: rpm, speed, gear, clutch, throttle, brake, ABS active, rev limiter
(computed from rpm vs the car's max), TC active, the car's max rpm,
turbo boost and the pit limiter. They drive the Engine, Gear shift, ABS,
Limiter and TC effects. The game and car names go along once a second.

For the sims the plugin knows, the per-wheel channels need nothing from
you either: it knows which raw fields each game exposes and sends them
by itself.

| Game | Sent per wheel, without setup | Feeds |
|---|---|---|
| Assetto Corsa | slip angle, wheel speeds, tyre loads, the combined slip, suspension travel, the road height under each tyre, wheelbase and track | Longitudinal slip, Lateral slip, Road, Kerb |
| Assetto Corsa Competizione, EVO, Rally | slip angle, a real slip ratio, wheel speeds, loads, combined slip, suspension travel, road height, wheelbase and track, the game's kerb vibration | Longitudinal slip, Lateral slip, Road, Kerb |
| Automobilista 2 | wheel speeds, suspension velocities, tyre slip speed, the surface under each tyre (snow and ice included), the rain, which tyres are on the ground and which are punctured | Longitudinal slip, Lateral slip, Road, Kerb, Surface, Wheels; a tyre in the air goes quiet |

Every game also sends the body's vertical acceleration, pitch and roll
from SimHub's own motion data, for the Road tile's chassis model.

Plain Assetto Corsa has no slip angle field, so the plugin works one out
for each tyre: the angle between where the tyre points and where its
contact patch actually moved since the last update. It needs the car
rolling (nothing below 5 km/h). The combined slip magnitude still goes
along (scaled so that about 6 on the game's own scale, a clear slide, is
100) and is what Lateral slip uses for a sim without slip angles.

## Optional: binding per-wheel channels yourself

For a game the plugin does not know, or to override a built-in group,
bind the game's raw per-wheel fields and nullCAT does the rest (it holds
the tyre model: where slip starts to matter, the limit, load weighting,
how the texture changes as the slide grows). Each group needs all four
wheels, in the order FL, FR, RL, RR; a group you bind replaces the
built-in one for that group only:

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
| `wheelSlip*` | the game's combined slip magnitude per wheel | 100 = the tyre let go (set the scale so a clear slide reads about 100) | Lateral slip, when the game has no slip angle |
| `slipRatio*` | longitudinal slip ratio per wheel | signed ratio, -1 = locked | Longitudinal slip |
| `wheelSpeed*` | wheel rotational speed per wheel | any unit; nullCAT learns the rolling factor | Longitudinal slip, when the game has no slip ratio |
| `load*` | vertical tyre load per wheel | any unit (ratios only) | weights the loaded tyre up (optional) |
| `suspVel*` | suspension velocity per corner | mm/s (`suspVelScale: 1000` for m/s) | Road (replays each corner) |
| `suspTravel*` | suspension travel per corner | mm (`suspTravelScale: 1000` for metres) | Road, when the game gives position rather than velocity |

SimHub numbers a game's per-wheel fields `01` to `04`, and in every game
reader checked that is FL, FR, RL, RR. rFactor 2 and Le Mans Ultimate
expose the contact-patch velocities and tyre load; iRacing exposes
suspension velocities. The raw fields are listed in SimHub under
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

Nothing to bind: every channel from this plugin arrives by name, so the
Sim channels map in the web Setup view (which is for senders that use
the numbered line) does not apply to it.

See the Haptics section of `Docs/DEVICES.md` in the nullCAT repository
for what each effect does, which channels it needs, and how to tune it.
