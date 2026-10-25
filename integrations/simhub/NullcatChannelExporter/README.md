# nullCAT Channel Exporter (SimHub plugin)

Sends raw sim telemetry to nullCAT for the force-device effects (shifter,
active pedal) and the haptic effect layer. Up to three UDP lines per tick,
nothing else (wire protocol 1.9, see `Docs/PROTOCOL.md` in the nullCAT
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
channels for the haptic layer". Version 1.9 of the plugin (protocol 1.9) goes with
nullCAT 0.9.9; it works with 0.9.6 to 0.9.8 too (the extra channels are simply
ignored there).

SimHub is one sender among others: FlyPT Mover, SimTools or your own
feeder can send the same lines (see the sender templates in
`Docs/PROTOCOL.md`).

## Install

Every nullCAT release carries the plugin ready to use, with this README
and its settings file:

- **nullCAT on Windows:** the `simhub` folder in the Windows zip. Its
  `NullcatChannelExporter.json` sends to `127.0.0.1` (SimHub and nullCAT
  on the same PC).
- **nullCAT on a Pi:** `nullCAT-v<version>-simhub-plugin-pi.zip` on the
  release page. Its settings send to `192.168.50.1`, the Pi's telemetry
  port in Docs/PI_SETUP.md.

1. Close SimHub. Copy `NullcatChannelExporter.dll` into your SimHub
   folder, and on a first install `NullcatChannelExporter.json` next to
   it (when updating, keep your own JSON).
2. Start SimHub and enable **nullCAT Channel Exporter** when prompted
   (or under Settings, Plugins).

To build it yourself instead: `NullcatChannelExporter.csproj` with
Visual Studio or `dotnet build`. If SimHub is not in the default
location, pass `-p:SimHubPath="D:\SimHub"`.

## Point it at your controller

The settings live in `NullcatChannelExporter.json`. When SimHub starts,
the plugin looks for it in three places and reads the first it finds:

1. `%LOCALAPPDATA%\nullCAT\NullcatChannelExporter.json` (paste
   `%LOCALAPPDATA%` into Explorer's address bar and make a `nullCAT`
   folder there). The best place for your own settings: any editor can
   save there, and SimHub updates never touch it.
2. The copy Windows keeps when you edit the file next to the DLL without
   administrator rights (see below).
3. `NullcatChannelExporter.json` next to the DLL, where a release puts it
   (the release copies come preset, see above; a built copy starts at
   `127.0.0.1`).

It is annotated: every `_help...` line explains the setting below it and
is ignored by the plugin. The two settings everyone needs:

```json
"host": "192.168.1.50",
"port": 4444
```

`host` is the Pi or PC running nullCAT; use the same port as your motion
telemetry (nullCAT tells the streams apart by their headers). Without
the file the plugin sends to `127.0.0.1:4444`.

Editing the file next to the DLL: SimHub lives under `Program Files`, and
Windows quietly saves a file you edit there with an ordinary editor into
a private copy, for the usual install
`%LOCALAPPDATA%\VirtualStore\Program Files (x86)\SimHub\NullcatChannelExporter.json`.
The plugin reads that copy too, so the edit counts, but the
`%LOCALAPPDATA%\nullCAT` folder is the simpler place to keep your
settings.

The first file found wins, so a copy in an earlier place hides the ones
after it: an old VirtualStore copy left from an edit long ago would stop
a newer file next to the DLL from counting. Delete the copy you do not
want. To check what the plugin actually read, look in SimHub's log
(`Logs\SimHub.txt` in the SimHub folder) for the line
`nullCAT Channel Exporter: sending to ...`. It names the address and the
settings file it used, any other copies it found but did not read (or
every place it looked, when it found none). Or find
`NullcatChannelExporter.Target` under Settings, Properties.

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
| Assetto Corsa Competizione | slip angle, a real slip ratio, wheel speeds, combined slip, suspension travel, road height, wheelbase and track, the game's kerb vibration, the rain and how wet the track is (no loads: the game leaves them at zero) | Longitudinal slip, Lateral slip, Road, Kerb, Surface (puddles only: no surface under each tyre) |
| Assetto Corsa EVO, Rally | slip angle, a real slip ratio, wheel speeds, loads, combined slip, suspension travel, road height, wheelbase and track, the game's kerb vibration | Longitudinal slip, Lateral slip, Road, Kerb |
| Le Mans Ultimate | wheel speeds, tyre loads, suspension travel, the tyre's slide over the road (nullCAT turns it into a slip angle), the surface under each tyre, punctures, tyre wear, the rain and how wet the track is | Longitudinal slip, Lateral slip, Road, Kerb, Surface, Wheels |
| Automobilista 2 | wheel speeds, suspension velocities, tyre slip speed (nullCAT turns it into a slip angle), the surface under each tyre (snow and ice included), the rain, which tyres are on the ground and which are punctured | Longitudinal slip, Lateral slip, Road, Kerb, Surface, Wheels; a tyre in the air goes quiet |

Every game also sends the body's motion from SimHub's own motion data
(its accelerations up, along and across, its pitch and roll, and the rates
it turns at), for the Road tile's chassis model and nullCAT's belt and
haptic effects, and the
lap number and the pit flag, so the Wheels tile can bring fresh tyres on a
restart or a jump back to the pits. Automobilista 2, Le Mans Ultimate and
the Assetto Corsa family send the game's own pit flag from the start
(Le Mans Ultimate's: in the garage or stopped in the pit box); other
games send
SimHub's once the game has shown it set (a game SimHub cannot read it for
is left to nullCAT's fallback). Automobilista 2 and Le Mans Ultimate also
send their tyre wear, so new tyres at a stop are fresh tyres too.

Plain Assetto Corsa has no slip angle field, so the plugin works one out
for each tyre: the angle between where the tyre points and where its
contact patch actually moved since the last update. It needs the car
rolling (nothing below 5 km/h). The combined slip magnitude still goes
along (scaled so that about 6 on the game's own scale, a clear slide, is
100) and is what Lateral slip uses for a sim without slip angles.

## Optional: Assetto Corsa accelerations at the physics rate

The Assetto Corsa family (AC, Competizione, EVO and Rally) runs its
physics at about 333 steps a second, while SimHub passes its values on
about 60 times a second. Set `"kunosDirect": "on"` in the settings file
and the plugin reads the car's accelerations (along, across and up)
straight from the game's own shared memory on every physics step and
sends them on a short line of their own, for nullCAT's belt and Impacts.

It reads the same telemetry page SimHub reads, for reading only: nothing
is written and no other program is touched.

Before it sends anything, the plugin matches the game's values against
SimHub's while you drive (about 10 s above 20 km/h), so they arrive in
the same units and directions as SimHub's and nothing on the rig needs
changing. Until they match, and whenever they stop matching, SimHub's
values go as before.

nullCAT's **direct telemetry** switch (the Haptics strip's Master tile)
turns it on or off from the rig: the plugin tells nullCAT once a second
whether it is on, off or not available for the game, and nullCAT asks it
to change whenever that differs from the switch. Once you have used the
switch, it wins over the settings file. To compare the two while
driving, bind the SimHub action **ToggleKunosDirect** to a button under
**Controls and events** (with the switch set, nullCAT turns it back
within about two seconds). The property `NullcatChannelExporter.KunosDirect`
shows what it is doing: off, matching, or live with the rate and the
match it found.

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
