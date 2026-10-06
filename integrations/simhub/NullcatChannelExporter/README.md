# nullCAT Channel Exporter (SimHub plugin)

Sends raw sim telemetry to nullCAT for the force-device effects (shifter,
active pedal) and the haptic effect layer. One UDP line per tick, nothing
else (wire protocol 1.2, see `Docs/PROTOCOL.md` in the nullCAT repo):

    NULLCATX,<rpm>,<speedKmh>,<gear>,<clutchPct>,<throttlePct>,
             <brakePct>,<absActive>,<skid>,<lockup>,<roadNoise>,
             <limiter>,<tcActive>,<curbs>

All the feel and logic lives in nullCAT - this plugin never changes when
effects do. Gear is numeric on the wire: `0` = neutral, `-1` = reverse.
A channel the current game cannot feed sends 0, which leaves its effect
silently inert on the rig.

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
telemetry (nullCAT tells the two streams apart by their headers). Without
the file the plugin sends to `127.0.0.1:4444`.

## What is sent without any setup

Nine channels come from SimHub's standard data and need nothing from you:
rpm, speed, gear, clutch, throttle, brake, ABS active, rev limiter
(computed from rpm vs the car's max) and TC active. They drive the Engine,
Gear shift, ABS, Limiter and TC effects.

## Optional: skid, lockup, road and kerb channels

These four are 0-100 magnitudes that SimHub has no standard property for
(they vary by game), so you point each at ANY SimHub property that yields
0-100. The annotated JSON has one line per channel:

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
the same list. Empty = that channel sends 0 and its effect stays silent.
Values are clamped to 0-100.

## nullCAT side

Nothing to bind: a fresh rig config ships with all 13 channels bound in
this plugin's slot order. If you have changed them, the Sim channels
section of the web Setup view has a "Reset to defaults" button.

See the Haptics section of `Docs/DEVICES.md` in the nullCAT repository
for what each effect does, which channels it needs, and how to tune it.
