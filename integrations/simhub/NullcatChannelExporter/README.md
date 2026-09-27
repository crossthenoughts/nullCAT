# nullCAT Channel Exporter (SimHub plugin)

Sends raw sim telemetry to nullCAT for the force-device effects (shifter,
active pedal) and the haptic effect layer. One UDP line per tick, nothing
else (wire protocol 1.1, see `Docs/PROTOCOL.md` in the nullCAT repo):

    NULLCATX,<rpm>,<speedKmh>,<gear>,<clutchPct>,<throttlePct>,
             <brakePct>,<absActive>,<skid>,<lockup>,<roadNoise>

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

By default it sends to `127.0.0.1:4444`. If nullCAT runs on another
machine (a Pi or NUC), create `NullcatChannelExporter.json` next to the
DLL:

```json
{ "host": "192.168.1.50", "port": 4444 }
```

Use the same port as your motion telemetry - nullCAT tells the two
streams apart by their headers.

## Optional: slip, lockup, and road channels

Brake and ABS come from SimHub's standard data automatically. The three
magnitude channels (skid, lockup, roadNoise: 0-100 each) vary by game, so
you bind them yourself to ANY SimHub property (including one you compute
with NCalc) in the same JSON:

```json
{ "host": "192.168.1.50", "port": 4444,
  "skidProp":   "SomePlugin.ComputedWheelSlip",
  "lockupProp": "",
  "roadProp":   "" }
```

Empty or missing = that channel sends 0 and its effect stays off. Values
are clamped to 0-100.

## nullCAT side

Bind the channels in your rig config (`ncxBindings`) - with this
plugin's channel order that is:

```json
"ncxBindings": [
  { "token": "rpm",         "slot": 0, "scale": 1.0, "offset": 0.0 },
  { "token": "speedKmh",    "slot": 1, "scale": 1.0, "offset": 0.0 },
  { "token": "gear",        "slot": 2, "scale": 1.0, "offset": 0.0 },
  { "token": "clutchPct",   "slot": 3, "scale": 1.0, "offset": 0.0 },
  { "token": "throttlePct", "slot": 4, "scale": 1.0, "offset": 0.0 },
  { "token": "brakePct",    "slot": 5, "scale": 1.0, "offset": 0.0 },
  { "token": "absActive",   "slot": 6, "scale": 1.0, "offset": 0.0 },
  { "token": "skid",        "slot": 7, "scale": 1.0, "offset": 0.0 },
  { "token": "lockup",      "slot": 8, "scale": 1.0, "offset": 0.0 },
  { "token": "roadNoise",   "slot": 9, "scale": 1.0, "offset": 0.0 }
]
```

See `Docs/DEVICES.md` in the nullCAT repository for what the effects do
and how to tune them.
