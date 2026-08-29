# nullCAT Channel Exporter (SimHub plugin)

Sends raw sim telemetry to nullCAT for the force-device effects (shifter,
active pedal). One UDP line per tick, nothing else:

    NULLCATX,<rpm>,<speedKmh>,<gear>,<clutchPct>,<throttlePct>

All the feel and logic lives in nullCAT - this plugin never changes when
effects do. Gear is numeric on the wire: `0` = neutral, `-1` = reverse.

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

## nullCAT side

Bind the channels in your rig config (`ncxBindings`) - with this
plugin's channel order that is:

```json
"ncxBindings": [
  { "token": "rpm",         "slot": 0, "scale": 1.0, "offset": 0.0 },
  { "token": "speedKmh",    "slot": 1, "scale": 1.0, "offset": 0.0 },
  { "token": "gear",        "slot": 2, "scale": 1.0, "offset": 0.0 },
  { "token": "clutchPct",   "slot": 3, "scale": 1.0, "offset": 0.0 },
  { "token": "throttlePct", "slot": 4, "scale": 1.0, "offset": 0.0 }
]
```

See `Docs/DEVICES.md` in the nullCAT repository for what the effects do
and how to tune them.
