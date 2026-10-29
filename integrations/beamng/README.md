# nullCAT for BeamNG.drive

A small BeamNG.drive mod that sends the car's telemetry straight from the
game to nullCAT, for the haptics (shakers, the belt and the actuator
effects) and the belt from the car's g. Motion stays with your motion
software (SimHub, SimTools, FlyPT Mover, ...), which reads BeamNG's own
motion output as before.

Needs BeamNG.drive 0.32 or later and nullCAT 0.9.10 or later.

## Install

Every nullCAT release carries the mod ready to use:

- **nullCAT on Windows:** the `beamng` folder in the Windows zip. Its
  `nullcat_beamng.zip` sends to `127.0.0.1` (BeamNG and nullCAT on the
  same PC).
- **nullCAT on a Pi:** `nullCAT-v<version>-beamng-pi.zip` on the release
  page. The `nullcat_beamng.zip` inside sends to `192.168.50.1`, the Pi's
  telemetry address in nullCAT's Pi setup guide.

1. Open BeamNG's user folder: in the BeamNG.drive launcher, choose
   **Manage User Folder** and open it. It is the folder with `mods`,
   `settings` and `beamng.log` in it.
2. Copy `nullcat_beamng.zip` into its `mods` folder as it is (do not
   unpack it). Take any older nullCAT BeamNG zip out of there first: two
   copies of the mod fight.
3. Start BeamNG and check in the **Mod Manager** that the mod is active.
   BeamNG switches mods off after each of its updates: switch it back on
   there.
4. Drive. nullCAT's Haptics strip names BeamNG.drive and the car, and
   **direct telemetry** reads *(always on with this sender)*: the mod is
   a direct source and has no other, so the switch beside it does not
   change it.

## With SimHub (or other motion software)

Keep your motion software for motion. The nullCAT Channel Exporter
plugin can stay on: while the mod sends, nullCAT takes every haptic
channel from the mod and sets the plugin's BeamNG channels aside (the
Haptics strip shows *(always on with this sender, another sender set
aside)*). Motion is not
touched. While BeamNG sits in a menu or paused, the mod sends nothing, and
nullCAT keeps the plugin's channels aside for up to five minutes (past
that, the plugin's read plays until you drive again). When the mod stops
(BeamNG closed, the mod switched off after an update), nullCAT uses the
plugin's channels again once SimHub moves to another game, or after those
five minutes.

nullCAT listens on port 4444. With nullCAT and your motion software on
the same PC, only one of them can listen there: give your motion
software's BeamNG connection another port (for example 4445), in its own
setup and in BeamNG's protocol settings.

## What it sends

- The engine: rpm, the redline and the limiter, the gear, the pedals.
- ABS and TC while they work, on cars that have them (a car without them
  in BeamNG keeps those tiles quiet).
- The car's g along, across and up, how fast it turns, pitches and rolls,
  and its pitch and roll.
- Per tyre: its speed, its load, how fast it slides over the road, whether
  it touches the ground, the surface under it (tarmac, kerb, gravel,
  grass, dirt, sand, cobbles, snow, ice, mud, standing water) and whether
  it is wet, the suspension travel, the height of the road under it, and
  a puncture.
- The car's name, once a second, and with it what the car is: its
  engine (the cylinders, V, inline, boxer or rotary, its size, turbo or
  not, how fast the limiter cuts) and its type (road, track or cup, race,
  rally gravel or tarmac, from the config's name: rally, gravel, snow or
  dirt reads as rally gravel, tarmac or asphalt as rally tarmac). For a car
  nullCAT's car table has no entry for, the Engine and Limiter tiles take
  that engine and the type of car becomes the starting point, once per
  car; change either and it stays as you set it. Mod cars are described
  the same way.

That drives the Engine, Limiter, Gear shift, Driveline, ABS, TC, Long
slip, Lat slip, Road, Wheels, Surface and Rumble strip tiles, and the belt from
the car's g.

## The car's name

nullCAT sees the vehicle and its config, for example *scintilla race*;
that is the name profiles bind to. A car you changed in the parts or
tuning menus without saving it is a different car to nullCAT: it learns
that car's redline, gear ratios and tyres afresh. Save it as a config of
its own and it gets its own name too.

## Rate, address and port

The mod sends 100 times a second, read once per rendered frame, so never
more often than your frame rate. On a PC that struggles to hold its frame
rate, 60 is lighter; 200 reads on BeamNG's physics step and costs frame
rate.

To change the rate, the address or the port: unpack the zip into
`mods\unpacked\nullcat_beamng` (and take the zip out of `mods`), open
`lua\vehicle\protocols\nullcat_zerowerks.lua` in a text editor, change
the `ADDRESS`, `PORT` or `RATE` line near the top, save, and reload the
car in the game (Ctrl+R).

## When nothing arrives

- Is the mod active in BeamNG's Mod Manager (after an update)?
- Does it send to the right place: the Pi's address, or `127.0.0.1` with
  nullCAT on the same PC? BeamNG's log says where, when a car loads:
  `beamng.log` in the user folder, lines starting with `nullCAT:` (the
  game's console shows them too). Each car also writes a few hundred
  lines there about its engine, once. Only the car you sit in sends.
- On a Pi rig: does the game PC reach the Pi (the same network setup as
  SimHub's motion output)?

The mod is GPL-3.0-or-later, written for nullCAT.
