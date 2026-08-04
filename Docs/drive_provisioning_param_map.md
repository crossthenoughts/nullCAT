# nullCAT A6-EC / AS715N Drive Parameter Reference and Provisioning Map

Authoritative working reference for drive-side provisioning. Built by cross-checking three
sources: the live SOEM OD dump (`drive-reference/drive_dump_full.txt`, slave 1 "ANCTL AS715N"),
the ESI (`drive-reference/STEPPERONLINE_A6_Servo_V0.02.xml`, factory defaults + ranges), and
DriveFacts.md. **Re-verify against a live drive before relying on any single value.**

## AUTHORITATIVE provisioning delta: fresh drive vs configured (the real set)

Derived by dumping a brand-new out-of-box drive (firmware V512) and diffing against a
configured drive. **No ESI defaults involved**, so this supersedes the inflated
"changed vs ESI" list further down. **18 parameters** are deliberately changed from
factory; everything else (vibration suppression C02, I/O C04, overload 1150, etc.) is
**factory**: a new drive ships with it, so it is never written. (The encoder-tolerance
anomaly is the one exception -- see Key findings.)

| Panel | CoE | Name | Factory | Configured |
|---|---|---|---|---|
| C00.04 | `0x2000:0x05` | Gain auto-tuning mode | 1 | 0 |
| C00.05 | `0x2000:0x06` | Stiffness level selection | 12 | 10 |
| C01.01 | `0x2001:0x02` | 1st Speed loop gain | 250 | 150 |
| C01.02 | `0x2001:0x03` | 1st Speed loop integral time constant | 3184 | 5685 |
| C01.03 | `0x2001:0x04` | 1st Torque reference filter time constan | 200 | 112 |
| C01.08 | `0x2001:0x09` | 2nd Position loop gain | 560 | 400 |
| C01.09 | `0x2001:0x0a` | 2nd Speed loop gain | 350 | 250 |
| C01.0A | `0x2001:0x0b` | 2nd Speed loop integral time constant | 2274 | 3184 |
| C01.0B | `0x2001:0x0c` | 2nd Torque reference filter time constan | 280 | 200 |
| C01.10 | `0x2001:0x11` | Speed feedback filter selection | 0 | 1 |
| C01.22 | `0x2001:0x23` | First-order low-pass filter time constan | 0 | 50 |
| C01.41 | `0x2001:0x42` | Width level of the 1st notch | 0 | 100 |
| C01.42 | `0x2001:0x43` | Depth level of the 1st notch | 1000 | 10 |
| C01.44 | `0x2001:0x45` | Width level of the 2nd notch | 0 | 100 |
| C01.45 | `0x2001:0x46` | Depth level of the 2nd notch | 1000 | 10 |
| C13.02 | `0x2013:0x03` | Sync lost window | 8 | 20 |
| C13.05 | `0x2013:0x06` | Sync mode set | 1 | 2 |
| C13.06 | `0x2013:0x07` | Sync error window | 3000 | 6000 |

Plus (not visible in an OD dump; hidden until unlocked): **carrier `R22.38`=1** (16 kHz),
preceded by unlock **`C00.31`=1107**. The carrier write is panel-only (see Procedures).
Verify current carrier state by reading `0x2022:39` post-unlock.

**Per-axis (tune, do NOT blind-clone):** inertia `C00.06` stays at the factory 100 on ALL axes,
and notch **frequency** (currently factory 8000 = parked → measure each axis resonance).

**Dumps + ESI archived alongside this doc:** `drive-reference/`.

## Panel ↔ CoE address convention (tri-verified)
`C[nn].[xx]` (panel)  =  CoE `0x20[nn] : 0x(xx+1)`, **all hex** (sub 0x00 = "Number of
Entries", so sub 0x01 = panel `.00`). Confirmed anchors (notes + dump + ESI agree):
`C13.05`=`0x2013:06` (sync), `C13.01`=`0x2013:02` (alias), `C01.40`=`0x2001:41` (notch1 freq),
`C00.31`=`0x2000:32` (unlock), `R22.38`=`0x2022:39` (carrier), `U40.30`=`0x2040:31` (IGBT temp).
Other groups: `0x2040`→U40 (monitor, read-only), `0x2022`→R22 (hidden until unlocked).

## Procedures
- **Unlock (first, drive disabled):** `C00.31` = **1107** → `0x2000:32`. Gates hidden params (R22).
- **Carrier → 16 kHz:** after unlock, set `R22.38` = **1** on the panel (×2 doubler on the 8 kHz
  base). The object (`0x2022:39`) rejects SDO writes even after unlock, so the panel is the only
  write path; the provisioner deliberately skips it. ONLY 8/16 kHz are valid; an off-nominal
  carrier FAULTS (Er31.0 / Er50.1). Power-cycle to apply.
- **EtherCAT sync:** `C13.05` = **2** → `0x2013:06` (mandatory on Win+Npcap; cycle must be ×125 µs).
- **Store to NVRAM:** `0x1010:01` = **"save"** (`0x65766173`). Then power-cycle. Then READBACK-VERIFY.

## Notch filters
- **5 main** notches: `C01.40–4E` = `0x2001:41–4f` (freq/width/depth each).
- **2 position** notches: `C01.24–29` = `0x2001:25–2a` (separate feature).
- **Adaptive notch mode:** `C01.30` = `0x2001:31` (0=off/manual; 1/2 auto-use sets 1&2; sets 3–5 always manual).
- **Value semantics:** width 0 = filter does nothing (set ~100); depth 1000 = no attenuation, 10 = deepest.
- **Active "double 8 kHz" target:** notch1 & 2 = freq 7999, width 100, depth 10 (8000 parks the
  filter OFF; 7999 engages it). (Dump predates this:
  it shows freq 8000 / width 0 / depth 1000 = factory-OFF; do NOT clone the dump's notch verbatim.)
- Notch is **per-axis** (each unit's resonance differs): measure each drive.

## Key findings
- **The current gain tune is NOT per-drive auto-tune.** All 3 slaves are byte-identical (speed gain
  150, integral 5685, stiffness 10, torque filter 112, vib-supp 1000) and **inertia = 100 = factory
  default** -- the rig runs the factory inertia baseline (100) on every axis by design. Gains are
  auto-computed from fixed stiffness (10) + inertia 100 then frozen (`C00.04` 1→0, auto→manual).
  Uniform, so clone-safe. (The drive's inertia auto-ID, F30.10, is a motion-producing drive
  function and is not part of the nullCAT tune.)
- **Encoder fault tolerance anomaly:** the fresh dump confirms a new drive ships `C06.1C`
  (`0x2006:1d`) at the default **3** (ESI range 0-88). The reference drive carries an out-of-spec
  **900** -- a non-factory modification of unknown origin, possibly masking encoder-comms glitches.
  Do NOT clone it; leave new drives at 3.
- **Exclude from any clone (hardware identity, not settings):** `C20.00/01/09` motor model+rating
  (`0x2020:01/02/0a`), `C21.00` driver model (`0x2021:01`), `C13.01` station alias (`0x2013:02`).
  ESI "defaults" there are generic placeholders; the drive reports its real hardware.

## Recommended provisioning approach
1. **Clone (uniform, deliberate config):** unlock → carrier (`R22.38`=1, panel-only) → `C13.05=2` (+ sync windows
   C13.02/06) → mode (`0x6060`, app does this) → notch (7999/100/10).
2. **Per-axis (don't clone):** notch frequency to each axis's measured resonance. Inertia stays
   at the factory 100 baseline.
3. `0x1010` save → power-cycle → readback-verify.
4. CiA-402 runtime objects (`0x6060` mode, `0x6065` FE window, `0x6071` torque) are written by the
   app at init; they are NOT drive-NVRAM, keep them out of the provisioner.

## Full writable manufacturer-parameter table (0x2000–0x2021, from the dump)
Columns: Panel · CoE · Name · Type · Range(ESI) · Factory default(ESI) · Current(dump) · Changed-from-default
Note: a few "Changed=yes" rows are display-format artifacts (hex vs decimal / signed); verify before acting.

| Panel | CoE | Name | Type | Range | Factory | Current | Changed |
|---|---|---|---|---|---|---|---|
| C00.00 | `0x2000:0x01` | Control mode selection | UNSIGNED16 | 0..10 | 10 | 10 |  |
| C00.01 | `0x2000:0x02` | Forward direction | UNSIGNED16 | 0..1 | 0 | 0 |  |
| C00.04 | `0x2000:0x05` | Gain auto-tuning mode | UNSIGNED16 | 0..2 | 1 | 0 | **yes** |
| C00.05 | `0x2000:0x06` | Stiffness level selection | UNSIGNED16 | 1..31 | 12 | 10 | **yes** |
| C00.06 | `0x2000:0x07` | Load moment of inertia ratio | UNSIGNED16 | 0..12000 | 100 | 100 |  |
| C00.07 | `0x2000:0x08` | Absolute mode selection | UNSIGNED16 | 0..5 | 0 | 0 |  |
| C00.10 | `0x2000:0x11` | Regenerative resistor type | UNSIGNED16 | 0..3 | 0 | 0 |  |
| C00.11 | `0x2000:0x12` | Power capacity of regenerative resistor | UNSIGNED16 | 1..65535 | 50 | 50 |  |
| C00.12 | `0x2000:0x13` | Resistance of  regenerative resistor | UNSIGNED16 | 1..65535 | 40 | 50 | **yes** |
| C00.13 | `0x2000:0x14` | Resistor heat dissipation coefficient | UNSIGNED16 | 1..100 | 30 | 30 |  |
| C00.14 | `0x2000:0x15` | BrakeEn switch | UNSIGNED16 | 0..1 | 0 | 0 |  |
| C00.15 | `0x2000:0x16` | Motor rotation phase sequence setting | UNSIGNED16 | 0..1 | 0 | 0 |  |
| C00.16 | `0x2000:0x17` | Default keypad display function | UNSIGNED16 | 0..4 | 0 | 0 |  |
| C00.30 | `0x2000:0x31` | Ordinary users | UNSIGNED16 | 0..65535 | 0 | 0 |  |
| C00.31 | `0x2000:0x32` | Super user | UNSIGNED16 | 0..65535 | 0 | 0 |  |
| C00.40 | `0x2000:0x41` | Non standard switch | UNSIGNED16 | (n/a) | (n/a) | 0 |  |
| C01.00 | `0x2001:0x01` | 1st Position loop gain | UNSIGNED16 | 0..20000 | 400 | 400 |  |
| C01.01 | `0x2001:0x02` | 1st Speed loop gain | UNSIGNED16 | 1..20000 | 250 | 150 | **yes** |
| C01.02 | `0x2001:0x03` | 1st Speed loop integral time constant | UNSIGNED16 | 1..51200 | 3184 | 5685 | **yes** |
| C01.03 | `0x2001:0x04` | 1st Torque reference filter time constan | UNSIGNED16 | 5..16000 | 200 | 112 | **yes** |
| C01.08 | `0x2001:0x09` | 2nd Position loop gain | UNSIGNED16 | 0..20000 | 560 | 400 | **yes** |
| C01.09 | `0x2001:0x0a` | 2nd Speed loop gain | UNSIGNED16 | 1..20000 | 350 | 250 | **yes** |
| C01.0A | `0x2001:0x0b` | 2nd Speed loop integral time constant | UNSIGNED16 | 1..51200 | 2274 | 3184 | **yes** |
| C01.0B | `0x2001:0x0c` | 2nd Torque reference filter time constan | UNSIGNED16 | 5..16000 | 280 | 200 | **yes** |
| C01.10 | `0x2001:0x11` | Speed feedback filter selection | UNSIGNED16 | 0..4 | 0 | 1 | **yes** |
| C01.11 | `0x2001:0x12` | Cutoff frequency of speed feedback low-p | UNSIGNED16 | 10..16000 | 8000 | 8000 |  |
| C01.12 | `0x2001:0x13` | Speed feedback moving average filter tim | UNSIGNED16 | 0..6 | 0 | 0 |  |
| C01.13 | `0x2001:0x14` | Speed feedforward selection | UNSIGNED16 | 0..5 | 0 | 0 |  |
| C01.14 | `0x2001:0x15` | Speed  feedforward gain | UNSIGNED16 | 0..2000 | 0 | 0 |  |
| C01.15 | `0x2001:0x16` | Speed  feedforward filter time constant | UNSIGNED16 | 5..16000 | 318 | 318 |  |
| C01.16 | `0x2001:0x17` | Torque feedforward selection | UNSIGNED16 | 0..5 | 0 | 0 |  |
| C01.17 | `0x2001:0x18` | Torque feedforward gain | UNSIGNED16 | 0..2000 | 0 | 0 |  |
| C01.18 | `0x2001:0x19` | Torque  feedforward filter time constant | UNSIGNED16 | 5..16000 | 318 | 318 |  |
| C01.1B | `0x2001:0x1c` | PDFF control coefficient | UNSIGNED16 | 0..1000 | 1000 | 1000 |  |
| C01.1C | `0x2001:0x1d` | Damping factor | UNSIGNED16 | 0..1000 | 0 | 0 |  |
| C01.20 | `0x2001:0x21` | Moving average filter time constant A | UNSIGNED16 | 0..1280 | 0 | 0 |  |
| C01.21 | `0x2001:0x22` | Moving average filter time constant B | UNSIGNED16 | 0..1280 | 0 | 0 |  |
| C01.22 | `0x2001:0x23` | First-order low-pass filter time constan | UNSIGNED16 | 0..65535 | 0 | 50 | **yes** |
| C01.23 | `0x2001:0x24` | First-order low-pass filter time constan | UNSIGNED16 | 0..65535 | 0 | 0 |  |
| C01.24 | `0x2001:0x25` | Postion Frequency of the 1st notch | UNSIGNED16 | 0..2000 | 0 | 0 |  |
| C01.25 | `0x2001:0x26` | Postion Width level of the 1st notch | UNSIGNED16 | 0..1000 | 0 | 0 |  |
| C01.26 | `0x2001:0x27` | Postion Depth level of the 1st notch | UNSIGNED16 | 10..1000 | 1000 | 1000 |  |
| C01.27 | `0x2001:0x28` | Postion Frequency of the 2nd notch | UNSIGNED16 | 0..2000 | 0 | 0 |  |
| C01.28 | `0x2001:0x29` | Postion Width level of the 2nd notch | UNSIGNED16 | 0..1000 | 0 | 0 |  |
| C01.29 | `0x2001:0x2a` | Postion Depth level of the 2nd notch | UNSIGNED16 | 10..1000 | 1000 | 1000 |  |
| C01.2A | `0x2001:0x2b` | Postion Command buffer filter time | UNSIGNED16 | 0..1280 | 0 | 0 |  |
| C01.30 | `0x2001:0x31` | Adaptive notch mode | UNSIGNED16 | 0..3 | 0 | 0 |  |
| C01.31 | `0x2001:0x32` | Adaptive notch detection times | UNSIGNED16 | 0..65535 | 0 | 0 |  |
| C01.38 | `0x2001:0x39` | Gain switchover condition | UNSIGNED16 | 0..8 | 0 | 0 |  |
| C01.39 | `0x2001:0x3a` | Gain switchover delay | UNSIGNED16 | 10..10000 | 50 | 50 |  |
| C01.3A | `0x2001:0x3b` | Gain switchover threshold | UNSIGNED16 | 0..65535 | 10 | 10 |  |
| C01.3B | `0x2001:0x3c` | Gain switching loop width | UNSIGNED16 | 0..65535 | 10 | 10 |  |
| C01.40 | `0x2001:0x41` | Frequency of the 1st notch | UNSIGNED16 | 10..8000 | 8000 | 8000 |  |
| C01.41 | `0x2001:0x42` | Width level of the 1st notch | UNSIGNED16 | 0..4000 | 0 | 0 |  |
| C01.42 | `0x2001:0x43` | Depth level of the 1st notch | UNSIGNED16 | 10..1000 | 1000 | 1000 |  |
| C01.43 | `0x2001:0x44` | Frequency of the 2nd notch | UNSIGNED16 | 10..8000 | 8000 | 8000 |  |
| C01.44 | `0x2001:0x45` | Width level of the 2nd notch | UNSIGNED16 | 0..4000 | 0 | 0 |  |
| C01.45 | `0x2001:0x46` | Depth level of the 2nd notch | UNSIGNED16 | 10..1000 | 1000 | 1000 |  |
| C01.46 | `0x2001:0x47` | Frequency of the 3rd notch | UNSIGNED16 | 10..8000 | 8000 | 8000 |  |
| C01.47 | `0x2001:0x48` | Width level of the 3rd notch | UNSIGNED16 | 0..4000 | 0 | 0 |  |
| C01.48 | `0x2001:0x49` | Depth level of the 3rd notch | UNSIGNED16 | 10..1000 | 1000 | 1000 |  |
| C01.49 | `0x2001:0x4a` | Frequency of the 4th notch | UNSIGNED16 | 10..8000 | 8000 | 8000 |  |
| C01.4A | `0x2001:0x4b` | Width level of the 4th notch | UNSIGNED16 | 0..4000 | 0 | 0 |  |
| C01.4B | `0x2001:0x4c` | Depth level of the 4th notch | UNSIGNED16 | 10..1000 | 1000 | 1000 |  |
| C01.4C | `0x2001:0x4d` | Frequency of the 5th notch | UNSIGNED16 | 10..8000 | 8000 | 8000 |  |
| C01.4D | `0x2001:0x4e` | Width level of the 5th notch | UNSIGNED16 | 0..4000 | 0 | 0 |  |
| C01.4E | `0x2001:0x4f` | Depth level of the 5th notch | UNSIGNED16 | 10..1000 | 1000 | 1000 |  |
| C02.00 | `0x2002:0x01` | Model following control selection | UNSIGNED16 | 10..20000 | 500 | 0 | **yes** |
| C02.01 | `0x2002:0x02` | Model following control gain | UNSIGNED16 | 10..20000 | 500 | 500 |  |
| C02.02 | `0x2002:0x03` | Inertia correction of MFC | UNSIGNED16 | 10..8000 | 1000 | 1000 |  |
| C02.30 | `0x2002:0x31` | Gain ratio of speed observer | UNSIGNED16 | 0..40000 | 0 | 0 |  |
| C02.31 | `0x2002:0x32` | Inertia correction of speed observer | UNSIGNED16 | 10..8000 | 1000 | 1000 |  |
| C02.32 | `0x2002:0x33` | Speed observer speed feedback cutoff fre | UNSIGNED16 | 0..16000 | 0 | 0 |  |
| C02.38 | `0x2002:0x39` | Vibration suppression frequency 1 | UNSIGNED16 | 0..20000 | 0 | 1000 | **yes** |
| C02.39 | `0x2002:0x3a` | Inertia correction of vibration suppress | UNSIGNED16 | 10..8000 | 1000 | 1000 |  |
| C02.3A | `0x2002:0x3b` | Correction of low pass filter for vibrat | INTEGER16 | -9999..9999 | 0 | 0 |  |
| C02.3B | `0x2002:0x3c` | Vibration suppression high pass filter 1 | INTEGER16 | 0..20000 | 0 | 0 |  |
| C02.3C | `0x2002:0x3d` | Vibration suppression high pass filter 2 | UNSIGNED16 | 0..20000 | 0 | 20000 | **yes** |
| C02.3D | `0x2002:0x3e` | Vibration suppression compensation 1 rat | UNSIGNED16 | 0..20000 | 0 | 0 |  |
| C02.3E | `0x2002:0x3f` | Vibration suppression compensation 2 rat | UNSIGNED16 | 0..20000 | 0 | 0 |  |
| C02.40 | `0x2002:0x41` | Vibration suppression frequency 2 | UNSIGNED16 | 0..20000 | 0 | 1000 | **yes** |
| C02.41 | `0x2002:0x42` | Inertia correction of vibration suppress | UNSIGNED16 | 10..8000 | 1000 | 1000 |  |
| C02.42 | `0x2002:0x43` | Correction of low pass filter for vibrat | INTEGER16 | -9999..9999 | 0 | 0 |  |
| C02.43 | `0x2002:0x44` | Vibration suppression high pass filter 1 | INTEGER16 | 0..20000 | 0 | 0 |  |
| C02.44 | `0x2002:0x45` | Vibration suppression high pass filter 2 | UNSIGNED16 | 0..20000 | 0 | 20000 | **yes** |
| C02.45 | `0x2002:0x46` | Vibration suppression compensation 1 rat | UNSIGNED16 | 0..20000 | 0 | 0 |  |
| C02.46 | `0x2002:0x47` | Vibration suppression compensation 2 rat | UNSIGNED16 | 0..20000 | 0 | 0 |  |
| C02.48 | `0x2002:0x49` | Vibration suppression frequency 3 | UNSIGNED16 | 0..20000 | 0 | 1000 | **yes** |
| C02.49 | `0x2002:0x4a` | Inertia correction of vibration suppress | UNSIGNED16 | 10..8000 | 1000 | 1000 |  |
| C02.4A | `0x2002:0x4b` | Correction of low pass filter for vibrat | INTEGER16 | -9999..9999 | 0 | 0 |  |
| C02.4B | `0x2002:0x4c` | Vibration suppression high pass filter 1 | INTEGER16 | 0..20000 | 0 | 0 |  |
| C02.4C | `0x2002:0x4d` | Vibration suppression high pass filter 2 | UNSIGNED16 | 0..20000 | 0 | 20000 | **yes** |
| C02.4D | `0x2002:0x4e` | Vibration suppression compensation 1 rat | UNSIGNED16 | 0..20000 | 0 | 0 |  |
| C02.4E | `0x2002:0x4f` | Vibration suppression compensation 2 rat | UNSIGNED16 | 0..20000 | 0 | 0 |  |
| C02.60 | `0x2002:0x61` | Gain correction coefficient of disturban | UNSIGNED16 | 0..40000 | 0 | 0 |  |
| C02.61 | `0x2002:0x62` | Inertia correction coefficient of distur | UNSIGNED16 | 1..10000 | 1000 | 1000 |  |
| C02.62 | `0x2002:0x63` | Low pass cutoff frequency correction of | UNSIGNED16 | 0..16000 | 0 | 0 |  |
| C02.63 | `0x2002:0x64` | Disturbance observer torque compensation | UNSIGNED16 | 0..2000 | 0 | 0 |  |
| C02.68 | `0x2002:0x69` | Friction compensation switch and related | UNSIGNED16 | 0..1000 | 10 | 0 | **yes** |
| C02.69 | `0x2002:0x6a` | Friction compensation speed threshold | UNSIGNED16 | 0..1000 | 10 | 20 | **yes** |
| C02.6A | `0x2002:0x6b` | Static friction compensation value | UNSIGNED16 | 0..2000 | 0 | 0 |  |
| C02.6B | `0x2002:0x6c` | Forward compensation value of Coulomb fr | UNSIGNED16 | 0..2000 | 0 | 0 |  |
| C02.6C | `0x2002:0x6d` | Coulomb friction reverse compensation va | INTEGER16 | -2000..0 | 0 | 0 |  |
| C02.6D | `0x2002:0x6e` | Viscous friction torque corresponding to | UNSIGNED16 | 0..2000 | 0 | 0 |  |
| C02.6E | `0x2002:0x6f` | Friction compensation filtering time | UNSIGNED16 | 0..1000 | 10 | 0 | **yes** |
| C02.6F | `0x2002:0x70` | Friction compensation zero speed thresho | UNSIGNED16 | 0..1000 | 10 | 10 |  |
| C03.21 | `0x2003:0x22` | Speed reference set | INTEGER16 | -8000..8000 | 100 | 100 |  |
| C03.22 | `0x2003:0x23` | Acceleration ramp time constant of speed | UNSIGNED32 | 00..#x36EE80 | 010 | 10 | **yes** |
| C03.24 | `0x2003:0x25` | Deceleration ramp time constant of speed | UNSIGNED32 | 00..#x36EE80 | 010 | 10 | **yes** |
| C03.27 | `0x2003:0x28` | Forward internal Speed limit | UNSIGNED16 | 0..8000 | 6000 | 6000 |  |
| C03.28 | `0x2003:0x29` | Reverse internal Speed limit | UNSIGNED16 | 0..8000 | 6000 | 6000 |  |
| C03.2B | `0x2003:0x2c` | Arrive Speed threshold | UNSIGNED16 | 0..8000 | 1000 | 1000 |  |
| C03.2C | `0x2003:0x2d` | Sync Speed threshold | UNSIGNED16 | 0..1000 | 10 | 10 |  |
| C03.2D | `0x2003:0x2e` | Motor speed threshold | UNSIGNED16 | 0..1000 | 20 | 20 |  |
| C03.2E | `0x2003:0x2f` | Zero speed threshold | UNSIGNED16 | 0..1000 | 10 | 10 |  |
| C03.41 | `0x2003:0x42` | Torque reference set | INTEGER16 | -4000..4000 | 0 | 0 |  |
| C03.43 | `0x2003:0x44` | Forward internal torque limit | UNSIGNED16 | 0..4000 | 3000 | 3000 |  |
| C03.44 | `0x2003:0x45` | Reverse internal torque limit | UNSIGNED16 | 0..4000 | 3000 | 3000 |  |
| C03.47 | `0x2003:0x48` | Forward Speed limit 1 in torque control | UNSIGNED16 | 0..8000 | 3000 | 3000 |  |
| C03.48 | `0x2003:0x49` | Reverse Speed limit 2 in torque control | UNSIGNED16 | 0..8000 | 3000 | 3000 |  |
| C03.49 | `0x2003:0x4a` | Reference value for torque reached | UNSIGNED16 | 0..4000 | 0 | 0 |  |
| C03.4A | `0x2003:0x4b` | Valid value for torque reached | UNSIGNED16 | 0..4000 | 200 | 200 |  |
| C03.4B | `0x2003:0x4c` | Invalid value for torque reached | UNSIGNED16 | 0..4000 | 100 | 100 |  |
| C04.00 | `0x2004:0x01` | DI1 function selection | UNSIGNED16 | 0..32 | 6 | 6 |  |
| C04.01 | `0x2004:0x02` | DI1 logic selection | UNSIGNED16 | 0..1 | 0 | 0 |  |
| C04.02 | `0x2004:0x03` | DI1 input filtering time constant | UNSIGNED16 | 0..65535 | 0 | 150 | **yes** |
| C04.04 | `0x2004:0x05` | DI2 function selection | UNSIGNED16 | 0..32 | 7 | 7 |  |
| C04.05 | `0x2004:0x06` | DI2 logic selection | UNSIGNED16 | 0..1 | 0 | 0 |  |
| C04.06 | `0x2004:0x07` | DI2 input filtering time constant | UNSIGNED16 | 0..65535 | 0 | 150 | **yes** |
| C04.08 | `0x2004:0x09` | DI3 function selection | UNSIGNED16 | 0..32 | 5 | 5 |  |
| C04.09 | `0x2004:0x0a` | DI3 logic selection | UNSIGNED16 | 0..1 | 0 | 0 |  |
| C04.0A | `0x2004:0x0b` | DI3 input filtering time constant | UNSIGNED16 | 0..65535 | 0 | 150 | **yes** |
| C04.0C | `0x2004:0x0d` | DI4 function selection | UNSIGNED16 | 0..32 | 2 | 31 | **yes** |
| C04.0D | `0x2004:0x0e` | DI4 logic selection | UNSIGNED16 | 0..1 | 0 | 0 |  |
| C04.0E | `0x2004:0x0f` | DI4 input filtering time constant | UNSIGNED16 | 0..65535 | 0 | 150 | **yes** |
| C04.10 | `0x2004:0x11` | DI5 function selection | UNSIGNED16 | 0..32 | 1 | 30 | **yes** |
| C04.11 | `0x2004:0x12` | DI5 logic selection | UNSIGNED16 | 0..1 | 0 | 0 |  |
| C04.12 | `0x2004:0x13` | DI5 input filtering time constant | UNSIGNED16 | 0..65535 | 0 | 150 | **yes** |
| C04.30 | `0x2004:0x31` | DO1 function selection | UNSIGNED16 | 0..20 | 1 | 1 |  |
| C04.31 | `0x2004:0x32` | DO1 logic level selection | UNSIGNED16 | 0..1 | 0 | 0 |  |
| C04.32 | `0x2004:0x33` | DO2 function selection | UNSIGNED16 | 0..20 | 4 | 4 |  |
| C04.33 | `0x2004:0x34` | DO2 logic level selection | UNSIGNED16 | 0..1 | 0 | 0 |  |
| C04.34 | `0x2004:0x35` | DO3 function selection | UNSIGNED16 | 0..20 | 3 | 3 |  |
| C04.35 | `0x2004:0x36` | DO3 logic level selection | UNSIGNED16 | 0..1 | 0 | 0 |  |
| C05.02 | `0x2005:0x03` | Stop mode at overtravel | UNSIGNED16 | 0..7 | 1 | 1 |  |
| C05.03 | `0x2005:0x04` | Stop mode at No. 1 fault | UNSIGNED16 | 0..2 | 2 | 2 |  |
| C05.07 | `0x2005:0x08` | stop speed threshold | UNSIGNED16 | 0..1000 | 10 | 10 |  |
| C05.0C | `0x2005:0x0d` | Torque stop limit | UNSIGNED16 | 0..3000 | 1000 | 1000 |  |
| C05.10 | `0x2005:0x11` | Delay from motor de-energized to brake o | UNSIGNED16 | 0..65535 | 100 | 100 |  |
| C05.11 | `0x2005:0x12` | Motor speed threshold at brake output OF | UNSIGNED16 | 10..3000 | 30 | 30 |  |
| C05.12 | `0x2005:0x13` | Delay from brake output OFF to motor de- | UNSIGNED16 | 0..65535 | 100 | 100 |  |
| C05.13 | `0x2005:0x14` | Delay from brake output ON to command re | UNSIGNED16 | 0..65535 | 100 | 100 |  |
| C05.14 | `0x2005:0x15` | Power on delay time of DB relay | UNSIGNED16 | 0..65535 | 20 | 20 |  |
| C06.00 | `0x2006:0x01` | Excessive position deviation threshold | UNSIGNED32 | 00..#xFFFFFFFF | 032767 | 32767 | **yes** |
| C06.03 | `0x2006:0x04` | Overspeed threshold | UNSIGNED16 | 0..9000 | 0 | 0 |  |
| C06.04 | `0x2006:0x05` | Power inputphase loss protection disable | UNSIGNED16 | 0..1 | 0 | 0 |  |
| C06.05 | `0x2006:0x06` | Power-off memory selection | UNSIGNED16 | 0..1 | 0 | 0 |  |
| C06.07 | `0x2006:0x08` | Mechanical limit selection | UNSIGNED16 | 0..2 | 0 | 0 |  |
| C06.08 | `0x2006:0x09` | Mechanical forward limit position | INTEGER32 | #x80000000..#x7FFFFFFF | #x7FFFFFFF | 2147483647 | **yes** |
| C06.0A | `0x2006:0x0b` | Mechanical negative limit position | INTEGER32 | #x80000000..#x7FFFFFFF | #x80000000 | -2147483648 | **yes** |
| C06.10 | `0x2006:0x11` | Driver overload protection threshold | UNSIGNED16 | 0..3500 | 1150 | 1150 |  |
| C06.11 | `0x2006:0x12` | Motor overload protection gain | UNSIGNED16 | 0..3500 | 1150 | 1150 |  |
| C06.12 | `0x2006:0x13` | Locked rotor over-temperature protection | UNSIGNED16 | 0..1 | 1 | 1 |  |
| C06.13 | `0x2006:0x14` | Motor locked rotor detection time | UNSIGNED16 | 0..3000 | 200 | 200 |  |
| C06.14 | `0x2006:0x15` | Motor locked rotor speed detection | UNSIGNED16 | 0..1000 | 10 | 10 |  |
| C06.15 | `0x2006:0x16` | Output phase loss detection is enabled | UNSIGNED16 | 0..1 | 0 | 0 |  |
| C06.1C | `0x2006:0x1d` | Encoder communication fault tolerance th | UNSIGNED16 | 0..88 | 3 | 900 | **yes** |
| C06.20 | `0x2006:0x21` | Runaway protection selection | UNSIGNED16 | 0..1 | 1 | 1 | **torque axes: 0** (profile `torqueOnly[]`; a back-driven CST belt trips Er06.0 runaway protection otherwise, manual p181; the panel write is the hardware-proven path, the provisioner's SDO write is unverified on hardware) |
| C10.00 | `0x2010:0x01` | Homing Enable | UNSIGNED16 | 0..3 | 0 | 0 |  |
| C10.08 | `0x2010:0x09` | Homing duration limit | UNSIGNED32 | 0..65535 | 60000 | 60000 |  |
| C10.10 | `0x2010:0x11` | Multiturn absolute position bias 32 bits | INTEGER32 | 0..#xFFFF | 0 | 0 |  |
| C10.12 | `0x2010:0x13` | Multiturn absolute position bias 32 bits | INTEGER32 | 0..#xFFFF | 0 | 0 |  |
| C10.16 | `0x2010:0x17` | Rotation mode Indicates the operation mo | UNSIGNED16 | 0..4 | 0 | 0 |  |
| C10.18 | `0x2010:0x19` | Rotational mode mechanical gear ratio mo | UNSIGNED16 | 1..65535 | 1 | 1 |  |
| C10.19 | `0x2010:0x1a` | Rotary mode mechanical gear score female | UNSIGNED16 | 1..65535 | 1 | 1 |  |
| C10.1A | `0x2010:0x1b` | Rotation mode mechanical absolute positi | UNSIGNED32 | 0..65535 | 0 | 0 |  |
| C10.1C | `0x2010:0x1d` | Rotation mode mechanical absolute positi | UNSIGNED32 | 0..65535 | 0 | 0 |  |
| C10.1E | `0x2010:0x1f` | single turn encoder homing offset | INTEGER32 | 0..#xFFFF | 0 | 0 |  |
| C10.30 | `0x2010:0x31` | crash homing torque | UNSIGNED16 | 0..3000 | 1000 | 1000 |  |
| C10.31 | `0x2010:0x32` | crash homing speed | UNSIGNED16 | 0..1000 | 10 | 10 |  |
| C10.32 | `0x2010:0x33` | crash homing times | UNSIGNED16 | 0..65535 | 30 | 30 |  |
| C13.01 | `0x2013:0x02` | station alias | UNSIGNED16 | 0..65535 | 0 | 0 |  |
| C13.02 | `0x2013:0x03` | Sync lost window | UNSIGNED16 | 1..20 | 8 | 20 | **yes** |
| C13.05 | `0x2013:0x06` | Sync mode set | UNSIGNED16 | 0..2 | 1 | 2 | **yes** |
| C13.06 | `0x2013:0x07` | Sync error window | UNSIGNED16 | 0..6000 | 3000 | 6000 | **yes** |
| C13.07 | `0x2013:0x08` | CSP postion increment over counter | UNSIGNED16 | 1..30 | 5 | 5 |  |
| C13.08 | `0x2013:0x09` | Enhanced link detection enable | UNSIGNED16 | 0..1 | 0 | 0 |  |
| C13.10 | `0x2013:0x11` | Update function code values written via | UNSIGNED16 | 0..1 | 1 | 1 |  |
| C13.11 | `0x2013:0x12` | Irq lost window | UNSIGNED16 | 0..10 | 5 | 5 |  |
| C20.00 | `0x2020:0x01` | Motor setting model | UNSIGNED16 | 0..65535 | 20000 | 10000 | **yes** |
| C20.01 | `0x2020:0x02` | Internal motor model | UNSIGNED16 | 0..65535 | 20063 | 10065 | **yes** |
| C20.08 | `0x2020:0x09` | Motor rated voltage | UNSIGNED16 | 0..2 | 0 | 0 |  |
| C20.09 | `0x2020:0x0a` | Motor rating | UNSIGNED32 | 01..#xFFFFFFFF | 040 | 75 | **yes** |
| C21.00 | `0x2021:0x01` | Driver Setting Model | UNSIGNED16 | 0..65535 | 3 | 5 | **yes** |
