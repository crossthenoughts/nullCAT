# Build Instructions

## Overview

One shared source tree (`src/`), two builds:

- **Linux / Raspberry Pi (headless)**: `pi/CMakeLists.txt` builds the `nullcat-pi`
  daemon. No GUI; the web UI is the control surface.
- **Windows (Qt GUI)**: the top-level `CMakeLists.txt` builds `nullCAT.exe`,
  a Qt desktop app with an optional web UI.

The two CMake trees are deliberately isolated; building one never touches the other.
Both use **SOEM** (Simple Open EtherCAT Master) for EtherCAT and **CMake 3.20+**.
The app drives up to 10 StepperOnline A6-EC servo drives, receiving motion data
from motion software (e.g. SimHub) over UDP.

---

# Linux / Raspberry Pi build

## Dependencies

```bash
sudo apt install cmake g++ qt6-base-dev libgpiod-dev
```

- **Qt 6 Core** is a transitional dependency: only the JSON config loader uses it
  (Core only, no Widgets or Network).
- **libgpiod (v2)** is optional. It enables the GPIO control panel; without it, or
  with a v1 libgpiod, the build proceeds and the panel is disabled with a warning.

## SOEM (pinned tree)

SOEM headers and library must come from the **same tree**. The generated
`ec_options.h` sizes the context struct, so mixing headers from one tree with a
library from another silently changes the struct layout (ABI mismatch). One pinned
tree rules that out.

```bash
git clone https://github.com/OpenEtherCATsociety/SOEM.git ~/SOEM
cmake -S ~/SOEM -B ~/SOEM/build
cmake --build ~/SOEM/build -j2
```

The Pi build looks for the tree at `~/SOEM` by default (override with
`-DSOEM_ROOT=`) and links `libsoem.a` straight from `<SOEM_ROOT>/build`.

## Build

```bash
cmake -S pi -B pi/build
cmake --build pi/build -j2
```

Keep `-j2` on a Pi: 2 schedulable cores and 2 GB RAM. Higher parallelism can
swap-thrash the box hard enough to drop it off the network.

Outputs in `pi/build/`: `nullcat-pi` (the daemon), `provision` (drive
provisioning tool), and the test binaries. The `web/`, `logs/`, and
`drive_profiles/` directories are staged next to the binary.

## Run

`nullcat-pi` needs raw-socket access for SOEM: either run as root or grant the
binary `CAP_NET_RAW`:

```bash
sudo setcap cap_net_raw+ep pi/build/nullcat-pi
```

Optional: `deploy/nullcat.service` is an Avahi service file that advertises the
web UI as `http://nullcat.local:8080` (install instructions in the file).

## Tests

```bash
ctest --test-dir pi/build
```

The default build includes the core suites (homing, status model, torque path,
telemetry parsing, WKC monitor, command contract, HTTP contract). The CMake option
`NULLCAT_FULL_TESTS` (default `OFF`) additionally builds the extended logic
suites; it requires Qt6 Test:

```bash
cmake -S pi -B pi/build -DNULLCAT_FULL_TESTS=ON
```

Default OFF keeps local builds fast; CI configures with it ON.

---

# Windows build

## Prerequisites

| # | Prerequisite |
|---|---|
| 1 | Visual Studio 2022 (C++ Desktop workload) |
| 2 | CMake 3.20+ |
| 3 | Qt 6.x (Qt 5.15+ also supported) |
| 4 | SOEM (built from source, below) |
| 5 | Npcap runtime **and** Npcap SDK (for SOEM raw Ethernet on Windows) |

## Step 1: Install Visual Studio 2022

1. Download from https://visualstudio.microsoft.com/
2. In the installer, select the workload **Desktop development with C++**
3. Under Individual Components, ensure these are selected:
   - MSVC v143 (or later) C++ x64/x86 build tools
   - Windows 11 SDK (or Windows 10 SDK)

## Step 2: Install CMake

1. Download the Windows x64 installer from https://cmake.org/download/
2. During install, select **"Add CMake to the system PATH"**
3. Verify: open a new Command Prompt and type `cmake --version`

## Step 3: Install Qt

1. Download the Qt Online Installer from https://www.qt.io/download-qt-installer
2. Select **Qt 6.x** with the `MSVC 2019 64-bit` component (works with VS 2022)
3. Note the install path; you will need it for CMake. Typical:
   `C:\Qt\6.6.0\msvc2019_64`

## Step 4: Install Npcap + the Npcap SDK (required for SOEM)

SOEM on Windows uses raw Ethernet sockets via Npcap. You need **two
separate downloads** from https://npcap.com/#download:

1. **The Npcap installer** (the runtime driver — needed to *run* nullCAT).
   During install, check **"Install Npcap in WinPcap API-compatible Mode"**
   (required for SOEM). Reboot if prompted.
2. **The Npcap SDK zip, version 1.16 or newer** (needed to *build*). Extract
   it anywhere, e.g. `C:\libs\npcap-sdk`, so that `Include\pcap.h` and
   `Lib\x64\wpcap.lib` exist under it. SOEM's `nicdrv.h` includes
   `<pcap.h>`, so the SDK is a compile-time requirement, not just a
   link-time one.

   > **Use 1.16+.** SOEM's `nicdrv.h` includes both `<pcap.h>` and
   > `<Packet32.h>`. In SDK 1.13 and older, `Packet32.h` re-defines
   > `struct bpf_program` and `struct bpf_insn`, which modern `pcap.h`
   > already defines — the build fails with
   > `error C2011: 'bpf_program': 'struct' type redefinition`.

You pass the SDK path to CMake as `-DNPCAP_SDK` in Step 6 (it defaults to
`C:/libs/npcap-sdk`, so extracting there means you can omit the flag).

## Step 5: Build SOEM

Open a **x64 Native Tools Command Prompt for VS 2022** (Start Menu):

```cmd
git clone https://github.com/OpenEtherCATsociety/SOEM.git C:\libs\SOEM-src
cd C:\libs\SOEM-src
cmake -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
cmake --install build --prefix C:\libs\SOEM --config Release
```

Expected layout after install:

```
C:\libs\SOEM\
  include\
    soem\
      soem.h          <- main SOEM header
      ec_type.h
      ... (other headers)
  lib\
    soem.lib          <- link this
```

The sources include SOEM headers as `soem/...`, so the headers must sit in an
`include\soem\` subdirectory as above.

## Step 6: Configure and build this project

From the same VS 2022 x64 Native Tools prompt:

```cmd
cd C:\path\to\nullcat
cmake -B build -G "Visual Studio 17 2022" -A x64 ^
  -DQt6_DIR="C:/Qt/6.6.0/msvc2019_64/lib/cmake/Qt6" ^
  -DSOEM_ROOT="C:/libs/SOEM" ^
  -DNPCAP_SDK="C:/libs/npcap-sdk"
```

Replace paths with your actual installation paths. For Qt 5, pass
`-DQt5_DIR="C:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5"` instead of `Qt6_DIR`.
`-DNPCAP_SDK` can be omitted if you extracted the SDK to the default
`C:/libs/npcap-sdk`. If either dependency is missing, CMake stops at
configure time with a message naming the expected files.

Build from the command line:

```cmd
cmake --build build --config Release
```

Or open `build\nullCAT.sln` in Visual Studio, set the configuration to
**Release / x64**, and Build Solution. The exe lands in
`build\Release\nullCAT.exe`.

## Step 7: Deploy Qt DLLs

Qt applications need Qt DLLs alongside the exe:

```cmd
cd build\Release
C:\Qt\6.6.0\msvc2019_64\bin\windeployqt.exe nullCAT.exe
```

## Step 8: Run as Administrator

SOEM raw socket access requires Administrator rights. The build embeds a
manifest that requests elevation automatically; if that does not take effect in
your environment, right-click the exe and choose **Run as administrator**.

---

# Configuration (both platforms)

The app reads two JSON files next to the binary:

- **`host.json`**: per-machine settings (NIC name, ports, bind addresses, web
  UI, RT tuning).
- **`rig.json`**: the portable rig definition (axes, drives, motion tuning).

On first launch the app cold-starts the pair with defaults, so the directory
always ends up with the two-file set (a legacy flat `config.json` is ignored).
`resources/host.reference.json` and `resources/rig.reference.json` list every
field at its default; full descriptions and units are in
`Docs/CONFIG_REFERENCE.md`.

Key `host.json` fields to check first:

```json
{
    "nicName": "Ethernet 2",
    "controlLoopHz": 500,
    "telemetryPort": 4444
}
```

## Finding your NIC name

`nicName` must match how the OS sees your EtherCAT NIC.

- **Adapter list in the log (both platforms):** the app logs all found adapters
  at startup. Run it once with any NIC name and read the list from the log.
- **Windows:** `ipconfig /all`, use the adapter's **Description** field (for
  example `"Intel(R) I210 Gigabit Network Connection"`), or check Device
  Manager under Network Adapters.
- **Linux:** `ip link` (for example `"eth0"`).

---

# Telemetry configuration

Any motion software that can send a custom UDP string works (SimHub,
SimTools, FlyPT Mover, ...). The SimHub click-path is walked step by step
in `Docs/FIRST_SETUP_WINDOWS.md`; the generic contract:

- **Host:** `127.0.0.1` for the same-PC case. For a dedicated controller box,
  the controller's IP; see `Docs/NUC_DEPLOYMENT.md`.
- **Port:** `4444` (must match `telemetryPort` in `host.json`)
- **Packet format:** CSV text, one packet per datagram

The packet format this application expects (full contract in
`Docs/CONFIG_REFERENCE.md`, "Telemetry wire format"):

```
NULLCAT,<pos0>,<pos1>,...,<posN>
```

Every field after the `NULLCAT` header is an axis value in decimal
(16-bit unsigned, center 32767, is the tested scaling).

---

# First run workflow

1. Connect the EtherCAT cable from the controller to the first drive,
   daisy-chain the remaining drives
2. Power on all drives
3. Start the application (Administrator on Windows; `CAP_NET_RAW` or root on
   Linux)
4. Edit `host.json` if needed (NIC name), and define axes in `rig.json` or via
   the web UI
5. Initialize EtherCAT (desktop button, or `POST /api/init` from the web UI).
   The app discovers slaves and transitions them to OP
6. Start your motion software with the UDP output configured as above
7. Start the control loop (desktop button, or `POST /api/start`). The RT
   control thread starts, drives are enabled via the DS402 state machine, and
   telemetry targets are commanded to each drive

---

# Drive PDO assumptions

The app uses the A6-EC's **fixed manufacturer PDO sets**; it never writes custom
PDO mappings. Position axes run the drive-default set (RxPDO `1701h`, TxPDO
`1B01h`). Torque (belt) axes are switched during PREOP to RxPDO `1702h` / TxPDO
`1B02h`; `1702h` carries target torque (0x6071), mode of operation (0x6060),
and max profile velocity (0x607F), which the app writes every cycle. Details,
including byte layouts, are in `Docs/DriveFacts.md`; drive-side parameters are
covered by `Docs/drive_provisioning_param_map.md` and `Docs/DRIVE_TUNING.md`.

If your drive reports a different PDO size the app logs it and falls back to
the position layout. Use the vendor manual to inspect the object dictionary.

---

# Troubleshooting

### "No EtherCAT slaves found"
- Check cable connections
- Verify NIC name in `host.json`
- Windows: ensure the app runs as Administrator and Npcap is installed in
  WinPcap-compatible mode. Linux: check `CAP_NET_RAW`/root
- Try disconnecting and reconnecting the Ethernet cable
- Some NICs require special drivers; Intel I210/I211 are well-tested with SOEM

### "ec_init() failed"
- Wrong NIC name in `host.json`
- Missing privileges (Administrator / `CAP_NET_RAW`)
- Windows: Npcap not installed, or firewall blocking raw socket access

### Drives stuck in "Switch On Disabled"
- Normal on startup; the enable state machine takes several cycles
- Check that DC sync is configured correctly
- Verify drive firmware supports DS402 CSP mode

### "WKC mismatch" errors
- One or more slaves are not responding correctly
- Check EtherCAT cable integrity
- On Windows the loop is fixed at 500 Hz: it is not a real-time OS, and 500 is the fastest rate that runs stably there (1000 was unstable in testing). The Pi build is the path to higher rates.
- Check that slave indices in `rig.json` match actual slave positions

### Web UI returns 421 "Host not allowed"
- The web server rejects requests whose `Host` header is not the machine's own
  address or hostname (DNS-rebinding defense). If you browse by another name,
  add it to `webAllowedHosts` in `host.json`. See `Docs/COMMAND_CONTRACT.md`

### Qt DLLs missing (Windows app won't start)
- Run `windeployqt.exe nullCAT.exe` as described above

### CMake can't find Qt
- Ensure `-DQt6_DIR` points to the correct cmake subdirectory; the path must
  end in `lib/cmake/Qt6` (or `lib/cmake/Qt5` for Qt 5)

### CMake can't find SOEM
- Windows: ensure `-DSOEM_ROOT` points to a directory containing
  `include\soem\soem.h` and `lib\soem.lib`, built for Release x64
- Linux: ensure the pinned tree at `~/SOEM` (or `-DSOEM_ROOT`) contains a
  built `build/libsoem.a`

---

# Project layout

```
CMakeLists.txt        Windows (Qt GUI) build
pi/                   Linux/Pi headless build + Pi-specific sources
src/                  Shared engine (EtherCAT master, drives, motion, web server)
web/                  Web UI (dashboard) assets
tests/                Test suites
resources/            host/rig reference JSON files
deploy/               Deployment helpers (watchdog task, mDNS advert, examples)
Docs/                 Reference documentation
```

---

# Safety notes

- Always test with **no load** first
- Set conservative position limits in `rig.json` initially
- The software e-stop immediately disables all drives, but the latched
  **hardware e-stop is the safety device**; wire one
- Never run at full stroke without verifying limits
- Read `SAFETY.md` before powering a rig

---

# Licensing

See `LICENSE` and `THIRD_PARTY_NOTICES.md`. SOEM is used under its GPLv3
option; Qt is available under LGPL or commercial license.
