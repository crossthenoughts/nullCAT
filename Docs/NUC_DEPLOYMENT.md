# Dedicated Windows NUC Deployment

Run the control software on a **dedicated Windows NUC** that sits between the game
PC and the drives: the Windows equivalent of the headless Pi. You tune and configure
from the game PC's **browser** (web UI) and run the NUC headless-ish.

```
[ Game PC ]  sim + SimHub
     │  UDP (static-IP point-to-point link)
     ▼
[ NUC ]  nullCAT
     │  EtherCAT (raw L2 / Npcap)
     ▼
[ Drives ]
```

The engine (SOEM/EtherCAT, MotionController, ControlLoop) transfers unchanged. A
NUC is just a Windows PC, and UDP is network-transparent.

One setting makes this work: `telemetryBindAddr`. The default (`127.0.0.1`) binds the
telemetry receiver to loopback, which receives nothing from a separate game PC. A NUC
must bind `0.0.0.0` or the link NIC's IP.

---

## 1. Hardware

- **Two NICs on the NUC** (same split as the Pi: one IP link, one raw EtherCAT):
  - **Link NIC**: static IP, point-to-point cable to the game PC's SimHub-output NIC.
  - **EtherCAT NIC**: dedicated; SOEM takes it raw, so it cannot share the IP stack.
    A USB-Ethernet adapter or a dual-NIC NUC.
- **Npcap** installed in **WinPcap-compatible mode** (for SOEM raw sockets).

## 2. Network / OS (host-level, not the app)

- **Static IP** on the link NIC, same subnet as the game PC's dedicated
  SimHub-output NIC; direct cable.
- **SimHub** on the game PC: point its UDP output at the **NUC's static IP : `telemetryPort`**
  (instead of `127.0.0.1`).
- **Windows Firewall: allow inbound UDP on `telemetryPort`.** This is a *new* requirement
  vs the same-PC case (loopback bypasses the firewall; cross-network does not). Easy to
  miss, and the failure mode looks identical to "no telemetry."

## 3. App config (`host.json`)

Edit the NUC's `host.json` (see `deploy/host.nuc.example.json` for a template; all
host fields are documented in `host.reference.json`):

| Field | NUC value | Why |
|---|---|---|
| `nicName` | the **EtherCAT** NIC name | the interface SOEM opens raw |
| `telemetryBindAddr` | `"0.0.0.0"` (or the link NIC's IP) | receive telemetry from the **game PC**, not loopback |
| `telemetryPort` | match the sender's target port | |
| `webUIEnabled` | `true` | you operate/configure from the game PC's browser |
| `webBindAddr` | `"0.0.0.0"` | reachable from the game PC (see exposure note) |
| `webPort` | e.g. `8080` | |
| `webAllowedHosts` | any extra names you browse by | the Host-header allowlist accepts the NUC's own addresses and hostname automatically; browsing by any other name gets 421 unless listed here |

`rig.json` (axes + tuning) is edited entirely from the **web UI**: copy it from a
known-good rig, or tune in the browser.

> **Exposure note:** binding the web UI to `0.0.0.0` exposes it on the network.
> The server enforces a Host-header allowlist (foreign `Host` gets 421, a
> DNS-rebinding defense) and accepts `/api/shutdown` and `/api/restart` only from
> private client addresses, but there is **no login**: anyone who can reach the
> port can operate the rig. Keep it on a trusted point-to-point link or LAN. See
> COMMAND_CONTRACT.md for the request-level guards.

## 4. Windows-as-appliance gotchas (the Pi doesn't have these)

- **Interactive session required.** The Qt app needs a logged-in desktop session and
  a visible window (ForegroundKeeper needs a message pump). An unattended NUC needs
  **auto-login to the console session, kept unlocked**. RDP disconnect, the lock
  screen, and fast user switching can throttle or tear down the GUI session. This is
  the biggest appliance risk the Pi (systemd, no GUI) does not have.
- **Auto-start + restart:** run `deploy/install-task.ps1` once (as admin) to register
  the Task Scheduler task that launches `nullCATWatchdog.exe` at logon with admin
  rights (no UAC prompt). The watchdog relaunches the app on crash/restart: the PC
  analogue of the Pi's `systemd Restart=always`.
- **RT determinism is a BIOS job.** Disable C-states / SpeedStep and NIC power
  management, the same tuning as any Windows EtherCAT master. Not code.
- **ForegroundKeeper** still matters for background-throttling/session state, though on
  a dedicated NUC there's no fullscreen game stealing foreground.

## 5. Bring-up check

- Start the app (or let the watchdog task start it), then open
  `http://<NUC-IP>:<webPort>/` from the game PC's browser.
- The dashboard's **UDP-rate diagnostic** (`udpNewHz` in `/api/status`) shows
  immediately whether telemetry is actually arriving: the "is the link talking"
  indicator. The app can only *surface* the link state, not enforce the static IP.
- On the first launch the app **cold-starts** a fresh `host.json` +
  `rig.json` (or cold-starts defaults), so the dir always ends up with the two-file set.

## 6. Safety

The latched hardware e-stop is the safety device, wired direct to the drives. No
software path (Qt, web, or bound button) can defeat it while latched; it clears only by
physical unlatch followed by an explicit software re-arm. Software senses and reflects
that state; it never overrides it. See SAFETY.md.
