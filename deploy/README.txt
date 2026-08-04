nullCAT — Windows Release
=========================

WHAT THIS IS
  A self-contained Windows build. Qt DLLs and the MSVC runtime are bundled in
  this folder, so it runs on a clean machine WITHOUT installing the Visual C++
  redistributable.

PREREQUISITE (one, and it can't be in this folder)
  * Npcap  — https://npcap.com  — install in "WinPcap API-compatible Mode".
    SOEM uses a raw Ethernet socket; Npcap is a kernel driver, so it must be
    installed separately. Without it, nullCAT.exe won't start at all (Windows
    reports wpcap.dll missing).

RUN
  * Run nullCAT.exe AS ADMINISTRATOR (raw socket access).
  * First launch writes fresh defaults: host.json + rig.json (the two-file
    config). Edit host.json for your machine via the app's Settings dialog
    (NIC name, ports); the web UI edits rig.json (axes + tuning).
  * Full walkthrough: docs\FIRST_SETUP_WINDOWS.md (in this folder) — start
    there. Config reference and drive tuning live in docs\ too.
  * Logs are written to logs\app.log next to the exe.

WEB UI (optional on the desktop; primary on a NUC)
  * Click "Enable Web UI" in the app, then "Open Web UI" — or set
    "webUIEnabled": true in host.json. Default bind is localhost (127.0.0.1).
  * On the desktop the host settings are read-only in the browser (the app owns
    them); axes + tuning are edited there.

DEDICATED NUC / REMOTE CONTROL BOX
  * See Docs\NUC_DEPLOYMENT.md and host.nuc.example.json.
  * deploy\install-task.ps1 (run once as admin) sets up auto-start + auto-restart
    via the watchdog.

SAFETY
  * The hardware e-stop wired direct to the drives is the safety device. No
    software path can defeat it while latched. Always test with no load first.
