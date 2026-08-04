# First setup: Windows, start to finish

This walks you from an empty PC to a moving rig using the Windows release
bundle. No programming knowledge needed. If you are building from source
instead, do [BUILD_INSTRUCTIONS.md](../BUILD_INSTRUCTIONS.md) first and come
back here at step 2.

Using a Raspberry Pi as the controller instead (the recommended path)?
That walkthrough is [PI_SETUP.md](PI_SETUP.md); the SimHub setup there
is the same as step 8 here, just targeting the Pi's address.

Read [SAFETY.md](../SAFETY.md) before you power anything. The short version:
these are industrial servo drives moving real mass. Wire the hardware e-stop
first, keep a hand near it for every first, and never put a person on the rig
until it has done the same motion empty.

---

## What you need

**Hardware**
- The rig: StepperOnline A6-EC servo drives (one per axis), motors, and the
  mechanics they move.
- A wired Ethernet port for EtherCAT. Onboard Ethernet works; a cheap add-in
  Intel NIC is a good upgrade if you see instability later (see
  [KNOWN_LIMITATIONS.md](../KNOWN_LIMITATIONS.md)).
- Ethernet cables daisy-chained: PC → drive 1 IN, drive 1 OUT → drive 2 IN,
  and so on. Order along the chain determines drive numbering.
- A hardware e-stop wired to the drives. This is the safety device; the
  software cannot replace it.

**Software**
- The nullCAT Windows release ZIP.
- Npcap (free, installed in step 1).
- SimHub on the same PC, if you want game telemetry driving the motion.

---

## Step 1: Install Npcap

nullCAT talks EtherCAT through a raw network socket, and Windows needs the
Npcap driver for that. It is a kernel driver, so it cannot ship inside the
ZIP.

1. Download Npcap from <https://npcap.com>.
2. Run the installer and tick **"Install Npcap in WinPcap API-compatible
   Mode"**.
3. Reboot if the installer asks.

Skipping this is the #1 first-run failure: without Npcap, `nullCAT.exe`
won't start at all; Windows shows a *"wpcap.dll was not found"* error
before the app can even open its window.

## Step 2: Unzip and first launch

1. Unzip the release anywhere you like (e.g. `C:\nullCAT`). Everything it
   needs is in the folder: Qt and the C++ runtime are bundled.
2. Double-click `nullCAT.exe` and accept the administrator prompt
   (raw socket access requires it; the app always asks).

On first launch the app writes two config files next to the exe:

| File | What it holds | Where you edit it |
|---|---|---|
| `host.json` | This PC: network card, ports, logging | The app's **Settings…** dialog |
| `rig.json` | Your rig: axes, stroke, speeds, tuning | The **web UI** (step 5) |

You don't need to hand-edit JSON for a normal setup; the two editors cover
everything. (`host.reference.json` and `rig.reference.json` in the folder
document every field if you ever want the details.)

The window you see is a compact instrument panel: axis status at the top,
system readouts in the middle, and the run deck (E-STOP, Initialize, Start)
at the bottom. Everything is disabled until EtherCAT is up; that's normal.

## Step 3: Tell it which network port is EtherCAT

1. Click **Settings…** (bottom of the window).
2. In **NIC Name**, enter your Ethernet adapter's **Description**, not the
   connection name. Open a command prompt, run `ipconfig /all`, find the
   adapter your drive chain is plugged into and copy its **Description**
   line exactly (it looks like `Intel(R) I210 Gigabit Network Connection`,
   not `Ethernet 2`). The app also lists every adapter it can see in its
   log at startup, which is the easiest place to copy it from.
3. OK to save. This lands in `host.json` and is remembered.

Leave everything else in Settings at its defaults for now; they are the
validated values (the loop rate is fixed at 500 Hz on Windows on purpose).

## Step 4: Prepare the drives (once per drive)

Fresh A6-EC drives need a few parameters set before first use: carrier
frequency, sync settings, and (recommended) the tuning baseline. Follow
[DRIVE_TUNING.md](DRIVE_TUNING.md): Step 1 there is done on each drive's
front panel in a couple of minutes; the advanced parameter cloning is
optional at this stage.

Then power the drives and check the chain: every drive shows its panel lit,
cables PC → IN, OUT → next IN down the line.

## Step 5: Describe your axes (the web UI)

Axis layout lives in the web UI, which runs inside the app:

1. In the app, click the small **○ Off** toggle (bottom row) to turn the web
   server on, then **Web UI** to open it in your browser.
2. Open the **Config** page. Under **Axes**, describe each axis in chain
   order: a name, its type (vertical / horizontal / belt), stroke in mm,
   ballscrew pitch, encoder counts, and speed limits. Unsure? Start
   conservative; speeds can go up later.
3. **Save.** The app picks the change up automatically; no restart needed.
   (If EtherCAT is already running, it applies at the next Initialize.)

The reference values for common setups are in
[CONFIG_REFERENCE.md](CONFIG_REFERENCE.md).

## Step 6: First contact (Initialize)

Clear everyone away from the rig. Hand near the e-stop.

1. Click **Initialize EtherCAT**.
2. Within a few seconds the status line should show your drives found and
   the master **OP** (operational). Each axis row lights up.

If it fails, the error dialog tells you which of the usual four to check:
Npcap installed, running as administrator, NIC name correct, cables seated.

## Step 7: First motion (Start)

**The rig will move on this click.** Starting the loop automatically homes
every axis (a gentle push to its end stop to find zero), then moves to
center and holds. Watch the first one from a safe distance.

1. Click **Start Loop**.
2. The axis rows walk through homing → unparking → running. The status bar
   narrates. When everything settles, the rig is live and holding center.
3. Try the **E-STOP** button once now, on purpose, while nothing is riding
   the rig. Park the axes first and let them settle: click **Park All**
   on the web UI's dashboard, or use **Stop Loop** (which parks as it
   stops). An e-stop de-energizes the drives, and a vertical axis
   holding center will fall the rest of its stroke when that happens (a
   long drop on a tall rig). Parking sets the verticals down on their
   rest position so there is nothing left to fall. With the rig parked,
   press **E-STOP** and learn the release flow. You want the first
   press to not be an emergency.

**Stop Loop** parks the axes (gently to their rest position) and stops.
**Stop EtherCAT** additionally sets the verticals down on their stops and
powers the drives back to idle. Use it when you're done for the day.

## Step 8: Connect your telemetry software (SimHub example)

nullCAT listens for motion telemetry as plain UDP packets on port `4444`
(changeable in Settings). Any motion software that can send a custom UDP
string can drive it. The walkthrough below uses **SimHub**, which is what
nullCAT's initial testing was done with.

In SimHub:

1. Go to **Motion controllers** and add a **Generic UDP output**.
2. **Target IP** comes pre-filled as `127.0.0.1` (this PC); leave it
   when nullCAT runs on the same PC. If your controller is a Raspberry
   Pi or a separate box, set the controller's address instead (see
   [PI_SETUP.md](PI_SETUP.md) step 6 or
   [NUC_DEPLOYMENT.md](NUC_DEPLOYMENT.md)). Set **Target UDP port** to
   `4444`.

   ![SimHub Generic UDP output page](media/simhub-udp-output.png)

3. Click **Edit UDP commands**. Under **Protocol settings**:
   - **Axis output format:** *Decimal (string)*
   - **Axis resolution (Bit range):** *16*
   - **Number of assignable axis:** leave at *10*.

   ![SimHub protocol settings](media/simhub-protocol-settings.png)

4. In the **Motion update commands** field, build the packet:
   - Type `NULLCAT` followed by a comma.
   - Click the **…** button beside the field, choose **Insert Axis value →
     Axis 1**.

     ![Insert Axis value menu](media/simhub-insert-axis.png)

   - Type another comma, insert **Axis 2**, and keep going through
     **Axis 10**. The field ends up reading
     `NULLCAT,<Axis1>,<Axis2>,…,<Axis10>`.

     ![Finished motion update command](media/simhub-update-command.png)

     *(The screenshot shows an older `SIMHUB` prefix; type `NULLCAT`.
     The startup/shutdown command boxes stay empty; nullCAT gates motion
     on its own readiness.)*
5. The **Delay** next to that command is how often SimHub sends. The
   default `10` ms is fine and light on CPU; `2` ms is SimHub's practical
   maximum (roughly 400 updates/second). Whether the difference is
   perceptible depends on the sim's own telemetry rate: Le Mans Ultimate
   updates at 60 Hz, so 10 ms loses nothing; Assetto Corsa updates at
   333 Hz, where 2 ms preserves the extra detail.
6. OK out of the commands dialog, then open **Edit axis assignments**
   (SimHub marks it *Required*) and map your motion effects onto the axis
   slots. **Axis 1 feeds your first configured nullCAT axis** (chain
   order), Axis 2 the second, and so on.

You can verify the link without a game: the **TELEMETRY** readout in the app
shows *Listening…* with no data and flips to *Receiving* once SimHub's
motion output is enabled. Once it's receiving and the loop is running, game
motion drives the rig.

## Step 9: Optional extras

- **Belts (torque axes).** If your rig has belt tensioners, a **Slack /
  Tension Belts** button appears automatically. Slack to get in and out of
  the seat; Tension to pull snug before driving.
- **USB button box.** Bind physical buttons to actions (park, belt toggle,
  e-stop) on the web UI's bindings page. Press the button when prompted
  and it's captured. Note: on the PC the button reader runs with the web
  server, so leave the web UI enabled for bound buttons to work.
- **Start with Windows.** Run `deploy\install-task.ps1` once as
  administrator and the app starts at boot under a watchdog that restarts
  it if it ever crashes. `uninstall-task.ps1` reverses it.
- **Remote control box.** Running nullCAT on a dedicated mini-PC fed by
  your game PC is covered in [NUC_DEPLOYMENT.md](NUC_DEPLOYMENT.md).

## Everyday flow after setup

Power drives → launch app (or let the watchdog do it) → **Initialize** →
**Start** → drive. When done: **Stop Loop** → **Stop EtherCAT** → power off.
Belts slack/tension around getting in and out.

---

## If something goes wrong

| Symptom | First things to check |
|---|---|
| App won't start: *"wpcap.dll was not found"* | Npcap isn't installed; see step 1 (tick WinPcap-compatible mode) |
| *"No slaves found"* at Initialize | Running as admin? NIC Name matches the adapter's **Description** in `ipconfig /all` (not the connection name)? Cables in the right ports (IN vs OUT)? Drives powered on? |
| Initialize finds drives but fails entering OP | Drive power, cable seating mid-chain, drive panel showing an error code |
| A drive panel shows an `Er` code | Look it up in [DriveFacts.md](DriveFacts.md); many clear with **Reset Faults** |
| TELEMETRY stuck on *Listening…* | SimHub motion enabled? Sending UDP to `127.0.0.1:4444`? Port matches Settings? Command format exactly as in step 8 (Decimal, 16-bit, `NULLCAT,` prefix)? |
| Motion feels wrong / too aggressive | Revisit axis speeds and stroke in the web UI; start lower |
| Anything unexplained | `logs\app.log` next to the exe narrates everything with timestamps. Read the last screenful, or attach it when asking for help |

The WKC error counter in the app ticking up occasionally is normal on
Windows and harmless; see [KNOWN_LIMITATIONS.md](../KNOWN_LIMITATIONS.md)
for the transport story and when to consider a dedicated NIC.
