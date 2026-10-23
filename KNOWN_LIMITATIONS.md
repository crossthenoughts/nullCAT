# Known limitations

Honest list, maintained. Read it before filing a bug; most "bugs" on
Windows are entries on this page.

## Windows transport is best-effort, not real-time

Windows is not an RT OS. The Windows build uses Npcap raw sockets and MMCSS
scheduling, which delivers a usable loop most of the time, but frame timing
can and does wobble under load (WKC misses, jitter spikes, occasional DC
re-syncs). The architecture absorbs this (hold-previous-PDO, time-based
recovery), but the Pi + PREEMPT_RT build is the reference platform and
behaves measurably better. Some recommendations for Windows: using a dedicated PCIE NIC for
the EtherCAT segment, wired everything, no virtualization, administrator
rights for Npcap.

### What the difference looks like in practice

An indicative comparison from the project's own session logs, not a
controlled benchmark: two Windows sessions on the reference rig
(August 2026, 500 Hz loop) against a Raspberry Pi session on the same
class of rig (2 kHz loop, six drives, including one continuous
two-hour drive).

- **Loop stalls.** The Windows sessions logged 1,900 to 2,600
  "stall -- resynced" events each: moments where the OS held the loop
  for 25 to 33 ms and two to five command cycles were skipped. Each
  one is a brief hold of the last command frame, in other words a
  micro-stutter, felt most in slow smooth motion and during homing.
  The Pi logged zero stall events, in every session. 0.9.8 removes the
  stall itself on Windows (see below); what remains of each pause is one
  cycle of held inputs.
- **Bus health.** The Windows sessions showed up to a few hundred
  working-counter (WKC) misses and dozens of DC sync-loss events per
  session. The Pi showed zero of both across nearly fifteen million
  cycles, with more drives at four times the loop rate.
- **Motion consequence.** Peak following error under hard driving ran
  7 to 8 mm on the Pi against 9 to 11 mm on Windows. Different
  sessions and different driving, so treat that as directional rather
  than a measurement; the stall-free cadence and the 0.5 ms command
  steps (versus 2 ms) are the mechanical reasons to expect it.

### Where the Windows stalls came from, and what is left

The stalls had one cause. The loop sent each frame with a call that only
returns once the adapter driver reports the transmit done, and on the
Intel I210 and I211 that report can come 15 or 31 ms late (the driver's
own housekeeping timer) while the frame itself left in microseconds.
Since 0.9.8 the frames leave through sender threads and the loop goes
straight on to the reply, so the loop keeps its cadence and the drives
get every frame on time. The hitches are gone.

What is left is the other half of the same pause: the reply sits in the
adapter until that timer fires, so the loop gives up waiting for it
(2 ms), holds the previous inputs for that cycle and counts a
working-counter miss. On the 0.9.8 test rig (Ryzen 7 5800X, Intel I210
and onboard I211, six drives, 500 Hz) that is about twenty short pauses
a second with the game running, mostly under 15 ms, a few up to 180 ms.
Motion is unaffected. The misses show in the log's minute summary and
in the loop-stop line as "late replies"; "stale replies" there is the
count of old replies caught and dropped. The pauses are a driver and
interrupt matter, not nullCAT's. Things worth checking, in this order:

- In Device Manager, the EtherCAT network port's **Resources** tab: the
  IRQ should be a negative number (message-signalled interrupts). A
  small positive number means a line interrupt shared with other
  devices. On the test rig both ports are message-signalled and the pauses
  remain, so this is a check, not a fix.
- The port's **Advanced** tab: **Interrupt Moderation** off, **Energy
  Efficient Ethernet** off, **System Idle Power Saver** off. The
  **Power Management** tab: do not allow the computer to turn the device
  off.
- A dedicated PCIe network card does not change the pauses on this rig;
  the PCIe I210 and the onboard I211 behave the same.

### Haptics on Windows

The Windows loop runs at 500 Hz, so an effect can carry nothing above
250 Hz (the Pi's 2 kHz loop goes to 1000 Hz). Most effects sit well
below that. What reaches it:

- **Engine firing thumps.** A V8 fires 250 times a second at about
  3750 rpm. Past that, on the default **foldback** setting (wrap), the
  thumps fold back into a throb that falls as the revs climb. **stop**
  fades them out before that point; **keep rising** carries on at every
  other firing, then every fourth, so the pitch keeps climbing.
- **The engine buzz.** On automatic order it reaches the redline at
  62.5 Hz on Windows (120 Hz on the Pi), so it sits lower. A hand-set
  order above about 0.75 makes the motor also play a faint falling
  mirror tone near the redline.
- **Force devices** (a shifter or pedal) need the Pi: the force loop
  needs 2 kHz, and the Devices section is hidden on Windows.

### Shakers on Windows

nullCAT takes each shaker card for itself (exclusive mode) and holds it
at full volume. Set the volume on the amps. Two things outside nullCAT
can still get in the way:

- The card must be allowed for apps (Windows Settings, Sound, the card,
  **Allow apps and Windows to use this device for audio**), and set up
  with enough channels in the classic Sound panel (`mmsys.cpl`,
  **Configure**: quadraphonic, 5.1 or 7.1).
- Another program using the card. SimHub was found binding a shaker
  card it had once used, on some starts, and muting it, which left every
  shaker effect barely felt. With nullCAT running first it cannot take
  the card; the shaker line counts any time something turned the card
  down.

None of this makes the Windows build unsafe or unusable, and that is
the point of the architecture: every one of those events is absorbed
by design. A stalled loop holds its last frame and resynchronizes
without a catch-up burst, a WKC miss rides on the previous process
image, sync wobble is ridden out rather than escalated, and the motion
guards (velocity clamps, braking-aware limits, belt folds) stay active
throughout. The thousands of events in those Windows logs produced
zero drive faults and zero aborted sessions; what they cost is
texture, not safety. Windows is the convenient platform. The Pi build
is the reference platform, and its logs simply have nothing to absorb.

## Tested scale

Validated on the reference rigs: 6 drives on one segment on the Pi
(CSP vertical and horizontal actuators, a CST belt tensioner, and a
CST force device), 6 drives on Windows, daily use on both platforms.
Larger topologies (7 to 10 drives) are untested and are a stated goal
of the beta program; if you run a bigger chain, your results are
exactly the data the project needs.

## Shakers

The status line under the shaker settings shows the card, its buffer
and an underrun count; a rising underrun count with the control loop
running is the data to report. Two cards at different buffer sizes land their effects a few
milliseconds apart; that difference is not compensated yet.

## Simulation mode cannot home

Simulated drives have no hardstop model, so homing (torque-threshold
search) cannot complete in simulation; sim sessions run pre-homed
lifecycles only. Full-lifecycle simulation is tracked as future work.

## Drive support is A6-EC-specific in places

The DS402 core is generic, but fault-code interpretation (Er-code tables),
runaway-protection provisioning (C06.20), and the provisioning tool's
register map are STEPPERONLINE A6-EC specific. Other drives will need their
own provisioning profile and fault table.

## Audible drive noise on A6-EC (carrier whine and encoder chirp)

Expect audible noise from A6-EC motors when the control loop is active.
There are two distinct sources, and they respond to different fixes:

- **Carrier whine.** The drives ship with the PWM carrier frequency at
  8 kHz, which is squarely audible. For CSP position axes, doubling it to
  16 kHz effectively silences the carrier.
- **Encoder-modulated tone under torque.** At higher torque gain a
  modulated signal at roughly 8 kHz returns even on the 16 kHz carrier;
  in testing on the reference rig this appears at around 55% torque in
  CSP. This can be designed around to some extent mechanically (ballscrew
  selection and similar choices affect how hard the drive works).

The 16 kHz recommendation does not carry over to CST torque mode. On a
belt tensioner at 16 kHz the encoder-modulated tone is markedly worse than
in CSP and appears at a lower torque level, so raising the carrier trades
a steady note for a nastier intermittent one. Leave CST axes at the 8 kHz
carrier: the noise there is constant and does not change with torque
level, which in practice is the more livable profile. Noise mitigation is
still under active investigation and this section will be updated as
findings land.

None of this is caused by nullCAT or fixable in software; it is drive-side
behaviour and drive-side tuning. See Docs/DRIVE_TUNING.md for carrier
frequency settings and the wider tuning walkthrough before filing a noise
issue.

For context against the drives most builders know: subjectively the A6
sits between the grey and white AASD drives in how disruptive its noise
profile is. Taken with the bigger picture of cost, EMI behaviour,
performance, and size, the A6 still comes out favourably.

## Platform caveats

- **Pi, building from source:** compile parallelism is limited by RAM, not
  CPU (all Pi 4 variants have 4 cores, but each parallel C++ compile job
  can peak near 1 GB). Rule of thumb is one job per GB of RAM: `-j1` on a
  1GB board, `-j2` on 2GB, `-j4` on 4GB or 8GB. Overshooting causes
  OOM-killed compiles (often surfacing as "internal compiler error" or a
  killed process) or SD-card swap thrash. On 1GB boards the distributed
  image is the supported path; source builds there work with `-j1` but are
  slow, and cross-compiling is the better option.
- **Pi web UI:** the UI rides whatever NIC you give it; a USB NIC carrier
  drop looks like "the UI died" while the rig keeps running.
- **EtherCAT segment:** one dedicated NIC, no switches with EEE/green
  Ethernet between master and slaves.
- **Telemetry:** the UDP stream is fire-and-forget; a stopped stream triggers
  staged standby (hold, ease to center, park), not an instant stop.

## Security model: trusted LAN

The web UI and API are served over plain HTTP, intended for a trusted
private network. Do not port-forward the controller to the internet.
What IS enforced: no CORS headers (arbitrary web pages cannot read API
responses cross-origin), a fail-closed Host-header allowlist that blocks
DNS rebinding (only Hosts naming this machine, or names listed in
`webAllowedHosts` in host.json, are served), and power-off/restart
additionally require the client address to be on the local network. An
optional **web password** (Setup, Network / telemetry) makes every
browser enter it once before it can see or control the rig; it keeps
casual access out on a shared network, but over plain HTTP it is not
encryption. Without it, anyone on your LAN can command the rig.
