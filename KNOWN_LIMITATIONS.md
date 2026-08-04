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

## Tested scale

Validated on the reference rig: 4 drives on one segment (3x CSP vertical
actuators plus 1x CST belt tensioner), daily use on both platforms. Larger
topologies (5 to 10 drives) are untested and are a stated goal of the beta
program; if you run a bigger chain, your results are exactly the data the
project needs.

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

The web UI and API are served over plain HTTP and are unauthenticated,
intended for a trusted private network. Do not port-forward the controller
to the internet. What IS enforced: no CORS headers (arbitrary web pages
cannot read API responses cross-origin), a fail-closed Host-header
allowlist that blocks DNS rebinding (only Hosts naming this machine, or
names listed in `webAllowedHosts` in host.json, are served), and
power-off/restart additionally require the client address to be on the
local network. There is no token or password auth in v1; anyone on your
LAN can command the rig.
