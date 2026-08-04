# A6-EC drive tuning & commissioning

This page covers the drive-side settings nullCAT expects and the tuning
that shapes how the drives sound and feel. It mirrors the Drive
Provisioning reference built into the web UI (Configuration → Drive
Provisioning) so the information is available without a running rig.
Scope: STEPPERONLINE A6-EC (ANCTL AS715N). Read
[SAFETY.md](../SAFETY.md) first. Drives stay disabled and nobody sits on
the rig while commissioning.

## Carrier frequency and audible noise

There are two separate noise sources, and they respond to different fixes
(background in [KNOWN_LIMITATIONS](../KNOWN_LIMITATIONS.md)):

- **Carrier whine.** The drives ship with the PWM carrier at 8 kHz, which
  is well inside the audible range. For CSP position axes, doubling it to
  16 kHz effectively silences the carrier. Step 1 below does this.
- **Encoder-modulated tone under torque.** At higher torque a modulated
  tone near 8 kHz returns even on the 16 kHz carrier. On the reference rig
  it appears from roughly 55% torque in CSP. This one is partly a
  mechanical design question, since ballscrew selection and similar
  choices set how hard the drive has to work.

Do NOT raise the carrier on CST torque axes (belt tensioners). At 16 kHz
the encoder-modulated tone is markedly worse than in CSP and shows up at a
lower torque level, so you trade a steady note for a nastier intermittent
one. Leave CST axes at 8 kHz: the noise there is constant and does not
change with torque, which in practice is the more livable profile.

## Step 1: unlock and carrier (every drive, front panel only)

1. **Unlock:** set `C00.31` = **1107** (super-user; drive disabled).
2. **Carrier to 16 kHz** (CSP axes only, see above): navigate to `R22.38`
   and change **0 to 1** (a ×2 multiplier on the 8 kHz base).
3. **Save** the params, then **power-cycle** the drive.

⚠ This step is panel-only. `R22.38` (`0x2022:39`) is not writable over
EtherCAT/SDO even after the unlock, so the `provision` tool cannot set it.
Only 8 and 16 kHz are valid; any other carrier value faults the drive
(`Er31.0` / `Er50.1`). Skip the carrier change if a drive must match
others still on the 8 kHz base.

## Step 2 (ADVANCED): config params via the `provision` tool

Please note: drives can be configured manually. Every value the tool
writes is listed in the parameter tables below with its front-panel code,
so keying them in on the drive panel gets the same result. The tool just
automates the writes, verifies each readback, and keeps a backup of the
before-state.

Run with the nullCAT service stopped, because the tool needs to own the
NIC:

```
sudo systemctl stop nullcat-pi
sudo ./provision eth0 <slave> --fresh    # write the params, verify, store to NVRAM
# power-cycle the drive
sudo ./provision eth0 <slave> --verify   # confirm everything persisted
```

The tool writes the 18 deliberate params plus both notch frequencies
(7999) and the factory inertia baseline, and stores to NVRAM only if every
readback verifies. A before-state backup file is written first. Notch
frequency is tuned per axis (to the measured resonance); inertia stays at
the factory 100 baseline on all axes.

## The parameter set

The values below are exactly what the `provision` tool writes. Every one
has a front-panel code, so you can key them in by hand if you would rather
not run the tool. Save to NVRAM on the panel when done.

### EtherCAT sync (required by this master)

| Panel  | Control            | nullCAT | Notes                                |
|--------|--------------------|---------|--------------------------------------|
| C13.05 | Sync mode          | 2       | DC sync (mandatory for this master)  |
| C13.02 | Sync lost window   | 20      |                                      |
| C13.06 | Sync error window  | 6000    |                                      |

### Gains, stiffness & inertia (cloned across drives)

| Panel  | Control                        | nullCAT | Notes                                    |
|--------|--------------------------------|---------|------------------------------------------|
| C00.04 | Gain auto-tune mode            | 0       | Manual: gains frozen, not auto-tuned     |
| C00.05 | Stiffness level                | 10      |                                          |
| C00.06 | Load inertia ratio             | 100     | Factory baseline; leave at 100 on all axes |
| C01.01 | 1st speed loop gain            | 150     |                                          |
| C01.02 | 1st speed loop integral time   | 5685    |                                          |
| C01.03 | 1st torque ref filter time     | 112     |                                          |
| C01.08 | 2nd position loop gain         | 400     |                                          |
| C01.09 | 2nd speed loop gain            | 250     |                                          |
| C01.0A | 2nd speed loop integral time   | 3184    |                                          |
| C01.0B | 2nd torque ref filter time     | 200     |                                          |
| C01.10 | Speed feedback filter          | 1       |                                          |
| C01.22 | 1st-order low-pass filter time | 50      |                                          |

### Notch filters, sound profile (notches 1 & 2)

| Panel  | Control             | nullCAT | Notes                                                          |
|--------|---------------------|---------|-----------------------------------------------------------------|
| C01.40 | 1st notch frequency | 7999    | 8000 = parked/off; 7999 engages. Per-axis: set to measured resonance |
| C01.41 | 1st notch width     | 100     | 0 = filter does nothing                                         |
| C01.42 | 1st notch depth     | 10      | 1000 = no attenuation, 10 = deepest                             |
| C01.43 | 2nd notch frequency | 7999    |                                                                 |
| C01.44 | 2nd notch width     | 100     |                                                                 |
| C01.45 | 2nd notch depth     | 10      |                                                                 |

## CST torque axes (belt tensioners): runaway protection

A CST belt axis is dragged by the driver against its commanded torque.
That is how it is meant to work, but the drive's runaway protection reads
it as motion inconsistent with torque and faults `Er06.0` (`0x8400`). The
remedy is `C06.20` = **0** on torque-mode drives only, saved to NVRAM. Set
it manually on the front panel; that is the proven path. (The `provision`
tool's profile includes it for torque axes, but this particular write has
not yet been verified over EtherCAT on real hardware, and some A6 objects
reject SDO writes, so treat the panel as authoritative for now.) The
application never manages it. Note that the drive-side
speed objects (`0x607F`, `C03.47/48`) do not restrain CST on the A6, so
nullCAT's master-side velocity fold is the enforced speed limit.

## References

- Full parameter table with the factory-vs-configured diff:
  [drive_provisioning_param_map.md](drive_provisioning_param_map.md)
- A6-EC drive user manual: obtain from the vendor (STEPPERONLINE). It is
  not distributed with this repository (see
  [THIRD_PARTY_NOTICES](../THIRD_PARTY_NOTICES.md)).
