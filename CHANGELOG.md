# Changelog

Notable changes to nullCAT. Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/);
versioning is [Semantic Versioning](https://semver.org/) — while on `0.x`, the
middle number carries breaking changes and the last carries fixes.

## [Unreleased]

### Added
- **Qt UI: Park / Unpark button.** The desktop panel was the only control
  surface without one (the engine, the web UI and the HID `park-toggle`
  binding all had it). Sits above the Belts toggle; label shows the action,
  and it is disabled mid-transition so it cannot reverse a park, unpark or
  home already in flight.

### Changed
- **Web config page: the Save bar is now sticky** and Button bindings are
  collapsed by default. The save button used to sit below a tall, full-width
  bindings panel that has its own "Save bindings" button, so it was easy to
  save the bindings and believe the config was saved too. The two write
  different files: bindings hot-apply, config needs a restart.
- **Axis defaults now come from the compiled-in values** in `Config.h`, as
  `Docs/CONFIG_REFERENCE.md` has always claimed. The `rig.json` reader carried
  its own second set of defaults which had drifted from the struct in three
  places.

  > **If your `rig.json` omits any of these keys, the effective value changes:**
  > `homeMode` `center` → `endstop`, `homingSpeedMmS` `5` → `250`,
  > `maxAccelerationMmS2` `2000` → `10000`. A rig configured through the web UI
  > is unaffected — it writes every key explicitly. A hand-written or partial
  > `rig.json` is not: in particular, homing would approach the hardstop 50×
  > faster than before. Check your axes before the first run on this version.

### Removed
- **`homeMode: "gravity"`.** It declared an axis homed at wherever it happened
  to be resting, with no search, on the assumption that gravity had already
  parked a vertical actuator on its bottom stop. A leftover from the PP-mode
  era; never exposed in the UI, never documented, unused. Any unrecognised
  `homeMode` now falls through to the real torque-based endstop search, so a
  config still carrying `"gravity"` gets safer rather than broken.

### Fixed
- **Unpark now refuses an axis that was never homed.** Not reachable in the
  normal flow (stopping the loop re-arms the rehome, starting it homes, and
  park/unpark require a running loop), but a homing fatal error left that axis
  parked-and-unhomed while its peers also ended parked — which reads as "all
  parked", so the toggle offered Unpark. Unpark ramps toward mid-stroke, and
  for an unhomed axis that target is in a coordinate frame unrelated to the
  machine.
- **Windows builds from a clean checkout.** The Npcap SDK's `Include`
  directory was never on the compile path, so SOEM's `nicdrv.h` could not find
  `pcap.h`. It built only on machines whose SOEM tree happened to vendor the
  pcap headers. `BUILD_INSTRUCTIONS.md` also now covers the SDK as a separate
  download from the Npcap runtime, and requires **1.16 or newer** — 1.13 and
  older redefine `bpf_program`/`bpf_insn` and fail to compile against SOEM 2.x.
- **Release archives use ZIP-spec path separators.** `Compress-Archive` writes
  backslashes, which Windows tools tolerate but some non-Windows tools turn
  into literal backslashes in filenames.

### Internal
- Windows CI is green: Npcap SDK and the pinned SOEM build are cached (the
  download is retried with backoff, since npcap.com intermittently blocks CI
  address ranges), and the SOEM-linking test binaries delay-load `wpcap.dll`
  so they run on a machine that has the SDK but not the Npcap driver.
  `nullCAT.exe` itself still links it normally, so a missing Npcap fails
  immediately and visibly for an end user.

## [0.9.0] — 2026-08-04

First public beta. Windows desktop build (Qt + Npcap/SOEM) and the Raspberry Pi
headless daemon share one motion core.

- EtherCAT master (SOEM) with DS402 drive management: staged init, DC-sync
  (SYNC0), fault monitoring and recovery, and provisioning tooling for
  STEPPERONLINE A6-EC drives.
- Motion: UDP telemetry (e.g. SimHub) → per-axis tracking filter → CSP position
  drives (heave/pitch/roll) and CST torque drives (belt tensioners), with homing
  against hardstops, parking, e-stop staging and telemetry-loss standby.
- Control surface: embedded web UI (status, config, drive cards, HID button
  bindings), a compact Qt desktop panel on Windows, and an optional GPIO control
  panel on the Pi.

[Unreleased]: https://github.com/crossthenoughts/nullCAT/compare/v0.9.0...main
[0.9.0]: https://github.com/crossthenoughts/nullCAT/releases/tag/v0.9.0
