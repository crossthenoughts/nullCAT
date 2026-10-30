# Third-party notices

nullCAT is licensed under GPL-3.0-or-later (see LICENSE). It builds against
or bundles the following third-party components.

Scope: this file covers THIS REPOSITORY. The distributed Pi image bundles a
full operating system (Raspberry Pi OS / Debian) whose package licenses are
documented on the image itself under /usr/share/doc/<package>/copyright.

## SOEM (Simple Open EtherCAT Master)

- https://github.com/OpenEtherCATsociety/SOEM
- Copyright notice reproduced verbatim from LICENSE.md at the pinned
  revision this project builds against:

  > Copyright (C) 2005-2025 Speciaal Machinefabriek Ketels v.o.f.
  > Copyright (C) 2005-2025 Arthur Ketels
  > Copyright (C) 2009-2025 RT-Labs AB, Sweden

- Dual-licensed: GPLv3, or a commercial license from RT-Labs. This project
  uses it under the GPLv3 option (verified against the pinned tree's
  LICENSE.md). Built from source at a pinned revision; headers and library
  must come from one tree (the generated `ec_options.h` sizes
  `ecx_contextt`).

## cpp-httplib

- https://github.com/yhirose/cpp-httplib
- MIT License. Vendored as `src/httplib.h` (single header, unmodified; the
  license text is embedded in the header itself).

## miniaudio

- https://github.com/mackron/miniaudio
- Public domain or MIT No Attribution, at your choice. Vendored as
  `src/miniaudio.h` (v0.11.21, single header, unmodified; the license
  text is embedded at the end of the header). Drives the USB sound card
  for the shaker outputs (ALSA on the Pi, WASAPI on Windows).

## Qt 6

- https://www.qt.io
- LGPLv3, dynamically linked. The Pi build uses QtCore only (JSON config
  parsing); Windows builds additionally use QtGui/QtWidgets/QtNetwork, also
  dynamically linked. Because linking is dynamic, you can replace the Qt
  libraries shipped with any binary release with your own compatible
  builds; Qt's corresponding source is available from
  https://www.qt.io/offline-installers (or https://download.qt.io).

## libgpiod (Linux only)

- https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git
- LGPL-2.1-or-later, dynamically linked (v2 API). Optional: only used when
  the GPIO control panel is enabled.

## Fonts (web UI and desktop window)

All SIL Open Font License 1.1. Each license text ships alongside the bundled
font files, as the OFL requires. The web UI serves the woff2 files in
`web/fonts/`; the Windows desktop window embeds TTF copies in `nullCAT.exe`
(sources in `resources/fonts/`, which a release does not ship separately; its
license texts are the same as those in `web/fonts/`).

### Barlow (web UI and desktop window)

- https://github.com/jpt/barlow
- Copyright 2017 The Barlow Project Authors.
- License text: `web/fonts/OFL-Barlow.txt`, `resources/fonts/OFL-Barlow.txt`.

### JetBrains Mono (web UI and desktop window)

- https://github.com/JetBrains/JetBrainsMono
- Copyright 2020 The JetBrains Mono Project Authors.
- License text: `web/fonts/OFL-JetBrainsMono.txt`, `resources/fonts/OFL-JetBrainsMono.txt`.

### Archivo (web UI and desktop window)

- https://github.com/Omnibus-Type/Archivo
- Copyright 2020 The Archivo Project Authors.
- License text: `web/fonts/OFL-Archivo.txt`, `resources/fonts/OFL-Archivo.txt`.

### DSEG7 Classic (web UI)

- https://github.com/keshikan/DSEG
- Copyright (c) 2017, keshikan, with Reserved Font Name "DSEG".
- License text: `web/fonts/OFL-DSEG.txt`.

## A6-EC servo drive documentation

- The drive user manual is © its vendor and is NOT distributed with this
  repository. Register/fault-code facts referenced in comments and docs
  were derived from the vendor manual; obtain it from the vendor
  (STEPPERONLINE).
