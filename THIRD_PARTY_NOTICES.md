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

## Space Grotesk (web UI font)

- https://github.com/floriankarsten/space-grotesk
- Copyright 2020 The Space Grotesk Project Authors.
- SIL Open Font License 1.1. The license text ships alongside the bundled
  font files (`web/fonts/OFL.txt`), as the OFL requires.

## A6-EC servo drive documentation

- The drive user manual is © its vendor and is NOT distributed with this
  repository. Register/fault-code facts referenced in comments and docs
  were derived from the vendor manual; obtain it from the vendor
  (STEPPERONLINE).
