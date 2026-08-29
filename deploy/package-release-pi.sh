#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
# SPDX-License-Identifier: GPL-3.0-or-later
#
# package-release-pi.sh - build the shippable Pi release tarball.
#
# Produces dist/nullCAT-v<version>-pi-aarch64.tar.gz (+ .sha256) from an
# already-built pi tree, staging an ALLOWLIST only - the Pi analogue of
# deploy/package-release.ps1. Run on aarch64 (CI runs it inside the same
# Debian trixie arm64 container that builds and tests the daemon; the
# release workflow attaches the result to the GitHub release).
#
# Usage: deploy/package-release-pi.sh <version> <build-dir>
#   e.g. deploy/package-release-pi.sh 0.9.5 build-arm64

set -euo pipefail

VERSION="${1:?usage: package-release-pi.sh <version> <build-dir>}"
BUILD="${2:?usage: package-release-pi.sh <version> <build-dir>}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$(cd "$BUILD" && pwd)"
DIST="$REPO/dist"
NAME="nullCAT-v${VERSION}-pi-aarch64"
STAGE="$DIST/$NAME"

echo "== nullCAT Pi release packaging v$VERSION =="

rm -rf "$STAGE"
mkdir -p "$STAGE/logs" "$STAGE/docs/media" "$STAGE/os-setup"

# ---- Binaries + web UI (from the build tree: what was tested is what ships)
cp "$BUILD/nullcat-pi"  "$STAGE/"
cp "$BUILD/provision"   "$STAGE/"
cp -r "$BUILD/web"      "$STAGE/web"
if [ -d "$BUILD/drive_profiles" ]; then
    cp -r "$BUILD/drive_profiles" "$STAGE/drive_profiles"
else
    echo "note: no drive_profiles/ in the build tree -- skipped"
fi

# ---- Reference configs, licence and safety texts, docs (offline-readable,
# same layout as the Windows zip so relative links resolve as on GitHub)
cp "$REPO/resources/host.reference.json" "$STAGE/"
cp "$REPO/resources/rig.reference.json"  "$STAGE/"
cp "$REPO/LICENSE"                "$STAGE/LICENSE.txt"
cp "$REPO/THIRD_PARTY_NOTICES.md" "$STAGE/"
cp "$REPO/SAFETY.md"              "$STAGE/"
cp "$REPO/KNOWN_LIMITATIONS.md"   "$STAGE/"
cp "$REPO/BUILD_INSTRUCTIONS.md"  "$STAGE/"
cp "$REPO/CHANGELOG.md"           "$STAGE/"
cp "$REPO"/Docs/*.md              "$STAGE/docs/"
cp "$REPO"/Docs/media/simhub-*.png "$STAGE/docs/media/" 2>/dev/null || true

# ---- Service units + updater: each version dir carries its own updater,
# so updating also updates the updater.
cp "$REPO/pi/nullcat-pi.service"            "$STAGE/os-setup/"
cp "$REPO/pi/nullcat-update@.service"       "$STAGE/os-setup/"
cp "$REPO/pi/nullcat-poweroff.sudoers"      "$STAGE/os-setup/" 2>/dev/null || true
cp "$REPO/pi/os-setup/nullcat-update.sh"    "$STAGE/os-setup/"
chmod +x "$STAGE/os-setup/nullcat-update.sh" "$STAGE/nullcat-pi" "$STAGE/provision"

# ---- Manifest: what the updater reads before applying.
# layout: versioned-directory scheme generation (install.sh >= 0.9.5).
# requiresFullReinstall: set true in a release whose OS-level tuning
# changed; the updater then refuses the quick path and says why.
cat > "$STAGE/manifest.json" <<EOF
{
    "version": "${VERSION}",
    "layout": 1,
    "requiresFullReinstall": false
}
EOF

# ---- Sanity: nothing machine-local or stale slips in
for f in host.json rig.json buttons.json config.json; do
    if [ -e "$STAGE/$f" ]; then echo "FORBIDDEN file staged: $f" >&2; exit 1; fi
done
for f in nullcat-pi provision manifest.json web/index.html \
         os-setup/nullcat-update.sh os-setup/nullcat-pi.service \
         os-setup/nullcat-update@.service SAFETY.md \
         host.reference.json rig.reference.json docs/PI_SETUP.md; do
    if [ ! -e "$STAGE/$f" ]; then echo "MISSING from stage: $f" >&2; exit 1; fi
done

# ---- Tarball + checksum
tar -czf "$DIST/$NAME.tar.gz" -C "$DIST" "$NAME"
( cd "$DIST" && sha256sum "$NAME.tar.gz" > "$NAME.tar.gz.sha256" )

echo "== DONE: $DIST/$NAME.tar.gz =="
ls -la "$DIST/$NAME.tar.gz" "$DIST/$NAME.tar.gz.sha256"
