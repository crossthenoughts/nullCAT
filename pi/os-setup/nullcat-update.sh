#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
# SPDX-License-Identifier: GPL-3.0-or-later
#
# nullcat-update.sh - apply a released nullCAT version on the Pi.
#
# Runs as root via nullcat-update@<version>.service (the daemon's
# /api/update/start launches it; manual use: sudo systemctl start
# nullcat-update@0.9.6). Every step is narrated to the journal:
# journalctl -u nullcat-update@<version>.
#
# Layout (created by install.sh >= 0.9.5):
#   /opt/nullcat/versions/vX.Y.Z/   one directory per version; the LIVE
#                                   config (host/rig/buttons.json) lives
#                                   inside each and is copied forward
#   /opt/nullcat/current            symlink to the running version
#   /opt/nullcat/staging/           downloads and unpack scratch
#
# Flow: download release assets -> verify sha256 -> unpack -> manifest
# gate -> copy config forward -> prune to 3 versions -> atomic symlink
# swap -> restart -> health check -> on failure walk back current ->
# prev -> prev2 (two rollback generations, so a bad rollback target has
# one more level behind it).

set -euo pipefail

OPT=/opt/nullcat
REPO_SLUG="crossthenoughts/nullCAT"
SERVICE=nullcat-pi
KEEP_VERSIONS=3          # the new one + two rollback generations

say()  { echo "nullcat-update: $*"; }
fail() { echo "nullcat-update: ERROR: $*" >&2; exit 1; }

CMD="${1:-}"
VERSION="${2:-}"
[ "$CMD" = "apply" ] || fail "usage: nullcat-update.sh apply <version>"
[[ "$VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || fail "bad version string '$VERSION'"
[ -L "$OPT/current" ] || fail "$OPT/current missing -- run install.sh once to adopt the versioned layout"

CUR_TARGET="$(readlink -f "$OPT/current")"
CUR_VER="$(basename "$CUR_TARGET")"
say "current $CUR_VER, applying v$VERSION"
[ "$CUR_VER" = "v$VERSION" ] && { say "already on v$VERSION -- nothing to do"; exit 0; }

# ---- 1. Download the release assets ----------------------------------------
NAME="nullCAT-v${VERSION}-pi-aarch64"
STAGING="$OPT/staging"
rm -rf "$STAGING"; mkdir -p "$STAGING"
API="https://api.github.com/repos/${REPO_SLUG}/releases/tags/v${VERSION}"
say "querying $API"
JSON="$(curl -sfL "$API")" || fail "release v$VERSION not found on GitHub (offline, or not released yet)"
for f in "$NAME.tar.gz" "$NAME.tar.gz.sha256"; do
    URL="$(echo "$JSON" | grep -oE "\"browser_download_url\": ?\"[^\"]*${f}\"" | grep -oE 'https[^"]*')"
    [ -n "$URL" ] || fail "asset $f not on the release (the pi-release workflow attaches it a few minutes after publish -- try again shortly)"
    say "downloading $f"
    curl -sfL -o "$STAGING/$f" "$URL" || fail "download failed: $f"
done

# ---- 2. Verify + unpack -----------------------------------------------------
say "verifying checksum"
( cd "$STAGING" && sha256sum -c "$NAME.tar.gz.sha256" >/dev/null ) || fail "sha256 MISMATCH -- refusing the tarball"
tar -xzf "$STAGING/$NAME.tar.gz" -C "$STAGING"
[ -x "$STAGING/$NAME/nullcat-pi" ] || fail "tarball has no nullcat-pi"

# ---- 3. Manifest gate -------------------------------------------------------
MANIFEST="$STAGING/$NAME/manifest.json"
[ -f "$MANIFEST" ] || fail "no manifest.json in the tarball"
grep -q "\"version\": \"$VERSION\"" "$MANIFEST" || fail "manifest version does not match $VERSION"
if grep -q '"requiresFullReinstall": true' "$MANIFEST"; then
    fail "v$VERSION changes OS-level setup and needs the full installer: git pull && ./pi/os-setup/install.sh (see the release notes)"
fi

# ---- 4. Install the version dir + copy config forward ----------------------
NEW="$OPT/versions/v$VERSION"
rm -rf "$NEW"
mkdir -p "$OPT/versions"
mv "$STAGING/$NAME" "$NEW"
for f in host.json rig.json buttons.json carcache.json devicepresets.json; do
    [ -f "$CUR_TARGET/$f" ] && cp -p "$CUR_TARGET/$f" "$NEW/$f" && say "config copied forward: $f"
done
mkdir -p "$NEW/logs"
OWNER="$(stat -c %U "$CUR_TARGET")"
chown -R "$OWNER:$OWNER" "$NEW"

# ---- 5. Prune old versions (keep the newest KEEP_VERSIONS, never current/new)
( cd "$OPT/versions" && ls -1d v* 2>/dev/null | sort -V | head -n -"$KEEP_VERSIONS" ) | while read -r old; do
    [ "$OPT/versions/$old" = "$CUR_TARGET" ] && continue
    [ "$old" = "v$VERSION" ] && continue
    say "pruning old version $old"
    rm -rf "${OPT:?}/versions/$old"
done

# ---- 6. Atomic swap + restart ----------------------------------------------
health_ok() {
    systemctl is-active --quiet "$SERVICE" || return 1
    local v; v="$("$OPT/current/nullcat-pi" --version 2>/dev/null)" || return 1
    [ "$v" = "$1" ]
}
swap_to() {
    ln -sfn "$1" "$OPT/current.new" && mv -Tf "$OPT/current.new" "$OPT/current"
    say "current -> $(basename "$1"), restarting $SERVICE"
    systemctl restart "$SERVICE"
}

swap_to "$NEW"
say "health check (up to 30s)..."
for _ in $(seq 1 15); do sleep 2; health_ok "$VERSION" && { say "v$VERSION healthy -- update complete"; rm -rf "$STAGING"; exit 0; }; done

# ---- 7. Rollback ladder: current -> prev -> prev2 ---------------------------
say "v$VERSION FAILED its health check -- walking back"
mapfile -t CANDIDATES < <(cd "$OPT/versions" && ls -1d v* | sort -rV | grep -vx "v$VERSION")
for c in "${CANDIDATES[@]:0:2}"; do
    swap_to "$OPT/versions/$c"
    for _ in $(seq 1 8); do sleep 2; health_ok "${c#v}" && { say "rolled back to $c -- v$VERSION left in versions/ for inspection"; exit 1; }; done
    say "$c also failed its health check"
done
systemctl stop "$SERVICE" || true
fail "no version passed the health check -- service stopped; inspect journalctl -u $SERVICE and $OPT/versions"
