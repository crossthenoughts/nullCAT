#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Script test for pi/os-setup/nullcat-update.sh: a scratch /opt layout, a
# local tarball with a stand-in nullcat-pi, no systemd. Pins:
#   1. a normal update copies every config file forward, flips the symlink,
#      leaves no .new residue and keeps the old version for rollback;
#   2. the live folder is NEVER deleted or emptied, even when the version
#      being applied already exists under that name as a symlink to it
#      (the git-built-install case): the updater refuses and the live
#      config survives untouched;
#   3. a version whose binary fails its health check is rolled back to the
#      previous one, with the live config still in place.
# Run: bash tests/pi/test-update.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UPD="$HERE/../../pi/os-setup/nullcat-update.sh"
T="$(mktemp -d)"
trap 'rm -rf "$T"' EXIT
pass=0; fail=0
ok()   { echo "[PASS] $1"; pass=$((pass+1)); }
bad()  { echo "[FAIL] $1"; fail=$((fail+1)); }
check(){ if eval "$1"; then ok "$2"; else bad "$2"; fi; }

# A release tarball for a version: the binary answers --version with
# "$1" (or fails when "$2" = broken), plus a manifest.
make_tarball() {
    local ver="$1" mode="${2:-good}"
    local name="nullCAT-v$ver-pi-aarch64"
    local dir="$T/build/$name"
    rm -rf "$T/build"; mkdir -p "$dir"
    if [ "$mode" = good ]; then
        printf '#!/usr/bin/env bash\n[ "$1" = --version ] && echo "%s"\n' "$ver" > "$dir/nullcat-pi"
    else
        printf '#!/usr/bin/env bash\nexit 1\n' > "$dir/nullcat-pi"
    fi
    chmod +x "$dir/nullcat-pi"
    printf '{ "version": "%s" }\n' "$ver" > "$dir/manifest.json"
    ( cd "$T/build" && tar -czf "$T/$name.tar.gz" "$name" )
    ( cd "$T" && sha256sum "$name.tar.gz" > "$name.tar.gz.sha256" )
    echo "$T/$name.tar.gz"
}

# A live install at $OPT/versions/v$1 with config files, current -> it.
fresh_layout() {
    OPT="$T/opt"; rm -rf "$OPT"; mkdir -p "$OPT/versions/v$1/logs"
    printf '#!/usr/bin/env bash\n[ "$1" = --version ] && echo "%s"\n' "$1" > "$OPT/versions/v$1/nullcat-pi"
    chmod +x "$OPT/versions/v$1/nullcat-pi"
    for f in host.json rig.json buttons.json carcache.json devicepresets.json effectstatus.json profiles.json cars.local.json; do
        echo "{\"file\":\"$f\",\"from\":\"v$1\"}" > "$OPT/versions/v$1/$f"
    done
    ln -sfn "$OPT/versions/v$1" "$OPT/current"
}
run_update() {   # version, expected exit code
    local rc=0
    NULLCAT_OPT="$OPT" NULLCAT_UPDATE_TARBALL="$1" NULLCAT_UPDATE_NO_SYSTEMD=1 NULLCAT_UPDATE_HEALTH_WAIT=0 \
        bash "$UPD" apply "$2" > "$T/out.txt" 2>&1 || rc=$?
    echo "$rc"
}

# ---- 1. normal update 0.9.6 -> 0.9.7 ----------------------------------------
fresh_layout 0.9.6
TB="$(make_tarball 0.9.7)"
rc="$(run_update "$TB" 0.9.7)"
check '[ "$rc" = 0 ]' "normal update exits 0"
check '[ "$(readlink -f "$OPT/current")" = "$(readlink -f "$OPT/versions/v0.9.7")" ]' "current points at v0.9.7"
check '[ "$("$OPT/current/nullcat-pi" --version)" = 0.9.7 ]' "the new binary is live"
for f in host.json rig.json buttons.json carcache.json devicepresets.json effectstatus.json profiles.json cars.local.json; do
    check "grep -q '\"from\":\"v0.9.6\"' '$OPT/versions/v0.9.7/$f'" "config copied forward: $f"
done
check '[ ! -e "$OPT/versions/v0.9.7.new" ]' "no .new residue after the swap"
check '[ -d "$OPT/versions/v0.9.6" ] && grep -q v0.9.6 "$OPT/versions/v0.9.6/rig.json"' "the old version stays for rollback, config intact"
check '[ ! -e "$OPT/staging" ]' "staging cleaned up"

# ---- 2. the live folder is the version being applied ------------------------
# A git-built install adopted as versions/v0.9.7 via a symlink to a build
# dir; current -> the build dir. Applying 0.9.7 from a tarball must refuse
# and leave every live file alone.
OPT="$T/opt"; rm -rf "$OPT"; mkdir -p "$OPT/versions" "$T/gitbuild/logs"
printf '#!/usr/bin/env bash\n[ "$1" = --version ] && echo "0.9.7"\n' > "$T/gitbuild/nullcat-pi"; chmod +x "$T/gitbuild/nullcat-pi"
echo '{"precious":true}' > "$T/gitbuild/rig.json"; echo '{"h":1}' > "$T/gitbuild/host.json"
ln -sfn "$T/gitbuild" "$OPT/versions/v0.9.7"
ln -sfn "$T/gitbuild" "$OPT/current"
TB="$(make_tarball 0.9.7)"
rc="$(run_update "$TB" 0.9.7)"
check '[ "$rc" != 0 ]' "applying the live version over itself is refused (exit $rc)"
check 'grep -q "refusing to replace the running folder" "$T/out.txt"' "...with the reason in the journal"
check 'grep -q precious "$T/gitbuild/rig.json" && [ -f "$T/gitbuild/host.json" ] && [ -x "$T/gitbuild/nullcat-pi" ]' "the live folder and its config are untouched"
check '[ "$(readlink -f "$OPT/current")" = "$(readlink -f "$T/gitbuild")" ]' "current still points at the live install"
check '[ ! -e "$OPT/versions/v0.9.7.new" ]' "no half-built tree left behind"

# ---- 3. a broken version rolls back ------------------------------------------
fresh_layout 0.9.6
TB="$(make_tarball 0.9.8 broken)"
rc="$(run_update "$TB" 0.9.8)"
check '[ "$rc" != 0 ]' "a version failing its health check exits nonzero"
check '[ "$(readlink -f "$OPT/current")" = "$(readlink -f "$OPT/versions/v0.9.6")" ]' "rolled back to v0.9.6"
check 'grep -q v0.9.6 "$OPT/versions/v0.9.6/rig.json"' "the rollback target still has its config"
check '[ -d "$OPT/versions/v0.9.8" ]' "the failed version is left for inspection"

echo
echo "test-update: $pass passed, $fail failed"
[ "$fail" = 0 ]
