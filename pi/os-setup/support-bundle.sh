#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
# SPDX-License-Identifier: GPL-3.0-or-later
#
# support-bundle.sh - put what a problem report needs into one archive.
#
# On the Pi:   /opt/nullcat/current/os-setup/support-bundle.sh
# It writes ~/nullCAT-support-<date>-<time>.tar.gz holding:
#   - the logs (app.log, app_soem.log, and the previous ones, .1); a very
#     long log is cut to its last part so the archive stays small enough
#   - the settings files you have (host.json, rig.json, buttons.json,
#     profiles.json, cars.local.json, effectstatus.json, carstate.json,
#     devicepresets.json)
#   - the end of the service's journal for the last day, and the updater's
#     journal (what a failed update did)
#   - system.txt: the nullCAT build and the versions installed, the OS, the
#     kernel, temperature and throttling, the network links
# A web password in host.json is replaced with <redacted> in the copy (and
# host.json is left out altogether if the password could not be removed).
# It runs at low priority (CPU and disk) and writes only its own archive.
# Nothing is sent anywhere: copy the file off the Pi and decide who gets it.
set -u
# Everything below at low priority, the staging copies included.
renice -n 19 -p $$ >/dev/null 2>&1 || true
command -v ionice >/dev/null 2>&1 && ionice -c3 -p $$ >/dev/null 2>&1 || true

DIR="${1:-/opt/nullcat/current}"
if [ ! -x "$DIR/nullcat-pi" ]; then
    echo "No nullcat-pi in $DIR. Pass the nullCAT folder: $0 /path/to/nullcat" >&2
    exit 1
fi
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="$HOME/nullCAT-support-$STAMP.tar.gz"
# Staged on the SD card, not in /tmp (RAM on this OS).
STAGE="$(mktemp -d "$HOME/.nullcat-support.XXXXXX")" || { echo "Could not make a folder in $HOME" >&2; exit 1; }
trap 'rm -rf "$STAGE"' EXIT
NOTES=""

# Copy a file, keeping only its last $3 bytes when it is longer.
copy_tail() {
    local src="$1" dst="$2" max="$3" size
    size="$(stat -c %s "$src" 2>/dev/null || echo 0)"
    if [ "$size" -gt "$max" ]; then
        tail -c "$max" "$src" > "$dst"
        NOTES="${NOTES}note:    $(basename "$src"): only the last $((max / 1048576)) MB kept"$'\n'
    else
        cp "$src" "$dst"
    fi
}

# ---- Logs: where host.json says (logFile), else logs/app.log. The SOEM
# diagnostics log is named as the app names it: the log path up to its
# last dot, then _soem.log. Each stream keeps one older file (.1).
LOGREL="logs/app.log"
if [ -f "$DIR/host.json" ]; then
    L="$(sed -n -E 's/.*"logFile"[[:space:]]*:[[:space:]]*"([^"]+)".*/\1/p' "$DIR/host.json" | head -1)"
    [ -n "$L" ] && LOGREL="$L"
fi
LOG="$DIR/$LOGREL"
case "$LOG" in
    *.*) SOEM="${LOG%.*}_soem.log" ;;
    *)   SOEM="${LOG}_soem.log" ;;
esac
MB=1048576
[ -f "$LOG" ]      && copy_tail "$LOG"      "$STAGE/$(basename "$LOG")"      $((16 * MB))
[ -f "$LOG.1" ]    && copy_tail "$LOG.1"    "$STAGE/$(basename "$LOG").1"    $((4 * MB))
[ -f "$SOEM" ]     && copy_tail "$SOEM"     "$STAGE/$(basename "$SOEM")"     $((16 * MB))
[ -f "$SOEM.1" ]   && copy_tail "$SOEM.1"   "$STAGE/$(basename "$SOEM").1"   $((4 * MB))

# ---- Settings files (only the ones present). host.json goes in with its web
# password, if any, replaced (an escaped quote inside the password included);
# if the password still shows in the copy, host.json stays out.
if [ -f "$DIR/host.json" ]; then
    # Whole-file (a hand edit may split the key and its value over lines).
    # An empty password stays empty: a report can then show none is set.
    perl -0777 -pe 's/("webAuthToken"\s*:\s*)"(?:[^"\\]|\\.)+"/$1"<redacted>"/g' "$DIR/host.json" > "$STAGE/host.json"
    if perl -0777 -ne 'exit(/"webAuthToken"\s*:(?>\s*)(?!"<redacted>"|"")/ ? 0 : 1)' "$STAGE/host.json"; then
        rm -f "$STAGE/host.json"
        NOTES="${NOTES}note:    host.json left out (the web password could not be removed from the copy)"$'\n'
    fi
fi
for n in rig.json buttons.json profiles.json cars.local.json effectstatus.json carstate.json devicepresets.json; do
    [ -f "$DIR/$n" ] && cp "$DIR/$n" "$STAGE/"
done

# ---- Journals (need the systemd-journal or adm group, or sudo; say so if not).
journalctl -u nullcat-pi --since "-1d" -n 3000 --no-pager > "$STAGE/journal.txt" 2>&1 \
    || echo "(journalctl could not read the nullcat-pi journal)" >> "$STAGE/journal.txt"
journalctl -u 'nullcat-update@*' --since "-30d" --no-pager > "$STAGE/journal-update.txt" 2>&1 \
    || echo "(journalctl could not read the updater's journal)" >> "$STAGE/journal-update.txt"

# ---- system.txt
{
    echo "nullCAT support bundle, $(date '+%Y-%m-%d %H:%M:%S %z')"
    echo
    echo "==== nullCAT ===="
    echo "folder:  $(readlink -f "$DIR")"
    echo "version: $("$DIR/nullcat-pi" --version 2>&1 | head -1)"
    echo "service: $(systemctl is-active nullcat-pi 2>&1)"
    printf '%s' "$NOTES"
    echo "installed versions:"
    ls -l /opt/nullcat/versions 2>&1
    echo
    echo "==== OS and kernel ===="
    uname -a
    cat /etc/os-release 2>/dev/null
    # The firmware puts the board's serial and MAC address on the kernel
    # command line: kept out.
    echo "cmdline: $(sed -E 's/((macaddr|serial)=)[^ ]*/\1<removed>/g' /proc/cmdline 2>/dev/null)"
    uptime
    free -m
    echo
    echo "==== Temperature and throttling ===="
    if command -v vcgencmd >/dev/null 2>&1; then vcgencmd measure_temp; vcgencmd get_throttled; else echo "(no vcgencmd)"; fi
    echo
    echo "==== Network links ===="
    ip -br link 2>/dev/null | awk '{print $1, $2}'
    ip -br -4 addr 2>/dev/null
} > "$STAGE/system.txt" 2>&1

# The files, not the folder (a ./ entry would carry this folder's permissions).
if ( cd "$STAGE" && tar -czf "$OUT" -- * ); then
    echo "Wrote $OUT"
    echo "Copy it to your PC (for example: scp <your-user>@<your-hostname>.local:$(basename "$OUT") .)"
    echo "and send it with a short note of what happened and when."
else
    rm -f "$OUT"
    echo "Could not write $OUT (is the SD card full?)." >&2
    exit 1
fi
