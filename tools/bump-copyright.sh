#!/bin/sh
# Bump the copyright year range in every SPDX-FileCopyrightText header and
# in README.md. The notice is templated HERE - this script is the single
# place that knows the canonical string; never hand-edit 77 headers.
#
# Usage:  tools/bump-copyright.sh 2026-2027
# (First arg is the new year or range; the holder string stays fixed.
#  ASCII-fallback contexts elsewhere use "Zero Werks" for "Ø Werks".)

set -e
[ -n "$1" ] || { echo "usage: $0 <year|year-range>  e.g. $0 2026-2027" >&2; exit 1; }
YEARS="$1"
HOLDER="Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>"
cd "$(git rev-parse --show-toplevel)"

# Source headers (SPDX-FileCopyrightText: <years> <holder>)
git ls-files '*.cpp' '*.h' 'web/app.js' | while read -r f; do
    case "$f" in src/httplib.h) continue ;; esac
    sed -i "s|^// SPDX-FileCopyrightText: [0-9-]* Tim Palmgren.*|// SPDX-FileCopyrightText: ${YEARS} ${HOLDER}|" "$f"
done

# README notice (HTML-escaped angle brackets)
sed -i "s|^Copyright © [0-9-]* Tim Palmgren.*|Copyright © ${YEARS} Tim Palmgren (Ø Werks) \&lt;tim@zerowerks.co.nz\&gt;|" README.md

echo "Copyright bumped to ${YEARS}:"
git diff --stat | tail -1
