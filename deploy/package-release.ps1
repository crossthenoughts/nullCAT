# SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
# SPDX-License-Identifier: GPL-3.0-or-later
#
# package-release.ps1 - build a clean, shippable nullCAT Windows release zip.
#
# Produces dist\nullCAT-v<version>-win64.zip from a FRESH build directory, so
# no stale artifacts (old exes, test binaries, machine-local host.json/rig.json/
# buttons.json, scratch files) can ride along. Qt runtime is deployed with
# windeployqt automatically.
#
# Usage (from a "x64 Native Tools Command Prompt for VS 2022" PowerShell, or any
# shell where cmake + your Qt install are reachable):
#   .\deploy\package-release.ps1 -Version 0.9.0 `
#       -QtDir  "C:\Qt\6.10.2\msvc2022_64" `
#       -SoemRoot "C:\libs\SOEM" `
#       [-NpcapSdk "C:\libs\npcap-sdk"]
#
# The build directory (build-package) is separate from any dev build dir and is
# recreated from scratch every run.

param(
    [Parameter(Mandatory=$true)][string]$Version,
    [Parameter(Mandatory=$true)][string]$QtDir,
    [Parameter(Mandatory=$true)][string]$SoemRoot,
    [string]$NpcapSdk = "C:/libs/npcap-sdk"
)
$ErrorActionPreference = "Stop"
$repo  = Split-Path -Parent $PSScriptRoot          # deploy\.. = repo root
$build = Join-Path $repo "build-package"
$dist  = Join-Path $repo "dist"
$stage = Join-Path $dist "nullCAT-v$Version-win64"
$zip   = "$stage.zip"

# Native tools write warnings to stderr, which Windows PowerShell 5.1 turns
# into terminating errors under ErrorActionPreference=Stop. Run them through
# cmd /c with stderr folded into stdout and check exit codes explicitly.
function Invoke-Tool([string]$What, [string]$CommandLine) {
    Write-Host "-- $What" -ForegroundColor DarkCyan
    cmd /c "$CommandLine 2>&1"
    if ($LASTEXITCODE -ne 0) { throw "$What failed (exit $LASTEXITCODE)" }
}

Write-Host "== nullCAT release packaging v$Version ==" -ForegroundColor Cyan

# ---- 1. Fresh configure + build (never reuse a dev build dir) ----
if (Test-Path $build) { Remove-Item -Recurse -Force $build }
Invoke-Tool "CMake configure" ("cmake -S `"$repo`" -B `"$build`" -G `"Visual Studio 17 2022`" -A x64 " +
    "-DQt6_DIR=`"$QtDir/lib/cmake/Qt6`" -DSOEM_ROOT=`"$SoemRoot`" -DNPCAP_SDK=`"$NpcapSdk`"")
Invoke-Tool "Build" "cmake --build `"$build`" --config Release"

# ---- 2. Test gate: a release zip is never cut from a red suite ----
Invoke-Tool "Test gate" "cd /d `"$build`" && ctest -C Release --output-on-failure"

# ---- 3. Stage the ALLOWLIST (nothing else ships) ----
if (Test-Path $stage) { Remove-Item -Recurse -Force $stage }
New-Item -ItemType Directory -Force $stage | Out-Null
$rel = Join-Path $build "Release"

# Application + watchdog
Copy-Item "$rel\nullCAT.exe"          $stage
Copy-Item "$rel\nullCATWatchdog.exe"  $stage
# MSVC runtime (bundled app-local by the post-build step)
Copy-Item "$rel\msvcp*.dll"       $stage
Copy-Item "$rel\vcruntime*.dll"   $stage
Copy-Item "$rel\concrt*.dll"      $stage -ErrorAction SilentlyContinue
# Web UI, reference configs, license/safety texts, deploy scripts
Copy-Item "$rel\web"                    $stage -Recurse
Copy-Item "$rel\host.reference.json"    $stage
Copy-Item "$rel\rig.reference.json"     $stage
Copy-Item "$rel\LICENSE.txt"            $stage
Copy-Item "$rel\THIRD_PARTY_NOTICES.md" $stage
Copy-Item "$rel\SAFETY.md"              $stage
Copy-Item "$rel\README.txt"             $stage
Copy-Item "$rel\install-task.ps1"       $stage
Copy-Item "$rel\uninstall-task.ps1"     $stage
Copy-Item "$rel\host.nuc.example.json"  $stage
New-Item -ItemType Directory -Force (Join-Path $stage "logs") | Out-Null

# User documentation, readable offline. docs\ carries the walkthrough (with
# its screenshots) and the reference docs; KNOWN_LIMITATIONS.md and
# BUILD_INSTRUCTIONS.md sit at the root so the docs' relative ../ links
# resolve exactly as they do on GitHub (SAFETY.md is already there).
Copy-Item "$repo\KNOWN_LIMITATIONS.md"  $stage
Copy-Item "$repo\BUILD_INSTRUCTIONS.md" $stage
New-Item -ItemType Directory -Force (Join-Path $stage "docs\media") | Out-Null
Copy-Item "$repo\Docs\*.md"                    (Join-Path $stage "docs")
Copy-Item "$repo\Docs\media\simhub-*.png"      (Join-Path $stage "docs\media")

# ---- 4. Qt runtime via windeployqt (into the STAGE dir, release-only) ----
Invoke-Tool "windeployqt" ("`"$QtDir\bin\windeployqt.exe`" --release --no-translations " +
    "--no-system-d3d-compiler --no-opengl-sw `"$stage\nullCAT.exe`"")

# ---- 5. Sanity check: nothing machine-local or stale slipped in ----
$forbidden = @("host.json", "rig.json", "buttons.json",
               "config.json") + (Get-ChildItem $stage -Filter "Test*.exe")
foreach ($f in $forbidden) {
    if (Test-Path (Join-Path $stage "$f")) { throw "Forbidden file staged: $f" }
}
$mustHave = @("nullCAT.exe", "nullCATWatchdog.exe", "LICENSE.txt", "SAFETY.md",
              "THIRD_PARTY_NOTICES.md", "web\index.html", "Qt6Core.dll",
              "platforms\qwindows.dll", "host.reference.json", "rig.reference.json",
              "KNOWN_LIMITATIONS.md", "docs\FIRST_SETUP_WINDOWS.md",
              "docs\media\simhub-update-command.png", "docs\CONFIG_REFERENCE.md")
foreach ($f in $mustHave) {
    if (-not (Test-Path (Join-Path $stage $f))) { throw "Missing from stage: $f" }
}

# ---- 6. Zip ----
# NOT Compress-Archive: on Windows PowerShell it writes BACKSLASH path
# separators, which the ZIP spec (APPNOTE 4.4.17) forbids -- forward slashes
# are required. Windows Explorer and 7-Zip cope, but some non-Windows tools
# create files with literal backslashes in the name instead of directories.
# ZipFile::CreateFromDirectory writes conformant entries.
if (Test-Path $zip) { Remove-Item -Force $zip }
Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem
# Entry names are written explicitly with '/'. Neither Compress-Archive nor
# ZipFile::CreateFromDirectory can be trusted here: on .NET Framework (which
# is what Windows PowerShell 5.1 runs) both use Path.DirectorySeparatorChar,
# so both emit backslashes. Only .NET Core+ normalises them.
$fs = [IO.File]::Open($zip, [IO.FileMode]::Create)
try {
    $archive = New-Object IO.Compression.ZipArchive($fs, [IO.Compression.ZipArchiveMode]::Create)
    try {
        foreach ($f in (Get-ChildItem $stage -Recurse -File)) {
            $rel = $f.FullName.Substring($stage.Length + 1).Replace('\', '/')
            $entry  = $archive.CreateEntry($rel, [IO.Compression.CompressionLevel]::Optimal)
            $out    = $entry.Open()
            $in     = [IO.File]::OpenRead($f.FullName)
            try { $in.CopyTo($out) } finally { $in.Dispose(); $out.Dispose() }
        }
    } finally { $archive.Dispose() }
} finally { $fs.Dispose() }

# Fail the build rather than ship a non-conformant archive.
$zf = [IO.Compression.ZipFile]::OpenRead($zip)
$bad = @($zf.Entries | Where-Object { $_.FullName -match '\\' }).Count
$entries = $zf.Entries.Count
$zf.Dispose()
if ($bad -gt 0) { throw "$bad zip entries use backslash separators (ZIP spec requires '/')" }
Write-Host "-- archive: $entries entries, all separators conformant"

$size = [math]::Round((Get-Item $zip).Length / 1MB, 1)
Write-Host "== DONE: $zip ($size MB) ==" -ForegroundColor Green
Write-Host "Contents staged in $stage for inspection."
Write-Host "Reminder: Npcap is a user-installed prerequisite (cannot ship in the zip)."
