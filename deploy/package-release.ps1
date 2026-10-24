# SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
# SPDX-License-Identifier: GPL-3.0-or-later
#
# package-release.ps1 - build a clean, shippable nullCAT Windows release zip.
#
# Produces dist\nullCAT-v<version>-win64.zip from a FRESH build directory, so
# no stale artifacts (old exes, test binaries, machine-local host.json/rig.json/
# buttons.json, scratch files) can ride along. Qt runtime is deployed with
# windeployqt automatically. The SimHub plugin is built from the same tree and
# ships in the zip's simhub folder (DLL, annotated settings, README; host
# 127.0.0.1), and also as dist\nullCAT-v<version>-simhub-plugin-pi.zip for Pi
# rigs (host 192.168.50.1, the Pi's telemetry port). Building it needs the
# .NET SDK and SimHub installed (its DLLs are the references).
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

# ---- 2b. The SimHub plugin, built fresh from this tree (needs the .NET SDK
# and SimHub installed: its DLLs are the build references). It ships with
# its annotated settings file and README, in the zip's simhub folder and as
# a zip of its own for Pi rigs, whose SimHub runs on the game PC. ----
$plugDir = Join-Path $repo "integrations\simhub\NullcatChannelExporter"
foreach ($d in @("bin", "obj")) { if (Test-Path (Join-Path $plugDir $d)) { Remove-Item -Recurse -Force (Join-Path $plugDir $d) } }
Invoke-Tool "SimHub plugin" "cd /d `"$plugDir`" && dotnet build -c Release -nologo"
$plugDll   = Join-Path $plugDir "bin\Release\net48\NullcatChannelExporter.dll"
$plugFiles = @($plugDll, (Join-Path $plugDir "NullcatChannelExporter.json"), (Join-Path $plugDir "README.md"))
foreach ($f in $plugFiles) { if (-not (Test-Path $f)) { throw "SimHub plugin file missing: $f" } }

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
Copy-Item "$rel\cars.json"              $stage
Copy-Item "$rel\presets.json"           $stage
Copy-Item "$rel\LICENSE.txt"            $stage
Copy-Item "$rel\THIRD_PARTY_NOTICES.md" $stage
Copy-Item "$rel\SAFETY.md"              $stage
Copy-Item "$rel\README.txt"             $stage
Copy-Item "$rel\install-task.ps1"       $stage
Copy-Item "$rel\uninstall-task.ps1"     $stage
Copy-Item "$rel\support-bundle.ps1"     $stage
Copy-Item "$rel\support-bundle.cmd"     $stage
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
New-Item -ItemType Directory -Force (Join-Path $stage "simhub") | Out-Null
foreach ($f in $plugFiles) { Copy-Item $f (Join-Path $stage "simhub") }

# ---- 4. Qt runtime via windeployqt (into the STAGE dir, release-only) ----
Invoke-Tool "windeployqt" ("`"$QtDir\bin\windeployqt.exe`" --release --no-translations " +
    "--no-system-d3d-compiler --no-opengl-sw `"$stage\nullCAT.exe`"")

# ---- 5. Sanity check: nothing machine-local or stale slipped in ----
$forbidden = @("host.json", "rig.json", "buttons.json", "cars.local.json", "carcache.json", "carstate.json", "cardefaults.txt", "profiles.json", "effectstatus.json", "devicepresets.json",
               "config.json") + (Get-ChildItem $stage -Filter "Test*.exe")
foreach ($f in $forbidden) {
    if (Test-Path (Join-Path $stage "$f")) { throw "Forbidden file staged: $f" }
}
$mustHave = @("nullCAT.exe", "nullCATWatchdog.exe", "LICENSE.txt", "SAFETY.md",
              "THIRD_PARTY_NOTICES.md", "web\index.html", "Qt6Core.dll",
              "platforms\qwindows.dll", "host.reference.json", "rig.reference.json",
              "cars.json", "presets.json", "KNOWN_LIMITATIONS.md", "docs\FIRST_SETUP_WINDOWS.md",
              "docs\media\simhub-update-command.png", "docs\CONFIG_REFERENCE.md",
              "simhub\NullcatChannelExporter.dll", "simhub\NullcatChannelExporter.json", "simhub\README.md",
              "support-bundle.ps1", "support-bundle.cmd")
foreach ($f in $mustHave) {
    if (-not (Test-Path (Join-Path $stage $f))) { throw "Missing from stage: $f" }
}

# ---- 6. Zip ----
# NOT Compress-Archive: on Windows PowerShell it writes BACKSLASH path
# separators, which the ZIP spec (APPNOTE 4.4.17) forbids -- forward slashes
# are required. Windows Explorer and 7-Zip cope, but some non-Windows tools
# create files with literal backslashes in the name instead of directories.
# ZipFile::CreateFromDirectory writes conformant entries.
Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem
# Entry names are written explicitly with '/'. Neither Compress-Archive nor
# ZipFile::CreateFromDirectory can be trusted here: on .NET Framework (which
# is what Windows PowerShell 5.1 runs) both use Path.DirectorySeparatorChar,
# so both emit backslashes. Only .NET Core+ normalises them.
function Write-ConformantZip([string]$From, [string]$To) {
    if (Test-Path $To) { Remove-Item -Force $To }
    $fs = [IO.File]::Open($To, [IO.FileMode]::Create)
    try {
        $archive = New-Object IO.Compression.ZipArchive($fs, [IO.Compression.ZipArchiveMode]::Create)
        try {
            foreach ($f in (Get-ChildItem $From -Recurse -File)) {
                $rel = $f.FullName.Substring($From.Length + 1).Replace('\', '/')
                $entry  = $archive.CreateEntry($rel, [IO.Compression.CompressionLevel]::Optimal)
                $out    = $entry.Open()
                $in     = [IO.File]::OpenRead($f.FullName)
                try { $in.CopyTo($out) } finally { $in.Dispose(); $out.Dispose() }
            }
        } finally { $archive.Dispose() }
    } finally { $fs.Dispose() }
    # Fail the build rather than ship a non-conformant archive.
    $zf = [IO.Compression.ZipFile]::OpenRead($To)
    $bad = @($zf.Entries | Where-Object { $_.FullName -match '\\' }).Count
    $entries = $zf.Entries.Count
    $zf.Dispose()
    if ($bad -gt 0) { throw "$bad zip entries use backslash separators (ZIP spec requires '/')" }
    Write-Host "-- archive $(Split-Path -Leaf $To): $entries entries, all separators conformant"
}
Write-ConformantZip $stage $zip

# The SimHub plugin on its own for a Pi rig (the DLL, its annotated settings,
# the README), its settings pointed at the Pi's telemetry port (192.168.50.1,
# Docs/PI_SETUP.md) instead of 127.0.0.1, which the Windows zip's copy keeps
# (SimHub and nullCAT on one PC).
$plugStage = Join-Path $dist "nullCAT-v$Version-simhub-plugin-pi"
if (Test-Path $plugStage) { Remove-Item -Recurse -Force $plugStage }
New-Item -ItemType Directory -Force $plugStage | Out-Null
foreach ($f in $plugFiles) { Copy-Item $f $plugStage }
$plugJson = Join-Path $plugStage "NullcatChannelExporter.json"
$jsonText = [IO.File]::ReadAllText($plugJson)
if (-not $jsonText.Contains('"host": "127.0.0.1"')) { throw "SimHub plugin settings: the default host line has changed; update the Pi preset step" }
[IO.File]::WriteAllText($plugJson, $jsonText.Replace('"host": "127.0.0.1"', '"host": "192.168.50.1"'), (New-Object Text.UTF8Encoding($false)))
$plugZip = "$plugStage.zip"
Write-ConformantZip $plugStage $plugZip

$size = [math]::Round((Get-Item $zip).Length / 1MB, 1)
Write-Host "== DONE: $zip ($size MB) ==" -ForegroundColor Green
Write-Host "== DONE: $plugZip (the SimHub plugin for a Pi rig, host 192.168.50.1) ==" -ForegroundColor Green
Write-Host "Contents staged in $stage for inspection."
Write-Host "Reminder: Npcap is a user-installed prerequisite (cannot ship in the zip)."
