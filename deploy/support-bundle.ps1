# SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
# SPDX-License-Identifier: GPL-3.0-or-later
#
# support-bundle.ps1 - put what a problem report needs into one zip.
#
# Double-click support-bundle.cmd in the nullCAT folder (it runs this script
# past Windows' default script policy and keeps the window open), or in a
# PowerShell window there:
#   powershell -ExecutionPolicy Bypass -File .\support-bundle.ps1
#
# It writes nullCAT-support-<date>-<time>.zip next to nullCAT.exe, holding:
#   - the logs (app.log, app_soem.log, and the previous ones, .1); a very
#     long log is cut to its last part so the zip stays small enough to send
#   - your settings files (host.json, rig.json, buttons.json, profiles.json,
#     cars.local.json, effectstatus.json, carstate.json, devicepresets.json),
#     the ones you have
#   - system.txt: this nullCAT build, Windows, the CPU, the network adapters
#     and the EtherCAT adapter's settings, the Npcap version
# A web password in host.json is replaced with <redacted> in the copy (and
# host.json is left out altogether if the password could not be removed).
# Nothing is sent anywhere: you decide who gets the zip.

param([string]$Folder = $PSScriptRoot)

$ErrorActionPreference = 'Continue'
if (-not $Folder) { $Folder = (Get-Location).Path }
if (-not (Test-Path -LiteralPath (Join-Path $Folder 'nullCAT.exe'))) {
    Write-Host "nullCAT.exe is not in $Folder. Run this from the nullCAT folder (or pass -Folder <path>)." -ForegroundColor Yellow
    exit 1
}

$utf8 = New-Object System.Text.UTF8Encoding $false   # what nullCAT writes: UTF-8, no BOM
function Read-Text([string]$p) { [System.IO.File]::ReadAllText($p, $utf8) }
function Write-Text([string]$p, [string]$t) { [System.IO.File]::WriteAllText($p, $t, $utf8) }

# Copy a file, keeping only its last $max bytes when it is longer (the zip
# has to fit an email or a forum post).
function Copy-Tail([string]$src, [string]$dst, [long]$max) {
    # The app may be writing it, or rotating it (a rename): share both.
    $fs = [System.IO.File]::Open($src, 'Open', 'Read', [System.IO.FileShare]'ReadWrite, Delete')
    try {
        $len = $fs.Length
        $skip = [Math]::Max([long]0, $len - $max)
        $fs.Seek($skip, 'Begin') | Out-Null
        $out = [System.IO.File]::Create($dst)
        try { $fs.CopyTo($out) } finally { $out.Close() }
        return $skip
    } finally { $fs.Close() }
}

$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$stage = Join-Path $env:TEMP "nullCAT-support-$stamp"
New-Item -ItemType Directory -Force -Path $stage | Out-Null
$notes = New-Object System.Collections.Generic.List[string]

# ---- Logs: where host.json says (logFile), else logs\app.log. The SOEM
# diagnostics log is named as the app names it: the log path up to its
# last dot, then _soem.log. Each stream keeps one older file (.1).
$hostPath = Join-Path $Folder 'host.json'
$hostText = if (Test-Path -LiteralPath $hostPath) { Read-Text $hostPath } else { $null }
$logRel = 'logs/app.log'
if ($hostText) {
    $m = [regex]::Match($hostText, '"logFile"\s*:\s*"([^"]+)"')
    if ($m.Success) { $logRel = $m.Groups[1].Value }
}
$logPath = Join-Path $Folder ($logRel -replace '/', '\')
$dot = $logPath.LastIndexOf('.')
$soemPath = if ($dot -ge 0) { $logPath.Substring(0, $dot) + '_soem.log' } else { $logPath + '_soem.log' }
$caps = @(@($logPath, 16MB), @("$logPath.1", 4MB), @($soemPath, 16MB), @("$soemPath.1", 4MB))
foreach ($c in $caps) {
    $f = $c[0]
    if (-not (Test-Path -LiteralPath $f)) { continue }
    try {
        $cut = Copy-Tail $f (Join-Path $stage (Split-Path $f -Leaf)) $c[1]
        if ($cut -gt 0) { $notes.Add("$(Split-Path $f -Leaf): only the last $([math]::Round($c[1] / 1MB)) MB kept (the first $([math]::Round($cut / 1MB, 1)) MB left out)") }
    } catch { $notes.Add("$(Split-Path $f -Leaf): could not be read ($($_.Exception.Message))") }
}

# ---- Settings files (only the ones present). host.json goes in with its web
# password, if any, replaced (an escaped quote inside the password included);
# if the password still shows in the copy, host.json stays out.
if ($hostText) {
    # An empty password stays empty: a report can then show none is set.
    $red = [regex]::Replace($hostText, '("webAuthToken"\s*:\s*)"(?:[^"\\]|\\.)+"', '$1"<redacted>"')
    if ([regex]::IsMatch($red, '"webAuthToken"\s*:(?>\s*)(?!"<redacted>"|"")')) {
        $notes.Add('host.json: left out (the web password could not be removed from the copy)')
    } else {
        Write-Text (Join-Path $stage 'host.json') $red
    }
}
foreach ($name in @('rig.json', 'buttons.json', 'profiles.json', 'cars.local.json',
                    'effectstatus.json', 'carstate.json', 'devicepresets.json')) {
    $p = Join-Path $Folder $name
    if (Test-Path -LiteralPath $p) { Copy-Item -LiteralPath $p -Destination $stage }
}

# ---- system.txt
$sys = New-Object System.Collections.Generic.List[string]
function Add-Section([string]$title) { $sys.Add(''); $sys.Add("==== $title ====") }
$sys.Add("nullCAT support bundle, $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss zzz')")

Add-Section 'nullCAT'
$exe = Get-Item -LiteralPath (Join-Path $Folder 'nullCAT.exe')
$sys.Add("folder:   $Folder")
# The version, from the last start the log recorded (0.9.9 and later log it).
$ver = $null
foreach ($f in @($logPath, "$logPath.1")) {
    if ($ver -or -not (Test-Path -LiteralPath $f)) { continue }
    $hit = Select-String -LiteralPath $f -Pattern 'nullCAT (\S+) starting' | Select-Object -Last 1
    if ($hit) { $ver = $hit.Matches[0].Groups[1].Value }
}
if ($ver) { $sys.Add("version:  $ver (from the log)") } else { $sys.Add('version:  not in the log (versions before 0.9.9 do not record it; the folder name usually does)') }
$sys.Add("exe:      $($exe.Length) bytes, dated $($exe.LastWriteTime.ToString('yyyy-MM-dd HH:mm:ss'))")
try { $sys.Add("sha256:   $((Get-FileHash -LiteralPath $exe.FullName -Algorithm SHA256).Hash)") } catch { }
foreach ($n in $notes) { $sys.Add("note:     $n") }

Add-Section 'Windows and CPU'
try {
    $os = Get-CimInstance Win32_OperatingSystem
    $sys.Add("$($os.Caption) $($os.Version) (build $($os.BuildNumber)), $([math]::Round($os.TotalVisibleMemorySize / 1MB, 1)) GB RAM")
    foreach ($c in Get-CimInstance Win32_Processor) { $sys.Add("$($c.Name.Trim()), $($c.NumberOfCores) cores, $($c.NumberOfLogicalProcessors) threads") }
} catch { $sys.Add("(could not read: $($_.Exception.Message))") }

Add-Section 'Network adapters'
try {
    foreach ($a in Get-NetAdapter | Sort-Object Name) {
        $sys.Add(("{0,-24} {1,-50} {2,-12} {3}" -f $a.Name, $a.InterfaceDescription, $a.Status, $a.LinkSpeed))
    }
} catch { $sys.Add("(could not read: $($_.Exception.Message))") }

# The EtherCAT adapter named in host.json: nicName is its description, its
# name, or a \Device\NPF_{GUID} path.
if ($hostText) {
    $nic = [regex]::Match($hostText, '"nicName"\s*:\s*"([^"]+)"')
    if ($nic.Success) {
        $want = $nic.Groups[1].Value -replace '\\\\', '\'
        Add-Section "EtherCAT adapter: $want"
        try {
            $guid = [regex]::Match($want, '\{[0-9A-Fa-f-]+\}').Value
            $ad = Get-NetAdapter | Where-Object {
                $_.InterfaceDescription -eq $want -or $_.Name -eq $want -or ($guid -and $_.InterfaceGuid -eq $guid)
            } | Select-Object -First 1
            if ($ad) {
                foreach ($p in Get-NetAdapterAdvancedProperty -Name $ad.Name -ErrorAction Stop | Sort-Object DisplayName) {
                    $sys.Add(("{0,-40} {1}" -f $p.DisplayName, $p.DisplayValue))
                }
            } else { $sys.Add('(no adapter with that name, description or id)') }
        } catch { $sys.Add("(could not read: $($_.Exception.Message))") }
    }
}

Add-Section 'Npcap'
$npcapSys = Join-Path $env:SystemRoot 'System32\drivers\npcap.sys'
if (Test-Path -LiteralPath $npcapSys) { $sys.Add("npcap.sys $((Get-Item -LiteralPath $npcapSys).VersionInfo.FileVersion)") } else { $sys.Add('npcap.sys not found (is Npcap installed?)') }
foreach ($k in @('HKLM:\SOFTWARE\WOW6432Node\Npcap', 'HKLM:\SOFTWARE\Npcap')) {
    try { $v = Get-ItemProperty $k -ErrorAction Stop; $sys.Add("$k : $(($v.PSObject.Properties | Where-Object { $_.Name -notlike 'PS*' } | ForEach-Object { "$($_.Name)=$($_.Value)" }) -join '; ')") } catch { }
}

Write-Text (Join-Path $stage 'system.txt') (($sys -join "`r`n") + "`r`n")

# ---- The zip (flat: no folders inside, so any unzip tool reads it alike).
$zip = Join-Path $Folder "nullCAT-support-$stamp.zip"
$zipped = $false
try {
    # .NET's zip writer takes both paths literally (Compress-Archive reads
    # brackets in a folder name as wildcards and fails).
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    if (Test-Path -LiteralPath $zip) { Remove-Item -LiteralPath $zip -Force }
    [System.IO.Compression.ZipFile]::CreateFromDirectory($stage, $zip)
    $zipped = $true
} catch {
    Write-Host "Could not write $zip ($($_.Exception.Message))." -ForegroundColor Yellow
    # A zip that failed part way is broken: never leave it to be sent.
    if (Test-Path -LiteralPath $zip) { Remove-Item -LiteralPath $zip -Force -ErrorAction SilentlyContinue }
}
Remove-Item -LiteralPath $stage -Recurse -Force
if ($zipped) {
    Write-Host "Wrote $zip" -ForegroundColor Green
    Write-Host 'Send that file with a short note of what happened and when.'
} else {
    Write-Host 'No zip was written. If the nullCAT folder needs administrator rights to write to, right-click support-bundle.cmd and choose Run as administrator.' -ForegroundColor Yellow
    exit 1
}
