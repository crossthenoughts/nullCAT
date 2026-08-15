# ============================================================
# install-task.ps1  (Build 71)
#
# Creates a Windows Task Scheduler task that runs
# nullCATWatchdog.exe with administrator privileges.
# After install, users launch via the desktop shortcut - no
# UAC prompt required.
#
# USAGE (run once as Administrator):
#   Right-click install-task.ps1 → Run as administrator
# - or - #   Start-Process powershell -Verb RunAs -ArgumentList "-File install-task.ps1"
# ============================================================

$taskName   = "nullCAT"
$scriptDir  = Split-Path -Parent $MyInvocation.MyCommand.Definition
$watchdog   = Join-Path $scriptDir "nullCATWatchdog.exe"

if (-not (Test-Path $watchdog)) {
    Write-Error "nullCATWatchdog.exe not found at: $watchdog"
    exit 1
}

# Remove existing task silently
Unregister-ScheduledTask -TaskName $taskName -Confirm:$false -ErrorAction SilentlyContinue

$action    = New-ScheduledTaskAction -Execute $watchdog -WorkingDirectory $scriptDir
$settings  = New-ScheduledTaskSettingsSet `
    -AllowStartIfOnBatteries `
    -DontStopIfGoingOnBatteries `
    -ExecutionTimeLimit 0 `
    -MultipleInstances IgnoreNew

# Run as current user with highest privileges (admin, no UAC prompt after setup)
$principal = New-ScheduledTaskPrincipal `
    -UserId ([System.Security.Principal.WindowsIdentity]::GetCurrent().Name) `
    -LogonType Interactive `
    -RunLevel Highest

Register-ScheduledTask `
    -TaskName  $taskName `
    -Action    $action `
    -Settings  $settings `
    -Principal $principal `
    -Force | Out-Null

Write-Host "Task '$taskName' registered." -ForegroundColor Green

# Create desktop shortcut that launches via schtasks (no UAC prompt)
$shell     = New-Object -ComObject WScript.Shell
$shortcut  = $shell.CreateShortcut("$env:USERPROFILE\Desktop\nullCAT.lnk")
$shortcut.TargetPath   = "schtasks.exe"
$shortcut.Arguments    = "/run /tn `"$taskName`""
$shortcut.WindowStyle  = 7          # minimised - hides the schtasks console flash
$shortcut.IconLocation = $watchdog  # use the watchdog exe icon
$shortcut.WorkingDirectory = $scriptDir
$shortcut.Save()

Write-Host "Desktop shortcut created." -ForegroundColor Green
Write-Host ""
Write-Host "Use 'nullCAT' on the desktop to launch without UAC prompt." -ForegroundColor Cyan
