# ============================================================
# uninstall-task.ps1  (Build 71)
#
# Removes the nullCAT scheduled task and desktop shortcut.
# Run as Administrator.
# ============================================================

$taskName = "nullCAT"

Unregister-ScheduledTask -TaskName $taskName -Confirm:$false -ErrorAction SilentlyContinue
Write-Host "Task '$taskName' removed." -ForegroundColor Yellow

$shortcut = "$env:USERPROFILE\Desktop\nullCAT.lnk"
if (Test-Path $shortcut) {
    Remove-Item $shortcut -Force
    Write-Host "Desktop shortcut removed." -ForegroundColor Yellow
}
