#Requires -RunAsAdministrator
<#
  nullCAT-Pi - put the PC's wired Ethernet on the Pi point-to-point link.
  Sets  PC = 192.168.50.2/24  (no gateway)  ->  Pi at 192.168.50.1.

  Usage (elevated PowerShell):
      .\pi-link.ps1                 # uses adapter named "Ethernet"
      .\pi-link.ps1 -IfAlias "Ethernet 2"
  List adapters:  Get-NetAdapter
  Revert to normal DHCP:  .\pi-link-off.ps1

  No-script alternative (auto-cutover): set the adapter to DHCP and put
  192.168.50.2 / 255.255.255.0 (no gateway) under IPv4 Properties ->
  "Alternate Configuration" -> User configured. Windows then uses DHCP on a
  normal network and falls back to .50.2 on the direct Pi cable automatically.
#>
param([string]$IfAlias = "Ethernet")
$ErrorActionPreference = "Stop"

Remove-NetIPAddress -InterfaceAlias $IfAlias -AddressFamily IPv4 -Confirm:$false -ErrorAction SilentlyContinue
Remove-NetRoute     -InterfaceAlias $IfAlias -AddressFamily IPv4 -Confirm:$false -ErrorAction SilentlyContinue
Set-NetIPInterface  -InterfaceAlias $IfAlias -Dhcp Disabled
New-NetIPAddress    -InterfaceAlias $IfAlias -IPAddress 192.168.50.2 -PrefixLength 24 | Out-Null

Write-Host "[$IfAlias] PC = 192.168.50.2/24  ->  Pi 192.168.50.1" -ForegroundColor Green
Write-Host "  Web UI : http://192.168.50.1:8080"
Write-Host "  Telemetry: send UDP to 192.168.50.1:4444"
