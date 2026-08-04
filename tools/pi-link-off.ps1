#Requires -RunAsAdministrator
<#
  nullCAT-Pi - revert the wired Ethernet to normal DHCP (off the Pi link).

  Usage (elevated PowerShell):
      .\pi-link-off.ps1
      .\pi-link-off.ps1 -IfAlias "Ethernet 2"
#>
param([string]$IfAlias = "Ethernet")
$ErrorActionPreference = "Stop"

Remove-NetIPAddress        -InterfaceAlias $IfAlias -AddressFamily IPv4 -Confirm:$false -ErrorAction SilentlyContinue
Remove-NetRoute            -InterfaceAlias $IfAlias -AddressFamily IPv4 -Confirm:$false -ErrorAction SilentlyContinue
Set-NetIPInterface         -InterfaceAlias $IfAlias -Dhcp Enabled
Set-DnsClientServerAddress -InterfaceAlias $IfAlias -ResetServerAddresses
ipconfig /renew | Out-Null

Write-Host "[$IfAlias] back to DHCP." -ForegroundColor Green
