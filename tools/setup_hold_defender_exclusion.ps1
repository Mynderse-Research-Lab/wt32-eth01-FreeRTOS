#Requires -RunAsAdministrator
<#
.SYNOPSIS
  Add Windows Defender exclusions for EtherNet/IP hold/jog state and log files.

.DESCRIPTION
  Atomic JSON rewrites under tools/ can stall under Defender and starve the
  Class 1 RPI loop. Run once (elevated) on the bench PC:

    powershell -ExecutionPolicy Bypass -File tools/setup_hold_defender_exclusion.ps1

  Or: py tools/eip_test.py hold-setup
#>
$ErrorActionPreference = "Stop"
$tools = Split-Path -Parent $MyInvocation.MyCommand.Path
$paths = @(
    $tools,
    (Join-Path $tools ".eip_servo_hold_state.json"),
    (Join-Path $tools ".eip_servo_hold.log"),
    (Join-Path $tools ".eip_servo_hold_watchdog.json"),
    (Join-Path $tools ".eip_servo_hold_watchdog.log"),
    (Join-Path $tools ".eip_jog.log")
)

foreach ($p in $paths) {
    $prefs = Get-MpPreference
    if ($prefs.ExclusionPath -notcontains $p) {
        Add-MpPreference -ExclusionPath $p
        Write-Host "Added exclusion: $p"
    } else {
        Write-Host "Already excluded: $p"
    }
}
Write-Host "Done."
