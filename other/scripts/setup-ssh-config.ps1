param(
    [string]$ConfigPath = "$env:USERPROFILE\.ssh\config",
    [string]$RpiDirectIp = "192.168.251.2",
    [string]$RpiLocalIp = "192.168.0.50",
    [string]$RpiZeroHostname = "YOUR_PI_ZERO_HOSTNAME.local",
    [string]$RpiZeroGadgetIp = "192.168.250.1",
    [string]$User = "YOUR_USERNAME",
    [string]$KeyFile = "~/.ssh/YOUR_KEY_FILE",
    [string]$RemoteHost = "pi.YOUR_DOMAIN.dev"
)

$entries = @(
    "# Raspberry Pi entries (added by task windows:ssh:setup-config)"
    "Host rpi-5-local"
    "    HostName $RpiLocalIp"
    "    User $User"
    "    Port 22"
    "    IdentityFile $KeyFile"
    "    ServerAliveInterval 60"
    "    ConnectTimeout 10"
    ""
    # Also matches the raw IP so `ProxyJump $User@$RpiDirectIp` (used by
    # rpi-zero-direct below) picks up the same IdentityFile/User — a ProxyJump
    # target string is looked up by literal host, not by alias name.
    "Host rpi-5-direct $RpiDirectIp"
    "    HostName $RpiDirectIp"
    "    User $User"
    "    Port 22"
    "    IdentityFile $KeyFile"
    "    ServerAliveInterval 60"
    "    ConnectTimeout 10"
    ""
    "Host rpi-5-remote"
    "    HostName $RemoteHost"
    "    ProxyCommand cloudflared access ssh --hostname %h"
    "    User $User"
    "    Port 22"
    "    IdentityFile $KeyFile"
    "    ServerAliveInterval 60"
    "    ConnectTimeout 10"
    ""
    "Host rpi-zero-local"
    "    HostName $RpiZeroHostname"
    "    User $User"
    "    Port 22"
    "    IdentityFile $KeyFile"
    "    ServerAliveInterval 60"
    "    ConnectTimeout 10"
    ""
    # Reaches the Pi Zero over its USB-gadget link to Pi 5 (see
    # scripts/setup_pi_zero.sh / setup_pi_5.sh) — only reachable when the
    # rpi-5-direct Ethernet cable is connected, since that link is
    # point-to-point behind Pi 5.
    "Host rpi-zero-direct"
    "    HostName $RpiZeroGadgetIp"
    "    User $User"
    "    Port 22"
    "    ProxyJump $User@$RpiDirectIp"
    "    IdentityFile $KeyFile"
    "    ServerAliveInterval 60"
    "    ConnectTimeout 10"
)

$hosts = @("rpi-5-local", "rpi-5-direct", "rpi-5-remote", "rpi-zero-local", "rpi-zero-direct")
$missing = @()

if (Test-Path $ConfigPath) {
    $content = Get-Content $ConfigPath -Raw
    foreach ($h in $hosts) {
        if ($content -notmatch "(?m)^Host $h\b") {
            $missing += $h
        }
    }
} else {
    $missing = $hosts
    $null = New-Item -ItemType File -Path $ConfigPath -Force
}

if ($missing.Count -eq 0) {
    Write-Host "All SSH hosts already configured in $ConfigPath" -ForegroundColor Green
} else {
    Write-Host "Adding missing hosts: $($missing -join ', ')" -ForegroundColor Yellow
    Add-Content -Path $ConfigPath -Value ($entries -join "`r`n")
    Write-Host "Done. Edit $ConfigPath or re-run with parameters to customize:" -ForegroundColor Cyan
    Write-Host "  powershell -File scripts/setup-ssh-config.ps1 -RpiDirectIp <IP> -User <user> -KeyFile <path> -RpiZeroHostname <host.local>" -ForegroundColor Cyan
}
