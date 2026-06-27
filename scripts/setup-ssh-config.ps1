param(
    [string]$ConfigPath = "$env:USERPROFILE\.ssh\config",
    [string]$RpiDirectIp = "10.250.250.2",
    [string]$RpiLocalIp = "192.168.0.50",
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
    "Host rpi-5-direct"
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
)

$hosts = @("rpi-5-local", "rpi-5-direct", "rpi-5-remote")
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
    Write-Host "  powershell -File scripts/setup-ssh-config.ps1 -RpiDirectIp <IP> -User <user> -KeyFile <path>" -ForegroundColor Cyan
}
