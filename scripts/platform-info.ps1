Write-Host "=== PYTHON ==="
uv --version
Write-Host ""
Write-Host "=== PIXI ==="
pixi --version
if (-not $?) { Write-Host "Pixi not installed" }
Write-Host ""
Write-Host "=== GO ==="
go version
if (-not $?) { Write-Host "Go not installed" }
Write-Host ""
Write-Host "=== BUF ==="
buf --version
if (-not $?) { Write-Host "buf not installed (needed for proto ACTION=generate)" }
Write-Host ""
Write-Host "=== DIRECTORIES ==="
Write-Host "Output dir:   $($args[0])"
Write-Host "Gazebo:       apps/gazebo/ (generator + runtime)"
Write-Host "Backend (Go): apps/backend/"
Write-Host "Frontend:     apps/frontend/"
Write-Host "Robot:        src/"
Write-Host "Proto:        contracts/proto/"
Write-Host "Shared:       src/shared/"
