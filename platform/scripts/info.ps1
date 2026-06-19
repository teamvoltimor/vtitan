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
if (-not $?) { Write-Host "buf not installed (needed for proto:generate)" }
Write-Host ""
Write-Host "=== DIRECTORIES ==="
Write-Host "Output dir:   $($args[0])"
Write-Host "Gazebo:       gazebo/ (generator + runtime)"
Write-Host "Backend (Go): backend/"
Write-Host "Frontend:     frontend/"
Write-Host "Robot:        robot/"
Write-Host "Proto:        proto/"
Write-Host "Shared:       shared/"
