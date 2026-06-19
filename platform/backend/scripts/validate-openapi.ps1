$ok = $true
foreach ($file in Get-ChildItem -Path ../openapi/contexts/*.yaml) {
  $content = Get-Content $file.FullName -Raw
  if ($content -match '^openapi: 3\.') {
    Write-Host "✓ $($file.Name) valid"
  } else {
    Write-Host "✗ $($file.Name): missing or invalid openapi version header"
    $ok = $false
  }
}
if (-not $ok) { exit 1 }
