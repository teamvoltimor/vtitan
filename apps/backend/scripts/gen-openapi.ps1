# Generate Go types from all context OpenAPI specs, all into api/{context}
# packages (spec-first, ready for handlers)
$packages = @{telemetry='telemetry'; robot='robot'; vision='vision'; navigation='navigation'; simulation='simulation'}
foreach ($entry in $packages.GetEnumerator()) {
    $ctx = $entry.Key
    $pkg = $entry.Value
    Write-Host "Generating $ctx (api/$pkg package)..."
    oapi-codegen -package $pkg -generate types -o internal/api/$pkg/openapi.gen.go ../../contracts/openapi/contexts/$ctx.yaml
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    Write-Host "  ✓ api/$pkg/openapi.gen.go"
}
