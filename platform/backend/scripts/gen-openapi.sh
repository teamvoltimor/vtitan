#!/usr/bin/env bash
set -euo pipefail

# Generate Go types from all context OpenAPI specs
# Telemetry → edge package (active handlers)
echo "Generating telemetry (edge package)..."
oapi-codegen -package edge -generate types \
  -o internal/edge/openapi.telemetry.gen.go \
  ../openapi/contexts/telemetry.yaml
echo "  ✓ openapi.telemetry.gen.go"

# Other contexts → api/{context} packages (spec-first, ready for handlers)
declare -A packages=(
  [robot]=robot
  [vision]=vision
  [navigation]=navigation
  [simulation]=simulation
)
for ctx in "${!packages[@]}"; do
  pkg="${packages[$ctx]}"
  echo "Generating $ctx (api/$pkg package)..."
  oapi-codegen -package "$pkg" -generate types \
    -o "internal/api/$pkg/openapi.gen.go" \
    "../openapi/contexts/$ctx.yaml"
  echo "  ✓ api/$pkg/openapi.gen.go"
done
