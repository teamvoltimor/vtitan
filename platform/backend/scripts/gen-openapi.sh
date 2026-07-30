#!/usr/bin/env bash
set -euo pipefail

# Generate Go types from all context OpenAPI specs, all into api/{context}
# packages (spec-first, ready for handlers)
declare -A packages=(
  [telemetry]=telemetry
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
