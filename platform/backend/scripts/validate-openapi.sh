#!/usr/bin/env bash
set -euo pipefail

ok=true
for file in ../openapi/contexts/*.yaml; do
  if grep -qE '^openapi: 3\.' "$file"; then
    echo "✓ $(basename "$file") valid"
  else
    echo "✗ $(basename "$file"): missing or invalid openapi version header"
    ok=false
  fi
done
if [ "$ok" = false ]; then exit 1; fi
