#!/usr/bin/env bash
# Prove pkg/portable/** builds for the Pico 2 and passes its tests under
# TinyGo (adr:0098-pico-actuation-board-and-portable-cores).
#
# Packages are discovered, never listed, so a portable package added later is
# checked without editing this script. A library package cannot be built on
# its own for a microcontroller target, so each one is imported by a
# generated throwaway main; any firmware under firmware/ is built as itself.
#
# What each half proves: a blank import compiles the package for the target,
# but TinyGo drops unreachable code, so every build is the size of an empty
# program and a construct TinyGo can only reject when it is reachable would
# pass. `tinygo test` is the stronger half, since it executes the code under
# the TinyGo compiler and runtime, on the host rather than the RP2350.
set -euo pipefail

GO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$GO_ROOT"
TARGET=${TINYGO_TARGET:-pico2}

mapfile -t PKGS < <(go list ./pkg/portable/...)
if [ "${#PKGS[@]}" -eq 0 ]; then
  echo "tinygo-check: no packages under pkg/portable" >&2
  exit 1
fi

echo "== tinygo test (host) ${#PKGS[@]} package(s)"
tinygo test "${PKGS[@]}"

mkdir -p tmp
WORK=$(mktemp -d "$GO_ROOT/tmp/tgcheck.XXXXXX")
trap 'rm -rf "$WORK"' EXIT

echo "== tinygo build -target=$TARGET, one package at a time"
for pkg in "${PKGS[@]}"; do
  cat > "$WORK/main.go" <<GO
//go:build tinygo

package main

import _ "$pkg"

func main() {}
GO
  printf '  %-60s ' "${pkg#github.com/teamvoltimor/vtitan/src/go/}"
  tinygo build -target="$TARGET" -size short -o "$WORK/out.elf" "./${WORK#"$GO_ROOT"/}" | tail -1
done

if [ -d firmware ]; then
  echo "== firmware"
  for dir in firmware/*/; do
    printf '  %-60s ' "$dir"
    tinygo build -target="$TARGET" -size short -o "$WORK/fw.elf" "./$dir" | tail -1
  done
fi
