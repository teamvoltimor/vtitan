# 0082. The wiring harness is defined as code

- Status: accepted
- Date: 2026-09-15

## Context

A hand-drawn connection diagram cannot be diffed, so a pin change is not versioned
the way the rest of the robot is. The README had claimed a connection diagram
existed for a long time while the folder held only two superseded flowcharts.

## Options considered

- (a) Draw the harness by hand and export an image.
- (b) Define the harness as code (tscircuit) and commit the exports.

## Decision

(b). The harness is defined in `schemes/wiring/tscircuit/circuit.tsx` and built
with tscircuit, so a pin change is a versioned code change. The exported
`harness.schematic.svg` and `harness.schematic.png` are committed because they are
what the documentation reads and rebuilding them needs the full toolchain; the PNG
is kept as a universal fallback and is the only scheme not converted to WebP.
Regeneration is `npm install` then `npm run artifacts`.

A schematic that renders proves nothing about connectivity: connectivity is
verified through the readable netlist, which lists net membership rather than
drawn wires. Ground is one 26-pin net, drawn as a GND netlabel per pin rather than
26 pairwise traces.

## Consequences

- The harness is reviewable and versioned with the code.
- Four intentionally unconnected pins are documented, not errors.
- The readable netlist is the check, not the rendered image.

## History

- 4e6beed3 2026-09-03: complete and verify the tscircuit harness. Splitters and
  buck converters had unconnected IN/OUT pins, so the battery rails stopped dead
  while the schematic rendered and `tsci build` reported zero errors.
- b7290f10 2026-09-05: pin the last unknowns and make the exports reproducible;
  correct the jumper location (Pi Zero, not Pi 5).
- 081fc868 2026-09-05: lay the schematic out in three bands.
- 3fa9cbfc 2026-09-05: draw ground as a symbol per pin, not as 26 wires (73 to 47
  traces in the JSX; still 73 source traces).
- 912c96b2 2026-09-05: render on white.
- 5034699f 2026-09-05: centralize diagrams and commit the harness schematic.
- 3a73ff3a 2026-09-08: move `docs/schemes/` to the repo-root `schemes/`.
- 878b8485 2026-09-14: document the harness in `schemes/README.md`.

## Refuted

- The harness compiling or rendering as evidence of connectivity; the jumper on
  the Pi 5; the retired JGB37-520 motor.

## Cross-references

- 0076 owns the pins and H-bridge wiring; 0073 owns the jumper.
