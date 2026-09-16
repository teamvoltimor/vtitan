# 0093. Marker geometry is float-typed at the model boundary and solid markers use CUBE

- Status: accepted
- Date: 2026-09-16

## Context

The live RViz visualizer builds `visualization_msgs/MarkerArray` from scenario
metadata. Two failure modes were invisible to ordinary use.

Whole-number coordinates (for example `"x": 1`) parse from JSON as a Python
`int`, not a `float`. Assigning that `int` straight to a `geometry_msgs/Point`
field reads back correctly in memory, because Python does not enforce the
annotation, but CDR serialization onto the DDS wire reinterprets the bits as a
float64 instead of converting the value. The coordinate collapses to a
near-zero subnormal and the sign or parking block jumps onto a wall in RViz.

Separately, walls were first drawn with `TRIANGLE_LIST`, which this
RoboStack/Kilted RViz build drops (as it does `MESH_RESOURCE` on Windows), so the
walls disappeared.

## Options considered

- (a) Hand-cast `float()` at each marker builder, and keep the triangle and mesh
      marker types.
- (b) Declare the scenario models' coordinates as `float` so coercion happens
      once at the model boundary, and draw solid markers with `CUBE`.

## Decision

(b). `SignPosition`, `BlockPosition` and `ParkingLot` declare their coordinates
`float`, so pydantic converts once when raw JSON becomes the validated model;
marker builders assign the fields directly and no builder has to remember a cast.
Solid markers use `CUBE`, the only solid type that renders reliably in this RViz
build; the cost is that RViz shades it with its single scene light, so brightness
swings as you orbit, which is acceptable for walls.

## Consequences

- A new marker builder cannot reintroduce the int-to-float corruption by
  forgetting a cast, because the boundary does it once.
- This class of bug is invisible to plain attribute access on the in-memory
  message; only an actual CDR serialize/deserialize round trip catches it, so the
  regression test forces one.
- Walls and floors are lit by a single scene light rather than per-face shading.

## Cross-references

- 0019 owns the simulation robot-model topic split this rides on, not the marker
  serialization contract.
- Code: `src/python/src/simulation/live_visualizer/visualizer.py` and
  `src/python/tests/unit/test_live_visualizer_markers.py`.
