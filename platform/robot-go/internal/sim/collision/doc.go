// Package collision implements the track wall/obstacle geometry and the
// chassis raycast/collision model the headless simulator uses, ported from
// platform/robot/src/simulation/track_model.py and
// platform/robot/src/simulation/collision_stepping.py.
//
// TrackModel wraps internal/nav/trackmodel.TrackWalls (the wall geometry
// shared with the real navigation stack) and adds what the simulator alone
// needs: obstacle raycasting/collision (ObstacleBox, for traffic signs and
// parking blocks), oriented-rectangle chassis footprint checks
// (ContactSurfaceAt, FootprintCollides), and how far a chassis has pushed
// into a touched obstacle (ObstacleDisplacements).
//
// AllowedStep is collision_stepping.py's bisection algorithm: how far a
// commanded kinematic step actually gets before a solid surface stops it,
// scaling rotation and translation SEPARATELY rather than together. See
// AllowedStep's doc comment for why scaling them together deadlocks the
// chassis against a wall forever -- that reasoning is load-bearing context
// from the historical fix (Python commit 06b7fc1a) and is not obvious from
// the code alone.
//
// # Obstacle-metadata scoping
//
// track_model.py's obstacles_from_metadata reads obstacle poses out of a
// raw scenario-metadata dict. There is no Go source for that dict yet in
// this codebase, so this package does not port a dict-parsing layer --
// only the "build an ObstacleBox from a known x/y/length/width/yaw" logic
// (ObstacleSpec, ObstaclesFromSpecs), which takes already-parsed obstacle
// data as a plain Go slice instead.
package collision
