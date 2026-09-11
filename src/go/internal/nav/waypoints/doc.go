// Package waypoints ports platform/robot/src/navigation/planning/waypoints/
// (classification.py, geometry.py, segments.py, and generation.py), the
// geometry-and-assembly half of WRO waypoint generation, including
// CorridorForPosition (used by internal/nav/signrouter and the navigator).
//
// generation.py's calculate_waypoints (the top-level entry point) and
// plan_believed_path are ported here as CalculateWaypoints and
// PlanBelievedPath. They take a PlannerInput -- a Go stand-in for the
// Pydantic ScenarioMetadata carrying only the fields generation.py actually
// reads (believed corridor geometry + starting conditions + track/chassis
// extents) -- so the planner stays decoupled from the rest of the
// scenario-model layer, which has no Go equivalent in this tree yet. The
// lower-level helpers (CenterBiasForCorridor, ValidatePathFeasibility,
// CornerArcRadius, BuildAllSegments, AssembleLoop, BuildWaypointSequence,
// ValidateBounds) are ported faithfully from their generation.py/
// geometry.py/segments.py originals.
package waypoints
