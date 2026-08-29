// Package waypoints ports the geometry-and-assembly half of
// platform/robot/src/navigation/planning/waypoints/ (classification.py,
// geometry.py, segments.py, and the two generation.py functions that don't
// need a ScenarioMetadata) -- everything portable without a Go
// ScenarioMetadata/StartingConditions/CorridorWidths domain-model port,
// including CorridorForPosition, the function
// internal/nav/signrouter/internal/nav/navigator actually need.
//
// generation.py's calculate_waypoints (the top-level entry point) and
// plan_believed_path are deliberately NOT ported here: both take a
// ScenarioMetadata, which has no Go equivalent in this tree yet (no
// StartingConditions, CorridorWidths/CorridorWidthEntry, Position2D, or
// PathPlannability-adjacent domain models beyond PathPlannability itself,
// which IS ported here since ValidatePathFeasibility needs it and nothing
// else does). Porting calculate_waypoints is a separate, larger effort
// once that domain-model layer exists.
package waypoints
