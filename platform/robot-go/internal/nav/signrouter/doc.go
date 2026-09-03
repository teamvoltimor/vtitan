// Package signrouter ports
// platform/robot/src/navigation/planning/sign_router/{router,routing,
// deformation,config}.py and sign_lane.py: WRO 2026 traffic-sign avoidance
// for the Obstacles Challenge.
//
// Computes lateral waypoint deformations so the robot passes a red obstacle
// on its own RIGHT and a green obstacle on its own LEFT (rules 2026 9.19)
// -- a TRAVEL-RELATIVE rule: since the vehicle's right is the outer wall
// driving counterclockwise and the inner square driving clockwise, the
// same rule points at opposite world directions between the two, and
// cannot be evaluated without knowing the travel direction. Corrected
// 2026-09-03 (Python 879198f7) after two months of an absolute reading
// (red always outward) that was right for counterclockwise and backwards
// for every clockwise round.
//
// # Scope and deviations from the Python source
//
// Blind sign discovery (Python's ObservedSignMap / SignDiscoveryParams /
// SignRouter(discover=True)) IS ported in discovery.go: DetectionToWorld /
// DetectionToObservation (pinhole projection, bearing accurate / range from
// the 0.10 m sign height at the bbox), ObservedSignMap (persistent world-frame
// track association gated on the robot's settled corridor, closest-observation
// position wins), and SignRouter.AppendSign / IsDiscovering wiring. Camera
// pinhole constants (CameraWidthPX, CameraHFOVRad, SignHeightM,
// LidarMountXOffsetM, CameraFarClipM) are restated in discovery.go as
// documented TODOs until a Go profile mirror of RobotSpecs/TrafficSignSpecs
// lands -- see plan §5b.
//
// .lane_fingerprint and the discovery-only constructor parameters (discover,
// discovery_config, tuning) are folded into ObservedSignMap: its Specs /
// NewlyConfirmed / Publish replace the Python lane_fingerprint property.
//
// signs_from_metadata (routing.py) is not ported either: it is the only
// function in the source package that touches ScenarioMetadata, it is not
// called by anything else ported here, and a Go caller can build
// []SignSpec directly from whatever scenario representation it already
// has.
//
// Python's SignRouterConfig (tuning knobs) and SignRouterContext /
// SignRouterConstants (chassis- and track-geometry-derived constants,
// threaded through the pure routing/deformation functions as an optional
// "context" parameter defaulting to a package-level singleton) are merged
// into a single Config struct here. Go has no equivalent of a mutable
// module-level default instance, and every call site in this package
// already has a live Config to hand around -- splitting the two back apart
// would only reintroduce Python's own "two independent copies of the same
// default can drift" hazard (see Config's doc comment) without buying
// anything back.
package signrouter
