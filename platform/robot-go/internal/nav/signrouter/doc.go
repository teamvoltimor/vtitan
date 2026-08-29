// Package signrouter ports
// platform/robot/src/navigation/planning/sign_router/{router,routing,
// deformation,config}.py and sign_lane.py: WRO 2026 traffic-sign avoidance
// for the Obstacles Challenge.
//
// Computes lateral waypoint deformations so the robot avoids a red obstacle
// on its OUTWARD side (toward the outer wall) and a green obstacle on its
// INWARD side (toward the inner square) -- an absolute rule tied to track
// geometry, not travel direction: it holds identically whether the round is
// run clockwise or counterclockwise.
//
// # Scope and deviations from the Python source
//
// Blind sign discovery (Python's ObservedSignMap /
// SignDiscoveryParams / SignRouter(discover=True)) is NOT ported. It lives
// in sign_discovery.py, a separate module never named in this port's scope,
// and is a materially larger piece of work (camera pinhole geometry, track
// association/publication state machine) than the routing logic itself.
// SignRouter here only supports the "sighted" mode -- signs handed to it up
// front, exactly as if every SignSpec came from scenario metadata. This
// also means SignRouter.is_discovering, .lane_fingerprint and the
// discovery-only constructor parameters (discover, discovery_config,
// tuning) have no Go equivalent.
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
