// Package controllers is the Go port of
// platform/robot/src/navigation/control/controllers/ plus
// platform/robot/src/navigation/clearances.py and the portable parts of
// platform/robot/src/navigation/ports.py.
//
// It provides the reactive layer of the navigation stack: pure-pursuit
// waypoint following (WaypointController), LIDAR-based collision detection
// and escape-maneuver generation (CollisionAvoidanceController), stuck
// detection (StuckDetector), and the LIDAR sector/clearance math they share
// (SectorToModel, ForwardPathRanges, ClearancesFromScan). HardwareGateway is
// declared here rather than in a hardware/simulation package so the
// dependency direction matches hexagonal architecture: adapters import this
// port, the domain never imports an adapter -- mirroring ports.py's own
// module doc comment.
//
// Enums (RiskLevel, ThreatDirection, ManeuverType) and the SectorRanges/
// LidarClearances value types live here too, matching
// shared.domain.enums/shared.domain.models -- the Go module has no separate
// "shared domain" package these belong to instead, so they are placed
// alongside their sole producer/consumer, controllers, the same way
// trackmodel already hosts Waypoint/Section/Direction/CorridorSide for the
// packages that need them.
package controllers
