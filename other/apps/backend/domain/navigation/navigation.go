// Package navigation is the Navigation bounded context: waypoint management,
// route planning, and navigation subsystem status/tuning.
package navigation

import (
	"errors"
	"time"
)

// ErrNotFound is returned when a waypoint (or other scoped resource) does not exist.
var ErrNotFound = errors.New("navigation resource not found")

// Waypoint is a single planned point along a route.
type Waypoint struct {
	ID        string
	X         float64
	Y         float64
	Index     int
	Tolerance *float64
	Section   *string
}

// CreateWaypointRequest is the payload for adding a waypoint.
type CreateWaypointRequest struct {
	X         float64
	Y         float64
	Tolerance *float64
	Section   *string
}

// Route is the current planned route.
type Route struct {
	ID                 string
	Waypoints          []Waypoint
	TotalDistance      float64
	EstimatedDurationS float64
	Laps               *int
	CreatedAt          *time.Time
}

// PlanRouteRequest is the payload for planning or replanning a route.
type PlanRouteRequest struct {
	StartX   float64
	StartY   float64
	StartYaw *float64
	Laps     *int
}

// RiskLevel is the current navigation risk assessment.
type RiskLevel string

// RiskLevel values.
const (
	RiskSafe     RiskLevel = "SAFE"
	RiskCritical RiskLevel = "CRITICAL"
	RiskObstacle RiskLevel = "OBSTACLE"
)

// Phase is the current navigation execution phase.
type Phase string

// Phase values.
const (
	PhaseTracking Phase = "TRACKING"
	PhaseAvoiding Phase = "AVOIDING"
	PhaseEscaping Phase = "ESCAPING"
	PhaseParking  Phase = "PARKING"
	PhaseFinished Phase = "FINISHED"
)

// Status is the navigation subsystem's current status.
type Status struct {
	Active               bool
	CurrentWaypointIndex int
	TotalWaypoints       int
	CurrentSection       *string
	RiskLevel            *RiskLevel
	Phase                *Phase
	LastReplan           *time.Time
}

// Clearance is the most recent LIDAR clearance reading.
type Clearance struct {
	Front          *float64
	Left           *float64
	Right          *float64
	Back           *float64
	PointsCaptured *int
	RangeMin       *float64
	RangeMax       *float64
	Timestamp      time.Time
}

// Tuning is the navigation subsystem's tunable PID/lookahead parameters.
type Tuning struct {
	LookaheadShort  *float64
	LookaheadLong   *float64
	SteerKp         *float64
	MaxSteeringRate *float64
	ContactDist     *float64
	SlowDist        *float64
	MediumDist      *float64
	FastDist        *float64
	MinSpeed        *float64
	MaxSpeed        *float64
}
