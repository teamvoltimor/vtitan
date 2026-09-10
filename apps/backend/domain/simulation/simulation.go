// Package simulation is the Simulation bounded context: WRO scenario
// generation/storage, environment configuration, and simulation run
// lifecycle/metrics.
package simulation

import (
	"errors"
	"time"
)

// ErrNotFound is returned when a scenario or run does not exist.
var ErrNotFound = errors.New("simulation resource not found")

// Challenge is the WRO challenge type a scenario targets.
type Challenge string

// Challenge values.
const (
	ChallengeOpen      Challenge = "open"
	ChallengeObstacles Challenge = "obstacles"
)

// Corridors is the corridor-width profile requested for scenario generation.
type Corridors string

// Corridors values.
const (
	CorridorsNarrow Corridors = "narrow"
	CorridorsWide   Corridors = "wide"
	CorridorsMixed  Corridors = "mixed"
)

// Lighting is a scenario's lighting condition preset.
type Lighting string

// Lighting values.
const (
	LightingDirectSunlight Lighting = "DIRECT_SUNLIGHT"
	LightingCloudy         Lighting = "CLOUDY"
	LightingIndoorBright   Lighting = "INDOOR_BRIGHT"
	LightingIndoorDim      Lighting = "INDOOR_DIM"
	LightingEvening        Lighting = "EVENING"
	LightingMixed          Lighting = "MIXED"
)

// Direction is the starting-zone traversal direction.
type Direction string

// Direction values.
const (
	DirectionClockwise        Direction = "CLOCKWISE"
	DirectionCounterclockwise Direction = "COUNTERCLOCKWISE"
)

// TrackConfig describes the generated track geometry.
type TrackConfig struct {
	TrackWidth     *float64
	CorridorWidths map[string]CorridorWidth
	Sections       []string
}

// CorridorWidth is a single named corridor's type/width.
type CorridorWidth struct {
	Type  *string
	Width *float64
}

// LightingConfig describes the generated lighting setup.
type LightingConfig struct {
	Intensity        *float32
	AmbientIntensity *float32
	Direction        []float32
	CastShadows      *bool
	Scenario         *string
}

// ParkingLotConfig describes the generated parking lot layout.
type ParkingLotConfig struct {
	Block1Position []float64
	Block2Position []float64
	Depth          *float64
}

// StartingZoneConfig describes the generated starting zone.
type StartingZoneConfig struct {
	Length    *float64
	Position  []float64
	Direction *Direction
}

// Scenario is a generated WRO scenario.
type Scenario struct {
	ID             string
	Name           string
	Challenge      Challenge
	CreatedAt      time.Time
	DetectionCount *int
	Environment    *string
	TrackConfig    *TrackConfig
	Lighting       *LightingConfig
	ParkingLot     *ParkingLotConfig
	StartingZone   *StartingZoneConfig
}

// GenerateScenarioRequest is the payload for generating a new scenario.
type GenerateScenarioRequest struct {
	Challenge Challenge
	Corridors *Corridors
	Lighting  *Lighting
	NumSigns  *int
	Randomize *bool
}

// RunStatus is a simulation run's lifecycle state.
type RunStatus string

// RunStatus values.
const (
	RunPending   RunStatus = "pending"
	RunRunning   RunStatus = "running"
	RunPaused    RunStatus = "paused"
	RunCompleted RunStatus = "completed"
	RunFailed    RunStatus = "failed"
	RunCancelled RunStatus = "cancelled"
)

// RunMetrics is a simulation run's accumulated performance metrics.
type RunMetrics struct {
	AvgLapTimeS      *float64
	MaxSpeedMS       *float64
	AvgSpeedMS       *float64
	PathLengthM      *float64
	ObstacleContacts *int
	SignDetections   *int
}

// Run is a single simulation execution.
type Run struct {
	ID             string
	ScenarioID     string
	Status         RunStatus
	StartedAt      time.Time
	CompletedAt    *time.Time
	LapsCompleted  *int
	TotalLaps      *int
	DurationS      *float64
	CollisionCount *int
	Metrics        *RunMetrics
}

// StartRunRequest is the payload for starting a new simulation run.
type StartRunRequest struct {
	ScenarioID string
	Laps       *int
	Headless   *bool
}

// RunAction is a control action applied to a running simulation.
type RunAction string

// RunAction values.
const (
	RunActionPause   RunAction = "pause"
	RunActionResume  RunAction = "resume"
	RunActionStop    RunAction = "stop"
	RunActionRestart RunAction = "restart"
)

// Environment is an available Gazebo simulation environment.
type Environment struct {
	ID        string
	Name      string
	WorldFile string
	Lighting  *LightingConfig
}
