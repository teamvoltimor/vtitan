package core

// SensorStatus is the readiness of one hardware/software component,
// checked during StateBootCheck. Mirrors types.py's SensorStatus
// dataclass. ErrorMessage is the empty string when IsReady is true or no
// error has been recorded, matching the Python field's `str | None`
// default of None -- an empty string and "no error" are the same thing
// here, so a pointer would add nothing a zero value doesn't already give.
type SensorStatus struct {
	Name         string
	ErrorMessage string
	IsReady      bool
}

// SystemStatus is the overall boot-check readiness snapshot built once
// per StateBootCheck tick. Mirrors types.py's SystemStatus dataclass.
// ChallengeMode is nil until the jumper reading has stabilized (see
// core/doc.go's scoping note -- the sampling/debounce logic that produces
// this value lives outside this package), matching the Python field's
// `ScenarioType | None = None` default.
type SystemStatus struct {
	NetworkStatus       string
	ChallengeMode       *ScenarioType
	IMUStatus           SensorStatus
	LidarStatus         SensorStatus
	HailoStatus         SensorStatus
	DriveStatus         SensorStatus
	ChallengeModeStatus SensorStatus
	AllReady            bool
}

// RaceStatus is an instantaneous race-state snapshot for telemetry
// display. Mirrors types.py's RaceStatus dataclass. CurrentCorridor is
// nil when the active track corridor is unknown, matching the Python
// field's `str | None = None` default.
type RaceStatus struct {
	CurrentCorridor    *string
	TotalRaceTimeSec   float64
	CurrentVelocityMPS float64
	CurrentSteeringDeg float64
	GyroYawDeg         float64
	LapsCompleted      int
}
