package scenario

import (
	"encoding/json"
	"fmt"
)

// Result is one scenario run's outcome, as reported by the Python simulator.
// Field names and JSON tags mirror scripts/sim/run_scenario.py's
// _result_payload exactly — nothing here is invented beyond what
// src.simulation.scenario_result.SimResult already produces, so this struct
// stays a direct, auditable translation of that payload rather than a
// reinterpretation of it.
type Result struct {
	// TerminalSurface names which surface (if any) ended the run — mirrors
	// SimResult.terminal_surface.value, a Python StrEnum member.
	TerminalSurface string `json:"terminal_surface"`
	Scenario        string `json:"scenario"`

	PassSideViolationSigns []int     `json:"pass_side_violation_signs"`
	LapStepIndices         []int     `json:"lap_step_indices"`
	CollisionXY            []float64 `json:"collision_xy"`
	FinalPose              []float64 `json:"final_pose"`

	// Parked is nil when the scenario has no parking lot (SimResult.parked
	// is None in that case) rather than false, which would misreport a
	// parking-less scenario as a failed parking attempt.
	Parked *bool `json:"parked"`
	// ParkPoints is the final pose scored against the WRO 15/7/0 point
	// tiers (parking.ScorePark), nil under the same condition as Parked.
	// No Python counterpart in SimResult -- that scorer was ported
	// standalone and never wired in there either.
	ParkPoints *int `json:"park_points,omitempty"`

	SimTimeS       float64 `json:"sim_time_s"`
	DistanceM      float64 `json:"distance_m"`
	MaxSpeedMPS    float64 `json:"max_speed_mps"`
	AvgSpeedMPS    float64 `json:"avg_speed_mps"`
	MinLidarRangeM float64 `json:"min_lidar_range_m"`
	ContactTimeS   float64 `json:"contact_time_s"`

	TargetLaps    int `json:"target_laps"`
	LapsCompleted int `json:"laps_completed"`
	Steps         int `json:"steps"`
	ContactCount  int `json:"contact_count"`

	Collided          bool `json:"collided"`
	TimedOut          bool `json:"timed_out"`
	Stuck             bool `json:"stuck"`
	PassSideViolation bool `json:"pass_side_violation"`
	Success           bool `json:"success"`
	OverTime          bool `json:"over_time"`
}

// parseResult decodes one line of JSON produced by
// scripts/sim/run_scenario.py's stdout into a Result.
func parseResult(data []byte) (Result, error) {
	var result Result
	if err := json.Unmarshal(data, &result); err != nil {
		return Result{}, fmt.Errorf("scenario: decoding result JSON: %w", err)
	}
	return result, nil
}
