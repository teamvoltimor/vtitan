package scenario_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/sim/scenario"
)

func TestParseResult(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name    string
		json    string
		want    scenario.Result
		wantErr bool
	}{
		{
			name: "successful three-lap run",
			json: `{"scenario":"scenario_0000_metadata.json","target_laps":3,"laps_completed":3,` +
				`"collided":false,"timed_out":false,"stuck":false,"pass_side_violation":false,` +
				`"pass_side_violation_signs":[],"parked":true,"success":true,"over_time":false,` +
				`"steps":2600,"sim_time_s":130.0,"distance_m":11.4,"max_speed_mps":0.156,` +
				`"avg_speed_mps":0.09,"min_lidar_range_m":0.18,"collision_xy":null,` +
				`"final_pose":[1.5,0.25,0.0],"contact_count":0,"contact_time_s":0.0,` +
				`"terminal_surface":"none","lap_step_indices":[820,1690,2600]}`,
			want: scenario.Result{
				Scenario:               "scenario_0000_metadata.json",
				TargetLaps:             3,
				LapsCompleted:          3,
				PassSideViolationSigns: []int{},
				Parked:                 new(true),
				Success:                true,
				Steps:                  2600,
				SimTimeS:               130.0,
				DistanceM:              11.4,
				MaxSpeedMPS:            0.156,
				AvgSpeedMPS:            0.09,
				MinLidarRangeM:         0.18,
				FinalPose:              []float64{1.5, 0.25, 0.0},
				TerminalSurface:        "none",
				LapStepIndices:         []int{820, 1690, 2600},
			},
		},
		{
			name: "collided run, no parking lot",
			json: `{"scenario":"scenario_0005_metadata.json","target_laps":3,"laps_completed":1,` +
				`"collided":true,"timed_out":false,"stuck":false,"pass_side_violation":false,` +
				`"pass_side_violation_signs":[],"parked":null,"success":false,"over_time":false,` +
				`"steps":940,"sim_time_s":47.0,"distance_m":4.1,"max_speed_mps":0.156,` +
				`"avg_speed_mps":0.09,"min_lidar_range_m":0.0,"collision_xy":[1.2,0.4],` +
				`"final_pose":[1.2,0.4,1.57],"contact_count":1,"contact_time_s":0.05,` +
				`"terminal_surface":"sign","lap_step_indices":[]}`,
			want: scenario.Result{
				Scenario:               "scenario_0005_metadata.json",
				TargetLaps:             3,
				LapsCompleted:          1,
				Collided:               true,
				PassSideViolationSigns: []int{},
				Parked:                 nil,
				Steps:                  940,
				SimTimeS:               47.0,
				DistanceM:              4.1,
				MaxSpeedMPS:            0.156,
				AvgSpeedMPS:            0.09,
				CollisionXY:            []float64{1.2, 0.4},
				FinalPose:              []float64{1.2, 0.4, 1.57},
				ContactCount:           1,
				ContactTimeS:           0.05,
				TerminalSurface:        "sign",
				LapStepIndices:         []int{},
			},
		},
		{
			name:    "malformed JSON",
			json:    `{"scenario": `,
			wantErr: true,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			got, err := scenario.ParseResultForTest(tt.json)
			if tt.wantErr {
				if err == nil {
					t.Fatal("ParseResultForTest() error = nil, want an error")
				}
				return
			}
			if err != nil {
				t.Fatalf("ParseResultForTest() error = %v, want nil", err)
			}
			assertResultEqual(t, got, tt.want)
		})
	}
}

func assertResultEqual(t *testing.T, got, want scenario.Result) {
	t.Helper()

	if got.Scenario != want.Scenario ||
		got.TargetLaps != want.TargetLaps ||
		got.LapsCompleted != want.LapsCompleted ||
		got.Collided != want.Collided ||
		got.Success != want.Success ||
		got.Steps != want.Steps ||
		got.TerminalSurface != want.TerminalSurface {
		t.Fatalf("ParseResultForTest() = %+v, want %+v", got, want)
	}
	if (got.Parked == nil) != (want.Parked == nil) {
		t.Fatalf("Parked = %v, want %v", got.Parked, want.Parked)
	}
	if got.Parked != nil && *got.Parked != *want.Parked {
		t.Fatalf("*Parked = %v, want %v", *got.Parked, *want.Parked)
	}
}
