// Package statev1_test proves the protovalidate constraints declared in
// proto/vtitan/state/v1/*.proto are enforced at runtime, not just present as
// unused schema annotations.
package statev1_test

import (
	"math"
	"testing"

	"buf.build/go/protovalidate"
	"google.golang.org/protobuf/proto"
	"google.golang.org/protobuf/types/known/timestamppb"

	statev1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/state/v1"
)

func TestStateValidation(t *testing.T) {
	t.Parallel()

	validator, err := protovalidate.New()
	if err != nil {
		t.Fatalf("protovalidate.New: %v", err)
	}

	tests := map[string]struct {
		msg     proto.Message
		wantErr bool
	}{
		"robot state passes": {
			msg:     &statev1.RobotState{Stamp: timestamppb.Now(), State: statev1.RobotState_STATE_RACING},
			wantErr: false,
		},
		"robot state without stamp fails": {
			msg:     &statev1.RobotState{State: statev1.RobotState_STATE_RACING},
			wantErr: true,
		},
		"race metrics pass": {
			msg:     &statev1.RaceMetrics{Stamp: timestamppb.Now(), LapsCompleted: 2},
			wantErr: false,
		},
		// Reverse is normal during escape maneuvers, so a signed velocity
		// must pass rather than trip a non-negative bound.
		"negative velocity is accepted": {
			msg:     &statev1.RaceMetrics{Stamp: timestamppb.Now(), CurrentVelocityMps: -0.3},
			wantErr: false,
		},
		"NaN velocity fails finite constraint": {
			msg:     &statev1.RaceMetrics{Stamp: timestamppb.Now(), CurrentVelocityMps: math.NaN()},
			wantErr: true,
		},
		"negative target laps fails": {
			msg:     &statev1.RaceMetrics{Stamp: timestamppb.Now(), TargetLaps: new(int32(-1))},
			wantErr: true,
		},
		"system status with a named entry passes": {
			msg: &statev1.SystemStatus{
				Stamp: timestamppb.Now(),
				Status: []*statev1.SystemStatus_Status{
					{Level: statev1.SystemStatus_LEVEL_OK, Name: "vision"},
				},
			},
			wantErr: false,
		},
		"system status entry without a name fails": {
			msg: &statev1.SystemStatus{
				Stamp:  timestamppb.Now(),
				Status: []*statev1.SystemStatus_Status{{Level: statev1.SystemStatus_LEVEL_OK}},
			},
			wantErr: true,
		},
	}

	for name, tt := range tests {
		t.Run(name, func(t *testing.T) {
			t.Parallel()

			validateErr := validator.Validate(tt.msg)
			if (validateErr != nil) != tt.wantErr {
				t.Fatalf("Validate() error = %v, wantErr %v", validateErr, tt.wantErr)
			}
		})
	}
}
