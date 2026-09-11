// Package navv1_test proves the protovalidate constraints declared in
// proto/vtitan/nav/v1/*.proto are enforced at runtime, not just present as
// unused schema annotations.
package navv1_test

import (
	"math"
	"testing"

	"buf.build/go/protovalidate"
	"google.golang.org/protobuf/types/known/timestamppb"

	navv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/nav/v1"
)

func TestNavigatorDebugValidation(t *testing.T) {
	t.Parallel()

	validator, err := protovalidate.New()
	if err != nil {
		t.Fatalf("protovalidate.New: %v", err)
	}

	tests := map[string]struct {
		msg     *navv1.NavigatorDebug
		wantErr bool
	}{
		"minimal snapshot passes": {
			msg:     &navv1.NavigatorDebug{Stamp: timestamppb.Now(), Phase: navv1.Phase_PHASE_NO_POSE},
			wantErr: false,
		},
		"missing stamp fails required constraint": {
			msg:     &navv1.NavigatorDebug{Phase: navv1.Phase_PHASE_NO_POSE},
			wantErr: true,
		},
		// A pose outside the mat is exactly what this topic exists to record,
		// so it must NOT be rejected -- only non-finite values are.
		"pose outside the mat is accepted": {
			msg: &navv1.NavigatorDebug{
				Stamp: timestamppb.Now(),
				Phase: navv1.Phase_PHASE_NORMAL_DRIVE,
				Pose:  &navv1.NavigatorPose{X: -12.5, Y: 99.0, Yaw: 41.3},
			},
			wantErr: false,
		},
		"NaN pose fails finite constraint": {
			msg: &navv1.NavigatorDebug{
				Stamp: timestamppb.Now(),
				Phase: navv1.Phase_PHASE_NORMAL_DRIVE,
				Pose:  &navv1.NavigatorPose{X: math.NaN()},
			},
			wantErr: true,
		},
		// Escape maneuvers reverse, so a negative commanded speed is normal.
		"negative commanded speed is accepted": {
			msg: &navv1.NavigatorDebug{
				Stamp:   timestamppb.Now(),
				Phase:   navv1.Phase_PHASE_ACTIVE_MANEUVER,
				Command: &navv1.DriveCommand{SpeedMps: new(-0.25), SteeringNorm: new(-1.0)},
			},
			wantErr: false,
		},
		"steering beyond the port contract fails": {
			msg: &navv1.NavigatorDebug{
				Stamp:   timestamppb.Now(),
				Phase:   navv1.Phase_PHASE_NORMAL_DRIVE,
				Command: &navv1.DriveCommand{SteeringNorm: new(1.5)},
			},
			wantErr: true,
		},
		"negative lap count fails": {
			msg: &navv1.NavigatorDebug{
				Stamp: timestamppb.Now(),
				Phase: navv1.Phase_PHASE_NORMAL_DRIVE,
				Race:  &navv1.RaceState{LapsCompleted: -1},
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
