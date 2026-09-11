// Package pb_test proves protovalidate constraints declared in the .proto
// files (see proto/vtitan/actuation/v1/ackermann_cmd.proto) are actually
// enforced at runtime, not just present as unused schema annotations.
package actuationv1_test

import (
	"math"
	"testing"

	"buf.build/go/protovalidate"
	"google.golang.org/protobuf/types/known/timestamppb"

	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
)

func TestAckermannCmdValidation(t *testing.T) {
	t.Parallel()

	validator, err := protovalidate.New()
	if err != nil {
		t.Fatalf("protovalidate.New: %v", err)
	}

	tests := map[string]struct {
		cmd     *actuationv1.AckermannCmd
		wantErr bool
	}{
		"valid command passes": {
			cmd: &actuationv1.AckermannCmd{
				Stamp:         timestamppb.Now(),
				FrameId:       "base_link",
				SteeringAngle: 0.1,
				Speed:         0.5,
			},
			wantErr: false,
		},
		"missing stamp fails required constraint": {
			cmd: &actuationv1.AckermannCmd{
				FrameId:       "base_link",
				SteeringAngle: 0.1,
				Speed:         0.5,
			},
			wantErr: true,
		},
		"NaN speed fails finite constraint": {
			cmd: &actuationv1.AckermannCmd{
				Stamp:         timestamppb.Now(),
				FrameId:       "base_link",
				SteeringAngle: 0.1,
				Speed:         float32(math.NaN()),
			},
			wantErr: true,
		},
	}

	for name, tt := range tests {
		t.Run(name, func(t *testing.T) {
			t.Parallel()

			validateErr := validator.Validate(tt.cmd)
			if (validateErr != nil) != tt.wantErr {
				t.Errorf("Validate() error = %v, wantErr %v", validateErr, tt.wantErr)
			}
		})
	}
}
