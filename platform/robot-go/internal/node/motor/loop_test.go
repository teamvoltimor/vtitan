package motor_test

import (
	"errors"
	"math"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/node/motor"
	actuationv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/actuation/v1"
)

const speedTolerance = 1e-9

func TestSpeedToNormalized(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		in   float32
		want float64
	}{
		{name: "zero", in: 0, want: 0},
		{name: "one third of scale", in: 1.0, want: 0.3},
		{name: "negative", in: -1.0, want: -0.3},
		{name: "clamped above max", in: 10.0, want: 1.0},
		{name: "clamped below min", in: -10.0, want: -1.0},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			got := motor.SpeedToNormalized(tt.in, motor.DefaultSpeedScalePercentPerMPS)
			if math.Abs(got-tt.want) > speedTolerance {
				t.Errorf("SpeedToNormalized(%v) = %v, want %v", tt.in, got, tt.want)
			}
		})
	}
}

func TestSpeedToNormalized_CustomScale(t *testing.T) {
	t.Parallel()

	got := motor.SpeedToNormalized(2.0, 10.0)
	want := 0.2
	if math.Abs(got-want) > speedTolerance {
		t.Errorf("SpeedToNormalized(2.0, 10.0) = %v, want %v", got, want)
	}
}

func TestStatusFor(t *testing.T) {
	t.Parallel()

	t.Run("idle", func(t *testing.T) {
		t.Parallel()

		got := motor.StatusFor(0, 0, nil)
		if got.GetState() != actuationv1.MotorStatus_STATE_IDLE {
			t.Errorf("State = %v, want STATE_IDLE", got.GetState())
		}
		if got.GetDetail() != "" {
			t.Errorf("Detail = %q, want empty", got.GetDetail())
		}
	})

	t.Run("running", func(t *testing.T) {
		t.Parallel()

		got := motor.StatusFor(0.5, 0, nil)
		if got.GetState() != actuationv1.MotorStatus_STATE_RUNNING {
			t.Errorf("State = %v, want STATE_RUNNING", got.GetState())
		}
		if got.GetDutyCycle() != 0.5 {
			t.Errorf("DutyCycle = %v, want 0.5", got.GetDutyCycle())
		}
	})

	t.Run("fault takes priority over nonzero duty", func(t *testing.T) {
		t.Parallel()

		wantErr := errors.New("boom")
		got := motor.StatusFor(0.5, 0, wantErr)
		if got.GetState() != actuationv1.MotorStatus_STATE_FAULT {
			t.Errorf("State = %v, want STATE_FAULT", got.GetState())
		}
		if got.GetDetail() != wantErr.Error() {
			t.Errorf("Detail = %q, want %q", got.GetDetail(), wantErr.Error())
		}
	})

	t.Run("command age is carried through", func(t *testing.T) {
		t.Parallel()

		got := motor.StatusFor(0, 750*time.Millisecond, nil)
		if got.GetCommandAgeMs() != 750 {
			t.Errorf("CommandAgeMs = %v, want 750", got.GetCommandAgeMs())
		}
	})
}

func TestStatusFor_FrameID(t *testing.T) {
	t.Parallel()

	if got := motor.StatusFor(0, 0, nil).GetFrameId(); got != motor.FrameID {
		t.Errorf("FrameId = %q, want %q", got, motor.FrameID)
	}
}
