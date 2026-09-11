package button_test

import (
	"testing"
	"time"

	"github.com/go-playground/validator/v10"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/button"
)

// TestThresholds_Validate exercises the validate tags on button.Thresholds
// directly (no GPIO/build-tag dependency, since Thresholds lives outside
// driver.go), matching how Driver.New validates the embedded struct in
// practice.
func TestThresholds_Validate(t *testing.T) {
	t.Parallel()

	tests := []struct {
		Name       string
		Thresholds button.Thresholds
		WantErr    bool
	}{
		{
			Name:       "DefaultThresholds is valid",
			Thresholds: button.DefaultThresholds(),
			WantErr:    false,
		},
		{
			Name: "zero DebounceInterval is invalid",
			Thresholds: button.Thresholds{
				DebounceInterval:       0,
				LongPressThreshold:     3 * time.Second,
				ShutdownPressThreshold: 10 * time.Second,
			},
			WantErr: true,
		},
		{
			Name: "zero LongPressThreshold is invalid",
			Thresholds: button.Thresholds{
				DebounceInterval:       50 * time.Millisecond,
				LongPressThreshold:     0,
				ShutdownPressThreshold: 10 * time.Second,
			},
			WantErr: true,
		},
		{
			Name: "ShutdownPressThreshold at or below LongPressThreshold is invalid",
			Thresholds: button.Thresholds{
				DebounceInterval:       50 * time.Millisecond,
				LongPressThreshold:     3 * time.Second,
				ShutdownPressThreshold: 3 * time.Second,
			},
			WantErr: true,
		},
		{
			Name: "ShutdownPressThreshold above LongPressThreshold is valid",
			Thresholds: button.Thresholds{
				DebounceInterval:       50 * time.Millisecond,
				LongPressThreshold:     3 * time.Second,
				ShutdownPressThreshold: 3*time.Second + time.Millisecond,
			},
			WantErr: false,
		},
	}

	for _, tt := range tests {
		t.Run(tt.Name, func(t *testing.T) {
			t.Parallel()

			err := validator.New().Struct(tt.Thresholds)
			if (err != nil) != tt.WantErr {
				t.Errorf("validate(%+v) error = %v, wantErr %v", tt.Thresholds, err, tt.WantErr)
			}
		})
	}
}

// TestKind_String locks down the wire-style spellings Kind.String returns,
// matching platform/robot/src/hardware/button/event.py's ButtonEvent
// values -- a silent drift here would be invisible everywhere String is
// used for logging/debug output.
func TestKind_String(t *testing.T) {
	t.Parallel()

	tests := []struct {
		Kind button.Kind
		Want string
	}{
		{Kind: button.KindPressed, Want: "pressed"},
		{Kind: button.KindLongPress, Want: "long_press"},
		{Kind: button.KindShutdownPress, Want: "shutdown_press"},
		{Kind: button.KindShortPress, Want: "short_press"},
		{Kind: button.KindReleased, Want: "released"},
	}

	for _, tt := range tests {
		t.Run(tt.Want, func(t *testing.T) {
			t.Parallel()

			if got := tt.Kind.String(); got != tt.Want {
				t.Errorf("%v.String() = %q, want %q", tt.Kind, got, tt.Want)
			}
		})
	}
}
