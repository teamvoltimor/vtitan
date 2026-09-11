package button_test

import (
	"testing"
	"time"

	"buf.build/go/protovalidate"

	nodebutton "github.com/teamvoltimor/vtitan/platform/robot-go/internal/node/button"
)

func TestHoldMessageFor(t *testing.T) {
	t.Parallel()

	got := nodebutton.HoldMessageFor(1500*time.Millisecond, []nodebutton.HoldThreshold{
		{At: 3 * time.Second, Kind: "reset"},
		{At: 5 * time.Second, Kind: "shutdown"},
	})

	if got.GetHeldS() != 1.5 {
		t.Fatalf("HeldS = %v, want 1.5", got.GetHeldS())
	}
	if len(got.GetThresholds()) != 2 {
		t.Fatalf("got %d thresholds, want 2", len(got.GetThresholds()))
	}
	if got.GetThresholds()[0].GetAtS() != 3 || got.GetThresholds()[0].GetKind() != "reset" {
		t.Fatalf("threshold[0] = %v, want 3s/reset", got.GetThresholds()[0])
	}
}

// TestHoldMessageFor_NoThresholds covers the OLED's requirement that a
// partial frame is still usable: a producer with nothing configured still
// reports how long the button has been held.
func TestHoldMessageFor_NoThresholds(t *testing.T) {
	t.Parallel()

	got := nodebutton.HoldMessageFor(200*time.Millisecond, nil)

	if got.GetHeldS() != 0.2 {
		t.Fatalf("HeldS = %v, want 0.2", got.GetHeldS())
	}
	if len(got.GetThresholds()) != 0 {
		t.Fatalf("got %d thresholds, want none", len(got.GetThresholds()))
	}
}

// TestHoldMessageFor_DropsUnnamedThresholds guards the schema's non-empty
// kind constraint: passing one through would build a message that fails
// validation at the publisher, where the cause is far less obvious.
func TestHoldMessageFor_DropsUnnamedThresholds(t *testing.T) {
	t.Parallel()

	validator, err := protovalidate.New()
	if err != nil {
		t.Fatalf("protovalidate.New: %v", err)
	}

	got := nodebutton.HoldMessageFor(time.Second, []nodebutton.HoldThreshold{
		{At: 2 * time.Second, Kind: ""},
		{At: 4 * time.Second, Kind: "shutdown"},
	})

	if len(got.GetThresholds()) != 1 {
		t.Fatalf("got %d thresholds, want the unnamed one dropped", len(got.GetThresholds()))
	}
	if validateErr := validator.Validate(got); validateErr != nil {
		t.Fatalf("Validate() = %v, want nil", validateErr)
	}
}
