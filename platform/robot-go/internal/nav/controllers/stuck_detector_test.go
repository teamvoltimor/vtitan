package controllers_test

import (
	"strings"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// TestNewStuckDetector_HistorySizeSmallerThanTimeoutFramesRejected ports
// test_history_size_smaller_than_timeout_frames_rejected: the position
// history would evict entries before the timeout window is reached,
// silently making the stuck check less sensitive.
func TestNewStuckDetector_HistorySizeSmallerThanTimeoutFramesRejected(t *testing.T) {
	t.Parallel()

	_, err := controllers.NewStuckDetector(0.03, 40, 10, 3, controllers.DefaultMinHistoryForDistance, nil)
	if err == nil {
		t.Fatal("NewStuckDetector(historySize=10, timeoutFrames=40) err = nil, want an error")
	}
	if !strings.Contains(err.Error(), "history_size") {
		t.Errorf("err = %q, want it to mention history_size", err.Error())
	}
}

// TestNewStuckDetector_HistorySizeEqualToTimeoutFramesIsAllowed ports
// test_history_size_equal_to_timeout_frames_is_allowed.
func TestNewStuckDetector_HistorySizeEqualToTimeoutFramesIsAllowed(t *testing.T) {
	t.Parallel()

	_, err := controllers.NewStuckDetector(0.03, 40, 40, 3, controllers.DefaultMinHistoryForDistance, nil)
	if err != nil {
		t.Errorf("NewStuckDetector(historySize=40, timeoutFrames=40) err = %v, want nil", err)
	}
}

// TestUpdate_DeclaresStuckAfterTimeoutWithoutMovement ports
// test_declares_stuck_after_timeout_without_movement.
func TestUpdate_DeclaresStuckAfterTimeoutWithoutMovement(t *testing.T) {
	t.Parallel()

	detector, err := controllers.NewStuckDetector(0.03, 5, 10, 3, controllers.DefaultMinHistoryForDistance, nil)
	if err != nil {
		t.Fatalf("NewStuckDetector() err = %v", err)
	}

	stuck := false
	for range 20 {
		stuck = detector.Update(trackmodel.Waypoint{})
	}
	if !stuck {
		t.Error("stuck = false after 20 stationary updates, want true")
	}
}

// TestUpdate_NotStuckWhenMoving ports test_not_stuck_when_moving.
func TestUpdate_NotStuckWhenMoving(t *testing.T) {
	t.Parallel()

	detector, err := controllers.NewStuckDetector(0.03, 5, 10, 3, controllers.DefaultMinHistoryForDistance, nil)
	if err != nil {
		t.Fatalf("NewStuckDetector() err = %v", err)
	}

	stuck := true
	for i := range 20 {
		stuck = detector.Update(trackmodel.Waypoint{X: 0.1 * float64(i), Y: 0.0})
	}
	if stuck {
		t.Error("stuck = true after 20 moving updates, want false")
	}
}

// TestNewStuckDetector_NilLoggerFallsBackToDefault has no Python oracle --
// StuckDetector.__init__ never accepted a logger (Python uses the module
// logger global). NewStuckDetector's doc comment states a nil logger falls
// back to slog.Default() specifically so callers aren't required to thread
// one through; this exercises that fallback wouldn't panic on the first
// Warn call a real stuck declaration makes.
func TestNewStuckDetector_NilLoggerFallsBackToDefault(t *testing.T) {
	t.Parallel()

	detector, err := controllers.NewStuckDetector(0.03, 5, 10, 3, controllers.DefaultMinHistoryForDistance, nil)
	if err != nil {
		t.Fatalf("NewStuckDetector() err = %v", err)
	}

	stuck := false
	for range 20 {
		stuck = detector.Update(trackmodel.Waypoint{})
	}
	if !stuck {
		t.Error("stuck = false, want true (nil logger must not prevent normal operation)")
	}
}

// TestReset_ClearsStuckState has no dedicated Python oracle test, but
// StuckDetector.reset's doc comment states its purpose directly ("e.g. after
// an escape maneuver"): a caller that resets mid-stuck-declaration must see
// a clean slate, not a detector that immediately re-declares stuck on the
// very next stationary update.
func TestReset_ClearsStuckState(t *testing.T) {
	t.Parallel()

	detector, err := controllers.NewStuckDetector(0.03, 5, 10, 3, controllers.DefaultMinHistoryForDistance, nil)
	if err != nil {
		t.Fatalf("NewStuckDetector() err = %v", err)
	}
	for range 20 {
		detector.Update(trackmodel.Waypoint{})
	}
	detector.Reset()

	if got := detector.GetDiagnostics(); got.IsStuck || got.StuckCount != 0 || got.HistorySize != 0 {
		t.Errorf("GetDiagnostics() after Reset() = %+v, want a clean slate", got)
	}
}
