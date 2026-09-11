package main

import (
	"log/slog"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/corridorestimator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/startconditions"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/waypoints"
)

// testWriter adapts *testing.T to io.Writer so slog output surfaces in test
// logs instead of stderr.
type testWriter struct{ t *testing.T }

func (w testWriter) Write(p []byte) (int, error) {
	w.t.Log(string(p))
	return len(p), nil
}

func TestParseDirection(t *testing.T) {
	t.Parallel()

	cw, known, err := parseDirection(directionCW)
	if err != nil || cw != trackmodel.Clockwise || known == nil || *known != trackmodel.Clockwise {
		t.Errorf("parseDirection(cw) = (%v, %v, %v), want (Clockwise, &Clockwise, nil)", cw, known, err)
	}

	ccw, known, err := parseDirection(directionCCW)
	if err != nil || ccw != trackmodel.Counterclockwise || known == nil || *known != trackmodel.Counterclockwise {
		t.Errorf("parseDirection(ccw) = (%v, %v, %v), want (Counterclockwise, &Counterclockwise, nil)", ccw, known, err)
	}

	provisional, known, err := parseDirection(directionUndetermined)
	if err != nil || provisional != trackmodel.Clockwise || known != nil {
		t.Errorf("parseDirection(undetermined) = (%v, %v, %v), want (Clockwise, nil, nil)", provisional, known, err)
	}

	if _, _, parseErr := parseDirection("sideways"); parseErr == nil {
		t.Error("parseDirection(sideways) = nil error, want a rejection")
	}
}

func TestNewBlindLayout(t *testing.T) {
	t.Parallel()

	logger := slog.New(slog.NewTextHandler(testWriter{t}, nil))
	base := waypoints.PlannerInput{MaxCoordM: 3.0, ChassisWidthM: 0.194}

	path, geometry, layout, err := newBlindLayout(
		logger,
		base,
		trackmodel.Clockwise,
		waypoints.DefaultConfig(),
		startconditions.DefaultConfig(),
		corridorestimator.DefaultConfig(),
	)
	if err != nil {
		t.Fatalf("newBlindLayout: %v", err)
	}
	if len(path) == 0 {
		t.Error("newBlindLayout: path is empty, want a planned prior-layout path")
	}
	if layout == nil {
		t.Fatal("newBlindLayout: layout is nil, want a belief loop to correct the prior")
	}
	for section, got := range geometry.ToWidthsDict() {
		if got != blindNarrowWidthM {
			t.Errorf("geometry width for %v = %v, want the NARROW prior %v", section, got, blindNarrowWidthM)
		}
	}
}
