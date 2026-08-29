package controllers_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
)

// There is no dedicated Python oracle test file for shared.domain.enums'
// RiskLevel/ThreatDirection/ManeuverType (they're plain str enums exercised
// only indirectly, e.g. via detect_threat_direction's string return values
// in test_collision_avoidance_controller.py). These pin the Go String()
// methods' labels directly, since enums.go's doc comment states each one is
// meant to match its Python counterpart's value/repr, and the out-of-range
// default case is Go-only surface area (an int-backed enum has no closed
// value set the way a Python str enum does).

// TestRiskLevel_String_MatchesPythonEnumValues pins the labels against
// shared.domain.enums.RiskLevel's values (used as detect_threat_direction's
// string results throughout the Python oracle suite).
func TestRiskLevel_String_MatchesPythonEnumValues(t *testing.T) {
	t.Parallel()

	cases := map[controllers.RiskLevel]string{
		controllers.RiskSafe:     "safe",
		controllers.RiskObstacle: "obstacle",
		controllers.RiskCritical: "critical",
	}
	for level, want := range cases {
		if got := level.String(); got != want {
			t.Errorf("RiskLevel(%d).String() = %q, want %q", level, got, want)
		}
	}
}

// TestRiskLevel_String_DefaultsToUnknown pins the fallback for a value
// outside the declared set -- Go-only surface area, since an int-backed enum
// has no closed value set the way Python's str enum does.
func TestRiskLevel_String_DefaultsToUnknown(t *testing.T) {
	t.Parallel()

	if got := controllers.RiskLevel(99).String(); got != "unknown" {
		t.Errorf("RiskLevel(99).String() = %q, want %q", got, "unknown")
	}
}

// TestThreatDirection_String_MatchesPythonEnumValues pins the labels against
// the exact strings test_collision_avoidance_controller.py asserts
// (detect_threat_direction(...) == "front"/"back"/"left"/"right"/"none").
func TestThreatDirection_String_MatchesPythonEnumValues(t *testing.T) {
	t.Parallel()

	cases := map[controllers.ThreatDirection]string{
		controllers.ThreatFront: "front",
		controllers.ThreatBack:  "back",
		controllers.ThreatLeft:  "left",
		controllers.ThreatRight: "right",
		controllers.ThreatNone:  "none",
	}
	for direction, want := range cases {
		if got := direction.String(); got != want {
			t.Errorf("ThreatDirection(%d).String() = %q, want %q", direction, got, want)
		}
	}
}

// TestThreatDirection_String_DefaultsToUnknown is the Go-only default-case
// analog of TestRiskLevel_String_DefaultsToUnknown.
func TestThreatDirection_String_DefaultsToUnknown(t *testing.T) {
	t.Parallel()

	if got := controllers.ThreatDirection(99).String(); got != "unknown" {
		t.Errorf("ThreatDirection(99).String() = %q, want %q", got, "unknown")
	}
}

// TestManeuverType_String_MatchesPythonEnumValues pins the labels against
// shared.domain.enums.ManeuverType's values.
func TestManeuverType_String_MatchesPythonEnumValues(t *testing.T) {
	t.Parallel()

	cases := map[controllers.ManeuverType]string{
		controllers.ManeuverKTurn:          "k_turn",
		controllers.ManeuverSideCorrection: "side_correction",
		controllers.ManeuverStuckReverse:   "stuck_reverse",
		controllers.ManeuverStuckForward:   "stuck_forward",
	}
	for maneuver, want := range cases {
		if got := maneuver.String(); got != want {
			t.Errorf("ManeuverType(%d).String() = %q, want %q", maneuver, got, want)
		}
	}
}

// TestManeuverType_String_DefaultsToUnknown is the Go-only default-case
// analog of TestRiskLevel_String_DefaultsToUnknown.
func TestManeuverType_String_DefaultsToUnknown(t *testing.T) {
	t.Parallel()

	if got := controllers.ManeuverType(99).String(); got != "unknown" {
		t.Errorf("ManeuverType(99).String() = %q, want %q", got, "unknown")
	}
}
