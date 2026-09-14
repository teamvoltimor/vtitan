package controllers

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// TestAgreesWithPathSense ports TestAgreesWithPathSense: the predicate behind
// TargetSenseGate, pinned on its own terms so the gate cannot be changed
// without this noticing.
func TestAgreesWithPathSense(t *testing.T) {
	t.Parallel()

	path := []trackmodel.Waypoint{{X: 0.0, Y: 0.0}, {X: 1.0, Y: 0.0}, {X: 2.0, Y: 0.0}}

	tests := []struct {
		name   string
		dx, dy float64
		dist   float64
		path   []trackmodel.Waypoint
		index  int
		want   bool
	}{
		{name: "approaching along the path agrees", dx: 1.0, dy: 0.0, dist: 1.0, path: path, index: 1, want: true},
		{
			name: "approaching against the path disagrees",
			dx:   -1.0, dy: 0.0, dist: 1.0,
			path: path, index: 1, want: false,
		},
		{
			name:  "perpendicular approach disagrees (strict boundary)",
			dx:    0.0,
			dy:    1.0,
			dist:  1.0,
			path:  path,
			index: 1,
			want:  false,
		},
		{
			name: "duplicated waypoint has no direction and cannot disagree",
			dx:   -1.0, dy: 0.0, dist: 1.0,
			path:  []trackmodel.Waypoint{{X: 0.0, Y: 0.0}, {X: 1.0, Y: 0.0}, {X: 1.0, Y: 0.0}},
			index: 1, want: true,
		},
		{
			name: "zero distance cannot disagree",
			dx:   0.0, dy: 0.0, dist: 0.0,
			path:  []trackmodel.Waypoint{{X: 0.0, Y: 0.0}, {X: 1.0, Y: 0.0}},
			index: 0, want: true,
		},
		{name: "index wraps past the end", dx: 1.0, dy: 0.0, dist: 1.0, path: path, index: 3, want: true},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()
			if got := agreesWithPathSense(tt.dx, tt.dy, tt.dist, tt.path, tt.index); got != tt.want {
				t.Errorf("agreesWithPathSense(%v, %v, %v, %v) = %v, want %v",
					tt.dx, tt.dy, tt.dist, tt.index, got, tt.want)
			}
		})
	}
}
