package signrouter_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/waypoints"
)

// TestRoutingTable_AxisAndMultiplierPerCorridorAndDirection matches
// routing.py's ROUTING_TABLE literal: the deformed waypoint moves to the
// vehicle's own RIGHT of red and its own LEFT of green, for the direction
// actually driven -- which means the CW rows are the NEGATION of the CCW
// rows (the vehicle's right is the outer wall counterclockwise and the
// inner square clockwise), not identical to them. See routingTable's doc
// comment for the worked example.
func TestRoutingTable_AxisAndMultiplierPerCorridorAndDirection(t *testing.T) {
	t.Parallel()

	want := map[signrouter.RoutingKey]signrouter.RoutingEntry{
		{Corridor: trackmodel.South, Direction: trackmodel.Counterclockwise}: {
			Axis:      signrouter.AxisY,
			RedMult:   -1,
			GreenMult: +1,
		},
		{Corridor: trackmodel.North, Direction: trackmodel.Counterclockwise}: {
			Axis:      signrouter.AxisY,
			RedMult:   +1,
			GreenMult: -1,
		},
		{Corridor: trackmodel.East, Direction: trackmodel.Counterclockwise}: {
			Axis:      signrouter.AxisX,
			RedMult:   +1,
			GreenMult: -1,
		},
		{Corridor: trackmodel.West, Direction: trackmodel.Counterclockwise}: {
			Axis:      signrouter.AxisX,
			RedMult:   -1,
			GreenMult: +1,
		},
		{Corridor: trackmodel.South, Direction: trackmodel.Clockwise}: {
			Axis:      signrouter.AxisY,
			RedMult:   +1,
			GreenMult: -1,
		},
		{Corridor: trackmodel.North, Direction: trackmodel.Clockwise}: {
			Axis:      signrouter.AxisY,
			RedMult:   -1,
			GreenMult: +1,
		},
		{Corridor: trackmodel.East, Direction: trackmodel.Clockwise}: {
			Axis:      signrouter.AxisX,
			RedMult:   -1,
			GreenMult: +1,
		},
		{Corridor: trackmodel.West, Direction: trackmodel.Clockwise}: {
			Axis:      signrouter.AxisX,
			RedMult:   +1,
			GreenMult: -1,
		},
	}

	got := signrouter.RoutingTable()
	if len(got) != len(want) {
		t.Fatalf("RoutingTable() has %d entries, want %d", len(got), len(want))
	}
	for key, wantEntry := range want {
		gotEntry, ok := got[key]
		if !ok {
			t.Errorf("RoutingTable()[%+v] missing", key)
			continue
		}
		if gotEntry != wantEntry {
			t.Errorf("RoutingTable()[%+v] = %+v, want %+v", key, gotEntry, wantEntry)
		}
	}
}

// TestPassSideLateralAxis_MatchesRoutingTableForItsOwnDirection matches
// TestPassSideLateralAxis.test_matches_routing_table_for_its_own_direction:
// the lookup is direction-KEYED, not direction-agnostic -- it must agree
// with ROUTING_TABLE[(section, direction)] for the SAME direction passed
// in, not either one. Was TestOutwardLateralAxis, which asserted the
// lookup gave the same answer for both directions; that held only while
// ROUTING_TABLE's CW/CCW rows were identical, which was itself the bug
// (see routingTable's doc comment) -- the old test could not have failed
// on the bug it was covering, because it asserted the bug.
func TestPassSideLateralAxis_MatchesRoutingTableForItsOwnDirection(t *testing.T) {
	t.Parallel()

	sections := []trackmodel.Section{
		trackmodel.South,
		trackmodel.North,
		trackmodel.East,
		trackmodel.West,
	}
	colors := []signrouter.SignColor{signrouter.SignColorRed, signrouter.SignColorGreen}
	directions := []trackmodel.Direction{trackmodel.Clockwise, trackmodel.Counterclockwise}
	table := signrouter.RoutingTable()

	for _, section := range sections {
		for _, color := range colors {
			for _, direction := range directions {
				entry := table[signrouter.RoutingKey{Corridor: section, Direction: direction}]
				wantMult := entry.RedMult
				if color == signrouter.SignColorGreen {
					wantMult = entry.GreenMult
				}

				gotAxis, gotMult, ok := signrouter.PassSideLateralAxis(section, color, direction)
				if !ok {
					t.Fatalf("PassSideLateralAxis(%v, %v, %v) ok = false", section, color, direction)
				}
				if gotAxis != entry.Axis || gotMult != wantMult {
					t.Errorf(
						"PassSideLateralAxis(%v, %v, %v) = (%v, %v), want (%v, %v)",
						section,
						color,
						direction,
						gotAxis,
						gotMult,
						entry.Axis,
						wantMult,
					)
				}
			}
		}
	}
}

// TestPassSideLateralAxis_DirectionsAreOpposite matches
// TestPassSideLateralAxis.test_directions_are_opposite: the whole point of
// the fix -- CW and CCW must never agree, since "the vehicle's right"
// names opposite world directions depending on which way it drives.
func TestPassSideLateralAxis_DirectionsAreOpposite(t *testing.T) {
	t.Parallel()

	sections := []trackmodel.Section{
		trackmodel.South,
		trackmodel.North,
		trackmodel.East,
		trackmodel.West,
	}
	colors := []signrouter.SignColor{signrouter.SignColorRed, signrouter.SignColorGreen}

	for _, section := range sections {
		for _, color := range colors {
			cwAxis, cwMult, cwOK := signrouter.PassSideLateralAxis(section, color, trackmodel.Clockwise)
			ccwAxis, ccwMult, ccwOK := signrouter.PassSideLateralAxis(
				section, color, trackmodel.Counterclockwise,
			)
			if !cwOK || !ccwOK {
				t.Fatalf("PassSideLateralAxis(%v, %v, ...) ok = false", section, color)
			}
			if cwAxis != ccwAxis {
				t.Errorf("%v/%v: axis CW %v != CCW %v, want same corridor same axis",
					section, color, cwAxis, ccwAxis)
			}
			if cwMult != -ccwMult {
				t.Errorf("%v/%v: multiplier CW %v, CCW %v, want opposites", section, color, cwMult, ccwMult)
			}
		}
	}
}

// TestClampLateral_LowSideCorridorClampsBothBounds matches clamp_lateral's
// asymmetric-by-corridor-side behavior for SOUTH/WEST (bordering the inner
// square on their HIGH side): a value pushed toward the inner square clamps
// below TrackCornerMinM, and one pushed toward the outer wall clamps above
// TrackMinCoordM.
func TestClampLateral_LowSideCorridorClampsBothBounds(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	wallClearance := cfg.ChassisHalfDiagonalM + cfg.WallClearanceMarginM

	if got, want := signrouter.ClampLateral(
		10.0,
		trackmodel.South,
		cfg,
	), cfg.TrackCornerMinM-wallClearance; got != want {
		t.Errorf("ClampLateral(10.0, South) = %v, want %v (inner-square side)", got, want)
	}
	if got, want := signrouter.ClampLateral(
		-10.0,
		trackmodel.West,
		cfg,
	), cfg.TrackMinCoordM+wallClearance; got != want {
		t.Errorf("ClampLateral(-10.0, West) = %v, want %v (outer-wall side)", got, want)
	}
}

// TestClampLateral_HighSideCorridorClampsBothBounds matches clamp_lateral's
// behavior for NORTH/EAST (bordering the inner square on their LOW side).
func TestClampLateral_HighSideCorridorClampsBothBounds(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	wallClearance := cfg.ChassisHalfDiagonalM + cfg.WallClearanceMarginM

	if got, want := signrouter.ClampLateral(
		-10.0,
		trackmodel.North,
		cfg,
	), cfg.TrackCornerMaxM+wallClearance; got != want {
		t.Errorf("ClampLateral(-10.0, North) = %v, want %v (inner-square side)", got, want)
	}
	if got, want := signrouter.ClampLateral(10.0, trackmodel.East, cfg), cfg.TrackMaxCoordM-wallClearance; got != want {
		t.Errorf("ClampLateral(10.0, East) = %v, want %v (outer-wall side)", got, want)
	}
}

// TestCandidateCorridors_StraightIsASingleSection matches
// candidate_corridors' behavior on a point squarely along one corridor's
// straight: exactly one candidate.
func TestCandidateCorridors_StraightIsASingleSection(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	got := signrouter.CandidateCorridors(1.5, 0.4, cfg)
	if len(got) != 1 || got[0] != trackmodel.South {
		t.Errorf("CandidateCorridors(1.5, 0.4) = %v, want [South]", got)
	}
}

// TestCandidateCorridors_CornerHasTwoAdjacentFaces matches
// candidate_corridors' behavior in a CORNER (both coordinates outside the
// inner square): the two adjacent faces, not a single nearest pick -- the
// ambiguity DepthConsistentCorridor/CorridorForPosition resolve.
func TestCandidateCorridors_CornerHasTwoAdjacentFaces(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	got := signrouter.CandidateCorridors(2.40, 2.003, cfg)
	if len(got) != 2 {
		t.Fatalf("CandidateCorridors(2.40, 2.003) = %v, want 2 candidates", got)
	}
	want := map[trackmodel.Section]bool{trackmodel.North: true, trackmodel.East: true}
	for _, c := range got {
		if !want[c] {
			t.Errorf("CandidateCorridors(2.40, 2.003) contains unexpected %v", c)
		}
	}
}

// TestDepthConsistentCorridor_FixesTheTracedMisfile matches
// TestSignCorridorHysteresis.test_the_traced_flip_no_longer_happens_at_all's
// premise: these two coordinates are the pair traced on go_obstacles_0000,
// where nearest-face flipped EAST/NORTH on every tick of an approach.
// x=2.40 is a LATERAL value and y~2.00 a DEPTH value, so both points
// describe an EAST sign -- DepthConsistentCorridor must resolve both to
// EAST regardless of which face nearest-face happened to pick, fixing a
// measured 42.1% corner misfile rate.
func TestDepthConsistentCorridor_FixesTheTracedMisfile(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	points := []struct {
		name string
		x, y float64
	}{
		// Not actually a corner (single candidate, East): fallback passes
		// straight through, already East.
		{"east_side", 2.40, 1.997},
		// A genuine corner: nearest-face picks NORTH (the misfile), but the
		// point's real depth lies along EAST's straight.
		{"north_side", 2.40, 2.003},
	}
	for _, p := range points {
		t.Run(p.name, func(t *testing.T) {
			t.Parallel()
			fallback := waypoints.CorridorForPosition(
				p.x,
				p.y,
				cfg.TrackCornerMinM,
				cfg.TrackCornerMaxM,
			)
			got := signrouter.DepthConsistentCorridor(p.x, p.y, fallback, cfg)
			if got != trackmodel.East {
				t.Errorf(
					"DepthConsistentCorridor(%v, %v, fallback=%v) = %v, want East",
					p.x,
					p.y,
					fallback,
					got,
				)
			}
		})
	}
}

// TestDepthConsistentCorridor_TrueCornerDiagonalCanDisagree matches
// TestSignCorridorHysteresis.test_the_diagonal_positions_really_do_straddle_a_corridor_boundary:
// a TRUE corner diagonal (both coordinates outside the inner square, so
// neither axis carries a legal depth) can still resolve EAST vs NORTH
// depending on which coordinate is fractionally less far out -- this is not
// a misfile like the traced pair above, just the genuine ambiguity the
// (unported, discovery-only) corridor-flip hysteresis exists to damp.
func TestDepthConsistentCorridor_TrueCornerDiagonalCanDisagree(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	eastFallback := waypoints.CorridorForPosition(
		2.010,
		2.003,
		cfg.TrackCornerMinM,
		cfg.TrackCornerMaxM,
	)
	east := signrouter.DepthConsistentCorridor(2.010, 2.003, eastFallback, cfg)
	if east != trackmodel.East {
		t.Errorf(
			"DepthConsistentCorridor(2.010, 2.003, fallback=%v) = %v, want East",
			eastFallback,
			east,
		)
	}
	northFallback := waypoints.CorridorForPosition(
		2.003,
		2.010,
		cfg.TrackCornerMinM,
		cfg.TrackCornerMaxM,
	)
	north := signrouter.DepthConsistentCorridor(2.003, 2.010, northFallback, cfg)
	if north != trackmodel.North {
		t.Errorf(
			"DepthConsistentCorridor(2.003, 2.010, fallback=%v) = %v, want North",
			northFallback,
			north,
		)
	}
}

// TestDepthConsistentCorridor_FewerThanTwoCandidatesKeepsFallback matches
// depth_consistent_corridor's short-circuit: a point squarely on one
// corridor's straight has only one CandidateCorridors entry, so the depth
// tie-break never runs and the fallback (whatever CorridorForPosition
// already said) passes through unchanged.
func TestDepthConsistentCorridor_FewerThanTwoCandidatesKeepsFallback(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	got := signrouter.DepthConsistentCorridor(1.5, 0.4, trackmodel.South, cfg)
	if got != trackmodel.South {
		t.Errorf(
			"DepthConsistentCorridor(1.5, 0.4, South) = %v, want South (fallback unchanged)",
			got,
		)
	}
}

// TestIsSquarelyInCorridor_TrueOnTheStraightWithinTheDepthBuffer and its
// siblings match is_squarely_in_corridor's two independent checks: the
// lateral axis must still read as this corridor, and the depth axis must
// stay within DeformDepthBufferM of the inner square's own span.
func TestIsSquarelyInCorridor_TrueOnTheStraightWithinTheDepthBuffer(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	if !signrouter.IsSquarelyInCorridor(1.5, 0.4, trackmodel.South, cfg) {
		t.Error("IsSquarelyInCorridor(1.5, 0.4, South) = false, want true")
	}
}

func TestIsSquarelyInCorridor_FalseWhenLateralHasCrossedIntoTheCorridor(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	// y >= TrackCornerMinM: no longer south of the inner square at all.
	if signrouter.IsSquarelyInCorridor(1.5, cfg.TrackCornerMinM, trackmodel.South, cfg) {
		t.Error("IsSquarelyInCorridor(1.5, TrackCornerMinM, South) = true, want false")
	}
}

func TestIsSquarelyInCorridor_FalseWhenDepthIsPastTheBufferedCorner(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	pastBuffer := cfg.TrackCornerMaxM + cfg.DeformDepthBufferM + 0.01
	if signrouter.IsSquarelyInCorridor(pastBuffer, 0.4, trackmodel.South, cfg) {
		t.Error("IsSquarelyInCorridor(pastBuffer, 0.4, South) = true, want false")
	}
}
