package trackconfig_test

import (
	"strings"
	"testing"

	"github.com/shopspring/decimal"

	"vtitan/gazebo/generator/internal/trackconfig"
)

func dec(s string) decimal.Decimal {
	d, err := decimal.NewFromString(s)
	if err != nil {
		panic(err)
	}
	return d
}

// baseConfig is the real mat layout, built in code so tests can perturb one
// field at a time without touching the checked-in TOML.
func baseConfig() trackconfig.Config {
	return trackconfig.Config{
		Track: trackconfig.Track{
			MatSize:   dec("3.2"),
			Size:      dec("3.0"),
			MinCoord:  dec("0.0"),
			MaxCoord:  dec("3.0"),
			CornerMin: dec("1.0"),
			CornerMax: dec("2.0"),
		},
		Corridor: trackconfig.Corridor{
			Narrow:        dec("0.6"),
			Wide:          dec("1.0"),
			DivisionLines: []decimal.Decimal{dec("0.40"), dec("0.60")},
		},
		StartingZone: trackconfig.StartingZone{
			DefaultLength:  dec("0.5"),
			SpawnAlignment: []string{"inner", "outer", "outer"},
		},
	}
}

// TestBandWidthsAreExact is the reason this package works in decimals at all:
// in float64 the middle band comes out 0.19999999999999996, and the generator
// would emit that into the file that is supposed to be the source of truth.
func TestBandWidthsAreExact(t *testing.T) {
	bands := baseConfig().Corridor.BandWidths()

	want := []string{"0.4", "0.2", "0.4"}
	if len(bands) != len(want) {
		t.Fatalf("got %d bands, want %d", len(bands), len(want))
	}
	for i, w := range want {
		if got := bands[i].String(); got != w {
			t.Errorf("band %d = %s, want %s", i, got, w)
		}
	}

	// The same subtraction in float64, for contrast. The operands have to be
	// variables: Go evaluates untyped constant arithmetic exactly, so writing
	// `0.6-0.4 == 0.2` here would pass and prove nothing. That is also why the
	// hazard is invisible in the hand-written Go constants this replaced and
	// very visible in Python, which has no untyped constants.
	inner, outer, middle := 0.6, 0.4, 0.2
	if inner-outer == middle {
		t.Error("float64 0.6-0.4 now equals 0.2; the decimal rationale needs revisiting")
	}
}

func TestDerivedTrackValues(t *testing.T) {
	cfg := baseConfig()
	if got := cfg.Track.CenterCoord().String(); got != "1.5" {
		t.Errorf("CenterCoord = %s, want 1.5", got)
	}
	if got := cfg.Track.CornerSize().String(); got != "1" {
		t.Errorf("CornerSize = %s, want 1", got)
	}
	left, right := cfg.CellCentersAlong()
	if left.String() != "1.25" || right.String() != "1.75" {
		t.Errorf("CellCentersAlong = %s, %s; want 1.25, 1.75", left, right)
	}
}

func TestValidateAcceptsTheRealLayout(t *testing.T) {
	if err := baseConfig().Validate(dec("0.20")); err != nil {
		t.Errorf("the checked-in layout must validate: %v", err)
	}
}

// TestSpawnOffsetsHugTheChosenEdge is the placement rule: the chassis is never
// centred in its band. Band 0 hugs its inner edge because its outer edge is the
// outer wall; bands 1 and 2 hug their outer edge because what lies beyond them
// is the inner block. With a 0.194 m chassis in the 0.20 m middle band, that is
// the difference between 3 mm and 6 mm of clearance from the block.
func TestSpawnOffsetsHugTheChosenEdge(t *testing.T) {
	offsets, err := baseConfig().SpawnOffsets(dec("0.194"))
	if err != nil {
		t.Fatalf("SpawnOffsets: %v", err)
	}
	want := []string{"0.303", "0.497", "0.697"}
	for i, w := range want {
		if got := offsets[i].String(); got != w {
			t.Errorf("offset %d = %s, want %s", i, got, w)
		}
	}
}

// TestSpawnOffsetsFollowTheChassis pins that the placement is derived, not
// declared: a re-measured chassis moves the offsets by itself. The middle band
// is only millimetres wider than the robot, so a hardcoded offset would quietly
// stop meaning "flush against the edge".
func TestSpawnOffsetsFollowTheChassis(t *testing.T) {
	cfg := baseConfig()
	narrow, err := cfg.SpawnOffsets(dec("0.194"))
	if err != nil {
		t.Fatalf("SpawnOffsets: %v", err)
	}
	wide, err := cfg.SpawnOffsets(dec("0.20"))
	if err != nil {
		t.Fatalf("SpawnOffsets: %v", err)
	}
	if narrow[1].Equal(wide[1]) {
		t.Error("the middle-band offset did not move when the chassis width changed")
	}
	// A chassis that exactly fills the middle band has only one placement.
	if got := wide[1].String(); got != "0.5" {
		t.Errorf("a 0.20 m chassis in the 0.20 m band must sit at 0.5, got %s", got)
	}
}

// TestValidateRejectsChassisWiderThanABand guards the case that motivated the
// re-measurement: a band narrower than the robot has no legal placement at all.
func TestValidateRejectsChassisWiderThanABand(t *testing.T) {
	err := baseConfig().Validate(dec("0.21"))
	if err == nil || !strings.Contains(err.Error(), "narrower than") {
		t.Errorf("want an error about the band being too narrow, got %v", err)
	}
}

func TestValidateRejects(t *testing.T) {
	for _, tc := range []struct {
		name   string
		mutate func(*trackconfig.Config)
		want   string
	}{
		{
			name:   "an alignment that is not a band edge",
			mutate: func(c *trackconfig.Config) { c.StartingZone.SpawnAlignment[1] = "centre" },
			want:   `want "outer" or "inner"`,
		},
		{
			name: "one alignment too few",
			mutate: func(c *trackconfig.Config) {
				c.StartingZone.SpawnAlignment = c.StartingZone.SpawnAlignment[:2]
			},
			want: "one per band",
		},
		{
			name:   "division lines out of order",
			mutate: func(c *trackconfig.Config) { c.Corridor.DivisionLines[1] = dec("0.30") },
			want:   "does not increase",
		},
		{
			name:   "division line outside the corridor",
			mutate: func(c *trackconfig.Config) { c.Corridor.DivisionLines[1] = dec("1.20") },
			want:   "outside the",
		},
		{
			name:   "no division lines",
			mutate: func(c *trackconfig.Config) { c.Corridor.DivisionLines = nil },
			want:   "is empty",
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			cfg := baseConfig()
			tc.mutate(&cfg)
			err := cfg.Validate(dec("0.20"))
			if err == nil {
				t.Fatalf("want an error mentioning %q, got nil", tc.want)
			}
			if !strings.Contains(err.Error(), tc.want) {
				t.Errorf("error %q does not mention %q", err, tc.want)
			}
		})
	}
}
