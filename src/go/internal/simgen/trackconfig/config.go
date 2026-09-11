// Package trackconfig loads the mat geometry TOML source of truth
// (src/config/track.toml) and renders it into the generated files
// each consumer (Go simconfig, Python shared.config) actually reads.
// Regenerate via `simgen generate-track-constants` (wired to
// `task gen:track-constants`).
//
// Every dimension is carried as a decimal.Decimal rather than a float64. The
// TOML holds exact decimals and the mat is specified in exact decimals, but
// float64 is not closed over them: 0.60 - 0.40 evaluates to
// 0.19999999999999996, and a generator that emitted that would be writing
// representation error into the very file meant to be the source of truth.
// Decimal arithmetic keeps the derived values exact, so the emitted literal is
// the decimal the mat actually has, and the band-fit checks below need no
// tolerance at all.
//
// The generated constants are still float/float64 for consumers. Decimals at
// runtime would not survive contact with numpy, which the raycast LIDAR and
// the risk assessment run on, and would put decimal arithmetic in a 20 Hz
// control loop.
package trackconfig

import (
	"fmt"
	"os"

	"github.com/pelletier/go-toml/v2"
	"github.com/shopspring/decimal"
)

type (
	// Config is the parsed contents of track.toml. All lengths are metres,
	// colours are normalized RGB triples.
	Config struct {
		Track        Track        `toml:"track"`
		Wall         Wall         `toml:"wall"`
		Corridor     Corridor     `toml:"corridor"`
		Sign         Sign         `toml:"sign"`
		Parking      Parking      `toml:"parking"`
		StartingZone StartingZone `toml:"starting_zone"`
	}

	// RGB is a normalized colour triple.
	RGB [3]decimal.Decimal

	// Track holds the mat and driveable-track extents and the corner region.
	Track struct {
		MatSize   decimal.Decimal `toml:"mat_size"`
		Size      decimal.Decimal `toml:"size"`
		MinCoord  decimal.Decimal `toml:"min_coord"`
		MaxCoord  decimal.Decimal `toml:"max_coord"`
		CornerMin decimal.Decimal `toml:"corner_min"`
		CornerMax decimal.Decimal `toml:"corner_max"`
	}

	// Wall holds the exterior and interior wall dimensions.
	Wall struct {
		Height             decimal.Decimal `toml:"height"`
		Thickness          decimal.Decimal `toml:"thickness"`
		CollisionThickness decimal.Decimal `toml:"collision_thickness"`
		ExteriorOffset     decimal.Decimal `toml:"exterior_offset"`
		InteriorOffset     decimal.Decimal `toml:"interior_offset"`
		Color              RGB             `toml:"color"`
	}

	// Corridor holds the legal corridor widths and the division lines that cut
	// every corridor lengthwise.
	Corridor struct {
		Narrow        decimal.Decimal   `toml:"narrow"`
		Wide          decimal.Decimal   `toml:"wide"`
		Obstacles     decimal.Decimal   `toml:"obstacles"`
		MinWidth      decimal.Decimal   `toml:"min_width"`
		MaxWidth      decimal.Decimal   `toml:"max_width"`
		DivisionLines []decimal.Decimal `toml:"division_lines"`
	}

	// Sign holds the traffic pillar dimensions, grid rows and colours.
	Sign struct {
		RedColor                RGB             `toml:"red_color"`
		GreenColor              RGB             `toml:"green_color"`
		RedColorStd             RGB             `toml:"red_std"`
		GreenColorStd           RGB             `toml:"green_std"`
		Width                   decimal.Decimal `toml:"width"`
		Depth                   decimal.Decimal `toml:"depth"`
		Height                  decimal.Decimal `toml:"height"`
		ZPosition               decimal.Decimal `toml:"z_position"`
		GridDepthNear           decimal.Decimal `toml:"grid_depth_near"`
		GridDepthMid            decimal.Decimal `toml:"grid_depth_middle"`
		GridDepthFar            decimal.Decimal `toml:"grid_depth_far"`
		PlacementCircleDiameter decimal.Decimal `toml:"placement_circle_diameter"`
		MinCount                int             `toml:"min_count"`
		MaxCount                int             `toml:"max_count"`
	}

	// Parking holds the magenta block dimensions and bay sizing.
	Parking struct {
		Length        decimal.Decimal `toml:"length"`
		Width         decimal.Decimal `toml:"width"`
		Height        decimal.Decimal `toml:"height"`
		ZPosition     decimal.Decimal `toml:"z_position"`
		WallOffset    decimal.Decimal `toml:"wall_offset"`
		SpacingFactor decimal.Decimal `toml:"spacing_factor"`
		Color         RGB             `toml:"color"`
	}

	// StartingZone holds the starting square's cell size, appearance and the
	// spawn offsets chosen within each band.
	StartingZone struct {
		DefaultLength       decimal.Decimal `toml:"default_length"`
		Thickness           decimal.Decimal `toml:"thickness"`
		ObstaclesSizeFactor decimal.Decimal `toml:"obstacles_size_factor"`
		IndicatorRadius     decimal.Decimal `toml:"indicator_radius"`
		Color               RGB             `toml:"color"`
		ClockwiseColor      RGB             `toml:"clockwise_color"`
		CounterClockwise    RGB             `toml:"counterclockwise_color"`
		SpawnAlignment      []string        `toml:"spawn_alignment"`
	}
)

// Band edges a spawn can be aligned to. The chassis is pushed flush against
// one of them; it is never centred, because the two edges are not alike — one
// is a painted line, the other may be the inner block.
const (
	AlignOuter = "outer"
	AlignInner = "inner"
)

// two is the divisor for midpoint arithmetic.
var two = decimal.NewFromInt(2)

// CenterCoord is the middle of the track on both axes.
func (t Track) CenterCoord() decimal.Decimal {
	return t.MinCoord.Add(t.MaxCoord).Div(two)
}

// CornerSize is the side length of one corner region.
func (t Track) CornerSize() decimal.Decimal {
	return t.CornerMax.Sub(t.CornerMin)
}

// BandWidths converts the division lines into the widths of the bands they
// delimit, measured out from the outer wall across a full-width corridor.
// Lines at 0.40 and 0.60 in a 1.0 m corridor give 0.40 / 0.20 / 0.40.
func (c Corridor) BandWidths() []decimal.Decimal {
	bands := make([]decimal.Decimal, 0, len(c.DivisionLines)+1)
	prev := decimal.Zero
	for _, line := range c.DivisionLines {
		bands = append(bands, line.Sub(prev))
		prev = line
	}
	return append(bands, c.Wide.Sub(prev))
}

// CellCentersAlong returns the along-corridor midpoints of the two cells in
// each band: the starting square occupies the middle metre of a side, so the
// cells sit half a cell either side of the track centre.
func (c Config) CellCentersAlong() (left, right decimal.Decimal) {
	half := c.StartingZone.DefaultLength.Div(two)
	center := c.Track.CenterCoord()
	return center.Sub(half), center.Add(half)
}

// SpawnOffsets derives where the robot is placed inside each band, measured
// out from the outer wall, by pushing the chassis flush against the band edge
// named in spawn_alignment.
//
// Deriving rather than declaring is what keeps the placement correct across a
// re-measurement of the chassis: the middle band is only a few millimetres
// wider than the robot, so a hardcoded offset silently stops meaning "flush"
// the moment the width changes.
func (c Config) SpawnOffsets(robotWidth decimal.Decimal) ([]decimal.Decimal, error) {
	bands := c.Corridor.BandWidths()
	if len(c.StartingZone.SpawnAlignment) != len(bands) {
		return nil, fmt.Errorf(
			"starting_zone.spawn_alignment has %d entries, want %d (one per band)",
			len(c.StartingZone.SpawnAlignment), len(bands),
		)
	}

	half := robotWidth.Div(two)
	offsets := make([]decimal.Decimal, 0, len(bands))
	edge := decimal.Zero
	for i, band := range bands {
		lo, hi := edge, edge.Add(band)
		if band.LessThan(robotWidth) {
			return nil, fmt.Errorf(
				"band %d is %s m wide, narrower than the %s m chassis, so no start fits in it",
				i, band, robotWidth,
			)
		}
		switch c.StartingZone.SpawnAlignment[i] {
		case AlignOuter:
			offsets = append(offsets, lo.Add(half))
		case AlignInner:
			offsets = append(offsets, hi.Sub(half))
		default:
			return nil, fmt.Errorf(
				"starting_zone.spawn_alignment[%d] = %q, want %q or %q",
				i, c.StartingZone.SpawnAlignment[i], AlignOuter, AlignInner,
			)
		}
		edge = hi
	}
	return offsets, nil
}

// Validate rejects a config whose derived geometry cannot hold, so a typo in
// the TOML fails at generation time rather than as a silently misplaced robot.
//
// The comparisons are exact. Working in decimals means the fit checks need no
// tolerance: a chassis that exactly fills its band is accepted because it
// genuinely fits, not because it fell inside an epsilon.
func (c Config) Validate(robotWidth decimal.Decimal) error {
	if len(c.Corridor.DivisionLines) == 0 {
		return fmt.Errorf("corridor.division_lines is empty")
	}
	prev := decimal.Zero
	for i, line := range c.Corridor.DivisionLines {
		if line.LessThanOrEqual(prev) {
			return fmt.Errorf("corridor.division_lines[%d] = %s does not increase past %s", i, line, prev)
		}
		prev = line
	}
	if prev.GreaterThanOrEqual(c.Corridor.Wide) {
		return fmt.Errorf(
			"corridor.division_lines end at %s, outside the %s m wide corridor",
			prev, c.Corridor.Wide,
		)
	}

	// Deriving the offsets is itself the check: it fails if a band cannot hold
	// the chassis at all, or if an alignment is not a band edge. A spawn can no
	// longer straddle a boundary the way a declared offset could, because it is
	// constructed flush against one.
	if _, err := c.SpawnOffsets(robotWidth); err != nil {
		return err
	}
	return nil
}

// Load reads, parses and validates the track.toml config at path.
func Load(path string, robotWidth decimal.Decimal) (*Config, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("read %s: %w", path, err)
	}
	var cfg Config
	if err := toml.Unmarshal(data, &cfg); err != nil {
		return nil, fmt.Errorf("parse %s: %w", path, err)
	}
	if err := cfg.Validate(robotWidth); err != nil {
		return nil, fmt.Errorf("validate %s: %w", path, err)
	}
	return &cfg, nil
}
