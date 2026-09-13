package simconfig

import (
	"errors"
	"fmt"
	"path/filepath"
	"strconv"

	"github.com/shopspring/decimal"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// Track is the mat geometry the sim pipeline works from, loaded once from
// track.toml at runtime.
//
// It replaces the former `track_constants.gen.go`: instead of a second,
// bespoke generated copy of the same numbers, the values come from the same
// track.toml (through its generated DTO) that Python and the rest of Go read.
// Derived values (band widths, spawn offsets, cell centers) are computed here
// in exact decimal, matching the generator's old arithmetic.
type Track struct {
	TrackMatSize     float64
	TrackSize        float64
	TrackMinCoord    float64
	TrackMaxCoord    float64
	TrackCenterCoord float64
	TrackCornerMin   float64
	TrackCornerMax   float64
	TrackCornerSize  float64

	WallHeight             float64
	WallThickness          float64
	WallCollisionThickness float64
	WallExteriorOffset     float64
	WallInteriorOffset     float64

	CorridorNarrow    float64
	CorridorWide      float64
	CorridorObstacles float64
	CorridorMinWidth  float64
	CorridorMaxWidth  float64
	CorridorDivOuter  float64
	CorridorDivInner  float64
	CorridorDivWidth  float64

	SignWidth                   float64
	SignDepth                   float64
	SignHeight                  float64
	SignZPosition               float64
	SignGridDepthNear           float64
	SignGridDepthMiddle         float64
	SignGridDepthFar            float64
	SignGridWidthOuter          float64
	SignGridWidthInner          float64
	SignPlacementCircleDiameter float64
	SignMinCount                int
	SignMaxCount                int

	ParkingLength        float64
	ParkingWidth         float64
	ParkingHeight        float64
	ParkingZPosition     float64
	ParkingWallOffset    float64
	ParkingSpacingFactor float64

	StartingZoneDefaultLength   float64
	StartingZoneWidth           float64
	StartingZoneThickness       float64
	StartingZoneObstaclesFactor float64
	StartingZoneIndicatorRadius float64

	GridLengthSectionLeft  float64
	GridLengthSectionRight float64

	WallColor                         RGB
	SignColorRed                      RGB
	SignColorGreen                    RGB
	SignColorRedStd                   RGB
	SignColorGreenStd                 RGB
	ParkingColor                      RGB
	StartingZoneColor                 RGB
	StartingZoneClockwiseColor        RGB
	StartingZoneCounterClockwiseColor RGB

	StartingZoneBandWidths   []float64
	StartingZoneSpawnOffsets []float64
}

// dec converts a float to an exact decimal via its shortest round-trip string.
// The TOML holds exact decimals, so this recovers the value as written (0.6,
// not 0.59999999999999998) before the derived arithmetic.
func dec(f float64) decimal.Decimal {
	d, err := decimal.NewFromString(strconv.FormatFloat(f, 'g', -1, 64))
	if err != nil {
		return decimal.NewFromFloat(f)
	}
	return d
}

func rgb(c generated.RGB) RGB {
	out := RGB{}
	for i := 0; i < 3 && i < len(c); i++ {
		out[i] = c[i]
	}
	return out
}

func bandWidths(cfg *generated.TrackConfig) []float64 {
	bands := make([]float64, 0, len(cfg.Corridor.DivisionLines)+1)
	prev := decimal.Zero
	for _, line := range cfg.Corridor.DivisionLines {
		d := dec(line)
		bands = append(bands, d.Sub(prev).InexactFloat64())
		prev = d
	}
	return append(bands, dec(cfg.Corridor.Wide).Sub(prev).InexactFloat64())
}

func spawnOffsets(cfg *generated.TrackConfig, chassisWidth float64) ([]float64, error) {
	bands := bandWidths(cfg)
	alignment := cfg.StartingZone.SpawnAlignment
	if len(alignment) != len(bands) {
		return nil, fmt.Errorf(
			"starting_zone.spawn_alignment has %d entries, want %d (one per band)",
			len(alignment), len(bands),
		)
	}

	half := dec(chassisWidth).Div(decimal.NewFromInt(2))
	offsets := make([]float64, 0, len(bands))
	edge := decimal.Zero
	for i, band := range bands {
		lo, hi := edge, edge.Add(dec(band))
		switch alignment[i] {
		case generated.SpawnAlignmentOuter:
			offsets = append(offsets, lo.Add(half).InexactFloat64())
		case generated.SpawnAlignmentInner:
			offsets = append(offsets, hi.Sub(half).InexactFloat64())
		default:
			return nil, fmt.Errorf(
				"starting_zone.spawn_alignment[%d] = %q, want %q or %q",
				i, alignment[i], generated.SpawnAlignmentOuter, generated.SpawnAlignmentInner,
			)
		}
		edge = hi
	}
	return offsets, nil
}

// LoadTrack loads track.toml under configRoot and derives the values the sim
// needs.
//
// Args:
//
//	configRoot: repository config root (the directory holding track.toml).
//	chassisWidth: robot chassis width (m), used for the spawn offsets.
//
// Returns:
//
//	The loaded Track, or an error if track.toml cannot be read or is invalid.
func LoadTrack(configRoot string, chassisWidth float64) (*Track, error) {
	cfg, err := profile.Load[generated.TrackConfig](filepath.Join(configRoot, "track.toml"), nil)
	if err != nil {
		return nil, fmt.Errorf("simconfig: load track.toml: %w", err)
	}
	if len(cfg.Corridor.DivisionLines) < 2 {
		return nil, errors.New("simconfig: corridor.division_lines needs at least two entries")
	}

	offsets, err := spawnOffsets(cfg, chassisWidth)
	if err != nil {
		return nil, fmt.Errorf("simconfig: %w", err)
	}
	bands := bandWidths(cfg)
	center := dec(cfg.Track.MinCoord).Add(dec(cfg.Track.MaxCoord)).Div(decimal.NewFromInt(2))
	halfCell := dec(cfg.StartingZone.DefaultLength).Div(decimal.NewFromInt(2))
	lines := cfg.Corridor.DivisionLines

	return &Track{
		TrackMatSize:     cfg.Track.MatSize,
		TrackSize:        cfg.Track.Size,
		TrackMinCoord:    cfg.Track.MinCoord,
		TrackMaxCoord:    cfg.Track.MaxCoord,
		TrackCenterCoord: center.InexactFloat64(),
		TrackCornerMin:   cfg.Track.CornerMin,
		TrackCornerMax:   cfg.Track.CornerMax,
		TrackCornerSize:  dec(cfg.Track.CornerMax).Sub(dec(cfg.Track.CornerMin)).InexactFloat64(),

		WallHeight:             cfg.Wall.Height,
		WallThickness:          cfg.Wall.Thickness,
		WallCollisionThickness: cfg.Wall.CollisionThickness,
		WallExteriorOffset:     cfg.Wall.ExteriorOffset,
		WallInteriorOffset:     cfg.Wall.InteriorOffset,

		CorridorNarrow:    cfg.Corridor.Narrow,
		CorridorWide:      cfg.Corridor.Wide,
		CorridorObstacles: cfg.Corridor.Obstacles,
		CorridorMinWidth:  cfg.Corridor.MinWidth,
		CorridorMaxWidth:  cfg.Corridor.MaxWidth,
		CorridorDivOuter:  lines[0],
		CorridorDivInner:  lines[len(lines)-1],
		CorridorDivWidth:  bands[1],

		SignWidth:                   cfg.Sign.Width,
		SignDepth:                   cfg.Sign.Depth,
		SignHeight:                  cfg.Sign.Height,
		SignZPosition:               cfg.Sign.ZPosition,
		SignGridDepthNear:           cfg.Sign.GridDepthNear,
		SignGridDepthMiddle:         cfg.Sign.GridDepthMiddle,
		SignGridDepthFar:            cfg.Sign.GridDepthFar,
		SignGridWidthOuter:          lines[0],
		SignGridWidthInner:          lines[len(lines)-1],
		SignPlacementCircleDiameter: cfg.Sign.PlacementCircleDiameter,
		SignMinCount:                cfg.Sign.MinCount,
		SignMaxCount:                cfg.Sign.MaxCount,

		ParkingLength:        cfg.Parking.Length,
		ParkingWidth:         cfg.Parking.Width,
		ParkingHeight:        cfg.Parking.Height,
		ParkingZPosition:     cfg.Parking.ZPosition,
		ParkingWallOffset:    cfg.Parking.WallOffset,
		ParkingSpacingFactor: cfg.Parking.SpacingFactor,

		StartingZoneDefaultLength:   cfg.StartingZone.DefaultLength,
		StartingZoneWidth:           bands[1],
		StartingZoneThickness:       cfg.StartingZone.Thickness,
		StartingZoneObstaclesFactor: cfg.StartingZone.ObstaclesSizeFactor,
		StartingZoneIndicatorRadius: cfg.StartingZone.IndicatorRadius,

		GridLengthSectionLeft:  center.Sub(halfCell).InexactFloat64(),
		GridLengthSectionRight: center.Add(halfCell).InexactFloat64(),

		WallColor:                         rgb(cfg.Wall.Color),
		SignColorRed:                      rgb(cfg.Sign.RedColor),
		SignColorGreen:                    rgb(cfg.Sign.GreenColor),
		SignColorRedStd:                   rgb(cfg.Sign.RedStd),
		SignColorGreenStd:                 rgb(cfg.Sign.GreenStd),
		ParkingColor:                      rgb(cfg.Parking.Color),
		StartingZoneColor:                 rgb(cfg.StartingZone.Color),
		StartingZoneClockwiseColor:        rgb(cfg.StartingZone.ClockwiseColor),
		StartingZoneCounterClockwiseColor: rgb(cfg.StartingZone.CounterclockwiseColor),

		StartingZoneBandWidths:   bands,
		StartingZoneSpawnOffsets: offsets,
	}, nil
}
