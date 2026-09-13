// Package preview renders a top-down SVG visualization of a generated WRO 2026 scenario.
// It reads a *_metadata.json file produced by the generate package and writes an SVG
// that shows the track layout, interior walls, sign placement grid, traffic signs,
// parking blocks, and robot spawn position — without requiring a running Gazebo instance.
package preview

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"strconv"
	"strings"

	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/generate"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
)

// SVG canvas geometry constants (pixels).
const (
	marginTop  = 30.0
	marginSide = 30.0
	trackPx    = 600.0 // 600 px for 3 m × 3 m track
	labelH     = 20.0  // label bar height below track
)

// SVG color palette — visual-only, not in Gazebo.
// Colors that correspond to WRO field elements are derived from simconfig RGB values.
const (
	colorCanvas     = "#d0d0d0"
	colorTrackFloor = "white" // WRO spec: white mat
	colorWall       = "#222"  // exterior and interior wall outline
	colorInnerZone  = "#ccc"  // forbidden inner area fill
	colorLabelBg    = "#222"
	colorLabelText  = "white"

	// WRO field element colors (must stay in sync with simconfig RGB vars).
	colorSignRed       = "#EE2737" // simconfig.SignColorRed  → RGB(238, 39, 55)
	colorSignGreen     = "#44D62C" // simconfig.SignColorGreen → RGB(68, 214, 44)
	colorParking       = "#FF00FF" // simconfig.ParkingColor  → RGB(255, 0, 255)
	colorParkingStroke = "#880088"
	colorCornerBlue    = "#0033FF" // simconfig.CornerMarkerBlueColor   → RGB(0, 51, 255)
	colorCornerOrange  = "#FF6600" // simconfig.CornerMarkerOrangeColor → RGB(255, 102, 0)
	colorRobot         = "#0044cc"
	colorRobotStroke   = "white"
	colorArrow         = colorCornerOrange // orange — visible on both white and blue
	colorSignStroke    = "#111"
	colorGridLine      = "#888"
	colorSubdivLine    = "#aaa"
)

// SVG stroke widths (pixels).
const (
	strokeTrack        = 2.0
	strokeInteriorWall = 2.5
	strokeGridLine     = 1.0
	strokeSubdivLine   = 1.0
	strokeArrow        = 3.0
	strokeParkingBlock = 1.5
	strokeSign         = 1.0
	strokeRobot        = 2.0

	minCornerMarkerPx = 3.0 // minimum visibility floor for thin markers at small scales
)

// Dash patterns for SVG stroke-dasharray.
const (
	dashGrid   = "6,4"
	dashSubdiv = "3,5"
)

// Arrow SVG marker geometry.
const (
	arrowMarkerSize = 8
	arrowMarkerRef  = 4
	arrowShapeD     = "M0,0 L8,4 L0,8 Z"
)

// signDisplayGrowthFactor makes the preview sign 20% larger than the actual
// sign for legibility.
const signDisplayGrowthFactor = 1.2

// Label bar layout constants (pixels).
const (
	labelPanelGap = 4.0
	labelPadX     = 6.0
	labelTextOffY = 13.0
	labelFontSize = 11
)

// SVG element format strings.
// Backtick raw strings are used so attribute values can contain double quotes
// without escaping, making the SVG structure directly readable in source.
const (
	xmlDecl = `<?xml version="1.0" encoding="utf-8"?>` + "\n"

	fmtSVGRoot = `<svg xmlns="http://www.w3.org/2000/svg"` +
		` width="%d" height="%d" viewBox="0 0 %d %d">` + "\n"

	fmtMarkerOpen = `  <defs>` + "\n" +
		`    <marker id="arrow" markerWidth="%d" markerHeight="%d"` +
		` refX="%d" refY="%d" orient="auto">`

	fmtMarkerClose = `<path d="%s" fill="%s"/></marker>` + "\n" + `  </defs>` + "\n"

	fmtCircle = `  <circle cx="%.1f" cy="%.1f" r="%.1f"` +
		` fill="%s" stroke="%s" stroke-width="%.1f"/>` + "\n"

	fmtArrowLine = `  <line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"` +
		` stroke="%s" stroke-width="%.1f" marker-end="url(#arrow)"/>` + "\n"

	fmtRect = `  <rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="%s"/>` + "\n"

	fmtRectStroked = `  <rect x="%.1f" y="%.1f" width="%.1f" height="%.1f"` +
		` fill="%s" stroke="%s" stroke-width="%.1f"/>` + "\n"

	fmtLine = `  <line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"` +
		` stroke="%s" stroke-width="%.1f"/>` + "\n"

	fmtLineDashed = `  <line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"` +
		` stroke="%s" stroke-width="%.1f" stroke-dasharray="%s"/>` + "\n"

	fmtText = `  <text x="%.1f" y="%.1f" font-family="monospace" font-size="%d" fill="%s">%s</text>` + "\n"
)

// signDisplayHalf is the half-size of the preview sign rectangle.
func signDisplayHalf(track *simconfig.Track) float64 {
	return track.SignWidth / 2 * signDisplayGrowthFactor // 30 mm (spec: 25 mm)
}

// pxPerMeter converts a world length in meters to pixels for one track.
func pxPerMeter(track *simconfig.Track) float64 {
	return trackPx / track.TrackMaxCoord
}

// GenerateSVG reads metadataPath, renders an SVG top-down preview, and writes it to outPath.
// If outPath is empty it is derived from metadataPath by replacing MetadataSuffix with
// PreviewSuffix (falling back to appending PreviewSuffix for other extensions).
// The resolved output path is returned alongside any error.
func GenerateSVG(track *simconfig.Track, robot *simconfig.Robot, metadataPath, outPath string) (string, error) {
	data, err := os.ReadFile(metadataPath)
	if err != nil {
		return "", fmt.Errorf("read metadata: %w", err)
	}
	var meta generate.Metadata
	if err = json.Unmarshal(data, &meta); err != nil {
		return "", fmt.Errorf("parse metadata: %w", err)
	}
	if outPath == "" {
		base := strings.TrimSuffix(metadataPath, simconfig.MetadataSuffix)
		if base == metadataPath {
			base = strings.TrimSuffix(metadataPath, ".json")
		}
		outPath = base + simconfig.PreviewSuffix
	}
	if err = os.WriteFile(outPath, []byte(renderSVG(track, robot, meta)), simconfig.FilePermissions); err != nil {
		return outPath, fmt.Errorf("write preview: %w", err)
	}
	return outPath, nil
}

// wx converts a world X coordinate (meters) to SVG X (pixels).
func wx(track *simconfig.Track, x float64) float64 {
	return marginSide + x*pxPerMeter(track)
}

// wy converts a world Y coordinate (meters) to SVG Y (pixels).
// World Y=0 is at the bottom; SVG Y=0 is at the top, so we flip.
func wy(track *simconfig.Track, y float64) float64 {
	return marginTop + (track.TrackMaxCoord-y)*pxPerMeter(track)
}

// wp converts a world length (meters) to pixels.
func wp(track *simconfig.Track, v float64) float64 {
	return v * pxPerMeter(track)
}

// renderSVG generates the SVG content for a scenario metadata.
func renderSVG(track *simconfig.Track, robot *simconfig.Robot, meta generate.Metadata) string {
	var b strings.Builder

	svgH := int(marginTop + trackPx + labelH + marginTop)
	svgW := int(marginSide + trackPx + marginSide)
	b.WriteString(xmlDecl)
	fmt.Fprintf(&b, fmtSVGRoot, svgW, svgH, svgW, svgH)

	// Arrow marker definition
	fmt.Fprintf(&b, fmtMarkerOpen, arrowMarkerSize, arrowMarkerSize, arrowMarkerRef, arrowMarkerRef)
	fmt.Fprintf(&b, fmtMarkerClose, arrowShapeD, colorArrow)

	// Canvas background
	writeRect(&b, 0, 0, float64(svgW), float64(svgH), colorCanvas, "none", 0)

	// Track floor (WRO spec: white)
	writeRect(
		&b,
		wx(track, 0),
		wy(track, track.TrackMaxCoord),
		trackPx,
		trackPx,
		colorTrackFloor,
		colorWall,
		strokeTrack,
	)

	// Inner forbidden zone derived from corridor widths
	drawInnerZone(&b, track, meta.CorridorWidths)

	// Corner diagonal markers — two per corner (blue + orange), matching addCornerMarkers in sdf/world.go.
	drawCornerMarkers(&b, track)

	// Corner boundary grid lines: vertical at x=1.0/2.0, horizontal at y=1.0/2.0.
	// These are the lines at the 1m×1m corner-square edges (matches addGridLines in sdf/world.go).
	drawGridLines(&b, track)

	// Corridor subdivision guides: centerlines and sign-placement width markers
	// (matches addCorridorSubdivisions in sdf/world.go).
	drawSubdivisionLines(&b, track)

	// Interior wall boundary lines (thick — where navigable corridor ends)
	drawInteriorWalls(&b, track, meta.CorridorWidths)

	// Traffic signs (50×50 mm pillars, top-down view)
	for _, sign := range meta.SignPositions {
		color := colorSignRed
		if sign.Color == simconfig.ColorNameGreen {
			color = colorSignGreen
		}
		half := signDisplayHalf(track)
		writeRect(&b, wx(track, sign.X)-half, wy(track, sign.Y)-half,
			half*2, half*2, color, colorSignStroke, strokeSign)
	}

	// Parking blocks (200×20 mm, magenta). Long axis is perpendicular to the outer wall.
	if meta.ParkingLot != nil {
		section := strings.ToLower(meta.StartingConditions.Section)
		drawParkingBlock(&b, track, meta.ParkingLot.Block1Position.X, meta.ParkingLot.Block1Position.Y, section)
		drawParkingBlock(&b, track, meta.ParkingLot.Block2Position.X, meta.ParkingLot.Block2Position.Y, section)
	}

	// Robot spawn: filled circle
	sc := meta.StartingConditions
	cx, cy := wx(track, sc.Position.X), wy(track, sc.Position.Y)
	robotR := wp(track, robot.RobotWidth/2)
	fmt.Fprintf(&b, fmtCircle, cx, cy, robotR, colorRobot, colorRobotStroke, strokeRobot)

	// Direction arrow: line from center along yaw.
	// World yaw is CCW from +X; SVG Y is flipped so the Y component is negated.
	arrowLen := wp(track, robot.RobotLength)
	endX := cx + arrowLen*math.Cos(sc.Yaw)
	endY := cy - arrowLen*math.Sin(sc.Yaw)
	fmt.Fprintf(&b, fmtArrowLine, cx, cy, endX, endY, colorArrow, strokeArrow)

	// Label bar
	drawLabelBar(&b, meta)

	b.WriteString("</svg>\n")
	return b.String()
}

// drawGridLines draws the four full-track corner-boundary lines that define the
// 1m×1m corner squares (x=1.0, x=2.0 vertical; y=1.0, y=2.0 horizontal).
// These match the grid_line_* models generated by addGridLines in sdf/world.go.
func drawGridLines(b *strings.Builder, track *simconfig.Track) {
	tMax := track.TrackMaxCoord
	cMin := track.TrackCornerMin // 1.0 m
	cMax := track.TrackCornerMax // 2.0 m
	// Vertical corner boundary lines
	writeDashedLine(
		b,
		wx(track, cMin),
		wy(track, 0),
		wx(track, cMin),
		wy(track, tMax),
		colorGridLine,
		strokeGridLine,
		dashGrid,
	)
	writeDashedLine(
		b,
		wx(track, cMax),
		wy(track, 0),
		wx(track, cMax),
		wy(track, tMax),
		colorGridLine,
		strokeGridLine,
		dashGrid,
	)
	// Horizontal corner boundary lines
	writeDashedLine(
		b,
		wx(track, 0),
		wy(track, cMin),
		wx(track, tMax),
		wy(track, cMin),
		colorGridLine,
		strokeGridLine,
		dashGrid,
	)
	writeDashedLine(
		b,
		wx(track, 0),
		wy(track, cMax),
		wx(track, tMax),
		wy(track, cMax),
		colorGridLine,
		strokeGridLine,
		dashGrid,
	)
}

// drawSubdivisionLines draws the corridor centerlines and sign-placement width markers
// that appear inside each corridor section. These match the corridor_* models generated
// by addCorridorSubdivisions in sdf/world.go.
//
// For NS corridors (south/north): vertical centerline at x=1.5, horizontal width markers
// at the outer (0.4/2.6) and inner (0.6/2.4) positions spanning x=1.0→2.0.
// For EW corridors (west/east): horizontal centerline at y=1.5, vertical width markers
// at the outer and inner positions spanning y=1.0→2.0.
func drawSubdivisionLines(b *strings.Builder, track *simconfig.Track) {
	tMax := track.TrackMaxCoord
	cMin := track.TrackCornerMin     // 1.0
	cMax := track.TrackCornerMax     // 2.0
	center := track.TrackCenterCoord // 1.5
	dOuter := track.CorridorDivOuter // 0.4
	dInner := track.CorridorDivInner // 0.6
	farInner := tMax - dInner        // 2.4
	farOuter := tMax - dOuter        // 2.6

	// South corridor (y=0 → 1.0): centerline along x=1.5, width markers across corridor
	writeDashedLine(
		b,
		wx(track, center),
		wy(track, 0),
		wx(track, center),
		wy(track, cMin),
		colorSubdivLine,
		strokeSubdivLine,
		dashSubdiv,
	)
	writeDashedLine(
		b,
		wx(track, cMin),
		wy(track, dOuter),
		wx(track, cMax),
		wy(track, dOuter),
		colorSubdivLine,
		strokeSubdivLine,
		dashSubdiv,
	)
	writeDashedLine(
		b,
		wx(track, cMin),
		wy(track, dInner),
		wx(track, cMax),
		wy(track, dInner),
		colorSubdivLine,
		strokeSubdivLine,
		dashSubdiv,
	)

	// North corridor (y=2.0 → 3.0): centerline along x=1.5, width markers across corridor
	writeDashedLine(
		b,
		wx(track, center),
		wy(track, cMax),
		wx(track, center),
		wy(track, tMax),
		colorSubdivLine,
		strokeSubdivLine,
		dashSubdiv,
	)
	writeDashedLine(
		b,
		wx(track, cMin),
		wy(track, farInner),
		wx(track, cMax),
		wy(track, farInner),
		colorSubdivLine,
		strokeSubdivLine,
		dashSubdiv,
	)
	writeDashedLine(
		b,
		wx(track, cMin),
		wy(track, farOuter),
		wx(track, cMax),
		wy(track, farOuter),
		colorSubdivLine,
		strokeSubdivLine,
		dashSubdiv,
	)

	// West corridor (x=0 → 1.0): centerline along y=1.5, width markers across corridor
	writeDashedLine(
		b,
		wx(track, 0),
		wy(track, center),
		wx(track, cMin),
		wy(track, center),
		colorSubdivLine,
		strokeSubdivLine,
		dashSubdiv,
	)
	writeDashedLine(
		b,
		wx(track, dOuter),
		wy(track, cMin),
		wx(track, dOuter),
		wy(track, cMax),
		colorSubdivLine,
		strokeSubdivLine,
		dashSubdiv,
	)
	writeDashedLine(
		b,
		wx(track, dInner),
		wy(track, cMin),
		wx(track, dInner),
		wy(track, cMax),
		colorSubdivLine,
		strokeSubdivLine,
		dashSubdiv,
	)

	// East corridor (x=2.0 → 3.0): centerline along y=1.5, width markers across corridor
	writeDashedLine(
		b,
		wx(track, cMax),
		wy(track, center),
		wx(track, tMax),
		wy(track, center),
		colorSubdivLine,
		strokeSubdivLine,
		dashSubdiv,
	)
	writeDashedLine(
		b,
		wx(track, farInner),
		wy(track, cMin),
		wx(track, farInner),
		wy(track, cMax),
		colorSubdivLine,
		strokeSubdivLine,
		dashSubdiv,
	)
	writeDashedLine(
		b,
		wx(track, farOuter),
		wy(track, cMin),
		wx(track, farOuter),
		wy(track, cMax),
		colorSubdivLine,
		strokeSubdivLine,
		dashSubdiv,
	)
}

// drawCornerMarkers renders the eight diagonal decorative markers (two per corner:
// one blue, one orange) that appear on the WRO 2026 field.
// Geometry comes from the shared simconfig.CornerMarkers table; only the
// color representation is SVG-specific.
func drawCornerMarkers(b *strings.Builder, track *simconfig.Track) {
	halfLen := wp(track, simconfig.CornerMarkerLength/2)
	strokeW := math.Max(wp(track, simconfig.CornerMarkerWidth), minCornerMarkerPx)
	for _, m := range simconfig.CornerMarkers(track) {
		color := colorCornerOrange
		if m.Blue {
			color = colorCornerBlue
		}
		cx, cy := wx(track, m.CX), wy(track, m.CY)
		// World yaw θ: unit vector is (cos θ, sin θ); SVG Y is flipped so ΔyS = -sin θ.
		x1 := cx - halfLen*math.Cos(m.YawRad)
		y1 := cy + halfLen*math.Sin(m.YawRad)
		x2 := cx + halfLen*math.Cos(m.YawRad)
		y2 := cy - halfLen*math.Sin(m.YawRad)
		writeLine(b, x1, y1, x2, y2, color, strokeW)
	}
}

func drawInnerZone(b *strings.Builder, track *simconfig.Track, widths map[string]generate.WidthMeta) {
	sw := mmToM(widths[string(simconfig.SectionSouth)].WidthMM)
	nw := mmToM(widths[string(simconfig.SectionNorth)].WidthMM)
	ew := mmToM(widths[string(simconfig.SectionEast)].WidthMM)
	ww := mmToM(widths[string(simconfig.SectionWest)].WidthMM)
	if sw <= 0 || nw <= 0 || ew <= 0 || ww <= 0 {
		return
	}
	x1, y1 := ww, sw
	x2, y2 := track.TrackMaxCoord-ew, track.TrackMaxCoord-nw
	writeRect(b, wx(track, x1), wy(track, y2), wp(track, x2-x1), wp(track, y2-y1), colorInnerZone, "none", 0)
}

func drawInteriorWalls(b *strings.Builder, track *simconfig.Track, widths map[string]generate.WidthMeta) {
	sw := mmToM(widths[string(simconfig.SectionSouth)].WidthMM)
	nw := mmToM(widths[string(simconfig.SectionNorth)].WidthMM)
	ew := mmToM(widths[string(simconfig.SectionEast)].WidthMM)
	ww := mmToM(widths[string(simconfig.SectionWest)].WidthMM)
	if sw <= 0 || nw <= 0 || ew <= 0 || ww <= 0 {
		return
	}
	tMax := track.TrackMaxCoord
	// Each wall spans only between the two perpendicular interior boundaries,
	// matching the SDF wall dimensions (eastX-westX / northY-southY).
	writeLine(b, wx(track, ww), wy(track, sw), wx(track, tMax-ew), wy(track, sw), colorWall, strokeInteriorWall)
	writeLine(
		b,
		wx(track, ww),
		wy(track, tMax-nw),
		wx(track, tMax-ew),
		wy(track, tMax-nw),
		colorWall,
		strokeInteriorWall,
	)
	writeLine(b, wx(track, ww), wy(track, sw), wx(track, ww), wy(track, tMax-nw), colorWall, strokeInteriorWall)
	writeLine(
		b,
		wx(track, tMax-ew),
		wy(track, sw),
		wx(track, tMax-ew),
		wy(track, tMax-nw),
		colorWall,
		strokeInteriorWall,
	)
}

// drawParkingBlock renders a single parking block at the given world coordinates.
func drawParkingBlock(b *strings.Builder, track *simconfig.Track, x, y float64, section string) {
	// WRO spec: 200×20 mm. Long axis is perpendicular to the corridor's outer wall (pointing inward).
	// South/North outer walls run along X → long axis is Y (200 mm), narrow axis is X (20 mm).
	// East/West outer walls run along Y → long axis is X (200 mm), narrow axis is Y (20 mm).
	var bw, bh float64
	switch section {
	case string(simconfig.SectionNorth), string(simconfig.SectionSouth):
		bw, bh = wp(track, track.ParkingWidth), wp(track, track.ParkingLength)
	default: // east, west
		bw, bh = wp(track, track.ParkingLength), wp(track, track.ParkingWidth)
	}
	writeRect(b, wx(track, x)-bw/2, wy(track, y)-bh/2, bw, bh, colorParking, colorParkingStroke, strokeParkingBlock)
}

// drawLabelBar renders the scenario information label bar at the bottom of the SVG.
func drawLabelBar(b *strings.Builder, meta generate.Metadata) {
	panelY := marginTop + trackPx + labelPanelGap
	writeRect(b, marginSide, panelY, trackPx, labelH-labelPanelGap, colorLabelBg, "none", 0)

	seed := "random"
	if meta.Seed != nil {
		seed = strconv.FormatInt(*meta.Seed, 10)
	}
	label := fmt.Sprintf(
		"#%04d  %s  %d signs  parking=%v  seed=%s  dir=%s (%s)",
		meta.ScenarioID, meta.ChallengeType, meta.NumSigns, meta.HasParkingLot,
		seed, meta.StartingConditions.Direction, meta.StartingConditions.Section,
	)
	fmt.Fprintf(b, fmtText, marginSide+labelPadX, panelY+labelTextOffY, labelFontSize, colorLabelText, label)
}

// writeRect writes a rectangle SVG element with optional stroke.
func writeRect(b *strings.Builder, x, y, w, h float64, fill, stroke string, strokeW float64) {
	if stroke == "none" || strokeW == 0 {
		fmt.Fprintf(b, fmtRect, x, y, w, h, fill)
	} else {
		fmt.Fprintf(b, fmtRectStroked, x, y, w, h, fill, stroke, strokeW)
	}
}

// writeLine writes a solid line SVG element.
func writeLine(b *strings.Builder, x1, y1, x2, y2 float64, stroke string, strokeW float64) {
	fmt.Fprintf(b, fmtLine, x1, y1, x2, y2, stroke, strokeW)
}

// writeDashedLine writes a dashed line SVG element.
func writeDashedLine(
	b *strings.Builder, x1, y1, x2, y2 float64, stroke string, strokeW float64, dash string,
) {
	fmt.Fprintf(b, fmtLineDashed, x1, y1, x2, y2, stroke, strokeW, dash)
}

// mmToM converts millimeters to meters.
func mmToM(mm int) float64 {
	return float64(mm) / simconfig.MillimetersPerMeter
}
