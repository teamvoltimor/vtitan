package trackmodel_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

const tolerance = 1e-9

func TestCorridorGeometryFromWidths(t *testing.T) {
	t.Parallel()

	const maxCoord = 3.0
	widths := map[trackmodel.Section]float64{
		trackmodel.North: 0.6,
		trackmodel.South: 0.6,
		trackmodel.East:  1.0,
		trackmodel.West:  1.0,
	}

	got := trackmodel.CorridorGeometryFromWidths(widths, maxCoord)

	if got.NorthWidthM != 0.6 || got.SouthWidthM != 0.6 || got.EastWidthM != 1.0 ||
		got.WestWidthM != 1.0 {
		t.Errorf("widths = %+v, want N=0.6 S=0.6 E=1.0 W=1.0", got)
	}

	want := trackmodel.InnerBlock{XMin: 1.0, YMin: 0.6, XMax: 2.0, YMax: 2.4}
	if got.InnerBlock != want {
		t.Errorf("InnerBlock = %+v, want %+v", got.InnerBlock, want)
	}
}

func TestCorridorGeometry_MinMeanWidth(t *testing.T) {
	t.Parallel()

	g := trackmodel.CorridorGeometry{
		NorthWidthM: 0.6,
		SouthWidthM: 1.0,
		EastWidthM:  1.0,
		WestWidthM:  1.0,
	}

	if got := g.MinWidthM(); got != 0.6 {
		t.Errorf("MinWidthM() = %v, want 0.6", got)
	}
	wantMean := (0.6 + 1.0 + 1.0 + 1.0) / 4.0
	if got := g.MeanWidthM(); got != wantMean {
		t.Errorf("MeanWidthM() = %v, want %v", got, wantMean)
	}
}
