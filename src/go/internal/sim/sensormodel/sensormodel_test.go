package sensormodel_test

import (
	"bytes"
	"log/slog"
	"math"
	"math/rand/v2"
	"path/filepath"
	"reflect"
	"runtime"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/sensormodel"
	"github.com/teamvoltimor/vtitan/src/go/pkg/geom"
)

// filterFloorM is lidar_sectors.toml's min_valid_range_m: the self-returns
// must land below it, as 99.7% of the measured ones do.
const filterFloorM = 0.044

func TestParseSet(t *testing.T) {
	t.Parallel()

	cases := []struct {
		raw  string
		want sensormodel.Set
	}{
		{"", 0},
		{"none", 0},
		{"all", sensormodel.All},
		{"lidar", sensormodel.Lidar},
		{"vision, IMU", sensormodel.Vision | sensormodel.IMU},
		{"tick-jitter,reverse-run", sensormodel.TickJitter | sensormodel.ReverseRun},
	}
	for _, c := range cases {
		got, err := sensormodel.ParseSet(c.raw)
		if err != nil {
			t.Fatalf("ParseSet(%q): %v", c.raw, err)
		}
		if got != c.want {
			t.Errorf("ParseSet(%q) = %v, want %v", c.raw, got, c.want)
		}
		back, err := sensormodel.ParseSet(got.String())
		if err != nil || back != got {
			t.Errorf("ParseSet(%q.String()) = %v, %v; want the set back", got, back, err)
		}
	}
	if _, err := sensormodel.ParseSet("lidar,sonar"); err == nil {
		t.Error("ParseSet accepted an unknown model")
	}
}

// Shipped distributions have analytic means the port must reproduce: the
// confidence mean is 0.7325 and the tick-period mean 0.05338 s, both
// computed from the piecewise-linear inverse CDF.
func TestSampleQuantile_MatchesTheAnalyticMeans(t *testing.T) {
	t.Parallel()

	p := sensormodel.DefaultParams()
	cases := []struct {
		name      string
		q, levels []float64
		want      float64
	}{
		{"confidence", p.Vision.ConfidenceQuantiles, p.Vision.ConfidenceLevels, 0.7325},
		{"tick period", p.TickPeriodQuantiles, p.TickPeriodLevels, 0.05338},
	}
	for _, c := range cases {
		const n = 200_000
		sum := 0.0
		for i := range n {
			sum += sensormodel.SampleQuantile((float64(i)+0.5)/n, c.q, c.levels, -1)
		}
		if got := sum / n; math.Abs(got-c.want)/c.want > 2e-3 {
			t.Errorf("%s mean = %.5f, want %.5f", c.name, got, c.want)
		}
	}
}

func TestSampleQuantile_FallsBackOnAMismatchedPair(t *testing.T) {
	t.Parallel()

	if got := sensormodel.SampleQuantile(0.3, []float64{1, 2}, []float64{0, 0.5, 1}, 0.9); got != 0.9 {
		t.Errorf("mismatched lengths = %v, want the fallback 0.9", got)
	}
	if got := sensormodel.SampleQuantile(0.3, nil, nil, 0.05); got != 0.05 {
		t.Errorf("empty = %v, want the fallback 0.05", got)
	}
}

// Over many scans the band rays drop at the measured 68.9%, every return
// left in the band sits below the filter floor and at or above zero, and
// no ray outside the band is touched.
func TestBands_MatchTheMeasuredC1(t *testing.T) {
	t.Parallel()

	p := sensormodel.DefaultParams().Lidar
	angles := geom.AngleFanClosed(500)
	bands := sensormodel.NewBands(p, angles, 7)

	const scans = 300
	var bandRays, drops, returns, subFloor, outsideTouched int
	for range scans {
		ranges := make([]float64, len(angles))
		for i := range ranges {
			ranges[i] = 1.0
		}
		bands.Apply(ranges)
		for i, r := range ranges {
			if !bands.Occluded(i) {
				if r != 1.0 {
					outsideTouched++
				}
				continue
			}
			bandRays++
			if math.IsInf(r, 1) {
				drops++
				continue
			}
			returns++
			if r >= 0 && r < filterFloorM {
				subFloor++
			}
		}
	}
	if bandRays == 0 {
		t.Fatal("no ray fell in the bands")
	}
	if got := float64(drops) / float64(bandRays); math.Abs(got-p.BandDropoutRate) > 0.02 {
		t.Errorf("band dropout = %.3f, want %.3f", got, p.BandDropoutRate)
	}
	if subFloor != returns {
		t.Errorf(
			"%d of %d band returns at or above the %.3f m floor, want none",
			returns-subFloor,
			returns,
			filterFloorM,
		)
	}
	if outsideTouched != 0 {
		t.Errorf("%d rays outside the band changed, want none", outsideTouched)
	}
	// 40 of 180 degrees per side: the bands are 2/9 of the sweep.
	if got := float64(bandRays) / float64(scans*len(angles)); math.Abs(got-2.0/9) > 0.01 {
		t.Errorf("band share of the sweep = %.3f, want %.3f", got, 2.0/9)
	}
}

// The range model is the logistic Python ships: p(0.7 m) = 0.935 and
// p(1.5 m) = 0.065 with no frame miss.
func TestCamera_RangeModelIsTheShippedLogistic(t *testing.T) {
	t.Parallel()

	p := sensormodel.DefaultParams().Vision
	p.FrameMissRate = 0
	cam := sensormodel.NewCamera(p, 3)

	for _, c := range []struct{ d, want float64 }{{0.7, 0.935}, {1.5, 0.065}} {
		const n = 40_000
		kept := 0
		sign := []signrouter.SignSpec{{X: c.d, Y: 0, Color: signrouter.SignColorRed}}
		for range n {
			kept += len(cam.Visible(sign, 0, 0))
		}
		if got := float64(kept) / n; math.Abs(got-c.want) > 0.01 {
			t.Errorf("p(detect at %.1f m) = %.3f, want %.3f", c.d, got, c.want)
		}
	}
}

// Scatter keeps each detection's range about the reporting pose exactly,
// and flips turn red into green at the configured rate.
func TestCamera_CorruptKeepsRangeAndFlipsAtTheRate(t *testing.T) {
	t.Parallel()

	p := sensormodel.DefaultParams().Vision
	cam := sensormodel.NewCamera(p, 11)
	rng := rand.New(rand.NewPCG(1, 2))

	const n = 40_000
	flips := 0
	for range n {
		x, y := rng.Float64()*2, rng.Float64()*2
		obs := []signrouter.TrafficSignObservation{{WorldXM: x, WorldYM: y, Color: signrouter.SignColorRed}}
		cam.Corrupt(obs, 0.5, 0.5)
		if got, want := math.Hypot(
			obs[0].WorldXM-0.5,
			obs[0].WorldYM-0.5,
		), math.Hypot(
			x-0.5,
			y-0.5,
		); math.Abs(
			got-want,
		) > 1e-9 {
			t.Fatalf("range after scatter = %v, want %v", got, want)
		}
		if obs[0].Color == signrouter.SignColorGreen {
			flips++
		}
		if obs[0].Confidence < p.ConfidenceQuantiles[0] ||
			obs[0].Confidence > p.ConfidenceQuantiles[len(p.ConfidenceQuantiles)-1] {
			t.Fatalf("confidence %v outside the measured quantiles", obs[0].Confidence)
		}
	}
	if got := float64(flips) / n; math.Abs(got-p.ColorFlipRate) > 0.005 {
		t.Errorf("flip rate = %.4f, want %.4f", got, p.ColorFlipRate)
	}
}

func TestTickPeriod_StaysInsideTheMeasuredRange(t *testing.T) {
	t.Parallel()

	p := sensormodel.DefaultParams()
	tick := sensormodel.NewTickPeriod(p, 0.05, 5)
	for range 10_000 {
		if got := tick.Next(); got < p.TickPeriodQuantiles[0] ||
			got > p.TickPeriodQuantiles[len(p.TickPeriodQuantiles)-1] {
			t.Fatalf("period %v outside [%v, %v]", got, p.TickPeriodQuantiles[0],
				p.TickPeriodQuantiles[len(p.TickPeriodQuantiles)-1])
		}
	}
	p.TickPeriodQuantiles = nil
	if got := sensormodel.NewTickPeriod(p, 0.05, 5).Next(); got != 0.05 {
		t.Errorf("empty distribution = %v, want the nominal 0.05", got)
	}
}

// The Go fallbacks must be the shipped TOML, or a run without a config
// root measures a different sensor than one with it.
func TestParamsFor_ShippedTOMLMatchesTheDefaults(t *testing.T) {
	t.Parallel()

	_, file, _, ok := runtime.Caller(0)
	if !ok {
		t.Fatal("runtime.Caller failed")
	}
	root := filepath.Join(filepath.Dir(file), "..", "..", "..", "..", "..")
	var logs bytes.Buffer
	got := sensormodel.ParamsFor(slog.New(slog.NewTextHandler(&logs, nil)), root)
	if logs.Len() > 0 {
		t.Fatalf("loading the shipped TOML logged %q, want a clean load", logs.String())
	}
	want := sensormodel.DefaultParams()
	if !reflect.DeepEqual(got.Vision, want.Vision) || !reflect.DeepEqual(got.IMU, want.IMU) ||
		!reflect.DeepEqual(got.TickPeriodQuantiles, want.TickPeriodQuantiles) ||
		!reflect.DeepEqual(got.TickPeriodLevels, want.TickPeriodLevels) {
		t.Errorf("shipped TOML differs from DefaultParams:\n got %+v\nwant %+v", got, want)
	}
	const tol = 1e-12
	gl, wl := got.Lidar, want.Lidar
	for _, pair := range [][2]float64{
		{gl.InvalidRayRate, wl.InvalidRayRate}, {gl.BandMinRad, wl.BandMinRad}, {gl.BandMaxRad, wl.BandMaxRad},
		{gl.BandDropoutRate, wl.BandDropoutRate}, {gl.SelfReturnM, wl.SelfReturnM},
		{gl.SelfReturnStdM, wl.SelfReturnStdM},
	} {
		if math.Abs(pair[0]-pair[1]) > tol {
			t.Errorf("shipped LIDAR params %+v differ from DefaultParams %+v", gl, wl)
			break
		}
	}
}
