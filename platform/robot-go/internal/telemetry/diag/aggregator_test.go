package diag_test

import (
	"context"
	"math"
	"testing"

	sensorv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/sensor/v1"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/telemetry/diag"
)

// fakeSource is a hand-written Source (see source.go) for testing Aggregator
// without any real transport — the same pattern
// internal/sim/scenario.Orchestrator's tests use against a fake Runner.
type fakeSource struct {
	scan         *sensorv1.Scan
	scanOK       bool
	imu          *sensorv1.Imu
	imuOK        bool
	detections   []diag.Detection
	detectionsOK bool
}

// summaryTolerance bounds the float comparisons below — the aggregation
// math is trig-based (angle wrapping, sin/cos), so exact equality isn't a
// meaningful assertion.
const summaryTolerance = 1e-6

// belowMinValidRangeM is a range reading below DefaultMinValidRangeM (and
// DefaultSelfDetectionThresholdM), used to fill scan rays that a test case
// wants excluded from every sector regardless of bearing.
const belowMinValidRangeM = 0.01

func (f fakeSource) LatestScan(context.Context) (*sensorv1.Scan, bool) {
	return f.scan, f.scanOK
}

func (f fakeSource) LatestIMU(context.Context) (*sensorv1.Imu, bool) {
	return f.imu, f.imuOK
}

func (f fakeSource) LatestDetections(context.Context) ([]diag.Detection, bool) {
	return f.detections, f.detectionsOK
}

// eightRaySweep builds an 8-ray scan (45 deg apart, angle(i) = -180 + i*45,
// matching sector.go's fullSweepRad synthesis with zero yaw offset) with
// every ray below the minimum valid range, so a test can set exactly the
// indices it cares about.
func eightRaySweep() []float32 {
	const rayCount = 8
	ranges := make([]float32, rayCount)
	for i := range ranges {
		ranges[i] = belowMinValidRangeM
	}
	return ranges
}

func TestAggregatorSummarizeNoInputYieldsZeroSummary(t *testing.T) {
	t.Parallel()

	aggregator := diag.NewAggregator(fakeSource{}, diag.DefaultConfig())
	summary := aggregator.Summarize(context.Background())

	if summary != (diag.TelemetrySummary{}) {
		t.Errorf("Summarize() with no input = %+v, want zero value", summary)
	}
}

func TestAggregatorSummarizeLidarSectors(t *testing.T) {
	t.Parallel()

	// eightRaySweep's index 4 sits at bearing 0 (front), index 6 at +90 deg
	// (left), index 2 at -90 deg (right) — see the derivation in
	// eightRaySweep's own angle formula. Each is set to a distinct valid
	// range so the three sectors are individually verifiable.
	const (
		frontIndex = 4
		leftIndex  = 6
		rightIndex = 2

		frontRangeM = 1.0
		leftRangeM  = 2.0
		rightRangeM = 3.0
	)
	ranges := eightRaySweep()
	ranges[frontIndex] = frontRangeM
	ranges[leftIndex] = leftRangeM
	ranges[rightIndex] = rightRangeM

	source := fakeSource{
		scan:   &sensorv1.Scan{Ranges: ranges},
		scanOK: true,
	}
	aggregator := diag.NewAggregator(source, diag.DefaultConfig())
	summary := aggregator.Summarize(context.Background())

	assertApprox(t, "LidarFrontCM", summary.LidarFrontCM, frontRangeM*100)
	assertApprox(t, "LidarLeftCM", summary.LidarLeftCM, leftRangeM*100)
	assertApprox(t, "LidarRightCM", summary.LidarRightCM, rightRangeM*100)
}

func TestAggregatorSummarizeLidarSectorMeansMultipleRays(t *testing.T) {
	t.Parallel()

	// At the default 45 deg ray spacing only one ray at a time falls inside
	// the default 30 deg front half-FOV (see TestAggregatorSummarizeLidarSectors),
	// so this test uses a finer 10-deg sweep to put two rays inside the same
	// sector and confirm the aggregator reports their mean, not either
	// individual value.
	const rayCount = 36 // 10 deg spacing
	ranges := make([]float32, rayCount)
	for i := range ranges {
		ranges[i] = belowMinValidRangeM
	}
	// angle(i) = -180 + i*10; i=18 -> 0 deg, i=19 -> 10 deg. Both within the
	// default 30 deg front half-FOV.
	const (
		firstFrontIndex  = 18
		secondFrontIndex = 19
		firstRangeM      = 1.0
		secondRangeM     = 3.0
	)
	ranges[firstFrontIndex] = firstRangeM
	ranges[secondFrontIndex] = secondRangeM

	source := fakeSource{
		scan:   &sensorv1.Scan{Ranges: ranges},
		scanOK: true,
	}
	aggregator := diag.NewAggregator(source, diag.DefaultConfig())
	summary := aggregator.Summarize(context.Background())

	wantMeanCM := (firstRangeM + secondRangeM) / 2 * 100
	assertApprox(t, "LidarFrontCM", summary.LidarFrontCM, wantMeanCM)
}

func TestAggregatorSummarizeGyroYaw(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name        string
		orientation *sensorv1.Quaternion
		wantYawDeg  float64
	}{
		{
			name:        "identity orientation is zero yaw",
			orientation: &sensorv1.Quaternion{W: 1},
			wantYawDeg:  0,
		},
		{
			name: "ninety degree yaw",
			// Pure yaw-about-Z quaternion for theta=90deg:
			// (x=0, y=0, z=sin(theta/2), w=cos(theta/2)).
			orientation: &sensorv1.Quaternion{Z: math.Sin(math.Pi / 4), W: math.Cos(math.Pi / 4)},
			wantYawDeg:  90,
		},
		{
			name: "negative ninety degree yaw",
			orientation: &sensorv1.Quaternion{
				Z: math.Sin(-math.Pi / 4),
				W: math.Cos(-math.Pi / 4),
			},
			wantYawDeg: -90,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			source := fakeSource{
				imu:   &sensorv1.Imu{Orientation: tt.orientation},
				imuOK: true,
			}
			aggregator := diag.NewAggregator(source, diag.DefaultConfig())
			summary := aggregator.Summarize(context.Background())

			assertApprox(t, "GyroYawDeg", summary.GyroYawDeg, tt.wantYawDeg)
		})
	}
}

func TestAggregatorSummarizeBestDetection(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name           string
		detections     []diag.Detection
		wantHasBest    bool
		wantClassID    string
		wantConfidence float64
	}{
		{
			name:        "no detections",
			detections:  nil,
			wantHasBest: false,
		},
		{
			name: "single detection wins by default",
			detections: []diag.Detection{
				{ClassName: "red", Confidence: 0.9, Width: 10, Height: 10},
			},
			wantHasBest:    true,
			wantClassID:    "red",
			wantConfidence: 0.9,
		},
		{
			name: "small high-confidence loses to large lower-confidence",
			detections: []diag.Detection{
				{ClassName: "small-hot", Confidence: 0.99, Width: 1, Height: 1},
				{ClassName: "large-warm", Confidence: 0.5, Width: 100, Height: 100},
			},
			wantHasBest:    true,
			wantClassID:    "large-warm",
			wantConfidence: 0.5,
		},
		{
			name: "ties keep the first candidate seen",
			detections: []diag.Detection{
				{ClassName: "first", Confidence: 0.5, Width: 10, Height: 10},
				{ClassName: "second", Confidence: 0.5, Width: 10, Height: 10},
			},
			wantHasBest:    true,
			wantClassID:    "first",
			wantConfidence: 0.5,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			source := fakeSource{
				detections:   tt.detections,
				detectionsOK: true,
			}
			aggregator := diag.NewAggregator(source, diag.DefaultConfig())
			summary := aggregator.Summarize(context.Background())

			if summary.HasBestDetection != tt.wantHasBest {
				t.Errorf("HasBestDetection = %v, want %v", summary.HasBestDetection, tt.wantHasBest)
			}
			if !tt.wantHasBest {
				return
			}
			if summary.BestDetectionClassID != tt.wantClassID {
				t.Errorf("BestDetectionClassID = %q, want %q", summary.BestDetectionClassID, tt.wantClassID)
			}
			assertApprox(t, "BestDetectionConfidence", summary.BestDetectionConfidence, tt.wantConfidence)
		})
	}
}

func TestConfigValidate(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name    string
		mutate  func(c diag.Config) diag.Config
		wantErr bool
	}{
		{
			name:    "default config is valid",
			mutate:  func(c diag.Config) diag.Config { return c },
			wantErr: false,
		},
		{
			name: "zero half-FOV is invalid",
			mutate: func(c diag.Config) diag.Config {
				c.FrontHalfFOVRad = 0
				return c
			},
			wantErr: true,
		},
		{
			name: "max not greater than min is invalid",
			mutate: func(c diag.Config) diag.Config {
				c.MaxValidRangeM = c.MinValidRangeM
				return c
			},
			wantErr: true,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			err := tt.mutate(diag.DefaultConfig()).Validate()
			if (err != nil) != tt.wantErr {
				t.Errorf("Validate() error = %v, wantErr %v", err, tt.wantErr)
			}
		})
	}
}

func assertApprox(t *testing.T, field string, got, want float64) {
	t.Helper()
	if math.Abs(got-want) > summaryTolerance {
		t.Errorf("%s = %v, want %v", field, got, want)
	}
}
