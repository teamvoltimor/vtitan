package lidar

import (
	"sort"
	"time"

	"google.golang.org/protobuf/types/known/timestamppb"

	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/lidar"
)

// FrameID is this sensor's TF frame, matching
// shared.config.constants.identifiers.TfFrames.LIDAR_LINK.
const FrameID = "lidar_link"

// MessageFor builds the Scan message to publish for one assembled
// 360-degree Scan.
//
// The classic SCAN protocol streams samples in acquisition order at
// whatever angular spacing the motor's rotation happened to produce --
// unlike EXPRESS_SCAN/ULTRA modes (not implemented, see the lidar package's
// doc.go), it makes no fixed-grid guarantee. sensor_msgs/LaserScan (and
// this repo's scan.proto, which mirrors it) models a regularly-spaced
// angle_min..angle_max sweep with one angle_increment step, so this sorts
// samples by angle and reports their *average* spacing as angle_increment
// rather than inventing a resampling/binning step -- there is no in-repo or
// vendor reference for how such a step should behave (see lidar/doc.go),
// so approximating the real, unevenly-spaced data as-is is preferred over
// guessing at one.
func MessageFor(scan lidar.Scan, sinceLastScan time.Duration) *sensorv1.Scan {
	points := append(lidar.Scan(nil), scan...)
	sort.Slice(points, func(i, j int) bool { return points[i].AngleRad < points[j].AngleRad })

	ranges := make([]float32, len(points))
	intensities := make([]float32, len(points))
	for i, pt := range points {
		ranges[i] = float32(pt.RangeM)
		intensities[i] = float32(pt.Quality)
	}

	var angleMin, angleMax, angleIncrement, timeIncrement float32
	if len(points) > 0 {
		angleMin = float32(points[0].AngleRad)
		angleMax = float32(points[len(points)-1].AngleRad)
	}
	if len(points) > 1 {
		angleIncrement = (angleMax - angleMin) / float32(len(points)-1)
		timeIncrement = float32(sinceLastScan.Seconds()) / float32(len(points)-1)
	}

	return &sensorv1.Scan{
		Stamp:   timestamppb.Now(),
		FrameId: FrameID,

		AngleMin:       angleMin,
		AngleMax:       angleMax,
		AngleIncrement: angleIncrement,
		TimeIncrement:  timeIncrement,
		ScanTime:       float32(sinceLastScan.Seconds()),
		RangeMin:       lidar.MinRangeM,
		RangeMax:       lidar.MaxRangeM,

		Ranges:      ranges,
		Intensities: intensities,
	}
}
