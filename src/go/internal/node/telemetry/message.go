package telemetry

import (
	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/teamvoltimor/vtitan/src/go/internal/telemetry/diag"

	uiv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/ui/v1"
)

// MessageFor converts a diag.TelemetrySummary into the TelemetrySummary
// message to publish.
func MessageFor(summary diag.TelemetrySummary) *uiv1.TelemetrySummary {
	return &uiv1.TelemetrySummary{
		Stamp:                   timestamppb.Now(),
		BestDetectionClassId:    summary.BestDetectionClassID,
		BestDetectionConfidence: summary.BestDetectionConfidence,
		HasBestDetection:        summary.HasBestDetection,
		LidarFrontCm:            summary.LidarFrontCM,
		LidarLeftCm:             summary.LidarLeftCM,
		LidarRightCm:            summary.LidarRightCM,
		GyroYawDeg:              summary.GyroYawDeg,
	}
}
