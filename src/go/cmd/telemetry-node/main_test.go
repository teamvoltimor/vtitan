//go:build linux

package main

import (
	"context"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/telemetry/diag"

	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
)

func TestNatsSource_LatestScanIMU_ReportsAbsenceUntilSet(t *testing.T) {
	t.Parallel()

	source := &natsSource{}
	ctx := context.Background()

	if _, ok := source.LatestScan(ctx); ok {
		t.Error("LatestScan() ok = true before any scan was set, want false")
	}
	if _, ok := source.LatestIMU(ctx); ok {
		t.Error("LatestIMU() ok = true before any IMU reading was set, want false")
	}

	wantScan := &sensorv1.Scan{FrameId: "lidar_link"}
	source.setScan(wantScan)
	if got, ok := source.LatestScan(ctx); !ok || got != wantScan {
		t.Errorf("LatestScan() = (%v, %v), want (%v, true)", got, ok, wantScan)
	}

	wantIMU := &sensorv1.Imu{FrameId: "imu_link"}
	source.setIMU(wantIMU)
	if got, ok := source.LatestIMU(ctx); !ok || got != wantIMU {
		t.Errorf("LatestIMU() = (%v, %v), want (%v, true)", got, ok, wantIMU)
	}
}

func TestNatsSource_LatestDetections_AlwaysAbsent(t *testing.T) {
	t.Parallel()

	source := &natsSource{}
	if _, ok := source.LatestDetections(context.Background()); ok {
		t.Error("LatestDetections() ok = true, want false (no vision wire schema yet)")
	}
}

func TestSummaryMessageFor(t *testing.T) {
	t.Parallel()

	summary := diag.TelemetrySummary{
		BestDetectionClassID:    "sign_red",
		BestDetectionConfidence: 0.9,
		HasBestDetection:        true,
		LidarFrontCM:            42,
		LidarLeftCM:             10,
		LidarRightCM:            20,
		GyroYawDeg:              180,
	}

	got := summaryMessageFor(summary)

	if got.GetBestDetectionClassId() != summary.BestDetectionClassID {
		t.Errorf(
			"BestDetectionClassId = %q, want %q",
			got.GetBestDetectionClassId(),
			summary.BestDetectionClassID,
		)
	}
	if got.GetBestDetectionConfidence() != summary.BestDetectionConfidence {
		t.Errorf(
			"BestDetectionConfidence = %v, want %v",
			got.GetBestDetectionConfidence(), summary.BestDetectionConfidence,
		)
	}
	if got.GetHasBestDetection() != summary.HasBestDetection {
		t.Errorf(
			"HasBestDetection = %v, want %v",
			got.GetHasBestDetection(),
			summary.HasBestDetection,
		)
	}
	if got.GetLidarFrontCm() != summary.LidarFrontCM ||
		got.GetLidarLeftCm() != summary.LidarLeftCM ||
		got.GetLidarRightCm() != summary.LidarRightCM {
		t.Errorf(
			"Lidar F/L/R = %v/%v/%v, want %v/%v/%v",
			got.GetLidarFrontCm(), got.GetLidarLeftCm(), got.GetLidarRightCm(),
			summary.LidarFrontCM, summary.LidarLeftCM, summary.LidarRightCM,
		)
	}
	if got.GetGyroYawDeg() != summary.GyroYawDeg {
		t.Errorf("GyroYawDeg = %v, want %v", got.GetGyroYawDeg(), summary.GyroYawDeg)
	}
}
