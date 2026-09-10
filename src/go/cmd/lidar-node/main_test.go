//go:build linux

package main

import (
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/lidar"
)

func TestScanMessageFor_SortsByAngleAndKeepsRangesAligned(t *testing.T) {
	t.Parallel()

	scan := lidar.Scan{
		{AngleRad: 2.0, RangeM: 2.5, Quality: 20},
		{AngleRad: 0.0, RangeM: 0.5, Quality: 40},
		{AngleRad: 1.0, RangeM: 1.5, Quality: 30},
	}

	got := scanMessageFor(scan, time.Second)

	wantRanges := []float32{0.5, 1.5, 2.5}
	if len(got.GetRanges()) != len(wantRanges) {
		t.Fatalf("len(Ranges) = %d, want %d", len(got.GetRanges()), len(wantRanges))
	}
	for i, want := range wantRanges {
		if got.GetRanges()[i] != want {
			t.Errorf("Ranges[%d] = %v, want %v", i, got.GetRanges()[i], want)
		}
	}

	if got.GetAngleMin() != 0.0 {
		t.Errorf("AngleMin = %v, want 0", got.GetAngleMin())
	}
	if got.GetAngleMax() != 2.0 {
		t.Errorf("AngleMax = %v, want 2", got.GetAngleMax())
	}
	if got.GetAngleIncrement() != 1.0 {
		t.Errorf("AngleIncrement = %v, want 1", got.GetAngleIncrement())
	}
	if got.GetScanTime() != 1.0 {
		t.Errorf("ScanTime = %v, want 1", got.GetScanTime())
	}
	if got.GetRangeMin() != lidar.MinRangeM || got.GetRangeMax() != lidar.MaxRangeM {
		t.Errorf(
			"RangeMin/Max = %v/%v, want %v/%v",
			got.GetRangeMin(), got.GetRangeMax(), lidar.MinRangeM, lidar.MaxRangeM,
		)
	}
}

func TestScanMessageFor_EmptyScan(t *testing.T) {
	t.Parallel()

	got := scanMessageFor(nil, time.Second)

	if len(got.GetRanges()) != 0 {
		t.Errorf("len(Ranges) = %d, want 0", len(got.GetRanges()))
	}
	if got.GetAngleIncrement() != 0 {
		t.Errorf("AngleIncrement = %v, want 0", got.GetAngleIncrement())
	}
}

func TestScanMessageFor_SingleSample(t *testing.T) {
	t.Parallel()

	got := scanMessageFor(lidar.Scan{{AngleRad: 1.0, RangeM: 3.0, Quality: 10}}, time.Second)

	if got.GetAngleMin() != 1.0 || got.GetAngleMax() != 1.0 {
		t.Errorf("AngleMin/Max = %v/%v, want 1/1", got.GetAngleMin(), got.GetAngleMax())
	}
	if got.GetAngleIncrement() != 0 {
		t.Errorf("AngleIncrement = %v, want 0 (single sample, no spacing)", got.GetAngleIncrement())
	}
}
