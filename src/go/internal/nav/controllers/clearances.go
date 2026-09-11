package controllers

import "math"

// LidarClearances holds directional LIDAR clearance distances from
// obstacles (meters), matching shared.domain.models.LidarClearances.
type LidarClearances struct {
	FrontM float64
	LeftM  float64
	RightM float64
	BackM  float64
}

// ClearanceAggregate is how a sector's valid rays collapse to a single
// clearance number, matching clearances.ClearanceAggregate. Mean is for
// display (a single noisy return shouldn't dominate the readout); Min is
// for threat detection (the worst case in the cone is what matters for
// collision avoidance).
type ClearanceAggregate int

// ClearanceSectorSource is the structural interface ClearancesFromScan
// needs, matching clearances.CollisionAvoidanceControllerProtocol.
// *CollisionAvoidanceController satisfies it.
type ClearanceSectorSource interface {
	// SectorRangeValues returns valid ranges within center +/- halfFov.
	SectorRangeValues(
		rangesM, anglesRad []float64,
		centerRad, halfFovRad float64,
		filterSelfDetection bool,
	) []float64
	// ComputeRearClearance returns the minimum rear-sector clearance (m),
	// or the no-data sentinel if unseen.
	ComputeRearClearance(rangesM, anglesRad []float64) float64
}

const (
	// AggregateMean matches ClearanceAggregate.MEAN.
	AggregateMean ClearanceAggregate = iota
	// AggregateMin matches ClearanceAggregate.MIN.
	AggregateMin
)

// AnyBlocked reports whether any side is closer than threshold meters,
// matching LidarClearances.any_blocked.
func (l LidarClearances) AnyBlocked(threshold float64) bool {
	return l.FrontM < threshold || l.LeftM < threshold || l.RightM < threshold ||
		l.BackM < threshold
}

// MostConstrainedSide returns the side with the smallest clearance,
// matching LidarClearances.most_constrained_side.
func (l LidarClearances) MostConstrainedSide() ThreatDirection {
	side, dist := ThreatFront, l.FrontM
	if l.LeftM < dist {
		side, dist = ThreatLeft, l.LeftM
	}
	if l.RightM < dist {
		side, dist = ThreatRight, l.RightM
	}
	if l.BackM < dist {
		side = ThreatBack
	}
	return side
}

// ClearancesFromScan returns four-sided LIDAR clearances (meters) for
// scan, matching clearances.clearances_from_scan.
//
// Front/left/right are the aggregated reading across each sector. Mean
// (aggregate) matches the OLED display's existing behavior; Min matches
// DetectThreatDirection's threat sectors, for use in collision logic. The
// back sector always uses source's own min-based rear-cone logic.
func ClearancesFromScan(
	scan LidarScan,
	source ClearanceSectorSource,
	frontHalfFovRad float64,
	aggregate ClearanceAggregate,
) LidarClearances {
	if len(scan.RangesM) == 0 {
		return LidarClearances{}
	}

	front := source.SectorRangeValues(scan.RangesM, scan.AnglesRad, 0.0, frontHalfFovRad, false)
	left := source.SectorRangeValues(scan.RangesM, scan.AnglesRad, math.Pi/2, frontHalfFovRad, true)
	right := source.SectorRangeValues(
		scan.RangesM,
		scan.AnglesRad,
		-math.Pi/2,
		frontHalfFovRad,
		true,
	)

	return LidarClearances{
		FrontM: reduce(front, aggregate),
		LeftM:  reduce(left, aggregate),
		RightM: reduce(right, aggregate),
		BackM:  source.ComputeRearClearance(scan.RangesM, scan.AnglesRad),
	}
}

// ThreatDirectionFrom maps clearances to a ThreatDirection for escape
// logic, matching clearances.threat_direction. Returns the most-
// constrained side when anything is within noDetectionRangeM, else
// ThreatNone -- the same gate
// CollisionAvoidanceController.DetectThreatDirection applies, but driven
// off a shared LidarClearances so the navigator builds the scan's
// directional picture exactly once.
func ThreatDirectionFrom(clearances LidarClearances, noDetectionRangeM float64) ThreatDirection {
	if clearances.AnyBlocked(noDetectionRangeM) {
		return clearances.MostConstrainedSide()
	}
	return ThreatNone
}

// reduce collapses values to a single number per aggregate, returning 0.0
// for an empty slice (matching the Python `float(reducer(front)) if
// front.size else 0.0` fallback).
func reduce(values []float64, aggregate ClearanceAggregate) float64 {
	if len(values) == 0 {
		return 0.0
	}
	if aggregate == AggregateMin {
		m := values[0]
		for _, v := range values[1:] {
			m = min(m, v)
		}
		return m
	}
	sum := 0.0
	for _, v := range values {
		sum += v
	}
	return sum / float64(len(values))
}
