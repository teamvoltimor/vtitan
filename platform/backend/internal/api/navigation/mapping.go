package navigation

import (
	"github.com/google/uuid"

	domain "github.com/teamvoldemor/voldemorbot/platform/backend/domain/navigation"
)

func parseUUID(s string) uuid.UUID {
	id, err := uuid.Parse(s)
	if err != nil {
		return uuid.Nil
	}
	return id
}

func toWireWaypoint(w domain.Waypoint) Waypoint {
	return Waypoint{
		Id:        parseUUID(w.ID),
		X:         w.X,
		Y:         w.Y,
		Index:     w.Index,
		Tolerance: w.Tolerance,
		Section:   w.Section,
	}
}

func toWireWaypoints(ws []domain.Waypoint) []Waypoint {
	out := make([]Waypoint, len(ws))
	for i, w := range ws {
		out[i] = toWireWaypoint(w)
	}
	return out
}

func toWireRoute(r domain.Route) Route {
	id := parseUUID(r.ID)
	return Route{
		Id:                 &id,
		Waypoints:          toWireWaypoints(r.Waypoints),
		TotalDistance:      r.TotalDistance,
		EstimatedDurationS: r.EstimatedDurationS,
		Laps:               r.Laps,
		CreatedAt:          r.CreatedAt,
	}
}

func toWireStatus(s domain.Status) NavigationStatus {
	out := NavigationStatus{
		Active:               s.Active,
		CurrentWaypointIndex: s.CurrentWaypointIndex,
		TotalWaypoints:       s.TotalWaypoints,
		CurrentSection:       s.CurrentSection,
		LastReplan:           s.LastReplan,
	}
	if s.RiskLevel != nil {
		rl := NavigationStatusRiskLevel(*s.RiskLevel)
		out.RiskLevel = &rl
	}
	if s.Phase != nil {
		ph := NavigationStatusPhase(*s.Phase)
		out.Phase = &ph
	}
	return out
}

func toWireClearance(c domain.Clearance) ClearanceData {
	return ClearanceData{
		Front:          c.Front,
		Left:           c.Left,
		Right:          c.Right,
		Back:           c.Back,
		PointsCaptured: c.PointsCaptured,
		RangeMin:       c.RangeMin,
		RangeMax:       c.RangeMax,
		Timestamp:      c.Timestamp,
	}
}

func toWireTuning(t domain.Tuning) NavigationTuning {
	return NavigationTuning{
		LookaheadShort:  t.LookaheadShort,
		LookaheadLong:   t.LookaheadLong,
		SteerKp:         t.SteerKp,
		MaxSteeringRate: t.MaxSteeringRate,
		ContactDist:     t.ContactDist,
		SlowDist:        t.SlowDist,
		MediumDist:      t.MediumDist,
		FastDist:        t.FastDist,
		MinSpeed:        t.MinSpeed,
		MaxSpeed:        t.MaxSpeed,
	}
}

func fromWireTuning(t NavigationTuning) domain.Tuning {
	return domain.Tuning{
		LookaheadShort:  t.LookaheadShort,
		LookaheadLong:   t.LookaheadLong,
		SteerKp:         t.SteerKp,
		MaxSteeringRate: t.MaxSteeringRate,
		ContactDist:     t.ContactDist,
		SlowDist:        t.SlowDist,
		MediumDist:      t.MediumDist,
		FastDist:        t.FastDist,
		MinSpeed:        t.MinSpeed,
		MaxSpeed:        t.MaxSpeed,
	}
}

func fromCreateWaypointRequest(req CreateWaypointRequest) domain.CreateWaypointRequest {
	return domain.CreateWaypointRequest{
		X:         req.X,
		Y:         req.Y,
		Tolerance: req.Tolerance,
		Section:   req.Section,
	}
}

func fromPlanRouteRequest(req PlanRouteRequest) domain.PlanRouteRequest {
	return domain.PlanRouteRequest{
		StartX:   req.StartX,
		StartY:   req.StartY,
		StartYaw: req.StartYaw,
		Laps:     req.Laps,
	}
}
