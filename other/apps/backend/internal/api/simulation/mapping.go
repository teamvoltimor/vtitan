package simulation

import (
	"github.com/google/uuid"

	domain "github.com/teamvoltimor/vtitan/apps/backend/domain/simulation"
)

func parseUUID(s string) uuid.UUID {
	id, err := uuid.Parse(s)
	if err != nil {
		return uuid.Nil
	}
	return id
}

func toWireLighting(l *domain.LightingConfig) *LightingConfig {
	if l == nil {
		return nil
	}
	out := &LightingConfig{
		Intensity:        l.Intensity,
		AmbientIntensity: l.AmbientIntensity,
		CastShadows:      l.CastShadows,
		Scenario:         l.Scenario,
	}
	if l.Direction != nil {
		out.Direction = &l.Direction
	}
	return out
}

func toWireTrackConfig(t *domain.TrackConfig) *TrackConfig {
	if t == nil {
		return nil
	}
	out := &TrackConfig{TrackWidth: t.TrackWidth}
	if t.Sections != nil {
		out.Sections = &t.Sections
	}
	if t.CorridorWidths != nil {
		widths := make(map[string]struct {
			Type  *string  `json:"type,omitempty"`
			Width *float64 `json:"width,omitempty"`
		}, len(t.CorridorWidths))
		for k, v := range t.CorridorWidths {
			widths[k] = struct {
				Type  *string  `json:"type,omitempty"`
				Width *float64 `json:"width,omitempty"`
			}{Type: v.Type, Width: v.Width}
		}
		out.CorridorWidths = &widths
	}
	return out
}

func toWireParkingLot(p *domain.ParkingLotConfig) *ParkingLotConfig {
	if p == nil {
		return nil
	}
	out := &ParkingLotConfig{Depth: p.Depth}
	if p.Block1Position != nil {
		out.Block1Position = &p.Block1Position
	}
	if p.Block2Position != nil {
		out.Block2Position = &p.Block2Position
	}
	return out
}

func toWireStartingZone(z *domain.StartingZoneConfig) *StartingZoneConfig {
	if z == nil {
		return nil
	}
	out := &StartingZoneConfig{Length: z.Length}
	if z.Position != nil {
		out.Position = &z.Position
	}
	if z.Direction != nil {
		d := StartingZoneConfigDirection(*z.Direction)
		out.Direction = &d
	}
	return out
}

func toWireScenarioSummary(sc domain.Scenario) ScenarioSummary {
	return ScenarioSummary{
		Id:             parseUUID(sc.ID),
		Name:           sc.Name,
		Challenge:      ScenarioSummaryChallenge(sc.Challenge),
		CreatedAt:      sc.CreatedAt,
		DetectionCount: sc.DetectionCount,
		Environment:    sc.Environment,
	}
}

func toWireScenarioSummaries(scs []domain.Scenario) []ScenarioSummary {
	out := make([]ScenarioSummary, len(scs))
	for i, sc := range scs {
		out[i] = toWireScenarioSummary(sc)
	}
	return out
}

func toWireScenario(sc domain.Scenario) Scenario {
	return Scenario{
		Id:             parseUUID(sc.ID),
		Name:           sc.Name,
		Challenge:      ScenarioChallenge(sc.Challenge),
		CreatedAt:      sc.CreatedAt,
		DetectionCount: sc.DetectionCount,
		Environment:    sc.Environment,
		TrackConfig:    toWireTrackConfig(sc.TrackConfig),
		Lighting:       toWireLighting(sc.Lighting),
		ParkingLot:     toWireParkingLot(sc.ParkingLot),
		StartingZone:   toWireStartingZone(sc.StartingZone),
	}
}

func toWireMetrics(m *domain.RunMetrics) *RunMetrics {
	if m == nil {
		return nil
	}
	return &RunMetrics{
		AvgLapTimeS:      m.AvgLapTimeS,
		MaxSpeedMs:       m.MaxSpeedMS,
		AvgSpeedMs:       m.AvgSpeedMS,
		PathLengthM:      m.PathLengthM,
		ObstacleContacts: m.ObstacleContacts,
		SignDetections:   m.SignDetections,
	}
}

func toWireRun(r domain.Run) SimulationRun {
	return SimulationRun{
		Id:             parseUUID(r.ID),
		ScenarioId:     parseUUID(r.ScenarioID),
		Status:         SimulationRunStatus(r.Status),
		StartedAt:      r.StartedAt,
		CompletedAt:    r.CompletedAt,
		LapsCompleted:  r.LapsCompleted,
		TotalLaps:      r.TotalLaps,
		DurationS:      r.DurationS,
		CollisionCount: r.CollisionCount,
		Metrics:        toWireMetrics(r.Metrics),
	}
}

func toWireRuns(rs []domain.Run) []SimulationRun {
	out := make([]SimulationRun, len(rs))
	for i, r := range rs {
		out[i] = toWireRun(r)
	}
	return out
}

func toWireEnvironment(e domain.Environment) EnvironmentConfig {
	id := parseUUID(e.ID)
	name := e.Name
	wf := e.WorldFile
	return EnvironmentConfig{
		Id:        &id,
		Name:      &name,
		WorldFile: &wf,
		Lighting:  toWireLighting(e.Lighting),
	}
}

func toWireEnvironments(es []domain.Environment) []EnvironmentConfig {
	out := make([]EnvironmentConfig, len(es))
	for i, e := range es {
		out[i] = toWireEnvironment(e)
	}
	return out
}

func fromGenerateScenarioRequest(req GenerateScenarioRequest) domain.GenerateScenarioRequest {
	out := domain.GenerateScenarioRequest{
		Challenge: domain.Challenge(req.Challenge),
		NumSigns:  req.NumSigns,
		Randomize: req.Randomize,
	}
	if req.Corridors != nil {
		c := domain.Corridors(*req.Corridors)
		out.Corridors = &c
	}
	if req.Lighting != nil {
		l := domain.Lighting(*req.Lighting)
		out.Lighting = &l
	}
	return out
}

func fromStartRunRequest(req StartRunRequest) domain.StartRunRequest {
	return domain.StartRunRequest{
		ScenarioID: req.ScenarioId.String(),
		Laps:       req.Laps,
		Headless:   req.Headless,
	}
}
