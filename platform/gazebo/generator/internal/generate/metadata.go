package generate

import (
	"vtitan/gazebo/generator/internal/simconfig"
)

type (
	// Metadata is the JSON-serializable record written alongside each world SDF.
	// Pointer-bearing fields first: minimizes GC scan region (128 → 88 bytes).
	Metadata struct {
		ChallengeType      string               `json:"challenge_type"`
		Seed               *int64               `json:"seed,omitempty"`
		ParkingLot         *ParkingMeta         `json:"parking_lot,omitempty"`
		CorridorWidths     map[string]WidthMeta `json:"corridor_widths"`
		SignPositions      []SignMeta           `json:"sign_positions"`
		StartingConditions StartingMeta         `json:"starting_conditions"`
		ScenarioID         int                  `json:"scenario_id"`
		NumSigns           int                  `json:"num_signs"`
		HasParkingLot      bool                 `json:"has_parking_lot"`
	}

	// WidthMeta describes the resolved width type and dimension for a corridor section.
	WidthMeta struct {
		Type    string `json:"type"`
		WidthMM int    `json:"width_mm"`
	}

	// StartingMeta describes the robot's starting configuration for a scenario.
	StartingMeta struct {
		Direction string  `json:"direction"`
		Section   string  `json:"section"`
		Position  PosMeta `json:"position"`
		Yaw       float64 `json:"yaw"`
	}

	// PosMeta describes a 2D position in the world.
	PosMeta struct {
		X float64 `json:"x"`
		Y float64 `json:"y"`
	}

	// SignMeta describes a traffic sign's position and color.
	SignMeta struct {
		Color string  `json:"color"`
		X     float64 `json:"x"`
		Y     float64 `json:"y"`
	}

	// ParkingMeta describes the positions, orientation, and spacing of parking blocks (obstacles challenge).
	ParkingMeta struct {
		Block1Position PosMeta `json:"block1_position"`
		Block2Position PosMeta `json:"block2_position"`
		Block1Yaw      float64 `json:"block1_yaw"`
		Block2Yaw      float64 `json:"block2_yaw"`
		Depth          float64 `json:"depth"`
	}
)

// BuildMetadata constructs the Metadata struct for a generated scenario.
func BuildMetadata(
	idx int,
	challengeType simconfig.ScenarioType,
	corridorWidths map[simconfig.Section]simconfig.CorridorWidth,
	sc simconfig.StartingConditions,
	signs []simconfig.Sign,
	parking *simconfig.ParkingConfig,
	seed *int64,
) Metadata {
	cwMeta := make(map[string]WidthMeta, 4)
	for _, section := range simconfig.AllSections {
		w := corridorWidths[section]
		cwMeta[string(section)] = WidthMeta{
			Type:    w.Type,
			WidthMM: int(w.Width * simconfig.MillimetersPerMeter),
		}
	}

	signMeta := make([]SignMeta, len(signs))
	for i, s := range signs {
		signMeta[i] = SignMeta{X: s.Position[0], Y: s.Position[1], Color: s.Color.Name}
	}

	var parkMeta *ParkingMeta
	if parking != nil {
		parkMeta = &ParkingMeta{
			Block1Position: PosMeta{X: parking.Block1Pos[0], Y: parking.Block1Pos[1]},
			Block2Position: PosMeta{X: parking.Block2Pos[0], Y: parking.Block2Pos[1]},
			Block1Yaw:      parking.Block1Yaw,
			Block2Yaw:      parking.Block2Yaw,
			Depth:          parking.Depth,
		}
	}

	return Metadata{
		ScenarioID:     idx,
		ChallengeType:  string(challengeType),
		CorridorWidths: cwMeta,
		StartingConditions: StartingMeta{
			Direction: string(sc.Direction),
			Section:   sc.SectionName,
			Position:  PosMeta{X: sc.Position[0], Y: sc.Position[1]},
			Yaw:       sc.Yaw,
		},
		NumSigns:      len(signs),
		HasParkingLot: parking != nil,
		SignPositions: signMeta,
		ParkingLot:    parkMeta,
		Seed:          seed,
	}
}
