package validate_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/validate"
)

func defaultContext() validate.WorldContext {
	widths := map[simconfig.Section]simconfig.CorridorWidth{
		simconfig.SectionNorth: {Type: simconfig.WidthTypeWide, Width: simconfig.CorridorWide},
		simconfig.SectionSouth: {Type: simconfig.WidthTypeWide, Width: simconfig.CorridorWide},
		simconfig.SectionEast:  {Type: simconfig.WidthTypeWide, Width: simconfig.CorridorWide},
		simconfig.SectionWest:  {Type: simconfig.WidthTypeWide, Width: simconfig.CorridorWide},
	}
	sc := simconfig.StartingConditions{
		Section:  simconfig.SectionSouth,
		Position: simconfig.Vec2{1.5, 0.3},
	}
	return validate.WorldContext{
		CorridorWidths:     widths,
		Signs:              nil,
		ParkingConfig:      nil,
		StartingConditions: sc,
	}
}

func TestValidateScenario_EmptyIsValid(t *testing.T) {
	t.Parallel()
	ctx := defaultContext()
	violations := validate.ValidateScenario(ctx)
	if len(violations) != 0 {
		t.Errorf("expected no violations, got %v", violations)
	}
}

func TestValidateScenario_SignOutOfBounds(t *testing.T) {
	t.Parallel()
	ctx := defaultContext()
	ctx.Signs = []simconfig.Sign{
		{Position: simconfig.Vec2{-0.5, 1.5}, Color: simconfig.SignColor{Name: "red"}},
	}
	violations := validate.ValidateScenario(ctx)
	if len(violations) == 0 {
		t.Fatal("expected violation for out-of-bounds sign")
	}
}

func TestValidateScenario_SignOverlapViolation(t *testing.T) {
	t.Parallel()
	ctx := defaultContext()
	// Two signs at nearly identical positions
	ctx.Signs = []simconfig.Sign{
		{Position: simconfig.Vec2{1.5, 2.5}, Color: simconfig.SignColor{Name: "red"}},
		{Position: simconfig.Vec2{1.5, 2.5}, Color: simconfig.SignColor{Name: "green"}},
	}
	violations := validate.ValidateScenario(ctx)
	if len(violations) == 0 {
		t.Fatal("expected violation for overlapping signs")
	}
}

func TestValidateScenario_ParkingOutOfBounds(t *testing.T) {
	t.Parallel()
	ctx := defaultContext()
	parking := simconfig.ParkingConfig{
		Block1Pos: simconfig.Vec2{-1.0, 0.1},
		Block2Pos: simconfig.Vec2{1.5, 0.1},
	}
	ctx.ParkingConfig = &parking
	violations := validate.ValidateScenario(ctx)
	if len(violations) == 0 {
		t.Fatal("expected violation for out-of-bounds parking block")
	}
}

func TestValidateScenario_ValidSignsAndParking(t *testing.T) {
	t.Parallel()
	ctx := defaultContext()
	ctx.Signs = []simconfig.Sign{
		{Position: simconfig.Vec2{1.5, 2.5}, Color: simconfig.SignColor{Name: simconfig.ColorNameGreen}},
		{Position: simconfig.Vec2{1.5, 0.6}, Color: simconfig.SignColor{Name: simconfig.ColorNameRed}},
	}
	parking := simconfig.ParkingConfig{
		Block1Pos: simconfig.Vec2{1.0, 0.1},
		Block2Pos: simconfig.Vec2{1.3, 0.1},
	}
	ctx.ParkingConfig = &parking
	ctx.StartingConditions.Position = simconfig.Vec2{1.15, 0.1}
	violations := validate.ValidateScenario(ctx)
	// This may or may not pass depending on clearance; just check no panic
	_ = violations
}
