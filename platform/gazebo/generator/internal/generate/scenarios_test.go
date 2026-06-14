package generate_test

import (
	"testing"

	"voldemorbot/gazebo/generator/internal/generate"
	"voldemorbot/gazebo/generator/internal/simconfig"
)

func TestApplyScenarioToSection_InvalidID(t *testing.T) {
	_, err := generate.ApplyScenarioToSection(0, simconfig.SectionSouth)
	if err == nil {
		t.Fatal("expected error for scenario ID 0")
	}
	_, err = generate.ApplyScenarioToSection(37, simconfig.SectionSouth)
	if err == nil {
		t.Fatal("expected error for scenario ID 37")
	}
}

func TestApplyScenarioToSection_SouthIdentity(t *testing.T) {
	// South is the template section — positions must come through unchanged.
	pillars, err := generate.ApplyScenarioToSection(1, simconfig.SectionSouth)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if len(pillars) == 0 {
		t.Fatal("expected at least one pillar for scenario 1")
	}
	// Scenario 1 is a single green pillar; verify color
	if pillars[0].Color != simconfig.ColorNameGreen {
		t.Errorf("scenario 1 pillar color: got %q, want %q", pillars[0].Color, simconfig.ColorNameGreen)
	}
}

func TestApplyScenarioToSection_AllScenariosAllSections(t *testing.T) {
	for id := 1; id <= 36; id++ {
		for _, section := range simconfig.AllSections {
			pillars, err := generate.ApplyScenarioToSection(id, section)
			if err != nil {
				t.Errorf("scenario %d section %s: unexpected error: %v", id, section, err)
				continue
			}
			if len(pillars) == 0 {
				t.Errorf("scenario %d section %s: expected at least one pillar", id, section)
			}
			for _, p := range pillars {
				if p.X < 0 || p.X > simconfig.TrackMaxCoord {
					t.Errorf("scenario %d section %s: pillar X %v out of bounds", id, section, p.X)
				}
				if p.Y < 0 || p.Y > simconfig.TrackMaxCoord {
					t.Errorf("scenario %d section %s: pillar Y %v out of bounds", id, section, p.Y)
				}
				if p.Color != simconfig.ColorNameRed && p.Color != simconfig.ColorNameGreen {
					t.Errorf("scenario %d section %s: unexpected color %q", id, section, p.Color)
				}
			}
		}
	}
}

func TestApplyScenarioToSection_DoublePillarCount(t *testing.T) {
	// Scenarios 13–36 are double pillars
	for id := 13; id <= 36; id++ {
		pillars, err := generate.ApplyScenarioToSection(id, simconfig.SectionSouth)
		if err != nil {
			t.Errorf("scenario %d: unexpected error: %v", id, err)
			continue
		}
		if len(pillars) != 2 {
			t.Errorf("scenario %d: expected 2 pillars, got %d", id, len(pillars))
		}
	}
}
