package simconfig

type (
	// Vec2 is a 2D coordinate pair [x, y] in meters.
	Vec2 [2]float64

	// RGB is a normalized color triple [r, g, b] in [0.0, 1.0].
	RGB [3]float64

	// CorridorWidth holds the resolved width for one section.
	CorridorWidth struct {
		Type  string  // WidthTypeNarrow | WidthTypeWide | WidthTypeFixed
		Width float64 // meters
	}

	// StartingZone is the computed position and length of the starting rectangle.
	StartingZone struct {
		Length float64
		X      float64
		Y      float64
	}

	// StartingConditions is the fully resolved robot starting state for one scenario.
	StartingConditions struct {
		Direction   Direction
		Section     Section
		SectionName string
		Position    Vec2
		Yaw         float64
		Zone        StartingZone
	}

	// ParkingConfig holds the resolved positions for both parking blocks.
	ParkingConfig struct {
		Block1Pos Vec2
		Block2Pos Vec2
		Block1Yaw float64
		Block2Yaw float64
		Depth     float64
	}

	// SignColor pairs a color name ("red"/"green") with its randomized RGB.
	SignColor struct {
		Name string
		RGB  RGB
	}

	// Sign is a traffic sign with a 2D world position and color.
	Sign struct {
		Color    SignColor
		Position Vec2
	}

	// LightingConfig is a fully resolved lighting configuration for one scenario.
	// Scenario first: minimizes GC scan region (string before float64s).
	LightingConfig struct {
		Scenario         string
		Intensity        float64
		AmbientIntensity float64
		Direction        [3]float64
		CastShadows      bool
	}
)
