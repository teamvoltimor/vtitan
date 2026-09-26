package sensormodel

import (
	"math"
	"math/rand/v2"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
)

// Camera is the measured camera model: which signs a frame detects and how
// wrong each detection is, as Python's get_vision_detections and
// _corrupt_detections.
type Camera struct {
	p   VisionParams
	rng *rand.Rand
}

// visionStreamSalt keeps the camera draws off every other stream.
const visionStreamSalt = 0x94d0_49bb_1331_11eb

// NewCamera builds the camera model on its own stream.
func NewCamera(p VisionParams, seed uint64) *Camera {
	return &Camera{p: p, rng: rand.New(rand.NewPCG(seed^visionStreamSalt, seed))}
}

// Visible returns the signs this frame detects, the chassis centre at
// (x, y): each one survives the range model, then a frame miss. Drawn per
// sign in list order, before the field-of-view test, as Python does.
func (v *Camera) Visible(signs []signrouter.SignSpec, x, y float64) []signrouter.SignSpec {
	var out []signrouter.SignSpec
	for _, s := range signs {
		if v.p.RangeModel && v.p.FalloffM > 0 {
			d := math.Hypot(s.X-x, s.Y-y)
			if v.rng.Float64() >= 1/(1+math.Exp((d-v.p.R50M)/v.p.FalloffM)) {
				continue
			}
		}
		if v.p.FrameMissRate > 0 && v.rng.Float64() < v.p.FrameMissRate {
			continue
		}
		out = append(out, s)
	}
	return out
}

// Corrupt applies the per-detection errors in place, in Python's order:
// a confidence drawn from the measured distribution, a bearing scatter
// about the pose the detection was reported through (ox, oy), keeping its
// range, and a red/green flip.
func (v *Camera) Corrupt(obs []signrouter.TrafficSignObservation, ox, oy float64) {
	if len(v.p.ConfidenceQuantiles) > 0 {
		for i := range obs {
			obs[i].Confidence = SampleQuantile(
				v.rng.Float64(), v.p.ConfidenceQuantiles, v.p.ConfidenceLevels, v.p.FallbackConfidence)
		}
	}
	if v.p.BearingScatterRad > 0 {
		for i := range obs {
			v.scatter(&obs[i], ox, oy)
		}
	}
	if v.p.ColorFlipRate > 0 {
		for i := range obs {
			v.flip(&obs[i])
		}
	}
}

// scatter rotates o about (ox, oy) by a Gaussian angle, keeping its range.
func (v *Camera) scatter(o *signrouter.TrafficSignObservation, ox, oy float64) {
	dx, dy := o.WorldXM-ox, o.WorldYM-oy
	r := math.Hypot(dx, dy)
	if r <= 0 {
		return
	}
	theta := math.Atan2(dy, dx) + v.rng.NormFloat64()*v.p.BearingScatterRad
	o.WorldXM = ox + r*math.Cos(theta)
	o.WorldYM = oy + r*math.Sin(theta)
}

// flip swaps red and green at the flip rate. Other colors draw nothing,
// as Python's short-circuit leaves them.
func (v *Camera) flip(o *signrouter.TrafficSignObservation) {
	switch o.Color {
	case signrouter.SignColorRed:
		if v.rng.Float64() < v.p.ColorFlipRate {
			o.Color = signrouter.SignColorGreen
		}
	case signrouter.SignColorGreen:
		if v.rng.Float64() < v.p.ColorFlipRate {
			o.Color = signrouter.SignColorRed
		}
	default:
	}
}
