package sensormodel

import (
	"math"
	"math/rand/v2"
)

// Bands is the chassis-occlusion model over a fixed ray fan: the rays whose
// |bearing| falls in [BandMinRad, BandMaxRad] see the chassis, not the
// world, as Python's _init_lidar_occlusion and _apply_lidar_sensor_model.
type Bands struct {
	p        LidarParams
	occluded []bool
	rng      *rand.Rand
}

// lidarBandStreamSalt keeps the band draws off the LIDAR noise stream, so
// the noise and invalid-ray sequences stay those of a run without bands.
const lidarBandStreamSalt = 0x3c6e_f372_fe94_f82b

// selfReturnSpanSigmas clips the self-return draw to +-3 sigma, keeping it
// off the negative axis and below the filter floor.
const selfReturnSpanSigmas = 3.0

// NewBands builds the band mask for angles (robot frame, 0 = nose).
func NewBands(p LidarParams, angles []float64, seed uint64) *Bands {
	occluded := make([]bool, len(angles))
	for i, a := range angles {
		abs := math.Abs(a)
		occluded[i] = abs >= p.BandMinRad && abs <= p.BandMaxRad
	}
	return &Bands{p: p, occluded: occluded, rng: rand.New(rand.NewPCG(seed^lidarBandStreamSalt, seed))}
}

// Occluded reports whether ray i sees the chassis; a nil Bands occludes
// nothing.
func (b *Bands) Occluded(i int) bool {
	return b != nil && i < len(b.occluded) && b.occluded[i]
}

// Apply replaces every band ray, after noise, with either no return
// (+Inf, BandDropoutRate) or a self-return off the chassis. It runs after
// the noise on purpose: the range noise describes wall ranging, not a
// surface 2 cm from the lens.
func (b *Bands) Apply(ranges []float64) {
	lo := math.Max(0, b.p.SelfReturnM-selfReturnSpanSigmas*b.p.SelfReturnStdM)
	hi := b.p.SelfReturnM + selfReturnSpanSigmas*b.p.SelfReturnStdM
	for i := range ranges {
		if !b.Occluded(i) {
			continue
		}
		if b.rng.Float64() < b.p.BandDropoutRate {
			ranges[i] = math.Inf(1)
			continue
		}
		ranges[i] = min(max(b.p.SelfReturnM+b.rng.NormFloat64()*b.p.SelfReturnStdM, lo), hi)
	}
}
