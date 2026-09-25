package scenario

import (
	"math/rand/v2"
	"slices"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/harness"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/kinematics"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/visionsim"
)

// simCamera makes the emulated camera late and lossy, as the real vision
// pipeline is (harness.TransportConfig.DetectionDelayS and
// DetectionDropRate).
//
// A detection delivered now was seen from the pose the chassis had
// DetectionDelayS ago, and is placed in the world through the pose it has
// now: the relative bearing and range the camera measured are right, the
// pose they are attached to is not. That is the error a real 0.85 s
// pipeline makes while the robot keeps moving. A dropped frame yields no
// detections for that tick.
type simCamera struct {
	delayS   float64
	dropRate float64
	nowS     func() float64
	rand     *rand.Rand

	history []poseAt

	// frameAtS and frameDropped cache the drop draw per tick, so the
	// navigator asking twice in one tick sees one frame, not two.
	frameAtS     float64
	haveFrame    bool
	frameDropped bool
}

// poseAt is the chassis state at a sim time.
type poseAt struct {
	atS   float64
	state kinematics.AckermannState
}

// cameraStreamSalt keeps the frame-drop draws off every other stream, so
// switching the camera model on cannot perturb the LIDAR noise or the
// transport drops an unperturbed run was measured on.
const cameraStreamSalt = 0x2545_f491_4f6c_dd1d

// newSimCamera returns nil when tc configures no detection latency or
// drops.
func newSimCamera(tc harness.TransportConfig, nowS func() float64, seed uint64) *simCamera {
	if tc.DetectionDelayS == 0 && tc.DetectionDropRate == 0 {
		return nil
	}
	return &simCamera{
		delayS:   tc.DetectionDelayS,
		dropRate: tc.DetectionDropRate,
		nowS:     nowS,
		rand:     rand.New(rand.NewPCG(seed^cameraStreamSalt, seed)),
	}
}

// newVisionGateway builds the emulated camera for an Obstacles run over gw,
// late and lossy when the harness transport config says so.
func (r *NativeRunner) newVisionGateway(gw *harness.SimHardwareGateway, signs []signrouter.SignSpec) *simVisionGateway {
	simTimeS := func() float64 {
		odo, _ := gw.GetWheelOdometry()
		return odo.StampS
	}
	return &simVisionGateway{
		gw:     gw,
		signs:  signs,
		cfg:    visionsim.ConfigFrom(r.srCfg, r.cfg.DetectionConfidence),
		camera: newSimCamera(r.cfg.Transport, simTimeS, r.seed),
	}
}

// detections emulates the frame the navigator receives now, with the
// chassis currently at st.
func (c *simCamera) detections(
	signs []signrouter.SignSpec, st kinematics.AckermannState, cfg visionsim.Config,
) ([]signrouter.TrafficSignObservation, bool) {
	now := c.nowS()
	if n := len(c.history); n == 0 || c.history[n-1].atS < now {
		c.history = append(c.history, poseAt{atS: now, state: st})
	}

	if !c.haveFrame || c.frameAtS != now {
		c.frameAtS, c.haveFrame = now, true
		c.frameDropped = c.dropRate > 0 && c.rand.Float64() < c.dropRate
	}
	if c.frameDropped {
		return nil, false
	}

	seen, ok := c.seenFrom(now)
	if !ok {
		return nil, false
	}
	obs := visionsim.EmulateSignObservations(signs, seen.X, seen.Y, seen.Yaw, cfg,
		&visionsim.BelievedPose{X: st.X, Y: st.Y, Yaw: st.Yaw})
	return obs, len(obs) > 0
}

// seenFrom returns the newest recorded state at least delayS old at now,
// dropping history nothing can ask for again, and false while the run is
// younger than the delay.
func (c *simCamera) seenFrom(now float64) (kinematics.AckermannState, bool) {
	idx := -1
	for i, p := range slices.Backward(c.history) {
		if now-p.atS >= c.delayS-harness.TimeEpsilonS {
			idx = i
			break
		}
	}
	if idx < 0 {
		return kinematics.AckermannState{}, false
	}
	c.history = c.history[idx:]
	return c.history[0].state, true
}
