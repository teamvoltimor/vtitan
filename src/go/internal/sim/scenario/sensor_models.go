package scenario

import (
	"log/slog"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/stateestimator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/wallheading"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/harness"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/kinematics"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/sensormodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/visionsim"
)

// runClock hands out each tick's period: the nominal one, or with tick
// jitter one drawn from the measured distribution.
type runClock struct {
	dtS    float64
	ticks  *sensormodel.TickPeriod
	drawnS float64
}

// applySensorModels switches the harness-side sensor models on in hc: the
// LIDAR bands, and the IMU budget with the localizer and the wall-heading
// correction that bound it on the robot (Python forces localization on
// with any sensor error, for the same reason). Explicit --sensor-error
// flags win over the measured budget, so an IMU sweep can still move one
// term at a time.
func applySensorModels(
	logger *slog.Logger, hc *harness.Config, models sensormodel.Set, p sensormodel.Params,
	explicitErrors bool, configRoot string,
) {
	if models.Has(sensormodel.Lidar) {
		lidar := p.Lidar
		hc.InvalidRayRate = lidar.InvalidRayRate
		hc.LidarBands = &lidar
	}
	if models.Has(sensormodel.IMU) {
		if !explicitErrors {
			hc.SensorErrors = p.IMU
		}
		hc.Localize = true
		hc.HeadingCorrection = &harness.HeadingCorrection{
			Gain:  stateestimator.ConfigFor(logger, configRoot).YawCorrectionGain,
			Walls: wallheading.ConfigFor(logger, configRoot),
		}
	}
}

func (r *NativeRunner) newRunClock(dtS float64) *runClock {
	c := &runClock{dtS: dtS}
	if r.models.Has(sensormodel.TickJitter) {
		c.ticks = sensormodel.NewTickPeriod(r.modelParams, dtS, r.seed)
	}
	return c
}

// next returns the coming tick's period, seconds.
func (c *runClock) next() float64 {
	if c.ticks == nil {
		return c.dtS
	}
	p := c.ticks.Next()
	c.drawnS += p
	return p
}

// simTimeS is the run's simulated time after steps ticks: the nominal
// product without jitter, so every existing number's sim_time_s stays
// bit for bit what it was, else the drawn periods summed.
func (c *runClock) simTimeS(steps int) float64 {
	if c.ticks == nil {
		return float64(steps) * c.dtS
	}
	return c.drawnS
}

// modelFrame is the frame the navigator receives now with the camera model
// on: drawn once per tick, so asking twice in one tick sees one frame, and
// placed through the pose the robot BELIEVES it has, as Python's gateway
// and the real natsvision do.
func (v *simVisionGateway) modelFrame(st kinematics.AckermannState) ([]signrouter.TrafficSignObservation, bool) {
	now := v.nowS()
	if v.haveFrame && v.frameAtS == now {
		return v.frame, len(v.frame) > 0
	}
	believed := &visionsim.BelievedPose{X: st.X, Y: st.Y, Yaw: st.Yaw}
	if pose, ok := v.gw.GetCurrentPose(); ok {
		believed = &visionsim.BelievedPose{X: pose.X, Y: pose.Y, Yaw: pose.Yaw}
	}
	var obs []signrouter.TrafficSignObservation
	if v.camera != nil {
		obs, _ = v.camera.detections(st, believed, v.observe)
	} else {
		obs = v.observe(st, believed)
	}
	v.frameAtS, v.haveFrame, v.frame = now, true, obs
	return obs, len(obs) > 0
}

// observe emulates one frame seen from seen and placed through believed,
// through the camera model when it is on.
func (v *simVisionGateway) observe(
	seen kinematics.AckermannState, believed *visionsim.BelievedPose,
) []signrouter.TrafficSignObservation {
	signs := v.signs
	if v.model != nil {
		signs = v.model.Visible(signs, seen.X, seen.Y)
	}
	obs := visionsim.EmulateSignObservations(signs, seen.X, seen.Y, seen.Yaw, v.cfg, believed)
	if v.model != nil && believed != nil {
		v.model.Corrupt(obs, believed.X, believed.Y)
	}
	return obs
}
