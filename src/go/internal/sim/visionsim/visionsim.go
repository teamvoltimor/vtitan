// Package visionsim ports
// platform/robot/src/simulation/vision_emulator.py: a synthetic
// camera-detection emulator for closed-loop sign-routing sim runs.
//
// Inverts the exact pinhole projection signrouter's discovery.go decodes,
// so a SignRouter driven through this emulator exercises the real
// camera-confirmation code path (MatchDetectionToSign) instead of always
// seeing no observations.
//
// Deliberately simple -- a fixed high confidence, no false positives, no
// wall-occlusion modeling (the LIDAR raycast model in internal/sim/collision
// does model occlusion; this does not). The goal is exercising the
// detection-to-router wiring end-to-end, not building a camera sensor
// model.
package visionsim

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// Config bundles the tuning values EmulateSignObservations reads, matching
// the handful of values Python's vision_emulator.py pulls from
// RobotSpecs/NavigationTuning.simulation.
type Config struct {
	// CameraHFOVRad is the horizontal field of view a sign must fall
	// within to be "seen", matching RobotSpecs.CAMERA_HFOV.
	CameraHFOVRad float64
	// MaxRangeM is the maximum detection range, matching
	// RobotSpecs.CAMERA_FAR_CLIP (Python's own default for this
	// function's max_range parameter).
	MaxRangeM float64
	// DetectionConfidence is the fixed confidence reported for every
	// emulated detection, matching tuning.simulation.DETECTION_CONFIDENCE.
	DetectionConfidence float64
}

// BelievedPose is the robot's BELIEVED (possibly diverged from true) pose,
// to reproject an emulated detection's relative bearing/range through --
// matching emulate_sign_observations' believed_pos/believed_yaw
// parameters. nil means "reproject through the true pose" (ground-truth
// mode, no belief error), matching Python's believed_pos=None default.
type BelievedPose struct {
	X, Y, Yaw float64
}

// DefaultConfig returns the Config matching the shipped defaults:
// signrouter's own camera geometry defaults for HFOV/range (which mirror
// robot.toml's [camera] section) and simulation.toml's
// detection_confidence (0.9).
//
// ConfigFrom is preferred wherever a resolved signrouter.Config is on hand,
// so a run with a --config-root sees the shipped camera rather than these
// fallbacks.
func DefaultConfig() Config {
	return Config{
		CameraHFOVRad:       signrouter.DefaultCameraHFOVRad,
		MaxRangeM:           signrouter.DefaultCameraFarClipM,
		DetectionConfidence: 0.9,
	}
}

// ConfigFrom takes the camera geometry from an already-resolved
// signrouter.Config, so the emulated camera and the sign router that
// consumes its detections agree on what the lens is.
func ConfigFrom(sr signrouter.Config, detectionConfidence float64) Config {
	return Config{
		CameraHFOVRad:       sr.CameraHFOVRad,
		MaxRangeM:           sr.CameraFarClipM,
		DetectionConfidence: detectionConfidence,
	}
}

// EmulateSignObservations returns synthetic TrafficSignObservations for
// every sign in signs that is within cfg.MaxRangeM and within
// cfg.CameraHFOVRad of the robot's TRUE heading, matching
// emulate_sign_observations.
//
// robotX/robotY/robotYaw is the robot's TRUE pose -- what the camera
// actually sees from, used only to decide whether a sign is in frame and
// how far away it genuinely is. believed is the robot's BELIEVED pose to
// report the observation's world coordinates through; nil reprojects
// through the true pose itself.
//
// Reporting true coordinates unconditionally would leak ground truth
// straight past any localizer error: a SignRouter fed a discovered sign
// this way and a Navigator steering on a diverged pose estimate would be
// comparing two different reference frames, which is incoherent regardless
// of which frame is "right." A real camera has the same blind spot -- it
// measures a correct bearing/range but has only the robot's own (possibly
// wrong) pose to convert that into a world position -- so reproducing that
// error here, rather than omitting it, is what makes a diverged localizer's
// cost show up in sim instead of being invisible.
func EmulateSignObservations(
	signs []signrouter.SignSpec,
	robotX, robotY, robotYaw float64,
	cfg Config,
	believed *BelievedPose,
) []signrouter.TrafficSignObservation {
	reportX, reportY, reportYaw := robotX, robotY, robotYaw
	if believed != nil {
		reportX, reportY, reportYaw = believed.X, believed.Y, believed.Yaw
	}
	robotPos := trackmodel.Waypoint{X: robotX, Y: robotY}

	var observations []signrouter.TrafficSignObservation
	for _, sign := range signs {
		signPos := trackmodel.Waypoint{X: sign.X, Y: sign.Y}
		distance := robotPos.DistanceTo(signPos)
		if distance <= 0.0 || distance > cfg.MaxRangeM {
			continue
		}

		bearing := robotPos.BearingTo(signPos)
		thetaH := navutil.WrapAngle(bearing - robotYaw)
		if math.Abs(thetaH) > cfg.CameraHFOVRad/navutil.Half {
			continue
		}

		// Reproject the TRUE relative bearing/range through the BELIEVED
		// pose -- the same relative geometry a real camera measured,
		// placed in world coordinates using the only pose the robot
		// actually has.
		reportBearing := reportYaw + thetaH
		observations = append(observations, signrouter.TrafficSignObservation{
			WorldXM:             reportX + distance*math.Cos(reportBearing),
			WorldYM:             reportY + distance*math.Sin(reportBearing),
			Color:               sign.Color,
			Confidence:          cfg.DetectionConfidence,
			DetectedAtTimestamp: 0.0,
		})
	}
	return observations
}
