package signrouter

import (
	"log/slog"
	"math"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/signs"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with track.toml (corner/track coordinates and sign width),
// robot.toml (chassis dimensions) and sign_router.toml (the tuning knobs),
// each loaded from <configRoot>/profile.DefaultXxxTOMLPath, if configRoot
// is non-empty and loading succeeds; otherwise, or on any load failure, the
// literal defaults for that file, logging why.
//
// Track and robot geometry load first because LateralOffsetM/
// ChassisHalfDiagonalM/BehindToleranceM are DERIVED from them (chassis
// half-diagonal + sign half-width + a tuning margin) rather than raw
// tunables -- ConfigFor recomputes the derivation from whatever chassis/
// sign dimensions it ends up with (freshly loaded or the DefaultConfig
// fallback), the same way SignRouterConfig.from_tuning always recomputes it
// from CHASSIS_HALF_DIAGONAL rather than trusting a stale copy. See
// Config.LateralOffsetM's doc comment for why this matters at sub-mm scale.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	signWidthM := DefaultSignWidthM
	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultTrackTOMLPath), nil,
		func(tc generated.TrackConfig) {
			cfg.TrackMinCoordM = tc.Track.MinCoord
			cfg.TrackMaxCoordM = tc.Track.MaxCoord
			cfg.TrackCornerMinM = tc.Track.CornerMin
			cfg.TrackCornerMaxM = tc.Track.CornerMax
			signWidthM = tc.Sign.Width
			cfg.SignWidthM = tc.Sign.Width
			cfg.SignHeightM = tc.Sign.Height
			cfg.GridDepthNear = tc.Sign.GridDepthNear
			cfg.GridDepthMiddle = tc.Sign.GridDepthMiddle
			cfg.GridDepthFar = tc.Sign.GridDepthFar
			cfg.GridWidthOuter = tc.Corridor.DivisionLines[0]
			cfg.GridWidthInner = tc.Corridor.DivisionLines[1]
			cfg.TrackSizeM = tc.Track.Size
		})

	robotPath := filepath.Join(configRoot, profile.DefaultRobotTOMLPath)
	if rc, err := profile.LoadRobotValues(robotPath, nil); err != nil {
		logger.Warn("signrouter: loading robot.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.ChassisHalfDiagonalM = chassisHalfDiagonalM(rc.Chassis.Length, rc.Chassis.Width)
		cfg.BehindToleranceM = behindToleranceM(rc.Chassis.Length)
		cfg.CameraHFOVRad = rc.Camera.Hfov
		cfg.CameraWidthPX = float64(rc.Camera.Width)
		cfg.CameraFarClipM = rc.Camera.FarClip
		cfg.SensorMountXOffsetM = rc.Camera.MountXOffset
	}

	signClearanceMarginM := DefaultSignClearanceMarginM
	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultSignRouterTOMLPath), nil,
		func(sr signs.NavigationSignsSignRouter) {
			signClearanceMarginM = sr.SignClearanceMarginM
			cfg.ActivationDistM = sr.ActivationDistM
			cfg.PassedDistM = sr.PassedDistM
			cfg.DepthPin = sr.DepthPin
			cfg.DetectionMatchDistM = sr.DetectionMatchDistM
			cfg.MinConfidence = sr.MinConfidence
			cfg.CommitHysteresis = sr.CommitHysteresis
			cfg.CorridorFlipTicks = sr.CorridorFlipTicks
			cfg.SettleTicks = sr.SettleTicks
			cfg.RelabelUnsatisfiable = sr.SignLaneRelabelUnsatisfiable
			cfg.DepthConsistentCorridor = sr.SignLaneDepthConsistentCorridor
			cfg.WallClearanceMarginM = sr.WallClearanceMarginM
			cfg.DeformDepthBufferM = sr.DeformDepthBufferM
			cfg.PinCornerGuard = sr.PinCornerGuard
			cfg.PinHeadingGuard = sr.PinHeadingGuard
			cfg.PinHeadingGuardRad = sr.PinHeadingGuardDeg * math.Pi / navutil.DegreesPerHalfTurn
			cfg.SlotSignMap = sr.SlotSignMap
			cfg.SlotAcceptRadiusM = sr.SlotAcceptRadiusM
			cfg.SlotMinEvidence = sr.SlotMinEvidence
			cfg.SlotRepointMargin = sr.SlotRepointMargin
		})

	cfg.LateralOffsetM = cfg.ChassisHalfDiagonalM + signWidthM/2 + signClearanceMarginM
	return cfg
}

// DiscoveryConfigFor resolves the DiscoveryConfig to run with:
// DefaultDiscoveryConfig's literals overlaid with sign_discovery.toml and the
// track geometry from track.toml, when configRoot is non-empty and loading
// succeeds; otherwise the literal defaults, logging why.
//
// The corridor bounds come from track.toml rather than being restated, for the
// same reason ConfigFor takes them from there: the association gate settles the
// robot's corridor with them, so a discovery map disagreeing with the router
// about where a corridor ends would associate observations to the wrong one.
func DiscoveryConfigFor(logger *slog.Logger, configRoot string) DiscoveryConfig {
	cfg := DefaultDiscoveryConfig()
	if configRoot == "" {
		return cfg
	}

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultTrackTOMLPath), nil,
		func(tc generated.TrackConfig) {
			cfg.CornerMinM = tc.Track.CornerMin
			cfg.CornerMaxM = tc.Track.CornerMax
		})

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultSignDiscoveryTOMLPath), nil,
		func(sd signs.NavigationSignsSignDiscovery) {
			cfg.MinReliableBBoxHeightPX = float64(sd.MinReliableBboxHeightPx)
			cfg.MaxIngestRangeM = sd.MaxIngestRangeM
			cfg.AssociationDistM = sd.AssociationDistM
			cfg.MinHits = sd.MinHits
			cfg.RobotCorridorFlipTicks = sd.RobotCorridorFlipTicks
		})

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultSignRouterTOMLPath), nil,
		func(sr signs.NavigationSignsSignRouter) {
			cfg.MinConfidence = sr.MinConfidence
		})

	return cfg
}
