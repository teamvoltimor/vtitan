package controllers

import (
	"fmt"
	"log/slog"
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// StuckDiagnostics is the current stuck-detection diagnostics, matching
// StuckDetector.get_diagnostics' returned dict.
type StuckDiagnostics struct {
	IsStuck         bool
	StuckCount      int
	RecentMovementM float64
	HistorySize     int
	FrameCount      int
}

// StuckDetector detects and responds to stuck robot conditions, matching
// controllers.stuck_detector.StuckDetector.
//
// Maintains a position history and checks if the robot has moved a minimum
// distance in recent frames. If not, triggers recovery.
type StuckDetector struct {
	// MoveThreshold is the minimum movement distance to consider not
	// stuck (m).
	MoveThreshold float64
	// TimeoutFrames is the frames without movement before declaring stuck.
	TimeoutFrames int
	// HistorySize is the max position history to maintain.
	HistorySize int
	// ConfirmationChecks is the consecutive below-threshold checks
	// required before declaring the robot stuck.
	ConfirmationChecks int
	// minHistoryForDistance is the minimum tracked positions to compute
	// movement distance for GetDiagnostics.
	minHistoryForDistance int

	positionHistory []trackmodel.Waypoint
	frameCount      int
	stuckCount      int
	isStuck         bool
	logger          *slog.Logger
}

// NewStuckDetector builds a StuckDetector, matching StuckDetector.__init__.
// Returns an error when historySize < timeoutFrames: the position history
// would evict entries before the timeout window is reached, silently
// making the stuck check less sensitive. A nil logger falls back to
// slog.Default().
func NewStuckDetector(
	moveThreshold float64,
	timeoutFrames, historySize, confirmationChecks, minHistoryForDistance int,
	logger *slog.Logger,
) (*StuckDetector, error) {
	if historySize < timeoutFrames {
		return nil, fmt.Errorf(
			"controllers: history_size (%d) must be >= timeout_frames (%d), "+
				"otherwise the position history evicts entries before the timeout window "+
				"is reached and the stuck check silently becomes less sensitive",
			historySize, timeoutFrames,
		)
	}
	if logger == nil {
		logger = slog.Default()
	}
	return &StuckDetector{
		MoveThreshold:         moveThreshold,
		TimeoutFrames:         timeoutFrames,
		HistorySize:           historySize,
		ConfirmationChecks:    confirmationChecks,
		minHistoryForDistance: minHistoryForDistance,
		positionHistory:       make([]trackmodel.Waypoint, 0, historySize),
		logger:                logger,
	}, nil
}

// Update updates the detector with the current position, matching
// StuckDetector.update. Returns true if the robot is stuck.
func (s *StuckDetector) Update(currentPos trackmodel.Waypoint) bool {
	s.positionHistory = append(s.positionHistory, currentPos)
	if len(s.positionHistory) > s.HistorySize {
		s.positionHistory = s.positionHistory[len(s.positionHistory)-s.HistorySize:]
	}
	s.frameCount++

	if len(s.positionHistory) < s.TimeoutFrames {
		s.isStuck = false
		return false
	}

	oldPos := s.positionHistory[0]
	currPos := s.positionHistory[len(s.positionHistory)-1]
	distanceMoved := math.Hypot(currPos.X-oldPos.X, currPos.Y-oldPos.Y)

	if distanceMoved < s.MoveThreshold {
		s.stuckCount++
		if s.stuckCount > s.ConfirmationChecks {
			s.isStuck = true
			s.logger.Warn(
				"controllers: robot stuck",
				"moved_m",
				distanceMoved,
				"frames",
				s.TimeoutFrames,
			)
			return true
		}
	} else {
		s.stuckCount = 0
		s.isStuck = false
	}

	return s.isStuck
}

// Reset clears the stuck detector, e.g. after an escape maneuver, matching
// StuckDetector.reset.
func (s *StuckDetector) Reset() {
	s.positionHistory = s.positionHistory[:0]
	s.stuckCount = 0
	s.isStuck = false
}

// GetDiagnostics returns current diagnostics, matching
// StuckDetector.get_diagnostics.
func (s *StuckDetector) GetDiagnostics() StuckDiagnostics {
	distance := 0.0
	if len(s.positionHistory) >= s.minHistoryForDistance {
		last := s.positionHistory[len(s.positionHistory)-1]
		first := s.positionHistory[0]
		distance = math.Hypot(last.X-first.X, last.Y-first.Y)
	}

	return StuckDiagnostics{
		IsStuck:         s.isStuck,
		StuckCount:      s.stuckCount,
		RecentMovementM: distance,
		HistorySize:     len(s.positionHistory),
		FrameCount:      s.frameCount,
	}
}
