package statemachine

import (
	"fmt"

	"google.golang.org/protobuf/types/known/timestamppb"

	statev1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/state/v1"
	uiv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/ui/v1"
	smcore "github.com/teamvoltimor/vtitan/platform/robot-go/internal/statemachine/core"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

// NATSRaceMetricsSink publishes race progress on the
// vtitan.state.v1.race_metrics subject, replacing state_machine_node's
// /race_metrics std_msgs/String publisher.
type NATSRaceMetricsSink struct {
	pub *nats.Publisher[*statev1.RaceMetrics]
}

// NATSSystemStatusSink publishes boot-check readiness on the
// vtitan.state.v1.system_status subject.
type NATSSystemStatusSink struct {
	pub *nats.Publisher[*statev1.SystemStatus]
}

// NewNATSRaceMetricsSink builds a NATSRaceMetricsSink over a connected pub.
func NewNATSRaceMetricsSink(pub *nats.Publisher[*statev1.RaceMetrics]) *NATSRaceMetricsSink {
	return &NATSRaceMetricsSink{pub: pub}
}

// PublishRaceMetrics publishes status, with targetLaps nil until the state
// machine knows the lap target for this race.
func (s *NATSRaceMetricsSink) PublishRaceMetrics(status smcore.RaceStatus, targetLaps *int) error {
	if err := s.pub.Publish(RaceMetricsMessageFor(status, targetLaps)); err != nil {
		return fmt.Errorf("node/statemachine: publishing RaceMetrics: %w", err)
	}
	return nil
}

// RaceMetricsMessageFor converts a race snapshot into its wire form.
//
// The unit-suffixed field names come straight from RaceStatus's own: steering
// and yaw are DEGREES here, not the normalized steering of ports.DriveCommand
// nor the radians every other angle in these protos uses.
func RaceMetricsMessageFor(status smcore.RaceStatus, targetLaps *int) *statev1.RaceMetrics {
	message := &statev1.RaceMetrics{
		Stamp:              timestamppb.Now(),
		LapsCompleted:      int32(status.LapsCompleted),
		TotalRaceTimeS:     status.TotalRaceTimeSec,
		CurrentVelocityMps: status.CurrentVelocityMPS,
		CurrentSteeringDeg: status.CurrentSteeringDeg,
		GyroYawDeg:         status.GyroYawDeg,
	}
	if targetLaps != nil {
		message.TargetLaps = new(int32(*targetLaps))
	}
	// Left empty rather than defaulted when the corridor is unknown: the OLED
	// blanks that line instead of asserting a side the navigator has not
	// classified yet.
	if status.CurrentCorridor != nil {
		message.CurrentCorridor = *status.CurrentCorridor
	}
	return message
}

// NewNATSSystemStatusSink builds a NATSSystemStatusSink over a connected pub.
func NewNATSSystemStatusSink(pub *nats.Publisher[*statev1.SystemStatus]) *NATSSystemStatusSink {
	return &NATSSystemStatusSink{pub: pub}
}

// PublishSystemStatus publishes status as this node's diagnostic report.
func (s *NATSSystemStatusSink) PublishSystemStatus(status smcore.SystemStatus) error {
	if err := s.pub.Publish(SystemStatusMessageFor(status)); err != nil {
		return fmt.Errorf("node/statemachine: publishing SystemStatus: %w", err)
	}
	return nil
}

// SystemStatusMessageFor converts a boot-check snapshot into its wire form,
// one Status entry per sensor plus a rollup entry for the network and
// overall readiness.
//
// The schema is DiagnosticArray-shaped because the topic has several
// independent producers; this one contributes entries named after its own
// sensors rather than assuming it owns the whole message.
func SystemStatusMessageFor(status smcore.SystemStatus) *statev1.SystemStatus {
	sensors := []struct {
		name   string
		sensor smcore.SensorStatus
	}{
		{"imu", status.IMUStatus},
		{"lidar", status.LidarStatus},
		{"hailo", status.HailoStatus},
		{"drive", status.DriveStatus},
		{"challenge_mode", status.ChallengeModeStatus},
	}

	entries := make([]*statev1.SystemStatus_Status, 0, len(sensors)+1)
	for _, entry := range sensors {
		// SensorStatus.Name is the sensor's own label and may be empty; the
		// schema requires a non-empty name, so fall back to the fixed key
		// rather than emitting a message that fails its own validation.
		name := entry.sensor.Name
		if name == "" {
			name = entry.name
		}
		entries = append(entries, &statev1.SystemStatus_Status{
			Level:      levelFor(entry.sensor.IsReady),
			Name:       name,
			Message:    entry.sensor.ErrorMessage,
			HardwareId: entry.name,
		})
	}

	rollup := &statev1.SystemStatus_Status{
		Level:   levelFor(status.AllReady),
		Name:    "state_machine",
		Message: status.NetworkStatus,
	}
	if status.ChallengeMode != nil {
		rollup.Values = []*statev1.SystemStatus_KeyValue{
			{Key: "challenge_mode", Value: status.ChallengeMode.String()},
		}
	}
	entries = append(entries, rollup)

	return &statev1.SystemStatus{Stamp: timestamppb.Now(), Status: entries}
}

// levelFor maps readiness onto a diagnostic level. Not-ready is ERROR rather
// than WARN: boot check gates the race starting at all, so anything short of
// ready is blocking.
func levelFor(ready bool) statev1.SystemStatus_Level {
	if ready {
		return statev1.SystemStatus_LEVEL_OK
	}
	return statev1.SystemStatus_LEVEL_ERROR
}

// ChallengeModeMessageFor converts the resolved challenge into its wire form
// for the vtitan.ui.v1.challenge_mode_active subject.
func ChallengeModeMessageFor(scenario smcore.ScenarioType) *uiv1.ChallengeModeActive {
	return &uiv1.ChallengeModeActive{
		Stamp:     timestamppb.Now(),
		Challenge: challengeFor(scenario),
	}
}

// JumperInsertedMessageFor converts the raw jumper reading into its wire
// form for the vtitan.ui.v1.jumper_inserted subject.
//
// Separate from ChallengeModeMessageFor on purpose: this is what the pin
// reads, that is what the stack should act on. Collapsing them would lose
// the difference between "no jumper fitted, defaulting to Open" and "jumper
// fitted, selecting Open".
func JumperInsertedMessageFor(inserted bool) *uiv1.JumperInserted {
	return &uiv1.JumperInserted{Stamp: timestamppb.Now(), Inserted: inserted}
}

func challengeFor(scenario smcore.ScenarioType) uiv1.Challenge {
	switch scenario {
	case smcore.ScenarioOpen:
		return uiv1.Challenge_CHALLENGE_OPEN
	case smcore.ScenarioObstacles:
		return uiv1.Challenge_CHALLENGE_OBSTACLES
	default:
		return uiv1.Challenge_CHALLENGE_UNSPECIFIED
	}
}
