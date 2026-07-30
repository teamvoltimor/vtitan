package robot

import (
	"github.com/google/uuid"

	domain "github.com/teamvoltimor/vtitan/platform/backend/domain/robot"
)

// parseUUID parses s as a UUID, falling back to uuid.Nil on failure. Used only
// for values already validated at the boundary (binding tags) or generated
// internally, so failure here indicates a bug rather than bad user input.
func parseUUID(s string) uuid.UUID {
	id, err := uuid.Parse(s)
	if err != nil {
		return uuid.Nil
	}
	return id
}

func toWireRobot(r domain.Robot) Robot {
	return Robot{
		Id:        parseUUID(r.ID),
		Name:      r.Name,
		FleetId:   parseUUID(r.FleetID),
		State:     RobotState(r.State),
		CreatedAt: r.CreatedAt,
		UpdatedAt: r.UpdatedAt,
	}
}

func toWireRobots(rs []domain.Robot) []Robot {
	out := make([]Robot, len(rs))
	for i, r := range rs {
		out[i] = toWireRobot(r)
	}
	return out
}

func toWireStatus(s domain.Status) RobotStatus {
	out := RobotStatus{
		RobotId:     parseUUID(s.RobotID),
		State:       RobotState(s.State),
		Pose:        Pose{X: s.Pose.X, Y: s.Pose.Y, Yaw: s.Pose.Yaw},
		Velocity:    Velocity{Linear: s.Velocity.Linear, Angular: s.Velocity.Angular},
		LapNumber:   s.LapNumber,
		MissionName: s.MissionName,
		Timestamp:   s.Timestamp,
		SystemStatus: SystemStatus{
			CpuTemp:        s.SystemStatus.CPUTemp,
			RosNodesActive: s.SystemStatus.ROSNodesActive,
			ImuReady:       s.SystemStatus.IMUReady,
			LidarReady:     s.SystemStatus.LidarReady,
			CameraReady:    s.SystemStatus.CameraReady,
			DriveReady:     s.SystemStatus.DriveReady,
		},
	}
	if s.SystemStatus.MotorTemps != nil {
		out.SystemStatus.MotorTemps = &s.SystemStatus.MotorTemps
	}
	if s.Battery != nil {
		out.Battery = &BatteryStatus{
			Voltage:       s.Battery.Voltage,
			Current:       s.Battery.Current,
			StateOfCharge: s.Battery.StateOfCharge,
		}
	}
	return out
}

func toWireConfig(c domain.Config) RobotConfig {
	out := RobotConfig{
		MaxLinearSpeed:    c.MaxLinearSpeed,
		MaxAngularSpeed:   c.MaxAngularSpeed,
		SteeringKp:        c.SteeringKp,
		LookaheadDistance: c.LookaheadDistance,
	}
	if c.SpeedProfiles != nil {
		out.SpeedProfiles = &c.SpeedProfiles
	}
	return out
}

func toWireCommandResult(r domain.CommandResult) RobotCommandResponse {
	out := RobotCommandResponse{
		CommandId: parseUUID(r.CommandID),
		Status:    RobotCommandResponseStatus(r.Status),
	}
	if r.Message != "" {
		out.Message = &r.Message
	}
	return out
}

func fromCreateRequest(req CreateRobotRequest) domain.CreateRequest {
	return domain.CreateRequest{Name: req.Name, FleetID: req.FleetId.String()}
}

func fromUpdateRequest(req UpdateRobotRequest) domain.UpdateRequest {
	return domain.UpdateRequest{Name: req.Name}
}

func fromUpdateConfigRequest(req UpdateRobotConfigRequest) domain.UpdateConfigRequest {
	return domain.UpdateConfigRequest{
		MaxLinearSpeed:    req.MaxLinearSpeed,
		MaxAngularSpeed:   req.MaxAngularSpeed,
		SteeringKp:        req.SteeringKp,
		LookaheadDistance: req.LookaheadDistance,
		SpeedProfile:      req.SpeedProfile,
	}
}

func fromCommand(req RobotCommand) domain.Command {
	cmd := domain.Command{Type: domain.CommandType(req.CommandType)}
	if req.Parameters != nil {
		cmd.Parameters = *req.Parameters
	}
	return cmd
}
