package picolink

import (
	"fmt"
	"math"
	"strings"

	"google.golang.org/protobuf/types/known/timestamppb"

	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
)

// FrameID is the frame_id on every message this package publishes: the
// Zero's internal/node/motor.FrameID.
const FrameID = "base_link"

// DetailUnconfigured is MotorStatus.Detail for a board that has not accepted
// a Config yet.
const DetailUnconfigured = "unconfigured"

// radiansPerRevolution and secondsPerMinute convert revolutions and RPM into
// the SI units JointStates carries, as internal/node/motor.JointStatesFor
// does.
const (
	radiansPerRevolution = 2 * math.Pi
	secondsPerMinute     = 60.0
)

// faultNames are the boardlink fault bits in the words MotorStatus.Detail
// uses, lowest bit first.
var faultNames = [...]struct {
	bit  uint8
	name string
}{
	{boardlink.FaultWatchdogReset, "watchdog_reset"},
	{boardlink.FaultActuator, "actuator"},
	{boardlink.FaultRejectedCommand, "rejected_command"},
}

// MotorStatusFor maps a board Status onto the MotorStatus the Zero
// publishes (internal/node/motor.StatusFor).
//
// State:
//
//   - Idle, Running and Fault map one to one.
//   - Unconfigured maps to FAULT with Detail "unconfigured". The Zero has no
//     such state. IDLE would tell a consumer the board is ready to act on
//     commands when it is not: an unconfigured board has no servo range or
//     speed scale and ignores them. It is normally seen only for the moment
//     between a board's Hello and its Config.
//   - FaultActuator on any state maps to FAULT, because the Zero reports a
//     failed actuator write as FAULT.
//   - An unknown state value maps to FAULT, naming the value.
//
// Detail lists the fault bits (see faultNames), but only when State is
// FAULT: MotorStatus documents Detail as empty otherwise. The other two bits
// do not make a healthy board FAULT. FaultRejectedCommand means a non-finite
// command was dropped and treated as missing, which the Zero logs but does
// not report as a fault; FaultWatchdogReset describes how the board last
// booted and stays set for the whole boot. Session logs both.
//
// DutyCycle and CommandAgeMs are carried as the board reports them.
// MeasuredSpeed stays zero, as on the Zero; wheel speed is JointStates'.
func MotorStatusFor(st boardlink.Status) *actuationv1.MotorStatus {
	var extra string
	state := actuationv1.MotorStatus_STATE_FAULT
	switch st.State {
	case boardlink.StateIdle:
		state = actuationv1.MotorStatus_STATE_IDLE
	case boardlink.StateRunning:
		state = actuationv1.MotorStatus_STATE_RUNNING
	case boardlink.StateFault:
	case boardlink.StateUnconfigured:
		extra = DetailUnconfigured
	default:
		extra = fmt.Sprintf("unknown board state %d", uint8(st.State))
	}
	if st.Faults&boardlink.FaultActuator != 0 {
		state = actuationv1.MotorStatus_STATE_FAULT
	}

	detail := ""
	if state == actuationv1.MotorStatus_STATE_FAULT {
		detail = joinDetail(extra, FaultString(st.Faults))
	}

	return &actuationv1.MotorStatus{
		Stamp:        timestamppb.Now(),
		FrameId:      FrameID,
		State:        state,
		Detail:       detail,
		DutyCycle:    st.Duty,
		CommandAgeMs: st.CommandAgeMS,
	}
}

// FaultString names the set bits of a boardlink fault byte, comma-separated,
// lowest bit first. Unknown bits are rendered in hex so a newer firmware's
// fault is visible rather than dropped. It is empty when faults is zero.
func FaultString(faults uint8) string {
	var parts []string
	known := uint8(0)
	for _, f := range faultNames {
		known |= f.bit
		if faults&f.bit != 0 {
			parts = append(parts, f.name)
		}
	}
	if rest := faults &^ known; rest != 0 {
		parts = append(parts, fmt.Sprintf("unknown(0x%02x)", rest))
	}
	return strings.Join(parts, ",")
}

// joinDetail joins the non-empty parts with "; ".
func joinDetail(parts ...string) string {
	var kept []string
	for _, p := range parts {
		if p != "" {
			kept = append(kept, p)
		}
	}
	return strings.Join(kept, "; ")
}

// jointStatesFor builds the JointStates the Zero's feedback loop publishes
// (internal/node/motor.JointStatesFor) for a wheel at revolutions turning at
// rpm: the drive joint only, position in rad, velocity in rad/s.
func jointStatesFor(revolutions, rpm float64) *actuationv1.JointStates {
	return &actuationv1.JointStates{
		Stamp:    timestamppb.Now(),
		FrameId:  FrameID,
		Name:     []string{actuationv1.DriveJoint},
		Position: []float64{revolutions * radiansPerRevolution},
		Velocity: []float64{rpm * radiansPerRevolution / secondsPerMinute},
	}
}
