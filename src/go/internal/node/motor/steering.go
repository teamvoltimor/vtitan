package motor

import "github.com/teamvoltimor/vtitan/src/go/pkg/portable/actuation"

// Steerer is the one steering call the loop makes; *servo.Driver satisfies
// it. servoDeg is an absolute SERVO angle, 0 = center.
type Steerer interface {
	SetAngle(servoDeg float64) error
}

// Steering pairs the servo with the conversion that feeds it
// (actuation.SteeringConfig). The zero value steers nothing and is only for
// cmd/motor-node, the drive-only bench binary; every command is still
// validated, steering_angle included.
type Steering struct {
	Servo  Steerer
	Config actuation.SteeringConfig
}
