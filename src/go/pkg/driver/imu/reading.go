package imu

import "github.com/teamvoltimor/vtitan/src/go/pkg/portable/bno085rvc"

// Reading is one decoded BNO08x UART-RVC heading sample; an alias so imu's
// callers keep their spelling after the frame parser moved to pkg/portable.
type Reading = bno085rvc.Reading
