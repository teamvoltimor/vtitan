// Package bno085rvc decodes the BNO08x (BNO085) UART-RVC stream: the
// 19-byte frame layout, its checksum, the sync-byte resync scan (ReadFrame)
// and the Euler-to-quaternion conversion RVC mode needs because it never
// reports a quaternion itself.
//
// It opens no port. The Linux serial driver (pkg/driver/imu, go.bug.st/
// serial) and the Pico 2 firmware (TinyGo machine.UART) each hand it a byte
// source, so both boards decode the same frame into the same Reading.
//
// Like every package under pkg/portable it must compile under TinyGo: no
// os, net, syscall, reflect, unsafe, log/slog, Linux or transport deps, and
// nothing from src/go/internal. See
// adr:0098-pico-actuation-board-and-portable-cores.
package bno085rvc
