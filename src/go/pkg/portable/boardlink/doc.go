// Package boardlink is the wire protocol between the Pi 5 and a
// microcontroller actuation board (the Pico 2, adr:0098-pico-actuation-board-and-portable-cores).
// Both ends compile it: the firmware under TinyGo, the Pi 5's picolink target
// under Go.
//
// A frame on the wire is
//
//	COBS( version:u8 | type:u8 | seq:u16 | body | crc16:u16 ) 0x00
//
// little-endian throughout. COBS removes every zero byte from the encoded
// frame, so 0x00 is an unambiguous delimiter and a receiver that starts
// mid-stream, or loses bytes, resynchronises at the next zero. The CRC is
// CRC-16/CCITT-FALSE over everything before it; a frame that fails it is
// dropped, never half-applied.
//
// The session is driven by the board. Until it is configured it repeats
// Hello; the host answers every Hello with Config, so a board that resets
// mid-round is reconfigured by the same path as one that just booted, and
// BootID tells the host a reset happened. Nothing on the board is tuned at
// compile time: the servo pulse range, linkage ratio, speed scale and command
// timeout all come from the host's hardware profile, as they do on the Zero.
//
// Ping/Pong carries the host's clock out and the board's back, which is the
// cross-board clock offset go-future.md section 4.6 asks for.
//
// The decoder keeps no heap state and allocates nothing per frame: at 50 Hz
// on a microcontroller, per-frame garbage is a GC pause waiting to happen.
//
// Files named *_std.go are built only by regular Go (//go:build !tinygo) and
// may use what the firmware cannot, such as log/slog. Nothing in the core may
// depend on them.
package boardlink
