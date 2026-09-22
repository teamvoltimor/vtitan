// Package quadrature is the pure half of the drive-shaft quadrature
// encoder: the A/B decode state machine (Decoder), the count->revolutions->
// distance conversions, the windowed, smoothed RPM estimator
// (SpeedEstimator) and the Odometry sample they produce. It ports
// src/python/src/hardware/motors/encoder/control.py.
//
// It reads no pins. The Pi Zero's Linux driver (pkg/driver/encoder,
// go-gpiocdev edge events) and the Pico 2 firmware (TinyGo pin interrupts)
// each feed it channel levels, so both boards count, and report distance,
// identically.
//
// Like every package under pkg/portable it must compile under TinyGo: no
// os, net, syscall, reflect, unsafe, log/slog, Linux or transport deps, and
// nothing from src/go/internal. See
// adr:0098-pico-actuation-board-and-portable-cores.
package quadrature
