// Package ssd1306 provides a low-level I2C driver for the SSD1306 OLED
// display, plus a minimal Framebuffer/text-rendering primitive to draw into
// before writing it out.
//
// Scope, deliberately narrow for this pass: this package ports only
// platform/robot/src/hardware/display/ssd1306/driver_raw_i2c.py — the
// raw-ioctl I2C backend (init sequence, page-addressed framebuffer write,
// command/data control-byte protocol). It does NOT port
// platform/robot/ros2_ws/src/vtitan_drivers/vtitan_drivers/oled_display_node.py's
// page-orchestration logic (which screen renders for /robot_state,
// /race_metrics, /ui/telemetry_summary, /button/hold, /system_status, and
// the button-driven/state-driven switching between them). That logic
// depends on internal/transport/nats (still a doc.go stub) and the
// telemetry/state wire message types, which are a separate, larger effort
// tightly coupled to the telemetry-bridge port happening in parallel. Out
// of scope here, not silently skipped.
//
// Like internal/driver/motor, a display has nothing to "Read" in the sense
// driver.Driver[T] (platform/robot-go/internal/driver) models — it is
// written to, not sampled — so this package exposes its own narrow
// Actuator interface (Connect/WriteFramebuffer/Clear/Close) instead of
// implementing driver.Driver[T]. See platform/robot-go/internal/driver/imu
// (rvc_driver.go) for the Config/New/Connect/Close shape this package
// follows, and internal/driver/motor for the actuator-shaped-interface
// precedent.
//
// Package layout mirrors internal/driver/motor: a pure, hardware-independent
// logic layer (framebuffer.go, text.go, commands.go — unit-testable with a
// fake i2cWriter, no real I2C bus) and a thin hardware layer (driver.go's
// Connect/Close) that wires periph.io/x/conn/v3/i2c access behind the same
// small i2cWriter interface the logic layer already writes through.
package ssd1306
