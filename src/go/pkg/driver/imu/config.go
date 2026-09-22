package imu

// DefaultPort matches bno08x_uart_rvc.toml's default_port -- the fallback
// port used when MCP2221 VID/PID auto-detection isn't available (that
// auto-detection, find_mcp2221_port in the Python driver, isn't ported
// here) and no profile or flag overrides it.
const DefaultPort = "/dev/ttyACM0"
