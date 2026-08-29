// Package button converts internal/driver/button.Kind into its
// vtitan.ui.v1.ButtonEvent wire counterpart -- shared by cmd/pi-zero's real
// button-driver loop and internal/statemachine/robotcmd's synthetic
// button-event sink (a backend start/stop/e-stop command publishes the
// exact same wire event a physical button press would).
package button
