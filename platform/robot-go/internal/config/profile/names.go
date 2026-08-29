package profile

import (
	"os"
	"strings"
)

// EnvVar is the environment variable
// shared.config.hardware_profile.HardwareProfileSettings reads to select
// active per-component hardware profiles: comma-separated names, applied in
// order, later profiles overriding earlier ones.
const EnvVar = "VTITAN_HARDWARE_PROFILE"

// ParseNames splits raw (as read from EnvVar) into an ordered list of
// profile names, mirroring
// shared.config.hardware_profile.active_profiles(): comma-split, trim
// whitespace, drop empties.
func ParseNames(raw string) []string {
	var names []string
	for part := range strings.SplitSeq(raw, ",") {
		if name := strings.TrimSpace(part); name != "" {
			names = append(names, name)
		}
	}
	return names
}

// ActiveNames returns the profile names currently active per EnvVar.
func ActiveNames() []string {
	return ParseNames(os.Getenv(EnvVar))
}
