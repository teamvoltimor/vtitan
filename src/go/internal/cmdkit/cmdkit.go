// Package cmdkit holds the CLI plumbing the cmd/* binaries share: the
// NATS/identity/config flag fields they all declare and the registration
// helpers that keep each flag's name, default, and standard wording in one
// place. Embed Common in a binary's cliConfig; only that binary's extras
// stay local.
package cmdkit

import "github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"

// Common holds the flag fields repeated by the cmd/* binaries. Embed it in a
// binary's cliConfig so the shared fields are declared once.
type Common struct {
	NATSURL    string
	NodeName   string
	Profiles   string
	ConfigRoot string
	RunsRoot   string
}

// stringFlagSet is the part of *pflag.FlagSet (and *flag.FlagSet) the base
// needs. Keeping it an interface lets cmdkit register into either flag
// package without importing both.
type stringFlagSet interface {
	StringVar(p *string, name, value, usage string)
}

// Wording shared by every binary that registers the matching flag. The
// config-root and runs-root text differs per binary (what an empty value
// falls back to varies), so those helpers take help from the caller.
const (
	helpNATSURL  = "nats-server URL"
	helpNodeName = "NATS client name, visible in nats-server's connz output"
	helpProfiles = "comma-separated hardware profiles (overrides VTITAN_HARDWARE_PROFILE)"
)

// RegisterNATSURL registers --nats-url, defaulting to nats.DefaultURL (the
// VTITAN_NATS_URL value if the deployment set it, else the localhost bench
// address).
func (c *Common) RegisterNATSURL(flags stringFlagSet) {
	flags.StringVar(&c.NATSURL, "nats-url", nats.DefaultURL(), helpNATSURL)
}

// RegisterNodeName registers --name with nodeName as the default.
func (c *Common) RegisterNodeName(flags stringFlagSet, nodeName string) {
	flags.StringVar(&c.NodeName, "name", nodeName, helpNodeName)
}

// RegisterProfiles registers --profiles.
func (c *Common) RegisterProfiles(flags stringFlagSet) {
	flags.StringVar(&c.Profiles, "profiles", "", helpProfiles)
}

// RegisterConfigRoot registers --config-root; help is the caller's because
// what an empty value falls back to is binary-specific.
func (c *Common) RegisterConfigRoot(flags stringFlagSet, help string) {
	flags.StringVar(&c.ConfigRoot, "config-root", "", help)
}

// RegisterRunsRoot registers --runs-root; help is the caller's because only
// some binaries gate it behind --record.
func (c *Common) RegisterRunsRoot(flags stringFlagSet, help string) {
	flags.StringVar(&c.RunsRoot, "runs-root", "", help)
}
