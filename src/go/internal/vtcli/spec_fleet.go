package vtcli

// windowsOnly marks the direct-link and route tasks: PowerShell, netsh and
// `route -p` against the Windows adapter. Elsewhere they are hidden.
var windowsOnly = []string{"windows"}

// sshHostFlag is the SSH alias most fleet tasks reach a board through.
var sshHostFlag = Flag{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-5-direct", Usage: "SSH alias"}

// sshConfigFlags are shared by the two ~/.ssh/config tasks. The defaults are
// Task's placeholders, shown so the form makes clear what to replace.
var sshConfigFlags = []Flag{
	{Name: "user", Var: "SSH_USER", Default: "YOUR_USERNAME", Usage: "SSH user"},
	{Name: "key-path", Var: "SSH_KEY_PATH", Default: "~/.ssh/YOUR_KEY_FILE", Usage: "private key path"},
	{Name: "domain", Var: "SSH_DOMAIN", Default: "YOUR_DOMAIN", Usage: "Cloudflare tunnel domain"},
	{
		Name:    "zero-hostname",
		Var:     "SSH_ZERO_HOSTNAME",
		Default: "YOUR_PI_ZERO_HOSTNAME.local",
		Usage:   "Pi Zero mDNS name",
	},
}

// fleetSpec wraps the dev-machine board tasks: windows:* (named for where they
// were first written, most also run on Linux) and rpi:migrate-data, the one
// rpi:* task that runs from the dev machine. Daily use sits at the top; the
// one-time network and SSH setup under `fleet setup`.
var fleetSpec = []Command{
	{
		Path:  []string{"fleet", "ping"},
		Task:  "windows:ping",
		Short: "Ping a board",
		Args:  []Arg{{Name: argHost, Var: "PING_HOST", Required: true, Usage: "board address"}},
	},
	{
		Path:  []string{"fleet", "ssh"},
		Task:  "windows:ssh",
		Short: "Open an SSH session on a board",
		Args:  []Arg{{Name: argHost, Var: "SSH_HOST", Required: true, Usage: "SSH alias of the Pi"}},
		// A remote shell reads raw keys and redraws its own screen.
		Terminal: true,
	},
	{
		Path:  []string{"fleet", "run"},
		Task:  "windows:run",
		Short: "Run a command on a board over SSH",
		Args:  []Arg{{Name: argHost, Var: "SSH_HOST", Required: true, Usage: "SSH alias of the Pi"}},
		Flags: []Flag{{Name: "cmd", Var: "CMD", Required: true, Usage: "command to run"}},
	},
	{
		Path:  []string{"fleet", "set-wifi"},
		Short: "Set a board's WiFi credentials; the Zero is reached through the Pi 5",
		Args:  []Arg{boardArg("windows:set-wifi:pi5", "windows:set-wifi:zero")},
		Flags: []Flag{
			{Name: "ssid", Var: "SSID", Required: true, Usage: "network name"},
			{Name: "password", Var: "PASSWORD", Required: true, Secret: true, Usage: "network password"},
			sshHostFlag,
		},
	},
	{
		Path:  []string{"fleet", "audit"},
		Short: "Pull and run a board's provisioning audit",
		Args:  []Arg{boardArg("windows:audit:pi5", "windows:audit:zero")},
		Flags: []Flag{
			{Name: flagSSHHost, Var: "SSH_HOST", Usage: "SSH alias (default rpi-5-direct, zero: rpi-zero-local)"},
		},
	},
	{
		Path:  []string{"fleet", "provision"},
		Task:  "windows:provision:pi5",
		Short: "Provision the Pi 5 (Ansible runs on the Pi itself)",
		Heavy: true,
		Flags: []Flag{
			sshHostFlag,
			{Name: "ip", Var: "PI5_IP", Default: "192.168.251.2", Usage: "Pi 5 address"},
			{Name: "tags", Var: "TAGS", Usage: "Ansible tags"},
			{Name: "skip-tags", Var: "SKIP_TAGS", Usage: "Ansible tags to skip"},
		},
	},
	{
		Path:      []string{"fleet", "setup", "ethernet", "configure"},
		Task:      "windows:ethernet:configure",
		Short:     "Set the Windows Ethernet adapter for a direct Pi link (admin)",
		Platforms: windowsOnly,
		Args:      []Arg{{Name: "mode", Var: "MODE", Required: true, Usage: "static|remove|dhcp"}},
	},
	{
		Path:      []string{"fleet", "setup", "ethernet", "link"},
		Task:      "windows:ethernet:setup-link",
		Short:     "Direct Ethernet link: Windows IP + Pi IP (admin)",
		Platforms: windowsOnly,
		Flags:     []Flag{{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-5-remote", Usage: "SSH alias"}},
	},
	{
		Path:      []string{"fleet", "setup", "ethernet", "unlink"},
		Task:      "windows:ethernet:unlink",
		Short:     "Tear down the direct link, both ends back to DHCP (admin)",
		Platforms: windowsOnly,
		Flags:     []Flag{{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-5-remote", Usage: "SSH alias"}},
	},
	{
		Path:      []string{"fleet", "setup", "route", "add"},
		Task:      "windows:route:add",
		Short:     "Add the WiFi and Ethernet persistent routes (admin)",
		Platforms: windowsOnly,
	},
	{
		Path:      []string{"fleet", "setup", "route", "delete"},
		Task:      "windows:route:delete",
		Short:     "Remove a persistent route (admin)",
		Platforms: windowsOnly,
		Flags:     []Flag{{Name: "subnet", Var: "SUBNET", Default: "192.168.251.0", Usage: "route subnet"}},
	},
	{
		Path:  []string{"fleet", "setup", "ssh-config", "print"},
		Task:  "windows:ssh:print-config",
		Short: "Print the recommended ~/.ssh/config entries",
		Flags: sshConfigFlags,
	},
	{
		Path:      []string{"fleet", "setup", "ssh-config", "write"},
		Task:      "windows:ssh:setup-config",
		Short:     "Append the missing host entries to ~/.ssh/config",
		Platforms: windowsOnly,
		Flags:     sshConfigFlags,
	},
	{
		Path:  []string{"fleet", "setup", "windscribe-deb"},
		Task:  "windows:stage-windscribe-deb",
		Short: "Stage the windscribe-cli arm64 .deb on the Pi 5 for Ansible",
		Args:  []Arg{{Name: "deb", Var: "DEB", Required: true, Usage: "local path of the downloaded .deb"}},
		Flags: []Flag{sshHostFlag},
	},
	{
		Path:  []string{"fleet", "setup", "migrate-data"},
		Task:  "rpi:migrate-data",
		Short: "One-off: move a pre-relayout ~/vtitan/data tree on a board",
		Flags: []Flag{sshHostFlag},
	},
	{
		Path:  []string{"fleet", "vpn"},
		Task:  "vpn",
		Short: "Windscribe VPN on this machine",
		Args:  []Arg{{Name: "action", Var: "ACTION", Required: true, Usage: "connect|disconnect|status"}},
	},
	{
		Path:  []string{"fleet", "cloudflare"},
		Task:  "cloudflare",
		Short: "Cloudflare tunnel service on this machine",
		Args:  []Arg{{Name: "action", Var: "ACTION", Required: true, Usage: "status|restart|logs"}},
	},
}

// boardArg picks the pi5 or zero variant of a fleet task.
func boardArg(pi5Task, zeroTask string) Arg {
	return Arg{Name: "board", Required: true, Tasks: map[string]string{"pi5": pi5Task, "zero": zeroTask}}
}
