package vtcli

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
// rpi:* task that runs from the dev machine.
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
	},
	{
		Path:  []string{"fleet", "run"},
		Task:  "windows:run",
		Short: "Run a command on a board over SSH",
		Args:  []Arg{{Name: argHost, Var: "SSH_HOST", Required: true, Usage: "SSH alias of the Pi"}},
		Flags: []Flag{{Name: "cmd", Var: "CMD", Required: true, Usage: "command to run"}},
	},
	{
		Path:  []string{"fleet", "set-wifi", "pi5"},
		Task:  "windows:set-wifi:pi5",
		Short: "Set the Pi 5 WiFi credentials from Windows",
		Flags: []Flag{
			{Name: "ssid", Var: "SSID", Required: true, Usage: "network name"},
			{Name: "password", Var: "PASSWORD", Required: true, Usage: "network password"},
			{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-5-direct", Usage: "SSH alias"},
		},
	},
	{
		Path:  []string{"fleet", "set-wifi", "zero"},
		Task:  "windows:set-wifi:zero",
		Short: "Set the Pi Zero WiFi credentials, hopping through the Pi 5",
		Flags: []Flag{
			{Name: "ssid", Var: "SSID", Required: true, Usage: "network name"},
			{Name: "password", Var: "PASSWORD", Required: true, Usage: "network password"},
			{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-5-direct", Usage: "SSH alias"},
		},
	},
	{
		Path:  []string{"fleet", "provision", "pi5"},
		Task:  "windows:provision:pi5",
		Short: "Kick off Pi 5 provisioning over SSH",
		Flags: []Flag{
			{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-5-direct", Usage: "SSH alias"},
			{Name: "ip", Var: "PI5_IP", Default: "192.168.251.2", Usage: "Pi 5 address"},
			{Name: "tags", Var: "TAGS", Usage: "Ansible tags"},
			{Name: "skip-tags", Var: "SKIP_TAGS", Usage: "Ansible tags to skip"},
		},
	},
	{
		Path:  []string{"fleet", "audit", "pi5"},
		Task:  "windows:audit:pi5",
		Short: "Audit the Pi 5 provisioning",
		Flags: []Flag{{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-5-direct", Usage: "SSH alias"}},
	},
	{
		Path:  []string{"fleet", "audit", "zero"},
		Task:  "windows:audit:zero",
		Short: "Audit the Pi Zero provisioning",
		Flags: []Flag{{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-zero-local", Usage: "SSH alias"}},
	},
	{
		Path:  []string{"fleet", "migrate-data"},
		Task:  "rpi:migrate-data",
		Short: "One-off: move a pre-relayout ~/vtitan/data tree on a board",
		Flags: []Flag{{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-5-direct", Usage: "SSH alias"}},
	},
	{
		Path:  []string{"fleet", "ethernet", "configure"},
		Task:  "windows:ethernet:configure",
		Short: "Set the Windows Ethernet adapter for a direct Pi link (Windows, admin)",
		Args:  []Arg{{Name: "mode", Var: "MODE", Required: true, Usage: "static|remove|dhcp"}},
	},
	{
		Path:  []string{"fleet", "ethernet", "setup-link"},
		Task:  "windows:ethernet:setup-link",
		Short: "Direct Ethernet link: Windows IP + Pi IP (Windows, admin)",
		Flags: []Flag{{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-5-remote", Usage: "SSH alias"}},
	},
	{
		Path:  []string{"fleet", "ethernet", "unlink"},
		Task:  "windows:ethernet:unlink",
		Short: "Tear down the direct link, both ends back to DHCP (Windows, admin)",
		Flags: []Flag{{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-5-remote", Usage: "SSH alias"}},
	},
	{
		Path:  []string{"fleet", "route", "add"},
		Task:  "windows:route:add",
		Short: "Add the WiFi and Ethernet persistent routes (Windows, admin)",
	},
	{
		Path:  []string{"fleet", "route", "delete"},
		Task:  "windows:route:delete",
		Short: "Remove a persistent route (Windows, admin)",
		Flags: []Flag{{Name: "subnet", Var: "SUBNET", Default: "192.168.251.0", Usage: "route subnet"}},
	},
	{
		Path:  []string{"fleet", "ssh-config", "print"},
		Task:  "windows:ssh:print-config",
		Short: "Print the recommended ~/.ssh/config entries",
		Flags: sshConfigFlags,
	},
	{
		Path:  []string{"fleet", "ssh-config", "setup"},
		Task:  "windows:ssh:setup-config",
		Short: "Append the missing host entries to ~/.ssh/config",
		Flags: sshConfigFlags,
	},
	{
		Path:  []string{"fleet", "stage-windscribe-deb"},
		Task:  "windows:stage-windscribe-deb",
		Short: "Stage the windscribe-cli arm64 .deb on the Pi 5 for Ansible",
		Args:  []Arg{{Name: "deb", Var: "DEB", Required: true, Usage: "local path of the downloaded .deb"}},
		Flags: []Flag{{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-5-direct", Usage: "SSH alias"}},
	},
}
