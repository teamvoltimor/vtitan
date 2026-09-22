package vtcli

// FlagKind is the value type a typed flag carries into its Task variable.
type FlagKind string

// Flag maps one CLI flag to one Task variable. A flag is only forwarded when
// the user sets it, so Task's own default still applies otherwise.
type Flag struct {
	Name     string
	Var      string
	Kind     FlagKind
	Default  string
	Usage    string
	Required bool
}

// Arg maps one positional CLI argument to one Task variable.
type Arg struct {
	Name     string
	Var      string
	Usage    string
	Required bool
}

// Command is one wrapped leaf: a path in the CLI tree that runs one Task.
type Command struct {
	Path        []string
	Task        string
	Short       string
	Flags       []Flag
	Args        []Arg
	Passthrough bool
	// Heavy marks a task that starts long-running processes (a simulator, a
	// service): the interactive form warns before running it.
	Heavy bool
}

// Domain groups a first-level CLI namespace and the Task-name prefix it owns.
// Only curated domains are checked by the anti-drift test.
type Domain struct {
	ID         string
	Title      string
	TaskPrefix string
}

const (
	// FlagString forwards the flag value verbatim.
	FlagString FlagKind = "string"
	// FlagInt forwards a base-10 integer.
	FlagInt FlagKind = "int"
	// FlagBool forwards "true" or "false".
	FlagBool FlagKind = "bool"
)

// Flag names and positional names reused across the table.
const (
	flagOut     = "out"
	flagPkg     = "pkg"
	flagSSHHost = "ssh-host"
	argHost     = "host"

	// reasonPhase1 marks tasks deliberately deferred to phase 1.
	reasonPhase1 = "fase 1"
)

// curatedDomains are the domains under the anti-drift contract. Everything
// matching their TaskPrefix must be either in the spec or excluded by name.
// Domains such as sim/rpi are present in the spec already but are not promoted
// to curated until phase 1 closes their exclusion lists.
var curatedDomains = []Domain{
	{ID: "go", Title: "go — módulo Go (robot)", TaskPrefix: "go:"},
	{ID: "fleet", Title: "fleet — placas por SSH", TaskPrefix: "windows:"},
}

// exclusions records, per curated domain, the tasks deliberately left out of
// the typed tree and why. Adding a task to a curated domain without deciding
// anything here fails the build (see spec_test.go).
var exclusions = map[string]string{
	"go:todo":                      "volcado informativo de la lista de migración; no es un flujo",
	"go:hw:stop-notes":             "helper interno de go:hw:stop, no es un comando de usuario",
	"windows:ethernet:configure":   reasonPhase1,
	"windows:ethernet:setup-link":  reasonPhase1,
	"windows:ethernet:unlink":      reasonPhase1,
	"windows:route:add":            reasonPhase1,
	"windows:route:delete":         reasonPhase1,
	"windows:ssh:print-config":     reasonPhase1,
	"windows:ssh:setup-config":     reasonPhase1,
	"windows:stage-windscribe-deb": reasonPhase1 + " (requiere la ruta local del .deb)",
}

// curatedSpec is the single declarative table the tree is built from. One
// entry per wrapped command; nothing here duplicates how a task runs.
var curatedSpec = []Command{
	// sim (spec-only for now; promoted to curated in phase 1).
	{
		Path:        []string{"sim", "navigate", "visualize", "all"},
		Task:        "sim:navigate:visualize:all",
		Short:       "RViz + escenarios en un solo comando",
		Passthrough: true,
		Heavy:       true,
	},
	{
		Path:  []string{"sim", "gazebo"},
		Task:  "sim:gazebo",
		Short: "Lanza Gazebo con un mundo SDF",
		Heavy: true,
		Flags: []Flag{{Name: "sdf", Var: "SDF", Usage: "ruta al mundo .sdf"}},
	},
	{Path: []string{"sim", "test"}, Task: "sim:test", Short: "Tests del simulador", Heavy: true},
	{Path: []string{"sim", "lint"}, Task: "sim:lint", Short: "Lint del simulador"},

	// go (curated).
	{
		Path:  []string{"go", "build", "static"},
		Task:  "go:build:static",
		Short: "Compila los binarios del robot (linux/arm64, CGO off)",
		Flags: []Flag{{Name: flagOut, Var: "OUT", Usage: "directorio de salida"}},
	},
	{
		Path:  []string{"go", "build", "capture"},
		Task:  "go:build:capture",
		Short: "Compila los binarios con gocv/OpenCV (CGO on)",
		Flags: []Flag{
			{Name: flagOut, Var: "OUT", Usage: "directorio de salida"},
			{Name: "cc", Var: "CC", Usage: "cross-compilador C"},
		},
	},
	{
		Path:  []string{"go", "deploy"},
		Task:  "go:deploy",
		Short: "Compila y despliega los binarios en un Pi (/opt/vtitan-go)",
		Flags: []Flag{
			{Name: "target-host", Var: "TARGET_HOST", Usage: "user@host de destino"},
			{Name: "install-dir", Var: "INSTALL_DIR", Usage: "directorio de instalación"},
			{Name: "skip-restart", Var: "SKIP_RESTART", Usage: "no reiniciar servicios"},
		},
	},
	{Path: []string{"go", "test"}, Task: "go:test", Short: "Tests del módulo Go"},
	{
		Path:  []string{"go", "test", "hw"},
		Task:  "go:test:hw",
		Short: "Cross-compila los tests de hardware (linux/arm64)",
		Flags: []Flag{
			{Name: flagPkg, Var: "PKG", Usage: "button|ssd1306|imu|lidar|motor|nats"},
			{Name: flagOut, Var: "OUT", Usage: "directorio de salida"},
		},
	},
	{
		Path:  []string{"go", "test", "hw", "interactive"},
		Task:  "go:test:hw:interactive",
		Short: "Cross-compila los tests de hardware interactivos",
		Flags: []Flag{
			{Name: flagPkg, Var: "PKG", Usage: "motor|imu|lidar|ssd1306"},
			{Name: flagOut, Var: "OUT", Usage: "directorio de salida"},
		},
	},
	{
		Path:  []string{"go", "hw", "run"},
		Task:  "go:hw:run",
		Short: "Compila, envía y ejecuta un test interactivo en un Pi",
		Flags: []Flag{
			{Name: flagPkg, Var: "PKG", Usage: "motor|imu|lidar|ssd1306"},
			{Name: "host", Var: "HOST", Usage: "alias SSH del Pi"},
			{Name: "yaw-offset", Var: "YAW_OFFSET", Usage: "desviación de montaje del lidar"},
			{Name: "inverted", Var: "INVERTED", Kind: FlagBool, Usage: "montaje del lidar invertido"},
			{Name: "scan-mode", Var: "SCAN_MODE", Usage: "modo de escaneo del lidar"},
			{Name: "dump-scan", Var: "DUMP_SCAN", Usage: "volcar cada punto válido"},
			{Name: flagOut, Var: "OUT", Usage: "directorio de salida"},
		},
	},
	{
		Path:  []string{"go", "hw", "stop"},
		Task:  "go:hw:stop",
		Short: "Para los servicios del robot para liberar puertos del Pi",
		Args:  []Arg{{Name: argHost, Var: "HOST", Required: true, Usage: "alias SSH del Pi"}},
	},

	// fleet (curated; task prefix windows:).
	{
		Path:  []string{"fleet", "ping"},
		Task:  "windows:ping",
		Short: "Hace ping a un Pi",
		Args:  []Arg{{Name: argHost, Var: "PING_HOST", Required: true, Usage: "dirección del Pi"}},
	},
	{
		Path:  []string{"fleet", "ssh"},
		Task:  "windows:ssh",
		Short: "Entra por SSH en un Pi",
		Args:  []Arg{{Name: argHost, Var: "SSH_HOST", Required: true, Usage: "alias SSH del Pi"}},
	},
	{
		Path:  []string{"fleet", "run"},
		Task:  "windows:run",
		Short: "Ejecuta un comando en un Pi por SSH",
		Args:  []Arg{{Name: argHost, Var: "SSH_HOST", Required: true, Usage: "alias SSH del Pi"}},
		Flags: []Flag{{Name: "cmd", Var: "CMD", Required: true, Usage: "comando a ejecutar"}},
	},
	{
		Path:  []string{"fleet", "set-wifi", "pi5"},
		Task:  "windows:set-wifi:pi5",
		Short: "Configura el WiFi del Pi 5 desde Windows",
		Flags: []Flag{
			{Name: "ssid", Var: "SSID", Required: true, Usage: "nombre de la red"},
			{Name: "password", Var: "PASSWORD", Required: true, Usage: "contraseña de la red"},
			{Name: flagSSHHost, Var: "SSH_HOST", Usage: "alias SSH (default rpi-5-direct)"},
		},
	},
	{
		Path:  []string{"fleet", "set-wifi", "zero"},
		Task:  "windows:set-wifi:zero",
		Short: "Configura el WiFi del Pi Zero saltando por el Pi 5",
		Flags: []Flag{
			{Name: "ssid", Var: "SSID", Required: true, Usage: "nombre de la red"},
			{Name: "password", Var: "PASSWORD", Required: true, Usage: "contraseña de la red"},
			{Name: flagSSHHost, Var: "SSH_HOST", Usage: "alias SSH (default rpi-5-direct)"},
		},
	},
	{
		Path:  []string{"fleet", "provision", "pi5"},
		Task:  "windows:provision:pi5",
		Short: "Lanza el aprovisionamiento del Pi 5 por SSH",
		Flags: []Flag{
			{Name: flagSSHHost, Var: "SSH_HOST", Usage: "alias SSH (default rpi-5-direct)"},
			{Name: "ip", Var: "PI5_IP", Usage: "IP del Pi 5"},
			{Name: "tags", Var: "TAGS", Usage: "tags de Ansible"},
			{Name: "skip-tags", Var: "SKIP_TAGS", Usage: "tags de Ansible a omitir"},
		},
	},
	{
		Path:  []string{"fleet", "audit", "pi5"},
		Task:  "windows:audit:pi5",
		Short: "Audita el aprovisionamiento del Pi 5",
		Flags: []Flag{{Name: flagSSHHost, Var: "SSH_HOST", Usage: "alias SSH"}},
	},
	{
		Path:  []string{"fleet", "audit", "zero"},
		Task:  "windows:audit:zero",
		Short: "Audita el aprovisionamiento del Pi Zero",
		Flags: []Flag{{Name: flagSSHHost, Var: "SSH_HOST", Usage: "alias SSH"}},
	},
}

// CuratedSpec returns the declarative command table.
func CuratedSpec() []Command {
	return curatedSpec
}

// CuratedDomains returns the domains under the anti-drift contract.
func CuratedDomains() []Domain {
	return curatedDomains
}

// Exclusions returns the explicit per-task exclusions for curated domains.
func Exclusions() map[string]string {
	return exclusions
}
