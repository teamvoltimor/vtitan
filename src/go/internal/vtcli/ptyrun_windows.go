//go:build windows

package vtcli

import "os/exec"

// ownTerminal is a no-op: ConPTY already attaches the task to its console.
func ownTerminal(*exec.Cmd) {}

// killTree kills the task. Windows has no process groups to signal; the
// children of a killed task lose their console and exit with it.
func killTree(cmd *exec.Cmd) {
	if cmd.Process != nil {
		_ = cmd.Process.Kill()
	}
}
