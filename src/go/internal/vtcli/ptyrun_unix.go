//go:build !windows

package vtcli

import (
	"os/exec"
	"syscall"
)

// ownTerminal starts the task in its own session with the PTY as its
// controlling terminal, so ctrl+c typed into the PTY reaches the task's
// process group and not vt's.
func ownTerminal(cmd *exec.Cmd) {
	cmd.SysProcAttr = &syscall.SysProcAttr{Setsid: true, Setctty: true}
}

// killTree kills the task's whole process group; the task is its leader.
// Failing means the group is already gone, which is what was asked for.
func killTree(cmd *exec.Cmd) {
	if cmd.Process == nil {
		return
	}

	if err := syscall.Kill(-cmd.Process.Pid, syscall.SIGKILL); err != nil {
		return
	}
}
