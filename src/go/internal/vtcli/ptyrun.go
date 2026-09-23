package vtcli

import (
	"context"
	"fmt"
	"os/exec"
	"time"

	tea "github.com/charmbracelet/bubbletea"
	"github.com/charmbracelet/x/xpty"
)

// taskRun is one task running on a pseudo-terminal for the run pane. A PTY
// rather than pipes because the tools behave as they do in a terminal only
// when they see one: pytest and pixi color their output, Python flushes
// line by line instead of in 4 KiB blocks, and progress bars redraw in place.
type taskRun struct {
	cmd     *exec.Cmd
	pty     xpty.Pty
	out     chan []byte
	done    chan runDoneMsg
	started time.Time
}

// runOutputMsg carries output the task wrote since the last one.
type runOutputMsg struct{ data []byte }

// runDoneMsg reports how the task ended: code is its exit status, err a
// failure to run it at all.
type runDoneMsg struct {
	code    int
	err     error
	elapsed time.Duration
}

// PTY plumbing sizes.
const (
	ptyReadSize = 32 * 1024
	// ptyDrainWait is how long output may keep arriving after the task
	// exits, for a grandchild still holding the terminal (a daemon it
	// started) before the PTY is closed under it.
	ptyDrainWait = 500 * time.Millisecond
	outBuffer    = 64
	ctrlC        = "\x03"
)

// startTaskRun starts `task <name> args...` from repoRoot on a new PTY of
// the given size.
func startTaskRun(ctx context.Context, repoRoot, name string, args []string, width, height int) (*taskRun, error) {
	term, err := xpty.NewPty(max(width, 1), max(height, 1))
	if err != nil {
		return nil, fmt.Errorf("open a pseudo-terminal: %w", err)
	}

	cmd := taskCommand(ctx, repoRoot, name, args)
	ownTerminal(cmd)

	if startErr := term.Start(cmd); startErr != nil {
		_ = term.Close()

		return nil, fmt.Errorf("start task %s: %w", name, startErr)
	}

	// Only the child keeps the slave end, so the master reads EOF when the
	// task and everything it started have exited.
	if unix, isUnix := term.(*xpty.UnixPty); isUnix {
		_ = unix.Slave().Close()
	}

	run := &taskRun{
		cmd: cmd, pty: term, started: time.Now(),
		out: make(chan []byte, outBuffer), done: make(chan runDoneMsg, 1),
	}

	readerDone := make(chan struct{})
	go run.read(readerDone)
	go run.wait(ctx, readerDone)

	return run, nil
}

// read forwards PTY output until the PTY closes.
func (r *taskRun) read(readerDone chan<- struct{}) {
	defer close(readerDone)
	defer close(r.out)

	buf := make([]byte, ptyReadSize)

	for {
		n, err := r.pty.Read(buf)
		if n > 0 {
			r.out <- append([]byte(nil), buf[:n]...)
		}

		if err != nil {
			return
		}
	}
}

// wait reaps the task, lets its last output drain, closes the PTY and
// reports the exit status.
func (r *taskRun) wait(ctx context.Context, readerDone <-chan struct{}) {
	err := xpty.WaitProcess(ctx, r.cmd)

	select {
	case <-readerDone:
	case <-time.After(ptyDrainWait):
	}

	_ = r.pty.Close()
	<-readerDone

	result := exitResult(err)
	result.elapsed = time.Since(r.started)
	r.done <- result
}

// next waits for the task's next output, or for its end once the output is
// exhausted. Everything already queued is sent as one message, so a chatty
// task costs one redraw per burst rather than one per read.
func (r *taskRun) next() tea.Cmd {
	return func() tea.Msg {
		chunk, open := <-r.out
		if !open {
			return <-r.done
		}

		for {
			select {
			case more, stillOpen := <-r.out:
				if !stillOpen {
					return runOutputMsg{data: chunk}
				}

				chunk = append(chunk, more...)
			default:
				return runOutputMsg{data: chunk}
			}
		}
	}
}

// input sends keystrokes to the task, as typing into its terminal would. A
// write can only fail once the task has exited and its PTY is closing, and
// then there is nothing left to type into: the done message is on its way.
func (r *taskRun) input(data string) {
	if _, err := r.pty.Write([]byte(data)); err != nil {
		return
	}
}

// interrupt is ctrl+c typed into the task's terminal: the terminal turns it
// into SIGINT for the task's process group, as it would outside vt.
func (r *taskRun) interrupt() {
	r.input(ctrlC)
}

// kill ends the task and everything it started, for a task that ignores the
// interrupt.
func (r *taskRun) kill() {
	killTree(r.cmd)
}

// resize tells the task its terminal changed size. Like input, it can only
// fail on a PTY that is closing because the task has ended.
func (r *taskRun) resize(width, height int) {
	if err := r.pty.Resize(max(width, 1), max(height, 1)); err != nil {
		return
	}
}

// taskCommand is `task <name> args...` from repoRoot, the one process shape
// every vt run has. name is checked against the inventory or the spec before
// it gets here, and nothing goes through a shell.
func taskCommand(ctx context.Context, repoRoot, name string, args []string) *exec.Cmd {
	argv := append([]string{name}, args...)

	cmd := exec.CommandContext(ctx, taskBinary, argv...)
	cmd.Dir = repoRoot

	return cmd
}
