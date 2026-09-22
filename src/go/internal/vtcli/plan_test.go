package vtcli

import (
	"errors"
	"strings"
	"testing"
)

// planFixture is a command using every feature planInvocation resolves: a
// task-picking argument, a variant switch, flags with and without defaults, a
// secret and a passthrough.
var planFixture = Command{
	Path:        []string{"fleet", "demo"},
	Task:        "",
	Passthrough: true,
	Args: []Arg{
		{Name: "board", Required: true, Tasks: map[string]string{"pi5": "demo:pi5", "zero": "demo:zero"}},
	},
	Variants: []Variant{{Flag: "dry", Task: "demo:dry"}, {Flag: "fast", Task: "demo:fast"}},
	Flags: []Flag{
		{Name: "host", Var: "HOST", Default: "rpi-5-local"},
		{Name: "password", Var: "PASSWORD", Secret: true},
		{Name: "inverted", Var: "INVERTED", Kind: FlagBool, Default: "true"},
	},
}

// TestPlanInvocation checks what one set of values resolves to, and that the
// preview, the argv and the echoed vt line all agree on it.
func TestPlanInvocation(t *testing.T) {
	t.Parallel()

	values := map[string]string{
		"board": "zero", "host": "rpi-5-local", "password": "it's secret", "inverted": "false",
	}

	inv, err := planInvocation(planFixture, values, []string{"--scenario", "5"})
	if err != nil {
		t.Fatal(err)
	}

	if inv.task != "demo:zero" {
		t.Errorf("task = %q, want the board's task demo:zero", inv.task)
	}

	wantArgv := "PASSWORD=it's secret INVERTED=false -- --scenario 5"
	if got := strings.Join(inv.argv(), " "); got != wantArgv {
		t.Errorf("argv = %q, want %q (the default host is not forwarded)", got, wantArgv)
	}

	wantDisplay := "task demo:zero PASSWORD='***' INVERTED=false -- --scenario 5"
	if got := inv.display(); got != wantDisplay {
		t.Errorf("display = %q, want %q", got, wantDisplay)
	}

	wantVT := "vt fleet demo zero --password '***' --inverted=false -- --scenario 5"
	if got := vtLine(planFixture, values, []string{"--scenario", "5"}); got != wantVT {
		t.Errorf("vt line = %q, want %q", got, wantVT)
	}
}

// TestPlanInvocationRejects checks the three ways values cannot resolve.
func TestPlanInvocationRejects(t *testing.T) {
	t.Parallel()

	cases := map[string]map[string]string{
		"missing board":     {},
		"unknown board":     {"board": "pico"},
		"exclusive variant": {"board": "pi5", "dry": "true", "fast": "true"},
	}

	for name, values := range cases {
		if _, err := planInvocation(planFixture, values, nil); err == nil {
			t.Errorf("%s: resolved, want an error", name)
		}
	}

	if _, err := planInvocation(Command{}, nil, nil); !errors.Is(err, errNoTask) {
		t.Errorf("a command with no task resolved: %v", err)
	}

	variant, err := planInvocation(planFixture, map[string]string{"board": "pi5", "dry": "true"}, nil)
	if err != nil || variant.task != "demo:dry" {
		t.Errorf("variant switch: task %q, err %v; want demo:dry", variant.task, err)
	}
}
