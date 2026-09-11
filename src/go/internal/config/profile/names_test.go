package profile_test

import (
	"reflect"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

func TestParseNames(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		raw  string
		want []string
	}{
		{name: "empty", raw: "", want: nil},
		{name: "single", raw: "270deg-hiwonder-35kg", want: []string{"270deg-hiwonder-35kg"}},
		{
			name: "multiple, ordered, trimmed",
			raw:  "270deg-hiwonder-35kg, rev-hd-hex-motor-6000rpm",
			want: []string{"270deg-hiwonder-35kg", "rev-hd-hex-motor-6000rpm"},
		},
		{name: "drops empties", raw: "a,,b,", want: []string{"a", "b"}},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			if got := profile.ParseNames(tt.raw); !reflect.DeepEqual(got, tt.want) {
				t.Errorf("ParseNames(%q) = %v, want %v", tt.raw, got, tt.want)
			}
		})
	}
}

func TestActiveNames(t *testing.T) {
	t.Setenv(profile.EnvVar, "a, b")

	got := profile.ActiveNames()
	want := []string{"a", "b"}
	if !reflect.DeepEqual(got, want) {
		t.Errorf("ActiveNames() = %v, want %v", got, want)
	}
}
