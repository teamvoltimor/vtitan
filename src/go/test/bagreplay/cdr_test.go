package bagreplay_test

import (
	"encoding/binary"
	"errors"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/test/bagreplay"
)

// encodeStdMsgsString builds a std_msgs/msg/String exactly as rmw does:
// a four-byte encapsulation header, then a uint32 length that INCLUDES the
// terminating NUL, then the bytes and that NUL.
func encodeStdMsgsString(payload string, order binary.ByteOrder, representation byte) []byte {
	out := []byte{0x00, representation, 0x00, 0x00}
	length := make([]byte, 4)
	order.PutUint32(length, uint32(len(payload)+1))
	out = append(out, length...)
	out = append(out, payload...)
	return append(out, 0x00)
}

func TestDecodeStdMsgsString(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name           string
		payload        string
		order          binary.ByteOrder
		representation byte
	}{
		{name: "little endian", payload: `{"phase":"normal_drive"}`, order: binary.LittleEndian, representation: 0x01},
		{name: "big endian", payload: `{"phase":"normal_drive"}`, order: binary.BigEndian, representation: 0x00},
		{name: "empty payload still carries its NUL", payload: "", order: binary.LittleEndian, representation: 0x01},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			got, err := bagreplay.DecodeStdMsgsString(encodeStdMsgsString(tt.payload, tt.order, tt.representation))
			if err != nil {
				t.Fatalf("DecodeStdMsgsString() error = %v, want nil", err)
			}
			if got != tt.payload {
				t.Fatalf("DecodeStdMsgsString() = %q, want %q", got, tt.payload)
			}
		})
	}
}

// TestDecodeStdMsgsString_Rejects covers the malformed cases. These matter
// more than usual: a wrong-but-accepted decode would silently corrupt every
// downstream parity comparison rather than failing the replay.
func TestDecodeStdMsgsString_Rejects(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		data []byte
		want error
	}{
		{
			name: "shorter than the encapsulation header",
			data: []byte{0x00, 0x01},
			want: bagreplay.ErrShortMessage,
		},
		{
			name: "header only, no length",
			data: []byte{0x00, 0x01, 0x00, 0x00},
			want: bagreplay.ErrShortMessage,
		},
		{
			name: "length runs past the buffer",
			data: []byte{0x00, 0x01, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 'a'},
			want: bagreplay.ErrShortMessage,
		},
		{
			name: "zero length cannot include a NUL",
			data: []byte{0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			want: bagreplay.ErrShortMessage,
		},
		{
			name: "unknown representation identifier",
			data: []byte{0x00, 0x7f, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00},
			want: bagreplay.ErrBadEncapsulation,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			_, err := bagreplay.DecodeStdMsgsString(tt.data)
			if !errors.Is(err, tt.want) {
				t.Fatalf("DecodeStdMsgsString() error = %v, want %v", err, tt.want)
			}
		})
	}
}

// TestDecodeNavDebug_PreservesAbsentFields is the reason every nullable
// field is a pointer: Python writes null for "not computed on this branch",
// and a parity diff must be able to tell that apart from a measured zero.
func TestDecodeNavDebug_PreservesAbsentFields(t *testing.T) {
	t.Parallel()

	const payload = `{"phase":"no_pose","commanded_speed_mps":0.0,"crosstrack_error_m":null,` +
		`"laps_completed":2,"num_laps":3}`

	got, err := bagreplay.DecodeNavDebug(encodeStdMsgsString(payload, binary.LittleEndian, 0x01))
	if err != nil {
		t.Fatalf("DecodeNavDebug() error = %v, want nil", err)
	}

	if got.Phase != "no_pose" {
		t.Fatalf("Phase = %q, want %q", got.Phase, "no_pose")
	}
	if got.CrosstrackErrorM != nil {
		t.Fatalf("CrosstrackErrorM = %v, want nil for an absent field", *got.CrosstrackErrorM)
	}
	if got.CommandedSpeedMPS == nil {
		t.Fatal("CommandedSpeedMPS = nil, want a measured 0.0 to survive as a set pointer")
	}
	if *got.CommandedSpeedMPS != 0 {
		t.Fatalf("CommandedSpeedMPS = %v, want 0", *got.CommandedSpeedMPS)
	}
	if got.LapsCompleted != 2 || got.NumLaps != 3 {
		t.Fatalf("laps = %d/%d, want 2/3", got.LapsCompleted, got.NumLaps)
	}
}
