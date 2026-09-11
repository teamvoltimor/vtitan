// Package uiv1_test proves the protovalidate constraints declared in
// proto/vtitan/ui/v1/*.proto are enforced at runtime, not just present as
// unused schema annotations.
package uiv1_test

import (
	"testing"

	"buf.build/go/protovalidate"
	"google.golang.org/protobuf/proto"
	"google.golang.org/protobuf/types/known/timestamppb"

	uiv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/ui/v1"
)

func TestUIValidation(t *testing.T) {
	t.Parallel()

	validator, err := protovalidate.New()
	if err != nil {
		t.Fatalf("protovalidate.New: %v", err)
	}

	tests := map[string]struct {
		msg     proto.Message
		wantErr bool
	}{
		// A producer with no configured thresholds still reports held_s, and
		// the OLED must not blank on a partial frame.
		"button hold with no thresholds passes": {
			msg:     &uiv1.ButtonHold{Stamp: timestamppb.Now(), HeldS: 1.5},
			wantErr: false,
		},
		"negative hold duration fails": {
			msg:     &uiv1.ButtonHold{Stamp: timestamppb.Now(), HeldS: -1},
			wantErr: true,
		},
		"threshold without a kind fails": {
			msg: &uiv1.ButtonHold{
				Stamp:      timestamppb.Now(),
				Thresholds: []*uiv1.ButtonHold_Threshold{{AtS: 3.0}},
			},
			wantErr: true,
		},
		"jumper reading passes": {
			msg:     &uiv1.JumperInserted{Stamp: timestamppb.Now(), Inserted: true},
			wantErr: false,
		},
		"challenge mode passes": {
			msg: &uiv1.ChallengeModeActive{
				Stamp:     timestamppb.Now(),
				Challenge: uiv1.Challenge_CHALLENGE_OBSTACLES,
			},
			wantErr: false,
		},
		"challenge mode without stamp fails": {
			msg:     &uiv1.ChallengeModeActive{Challenge: uiv1.Challenge_CHALLENGE_OPEN},
			wantErr: true,
		},
	}

	for name, tt := range tests {
		t.Run(name, func(t *testing.T) {
			t.Parallel()

			validateErr := validator.Validate(tt.msg)
			if (validateErr != nil) != tt.wantErr {
				t.Fatalf("Validate() error = %v, wantErr %v", validateErr, tt.wantErr)
			}
		})
	}
}
