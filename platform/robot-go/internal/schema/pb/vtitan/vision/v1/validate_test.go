// Package visionv1_test proves the protovalidate constraints declared in
// proto/vtitan/vision/v1/detections.proto are enforced at runtime, not just
// present as unused schema annotations.
package visionv1_test

import (
	"testing"

	"buf.build/go/protovalidate"
	"google.golang.org/protobuf/types/known/timestamppb"

	visionv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/vision/v1"
)

func TestDetectionsValidation(t *testing.T) {
	t.Parallel()

	validator, err := protovalidate.New()
	if err != nil {
		t.Fatalf("protovalidate.New: %v", err)
	}

	tests := map[string]struct {
		msg     *visionv1.Detections
		wantErr bool
	}{
		// The camera sees no sign for most of a lap, so an empty list is a
		// measurement rather than an error and must validate.
		"empty detection list passes": {
			msg:     &visionv1.Detections{Stamp: timestamppb.Now(), FrameId: "camera"},
			wantErr: false,
		},
		"missing stamp fails required constraint": {
			msg:     &visionv1.Detections{FrameId: "camera"},
			wantErr: true,
		},
		"well-formed detection passes": {
			msg: &visionv1.Detections{
				Stamp: timestamppb.Now(),
				Detections: []*visionv1.Detection{{
					ClassName:  visionv1.SignColor_SIGN_COLOR_RED,
					Confidence: 0.82,
					Bbox:       &visionv1.BBox{XMin: 10, YMin: 20, XMax: 40, YMax: 60},
					Width:      30,
					Height:     40,
					Area:       1200,
				}},
			},
			wantErr: false,
		},
		"confidence above 1 fails": {
			msg: &visionv1.Detections{
				Stamp:      timestamppb.Now(),
				Detections: []*visionv1.Detection{{Confidence: 1.4}},
			},
			wantErr: true,
		},
		"negative area fails": {
			msg: &visionv1.Detections{
				Stamp:      timestamppb.Now(),
				Detections: []*visionv1.Detection{{Confidence: 0.5, Area: -1}},
			},
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
