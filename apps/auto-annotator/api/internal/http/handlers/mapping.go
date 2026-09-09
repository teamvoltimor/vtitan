package handlers

import (
	annotationdomain "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/annotation"
	computedomain "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/compute"
)

// toAnnotationSaveReq and its helpers below convert Gin-bound, openapi-generated
// request DTOs into domain types. They stay in package handlers (rather than
// moving to the mapping sub-package) because their input types (e.g.
// SaveAnnotationsRequest, SegmentationRequest, Point) are defined in
// openapi.gen.go, which lives in this package; the mapping sub-package holds
// only the domain -> wire (response) direction to avoid an import cycle.

func toAnnotationSaveReq(req SaveAnnotationsRequest) annotationdomain.SaveRequest {
	shapes := make([]annotationdomain.Shape, len(req.Shapes))
	for i, s := range req.Shapes {
		shapes[i] = annotationdomain.Shape{
			ID:        s.Id,
			ClassName: s.ClassName,
			Points:    oapiPointsToAnnotation(s.Points),
		}
	}
	return annotationdomain.SaveRequest{
		ImageID:      req.ImageId,
		ExportFormat: string(req.ExportFormat),
		Shapes:       shapes,
	}
}

func oapiPointsToAnnotation(pts []Point) []annotationdomain.Point {
	out := make([]annotationdomain.Point, len(pts))
	for i, p := range pts {
		x, _ := coordFromStr(p.X)
		y, _ := coordFromStr(p.Y)
		out[i] = annotationdomain.Point{X: x, Y: y}
	}
	return out
}

func toSegmentReq(req SegmentationRequest) computedomain.SegmentReq {
	pts := make([]computedomain.ClickPoint, len(req.Points))
	for i, p := range req.Points {
		x, _ := coordFromStr(p.X)
		y, _ := coordFromStr(p.Y)
		pts[i] = computedomain.ClickPoint{
			X: x, Y: y,
			PointType: string(p.PointType),
			ClassName: p.ClassName,
		}
	}
	return computedomain.SegmentReq{ImageID: req.ImageId, Points: pts}
}
