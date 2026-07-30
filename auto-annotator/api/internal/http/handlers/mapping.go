package handlers

import (
	"path/filepath"

	annotationdomain "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/annotation"
	computedomain "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/compute"
	gallerydomain "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/gallery"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/domain"
)

// --- gallery mappings ---

func toGalleryResponse(g gallerydomain.Gallery) GalleryResponse {
	items := make([]GalleryItem, len(g.Items))
	for i, item := range g.Items {
		anns := annotationShapesToOAPI(item.Annotations)
		items[i] = GalleryItem{
			Id:          item.ID,
			Label:       filepath.Base(item.Path),
			Src:         item.ImageURL,
			ThumbSrc:    item.ThumbURL,
			Format:      item.Format,
			Status:      GalleryItemStatus(item.Status),
			Updated:     item.UpdatedAt,
			Annotations: &anns,
		}
	}
	return GalleryResponse{
		Items: items,
		Stats: GalleryStats{
			Pending: g.Stats.Pending,
			Done:    g.Stats.Done,
			Skipped: g.Stats.Skipped,
			Total:   g.Stats.Total,
			Pct:     g.Stats.Pct,
		},
	}
}

func toParentImageItems(rows []gallerydomain.GroupedImage) []ParentImageItem {
	items := make([]ParentImageItem, len(rows))
	for i, r := range rows {
		items[i] = ParentImageItem{
			Id:        r.ID,
			Label:     r.Label,
			Src:       r.ImageURL,
			Format:    r.FormatUsed,
			Status:    ParentImageItemStatus(domain.StatusName(r.Status)),
			UpdatedAt: r.UpdatedAt,
			AugCount:  r.AugCount,
		}
	}
	return items
}

// --- annotation mappings ---

func annotationShapesToOAPI(shapes []annotationdomain.Shape) []Shape {
	out := make([]Shape, len(shapes))
	for i, s := range shapes {
		pts := make([]Point, len(s.Points))
		for j, p := range s.Points {
			pts[j] = Point{X: coordToStr(p.X), Y: coordToStr(p.Y)}
		}
		out[i] = Shape{Id: s.ID, ClassName: s.ClassName, Points: pts}
	}
	return out
}

func classesToOAPI(classes []annotationdomain.Class) []ClassItem {
	items := make([]ClassItem, len(classes))
	for i, c := range classes {
		items[i] = ClassItem{Id: c.ID, Name: c.Name, Color: c.Color}
	}
	return items
}

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

// --- compute mappings ---

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

func toSegmentationResponse(r computedomain.SegmentResult) SegmentationResponse {
	shapes := make([]Shape, len(r.Shapes))
	for i, s := range r.Shapes {
		pts := make([]Point, len(s.Points))
		for j, p := range s.Points {
			pts[j] = Point{X: coordToStr(p.X), Y: coordToStr(p.Y)}
		}
		shapes[i] = Shape{Id: s.ID, ClassName: s.ClassName, Points: pts}
	}
	return SegmentationResponse{
		State:   SegmentationResponseState(r.State),
		Message: r.Message,
		Shapes:  shapes,
	}
}

func toModelsOAPI(models []computedomain.Model) []ModelItem {
	items := make([]ModelItem, len(models))
	for i, m := range models {
		items[i] = ModelItem{
			Id:           m.ID,
			Label:        m.Label,
			ModelType:    m.ModelType,
			Available:    m.Available,
			Active:       m.Active,
			SupportsText: m.SupportsText,
		}
	}
	return items
}
