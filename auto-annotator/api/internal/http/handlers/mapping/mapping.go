// Package mapping converts domain types into API wire (JSON) response
// shapes. It is a leaf package: it depends only on the domain packages, not
// on the handlers package, so it defines its own response DTOs mirroring
// the OpenAPI-generated ones in handlers/openapi.gen.go (field names, order,
// and json tags kept in lockstep with api/openapi.yaml). This avoids an
// import cycle, since openapi.gen.go's types must stay in package handlers
// (it's generated code we don't restructure) while handlers needs to import
// this package to call its conversion functions.
//
// Request (wire -> domain) conversions are NOT here: they consume
// Gin-bound, openapi-generated request DTOs (e.g. SegmentationRequest),
// so they stay colocated with their handler in the handlers package.
package mapping

import (
	"path/filepath"
	"strconv"

	annotationdomain "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/annotation"
	computedomain "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/compute"
	gallerydomain "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/gallery"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/domain"
)

// GalleryResponse mirrors handlers.GalleryResponse.
type GalleryResponse struct {
	Items []GalleryItem `json:"items"`
	Stats GalleryStats  `json:"stats"`
}

// GalleryItem mirrors handlers.GalleryItem.
type GalleryItem struct {
	Annotations *[]Shape `json:"annotations,omitempty"`
	Format      string   `json:"format"`
	Id          int64    `json:"id"`
	Label       string   `json:"label"`
	Src         string   `json:"src"`
	Status      string   `json:"status"`
	ThumbSrc    string   `json:"thumbSrc"`
	Updated     string   `json:"updated"`
}

// GalleryStats mirrors handlers.GalleryStats.
type GalleryStats struct {
	Done    int     `json:"done"`
	Pct     float64 `json:"pct"`
	Pending int     `json:"pending"`
	Skipped int     `json:"skipped"`
	Total   int     `json:"total"`
}

// ParentImageItem mirrors handlers.ParentImageItem.
type ParentImageItem struct {
	AugCount  int64  `json:"aug_count"`
	Format    string `json:"format"`
	Id        int64  `json:"id"`
	Label     string `json:"label"`
	Src       string `json:"src"`
	Status    string `json:"status"`
	UpdatedAt string `json:"updated_at"`
}

// Shape mirrors handlers.Shape.
type Shape struct {
	ClassName string  `json:"className"`
	Id        string  `json:"id"`
	Points    []Point `json:"points"`
}

// Point mirrors handlers.Point.
type Point struct {
	X string `json:"x"`
	Y string `json:"y"`
}

// ClassItem mirrors handlers.ClassItem.
type ClassItem struct {
	Color string `json:"color"`
	Id    int64  `json:"id"`
	Name  string `json:"name"`
}

// SegmentationResponse mirrors handlers.SegmentationResponse.
type SegmentationResponse struct {
	Message string  `json:"message"`
	Shapes  []Shape `json:"shapes"`
	State   string  `json:"state"`
}

// ModelItem mirrors handlers.ModelItem.
type ModelItem struct {
	Active       bool   `json:"active"`
	Available    bool   `json:"available"`
	Id           string `json:"id"`
	Label        string `json:"label"`
	ModelType    string `json:"modelType"`
	SupportsText bool   `json:"supportsText"`
}

// coordToStr formats a float64 coordinate into a decimal string for the API
// wire format.
func coordToStr(f float64) string {
	return strconv.FormatFloat(f, 'f', -1, 64)
}

// ToGalleryResponse converts a gallery domain result into its wire response shape.
func ToGalleryResponse(g gallerydomain.Gallery) GalleryResponse {
	items := make([]GalleryItem, len(g.Items))
	for i, item := range g.Items {
		anns := AnnotationShapesToOAPI(item.Annotations)
		items[i] = GalleryItem{
			Id:          item.ID,
			Label:       filepath.Base(item.Path),
			Src:         item.ImageURL,
			ThumbSrc:    item.ThumbURL,
			Format:      item.Format,
			Status:      item.Status,
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

// ToParentImageItems converts grouped gallery rows into their wire response shape.
func ToParentImageItems(rows []gallerydomain.GroupedImage) []ParentImageItem {
	items := make([]ParentImageItem, len(rows))
	for i, r := range rows {
		items[i] = ParentImageItem{
			Id:        r.ID,
			Label:     r.Label,
			Src:       r.ImageURL,
			Format:    r.FormatUsed,
			Status:    domain.StatusName(r.Status),
			UpdatedAt: r.UpdatedAt,
			AugCount:  r.AugCount,
		}
	}
	return items
}

// AnnotationShapesToOAPI converts annotation domain shapes into their wire response shape.
func AnnotationShapesToOAPI(shapes []annotationdomain.Shape) []Shape {
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

// ClassesToOAPI converts annotation domain classes into their wire response shape.
func ClassesToOAPI(classes []annotationdomain.Class) []ClassItem {
	items := make([]ClassItem, len(classes))
	for i, c := range classes {
		items[i] = ClassItem{Id: c.ID, Name: c.Name, Color: c.Color}
	}
	return items
}

// ToSegmentationResponse converts a compute domain segmentation result into its wire response shape.
func ToSegmentationResponse(r computedomain.SegmentResult) SegmentationResponse {
	shapes := make([]Shape, len(r.Shapes))
	for i, s := range r.Shapes {
		pts := make([]Point, len(s.Points))
		for j, p := range s.Points {
			pts[j] = Point{X: coordToStr(p.X), Y: coordToStr(p.Y)}
		}
		shapes[i] = Shape{Id: s.ID, ClassName: s.ClassName, Points: pts}
	}
	return SegmentationResponse{
		State:   r.State,
		Message: r.Message,
		Shapes:  shapes,
	}
}

// ToModelsOAPI converts compute domain models into their wire response shape.
func ToModelsOAPI(models []computedomain.Model) []ModelItem {
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
