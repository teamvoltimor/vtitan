// Package dto defines the HTTP request/response shapes. JSON tags use the
// camelCase wire format produced by the Python Pydantic models (to_camel),
// except where the originals pinned snake_case aliases (updated_at, aug_count).
package dto

// Point is a normalized 2-D coordinate.
type Point struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// Shape is a single mask/bounding-box annotation.
type Shape struct {
	ID        string  `json:"id"`
	ClassName string  `json:"className"`
	Points    []Point `json:"points"`
}

// GalleryStats holds aggregate image status counts.
type GalleryStats struct {
	Pending int     `json:"pending"`
	Done    int     `json:"done"`
	Skipped int     `json:"skipped"`
	Total   int     `json:"total"`
	Pct     float64 `json:"pct"`
}

// GalleryItem is one image entry in the gallery listing.
type GalleryItem struct {
	ID          int64   `json:"id"`
	Label       string  `json:"label"`
	Src         string  `json:"src"`
	Format      string  `json:"format"`
	Status      string  `json:"status"`
	Updated     string  `json:"updated"`
	Annotations []Shape `json:"annotations"`
}

// GalleryResponse is the full gallery payload.
type GalleryResponse struct {
	Items []GalleryItem `json:"items"`
	Stats GalleryStats  `json:"stats"`
}

// ParentImageItem is a parent (original) image with its augmentation count,
// returned by GET /gallery/grouped. updated_at and aug_count keep snake_case.
type ParentImageItem struct {
	ID        int64  `json:"id"`
	Label     string `json:"label"`
	Src       string `json:"src"`
	Format    string `json:"format"`
	Status    string `json:"status"`
	UpdatedAt string `json:"updated_at"`
	AugCount  int64  `json:"aug_count"`
}

// ClassItem is a single annotation class.
type ClassItem struct {
	ID    int64  `json:"id"`
	Name  string `json:"name"`
	Color string `json:"color"`
}

// ModelItem is a single configured model.
type ModelItem struct {
	ID           string `json:"id"`
	Label        string `json:"label"`
	ModelType    string `json:"modelType"`
	Available    bool   `json:"available"`
	Active       bool   `json:"active"`
	SupportsText bool   `json:"supportsText"`
}

// HealthStatus is the readiness/health payload.
type HealthStatus struct {
	Ready              bool   `json:"ready"`
	InferenceAvailable bool   `json:"inferenceAvailable"`
	DatabaseAccessible bool   `json:"databaseAccessible"`
	Message            string `json:"message"`
}

// UpsertClassRequest is the body for POST /classes.
type UpsertClassRequest struct {
	Name  string `json:"name" binding:"required"`
	Color string `json:"color" binding:"required"`
}

// DeleteImagesRequest is the body for POST /images/delete.
type DeleteImagesRequest struct {
	ImageIDs []int64 `json:"imageIds" binding:"required"`
}
