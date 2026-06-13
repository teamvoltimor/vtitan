// Package dto defines the HTTP request/response shapes. JSON tags use the
// camelCase wire format produced by the Python Pydantic models (to_camel),
// except where the originals pinned snake_case aliases (updated_at, aug_count).
package dto

type (
	// Point is a normalized 2-D coordinate.
	Point struct {
		X float64 `json:"x"`
		Y float64 `json:"y"`
	}

	// Shape is a single mask/bounding-box annotation.
	Shape struct {
		ID        string  `json:"id"`
		ClassName string  `json:"className"`
		Points    []Point `json:"points"`
	}

	// GalleryStats holds aggregate image status counts.
	GalleryStats struct {
		Pending int     `json:"pending"`
		Done    int     `json:"done"`
		Skipped int     `json:"skipped"`
		Total   int     `json:"total"`
		Pct     float64 `json:"pct"`
	}

	// GalleryItem is one image entry in the gallery listing.
	GalleryItem struct {
		Label       string  `json:"label"`
		Src         string  `json:"src"`
		Format      string  `json:"format"`
		Status      string  `json:"status"`
		Updated     string  `json:"updated"`
		Annotations []Shape `json:"annotations"`
		ID          int64   `json:"id"`
	}

	// GalleryResponse is the full gallery payload.
	GalleryResponse struct {
		Items []GalleryItem `json:"items"`
		Stats GalleryStats  `json:"stats"`
	}

	// ParentImageItem is a parent (original) image with its augmentation count,
	// returned by GET /gallery/grouped. updated_at and aug_count keep snake_case.
	ParentImageItem struct {
		Label     string `json:"label"`
		Src       string `json:"src"`
		Format    string `json:"format"`
		Status    string `json:"status"`
		UpdatedAt string `json:"updated_at"`
		ID        int64  `json:"id"`
		AugCount  int64  `json:"aug_count"`
	}

	// ClassItem is a single annotation class.
	ClassItem struct {
		Name  string `json:"name"`
		Color string `json:"color"`
		ID    int64  `json:"id"`
	}

	// ModelItem is a single configured model.
	ModelItem struct {
		ID           string `json:"id"`
		Label        string `json:"label"`
		ModelType    string `json:"modelType"`
		Available    bool   `json:"available"`
		Active       bool   `json:"active"`
		SupportsText bool   `json:"supportsText"`
	}

	// LivenessResponse is the response payload for GET /healthz.
	LivenessResponse struct {
		Status string `json:"status"`
	}

	// HealthStatus is the readiness/health payload.
	HealthStatus struct {
		Message            string `json:"message"`
		Ready              bool   `json:"ready"`
		InferenceAvailable bool   `json:"inferenceAvailable"`
		DatabaseAccessible bool   `json:"databaseAccessible"`
	}

	// UpsertClassRequest is the body for POST /classes.
	UpsertClassRequest struct {
		Name  string `json:"name"  binding:"required"`
		Color string `json:"color" binding:"required"`
	}

	// DeleteImagesRequest is the body for POST /images/delete.
	DeleteImagesRequest struct {
		ImageIDs []int64 `json:"imageIds" binding:"required"`
	}

	// SaveAnnotationsRequest is the body for POST /annotations/save. An empty
	// shapes list is validated in the handler (400) to match the Python validator,
	// so it is not marked required here.
	SaveAnnotationsRequest struct {
		ExportFormat string  `json:"exportFormat" binding:"required,oneof=segmentation detection"`
		Shapes       []Shape `json:"shapes"`
		ImageID      int64   `json:"imageId"      binding:"required"`
	}

	// SkipRequest is the body for POST /annotations/skip.
	SkipRequest struct {
		ImageID int64 `json:"imageId" binding:"required"`
	}

	// SegmentationPoint is a click point for POST /segment.
	SegmentationPoint struct {
		PointType string  `json:"pointType" binding:"required,oneof=positive negative"`
		ClassName string  `json:"className" binding:"required"`
		X         float64 `json:"x"`
		Y         float64 `json:"y"`
	}

	// SegmentationRequest is the body for POST /segment. An empty points list is
	// validated in the handler (400) to match the Python validator.
	SegmentationRequest struct {
		Points  []SegmentationPoint `json:"points"`
		ImageID int64               `json:"imageId" binding:"required"`
	}

	// SegmentationResponse is the envelope for POST /segment.
	SegmentationResponse struct {
		State   string  `json:"state"`
		Message string  `json:"message"`
		Shapes  []Shape `json:"shapes"`
	}

	// JobStatusResponse is the status payload for augmentation/training jobs.
	JobStatusResponse struct {
		Message string `json:"message"`
		Running bool   `json:"running"`
	}

	// AugmentRequest is the body for POST /augment/start.
	AugmentRequest struct {
		ImageIDs         []int64 `json:"imageIds"`
		NumAugmentations int32   `json:"numAugmentations"`
	}

	// TrainRequest is the body for POST /train/start.
	TrainRequest struct {
		ModelName string `json:"modelName"`
		Epochs    int32  `json:"epochs"`
		Batch     int32  `json:"batch"`
		Imgsz     int32  `json:"imgsz"`
	}
)
