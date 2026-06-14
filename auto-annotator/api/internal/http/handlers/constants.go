package handlers

import "time"

const (
	// RouteLiveness is the health probe liveness route (unversioned, mirroring FastAPI).
	RouteLiveness = "/healthz"
	// RouteReadiness is the health probe readiness route.
	RouteReadiness = "/readyz"
	// RouteHealth is the health check route.
	RouteHealth = "/health"

	// RouteAPIv1 is the API v1 base group path.
	RouteAPIv1 = "/api/v1"

	// RouteGallery is the gallery route.
	RouteGallery = "/gallery"
	// RouteGalleryGrouped is the grouped gallery route.
	RouteGalleryGrouped = "/gallery/grouped"
	// RouteGalleryImport is the gallery import route.
	RouteGalleryImport = "/gallery/import"
	// GalleryImportFormFieldKey is the form field key for gallery imports.
	GalleryImportFormFieldKey = "files"

	// RouteImages is the image detail route with ID path parameter.
	RouteImages = "/images/:id"
	// RouteImagesDelete is the image delete route.
	RouteImagesDelete = "/images/delete"
	// ImageIDParamName is the path parameter name for image ID.
	ImageIDParamName = "id"

	// RouteAnnotations is the annotation detail route with ID path parameter.
	RouteAnnotations = "/annotations/:id"
	// RouteSaveAnnotations is the annotation save route.
	RouteSaveAnnotations = "/annotations/save"
	// RouteSkipAnnotation is the annotation skip route.
	RouteSkipAnnotation = "/annotations/skip"
	// AnnotationIDParamName is the path parameter name for annotation ID.
	AnnotationIDParamName = "id"

	// RouteListClasses is the list classes route.
	RouteListClasses = "/classes"
	// RouteUpsertClass is the upsert class route.
	RouteUpsertClass = "/classes"

	// RouteListModels is the list models route.
	RouteListModels = "/models"

	// RouteSegment is the segmentation route.
	RouteSegment = "/segment"

	// RouteStartAugment is the augmentation start route.
	RouteStartAugment = "/augment/start"
	// RouteStatusAugment is the augmentation status route.
	RouteStatusAugment = "/augment/status"
	// RouteStreamAugment is the augmentation stream SSE route.
	RouteStreamAugment = "/augment/stream"
	// JobIDQueryKey is the query key for job ID in augmentation endpoints.
	JobIDQueryKey = "id"

	// RouteStartTrain is the training start route.
	RouteStartTrain = "/train/start"
	// RouteStatusTrain is the training status route.
	RouteStatusTrain = "/train/status"
	// RouteStreamTrain is the training stream SSE route.
	RouteStreamTrain = "/train/stream"

	// ExportFormatSegmentation is the export format constant for segmentation annotations.
	ExportFormatSegmentation = "segmentation"
	// ExportFormatDetection is the export format constant for detection annotations.
	ExportFormatDetection = "detection"
	// FormatDetectionAbbr is the abbreviation for detection format.
	FormatDetectionAbbr = "det"
	// FormatSegmentationAbbr is the abbreviation for segmentation format.
	FormatSegmentationAbbr = "seg"

	// LabelsDirName is the directory name for annotation labels.
	LabelsDirName = "labels"
	// ImagesDirName is the directory name for images.
	ImagesDirName = "images"
	// PendingDirName is the directory name for pending exports.
	PendingDirName = "pending"
	// DataYAMLName is the data.yaml file name for training.
	DataYAMLName = "data.yaml"

	// StatusOK is the response status indicator for success.
	StatusOK = "ok"

	// ImageURLTemplate is the URL path template format for image routes.
	ImageURLTemplate = "%s/api/v1/images/%d"

	// TimeFormatISO8601 is the time format for RFC3339 with microsecond precision
	// matching Python datetime.isoformat().
	TimeFormatISO8601 = "2006-01-02T15:04:05.000000-07:00"

	// DefaultNumAugmentations is the default number of augmentations.
	DefaultNumAugmentations = 9
	// DefaultTrainModel is the default training model.
	DefaultTrainModel = "yolo11s.pt"
	// DefaultTrainEpochs is the default number of training epochs.
	DefaultTrainEpochs = 50
	// DefaultTrainBatch is the default training batch size.
	DefaultTrainBatch = 16
	// DefaultTrainImgsz is the default training image size.
	DefaultTrainImgsz = 640

	// SSEHeartbeatInterval is the SSE streaming heartbeat interval.
	SSEHeartbeatInterval = 30 * time.Second

	// PercentageScale is the percentage calculation scale (multiply by 1000, divide by 10 for decimal precision).
	PercentageScale = 1000
)
