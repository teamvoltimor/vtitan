package handlers

import (
	"time"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/domain"
)

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
	// RouteImageThumb serves a downscaled thumbnail for an image.
	RouteImageThumb = "/images/:id/thumb"
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
	ExportFormatSegmentation = domain.ExportFormatSegmentation
	// ExportFormatDetection is the export format constant for detection annotations.
	ExportFormatDetection = domain.ExportFormatDetection
	// FormatDetectionAbbr is the abbreviation for detection format.
	FormatDetectionAbbr = domain.FormatDetectionAbbr
	// FormatSegmentationAbbr is the abbreviation for segmentation format.
	FormatSegmentationAbbr = domain.FormatSegmentationAbbr

	// LabelsDirName is the directory name for annotation labels.
	LabelsDirName = domain.LabelsDirName
	// ImagesDirName is the directory name for images.
	ImagesDirName = domain.ImagesDirName
	// PendingDirName is the directory name for pending exports.
	PendingDirName = domain.PendingDirName
	// ThumbsDirName is the directory where generated thumbnails are cached.
	ThumbsDirName = domain.ThumbsDirName
	// DataYAMLName is the data.yaml file name for training.
	DataYAMLName = domain.DataYAMLName

	// ThumbMaxWidth is the max width (px) of generated thumbnails; height scales
	// to preserve aspect ratio.
	ThumbMaxWidth = domain.ThumbMaxWidth
	// ThumbJPEGQuality is the JPEG quality used when encoding cached thumbnails.
	ThumbJPEGQuality = domain.ThumbJPEGQuality
	// ThumbCacheControl lets browsers cache thumbnails aggressively.
	ThumbCacheControl = "public, max-age=86400"

	// ImageURLTemplate is the absolute URL template for image routes, used when
	// API_PUBLIC_URL is explicitly configured.
	ImageURLTemplate = domain.ImageURLTemplate

	// RelativeImageURLTemplate is the proxy-relative image URL, used by default
	// so image requests share the frontend's origin.
	RelativeImageURLTemplate = domain.RelativeImageURLTemplate

	// ImageCacheControl lets the browser cache served image files so repeated
	// gallery/preview renders don't refetch them.
	ImageCacheControl = "public, max-age=3600"

	// DefaultNumAugmentations is the default number of augmentations.
	DefaultNumAugmentations = domain.DefaultNumAugmentations
	// DefaultTrainModel is the default training model.
	DefaultTrainModel = domain.DefaultTrainModel
	// DefaultTrainEpochs is the default number of training epochs.
	DefaultTrainEpochs = domain.DefaultTrainEpochs
	// DefaultTrainBatch is the default training batch size.
	DefaultTrainBatch = domain.DefaultTrainBatch
	// DefaultTrainImgsz is the default training image size.
	DefaultTrainImgsz = domain.DefaultTrainImgsz

	// SSEHeartbeatInterval is the SSE streaming heartbeat interval.
	SSEHeartbeatInterval = 30 * time.Second

	// PercentageScale is the percentage calculation scale (multiply by 1000, divide by 10 for decimal precision).
	PercentageScale = domain.PercentageScale
	// PercentageDivisor is the divisor to convert from PercentageScale back to percentage.
	PercentageDivisor = domain.PercentageDivisor

	ErrTitleValidation = "Validation Error"

	// Error messages
	ErrInvalidImageID      = "invalid image id"
	ErrNoFilesProvided     = "No files provided"
	ErrNoImagesToDelete    = "No images to delete"
	ErrNoImagesSpecified   = domain.ErrNoImagesSpecified
	ErrAugmentationRunning = domain.ErrAugmentationRunning
	ErrTrainingRunning     = domain.ErrTrainingRunning
	ErrAtLeastOneShape     = domain.ErrAtLeastOneShape
	ErrAtLeastOnePoint     = domain.ErrAtLeastOnePoint

	// Error format strings
	ErrFmtUnknownClass        = domain.ErrFmtUnknownClass
	ErrFmtUnknownPrimaryClass = domain.ErrFmtUnknownPrimaryClass
	ErrFmtNoJobRunning        = "No %s job running"

	// Status / success messages
	MsgSystemReady           = "System ready"
	MsgDatabaseError         = "Database error"
	MsgAugmentationStarted   = "Augmentation started"
	MsgTrainingStarted       = "Training started"
	MsgAugmentationCompleted = domain.MsgAugmentationCompleted
	MsgTrainingCompleted     = domain.MsgTrainingCompleted

	// Magic numbers
	NumBBoxCoords    = domain.NumBBoxCoords
	MinPolygonCoords = domain.MinPolygonCoords
	CoordPairSize    = domain.CoordPairSize
	RandomHexBytes   = domain.RandomHexBytes

	// Directory / file permissions
	DirPerm = domain.DirPerm

	// File extensions
	ThumbFileExt     = domain.ThumbFileExt
	ThumbTempPattern = domain.ThumbTempPattern
	LabelFileExt     = domain.LabelFileExt

	// Content types
	ContentTypeYAML = "application/yaml"

	// HTTP headers
	HeaderContentType       = "Content-Type"
	HeaderCacheControl      = "Cache-Control"
	HeaderXAccelBuffering   = "X-Accel-Buffering"
	HeaderValueEventStream  = "text/event-stream"
	HeaderValueNoCache      = "no-cache"
	HeaderValueBufferingOff = "no"

	// SSE format strings
	SSEDataFormat      = "data: %s\n\n"
	SSEHeartbeatFormat = "data: {\"heartbeat\": true}\n\n"

	// SSE payload map keys
	MapKeyError    = domain.MapKeyError
	MapKeyFinished = domain.MapKeyFinished
	MapKeyStage    = domain.MapKeyStage
	MapKeyProgress = domain.MapKeyProgress
	MapKeyDetails  = domain.MapKeyDetails

	// Image/annotation metadata
	ClassFallbackNameFmt = domain.ClassFallbackNameFmt
	ShapeIDLoadedFmt     = domain.ShapeIDLoadedFmt
	UnderscoreSep        = domain.UnderscoreSep
	FallbackHexString    = domain.FallbackHexString

	// Error context prefixes (fmt.Errorf wraparound)
	ErrCtxListImages   = domain.ErrCtxListImages
	ErrCtxStatusCounts = domain.ErrCtxStatusCounts
	ErrCtxListClasses  = domain.ErrCtxListClasses
)
