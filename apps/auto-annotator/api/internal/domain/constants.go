package domain

// Filesystem layout shared across the gallery, annotation, and handlers packages.
const (
	LabelsDirName  = "labels"
	ImagesDirName  = "images"
	PendingDirName = "pending"
	ThumbsDirName  = "thumbs"
	DataYAMLName   = "data.yaml"
	DirPerm        = 0o750
)

// Thumbnail generation.
const (
	ThumbMaxWidth    = 320
	ThumbJPEGQuality = 80
	ThumbFileExt     = ".jpg"
	ThumbTempPattern = "thumb-*.jpg"
)

// Export formats.
const (
	ExportFormatSegmentation = "segmentation"
	ExportFormatDetection    = "detection"
	FormatDetectionAbbr      = "det"
	FormatSegmentationAbbr   = "seg"
	LabelFileExt             = ".txt"
)

// Annotation shape validation.
const (
	NumBBoxCoords    = 4
	MinPolygonCoords = 4
	CoordPairSize    = 2
)

// Image URLs.
const (
	// ImageURLTemplate is the absolute URL template for image routes, used when
	// API_PUBLIC_URL is explicitly configured.
	ImageURLTemplate = "%s/api/v1/images/%d"
	// RelativeImageURLTemplate is the proxy-relative image URL, used by default
	// so image requests share the frontend's origin.
	RelativeImageURLTemplate = "/api/v1/images/%d"
)

// Percentage calculation (multiply by PercentageScale, divide by PercentageDivisor
// for one decimal place of precision).
const (
	PercentageScale   = 1000
	PercentageDivisor = 10
)

// Random hex generation (upload filenames, fallback IDs).
const (
	RandomHexBytes    = 16
	UnderscoreSep     = "_"
	FallbackHexString = "00000000000000000000000000000000"
)

// Training / augmentation defaults.
const (
	DefaultNumAugmentations = 9
	DefaultTrainModel       = "yolo11s.pt"
	DefaultTrainEpochs      = 50
	DefaultTrainBatch       = 16
	DefaultTrainImgsz       = 640
)

// Timestamp format used across all SQLite stores — ISO 8601 with microseconds
// and timezone offset, matching Python datetime.isoformat().
const TimeFormat = "2006-01-02T15:04:05.000000-07:00"

// Shared error and status messages.
const (
	ErrAtLeastOneShape        = "At least one shape required"
	ErrAtLeastOnePoint        = "At least one point required"
	ErrNoImagesSpecified      = "No images specified"
	ErrAugmentationRunning    = "Augmentation already running"
	ErrTrainingRunning        = "Training already running"
	ErrFmtUnknownClass        = "Unknown class '%s'"
	ErrFmtUnknownPrimaryClass = "Unknown primary class '%s'"
	MsgAugmentationCompleted  = "Augmentation completed"
	MsgTrainingCompleted      = "Training completed"
)

// Annotation metadata formatting.
const (
	ClassFallbackNameFmt = "class_%d"
	ShapeIDLoadedFmt     = "loaded-%d-%d"
)

// Error-context prefixes for fmt.Errorf wraparound.
const (
	ErrCtxListImages   = "list images"
	ErrCtxStatusCounts = "status counts"
	ErrCtxListClasses  = "list classes"
)
