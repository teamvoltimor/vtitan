// Package routes holds the REST route-path constants and path-parameter
// names that define the API's routing contract. These are shared between
// router.go (which wires routes onto the Gin engine) and the handlers
// package (which registers per-resource routes and reads path parameters),
// so they're kept separate from handlers/constants.go's implementation-only
// constants (headers, error messages, SSE formats, etc.).
package routes

const (
	// Liveness is the health probe liveness route (unversioned, mirroring FastAPI).
	Liveness = "/healthz"
	// Readiness is the health probe readiness route.
	Readiness = "/readyz"
	// Health is the health check route.
	Health = "/health"

	// APIv1 is the API v1 base group path.
	APIv1 = "/api/v1"

	// OpenAPISpec is the OpenAPI YAML spec route.
	OpenAPISpec = "/openapi.yaml"

	// Gallery is the gallery route.
	Gallery = "/gallery"
	// GalleryGrouped is the grouped gallery route.
	GalleryGrouped = "/gallery/grouped"
	// GalleryImport is the gallery import route.
	GalleryImport = "/gallery/import"

	// Images is the image detail route with ID path parameter.
	Images = "/images/:id"
	// ImageThumb serves a downscaled thumbnail for an image.
	ImageThumb = "/images/:id/thumb"
	// ImagesDelete is the image delete route.
	ImagesDelete = "/images/delete"
	// ImageIDParamName is the path parameter name for image ID.
	ImageIDParamName = "id"

	// Annotations is the annotation detail route with ID path parameter.
	Annotations = "/annotations/:id"
	// SaveAnnotations is the annotation save route.
	SaveAnnotations = "/annotations/save"
	// SkipAnnotation is the annotation skip route.
	SkipAnnotation = "/annotations/skip"
	// AnnotationIDParamName is the path parameter name for annotation ID.
	AnnotationIDParamName = "id"

	// ListClasses is the list classes route.
	ListClasses = "/classes"
	// UpsertClass is the upsert class route.
	UpsertClass = "/classes"

	// ListModels is the list models route.
	ListModels = "/models"

	// Segment is the segmentation route.
	Segment = "/segment"

	// StartAugment is the augmentation start route.
	StartAugment = "/augment/start"
	// StatusAugment is the augmentation status route.
	StatusAugment = "/augment/status"
	// StreamAugment is the augmentation stream SSE route.
	StreamAugment = "/augment/stream"

	// StartTrain is the training start route.
	StartTrain = "/train/start"
	// StatusTrain is the training status route.
	StatusTrain = "/train/status"
	// StreamTrain is the training stream SSE route.
	StreamTrain = "/train/stream"
)
