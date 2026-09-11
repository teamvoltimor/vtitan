package api

// Telemetry context route paths (registered under edge.RouteV1Telemetry).
const (
	RouteHealth   = "/health"
	RouteLatest   = "/latest"
	RouteHistory  = "/history"
	RouteTopics   = "/topics"
	RouteSpeed    = "/robot/config/speed"
	RouteConfig   = "/config"
	RouteSessions = "/sessions"
	RouteSession  = "/sessions/:id"
)

// Robot context route paths.
const (
	RouteRobots       = "/robots"
	RouteRobot        = "/robots/:robotId"
	RouteRobotStatus  = "/robots/:robotId/status"
	RouteRobotConfig  = "/robots/:robotId/config"
	RouteRobotCommand = "/robots/:robotId/command"
)

// Navigation context route paths.
const (
	RouteNavigation = "/navigation"
	RouteWaypoints  = "/waypoints"
	RouteWaypoint   = "/waypoints/:waypointId"
	RouteRoute      = "/route"
	RouteNavStatus  = "/status"
	RouteClearance  = "/clearance"
	RouteTuning     = "/tuning"
)

// Simulation context route paths.
const (
	RouteSimulation  = "/simulation"
	RouteScenarios   = "/scenarios"
	RouteScenario    = "/scenarios/:scenarioId"
	RouteRuns        = "/runs"
	RouteRun         = "/runs/:runId"
	RouteEnvironment = "/environments"
)

// Vision context route paths.
const (
	RouteVision            = "/vision"
	RouteDetections        = "/detections"
	RouteDetectionsCurrent = "/detections/current"
	RouteAnnotations       = "/annotations"
	RouteAnnotation        = "/annotations/:annotationId"
	RouteModel             = "/model"
	RoutePipelineStatus    = "/pipeline/status"
	DefaultDetectionLimit  = 50
)
