package navutil

// LidarScan is a single LIDAR sweep in the robot frame (0 rad = forward,
// +pi/2 = left). It lives in this lowest-level navigation package so the
// geometry helpers here and the controllers that consume them can share one
// definition without an import cycle: controllers imports navutil, never the
// reverse.
type LidarScan struct {
	RangesM   []float64
	AnglesRad []float64
}
