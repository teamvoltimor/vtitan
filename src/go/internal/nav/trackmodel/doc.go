// Package trackmodel models the WRO track's wall geometry and waypoint-path
// frame -- the Go port of src/python/src/navigation/track_geometry.py.
//
// TrackWalls.Raycast is the single source of truth for where the track's
// walls sit, given each corridor's width, so a simulator generating a
// synthetic LIDAR scan and a real localizer inferring pose from one can
// never silently drift apart -- same role as the Python original.
//
// RaycastGrid (the localizer's batched grid-search variant) is deliberately
// not ported: it is a pure performance optimization over repeated Raycast
// calls with no behavioral difference, and the Go localizer
// (internal/nav/localization.LidarLocalizer) drives its grid search through
// RaycastFan instead, so nothing needs it.
package trackmodel
