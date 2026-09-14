package profile

// DefaultDirectionEstimatorTOMLPath is
// src/config/navigation/blind_nav/direction_estimator.toml,
// relative to the repo root. No per-component profile overlays -- pass
// nil profileNames to Load. The file's shape is the generated
// blind_nav.NavigationBlindNavDirectionEstimator DTO; the extra generated
// fields (CornerClearanceM, GateLogPeriodTicks) are carried but still
// unconsumed by Go.
const DefaultDirectionEstimatorTOMLPath = "src/config/navigation/blind_nav/direction_estimator.toml"
