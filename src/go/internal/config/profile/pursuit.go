package profile

// DefaultPursuitTOMLPath is
// src/config/navigation/motion/pursuit.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load. The file's shape is the generated motion.NavigationMotionPursuit DTO.
// WallMarginSafetyM/MinLookaheadTransitionM are carried even though
// internal/nav/controllers.WaypointController does not (yet) derive a
// crosstrack budget the way CoreNavigator does.
const DefaultPursuitTOMLPath = "src/config/navigation/motion/pursuit.toml"
