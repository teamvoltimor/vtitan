# Obstacles Challenge tuning overlay

Empty by default. Add `<subfolder>/<group>.toml` files here (same layout as
`src/shared/config/navigation/`, e.g. `waypoint/waypoints.toml` for
`ARC_RADIUS`, `motion/pursuit.toml` for lookahead) to override specific
tuning keys for the Obstacles Challenge only. Loaded via
`NavigationTuning.load_default(challenge=ScenarioType.OBSTACLES)`, merged
last (highest priority) over the base config and any active hardware
profile.

Do not add a value here without a measurement backing it -- see
`src/docs/sign-avoidance-investigation.md` for why an earlier
per-challenge tuning profile was removed after being found not to move the
outcome it was meant to fix.
