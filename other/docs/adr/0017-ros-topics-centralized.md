# 0017. ROS2 topic names are centralized

- Status: superseded by 0069
- Superseded by: 0069
- Date: 2026-08-08
- Commit: cdb4ca15

## Context

Topic names were hardcoded across about ten node files. A 2026-08-08 audit found
the stale `cmd_vel`/`Twist` pair and a set of topics with no central definition:
challenge-mode jumper, motor feedback, button events, `race_metrics`,
`system_status`. As of 2026-08-02 no node published `/odom` either (the telemetry
bridge subscribes but receives no data).

## Options considered

- (a) Keep literal topic strings in each node.
- (b) One TOML of topic names, loaded via `RosTopicConfig.load_default()`.

## Decision

(b). `src/config/ros_topics.toml` is the single source of truth. The 2026-08-08
update replaced stale `cmd_vel`/`Twist` with the real `ackermann_cmd` topic and
filled in the topics the audit found hardcoded.

## Consequences

- A topic rename is one edit, and stale topics are visible in one place.
- `[navigation].odometry` stays commented out until odometry is implemented; the
  schema allows it but the TOML omits it.

## Superseded by 0069

Replaced by [0069](0069-config-governance.md): Config values live in TOML, descriptions
in schemas, rationale in ADRs.

The successor carries the current decision and its rationale; this file keeps
the original decision above so the supersede chain stays readable.
