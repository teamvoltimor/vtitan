// Package bagreplay reads recorded MCAP bags from the Python/ROS2 stack so
// the Go nav port can be diffed against the behavior that produced them.
//
// This exists because every migration stage is gated on parity against the
// Python baseline before cutover, and unit tests cannot supply that: they
// prove the Go code does what its author intended, not that it does what
// CoreNavigator did. A bag is the only reference that carries real sensor
// input and the Python navigator's own per-tick internal state together.
//
// # Why /nav_debug is the reference
//
// The robot publishes /nav_debug every control tick regardless of which
// branch step() took, carrying the full NavigatorDebugSnapshot: pose, phase,
// crosstrack error, chosen steer target, risk levels, speed selection, and
// the final command. Several of those values cannot be reconstructed from
// /motor/* and /imu/data after the fact at all, which is why the topic was
// added in the first place.
//
// It is a std_msgs/String carrying pydantic's JSON, NOT a custom ROS2
// message -- see track_navigator_node.py's create_publisher(String, ...) and
// scripts/common/bag_io.py's decode_nav_debug. That is what makes reading it
// from Go cheap: no custom .msg schema has to be ported, only the trivial
// std_msgs/String CDR envelope, and the payload is ordinary JSON.
//
// # Driving the navigator from a bag
//
// The snapshot carries pose_x/pose_y/pose_yaw, so a replay can feed the Go
// navigator the same pose the Python navigator saw rather than re-deriving
// one. That is deliberate: it isolates the navigator under test. A localiser
// difference would otherwise show up as a navigator difference, and the
// point of the gate is to find out whether THIS port changed behavior.
package bagreplay
