"""ROS2 parameter declaration and retrieval utilities.

Eliminates repetitive parameter handling patterns across nodes.

Split into ``declare_param`` (declare only) and ``get_*_param`` (get only) so
a :class:`~rclpy.lifecycle.LifecycleNode` can declare in ``__init__`` and
defer reading to ``on_configure``/``on_activate`` -- the shape every lifecycle
node in this package actually needs, since hardware isn't connected and
config isn't consumed until activation. ``declare_and_get_*_param`` composes
the two for the common case where a plain :class:`~rclpy.node.Node` wants
both immediately.
"""

from rclpy.node import Node


def declare_param(node: Node, key: str, default: object) -> None:
    """Declare a parameter, guarded against a caller that already has.

    ``rclpy`` raises if the same parameter is declared twice. The guard
    matters for anything built against a node it doesn't own end-to-end --
    e.g. ``ROS2HardwareGateway``, which is constructed against a host node
    (``TrackNavigator``) that's expected to have declared its topic
    parameters, but also against ad hoc nodes in contract tests that haven't.
    Declaring on demand here means adding a topic can't turn into a required
    ritual for every caller.

    Args:
        node: ROS2 Node instance.
        key: Parameter name.
        default: Default value if not already declared.
    """
    if not node.has_parameter(key):
        node.declare_parameter(key, default)


def get_str_param(node: Node, key: str) -> str:
    """Get an already-declared string parameter."""
    return str(node.get_parameter(key).get_parameter_value().string_value)


def get_int_param(node: Node, key: str) -> int:
    """Get an already-declared integer parameter."""
    return int(node.get_parameter(key).get_parameter_value().integer_value)


def get_float_param(node: Node, key: str) -> float:
    """Get an already-declared float parameter."""
    return float(node.get_parameter(key).get_parameter_value().double_value)


def get_bool_param(node: Node, key: str) -> bool:
    """Get an already-declared boolean parameter."""
    return bool(node.get_parameter(key).get_parameter_value().bool_value)


def declare_and_get_str_param(node: Node, key: str, default: str) -> str:
    """Declare and get a string parameter in one call.

    Args:
        node: ROS2 Node instance.
        key: Parameter name.
        default: Default value if not set.

    Returns:
        str: Parameter value.
    """
    declare_param(node, key, default)
    return get_str_param(node, key)


def declare_and_get_int_param(node: Node, key: str, default: int) -> int:
    """Declare and get an integer parameter in one call.

    Args:
        node: ROS2 Node instance.
        key: Parameter name.
        default: Default value if not set.

    Returns:
        int: Parameter value.
    """
    declare_param(node, key, default)
    return get_int_param(node, key)


def declare_and_get_float_param(node: Node, key: str, default: float) -> float:
    """Declare and get a float parameter in one call.

    Args:
        node: ROS2 Node instance.
        key: Parameter name.
        default: Default value if not set.

    Returns:
        float: Parameter value.
    """
    declare_param(node, key, default)
    return get_float_param(node, key)


def declare_and_get_bool_param(node: Node, key: str, default: bool) -> bool:
    """Declare and get a boolean parameter in one call.

    Args:
        node: ROS2 Node instance.
        key: Parameter name.
        default: Default value if not set.

    Returns:
        bool: Parameter value.
    """
    declare_param(node, key, default)
    return get_bool_param(node, key)
