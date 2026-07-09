"""ROS2 parameter declaration and retrieval utilities.

Eliminates repetitive parameter handling patterns across nodes.
"""

from rclpy.node import Node


def declare_and_get_str_param(node: Node, key: str, default: str) -> str:
    """Declare and get a string parameter in one call.

    Args:
        node: ROS2 Node instance.
        key: Parameter name.
        default: Default value if not set.

    Returns:
        str: Parameter value.
    """
    node.declare_parameter(key, default)
    return str(node.get_parameter(key).get_parameter_value().string_value)


def declare_and_get_int_param(node: Node, key: str, default: int) -> int:
    """Declare and get an integer parameter in one call.

    Args:
        node: ROS2 Node instance.
        key: Parameter name.
        default: Default value if not set.

    Returns:
        int: Parameter value.
    """
    node.declare_parameter(key, default)
    return int(node.get_parameter(key).get_parameter_value().integer_value)


def declare_and_get_float_param(node: Node, key: str, default: float) -> float:
    """Declare and get a float parameter in one call.

    Args:
        node: ROS2 Node instance.
        key: Parameter name.
        default: Default value if not set.

    Returns:
        float: Parameter value.
    """
    node.declare_parameter(key, default)
    return float(node.get_parameter(key).get_parameter_value().double_value)


def declare_and_get_bool_param(node: Node, key: str, default: bool) -> bool:
    """Declare and get a boolean parameter in one call.

    Args:
        node: ROS2 Node instance.
        key: Parameter name.
        default: Default value if not set.

    Returns:
        bool: Parameter value.
    """
    node.declare_parameter(key, default)
    return bool(node.get_parameter(key).get_parameter_value().bool_value)
