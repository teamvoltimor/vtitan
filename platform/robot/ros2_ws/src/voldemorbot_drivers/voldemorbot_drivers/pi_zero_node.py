"""Single-process entry point for all Pi Zero nodes.

Replaces three separate ROS2 processes (ackermann_motor_node, button_node,
oled_display_node) with one process and one rclpy init. Saves ~2 Python
interpreters and ~2 DDS participants (~60-100 MB on Pi Zero 2W's 512 MB
budget).

Usage:
    ros2 run voldemorbot_drivers pi_zero_node

Or via launch file:
    ros2 launch voldemorbot_bringup rpi_zero_nodes.launch.py
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    from voldemorbot_drivers.motors.ackermann_motor_node import AckermannMotorNode
    from voldemorbot_drivers.button_node import ButtonNode
    from voldemorbot_drivers.oled_display_node import OLEDDisplayNode

    ackermann = AckermannMotorNode()
    button = ButtonNode()
    oled = OLEDDisplayNode()

    ackermann.trigger_configure()
    ackermann.trigger_activate()
    button.trigger_configure()
    button.trigger_activate()
    oled.trigger_configure()
    oled.trigger_activate()

    executor = MultiThreadedExecutor()
    executor.add_node(ackermann)
    executor.add_node(button)
    executor.add_node(oled)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ackermann.destroy_node()
        button.destroy_node()
        oled.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
