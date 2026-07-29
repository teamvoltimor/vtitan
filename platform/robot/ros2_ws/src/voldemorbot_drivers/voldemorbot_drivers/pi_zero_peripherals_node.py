"""Single-process entry point for the Pi Zero's low-rate peripherals.

Runs button_node and oled_display_node together in one process/rclpy init,
same rationale as the old 3-way pi_zero_node merge (fewer DDS participants,
less RAM). ackermann_motor_node now runs as its own process instead, so it
gets a dedicated core and isn't sharing executor threads with these two --
see platform/robot/docs/sensor-verification.md's feedback-rate tuning
section for why.

Usage:
    ros2 run voldemorbot_drivers pi_zero_peripherals_node

Or via launch file:
    ros2 launch voldemorbot_bringup rpi_zero_nodes.launch.py
"""

import rclpy
from rclpy.executors import SingleThreadedExecutor

from voldemorbot_drivers.button_node import ButtonNode
from voldemorbot_drivers.challenge_mode_node import ChallengeModeNode
from voldemorbot_drivers.oled_display_node import OLEDDisplayNode


def main(args: list[str] | None = None) -> None:
    """Run the Zero's GPIO peripherals in one process/rclpy init.

    button_node, oled_display_node and challenge_mode_node -- the challenge
    jumper is wired to the ZERO's GPIO23, so it has to be read here and
    published for state_machine_node on the Pi 5.
    """
    rclpy.init(args=args)

    button = ButtonNode()
    oled = OLEDDisplayNode()
    challenge_mode = ChallengeModeNode()

    button.trigger_configure()
    button.trigger_activate()
    oled.trigger_configure()
    oled.trigger_activate()

    # SingleThreadedExecutor, not MultiThreadedExecutor (2026-07-28): strace
    # on hardware showed 61.6% of this process's CPU time in futex calls --
    # GIL/lock contention between the executor's worker threads, not the I2C
    # write or GPIO polling anyone suspected. MultiThreadedExecutor was
    # originally required because oled_display_node's I2C-write timer
    # callback used to block for 0.3-3.25s (32 sequential blocking
    # os.write() calls per frame), which would have starved every
    # subscription callback for that long on a single thread -- confirmed on
    # hardware as /ui/telemetry_summary silently never processing. That write
    # is now a single syscall and measured well under 0.3s, so the
    # single-thread blocking window is short enough to tolerate; watch
    # oled_display_node's "[DIAG] ui_summary receive gap" warning if that
    # assumption turns out wrong.
    executor = SingleThreadedExecutor()
    executor.add_node(button)
    executor.add_node(oled)
    executor.add_node(challenge_mode)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        button.destroy_node()
        oled.destroy_node()
        challenge_mode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
