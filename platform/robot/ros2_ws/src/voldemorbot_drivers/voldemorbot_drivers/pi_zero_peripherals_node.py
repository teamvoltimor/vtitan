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
from rclpy.executors import MultiThreadedExecutor

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

    # num_threads=2, not the default (os.cpu_count(), 4 on this board): only
    # one real concurrency need exists here -- oled_display_node's I2C-write
    # timer callback must not block its own subscription callbacks (see that
    # module's _timer_callback_group comment for the confirmed-on-hardware
    # bug this fixes: /ui/telemetry_summary silently never processed while a
    # slow write held the only thread). button_node and challenge_mode_node
    # are lightweight, non-blocking timers with no similar need. Defaulting
    # to 4 threads spins up idle workers with nothing concurrent to do on an
    # already CPU-starved board; 2 is the minimum that preserves the fix.
    executor = MultiThreadedExecutor(num_threads=2)
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
