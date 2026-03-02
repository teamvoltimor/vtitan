#!/usr/bin/env python3
"""
LIDAR Visualization Tool - Debug LIDAR distance measurements

Subscribes to /lidar topic and displays real-time 2D visualization
of distance measurements around the robot.

Usage:
    python3 visualize_lidar.py
"""

import math

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib.animation import FuncAnimation
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarVisualizer(Node):
    """Visualize LIDAR scan data in real-time"""

    def __init__(self):
        super().__init__("lidar_visualizer")

        # Subscribe to LIDAR topic
        self.subscription = self.create_subscription(
            LaserScan,
            "/lidar",
            self.lidar_callback,
            10,
        )

        # Store latest scan data
        self.ranges = None
        self.angle_min = 0
        self.angle_max = 2 * math.pi
        self.angle_increment = 0
        self.range_min = 0.05
        self.range_max = 3.0

        self.get_logger().info("LIDAR Visualizer started. Waiting for LIDAR data...")

    def lidar_callback(self, msg):
        """Process incoming LIDAR scan data"""
        self.ranges = np.array(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max

        # Replace inf values with max range
        self.ranges[np.isinf(self.ranges)] = self.range_max


def main():
    # Initialize ROS2
    rclpy.init()

    # Create visualizer node
    visualizer = LidarVisualizer()

    # Set up matplotlib figure
    fig, (ax_polar, ax_cart) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("LIDAR Visualization - Real-time Distance Measurements", fontsize=14, fontweight="bold")

    # Polar plot (left side) - shows LIDAR rays
    ax_polar.set_theta_zero_location("N")  # 0 degrees at top
    ax_polar.set_theta_direction(-1)  # Clockwise
    ax_polar.set_ylim(0, 3.5)
    ax_polar.set_title("Polar View (Robot at Center)", fontsize=12)
    ax_polar.grid(True, alpha=0.3)
    line_polar, = ax_polar.plot([], [], "b.", markersize=2, alpha=0.6)

    # Cartesian plot (right side) - top-down view
    ax_cart.set_xlim(-3.5, 3.5)
    ax_cart.set_ylim(-3.5, 3.5)
    ax_cart.set_aspect("equal")
    ax_cart.set_xlabel("X (meters)", fontsize=10)
    ax_cart.set_ylabel("Y (meters)", fontsize=10)
    ax_cart.set_title("Top-Down View (Robot at Origin)", fontsize=12)
    ax_cart.grid(True, alpha=0.3)
    ax_cart.axhline(y=0, color="k", linewidth=0.5, alpha=0.3)
    ax_cart.axvline(x=0, color="k", linewidth=0.5, alpha=0.3)

    # Robot indicator (red triangle pointing forward)
    robot_triangle = plt.Polygon([[0, 0.15], [-0.1, -0.1], [0.1, -0.1]],
                                  color="red", alpha=0.7, label="Robot")
    ax_cart.add_patch(robot_triangle)

    line_cart, = ax_cart.plot([], [], "b.", markersize=3, alpha=0.6, label="LIDAR points")
    ax_cart.legend(loc="upper right", fontsize=9)

    # Text for statistics
    stats_text = ax_cart.text(0.02, 0.98, "", transform=ax_cart.transAxes,
                               verticalalignment="top", fontsize=9,
                               bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

    def update_plot(frame):
        """Update plot with latest LIDAR data"""
        # Spin ROS2 to get latest data
        rclpy.spin_once(visualizer, timeout_sec=0.01)

        if visualizer.ranges is None or len(visualizer.ranges) == 0:
            return line_polar, line_cart, stats_text

        # Calculate angles for each measurement
        num_points = len(visualizer.ranges)
        angles = np.linspace(visualizer.angle_min, visualizer.angle_max, num_points)

        # Filter valid ranges
        valid_mask = (visualizer.ranges >= visualizer.range_min) & \
                     (visualizer.ranges <= visualizer.range_max)
        valid_angles = angles[valid_mask]
        valid_ranges = visualizer.ranges[valid_mask]

        # Update polar plot
        line_polar.set_data(valid_angles, valid_ranges)

        # Convert to Cartesian coordinates for top-down view
        # Robot faces forward (+Y), LIDAR angle 0 is forward
        # Negate X so left appears on left side of plot
        x = -valid_ranges * np.sin(valid_angles)  # X: negative for correct left/right
        y = valid_ranges * np.cos(valid_angles)   # Y: forward/back
        line_cart.set_data(x, y)

        # Calculate statistics
        if len(valid_ranges) > 0:
            min_dist = np.min(valid_ranges)
            max_dist = np.max(valid_ranges)
            mean_dist = np.mean(valid_ranges)

            # Find closest obstacles in each direction
            # Forward (around 0°, ±15°)
            forward_mask = (np.abs(valid_angles) < 0.26) | (np.abs(valid_angles - 2*np.pi) < 0.26)
            forward_dist = np.min(valid_ranges[forward_mask]) if np.any(forward_mask) else float("inf")

            # Left (around 90°, π/2)
            left_mask = np.abs(valid_angles - np.pi/2) < 0.26
            left_dist = np.min(valid_ranges[left_mask]) if np.any(left_mask) else float("inf")

            # Right (around 270°, 3π/2)
            right_mask = np.abs(valid_angles - 3*np.pi/2) < 0.26
            right_dist = np.min(valid_ranges[right_mask]) if np.any(right_mask) else float("inf")

            # Back (around 180°, π)
            back_mask = np.abs(valid_angles - np.pi) < 0.26
            back_dist = np.min(valid_ranges[back_mask]) if np.any(back_mask) else float("inf")

            stats_text.set_text(
                f"Statistics:\n"
                f"Points: {len(valid_ranges)}/{num_points}\n"
                f"Range: {min_dist:.2f}m - {max_dist:.2f}m\n"
                f"Mean: {mean_dist:.2f}m\n\n"
                f"Distances:\n"
                f"Forward: {forward_dist:.2f}m\n"
                f"Left: {left_dist:.2f}m\n"
                f"Right: {right_dist:.2f}m\n"
                f"Back: {back_dist:.2f}m",
            )

        return line_polar, line_cart, stats_text

    # Set up animation
    ani = FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)

    # Show plot
    plt.tight_layout()

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
