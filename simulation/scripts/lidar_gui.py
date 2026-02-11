#!/usr/bin/env python3
"""
LIDAR GUI - Interactive visualization tool with controls

Real-time LIDAR distance visualization with a proper GUI interface.

Usage:
    python3 lidar_gui.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import tkinter as tk
from tkinter import ttk
import math
import threading


class LidarGUI:
    """Interactive LIDAR visualization GUI"""

    def __init__(self, root):
        self.root = root
        self.root.title("LIDAR Distance Visualizer")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2b2b2b')

        # Initialize ROS2 in background thread
        self.ros_thread = None
        self.node = None
        self.running = False
        self.paused = False

        # LIDAR data
        self.ranges = None
        self.angles = None
        self.range_max = 3.0

        # Colors
        self.bg_color = '#2b2b2b'
        self.canvas_bg = '#1e1e1e'
        self.grid_color = '#404040'
        self.robot_color = '#ff4444'
        self.lidar_color = '#00ff00'
        self.text_color = '#ffffff'

        self.setup_ui()
        self.start_ros()

    def setup_ui(self):
        """Set up the GUI layout"""
        # Main container
        main_frame = tk.Frame(self.root, bg=self.bg_color)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left panel - Controls and stats
        left_panel = tk.Frame(main_frame, bg=self.bg_color, width=300)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left_panel.pack_propagate(False)

        # Title
        title = tk.Label(left_panel, text="LIDAR Control Panel",
                        font=('Arial', 16, 'bold'),
                        bg=self.bg_color, fg=self.text_color)
        title.pack(pady=(0, 20))

        # Status indicator
        self.status_frame = tk.Frame(left_panel, bg=self.bg_color)
        self.status_frame.pack(pady=10)

        tk.Label(self.status_frame, text="Status:", font=('Arial', 12),
                bg=self.bg_color, fg=self.text_color).pack(side=tk.LEFT, padx=5)

        self.status_indicator = tk.Canvas(self.status_frame, width=20, height=20,
                                         bg=self.bg_color, highlightthickness=0)
        self.status_indicator.pack(side=tk.LEFT, padx=5)
        self.status_circle = self.status_indicator.create_oval(2, 2, 18, 18, fill='gray')

        self.status_label = tk.Label(self.status_frame, text="Waiting...",
                                     font=('Arial', 10),
                                     bg=self.bg_color, fg='gray')
        self.status_label.pack(side=tk.LEFT, padx=5)

        # Control buttons
        button_frame = tk.Frame(left_panel, bg=self.bg_color)
        button_frame.pack(pady=10, fill=tk.X)

        self.pause_button = tk.Button(button_frame, text="⏸ Pause",
                                      command=self.toggle_pause,
                                      font=('Arial', 12),
                                      bg='#444444', fg=self.text_color,
                                      activebackground='#555555',
                                      width=15, height=2)
        self.pause_button.pack(pady=5)

        tk.Button(button_frame, text="🔄 Reset View",
                 command=self.reset_view,
                 font=('Arial', 12),
                 bg='#444444', fg=self.text_color,
                 activebackground='#555555',
                 width=15, height=2).pack(pady=5)

        # Separator
        ttk.Separator(left_panel, orient='horizontal').pack(fill=tk.X, pady=20)

        # Statistics display
        stats_label = tk.Label(left_panel, text="Distance Statistics",
                              font=('Arial', 14, 'bold'),
                              bg=self.bg_color, fg=self.text_color)
        stats_label.pack(pady=(0, 10))

        # Stats frame with better formatting
        stats_frame = tk.Frame(left_panel, bg='#333333', relief=tk.RIDGE, bd=2)
        stats_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.stats_text = tk.Text(stats_frame, font=('Courier', 10),
                                  bg='#1e1e1e', fg='#00ff00',
                                  height=20, relief=tk.FLAT,
                                  padx=10, pady=10)
        self.stats_text.pack(fill=tk.BOTH, expand=True)
        self.stats_text.config(state=tk.DISABLED)

        # Right panel - Visualization canvas
        right_panel = tk.Frame(main_frame, bg=self.canvas_bg, relief=tk.SUNKEN, bd=2)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        canvas_label = tk.Label(right_panel, text="Top-Down View (Robot at Center)",
                               font=('Arial', 12, 'bold'),
                               bg=self.canvas_bg, fg=self.text_color)
        canvas_label.pack(pady=5)

        # Main visualization canvas
        self.canvas = tk.Canvas(right_panel, bg=self.canvas_bg,
                               highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Bind resize event
        self.canvas.bind('<Configure>', self.on_canvas_resize)

        # Start update loop
        self.update_visualization()

    def start_ros(self):
        """Start ROS2 node in background thread"""
        self.running = True

        def ros_spin():
            rclpy.init()
            self.node = LidarNode()
            while self.running:
                rclpy.spin_once(self.node, timeout_sec=0.01)
            self.node.destroy_node()
            rclpy.shutdown()

        self.ros_thread = threading.Thread(target=ros_spin, daemon=True)
        self.ros_thread.start()

    def toggle_pause(self):
        """Toggle pause state"""
        self.paused = not self.paused
        if self.paused:
            self.pause_button.config(text="▶ Resume")
        else:
            self.pause_button.config(text="⏸ Pause")

    def reset_view(self):
        """Reset visualization"""
        self.canvas.delete('all')
        self.draw_grid()

    def on_canvas_resize(self, event):
        """Handle canvas resize"""
        self.draw_grid()

    def draw_grid(self):
        """Draw background grid and axes"""
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        center_x = width // 2
        center_y = height // 2

        # Calculate scale (pixels per meter)
        max_size = min(width, height)
        self.scale = (max_size - 100) / (2 * self.range_max)

        # Draw grid circles (distance rings)
        for dist in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]:
            radius = dist * self.scale
            self.canvas.create_oval(
                center_x - radius, center_y - radius,
                center_x + radius, center_y + radius,
                outline=self.grid_color, width=1
            )
            # Distance labels
            self.canvas.create_text(
                center_x, center_y - radius - 15,
                text=f'{dist}m', fill=self.grid_color,
                font=('Arial', 9)
            )

        # Draw axes
        self.canvas.create_line(center_x, center_y - max_size//2,
                               center_x, center_y + max_size//2,
                               fill=self.grid_color, width=1, dash=(3, 3))
        self.canvas.create_line(center_x - max_size//2, center_y,
                               center_x + max_size//2, center_y,
                               fill=self.grid_color, width=1, dash=(3, 3))

        # Direction labels
        self.canvas.create_text(center_x, center_y - max_size//2 + 20,
                               text='↑ FORWARD', fill=self.text_color,
                               font=('Arial', 11, 'bold'))
        self.canvas.create_text(center_x + max_size//2 - 40, center_y,
                               text='RIGHT →', fill=self.text_color,
                               font=('Arial', 11, 'bold'))
        self.canvas.create_text(center_x - max_size//2 + 40, center_y,
                               text='← LEFT', fill=self.text_color,
                               font=('Arial', 11, 'bold'))
        self.canvas.create_text(center_x, center_y + max_size//2 - 20,
                               text='↓ BACK', fill=self.text_color,
                               font=('Arial', 11, 'bold'))

    def draw_robot(self, center_x, center_y):
        """Draw robot at center"""
        size = 20
        # Triangle pointing forward
        points = [
            center_x, center_y - size,      # Top point (forward)
            center_x - size, center_y + size,  # Bottom left
            center_x + size, center_y + size   # Bottom right
        ]
        self.canvas.create_polygon(points, fill=self.robot_color,
                                   outline='white', width=2)

    def update_statistics(self):
        """Update statistics display"""
        if self.node is None or self.node.ranges is None:
            stats = "Waiting for LIDAR data...\n\n"
            stats += "Status: No data received\n"
            stats += "\nMake sure:\n"
            stats += "• Gazebo is running\n"
            stats += "• ROS bridge is active\n"
            stats += "• /lidar topic exists\n"
        else:
            ranges = self.node.ranges
            angles = self.node.angles

            # Filter valid ranges
            valid_mask = (ranges >= self.node.range_min) & (ranges <= self.node.range_max)
            valid_ranges = ranges[valid_mask]
            valid_angles = angles[valid_mask]

            if len(valid_ranges) > 0:
                # Calculate directional distances
                forward = self.get_min_distance(valid_angles, valid_ranges, 0, 0.26)
                left = self.get_min_distance(valid_angles, valid_ranges, np.pi/2, 0.26)
                right = self.get_min_distance(valid_angles, valid_ranges, 3*np.pi/2, 0.26)
                back = self.get_min_distance(valid_angles, valid_ranges, np.pi, 0.26)

                stats = f"{'='*30}\n"
                stats += f"  LIDAR MEASUREMENTS\n"
                stats += f"{'='*30}\n\n"
                stats += f"Points Detected: {len(valid_ranges)}/{len(ranges)}\n\n"
                stats += f"Overall:\n"
                stats += f"  Min:  {np.min(valid_ranges):.3f} m\n"
                stats += f"  Max:  {np.max(valid_ranges):.3f} m\n"
                stats += f"  Mean: {np.mean(valid_ranges):.3f} m\n\n"
                stats += f"{'─'*30}\n\n"
                stats += f"Directional Distances:\n\n"
                stats += f"  ↑ Forward: {forward:.3f} m\n"
                stats += f"  ← Left:    {left:.3f} m\n"
                stats += f"  → Right:   {right:.3f} m\n"
                stats += f"  ↓ Back:    {back:.3f} m\n\n"
                stats += f"{'─'*30}\n\n"

                # Warnings for close obstacles
                warnings = []
                if forward < 0.3:
                    warnings.append("⚠ FRONT OBSTACLE!")
                if left < 0.2:
                    warnings.append("⚠ LEFT WALL CLOSE!")
                if right < 0.2:
                    warnings.append("⚠ RIGHT WALL CLOSE!")

                if warnings:
                    stats += "WARNINGS:\n"
                    for w in warnings:
                        stats += f"  {w}\n"
            else:
                stats = "No valid measurements\n"

        self.stats_text.config(state=tk.NORMAL)
        self.stats_text.delete(1.0, tk.END)
        self.stats_text.insert(1.0, stats)
        self.stats_text.config(state=tk.DISABLED)

    def get_min_distance(self, angles, ranges, target_angle, tolerance):
        """Get minimum distance in a specific direction"""
        mask = np.abs(angles - target_angle) < tolerance
        if np.any(mask):
            return np.min(ranges[mask])
        return float('inf')

    def update_visualization(self):
        """Main update loop"""
        if not self.paused:
            # Clear canvas
            self.canvas.delete('all')

            # Draw grid
            self.draw_grid()

            width = self.canvas.winfo_width()
            height = self.canvas.winfo_height()
            center_x = width // 2
            center_y = height // 2

            # Draw LIDAR points
            if self.node and self.node.ranges is not None:
                ranges = self.node.ranges
                angles = self.node.angles

                # Update status
                self.status_indicator.itemconfig(self.status_circle, fill='#00ff00')
                self.status_label.config(text='Active', fg='#00ff00')

                # Filter and convert to canvas coordinates
                valid_mask = (ranges >= self.node.range_min) & (ranges <= self.node.range_max)
                valid_ranges = ranges[valid_mask]
                valid_angles = angles[valid_mask]

                for angle, dist in zip(valid_angles, valid_ranges):
                    # Convert polar to Cartesian (Y forward, X left/right)
                    # Negate X to flip left/right (sin gives right as positive, we want left as positive)
                    x = -dist * np.sin(angle) * self.scale  # Negative: left is positive X on display
                    y = -dist * np.cos(angle) * self.scale  # Negative because canvas Y increases downward

                    # Color based on distance (green=far, red=close)
                    color_intensity = min(255, int(255 * dist / self.range_max))
                    color = f'#{255-color_intensity:02x}{color_intensity:02x}00'

                    # Draw point
                    size = 3
                    self.canvas.create_oval(
                        center_x + x - size, center_y + y - size,
                        center_x + x + size, center_y + y + size,
                        fill=color, outline=''
                    )

            # Draw robot
            self.draw_robot(center_x, center_y)

            # Update statistics
            self.update_statistics()

        # Schedule next update
        self.root.after(50, self.update_visualization)

    def cleanup(self):
        """Cleanup on exit"""
        self.running = False
        if self.ros_thread:
            self.ros_thread.join(timeout=1.0)


class LidarNode(Node):
    """ROS2 node for LIDAR data"""

    def __init__(self):
        super().__init__('lidar_gui_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10
        )

        self.ranges = None
        self.angles = None
        self.range_min = 0.05
        self.range_max = 3.0

    def lidar_callback(self, msg):
        """Process LIDAR data"""
        self.ranges = np.array(msg.ranges)
        self.range_min = msg.range_min
        self.range_max = msg.range_max

        # Calculate angles
        num_points = len(self.ranges)
        self.angles = np.linspace(msg.angle_min, msg.angle_max, num_points)

        # Replace inf with max range
        self.ranges[np.isinf(self.ranges)] = self.range_max


def main():
    """Main entry point"""
    root = tk.Tk()
    app = LidarGUI(root)

    def on_closing():
        app.cleanup()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        on_closing()


if __name__ == '__main__':
    main()
