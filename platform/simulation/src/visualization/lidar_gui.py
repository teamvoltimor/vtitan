#!/usr/bin/env python3
"""LIDAR GUI - Interactive visualization tool with controls"""

from __future__ import annotations

import threading
import tkinter as tk

import numpy as np
import rclpy  # type: ignore[import]
from rclpy.node import Node  # type: ignore[import]
from sensor_msgs.msg import LaserScan  # type: ignore[import]


class LidarGUI:
    """Interactive LIDAR visualization GUI"""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("WRO LIDAR Visualizer")
        self.root.geometry("1200x800")
        self.root.configure(bg="#0d1117")

        # Initialize ROS2 in background thread
        self.ros_thread: threading.Thread | None = None
        self.node: LidarNode | None = None
        self.running = False
        self.paused = False

        # LIDAR data
        self.ranges = None
        self.angles = None
        self.range_max = 3.0

        # Theme colors
        self.bg_color = "#0d1117"
        self.panel_bg = "#161b22"
        self.canvas_bg = "#0d1117"
        self.grid_color = "#1f3a5f"
        self.grid_label_color = "#3a7bd5"
        self.robot_color = "#e94560"
        self.accent_color = "#58a6ff"
        self.text_color = "#c9d1d9"
        self.text_muted = "#8b949e"
        self.btn_bg = "#21262d"
        self.btn_hover = "#30363d"
        self.border_color = "#30363d"
        self.warning_color = "#d29922"

        self.scale = 1.0
        self.setup_ui()
        self.start_ros()

    def setup_ui(self) -> None:
        """Set up the GUI layout"""
        # Main container
        main_frame = tk.Frame(self.root, bg=self.bg_color)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        # Left panel - Controls and stats
        left_panel = tk.Frame(
            main_frame,
            bg=self.panel_bg,
            width=300,
            highlightbackground=self.border_color,
            highlightthickness=1,
        )
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        left_panel.pack_propagate(False)

        # Inner padding for left panel
        left_inner = tk.Frame(left_panel, bg=self.panel_bg)
        left_inner.pack(fill=tk.BOTH, expand=True, padx=12, pady=12)

        # Title
        title = tk.Label(
            left_inner,
            text="LIDAR Control",
            font=("Consolas", 15, "bold"),
            bg=self.panel_bg,
            fg=self.accent_color,
        )
        title.pack(anchor=tk.W, pady=(0, 16))

        # Status indicator
        self.status_frame = tk.Frame(left_inner, bg=self.panel_bg)
        self.status_frame.pack(fill=tk.X, pady=(0, 12))

        tk.Label(
            self.status_frame,
            text="STATUS",
            font=("Consolas", 9),
            bg=self.panel_bg,
            fg=self.text_muted,
        ).pack(side=tk.LEFT, padx=(0, 8))

        self.status_indicator = tk.Canvas(
            self.status_frame, width=12, height=12, bg=self.panel_bg, highlightthickness=0
        )
        self.status_indicator.pack(side=tk.LEFT, padx=(0, 6))
        self.status_circle = self.status_indicator.create_oval(
            1, 1, 11, 11, fill=self.border_color
        )

        self.status_label = tk.Label(
            self.status_frame,
            text="Waiting",
            font=("Consolas", 10),
            bg=self.panel_bg,
            fg=self.text_muted,
        )
        self.status_label.pack(side=tk.LEFT)

        # Control buttons
        button_frame = tk.Frame(left_inner, bg=self.panel_bg)
        button_frame.pack(fill=tk.X, pady=(0, 16))

        self.pause_button = tk.Button(
            button_frame,
            text="PAUSE",
            command=self.toggle_pause,
            font=("Consolas", 10, "bold"),
            bg=self.btn_bg,
            fg=self.text_color,
            activebackground=self.btn_hover,
            activeforeground=self.text_color,
            relief=tk.FLAT,
            bd=0,
            pady=8,
            cursor="hand2",
        )
        self.pause_button.pack(fill=tk.X, pady=(0, 4))

        tk.Button(
            button_frame,
            text="RESET VIEW",
            command=self.reset_view,
            font=("Consolas", 10, "bold"),
            bg=self.btn_bg,
            fg=self.text_color,
            activebackground=self.btn_hover,
            activeforeground=self.text_color,
            relief=tk.FLAT,
            bd=0,
            pady=8,
            cursor="hand2",
        ).pack(fill=tk.X)

        # Divider
        tk.Frame(left_inner, bg=self.border_color, height=1).pack(fill=tk.X, pady=16)

        # Statistics header
        stats_label = tk.Label(
            left_inner,
            text="Distance Stats",
            font=("Consolas", 13, "bold"),
            bg=self.panel_bg,
            fg=self.accent_color,
        )
        stats_label.pack(anchor=tk.W, pady=(0, 8))

        # Stats text area
        stats_frame = tk.Frame(left_inner, bg=self.border_color, bd=1, relief=tk.FLAT)
        stats_frame.pack(fill=tk.BOTH, expand=True)

        self.stats_text = tk.Text(
            stats_frame,
            font=("Consolas", 9),
            bg=self.bg_color,
            fg=self.text_color,
            height=20,
            relief=tk.FLAT,
            padx=8,
            pady=8,
            insertbackground=self.accent_color,
            selectbackground=self.grid_color,
        )
        self.stats_text.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)
        self.stats_text.config(state=tk.DISABLED)

        # Right panel - Visualization canvas
        right_panel = tk.Frame(
            main_frame,
            bg=self.panel_bg,
            highlightbackground=self.border_color,
            highlightthickness=1,
        )
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        canvas_label = tk.Label(
            right_panel,
            text="Top-Down View",
            font=("Consolas", 11, "bold"),
            bg=self.panel_bg,
            fg=self.text_muted,
        )
        canvas_label.pack(pady=(8, 4))

        # Main visualization canvas
        self.canvas = tk.Canvas(right_panel, bg=self.canvas_bg, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        # Bind resize event
        self.canvas.bind("<Configure>", self.on_canvas_resize)

        # Start update loop
        self.update_visualization()

    def start_ros(self) -> None:
        """Start ROS2 node in background thread"""
        self.running = True

        def ros_spin() -> None:
            rclpy.init()
            self.node = LidarNode()
            while self.running:
                rclpy.spin_once(self.node, timeout_sec=0.01)
            self.node.destroy_node()
            rclpy.shutdown()

        self.ros_thread = threading.Thread(target=ros_spin, daemon=True)
        self.ros_thread.start()

    def toggle_pause(self) -> None:
        """Toggle pause state"""
        self.paused = not self.paused
        if self.paused:
            self.pause_button.config(text="RESUME")
        else:
            self.pause_button.config(text="PAUSE")

    def reset_view(self) -> None:
        """Reset visualization"""
        self.canvas.delete("all")
        self.draw_grid()

    def on_canvas_resize(self, event: tk.Event) -> None:
        """Handle canvas resize"""
        self.draw_grid()

    def draw_grid(self) -> None:
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
                center_x - radius,
                center_y - radius,
                center_x + radius,
                center_y + radius,
                outline=self.grid_color,
                width=1,
            )
            # Distance labels
            self.canvas.create_text(
                center_x,
                center_y - radius - 12,
                text=f"{dist}m",
                fill=self.grid_label_color,
                font=("Consolas", 8),
            )

        # Draw axes
        self.canvas.create_line(
            center_x,
            center_y - max_size // 2,
            center_x,
            center_y + max_size // 2,
            fill=self.grid_color,
            width=1,
            dash=(3, 3),
        )
        self.canvas.create_line(
            center_x - max_size // 2,
            center_y,
            center_x + max_size // 2,
            center_y,
            fill=self.grid_color,
            width=1,
            dash=(3, 3),
        )

        # Direction labels
        self.canvas.create_text(
            center_x,
            center_y - max_size // 2 + 20,
            text="FORWARD",
            fill=self.text_muted,
            font=("Consolas", 10, "bold"),
        )
        self.canvas.create_text(
            center_x + max_size // 2 - 40,
            center_y,
            text="RIGHT",
            fill=self.text_muted,
            font=("Consolas", 10, "bold"),
        )
        self.canvas.create_text(
            center_x - max_size // 2 + 30,
            center_y,
            text="LEFT",
            fill=self.text_muted,
            font=("Consolas", 10, "bold"),
        )
        self.canvas.create_text(
            center_x,
            center_y + max_size // 2 - 20,
            text="BACK",
            fill=self.text_muted,
            font=("Consolas", 10, "bold"),
        )

    def draw_robot(self, center_x: float, center_y: float) -> None:
        """Draw robot at center"""
        size = 18
        # Triangle pointing forward
        points = [
            center_x,
            center_y - size,  # Top point (forward)
            center_x - size,
            center_y + size,  # Bottom left
            center_x + size,
            center_y + size,  # Bottom right
        ]
        self.canvas.create_polygon(
            points, fill=self.robot_color, outline=self.accent_color, width=2
        )

    def update_statistics(self) -> None:
        """Update statistics display"""
        if self.node is None or self.node.ranges is None or self.node.angles is None:
            stats = "Waiting for LIDAR data...\n\n"
            stats += "Status: No data received\n"
            stats += "\nCheck:\n"
            stats += "  - Gazebo is running\n"
            stats += "  - ROS bridge is active\n"
            stats += "  - /lidar topic exists\n"
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
                left = self.get_min_distance(valid_angles, valid_ranges, np.pi / 2, 0.26)
                right = self.get_min_distance(valid_angles, valid_ranges, -np.pi / 2, 0.26)
                back = self.get_min_distance(valid_angles, valid_ranges, np.pi, 0.26)

                stats = f"{'=' * 28}\n"
                stats += "  LIDAR MEASUREMENTS\n"
                stats += f"{'=' * 28}\n\n"
                stats += f"Points: {len(valid_ranges)}/{len(ranges)}\n\n"
                stats += "Overall:\n"
                stats += f"  Min:  {np.min(valid_ranges):.3f} m\n"
                stats += f"  Max:  {np.max(valid_ranges):.3f} m\n"
                stats += f"  Mean: {np.mean(valid_ranges):.3f} m\n\n"
                stats += f"{'-' * 28}\n\n"
                stats += "Directional:\n\n"
                stats += f"  Forward: {forward:.3f} m\n"
                stats += f"  Left:    {left:.3f} m\n"
                stats += f"  Right:   {right:.3f} m\n"
                stats += f"  Back:    {back:.3f} m\n\n"
                stats += f"{'-' * 28}\n\n"

                # Warnings for close obstacles
                warnings = []
                if forward < 0.3:
                    warnings.append("[!] FRONT OBSTACLE")
                if left < 0.2:
                    warnings.append("[!] LEFT WALL CLOSE")
                if right < 0.2:
                    warnings.append("[!] RIGHT WALL CLOSE")

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

    def get_min_distance(
        self,
        angles: np.ndarray,
        ranges: np.ndarray,
        target_angle: float,
        tolerance: float,
    ) -> float:
        """Get minimum distance in a specific direction"""
        mask = np.abs(angles - target_angle) < tolerance
        if np.any(mask):
            return np.min(ranges[mask])
        return float("inf")

    def update_visualization(self) -> None:
        """Main update loop"""
        if not self.paused:
            # Clear canvas
            self.canvas.delete("all")

            # Draw grid
            self.draw_grid()

            width = self.canvas.winfo_width()
            height = self.canvas.winfo_height()
            center_x = width // 2
            center_y = height // 2

            # Draw LIDAR points
            if self.node and self.node.ranges is not None and self.node.angles is not None:
                ranges = self.node.ranges
                angles = self.node.angles

                # Update status
                self.status_indicator.itemconfig(self.status_circle, fill="#3fb950")
                self.status_label.config(text="Active", fg="#3fb950")

                # Filter and convert to canvas coordinates
                valid_mask = (ranges >= self.node.range_min) & (ranges <= self.node.range_max)
                valid_ranges = ranges[valid_mask]
                valid_angles = angles[valid_mask]

                for angle, dist in zip(valid_angles, valid_ranges, strict=True):
                    # Convert polar to Cartesian (Y forward, X left/right)
                    # Negate X so left remains positive when sin returns right as positive
                    x = -dist * np.sin(angle) * self.scale
                    y = -dist * np.cos(angle) * self.scale

                    # Color based on distance: cyan (far) -> yellow (mid) -> red (close)
                    ratio = min(1.0, dist / self.range_max)
                    if ratio > 0.5:
                        # Cyan to yellow
                        t = (ratio - 0.5) * 2
                        r = int(255 * (1 - t))
                        g = int(255 * (1 - t * 0.2))
                        b = int(255 * t)
                    else:
                        # Yellow to red
                        t = ratio * 2
                        r = 255
                        g = int(255 * t)
                        b = 0
                    color = f"#{r:02x}{g:02x}{b:02x}"

                    # Draw point
                    size = 3
                    self.canvas.create_oval(
                        center_x + x - size,
                        center_y + y - size,
                        center_x + x + size,
                        center_y + y + size,
                        fill=color,
                        outline="",
                    )

            # Draw robot
            self.draw_robot(center_x, center_y)

            # Update statistics
            self.update_statistics()

        # Schedule next update
        self.root.after(50, self.update_visualization)

    def cleanup(self) -> None:
        """Cleanup on exit"""
        self.running = False
        if self.ros_thread:
            self.ros_thread.join(timeout=1.0)


class LidarNode(Node):
    """ROS2 node for LIDAR data"""

    def __init__(self) -> None:
        super().__init__("lidar_gui_node")

        self.subscription = self.create_subscription(
            LaserScan,
            "/lidar",
            self.lidar_callback,
            10,
        )

        self.ranges = None
        self.angles = None
        self.range_min = 0.05
        self.range_max = 3.0

    def lidar_callback(self, msg: LaserScan) -> None:
        """Process LIDAR data"""
        self.ranges = np.array(msg.ranges)
        self.range_min = msg.range_min
        self.range_max = msg.range_max

        # Calculate angles
        num_points = len(self.ranges)
        self.angles = np.linspace(msg.angle_min, msg.angle_max, num_points)

        # Replace inf with max range
        self.ranges[np.isinf(self.ranges)] = self.range_max


def main() -> None:
    """Main entry point"""
    root = tk.Tk()
    app = LidarGUI(root)

    def on_closing() -> None:
        app.cleanup()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        on_closing()


if __name__ == "__main__":
    main()
