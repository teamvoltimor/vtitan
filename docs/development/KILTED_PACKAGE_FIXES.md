# ROS2 Kilted Package Name Fixes

## Issue: Package Not Found Errors

If you're getting errors like `E: Unable to locate package ros-kilted-gazebo-ros-pkgs`, it's because ROS2 Kilted uses **different package names** than Humble.

---

## Package Name Changes

### Gazebo Packages

**Old (Humble):**
```bash
ros-humble-gazebo-ros-pkgs
```

**New (Kilted):**
```bash
ros-kilted-ros-gz  # This is the correct package!
```

**Why the change?**
- Kilted uses Gazebo Ionic (not Garden)
- Gazebo Ionic uses vendor packages in ROS2
- New naming convention: `ros-gz` instead of `gazebo-ros-pkgs`

---

## Quick Fix

If you already ran the old script and got errors:

```bash
# Install the correct Gazebo package for Kilted
sudo apt install ros-kilted-ros-gz -y

# Verify installation
ros2 pkg list | grep gz

# Should show packages like:
# ros_gz
# ros_gz_bridge
# ros_gz_image
# ros_gz_interfaces
# ros_gz_sim
```

---

## Complete Correct Package List for Kilted

### Core ROS2 Kilted
```bash
sudo apt install ros-kilted-desktop -y
sudo apt install ros-dev-tools -y
sudo apt install python3-colcon-common-extensions -y
```

### Simulation (Gazebo Ionic)
```bash
sudo apt install ros-kilted-ros-gz -y
```

### Navigation
```bash
sudo apt install ros-kilted-navigation2 -y
sudo apt install ros-kilted-nav2-bringup -y
sudo apt install ros-kilted-robot-localization -y
```

### Vision
```bash
sudo apt install ros-kilted-vision-msgs -y
sudo apt install ros-kilted-image-transport -y
sudo apt install ros-kilted-cv-bridge -y
```

### Visualization
```bash
sudo apt install ros-kilted-rqt -y
sudo apt install ros-kilted-plotjuggler-ros -y
```

### Sensors
```bash
sudo apt install ros-kilted-laser-geometry -y
```

---

## Verify Your Installation

```bash
# Check ROS2 version
ros2 --version
# Should show: ros2 cli version: 0.33.0 or later

# List all installed ROS2 packages
ros2 pkg list

# Check Gazebo
gz sim --versions
# Should show Gazebo Ionic version

# Test Gazebo launch
gz sim empty.sdf
# Should open Gazebo Ionic (close with Ctrl+C)
```

---

## Common Package Naming Patterns

| Old Name (Humble) | New Name (Kilted) | Notes |
|-------------------|-------------------|-------|
| `gazebo-ros-pkgs` | `ros-gz` | Major change! |
| `humble` | `kilted` | Distribution name |
| Same | Same | Most other packages unchanged |

---

## If You Still Get Errors

### Error: "Unable to locate package ros-kilted-XXX"

**Check 1: Is your Ubuntu version correct?**
```bash
lsb_release -a
# Should show: Ubuntu 24.04 LTS (Noble Numbat)
```

Kilted **requires** Ubuntu 24.04. If you're on 22.04, you must upgrade or use Humble.

**Check 2: Is the ROS2 repository added correctly?**
```bash
cat /etc/apt/sources.list.d/ros2.list
# Should show: deb [arch=amd64] ... noble main
```

The important part is `noble` (Ubuntu 24.04 codename).

**Check 3: Update package lists**
```bash
sudo apt update
```

### Error: Gazebo doesn't launch

**Solution 1: Check Gazebo installation**
```bash
# Check if Gazebo is installed
which gz
# Should show: /usr/bin/gz

# Check Gazebo version
gz --version
# Should show Gazebo Ionic
```

**Solution 2: Install missing Gazebo components**
```bash
# Install full Gazebo Ionic
sudo apt install ros-kilted-ros-gz -y

# If still issues, install Gazebo Ionic directly
sudo apt install gz-ionic -y
```

---

## Updated Setup Script

The setup script has been fixed with the correct package names. If you've already run it and got errors:

```bash
# Re-run just the Gazebo installation part
sudo apt install ros-kilted-ros-gz -y

# Verify
ros2 pkg list | grep gz
```

---

## Testing Gazebo with ROS2 Kilted

```bash
# Terminal 1: Launch Gazebo
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf"

# Terminal 2: Check ROS2 topics
ros2 topic list
# Should show Gazebo-related topics

# Terminal 3: Test bridge
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

---

## Sources

- [Gazebo ROS Installation](https://gazebosim.org/docs/latest/ros_installation/)
- [ROS2 Kilted Gazebo Tutorial](https://docs.ros.org/en/kilted/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [ros_gz GitHub](https://github.com/gazebosim/ros_gz)

---

## Summary

**The fix:** Change `ros-kilted-gazebo-ros-pkgs` to `ros-kilted-ros-gz`

This is the **only** major package name change from Humble to Kilted. Everything else uses the same naming pattern (just replace `humble` with `kilted`).

The setup script has been updated with the correct package names. ✅
