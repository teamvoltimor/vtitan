# ROS2 Quick Reference - Essential Commands

A cheat sheet for daily development with ROS2 Humble.

---

## Workspace Management

### Build Workspace
```bash
cd ~/teamsteelbot_ws
colcon build                           # Build all packages
colcon build --packages-select pkg     # Build specific package
colcon build --symlink-install         # Symlink Python files (no rebuild needed)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release  # Release build (faster)
```

### Source Workspace
```bash
source /opt/ros/humble/setup.bash      # Source ROS2
source ~/teamsteelbot_ws/install/setup.bash  # Source your workspace
# Or add to ~/.bashrc to auto-source
```

### Clean Build
```bash
rm -rf build/ install/ log/            # Delete build artifacts
colcon build                           # Rebuild from scratch
```

---

## Running Nodes

### Run Single Node
```bash
ros2 run <package_name> <executable_name>

# Examples:
ros2 run teamsteelbot_simulation mock_camera_node
ros2 run teamsteelbot_vision sign_detector_classic
ros2 run teamsteelbot_control decision_node
```

### Run with Parameters
```bash
ros2 run pkg node --ros-args -p param_name:=value

# Example:
ros2 run teamsteelbot_simulation mock_camera_node --ros-args -p fps:=60
```

### Launch Multiple Nodes
```bash
ros2 launch <package_name> <launch_file>

# Example:
ros2 launch teamsteelbot_bringup simulation.launch.py
```

---

## Topics (Data Streams)

### List Topics
```bash
ros2 topic list                        # Show all topics
ros2 topic list -t                     # Include message types
```

### Echo Topic (View Data)
```bash
ros2 topic echo /camera/image_raw      # Show raw data
ros2 topic echo /detections            # Show detection messages
ros2 topic echo /cmd_vel               # Show velocity commands
```

### Topic Info
```bash
ros2 topic info /camera/image_raw      # Publisher/subscriber count, type
ros2 topic hz /camera/image_raw        # Measure publishing rate
ros2 topic bw /camera/image_raw        # Measure bandwidth
```

### Publish to Topic (Manual)
```bash
# Publish geometry_msgs/Twist (velocity command)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# One-shot publish (--once)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}}"
```

---

## Nodes (Running Processes)

### List Nodes
```bash
ros2 node list                         # Show all running nodes
```

### Node Info
```bash
ros2 node info /sign_detector          # Show subscribers, publishers, services
```

---

## Services (Request/Response)

### List Services
```bash
ros2 service list                      # Show all services
ros2 service list -t                   # Include service types
```

### Call Service
```bash
ros2 service call /service_name package_name/srv/ServiceType "{field: value}"

# Example: Trigger service
ros2 service call /emergency_stop std_srvs/srv/Trigger
```

---

## Parameters (Configuration)

### List Parameters
```bash
ros2 param list                        # All parameters from all nodes
ros2 param list /sign_detector         # Parameters for specific node
```

### Get Parameter
```bash
ros2 param get /sign_detector min_area
```

### Set Parameter
```bash
ros2 param set /sign_detector min_area 1500
```

### Save Parameters to File
```bash
ros2 param dump /sign_detector > params.yaml
```

### Load Parameters from File
```bash
ros2 run pkg node --ros-args --params-file params.yaml
```

---

## ROS Bags (Record/Replay)

### Record Topics
```bash
ros2 bag record -a                     # Record ALL topics
ros2 bag record /camera/image_raw /detections  # Specific topics
ros2 bag record -a -o my_test_run      # Output to specific file
```

### Bag Info
```bash
ros2 bag info my_test_run_0.db3        # Show topics, duration, size
```

### Play Back Bag
```bash
ros2 bag play my_test_run_0.db3        # Replay at normal speed
ros2 bag play -r 0.5 bag_file          # Play at 0.5x speed (slow motion)
ros2 bag play -l bag_file              # Loop playback
```

---

## Visualization

### RViz2 (3D Visualization)
```bash
rviz2                                  # Launch RViz2
rviz2 -d config.rviz                   # Load saved configuration
```

**Common displays to add:**
- Image: View camera feed
- LaserScan: View LiDAR data
- TF: View coordinate frames
- Marker: View custom visualizations

### rqt_graph (Node Graph)
```bash
rqt_graph                              # Visualize node connections
```

### PlotJuggler (Time-Series Plots)
```bash
ros2 run plotjuggler plotjuggler       # Launch PlotJuggler
# File > Load ROS2 bag > Select bag file
# Drag topics to plot area
```

### rqt_image_view (Camera Viewer)
```bash
ros2 run rqt_image_view rqt_image_view
# Select topic from dropdown
```

---

## Debugging

### Enable Debug Logging
```bash
ros2 run pkg node --ros-args --log-level DEBUG
```

### Check Node Health
```bash
ros2 node info /node_name              # See if node is publishing/subscribing
ros2 topic hz /topic_name              # Verify publishing rate
ros2 topic echo /topic_name            # Verify data is correct
```

### Common Issues

**"Package not found"**
```bash
# Did you source the workspace?
source ~/teamsteelbot_ws/install/setup.bash

# Did you build?
colcon build
```

**"Topic not updating"**
```bash
# Check if node is running
ros2 node list

# Check publishing rate
ros2 topic hz /topic_name

# Check QoS compatibility
ros2 topic info /topic_name -v
```

**"Python module not found"**
```bash
# Did you add to setup.py?
# Rebuild with --symlink-install
colcon build --symlink-install
```

---

## Testing

### Run Tests
```bash
colcon test                            # Run all tests
colcon test --packages-select pkg      # Test specific package
colcon test-result --verbose           # Show test results
```

### pytest (Python Unit Tests)
```bash
pytest src/pkg_name/test/              # Run pytest directly
pytest -v                              # Verbose output
pytest --cov                           # With coverage
```

---

## Performance Profiling

### CPU Usage
```bash
top                                    # General system monitor
htop                                   # Better interface (install: sudo apt install htop)
ros2 run ros2trace trace               # ROS2 tracing (advanced)
```

### Memory Usage
```bash
ros2 node info /node_name              # Basic info
free -h                                # System memory
```

### Network Bandwidth
```bash
ros2 topic bw /topic_name              # Bandwidth per topic
iftop                                  # Network monitor (install: sudo apt install iftop)
```

---

## Message Types

### List Available Messages
```bash
ros2 interface list                    # All message types
ros2 interface list | grep sensor      # Filter by keyword
```

### Show Message Definition
```bash
ros2 interface show sensor_msgs/msg/Image
ros2 interface show geometry_msgs/msg/Twist
ros2 interface show vision_msgs/msg/Detection2D
```

### Create Custom Message
1. Create `.msg` file in `msg/` directory
2. Update `CMakeLists.txt` (for ament_cmake packages)
3. Update `package.xml` (add dependencies)
4. Build: `colcon build`

---

## Common Message Types

### Images
```bash
sensor_msgs/msg/Image                  # Raw image data
sensor_msgs/msg/CompressedImage        # JPEG compressed
```

### Velocity
```bash
geometry_msgs/msg/Twist                # Linear + angular velocity
# Fields: linear.{x,y,z}, angular.{x,y,z}
```

### Detections
```bash
vision_msgs/msg/Detection2D            # Single detection
vision_msgs/msg/Detection2DArray       # Multiple detections
```

### Laser Scan
```bash
sensor_msgs/msg/LaserScan              # LiDAR data
```

### IMU
```bash
sensor_msgs/msg/Imu                    # Orientation, accel, gyro
```

### Odometry
```bash
nav_msgs/msg/Odometry                  # Position + velocity
```

---

## Useful Aliases (Add to ~/.bashrc)

```bash
# Add these to ~/.bashrc for faster workflow
alias cb='cd ~/teamsteelbot_ws && colcon build --symlink-install'
alias cs='source ~/teamsteelbot_ws/install/setup.bash'
alias cbs='cb && cs'  # Build and source
alias ct='colcon test && colcon test-result --verbose'

alias tl='ros2 topic list'
alias te='ros2 topic echo'
alias nl='ros2 node list'

alias kill_ros='killall -9 ros2 rviz2 gazebo'  # Emergency kill

# ROS2 domain ID (to isolate your robot from others)
export ROS_DOMAIN_ID=42
```

After adding, reload:
```bash
source ~/.bashrc
```

---

## Simulation Tips

### Gazebo
```bash
gz sim empty.sdf                       # Empty world
gz sim -v4                             # Verbose output
gz sim -s                              # Server only (no GUI, faster)
```

### Mock Sensors
When developing, prefer simple mock nodes over full Gazebo simulation:
- **Faster iteration** (no physics simulation overhead)
- **Easier debugging** (pure Python, no plugins)
- **Deterministic** (no random physics behavior)

Switch to Gazebo later for realistic testing.

---

## Git Workflow (Recommended)

```bash
# Create feature branch
git checkout -b feature/vision-detector

# Make changes, test
# ...

# Commit
git add .
git commit -m "Add HSV color detection"

# Push
git push origin feature/vision-detector

# Merge to main
git checkout main
git merge feature/vision-detector

# Delete branch
git branch -d feature/vision-detector
```

---

## Daily Development Workflow

```bash
# Morning: Start fresh
cd ~/teamsteelbot_ws
git pull
colcon build --symlink-install
source install/setup.bash

# Develop: Edit Python files (no rebuild needed with --symlink-install)

# Test individual node
ros2 run teamsteelbot_vision sign_detector_classic

# Test full system
ros2 launch teamsteelbot_bringup simulation.launch.py

# Debug
ros2 topic echo /detections
ros2 node info /sign_detector

# Record test run
ros2 bag record -a -o test_$(date +%Y%m%d_%H%M%S)

# Evening: Commit progress
git add .
git commit -m "Improved detection accuracy"
git push
```

---

## Performance Targets

Monitor these metrics to ensure your system is performant:

| Metric | Target | Command |
|--------|--------|---------|
| Camera FPS | 30 Hz | `ros2 topic hz /camera/image_raw` |
| Detection FPS | 30 Hz | `ros2 topic hz /detections` |
| Control loop | 50 Hz | `ros2 topic hz /cmd_vel` |
| Latency (camera→cmd) | <50 ms | Measure with timestamps |

---

## Resources

- **ROS2 Docs:** https://docs.ros.org/en/humble/
- **ROS2 Tutorials:** https://docs.ros.org/en/humble/Tutorials.html
- **ROS Answers:** https://answers.ros.org/
- **ROS Discord:** https://discord.gg/ros

---

## Emergency Commands

```bash
# Kill all ROS nodes
killall -9 ros2

# Kill Gazebo
killall -9 gz

# Kill RViz
killall -9 rviz2

# Reset ROS daemon
ros2 daemon stop
ros2 daemon start

# Clear build artifacts
cd ~/teamsteelbot_ws
rm -rf build/ install/ log/
```

---

**Print this and keep it nearby! 📋**
