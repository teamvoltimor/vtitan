#!/bin/bash
# Debug script to check camera topics

echo "================================"
echo "Camera Topic Debugging"
echo "================================"
echo ""

# Step 1: Launch Gazebo with scenario
echo "Step 1: Launching Gazebo..."
cd "$(dirname "$0")"
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/../models

gz sim -r -s training_data/open/scenarios/scenario_0000.sdf &
GZ_PID=$!
sleep 5

echo "✓ Gazebo launched (PID: $GZ_PID)"
echo ""

# Step 2: Check Gazebo topics
echo "Step 2: Checking Gazebo topics..."
echo "Available Gazebo topics:"
gz topic -l | grep -i camera
echo ""

# Step 3: Check if camera is publishing
echo "Step 3: Checking if camera is publishing..."
timeout 2s gz topic -e -t /camera/image_raw 2>&1 | head -5
if [ $? -eq 124 ]; then
    echo "✓ Camera is publishing to /camera/image_raw"
else
    echo "✗ Camera NOT publishing to /camera/image_raw"
fi
echo ""

# Step 4: Launch bridge
echo "Step 4: Launching ros_gz_bridge..."
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image --ros-args --log-level error &
BRIDGE_PID=$!
sleep 3
echo "✓ Bridge launched (PID: $BRIDGE_PID)"
echo ""

# Step 5: Check ROS2 topics
echo "Step 5: Checking ROS2 topics..."
echo "Available ROS2 topics:"
ros2 topic list | grep -i camera
echo ""

# Step 6: Check if ROS2 topic is receiving data
echo "Step 6: Checking if ROS2 topic is receiving data..."
timeout 3s ros2 topic echo /camera/image_raw --once 2>&1 | head -10
if [ $? -eq 0 ]; then
    echo "✓ ROS2 topic IS receiving data!"
else
    echo "✗ ROS2 topic NOT receiving data"
fi
echo ""

# Cleanup
echo "Cleaning up..."
kill $BRIDGE_PID 2>/dev/null
kill $GZ_PID 2>/dev/null
killall -9 gz 2>/dev/null
sleep 2

echo ""
echo "================================"
echo "Debug complete"
echo "================================"
