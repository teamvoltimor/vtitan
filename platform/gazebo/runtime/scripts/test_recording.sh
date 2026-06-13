#!/bin/bash
# Quick test script for video recording pipeline

echo "=========================================="
echo "WRO Video Recording Pipeline - Quick Test"
echo "=========================================="
echo ""

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo "❌ ERROR: ROS2 not found"
    echo "   Please install ROS2 Humble or Jazzy"
    echo "   sudo apt install ros-humble-desktop"
    exit 1
fi

echo "✓ ROS2 found"

# Check if Gazebo is available
if ! command -v gz &> /dev/null; then
    echo "❌ ERROR: Gazebo not found"
    echo "   Please install Gazebo Harmonic"
    echo "   sudo apt install gz-harmonic"
    exit 1
fi

echo "✓ Gazebo found"

# Check Python dependencies
echo ""
echo "Checking Python dependencies..."

python3 -c "import cv2" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ ERROR: OpenCV not found"
    echo "   pip3 install opencv-python"
    exit 1
fi
echo "✓ OpenCV found"

python3 -c "import numpy" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ ERROR: NumPy not found"
    echo "   pip3 install numpy"
    exit 1
fi
echo "✓ NumPy found"

python3 -c "from cv_bridge import CvBridge" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ ERROR: cv_bridge not found"
    echo "   sudo apt install ros-humble-cv-bridge"
    exit 1
fi
echo "✓ cv_bridge found"

# All checks passed
echo ""
echo "=========================================="
echo "All dependencies satisfied! ✓"
echo "=========================================="
echo ""

# Offer to run test
read -p "Run test recording? (1 scenario, 15 seconds) [y/N]: " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "Starting test recording..."
    echo "This will:"
    echo "  1. Generate 1 test scenario"
    echo "  2. Launch Gazebo (headless)"
    echo "  3. Record 15 seconds of video"
    echo ""

    # Create test directory
    TEST_DIR="./test_recording_output"
    mkdir -p "$TEST_DIR"

    # Run pipeline
    python3 record_scenario_videos.py \
        --challenge open \
        --num-scenarios 1 \
        --duration 15 \
        --output-dir "$TEST_DIR"

    # Check results
    if [ -f "$TEST_DIR/open/videos/scenario_0000.mp4" ]; then
        echo ""
        echo "=========================================="
        echo "✓ Test recording successful!"
        echo "=========================================="
        echo ""
        echo "Output files:"
        echo "  Scenario: $TEST_DIR/open/scenarios/scenario_0000.sdf"
        echo "  Video:    $TEST_DIR/open/videos/scenario_0000.mp4"
        echo ""
        echo "Play video with:"
        echo "  vlc $TEST_DIR/open/videos/scenario_0000.mp4"
        echo ""
    else
        echo ""
        echo "❌ Test recording failed"
        echo "   Check the logs above for errors"
        echo ""
    fi
else
    echo ""
    echo "Test skipped. To run manually:"
    echo ""
    echo "  python3 record_scenario_videos.py --challenge open --num-scenarios 1 --duration 15"
    echo ""
fi

echo "For full documentation, see:"
echo "  platform/docs/VIDEO_RECORDING_GUIDE.md"
echo ""
