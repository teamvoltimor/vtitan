#!/bin/bash
#
# Automated Training Data Generation Pipeline
#
# Generates scenarios, launches Gazebo, records camera data,
# extracts frames, and creates YOLO dataset.
#
# Usage:
#   ./automated_pipeline.sh --challenge open --num-scenarios 50
#

set -e  # Exit on error

# Default parameters
CHALLENGE="open"
NUM_SCENARIOS=10
DURATION=30
OUTPUT_DIR="$HOME/wro_training_data"
RANDOMIZE="--randomize-all"
FRAME_SKIP=10
HEADLESS=false

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --challenge)
      CHALLENGE="$2"
      shift 2
      ;;
    --num-scenarios)
      NUM_SCENARIOS="$2"
      shift 2
      ;;
    --duration)
      DURATION="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --no-randomize)
      RANDOMIZE=""
      shift
      ;;
    --frame-skip)
      FRAME_SKIP="$2"
      shift 2
      ;;
    --headless)
      HEADLESS=true
      shift
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

echo "=========================================="
echo "WRO Training Data Generation Pipeline"
echo "=========================================="
echo "Challenge:      $CHALLENGE"
echo "Scenarios:      $NUM_SCENARIOS"
echo "Duration:       ${DURATION}s per scenario"
echo "Output:         $OUTPUT_DIR"
echo "Randomization:  $([ -n "$RANDOMIZE" ] && echo 'ENABLED' || echo 'DISABLED')"
echo "Headless:       $HEADLESS"
echo "=========================================="
echo ""

# Step 1: Generate scenarios
echo "[1/5] Generating scenarios..."
python3 generate_training_data.py \
    --challenge "$CHALLENGE" \
    --num-scenarios "$NUM_SCENARIOS" \
    $RANDOMIZE \
    --output-dir "$OUTPUT_DIR"

if [ $? -ne 0 ]; then
    echo "ERROR: Scenario generation failed"
    exit 1
fi

echo "✓ Scenarios generated"
echo ""

# Step 2: Setup ROS2 environment
echo "[2/5] Setting up ROS2 environment..."
source /opt/ros/kilted/setup.bash

# Use Zenoh middleware
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Check if workspace exists
if [ -d "$HOME/wro_ws/install" ]; then
    source "$HOME/wro_ws/install/setup.bash"
    echo "✓ ROS2 workspace sourced"
else
    echo "⚠ ROS2 workspace not found at ~/wro_ws"
    echo "  Continuing with system ROS2..."
fi

# Set Gazebo model path
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$(pwd)/../models"

# Start Zenoh router
echo "  Starting Zenoh router..."
ros2 run rmw_zenoh_cpp rmw_zenohd &
ZENOH_PID=$!
sleep 2
echo "  ✓ Zenoh router started (PID: $ZENOH_PID)"

echo ""

# Step 3: Launch scenarios and record
echo "[3/5] Launching scenarios and recording..."

BAGS_DIR="$OUTPUT_DIR/bags"
mkdir -p "$BAGS_DIR"

for (( i=0; i<NUM_SCENARIOS; i++ )); do
    SCENARIO_ID=$(printf "%04d" $i)
    WORLD_FILE="$OUTPUT_DIR/scenarios/scenario_${SCENARIO_ID}.sdf"
    BAG_FILE="$BAGS_DIR/scenario_${SCENARIO_ID}"

    echo ""
    echo "Recording scenario $((i+1))/$NUM_SCENARIOS (ID: $SCENARIO_ID)..."

    # Launch Gazebo
    if [ "$HEADLESS" = true ]; then
        gz sim -s -r "$WORLD_FILE" &
    else
        gz sim -r "$WORLD_FILE" &
    fi

    GZ_PID=$!
    echo "  Gazebo PID: $GZ_PID"

    # Wait for Gazebo to start
    sleep 5

    # Check if robot URDF exists
    URDF_FILE="../urdf/wro_robot.urdf.xacro"

    if [ -f "$URDF_FILE" ]; then
        # Spawn robot
        ros2 run ros_gz_sim create \
            -name wro_robot \
            -file "$URDF_FILE" \
            -x 0.0 -y -1.2 -z 0.1 -Y 1.5708 &

        SPAWN_PID=$!
        sleep 3
    else
        echo "  ⚠ Robot URDF not found, skipping spawn"
    fi

    # Start ROS2 bag recording
    ros2 bag record \
        /wro_robot/camera/image_raw \
        /wro_robot/camera/camera_info \
        /wro_robot/scan \
        /wro_robot/odom \
        -o "$BAG_FILE" &

    BAG_PID=$!
    echo "  Recording bag PID: $BAG_PID"

    # Start teleop (optional - user can drive manually)
    echo "  You can control the robot with:"
    echo "    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/wro_robot/cmd_vel"

    # Record for specified duration
    sleep "$DURATION"

    # Stop recording
    kill $BAG_PID 2>/dev/null || true
    sleep 1

    # Stop Gazebo
    kill $GZ_PID 2>/dev/null || true
    sleep 2

    # Cleanup any remaining processes
    pkill -f "gz sim" || true

    echo "  ✓ Scenario $SCENARIO_ID recorded"
done

echo ""
echo "✓ All scenarios recorded"
echo ""

# Step 4: Extract frames from bags
echo "[4/5] Extracting frames from ROS2 bags..."

FRAMES_DIR="$OUTPUT_DIR/frames"
mkdir -p "$FRAMES_DIR"

python3 extract_frames_and_annotate.py \
    --bag-dir "$BAGS_DIR" \
    --metadata-dir "$OUTPUT_DIR/scenarios" \
    --frames-dir "$FRAMES_DIR" \
    --output-dir "$OUTPUT_DIR/yolo_dataset" \
    --frame-skip "$FRAME_SKIP"

if [ $? -ne 0 ]; then
    echo "ERROR: Frame extraction failed"
    exit 1
fi

echo "✓ Frames extracted"
echo ""

# Step 5: Create YOLO dataset
echo "[5/5] Creating YOLO dataset..."

# Dataset already created by extract_frames_and_annotate.py
YOLO_DATASET="$OUTPUT_DIR/yolo_dataset"

# Cleanup Zenoh router
echo "Stopping Zenoh router..."
kill $ZENOH_PID 2>/dev/null || true

echo ""
echo "=========================================="
echo "✅ Pipeline Complete!"
echo "=========================================="
echo ""
echo "Generated Data:"
echo "  Scenarios:    $OUTPUT_DIR/scenarios/ ($NUM_SCENARIOS files)"
echo "  ROS2 Bags:    $OUTPUT_DIR/bags/ ($NUM_SCENARIOS bags)"
echo "  Frames:       $OUTPUT_DIR/frames/"
echo "  YOLO Dataset: $OUTPUT_DIR/yolo_dataset/"
echo ""
echo "Next Steps:"
echo ""
echo "1. Review dataset:"
echo "   ls -lh $YOLO_DATASET/images/train/"
echo ""
echo "2. Train YOLO model:"
echo "   yolo task=detect mode=train \\"
echo "     model=yolo26n.pt \\"
echo "     data=$YOLO_DATASET/data.yaml \\"
echo "     epochs=100 \\"
echo "     imgsz=640 \\"
echo "     batch=16"
echo ""
echo "3. Validate model:"
echo "   yolo task=detect mode=val \\"
echo "     model=runs/detect/train/weights/best.pt \\"
echo "     data=$YOLO_DATASET/data.yaml"
echo ""
echo "4. Test on real robot!"
echo ""
echo "=========================================="
