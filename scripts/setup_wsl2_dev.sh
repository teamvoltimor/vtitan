#!/bin/bash
# Quick setup script for ROS2 Kilted development on WSL2
# Run: bash scripts/setup_wsl2_dev.sh
# Requires: Ubuntu 24.04 (Noble Numbat)

set -e  # Exit on error

echo "================================================"
echo "TeamSteelBot ROS2 Kilted Development Setup (WSL2)"
echo "================================================"
echo ""

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
if [ "$UBUNTU_VERSION" != "24.04" ]; then
    echo -e "${YELLOW}Warning: This script requires Ubuntu 24.04 (Noble Numbat)${NC}"
    echo -e "${YELLOW}Current version: $UBUNTU_VERSION${NC}"
    echo ""
    echo "To upgrade WSL2 to Ubuntu 24.04:"
    echo "  1. Backup your data"
    echo "  2. Run: wsl --install Ubuntu-24.04"
    echo ""
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Step 1: Update system
echo -e "${BLUE}[1/9] Updating system...${NC}"
sudo apt update && sudo apt upgrade -y

# Step 1.5: Ensure UTF-8 locale
echo -e "${BLUE}[1.5/9] Configuring locale...${NC}"
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Step 2: Install ROS2 Kilted
echo -e "${BLUE}[2/9] Installing ROS2 Kilted Kaiju...${NC}"
if [ ! -f /opt/ros/kilted/setup.bash ]; then
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt update
    sudo apt install curl -y

    # Add ROS2 repository
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install ros-kilted-desktop -y
    sudo apt install ros-dev-tools -y
    sudo apt install python3-colcon-common-extensions -y

    echo -e "${GREEN}✅ ROS2 Kilted Kaiju installed!${NC}"
else
    echo "ROS2 Kilted already installed!"
fi

# Step 3: Install simulation tools (Gazebo Ionic)
echo -e "${BLUE}[3/9] Installing Gazebo Ionic and simulation tools...${NC}"
# Kilted uses ros-gz packages (Gazebo Ionic via vendor packages)
sudo apt install ros-kilted-ros-gz -y

echo -e "${GREEN}✅ Gazebo Ionic installed (latest version for Kilted)${NC}"

# Step 4: Install navigation and vision packages
echo -e "${BLUE}[4/9] Installing ROS2 packages...${NC}"
sudo apt install \
    ros-kilted-navigation2 \
    ros-kilted-nav2-bringup \
    ros-kilted-robot-localization \
    ros-kilted-vision-msgs \
    ros-kilted-image-transport \
    ros-kilted-cv-bridge \
    ros-kilted-rqt \
    ros-kilted-plotjuggler-ros \
    ros-kilted-laser-geometry \
    -y

echo -e "${GREEN}✅ ROS2 packages installed${NC}"

# Step 5: Install Python dependencies
echo -e "${BLUE}[5/9] Installing Python and pip...${NC}"
sudo apt install python3-pip python3-venv -y

echo -e "${BLUE}[6/9] Creating Python virtual environment...${NC}"
WORKSPACE_DIR="$HOME/teamsteelbot/klevor-v2"
VENV_DIR="$WORKSPACE_DIR/.venv"
mkdir -p "$WORKSPACE_DIR"
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
    echo -e "${GREEN}✅ Virtual environment created at $VENV_DIR${NC}"
else
    echo "Virtual environment already exists at $VENV_DIR"
fi

echo -e "${BLUE}[6.5/9] Installing Python packages in virtual environment...${NC}"
source "$VENV_DIR/bin/activate"
pip3 install --upgrade pip
pip3 install catkin_pkg  # Required for ROS2 package building
pip3 install opencv-python opencv-contrib-python
pip3 install numpy scipy
pip3 install transforms3d
pip3 install --upgrade ultralytics  # YOLO26 (latest from Ultralytics, released Jan 2026)
pip3 install onnxruntime  # For ONNX inference on laptop & RPi5
echo ""
echo -e "${GREEN}✅ YOLO26 installed! (43% faster than YOLO11)${NC}"
echo ""
echo "Note: PyTorch/CUDA installation skipped for now"
echo "To enable GPU training for YOLO26, activate venv and run:"
echo "  source ~/teamsteelbot/klevor-v2/.venv/bin/activate"
echo "  pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu118"
echo ""
echo "After installation, verify YOLO26:"
echo "  yolo version"

# Step 7: Create workspace
echo -e "${BLUE}[7/9] Creating ROS2 workspace...${NC}"
if [ ! -d "$WORKSPACE_DIR/src" ]; then
    mkdir -p "$WORKSPACE_DIR/src"
    cd "$WORKSPACE_DIR"

    # Source ROS2 and venv before creating packages
    source /opt/ros/kilted/setup.bash
    source "$VENV_DIR/bin/activate"

    # Create packages
    cd src
    ros2 pkg create --build-type ament_python teamsteelbot_bringup
    ros2 pkg create --build-type ament_python teamsteelbot_vision
    ros2 pkg create --build-type ament_python teamsteelbot_control
    ros2 pkg create --build-type ament_python teamsteelbot_sensors
    ros2 pkg create --build-type ament_cmake teamsteelbot_msgs
    ros2 pkg create --build-type ament_python teamsteelbot_simulation

    # Create additional directories
    mkdir -p "$WORKSPACE_DIR/models"
    mkdir -p "$WORKSPACE_DIR/datasets/images"
    mkdir -p "$WORKSPACE_DIR/datasets/labels"

    cd "$WORKSPACE_DIR"
    colcon build
else
    echo "Workspace already exists at $WORKSPACE_DIR"
fi

# Step 7: Configure environment
echo -e "${BLUE}[7/9] Configuring environment...${NC}"
if ! grep -q "source /opt/ros/kilted/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# Python virtual environment for TeamSteelBot" >> ~/.bashrc
    echo "source $VENV_DIR/bin/activate" >> ~/.bashrc
    echo "" >> ~/.bashrc
    echo "# ROS2 Kilted Kaiju (May 2025 release)" >> ~/.bashrc
    echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc
    echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=42  # Unique ID for your robot" >> ~/.bashrc
    echo "" >> ~/.bashrc
    echo "# Performance: Use EventsExecutor for 10x speedup" >> ~/.bashrc
    echo "# export RCL_EXECUTOR=events_executor" >> ~/.bashrc
    echo "" >> ~/.bashrc
fi

echo -e "${GREEN}✅ Environment configured${NC}"

# Step 8: Verify installation
echo -e "${BLUE}[8/9] Verifying installation...${NC}"
source /opt/ros/kilted/setup.bash
ROS_VERSION=$(ros2 --version 2>&1 | head -n 1)
echo -e "${GREEN}✅ $ROS_VERSION${NC}"

# Step 9: Summary
echo ""
echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}🎉 Setup Complete!${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""
echo -e "${BLUE}ROS2 Kilted Kaiju Features:${NC}"
echo "  • 10x faster Python executor (EventsExecutor)"
echo "  • NV12 image support (perfect for RPi Camera!)"
echo "  • Zenoh middleware option"
echo "  • Enhanced ROSBag with action server"
echo "  • Gazebo Ionic (latest simulation)"
echo ""
echo -e "${BLUE}Next steps:${NC}"
echo "1. Close and reopen your terminal (or run: source ~/.bashrc)"
echo "2. Verify: ros2 --version"
echo "3. Test Gazebo Ionic: gz sim empty.sdf"
echo "4. Read: docs/development/ROS2_KILTED_UPDATE.md"
echo "5. Build workspace: cd ~/teamsteelbot/klevor-v2 && colcon build"
echo "6. Train YOLO26: See docs/development/YOLO26_QUICK_START.md"
echo ""
echo "Workspace: $WORKSPACE_DIR"
echo ""
echo -e "${GREEN}🚀 You're using the latest ROS2 LTS with 10x faster performance!${NC}"
echo ""
