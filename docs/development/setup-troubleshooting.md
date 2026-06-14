# Setup Script Troubleshooting Guide

## Common Errors and Fixes

### Error: `pip3: command not found`

**Problem:** The script tried to use pip3 before installing it.

**Fix:** The script has been updated to install `python3-pip` first.

**Manual fix if needed:**
```bash
# Install pip3
sudo apt install python3-pip python3-venv -y

# Then continue with Python packages
pip3 install --upgrade pip
pip3 install opencv-python opencv-contrib-python numpy scipy
pip3 install transforms3d
pip3 install --upgrade ultralytics
pip3 install onnxruntime
```

---

### Error: `E: Unable to locate package ros-kilted-gazebo-ros-pkgs`

**Problem:** Package name has changed in Kilted.

**Fix:** Use `ros-kilted-ros-gz` instead.

```bash
sudo apt install ros-kilted-ros-gz -y
```

See: `kilted-package-fixes.md` for details.

---

### Error: `Release file for ... is not valid yet`

**Problem:** System clock might be wrong or repository metadata is from the future.

**Fix:**
```bash
# Check system time
date

# If wrong, sync time
sudo apt install ntpdate -y
sudo ntpdate time.nist.gov

# Update package lists
sudo apt update
```

---

### Error: `Ubuntu version is not 24.04`

**Problem:** Kilted requires Ubuntu 24.04 LTS (Noble Numbat).

**Fix:**
```bash
# Check Ubuntu version
lsb_release -a

# If not 24.04, you have two options:

# Option 1: Upgrade WSL to Ubuntu 24.04
wsl --install Ubuntu-24.04

# Option 2: Use ROS2 Humble on Ubuntu 22.04
# (See HUMBLE fallback guide)
```

---

### Error: `Could not get lock /var/lib/apt/lists/lock`

**Problem:** Another apt process is running.

**Fix:**
```bash
# Wait for other apt processes to finish, then:
sudo apt update

# If still locked after 5 minutes:
sudo rm /var/lib/apt/lists/lock
sudo rm /var/cache/apt/archives/lock
sudo rm /var/lib/dpkg/lock*
sudo dpkg --configure -a
sudo apt update
```

---

### Error: Python packages fail to install

**Problem:** Missing build dependencies.

**Fix:**
```bash
# Install build essentials
sudo apt install build-essential python3-dev -y

# Install pip dependencies
sudo apt install python3-pip python3-venv -y

# Retry pip installations
pip3 install --upgrade pip
pip3 install opencv-python opencv-contrib-python numpy scipy
```

---

### Error: `EXTERNALLY-MANAGED-ENVIRONMENT`

**Problem:** Ubuntu 24.04 uses externally managed Python environments.

**Fix (Option 1 - Recommended): Use virtual environment**
```bash
# Create virtual environment
python3 -m venv ~/teamvoldemor/voldemorbot/.venv

# Activate it
source ~/teamvoldemor/voldemorbot/.venv/bin/activate

# Install packages
pip3 install opencv-python opencv-contrib-python numpy scipy
pip3 install transforms3d ultralytics onnxruntime

# Add to ~/.bashrc for persistence
echo "source ~/teamvoldemor/voldemorbot/.venv/bin/activate" >> ~/.bashrc
```

**Fix (Option 2): Use system packages with --break-system-packages**
```bash
# Install with flag (not recommended, but works)
pip3 install --break-system-packages opencv-python
```

**Fix (Option 3): Use apt for system packages**
```bash
# Install what's available via apt
sudo apt install python3-opencv python3-numpy python3-scipy -y

# Only use pip for packages not in apt
pip3 install --user ultralytics onnxruntime transforms3d
```

---

### Error: Insufficient disk space

**Problem:** Not enough space for all packages.

**Fix:**
```bash
# Check available space
df -h

# Clean up if needed
sudo apt clean
sudo apt autoremove -y

# Increase WSL2 disk size (from Windows PowerShell):
# wsl --shutdown
# diskpart
# select vdisk file="C:\Users\YourUser\AppData\Local\Packages\...\ext4.vhdx"
# expand vdisk maximum=100000  # Size in MB
```

---

### Error: Network/download issues

**Problem:** Slow or failing downloads.

**Fix:**
```bash
# Try different Ubuntu mirror
sudo sed -i 's|http://archive.ubuntu.com|http://mirror.us.leaseweb.net|g' /etc/apt/sources.list
sudo apt update

# Or use main server
sudo sed -i 's|http://mirror.us.leaseweb.net|http://archive.ubuntu.com|g' /etc/apt/sources.list
sudo apt update
```

---

## Step-by-Step Manual Installation

If the script keeps failing, install manually:

### 1. Update System
```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Setup Locale
```bash
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 3. Add ROS2 Repository
```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update
sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

### 4. Install ROS2 Kilted
```bash
sudo apt install ros-kilted-desktop -y
sudo apt install ros-dev-tools -y
sudo apt install python3-colcon-common-extensions -y
```

### 5. Install Gazebo
```bash
sudo apt install ros-kilted-ros-gz -y
```

### 6. Install Other ROS2 Packages
```bash
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
```

### 7. Install Python Tools
```bash
sudo apt install python3-pip python3-venv -y
pip3 install --upgrade pip
pip3 install opencv-python opencv-contrib-python numpy scipy
pip3 install transforms3d ultralytics onnxruntime
```

### 8. Create Workspace
```bash
mkdir -p ~/teamvoldemor/voldemorbot/src
cd ~/teamvoldemor/voldemorbot
source /opt/ros/kilted/setup.bash
colcon build
```

### 9. Configure Environment
```bash
echo "" >> ~/.bashrc
echo "# Python virtual environment" >> ~/.bashrc
echo "source ~/teamvoldemor/voldemorbot/.venv/bin/activate" >> ~/.bashrc
echo "" >> ~/.bashrc
echo "# ROS2 Kilted Kaiju" >> ~/.bashrc
echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc
echo "source ~/teamvoldemor/voldemorbot/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

---

## Verification Commands

After installation, verify everything works:

```bash
# Check Ubuntu version
lsb_release -a
# Should show: Ubuntu 24.04 LTS

# Check ROS2
ros2 --version
# Should show: ros2 cli version: 0.33.0 or later

# Check Python
python3 --version
# Should show: Python 3.12.x

# Check pip
pip3 --version
# Should show: pip 24.x or later

# Check Gazebo
gz sim --versions
# Should show Gazebo Ionic

# Check YOLO26
yolo version
# Should show: Ultralytics YOLO 8.3.x

# List ROS2 packages
ros2 pkg list | head -20

# Test Gazebo
gz sim empty.sdf
# Should open Gazebo (Ctrl+C to close)
```

---

## If All Else Fails

### Option 1: Use Docker
```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Pull ROS2 Kilted image
docker pull osrf/ros:kilted-desktop

# Run container
docker run -it --rm \
  --network host \
  -v ~/teamvoldemor/voldemorbot:/root/teamvoldemor/voldemorbot \
  osrf/ros:kilted-desktop
```

### Option 2: Use Humble Instead
If you can't upgrade to Ubuntu 24.04, use ROS2 Humble (on Ubuntu 22.04):

```bash
# Replace 'kilted' with 'humble' in all commands
sudo apt install ros-humble-desktop -y
```

**Note:** You'll lose the 10x EventsExecutor speedup, but YOLO26 will still work!

---

## Getting Help

If you're still stuck:

1. **Check logs:** Look for specific error messages
2. **Google the error:** Often someone else had the same issue
3. **ROS Answers:** https://answers.ros.org/
4. **ROS Discord:** https://discord.gg/ros

---

## Script Status

✅ **Script has been fixed** with:
- pip3 installation added
- Correct Gazebo package name
- Ubuntu 24.04 version check
- Virtual environment for Python packages (solves externally-managed-environment)
- Auto-activation in ~/.bashrc
- Better error messages

The updated script should work without these errors! 🎉

**Note:** The script now uses a Python virtual environment at `~/teamvoldemor/voldemorbot/.venv` which is automatically activated in your shell. This is the recommended approach for Ubuntu 24.04.
