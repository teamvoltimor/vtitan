# START HERE - Local Development Guide

## Welcome! 👋

You want to implement **Proposal 4: ROS2 Edge Racer** on your laptop (32GB RAM, RTX 4050, WSL2) with **ROS2 Kilted Kaiju** (latest LTS release) before porting to the Raspberry Pi 5. This is the **perfect approach** - develop 80% of the logic locally with fast iteration, then port to hardware later.

**ROS2 Distribution:** Kilted Kaiju (May 2025) - **10x faster Python executor!** 🚀

---

## Your Setup is Perfect For:

✅ **ROS2 Development** - Plenty of RAM for Humble + Gazebo
✅ **Vision Processing** - Test classical CV and YOLO locally
✅ **ML Training** - RTX 4050 can train YOLO26 models fast! (43% faster than YOLO11!)
✅ **Fast Iteration** - No waiting for RPi5 to build/test
✅ **Simulation** - Run full robot simulation before hardware

---

## 🚀 Quick Start (30 minutes)

### Step 1: Read the Proposal (5 min)
- Already done! You've read `docs/proposals/systems/04-ros2-edge-racer-hybrid.md`
- Key takeaway: ROS2 + Raspberry Pi 5 + Hailo + 95% Klevor reuse

### Step 2: Install ROS2 on WSL2 (20 min)

Open WSL2 terminal:

```bash
cd /mnt/c/Users/ralva/Documents/private/projects/archived/teamsteelbot/klevor-v2

# Run automated setup script
bash scripts/setup_wsl2_dev.sh
```

This will:
- Install ROS2 Humble
- Install Gazebo and visualization tools
- Create ROS2 workspace at `~/teamsteelbot_ws`
- Install Python dependencies (OpenCV, YOLO26, ONNX Runtime, etc.)
- Configure your environment

**Note:** PyTorch/CUDA setup is optional - only needed if you want to train YOLO models locally.

### Step 3: Verify Installation (5 min)

```bash
# Close and reopen terminal, then test:
ros2 --version            # Should show "ros2 humble"
gz sim empty.sdf          # Gazebo should open (Ctrl+C to close)
rviz2                     # RViz2 should open (Ctrl+C to close)

cd ~/teamsteelbot_ws
colcon build              # Should build successfully
```

---

## 📚 What You Got

I've created complete documentation for your journey:

### 1. **Local Setup Guide** (`docs/development/local-setup-wsl2.md`)
- Detailed WSL2 + ROS2 installation
- Package structure
- Simulation vs hardware architecture
- GPU setup for YOLO training
- Troubleshooting guide

### 2. **Development Roadmap** (`docs/development/roadmap-local-dev.md`)
- 8-week plan broken into phases
- Week-by-week tasks
- Clear milestones and deliverables
- Best practices and workflows
- Success criteria

### 3. **Starter Code** (`docs/development/starter-code-examples.md`)
- Mock camera node (publishes test images)
- Classical CV sign detector (HSV-based)
- State machine (race logic)
- Decision node (vision → motor commands)
- Launch files to run everything
- Unit tests

### 4. **ROS2 Quick Reference** (`docs/development/ros2-quick-reference.md`)
- Essential commands cheat sheet
- Topic, node, service commands
- ROS bag recording/playback
- Debugging tips
- Performance monitoring

### 5. **Setup Script** (`scripts/setup_wsl2_dev.sh`)
- Automated installation of everything
- Just run and wait!

---

## 🎯 Your Path Forward

### Week 1: Foundation (START HERE!)

**Day 1-2: ROS2 Basics**
1. Complete ROS2 beginner tutorials (2 hours): https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html
   - Topics, nodes, publishers, subscribers
2. Create hello world node:
   ```bash
   cd ~/teamsteelbot_ws/src
   ros2 pkg create --build-type ament_python my_test_pkg
   # Follow tutorial to create simple pub/sub
   cd ~/teamsteelbot_ws
   colcon build --symlink-install
   source install/setup.bash
   ros2 run my_test_pkg talker
   ```

**Day 3-5: First Real Nodes**
1. Copy starter code from `docs/development/starter-code-examples.md`
2. Implement mock camera node
3. Test in RViz2
4. Goal: See test images streaming at 30 Hz

**Day 6-7: Vision Detection**
1. Implement classical CV sign detector
2. Test on mock camera output
3. Goal: Detect colored rectangles with bounding boxes

**End of Week 1 Checkpoint:**
- [ ] ROS2 workspace builds successfully
- [ ] Mock camera publishes test images
- [ ] Sign detector finds colored signs
- [ ] Can visualize everything in RViz2

### Week 2-3: Vision Pipeline
- Improve detection accuracy
- Handle edge cases (multiple signs, partial occlusion)
- (Optional) Train YOLO model on your RTX 4050
- Create test dataset with various lighting conditions

### Week 3-4: Control Logic
- State machine for race logic
- Decision node (detections → velocity commands)
- Speed controller with smooth acceleration
- Test: Does robot react correctly to sign colors?

### Week 4-6: Integration
- Add mock sensors (LiDAR, IMU)
- Sensor fusion with robot_localization
- Full system launch file
- Complete simulated lap!

### Week 6-8: Optimization
- Performance tuning
- Add unique features (TCS34725 validation logic)
- Documentation (start Engineer's Journal)
- Prepare for RPi5 port

### Week 9+: Hardware (On RPi5)
- Transfer code to Raspberry Pi 5
- Integrate real sensors
- Test with Hailo acceleration
- Physical track testing

---

## 🛠️ Development Workflow (Daily)

```bash
# Morning routine
cd ~/teamsteelbot_ws
git pull                           # Get latest code
colcon build --symlink-install     # Build (symlink = no rebuild for Python edits)
source install/setup.bash          # Source workspace

# Develop
# Edit Python files in src/teamsteelbot_*/teamsteelbot_*/*.py
# With --symlink-install, changes take effect immediately (no rebuild!)

# Test single node
ros2 run teamsteelbot_vision sign_detector_classic

# Test full system
ros2 launch teamsteelbot_bringup simulation.launch.py

# Debug
ros2 topic echo /detections        # See what's being detected
ros2 topic hz /camera/image_raw    # Check publishing rate
rqt_graph                          # Visualize node connections

# Record test run
ros2 bag record -a -o test_run_$(date +%s)

# Commit progress
git add .
git commit -m "Improved color detection thresholds"
git push
```

---

## 📖 Learning Resources

### Must-Read (Start Here)
1. **ROS2 Beginner Tutorials** (2-3 hours)
   https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html

2. **ROS2 Intermediate Tutorials** (3-4 hours)
   https://docs.ros.org/en/humble/Tutorials/Intermediate.html

### Reference
- **OpenCV Python Tutorials:** https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html
- **YOLO26 Docs:** https://docs.ultralytics.com/models/yolo26/ (latest & fastest - 43% faster!)
- **YOLO26 Quick Start:** `docs/development/YOLO26-QUICK-START.md` ⚡ **Start here!**
- **YOLO26 + Hailo Guide:** `docs/development/yolo26-hailo-guide.md`
- **Note:** YOLO26 was released Jan 14, 2026 (2 weeks ago!)
- **ROS2 API Docs:** https://docs.ros.org/en/humble/

### Community
- **ROS Discord:** https://discord.gg/ros
- **ROS Answers:** https://answers.ros.org/
- **Stack Overflow:** Tag `ros2`

---

## 🎓 Key Concepts to Understand

### ROS2 Basics
- **Node:** A running process (e.g., camera, detector, decision maker)
- **Topic:** Data stream between nodes (e.g., `/camera/image_raw`)
- **Message:** Data format (e.g., `sensor_msgs/Image`)
- **Publisher:** Node that sends data on a topic
- **Subscriber:** Node that receives data from a topic
- **Launch File:** Starts multiple nodes at once

### Vision Pipeline
```
Camera Node → Vision Node → Decision Node → Motor Control
  (images)    (detections)   (commands)
```

### State Machine
```
IDLE → RACING → TURNING_LEFT/RIGHT → RACING → ...
              ↑                              ↑
         (red/green sign detected)    (turn complete)
```

---

## 🚨 Common Pitfalls

### 1. Forgetting to Source Workspace
**Problem:** `Package 'teamsteelbot_vision' not found`
**Solution:** `source ~/teamsteelbot_ws/install/setup.bash`
**Pro tip:** Add to `~/.bashrc` to auto-source

### 2. Not Updating setup.py
**Problem:** `ros2 run pkg node` doesn't find executable
**Solution:** Add entry point to `setup.py`, then rebuild

### 3. QoS Mismatch
**Problem:** Topic publishes but subscriber doesn't receive
**Solution:** Match QoS settings (reliability, durability)
**Quick fix:** Use default QoS (depth=10)

### 4. Image Encoding Issues
**Problem:** `cv2_to_imgmsg` or `imgmsg_to_cv2` fails
**Solution:** Check encoding ('bgr8', 'rgb8', 'mono8')
**OpenCV uses BGR, not RGB!**

### 5. Symlink Install Not Working
**Problem:** Python changes don't take effect
**Solution:** Use `colcon build --symlink-install` (with two dashes!)

---

## 🎯 Success Metrics

Track your progress with these milestones:

| Week | Milestone | Success Criteria |
|------|-----------|------------------|
| 1 | Foundation | ROS2 workspace builds; mock camera publishes |
| 2 | Vision | Detects signs at 30+ FPS with >70% accuracy |
| 3 | Control | State machine transitions correctly |
| 4 | Sensors | Sensor fusion working; odometry published |
| 5 | Integration | Single launch file starts full system |
| 6 | Simulation | Complete virtual lap end-to-end |
| 7 | Features | TCS34725 validation logic implemented |
| 8 | Ready | Code runs on RPi5 (with config changes) |

---

## 🔄 Laptop → RPi5 Port Strategy

**The beauty of this approach:** Most code is identical!

### What Stays the Same (80%)
- ✅ Vision detection nodes
- ✅ Decision/control logic
- ✅ State machine
- ✅ Custom messages
- ✅ Launch file structure

### What Changes (20%)
- ❌ Mock camera → Real camera (libcamera)
- ❌ Mock LiDAR → rplidar_ros
- ❌ Mock sensors → Real I2C sensors
- ❌ Config: simulation_params.yaml → robot_params.yaml

### Transfer Process
```bash
# On laptop: Export workspace
cd ~/teamsteelbot_ws/src
tar -czf teamsteelbot_code.tar.gz teamsteelbot_*

# Transfer (USB, scp, git, etc.)
scp teamsteelbot_code.tar.gz pi@raspberrypi.local:~/

# On RPi5: Extract and build
cd ~/teamsteelbot_ws/src
tar -xzf ~/teamsteelbot_code.tar.gz
cd ~/teamsteelbot_ws
colcon build --symlink-install
```

---

## 💡 Pro Tips

1. **Use Git from Day 1**
   Push to GitHub/GitLab daily. Makes laptop→RPi5 transfer easy.

2. **Record Everything**
   Use `ros2 bag record -a` during tests. Analyze later with PlotJuggler.

3. **Start Simple**
   Don't jump to YOLO immediately. Classical CV is faster to develop and debug.

4. **Test Incrementally**
   Get one node working before adding the next. Don't build the entire system at once.

5. **Use RViz2 Constantly**
   Visualize everything: camera feed, detections, robot state, tf frames.

6. **Document as You Go**
   Take screenshots, write notes. Your Engineer's Journal starts NOW.

7. **Join ROS Discord**
   Get help from the community when stuck.

---

## 📝 Next Actions (Right Now!)

### Action 1: Install ROS2 (30 min)
```bash
cd /mnt/c/Users/ralva/Documents/private/projects/archived/teamsteelbot/klevor-v2
bash scripts/setup_wsl2_dev.sh
```

### Action 2: ROS2 Tutorials (2 hours)
https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html

Complete:
- "Configuring environment"
- "Using turtlesim, ros2, and rqt"
- "Understanding nodes"
- "Understanding topics"
- "Understanding services"

### Action 3: Build First Node (1 hour)
Follow starter code in `docs/development/starter-code-examples.md`
- Copy mock camera node
- Update setup.py
- Build and run
- Verify in RViz2

### Action 4: Plan Week 1 (30 min)
Read `docs/development/roadmap-local-dev.md` Phase 1 in detail.
Set daily goals for next 7 days.

---

## 🎉 You're Ready!

You have:
- ✅ Hardware (laptop with WSL2)
- ✅ Documentation (setup, roadmap, starter code, reference)
- ✅ Plan (8 weeks on laptop, 3 weeks on RPi5)
- ✅ Code skeleton (ready to copy and build on)

**Time to start!** Begin with the setup script, then dive into ROS2 tutorials.

---

## 📞 Questions?

If you get stuck:
1. Check `docs/development/ros2-quick-reference.md` for commands
2. Check `docs/development/local-setup-wsl2.md` for troubleshooting
3. Search ROS Answers: https://answers.ros.org/
4. Ask on ROS Discord: https://discord.gg/ros

---

## 🏁 Final Thoughts

**Why this approach works:**

1. **Fast Iteration:** No waiting for RPi5 builds (faster CPU on laptop)
2. **Easy Debugging:** printf debugging, breakpoints, full IDE support
3. **Risk Reduction:** 80% of code tested before touching hardware
4. **GPU Access:** Train YOLO models locally with RTX 4050
5. **Simulation:** Test edge cases without physical robot
6. **Professional:** ROS2 gives you visualization, bags, tooling

**You'll thank yourself later when hardware integration takes days, not weeks!**

---

**Ready? Let's go! 🚀**

```bash
bash scripts/setup_wsl2_dev.sh
```

**See you at the competition! 🏁🤖**
