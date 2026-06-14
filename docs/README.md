# Future Engineers 2026 - Team Voldemor

## Documentation Hub

This documentation repository contains comprehensive proposals, design documents, and development logs for Team Voldemor's Future Engineers 2026 competition robot.

### WRO 2026 Challenge Overview

**Competition:** World Robot Olympiad - Future Engineers Category
**Challenge:** Autonomous self-driving robot car navigating a track with traffic signs
**Key Requirements:**
- Maximum size: 30×20×30 cm
- Detect and respond to green/red traffic sign pillars (left/right lane indicators)
- Engineer's Journal contributes **50% of total score**
- Surprise rules will be announced
- Must complete full lap to earn parking points

---

## Three Unique Proposals

We've developed three distinct approaches, each optimized for different strengths:

### 🚀 [Proposal 1: Velocity Edge - ROS2 Performance Architecture](proposals/01-velocity-edge-ros2.md)
**Philosophy:** Performance-first with enterprise-grade robotics framework
- **Core Tech:** NVIDIA Jetson Orin Nano + ROS2 + Stereo Vision + YOLOv8
- **Strengths:** Speed (2.5 m/s), adaptability, professional ecosystem
- **Risk Level:** Medium
- **Best For:** Teams experienced with ROS2 and ML, need maximum adaptability

### ⚡ [Proposal 2: Minimalist Racer - Classical Engineering Excellence](proposals/02-minimalist-racer-classical.md) ⭐ **RECOMMENDED**
**Philosophy:** Ultra-reliable through simplicity and redundancy
- **Core Tech:** RPi5 FreeRTOS + Classical CV + Triple Sensor Redundancy
- **Strengths:** Maximum reliability, fastest speed (3.0 m/s), explainable engineering
- **Risk Level:** Low
- **Best For:** Winning through consistency, clear documentation, proven methods

### 🧠 [Proposal 3: Cognitive Racer - Vision Transformer Revolution](proposals/03-cognitive-racer-vit.md)
**Philosophy:** AI-first with end-to-end learning
- **Core Tech:** RPi5 + Hailo-8L + Vision Transformer (DINOv2) + RL
- **Strengths:** Innovation, adaptability to surprise rules, research-grade documentation
- **Risk Level:** High
- **Best For:** Teams with ML expertise, want to showcase cutting-edge AI

---

## Recommended Strategy: Hybrid Approach

**Primary (Weeks 1-8):** Build Proposal 2 "Minimalist Racer"
- Focus on reliability and speed
- Complete by Week 8 with extensive testing
- Guaranteed competition-ready robot

**Secondary (Weeks 8-12):** Explore Proposal 3 "Cognitive Racer"
- If primary is successful, invest in innovation
- Document both approaches in Engineer's Journal
- Show evolution of thinking (judges appreciate this!)

**Fallback:** Proposal 1 "Velocity Edge"
- If surprise rules require major pivot
- ROS2 provides fastest adaptation path

---

## Repository Structure

```
docs/
├── README.md                              # This file
├── proposals/                             # Detailed proposal documents
│   ├── 01-velocity-edge-ros2.md          # ROS2 + Jetson approach
│   ├── 02-minimalist-racer-classical.md  # Classical CV + FreeRTOS (PRIMARY)
│   └── 03-cognitive-racer-vit.md         # Vision Transformer AI
├── hardware/                              # Hardware documentation
│   ├── bom.csv                           # Bill of materials with sourcing
│   ├── wiring-diagram.pdf                # Circuit diagrams
│   └── mechanical-design/                # 3D CAD files and STLs
├── software/                              # Software architecture
│   ├── architecture.md                   # System design overview
│   ├── algorithms/                       # Algorithm explanations
│   └── api-reference.md                  # Code documentation
├── testing/                               # Testing and validation
│   ├── test-log-template.md             # Structured test format
│   ├── results/                          # Test run data (CSV)
│   └── analysis.ipynb                    # Jupyter analysis notebooks
├── journal/                               # Engineer's Journal (50% of score!)
│   ├── engineer-journal-draft.md        # Main competition submission
│   ├── build-logs/                       # Weekly development updates
│   └── media/                            # Photos, videos, diagrams
└── research/                              # Background research
    ├── literature-review.md              # Related work and inspiration
    └── experiments/                      # A/B testing and experiments
```

---

## Key Design Decisions

### Hardware Platform
**Selected:** Raspberry Pi 5 (all proposals)
- Proven from VoldemorBot robot (WRO 2025)
- Excellent community support
- Sufficient compute for both classical CV and AI
- Cost-effective ($80 vs $500 for Jetson)

### Sensor Strategy
**Triple Redundancy (Proposal 2 - Recommended):**
1. **Camera:** RPi Camera Module 3 - HSV color detection at 120 FPS
2. **Color Sensor:** TCS34725 hardware RGB sensor - physical validation
3. **LiDAR:** RPLiDAR C1 - spatial awareness and wall following

**Rationale:** 2-of-3 voting ensures reliability even if one sensor fails. The hardware color sensor is unique innovation - validates camera detection as robot passes signs.

### Vision Approach Comparison

| Approach | FPS | Accuracy | Explainability | Training | Latency |
|----------|-----|----------|----------------|----------|---------|
| **Classical CV (Prop. 2)** | 120 | High | Excellent | None | 5ms |
| **YOLO (Prop. 1)** | 60 | Very High | Medium | Weeks | 10ms |
| **ViT (Prop. 3)** | 30 | Highest | Low | Months | 20ms |

**Winner:** Classical CV for this specific task (simple color detection). Save AI for complex perception tasks.

### Control Architecture
**FreeRTOS (Proposal 2) vs ROS2 (Proposal 1):**
- **FreeRTOS:** Deterministic, lightweight, 200Hz control loop, faster development
- **ROS2:** Powerful ecosystem, better for complex behaviors, steeper learning curve

**Decision:** FreeRTOS for maximum reliability in 12-week timeline.

---

## Documentation Philosophy

### Engineer's Journal = 50% of Score
This is as important as robot performance! Our approach:

**1. Show Your Work**
- Daily build logs with photos/videos
- Failed designs with lessons learned
- Mathematical derivations and proofs
- Testing data with statistical analysis

**2. Tell a Story**
- Evolution of design thinking
- Trade-off analysis and decision rationale
- Problem-solving narrative
- Team collaboration insights

**3. Make it Visual**
- Circuit diagrams and wiring photos
- 3D CAD renders and assembly guides
- Algorithm flowcharts
- Performance graphs and heatmaps

**4. Demonstrate Rigor**
- 100+ test runs with documented results
- A/B testing of design alternatives
- Systematic parameter tuning logs
- Failure mode analysis

**5. Enable Reproducibility**
- Complete BOM with part numbers
- Step-by-step assembly instructions
- Code walkthroughs with explanations
- Calibration procedures

---

## Timeline to Competition

### Phase 1: Foundation (Weeks 1-4)
- ✅ Hardware sourcing and assembly
- ✅ FreeRTOS environment setup
- ✅ Sensor driver integration
- ✅ Basic motor control
- ✅ Color detection prototype

### Phase 2: Integration (Weeks 5-8)
- ✅ Complete vision pipeline
- ✅ State machine implementation
- ✅ Triple redundancy voting
- ✅ Hardware watchdog
- ✅ Initial autonomous laps

### Phase 3: Optimization (Weeks 9-12)
- ✅ Performance tuning (target 3.0 m/s)
- ✅ 100+ test runs with logging
- ✅ Edge case handling
- ✅ Engineer's Journal finalization
- ✅ Competition preparation

**Total: 12 weeks to competition-ready**

---

## Reuse from VoldemorBot (WRO 2025 Robot)

### ✅ Keep These Proven Components
- **RPLiDAR C1:** Excellent 2D navigation, battle-tested
- **BNO08X IMU:** Superior sensor fusion capabilities
- **VL53L0X Distance Sensors:** Reliable time-of-flight measurements
- **Raspberry Pi Pico 2W:** Perfect for motor control
- **Raspberry Pi 5:** Powerful enough for all proposals
- **Documentation Structure:** 74 .md files show strong culture
- **Monitoring Concepts:** Adapt Grafana/Prometheus approach

### 🔄 Selectively Adapt
- **Challenge Handlers:** Reuse algorithms, simplify architecture
- **USB-CDC Protocol:** Proven Pi5↔Pico communication
- **Calibration Routines:** Sensor calibration procedures

### ❌ Replace for V2
- **Hailo AI (Proposal 2):** Not needed for classical CV
- **Go Implementation (Proposal 2):** C++ for bare-metal FreeRTOS
- **CLIP Vision (All):** Overkill for simple color detection

---

## Success Metrics

### Competition Performance Goals
- ✅ **Lap Completion:** 100% success rate in 10 consecutive runs
- ✅ **Lap Time:** <25 seconds (competitive), <20 seconds (top tier)
- ✅ **Sign Detection:** >95% accuracy across lighting conditions
- ✅ **Reliability:** Zero collisions in practice runs
- ✅ **Size Compliance:** 30×20×30 cm maximum

### Documentation Goals
- ✅ **Engineer's Journal:** 50+ pages with rich media
- ✅ **Build Logs:** Weekly updates with photos/videos
- ✅ **Test Data:** 100+ runs with statistical analysis
- ✅ **Technical Depth:** Mathematical models and derivations
- ✅ **Clarity:** Understandable by judges without robotics background

### Innovation Goals
- ✅ **Unique Element:** Hardware color sensor validation (Proposal 2)
- ✅ **Novel Approach:** If time permits, explore ViT (Proposal 3)
- ✅ **Community Contribution:** Open-source code and designs

---

## Risk Management

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|-----------|
| Component delivery delays | High | Medium | Order Week 1, source alternatives |
| Sensor calibration issues | Medium | Low | Daily calibration routine, redundancy |
| Lighting variations | High | High | Auto white-balance, extensive testing |
| Surprise rule changes | High | Certain | Modular design, 2-week adaptation buffer |
| Software bugs | Medium | Medium | Extensive testing, hardware watchdog |
| Mechanical failures | High | Low | Spare parts, modular assembly |
| Documentation quality | Critical | Low | Start early, external review, exemplars |

---

## References & Resources

### Official WRO
- **[2026 Game Rules (PDF)](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)**
- **[Future Engineers Getting Started](https://world-robot-olympiad-association.github.io/future-engineers-gs/)**

### Technical Resources
- FreeRTOS Documentation
- OpenCV HSV Color Space Tutorials
- PID Control Tuning Guides
- Raspberry Pi 5 Technical Specifications
- RPi Camera Module 3 Documentation

### Inspiration & Case Studies
- F1TENTH Autonomous Racing Competition
- DonkeyCar Community Projects
- ROS2 Navigation Stack Examples
- WRO Previous Winners' Repositories

### Academic Papers
- Classical CV: "Real-time Color-Based Traffic Sign Detection"
- Sensor Fusion: "Multi-Sensor Fusion for Mobile Robotics"
- PID Tuning: "Ziegler-Nichols Method and Variants"

---

## Team Information

**Team Name:** Team Voldemor
**Competition:** WRO Future Engineers 2026
**Robot Name:** TBD (naming after design selection)
**Previous Robot:** VoldemorBot (WRO 2025 Futuros Ingenieros)

### Contact & Collaboration
- **Repository:** teamvoldemor (this repo)
- **Previous Work:** teamvoldemor/VoldemorBot (reference implementation)
- **Documentation Site:** (TBD - MkDocs Material setup)

---

## Next Steps

### Immediate Actions (Week 1)
1. **Decision:** Select primary proposal (recommend Proposal 2)
2. **Hardware:** Finalize BOM and order all components
3. **Environment:** Set up development tools (FreeRTOS, cross-compiler)
4. **Documentation:** Begin Engineer's Journal - document from Day 1!
5. **Repository:** Set up version control and branching strategy

### Questions to Resolve
- [ ] Budget approval for hardware components?
- [ ] Team composition and role assignments?
- [ ] Access to practice track for testing?
- [ ] Timeline to competition date?
- [ ] Any specific constraints (cost, size, preferences)?

---

## Conclusion

We have three strong proposals, each with unique advantages. The **Minimalist Racer (Proposal 2)** offers the best balance of performance, reliability, and documentation potential for winning WRO 2026.

By leveraging proven VoldemorBot components while introducing innovative elements (hardware color sensor, triple redundancy), we can build a robot that is:
- **Fast** (3.0 m/s target)
- **Reliable** (triple sensor redundancy)
- **Explainable** (classical engineering, not "black box" AI)
- **Well-documented** (100+ test runs, comprehensive journal)

The hybrid strategy allows us to compete with a reliable robot while exploring cutting-edge AI innovation if time permits.

**Ready to begin implementation upon approval. Let's build a winner! 🏆**
