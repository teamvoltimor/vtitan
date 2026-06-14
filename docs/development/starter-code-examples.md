# Starter Code Examples

Ready-to-use code snippets to kickstart your development. Copy these into your ROS2 workspace after running the setup script.

---

## 1. Mock Camera Node (Phase 1)

**File:** `~/teamvoldemor_ws/src/teamvoldemor_simulation/teamvoldemor_simulation/mock_camera_node.py`

```python
#!/usr/bin/env python3
"""
Mock camera node - publishes test images with colored rectangles (signs)
Great for testing vision pipeline without real camera
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class MockCameraNode(Node):
    def __init__(self):
        super().__init__('mock_camera')

        # Parameters
        self.declare_parameter('fps', 30)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        fps = self.get_parameter('fps').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value

        # Publisher
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        # Timer for publishing at fixed rate
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

        # CV Bridge for ROS-OpenCV conversion
        self.bridge = CvBridge()

        # Current frame number (for animation)
        self.frame_count = 0

        self.get_logger().info(f'Mock camera started: {self.width}x{self.height} @ {fps} FPS')

    def generate_test_image(self):
        """Generate test image with moving colored rectangle (simulated sign)"""
        # Create blank image (gray background)
        img = np.full((self.height, self.width, 3), 128, dtype=np.uint8)

        # Simulate sign moving across frame
        sign_width = 100
        sign_height = 120
        x_pos = int((self.frame_count * 5) % (self.width + sign_width)) - sign_width
        y_pos = self.height // 2 - sign_height // 2

        # Cycle through colors (red, green, blue signs)
        colors = [
            (0, 0, 255),    # Red (BGR format)
            (0, 255, 0),    # Green
            (255, 0, 0),    # Blue
        ]
        color_idx = (self.frame_count // 60) % 3
        color = colors[color_idx]

        # Draw rectangle (sign)
        if 0 <= x_pos < self.width:
            cv2.rectangle(img,
                         (x_pos, y_pos),
                         (x_pos + sign_width, y_pos + sign_height),
                         color,
                         -1)  # Filled

            # Add white border
            cv2.rectangle(img,
                         (x_pos, y_pos),
                         (x_pos + sign_width, y_pos + sign_height),
                         (255, 255, 255),
                         3)  # Border thickness

        # Add frame counter (for debugging)
        cv2.putText(img, f'Frame: {self.frame_count}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return img

    def timer_callback(self):
        """Publish image at fixed rate"""
        img = self.generate_test_image()

        # Convert OpenCV image to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'

        self.publisher.publish(msg)
        self.frame_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Test it:**
```bash
# Terminal 1: Run node
ros2 run teamvoldemor_simulation mock_camera_node

# Terminal 2: Check if publishing
ros2 topic hz /camera/image_raw

# Terminal 3: View in RViz2
rviz2
# Add Image display, set topic to /camera/image_raw
```

---

## 2. Classical CV Sign Detector (Phase 2 - Quick Start)

**Note:** For YOLO26 integration (recommended for production - 43% faster!), see:
- Quick start: `docs/development/YOLO26-QUICK-START.md`
- Complete guide: `docs/development/yolo26-hailo-guide.md`

Start with Classical CV for rapid prototyping, then add YOLO26 for better accuracy!

**File:** `~/teamvoldemor_ws/src/teamvoldemor_vision/teamvoldemor_vision/sign_detector_classic.py`

```python
#!/usr/bin/env python3
"""
Classical computer vision sign detector using HSV color segmentation
Detects red, green, and blue rectangular signs
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np


class SignDetectorClassic(Node):
    def __init__(self):
        super().__init__('sign_detector_classic')

        # Parameters
        self.declare_parameter('min_area', 1000)  # Minimum contour area
        self.declare_parameter('confidence_threshold', 0.7)

        self.min_area = self.get_parameter('min_area').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)

        # Optional: Publish debug image with bounding boxes
        self.debug_pub = self.create_publisher(Image, '/detections/debug_image', 10)

        self.bridge = CvBridge()

        # HSV color ranges for sign detection
        self.color_ranges = {
            'red': {
                'lower1': np.array([0, 100, 100]),      # Red wraps around in HSV
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 100, 100]),
                'upper2': np.array([180, 255, 255]),
            },
            'green': {
                'lower': np.array([40, 50, 50]),
                'upper': np.array([80, 255, 255]),
            },
            'blue': {
                'lower': np.array([100, 50, 50]),
                'upper': np.array([130, 255, 255]),
            }
        }

        self.get_logger().info('Sign detector (classical CV) started')

    def detect_color(self, hsv_image, color_name):
        """Detect a specific color in HSV image"""
        if color_name == 'red':
            # Red requires two ranges (wraps around hue circle)
            mask1 = cv2.inRange(hsv_image,
                               self.color_ranges['red']['lower1'],
                               self.color_ranges['red']['upper1'])
            mask2 = cv2.inRange(hsv_image,
                               self.color_ranges['red']['lower2'],
                               self.color_ranges['red']['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv_image,
                              self.color_ranges[color_name]['lower'],
                              self.color_ranges[color_name]['upper'])

        # Morphological operations to reduce noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        return mask

    def find_contours(self, mask):
        """Find contours in binary mask"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter by area
        valid_contours = [c for c in contours if cv2.contourArea(c) > self.min_area]

        return valid_contours

    def image_callback(self, msg):
        """Process incoming image"""
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to HSV for color detection
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Detect each color
        detections = []
        debug_image = cv_image.copy()

        for color_name in ['red', 'green', 'blue']:
            mask = self.detect_color(hsv, color_name)
            contours = self.find_contours(mask)

            for contour in contours:
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate confidence (based on area ratio and shape)
                area = cv2.contourArea(contour)
                bbox_area = w * h
                fill_ratio = area / bbox_area if bbox_area > 0 else 0

                # Simple confidence: rectangles have high fill ratio
                confidence = min(fill_ratio * 1.2, 1.0)

                if confidence >= self.confidence_threshold:
                    # Create detection message
                    detection = Detection2D()
                    detection.header = msg.header

                    # Bounding box
                    detection.bbox.center.position.x = float(x + w / 2)
                    detection.bbox.center.position.y = float(y + h / 2)
                    detection.bbox.size_x = float(w)
                    detection.bbox.size_y = float(h)

                    # Classification
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = color_name
                    hypothesis.hypothesis.score = confidence
                    detection.results.append(hypothesis)

                    detections.append(detection)

                    # Draw on debug image
                    color_bgr = {
                        'red': (0, 0, 255),
                        'green': (0, 255, 0),
                        'blue': (255, 0, 0),
                    }[color_name]

                    cv2.rectangle(debug_image, (x, y), (x + w, y + h), color_bgr, 3)
                    cv2.putText(debug_image,
                               f'{color_name} {confidence:.2f}',
                               (x, y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX,
                               0.6,
                               color_bgr,
                               2)

        # Publish detections
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        detection_array.detections = detections
        self.detection_pub.publish(detection_array)

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

        # Log
        if detections:
            self.get_logger().info(f'Detected {len(detections)} signs')


def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorClassic()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Test it:**
```bash
# Terminal 1: Mock camera
ros2 run teamvoldemor_simulation mock_camera_node

# Terminal 2: Detector
ros2 run teamvoldemor_vision sign_detector_classic

# Terminal 3: Check detections
ros2 topic echo /detections

# Terminal 4: View debug image in RViz2
rviz2
# Add Image display, set topic to /detections/debug_image
```

---

## 3. Simple State Machine (Phase 3)

**File:** `~/teamvoldemor_ws/src/teamvoldemor_control/teamvoldemor_control/state_machine.py`

```python
#!/usr/bin/env python3
"""
Simple state machine for race control
States: IDLE, RACING, TURNING_LEFT, TURNING_RIGHT, STOPPING
"""

from enum import Enum, auto
from typing import Optional


class RaceState(Enum):
    """Robot states during race"""
    IDLE = auto()
    RACING = auto()
    TURNING_LEFT = auto()
    TURNING_RIGHT = auto()
    STOPPING = auto()


class StateMachine:
    def __init__(self):
        self.state = RaceState.IDLE
        self.previous_state = None

        # State durations (seconds)
        self.turn_duration = 2.0
        self.turn_start_time = None

    def get_state(self) -> RaceState:
        """Get current state"""
        return self.state

    def transition_to(self, new_state: RaceState, current_time: float):
        """Transition to new state"""
        if new_state != self.state:
            self.previous_state = self.state
            self.state = new_state

            # Record time for timed transitions
            if new_state in [RaceState.TURNING_LEFT, RaceState.TURNING_RIGHT]:
                self.turn_start_time = current_time

            return True
        return False

    def update(self, current_time: float, detected_sign: Optional[str] = None) -> RaceState:
        """
        Update state machine based on time and detections

        Args:
            current_time: Current time in seconds
            detected_sign: Color of detected sign ('red', 'green', 'blue', or None)

        Returns:
            New state (may be same as current)
        """

        # State transition logic
        if self.state == RaceState.IDLE:
            # Start racing when ready
            self.transition_to(RaceState.RACING, current_time)

        elif self.state == RaceState.RACING:
            # React to signs
            if detected_sign == 'red':
                self.transition_to(RaceState.TURNING_RIGHT, current_time)
            elif detected_sign == 'green':
                self.transition_to(RaceState.TURNING_LEFT, current_time)
            elif detected_sign == 'blue':
                # Blue sign could mean "speed up" or other action
                pass

        elif self.state == RaceState.TURNING_LEFT:
            # Turn for fixed duration, then return to racing
            if current_time - self.turn_start_time > self.turn_duration:
                self.transition_to(RaceState.RACING, current_time)

        elif self.state == RaceState.TURNING_RIGHT:
            # Turn for fixed duration, then return to racing
            if current_time - self.turn_start_time > self.turn_duration:
                self.transition_to(RaceState.RACING, current_time)

        elif self.state == RaceState.STOPPING:
            # Terminal state
            pass

        return self.state

    def get_target_velocity(self):
        """
        Get target velocity based on current state

        Returns:
            (linear_velocity, angular_velocity) in m/s and rad/s
        """
        if self.state == RaceState.IDLE:
            return (0.0, 0.0)

        elif self.state == RaceState.RACING:
            return (2.0, 0.0)  # Straight ahead at 2 m/s

        elif self.state == RaceState.TURNING_LEFT:
            return (1.0, 0.5)  # Slow down and turn left

        elif self.state == RaceState.TURNING_RIGHT:
            return (1.0, -0.5)  # Slow down and turn right

        elif self.state == RaceState.STOPPING:
            return (0.0, 0.0)

        return (0.0, 0.0)
```

**Unit test:** `~/teamvoldemor_ws/src/teamvoldemor_control/test/test_state_machine.py`

```python
import pytest
from teamvoldemor_control.state_machine import StateMachine, RaceState


def test_initial_state():
    sm = StateMachine()
    assert sm.get_state() == RaceState.IDLE


def test_start_racing():
    sm = StateMachine()
    new_state = sm.update(0.0)
    assert new_state == RaceState.RACING


def test_red_sign_turns_right():
    sm = StateMachine()
    sm.update(0.0)  # Start racing
    new_state = sm.update(1.0, detected_sign='red')
    assert new_state == RaceState.TURNING_RIGHT


def test_green_sign_turns_left():
    sm = StateMachine()
    sm.update(0.0)  # Start racing
    new_state = sm.update(1.0, detected_sign='green')
    assert new_state == RaceState.TURNING_LEFT


def test_turn_completes_after_duration():
    sm = StateMachine()
    sm.update(0.0)  # Start racing
    sm.update(1.0, detected_sign='red')  # Start turning

    # Still turning
    state = sm.update(2.5)
    assert state == RaceState.TURNING_RIGHT

    # Turn complete
    state = sm.update(4.0)
    assert state == RaceState.RACING


def test_velocity_commands():
    sm = StateMachine()

    # Idle = stopped
    v, w = sm.get_target_velocity()
    assert v == 0.0 and w == 0.0

    # Racing = forward
    sm.update(0.0)
    v, w = sm.get_target_velocity()
    assert v > 0.0 and w == 0.0

    # Turning left
    sm.update(1.0, detected_sign='green')
    v, w = sm.get_target_velocity()
    assert v > 0.0 and w > 0.0  # Positive angular = left
```

**Run tests:**
```bash
cd ~/teamvoldemor_ws
colcon test --packages-select teamvoldemor_control
colcon test-result --verbose
```

---

## 4. Decision Node (Phase 3)

**File:** `~/teamvoldemor_ws/src/teamvoldemor_control/teamvoldemor_control/decision_node.py`

```python
#!/usr/bin/env python3
"""
Decision node - connects vision detections to motor commands via state machine
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
from .state_machine import StateMachine, RaceState


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # State machine
        self.sm = StateMachine()

        # Subscriber to vision detections
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop (50 Hz)
        self.timer = self.create_timer(0.02, self.control_loop)

        # Latest detection
        self.latest_sign = None

        self.get_logger().info('Decision node started')

    def detection_callback(self, msg: Detection2DArray):
        """Receive vision detections"""
        if msg.detections:
            # Take highest confidence detection
            best_detection = max(msg.detections,
                               key=lambda d: d.results[0].hypothesis.score)
            self.latest_sign = best_detection.results[0].hypothesis.class_id

            self.get_logger().info(f'Detected: {self.latest_sign}')
        else:
            self.latest_sign = None

    def control_loop(self):
        """Main control loop - runs at 50 Hz"""
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Update state machine
        state = self.sm.update(current_time, self.latest_sign)

        # Get velocity from state machine
        linear_vel, angular_vel = self.sm.get_target_velocity()

        # Publish velocity command
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)

        # Clear detection after processing (don't react to same sign repeatedly)
        if self.latest_sign:
            self.latest_sign = None


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Test full pipeline:**
```bash
# Terminal 1: Mock camera
ros2 run teamvoldemor_simulation mock_camera_node

# Terminal 2: Vision detector
ros2 run teamvoldemor_vision sign_detector_classic

# Terminal 3: Decision node
ros2 run teamvoldemor_control decision_node

# Terminal 4: Monitor velocity commands
ros2 topic echo /cmd_vel

# You should see:
# - Camera publishes images
# - Detector finds colored rectangles
# - Decision node outputs velocity commands based on color
```

---

## 5. Launch File to Start Everything

**File:** `~/teamvoldemor_ws/src/teamvoldemor_bringup/launch/simulation.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Mock camera
        Node(
            package='teamvoldemor_simulation',
            executable='mock_camera_node',
            name='camera',
            parameters=[{
                'fps': 30,
                'width': 640,
                'height': 480,
            }],
            output='screen',
        ),

        # Sign detector
        Node(
            package='teamvoldemor_vision',
            executable='sign_detector_classic',
            name='sign_detector',
            parameters=[{
                'min_area': 1000,
                'confidence_threshold': 0.7,
            }],
            output='screen',
        ),

        # Decision node
        Node(
            package='teamvoldemor_control',
            executable='decision_node',
            name='decision',
            output='screen',
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
```

**Launch entire system:**
```bash
ros2 launch teamvoldemor_bringup simulation.launch.py
```

---

## 6. Package Configuration

Don't forget to update `setup.py` for each package to register executables!

**Example:** `~/teamvoldemor_ws/src/teamvoldemor_simulation/setup.py`

```python
from setuptools import setup

package_name = 'teamvoldemor_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Simulation nodes for teamvoldemor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_camera_node = teamvoldemor_simulation.mock_camera_node:main',
        ],
    },
)
```

Repeat for other packages (`teamvoldemor_vision`, `teamvoldemor_control`).

---

## Next Steps

1. Copy these files into your workspace
2. Update `setup.py` for each package
3. Build: `colcon build --symlink-install`
4. Test each node individually
5. Launch full system
6. Iterate and improve!

**You now have a working skeleton to build on! 🎉**
