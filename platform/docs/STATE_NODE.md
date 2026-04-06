This takes your robot to a completely different level! Adding a **Hailo AI processor** means you are moving from basic OpenCV color thresholding to advanced machine learning (like YOLO object detection), and using **Ackermann steering** means your robot drives like a real car rather than a tank.

These additions change exactly *what* you need to monitor to keep the robot healthy. For example, Ackermann steering cannot turn in place, so your steering angles are critical, and the Hailo chip needs to confirm its neural network model is loaded.

Here is the updated 4-Stage State Machine and the precise OLED layouts tailored for your Hailo and Ackermann architecture. 

---

### The Updated 4-Stage State Machine

#### 1. State: `BOOT_CHECK`
*   **The Logic:** Your UI node now subscribes to the Hailo inference topic (e.g., `/hailo/detections`) and your Ackermann drive hardware interface. 
*   **The Critical Check:** Hailo chips take a few seconds to load the `.hef` neural network model into their hardware upon boot. The robot *must not* start until the Hailo node confirms the model is loaded and inferencing.
*   **What it displays:**
```text
+-----------------------+
| SYSTEM BOOTING...     |
|                       |
| IMU:   [ OK ]         |
| LiDAR: [ OK ]         |
| Hailo: [WAIT] NPU Load|
| Drive: [ OK ]         |
+-----------------------+
```

#### 2. State: `READY`
*   **The Logic:** All nodes are publishing. The NPU is hot, the IMU is calibrated, and the Ackermann steering servo is centered and holding.
*   **What it displays:** Notice how we now display the AI Model and the IP address.
```text
+-----------------------+
| *** READY TO GO ***   |
| IP: 192.168.1.104     |
|                       |
| Model: YOLOv8_WRO.hef |
| Steering: CENTERED    |
| [PRESS BTN TO START]  |
+-----------------------+
```

#### 3. State: `RACING` (The Auto-Carousel)
*   **The Logic:** The robot is driving. The UI auto-cycles every 3 seconds to show you the brain of the robot. 

**Page A: Ackermann Drive Control**
Because you are using an Ackermann chassis, you don't care about "Left Motor / Right Motor PWM" anymore. You care about your `ackermann_msgs/AckermannDrive` topic, which commands **Speed (m/s)** and **Steering Angle**. 
*If your robot hits a wall, this page tells you if the software *told* it to turn, or if the steering servo just physically jammed.*
```text
+-----------------------+
| RACING        Lap 1/3 |
| State: AVOID_RED      |
|                       |
| Cmd Vel:  1.20 m/s    |
| Cmd Ang: +18.5 deg (R)|
| Gyro Yaw: 45.2 deg    |
+-----------------------+
```

**Page B: Hailo AI Telemetry**
This replaces the old basic vision page. It shows exactly what the Hailo NPU is detecting, the confidence score, and the inference speed.
```text
+-----------------------+
| HAILO NPU VISION      |
| NPU FPS: 30           |
|                       |
| Lock: RED_PILLAR (94%)|
| Box: [x:320, y:140]   |
| Est Dist: 65 cm       |
+-----------------------+
```

**Page C: LiDAR & Spatial**
With Ackermann steering, your robot has a "minimum turning radius." If a wall is closer than that radius, the robot is physically incapable of avoiding it. This page helps you debug if your path-planning algorithm is plotting impossible turns.
```text
+-----------------------+
| LIDAR CLEARANCE       |
|                       |
| Front: 120 cm         |
| Left:   15 cm         |
| Right:  80 cm         |
| Status: PATH_CLEAR    |
+-----------------------+
```

#### 4. State: `FINISHED / E-STOP`
*   **The Logic:** Triggered by the 3rd lap completion or a 2-second long press on your GP0 button. Sends an `ackermann_cmd` of `speed: 0.0, steering_angle: 0.0`.
*   **What it displays:**
```text
+-----------------------+
| RACE FINISHED         |
| Status: SUCCESS       |
|                       |
| Laps Completed: 3     |
| Total Time: 02:14.5   |
| [PRESS TO RESTART]    |
+-----------------------+
```

### Pro-Tip for your Ackermann ROS 2 Nodes
Since you are using Ackermann steering, your math heavily relies on the **Wheelbase** (distance between front and rear axles) and **Track Width** (distance between left and right wheels). 

Make sure your UI Node and your Navigation Node are pulling these physical measurements from the exact same ROS 2 `parameters.yaml` file. If you tune the steering in your navigation node but forget to update it elsewhere, your OLED debug screens (like calculating if a turn is possible based on LiDAR) will lie to you!

That is a fantastic design choice. In competition environments, Wi-Fi can be incredibly unreliable, banned entirely, or you might just want to run the robot completely "headless" without connecting a router. The robot must be able to complete its WRO run even if it is completely isolated from the outside world.

Here is how you adjust the logic so the IP address becomes a "nice-to-have" diagnostic rather than a mandatory passing condition.

### The Logic Update (Non-Blocking IP Check)

In your `BOOT_CHECK` phase, your Python code should attempt to fetch the IP address (using a library like `socket` or `netifaces`), but wrap it in a `try/except` block. 

*   If it finds a valid `192.168.x.x` or `10.x.x.x` address, it saves it to a variable.
*   If it fails, times out, or only finds the local loopback (`127.0.0.1`), it simply sets the variable to `"NO NETWORK"` or `"OFFLINE"` and **allows the system to proceed to `READY` anyway.**

### The Updated `READY` Screen

Because you have 128x64 pixels, you can conditionally format that specific line on the OLED based on what the network check found.

**Scenario A: Connected to your router/laptop (Wi-Fi is up)**
```text
+-----------------------+
| *** READY TO GO ***   |
| IP: 192.168.1.104     |
|                       |
| Model: YOLOv8_WRO.hef |
| Steering: CENTERED    |
| [PRESS BTN TO START]  |
+-----------------------+
```

**Scenario B: Competition Mode (No Wi-Fi, fully autonomous)**
```text
+-----------------------+
| *** READY TO GO ***   |
| Net: [LOCAL / OFFLINE]|
|                       |
| Model: YOLOv8_WRO.hef |
| Steering: CENTERED    |
| [PRESS BTN TO START]  |
+-----------------------+
```

### Quick Python Snippet for the Non-Blocking Fetch
Here is a very clean, robust way to grab that IP address in your UI Node without freezing your ROS 2 startup sequence:

```python
import socket

def get_ip_address():
    """Attempts to get the active IP address, returning a fallback if offline."""
    try:
        # Create a dummy socket to figure out the default routing interface
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # It doesn't actually connect to 8.8.8.8, just checks the route
        s.connect(("8.8.8.8", 80)) 
        ip = s.getsockname()[0]
        s.close()
        return f"IP: {ip}"
    except Exception:
        # If there's no route/network, it fails gracefully
        return "Net: OFFLINE"

# In your state machine:
# self.display_ip = get_ip_address()
```

This perfectly aligns with the WRO philosophy: your robot should be completely self-reliant. If the Hailo AI is loaded, the Ackermann servos are centered, and the IMU is calibrated, the robot is ready to race, network or not!

You are spot on. Hardcoding a 3-second delay deep in your logic is a bad practice. Moving it to a constant at the top of your file makes tuning your robot much easier, and you are right—3 seconds is an eternity when a robot is driving at 1.5 m/s. 

If your robot crashes, you want to see the relevant data immediately, not wait 3 seconds for the screen to roll over to the LiDAR page.

Here is how you implement that efficiently in your ROS 2 node, keeping two very important timings separate:
1.  **The Data Refresh Rate:** How fast the numbers on the screen update (e.g., 5 times a second).
2.  **The Page Cycle Rate:** How fast the screen completely swaps to the next view (e.g., every 1.2 seconds).

### The "Strobe Light" Warning
When picking your new, faster interval, remember that the human brain takes about 0.3 to 0.5 seconds just to recognize what page it is looking at, and another half-second to read the numbers. 

If you set the cycle interval to something extremely fast like `0.3` seconds, the screen will just look like a flickering strobe light. **A constant between `1.0` and `1.5` seconds is usually the sweet spot for a fast-moving WRO robot.**

### How to Code the Constant and Timers

In ROS 2, the cleanest way to do this without freezing up your node is to use your high-speed UI Timer to check a clock, rather than using `time.sleep()`.

```python
import time

# --- CONFIGURATION CONSTANTS ---
PAGE_CYCLE_INTERVAL = 1.2  # Seconds before swapping to the next telemetry page
UI_REFRESH_RATE = 0.1      # Update the data on the screen at 10Hz (0.1s)
# -------------------------------

class RobotUI:
    def __init__(self, oled):
        self.oled = oled
        self.state = "RACING"
        
        self.current_page = 0
        self.total_pages = 3
        
        # Track exactly when we last flipped the page
        self.last_page_flip_time = time.time()

    def ui_timer_callback(self):
        """This function is called by a ROS Timer every 0.1 seconds (10Hz)"""
        
        if self.state == "RACING":
            current_time = time.time()
            
            # 1. Check if it's time to swap to the next page
            if (current_time - self.last_page_flip_time) >= PAGE_CYCLE_INTERVAL:
                self.current_page = (self.current_page + 1) % self.total_pages
                self.last_page_flip_time = current_time # Reset the stopwatch
            
            # 2. Draw the current page with fresh, live data
            self.draw_racing_telemetry()

    def draw_racing_telemetry(self):
        # The numbers inside here will update at the fast 10Hz rate, 
        # but the layout only swaps every 1.2 seconds.
        if self.current_page == 0:
            self.oled.draw_ackermann_page(live_speed, live_steering)
        elif self.current_page == 1:
            self.oled.draw_hailo_page(live_target, live_fps)
        elif self.current_page == 2:
            self.oled.draw_lidar_page(live_left_dist, live_right_dist)
```

By setting it up this way, your steering angles and LiDAR distances will look buttery smooth and update instantly as the robot drives, but the physical layout only swaps every `1.2` seconds. 

If you get to the track and find 1.2 seconds is still too slow, you just change `PAGE_CYCLE_INTERVAL` at the top of your file, rebuild, and you are good to go!
