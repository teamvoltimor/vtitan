"""Robot physical and sensor constants -- physical fields from robot.toml, plus hand-maintained LIDAR/IMU/camera simulation parameters."""

from __future__ import annotations

import math
from typing import Final

from shared.config.constants._shared import _robot


class RobotSpecs:
    """WRO Future Engineers robot specs (vTitan + Ackermann).

    Physical constants (chassis, Ackermann geometry, wheel, LIDAR/camera mount offsets) are
    sourced from platform/config/robot.toml via RobotConstants — see
    shared.config.robot_constants — and must not be hand-edited here. Everything else in this
    class (LIDAR/IMU/camera sim parameters) is not duplicated in Go/xacro and stays
    hand-maintained.
    """

    # Chassis dimensions
    LENGTH: Final[float] = _robot.chassis.length  # 300mm chassis length
    WIDTH: Final[float] = _robot.chassis.width  # 200mm chassis width
    HEIGHT: Final[float] = _robot.chassis.height  # 100mm chassis height

    # Ackermann geometry (measured 2026-07-11)
    WHEELBASE: Final[float] = _robot.ackermann.wheelbase  # 190mm axle-to-axle distance
    TRACK_WIDTH: Final[float] = _robot.ackermann.track_width  # 167.5mm wheel-to-wheel distance
    WHEEL_RADIUS: Final[float] = _robot.wheel.radius  # 35mm (measured 70mm wheel diameter / 2)
    MAX_STEERING_ANGLE: Final[float] = _robot.steering.max_steering_angle
    """Largest road-wheel angle the navigator may COMMAND (radians).

    ``steering_limit_deg`` when the config sets one, otherwise the linkage's
    full travel (``MAX_WHEEL_ANGLE_DEG``). This is the value planners, the
    pursuit controller and the simulator gate on. Everything upstream of the
    servo speaks wheel angles.

    Not interchangeable with MAX_WHEEL_ANGLE_DEG any more -- see Steering's
    class docstring for the 1.55x servo error that conflating them produces."""

    # Steering hardware. The servo speaks servo degrees; the linkage converts.
    SERVO_MAX_ANGLE_DEG: Final[float] = _robot.steering.servo_max_angle_deg
    MAX_WHEEL_ANGLE_DEG: Final[float] = _robot.steering.max_wheel_angle_deg
    """Road-wheel angle (deg) the LINKAGE produces at full servo lock -- physics,
    not policy. See robot.toml's [steering] comment: re-measure and update this
    after any linkage/servo change rather than hand-computing a ratio, and never
    lower it to mean 'steer more gently' (use steering_limit_deg for that)."""
    LINKAGE_RATIO: Final[float] = _robot.steering.linkage_ratio
    """Road-wheel degrees per servo degree, derived from MAX_WHEEL_ANGLE_DEG /
    SERVO_MAX_ANGLE_DEG. Not the source of truth -- kept for consumers that
    convert an arbitrary wheel angle to a servo angle."""

    # Wheel details (measured 2026-07-11)
    WHEEL_WIDTH: Final[float] = _robot.wheel.width  # 25mm
    WHEEL_MASS: Final[float] = _robot.wheel.mass  # 50g per wheel
    CHASSIS_MASS: Final[float] = _robot.chassis.mass  # body alone, without wheels
    WHEEL_COUNT: Final[int] = 4
    TOTAL_MASS: Final[float] = CHASSIS_MASS + WHEEL_COUNT * WHEEL_MASS
    """Assembled car, 1.5 kg measured 2026-08-01.

    Derived rather than declared: the URDF and the Gazebo model build the robot
    out of a body plus four wheels, so the total is a consequence of those
    masses. Declaring it separately would let it disagree with the model that
    actually runs."""

    # Drivetrain limits (from robot.toml). Hard ceilings the kinematics clamp
    # to, not tuning: a speed profile asking for more is inert.
    MAX_SPEED_MPS: Final[float] = _robot.drivetrain.max_speed_mps
    MAX_ACCEL_MPS2: Final[float] = _robot.drivetrain.max_accel_mps2
    REAR_STEER_RATIO: Final[float] = _robot.drivetrain.rear_steer_ratio

    # Measured departures from the textbook model, not ceilings: these two say
    # how the real chassis MISSES what the kinematics predict, so unlike the
    # limits above they apply on every step rather than only at saturation.
    SPEED_RESPONSE_TAU_S: Final[float] = _robot.drivetrain.speed_response_tau_s
    YAW_GAIN: Final[float] = _robot.drivetrain.yaw_gain

    # LIDAR (Slamtec C1) — mounted upside-down, centered left/right, at the front of the
    # chassis (measured 2026-07-11). See docs/robot-physical-constants.md.
    LIDAR_MIN_RANGE: Final[float] = _robot.lidar.min_range
    """Closest range the sensor can report (m), from robot.toml [lidar].

    Was hand-written here as 0.05 until 2026-08-21; the C1 measures to about
    0.045, and a sensor spec belongs in the hardware description rather than
    beside the simulation parameters below. LIDAR_SIM_MIN_RANGE (0.01, "clamp to
    50mm in callback") sat next to it with ZERO consumers repo-wide and was
    removed in the same change -- the simulator clamps to this value, so a second
    floor that nothing read only suggested a near-wall behaviour that did not
    exist."""

    LIDAR_MAX_RANGE: Final[float] = _robot.lidar.max_range  # 12m maximum detection range
    LIDAR_SAMPLES: Final[int] = _robot.lidar.samples  # Slamtec C1 horizontal samples
    LIDAR_UPDATE_RATE: Final[float] = _robot.lidar.update_rate  # 10 Hz scan rate
    LIDAR_NOISE_STDDEV: Final[float] = _robot.lidar.noise_stddev  # 30mm noise
    # 55.6mm diameter x 41.3mm height: matches the lidar_link visual/collision mesh already
    # modeled in wro_robot.urdf.xacro (radius=0.0278, length=0.0413) — used here instead of
    # the C1's raw datasheet form factor so the mount-offset derivation below stays
    # self-consistent with the mesh actually rendered in sim.
    LIDAR_DIAMETER: Final[float] = _robot.lidar.diameter
    LIDAR_HEIGHT: Final[float] = _robot.lidar.height
    # = LENGTH/2 - LIDAR_DIAMETER/2 = 0.15 - 0.0278: the C1 mounted flush with the front
    # edge, offset back by its own puck radius (same derivation style as the camera mount
    # offset below). Cross-checked against the existing z-mount height (HEIGHT + LIDAR_HEIGHT/2
    # = 0.10 + 0.0207 ~= 0.1207, matching the long-standing z=0.12 lidar_link offset in
    # static_tfs.launch.py / the URDF within rounding).
    LIDAR_MOUNT_X_OFFSET: Final[float] = _robot.lidar.mount_x_offset
    LIDAR_TO_FRONT_BUMPER: Final[float] = _robot.chassis.length / 2 - _robot.lidar.mount_x_offset
    """Metres from the sensor to the FRONT bumper face (~0.028).

    A forward range reading MINUS this is the true gap between the bumper and
    the obstacle. Thresholds compared against a raw range are implicitly
    measured from wherever the sensor happens to sit, which makes them move
    when the mount moves -- the same defect as a fractional constant, in a
    physical unit. Subtracting this converts to a chassis-referenced gap, so
    re-mounting the sensor is calibration (this value re-derives) rather than a
    retune of every threshold.
    """

    LIDAR_TO_REAR_BUMPER: Final[float] = _robot.chassis.length / 2 + _robot.lidar.mount_x_offset
    """Metres from the sensor to the REAR bumper face (~0.272).

    Nearly TEN TIMES the front figure, because the sensor sits at the front.
    That asymmetry is why one raw-range threshold cannot serve both ends: with
    the shipped 0.10 m contact distance, an obstacle touching the rear bumper
    reads 0.272 m and the gate never fires, while the same 0.10 m at the front
    is a real near-contact. Measured 2026-08-22 as the cause of a reverse guard
    that could not fire before impact.
    """

    # LIDAR sits above the chassis top by this much; add HEIGHT for the LIDAR's absolute
    # mount z (matches the long-standing z=0.12 in static_tfs.launch.py / the URDF).
    LIDAR_MOUNT_Z_OFFSET: Final[float] = _robot.lidar.mount_z_offset
    # Single source of truth for the upside-down mount. Drives BOTH the sllidar_ros2
    # driver's own `inverted` launch parameter AND a mandatory 180deg yaw rotation
    # wherever raw /scan angles are consumed -- see robot.toml's [lidar] section for
    # why these must never be set independently again.
    LIDAR_INVERTED: Final[bool] = _robot.lidar.inverted
    # Residual yaw miscalibration NOT explained by LIDAR_INVERTED's 180deg -- added on
    # top of it, not a replacement. Use lidar_yaw_offset_rad() below for the
    # combined correction rather than reading these two fields separately.
    LIDAR_MOUNT_YAW_OFFSET_DEG: Final[float] = _robot.lidar.mount_yaw_offset_deg
    # LIDAR_SELF_DETECTION_THRESHOLD moved to NavigationTuning's LidarSectorParams
    # (platform/config/navigation/sensors/lidar_sectors.toml) -- it's
    # collision-logic tuning, not physical geometry, unlike everything else in
    # this class.

    @classmethod
    def lidar_yaw_offset_rad(cls) -> float:
        """Rotates raw /scan bearings into the robot frame (0 rad = forward).

        Combines two independent facts -- LIDAR_INVERTED (mandatory 180deg
        when the mount is upside-down, the same fact that also drives the
        sllidar_ros2 driver's own `inverted` launch parameter) and
        LIDAR_MOUNT_YAW_OFFSET_DEG (any additional residual miscalibration,
        added on top of the 180deg, not a replacement for it) -- see
        robot.toml's [lidar] section.

        A classmethod rather than a stored Final constant so every consumer
        of raw LIDAR angles calls this instead of recomputing the formula
        locally. That duplication is exactly how telemetry_bridge_node.py's
        own copy went stale: written 2026-07-28, before LIDAR_INVERTED
        existed as a separate flag, it silently kept applying only the
        residual offset while ros2_hardware_gateway.py's and
        static_tfs.launch.py's copies got the 180deg term added 2026-08-02.
        Confirmed live on hardware 2026-08-09: the OLED's F/L/R readout was
        still un-inverted (front read as back, left read as right) despite
        the driver and the navigation path both being correct.
        """
        return math.radians((180.0 if cls.LIDAR_INVERTED else 0.0) + cls.LIDAR_MOUNT_YAW_OFFSET_DEG)

    # IMU (Adafruit BNO085)
    IMU_UPDATE_RATE: Final[float] = _robot.imu.update_rate  # 100 Hz update rate
    IMU_GYRO_NOISE: Final[float] = _robot.imu.gyro_noise  # rad/s gyroscope noise stddev
    IMU_ACCEL_NOISE: Final[float] = _robot.imu.accel_noise  # m/s² accelerometer noise stddev
    IMU_MASS: Final[float] = _robot.imu.mass  # 2.5g board mass
    IMU_SIZE: Final[tuple[float, float, float]] = _robot.imu.size  # 25.6mm × 22.7mm × 4.6mm
    # IMU sits above the chassis floor by this much (matches the long-standing z=0.01 in
    # static_tfs.launch.py / the URDF).
    IMU_MOUNT_Z_OFFSET: Final[float] = _robot.imu.mount_z_offset

    # Camera (RPi Camera 3 Wide) — mounted above the LIDAR, angled down (measured 2026-07-11,
    # approximate; see docs/robot-physical-constants.md).
    CAMERA_HFOV: Final[float] = _robot.camera.hfov  # 102 degrees horizontal FOV (radians)
    CAMERA_WIDTH: Final[int] = _robot.camera.width  # Horizontal resolution (pixels)
    CAMERA_HEIGHT: Final[int] = _robot.camera.height  # Vertical resolution (pixels)
    CAMERA_UPDATE_RATE: Final[float] = _robot.camera.update_rate  # 30 FPS
    CAMERA_NEAR_CLIP: Final[float] = _robot.camera.near_clip  # 50mm near clip
    CAMERA_FAR_CLIP: Final[float] = _robot.camera.far_clip  # 10m far clip
    # Directly over the LIDAR (same x as LIDAR_MOUNT_X_OFFSET), mounted above its top edge
    # (LIDAR z=0.12 + LIDAR_HEIGHT/2 ~= 0.14) with a small mounting-bracket gap. Unlike
    # LIDAR_MOUNT_X_OFFSET, this z isn't derived from a datasheet — it's an estimate pending
    # a real measurement.
    CAMERA_MOUNT_X_OFFSET: Final[float] = _robot.camera.mount_x_offset
    CAMERA_MOUNT_Z_OFFSET: Final[float] = _robot.camera.mount_z_offset
    CAMERA_MOUNT_PITCH_DEG: Final[float] = math.degrees(
        _robot.camera.mount_pitch,
    )  # tilted down; angle is an estimate ("~30")
