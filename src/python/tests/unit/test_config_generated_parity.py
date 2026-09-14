"""Parity checks for hand-written config wrappers over generated DTOs.

Each migrated module must keep the public names and the exact loaded values the
old hand-written model produced. These tests load the REAL shipped config (the
same tree the robot reads) and pin the values, so a wrapper that silently
switched to a generated default or lost a field fails here rather than on the
car.

The subclass assertions are the migration contract itself: the wrapper is now
anchored to the generated DTO, and the generated field set is what the wrapper
validates.
"""

from __future__ import annotations

import sys
from pathlib import Path
from unittest import mock

# The display and IMU config modules live behind package __init__s that eagerly
# import their hardware transports (board/busio/adafruit_ssd1306/fcntl and the
# adafruit BNO08x bindings), none of which import off-hardware. Substitute only
# where the real module cannot be imported, matching tests/ros2's pattern.
for _optional in (
    "board",
    "busio",
    "adafruit_ssd1306",
    "adafruit_bno08x",
    "adafruit_bno08x.i2c",
    "adafruit_bno08x_rvc",
    "fcntl",
):
    if _optional not in sys.modules:
        try:
            __import__(_optional)
        except (ImportError, NotImplementedError):
            sys.modules[_optional] = mock.MagicMock()

import pytest
from shared.config.constants.simulation import CompetitionSpecs
from shared.config.generated.competition_specs_schema import CompetitionSpecs as CompetitionSpecsDTO
from shared.config.generated.hardware.motors.encoder_schema import HardwareMotorsEncoder
from shared.config.generated.hardware.motors.servo_schema import HardwareMotorsServo
from shared.config.generated.ros_topics_schema import RosTopics
from shared.config.ros_topics import RosMessageType, RosTopicConfig, StateMachineTopics

from src.hardware.motors.encoder import EncoderConfig
from src.hardware.motors.servo import ServoConfig

# The per-board node implementations (button_node, ackermann_motor_node, ...)
# live across ament_python packages under ros2_ws/src/vtitan_*, none of which
# are on PYTHONPATH (they normally require a colcon build). tests/ros2/conftest
# adds each source dir for the ROS2 suite; the unit suite has no such conftest,
# so the same bootstrap is repeated here for the node NodeConfig parity checks.
_ROS2_WS_SRC = Path(__file__).resolve().parents[2] / "ros2_ws" / "src"
for _pkg_dir in sorted(_ROS2_WS_SRC.glob("vtitan_*")):
    if str(_pkg_dir) not in sys.path:
        sys.path.insert(0, str(_pkg_dir))


class TestGeneratedAnchoring:
    def test_ros_topic_config_subclasses_generated_dto(self):
        assert issubclass(RosTopicConfig, RosTopics)
    def test_competition_specs_model_subclasses_generated_dto(self):
        from shared.config.constants.simulation import _CompetitionSpecsModel

        assert issubclass(_CompetitionSpecsModel, CompetitionSpecsDTO)

    def test_servo_config_subclasses_generated_dto(self):
        assert issubclass(ServoConfig, HardwareMotorsServo)

    def test_encoder_config_subclasses_generated_dto(self):
        assert issubclass(EncoderConfig, HardwareMotorsEncoder)


class TestRosTopicsLoadedValues:
    """ros_topics.toml, as the old hand-written nested models read it."""

    def test_nested_group_alias_is_the_generated_class(self):
        from shared.config.generated.ros_topics_schema import StateMachine

        assert StateMachineTopics is StateMachine

    def test_shipped_topics(self):
        topics = RosTopicConfig.load_default()

        assert topics.state_machine.state == "/robot_state"
        assert topics.state_machine.race_metrics == "/race_metrics"
        assert topics.state_machine.system_status == "/system_status"
        assert topics.navigation.laps_completed == "/race/laps_completed"
        assert topics.navigation.current_corridor == "/race/current_corridor"
        assert topics.navigation.nav_debug == "/nav_debug"
        assert topics.navigation.odometry is None
        assert topics.sensors.scan == "/scan"
        assert topics.sensors.imu == "/imu/data"
        assert topics.commands.ackermann_cmd == "/ackermann_cmd"
        assert topics.actuators.joint_states == "/joint_states"
        assert topics.simulation.robot_model == "/sim/robot_model"

    def test_message_type_enum_is_preserved(self):
        assert RosMessageType.STRING == "std_msgs/msg/String"


class TestCompetitionSpecsLoadedValues:
    def test_shipped_rules(self):
        assert pytest.approx(180.0) == CompetitionSpecs.ROUND_TIME_LIMIT_S
        assert CompetitionSpecs.OPEN_CHALLENGE_LAPS == 3
        assert CompetitionSpecs.OBSTACLE_CHALLENGE_LAPS == 3


class TestHardwareLoaderLoadedValues:
    """The servo and encoder TOMLs (plus the active profile overlay)."""

    def test_servo_values(self):
        servo = ServoConfig()

        assert servo.gpio_pin == 12
        assert servo.pwmchip == 0
        assert servo.pwm_channel == 0
        assert servo.min_pulse_us == pytest.approx(500.0)
        assert servo.max_pulse_us == pytest.approx(2500.0)
        assert servo.range_deg == pytest.approx(270.0)  # active profile overlay
        assert servo.center_pulse_us == pytest.approx(1500.0)
        assert servo.reversed is False
        assert servo.pwm_frequency_hz == 50

    def test_encoder_values(self):
        encoder = EncoderConfig.load()

        assert encoder.pin_a == 16
        assert encoder.pin_b == 20
        assert encoder.counts_per_rev == pytest.approx(60.0)
        assert encoder.max_rpm == pytest.approx(348.0)
        assert encoder.feedforward_deadband_duty == pytest.approx(0.20)
        assert encoder.pid_kp == pytest.approx(0.00125)
        assert encoder.pid_ki == pytest.approx(0.0025)
        assert encoder.pid_kd == pytest.approx(0.0)
        assert encoder.max_duty == pytest.approx(0.5)


class TestNavigationTuningAnchoring:
    """Every navigation tuning group is anchored to its generated DTO.

    The generated schema is the single source for the field set and the TOML
    keys; the wrapper adds only the tuning layer's shipped fallbacks and derived
    accessors.
    """

    def test_every_migrated_group_subclasses_its_generated_dto(self):
        from shared.config.generated.navigation.blind_nav.corridor_estimator_schema import (
            NavigationBlindNavCorridorEstimator,
        )
        from shared.config.generated.navigation.blind_nav.corridor_follower_schema import (
            NavigationBlindNavCorridorFollower,
        )
        from shared.config.generated.navigation.blind_nav.direction_estimator_schema import (
            NavigationBlindNavDirectionEstimator,
        )
        from shared.config.generated.navigation.blind_nav.localization_schema import (
            NavigationBlindNavLocalization,
        )
        from shared.config.generated.navigation.blind_nav.state_estimator_schema import (
            NavigationBlindNavStateEstimator,
        )
        from shared.config.generated.navigation.escape.escape_schema import (
            NavigationEscapeEscape,
        )
        from shared.config.generated.navigation.motion.clearance_schema import (
            NavigationMotionClearance,
        )
        from shared.config.generated.navigation.motion.control_schema import (
            NavigationMotionControl,
        )
        from shared.config.generated.navigation.motion.heading_schema import (
            NavigationMotionHeading,
        )
        from shared.config.generated.navigation.motion.pursuit_schema import (
            NavigationMotionPursuit,
        )
        from shared.config.generated.navigation.motion.speed_schema import (
            NavigationMotionSpeed,
        )
        from shared.config.generated.navigation.parking.parking_schema import (
            NavigationParkingParking,
        )
        from shared.config.generated.navigation.sensors.lidar_sectors_schema import (
            NavigationSensorsLidarSectors,
        )
        from shared.config.generated.navigation.sensors.sensor_schema import (
            NavigationSensorsSensor,
        )
        from shared.config.generated.navigation.sensors.start_measurement_schema import (
            NavigationSensorsStartMeasurement,
        )
        from shared.config.generated.navigation.sensors.wall_heading_schema import (
            NavigationSensorsWallHeading,
        )
        from shared.config.generated.navigation.signs.sign_discovery_schema import (
            NavigationSignsSignDiscovery,
        )
        from shared.config.generated.navigation.signs.sign_router_schema import (
            NavigationSignsSignRouter,
        )
        from shared.config.generated.navigation.simulation.simulation_schema import (
            NavigationSimulationSimulation,
        )
        from shared.config.generated.navigation.waypoint.waypoints_schema import (
            NavigationWaypointWaypoints,
        )
        from shared.config.navigation_tuning import (
            ClearanceZones,
            ControlLoopParams,
            CorridorEstimatorParams,
            CorridorFollowerParams,
            DirectionEstimatorParams,
            EscapeManeuverParams,
            HeadingErrorZones,
            LidarSectorParams,
            LocalizationParams,
            ParkingParams,
            PurePursuitParams,
            SensorHealthParams,
            SignDiscoveryParams,
            SignRouterParams,
            SimulationParams,
            SpeedControlParams,
            StartMeasurementParams,
            StateEstimatorParams,
            WallHeadingParams,
            WaypointParams,
        )

        pairs = [
            (ClearanceZones, NavigationMotionClearance),
            (HeadingErrorZones, NavigationMotionHeading),
            (PurePursuitParams, NavigationMotionPursuit),
            (SpeedControlParams, NavigationMotionSpeed),
            (ControlLoopParams, NavigationMotionControl),
            (EscapeManeuverParams, NavigationEscapeEscape),
            (SensorHealthParams, NavigationSensorsSensor),
            (LidarSectorParams, NavigationSensorsLidarSectors),
            (StartMeasurementParams, NavigationSensorsStartMeasurement),
            (WallHeadingParams, NavigationSensorsWallHeading),
            (CorridorEstimatorParams, NavigationBlindNavCorridorEstimator),
            (CorridorFollowerParams, NavigationBlindNavCorridorFollower),
            (DirectionEstimatorParams, NavigationBlindNavDirectionEstimator),
            (LocalizationParams, NavigationBlindNavLocalization),
            (StateEstimatorParams, NavigationBlindNavStateEstimator),
            (SignRouterParams, NavigationSignsSignRouter),
            (SignDiscoveryParams, NavigationSignsSignDiscovery),
            (ParkingParams, NavigationParkingParking),
            (WaypointParams, NavigationWaypointWaypoints),
            (SimulationParams, NavigationSimulationSimulation),
        ]

        for wrapper, dto in pairs:
            assert issubclass(wrapper, dto), wrapper.__name__

    def test_base_tree_values_load_through_the_generated_fields(self):
        """Values that had drifted in the old hand-written models."""
        from shared.config.navigation_tuning import DEFAULT_CONFIG_DIR, NavigationTuning

        tuning = NavigationTuning.load_from_toml_dir(DEFAULT_CONFIG_DIR)

        # Old hand defaults disagreed with the checked-in base TOML on these.
        assert tuning.clearance.risk_ray_window == 1
        assert tuning.corridor_follower.bay_exit_speed_mps == pytest.approx(0.15)
        assert tuning.sign_router.slot_sign_map is True
        assert tuning.sign_router.escape_mask_cluster_assoc_m == pytest.approx(0.35)

        # Spot checks across the newly generated-backed groups.
        assert tuning.clearance.obstacles_contact_dist == pytest.approx(0.04)
        assert tuning.pursuit.open_lookahead_long == pytest.approx(0.24)
        assert tuning.speed.creep_mps == pytest.approx(0.1014)
        assert tuning.waypoints.arc_radius == pytest.approx(0.45)
        assert tuning.simulation.vision_detect_r50_m == pytest.approx(1.10)
        assert tuning.corridor_estimator.decision_boundary_m == pytest.approx(0.8)
        assert tuning.direction_estimator.corner_clearance_m == pytest.approx(1.0)
        assert tuning.localization.max_speed_mps == pytest.approx(0.60)
        assert tuning.state_estimator.yaw_correction_gain == pytest.approx(0.05)
        assert tuning.sign_discovery.min_hits == 3


class TestHardwareConfigAnchoring:
    """Every newly migrated hardware/node wrapper is anchored to its DTO.

    The subclass assertion is the migration contract: the wrapper validates the
    generated field set, and no field is redeclared by hand.
    """

    def test_camera_config_subclasses_generated_dto(self):
        from shared.config.generated.hardware.camera.config_schema import HardwareCameraConfig

        from src.hardware.camera.config import Config as CameraConfig

        assert issubclass(CameraConfig, HardwareCameraConfig)

    def test_teleop_config_subclasses_generated_dto(self):
        from shared.config.generated.hardware.teleop_schema import HardwareTeleop

        from src.teleop.config import Config as TeleopConfig

        assert issubclass(TeleopConfig, HardwareTeleop)

    def test_lidar_launch_defaults_subclasses_generated_dto(self):
        from shared.config.generated.hardware.lidar_schema import HardwareLidar

        from src.config.launch_settings import LidarLaunchDefaults

        assert issubclass(LidarLaunchDefaults, HardwareLidar)

    def test_ros2_node_configs_subclass_generated_dtos(self):
        from shared.config.generated.hardware.button.button_node_schema import HardwareButtonButtonNode
        from shared.config.generated.hardware.challenge_mode_node_schema import HardwareChallengeModeNode
        from shared.config.generated.hardware.motors.ackermann_motor_node_schema import (
            HardwareMotorsAckermannMotorNode,
        )
        from shared.config.generated.hardware.state_machine.state_machine_node_schema import (
            HardwareStateMachineStateMachineNode,
        )
        from vtitan_drivers.button_node import NodeConfig as ButtonNodeConfig
        from vtitan_drivers.challenge_mode_node import NodeConfig as ChallengeModeNodeConfig
        from vtitan_drivers.motors.ackermann_motor_node import NodeConfig as AckermannNodeConfig
        from vtitan_state_machine.state_machine_node import NodeConfig as StateMachineNodeConfig

        assert issubclass(ButtonNodeConfig, HardwareButtonButtonNode)
        assert issubclass(ChallengeModeNodeConfig, HardwareChallengeModeNode)
        assert issubclass(AckermannNodeConfig, HardwareMotorsAckermannMotorNode)
        assert issubclass(StateMachineNodeConfig, HardwareStateMachineStateMachineNode)


class TestHardwareConfigLoadedValues:
    """The real shipped TOMLs, as the migrated wrappers read them."""

    def test_camera_values(self):
        from src.hardware.camera.config import Config as CameraConfig

        camera = CameraConfig()

        assert camera.device == "/dev/video0"
        assert camera.width == 640
        assert camera.height == 640
        assert camera.fps == 30
        assert camera.rotation == 0
        assert camera.hflip is False
        assert camera.vflip is False

    def test_teleop_values(self):
        from src.teleop.config import Config as TeleopConfig

        teleop = TeleopConfig()

        assert teleop.steering_axis_index == 0
        assert teleop.throttle_axis_index == 4
        assert teleop.deadman_button_index == 6
        assert teleop.steering_invert is False
        assert teleop.throttle_invert is False
        assert teleop.max_steering_deg == pytest.approx(30.0)
        assert teleop.max_speed_mps == pytest.approx(0.3)
        assert teleop.publish_rate_hz == pytest.approx(20.0)
        assert teleop.joy_timeout_s == pytest.approx(0.5)

    def test_lidar_launch_defaults_values(self):
        from src.config.launch_settings import LidarLaunchDefaults

        lidar = LidarLaunchDefaults()

        assert lidar.serial_port == "/dev/ttyUSB0"
        assert lidar.serial_baudrate == 460800
        assert lidar.scan_mode == "Standard"
        assert lidar.angle_compensate is True

    def test_ros2_node_config_values(self):
        from vtitan_drivers.button_node import NodeConfig as ButtonNodeConfig
        from vtitan_drivers.challenge_mode_node import NodeConfig as ChallengeModeNodeConfig
        from vtitan_drivers.motors.ackermann_motor_node import NodeConfig as AckermannNodeConfig
        from vtitan_state_machine.state_machine_node import NodeConfig as StateMachineNodeConfig

        assert ButtonNodeConfig().poll_hz == pytest.approx(20.0)

        assert ChallengeModeNodeConfig().publish_rate_hz == pytest.approx(2.0)

        ackermann = AckermannNodeConfig()
        assert ackermann.publisher_rate_hz == pytest.approx(20.0)
        assert ackermann.diagnostics_rate_hz == pytest.approx(2.0)

        state_machine = StateMachineNodeConfig()
        assert state_machine.challenge_mode_samples_required == 3
        assert state_machine.challenge_mode_timeout_sec == pytest.approx(180.0)


class TestSecondWaveHardwareAnchoring:
    """The second migration wave is anchored to its generated DTO."""

    def test_challenge_mode_subclasses_generated_dto(self):
        from shared.config.generated.hardware.challenge_mode_schema import HardwareChallengeMode

        from src.hardware.challenge_mode.config import Config as ChallengeModeConfig

        assert issubclass(ChallengeModeConfig, HardwareChallengeMode)

    def test_ssd1306_subclasses_generated_dto(self):
        from shared.config.generated.hardware.display.ssd1306_schema import HardwareDisplaySsd1306

        from src.hardware.display.ssd1306.config import Config as Ssd1306Config

        assert issubclass(Ssd1306Config, HardwareDisplaySsd1306)

    def test_button_gpio_subclasses_generated_dto(self):
        from shared.config.generated.hardware.button.gpio_schema import HardwareButtonGpio

        from src.hardware.button.gpio.driver import Config as ButtonGpioConfig

        assert issubclass(ButtonGpioConfig, HardwareButtonGpio)

    def test_oled_node_subclasses_generated_dto(self):
        from shared.config.generated.hardware.display.oled_node_schema import HardwareDisplayOledNode
        from vtitan_drivers.oled_display_node import NodeConfig as OledNodeConfig

        assert issubclass(OledNodeConfig, HardwareDisplayOledNode)

    def test_hailo_configs_subclass_generated_dtos(self):
        from shared.config.generated.hardware.hailo_schema import HardwareHailo
        from shared.config.generated.hardware.hailo_streaming_schema import HardwareHailoStreaming

        from src.hardware.hailo.config import (
            Config as HailoConfig,
            StreamingConfig,
        )

        assert issubclass(HailoConfig, HardwareHailo)
        assert issubclass(StreamingConfig, HardwareHailoStreaming)

    def test_detector_subclasses_generated_dto(self):
        from shared.config.generated.hardware.vision.detector_schema import HardwareVisionDetector

        from src.vision.detector import DetectorConfig

        assert issubclass(DetectorConfig, HardwareVisionDetector)

    def test_hud_subclasses_generated_dto(self):
        from shared.config.generated.hardware.vision.hud_schema import HardwareVisionHud

        from src.vision.hud import HudConfig

        assert issubclass(HudConfig, HardwareVisionHud)

    def test_bno08x_uart_rvc_subclasses_generated_dto(self):
        from shared.config.generated.hardware.imu.bno08x_uart_rvc_schema import HardwareImuBno08xUartRvc

        from src.hardware.imu.bno08x.uart_rvc import Config as UartRvcConfig

        assert issubclass(UartRvcConfig, HardwareImuBno08xUartRvc)

    def test_vision_node_config_subclasses_generated_dto(self):
        from shared.config.generated.hardware.vision.node_schema import HardwareVisionNode

        from src.ros2.vision.node import Config as VisionNodeConfig

        assert issubclass(VisionNodeConfig, HardwareVisionNode)

    def test_rpicam_subclasses_generated_dto(self):
        from shared.config.generated.hardware.camera.rpicam_schema import HardwareCameraRpicam

        from src.hardware.camera.rpicam.driver import Config as RpicamConfig

        assert issubclass(RpicamConfig, HardwareCameraRpicam)


class TestSecondWaveLoadedValues:
    """The real shipped TOMLs, as the second-wave wrappers read them."""

    def test_challenge_mode_values(self):
        from src.hardware.challenge_mode.config import Config as ChallengeModeConfig

        assert ChallengeModeConfig().challenge_mode_gpio_pin == 23

    def test_ssd1306_values(self):
        from src.hardware.display.ssd1306.config import Config as Ssd1306Config

        ssd = Ssd1306Config()

        assert ssd.width == 128
        assert ssd.height == 64
        assert ssd.i2c_address == "0x3C"
        assert ssd.i2c_address_int == 0x3C
        assert ssd.i2c_bus == 1

    def test_button_gpio_values(self):
        from src.hardware.button.gpio.driver import Config as ButtonGpioConfig

        button = ButtonGpioConfig()

        assert button.button_gpio_pin == 4
        assert button.button.pull_up is True
        assert button.button.debounce_ms == 50
        assert button.button.long_press_threshold_sec == pytest.approx(3.0)
        assert button.button.shutdown_press_threshold_sec == pytest.approx(10.0)

    def test_oled_node_values(self):
        from vtitan_drivers.oled_display_node import NodeConfig as OledNodeConfig

        from src.hardware.display.enums import DisplayBackend

        oled = OledNodeConfig()

        assert oled.ui_refresh_rate_hz == pytest.approx(10.0)
        assert oled.display_backend is DisplayBackend.BLINKA

    def test_hailo_values(self):
        from src.hardware.hailo.config import (
            Config as HailoConfig,
            StreamingConfig,
        )

        hailo = HailoConfig()

        assert hailo.model_path == "/usr/local/hailo/models/gmr.hef"
        assert hailo.inference_timeout_ms == 10000
        assert hailo.benchmark_iterations == 10
        assert hailo.data_yaml_path == "/usr/local/hailo/models/data.yaml"
        assert hailo.min_confidence == pytest.approx(0.45)
        assert hailo.class_map[0].value == "green"

        streaming = StreamingConfig()
        assert streaming.width == 640
        assert streaming.height == 640
        assert streaming.fps == 30
        assert streaming.model_input_width == 640
        assert streaming.queue_size == 1
        assert streaming.async_inference is False
        assert streaming.min_confidence == pytest.approx(0.45)

    def test_detector_values(self):
        from src.vision.detector import BBoxFormat, DetectorConfig

        detector = DetectorConfig(model_path="", class_to_color={})

        assert detector.min_confidence == pytest.approx(0.45)
        assert detector.output_format is BBoxFormat.NORMALIZED

    def test_hud_values(self):
        import cv2
        from shared.config.constants import RobotSpecs

        from src.vision.hud import HudConfig

        hud = HudConfig()

        assert hud.font_face == cv2.FONT_HERSHEY_DUPLEX
        assert hud.font_scale == pytest.approx(0.55)
        assert hud.text_thickness == 1
        assert hud.line_height_px == 22
        assert hud.text_rgb == (248, 250, 252)
        assert hud.panel_alpha == pytest.approx(0.55)
        assert hud.radar_radius_px == 90
        assert hud.join_timeout_sec == pytest.approx(30.0)
        assert hud.lidar_inverted == RobotSpecs.LIDAR_INVERTED

    def test_bno08x_uart_rvc_values(self, monkeypatch):
        monkeypatch.setenv("BNO08X_UART_RVC_PORT", "/dev/ttyACM0")
        from src.hardware.imu.bno08x.uart_rvc import Config as UartRvcConfig

        imu = UartRvcConfig()

        assert imu.port == "/dev/ttyACM0"
        assert imu.default_port == "/dev/ttyACM0"
        assert imu.baudrate == 115200
        assert imu.poll_rate_hz == pytest.approx(100.0)
        assert imu.serial_timeout == pytest.approx(1.0)
        assert imu.data_lock_timeout == pytest.approx(2.0)
        assert imu.quaternion.euler_sequence == "xyz"
        assert imu.quaternion.negate_yaw is True
        assert imu.quaternion.negate_pitch is False
        assert imu.quaternion.negate_roll is True

    def test_vision_node_values(self):
        from src.ros2.vision.node import Config as VisionNodeConfig

        config = VisionNodeConfig()

        assert config.camera_topic == "/camera/image_raw"
        assert config.model_path == "yolov8n.pt"
        assert config.backend == "yolo"
        assert config.camera_source == "topic"
        assert config.capture_fps == pytest.approx(15.0)
        assert config.video_width == 1536
        assert config.capture_interval_s == pytest.approx(10.0)
        assert config.capture_subdir == "captures"
        assert config.debug_stream_fps == pytest.approx(0.0)
        assert config.record_video is True

    def test_rpicam_values(self):
        from src.hardware.camera.rpicam.driver import (
            AfMode,
            AwbMode,
            Config as RpicamConfig,
            ExposureMode,
        )

        camera = RpicamConfig()

        assert camera.camera_width == 1536
        assert camera.camera_height == 864
        assert camera.camera_fps == 30
        assert camera.camera_inverted is True
        assert camera.camera_hflip is False
        assert camera.camera_vflip is False
        assert camera.camera_read_timeout_sec == pytest.approx(5.0)
        assert camera.camera_af_mode is AfMode.MANUAL
        assert camera.camera_lens_position == pytest.approx(1.25)
        assert camera.camera_awb_mode is AwbMode.AUTO
        assert camera.camera_exposure_mode is ExposureMode.SHORT
        assert camera.camera_sharpness == pytest.approx(1.0)
        assert camera.camera_analogue_gain == pytest.approx(1.0)
        assert camera.camera_awb_gains is None
        assert camera.camera_exposure_time_us is None
        assert camera.camera_flicker_period_us is None

        # resolved_flips folds camera_inverted in: mounted upside-down, so an
        # unflipped config still yields both mirrors.
        assert camera.resolved_flips() == (True, True)
