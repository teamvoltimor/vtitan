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
