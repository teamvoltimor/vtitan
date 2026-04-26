"""Tests for SDF robot builder sensor plugins (CR-07).

Verifies:
- Camera sensor has explicit pose, correct topic (/wro_robot/camera).
- LIDAR noise block present with correct mean=0, stddev=0.03.
- LIDAR min range = 0.05 m (50 mm) to match real sensor.
- IMU noise blocks present for angular_velocity and linear_acceleration
  with declared stddevs (gyro 0.054 rad/s, accel 0.3 m/s²).
- Generated SDF is parseable XML.
"""

from __future__ import annotations

from xml.etree import ElementTree as ET

import pytest

from shared.config.constants import RobotSpecs
from src.generation.sdf_robot_builder import add_robot_model


@pytest.fixture
def robot_world() -> ET.Element:
    """Minimal world element with a robot model appended."""
    world = ET.Element("world")
    starting_conditions = {
        "position": (1.5, 0.5),
        "yaw": 0.0,
        "section": "south",
        "section_name": "south",
        "direction": "clockwise",
    }
    add_robot_model(world, starting_conditions)
    return world


@pytest.fixture
def robot_model(robot_world) -> ET.Element:
    return robot_world.find("model[@name='wro_robot']")


# ── Camera ────────────────────────────────────────────────────────────────────


class TestCamera:
    def test_camera_link_exists(self, robot_model):
        assert robot_model.find("link[@name='camera_link']") is not None

    def test_camera_sensor_present(self, robot_model):
        cam_link = robot_model.find("link[@name='camera_link']")
        assert cam_link.find("sensor[@type='camera']") is not None

    def test_camera_topic(self, robot_model):
        cam_link = robot_model.find("link[@name='camera_link']")
        sensor = cam_link.find("sensor[@type='camera']")
        topic = sensor.find("topic")
        assert topic is not None
        assert topic.text == "/wro_robot/camera"

    def test_camera_sensor_has_explicit_pose(self, robot_model):
        cam_link = robot_model.find("link[@name='camera_link']")
        sensor = cam_link.find("sensor[@type='camera']")
        assert sensor.find("pose") is not None

    def test_camera_has_correct_hfov(self, robot_model):
        cam_link = robot_model.find("link[@name='camera_link']")
        sensor = cam_link.find("sensor[@type='camera']")
        hfov = sensor.find("camera/horizontal_fov")
        assert hfov is not None
        assert float(hfov.text) == pytest.approx(RobotSpecs.CAMERA_HFOV, rel=1e-3)

    def test_camera_image_resolution(self, robot_model):
        cam_link = robot_model.find("link[@name='camera_link']")
        sensor = cam_link.find("sensor[@type='camera']")
        assert int(sensor.find("camera/image/width").text) == RobotSpecs.CAMERA_WIDTH
        assert int(sensor.find("camera/image/height").text) == RobotSpecs.CAMERA_HEIGHT


# ── LIDAR ─────────────────────────────────────────────────────────────────────


class TestLidar:
    def test_lidar_link_exists(self, robot_model):
        assert robot_model.find("link[@name='lidar_link']") is not None

    def test_lidar_sensor_present(self, robot_model):
        lidar_link = robot_model.find("link[@name='lidar_link']")
        assert lidar_link.find("sensor[@type='gpu_lidar']") is not None

    def test_lidar_noise_block_present(self, robot_model):
        lidar_link = robot_model.find("link[@name='lidar_link']")
        sensor = lidar_link.find("sensor[@type='gpu_lidar']")
        noise = sensor.find(".//noise")
        assert noise is not None, "LIDAR sensor must have a noise block"

    def test_lidar_noise_type_gaussian(self, robot_model):
        lidar_link = robot_model.find("link[@name='lidar_link']")
        sensor = lidar_link.find("sensor[@type='gpu_lidar']")
        noise = sensor.find(".//noise")
        assert noise.find("type").text == "gaussian"

    def test_lidar_noise_mean_zero(self, robot_model):
        lidar_link = robot_model.find("link[@name='lidar_link']")
        sensor = lidar_link.find("sensor[@type='gpu_lidar']")
        noise = sensor.find(".//noise")
        assert float(noise.find("mean").text) == pytest.approx(0.0)

    def test_lidar_noise_stddev(self, robot_model):
        lidar_link = robot_model.find("link[@name='lidar_link']")
        sensor = lidar_link.find("sensor[@type='gpu_lidar']")
        noise = sensor.find(".//noise")
        assert float(noise.find("stddev").text) == pytest.approx(RobotSpecs.LIDAR_NOISE_STDDEV)

    def test_lidar_min_range_50mm(self, robot_model):
        lidar_link = robot_model.find("link[@name='lidar_link']")
        sensor = lidar_link.find("sensor[@type='gpu_lidar']")
        min_range = sensor.find(".//range/min")
        assert min_range is not None
        assert float(min_range.text) == pytest.approx(0.05, abs=1e-4), (
            f"LIDAR min_range should be 0.05 m (50 mm), got {min_range.text}"
        )

    def test_lidar_max_range(self, robot_model):
        lidar_link = robot_model.find("link[@name='lidar_link']")
        sensor = lidar_link.find("sensor[@type='gpu_lidar']")
        max_range = sensor.find(".//range/max")
        assert float(max_range.text) == pytest.approx(RobotSpecs.LIDAR_MAX_RANGE)


# ── IMU ───────────────────────────────────────────────────────────────────────


class TestImu:
    def test_imu_link_exists(self, robot_model):
        assert robot_model.find("link[@name='imu_link']") is not None

    def test_imu_sensor_present(self, robot_model):
        imu_link = robot_model.find("link[@name='imu_link']")
        assert imu_link.find("sensor[@type='imu']") is not None

    def test_imu_angular_velocity_noise_present(self, robot_model):
        imu_link = robot_model.find("link[@name='imu_link']")
        sensor = imu_link.find("sensor[@type='imu']")
        assert sensor.find(".//angular_velocity") is not None
        noise_nodes = sensor.findall(".//angular_velocity//noise")
        assert len(noise_nodes) > 0, "IMU angular_velocity must have noise blocks"

    def test_imu_angular_velocity_stddev(self, robot_model):
        imu_link = robot_model.find("link[@name='imu_link']")
        sensor = imu_link.find("sensor[@type='imu']")
        # All three axes should have stddev = IMU_GYRO_NOISE
        for noise in sensor.findall(".//angular_velocity//noise"):
            assert float(noise.find("stddev").text) == pytest.approx(
                RobotSpecs.IMU_GYRO_NOISE
            ), f"Expected gyro noise {RobotSpecs.IMU_GYRO_NOISE} rad/s"

    def test_imu_linear_acceleration_noise_present(self, robot_model):
        imu_link = robot_model.find("link[@name='imu_link']")
        sensor = imu_link.find("sensor[@type='imu']")
        assert sensor.find(".//linear_acceleration") is not None
        noise_nodes = sensor.findall(".//linear_acceleration//noise")
        assert len(noise_nodes) > 0, "IMU linear_acceleration must have noise blocks"

    def test_imu_linear_acceleration_stddev(self, robot_model):
        imu_link = robot_model.find("link[@name='imu_link']")
        sensor = imu_link.find("sensor[@type='imu']")
        for noise in sensor.findall(".//linear_acceleration//noise"):
            assert float(noise.find("stddev").text) == pytest.approx(
                RobotSpecs.IMU_ACCEL_NOISE
            ), f"Expected accel noise {RobotSpecs.IMU_ACCEL_NOISE} m/s²"


# ── XML validity ──────────────────────────────────────────────────────────────


def test_generated_sdf_is_valid_xml(robot_world):
    """Serialise to string and re-parse to confirm valid XML."""
    xml_str = ET.tostring(robot_world, encoding="unicode")
    parsed = ET.fromstring(xml_str)
    assert parsed is not None


def test_all_sensor_links_have_fixed_joints(robot_model):
    """Camera, LIDAR, and IMU must each have a fixed joint to base_link."""
    expected_joints = {"camera_joint", "lidar_joint", "imu_joint"}
    found = {
        j.get("name")
        for j in robot_model.findall("joint[@type='fixed']")
        if j.get("name") in expected_joints
    }
    assert found == expected_joints, f"Missing fixed joints: {expected_joints - found}"
