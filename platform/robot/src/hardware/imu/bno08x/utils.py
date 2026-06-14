from typing import cast

from scipy.spatial.transform import Rotation as R

from src.hardware.imu.readings import QuaternionReading


def calculate_quaternion_from_euler(
    yaw_deg: float,
    pitch_deg: float,
    roll_deg: float,
    euler_sequence: str = "xyz",
    negate_yaw: bool = False,
    negate_pitch: bool = False,
    negate_roll: bool = False,
) -> QuaternionReading:
    """
    Calculate quaternion from Euler angles with optional negation for ROS 2 compatibility.

    Args:
        yaw_deg (float): Yaw angle in degrees.
        pitch_deg (float): Pitch angle in degrees.
        roll_deg (float): Roll angle in degrees.
        euler_sequence (str): The sequence of Euler angles for conversion to quaternion. Default is 'xyz' (roll, pitch, yaw).
        negate_yaw (bool): If True, negate the yaw angle for ROS 2 CCW-positive Yaw.
        negate_pitch (bool): If True, negate the pitch angle if Pitch is inverted (Nose up = Model down).
        negate_roll (bool): If True, negate the roll angle if Roll is inverted (Bank right = Model left).

    Returns:
        QuaternionReading: The calculated quaternion based on the provided Euler angles and negation settings.
    """
    # Diagnostic Fixes:
    # 1. Negate 'y' for ROS 2 CCW-positive Yaw.
    # 2. Negate 'p' if Pitch is inverted (Nose up = Model down).
    # 3. Negate 'r' if Roll is inverted (Bank right = Model left).
    corrected_yaw_deg = yaw_deg if not negate_yaw else -yaw_deg
    corrected_pitch_deg = pitch_deg if not negate_pitch else -pitch_deg
    corrected_roll_def = roll_deg if not negate_roll else -roll_deg

    # Convert Euler angles to quaternion using scipy.spatial.transform.Rotation
    r = R.from_euler(
        euler_sequence,
        [corrected_roll_def, corrected_pitch_deg, corrected_yaw_deg],
        degrees=True,
    )

    # The adafruit_bno08x_rvc library does not provide quaternion data directly, but we can convert from Euler angles.
    qx, qy, qz, qw = cast("tuple[float, float, float, float]", cast("object", r.as_quat()))
    return QuaternionReading(x=qx, y=qy, z=qz, w=qw)
