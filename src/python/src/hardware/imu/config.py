from pydantic import BaseModel, field_validator


class QuaternionConfig(BaseModel):
    """Configuration for quaternion calculation from Euler angles."""

    negate_yaw: bool = True
    """
    Whether to negate the yaw angle for ROS 2 CCW-positive Yaw. Depending on how the IMU is mounted, you may need to negate yaw to ensure that counterclockwise rotation corresponds to positive yaw angles in ROS 2.
    """

    negate_pitch: bool = False
    """Whether to negate the pitch angle if it is inverted due to mounting orientation. Depending on how the IMU is mounted, you may need to negate pitch to ensure that nose-up corresponds to positive pitch angles in ROS 2."""

    negate_roll: bool = True
    """Whether to negate the roll angle if it is inverted due to mounting orientation. Depending on how the IMU is mounted, you may need to negate roll to ensure that banking right corresponds to positive roll angles in ROS 2."""

    euler_sequence: str = "xyz"
    """The sequence of Euler angles for conversion to quaternion. The default is 'xyz' (roll, pitch, yaw). Adjust this if your IMU uses a different convention."""

    @field_validator("euler_sequence")
    @classmethod
    def _validate_euler_sequence(cls, value: str) -> str:
        valid_sequences = {"xyz", "zyx", "xzy", "yzx", "zxy", "yxz"}
        if value not in valid_sequences:
            msg = f"Invalid euler_sequence '{value}'. Valid options are: {valid_sequences}"
            raise ValueError(msg)
        return value
