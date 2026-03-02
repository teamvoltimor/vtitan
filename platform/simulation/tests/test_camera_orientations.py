#!/usr/bin/env python3
"""
Test different camera orientations to find the correct one.
Generates scenarios with different camera poses to test.
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))


# Test orientations (Roll, Pitch, Yaw in radians)
TEST_ORIENTATIONS = {
    "default": "0 0 0",
    "pitch_90": "0 1.5708 0",
    "pitch_neg90": "0 -1.5708 0",
    "yaw_90": "0 0 1.5708",
    "yaw_neg90": "0 0 -1.5708",
    "roll_90": "1.5708 0 0",
    "roll_neg90": "-1.5708 0 0",
    "roll_pitch_90": "1.5708 1.5708 0",
    "roll_yaw_90": "1.5708 0 1.5708",
    "pitch_yaw_90": "0 1.5708 1.5708",
    "all_90": "1.5708 1.5708 1.5708",
}

print("Camera Orientation Test Guide")
print("=" * 60)
print("\nThis will help find the correct camera orientation.")
print("For each test, note what you see:\n")

for name, rotation in TEST_ORIENTATIONS.items():
    print(f"{name:20s} -> pose='0.10 0 0.065 {rotation}'")

print("\n" + "=" * 60)
print("\nTo test an orientation, edit generate_training_data.py:")
print("Find the camera_link pose line and change the rotation values.")
print("\nExample:")
print(
    "  ET.SubElement(camera_link, 'pose', relative_to='base_link').text = "
    "'0.10 0 0.065 0 1.5708 0'",
)
print(" " * 54 + "^  ^      ^")
print(" " * 52 + "Roll Pitch Yaw")
print("\nThen generate a test video and note what you see.")
print("\nDescribe the view for each test:")
print("  - Is track visible?")
print("  - Is it moving toward or away from camera?")
print("  - Is view upside-down?")
print("  - Is view sideways?")
print("  - Can you see the floor/track?")
