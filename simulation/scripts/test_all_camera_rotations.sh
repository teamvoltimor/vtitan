#!/bin/bash
# Test all possible 90-degree camera rotations
# This will help us find the correct orientation

echo "Testing camera orientations..."
echo "Each test will create a 15-second video"
echo ""

# Test rotations to try (Roll, Pitch, Yaw in radians)
declare -A tests=(
    ["test1_pitch_pos90"]="0 1.5708 0"
    ["test2_pitch_neg90"]="0 -1.5708 0"
    ["test3_yaw_pos90"]="0 0 1.5708"
    ["test4_yaw_neg90"]="0 0 -1.5708"
    ["test5_roll_pos90"]="1.5708 0 0"
    ["test6_roll_neg90"]="-1.5708 0 0"
    ["test7_rp_pos90"]="1.5708 1.5708 0"
    ["test8_ry_pos90"]="1.5708 0 1.5708"
    ["test9_py_pos90"]="0 1.5708 1.5708"
    ["test10_all_pos90"]="1.5708 1.5708 1.5708"
)

echo "Copy and paste these rotations into generate_training_data.py"
echo "Find the line with camera_link pose and change the last 3 numbers"
echo ""
echo "Format: 'X Y Z Roll Pitch Yaw'"
echo ""

for name in "${!tests[@]}"; do
    rotation="${tests[$name]}"
    echo "$name: '0.10 0 0.065 $rotation'"
done

echo ""
echo "Test each one and note which gives you a forward-looking view!"
