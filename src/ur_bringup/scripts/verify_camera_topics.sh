#!/bin/bash
# Script to verify camera topics are published correctly

echo "=== Camera Topic Verification Script ==="
echo ""
echo "Waiting for Gazebo to start..."
sleep 8

echo ""
echo "=== Checking Camera Topics ==="
echo ""
echo "World Camera Topics (world/ prefix):"
ros2 topic list 2>/dev/null | grep "^world/" || echo "  No world/ topics found"
echo ""
echo "End-Effector Camera Topics (ee/ prefix):"
ros2 topic list 2>/dev/null | grep "^ee/" || echo "  No ee/ topics found"
echo ""
echo "Checking if default topics exist (should NOT exist):"
if ros2 topic list 2>/dev/null | grep -E "^(depth|color)/" > /dev/null; then
    echo "  WARNING: Found default topics (depth/, color/):"
    ros2 topic list 2>/dev/null | grep -E "^(depth|color)/"
else
    echo "  OK: No default topics found"
fi

echo ""
echo "=== Camera Frame TFs ==="
echo "World camera TF:"
timeout 2 ros2 run tf2_ros tf2_echo world world_view_camera_link 2>&1 | grep -A3 "At time" || echo "  TF not available yet"
echo ""
echo "End-effector camera TF:"
timeout 2 ros2 run tf2_ros tf2_echo tool0 camera_link 2>&1 | grep -A3 "At time" || echo "  TF not available yet"

echo ""
echo "=== Topic Publishing Rate ==="
echo "World color image:"
ros2 topic hz /world/color/image_raw 2>/dev/null | head -2 || echo "  Not publishing"
echo ""
echo "EE color image:"
ros2 topic hz /ee/color/image_raw 2>/dev/null | head -2 || echo "  Not publishing"

echo ""
echo "=== Verification Complete ==="
echo ""
echo "Expected results:"
echo "  - world/depth/image_raw ✓"
echo "  - world/color/image_raw ✓"
echo "  - ee/depth/image_raw ✓"
echo "  - ee/color/image_raw ✓"
echo "  - NO depth/image_raw ✗"
echo "  - NO color/image_raw ✗"
