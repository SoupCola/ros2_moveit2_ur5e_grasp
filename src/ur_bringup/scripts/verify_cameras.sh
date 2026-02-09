#!/bin/bash
# Script to verify camera topics are published correctly

echo "=== Camera Topic Verification Script ==="
echo ""
echo "Waiting for Gazebo to start..."
sleep 5

echo ""
echo "=== Available Camera Topics ==="
ros2 topic list | grep -E "(world|depth|color|infra|points)" | sort

echo ""
echo "=== World Camera Topics ==="
ros2 topic list | grep "^world/"

echo ""
echo "=== End-Effector Camera Topics ==="
ros2 topic list | grep -E "^(depth|color|infra)/" | grep -v "world"

echo ""
echo "=== Camera Frame TFs ==="
ros2 run tf2_ros tf2_echo world world_camera_link 2>&1 | head -5 &
ros2 run tf2_ros tf2_echo tool0 ee_camera_link 2>&1 | head -5 &

wait
echo ""
echo "=== Verification Complete ==="
echo "You can now visualize cameras in RViz or Gazebo"
