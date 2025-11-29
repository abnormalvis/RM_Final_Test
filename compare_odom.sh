#!/bin/bash
# 对比Gazebo ground truth和控制器里程计

echo "=== Gazebo Ground Truth (from /odom) ==="
timeout 2 rostopic echo /odom/pose/pose | head -20

echo -e "\n=== Controller Odometry (from /odom_controller) ==="
timeout 2 rostopic echo /odom_controller/pose/pose | head -20

echo -e "\n=== Twist Comparison ==="
echo "Gazebo /odom twist:"
timeout 1 rostopic echo /odom/twist/twist -n 1
echo -e "\nController /odom_controller twist:"
timeout 1 rostopic echo /odom_controller/twist/twist -n 1
