# Test the enhanced system
source /home/idris/final_ws/devel/setup.bash
roslaunch sentry_chassis_controller sentry_with_odom.launch

# Live monitoring of results
rosrun sentry_chassis_controller debug_fk_robot.py

# Verify non-zero wheel positions
rostopic echo /joint_states position[0:3] -n 10

# Check changing odometry
rostopic echo /odom pose.pose.position -n 5

# RViz odometry visualization
# 1. Launch rviz with sentry config
# 2. Add Odometry display plugin
# 3. Verify robot trajectory appears when moving