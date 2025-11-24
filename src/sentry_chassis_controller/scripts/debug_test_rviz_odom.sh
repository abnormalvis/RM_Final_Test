# Step 1 - Launch your system
source /home/idris/final_ws/devel/setup.bash
roslaunch sentry_chassis_controller sentry_with_odom.launch 2>&1 | tee src/sentry_chassis_controller/log/sentry_with_odom.log

# Step 2 - Check joint changes while keyboard-driving
# (Hit some keys to move robot while this runs)
