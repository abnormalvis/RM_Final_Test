source ~/final_ws/devel/setup.bash
roslaunch sentry_chassis_controller sentry_with_odom.launch 2>&1 | tee -a ~/final_ws/src/sentry_chassis_controller/sentry_with_odom.log