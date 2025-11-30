source ~/final_ws/devel/setup.bash
roslaunch sentry_chassis_controller sentry_with_odom_feature.launch 2>&1 | tee src/sentry_chassis_controller/log/sentry_with_odom_launch.log 
