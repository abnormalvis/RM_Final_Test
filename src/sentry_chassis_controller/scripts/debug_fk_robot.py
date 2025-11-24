#!/usr/bin/env python3

import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
from datetime import datetime
import time

"""
Simple forward kinematics debug monitor for Sentry Chassis.
Monitors the key topics and shows what's happening.
"""

def main():
    rospy.init_node('fk_debugger')
    print("=== Forward Kinematics Debug Monitor (Sentry Chassis) ===")
    print(f"Started debugging")

    # Tracking variables
    first_pose = None
    prev_wheel_data = {}
    movement_detected = False
    cmd_vel_time = None

    print("Watching /cmd_vel and /joint_states...")

    def cmd_vel_callback(msg):
        """Monitor cmd_vel messages to detect user commands"""
        nonlocal cmd_vel_time
        if msg.linear.x != 0 or msg.linear.y != 0 or msg.angular.z != 0:
            print(f"🎯 CMD_VEL detected: lin({msg.linear.x:.2f},{msg.linear.y:.2f}) ang({msg.angular.z:.2f})")
            cmd_vel_time = datetime.now()
            print("   Command is happening...")

    def joint_states_callback(msg):
        """Monitor and decode joint states to see if wheels are actually turning"""
        nonlocal movement_detected
        wheel_detected = False

        # Check all joints
        for i, (name, pos) in enumerate(zip(msg.name, msg.position)):
            # Look for wheel joints - we expect pattern like *_wheel_* or *wheel* or *wh*
            if 'wheel' in name.lower() and not 'pivot' in name.lower():
                wheel_detected = True

                # Track position changes for velocity calculation
                if name in prev_wheel_data:
                    prev_pos, prev_time = prev_wheel_data[name]
                    dt = (msg.header.stamp - prev_time).to_sec()

                    if dt > 0.01:  # Solid time difference
                        delta_pos = pos - prev_pos
                        velocity = delta_pos / dt if dt != 0 else 0

                        if abs(velocity) > 0.001:  # Significant movement
                            print(f"⚙️  {name:30} POS={pos:7.3f} Δ={delta_pos:8.4f} dt={dt:6.3f}s ➜ VEL={velocity:8.3f} rad/s")
                            movement_detected = True

                        # Show special detection for left vs right differences
                        if 'left' in name.lower() and abs(velocity) > 0.01:
                            print(f"   👍 Left wheel detected: {velocity:.3f} rad/s")
                        elif 'right' in name.lower() and abs(velocity) > 0.01:
                            print(f"   👍 Right wheel detected: {velocity:.3f} rad/s")

                # Store for next comparison
                prev_wheel_data[name] = (pos, msg.header.stamp)
                break  # Show one example wheel

        if wheel_detected and not movement_detected:
            print("🔄 None of wheels are moving? All vel≈0")
            print("   Try: Full throttle movement (WASD keys)")
        elif wheel_detected and movement_detected:
            print("✅ Motor movement detected in joints!")

    def odom_callback(msg):
        """Monitor odometry to see if our calculation produces any movement"""
        nonlocal first_pose

        now = datetime.now()

        if first_pose is None:
            first_pose = msg.pose.pose.position
            print(f"📍 Initial ODOM position set: ({first_pose.x:.4f}, {first_pose.y:.4f}, {first_pose.z:.4f})")
        else:
            moved = (msg.pose.pose.position.x != first_pose.x or
                    msg.pose.pose.position.y != first_pose.y or
                    msg.pose.pose.position.z != first_pose.z)

            any_twist = (msg.twist.twist.linear.x != 0 or
                        msg.twist.twist.linear.y != 0 or
                        msg.twist.twist.angular.z != 0)

            if moved:
                print(f"🚀 ODOM POSITION CHANGE detected!")
                print(f"   new pos: ({msg.pose.pose.position.x:.4f}, {msg.pose.pose.position.y:.4f}, {msg.pose.pose.position.z:.4f})")
                print(f"   twist: ({msg.twist.twist.linear.x:.4f}, {msg.twist.twist.linear.y:.4f}, ang_z={msg.twist.twist.angular.z:.4f})")
                first_pose = msg.pose.pose.position

            elif any_twist and abs(msg.twist.twist.linear.x) > 0.001:
                print(f"🚀 ODOM TWIST ACTIVE! lin({msg.twist.twist.linear.x:.4f},{msg.twist.twist.linear.y:.4f}) ang_z({msg.twist.twist.angular.z:.4f})")

    # Subscribe to key topics
    rospy.Subscriber('/cmd_vel', geometry_msgs.msg.Twist, cmd_vel_callback)
    rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, joint_states_callback)
    rospy.Subscriber('/odom', nav_msgs.msg.Odometry, odom_callback)

    try:
        print("🔍 Monitoring... (Use keyboard NOW! Press Ctrl+C to stop)")
        rospy.sleep(0.5)
        print("Move the robot with WASD/arrow keys and watch what happens...")
        rospy.spin()
    except KeyboardInterrupt:
        print("\
=== Debug Summary ===")
        if movement_detected:
            print("✓ Robot joint movement was detected")
        else:
            print("✗ No robot joint movement was detected")
        print("If okay, check RViz Odometry display! If not, investigate joint names or controller")

if __name__ == '__main__':
    main()