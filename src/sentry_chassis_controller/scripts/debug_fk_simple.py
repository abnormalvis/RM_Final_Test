#!/usr/bin/env python3

import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
from datetime import datetime

"""
Simple forward kinematics debug monitor.
Monitors the key topics and shows what's happening.
"""

class FKDebugger:
    def __init__(self):
        rospy.init_node('fk_debugger')

        # Track time for analysis
        self.start_time = rospy.Time.now()
        self.movement_detected = False
        self.prev_wheel_data = {}

        print(f"=== Forward Kinematics Debug Monitor ===")
        print(f"Started at {self.start_time.to_sec():.1f}")
        print(f"Watching /joint_states and /cmd_vel...")

        # Subscribe to key topics
        rospy.Subscriber('/cmd_vel', geometry_msgs.msg.Twist, self.cmd_vel_callback)
        rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.joint_states_callback)
        rospy.Subscriber('/odom', nav_msgs.msg.Odometry, self.odom_callback)

    def cmd_vel_callback(self, msg):
        """Monitor cmd_vel messages to detect user commands"""
        if msg.linear.x != 0 or msg.linear.y != 0 or msg.angular.z != 0:
            print(f"🎯 CMD_VEL detected: lin({msg.linear.x:.2f},{msg.linear.y:.2f}) ang({msg.angular.z:.2f})")

    def joint_states_callback(self, msg):
        """Monitor and decode joint states to see if wheels are actually turning"""
        # Check if any wheel joint positions are changing significantly
        wheel_detected = False

        # Look for wheel joints - check various naming conventions
        key_wheel_names = ['left_front_wheel', 'front_left_wheel', 'wheel_fl', 'left_wheel']

        for name, pos in zip(msg.name, msg.position):
            if any(wheel_term in name.lower() for wheel_term in ['wheel', 'wh']) and not 'pivot' in name.lower():
                wheel_detected = True

                # Track position changes for velocity calculation
                if name in self.prev_wheel_data:
                    prev_pos, prev_time = self.prev_wheel_data[name]
                    dt = (msg.header.stamp - prev_time).to_sec()

                    if dt > 0.01:  # Solid time difference
                        delta_pos = pos - prev_pos
                        # Handle angle wrapping
                        if abs(delta_pos) > 3.14:
                            delta_pos = delta_pos - 2*3.14159 if delta_pos > 0 else delta_pos + 2*3.14159

                        velocity = delta_pos / dt

                        if abs(velocity) > 0.01:  # Significant movement
                            print(f"⚙️  {name:25} POS={pos:7.3f} Δ={delta_pos:8.4f} dt={dt:6.3f}s ➜ VEL={velocity:8.3f} rad/s")
                            self.movement_detected = True

                        # Show accum/diff for expected wheel motion pattern
                        if 'left' in name.lower() and abs(velocity) > 0.1:
                            print(f"   👍 Left wheel seems active: {velocity:.3f} rad/s")
                        elif 'right' in name.lower() and abs(velocity) > 0.1:
                            print(f"   👍 Right wheel seems active: {velocity:.3f} rad/s")

                # Store for next comparison
                self.prev_wheel_data[name] = (pos, msg.header.stamp)

        if wheel_detected and not self.movement_detected:
            print(f"🔄 None of wheels are moving? All vel≈0")

    def odom_callback(self, msg):
        """Monitor odometry to see if our calculation produces any movement"""
        # Check if position or twist has changed at all
        static first_pose = None
        static first_time = None

        if first_pose is None:
            first_pose = msg.pose.pose.position
            first_time = datetime.now()
        else:
            moved = (msg.pose.pose.position.x != first_pose.x or
                    msg.pose.pose.position.y != first_pose.y or
                    msg.pose.pose.position.z != first_pose.z)

            any_twist = (msg.twist.twist.linear.x != 0 or
                        msg.twist.twist.linear.y != 0 or
                        msg.twist.twist.angular.z != 0)

            elapsed = (datetime.now() - first_time).total_seconds()

            if moved:
                print(f"🚀 ODOM POSITION CHANGE detected after {elapsed:.1f}s!")
                print(f"   new pos: ({msg.pose.pose.position.x:.4f}, {msg.pose.pose.position.y:.4f}, {msg.pose.pose.position.z:.4f})")
                first_pose = msg.pose.pose.position

            elif any_twist:
                print(f"🚀 ODOM TWIST NON-ZERO detected! ({msg.twist.twist.linear.x:.4f}, {msg.twist.twist.linear.y:.4f}, {msg.twist.twist.angular.z:.4f})")

            elif elapsed > 15:  # After 15 seconds, if still zero:
                print(f"❌ Still no odom movement after {elapsed:.1f}s")
                print(f"   Current odom: pos=({msg.pose.pose.position.x:.3f},{msg.pose.pose.position.y:.3f}) twist=({msg.twist.twist.linear.x:.3f},{msg.twist.twist.linear.y:.3f},{msg.twist.twist.angular.z:.3f})")
                return  # Don't spam anymore

def main():
    try:
        debugger = FKDebugger()
        print("\n🔍 Monitoring... (Press Ctrl+C to see focus output)")
        rospy.spin()
    except KeyboardInterrupt:
        print("\n=== Debug Summary ===")
        if debugger.movement_detected:
            print("✓ Robot joint movement detected")
        else:
            print("✗ No robot joint movement detected")
        print("Check if joint names contain 'wheel', or pivot positions change")

if __name__ == '__main__':
    main()