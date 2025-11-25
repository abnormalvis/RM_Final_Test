#!/usr/bin/env python3
"""
自动化 teleop 测试脚本：发布一段时间的世界坐标速度并将其转换为机体坐标发布到 /cmd_vel，同时发布 /cmd_vel_stamped（header.frame_id=odom）。
序列：
 - 3s: 仅前进 (vx=0.5)
 - 6s: 前进 + 自转 (vx=0.5, wz=1.0)
 - 2s: 停止

将帮助验证：在自转期间，发布到 /cmd_vel 的机体速度是否被正确变换，且地面速度方向（/cmd_vel_stamped）保持不变。
"""

import rospy
from geometry_msgs.msg import Twist, TwistStamped
import tf2_ros
import math
import time


def quat_to_yaw(qw, qx, qy, qz):
    return math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))


if __name__ == '__main__':
    rospy.init_node('auto_teleop_test')
    pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_stamped = rospy.Publisher('/cmd_vel_stamped', TwistStamped, queue_size=1)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(20)

    def publish_world(vx_w, vy_w, wz, duration):
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < duration and not rospy.is_shutdown():
            # lookup yaw from odom->base_link
            yaw = 0.0
            try:
                tfst = tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(0.1))
                q = tfst.transform.rotation
                yaw = quat_to_yaw(q.w, q.x, q.y, q.z)
            except Exception as e:
                # TF may not be ready yet
                rospy.logwarn_throttle(5, 'TF lookup failed: %s' % str(e))

            # world -> body: v_body = R(-yaw) * v_world
            vbx = math.cos(yaw) * vx_w + math.sin(yaw) * vy_w
            vby = -math.sin(yaw) * vx_w + math.cos(yaw) * vy_w

            t = Twist()
            t.linear.x = vbx
            t.linear.y = vby
            t.angular.z = wz

            ts = TwistStamped()
            ts.header.stamp = rospy.Time.now()
            ts.header.frame_id = 'odom'
            ts.twist.linear.x = vx_w
            ts.twist.linear.y = vy_w
            ts.twist.angular.z = wz

            pub_stamped.publish(ts)
            pub_twist.publish(t)
            rate.sleep()

    # wait a bit for topics and tf
    rospy.loginfo('Auto teleop test starting in 2s...')
    time.sleep(2.0)

    rospy.loginfo('Phase 1: forward only (3s)')
    publish_world(0.5, 0.0, 0.0, 3.0)

    rospy.loginfo('Phase 2: forward + spin (6s)')
    publish_world(0.5, 0.0, 1.0, 6.0)

    rospy.loginfo('Phase 3: stop (2s)')
    publish_world(0.0, 0.0, 0.0, 2.0)

    rospy.loginfo('Auto teleop test finished')

