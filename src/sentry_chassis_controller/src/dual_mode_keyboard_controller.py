#!/usr/bin/env python3
"""
双模式解耦键盘控制器 - 基于rotation_matrix_chassis_application.md文档
支持旋转时平移运动而不出现螺旋线运动
"""

import rospy
import sys
import select
import termios
import tty
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import tf2_ros
import numpy as np

# 配置参数
LINEAR_SPEED = 0.5    # odom坐标系下平移速度 (m/s)
RUN_LINEAR_SPEED = 1.0
ANGULAR_SPEED = 1.0   # 直接角速度 (rad/s)
RUN_ANGULAR_SPEED = 1.5
PUBLISH_RATE = 20     # Hz

class DualModeKeyboardController:
    """
    双模式键盘控制器：
    - QE：直接发布角速度指令（底盘自转）
    - WSAD：发布odom坐标系下的绝对平移速度
    - 两种运动方式可以叠加，不会出现螺旋线运动
    """
    def __init__(self):
        rospy.init_node('dual_mode_keyboard_controller', anonymous=True)

        # 参数配置
        self.linear_speed = rospy.get_param('~linear_speed', LINEAR_SPEED)
        self.run_linear_speed = rospy.get_param('~run_linear_speed', RUN_LINEAR_SPEED)
        self.angular_speed = rospy.get_param('~angular_speed', ANGULAR_SPEED)
        self.run_angular_speed = rospy.get_param('~run_angular_speed', RUN_ANGULAR_SPEED)
        self.publish_rate = rospy.get_param('~publish_rate', PUBLISH_RATE)

        # 发布者：分离平移和旋转控制
        self.pub_linear_absolute = rospy.Publisher('/cmd_vel_linear_absolute', TwistStamped, queue_size=10)
        self.pub_angular_direct = rospy.Publisher('/cmd_vel_angular_direct', Twist, queue_size=10)

        # TF缓存
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 当前状态
        self.current_linear_vel = [0.0, 0.0]  # [vx, vy] in odom frame
        self.current_angular_vel = 0.0       # rad/s
        self.current_yaw = 0.0               # rad

        # 键位映射
        self.key_bindings = {
            'w': (1.0, 0.0),   # 前 (odom x+方向)
            's': (-1.0, 0.0),  # 后 (odom x-方向)
            'a': (0.0, 1.0),   # 左 (odom y+方向)
            'd': (0.0, -1.0),  # 右 (odom y-方向)
            'q': 'turn_left',
            'e': 'turn_right'
        }

        # 终端设置
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        """非阻塞按键读取"""
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            return key
        except KeyboardInterrupt:
            return '\x03'  # Ctrl-C
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def update_current_yaw(self):
        """通过TF获取当前底盘航向角"""
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_link", rospy.Time(0), rospy.Duration(0.1))
            q = transform.transform.rotation
            self.current_yaw = np.arctan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn_throttle(10.0, "TF lookup failed: %s" % str(ex))
            # 保持之前的yaw值

    def keyboard_loop(self):
        """主循环处理键盘输入"""
        print("双模式键盘控制已启动:")
        print("- W/S: Odom坐标系下前进/后退")
        print("- A/D: Odom坐标系下左/右平移")
        print("- Q/E: 底盘自转（逆时针/顺时针）")
        print("- C: 停止")
        print("- 空格: 清零平移速度")
        print("- F: 切换加速模式")
        print("- Ctrl-C: 退出")

        rate = rospy.Rate(self.publish_rate)
        is_run_shifted = False

        try:
            while not rospy.is_shutdown():
                # 处理键盘输入
                key = self.get_key()

                # 每次循环重置按键请求（按键按下时生效，松开后不保持）
                requested_linear = [0.0, 0.0]   # odom下的平移速度请求
                requested_angular = 0.0         # 角速度请求
                any_key_pressed = False

                # 处理大小写（加速模式）
                lower_key = key.lower()
                if key.isupper():
                    is_run_shifted = True
                else:
                    is_run_shifted = False

                # 根据按键状态设置请求速度
                if lower_key == 'q':
                    # Q：逆时针旋转
                    requested_angular = self.run_angular_speed if is_run_shifted else self.angular_speed
                    any_key_pressed = True
                elif lower_key == 'e':
                    # E：顺时针旋转
                    requested_angular = -self.run_angular_speed if is_run_shifted else -self.angular_speed
                    any_key_pressed = True
                elif lower_key in self.key_bindings and isinstance(self.key_bindings[lower_key], tuple):
                    # W/S/A/D：odom坐标系下平移
                    speed = self.run_linear_speed if is_run_shifted else self.linear_speed
                    vx_factor, vy_factor = self.key_bindings[lower_key]
                    requested_linear[0] = vx_factor * speed
                    requested_linear[1] = vy_factor * speed
                    any_key_pressed = True
                elif key == 'c':
                    # 停止所有运动
                    requested_linear = [0.0, 0.0]
                    requested_angular = 0.0
                    any_key_pressed = True
                elif key == ' ':
                    # 空格：清零平移速度，保持旋转
                    requested_linear = [0.0, 0.0]
                    any_key_pressed = True
                elif key == '\x03':  # Ctrl-C
                    break
                else:
                    # 无按键时清零线性速度，保持旋转速度
                    requested_linear = [0.0, 0.0]

                # 更新当前状态
                self.current_linear_vel = requested_linear
                self.current_angular_vel = requested_angular

                # 不按键时保持旋转（小陀螺模式的核心）
                if not any_key_pressed:
                    requested_linear = [0.0, 0.0]  # 清空平移

                # 发布消息
                self.publish_commands(requested_linear, requested_angular)

                # 更新yaw角
                self.update_current_yaw()

                rate.sleep()

        except rospy.ROSInterruptException:
            pass
        finally:
            # 清理操作
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def publish_commands(self, linear_vel, angular_vel):
        """发布分离的线性和角速度命令"""
        # 发布平移速度（odom坐标系下）- TwistStamped格式
        linear_msg = TwistStamped()
        linear_msg.header.stamp = rospy.Time.now()
        linear_msg.header.frame_id = "odom"
        linear_msg.twist.linear.x = linear_vel[0]
        linear_msg.twist.linear.y = linear_vel[1]
        linear_msg.twist.linear.z = 0.0

        # 发布角速度（直接指令）- Twist格式
        angular_msg = Twist()
        angular_msg.linear.x = 0.0
        angular_msg.linear.y = 0.0
        angular_msg.linear.z = 0.0
        angular_msg.angular.z = angular_vel

        self.pub_linear_absolute.publish(linear_msg)
        self.pub_angular_direct.publish(angular_msg)

        # 显示调试信息
        rospy.loginfo_throttle(1.0, "Linear: [%.2f, %.2f] m/s in odom | Angular: %.2f rad/s",
                              linear_vel[0], linear_vel[1], angular_vel)

def main():
    try:
        controller = DualModeKeyboardController()
        controller.keyboard_loop()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()