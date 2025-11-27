### 我在做一个键盘控制节点，我希望通过qe来控制底盘的顺逆时针旋转，也就是按下后会发布一个直接给底盘的角速度指令；然后通过wsad发布前后左右的速度指令，为了这个两种运动状态互不干扰，也就是说，底盘可以在自转的状态下前后左右平移运动。我希望wsad发布的速度指令，是底盘的中心的投影在odom坐标系下xy方向上的速度，而不是直接发送给底盘，底盘需要订阅这个绝对速度指令，再通过旋转矩阵解算到底盘的速度指令。

我需要实现一个**双模式解耦的键盘控制系统**：

- **QE**：直接发送**角速度**指令（底盘自转）
- **WSAD**：发送**odom坐标系下的绝对平移速度**，底盘节点需自行解算到自身坐标系
- **核心**：两种运动独立控制，可叠加（如边转边斜移）

下面提供完整实现方案，包含键盘发布节点和底盘解算节点：

---

### 1. 键盘控制节点（`keyboard_controller.py`）

```python
#!/usr/bin/env python3
import rospy
import sys
import select
import termios
import tty
from geometry_msgs.msg import Twist

# 配置参数
LINEAR_SPEED = 0.5    # odom系下平移速度 (m/s)
ANGULAR_SPEED = 1.0   # 直接角速度 (rad/s)

class KeyboardController:
    def __init__(self):
        rospy.init_node('keyboard_controller', anonymous=True)
        
        # 发布者：分离平移和旋转控制
        self.pub_linear = rospy.Publisher('/cmd_vel_linear', Twist, queue_size=10)
        
        # QE的角速度直接发给底盘（可选：如果底盘节点需要单独订阅）
        self.pub_angular = rospy.Publisher('/cmd_vel_angular', Twist, queue_size=10)
        
        # 用于显示当前状态
        self.current_linear = [0.0, 0.0]  # [vx, vy] in odom frame
        self.current_angular = 0.0       # rad/s
        
        # 绑定按键映射
        self.key_bindings = {
            'w': (1.0, 0.0),   # 前 (odom x+)
            's': (-1.0, 0.0),  # 后 (odom x-)
            'a': (0.0, 1.0),   # 左 (odom y+)
            'd': (0.0, -1.0),  # 右 (odom y-)
            # QE在键盘逻辑中直接处理为角速度
        }

    def get_key(self):
        """非阻塞按键读取"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        global settings
        settings = termios.tcgetattr(sys.stdin)
        
        print("键盘控制已启动:")
        print("- W/S: Odom坐标系下前进/后退")
        print("- A/D: Odom坐标系下左/右平移")
        print("- Q/E: 底盘自转（逆时针/顺时针）")
        print("- 空格: 停止")
        print("- Ctrl+C: 退出")
        
        rate = rospy.Rate(20)  # 20Hz
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                
                # 处理旋转（QE）
                if key == 'q':
                    self.current_angular = ANGULAR_SPEED  # 逆时针
                elif key == 'e':
                    self.current_angular = -ANGULAR_SPEED # 顺时针
                elif key == ' ':
                    self.current_angular = 0.0
                    self.current_linear = [0.0, 0.0]
                elif key in self.key_bindings:
                    # 累加速度（允许组合按键）
                    vx_factor, vy_factor = self.key_bindings[key]
                    self.current_linear[0] = vx_factor * LINEAR_SPEED
                    self.current_linear[1] = vy_factor * LINEAR_SPEED
                else:
                    # 无按键时速度归零（或根据需求改为保持）
                    self.current_linear = [0.0, 0.0]
                    # 角速度不归零，允许持续旋转
                
                # 发布平移速度（在odom坐标系下）
                linear_twist = Twist()
                linear_twist.linear.x = self.current_linear[0]
                linear_twist.linear.y = self.current_linear[1]
                self.pub_linear.publish(linear_twist)
                
                # 发布角速度（直接指令）
                angular_twist = Twist()
                angular_twist.angular.z = self.current_angular
                self.pub_angular.publish(angular_twist)
                
                rate.sleep()
                
        except rospy.ROSInterruptException:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    controller = KeyboardController()
    controller.run()

```

---

### 2. 底盘解算节点（`chassis_controller.cpp`）

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>

class ChassisController {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_linear_vel_;
    ros::Subscriber sub_angular_vel_;
    
    // 发布者
    ros::Publisher pub_cmd_vel_;
    
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // 速度缓存
    geometry_msgs::TwistStamped cmd_linear_absolute_;  // odom系下的平移
    geometry_msgs::Twist cmd_angular_;                  // 直接角速度
    std::mutex mutex_;
    
    // 配置参数
    std::string odom_frame_;
    std::string base_frame_;
    
public:
    ChassisController() 
        : private_nh_("~")
        , tf_listener_(tf_buffer_)
    {
        // 加载参数
        private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
        private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
        
        // 订阅
        sub_linear_vel_ = nh_.subscribe("/cmd_vel_linear", 10, 
            &ChassisController::linearVelCallback, this);
        sub_angular_vel_ = nh_.subscribe("/cmd_vel_angular", 10,
            &ChassisController::angularVelCallback, this);
        
        // 发布
        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        // 初始化定时器（用于持续发布）
        ros::Timer timer = nh_.createTimer(ros::Duration(0.1), 
            &ChassisController::controlLoop, this);
    }
    
    void linearVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_linear_absolute_.twist = *msg;
        cmd_linear_absolute_.header.stamp = ros::Time::now();
    }
    
    void angularVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_angular_ = *msg;
    }
    
    void controlLoop(const ros::TimerEvent&) {
        // 获取坐标变换
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(
                base_frame_, odom_frame_, ros::Time(0), ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5.0, "TF lookup failed: %s", ex.what());
            return;
        }
        
        // 解算平移速度（从odom到base）
        geometry_msgs::TwistStamped cmd_linear_base;
        tf2::doTransform(cmd_linear_absolute_, cmd_linear_base, transform);
        
        // 组合最终指令
        geometry_msgs::Twist final_cmd;
        final_cmd.linear.x = cmd_linear_base.twist.linear.x;
        final_cmd.linear.y = cmd_linear_base.twist.linear.y;
        final_cmd.angular.z = cmd_angular_.angular.z;  // 直接使用角速度
        
        // 发布
        pub_cmd_vel_.publish(final_cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "chassis_controller");
    ChassisController controller;
    ros::spin();
    return 0;
}
```

---

### 3. 启动文件（`launch/robot_control.launch`）

```xml
<launch>
    <!-- 键盘控制节点 -->
    <node name="keyboard_controller" pkg="your_package" type="keyboard_controller.py" output="screen">
        <param name="linear_speed" value="0.5"/>
        <param name="angular_speed" value="1.0"/>
    </node>
    
    <!-- 底盘解算节点 -->
    <node name="chassis_controller" pkg="your_package" type="chassis_controller" output="screen">
        <param name="odom_frame" value="odom"/>
        <param name="base_frame" value="base_footprint"/>
    </node>
    
    <!-- 可选：速度可视化 -->
    <node name="rqt_robot_steering" pkg="rqt_robot_steering" type="rqt_robot_steering">
        <remap from="/cmd_vel" to="/cmd_vel_absolute"/>
    </node>
</launch>
```

---

### 4. 核心原理说明

#### **坐标系变换公式**

设底盘在odom坐标系下的偏航角为 θ，则旋转矩阵（odom → base）：

```
| vx_base |   | cosθ  sinθ |   | vx_odom |
| vy_base | = | -sinθ cosθ | * | vy_odom |
```

#### **消息流**

```
键盘节点
├─ /cmd_vel_linear (Twist: vx_odom, vy_odom)
└─ /cmd_vel_angular (Twist: ω)

底盘节点
├─ 订阅 /cmd_vel_linear
├─ 订阅 /cmd_vel_angular
├─ 监听 TF: odom → base
└─ 发布 /cmd_vel (Twist: vx_base, vy_base, ω)
```

---

### 5. 使用说明

1. **编译C++节点**：
```bash
catkin_make
```

2. **运行**：
```bash
roslaunch your_package robot_control.launch
```

3. **测试TF**（确保odom到base的变换正常）：
```bash
rosrun tf tf_echo odom base_link
```

4. **查看速度指令**：
```bash
rostopic echo /cmd_vel
```

这样就实现了**解耦控制**：无论底盘如何旋转，WSAD始终控制其在odom坐标系下的绝对方向，而QE独立控制自转，两者可通过TF实时叠加！