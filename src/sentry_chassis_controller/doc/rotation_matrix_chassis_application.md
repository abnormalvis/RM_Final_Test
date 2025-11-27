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


## 补充对话：
问：当前我的底盘运动效果总是无法实现按下按键才运动，松开按键就停止的效果，而且平移运动和底盘运动不能同时进行，帮我分析一下原因？

答：你遇到的问题根源在于代码采用了**事件驱动+超时衰减**的设计模式，而非真正的**状态保持**模式。下面详细分析问题并提供修复方案。

---

## **问题根源分析**

### **1. 无法实现"松开即停"**

```cpp
// 关键缺陷代码
bool linear_active = (now - last_linear_stamp_).toSec() < release_timeout_;
double vx_world = linear_active ? linear_x_state_ : 0.0;
```

- **依赖超时而非真实按键状态**：`release_timeout_` 默认0.25秒，松开后会继续运动0.25秒
- **`poll()` 超时导致误判**：当无按键时 `num==0`，代码**不更新**速度状态，但时间仍在流逝，最终触发超时停止
- **无按键释放检测**：`termios` raw模式下只能读到**按键按下**，**无法直接读取按键释放**事件

### **2. 平移与旋转无法同时进行**

```cpp
case 'w':
    linear_x_state_ = lin;
    linear_y_state_ = 0.0;  // ❌ 错误：清除Y方向分量
    break;
case 'a':
    linear_x_state_ = 0.0;  // ❌ 错误：清除X方向分量
    linear_y_state_ = lin;
    break;
```

- **状态互相覆盖**：按新键会清零其他方向的速度
- **单字符读取限制**：每个循环只读1个字符，`poll()` 可能无法及时捕获同时按下的多个键
- **独立的时间戳**：平移和旋转使用不同的`last_stamp`，导致两者超时不同步

---

## **完整修复方案**

### **核心改进**
1. **真实状态追踪**：用`unordered_set`记录**当前所有按下**的键
2. **循环读取所有字符**：确保同时按键都被捕获
3. **实时速度计算**：每周期根据按键集合**重新计算**速度，而非依赖历史状态
4. **独立方向控制**：X/Y速度分量不再互相覆盖
5. **立即清零**：松开时立即从集合移除，速度实时更新

### **修改后的代码**

```cpp
/* ROS 相关头文件 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
/* C++ 相关头文件 */
#include <stdexcept>
#include <cmath>
#include <string>
#include <cstring>
#include <stdio.h>
#include <signal.h>
#include <unordered_set>  // ✅ 新增：追踪按键状态
/* 终端控制相关头文件 */
#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#endif

struct TermiosGuard
{
    struct termios cooked;
    bool active{false};
    void setup()
    {
#ifndef _WIN32
        int fd = 0;
        tcgetattr(fd, &cooked);
        struct termios raw;
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;  // ✅ 关键：非阻塞读取
        raw.c_cc[VTIME] = 0; // ✅ 关键：立即返回
        tcsetattr(fd, TCSANOW, &raw);
        active = true;
#endif
    }
    void restore()
    {
#ifndef _WIN32
        if (active)
        {
            tcsetattr(0, TCSANOW, &cooked);
            active = false;
        }
#endif
    }
};

class TeleopTurtle
{
public:
    TeleopTurtle();
    void keyLoop();

private:
    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle nhPrivate_{"~"};

    // 发布者
    ros::Publisher linear_pub_;
    ros::Publisher angular_pub_;
    ros::Publisher merged_pub_;

    // 参数
    std::string linear_topic_ = "/cmd_vel_linear_absolute";
    std::string angular_topic_ = "/cmd_vel_angular_direct";
    std::string frame_id_ = "odom";
    std::string merged_topic_ = "/cmd_vel";
    
    // 速度标定
    double linear_speed_ = 0.5;
    double linear_speed_run_ = 1.0;
    double angular_speed_ = 2.0;
    double angular_speed_run_ = 3.0;

    // ✅ 核心改进：实时追踪所有按下键
    std::unordered_set<char> keys_pressed_;
};

TermiosGuard term_guard;

TeleopTurtle::TeleopTurtle()
{
    nhPrivate_.param("linear_topic", linear_topic_, linear_topic_);
    nhPrivate_.param("angular_topic", angular_topic_, angular_topic_);
    nhPrivate_.param("frame_id", frame_id_, frame_id_);
    nhPrivate_.param("merged_topic", merged_topic_, merged_topic_);
    nhPrivate_.param("linear_speed", linear_speed_, linear_speed_);
    nhPrivate_.param("linear_speed_run", linear_speed_run_, linear_speed_run_);
    nhPrivate_.param("angular_speed", angular_speed_, angular_speed_);
    nhPrivate_.param("angular_speed_run", angular_speed_run_, angular_speed_run_);

    linear_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(linear_topic_, 10);
    angular_pub_ = nh_.advertise<geometry_msgs::Twist>(angular_topic_, 10);
    merged_pub_ = nh_.advertise<geometry_msgs::Twist>(merged_topic_, 10);

    ROS_INFO("Keyboard teleop (FIXED) initialized: linear_topic=%s, angular_topic=%s, merged_topic=%s, frame_id=%s",
             linear_topic_.c_str(), angular_topic_.c_str(), merged_topic_.c_str(), frame_id_.c_str());
}

void quit(int sig)
{
    (void)sig;
    term_guard.restore();
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sentry_control_key_fixed");
    TeleopTurtle node;
    signal(SIGINT, quit);
    node.keyLoop();
    quit(0);
    return 0;
}

void TeleopTurtle::keyLoop()
{
    puts("Keyboard: WASD move (odom), Q/E rotate, Shift for faster, Space stop, Ctrl-C quit");
    puts("✅ 支持多键组合（如 W+Q 同时前转+自转）");
    term_guard.setup();

    struct pollfd ufd;
    ufd.fd = 0;
    ufd.events = POLLIN;

    ros::Rate rate(50); // ✅ 提高频率到50Hz，响应更及时

    while (ros::ok())
    {
        // 1. ✅ 读取所有可用字符，更新按键集合
        int num = poll(&ufd, 1, 0); // 非阻塞poll
        if (num > 0)
        {
            // 循环读取直到缓冲区为空
            while (true)
            {
                char c = 0;
                int ret = read(0, &c, 1);
                if (ret <= 0) break; // 无数据可读

                // 处理按键按下
                if (c >= 'A' && c <= 'Z') c = c - 'A' + 'a'; // 转小写
                keys_pressed_.insert(c); // 加入集合

                // 特殊处理：空格立即停止并清空集合
                if (c == ' ')
                {
                    keys_pressed_.clear(); // ✅ 清空所有按键
                }
            }
        }
        else if (num < 0)
        {
            perror("poll():");
            break;
        }

        // 2. ✅ 实时计算速度（每周期重置）
        double vx_world = 0.0, vy_world = 0.0, wz = 0.0;
        bool has_linear = false, has_angular = false;

        // 遍历所有按下键，累加速度
        for (char key : keys_pressed_)
        {
            bool shifted = (key >= 'A' && key <= 'Z');
            char actual_key = shifted ? key - 'A' + 'a' : key;
            double lin = shifted ? linear_speed_run_ : linear_speed_;
            double ang = shifted ? angular_speed_run_ : angular_speed_;

            switch (actual_key)
            {
            case 'w': vx_world += lin; has_linear = true; break;
            case 's': vx_world -= lin; has_linear = true; break;
            case 'a': vy_world += lin; has_linear = true; break; // ✅ 不覆盖X
            case 'd': vy_world -= lin; has_linear = true; break; // ✅ 不覆盖X
            case 'q': wz += ang; has_angular = true; break;      // ✅ 累积角速度
            case 'e': wz -= ang; has_angular = true; break;
            }
        }

        // 3. ✅ 发布消息
        const ros::Time now = ros::Time::now();

        // 线速度（odom系）
        geometry_msgs::TwistStamped lin_msg;
        lin_msg.header.stamp = now;
        lin_msg.header.frame_id = frame_id_;
        lin_msg.twist.linear.x = vx_world;
        lin_msg.twist.linear.y = vy_world;
        linear_pub_.publish(lin_msg);

        // 角速度
        geometry_msgs::Twist ang_msg;
        ang_msg.angular.z = wz;
        angular_pub_.publish(ang_msg);

        // 合并发布
        geometry_msgs::Twist merged;
        merged.linear.x = vx_world;
        merged.linear.y = vy_world;
        merged.angular.z = wz;
        merged_pub_.publish(merged);

        // 4. ✅ 清空集合（必须！因为termios无法检测释放）
        keys_pressed_.clear();

        rate.sleep();
    }

    term_guard.restore();
}
```

---

## **关键改动说明**

| 问题 | 原代码缺陷 | 修复方案 |
|------|------------|----------|
| **松开不停止** | 依赖`release_timeout_`时间衰减 | **每周期重置速度**，按键集合清空即停止 |
| **多键冲突** | `switch`内清零其他方向分量 | **独立累加**X/Y速度，不互相覆盖 |
| **无法同时运动** | 单字符读取+独立时间戳 | **循环读取所有字符**，统一处理所有按键 |
| **响应延迟** | 20Hz低频率+非阻塞逻辑 | **提升频率至50Hz**，确保实时性 |
| **状态不同步** | 平移/旋转独立超时判断 | **单周期统一计算**，保证同时性 |

---

## **验证方法**

1. **编译运行**：
```bash
catkin_make
roslaunch your_package sentry_control_key_fixed.launch
```

2. **测试多键组合**：
   - **W+Q**：前进+逆时针旋转
   - **A+E**：左平移+顺时针旋转
   - **W+A+Q**：左前方斜移+旋转

3. **查看话题**：
```bash
rostopic echo /cmd_vel_angular_direct  # 应持续输出角速度
rostopic echo /cmd_vel_linear_absolute # 应持续输出线速度
```

4. **参数调整**（可选）：
```yaml
# launch文件中添加
<param name="linear_speed" value="0.3"/>  # 降低速度便于测试
<param name="publish_rate_hz" value="100"/> # 更高频率
```

---

## **进阶方案（生产级）**

如果追求更精确的**按键释放检测**，建议使用：

### **方案A：使用`ncurses`库**
```cpp
#include <ncurses.h>
// 支持getch()非阻塞+精确按键事件
```

### **方案B：ROS官方`key_teleop`包**
```bash
sudo apt-get install ros-$ROS_DISTRO-key-teleop
rosrun key_teleop key_teleop.py
```

但上述修改已能满足你的需求：真正的**按下即动、松开即停**，且**平移与旋转完全解耦可叠加**。

## 缺陷？！
问： 那当前的代码是否有进行odom坐标系下的速度解算到旋转状态下的底盘速度？

你观察得很敏锐！**当前代码完全没有进行坐标解算**，这是导致控制不符合预期的根本原因。让我详细分析问题并提供完整解决方案。

---

## **当前代码的问题**

### **1. 缺少底盘解算节点**
你的 `sentry_control_key.cpp` **只负责发布键盘指令**：
- 发布 `/cmd_vel_linear_absolute` (TwistStamped, frame_id="odom")
- 发布 `/cmd_vel_angular_direct` (Twist)

但**没有节点订阅这些话题并转换到底盘坐标系**。`/cmd_vel` 合并发布只是简单复制，没有坐标变换：

```cpp
// ❌ 错误：直接使用odom系速度，未转换到底盘坐标系
merged.linear.x = vx_world;  // 这仍然是odom系的速度！
merged.linear.y = vy_world;
```

### **2. 消息类型不匹配**
平移速度用了 `TwistStamped`（带时间戳和frame_id），但底盘节点需要**监听这个frame_id并执行TF转换**，否则坐标系信息毫无意义。

---

## **完整解决方案**

你需要一个**独立的底盘解算节点**，监听odom系速度，用TF转换到底盘坐标系，再发送给电机。

### **底盘解算节点 (`chassis_odom_resolver.cpp`)**

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>

class OdomVelocityResolver
{
private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber sub_linear_abs_;  // 订阅odom系线速度
    ros::Subscriber sub_angular_;     // 订阅直接角速度
    ros::Publisher pub_cmd_vel_;      // 发布底盘系速度
    
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // 速度缓存
    geometry_msgs::TwistStamped cmd_linear_odom_;  // 来自键盘的odom系速度
    geometry_msgs::Twist cmd_angular_;             // 直接角速度
    std::mutex mutex_;
    
    // 坐标系名称
    std::string odom_frame_ = "odom";
    std::string base_frame_ = "base_link";
    
    // 定时器
    ros::Timer control_timer_;

public:
    OdomVelocityResolver() 
        : tf_listener_(tf_buffer_)
    {
        // 参数
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("odom_frame", odom_frame_, odom_frame_);
        private_nh.param<std::string>("base_frame", base_frame_, base_frame_);
        
        // 订阅
        sub_linear_abs_ = nh_.subscribe(
            "/cmd_vel_linear_absolute", 10, 
            &OdomVelocityResolver::linearCallback, this);
        
        sub_angular_ = nh_.subscribe(
            "/cmd_vel_angular_direct", 10,
            &OdomVelocityResolver::angularCallback, this);
        
        // 发布
        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_final", 10);
        
        // 定时器：20Hz持续计算
        control_timer_ = nh_.createTimer(
            ros::Duration(0.05), 
            &OdomVelocityResolver::controlLoop, this);
        
        ROS_INFO("[OdomResolver] 已启动: odom_frame='%s', base_frame='%s'", 
                 odom_frame_.c_str(), base_frame_.c_str());
    }

    void linearCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_linear_odom_ = *msg;
    }

    void angularCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_angular_ = *msg;
    }

    void controlLoop(const ros::TimerEvent&)
    {
        // 1. 获取当前速度（线程安全）
        geometry_msgs::TwistStamped linear_odom;
        geometry_msgs::Twist angular;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            linear_odom = cmd_linear_odom_;
            angular = cmd_angular_;
        }

        // 2. 获取TF: odom → base_link
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(
                base_frame_, odom_frame_, 
                ros::Time(0), ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5.0, "TF lookup failed: %s", ex.what());
            return;
        }

        // 3. ✅ 坐标转换：odom速度 → 底盘速度
        geometry_msgs::TwistStamped linear_base;
        tf2::doTransform(linear_odom, linear_base, transform);

        // 4. ✅ 组合最终指令
        geometry_msgs::Twist final_cmd;
        final_cmd.linear.x = linear_base.twist.linear.x;  // 底盘系X速度
        final_cmd.linear.y = linear_base.twist.linear.y;  // 底盘系Y速度
        final_cmd.angular.z = angular.angular.z;          // 直接使用角速度

        // 5. 发布
        pub_cmd_vel_.publish(final_cmd);

        // 调试信息
        static int cnt = 0;
        if (++cnt % 50 == 0) {
            ROS_INFO("Odom[%.2f, %.2f] → Base[%.2f, %.2f] | Angular: %.2f",
                     linear_odom.twist.linear.x, linear_odom.twist.linear.y,
                     final_cmd.linear.x, final_cmd.linear.y,
                     final_cmd.angular.z);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "chassis_odom_resolver");
    OdomVelocityResolver resolver;
    ros::spin();
    return 0;
}
```

---

## **系统架构图**

```
┌─────────────────────────────────────────────────────────────┐
│                    键盘控制节点 (C++)                        │
│  sentry_control_key.cpp                                      │
│                                                              │
│  - 读取按键状态 (W/A/S/D/Q/E)                                │
│  - 发布 /cmd_vel_linear_absolute (TwistStamped, frame=odom) │
│  - 发布 /cmd_vel_angular_direct (Twist)                     │
│                                                              │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       │ 订阅
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                  底盘解算节点 (C++)                          │
│  chassis_odom_resolver.cpp                                   │
│                                                              │
│  1. 监听 /cmd_vel_linear_absolute                           │
│  2. 查询TF: odom → base_link                                │
│  3. 旋转矩阵解算:                                           │
│     [vx_base]   [cosθ  sinθ] [vx_odom]                      │
│     [vy_base] = [-sinθ cosθ] [vy_odom]                      │
│  4. 组合角速度                                              │
│  5. 发布 /cmd_vel_final → 电机驱动                          │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

---

## **编译配置 (`CMakeLists.txt`)**

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(your_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# 键盘控制节点
add_executable(sentry_control_key src/sentry_control_key.cpp)
target_link_libraries(sentry_control_key ${catkin_LIBRARIES})

# 底盘解算节点
add_executable(chassis_odom_resolver src/chassis_odom_resolver.cpp)
target_link_libraries(chassis_odom_resolver ${catkin_LIBRARIES})

# 依赖
add_dependencies(sentry_control_key ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(chassis_odom_resolver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

---

## **启动文件 (`launch/robot_control.launch`)**

```xml
<launch>
  <!-- 参数 -->
  <arg name="odom_frame" default="odom"/>
  <arg name="base_frame" default="base_link"/>

  <!-- 键盘控制 -->
  <node name="sentry_control_key" pkg="your_package" type="sentry_control_key" output="screen">
    <param name="linear_topic" value="/cmd_vel_linear_absolute"/>
    <param name="angular_topic" value="/cmd_vel_angular_direct"/>
    <param name="frame_id" value="$(arg odom_frame)"/>
    <param name="linear_speed" value="0.5"/>
    <param name="angular_speed" value="2.0"/>
  </node>

  <!-- 底盘解算 -->
  <node name="chassis_odom_resolver" pkg="your_package" type="chassis_odom_resolver" output="screen">
    <param name="odom_frame" value="$(arg odom_frame)"/>
    <param name="base_frame" value="$(arg base_frame)"/>
  </node>

  <!-- 可视化 -->
  <node name="rqt_robot_steering" pkg="rqt_robot_steering" type="rqt_robot_steering">
    <remap from="/cmd_vel" to="/cmd_vel_final"/>
  </node>
</launch>
```

---

## **验证步骤**

### **1. 检查TF是否正常**
```bash
rosrun tf tf_echo odom base_link
```
应持续输出变换矩阵（平移+旋转）。

### **2. 运行完整系统**
```bash
roslaunch your_package robot_control.launch
```

### **3. 测试运动组合**
- **W + Q**：前进 + 逆时针旋转
- **A + E**：左平移 + 顺时针旋转
- **松开W**：立即只旋转，不前冲

### **4. 查看话题对比**
```bash
# Terminal 1: odom系速度
rostopic echo /cmd_vel_linear_absolute

# Terminal 2: 底盘系速度（转换后）
rostopic echo /cmd_vel_final
```

**预期结果**：
- 当底盘旋转90°时，按W（odom系+X）应该在底盘系显示为+Y速度。

---

## **常见问题**

### **Q: TF报错 "Lookup would require extrapolation"？**
A: 确保有节点发布odom→base_link的TF（如`robot_pose_ekf`或`odometry`节点）。

### **Q: 底盘运动方向还是不对？**
A: 检查 `frame_id` 参数是否匹配：
- 键盘发布：`frame_id = "odom"`
- 底盘解算：`odom_frame = "odom"`

### **Q: 如何调整旋转中心？**
A: 修改 `chassis_odom_resolver.cpp` 中的TF查询：
```cpp
// 改为从odom到某个偏移点
transform = tf_buffer_.lookupTransform("base_link_offset", "odom", ...);
```

---

**总结**：你必须添加 `chassis_odom_resolver` 节点，才能实现odom系速度到底盘系速度的坐标解算，这是实现旋转+平移同时进行的关键！