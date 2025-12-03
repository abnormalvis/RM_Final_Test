/*
 * 舵轮底盘键盘控制节点（Feature 增强版）
 * 功能：
 *   - 键盘 teleop（遥操作）：通过 WASD 控制平移，QE 控制旋转
 *   - 支持同时平移与旋转（多按键组合）
 *   - 支持两种速度模式：
 *     * global 模式：按键定义世界坐标系（odom）下的运动意图
 *     * chassis 模式：按键定义底盘坐标系（base_link）下的运动意图
 *   - 按键状态持久化（按住生效，松开后超时自动清零）
 *   - Shift 加速、速度归一化（防止斜向速度过大）
 *   - 发布 TwistStamped（带坐标系标识）与 Twist（底盘系速度命令）
 * 
 * 详细说明见文档：docs/键盘同时平移旋转功能说明.md
 */

/* ROS 相关头文件 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

/* C++ 相关头文件 */
#include <stdexcept>
#include <cmath>
#include <string>
#include <cstring>
#include <stdio.h>
#include <signal.h>

/* 终端控制相关头文件（用于非阻塞键盘输入） */
#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#endif

/*
 * 终端设置守卫类
 * 功能：在构造时设置终端为原始模式（非缓冲、非回显），在析构或手动调用时恢复
 * 用途：实现非阻塞的键盘输入，避免需要按 Enter 才能读取按键
 */
struct TermiosGuard
{
    struct termios cooked;  // 保存原始终端设置（用于恢复）
    bool active{false};     // 标志是否已激活原始模式
    
    // 设置终端为原始模式（非规范、非回显）
    void setup()
    {
#ifndef _WIN32
        int fd = 0;  // 标准输入文件描述符
        tcgetattr(fd, &cooked);  // 保存当前终端设置
        struct termios raw;
        memcpy(&raw, &cooked, sizeof(struct termios));

        // 关闭规范模式（ICANON）和回显（ECHO）
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VEOL] = 1;  // 行结束符
        raw.c_cc[VEOF] = 2;  // 文件结束符

        tcsetattr(fd, TCSANOW, &raw);  // 立即应用新设置
        active = true;
#endif
    }

    // 恢复终端到原始设置
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

/*
 * 键盘控制主类
 * 管理键盘输入、速度计算、坐标变换与消息发布
 */
class TeleopTurtle
{
public:
    TeleopTurtle();
    void keyLoop();  // 主循环：监听键盘并发布速度命令

private:
    // ==================== ROS 句柄 ====================
    ros::NodeHandle nh_;         // 公共命名空间句柄
    ros::NodeHandle nhPrivate_ = ros::NodeHandle("~");  // 私有命名空间句柄（用于参数）
             // 注意：此节点不再订阅里程计或使用 yaw 信息
    // 底盘坐标系速度发布器（/cmd_vel，Twist 消息）
    ros::Publisher twist_pub_;
    bool publish_zero_when_idle_ = false;
    // 速度指令话题名称
    std::string cmd_vel_topic_;          // Twist 话题（默认 /cmd_vel）


    // ==================== 速度模式与坐标系 ====================
    // velocity_mode_: "global" 表示按键为全局系意图（world/odom frame）
    //                 "chassis" 表示按键为底盘系意图（body/base_link frame）
    std::string velocity_mode_ = "global";
    std::string global_frame_ = "odom";         // 全局参考坐标系名称
    std::string base_link_frame_ = "base_link"; // 底盘坐标系名称
    // 注意：此节点不再订阅里程计或使用 yaw 信息
    
    // ==================== 模式标志（未使用） ====================
    bool field_centric_ = true;     // 场地中心模式（保留）
    bool spinning_top_mode_ = true; // 小陀螺模式（保留）
};

// 全局终端守卫对象（用于信号处理时恢复终端设置）
TermiosGuard term_guard;

/*
 * 构造函数：读取参数、初始化发布器与订阅器
 */
TeleopTurtle::TeleopTurtle()
{
    // 读取速度指令话题名称（可通过 launch 文件或参数服务器配置）
    nhPrivate_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));

    // 读取空闲状态下是否发布零速度（默认 false，仅在有运动时发布）
    nhPrivate_.param("publish_zero_when_idle", publish_zero_when_idle_, publish_zero_when_idle_);

    // 读取速度模式（"global" 或 "chassis"）
    nhPrivate_.param("velocity_mode", velocity_mode_, velocity_mode_);

    // 读取坐标系名称
    nhPrivate_.param("global_frame", global_frame_, global_frame_);
    nhPrivate_.param("base_link_frame", base_link_frame_, base_link_frame_);


    // 创建底盘坐标系速度发布器（Twist 消息，用于控制器订阅）
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
}

/*
 * 退出信号处理函数（Ctrl-C）
 * 功能：恢复终端设置并安全退出
 */
void quit(int sig)
{
    (void)sig;
    term_guard.restore();  // 恢复终端到原始设置
    ros::shutdown();
    exit(0);
}

/*
 * 主函数：初始化 ROS 节点并启动键盘循环
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sentry_control_key");  // 初始化节点
    TeleopTurtle node;  // 创建键盘控制对象

    signal(SIGINT, quit);  // 注册 Ctrl-C 信号处理函数

    node.keyLoop();  // 进入键盘监听主循环
    quit(0);         // 正常退出（实际不会执行到这里）

    return 0;
}

/*
 * 键盘监听主循环
 * 功能：
 *   1. 设置终端为非阻塞原始模式
 *   2. 使用 poll() 监听键盘输入
 *   3. 维护按键状态（支持同时按下多个键）
 *   4. 根据按键状态计算速度并发布
 *   5. 支持超时自动清零（0.5秒无新按键）
 */
void TeleopTurtle::keyLoop()
{
    // 打印使用说明
    puts("Reading from keyboard (WASD to move, Q/E rotate, Shift+WASD run, c stop, Ctrl-C quit) [velocity_mode: global|chassis]");
    term_guard.setup();  // 设置终端为原始模式

    // ==================== 读取速度参数 ====================
    double walk_vel = 0.5;        // 步行速度（m/s）
    double run_vel = 1.0;         // 跑步速度（m/s，Shift 加速）
    double default_omega = 1.0;   // 默认角速度（rad/s）

    // 发布频率（Hz）
    int hz = 10;
    
    // 从参数服务器读取（可通过 launch 文件配置）
    nhPrivate_.param("walk_vel", walk_vel, walk_vel);
    nhPrivate_.param("run_vel", run_vel, run_vel);
    nhPrivate_.param("default_omega", default_omega, default_omega);
    nhPrivate_.param("hz", hz, hz);

    // ==================== 初始化速度消息 ====================
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    // ==================== 创建 poll 结构体用于监听键盘输入 ====================
    struct pollfd ufd;
    ufd.fd = 0;          // 标准输入（stdin）
    ufd.events = POLLIN; // 监听数据可读事件

    // ==================== 按键状态（支持多按键同时按下） ====================
    bool key_w = false, key_s = false, key_a = false, key_d = false;  // 平移按键
    bool key_q = false, key_e = false;  // 旋转按键
    bool shifted = false;  // Shift 加速标志

    ros::Rate rate(hz);  // 发布频率控制

    // ==================== 按键超时机制（用于自动清零） ====================
    ros::Time last_key_time = ros::Time::now();  // 上次按键时间戳
    double key_timeout = 0.5;  // 超时时间（秒）：0.5秒无新按键则清零所有状态

    // ==================== 主循环 ====================
    while (ros::ok())
    {
        // 等待键盘输入，超时时间为 1000/hz 毫秒（例如 100ms @ 10Hz）
        int num = poll(&ufd, 1, 1000 / hz);
        if (num < 0)
        {
            perror("poll():");
            break;
        }

        if (num == 0)  // poll 超时（无新键盘输入）
        {
            // 检查按键是否需要超时清零
            ros::Time now = ros::Time::now();
            if ((now - last_key_time).toSec() > key_timeout)
            {
                // 超过 0.5 秒没有新按键，清零所有按键状态
                key_w = key_s = key_a = key_d = false;
                key_q = key_e = false;
            }

            // 根据配置决定是否发布当前速度（可能为零）
            if (publish_zero_when_idle_)
            {
                twist_pub_.publish(twist);
            }
            else
            {
                // 仅当速度非零时发布（避免不必要的消息）
                if (std::abs(twist.linear.x) > 1e-9 || std::abs(twist.linear.y) > 1e-9 || std::abs(twist.angular.z) > 1e-9)
                {
                    twist_pub_.publish(twist);
                }
            }
            rate.sleep();
            continue;
        }

        // ==================== 读取按键字符 ====================
        char c = 0;
        if (read(0, &c, 1) < 0)
        {
            perror("read():");
            break;
        }

        bool any_key_pressed = false;  // 标记本次循环是否有按键输入（用于发布判断）

        // ==================== 处理 Shift 加速键 ====================
        // 大写字母表示按下 Shift（加速模式）
        if (c >= 'A' && c <= 'Z')
        {
            shifted = true;
            any_key_pressed = true;
        }
        else if (c >= 'a' && c <= 'z')
        {
            shifted = false;
        }

        // 统一映射为小写字母（便于后续处理）
        char lower = c;
        if (c >= 'A' && c <= 'Z')
            lower = c - 'A' + 'a';

        // ==================== 更新按键状态（按下时设置为 true，互斥处理） ====================
        switch (lower)
        {
        case 'w':  // 前进
            key_w = true;
            key_s = false;  // 前后互斥
            any_key_pressed = true;
            last_key_time = ros::Time::now();  // 更新按键时间戳
            break;
        case 's':  // 后退
            key_s = true;
            key_w = false;  // 前后互斥
            any_key_pressed = true;
            last_key_time = ros::Time::now();
            break;
        case 'a':  // 左平移
            key_a = true;
            key_d = false;  // 左右互斥
            any_key_pressed = true;
            last_key_time = ros::Time::now();
            break;
        case 'd':  // 右平移
            key_d = true;
            key_a = false;  // 左右互斥
            any_key_pressed = true;
            last_key_time = ros::Time::now();
            break;
        case 'q':  // 左转
            key_q = true;
            key_e = false;  // 左转右转互斥
            any_key_pressed = true;
            last_key_time = ros::Time::now();
            break;
        case 'e':  // 右转
            key_e = true;
            key_q = false;  // 左转右转互斥
            any_key_pressed = true;
            last_key_time = ros::Time::now();
            break;
        case 'c':  // 立即停止所有运动
            key_w = key_s = key_a = key_d = false;
            key_q = key_e = false;
            any_key_pressed = true;
            last_key_time = ros::Time::now();
            break;
        case '\x03':  // Ctrl-C（退出）
            quit(0);
            break;
        default:
            break;
        }

        // ==================== 根据按键状态计算速度（支持同时平移+旋转） ====================
        double linear_speed = shifted ? run_vel : walk_vel;  // 根据 Shift 选择速度档位
        double omega_speed = default_omega;

        double cur_vx_world = 0.0;  // 世界系 x 方向速度（累积）
        double cur_vy_world = 0.0;  // 世界系 y 方向速度（累积）
        double cur_omega = 0.0;     // 角速度（绕 z 轴）

        // 平移速度累加（WASD 可以组合，例如 W+A = 斜向移动）
        if (key_w)
            cur_vx_world += linear_speed;  // 向前
        if (key_s)
            cur_vx_world -= linear_speed;  // 向后
        if (key_a)
            cur_vy_world += linear_speed;  // 向左
        if (key_d)
            cur_vy_world -= linear_speed;  // 向右

        // 旋转速度（Q/E 独立于平移，可同时生效）
        if (key_q)
            cur_omega += omega_speed;  // 逆时针旋转
        if (key_e)
            cur_omega -= omega_speed;  // 顺时针旋转

        // ==================== 归一化斜向速度（防止 W+A 时速度变成 √2 倍） ====================
        double speed_magnitude = std::sqrt(cur_vx_world * cur_vx_world + cur_vy_world * cur_vy_world);
        if (speed_magnitude > linear_speed * 1.01)  // 允许小误差（浮点精度）
        {
            double scale = linear_speed / speed_magnitude;
            cur_vx_world *= scale;
            cur_vy_world *= scale;
        }

        // ==================== 计算最终速度命令 ====================
        // 速度模式决定按键语义：
        // - global 模式：WASD 为全局坐标系意图（控制器负责坐标变换）
        // - chassis 模式：WASD 为底盘坐标系意图（直接使用）
        double vx = cur_vx_world;
        double vy = cur_vy_world;
        
        twist.linear.x = vx;
        twist.linear.y = vy;
        twist.angular.z = cur_omega;
        
        // 日志输出（调试用）
        ROS_INFO_THROTTLE(1.0, "Teleop: mode=%s vel=(%.2f,%.2f) omega=%.2f",
                          velocity_mode_.c_str(), vx, vy, cur_omega);

        // ==================== 发布速度消息 ====================
        // 判断是否有运动（用于决定是否发布）
        bool has_motion = (std::abs(twist.linear.x) > 1e-9) || 
                          (std::abs(twist.linear.y) > 1e-9) || 
                          (std::abs(twist.angular.z) > 1e-9);
        
        if (any_key_pressed || publish_zero_when_idle_)
        {
            // 发布到 /cmd_vel（控制器根据自身 speed_mode 处理）
            if (has_motion || publish_zero_when_idle_)
                twist_pub_.publish(twist);
        }
        
        rate.sleep();  // 控制发布频率
    }

    term_guard.restore();  // 退出循环后恢复终端设置
    return;
}
