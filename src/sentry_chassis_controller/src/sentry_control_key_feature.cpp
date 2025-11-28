/* ROS 相关头文件 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
/* C++ 相关头文件 */
#include <stdexcept>
#include <cmath>
#include <string>
#include <cstring>
#include <stdio.h>
#include <signal.h>
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
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
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
    void odomCb(const nav_msgs::Odometry::ConstPtr &msg);

private:
    // 变量
    ros::NodeHandle nh_;
    ros::NodeHandle nhPrivate_ = ros::NodeHandle("~");

    // 速度指令发布方，用于发布底盘在世界坐标系下的速度
    ros::Publisher twist_pub_;

    // 速度指令发布方，用于发布底盘在odom坐标系下的速度
    ros::Publisher twist_stamped_pub_;

    // 空闲状态下是否发布零速度
    bool publish_zero_when_idle_ = false;

    // 速度指令话题名称
    std::string cmd_vel_topic_;
    std::string cmd_vel_stamped_topic_;

    // 速度模式与参考坐标
    // velocity_mode_: "global" 表示按键为全局系意图（world frame），"chassis" 表示按键为底盘系（body frame）
    std::string velocity_mode_ = "global";
    std::string global_frame_ = "odom";         // 全局参考坐标系
    std::string base_link_frame_ = "base_link"; // 底盘坐标系
    std::string odom_topic_ = "/odom";          // 里程计话题
    // 键盘节点只订阅里程计，不再计算或查询 TF/topic 航向

    // 底盘的航向偏角
    double yaw_ = 0.0;
    double yaw_odom_ = 0.0;
    bool have_odom_yaw_ = false;
    // 仅使用里程计航向

    // 订阅里程计信息
    ros::Subscriber odom_sub_;
    // 订阅偏航角信息
    ros::Subscriber yaw_sub_;
    bool field_centric_ = true;     // 当为 true 时，键盘输入表示世界坐标系下的方向，旋转不影响平移方向
    bool spinning_top_mode_ = true; // 小陀螺模式默认开启
};

TermiosGuard term_guard;

TeleopTurtle::TeleopTurtle()
{
    nhPrivate_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    nhPrivate_.param("cmd_vel_stamped_topic", cmd_vel_stamped_topic_, std::string("/cmd_vel_stamped"));
    nhPrivate_.param("publish_zero_when_idle", publish_zero_when_idle_, publish_zero_when_idle_);
    nhPrivate_.param("velocity_mode", velocity_mode_, velocity_mode_);
    nhPrivate_.param("global_frame", global_frame_, global_frame_);
    nhPrivate_.param("base_link_frame", base_link_frame_, base_link_frame_);
    nhPrivate_.param("odom_topic", odom_topic_, odom_topic_);
    // nhPrivate_.param("yaw_topic", yaw_topic_, yaw_topic_);
    // nhPrivate_.param("yaw_source", yaw_source_, yaw_source_);
    // nhPrivate_.param("yaw_stale_sec", yaw_stale_sec_, yaw_stale_sec_);
    // nhPrivate_.param("tf_timeout_sec", tf_timeout_sec_, tf_timeout_sec_);

    // 局部坐标系速度发布方
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

    // 全局坐标系速度发布方
    twist_stamped_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(cmd_vel_stamped_topic_, 1);

    // 订阅 odom，作为 yaw 的唯一来源
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &TeleopTurtle::odomCb, this);
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
    ros::init(argc, argv, "sentry_control_key");
    TeleopTurtle node;

    signal(SIGINT, quit);

    node.keyLoop();
    quit(0);

    return 0;
}

void TeleopTurtle::keyLoop()
{
    puts("Reading from keyboard (WASD to move, Q/E rotate, Shift+WASD run, c stop, Ctrl-C quit) [velocity_mode: global|chassis]");
    term_guard.setup();

    // 速度参数
    double walk_vel = 0.5;
    double run_vel = 1.0;
    double default_omega = 1.0;
    int hz = 10;
    nhPrivate_.param("walk_vel", walk_vel, walk_vel);
    nhPrivate_.param("run_vel", run_vel, run_vel);
    nhPrivate_.param("default_omega", default_omega, default_omega);
    nhPrivate_.param("hz", hz, hz);

    // 组织速度信息
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    // 创建 poll 结构体用于监听键盘输入
    struct pollfd ufd;
    ufd.fd = 0;          // stdin
    ufd.events = POLLIN; // 数据可读事件标志位

    ros::Rate rate(hz);
    while (ros::ok())
    {
        int num = poll(&ufd, 1, 1000 / hz);
        if (num < 0)
        {
            perror("poll():");
            break;
        }

        if (num == 0)
        {
            // 当前速度可能发布超时：发布当前速度（可能为零）
            if (publish_zero_when_idle_)
            {
                twist_pub_.publish(twist);
            }
            else
            {
                if (std::abs(twist.linear.x) > 1e-9 || std::abs(twist.linear.y) > 1e-9 || std::abs(twist.angular.z) > 1e-9)
                {
                    twist_pub_.publish(twist);
                }
            }
            rate.sleep();
            continue;
        }

        char c = 0;
        if (read(0, &c, 1) < 0)
        {
            perror("read():");
            break;
        }

        // 每次循环重置本次按键请求（按下时生效，松开后不再保持）
        double cur_vx_world = 0.0; // 世界坐标系下的x轴方向的速度请求
        double cur_vy_world = 0.0; // 世界坐标系下的y轴方向的速度请求
        double cur_omega = 0.0;    // 机体角速度请求（Q/E 直接控制角速度）

        bool any_key_pressed = false; // 标记本次循环是否有按键输入

        static double translation_magnitude = 0.0; // 平移速度大小
        static double translation_angle = 0.0;     // 平移方向角（相对于底盘初始方向）
        static bool translation_active = false;    // 是否处于平移状态

        // 处理shift加速
        bool shifted = false;
        if (c >= 'A' && c <= 'Z')
            shifted = true;

        // 处理大小写字母统一映射
        char lower = c;
        if (shifted)
            lower = c - 'A' + 'a';

        double linear_speed = shifted ? run_vel : walk_vel;
        double omega_speed = default_omega;

        switch (lower)
        {
        case 'w':
            cur_vx_world = linear_speed;
            cur_vy_world = 0.0;
            any_key_pressed = true;
            break;
        case 's':
            cur_vx_world = -linear_speed;
            cur_vy_world = 0.0;
            any_key_pressed = true;
            break;
        case 'a':
            cur_vx_world = 0.0;
            cur_vy_world = linear_speed;
            any_key_pressed = true;
            break;
        case 'd':
            cur_vx_world = 0.0;
            cur_vy_world = -linear_speed;
            any_key_pressed = true;
            break; // right strafe
        case 'q':
            // Quat/E 直接作为角速度命令（按下时有效）
            cur_omega = omega_speed;
            any_key_pressed = true;
            break; // turn left
        case 'e':
            cur_omega = -omega_speed;
            any_key_pressed = true;
            break; // turn right
        case 'c':
            // 立即停止运动
            cur_vx_world = 0.0;
            cur_vy_world = 0.0;
            cur_omega = 0.0;
            any_key_pressed = true;
            break; // stop
        case 'f':
            break;
        // 删除小陀螺模式切换键 't'，模式始终开启
        case '\x03': // ctrl-c
            quit(0);
            break;
        default:
            break;
        }

        // 更新底盘航向角 yaw_：仅使用里程计提供的航向
        if (have_odom_yaw_)
        {
            yaw_ = yaw_odom_;
        }
        else
        {
            yaw_ = 0.0;
            ROS_WARN_THROTTLE(2.0, "No odom yaw yet; defaulting yaw to 0");
        }

        // 发布一个带时间戳的 TwistStamped，表示请求的速度（根据模式选择 frame_id）
        geometry_msgs::TwistStamped stamped;
        stamped.header.stamp = ros::Time::now();
        stamped.header.frame_id = (velocity_mode_ == "global") ? global_frame_ : base_link_frame_;

        // 根据模式组装 body/world 速度
        double body_vx = 0.0, body_vy = 0.0;
        double world_vx = 0.0, world_vy = 0.0;
        if (velocity_mode_ == "global")
        {
            // 按键定义为全局系意图（world），将其转换到 body 发布到 /cmd_vel
            world_vx = cur_vx_world;
            world_vy = cur_vy_world;
            double cos_yaw = std::cos(yaw_);
            double sin_yaw = std::sin(yaw_);
            body_vx = cos_yaw * world_vx + sin_yaw * world_vy; // R(-yaw) * [vx; vy]
            body_vy = -sin_yaw * world_vx + cos_yaw * world_vy;
            stamped.twist.linear.x = world_vx;
            stamped.twist.linear.y = world_vy;
            ROS_INFO_THROTTLE(1.0, "mode=global frame=%s yaw(odom)=%.3f world=(%.2f,%.2f) -> body=(%.2f,%.2f)",
                              global_frame_.c_str(), yaw_, world_vx, world_vy, body_vx, body_vy);
        }
        else // chassis 模式：按键直接定义底盘系速度
        {
            body_vx = cur_vx_world; // 在 chassis 模式下，W/S 映射到底盘 x，A/D 映射到底盘 y
            body_vy = cur_vy_world;
            // 为了记录/上层需要，如需世界系可在此再做 R(yaw) 变换得到 world_vx/world_vy；此处直接发布 base_link 帧
            stamped.twist.linear.x = body_vx;
            stamped.twist.linear.y = body_vy;
            ROS_INFO_THROTTLE(1.0, "mode=chassis body=(%.2f,%.2f) yaw(odom)=%.3f", body_vx, body_vy, yaw_);
        }
        // 角速度（机体系）直接使用按键请求
        twist.angular.z = cur_omega;
        stamped.twist.angular.z = cur_omega;

        // 写入 /cmd_vel（body frame）
        twist.linear.x = body_vx;
        twist.linear.y = body_vy;

        // 仅当本次按键请求非零（或用户允许空闲零发布）时发送
        bool has_motion = (std::abs(twist.linear.x) > 1e-9) || (std::abs(twist.linear.y) > 1e-9) || (std::abs(twist.angular.z) > 1e-9);
        if (any_key_pressed || publish_zero_when_idle_)
        {
            // 发布带时间戳的速度意图（global 模式为 global_frame_，chassis 模式为 base_link）
            twist_stamped_pub_.publish(stamped);
            // 如果有运动或配置允许空闲零发布，发布到 /cmd_vel
            if (has_motion || publish_zero_when_idle_)
                twist_pub_.publish(twist);
        }
        rate.sleep();
    }

    term_guard.restore();
    return;
}

void TeleopTurtle::odomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    const auto &q = msg->pose.pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double r, p, y;
    tf2::Matrix3x3(quat).getRPY(r, p, y);
    yaw_odom_ = y;
    have_odom_yaw_ = true;
}
