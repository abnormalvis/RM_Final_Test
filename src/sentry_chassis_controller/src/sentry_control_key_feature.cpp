/* ROS 相关头文件 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
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
    std::string yaw_topic_ = "/robot_yaw";      // 由 yaw_publisher 发布的航向角

    // 偏航角的来源
    std::string yaw_source_ = "topic";
    double yaw_stale_sec_ = 2.0;   // 判定 tf 航向“停滞”的阈值
    double tf_timeout_sec_ = 0.05; // TF 查询超时时间

    // 使用一个buffer对象来寻找tf变换的对象来实现底盘到odom坐标系的转换（作为备用方案）
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;

    // 底盘的航向偏角
    double yaw_ = 0.0;
    double yaw_tf_ = 0.0;
    double yaw_odom_ = 0.0;
    bool have_odom_yaw_ = false;
    ros::Time last_yaw_change_tf_;
    // 外部 yaw 订阅
    double yaw_topic_val_ = 0.0;
    bool have_yaw_topic_ = false;

    // 订阅 odom 作为备用航向来源
    ros::Subscriber odom_sub_;
    // 订阅外部 yaw
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
    nhPrivate_.param("yaw_topic", yaw_topic_, yaw_topic_);
    nhPrivate_.param("yaw_source", yaw_source_, yaw_source_);
    nhPrivate_.param("yaw_stale_sec", yaw_stale_sec_, yaw_stale_sec_);
    nhPrivate_.param("tf_timeout_sec", tf_timeout_sec_, tf_timeout_sec_);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    twist_stamped_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(cmd_vel_stamped_topic_, 1);
    // 监听 baselink -> odom 变换（备用）
    tfBuffer_.reset(new tf2_ros::Buffer(ros::Duration(10)));
    tfListener_.reset(new tf2_ros::TransformListener(*tfBuffer_));
    // 订阅 odom，作为 yaw 的备用来源
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &TeleopTurtle::odomCb, this);
    // 订阅外部 yaw
    yaw_sub_ = nh_.subscribe<std_msgs::Float64>(yaw_topic_, 1, [this](const std_msgs::Float64::ConstPtr &msg)
                                                {
        yaw_topic_val_ = msg->data;
        have_yaw_topic_ = true; });
    last_yaw_change_tf_ = ros::Time::now();
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

        // 更新底盘航向角 yaw_（优先 topic；可按参数回落到 tf/odom）
        double yaw_selected = yaw_;
        bool tf_ok = false;
        if (tfBuffer_)
        {
            try
            {
                geometry_msgs::TransformStamped tfst = tfBuffer_->lookupTransform(global_frame_, base_link_frame_, ros::Time(0), ros::Duration(tf_timeout_sec_));
                const auto &qr = tfst.transform.rotation;
                tf2::Quaternion q(qr.x, qr.y, qr.z, qr.w);
                double roll, pitch, yaw_tmp;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_tmp);
                yaw_tf_ = yaw_tmp;
                tf_ok = true;
                if (std::fabs(yaw_tf_ - yaw_selected) > 1e-6)
                {
                    last_yaw_change_tf_ = ros::Time::now();
                }
            }
            catch (const tf2::TransformException &ex)
            {
                ROS_WARN_THROTTLE(5.0, "TF lookup %s->%s failed: %s", global_frame_.c_str(), base_link_frame_.c_str(), ex.what());
            }
        }

        const bool tf_stale = (ros::Time::now() - last_yaw_change_tf_).toSec() > yaw_stale_sec_;
        if (yaw_source_ == "topic")
        {
            if (have_yaw_topic_)
                yaw_selected = yaw_topic_val_;
            else
                ROS_WARN_THROTTLE(2.0, "Yaw source 'topic' selected but no data on %s yet", yaw_topic_.c_str());
        }
        else if (yaw_source_ == "tf")
        {
            if (tf_ok)
                yaw_selected = yaw_tf_;
        }
        else if (yaw_source_ == "odom")
        {
            if (have_odom_yaw_)
                yaw_selected = yaw_odom_;
        }
        else // auto: topic > tf (not stale) > odom
        {
            if (have_yaw_topic_)
            {
                yaw_selected = yaw_topic_val_;
            }
            else if (tf_ok && !tf_stale)
            {
                yaw_selected = yaw_tf_;
            }
            else if (have_odom_yaw_)
            {
                yaw_selected = yaw_odom_;
                ROS_WARN_THROTTLE(2.0, "Yaw source fallback to odom (topic missing, tf stale=%d ok=%d)", (int)tf_stale, (int)tf_ok);
            }
        }
        yaw_ = yaw_selected;

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
            ROS_INFO_THROTTLE(1.0, "mode=global frame=%s yaw=%.3f (topic=%d tf=%.3f odom=%.3f src=%s) world=(%.2f,%.2f) -> body=(%.2f,%.2f)",
                              global_frame_.c_str(), yaw_, (int)have_yaw_topic_, yaw_tf_, yaw_odom_, yaw_source_.c_str(), world_vx, world_vy, body_vx, body_vy);
        }
        else // chassis 模式：按键直接定义底盘系速度
        {
            body_vx = cur_vx_world; // 在 chassis 模式下，W/S 映射到底盘 x，A/D 映射到底盘 y
            body_vy = cur_vy_world;
            // 为了记录/上层需要，如需世界系可在此再做 R(yaw) 变换得到 world_vx/world_vy；此处直接发布 base_link 帧
            stamped.twist.linear.x = body_vx;
            stamped.twist.linear.y = body_vy;
            ROS_INFO_THROTTLE(1.0, "mode=chassis body=(%.2f,%.2f) yaw=%.3f (topic=%d tf=%.3f odom=%.3f src=%s)", body_vx, body_vy, yaw_, (int)have_yaw_topic_, yaw_tf_, yaw_odom_, yaw_source_.c_str());
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
