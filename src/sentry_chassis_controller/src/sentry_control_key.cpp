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
#include <unordered_set>
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
        // 非阻塞立即返回，便于每周期读取所有按键
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
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
    void finalCmdCB(const geometry_msgs::Twist::ConstPtr &msg);

private:
    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle nhPrivate_{"~"};

    // 分离发布者：
    // 1) 线速度（odom坐标系，带时间戳）
    ros::Publisher linear_pub_;
    // 2) 角速度（直接使用）
    ros::Publisher angular_pub_;
    // 3) 合并发布（传统 /cmd_vel 接口）——仅用于转发解算器输出
    ros::Publisher merged_pub_;
    // 4) 订阅 /cmd_vel_final（解算器输出），用于兼容转发到 /cmd_vel
    ros::Subscriber final_sub_;

    // 参数
    std::string linear_topic_ = "/cmd_vel_linear_absolute";
    std::string angular_topic_ = "/cmd_vel_angular_direct";
    std::string frame_id_ = "odom";         // 线速度TwistStamped的坐标系
    std::string merged_topic_ = "/cmd_vel"; // 底盘最终速度话题（本节点不再主动合成，只做订阅/转发）
    bool publish_zero_when_idle_ = false;
    int publish_rate_hz_ = 20;          // 发布频率
    double release_timeout_ = 0.25;     // 松手后保持的持续时间(s)
    bool forward_cmd_vel_final_ = true; // 将 /cmd_vel_final 转发到 merged_topic (/cmd_vel)

    // 速度标定
    double linear_speed_ = 0.5;      // 基础线速度 m/s
    double linear_speed_run_ = 1.0;  // 按住Shift的线速度 m/s
    double angular_speed_ = 2.0;     // 基础角速度 rad/s
    double angular_speed_run_ = 3.0; // 按住Shift的角速度 rad/s

    // 按键保持状态（按下才运动 + 释放后短暂保持）
    double linear_x_state_ = 0.0;  // odom系前向
    double linear_y_state_ = 0.0;  // odom系侧向
    double angular_z_state_ = 0.0; // 角速度
    ros::Time last_linear_stamp_;
    ros::Time last_angular_stamp_;

    // 实时追踪按住的键集合（本周期内采集到的所有按键）
    std::unordered_set<char> keys_pressed_;
};

TermiosGuard term_guard;

TeleopTurtle::TeleopTurtle()
{
    // 可配置参数
    nhPrivate_.param("linear_topic", linear_topic_, linear_topic_);
    nhPrivate_.param("angular_topic", angular_topic_, angular_topic_);
    nhPrivate_.param("frame_id", frame_id_, frame_id_);
    nhPrivate_.param("merged_topic", merged_topic_, merged_topic_);
    nhPrivate_.param("publish_zero_when_idle", publish_zero_when_idle_, publish_zero_when_idle_);
    nhPrivate_.param("publish_rate_hz", publish_rate_hz_, publish_rate_hz_);
    nhPrivate_.param("release_timeout", release_timeout_, release_timeout_);
    nhPrivate_.param("forward_cmd_vel_final", forward_cmd_vel_final_, forward_cmd_vel_final_);
    nhPrivate_.param("linear_speed", linear_speed_, linear_speed_);
    nhPrivate_.param("linear_speed_run", linear_speed_run_, linear_speed_run_);
    nhPrivate_.param("angular_speed", angular_speed_, angular_speed_);
    nhPrivate_.param("angular_speed_run", angular_speed_run_, angular_speed_run_);

    linear_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(linear_topic_, 10);
    angular_pub_ = nh_.advertise<geometry_msgs::Twist>(angular_topic_, 10);
    merged_pub_ = nh_.advertise<geometry_msgs::Twist>(merged_topic_, 10);
    if (forward_cmd_vel_final_)
    {
        final_sub_ = nh_.subscribe("/cmd_vel_final", 10, &TeleopTurtle::finalCmdCB, this);
        ROS_INFO("Keyboard teleop forwarding enabled: /cmd_vel_final -> %s", merged_topic_.c_str());
    }

    ROS_INFO("Keyboard teleop initialized: linear_topic=%s, angular_topic=%s, merged_topic=%s, frame_id=%s, rate=%d Hz, release_timeout=%.2fs",
             linear_topic_.c_str(), angular_topic_.c_str(), merged_topic_.c_str(), frame_id_.c_str(), publish_rate_hz_, release_timeout_);
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
    puts("Keyboard: WASD move (odom), Q/E rotate, Shift for faster, C/Space stop, Ctrl-C quit");
    term_guard.setup();

    // poll 键盘
    struct pollfd ufd;
    ufd.fd = 0;          // stdin
    ufd.events = POLLIN; // 可读

    ros::Rate rate(std::max(20, publish_rate_hz_));
    while (ros::ok())
    {
        // 非阻塞poll，尽可能读取缓冲区内所有按键
        int num = poll(&ufd, 1, 0);
        if (num < 0)
        {
            perror("poll():");
            break;
        }

        if (num > 0)
        {
            // 循环读取直到缓冲区为空
            while (true)
            {
                char c = 0;
                int ret = read(0, &c, 1);
                if (ret <= 0)
                    break;

                // 转小写以识别Shift加速
                bool shifted = (c >= 'A' && c <= 'Z');
                char lower = shifted ? static_cast<char>(c - 'A' + 'a') : c;

                // 空格或c：立即清空集合并停
                if (lower == ' ' || lower == 'c')
                {
                    keys_pressed_.clear();
                    linear_x_state_ = 0.0;
                    linear_y_state_ = 0.0;
                    angular_z_state_ = 0.0;
                    last_linear_stamp_ = ros::Time::now();
                    last_angular_stamp_ = ros::Time::now();
                    continue;
                }

                // Ctrl-C 退出
                if (lower == '\x03')
                {
                    quit(0);
                }

                // 记录按键（本周期内累加处理）
                keys_pressed_.insert(c);
            }
        }

        // 基于集合计算当前期望速度（odom系vx/vy，角速度wz）
        const ros::Time now = ros::Time::now();
        double vx_world = 0.0;
        double vy_world = 0.0;
        double wz = 0.0;

        for (char c : keys_pressed_)
        {
            bool shifted = (c >= 'A' && c <= 'Z');
            char lower = shifted ? static_cast<char>(c - 'A' + 'a') : c;
            double lin = shifted ? linear_speed_run_ : linear_speed_;
            double ang = shifted ? angular_speed_run_ : angular_speed_;

            switch (lower)
            {
            case 'w':
                vx_world += lin;
                last_linear_stamp_ = now;
                break;
            case 's':
                vx_world -= lin;
                last_linear_stamp_ = now;
                break;
            case 'a':
                vy_world += lin;
                last_linear_stamp_ = now;
                break;
            case 'd':
                vy_world -= lin;
                last_linear_stamp_ = now;
                break;
            case 'q':
                wz += ang;
                last_angular_stamp_ = now;
                break;
            case 'e':
                wz -= ang;
                last_angular_stamp_ = now;
                break;
            default:
                break;
            }
        }

        // 松手检测：根据release_timeout做短暂保持（避免掉帧抖动）
        bool linear_active = (now - last_linear_stamp_).toSec() < release_timeout_;
        bool angular_active = (now - last_angular_stamp_).toSec() < release_timeout_;
        if (!linear_active)
        {
            vx_world = 0.0;
            vy_world = 0.0;
        }
        if (!angular_active)
        {
            wz = 0.0;
        }

        // 发布线速度（odom系）
        geometry_msgs::TwistStamped lin_msg;
        lin_msg.header.stamp = now;
        lin_msg.header.frame_id = frame_id_;
        lin_msg.twist.linear.x = vx_world;
        lin_msg.twist.linear.y = vy_world;
        lin_msg.twist.linear.z = 0.0;
        lin_msg.twist.angular.z = 0.0; // 仅线速度通道，角速度走独立话题

        // 发布角速度（直接）
        geometry_msgs::Twist ang_msg;
        ang_msg.linear.x = ang_msg.linear.y = ang_msg.linear.z = 0.0;
        ang_msg.angular.x = ang_msg.angular.y = 0.0;
        ang_msg.angular.z = wz;

        bool should_publish = publish_zero_when_idle_ ||
                              std::fabs(vx_world) > 1e-9 || std::fabs(vy_world) > 1e-9 || std::fabs(wz) > 1e-9;

        if (should_publish)
        {
            // 仅发布解耦后的指令：线速度(odom)与角速度(直接)
            linear_pub_.publish(lin_msg);
            angular_pub_.publish(ang_msg);
        }

        // 清空集合，下一周期重新采集（termios无法获得KeyUp事件）
        keys_pressed_.clear();

        rate.sleep();
    }

    term_guard.restore();
}

void TeleopTurtle::finalCmdCB(const geometry_msgs::Twist::ConstPtr &msg)
{
    // 将解算器输出的底盘速度直接转发到 merged_topic（通常为 /cmd_vel）
    merged_pub_.publish(*msg);
}
