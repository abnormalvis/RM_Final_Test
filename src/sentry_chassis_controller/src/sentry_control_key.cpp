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

private:
    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle nhPrivate_{"~"};

    // 分离发布者：
    // 1) 线速度（odom坐标系，带时间戳）
    ros::Publisher linear_pub_;
    // 2) 角速度（直接使用）
    ros::Publisher angular_pub_;
    // 3) 合并发布（传统 /cmd_vel 接口）
    ros::Publisher merged_pub_;

    // 参数
    std::string linear_topic_ = "/cmd_vel_linear_absolute";
    std::string angular_topic_ = "/cmd_vel_angular_direct";
    std::string frame_id_ = "odom"; // 线速度TwistStamped的坐标系
    std::string merged_topic_ = "/cmd_vel"; // 合并发布的底盘速度（无需外部合成）
    bool publish_zero_when_idle_ = false;
    int publish_rate_hz_ = 20; // 发布频率
    double release_timeout_ = 0.25; // 松手后保持的持续时间(s)

    // 速度标定
    double linear_speed_ = 0.5;       // 基础线速度 m/s
    double linear_speed_run_ = 1.0;   // 按住Shift的线速度 m/s
    double angular_speed_ = 1.0;      // 基础角速度 rad/s
    double angular_speed_run_ = 1.5;  // 按住Shift的角速度 rad/s

    // 按键保持状态（按下才运动 + 释放后短暂保持）
    double linear_x_state_ = 0.0; // odom系前向
    double linear_y_state_ = 0.0; // odom系侧向
    double angular_z_state_ = 0.0; // 角速度
    ros::Time last_linear_stamp_;
    ros::Time last_angular_stamp_;
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
    nhPrivate_.param("linear_speed", linear_speed_, linear_speed_);
    nhPrivate_.param("linear_speed_run", linear_speed_run_, linear_speed_run_);
    nhPrivate_.param("angular_speed", angular_speed_, angular_speed_);
    nhPrivate_.param("angular_speed_run", angular_speed_run_, angular_speed_run_);

    linear_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(linear_topic_, 10);
    angular_pub_ = nh_.advertise<geometry_msgs::Twist>(angular_topic_, 10);
    merged_pub_ = nh_.advertise<geometry_msgs::Twist>(merged_topic_, 10);

    ROS_INFO("Keyboard teleop (single-node) initialized: linear_topic=%s, angular_topic=%s, merged_topic=%s, frame_id=%s, rate=%d Hz, release_timeout=%.2fs",
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

    ros::Rate rate(publish_rate_hz_);
    while (ros::ok())
    {
        int num = poll(&ufd, 1, 1000 / publish_rate_hz_);
        if (num < 0)
        {
            perror("poll():");
            break;
        }

        if (num > 0)
        {
            char c = 0;
            if (read(0, &c, 1) < 0)
            {
                perror("read():");
                break;
            }

            // Shift检测（大写表示按下了Shift）
            bool shifted = (c >= 'A' && c <= 'Z');
            char lower = shifted ? static_cast<char>(c - 'A' + 'a') : c;
            double lin = shifted ? linear_speed_run_ : linear_speed_;
            double ang = shifted ? angular_speed_run_ : angular_speed_;

            switch (lower)
            {
            case 'w':
                linear_x_state_ = lin;
                linear_y_state_ = 0.0;
                last_linear_stamp_ = ros::Time::now();
                break;
            case 's':
                linear_x_state_ = -lin;
                linear_y_state_ = 0.0;
                last_linear_stamp_ = ros::Time::now();
                break;
            case 'a':
                linear_x_state_ = 0.0;
                linear_y_state_ = lin;
                last_linear_stamp_ = ros::Time::now();
                break;
            case 'd':
                linear_x_state_ = 0.0;
                linear_y_state_ = -lin;
                last_linear_stamp_ = ros::Time::now();
                break;
            case 'q':
                angular_z_state_ = ang;
                last_angular_stamp_ = ros::Time::now();
                break;
            case 'e':
                angular_z_state_ = -ang;
                last_angular_stamp_ = ros::Time::now();
                break;
            case 'c':
            case ' ': // space
                linear_x_state_ = 0.0;
                linear_y_state_ = 0.0;
                angular_z_state_ = 0.0;
                last_linear_stamp_ = ros::Time::now();
                last_angular_stamp_ = ros::Time::now();
                break;
            case '\x03': // ctrl-c
                quit(0);
                break;
            default:
                break;
            }
        }

        // 基于释放超时的按住逻辑
        const ros::Time now = ros::Time::now();
        bool linear_active = (now - last_linear_stamp_).toSec() < release_timeout_;
        bool angular_active = (now - last_angular_stamp_).toSec() < release_timeout_;

        double vx_world = linear_active ? linear_x_state_ : 0.0;
        double vy_world = linear_active ? linear_y_state_ : 0.0;
        double wz = angular_active ? angular_z_state_ : 0.0;

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
            // 分离发布
            linear_pub_.publish(lin_msg);
            angular_pub_.publish(ang_msg);

            // 合并发布到 /cmd_vel （这里直接使用世界系的 vx, vy，不做坐标变换；如果需要机体系可后续加 TF）
            geometry_msgs::Twist merged;
            merged.linear.x = vx_world;
            merged.linear.y = vy_world;
            merged.linear.z = 0.0;
            merged.angular.x = 0.0;
            merged.angular.y = 0.0;
            merged.angular.z = wz;
            merged_pub_.publish(merged);
        }

        rate.sleep();
    }

    term_guard.restore();
}
