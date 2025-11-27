/* ROS 相关头文件 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
/* C++ 相关头文件 */
#include <cmath>
#include <string>
#include <cstring>
#include <stdio.h>
#include <signal.h>
/* 终端控制相关头文件 */
#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#endif

/* termios 结构体 */
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

/*
 * 键盘控制节点
 * 功能： 当按下键盘的时候，底盘能够接收到世界坐标系下的速度指令，并转换为机体坐标系下的速度指令发布到 /cmd_vel 话题
 * 1. 终端监听键盘的输入，然后发布世界坐标系速度指令
 * 2. 监听 /odom 话题，获取底盘在世界坐标系下的航向偏角 yaw
 * 3. 根据 yaw 将世界坐标系速度转换为机体坐标系速度，并发布到 /cmd_vel 话题
 */
class KeyboardControlNode
{
public:
    KeyboardControlNode();
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);

private:
    /* 变量定义 */
    ros::NodeHandle nh_;
    ros::NodeHandle nhPrivate_ = ros::NodeHandle("~");

    // 速度指令发布方，用于发布底盘在世界坐标系下的速度
    ros::Publisher twist_pub_;

    // 速度指令话题名称
    std::string cmd_vel_topic_;

    // 使用一个buffer对象来寻找tf变换的对象来实现底盘到odom坐标系的转换
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;

    // 空闲状态下是否发布零速度
    bool publish_zero_when_idle_ = false;
    // 底盘的航向偏角
    double yaw_ = 0.0;
    bool field_centric_ = true;     // 当为 true 时，键盘输入表示世界坐标系下的方向，旋转不影响平移方向
    bool spinning_top_mode_ = true; // 小陀螺模式开关

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        
    }
};

TermiosGuard term_guard;

KeyboardControlNode::KeyboardControlNode()
{
    nhPrivate_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    nhPrivate_.param("publish_zero_when_idle", publish_zero_when_idle_, publish_zero_when_idle_);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    // 监听baselink -> odom 变换
    tfBuffer_.reset(new tf2_ros::Buffer(ros::Duration(10)));
    tfListener_.reset(new tf2_ros::TransformListener(*tfBuffer_));
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sentry_keyboard_controller");
    KeyboardControlNode node;
    ros::spin();
    return 0;
}