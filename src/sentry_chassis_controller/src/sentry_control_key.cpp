// 支持 W/A/S/D 键进行前后左右平移，Q/E 键进行左右转向，，c 键停止，Ctrl-C 退出程序
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include <string>
#include <stdexcept>
#include <cstring>
#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#endif
#include <poll.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <cmath>

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

    // 使用一个buffer对象来寻找tf变换的对象来实现底盘到odom坐标系的转换
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;

    // 底盘的航向偏角
    double yaw_ = 0.0;
    bool field_centric_ = true;      // 当为 true 时，键盘输入表示世界坐标系下的方向，旋转不影响平移方向
    bool spinning_top_mode_ = false; // 小陀螺模式开关
};

//
TermiosGuard term_guard;

TeleopTurtle::TeleopTurtle()
{
    //
    nhPrivate_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    nhPrivate_.param("cmd_vel_stamped_topic", cmd_vel_stamped_topic_, std::string("/cmd_vel_stamped"));
    nhPrivate_.param("publish_zero_when_idle", publish_zero_when_idle_, publish_zero_when_idle_);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    twist_stamped_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(cmd_vel_stamped_topic_, 1);
    // 监听baselink -> odom 变换
    tfBuffer_.reset(new tf2_ros::Buffer(ros::Duration(10)));
    tfListener_.reset(new tf2_ros::TransformListener(*tfBuffer_));
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
    puts("Reading from keyboard (WASD to move, Q/E rotate, Shift+WASD run, c stop, f toggle field-centric, t toggle spinning-top, Ctrl-C quit)");
    term_guard.setup();

    // 速度参数
    double walk_vel = 0.5;
    double run_vel = 1.0;
    double yaw_rate = 1.0;
    double yaw_rate_run = 1.5;
    int hz = 10;
    nhPrivate_.param("walk_vel", walk_vel, walk_vel);
    nhPrivate_.param("run_vel", run_vel, run_vel);
    nhPrivate_.param("yaw_rate", yaw_rate, yaw_rate);
    nhPrivate_.param("yaw_rate_run", yaw_rate_run, yaw_rate_run);
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

        // 持久化当前运动状态：按键只改变对应分量，允许平移与自转叠加
        static double cur_lin_world = 0.0; // 当 field-centric 时，保存世界坐标系下的速度
        static double cur_lat_world = 0.0;
        static double cur_ang = 0.0; // 角速度仍然按机体角速度发送

        // 小陀螺模式专用变量
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

        double lin_speed_use = shifted ? run_vel : walk_vel;
        double ang_speed_use = shifted ? yaw_rate_run : yaw_rate;

        switch (lower)
        {
        case 'w':
            if (spinning_top_mode_)
            {
                // 小陀螺模式：更新平移方向和大小
                double vx = lin_speed_use; // 前向
                double vy = 0;
                translation_magnitude = std::hypot(vx, vy);
                translation_angle = std::atan2(vy, vx);
                translation_active = (translation_magnitude > 0);
            }
            else
            {
                // 将按键解释为在世界坐标系的前向命令（cur_lin_world）
                cur_lin_world = lin_speed_use;
            }
            break; 
        case 's':
            if (spinning_top_mode_)
            {
                double vx = -lin_speed_use; // 后向
                double vy = 0;
                translation_magnitude = std::hypot(vx, vy);
                translation_angle = std::atan2(vy, vx);
                translation_active = (translation_magnitude > 0);
            }
            else
            {
                cur_lin_world = -lin_speed_use;
            }
            break; 
        case 'a':
            if (spinning_top_mode_)
            {
                double vx = 0;
                double vy = lin_speed_use;
                translation_magnitude = std::hypot(vx, vy);
                translation_angle = std::atan2(vy, vx);
                translation_active = (translation_magnitude > 0);
            }
            else
            {
                cur_lat_world = lin_speed_use;
            }
            break; 
        case 'd':
            if (spinning_top_mode_)
            {
                double vx = 0;
                double vy = -lin_speed_use; // 右向
                translation_magnitude = std::hypot(vx, vy);
                translation_angle = std::atan2(vy, vx);
                translation_active = (translation_magnitude > 0);
            }
            else
            {
                cur_lat_world = -lin_speed_use;
            }
            break; // right strafe
        case 'q':
            cur_ang = ang_speed_use;
            break; // turn left
        case 'e':
            cur_ang = -ang_speed_use;
            break; // turn right
        case 'c':
            // stop all motion
            if (spinning_top_mode_)
            {
                // 小陀螺模式：只停止平移，保持自转
                translation_magnitude = 0.0;
                translation_angle = 0.0;
                translation_active = false;
            }
            else
            {
                // 普通模式：停止所有运动
                cur_lin_world = 0.0;
                cur_lat_world = 0.0;
            }
            cur_ang = 0.0;
            break; // stop
        case 'f':
            // 切换 field-centric 模式
            field_centric_ = !field_centric_;
            ROS_INFO("Field-centric mode: %s", field_centric_ ? "ON" : "OFF");
            break;
        case 't':
            // 切换小陀螺模式
            spinning_top_mode_ = !spinning_top_mode_;
            if (spinning_top_mode_)
            {
                // 进入小陀螺模式：转换当前速度到极坐标
                double cur_vx = cur_lin_world;
                double cur_vy = cur_lat_world;
                translation_magnitude = std::hypot(cur_vx, cur_vy);
                translation_angle = std::atan2(cur_vy, cur_vx);
                translation_active = (translation_magnitude > 0);
                ROS_INFO("Spinning-top mode: ON (translation locked at %.2f m/s, %.2f deg)",
                         translation_magnitude, translation_angle * 180.0 / M_PI);
            }
            else
            {
                // 退出小陀螺模式：转换极坐标回笛卡尔坐标
                cur_lin_world = translation_magnitude * std::cos(translation_angle);
                cur_lat_world = translation_magnitude * std::sin(translation_angle);
                translation_active = false;
                ROS_INFO("Spinning-top mode: OFF");
            }
            break;
        case '\x03': // ctrl-c
            quit(0);
            break;
        default:
            // ignore other keys
            break;
        }

        // 更新底盘航向角 yaw_
        if (tfBuffer_)
        {
            try
            {
                // 尝试通过 TF 查询 odom -> base_link 变换以获取航向角
                geometry_msgs::TransformStamped tfst = tfBuffer_->lookupTransform("odom", "base_link", ros::Time(0), ros::Duration(0.05));
                const auto &q = tfst.transform.rotation;
                double qw = q.w;
                double qx = q.x;
                double qy = q.y;
                double qz = q.z;
                yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            }
            catch (const tf2::TransformException &ex)
            {
                // 查找失败就保持之前的 yaw_ 不变
                ROS_WARN_THROTTLE(5, "TF lookup odom->base_link failed: %s", ex.what());
            }
        }

        // 发布一个带时间戳的 Twist，表示请求的世界坐标系速度（header.frame_id = "odom"）
        geometry_msgs::TwistStamped stamped;
        stamped.header.stamp = ros::Time::now();
        stamped.header.frame_id = "odom";

        // 如果处于小陀螺模式，则计算带时间戳消息的世界坐标系请求速度
        if (spinning_top_mode_)
        {
            if (translation_active)
            {
                // 解算：平移速度左乘旋转矩阵
                double v_world_x = translation_magnitude * std::cos(translation_angle);
                double v_world_y = translation_magnitude * std::sin(translation_angle);
                stamped.twist.linear.x = v_world_x;
                stamped.twist.linear.y = v_world_y;
            }
            else
            {
                stamped.twist.linear.x = 0.0;
                stamped.twist.linear.y = 0.0;
            }
            stamped.twist.angular.z = cur_ang; // 请求的航向角速度
        }
        else
        {
            stamped.twist.linear.x = cur_lin_world;
            stamped.twist.linear.y = cur_lat_world;
            stamped.twist.angular.z = cur_ang;
        }

        // 发布带时间戳的世界坐标系请求速度
        twist_stamped_pub_.publish(stamped);

        // 根据 field_centric_ 将世界速度转换到机体速度并发布到 /cmd_vel（保持与控制器兼容）
        if (spinning_top_mode_)
        {
            if (translation_active)
            {
                double v_world_x = stamped.twist.linear.x;
                double v_world_y = stamped.twist.linear.y;
                if (field_centric_)
                {
                    double vbx = cos(yaw_) * v_world_x + sin(yaw_) * v_world_y;
                    double vby = -sin(yaw_) * v_world_x + cos(yaw_) * v_world_y;
                    twist.linear.x = vbx;
                    twist.linear.y = vby;
                }
                else
                {
                    twist.linear.x = v_world_x;
                    twist.linear.y = v_world_y;
                }
            }
            else
            {
                twist.linear.x = 0;
                twist.linear.y = 0;
            }
            twist.angular.z = cur_ang;
        }
        else
        {
            if (field_centric_)
            {
                double vbx = cos(yaw_) * cur_lin_world + sin(yaw_) * cur_lat_world;
                double vby = -sin(yaw_) * cur_lin_world + cos(yaw_) * cur_lat_world;
                twist.linear.x = vbx;
                twist.linear.y = vby;
            }
            else
            {
                twist.linear.x = cur_lin_world;
                twist.linear.y = cur_lat_world;
            }
            twist.angular.z = cur_ang;
        }

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
    }

    term_guard.restore();
    return;
}
