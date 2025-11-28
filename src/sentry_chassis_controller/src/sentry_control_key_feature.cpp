/* ROS 相关头文件 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
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
    bool field_centric_ = true;     // 当为 true 时，键盘输入表示世界坐标系下的方向，旋转不影响平移方向
    bool spinning_top_mode_ = true; // 小陀螺模式默认开启
};

TermiosGuard term_guard;

TeleopTurtle::TeleopTurtle()
{
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
    puts("Reading from keyboard (WASD to move, Q/E rotate, Shift+WASD run, c stop, f toggle field-centric, Ctrl-C quit) [Spinning-top mode: DEFAULT ON]");
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
        double cur_vx_world = 0.0;    // 世界坐标系下的x轴方向的速度请求
        double cur_vy_world = 0.0;    // 世界坐标系下的y轴方向的速度请求
        double cur_omega = 0.0;       // 机体角速度请求（Q/E 直接控制角速度）
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
            // Q/E 直接作为角速度命令（按下时有效）
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
            // 切换 field-centric 模式
            field_centric_ = !field_centric_;
            ROS_INFO("Field-centric mode: %s", field_centric_ ? "ON" : "OFF");
            break;
        // 删除小陀螺模式切换键 't'，模式始终开启
        case '\x03': // ctrl-c
            quit(0);
            break;
        default:
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

        // 发布一个带时间戳的 TwistStamped，表示请求的世界坐标系速度（header.frame_id = "odom")
        geometry_msgs::TwistStamped stamped;
        stamped.header.stamp = ros::Time::now();
        stamped.header.frame_id = "odom";

        // 将本次按键请求包装为世界坐标系下的速度（仅按下按键时 non-zero）
        // 小陀螺模式默认开启：当按下WASD产生非零平移时，更新锁定的平移向量；未产生新的平移键时保持之前锁定的方向与大小
        if (spinning_top_mode_)
        {
            if (std::fabs(cur_vx_world) > 1e-9 || std::fabs(cur_vy_world) > 1e-9)
            {
                translation_magnitude = std::hypot(cur_vx_world, cur_vy_world);
                translation_angle = std::atan2(cur_vy_world, cur_vx_world);
                translation_active = (translation_magnitude > 0.0);
            }
            if (translation_active)
            {
                cur_vx_world = translation_magnitude * std::cos(translation_angle);
                cur_vy_world = translation_magnitude * std::sin(translation_angle);
            }
        }

        stamped.twist.linear.x = cur_vx_world;
        stamped.twist.linear.y = cur_vy_world;
        stamped.twist.angular.z = cur_omega;

        // 发布带时间戳的世界坐标系请求速度
        twist_stamped_pub_.publish(stamped);

        // 根据 field_centric_ 将世界速度转换到机体速度并发布到 /cmd_vel（明确使用 2x2 旋转矩阵）
        double v_world_x = stamped.twist.linear.x;
        double v_world_y = stamped.twist.linear.y;
        // 旋转矩阵 R(-yaw) 的计算（将 world -> body）
        // R(-yaw) = [ cos(yaw)  sin(yaw)
        //             -sin(yaw)  cos(yaw) ]
        if (field_centric_)
        {
            double c = cos(yaw_);
            double s = sin(yaw_);
            double vbx = c * v_world_x + s * v_world_y;
            double vby = -s * v_world_x + c * v_world_y;
            twist.linear.x = vbx;
            twist.linear.y = vby;
        }
        else
        {
            twist.linear.x = v_world_x;
            twist.linear.y = v_world_y;
        }
        // 角速度直接由 Q/E 键控制（按下时有效）
        twist.angular.z = stamped.twist.angular.z;

        // 仅当本次按键请求非零（或用户允许空闲零发布）时发送
        bool has_motion = (std::abs(twist.linear.x) > 1e-9) || (std::abs(twist.linear.y) > 1e-9) || (std::abs(twist.angular.z) > 1e-9);
        if (any_key_pressed || publish_zero_when_idle_)
        {
            // 发布带时间戳的世界意图（方便上层/记录）
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
