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
    ros::NodeHandle nh_;
    ros::NodeHandle nhPrivate_ = ros::NodeHandle("~");
    ros::Publisher twist_pub_;
    std::string cmd_vel_topic_;
};

TermiosGuard term_guard;

TeleopTurtle::TeleopTurtle()
{
    nhPrivate_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
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
    puts("Reading from keyboard (WASD to move, Q/E rotate, Shift+WASD run, c stop, Ctrl-C quit)");
    term_guard.setup();

    // parameters for speeds
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

    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    struct pollfd ufd;
    ufd.fd = 0; // stdin
    ufd.events = POLLIN;

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
            // timeout: publish current twist (may be zero)
            twist_pub_.publish(twist);
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
        static double cur_lin = 0.0;
        static double cur_lat = 0.0;
        static double cur_ang = 0.0;

        // handle uppercase (Shift) -> boost speeds for this key press
        bool shifted = false;
        if (c >= 'A' && c <= 'Z')
            shifted = true;

        // normalize to lowercase for mapping
        char lower = c;
        if (shifted)
            lower = c - 'A' + 'a';

        double lin_speed_use = shifted ? run_vel : walk_vel;
        double ang_speed_use = shifted ? yaw_rate_run : yaw_rate;

        switch (lower)
        {
        case 'w':
            cur_lin = lin_speed_use;
            break; // forward
        case 's':
            cur_lin = -lin_speed_use;
            break; // back
        case 'a':
            cur_lat = lin_speed_use;
            break; // left strafe
        case 'd':
            cur_lat = -lin_speed_use;
            break; // right strafe
        case 'q':
            cur_ang = ang_speed_use;
            break; // turn left
        case 'e':
            cur_ang = -ang_speed_use;
            break; // turn right
        case 'c':
            // stop all motion
            cur_lin = 0.0;
            cur_lat = 0.0;
            cur_ang = 0.0;
            break;   // stop
        case '\x03': // ctrl-c
            quit(0);
            break;
        default:
            // ignore other keys
            break;
        }

        twist.linear.x = cur_lin;
        twist.linear.y = cur_lat;
        twist.angular.z = cur_ang;

        twist_pub_.publish(twist);
        rate.sleep();
    }

    term_guard.restore();
    return;
}
