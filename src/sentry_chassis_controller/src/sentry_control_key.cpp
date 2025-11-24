// Improved keyboard teleop based on learning notes.
// Supports WASD for x/y strafing, Q/E for rotation, Shift+WASD for run, 'c' to stop.

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

        // default stop for this key iteration
        double lin = 0.0, lat = 0.0, ang = 0.0;
        double lin_speed = walk_vel;
        double ang_speed = yaw_rate;

        // handle uppercase (Shift) -> boost speeds
        bool shifted = false;
        if (c >= 'A' && c <= 'Z')
            shifted = true;

        // normalize to lowercase for mapping
        char lower = c;
        if (shifted)
            lower = c - 'A' + 'a';

        if (shifted)
        {
            lin_speed = run_vel;
            ang_speed = yaw_rate_run;
        }

        switch (lower)
        {
        case 'w':
            lin = 1.0;
            break; // forward
        case 's':
            lin = -1.0;
            break; // back
        case 'a':
            lat = 1.0;
            break; // left strafe
        case 'd':
            lat = -1.0;
            break; // right strafe
        case 'q':
            ang = 1.0;
            break; // turn left
        case 'e':
            ang = -1.0;
            break; // turn right
        case 'c':
            lin = 0.0;
            lat = 0.0;
            ang = 0.0;
            break;   // stop
        case '\x03': // ctrl-c
            quit(0);
            break;
        default:
            // ignore other keys
            break;
        }

        twist.linear.x = lin * lin_speed;
        twist.linear.y = lat * lin_speed;
        twist.angular.z = ang * ang_speed;

        twist_pub_.publish(twist);
        rate.sleep();
    }

    term_guard.restore();
    return;
}
