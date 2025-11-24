#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <array>
#include <sentry_chassis_controller/inverse_kinematics.hpp>
#include <cmath>
#include <iostream>

namespace sentry_kinematics
{

    // (IKResult is declared in the header)

    // Compute inverse kinematics for a four-wheel steerable chassis.

    // Inputs:

    //  vx, vy: linear velocity of the chassis in the robot (base_link) frame (m/s)

    //  wz: angular velocity around z (rad/s), positive CCW

    //  wheel_base: distance between front and rear wheel centers (m)

    //  wheel_track: distance between left and right wheels (m)

    //  wheel_radius: radius of the driving wheel (m)

    // Output: IKResult containing desired steer angles and wheel angular velocities (rad/s)

    // Wheel order: 0 = front-left (FL), 1 = front-right (FR), 2 = rear-left (RL), 3 = rear-right (RR)

    IKResult inverseKinematics(double vx, double vy, double wz,

                               double wheel_base, double wheel_track, double wheel_radius)

    {

        IKResult res;

        // positions of wheel contact points relative to base_link (x forward, y left)

        // front wheels have x = +wheel_base/2, rear wheels x = -wheel_base/2

        // left wheels have y = +wheel_track/2, right wheels have y = -wheel_track/2

        const std::array<std::pair<double, double>, 4> wheel_pos = {{{wheel_base / 2.0, wheel_track / 2.0}, // FL

                                                                     {wheel_base / 2.0, -wheel_track / 2.0}, // FR

                                                                     {-wheel_base / 2.0, wheel_track / 2.0}, // RL

                                                                     {-wheel_base / 2.0, -wheel_track / 2.0}}}; // RR

        for (size_t i = 0; i < 4; ++i)
        {

            double rx = wheel_pos[i].first;

            double ry = wheel_pos[i].second;

            // velocity at wheel contact point: v + omega x r

            // omega x r = [-wz * y, wz * x]

            double vx_w = vx - wz * ry;

            double vy_w = vy + wz * rx;

            // desired steering angle aligns wheel forward direction with the velocity vector

            double angle = std::atan2(vy_w, vx_w); // returns value in [-pi, pi]

            // wheel linear velocity along the wheel forward direction

            double v_along = std::hypot(vx_w, vy_w); // magnitude

            // if the velocity is nearly zero, keep wheel speed zero but still provide angle

            const double eps = 1e-6;

            double wheel_omega = 0.0;

            if (v_along > eps)
            {

                // wheel angular velocity = linear velocity along wheel direction / wheel_radius

                wheel_omega = v_along / wheel_radius;
            }

            res.steer_angle[i] = angle;

            res.wheel_linear_vel[i] = v_along;

            res.wheel_angular_vel[i] = wheel_omega;
        }

        return res;
    }

} 

int main(int argc, char **argv)

{

    // Example: chassis vx=1.0 m/s forward, vy=0, wz=0.5 rad/s

    double vx = 1.0;

    double vy = 0.0;

    double wz = 0.5;

    double wheel_base = 0.36; // default from launch/config

    double wheel_track = 0.36;

    double wheel_radius = 0.05; // example

    auto res = sentry_kinematics::inverseKinematics(vx, vy, wz, wheel_base, wheel_track, wheel_radius);

    std::cout << "Inverse kinematics result:\n";

    const char *names[4] = {"FL", "FR", "RL", "RR"};

    for (int i = 0; i < 4; ++i)
    {

        std::cout << names[i] << ": steer(rad)=" << res.steer_angle[i]

                  << ", wheel_omega(rad/s)=" << res.wheel_angular_vel[i]

                  << ", wheel_v(m/s)=" << res.wheel_linear_vel[i] << "\n";
    }

    return 0;
}