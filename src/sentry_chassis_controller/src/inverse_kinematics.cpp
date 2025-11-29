#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <array>
#include <sentry_chassis_controller/inverse_kinematics.hpp>
#include <cmath>
#include <iostream>

namespace sentry_kinematics
{
    // 输入: 机器人底盘线速度 vx, vy (m/s) 和角速度 wz (rad/s)
    // 输出给到每个轮子期望的舵角和轮速
    // 顺序: 0 = front-left (FL), 1 = front-right (FR), 2 = rear-left (RL), 3 = rear-right (RR)
    IKResult inverseKinematics(double vx, double vy, double wz,
                               double wheel_base, double wheel_track, double wheel_radius)
    {
        IKResult res;
        // 轮子接触点相对于 base_link 的位置 (x 向前, y 向左)
        // 前轮的 x = +wheel_base/2, 后轮的 x = -wheel_base/2
        // 左轮的 y = +wheel_track/2, 右轮的 y = -wheel_track/2
        const std::array<std::pair<double, double>, 4> wheel_pos = {{{wheel_base / 2.0, wheel_track / 2.0}, // FL

                                                                     {wheel_base / 2.0, -wheel_track / 2.0}, // FR

                                                                     {-wheel_base / 2.0, wheel_track / 2.0}, // RL

                                                                     {-wheel_base / 2.0, -wheel_track / 2.0}}}; // RR
        for (size_t i = 0; i < 4; ++i)
        {
            double rx = wheel_pos[i].first;
            double ry = wheel_pos[i].second;

            // 线速度等于底盘线速度加上角速度引起的附加速度
            // omega x r = [-wz * y, wz * x]
            double vx_w = vx - wz * ry;
            double vy_w = vy + wz * rx;

            // 期望的舵角使轮子前进方向与速度向量对齐
            double angle = std::atan2(vy_w, vx_w); // returns value in [-pi, pi]

            // 轮子沿前进方向的线速度
            double v_along = std::hypot(vx_w, vy_w); // magnitude

            // 如果速度接近于零，则保持轮速为零，但仍提供舵角
            const double eps = 1e-6;
            double wheel_omega = 0.0;

            if (v_along > eps)
            {
                // 轮子的角速度 = 线速度 / 轮子半径
                wheel_omega = v_along / wheel_radius;
            }

            // 输出给到每个轮子期望的舵角和轮速
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
    std::cout << "逆运动学 结果:\n";
    const char *names[4] = {"前左(FL)", "前右(FR)", "后左(RL)", "后右(RR)"};
    for (int i = 0; i < 4; ++i)
    {
        std::cout << names[i] << ": 舵角(rad)=" << res.steer_angle[i]
                  << ", 轮子角速度(rad/s)=" << res.wheel_angular_vel[i]
                  << ", 轮子线速度(m/s)=" << res.wheel_linear_vel[i] << "\n";
    }
    return 0;
}