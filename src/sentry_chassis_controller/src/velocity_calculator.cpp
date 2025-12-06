#include "sentry_chassis_controller/velocity_calculator.hpp"
#include <cmath>
#include <algorithm>

namespace sentry_chassis_controller
{

    VelocityCalculator::VelocityCalculator(const VelocityCalcConfig &config)
        : config_(config)
    {
    }

    VelocityCalcOutput VelocityCalculator::compute(const VelocityCalcInput &input)
    {
        VelocityCalcOutput output;
        double vx = input.vx;
        double vy = input.vy;
        double omega = input.omega;

        // 限制线速度
        limit_velocity(vx, vy, config_.max_linear_vel);

        // 限制角速度
        if (std::abs(omega) > config_.max_angular_vel)
        {
            omega = std::copysign(config_.max_angular_vel, omega); // 保持符号相同
        }

        output.vx = vx;
        output.vy = vy;
        output.omega = omega;

        return output;
    }

    void VelocityCalculator::limit_velocity(double &vx, double &vy, double max_vel)
    {
        double magnitude = std::sqrt(vx * vx + vy * vy);

        if (magnitude > max_vel)
        {
            double scale = max_vel / magnitude;
            vx *= scale;
            vy *= scale;
        }
    }

} // namespace sentry_chassis_controller
