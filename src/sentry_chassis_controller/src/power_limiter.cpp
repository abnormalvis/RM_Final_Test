#include "sentry_chassis_controller/power_limiter.hpp"
#include <cmath>

namespace sentry_chassis_controller
{

    void apply_power_limit(const PowerLimiterConfig &config,
                           const PowerLimiterInput &input,
                           PowerLimiterOutput &output,
                           ros::Publisher *debug_pub)
    {
        // 初始化输出为输入值（默认不缩放）
        for (int i = 0; i < 4; ++i)
        {
            output.cmd[i] = input.cmd[i];
        }
        output.scaling_factor = 1.0;
        output.limited = false;

        // 如果功率限制未启用，直接返回
        if (!config.enabled)
        {
            return;
        }

        // 平方函数
        auto sq = [](double x)
        { return x * x; };

        // 构建二次方程系数：a*s² + b*s + c ≤ 0
        // 其中：
        //   a = effort_coeff * Σ(cmd²)
        //   b = Σ|cmd * vel|
        //   c = velocity_coeff * Σ(vel²) - power_offset - power_limit

        double a = sq(input.cmd[0]) + sq(input.cmd[1]) +
                   sq(input.cmd[2]) + sq(input.cmd[3]);
        a *= config.effort_coeff;

        double b = std::abs(input.cmd[0] * input.vel[0]) +
                   std::abs(input.cmd[1] * input.vel[1]) +
                   std::abs(input.cmd[2] * input.vel[2]) +
                   std::abs(input.cmd[3] * input.vel[3]);

        double c = sq(input.vel[0]) + sq(input.vel[1]) +
                   sq(input.vel[2]) + sq(input.vel[3]);
        c = c * config.velocity_coeff - config.power_offset - config.power_limit;

        // 计算判别式
        double disc = sq(b) - 4.0 * a * c;

        // 保存调试信息
        output.coeff_a = a;
        output.coeff_b = b;
        output.coeff_c = c;
        output.discriminant = disc;

        // 求解缩放因子
        // 使用公式: s = (-b + sqrt(b² - 4ac)) / (2a)
        // 仅当判别式 > 0 且 a ≠ 0 时有效解
        double scaling_factor = 1.0;
        if (disc > 0.0 && a != 0.0)
        {
            scaling_factor = (-b + std::sqrt(disc)) / (2.0 * a);
        }

        output.scaling_factor = scaling_factor;

        // 如果缩放因子 < 1.0，则需要限制功率
        if (scaling_factor < 1.0)
        {
            output.limited = true;
            for (int i = 0; i < 4; ++i)
            {
                output.cmd[i] = input.cmd[i] * scaling_factor;
            }

            ROS_INFO_THROTTLE(0.5,
                              "Power limit triggered: scaling=%.3f (a=%.3f, b=%.3f, c=%.3f)",
                              scaling_factor, a, b, c);
        }

        // 发布调试信息（如果启用且提供了发布器）
        if (config.debug_enabled && debug_pub && debug_pub->getNumSubscribers() > 0)
        {
            std_msgs::Float64MultiArray dbg;
            dbg.data.resize(5);
            dbg.data[0] = a;
            dbg.data[1] = b;
            dbg.data[2] = c;
            dbg.data[3] = disc;
            dbg.data[4] = scaling_factor;
            debug_pub->publish(dbg);
        }
    }

} // namespace sentry_chassis_controller
