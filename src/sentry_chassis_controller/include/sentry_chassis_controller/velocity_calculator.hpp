/*
 * VelocityCalculator - 速度计算器
 * 功能：将状态机输出转换为最终的速度命令
 * 支持：速度归一化、坐标变换、速度限制
 */

#ifndef SENTRY_CHASSIS_CONTROLLER_VELOCITY_CALCULATOR_HPP
#define SENTRY_CHASSIS_CONTROLLER_VELOCITY_CALCULATOR_HPP

#include <string>

namespace sentry_chassis_controller
{

    /*
     * 速度计算配置
     */
    struct VelocityCalcConfig
    {
        std::string velocity_mode; // "global" 或 "chassis"
        double max_linear_vel;     // 最大线速度 (m/s)
        double max_angular_vel;    // 最大角速度 (rad/s)

        VelocityCalcConfig()
            : velocity_mode("global"), max_linear_vel(1.0), max_angular_vel(2.0)
        {
        }
    };

    /*
     * 速度计算输入
     */
    struct VelocityCalcInput
    {
        double vx;    // x 方向速度（来自状态机）
        double vy;    // y 方向速度
        double omega; // 角速度
        double yaw;   // 当前 yaw 角（用于坐标变换，仅 global 模式需要）

        VelocityCalcInput()
            : vx(0.0), vy(0.0), omega(0.0), yaw(0.0)
        {
        }
    };

    /*
     * 速度计算输出
     */
    struct VelocityCalcOutput
    {
        double vx;    // 最终 x 速度
        double vy;    // 最终 y 速度
        double omega; // 最终角速度

        VelocityCalcOutput()
            : vx(0.0), vy(0.0), omega(0.0)
        {
        }

        void reset()
        {
            vx = vy = omega = 0.0;
        }
    };

    /*
     * 速度计算器类
     */
    class VelocityCalculator
    {
    public:
        explicit VelocityCalculator(const VelocityCalcConfig &config);

        /*
         * 计算速度命令
         * @param input 输入（来自状态机的速度 + 当前姿态）
         * @return 输出（最终速度命令）
         */
        VelocityCalcOutput compute(const VelocityCalcInput &input);

        /*
         * 更新配置
         */
        void set_config(const VelocityCalcConfig &config) { config_ = config; }
        const VelocityCalcConfig &get_config() const { return config_; }

    private:
        VelocityCalcConfig config_;

        // 限制速度幅值
        void limit_velocity(double &vx, double &vy, double max_vel);
    };

} // namespace sentry_chassis_controller

#endif // SENTRY_CHASSIS_CONTROLLER_VELOCITY_CALCULATOR_HPP
