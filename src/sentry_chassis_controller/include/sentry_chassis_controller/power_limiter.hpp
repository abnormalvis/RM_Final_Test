#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

namespace sentry_chassis_controller
{

    /**
     * @brief 功率限制配置参数
     *
     * 功率限制采用二次方程近似模型：
     * P ≈ a*s² + b*s + c ≤ P_limit
     * 其中 s 为力矩缩放因子，通过求解二次不等式确定最大允许缩放系数
     */
    struct PowerLimiterConfig
    {
        double power_limit{0.0};    // 功率上限 (W)
        double velocity_coeff{0.0}; // 速度平方项系数
        double effort_coeff{0.0};   // 力矩平方项系数
        double power_offset{0.0};   // 功率偏置
        bool enabled{false};        // 是否启用功率限制
        bool debug_enabled{false};  // 是否启用调试输出
    };

    /**
     * @brief 功率限制输入数据
     */
    struct PowerLimiterInput
    {
        double cmd[4]; // 四个轮子的原始力矩命令
        double vel[4]; // 四个轮子的当前速度 (rad/s)
    };

    /**
     * @brief 功率限制输出数据
     */
    struct PowerLimiterOutput
    {
        double cmd[4];              // 缩放后的力矩命令
        double scaling_factor{1.0}; // 实际使用的缩放因子
        bool limited{false};        // 是否触发了限制

        // 调试信息
        double coeff_a{0.0};      // 二次项系数
        double coeff_b{0.0};      // 一次项系数
        double coeff_c{0.0};      // 常数项
        double discriminant{0.0}; // 判别式
    };

    /**
     * @brief 应用功率限制到四个轮子的力矩命令
     *
     * 基于二次方程求解最优缩放因子，确保总功率不超过设定上限。
     * 算法原理：
     * 1. 将功率表达式转换为关于缩放因子 s 的二次不等式
     * 2. 求解二次方程得到临界缩放因子
     * 3. 如果 s < 1.0，则按比例缩放所有力矩命令
     *
     * @param config 功率限制配置参数
     * @param input 输入的力矩命令和速度反馈
     * @param output 输出的缩放后力矩和调试信息
     * @param debug_pub 可选的调试信息发布器
     */
    void apply_power_limit(const PowerLimiterConfig &config,
                           const PowerLimiterInput &input,
                           PowerLimiterOutput &output,
                           ros::Publisher *debug_pub = nullptr);

} // namespace sentry_chassis_controller
