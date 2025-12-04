#pragma once

#include <ros/ros.h>

namespace sentry_chassis_controller
{

    /**
     * @brief 一阶低通滤波器（指数加权移动平均）
     *
     * 离散形式：y[k] = alpha * x[k] + (1-alpha) * y[k-1]
     * 其中：alpha = dt / (tau + dt)
     *       tau 为时间常数（秒），决定滤波强度
     *       dt 为采样时间间隔（秒）
     *
     * 特性：
     * - tau 越大，滤波越强，响应越慢
     * - tau 越小，滤波越弱，响应越快
     * - tau = 0 时禁用滤波，直通输入
     */
    class LowPassFilter
    {
    public:
        /**
         * @brief 构造函数
         * @param tau 时间常数（秒），默认 0.02s（对应约 50Hz 截止频率）
         */
        explicit LowPassFilter(double tau = 0.02);

        /**
         * @brief 析构函数
         */
        ~LowPassFilter();

        /**
         * @brief 更新滤波器（处理一个新样本）
         *
         * @param input 输入值（原始信号）
         * @param dt 时间步长（秒）
         * @return 滤波后的输出值
         */
        double update(double input, double dt);

        /**
         * @brief 设置时间常数
         * @param tau 新的时间常数（秒）
         */
        void set_tau(double tau);

        /**
         * @brief 获取当前时间常数
         * @return 时间常数（秒）
         */
        double get_tau() const;

        /**
         * @brief 重置滤波器状态
         * @param initial_value 初始值（默认为0）
         */
        void reset(double initial_value = 0.0);

        /**
         * @brief 获取当前滤波器输出（不更新）
         * @return 当前输出值
         */
        double get_output() const;

    private:
        double tau_;    // 时间常数（秒）
        double output_; // 滤波器内部状态（上一次输出）
    };

    /**
     * @brief 多通道低通滤波器（用于批量处理）
     *
     * 适用于需要对多个信号同时滤波的场景（如四个轮子的力矩命令）
     */
    class MultiChannelLowPassFilter
    {
    public:
        /**
         * @brief 构造函数
         * @param num_channels 通道数量
         * @param tau 所有通道的默认时间常数（秒）
         */
        explicit MultiChannelLowPassFilter(int num_channels, double tau = 0.02);

        /**
         * @brief 析构函数
         */
        ~MultiChannelLowPassFilter();

        /**
         * @brief 更新所有通道（批量处理）
         *
         * @param inputs 输入数组（长度必须等于 num_channels）
         * @param outputs 输出数组（长度必须等于 num_channels）
         * @param dt 时间步长（秒）
         */
        void update(const double *inputs, double *outputs, double dt);

        /**
         * @brief 设置单个通道的时间常数
         * @param channel 通道索引（0-based）
         * @param tau 时间常数（秒）
         */
        void set_tau(int channel, double tau);

        /**
         * @brief 设置所有通道的时间常数
         * @param tau 时间常数（秒）
         */
        void set_all_tau(double tau);

        /**
         * @brief 获取通道数量
         * @return 通道数量
         */
        int get_num_channels() const;

        /**
         * @brief 重置所有通道
         * @param initial_value 初始值（默认为0）
         */
        void reset(double initial_value = 0.0);

        /**
         * @brief 重置单个通道
         * @param channel 通道索引
         * @param initial_value 初始值
         */
        void reset(int channel, double initial_value);

    private:
        int num_channels_;                 // 通道数量
        std::vector<LowPassFilter> filters_; // 各通道的滤波器
    };

} // namespace sentry_chassis_controller
