#pragma once
#include <ros/ros.h>

namespace sentry_chassis_controller
{

    /**
     * @brief 一阶低通滤波器（指数加权移动平均）
     * 离散形式：y[k] = alpha * x[k] + (1-alpha) * y[k-1]
     * alpha = dt / (tau + dt)
     * tau 为时间常数（秒），决定滤波强度
     * dt 为采样时间间隔（秒）
     * tau 越大，滤波越强，响应越慢，tau 越小，滤波越弱，响应越快，tau = 0 时禁用滤波，直通输入
     */
    class LowPassFilter
    {
    public:
        explicit LowPassFilter(double tau = 0.02);
        ~LowPassFilter();

        double update(double input, double dt);
        void set_tau(double tau);
        double get_tau() const;
        void reset(double initial_value = 0.0);
        double get_output() const;
    private:
        double tau_;    // 时间常数（秒）
        double output_; // 滤波器内部状态（上一次输出）
    };

    /*多通道低通滤波器 适用于需要对多个信号同时滤波的场景（如四个轮子的力矩命令）*/
    class MultiChannelLowPassFilter
    {
    public:
        explicit MultiChannelLowPassFilter(int num_channels, double tau = 0.02);
        ~MultiChannelLowPassFilter();
        void update(const double *inputs, double *outputs, double dt);
        void set_tau(int channel, double tau);
        void set_all_tau(double tau);
        int get_num_channels() const;
        void reset(double initial_value = 0.0);
        void reset(int channel, double initial_value);

    private:
        int num_channels_;                 // 通道数量
        std::vector<LowPassFilter> filters_; // 各通道的滤波器
    };

} // namespace sentry_chassis_controller
