#include "sentry_chassis_controller/low_pass_filter.hpp"
#include <stdexcept>

namespace sentry_chassis_controller
{

    // ==================== LowPassFilter 实现 ====================

    LowPassFilter::LowPassFilter(double tau)
        : tau_(tau), output_(0.0)
    {
        if (tau < 0.0)
        {
            ROS_WARN("LowPassFilter: tau cannot be negative, setting to 0.0");
            tau_ = 0.0;
        }
    }

    LowPassFilter::~LowPassFilter()
    {
    }

    double LowPassFilter::update(double input, double dt)
    {
        if (dt <= 0.0)
        {
            // 无效时间步长，返回上一次输出
            return output_;
        }

        if (tau_ > 1e-6) // tau > 0 启用滤波
        {
            // 计算滤波系数 alpha = dt / (tau + dt)
            double alpha = dt / (tau_ + dt);
            // 一阶低通滤波：y[k] = alpha * x[k] + (1-alpha) * y[k-1]
            output_ = alpha * input + (1.0 - alpha) * output_;
        }
        else
        {
            // tau ≈ 0 禁用滤波，直接使用输入
            output_ = input;
        }

        return output_;
    }

    void LowPassFilter::set_tau(double tau)
    {
        if (tau < 0.0)
        {
            ROS_WARN("LowPassFilter: tau cannot be negative, ignoring");
            return;
        }
        tau_ = tau;
    }

    double LowPassFilter::get_tau() const
    {
        return tau_;
    }

    void LowPassFilter::reset(double initial_value)
    {
        output_ = initial_value;
    }

    double LowPassFilter::get_output() const
    {
        return output_;
    }

    // ==================== MultiChannelLowPassFilter 实现 ====================

    MultiChannelLowPassFilter::MultiChannelLowPassFilter(int num_channels, double tau)
        : num_channels_(num_channels)
    {
        if (num_channels <= 0)
        {
            throw std::invalid_argument("MultiChannelLowPassFilter: num_channels must be positive");
        }

        // 初始化各通道的滤波器
        filters_.reserve(num_channels);
        for (int i = 0; i < num_channels; ++i)
        {
            filters_.emplace_back(tau);
        }
    }

    MultiChannelLowPassFilter::~MultiChannelLowPassFilter()
    {
    }

    void MultiChannelLowPassFilter::update(const double *inputs, double *outputs, double dt)
    {
        if (inputs == nullptr || outputs == nullptr)
        {
            ROS_ERROR("MultiChannelLowPassFilter: null pointer passed to update()");
            return;
        }

        for (int i = 0; i < num_channels_; ++i)
        {
            outputs[i] = filters_[i].update(inputs[i], dt);
        }
    }

    void MultiChannelLowPassFilter::set_tau(int channel, double tau)
    {
        if (channel < 0 || channel >= num_channels_)
        {
            ROS_WARN("MultiChannelLowPassFilter: invalid channel %d (valid range: 0-%d)",
                     channel, num_channels_ - 1);
            return;
        }
        filters_[channel].set_tau(tau);
    }

    void MultiChannelLowPassFilter::set_all_tau(double tau)
    {
        for (auto &filter : filters_)
        {
            filter.set_tau(tau);
        }
    }

    int MultiChannelLowPassFilter::get_num_channels() const
    {
        return num_channels_;
    }

    void MultiChannelLowPassFilter::reset(double initial_value)
    {
        for (auto &filter : filters_)
        {
            filter.reset(initial_value);
        }
    }

    void MultiChannelLowPassFilter::reset(int channel, double initial_value)
    {
        if (channel < 0 || channel >= num_channels_)
        {
            ROS_WARN("MultiChannelLowPassFilter: invalid channel %d (valid range: 0-%d)",
                     channel, num_channels_ - 1);
            return;
        }
        filters_[channel].reset(initial_value);
    }

} // namespace sentry_chassis_controller
