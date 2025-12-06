#include "sentry_chassis_controller/geo_lock.hpp"
#include <ros/console.h>
#include <cmath>
#include <algorithm>

namespace sentry_chassis_controller
{

GeoLock::GeoLock(const GeoLockConfig& config)
    : config_(config)
{
    compute_lock_angles();
    
    ROS_INFO("GeoLock initialized: enabled=%s, idle_timeout=%.2fs, wheel_brake=%s",
             config_.enabled ? "true" : "false",
             config_.idle_timeout,
             config_.wheel_brake ? "true" : "false");
    
    ROS_INFO("GeoLock angles: FL=%.2f° FR=%.2f° RL=%.2f° RR=%.2f°",
             lock_angles_[0] * 180.0 / M_PI,
             lock_angles_[1] * 180.0 / M_PI,
             lock_angles_[2] * 180.0 / M_PI,
             lock_angles_[3] * 180.0 / M_PI);
}

void GeoLock::compute_lock_angles()
{
    // 预计算几何自锁舵角（X 字布局）
    // 轮子相对底盘中心的位置
    double rx = config_.wheel_base / 2.0;   // 轴距半值
    double ry = config_.wheel_track / 2.0;  // 轮距半值
    
    // 切向方向：使轮子滚动方向沿底盘自转切线
    // 公式：θ = atan2(rx, -ry)
    lock_angles_[0] = std::atan2(rx, -ry);   // FL: (+rx, +ry) → atan2(+rx, -ry) = -45°
    lock_angles_[1] = std::atan2(rx, ry);    // FR: (+rx, -ry) → atan2(+rx, +ry) = +45°
    lock_angles_[2] = std::atan2(-rx, -ry);  // RL: (-rx, +ry) → atan2(-rx, -ry) = -135°
    lock_angles_[3] = std::atan2(-rx, ry);   // RR: (-rx, -ry) → atan2(-rx, +ry) = +135°
}

void GeoLock::update(const GeoLockInput& input, GeoLockOutput& output)
{
    // 如果功能未启用，直接返回未锁定状态
    if (!config_.enabled)
    {
        output.is_locked = false;
        is_locked_ = false;
        return;
    }
    
    // 计算空闲时长
    double idle_duration = (input.current_time - input.last_cmd_time).toSec();
    
    // 检查是否超时
    if (idle_duration > config_.idle_timeout)
    {
        // 进入或保持自锁状态
        if (!is_locked_)
        {
            // 首次进入自锁：记录当前轮子位置
            for (int i = 0; i < 4; ++i)
            {
                locked_wheel_pos_[i] = input.wheel_positions[i];
            }
            is_locked_ = true;
        }
        
        // 设置输出状态
        output.is_locked = true;
        
        // 设置舵角命令为几何自锁角度
        for (int i = 0; i < 4; ++i)
        {
            output.pivot_commands[i] = lock_angles_[i];
        }
        
        // 根据配置选择轮子控制模式
        if (config_.wheel_brake)
        {
            // 轮子位置锁定）
            apply_wheel_brake(input, output);
        }
        else
        {
            // 轮子可以滚动
            output.use_position_control = false;
            for (int i = 0; i < 4; ++i)
            {
                output.wheel_commands[i] = 0.0;
            }
        }
    }
    else
    {
        // 未超时，解除自锁
        if (is_locked_)
        {
            is_locked_ = false;
            ROS_INFO_THROTTLE(2.0, "GeoLock released");
        }
        output.is_locked = false;
    }
}

void GeoLock::apply_wheel_brake(const GeoLockInput& input, GeoLockOutput& output)
{
    // 使用位置 PD 控制器计算锁定力矩
    output.use_position_control = true;
    
    for (int i = 0; i < 4; ++i)
    {
        // 位置误差
        double pos_error = locked_wheel_pos_[i] - input.wheel_positions[i];
        
        // PD 控制：torque = Kp * pos_error - Kd * velocity
        double lock_torque = config_.lock_pos_p * pos_error - config_.lock_pos_d * input.wheel_velocities[i];
        
        // 力矩限幅
        lock_torque = std::max(-config_.max_lock_torque, std::min(config_.max_lock_torque, lock_torque));
        
        // 存储力矩命令（乘以1000作为标记，表示这是位置控制模式）
        output.wheel_commands[i] = lock_torque * 1000.0;
    }
}

void GeoLock::reset()
{
    is_locked_ = false;
}

void GeoLock::set_config(const GeoLockConfig& config)
{
    bool geometry_changed = (config.wheel_base != config_.wheel_base) || 
                           (config.wheel_track != config_.wheel_track);
    
    config_ = config;
    
    // 如果底盘尺寸改变，重新计算锁定角度
    if (geometry_changed)
    {
        compute_lock_angles();
        ROS_INFO("GeoLock angles updated: FL=%.2f° FR=%.2f° RL=%.2f° RR=%.2f°",
                 lock_angles_[0] * 180.0 / M_PI,
                 lock_angles_[1] * 180.0 / M_PI,
                 lock_angles_[2] * 180.0 / M_PI,
                 lock_angles_[3] * 180.0 / M_PI);
    }
}

void GeoLock::get_lock_angles(double angles[4]) const
{
    for (int i = 0; i < 4; ++i)
    {
        angles[i] = lock_angles_[i];
    }
}

} // namespace sentry_chassis_controller
