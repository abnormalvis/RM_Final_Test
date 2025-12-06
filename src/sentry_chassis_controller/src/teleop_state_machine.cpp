/*
 * TeleopStateMachine - 遥操作状态机实现
 */

#include "sentry_chassis_controller/teleop_state_machine.hpp"
#include <cmath>

namespace sentry_chassis_controller
{

    TeleopStateMachine::TeleopStateMachine(const TeleopConfig &config)
        : config_(config), key_w_(false), key_s_(false), key_a_(false), key_d_(false), key_q_(false), key_e_(false), current_mode_(MotionMode::NONE), rotation_latched_(false), latched_rotation_value_(0.0)
    {
    }

    /* 复位运动状态 */
    void TeleopStateMachine::reset()
    {
        key_w_ = key_s_ = key_a_ = key_d_ = false;
        key_q_ = key_e_ = false;
        current_mode_ = MotionMode::NONE;
        rotation_latched_ = false;
        latched_rotation_value_ = 0.0;
        output_.reset();
    }

    /* 清除平移按键 */
    void TeleopStateMachine::clear_translation_keys()
    {
        key_w_ = key_s_ = key_a_ = key_d_ = false;
    }

    /* 清除旋转按键 */
    void TeleopStateMachine::clear_rotation_keys()
    {
        key_q_ = key_e_ = false;
        rotation_latched_ = false;
        latched_rotation_value_ = 0.0;
    }

    /* 处理按键输入 */
    bool TeleopStateMachine::process_key(char key, double current_time)
    {
        bool processed = false;

        switch (key)
        {
        case 'w': // 前进
            current_mode_ = MotionMode::TRANSLATION;
            clear_rotation_keys();  /* 可以选择不清除 */
            key_w_ = true;
            key_s_ = false;          // 前后互斥
            key_a_ = key_d_ = false; // 禁止斜向
            processed = true;
            break;

        case 's': // 后退
            current_mode_ = MotionMode::TRANSLATION;
            clear_rotation_keys();
            key_s_ = true;
            key_w_ = false;
            key_a_ = key_d_ = false;
            processed = true;
            break;

        case 'a': // 左平移
            current_mode_ = MotionMode::TRANSLATION;
            clear_rotation_keys();
            key_a_ = true;
            key_d_ = false;
            key_w_ = key_s_ = false;
            processed = true;
            break;

        case 'd': // 右平移
            current_mode_ = MotionMode::TRANSLATION;
            clear_rotation_keys();
            key_d_ = true;
            key_a_ = false;
            key_w_ = key_s_ = false;
            processed = true;
            break;

        case 'q': // 左转（锁存）
            current_mode_ = MotionMode::ROTATION;
            clear_translation_keys();
            rotation_latched_ = true;
            latched_rotation_value_ = config_.default_omega; // 正值表示左转
            processed = true;
            break;

        case 'e': // 右转（锁存）
            current_mode_ = MotionMode::ROTATION;
            clear_translation_keys();
            rotation_latched_ = true;
            latched_rotation_value_ = -config_.default_omega; // 负值表示右转
            processed = true;
            break;

        case 'c': // 立即停止
            clear_translation_keys();
            clear_rotation_keys();
            current_mode_ = MotionMode::NONE;
            processed = true;
            break;

        case 'u': // 增加平移速度
            adjust_walk_vel(0.1);
            processed = true;
            break;

        case 'i': // 减小平移速度
            adjust_walk_vel(-0.1);
            processed = true;
            break;

        case 'o': // 增加角速度
            adjust_default_omega(0.1);
            processed = true;
            break;

        case 'p': // 减小角速度
            adjust_default_omega(-0.1);
            processed = true;
            break;

        default:
            break;
        }

        if (processed)
        {
            compute_output();
        }

        return processed;
    }

    void TeleopStateMachine::compute_output()
    {
        double linear_speed = config_.walk_vel;

        double cur_vx = 0.0;
        double cur_vy = 0.0;
        double cur_omega = 0.0;

        // 计算平移速度
        if (key_w_)
            cur_vx += linear_speed;
        if (key_s_)
            cur_vx -= linear_speed;
        if (key_a_)
            cur_vy += linear_speed;
        if (key_d_)
            cur_vy -= linear_speed;

        // 归一化斜向速度（防止速度叠加）
        double speed_magnitude = std::sqrt(cur_vx * cur_vx + cur_vy * cur_vy);
        if (speed_magnitude > linear_speed * 1.01)
        {
            double scale = linear_speed / speed_magnitude;
            cur_vx *= scale;
            cur_vy *= scale;
        }

        // 计算旋转速度
        if (current_mode_ == MotionMode::ROTATION && rotation_latched_)
        {
            cur_omega = latched_rotation_value_;
        }

        // 根据互斥模式生成最终输出
        if (current_mode_ == MotionMode::TRANSLATION)
        {
            output_.vx = cur_vx;
            output_.vy = cur_vy;
            output_.omega = 0.0;
        }
        else if (current_mode_ == MotionMode::ROTATION && rotation_latched_)
        {
            output_.vx = 0.0;
            output_.vy = 0.0;
            output_.omega = latched_rotation_value_;
        }
        else
        {
            output_.vx = 0.0;
            output_.vy = 0.0;
            output_.omega = 0.0;
        }

        // 判断是否有运动
        output_.has_motion = (std::abs(output_.vx) > 1e-9) ||
        (std::abs(output_.vy) > 1e-9) || (std::abs(output_.omega) > 1e-9);
    }

    void TeleopStateMachine::adjust_walk_vel(double delta)
    {
        config_.walk_vel += delta;
        // 限制范围在 0.1 到 5.0 之间
        if (config_.walk_vel > 5.0)
            config_.walk_vel = 5.0;
        if (config_.walk_vel < 0.1)
            config_.walk_vel = 0.1;
        
        // 重新计算输出（如果当前有运动）
        compute_output();
    }

    void TeleopStateMachine::adjust_default_omega(double delta)
    {
        config_.default_omega += delta;
        // 限制范围在 0.1 到 5.0 之间
        if (config_.default_omega > 5.0)
            config_.default_omega = 5.0;
        if (config_.default_omega < 0.1)
            config_.default_omega = 0.1;
        
        // 如果当前正在旋转锁存，同步更新锁存值
        if (rotation_latched_)
        {
            latched_rotation_value_ = (latched_rotation_value_ > 0) ? config_.default_omega : -config_.default_omega;
            compute_output();
        }
    }

} // namespace sentry_chassis_controller
