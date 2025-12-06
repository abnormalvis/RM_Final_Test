/*
 * TeleopStateMachine - 遥操作状态机
 * 功能：管理运动模式、按键状态、互斥逻辑、锁存行为
 * 输入：按键事件 + 时间戳
 * 输出：运动状态（线速度、角速度）
 */

#ifndef SENTRY_CHASSIS_CONTROLLER_TELEOP_STATE_MACHINE_HPP
#define SENTRY_CHASSIS_CONTROLLER_TELEOP_STATE_MACHINE_HPP

#include <string>

namespace sentry_chassis_controller
{

    /*
     * 配置参数
     */
    struct TeleopConfig
    {
        double walk_vel;            // 步行速度 (m/s)，可通过 u/i 键调节
        double default_omega;       // 默认角速度 (rad/s)，可通过 o/p 键调节
        std::string velocity_mode;  // "global" 或 "chassis"

        TeleopConfig()
            : walk_vel(0.5), default_omega(1.0), velocity_mode("global")
        {
        }
    };

    /*
     * 输出状态
     */
    struct TeleopOutput
    {
        double vx;       // x 方向速度
        double vy;       // y 方向速度
        double omega;    // 角速度
        bool has_motion; // 是否有运动

        TeleopOutput()
            : vx(0.0), vy(0.0), omega(0.0), has_motion(false)
        {
        }

        void reset()
        {
            vx = vy = omega = 0.0;
            has_motion = false;
        }
    };

    /*
     * 运动模式枚举
     */
    enum class MotionMode
    {
        NONE,        // 无运动
        TRANSLATION, // 平移模式
        ROTATION     // 旋转模式
    };

    /*
     * 状态机类
     */
    class TeleopStateMachine
    {
    public:
        explicit TeleopStateMachine(const TeleopConfig &config);

        /*
         * 处理按键事件
         * @param key 按键字符（已转小写）
         * @param current_time 当前时间（秒）
         * @return 是否处理了按键
         */
        bool process_key(char key, double current_time);

        /*
         * 获取当前输出状态
         */
        const TeleopOutput &get_output() const { return output_; }

        /*
         * 重置状态机
         */
        void reset();

        /*
         * 获取当前运动模式
         */
        MotionMode get_mode() const { return current_mode_; }

        /*
         * 更新配置
         */
        void set_config(const TeleopConfig &config) { config_ = config; }
        const TeleopConfig &get_config() const { return config_; }

    private:
        // 配置
        TeleopConfig config_;

        // 按键状态
        bool key_w_, key_s_, key_a_, key_d_; // 平移按键
        bool key_q_, key_e_;                 // 旋转按键

        // 运动模式
        MotionMode current_mode_;

        // 旋转锁存
        bool rotation_latched_;
        double latched_rotation_value_;

        // 输出状态
        TeleopOutput output_;

        // 内部方法
        void clear_translation_keys();
        void clear_rotation_keys();
        void compute_output();
        
        // 速度调节方法
        void adjust_walk_vel(double delta);     // 调节平移速度
        void adjust_default_omega(double delta); // 调节角速度
    };

} // namespace sentry_chassis_controller

#endif // SENTRY_CHASSIS_CONTROLLER_TELEOP_STATE_MACHINE_HPP
