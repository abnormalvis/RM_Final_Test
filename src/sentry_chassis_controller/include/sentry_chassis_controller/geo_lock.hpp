// 几何自锁模块：利用轮子滚动方向与平移方向垂直的特性，通过将轮子转到特定角度
// 形成"X"字布局，使外力只能推动轮子侧滑（滑动摩擦远大于滚动摩擦），从而抵抗外力

#ifndef SENTRY_CHASSIS_CONTROLLER_GEO_LOCK_HPP
#define SENTRY_CHASSIS_CONTROLLER_GEO_LOCK_HPP

#include <ros/time.h>

namespace sentry_chassis_controller
{

    /**
     * @brief 几何自锁配置参数
     */
    struct GeoLockConfig
    {
        bool enabled{true};             // 几何自锁功能开关
        double idle_timeout{0.5};       // 空闲超时时间（秒），超过此时间无命令则进入自锁
        bool wheel_brake{false};         // 自锁时是否同时锁定轮子（true=锁定，false=自由滚动）
        double velocity_deadband{0.05}; // 速度死区阈值（m/s），用于判断是否静止

        // 轮子位置锁定 PD 参数（仅在 wheel_brake=true 时使用）
        double lock_pos_p{8.0};       // 位置 P 增益
        double lock_pos_d{1.0};       // 位置 D 增益（阻尼）
        double max_lock_torque{30.0}; // 最大锁定力矩（N·m）

        // 底盘尺寸参数（用于计算几何自锁角度）
        double wheel_base{0.36};  // 轴距（m）
        double wheel_track{0.36}; // 轮距（m）
    };

    /**
     * @brief 几何自锁输入数据
     */
    struct GeoLockInput
    {
        ros::Time current_time;  // 当前时间
        ros::Time last_cmd_time; // 上次收到速度命令的时间

        // 当前轮子状态（4 个轮子）
        double wheel_positions[4];  // 当前轮子位置（rad）
        double wheel_velocities[4]; // 当前轮子速度（rad/s）
        double pivot_positions[4];  // 当前舵角位置（rad）
    };

    /**
     * @brief 几何自锁输出数据
     */
    struct GeoLockOutput
    {
        bool is_locked{false}; // 是否处于自锁状态

        // 控制命令（仅在 is_locked=true 时有效）
        double pivot_commands[4];         // 舵角命令（rad）
        double wheel_commands[4];         // 轮子命令（力矩 N·m 或速度 rad/s，取决于模式）
        bool use_position_control{false}; // true=wheel_commands 是力矩，false=wheel_commands 是速度
    };

    /**
     * @brief 几何自锁管理器
     *
     * 功能：
     * 1. 监测底盘空闲状态（超时未收到速度命令）
     * 2. 计算几何自锁舵角（X字布局）
     * 3. 生成自锁状态下的舵角和轮子控制命令
     * 4. 可选择轮子刹车或自由滚动
     */
    class GeoLock
    {
    public:
        /**
         * @brief 构造函数
         * @param config 几何自锁配置参数
         */
        explicit GeoLock(const GeoLockConfig &config);

        /**
         * @brief 更新几何自锁状态和命令
         * @param input 输入数据（当前状态和时间）
         * @param output 输出数据（自锁命令）
         *
         * 工作流程：
         * 1. 检查是否超时（current_time - last_cmd_time > idle_timeout）
         * 2. 如果超时且未锁定：进入自锁状态，记录当前位置
         * 3. 如果已锁定：生成自锁命令（舵角=几何自锁角度，轮子=位置锁定/自由滚动）
         * 4. 如果未超时：解除自锁状态
         */
        void update(const GeoLockInput &input, GeoLockOutput &output);

        /**
         * @brief 重置自锁状态（手动解锁）
         */
        void reset();

        /**
         * @brief 更新配置参数
         * @param config 新的配置参数
         */
        void set_config(const GeoLockConfig &config);

        /**
         * @brief 获取当前配置
         * @return 当前配置参数
         */
        const GeoLockConfig &get_config() const { return config_; }

        /**
         * @brief 获取几何自锁角度（调试用）
         * @param angles 输出数组（长度至少为4）
         */
        void get_lock_angles(double angles[4]) const;

    private:
        /**
         * @brief 预计算几何自锁舵角
         *
         * 几何原理：
         * - 轮子布局（右手系：x向前，y向左）：
         *   FL: (+wheel_base/2, +wheel_track/2)  →  切向角 = atan2(+rx, -ry) ≈ -45°
         *   FR: (+wheel_base/2, -wheel_track/2)  →  切向角 = atan2(+rx, +ry) ≈ +45°
         *   RL: (-wheel_base/2, +wheel_track/2)  →  切向角 = atan2(-rx, -ry) ≈ -135°
         *   RR: (-wheel_base/2, -wheel_track/2)  →  切向角 = atan2(-rx, +ry) ≈ +135°
         *
         * - 切向方向：使轮子滚动方向沿底盘自转切线（垂直于平移方向）
         * - 公式：θ = atan2(rx, -ry)，其中 rx = ±wheel_base/2, ry = ±wheel_track/2
         */
        void compute_lock_angles();

        /**
         * @brief 生成位置锁定命令（轮子刹车模式）
         * @param input 输入数据
         * @param output 输出数据
         */
        void apply_wheel_brake(const GeoLockInput &input, GeoLockOutput &output);

        GeoLockConfig config_;         // 配置参数
        bool is_locked_{false};        // 当前是否处于自锁状态
        double lock_angles_[4]{};      // 预计算的几何自锁舵角（rad）
        double locked_wheel_pos_[4]{}; // 进入自锁时记录的轮子位置（rad）
    };

} // namespace sentry_chassis_controller

#endif // SENTRY_CHASSIS_CONTROLLER_GEO_LOCK_HPP
