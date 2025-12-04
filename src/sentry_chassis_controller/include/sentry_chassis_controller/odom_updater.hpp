#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>

namespace sentry_chassis_controller
{

    /**
     * @brief 里程计更新器输入数据
     *
     * 包含四个舵轮的反馈数据和底盘几何参数
     */
    struct OdomUpdaterInput
    {
        // 四个轮子的反馈数据（索引：0=FL, 1=FR, 2=RL, 3=RR）
        double wheel_velocities[4]; // 轮子角速度 (rad/s)
        double pivot_angles[4];     // 舵角位置 (rad)

        // 底盘几何参数
        double wheel_base;   // 轴距（前后轮间距，m）
        double wheel_track;  // 轮距（左右轮间距，m）
        double wheel_radius; // 轮子半径 (m)

        // 坐标系配置
        std::string odom_frame;      // 里程计坐标系名称（默认 "odom"）
        std::string base_link_frame; // 底盘坐标系名称（默认 "base_link"）

        // 发布控制
        bool publish_tf{true}; // 是否发布 TF 变换

        // 速度死区（用于消除静止抖动导致的漂移）
        double velocity_deadband{0.05}; // 速度死区阈值 (m/s 或 rad/s)
    };

    /**
     * @brief 里程计更新器输出数据
     *
     * 包含求解的底盘速度和调试信息
     */
    struct OdomUpdaterOutput
    {
        // 底盘速度（机体坐标系）
        double vx{0.0}; // x 方向线速度 (m/s)
        double vy{0.0}; // y 方向线速度 (m/s)
        double wz{0.0}; // 绕 z 轴角速度 (rad/s)

        // 求解状态
        bool solution_valid{false};     // 最小二乘解是否有效（矩阵非奇异）
        double matrix_determinant{0.0}; // ATA 矩阵的行列式（调试用）

        // 位姿（世界坐标系，累积积分结果）
        double odom_x{0.0};   // x 坐标 (m)
        double odom_y{0.0};   // y 坐标 (m)
        double odom_yaw{0.0}; // 航向角 (rad)
    };

    /**
     * @brief 里程计更新器类
     *
     * 封装四舵轮前向运动学的里程计计算逻辑，包括：
     * 1. 最小二乘求解超定方程（4个轮子→3个底盘速度）
     * 2. 坐标变换（机体系→世界系）
     * 3. 欧拉积分更新位姿
     * 4. 发布 Odometry 消息和 TF 变换
     */
    class OdomUpdater
    {
    public:
        /**
         * @brief 构造函数
         */
        OdomUpdater();

        /**
         * @brief 析构函数
         */
        ~OdomUpdater();

        /**
         * @brief 更新里程计
         *
         * 基于四舵轮反馈数据计算底盘速度，积分更新位姿，并发布里程计和TF
         *
         * 算法原理：
         * 1. 构建超定线性方程组 A*[vx; vy; wz] = b（4方程3未知数）
         * 2. 使用最小二乘法求解：x = (A^T*A)^(-1) * A^T * b
         * 3. 将机体系速度旋转到世界系并积分得到位姿
         * 4. 发布 nav_msgs::Odometry 和 TF 变换
         *
         * @param input 输入数据（轮速、舵角、底盘参数等）
         * @param time 当前时间戳
         * @param period 控制周期（用于积分）
         * @param odom_pub 里程计发布器
         * @param output 输出数据（速度、位姿、求解状态）
         */
        void update(const OdomUpdaterInput &input,
                    const ros::Time &time,
                    const ros::Duration &period,
                    ros::Publisher &odom_pub,
                    OdomUpdaterOutput &output);

        /**
         * @brief 重置里程计位姿（用于重新定位或测试）
         *
         * @param x 新的 x 坐标 (m)
         * @param y 新的 y 坐标 (m)
         * @param yaw 新的航向角 (rad)
         */
        void reset_pose(double x = 0.0, double y = 0.0, double yaw = 0.0);

    private:
        // 内部状态：累积的里程计位姿（世界坐标系）
        double odom_x_{0.0};   // x 坐标 (m)
        double odom_y_{0.0};   // y 坐标 (m)
        double odom_yaw_{0.0}; // 航向角 (rad)

        // TF 广播器（持久化，避免重复构造）
        tf2_ros::TransformBroadcaster tf_broadcaster_;

        /**
         * @brief 求解最小二乘问题（内部辅助函数）
         *
         * 求解超定方程组 A*x = b，其中 A 是 4x3 矩阵，b 是 4x1 向量
         * 使用法方程：x = (A^T*A)^(-1) * A^T * b
         *
         * @param A 系数矩阵 (4x3)
         * @param b 观测向量 (4x1)
         * @param x 解向量 (3x1)，输出参数
         * @return 是否求解成功（矩阵非奇异）
         */
        bool solve_least_squares(const double A[4][3], const double b[4], double x[3], double &det);
    };

} // namespace sentry_chassis_controller
