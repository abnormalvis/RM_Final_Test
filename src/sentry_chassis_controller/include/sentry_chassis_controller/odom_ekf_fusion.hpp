#ifndef SENTRY_CHASSIS_CONTROLLER_ODOM_EKF_FUSION_HPP
#define SENTRY_CHASSIS_CONTROLLER_ODOM_EKF_FUSION_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <mutex>

namespace sentry_chassis_controller
{

/**
 * @brief 扩展卡尔曼滤波器 - 融合 IMU 和轮式里程计
 * 
 * 状态向量: [x, y, θ, vx, vy, ω]^T (位置、航向、速度)
 * - x, y: 世界坐标系位置 (m)
 * - θ: 航向角 (rad)
 * - vx, vy: 世界坐标系速度 (m/s)
 * - ω: 角速度 (rad/s)
 * 
 * 观测来源:
 * 1. IMU: 角速度 ω, 线加速度 ax, ay (高频，用于预测)
 * 2. 轮式里程计: vx_body, vy_body, ω (相对较低频，用于校正)
 */
class OdomEkfFusion
{
public:
    OdomEkfFusion();
    ~OdomEkfFusion() = default;

    /**
     * @brief 初始化 ROS 节点
     * @param nh NodeHandle
     * @param pnh Private NodeHandle
     */
    void init(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    /**
     * @brief 主循环（在 timer 回调中发布融合后的里程计）
     */
    void spin();

private:
    // ==================== EKF 核心函数 ====================
    
    /**
     * @brief EKF 预测步骤（使用 IMU 数据）
     * @param dt 时间步长
     * @param angular_vel IMU 角速度
     * @param linear_accel IMU 线加速度（世界系）
     */
    void predict(double dt, const Eigen::Vector3d& angular_vel, const Eigen::Vector3d& linear_accel);

    /**
     * @brief EKF 更新步骤（使用轮式里程计）
     * @param odom_vx 轮式里程计 x 速度（body 系）
     * @param odom_vy 轮式里程计 y 速度（body 系）
     * @param odom_omega 轮式里程计角速度
     */
    void update(double odom_vx, double odom_vy, double odom_omega);

    /**
     * @brief 发布融合后的里程计
     */
    void publishFusedOdom();

    // ==================== ROS 回调 ====================
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent& event);

    // ==================== 工具函数 ====================
    
    /**
     * @brief 归一化角度到 [-π, π]
     */
    double normalizeAngle(double angle);

    /**
     * @brief 从四元数提取 yaw 角
     */
    double getYawFromQuaternion(const geometry_msgs::Quaternion& q);

    // ==================== ROS 接口 ====================
    ros::Subscriber imu_sub_;
    ros::Subscriber wheel_odom_sub_;
    ros::Publisher fused_odom_pub_;
    ros::Timer publish_timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // ==================== EKF 状态 ====================
    Eigen::VectorXd state_;           // 状态向量 [x, y, θ, vx, vy, ω]
    Eigen::MatrixXd P_;               // 状态协方差矩阵 (6x6)
    Eigen::MatrixXd Q_;               // 过程噪声协方差 (6x6)
    Eigen::MatrixXd R_odom_;          // 轮式里程计观测噪声 (3x3)

    // ==================== 传感器数据缓存 ====================
    sensor_msgs::Imu latest_imu_;
    nav_msgs::Odometry latest_wheel_odom_;
    bool imu_received_{false};
    bool wheel_odom_received_{false};
    ros::Time last_predict_time_;
    ros::Time last_update_time_;

    // ==================== 配置参数 ====================
    std::string odom_frame_{"odom"};
    std::string base_link_frame_{"base_link"};
    std::string imu_frame_{"imu_link"};
    double publish_rate_{50.0};        // 发布频率 (Hz)
    bool publish_tf_{true};            // 是否发布 TF
    
    // 噪声参数
    double process_noise_pos_{0.01};   // 位置过程噪声
    double process_noise_vel_{0.1};    // 速度过程噪声
    double process_noise_yaw_{0.01};   // 航向过程噪声
    double odom_noise_vel_{0.05};      // 里程计速度观测噪声
    double odom_noise_omega_{0.05};    // 里程计角速度观测噪声
    double imu_accel_noise_{0.1};      // IMU 加速度噪声
    double imu_gyro_noise_{0.01};      // IMU 陀螺仪噪声

    // 线程安全
    std::mutex state_mutex_;
    std::mutex imu_mutex_;
    std::mutex odom_mutex_;

    // 初始化标志
    bool initialized_{false};
    
    // IMU 偏置估计（简化版，实际可用更复杂的在线估计）
    Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d accel_bias_{Eigen::Vector3d::Zero()};
    int bias_sample_count_{0};
    static constexpr int BIAS_SAMPLE_NUM = 100;  // 偏置采样数
};

}  // namespace sentry_chassis_controller

#endif  // SENTRY_CHASSIS_CONTROLLER_ODOM_EKF_FUSION_HPP
