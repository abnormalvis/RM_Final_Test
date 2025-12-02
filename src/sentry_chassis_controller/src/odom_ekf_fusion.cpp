#include "sentry_chassis_controller/odom_ekf_fusion.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace sentry_chassis_controller
{

OdomEkfFusion::OdomEkfFusion()
{
    // 初始化状态向量 [x, y, θ, vx, vy, ω]
    state_ = Eigen::VectorXd::Zero(6);
    
    // 初始化协方差矩阵
    P_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;
    Q_ = Eigen::MatrixXd::Identity(6, 6);
    R_odom_ = Eigen::MatrixXd::Identity(3, 3);
}

void OdomEkfFusion::init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    // 读取参数
    pnh.param<std::string>("odom_frame", odom_frame_, "odom");
    pnh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
    pnh.param<std::string>("imu_frame", imu_frame_, "imu_link");
    pnh.param<double>("publish_rate", publish_rate_, 50.0);
    pnh.param<bool>("publish_tf", publish_tf_, true);
    
    // 噪声参数
    pnh.param<double>("process_noise_pos", process_noise_pos_, 0.01);
    pnh.param<double>("process_noise_vel", process_noise_vel_, 0.1);
    pnh.param<double>("process_noise_yaw", process_noise_yaw_, 0.01);
    pnh.param<double>("odom_noise_vel", odom_noise_vel_, 0.05);
    pnh.param<double>("odom_noise_omega", odom_noise_omega_, 0.05);
    pnh.param<double>("imu_accel_noise", imu_accel_noise_, 0.1);
    pnh.param<double>("imu_gyro_noise", imu_gyro_noise_, 0.01);

    // 设置过程噪声矩阵 Q
    Q_(0, 0) = process_noise_pos_;   // x
    Q_(1, 1) = process_noise_pos_;   // y
    Q_(2, 2) = process_noise_yaw_;   // θ
    Q_(3, 3) = process_noise_vel_;   // vx
    Q_(4, 4) = process_noise_vel_;   // vy
    Q_(5, 5) = process_noise_yaw_;   // ω

    // 设置轮式里程计观测噪声矩阵 R
    R_odom_(0, 0) = odom_noise_vel_ * odom_noise_vel_;    // vx
    R_odom_(1, 1) = odom_noise_vel_ * odom_noise_vel_;    // vy
    R_odom_(2, 2) = odom_noise_omega_ * odom_noise_omega_; // ω

    // 订阅 IMU 和轮式里程计
    imu_sub_ = nh.subscribe("/imu/data", 100, &OdomEkfFusion::imuCallback, this);
    wheel_odom_sub_ = nh.subscribe("/odom_controller", 10, &OdomEkfFusion::wheelOdomCallback, this);
    
    // 发布融合后的里程计
    fused_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom_fused", 10);
    
    // 定时发布
    publish_timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_), 
                                     &OdomEkfFusion::timerCallback, this);

    ROS_INFO("[EKF] Odom-IMU fusion node initialized");
    ROS_INFO("[EKF] Subscribing: /imu/data, /odom_controller");
    ROS_INFO("[EKF] Publishing: /odom_fused (rate: %.1f Hz), TF: %s", 
             publish_rate_, publish_tf_ ? "enabled" : "disabled");
}

void OdomEkfFusion::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    // IMU 偏置初始化（静止时采样）
    if (!initialized_ && bias_sample_count_ < BIAS_SAMPLE_NUM)
    {
        gyro_bias_(0) += msg->angular_velocity.x;
        gyro_bias_(1) += msg->angular_velocity.y;
        gyro_bias_(2) += msg->angular_velocity.z;
        accel_bias_(0) += msg->linear_acceleration.x;
        accel_bias_(1) += msg->linear_acceleration.y;
        accel_bias_(2) += msg->linear_acceleration.z - 9.81;  // 减去重力
        
        bias_sample_count_++;
        
        if (bias_sample_count_ == BIAS_SAMPLE_NUM)
        {
            gyro_bias_ /= BIAS_SAMPLE_NUM;
            accel_bias_ /= BIAS_SAMPLE_NUM;
            initialized_ = true;
            last_predict_time_ = msg->header.stamp;
            ROS_INFO("[EKF] IMU bias initialized: gyro=[%.4f, %.4f, %.4f], accel=[%.4f, %.4f, %.4f]",
                     gyro_bias_(0), gyro_bias_(1), gyro_bias_(2),
                     accel_bias_(0), accel_bias_(1), accel_bias_(2));
        }
        return;
    }
    
    if (!initialized_) return;
    
    // 计算时间步长
    double dt = (msg->header.stamp - last_predict_time_).toSec();
    if (dt <= 0 || dt > 0.5)
    {
        last_predict_time_ = msg->header.stamp;
        return;
    }
    
    // 提取 IMU 数据并减去偏置
    Eigen::Vector3d angular_vel(
        msg->angular_velocity.x - gyro_bias_(0),
        msg->angular_velocity.y - gyro_bias_(1),
        msg->angular_velocity.z - gyro_bias_(2)
    );
    
    Eigen::Vector3d linear_accel(
        msg->linear_acceleration.x - accel_bias_(0),
        msg->linear_acceleration.y - accel_bias_(1),
        msg->linear_acceleration.z - accel_bias_(2)
    );
    
    // EKF 预测步骤
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        predict(dt, angular_vel, linear_accel);
    }
    
    last_predict_time_ = msg->header.stamp;
    latest_imu_ = *msg;
    imu_received_ = true;
}

void OdomEkfFusion::wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    
    if (!initialized_) return;
    
    // 提取轮式里程计速度（body 坐标系）
    double odom_vx = msg->twist.twist.linear.x;
    double odom_vy = msg->twist.twist.linear.y;
    double odom_omega = msg->twist.twist.angular.z;
    
    // EKF 更新步骤
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        update(odom_vx, odom_vy, odom_omega);
    }
    
    last_update_time_ = msg->header.stamp;
    latest_wheel_odom_ = *msg;
    wheel_odom_received_ = true;
}

void OdomEkfFusion::predict(double dt, const Eigen::Vector3d& angular_vel, 
                             const Eigen::Vector3d& linear_accel)
{
    // 当前状态
    double x = state_(0);
    double y = state_(1);
    double theta = state_(2);
    double vx = state_(3);
    double vy = state_(4);
    double omega = state_(5);
    
    // IMU 提供的角速度（z 轴）用于更新航向
    double omega_imu = angular_vel(2);
    
    // 将 body 系加速度转换到世界系
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    double ax_world = linear_accel(0) * cos_theta - linear_accel(1) * sin_theta;
    double ay_world = linear_accel(0) * sin_theta + linear_accel(1) * cos_theta;
    
    // ==================== 状态预测 ====================
    // 使用二阶 Runge-Kutta 积分（中点法）
    double theta_mid = theta + omega_imu * dt / 2.0;
    double cos_mid = std::cos(theta_mid);
    double sin_mid = std::sin(theta_mid);
    
    // 速度更新（考虑加速度）
    double vx_new = vx + ax_world * dt;
    double vy_new = vy + ay_world * dt;
    
    // 位置更新
    double x_new = x + (vx + vx_new) / 2.0 * dt;
    double y_new = y + (vy + vy_new) / 2.0 * dt;
    
    // 航向更新
    double theta_new = normalizeAngle(theta + omega_imu * dt);
    
    // 更新状态
    state_(0) = x_new;
    state_(1) = y_new;
    state_(2) = theta_new;
    state_(3) = vx_new;
    state_(4) = vy_new;
    state_(5) = omega_imu;  // 角速度直接用 IMU 值
    
    // ==================== 雅可比矩阵 F（状态转移矩阵的导数） ====================
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 2) = -vx * sin_theta * dt - vy * cos_theta * dt;  // dx/dθ
    F(0, 3) = cos_theta * dt;                               // dx/dvx
    F(0, 4) = -sin_theta * dt;                              // dx/dvy
    F(1, 2) = vx * cos_theta * dt - vy * sin_theta * dt;   // dy/dθ
    F(1, 3) = sin_theta * dt;                               // dy/dvx
    F(1, 4) = cos_theta * dt;                               // dy/dvy
    F(2, 5) = dt;                                           // dθ/dω
    
    // ==================== 协方差预测 ====================
    // P = F * P * F' + Q
    P_ = F * P_ * F.transpose() + Q_ * dt;
}

void OdomEkfFusion::update(double odom_vx, double odom_vy, double odom_omega)
{
    // 当前状态
    double theta = state_(2);
    double vx_world = state_(3);
    double vy_world = state_(4);
    double omega_state = state_(5);
    
    // 将世界系速度转换到 body 系进行比较
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    double vx_body_pred = vx_world * cos_theta + vy_world * sin_theta;
    double vy_body_pred = -vx_world * sin_theta + vy_world * cos_theta;
    
    // ==================== 观测矩阵 H ====================
    // z = [vx_body, vy_body, ω]
    // 观测模型: z = h(x) 其中 h 将世界系速度转换到 body 系
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    // dvx_body/dθ
    H(0, 2) = -vx_world * sin_theta + vy_world * cos_theta;
    // dvx_body/dvx_world
    H(0, 3) = cos_theta;
    // dvx_body/dvy_world
    H(0, 4) = sin_theta;
    // dvy_body/dθ
    H(1, 2) = -vx_world * cos_theta - vy_world * sin_theta;
    // dvy_body/dvx_world
    H(1, 3) = -sin_theta;
    // dvy_body/dvy_world
    H(1, 4) = cos_theta;
    // dω/dω
    H(2, 5) = 1.0;
    
    // ==================== 创新（残差） ====================
    Eigen::Vector3d z_pred(vx_body_pred, vy_body_pred, omega_state);
    Eigen::Vector3d z_meas(odom_vx, odom_vy, odom_omega);
    Eigen::Vector3d y = z_meas - z_pred;  // 创新
    
    // ==================== 卡尔曼增益 ====================
    // S = H * P * H' + R
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_odom_;
    // K = P * H' * S^(-1)
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // ==================== 状态更新 ====================
    state_ = state_ + K * y;
    state_(2) = normalizeAngle(state_(2));  // 归一化航向角
    
    // ==================== 协方差更新 ====================
    // P = (I - K * H) * P
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    P_ = (I - K * H) * P_;
}

void OdomEkfFusion::timerCallback(const ros::TimerEvent& event)
{
    if (!initialized_) return;
    
    publishFusedOdom();
}

void OdomEkfFusion::publishFusedOdom()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    ros::Time now = ros::Time::now();
    
    // 构建融合后的里程计消息
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_link_frame_;
    
    // 位姿
    odom_msg.pose.pose.position.x = state_(0);
    odom_msg.pose.pose.position.y = state_(1);
    odom_msg.pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, state_(2));
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    
    // 位姿协方差（简化：只填对角线）
    odom_msg.pose.covariance[0] = P_(0, 0);   // x
    odom_msg.pose.covariance[7] = P_(1, 1);   // y
    odom_msg.pose.covariance[35] = P_(2, 2);  // θ
    
    // 速度（转换到 body 系）
    double theta = state_(2);
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    odom_msg.twist.twist.linear.x = state_(3) * cos_theta + state_(4) * sin_theta;
    odom_msg.twist.twist.linear.y = -state_(3) * sin_theta + state_(4) * cos_theta;
    odom_msg.twist.twist.angular.z = state_(5);
    
    // 速度协方差
    odom_msg.twist.covariance[0] = P_(3, 3);   // vx
    odom_msg.twist.covariance[7] = P_(4, 4);   // vy
    odom_msg.twist.covariance[35] = P_(5, 5);  // ω
    
    fused_odom_pub_.publish(odom_msg);
    
    // 发布 TF
    if (publish_tf_)
    {
        geometry_msgs::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = odom_frame_;
        t.child_frame_id = base_link_frame_;
        t.transform.translation.x = state_(0);
        t.transform.translation.y = state_(1);
        t.transform.translation.z = 0.0;
        t.transform.rotation = odom_msg.pose.pose.orientation;
        
        tf_broadcaster_.sendTransform(t);
    }
}

double OdomEkfFusion::normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double OdomEkfFusion::getYawFromQuaternion(const geometry_msgs::Quaternion& q)
{
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void OdomEkfFusion::spin()
{
    ros::spin();
}

}  // namespace sentry_chassis_controller

// ==================== Main ====================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_ekf_fusion");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    sentry_chassis_controller::OdomEkfFusion ekf;
    ekf.init(nh, pnh);
    ekf.spin();
    
    return 0;
}
