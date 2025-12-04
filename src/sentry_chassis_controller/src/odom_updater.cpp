#include "sentry_chassis_controller/odom_updater.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace sentry_chassis_controller
{

    OdomUpdater::OdomUpdater()
        : odom_x_(0.0), odom_y_(0.0), odom_yaw_(0.0)
    {
        ROS_INFO("OdomUpdater initialized");
    }

    OdomUpdater::~OdomUpdater()
    {
    }

    void OdomUpdater::reset_pose(double x, double y, double yaw)
    {
        odom_x_ = x;
        odom_y_ = y;
        odom_yaw_ = yaw;
        ROS_INFO("OdomUpdater pose reset to (%.3f, %.3f, %.3f rad)", x, y, yaw);
    }

    bool OdomUpdater::solve_least_squares(const double A[4][3], const double b[4], double x[3], double &det)
    {
        // ==================== 计算 A^T * A (3x3) 和 A^T * b (3x1) ====================
        double ATA[3][3] = {{0}}; // A^T * A 的结果矩阵（初始化为零）
        double ATb[3] = {0};      // A^T * b 的结果向量（初始化为零）

        // 双重循环计算矩阵乘法
        for (int i = 0; i < 4; ++i) // 遍历四个轮子（观测）
        {
            for (int j = 0; j < 3; ++j) // 遍历未知数维度（vx/vy/wz）
            {
                ATb[j] += A[i][j] * b[i]; // A^T * b 的第 j 个元素
                for (int k = 0; k < 3; ++k)
                {
                    ATA[j][k] += A[i][j] * A[i][k]; // A^T * A 的 (j,k) 元素
                }
            }
        }

        // ==================== 计算行列式（判断矩阵是否奇异） ====================
        // 使用萨吕斯法则（对3x3矩阵展开）
        det = ATA[0][0] * (ATA[1][1] * ATA[2][2] - ATA[1][2] * ATA[2][1]) -
              ATA[0][1] * (ATA[1][0] * ATA[2][2] - ATA[1][2] * ATA[2][0]) +
              ATA[0][2] * (ATA[1][0] * ATA[2][1] - ATA[1][1] * ATA[2][0]);

        // 判断矩阵是否奇异（行列式绝对值 > 阈值表示可逆）
        if (std::abs(det) < 1e-9)
        {
            // 矩阵奇异，可能原因：四轮舵角共线、传感器故障、初始化阶段等
            x[0] = x[1] = x[2] = 0.0;
            return false;
        }

        // ==================== 计算逆矩阵（伴随矩阵法） ====================
        double inv[3][3]; // 存储逆矩阵

        // 第一行（对应 vx）
        inv[0][0] = (ATA[1][1] * ATA[2][2] - ATA[1][2] * ATA[2][1]) / det;
        inv[0][1] = (ATA[0][2] * ATA[2][1] - ATA[0][1] * ATA[2][2]) / det;
        inv[0][2] = (ATA[0][1] * ATA[1][2] - ATA[0][2] * ATA[1][1]) / det;

        // 第二行（对应 vy）
        inv[1][0] = (ATA[1][2] * ATA[2][0] - ATA[1][0] * ATA[2][2]) / det;
        inv[1][1] = (ATA[0][0] * ATA[2][2] - ATA[0][2] * ATA[2][0]) / det;
        inv[1][2] = (ATA[0][2] * ATA[1][0] - ATA[0][0] * ATA[1][2]) / det;

        // 第三行（对应 wz）
        inv[2][0] = (ATA[1][0] * ATA[2][1] - ATA[1][1] * ATA[2][0]) / det;
        inv[2][1] = (ATA[0][1] * ATA[2][0] - ATA[0][0] * ATA[2][1]) / det;
        inv[2][2] = (ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0]) / det;

        // ==================== 求解：x = inv(ATA) * ATb ====================
        x[0] = inv[0][0] * ATb[0] + inv[0][1] * ATb[1] + inv[0][2] * ATb[2]; // vx
        x[1] = inv[1][0] * ATb[0] + inv[1][1] * ATb[1] + inv[1][2] * ATb[2]; // vy
        x[2] = inv[2][0] * ATb[0] + inv[2][1] * ATb[1] + inv[2][2] * ATb[2]; // wz

        return true;
    }

    void OdomUpdater::update(const OdomUpdaterInput &input,
                             const ros::Time &time,
                             const ros::Duration &period,
                             ros::Publisher &odom_pub,
                             OdomUpdaterOutput &output)
    {
        // ==================== 步骤 1: 准备前向运动学方程数据 ====================
        // 舵轮前向运动学（FK）：已知每个轮子的舵角和轮速，反推底盘速度 (vx, vy, wz)
        // 模型：对于第 i 个轮子，沿舵向的线速度 v_i 满足
        //   v_i = cos(θ_i)*vx + sin(θ_i)*vy + (-ry_i*cos(θ_i) + rx_i*sin(θ_i))*wz
        // 四个轮子形成超定方程组 A*[vx; vy; wz] = b（4方程3未知数），用最小二乘求解

        // 轮子相对底盘中心 (base_link) 的位置（右手坐标系：x向前，y向左）
        // FL=(+x/2, +y/2), FR=(+x/2, -y/2), RL=(-x/2, +y/2), RR=(-x/2, -y/2)
        double rx[4] = {
            input.wheel_base / 2.0, input.wheel_base / 2.0,
            -input.wheel_base / 2.0, -input.wheel_base / 2.0}; // x坐标

        double ry[4] = {
            input.wheel_track / 2.0, -input.wheel_track / 2.0,
            input.wheel_track / 2.0, -input.wheel_track / 2.0}; // y坐标

        // 初始化系数矩阵 A (4x3) 和观测向量 b (4x1)
        double A[4][3]; // 每行对应一个轮子的运动学方程系数
        double b[4];    // 每个元素是轮子沿舵向的线速度（由轮速×轮径得到）

        // 循环构造每个轮子的方程（i = 0..3，对应 FL/FR/RL/RR）
        for (int i = 0; i < 4; ++i)
        {
            double theta = input.pivot_angles[i]; // 当前轮子的舵角（弧度）
            double cos_t = std::cos(theta);       // 舵角的余弦（舵向 x 分量）
            double sin_t = std::sin(theta);       // 舵角的正弦（舵向 y 分量）

            // 将轮子的角速度转换为沿舵向的线速度：v_along = ω × r
            double v_along = input.wheel_velocities[i] * input.wheel_radius;

            // 组装系数矩阵 A 的第 i 行（对应底盘速度 [vx, vy, wz] 的贡献）
            // v_i = A[i][0]*vx + A[i][1]*vy + A[i][2]*wz
            A[i][0] = cos_t;                          // vx 的系数：舵向在 x 轴投影
            A[i][1] = sin_t;                          // vy 的系数：舵向在 y 轴投影
            A[i][2] = -ry[i] * cos_t + rx[i] * sin_t; // wz 的系数：底盘旋转导致该轮位置的切向速度在舵向上的投影

            // 观测向量 b：轮子实测的沿舵向线速度
            b[i] = v_along;
        }

        // ==================== 步骤 2: 最小二乘求解 ====================
        double solution[3] = {0.0, 0.0, 0.0}; // vx, vy, wz
        double det = 0.0;
        bool valid = solve_least_squares(A, b, solution, det);

        // 保存求解状态
        output.solution_valid = valid;
        output.matrix_determinant = det;

        if (!valid)
        {
            // 矩阵奇异，使用零速度作为回退（避免发布错误的里程计）
            ROS_WARN_THROTTLE(1.0, "Forward kinematics matrix singular (det=%.2e), using zero velocity", det);
            output.vx = output.vy = output.wz = 0.0;
        }
        else
        {
            output.vx = solution[0];
            output.vy = solution[1];
            output.wz = solution[2];
        }

        // ==================== 步骤 3: 速度死区滤波（消除静止抖动） ====================
        if (std::abs(output.vx) < input.velocity_deadband)
        {
            output.vx = 0.0;
        }
        if (std::abs(output.vy) < input.velocity_deadband)
        {
            output.vy = 0.0;
        }
        if (std::abs(output.wz) < input.velocity_deadband)
        {
            output.wz = 0.0;
        }

        // ==================== 步骤 4: 坐标变换与位姿积分 ====================
        double dt = period.toSec(); // 时间步长（秒）

        // 将机体坐标系速度 (vx, vy) 旋转到世界坐标系（odom frame）
        double cos_y = std::cos(odom_yaw_);
        double sin_y = std::sin(odom_yaw_);
        double world_vx = output.vx * cos_y - output.vy * sin_y; // 世界系 x 方向速度
        double world_vy = output.vx * sin_y + output.vy * cos_y; // 世界系 y 方向速度

        // 欧拉积分更新位姿
        odom_x_ += world_vx * dt;
        odom_y_ += world_vy * dt;
        odom_yaw_ += output.wz * dt;

        // 保存到输出
        output.odom_x = odom_x_;
        output.odom_y = odom_y_;
        output.odom_yaw = odom_yaw_;

        // ==================== 步骤 5: 发布 TF 变换 ====================
        if (input.publish_tf)
        {
            geometry_msgs::TransformStamped t;
            t.header.stamp = time;
            t.header.frame_id = input.odom_frame;
            t.child_frame_id = input.base_link_frame;

            t.transform.translation.x = odom_x_;
            t.transform.translation.y = odom_y_;
            t.transform.translation.z = 0.0;

            // 将欧拉角转换为四元数
            tf2::Quaternion qtn;
            qtn.setRPY(0.0, 0.0, odom_yaw_);
            t.transform.rotation.x = qtn.getX();
            t.transform.rotation.y = qtn.getY();
            t.transform.rotation.z = qtn.getZ();
            t.transform.rotation.w = qtn.getW();

            tf_broadcaster_.sendTransform(t);
        }

        // ==================== 步骤 6: 发布里程计消息 ====================
        nav_msgs::Odometry odom;
        odom.header.stamp = time;
        odom.header.frame_id = input.odom_frame;
        odom.child_frame_id = input.base_link_frame;

        // 位姿（世界坐标系）
        odom.pose.pose.position.x = odom_x_;
        odom.pose.pose.position.y = odom_y_;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion qtn;
        qtn.setRPY(0.0, 0.0, odom_yaw_);
        odom.pose.pose.orientation.x = qtn.getX();
        odom.pose.pose.orientation.y = qtn.getY();
        odom.pose.pose.orientation.z = qtn.getZ();
        odom.pose.pose.orientation.w = qtn.getW();

        // 速度（机体坐标系）
        odom.twist.twist.linear.x = output.vx;
        odom.twist.twist.linear.y = output.vy;
        odom.twist.twist.angular.z = output.wz;

        odom_pub.publish(odom);
    }

} // namespace sentry_chassis_controller
