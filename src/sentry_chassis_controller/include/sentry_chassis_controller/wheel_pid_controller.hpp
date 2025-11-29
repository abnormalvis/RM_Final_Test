/*
 * 舵轮底盘 PID 控制器头文件
 * 功能：基于 EffortJointInterface 的四舵轮独立控制
 *       - 舵角位置控制（4个pivot关节，PID控制到目标角度）
 *       - 轮速速度控制（4个wheel关节，PID控制到目标角速度）
 *       - 逆运动学求解（将底盘速度命令转换为各轮舵角+轮速）
 *       - 前向运动学与里程计发布（基于轮子反馈计算位姿）
 *       - 功率限制（防止电机/电源过载）
 *       - 动态参数调整（通过 dynamic_reconfigure）
 */

#ifndef SENTRY_CHASSIS_CONTROLLER_WHEEL_PID_CONTROLLER_H
#define SENTRY_CHASSIS_CONTROLLER_WHEEL_PID_CONTROLLER_H

// ROS 核心
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

// 消息类型
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

// 动态重配置
#include <dynamic_reconfigure/server.h>
#include <sentry_chassis_controller/WheelPidConfig.h>

// 自定义工具
#include <sentry_chassis_controller/inverse_kinematics.hpp>

// TF 相关
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

namespace sentry_chassis_controller
{

  /*
   * 舵轮 PID 控制器类
   * 继承自 ROS Control 的 Controller 基类，使用 EffortJointInterface（力矩接口）
   */
  class WheelPidController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
  {
  public:
    WheelPidController() = default;
    ~WheelPidController() override = default;

    /**
     * 初始化函数
     * @param effort_joint_interface: 硬件接口（用于获取关节句柄）
     * @param root_nh: 根命名空间句柄（用于话题订阅/发布）
     * @param controller_nh: 控制器命名空间句柄（用于参数读取）
     * @return 初始化成功返回 true
     */
    bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
              ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    /**
     * 控制循环更新函数（每个控制周期调用一次）
     * @param time: 当前时间戳
     * @param period: 距上次调用的时间间隔
     */
    void update(const ros::Time &time, const ros::Duration &period) override;

  private:
    // ==================== 关节句柄（硬件接口） ====================
    // 舵向关节（pivot/steer）：控制轮子转向角度
    hardware_interface::JointHandle front_left_pivot_joint_;   // 左前舵角关节
    hardware_interface::JointHandle front_right_pivot_joint_;  // 右前舵角关节
    hardware_interface::JointHandle back_left_pivot_joint_;    // 左后舵角关节
    hardware_interface::JointHandle back_right_pivot_joint_;   // 右后舵角关节
    
    // 轮速关节（wheel）：控制轮子旋转速度
    hardware_interface::JointHandle front_left_wheel_joint_;   // 左前轮速关节
    hardware_interface::JointHandle front_right_wheel_joint_;  // 右前轮速关节
    hardware_interface::JointHandle back_left_wheel_joint_;    // 左后轮速关节
    hardware_interface::JointHandle back_right_wheel_joint_;   // 右后轮速关节

    // ==================== PID 控制器 ====================
    // 舵角 PID（位置控制）：控制舵向电机到达目标角度
    control_toolbox::Pid pid_lf_, pid_rf_, pid_lb_, pid_rb_;  // 左前/右前/左后/右后
    
    // 轮速 PID（速度控制）：控制驱动电机到达目标角速度
    control_toolbox::Pid pid_lf_wheel_, pid_rf_wheel_, pid_lb_wheel_, pid_rb_wheel_;

    // ==================== ROS 句柄与参数 ====================
    // 控制器命名空间句柄（用于参数镜像回 YAML 风格键值）
    ros::NodeHandle controller_nh_;

    // ==================== 运动学参数 ====================
    double wheel_track_{0.36};    // 轮距（左右轮间距，米）
    double wheel_base_{0.36};     // 轴距（前后轮间距，米）
    double wheel_radius_{0.05};   // 轮子半径（米）

    // ==================== 功率限制参数 ====================
    // 参考 hero_chassis_controller 的功率限制算法
    double power_limit_{0.0};      // 功率上限（W），0 表示禁用
    double effort_coeff_{0.0};     // 力矩平方项系数（用于构造二次约束）
    double velocity_coeff_{0.0};   // 速度平方项系数
    double power_offset_{0.0};     // 基线功率偏置（W）
    bool power_limit_enabled_{false}; // 功率限制是否启用（自动根据系数判断）

    // ==================== 几何辅助变量 ====================
    double rx_{0.0};  // wheel_track / 2（轮距半值）
    double ry_{0.0};  // wheel_base / 2（轴距半值）

    // ==================== 期望命令缓存 ====================
    double pivot_cmd_[4]{};  // 四个舵轮的期望舵角（rad）
    double wheel_cmd_[4]{};  // 四个车轮的期望角速度（rad/s）

    // ==================== 订阅器与发布器 ====================
    // cmd_vel 订阅器（接收底盘速度命令）
    ros::Subscriber cmd_vel_sub_;
    std::string cmd_vel_topic_{"/cmd_vel"};  // 速度命令话题名（可配置，建议使用绝对路径）
    
    bool synthetic_fallback_{false};  // 合成关节状态回退标志（用于调试）
    
    // 期望命令发布器（用于测试/检查逆运动学输出）
    ros::Publisher desired_pub_;
    
    // 关节状态发布器（CRITICAL FIX：为前向运动学提供数据源）
    ros::Publisher joint_states_pub_;
    ros::Time last_state_pub_;  // 上次发布关节状态的时间戳
    
    // 应用力矩发布器（用于功率限制调试）
    ros::Publisher applied_effort_pub_;
    ros::Time last_effort_pub_;
    
    // 功率调试发布器（发布 a/b/c/disc/scaling_factor 数组）
    ros::Publisher power_debug_pub_;
    bool power_debug_enabled_{false};  // 是否启用功率调试发布

    // ==================== 里程计发布（集成到控制器内部） ====================
    ros::Publisher odom_pub_;                      // 里程计消息发布器
    tf2_ros::TransformBroadcaster tf_broadcaster_; // TF 变换广播器
    std::string odom_frame_{"odom"};               // 里程计坐标系名称
    std::string base_link_frame_{"base_link"};     // 底盘坐标系名称
    
    // 里程计状态（位姿累积）
    double odom_x_{0.0};      // 世界系 x 坐标（米）
    double odom_y_{0.0};      // 世界系 y 坐标（米）
    double odom_yaw_{0.0};    // 航向角（弧度）
    ros::Time last_odom_time_;

    // ==================== 速度模式（坐标系选择） ====================
    // "local" (base_link): cmd_vel 中的速度在底盘坐标系下解释
    // "global" (odom): cmd_vel 中的速度在世界坐标系下解释（需 TF 变换）
    std::string speed_mode_{"local"};
    // 注意：tf::TransformListener 在 cmdVelCallback 中本地创建以避免静态初始化问题

    // ==================== 动态重配置服务器 ====================
    typedef sentry_chassis_controller::WheelPidConfig Config;
    std::shared_ptr<dynamic_reconfigure::Server<Config>> dyn_server_;
    
    /**
     * 动态重配置回调函数
     * @param config: 新的配置参数
     * @param level: 变更级别掩码（未使用）
     */
    void reconfigureCallback(Config &config, uint32_t level);

    /**
     * cmd_vel 话题回调函数
     * 功能：接收速度命令，执行逆运动学并更新期望舵角/轮速
     * @param msg: Twist 消息（底盘速度命令）
     */
    void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg);

    // ==================== PID 初始化辅助函数 ====================
    /**
     * 初始化单个舵角 PID
     * @param name: 舵角名称（如 "pivot_fl"）
     * @param pid: PID 控制器对象引用
     * @param controller_nh: 控制器命名空间句柄
     * @param def_p/i/d/i_clamp/antiwindup: 默认 PID 参数
     */
    void initPivot(const std::string &name, control_toolbox::Pid &pid,
                   ros::NodeHandle &controller_nh, double def_p, double def_i,
                   double def_d, double def_i_clamp, double def_antiwindup);
    
    /**
     * 初始化单个轮速 PID
     * @param name: 轮速名称（如 "wheel_fl"）
     * @param pid: PID 控制器对象引用
     * @param controller_nh: 控制器命名空间句柄
     * @param def_wp/wi/wd/wi_clamp/wanti: 默认 PID 参数
     */
    void initWheel(const std::string &name, control_toolbox::Pid &pid,
                   ros::NodeHandle &controller_nh, double def_wp, double def_wi,
                   double def_wd, double def_wi_clamp, double def_wanti);

    // ==================== 辅助函数 ====================
    /**
     * 发布里程计消息（未使用的辅助函数，实际由 odom_update 替代）
     * @param stamp: 时间戳
     * @param vx_body/vy_body/wz: 机体坐标系速度
     */
    void publishOdometry_(const ros::Time &stamp, double vx_body, double vy_body, double wz);
    
    /**
     * 里程计更新与发布（主函数）
     * 功能：基于四轮反馈计算底盘速度与位姿，发布 TF 与 Odometry
     * 详细说明见文档：docs/里程计更新原理详解.md
     * @param time: 当前时间戳
     * @param period: 时间间隔（用于积分）
     */
    void odom_update(const ros::Time &time, const ros::Duration &period);
  };

} // namespace sentry_chassis_controller

#endif // SENTRY_CHASSIS_CONTROLLER_WHEEL_PID_CONTROLLER_H
