#include "sentry_chassis_controller/wheel_pid_controller.hpp"
#include "sentry_chassis_controller/power_limiter.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <sentry_chassis_controller/WheelPidConfig.h>
#include <sentry_chassis_controller/inverse_kinematics.hpp>

namespace sentry_chassis_controller
{
    bool WheelPidController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        controller_nh_ = controller_nh;
        try
        {
            // 速度轮关节
            front_left_wheel_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
            front_right_wheel_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
            back_left_wheel_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
            back_right_wheel_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

            // 舵向轮关节
            front_left_pivot_joint_ = effort_joint_interface->getHandle("left_front_pivot_joint");
            front_right_pivot_joint_ = effort_joint_interface->getHandle("right_front_pivot_joint");
            back_left_pivot_joint_ = effort_joint_interface->getHandle("left_back_pivot_joint");
            back_right_pivot_joint_ = effort_joint_interface->getHandle("right_back_pivot_joint");
        }
        catch (const hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR("WheelPidController: Exception getting joint handle: %s", ex.what());
            return false;
        }

        // 读取运动学参数
        controller_nh.param("wheel_track", wheel_track_, 0.36);   // 轮距（左右轮间距）
        controller_nh.param("wheel_base", wheel_base_, 0.36);     // 轴距（前后轮间距）
        controller_nh.param("wheel_radius", wheel_radius_, 0.05); // 轮子半径
        rx_ = wheel_track_ / 2.0;
        ry_ = wheel_base_ / 2.0;

        // 读取功率限制参数
        controller_nh.param("power_limit", power_limit_, 0.0);
        controller_nh.param("power/velocity_coeff", velocity_coeff_, 0.0);
        controller_nh.param("power/effort_coeff", effort_coeff_, 0.0);
        controller_nh.param("power/power_offset", power_offset_, 0.0);
        power_limit_enabled_ = (effort_coeff_ != 0.0 || velocity_coeff_ != 0.0);

        // 初始化 PID
        double def_p, def_i, def_d, def_i_clamp, def_antwindup;
        // 读取舵向PID默认值
        controller_nh.param("pivot/p", def_p, 1.0);
        controller_nh.param("pivot/i", def_i, 0.0);
        controller_nh.param("pivot/d", def_d, 0.0);
        controller_nh.param("pivot/i_clamp", def_i_clamp, 0.0);
        controller_nh.param("pivot/antiwindup", def_antwindup, 0.0);
        double def_wp, def_wi, def_wd, def_wi_clamp, def_wanti;
        controller_nh.param("wheel/p", def_wp, 2.0);
        controller_nh.param("wheel/i", def_wi, 0.1);
        controller_nh.param("wheel/d", def_wd, 0.0);
        controller_nh.param("wheel/i_clamp", def_wi_clamp, 0.0);
        controller_nh.param("wheel/antiwindup", def_wanti, 0.0);

        // 初始化舵向PID（YAML中的名称为 pivot_fl, pivot_fr, pivot_rl, pivot_rr）
        initPivot("pivot_fl", pid_lf_, controller_nh, def_p, def_i, def_d, def_i_clamp, def_antwindup);
        initPivot("pivot_fr", pid_rf_, controller_nh, def_p, def_i, def_d, def_i_clamp, def_antwindup);
        initPivot("pivot_rl", pid_lb_, controller_nh, def_p, def_i, def_d, def_i_clamp, def_antwindup);
        initPivot("pivot_rr", pid_rb_, controller_nh, def_p, def_i, def_d, def_i_clamp, def_antwindup);

        // 初始化轮速PID（YAML中的名称为 wheel_fl, wheel_fr, wheel_rl, wheel_rr）
        initWheel("wheel_fl", pid_lf_wheel_, controller_nh, def_wp, def_wi, def_wd, def_wi_clamp, def_wanti);
        initWheel("wheel_fr", pid_rf_wheel_, controller_nh, def_wp, def_wi, def_wd, def_wi_clamp, def_wanti);
        initWheel("wheel_rl", pid_lb_wheel_, controller_nh, def_wp, def_wi, def_wd, def_wi_clamp, def_wanti);
        initWheel("wheel_rr", pid_rb_wheel_, controller_nh, def_wp, def_wi, def_wd, def_wi_clamp, def_wanti);

        // ==================== 初始化轮速 PID 低通滤波器（解耦到 low_pass_filter 模块） ====================
        // 读取低通滤波器时间常数
        double def_lpf_tau = 0.02;
        controller_nh.param("wheel/lpf_tau", def_lpf_tau, 0.02);

        double lpf_tau_fl, lpf_tau_fr, lpf_tau_rl, lpf_tau_rr;
        controller_nh.param("wheels/wheel_fl/lpf_tau", lpf_tau_fl, def_lpf_tau);
        controller_nh.param("wheels/wheel_fr/lpf_tau", lpf_tau_fr, def_lpf_tau);
        controller_nh.param("wheels/wheel_rl/lpf_tau", lpf_tau_rl, def_lpf_tau);
        controller_nh.param("wheels/wheel_rr/lpf_tau", lpf_tau_rr, def_lpf_tau);

        // 设置各通道的时间常数
        wheel_lpf_.set_tau(0, lpf_tau_fl);
        wheel_lpf_.set_tau(1, lpf_tau_fr);
        wheel_lpf_.set_tau(2, lpf_tau_rl);
        wheel_lpf_.set_tau(3, lpf_tau_rr);
        // 初始化控制命令值
        for (int i = 0; i < 4; ++i)
        {
            pivot_cmd_[i] = 0.0;
            wheel_cmd_[i] = 0.0;
        }

        // 订阅 cmd_vel 以获取期望的底盘速度
        cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &WheelPidController::cmdVelCallback, this);

        // 发布器，用于暴露期望的轮子/舵机命令，便于测试/检查
        desired_pub_ = root_nh.advertise<sensor_msgs::JointState>("desired_wheel_states", 1);

        // 设置动态重配置服务器，允许在运行时调整每个轮子的 PID 参数
        /* inline dynamic_reconfigure::Server<sentry_chassis_controller::WheelPidController::Config>::Server(const ros::NodeHandle &nh) */
        dyn_server_.reset(new dynamic_reconfigure::Server<Config>(controller_nh));

        // 动态参数回调函数中接收两个参数：配置对象和级别掩码
        dynamic_reconfigure::Server<Config>::CallbackType cb = boost::bind(&WheelPidController::reconfigureCallback, this, _1, _2);

        // 注册回调函数
        dyn_server_->setCallback(cb);

        // 发布轮子关节的状态，为正运动学解算提供支持
        joint_states_pub_ = root_nh.advertise<sensor_msgs::JointState>("joint_states", 1);
        last_state_pub_ = ros::Time(0);

        // 应用的力矩和功率限制功能调试发布器
        applied_effort_pub_ = root_nh.advertise<sensor_msgs::JointState>("applied_wheel_efforts", 1); // 力矩传感器输出
        power_debug_pub_ = root_nh.advertise<std_msgs::Float64MultiArray>("power_debug", 1);          // 发布功率调试信息
        last_effort_pub_ = ros::Time(0);
        controller_nh.param("power_debug", power_debug_enabled_, false); // 是否启用功率调试发布

        // ==================== PID 调试发布器初始化（用于 rqt_plot） ====================
        controller_nh.param("pid_debug", pid_debug_enabled_, true); // 默认启用 PID 调试
        if (pid_debug_enabled_)
        {
            // 舵轮期望位置话题
            pivot_desired_fl_pub_ = root_nh.advertise<std_msgs::Float64>("pivot_debug/fl/desired", 1);
            pivot_desired_fr_pub_ = root_nh.advertise<std_msgs::Float64>("pivot_debug/fr/desired", 1);
            pivot_desired_rl_pub_ = root_nh.advertise<std_msgs::Float64>("pivot_debug/rl/desired", 1);
            pivot_desired_rr_pub_ = root_nh.advertise<std_msgs::Float64>("pivot_debug/rr/desired", 1);
            // 舵轮实际位置话题
            pivot_actual_fl_pub_ = root_nh.advertise<std_msgs::Float64>("pivot_debug/fl/actual", 1);
            pivot_actual_fr_pub_ = root_nh.advertise<std_msgs::Float64>("pivot_debug/fr/actual", 1);
            pivot_actual_rl_pub_ = root_nh.advertise<std_msgs::Float64>("pivot_debug/rl/actual", 1);
            pivot_actual_rr_pub_ = root_nh.advertise<std_msgs::Float64>("pivot_debug/rr/actual", 1);
            // 舵轮误差话题
            pivot_error_fl_pub_ = root_nh.advertise<std_msgs::Float64>("pivot_debug/fl/error", 1);
            pivot_error_fr_pub_ = root_nh.advertise<std_msgs::Float64>("pivot_debug/fr/error", 1);
            pivot_error_rl_pub_ = root_nh.advertise<std_msgs::Float64>("pivot_debug/rl/error", 1);
            pivot_error_rr_pub_ = root_nh.advertise<std_msgs::Float64>("pivot_debug/rr/error", 1);
            ROS_INFO("PID debug publishers initialized under /pivot_debug/*");
        }

        // ==================== 底盘自锁功能参数（几何自锁） ====================
        GeoLockConfig geo_config;
        controller_nh.param("self_lock/enabled", geo_config.enabled, true);
        controller_nh.param("self_lock/idle_timeout", geo_config.idle_timeout, 0.5);
        controller_nh.param("self_lock/geo_lock_wheel_brake", geo_config.wheel_brake, true);
        controller_nh.param("self_lock/velocity_deadband", geo_config.velocity_deadband, 0.05);
        controller_nh.param("self_lock/lock_pos_p", geo_config.lock_pos_p, 8.0);
        controller_nh.param("self_lock/lock_pos_d", geo_config.lock_pos_d, 1.0);
        controller_nh.param("self_lock/max_lock_torque", geo_config.max_lock_torque, 30.0);
        
        // 底盘尺寸参数（复用已有配置）
        geo_config.wheel_base = wheel_base_;
        geo_config.wheel_track = wheel_track_;
        
        // 初始化几何自锁模块
        geo_lock_ = std::make_shared<GeoLock>(geo_config);
        
        last_cmd_time_ = ros::Time::now(); // 初始化命令时间戳        // 里程计发布集成，支持不同速度模式下的里程计计算
        controller_nh.param<std::string>("odom_frame", odom_frame_, odom_frame_);                // 里程计坐标系
        controller_nh.param<std::string>("base_link_frame", base_link_frame_, base_link_frame_); // 底盘
        controller_nh.param<std::string>("speed_mode", speed_mode_, speed_mode_);                // 速度模式：local 或 global
        controller_nh.param<bool>("publish_tf", publish_tf_, true);                              // 是否发布 TF（使用 ground truth 时设为 false）
        odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("odom_controller", 10);                // 里程计发布器
        last_odom_time_ = ros::Time(0);

        // 舵轮同步检查参数（防止舵轮未到位就驱动导致的偏航问题）
        controller_nh.param("pivot_sync/enabled", pivot_sync_enabled_, true);     // 启用舵轮同步检查
        controller_nh.param("pivot_sync/threshold", pivot_sync_threshold_, 0.15); // 舵角同步阈值（rad，约8.6°）
        controller_nh.param("pivot_sync/scale_min", pivot_sync_scale_min_, 0.1);  // 最小轮速缩放（0.1 = 10%）
        ROS_INFO("Pivot sync: %s, threshold=%.3f rad (%.1f°), scale_min=%.2f",
                 pivot_sync_enabled_ ? "enabled" : "disabled",
                 pivot_sync_threshold_, pivot_sync_threshold_ * 180.0 / M_PI, pivot_sync_scale_min_);

        // 初始化 TF 监听器（作为类成员，持续缓存 TF 数据，避免每次回调重新创建）
        tf_listener_ = std::make_shared<tf::TransformListener>();

        ROS_INFO("WheelPidController initialized (enhanced state feedback)");
        ROS_INFO("Velocity mode: %s (local=base_link, global=odom), TF publish: %s",
                 speed_mode_.c_str(), publish_tf_ ? "enabled" : "disabled");
        return true;
    }

    /*
     * 动态重配置回调函数
     * 用于在运行时更新 PID 参数和低通滤波器时间常数
     * @param config: 配置对象
     * @param level: 级别掩码
     */
    void WheelPidController::reconfigureCallback(Config &config, uint32_t level)
    {
        // 更新轮速 PIDs
        pid_lf_wheel_.initPid(config.wheel_fl_p, config.wheel_fl_i, config.wheel_fl_d, config.wheel_fl_i_clamp, 0.0);
        pid_rf_wheel_.initPid(config.wheel_fr_p, config.wheel_fr_i, config.wheel_fr_d, config.wheel_fr_i_clamp, 0.0);
        pid_lb_wheel_.initPid(config.wheel_rl_p, config.wheel_rl_i, config.wheel_rl_d, config.wheel_rl_i_clamp, 0.0);
        pid_rb_wheel_.initPid(config.wheel_rr_p, config.wheel_rr_i, config.wheel_rr_d, config.wheel_rr_i_clamp, 0.0);

        // 更新轮速 PID 低通滤波器时间常数（解耦到 low_pass_filter 模块）
        wheel_lpf_.set_tau(0, config.wheel_fl_lpf_tau);
        wheel_lpf_.set_tau(1, config.wheel_fr_lpf_tau);
        wheel_lpf_.set_tau(2, config.wheel_rl_lpf_tau);
        wheel_lpf_.set_tau(3, config.wheel_rr_lpf_tau);

        // 更新舵向电机 PIDs
        pid_lf_.initPid(config.pivot_fl_p, config.pivot_fl_i, config.pivot_fl_d, config.pivot_fl_i_clamp, 0.0);
        pid_rf_.initPid(config.pivot_fr_p, config.pivot_fr_i, config.pivot_fr_d, config.pivot_fr_i_clamp, 0.0);
        pid_lb_.initPid(config.pivot_rl_p, config.pivot_rl_i, config.pivot_rl_d, config.pivot_rl_i_clamp, 0.0);
        pid_rb_.initPid(config.pivot_rr_p, config.pivot_rr_i, config.pivot_rr_d, config.pivot_rr_i_clamp, 0.0);

        // 把动态调整的PID参数镜像回 wheels/* 结构，方便通过 rosparam get 获取
        // 函数对象，用于设置单个轮子的 PID 参数（包含 LPF 时间常数）
        auto setWheel = [this](const std::string &name, double p, double i, double d, double i_clamp, double lpf_tau)
        {
            // 参数的命名空间
            const std::string base = std::string("wheels/") + name;

            // 设置参数
            controller_nh_.setParam(base + "/p", p);
            controller_nh_.setParam(base + "/i", i);
            controller_nh_.setParam(base + "/d", d);
            controller_nh_.setParam(base + "/i_clamp", i_clamp);
            controller_nh_.setParam(base + "/lpf_tau", lpf_tau);
        };

        // 设置轮速 PID 参数（包含 LPF）
        setWheel("wheel_fl", config.wheel_fl_p, config.wheel_fl_i, config.wheel_fl_d, config.wheel_fl_i_clamp, config.wheel_fl_lpf_tau);
        setWheel("wheel_fr", config.wheel_fr_p, config.wheel_fr_i, config.wheel_fr_d, config.wheel_fr_i_clamp, config.wheel_fr_lpf_tau);
        setWheel("wheel_rl", config.wheel_rl_p, config.wheel_rl_i, config.wheel_rl_d, config.wheel_rl_i_clamp, config.wheel_rl_lpf_tau);
        setWheel("wheel_rr", config.wheel_rr_p, config.wheel_rr_i, config.wheel_rr_d, config.wheel_rr_i_clamp, config.wheel_rr_lpf_tau);

        // 函数对象，用于设置单个舵向电机的 PID 参数
        auto setPivot = [this](const std::string &name, double p, double i, double d, double i_clamp)
        {
            const std::string base = std::string("wheels/") + name;
            controller_nh_.setParam(base + "/p", p);
            controller_nh_.setParam(base + "/i", i);
            controller_nh_.setParam(base + "/d", d);
            controller_nh_.setParam(base + "/i_clamp", i_clamp);
        };

        // 设置舵向电机 PID 参数
        setPivot("pivot_fl", config.pivot_fl_p, config.pivot_fl_i, config.pivot_fl_d, config.pivot_fl_i_clamp);
        setPivot("pivot_fr", config.pivot_fr_p, config.pivot_fr_i, config.pivot_fr_d, config.pivot_fr_i_clamp);
        setPivot("pivot_rl", config.pivot_rl_p, config.pivot_rl_i, config.pivot_rl_d, config.pivot_rl_i_clamp);
        setPivot("pivot_rr", config.pivot_rr_p, config.pivot_rr_i, config.pivot_rr_d, config.pivot_rr_i_clamp);

        ROS_INFO("WheelPidController: dynamic_reconfigure updated PID gains and LPF tau [%.3f, %.3f, %.3f, %.3f]",
                 config.wheel_fl_lpf_tau, config.wheel_fr_lpf_tau,
                 config.wheel_rl_lpf_tau, config.wheel_rr_lpf_tau);
    }

    void WheelPidController::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
    {
        // 更新命令时间戳（用于自锁功能的空闲检测）
        last_cmd_time_ = ros::Time::now();

        // 通过逆向运动学计算每个轮子的期望舵角和轮子的期望角速度
        double vx = msg->linear.x;
        double vy = msg->linear.y;
        double wz = msg->angular.z;
        if (speed_mode_ == "global")
        {
            // 将速度从全局（odom）坐标系转换到局部（base_link）坐标系
            // 使用成员变量 tf_listener_，它持续缓存 TF 数据
            try
            {
                geometry_msgs::Vector3Stamped vel_global, vel_local; // 全局和局部速度向量
                vel_global.header.frame_id = odom_frame_;            // 全局坐标系
                vel_global.header.stamp = ros::Time(0);              // 使用最新的变换
                vel_global.vector.x = vx;                            // 速度在全局坐标系中的x分量
                vel_global.vector.y = vy;                            // 速度在全局坐标系中的y分量
                vel_global.vector.z = 0.0;

                // 使用缓存的 TF 变换（ros::Time(0) 表示最新可用的变换）
                if (tf_listener_->canTransform(base_link_frame_, odom_frame_, ros::Time(0)))
                {
                    /*
                    Transform a Stamped Vector Message into the target frame This can throw all that lookupTransform can throw as well as tf::InvalidTransform
                    */
                    /*
                    void tf::TransformListener::transformVector(const std::string &target_frame, const geometry_msgs::Vector3Stamped &stamped_in, geometry_msgs::Vector3Stamped &stamped_out) const
                    */
                    tf_listener_->transformVector(base_link_frame_, vel_global, vel_local);

                    // 使用转换后的速度
                    vx = vel_local.vector.x;
                    vy = vel_local.vector.y;

                    ROS_DEBUG_THROTTLE(1.0, "Global mode: odom_vel(%.2f,%.2f) -> body_vel(%.2f,%.2f)",
                                       msg->linear.x, msg->linear.y, vx, vy);
                }
                else
                {
                    ROS_WARN_THROTTLE(2.0, "Global mode: TF not available (odom->base_link), using raw velocity");
                }
            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN_THROTTLE(2.0, "Global mode TF transform error: %s", ex.what());
                // Fallback: use original command (assume body frame)
            }
        }
        // 否则speed_mode_ == "local"，使用原始速度（已在 base_link 坐标系中）

        // 创建逆运动学对象并计算轮速和舵角
        auto ik = sentry_kinematics::inverseKinematics(vx, vy, wz, wheel_base_, wheel_track_, wheel_radius_);

        // 把计算结果存储为期望命令
        for (int i = 0; i < 4; ++i)
        {
            wheel_cmd_[i] = ik.wheel_angular_vel[i];
            pivot_cmd_[i] = ik.steer_angle[i];
        }

        // 发布期望命令以供检查
        sensor_msgs::JointState js; // 期望关节状态消息
        js.header.stamp = ros::Time::now();
        js.name = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"};
        js.position.resize(4);
        js.velocity.resize(4);
        for (int i = 0; i < 4; ++i)
        {
            js.position[i] = pivot_cmd_[i];
            js.velocity[i] = wheel_cmd_[i];
        }
        // 发布期望的轮速和舵角状态
        desired_pub_.publish(js);
    }
    void WheelPidController::initPivot(const std::string &name, control_toolbox::Pid &pid,
                                       ros::NodeHandle &controller_nh, double def_p, double def_i,
                                       double def_d, double def_i_clamp, double def_antwindup)
    {
        // 舵轮 PID 参数命名空间
        std::string base = "wheels/" + name;
        double p = def_p, i = def_i, d = def_d, i_clamp = def_i_clamp, antiwindup = def_antwindup;
        // 设置参数
        controller_nh.param(base + "/p", p, p);
        controller_nh.param(base + "/i", i, i);
        controller_nh.param(base + "/d", d, d);
        controller_nh.param(base + "/i_clamp", i_clamp, i_clamp);
        controller_nh.param(base + "/antiwindup", antiwindup, antiwindup);
        pid.initPid(p, i, d, i_clamp, antiwindup);
        ROS_INFO("WheelPidController: Init pivot PID '%s' p=%.3f i=%.3f d=%.3f i_clamp=%.3f", name.c_str(), p, i, d, i_clamp);
    }
    void WheelPidController::initWheel(const std::string &name, control_toolbox::Pid &pid,
                                       ros::NodeHandle &controller_nh, double def_wp, double def_wi,
                                       double def_wd, double def_wi_clamp, double def_wanti)
    {
        // 轮向电机 PID 命名空间
        std::string base = "wheels/" + name;
        // 设置参数
        double p = def_wp, i = def_wi, d = def_wd, i_clamp = def_wi_clamp, antiwindup = def_wanti;
        controller_nh.param(base + "/p", p, p);
        controller_nh.param(base + "/i", i, i);
        controller_nh.param(base + "/d", d, d);
        controller_nh.param(base + "/i_clamp", i_clamp, i_clamp);
        controller_nh.param(base + "/antiwindup", antiwindup, antiwindup);
        pid.initPid(p, i, d, i_clamp, antiwindup);
        ROS_INFO("WheelPidController: Init wheel PID '%s' p=%.3f i=%.3f d=%.3f i_clamp=%.3f", name.c_str(), p, i, d, i_clamp);
    }
    void WheelPidController::update(const ros::Time &time, const ros::Duration &period)
    {
        // 初始化要更新的参数
        double lf_wheel_pos = 0.0, rf_wheel_pos = 0.0, lb_wheel_pos = 0.0, rb_wheel_pos = 0.0;
        double lf_wheel_vel = 0.0, rf_wheel_vel = 0.0, lb_wheel_vel = 0.0, rb_wheel_vel = 0.0;

        try
        {
            // 获取轮向电机位置和速度
            lf_wheel_pos = front_left_wheel_joint_.getPosition();
            rf_wheel_pos = front_right_wheel_joint_.getPosition();
            lb_wheel_pos = back_left_wheel_joint_.getPosition();
            rb_wheel_pos = back_right_wheel_joint_.getPosition();

            lf_wheel_vel = front_left_wheel_joint_.getVelocity();
            rf_wheel_vel = front_right_wheel_joint_.getVelocity();
            lb_wheel_vel = back_left_wheel_joint_.getVelocity();
            rb_wheel_vel = back_right_wheel_joint_.getVelocity();

            // 调试日志
            static ros::Time last_log = ros::Time(0);
            if (time.toSec() - last_log.toSec() > 1.0) // Log every second
            {
                ROS_INFO_STREAM_THROTTLE(1.0, "Joint state:"
                << " LF pos=" << lf_wheel_pos << " v=" << lf_wheel_vel
                << " RF pos=" << rf_wheel_pos << " v=" << rf_wheel_vel
                << " LB pos=" << lb_wheel_pos << " v=" << lb_wheel_vel
                << " RB pos=" << rb_wheel_pos << " v=" << rb_wheel_vel);
                last_log = time;
            }
        }
        catch (const hardware_interface::HardwareInterfaceException &e)
        {
            ROS_ERROR_THROTTLE(1.0, "Failed to read joint states: %s", e.what());
        }

        // 确保非零状态（安全回退）
        // 如果所有位置都是零（模拟传感器故障），报告最小移动
        double min_movement = 0.001; // 最小非零值以防止除以零
        if (fabs(lf_wheel_pos) + fabs(rf_wheel_pos) + fabs(lb_wheel_pos) + fabs(rb_wheel_pos) < min_movement * 4)
        {
            // 提供最小的合成移动以保持里程计活跃（仅在有命令时）
            double synthetic_pos = min_movement * wheel_cmd_[0] * 0.01; // 基于命令的微小偏转
            lf_wheel_pos = synthetic_pos;
            rf_wheel_pos = synthetic_pos;
            lb_wheel_pos = synthetic_pos;
            rb_wheel_pos = synthetic_pos;
            ROS_WARN_THROTTLE(2.0, "All joint positions are zero! Using synthetic fallback values");
        }

        // 坑：发布实际关节名称对应关节的状态
        if (time.toSec() - last_state_pub_.toSec() > 0.1) // 10Hz 发布频率
        {
            sensor_msgs::JointState js;
            js.header.stamp = time;
            // 关节的名称不要搞错
            js.name = {"left_front_wheel_joint",
                       "right_front_wheel_joint",
                       "left_back_wheel_joint",
                       "right_back_wheel_joint",
                       "left_front_pivot_joint",
                       "right_front_pivot_joint",
                       "left_back_pivot_joint",
                       "right_back_pivot_joint"};
            js.position.resize(8);
            js.velocity.resize(8);

            // 轮子位置/速度
            js.position[0] = lf_wheel_pos;
            js.velocity[0] = lf_wheel_vel;
            js.position[1] = rf_wheel_pos;
            js.velocity[1] = rf_wheel_vel;
            js.position[2] = lb_wheel_pos;
            js.velocity[2] = lb_wheel_vel;
            js.position[3] = rb_wheel_pos;
            js.velocity[3] = rb_wheel_vel;

            try
            {
                js.position[4] = front_left_pivot_joint_.getPosition();
                js.position[5] = front_right_pivot_joint_.getPosition();
                js.position[6] = back_left_pivot_joint_.getPosition();
                js.position[7] = back_right_pivot_joint_.getPosition();
            }
            catch (...)
            {
                js.position[4] = 0.0;
                js.position[5] = 0.0;
                js.position[6] = 0.0;
                js.position[7] = 0.0;
            }

            // Check current published state via log
            ROS_DEBUG_STREAM_THROTTLE(1.0, "Publishing joint states: "
            << " wheel pos=" << js.position[0] << "," << js.position[1]
            << " vel=" << js.velocity[0] << "," << js.velocity[1]
            << " pivot pos=" << js.position[4]);
            joint_states_pub_.publish(js);
            last_state_pub_ = time;
        }

        // ==================== 底盘自锁逻辑（几何自锁） ====================
        // 准备输入数据
        GeoLockInput geo_input;
        geo_input.current_time = time;
        geo_input.last_cmd_time = last_cmd_time_;
        geo_input.wheel_positions[0] = lf_wheel_pos;
        geo_input.wheel_positions[1] = rf_wheel_pos;
        geo_input.wheel_positions[2] = lb_wheel_pos;
        geo_input.wheel_positions[3] = rb_wheel_pos;
        geo_input.wheel_velocities[0] = lf_wheel_vel;
        geo_input.wheel_velocities[1] = rf_wheel_vel;
        geo_input.wheel_velocities[2] = lb_wheel_vel;
        geo_input.wheel_velocities[3] = rb_wheel_vel;
        geo_input.pivot_positions[0] = front_left_pivot_joint_.getPosition();
        geo_input.pivot_positions[1] = front_right_pivot_joint_.getPosition();
        geo_input.pivot_positions[2] = back_left_pivot_joint_.getPosition();
        geo_input.pivot_positions[3] = back_right_pivot_joint_.getPosition();
        
        // 更新自锁状态
        GeoLockOutput geo_output;
        geo_lock_->update(geo_input, geo_output);
        
        // 如果处于自锁状态，使用自锁命令
        if (geo_output.is_locked)
        {
            // 设置舵角命令
            for (int i = 0; i < 4; ++i)
            {
                pivot_cmd_[i] = geo_output.pivot_commands[i];
            }
            
            // 设置轮子命令
            for (int i = 0; i < 4; ++i)
            {
                wheel_cmd_[i] = geo_output.wheel_commands[i];
            }
        }    

        // 计算轮子速度误差并应用 PID -> 力矩命令
        double cmd0, cmd1, cmd2, cmd3;

        // 自锁模式下（轮子刹车）直接使用预计算的力矩，绕过速度 PID
        if (geo_output.is_locked && geo_output.use_position_control)
        {
            // wheel_cmd_ 中存储的是位置锁定力矩（乘以1000作为标记）
            cmd0 = wheel_cmd_[0] / 1000.0;
            cmd1 = wheel_cmd_[1] / 1000.0;
            cmd2 = wheel_cmd_[2] / 1000.0;
            cmd3 = wheel_cmd_[3] / 1000.0;

            // Reset PID integrators to prevent spikes when unlocking
            pid_lf_wheel_.reset();
            pid_rf_wheel_.reset();
            pid_lb_wheel_.reset();
            pid_rb_wheel_.reset();

            ROS_DEBUG_THROTTLE(1.0, "GeoLock position control torque: [%.2f, %.2f, %.2f, %.2f] Nm", cmd0, cmd1, cmd2, cmd3);
        }
        else
        {
            // 正常模式：使用速度 PID 控制
            double raw_cmd0 = pid_lf_wheel_.computeCommand(wheel_cmd_[0] - lf_wheel_vel, period);
            double raw_cmd1 = pid_rf_wheel_.computeCommand(wheel_cmd_[1] - rf_wheel_vel, period);
            double raw_cmd2 = pid_lb_wheel_.computeCommand(wheel_cmd_[2] - lb_wheel_vel, period);
            double raw_cmd3 = pid_rb_wheel_.computeCommand(wheel_cmd_[3] - rb_wheel_vel, period);

            // 低通滤波器
            double dt = period.toSec();
            double raw_cmds[4] = {raw_cmd0, raw_cmd1, raw_cmd2, raw_cmd3};
            double filtered_cmds[4];

            wheel_lpf_.update(raw_cmds, filtered_cmds, dt);

            cmd0 = filtered_cmds[0];
            cmd1 = filtered_cmds[1];
            cmd2 = filtered_cmds[2];
            cmd3 = filtered_cmds[3];

            // 舵轮同步检查防止偏航
            // 问题：舵轮未到位时驱动轮子会产生错误的推力方向，导致车辆偏航或打滑
            // 方案：根据舵角误差动态缩放轮速命令
            //       - 舵角误差小于阈值：全功率驱动
            //       - 舵角误差较大：按比例降低轮速（最低到 scale_min）
            if (pivot_sync_enabled_)
            {
                // 读取当前舵角位置
                double pivot_pos[4];
                try
                {
                    pivot_pos[0] = front_left_pivot_joint_.getPosition();
                    pivot_pos[1] = front_right_pivot_joint_.getPosition();
                    pivot_pos[2] = back_left_pivot_joint_.getPosition();
                    pivot_pos[3] = back_right_pivot_joint_.getPosition();
                }
                catch (...)
                {
                    pivot_pos[0] = pivot_pos[1] = pivot_pos[2] = pivot_pos[3] = 0.0;
                }

                // 计算每个舵轮的角度误差（归一化到 [-π, π]）
                double pivot_errors[4];
                double max_error = 0.0;
                for (int i = 0; i < 4; ++i)
                {
                    double err = pivot_cmd_[i] - pivot_pos[i];
                    // 归一化到 [-π, π]
                    while (err > M_PI)
                        err -= 2.0 * M_PI;
                    while (err < -M_PI)
                        err += 2.0 * M_PI;
                    pivot_errors[i] = std::abs(err);
                    if (pivot_errors[i] > max_error)
                        max_error = pivot_errors[i];
                }

                // 基于最大舵角误差计算轮速缩放因子
                // 误差 <= threshold: scale = 1.0（全功率）
                // 误差 > threshold: scale 线性衰减到 scale_min
                double pivot_sync_scale = 1.0;
                if (max_error > pivot_sync_threshold_)
                {
                    // 线性插值：误差从 threshold 增加到 π 时，scale 从 1.0 降到 scale_min
                    double t = (max_error - pivot_sync_threshold_) / (M_PI - pivot_sync_threshold_);
                    t = std::min(1.0, std::max(0.0, t)); // clamp to [0, 1]
                    pivot_sync_scale = 1.0 - t * (1.0 - pivot_sync_scale_min_);

                    ROS_DEBUG_THROTTLE(0.2, "Pivot sync: max_err=%.2f° > threshold, scale=%.2f",
                                       max_error * 180.0 / M_PI, pivot_sync_scale);
                }

                // 应用缩放
                cmd0 *= pivot_sync_scale;
                cmd1 *= pivot_sync_scale;
                cmd2 *= pivot_sync_scale;
                cmd3 *= pivot_sync_scale;
            }
        }

        // 应用功率限制
        PowerLimiterConfig power_config;
        power_config.power_limit = power_limit_;
        power_config.velocity_coeff = velocity_coeff_;
        power_config.effort_coeff = effort_coeff_;
        power_config.power_offset = power_offset_;
        power_config.enabled = power_limit_enabled_;
        power_config.debug_enabled = power_debug_enabled_;

        PowerLimiterInput power_input;
        power_input.cmd[0] = cmd0;
        power_input.cmd[1] = cmd1;
        power_input.cmd[2] = cmd2;
        power_input.cmd[3] = cmd3;
        power_input.vel[0] = lf_wheel_vel;
        power_input.vel[1] = rf_wheel_vel;
        power_input.vel[2] = lb_wheel_vel;
        power_input.vel[3] = rb_wheel_vel;

        PowerLimiterOutput power_output;
        apply_power_limit(power_config, power_input, power_output, &power_debug_pub_);

        // 使用功率限制后的力矩命令
        cmd0 = power_output.cmd[0];
        cmd1 = power_output.cmd[1];
        cmd2 = power_output.cmd[2];
        cmd3 = power_output.cmd[3];

        // 发送应用功率控制后的力矩命令到轮子关节
        front_left_wheel_joint_.setCommand(cmd0);
        front_right_wheel_joint_.setCommand(cmd1);
        back_left_wheel_joint_.setCommand(cmd2);
        back_right_wheel_joint_.setCommand(cmd3);

        // 读取舵轮当前位置
        double pivot_pos_fl = front_left_pivot_joint_.getPosition();
        double pivot_pos_fr = front_right_pivot_joint_.getPosition();
        double pivot_pos_rl = back_left_pivot_joint_.getPosition();
        double pivot_pos_rr = back_right_pivot_joint_.getPosition();

        // 计算舵角误差
        double pivot_err_fl = pivot_cmd_[0] - pivot_pos_fl;
        double pivot_err_fr = pivot_cmd_[1] - pivot_pos_fr;
        double pivot_err_rl = pivot_cmd_[2] - pivot_pos_rl;
        double pivot_err_rr = pivot_cmd_[3] - pivot_pos_rr;

        // 计算舵角 PID 输出
        double p0 = pid_lf_.computeCommand(pivot_err_fl, period);
        double p1 = pid_rf_.computeCommand(pivot_err_fr, period);
        double p2 = pid_lb_.computeCommand(pivot_err_rl, period);
        double p3 = pid_rb_.computeCommand(pivot_err_rr, period);

        // 发送舵角力矩命令到关节
        front_left_pivot_joint_.setCommand(p0);
        front_right_pivot_joint_.setCommand(p1);
        back_left_pivot_joint_.setCommand(p2);
        back_right_pivot_joint_.setCommand(p3);

        // ==================== PID 调试话题发布（用于 rqt_plot） ====================
        if (pid_debug_enabled_)
        {
            std_msgs::Float64 msg;

            // 发布期望位置（单位：rad）
            msg.data = pivot_cmd_[0];
            pivot_desired_fl_pub_.publish(msg);
            msg.data = pivot_cmd_[1];
            pivot_desired_fr_pub_.publish(msg);
            msg.data = pivot_cmd_[2];
            pivot_desired_rl_pub_.publish(msg);
            msg.data = pivot_cmd_[3];
            pivot_desired_rr_pub_.publish(msg);

            // 发布实际位置（单位：rad）
            msg.data = pivot_pos_fl;
            pivot_actual_fl_pub_.publish(msg);
            msg.data = pivot_pos_fr;
            pivot_actual_fr_pub_.publish(msg);
            msg.data = pivot_pos_rl;
            pivot_actual_rl_pub_.publish(msg);
            msg.data = pivot_pos_rr;
            pivot_actual_rr_pub_.publish(msg);

            // 发布误差（单位：rad）
            msg.data = pivot_err_fl;
            pivot_error_fl_pub_.publish(msg);
            msg.data = pivot_err_fr;
            pivot_error_fr_pub_.publish(msg);
            msg.data = pivot_err_rl;
            pivot_error_rl_pub_.publish(msg);
            msg.data = pivot_err_rr;
            pivot_error_rr_pub_.publish(msg);
        }

        // 以约10Hz频率发布应用的力矩
        if (time.toSec() - last_effort_pub_.toSec() > 0.1)
        {
            sensor_msgs::JointState js_eff;
            js_eff.header.stamp = time;
            js_eff.name = {"left_front_wheel_joint",
                           "right_front_wheel_joint",
                           "left_back_wheel_joint",
                           "right_back_wheel_joint"};
            js_eff.effort.resize(4);
            js_eff.effort[0] = front_left_wheel_joint_.getCommand();
            js_eff.effort[1] = front_right_wheel_joint_.getCommand();
            js_eff.effort[2] = back_left_wheel_joint_.getCommand();
            js_eff.effort[3] = back_right_wheel_joint_.getCommand();
            applied_effort_pub_.publish(js_eff);
            last_effort_pub_ = time;
        }

        // 封装的里程计更新与发布（解耦到 OdomUpdater 模块）
        {
            // 准备输入数据
            OdomUpdaterInput odom_input;
            odom_input.wheel_velocities[0] = front_left_wheel_joint_.getVelocity();
            odom_input.wheel_velocities[1] = front_right_wheel_joint_.getVelocity();
            odom_input.wheel_velocities[2] = back_left_wheel_joint_.getVelocity();
            odom_input.wheel_velocities[3] = back_right_wheel_joint_.getVelocity();

            odom_input.pivot_angles[0] = front_left_pivot_joint_.getPosition();
            odom_input.pivot_angles[1] = front_right_pivot_joint_.getPosition();
            odom_input.pivot_angles[2] = back_left_pivot_joint_.getPosition();
            odom_input.pivot_angles[3] = back_right_pivot_joint_.getPosition();

            odom_input.wheel_base = wheel_base_;
            odom_input.wheel_track = wheel_track_;
            odom_input.wheel_radius = wheel_radius_;
            odom_input.odom_frame = odom_frame_;
            odom_input.base_link_frame = base_link_frame_;
            odom_input.publish_tf = publish_tf_;
            odom_input.velocity_deadband = geo_lock_->get_config().velocity_deadband;

            // 调用里程计更新器
            OdomUpdaterOutput odom_output;
            odom_updater_.update(odom_input, time, period, odom_pub_, odom_output);
        }
    }

    // 注册插件
    PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::WheelPidController, controller_interface::ControllerBase)

}
