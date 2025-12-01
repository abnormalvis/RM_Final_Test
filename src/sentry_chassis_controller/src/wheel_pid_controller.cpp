#include "sentry_chassis_controller/wheel_pid_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <sentry_chassis_controller/WheelPidConfig.h>
#include <sentry_chassis_controller/inverse_kinematics.hpp>

namespace sentry_chassis_controller
{

    /*
     * 初始化函数 init()
     * 初始化的参数包含：
     * 关节句柄获取
     * 运动学参数读取
     * 功率限制参数读取
     * PID 参数初始化
     * 订阅器和发布器初始化
     * 动态重配置服务器初始化
     * 里程计发布器初始化
     */
    bool WheelPidController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        // 存储控制器命名空间句柄以便后续参数设置
        controller_nh_ = controller_nh;
        // 获取关节句柄（名称可通过 ROS 参数覆盖）
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

        // 读取功率限制参数（可选）
        controller_nh.param("power_limit", power_limit_, 0.0);
        controller_nh.param("power/velocity_coeff", velocity_coeff_, 0.0);
        controller_nh.param("power/effort_coeff", effort_coeff_, 0.0);
        controller_nh.param("power/power_offset", power_offset_, 0.0);
        power_limit_enabled_ = (effort_coeff_ != 0.0 || velocity_coeff_ != 0.0);

        // 使用参数（如果存在）或默认值初始化 PID
        double def_p, def_i, def_d, def_i_clamp, def_antwindup;
        // 读取舵向PID默认值
        controller_nh.param("pivot/p", def_p, 1.0);
        controller_nh.param("pivot/i", def_i, 0.0);
        controller_nh.param("pivot/d", def_d, 0.0);
        controller_nh.param("pivot/i_clamp", def_i_clamp, 0.0);
        controller_nh.param("pivot/antiwindup", def_antwindup, 0.0);

        // 读取轮速PID默认值
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

        // 初始化控制命令值
        for (int i = 0; i < 4; ++i)
        {
            pivot_cmd_[i] = 0.0;
            wheel_cmd_[i] = 0.0;
        }

        // 订阅 cmd_vel 以获取期望的底盘速度（底盘坐标系）
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

        // 底盘自锁功能参数
        controller_nh.param("self_lock/enabled", self_lock_enabled_, true);                // 自锁功能开关
        controller_nh.param("self_lock/idle_timeout", idle_timeout_, 0.5);                 // 空闲超时（秒）
        controller_nh.param("self_lock/velocity_deadband", odom_velocity_deadband_, 0.05); // 里程计速度死区（增大）

        // 位置锁定模式参数
        controller_nh.param("self_lock/lock_position", lock_pos_enabled_, true); // 位置锁定模式开关
        controller_nh.param("self_lock/lock_pos_p", lock_pos_p_, 8.0);           // 位置锁定 P 增益
        controller_nh.param("self_lock/lock_pos_i", lock_pos_i_, 0.0);           // 位置锁定 I 增益
        controller_nh.param("self_lock/lock_pos_d", lock_pos_d_, 1.0);           // 位置锁定 D 增益
        
        // ==================== 几何自锁模式参数 ====================
        // 几何自锁：将舵轮转到底盘自转切向方向，形成"X"字布局
        // 原理：轮子滚动方向与任意平移方向垂直，外力只能推动轮子侧滑（滑动摩擦 >> 滚动摩擦）
        controller_nh.param("self_lock/geo_lock_enabled", geo_lock_enabled_, false);     // 几何自锁开关
        controller_nh.param("self_lock/geo_lock_wheel_brake", geo_lock_wheel_brake_, true); // 几何自锁时是否锁定轮子
        
        // 预计算几何自锁舵角（基于轮子相对底盘中心的位置）
        // 轮子布局（右手系：x向前，y向左）：
        //   FL: (+wheel_base/2, +wheel_track/2)  →  切向角 = atan2(+rx, -ry) = atan2(+, -)
        //   FR: (+wheel_base/2, -wheel_track/2)  →  切向角 = atan2(+rx, -ry) = atan2(+, +)
        //   RL: (-wheel_base/2, +wheel_track/2)  →  切向角 = atan2(-rx, -ry) = atan2(-, -)
        //   RR: (-wheel_base/2, -wheel_track/2)  →  切向角 = atan2(-rx, -ry) = atan2(-, +)
        // 切向方向：从轮子位置绕底盘中心逆时针旋转90度的方向
        // 公式：θ = atan2(rx, -ry)  （使轮子滚动方向沿自转切线）
        double rx = wheel_base_ / 2.0;   // 轴距半值
        double ry = wheel_track_ / 2.0;  // 轮距半值
        
        // FL: (+rx, +ry) → 切向 = atan2(+rx, -ry)
        geo_lock_angles_[0] = std::atan2(rx, -ry);   // 约 -0.785 rad (-45°)
        // FR: (+rx, -ry) → 切向 = atan2(+rx, +ry)  
        geo_lock_angles_[1] = std::atan2(rx, ry);    // 约 +0.785 rad (+45°)
        // RL: (-rx, +ry) → 切向 = atan2(-rx, -ry)
        geo_lock_angles_[2] = std::atan2(-rx, -ry);  // 约 -2.356 rad (-135°) 或 +2.356
        // RR: (-rx, -ry) → 切向 = atan2(-rx, +ry)
        geo_lock_angles_[3] = std::atan2(-rx, ry);   // 约 +2.356 rad (+135°) 或 -0.785
        
        ROS_INFO("Geo-lock pivot angles: FL=%.2f° FR=%.2f° RL=%.2f° RR=%.2f°",
                 geo_lock_angles_[0] * 180.0 / M_PI, geo_lock_angles_[1] * 180.0 / M_PI,
                 geo_lock_angles_[2] * 180.0 / M_PI, geo_lock_angles_[3] * 180.0 / M_PI);

        last_cmd_time_ = ros::Time::now(); // 初始化命令时间戳
        is_locked_ = false;                // 初始状态：未锁定
        for (int i = 0; i < 4; ++i)
        {
            locked_pivot_pos_[i] = 0.0; // 初始锁定舵角为0
            locked_wheel_pos_[i] = 0.0; // 初始锁定轮子位置为0
        }
        ROS_INFO("Self-lock: %s, idle_timeout=%.2fs, velocity_deadband=%.3f",
                 self_lock_enabled_ ? "enabled" : "disabled", idle_timeout_, odom_velocity_deadband_);
        ROS_INFO("  Position lock=%s (P=%.1f D=%.2f), Geo-lock=%s (wheel_brake=%s)",
                 lock_pos_enabled_ ? "enabled" : "disabled", lock_pos_p_, lock_pos_d_,
                 geo_lock_enabled_ ? "enabled" : "disabled", geo_lock_wheel_brake_ ? "yes" : "no");

        // 里程计发布集成，支持不同速度模式下的里程计计算
        controller_nh.param<std::string>("odom_frame", odom_frame_, odom_frame_);                // 里程计坐标系
        controller_nh.param<std::string>("base_link_frame", base_link_frame_, base_link_frame_); // 底盘
        controller_nh.param<std::string>("speed_mode", speed_mode_, speed_mode_);                // 速度模式：local 或 global
        odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("odom_controller", 10);                // 里程计发布器
        last_odom_time_ = ros::Time(0);

        ROS_INFO("WheelPidController initialized (enhanced state feedback)");
        ROS_INFO("Velocity mode: %s (local=base_link, global=odom)", speed_mode_.c_str());
        return true;
    }

    /*
     * 动态重配置回调函数
     * 用于在运行时更新 PID 参数
     * @param config: 配置对象
     * @param level: 级别掩码
     */
    void WheelPidController::reconfigureCallback(Config &config, uint32_t level)
    {
        // 更新轮向点击 PIDs
        pid_lf_wheel_.initPid(config.wheel_fl_p, config.wheel_fl_i, config.wheel_fl_d, config.wheel_fl_i_clamp, 0.0);
        pid_rf_wheel_.initPid(config.wheel_fr_p, config.wheel_fr_i, config.wheel_fr_d, config.wheel_fr_i_clamp, 0.0);
        pid_lb_wheel_.initPid(config.wheel_rl_p, config.wheel_rl_i, config.wheel_rl_d, config.wheel_rl_i_clamp, 0.0);
        pid_rb_wheel_.initPid(config.wheel_rr_p, config.wheel_rr_i, config.wheel_rr_d, config.wheel_rr_i_clamp, 0.0);

        // 更新舵向电机 PIDs
        pid_lf_.initPid(config.pivot_fl_p, config.pivot_fl_i, config.pivot_fl_d, config.pivot_fl_i_clamp, 0.0);
        pid_rf_.initPid(config.pivot_fr_p, config.pivot_fr_i, config.pivot_fr_d, config.pivot_fr_i_clamp, 0.0);
        pid_lb_.initPid(config.pivot_rl_p, config.pivot_rl_i, config.pivot_rl_d, config.pivot_rl_i_clamp, 0.0);
        pid_rb_.initPid(config.pivot_rr_p, config.pivot_rr_i, config.pivot_rr_d, config.pivot_rr_i_clamp, 0.0);

        // 把动态调整的PID参数镜像回 wheels/* 结构，方便通过 rosparam get 获取
        // 函数对象，用于设置单个轮子的 PID 参数
        auto setWheel = [this](const std::string &name, double p, double i, double d, double i_clamp)
        {
            // 参数的命名空间
            const std::string base = std::string("wheels/") + name;

            // 设置参数
            controller_nh_.setParam(base + "/p", p);
            controller_nh_.setParam(base + "/i", i);
            controller_nh_.setParam(base + "/d", d);
            controller_nh_.setParam(base + "/i_clamp", i_clamp);
        };

        // 设置轮速 PID 参数
        setWheel("wheel_fl", config.wheel_fl_p, config.wheel_fl_i, config.wheel_fl_d, config.wheel_fl_i_clamp);
        setWheel("wheel_fr", config.wheel_fr_p, config.wheel_fr_i, config.wheel_fr_d, config.wheel_fr_i_clamp);
        setWheel("wheel_rl", config.wheel_rl_p, config.wheel_rl_i, config.wheel_rl_d, config.wheel_rl_i_clamp);
        setWheel("wheel_rr", config.wheel_rr_p, config.wheel_rr_i, config.wheel_rr_d, config.wheel_rr_i_clamp);

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

        ROS_INFO("WheelPidController: dynamic_reconfigure updated PID gains, mirrored to wheels/* params");
    }

    /*
     * 订阅 cmd_vel 话题的回调函数
     * @param msg: 速度指令消息
     */
    void WheelPidController::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
    {
        // 更新命令时间戳（用于自锁功能的空闲检测）
        last_cmd_time_ = ros::Time::now();

        // 通过逆向运动学计算每个轮子的期望舵角和轮子的期望角速度
        double vx = msg->linear.x;
        double vy = msg->linear.y;
        double wz = msg->angular.z;

        // 速度模式转换（类似 hero_chassis_controller）
        if (speed_mode_ == "global")
        {
            // 将速度从全局（odom）坐标系转换到局部（base_link）坐标系
            // 在本地创建 tf_listener，避免静态初始化问题
            try
            {
                tf::TransformListener tf_listener;                   // 本地tf监听器对象
                geometry_msgs::Vector3Stamped vel_global, vel_local; // 全局和局部速度向量
                vel_global.header.frame_id = odom_frame_;            // 全局坐标系
                vel_global.vector.x = vx;                            // 速度在全局坐标系中的x分量
                vel_global.vector.y = vy;                            // 速度在全局坐标系中的y分量
                vel_global.vector.z = 0.0;

                // 等待变换并转换
                /*
                target_frame – The frame into which to transform
                source_frame – The frame from which to transform
                time – The time at which to transform
                timeout – How long to block before failing
                polling_sleep_duration – How often to retest if failed
                */
                tf_listener.waitForTransform(base_link_frame_, odom_frame_, ros::Time(), ros::Duration(0.1));
                tf_listener.transformVector(base_link_frame_, vel_global, vel_local);

                // 使用转换后的速度
                vx = vel_local.vector.x;
                vy = vel_local.vector.y;

                ROS_DEBUG_THROTTLE(1.0, "Global mode: odom_vel(%.2f,%.2f) -> body_vel(%.2f,%.2f)",
                                   msg->linear.x, msg->linear.y, vx, vy);
            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN_THROTTLE(2.0, "Global mode TF transform error: %s", ex.what());
                // Fallback: use original command (assume body frame)
                vx = msg->linear.x;
                vy = msg->linear.y;
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

    /*
     * 初始化舵向电机 PID
     * @param name: 舵角名称
     * @param pid: PID 控制器对象
     * @param controller_nh: 控制器命名空间句柄
     * @param def_p: 默认比例增益
     * @param def_i: 默认积分增益
     * @param def_d: 默认微分增益
     * @param def_i_clamp: 默认积分限幅
     * @param def_antwindup: 默认防积分饱和值
     */
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

    /*
     * 初始化轮向电机 PID
     * @param name: 轮子名称
     * @param pid: PID 控制器对象
     * @param controller_nh: 控制器命名空间句柄
     * @param def_wp: 默认比例增益
     * @param def_wi: 默认积分增益
     * @param def_wd: 默认微分增益
     * @param def_wi_clamp: 默认积分限幅
     * @param def_wanti: 默认防积分饱和值
     */
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

    /*
     * 更新控制器状态的主循环函数
     * @param time: 当前时间戳
     * @param period: 上次更新到现在的时间间隔
     */
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

            // 转向架位置（发布这些以完整性）
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

        // ==================== 底盘自锁逻辑 ====================
        // 当超时未收到速度命令时，锁定底盘位置以防止滑动/抖动
        // 支持两种自锁模式：
        //   1. 位置锁定（lock_pos_enabled_）：记录当前位置，使用位置PD控制
        //   2. 几何自锁（geo_lock_enabled_）：舵轮转到自转切向，利用滑动摩擦抵抗外力
        if (self_lock_enabled_)
        {
            double idle_duration = (time - last_cmd_time_).toSec();

            if (idle_duration > idle_timeout_)
            {
                // 进入自锁状态
                if (!is_locked_)
                {
                    // 首次进入自锁：记录当前舵角和轮子位置
                    try
                    {
                        locked_pivot_pos_[0] = front_left_pivot_joint_.getPosition();
                        locked_pivot_pos_[1] = front_right_pivot_joint_.getPosition();
                        locked_pivot_pos_[2] = back_left_pivot_joint_.getPosition();
                        locked_pivot_pos_[3] = back_right_pivot_joint_.getPosition();

                        // 记录轮子位置用于位置锁定
                        locked_wheel_pos_[0] = lf_wheel_pos;
                        locked_wheel_pos_[1] = rf_wheel_pos;
                        locked_wheel_pos_[2] = lb_wheel_pos;
                        locked_wheel_pos_[3] = rb_wheel_pos;
                    }
                    catch (...)
                    {
                        // 读取失败时保持上次值
                    }
                    is_locked_ = true;
                    
                    // 日志输出当前自锁模式
                    if (geo_lock_enabled_)
                    {
                        ROS_INFO_THROTTLE(2.0, "Self-lock activated (geo-lock): pivot angles [%.1f°, %.1f°, %.1f°, %.1f°], wheel_brake=%s",
                                          geo_lock_angles_[0] * 180.0 / M_PI, geo_lock_angles_[1] * 180.0 / M_PI,
                                          geo_lock_angles_[2] * 180.0 / M_PI, geo_lock_angles_[3] * 180.0 / M_PI,
                                          geo_lock_wheel_brake_ ? "yes" : "no");
                    }
                    else if (lock_pos_enabled_)
                    {
                        ROS_INFO_THROTTLE(2.0, "Self-lock activated (position lock): wheel pos [%.2f, %.2f, %.2f, %.2f] rad",
                                          locked_wheel_pos_[0], locked_wheel_pos_[1],
                                          locked_wheel_pos_[2], locked_wheel_pos_[3]);
                    }
                    else
                    {
                        ROS_INFO_THROTTLE(2.0, "Self-lock activated (velocity lock): pivot angles [%.2f, %.2f, %.2f, %.2f] rad",
                                          locked_pivot_pos_[0], locked_pivot_pos_[1],
                                          locked_pivot_pos_[2], locked_pivot_pos_[3]);
                    }
                }

                // ==================== 选择自锁模式 ====================
                if (geo_lock_enabled_)
                {
                    // ========== 几何自锁模式 ==========
                    // 原理：将舵轮转到底盘自转的切向方向（形成"X"字或旋转姿态）
                    // 效果：任意平移方向的外力都与轮子滚动方向垂直
                    //       轮子只能侧滑（滑动摩擦）而不能滚动（滚动摩擦）
                    //       滑动摩擦系数通常远大于滚动摩擦系数，从而实现结构自锁
                    
                    // 设置舵角为预计算的几何自锁角度
                    for (int i = 0; i < 4; ++i)
                    {
                        pivot_cmd_[i] = geo_lock_angles_[i];
                    }
                    
                    // 轮子控制：可选是否锁定轮子
                    if (geo_lock_wheel_brake_)
                    {
                        // 锁定轮子位置（使用位置PD控制）
                        double wheel_pos[4] = {lf_wheel_pos, rf_wheel_pos, lb_wheel_pos, rb_wheel_pos};
                        double wheel_vel[4] = {lf_wheel_vel, rf_wheel_vel, lb_wheel_vel, rb_wheel_vel};

                        for (int i = 0; i < 4; ++i)
                        {
                            double pos_error = locked_wheel_pos_[i] - wheel_pos[i];
                            double lock_torque = lock_pos_p_ * pos_error - lock_pos_d_ * wheel_vel[i];
                            const double max_lock_torque = 30.0;
                            lock_torque = std::max(-max_lock_torque, std::min(max_lock_torque, lock_torque));
                            wheel_cmd_[i] = lock_torque * 1000.0; // 乘以1000作为标记
                        }
                    }
                    else
                    {
                        // 轮子自由滚动（目标速度=0，但不主动制动）
                        // 依靠几何结构的滑动摩擦来抵抗外力
                        for (int i = 0; i < 4; ++i)
                        {
                            wheel_cmd_[i] = 0.0;
                        }
                    }
                }
                else if (lock_pos_enabled_)
                {
                    // ========== 位置锁定模式 ==========
                    // 舵角保持进入自锁时的位置
                    for (int i = 0; i < 4; ++i)
                    {
                        pivot_cmd_[i] = locked_pivot_pos_[i];
                    }
                    
                    // 使用位置 PD 控制器直接计算力矩，绕过速度 PID
                    double wheel_pos[4] = {lf_wheel_pos, rf_wheel_pos, lb_wheel_pos, rb_wheel_pos};
                    double wheel_vel[4] = {lf_wheel_vel, rf_wheel_vel, lb_wheel_vel, rb_wheel_vel};

                    for (int i = 0; i < 4; ++i)
                    {
                        double pos_error = locked_wheel_pos_[i] - wheel_pos[i];
                        double lock_torque = lock_pos_p_ * pos_error - lock_pos_d_ * wheel_vel[i];
                        const double max_lock_torque = 30.0;
                        lock_torque = std::max(-max_lock_torque, std::min(max_lock_torque, lock_torque));
                        wheel_cmd_[i] = lock_torque * 1000.0; // 乘以1000作为标记
                    }
                }
                else
                {
                    // ========== 速度锁定模式（原方案）==========
                    // 舵角保持进入自锁时的位置
                    for (int i = 0; i < 4; ++i)
                    {
                        pivot_cmd_[i] = locked_pivot_pos_[i];
                    }
                    
                    // 将目标速度设为 0，由速度 PID 控制
                    for (int i = 0; i < 4; ++i)
                    {
                        wheel_cmd_[i] = 0.0;
                    }
                }
            }
            else
            {
                // 有命令输入，解除自锁
                if (is_locked_)
                {
                    is_locked_ = false;
                    ROS_INFO_THROTTLE(2.0, "Self-lock released");
                }
            }
        }

        // 计算轮子速度误差并应用 PID -> 力矩命令
        /* Set the PID error and compute the PID command with nonuniform time step size. The derivative error is computed from the change in the error and the timestep dt.
         */
        double cmd0, cmd1, cmd2, cmd3;

        // 自锁模式下（位置锁定或几何自锁+轮子制动）直接使用预计算的力矩，绕过速度 PID
        bool use_position_lock_torque = is_locked_ && (lock_pos_enabled_ || (geo_lock_enabled_ && geo_lock_wheel_brake_));
        
        if (use_position_lock_torque)
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

            ROS_DEBUG_THROTTLE(1.0, "Position lock torque: [%.2f, %.2f, %.2f, %.2f] Nm", cmd0, cmd1, cmd2, cmd3);
        }
        else
        {
            // 正常模式：使用速度 PID 控制
            cmd0 = pid_lf_wheel_.computeCommand(wheel_cmd_[0] - lf_wheel_vel, period);
            cmd1 = pid_rf_wheel_.computeCommand(wheel_cmd_[1] - rf_wheel_vel, period);
            cmd2 = pid_lb_wheel_.computeCommand(wheel_cmd_[2] - lb_wheel_vel, period);
            cmd3 = pid_rb_wheel_.computeCommand(wheel_cmd_[3] - rb_wheel_vel, period);
        }

        // 应用功率限制
        double scaling_factor_used = 1.0; // 缩放系数

        /*
        功率限制问题近似为关于缩放因子 scaling_factor(简称s) 的二次不等式 a * s^2 + b * s + c <= 0，其中：
        a 与 力矩的平方 (torque^2) 相关（乘 scaling_factor），
        b 与 力矩乘速度 (torque*omega) 相关（直接是功率量的绝对和），
        c 包含速度项并减去允许的功率上限与偏置。
        解析求根并取合理根（(-b + sqrt(...))/(2a)）作为允许的最大缩放因子，从而在需要时把四轮命令一并按该因子缩小，避免超过功率上限。
        该方法实现简单、计算量小、可调，但在保守性、平滑性和物理精确度上有可改进空间。
        */
        // 使能功率限制
        if (power_limit_enabled_)
        {
            // 平方函数对象
            auto sq = [](double x)
            { return x * x; };

            // 二元一次方程的求解
            double a = sq(cmd0) + sq(cmd1) + sq(cmd2) + sq(cmd3);
            double b = std::abs(cmd0 * lf_wheel_vel) + std::abs(cmd1 * rf_wheel_vel) +
                       std::abs(cmd2 * lb_wheel_vel) + std::abs(cmd3 * rb_wheel_vel);
            double c = sq(lf_wheel_vel) + sq(rf_wheel_vel) + sq(lb_wheel_vel) + sq(rb_wheel_vel);
            a *= effort_coeff_;
            c = c * velocity_coeff_ - power_offset_ - power_limit_;
            double disc = sq(b) - 4.0 * a * c;

            // 求解缩放系数
            double scaling_factor = (disc > 0.0 && a != 0.0) ? ((-b + std::sqrt(disc)) / (2.0 * a)) : 1.0;
            scaling_factor_used = scaling_factor;
            if (scaling_factor < 1.0)
            {
                cmd0 *= scaling_factor;
                cmd1 *= scaling_factor;
                cmd2 *= scaling_factor;
                cmd3 *= scaling_factor;
                ROS_INFO_THROTTLE(0.5, "Power limit triggered: scaling=%.3f (a=%.3f, b=%.3f, c=%.3f)", scaling_factor, a, b, c);
            }

            if (power_debug_enabled_)
            {
                // 发送功率限制调试信息
                std_msgs::Float64MultiArray dbg;
                dbg.data.resize(5);
                dbg.data[0] = a;
                dbg.data[1] = b;
                dbg.data[2] = c;
                dbg.data[3] = disc;
                dbg.data[4] = scaling_factor;
                power_debug_pub_.publish(dbg);
            }
        }

        // 发送应用功率控制后的力矩命令到轮子关节
        front_left_wheel_joint_.setCommand(cmd0);
        front_right_wheel_joint_.setCommand(cmd1);
        back_left_wheel_joint_.setCommand(cmd2);
        back_right_wheel_joint_.setCommand(cmd3);

        // 计算舵角误差并应用 PID -> 力矩命令
        double p0 = pid_lf_.computeCommand(pivot_cmd_[0] - front_left_pivot_joint_.getPosition(), period);
        double p1 = pid_rf_.computeCommand(pivot_cmd_[1] - front_right_pivot_joint_.getPosition(), period);
        double p2 = pid_lb_.computeCommand(pivot_cmd_[2] - back_left_pivot_joint_.getPosition(), period);
        double p3 = pid_rb_.computeCommand(pivot_cmd_[3] - back_right_pivot_joint_.getPosition(), period);

        // 发送舵角力矩命令到关节
        front_left_pivot_joint_.setCommand(p0);
        front_right_pivot_joint_.setCommand(p1);
        back_left_pivot_joint_.setCommand(p2);
        back_right_pivot_joint_.setCommand(p3);

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

        // 封装的里程计更新与发布
        odom_update(time, period);
    }

    /*
     * 里程计更新函数 odom_update()
     * 功能：基于四个舵轮的反馈数据（轮速 + 舵角）计算底盘位姿并发布 TF 与 Odometry
     * 原理：舵轮前向运动学（最小二乘求解超定方程）+ 坐标变换 + 欧拉积分
     *
     * 详细说明见文档：docs/里程计更新原理详解.md
     *
     * @param time: 当前时间戳
     * @param period: 上次更新到现在的时间间隔（用于积分）
     */
    void WheelPidController::odom_update(const ros::Time &time, const ros::Duration &period)
    {
        // ==================== 步骤 1: 读取传感器反馈 ====================
        // 从硬件接口获取四个轮子的实际角速度（rad/s）
        double lf_wheel_vel = front_left_wheel_joint_.getVelocity();  // 左前轮速
        double rf_wheel_vel = front_right_wheel_joint_.getVelocity(); // 右前轮速
        double lb_wheel_vel = back_left_wheel_joint_.getVelocity();   // 左后轮速
        double rb_wheel_vel = back_right_wheel_joint_.getVelocity();  // 右后轮速

        // 读取四个舵角的当前位置（rad，表示舵轮转向方向）
        double lf_pivot_pos = front_left_pivot_joint_.getPosition();  // 左前舵角
        double rf_pivot_pos = front_right_pivot_joint_.getPosition(); // 右前舵角
        double lb_pivot_pos = back_left_pivot_joint_.getPosition();   // 左后舵角
        double rb_pivot_pos = back_right_pivot_joint_.getPosition();  // 右后舵角

        // ==================== 步骤 2: 准备前向运动学方程数据 ====================
        // 舵轮前向运动学（FK）：已知每个轮子的舵角和轮速，反推底盘速度 (vx, vy, wz)
        // 模型：对于第 i 个轮子，沿舵向的线速度 v_i 满足
        //   v_i = cos(θ_i)*vx + sin(θ_i)*vy + (-ry_i*cos(θ_i) + rx_i*sin(θ_i))*wz
        // 四个轮子形成超定方程组 A*[vx; vy; wz] = b（4方程3未知数），用最小二乘求解

        // 数组存储：四个轮子的角速度（索引 0=FL, 1=FR, 2=RL, 3=RR）
        double wheel_vels[4] = {lf_wheel_vel, rf_wheel_vel, lb_wheel_vel, rb_wheel_vel};
        // 数组存储：四个舵角（弧度）
        double pivot_angles[4] = {lf_pivot_pos, rf_pivot_pos, lb_pivot_pos, rb_pivot_pos};

        // 轮子相对底盘中心 (base_link) 的位置（右手坐标系：x向前，y向左）
        // FL=(+x/2, +y/2), FR=(+x/2, -y/2), RL=(-x/2, +y/2), RR=(-x/2, -y/2)
        double rx[4] = {wheel_base_ / 2.0, wheel_base_ / 2.0, -wheel_base_ / 2.0, -wheel_base_ / 2.0};     // x坐标
        double ry[4] = {wheel_track_ / 2.0, -wheel_track_ / 2.0, wheel_track_ / 2.0, -wheel_track_ / 2.0}; // y坐标

        // 初始化系数矩阵 A (4x3) 和观测向量 b (4x1)
        double A[4][3]; // 每行对应一个轮子的运动学方程系数
        double b[4];    // 每个元素是轮子沿舵向的线速度（由轮速×轮径得到）
        // 循环构造每个轮子的方程（i = 0..3，对应 FL/FR/RL/RR）
        for (int i = 0; i < 4; ++i)
        {
            double theta = pivot_angles[i]; // 当前轮子的舵角（弧度）
            double cos_t = std::cos(theta); // 舵角的余弦（舵向 x 分量）
            double sin_t = std::sin(theta); // 舵角的正弦（舵向 y 分量）

            // 将轮子的角速度转换为沿舵向的线速度：v_along = ω × r
            double v_along = wheel_vels[i] * wheel_radius_;

            // 组装系数矩阵 A 的第 i 行（对应底盘速度 [vx, vy, wz] 的贡献）
            // v_i = A[i][0]*vx + A[i][1]*vy + A[i][2]*wz
            A[i][0] = cos_t;                          // vx 的系数：舵向在 x 轴投影
            A[i][1] = sin_t;                          // vy 的系数：舵向在 y 轴投影
            A[i][2] = -ry[i] * cos_t + rx[i] * sin_t; // wz 的系数：底盘旋转导致该轮位置的切向速度在舵向上的投影

            // 观测向量 b：轮子实测的沿舵向线速度
            b[i] = v_along;
        }

        // ==================== 步骤 3: 最小二乘求解（法方程） ====================
        // 目标：求解 min ||A*x - b||^2，其中 x = [vx; vy; wz]
        // 解析解：x = (A^T * A)^(-1) * A^T * b
        // 先计算 A^T*A (3x3 对称矩阵) 和 A^T*b (3x1 向量)

        double ATA[3][3] = {{0}}; // A^T * A 的结果矩阵（初始化为零）
        double ATb[3] = {0};      // A^T * b 的结果向量（初始化为零）

        // 双重循环计算矩阵乘法：ATA[j][k] += sum_i A[i][j] * A[i][k]
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

        // ==================== 步骤 4: 求解 3x3 线性方程组 ====================
        // 需要计算 ATA 的逆矩阵，先计算行列式判断是否奇异
        // det(ATA) 使用萨吕斯法则（对3x3矩阵展开）
        double det = ATA[0][0] * (ATA[1][1] * ATA[2][2] - ATA[1][2] * ATA[2][1]) -
                     ATA[0][1] * (ATA[1][0] * ATA[2][2] - ATA[1][2] * ATA[2][0]) +
                     ATA[0][2] * (ATA[1][0] * ATA[2][1] - ATA[1][1] * ATA[2][0]);

        // 初始化底盘速度解（默认零速度）
        double vx = 0.0, vy = 0.0, wz = 0.0;

        // 判断矩阵是否奇异（行列式绝对值 > 阈值表示可逆）
        if (std::abs(det) > 1e-9)
        {
            // ========== 计算 3x3 逆矩阵（伴随矩阵法：inv = adj(ATA) / det） ==========
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

            // ========== 求解：x = inv(ATA) * ATb ==========
            // 矩阵-向量乘法得到底盘速度（机体坐标系）
            vx = inv[0][0] * ATb[0] + inv[0][1] * ATb[1] + inv[0][2] * ATb[2]; // base_link x 方向速度 (m/s)
            vy = inv[1][0] * ATb[0] + inv[1][1] * ATb[1] + inv[1][2] * ATb[2]; // base_link y 方向速度 (m/s)
            wz = inv[2][0] * ATb[0] + inv[2][1] * ATb[1] + inv[2][2] * ATb[2]; // 绕 z 轴角速度 (rad/s)
        }
        else
        {
            // Matrix singular (possible causes: four-wheel pivot angles collinear, sensor failure, initialization phase, etc.)
            // Issue warning and use zero velocity as fallback (avoid publishing incorrect odometry)
            ROS_WARN_THROTTLE(1.0, "Forward kinematics matrix singular, using zero velocity");
        }

        // ==================== 步骤 4.5: 速度死区滤波（消除静止抖动导致的里程计漂移） ====================
        // 当计算出的速度小于死区阈值时，视为噪声并强制归零
        // 这可以有效消除底盘静止时由于传感器噪声或微小抖动导致的里程计漂移
        if (std::abs(vx) < odom_velocity_deadband_)
        {
            vx = 0.0;
        }
        if (std::abs(vy) < odom_velocity_deadband_)
        {
            vy = 0.0;
        }
        if (std::abs(wz) < odom_velocity_deadband_)
        {
            wz = 0.0;
        }

        // ==================== 步骤 5: 坐标变换（机体系 → 世界系）与位姿积分 ====================
        // 获取当前时间戳与控制周期
        ros::Time now = time;
        double dt = period.toSec(); // 时间步长（秒）

        // 将机体坐标系速度 (vx, vy) 旋转到世界坐标系（odom frame）
        // 二维旋转矩阵：[vx_world; vy_world] = R(yaw) * [vx; vy]
        // 其中 R(yaw) = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)]
        double cos_y = std::cos(odom_yaw_);        // 当前航向的余弦
        double sin_y = std::sin(odom_yaw_);        // 当前航向的正弦
        double world_vx = vx * cos_y - vy * sin_y; // 世界系 x 方向速度（向前）
        double world_vy = vx * sin_y + vy * cos_y; // 世界系 y 方向速度（向左）

        // 欧拉积分更新位姿（一阶近似：新位置 = 旧位置 + 速度 × 时间步长）
        odom_x_ += world_vx * dt; // 世界系 x 坐标累积（米）
        odom_y_ += world_vy * dt; // 世界系 y 坐标累积（米）
        odom_yaw_ += wz * dt;     // 航向角累积（弧度）；注意：角速度在 2D 平面下无需旋转变换

        // ==================== 步骤 6: 发布 TF 变换（odom → base_link） ====================
        // 创建坐标变换消息（TransformStamped）
        geometry_msgs::TransformStamped t;
        t.header.stamp = now;                // 时间戳（与当前控制周期同步）
        t.header.frame_id = odom_frame_;     // 父坐标系：odom（世界/里程计坐标系）
        t.child_frame_id = base_link_frame_; // 子坐标系：base_link（底盘机体坐标系）

        // 平移部分：底盘在 odom 系中的位置
        t.transform.translation.x = odom_x_; // x 方向位移（米）
        t.transform.translation.y = odom_y_; // y 方向位移（米）
        t.transform.translation.z = 0.0;     // z 方向位移（假设平面运动，固定为 0）

        // 旋转部分：将欧拉角（roll=0, pitch=0, yaw=odom_yaw_）转换为四元数
        // 四元数用于避免万向锁并保证插值平滑
        tf2::Quaternion qtn;
        qtn.setRPY(0.0, 0.0, odom_yaw_);     // 设置欧拉角（仅绕 z 轴旋转）
        t.transform.rotation.x = qtn.getX(); // 四元数 x 分量
        t.transform.rotation.y = qtn.getY(); // 四元数 y 分量
        t.transform.rotation.z = qtn.getZ(); // 四元数 z 分量
        t.transform.rotation.w = qtn.getW(); // 四元数 w 分量（实部）

        // 广播 TF 变换到 /tf 话题（供 TF 树查询使用）
        tf_broadcaster_.sendTransform(t);

        // ==================== 步骤 7: 发布里程计消息（nav_msgs::Odometry） ====================
        // 创建里程计消息（包含位姿 + 速度信息）
        nav_msgs::Odometry odom;
        odom.header.stamp = now;                // 时间戳
        odom.header.frame_id = odom_frame_;     // 位姿参考系（pose 在 odom 坐标系中表示）
        odom.child_frame_id = base_link_frame_; // 速度参考系（twist 在 base_link 坐标系中表示）

        // ========== 位姿部分（pose.pose）：底盘在 odom 系中的位置和姿态 ==========
        odom.pose.pose.position.x = odom_x_; // x 坐标（米）
        odom.pose.pose.position.y = odom_y_; // y 坐标（米）
        odom.pose.pose.position.z = 0.0;     // z 坐标（平面运动，固定为 0）

        // 姿态用四元数表示（与 TF 中保持一致）
        odom.pose.pose.orientation.x = qtn.getX();
        odom.pose.pose.orientation.y = qtn.getY();
        odom.pose.pose.orientation.z = qtn.getZ();
        odom.pose.pose.orientation.w = qtn.getW();

        // ========== 速度部分（twist.twist）：底盘在 base_link 系（机体系）中的速度 ==========
        // 注意：ROS 标准约定 twist 在 child_frame_id 坐标系（即 base_link）中表示
        odom.twist.twist.linear.x = vx;  // 机体 x 方向线速度（前进方向，m/s）
        odom.twist.twist.linear.y = vy;  // 机体 y 方向线速度（横向，m/s）
        odom.twist.twist.angular.z = wz; // 绕 z 轴角速度（自转，rad/s）

        // 发布里程计消息到话题（默认 /odom_controller，供导航栈使用）
        odom_pub_.publish(odom);
    }

    // 注册插件
    PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::WheelPidController, controller_interface::ControllerBase)

}
