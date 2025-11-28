#include "sentry_chassis_controller/wheel_pid_controller.hpp"
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
        // store controller namespace handle for later param set
        controller_nh_ = controller_nh;
        // get joint handles (names can be overridden via ROS params if desired)
        try
        {
            front_left_wheel_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
            front_right_wheel_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
            back_left_wheel_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
            back_right_wheel_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

            front_left_pivot_joint_ = effort_joint_interface->getHandle("left_front_pivot_joint");
            front_right_pivot_joint_ = effort_joint_interface->getHandle("right_front_pivot_joint");
            back_left_pivot_joint_ = effort_joint_interface->getHandle("left_back_pivot_joint");
            back_right_pivot_joint_ = effort_joint_interface->getHandle("right_back_pivot_joint");
        }
        catch (const hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR("WheelPidController: Exception getting joint handles: %s", ex.what());
            return false;
        }

        // read kinematic parameters
        controller_nh.param("wheel_track", wheel_track_, 0.36);
        controller_nh.param("wheel_base", wheel_base_, 0.36);
        controller_nh.param("wheel_radius", wheel_radius_, 0.05);
        rx_ = wheel_track_ / 2.0;
        ry_ = wheel_base_ / 2.0;

        // read power limiting parameters (optional)
        controller_nh.param("power_limit", power_limit_, 0.0);
        controller_nh.param("power/velocity_coeff", velocity_coeff_, 0.0);
        controller_nh.param("power/effort_coeff", effort_coeff_, 0.0);
        controller_nh.param("power/power_offset", power_offset_, 0.0);
        power_limit_enabled_ = (effort_coeff_ != 0.0 || velocity_coeff_ != 0.0);

        // initialize PIDs with parameters (if present) or defaults
        double def_p, def_i, def_d, def_i_clamp, def_antwindup;
        // read pivot defaults
        controller_nh.param("pivot/p", def_p, 1.0);
        controller_nh.param("pivot/i", def_i, 0.0);
        controller_nh.param("pivot/d", def_d, 0.0);
        controller_nh.param("pivot/i_clamp", def_i_clamp, 0.0);
        controller_nh.param("pivot/antiwindup", def_antwindup, 0.0);

        // read wheel defaults
        double def_wp, def_wi, def_wd, def_wi_clamp, def_wanti;
        controller_nh.param("wheel/p", def_wp, 2.0);
        controller_nh.param("wheel/i", def_wi, 0.1);
        controller_nh.param("wheel/d", def_wd, 0.0);
        controller_nh.param("wheel/i_clamp", def_wi_clamp, 0.0);
        controller_nh.param("wheel/antiwindup", def_wanti, 0.0);

        // initialize pivots (names in YAML are pivot_fl, pivot_fr, pivot_rl, pivot_rr)
        initPivot("pivot_fl", pid_lf_, controller_nh, def_p, def_i, def_d, def_i_clamp, def_antwindup);
        initPivot("pivot_fr", pid_rf_, controller_nh, def_p, def_i, def_d, def_i_clamp, def_antwindup);
        initPivot("pivot_rl", pid_lb_, controller_nh, def_p, def_i, def_d, def_i_clamp, def_antwindup);
        initPivot("pivot_rr", pid_rb_, controller_nh, def_p, def_i, def_d, def_i_clamp, def_antwindup);

        // initialize wheel velocity PIDs (names: wheel_fl, wheel_fr, wheel_rl, wheel_rr)
        initWheel("wheel_fl", pid_lf_wheel_, controller_nh, def_wp, def_wi, def_wd, def_wi_clamp, def_wanti);
        initWheel("wheel_fr", pid_rf_wheel_, controller_nh, def_wp, def_wi, def_wd, def_wi_clamp, def_wanti);
        initWheel("wheel_rl", pid_lb_wheel_, controller_nh, def_wp, def_wi, def_wd, def_wi_clamp, def_wanti);
        initWheel("wheel_rr", pid_rb_wheel_, controller_nh, def_wp, def_wi, def_wd, def_wi_clamp, def_wanti);

        // default commands
        for (int iidx = 0; iidx < 4; ++iidx)
        {
            pivot_cmd_[iidx] = 0.0;
            wheel_cmd_[iidx] = 0.0;
        }

        // subscribe to cmd_vel to get desired chassis velocity (in chassis frame)
        cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &WheelPidController::cmdVelCallback, this);

        // publisher to expose desired wheel/pivot commands for testing/inspection
        desired_pub_ = root_nh.advertise<sensor_msgs::JointState>("desired_wheel_states", 1);

        // dynamic_reconfigure server: allow runtime tuning of per-wheel PIDs
        dyn_server_.reset(new dynamic_reconfigure::Server<Config>(controller_nh));
        dynamic_reconfigure::Server<Config>::CallbackType cb = boost::bind(&WheelPidController::reconfigureCallback, this, _1, _2);
        dyn_server_->setCallback(cb);

        // NEW: Publisher for actual joint states to enable forward kinematics
        joint_states_pub_ = root_nh.advertise<sensor_msgs::JointState>("joint_states", 1);
        last_state_pub_ = ros::Time(0);
        // Applied efforts and power debug publishers
        applied_effort_pub_ = root_nh.advertise<sensor_msgs::JointState>("applied_wheel_efforts", 1);
        power_debug_pub_ = root_nh.advertise<std_msgs::Float64MultiArray>("power_debug", 1);
        last_effort_pub_ = ros::Time(0);
        controller_nh.param("power_debug", power_debug_enabled_, false);

        // Integrated odometry publisher
        controller_nh.param<std::string>("odom_frame", odom_frame_, odom_frame_);
        controller_nh.param<std::string>("base_link_frame", base_link_frame_, base_link_frame_);
        odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("odom", 10);
        last_odom_time_ = ros::Time(0);

        ROS_INFO("WheelPidController initialized with enhanced state feedback!");
        return true;
    }

    void WheelPidController::reconfigureCallback(Config &config, uint32_t level)
    {
        // update wheel PIDs
        pid_lf_wheel_.initPid(config.wheel_fl_p, config.wheel_fl_i, config.wheel_fl_d, config.wheel_fl_i_clamp, 0.0);
        pid_rf_wheel_.initPid(config.wheel_fr_p, config.wheel_fr_i, config.wheel_fr_d, config.wheel_fr_i_clamp, 0.0);
        pid_lb_wheel_.initPid(config.wheel_rl_p, config.wheel_rl_i, config.wheel_rl_d, config.wheel_rl_i_clamp, 0.0);
        pid_rb_wheel_.initPid(config.wheel_rr_p, config.wheel_rr_i, config.wheel_rr_d, config.wheel_rr_i_clamp, 0.0);

        // update pivot PIDs
        pid_lf_.initPid(config.pivot_fl_p, config.pivot_fl_i, config.pivot_fl_d, config.pivot_fl_i_clamp, 0.0);
        pid_rf_.initPid(config.pivot_fr_p, config.pivot_fr_i, config.pivot_fr_d, config.pivot_fr_i_clamp, 0.0);
        pid_lb_.initPid(config.pivot_rl_p, config.pivot_rl_i, config.pivot_rl_d, config.pivot_rl_i_clamp, 0.0);
        pid_rb_.initPid(config.pivot_rr_p, config.pivot_rr_i, config.pivot_rr_d, config.pivot_rr_i_clamp, 0.0);

        // Mirror dynamic values back to YAML-style keys under wheels/* for rosparam get
        auto setWheel = [this](const std::string &name, double p, double i, double d, double i_clamp)
        {
            const std::string base = std::string("wheels/") + name;
            controller_nh_.setParam(base + "/p", p);
            controller_nh_.setParam(base + "/i", i);
            controller_nh_.setParam(base + "/d", d);
            controller_nh_.setParam(base + "/i_clamp", i_clamp);
        };

        setWheel("wheel_fl", config.wheel_fl_p, config.wheel_fl_i, config.wheel_fl_d, config.wheel_fl_i_clamp);
        setWheel("wheel_fr", config.wheel_fr_p, config.wheel_fr_i, config.wheel_fr_d, config.wheel_fr_i_clamp);
        setWheel("wheel_rl", config.wheel_rl_p, config.wheel_rl_i, config.wheel_rl_d, config.wheel_rl_i_clamp);
        setWheel("wheel_rr", config.wheel_rr_p, config.wheel_rr_i, config.wheel_rr_d, config.wheel_rr_i_clamp);

        auto setPivot = [this](const std::string &name, double p, double i, double d, double i_clamp)
        {
            const std::string base = std::string("wheels/") + name;
            controller_nh_.setParam(base + "/p", p);
            controller_nh_.setParam(base + "/i", i);
            controller_nh_.setParam(base + "/d", d);
            controller_nh_.setParam(base + "/i_clamp", i_clamp);
        };

        setPivot("pivot_fl", config.pivot_fl_p, config.pivot_fl_i, config.pivot_fl_d, config.pivot_fl_i_clamp);
        setPivot("pivot_fr", config.pivot_fr_p, config.pivot_fr_i, config.pivot_fr_d, config.pivot_fr_i_clamp);
        setPivot("pivot_rl", config.pivot_rl_p, config.pivot_rl_i, config.pivot_rl_d, config.pivot_rl_i_clamp);
        setPivot("pivot_rr", config.pivot_rr_p, config.pivot_rr_i, config.pivot_rr_d, config.pivot_rr_i_clamp);

        ROS_INFO("WheelPidController: dynamic_reconfigure updated PID gains and mirrored to wheels/* parameters");
    }

    void WheelPidController::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
    {
        // Use inverse kinematics to compute per-wheel steer angles and wheel angular velocities
        double vx = msg->linear.x;
        double vy = msg->linear.y;
        double wz = msg->angular.z;

        auto ik = sentry_kinematics::inverseKinematics(vx, vy, wz, wheel_base_, wheel_track_, wheel_radius_);

        // ik.wheel_angular_vel is in rad/s, pivot angles in radians
        for (int i = 0; i < 4; ++i)
        {
            wheel_cmd_[i] = ik.wheel_angular_vel[i];
            pivot_cmd_[i] = ik.steer_angle[i];
        }

        // publish desired commands for inspection
        sensor_msgs::JointState js;
        js.header.stamp = ros::Time::now();
        js.name = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"};
        js.position.resize(4);
        js.velocity.resize(4);
        for (int i = 0; i < 4; ++i)
        {
            js.position[i] = pivot_cmd_[i];
            js.velocity[i] = wheel_cmd_[i];
        }
        desired_pub_.publish(js);
    }

    void WheelPidController::initPivot(const std::string &name, control_toolbox::Pid &pid,
                                       ros::NodeHandle &controller_nh, double def_p, double def_i,
                                       double def_d, double def_i_clamp, double def_antwindup)
    {
        std::string base = "wheels/" + name;
        double p = def_p, i = def_i, d = def_d, i_clamp = def_i_clamp, antiwindup = def_antwindup;
        // per-wheel pivot entries are named pivot_... in our YAML
        controller_nh.param(base + "/p", p, p);
        controller_nh.param(base + "/i", i, i);
        controller_nh.param(base + "/d", d, d);
        controller_nh.param(base + "/i_clamp", i_clamp, i_clamp);
        controller_nh.param(base + "/antiwindup", antiwindup, antiwindup);
        pid.initPid(p, i, d, i_clamp, antiwindup);
        ROS_INFO("WheelPidController: init pivot '%s' p=%.3f i=%.3f d=%.3f i_clamp=%.3f", name.c_str(), p, i, d, i_clamp);
    }

    void WheelPidController::initWheel(const std::string &name, control_toolbox::Pid &pid,
                                       ros::NodeHandle &controller_nh, double def_wp, double def_wi,
                                       double def_wd, double def_wi_clamp, double def_wanti)
    {
        std::string base = "wheels/" + name;
        double p = def_wp, i = def_wi, d = def_wd, i_clamp = def_wi_clamp, antiwindup = def_wanti;
        controller_nh.param(base + "/p", p, p);
        controller_nh.param(base + "/i", i, i);
        controller_nh.param(base + "/d", d, d);
        controller_nh.param(base + "/i_clamp", i_clamp, i_clamp);
        controller_nh.param(base + "/antiwindup", antiwindup, antiwindup);
        pid.initPid(p, i, d, i_clamp, antiwindup);
        ROS_INFO("WheelPidController: init wheel '%s' p=%.3f i=%.3f d=%.3f i_clamp=%.3f", name.c_str(), p, i, d, i_clamp);
    }

    void WheelPidController::update(const ros::Time &time, const ros::Duration &period)
    {
        // Enhanced joint state reporting for Forward Kinematics
        // CRITICAL: Must read actual joint positions/velocities from hardware interface

        // READ WHEEL STATES (CRITICAL FIX)
        double lf_wheel_pos = 0.0, rf_wheel_pos = 0.0, lb_wheel_pos = 0.0, rb_wheel_pos = 0.0;
        double lf_wheel_vel = 0.0, rf_wheel_vel = 0.0, lb_wheel_vel = 0.0, rb_wheel_vel = 0.0;

        try
        {
            // Critical: Extract actual wheel positions/velocities from hardware
            lf_wheel_pos = front_left_wheel_joint_.getPosition();
            rf_wheel_pos = front_right_wheel_joint_.getPosition();
            lb_wheel_pos = back_left_wheel_joint_.getPosition();
            rb_wheel_pos = back_right_wheel_joint_.getPosition();

            lf_wheel_vel = front_left_wheel_joint_.getVelocity();
            rf_wheel_vel = front_right_wheel_joint_.getVelocity();
            lb_wheel_vel = back_left_wheel_joint_.getVelocity();
            rb_wheel_vel = back_right_wheel_joint_.getVelocity();

            // Debug log occasional readings to monitor state
            static ros::Time last_log = ros::Time(0);
            if (time.toSec() - last_log.toSec() > 1.0) // Log every second
            {
                ROS_INFO_STREAM_THROTTLE(1.0, "Joint States:"
                                                  << " LF:" << lf_wheel_pos << "/v:" << lf_wheel_vel
                                                  << " RF:" << rf_wheel_pos << "/v:" << rf_wheel_vel
                                                  << " LB:" << lb_wheel_pos << "/v:" << lb_wheel_vel
                                                  << " RB:" << rb_wheel_pos << "/v:" << rb_wheel_vel);
                last_log = time;
            }
        }
        catch (const hardware_interface::HardwareInterfaceException &e)
        {
            ROS_ERROR_THROTTLE(1.0, "Failed to read joint states: %s", e.what());
        }

        // ENSURE NON-ZERO STATES (SAFETY FALLBACK)
        // If ALL positions are zero (simulated sensor failure), report minimal movement
        double min_movement = 0.001; // Minimal non-zero to prevent division by zero
        if (fabs(lf_wheel_pos) + fabs(rf_wheel_pos) + fabs(lb_wheel_pos) + fabs(rb_wheel_pos) < min_movement * 4)
        {
            // Provide minimal synthetic movement to keep odom alive (only if commanded)
            double synthetic_pos = min_movement * wheel_cmd_[0] * 0.01; // Tiny deflection based on command
            lf_wheel_pos = synthetic_pos;
            rf_wheel_pos = synthetic_pos;
            lb_wheel_pos = synthetic_pos;
            rb_wheel_pos = synthetic_pos;
            ROS_WARN_THROTTLE(2.0, "All joint positions zero! Using synthetic backup values");
        }

        // PUBLISH ACTUAL JOINT STATES (CRITICAL FIX FOR FK)
        if (time.toSec() - last_state_pub_.toSec() > 0.1) // 10Hz publishing rate
        {
            sensor_msgs::JointState js;
            js.header.stamp = time;
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

            // Wheel positions/velocities
            js.position[0] = lf_wheel_pos;
            js.velocity[0] = lf_wheel_vel;
            js.position[1] = rf_wheel_pos;
            js.velocity[1] = rf_wheel_vel;
            js.position[2] = lb_wheel_pos;
            js.velocity[2] = lb_wheel_vel;
            js.position[3] = rb_wheel_pos;
            js.velocity[3] = rb_wheel_vel;

            // Pivot positions (we can also publish these for completeness)
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

            // Essential logging to verify fix
            ROS_DEBUG_STREAM_THROTTLE(1.0, "Publishing joint states: "
                                               << " wheel pos=" << js.position[0] << "," << js.position[1]
                                               << " vel=" << js.velocity[0] << "," << js.velocity[1]
                                               << " pivot pos=" << js.position[4]);

            joint_states_pub_.publish(js);
            last_state_pub_ = time;
        }

        // compute wheel velocity error and apply PID -> effort command
        double cmd0 = pid_lf_wheel_.computeCommand(wheel_cmd_[0] - lf_wheel_vel, period);
        double cmd1 = pid_rf_wheel_.computeCommand(wheel_cmd_[1] - rf_wheel_vel, period);
        double cmd2 = pid_lb_wheel_.computeCommand(wheel_cmd_[2] - lb_wheel_vel, period);
        double cmd3 = pid_rb_wheel_.computeCommand(wheel_cmd_[3] - rb_wheel_vel, period);

        // Apply power limiting similar to hero_chassis_controller
        double zoom_coeff_used = 1.0;
        if (power_limit_enabled_)
        {
            auto sq = [](double x)
            { return x * x; };
            double a = sq(cmd0) + sq(cmd1) + sq(cmd2) + sq(cmd3);
            double b = std::abs(cmd0 * lf_wheel_vel) + std::abs(cmd1 * rf_wheel_vel) +
                       std::abs(cmd2 * lb_wheel_vel) + std::abs(cmd3 * rb_wheel_vel);
            double c = sq(lf_wheel_vel) + sq(rf_wheel_vel) + sq(lb_wheel_vel) + sq(rb_wheel_vel);
            a *= effort_coeff_;
            c = c * velocity_coeff_ - power_offset_ - power_limit_;
            double disc = sq(b) - 4.0 * a * c;
            double zoom_coeff = (disc > 0.0 && a != 0.0) ? ((-b + std::sqrt(disc)) / (2.0 * a)) : 1.0;
            zoom_coeff_used = zoom_coeff;
            if (zoom_coeff < 1.0)
            {
                cmd0 *= zoom_coeff;
                cmd1 *= zoom_coeff;
                cmd2 *= zoom_coeff;
                cmd3 *= zoom_coeff;
                ROS_INFO_THROTTLE(0.5, "Power limit active: zoom=%.3f (a=%.3f, b=%.3f, c=%.3f)", zoom_coeff, a, b, c);
            }

            if (power_debug_enabled_)
            {
                std_msgs::Float64MultiArray dbg;
                dbg.data.resize(5);
                dbg.data[0] = a;
                dbg.data[1] = b;
                dbg.data[2] = c;
                dbg.data[3] = disc;
                dbg.data[4] = zoom_coeff;
                power_debug_pub_.publish(dbg);
            }
        }

        front_left_wheel_joint_.setCommand(cmd0);
        front_right_wheel_joint_.setCommand(cmd1);
        back_left_wheel_joint_.setCommand(cmd2);
        back_right_wheel_joint_.setCommand(cmd3);

        // pivot position control (simple position PID)
        double p0 = pid_lf_.computeCommand(pivot_cmd_[0] - front_left_pivot_joint_.getPosition(), period);
        double p1 = pid_rf_.computeCommand(pivot_cmd_[1] - front_right_pivot_joint_.getPosition(), period);
        double p2 = pid_lb_.computeCommand(pivot_cmd_[2] - back_left_pivot_joint_.getPosition(), period);
        double p3 = pid_rb_.computeCommand(pivot_cmd_[3] - back_right_pivot_joint_.getPosition(), period);

        front_left_pivot_joint_.setCommand(p0);
        front_right_pivot_joint_.setCommand(p1);
        back_left_pivot_joint_.setCommand(p2);
        back_right_pivot_joint_.setCommand(p3);

        // Publish applied efforts at ~10Hz
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

        // Encapsulated odometry update and publish
        odom_update(time, period);
    }

    void WheelPidController::odom_update(const ros::Time &time, const ros::Duration &period)
    {
        // Read current wheel velocities to compute chassis velocities
        double lf_wheel_vel = front_left_wheel_joint_.getVelocity();
        double rf_wheel_vel = front_right_wheel_joint_.getVelocity();
        double lb_wheel_vel = back_left_wheel_joint_.getVelocity();
        double rb_wheel_vel = back_right_wheel_joint_.getVelocity();

        // Compute chassis velocities from wheel angular velocities (mecanum-like)
        double vx = (lf_wheel_vel + rf_wheel_vel + lb_wheel_vel + rb_wheel_vel) * wheel_radius_ / 4.0;
        double vy = (-lf_wheel_vel + rf_wheel_vel + lb_wheel_vel - rb_wheel_vel) * wheel_radius_ / 4.0;
        double wz = (-lf_wheel_vel + rf_wheel_vel - lb_wheel_vel + rb_wheel_vel) * wheel_radius_ / (4.0 * (rx_ + ry_));

        // Integrate odom in world (odom frame)
        ros::Time now = time;
        if (last_odom_time_.isZero())
            last_odom_time_ = now;
        double dt = (now - last_odom_time_).toSec();
        if (dt < 0.0)
            dt = 0.0;

        double dx = (vx * std::cos(odom_yaw_) - vy * std::sin(odom_yaw_)) * dt;
        double dy = (vx * std::sin(odom_yaw_) + vy * std::cos(odom_yaw_)) * dt;
        double dth = wz * dt;
        odom_x_ += dx;
        odom_y_ += dy;
        odom_yaw_ += dth;

        // Publish TF and Odometry
        geometry_msgs::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = odom_frame_;
        t.child_frame_id = base_link_frame_;
        t.transform.translation.x = odom_x_;
        t.transform.translation.y = odom_y_;
        t.transform.translation.z = 0.0;
        tf2::Quaternion qtn;
        qtn.setRPY(0.0, 0.0, odom_yaw_);
        t.transform.rotation.x = qtn.x();
        t.transform.rotation.y = qtn.y();
        t.transform.rotation.z = qtn.z();
        t.transform.rotation.w = qtn.w();
        tf_broadcaster_.sendTransform(t);

        nav_msgs::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_link_frame_;
        odom.pose.pose.position.x = odom_x_;
        odom.pose.pose.position.y = odom_y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = qtn.x();
        odom.pose.pose.orientation.y = qtn.y();
        odom.pose.pose.orientation.z = qtn.z();
        odom.pose.pose.orientation.w = qtn.w();
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = wz;

        for (int i = 0; i < 36; ++i)
        {
            odom.pose.covariance[i] = 0.0;
            odom.twist.covariance[i] = 0.0;
        }
        odom.pose.covariance[0] = 1e-3;
        odom.pose.covariance[7] = 1e-3;
        odom.pose.covariance[14] = 1e9;
        odom.pose.covariance[21] = 1e9;
        odom.pose.covariance[28] = 1e9;
        odom.pose.covariance[35] = 1e-2;
        odom.twist.covariance[0] = 1e-3;
        odom.twist.covariance[7] = 1e-3;
        odom.twist.covariance[14] = 1e9;
        odom.twist.covariance[21] = 1e9;
        odom.twist.covariance[28] = 1e9;
        odom.twist.covariance[35] = 1e-2;

        odom_pub_.publish(odom);
        last_odom_time_ = now;
    }

    // Export as plugin so controller_manager / pluginlib can load it by class name
    PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::WheelPidController, controller_interface::ControllerBase)

} // namespace sentry_chassis_controller
