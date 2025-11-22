// Minimal wheel PID controller using control_toolbox::Pid and exported as a plugin
// Implementation is based on the simple_chassis_controller pattern: 4 pivot PIDs (position) and 4 wheel PIDs (velocity)

#include "sentry_chassis_controller/wheel_pid_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <string>

namespace sentry_chassis_controller
{

    bool WheelPidController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
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

        // initialize PIDs with parameters (if present) or defaults
        double p, i, d, i_clamp, antiwindup;
        // pivot PIDs
        controller_nh.param("pivot/p", p, 1.0);
        controller_nh.param("pivot/i", i, 0.0);
        controller_nh.param("pivot/d", d, 0.0);
        controller_nh.param("pivot/i_clamp", i_clamp, 0.0);
        controller_nh.param("pivot/antiwindup", antiwindup, 0.0);
        pid_lf_.initPid(p, i, d, i_clamp, antiwindup);
        pid_rf_.initPid(p, i, d, i_clamp, antiwindup);
        pid_lb_.initPid(p, i, d, i_clamp, antiwindup);
        pid_rb_.initPid(p, i, d, i_clamp, antiwindup);

        // wheel velocity PIDs
        controller_nh.param("wheel/p", p, 2.0);
        controller_nh.param("wheel/i", i, 0.1);
        controller_nh.param("wheel/d", d, 0.0);
        controller_nh.param("wheel/i_clamp", i_clamp, 0.0);
        controller_nh.param("wheel/antiwindup", antiwindup, 0.0);
        pid_lf_wheel_.initPid(p, i, d, i_clamp, antiwindup);
        pid_rf_wheel_.initPid(p, i, d, i_clamp, antiwindup);
        pid_lb_wheel_.initPid(p, i, d, i_clamp, antiwindup);
        pid_rb_wheel_.initPid(p, i, d, i_clamp, antiwindup);

        // default commands
        for (int iidx = 0; iidx < 4; ++iidx)
        {
            pivot_cmd_[iidx] = 0.0;
            wheel_cmd_[iidx] = 0.0;
        }

        // subscribe to cmd_vel to get desired chassis velocity (in chassis frame)
        cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &WheelPidController::cmdVelCallback, this);

        ROS_INFO("WheelPidController initialized");
        return true;
    }

    void WheelPidController::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
    {
        // Very simple mapping: set pivot angles to 0 and distribute linear.x to all wheels.
        // For angular.z, add differential term to left/right wheels.
        double vx = msg->linear.x;
        double vy = msg->linear.y; // unused in this simple mapping
        double wz = msg->angular.z;

        // Basic differential contribution (not full mecanum/swerve inverse kinematics)
        double ang_contrib = (wz * (wheel_base_ / 2.0));

        wheel_cmd_[0] = vx - ang_contrib; // front left
        wheel_cmd_[1] = vx + ang_contrib; // front right
        wheel_cmd_[2] = vx - ang_contrib; // back left
        wheel_cmd_[3] = vx + ang_contrib; // back right

        // keep pivot angles at zero (facing forward) for this minimal implementation
        for (int i = 0; i < 4; ++i)
            pivot_cmd_[i] = 0.0;
    }

    void WheelPidController::update(const ros::Time &time, const ros::Duration &period)
    {
        // compute wheel velocity error and apply PID -> effort command

        double cmd0 = pid_lf_wheel_.computeCommand(wheel_cmd_[0] - front_left_wheel_joint_.getVelocity(), period);
        double cmd1 = pid_rf_wheel_.computeCommand(wheel_cmd_[1] - front_right_wheel_joint_.getVelocity(), period);
        double cmd2 = pid_lb_wheel_.computeCommand(wheel_cmd_[2] - back_left_wheel_joint_.getVelocity(), period);
        double cmd3 = pid_rb_wheel_.computeCommand(wheel_cmd_[3] - back_right_wheel_joint_.getVelocity(), period);

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
    }

    // Export as plugin so controller_manager / pluginlib can load it by class name
    PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::WheelPidController, controller_interface::ControllerBase)

} // namespace sentry_chassis_controller
