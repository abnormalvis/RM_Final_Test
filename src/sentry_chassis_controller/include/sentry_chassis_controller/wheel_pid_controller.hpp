#ifndef SENTRY_CHASSIS_CONTROLLER_WHEEL_PID_CONTROLLER_H
#define SENTRY_CHASSIS_CONTROLLER_WHEEL_PID_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <sentry_chassis_controller/WheelPidConfig.h>

namespace sentry_chassis_controller
{

    class WheelPidController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        WheelPidController() = default;
        ~WheelPidController() override = default;

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

    private:
        // joint handles: wheel angular joints and steer/pivot position joints
        hardware_interface::JointHandle front_left_pivot_joint_, front_right_pivot_joint_, back_left_pivot_joint_, back_right_pivot_joint_;
        hardware_interface::JointHandle front_left_wheel_joint_, front_right_wheel_joint_, back_left_wheel_joint_, back_right_wheel_joint_;

        // PIDs: one for each pivot position and one for each wheel velocity
        control_toolbox::Pid pid_lf_, pid_rf_, pid_lb_, pid_rb_;                         // pivot (position)
        control_toolbox::Pid pid_lf_wheel_, pid_rf_wheel_, pid_lb_wheel_, pid_rb_wheel_; // wheel (velocity)

        // parameters
        double wheel_track_{0.36};
        double wheel_base_{0.36};

        // desired commands
        double pivot_cmd_[4]{}; // desired pivot angles for 4 modules
        double wheel_cmd_[4]{}; // desired wheel speeds for 4 modules

        // cmd_vel subscriber
        ros::Subscriber cmd_vel_sub_;

    // dynamic_reconfigure server
    typedef sentry_chassis_controller::WheelPidConfig Config;
    std::shared_ptr<dynamic_reconfigure::Server<Config>> dyn_server_;
    void reconfigureCallback(Config &config, uint32_t level);

        void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg);

        // Helper functions for PID initialization
        void initPivot(const std::string &name, control_toolbox::Pid &pid,
                       ros::NodeHandle &controller_nh, double def_p, double def_i,
                       double def_d, double def_i_clamp, double def_antiwindup);
        void initWheel(const std::string &name, control_toolbox::Pid &pid,
                       ros::NodeHandle &controller_nh, double def_wp, double def_wi,
                       double def_wd, double def_wi_clamp, double def_wanti);
    };

} // namespace sentry_chassis_controller

#endif // SENTRY_CHASSIS_CONTROLLER_WHEEL_PID_CONTROLLER_H
