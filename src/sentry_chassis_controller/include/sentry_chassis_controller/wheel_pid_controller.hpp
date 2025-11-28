#ifndef SENTRY_CHASSIS_CONTROLLER_WHEEL_PID_CONTROLLER_H
#define SENTRY_CHASSIS_CONTROLLER_WHEEL_PID_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <sentry_chassis_controller/WheelPidConfig.h>
#include <sentry_chassis_controller/inverse_kinematics.hpp>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

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
      double wheel_radius_{0.05};

        // power limiting parameters (borrowed from hero_chassis_controller)
        double power_limit_{0.0};
        double effort_coeff_{0.0};
        double velocity_coeff_{0.0};
        double power_offset_{0.0};
        bool power_limit_enabled_{false};

        // geometry helpers
        double rx_{0.0}; // wheel_track/2
        double ry_{0.0}; // wheel_base/2

        // desired commands
        double pivot_cmd_[4]{}; // desired pivot angles for 4 modules
        double wheel_cmd_[4]{}; // desired wheel speeds for 4 modules

        // cmd_vel subscriber
        ros::Subscriber cmd_vel_sub_;
        std::string cmd_vel_topic_{"/cmd_vel"}; // configurable cmd_vel topic (absolute recommended)
        bool synthetic_fallback_{false};         // guard synthetic joint state fallback
      // publisher for desired commands (for testing/inspection)
      ros::Publisher desired_pub_;
      // NEW: publisher for actual joint states (CRITICAL FIX)
      ros::Publisher joint_states_pub_;
      ros::Time last_state_pub_; // timestamp tracking for joint states

  // Odometry publishing integrated into controller
  ros::Publisher odom_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::string odom_frame_{"odom"};
  std::string base_link_frame_{"base_link"};
  // odom state
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  ros::Time last_odom_time_;

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

  // Helpers
    void publishOdometry_(const ros::Time &stamp, double vx_body, double vy_body, double wz);
    void odom_update(const ros::Time &time, const ros::Duration &period);
    };

} // namespace sentry_chassis_controller

#endif // SENTRY_CHASSIS_CONTROLLER_WHEEL_PID_CONTROLLER_H
