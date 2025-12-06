/*
 * TeleopNode - 精简的遥操作节点
 * 职责：ROS通信、参数管理、模块协调
 */

#ifndef SENTRY_CHASSIS_CONTROLLER_TELEOP_NODE_HPP
#define SENTRY_CHASSIS_CONTROLLER_TELEOP_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <memory>

#include "sentry_chassis_controller/keyboard_input.hpp"
#include "sentry_chassis_controller/teleop_state_machine.hpp"
#include "sentry_chassis_controller/velocity_calculator.hpp"

namespace sentry_chassis_controller
{
    /*整合键盘输入、状态机、速度计算，负责ROS通信*/
    class TeleopNode
    {
    public:
        TeleopNode();
        ~TeleopNode();

        // 主循环监听键盘并发布速度命令
        void run();

        // 打印使用说明
        void print_usage();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Publisher twist_pub_;
        std::unique_ptr<KeyboardInput> keyboard_input_;     // 键盘输入
        std::unique_ptr<TeleopStateMachine> state_machine_; // 状态机
        std::unique_ptr<VelocityCalculator> velocity_calc_; // 速度计算

        // 参数
        std::string cmd_vel_topic_;
        bool publish_zero_when_idle_;
        int publish_rate_hz_;

        // 加载参数
        void load_parameters();

        // 发布速度命令
        void publish_velocity(const VelocityCalcOutput &output, bool force_publish = false);
    };

} // namespace sentry_chassis_controller

#endif
