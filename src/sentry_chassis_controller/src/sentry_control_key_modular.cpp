#include <ros/ros.h>
#include "sentry_chassis_controller/teleop_node.hpp"

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "sentry_control_key_modular");

    // 创建节点实例
    sentry_chassis_controller::TeleopNode node;

    // 运行主循环
    node.run();

    return 0;
}
