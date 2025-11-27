#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>

int main() {
    geometry_msgs::TwistStamped test_msg;

    // 正确访问方式测试
    std::cout << "Testing geometry_msgs::TwistStamped structure" << std::endl;
    std::cout << "test_msg.twist type: " << typeid(test_msg.twist).name() << std::endl;
    std::cout << "test_msg.twist.linear type: " << typeid(test_msg.twist.linear).name() << std::endl;

    // 尝试直接访问
    test_msg.twist.linear.x = 1.0;
    std::cout << "test_msg.twist.linear.x = " << test_msg.twist.linear.x << std::endl;

    return 0;
}