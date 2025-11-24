#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ik_test_publisher");
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Rate rate(1.0); // 1 Hz

  double vx = 0.5;
  double vy = 0.0;
  double wz = 0.3;

  ROS_INFO("ik_test_publisher started: publishing cmd_vel at 1Hz");
  while (ros::ok()) {
    geometry_msgs::Twist t;
    t.linear.x = vx;
    t.linear.y = vy;
    t.angular.z = wz;
    pub.publish(t);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
