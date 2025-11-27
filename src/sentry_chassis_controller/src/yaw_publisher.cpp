#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class YawPublisher
{
public:
    YawPublisher()
    {
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("odom_topic", odom_topic_, "/odom");
        pnh.param<std::string>("yaw_topic", yaw_topic_, "/yaw");

        yaw_pub_ = nh_.advertise<std_msgs::Float64>(yaw_topic_, 10);
        odom_sub_ = nh_.subscribe(odom_topic_, 10, &YawPublisher::odomCB, this);

        ROS_INFO("YawPublisher: subscribing to %s, publishing %s", odom_topic_.c_str(), yaw_topic_.c_str());
    }

private:
    void odomCB(const nav_msgs::Odometry::ConstPtr &msg)
    {
        const auto &q = msg->pose.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        std_msgs::Float64 out;
        out.data = yaw;
        yaw_pub_.publish(out);
    }

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher yaw_pub_;
    std::string odom_topic_;
    std::string yaw_topic_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yaw_publisher");
    YawPublisher node;
    ros::spin();
    return 0;
}
