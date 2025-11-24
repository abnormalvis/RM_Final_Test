#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_odom_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string model_name;
    std::string base_link_frame;
    std::string odom_frame;
    pnh.param<std::string>("model_name", model_name, "sentry");
    pnh.param<std::string>("base_link_frame", base_link_frame, "base_link");
    pnh.param<std::string>("odom_frame", odom_frame, "odom");

    // 里程计发布方
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    // TF广播方
    tf2_ros::TransformBroadcaster tf_broadcaster;


    //typedef gazebo_msgs::ModelStates_<std::allocator<void>> gazebo_msgs::ModelStates
    auto cb = [=, &odom_pub, &tf_broadcaster](const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        // 找到模型索引
        size_t idx = std::string::npos;
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == model_name)
            {
                idx = i;
                break;
            }
        }
        if (idx == std::string::npos)
            return; // 没有找到模型的索引

        // 组合消息并发布模型的位姿和速度
        geometry_msgs::Pose pose = msg->pose[idx];
        geometry_msgs::Twist twist = msg->twist[idx];

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = odom_frame;
        odom.child_frame_id = base_link_frame;
        odom.pose.pose = pose;
        odom.twist.twist = twist;

        // 发布里程计消息
        odom_pub.publish(odom);

        // 组织坐标变换
        geometry_msgs::TransformStamped t;
        t.header.stamp = odom.header.stamp;
        t.header.frame_id = odom_frame;
        t.child_frame_id = base_link_frame;
        t.transform.translation.x = pose.position.x;
        t.transform.translation.y = pose.position.y;
        t.transform.translation.z = pose.position.z;
        t.transform.rotation = pose.orientation;

        tf_broadcaster.sendTransform(t);
    };

    ros::Subscriber sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, cb);

    ROS_INFO("gazebo_odom_bridge started for model '%s' publishing /odom and TF %s -> %s", model_name.c_str(), odom_frame.c_str(), base_link_frame.c_str());

    ros::spin();
    return 0;
}
