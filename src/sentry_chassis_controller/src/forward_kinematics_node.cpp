#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>

#include <string>
#include <vector>
#include <unordered_map>

class ForwardKinematicsNode
{
public:
    ForwardKinematicsNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh)
    {
        // params
        pnh_.param("wheel_base", wheel_base_, 0.36);
        pnh_.param("wheel_track", wheel_track_, 0.36);
        pnh_.param("wheel_radius", wheel_radius_, 0.05);
        pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
        pnh_.param<std::string>("base_link_frame", base_link_frame_, "base_link");

        // default joint names
        wheel_joint_names_ = {"left_front_wheel_joint", "right_front_wheel_joint", "left_back_wheel_joint", "right_back_wheel_joint"};
        pivot_joint_names_ = {"left_front_pivot_joint", "right_front_pivot_joint", "left_back_pivot_joint", "right_back_pivot_joint"};

        // allow override from params (list)
        pnh_.getParam("wheel_joints", wheel_joint_names_);
        pnh_.getParam("pivot_joints", pivot_joint_names_);

        joint_sub_ = nh_.subscribe("/joint_states", 10, &ForwardKinematicsNode::jointStatesCB, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

        last_time_ = ros::Time::now();
        x_ = 0.0;
        y_ = 0.0;
        yaw_ = 0.0;
    }

private:
    void jointStatesCB(const sensor_msgs::JointState::ConstPtr &msg)
    {
        ros::Time now = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        double dt = (now - last_time_).toSec();
        if (dt <= 0.0)
            dt = 1e-3;

        // build map name->index
        std::unordered_map<std::string, size_t> idx;
        for (size_t i = 0; i < msg->name.size(); ++i)
            idx[msg->name[i]] = i;

        // read per-wheel linear velocities and steer angles
        Eigen::VectorXd b(4);
        Eigen::MatrixXd A(4, 3);
        A.setZero();

        for (int i = 0; i < 4; ++i)
        {
            double wheel_vel = 0.0; // rad/s
            double pivot_pos = 0.0; // rad
            if (idx.find(wheel_joint_names_[i]) != idx.end())
            {
                size_t j = idx[wheel_joint_names_[i]];
                if (j < msg->velocity.size())
                    wheel_vel = msg->velocity[j];
            }
            if (idx.find(pivot_joint_names_[i]) != idx.end())
            {
                size_t j = idx[pivot_joint_names_[i]];
                if (j < msg->position.size())
                    pivot_pos = msg->position[j];
            }

            double v_along = wheel_vel * wheel_radius_;
            // wheel positions relative to base_link
            double rx = (i < 2 ? wheel_base_ / 2.0 : -wheel_base_ / 2.0);
            double ry = (i % 2 == 0 ? wheel_track_ / 2.0 : -wheel_track_ / 2.0); // FL,FR,RL,RR mapping

            double cos_t = std::cos(pivot_pos);
            double sin_t = std::sin(pivot_pos);

            A(i, 0) = cos_t;
            A(i, 1) = sin_t;
            A(i, 2) = -ry * cos_t + rx * sin_t;
            b(i) = v_along;
        }

        // least squares solution for [vx vy wz]
        Eigen::Vector3d sol = (A.transpose() * A).ldlt().solve(A.transpose() * b);
        double vx = sol(0);
        double vy = sol(1);
        double wz = sol(2);

        // integrate pose (body -> world)
        double cos_y = std::cos(yaw_);
        double sin_y = std::sin(yaw_);
        double world_vx = vx * cos_y - vy * sin_y;
        double world_vy = vx * sin_y + vy * cos_y;

        x_ += world_vx * dt;
        y_ += world_vy * dt;
        yaw_ += wz * dt;

        // publish odom
        nav_msgs::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_link_frame_;
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
        odom.pose.pose.orientation.w = std::cos(yaw_ * 0.5);
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = wz;

        odom_pub_.publish(odom);

        // broadcast TF
        geometry_msgs::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = odom_frame_;
        t.child_frame_id = base_link_frame_;
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        t.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_.sendTransform(t);

        last_time_ = now;
    }

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber joint_sub_;
    ros::Publisher odom_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    double wheel_base_, wheel_track_, wheel_radius_;
    std::string odom_frame_, base_link_frame_;
    std::vector<std::string> wheel_joint_names_, pivot_joint_names_;

    double x_, y_, yaw_;
    ros::Time last_time_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forward_kinematics_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ForwardKinematicsNode node(nh, pnh);
    ros::spin();
    return 0;
}
