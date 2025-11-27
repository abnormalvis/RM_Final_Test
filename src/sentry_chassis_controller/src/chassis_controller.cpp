/*
 * ChassisController: 方案A 旋转矩阵解算
 * 订阅：/cmd_vel_linear_absolute (TwistStamped, frame_id=odom)
 *      /cmd_vel_angular_direct (Twist)
 * TF：  lookupTransform(base_link, odom)
 * 合成：R(yaw) * v_odom -> v_base, 角速度直接取 angular.z
 * 发布：/cmd_vel_final (Twist)
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>
#include <cmath>

class ChassisController
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_linear_, sub_angular_;
    ros::Publisher pub_cmd_vel_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    geometry_msgs::TwistStamped cmd_linear_odom_;
    geometry_msgs::Twist cmd_angular_;
    std::mutex mutex_;

    std::string odom_frame_ = "odom";
    std::string base_frame_ = "base_link";

    ros::Timer timer_;

public:
    ChassisController() : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle pnh("~");
        pnh.param("odom_frame", odom_frame_, odom_frame_);
        pnh.param("base_frame", base_frame_, base_frame_);

        sub_linear_ = nh_.subscribe("/cmd_vel_linear_absolute", 10,
                                    &ChassisController::linearCallback, this);
        sub_angular_ = nh_.subscribe("/cmd_vel_angular_direct", 10,
                                     &ChassisController::angularCallback, this);
        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_final", 10);

        timer_ = nh_.createTimer(ros::Duration(0.05), &ChassisController::controlLoop, this);
        ROS_INFO("[ChassisController] rotation-matrix resolver started (odom=%s, base=%s)",
                 odom_frame_.c_str(), base_frame_.c_str());
    }

    void linearCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_linear_odom_ = *msg;
    }

    void angularCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_angular_ = *msg;
    }

    void controlLoop(const ros::TimerEvent &)
    {
        geometry_msgs::TwistStamped linear_odom;
        geometry_msgs::Twist angular;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            linear_odom = cmd_linear_odom_;
            angular = cmd_angular_;
        }

        geometry_msgs::TransformStamped transform;
        try
        {
            transform = tf_buffer_.lookupTransform(
                base_frame_, odom_frame_, ros::Time(0), ros::Duration(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(5.0, "TF lookup failed: %s", ex.what());
            return;
        }

        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        const double cos_yaw = std::cos(yaw);
        const double sin_yaw = std::sin(yaw);

        const double vx_odom = linear_odom.twist.linear.x;
        const double vy_odom = linear_odom.twist.linear.y;

        // R(yaw) * v_odom 旋转矩阵解算
        const double vx_base = cos_yaw * vx_odom + sin_yaw * vy_odom;
        const double vy_base = -sin_yaw * vx_odom + cos_yaw * vy_odom;

        geometry_msgs::Twist final_cmd;
        final_cmd.linear.x = vx_base;
        final_cmd.linear.y = vy_base;
        final_cmd.linear.z = 0.0;
        final_cmd.angular.x = 0.0;
        final_cmd.angular.y = 0.0;
        final_cmd.angular.z = angular.angular.z;

        pub_cmd_vel_.publish(final_cmd);

        static int cnt = 0;
        if (++cnt % 20 == 0)
        {
            ROS_INFO("Yaw: %.3f | Odom[%.2f, %.2f] -> Base[%.2f, %.2f]",
                     yaw, vx_odom, vy_odom, vx_base, vy_base);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chassis_controller");
    ChassisController ctrl;
    ros::spin();
    return 0;
}
