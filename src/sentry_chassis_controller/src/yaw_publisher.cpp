#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class YawPublisher
{
public:
    YawPublisher()
        : nh_(), pnh_("~"), tf_buffer_(ros::Duration(10.0)), tf_listener_(tf_buffer_)
    {
        pnh_.param<std::string>("odom_topic", odom_topic_, "/odom");
        // 不再对外发布 yaw，仅用于监视
        pnh_.param<std::string>("global_frame", global_frame_, "odom");
        pnh_.param<std::string>("base_link_frame", base_link_frame_, "base_link");
        pnh_.param<std::string>("source", source_, std::string("auto")); // tf | odom | auto
        pnh_.param<double>("hz", hz_, 50.0);
        pnh_.param<double>("tf_timeout_sec", tf_timeout_sec_, 0.05);
        pnh_.param<double>("stale_sec", stale_sec_, 1.0);

        odom_sub_ = nh_.subscribe(odom_topic_, 10, &YawPublisher::odomCB, this);
        timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, hz_)), &YawPublisher::onTimer, this);

    ROS_INFO("航向监视器已启动: 数据源=%s, TF(%s->%s), odom=%s, 频率=%.1fHz (仅监视)", source_.c_str(),
         global_frame_.c_str(), base_link_frame_.c_str(), odom_topic_.c_str(), hz_);
    }

private:
    void onTimer(const ros::TimerEvent &)
    {
        // 1) Try TF
        bool tf_ok = false;
        double yaw_tf = last_tf_yaw_;
        try
        {
            auto tfst = tf_buffer_.lookupTransform(global_frame_, base_link_frame_, ros::Time(0), ros::Duration(tf_timeout_sec_));
            const auto &qr = tfst.transform.rotation;
            tf2::Quaternion q(qr.x, qr.y, qr.z, qr.w);
            double r, p, y;
            tf2::Matrix3x3(q).getRPY(r, p, y);
            yaw_tf = y;
            tf_ok = true;
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(5.0, "yaw_publisher: TF 查询 %s->%s 失败：%s", global_frame_.c_str(), base_link_frame_.c_str(), ex.what());
        }

        if (tf_ok)
        {
            if (std::fabs(yaw_tf - last_tf_yaw_) > 1e-6)
            {
                last_tf_yaw_ = yaw_tf;
                last_tf_change_time_ = ros::Time::now();
            }
        }

        const bool tf_stale = (ros::Time::now() - last_tf_change_time_).toSec() > stale_sec_;

        double yaw_out = last_tf_yaw_;
        if (source_ == "tf")
        {
            if (tf_ok)
                yaw_out = yaw_tf;
        }
        else if (source_ == "odom")
        {
            if (have_odom_yaw_)
                yaw_out = yaw_odom_;
        }
        else
        { // auto
            if (tf_ok && !tf_stale)
            {
                yaw_out = yaw_tf;
            }
            else if (have_odom_yaw_)
            {
                yaw_out = yaw_odom_;
                ROS_WARN_THROTTLE(2.0, "yaw_publisher: 回退到 odom 航向 (tf stale=%d ok=%d)", (int)tf_stale, (int)tf_ok);
            }
        }

        // 监视与日志输出
    ROS_INFO_THROTTLE(1.0, "航向监视: src=%s yaw=%.3f (tf_ok=%d tf_stale=%d yaw_tf=%.3f have_odom=%d yaw_odom=%.3f)",
              source_.c_str(), yaw_out, (int)tf_ok, (int)tf_stale, yaw_tf, (int)have_odom_yaw_, yaw_odom_);
    }

    void odomCB(const nav_msgs::Odometry::ConstPtr &msg)
    {
        const auto &q = msg->pose.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double r, p, y;
        tf2::Matrix3x3(quat).getRPY(r, p, y);
        yaw_odom_ = y;
        have_odom_yaw_ = true;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Timer timer_;

    ros::Subscriber odom_sub_;
    std::string odom_topic_;
    std::string global_frame_;
    std::string base_link_frame_;
    std::string source_;
    double hz_ = 50.0;
    double tf_timeout_sec_ = 0.05;
    double stale_sec_ = 1.0;

    double last_tf_yaw_ = 0.0;
    ros::Time last_tf_change_time_ = ros::Time(0);

    double yaw_odom_ = 0.0;
    bool have_odom_yaw_ = false;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yaw_publisher");
    YawPublisher node;
    ros::spin();
    return 0;
}
