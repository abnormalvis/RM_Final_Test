#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>

#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>

class ForwardKinematics
{
public:
    ForwardKinematics(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh)
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

        joint_sub_ = nh_.subscribe("/joint_states", 10, &ForwardKinematics::jointStatesCB, this);
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

        // protect against large/negative dt on first callback or out-of-order stamps
        if (last_time_.isZero() || last_time_ > now)
        {
            last_time_ = now;
        }
        double dt = (now - last_time_).toSec();
        if (dt <= 0.0)
            dt = 1e-3;

        // Debug log: print all joint names in the message
        if (msg->name.size() > 0) {
            ROS_INFO("=== Joint States Message (dt=%.3f) ===", dt);
            ROS_INFO("Joint names in message (%lu):", msg->name.size());
            for (size_t i = 0; i < msg->name.size(); ++i) {
                std::string vel_str = "none";
                std::string pos_str = "none";
                if (i < msg->velocity.size()) vel_str = std::to_string(msg->velocity[i]);
                if (i < msg->position.size()) pos_str = std::to_string(msg->position[i]);

                // Check if this is one of our expected wheel joints
                bool is_expected_wheel = false;
                for (int j = 0; j < 4; ++j) {
                    if (msg->name[i] == wheel_joint_names_[j]) {
                        is_expected_wheel = true;
                        break;
                    }
                }
                if (is_expected_wheel) {
                    ROS_INFO("  [%lu] <DCB> %s: pos=%s, vel=%s", i, msg->name[i].c_str(), pos_str.c_str(), vel_str.c_str());
                } else {
                    ROS_INFO("  [%lu] %s: pos=%s, vel=%s", i, msg->name[i].c_str(), pos_str.c_str(), vel_str.c_str());
                }
            }
            ROS_INFO("Velocity size: %lu, Position size: %lu", msg->velocity.size(), msg->position.size());
            ROS_INFO("=== End Joint States Message ===");
        }

        // build map name->index
        std::unordered_map<std::string, size_t> idx;
        for (size_t i = 0; i < msg->name.size(); ++i)
            idx[msg->name[i]] = i;

        // quick check: are expected wheel joints present?
        int found_wheels = 0;
        for (int i = 0; i < 4; ++i)
        {
            if (idx.find(wheel_joint_names_[i]) != idx.end()) {
                ++found_wheels;
                ROS_INFO_THROTTLE(5.0, "Found wheel joint: %s at index %lu", wheel_joint_names_[i].c_str(), idx[wheel_joint_names_[i]]);
            }
        }

        ROS_INFO_THROTTLE(5.0, "Expected joints:");
        for (int i = 0; i < 4; ++i) {
            ROS_INFO_THROTTLE(5.0, "  %s", wheel_joint_names_[i].c_str());
        }

        if (found_wheels == 0)
        {
            ROS_WARN_THROTTLE(5.0, "forward_kinematics: no wheel joints found in /joint_states (names: %lu)", msg->name.size());
            // still update last_time_ to avoid dt blowup
            last_time_ = now;
            return;
        }

        ROS_INFO_THROTTLE(5.0, "Found %d wheel joints out of 4", found_wheels);

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
                // prefer velocity if available
                bool use_velocity = false;
                if (j < msg->velocity.size())
                {
                    wheel_vel = msg->velocity[j];
                    use_velocity = true;
                }
                // POSITION-BASED VELOCITY ESTIMATION (PRIMARY FOR YOUR ROBOT)
                if (j < msg->position.size()) {
                    double cur_pos = msg->position[j];
                    auto it = prev_wheel_pos_.find(wheel_joint_names_[i]);
                    if (it != prev_wheel_pos_.end() && !prev_wheel_pos_.empty() && dt > 0.0 && dt < 1.0)
                    {
                        double prev = it->second;
                        double delta = cur_pos - prev;
                        // simple unwrap if crossing +/-pi (optional)
                        if (delta > M_PI)
                            delta -= 2.0 * M_PI;
                        else if (delta < -M_PI)
                            delta += 2.0 * M_PI;

                        // Always use position-based estimation (your robot has no velocity data)
                        if (!use_velocity) {
                            wheel_vel = delta / dt;
                            ROS_INFO_THROTTLE(5.0, "Position-based velocity for %s: delta=%f, dt=%f => vel=%f rad/s",
                                             wheel_joint_names_[i].c_str(), delta, dt, wheel_vel);
                        }
                    }
                    prev_wheel_pos_[wheel_joint_names_[i]] = cur_pos;
                }
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

            ROS_INFO_THROTTLE(5.0, "Wheel %d: vel=%.3f rad/s, v_along=%.3f m/s, pivot=%.3f rad", i, wheel_vel, v_along, pivot_pos);
            ROS_INFO_THROTTLE(5.0, "  A row %d: [%f, %f, %f], b=%f", i, A(i,0), A(i,1), A(i,2), b(i));
        }

        // least squares solution for [vx vy wz]
        Eigen::Vector3d sol = (A.transpose() * A).ldlt().solve(A.transpose() * b);
        double vx = sol(0);
        double vy = sol(1);
        double wz = sol(2);

        ROS_INFO_THROTTLE(5.0, "=== Odometry Calculation ===");
        ROS_INFO_THROTTLE(5.0, "Solved velocities: vx=%.3f, vy=%.3f, wz=%.3f", vx, vy, wz);
        ROS_INFO_THROTTLE(5.0, "Current world pose: x=%.3f, y=%.3f, yaw=%.3f", x_, y_, yaw_);

        // integrate pose (body -> world)
        double cos_y = std::cos(yaw_);
        double sin_y = std::sin(yaw_);
        double world_vx = vx * cos_y - vy * sin_y;
        double world_vy = vx * sin_y + vy * cos_y;

        x_ += world_vx * dt;
        y_ += world_vy * dt;
        yaw_ += wz * dt;

        ROS_INFO_THROTTLE(5.0, "Updated world pose: x=%.3f, y=%.3f, yaw=%.3f (dt=%.3f)", x_, y_, yaw_, dt);

        // publish odom
        nav_msgs::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_link_frame_;
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        // full quaternion (assume planar robot)
        double half_yaw = yaw_ * 0.5;
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = std::sin(half_yaw);
        odom.pose.pose.orientation.w = std::cos(half_yaw);
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = wz;

        // fill reasonable covariances: small on x,y,yaw; large on unused axes
        // pose.covariance and twist.covariance are 6x6 flattened row-major
        for (size_t i = 0; i < 36; ++i)
        {
            odom.pose.covariance[i] = 0.0;
            odom.twist.covariance[i] = 0.0;
        }
        // pose: var(x), var(y), var(z), var(roll), var(pitch), var(yaw)
        odom.pose.covariance[0] = 1e-3;  // x
        odom.pose.covariance[7] = 1e-3;  // y
        odom.pose.covariance[14] = 1e9;  // z (unused)
        odom.pose.covariance[21] = 1e9;  // roll
        odom.pose.covariance[28] = 1e9;  // pitch
        odom.pose.covariance[35] = 1e-2; // yaw
        // twist: var(vx), var(vy), var(vz), var(v_roll), var(v_pitch), var(v_yaw)
        odom.twist.covariance[0] = 1e-3;  // vx
        odom.twist.covariance[7] = 1e-3;  // vy
        odom.twist.covariance[14] = 1e9;  // vz
        odom.twist.covariance[21] = 1e9;  // v_roll
        odom.twist.covariance[28] = 1e9;  // v_pitch
        odom.twist.covariance[35] = 1e-2; // v_yaw

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

    // store previous wheel positions to estimate velocity when velocity[] is absent
    std::unordered_map<std::string, double> prev_wheel_pos_;

    double x_, y_, yaw_;
    ros::Time last_time_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forward_kinematics");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ForwardKinematics node(nh, pnh);
    ros::spin();
    return 0;
}