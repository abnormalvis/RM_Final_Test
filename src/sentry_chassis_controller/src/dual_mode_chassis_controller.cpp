/**
 * @file dual_mode_chassis_controller.cpp
 * @brief 双模式底盘控制器 - 根据rotation_matrix_chassis_application.md实现
 * 订阅分离的线性和角速度指令，通过旋转矩阵进行坐标系转换
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>
#include <cmath>

class DualModeChassisController
{
private:
    // ROS节点句柄
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_{"~"};

    // 订阅者
    ros::Subscriber sub_linear_absolute_; // odom坐标系下的线性速度
    ros::Subscriber sub_angular_direct_;  // 直接的角速度指令

    // 发布者
    ros::Publisher pub_cmd_vel_merged_; // 合并后的底盘速度指令

    // TF监听器
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_{tf_buffer_};

    // 速度缓存 - 线程安全
    mutable std::mutex mutex_;
    geometry_msgs::TwistStamped cmd_linear_absolute_; // odom坐标系下的线性速度
    geometry_msgs::Twist cmd_angular_direct_;         // 直接的角速度指令
    bool linear_updated_{false};
    bool angular_updated_{false};

    // 配置参数
    std::string odom_frame_;
    std::string base_frame_;
    double control_loop_rate_{20.0}; // Hz
    double tf_timeout_{0.2};         // seconds

public:
    DualModeChassisController()
        : tf_listener_(tf_buffer_), odom_frame_("odom"), base_frame_("base_link")
    {
        // 加载参数
        loadParameters();

        // 创建订阅者和发布者
        setupPublishersAndSubscribers();

        // 创建主控制循环定时器
        ros::Timer control_timer = nh_.createTimer(
            ros::Duration(1.0 / control_loop_rate_),
            &DualModeChassisController::controlLoop, this);

        // Initialize to zero - workaround for type issue
        cmd_linear_absolute_ = geometry_msgs::TwistStamped();
        cmd_angular_direct_ = geometry_msgs::Twist();

        ROS_INFO("Dual mode chassis controller initialized");
        ROS_INFO("Frame settings: odom=[%s], base=[%s]", odom_frame_.c_str(), base_frame_.c_str());
        ROS_INFO("Control loop rate: %.1f Hz", control_loop_rate_);

        ros::spin();
    }

    /**
     * @brief 加载配置参数
     */
    void loadParameters()
    {
        private_nh_.param("odom_frame", odom_frame_, std::string("odom"));
        private_nh_.param("base_frame", base_frame_, std::string("base_link"));
        private_nh_.param("control_loop_rate", control_loop_rate_, 20.0);
        private_nh_.param("tf_timeout", tf_timeout_, 0.2);
    }

    /**
     * @brief 创建ROS订阅者和发布者
     */
    void setupPublishersAndSubscribers()
    {
        sub_linear_absolute_ = nh_.subscribe(
            "/cmd_vel_linear_absolute", 10,
            &DualModeChassisController::linearAbsoluteCallback, this);

        sub_angular_direct_ = nh_.subscribe(
            "/cmd_vel_angular_direct", 10,
            &DualModeChassisController::angularDirectCallback, this);

        pub_cmd_vel_merged_ = nh_.advertise<geometry_msgs::Twist>(
            "/cmd_vel_merged", 10);
    }

    /**
     * @brief 接收odom坐标系下的线性速度指令
     */
    void linearAbsoluteCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_linear_absolute_ = *msg;
        linear_updated_ = true;
        ROS_DEBUG("Received odom-frame linear velocity: [%.3f, %.3f] m/s",
                  msg->twist.linear.x, msg->twist.linear.y);
    }

    /**
     * @brief 接收直接的角速度指令
     */
    void angularDirectCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_angular_direct_ = *msg;
        angular_updated_ = true;
        ROS_DEBUG("Received direct angular velocity: %.3f rad/s", msg->angular.z);
    }

    /**
     * @brief 主控制循环：执行坐标系转换和运动合成
     */
    void controlLoop(const ros::TimerEvent &)
    {
        // 获取最新的速度指令
        geometry_msgs::TwistStamped local_linear_abs;
        geometry_msgs::Twist local_angular;
        bool has_linear = false;
        bool has_angular = false;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            local_linear_abs = cmd_linear_absolute_;
            local_angular = cmd_angular_direct_;
            has_linear = linear_updated_;
            has_angular = angular_updated_;
            linear_updated_ = false;
            angular_updated_ = false;
        }

        // 如果没有收到任何指令，发布零速度
        if (!has_linear && !has_angular)
        {
            geometry_msgs::Twist zero_cmd;
            zero_cmd.linear.x = 0.0;
            zero_cmd.linear.y = 0.0;
            zero_cmd.angular.z = 0.0;
            pub_cmd_vel_merged_.publish(zero_cmd);
            return;
        }

        // 获取坐标变换矩阵：odom -> base_link
        geometry_msgs::TransformStamped transform_odom_to_base;
        try
        {
            transform_odom_to_base = tf_buffer_.lookupTransform(
                base_frame_, odom_frame_, ros::Time(0), ros::Duration(tf_timeout_));
            ROS_DEBUG("TF found: %s -> %s", transform_odom_to_base.header.frame_id.c_str(),
                      transform_odom_to_base.child_frame_id.c_str());
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(1.0, "TF lookup failure: %s", ex.what());
            // TF不可用，保持当前状态
            return;
        }

        // 核心：坐标转换和速度合成
        geometry_msgs::Twist base_frame_cmd;

        // 转换线性速度（odom系 -> base系）
        if (has_linear)
        {
            // 手动实现2D旋转矩阵变换：odom -> base
            // 从四元数获取yaw角：quaternion to yaw = 2*arctan2(z, w) (假设roll=0, pitch=0)
            double w = transform_odom_to_base.transform.rotation.w;
            double z = transform_odom_to_base.transform.rotation.z;
            double yaw_angle = 2.0 * std::atan2(z, w);

            double cos_yaw = std::cos(yaw_angle);
            double sin_yaw = std::sin(yaw_angle);

            // odom坐标系下的速度
            double vx_odom = local_linear_abs.twist.linear.x;
            double vy_odom = local_linear_abs.twist.linear.y;

            // 应用旋转矩阵转换到base坐标系
            // 坐标系转换公式：v_base = R(-yaw) * v_odom
            // 其中R(-yaw) = [ cos(yaw)  sin(yaw) ; -sin(yaw)  cos(yaw) ]
            base_frame_cmd.linear.x = cos_yaw * vx_odom + sin_yaw * vy_odom;
            base_frame_cmd.linear.y = -sin_yaw * vx_odom + cos_yaw * vy_odom;

            ROS_INFO("Linear conversion: [%.3f, %.3f] -> [%.3f, %.3f] m/s (yaw=%.3f rad)",
                     vx_odom, vy_odom,
                     base_frame_cmd.linear.x, base_frame_cmd.linear.y,
                     yaw_angle);
        }
        else
        {
            base_frame_cmd.linear.x = 0.0;
            base_frame_cmd.linear.y = 0.0;
        }

        // 直接使用角速度（无需转换）
        base_frame_cmd.angular.z = local_angular.angular.z;

        // 发布最终合并的速度指令
        pub_cmd_vel_merged_.publish(base_frame_cmd);

        // 定期状态输出
        ROS_INFO_THROTTLE(2.0, "Controller state: linearx=%.3f, lineary=%.3f, angular=%.3f",
                          base_frame_cmd.linear.x, base_frame_cmd.linear.y, base_frame_cmd.angular.z);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_mode_chassis_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        DualModeChassisController controller;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Controller initialization failed: %s", e.what());
        return EXIT_FAILURE;
    }
    catch (ros::Exception &e)
    {
        ROS_ERROR("ROS exception: %s", e.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}