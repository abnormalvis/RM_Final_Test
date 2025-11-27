#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <csignal>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <cmath>
#include <mutex>
#include <memory>

namespace sentry_control
{

    // 维护常量
    constexpr double VELOCITY_EPSILON = 1e-9;
    constexpr double TF_LOOKUP_TIMEOUT = 0.05;       // 50ms
    constexpr double TF_VALID_MAX_AGE = 0.2;         // 200ms
    constexpr double DEFAULT_LINEAR_VELOCITY = 1.0;  // m/s
    constexpr double DEFAULT_ANGULAR_VELOCITY = 1.0; // rad/s
    constexpr double SPINNING_TOP_LINEAR_VEL = 1.0;  // m/s for spinning top mode
    constexpr double SPINNING_TOP_ANGULAR_VEL = 3.0; // rad/s for spinning top mode
    constexpr int KEYBOARD_POLL_TIMEOUT = 100;       // ms for poll

    // 全局退出标志
    static bool g_quit = false;

    class KeyboardControlNode
    {
    public:
        KeyboardControlNode(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
        ~KeyboardControlNode();

        void run();
        KeyboardControlNode() = delete;
        KeyboardControlNode(const KeyboardControlNode &) = delete;
        KeyboardControlNode &operator=(const KeyboardControlNode &) = delete;

    private:
        void loadParameters();
        void setupPublishers();
        void setupSubscribers();
        void resetVelocityStates();
        bool waitForTransform();
        void updateTransform();
        void processKeyboardInput();
        void updateAndPublish();
        void handleSpinningTopMode();
        geometry_msgs::Twist convertToRobotFrame(const geometry_msgs::Twist &world_vel);
        void publishCommands(bool has_motion);
        void publishStopCommand();
        void flushInputBuffer();
        void printHelp();
        bool shouldQuit() const { return g_quit || quit_requested_; }

        // ROS接口
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        // 速度指令发布者
        ros::Publisher cmd_vel_pub_;
        ros::Publisher cmd_vel_stamped_pub_;

        // TF组件
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

        // 状态变量
        bool field_centric_;    // 当为 true 时，键盘输入表示世界坐标系下的方向，旋转不影响平移方向
        bool publish_zero_when_idle_;   // 当机器人处于空闲状态时，是否发布零速度
        bool spinning_top_mode_;        // 小陀螺模式开关
        bool spinning_active_;          // 小陀螺模式激活状态
        bool quit_requested_;           // 退出请求标志

        // TF状态
        bool tf_valid_{false};
        bool tf_updated_{false};
        double yaw_{0.0};
        double cached_cos_yaw_{1.0};
        double cached_sin_yaw_{0.0};
        geometry_msgs::TransformStamped last_transform_;    // 上一次有效的变换
        ros::Time last_tf_update_;

        // 速度状态
        geometry_msgs::Twist current_twist_;
        geometry_msgs::Twist target_world_vel_;

        // 速度参数
        double vel_scale_linear_;
        double vel_scale_angular_;
        double vel_scale_smart_linear_;
        double vel_scale_smart_angular_;
    };

    KeyboardControlNode::KeyboardControlNode(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
        : nh_(nh),
          private_nh_(private_nh),
          tf_buffer_(std::make_unique<tf2_ros::Buffer>()),
          tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, nh_)),
          quit_requested_(false),
          field_centric_(true),
          publish_zero_when_idle_(true),
          spinning_top_mode_(false),
          spinning_active_(false),
          last_tf_update_(ros::Time::now())
    {
        loadParameters();
        setupPublishers();
        setupSubscribers();
        resetVelocityStates();
    }

    // 析构函数
    KeyboardControlNode::~KeyboardControlNode()
    {
        struct termios term;
        if (tcgetattr(STDIN_FILENO, &term) == 0)
        {
            // 恢复终端设置
            term.c_lflag |= (ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &term);
        }
    }

    void KeyboardControlNode::run()
    {
        ros::Rate rate(50); // 50Hz control loop

        // 等待 TF 可用
        if (!waitForTransform())
        {
            // 等待
            ROS_ERROR("Failed to get TF transform. Exiting.");
            return;
        }

        ROS_INFO("Keyboard control node started. Use 'h' for help, 'q' to quit");

        // 主循环只要roscore处于运行状态，run函数就会一直运行
        while (ros::ok() && !quit_requested_ && !shouldQuit())
        {
            // 更新 TF 状态，然后获取键盘输入，处理并发布速度指令
            updateTransform();
            waitForTransform();
            processKeyboardInput();
            updateAndPublish();
            ros::spinOnce();
            rate.sleep();
        }

        // 退出时停止机器人
        publishStopCommand();
    }

    void KeyboardControlNode::loadParameters()
    {
        private_nh_.param("field_centric", field_centric_, true);
        private_nh_.param("publish_zero_when_idle", publish_zero_when_idle_, true);

        private_nh_.param("vel_scale_linear", vel_scale_linear_, DEFAULT_LINEAR_VELOCITY);
        private_nh_.param("vel_scale_angular", vel_scale_angular_, DEFAULT_ANGULAR_VELOCITY);
        private_nh_.param("vel_scale_smart_linear", vel_scale_smart_linear_, SPINNING_TOP_LINEAR_VEL);
        private_nh_.param("vel_scale_smart_angular", vel_scale_smart_angular_, SPINNING_TOP_ANGULAR_VEL);

        ROS_INFO("Parameters loaded:");
        ROS_INFO("  Field centric: %s", field_centric_ ? "true" : "false");
        ROS_INFO("  Publish zero when idle: %s", publish_zero_when_idle_ ? "true" : "false");
        ROS_INFO("  Linear velocity scale: %.2f m/s", vel_scale_linear_);
        ROS_INFO("  Angular velocity scale: %.2f rad/s", vel_scale_angular_);
    }

    void KeyboardControlNode::setupPublishers()
    {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        cmd_vel_stamped_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped", 1);
    }

    void KeyboardControlNode::setupSubscribers()
    {
        }

    void KeyboardControlNode::resetVelocityStates()
    {
        current_twist_.linear.x = 0.0;
        current_twist_.linear.y = 0.0;
        current_twist_.linear.z = 0.0;
        current_twist_.angular.x = 0.0;
        current_twist_.angular.y = 0.0;
        current_twist_.angular.z = 0.0;

        target_world_vel_.linear.x = 0.0;
        target_world_vel_.linear.y = 0.0;
        target_world_vel_.angular.z = 0.0;
    }

    bool KeyboardControlNode::waitForTransform()
    {
        ROS_INFO("Waiting for TF transform from odom to base_link...");

        ros::Rate try_rate(10);
        int max_attempts = 50; // 5 seconds total

        for (int i = 0; i < max_attempts && ros::ok(); ++i)
        {
            try
            {
                // 尝试获取变换
                geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
                    "odom", "base_link", ros::Time(0), ros::Duration(0.05));

                if ((ros::Time::now() - transform.header.stamp).toSec() < TF_VALID_MAX_AGE)
                {
                    // 获取变换的欧拉角
                    const auto &q = transform.transform.rotation;

                    // 计算偏航角
                    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

                    // 更新tf状态
                    last_transform_ = transform;
                    last_tf_update_ = ros::Time::now();
                    tf_valid_ = true;

                    ROS_INFO("TF transform received successfully. Initial yaw: %.3f rad", yaw_);
                    return true;
                }
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
            }

            try_rate.sleep();
        }

        return false;
    }

    void KeyboardControlNode::updateTransform()
    {
        try
        {
            geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
                "odom", "base_link", ros::Time(0), ros::Duration(TF_LOOKUP_TIMEOUT));

            // Validate transform freshness
            if ((ros::Time::now() - transform.header.stamp).toSec() < TF_VALID_MAX_AGE)
            {
                const auto &q = transform.transform.rotation;
                yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                last_transform_ = transform;
                last_tf_update_ = ros::Time::now();
                tf_valid_ = true;

                // Cache rotation components
                cached_cos_yaw_ = std::cos(yaw_);
                cached_sin_yaw_ = std::sin(yaw_);
                tf_updated_ = true;
            }
            else
            {
                ROS_WARN_THROTTLE(5.0, "Transform too old, ignoring");
                tf_updated_ = false;
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(5.0, "TF lookup failed: %s", ex.what());
            tf_valid_ = false;
            tf_updated_ = false;
        }
    }

    void KeyboardControlNode::processKeyboardInput()
    {
        static bool terminal_initialized = false;

        // Initialize terminal on first call
        if (!terminal_initialized)
        {
            struct termios original;
            if (tcgetattr(STDIN_FILENO, &original) == 0)
            {
                struct termios new_term = original;
                new_term.c_lflag &= ~(ICANON | ECHO);
                tcsetattr(STDIN_FILENO, TCSANOW, &new_term);
                fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
            }
            terminal_initialized = true;

            // Set quit handler
            std::signal(SIGINT, [](int)
                        { g_quit = true; });
            std::signal(SIGTERM, [](int)
                        { g_quit = true; });
        }

        // Poll for keyboard input
        struct pollfd pfd = {STDIN_FILENO, POLLIN | POLLPRI | POLLERR, 0};
        if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN))
        {
            char key = getchar();

            // Process key
            bool key_processed = true;

            // Movement keys
            switch (key)
            {
            case 'w':
            case 'W':
                target_world_vel_.linear.x = vel_scale_linear_;
                target_world_vel_.linear.y = 0.0;
                target_world_vel_.angular.z = 0.0;
                spinning_active_ = false;
                break;
            case 's':
            case 'S':
                target_world_vel_.linear.x = -vel_scale_linear_;
                target_world_vel_.linear.y = 0.0;
                target_world_vel_.angular.z = 0.0;
                spinning_active_ = false;
                break;
            case 'a':
            case 'A':
                target_world_vel_.linear.x = 0.0;
                target_world_vel_.linear.y = vel_scale_linear_;
                target_world_vel_.angular.z = 0.0;
                spinning_active_ = false;
                break;
            case 'd':
            case 'D':
                target_world_vel_.linear.x = 0.0;
                target_world_vel_.linear.y = -vel_scale_linear_;
                target_world_vel_.angular.z = 0.0;
                spinning_active_ = false;
                break;
            case 'q':
            case 'Q':
                target_world_vel_.linear.x = 0.0;
                target_world_vel_.linear.y = 0.0;
                target_world_vel_.angular.z = vel_scale_angular_;
                spinning_active_ = false;
                break;
            case 'e':
            case 'E':
                target_world_vel_.linear.x = 0.0;
                target_world_vel_.linear.y = 0.0;
                target_world_vel_.angular.z = -vel_scale_angular_;
                spinning_active_ = false;
                break;
            case 't':
            case 'T':
                spinning_top_mode_ = !spinning_top_mode_;
                ROS_INFO("Spinning top mode: %s", spinning_top_mode_ ? "ON" : "OFF");
                if (!spinning_top_mode_)
                {
                    spinning_active_ = false;
                    target_world_vel_.linear.x = 0.0;
                    target_world_vel_.linear.y = 0.0;
                    target_world_vel_.angular.z = 0.0;
                }
                break;
            case ' ':
                spinning_active_ = false;
                target_world_vel_.linear.x = 0.0;
                target_world_vel_.linear.y = 0.0;
                target_world_vel_.angular.z = 0.0;
                break;
            case 'h':
            case 'H':
                printHelp();
                break;
            case 27:
                // ESC
                key_processed = true;
                quit_requested_ = true;
                break;
            default:
                key_processed = false;
                break;
            }

            if (key_processed)
            {
                flushInputBuffer();
            }
        }
    }

    void KeyboardControlNode::updateAndPublish()
    {
        // Handle spinning top mode
        if (spinning_top_mode_ && tf_valid_)
        {
            handleSpinningTopMode();
        }

        // Convert world frame velocities to robot frame
        current_twist_ = convertToRobotFrame(target_world_vel_);

        // Check if we have motion
        bool has_motion = (std::abs(current_twist_.linear.x) > VELOCITY_EPSILON ||
                           std::abs(current_twist_.linear.y) > VELOCITY_EPSILON ||
                           std::abs(current_twist_.angular.z) > VELOCITY_EPSILON);

        // Publish commands
        if (has_motion || publish_zero_when_idle_)
        {
            publishCommands(has_motion);
        }
    }

    void KeyboardControlNode::handleSpinningTopMode()
    {
        if (spinning_top_mode_)
        {
            if (!spinning_active_)
            {
                target_world_vel_.linear.x = vel_scale_smart_linear_;
                target_world_vel_.linear.y = 0.0;
                target_world_vel_.angular.z = vel_scale_smart_angular_;
                spinning_active_ = true;
            }
        }
    }

    geometry_msgs::Twist KeyboardControlNode::convertToRobotFrame(const geometry_msgs::Twist &world_vel)
    {
        geometry_msgs::Twist robot_vel;

        if (field_centric_ && tf_valid_ && tf_updated_)
        {
            // 使用缓存的旋转分量以提高效率
            robot_vel.linear.x = world_vel.linear.x * cached_cos_yaw_ - world_vel.linear.y * cached_sin_yaw_;
            robot_vel.linear.y = world_vel.linear.x * cached_sin_yaw_ + world_vel.linear.y * cached_cos_yaw_;
        }
        else
        {
            // Direct pass-through if field-centric is disabled or TF is invalid
            robot_vel.linear.x = world_vel.linear.x;
            robot_vel.linear.y = world_vel.linear.y;
        }

        robot_vel.linear.z = world_vel.linear.z;
        robot_vel.angular.x = world_vel.angular.x;
        robot_vel.angular.y = world_vel.angular.y;
        robot_vel.angular.z = world_vel.angular.z;

        return robot_vel;
    }

    void KeyboardControlNode::publishCommands(bool has_motion)
    {
        // Publish Twist only if there are subscribers
        if (cmd_vel_pub_.getNumSubscribers() > 0 && (has_motion || publish_zero_when_idle_))
        {
            cmd_vel_pub_.publish(current_twist_);
        }

        // Publish TwistStamped only if there are subscribers
        if (cmd_vel_stamped_pub_.getNumSubscribers() > 0)
        {
            geometry_msgs::TwistStamped stamped;
            stamped.header.stamp = ros::Time::now();
            stamped.header.frame_id = "odom";
            stamped.twist = target_world_vel_; // Publish world-frame velocities

            cmd_vel_stamped_pub_.publish(stamped);
        }
    }

    void KeyboardControlNode::publishStopCommand()
    {
        current_twist_.linear.x = 0.0;
        current_twist_.linear.y = 0.0;
        current_twist_.angular.z = 0.0;

        cmd_vel_pub_.publish(current_twist_);

        if (cmd_vel_stamped_pub_.getNumSubscribers() > 0)
        {
            geometry_msgs::TwistStamped stamped;
            stamped.header.stamp = ros::Time::now();
            stamped.header.frame_id = "odom";
            stamped.twist.linear.x = 0.0;
            stamped.twist.linear.y = 0.0;
            stamped.twist.angular.z = 0.0;
            cmd_vel_stamped_pub_.publish(stamped);
        }
    }

    void KeyboardControlNode::flushInputBuffer()
    {
        std::string line;
        if (std::getline(std::cin, line))
        {
            std::cout.flush();
        }
    }

    void KeyboardControlNode::printHelp()
    {
        ROS_INFO("\n=== Keyboard Control Help ===");
        ROS_INFO("Movement Keys (in Field-Centric Mode):");
        ROS_INFO("  W - Forward");
        ROS_INFO("  S - Backward");
        ROS_INFO("  A - Left");
        ROS_INFO("  D - Right");
        ROS_INFO("  Q - Rotate Left");
        ROS_INFO("  E - Rotate Right");
        ROS_INFO("  T - Toggle Spinning Top Mode");
        ROS_INFO("  Space - Stop");
        ROS_INFO("Special:");
        ROS_INFO("  H - Show this help");
        ROS_INFO("  ESC/Q - Quit");
        ROS_INFO("Current Settings:");
        ROS_INFO("  Field Centric: %s", field_centric_ ? "ON" : "OFF");
        ROS_INFO("  Spinning Top Mode: %s", spinning_top_mode_ ? "ON" : "OFF");
        if (tf_valid_)
        {
            ROS_INFO("  TF Status: OK (yaw: %.3f rad)", yaw_);
        }
        else
        {
            ROS_INFO("  TF Status: INVALID");
        }
    }

} // namespace sentry_control

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sentry_control_key_optimized");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    try
    {
        sentry_control::KeyboardControlNode node(nh, private_nh);
        node.run();

        ROS_INFO("Keyboard control node shut down cleanly");
        return 0;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Keyboard control node crashed: %s", e.what());
        return 1;
    }
}