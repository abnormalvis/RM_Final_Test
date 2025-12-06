#include "sentry_chassis_controller/teleop_node.hpp"
#include <signal.h>
#include <cstdio>

namespace sentry_chassis_controller
{
    // 全局节点指针（用于信号处理）
    static TeleopNode *g_node_ptr = nullptr;
    // 信号处理函数
    static void signal_handler(int sig)
    {
        (void)sig;
        ros::shutdown();
        exit(0);
    }

    TeleopNode::TeleopNode() : nh_private_("~"), publish_zero_when_idle_(false), publish_rate_hz_(10)
    {
        // 加载参数
        load_parameters();
        // 发布速度指令
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
        // 参数配置
        TeleopConfig teleop_config;
        nh_private_.param("walk_vel", teleop_config.walk_vel, 0.5);
        nh_private_.param("default_omega", teleop_config.default_omega, 1.0);
        nh_private_.param("velocity_mode", teleop_config.velocity_mode, std::string("global"));
        VelocityCalcConfig vel_calc_config;
        vel_calc_config.velocity_mode = teleop_config.velocity_mode;
        nh_private_.param("max_linear_vel", vel_calc_config.max_linear_vel, 1.0);
        nh_private_.param("max_angular_vel", vel_calc_config.max_angular_vel, 2.0);

        // 创建模块实例
        keyboard_input_ = std::make_unique<KeyboardInput>();
        state_machine_ = std::make_unique<TeleopStateMachine>(teleop_config);
        velocity_calc_ = std::make_unique<VelocityCalculator>(vel_calc_config);

        // 设置用于信号处理的全局指针
        g_node_ptr = this;
        signal(SIGINT, signal_handler);
    }

    TeleopNode::~TeleopNode()
    {
        g_node_ptr = nullptr;
    }

    void TeleopNode::load_parameters()
    {
        nh_private_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
        nh_private_.param("publish_zero_when_idle", publish_zero_when_idle_, false);
        nh_private_.param("hz", publish_rate_hz_, 10);
    }

    void TeleopNode::print_usage()
    {
        puts("========================================");
        puts("  Sentry Chassis Keyboard Teleop");
        puts("========================================");
        puts("  WASD - Translation");
        puts("    W: Forward");
        puts("    S: Backward");
        puts("    A: Left");
        puts("    D: Right");
        puts("  Q/E  - Rotation (latched)");
        puts("    Q: Rotate Left");
        puts("    E: Rotate Right");
        puts("  C    - Stop all motion");
        puts("  U/I  - Increase/Decrease linear velocity (±0.1 m/s)");
        puts("  O/P  - Increase/Decrease angular velocity (±0.1 rad/s)");
        puts("  Ctrl-C - Quit");
        puts("========================================");

        std::string mode = state_machine_->get_config().velocity_mode;
        printf("  Velocity Mode: %s\n", mode.c_str());
        printf("  Walk Vel: %.2f m/s\n", state_machine_->get_config().walk_vel);
        printf("  Default Omega: %.2f rad/s\n", state_machine_->get_config().default_omega);
        puts("========================================");
    }

    void TeleopNode::publish_velocity(const VelocityCalcOutput &output, bool force_publish)
    {
        bool has_motion = (std::abs(output.vx) > 1e-9) ||
                          (std::abs(output.vy) > 1e-9) ||
                          (std::abs(output.omega) > 1e-9);

        if (force_publish || has_motion || publish_zero_when_idle_)
        {
            geometry_msgs::Twist msg;
            msg.linear.x = output.vx;
            msg.linear.y = output.vy;
            msg.linear.z = 0.0;
            msg.angular.x = 0.0;
            msg.angular.y = 0.0;
            msg.angular.z = output.omega;

            twist_pub_.publish(msg);
        }
    }

    void TeleopNode::run()
    {
        print_usage();  // 打印使用说明
        ros::Rate rate(publish_rate_hz_);   // 有参构造rate对象
        int poll_timeout_ms = 1000 / publish_rate_hz_;  // 计算轮询超时时间

        while (ros::ok())
        {
            char key = 0;   // 按键
            bool has_key = keyboard_input_->poll_key(key, poll_timeout_ms);
            // 获取当前时间（秒）
            double current_time = ros::Time::now().toSec(); // 获取当前时间
            
            if (has_key)
            {
                char lower_key = KeyboardInput::to_lower(key);  // 转换为小写
                if (lower_key == '\x03')    // 处理 Ctrl-C
                {
                    break;
                }
                
                // 处理按键
                bool processed = state_machine_->process_key(lower_key, current_time);
                
                // 如果是速度调节键(u/i/o/p)，输出当前速度
                if (processed && (lower_key == 'u' || lower_key == 'i' || lower_key == 'o' || lower_key == 'p'))
                {
                    if (lower_key == 'u' || lower_key == 'i')
                    {
                        ROS_INFO("Walk velocity %s to %.2f m/s", 
                                 (lower_key == 'u') ? "increased" : "decreased",
                                 state_machine_->get_config().walk_vel);
                    }
                    else // o or p
                    {
                        ROS_INFO("Angular velocity %s to %.2f rad/s",
                                 (lower_key == 'o') ? "increased" : "decreased", 
                                 state_machine_->get_config().default_omega);
                    }
                }
                
                // 日志输出（非速度调节键）
                if (lower_key != 'u' && lower_key != 'i' && lower_key != 'o' && lower_key != 'p')
                {
                    const TeleopOutput &state_output = state_machine_->get_output();
                    ROS_INFO_THROTTLE(1.0, "Teleop: mode=%s vel=(%.2f,%.2f) omega=%.2f",
                                      state_machine_->get_config().velocity_mode.c_str(),
                                      state_output.vx, state_output.vy, state_output.omega);
                }
            }
            
            // 计算最终速度（无论是否有按键都计算并发布）
            const TeleopOutput &state_output = state_machine_->get_output();
            VelocityCalcInput vel_input;
            vel_input.vx = state_output.vx;
            vel_input.vy = state_output.vy;
            vel_input.omega = state_output.omega;
            vel_input.yaw = 0.0; // 暂时不使用（控制器会处理坐标变换）

            VelocityCalcOutput vel_output = velocity_calc_->compute(vel_input);

            // 始终发布速度（包括零速度）
            publish_velocity(vel_output, true);

            rate.sleep();
        }

        ROS_INFO("Teleop node shutting down...");
    }

} // namespace sentry_chassis_controller
