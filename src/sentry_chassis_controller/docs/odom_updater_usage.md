# OdomUpdater 模块说明

## 概述

`odom_updater` 模块封装了四舵轮底盘的前向运动学（Forward Kinematics）和里程计计算逻辑，使用**最小二乘法**求解超定方程，从四个轮子的反馈数据（轮速+舵角）精确计算底盘位姿。

## 核心算法

### 1. 前向运动学模型

对于四舵轮底盘，每个轮子 $i$ 沿舵向的线速度 $v_i$ 与底盘速度 $(v_x, v_y, \omega_z)$ 的关系为：

```
v_i = cos(θ_i)·v_x + sin(θ_i)·v_y + (-r_{y,i}·cos(θ_i) + r_{x,i}·sin(θ_i))·ω_z
```

其中：
- $θ_i$ 为第 $i$ 个轮子的舵角（弧度）
- $(r_{x,i}, r_{y,i})$ 为轮子相对底盘中心的位置
- $v_i = \omega_i \times r_{wheel}$ （轮子角速度 × 轮径）

### 2. 最小二乘求解

四个轮子形成超定方程组（4个方程，3个未知数）：

```
A·x = b
```

其中：
- $A$ 是 4×3 系数矩阵
- $x = [v_x; v_y; \omega_z]$ 是底盘速度（未知数）
- $b$ 是 4×1 观测向量（轮速反馈）

使用**法方程**求解：

```
x = (A^T·A)^(-1)·A^T·b
```

### 3. 位姿积分

通过欧拉积分更新世界坐标系位姿：

```
dx = v_x·cos(yaw) - v_y·sin(yaw)
dy = v_x·sin(yaw) + v_y·cos(yaw)
x += dx·dt
y += dy·dt
yaw += ω_z·dt
```

## 数据结构

### OdomUpdaterInput
输入数据结构：
```cpp
struct OdomUpdaterInput {
    double wheel_velocities[4];    // 轮子角速度 (rad/s) [FL, FR, RL, RR]
    double pivot_angles[4];        // 舵角位置 (rad) [FL, FR, RL, RR]
    double wheel_base;             // 轴距 (m)
    double wheel_track;            // 轮距 (m)
    double wheel_radius;           // 轮径 (m)
    std::string odom_frame;        // 里程计坐标系
    std::string base_link_frame;   // 底盘坐标系
    bool publish_tf;               // 是否发布 TF
    double velocity_deadband;      // 速度死区 (m/s)
};
```

### OdomUpdaterOutput
输出数据结构：
```cpp
struct OdomUpdaterOutput {
    double vx, vy, wz;             // 底盘速度 (机体坐标系)
    bool solution_valid;           // 最小二乘解是否有效
    double matrix_determinant;     // ATA 矩阵行列式（调试用）
    double odom_x, odom_y, odom_yaw; // 累积位姿（世界坐标系）
};
```

### OdomUpdater 类
```cpp
class OdomUpdater {
public:
    OdomUpdater();
    ~OdomUpdater();
    
    // 更新里程计
    void update(const OdomUpdaterInput& input,
                const ros::Time& time,
                const ros::Duration& period,
                ros::Publisher& odom_pub,
                OdomUpdaterOutput& output);
    
    // 重置位姿
    void reset_pose(double x = 0.0, double y = 0.0, double yaw = 0.0);
    
private:
    double odom_x_, odom_y_, odom_yaw_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    bool solve_least_squares(const double A[4][3], 
                            const double b[4], 
                            double x[3], 
                            double& det);
};
```

## 使用示例

```cpp
#include "sentry_chassis_controller/odom_updater.hpp"

// 1. 创建里程计更新器
OdomUpdater odom_updater;

// 2. 在控制循环中调用
void update_callback(const ros::Time& time, const ros::Duration& period) {
    // 准备输入数据
    OdomUpdaterInput input;
    input.wheel_velocities[0] = front_left_wheel_joint.getVelocity();
    input.wheel_velocities[1] = front_right_wheel_joint.getVelocity();
    input.wheel_velocities[2] = back_left_wheel_joint.getVelocity();
    input.wheel_velocities[3] = back_right_wheel_joint.getVelocity();
    
    input.pivot_angles[0] = front_left_pivot_joint.getPosition();
    input.pivot_angles[1] = front_right_pivot_joint.getPosition();
    input.pivot_angles[2] = back_left_pivot_joint.getPosition();
    input.pivot_angles[3] = back_right_pivot_joint.getPosition();
    
    input.wheel_base = 0.36;
    input.wheel_track = 0.36;
    input.wheel_radius = 0.05;
    input.odom_frame = "odom";
    input.base_link_frame = "base_link";
    input.publish_tf = true;
    input.velocity_deadband = 0.05;
    
    // 调用更新器
    OdomUpdaterOutput output;
    odom_updater.update(input, time, period, odom_pub, output);
    
    // 使用输出结果
    if (output.solution_valid) {
        ROS_INFO("Chassis velocity: vx=%.2f, vy=%.2f, wz=%.2f", 
                 output.vx, output.vy, output.wz);
        ROS_INFO("Pose: x=%.2f, y=%.2f, yaw=%.2f", 
                 output.odom_x, output.odom_y, output.odom_yaw);
    }
}
```

## 核心特性

### 1. 最小二乘求解
- ✅ 处理超定方程（4轮→3速度）
- ✅ 鲁棒性好，可应对单轮传感器故障
- ✅ 平滑融合所有轮子的反馈

### 2. 奇异性检测
- 自动检测矩阵奇异性（行列式 < 1e-9）
- 奇异情况下使用零速度作为安全回退
- 输出 `solution_valid` 标志供上层判断

### 3. 速度死区滤波
- 消除静止时的传感器噪声导致的里程计漂移
- 可配置死区阈值（默认 0.05 m/s）
- 对 vx, vy, wz 分别独立滤波

### 4. 坐标变换
- 机体坐标系 (base_link) → 世界坐标系 (odom)
- 二维旋转矩阵变换
- 欧拉积分更新位姿

### 5. ROS 集成
- 发布 `nav_msgs::Odometry` 消息
- 可选发布 TF 变换 (odom → base_link)
- 与导航栈完全兼容

## 对比 forward_kinematics

| 特性 | forward_kinematics.cpp (旧) | odom_updater.cpp (新) |
|------|---------------------------|---------------------|
| 算法 | 简化平均法 | 最小二乘法 |
| 精度 | 低（近似） | 高（精确求解） |
| 鲁棒性 | 弱 | 强（超定方程） |
| 封装 | 函数式 | 面向对象（类） |
| 状态管理 | 文件静态变量 | 类成员变量 |
| 位姿重置 | 不支持 | 支持 `reset_pose()` |
| 调试信息 | 无 | 行列式、求解状态 |

## 数学推导

### 轮子运动学方程

对于位于 $(r_x, r_y)$ 的轮子，其沿舵向 $θ$ 的速度为：

```
v = v_chassis·n + ω_z × r
```

其中 $n = [\cos θ, \sin θ]^T$ 是舵向单位向量，展开得：

```
v = cos(θ)·v_x + sin(θ)·v_y + ω_z·(-r_y·cos(θ) + r_x·sin(θ))
```

### 四轮方程组

```
┌           ┐   ┌    ┐   ┌    ┐
│ cos θ₁ sin θ₁ -r_y₁cosθ₁+r_x₁sinθ₁ │   │ v_x  │   │ v₁ │
│ cos θ₂ sin θ₂ -r_y₂cosθ₂+r_x₂sinθ₂ │ · │ v_y  │ = │ v₂ │
│ cos θ₃ sin θ₃ -r_y₃cosθ₃+r_x₃sinθ₃ │   │ ω_z  │   │ v₃ │
│ cos θ₄ sin θ₄ -r_y₄cosθ₄+r_x₄sinθ₄ │   └    ┘   │ v₄ │
└           ┘                               └    ┘
     A (4×3)                   x (3×1)      b (4×1)
```

### 法方程求解

```
A^T·A·x = A^T·b
x = (A^T·A)^(-1)·A^T·b
```

其中 $A^T·A$ 是 3×3 对称正定矩阵（正常情况），使用**伴随矩阵法**计算逆矩阵：

```
(A^T·A)^(-1) = adj(A^T·A) / det(A^T·A)
```

## 常见问题

### Q1: 什么情况下矩阵会奇异？
**A:** 当四个轮子的舵向共线或近似共线时，方程组秩 < 3，无法唯一确定底盘速度。常见原因：
- 初始化阶段，舵角未就位
- 四轮舵角全部指向同一方向
- 传感器故障导致舵角反馈错误

### Q2: 如何处理奇异情况？
**A:** 模块自动检测 `det(A^T·A) < 1e-9`，输出 `solution_valid = false`，并使用零速度作为安全回退。上层可根据此标志做降级处理。

### Q3: 速度死区如何设置？
**A:** 根据传感器噪声水平和期望精度调整：
- 噪声大 → 增大死区（0.05~0.1 m/s）
- 低速精度要求高 → 减小死区（0.01~0.03 m/s）
- 建议通过实验调优

### Q4: 里程计漂移如何处理？
**A:** 模块已包含速度死区滤波，但长时间运行仍会积累误差。建议：
- 融合其他传感器（IMU、视觉里程计）
- 使用 SLAM/定位算法校正
- 定期重置位姿（如到达已知位置）

## 性能优化

### 计算复杂度
- 矩阵乘法：O(48) 次乘法（4×3×3 + 4×3）
- 逆矩阵计算：O(45) 次乘法（3×3 伴随矩阵）
- 总计：O(100) 次浮点运算，实时性优秀

### 内存占用
- 静态大小：约 200 字节（类成员）
- 栈内存：约 300 字节（局部变量）
- 无动态分配，适合嵌入式环境

## 文件位置

- 头文件: `include/sentry_chassis_controller/odom_updater.hpp`
- 源文件: `src/odom_updater.cpp`
- 集成示例: `src/wheel_pid_controller.cpp` (update() 函数)

## 版本历史

### v2.0 (当前)
- ✅ 完整的最小二乘求解
- ✅ 面向对象封装
- ✅ 奇异性检测与降级
- ✅ 速度死区滤波
- ✅ 位姿重置接口

### v1.0 (已废弃: forward_kinematics.cpp)
- 简化的速度平均法
- 函数式接口
- 无奇异性处理
