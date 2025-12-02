# IMU + 里程计 EKF 融合系统

## 概述

本模块实现了 **扩展卡尔曼滤波器 (EKF)** 融合 IMU 和轮式里程计，用于校正里程计漂移，提升定位精度。

### 系统架构

```
┌─────────────────┐     ┌─────────────────┐
│   Gazebo IMU    │     │  轮式里程计     │
│  /imu/data      │     │ /odom_controller│
│  (200Hz)        │     │  (50Hz)         │
└────────┬────────┘     └────────┬────────┘
         │                       │
         │    ┌──────────────────┘
         │    │
         v    v
┌────────────────────────────────────────┐
│           EKF 融合节点                 │
│   odom_ekf_fusion                      │
│                                        │
│  状态: [x, y, θ, vx, vy, ω]           │
│  预测: IMU 角速度 + 加速度             │
│  更新: 轮式里程计速度                  │
└────────────────────────────────────────┘
         │
         v
┌────────────────────────────────────────┐
│   融合输出                             │
│   /odom_fused (nav_msgs/Odometry)      │
│   TF: odom -> base_link                │
└────────────────────────────────────────┘
```

## 使用方法

### 启动 EKF 融合模式（推荐）

```bash
# 使用 EKF 融合 IMU + 轮式里程计
roslaunch sentry_chassis_controller sentry_ekf_fusion.launch odom_mode:=ekf
```

### 其他里程计模式

```bash
# 使用 Ground Truth（Gazebo 真实位姿，调试用）
roslaunch sentry_chassis_controller sentry_ekf_fusion.launch odom_mode:=ground_truth

# 使用纯轮式里程计（会有漂移）
roslaunch sentry_chassis_controller sentry_ekf_fusion.launch odom_mode:=wheel
```

### 旧版 launch 文件（向后兼容）

```bash
# 原有的 ground truth 模式
roslaunch sentry_chassis_controller sentry_pid_test_fixed.launch use_ground_truth_odom:=true
```

## 话题说明

| 话题 | 类型 | 说明 |
|------|------|------|
| `/imu/data` | `sensor_msgs/Imu` | Gazebo IMU 仿真数据（输入） |
| `/odom_controller` | `nav_msgs/Odometry` | 控制器轮式里程计（输入） |
| `/odom_fused` | `nav_msgs/Odometry` | EKF 融合里程计（输出） |
| `/odom` | `nav_msgs/Odometry` | Ground Truth 里程计（仅 ground_truth 模式） |

## EKF 算法原理

### 状态向量

$$\mathbf{x} = [x, y, \theta, v_x, v_y, \omega]^T$$

- $(x, y)$: 世界坐标系位置
- $\theta$: 航向角
- $(v_x, v_y)$: 世界坐标系速度
- $\omega$: 角速度

### 预测模型（使用 IMU）

1. **角速度更新**：直接使用 IMU 陀螺仪 $\omega_{imu}$
2. **加速度积分**：将 body 系加速度转换到世界系后积分
3. **位置积分**：二阶 Runge-Kutta 方法

$$
\begin{aligned}
\theta_{k+1} &= \theta_k + \omega_{imu} \cdot \Delta t \\
v_{k+1} &= v_k + R(\theta_k) \cdot a_{body} \cdot \Delta t \\
p_{k+1} &= p_k + \frac{v_k + v_{k+1}}{2} \cdot \Delta t
\end{aligned}
$$

### 更新模型（使用轮式里程计）

观测：body 系速度 $[v_{x,body}, v_{y,body}, \omega]$

将状态中的世界系速度转换到 body 系后与观测比较：

$$
\begin{bmatrix} v_{x,body} \\ v_{y,body} \end{bmatrix} = R(-\theta) \begin{bmatrix} v_x \\ v_y \end{bmatrix}
$$

## 参数配置

配置文件：`config/ekf_fusion.yaml`

```yaml
odom_ekf_fusion:
  # 坐标系
  odom_frame: "odom"
  base_link_frame: "base_link"
  
  # 发布频率
  publish_rate: 50.0
  
  # 过程噪声（越大 = 越不信任模型）
  process_noise_pos: 0.01
  process_noise_vel: 0.1
  process_noise_yaw: 0.01
  
  # 观测噪声（越大 = 越不信任轮式里程计）
  odom_noise_vel: 0.05
  odom_noise_omega: 0.05
```

### 噪声参数调节指南

| 场景 | 建议调整 |
|------|----------|
| 轮子打滑严重 | 增大 `odom_noise_vel` |
| IMU 噪声大 | 增大 `process_noise_*` |
| 转弯时漂移 | 减小 `process_noise_yaw` |
| 加减速时漂移 | 减小 `process_noise_vel` |

## 与 Ground Truth 模式对比

| 特性 | EKF 融合 | Ground Truth |
|------|----------|--------------|
| 真实性 | 接近真实传感器 | 完美（作弊） |
| 漂移 | 显著减少 | 无 |
| 适用场景 | 算法验证、调参 | 快速调试 |
| 实车迁移 | 架构可复用 | 不可用 |

## 文件结构

```
sentry_chassis_controller/
├── include/sentry_chassis_controller/
│   └── odom_ekf_fusion.hpp      # EKF 融合头文件
├── src/
│   └── odom_ekf_fusion.cpp      # EKF 融合实现
├── config/
│   └── ekf_fusion.yaml          # EKF 参数配置
├── launch/
│   └── sentry_ekf_fusion.launch # EKF 融合启动文件
└── docs/
    └── EKF里程计融合说明.md      # 本文档

rm_description_for_task/urdf/sentry/
└── imu.urdf.xacro               # IMU 仿真模块定义
```

## 调试建议

### 查看 IMU 数据

```bash
rostopic echo /imu/data
```

### 查看融合里程计

```bash
rostopic echo /odom_fused
```

### 对比三种里程计

```bash
# 在 RViz 中同时显示：
# 1. /odom_fused (EKF)
# 2. /odom_controller (轮式)
# 3. /odom (Ground Truth)
```

### 常见问题

1. **IMU 数据全零**：检查 Gazebo 插件是否正确加载
2. **EKF 不收敛**：检查 IMU 偏置初始化（需静止启动）
3. **方向偏移**：调整 `process_noise_yaw` 和 `odom_noise_omega`

## 参考文献

- [Probabilistic Robotics - EKF 章节](https://docs.ufpr.br/~danielsantos/ProbijisticRobotics.pdf)
- [ROS robot_localization 包](http://docs.ros.org/en/noetic/api/robot_localization/html/)
- 本项目文档：`里程计标定与矫正.md`
