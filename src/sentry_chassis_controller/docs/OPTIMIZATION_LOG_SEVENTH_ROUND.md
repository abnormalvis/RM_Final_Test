# 第七轮优化记录 - PID输出限幅与安全误差重置

**优化日期**: 2025-01-29  
**当前commit**: develop-selflock分支  
**回退目标**: 4e0b6bc (第六轮优化前的稳定版本)

---

## 问题描述

在第六轮优化（添加软件限位和速度限制）后，发现软件限位虽然生效（日志中出现"Pivot X clamped to ±80°"警告），但**舵角仍然失控到-720°和-360°**，导致Gazebo仿真崩溃。

### 关键发现（从日志分析）

**时间线分析**（参考 `/home/idris/final_ws/src/sentry_chassis_controller/log/sentry_with_odom_launch.log`）：

1. **t=88-92s**: 前进运动基本正常
   - 轮速: 4-6 rad/s (合理范围)
   - 舵角: 18.2°, -51.4°, -32.9°, 60.0° (较大但在范围内)
   
2. **t=92s**: 开始出现警告
   ```
   [WARN] Pivot 2 error -0.70 rad too large, resetting cmd
   [WARN] Pivot 1 clamped to -80°
   [WARN] Pivot 3 clamped to +80°
   ```

3. **t=93-96s**: 持续警告
   - Pivot 1 (FR) 和 Pivot 3 (RR) 反复被限制在±80°
   - 但实际舵角仍然继续增长: -61.3°, -76.8°
   
4. **最终崩溃特征**:
   - 舵角失控达到±90°极限后继续累积
   - 日志中出现yaw=2207.51°（异常大的角度累积）

---

## 根本原因分析

### 1. 软件限位失效原因

**问题1**: 软件限位只能限制`pivot_cmd_[i]`（命令值），但**无法阻止Gazebo中实际关节位置失控**。

**问题2**: **PID输出没有限幅**！
- `wheel_pid_params.yaml`中设置的`output_min/max=±10.0`**没有被使用**
- 代码中`pid.initPid()`只接受`i_clamp`参数，没有设置output限幅
- PID控制器在误差很大时（例如期望80°但实际-50°，误差130°），会产生**巨大的输出力矩**
- 巨大的力矩直接施加到Gazebo关节上，导致物理仿真数值爆炸

### 2. 失控反馈循环

```
1. IK要求舵角旋转到80° (软件限位)
2. 实际舵角-50° (Gazebo物理不稳定)
3. 误差=130° → PID输出可能>100 Nm (无限幅)
4. 巨大力矩施加到Gazebo → 舵角瞬间跳变到-720°
5. FK计算时使用-720°的异常值 → 里程计爆炸
6. 系统崩溃
```

---

## 第七轮优化方案

### 修改1: PID输出力矩限幅

**文件**: `src/sentry_chassis_controller/src/wheel_pid_controller.cpp`

**位置**: `update()` 函数，PID计算后

**代码**:
```cpp
// Clamp PID output torque to prevent Gazebo instability
const double MAX_PIVOT_TORQUE = 5.0;  // Nm, conservative limit
p0 = std::max(-MAX_PIVOT_TORQUE, std::min(MAX_PIVOT_TORQUE, p0));
p1 = std::max(-MAX_PIVOT_TORQUE, std::min(MAX_PIVOT_TORQUE, p1));
p2 = std::max(-MAX_PIVOT_TORQUE, std::min(MAX_PIVOT_TORQUE, p2));
p3 = std::max(-MAX_PIVOT_TORQUE, std::min(MAX_PIVOT_TORQUE, p3));
```

**原理**:
- 限制舵角PID输出的最大力矩为±5.0 Nm
- 防止误差过大时产生巨大力矩导致Gazebo崩溃
- 5.0 Nm是保守值，应该足够驱动小型舵轮

### 修改2: 安全误差重置机制

**文件**: `src/sentry_chassis_controller/src/wheel_pid_controller.cpp`

**位置**: `update()` 函数，PID计算前

**代码**:
```cpp
// Safety: if position error too large, reset cmd to current position to prevent runaway
const double SAFETY_ANGLE_ERROR = 0.7;  // ~40 degrees, half of limit
double curr_pos[4] = {
    front_left_pivot_joint_.getPosition(),
    front_right_pivot_joint_.getPosition(),
    back_left_pivot_joint_.getPosition(),
    back_right_pivot_joint_.getPosition()
};

for (int i = 0; i < 4; i++) {
    double error = pivot_cmd_[i] - curr_pos[i];
    if (std::abs(error) > SAFETY_ANGLE_ERROR) {
        ROS_WARN_THROTTLE(1.0, "Pivot %d error %.2f rad too large, resetting cmd", i, error);
        pivot_cmd_[i] = curr_pos[i];  // Reset to current position
    }
}
```

**原理**:
- 当舵角位置误差超过40°（0.7 rad）时，认为控制已失控
- 直接将期望值重置为当前位置，而不是尝试纠正巨大误差
- 让系统逐步稳定下来，避免PID持续施加大力矩

---

## 参数总结

### Gazebo物理参数（第六轮已优化）
```yaml
舵角关节:
  damping: 1.5
  friction: 0.4
  velocity: 15 rad/s
  lower/upper: ±1.57 rad (±90°，但未生效)

轮子关节:
  damping: 0.0
  friction: 0.05
  mu1/mu2: 1.2
  kp: 100000.0
  kd: 10.0
```

### PID参数（第六轮已优化）
```yaml
舵角PID:
  p: 2.0
  i: 0.01
  d: 3.0
  i_clamp: 0.2
  output: ±10.0 (YAML中设置但代码未使用)

实际第七轮添加:
  output_torque_limit: ±5.0 Nm (代码中硬编码)
  safety_angle_error: 0.7 rad (~40°)
```

### 软件限位（第六轮已添加）
```cpp
cmdVelCallback中:
  MAX_PIVOT_ANGLE: 1.396 rad (±80°)
  MAX_PIVOT_VELOCITY: 5.0 rad/s
```

---

## 测试结果与问题

### 测试过程
1. 重新编译: `catkin build sentry_chassis_controller`
2. 启动仿真: `roslaunch sentry_chassis_controller sentry_with_odom_feature.launch`
3. 键盘控制: 按W前进，观察运动状态

### 观察到的问题

**日志特征** (`sentry_with_odom_launch.log`):
```
t=88-92s: 基本正常前进
  - 轮速: 4-6 rad/s
  - 舵角: ±60° 范围内
  
t=92s: 开始出现问题
  - "Pivot 2 error -0.70 rad too large, resetting cmd"
  - "Pivot 1 clamped to -80°"
  - "Pivot 3 clamped to +80°"
  
t=93-96s: 持续警告
  - FR和RR舵角反复被限制
  - 但实际舵角仍在增长: -61.3° → -76.8°
  - yaw累积异常: 1826° → 2207°
```

**问题总结**:
1. ✅ **软件限位生效**: 日志中出现"clamped to ±80°"警告
2. ✅ **安全误差重置生效**: 日志中出现"error too large, resetting cmd"警告
3. ❌ **但舵角仍然失控**: 实际舵角继续增长，最终达到±90°极限
4. ❌ **里程计异常**: yaw累积到2207°（应该在±180°范围内）

### 可能的原因

1. **Gazebo物理仿真根本不稳定**
   - 即使PID输出限制在5.0 Nm，仍可能导致仿真不稳定
   - 可能需要进一步降低力矩限制到3.0 Nm甚至更低

2. **安全误差阈值设置过大**
   - 0.7 rad (40°) 可能太大，应该降低到30°（0.52 rad）

3. **PID参数仍然不够保守**
   - P=2.0可能仍然太高，需要继续降低

4. **Gazebo硬件限位完全失效**
   - URDF中的`lower/upper=±1.57`在revolute joint上不可靠
   - 可能需要使用不同的关节类型或Gazebo版本特定的配置

---

## 参考实现对比

### AgileX Scout Mini Omni
- **架构**: 底层固件实现FK，ROS层只做速度积分
- **里程计**: 直接从CAN读取body速度，不需要FK
- **不适用**: 我们的舵轮控制器需要自己实现FK

### sentry_sim (planar_move)
- **架构**: 理想全向移动插件，直接设置模型速度
- **no轮子控制**: Fixed joint，没有真实动力学
- **不适用**: 完全不同的仿真架构，无参考价值

**结论**: 我们的实现方向正确（FK在控制器中），但需要更激进的参数优化。

---

## 回退决策

### 为什么回退？

1. **运动状态不理想**: 虽然添加了两层保护（力矩限幅+误差重置），但舵角仍然失控
2. **调试难度大**: 问题涉及Gazebo物理引擎、PID参数、FK实现多个层面
3. **需要重新规划**: 可能需要：
   - 更激进的参数降低（P=1.0, 力矩限制=3.0 Nm）
   - 简化底盘模型（去掉部分传感器/碰撞体）
   - 使用不同的Gazebo插件或控制策略

### 回退目标

**Commit**: `4e0b6bc`
- 这是第六轮优化前的版本
- 应该是相对稳定的基线版本
- 可以从这里重新开始更保守的优化策略

---

## 下一步建议

1. **回退到4e0b6bc**
2. **记录当前第七轮修改为参考**
3. **重新评估优化策略**:
   - 先验证基线版本的运动状态
   - 如果基线也有问题，考虑更根本的架构调整
   - 如果基线正常，逐步添加保护机制（每次只改一个参数）

4. **可选的替代方案**:
   - 使用position_controllers替代effort_controllers
   - 简化为麦克纳姆轮（去掉舵角，只用轮速）
   - 参考其他开源舵轮底盘实现（如NASA的Open Source Rover）

---

## 附录：完整改动diff

详见附件或使用 `git diff 4e0b6bc HEAD` 查看。

主要改动文件:
- `src/sentry_chassis_controller/src/wheel_pid_controller.cpp`
  - 第401-433行: 添加安全误差重置和PID输出限幅
  
配置文件未改动（第六轮已优化）。

---

**记录人**: AI Assistant  
**记录时间**: 2025-01-29  
**状态**: 准备回退到4e0b6bc
