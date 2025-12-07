# Power Limiter 模块说明

## 概述

`power_limiter` 模块提供底盘功率限制功能，通过二次方程求解最优缩放因子，确保总功率不超过设定上限。

## 功能原理

功率限制采用二次方程近似模型：

```
P ≈ a*s² + b*s + c ≤ P_limit
```

其中：
- `s` 为力矩缩放因子（scaling factor）
- `a = effort_coeff * Σ(cmd²)` - 力矩平方项系数
- `b = Σ|cmd * vel|` - 力矩-速度交叉项
- `c = velocity_coeff * Σ(vel²) - power_offset - power_limit` - 常数项

通过求解二次不等式 `a*s² + b*s + c ≤ 0`，得到最大允许缩放因子：

```
s = (-b + √(b² - 4ac)) / (2a)
```

当 `s < 1.0` 时，按比例缩放所有四个轮子的力矩命令。

## 使用示例

```cpp
#include "sentry_chassis_controller/power_limiter.hpp"

// 1. 配置功率限制参数
PowerLimiterConfig config;
config.power_limit = 80.0;        // 功率上限 80W
config.velocity_coeff = 0.1;      // 速度平方项系数
config.effort_coeff = 0.05;       // 力矩平方项系数
config.power_offset = 10.0;       // 功率偏置
config.enabled = true;            // 启用功率限制
config.debug_enabled = false;     // 关闭调试输出

// 2. 准备输入数据
PowerLimiterInput input;
input.cmd[0] = 5.0;   // 左前轮原始力矩命令
input.cmd[1] = 5.0;   // 右前轮
input.cmd[2] = 5.0;   // 左后轮
input.cmd[3] = 5.0;   // 右后轮
input.vel[0] = 10.0;  // 左前轮速度 (rad/s)
input.vel[1] = 10.0;  // 右前轮
input.vel[2] = 10.0;  // 左后轮
input.vel[3] = 10.0;  // 右后轮

// 3. 应用功率限制
PowerLimiterOutput output;
apply_power_limit(config, input, output);

// 4. 使用输出结果
if (output.limited) {
    ROS_INFO("Power limited! Scaling factor: %.3f", output.scaling_factor);
}
// 使用缩放后的力矩命令
motor_fl.setCommand(output.cmd[0]);
motor_fr.setCommand(output.cmd[1]);
motor_rl.setCommand(output.cmd[2]);
motor_rr.setCommand(output.cmd[3]);
```

## 数据结构

### PowerLimiterConfig
配置参数结构体：
- `power_limit`: 功率上限 (W)
- `velocity_coeff`: 速度平方项系数
- `effort_coeff`: 力矩平方项系数
- `power_offset`: 功率偏置
- `enabled`: 是否启用功率限制
- `debug_enabled`: 是否启用调试输出

### PowerLimiterInput
输入数据结构体：
- `cmd[4]`: 四个轮子的原始力矩命令
- `vel[4]`: 四个轮子的当前速度 (rad/s)

### PowerLimiterOutput
输出数据结构体：
- `cmd[4]`: 缩放后的力矩命令
- `scaling_factor`: 实际使用的缩放因子
- `limited`: 是否触发了限制
- `coeff_a`, `coeff_b`, `coeff_c`: 二次方程系数（调试用）
- `discriminant`: 判别式（调试用）

## 调试功能

启用 `debug_enabled` 并提供 ROS Publisher 可以发布调试信息：

```cpp
ros::Publisher debug_pub = nh.advertise<std_msgs::Float64MultiArray>("power_debug", 1);

PowerLimiterConfig config;
config.debug_enabled = true;
// ... 其他配置

PowerLimiterOutput output;
apply_power_limit(config, input, output, &debug_pub);
```

调试话题数据格式（Float64MultiArray）：
- `data[0]`: a (二次项系数)
- `data[1]`: b (一次项系数)
- `data[2]`: c (常数项)
- `data[3]`: discriminant (判别式)
- `data[4]`: scaling_factor (缩放因子)

## 优化建议

当前实现具有以下特点：
- ✅ 简单高效的二次方程求解
- ✅ 实时性好，计算量小
- ✅ 参数可调，易于配置

可改进方向：
- 增加滤波机制，平滑缩放因子变化
- 考虑更精确的功率模型
- 添加轮子优先级权重
- 支持动态功率上限调整

## 文件位置

- 头文件: `include/sentry_chassis_controller/power_limiter.hpp`
- 源文件: `src/power_limiter.cpp`
- 集成示例: `src/wheel_pid_controller.cpp` (update() 函数)
