# 键盘控制节点解耦重构文档

## 概述

本次重构将原来的单体键盘控制节点 `sentry_control_key_feature.cpp` 解耦为 4 个独立模块，提高了代码的复用性、可测试性和可维护性。

## 重构日期
2025年12月4日

## 模块架构

```
┌─────────────────────────────────────────────────────────────┐
│                      TeleopNode (ROS层)                      │
│  - ROS 初始化与参数管理                                        │
│  - 发布 Twist 消息                                            │
│  - 模块协调与主循环控制                                        │
└──────────────┬──────────────┬──────────────┬────────────────┘
               │              │              │
               v              v              v
    ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
    │ KeyboardInput│  │   Teleop     │  │  Velocity    │
    │              │  │ StateMachine │  │  Calculator  │
    │ - 终端设置    │  │              │  │              │
    │ - 按键读取    │  │ - 运动模式   │  │ - 速度归一化 │
    │ - poll封装   │  │ - 按键状态   │  │ - 速度限制   │
    │              │  │ - 互斥逻辑   │  │ - 坐标变换   │
    └──────────────┘  └──────────────┘  └──────────────┘
```

## 新增模块详解

### 1. KeyboardInput 模块
**文件**: `keyboard_input.hpp` / `keyboard_input.cpp`

**职责**:
- 封装终端设置（原始模式 vs 规范模式）
- 提供非阻塞按键读取 API
- 跨平台支持（Linux/Windows）

**主要接口**:
```cpp
class KeyboardInput {
public:
    bool poll_key(char& key, int timeout_ms);  // 轮询按键
    bool has_key(int timeout_ms);               // 检查是否有按键
    static char to_lower(char c);               // 转小写
};
```

**优势**:
- 将底层终端操作与业务逻辑完全分离
- 可复用于其他需要键盘输入的节点（云台控制、调试工具等）
- 易于测试（可模拟输入）

---

### 2. TeleopStateMachine 模块
**文件**: `teleop_state_machine.hpp` / `teleop_state_machine.cpp`

**职责**:
- 管理运动模式（NONE / TRANSLATION / ROTATION）
- 维护按键状态（WASD / QE）
- 处理按键互斥逻辑（平移与旋转互斥、前后左右互斥）
- 实现旋转锁存行为
- 处理平移超时

**主要接口**:
```cpp
struct TeleopConfig {
    double walk_vel;              // 步行速度
    double default_omega;         // 默认角速度
    double translation_timeout;   // 平移超时
    std::string velocity_mode;    // "global" 或 "chassis"
};

struct TeleopOutput {
    double vx, vy, omega;  // 速度输出
    bool has_motion;       // 是否有运动
};

class TeleopStateMachine {
public:
    bool process_key(char key, double current_time);
    void update(double current_time);
    const TeleopOutput& get_output() const;
};
```

**优势**:
- 清晰的 Input/Output 设计
- 状态机逻辑独立，易于调试和扩展
- 可独立单元测试（无需 ROS 环境）

---

### 3. VelocityCalculator 模块
**文件**: `velocity_calculator.hpp` / `velocity_calculator.cpp`

**职责**:
- 速度归一化（防止斜向速度过大）
- 速度限制（最大线速度、角速度）
- 预留坐标变换接口（当前由控制器处理）

**主要接口**:
```cpp
struct VelocityCalcConfig {
    std::string velocity_mode;
    double max_linear_vel;
    double max_angular_vel;
};

struct VelocityCalcInput {
    double vx, vy, omega;
    double yaw;  // 用于坐标变换（预留）
};

struct VelocityCalcOutput {
    double vx, vy, omega;
};

class VelocityCalculator {
public:
    VelocityCalcOutput compute(const VelocityCalcInput& input);
};
```

**优势**:
- 速度计算逻辑集中管理
- 易于添加新的速度处理策略（加速度限制、平滑等）
- 可独立测试速度计算正确性

---

### 4. TeleopNode 模块
**文件**: `teleop_node.hpp` / `teleop_node.cpp`

**职责**:
- ROS 初始化与参数管理
- 创建并协调上述 3 个模块
- 发布 Twist 消息
- 主循环控制

**主要接口**:
```cpp
class TeleopNode {
public:
    TeleopNode();   // 加载参数，创建模块
    void run();     // 主循环
    void print_usage();  // 打印使用说明
};
```

**优势**:
- 节点代码极度精简（~150 行 vs 原来 ~500 行）
- 职责单一：只负责 ROS 通信和模块协调
- 易于维护和理解

---

## 新旧节点对比

| 特性 | 原节点 (sentry_control_key_feature) | 新节点 (sentry_control_key_modular) |
|------|-------------------------------------|-------------------------------------|
| 代码行数 | ~500 行 | ~150 行 (节点) + ~400 行 (模块) |
| 模块化 | 单体 | 4 个独立模块 |
| 可测试性 | 难（需要 ROS + 终端） | 易（各模块可独立测试） |
| 可复用性 | 低 | 高（模块可用于其他项目） |
| 可维护性 | 中 | 高（职责分离清晰） |
| 功能 | 完整 | 完整（保持一致） |

---

## 使用方法

### 编译
```bash
cd /home/idris/final_ws
catkin build sentry_chassis_controller
```

### 运行新节点（模块化版本）
```bash
rosrun sentry_chassis_controller sentry_control_key_modular
```

### 运行旧节点（保持向后兼容）
```bash
rosrun sentry_chassis_controller sentry_control_key_feature
```

### 参数配置（与原节点一致）
```yaml
sentry_control_key_modular:
  cmd_vel_topic: "/cmd_vel"
  publish_zero_when_idle: false
  hz: 10
  walk_vel: 0.5
  default_omega: 1.0
  translation_timeout: 0.5
  velocity_mode: "global"  # 或 "chassis"
  max_linear_vel: 1.0
  max_angular_vel: 2.0
```

---

## 扩展性示例

### 1. 添加新的输入方式（游戏手柄）
只需：
- 创建 `GamepadInput` 类（类似 `KeyboardInput`）
- 在 `TeleopNode` 中切换输入源
- 状态机和速度计算器无需修改

### 2. 添加加速度限制
只需：
- 在 `VelocityCalculator` 中添加加速度限制逻辑
- 其他模块无需修改

### 3. 单元测试示例
```cpp
// 测试状态机
TEST(TeleopStateMachine, TranslationMode) {
    TeleopConfig config;
    config.walk_vel = 0.5;
    TeleopStateMachine sm(config);
    
    sm.process_key('w', 0.0);
    auto output = sm.get_output();
    
    EXPECT_EQ(sm.get_mode(), MotionMode::TRANSLATION);
    EXPECT_NEAR(output.vx, 0.5, 1e-6);
    EXPECT_NEAR(output.vy, 0.0, 1e-6);
    EXPECT_NEAR(output.omega, 0.0, 1e-6);
}
```

---

## 文件清单

### 新增文件
```
include/sentry_chassis_controller/
  ├── keyboard_input.hpp
  ├── teleop_state_machine.hpp
  ├── velocity_calculator.hpp
  └── teleop_node.hpp

src/
  ├── keyboard_input.cpp
  ├── teleop_state_machine.cpp
  ├── velocity_calculator.cpp
  ├── teleop_node.cpp
  └── sentry_control_key_modular.cpp  (主程序)
```

### 保留文件（向后兼容）
```
src/
  └── sentry_control_key_feature.cpp  (原节点)
```

---

## 编译验证

编译成功，生成可执行文件：
```
/home/idris/final_ws/devel/lib/sentry_chassis_controller/sentry_control_key_modular
大小: 727K
```

---

## 后续改进建议

1. **添加单元测试**
   - 为每个模块添加 GTest 单元测试
   - 覆盖边界情况（超时、按键冲突等）

2. **添加集成测试**
   - 使用 rostest 验证完整流程
   - 测试与控制器的集成

3. **性能优化**
   - 减少不必要的拷贝（使用引用）
   - 优化 poll 超时时间

4. **文档完善**
   - 添加 Doxygen 注释
   - 生成 API 文档

5. **游戏手柄支持**
   - 基于 `KeyboardInput` 的设计，添加 `JoystickInput` 模块
   - 实现平滑的模拟量控制

---

## 总结

本次重构成功将键盘控制节点解耦为 4 个独立模块，每个模块职责单一、接口清晰。这为未来的功能扩展（游戏手柄、自动驾驶模式切换等）奠定了良好的基础，同时保持了与原节点的功能一致性和向后兼容性。
