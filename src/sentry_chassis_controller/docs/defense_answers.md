# 哨兵底盘控制器答辩参考答案

## 一、ros_control 框架相关

### 1. 什么是 ros_control？它在机器人控制中扮演什么角色？

ros_control 是 ROS 中的一个控制框架，提供了硬件抽象层和控制器接口的标准化实现。它的核心作用是：

1. **硬件抽象**：将底层硬件（电机、传感器）抽象为统一的接口
2. **控制器管理**：提供控制器的加载、启动、停止等生命周期管理
3. **实时控制**：支持实时控制循环，保证控制的时效性
4. **可复用性**：控制器可在仿真和实际硬件间无缝切换

### 2. 解释 Controller Manager 的作用和工作原理

Controller Manager 是 ros_control 的核心组件：

```
+------------------+
+--------+---------+
         |
    +----+----+----+----+
         |
+--------+---------+
| Hardware         |
| Interface        |
+------------------+
```

**主要功能**：
- 加载/卸载控制器
- 启动/停止控制器
- 管理控制器之间的资源冲突
- 周期性调用控制器的 update() 方法

### 3. Hardware Interface 有哪些类型？你的项目用了哪种？

常见类型：
- `JointStateInterface`：只读关节状态
- `PositionJointInterface`：位置控制
- `VelocityJointInterface`：速度控制
- `EffortJointInterface`：力矩/力控制

**本项目使用 `EffortJointInterface`**，原因：
1. 舵轮需要精确的力矩控制
2. 可以实现 PID 控制器输出直接映射到电机力矩
3. 更接近实际电机的控制方式

### 4. 控制器的生命周期包括哪些阶段？
bool init(hardware_interface::EffortJointInterface* hw,
          ros::NodeHandle& root_nh,
          ros::NodeHandle& controller_nh);  // 初始化

void starting(const ros::Time& time);       // 启动时调用一次

void update(const ros::Time& time,
            const ros::Duration& period);   // 每个控制周期调用

void stopping(const ros::Time& time);       // 停止时调用
```

### 5. 如何实现一个自定义的 ros_control 控制器？

```cpp
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

class MyController : public controller_interface::Controller<
    hardware_interface::EffortJointInterface>
{
public:
              ros::NodeHandle& controller_nh) override
    {
        // 获取关节句柄、读取参数等
    }

    void update(const ros::Time& time,
        // 实现控制逻辑
        double error = setpoint_ - joint_.getPosition();
        joint_.setCommand(kp_ * error);
    double setpoint_, kp_;
};

    controller_interface::ControllerBase)
```

---

### 6. ros_control 与直接发布话题控制有什么区别？优势在哪里？

- 直接发布话题控制通常是“开环”或弱耦合，控制周期和资源管理由各节点自行处理；
- ros_control提供统一的实时控制循环、硬件抽象、控制器生命周期和资源互斥，便于在仿真与实机间切换；
- 优势：实时性更好、接口标准化、可复用、与Gazebo紧密集成、易于参数化与动态重配置。

### 7. JointStateInterface 和 EffortJointInterface 的区别？

- JointStateInterface：只读接口，提供 position/velocity/effort 的状态读取，不支持写命令；
- EffortJointInterface：可写接口，允许通过 setCommand(τ) 写入力矩命令，适合基于力矩的控制器。

### 8. 如何在 Gazebo 中使用 ros_control？需要哪些插件？

- 在URDF/SDF中加载 gazebo_ros_control 插件，使Gazebo将物理仿真与ros_control桥接；
- 为各关节添加 transmission/actuator 配置；
- 控制器通过 controller_manager 在仿真中被加载和更新。

### 9. 控制器的更新频率是如何确定的？

- 由 Controller Manager 的更新循环（通常与ros_control的线程或Gazebo仿真步长）驱动；
- 可在控制器参数或launch中设置期望频率；
- 实机中建议绑定到稳定的高优先级实时线程（如 100–1000 Hz），仿真中受仿真步长与CPU约束。

### 10. 如何处理多个控制器之间的资源冲突？

- ros_control通过资源管理避免同一关节被多个控制器同时写命令；
- 可通过 switchController 启停控制器来切换资源占用；
- 若需要协同控制，采用组合控制器或在同一控制器内协调输出。

## 二、舵轮底盘运动学

### 11. 解释舵轮底盘（Swerve Drive）的基本原理

舵轮底盘每个轮子有两个自由度：
1. **舵向**：控制轮子朝向（0-360°）
2. **轮速**：控制轮子转速

这使得机器人可以实现全向移动，同时保持任意朝向。

**优点**：
- 真正的全向移动能力
- 高负载能力（轮子始终垂直于地面）

### 12. 与麦克纳姆轮、差速底盘相比，舵轮有什么优缺点？

- 优点：真正全向、抓地力强、效率高、姿态可控；
- 缺点：机构复杂、成本高、控制与标定更难；
- 麦轮：易实现全向但效率低、滑移多；差速：结构简单、控制易但不全向。
- 精确的轨迹跟踪

### 13. 推导从机器人速度 (vx, vy, ω) 到各轮期望状态的逆运动学公式

设第 i 个轮子相对于机器人中心的位置为 $(r_{xi}, r_{yi})$，机器人速度为 $(v_x, v_y, \omega)$。

**轮子的线速度向量**：
$$\vec{v}_i = \begin{pmatrix} v_x \\ v_y \end{pmatrix} + \omega \begin{pmatrix} -r_{yi} \\ r_{xi} \end{pmatrix}$$

展开得：
$$v_{xi} = v_x - \omega \cdot r_{yi}$$
$$v_{yi} = v_y + \omega \cdot r_{xi}$$

**轮子速度大小和方向**：
$$|\vec{v}_i| = \sqrt{v_{xi}^2 + v_{yi}^2}$$
$$\theta_i = \text{atan2}(v_{yi}, v_{xi})$$

**代码实现**：
```cpp
// 对于左前轮 (rx, ry)
double vx_wheel = vx - omega * ry;
double vy_wheel = vy + omega * rx;
double wheel_speed = std::sqrt(vx_wheel*vx_wheel + vy_wheel*vy_wheel);
double wheel_angle = std::atan2(vy_wheel, vx_wheel);
```

### 15. 如何处理舵轮的 180° 翻转问题？

当期望角度与当前角度相差接近 180° 时，可以选择：
1. 反转轮子方向
2. 反转轮速
 
 更完整的处理流程是：
 - 计算角度差 delta = normalize(desired - current)（归一化到 (-pi, pi]），
 - 如果 |delta| > 90° (pi/2)，则把期望角加/减 180°（即 desired -= copysign(pi, delta)），同时把期望线速度取反（speed = -speed）。
 - 最后再把期望角归一化到 (-pi, pi]，送入角度跟踪控制器。

 这样做的目的是让舵轮旋转的角度最小化（避免绕大圈），并通过翻转速度保持车体期望运动不变。

 示例（数值演示，角度以度/弧度说明）：
 - 当前角度 current = 10° = 0.17453 rad
 - 期望角度 desired = 190° = 3.31613 rad
 - delta = normalize(desired - current) = normalize(3.31613 - 0.17453) = normalize(3.1416) = pi (≈ 3.1416)
 - 因为 |delta| = 180° > 90°，选择翻转：
     - desired' = normalize(desired - sign(delta)*pi) = normalize(3.31613 - 3.1416) = 0.17453 rad = 10°
     - speed' = -speed

 结果说明：原本要把舵轮从 10° 转到 190°（需要转 180°），通过翻转得到新的期望角 10°（与当前角相同），同时把轮速取反，使得轮子物理方向与整体行驶方向保持一致，但舵轮只需非常小的角度变化。

 简单实现代码（C++ 风格伪代码）：

 ```cpp
 double normalizeAngle(double a){
     while(a > M_PI) a -= 2.0*M_PI;
     while(a <= -M_PI) a += 2.0*M_PI;
     return a;
 }

 double current = /* 当前角度, rad */;
 double desired = /* 期望角度, rad */;
 double speed = /* 期望线速度, m/s */;

 double delta = normalizeAngle(desired - current);
 if (std::fabs(delta) > M_PI_2) {
     // 翻转方向以减少轮子转动角度
     desired = normalizeAngle(desired - std::copysign(M_PI, delta));
     speed = -speed;
 }
 // 现在把 desired 和 speed 发送到舵轮控制器
 ```

```cpp
double delta = desired_angle - current_angle;
delta = normalizeAngle(delta);  // 归一化到 [-π, π]

if (std::abs(delta) > M_PI / 2.0) {
    // 翻转模式：调整角度，反转速度
    desired_angle += (delta > 0) ? -M_PI : M_PI;
    wheel_speed = -wheel_speed;
}
```

这样可以减少舵轮的旋转距离，提高响应速度。

### 14. 什么是运动学奇异点？舵轮底盘在什么情况下会遇到？

运动学奇异点是指逆运动学映射矩阵退化、无法唯一或无法实现某些基速度 (vx, vy, ω) 的场景。形式上，当将机器人底盘速度映射到各轮速度/角度的雅可比矩阵秩不足时，就出现奇异。

在舵轮底盘中常见情形：
- 轮子布置退化（比如所有轮子在一条直线上或位置向量共线），导致无法产生独立的转动/平移自由度；
- 当要求绕某个轮子所在点做纯旋转时，某些轮子的期望线速度会趋近于零或方向突变，舵向速度可能很大（角速度放大），导致控制器难以跟踪；
- 在数值解算中，若计算涉及除以极小量（如轮子到旋转中心距离 very small）会引起不稳定。

工程应对策略：
- 保证轮子位置参数合理且不共线；
- 对低速或接近奇异点的情况施加平滑与饱和（限制角速度和角加速度）；
- 在逆解中加入数值正则化（对小模长加 epsilon）；
- 基于优先级分配：在无法同时满足所有分量时优先保证旋转或平动中的关键分量。

### 16. 解释你代码中 normalizeAngle 函数的作用

`normalizeAngle` 的作用是把任意角度映射到一个连续区间（常用 (-π, π] 或 [0, 2π)），以避免角度跳变导致的控制误差。例如期望角 359° 与当前角 1° 的差应为 -2° 而不是 +358°。

实现要点：
- 使用循环或 fmod 保证数值稳定；
- 对边界（±π）的一致处理以避免不连续。示例实现：

```cpp
double normalizeAngle(double a){
        while (a > M_PI) a -= 2.0*M_PI;
        while (a <= -M_PI) a += 2.0*M_PI;
        return a;
}
```

该函数在计算角度差、决定最短旋转方向以及判定是否触发 180° 翻转逻辑时非常重要。

### 17. 全局坐标系和局部坐标系的速度转换是如何实现的？

把全局/地图系的线速度 (Vx_g, Vy_g) 转为机体局部坐标系 (Vx_b, Vy_b) 可用旋转变换：

$$\begin{pmatrix}Vx_b\\Vy_b\end{pmatrix} = R(-\theta) \begin{pmatrix}Vx_g\\Vy_g\end{pmatrix},\quad R(-\theta)=\begin{pmatrix}\cos\theta & \sin\theta\\-\sin\theta & \cos\theta\end{pmatrix}$$

其中 θ 为机器人朝向（从全局系到机体系的转角）。在 ROS 中常用 tf 或 tf2 来做这个变换（把 Twist 从 map 变换到 base_link）。示例代码：

```cpp
double yaw = getYawFromOdom();
double vx_b =  cos(yaw)*vx_g + sin(yaw)*vy_g;
double vy_b = -sin(yaw)*vx_g + cos(yaw)*vy_g;
```

注意时序一致性（使用相同时间戳的变换）和速度向量的参考点一致性（线速度表示质心速度或某固定点速度）。

### 18. 四个轮子的位置参数 (rx, ry) 在运动学中是如何使用的？

每个轮子的相对位矢 r_i=(r_xi, r_yi) 用于计算角速度引入的线速度分量：

$$v_{rot,i} = \omega \times r_i = \omega \begin{pmatrix}-r_{yi}\\ r_{xi}\end{pmatrix}$$

于是总体轮速为平动部分与旋转部分之和：
$$v_{i} = \begin{pmatrix}v_x\\v_y\end{pmatrix} + v_{rot,i}$$

因此 (rx,ry) 同时决定了：
- 轮子因车体旋转应有的切向速度大小（与距离成正比）；
- 当做纯旋转时，轮子的期望速度方向为与 r_i 垂直的切向方向；
- 在奇异点判断（如距离接近 0）时需要特殊处理。

### 19. 如何验证逆运动学实现的正确性？

验证方法包括：
1. **闭环一致性测试（forward–inverse round trip）**：给定随机基速度 (vx,vy,ω)，用逆运动学计算各轮期望（angle,speed），再用正运动学把这些轮子量合成为重建的基速度，比较误差应在数值容忍范围内。
2. **仿真对比**：在 Gazebo 中发布 cmd_vel，记录 odom，与理论轨迹对比。
3. **边界样例**：测试纯平移、纯旋转、以不同旋转中心旋转、以及接近奇异配置的输入，确保行为合理且受限饱和处理生效。
4. **单轮单元测试**：给定已知 rx,ry 与 ω，验证单个轮子的期望角度与速度与解析解一致。

### 20. 舵轮底盘能否实现原地旋转？如何实现？

可以。实现方法是把线速度设置为零，给每个轮子设置与旋转方向一致的切向速度：

对于纯角速度 ω（以机器人质心为中心）：
$$v_{xi} = -\omega r_{yi},\quad v_{yi} = \omega r_{xi}$$
轮速大小：
$$speed_i = |\omega| \sqrt{r_{xi}^2 + r_{yi}^2}$$
轮子角度：
$$\theta_i = \operatorname{atan2}(\omega r_{xi}, -\omega r_{yi}) = \operatorname{atan2}(r_{xi}, -r_{yi})\quad(\text{再结合 } \operatorname{sign}(\omega))$$

代码示例：

```cpp
double vx = 0.0, vy = 0.0;
for (auto &wheel : wheels) {
    double rx = wheel.rx, ry = wheel.ry;
    double vx_w = -omega * ry;
    double vy_w =  omega * rx;
    wheel.angle = std::atan2(vy_w, vx_w);
    wheel.speed = std::hypot(vx_w, vy_w);
}
```

注意：当某个轮子距离中心非常小时，wheel.speed 会很小但角度可能不稳定，应添加阈值避免角度抖动。

---

## 三、PID 控制

### 21. 解释 PID 控制器的三个参数 (P, I, D) 各自的作用

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau)d\tau + K_d \frac{de(t)}{dt}$$

| 参数 | 作用 | 过大影响 | 过小影响 |
|------|------|----------|----------|
| **P** | 比例响应，减小稳态误差 | 振荡、超调 | 响应慢、误差大 |
| **I** | 消除稳态误差 | 积分饱和、振荡 | 稳态误差残留 |
| **D** | 阻尼作用，减小超调 | 放大噪声 | 超调大 |

### 22. 什么是积分饱和（Integral Windup）？如何防止？

**积分饱和**：当控制量达到饱和（如电机最大力矩），误差持续存在，积分项持续累积，导致解除饱和后过度超调。

**防止方法**：
1. **积分限幅**（本项目使用）：
   ```cpp
   integral_ = std::clamp(integral_, -i_clamp, i_clamp);
   ```

2. **条件积分**：只在误差较小时启用积分

3. **反馈抑制**：根据输出饱和程度减小积分

### 26. 什么是低通滤波器？为什么在 PID 输出上使用？

**低通滤波器**：滤除高频信号，只保留低频成分。

一阶低通滤波器：
$$y_n = \alpha \cdot x_n + (1-\alpha) \cdot y_{n-1}$$

其中 $\alpha = \frac{\Delta t}{\tau + \Delta t}$，$\tau$ 是时间常数。

**使用原因**：
1. 滤除测量噪声
2. 平滑 D 项的微分计算
3. 减少执行器的高频振动
4. 提高系统稳定性

**时间常数选择**：
- τ 越大 → 滤波越强 → 响应越慢
- τ 越小 → 滤波越弱 → 响应越快但更受噪声影响
- 本项目默认 τ = 0.02s

### 23. 解释项目中的抗积分饱和策略

- I项限幅与反风up：将积分项限制在±i_clamp范围；
- 条件积分：在误差过大或输出饱和时减少或暂停积分累加；
- 输出饱和反馈：根据执行器饱和程度抑制积分增长。

### 24. 舵向PID与轮速PID参数为什么不同？

- 舵向是位置环，对角度误差敏感、需要合适的阻尼；
- 轮速是速度环，受负载与摩擦影响，I项更关键以消除稳态误差；
- 两者动态与噪声特性不同，故参数配置不同。

### 25. 如何调试PID参数？你使用了什么方法？

- 经验法与Ziegler–Nichols近似，先调P至响应适中，再加D抑制超调，最后加I消除稳态误差；
- 借助 rqt_reconfigure 在线微调，并用 rqt_plot观察响应曲线与误差；
- 在不同负载/速度下重复验证。

### 27. 低通滤波器的时间常数τ如何选择？

- 依据采样周期Δt、噪声水平与期望带宽：τ≈(1~3)×Δt起步；
- 噪声大→加大τ，响应慢→减小τ；
- 实测下微调以兼顾平滑与滞后。

### 28. 位置PID与速度PID的区别？舵向控制用哪种？

- 位置PID对角度误差积分，速度PID对角速度误差积分；
- 舵向控制通常采用位置PID（或带速前馈），轮速控制采用速度PID。

### 29. 如何使用 dynamic_reconfigure 在线调整PID参数？

- 在 cfg/WheelPid.cfg 中定义参数；控制器中创建动态重配置Server并在回调中更新PID；
- 使用 rqt_reconfigure 在运行中调整并观察效果。

### 30. PID输出振荡的可能原因与解决？

- P过大、D过小、采样噪声、延迟、执行器饱和；
- 降低P、增大D、加滤波、提高更新率、加入前馈与限幅。

---

## 四、功率控制

### 32. 解释你的功率控制数学模型：P(s) = a*s² + b*s + c

**模型假设**：电机功率由力矩和速度共同决定

$$P = \sum_i |F_i \cdot v_i|$$

其中力矩 $F_i$ 与控制指令 $cmd_i$ 成正比，速度 $v_i$ 由关节测量。

**引入缩放因子 s**：
- 对控制指令进行缩放：$cmd'_i = s \cdot cmd_i$
- 实际力矩：$F'_i = s \cdot F_i$

**推导二次方程**：
$$P(s) = \sum_i |s \cdot cmd_i \cdot v_i| + k_{effort} \cdot \sum_i (s \cdot cmd_i)^2 + k_{vel} \cdot \sum_i v_i^2$$

整理得：
$$P(s) = a \cdot s^2 + b \cdot s + c$$

其中：
- $a = k_{effort} \cdot \sum_i cmd_i^2$ （二次项系数）
- $b = \sum_i |cmd_i \cdot v_i|$ （一次项系数）
- $c = k_{vel} \cdot \sum_i v_i^2 - P_{offset} - P_{limit}$ （常数项）

### 36. 如何求解 s 使得 P(s) ≤ 0？

**求解条件**：$a \cdot s^2 + b \cdot s + c \leq 0$

使用求根公式：
$$s = \frac{-b + \sqrt{b^2 - 4ac}}{2a}$$

选择较大的根（$+\sqrt{}$），因为我们需要最大的允许缩放因子。

```cpp
double disc = b*b - 4.0*a*c;
if (disc > 0.0 && a != 0.0) {
    scaling_factor = (-b + std::sqrt(disc)) / (2.0 * a);
}
```

**物理意义**：
- $s = 1.0$：无需缩放，功率未超限
- $s < 1.0$：需要缩放，$cmd' = s \cdot cmd$
- $s > 1.0$：功率远未达到限制

### 39. 如果 scaling_factor 总是远大于 1，说明什么问题？

**诊断**：功率限制设置过于宽松，永远不会触发。

**可能原因**：
1. `effort_coeff` 参数过小
2. `power_limit` 参数过大
3. 控制指令或速度值很小

**解决方案**：
```yaml
# 调整参数
power_limiter:
  effort_coeff: 90.0    # 从 6.0 增加到 90-100
  power_limit: 300.0    # 或减小功率限制
```

**验证方法**：
1. 使用 rqt_plot 观察 `/power_debug/data[4]`
2. 通过 rqt_reconfigure 动态调整参数
3. 在大控制指令下验证 scaling_factor < 1

### 31. 为什么需要功率限制？在 RoboMaster 比赛中有什么意义？

- 赛规通常对功率或电流有上限；
- 防止过载、过热与供电跌落，保障可靠性与续航；
- 在对抗中维持稳定输出，避免瞬时峰值导致系统保护触发。

### 33. 系数 a, b, c 的物理含义？

- a：与力矩指令平方和相关，体现指令强度对功率的二次贡献；
- b：指令与速度的线性耦合项，体现做功率的线性部分；
- c：速度平方项与功率限/偏置的合成常数，决定是否越界。

### 34. 为什么用二次方程建模功率？

- 力矩指令与功率关系近似为平方项（等效损耗/铜损等随电流平方增长）；
- 与速度线性耦合形成一次项；
- 合并形成二次模型，便于解析求解与实时计算。

### 35. 缩放因子 s 的物理意义是什么？

- 对所有力矩指令的统一比例缩放，使得总功率不超过限制；
- s≤1表示限功；s=1表示未限功；s>1表示余量充足。

### 37. 何时触发功率限制？

- 当F(1)=a+b+c<0或判别式满足且解s<1时触发；
- 实际实现中以 scaling_factor<1 作为判定。

### 38. effort_coeff 和 velocity_coeff 如何标定？

- 结合实测电机功率曲线与指令、电机速度数据拟合；
- 先设定 velocity_coeff 基于空载/滚动损耗，再通过不同负载下的电流/功率测定调整 effort_coeff；
- 用rqt_plot与rosbag离线拟合优化。

### 40. 功率控制对响应速度的影响与平衡？

- 过强的限功会降低加速与爬坡能力；
- 通过调节 effort_coeff、power_limit、加前馈/缓升策略，在安全与性能间折中；
- 在瞬时需求高的动作中允许短时更大的power_limit或缓释策略。

---

## 五、键盘遥控

### 41. 解释键盘控制节点的状态机设计

```
        ┌──────────────────────────────────────────────┐
        │                                              │
        ▼                                              │
    ┌───────┐   w/a/s/d    ┌─────────────┐             │
    │ NONE  │─────────────▶│ TRANSLATION │─────────────┤
    └───────┘              └─────────────┘    timeout  │
        │                        │                     │
        │ q/e               'c' stop                   │
        ▼                        ▼                     │
    ┌───────────┐          ┌───────────┐               │
    │ ROTATION  │          │   STOP    │───────────────┘
    │ (latched) │          └───────────┘
    └───────────┘
        │
        │ 'c' stop
        ▼
    ┌───────────┐
    │   STOP    │
    └───────────┘
```

**设计要点**：
1. 平移和旋转互斥
2. 旋转采用锁存模式（按一次持续旋转）
3. 'c' 键紧急停止
4. 平移有超时自动停止

### 42. 如何实现非阻塞的键盘输入读取？

使用 `poll()` 系统调用：

```cpp
#include <poll.h>

struct pollfd fds[1];
fds[0].fd = STDIN_FILENO;
fds[0].events = POLLIN;

// 非阻塞检查是否有输入
int ret = poll(fds, 1, 0);  // 超时设为 0
if (ret > 0 && (fds[0].revents & POLLIN)) {
    char c;
    read(STDIN_FILENO, &c, 1);
    // 处理按键
}
```

**优势**：
- 不阻塞主循环
- 可以同时处理其他任务
- 可以设置超时时间

### 43. 什么是终端的 raw 模式？为什么需要它？

**默认模式（cooked mode）**：
- 行缓冲：需要按回车才能读取输入
- 回显：输入的字符会显示在屏幕上
- 特殊字符处理：Ctrl+C 等有特殊含义

**Raw 模式**：
- 无缓冲：立即读取每个按键
- 无回显：按键不显示
- 禁用特殊处理：可以捕获所有按键

```cpp
struct termios raw;
tcgetattr(STDIN_FILENO, &raw);
raw.c_lflag &= ~(ICANON | ECHO);  // 关闭行缓冲和回显
tcsetattr(STDIN_FILENO, TCSANOW, &raw);
```

### 46. 如何实现速度的实时调节功能？(u/i/o/p 键)

**实现方式**：成员变量 + 按键增减

```cpp
class TeleopKeyboard {
private:
    double walk_vel_ = 0.5;      // 平移速度
    double default_omega_ = 1.0; // 角速度

public:
    void processKey(char c) {
        switch(c) {
        case 'u':  // 增加平移速度
            walk_vel_ += 0.1;
            if (walk_vel_ > 5.0) walk_vel_ = 5.0;
            ROS_INFO("Walk velocity: %.2f m/s", walk_vel_);
            break;
        case 'i':  // 减小平移速度
            walk_vel_ -= 0.1;
            if (walk_vel_ < 0.1) walk_vel_ = 0.1;
            break;
        case 'o':  // 增加角速度
            default_omega_ += 0.1;
            if (default_omega_ > 5.0) default_omega_ = 5.0;
            break;
        case 'p':  // 减小角速度
            default_omega_ -= 0.1;
            if (default_omega_ < 0.1) default_omega_ = 0.1;
            break;
        }
    }
};
```

**设计要点**：
- 步长 0.1，范围 0.1-5.0
- 实时反馈当前速度值
- 旋转锁存时同步更新锁存值

### 44. 解释锁存模式（latched mode）的设计思路

- 单次按键（q/e）即可进入持续旋转状态，减少长按负担；
- 再次按键或紧急停止（c）退出锁存；
- 锁存值随 o/p 调整同步更新。

### 45. 平移和旋转为什么互斥？

- 降低复杂按键组合导致的误操作；
- 保证旋转时舵向保持一致，避免同时平移带来的轨迹不确定性；
- 简化状态机与用户体验。

### 47. 紧急停止如何实现？

- 接收到 'c' 将所有按键状态清零、锁存关闭、速度设为0；
- 立即发布零Twist并清理内部状态。

### 48. 多按键同时按下的处理？

- 采用互斥逻辑与优先级，或按最后按下的键生效；
- 对 w/a/s/d 组合做向量合成或限制为单轴；
- 保持安全优先（c最高优先级）。

### 49. 为什么用 poll() 而不是 read()？

- poll支持非阻塞与超时，便于在主循环中轮询输入；
- read易阻塞导致控制周期中断。

### 50. 退出时如何恢复终端设置？

- 在析构或捕获Ctrl-C时调用 tcsetattr 恢复原有termios配置；
- 保障后续终端正常使用。

---

## 六、TF 与坐标变换

### 51. 什么是 TF？在 ROS 中的作用是什么？

TF (Transform) 是 ROS 中的坐标变换库，用于跟踪多个坐标系之间的关系。

**核心功能**：
1. 维护坐标系树
2. 时间同步的变换查询
3. 坐标点/向量变换

**常见坐标系**：
```
map → odom → base_link → sensor_link
```

### 54. 全局速度模式下，为什么需要坐标变换？
### 52. 解释 odom→base_link 变换的含义

- odom为漂移较小的短期参考系，base_link为机器人机体坐标系；
- 变换描述机器人在世界系中的位置与姿态，用于速度/位置的坐标转换与导航。

### 53. 如何使用 tf::TransformListener 获取变换？

- 构造listener，调用 lookupTransform("odom","base_link", ros::Time(0), transform)；
- 处理异常与超时，必要时重试或回退到局部控制。

### 55. 如何处理 TF 查询失败？

- 记录警告、保持上次有效变换或切换到本体坐标控制；
- 增加查询超时时间与缓冲；
- 检查TF树发布频率与连接。

### 56. lookupTransform 的超时如何设置？

- 通过 tf::Transformer 或 tf2 的缓冲区超时参数；
- 结合控制周期设置一个小于周期的安全超时，如 10–50 ms。

**问题**：键盘输入的速度是相对于世界坐标系（odom），但控制器需要相对于机器人坐标系（base_link）的速度。

**解决方案**：
```cpp
// 获取机器人在世界系中的朝向
tf::StampedTransform transform;
tf_listener_.lookupTransform("odom", "base_link", ros::Time(0), transform);
double yaw = tf::getYaw(transform.getRotation());

// 将世界系速度转换到机器人系
double vx_local = vx_world * cos(yaw) + vy_world * sin(yaw);
double vy_local = -vx_world * sin(yaw) + vy_world * cos(yaw);
```

---

## 七、Dynamic Reconfigure

### 如何使用 dynamic_reconfigure 实现参数在线调整？

**1. 创建配置文件 (.cfg)**：
```python
#!/usr/bin/env python
from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()
gen.add("effort_coeff", double_t, 0, "Power effort coefficient", 6.0, 0.0, 200.0)
gen.add("power_limit", double_t, 0, "Power limit (W)", 300.0, 0.0, 1000.0)

exit(gen.generate("package_name", "node_name", "Config"))
```

**2. 在控制器中使用**：
```cpp
#include <dynamic_reconfigure/server.h>
#include <package_name/ConfigConfig.h>

class Controller {
    dynamic_reconfigure::Server<Config> dyn_server_;
    
    void init() {
        dyn_server_.setCallback(
            boost::bind(&Controller::reconfigureCallback, this, _1, _2));
    }
    
    void reconfigureCallback(Config& config, uint32_t level) {
        effort_coeff_ = config.effort_coeff;
        power_limit_ = config.power_limit;
        ROS_INFO("Parameters updated!");
    }
};
```

**3. 使用 rqt_reconfigure 调整**：
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

---

## 八、调试与优化

### 71. 如何使用 rqt_plot 可视化控制数据？

```bash
# 查看功率控制指标
rqt_plot /power_debug/data[4]:label=scaling_factor \
         /power_debug/data[5]:label=F_1 \
         /power_debug/data[10]:label=limited

# 查看 PID 输出
rqt_plot /joint_states/velocity[0] /joint_states/effort[0]
```

**/power_debug 话题字段说明**：
| 索引 | 字段 | 说明 |
|------|------|------|
| data[0] | a | 二次项系数 |
| data[1] | b | 一次项系数 |
| data[2] | c | 常数项 |
| data[3] | disc | 判别式 |
| data[4] | scaling_factor | **缩放因子（关键指标）** |
| data[5] | F(1) | s=1 时的函数值 |
| data[6] | Σ(cmd²) | 指令平方和 |
| data[7] | Σ(vel²) | 速度平方和 |
| data[8] | r1 | 较大的根 |
| data[9] | r2 | 较小的根 |
| data[10] | limited | 是否限制 |

### 72. 如何使用 rosbag 记录与回放数据？

- 记录：rosbag record -O test.bag /power_debug /cmd_vel /joint_states 等；
- 回放：rosbag play test.bag；
- 导出：rostopic echo -b test.bag -p /power_debug > power.csv。

### 73. 如何分析 /power_debug 数据？

- 关注 scaling_factor(data[4])与F(1)(data[5])；
- 结合 a,b,c 与根r1,r2判断是否越界与余量；
- 对比 Σ(cmd²)与Σ(vel²)理解负载与速度影响。

### 74. 控制不稳定如何排查？

- 先看PID参数与滤波；再看功率限制是否频繁触发；
- 检查TF与里程计一致性；
- 限制输入、分步隔离问题并用rosbag复现。

### 75. 如何优化控制器性能？

- 前馈项、限幅与缓升、抗饱和、合理滤波；
- 提升更新频率与降低延迟；
- 针对不同模式使用不同参数集。

### 74. 遇到控制不稳定时，如何排查问题？

**排查步骤**：

1. **检查 PID 参数**：
   - P 过大 → 振荡
   - D 过大 → 对噪声敏感
   - 使用 rqt_reconfigure 动态调整

2. **检查低通滤波**：
   - τ 过小 → 噪声放大
   - τ 过大 → 响应迟钝

3. **检查功率限制**：
   - scaling_factor 频繁 < 1 可能导致输出抖动
   - 增大 power_limit 或减小 effort_coeff

4. **查看话题频率**：
   ```bash
   rostopic hz /cmd_vel
   rostopic hz /joint_states
   ```

5. **记录数据分析**：
   ```bash
   rosbag record -O debug.bag /power_debug /cmd_vel /joint_states
   ```

---

## 九、扩展问题

### 76. 如果要添加自主导航功能，需要做哪些修改？

1. **添加传感器**：激光雷达、IMU、编码器
2. **建图与定位**：gmapping、AMCL 或 Cartographer
3. **路径规划**：move_base、Navigation2
4. **接口适配**：
   - 接收 `/cmd_vel` 替代键盘输入
   - 发布准确的里程计和 TF

### 77. 如何将此控制器移植到实际硬件上？

1. **编写硬件接口**：
   ```cpp
   class MyRobotHW : public hardware_interface::RobotHW {
       void read();   // 从硬件读取状态
       void write();  // 向硬件写入命令
   };
   ```

2. **通信接口**：CAN 总线、串口、EtherCAT 等

3. **参数标定**：
   - 重新标定 PID 参数
   - 标定功率系数（effort_coeff, velocity_coeff）
   - 测量实际电机功率曲线

4. **安全措施**：
   - 限位开关
   - 紧急停止
   - 通信超时检测

---

### 78. 与 ROS2 的 ros2_control 有什么区别？

- ROS2 的 ros2_control 基于 rclcpp 与新型执行器模型，线程与实时支持更好；
- 参数、插件、生命周期管理接口有所差异；
- 迁移需要适配接口与执行环境。

### 79. 如何实现多机器人协同控制？

- 去中心化或中心化调度；
- 使用命名空间隔离话题，共享地图/TF；
- 协议层实现任务分配与避碰（如分布式MPC或行为树）。

### 80. 提升更高动态性能的改进方向？

- 增加前馈与模型预测控制（MPC）；
- 更快的更新率与实时内核；
- 更精确的传感器与状态估计（IMU融合、卡尔曼滤波）。

**答案总数**: 对应 80 个问题  
**建议**: 重点掌握功率控制数学模型、逆运动学推导、PID 调试方法  
**答辩技巧**: 结合代码讲解，用 rqt_plot 演示实时效果

---

## 十二、代码实现与函数设计细节

### 81. 解释 `init()` 函数中的初始化顺序，为什么要先获取关节句柄再读取参数？

- 先获取关节句柄可立即验证硬件资源是否存在，若失败直接返回 false，避免后续参数读取浪费时间。
- 许多参数（如关节名称数组）在 `init()` 中通过句柄验证；确保句柄成功后再读取依赖这些句柄的信息更安全。
- 顺序通常为：解析 joint_names → 通过 `hw->getHandle()` 获取 → 读取 PID/功率等参数 → 初始化内部状态。

### 82. `update()` 函数的调用频率是多少？如何保证实时性？

- 在我们的控制器中，`update()` 由 controller_manager 以固定频率调用（默认 500–1000 Hz）。
- 保证实时性的措施：
    1. 避免在 `update()` 中做内存分配、系统调用或阻塞操作；
    2. 预先缓存句柄和配置；
    3. 将复杂计算（如路径规划）移到低频线程，只在 `update()` 中读取结果。

### 83. 在 `update()` 中如何处理时间戳？`period` 参数的作用是什么？

- controller_manager 调用 `update(now, period)`，`period` 是距离上次调用的时间差（Duration）。
- PID 和滤波器都需要 `period` 来计算积分累积与微分项，让控制与真实时间同步。
- 时间戳 `now` 用于发布消息、采样 TF，与 odom/IMU 数据对齐。

### 84. 控制器的 `starting()` 和 `stopping()` 函数中应该做哪些状态重置？

- `starting()`：重置 PID 积分项、舵向/轮速目标、功率统计量，使系统以干净状态开始；可发布零输出避免冲击。
- `stopping()`：将输出置零或安全值，保存最后时间戳，防止控制器停止后硬件仍保持旧命令。

### 85. 如何在代码中区分仿真环境和实际硬件环境？

- 通过参数（如 `controller_nh.param("simulated", false)`) 或 `ros::param::get("/use_sim_time")` 判断。
- 也可检查是否加载了 Gazebo 插件或读取 `hardware_interface::RobotHW` 的派生类型。
- 根据环境切换特定功能：仿真中关闭安全看门狗，实机中启用限流与冗余检测。

### 86. `calculateInverseKinematics()` 函数的输入输出是什么？如何处理数值不稳定？

- 输入：机器人基坐标速度 (vx, vy, ω) 以及每个轮子的 (rx, ry)。
- 输出：每个轮子的期望角度、线速度。
- 数值稳定：在计算 `hypot()` 和 `atan2()` 时加入 epsilon 判断，若速度过小则保持上一帧角度；对 ω→0 时的旋转分量设阈值防止抖动。

### 87. 当轮子速度非常小（接近零）时，如何避免 `atan2()` 的方向跳变？

- 判断 `std::hypot(vx, vy) < v_threshold` 时直接返回上一帧角度。
- 或将角度更新改为 `normalizeAngle(prev + atan2(vy, vx) - prev)` 的增量方式，保证连续性。
- 设置最小速度阈值（如 1e-3 m/s）避免噪声导致方向翻转。

### 88. 解释 `handleWheelFlip()` 函数的具体逻辑和边界条件

- 计算期望角与当前角的差 delta；若 |delta|>90°，则把期望角 ±180° 并反转轮速。
- 边界：当 delta 接近 ±90° 时需考虑滞回，避免频繁翻转；可设置双阈值（进入翻转 >100° 离开翻转 <80°）。
- 翻转后要再次归一化角度，确保发送到舵向 PID 的值在 (-π, π] 内。

### 89. 为什么需要对角度差进行归一化？不归一化会导致什么问题？

- 因为角度具有周期性，若直接使用差值，359° 与 1° 之间会被认为相差 358°，导致 PID 输出错误。
- 归一化到 (-π, π] 可保证控制器总是沿最短路径旋转，减少超调。
- 不归一化可能导致舵向无穷旋转、积分爆炸。

### 90. 在运动学计算中如何处理浮点精度误差？

- 为比较值设置 epsilon（如 1e-6），仅当 |x|>epsilon 才认为非零。
- 对向量长度使用 `std::hypot` 以降低溢出/下溢风险。
- 对角度使用 `normalizeAngle()` 防止累计误差。

### 91. `PIDController` 类的成员变量有哪些？各自的作用是什么？

- `kp, ki, kd`：比例、积分、微分增益；
- `integral_`：积分累积；
- `prev_error_` 或滤波状态：用于计算微分；
- `i_clamp_`：积分限幅；
- `output_limit_`：最终输出饱和；
- `lowpass_` 状态：用于 D 项滤波。

### 92. `compute()` 函数中微分项是如何计算的？为什么要用滤波后的误差？

- 典型实现：
    ```cpp
    double error = setpoint - measurement;
    double derivative = (error - filtered_prev_error) / dt;
    filtered_prev_error = alpha * error + (1-alpha) * filtered_prev_error;
    ```
- 使用滤波后的误差可以抑制测量噪声放大，使 D 项更平滑。

### 93. 积分项的限幅值（`i_clamp`）如何确定？

- 根据执行器可提供的最大力矩推算：`i_clamp ≈ (output_max / ki)`。
- 过大：易出现积分饱和和长时间超调；过小：难以消除稳态误差。
- 可以通过实验，在稳态时逐渐增大 `i_clamp` 至刚好消除误差即可。

### 94. 如何在代码中实现输出饱和限制？饱和后对积分项有什么处理？

- 输出饱和：`cmd = std::clamp(cmd, -max_effort, max_effort);`
- 若输出被饱和且 PI 差导致饱和，则可暂停积分（Anti-Windup）：
    ```cpp
    if ((cmd == max && error > 0) || (cmd == -max && error < 0)) {
            integral_ -= ki * error * dt; // 或不再积分
    }
    ```

### 95. PID 的 `reset()` 函数应该在什么时候调用？

- 控制器启动、停止、模式切换、输入信号跳变较大（如紧急制动后恢复）时调用。
- `reset()` 应清零积分项、微分状态、上次误差。

### 96. `PowerLimiter::limit()` 函数的完整执行流程？

1. 统计 Σ(cmd²)、Σ(cmd·vel)、Σ(vel²)；
2. 计算 a,b,c；
3. 若 a≈0 且 b≤0，直接返回不限制；
4. 计算判别式 disc；若 disc<0，说明即使 s=0 仍超限，通常将 scaling_factor 置 0；
5. 取 s = (-b + sqrt(disc)) / (2a)，并裁剪到 [0,1]；
6. 用 s 统一缩放四个轮子的力矩指令；
7. 若开启 debug，则发布 `/power_debug`。

### 97. 如何处理 `disc < 0` 的情况？物理意义是什么？

- `disc < 0` 表示方程无实根，即无论怎样缩放都无法使功率 ≤ 限制（c>0 且 a≤0）。
- 物理意义：速度项或偏置太大，表示系统即使不再加力也超出功率预算（比如高速度巡航时 k_vel 太大）。
- 处理：将 scaling_factor 设为 0 或最小值，主动触发降速/刹车，并记录警告。

### 98. `scaling_factor` 如何应用到各个轮子的命令？

- 统一缩放：`cmd_i = scaling_factor * cmd_i`，保证总功率按比例降低而保持方向一致。
- 若某些轮子允许单独限功，可在统一缩放后再根据各轮功率占比进行微调，但当前实现是统一系数。

### 99. `/power_debug` 发布 12 个字段的意义？

- 便于在 rqt_plot 中观察：a(0)、b(1)、c(2)、判别式(3)、scaling_factor(4)、F(1)(5)、Σcmd²(6)、Σvel²(7)、根 r1/r2(8/9)、是否限功标志(10)、b_copy(11)。
- 帮助分析功率触发原因（是速度项还是指令项导致）。

### 100. 功率限制是在速度解算前还是解算后应用？

- 在轮速/舵角解算后、发送力矩指令前应用。先得到期望 cmd → 功率模型判断 → 统一缩放。
- 这样可保持运动学一致性，仅调整幅值。

### 101. 轮子数据如何组织存储？

- 使用结构体 `Wheel`（包含 joint handle、rx,ry、当前角度等）组成 `std::vector<Wheel>`。
- 统一容器便于循环处理和与 URDF 配置对应。

### 102. 为什么使用 `std::vector`？

- 轮子数量可能在仿真与实机间不同；vector 便于在初始化时根据参数动态填充。
- 方便使用 STL 算法（for_each、transform）迭代。

### 103. 如何确保四个轮子的索引顺序一致？

- 在参数文件中约定顺序（LF, RF, LR, RR），在读取 joint_names 时按照此顺序 push_back。
- 对结构体添加 `name` 字段，并在调试输出中打印，便于检测错误。

### 104. 动态重配置回调如何线程安全？

- dynamic_reconfigure 在独立线程运行；回调中更新共享变量需使用 mutex 或原子变量。
- 在我们的实现中，回调只更新简单的 double 值，且在 update 中读取的频率高，可采用 `std::atomic<double>` 或在回调中加锁。

### 105. 关节句柄获取失败时应该如何处理？

- `init()` 返回 `false` 或抛出 `HardwareInterfaceException`，ROS 日志输出 ERROR，提示缺失的 joint 名称。
- 终止控制器加载，提醒检查 URDF 或 hardware_interface。

### 106. 如果 `cmd_vel` 长时间没有更新怎么处理？

- 设置超时（如 0.5s），若超时则逐渐减小目标速度至 0（dead-man switch）。
- 实现：记录最新 `cmd_stamp_`，在 `update()` 中比较 `now - cmd_stamp_`。

### 107. 速度命令的饱和限制在哪里实现？

- 在速度解算结果之后调用 `saturate(value, max_linear, max_angular)`。
- 参数在 YAML 中配置，可通过 dynamic_reconfigure 在线修改。

### 108. 舵向角度传感器异常如何检测？

- 检查角度跳变：若 |θ(t)-θ(t-1)| > π，则判定异常。
- 可对传感器值加中值滤波，若多次超出范围则触发错误状态（WARN/ERROR）。

### 109. 如何防止除零错误？

- 在计算 scaling_factor 前判断 `fabs(a) < epsilon`；若是则改用线性解或直接返回。
- 在运动学中处理 `wheel_speed ≈ 0` 时避免 `vx / speed` 形式。

### 110. `keyLoop()` 是阻塞还是非阻塞？

- 通过 `poll()` + 非阻塞 read 实现“准实时”循环，主循环不会因没有键盘输入而阻塞。
- 当 poll 超时时，会继续循环发布最近一次命令。

### 111. 终端属性的保存与恢复在哪实现？

- 使用 `tcgetattr` 保存原属性，`tcsetattr` 设置 raw 模式；在析构或节点退出前恢复。
- 为防止异常退出导致终端异常，注册 `atexit` 或使用 RAII 对象。

### 112. `poll()` 的超时参数设置为多少？

- 典型值 100ms；足够响应用户输入，又不会占用过多 CPU。
- 可通过参数配置，以适应不同串口/终端速度。

### 113. 速度增减步长（0.1）是否可配置？

- 通过 ROS 参数 `walk_step`, `omega_step` 设置；若未配置则使用默认 0.1。
- 在键盘节点启动时读取，按键事件里使用最新值。

### 114. 如何确保按键响应实时？

- 使用非阻塞输入 + 高频循环；
- 对每次键盘事件立即更新内部状态并发布 cmd_vel；
- 限制控制频率（如 20 Hz）以防刷屏。

### 115. `param()` 与 `getParam()` 区别？

- `param(name, default)`：若参数不存在会写入默认值；
- `getParam(name, value)`：若不存在返回 false，value 不变。
- `param` 更适合有默认值的参数，`getParam` 适合检查必需参数。

### 116. 动态重配置服务器的生命周期如何管理？

- 在控制器 `init()` 中创建 `server_.reset(new Server(controller_nh));`，并保存到智能指针中。
- 在控制器析构或停止时自动销毁；也可在 `stopping()` 中 `server_.reset()`。

### 117. 参数文件缺少必需参数时怎么处理？

- `if (!controller_nh.getParam("wheel_radius", radius_)) { ROS_ERROR(...); return false; }`
- 通过返回失败阻止控制器加载，避免运行时崩溃。

### 118. 多个控制器实例如何避免命名空间冲突？

- 在 launch 或 YAML 中为不同实例设置不同 `name/ns`，控制器内部使用 `controller_nh`（已包含前缀）。
- 对共享参数（如功率限制）使用全局命名空间 `/power_limiter`，其余使用私有命名空间 `~`。

### 119. 如何验证参数合法性？

- 读取后立即做范围检查（如 kp>0），若非法则打印 ERROR 并返回。
- 对组合参数（如四个轮子坐标）验证长度一致。

### 120. 代码中 ROS 日志级别如何使用？

- `ROS_DEBUG`：高频细节（更新循环）；
- `ROS_INFO`：关键状态（控制器启动、参数加载）；
- `ROS_WARN`：非致命异常（cmd_vel 超时）；
- `ROS_ERROR`：致命错误（句柄缺失、求根失败）。

### 121. 如何通过日志定位初始化失败？

- 在 `init()` 的每个关键步骤（参数读取、句柄获取）打印日志，配合 `roslaunch --screen` 查看。
- 若返回 false，controller_manager 会打印“Failed to initialize the controller”并附带日志。

### 122. `/power_debug` 发布频率是多少？

- 默认每 50 ms（20 Hz）发布一次，以平衡观察精度与带宽。
- 通过参数 `debug_publish_period` 配置，调试阶段可提高频率。

### 123. 如何在不改代码的情况下开启更详细的调试输出？

- 使用 `rosconsole` 配置：
    ```bash
    rosconsole set /sentry_chassis_controller DEBUG
    ```
- 或在 launch 文件中设置 `<env name="ROSCONSOLE_CONFIG_FILE" value="..."/>`。

### 124. 性能敏感路径中应避免哪些操作？

- 避免 string 拼接、日志打印、动态分配、IO（如读文件）。
- 高度复杂的数学运算应预计算，循环中只进行简单操作。

### 125. 运动学计算中哪些部分可预计算？

- 轮子位置 (rx, ry) 的常用组合：`rx^2+ry^2`、`atan2(rx,-ry)` 等。
- 逆矩阵或旋转矩阵可在初始化时生成常数表。

### 126. `std::sqrt()` 和 `std::atan2()` 调用频率的影响？

- 它们是较慢的数学函数，频繁调用会增加 CPU 占用。
- 可通过查表、近似或仅在必要时调用（如速度大于阈值时）。

### 127. 如何减少内存分配？

- 预先 `reserve()` 容器容量；
- 使用栈对象或成员缓存（例如 `std::array<double,4>`）；
- 禁止在 update 中创建临时 string/stream。

### 128. 低通滤波器能否用查表加速？

- 可以，将 `alpha = dt/(tau+dt)` 预计算，对多轮使用相同 alpha；
- 若 τ 变化不大，可用小表格存储 `alpha` 与 `exp(-dt/tau)`。

### 129. 如果要支持 6/8 轮，需要做哪些修改？

- 参数化轮子数量：不再硬编码 4，使用 vector 遍历；
- 功率求和、运动学求解改为循环累计；
- 键盘/上层接口可能需扩展以适配不同拓扑。

### 130. 如何为逆运动学函数编写单元测试？

- 使用 gtest：输入已知的 (vx,vy,ω) 和轮坐标，检查输出角度/速度是否与手算一致。
- 测试普通平移、纯旋转、零输入、奇异接近等场景。

### 131. PID 控制器单元测试应覆盖哪些场景？

- 阶跃响应：验证输出随误差变化；
- 斜坡输入：检查积分能否消除稳态误差；
- 噪声输入：观察 D 项滤波效果；
- 饱和与 anti-windup 逻辑。

### 132. 功率限制器如何测试？

- 构造一组 cmd/vel，手工计算预期 scaling_factor，与函数输出比对；
- 测试 disc<0、a≈0、scaling_factor>1 等边界条件。

### 133. 如何模拟传感器噪声测试滤波器？

- 在单元测试中给 PID 输入叠加随机噪声（高斯），观察滤波后信号统计特性；
- 或在 Gazebo 中向关节状态添加噪声插件。

### 134. 集成测试如何验证键盘控制？

- 运行键盘节点，录制 `/cmd_vel` 与 odom，检查按键对应的动作为正向、侧移、旋转；
- 使用自动化脚本模拟按键（`xte` 或 Python `pty`）并比对期望输出。

---

**答案总数**: 对应 134 个问题  
**建议**: 重点掌握功率控制数学模型、逆运动学、PID、代码实现细节  
**答辩技巧**: 结合代码、rqt_plot、rosbag 示例展示调试过程
