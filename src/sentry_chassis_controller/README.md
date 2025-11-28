# sentry_chassis_controller

> 舵轮底盘控制器 (swerve) – 支持 PID 舵角/轮速、逆/正运动学、功率限制、漂移抑制与“自锁”保持功能。

## 功能概览
- 8 路 PID：4 个舵角 + 4 个轮速 (control_toolbox::Pid)
- 逆运动学：订阅 `/cmd_vel` (Twist) 计算每个模块舵角与轮速
- 正运动学：利用舵角 + 轮速最小二乘求解底盘 (vx, vy, wz) 并积分发布 `/odom` 及 TF (`odom -> base_link`)
- 功率限制：按照 effort/velocity 二次项估算，缩放 4 轮力矩避免超功率
- 漂移抑制：静止时估计并消除微小偏置 (EMA + deadband)
- 自锁保持：无指令时检测残余速度，自动生成反向补偿轮速，降低惯性/斜坡导致的缓慢位移

## 关键参数 (wheel_pid_params.yaml)
```yaml
wheel_pid_controller:
  wheel_track: 0.36
  wheel_base: 0.36
  power_limit: 100.0
  power:
    effort_coeff: 12.0
    velocity_coeff: 0.0048
    power_offset: 0.0
  drift:
    bias_alpha: 0.02
    deadband_trans: 0.005
    deadband_rot: 0.005
    min_wheel_speed_sum: 0.02
    bias_enabled: true
  self_lock:
    enabled: true
    gain: 1.0
    trans_thresh: 0.01
    rot_thresh: 0.02
    pivot_freeze: true
```
### 自锁逻辑说明
1. 判断“原始指令”为零：`sum(|wheel_cmd_i|) < 1e-6`。
2. 快速前向运动学估计当前机体速度 `(vx, vy, wz)`。
3. 若 `hypot(vx, vy) > trans_thresh` 或 `|wz| > rot_thresh`：
   - 使用逆运动学计算补偿命令：`cmd = inverseKinematics(-gain*vx, -gain*vy, -gain*wz)`。
   - 若 `pivot_freeze=true` 只覆盖轮速，保持当前舵角；否则同时重设舵角以最佳抵消。
4. 未达阈值：轮速保持 0，防止微小噪声持续积分。

### 调参建议
| 目的 | 参数 | 建议范围 | 说明 |
|------|------|----------|------|
| 更快消除缓慢漂移 | `drift.bias_alpha` | 0.02 – 0.1 | 越大收敛越快但不够平滑 |
| 降低虚假运动触发 | `drift.deadband_trans` / `rot` | 0.003 – 0.01 | 过大将吞掉真实很慢运动 |
| 减少自锁抖动 | `self_lock.gain` | 0.6 – 1.0 | <1 留少量残差防止来回振荡 |
| 提高触发灵敏度 | `self_lock.trans_thresh` | 0.005 – 0.02 | 降低会更容易补偿但可能频繁切换 |
| 舵角是否保持 | `self_lock.pivot_freeze` | true/false | false 时可能频繁转向造成噪声 |

调试流程：
1. 关闭自锁：`enabled=false`，观察纯漂移抑制效果。
2. 调整 `bias_alpha` 与 deadband 使静止 `/odom` 线速度接近 0 且不跳变。
3. 开启自锁：`enabled=true`，以手推或斜坡方式施加微扰，观察补偿是否平滑收敛。
4. 若出现“快停快速反向抖动”，降低 `gain` 或提升触发阈值。

## 启动
```bash
roslaunch sentry_chassis_controller sentry_with_odom.launch
```
- Launch 文件会加载 `config/wheel_pid_params.yaml` 并生成控制器 + `/odom`。

## 话题
| Topic | 类型 | 描述 |
|-------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 输入底盘目标速度 (底盘坐标系或全局，取决于上游) |
| `/odom` | `nav_msgs/Odometry` | 发布里程计（含漂移抑制 + 自锁后的速度）|
| `/tf` | TF | `odom -> base_link` 变换 |
| `/desired_wheel_states` | `sensor_msgs/JointState` | 逆运动学结果（舵角=position, 轮速=velocity）|
| `/joint_states` | `sensor_msgs/JointState` | 实际关节状态（用于 FK）|
| `/applied_wheel_efforts` | `sensor_msgs/JointState` | 当前输出 effort |
| `/power_debug` | `std_msgs/Float64MultiArray` | 功率限制调试数据 |

## 常见问题
- 自锁不触发：检查 `self_lock/enabled` 是否为 true 且原始指令确实为零；确认阈值不太高。
- 振荡：降低 `gain` 或提高 `trans_thresh/rot_thresh`；开启 `pivot_freeze`。
- 纯旋转仍出现平移：确认 pivot 关节状态发布正常；查看 FK 部分是否日志显示奇异矩阵回退。

## 后续可扩展
- 添加自锁状态话题 `/self_lock_state` (bool + 当前补偿速度)
- 引入积分式位置保持（记录最后姿态，对漂移进行基于位姿误差的闭环，而不是速度闭环）
- 对功率限制与自锁耦合：高功率时弱化自锁补偿防止过载

欢迎根据需求继续优化。
