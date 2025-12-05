# 功率控制可视化和调试指南

## 一、功能概述

### 1.1 键盘控制增强功能
在 `sentry_control_key_feature` 节点中新增了速度实时调整功能：

| 按键 | 功能 | 步长 | 范围 |
|------|------|------|------|
| `u` | 增加平移速度 | +0.1 m/s | 0.1 - 5.0 m/s |
| `i` | 减小平移速度 | -0.1 m/s | 0.1 - 5.0 m/s |
| `o` | 增加角速度 | +0.1 rad/s | 0.1 - 5.0 rad/s |
| `p` | 减小角速度 | -0.1 rad/s | 0.1 - 5.0 rad/s |

### 1.2 功率控制指标话题
`/power_debug` 话题（`std_msgs/Float64MultiArray`）现在发布 12 个指标：

| 索引 | 字段名 | 说明 | 用途 |
|------|--------|------|------|
| data[0] | a | 二次项系数 `effort_coeff * Σ(cmd²)` | 分析控制指令的影响 |
| data[1] | b | 一次项系数 `Σ|cmd*vel|` | 分析功率主要来源 |
| data[2] | c | 常数项 `velocity_coeff*Σ(vel²) - power_offset - power_limit` | 判断是否超限 |
| data[3] | disc | 判别式 `b² - 4ac` | 判断方程是否有实根 |
| data[4] | **scaling_factor** | 缩放因子 s | **关键指标：s<1 表示功率被限制** |
| data[5] | F(1) | `a + b + c` | s=1时的函数值（负值表示需要限制） |
| data[6] | sum_cmd² | `Σ(cmd_i²)` | 控制指令的平方和 |
| data[7] | sum_vel² | `Σ(vel_i²)` | 速度的平方和 |
| data[8] | r1 | 较大的根 `(-b+√disc)/(2a)` | 功率约束的上界 |
| data[9] | r2 | 较小的根 `(-b-√disc)/(2a)` | 功率约束的下界（通常为负） |
| data[10] | limited | 是否限制 (1.0/0.0) | 指示当前是否触发功率限制 |
| data[11] | b_copy | b系数副本 | 方便单独绘图 |

## 二、使用 rqt_plot 可视化

### 2.1 启动系统
```bash
# 终端1: 启动仿真
roslaunch sentry_sim sentry_sim.launch

# 终端2: 启动键盘控制
rosrun sentry_chassis_controller sentry_control_key_feature
```

### 2.2 查看关键指标
```bash
# 查看缩放因子（最重要）
rqt_plot /power_debug/data[4]

# 同时查看多个指标
rqt_plot /power_debug/data[4] /power_debug/data[5] /power_debug/data[10]
```

### 2.3 指标解读

#### **核心判断逻辑**
- `scaling_factor < 1.0` → 功率被限制（指令被缩放）
- `scaling_factor = 1.0` → 功率未超限
- `scaling_factor > 1.0` → 功率远未达到限制（说明限制参数过于宽松）

#### **F(1) 的意义**
- `F(1) < 0` → 在 s=1 时功率超限，需要缩小指令
- `F(1) > 0` → 在 s=1 时功率未超限，无需限制
- `F(1) ≈ 0` → 临界状态

#### **参数调试建议**
如果 `scaling_factor` 总是远大于 1（如 3.96），说明：
- `effort_coeff` 设置过小（当前为 6.0）
- 建议增加到 90-100，使其在大控制指令下触发限制

### 2.4 完整可视化配置

**基础监控（3条曲线）**
```bash
rqt_plot /power_debug/data[4]:label=scaling_factor \
         /power_debug/data[5]:label=F_1 \
         /power_debug/data[10]:label=limited
```

**详细分析（6条曲线）**
```bash
rqt_plot /power_debug/data[0]:label=a_coeff \
         /power_debug/data[1]:label=b_coeff \
         /power_debug/data[2]:label=c_coeff \
         /power_debug/data[4]:label=scaling_factor \
         /power_debug/data[6]:label=sum_cmd_squared \
         /power_debug/data[7]:label=sum_vel_squared
```

## 三、测试场景设计

### 3.1 功率限制触发测试
**目标**: 验证功率限制是否正常工作

1. 按 `u` 多次增加平移速度到最大 (5.0 m/s)
2. 按 `w` 键全速前进
3. 观察 rqt_plot 中 `data[4]` 是否 < 1.0
4. 预期: 
   - 如果 `scaling_factor < 1.0` → 限制生效 ✓
   - 如果 `scaling_factor ≈ 1.0` → 限制临界
   - 如果 `scaling_factor > 1.0` → 限制未触发（参数需调整）

### 3.2 多轮联动测试
**目标**: 验证复杂运动下的功率控制

1. 同时按 `w` + `d` (斜向运动)
2. 观察 `sum_cmd²` (data[6]) 和 `scaling_factor` (data[4])
3. 预期: 多轮同时运动时功率消耗增大，更容易触发限制

### 3.3 参数调优测试
**目标**: 找到合适的 `effort_coeff` 值

1. 修改 `config/wheel_pid_merged.yaml`:
   ```yaml
   power_limiter:
     effort_coeff: 90.0  # 从 6.0 增加到 90.0
   ```
2. 重启节点，重复测试 3.1
3. 观察是否在合理速度下触发限制

### 3.4 实时调速测试
**目标**: 验证 u/i/o/p 键功能

1. 运行中按 `u` 增加速度，观察机器人加速
2. 按 `i` 减速，观察机器人减速
3. 按 `o` 增加转速，按 `q` 测试旋转
4. 按 `p` 减小转速，再次测试旋转

## 四、故障排查

### 4.1 话题未发布
```bash
# 检查话题是否存在
rostopic list | grep power

# 检查是否有订阅者（rqt_plot 会订阅）
rostopic info /power_debug

# 确认 debug_enabled 参数
rosparam get /sentry_chassis_controller/power_limiter/debug_enabled
```

### 4.2 scaling_factor 总是 > 1
**原因**: `effort_coeff` 参数过小，导致 a 系数太小

**解决方案**:
```yaml
# 在 wheel_pid_merged.yaml 中
power_limiter:
  effort_coeff: 90.0  # 增大此值
  power_limit: 300.0   # 或减小功率限制
```

### 4.3 rqt_plot 无数据
1. 确认话题有数据: `rostopic echo /power_debug`
2. 重启 rqt_plot
3. 检查话题名称是否正确（注意 data 后的索引）

## 五、数据记录和离线分析

### 5.1 录制测试数据
```bash
# 录制所有相关话题
rosbag record -O power_test.bag \
    /power_debug \
    /cmd_vel \
    /sentry_chassis_controller/desired_wheel_states \
    /sentry_chassis_controller/applied_wheel_efforts
```

### 5.2 回放分析
```bash
# 回放 bag 文件
rosbag play power_test.bag

# 在另一个终端启动 rqt_plot 查看
rqt_plot /power_debug/data[4]
```

### 5.3 导出数据到 CSV
```bash
# 使用 rostopic 导出
rostopic echo -b power_test.bag -p /power_debug > power_data.csv
```

## 六、下一步工作

- [ ] 添加 dynamic_reconfigure 支持，可在运行时调整 `walk_vel` 和 `default_omega`
- [ ] 测试所有功能的集成效果
- [ ] 根据实际测试结果优化 `effort_coeff` 和 `power_limit` 参数
- [ ] 编写答辩演示脚本，展示功率控制效果

---

**创建日期**: 2025-12-05  
**版本**: 1.0
