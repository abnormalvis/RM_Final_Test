# Dynamic Reconfigure 使用指南 - 功率参数实时调节

## 功能概述

现在可以通过 `rqt_reconfigure` 实时调整功率限制参数，无需重启节点或修改配置文件。

### 新增的可调参数

| 参数名 | 说明 | 默认值 | 范围 | 推荐调整方向 |
|--------|------|--------|------|-------------|
| `effort_coeff` | 控制力矩系数 | 6.0 | 0.0 - 200.0 | **增加到 90-100 以触发限制** |
| `velocity_coeff` | 速度系数 | 0.0048 | 0.0 - 1.0 | 通常保持不变 |
| `power_limit` | 功率限制 (W) | 300.0 | 0.0 - 1000.0 | 根据实际硬件调整 |
| `power_offset` | 功率偏移 (W) | 0.0 | 0.0 - 500.0 | 补偿静态功耗 |

## 使用方法

### 1. 启动系统

```bash
# 终端1: 启动仿真
roslaunch sentry_sim sentry_sim.launch

# 终端2: 启动键盘控制
rosrun sentry_chassis_controller sentry_control_key_feature

# 终端3: 启动功率监控
rqt_plot /power_debug/data[4]:label=scaling_factor \
         /power_debug/data[5]:label=F_1 \
         /power_debug/data[10]:label=limited

# 终端4: 启动参数调节工具
rqt_reconfigure
```

### 2. 在 rqt_reconfigure 中调整参数

1. 在左侧面板找到 `/sentry_chassis_controller` 节点
2. 展开参数列表，找到新增的功率参数：
   - `effort_coeff`
   - `velocity_coeff`
   - `power_limit`
   - `power_offset`
3. 拖动滑块或输入数值实时调整

### 3. 实时观察效果

调整参数后，立即在 rqt_plot 中观察：
- `data[4]` (scaling_factor) 的变化
- `data[5]` (F(1)) 的变化
- `data[10]` (limited 标志) 是否从 0 变为 1

## 典型调试场景

### 场景 1: 当前功率限制未触发 (scaling_factor ≈ 3.96)

**目标**: 使功率限制在合理速度下触发

**步骤**:
1. 打开 rqt_reconfigure
2. 将 `effort_coeff` 从 6.0 逐步增加：
   - 先调到 20 → 观察 scaling_factor
   - 再调到 50 → 观察 scaling_factor
   - 继续调到 90-100 → 应该看到 scaling_factor < 1
3. 在键盘控制中按 `w` 全速前进
4. 观察 rqt_plot 中 `scaling_factor` 是否降到 < 1.0

**预期结果**:
- `effort_coeff = 90` 时，大控制指令下 `scaling_factor` 应接近或小于 1.0
- `data[10]` (limited) 变为 1.0，表示触发功率限制

### 场景 2: 功率限制过于严格

**症状**: scaling_factor 总是远小于 1，机器人响应缓慢

**解决方案**:
1. 在 rqt_reconfigure 中降低 `effort_coeff`
2. 或增加 `power_limit` 值
3. 观察 scaling_factor 逐渐接近 1.0

### 场景 3: 参数标定

**目标**: 找到最优的 effort_coeff 值

**方法**:
1. 设置键盘控制速度到中等水平（按 `u` 几次）
2. 全速运动（按 `w` 或对角线 `w+d`）
3. 在 rqt_reconfigure 中调整 `effort_coeff`：
   ```
   scaling_factor > 1.5  → 增加 effort_coeff
   scaling_factor ≈ 1.0  → 理想状态 ✓
   scaling_factor < 0.8  → 减小 effort_coeff
   ```
4. 目标: 在最大期望速度下，scaling_factor ≈ 0.9-1.0

## 参数计算公式

理解这些公式有助于调参：

```
a = effort_coeff × Σ(cmd_i²)
b = Σ|cmd_i × vel_i|
c = velocity_coeff × Σ(vel_i²) - power_offset - power_limit

F(s) = a×s² + b×s + c ≤ 0
scaling_factor = (-b + √(b² - 4ac)) / (2a)
```

**关键洞察**:
- `effort_coeff` 越大 → a 越大 → scaling_factor 越小 → 限制越强
- `power_limit` 越小 → c 越负 → scaling_factor 越小 → 限制越强
- F(1) < 0 表示在 s=1 时功率超限，需要缩放

## 实时调参技巧

### 技巧 1: 二分法寻找临界值
```
1. 设置 effort_coeff = 50
2. 如果 scaling_factor > 1.5，增加到 100
3. 如果 scaling_factor < 0.5，减小到 25
4. 重复直到找到 scaling_factor ≈ 0.9-1.0 的值
```

### 技巧 2: 对比测试
```
1. 记录当前参数和 scaling_factor
2. 修改一个参数
3. 重复相同的运动（如按 w 5秒）
4. 对比 scaling_factor 的变化
5. 选择最优值
```

### 技巧 3: 记录优化结果
```bash
# 保存当前参数到文件
rosparam dump power_params_optimized.yaml /sentry_chassis_controller

# 下次可以直接加载
rosparam load power_params_optimized.yaml /sentry_chassis_controller
```

## 与键盘控制的配合使用

### 完整工作流

```
1. 启动所有节点（仿真 + 键盘 + rqt_plot + rqt_reconfigure）

2. 调整运动速度（键盘）:
   - 按 u/i 调节平移速度
   - 按 o/p 调节角速度

3. 调整功率参数（rqt_reconfigure）:
   - 调节 effort_coeff
   - 调节 power_limit

4. 实时观察（rqt_plot）:
   - 观察 scaling_factor
   - 观察 F(1)
   - 观察 limited 标志

5. 迭代优化:
   - 找到合适的参数组合
   - 验证在不同速度下都表现良好
```

## 常见问题

### Q1: 修改参数后没有立即生效？

**A**: 检查终端输出，应该看到：
```
WheelPidController: Power limiter updated - effort_coeff=90.00, ...
```

如果没有，尝试：
1. 确认 rqt_reconfigure 连接到正确的节点
2. 检查参数值是否真的改变了
3. 重启 rqt_reconfigure

### Q2: scaling_factor 一直是 1.0 不变？

**A**: 可能原因：
1. `effort_coeff` 太小（需要增大）
2. 控制指令太小（按 `u` 增加速度，再按 `w` 移动）
3. `power_limit` 太大（减小试试）

### Q3: 如何保存调试好的参数？

**A**: 两种方法：

方法1 - 更新 YAML 配置文件：
```yaml
# 编辑 config/wheel_pid_merged.yaml
power_limiter:
  effort_coeff: 90.0      # 更新为调试好的值
  velocity_coeff: 0.0048
  power_limit: 300.0
  power_offset: 0.0
```

方法2 - 导出当前参数：
```bash
rosparam dump optimized_params.yaml /sentry_chassis_controller
```

## 可视化最佳实践

### 推荐的 rqt_plot 配置

```bash
# 核心指标（3条曲线）
rqt_plot /power_debug/data[4]:label=scaling \
         /power_debug/data[5]:label=F_1 \
         /power_debug/data[10]:label=limited

# 完整分析（6条曲线）
rqt_plot /power_debug/data[0]:label=a_coeff \
         /power_debug/data[1]:label=b_coeff \
         /power_debug/data[2]:label=c_coeff \
         /power_debug/data[4]:label=scaling \
         /power_debug/data[5]:label=F_1 \
         /power_debug/data[10]:label=limited
```

### 推荐的窗口布局

```
+-------------------+-------------------+
|   rqt_plot        |  rqt_reconfigure  |
|   (功率监控)       |   (参数调节)       |
+-------------------+-------------------+
|   终端 (键盘控制)   |   RViz (可选)     |
+-------------------+-------------------+
```

## 下一步

完成参数调试后：
1. 更新配置文件 `config/wheel_pid_merged.yaml`
2. 在不同运动模式下验证
3. 记录最优参数和测试结果
4. 准备答辩演示

---

**创建日期**: 2025-12-05  
**状态**: ✅ 已实现并测试
