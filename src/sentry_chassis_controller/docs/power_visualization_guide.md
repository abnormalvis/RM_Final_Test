# 功率可视化与标定指南

本文档汇总了功率模型的物理推导、实验标定步骤，以及答辩时的简明陈述要点，便于在调试与答辩中使用。

## 1 物理推导（精要）

我们在代码中使用的经验模型形式为：

$$P(s)=a\cdot s^2 + b\cdot s + c$$

其中通过对单轮或全车求和得到：
- 控制指令缩放：$cmd'_i = s\cdot cmd_i$。
- 机械做功项：$P_{mech}=\sum_i |T_i\cdot \omega_i|$，若扭矩 $T\propto cmd$ 则出现 $\sum |cmd_i\cdot v_i|$ 的一次项（这里 $v_i$ 代表轮端线速度）。
- 电热损耗（铜损）近似与电流平方成正比：$P_{cu}\propto I^2R$。因为 $T\propto I$，若 $cmd\propto T$，则铜损可等效表示为 $\alpha\cdot cmd^2$，这给出了二次项 $a\cdot s^2$ 的物理来源。
- 转速相关损耗（铁损、风阻、部分机械摩擦）常随转速的幂次增长，工程上常近似为 $\beta\cdot v^2$，这对应模型中的 $c$ 中的速度平方项。

因此可以将系数解释为：
- $a = k_{effort}\cdot \sum cmd_i^2$：等效放大电流/扭矩相关的平方损耗（铜损为主）；
- $b = \sum |cmd_i\cdot v_i|$：机械做功的线性耦合项；
- $c = k_{vel}\cdot \sum v_i^2 - P_{offset} - P_{limit}$：速度相关的耗散与偏置合并项（铁损、风阻等）。

注意：这是经验模型，整合了多种物理效应。真实电机损耗更加复杂（与温度、磁滞、效率曲线相关），但该模型在在线实时计算与控制中足够实用。

## 2 标定（实验步骤与回归拟合）

目标：通过实验数据拟合出可用的 $k_{effort}$ 与 $k_{vel}$，使得模型在实际工况下给出合理的 scaling_factor。

推荐步骤：

1. 数据准备：
   - 在机器人或测试台上准备若干工况组合 (cmd, speed)。建议覆盖低/中/高三档 cmd 和低/中/高 三档速度，总计 9~20 组左右。
   - 对每组工况记录：控制命令 `cmd_i`（每轮或汇总）、轮端速度 `v_i`、整车实时功率 P_meas（可以通过测量电源侧电压×电流或读取电机驱动的电流/电压计算）。
   - 使用 rosbag 或 CSV 记录以下话题/变量：`/power_debug`（代码中已发布的辅助量）、`/motor_current`（若可得）、`/odom`（速度）、时间戳。

2. 数据整理与特征构造：对每条记录计算：
   - S_cmd2 = \sum_i cmd_i^2
   - S_cmdvel = \sum_i |cmd_i\cdot v_i|
   - S_vel2 = \sum_i v_i^2

3. 线性回归拟合：在 s=1 的前提下对测得功率做拟合：

$$P_{meas} \approx A\cdot S_{cmd2} + B\cdot S_{cmdvel} + C\cdot S_{vel2} + D$$

   - 用最小二乘（numpy.linalg.lstsq 或 sklearn.linear_model.LinearRegression）求解 A,B,C,D。
   - 解释：A 对应 k_effort 的等效份额（注意尺度因子与单位），C 对应 k_vel 的等效份额，B 对应做功能量耦合。

4. 映射回控制参数：根据模型中 a,b,c 的定义，把拟合得到的 A,C 映射为 `effort_coeff` 与 `velocity_coeff` 的初始值（视实现可能需要缩放或单位转换）。

5. 验证与微调：
   - 在未参与拟合的工况下验证 scaling_factor 是否与观测的功率越界一致；
   - 使用 `rqt_reconfigure` 在线调整 `effort_coeff`、`velocity_coeff` 和 `power_limit`，并用 `rqt_plot` 观察 `/power_debug` 的字段（a,b,c,scaling_factor 等）。

简单的 Python 回归示例（离线，示意）：

```python
import numpy as np
from sklearn.linear_model import LinearRegression

# X columns: [S_cmd2, S_cmdvel, S_vel2, 1]
X = np.column_stack([S_cmd2, S_cmdvel, S_vel2, np.ones(len(S_cmd2))])
y = P_meas
model = LinearRegression(fit_intercept=False).fit(X, y)
coef = model.coef_  # [A, B, C, D]
print('A,B,C,D =', coef)
```

## 3 答辩中可用的简短陈述（2~3句）

“我们使用一个二次经验模型来近似整车功率：二次项代表与电流或扭矩平方相关的电热损耗（铜损），一次项代表扭矩与速度的做功能量，而速度平方项代表铁损、风阻等与转速相关的损耗。通过在不同 (cmd, speed) 工况下做回归拟合得到 `effort_coeff` 与 `velocity_coeff` 的初始值，然后在运行中使用 `rqt_reconfigure` 微调，保证 scaling_factor 在极端工况下能正确触发限幅。”

可选补充（如被问到具体如何量测）：
- 使用电源测量电压与电流或驱动器的电流回读结合速度编码器，按时间对齐做离线回归；验证时重点看 `scaling_factor<1` 的时刻与实际功率峰值是否一致。

---

如果你希望，我可以：
- 把示例的 Python 回归脚本放在 `tools/` 下并给出用法；或
- 直接把回归脚本修改为读取 rosbag 并输出拟合结果（需要你确认 rosbag 中的话题名）。

（文档完）
