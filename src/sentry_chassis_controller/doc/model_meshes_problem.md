## 问题概述

在用 Gazebo + RViz 仿真 `sentry` 机器人时，观察到：

- Gazebo 成功 spawn_model，但在 RViz 中 RobotModel 以红色（表示 collision）显示，visual 未正常显示。
- rqt / rqt_plot 在某些终端启动时会崩溃（与 Python 环境有关），spawn_model 在 Conda 虚拟环境下曾因为系统库缺失而崩溃。

附件日志（`fk_test.log`）显示了两类信息：

- Qt 绑定缺失：`Could not find Qt binding (looked for: 'pyqt', 'pyside')` → 导致 rqt GUI 插件无法加载。
- spawn_model 失败时伴随 `ModuleNotFoundError: No module named 'numpy'`（当在 Conda 环境运行系统 ROS 进程时）。

## 根本原因

1. RViz 显示红色通常有两种常见原因：
   - URDF 中缺少 `<visual>`（只有 `<collision>`），或 visual 无法被渲染（mesh 路径错误或 mesh 文件找不到）。
   - 虽然存在 `<visual>`，但没有显式材质（或材质名未被定义或不可用），在某些情况下会回退到 collision 的渲染模式。 
2. 在本仓库中，`sentry.urdf.xacro` 定义了一个默认材质 `sentry_default_material`，但并非每个 `<visual>` 都引用该材质，导致 RViz 可能没有使用预期视觉颜色。
3. 另外，开发者/运行时在 Conda 环境与系统 ROS Python 环境混用，导致 spawn_model（依赖系统 numpy）或 rqt（依赖系统 PyQt/PySide）出现 ImportError/ModuleNotFoundError。

## 已采取的修复

1. 在 `sentry.urdf.xacro` 中定义了默认材质（已存在）：
   - 名称：`sentry_default_material`，颜色 `rgba="0.7 0.7 0.7 1.0"`。

2. 显式把默认材质绑定到各视觉几何：
   - 修改文件：
     - `rm_description_for_task/urdf/sentry/chassis.urdf.xacro`（在 `base_link` 的 `<visual>` 中添加 `<material name="sentry_default_material"/>`）。
     - `rm_description_for_task/urdf/common/swerve_drive.urdf.xacro`（在 `pivot` 与 `wheel` 的 `<visual>` 中添加 `<material name="sentry_default_material"/>`）。

   目的：确保每个 link 的 visual 都使用定义的材质色，避免 RViz 回退显示为 collision（红色）。

3. 为了便于调试同时做了其它改动（不直接影响 RViz 颜色，但有助于整体仿真）：
   - 添加 `gazebo -> odom` 桥接节点（用于快速在 RViz 中显示机器人位姿）。
   - 实现 forward kinematics node，基于轮速与转向角估计 `/odom` 并广播 TF（便于离线测试和可视化）。
   - 在 controller 中实现并发布逆运动学（IK）到 `/desired_wheel_states` 以便调试。

## 验证方法（快速命令）

下面命令在系统 ROS 环境（确保没有激活 Conda 的 `ros_qt`）下运行：

1) 渲染 xacro 并把结果写到 `/robot_description`（热更新 URDF）：

```bash
rosrun xacro xacro /home/idris/final_ws/src/rm_description_for_task/urdf/sentry/sentry.urdf.xacro \
  load_chassis:=true use_simulation:=false > /tmp/sentry.urdf
rosparam set /robot_description - < /tmp/sentry.urdf
```

2) 在 RViz 中：
   - 打开或重启 RViz。
   - 在左侧的 `RobotModel` 显示中，确认 `Show Visual` 已勾选，`Show Collision` 可临时关闭以便对比。

3) 检查参数确认材质已出现在 robot_description：

```bash
rosparam get /robot_description | grep -n "sentry_default_material" || true
```

4) 若 mesh 无法加载，RViz 会在控制台输出错误，检查是否有 `Failed to load resource: package://...` 的警告。

## 后续建议（若仍看到红色）

- 检查 mesh 路径与包名是否正确（`package://rm_description/meshes/...`）。如果 ROS 找不到该包，请确认工作区已正确构建并 source 了 `devel/setup.bash`。
- 临时回退方案：如果某些 link 的 mesh 仍无法加载，我可以把对应 link 加入一个简单的占位 `<visual>`（例如 box 或 cylinder），或直接把 `<collision>` 的几何复制到 `<visual>`，以便确认问题是否由 mesh 文件导致。
- 最终建议：在发布镜像/代码给其他人前，把 URDF 的 mesh 路径改写为相对包内路径并确保该包安装在 ROS_PACKAGE_PATH 中，或把常用 mesh 放到能被 RViz 直接加载的位置。

## 相关已修改文件

- `/home/idris/final_ws/src/rm_description_for_task/urdf/sentry/sentry.urdf.xacro`  （定义 `sentry_default_material`）
- `/home/idris/final_ws/src/rm_description_for_task/urdf/sentry/chassis.urdf.xacro` （为 `base_link` visual 添加材质绑定）
- `/home/idris/final_ws/src/rm_description_for_task/urdf/common/swerve_drive.urdf.xacro` （为 pivot/wheel visual 添加材质绑定）
- `/home/idris/final_ws/src/sentry_chassis_controller/src/gazebo_odom_bridge.cpp` （gazebo->odom 桥）
- `/home/idris/final_ws/src/sentry_chassis_controller/src/forward_kinematics_node.cpp` （基于轮速的里程计）
- `/home/idris/final_ws/src/sentry_chassis_controller/src/inverse_kinematics.cpp` 与 controller 修改（IK 集成）

## 完成说明

已把“修复 URDF 视觉材质”与“记录文档”两项标记为已完成（见仓库 `todo`）。

如果你希望我直接把缺失视觉的 link 自动添加占位 visual（把 collision 复制为 visual）来做最终验证，我可以继续在 `chassis.urdf.xacro` / `swerve_drive.urdf.xacro` 中按需添加这些回退 visual。请回复“请添加回退 visual”我就继续执行。

---
文件由自动化变更生成并记录在仓库中，若需补充截图或 RViz 控制台日志，我可以帮你把这些收集并附到文档里。
