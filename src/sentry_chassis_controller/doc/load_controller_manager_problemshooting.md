## 问题：Gazebo/ControllerManager 未正确加载控制器（sentry_chassis_controller/WheelPidController）

本文档总结了我为让 `controller_manager` 与 `WheelPidController` 在仿真中正常工作所做的排查、修改、构建步骤，以及仍需解决的事项与下一步可执行命令。

### 已完成的工作（按时间顺序）

- 在工作区中实现并单包编译 `sentry_chassis_controller`（包含 `WheelPidController` 插件）——生成库 `libsentry_chassis_controller.so`。
- 创建并添加测试 launch/参数文件：
  - `launch/sentry_pid_test.launch`（包含 `sentry_with_tf.launch`、加载 `config/wheel_pid_params.yaml` 并用 `controller_manager/spawner` 启动控制器）
  - `config/wheel_pid_params.yaml`（现在已包含 `type: "sentry_chassis_controller/WheelPidController"` 字段）
- 在 workspace 中定位到缺失的 Gazebo RobotHW 插件源码：`/home/idris/final_ws/src/rm_control/rm_gazebo`，确认 `src/rm_robot_hw_sim.cpp` 中有 `PLUGINLIB_EXPORT_CLASS` 并且 `CMakeLists.txt` 会生成 `librm_robot_hw_sim.so`。
- 运行 `catkin build rm_common`：发现构建失败是因为系统缺少第三方 IMU filter 包（如 `imu_complementary_filter`）；为降低阻塞，我对 `rm_common/CMakeLists.txt` 做了小改动：使 IMU 相关源码在找不到对应包时被跳过（通过 `find_package(... QUIET)` + 有条件地加入 `src/filter/*.cpp`）。随后 `rm_common` 构建成功。
- 运行 `catkin build rm_gazebo --no-deps -v`：构建成功，已生成并 symlink 出 `librm_robot_hw_sim.so`（位置参考：`devel/.private/rm_gazebo/lib/librm_robot_hw_sim.so`，并在 `devel/lib` 有 symlink）。

### 现场复现（我执行的命令与关键输出）

- 启动测试 launch：
  ```bash
  source devel/setup.bash
  roslaunch sentry_chassis_controller sentry_pid_test.launch use_simulation:=true
  ```

- 关键日志（节选）：
  - Gazebo 启动并加载了 `gazebo_ros_control` 插件。
  - spawn_urdf 成功将模型放入场景。
  - controller spawner 报错：
    "Could not load controller 'wheel_pid_controller' because the type was not specified. Did you load the controller configuration on the parameter server (namespace: '/wheel_pid_controller')?"
  - 最后出现："Failed to load sentry_chassis_controller/WheelPidController"

  （完整运行日志已保存为 `doc/test_pid_log.txt`）

### 我为解决该问题所做的直接修复尝试

- 在 `config/wheel_pid_params.yaml` 中加入 `type: "sentry_chassis_controller/WheelPidController"`，并在 `sentry_pid_test.launch` 中通过 `<rosparam file=... command="load"/>` 先加载参数，再调用 `controller_spawner`。这可以让 `controller_spawner` 从参数服务器读取控制器类型（或直接在 spawner args 中给出 type）。

### 当前已解决的依赖/构建问题

- `rm_common`：已修改 CMakeLists 以在缺少 IMU filter 包时继续构建；`catkin build rm_common` 成功。
- `rm_gazebo`：已成功构建并生成 `librm_robot_hw_sim.so`。

构建产物快速检查路径（已在工作区生成）：

- `devel/.private/rm_gazebo/lib/librm_robot_hw_sim.so`
- `devel/lib/librm_robot_hw_sim.so`（symlink）
- `devel/.private/rm_common/lib/librm_common.so`（以及相应的 symlink）
- `devel/.private/sentry_chassis_controller/lib/libsentry_chassis_controller.so` 或 `devel/lib/libsentry_chassis_controller.so`（请用下面的命令确认）

### 尚未完全确认 / 需要进一步验证的问题

1. controller type 是否最终正确出现在参数服务器上的命名空间下（例如 `/wheel_pid_controller/type` 或 `/sentry_chassis_controller/WheelPidController/type`，取决于你的 launch/参数组织方式）。如果 `controller_spawner` 启动时看不到该参数，会报“type not specified”。
2. pluginlib 能否找到并加载 `libsentry_chassis_controller.so`：
   - 需要确认 `sentry_chassis_controller_plugins.xml` 中 `library` 的 `path` 是否与实际生成位置匹配（通常是 `lib/libsentry_chassis_controller`）；`devel` 下的 `lib` 通常被 pluginlib/ros 包自动搜索到，但如果 launch 在不同的 env（未 source devel/setup.bash）下运行，会找不到。
3. `controller_spawner` 与 `controller_manager` 的服务是否已经就绪（日志中看到 `wait_for_service(/controller_manager/load_controller)` 在等待，可能是 `gazebo_ros_control` 未能把 RobotHW 正确注册到 ROS 上或 URDF 插件未加载顺序问题）。

### 针对上述问题的检查命令（可复制执行）

1) 确认参数服务器上是否有 type：

```bash
source /home/idris/final_ws/devel/setup.bash
rosparam get /wheel_pid_controller
rosparam get /wheel_pid_controller/type
rosparam get /sentry_chassis_controller/WheelPidController
```

2) 列出生成的库，确认文件存在：

```bash
ls -l /home/idris/final_ws/devel/.private/rm_gazebo/lib/librm_robot_hw_sim.so
ls -l /home/idris/final_ws/devel/lib/libsentry_chassis_controller.so || ls -l /home/idris/final_ws/devel/.private/sentry_chassis_controller/lib/libsentry_chassis_controller.so
```

3) 检查 plugin XML 与导出是否匹配：

```bash
sed -n '1,160p' $(rospack find sentry_chassis_controller)/sentry_chassis_controller_plugins.xml
```

4) 在 launch 运行时，监控 controller_manager 服务：

```bash
rosservice list | grep controller_manager
rosservice call /controller_manager/list_controllers
```

5) 重新启动 launch（保证已经 source devel/setup.bash），并保存完整日志：

```bash
source /home/idris/final_ws/devel/setup.bash
roslaunch sentry_chassis_controller sentry_pid_test.launch use_simulation:=true 2>&1 | tee /home/idris/final_ws/src/sentry_chassis_controller/doc/test_pid_run_full.log
```

### 下一步推荐的工作

- A 
- 1. 运行上面的参数与文件存在性检查命令（第 1、2、3 步），把输出贴回给你。这样能快速定位是参数没加载，还是 plugin 库或 plugin XML 问题。
- 2. 在确认完这些后，重新以可捕获日志的方式启动 `sentry_pid_test.launch` 并把完整日志发回（当前已有一份摘录 `doc/test_pid_log.txt`）。
- B. 按上面的命令检查 `rosparam get /wheel_pid_controller/type` 与库路径，然后重新运行 launch。
---
