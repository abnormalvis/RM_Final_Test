## 问题描述

在使用 `sentry_chassis_controller/launch/sentry_with_tf.launch` 启动 Gazebo + RViz 时，能够看到机器人模型和 /map -> /base_link 的静态 TF，但无法看到基于关节状态的 link-to-link TF（例如轮子、转向器等）。

经排查，`robot_state_publisher` 已启动并订阅 `/joint_states`，但没有任何节点在发布 `/joint_states`，导致 robot_state_publisher 没有输入数据来生成基于关节的 TF。

## 诊断步骤（复现我做过的检查）

1. 确认正在运行的 topic：
	- rostopic list
	- rostopic info /joint_states
	结果显示 `/joint_states` 存在但没有 Publishers（Publishers: None）。

2. 采样 `/joint_states`（确认是否有消息）：
	- rostopic echo -n1 /joint_states
	结果：在模拟时间下没有接收到消息（说明无人发布 joint_states）。

3. 检查 `robot_state_publisher` 节点：
	- rosnode list
	- rosnode info /robot_state_publisher
	结果：节点存在并订阅 `/joint_states`，会发布 `/tf` 与 `/tf_static`，但在无人发布 joint_states 时不会生成 link-to-link 的动态 TF。

4. 查看 robot_description（URDF）是否包含 plugin 插件：
	- rosparam get /robot_description | head -n 50
	结果：URDF 中有一段 gazebo plugin 配置，使用了 `librm_robot_hw_sim.so` 之类的库（由 `use_simulation` 参数控制注入）。

5. 查看 Gazebo 日志、spawn 报错信息：
	- 注意到曾出现 `Failed to load plugin librm_robot_hw_sim.so` 的错误（表示仿真插件缺失），并且 spawn_model 有时会报 `model already exist`，提示重复 spawn 或未清理之前的实体。

## 解决过程与修改（步骤详述）

为保证尽快恢复模型显示和完整 TF，我们采取了两条并行策略：先让系统能看见 TF（临时方案），再修复仿真插件以获取真实 joint_states（长期方案）。以下为实际改动与理由：

1. 修复 CMakeLists.txt（与编译相关）
	- 问题：`CMakeLists.txt` 中错误地把 `boost` 与 `eigen` 当作 catkin 的组件写入 `find_package(catkin REQUIRED COMPONENTS ...)`，导致 catkin/cmake 在查找包时失败。
	- 修改：把 `boost` / `eigen` 从 catkin 组件中移除，改为使用 CMake 的 `find_package(Boost REQUIRED)` 和 `find_package(Eigen3 REQUIRED)`，并将 `${Boost_INCLUDE_DIRS}` 与 `${EIGEN3_INCLUDE_DIRS}` 加入 `include_directories`。
	- 结果：修复后可以成功 `catkin build sentry_chassis_controller`。

2. 临时避免加载缺失的 Gazebo plugin
	- 问题：URDF 的 xacro 在 `use_simulation:=true` 时会注入 `<gazebo><plugin filename="librm_robot_hw_sim.so">...</plugin></gazebo>`，但工作区中没有该共享库，Gazebo 报错并无法加载插件。
	- 修改：在 launch 调用 xacro 时将 `use_simulation` 设为 `false`（即 `use_simulation:=false`），保证 URDF 加载但不注入该插件，从而避免插件加载错误并能正常 spawn 模型。
	- 结果：避免了 Gazebo 插件相关的加载失败，模型能被正确加载到 Gazebo / RViz。

3. 参考 `sentry_sim/sentry_gazebo/launch/gazebo_rmuc.launch` 重构 `sentry_with_tf.launch`
	- 加入命令行参数：`paused`, `use_sim_time`, `gui`, `verbose`, `x_pos`, `y_pos`, `z_pos`, `R_pos`, `P_pos`, `Y_pos`, `is_open_rviz` 等。
	- 使用可配置的 `spawn_model` args，将机器人的初始位姿通过参数传递。
	- 添加 `robot_state_publisher` 节点并设置 `publish_frequency`。

4. 为保证在没有真实 joint_states 的情况下仍能看到 link-to-link TF，添加可选 `joint_state_publisher`（默认启用）
	- 在 `sentry_with_tf.launch` 中新增参数 `use_joint_state_publisher`（默认 `true`），并添加节点：
	  ```xml
	  <node if="$(arg use_joint_state_publisher)" pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" output="screen"/>
	  ```
	- 目的：当没有其它 `/joint_states` 发布者时，`joint_state_publisher` 会发布默认（静态或 GUI 可调）的 joint states，`robot_state_publisher` 接收后会生成 link-to-link TF，从而在 RViz 中显示完整 TF 树。
	- 结果：启动修改后的 launch 后 `/joint_states` 出现发布者 `/joint_state_publisher`，并能在 /tf 中看到由 `robot_state_publisher` 生成的 link-to-link transforms。注意：这些 joint_states 默认各位置为 0（静态），不是控制器或插件产生的动态数据。

5. 处理重复节点、spawn 冲突
	- 在多次重启 launch 或未清理 Gazebo 实体时，会出现 `new node registered with same name` 或 `model name sentry already exist` 等警告/错误。建议在重启时先整洁地关闭上一次的 launch，或者在 spawn 前删除旧实体/换 model 名，避免冲突。

## 可选（长期）解决方案：恢复真实的 joint_states 发布

如果希望在 Gazebo 仿真中获得真实、动态的 joint_states（由控制器或仿真插件产生），需要：

1. 找到并编译产生 `librm_robot_hw_sim.so` 的包（或其它对应的仿真插件），将其安装到系统可被 Gazebo 加载的路径（或把路径加入 `GAZEBO_PLUGIN_PATH` / `LD_LIBRARY_PATH`）。常见做法：
	- 在 workspace 中找到含有该插件的包（例如名为 `rm_gazebo` / `rm_hw_sim` 等），并 `catkin build` 编译；确保生成的 .so 位于 `devel` 或 `install` 可访问路径。
	- 若插件不在工作区，需要从相应仓库 clone 并编译或安装已发布的二进制包。

2. 把 `sentry_with_tf.launch` 中调用 xacro 的 `use_simulation` 恢复为 `true`：
	```xml
	<param name="robot_description" command="$(find xacro)/xacro ... use_simulation:=true ..."/>
	```
	这样 URDF 会注入 `<gazebo>` plugin，Gazebo 在加载模型时将加载插件并在仿真中驱动/发布 joint_states（前提是插件实现了该功能）。

3. 或者让真实控制器/仿真节点直接发布 `/joint_states`，并确保 topic 命名与 `robot_state_publisher` 订阅一致（或通过 launch remap 将其 remap 到 `/joint_states`）。

## 如何回滚临时改动

- 若要回退到原始行为（由插件驱动 joint_states）：把 `sentry_with_tf.launch` 中 xacro 的 `use_simulation` 改回 `true`，并删除（或设置 `use_joint_state_publisher:=false`）临时加入的 `joint_state_publisher`。
- 若要恢复插件方式，请先确保插件库存在并能被 Gazebo 加载，然后再启用 `use_simulation`。

## 快速检查清单（启动后按序执行）

1. source 工作区环境：
	```bash
	source /home/idris/final_ws/devel/setup.bash
	```
2. 启动 launch（示例）：
	```bash
	roslaunch sentry_chassis_controller sentry_with_tf.launch
	```
3. 检查 `/joint_states`：
	```bash
	rostopic info /joint_states
	rostopic echo -n1 /joint_states
	```
4. 检查 TF：
	```bash
	rosrun tf tf_monitor
	rosrun tf tf_echo map base_link
	```

## 总结

问题的核心在于没有 `/joint_states` 的发布者。为快速恢复可视化，我临时添加并默认启用了 `joint_state_publisher`，并修改 URDF xacro 以避免加载缺失的 Gazebo plugin。长期方案是定位并编译缺失的仿真插件或让真实控制器发布 `/joint_states`，以获得动态的、真实的关节状态和相应的 TF 发布。


