# catkin-tools 构建问题汇总

日期：2025-11-22

摘要
-----
在本工作区尝试使用 `catkin_tools` 构建 ROS 包时执行的操作、输出和当前遗留问题的汇总。目标是构建 `src/` 下的所有包并记录遇到的依赖/构建错误以便后续修复。

已执行的关键步骤（按顺序）
--------------------------------
1. 检查系统和工具
	- 发现系统已安装 ROS Noetic（/opt/ros/noetic）。
	- 确认存在 `/usr/bin/catkin`，并通过 apt 安装了 `python3-catkin-tools`（catkin_tools）。

2. 安装并更新 rosdep
	- 已安装 `python3-rosdep`。
	- 运行 `rosdep update`：初次尝试出现 GitHub 超时，后续重试成功并更新缓存（~/.ros/rosdep/sources.cache）。

3. 运行 rosdep install（为 workspace 安装系统依赖）
	- 命令：
	  rosdep install --from-paths src --ignore-src -r -y
	- 结果：大部分可解析的系统依赖被自动安装成功。
	- 未解析的 rosdep keys（列出部分）：`effort_controllers`、`rm_common`。这些项没有对应的 rosdep 规则（或不是系统包名），因此 rosdep 报告无法解析。

4. 构建包（单包与全量）
	- 注意到 `src/rm_description_for_task/package.xml` 中的 `<name>` 为 `rm_description`，使用正确包名进行了构建：
	  catkin build rm_description
	  构建成功，rospack 能找到包：`rospack find rm_description` -> `/home/idris/final_ws/src/rm_description_for_task`。

	- 然后尝试构建整个 workspace：
	  catkin build
	- 构建结果：部分包构建成功（如 `mimic_joint_controller`、`rm_description`），但多个包失败或被放弃（摘要：2 成功，7 失败，3 放弃）。

核心错误与原因
------------------
- 未解析的 rosdep keys：
  - `effort_controllers`：rosdep 中没有直接规则，但存在 Debian/ROS 系统包 `ros-noetic-effort-controllers`，已通过 apt 安装解决该依赖。
  - `rm_common`：没有对应的系统包（`rospack find rm_common` 报错），构建时 CMake 找不到 `rm_commonConfig.cmake`，因此多个依赖它的包（gpio_controller、rm_gimbal_controllers、rm_chassis_controllers、robot_state_controller 等）配置阶段失败。

- CMake 的典型报错（示例）：
  Could NOT find rm_common (missing: rm_common_DIR)
  Could not find a package configuration file provided by "rm_common" with any of the following names:
	 rm_commonConfig.cmake
	 rm_common-config.cmake

已做的临时修复/处理
---------------------
- 为 `effort_controllers` 安装了系统包：
  sudo apt install -y ros-noetic-effort-controllers
  这解决了与 effort_controllers 相关的编译/头文件缺失问题。

仍需处理的问题（建议优先级）
--------------------------------
1. 提供或添加 `rm_common` 包
	- 说明：多数失败包依赖 `rm_common`，这是一个工作区内或外的共享库/工具包（包含头文件如 `rm_common/ros_utilities.h` 等）。
	- 方案 A（推荐）：如果 `rm_common` 的源码存在于其他仓库，请把其源码目录放入当前 workspace 的 `src/` 下，然后重新运行 `rosdep install`（或直接 `catkin build`）。
	- 方案 B：如果 `rm_common` 以二进制 ROS 包发布（apt），可以查找并安装：
		 apt search ros-noetic-rm-common  # 若存在则安装
	- 方案 C：如果 `rm_common` 为私有/组织内部包且没有发布，需从原始仓库获取源码并加入 workspace。

2. 为工作区补全 rosdep 规则（可选）
	- 如果有自定义依赖名，需要添加自定义 rosdep 源（`/etc/ros/rosdep/sources.list.d/` 或用户级别），但通常更简单的是将缺失的包源码放入 workspace。

下一步建议（操作顺序）
-----------------------
1. 在本 workspace 中确认是否存在 `rm_common` 的源码：
	- 如果没有，请从项目来源获取 `rm_common` 源码并放到 `src/` 下，或把该源码子模块/仓库克隆进来。
2. 重新运行：
	```bash
	cd /home/idris/final_ws
	source /opt/ros/noetic/setup.bash
	rosdep install --from-paths src --ignore-src -r -y
	catkin build
	```
3. 如果需要，我可以：
	- 帮你在 workspace 中搜索或添加 `rm_common`（如果能定位到对应仓库地址，请提供或允许我尝试查找），
	- 或者只构建不依赖 `rm_common` 的包（通过 `catkin build <pkg> --no-deps`）以便先验证其他部分。

构建中出现的命令（记录，便于复现）
---------------------------------
- sudo apt update && sudo apt install -y python3-catkin-tools python3-rosdep
- sudo rosdep init || true; rosdep update
- rosdep install --from-paths src --ignore-src -r -y
- source /opt/ros/noetic/setup.bash
- catkin build rm_description --verbose
- sudo apt install -y ros-noetic-effort-controllers
- catkin build --verbose
