(# rqt_plot 与 Gazebo spawn_model 故障排查指南)

## 概要
本指南汇总了在启动 `sentry_pid_test_fixed.launch` 时遇到的常见问题、成因分析与逐步修复命令。典型症状包括：
- `spawn_model` 进程因为 `ModuleNotFoundError: No module named 'numpy'` 崩溃，导致 Gazebo 中看不到机器人模型；
- `rqt_plot` 或其它 rqt 插件报 `ModuleNotFoundError: No module named 'numpy'` 或 `ValueError: PyCapsule_GetPointer called with incorrect name`（PyQt/SIP 二进制不兼容）。

根本原因通常是：在 Conda/virtualenv 等虚拟环境中运行 `roslaunch`，导致 Python 运行时优先加载虚拟环境里的包（或缺少 numpy），而 ROS 的 GUI 插件和 /opt/ros/noetic 下的二进制扩展期望使用系统 Python（/usr/bin/python3）和 apt 安装的 PyQt/SIP/NumPy。虚拟环境可能造成模块缺失或 ABI 不兼容。

## 快速检查（先做）
在启动任何修复之前，请在终端执行以下检查，确认是否处于虚拟环境并检查 python 路径：

```bash
# 是否处于 virtualenv/conda
echo "VIRTUAL_ENV=$VIRTUAL_ENV"
echo "CONDA_PREFIX=$CONDA_PREFIX"

# 当前 python 可执行文件和版本
python3 -c "import sys; print('exe=', sys.executable); print('ver=', sys.version)"

# numpy 是否可用
python3 -c "import importlib
try:
	m=importlib.import_module('numpy'); print('numpy ok', m.__file__, m.__version__)
except Exception as e:
	print('numpy import error:', e)"
```

预期：如果你看到 `CONDA_PREFIX` 指向 `miniconda3/...` 或 `VIRTUAL_ENV` 非空，则表示当前激活了虚拟环境；`python3` 的 exe 应该为 `/usr/bin/python3` 才是系统 Python。若 `numpy import error` 出现，需为系统 Python 安装 numpy。

## 逐步修复流程（推荐顺序）

1) 退出 virtualenv/conda（最简单且推荐）

```bash
conda deactivate     # 如果使用 conda
# 或者
deactivate           # 如果使用 virtualenv

# 验证 python 指向系统解释器
python3 -c "import sys; print('exe=', sys.executable)"
```

预期：`exe=` 应返回 `/usr/bin/python3`（或系统默认路径，而非 miniconda 环境）。

2) 为系统 Python 安装 NumPy（优先 apt）

```bash
sudo apt update
sudo apt install -y python3-numpy

# 验证
python3 -c "import numpy; print('numpy', numpy.__version__)"
```

说明：APT 安装能够保证与 ROS 系统包的二进制兼容性；若无法使用 sudo，可临时使用 `pip3 install --user numpy`，但这不保证与系统 PyQt/SIP 的兼容性。

3) 若出现 PyCapsule / SIP / PyQt 相关错误（ABI 不兼容），确保使用系统 PyQt 与 sip

```bash
sudo apt install -y python3-pyqt5 python3-pyqt5.qtsvg python3-sip
```

说明：当错误为 `ValueError: PyCapsule_GetPointer called with incorrect name` 时，通常表示某个 Python 扩展（由 sip 生成的绑定）是用与当前 Python 运行时不兼容的 sip/pyqt 编译的。最可靠的办法是不使用 conda 运行 ROS GUI 工具，或确保 conda 中安装的 pyqt/sip 与系统二进制兼容（不推荐，极易出错）。

4) 在未激活虚拟环境的终端重新运行 launch

```bash
# 确认未激活 conda/virtualenv，再运行
roslaunch /home/idris/final_ws/src/sentry_chassis_controller/launch/sentry_pid_test_fixed.launch
```

预期结果：
- 在日志中不再看到 `ModuleNotFoundError: No module named 'numpy'`；
- `spawn_urdf`（spawn_model）进程不再崩溃，Gazebo 中应该能看到 `sentry` 机器人模型；
- rqt/rqt_plot 插件能正常加载，不再报缺少 numpy 或 PyCapsule 错误。

## 验证 spawn 与 controller 状态

成功 spawn 后，可用下列命令进一步确认模型与控制器状态：

```bash
# 检查 gazebo 中的模型状态
rostopic echo -n1 /gazebo/model_states

# 检查 controller_manager 服务/节点
rosnode list | grep controller_manager
rosservice list | grep controller_manager

# 如果需要查看某个 controller 是否已启动
rosservice call /controller_manager/list_controllers
```

如果你仍看到 `Controller Spawner couldn't find the expected controller_manager ROS interface.`，先确认 `spawn_urdf` 未崩溃并且 `robot_state_publisher`、`controller_manager` 等节点已被正常启动。

## 无 sudo / 无法修改系统环境的替代方案

- 方案 A：在全新未激活 conda 的终端运行 ROS（最简单）。
- 方案 B（高级）：为 conda 环境安装与系统二进制兼容的 PyQt/SIP/Numpy，这通常需要用 conda-forge 精确匹配版本并且仍有可能失败。只在非常必要且有经验时尝试。
- 方案 C：使用官方 ROS Docker 镜像（noetic）在容器内运行你的仿真，容器内使用系统包，避免宿主机 python 环境冲突。

## 收集错误日志（用于进一步诊断）

如果问题未解决，请收集并贴出以下信息：

1. 完整的 `spawn_urdf` 崩溃堆栈（在 `~/.ros/log/.../spawn_urdf-*.log`）；
2. 运行 `python3 -c "import sys, importlib
for mod in ['sip','PyQt5','numpy','qt_gui_cpp']:
	try:
		m=importlib.import_module(mod); print(mod, '->', getattr(m,'__file__', 'built-in'))
	except Exception as e:
		print(mod, 'import error ->', e)"`
3. `roslaunch` 的关键日志片段（spawn 崩溃前后 20-50 行）。

把上面输出贴过来后，可以更精准地判断是否为 ABI 冲突或包缺失，并给出下一步修复建议。

## 结语 / 小贴士

- 任何时候运行含 GUI 的 ROS 节点（rviz, rqt, gazebo spawn_model 的 python 脚本等）时，优先使用系统 Python（不激活 conda）。
- 如果你有长期需要在 conda 中开发的需求，建议把仿真/系统级运行（roslaunch）和开发环境分离：在未激活 conda 的终端运行仿真，在 conda 中运行仅需的分析脚本。
- 本文档位于：`/home/idris/final_ws/src/sentry_chassis_controller/doc/rqt_plot_probleshooting.md`，可根据需要补充实际日志片段与更多场景。

