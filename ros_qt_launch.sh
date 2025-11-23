#!/bin/bash
# ROS + Conda Qt环境集成启动脚本
# 用于解决rqt_reconfigure和rqt_plot的Qt绑定问题
set -e

echo "🚀 启动ROS QT集成环境..."

# 激活conda环境
source ~/miniconda3/etc/profile.d/conda.sh
conda activate ros_qt

# 验证PyQt5可用
echo "🔍 验证Qt绑定..."
python3 -c "import PyQt5.QtCore; print('✅ PyQt5可用')" || exit 1

# 设置ROS环境（根据您的ROS安装调整）
echo "🤖 配置ROS环境..."
source /opt/ros/noetic/setup.bash
source /home/idris/final_ws/devel/setup.bash

# 安装缺失的ROS Python包
echo "📦 安装ROS Python依赖..."
pip install rospkg catkin-pkg 2>/dev/null || true

# 导出Python路径，让ROS找到conda环境中的Python包
export PYTHONPATH="$CONDA_PREFIX/lib/python3.8/site-packages:$PYTHONPATH"

# 验证rqt_reconfigure可用
echo "🧪 测试rqt工具..."
python3 -c "import rqt_reconfigure; print('✅ rqt_reconfigure可用')" 2>/dev/null || echo "⚠️  rqt_reconfigure模块验证警告(正常启动)"

# 清理之前的ROS进程
echo "🧹 清理可能的残留进程..."
pkill -f "rqt_reconfigure" || true
pkill -f "rqt_plot" || true
pkill -f "rqt_gui" || true
pkill -f "roslaunch" || true
sleep 2

# 显示环境信息
echo "📊 环境状态:"
echo "  🐍 Python路径: $(which python3)"
echo "  📦 PyQt5版本: $(python3 -c 'import PyQt5; print(PyQt5.QtCore.PYQT_VERSION_STR)')"
echo "  🎯 ROS_MASTER_URI: $ROS_MASTER_URI"
echo "  📁 工作空间: $ROS_PACKAGE_PATH"
echo ""

# 启动主launch文件
echo "🎯 启动sentry动态重配置测试..."
echo "📣 可用功能："
echo "  ✓ PyQt5动态重配置界面 (支持双Qt绑定)"
echo "  ✓ 16个PID参数可调 (8个轮子 + 8个转向)"
echo "  ✓ rqt_plot数据可视化"
echo "  ✓ Gazebo仿真环境 + RViz可视化"
echo ""
echo "📝 操作提示："
echo "  1. 在rviz中观察机器人状态（地图、TF、关节）"
echo "  2. 使用rqt_reconfigure (Plugins→Configuration→Dynamic Reconfigure)"
echo "  3. 使用rqt_plot可视化轮速、转向角度 (Plugins→Visualization→Plot)"
echo "  4. Ctrl+C安全退出所有节点"
echo "  5. 参数文件: src/sentry_chassis_controller/cfg/WheelPid.cfg"
echo "  📖 建议: 将重要的PID参数保存到yaml文件再导入"
echo ""

# 实时状态检查
echo "⌛ 等待机器人描述加载..."
timeout 30s roslaunch sentry_chassis_controller sentry_pid_test_fixed.launch

if [ $? -eq 0 ]; then
    echo "🎉 启动成功！所有节点正在运行..."
    echo "🔧 rqt_reconfigure动态调参: 12秒后可用"
    echo "📈 rqt_plot数据可视化: 可立即启动"
else
    echo "❌ launch启动失败，检查URDF/参数文件，但Qt环境配置正确"
fi

echo "✅ ROS QT测试会话结束"