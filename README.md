# 项目进度总结

## 感谢 `https://zju-helloworld.github.io/Wiki/` 对项目的参考提供的帮助
截至目前的主要进展：

1. 成功运行了 `rom_description_for_task`，可以正确显示机器人的底盘模型（在仿真/可视化环境中能看到底盘）。

2. 在 CLion 中完成了编译工具链的配置，并使用 `catkin` 工具（catkin tools）对工程进行了编译。期间学习并阅读了 `simple_chassis_controller.cpp` 的实现细节，相关学习总结见 `doc/simple_chassis_controller_explanation.md`。

3. 已创建仓库 `sentry_chassis_controller`，并将本工作内容纳入该仓库中以便后续迭代与协作。

4. 新增了 launch 文件 `sentry_with_rviz`，启动仿真时可以同时查看机器人底盘模型与坐标系的变化，便于后面调试底盘。

备注：更多实现细节、代码分析与 pluginlib 学习笔记见 `doc/simple_chassis_controller_explanation.md`。

5. 新增了 `sentry_chassis_controller` 中的 `sentry_chassis_controller.cpp` 文件，并完成 `sentry_chassis_controller` 的实现。

6. 修复了launch文件启动的时候提示Missing model.config以及controller_manager无法正常加载的问题，通过修改xml文件以及通过修改目录 `rm_control` 下的 `rm_gazebo`功能包，并成功编译得到gazebo环境中的仿真插件库 `so` 件，已经能够在启动launch文件 `sentry_pid_test.launch` 的时候 `rostopic list` 显示 `/cmd_vel`

7. 修改了 `wheel_pid_controller.cpp` 文件，并通过学习博客(见文章 `rqt_reconfigure_learning.md`，在新创建的 `cfg` 目录下创建了 `WheelPid.cfg` 文件。同时修改参数文件 `wheel_pid_params`，完成工作空间的重新编译后，测试动态调整参数。

8. 集成了 `rqt_plot` `rqt_reconfigure` 插件到新的launch文件 `sentry_pid_test_fix.launch` 中，并成功在 `rqt` 中查看参数，能够支持动态调整参数。

9. 运行时发现rqt_plot无法正常显示数据，通过解决环境依赖的问题，并重新编译，成功运行，同时rqt_reconfigure插件也成功运行。![](src/sentry_chassis_controller/pictures/rqt_reconfigure_plot_demo.png)

10. 运动学正逆解算 `forward_kinematics.cpp` `inverse_kinematics.cpp` 测试完毕。现在可以通过键盘控制节点在仿真中控制底盘运动，同时通过rosbag的记录功能，记录/odom话题数据，在 `plotjuggler` 中回放查看数据，并完成数据可视化。![](src/sentry_chassis_controller/pictures/sentry_with_odom_running.png)

11. 正在努力实现按下按键wsad时底盘才会，qe按下后底盘旋转，两种运动可以独立进行并进行速度指令融合的效果，但是遇到了很多问题。

12. 回退到了安全的版本重新开发，准备在控制器中加入功率控制和重新写里程计发布的功能，以及重新修改键盘控制节点 `sentry_control_key_feature.cpp` 。

13. 再 `wheel_pid_controller.cpp` 添加了底盘功率控制，并通过终端INFO日志输出来查看是否有作用。

### 动态调参与参数路径说明（11月28日更新）

现在支持 rqt_reconfigure 动态调参。需要注意两套参数键路径：

- 动态参数（扁平键，供 rqt_reconfigure 使用）：
	- 例如 `/wheel_pid_controller/wheel_fl_p`、`/wheel_pid_controller/pivot_fr_i_clamp` 等。
	- 这些键直接驱动控制器内的 PID，在运行时可随时调整。

- YAML 分层参数（启动时加载的默认值）：
	- 例如 `/wheel_pid_controller/wheels/wheel_fl/p`、`/wheel_pid_controller/wheels/pivot_rr/d`。
	- 这是配置文件 `config/wheel_pid_params.yaml` 中的层级结构。

为方便在运行期用 `rosparam get` 查询旧的分层键，我们在动态回调中做了“镜像回写”：当你用 rqt_reconfigure 修改了 PID 值后，会同步写回到对应的 `wheels/<name>/{p,i,d,i_clamp}` 参数路径上（仅限参数服务器中的当前会话，文件不会自动修改）。

持久化方法：

- 导出动态参数：
	- `rosrun dynamic_reconfigure dynparam dump /wheel_pid_controller /tmp/wheel_pid_runtime.yaml`
- 或导出整个命名空间：
	- `rosparam dump /tmp/wheel_pid_namespace.yaml /wheel_pid_controller`

提示：`power_limit` 与 `power/*`（effort_coeff、velocity_coeff、power_offset）位于控制器命名空间的顶层，而不是 `wheels/` 下。

14.  完成了底盘控制器 `wheel_pid_controller.cpp` 全局速度模式的实现，并成功运行，并测试了小陀螺的功能，实现的方式其实是在 `global` 速度模式下把 `sentry_control_key.cpp` 中速度指令的控制方式修改为允许角速度和线速度同时叠加到底盘的运动中。

15. 完成了底盘控制器 `wheel_pid_controller.cpp` 自锁防止溜坡功能的实现，并进行了测试。。

16. 严肃使用rqt_plot和rqt_reconfigure进行参数调试，但是遇到车子会在前进方向偏移的问题。