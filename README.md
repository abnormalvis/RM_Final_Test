项目进度总结

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

9. ![运行时发现rqt_plot无法正常显示数据，通过解决环境依赖的问题，并重新编译，成功运行，同时rqt_reconfigure插件也成功运行。](src/sentry_chassis_controller/pictures/rqt_reconfigure_plot_demo.png)