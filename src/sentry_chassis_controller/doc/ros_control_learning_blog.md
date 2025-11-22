![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/6c551c1dbd68509eeca963cc3a464c28.png#pic_center)

[ROS 系列学习教程(总目录)](https://blog.csdn.net/maizousidemao/article/details/135396993?spm=1001.2014.3001.5502)

#### 本文目录

+   [一、ros\_control 架构](#ros_control__11)
+   +   [1.1 hardware\_interface](#11_hardware_interface_31)
    +   [1.2 combined\_robot\_hw](#12_combined_robot_hw_37)
    +   [1.3 controller\_interface](#13_controller_interface_43)
    +   [1.4 controller\_manager](#14_controller_manager_49)
    +   [1.5 controller\_manager\_msgs](#15_controller_manager_msgs_55)
    +   [1.6 joint\_limits\_interface](#16_joint_limits_interface_61)
    +   [1.7 transmission\_interface](#17_transmission_interface_67)
    +   [1.8 realtime\_tools](#18_realtime_tools_73)
+   [二、ros\_controllers](#ros_controllers_79)
+   +   [2.1 控制器列表](#21__85)
    +   [2.2 配置控制器](#22__124)
+   [三、硬件抽象接口](#_151)
+   [四、命令行工具](#_184)
+   +   [4.1 controller\_manager](#41_controller_manager_186)
    +   [4.2 spawner](#42_spawner_222)
    +   [4.3 unspawner](#43_unspawner_234)
    +   [4.4 controller\_group （melodic 新增）](#44_controller_group_melodic__244)
+   [五、加载配置并管理控制器](#_279)
+   [六、ROS API](#ROS_API_307)
+   +   [6.1 controller\_manager/load\_controller](#61_controller_managerload_controller_311)
    +   [6.2 controller\_manager/unload\_controller](#62_controller_managerunload_controller_319)
    +   [6.3 controller\_manager/switch\_controller](#63_controller_managerswitch_controller_327)
    +   [6.4 controller\_manager/list\_controllers](#64_controller_managerlist_controllers_335)
    +   [6.5 controller\_manager/list\_controller\_types](#65_controller_managerlist_controller_types_343)
    +   [6.6 controller\_manager/reload\_controller\_libraries](#66_controller_managerreload_controller_libraries_351)

`ros_control` 是一个 ROS 功能包，是一个通用的机器人控制框架，用于实现机器人硬件接口、控制器管理和控制器接口。它提供了一套用于控制各种类型机器人（如移动机器人、机械臂等）的工具和接口。`ros_control` 的设计目的是为开发者提供一个灵活且可扩展的框架，使得控制算法能够独立于具体的硬件平台运行，并且可以轻松地在不同的机器人之间移植。

## 一、ros\_control 架构

`ros_control` 架构图如下：  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/bfa18113eb2c8932ff323d243ebfc1b2.png#pic_center)

`ros_control` 由以下几个主要组件组成：

+   hardware\_interface：硬件底层接口
+   combined\_robot\_hw：硬件包
+   controller\_interface：controller接口
+   controller\_manager：controller管理器
+   controller\_manager\_msgs：controller管理器的消息类型
+   joint\_limits\_interface：joints限制接口
+   transmission\_interface：传动接口
+   realtime\_tools：实时控制工具

### 1.1 hardware\_interface

`hardware_interface` 提供了一套硬件抽象接口，使得控制器可以独立于具体的硬件实现。

### 1.2 combined\_robot\_hw

`combined_robot_hw` 由 `CombinedRobotHW` 类实现，`CombinedRobotHW` 是一个硬件抽象类，允许将多个硬件接口组合在一起，从而让多个独立的硬件模块作为一个整体被控制器管理器控制。

### 1.3 controller\_interface

`controller_interface` 提供了定义控制器的基础接口，用于实现具体的控制器。控制器需要继承 `controller_interface::Controller` 类，并实现控制逻辑。

### 1.4 controller\_manager

`controller_manager` 负责管理控制器的生命周期，包括加载、启动、停止和卸载控制器。它提供了服务和话题接口，用于与控制器进行交互。

### 1.5 controller\_manager\_msgs

`controller_manager_msgs` 包含了与控制器管理器交互的消息类型和服务类型，例如加载、启动和停止控制器。

### 1.6 joint\_limits\_interface

`joint_limits_interface` 提供了一套接口，用于管理关节的限制条件（例如位置、速度、加速度和力矩限制），以确保关节在安全范围内运行。

### 1.7 transmission\_interface

`transmission_interface` 提供了传动装置的抽象接口，用于处理关节和电机之间的传动关系。它定义了传动装置的结构和方法，以便在控制器中正确处理关节和电机的转换。

### 1.8 realtime\_tools

`realtime_tools` 提供了一些实用工具，用于实现实时控制，包括实时缓冲区和周期性任务管理。

## 二、ros\_controllers

`ros_controllers` 也是一个 ROS 功能包，提供了一组预定义的控制器，这些控制器已经为各种常见的机器人控制任务进行了优化和实现，可以用于控制不同类型的机器人，如移动机器人、机械臂和其他多关节系统。

### 2.1 控制器列表

目前主要包含以下控制器：

+   joint\_state\_controller
    +   JointStateController：从硬件接口获取关节状态，并将这些状态信息发布到 `/joint_states` 话题
+   joint\_trajectory\_controller
    +   JointTrajectoryController：接收轨迹信息（即一系列期望的关节位置、速度和加速度），并控制机器人的关节按照这些轨迹运动。该控制器适用于需要精确轨迹跟踪的任务，如路径规划和任务执行。
+   gripper\_action\_controller
    +   GripperActionController：控制机械手夹具的开闭位置
+   diff\_drive\_controller
    +   DiffDriveController：控制差速驱动的机器人
+   effort\_controllers
    +   JointEffortController：控制单个关节的力/力矩。通过设定期望的力/力矩，使关节达到所需状态。
    +   JointPositionController：控制单个关节的位置，通过力/力矩控制算法使关节达到设定的位置。
    +   JointVelocityController：控制单个关节的速度，通过力/力矩控制算法使关节达到设定的速度。
    +   JointGroupEffortController：控制一组关节的力/力矩。通过设定多个关节的力/力矩，使它们达到所需状态。
    +   JointGroupPositionController：控制一组关节的位置，通过力/力矩控制算法使这些关节达到设定的位置。
+   velocity\_controllers
    +   JointVelocityController：用于控制机器人单个关节的速度
    +   JointPositionController：通过PID控制速度以达到设置的位置
    +   JointGroupVelocityController：用于控制机器人多个关节的速度
+   position\_controllers
    +   JointPositionController：直接控制单个关节的位置
    +   JointGroupPositionController：直接控制一组关节的位置
+   imu\_sensor\_controller
    +   IMUSensorController：发布 IMU 传感器的数据
+   force\_torque\_sensor\_controller
    +   ForceTorqueSensorController：发布力矩传感器的数据
+   forward\_command\_controller
    +   ForwardCommandController：用于将单个关节的命令直接传递到硬件接口
    +   ForwardJointGroupCommandController：用于将一组关节的命令直接传递到硬件接口
+   ackermann\_steering\_controller
    +   AckermannSteeringController：接收线速度和转向角度指令，然后根据 Ackermann 转向几何原理计算并控制各个车轮的转向角和速度，使车辆能够按照预定轨迹运动
+   four\_wheel\_steering\_controller
    +   FourWheelSteeringController：接收线速度和转向角度指令，然后根据四轮转向几何原理计算并控制各个车轮的转向角和速度，使车辆能够按照预定轨迹运动

### 2.2 配置控制器

通过将参数加载到ROS参数服务器来配置控制器，一般先将配置写入 `yaml` 文件，再通过文件加载到ROS参数服务器，然后 `ros_control` 会解析特定参数。控制器配置示例如下：

```yaml
my_robot:
	joint_state_controller:
       type: joint_state_controller/JointStateController
       publish_rate: 50

    position_trajectory_controller:
       type: position_controllers/JointTrajectoryController
       joints:
          - joint1
          - joint2
       constraints:
          goal_time: 5.0
          joint1:
              trajectory: 0.60
              goal:       0.15
          joint2:
              trajectory: 0.60
              goal:       0.15
```

不同控制器的参数不同，详见 [http://wiki.ros.org/ros\_controllers](http://wiki.ros.org/ros_controllers)

## 三、硬件抽象接口

ros\_control 的最大优点就是把上层业务与底层硬件隔离开，使业务层不依赖于特定的硬件，为此， ros\_control 提供了硬件抽象接口，控制器通过硬件接口和硬件交互数据。

目前 ros\_control 实现的硬件接口有：

+   [JointCommandInterface](https://docs.ros.org/en/melodic/api/hardware_interface/html/c++/joint__command__interface_8h_source.html)：支持命令关节阵列的硬件接口，他有三个派生类：
    +   EffortJointInterface：用于控制基于力/力矩的关节
    +   PositionJointInterface：用于控制基于速度的关节
    +   VelocityJointInterface：用于控制基于位置的关节
+   [JointStateInterface](https://docs.ros.org/en/melodic/api/hardware_interface/html/c++/joint__state__interface_8h_source.html)：读取关节的状态，每个关节都具有一定的位置、速度和力（或扭矩）
+   [ActuatorStateInterface](https://docs.ros.org/en/melodic/api/hardware_interface/html/c++/actuator__state__interface_8h_source.html)：读取执行器的状态，每个执行器都具有一定的位置、速度和力（或扭矩）
+   [ActuatorCommandInterface](https://docs.ros.org/en/melodic/api/hardware_interface/html/c++/actuator__command__interface_8h_source.html)
    +   EffortActuatorInterface
    +   VelocityActuatorInterface
    +   PositionActuatorInterface
+   [PosVelJointInterface](https://docs.ros.org/en/melodic/api/hardware_interface/html/c++/posvel__command__interface_8h_source.html)：通过位置、速度控制关节
+   [PosVelAccJointInterface](https://docs.ros.org/en/melodic/api/hardware_interface/html/c++/posvelacc__command__interface_8h_source.html)：通过位置、速度和加速度来控制关节
+   [ForceTorqueSensorInterface](https://docs.ros.org/en/melodic/api/hardware_interface/html/c++/force__torque__sensor__interface_8h_source.html)：读取力矩传感器的状态
+   [ImuSensorInterface](https://docs.ros.org/en/melodic/api/hardware_interface/html/c++/imu__sensor__interface_8h_source.html)：读取 IMU 传感器的状态
+   [JointModeInterface](https://docs.ros.org/en/melodic/api/hardware_interface/html/c++/joint__mode__interface_8h_source.html)：控制关节模式的切换

各接口类继承关系如下：

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/5d1fc80fa469e9d3ec42d79a78b72217.png#pic_center)

各 `Interface` 实例将对应的 `Handel` 实例注册到硬件资源管理器，在硬件资源管理器统一管理所有抽象硬件。

如`ros_control` 架构图所示，上层 `Controller` 通过各 `Interface` 与抽象硬件交互数据，进而对实际硬件进行读写数据。

## 四、命令行工具

### 4.1 controller\_manager

管理控制器生命周期：

```bash
rosrun controller_manager controller_manager <command> <name1> <name2> ...
```

其中，`<command>` 可选如下：

+   `load`：加载控制器（构造和初始化）
+   `unload`：卸载控制器（析构）
+   `start`：启动控制器
+   `stop`：停止控制器
+   `spawn`：加载并启动控制器
+   `kill`：停止并卸载控制器

`<name1> <name2>` 为控制器的名称。

获取控制器的状态：

```bash
rosrun controller_manager controller_manager <command>
```

其中，`<command>` 可选如下：

+   `list`：按执行顺序列出所有控制器，并给出每个控制器的状态
+   `list-types`：列出控制器管理器知道的所有控制器类型。如果控制器不在此列表中，将无法生成。
+   `reload-libraries`：重新加载所有可用作插件的控制器库。当新开发控制器并想要测试新的控制器代码时，这很方便，而无需每次都重新启动机器人。这不会重新启动之前运行的控制器。
+   `reload-libraries --restore`：重新加载所有可用作插件的控制器库，并将所有控制器恢复到其原始状态。

### 4.2 spawner

自动加载并启动一组控制器，并自动停止并卸载相同的控制器：

```
rosrun controller_manager spawner [--stopped] <name1> <name2> ...
```

当运行 spawner 时，列出的控制器将被加载并启动，如果加上 `--stopped` 参数，控制器只会被加载不会被启动。在控制器启动时，spawner 将继续运行，当终止 spawner（如：ctrl-c）时，它将自动停止并卸载它最初启动的所有控制器。

### 4.3 unspawner

停止控制器，但不卸载

```bash
rosrun controller_manager unspawner <name1> <name2> ...
```

### 4.4 controller\_group （melodic 新增）

`controller_manager` 允许开发人员在运行时切换控制器，但是当出于某些特殊目的想要从一组控制器切换到另一组控制器时，它就不那么方便了。如果在 ROS 参数 `controller_groups` 中定义了这样的组，`controller_group` 脚本就可以让这变得简单。它知道所有涉及的控制器，然后知道在从一个组切换到另一个组时需要停止和启动的控制器。因此，不同的组可以共享一些控制器。

控制器分组配置，和控制器配置一样要先加载到ROS参数服务器：

```xml
controller_groups:
  production:
    - prod_controller_1
    - prod_controller_2
  development:
    - devel_controller_1
    - devel_controller_2
    - shared_controller_3
  diagnostics:
    - diag_controller_1
    - diag_controller_2
    - shared_controller_3
```

运行 `controller_group` ：

```bash
rosrun controller_manager controller_group <command> <args>
```

其中，`<command>` 可选如下：

+   `list` ：列出在 `controller_groups` 参数中找到的所有组定义
+   `spawn <group>` ：加载并启动名为 `<group>` 的组中包含的所有控制器。这通常在 ROS 启动文件中使用
+   `switch <group>` ：切换到名为 `<group>` 的组。这将停止那些在其他组中定义但未在本组中运行的控制器，并启动本组中定义的未运行的控制器。

## 五、加载配置并管理控制器

启动控制器时要注意，运行 `controller_manager` 来从启动文件中启动控制器时，即使启动文件被删除，控制器也会继续运行。建议使用 `spawner` 工具从启动文件中自动加载、启动、停止和卸载控制器，当启动 `spawner` 时，将加载并启动控制器。当停止 `spawner` 时（启动文件被删除或停止运行），将停止并卸载控制器。

```xml
<launch>
   <!-- 加载配置文件 --> 
	<rosparam file="$(find rrbot_control)/config/rrbot_control.yaml" command="load"/>

    <!-- 启动控制器 -->
 	<node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen" ns="/my_robot" args="joint_state_controller position_trajectory_controller"/>
</launch>
```

图形化管理控制器：

rqt\_controller\_manager是一个 rqt 插件，允许以图形方式加载、卸载、启动和停止控制器，以及显示有关已加载控制器的信息。

它可以从 rqt 的插件菜单启动，或者作为独立可执行文件启动：

```bash
rosrun rqt_controller_manager rqt_controller_manager
```

## 六、ROS API

为了与其他 ROS 节点的控制器交互，控制器管理器提供了五个服务供调用：

### 6.1 controller\_manager/load\_controller

加载指定控制器

消息体：[controller\_manager\_msgs/LoadController](http://docs.ros.org/en/api/controller_manager_msgs/html/srv/LoadController.html)

### 6.2 controller\_manager/unload\_controller

卸载控制器，控制器仅在停止状态下才可卸载。

消息体：[controller\_manager\_msgs/UnloadController](http://docs.ros.org/en/api/controller_manager_msgs/html/srv/UnloadController.html)

### 6.3 controller\_manager/switch\_controller

启动/停止 控制器

消息体：[controller\_manager\_msgs/SwitchController](http://docs.ros.org/en/api/controller_manager_msgs/html/srv/SwitchController.html)

### 6.4 controller\_manager/list\_controllers

获取当前已加载的所有控制器

消息体：[controller\_manager\_msgs/ListControllers](http://docs.ros.org/en/api/controller_manager_msgs/html/srv/ListControllers.html)

### 6.5 controller\_manager/list\_controller\_types

获取已知的所有控制器类型

消息体：[controller\_manager\_msgs/ListControllerTypes](http://docs.ros.org/en/api/controller_manager_msgs/html/srv/ListControllerTypes.html)

### 6.6 controller\_manager/reload\_controller\_libraries

重新加载所有可用作插件的控制器库

消息体：[controller\_manager\_msgs/ReloadControllerLibraries](http://docs.ros.org/en/api/controller_manager_msgs/html/srv/ReloadControllerLibraries.html)
