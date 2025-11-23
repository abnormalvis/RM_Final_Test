```markdown
# ROS 动态调参（dynamic_reconfigure）完整流程  
以「PID 参数实时调节」为例，给出从功能包创建到 rqt_reconfigure 调试的每一步命令与代码。

---

## 1  新建功能包
```bash
cd ~/catkin_ws/src
catkin_create_pkg pid roscpp rospy std_msgs dynamic_reconfigure
```

---

## 2  编写动态调参描述文件
### 2.1 建立 cfg 目录并新建描述文件
```bash
roscd pid
mkdir cfg
touch cfg/PID.cfg
chmod +x cfg/PID.cfg          # 必须可执行
```

### 2.2 PID.cfg 内容
```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
PACKAGE = "pid"

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()
gen.add("p", double_t, 0, "KP param", 0.0, -100.0, 100.0)
gen.add("i", double_t, 0, "KI param", 0.0, -100.0, 100.0)
gen.add("d", double_t, 0, "KD param", 0.0, -100.0, 100.0)

exit(gen.generate(PACKAGE, "pid_node", "PID"))
```

### 2.3 修改 CMakeLists.txt
取消注释并补充：
```cmake
find_package(catkin REQUIRED COMPONENTS
  ...
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/PID.cfg
)

catkin_package(
  ...
  CATKIN_DEPENDS dynamic_reconfigure
)
```

---

## 3  编写 Python 节点
### 3.1 新建节点
```bash
roscd pid
mkdir scripts
touch scripts/update_pid_node.py
chmod +x scripts/update_pid_node.py
```

### 3.2 update_pid_node.py 源码
```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from dynamic_reconfigure.server import Server
from pid.cfg import PIDConfig
from std_msgs.msg import Float32MultiArray

class UpdatePID:
    def __init__(self):
        rospy.init_node("update_pid")
        rospy.on_shutdown(self.shutdown)

        self.pub = rospy.Publisher("/pid", Float32MultiArray, queue_size=10)
        Server(PIDConfig, self.callback)

        rospy.loginfo("update_pid node ready. Launch rqt_reconfigure to tune PID.")
        rospy.spin()

    def callback(self, config, level):
        msg = Float32MultiArray()
        msg.data = [config["p"], config["i"], config["d"]]
        self.pub.publish(msg)
        rospy.loginfo("PID: P=%.2f  I=%.2f  D=%.2f", *msg.data)
        return config

    def shutdown(self):
        rospy.loginfo("Shutting down update_pid node.")

if __name__ == "__main__":
    UpdatePID()
```

---

## 4  编译与运行
```bash
cd ~/catkin_ws
catkin_make              # 生成头文件与 Python 配置模块
source devel/setup.bash

# 启动核心与节点
roscore
rosrun pid update_pid_node.py

# 动态调参界面
rosrun rqt_reconfigure rqt_reconfigure

# 观察话题（可选）
rosrun rqt_topic rqt_topic
```

---

## 5  效果
在 `rqt_reconfigure` 中拖动滑块即可实时修改 P/I/D，节点立即将新参数通过 `/pid` 话题发布，下位机或其他节点订阅后即可实现动态调参。
```