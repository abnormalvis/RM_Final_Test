这里面我觉得重要的话，而且我还看到了twist，这不正是前阵子普罗米修斯群里问的T265发布的带不带速度信息所说到的twist？

![](https://i-blog.csdnimg.cn/blog_migrate/f6d7eb22f98a373e90aaa6048df8d54c.png)

导航包使用tf来确定机器人在世界中的位置，并将传感器数据与静态地图相关联。然而，tf不提供关于机器人的速度的任何信息。因此，导航包要求任何odometry源通过ROS发布包含速度信息的transform和nav\_msgs/Odometry消息。

摘自：[https://www.cnblogs.com/gary-guo/p/7215284.html](https://www.cnblogs.com/gary-guo/p/7215284.html)

### [Odometry的发布和发布odom到base\_link的tf变换](https://www.cnblogs.com/gary-guo/p/7215284.html)

转载自http://www.ncnynl.com/archives/201702/1328.html

ROS发布nav\_msgs/Odometry消息，以及通过tf从“odom”坐标系到“base\_link”坐标系的转换。

**在ROS上发布Odometry信息**

导航包使用tf来确定机器人在世界中的位置，并将传感器数据与静态地图相关联。然而，tf不提供关于机器人的速度的任何信息。因此，导航包要求任何odometry源通过ROS发布包含速度信息的transform和nav\_msgs/Odometry消息。

**nav\_msgs/Odometry消息**

nav\_msgs/Odometry信息存储在自由空间的机器人的位置和速度的估计：

![复制代码](https://i-blog.csdnimg.cn/blog_migrate/9fafc5631ba1215db74dcb775c5144aa.png)

```html
# This represents an estimate of a position and velocity in free space.  
# 这表示对自由空间中的位置和速度的估计
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# 此消息中的姿势应在由header.frame_id给定的坐标系中指定
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
# 这个消息中的twist应该在由child_frame_id给出的坐标系指定
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

![复制代码](https://i-blog.csdnimg.cn/blog_migrate/9fafc5631ba1215db74dcb775c5144aa.png)

下面是更详细的消息类型

[std\_msgs/Header](http://docs.ros.org/api/std_msgs/html/msg/Header.html) header  
string child\_frame\_id  
[geometry\_msgs/PoseWithCovariance](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html) pose  
[geometry\_msgs/TwistWithCovariance](http://docs.ros.org/api/geometry_msgs/html/msg/TwistWithCovariance.html) twist

该消息中的姿势(pose)对应于机器人在世界坐标系中的估计位置以及该姿势估计特定的可选协方差。

该消息中的速度(twist)对应于机器人在子坐标系的速度，通常是移动基站的坐标系，以及该速度估计特定的可选协方差。

**使用tf发布Odometry变换**

如[变换配置教程](http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF)中所讨论的，“tf”软件库负责管理与变换树中的机器人相关的坐标系之间的关系。

**因此，任何odometry(里程)源都必须发布其管理的坐标系的相关信息。**

**编写代码**

我们将编写一些用于通过ROS发布 nav\_msgs/Odometry消息的示例代码，以及使用tf的虚拟机器人的变换。

catkin\_create\_pkg Odom tf nav\_msgs roscpp rospy

![复制代码](https://i-blog.csdnimg.cn/blog_migrate/9fafc5631ba1215db74dcb775c5144aa.png)

```html
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
```

![复制代码](https://i-blog.csdnimg.cn/blog_migrate/9fafc5631ba1215db74dcb775c5144aa.png)

**代码解释：**

+   代码：

```
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
```

+   解释：我们发布odom坐标系到base\_link坐标系的变换和nav\_msgs/Odometry消息。需要包含相关头文件
+   代码：

```
ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
tf::TransformBroadcaster odom_broadcaster;
```

+   解释：创建ros::Publisher和tf::TransformBroadcaster实例，以便能够分别使用ROS和tf发送消息。
+   代码：

```
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
```

+   解释：我们假设机器人最初从“odom”坐标系的原点开始
+   代码：

```
 double vx = 0.1;
 double vy = -0.1;
 double vth = 0.1;
```

+   解释：在这里，我们将设置一些速度，其将导致“base\_link”坐标系相对于“odom”坐标系，在x方向上以0.1m/s，在y方向上以-0.1m/s的速率和在th方向以0.1rad/s角度移动。这让虚拟机器人走一个圆圈。
    
+   代码:
    

```
ros::Rate r(1.0);
```

+   解释：我们将在这个例子中以1Hz的速率发布里程计信息，以使自我检查更容易，大多数系统将希望以更高的速率发布里程消息。
+   代码：

```
//compute odometry in a typical way given the velocities of the robot
double dt = (current_time - last_time).toSec();
double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
double delta_th = vth * dt;

x += delta_x;
y += delta_y;
th += delta_th;
```

+   解释：
    
    +   在这里，我们将根据我们设置的恒定速度更新我们的里程信息。
    +   当然，真正的里程系统将整合计算的速度。
+   代码：
    

```
//since all odometry is 6DOF we'll need a quaternion created from yaw
geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
```

+   解释：
    
    +   我们通常尝试使用我们系统中所有消息的3D版本，以允许2D和3D组件在适当的时候一起工作，并将我们要创建的消息数量保持在最小。
    +   因此，有必要将我们用于里程的偏航值转换为四元数而发送出去。
    +   幸运的是，tf提供了允许从偏航值容易地创建四元数并且容易地访问四元数的偏航值的功能。
+   代码：
    

```
//first, we'll publish the transform over tf
geometry_msgs::TransformStamped odom_trans;
odom_trans.header.stamp = current_time;
odom_trans.header.frame_id = "odom";
odom_trans.child_frame_id = "base_link";
```

+   解释：
    
    +   这里我们将创建一个TransformStamped消息，我们将通过tf发送。
    +   我们想在“current\_time”发布从“odom”坐标系到“base\_link”坐标系的转换。
    +   因此，我们将相应地设置消息的头部和child\_frame\_id，确保使用“odom”作为父坐标系，“base\_link”作为子坐标系。
+   代码：
    

```
odom_trans.transform.translation.x = x;
odom_trans.transform.translation.y = y;
odom_trans.transform.translation.z = 0.0;
odom_trans.transform.rotation = odom_quat;

//send the transform
odom_broadcaster.sendTransform(odom_trans);
```

+   解释：这里我们从我们的odometry数据填充变换消息，然后使用我们的TransformBroadcaster发送变换。
    
+   代码：
    

```
//next, we'll publish the odometry message over ROS
nav_msgs::Odometry odom;
odom.header.stamp = current_time;
odom.header.frame_id = "odom";
```

+   解释：
    
    +   我们还需要发布nav\_msgs/Odometry消息，以便导航堆栈可以从中获取速度信息。
    +   我们将消息的头部设置为current\_time和“odom”坐标系。
+   代码：
    

```
//set the position
odom.pose.pose.position.x = x;
odom.pose.pose.position.y = y;
odom.pose.pose.position.z = 0.0;
odom.pose.pose.orientation = odom_quat;

//set the velocity
odom.child_frame_id = "base_link";
odom.twist.twist.linear.x = vx;
odom.twist.twist.linear.y = vy;
odom.twist.twist.angular.z = vth;
```

+   解释：
    +   这将使用里程数据填充消息，并通过线路发送。
    +   我们将消息的child\_frame\_id设置为“base\_link”坐标系，因为这是我们发送速度信息的坐标系。

CMakeLists.txt

 add\_executable(Odom\_exam src/Odom\_exam.cpp)  
target\_link\_libraries(Odom\_exam ${catkin\_LIBRARIES})  
![](https://i-blog.csdnimg.cn/blog_migrate/82c5398e8eb6d7bce169fb3fe43de71c.png)

![](https://i-blog.csdnimg.cn/blog_migrate/57554bbe2638baa04d215ca530ad5c8c.png)

![](https://i-blog.csdnimg.cn/blog_migrate/98740093f37be9b5b06dc93f8b2bf1c2.png)

分类: [ROS](https://www.cnblogs.com/gary-guo/category/932622.html)