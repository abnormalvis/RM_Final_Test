#### 文章目录

- [1. 键盘控制C++实现(x、y、z、theta)](#1-键盘控制c实现xyztheta)
- [2. 键盘控制Python实现(x、y、z、theta)](#2-键盘控制python实现xyztheta)
  - [代码1](#代码1)
  - [代码2](#代码2)
- [3.键盘笛卡尔方向控制(x、y、z)](#3键盘笛卡尔方向控制xyz)
- [4. 总结](#4-总结)

通常情况下，对于机械臂的笛卡尔系下的控制或者小车的控制，我们在测试的时候可以通过键盘来实现，以下给出博主在网上找到的源码，能避免重复造轮子，更快的达到键盘控制的目的。

值得注意的是，以下代码左右键均是小车控制的代码，左右方向控制的是theta，因此对于机械臂的笛卡尔系控制还需要进一步修改代码。

## 1\. 键盘控制C++实现(x、y、z、theta)

* * *

**说明：** 在终端中按下键盘里的“W”、“S”、“D”、“A”以及“Shift”键进行机器人的控制[1](#fn1)。

* * *

```
/*
 * =====================================================================================
 *        COPYRIGHT NOTICE
 *        Copyright (c) 2013  HUST-Renesas Lab
 *        ALL rights reserved.
 *//**        
 *        @file     keyboard.cpp
 *        @brief    robot keyboard control
 *        @version  0.1
 *        @date     2013/5/23 15:22:40
 *        @author   Hu Chunxu , huchunxu@hust.edu.cn
 *//* ==================================================================================
 *  @0.1    Hu Chunxu    2013/5/23 15:22:40   create orignal file
 * =====================================================================================
 */

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

/* 带有shift键 */
#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57

class SmartCarKeyboardTeleopNode
{
    private:
        double walk_vel_;
        double run_vel_;
        double yaw_rate_;
        double yaw_rate_run_;
        
        geometry_msgs::Twist cmdvel_;
        ros::NodeHandle n_;
        ros::Publisher pub_;

    public:
        SmartCarKeyboardTeleopNode()
        {
            pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            
            ros::NodeHandle n_private("~");
            n_private.param("walk_vel", walk_vel_, 0.5);
            n_private.param("run_vel", run_vel_, 1.0);
            n_private.param("yaw_rate", yaw_rate_, 1.0);
            n_private.param("yaw_rate_run", yaw_rate_run_, 1.5);
        }
        
        ~SmartCarKeyboardTeleopNode() { }
        void keyboardLoop();
        
        void stopRobot()
        {
            cmdvel_.linear.x = 0.0;
            cmdvel_.angular.z = 0.0;
            pub_.publish(cmdvel_);
        }
};

SmartCarKeyboardTeleopNode* tbk;

/**
 * 文件描述符
 * 内核（kernel）利用文件描述符（file descriptor）来访问文件。文件描述符是非负整数。
 * 标准输入（standard input）的文件描述符是 0，标准输出（standard output）是 1，标准错误（standard error）是 2。
 */
int kfd = 0;

/** 
 *  === struct termios ===
 *  tcflag_t c_iflag;  输入模式
 *　tcflag_t c_oflag;  输出模式 
 *　tcflag_t c_cflag;  控制模式 
 *  tcflag_t c_lflag;  本地模式
 *  cc_t c_cc[NCCS];   控制字符 
 */
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    SmartCarKeyboardTeleopNode tbk;
   
	/* 创建一个新的线程 */
    boost::thread t = boost::thread(boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));
    
    ros::spin();
    
    t.interrupt();
    t.join();
    tbk.stopRobot();

	/* 设置终端参数 */
    tcsetattr(kfd, TCSANOW, &cooked);
    
    return(0);
}

void SmartCarKeyboardTeleopNode::keyboardLoop()
{
    char c;
    double max_tv = walk_vel_;
    double max_rv = yaw_rate_;
    bool dirty = false;
    int speed = 0;
    int turn = 0;
    
    /** 
	 * 从终端中获取按键 
	 * int tcgetattr(int fd, struct termios *termios_p);
	 */
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));

    /**
	 * c_lflag : 本地模式标志，控制终端编辑功能
	 * ICANON: 使用标准输入模式
	 * ECHO: 显示输入字符
	 */
    raw.c_lflag &=~ (ICANON | ECHO);

	/** 
	 * c_cc[NCCS]：控制字符，用于保存终端驱动程序中的特殊字符，如输入结束符等
	 * VEOL: 附加的End-of-file字符
	 * VEOF: End-of-file字符
	 * */
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    
    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");
    
	/* *
	 * struct pollfd {
　　       int fd;        文件描述符 
　       　short events;  等待的事件 
　　       short revents; 实际发生了的事件 
　       　};
    */
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    
    for(;;)
    {
        boost::this_thread::interruption_point();
        
        /* get the next event from the keyboard */
        int num;
        
		/**
		 * poll:把当前的文件指针挂到设备内部定义的等待队列中。
		 * unsigned int (*poll)(struct file * fp, struct poll_table_struct * table)
		 */
        if ((num = poll(&ufd, 1, 250)) < 0)
        {
			/**
			 * perror( ) 用来将上一个函数发生错误的原因输出到标准设备(stderr)。
			 * 参数s所指的字符串会先打印出,后面再加上错误原因字符串。
			 * 此错误原因依照全局变量errno 的值来决定要输出的字符串。
			 * */
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
			/* 每按下一次动一下 */
            if (dirty == true)
            {
                stopRobot();
                dirty = false;
            }
            
            continue;
        }
        
        switch(c)
        {
            case KEYCODE_W:
                max_tv = walk_vel_;
                speed = 1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_S:
                max_tv = walk_vel_;
                speed = -1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_A:
                max_rv = yaw_rate_;
                speed = 0;
                turn = 1;
                dirty = true;
                break;
            case KEYCODE_D:
                max_rv = yaw_rate_;
                speed = 0;
                turn = -1;
                dirty = true;
                break;
                
            case KEYCODE_W_CAP:
                max_tv = run_vel_;
                speed = 1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_S_CAP:
                max_tv = run_vel_;
                speed = -1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_A_CAP:
                max_rv = yaw_rate_run_;
                speed = 0;
                turn = 1;
                dirty = true;
                break;
            case KEYCODE_D_CAP:
                max_rv = yaw_rate_run_;
                speed = 0;
                turn = -1;
                dirty = true;
                break;
                
            default:
                max_tv = walk_vel_;
                max_rv = yaw_rate_;
                speed = 0;
                turn = 0;
                dirty = false;
        }
        
        cmdvel_.linear.x = speed * max_tv;
        cmdvel_.angular.z = turn * max_rv;
        pub_.publish(cmdvel_);
    }
}

```

## 2\. 键盘控制Python实现(x、y、z、theta)

### 代码1

* * *

**说明：** 在终端中按下键盘里的“W”、“S”、“D”、“A”以及“Shift”键进行机器人的控制[1](#fn1)。

* * *

```
#!/usr/bin/env python
# -*- coding: utf-8 -*
# =====================================================================================
#        COPYRIGHT NOTICE
#        Copyright (c) 2013  HUST-Renesas Lab
#        ALL rights reserved.
#        
#        @file     teleop
#        @brief    robot keyboard control
#        @version  0.1
#        @date     2013/5/23 15:34:40
#        @author   Hu Chunxu , huchunxu@hust.edu.cn
# ==================================================================================
#  @0.1    Hu Chunxu    2013/5/23   create orignal file
# =====================================================================================
import  os
import  sys
import  tty, termios

import roslib; roslib.load_manifest('smartcar_teleop')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# 全局变量
cmd = Twist()
pub = rospy.Publisher('cmd_vel', Twist)

def keyboardLoop():
    #初始化
    rospy.init_node('smartcar_teleop')
    # Set rospy to exectute a shutdown function when exiting       
    rate = rospy.Rate(rospy.get_param('~hz', 1))

    #速度变量
    walk_vel_ = rospy.get_param('walk_vel', 0.5)
    run_vel_ = rospy.get_param('run_vel', 1.0)
    yaw_rate_ = rospy.get_param('yaw_rate', 1.0)
    yaw_rate_run_ = rospy.get_param('yaw_rate_run', 1.5)

    max_tv = walk_vel_
    max_rv = yaw_rate_

    #显示提示信息
    print "Reading from keyboard"
    print "Use WASD keys to control the robot"
    print "Press Caps to move faster"
    print "Press q to quit"

    #读取按键循环
    while not rospy.is_shutdown():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
		#不产生回显效果
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try :
            tty.setraw( fd )
            ch = sys.stdin.read( 1 )
        finally :
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if ch == 'w':
            max_tv = walk_vel_
            speed = 1
            turn = 0
        elif ch == 's':
            max_tv = walk_vel_
            speed = -1
            turn = 0
        elif ch == 'a':
            max_rv = yaw_rate_
            speed = 0
            turn = 1
        elif ch == 'd':
            max_rv = yaw_rate_
            speed = 0
            turn = -1
        elif ch == 'W':
            max_tv = run_vel_
            speed = 1
            turn = 0
        elif ch == 'S':
            max_tv = run_vel_
            speed = -1
            turn = 0
        elif ch == 'A':
            max_rv = yaw_rate_run_
            speed = 0
            turn = 1
        elif ch == 'D':
            max_rv = yaw_rate_run_
            speed = 0
            turn = -1
        elif ch == 'q':
            exit()
        else:
            max_tv = walk_vel_
            max_rv = yaw_rate_
            speed = 0
            turn = 0

        #发送消息
        cmd.linear.x = speed * max_tv;
        cmd.angular.z = turn * max_rv;
        pub.publish(cmd)
        rate.sleep()
		#停止机器人
        stop_robot();

def stop_robot():
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    pub.publish(cmd)

if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
```

### 代码2

* * *

**说明[2](#fn2)：**

For Holonomic mode (strafing), hold down the shift key:

U ---------- I ---------- O  
J ---------- K ---------- L  
M ---------- < ---------- >

t : up (+z)

b : down (-z)

其他按键 : stop

q/z : 最大速度增加/减少10%  
w/x : 仅线性速度增加/减少10%  
e/c : 只增加/减少角速度10%

CTRL-C 退出

* * *

```
#!/usr/bin/env python
# -*- coding: utf-8 -*

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	rospy.init_node('teleop_twist_keyboard')

	speed = rospy.get_param("~speed", 0.5)
	turn = rospy.get_param("~turn", 1.0)
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		print(msg)
		print(vels(speed,turn))
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print(vels(speed,turn))
				if (status == 14):
					print(msg)
				status = (status + 1) % 15
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			pub.publish(twist)

	except Exception as e:
		print(e)

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


```

## 3.键盘笛卡尔方向控制(x、y、z)

基于2中的代码2来实现

```
#!/usr/bin/env python
# -*- coding: utf-8 -*

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
---------------------------
        I     
   J    K    L
        <     

I : +x		J : +y		T : +z
< : -x		L : -y		B : -z
		K : stop

其他按键 : stop

q/z : 最大速度增加/减少10%
w/x : 仅线性速度增加/减少10%
e/c : 只增加/减少角速度10%

CTRL-C 退出
---------------------------
"""

moveBindings = {
		'i':(1,0,0,0),
		'j':(0,1,0,0),
		'l':(0,-1,0,0),
		',':(-1,0,0,0),

		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'<':(-1,0,0,0),


		't':(0,0,1,0),
		'b':(0,0,-1,0),
		'T':(0,0,1,0),
		'B':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	rospy.init_node('teleop_twist_keyboard')

	speed = rospy.get_param("~speed", 0.05)
	turn = rospy.get_param("~turn", 0.1)
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		print(msg)
		print(vels(speed,turn))
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print(vels(speed,turn))
				if (status == 14):
					print(msg)
				status = (status + 1) % 15
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			pub.publish(twist)

	except Exception as e:
		print(e)

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


```

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/f581668a9432edfdcd15366da2f74660.png)

## 4\. 总结

+   在ROS下进行控制时，首先启动键盘控制节点，然后订阅相应的话题即可。
    
+   以3为例：
    

1.  首先启动ROS Master：`roscore`
    
2.  运行该节点：`rosrun teleop_twist_keyboard teleop_twist_keyboard_descartes.py`  
    ![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/18751220a15ce39955991afe31d53a18.png#pic_center)
    
3.  查看话题列表：`rostopic list`  
    ![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/428bbf086ca568bb76f3fe629af7f8cb.png#pic_center)
    
4.  查看话题内容：`rostopic echo cmd_vel`  
    ![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/b13a3cb172a5b1215180dbc3adc6fe4d.png#pic_center)
    
5.  查看`/cmd_vel`话题的消息类型：`rostopic type cmd_vel`  
    ![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/94293a4b6c3893697738aea1f7afcb78.png#pic_center)
    
6.  订阅`/cmd_vel`话题后根据消息文件接收数据即可
    

* * *

1.  [http://www.guyuehome.com/253](http://www.guyuehome.com/253) [↩︎](#fnref1) [↩︎](#fnref1:1)
    
2.  [https://github.com/ncnynl/teleop\_twist\_keyboard](https://github.com/ncnynl/teleop_twist_keyboard) [↩︎](#fnref2)