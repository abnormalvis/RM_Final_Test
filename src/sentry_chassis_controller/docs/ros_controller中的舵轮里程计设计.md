ros\_controllers里有个four\_wheel\_steering\_controller,可以用于四驱车的控制,可以作为四驱车控制的gazebo plugin使用.

### 四个轮子的控制,包括轮子速度及轮子摆角

这里的四驱车模型是,每个轮子有独立的转向系统, 有独立的驱动系统. 因此需要地盘单独驱动每个轮子的摆角和速度.

有两种指令方式,一种是普通的geometry\_msg::twist, 一种是自定义的four\_wheel\_steering\_msgs,他的具体msg内容如下:

> float32 front\_steering\_angle           # position of the virtual angle (radians)  
> float32 rear\_steering\_angle            # position of the virtual angle (radians)  
> float32 front\_steering\_angle\_velocity  # rate of change (radians/s)  
> float32 rear\_steering\_angle\_velocity   # rate of change (radians/s)
> 
> float32 speed                   # forward speed (m/s)  
> float32 acceleration            # acceleration (m/s^2)  
> float32 jerk                    # jerk (m/s^3)

可以看出,这个指令,其实也没有具体到每个轮子的摆角和速度.因此也是需要根据地盘的几何信息i进行具体解算的. 那么就来看看他这里是怎么进行解算的.

两种msg类型,分别用了不同的callback: cmdVelCallback和cmdFourWheelSteeringCallback;msg中的数据分别放在了 command\_twist\_和command\_four\_wheel\_steering\_里, 然后在updateCommand中比较二者的stamp,选取时间最新的那个作为指令,同时检查是否超时, 若超时就刹车.

需要解算的量包括8个:

> double vel\_left\_front = 0, vel\_right\_front = 0;
> 
> double vel\_left\_rear = 0, vel\_right\_rear = 0;
> 
> double front\_left\_steering = 0, front\_right\_steering = 0;
> 
> double rear\_left\_steering = 0, rear\_right\_steering = 0;

需要的地盘几何信息有:

> const double steering\_track = track\_-2\*wheel\_steering\_y\_offset\_; 
> 
> 这个就是左右轮转向joint的间距, 是轮间距减去轮偏移. 

如果是geometry\_msg::twist指令,解算方法是:

> 四个轮子转速:
> 
> const double vel\_steering\_offset = (curr\_cmd\_twist.ang\*wheel\_steering\_y\_offset\_)/wheel\_radius\_;
> 
> const double sign = copysign(1.0, curr\_cmd\_twist.lin\_x);
> 
> vel\_left\_front = sign \* std::hypot((curr\_cmd\_twist.lin\_x - curr\_cmd\_twist.ang\*steering\_track/2),
> 
>                         (wheel\_base\_\*curr\_cmd\_twist.ang/2.0)) / wheel\_radius\_ - vel\_steering\_offset;
> 
> vel\_right\_front = sign \* std::hypot((curr\_cmd\_twist.lin\_x + curr\_cmd\_twist.ang\*steering\_track/2),
> 
>                         (wheel\_base\_\*curr\_cmd\_twist.ang/2.0)) / wheel\_radius\_ + vel\_steering\_offset;
> 
>  vel\_left\_rear = sign \* std::hypot((curr\_cmd\_twist.lin\_x - curr\_cmd\_twist.ang\*steering\_track/2),
> 
>                         (wheel\_base\_\*curr\_cmd\_twist.ang/2.0)) / wheel\_radius\_ - vel\_steering\_offset;
> 
> vel\_right\_rear = sign \* std::hypot((curr\_cmd\_twist.lin\_x + curr\_cmd\_twist.ang\*steering\_track/2),
> 
>                         (wheel\_base\_\*curr\_cmd\_twist.ang/2.0)) / wheel\_radius\_ + vel\_steering\_offset;
> 
> 4个轮子摆角:
> 
> if(fabs(2.0\*curr\_cmd\_twist.lin\_x) > fabs(curr\_cmd\_twist.ang\*steering\_track))
> 
> {
> 
>                 front\_left\_steering = atan(curr\_cmd\_twist.ang\*wheel\_base\_ / (2.0\*curr\_cmd\_twist.lin\_x - curr\_cmd\_twist.ang\*steering\_track));
> 
>                 front\_right\_steering = atan(curr\_cmd\_twist.ang\*wheel\_base\_ / (2.0\*curr\_cmd\_twist.lin\_x + curr\_cmd\_twist.ang\*steering\_track));
> 
> }
> 
> else if(fabs(curr\_cmd\_twist.lin\_x) > 0.001)
> 
> {
> 
>                 front\_left\_steering = copysign(M\_PI\_2, curr\_cmd\_twist.ang);
> 
>                 front\_right\_steering = copysign(M\_PI\_2, curr\_cmd\_twist.ang);
> 
> }
> 
> rear\_left\_steering = -front\_left\_steering;
> 
> rear\_right\_steering = -front\_right\_steering;

 另一种消息:four\_wheel\_steering\_msgs的解算如下:

四个轮子摆角:

```
front_left_steering  = atan(wheel_base_*tan_front_steering/(wheel_base_-steering_diff));
front_right_steering = atan(wheel_base_*tan_front_steering/(wheel_base_+steering_diff));
rear_left_steering   = atan(wheel_base_*tan_rear_steering/(wheel_base_-steering_diff));
rear_right_steering  = atan(wheel_base_*tan_rear_steering/(wheel_base_+steering_diff));
```

四个轮子速度:

```
        //Virutal front and rear wheelbase 虚拟的中心前轮/中心后轮线速度
        // distance between the projection of the CIR on the wheelbase and the front axle
        double l_front = 0;
        if(fabs(tan(front_left_steering) - tan(front_right_steering)) > 0.01)
        {
          l_front = tan(front_right_steering) * tan(front_left_steering) * steering_track
              / (tan(front_left_steering) - tan(front_right_steering));
        }
        // distance between the projection of the CIR on the wheelbase and the rear axle
        double l_rear = 0;
        if(fabs(tan(rear_left_steering) - tan(rear_right_steering)) > 0.01)
        {
          l_rear = tan(rear_right_steering) * tan(rear_left_steering) * steering_track
              / (tan(rear_left_steering) - tan(rear_right_steering));
        }

        // 再根据摆角和steering_track,计算左右轮各自的速度, 肯定是外侧的轮子速度要大一些的.
        const double angular_speed_cmd = curr_cmd_4ws.lin * (tan_front_steering-tan_rear_steering)/wheel_base_;
        const double vel_steering_offset = (angular_speed_cmd*wheel_steering_y_offset_)/wheel_radius_;
        const double sign = copysign(1.0, curr_cmd_4ws.lin);

        vel_left_front  = sign * std::hypot((curr_cmd_4ws.lin - angular_speed_cmd*steering_track/2),
                                            (l_front*angular_speed_cmd))/wheel_radius_
                          - vel_steering_offset;
        vel_right_front = sign * std::hypot((curr_cmd_4ws.lin + angular_speed_cmd*steering_track/2),
                                            (l_front*angular_speed_cmd))/wheel_radius_
                          + vel_steering_offset;
        vel_left_rear = sign * std::hypot((curr_cmd_4ws.lin - angular_speed_cmd*steering_track/2),
                                          (l_rear*angular_speed_cmd))/wheel_radius_
                        - vel_steering_offset;
        vel_right_rear = sign * std::hypot((curr_cmd_4ws.lin + angular_speed_cmd*steering_track/2),
                                           (l_rear*angular_speed_cmd))/wheel_radius_
                         + vel_steering_offset;
```

以上计算方法, 是要学习四驱车底盘运动学知识才能理解..

### odometry计算

odom的消息类型,这里也分了两类,一种是普通的geometry::odometry,另一种还是自定义的four\_wheel\_steering\_msgs::FourWheelSteeringStamped类型. 

计算的过程,是先读取每个wheel joint的速度, 以及每个steer joint的摆角位置信息.

第一步是先计算等效中心前轮/中心后轮的摆角位置, 

第二步是在自定义类Odometry中进行的.