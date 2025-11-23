 

* * *

[](https://blog.csdn.net/weixin_73037889/article/details/131052481)前言
---------------------------------------------------------------------

        在RoboMaster比赛中，舵轮底盘具有很强的机动性。舵轮机器人的底盘上一共有八个电机，其中包括四个行进轮电机和四个转向轮电机（舵机）。目前我们机械组采用GM6020电机作为舵机，行进轮是M3508、当然其实用什么电机做舵机对我们嵌入式来说并没有太大的关系。，我们只需要在代码上稍加改动即可。

[](https://blog.csdn.net/weixin_73037889/article/details/131052481)一、舵轮姿态解算
---------------------------------------------------------------------------

舵轮底盘的姿态解算如下：

  首先，任何机器人的运动都可以分解成三个方向：Vx（前后）、Vy（左右）、Vw（旋转）。

![](https://i-blog.csdnimg.cn/blog_migrate/13ce6bfdb35271143b2d7f68ac5fa040.png)

 ![](https://i-blog.csdnimg.cn/blog_migrate/80db89cd071b28457a96fb097ad8d044.png)

其中，Vw1=Vw2=Vw3=Vw4=w(旋转角速度) xR（圆周半径），V1、V2、V3、V4为行进轮的速度，θ1、θ2、θ3、θ4为舵轮的转向角。知道了这八个参数，就可以确定车体的运动。

[](https://blog.csdn.net/weixin_73037889/article/details/131052481)二、代码分享
-------------------------------------------------------------------------

### [](https://blog.csdn.net/weixin_73037889/article/details/131052481)1.Chassis\_steer\_app.c

代码如下:

```
#include "Chassis_steer_app.h"
#include "string.h"

ChassisHandle_t ChassisHandle;


void Chassis_steerAppConfig(void)
{
	ChassisHandle.console=Console_Pointer();
	ChassisHandle.imu=IMU_GetDataPointer();
	ChassisHandle.ctrl_mode = CHASSIS_RELAX;
	
	ChassisHandle.structure.wheel_perimeter = WHEEL_PERIMETER;       //轮子硬性参数
    ChassisHandle.structure.wheeltrack = WHEELTRACK;
    ChassisHandle.structure.wheelbase = WHEELBASE;
	ChassisHandle.structure.Radius = RADIUS;
	
	ChassisHandle.chassis_steer_motor[0].offset_ecd.chassis_offset_ecd =1726;
	ChassisHandle.chassis_steer_motor[1].offset_ecd.chassis_offset_ecd =20;
	ChassisHandle.chassis_steer_motor[2].offset_ecd.chassis_offset_ecd =7789;
	ChassisHandle.chassis_steer_motor[3].offset_ecd.chassis_offset_ecd =3370;
	
	ChassisHandle.steer_set[0].speed_direction=-1;                         
	ChassisHandle.steer_set[1].speed_direction=-1;
	ChassisHandle.steer_set[2].speed_direction=1;
	ChassisHandle.steer_set[3].speed_direction=1;
	
	for (uint8_t i=0; i<4; i++)      //底盘8电机PID控制
	{ 
    ChassisHandle.chassis_motor[i].motor_info = ChassisMotor_Pointer(i);
		ChassisHandle.chassis_steer_motor[i].motor_info=Chassis_steer_Motor_Pointer(i);
		ChassisHandle.chassis_steer_motor[i].ecd_ratio=STEER_MOTO_POSITIVE_DIR * 
    STEER_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;// 初始化舵机的转向系数
	  ChassisHandle.chassis_steer_motor[i].max_relative_angle = 180;
    ChassisHandle.chassis_steer_motor[i].min_relative_angle = -180;
		
		pid_init(&ChassisHandle.chassis_motor[i].pid, POSITION_PID, M3508_MOTOR_MAX_CURRENT, 3000.0f,
                 10.0f, 0.0f, 2.0f);
//		pid_init(&ChassisHandle.chassis_steer_motor[i].pid.outer_pid, POSITION_PID, 500.0f, 0.0f,
//                 20.0f, 0.0f, 4.0f);         // 20 0 4
//		pid_init(&ChassisHandle.chassis_steer_motor[i].pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 3000.0f,
//                 40.0f, 0.1f, 0.0f);      // 40   
		pid_init(&ChassisHandle.chassis_steer_motor[i].pid.outer_pid, POSITION_PID, 500.0f, 0.0f,
                 20.0f, 0.0f, 0.0f);         // 20 0 4
		pid_init(&ChassisHandle.chassis_steer_motor[i].pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 3000.0f,
                 10.0f, 0.0f, 0.0f);      // 40   
	}
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];
	/*CAN2接收*/
  if(hcan->Instance == CAN2)
  {
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data
  switch(rx_header.StdId)
	{
		/*行进轮*/
		case CHASSIS_MOTOR_LF_MESSAGE_ID :
	{
    Motor_EncoderData(ChassisHandle.chassis_motor[0].motor_info, rx_data);
		break;
	}
	
		case CHASSIS_MOTOR_RF_MESSAGE_ID:
	{
    Motor_EncoderData(ChassisHandle.chassis_motor[1].motor_info, rx_data);
		break;
	}
	
		case CHASSIS_MOTOR_LB_MESSAGE_ID:
	{
    Motor_EncoderData(ChassisHandle.chassis_motor[2].motor_info, rx_data);
		break;
	}
	
		case CHASSIS_MOTOR_RB_MESSAGE_ID:
	{
    Motor_EncoderData(ChassisHandle.chassis_motor[3].motor_info, rx_data);
		break;
	}
		
	/*转向轮*/
	  case CHASSIS_STEER_MOTOR_LF_MESSAGE_ID :
	{
    Motor_EncoderData(ChassisHandle.chassis_steer_motor[0].motor_info, rx_data);
		break;
	}
	
		case CHASSIS_STEER_MOTOR_RF_MESSAGE_ID:
	{
    Motor_EncoderData(ChassisHandle.chassis_steer_motor[1].motor_info, rx_data);
		break;
	}
	
		case CHASSIS_STEER_MOTOR_LB_MESSAGE_ID:
	{
    Motor_EncoderData(ChassisHandle.chassis_steer_motor[2].motor_info, rx_data);
		break;
	}
	
		case CHASSIS_STEER_MOTOR_RB_MESSAGE_ID:
	{
    Motor_EncoderData(ChassisHandle.chassis_steer_motor[3].motor_info, rx_data);
		break;
	}
		
	}
  }
	/*CAN1接收*/
	else if(hcan->Instance == CAN1)
  {
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data
  switch(rx_header.StdId)
	{
	  case CHASSIS_STEER_MOTOR_LF_MESSAGE_ID :
	{
    Motor_EncoderData(ChassisHandle.chassis_motor[0].motor_info, rx_data);
		break;
	}
	
		case CHASSIS_STEER_MOTOR_RF_MESSAGE_ID:
	{
    Motor_EncoderData(ChassisHandle.chassis_motor[1].motor_info, rx_data);
		break;
	}
	
		case CHASSIS_STEER_MOTOR_LB_MESSAGE_ID:
	{
    Motor_EncoderData(ChassisHandle.chassis_motor[2].motor_info, rx_data);
		break;
	}
	
		case CHASSIS_STEER_MOTOR_RB_MESSAGE_ID:
	{
    Motor_EncoderData(ChassisHandle.chassis_motor[3].motor_info, rx_data);
		break;
	}
		
	}
  }
}
```

首先，我们定义了ChassisHandle 的结构体，结构体内包含各个电机的信息、控制的模式、行进速度、舵机转角等，具体如下：

```
typedef struct
{
	Console_t*              console; //控制器模式
	IMU_Data_t*             imu; //IMU模块
	
	MechanicalStructure_t   structure; //底盘硬性参数
	ChassisMotor_t          chassis_motor[4]; //行进轮电机
	Chassis_steer_Motor_t   chassis_steer_motor[4]; //舵轮电机
	
	ChassisCtrlMode_e       ctrl_mode; //控制模式
	
	fp32                    vx; //Vx                      
    fp32                    vy; //Vy                   
    fp32                    vw; //Vw	
    fp32                    wheel_rpm[4]; //用于存储转速信息
	
	fp32                    steeringAngleTarget[4];//舵机的目标转角              
	fp32                    lastSteeringAngletarget[4]; //上一次的目标转角
	fp32                    steeringAngle[4]; //当前的舵机转角                     
	fp32                    last_steeringAngle[4]; //上一次的舵机转角
	int32_t                 motor_circle[4]; //电机转过的圈数
	int32_t                 motor_target_count[4]; //
	Steer_Type              steer_set[4]; //用于舵轮速度解算
	uint16_t                turnFlag[4];       //舵轮角度解算
	
	fp32                    chassis_yaw; //陀螺仪角度信息，下同  
    fp32                    chassis_pitch; 
    fp32                    chassis_roll;
	
	
}ChassisHandle_t;
```

app文件用于初始化底盘信息的结构体，并获取各模块的指针信息。HAL\_CAN\_RxFifo0MsgPendingCallback(CAN\_HandleTypeDef \*hcan) 为CAN的中断回调函数，接收电机编码器信息存入motor\_info中。详细可看我的上一篇博客[Robomaster电控组小白的学习经验分享（一）——用大疆C型开发板控制GM6020电机转动到既定角度\_STF故事的博客-CSDN博客](https://blog.csdn.net/weixin_73037889/article/details/130696750?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522168597264816800184183049%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=168597264816800184183049&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-5-130696750-null-null.142%5Ev88%5Econtrol_2,239%5Ev2%5Einsert_chatgpt&utm_term=Robomaster%E5%B0%8F%E7%99%BD&spm=1018.2226.3001.4187 "Robomaster电控组小白的学习经验分享（一）——用大疆C型开发板控制GM6020电机转动到既定角度_STF故事的博客-CSDN博客")

ChassisHandle.chassis\_steer\_motor\[0\].offset\_ecd.chassis\_offset\_ecd =1726;  ChassisHandle.chassis\_steer\_motor\[1\].offset\_ecd.chassis\_offset\_ecd =20;  
ChassisHandle.chassis\_steer\_motor\[2\].offset\_ecd.chassis\_offset\_ecd =7789;  
ChassisHandle.chassis\_steer\_motor\[3\].offset\_ecd.chassis\_offset\_ecd =3370; 

这四行代码用于初始化舵机的偏移量，使四个舵机无论怎样安装，都可以在开机时回到设定的初始位置，偏移量要根据自己舵轮底盘舵机的安装方位而改变。

### [](https://blog.csdn.net/weixin_73037889/article/details/131052481)2.Chassis\_steer\_function.c

```
#include "Chassis_steer_function.h"
#include "arm_math.h"
#include "math.h"

fp32 Chassis_Steer_PID_Calc(Chassis_steer_pid_t* pid, fp32 angle_ref, fp32 angle_fdb, fp32 speed_fdb)   //底盘舵电机PID（角度+速度）
{
    pid->angle_ref = angle_ref;
    pid->angle_fdb = angle_fdb;
    pid_calc(&pid->outer_pid, pid->angle_fdb, pid->angle_ref);//角度环
    pid->speed_ref = pid->outer_pid.out;
    pid->speed_fdb = speed_fdb;
    pid_calc(&pid->inter_pid, pid->speed_fdb, pid->speed_ref);//速度环
    return pid->inter_pid.out;
}

void Steer_Speed_Calculate(ChassisHandle_t* chassis_handle, fp32 chassis_vx, fp32 chassis_vy, fp32 chassis_vw) //舵轮轮速解算
{
    float theta = atan(1.0/1.0);                     //程序换算角度为弧度不能直接使用！！
    fp32 steer_vw=chassis_vw*3.14/180;
    float wheel_rpm_ratio;
	
    fp32 wheel_rpm[4];
    fp32 max = 0;
	
	/*M3508_REDUCTION_RATIO为M3508减速比（1.0/19.0）
		*/
	wheel_rpm_ratio = 60.0f/(chassis_handle->structure.wheel_perimeter * M3508_REDUCTION_RATIO);    //舵轮转速转换
	
	wheel_rpm[0]
			  = sqrt(pow(chassis_vx - steer_vw*RADIUS*sin(theta),2)                
			  + pow(chassis_vy - steer_vw*RADIUS*cos(theta),2))* 
				wheel_rpm_ratio*chassis_handle->steer_set[0].speed_direction;        //对应华南理工公式  Vx1-Vw1sin45*RADIUS    
			
			wheel_rpm[1]
			= sqrt(	pow(chassis_vx - steer_vw*RADIUS*sin(theta),2)
						+	pow(chassis_vy + steer_vw*RADIUS*cos(theta),2))* 
	          wheel_rpm_ratio*chassis_handle->steer_set[1].speed_direction;      	 //对应华南理工公式  Vx2+Vw2sin45*RADIUS  																																																																								
			
			wheel_rpm[2]
			= sqrt(	pow(chassis_vx + steer_vw*RADIUS*sin(theta),2)
						+	pow(chassis_vy + steer_vw*RADIUS*cos(theta),2))* 
						wheel_rpm_ratio*chassis_handle->steer_set[2].speed_direction;        //对应华南理工公式  Vy1-Vw1cos45*RADIUS  
																																																																																
			wheel_rpm[3]
			= sqrt(	pow(chassis_vx + steer_vw*RADIUS*sin(theta),2)
						+	pow(chassis_vy - steer_vw*RADIUS*cos(theta),2))*
						wheel_rpm_ratio*chassis_handle->steer_set[3].speed_direction;        //对应华南理工公式  Vy2+Vw2cos45*RADIUS 
	
			//find max item
			for (uint8_t i = 0; i < 4; i++)
			{
				
					if(chassis_handle->turnFlag[i]==1)
					{
						wheel_rpm[i] = -wheel_rpm[i];
					}
					else
					{
						wheel_rpm[i] = wheel_rpm[i];
					}
					
					if (fabs(wheel_rpm[i]) > max)
					{
							max = fabs(wheel_rpm[i]);
					}
			}
			
			memcpy(chassis_handle->wheel_rpm, wheel_rpm, 4 * sizeof(fp32));
		}

void Steer_angle_change(ChassisHandle_t* chassis_handle, fp32 chassis_vx, fp32 chassis_vy, fp32 chassis_vw)  
{
	float theta = atan(1.0/1.0);
	fp32 wheel_angle[4];      
	fp32 steer_vw=chassis_vw*3.14/180;
	if((chassis_vx==0)&&(chassis_vy==0)&&(chassis_vw==0))              
	{
		memcpy(wheel_angle, chassis_handle->lastSteeringAngletarget, 4 * sizeof(fp32));
	}
	else      //舵轮角度解算       
	{
		wheel_angle[0]=atan2((chassis_vy-steer_vw*RADIUS*sin(theta)),
							(chassis_vx-steer_vw*RADIUS*cos(theta)));       //对应华南理工公式VX=tan-1((vy1-vw1cos45)/(vx1-vw1sin45))
		wheel_angle[1]=atan2((chassis_vy+steer_vw*RADIUS*sin(theta)),
							(chassis_vx-steer_vw*RADIUS*cos(theta)));       //对应华南理工公式VX=tan-1((vy2-vw2cos45)/(vx2+vw2sin45)) 
		wheel_angle[2]=atan2((chassis_vy+steer_vw*RADIUS*sin(theta)),
							(chassis_vx+steer_vw*RADIUS*cos(theta)));       //对应华南理工公式VX=tan-1((vy3+vw3cos45)/(vx3+vw3sin45)) 
		wheel_angle[3]=atan2((chassis_vy-steer_vw*RADIUS*sin(theta)),
							(chassis_vx+steer_vw*RADIUS*cos(theta)));       //对应华南理工公式VX=tan-1((vy4+vw4cos45)/(vx4-vw4sin45)) 
			 
		for(uint8_t i=0;i<4;i++)                                                
		{
			if(wheel_angle[i]-chassis_handle->lastSteeringAngletarget[i]>PI/2)
			{
				wheel_angle[i]=fmodf(wheel_angle[i]-PI,2*PI);
				chassis_handle->turnFlag[i]=1;
			}
			else if(wheel_angle[i]-chassis_handle->lastSteeringAngletarget[i]<-PI/2)
			{
				wheel_angle[i]=fmodf(wheel_angle[i]+PI,2*PI);
				chassis_handle->turnFlag[i]=1;
			}
			else
			{
				chassis_handle->turnFlag[i]=0;
			}		
		}

		 }
				 
	memcpy(chassis_handle->lastSteeringAngletarget, wheel_angle, 4 * sizeof(fp32));
	for(uint8_t i=0;i<4;i++)
	{
		wheel_angle[i]=wheel_angle[i]*180/PI;       
	}		 
	 memcpy(chassis_handle->steeringAngleTarget, wheel_angle, 4 * sizeof(fp32));  
}

void Steer_Calculate(ChassisHandle_t* chassis_handle, fp32 chassis_vx, fp32 chassis_vy, fp32 chassis_vw)   //舵轮底盘解算
{	
	Steer_angle_change(chassis_handle,chassis_vx,chassis_vy,chassis_vw);    
	Steer_Speed_Calculate(chassis_handle,chassis_vx,chassis_vy,chassis_vw);
}

void Steer_Chassis_ControlCalc(ChassisHandle_t* chassis_handle)      
{
    Steer_Calculate(chassis_handle, chassis_handle->vx, chassis_handle->vy, chassis_handle->vw);
}

void calculateRoundCnt(ChassisHandle_t* chassis_handle)    
{
	static float last_encoder[4] = {0};
	for(uint8_t i=0;i<4;i++)
	{
		float now_encoder = chassis_handle->chassis_steer_motor[i].motor_info->ecd
						  -	chassis_handle->chassis_steer_motor[i].offset_ecd.chassis_offset_ecd;
		now_encoder = now_encoder/8192*360;
		
		if(now_encoder - last_encoder[i] > 180)
			chassis_handle->motor_circle[i] --;
		else if(now_encoder - last_encoder[i] < -180) 
			chassis_handle->motor_circle[i] ++;
		
		last_encoder[i] = now_encoder;
		chassis_handle->steeringAngle[i] = -now_encoder -  chassis_handle->motor_circle[i]*360;
		chassis_handle->chassis_steer_motor[i].sensor.relative_angle=chassis_handle->steeringAngle[i]*-1;
	}
}

void calculateTargetRoundCnt(ChassisHandle_t* chassis_handle)   
{
	float now_target[4];
	static float last_target[4]={0,0,0,0};
	memcpy(now_target, chassis_handle->steeringAngleTarget, 4 * sizeof(fp32));
	
	for(uint8_t i=0;i<4;i++)
	{
		if(now_target[i] - last_target[i] > 180) 
			chassis_handle->motor_target_count[i] --;
		else if(now_target[i] - last_target[i]  < -180) 
			chassis_handle->motor_target_count[i] ++;
		
		last_target[i] = now_target[i];
		chassis_handle->steeringAngleTarget[i] = now_target[i] + chassis_handle->motor_target_count[i]*360 ;
	}
}
```

function文件用于舵轮的姿态解算以及其他的底层函数 。以上代码参考了华南理工的舵轮开源

[RM\_scut\_infantry: RM2021-华南理工大学-普渡华南虎战队-舵轮步兵代码](https://gitee.com/hhrccc/rm_scut_infantry "RM_scut_infantry: RM2021-华南理工大学-普渡华南虎战队-舵轮步兵代码")

Chassis\_Steer\_PID\_Calc函数为双环PID的解算函数；Steer\_Speed\_Calculate函数为行进轮转速的解算函数，即求Vx1、Vx2、Vx3、Vx4，然后将计算结果复制到ChassisHandle结构体中的wheel\_rpm\[4\]；Steer\_angle\_change函数用于解算舵机的转角（θ1、θ2、θ3、θ4），然后复制到ChassisHandle结构体中的steeringAngleTarget\[4\]数组。

calculateRoundCnt函数用于计算当前舵机的相对角度，将编码器的值转换为以度为单位的角度值，存入ChassisHandle结构体中的steeringAngle\[4\]；calculateTargetRoundCnt函数是对ChassisHandle结构体中steeringAngleTarget\[4\]数组中的数据进行处理，跟踪每个电机旋转的次数，并相应地调整转向角度目标值。

### 3.Chassis\_steer\_function.c

```
#include "Chassis_steer_task.h"
#include "Chassis_steer_app.h"
#include "Chassis_steer_function.h"
#include "cmsis_os.h"
#include "bsp_can.h"

extern ChassisHandle_t ChassisHandle;

static void ChassisSensorUpdata(void);
void ChassisCtrlModeSwitch(void);
static void ChassisStopMode(void);
static void ChassisTestMode(void);
static void ChassisSeparateGimbalMode(void);
static void ChassisSpinMode(void);

void Chassis_Task(void const * argument)
{
  for(;;)
  {
		ChassisSensorUpdata();
		ChassisCtrlModeSwitch();
		switch(ChassisHandle.ctrl_mode)
		{
			case CHASSIS_SEPARATE_GIMBAL:
			{
				ChassisSeparateGimbalMode();
			}
			break;
			
			case CHASSIS_SPIN:
			{
				ChassisSpinMode();
			}
			break;
			
			case CHASSIS_TEST:
			{
				ChassisTestMode();
			}
			break;
			
			case CHASSIS_STOP:
			{
				ChassisStopMode();
			}
			break;
			
			default:
				break;
			
		}
		
		
		calculateRoundCnt(&ChassisHandle);
		calculateTargetRoundCnt(&ChassisHandle);
		Steer_Chassis_ControlCalc(&ChassisHandle);
		
		for (uint8_t i = 0; i < 4; i++)             //PID控制
    {
      ChassisHandle.chassis_motor[i].given_speed = ChassisHandle.wheel_rpm[i];
      ChassisHandle.chassis_motor[i].current_set = pid_calc(&ChassisHandle.chassis_motor[i].pid,
                                                             ChassisHandle.chassis_motor[i].motor_info->speed_rpm,
                                                             ChassisHandle.chassis_motor[i].given_speed);
			
			ChassisHandle.chassis_steer_motor[i].given_value=ChassisHandle.steeringAngleTarget[i];
			ChassisHandle.chassis_steer_motor[i].current_set = Chassis_Steer_PID_Calc(&ChassisHandle.chassis_steer_motor->pid, 
			                                                                          ChassisHandle.steeringAngleTarget[i],
			                                                                          ChassisHandle.chassis_steer_motor->sensor.relative_angle, 
			                                                                          ChassisHandle.chassis_steer_motor[i].sensor.palstance);//ChassisHandle.imu->gyro[2]*RAD_TO_ANGLE
    }                                                                                                                    //ChassisHandle.chassis_steer_motor->motor_info->speed_rpm
		
		if(ChassisHandle.ctrl_mode == CHASSIS_RELAX)     
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                ChassisHandle.chassis_motor[i].current_set = 0;
						  	ChassisHandle.chassis_steer_motor[i].current_set = 0;
            }
        }
				
		 Motor_SendMessage(&hcan2,MOTOR_1TO4_CONTROL_STD_ID,ChassisHandle.chassis_motor[0].current_set,
                                ChassisHandle.chassis_motor[1].current_set,
                                ChassisHandle.chassis_motor[2].current_set,
                                ChassisHandle.chassis_motor[3].current_set);       //轮输出
				
	  Motor_SendMessage(&hcan2,MOTOR_5TO8_CONTROL_STD_ID,ChassisHandle.chassis_steer_motor[0].current_set, 
									ChassisHandle.chassis_steer_motor[1].current_set, 
									ChassisHandle.chassis_steer_motor[2].current_set,
									ChassisHandle.chassis_steer_motor[3].current_set);   //舵输出
		
    osDelay(10);
  }
	
}

static void ChassisSensorUpdata(void)    //传感器数据更新
{
	 for(uint8_t i=0;i<4;i++)
	  {
		   ChassisHandle.chassis_steer_motor[i].sensor.palstance=ChassisHandle.chassis_steer_motor[i].motor_info->speed_rpm*6;	//更新四个电机的角速度值
		}
	ChassisHandle.chassis_pitch=ChassisHandle.imu->attitude.pitch;
	ChassisHandle.chassis_roll=ChassisHandle.imu->attitude.roll;
	ChassisHandle.chassis_yaw=ChassisHandle.imu->attitude.yaw;
}

void ChassisCtrlModeSwitch(void)
{
	if(ChassisHandle.console->chassis_cmd == CHASSIS_RELEASE_CMD)
	{
		ChassisHandle.ctrl_mode = CHASSIS_RELAX;
	}
	
	else if(ChassisHandle.console->chassis_cmd == CHASSIS_FOLLOW_GIMBAL_CMD)
	{
		ChassisHandle.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
	}
	
	else if(ChassisHandle.console->chassis_cmd == CHASSIS_SEPARATE_GIMBAL_CMD)
	{
		ChassisHandle.ctrl_mode = CHASSIS_SEPARATE_GIMBAL;
	}
	
	else if(ChassisHandle.console->chassis_cmd == CHASSIS_SPIN_CMD)
	{
		ChassisHandle.ctrl_mode = CHASSIS_SPIN;
	}
	
	else if(ChassisHandle.console->chassis_cmd == CHASSIS_TEST_CMD)
	{
		ChassisHandle.ctrl_mode = CHASSIS_TEST;
	}
	
	else if (ChassisHandle.console->chassis_cmd == CHASSIS_STOP_CMD)
  {
    ChassisHandle.ctrl_mode = CHASSIS_STOP;
	}
}
	

static void ChassisStopMode(void)//停止模式
{
   ChassisHandle.vx=0;
	 ChassisHandle.vy=0;
	 ChassisHandle.vw=0;
}

static void ChassisTestMode(void)//底盘测试模式VX,VY,VW测试
{
   ChassisHandle.vx=ChassisHandle.console->chassis.vx;
	 ChassisHandle.vy=ChassisHandle.console->chassis.vy;
	 ChassisHandle.vw=ChassisHandle.console->chassis.vw;
}
	
static void ChassisSeparateGimbalMode(void)//分离模式
{
	
	 ChassisHandle.vx = ChassisHandle.console->chassis.vx;
	 ChassisHandle.vy = ChassisHandle.console->chassis.vy;
	 ChassisHandle.vw=ChassisHandle.console->chassis.vw;
}	

static void ChassisSpinMode(void)//小陀螺模式
{
	
	ChassisHandle.vw = 160;//160为初始值（详细可见infantry_console.c中的小陀螺模式切换，会在160的基础上再乘以spin_rate）
	/*在除keyboard2模式下，vw的值在infantry_console.c中都没有过赋值，因此在此处自己赋值*/
  ChassisHandle.vx = ChassisHandle.console->chassis.vx;
	ChassisHandle.vy = ChassisHandle.console->chassis.vy;	
	

}
```

最后写freeRTOS的任务函数（各种模式的选择，PID运算以及CAN发送等）。

* * *

[](https://blog.csdn.net/weixin_73037889/article/details/131052481)总结
---------------------------------------------------------------------

        以上便是本人对舵轮底盘的学习分享，自己加入嵌入式组也仅仅只有一年，还有很多要学习的地方。文章中有什么错误的、不合理的地方也希望大家能批评指正。写这篇博客旨在对自己近期的学习进行总结，也希望能帮助到大家。代码中还有很多需要改善的地方。由于机械组还未做出舵轮底盘，所以以下的视频仅仅只是电机的测试。

舵轮电机测试

  

本文转自 [https://blog.csdn.net/weixin\_73037889/article/details/131052481](https://blog.csdn.net/weixin_73037889/article/details/131052481)，如有侵权，请联系删除。