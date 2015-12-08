//=========================================================================================================
//===亲爱的羽叶沁馨(同济大学):
//===感谢你选择圆点博士小四轴,并承诺遵守源代码保密协议(NDA)。你的鼓励和支持是我们前进的动力！==============
//---------------------------------------------------------------------------------------------------------
//===Dear 羽叶沁馨(同济大学):
//===Thanks to select Dr.R&D quad-aircraft, and agree with the source code NDA(Non-Disclosure Agreement)===
//===Your ecourange and support are greatly appreciated!===================================================
//=========================================================================================================

/*************************************************************************************************************
圆点博士小四轴飞行器2014版配套源代码声明:
该源代码仅供参考,圆点博士不对源代码提供任何形式的担保,也不对因使用该源代码而出现的损失负责.
用户可以以学习的目的修改和使用该源代码.
但用户在修改该源代码时,不得移除该部分版权信息，必须保留原版声明.

更多信息，请访问官方网站www.etootle.com, 官方博客:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include "etootle_main.h"
//依次为 系统空闲计数 系统1ms事件标志 系统计数器状态标志 系统状态缓存
unsigned int system_idle_counter=0,system_timer_1ms_event=0,system_timer_state=0,system_status=0;
//依次为 系统计数器计数值 系统LED计数器计数值
unsigned int system_timer_counter=0,system_led_timer_counter=0;
//接收到的命令
extern unsigned char bs004_com_command_ops;
//串口空闲标志
extern unsigned char com_status_is_idle;	
//依次为 油门控制参数 控制参数有效标志
extern unsigned char BS004_Ctrl_Gas,BS004_Ctrl_Valid;
//依次为 俯仰控制参数 横滚控制参数 航向控制参数
extern signed char   BS004_Ctrl_Pitch,BS004_Ctrl_Roll,BS004_Ctrl_Yaw;
//依次为 四个电机的油门参数以及油门参数的控制比例
extern unsigned int Motor_BS004_M4,Motor_BS004_M2,Motor_BS004_M3,Motor_BS004_M1,BS004_Motor_Scale;
//
int main()
{
	BS004_RCC_Configuration();				//圆点博士:时钟设置 设定系统时钟
	BS004_NVIC_Configuration();     	//圆点博士:中断设置 初始化系统中断
	//
	BS004_SYS_LED_Configuration();    //圆点博士:LED设置	初始化系统LED
	BS004_SYS_Timer_Configuration();  //圆点博士:系统时间设置	初始化系统计数器
	BS004_LED_GPIO_Configuration();   //圆点博士:LED设置	初始化LED
	//
	BS004_COM1_GPIO_Configuration();	//圆点博士:串口设置	初始化串口所使用的端口
	BS004_COM1_Port_Configuration();  //圆点博士:串口设置	初始化串口
	BS004_ADC_Configuration();        //圆点博士:电压检测设置	初始化ADC
	//
	BS004_MPU6050_Init();             //圆点博士:MPU6050初始化
	BS004_Load_Fly_Parameter();			  //圆点博士:装载参数
	BS004_Show_Calibrated_Data();     //圆点博士:检查校验后的MPU6050数据
	//
	BS004_Motor_GPIO_Configuration(); //圆点博士:电机设置	初始化电机输出端口
	BS004_Motor_PWM_Configuration(); 	//圆点博士:电机设置	初始化电机PWM输出
	//
	//状态机循环	由于程序不能超过main函数，状态机在循环中运行
	while(1)
	{
		if(system_timer_1ms_event)			//圆点博士:1MS事件触发 1ms时间标志，该标志位每1ms置1一次，即1ms进入状态机一次
		{	
			system_timer_1ms_event=0;			//圆点博士:清除1ms事件标志
			system_timer_counter++;       //圆点博士:系统计数器自加
			system_led_timer_counter++;   //圆点博士:LED灯闪烁计数器自加
			//根据状态机的状态执行相应动作
			switch (system_timer_state)
			{
				case IDLE:									//圆点博士:空闲状态 开机后的初始化状态，等待传感器数据稳定
					system_idle_counter++;		//空闲计数器自加
					BS004_Get_MPU6050_Data();	//获取传感器数据
					//空闲5次即开机5ms后，状态机状态更新为：从串口更新指令
					if(system_idle_counter>5) system_timer_state=UPDATE_COMMAND;
					break;
					//从串口更新指令
				case UPDATE_COMMAND:				//圆点博士:读取MPU6050数据和串口数据
					BS004_Get_MPU6050_Data();	//获取传感器数据
					system_status=BS004_COM1_Task_Process();	//串口获取从串口更新命令
					//如果油门大于0，状态机更新为：UPDATE_MOTOR
					if(system_status) system_timer_state=UPDATE_MOTOR;
					//如果油门为0，状态机更新为：RESET_MOTOR
					else system_timer_state=RESET_MOTOR;
					break;

				case UPDATE_MOTOR:					//圆点博士:更新电机输出
					BS004_Get_MPU6050_Data();	//获取传感器数据
					BS004_Quad_Calculation();	// 四元数计算
				  BS004_PID_Control();			//PID控制计算
					BS004_Motor_Control();		//电机输出
					//状态机状态更新：从串口更新命令
					system_timer_state=UPDATE_COMMAND;
					break;

				case RESET_MOTOR:          //圆点博士:复位电机输出
					BS004_Get_MPU6050_Data();	//获取传感器数据
				  BS004_Quad_Calculation();	//四元数计算
					BS004_Motor_Reset();			//电机输出重置
					//状态机状态更新为：从串口更新命令
					system_timer_state=UPDATE_COMMAND;
					break;
					//其他情况则跳出
				default:
					break;
			}
		}
		else
		{			
			BS004_SYS_LED_TWINKLE();   //圆点博士:LED灯闪烁
		}
	}	
}
//
void BS004_Quad_Calculation(void)
{
	float ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
	//依次为x,y,z轴的加速度缓存和x,y,z轴的角速度缓存
	bs004_gyro_to_rad_scale=bs004_mpu6050_pi_scale*bs004_mpu6050_gyro_scale;
	//	陀螺仪转弧度比例系数=弧度系数（实际值为57.3f(57.3/rad))*陀螺仪灵敏度(实际值为16.4(陀螺仪单位))
	
	//MPU6050滤波后的数据/陀螺仪转弧度比例系数    实现了将角速度(陀螺仪)转换成弧速度
	gx=bs004_mpu6050_gyro_angel_pitch_ave/bs004_gyro_to_rad_scale;
	gy=bs004_mpu6050_gyro_angel_roll_ave/bs004_gyro_to_rad_scale;
	gz=bs004_mpu6050_gyro_angel_yaw_ave/bs004_gyro_to_rad_scale;		
	//MPU6050滤波后的加速度值
	ax=bs004_mpu6050_acc_angel_roll_ave;
	ay=bs004_mpu6050_acc_angel_pitch_ave;	
	az=bs004_mpu6050_acc_angel_yaw_ave;		
	//更新惯性测量单元的数据即计算姿态
	BS004_IMU_Update(ax,ay,az,gx,gy,gz);
}
//串口事件处理过程
unsigned char BS004_COM1_Task_Process(void)
{
	//如果接收到的命令代码大于0xA0
	if(bs004_com_command_ops>0xA0)
	{
		com_status_is_idle=0;					//圆点博士:设置串口忙标志
		BS004_COM1_Communication();   //圆点博士:执行命令 调用串口通信回应函数
		bs004_com_command_ops=0;			//圆点博士:清空命令操作标志
		com_status_is_idle=1;					//圆点博士:清除串口忙标志
	} 
	else if((bs004_com_command_ops==0xA0))//  respond to gas 写入遥控数据
	{
		if(BS004_Ctrl_Valid==1)					//圆点博士:检查控制数据是否有效 如果控制信号标志有效
		{	
			com_status_is_idle=0;					//圆点博士:设置串口忙标志			
			//如果油门控制参数 大于 油门控制阈值，则实际油门为 油门控制参数减去油门控制阈值，反之实际油门为0			
			if(BS004_Ctrl_Gas>BS004_Ctrl_Gas_Noise) bs004_fly_gas=BS004_Ctrl_Gas-BS004_Ctrl_Gas_Noise;
			else bs004_fly_gas=0;
			//如果俯仰控制参数大于姿态控制阈值，则实际俯仰为 俯仰控制参数 减去姿态控制阈值， 反之实际俯仰为0
			if(fabs(BS004_Ctrl_Pitch)>BS004_Ctrl_Dir_Noise) bs004_fly_pitch=BS004_Ctrl_Pitch;
			else bs004_fly_pitch=0;
			//如果控制参数大于姿态控制阈值，则实际横滚为 横滚控制参数 减去姿态控制阈值， 反之实际横滚为0
			if(fabs(BS004_Ctrl_Roll)>BS004_Ctrl_Dir_Noise) bs004_fly_roll=BS004_Ctrl_Roll;
			else bs004_fly_roll=0;
			//如果偏航控制参数大于姿态控制阈值，则实际偏航为 偏航控制参数 减去姿态控制阈值， 反之实际偏航为0
			if(fabs(BS004_Ctrl_Yaw)>BS004_Ctrl_Dir_Noise) bs004_fly_yaw=BS004_Ctrl_Yaw;
			bs004_fly_yaw=0;
			//
			BS004_Ctrl_Valid=0;					//圆点博士:清除控制信号标志
			com_status_is_idle=1;				//圆点博士:清除串口忙标志
		}
	}	
	return bs004_fly_gas;		//返回油门大小
}
//电机输出控制函数
void BS004_Motor_Control(void)
{
	//对电机输出PWM占空比进行比例换算
	bs004_fly_m1_out=BS004_Motor_Speed_Scale(bs004_fly_m1);
	bs004_fly_m2_out=BS004_Motor_Speed_Scale(bs004_fly_m2);	
	bs004_fly_m3_out=BS004_Motor_Speed_Scale(bs004_fly_m3);
	bs004_fly_m4_out=BS004_Motor_Speed_Scale(bs004_fly_m4);	
	//更新PWM占空比
	TIM_SetCompare4(TIM8,bs004_fly_m1_out);		//圆点博士:更新PWM占空比  
	TIM_SetCompare2(TIM8,bs004_fly_m2_out);	  //圆点博士:更新PWM占空比  
	TIM_SetCompare1(TIM8,bs004_fly_m3_out);   //圆点博士:更新PWM占空比      
	TIM_SetCompare3(TIM8,bs004_fly_m4_out);		//圆点博士:更新PWM占空比 
}
//串口发棕欧拉角姿态数据
void ANBT_SEND_DMP_EULER_DATA(void)
{
	//依次为俯仰显示数据，横滚显示数据，偏航显示数据
	float bs004_display_pitch=0,bs004_display_roll=0,bs004_display_yaw=0;
	//依次为数据包类型 校验和 欧拉角符号标志 循环数据计数
	unsigned char data_type,checksum=0,euler_data_sign=0,i=0;
	//欧拉角数据缓存
	unsigned int bs004_mpu6050_euler_data[3];
	//欧拉角数据发送缓存区
	unsigned char bs004_mpu6050_euler_data_buffer[6];
	//俯仰显示数据=获取到的IMU俯仰数据的100倍	(避免发送浮点数)
	bs004_display_pitch=bs004_imu_pitch*100;
	bs004_display_roll =bs004_imu_roll*100;
	bs004_display_yaw  =bs004_imu_yaw*100;
	
	//	如果俯仰显示数据 小于0
	if(bs004_display_pitch<0) 
	{
		//将欧拉角数据符号标志的第二位置1(00000100),表示俯仰显示数据为负数
		euler_data_sign|=4;
		//对数据进行+18000的偏移，(避免发送负数)
		bs004_display_pitch+=18000;
	}
	if(bs004_display_roll<0) 
	{
		//将欧拉角数据符号标志的第一位置1(00000010),表示横滚显示数据为负数
		euler_data_sign|=2;	
		bs004_display_roll+=18000;
	}
	if(bs004_display_yaw<0) 
	{
		//将欧拉角数据符号标志的第零位置1(00000001),表示偏航显示数据为负数
		euler_data_sign|=1;		
		bs004_display_yaw+=18000;
	}
	//euler_data_sign存储了欧拉角数据的符号位
	
	//将俯仰 横滚 偏航数据放入了欧拉角数据缓存
	bs004_mpu6050_euler_data[0]=(unsigned int)bs004_display_pitch;
	bs004_mpu6050_euler_data[1]=(unsigned int)bs004_display_roll;	
	bs004_mpu6050_euler_data[2]=(unsigned int)bs004_display_yaw;	
	
	//实现了对16位欧拉角数据进行拆分并存入欧拉角数据发送缓存区	排列如下
	//[俯仰显示数据高八位][俯仰显示数据低八位][横滚显示数据高八位][横滚显示数据低八位][偏航显示数据高八位][偏航显示数据低八位]
  for(i=0;i<3;i++) 
	{
		bs004_mpu6050_euler_data_buffer[i*2]=(bs004_mpu6050_euler_data[i]>>8)&0xff;
		bs004_mpu6050_euler_data_buffer[i*2+1]=bs004_mpu6050_euler_data[i]&0xff;
	}
	//将欧拉角数据符号标志高四位设为0xB0
	data_type=0xB0| euler_data_sign;
	//计算校验和
	checksum=data_type;
	for(i=0;i<6;i++) checksum+=bs004_mpu6050_euler_data_buffer[i];
	checksum&=0xff;
	checksum=~checksum;
	checksum++;
	//发送欧拉角数据
	//如果发送数据寄存器空标志位有效即发送缓冲区为空
	if(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==SET)
	{
		BS004_COM1_Send_Char(':');		//发送数据头
		BS004_COM1_Send_Num(data_type);	//发送数据包类型(包括欧拉角符号)
		//发送欧拉角数据发送区数据
		for(i=0;i<6;i++) BS004_COM1_Send_Num(bs004_mpu6050_euler_data_buffer[i]);			
		//发送和校验数据
		BS004_COM1_Send_Num(checksum);
		//发送数据尾
		BS004_COM1_Send_Char('/');
		//发送换行符
		BS004_COM1_Send_Char('\n');
	}
}



