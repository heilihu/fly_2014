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
#include "etootle_motor.h" 
//电机参数
extern float bs004_fly_m1,bs004_fly_m2,bs004_fly_m3,bs004_fly_m4;
//电机输出
signed short bs004_fly_m1_out=0,bs004_fly_m2_out=0,bs004_fly_m3_out=0,bs004_fly_m4_out=0;	
//当前俯仰角，横滚角，偏航角
extern float bs004_angle_cur_pitch,bs004_angle_cur_roll,bs004_angle_cur_yaw;
//上次俯仰角，横滚角，偏航角
extern float bs004_angle_last_pitch,bs004_angle_last_roll,bs004_angle_last_yaw;
//电机周期
unsigned int bs004_motor_pwm_period=0;
//油门参数的缩放比例
unsigned int BS004_Motor_Scale=0;
//四个电机的油门参数
unsigned int Motor_BS004_M1=0,Motor_BS004_M2=0,Motor_BS004_M3=0,Motor_BS004_M4=0;
//打开电机的电源
void BS004_Motor_Power_On(void)
{
	GPIO_SetBits(BS004_MOTOR_POWER_MA_PORT, BS004_MOTOR_POWER_MA);		//圆点博士:打开电机电源
	GPIO_SetBits(BS004_MOTOR_POWER_MB_PORT, BS004_MOTOR_POWER_MB);    //圆点博士:打开电机电源
}
//关闭电机的电源
void BS004_Motor_Power_Off(void)
{
	GPIO_ResetBits(BS004_MOTOR_POWER_MA_PORT, BS004_MOTOR_POWER_MA);	//圆点博士:关闭电机电源
	GPIO_ResetBits(BS004_MOTOR_POWER_MB_PORT, BS004_MOTOR_POWER_MB);	//圆点博士:关闭电机电源
}
//打开电机的PWM输出
void BS004_MOTOR_PWM_ON(void)		
{
	GPIO_SetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M1);			//圆点博士:打开PWM输出
	GPIO_SetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M2);			//圆点博士:打开PWM输出
	GPIO_SetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M3);			//圆点博士:打开PWM输出
	GPIO_SetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M4);			//圆点博士:打开PWM输出
}
//关闭电机的PWM输出
void BS004_MOTOR_PWM_OFF(void)		
{
	GPIO_ResetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M1);		//圆点博士:关闭PWM输出
	GPIO_ResetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M2);		//圆点博士:关闭PWM输出
	GPIO_ResetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M3);		//圆点博士:关闭PWM输出
	GPIO_ResetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M4);		//圆点博士:关闭PWM输出
}

//===============================================================
//PWM的IO口配置，四个电机引脚为M1-PC9,M2-PC7,M3-PC6,M4-PC8
void BS004_Motor_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	//使能GPIO端口B和C时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //圆点博士:设置PWM口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //圆点博士:设置PWM口为输出
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure); 
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //圆点博士:设置PWM口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //圆点博士:设置PWM口为输出
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //圆点博士:设置PWM口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //圆点博士:设置PWM口为输出
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //圆点博士:设置PWM口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //圆点博士:设置PWM口为输出
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//	
	BS004_MOTOR_PWM_OFF();															//圆点博士:设置PWM口输出为低
	//设置点击电源控制引脚 MA-PB9，MB-PC15
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_POWER_MA;					
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //圆点博士:设置电机电源控制口为输出
  GPIO_Init(BS004_MOTOR_POWER_MA_PORT, &GPIO_InitStructure); 
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_POWER_MB;					
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //圆点博士:设置电机电源控制口为输出
  GPIO_Init(BS004_MOTOR_POWER_MB_PORT, &GPIO_InitStructure); 
	//
	BS004_Motor_Power_Off();														//圆点博士:关闭电机电源
}
//配置PWM信号的工作模式，如周期，极性，占空比等
void BS004_Motor_PWM_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
	//使能TIM8的系统时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //圆点博士:设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//圆点博士:设置LED口为输出
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure); 
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //圆点博士:设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//圆点博士:设置LED口为输出
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //圆点博士:设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//圆点博士:设置LED口为输出
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //圆点博士:设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//圆点博士:设置LED口为输出
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//	
	//圆点博士:PWM频率=BS004_MOTOR_PWM_CLK_36MHZ/（BS004_MOTOR_PWM_PERIOD+1)
	//设置时钟分频系数，这里不分配
	TIM_TimeBaseStructure.TIM_ClockDivision = BS004_MOTOR_PWM_SYSCLK_DIV;
	//设置PWM的预分频系数
	TIM_TimeBaseStructure.TIM_Prescaler = BS004_MOTOR_PWM_CLK_72MHZ;
	//设置PWM计数模式，这里为向上计数
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	//设置在下一个更新时间装入活动的自动重装载寄存器周期值
	TIM_TimeBaseStructure.TIM_Period = bs004_motor_pwm_period; 
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);								//圆点博士:设置PWM周期和频率		
	
	//配置为定时器模式1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	//选择输出定时器的状态
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//设置通道1的电平跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_Pulse = Motor_BS004_M3;
	//设置PWM的输出极性
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	//设置互补滤波极性
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High; 
	//设置空闲状态下的非工作状态	
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	//选择互补空闲状态下的非工作状态
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OC1Init(TIM8, &TIM_OCInitStructure);											//圆点博士:设置PWM占空比		
	
	//选择输出定时器的状态
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//设置通道2的电平跳变值
  TIM_OCInitStructure.TIM_Pulse = Motor_BS004_M2;
  TIM_OC2Init(TIM8, &TIM_OCInitStructure);											//圆点博士:设置PWM占空比		
	//选择输出定时器的状态
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//设置通道3额电平跳变值
  TIM_OCInitStructure.TIM_Pulse = Motor_BS004_M4;
  TIM_OC3Init(TIM8, &TIM_OCInitStructure);											//圆点博士:设置PWM占空比		
	//设置通道4的电平跳变值
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Motor_BS004_M1;
  TIM_OC4Init(TIM8, &TIM_OCInitStructure);											//圆点博士:设置PWM占空比		
	//
	BS004_Motor_NVIC_Configuration();					//圆点博士:设置PWM中断优先级	
	TIM_Cmd(TIM8, ENABLE);										//圆点博士:启动PWM
	TIM_CtrlPWMOutputs(TIM8,ENABLE);          //圆点博士:允许PWM输出
	
	BS004_COM1_Send_Str_Head();								//发送命令的头文件
	BS004_COM1_Send_Str_Body("finish to init motor device.");					//圆点博士:初始化PWM IO
	BS004_COM1_Send_Str_Tail();								//发送命令的尾部
}

//配置中断函数
void BS004_Motor_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;			//圆点博士:设置PWM中断优先级	
	//
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQChannel;		//TIM8中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		//抢占优先级0级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					//子优先级0级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//TIM8中断使能
  NVIC_Init(&NVIC_InitStructure);
}

//===============================================================
void BS004_Motor_Interrupt(void)					//圆点博士:PWM中断函数
{	
	TIM_SetCompare4(TIM8,Motor_BS004_M1);		//圆点博士:更新PWM占空比  
	TIM_SetCompare2(TIM8,Motor_BS004_M2);	  //圆点博士:更新PWM占空比  
	TIM_SetCompare1(TIM8,Motor_BS004_M3);   //圆点博士:更新PWM占空比      
	TIM_SetCompare3(TIM8,Motor_BS004_M4);		//圆点博士:更新PWM占空比  	    
}
//限制油门参数
signed short BS004_Motor_Speed_Scale(float motor_speed_input)
{
	float motor_speed_output;		//电机输出
	//最大油门限制
	if(motor_speed_input>BS004_FLY_MAX_OUT) motor_speed_output=BS004_FLY_MAX_OUT;
	//最小油门限制
	else if(motor_speed_input<BS004_FLY_MIN_OUT) motor_speed_output=BS004_FLY_MIN_OUT;
	else motor_speed_output=motor_speed_input;
	return motor_speed_output;
}

//电机复位
void BS004_Motor_Reset(void)
{
	bs004_fly_m1=0;
	bs004_fly_m2=0;	
	bs004_fly_m3=0;
	bs004_fly_m4=0;
	//当前俯仰角，横滚角，偏航角
	bs004_angle_cur_pitch=0;
	bs004_angle_cur_roll=0;
	bs004_angle_cur_yaw=0;
	//上次俯仰角，横滚角，偏航角
	bs004_angle_last_pitch=0;
	bs004_angle_last_roll=0;	
	bs004_angle_last_yaw=0;		
	//更新电机参数
	Motor_BS004_M1=BS004_FLY_MIN_OUT;
	Motor_BS004_M2=BS004_FLY_MIN_OUT;
	Motor_BS004_M3=BS004_FLY_MIN_OUT;
	Motor_BS004_M4=BS004_FLY_MIN_OUT;
	//更新PWM占空比
	TIM_SetCompare4(TIM8,5);		//圆点博士:更新PWM占空比  
	TIM_SetCompare2(TIM8,5);	  //圆点博士:更新PWM占空比  
	TIM_SetCompare1(TIM8,5);   //圆点博士:更新PWM占空比      
	TIM_SetCompare3(TIM8,5);		//圆点博士:更新PWM占空比 
}






