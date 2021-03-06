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
#include "etootle_led.h" 
//初始化LED控制的GPIO端口
void BS004_LED_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	//
  GPIO_InitStructure.GPIO_Pin = AnBT_MOTOR_LED_A;				//圆点博士:配置使用的LED口
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   		//圆点博士:设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  	//圆点博士:设置LED口为输出
  GPIO_Init(AnBT_MOTOR_LED_A_PORT, &GPIO_InitStructure); 
	//
  GPIO_InitStructure.GPIO_Pin = AnBT_MOTOR_LED_B;				//圆点博士:配置使用的LED口
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   		//圆点博士:设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  	//圆点博士:设置LED口为输出
  GPIO_Init(AnBT_MOTOR_LED_B_PORT, &GPIO_InitStructure); 
	//
	GPIO_InitStructure.GPIO_Pin = AnBT_MOTOR_LED_C;				//圆点博士:配置使用的LED口
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   		//圆点博士:设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  	//圆点博士:设置LED口为输出
  GPIO_Init(AnBT_MOTOR_LED_C_PORT, &GPIO_InitStructure); 
	//
	GPIO_InitStructure.GPIO_Pin = AnBT_MOTOR_LED_D;				//圆点博士:配置使用的LED口
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   		//圆点博士:设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  	//圆点博士:设置LED口为输出
  GPIO_Init(AnBT_MOTOR_LED_D_PORT, &GPIO_InitStructure); 	
	//
	BS004_MOTOR_LED_OFF();
}

//使用STM32的库函数来设置GPIO的端口
void BS004_MOTOR_LED_ON(void)		
{
	GPIO_SetBits(AnBT_MOTOR_LED_A_PORT, AnBT_MOTOR_LED_A);		//圆点博士:点亮LED
	GPIO_SetBits(AnBT_MOTOR_LED_B_PORT, AnBT_MOTOR_LED_B);		//圆点博士:点亮LED
	GPIO_SetBits(AnBT_MOTOR_LED_C_PORT, AnBT_MOTOR_LED_C);		//圆点博士:点亮LED
	GPIO_SetBits(AnBT_MOTOR_LED_D_PORT, AnBT_MOTOR_LED_D);		//圆点博士:点亮LED
}

void BS004_MOTOR_LED_OFF(void)		
{
	GPIO_ResetBits(AnBT_MOTOR_LED_A_PORT, AnBT_MOTOR_LED_A);	//圆点博士:熄灭LED
	GPIO_ResetBits(AnBT_MOTOR_LED_B_PORT, AnBT_MOTOR_LED_B);	//圆点博士:熄灭LED
	GPIO_ResetBits(AnBT_MOTOR_LED_C_PORT, AnBT_MOTOR_LED_C);	//圆点博士:熄灭LED
	GPIO_ResetBits(AnBT_MOTOR_LED_D_PORT, AnBT_MOTOR_LED_D);	//圆点博士:熄灭LED
}

