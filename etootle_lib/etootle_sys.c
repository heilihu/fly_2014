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
#include "etootle_sys.h"
#include "etootle_main.h"
//
extern unsigned int system_led_timer_counter,system_timer_1ms_event,system_timer_counter;
unsigned int bs004_sys_timer_period=999;		//定时器的中断周期值
extern unsigned char BS004_IMU_Output,BS004_Motor_Lock;	
//申明外部定义的变量，在这个文件中可以使用

void BS004_RCC_Configuration(void)		//圆点博士:配置系统时钟
{
  ErrorStatus HSEStartUpStatus;
	//定义一个枚举变量，用于函数参数的返回，返回值为ERROR，SUCCESS
	NVIC_DeInit();		//初始化系统的中断向量表，功能， 使能中断，所有任务优先级置零
  RCC_DeInit();			//初始化系统的时钟寄存器，配置为缺省值
  RCC_HSEConfig(RCC_HSE_ON);	//启动外部高速晶振
	//等待HSE起震，返回值：SUCCESS为稳定且就绪，ERROR为未就绪
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  if(HSEStartUpStatus == SUCCESS)
  {
		//如果HSI稳定起震且就绪
		//使能预取指缓存 加速程序的读取
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		//FLASH时序延迟几个周期，等待总线同步操作
    FLASH_SetLatency(FLASH_Latency_2);
		//设置系统时钟AHB分频器
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
		//设置低速AHB(PCLK2)频率
    RCC_PCLK2Config(RCC_HCLK_Div2);
		//设置低速AHB(PCLK1)频率
    RCC_PCLK1Config(RCC_HCLK_Div2);
		//设置PLL时钟源与倍频系数
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		//使能PLL
    RCC_PLLCmd(ENABLE);
		//等待PLL就绪
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }
		//设置系统时钟为PLL输出 72MHz
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		//等待 系统设置PLL为系统时钟源 
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}

//配置中断向量表以及使能各个模块时钟
void BS004_NVIC_Configuration(void)				//圆点博士:中断设置
{
	NVIC_SetVectorTable(NVIC_VectTab_FLASH,(u32)0x8000);	//设置中断向量表de 
	//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);		//圆点博士:使能串口
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);			//圆点博士:使能电压检测
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);				//圆点博士:使能电压DMA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);			//圆点博士:使能系统时钟驱动
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);			//圆点博士:使能电机驱动
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	//圆点博士:使能SWD下载线
	//失能PRIMAS看优先级，既允许NMI和hard fault异常，也允许其他中断/异常 抢占优先级 即打开了中断系统
	NVIC_RESETPRIMASK();																			//圆点博士:使能中断
}

void BS004_SYS_LED_Configuration(void)											//圆点博士:LED设置
{
	unsigned char i;
	GPIO_InitTypeDef GPIO_InitStructure;
	//

	GPIO_InitStructure.GPIO_Pin = BS004_Bootloader_USB_CON;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BS004_Bootloader_USB_CON_PORT, &GPIO_InitStructure);
	GPIO_SetBits(BS004_Bootloader_USB_CON_PORT, BS004_Bootloader_USB_CON); 	
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MCU_LED;					//圆点博士:配置使用的LED口
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   		//圆点博士:设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  	//圆点博士:设置LED口为输出
  GPIO_Init(BS004_MCU_LED_PORT, &GPIO_InitStructure); 
	BS004_MCU_LED_ON();
	//
  GPIO_InitStructure.GPIO_Pin = BS004_LED_EXT;					//圆点博士:配置使用的LED口
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   		//圆点博士:设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  	//圆点博士:设置LED口为输出
  GPIO_Init(BS004_LED_EXT_PORT, &GPIO_InitStructure); 
	BS004_LED_EXT_ON();
	//
	for(i=0;i<30;i++)				//圆点博士:快速闪动LED
	{
		BS004_MCU_LED_OFF();
		BS004_Long_Delay(300000);
		BS004_MCU_LED_ON();
		BS004_Long_Delay(300000);
	}
}

//设置定时器中断，配置定时器1ms中断一次
void BS004_SYS_Timer_Configuration(void)		
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);		//使能TIM1时钟
	//
	TIM_TimeBaseStructure.TIM_ClockDivision = BS004_SYS_TIMER_SYSCLK_DIV;	//时钟分割，设置为时钟不分割
	TIM_TimeBaseStructure.TIM_Prescaler = BS004_SYS_TIMER_CLK_1MHZ;		//设置预分频数，Prescaler=71+1=72
	TIM_TimeBaseStructure.TIM_Period = bs004_sys_timer_period;				//设置周期值：Period=999+1=1000
	//中断频率为(72MHz/72)/1000 = 1KHz ，即1ms一次中断
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;			//设置为向上计数
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);		//圆点博士:设置PWM周期和频率:72*1K/72M=1mS			
	//设置TIM1的优先级
	BS004_SYS_NVIC_Configuration();
	TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);			//清除标志位
	TIM_ITConfig(TIM1,TIM_IT_Update, ENABLE);					//圆点博士:打开中断
	TIM_Cmd(TIM1, ENABLE);													  //圆点博士:启动PWM
}

//设置定时器中断向量表 配置TIM1的中断优先级和从优先级
void BS004_SYS_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;			//圆点博士:设置PWM中断优先级	
	//设置优先级组为组1，抢占优先级为0，1 从优先级为0~7
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	//选择通道
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	//开启中断
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

//定时器中断子程序 置位标志位
void BS004_SYS_Timer_Interrupt(void)					//圆点博士:PWM中断函数
{	
	system_timer_1ms_event=1;
}
//由于LED的阳极接VCC，所以LED端口为0时亮，为1时暗
void BS004_LED_EXT_OFF(void)		
{	
	GPIO_ResetBits(BS004_LED_EXT_PORT, BS004_LED_EXT);	 //圆点博士:点亮LED
}
void BS004_LED_EXT_ON(void)		
{
	GPIO_SetBits(BS004_LED_EXT_PORT, BS004_LED_EXT);		//圆点博士:熄灭LED
}
//
void BS004_MCU_LED_OFF(void)		
{
	GPIO_SetBits(BS004_MCU_LED_PORT, BS004_MCU_LED);		//圆点博士:熄灭LED
}
void BS004_MCU_LED_ON(void)		
{
	GPIO_ResetBits(BS004_MCU_LED_PORT, BS004_MCU_LED);	 //圆点博士:点亮LED
}
//LED闪烁即LED端口一段时间为低电平，一段时间为高电平
void BS004_SYS_LED_TWINKLE(void)										   //圆点博士:闪烁LED
{
	if(system_led_timer_counter>1000) system_led_timer_counter=0;	//当计数器超过范围，重新开始计时
	//计数值为500时(500~1000)，状态为MCU的LED灯灭，EXT的LED灯亮
	if(system_led_timer_counter==500) 
	{
		BS004_MCU_LED_OFF();
		BS004_LED_EXT_ON();
	}
	//当计时数为1000时(0~500)，状态为MCU的LED灯亮。EXT的LED灯灭
	else if(system_led_timer_counter==1000) 
	{
		BS004_MCU_LED_ON();
		BS004_LED_EXT_OFF();
	}
	//系统每20ms进行一次检查
	if(system_timer_counter>20) 
	{
		system_timer_counter=0;
		//如果此时允许姿态数据传回上位机而且电机电源是关闭的，则进行数据发回
		if(BS004_IMU_Output && BS004_Motor_Lock) ANBT_SEND_DMP_EULER_DATA(); 
	}	
}

void BS004_Long_Delay(unsigned int nCount) 		  //圆点博士:延时函数
{
	for(; nCount != 0; nCount--);
}




