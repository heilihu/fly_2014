//=========================================================================================================
//===�װ�����Ҷ��ܰ(ͬ�ô�ѧ):
//===��л��ѡ��Բ�㲩ʿС����,����ŵ����Դ���뱣��Э��(NDA)����Ĺ�����֧��������ǰ���Ķ�����==============
//---------------------------------------------------------------------------------------------------------
//===Dear ��Ҷ��ܰ(ͬ�ô�ѧ):
//===Thanks to select Dr.R&D quad-aircraft, and agree with the source code NDA(Non-Disclosure Agreement)===
//===Your ecourange and support are greatly appreciated!===================================================
//=========================================================================================================

/*************************************************************************************************************
Բ�㲩ʿС���������2014������Դ��������:
��Դ��������ο�,Բ�㲩ʿ����Դ�����ṩ�κ���ʽ�ĵ���,Ҳ������ʹ�ø�Դ��������ֵ���ʧ����.
�û�������ѧϰ��Ŀ���޸ĺ�ʹ�ø�Դ����.
���û����޸ĸ�Դ����ʱ,�����Ƴ��ò��ְ�Ȩ��Ϣ�����뱣��ԭ������.

������Ϣ������ʹٷ���վwww.etootle.com, �ٷ�����:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include "etootle_sys.h"
#include "etootle_main.h"
//
extern unsigned int system_led_timer_counter,system_timer_1ms_event,system_timer_counter;
unsigned int bs004_sys_timer_period=999;		//��ʱ�����ж�����ֵ
extern unsigned char BS004_IMU_Output,BS004_Motor_Lock;	
//�����ⲿ����ı�����������ļ��п���ʹ��

void BS004_RCC_Configuration(void)		//Բ�㲩ʿ:����ϵͳʱ��
{
  ErrorStatus HSEStartUpStatus;
	//����һ��ö�ٱ��������ں��������ķ��أ�����ֵΪERROR��SUCCESS
	NVIC_DeInit();		//��ʼ��ϵͳ���ж����������ܣ� ʹ���жϣ������������ȼ�����
  RCC_DeInit();			//��ʼ��ϵͳ��ʱ�ӼĴ���������Ϊȱʡֵ
  RCC_HSEConfig(RCC_HSE_ON);	//�����ⲿ���پ���
	//�ȴ�HSE���𣬷���ֵ��SUCCESSΪ�ȶ��Ҿ�����ERRORΪδ����
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  if(HSEStartUpStatus == SUCCESS)
  {
		//���HSI�ȶ������Ҿ���
		//ʹ��Ԥȡָ���� ���ٳ���Ķ�ȡ
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		//FLASHʱ���ӳټ������ڣ��ȴ�����ͬ������
    FLASH_SetLatency(FLASH_Latency_2);
		//����ϵͳʱ��AHB��Ƶ��
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
		//���õ���AHB(PCLK2)Ƶ��
    RCC_PCLK2Config(RCC_HCLK_Div2);
		//���õ���AHB(PCLK1)Ƶ��
    RCC_PCLK1Config(RCC_HCLK_Div2);
		//����PLLʱ��Դ�뱶Ƶϵ��
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		//ʹ��PLL
    RCC_PLLCmd(ENABLE);
		//�ȴ�PLL����
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }
		//����ϵͳʱ��ΪPLL��� 72MHz
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		//�ȴ� ϵͳ����PLLΪϵͳʱ��Դ 
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}

//�����ж��������Լ�ʹ�ܸ���ģ��ʱ��
void BS004_NVIC_Configuration(void)				//Բ�㲩ʿ:�ж�����
{
	NVIC_SetVectorTable(NVIC_VectTab_FLASH,(u32)0x8000);	//�����ж�������de 
	//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);		//Բ�㲩ʿ:ʹ�ܴ���
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);			//Բ�㲩ʿ:ʹ�ܵ�ѹ���
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);				//Բ�㲩ʿ:ʹ�ܵ�ѹDMA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);			//Բ�㲩ʿ:ʹ��ϵͳʱ������
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);			//Բ�㲩ʿ:ʹ�ܵ������
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	//Բ�㲩ʿ:ʹ��SWD������
	//ʧ��PRIMAS�����ȼ���������NMI��hard fault�쳣��Ҳ���������ж�/�쳣 ��ռ���ȼ� �������ж�ϵͳ
	NVIC_RESETPRIMASK();																			//Բ�㲩ʿ:ʹ���ж�
}

void BS004_SYS_LED_Configuration(void)											//Բ�㲩ʿ:LED����
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
	GPIO_InitStructure.GPIO_Pin = BS004_MCU_LED;					//Բ�㲩ʿ:����ʹ�õ�LED��
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   		//Բ�㲩ʿ:����LED�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  	//Բ�㲩ʿ:����LED��Ϊ���
  GPIO_Init(BS004_MCU_LED_PORT, &GPIO_InitStructure); 
	BS004_MCU_LED_ON();
	//
  GPIO_InitStructure.GPIO_Pin = BS004_LED_EXT;					//Բ�㲩ʿ:����ʹ�õ�LED��
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   		//Բ�㲩ʿ:����LED�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  	//Բ�㲩ʿ:����LED��Ϊ���
  GPIO_Init(BS004_LED_EXT_PORT, &GPIO_InitStructure); 
	BS004_LED_EXT_ON();
	//
	for(i=0;i<30;i++)				//Բ�㲩ʿ:��������LED
	{
		BS004_MCU_LED_OFF();
		BS004_Long_Delay(300000);
		BS004_MCU_LED_ON();
		BS004_Long_Delay(300000);
	}
}

//���ö�ʱ���жϣ����ö�ʱ��1ms�ж�һ��
void BS004_SYS_Timer_Configuration(void)		
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);		//ʹ��TIM1ʱ��
	//
	TIM_TimeBaseStructure.TIM_ClockDivision = BS004_SYS_TIMER_SYSCLK_DIV;	//ʱ�ӷָ����Ϊʱ�Ӳ��ָ�
	TIM_TimeBaseStructure.TIM_Prescaler = BS004_SYS_TIMER_CLK_1MHZ;		//����Ԥ��Ƶ����Prescaler=71+1=72
	TIM_TimeBaseStructure.TIM_Period = bs004_sys_timer_period;				//��������ֵ��Period=999+1=1000
	//�ж�Ƶ��Ϊ(72MHz/72)/1000 = 1KHz ����1msһ���ж�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;			//����Ϊ���ϼ���
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);		//Բ�㲩ʿ:����PWM���ں�Ƶ��:72*1K/72M=1mS			
	//����TIM1�����ȼ�
	BS004_SYS_NVIC_Configuration();
	TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);			//�����־λ
	TIM_ITConfig(TIM1,TIM_IT_Update, ENABLE);					//Բ�㲩ʿ:���ж�
	TIM_Cmd(TIM1, ENABLE);													  //Բ�㲩ʿ:����PWM
}

//���ö�ʱ���ж������� ����TIM1���ж����ȼ��ʹ����ȼ�
void BS004_SYS_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;			//Բ�㲩ʿ:����PWM�ж����ȼ�	
	//�������ȼ���Ϊ��1����ռ���ȼ�Ϊ0��1 �����ȼ�Ϊ0~7
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	//ѡ��ͨ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	//�����ж�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

//��ʱ���ж��ӳ��� ��λ��־λ
void BS004_SYS_Timer_Interrupt(void)					//Բ�㲩ʿ:PWM�жϺ���
{	
	system_timer_1ms_event=1;
}
//����LED��������VCC������LED�˿�Ϊ0ʱ����Ϊ1ʱ��
void BS004_LED_EXT_OFF(void)		
{	
	GPIO_ResetBits(BS004_LED_EXT_PORT, BS004_LED_EXT);	 //Բ�㲩ʿ:����LED
}
void BS004_LED_EXT_ON(void)		
{
	GPIO_SetBits(BS004_LED_EXT_PORT, BS004_LED_EXT);		//Բ�㲩ʿ:Ϩ��LED
}
//
void BS004_MCU_LED_OFF(void)		
{
	GPIO_SetBits(BS004_MCU_LED_PORT, BS004_MCU_LED);		//Բ�㲩ʿ:Ϩ��LED
}
void BS004_MCU_LED_ON(void)		
{
	GPIO_ResetBits(BS004_MCU_LED_PORT, BS004_MCU_LED);	 //Բ�㲩ʿ:����LED
}
//LED��˸��LED�˿�һ��ʱ��Ϊ�͵�ƽ��һ��ʱ��Ϊ�ߵ�ƽ
void BS004_SYS_LED_TWINKLE(void)										   //Բ�㲩ʿ:��˸LED
{
	if(system_led_timer_counter>1000) system_led_timer_counter=0;	//��������������Χ�����¿�ʼ��ʱ
	//����ֵΪ500ʱ(500~1000)��״̬ΪMCU��LED����EXT��LED����
	if(system_led_timer_counter==500) 
	{
		BS004_MCU_LED_OFF();
		BS004_LED_EXT_ON();
	}
	//����ʱ��Ϊ1000ʱ(0~500)��״̬ΪMCU��LED������EXT��LED����
	else if(system_led_timer_counter==1000) 
	{
		BS004_MCU_LED_ON();
		BS004_LED_EXT_OFF();
	}
	//ϵͳÿ20ms����һ�μ��
	if(system_timer_counter>20) 
	{
		system_timer_counter=0;
		//�����ʱ������̬���ݴ�����λ�����ҵ����Դ�ǹرյģ���������ݷ���
		if(BS004_IMU_Output && BS004_Motor_Lock) ANBT_SEND_DMP_EULER_DATA(); 
	}	
}

void BS004_Long_Delay(unsigned int nCount) 		  //Բ�㲩ʿ:��ʱ����
{
	for(; nCount != 0; nCount--);
}




