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
#include "stm32f10x_lib.h"
#include "etootle_motor.h"
#include "etootle_mpu6050.h"
#include "etootle_imu.h"
#include "etootle_pid.h"
//
#define BS004_MCU_LED				GPIO_Pin_8		//PA8
#define BS004_MCU_LED_PORT	GPIOA
#define BS004_LED_EXT				GPIO_Pin_13		//PC13
#define BS004_LED_EXT_PORT	GPIOC
//
#define BS004_Bootloader_USB_CON_PORT		GPIOB					
#define BS004_Bootloader_USB_CON   			GPIO_Pin_8			
//
#define BS004_SYS_TIMER_SYSCLK_DIV	0
#define BS004_SYS_TIMER_CLK_1MHZ  	71
//
void BS004_RCC_Configuration(void);
void BS004_NVIC_Configuration(void);
void BS004_Long_Delay(unsigned int nCount);
//
void BS004_SYS_LED_Configuration(void);
void BS004_LED_EXT_OFF(void);
void BS004_LED_EXT_ON(void);
void BS004_MCU_LED_OFF(void);
void BS004_MCU_LED_ON(void);
void BS004_SYS_LED_TWINKLE(void);
void BS004_SYS_EULER_Update(void);
//
void BS004_SYS_Timer_Configuration(void);
void BS004_SYS_NVIC_Configuration(void);
void BS004_SYS_Timer_Interrupt(void);


