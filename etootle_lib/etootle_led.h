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
#include "stm32f10x_it.h"
//
#define AnBT_MOTOR_LED_A				GPIO_Pin_15		//PA15
#define AnBT_MOTOR_LED_A_PORT		GPIOA
#define AnBT_MOTOR_LED_B				GPIO_Pin_6		//PB6
#define AnBT_MOTOR_LED_B_PORT		GPIOB
#define AnBT_MOTOR_LED_C				GPIO_Pin_5		//PB5
#define AnBT_MOTOR_LED_C_PORT		GPIOB
#define AnBT_MOTOR_LED_D				GPIO_Pin_7		//PB7
#define AnBT_MOTOR_LED_D_PORT		GPIOB

void BS004_LED_GPIO_Configuration(void);
void BS004_MOTOR_LED_ON(void);
void BS004_MOTOR_LED_OFF(void);


