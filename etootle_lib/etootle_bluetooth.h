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
#include <math.h>

#define BS004_COM1_TX	GPIO_Pin_9
#define BS004_COM1_RX	GPIO_Pin_10

void BS004_COM1_Communication(void);
void BS004_COM1_Interrupt(void);  
//
void BS004_COM1_Send_Char(unsigned char ascii_code);
void BS004_COM1_Send_Num(unsigned char number);
void BS004_COM1_Send_Str_Head(void);
void BS004_COM1_Send_Str_Body(unsigned char* str_buf);
void BS004_COM1_Send_Str_Tail(void);
void BS004_COM1_Send_4bits_BCD_Num(int number);
//
void BS004_COM1_GPIO_Configuration(void);
void BS004_COM1_Port_Configuration(void);
void BS004_COM1_NVIC_Configuration(void);
unsigned char BS004_COM1_Task_Process(void);
//

