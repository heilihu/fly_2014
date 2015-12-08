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
/**************************************************************************************************************
�����ֹ���Ϊʵʱ����ص�ѹ ADCת��-->DMA����-->�����ѹ
**************************************************************************************************************/
#include "etootle_adc.h"
//����ADC��ת�����
u16 BS004_ADC_Tab[2];
extern unsigned int bs004_bat_value;	
//��ص�ѹȫ�ֱ���
void BS004_ADC_Configuration(void)	
{
	BS004_ADC_IO_Configuration();
	//ADC����IO����
	BS004_ADC_DEVICE_Configuration();
	//ʹ��ADC����
	BS004_COM1_Send_Str_Head();		//����ͷ
	BS004_COM1_Send_Str_Body("finish to init adc device.");		//Բ�㲩ʿ:�����ַ���
	BS004_COM1_Send_Str_Tail();		//����β
	//
}
//ADC����IO���ã�ʹ��ʱ��������GPIO_ModeΪģ������
void BS004_ADC_IO_Configuration(void)	
{
  GPIO_InitTypeDef GPIO_InitStructure;
	//����PB0��PB1ΪADC����IO
  GPIO_InitStructure.GPIO_Pin = BS004_VOLTAGE_CHA | BS004_VOLTAGE_CHB;					//Բ�㲩ʿ:����ʹ�õ�ADC��
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //Բ�㲩ʿ:����IO�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	      //Բ�㲩ʿ:����IO��ģ������
  GPIO_Init(BS004_VOLTAGE_PORT, &GPIO_InitStructure); 
}

//����DMA��ʹ��ADC
void BS004_ADC_DEVICE_Configuration(void)	
{
		DMA_InitTypeDef	 DMA_InitStructure;
		ADC_InitTypeDef ADC_InitStructure;  
		//
		DMA_DeInit(DMA1_Channel1);
		DMA_InitStructure.DMA_PeripheralBaseAddr = BS004_ADC_Address;	//����DMA�����ַ
		DMA_InitStructure.DMA_MemoryBaseAddr =(u32)BS004_ADC_Tab;			//����DMA�ڴ��ַ
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;						//�趨ADC��Ϊ���ݴ������Դ
		DMA_InitStructure.DMA_BufferSize = 2;													//ָ��DMAͨ����DMA����Ĵ�СΪ 2���������ݿ��
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//�趨�����ַ����ģʽΪ����
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//�����ڴ��ַ�Ĵ�������
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//�趨�������ݿ��Ϊ16λ
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	//�趨�洢�����ݿ��Ϊ16λ
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 									//�趨����ģʽΪѭ�����幤��ģʽ
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;							//DMAͨ��ӵ�и����ȼ�
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;										//DMAͨ��1������Ϊ�ڴ浽�ڴ洫��
		DMA_Init(DMA1_Channel1, &DMA_InitStructure);
		DMA_Cmd(DMA1_Channel1, ENABLE);											//Բ�㲩ʿ:ʹ��DMA1_Channel1

		RCC_ADCCLKConfig(RCC_PCLK2_Div6);  										//�趨ADC��ʱ��ΪPCLK/6
		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;		//�趨ÿ��ADC��������
		ADC_InitStructure.ADC_ScanConvMode = ENABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 2;
		ADC_Init(ADC1,&ADC_InitStructure);
		//�趨ADC1�Ĺ���ͨ��8��ת��˳��Ϊ1������ʱ��Ϊ239.5����
		ADC_RegularChannelConfig(ADC1,ADC_Channel_8,1,ADC_SampleTime_239Cycles5);
		ADC_RegularChannelConfig(ADC1,ADC_Channel_9,2,ADC_SampleTime_239Cycles5);
		//ʹ��ADC1��DMA����
		ADC_DMACmd(ADC1,ENABLE);
		ADC_Cmd(ADC1,ENABLE);														//Բ�㲩ʿ:ʹ��ADC

		ADC_ResetCalibration(ADC1);											//Բ�㲩ʿ:ADCУ�� ����ADC1��У׼�Ĵ���
		while(ADC_GetResetCalibrationStatus(ADC1));
		ADC_StartCalibration(ADC1);											//��ȡADC1���üĴ�����״̬�ɹ���ʼADC1��У׼
		while(ADC_GetCalibrationStatus(ADC1));
		ADC_SoftwareStartConvCmd(ADC1,ENABLE);					//Բ�㲩ʿ:У׼�ɹ���ʼADC��ѹת��
}
/**************************************************************************************************
����Ӳ����ѹ��ͨ�����֪ ��ص�ѹ = 2 * ADCת����ѹ
ADC�ֱ���Ϊ12�� �ο���ѹΪ3.3v��ADCת����ѹ = ��ADCת��ֵ/2^12�� * 3.3v
ȫ�ֱ�����ص�ѹΪ unsigned int�ͣ�Ϊ�˱���������Ҫ����10000
��� bat_voltage=(2*bat_voltage/2^12)*3.3v*1000 = (bat_voltage/2048)*3300;
**************************************************************************************************/
void BS004_ADC_Get_ADC_Value(void)	
{
	float bat_volatge=0;
	//������ADCת�������ȡ��Сֵ
	if(BS004_ADC_Tab[0]>BS004_ADC_Tab[1]) bat_volatge=(float)BS004_ADC_Tab[1];
	else bat_volatge=(float)BS004_ADC_Tab[0];
	//��ADCת���������ɵ�ѹֵ
	bat_volatge=(bat_volatge/2048)*3300;
	//��ֵ��ȫ�ֵ�ѹ����
	bs004_bat_value=(unsigned int) bat_volatge;
}


