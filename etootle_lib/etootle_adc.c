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
/**************************************************************************************************************
本部分功能为实时检测电池电压 ADC转换-->DMA传输-->计算电压
**************************************************************************************************************/
#include "etootle_adc.h"
//两个ADC的转换结果
u16 BS004_ADC_Tab[2];
extern unsigned int bs004_bat_value;	
//电池电压全局变量
void BS004_ADC_Configuration(void)	
{
	BS004_ADC_IO_Configuration();
	//ADC功能IO配置
	BS004_ADC_DEVICE_Configuration();
	//使能ADC配置
	BS004_COM1_Send_Str_Head();		//发送头
	BS004_COM1_Send_Str_Body("finish to init adc device.");		//圆点博士:发送字符串
	BS004_COM1_Send_Str_Tail();		//发送尾
	//
}
//ADC功能IO配置，使用时必须设置GPIO_Mode为模拟输入
void BS004_ADC_IO_Configuration(void)	
{
  GPIO_InitTypeDef GPIO_InitStructure;
	//配置PB0，PB1为ADC功能IO
  GPIO_InitStructure.GPIO_Pin = BS004_VOLTAGE_CHA | BS004_VOLTAGE_CHB;					//圆点博士:配置使用的ADC口
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //圆点博士:设置IO口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	      //圆点博士:设置IO口模拟输入
  GPIO_Init(BS004_VOLTAGE_PORT, &GPIO_InitStructure); 
}

//配置DMA并使能ADC
void BS004_ADC_DEVICE_Configuration(void)	
{
		DMA_InitTypeDef	 DMA_InitStructure;
		ADC_InitTypeDef ADC_InitStructure;  
		//
		DMA_DeInit(DMA1_Channel1);
		DMA_InitStructure.DMA_PeripheralBaseAddr = BS004_ADC_Address;	//定义DMA外设地址
		DMA_InitStructure.DMA_MemoryBaseAddr =(u32)BS004_ADC_Tab;			//定义DMA内存地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;						//设定ADC作为数据传输的来源
		DMA_InitStructure.DMA_BufferSize = 2;													//指定DMA通道的DMA缓存的大小为 2个外设数据宽度
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//设定外设地址增量模式为不变
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//设置内存地址寄存器递增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//设定外设数据宽度为16位
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	//设定存储器数据宽度为16位
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 									//设定工作模式为循环缓冲工作模式
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;							//DMA通道拥有高优先级
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;										//DMA通道1不设置为内存到内存传输
		DMA_Init(DMA1_Channel1, &DMA_InitStructure);
		DMA_Cmd(DMA1_Channel1, ENABLE);											//圆点博士:使能DMA1_Channel1

		RCC_ADCCLKConfig(RCC_PCLK2_Div6);  										//设定ADC的时钟为PCLK/6
		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;		//设定每个ADC独立工作
		ADC_InitStructure.ADC_ScanConvMode = ENABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 2;
		ADC_Init(ADC1,&ADC_InitStructure);
		//设定ADC1的规则通道8，转换顺序为1，采样时间为239.5周期
		ADC_RegularChannelConfig(ADC1,ADC_Channel_8,1,ADC_SampleTime_239Cycles5);
		ADC_RegularChannelConfig(ADC1,ADC_Channel_9,2,ADC_SampleTime_239Cycles5);
		//使能ADC1的DMA请求
		ADC_DMACmd(ADC1,ENABLE);
		ADC_Cmd(ADC1,ENABLE);														//圆点博士:使能ADC

		ADC_ResetCalibration(ADC1);											//圆点博士:ADC校正 重置ADC1的校准寄存器
		while(ADC_GetResetCalibrationStatus(ADC1));
		ADC_StartCalibration(ADC1);											//获取ADC1重置寄存器的状态成功后开始ADC1的校准
		while(ADC_GetCalibrationStatus(ADC1));
		ADC_SoftwareStartConvCmd(ADC1,ENABLE);					//圆点博士:校准成功后开始ADC电压转换
}
/**************************************************************************************************
由于硬件分压，通过检测知 电池电压 = 2 * ADC转换电压
ADC分辨率为12， 参考电压为3.3v，ADC转换电压 = （ADC转换值/2^12） * 3.3v
全局变量电池电压为 unsigned int型，为了保留精度需要乘以10000
因此 bat_voltage=(2*bat_voltage/2^12)*3.3v*1000 = (bat_voltage/2048)*3300;
**************************************************************************************************/
void BS004_ADC_Get_ADC_Value(void)	
{
	float bat_volatge=0;
	//在两个ADC转换结果中取最小值
	if(BS004_ADC_Tab[0]>BS004_ADC_Tab[1]) bat_volatge=(float)BS004_ADC_Tab[1];
	else bat_volatge=(float)BS004_ADC_Tab[0];
	//将ADC转换结果换算成电压值
	bat_volatge=(bat_volatge/2048)*3300;
	//赋值给全局电压变量
	bs004_bat_value=(unsigned int) bat_volatge;
}


