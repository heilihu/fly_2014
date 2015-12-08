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
#include "etootle_mpu6050.h"
#include "etootle_sys.h"
//
signed short int bs004_mpu6050_gyro_pitch_raw=0,bs004_mpu6050_gyro_roll_raw=0,bs004_mpu6050_gyro_yaw_raw=0;
signed short int bs004_mpu6050_gyro_pitch_cal=0,bs004_mpu6050_gyro_roll_cal=0,bs004_mpu6050_gyro_yaw_cal=0;
signed short int bs004_mpu6050_acc_pitch_raw=0,bs004_mpu6050_acc_roll_raw=0,bs004_mpu6050_acc_yaw_raw=0;
signed short int bs004_mpu6050_acc_pitch_cal=0,bs004_mpu6050_acc_roll_cal=0,bs004_mpu6050_acc_yaw_cal=0;
signed short int bs004_hmc5883l_mag_pitch_cal=0,bs004_hmc5883l_mag_roll_cal=0,bs004_hmc5883l_mag_yaw_cal=0;
//
float  bs004_filter_high=0,bs004_filter_low=0,bs004_filter_time=0;
float  bs004_mpu6050_acc_pitch_com=0,bs004_mpu6050_acc_roll_com=0;
//
float bs004_mpu6050_gyro_angel_pitch_ave=0,bs004_mpu6050_gyro_angel_roll_ave=0,bs004_mpu6050_gyro_angel_yaw_ave=0;
float bs004_mpu6050_acc_angel_pitch_ave=0,bs004_mpu6050_acc_angel_roll_ave=0,bs004_mpu6050_acc_angel_yaw_ave=0;
//����mpu6050�Ľ��ٶȺͼ��ٶȵ����ݲ�����ֵ����mpu6050_gyro_data_buffer�� ͨ��ֱ�Ӷ�ȡmpu6050��ACEL_XOUT�Ĵ���������
void BS004_Get_MPU6050_Data(void)   
{
	unsigned char i;
	unsigned char bs004_mpu6050_data_buffer[14];
	signed short int bs004_mpu6050_raw_data[7];
	//����ж�����
	if(!BS004_MPU6050_INT_STATE) 
	{
		BS004_I2C_START();
		BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);					//Բ�㲩ʿ:����������д��ַ
		BS004_I2C_SendByte(BS004_MPU6050_ACCEL_DATA_ADDR);    //Բ�㲩ʿ:���������ǼĴ�����ַ
		BS004_I2C_START();
		BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR+1);      	//Բ�㲩ʿ:���������Ƕ���ַ
		BS004_I2C_Receive14Bytes(bs004_mpu6050_data_buffer);		//Բ�㲩ʿ:���������ǼĴ���ֵ
		BS004_I2C_STOP();
		//				
		for(i=0;i<7;i++) bs004_mpu6050_raw_data[i]=(((signed short int)bs004_mpu6050_data_buffer[i*2]) << 8) | bs004_mpu6050_data_buffer[i*2+1];
		bs004_mpu6050_acc_roll_raw=bs004_mpu6050_raw_data[0]-bs004_mpu6050_acc_roll_cal;
		bs004_mpu6050_acc_pitch_raw=bs004_mpu6050_raw_data[1]-bs004_mpu6050_acc_pitch_cal;
		bs004_mpu6050_acc_yaw_raw=bs004_mpu6050_raw_data[2];
		bs004_mpu6050_gyro_pitch_raw=bs004_mpu6050_raw_data[4]-bs004_mpu6050_gyro_pitch_cal;
		bs004_mpu6050_gyro_roll_raw=bs004_mpu6050_raw_data[5]-bs004_mpu6050_gyro_roll_cal;;
		bs004_mpu6050_gyro_yaw_raw=bs004_mpu6050_raw_data[6]-bs004_mpu6050_gyro_yaw_cal;
		//
	}
	//	
	bs004_mpu6050_acc_pitch_com=(bs004_filter_high*bs004_mpu6050_acc_pitch_com+bs004_filter_low*bs004_mpu6050_acc_pitch_raw);	
	bs004_mpu6050_acc_roll_com =(bs004_filter_high*bs004_mpu6050_acc_roll_com +bs004_filter_low*bs004_mpu6050_acc_roll_raw);
	//
  bs004_mpu6050_acc_angel_pitch_ave=(bs004_mpu6050_acc_angel_pitch_ave+bs004_mpu6050_acc_pitch_com)/2.0f;
	bs004_mpu6050_acc_angel_roll_ave =(bs004_mpu6050_acc_angel_roll_ave +bs004_mpu6050_acc_roll_com)/2.0f;
	bs004_mpu6050_acc_angel_yaw_ave  =(bs004_mpu6050_acc_angel_yaw_ave  +bs004_mpu6050_acc_yaw_raw)/2.0f;
	//
	bs004_mpu6050_gyro_angel_pitch_ave=(bs004_mpu6050_gyro_angel_pitch_ave+bs004_mpu6050_gyro_pitch_raw)/2.0f;
	bs004_mpu6050_gyro_angel_roll_ave =(bs004_mpu6050_gyro_angel_roll_ave +bs004_mpu6050_gyro_roll_raw)/2.0f;
	bs004_mpu6050_gyro_angel_yaw_ave  =(bs004_mpu6050_gyro_angel_yaw_ave  +bs004_mpu6050_gyro_yaw_raw)/2.0f;
}
//
void BS004_I2C_Configuration(void)			
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//
  GPIO_InitStructure.GPIO_Pin = BS004_I2C_SCL | BS004_I2C_SDA;	//Բ�㲩ʿ:����ʹ�õ�I2C��
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   						//Բ�㲩ʿ:����I2C�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	  					//Բ�㲩ʿ:����I2CΪ��©���
  GPIO_Init(BS004_I2C_PORT, &GPIO_InitStructure); 
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MPU6050_INT;					//Բ�㲩ʿ:����ʹ�õ�I2C��
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   				//Բ�㲩ʿ:����I2C�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	  	//Բ�㲩ʿ:����I2CΪ��©���
  GPIO_Init(BS004_MPU6050_INT_PORT, &GPIO_InitStructure); 
	//
	BS004_I2C_SCL_1; 
	BS004_I2C_SDA_1;
	BS004_I2C_DELAY;
}

void BS004_I2C_Delay(unsigned int dly)               
{
	while(--dly);	//dly=100: 8.75us; dly=100: 85.58 us (SYSCLK=72MHz)
}

unsigned char BS004_I2C_START(void)
{ 
	BS004_I2C_SDA_1; 
 	BS004_I2C_NOP;
  // 
 	BS004_I2C_SCL_1; 
 	BS004_I2C_NOP;    
	//
 	if(!BS004_I2C_SDA_STATE) return BS004_I2C_BUS_BUSY;
	//
 	BS004_I2C_SDA_0;
 	BS004_I2C_NOP;
  //
 	BS004_I2C_SCL_0;  
 	BS004_I2C_NOP; 
	//
 	if(BS004_I2C_SDA_STATE) return BS004_I2C_BUS_ERROR;
	//
 	return BS004_I2C_READY;
}

void BS004_I2C_STOP(void)
{
 	BS004_I2C_SDA_0; 
 	BS004_I2C_NOP;
  // 
 	BS004_I2C_SCL_1; 
 	BS004_I2C_NOP;    
	//
 	BS004_I2C_SDA_1;
 	BS004_I2C_NOP;
}

void BS004_I2C_SendACK(void)
{
 	BS004_I2C_SDA_0;
 	BS004_I2C_NOP;
 	BS004_I2C_SCL_1;
 	BS004_I2C_NOP;
 	BS004_I2C_SCL_0; 
 	BS004_I2C_NOP;  
}

void BS004_I2C_SendNACK(void)
{
	BS004_I2C_SDA_1;
	BS004_I2C_NOP;
	BS004_I2C_SCL_1;
	BS004_I2C_NOP;
	BS004_I2C_SCL_0; 
	BS004_I2C_NOP;  
}

unsigned char BS004_I2C_SendByte(u8 bs004_i2c_data)
{
 	unsigned char i;
 	//
	BS004_I2C_SCL_0;
 	for(i=0;i<8;i++)
 	{  
  		if(bs004_i2c_data&0x80) BS004_I2C_SDA_1;
   		else BS004_I2C_SDA_0;
			//
  		bs004_i2c_data<<=1;
  		BS004_I2C_NOP;
			//
  		BS004_I2C_SCL_1;
  		BS004_I2C_NOP;
  		BS004_I2C_SCL_0;
  		BS004_I2C_NOP; 
 	}
	//
 	BS004_I2C_SDA_1; 
 	BS004_I2C_NOP;
 	BS004_I2C_SCL_1;
 	BS004_I2C_NOP;   
 	if(BS004_I2C_SDA_STATE)
 	{
  		BS004_I2C_SCL_0;
  		return BS004_I2C_NACK;
 	}
 	else
 	{
  		BS004_I2C_SCL_0;
  		return BS004_I2C_ACK;  
 	}    
}

unsigned char BS004_I2C_ReceiveByte_NoACK(void)
{
	unsigned char i,bs004_i2c_data;
	//
 	BS004_I2C_SDA_1;
 	BS004_I2C_SCL_0; 
 	bs004_i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		BS004_I2C_SCL_1;
  		BS004_I2C_NOP; 
  		bs004_i2c_data<<=1;
			//
  		if(BS004_I2C_SDA_STATE)	bs004_i2c_data|=0x01; 
  
  		BS004_I2C_SCL_0;  
  		BS004_I2C_NOP;         
 	}
	BS004_I2C_SendNACK();
 	return bs004_i2c_data;
}

unsigned char BS004_I2C_ReceiveByte_WithACK(void)
{
	unsigned char i,bs004_i2c_data;
	//
 	BS004_I2C_SDA_1;
 	BS004_I2C_SCL_0; 
 	bs004_i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		BS004_I2C_SCL_1;
  		BS004_I2C_NOP; 
  		bs004_i2c_data<<=1;
			//
  		if(BS004_I2C_SDA_STATE)	bs004_i2c_data|=0x01; 
  
  		BS004_I2C_SCL_0;  
  		BS004_I2C_NOP;         
 	}
	BS004_I2C_SendACK();
 	return bs004_i2c_data;
}

void BS004_I2C_Receive14Bytes(u8 *anbt_i2c_data_buffer)
{
	u8 i,j;
	u8 anbt_i2c_data;

	for(j=0;j<13;j++)
	{
		BS004_I2C_SDA_1;
		BS004_I2C_SCL_0; 
		anbt_i2c_data=0;
		//
		for(i=0;i<8;i++)
		{
  		BS004_I2C_SCL_1;
  		BS004_I2C_NOP; 
  		anbt_i2c_data<<=1;
			//
  		if(BS004_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  		BS004_I2C_SCL_0;  
  		BS004_I2C_NOP;         
		}
		anbt_i2c_data_buffer[j]=anbt_i2c_data;
		BS004_I2C_SendACK();
	}
	//
	BS004_I2C_SDA_1;
	BS004_I2C_SCL_0; 
	anbt_i2c_data=0;
	for(i=0;i<8;i++)
	{
  	BS004_I2C_SCL_1;
  	BS004_I2C_NOP; 
  	anbt_i2c_data<<=1;
			//
  	if(BS004_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  	BS004_I2C_SCL_0;  
  	BS004_I2C_NOP;         
	}
	anbt_i2c_data_buffer[13]=anbt_i2c_data;
	BS004_I2C_SendNACK();
}
//
//��ʼ������mpu6050
void BS004_MPU6050_Init(void)	
{
	unsigned char mpu6050_chip_id;
	//
	BS004_I2C_Configuration();
	//
	BS004_MPU6050_PWM_CFG_FUN(); 				//Բ�㲩ʿ:�����ڲ�ʱ��,ѡ��32.768KHzʱ��
	BS004_MPU6050_EXIT_SLEEP_FUN();    	//Բ�㲩ʿ:�˳�����ģʽ
	BS004_MPU6050_GYRO_CFG_FUN();      	//Բ�㲩ʿ:�������������̣�����2000��/s
	BS004_MPU6050_ACCEL_CFG_FUN();     	//Բ�㲩ʿ:���ü��ٶ�����: ��16g
	BS004_MPU6050_USER_CTRL_FUN();			//����mpu6050��IIC�ӻ�ģʽ
	BS004_MPU6050_I2CBYPASS_CFG_FUN(); 	//Բ�㲩ʿ:���õ�Ŷ�дģʽ
	BS004_MPU6050_INT_CFG_FUN();				//��������IIC����������ʹ�ú������ж�����
	//
	mpu6050_chip_id=BS004_MPU6050_GYRO_WHOAMI_FUN();
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("mpu6050 chip id .");		//Բ�㲩ʿ:��ʼ��ADC
	BS004_COM1_Send_4bits_BCD_Num(mpu6050_chip_id);
	BS004_COM1_Send_Str_Tail();	
	//
	BS004_Long_Delay(10000);	//Բ�㲩ʿ:������ʱ
}
//����mpu6050��ID��(who am i),ͨ����mpu6050��WHO_AM_I ��ַ0x75���� �� ����Ȼ����յ�mpu6050��ID��
unsigned char BS004_MPU6050_GYRO_WHOAMI_FUN(void)
{
	unsigned char bs004_mpu6050_gyro_id;
	//����Ϊ��׼��IICд��ʽ
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);					//Բ�㲩ʿ:����������д��ַ
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_WHOAMI_ADDR);  	//Բ�㲩ʿ:����������ID��ַ
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR+1);      	//Բ�㲩ʿ:���������Ƕ���ַ
	bs004_mpu6050_gyro_id=BS004_I2C_ReceiveByte_NoACK();	//Բ�㲩ʿ:����������ID
	BS004_I2C_STOP();
	//
	return bs004_mpu6050_gyro_id;
	//
}
//����mpu6050��ָ���Ĵ�����ֵ
unsigned char BS004_MPU6050_READ_REG_FUN(unsigned char bs004_mpu6050_dev_addr,unsigned char bs004_mpu6050_reg_addr)   
{
	unsigned char bs004_mpu6050_reg;
	
	BS004_I2C_START();
	BS004_I2C_SendByte(bs004_mpu6050_dev_addr);					//Բ�㲩ʿ:����������д��ַ
	BS004_I2C_SendByte(bs004_mpu6050_reg_addr);    			//Բ�㲩ʿ:���������ǼĴ�����ַ
	BS004_I2C_START();
	BS004_I2C_SendByte(bs004_mpu6050_dev_addr+1);      	//Բ�㲩ʿ:���������Ƕ���ַ
	bs004_mpu6050_reg=BS004_I2C_ReceiveByte_NoACK();		//Բ�㲩ʿ:���������ǼĴ���ֵ
	BS004_I2C_STOP();
	//
	return bs004_mpu6050_reg;
}
//����mpu6050�ڲ�ʱ��Ϊ32768Hzʱ�ӣ���λmpu6050,ͨ����mpu6050��PWR_MGMT_1�Ĵ���д��0x01ʵ��
void BS004_MPU6050_PWM_CFG_FUN(void)   
{
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);				//Բ�㲩ʿ:����������д��ַ
	BS004_I2C_SendByte(BS004_MPU6050_PWR_MGMT_1_ADDR);  //Բ�㲩ʿ:����������PWM��ַ
	BS004_I2C_SendByte(BS004_MPU6050_PWR_MGMT_1_VALUE); //Բ�㲩ʿ:����������PWMֵ
	BS004_I2C_STOP();
}
//���������ǵ�����Ϊ����2000��/s ͨ����mpu6050��GYRO����CONFIG�Ĵ��� ��ַΪ0x1B����0x18ʵ��
void BS004_MPU6050_GYRO_CFG_FUN(void)   
{
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);				//Բ�㲩ʿ:����������д��ַ
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_CFG_ADDR);   	//Բ�㲩ʿ:����������PWM��ַ
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_CFG_VALUE); 	//Բ�㲩ʿ:����������PWMֵ
	BS004_I2C_STOP();
}
//����mpu6050���ٶ�����Ϊ����16g���������ݸ����˲�ģʽ��ֹƵ��Ϊ5MHz,ͨ����mpu6050��ACCEL_CONFIG��ַΪ0x1C����0x18ʵ��
void BS004_MPU6050_ACCEL_CFG_FUN(void)   
{
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);					//Բ�㲩ʿ:����������д��ַ
	BS004_I2C_SendByte(BS004_MPU6050_ACCEL_CFG_ADDR);   //Բ�㲩ʿ:����������PWM��ַ
	BS004_I2C_SendByte(BS004_MPU6050_ACCEL_CFG_VALUE); 	//Բ�㲩ʿ:����������PWMֵ
	BS004_I2C_STOP();
}
//�˳�˯��ģʽ���ظ�mpu6050��Ĭ�����á�ͨ����mpu6050��PWR_MGMT_1�Ĵ�������ַΪ0X6Bд��0x01ʵ��
void BS004_MPU6050_EXIT_SLEEP_FUN(void)  
{
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);					//Բ�㲩ʿ:����������д��ַ
	BS004_I2C_SendByte(BS004_MPU6050_PWR_MGMT_1_ADDR);  //Բ�㲩ʿ:����������PWM��ַ
	BS004_I2C_SendByte(BS004_MPU6050_EXIT_SLEEP_VALUE); //Բ�㲩ʿ:����������PWMֵ
	BS004_I2C_STOP();
}
//����mpu6050��ͨ��ģʽΪIIC�ӻ�ģʽ��������FIFO��ʵ�ַ���ͨ����mpu6050��USER����CTRL�Ĵ��� ��ַΪ0x6a��д��0x00ʵ��
void BS004_MPU6050_USER_CTRL_FUN(void)   
{
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);			 //Բ�㲩ʿ:����������д��ַ
	BS004_I2C_SendByte(BS004_MPU6050_USER_CTRL_ADDR);  //Բ�㲩ʿ:����������PWM��ַ
	BS004_I2C_SendByte(BS004_MPU6050_USER_CTRL_VALUE); //Բ�㲩ʿ:����������PWMֵ
	BS004_I2C_STOP();
}
//���ÿ�������IIC�����������̻�����ʹ��IICͨ�ŵ��豸���˴�Ϊ�������̣������ж��źŲ����������ж������ź�Ϊ��©�����
//ͨ����mpu6050��INT_PIN_CFG�Ĵ���д��0xb2ʵ��
void BS004_MPU6050_I2CBYPASS_CFG_FUN(void)   
{
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);					  //Բ�㲩ʿ:����������д��ַ
	BS004_I2C_SendByte(BS004_MPU6050_I2CBYPASS_CFG_ADDR);   //Բ�㲩ʿ:����������PWM��ַ
	BS004_I2C_SendByte(BS004_MPU6050_I2CBYPASS_CFG_VALUE); 	//Բ�㲩ʿ:����������PWMֵ
	BS004_I2C_STOP();
}
//���ÿ���mpu6050�������ж����ѣ������������������ݶ�׼�������������STM32�����ж�����
//ͨ����mpu6050��INT_ENABLE�Ĵ���д��0x01ʵ��
void BS004_MPU6050_INT_CFG_FUN(void)   
{
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);			//Բ�㲩ʿ:����������д��ַ
	BS004_I2C_SendByte(BS004_MPU6050_INT_CFG_ADDR);   //Բ�㲩ʿ:����������PWM��ַ
	BS004_I2C_SendByte(BS004_MPU6050_INT_CFG_VALUE); 	//Բ�㲩ʿ:����������PWMֵ
	BS004_I2C_STOP();
}

