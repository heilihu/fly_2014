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
#include "etootle_main.h"
//����Ϊ ϵͳ���м��� ϵͳ1ms�¼���־ ϵͳ������״̬��־ ϵͳ״̬����
unsigned int system_idle_counter=0,system_timer_1ms_event=0,system_timer_state=0,system_status=0;
//����Ϊ ϵͳ����������ֵ ϵͳLED����������ֵ
unsigned int system_timer_counter=0,system_led_timer_counter=0;
//���յ�������
extern unsigned char bs004_com_command_ops;
//���ڿ��б�־
extern unsigned char com_status_is_idle;	
//����Ϊ ���ſ��Ʋ��� ���Ʋ�����Ч��־
extern unsigned char BS004_Ctrl_Gas,BS004_Ctrl_Valid;
//����Ϊ �������Ʋ��� ������Ʋ��� ������Ʋ���
extern signed char   BS004_Ctrl_Pitch,BS004_Ctrl_Roll,BS004_Ctrl_Yaw;
//����Ϊ �ĸ���������Ų����Լ����Ų����Ŀ��Ʊ���
extern unsigned int Motor_BS004_M4,Motor_BS004_M2,Motor_BS004_M3,Motor_BS004_M1,BS004_Motor_Scale;
//
int main()
{
	BS004_RCC_Configuration();				//Բ�㲩ʿ:ʱ������ �趨ϵͳʱ��
	BS004_NVIC_Configuration();     	//Բ�㲩ʿ:�ж����� ��ʼ��ϵͳ�ж�
	//
	BS004_SYS_LED_Configuration();    //Բ�㲩ʿ:LED����	��ʼ��ϵͳLED
	BS004_SYS_Timer_Configuration();  //Բ�㲩ʿ:ϵͳʱ������	��ʼ��ϵͳ������
	BS004_LED_GPIO_Configuration();   //Բ�㲩ʿ:LED����	��ʼ��LED
	//
	BS004_COM1_GPIO_Configuration();	//Բ�㲩ʿ:��������	��ʼ��������ʹ�õĶ˿�
	BS004_COM1_Port_Configuration();  //Բ�㲩ʿ:��������	��ʼ������
	BS004_ADC_Configuration();        //Բ�㲩ʿ:��ѹ�������	��ʼ��ADC
	//
	BS004_MPU6050_Init();             //Բ�㲩ʿ:MPU6050��ʼ��
	BS004_Load_Fly_Parameter();			  //Բ�㲩ʿ:װ�ز���
	BS004_Show_Calibrated_Data();     //Բ�㲩ʿ:���У����MPU6050����
	//
	BS004_Motor_GPIO_Configuration(); //Բ�㲩ʿ:�������	��ʼ���������˿�
	BS004_Motor_PWM_Configuration(); 	//Բ�㲩ʿ:�������	��ʼ�����PWM���
	//
	//״̬��ѭ��	���ڳ����ܳ���main������״̬����ѭ��������
	while(1)
	{
		if(system_timer_1ms_event)			//Բ�㲩ʿ:1MS�¼����� 1msʱ���־���ñ�־λÿ1ms��1һ�Σ���1ms����״̬��һ��
		{	
			system_timer_1ms_event=0;			//Բ�㲩ʿ:���1ms�¼���־
			system_timer_counter++;       //Բ�㲩ʿ:ϵͳ�������Լ�
			system_led_timer_counter++;   //Բ�㲩ʿ:LED����˸�������Լ�
			//����״̬����״ִ̬����Ӧ����
			switch (system_timer_state)
			{
				case IDLE:									//Բ�㲩ʿ:����״̬ ������ĳ�ʼ��״̬���ȴ������������ȶ�
					system_idle_counter++;		//���м������Լ�
					BS004_Get_MPU6050_Data();	//��ȡ����������
					//����5�μ�����5ms��״̬��״̬����Ϊ���Ӵ��ڸ���ָ��
					if(system_idle_counter>5) system_timer_state=UPDATE_COMMAND;
					break;
					//�Ӵ��ڸ���ָ��
				case UPDATE_COMMAND:				//Բ�㲩ʿ:��ȡMPU6050���ݺʹ�������
					BS004_Get_MPU6050_Data();	//��ȡ����������
					system_status=BS004_COM1_Task_Process();	//���ڻ�ȡ�Ӵ��ڸ�������
					//������Ŵ���0��״̬������Ϊ��UPDATE_MOTOR
					if(system_status) system_timer_state=UPDATE_MOTOR;
					//�������Ϊ0��״̬������Ϊ��RESET_MOTOR
					else system_timer_state=RESET_MOTOR;
					break;

				case UPDATE_MOTOR:					//Բ�㲩ʿ:���µ�����
					BS004_Get_MPU6050_Data();	//��ȡ����������
					BS004_Quad_Calculation();	// ��Ԫ������
				  BS004_PID_Control();			//PID���Ƽ���
					BS004_Motor_Control();		//������
					//״̬��״̬���£��Ӵ��ڸ�������
					system_timer_state=UPDATE_COMMAND;
					break;

				case RESET_MOTOR:          //Բ�㲩ʿ:��λ������
					BS004_Get_MPU6050_Data();	//��ȡ����������
				  BS004_Quad_Calculation();	//��Ԫ������
					BS004_Motor_Reset();			//����������
					//״̬��״̬����Ϊ���Ӵ��ڸ�������
					system_timer_state=UPDATE_COMMAND;
					break;
					//�������������
				default:
					break;
			}
		}
		else
		{			
			BS004_SYS_LED_TWINKLE();   //Բ�㲩ʿ:LED����˸
		}
	}	
}
//
void BS004_Quad_Calculation(void)
{
	float ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
	//����Ϊx,y,z��ļ��ٶȻ����x,y,z��Ľ��ٶȻ���
	bs004_gyro_to_rad_scale=bs004_mpu6050_pi_scale*bs004_mpu6050_gyro_scale;
	//	������ת���ȱ���ϵ��=����ϵ����ʵ��ֵΪ57.3f(57.3/rad))*������������(ʵ��ֵΪ16.4(�����ǵ�λ))
	
	//MPU6050�˲��������/������ת���ȱ���ϵ��    ʵ���˽����ٶ�(������)ת���ɻ��ٶ�
	gx=bs004_mpu6050_gyro_angel_pitch_ave/bs004_gyro_to_rad_scale;
	gy=bs004_mpu6050_gyro_angel_roll_ave/bs004_gyro_to_rad_scale;
	gz=bs004_mpu6050_gyro_angel_yaw_ave/bs004_gyro_to_rad_scale;		
	//MPU6050�˲���ļ��ٶ�ֵ
	ax=bs004_mpu6050_acc_angel_roll_ave;
	ay=bs004_mpu6050_acc_angel_pitch_ave;	
	az=bs004_mpu6050_acc_angel_yaw_ave;		
	//���¹��Բ�����Ԫ�����ݼ�������̬
	BS004_IMU_Update(ax,ay,az,gx,gy,gz);
}
//�����¼��������
unsigned char BS004_COM1_Task_Process(void)
{
	//������յ�������������0xA0
	if(bs004_com_command_ops>0xA0)
	{
		com_status_is_idle=0;					//Բ�㲩ʿ:���ô���æ��־
		BS004_COM1_Communication();   //Բ�㲩ʿ:ִ������ ���ô���ͨ�Ż�Ӧ����
		bs004_com_command_ops=0;			//Բ�㲩ʿ:������������־
		com_status_is_idle=1;					//Բ�㲩ʿ:�������æ��־
	} 
	else if((bs004_com_command_ops==0xA0))//  respond to gas д��ң������
	{
		if(BS004_Ctrl_Valid==1)					//Բ�㲩ʿ:�����������Ƿ���Ч ��������źű�־��Ч
		{	
			com_status_is_idle=0;					//Բ�㲩ʿ:���ô���æ��־			
			//������ſ��Ʋ��� ���� ���ſ�����ֵ����ʵ������Ϊ ���ſ��Ʋ�����ȥ���ſ�����ֵ����֮ʵ������Ϊ0			
			if(BS004_Ctrl_Gas>BS004_Ctrl_Gas_Noise) bs004_fly_gas=BS004_Ctrl_Gas-BS004_Ctrl_Gas_Noise;
			else bs004_fly_gas=0;
			//����������Ʋ���������̬������ֵ����ʵ�ʸ���Ϊ �������Ʋ��� ��ȥ��̬������ֵ�� ��֮ʵ�ʸ���Ϊ0
			if(fabs(BS004_Ctrl_Pitch)>BS004_Ctrl_Dir_Noise) bs004_fly_pitch=BS004_Ctrl_Pitch;
			else bs004_fly_pitch=0;
			//������Ʋ���������̬������ֵ����ʵ�ʺ��Ϊ ������Ʋ��� ��ȥ��̬������ֵ�� ��֮ʵ�ʺ��Ϊ0
			if(fabs(BS004_Ctrl_Roll)>BS004_Ctrl_Dir_Noise) bs004_fly_roll=BS004_Ctrl_Roll;
			else bs004_fly_roll=0;
			//���ƫ�����Ʋ���������̬������ֵ����ʵ��ƫ��Ϊ ƫ�����Ʋ��� ��ȥ��̬������ֵ�� ��֮ʵ��ƫ��Ϊ0
			if(fabs(BS004_Ctrl_Yaw)>BS004_Ctrl_Dir_Noise) bs004_fly_yaw=BS004_Ctrl_Yaw;
			bs004_fly_yaw=0;
			//
			BS004_Ctrl_Valid=0;					//Բ�㲩ʿ:��������źű�־
			com_status_is_idle=1;				//Բ�㲩ʿ:�������æ��־
		}
	}	
	return bs004_fly_gas;		//�������Ŵ�С
}
//���������ƺ���
void BS004_Motor_Control(void)
{
	//�Ե�����PWMռ�ձȽ��б�������
	bs004_fly_m1_out=BS004_Motor_Speed_Scale(bs004_fly_m1);
	bs004_fly_m2_out=BS004_Motor_Speed_Scale(bs004_fly_m2);	
	bs004_fly_m3_out=BS004_Motor_Speed_Scale(bs004_fly_m3);
	bs004_fly_m4_out=BS004_Motor_Speed_Scale(bs004_fly_m4);	
	//����PWMռ�ձ�
	TIM_SetCompare4(TIM8,bs004_fly_m1_out);		//Բ�㲩ʿ:����PWMռ�ձ�  
	TIM_SetCompare2(TIM8,bs004_fly_m2_out);	  //Բ�㲩ʿ:����PWMռ�ձ�  
	TIM_SetCompare1(TIM8,bs004_fly_m3_out);   //Բ�㲩ʿ:����PWMռ�ձ�      
	TIM_SetCompare3(TIM8,bs004_fly_m4_out);		//Բ�㲩ʿ:����PWMռ�ձ� 
}
//���ڷ���ŷ������̬����
void ANBT_SEND_DMP_EULER_DATA(void)
{
	//����Ϊ������ʾ���ݣ������ʾ���ݣ�ƫ����ʾ����
	float bs004_display_pitch=0,bs004_display_roll=0,bs004_display_yaw=0;
	//����Ϊ���ݰ����� У��� ŷ���Ƿ��ű�־ ѭ�����ݼ���
	unsigned char data_type,checksum=0,euler_data_sign=0,i=0;
	//ŷ�������ݻ���
	unsigned int bs004_mpu6050_euler_data[3];
	//ŷ�������ݷ��ͻ�����
	unsigned char bs004_mpu6050_euler_data_buffer[6];
	//������ʾ����=��ȡ����IMU�������ݵ�100��	(���ⷢ�͸�����)
	bs004_display_pitch=bs004_imu_pitch*100;
	bs004_display_roll =bs004_imu_roll*100;
	bs004_display_yaw  =bs004_imu_yaw*100;
	
	//	���������ʾ���� С��0
	if(bs004_display_pitch<0) 
	{
		//��ŷ�������ݷ��ű�־�ĵڶ�λ��1(00000100),��ʾ������ʾ����Ϊ����
		euler_data_sign|=4;
		//�����ݽ���+18000��ƫ�ƣ�(���ⷢ�͸���)
		bs004_display_pitch+=18000;
	}
	if(bs004_display_roll<0) 
	{
		//��ŷ�������ݷ��ű�־�ĵ�һλ��1(00000010),��ʾ�����ʾ����Ϊ����
		euler_data_sign|=2;	
		bs004_display_roll+=18000;
	}
	if(bs004_display_yaw<0) 
	{
		//��ŷ�������ݷ��ű�־�ĵ���λ��1(00000001),��ʾƫ����ʾ����Ϊ����
		euler_data_sign|=1;		
		bs004_display_yaw+=18000;
	}
	//euler_data_sign�洢��ŷ�������ݵķ���λ
	
	//������ ��� ƫ�����ݷ�����ŷ�������ݻ���
	bs004_mpu6050_euler_data[0]=(unsigned int)bs004_display_pitch;
	bs004_mpu6050_euler_data[1]=(unsigned int)bs004_display_roll;	
	bs004_mpu6050_euler_data[2]=(unsigned int)bs004_display_yaw;	
	
	//ʵ���˶�16λŷ�������ݽ��в�ֲ�����ŷ�������ݷ��ͻ�����	��������
	//[������ʾ���ݸ߰�λ][������ʾ���ݵͰ�λ][�����ʾ���ݸ߰�λ][�����ʾ���ݵͰ�λ][ƫ����ʾ���ݸ߰�λ][ƫ����ʾ���ݵͰ�λ]
  for(i=0;i<3;i++) 
	{
		bs004_mpu6050_euler_data_buffer[i*2]=(bs004_mpu6050_euler_data[i]>>8)&0xff;
		bs004_mpu6050_euler_data_buffer[i*2+1]=bs004_mpu6050_euler_data[i]&0xff;
	}
	//��ŷ�������ݷ��ű�־����λ��Ϊ0xB0
	data_type=0xB0| euler_data_sign;
	//����У���
	checksum=data_type;
	for(i=0;i<6;i++) checksum+=bs004_mpu6050_euler_data_buffer[i];
	checksum&=0xff;
	checksum=~checksum;
	checksum++;
	//����ŷ��������
	//����������ݼĴ����ձ�־λ��Ч�����ͻ�����Ϊ��
	if(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==SET)
	{
		BS004_COM1_Send_Char(':');		//��������ͷ
		BS004_COM1_Send_Num(data_type);	//�������ݰ�����(����ŷ���Ƿ���)
		//����ŷ�������ݷ���������
		for(i=0;i<6;i++) BS004_COM1_Send_Num(bs004_mpu6050_euler_data_buffer[i]);			
		//���ͺ�У������
		BS004_COM1_Send_Num(checksum);
		//��������β
		BS004_COM1_Send_Char('/');
		//���ͻ��з�
		BS004_COM1_Send_Char('\n');
	}
}



