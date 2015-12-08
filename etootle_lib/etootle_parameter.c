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
#include "etootle_parameter.h" 
//
//unsigned int bs004_load_par_sign=0;   //Բ�㲩ʿ:���ھ�����flashװ�ط��в���(=0)������ʹ�ô���Ĭ�ϲ���(=1)
unsigned int bs004_load_par_sign=1;   //Բ�㲩ʿ:���ھ�����flashװ�ط��в���(=0)������ʹ�ô���Ĭ�ϲ���(=1)
//���ط��в���
void BS004_Load_Fly_Parameter(void)
{
	BS004_COM1_Send_Str_Head();					//���ô���ͨ�ź���������Ϊ�����ַ���D-
	if(bs004_load_par_sign==0) BS004_COM1_Send_Str_Body("load parameter from flash.");      //Բ�㲩ʿ:��flashװ�ز���   
	else if(bs004_load_par_sign==1) BS004_COM1_Send_Str_Body("load default parameter .");   //Բ�㲩ʿ:ʹ�ô���Ĭ�ϲ���
	BS004_COM1_Send_Str_Tail();				//���ô���ͨ�ź���������Ϊ�����ַ� / 13
	//
	BS004_Load_Filter_Parameter();			//Բ�㲩ʿ:װ�ز��� �˲�
	BS004_Load_Pid_Parameter();         //Բ�㲩ʿ:װ�ز���	PID
	BS004_Load_Calibration_Parameter(); //Բ�㲩ʿ:װ�ز���	У׼
}
//װ���˲�����
void BS004_Load_Filter_Parameter(void)
{
	int bs004_filter_par[12];						//�������Ĭ�ϲ���
	unsigned char i;										//�ַ�����
	unsigned int *bs004_parameter_addr;//��ַ����
	unsigned int bs004_parameter_data,bs004_parameter_sign,bs004_parameter_valid; //���ݱ���  ��־λ����  ��Чλ����
	unsigned char bs004_parameter_byte_data[32];			//����һ���ַ�����
	//	
	if(bs004_load_par_sign==0)			//Բ�㲩ʿ:��flashװ�ز���   
	{
		for(i=0;i<7;i++) 											
		{
			bs004_parameter_addr=(unsigned int *)(0x803F400+i*4);				//Բ�㲩ʿ:���µ�ַ
			bs004_parameter_data=(unsigned int)*(bs004_parameter_addr);	//Բ�㲩ʿ:�ӵ�ַ��ȡ����
			//bs004_parameter_byte_data[i*4]ȡbs004_parameter_data���ݵ�0-7λ
			bs004_parameter_byte_data[i*4]=bs004_parameter_data & 0xff;
			//bs004_parameter_byte_data[i*4+1]ȡbs004_parameter_data���ݵ�8-15λ
			bs004_parameter_byte_data[i*4+1]=(bs004_parameter_data>>8) & 0xff;
			//bs004_parameter_byte_data[i*4+2]ȡbs004_parameter_data���ݵ�16-23λ
			bs004_parameter_byte_data[i*4+2]=(bs004_parameter_data>>16) & 0xff;
			//bs004_parameter_byte_data[i*4+3]ȡbs004_parameter_data���ݵ�24-31λ
			bs004_parameter_byte_data[i*4+3]=(bs004_parameter_data>>24) & 0xff;			
		}
		//bs004_parameter_signȡbs004_parameter_data���ݵ�0-15λ
		bs004_parameter_sign=bs004_parameter_data&0x0ffff;
		//bs004_parameter_signȡbs004_parameter_data���ݵ�16-31λ		
		bs004_parameter_valid=(bs004_parameter_data&0x0ffff0000)>>16;
		//�����ж���䣬����˵������׼ȷ�Ľ����������
		if(bs004_parameter_valid==0x4c46)
		{
			for(i=0;i<12;i++)
			{
//�õ�bs004_parameter_byte_dataż��λ�ĵ�4λ�͵�7λ��Ȼ������4λ�������������1000��������i�����󣬻�ȡ��bs004_parameter_byte_data����λ�õ�����
				bs004_filter_par[i]=((bs004_parameter_byte_data[i*2]&0xf0)>>4)*1000;
//�õ�bs004_parameter_byte_dataż��λ�ĵ�0λ�͵�3λ�������������100��.�ټ�����һ�����Լ�		
				bs004_filter_par[i]+=(bs004_parameter_byte_data[i*2]&0x0f)*100;
//�õ�bs004_parameter_byte_data����λ�ĵ�4λ�͵�7λ��Ȼ������4λ�������������10��.�ټ�����һ�����Լ�				
				bs004_filter_par[i]+=((bs004_parameter_byte_data[i*2+1]&0xf0)>>4)*10;
//�õ�bs004_parameter_byte_data����λ�ĵ�0λ�͵�3λ���ټ�����һ�����Լ�		
				bs004_filter_par[i]+=bs004_parameter_byte_data[i*2+1]&0x0f;
			}
//�ж�bs004_parameter_sign�ĵ�4λ����7λ�Ƿ�Ϊ0xd0���жϷ���������Ϊ0���Ǹ�����ǰ����ӷ���
			if((bs004_parameter_sign&0x0f0)==0xd0)
			{
				//�ж�bs004_parameter_sign��8λ�Ƿ�Ϊ0��������ǰ��Ӹ���
				if((bs004_parameter_sign&0x0100)==0) bs004_filter_par[0]=-bs004_filter_par[0];
				//�ж�bs004_parameter_sign��9λ�Ƿ�Ϊ0��������ǰ��Ӹ���				
				if((bs004_parameter_sign&0x0200)==0) bs004_filter_par[1]=-bs004_filter_par[1];		
				//�ж�bs004_parameter_sign��10λ�Ƿ�Ϊ0��������ǰ��Ӹ���				
				if((bs004_parameter_sign&0x0400)==0) bs004_filter_par[2]=-bs004_filter_par[2];	
				if((bs004_parameter_sign&0x0800)==0) bs004_filter_par[3]=-bs004_filter_par[3];	
				if((bs004_parameter_sign&0x1000)==0) bs004_filter_par[4]=-bs004_filter_par[4];	
				if((bs004_parameter_sign&0x2000)==0) bs004_filter_par[5]=-bs004_filter_par[5];	
				if((bs004_parameter_sign&0x4000)==0) bs004_filter_par[6]=-bs004_filter_par[6];	
				if((bs004_parameter_sign&0x8000)==0) bs004_filter_par[7]=-bs004_filter_par[7];	
				//�ж�bs004_parameter_sign��0λ�Ƿ�Ϊ0��������ǰ��Ӹ���				
				if((bs004_parameter_sign&0x0001)==0) bs004_filter_par[8]=-bs004_filter_par[8];	
				if((bs004_parameter_sign&0x0002)==0) bs004_filter_par[9]=-bs004_filter_par[9];
				if((bs004_parameter_sign&0x0004)==0) bs004_filter_par[10]=-bs004_filter_par[10];
				if((bs004_parameter_sign&0x0008)==0) bs004_filter_par[11]=-bs004_filter_par[11];			
			}	
		}
	}
	else			//Բ�㲩ʿ:ʹ�ô���Ĭ�ϲ���
	{
		bs004_filter_par[0]=950;
		bs004_filter_par[1]=50;
		bs004_filter_par[2]=1;
		bs004_filter_par[3]=1000;
		bs004_filter_par[4]=1640;
		bs004_filter_par[5]=5730;
		bs004_filter_par[6]=1000;
		bs004_filter_par[7]=1000;
		bs004_filter_par[8]=36;
		bs004_filter_par[9]=1;
		bs004_filter_par[10]=1600;
		bs004_filter_par[11]=1;		
	}
	//
	bs004_filter_high=(float)bs004_filter_par[0]/1000.0f;						//Բ�㲩ʿ:�˲�����
	bs004_filter_low=(float)bs004_filter_par[1]/1000.0f;						//Բ�㲩ʿ:�˲�����	
	bs004_filter_time=(float)bs004_filter_par[2]/1000.0f;						//Բ�㲩ʿ:�˲�����		
	bs004_sys_timer_period=(unsigned int)bs004_filter_par[3]-1; 		//Բ�㲩ʿ:����������Ƶ��
	bs004_mpu6050_gyro_scale=(float)bs004_filter_par[4]/100.0f;			//Բ�㲩ʿ:������������
	bs004_mpu6050_pi_scale=(float)bs004_filter_par[5]/100.0f;			  //Բ�㲩ʿ:����ϵ��
	bs004_hmc5883l_mag_scale=(float)bs004_filter_par[6]/1000.0f;		//Բ�㲩ʿ:������������
	bs004_motor_pwm_period=(unsigned int)bs004_filter_par[7];	  		//Բ�㲩ʿ:�����������
	BS004_Motor_Scale=(unsigned int)bs004_filter_par[8];          	//Բ�㲩ʿ:�������������
	bs004_quad_halfT=(float)bs004_filter_par[9]/1000.0f;						//Բ�㲩ʿ:��Ԫ��ʱ��ϵ��
	bs004_quad_Kp=(float)bs004_filter_par[10]/1000.0f;							//Բ�㲩ʿ:��Ԫ������ϵ��
	bs004_quad_Ki=(float)bs004_filter_par[11]/1000.0f;							//Բ�㲩ʿ:��Ԫ������ϵ��	
}

//����PID����
void BS004_Load_Pid_Parameter(void)
{
	int bs004_pid_par[12];
	unsigned char i;
	unsigned int *bs004_parameter_addr;
	unsigned int bs004_parameter_data,bs004_parameter_sign,bs004_parameter_valid;
	unsigned char bs004_parameter_byte_data[32];
	//	
	if(bs004_load_par_sign==0)			//Բ�㲩ʿ:��flashװ�ز���   
	{
		for(i=0;i<7;i++) 											
		{
			bs004_parameter_addr=(unsigned int *)(0x803F41C+i*4);				//Բ�㲩ʿ:���µ�ַ
			bs004_parameter_data=(unsigned int)*(bs004_parameter_addr);	//Բ�㲩ʿ:�ӵ�ַ��ȡ����
			//ȡbs004_parameter_byte_data�ĵ�0λ����7λ
			bs004_parameter_byte_data[i*4]=bs004_parameter_data & 0xff;
			//ȡbs004_parameter_byte_data�ĵ�8λ����15λ			
			bs004_parameter_byte_data[i*4+1]=(bs004_parameter_data>>8) & 0xff;
			//ȡbs004_parameter_byte_data�ĵ�16λ����23λ			
			bs004_parameter_byte_data[i*4+2]=(bs004_parameter_data>>16) & 0xff;
			//ȡbs004_parameter_byte_data�ĵ�24λ����31λ			
			bs004_parameter_byte_data[i*4+3]=(bs004_parameter_data>>24) & 0xff;			
		}
		//bs004_parameter_signȡbs004_parameter_data��0-15λ
		bs004_parameter_sign=bs004_parameter_data&0x0ffff;
		//bs004_parameter_signȡbs004_parameter_data��16-31λ		
		bs004_parameter_valid=(bs004_parameter_data&0x0ffff0000)>>16;
		//
		if(bs004_parameter_valid==0x4450)
		{
			//bs004_load_par_signʹ�ú��1
			bs004_load_par_sign++;
			for(i=0;i<12;i++)
			{
//�õ�bs004_parameter_byte_dataż��λ���ݵĵ�4λ����7λ��Ȼ������4λ�������������1000��������i�����󣬻�ȡ��bs004_parameter_byte_data����λ�õ�����				
				bs004_pid_par[i]=((bs004_parameter_byte_data[i*2]&0xf0)>>4)*1000;
//�õ�bs004_parameter_byte_dataż��λ���ݵĵ�0λ����3λ�������������100�����ټ�����һ�����Լ�
				bs004_pid_par[i]+=(bs004_parameter_byte_data[i*2]&0x0f)*100;
//�õ�bs004_parameter_byte_data����λ���ݵĵ�4λ����7λ��Ȼ������4λ�������������10�����ټ�����һ�����Լ�
				bs004_pid_par[i]+=((bs004_parameter_byte_data[i*2+1]&0xf0)>>4)*10;
//�õ�bs004_parameter_byte_data����λ���ݵĵ�0λ����3λ���ټ�����һ�����Լ�				
				bs004_pid_par[i]+=bs004_parameter_byte_data[i*2+1]&0x0f;
			}
//�ж�bs004_parameter_sign�ĵ�4λ����7λ�Ƿ�Ϊ0xd0���жϷ���������Ϊ0���Ǹ�����ǰ����ӷ���			
			if((bs004_parameter_sign&0x0f0)==0xd0)
			{
//�ж�bs004_parameter_sign�ĵ�8λ�Ƿ�Ϊ0��������ǰ��Ӹ���				
				if((bs004_parameter_sign&0x0100)==0) bs004_pid_par[0]=-bs004_pid_par[0];
				if((bs004_parameter_sign&0x0200)==0) bs004_pid_par[1]=-bs004_pid_par[1];		
				if((bs004_parameter_sign&0x0400)==0) bs004_pid_par[2]=-bs004_pid_par[2];	
				if((bs004_parameter_sign&0x0800)==0) bs004_pid_par[3]=-bs004_pid_par[3];	
				if((bs004_parameter_sign&0x1000)==0) bs004_pid_par[4]=-bs004_pid_par[4];	
				if((bs004_parameter_sign&0x2000)==0) bs004_pid_par[5]=-bs004_pid_par[5];	
				if((bs004_parameter_sign&0x4000)==0) bs004_pid_par[6]=-bs004_pid_par[6];	
				if((bs004_parameter_sign&0x8000)==0) bs004_pid_par[7]=-bs004_pid_par[7];	
//�ж�bs004_parameter_sign�ĵ�1λ�Ƿ�Ϊ0��������ǰ��Ӹ���					
				if((bs004_parameter_sign&0x0001)==0) bs004_pid_par[8]=-bs004_pid_par[8];	
				if((bs004_parameter_sign&0x0002)==0) bs004_pid_par[9]=-bs004_pid_par[9];
				if((bs004_parameter_sign&0x0004)==0) bs004_pid_par[10]=-bs004_pid_par[10];
				if((bs004_parameter_sign&0x0008)==0) bs004_pid_par[11]=-bs004_pid_par[11];			
			}	
		}
	}
	else		//Բ�㲩ʿ:ʹ�ô���Ĭ�ϲ���
	{
		bs004_pid_par[0]=150;
		bs004_pid_par[1]=150;
		bs004_pid_par[2]=150;
		bs004_pid_par[3]=0;
		bs004_pid_par[4]=0;
		bs004_pid_par[5]=0;
		bs004_pid_par[6]=250;
		bs004_pid_par[7]=250;
		bs004_pid_par[8]=500;
		bs004_pid_par[9]=6;
		bs004_pid_par[10]=15;
		bs004_pid_par[11]=20;		
	}
	//
	for(i=0;i<3;i++) pid_setting_P_value[i]=(unsigned int)bs004_pid_par[i];		//Բ�㲩ʿ:P����
	for(i=0;i<3;i++) pid_setting_I_value[i]=(unsigned int)bs004_pid_par[i+3]; //Բ�㲩ʿ:I����
	for(i=0;i<3;i++) pid_setting_D_value[i]=(unsigned int)bs004_pid_par[i+6]; //Բ�㲩ʿ:D����
	for(i=0;i<3;i++) pid_setting_M_value[i]=0;                                //Բ�㲩ʿ:��е���ĵ��������
	bs004_fly_gas_scale=(float)bs004_pid_par[9]/1.0f;													//�������Ų���
	//ŷ���ǲ������ֱ�Ϊ����ϵ�������ϵ��������ϵ��
	bs004_fly_pitch_scale=0.0f;
	bs004_fly_roll_scale=0.0f;
	bs004_fly_yaw_scale=0.0f;
}
//����У׼ϵ��
void BS004_Load_Calibration_Parameter(void)
{
	int bs004_cal_par[12];
	unsigned char i;
	unsigned int *bs004_parameter_addr;
	unsigned int bs004_parameter_data,bs004_parameter_sign,bs004_parameter_valid;
	unsigned char bs004_parameter_byte_data[32];
	//	
	if(bs004_load_par_sign==0)			//Բ�㲩ʿ:��flashװ�ز���   
	{
		for(i=0;i<7;i++) 											
		{
			bs004_parameter_addr=(unsigned int *)(0x803F438+i*4);				//Բ�㲩ʿ:���µ�ַ
			bs004_parameter_data=(unsigned int)*(bs004_parameter_addr);	//Բ�㲩ʿ:�ӵ�ַ��ȡ����
			//��ȡbs004_parameter_byte_data�ĵ�0����7λ����ͬ��
			bs004_parameter_byte_data[i*4]=bs004_parameter_data & 0xff;
			bs004_parameter_byte_data[i*4+1]=(bs004_parameter_data>>8) & 0xff;
			bs004_parameter_byte_data[i*4+2]=(bs004_parameter_data>>16) & 0xff;
			bs004_parameter_byte_data[i*4+3]=(bs004_parameter_data>>24) & 0xff;			
		}
		//bs004_parameter_signȡbs004_parameter_data��0-15λ
		bs004_parameter_sign=bs004_parameter_data&0x0ffff;
		bs004_parameter_valid=(bs004_parameter_data&0x0ffff0000)>>16;
		//
		if(bs004_parameter_valid==0x4c43)
		{			
			bs004_load_par_sign++;
			//
			for(i=0;i<12;i++)
			{
				bs004_cal_par[i]=((bs004_parameter_byte_data[i*2]&0xf0)>>4)*1000;
				bs004_cal_par[i]+=(bs004_parameter_byte_data[i*2]&0x0f)*100;
				bs004_cal_par[i]+=((bs004_parameter_byte_data[i*2+1]&0xf0)>>4)*10;
				bs004_cal_par[i]+=bs004_parameter_byte_data[i*2+1]&0x0f;
			}
			//
			if((bs004_parameter_sign&0x0f0)==0xd0)
			{
				if((bs004_parameter_sign&0x0100)==0) bs004_cal_par[0]=-bs004_cal_par[0];
				if((bs004_parameter_sign&0x0200)==0) bs004_cal_par[1]=-bs004_cal_par[1];		
				if((bs004_parameter_sign&0x0400)==0) bs004_cal_par[2]=-bs004_cal_par[2];	
				if((bs004_parameter_sign&0x0800)==0) bs004_cal_par[3]=-bs004_cal_par[3];	
				if((bs004_parameter_sign&0x1000)==0) bs004_cal_par[4]=-bs004_cal_par[4];	
				if((bs004_parameter_sign&0x2000)==0) bs004_cal_par[5]=-bs004_cal_par[5];	
				if((bs004_parameter_sign&0x4000)==0) bs004_cal_par[6]=-bs004_cal_par[6];	
				if((bs004_parameter_sign&0x8000)==0) bs004_cal_par[7]=-bs004_cal_par[7];	
				if((bs004_parameter_sign&0x0001)==0) bs004_cal_par[8]=-bs004_cal_par[8];	
				if((bs004_parameter_sign&0x0002)==0) bs004_cal_par[9]=-bs004_cal_par[9];
				if((bs004_parameter_sign&0x0004)==0) bs004_cal_par[10]=-bs004_cal_par[10];
				if((bs004_parameter_sign&0x0008)==0) bs004_cal_par[11]=-bs004_cal_par[11];			
			}			
		}
	}
	else	//Բ�㲩ʿ:ʹ�ô���Ĭ�ϲ���
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("start to do self calibration.");      //Բ�㲩ʿ:���ҽ������������е���λ�ã�������Ӧ�ñ���ˮƽ��
		BS004_COM1_Send_Str_Tail();
		BS004_Long_Delay(300000);	
		//
		BS004_Cal_MPU6050_Data(bs004_cal_par);
	}
	//
	bs004_mpu6050_gyro_pitch_cal=(signed short int)bs004_cal_par[0];	//Բ�㲩ʿ:������У�����
	bs004_mpu6050_gyro_roll_cal=(signed short int)bs004_cal_par[1];   //Բ�㲩ʿ:������У�����
	bs004_mpu6050_gyro_yaw_cal=(signed short int)bs004_cal_par[2];    //Բ�㲩ʿ:������У�����
	bs004_mpu6050_acc_roll_cal=(signed short int)bs004_cal_par[3];		//Բ�㲩ʿ:���ٶ�У�����
	bs004_mpu6050_acc_pitch_cal=(signed short int)bs004_cal_par[4];   //Բ�㲩ʿ:���ٶ�У�����
	bs004_mpu6050_acc_yaw_cal=(signed short int)bs004_cal_par[5];     //Բ�㲩ʿ:���ٶ�У�����
	bs004_hmc5883l_mag_pitch_cal=(signed short int)bs004_cal_par[6];  //Բ�㲩ʿ:������У�����
	bs004_hmc5883l_mag_roll_cal=(signed short int)bs004_cal_par[7];   //Բ�㲩ʿ:������У�����
	bs004_hmc5883l_mag_yaw_cal=(signed short int)bs004_cal_par[8];    //Բ�㲩ʿ:������У�����
	//���ô���ͨ�ź����������ַ� D-
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("gyro pitch cal= .");
	BS004_COM1_Send_4bits_BCD_Num(bs004_mpu6050_gyro_pitch_cal);
	//���ô���ͨ�ź����������ַ� /13
	BS004_COM1_Send_Str_Tail();
	BS004_Long_Delay(300000);	
	//
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("gyro roll cal= .");
	BS004_COM1_Send_4bits_BCD_Num(bs004_mpu6050_gyro_roll_cal);
	BS004_COM1_Send_Str_Tail();
	BS004_Long_Delay(300000);	
	//
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("gyro yaw cal= .");
	BS004_COM1_Send_4bits_BCD_Num(bs004_mpu6050_gyro_yaw_cal);
	BS004_COM1_Send_Str_Tail();
	BS004_Long_Delay(300000);	
	//
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("acc roll cal= .");
	BS004_COM1_Send_4bits_BCD_Num(bs004_mpu6050_acc_roll_cal);
	BS004_COM1_Send_Str_Tail();	
	BS004_Long_Delay(300000);	
	//
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("acc pitch cal= .");
	BS004_COM1_Send_4bits_BCD_Num(bs004_mpu6050_acc_pitch_cal);
	BS004_COM1_Send_Str_Tail();	
	BS004_Long_Delay(300000);	
	//
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("acc yaw cal= .");
	BS004_COM1_Send_4bits_BCD_Num(bs004_mpu6050_acc_yaw_cal);
	BS004_COM1_Send_Str_Tail();	
	BS004_Long_Delay(300000);	
	//
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("finish the self calibration.");      //Բ�㲩ʿ:�������ҽ���
	BS004_COM1_Send_Str_Tail();
	BS004_Long_Delay(300000);	
}

//���������ǲ���
void BS004_Cal_MPU6050_Data(int *bs004_cal_data)   
{
	unsigned char i,j;
	unsigned char bs004_mpu6050_cal_data_buffer[14];
	signed short int bs004_mpu6050_cal_data[100][7];
	int bs004_mpu6050_cal_sum[7];
	//ѭ����䣬��¼У׼���������ݴ���
	for(i=0;i<7;i++) bs004_mpu6050_cal_sum[i]=0;
	//
	for(j=0;j<100;j++)
	{
		while(BS004_MPU6050_INT_STATE);
		//����ж�����
		BS004_I2C_START();																				//�������亯��
		BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);							//Բ�㲩ʿ:����������д��ַ
		BS004_I2C_SendByte(BS004_MPU6050_ACCEL_DATA_ADDR);    		//Բ�㲩ʿ:���������ǼĴ�����ַ
		BS004_I2C_START();
		BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR+1);      			//Բ�㲩ʿ:���������Ƕ���ַ
		BS004_I2C_Receive14Bytes(bs004_mpu6050_cal_data_buffer);	//Բ�㲩ʿ:���������ǼĴ���ֵ
		BS004_I2C_STOP();																					//ֹͣ���亯��
		//ѭ����䣬�bs004_mpu6050_cal_data_buffer��λ��bs004_mpu6050_cal_data_buffer Ȼ�����bs004_mpu6050_cal_data		
		for(i=0;i<7;i++) bs004_mpu6050_cal_data[j][i]=(((signed short int)bs004_mpu6050_cal_data_buffer[i*2]) << 8) | bs004_mpu6050_cal_data_buffer[i*2+1];
	}
	//
	for(i=0;i<7;i++)
	{
		for(j=0;j<100;j++) bs004_mpu6050_cal_sum[i]=bs004_mpu6050_cal_sum[i]+(int)bs004_mpu6050_cal_data[j][i];
	}
	//�����ݻָ�����ֵ�����bs004_cal_data
	bs004_cal_data[0]=bs004_mpu6050_cal_sum[4]/100;
	bs004_cal_data[1]=bs004_mpu6050_cal_sum[5]/100;
	bs004_cal_data[2]=bs004_mpu6050_cal_sum[6]/100;
	bs004_cal_data[3]=bs004_mpu6050_cal_sum[0]/100;
	bs004_cal_data[4]=bs004_mpu6050_cal_sum[1]/100;
	bs004_cal_data[5]=bs004_mpu6050_cal_sum[2]/100;;
	//�ò����������0
	bs004_cal_data[6]=0;
	bs004_cal_data[7]=0;
	bs004_cal_data[8]=0;
}

//����У׼����
void BS004_Show_Calibrated_Data(void)
{
	unsigned char i;
	unsigned char bs004_mpu6050_caled_data_buffer[14];
	signed short int bs004_mpu6050_caled_data[7];
	int bs004_mpu6050_acc_roll_caled,bs004_mpu6050_acc_pitch_caled,bs004_mpu6050_acc_yaw_caled;
	int bs004_mpu6050_gyro_pitch_caled,bs004_mpu6050_gyro_roll_caled,bs004_mpu6050_gyro_yaw_caled;
	//
	while(BS004_MPU6050_INT_STATE);
	//
	BS004_I2C_START();																					//�������亯��
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);								//Բ�㲩ʿ:����������д��ַ
	BS004_I2C_SendByte(BS004_MPU6050_ACCEL_DATA_ADDR);    			//Բ�㲩ʿ:���������ǼĴ�����ַ
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR+1);      				//Բ�㲩ʿ:���������Ƕ���ַ
	BS004_I2C_Receive14Bytes(bs004_mpu6050_caled_data_buffer);	//Բ�㲩ʿ:���������ǼĴ���ֵ
	BS004_I2C_STOP();																						//ֹͣ���亯��
	//				
	for(i=0;i<7;i++) bs004_mpu6050_caled_data[i]=(((signed short int)bs004_mpu6050_caled_data_buffer[i*2]) << 8) | bs004_mpu6050_caled_data_buffer[i*2+1];
	bs004_mpu6050_acc_roll_caled=bs004_mpu6050_caled_data[0]-bs004_mpu6050_acc_roll_cal;
	bs004_mpu6050_acc_pitch_caled=bs004_mpu6050_caled_data[1]-bs004_mpu6050_acc_pitch_cal;
	bs004_mpu6050_acc_yaw_caled=bs004_mpu6050_caled_data[2];
	bs004_mpu6050_gyro_pitch_caled=bs004_mpu6050_caled_data[4]-bs004_mpu6050_gyro_pitch_cal;
	bs004_mpu6050_gyro_roll_caled=bs004_mpu6050_caled_data[5]-bs004_mpu6050_gyro_roll_cal;;
	bs004_mpu6050_gyro_yaw_caled=bs004_mpu6050_caled_data[6]-bs004_mpu6050_gyro_yaw_cal;
	//
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("begin to show mpu6050 data.");      
	BS004_COM1_Send_Str_Tail();
	BS004_Long_Delay(300000);	
	//
		BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("gyro pitch= .");
	BS004_COM1_Send_4bits_BCD_Num(bs004_mpu6050_gyro_pitch_caled);
	BS004_COM1_Send_Str_Tail();
	BS004_Long_Delay(300000);	
	//
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("gyro roll= .");
	BS004_COM1_Send_4bits_BCD_Num(bs004_mpu6050_gyro_roll_caled);
	BS004_COM1_Send_Str_Tail();
	BS004_Long_Delay(300000);	
	//
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("gyro yaw= .");
	BS004_COM1_Send_4bits_BCD_Num(bs004_mpu6050_gyro_yaw_caled);
	BS004_COM1_Send_Str_Tail();
	BS004_Long_Delay(300000);	
	//
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("acc roll= .");
	BS004_COM1_Send_4bits_BCD_Num(bs004_mpu6050_acc_roll_caled);
	BS004_COM1_Send_Str_Tail();	
	BS004_Long_Delay(300000);	
	//
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("acc pitch= .");
	BS004_COM1_Send_4bits_BCD_Num(bs004_mpu6050_acc_pitch_caled);
	BS004_COM1_Send_Str_Tail();	
	BS004_Long_Delay(300000);	
	//
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("acc yaw= .");
	BS004_COM1_Send_4bits_BCD_Num(bs004_mpu6050_acc_yaw_caled);
	BS004_COM1_Send_Str_Tail();	
	BS004_Long_Delay(300000);	
	//
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("finish to show mpu6050 data.");     
	BS004_COM1_Send_Str_Tail();
	BS004_Long_Delay(300000);	
}

