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
#include "etootle_parameter.h" 
//
//unsigned int bs004_load_par_sign=0;   //圆点博士:用于决定从flash装载飞行参数(=0)，还是使用代码默认参数(=1)
unsigned int bs004_load_par_sign=1;   //圆点博士:用于决定从flash装载飞行参数(=0)，还是使用代码默认参数(=1)
//加载飞行参数
void BS004_Load_Fly_Parameter(void)
{
	BS004_COM1_Send_Str_Head();					//调用串口通信函数，作用为发送字符：D-
	if(bs004_load_par_sign==0) BS004_COM1_Send_Str_Body("load parameter from flash.");      //圆点博士:从flash装载参数   
	else if(bs004_load_par_sign==1) BS004_COM1_Send_Str_Body("load default parameter .");   //圆点博士:使用代码默认参数
	BS004_COM1_Send_Str_Tail();				//调用串口通信函数，作用为发送字符 / 13
	//
	BS004_Load_Filter_Parameter();			//圆点博士:装载参数 滤波
	BS004_Load_Pid_Parameter();         //圆点博士:装载参数	PID
	BS004_Load_Calibration_Parameter(); //圆点博士:装载参数	校准
}
//装载滤波参数
void BS004_Load_Filter_Parameter(void)
{
	int bs004_filter_par[12];						//定义代码默认参数
	unsigned char i;										//字符变量
	unsigned int *bs004_parameter_addr;//地址变量
	unsigned int bs004_parameter_data,bs004_parameter_sign,bs004_parameter_valid; //数据变量  标志位变量  有效位变量
	unsigned char bs004_parameter_byte_data[32];			//定义一个字符数组
	//	
	if(bs004_load_par_sign==0)			//圆点博士:从flash装载参数   
	{
		for(i=0;i<7;i++) 											
		{
			bs004_parameter_addr=(unsigned int *)(0x803F400+i*4);				//圆点博士:更新地址
			bs004_parameter_data=(unsigned int)*(bs004_parameter_addr);	//圆点博士:从地址读取数据
			//bs004_parameter_byte_data[i*4]取bs004_parameter_data数据的0-7位
			bs004_parameter_byte_data[i*4]=bs004_parameter_data & 0xff;
			//bs004_parameter_byte_data[i*4+1]取bs004_parameter_data数据的8-15位
			bs004_parameter_byte_data[i*4+1]=(bs004_parameter_data>>8) & 0xff;
			//bs004_parameter_byte_data[i*4+2]取bs004_parameter_data数据的16-23位
			bs004_parameter_byte_data[i*4+2]=(bs004_parameter_data>>16) & 0xff;
			//bs004_parameter_byte_data[i*4+3]取bs004_parameter_data数据的24-31位
			bs004_parameter_byte_data[i*4+3]=(bs004_parameter_data>>24) & 0xff;			
		}
		//bs004_parameter_sign取bs004_parameter_data数据的0-15位
		bs004_parameter_sign=bs004_parameter_data&0x0ffff;
		//bs004_parameter_sign取bs004_parameter_data数据的16-31位		
		bs004_parameter_valid=(bs004_parameter_data&0x0ffff0000)>>16;
		//条件判断语句，等于说明数据准确的进入参数设置
		if(bs004_parameter_valid==0x4c46)
		{
			for(i=0;i<12;i++)
			{
//得到bs004_parameter_byte_data偶数位的第4位和第7位，然后右移4位，最后将数据扩大1000倍。随着i的增大，会取到bs004_parameter_byte_data后面位置的数据
				bs004_filter_par[i]=((bs004_parameter_byte_data[i*2]&0xf0)>>4)*1000;
//得到bs004_parameter_byte_data偶数位的第0位和第3位，最后将数据扩大100倍.再加上上一步的自己		
				bs004_filter_par[i]+=(bs004_parameter_byte_data[i*2]&0x0f)*100;
//得到bs004_parameter_byte_data奇数位的第4位和第7位，然后右移4位，最后将数据扩大10倍.再加上上一步的自己				
				bs004_filter_par[i]+=((bs004_parameter_byte_data[i*2+1]&0xf0)>>4)*10;
//得到bs004_parameter_byte_data奇数位的第0位和第3位，再加上上一步的自己		
				bs004_filter_par[i]+=bs004_parameter_byte_data[i*2+1]&0x0f;
			}
//判断bs004_parameter_sign的第4位至第7位是否为0xd0。判断符号正负，为0则是负，在前面添加符号
			if((bs004_parameter_sign&0x0f0)==0xd0)
			{
				//判断bs004_parameter_sign第8位是否为0，是则在前添加负号
				if((bs004_parameter_sign&0x0100)==0) bs004_filter_par[0]=-bs004_filter_par[0];
				//判断bs004_parameter_sign第9位是否为0，是则在前添加负号				
				if((bs004_parameter_sign&0x0200)==0) bs004_filter_par[1]=-bs004_filter_par[1];		
				//判断bs004_parameter_sign第10位是否为0，是则在前添加负号				
				if((bs004_parameter_sign&0x0400)==0) bs004_filter_par[2]=-bs004_filter_par[2];	
				if((bs004_parameter_sign&0x0800)==0) bs004_filter_par[3]=-bs004_filter_par[3];	
				if((bs004_parameter_sign&0x1000)==0) bs004_filter_par[4]=-bs004_filter_par[4];	
				if((bs004_parameter_sign&0x2000)==0) bs004_filter_par[5]=-bs004_filter_par[5];	
				if((bs004_parameter_sign&0x4000)==0) bs004_filter_par[6]=-bs004_filter_par[6];	
				if((bs004_parameter_sign&0x8000)==0) bs004_filter_par[7]=-bs004_filter_par[7];	
				//判断bs004_parameter_sign第0位是否为0，是则在前添加负号				
				if((bs004_parameter_sign&0x0001)==0) bs004_filter_par[8]=-bs004_filter_par[8];	
				if((bs004_parameter_sign&0x0002)==0) bs004_filter_par[9]=-bs004_filter_par[9];
				if((bs004_parameter_sign&0x0004)==0) bs004_filter_par[10]=-bs004_filter_par[10];
				if((bs004_parameter_sign&0x0008)==0) bs004_filter_par[11]=-bs004_filter_par[11];			
			}	
		}
	}
	else			//圆点博士:使用代码默认参数
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
	bs004_filter_high=(float)bs004_filter_par[0]/1000.0f;						//圆点博士:滤波参数
	bs004_filter_low=(float)bs004_filter_par[1]/1000.0f;						//圆点博士:滤波参数	
	bs004_filter_time=(float)bs004_filter_par[2]/1000.0f;						//圆点博士:滤波参数		
	bs004_sys_timer_period=(unsigned int)bs004_filter_par[3]-1; 		//圆点博士:传感器采样频率
	bs004_mpu6050_gyro_scale=(float)bs004_filter_par[4]/100.0f;			//圆点博士:陀螺仪灵敏度
	bs004_mpu6050_pi_scale=(float)bs004_filter_par[5]/100.0f;			  //圆点博士:弧度系数
	bs004_hmc5883l_mag_scale=(float)bs004_filter_par[6]/1000.0f;		//圆点博士:磁力计灵敏度
	bs004_motor_pwm_period=(unsigned int)bs004_filter_par[7];	  		//圆点博士:电机控制周期
	BS004_Motor_Scale=(unsigned int)bs004_filter_par[8];          	//圆点博士:电机控制灵敏度
	bs004_quad_halfT=(float)bs004_filter_par[9]/1000.0f;						//圆点博士:四元数时间系数
	bs004_quad_Kp=(float)bs004_filter_par[10]/1000.0f;							//圆点博士:四元数比例系数
	bs004_quad_Ki=(float)bs004_filter_par[11]/1000.0f;							//圆点博士:四元数积分系数	
}

//加载PID参数
void BS004_Load_Pid_Parameter(void)
{
	int bs004_pid_par[12];
	unsigned char i;
	unsigned int *bs004_parameter_addr;
	unsigned int bs004_parameter_data,bs004_parameter_sign,bs004_parameter_valid;
	unsigned char bs004_parameter_byte_data[32];
	//	
	if(bs004_load_par_sign==0)			//圆点博士:从flash装载参数   
	{
		for(i=0;i<7;i++) 											
		{
			bs004_parameter_addr=(unsigned int *)(0x803F41C+i*4);				//圆点博士:更新地址
			bs004_parameter_data=(unsigned int)*(bs004_parameter_addr);	//圆点博士:从地址读取数据
			//取bs004_parameter_byte_data的第0位至第7位
			bs004_parameter_byte_data[i*4]=bs004_parameter_data & 0xff;
			//取bs004_parameter_byte_data的第8位至第15位			
			bs004_parameter_byte_data[i*4+1]=(bs004_parameter_data>>8) & 0xff;
			//取bs004_parameter_byte_data的第16位至第23位			
			bs004_parameter_byte_data[i*4+2]=(bs004_parameter_data>>16) & 0xff;
			//取bs004_parameter_byte_data的第24位至第31位			
			bs004_parameter_byte_data[i*4+3]=(bs004_parameter_data>>24) & 0xff;			
		}
		//bs004_parameter_sign取bs004_parameter_data的0-15位
		bs004_parameter_sign=bs004_parameter_data&0x0ffff;
		//bs004_parameter_sign取bs004_parameter_data的16-31位		
		bs004_parameter_valid=(bs004_parameter_data&0x0ffff0000)>>16;
		//
		if(bs004_parameter_valid==0x4450)
		{
			//bs004_load_par_sign使用后加1
			bs004_load_par_sign++;
			for(i=0;i<12;i++)
			{
//得到bs004_parameter_byte_data偶数位数据的第4位至第7位，然后右移4位，最后将数据扩大1000倍，随着i的增大，会取到bs004_parameter_byte_data后面位置的数据				
				bs004_pid_par[i]=((bs004_parameter_byte_data[i*2]&0xf0)>>4)*1000;
//得到bs004_parameter_byte_data偶数位数据的第0位至第3位，最后将数据扩大100倍，再加上上一步的自己
				bs004_pid_par[i]+=(bs004_parameter_byte_data[i*2]&0x0f)*100;
//得到bs004_parameter_byte_data奇数位数据的第4位至第7位，然后右移4位，最后将数据扩大10倍，再加上上一步的自己
				bs004_pid_par[i]+=((bs004_parameter_byte_data[i*2+1]&0xf0)>>4)*10;
//得到bs004_parameter_byte_data奇数位数据的第0位至第3位，再加上上一步的自己				
				bs004_pid_par[i]+=bs004_parameter_byte_data[i*2+1]&0x0f;
			}
//判断bs004_parameter_sign的第4位至第7位是否为0xd0。判断符号正负，为0则是负，在前面添加符号			
			if((bs004_parameter_sign&0x0f0)==0xd0)
			{
//判断bs004_parameter_sign的第8位是否为0，是则在前添加负号				
				if((bs004_parameter_sign&0x0100)==0) bs004_pid_par[0]=-bs004_pid_par[0];
				if((bs004_parameter_sign&0x0200)==0) bs004_pid_par[1]=-bs004_pid_par[1];		
				if((bs004_parameter_sign&0x0400)==0) bs004_pid_par[2]=-bs004_pid_par[2];	
				if((bs004_parameter_sign&0x0800)==0) bs004_pid_par[3]=-bs004_pid_par[3];	
				if((bs004_parameter_sign&0x1000)==0) bs004_pid_par[4]=-bs004_pid_par[4];	
				if((bs004_parameter_sign&0x2000)==0) bs004_pid_par[5]=-bs004_pid_par[5];	
				if((bs004_parameter_sign&0x4000)==0) bs004_pid_par[6]=-bs004_pid_par[6];	
				if((bs004_parameter_sign&0x8000)==0) bs004_pid_par[7]=-bs004_pid_par[7];	
//判断bs004_parameter_sign的第1位是否为0，是则在前添加负号					
				if((bs004_parameter_sign&0x0001)==0) bs004_pid_par[8]=-bs004_pid_par[8];	
				if((bs004_parameter_sign&0x0002)==0) bs004_pid_par[9]=-bs004_pid_par[9];
				if((bs004_parameter_sign&0x0004)==0) bs004_pid_par[10]=-bs004_pid_par[10];
				if((bs004_parameter_sign&0x0008)==0) bs004_pid_par[11]=-bs004_pid_par[11];			
			}	
		}
	}
	else		//圆点博士:使用代码默认参数
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
	for(i=0;i<3;i++) pid_setting_P_value[i]=(unsigned int)bs004_pid_par[i];		//圆点博士:P参数
	for(i=0;i<3;i++) pid_setting_I_value[i]=(unsigned int)bs004_pid_par[i+3]; //圆点博士:I参数
	for(i=0;i<3;i++) pid_setting_D_value[i]=(unsigned int)bs004_pid_par[i+6]; //圆点博士:D参数
	for(i=0;i<3;i++) pid_setting_M_value[i]=0;                                //圆点博士:机械中心点调整参数
	bs004_fly_gas_scale=(float)bs004_pid_par[9]/1.0f;													//飞行油门参数
	//欧拉角参数，分别为俯仰系数，横滚系数，航向系数
	bs004_fly_pitch_scale=0.0f;
	bs004_fly_roll_scale=0.0f;
	bs004_fly_yaw_scale=0.0f;
}
//加载校准系数
void BS004_Load_Calibration_Parameter(void)
{
	int bs004_cal_par[12];
	unsigned char i;
	unsigned int *bs004_parameter_addr;
	unsigned int bs004_parameter_data,bs004_parameter_sign,bs004_parameter_valid;
	unsigned char bs004_parameter_byte_data[32];
	//	
	if(bs004_load_par_sign==0)			//圆点博士:从flash装载参数   
	{
		for(i=0;i<7;i++) 											
		{
			bs004_parameter_addr=(unsigned int *)(0x803F438+i*4);				//圆点博士:更新地址
			bs004_parameter_data=(unsigned int)*(bs004_parameter_addr);	//圆点博士:从地址读取数据
			//读取bs004_parameter_byte_data的第0至第7位，下同理
			bs004_parameter_byte_data[i*4]=bs004_parameter_data & 0xff;
			bs004_parameter_byte_data[i*4+1]=(bs004_parameter_data>>8) & 0xff;
			bs004_parameter_byte_data[i*4+2]=(bs004_parameter_data>>16) & 0xff;
			bs004_parameter_byte_data[i*4+3]=(bs004_parameter_data>>24) & 0xff;			
		}
		//bs004_parameter_sign取bs004_parameter_data的0-15位
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
	else	//圆点博士:使用代码默认参数
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("start to do self calibration.");      //圆点博士:自我矫正，程序运行到该位置，飞行器应该保持水平。
		BS004_COM1_Send_Str_Tail();
		BS004_Long_Delay(300000);	
		//
		BS004_Cal_MPU6050_Data(bs004_cal_par);
	}
	//
	bs004_mpu6050_gyro_pitch_cal=(signed short int)bs004_cal_par[0];	//圆点博士:陀螺仪校验参数
	bs004_mpu6050_gyro_roll_cal=(signed short int)bs004_cal_par[1];   //圆点博士:陀螺仪校验参数
	bs004_mpu6050_gyro_yaw_cal=(signed short int)bs004_cal_par[2];    //圆点博士:陀螺仪校验参数
	bs004_mpu6050_acc_roll_cal=(signed short int)bs004_cal_par[3];		//圆点博士:加速度校验参数
	bs004_mpu6050_acc_pitch_cal=(signed short int)bs004_cal_par[4];   //圆点博士:加速度校验参数
	bs004_mpu6050_acc_yaw_cal=(signed short int)bs004_cal_par[5];     //圆点博士:加速度校验参数
	bs004_hmc5883l_mag_pitch_cal=(signed short int)bs004_cal_par[6];  //圆点博士:磁力计校验参数
	bs004_hmc5883l_mag_roll_cal=(signed short int)bs004_cal_par[7];   //圆点博士:磁力计校验参数
	bs004_hmc5883l_mag_yaw_cal=(signed short int)bs004_cal_par[8];    //圆点博士:磁力计校验参数
	//调用串口通信函数，发送字符 D-
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("gyro pitch cal= .");
	BS004_COM1_Send_4bits_BCD_Num(bs004_mpu6050_gyro_pitch_cal);
	//调用串口通信函数，发送字符 /13
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
	BS004_COM1_Send_Str_Body("finish the self calibration.");      //圆点博士:结束自我矫正
	BS004_COM1_Send_Str_Tail();
	BS004_Long_Delay(300000);	
}

//加载陀螺仪参数
void BS004_Cal_MPU6050_Data(int *bs004_cal_data)   
{
	unsigned char i,j;
	unsigned char bs004_mpu6050_cal_data_buffer[14];
	signed short int bs004_mpu6050_cal_data[100][7];
	int bs004_mpu6050_cal_sum[7];
	//循环语句，记录校准传感器数据次数
	for(i=0;i<7;i++) bs004_mpu6050_cal_sum[i]=0;
	//
	for(j=0;j<100;j++)
	{
		while(BS004_MPU6050_INT_STATE);
		//获得中断提醒
		BS004_I2C_START();																				//启动传输函数
		BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);							//圆点博士:发送陀螺仪写地址
		BS004_I2C_SendByte(BS004_MPU6050_ACCEL_DATA_ADDR);    		//圆点博士:发送陀螺仪寄存器地址
		BS004_I2C_START();
		BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR+1);      			//圆点博士:发送陀螺仪读地址
		BS004_I2C_Receive14Bytes(bs004_mpu6050_cal_data_buffer);	//圆点博士:读出陀螺仪寄存器值
		BS004_I2C_STOP();																					//停止传输函数
		//循环语句，絙s004_mpu6050_cal_data_buffer按位或bs004_mpu6050_cal_data_buffer 然后传输给bs004_mpu6050_cal_data		
		for(i=0;i<7;i++) bs004_mpu6050_cal_data[j][i]=(((signed short int)bs004_mpu6050_cal_data_buffer[i*2]) << 8) | bs004_mpu6050_cal_data_buffer[i*2+1];
	}
	//
	for(i=0;i<7;i++)
	{
		for(j=0;j<100;j++) bs004_mpu6050_cal_sum[i]=bs004_mpu6050_cal_sum[i]+(int)bs004_mpu6050_cal_data[j][i];
	}
	//将数据恢复正常值传输给bs004_cal_data
	bs004_cal_data[0]=bs004_mpu6050_cal_sum[4]/100;
	bs004_cal_data[1]=bs004_mpu6050_cal_sum[5]/100;
	bs004_cal_data[2]=bs004_mpu6050_cal_sum[6]/100;
	bs004_cal_data[3]=bs004_mpu6050_cal_sum[0]/100;
	bs004_cal_data[4]=bs004_mpu6050_cal_sum[1]/100;
	bs004_cal_data[5]=bs004_mpu6050_cal_sum[2]/100;;
	//用不到，因此置0
	bs004_cal_data[6]=0;
	bs004_cal_data[7]=0;
	bs004_cal_data[8]=0;
}

//传输校准参数
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
	BS004_I2C_START();																					//启动传输函数
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);								//圆点博士:发送陀螺仪写地址
	BS004_I2C_SendByte(BS004_MPU6050_ACCEL_DATA_ADDR);    			//圆点博士:发送陀螺仪寄存器地址
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR+1);      				//圆点博士:发送陀螺仪读地址
	BS004_I2C_Receive14Bytes(bs004_mpu6050_caled_data_buffer);	//圆点博士:读出陀螺仪寄存器值
	BS004_I2C_STOP();																						//停止传输函数
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

