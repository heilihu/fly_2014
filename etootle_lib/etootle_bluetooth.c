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
#include "etootle_bluetooth.h" 
#include "etootle_parameter.h" 
#include "etootle_motor.h" 
#include "etootle_adc.h" 
#include "etootle_led.h" 
#include "etootle_sys.h" 
//数据包宏定义
#define AnBT_Command_Head     58		//圆点博士数据头标志 :
#define AnBT_Command_Tail	    47    //圆点博士数据头标志 
#define AnBT_COM_Buf_Length		64		//圆点博士接收缓冲长度
//PID参数相关变量
unsigned char  pid_data_buffer_valid=0;			//圆点博士:PID数据合法标志
unsigned int   pid_data_buffer=0;         	//圆点博士:PID数据缓冲
unsigned int   pid_setting_P_value[3]={0};	//圆点博士:PID的P参数，含X轴,Y轴,Z轴
unsigned int   pid_setting_I_value[3]={0};	//圆点博士:PID的I参数，含X轴,Y轴,Z轴	
unsigned int   pid_setting_D_value[3]={0};	//圆点博士:PID的D参数，含X轴,Y轴,Z轴	
unsigned int   pid_setting_M_value[3]={0};  //圆点博士:PID的M参数，含X轴,Y轴,Z轴	
//遥控数据相关变量
unsigned char  ctrl_data_buffer_valid=0;          //圆点博士:控制数据合法标志
unsigned char  remote_ctrl_data_buffer[4]={0};		//圆点博士:遥控数据缓冲
unsigned int   bs004_bat_value=0;									//圆点博士:电池电压值
//串口参数变量定义
unsigned char com_status_is_idle=1;												//圆点博士:串口通讯空闲标志
unsigned char valid_command_was_received=0;								//圆点博士:合法命令接收标志
unsigned char bs004_com_command_ops=0;											//圆点博士:接收到的命令
unsigned char com_receive_str_index=0;										//圆点博士:接收缓冲地址索引
unsigned char com_receive_str_buf[AnBT_COM_Buf_Length]; 	//圆点博士:接收字符缓冲
unsigned char com_receive_data_buf[AnBT_COM_Buf_Length]; 	//圆点博士:接收数据缓冲
//遥控参数变量定义
//依次为油门控制参数 控制信号有效标志  油门控制阈值
unsigned char BS004_Ctrl_Gas=0,BS004_Ctrl_Valid=0,BS004_Ctrl_Gas_Noise=50;
//依次为俯仰控制参数，横滚控制参数 偏航控制参数 姿态控制阈值
signed char   BS004_Ctrl_Pitch=0,BS004_Ctrl_Roll=0,BS004_Ctrl_Yaw=0,BS004_Ctrl_Dir_Noise=20;
//实际飞行的油门俯仰 横滚 偏航参数，本文件内没有用到
signed short  bs004_fly_gas=0,bs004_fly_pitch=0,bs004_fly_roll=0,bs004_fly_yaw=0;
//依次为允许姿态输出到上位机的标志 电机锁定标志
unsigned char BS004_IMU_Output=1,BS004_Motor_Lock=1;

//串口接收中断处理函数 在接收到数据后会自动调用
void BS004_COM1_Interrupt(void)	//圆点博士:蓝牙接收中断
{
	unsigned char com_receive_data=0;						//接收数据缓存变量
	unsigned char com_receive_data_checksum=0;	//接收数据校验和变量
	unsigned char com_receive_data_checksum_low,com_receive_data_checksum_high;		//接收数据校验和的低位  高位变量
	unsigned char com_data_checksum=0;					//数据校验和变量
	//存储接收到的表示控制功能符的四个变量
	unsigned char anbt_com_command_HH=0,anbt_com_command_HL=0,anbt_com_command_LH=0,anbt_com_command_LL=0;
	//定义循环变量
	unsigned char i;
	//读取接收到的1字节的数据
	com_receive_data=USART_ReceiveData(USART1);   
	
	//如果收到的数据等于数据头标志
	if(com_receive_data==AnBT_Command_Head)   
	{
			com_receive_str_index=0;     		//接收缓存定制索引清零
			valid_command_was_received=0;		//已接受有效命令标志置零
			com_data_checksum=0;						//数据和校验清零
	}
	//如果收到的数据为数据尾标志
	else if(com_receive_data==AnBT_Command_Tail)  
	{
		//如果接收缓存地址索引长度大于1
			if(com_receive_str_index>1) 
			{			
				 //圆点博士:计算数据校验和标志
				for(i=0;i<com_receive_str_index-2;i++) com_data_checksum += com_receive_str_buf[i]; 
				//读取接收到的校验和的低位
				com_receive_data_checksum_low=com_receive_str_buf[com_receive_str_index-1];
				//读取接收到的校验和的高位
				com_receive_data_checksum_high=com_receive_str_buf[com_receive_str_index-2];				
				//将两个字节的字符串转换成一个字节的数据
				if(com_receive_data_checksum_low>58) com_receive_data_checksum_low-=55;
				else com_receive_data_checksum_low-=48;
				//将两个字节的字符串转换成一个字节的数据				
				if(com_receive_data_checksum_high>58) com_receive_data_checksum_high-=55;
				else com_receive_data_checksum_high-=48;	
				//将两个1位16进制数字分别作为高4位和低四位合成一个字节存入收到数据校验和变量中，结合前面两个if语句实现将字符串"5A"转换成0x5A
				com_receive_data_checksum=((com_receive_data_checksum_high<<4)&0xf0)|(com_receive_data_checksum_low&0x0f);
				//计算接收到的校验和
				com_data_checksum=com_data_checksum+com_receive_data_checksum;
				//如果检验和等于0，则认为已经接收到有效命令
				if(com_data_checksum==0) valid_command_was_received=1;
			}
			//串口空闲同时接收到有效命令
			if(com_status_is_idle && valid_command_was_received) //data is correct and allowed to receive
			{				
				//将字符缓冲区的数据转存到四个控制功能符变量中
				anbt_com_command_HH=com_receive_str_buf[0];
				anbt_com_command_HL=com_receive_str_buf[1];
				anbt_com_command_LH=com_receive_str_buf[2];
				anbt_com_command_LL=com_receive_str_buf[3];
				//清除PID数据合法标志
				pid_data_buffer_valid=0;
				//清除控制数据合法标志
				ctrl_data_buffer_valid=0;
				//根据功能串设置接收到的命令
				//RC 写入远程遥控
				if((anbt_com_command_HH=='R')&&(anbt_com_command_HL=='C'))	bs004_com_command_ops=0xA0;					//write remote ctrol
				//BAT 读取电池电压
				else if((anbt_com_command_HH=='B')&&(anbt_com_command_HL=='A')&&(anbt_com_command_LH=='T'))  	bs004_com_command_ops=0xA1;		//read voltage
				//VER  读取版本号
				else if((anbt_com_command_HH=='V')&&(anbt_com_command_HL=='E')&&(anbt_com_command_LH=='R'))		bs004_com_command_ops=0xA2;		//return version
				//CTRL 读取远程遥控
				else if((anbt_com_command_HH=='C')&&(anbt_com_command_HL=='T')&&(anbt_com_command_LH=='R')&&(anbt_com_command_LL=='L'))	bs004_com_command_ops=0xA3;	  //read remote ctrol
				//PWON 电机PWM输出开
				else if((anbt_com_command_HH=='P')&&(anbt_com_command_HL=='W')&&(anbt_com_command_LH=='O')&&(anbt_com_command_LL=='N'))	bs004_com_command_ops=0xA4;		//motor pwm on
				//PWOF 电机PWM输出关
				else if((anbt_com_command_HH=='P')&&(anbt_com_command_HL=='W')&&(anbt_com_command_LH=='O')&&(anbt_com_command_LL=='F'))	bs004_com_command_ops=0xA5;		//motor pwm off
				//PRST 重置PID参数
				else if((anbt_com_command_HH=='P')&&(anbt_com_command_HL=='R')&&(anbt_com_command_LH=='S')&&(anbt_com_command_LL=='T'))	bs004_com_command_ops=0xA6;		//PID paramater mode	
				//ZTON  输出姿态
				else if((anbt_com_command_HH=='Z')&&(anbt_com_command_HL=='T')&&(anbt_com_command_LH=='O')&&(anbt_com_command_LL=='N'))	bs004_com_command_ops=0xA7;		//PID paramater mode	
				//ZTOF 停止输出姿态
				else if((anbt_com_command_HH=='Z')&&(anbt_com_command_HL=='T')&&(anbt_com_command_LH=='O')&&(anbt_com_command_LL=='F'))	bs004_com_command_ops=0xA8;		//PID paramater mode	
				//RST 系统复位
				else if((anbt_com_command_HH=='R')&&(anbt_com_command_HL=='S')&&(anbt_com_command_LH=='T')) bs004_com_command_ops=0xA9;
				//PALL 读取P数据	
				else if((anbt_com_command_HH=='P')&&(anbt_com_command_HL=='A')&&(anbt_com_command_LH=='L')&&(anbt_com_command_LL=='L'))	bs004_com_command_ops=0xB0;		//read P data
				//IALL 读取I数据
				else if((anbt_com_command_HH=='I')&&(anbt_com_command_HL=='A')&&(anbt_com_command_LH=='L')&&(anbt_com_command_LL=='L'))	bs004_com_command_ops=0xB1;		//read I data
				//DALL 读取D数据
				else if((anbt_com_command_HH=='D')&&(anbt_com_command_HL=='A')&&(anbt_com_command_LH=='L')&&(anbt_com_command_LL=='L'))	bs004_com_command_ops=0xB2;		//read D data
				//MALL 读取M数据
				else if((anbt_com_command_HH=='M')&&(anbt_com_command_HL=='A')&&(anbt_com_command_LH=='L')&&(anbt_com_command_LL=='L'))	bs004_com_command_ops=0xB3;		//read M data
				//WRPX 写入PX数据
				else if((anbt_com_command_HH=='W')&&(anbt_com_command_HL=='R')&&(anbt_com_command_LH=='P')&&(anbt_com_command_LL=='X'))	bs004_com_command_ops=0xB4;		//write PX
				//WRPY 写入PY数据
				else if((anbt_com_command_HH=='W')&&(anbt_com_command_HL=='R')&&(anbt_com_command_LH=='P')&&(anbt_com_command_LL=='Y'))	bs004_com_command_ops=0xB5;		//write PY
				//WRPZ 写入PZ数据
				else if((anbt_com_command_HH=='W')&&(anbt_com_command_HL=='R')&&(anbt_com_command_LH=='P')&&(anbt_com_command_LL=='Z'))	bs004_com_command_ops=0xB6;		//write PZ
				//WRIX 写入IX数据
				else if((anbt_com_command_HH=='W')&&(anbt_com_command_HL=='R')&&(anbt_com_command_LH=='I')&&(anbt_com_command_LL=='X'))	bs004_com_command_ops=0xB7;		//write IX
				//WRIY 写入IY数据
				else if((anbt_com_command_HH=='W')&&(anbt_com_command_HL=='R')&&(anbt_com_command_LH=='I')&&(anbt_com_command_LL=='Y'))	bs004_com_command_ops=0xB8;		//write IY
				//WRIZ  写入IZ数据
				else if((anbt_com_command_HH=='W')&&(anbt_com_command_HL=='R')&&(anbt_com_command_LH=='I')&&(anbt_com_command_LL=='Z'))	bs004_com_command_ops=0xB9;		//write IZ				
				//WRDX 写入DX数据
				else if((anbt_com_command_HH=='W')&&(anbt_com_command_HL=='R')&&(anbt_com_command_LH=='D')&&(anbt_com_command_LL=='X'))	bs004_com_command_ops=0xBA;		//write DX
				//WRDY  写入DY数据
				else if((anbt_com_command_HH=='W')&&(anbt_com_command_HL=='R')&&(anbt_com_command_LH=='D')&&(anbt_com_command_LL=='Y'))	bs004_com_command_ops=0xBB;		//write DY
				//WRDZ  写入DZ数据
				else if((anbt_com_command_HH=='W')&&(anbt_com_command_HL=='R')&&(anbt_com_command_LH=='D')&&(anbt_com_command_LL=='Z'))	bs004_com_command_ops=0xBC;		//write DZ
				//WRMX  写入MX数据
				else if((anbt_com_command_HH=='W')&&(anbt_com_command_HL=='R')&&(anbt_com_command_LH=='M')&&(anbt_com_command_LL=='X'))	bs004_com_command_ops=0xBD;		//write MX
				//WRMY  写入MY数据
				else if((anbt_com_command_HH=='W')&&(anbt_com_command_HL=='R')&&(anbt_com_command_LH=='M')&&(anbt_com_command_LL=='Y'))	bs004_com_command_ops=0xBE;		//write MY
				//WRMZ  写入MZ数据
				else if((anbt_com_command_HH=='W')&&(anbt_com_command_HL=='R')&&(anbt_com_command_LH=='M')&&(anbt_com_command_LL=='Z'))	bs004_com_command_ops=0xBF;		//write MZ
				//其他情况 等于0 则表示误操作
				else bs004_com_command_ops=0;
				
				//一下判断条件大前提是  串口空闲并且接收到有效命令
				//如果接收超过四个数据
				if(com_receive_str_index>4)	
				{
					//如果收到数据长度为10字节
					if(com_receive_str_index==10)
					{
						//若四个字节数据为"xxxx4321xx"(字符串)，则接收缓存区得到数据为{4 3 2 1}(数组，十进制) PID数据缓冲为4321{十进制}
						for(i=0;i<4;i++) 
						{
							//将ASCII字符形式的BCD码以数值方式按位倒序存入到接收数据缓存区
							com_receive_data_buf[i]=com_receive_str_buf[i+4]-48;
							//转换后单字符数值超过9则等于0
							if(com_receive_data_buf[i]>9) com_receive_data_buf[i]=0;
						}
						//将计算得到的数据送入PID数据缓存
						pid_data_buffer=com_receive_data_buf[0]*1000+com_receive_data_buf[1]*100+com_receive_data_buf[2]*10+com_receive_data_buf[3];	
						//PID数据有效
						pid_data_buffer_valid=1;
					}
					//如果收到数据长度为12字节
					else if(com_receive_str_index==12)
					{
						for(i=0;i<8;i++) 
						{			
							com_receive_data_buf[i]=com_receive_str_buf[i+2];
							//将两个字节的字符串转换成一个字节的数据
							if(com_receive_data_buf[i]>58) com_receive_data_buf[i]-=55;
							else com_receive_data_buf[i]-=48;
						}
						//将两个1为16进制数字分别作为高4位和低4位合成一个字节，并存入控制数据缓冲区
						for(i=0;i<4;i++) remote_ctrl_data_buffer[i]=com_receive_data_buf[i*2]*16+com_receive_data_buf[i*2+1];
						//控制数据有效标志置一
						ctrl_data_buffer_valid=1;
					}
				}
				//		如果指令是 写入远程遥控
				if(bs004_com_command_ops==0xA0)
				{
					//先忽略符号位赋值，符号到后面处理 下同
					BS004_Ctrl_Gas=remote_ctrl_data_buffer[0];
					BS004_Ctrl_Pitch=remote_ctrl_data_buffer[1]&0x7f;
					BS004_Ctrl_Roll=remote_ctrl_data_buffer[2]&0x7f;
					BS004_Ctrl_Yaw=remote_ctrl_data_buffer[3]&0x7f;
					//以上四行分别对油门控制参数 俯仰控制参数 横滚控制参数 偏航控制参数 进行数值提取和赋值，不过都是无符号的绝对值 下面对符号进行处理
					//如果控制数据缓存区最高位是1 即数据为负，则取相反数并赋值给哥哥控制变量，下同
					if(remote_ctrl_data_buffer[1]&0x80) BS004_Ctrl_Pitch=-BS004_Ctrl_Pitch;
					if(remote_ctrl_data_buffer[2]&0x80) BS004_Ctrl_Roll=-BS004_Ctrl_Roll;
					if((remote_ctrl_data_buffer[3]&0x80)==0x80) BS004_Ctrl_Yaw=-BS004_Ctrl_Yaw;					
					//控制有效标志
					BS004_Ctrl_Valid=1;
				}
		 }
	}
	//如果接收到的数据既不是数据头也不是数据尾
	else
	{
		//将数据装入缓冲区
			com_receive_str_buf[com_receive_str_index] = com_receive_data;
			if(com_receive_str_index<AnBT_COM_Buf_Length-1) com_receive_str_index++;			//圆点博士:接收缓冲地址加1
			else com_receive_str_index=0;					//圆点博士:清0接受缓冲地址,防止数组溢出
	}
}

//---------------------------------------------------------------------------------------------------
//发送一个字符函数
void BS004_COM1_Send_Char(unsigned char ascii_code) 						//圆点博士:发送一个字符
{
	//长延时防止串口忙不过来
	BS004_Long_Delay(10000);
	//发送单个字符
	USART_SendData(USART1,ascii_code);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}	//圆点博士:等待直到发送完成
}

//以字符方式发送一字节16进制数据
void BS004_COM1_Send_Num(unsigned char number) 	//圆点博士:发送一个数字
{
	unsigned char num_low,num_high;
	num_low=number&0x0f;													//圆点博士:取数据低位
	num_high=(number&0xf0)>>4;										//圆点博士:取数据高位
	//数字转成字符，将16进制的A-F转成字符
	if(num_high<10)USART_SendData(USART1,num_high+48);
	else USART_SendData(USART1,num_high+55);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}								//圆点博士:等待直到发送完成
	//0-F转换成"0"-"F"
	if(num_low<10)USART_SendData(USART1,num_low+48);
	else USART_SendData(USART1,num_low+55);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}								//圆点博士:等待直到发送完成
}

//发送数据头函数
void BS004_COM1_Send_Str_Head(void)
{
	BS004_COM1_Send_Char(':');  		//圆点博士:发送字符:
	BS004_COM1_Send_Char('D');  		//圆点博士:发送字符D
	BS004_COM1_Send_Char('-');  		//圆点博士:发送字符-
}
//发送一个以"."结尾的字符串("."用于计算字符串长度 不能被发送)
void BS004_COM1_Send_Str_Body(unsigned char* str_buf)		//圆点博士:发送一个指定长度的字符串
{
	unsigned char i,str_len;
	//循环变量和字符串长度变量
	str_len=0;
	//每个字符串都小于64字节，不包含"."并以"."结束，所以通过上面这个for查找"."可以确定字符串的长度 然后租个进行发送
	for(i=0;i<64;i++)
	{	
		str_len++;
		if(str_buf[i]=='.') break;
	}
  for(i=0;i<str_len;i++) BS004_COM1_Send_Char(str_buf[i]); 	//圆点博士:发送字符:
}
//发送数据尾函数
void BS004_COM1_Send_Str_Tail(void)
{
	BS004_COM1_Send_Char('/');																//圆点博士:发送字符/
	BS004_COM1_Send_Char(13);																	//圆点博士:发送回车字符
}
//串口通信协议
void BS004_COM1_Send_4bits_BCD_Num(int number) 	//圆点博士:发送一个字符
{
	unsigned int num;
	unsigned char num_th[4]={0};
	unsigned char i,bcd_code_len;
	//
	num=fabs(number);
	if(number<0) BS004_COM1_Send_Char('-');  		//圆点博士:发送字符-
	if(num>9999) num=9999;
	if(num>999) bcd_code_len=4;
	else if(num>99) bcd_code_len=3;
	else if(num>9) bcd_code_len=2;		
	else bcd_code_len=1;
	//
	if(num>999) 	{num_th[3]=num/1000;	num-=num_th[3]*1000;}
	if(num>99) 	 	{num_th[2]=num/100;		num-=num_th[2]*100; }
	if(num>9) 		{num_th[1]=num/10;		num-=num_th[1]*10;	}	
	num_th[0]=num;
	//
	for(i=0;i<bcd_code_len;i++) BS004_COM1_Send_Char(num_th[bcd_code_len-1-i]+48);
}

//---------------------------------------------------------------------------------------------------
// 串口通信回应函数 收到正确的命令后调用此函数进行回应
void BS004_COM1_Communication(void)
{
	unsigned char i;
	//
	if(bs004_com_command_ops==0xA1) 			//圆点博士:返回电池电压,注意,如果电机电源没开,将返回0
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("read bat voltage .");
		BS004_ADC_Get_ADC_Value();
		//发送电池电压
		BS004_COM1_Send_4bits_BCD_Num(bs004_bat_value);	
		BS004_COM1_Send_Str_Tail();
	}
	//
	else if(bs004_com_command_ops==0xA2) //圆点博士:返回版本号
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("version 03.");
		BS004_COM1_Send_Str_Tail();
	}
	//
	else if(bs004_com_command_ops==0xA3) //圆点博士:返回遥控器数据
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("read remote control data .");
		//	发送遥控油门
		BS004_COM1_Send_4bits_BCD_Num(BS004_Ctrl_Gas);			
		BS004_COM1_Send_Char(',');	
		//发送遥控俯仰		
		BS004_COM1_Send_4bits_BCD_Num(BS004_Ctrl_Pitch);			
		BS004_COM1_Send_Char(',');			
		//发送遥控横滚
		BS004_COM1_Send_4bits_BCD_Num(BS004_Ctrl_Roll);			
		BS004_COM1_Send_Char(',');	
		//发送遥控偏航
		BS004_COM1_Send_4bits_BCD_Num(BS004_Ctrl_Yaw);		
		//		
		BS004_COM1_Send_Str_Tail();
	}
	else if(bs004_com_command_ops==0xA4) //圆点博士:打开电机电源
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("motor power on.");	
		BS004_COM1_Send_Str_Tail();		
		//电机解锁
		BS004_Motor_Lock=0;
		BS004_Ctrl_Gas=0;					//圆点博士:复位遥控数据
		BS004_Ctrl_Pitch=0;				//圆点博士:复位遥控数据
		BS004_Ctrl_Roll=0;				//圆点博士:复位遥控数据
		BS004_Ctrl_Yaw=0;					//圆点博士:复位遥控数据
		BS004_MOTOR_LED_ON();			//圆点博士:点亮LED
		BS004_Motor_Power_On();   //圆点博士:打开电机电源
	}
	//
	else if(bs004_com_command_ops==0xA5) //圆点博士:关闭电机电源
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("motor power off.");	
		BS004_COM1_Send_Str_Tail();		
		//电机上锁
		BS004_Motor_Lock=1;
		BS004_Ctrl_Gas=0;					//圆点博士:复位遥控数据
		BS004_Ctrl_Pitch=0;				//圆点博士:复位遥控数据
		BS004_Ctrl_Roll=0;				//圆点博士:复位遥控数据
		BS004_Ctrl_Yaw=0;					//圆点博士:复位遥控数据
		BS004_MOTOR_LED_OFF();		//圆点博士:关闭LED
	  BS004_Motor_Power_Off();  //圆点博士:关闭电机电源
	}
	else if(bs004_com_command_ops==0xA6) //圆点博士:复位PID数据和校验数据
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("reset PID cal data .");	
		BS004_COM1_Send_Str_Tail();		
		//复位加速度和角速度计算值
		bs004_mpu6050_acc_roll_cal=0;
		bs004_mpu6050_acc_pitch_cal=0;		
		bs004_mpu6050_acc_yaw_cal=0;
		bs004_mpu6050_gyro_pitch_cal=0;
		bs004_mpu6050_gyro_roll_cal=0;
		bs004_mpu6050_gyro_yaw_cal=0;
		//复位各州PIDM值
		for(i=0;i<3;i++) pid_setting_P_value[i]=0;
		for(i=0;i<3;i++) pid_setting_I_value[i]=0;
		for(i=0;i<3;i++) pid_setting_D_value[i]=0;
		for(i=0;i<3;i++) pid_setting_M_value[i]=0;
	}	
	else if(bs004_com_command_ops==0xA7) //圆点博士:允许输出姿态到上位机
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("begin to output angle .");	
		BS004_COM1_Send_Str_Tail();	
		//惯性测量单元输出标志位打开
		BS004_IMU_Output=1;
	}
	else if(bs004_com_command_ops==0xA8) //圆点博士:停止输出姿态到上位机
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("stop to output angle .");	
		BS004_COM1_Send_Str_Tail();	
		//惯性测量单元输出标志位关闭
		BS004_IMU_Output=0;
	}
	//
	else if(bs004_com_command_ops==0xA9) //圆点博士:复位飞机
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("reset the system .");	
		BS004_COM1_Send_Str_Tail();		
		//关闭系统总中断
		NVIC_SETFAULTMASK();
		//复位中断
		NVIC_GenerateSystemReset();
	}	
	//
	else if(bs004_com_command_ops==0xB0) 	//圆点博士:返回P值
	{		
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("read All P data .");
		for(i=0;i<3;i++)
		{
			BS004_COM1_Send_4bits_BCD_Num(pid_setting_P_value[i]);			
			BS004_COM1_Send_Char(',');		
		}
		//
		BS004_COM1_Send_Str_Tail();
	}
	//
	else if(bs004_com_command_ops==0xB1) 	//圆点博士:返回I值
	{		
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("read All I data .");
		//
		for(i=0;i<3;i++)
		{
			BS004_COM1_Send_4bits_BCD_Num(pid_setting_I_value[i]);			
			BS004_COM1_Send_Char(',');		
		}
		//
		BS004_COM1_Send_Str_Tail();
	}
	//
	else if(bs004_com_command_ops==0xB2) 	//圆点博士:返回D值
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("read All D data .");
		//
		for(i=0;i<3;i++)
		{
			BS004_COM1_Send_4bits_BCD_Num(pid_setting_D_value[i]);			
			BS004_COM1_Send_Char(',');		
		}
		//
		BS004_COM1_Send_Str_Tail();
	}
	//
	else if(bs004_com_command_ops==0xB3) 	//圆点博士:返回M值
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("read All M data .");
		//
		for(i=0;i<3;i++)
		{
			BS004_COM1_Send_4bits_BCD_Num(pid_setting_M_value[i]);			
			BS004_COM1_Send_Char(',');		
		}
		//
		BS004_COM1_Send_Str_Tail();
	}
	//
	else if(bs004_com_command_ops==0xB4)  //圆点博士:设置X轴P值
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("write PX data .");
		//
		if(pid_data_buffer_valid==1)
		{
			pid_setting_P_value[0]=pid_data_buffer;
			pid_data_buffer_valid=0;
		}
		//
		BS004_COM1_Send_4bits_BCD_Num(pid_data_buffer);	
		BS004_COM1_Send_Str_Tail();		
	}
	//
	else if(bs004_com_command_ops==0xB5) //圆点博士:设置Y轴P值
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("write PY data .");		
		//
		if(pid_data_buffer_valid==1)
		{
			pid_setting_P_value[1]=pid_data_buffer;
			pid_data_buffer_valid=0;
		}
		//
		BS004_COM1_Send_4bits_BCD_Num(pid_data_buffer);	
		BS004_COM1_Send_Str_Tail();		
	}
	//
	else if(bs004_com_command_ops==0xB6) //圆点博士:设置Z轴P值
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("write PZ data .");				
		//
		if(pid_data_buffer_valid==1)
		{
			pid_setting_P_value[2]=pid_data_buffer;
			pid_data_buffer_valid=0;
		}
		//
		BS004_COM1_Send_4bits_BCD_Num(pid_data_buffer);	
		BS004_COM1_Send_Str_Tail();		
	}
	//
	else if(bs004_com_command_ops==0xB7)  //圆点博士:设置X轴I值
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("write IX data .");
		//
		if(pid_data_buffer_valid==1)
		{
			pid_setting_I_value[0]=pid_data_buffer;
			pid_data_buffer_valid=0;
		}
		//
		BS004_COM1_Send_4bits_BCD_Num(pid_data_buffer);	
		BS004_COM1_Send_Str_Tail();		
	}
	//
	else if(bs004_com_command_ops==0xB8) //圆点博士:设置Y轴I值
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("write IY data .");		
		//
		if(pid_data_buffer_valid==1)
		{
			pid_setting_I_value[1]=pid_data_buffer;
			pid_data_buffer_valid=0;
		}
		//
		BS004_COM1_Send_4bits_BCD_Num(pid_data_buffer);	
		BS004_COM1_Send_Str_Tail();		
	}
	//
	else if(bs004_com_command_ops==0xB9) //圆点博士:设置Z轴I值
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("write IZ data .");				
		//
		if(pid_data_buffer_valid==1)
		{
			pid_setting_I_value[2]=pid_data_buffer;
			pid_data_buffer_valid=0;
		}
		//
		BS004_COM1_Send_4bits_BCD_Num(pid_data_buffer);	
		BS004_COM1_Send_Str_Tail();		
	}
	//	
	else if(bs004_com_command_ops==0xBA) //圆点博士:设置X轴D值
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("write DX data .");	
		//
		if(pid_data_buffer_valid==1)
		{
			pid_setting_D_value[0]=pid_data_buffer;
			pid_data_buffer_valid=0;
		}
		//
		BS004_COM1_Send_4bits_BCD_Num(pid_data_buffer);	
		BS004_COM1_Send_Str_Tail();			
	}
	//
	else if(bs004_com_command_ops==0xBB) //圆点博士:设置Y轴D值
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("write DY data .");	
		//
		if(pid_data_buffer_valid==1)
		{
			pid_setting_D_value[1]=pid_data_buffer;;
			pid_data_buffer_valid=0;
		}
		//
		BS004_COM1_Send_4bits_BCD_Num(pid_data_buffer);	
		BS004_COM1_Send_Str_Tail();		
	}
	//
	else if(bs004_com_command_ops==0xBC) //圆点博士:设置Z轴D值
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("write DZ data .");	
		//
		if(pid_data_buffer_valid==1)
		{
			pid_setting_D_value[2]=pid_data_buffer;
			pid_data_buffer_valid=0;
		}
		//
		BS004_COM1_Send_4bits_BCD_Num(pid_data_buffer);	
		BS004_COM1_Send_Str_Tail();		
	}
	//
	else if(bs004_com_command_ops==0xBD) //圆点博士:设置X轴M值
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("write MX data .");	
		//
		if(pid_data_buffer_valid==1)
		{
			pid_setting_M_value[0]=pid_data_buffer;
			pid_data_buffer_valid=0;
		}
		//
		BS004_COM1_Send_4bits_BCD_Num(pid_data_buffer);	
		BS004_COM1_Send_Str_Tail();			
	}
	//
	else if(bs004_com_command_ops==0xBE) //圆点博士:设置Y轴M值
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("write MY data .");	
		//
		if(pid_data_buffer_valid==1)
		{
			pid_setting_M_value[1]=pid_data_buffer;;
			pid_data_buffer_valid=0;
		}
		//
		BS004_COM1_Send_4bits_BCD_Num(pid_data_buffer);	
		BS004_COM1_Send_Str_Tail();		
	}
	//
	else if(bs004_com_command_ops==0xBF) //圆点博士:设置Z轴M值
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("write MZ data .");	
		//
		if(pid_data_buffer_valid==1)
		{
			pid_setting_M_value[2]=pid_data_buffer;
			pid_data_buffer_valid=0;
		}
		//
		BS004_COM1_Send_4bits_BCD_Num(pid_data_buffer);	
		BS004_COM1_Send_Str_Tail();		
	}
	else
	{
		BS004_COM1_Send_Str_Head();
		BS004_COM1_Send_Str_Body("unknown command.");	//圆点博士:未知命令
		BS004_COM1_Send_Str_Tail();		
	}
}

//---------------------------------------------------------------------------------------------------
//串口端口初始化函数
void BS004_COM1_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	//开启GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//开启串口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
	//
  GPIO_InitStructure.GPIO_Pin = BS004_COM1_TX;					//圆点博士:设置PA9管脚为串口TX
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 		//圆点博士:设置串口TX最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   		//圆点博士:设置串口TX为输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = BS004_COM1_RX;					//圆点博士:设置PA9管脚为串口RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //圆点博士:设置串口RX为输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
}
//串口初始化
void BS004_COM1_Port_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
	//
  USART_InitStructure.USART_BaudRate = 115200;									//圆点博士:设置串口波特率为115200
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;   //圆点博士:设置串口数据长度为8位
  USART_InitStructure.USART_StopBits = USART_StopBits_1;        //圆点博士:设置串口停止位长度为1位
  USART_InitStructure.USART_Parity = USART_Parity_No ;					//圆点博士:设置串口奇偶校验为无
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //圆点博士:设置串口数据流控制为无
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;										//圆点博士:设置串口为发送和接收模式
  USART_Init(USART1, &USART_InitStructure);			//圆点博士:设置串口参数
	//
	BS004_COM1_NVIC_Configuration();							//圆点博士:设置中断优先级
  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);	//圆点博士:允许接收中断
  USART_Cmd(USART1, ENABLE);  									//圆点博士:使能串口
	//发送数据头
	BS004_COM1_Send_Str_Head();
	//发送消息告知 蓝牙串口设备已经初始化完毕
	BS004_COM1_Send_Str_Body("finish to init bluetooth device.");	//圆点博士:未知命令
	//发送数据尾
	BS004_COM1_Send_Str_Tail();		
}
//串口中断初始化，该函数在串口初始化中调用，只有调用该函数之后 收到数据时才会自动进入接收中断
void BS004_COM1_NVIC_Configuration(void)				//圆点博士:设置串口中断优先级
{
	NVIC_InitTypeDef NVIC_InitStructure;
	//设置要开启de中断向量为 串口1中断
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	//设置该中断通道的抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//开启该中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}





