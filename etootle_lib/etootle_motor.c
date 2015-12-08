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
#include "etootle_motor.h" 
//�������
extern float bs004_fly_m1,bs004_fly_m2,bs004_fly_m3,bs004_fly_m4;
//������
signed short bs004_fly_m1_out=0,bs004_fly_m2_out=0,bs004_fly_m3_out=0,bs004_fly_m4_out=0;	
//��ǰ�����ǣ�����ǣ�ƫ����
extern float bs004_angle_cur_pitch,bs004_angle_cur_roll,bs004_angle_cur_yaw;
//�ϴθ����ǣ�����ǣ�ƫ����
extern float bs004_angle_last_pitch,bs004_angle_last_roll,bs004_angle_last_yaw;
//�������
unsigned int bs004_motor_pwm_period=0;
//���Ų��������ű���
unsigned int BS004_Motor_Scale=0;
//�ĸ���������Ų���
unsigned int Motor_BS004_M1=0,Motor_BS004_M2=0,Motor_BS004_M3=0,Motor_BS004_M4=0;
//�򿪵���ĵ�Դ
void BS004_Motor_Power_On(void)
{
	GPIO_SetBits(BS004_MOTOR_POWER_MA_PORT, BS004_MOTOR_POWER_MA);		//Բ�㲩ʿ:�򿪵����Դ
	GPIO_SetBits(BS004_MOTOR_POWER_MB_PORT, BS004_MOTOR_POWER_MB);    //Բ�㲩ʿ:�򿪵����Դ
}
//�رյ���ĵ�Դ
void BS004_Motor_Power_Off(void)
{
	GPIO_ResetBits(BS004_MOTOR_POWER_MA_PORT, BS004_MOTOR_POWER_MA);	//Բ�㲩ʿ:�رյ����Դ
	GPIO_ResetBits(BS004_MOTOR_POWER_MB_PORT, BS004_MOTOR_POWER_MB);	//Բ�㲩ʿ:�رյ����Դ
}
//�򿪵����PWM���
void BS004_MOTOR_PWM_ON(void)		
{
	GPIO_SetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M1);			//Բ�㲩ʿ:��PWM���
	GPIO_SetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M2);			//Բ�㲩ʿ:��PWM���
	GPIO_SetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M3);			//Բ�㲩ʿ:��PWM���
	GPIO_SetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M4);			//Բ�㲩ʿ:��PWM���
}
//�رյ����PWM���
void BS004_MOTOR_PWM_OFF(void)		
{
	GPIO_ResetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M1);		//Բ�㲩ʿ:�ر�PWM���
	GPIO_ResetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M2);		//Բ�㲩ʿ:�ر�PWM���
	GPIO_ResetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M3);		//Բ�㲩ʿ:�ر�PWM���
	GPIO_ResetBits(BS004_MOTOR_PWM_PORT, BS004_MOTOR_PWM_M4);		//Բ�㲩ʿ:�ر�PWM���
}

//===============================================================
//PWM��IO�����ã��ĸ��������ΪM1-PC9,M2-PC7,M3-PC6,M4-PC8
void BS004_Motor_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	//ʹ��GPIO�˿�B��Cʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //Բ�㲩ʿ:����PWM�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //Բ�㲩ʿ:����PWM��Ϊ���
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure); 
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //Բ�㲩ʿ:����PWM�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //Բ�㲩ʿ:����PWM��Ϊ���
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //Բ�㲩ʿ:����PWM�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //Բ�㲩ʿ:����PWM��Ϊ���
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //Բ�㲩ʿ:����PWM�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //Բ�㲩ʿ:����PWM��Ϊ���
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//	
	BS004_MOTOR_PWM_OFF();															//Բ�㲩ʿ:����PWM�����Ϊ��
	//���õ����Դ�������� MA-PB9��MB-PC15
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_POWER_MA;					
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //Բ�㲩ʿ:���õ����Դ���ƿ�Ϊ���
  GPIO_Init(BS004_MOTOR_POWER_MA_PORT, &GPIO_InitStructure); 
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_POWER_MB;					
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  //Բ�㲩ʿ:���õ����Դ���ƿ�Ϊ���
  GPIO_Init(BS004_MOTOR_POWER_MB_PORT, &GPIO_InitStructure); 
	//
	BS004_Motor_Power_Off();														//Բ�㲩ʿ:�رյ����Դ
}
//����PWM�źŵĹ���ģʽ�������ڣ����ԣ�ռ�ձȵ�
void BS004_Motor_PWM_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
	//ʹ��TIM8��ϵͳʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //Բ�㲩ʿ:����LED�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//Բ�㲩ʿ:����LED��Ϊ���
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure); 
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //Բ�㲩ʿ:����LED�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//Բ�㲩ʿ:����LED��Ϊ���
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //Բ�㲩ʿ:����LED�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//Բ�㲩ʿ:����LED��Ϊ���
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MOTOR_PWM_M4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //Բ�㲩ʿ:����LED�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	  	//Բ�㲩ʿ:����LED��Ϊ���
  GPIO_Init(BS004_MOTOR_PWM_PORT, &GPIO_InitStructure);
	//	
	//Բ�㲩ʿ:PWMƵ��=BS004_MOTOR_PWM_CLK_36MHZ/��BS004_MOTOR_PWM_PERIOD+1)
	//����ʱ�ӷ�Ƶϵ�������ﲻ����
	TIM_TimeBaseStructure.TIM_ClockDivision = BS004_MOTOR_PWM_SYSCLK_DIV;
	//����PWM��Ԥ��Ƶϵ��
	TIM_TimeBaseStructure.TIM_Prescaler = BS004_MOTOR_PWM_CLK_72MHZ;
	//����PWM����ģʽ������Ϊ���ϼ���
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	//��������һ������ʱ��װ�����Զ���װ�ؼĴ�������ֵ
	TIM_TimeBaseStructure.TIM_Period = bs004_motor_pwm_period; 
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);								//Բ�㲩ʿ:����PWM���ں�Ƶ��		
	
	//����Ϊ��ʱ��ģʽ1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	//ѡ�������ʱ����״̬
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//����ͨ��1�ĵ�ƽ����ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_Pulse = Motor_BS004_M3;
	//����PWM���������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	//���û����˲�����
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High; 
	//���ÿ���״̬�µķǹ���״̬	
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	//ѡ�񻥲�����״̬�µķǹ���״̬
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OC1Init(TIM8, &TIM_OCInitStructure);											//Բ�㲩ʿ:����PWMռ�ձ�		
	
	//ѡ�������ʱ����״̬
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//����ͨ��2�ĵ�ƽ����ֵ
  TIM_OCInitStructure.TIM_Pulse = Motor_BS004_M2;
  TIM_OC2Init(TIM8, &TIM_OCInitStructure);											//Բ�㲩ʿ:����PWMռ�ձ�		
	//ѡ�������ʱ����״̬
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//����ͨ��3���ƽ����ֵ
  TIM_OCInitStructure.TIM_Pulse = Motor_BS004_M4;
  TIM_OC3Init(TIM8, &TIM_OCInitStructure);											//Բ�㲩ʿ:����PWMռ�ձ�		
	//����ͨ��4�ĵ�ƽ����ֵ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Motor_BS004_M1;
  TIM_OC4Init(TIM8, &TIM_OCInitStructure);											//Բ�㲩ʿ:����PWMռ�ձ�		
	//
	BS004_Motor_NVIC_Configuration();					//Բ�㲩ʿ:����PWM�ж����ȼ�	
	TIM_Cmd(TIM8, ENABLE);										//Բ�㲩ʿ:����PWM
	TIM_CtrlPWMOutputs(TIM8,ENABLE);          //Բ�㲩ʿ:����PWM���
	
	BS004_COM1_Send_Str_Head();								//���������ͷ�ļ�
	BS004_COM1_Send_Str_Body("finish to init motor device.");					//Բ�㲩ʿ:��ʼ��PWM IO
	BS004_COM1_Send_Str_Tail();								//���������β��
}

//�����жϺ���
void BS004_Motor_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;			//Բ�㲩ʿ:����PWM�ж����ȼ�	
	//
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQChannel;		//TIM8�ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		//��ռ���ȼ�0��
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					//�����ȼ�0��
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//TIM8�ж�ʹ��
  NVIC_Init(&NVIC_InitStructure);
}

//===============================================================
void BS004_Motor_Interrupt(void)					//Բ�㲩ʿ:PWM�жϺ���
{	
	TIM_SetCompare4(TIM8,Motor_BS004_M1);		//Բ�㲩ʿ:����PWMռ�ձ�  
	TIM_SetCompare2(TIM8,Motor_BS004_M2);	  //Բ�㲩ʿ:����PWMռ�ձ�  
	TIM_SetCompare1(TIM8,Motor_BS004_M3);   //Բ�㲩ʿ:����PWMռ�ձ�      
	TIM_SetCompare3(TIM8,Motor_BS004_M4);		//Բ�㲩ʿ:����PWMռ�ձ�  	    
}
//�������Ų���
signed short BS004_Motor_Speed_Scale(float motor_speed_input)
{
	float motor_speed_output;		//������
	//�����������
	if(motor_speed_input>BS004_FLY_MAX_OUT) motor_speed_output=BS004_FLY_MAX_OUT;
	//��С��������
	else if(motor_speed_input<BS004_FLY_MIN_OUT) motor_speed_output=BS004_FLY_MIN_OUT;
	else motor_speed_output=motor_speed_input;
	return motor_speed_output;
}

//�����λ
void BS004_Motor_Reset(void)
{
	bs004_fly_m1=0;
	bs004_fly_m2=0;	
	bs004_fly_m3=0;
	bs004_fly_m4=0;
	//��ǰ�����ǣ�����ǣ�ƫ����
	bs004_angle_cur_pitch=0;
	bs004_angle_cur_roll=0;
	bs004_angle_cur_yaw=0;
	//�ϴθ����ǣ�����ǣ�ƫ����
	bs004_angle_last_pitch=0;
	bs004_angle_last_roll=0;	
	bs004_angle_last_yaw=0;		
	//���µ������
	Motor_BS004_M1=BS004_FLY_MIN_OUT;
	Motor_BS004_M2=BS004_FLY_MIN_OUT;
	Motor_BS004_M3=BS004_FLY_MIN_OUT;
	Motor_BS004_M4=BS004_FLY_MIN_OUT;
	//����PWMռ�ձ�
	TIM_SetCompare4(TIM8,5);		//Բ�㲩ʿ:����PWMռ�ձ�  
	TIM_SetCompare2(TIM8,5);	  //Բ�㲩ʿ:����PWMռ�ձ�  
	TIM_SetCompare1(TIM8,5);   //Բ�㲩ʿ:����PWMռ�ձ�      
	TIM_SetCompare3(TIM8,5);		//Բ�㲩ʿ:����PWMռ�ձ� 
}






