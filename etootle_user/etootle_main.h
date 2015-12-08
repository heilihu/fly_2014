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
#include "stm32f10x_lib.h"
#include <math.h>
#include "etootle_led.h"
#include "etootle_bluetooth.h"
#include "etootle_motor.h"
#include "etootle_adc.h"
#include "etootle_sys.h"
#include "etootle_mpu6050.h"
#include "etootle_imu.h"
#include "etootle_parameter.h"

#define IDLE 0
#define UPDATE_COMMAND 1
#define UPDATE_MOTOR   2
#define RESET_MOTOR    3

extern float bs004_mpu6050_gyro_scale,bs004_mpu6050_pi_scale,bs004_gyro_to_rad_scale,bs004_hmc5883l_mag_scale;
extern float bs004_mpu6050_gyro_angel_pitch_ave,bs004_mpu6050_gyro_angel_roll_ave,bs004_mpu6050_gyro_angel_yaw_ave;
extern float bs004_mpu6050_acc_angel_pitch_ave,bs004_mpu6050_acc_angel_roll_ave,bs004_mpu6050_acc_angel_yaw_ave;
extern float bs004_imu_pitch,bs004_imu_roll,bs004_imu_yaw;
extern signed short  bs004_fly_gas,bs004_fly_pitch,bs004_fly_roll,bs004_fly_yaw;
extern unsigned char BS004_Ctrl_Gas,BS004_Ctrl_Valid,BS004_Ctrl_Gas_Noise;
extern signed char BS004_Ctrl_Pitch,BS004_Ctrl_Roll,BS004_Ctrl_Yaw,BS004_Ctrl_Dir_Noise;
extern float bs004_fly_m1,bs004_fly_m2,bs004_fly_m3,bs004_fly_m4;
extern signed short bs004_fly_m1_out,bs004_fly_m2_out,bs004_fly_m3_out,bs004_fly_m4_out;	

unsigned char BS004_COM1_Task_Process(void);
void BS004_Quad_Calculation(void);
void ANBT_SEND_DMP_EULER_DATA(void);
void BS004_Motor_Control(void);


