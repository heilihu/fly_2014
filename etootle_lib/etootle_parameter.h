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
#include "etootle_bluetooth.h"
#include "etootle_sys.h"
//
extern float bs004_filter_high;
extern float bs004_filter_low;
extern float bs004_filter_time;
extern unsigned int bs004_sys_timer_period;
extern float bs004_mpu6050_gyro_scale;
extern float bs004_mpu6050_pi_scale;
extern float bs004_hmc5883l_mag_scale;
extern unsigned int bs004_motor_pwm_period;
extern unsigned int BS004_Motor_Scale;
extern float bs004_quad_halfT;
extern float bs004_quad_Kp;
extern float bs004_quad_Ki;
//
extern unsigned int pid_setting_P_value[3],pid_setting_I_value[3],pid_setting_D_value[3],pid_setting_M_value[3];
extern float bs004_fly_gas_scale,bs004_fly_pitch_scale,bs004_fly_roll_scale,bs004_fly_yaw_scale;
//
extern signed short int bs004_mpu6050_gyro_pitch_cal,bs004_mpu6050_gyro_roll_cal,bs004_mpu6050_gyro_yaw_cal;
extern signed short int bs004_mpu6050_acc_pitch_cal,bs004_mpu6050_acc_roll_cal,bs004_mpu6050_acc_yaw_cal;
extern signed short int bs004_hmc5883l_mag_pitch_cal,bs004_hmc5883l_mag_roll_cal,bs004_hmc5883l_mag_yaw_cal;
//
void BS004_Load_Filter_Parameter(void);
void BS004_Load_Pid_Parameter(void);
void BS004_Load_Calibration_Parameter(void);
void BS004_Load_Fly_Parameter(void);
//
void BS004_Cal_MPU6050_Data(int *bs004_cal_data);
void BS004_Show_Calibrated_Data(void);




