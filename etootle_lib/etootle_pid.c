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
#include "etootle_pid.h" 
//PID ���������趨
extern unsigned int pid_setting_P_value[3];		//Բ�㲩ʿ:PID��P��������X��,Y��,Z��
extern unsigned int pid_setting_I_value[3];		//Բ�㲩ʿ:PID��I��������X��,Y��,Z��
extern unsigned int pid_setting_D_value[3];		//Բ�㲩ʿ:PID��D��������X��,Y��,Z��
extern unsigned int pid_setting_M_value[3];		//Բ�㲩ʿ:PID��M��������X��,Y��,Z��
//PID�������� X�ᣬY�ᣬ Z��
float bs004_pitch_p,bs004_roll_p,bs004_yaw_p;
float bs004_pitch_i,bs004_roll_i,bs004_yaw_i;
float bs004_pitch_d,bs004_roll_d,bs004_yaw_d;
//�������ĵ�ǶȲ���
int bs004_pitch_m,bs004_roll_m,bs004_yaw_m;
float bs004_pitch_mf,bs004_roll_mf,bs004_yaw_mf;
//X��Y��Z�᷽����ٶȣ�ͨ��mpu6050�����˲��õ�
extern float bs004_mpu6050_gyro_angel_pitch_ave,bs004_mpu6050_gyro_angel_roll_ave,bs004_mpu6050_gyro_angel_yaw_ave;
///X��Y��Z�᷽����ٶȣ�ͨ��mpu6050�����˲��õ�
extern float bs004_mpu6050_acc_angel_pitch_ave,bs004_mpu6050_acc_angel_roll_ave,bs004_mpu6050_acc_angel_yaw_ave;
//ң�������ţ����������
extern signed short bs004_fly_gas,bs004_fly_pitch,bs004_fly_roll,bs004_fly_yaw;
//ң�������ţ��������ϵ��
float bs004_fly_gas_scale=0,bs004_fly_pitch_scale=0,bs004_fly_roll_scale=0,bs004_fly_yaw_scale=0;
//��ֵͨ��IMU�����õ�
extern float bs004_imu_pitch,bs004_imu_roll,bs004_imu_yaw;
//��ǰ�Ƕ�
float bs004_angle_cur_pitch=0,bs004_angle_cur_roll=0,bs004_angle_cur_yaw=0;
//��һ���Ƕ�
float bs004_angle_last_pitch=0,bs004_angle_last_roll=0,bs004_angle_last_yaw=0;
//�ǶȻ������
float bs004_angle_dif_pitch=0,bs004_angle_dif_roll=0,bs004_angle_dif_yaw=0;
//����ϵ��
float bs004_angle_int_pitch=0,bs004_angle_int_roll=0,bs004_angle_int_yaw=0;
//�������
float bs004_fly_m1=0,bs004_fly_m2=0,bs004_fly_m3=0,bs004_fly_m4=0;
//ң���������ж�
float bs004_fly_pitch_dir,bs004_fly_roll_dir,bs004_fly_yaw_dir;
/*
 *ͨ��PID�������Ʒ�������̬�ĺ���
  *��Ʒ�����λ�ÿ��Ƶ�3�������Ŀ�����x,y,z��PID����������3��ͨ����λ�ÿ��ƣ�������ͨ���Ŀ�����ͬʱ�໥����
	*������=��ǰ���+P*Ŀ���뵱ǰ�Ƕ����+I*�Ƕ����֮��+D*�Ƕȱ仯��
	*�涨˳ʱ�뷽��Ϊ�Ƕ��������ɴ�ȷ��PID�����еļӼ�����
 */
void BS004_PID_Control(void)
{
	//��P��ֵ��X��Y��Z�����
	bs004_roll_p =pid_setting_P_value[0]/10.0f;
	bs004_pitch_p=pid_setting_P_value[1]/10.0f;
	bs004_yaw_p  =pid_setting_P_value[2]/10.0f;
	//��I��ֵ��X��Y��Z�����	
	bs004_roll_i =pid_setting_I_value[0]/10000.0f;
	bs004_pitch_i=pid_setting_I_value[1]/10000.0f;
	bs004_yaw_i  =pid_setting_I_value[2]/10000.0f;
	//��D��ֵ��X��Y��Z�����	
	bs004_roll_d =pid_setting_D_value[0]/1000.0f;
	bs004_pitch_d=pid_setting_D_value[1]/1000.0f;
	bs004_yaw_d  =pid_setting_D_value[2]/1000.0f;
	//�����ĵ������ֵ����Ӧ����
	bs004_roll_m=pid_setting_M_value[0];
	bs004_pitch_m=pid_setting_M_value[1];
	bs004_yaw_m=pid_setting_M_value[2];
	//���ĵ�����ж�
	if(bs004_roll_m >1000) bs004_roll_m =-(bs004_roll_m-1000);		//rollֵ����1000���Ƕ�Ϊ�� ��ͬ��
	if(bs004_pitch_m>1000) bs004_pitch_m=-(bs004_pitch_m-1000);
	if(bs004_yaw_m  >1000) bs004_yaw_m  =-(bs004_yaw_m-1000);
	//���ĵ��������
	bs004_roll_mf=bs004_roll_m/10.0f;
	bs004_pitch_mf=bs004_pitch_m/10.0f;
	bs004_yaw_mf=bs004_yaw_m/10.0f;
	//ң�������Ʒ�������
	bs004_fly_pitch_dir=(float)bs004_fly_pitch/10.0f;
	bs004_fly_roll_dir=(float)bs004_fly_roll/10.0f;
	bs004_fly_yaw_dir=(float)bs004_fly_yaw/10.0f;
	//
	//Բ�㲩ʿ:�ں�ң���������ź�
	bs004_fly_m1=bs004_fly_gas*bs004_fly_gas_scale - bs004_fly_pitch*bs004_fly_pitch_scale + bs004_fly_yaw*bs004_fly_yaw_scale;
	bs004_fly_m3=bs004_fly_gas*bs004_fly_gas_scale + bs004_fly_pitch*bs004_fly_pitch_scale + bs004_fly_yaw*bs004_fly_yaw_scale;
	bs004_fly_m2=bs004_fly_gas*bs004_fly_gas_scale - bs004_fly_roll*bs004_fly_roll_scale   - bs004_fly_yaw*bs004_fly_yaw_scale;
	bs004_fly_m4=bs004_fly_gas*bs004_fly_gas_scale + bs004_fly_roll*bs004_fly_roll_scale   - bs004_fly_yaw*bs004_fly_yaw_scale;
	//����ϵ������
	//pitch����ֵ��IMU����ֵ����ֵ���㣬��Ϊ����ϵ��
	bs004_angle_cur_pitch=bs004_imu_pitch-bs004_pitch_mf-bs004_fly_pitch_dir;
	//roll����ֵ��IMU����ֵ����ֵ���㣬��Ϊ����ϵ��	
	bs004_angle_cur_roll =bs004_imu_roll-bs004_roll_mf-bs004_fly_roll_dir;
	//yaw����ֵ��IMU����ֵ����ֵ���㣬��Ϊ����ϵ��	
	bs004_angle_cur_yaw  =bs004_imu_yaw-bs004_yaw_mf-bs004_fly_yaw_dir;
	//
	//Բ�㲩ʿ:�ں�P�Ƕȱ�������
	bs004_fly_m1=bs004_fly_m1 + bs004_pitch_p*bs004_angle_cur_pitch - bs004_roll_p *bs004_angle_cur_roll + bs004_yaw_p*bs004_angle_cur_yaw;
	bs004_fly_m2=bs004_fly_m2 - bs004_pitch_p*bs004_angle_cur_pitch - bs004_roll_p *bs004_angle_cur_roll - bs004_yaw_p*bs004_angle_cur_yaw;
	bs004_fly_m3=bs004_fly_m3 - bs004_pitch_p*bs004_angle_cur_pitch + bs004_roll_p *bs004_angle_cur_roll + bs004_yaw_p*bs004_angle_cur_yaw;
	bs004_fly_m4=bs004_fly_m4 + bs004_pitch_p*bs004_angle_cur_pitch + bs004_roll_p *bs004_angle_cur_roll - bs004_yaw_p*bs004_angle_cur_yaw;
	//����ϵ������
	//��ÿ��pitch�Ƕ������ӣ���Ϊ����ϵ��
	bs004_angle_int_pitch=bs004_angle_int_pitch+bs004_angle_cur_pitch;
	bs004_angle_int_roll =bs004_angle_int_roll +bs004_angle_cur_roll;	
	bs004_angle_int_roll =bs004_angle_int_yaw +bs004_angle_cur_yaw;	
	//
  //Բ�㲩ʿ:�ں�I�����ֿ���
	bs004_fly_m1=bs004_fly_m1 + bs004_pitch_i*bs004_angle_int_pitch - bs004_roll_i *bs004_angle_int_roll + bs004_yaw_i *bs004_angle_int_yaw;
	bs004_fly_m2=bs004_fly_m2 - bs004_pitch_i*bs004_angle_int_pitch - bs004_roll_i *bs004_angle_int_roll - bs004_yaw_i *bs004_angle_int_yaw;
	bs004_fly_m3=bs004_fly_m3 - bs004_pitch_i*bs004_angle_int_pitch + bs004_roll_i *bs004_angle_int_roll + bs004_yaw_i *bs004_angle_int_yaw;
	bs004_fly_m4=bs004_fly_m4 + bs004_pitch_i*bs004_angle_int_pitch + bs004_roll_i *bs004_angle_int_roll - bs004_yaw_i *bs004_angle_int_yaw;
	//�Ƕ������㣬���Ƕȱ仯�������ٶ���Ϊ΢��ϵ��
	//�����Ƕ����
	bs004_angle_dif_pitch=bs004_angle_cur_pitch-bs004_angle_last_pitch;
	bs004_angle_dif_roll =bs004_angle_cur_roll-bs004_angle_last_roll;
	bs004_angle_dif_yaw  =bs004_angle_last_yaw-bs004_angle_cur_yaw;
	//
	//Բ�㲩ʿ:�ں�D
	bs004_fly_m1=bs004_fly_m1 + bs004_pitch_d*bs004_mpu6050_gyro_angel_pitch_ave - bs004_roll_d *bs004_mpu6050_gyro_angel_roll_ave + bs004_yaw_d*bs004_mpu6050_gyro_angel_yaw_ave;
	bs004_fly_m2=bs004_fly_m2 - bs004_pitch_d*bs004_mpu6050_gyro_angel_pitch_ave - bs004_roll_d *bs004_mpu6050_gyro_angel_roll_ave - bs004_yaw_d*bs004_mpu6050_gyro_angel_yaw_ave;
	bs004_fly_m3=bs004_fly_m3 - bs004_pitch_d*bs004_mpu6050_gyro_angel_pitch_ave + bs004_roll_d *bs004_mpu6050_gyro_angel_roll_ave + bs004_yaw_d*bs004_mpu6050_gyro_angel_yaw_ave;
	bs004_fly_m4=bs004_fly_m4 + bs004_pitch_d*bs004_mpu6050_gyro_angel_pitch_ave + bs004_roll_d *bs004_mpu6050_gyro_angel_roll_ave - bs004_yaw_d*bs004_mpu6050_gyro_angel_yaw_ave;
	//����ǰ��ֵ����һ��
	bs004_angle_last_pitch=bs004_angle_cur_pitch;
	bs004_angle_last_roll =bs004_angle_cur_roll;	
	bs004_angle_last_yaw =bs004_angle_cur_yaw;	
}

