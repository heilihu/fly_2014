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
#include "etootle_imu.h" 
//��������Ϊ�����������ȣ�����ϵ������������������
float bs004_mpu6050_gyro_scale=0,bs004_mpu6050_pi_scale=0,bs004_gyro_to_rad_scale=0,bs004_hmc5883l_mag_scale=0;
//������ͨ���˲��������
extern float bs004_mpu6050_gyro_angel_pitch_ave,bs004_mpu6050_gyro_angel_roll_ave,bs004_mpu6050_gyro_angel_yaw_ave;
//������ͨ�������˲��������
extern float bs004_mpu6050_acc_angel_pitch_ave,bs004_mpu6050_acc_angel_roll_ave,bs004_mpu6050_acc_angel_yaw_ave;
//��̬��Ԫ��
float q0=1,q1=0,q2=0,q3=0;	
//����м����
float exInt=0,eyInt=0,ezInt=0;	
//IMU����õ���ŷ���ǣ����и�����pitch---(-180 180),�����roll---(-90 90),ƫ����yaw---(-180 180)
float bs004_imu_pitch=0,bs004_imu_roll=0,bs004_imu_yaw=0;
//�ֱ�Ϊ�����ӷ�����P��������������r,��̬�������ڵ�һ�룬����Ԫ��΢�ַ������ʱʹ��
float bs004_quad_Kp=0,bs004_quad_Ki=0,bs004_quad_halfT=0;			

//���������Ǻͼ��ٶȼƵĻ����˲�������̬ N����ϵ��ʾ�ο�����ϵ��B����ϵ��ʾ��������ϵ
unsigned char BS004_IMU_Update(float ax,float ay,float az,float gx,float gy,float gz) 
{
	float norm;					//����ռ������ķ������м����
	float vx, vy, vz;		//��ǰ��̬����ֱ�����ϵķ�����N����ϵ�еķ�����B����ϵ�еı�ʾ
	float ex, ey, ez;  //���ٶȼƱ�ʾ����ת�͵�ǰ����ֱ�����ϵķ�����B����ϵ�еķ��������
	float gz_input;			//������z��Ƕȼ���(����)���м����
  //	
	//Բ�㲩ʿ:��Ԫ���˷�����
	float q0q0 = q0 * q0;							
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q1q1 = q1 * q1;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;
	//	������z��Ƕȼ�����м����
	gz_input=gz*bs004_quad_halfT;
	//
	//Բ�㲩ʿ:���ٶȹ�һ������
	norm = sqrt(ax*ax + ay*ay + az*az);     
	if(norm==0) return 0;	
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;   
  //	
	//Բ�㲩ʿ:����С��������ϵ	 �ο�����ϵ�е���̬����ֱ������ת����������ϵB�����������ҵĵ�3��
	vx = 2*(q1q3 - q0q2);								
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	//
	//Բ�㲩ʿ:����ϵ�������������  ������ٶȼƼ����������ٶȱ�ʾ����ת��ǰ��̬����ֱ����֮��Ĳ�ֵ����������������ǰ��̬�����ý���ƫ��
	ex = (ay*vz - az*vy);								
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	//
	//Բ�㲩ʿ:��������  PI����������ƫ�ƫ���������
	exInt = exInt + ex*bs004_quad_Ki;
	eyInt = eyInt + ey*bs004_quad_Ki;
	ezInt = ezInt + ez*bs004_quad_Ki;
	//
	//Բ�㲩ʿ:�������ں� ƫ��ı����������㣬����������ֵ
	gx = gx + bs004_quad_Kp*ex + exInt;
	gy = gy + bs004_quad_Kp*ey + eyInt;
	gz = gz + bs004_quad_Kp*ez + ezInt;
	//
	//Բ�㲩ʿ:������Ԫ����
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*bs004_quad_halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*bs004_quad_halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*bs004_quad_halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*bs004_quad_halfT;  
	//
	//Բ�㲩ʿ:��Ԫ����һ������
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	if(norm==0) return 0;							//��ֹ����
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	//
	//Բ�㲩ʿ:��Ԫ����ŷ����ת������ת����ΪZ-Y-X
	bs004_imu_roll=asin(-2*q1q3 + 2*q0q2)*57.30f;
  bs004_imu_pitch=atan2(2*q2q3 + 2*q0q1, -2*q1q1-2*q2q2 + 1)*57.30f; 
	//������z����ֵõ�ƫ���ǣ������ýǶ�û�еشŽ�����һ�㲻��
  bs004_imu_yaw=bs004_imu_yaw+2*gz_input/bs004_mpu6050_gyro_scale;
	//������̬��ɱ�־
	return 1;	
}


