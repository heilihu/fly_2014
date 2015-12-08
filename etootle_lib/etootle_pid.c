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
#include "etootle_pid.h" 
//PID 参数变量设定
extern unsigned int pid_setting_P_value[3];		//圆点博士:PID的P参数，含X轴,Y轴,Z轴
extern unsigned int pid_setting_I_value[3];		//圆点博士:PID的I参数，含X轴,Y轴,Z轴
extern unsigned int pid_setting_D_value[3];		//圆点博士:PID的D参数，含X轴,Y轴,Z轴
extern unsigned int pid_setting_M_value[3];		//圆点博士:PID的M参数，含X轴,Y轴,Z轴
//PID参数变量 X轴，Y轴， Z轴
float bs004_pitch_p,bs004_roll_p,bs004_yaw_p;
float bs004_pitch_i,bs004_roll_i,bs004_yaw_i;
float bs004_pitch_d,bs004_roll_d,bs004_yaw_d;
//四轴中心点角度测量
int bs004_pitch_m,bs004_roll_m,bs004_yaw_m;
float bs004_pitch_mf,bs004_roll_mf,bs004_yaw_mf;
//X，Y，Z轴方向角速度，通过mpu6050互补滤波得到
extern float bs004_mpu6050_gyro_angel_pitch_ave,bs004_mpu6050_gyro_angel_roll_ave,bs004_mpu6050_gyro_angel_yaw_ave;
///X，Y，Z轴方向加速度，通过mpu6050互补滤波得到
extern float bs004_mpu6050_acc_angel_pitch_ave,bs004_mpu6050_acc_angel_roll_ave,bs004_mpu6050_acc_angel_yaw_ave;
//遥控器油门，方向控制量
extern signed short bs004_fly_gas,bs004_fly_pitch,bs004_fly_roll,bs004_fly_yaw;
//遥控器油门，方向比例系数
float bs004_fly_gas_scale=0,bs004_fly_pitch_scale=0,bs004_fly_roll_scale=0,bs004_fly_yaw_scale=0;
//数值通过IMU测量得到
extern float bs004_imu_pitch,bs004_imu_roll,bs004_imu_yaw;
//当前角度
float bs004_angle_cur_pitch=0,bs004_angle_cur_roll=0,bs004_angle_cur_yaw=0;
//上一个角度
float bs004_angle_last_pitch=0,bs004_angle_last_roll=0,bs004_angle_last_yaw=0;
//角度积分误差
float bs004_angle_dif_pitch=0,bs004_angle_dif_roll=0,bs004_angle_dif_yaw=0;
//积分系数
float bs004_angle_int_pitch=0,bs004_angle_int_roll=0,bs004_angle_int_yaw=0;
//电机参数
float bs004_fly_m1=0,bs004_fly_m2=0,bs004_fly_m3=0,bs004_fly_m4=0;
//遥控器方向判断
float bs004_fly_pitch_dir,bs004_fly_roll_dir,bs004_fly_yaw_dir;
/*
 *通过PID反馈控制飞行器姿态的函数
  *设计飞信其位置控制的3个独立的控制量x,y,z的PID控制器进行3个通道的位置控制，这三个通道的控制器同时相互作用
	*电机输出=当前输出+P*目标与当前角度误差+I*角度误差之和+D*角度变化量
	*规定顺时针方向为角度正方向，由此确定PID运算中的加减符号
 */
void BS004_PID_Control(void)
{
	//将P赋值给X，Y，Z轴变量
	bs004_roll_p =pid_setting_P_value[0]/10.0f;
	bs004_pitch_p=pid_setting_P_value[1]/10.0f;
	bs004_yaw_p  =pid_setting_P_value[2]/10.0f;
	//将I赋值给X，Y，Z轴变量	
	bs004_roll_i =pid_setting_I_value[0]/10000.0f;
	bs004_pitch_i=pid_setting_I_value[1]/10000.0f;
	bs004_yaw_i  =pid_setting_I_value[2]/10000.0f;
	//将D赋值给X，Y，Z轴变量	
	bs004_roll_d =pid_setting_D_value[0]/1000.0f;
	bs004_pitch_d=pid_setting_D_value[1]/1000.0f;
	bs004_yaw_d  =pid_setting_D_value[2]/1000.0f;
	//将中心点参数赋值到相应变量
	bs004_roll_m=pid_setting_M_value[0];
	bs004_pitch_m=pid_setting_M_value[1];
	bs004_yaw_m=pid_setting_M_value[2];
	//中心点参数判断
	if(bs004_roll_m >1000) bs004_roll_m =-(bs004_roll_m-1000);		//roll值大于1000，角度为负 下同理
	if(bs004_pitch_m>1000) bs004_pitch_m=-(bs004_pitch_m-1000);
	if(bs004_yaw_m  >1000) bs004_yaw_m  =-(bs004_yaw_m-1000);
	//中心点参数运算
	bs004_roll_mf=bs004_roll_m/10.0f;
	bs004_pitch_mf=bs004_pitch_m/10.0f;
	bs004_yaw_mf=bs004_yaw_m/10.0f;
	//遥控器控制方向运算
	bs004_fly_pitch_dir=(float)bs004_fly_pitch/10.0f;
	bs004_fly_roll_dir=(float)bs004_fly_roll/10.0f;
	bs004_fly_yaw_dir=(float)bs004_fly_yaw/10.0f;
	//
	//圆点博士:融合遥控器控制信号
	bs004_fly_m1=bs004_fly_gas*bs004_fly_gas_scale - bs004_fly_pitch*bs004_fly_pitch_scale + bs004_fly_yaw*bs004_fly_yaw_scale;
	bs004_fly_m3=bs004_fly_gas*bs004_fly_gas_scale + bs004_fly_pitch*bs004_fly_pitch_scale + bs004_fly_yaw*bs004_fly_yaw_scale;
	bs004_fly_m2=bs004_fly_gas*bs004_fly_gas_scale - bs004_fly_roll*bs004_fly_roll_scale   - bs004_fly_yaw*bs004_fly_yaw_scale;
	bs004_fly_m4=bs004_fly_gas*bs004_fly_gas_scale + bs004_fly_roll*bs004_fly_roll_scale   - bs004_fly_yaw*bs004_fly_yaw_scale;
	//比例系数运算
	//pitch期望值与IMU测量值作插值运算，作为比例系数
	bs004_angle_cur_pitch=bs004_imu_pitch-bs004_pitch_mf-bs004_fly_pitch_dir;
	//roll期望值与IMU测量值作插值运算，作为比例系数	
	bs004_angle_cur_roll =bs004_imu_roll-bs004_roll_mf-bs004_fly_roll_dir;
	//yaw期望值与IMU测量值作插值运算，作为比例系数	
	bs004_angle_cur_yaw  =bs004_imu_yaw-bs004_yaw_mf-bs004_fly_yaw_dir;
	//
	//圆点博士:融合P角度比例控制
	bs004_fly_m1=bs004_fly_m1 + bs004_pitch_p*bs004_angle_cur_pitch - bs004_roll_p *bs004_angle_cur_roll + bs004_yaw_p*bs004_angle_cur_yaw;
	bs004_fly_m2=bs004_fly_m2 - bs004_pitch_p*bs004_angle_cur_pitch - bs004_roll_p *bs004_angle_cur_roll - bs004_yaw_p*bs004_angle_cur_yaw;
	bs004_fly_m3=bs004_fly_m3 - bs004_pitch_p*bs004_angle_cur_pitch + bs004_roll_p *bs004_angle_cur_roll + bs004_yaw_p*bs004_angle_cur_yaw;
	bs004_fly_m4=bs004_fly_m4 + bs004_pitch_p*bs004_angle_cur_pitch + bs004_roll_p *bs004_angle_cur_roll - bs004_yaw_p*bs004_angle_cur_yaw;
	//积分系数运算
	//将每次pitch角度误差相加，作为积分系数
	bs004_angle_int_pitch=bs004_angle_int_pitch+bs004_angle_cur_pitch;
	bs004_angle_int_roll =bs004_angle_int_roll +bs004_angle_cur_roll;	
	bs004_angle_int_roll =bs004_angle_int_yaw +bs004_angle_cur_yaw;	
	//
  //圆点博士:融合I，积分控制
	bs004_fly_m1=bs004_fly_m1 + bs004_pitch_i*bs004_angle_int_pitch - bs004_roll_i *bs004_angle_int_roll + bs004_yaw_i *bs004_angle_int_yaw;
	bs004_fly_m2=bs004_fly_m2 - bs004_pitch_i*bs004_angle_int_pitch - bs004_roll_i *bs004_angle_int_roll - bs004_yaw_i *bs004_angle_int_yaw;
	bs004_fly_m3=bs004_fly_m3 - bs004_pitch_i*bs004_angle_int_pitch + bs004_roll_i *bs004_angle_int_roll + bs004_yaw_i *bs004_angle_int_yaw;
	bs004_fly_m4=bs004_fly_m4 + bs004_pitch_i*bs004_angle_int_pitch + bs004_roll_i *bs004_angle_int_roll - bs004_yaw_i *bs004_angle_int_yaw;
	//角度误差计算，将角度变化量即角速度作为微分系数
	//俯仰角度误差
	bs004_angle_dif_pitch=bs004_angle_cur_pitch-bs004_angle_last_pitch;
	bs004_angle_dif_roll =bs004_angle_cur_roll-bs004_angle_last_roll;
	bs004_angle_dif_yaw  =bs004_angle_last_yaw-bs004_angle_cur_yaw;
	//
	//圆点博士:融合D
	bs004_fly_m1=bs004_fly_m1 + bs004_pitch_d*bs004_mpu6050_gyro_angel_pitch_ave - bs004_roll_d *bs004_mpu6050_gyro_angel_roll_ave + bs004_yaw_d*bs004_mpu6050_gyro_angel_yaw_ave;
	bs004_fly_m2=bs004_fly_m2 - bs004_pitch_d*bs004_mpu6050_gyro_angel_pitch_ave - bs004_roll_d *bs004_mpu6050_gyro_angel_roll_ave - bs004_yaw_d*bs004_mpu6050_gyro_angel_yaw_ave;
	bs004_fly_m3=bs004_fly_m3 - bs004_pitch_d*bs004_mpu6050_gyro_angel_pitch_ave + bs004_roll_d *bs004_mpu6050_gyro_angel_roll_ave + bs004_yaw_d*bs004_mpu6050_gyro_angel_yaw_ave;
	bs004_fly_m4=bs004_fly_m4 + bs004_pitch_d*bs004_mpu6050_gyro_angel_pitch_ave + bs004_roll_d *bs004_mpu6050_gyro_angel_roll_ave - bs004_yaw_d*bs004_mpu6050_gyro_angel_yaw_ave;
	//将当前赋值给下一次
	bs004_angle_last_pitch=bs004_angle_cur_pitch;
	bs004_angle_last_roll =bs004_angle_cur_roll;	
	bs004_angle_last_yaw =bs004_angle_cur_yaw;	
}

