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
#include "etootle_imu.h" 
//变量依次为陀螺仪灵敏度，弧度系数，电子罗盘灵敏度
float bs004_mpu6050_gyro_scale=0,bs004_mpu6050_pi_scale=0,bs004_gyro_to_rad_scale=0,bs004_hmc5883l_mag_scale=0;
//陀螺仪通过滤波后的数据
extern float bs004_mpu6050_gyro_angel_pitch_ave,bs004_mpu6050_gyro_angel_roll_ave,bs004_mpu6050_gyro_angel_yaw_ave;
//陀螺仪通过数字滤波后的数据
extern float bs004_mpu6050_acc_angel_pitch_ave,bs004_mpu6050_acc_angel_roll_ave,bs004_mpu6050_acc_angel_yaw_ave;
//姿态四元数
float q0=1,q1=0,q2=0,q3=0;	
//误差中间变量
float exInt=0,eyInt=0,ezInt=0;	
//IMU计算得到的欧拉角，其中俯仰角pitch---(-180 180),横滚角roll---(-90 90),偏航角yaw---(-180 180)
float bs004_imu_pitch=0,bs004_imu_roll=0,bs004_imu_yaw=0;
//分别为比例加分器的P，比例积分器的r,姿态更新周期的一半，在四元数微分方程求解时使用
float bs004_quad_Kp=0,bs004_quad_Ki=0,bs004_quad_halfT=0;			

//基于陀螺仪和加速度计的互补滤波解算姿态 N坐标系表示参考坐标系，B坐标系表示机体坐标系
unsigned char BS004_IMU_Update(float ax,float ay,float az,float gx,float gy,float gz) 
{
	float norm;					//计算空间向量的范数的中间变量
	float vx, vy, vz;		//当前姿态在竖直方向上的分量即N坐标系中的分量在B坐标系中的表示
	float ex, ey, ez;  //加速度计表示的旋转和当前在竖直方向上的分量在B坐标系中的分量外积差
	float gz_input;			//陀螺仪z轴角度计算(积分)的中间变量
  //	
	//圆点博士:四元数乘法运算
	float q0q0 = q0 * q0;							
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q1q1 = q1 * q1;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;
	//	陀螺仪z轴角度计算的中间变量
	gz_input=gz*bs004_quad_halfT;
	//
	//圆点博士:加速度归一化处理
	norm = sqrt(ax*ax + ay*ay + az*az);     
	if(norm==0) return 0;	
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;   
  //	
	//圆点博士:建立小四轴坐标系	 参考坐标系中的姿态的竖直分量旋转到载体坐标系B，即方向余弦的第3列
	vx = 2*(q1q3 - q0q2);								
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	//
	//圆点博士:坐标系和重力叉积运算  计算加速度计检测的重力加速度表示的旋转当前姿态的竖直分量之间的差值，用重力场矫正当前姿态，所得矫正偏差
	ex = (ay*vz - az*vy);								
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	//
	//圆点博士:比例运算  PI控制器矫正偏差，偏差积分运算
	exInt = exInt + ex*bs004_quad_Ki;
	eyInt = eyInt + ey*bs004_quad_Ki;
	ezInt = ezInt + ez*bs004_quad_Ki;
	//
	//圆点博士:陀螺仪融合 偏差的比例积分运算，纠正陀螺仪值
	gx = gx + bs004_quad_Kp*ex + exInt;
	gy = gy + bs004_quad_Kp*ey + eyInt;
	gz = gz + bs004_quad_Kp*ez + ezInt;
	//
	//圆点博士:整合四元数率
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*bs004_quad_halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*bs004_quad_halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*bs004_quad_halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*bs004_quad_halfT;  
	//
	//圆点博士:四元数归一化处理
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	if(norm==0) return 0;							//防止出错
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	//
	//圆点博士:四元数到欧拉角转换，旋转次序为Z-Y-X
	bs004_imu_roll=asin(-2*q1q3 + 2*q0q2)*57.30f;
  bs004_imu_pitch=atan2(2*q2q3 + 2*q0q1, -2*q1q1-2*q2q2 + 1)*57.30f; 
	//陀螺仪z轴积分得到偏航角，不过该角度没有地磁矫正，一般不用
  bs004_imu_yaw=bs004_imu_yaw+2*gz_input/bs004_mpu6050_gyro_scale;
	//更新姿态完成标志
	return 1;	
}


