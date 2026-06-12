/* Copyright (c)  2019-2040 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/																					重要的事情说三遍
                先驱者的历史已经证明，在当前国内略浮躁+躺平+内卷的大环境下，对于毫无收益的开源项目，单靠坊间飞控爱好者、
                个人情怀式、自发地主动输出去参与开源项目的方式行不通，好的开源项目需要请专职人员做好售后技术服务、配套
                手册和视频教程要覆盖新手入门到进阶阶段，使用过程中对用户反馈问题和需求进行统计、在实践中完成对产品的一
                次次完善与迭代升级。
-----------------------------------------------------------------------------------------------------------------------
*                                                 为什么选择无名创新？
*                                         感动人心价格厚道，最靠谱的开源飞控；
*                                         国内业界良心之作，最精致的售后服务；
*                                         追求极致用户体验，高效进阶学习之路；
*                                         萌新不再孤单求索，合理把握开源尺度；
*                                         响应国家扶贫号召，促进教育体制公平；
*                                         新时代奋斗最出彩，建人类命运共同体。 
-----------------------------------------------------------------------------------------------------------------------
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               飞跃雷区组第21届智能汽车竞赛交流群：579581554
*               无名创新开源飞控QQ群：2号群465082224、1号群540707961
*               CSDN博客：http://blog.csdn.net/u011992534
*               B站教学视频：https://space.bilibili.com/67803559/#/video				
*               无名创新国内首款TI开源飞控设计初衷、知乎专栏:https://zhuanlan.zhihu.com/p/54471146
*               淘宝店铺：https://shop348646912.taobao.com/
*               公司官网:www.nameless.tech
*               修改日期:2025/10/01                  
*               版本：枫叶飞控MaplePilot_V1.0
*               版权所有，盗版必究。
*               Copyright(C) 2019-2040 武汉无名创新科技有限公司 
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
/******************************觉得代码很有帮助，欢迎打赏一碗热干面（武科大二号门老汉口红油热干面4元一碗），无名小哥支付宝：1094744141@qq.com********/

#include "main.h"
#include "datatype.h"
#include "quaternion.h"
#include "wp_math.h"
#include "arm_math.h"
#include "maple_ahrs.h"

#define EKF_PQ_INITIAL 1e-5f//		1e-5f
#define EKF_QQ_INITIAL 1e-5f//		1e-5f  5e-6f
#define EKF_RA_INITIAL 0.1f

#define EKF_STATE_DIM 4					//q0 q1 q2 q3
#define EKF_MEASUREMENT_DIM 4		//qb0 qb1 qb2 qb3


static float P[EKF_STATE_DIM * EKF_STATE_DIM] = {
  EKF_PQ_INITIAL, 0, 0, 0,
  0, EKF_PQ_INITIAL, 0, 0,
  0, 0, EKF_PQ_INITIAL, 0,
  0, 0, 0, EKF_PQ_INITIAL,
};

static float Q[EKF_STATE_DIM * EKF_STATE_DIM] = {
  EKF_QQ_INITIAL, 0, 0, 0,
  0, EKF_QQ_INITIAL, 0, 0,
  0, 0, EKF_QQ_INITIAL, 0,
  0, 0, 0, EKF_QQ_INITIAL,
};

static float R[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM] = {
  EKF_RA_INITIAL, 0, 0,0,
  0, EKF_RA_INITIAL, 0,0,
  0, 0, EKF_RA_INITIAL,0,
  0, 0, 0, EKF_RA_INITIAL,
};

static float F[EKF_STATE_DIM * EKF_STATE_DIM] = {
  0.0f, 0, 0, 0,
  0, 0.0f, 0, 0,
  0, 0, 0.0f, 0,
  0, 0, 0, 0.0f,
};

static float H[EKF_MEASUREMENT_DIM * EKF_STATE_DIM] = {
  1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, 1, 0,
  0, 0, 0, 1,
};

//static 
static float X[EKF_STATE_DIM]={1.0,0,0,0};
static float KY[EKF_STATE_DIM];
//static float X_H[EKF_STATE_DIM];
//measurement
static float Y[EKF_MEASUREMENT_DIM];

static float PX[EKF_STATE_DIM * EKF_STATE_DIM];
static float PXX[EKF_STATE_DIM * EKF_STATE_DIM];
static float PXY[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
static float K[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
static float S[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];
static float SI[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];
static float HT[EKF_STATE_DIM*EKF_MEASUREMENT_DIM];
arm_matrix_instance_f32 I_Matrix;
arm_matrix_instance_f32 P_Matrix;
arm_matrix_instance_f32 Q_Matrix;
arm_matrix_instance_f32 R_Matrix;
arm_matrix_instance_f32 F_Matrix;
arm_matrix_instance_f32 H_Matrix;
arm_matrix_instance_f32 X_Matrix;
arm_matrix_instance_f32 KY_Matrix;
arm_matrix_instance_f32 Y_Matrix;
arm_matrix_instance_f32 PX_Matrix;
arm_matrix_instance_f32 PXX_Matrix;
arm_matrix_instance_f32 PXY_Matrix;
arm_matrix_instance_f32 K_Matrix;
arm_matrix_instance_f32 S_Matrix;
arm_matrix_instance_f32 SI_Matrix;
arm_matrix_instance_f32 HT_Matrix;


void ahrs_ekf_init(void)
{
#ifdef UPDATE_P_COMPLICATED	
  arm_mat_init_f32(&I_Matrix, EKF_STATE_DIM, EKF_STATE_DIM, I);
#endif	
  arm_mat_init_f32(&P_Matrix, EKF_STATE_DIM, EKF_STATE_DIM, P);
  arm_mat_init_f32(&Q_Matrix, EKF_STATE_DIM, EKF_STATE_DIM, Q);
  arm_mat_init_f32(&R_Matrix, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM, R);
  arm_mat_init_f32(&F_Matrix, EKF_STATE_DIM, EKF_STATE_DIM, F);
  arm_mat_init_f32(&H_Matrix, EKF_MEASUREMENT_DIM, EKF_STATE_DIM, H);
  arm_mat_init_f32(&X_Matrix, EKF_STATE_DIM, 1, X);
  arm_mat_init_f32(&KY_Matrix,EKF_STATE_DIM, 1, KY);
  arm_mat_init_f32(&Y_Matrix, EKF_MEASUREMENT_DIM, 1, Y);
  arm_mat_init_f32(&PX_Matrix, EKF_STATE_DIM, EKF_STATE_DIM, PX);
  arm_mat_init_f32(&PXX_Matrix, EKF_STATE_DIM, EKF_STATE_DIM, PXX);
  arm_mat_init_f32(&PXY_Matrix, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, PXY);
  arm_mat_init_f32(&K_Matrix, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, K);
  arm_mat_init_f32(&S_Matrix, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM, S);
  arm_mat_init_f32(&SI_Matrix, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM, SI);
  arm_mat_init_f32(&HT_Matrix, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, HT);
}

void ahrs_attitude_ekf(vector3f *gyro, vector3f *accel, float *obs,float dt)
{	
  static float q[4];
  //	float _accel[3];//g
  //	_accel[0]=accel->x;
  //	_accel[1]=accel->y;
  //	_accel[2]=accel->z;
  float halfdx, halfdy, halfdz;
  float neghalfdx, neghalfdy, neghalfdz;
  float halfdt = 0.5f * dt;
  halfdx = halfdt * gyro->x * DEG2RAD;
  halfdy = halfdt * gyro->y * DEG2RAD;
  halfdz = halfdt * gyro->z * DEG2RAD;
  
  neghalfdx=-halfdx;
  neghalfdy=-halfdy;
  neghalfdz=-halfdz;
  
  q[0] = X[0];
  q[1] = X[1];
  q[2] = X[2];
  q[3] = X[3];
  
  //四元数增量更新
  X[0] =q[0] - halfdx * q[1] - halfdy * q[2] - halfdz * q[3];
  X[1] =q[1] + halfdx * q[0] - halfdy * q[3] + halfdz * q[2];
  X[2] =q[2] + halfdx * q[3] + halfdy * q[0] - halfdz * q[1];
  X[3] =q[3] - halfdx * q[2] + halfdy * q[1] + halfdz * q[0];
  
  //计算状态转移矩阵
  F[0] = 1.0f;	  F[1] = neghalfdx; F[2] = neghalfdy; F[3] = neghalfdz;
  F[4] = halfdx;  F[5] = 1.0f;			F[6] = halfdz;    F[7] = neghalfdy;
  F[8] = halfdy;	F[9] = neghalfdz;	F[10]= 1.0f;		  F[11]= halfdx;
  F[12]= halfdz;  F[13]= halfdy;    F[14]= neghalfdx; F[15]= 1.0f;
  
  //计算先验协方差矩阵
  //P = F*P*F' + Q; 
  //update process noise	
  arm_mat_mult_f32(&F_Matrix, &P_Matrix, &PX_Matrix);
  arm_mat_trans_f32(&F_Matrix,&P_Matrix);
  arm_mat_mult_f32(&PX_Matrix,&P_Matrix,&PXX_Matrix);
  arm_mat_add_f32(&PXX_Matrix, &Q_Matrix, &P_Matrix);
  
  
  //计算卡尔曼增益
  arm_mat_trans_f32(&H_Matrix,&HT_Matrix);
  arm_mat_mult_f32(&P_Matrix,&HT_Matrix,&PXY_Matrix);
  arm_mat_mult_f32(&H_Matrix,&PXY_Matrix,&S_Matrix);
  arm_mat_add_f32(&S_Matrix,&R_Matrix,&S_Matrix);
  arm_mat_inverse_f32(&S_Matrix, &SI_Matrix);
  arm_mat_mult_f32(&PXY_Matrix,&SI_Matrix,&K_Matrix);
  
  Y[0] =(obs[0] - X[0]);
  Y[1] =(obs[1] - X[1]);
  Y[2] =(obs[2] - X[2]);
  Y[3] =(obs[3] - X[3]);
  
  //Update State Vector 
  arm_mat_mult_f32(&K_Matrix,&Y_Matrix,&KY_Matrix);
  arm_mat_add_f32(&X_Matrix,&KY_Matrix,&X_Matrix);
  
  //covariance measurement update
  //P = (I - K * H) * P
  //P = P - K * H * P
  //or
  //P=(I - K*H)*P*(I - K*H)' + K*R*K'
  arm_mat_mult_f32(&K_Matrix,&H_Matrix,&PX_Matrix);
  arm_mat_mult_f32(&PX_Matrix,&P_Matrix,&PXX_Matrix);
  arm_mat_sub_f32(&P_Matrix,&PXX_Matrix,&P_Matrix);
  
  quaternion_normalize(X);
}

void ahrs_get_quaternion(float *q)
{
  q[0]=X[0];
  q[1]=X[1];
  q[2]=X[2];
  q[3]=X[3];
}


void ahrs_get_euler(float *rpy)
{
  quaternion_to_euler(X,rpy);
}

/*******************************************************************************************/
float betaDef=0.0125f;//0.0175f;
static float beta;				  // algorithm gain
static float q0,q1,q2,q3;	  // quaternion of sensor frame relative to auxiliary frame
float q_backups[20][4];
void madgwick_init(void) 
{
  beta = betaDef;
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
}

float KpMag=0.00f;
void madgwick_updateimu(vector3f *gyro_r,vector3f *accel_g,vector3f *mag,float dt,uint16_t sync)
{
  float gx=gyro_r->x * DEG2RAD;
  float gy=gyro_r->y * DEG2RAD;
  float gz=gyro_r->z * DEG2RAD;
  float ax=accel_g->x;
  float ay=accel_g->y;
  float az=accel_g->z;
  float norm;
  float s0, s1, s2, s3;
  float qdot1, qdot2, qdot3, qdot4;
  float _q1, _q2, _q3,_2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
  
  float gyro_mold=RADTODEG(sqrtf(gx*gx+gy*gy+gz*gz));
  float accel_mold=sqrtf(ax*ax+ay*ay+az*az);
  accel_mold=constrain_float(accel_mold,1.0f,16.0f);	
  
  beta=betaDef+0.02f*dt*constrain_float(gyro_mold,0,500);//0.035
  beta/=FastAbs(cube(accel_mold));
  beta=constrain_float(beta,betaDef,0.06f);
  for(uint16_t i=19;i>0;i--)
  {
    q_backups[i][0]=q_backups[i-1][0];
    q_backups[i][1]=q_backups[i-1][1];
    q_backups[i][2]=q_backups[i-1][2];
    q_backups[i][3]=q_backups[i-1][3];
  }
  q_backups[0][0]=q0;
  q_backups[0][1]=q1;
  q_backups[0][2]=q2;
  q_backups[0][3]=q3;
  
  // Rate of change of quaternion from gyroscope
  qdot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qdot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qdot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qdot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
  
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    
    // Normalise accelerometer measurement
    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    // Auxiliary variables to avoid repeated arithmetic
    _q1  = q_backups[sync][1];
    _q2  = q_backups[sync][2];
    _q3  = q_backups[sync][3];
    _2q0 = 2.0f * q_backups[sync][0];
    _2q1 = 2.0f * q_backups[sync][1];
    _2q2 = 2.0f * q_backups[sync][2];
    _2q3 = 2.0f * q_backups[sync][3];
    _4q0 = 4.0f * q_backups[sync][0];
    _4q1 = 4.0f * q_backups[sync][1];
    _4q2 = 4.0f * q_backups[sync][2];
    _8q1 = 8.0f * q_backups[sync][1];
    _8q2 = 8.0f * q_backups[sync][2];
    q0q0 = q_backups[sync][0] * q_backups[sync][0];
    q1q1 = q_backups[sync][1] * q_backups[sync][1];
    q2q2 = q_backups[sync][2] * q_backups[sync][2];
    q3q3 = q_backups[sync][3] * q_backups[sync][3];
    
    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax  +  4.0f* q0q0 * _q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * _q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * _q3 - _2q1 * ax + 4.0f * q2q2 * _q3 - _2q2 * ay;
    norm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    
    // Apply feedback step
    qdot1 -= beta * s0;
    qdot2 -= beta * s1;
    qdot3 -= beta * s2;
    qdot4 -= beta * s3;
  }
  
  // Integrate rate of change of quaternion to yield quaternion
  q0 += qdot1 * dt;
  q1 += qdot2 * dt;
  q2 += qdot3 * dt;
  q3 += qdot4 * dt;
  
  // Normalise quaternion
  norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= norm;
  q1 *= norm;
  q2 *= norm;
  q3 *= norm;
}



void madgwick_getangle(float* rpy)
{
  float roll,pitch,yaw;
  pitch= RADTODEG(atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2));
  roll = RADTODEG(asinf(-2.0f * (q1*q3 - q0*q2)));
  yaw  = RADTODEG(atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3));
  
  rpy[0] = roll;
  rpy[1] = pitch;
  rpy[2] = yaw;	
}



void madgwick_get_quaternion(float *q)
{
  q[0]=q0;
  q[1]=q1;
  q[2]=q2;
  q[3]=q3;
}

void madgwick_set_quaternion(float *q)
{
  static bool init=false;
  if(init)  return;
  q0=q[0];
  q1=q[1];
  q2=q[2];
  q3=q[3];
  init=true;
}


#define KpDef	(1.0f * 1.2f)	  //proportional gain 0.5
#define KiDef	(1.0f * 0.5f)	  //integral gain 		0.02
static float Kp = KpDef;
static float Ki = KiDef;
vector3f integral = {0.0f,0.0f,0.0f};
void mahony_updateimu(vector3f *gyro_r,vector3f *accel_g,float dt)
{
  float recipNorm;
  float gx=gyro_r->x * DEG2RAD;
  float gy=gyro_r->y * DEG2RAD;
  float gz=gyro_r->z * DEG2RAD;
  float ax=accel_g->x;
  float ay=accel_g->y;
  float az=accel_g->z;
  float gyro_mold=RADTODEG(sqrtf(gx*gx+gy*gy+gz*gz));
  float accel_mold=sqrtf(ax*ax+ay*ay+az*az);
  accel_mold=constrain_float(accel_mold,1.0f,16.0f);
  //归一化加速度计测量值
  recipNorm = invSqrt(ax * ax + ay * ay + az * az);
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;		
  
  float gbx, gby, gbz;
  float ex, ey, ez;
  
  //四元数预更新
  float halfdx, halfdy, halfdz;
  float halfdt = 0.5f * dt;
  
  //计算重力观测列向量
  gbx=2*(q1*q3-q0*q2);
  gby=2*(q2*q3+q0*q1);	
  gbz=q0*q0-q1*q1-q2*q2+q3*q3;
  
  //列向量叉乘求取状态误差
  ex=-(gby*az-gbz*ay) / FastAbs(cube(accel_mold));
  ey=-(gbz*ax-gbx*az) / FastAbs(cube(accel_mold));
  ez=-(gbx*ay-gby*ax) / FastAbs(cube(accel_mold));
  
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))//加速度计数据有效
  {
    if(Ki > 0.0f) 
    {
      if(gyro_mold<=50.0f)//角速度比较小时，引入积分修正
      {
        integral.x += Ki * ex * dt;
        integral.y += Ki * ey * dt;
        integral.z += Ki * ez * dt;
      }
    } 
    else 
    {
      integral.x = 0.0f;	
      integral.y = 0.0f;
      integral.z = 0.0f;
    }
    integral.x=constrain_float(integral.x,-0.5f,0.5f);
    integral.y=constrain_float(integral.y,-0.5f,0.5f);
    integral.z=constrain_float(integral.z,-0.5f,0.5f);
  }
  
  gx += Kp * ex  + integral.x;
  gy += Kp * ey  + integral.y;
  gz += Kp * ez  + integral.z;
  
  halfdx = halfdt * gx ;
  halfdy = halfdt * gy ;
  halfdz = halfdt * gz ;
  
  float q[4]={0};
  q[0]=q0;q[1]=q1;q[2]=q2;q[3]=q3;
  //四元数增量更新
  q0 =q[0] - halfdx * q[1] - halfdy * q[2] - halfdz * q[3];
  q1 =q[1] + halfdx * q[0] - halfdy * q[3] + halfdz * q[2];
  q2 =q[2] + halfdx * q[3] + halfdy * q[0] - halfdz * q[1];
  q3 =q[3] - halfdx * q[2] + halfdy * q[1] + halfdz * q[0];
  
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}



/******************************************************************/

