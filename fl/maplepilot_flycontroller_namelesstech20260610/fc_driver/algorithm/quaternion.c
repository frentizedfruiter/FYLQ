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


#include "wp_math.h"
#include "quaternion.h"


void quaternion_to_cb2n(float *q,float *cb2n)
{
  float a=q[0];
  float b=q[1];
  float c=q[2];
  float d=q[3];
  float bc=b*c;
  float ad=a*d;
  float bd=b*d;
  float ac=a*c;
  float cd=c*d;
  float ab=a*b;
  float a2=a*a;
  float b2=b*b;
  float c2=c*c;
  float d2=d*d;
  cb2n[0]=a2+b2-c2-d2;
  cb2n[1]=2*(bc-ad);
  cb2n[2]=2*(bd+ac);  
  cb2n[3]=2*(bc+ad);
  cb2n[4]=a2-b2+c2-d2;
  cb2n[5]=2*(cd-ab);
  cb2n[6]=2*(bd-ac);
  cb2n[7]=2*(cd+ab);
  cb2n[8]=a2-b2-c2+d2;	
}



void euler_to_quaternion(float *rpy,float *q)
{
  float sPitch2, cPitch2; // sin(phi/2) and cos(phi/2)
  float sRoll2 , cRoll2;  // sin(theta/2) and cos(theta/2)
  float sYaw2  , cYaw2;   // sin(psi/2) and cos(psi/2)
  //calculate sines and cosines
  
  FastSinCos(0.5f * rpy[0]*DEG2RAD, &sRoll2, &cRoll2);//roll
  FastSinCos(0.5f * rpy[1]*DEG2RAD, &sPitch2,&cPitch2);//pitch
  FastSinCos(0.5f * rpy[2]*DEG2RAD, &sYaw2,  &cYaw2);//yaw
  
  // compute the quaternion elements
  q[0] = cPitch2*cRoll2*cYaw2+sPitch2*sRoll2*sYaw2;
  q[1] = sPitch2*cRoll2*cYaw2-cPitch2*sRoll2*sYaw2;
  q[2] = cPitch2*sRoll2*cYaw2+sPitch2*cRoll2*sYaw2;
  q[3] = cPitch2*cRoll2*sYaw2-sPitch2*sRoll2*cYaw2;
  
  // Normalise quaternion
  float recipNorm = invSqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;
}


void quaternion_add(float *q_srv,float *delta,float *q_dst)
{
  q_dst[0] = q_srv[0] + delta[0];
  q_dst[1] = q_srv[1] + delta[1];
  q_dst[2] = q_srv[2] + delta[2];
  q_dst[3] = q_srv[3] + delta[3];
}

void quaternion_sub(float *q_srv,float *delta,float *q_dst)
{
  q_dst[0] = q_srv[0] - delta[0];
  q_dst[1] = q_srv[1] - delta[1];
  q_dst[2] = q_srv[2] - delta[2];
  q_dst[3] = q_srv[3] - delta[3];
}

void quaternion_mul(float *q1_src, float *q2_src,float *q_dst)
{
  q_dst[0] = q1_src[0] * q2_src[0] - q1_src[1] * q2_src[1] - q1_src[2] * q2_src[2] - q1_src[3] * q2_src[3];
  q_dst[1] = q1_src[0] * q2_src[1] + q1_src[1] * q2_src[0] + q1_src[2] * q2_src[3] - q1_src[3] * q2_src[2];
  q_dst[2] = q1_src[0] * q2_src[2] - q1_src[1] * q2_src[3] + q1_src[2] * q2_src[0] + q1_src[3] * q2_src[1];
  q_dst[3] = q1_src[0] * q2_src[3] + q1_src[1] * q2_src[2] - q1_src[2] * q2_src[1] + q1_src[3] * q2_src[0];
}

void quaternion_conjugate(float *q_srv,float *q_dst)
{
  q_dst[0] =  q_srv[0];
  q_dst[1] = -q_srv[1];
  q_dst[2] = -q_srv[2];
  q_dst[3] = -q_srv[3];
}

void quaternion_scale(float *q_srv,float scale,float *q_dst)
{
  q_dst[0] =  q_srv[0]*scale;
  q_dst[1] =  q_srv[1]*scale;
  q_dst[2] =  q_srv[2]*scale;
  q_dst[3] =  q_srv[3]*scale;
}


void quaternion_normalize(float *q)
{
  float norm = FastSqrtI(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] *= norm;
  q[1] *= norm;
  q[2] *= norm;
  q[3] *= norm;
}

void quaternion_from_cb2n(float *q,float *cb2n)
{
  q[0]=0.5f*FastSqrt(1+cb2n[0]+cb2n[4]+cb2n[8]);
  q[1]=(cb2n[7]-cb2n[5])/(4*cb2n[0]);
  q[2]=(cb2n[2]-cb2n[6])/(4*cb2n[0]);
  q[3]=(cb2n[3]-cb2n[1])/(4*cb2n[0]);
  quaternion_normalize(q);	
}

void quaternion_to_euler(float *q,float *rpy)
{
  rpy[0]= asinf( 2.0f * q[0] * q[2] - 2.0f * q[1] * q[3]) * RAD2DEG;																					            // Roll
  rpy[1]= atan2f(2.0f * q[2] * q[3] + 2.0f * q[0] * q[1], -2.0f *q[1] *q[1] - 2.0f * q[2]* q[2] + 1.0f) * RAD2DEG;		    // Pitch	
  rpy[2]= FastAtan2(2.0f * q[1] * q[2] + 2.0f * q[0] * q[3], -2.0f * q[3] *q[3] - 2.0f * q[2] * q[2] + 1.0f) * RAD2DEG;		// Yaw
  
  if(rpy[2]<0) rpy[2]=rpy[2]+360;
  rpy[2]=constrain_float(rpy[2],0.0f,360);
  
}

void quaternion_rungekutta4(float *q, float *w, float dt, int normalize)
{
  float half = 0.5f;
  float two = 2.0f;
  float qw[4], k2[4], k3[4], k4[4];
  float tmpq[4], tmpk[4];
  //qw = q * w * half;
  quaternion_mul(q,w,qw);
  quaternion_scale(qw,half,qw);
  //k2 = (q + qw * dt * half) * w * half;
  quaternion_scale(qw,dt * half,tmpk);
  quaternion_add(tmpk, q, tmpk);
  quaternion_mul(tmpk, w,k2);
  quaternion_scale(k2,half,k2);
  //k3 = (q + k2 * dt * half) * w * half;
  quaternion_scale(k2,dt * half,tmpk);
  quaternion_add(tmpk, q, tmpk);
  quaternion_mul(tmpk, w,k3);
  quaternion_scale(k3, half, k3);
  //k4 = (q + k3 * dt) * w * half;
  quaternion_scale( k3, dt,tmpk);
  quaternion_add(tmpk, q, tmpk);
  quaternion_mul(tmpk, w, k4);
  quaternion_scale(k4, half, k4);
  //q += (qw + k2 * two + k3 * two + k4) * (dt / 6);
  quaternion_scale(k2, two,tmpk);
  quaternion_add(tmpk, qw, tmpq);
  quaternion_scale(k3, two,tmpk);
  quaternion_add(tmpq, tmpk,tmpq);
  quaternion_add(tmpq, k4, tmpq);
  quaternion_scale(tmpq, dt / 6.0f, tmpq);
  quaternion_add(q, tmpq, q);
  if (normalize){
    quaternion_normalize(q);
  }
}

void vector_from_bodyframe2earthframe(vector3f *bf,vector3f *ef,float *cb2n)
{
  ef->x=cb2n[0]*bf->x+cb2n[1]*bf->y+cb2n[2]*bf->z;
  ef->y=cb2n[3]*bf->x+cb2n[4]*bf->y+cb2n[5]*bf->z;
  ef->z=cb2n[6]*bf->x+cb2n[7]*bf->y+cb2n[8]*bf->z;
}

void vector_from_earthframe2bodyframe(vector3f *ef,vector3f *bf,float *cb2n)
{
  bf->x=cb2n[0]*ef->x+cb2n[3]*ef->y+cb2n[6]*ef->z;
  bf->y=cb2n[1]*ef->x+cb2n[4]*ef->y+cb2n[7]*ef->z;
  bf->z=cb2n[2]*ef->x+cb2n[5]*ef->y+cb2n[8]*ef->z;
}

