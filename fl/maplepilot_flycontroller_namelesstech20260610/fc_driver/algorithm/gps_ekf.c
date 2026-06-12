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
#include "schedule.h"
#include "wp_math.h"
#include "datatype.h"
#include "sensor.h"
#include "quaternion.h"
#include "filter.h"
#include "drv_opticalflow.h"
#include "drv_gps.h"
#include "sins.h"
#include "gps_ekf.h"


EKF_FILTER ins_ekf;

#define EKF_DT  0.1f
#define EKF_DT2 EKF_DT*EKF_DT
#define EKF_DT3 EKF_DT2*EKF_DT
#define EKF_DT4 EKF_DT2*EKF_DT2


#define EKF_ACC_P_NOISE_DEFAULT   50.0f//加速度计过程噪声   50cm/s^2
#define EKF_QP_INITIAL EKF_ACC_P_NOISE_DEFAULT*EKF_DT4   //位置过程噪声 0.005f
#define EKF_QV_INITIAL EKF_ACC_P_NOISE_DEFAULT*EKF_DT2   //速度过程噪声 0.1f
#define EKF_QA_INITIAL 0.001f*EKF_DT2   //加速度漂移过程噪声  1e-6f
//////////////////////////////////////////////////////////////

//0.5m 0.3m/s
#define EKF_GPS_P_NOISE_DEFAULT   50.0f   //位置观测噪声  5  10 20  50    10
#define EKF_GPS_V_NOISE_DEFAULT   30.0f   //速度观测噪声  10 20 50  100   1500
#define EKF_STATE_RECORD_PERIOD   10
#define EKF_GPS_P_DELAY_SYNC      220			//120	100
#define EKF_GPS_V_DELAY_SYNC      220			//120	100
//5 10 10 100 100
//备用
#define EKF_GPS_P_NOISE   30.0f     //GPS海拔观测噪声
#define EKF_BARO_P_NOISE  10.0f     //气压计观测噪声     
#define EKF_RNG_P_NOISE   1.0f      //对地测距观测噪声
#define GPS_MIN_HACC_M    1.5f	    //1.5m  
#define GPS_MIN_SACC_M    0.5f	    //0.5m/s
#define GPS_MAX_POS_ERR   10000.0f  //10000cm-100m
#define GPS_MAX_VEL_ERR   5000.0f   //5000cm/s-50m/s
#define GPS_MAX_ACC_BIAS  200.0f    //200cm/s^2-2m/s^2


#define EKF_PQ_INITIAL 0.1f    //协方差矩阵初始值
//协方差矩阵
const float P_INIT[INS_EKF_STATE_DIM * INS_EKF_STATE_DIM] = {
  EKF_PQ_INITIAL,0,0,0,0,0,
  0,EKF_PQ_INITIAL,0,0,0,0,  
  0,0,EKF_PQ_INITIAL,0,0,0,
  0,0,0,EKF_PQ_INITIAL,0,0,
  0,0,0,0,EKF_PQ_INITIAL,0,
  0,0,0,0,0,EKF_PQ_INITIAL,
};

//过程噪声矩阵
const float Q_INIT[INS_EKF_STATE_DIM * INS_EKF_STATE_DIM] = {
  EKF_QP_INITIAL,0,0,0,0,0,
  0,EKF_QP_INITIAL,0,0,0,0,  
  0,0,EKF_QV_INITIAL,0,0,0,
  0,0,0,EKF_QV_INITIAL,0,0,
  0,0,0,0,EKF_QA_INITIAL,0,
  0,0,0,0,0,EKF_QA_INITIAL,
};


//观测噪声矩阵
const float R_INIT[INS_EKF_MEASUREMENT_DIM * INS_EKF_MEASUREMENT_DIM] = {
  EKF_GPS_P_NOISE_DEFAULT,0,0,0,
  0,EKF_GPS_P_NOISE_DEFAULT,0,0,
  0,0,EKF_GPS_V_NOISE_DEFAULT,0,
  0,0,0,EKF_GPS_V_NOISE_DEFAULT,
};


const float F_INIT[INS_EKF_STATE_DIM * INS_EKF_STATE_DIM]={
  1,0,0,0,0,0,
  0,1,0,0,0,0,
  0,0,1,0,0,0,
  0,0,0,1,0,0,
  0,0,0,0,1,0,
  0,0,0,0,0,1,
};

const float H_INIT[INS_EKF_MEASUREMENT_DIM * INS_EKF_STATE_DIM] = {
  1,0,0,0,0,0,
  0,1,0,0,0,0,
  0,0,1,0,0,0,
  0,0,0,1,0,0,
};

void gps_ekf_init(void)
{
  for(uint16_t i=0;i<INS_EKF_STATE_DIM*INS_EKF_STATE_DIM;i++)
  {
    ins_ekf.P[i]=P_INIT[i];
    ins_ekf.Q[i]=Q_INIT[i];
    ins_ekf.F[i]=F_INIT[i];
  }
  for(uint16_t i=0;i<INS_EKF_MEASUREMENT_DIM*INS_EKF_MEASUREMENT_DIM;i++)
  {
    ins_ekf.R[i]=R_INIT[i];
  }
  for(uint16_t i=0;i<INS_EKF_MEASUREMENT_DIM*INS_EKF_STATE_DIM;i++)
  {
    ins_ekf.H[i]=H_INIT[i];
  }
  
#ifdef UPDATE_P_COMPLICATED	
  arm_mat_init_f32(&I_Matrix, EKF_STATE_DIM, EKF_STATE_DIM, I);
#endif	
  arm_mat_init_f32(&ins_ekf.P_Matrix, INS_EKF_STATE_DIM, INS_EKF_STATE_DIM, ins_ekf.P);
  arm_mat_init_f32(&ins_ekf.Q_Matrix, INS_EKF_STATE_DIM, INS_EKF_STATE_DIM, ins_ekf.Q);
  arm_mat_init_f32(&ins_ekf.R_Matrix, INS_EKF_MEASUREMENT_DIM, INS_EKF_MEASUREMENT_DIM, ins_ekf.R);
  arm_mat_init_f32(&ins_ekf.F_Matrix, INS_EKF_STATE_DIM, INS_EKF_STATE_DIM, ins_ekf.F);
  arm_mat_init_f32(&ins_ekf.H_Matrix, INS_EKF_MEASUREMENT_DIM, INS_EKF_STATE_DIM, ins_ekf.H);
  //
  arm_mat_init_f32(&ins_ekf.X_Matrix, INS_EKF_STATE_DIM, 1, ins_ekf.X);
  arm_mat_init_f32(&ins_ekf.KY_Matrix,INS_EKF_STATE_DIM, 1, ins_ekf.KY);
  arm_mat_init_f32(&ins_ekf.Y_Matrix, INS_EKF_MEASUREMENT_DIM, 1, ins_ekf.Y);
  arm_mat_init_f32(&ins_ekf.PX_Matrix, INS_EKF_STATE_DIM, INS_EKF_STATE_DIM, ins_ekf.PX);
  arm_mat_init_f32(&ins_ekf.PXX_Matrix, INS_EKF_STATE_DIM, INS_EKF_STATE_DIM, ins_ekf.PXX);
  arm_mat_init_f32(&ins_ekf.PXY_Matrix, INS_EKF_STATE_DIM, INS_EKF_MEASUREMENT_DIM, ins_ekf.PXY);
  arm_mat_init_f32(&ins_ekf.K_Matrix, INS_EKF_STATE_DIM, INS_EKF_MEASUREMENT_DIM, ins_ekf.K);
  arm_mat_init_f32(&ins_ekf.S_Matrix, INS_EKF_MEASUREMENT_DIM, INS_EKF_MEASUREMENT_DIM, ins_ekf.S);
  arm_mat_init_f32(&ins_ekf.SI_Matrix, INS_EKF_MEASUREMENT_DIM, INS_EKF_MEASUREMENT_DIM, ins_ekf.SI);
  
  arm_mat_init_f32(&ins_ekf.HT_Matrix, INS_EKF_STATE_DIM, INS_EKF_MEASUREMENT_DIM, ins_ekf.HT);
  
  ins_ekf.init=true;
}





void gps_ekf_update(sins *g_ins,float dt,uint8_t *obs_update_flag)
{
  if(ins_ekf.init!=true)	
  {
    //首次进入初始化相关变量
    gps_ekf_init();
    return;
  }
  float d2t=dt*dt;
  //系统驱动获取
  g_ins->acceleration[EAST] =ins.acceleration_initial[EAST] +g_ins->acce_bias[EAST];
  g_ins->acceleration[NORTH]=ins.acceleration_initial[NORTH]+g_ins->acce_bias[NORTH];
  //观测量获取	
  g_ins->obs_pos[EAST] =ins.gps_obs_pos_enu[EAST];
  g_ins->obs_pos[NORTH]=ins.gps_obs_pos_enu[NORTH];
  
  g_ins->obs_vel[EAST] =ins.gps_obs_vel_enu[EAST];
  g_ins->obs_vel[NORTH]=ins.gps_obs_vel_enu[NORTH];
  
  g_ins->position[EAST] +=g_ins->speed[EAST]*dt+0.5*g_ins->acceleration[EAST]*d2t;
  g_ins->position[NORTH]+=g_ins->speed[NORTH]*dt+0.5*g_ins->acceleration[NORTH]*d2t;
  
  g_ins->speed[EAST] +=g_ins->acceleration[EAST]*dt;
  g_ins->speed[NORTH]+=g_ins->acceleration[NORTH]*dt;
  
  if(*obs_update_flag==0)  return;
  *obs_update_flag=0;
  
  //根据GPS输出的位置、速度精度信息，动态调整观测噪声大小
  //float _Q_GPS[2]={0};
  float q_scale_pos=0,q_scale_speed=0;
  //位置定位精度小于GPS_MIN_HACC_M时，量程增益恒为1；大于GPS_MIN_HACC_M时，量程增益大于1，表示GPS位置噪声大
  q_scale_pos  =constrain_float(fmaxf(GPS_MIN_HACC_M,ins.gps_hacc_m  )/GPS_MIN_HACC_M,1.0f,10.0f);
  //速度定位精度小于GPS_MIN_SACC_M时，量程增益恒为1；大于GPS_MIN_SACC_M时，量程增益大于1，表示GPS速度噪声大
  q_scale_speed=constrain_float(fmaxf(GPS_MIN_SACC_M,ins.gps_sacc_mps)/GPS_MIN_SACC_M,1.0f,10.0f);
  ins_ekf.R[0] =R_INIT[0]*q_scale_pos;
  ins_ekf.R[5] =R_INIT[5]*q_scale_pos;
  ins_ekf.R[10]=R_INIT[10]*q_scale_speed;
  ins_ekf.R[15]=R_INIT[15]*q_scale_speed;
  
  //卡尔曼滤波器先验状态更新
  ins_ekf.X[0]=g_ins->position[EAST];
  ins_ekf.X[1]=g_ins->position[NORTH];
  ins_ekf.X[2]=g_ins->speed[EAST];
  ins_ekf.X[3]=g_ins->speed[NORTH];
  //观测矩阵更新
  ins_ekf.F[2] =EKF_DT;ins_ekf.F[4] =0.5f*EKF_DT*EKF_DT;
  ins_ekf.F[9] =EKF_DT;ins_ekf.F[11]=0.5f*EKF_DT*EKF_DT;
  ins_ekf.F[16]=EKF_DT;
  ins_ekf.F[23]=EKF_DT;
  //covariance time propagation
  //P = F*P*F' + Q; 
  //update process noise
  //先验协方差矩阵更新	
  arm_mat_mult_f32(&ins_ekf.F_Matrix, &ins_ekf.P_Matrix, &ins_ekf.PX_Matrix);
  arm_mat_trans_f32(&ins_ekf.F_Matrix,&ins_ekf.P_Matrix);
  arm_mat_mult_f32(&ins_ekf.PX_Matrix,&ins_ekf.P_Matrix,&ins_ekf.PXX_Matrix);
  arm_mat_add_f32(&ins_ekf.PXX_Matrix,  &ins_ekf.Q_Matrix,&ins_ekf.P_Matrix);	
  //measurement update
  //kalman gain calculation
  //K = P * H' / (R + H * P * H')	
  //计算卡尔曼增益
  arm_mat_trans_f32(&ins_ekf.H_Matrix,&ins_ekf.HT_Matrix);
  arm_mat_mult_f32(&ins_ekf.P_Matrix,&ins_ekf.HT_Matrix,&ins_ekf.PXY_Matrix);
  arm_mat_mult_f32(&ins_ekf.H_Matrix,&ins_ekf.PXY_Matrix,&ins_ekf.S_Matrix);
  
  arm_mat_add_f32(&ins_ekf.S_Matrix,&ins_ekf.R_Matrix,&ins_ekf.S_Matrix);
  arm_mat_inverse_f32(&ins_ekf.S_Matrix, &ins_ekf.SI_Matrix);
  arm_mat_mult_f32(&ins_ekf.PXY_Matrix,&ins_ekf.SI_Matrix,&ins_ekf.K_Matrix);
  
  //计算状态误差
  ins_ekf.Y[0] =g_ins->obs_pos[EAST] -g_ins->pos_backups[EAST][EKF_GPS_P_DELAY_SYNC/EKF_STATE_RECORD_PERIOD];
  ins_ekf.Y[1] =g_ins->obs_pos[NORTH]-g_ins->pos_backups[NORTH][EKF_GPS_P_DELAY_SYNC/EKF_STATE_RECORD_PERIOD];
  ins_ekf.Y[2] =g_ins->obs_vel[EAST] -g_ins->vel_backups[EAST][EKF_GPS_V_DELAY_SYNC/EKF_STATE_RECORD_PERIOD];
  ins_ekf.Y[3] =g_ins->obs_vel[NORTH]-g_ins->vel_backups[NORTH][EKF_GPS_V_DELAY_SYNC/EKF_STATE_RECORD_PERIOD];
  
  //状态误差限幅
  ins_ekf.Y[0] =constrain_float(ins_ekf.Y[0],-GPS_MAX_POS_ERR,GPS_MAX_POS_ERR);
  ins_ekf.Y[1] =constrain_float(ins_ekf.Y[1],-GPS_MAX_POS_ERR,GPS_MAX_POS_ERR);
  ins_ekf.Y[2] =constrain_float(ins_ekf.Y[2],-GPS_MAX_VEL_ERR,GPS_MAX_VEL_ERR);
  ins_ekf.Y[3] =constrain_float(ins_ekf.Y[3],-GPS_MAX_VEL_ERR,GPS_MAX_VEL_ERR);
  
  //后验状态更新
  //Update State Vector 
  arm_mat_mult_f32(&ins_ekf.K_Matrix,&ins_ekf.Y_Matrix,&ins_ekf.KY_Matrix);
  arm_mat_add_f32(&ins_ekf.X_Matrix,&ins_ekf.KY_Matrix,&ins_ekf.X_Matrix);
  //covariance measurement update
  //P = P - K * H * P
  //后验协方差矩阵更新	
  arm_mat_mult_f32(&ins_ekf.K_Matrix,&ins_ekf.H_Matrix,&ins_ekf.PX_Matrix);
  arm_mat_mult_f32(&ins_ekf.PX_Matrix,&ins_ekf.P_Matrix,&ins_ekf.PXX_Matrix);
  arm_mat_sub_f32(&ins_ekf.P_Matrix,&ins_ekf.PXX_Matrix,&ins_ekf.P_Matrix);		
  //惯导加速度bias限幅
  ins_ekf.X[4]=constrain_float(ins_ekf.X[4],-GPS_MAX_ACC_BIAS,GPS_MAX_ACC_BIAS);
  ins_ekf.X[5]=constrain_float(ins_ekf.X[5],-GPS_MAX_ACC_BIAS,GPS_MAX_ACC_BIAS);
  
  g_ins->position[EAST]  =ins_ekf.X[0];
  g_ins->position[NORTH] =ins_ekf.X[1];
  g_ins->speed[EAST]     =ins_ekf.X[2];
  g_ins->speed[NORTH]    =ins_ekf.X[3];
  g_ins->acce_bias[EAST] =ins_ekf.X[4];
  g_ins->acce_bias[NORTH]=ins_ekf.X[5];
}
