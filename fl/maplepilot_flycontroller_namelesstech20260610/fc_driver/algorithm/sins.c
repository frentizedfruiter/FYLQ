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
#include "nclink.h"
#include "gps_ekf.h"
#include "attitude_ctrl.h"
#include "maple_config.h"
#include "sins.h"


#define DYNAMIC_PROPERTY_DN_ACCEL_G   -4.0f //-4.0f
#define DYNAMIC_PROPERTY_UP_ACCEL_G    8.0f // 8.0f
#define ACC_BIAS_MAX 100.0f

#define KALMAN_DT 0.05f
#define ACC_NOISE_DEFAULT  1.0f
float ACC_BIAS_P=1e-4f;//0.05f
float q_init[3]={ACC_NOISE_DEFAULT*KALMAN_DT*KALMAN_DT*0.5f,
ACC_NOISE_DEFAULT*KALMAN_DT,
0};                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
uint16_t sync_cnt[6]={5 ,10,5 ,5 ,5 ,5 };
float rp_init[6]    ={10,10,20,10,10,10};
float rv_init[6]    ={0 ,0 ,0 ,0 ,0 ,0 };



static float p_init[3][3]={
  0.81,0.68,0,  //4.95,2.83,//0.18,0.1,
  0.68,1.25,0,  //2.83,3.49 //0.1,0.18
  0   ,	0  ,0,
};
#define history_record_period 2//2*5=10ms
systime alt_obs_delta;
kalman_filter alt_kf;
void kalman_filter_init(kalman_filter *kf,float *p,float qp,float qv,float qb,float *rp,float *rv)
{
  for(uint8_t i=0;i<3;i++)
    for(uint8_t j=0;j<3;j++)
    {
      kf->p[i][j]=kf->p[i][j];
      kf->k[i][j]=0;
    }
  kf->qp=qp;
  kf->qv=qv;
  kf->qb=qb;
  for(uint8_t i=0;i<6;i++)
  {
    kf->rp[i]=*(rp+i);
    kf->rv[i]=*(rv+i);
  }
  kf->init=1;
}



void altitude_kalman_filter(kalman_filter *kf,sins *_ins,float acc,float alt_obs,float alt_vel_obs,float dt)
{
#define P   kf->p		//协方差矩阵
#define QP  kf->qp  //位置过程噪声
#define QV  kf->qv	//速度过程噪声
  //#define R   kf->rp	//位置观测噪声 
#define K   kf->k	  //卡尔曼增益
#define ERR kf->err //观测误差
#define CP  kf->cp  //位置修正量
#define CV  kf->cv	//速度修正量
#define CB  kf->cb	//零偏修正量
  
  if(kf->init==0)	
  {
    //首次进入,初始化滤波器
    kalman_filter_init(kf,
                       &p_init[0][0],//协方差矩阵初值
                       q_init[0],   //位置过程噪声
                       q_init[1],   //速度过程噪声
                       q_init[2],   //零偏过程噪声
                       rp_init,			//位置观测噪声
                       rv_init);    //速度观测噪声
    return;
  }
  
  static uint32_t _cnt=0;
  //惯导历史值保存
  _cnt++;
  if(_cnt%history_record_period==0)
  {
    for(uint16_t i=INS_NUM-1;i>0;i--)
    {
      _ins->pos_backups[_UP][i] =_ins->pos_backups[_UP][i-1];
      _ins->vel_backups[_UP][i] =_ins->vel_backups[_UP][i-1];
      _ins->acce_backups[_UP][i]=_ins->acce_backups[_UP][i-1];
    }	
  }
  _ins->pos_backups[_UP][0]=_ins->position[_UP];
  _ins->vel_backups[_UP][0]=_ins->speed[_UP];
  _ins->acce_backups[_UP][0]=_ins->acceleration[_UP];
  //根据多旋翼动力特性，对惯导加速度进行约束
  acc=constrain_float(acc,DYNAMIC_PROPERTY_DN_ACCEL_G*GRAVITY_MSS*100,DYNAMIC_PROPERTY_UP_ACCEL_G*GRAVITY_MSS*100);//-4G~8G
  
  //1、系统先验状态更新
  _ins->acceleration[_UP]=acc+_ins->acce_bias[_UP];
  _ins->position[_UP]+=_ins->speed[_UP]*dt+0.5f*dt*dt*_ins->acceleration[_UP];
  _ins->speed[_UP]+=dt*_ins->acceleration[_UP];
  
  if(_ins->alt_obs_update==1)
  {
    _ins->alt_obs_update=0;
    get_systime(&alt_obs_delta);
    float _dt=alt_obs_delta.period*0.001f;
    _dt=constrain_float(_dt,0.005f,1.0f);
    /*********************************************************************************************************/
    static uint32_t obs_cnt=0;
    obs_cnt++;
    if(obs_cnt%1==0)
    {
      for(uint16_t i=INS_NUM-1;i>0;i--)
      {
        _ins->obs_pos_backups[_UP][i]=_ins->obs_pos_backups[_UP][i-1];
        _ins->obs_vel_backups[_UP][i]=_ins->obs_vel_backups[_UP][i-1];
      }	
    }
    _ins->obs_pos_backups[_UP][0]=alt_obs;
    _ins->obs_vel_backups[_UP][0]=alt_vel_obs;			
    /*********************************************************************************************************/
    _ins->alt_obs_sync_ms=_ins->alt_obs_update_period;
    //状态误差*
    ERR[0]=_ins->obs_pos_backups[_UP][0]-_ins->pos_backups[_UP][sync_cnt[ins.alt_obs_type]];//5、10
    ERR[0]=constrain_float(ERR[0],-2000,2000);//±20m
    ERR[1]=0;//不存在速度观测量
    
    float pt[3][3]={0};//先验协方差		
    //2、先验协方差
    float ct =P[0][1]+P[1][1]*_dt;
    pt[0][0]=P[0][0]+P[1][0]*_dt+ct*_dt+QP;
    pt[0][1]=ct;
    pt[1][0]=P[1][0]+P[1][1]*_dt;
    pt[1][1]=P[1][1]+QV;
    
    //3、计算卡尔曼增益
    float cz=1/(pt[0][0]+kf->rp[ins.alt_obs_type]);
    K[0][0]=pt[0][0]*cz;//稳态约为0.069
    K[0][1]=0;
    K[1][0]=pt[1][0]*cz;//稳态约为0.096
    K[1][1]=0;
    
    CP=K[0][0]*ERR[0]+K[0][1]*ERR[1];//位置修正量
    CV=K[1][0]*ERR[0]+K[1][1]*ERR[1];//速度修正量
    CB=K[2][0]*ERR[0]+K[2][1]*ERR[1];//加速度计偏移修正量
    
    //4、系统后验状态更新
    _ins->position[_UP]+=CP;
    _ins->speed[_UP]	  +=CV;
    
    _ins->acce_bias[_UP]+=ACC_BIAS_P*ERR[0]*_dt;
    _ins->acce_bias[_UP]=constrain_float(_ins->acce_bias[_UP],-ACC_BIAS_MAX,ACC_BIAS_MAX);
    
    //5、更新状态协方差矩阵
    float kt=(1-K[0][0]);
    P[0][0]=kt*pt[0][0];
    P[0][1]=kt*pt[0][1];
    P[1][0]=pt[1][0]-K[1][0]*pt[0][0];
    P[1][1]=pt[1][1]-K[1][0]*pt[0][1];
  }	
#undef P
#undef QP
#undef QV
  //#undef R
#undef K
#undef ERR
#undef CP
#undef CV
#undef CB	
}



#define _KALMAN_DT 0.05f
#define history_record_period 2//2*5=10ms
uint16_t fusion_sync_cnt=5;//10  5
float _ACC_BIAS_P=0.2f;//1e-4f 0.05f
float qk_init[3]={ACC_NOISE_DEFAULT*_KALMAN_DT*_KALMAN_DT*0.5f,
ACC_NOISE_DEFAULT*_KALMAN_DT,
0};
float rk_init[2]={0.15,0};//0.1-20241002,0.5
static float pk_init[3][3]={
  0.81,0.68,0,  //4.95,2.83,//0.18,0.1,
  0.68,1.25,0,  //2.83,3.49 //0.1,0.18
  0   ,	0  ,0,
};

void slam_kalman_filter_init(_kalman_filter *kf,float *p,float qp,float qv,float qb,float rp,float rv)
{
  for(uint8_t i=0;i<3;i++)
    for(uint8_t j=0;j<3;j++)
    {
      kf->p[i][j]=kf->p[i][j];
      kf->k[i][j]=0;
    }
  kf->qp=qp;
  kf->qv=qv;
  kf->qb=qb;
  kf->rp=rp;
  kf->rv=rv;
  kf->init=1;
}


void slam_altitude_kalman_filter(_kalman_filter *kf,sins_lite *_ins,float dt)
{
  //if(current_state.loam_update_flag==0) return ;
  if(kf->init==0)	
  {
    //首次进入,初始化滤波器
    slam_kalman_filter_init(kf,
                            &pk_init[0][0],
                            qk_init[0],
                            qk_init[1],
                            qk_init[2],
                            rk_init[0],
                            rk_init[1]);
    return;
  }
#define P   kf->p
#define QP  kf->qp
#define QV  kf->qv
#define R   kf->rp	
#define K   kf->k	
#define ERR kf->err
#define CP  kf->cp
#define CV  kf->cv	
#define CB  kf->cb	
  //根据多旋翼动力特性，对惯导加速度进行约束
  vector2f acc;
  acc.x= ins.acceleration_initial[_EAST];
  acc.y= ins.acceleration_initial[_NORTH];
  
  acc.x=constrain_float(acc.x,DYNAMIC_PROPERTY_DN_ACCEL_G*GRAVITY_MSS*100,DYNAMIC_PROPERTY_UP_ACCEL_G*GRAVITY_MSS*100);//-4G~8G
  acc.y=constrain_float(acc.y,DYNAMIC_PROPERTY_DN_ACCEL_G*GRAVITY_MSS*100,DYNAMIC_PROPERTY_UP_ACCEL_G*GRAVITY_MSS*100);//-4G~8G
  //1、系统先验状态更新
  _ins->accel_cmpss[_EAST]=acc.x+_ins->acce_bias[_EAST];
  _ins->position[_EAST]+=_ins->speed[_EAST]*dt+0.5f*dt*dt*_ins->accel_cmpss[_EAST];
  _ins->speed[_EAST]+=dt*_ins->accel_cmpss[_EAST];
  
  _ins->accel_cmpss[_NORTH]=acc.y+_ins->acce_bias[_NORTH];
  _ins->position[_NORTH]+=_ins->speed[_NORTH]*dt+0.5f*dt*dt*_ins->accel_cmpss[_NORTH];
  _ins->speed[_NORTH]+=dt*_ins->accel_cmpss[_NORTH];
  
  if(current_state.update_flag==1)//存在数据SLAM更新时，50ms一次
  { 
    current_state.valid=1;
    current_state.update_flag=0; 
    
    float _dt=_KALMAN_DT;//0.1f
    //状态误差*
    ERR[0][0]=current_state.position_x-_ins->pos_backups[_EAST][fusion_sync_cnt];//5、10
    ERR[0][0]=constrain_float(ERR[0][0],-2000,2000);//±20m
    ERR[0][1]=0;//不存在速度观测量
    
    ERR[1][0]=current_state.position_y-_ins->pos_backups[_NORTH][fusion_sync_cnt];//5、10
    ERR[1][0]=constrain_float(ERR[1][0],-2000,2000);//±20m
    ERR[1][1]=0;//不存在速度观测量		
    
    float pt[3][3]={0};//先验协方差		
    //2、先验协方差
    float ct =P[0][1]+P[1][1]*_dt;
    pt[0][0]=P[0][0]+P[1][0]*_dt+ct*_dt+QP;
    pt[0][1]=ct;
    pt[1][0]=P[1][0]+P[1][1]*_dt;
    pt[1][1]=P[1][1]+QV;
    
    //3、计算卡尔曼增益
    float cz=1/(pt[0][0]+R);
    K[0][0]=pt[0][0]*cz;//稳态约为0.069
    K[0][1]=0;
    K[1][0]=pt[1][0]*cz;//稳态约为0.096
    K[1][1]=0;
    
    CP[0]=K[0][0]*ERR[0][0]+K[0][1]*ERR[0][1];//位置修正量
    CV[0]=K[1][0]*ERR[0][0]+K[1][1]*ERR[0][1];//速度修正量
    CB[0]=K[2][0]*ERR[0][0]+K[2][1]*ERR[0][1];//加速度计偏移修正量
    
    CP[1]=K[0][0]*ERR[1][0]+K[0][1]*ERR[1][1];//位置修正量
    CV[1]=K[1][0]*ERR[1][0]+K[1][1]*ERR[1][1];//速度修正量
    CB[1]=K[2][0]*ERR[1][0]+K[2][1]*ERR[1][1];//加速度计偏移修正量		
    //4、系统后验状态更新
    _ins->position[_EAST] +=CP[0];
    _ins->speed[_EAST]	  +=CV[0];
    _ins->position[_NORTH]+=CP[1];
    _ins->speed[_NORTH]	  +=CV[1];
    
    _ins->acce_bias[_EAST] +=_ACC_BIAS_P*ERR[0][0]*_dt;
    _ins->acce_bias[_EAST]  =constrain_float(_ins->acce_bias[_EAST],-ACC_BIAS_MAX,ACC_BIAS_MAX);
    _ins->acce_bias[_NORTH]+=_ACC_BIAS_P*ERR[1][0]*_dt;
    _ins->acce_bias[_NORTH] =constrain_float(_ins->acce_bias[_NORTH],-ACC_BIAS_MAX,ACC_BIAS_MAX);		
    //5、更新状态协方差矩阵
    float kt=(1-K[0][0]);
    P[0][0]=kt*pt[0][0];
    P[0][1]=kt*pt[0][1];
    P[1][0]=pt[1][0]-K[1][0]*pt[0][0];
    P[1][1]=pt[1][1]-K[1][0]*pt[0][1];
  }	
#undef P
#undef QP
#undef QV
#undef R
#undef K
#undef ERR
#undef CP
#undef CV
#undef CB
  
  static uint32_t _cnt=0;//惯导历史值保存
  _cnt++;
  if(_cnt%history_record_period==0)
  {
    for(uint16_t i=INS_NUM-1;i>0;i--)
    {
      _ins->pos_backups[_EAST][i] =_ins->pos_backups[_EAST][i-1];
      _ins->vel_backups[_EAST][i] =_ins->vel_backups[_EAST][i-1];
      _ins->pos_backups[_NORTH][i] =_ins->pos_backups[_NORTH][i-1];
      _ins->vel_backups[_NORTH][i] =_ins->vel_backups[_NORTH][i-1];
    }	
  }
  _ins->pos_backups[_EAST][0]=_ins->position[_EAST];
  _ins->vel_backups[_EAST][0]=_ins->speed[_EAST];
  _ins->pos_backups[_NORTH][0]=_ins->position[_NORTH];
  _ins->vel_backups[_NORTH][0]=_ins->speed[_NORTH];
  //将EN方向状态旋转的RP上
  
  from_vio_to_body_frame(_ins->position[_EAST],
                         _ins->position[_NORTH],
                         &ins.opt.position_ctrl.x,
                         &ins.opt.position_ctrl.y);
  
  from_vio_to_body_frame(_ins->speed[_EAST],
                         _ins->speed[_NORTH],
                         &ins.opt.speed_ctrl.x,
                         &ins.opt.speed_ctrl.y);
}



/*****************算法技术博客讲解***************************************************
1、四旋翼定高篇之惯导加速度+速度+位置三阶互补融合方案:
http://blog.csdn.net/u011992534/article/details/61924200
2、四旋翼惯导融合之观测传感器滞后问题汇总与巴特沃斯低通滤波器设计
（气压计MS5611、GPS模块M8N、超声波、PX4FLOW等）:
http://blog.csdn.net/u011992534/article/details/73743955
3、从APM源码分析GPS、气压计惯导融合
http://blog.csdn.net/u011992534/article/details/78257684
**********************************************************************************/
float pos_z_correction,acc_z_correction;
/****气压计三阶互补滤波方案——参考开源飞控APM****/
#define TIME_CONTANST_ZER    2.0f
#define K3_ACC_ZER 	        (1.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))//1
#define K2_VEL_ZER	        (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))//3
#define K1_POS_ZER          (3.0f / TIME_CONTANST_ZER)//3
float pos_z_fusion,vel_z_fusion,acc_z_fusion;
static float pos_base_z;
static float pos_z_bkp[40];
uint16_t baro_sync_cnt=5;
void strapdown_ins_height(void)
{
  float err=0,vel_delta=0;
  float dt=0.005f;
  //由观测量（气压计）得到状态误差
  err=constrain_float(flymaple.baro_height-pos_z_bkp[baro_sync_cnt],-500,500);//气压计(超声波)与SINS估计量的差，单位cm
  //三路积分反馈量修正惯导
  acc_z_correction +=err* K3_ACC_ZER*dt ;//加速度矫正量
  vel_z_fusion     +=err* K2_VEL_ZER*dt ;//速度矫正量
  pos_z_correction +=err* K1_POS_ZER*dt ;//位置矫正量
  
  acc_z_fusion=ins.acceleration_initial[_UP]+acc_z_correction;//加速度计矫正后更新
  vel_delta=acc_z_fusion*dt;//速度增量矫正后更新，用于更新位置
  
  pos_base_z+=(vel_z_fusion+0.5f*vel_delta)*dt;//原始位置更新
  pos_z_fusion=pos_base_z+pos_z_correction;//位置矫正后更新
  vel_z_fusion+=vel_delta;//速度更新 
  
  static uint16_t cnt=0; cnt++;
  if(cnt>=2)//10ms滑动一次
  {
    for(uint16_t i=40-1;i>0;i--)	pos_z_bkp[i]=pos_z_bkp[i-1];
    cnt=0;
  }
  pos_z_bkp[0]=pos_z_fusion;	
}










vector3s correct[2];
float K_ACC_OPT=0.0f,K_VEL_OPT=3.0f,K_POS_OPT=0.0f;//1 3 5
void third_order_complementarity(float ground_height,vector2f acc,opticalflow *opt)
{
  float obs_err[2];
  vector2f speed_delta={0};
  float user_height=constrain_float(ground_height,5,10000);//5cm-100m
  if(opt->valid==1&&opt->is_okay==1)//数据20ms更新一次
  {
    //观测速度直接积分后得到观测位置
    ins.opt.speed_obs.x=opt->flow_correct.x*user_height;
    ins.opt.speed_obs.y=opt->flow_correct.y*user_height;
    ins.opt.position_obs.x+=ins.opt.speed_obs.x*opt->ms*0.001f;
    ins.opt.position_obs.y+=ins.opt.speed_obs.y*opt->ms*0.001f;
    //计算位置、速度观测误差
    ins.opt.position_err.x=ins.opt.position_obs.x-ins.opt.position.x;
    ins.opt.position_err.y=ins.opt.position_obs.y-ins.opt.position.y;
    ins.opt.speed_err.x=ins.opt.speed_obs.x-ins.opt.speed.x;
    ins.opt.speed_err.y=ins.opt.speed_obs.y-ins.opt.speed.y;
    ins.opt.speed_err.x=constrain_float(ins.opt.speed_err.x,-200,200);
    ins.opt.speed_err.y=constrain_float(ins.opt.speed_err.y,-200,200);
    
    opt->is_okay=0;
  }
  obs_err[0]=ins.opt.speed_err.x;//ins.opt.position_err.x;
  obs_err[1]=ins.opt.speed_err.y;//ins.opt.position_err.y;
  //三路积分反馈量修正惯导
  correct[0].acc +=obs_err[0]* K_ACC_OPT*0.005f;//加速度矫正量
  correct[1].acc +=obs_err[1]* K_ACC_OPT*0.005f;//加速度矫正
  correct[0].acc	=constrain_float(correct[0].acc,-50,50);
  correct[1].acc	=constrain_float(correct[1].acc,-50,50);
  correct[0].vel  =ins.opt.speed_err.x*K_VEL_OPT*0.005f;//速度矫正量
  correct[1].vel  =ins.opt.speed_err.y*K_VEL_OPT*0.005f;//速度矫正量
  correct[0].pos  =obs_err[0]* K_POS_OPT*0.005f;//位置矫正量	
  correct[1].pos  =obs_err[1]* K_POS_OPT*0.005f;//位置矫正量
  
  ins.opt.acceleration.x= acc.x;//惯导加速度沿载体横滚,机头右侧为正
  ins.opt.acceleration.y= acc.y;//惯导加速度沿载体机头,机头前向为正
  ins.opt.acceleration.x+=correct[0].acc;
  ins.opt.acceleration.y+=correct[1].acc;
  //计算速度增量//
  float dt=min_ctrl_dt;
  speed_delta.x=ins.opt.acceleration.x*dt;
  speed_delta.y=ins.opt.acceleration.y*dt; 
  //位置更新
  ins.opt.position.x+=ins.opt.speed.x*dt+0.5f*speed_delta.x*dt+correct[0].pos;
  ins.opt.position.y+=ins.opt.speed.y*dt+0.5f*speed_delta.y*dt+correct[1].pos;
  //速度更新
  ins.opt.speed.x+=speed_delta.x+correct[0].vel;
  ins.opt.speed.y+=speed_delta.y+correct[1].vel;
  
  //用于位置、速度控制
  ins.opt.position_ctrl.x=ins.opt.position.x;
  ins.opt.position_ctrl.y=ins.opt.position.y;
  ins.opt.speed_ctrl.x=ins.opt.speed.x;
  ins.opt.speed_ctrl.y=ins.opt.speed.y;
}
/*************************************************************************************************************/

void from_vio_to_body_frame(float map_x,float map_y,float *bod_x,float *bod_y)
{
  float _cos=flymaple.cos_rpy[_YAW];
  float _sin=flymaple.sin_rpy[_YAW];
  *bod_x=  map_x*_cos+map_y*_sin;
  *bod_y= -map_x*_sin+map_y*_cos;
}




void from_body_to_nav_frame(float bod_x,float bod_y,float *map_x,float *map_y)
{
  float _cos=flymaple.cos_rpy[_YAW];
  float _sin=flymaple.sin_rpy[_YAW];
  *map_x= bod_x*_cos-bod_y*_sin;
  *map_y= bod_x*_sin+bod_y*_cos;
}


//30  300  100  10
#define K_ACC_RPLIDAR 30.0f
#define K_VEL_RPLIDAR 400.0f//range(300,1000)500
#define K_POS_RPLIDAR 100.0f//range(50,150)100
#define SYNC_CNT_RPLIDAR 5//10  5
//5  120  30 5
#define K_ACC_T265 5.0f
#define K_VEL_T265 120.0f
#define K_POS_T265 30.0f
#define SYNC_CNT_T265 5
float slam_fusion_param[3]={0};
uint16_t slam_sync_cnt=0;

void vio_slam_ins_fusion(void)
{
  vector2f acc={0},speed_delta={0};
  float obs_err[2];
  switch(current_state.slam_sensor)
  {
  case NO_SLAM:
    {
      return ;
    }
  case RPLIDAR_SLAM:
    {
      slam_fusion_param[0]=K_ACC_RPLIDAR;
      slam_fusion_param[1]=K_VEL_RPLIDAR;
      slam_fusion_param[2]=K_POS_RPLIDAR;	
      slam_sync_cnt=SYNC_CNT_RPLIDAR;			
    };
    break;
  case T265_SLAM:
    {
      slam_fusion_param[0]=K_ACC_T265;
      slam_fusion_param[1]=K_VEL_T265;
      slam_fusion_param[2]=K_POS_T265;	
      slam_sync_cnt=SYNC_CNT_T265;				
    };
    break;
  default:	return;		
  }
  
  if(current_state.update_flag==1)//存在数据SLAM更新时，50ms一次
  { 
    current_state.valid=1;
    current_state.update_flag=0; 
    
    ins.opt.position_obs.x= current_state.position_x;
    ins.opt.position_obs.y= current_state.position_y;
    
    ins.opt.position_err.x=ins.opt.position_obs.x-ins.opt.pos_backups[slam_sync_cnt].x;
    ins.opt.position_err.y=ins.opt.position_obs.y-ins.opt.pos_backups[slam_sync_cnt].y;
  }
  else
  {
    ins.opt.position_err.x=0;
    ins.opt.position_err.y=0;
  }
  obs_err[0]=ins.opt.position_err.x;
  obs_err[1]=ins.opt.position_err.y;
  //三路反馈量修正惯导
  correct[0].acc +=obs_err[0]* slam_fusion_param[0]*0.005f;//加速度矫正量
  correct[1].acc +=obs_err[1]* slam_fusion_param[0]*0.005f;//加速度矫正
  correct[0].acc=constrain_float(correct[0].acc,-50,50);
  correct[1].acc=constrain_float(correct[1].acc,-50,50);
  correct[0].vel  =obs_err[0]* slam_fusion_param[1]*0.005f;//速度矫正量
  correct[1].vel  =obs_err[1]* slam_fusion_param[1]*0.005f;//速度矫正量
  correct[0].pos  =obs_err[0]* slam_fusion_param[2]*0.005f;//位置矫正量	
  correct[1].pos  =obs_err[1]* slam_fusion_param[2]*0.005f;//位置矫正量
  
  acc.x= ins.acceleration_initial[EAST]; //惯导加速度沿载体横滚,机头左侧为正
  acc.y= ins.acceleration_initial[NORTH];//惯导加速度沿载体机头,机头前向为正
  
  ins.opt.acceleration.x= acc.x;//惯导加速度沿载体横滚,机头右侧为正
  ins.opt.acceleration.y= acc.y;//惯导加速度沿载体机头,机头前向为正
  ins.opt.acceleration.x+=correct[0].acc;//_EAST
  ins.opt.acceleration.y+=correct[1].acc;//_NORTH
  
  float dt=min_ctrl_dt;
  //计算速度增量//
  speed_delta.x=ins.opt.acceleration.x * dt;
  speed_delta.y=ins.opt.acceleration.y * dt; 
  
  //位置更新
  ins.opt.position.x +=ins.opt.speed.x * dt+0.5f*speed_delta.x*dt+correct[0].pos;
  ins.opt.position.y +=ins.opt.speed.y * dt+0.5f*speed_delta.y*dt+correct[1].pos;
  //速度更新
  ins.opt.speed.x  +=speed_delta.x+correct[0].vel;
  ins.opt.speed.y  +=speed_delta.y+correct[1].vel;	
  
  for(uint16_t i=INS_NUM-1;i>0;i--)
  {
    ins.opt.pos_backups[i].x=ins.opt.pos_backups[i-1].x;
    ins.opt.pos_backups[i].y=ins.opt.pos_backups[i-1].y;	
    ins.opt.vel_backups[i].x=ins.opt.vel_backups[i-1].x;
    ins.opt.vel_backups[i].y=ins.opt.vel_backups[i-1].y; 		
  }   
  ins.opt.pos_backups[0].x=ins.opt.position.x;
  ins.opt.pos_backups[0].y=ins.opt.position.y; 
  ins.opt.vel_backups[0].x=ins.opt.speed.x;
  ins.opt.vel_backups[0].y=ins.opt.speed.y;
  
  //用于位置、速度控制
  from_vio_to_body_frame(ins.opt.position.x,
                         ins.opt.position.y,
                         &ins.opt.position_ctrl.x,
                         &ins.opt.position_ctrl.y);
  
  from_vio_to_body_frame(ins.opt.speed.x,
                         ins.opt.speed.y,
                         &ins.opt.speed_ctrl.x,
                         &ins.opt.speed_ctrl.y);
}



void indoor_position_fusion(void)
{
  switch(maplepilot.indoor_position_sensor)
  {
  case 	RPISLAM:
    {
      vio_slam_ins_fusion();
    }
    break;		
  case 	OPTICALFLOW0:
  default:
    {
      third_order_complementarity(ins.position_z,ins.horizontal_acceleration,&opt_data);
    }
  }
}



/*************************************************************************************************************/
void strapdown_ins_reset(sins *ins,uint8_t chl,float target_pos,float target_vel)
{
  int16_t _cnt=0;
  ins->position[chl]=target_pos;//位置重置
  ins->speed[chl]=target_vel;	 //速度重置
  //Ins->acceleration[chl]=0.0f;  //加速度清零
  //Ins->acceleration_initial[chl]=0.0f;
  ins->acce_bias[chl]=0.0f;
  for(_cnt=INS_NUM-1;_cnt>0;_cnt--)//历史位置值，全部装载为当前观测值
  {
    ins->pos_backups[chl][_cnt]=target_pos;
  }
  ins->pos_backups[chl][0]=target_pos;
  for(_cnt=INS_NUM-1;_cnt>0;_cnt--)//历史速度值，全部装载为当前观测值
  {
    ins->vel_backups[chl][_cnt]=target_vel;
  }
  ins->vel_backups[chl][0]=target_vel;
}

Location gps_home_point;   //初始定位成功点信息
Location gps_present_point;//当前位置点信息
static uint16_t gps_home_cnt=0;
void gps_homepoint_init(void)
{
  float hacc_m=gps_data.gps.hacc*0.001f;	
  if(ins.gps_home_fixed_flag==1)  return;//home点未刷新时，才进入初始home点设置
  if(ins.gps_update_flag==0)      return;//GPS数据刷新时，才判断GPS数据状态
  //100ms更新一次
  ins.gps_update_flag=0;//复位GPS更新状态
  if(gps_data.gps.fixtype>=0x03// 3D-Fix
     &&gps_data.gps.numsv>=9		 //星数大于等于9
       &&hacc_m<=1.5f)	       //水平位置估计精度小于1.5m
  {
    if(gps_home_cnt<=50) gps_home_cnt++;//刷新10hz，连续5S满足
  }
  else
  {
    gps_home_cnt/=2;
  }
  
  if(gps_home_cnt==50)//全程只设置一次
  {
    //设置飞行器的返航点
    ins.gps_home_fixed_flag=1;
    ins.gps_home_lat=gps_home_point.lat=gps_data.gps.lat;
    ins.gps_home_lng=gps_home_point.lng=gps_data.gps.lon;
    ins.gps_home_alt=gps_data.gps.hmsl;	
    ins.gps_home_alt_cm=gps_data.gps.hmsl/10.0f;//cm
    
    ins.magnetic_declination=get_declination(0.0000001f*gps_data.gps.lat,0.0000001f*gps_data.gps.lon);//获取当地磁偏角
    
    strapdown_ins_reset(&ins,EAST,ins.gps_obs_pos_enu[EAST],
                        ins.gps_obs_vel_enu[EAST]);  //复位惯导融合
    strapdown_ins_reset(&ins,NORTH,ins.gps_obs_pos_enu[NORTH],
                        ins.gps_obs_vel_enu[NORTH]);//复位惯导融合
  }
}


float get_earth_declination(void)
{
  float tmp_mag_dec=0;
  if(ins.gps_home_fixed_flag==0)  tmp_mag_dec=ins.magnetic_declination;//home点未刷新时,输出值恒为0
  return tmp_mag_dec;
}


void gps_prase_enu(void)
{
  vector2f location_delta={0,0};
  gps_present_point.lng=gps_data.gps.lon;//更新当前经纬度
  gps_present_point.lat=gps_data.gps.lat;
  location_delta=location_diff(gps_home_point,gps_present_point);//根据当前GPS定位信息与Home点位置信息计算正北、正东方向位置偏移
  /***********************************************************************************
  明确下导航系方向，这里正北、正东为正方向:
  沿着正东，经度增加,当无人机相对home点，往正东向移动时，此时GPS_Present.lng>GPS_Home.lng，得到的location_delta.x大于0;
  沿着正北，纬度增加,当无人机相对home点，往正北向移动时，此时GPS_Present.lat>GPS_Home.lat，得到的location_delta.y大于0;
  ******************************************************************************/
  location_delta.x*=100.0f;//沿地理坐标系，正东方向位置偏移,单位为CM
  location_delta.y*=100.0f;//沿地理坐标系，正北方向位置偏移,单位为CM
  ins.gps_obs_pos_enu[EAST] =	location_delta.x;//地理系下相对Home点正东位置偏移，单位为CM
  ins.gps_obs_pos_enu[NORTH]= location_delta.y;//地理系下相对Home点正北位置偏移，单位为CM
  //将无人机在导航坐标系下的沿着正东、正北方向的位置偏移旋转到当前航向的位置偏移:机头(俯仰)+横滚
  ins.gps_obs_pos_body[_RIGHT]  = location_delta.x*flymaple.cos_rpy[_YAW]+location_delta.y*flymaple.sin_rpy[_YAW];//横滚正向位置偏移  X轴正向
  ins.gps_obs_pos_body[_FORWARD]=-location_delta.x*flymaple.sin_rpy[_YAW]+location_delta.y*flymaple.cos_rpy[_YAW];//机头正向位置偏移  Y轴正向
  
  ins.gps_lat=gps_data.gps.lat;
  ins.gps_lng=gps_data.gps.lon;
  ins.gps_alt=gps_data.gps.hmsl;
  ins.gps_alt_cm=gps_data.gps.hmsl/10.0f;//cm
  ins.gps_obs_vel_enu[EAST] = 0.1f*gps_data.gps.vele;//cm/s
  ins.gps_obs_vel_enu[NORTH]= 0.1f*gps_data.gps.veln;//cm/s
  ins.gps_obs_vel_enu[_UP]   =-0.1f*gps_data.gps.veld;//cm/s
  ins.gps_heading=0.00001f*gps_data.gps.headmot;		 //deg
  ins.gps_hacc=gps_data.gps.hacc*0.001f;//m
  ins.gps_vacc=gps_data.gps.vacc*0.001f;//m
  ins.gps_sacc=gps_data.gps.sacc*0.001f;//m/s
  ins.gps_pdop=0.01f*gps_data.gps.pdop;
  
  ins.gps_hacc_m=ins.gps_hacc;
  ins.gps_vacc_m=ins.gps_vacc;
  ins.gps_sacc_mps=ins.gps_sacc;
  
  ins.return_distance_m=get_distance(gps_home_point,gps_present_point);//单位m
  
  for(int16_t i=INS_NUM-1;i>0;i--)//历史位置值，全部装载为当前观测值
  {
    ins.gps_alt_history[i]=ins.gps_alt_history[i-1];
  }
  ins.gps_alt_history[0]=ins.gps_alt_cm-ins.gps_home_alt_cm;
  ins.gps_alt_standard_deviation=calculate_standard_deviation(ins.gps_alt_history,10);
}

bool gps_fix_health(void)//用于决策是否加入GPS定位控制
{
  bool fix_state=false;
  if(gps_data.gps.numsv>=7			 //星数大于等于7
     &&gps_data.gps.fixtype>=0X03//3D-Fix
       &&gps_data.gps.hacc<=2500   //水平位置估计精度小于2.5m=2500mm
	 &&gps_data.gps.sacc<=2000   //水平速度估计精度小于2.0m/s=2000mm/s
           )
    fix_state=true;	
  return fix_state ;
}

bool gps_fusion_break(void)//用于决策是否进行GPS融合
{
  bool fix_state=false;
  if(gps_data.gps.numsv>=5			 //星数大于等于5
     &&gps_data.gps.fixtype>=0X03//3D-Fix
       &&gps_data.gps.hacc<=5000   //水平位置估计精度小于5.0m=5000mm
	 &&gps_data.gps.sacc<=4000   //水平速度估计精度小于4.0m/s=4000mm/s
           )
    fix_state=true;	
  return fix_state ;
}









/*************************************************************************/
systime pos_ins_delta;
float hor_dt=min_ctrl_dt;
uint16_t hor_history_record_period=2;//4*2.5=10ms
void kalmanfilter_horizontal(void)
{	
  get_systime(&pos_ins_delta);
  hor_dt=pos_ins_delta.period/1000.0f;
  if(hor_dt>1.05f*min_ctrl_dt||hor_dt<0.95f*min_ctrl_dt||isnan(hor_dt)!=0)   hor_dt=min_ctrl_dt;
  
  if(ins.gps_home_fixed_flag==0) 
  {
    gps_homepoint_init();
    return;//返航点未刷新时，不进行融合
  }
  
  if(ins.gps_update_flag==1)
  {
    ins.gps_update_flag=0;
    gps_prase_enu();
    ins.gps_east_update_flag=1;
    ins.gps_north_update_flag=1;
    ins.gps_state_update_flag=1;
  }
  
  //已刷新返航点
  if(gps_fusion_break()==false)//gps定位质量差，退出当前惯导融合
  {
    strapdown_ins_reset(&ins,EAST,ins.gps_obs_pos_enu[EAST],
                        ins.gps_obs_vel_enu[EAST]);  //复位惯导融合
    strapdown_ins_reset(&ins,NORTH,ins.gps_obs_pos_enu[NORTH],
                        ins.gps_obs_vel_enu[NORTH]);//复位惯导融合
    return;
  }
  
  //惯导历史值保存
  static uint32_t _cnt=0;
  _cnt++;
  if(_cnt%hor_history_record_period==0)
  {
    for(uint16_t i=INS_NUM-1;i>0;i--)
    {
      ins.pos_backups[EAST][i]=ins.pos_backups[EAST][i-1];
      ins.pos_backups[NORTH][i]=ins.pos_backups[NORTH][i-1];
      ins.vel_backups[EAST][i]=ins.vel_backups[EAST][i-1];
      ins.vel_backups[NORTH][i]=ins.vel_backups[NORTH][i-1];
    }	
  }
  ins.pos_backups[EAST][0] =ins.position[EAST];
  ins.vel_backups[EAST][0] =ins.speed[EAST];
  ins.pos_backups[NORTH][0]=ins.position[NORTH];
  ins.vel_backups[NORTH][0]=ins.speed[NORTH];
  
  gps_ekf_update(&ins,hor_dt,&ins.gps_state_update_flag);
}

