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


#include <math.h>
#include <wp_math.h>
#include "quaternion.h"
#include <string.h>
#include "datatype.h"
#include "maple_config.h"
#include "rc.h"
#include "calibration.h"
#include "drv_notify.h"
#include "pid.h"
#include "altitude_ctrl.h"


extern int16_t page_number;


rc rc_data={
  .fc_ready_flag=0,
  .unwanted_lock_flag=1,
};

uint8_t rc_read_switch(uint16_t ch)
{
  uint16_t pulsewidth = ch;
  if (pulsewidth <= 900 || pulsewidth >= 2200) return 255;// This is an error condition
  if (pulsewidth <= 1230) return 0;
  if (pulsewidth <= 1360) return 1;
  if (pulsewidth <= 1490) return 2;
  if (pulsewidth <= 1620) return 3;
  if (pulsewidth <= 1749) return 4;// Software Manual
  return 5;// Hardware Manual
}

void rc_init(void)
{
  float _rc_deadband_percent=0;
  for(uint16_t i=0;i<RC_CHL_MAX;i++)//先用默认值初始化
  {
    if(i==RC_THR_CHANNEL) _rc_deadband_percent=RC_THR_DEADBAND_PERCENT;
    else _rc_deadband_percent=RC_DEADBAND_PERCENT;
    rc_data.cal[i].max_value=RC_MAX_DEFAULT;
    rc_data.cal[i].min_value=RC_MIN_DEFAULT;
    rc_data.cal[i].deadband	=(rc_data.cal[i].max_value-rc_data.cal[i].min_value)*_rc_deadband_percent;
    rc_data.cal[i].middle_value=RC_MIDDLE_DEFAULT;
    rc_data.cal[i].reverse_flag=RC_REVERSE_FLAG;
    rc_data.cal[i].scale=(rc_data.cal[i].max_value-rc_data.cal[i].min_value-rc_data.cal[i].deadband)*0.5f;
  }
}


float remote_data_remap(rc *data,uint16_t ch,float max_down,float max_up,bool reverse_flag)
{
  float value=0;
  if(data->rcdata[ch]<=data->cal[ch].middle_value-0.5*data->cal[ch].deadband)
    value=(data->cal[ch].middle_value-0.5*data->cal[ch].deadband-data->rcdata[ch])*max_down/data->cal[ch].scale;
  else if(data->rcdata[ch]>=data->cal[ch].middle_value+0.5*data->cal[ch].deadband)
    value=(data->cal[ch].middle_value+0.5*data->cal[ch].deadband-data->rcdata[ch])*max_up/data->cal[ch].scale;	
  else value=0;
  
  if(reverse_flag)  value*=(-1);
  return 	value;
}




THR_PUSH_MODE thr_push_over_deadband(void)
{
  static THR_POSITION now_state=THR_BUTTOM,last_state=THR_BUTTOM;
  static THR_PUSH_MODE push_flag=FREE_STYLE;
  last_state=now_state;
  if(rc_data.rc_rpyt[RC_THR]<0)//处于低位
    now_state=THR_BUTTOM;
  else if(rc_data.rc_rpyt[RC_THR]==0)//处于中位 
    now_state=THR_MIDDLE;
  else now_state=THR_UP;//处于高位  
  
  if(last_state!=now_state)//前后两次状态不一
  {
    if(now_state==THR_MIDDLE&&last_state==THR_BUTTOM)       push_flag=BUTTOM_TO_MIDDLE;//低位向上推到中位死区
    else if(now_state==THR_BUTTOM&&last_state==THR_MIDDLE)  push_flag=MIDDLE_TO_BUTTOM;//中位死区向下推到低位
    else if(now_state==THR_UP&&last_state==THR_MIDDLE)      push_flag=MIDDLE_TO_UP;//中位死区向上推到高位
    else if(now_state==THR_MIDDLE&&last_state==THR_UP)      push_flag=UP_TO_MIDDLE;//高位向下推到中位死区	
  }
  else
  {
    if(now_state==THR_BUTTOM&&last_state==THR_BUTTOM)       push_flag=FREE_STYLE;//持续处于低位
  }
  return push_flag;
}


void rc_prase(ppm *_ppm)
{
  if(_ppm->update_flag==0)  return;
  _ppm->update_flag=0;
  
  memcpy(rc_data.rcdata,_ppm->data,RC_CHL_MAX*sizeof(uint16_t));//更新每个通道遥控器数据
  
  rc_data.rc_rpyt[RC_ROLL]	=remote_data_remap(&rc_data ,RC_ROLL_CHANNEL	,Pit_Rol_Max,Pit_Rol_Max,true);
  rc_data.rc_rpyt[RC_PITCH] =remote_data_remap(&rc_data ,RC_PITCH_CHANNEL ,Pit_Rol_Max,Pit_Rol_Max,true);
  rc_data.rc_rpyt[RC_YAW]		=remote_data_remap(&rc_data ,RC_YAW_CHANNEL	  ,Yaw_Max		,Yaw_Max    ,false);
  rc_data.rc_rpyt[RC_THR]		=remote_data_remap(&rc_data ,RC_THR_CHANNEL	  ,Climb_Down_Speed_Max,Climb_Up_Speed_Max,true);
  
  rc_data.rc_rpy[RC_PITCH] =-DEGTORAD(rc_data.rc_rpyt[RC_ROLL]);
  rc_data.rc_rpy[RC_ROLL]  =-DEGTORAD(rc_data.rc_rpyt[RC_PITCH]);
  rc_data.rc_rpy[RC_YAW]   =0;//DEGTORAD(rc_data.rc_rpyt[RC_YAW]);
  //euler_to_quaternion(rc_data.rc_rpy,rc_data.rc_q);
  
  float thr_remap=(float)(rc_data.rcdata[RC_THR_CHANNEL]-rc_data.cal[RC_THR_CHANNEL].min_value) 
    /(rc_data.cal[RC_THR_CHANNEL].max_value-rc_data.cal[RC_THR_CHANNEL].min_value);
  rc_data.throttle=1000+1000*thr_remap;
  
  rc_data.thr      =rc_data.rcdata[RC_THR_CHANNEL];
  rc_data.aux[AUX1]=rc_data.rcdata[RC_AUX1_CHANNEL];
  rc_data.aux[AUX2]=rc_data.rcdata[RC_AUX2_CHANNEL];
  rc_data.aux[AUX3]=rc_data.rcdata[RC_AUX3_CHANNEL];
  rc_data.aux[AUX4]=rc_data.rcdata[RC_AUX4_CHANNEL];
  
  switch(rc_read_switch(rc_data.aux[AUX1]))//第5通道
  {
  case 0:
  case 1:rc_data.height_mode=1;break;//姿态自稳
  case 2:
  case 3:
  case 4:
  case 5:rc_data.height_mode=5;break;//姿态定高
  default:rc_data.height_mode=1;
  }
  
  switch(rc_read_switch(rc_data.aux[AUX2]))//第6通道
  {
  case 0:
  case 1:rc_data.sdk_duty_mode=1;break;//非SDK
  case 2:
  case 3:
  case 4:
  case 5:rc_data.sdk_duty_mode=5;break;//SDK模式
  default:rc_data.sdk_duty_mode=1;
  }
  
  switch(rc_read_switch(rc_data.aux[AUX3]))//第7通道
  {
  case 0:
  case 1:rc_data.gps_mode=1;break;//非LAND
  case 2:
  case 3:
  case 4:rc_data.gps_mode=4;break;//GPS模式
  case 5:rc_data.gps_mode=5;break;//返航着陆模式
  default:rc_data.gps_mode=1;
  }
  
  switch(rc_read_switch(rc_data.aux[AUX4]))//第8通道
  {
  case 0:
  case 1:rc_data.position_mode=1;break;//普通模式
  case 2:
  case 3:
  case 4:
  case 5:rc_data.position_mode=5;break;//光流定点
  default:rc_data.position_mode=1;
  }
  
  rc_range_calibration(&rc_data);//遥控器行程标定
  unlock_state_check();//上锁、解锁动作判断
  
  rc_data.thr_push_over_state=thr_push_over_deadband();
  if((rc_data.thr_push_over_state!=FREE_STYLE)//油门推过低位死区
   &&(rc_data.thr_push_over_state!=BUTTOM_TO_MIDDLE))//油门从低位推到中位
  {
    rc_data.unwanted_lock_flag=0;//禁止地面检测自动上锁标志位
    
    rc_data.auto_relock_flag=0;
    rc_data.auto_relock_cnt=0;
  }
}

void unlock_state_check(void)
{
  rc_data.last_lock_state=rc_data.lock_state;
  
  //上锁逻辑
  if(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
     &&rc_data.rc_rpyt[RC_ROLL]==0
       &&rc_data.rc_rpyt[RC_PITCH]==0
	 &&rc_data.rc_rpyt[RC_YAW]>=Yaw_Max*Scale_Pecent_Max)
    rc_data.lock_makesure_cnt++;
  else rc_data.lock_makesure_cnt/=5;
  
  if(rc_data.lock_makesure_cnt>=50)//50*20ms=1000ms持续1S时间
  {
    rc_data.lock_state=LOCK;        //上锁状态赋值 
    rc_data.unlock_makesure_cnt=0;  //清空解锁计数器
    rc_data.lock_makesure_cnt=0;    //清空上锁计数器
    rc_data.auto_relock_flag=0;     //清空自动上锁标志位
    rgb_notify_set(GREEN ,TOGGLE	 ,500,3000,200);
    
    rc_data.rc_return_flag=0;
    rc_data.auto_relock_cnt=0;
    page_number=0;
  }
  
  //解锁逻辑
  if(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
     &&rc_data.rc_rpyt[RC_ROLL]==0
       &&rc_data.rc_rpyt[RC_PITCH]==0
	 &&rc_data.rc_rpyt[RC_YAW]<=-Yaw_Max*Scale_Pecent_Max
           &&rc_data.fc_ready_flag==1 //温控就位
             &&flight_calibrate.free==1)//非校准状态
    rc_data.unlock_makesure_cnt++;
  else rc_data.unlock_makesure_cnt/=5;
  
  if(rc_data.unlock_makesure_cnt>=100)//100*20ms=2000ms持续2S时间
  {
    rc_data.lock_state=UNLOCK;			//解锁状态赋值
    rc_data.unlock_makesure_cnt=0;  //清空解锁计数器
    rc_data.lock_makesure_cnt=0;    //清空上锁计数器
    rgb_notify_set(BLUE ,TOGGLE	 ,200,5000,200);	
  }
  
  //飞控自动上锁判断
  if(rc_data.last_lock_state==LOCK&&rc_data.lock_state==UNLOCK)//解锁完成时刻
  { 		
    rc_data.auto_relock_flag	=	0; //设置自动上锁标志位
    rc_data.auto_relock_cnt	=	300; //设置解锁无任何操作，6秒后自动上锁
    
    rc_data.rc_return_flag=0;  		 //遥控器解锁后未回中标志位
    
    rc_data.thr_push_over_state = FREE_STYLE;
    reset_landon_state();        //复位地面检测相关状态量
    
    if(rc_data.height_mode==5)//定高模式下
    {
      maple_ctrl.height_accel_ctrl.integrate=-maple_ctrl.height_accel_ctrl.integrate_max;
      rc_data.unwanted_lock_flag = 1;//禁止地面检测自动上锁标志位
    }
    else rc_data.unwanted_lock_flag = 0;		
  }
  
  //解锁动作触发解锁后，待遥感回中后进入无动作自动上锁检测
  if(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
     &&rc_data.rc_rpyt[RC_ROLL]==0
       &&rc_data.rc_rpyt[RC_PITCH]==0
	 &&rc_data.rc_rpyt[RC_YAW]==0
           &&rc_data.lock_state==UNLOCK
             &&rc_data.auto_relock_flag==0
               &&rc_data.auto_relock_cnt==300)
  {
    if(rc_data.unwanted_lock_flag==0)//纯姿态模式下解锁，才会允许遥控器回中后检测无动作自动上锁
    {
      rc_data.auto_relock_flag=1;  //设置自动上锁标志位
    }
    rc_data.rc_return_flag=1;//回中动作标志位
  }
  
  
  //解锁后自动上锁逻辑判断
  if(rc_data.auto_relock_flag==1)
  {
    if(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
       &&rc_data.rc_rpyt[RC_ROLL]==0
         &&rc_data.rc_rpyt[RC_PITCH]==0
           &&rc_data.rc_rpyt[RC_YAW]==0)//遥控器解锁后，6S内无任何操作动作位
    {	
      rc_data.auto_relock_cnt--;//自动上锁计数器自减
      if(rc_data.auto_relock_cnt<=0)  rc_data.auto_relock_cnt=0;
      if(rc_data.auto_relock_cnt==0)
      {
        rc_data.lock_state=LOCK;
        rc_data.auto_relock_flag=0;//清空自动上锁标志位
        rc_data.rc_return_flag=0;
        rgb_notify_set(GREEN ,TOGGLE	 ,500,3000,200);
      }
    }
    
    if((rc_data.thr>rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
        ||FastAbs(rc_data.rc_rpyt[RC_ROLL])>=0.1f*Pit_Rol_Max
          ||FastAbs(rc_data.rc_rpyt[RC_PITCH])>=0.1f*Pit_Rol_Max
            ||FastAbs(rc_data.rc_rpyt[RC_YAW])>=0.5f*Yaw_Max)
       &&rc_data.rc_return_flag==1&&rc_data.auto_relock_cnt>0)//遥控器解锁回中后，6S内存在操作动作位
    {	
      rc_data.auto_relock_flag=0;//清空自动上锁标志位
      rc_data.auto_relock_cnt=0;			 
      //rc_data.rc_return_flag=0;
    }		 
  }
}

