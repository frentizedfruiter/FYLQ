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


#include "datatype.h"
#include "schedule.h"
#include "filter.h"
#include "wp_math.h"
#include "pid.h"
#include "rc.h"
#include "drv_tofsense.h"
#include "drv_opticalflow.h"
#include "nclink.h"
#include "sins.h"
#include "attitude_ctrl.h"
#include "altitude_ctrl.h"
#include "position_ctrl.h"
#include "land_ctrl.h"



/***************************************************************************************/
maple_land_state land_state={
  .first_swich_in=1,
  .alt_target_time=2000,
  .alt_cnt=0,
  .process_state=0,
  .home_fixed_flag=FARAWAY_MODE,
  .max_cruising_speed=NAV_SPEED_FIRST
};


float speed_curve_fit(float target,uint16_t transition_time_ms,uint16_t *cnt)
{
  uint16_t max_cnt=transition_time_ms/duty_dt_ms; 
  if(*cnt<max_cnt) *cnt+=1;
  float scale=*cnt/(max_cnt*1.0f);
  return scale*target;
}


void land_run(void)
{
#define land_gps_home_lng ins.gps_home_lng
#define land_gps_home_lat ins.gps_home_lat
  if(gps_fix_health()==false)//GPS定位状态未锁定，姿态期望直接来源于遥控器给定
  {
    land_without_gps();//GPS无效时，执行一键着陆操作
  }
  else
  {
    land_with_gps(land_gps_home_lng,land_gps_home_lat);//GPS有效时，执行一键返航操作
  }
#undef land_gps_home_lng
#undef land_gps_home_lat	
}


void land_without_gps(void)
{
#define curr_height ins.position_z
  
  maplepilot.roll_outer_control_output =rc_data.rc_rpyt[RC_ROLL];
  maplepilot.pitch_outer_control_output=rc_data.rc_rpyt[RC_PITCH];
  maplepilot.yaw_ctrl_mode=ROTATE;
  maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
  
  float decline_speed=0;
  if(curr_height<=NEAR_GROUND_DISTANCE)//接近地面
    decline_speed=DECLINE_VEL_NEAR_GROUND;
  else if(curr_height<=SAFE_GROUND_DISTANCE)//安全高度以内
    decline_speed=DECLINE_VEL_DEFAULT;
  else  decline_speed=DECLINE_VEL_FAR_GROUND;//远离地面
  
  flight_altitude_control(ALTHOLD_AUTO_VEL_CTRL,NUL,decline_speed);//高度控制
#undef curr_height
}




void land_state_reset(void)
{
#define curr_height ins.position_z
#define curr_lng		ins.gps_lng
#define curr_lat		ins.gps_lat
  
  land_state.last_distance_mode=0;
  land_state.curr_distance_mode=0;
  land_state.home_fixed_flag=FARAWAY_MODE;//远离home点
  
  land_state.first_swich_in=1;
  
  land_state.target.lng=curr_lng;
  land_state.target.lat=curr_lat;
  land_state.target.update_flag=1;
  
  land_state.alt_target_pos=curr_height;
  land_state.alt_target_time=2000;
  land_state.alt_cnt=0;
  land_state.alt_target_vel=0;
  
  land_state.poshold_mode=ALTHOLD_MANUAL_CTRL;
  land_state.althold_mode=POSHLOD_MANUAL_CTRL;
  land_state.process_state=0;
  
#undef curr_height
#undef curr_lng	
#undef curr_lat
}



void land_state_check(void)
{
  float distance=100*get_distance(gps_home_point,gps_present_point);//单位cm
  
  land_state.last_distance_mode=land_state.curr_distance_mode;
  
  if(distance>=FARAWAY_DIS_CM)//离起飞点水平距离超过Faraway_Distance半径      
  {
    land_state.home_fixed_flag=FARAWAY_MODE;//远离home点
    land_state.curr_distance_mode=1;//先原地上升至安全高度再返航到水平点正上方
  }
  else if(distance>=NEAR_DIS_CM)//离起飞点水平距离Near_Distance~Faraway_Distance半径 
  {
    land_state.home_fixed_flag=NEAR_MODE;//接近home点
    land_state.curr_distance_mode=2;//先保持当前高度，水平运动至home点正上方，再下降
  }
  else//与home点距离小于等于Near_Distance半径，已到达home点 
  {
    land_state.home_fixed_flag=REACH_MODE;//已锁定home点
    land_state.curr_distance_mode=3;//水平位置保持为home点，并直接下降
  }
}

void land_with_gps(int32_t lng,int32_t lat)
{
#define curr_height ins.position_z
#define curr_lng		ins.gps_lng
#define curr_lat		ins.gps_lat
  
  static uint8_t breakoff_cnt=0;
  //返航降落过程中,存在手动打杆介入控制时
  if(rc_data.rc_rpyt[RC_ROLL]!=0
     ||rc_data.rc_rpyt[RC_PITCH]!=0
       ||rc_data.rc_rpyt[RC_YAW]!=0
         ||rc_data.rc_rpyt[RC_THR]!=0)
    //执行普通GPS定点模式
  { 
    if(breakoff_cnt==0)//第二步：强制将目标悬停高度、水平位置刷新到当前位置估计值
    {
      flight_altitude_control(ALTHOLD_AUTO_POS_CTRL,curr_height,NUL);		//高度控制
      flight_position_control(POSHLOD_MANUAL_CTRL,NUL,NUL,NUL,NUL,NUL,NUL,1); //位置控制		
      breakoff_cnt=1;
    }
    else if(breakoff_cnt==1)//第二步：执行常规GPS定点模式
    {
      flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
      flight_position_control(POSHLOD_MANUAL_CTRL,NUL,NUL,NUL,NUL,NUL,NUL,0);//位置控制		
    }
    //存在打杆操作时，对返航中间状态复位，回中即重新恢复返航
    land_state_reset();
    return ;		
  }
  breakoff_cnt=0;
  
  static systime land_dt;
  get_systime(&land_dt);
  //如果本函数运行周期大于控制周期10倍
  if(land_dt.period>=10*duty_dt_ms)
  {
    //情况1:初次从其它模式切入本模式
    //情况2:系统调度超时，在系统设计合理情况下，本情况不可能发生
    land_state_reset();//复位返航中间状态
  }
  
  /***********************************************************************/
  land_state_check();//实时监测水平位置状态
  
  switch(land_state.home_fixed_flag)
  {
  case FARAWAY_MODE:land_state.max_cruising_speed=NAV_SPEED_FIRST;	break;//从A->B，开启一级巡航速度，单位cm
  case NEAR_MODE	 :land_state.max_cruising_speed=NAV_SPEED_SECOND;	break;//从B->C，开启二级巡航速度，单位cm
  case REACH_MODE  :land_state.max_cruising_speed=NAV_SPEED_THIRD;	break;//处于C点附近，开启三级巡航速度, 单位cm
  default					 :land_state.max_cruising_speed=NAV_SPEED_DEFAULT;//默认巡航速度
  }
  
  if(land_state.curr_distance_mode==1&&land_state.first_swich_in==1)//首次切返航模式，距离home很远时
  {
    if(curr_height<SAFE_GROUND_DISTANCE)//当切返航瞬间的高度小于安全高度时，保持当前水平位置，攀升至安全高度再返航
    {
      land_state.target.lng=curr_lng;
      land_state.target.lat=curr_lat;
      land_state.target.update_flag=1;
      
      //
      land_state.alt_target_time=2000;
      land_state.alt_cnt=0;
      land_state.alt_target_vel=speed_curve_fit(RISE_VEL_DEFAULT,
                                                land_state.alt_target_time,
                                                &land_state.alt_cnt);
      
      land_state.poshold_mode=POSHOLD_SEMI_AUTO_POS_CTRL;//水平方向——位置控制
      land_state.althold_mode=ALTHOLD_AUTO_VEL_CTRL;//高度方向——速度控制
      
      land_state.process_state=0;
    }
    else//当距离很远且返航瞬间的高度大于安全高度时：保持当前高度，执行返航
    {
      land_state.target.lng=lng;
      land_state.target.lat=lat;	
      land_state.target.update_flag=1;
      //
      land_state.alt_target_pos=curr_height;
      
      land_state.poshold_mode=POSHOLD_SEMI_AUTO_POS_CTRL;//水平方向——位置控制
      land_state.althold_mode=ALTHOLD_AUTO_POS_CTRL;//高度方向——位置控制
      
      land_state.process_state=1;
    }
    land_state.first_swich_in=0;
    land_state.last_distance_mode=1;//确保能顺利进入下一阶段
    
    //
    flight_altitude_control(land_state.althold_mode,
                            land_state.alt_target_pos,
                            land_state.alt_target_vel);//高度控制
    flight_position_control(land_state.poshold_mode,
                            land_state.target.lng,
                            land_state.target.lat,
                            NUL,NUL,
                            land_state.max_cruising_speed,
                            land_state.target.update_flag,
                            NUL);//位置控制		
  }
  else if(land_state.curr_distance_mode==1&&land_state.last_distance_mode==1)//持续处于远离home点较远位置
  {
    //继续执行上一线程未完成的任务
    if(land_state.process_state==0)//先上升到安全高度
    {
      land_state.alt_target_vel=speed_curve_fit(RISE_VEL_DEFAULT,
                                                land_state.alt_target_time,
                                                &land_state.alt_cnt);	
      
      if(curr_height>=SAFE_GROUND_DISTANCE)//已经到达安全高度
      {
        //保持当前高度，直接返航
        land_state.target.lng=lng;
        land_state.target.lat=lat;	
        land_state.target.update_flag=1;
        //
        land_state.alt_target_pos=curr_height;
        
        land_state.poshold_mode=POSHOLD_SEMI_AUTO_POS_CTRL;//水平方向——位置控制
        land_state.althold_mode=ALTHOLD_AUTO_POS_CTRL;//高度方向——位置控制
        
        land_state.process_state=1;
      }
    }
    else if(land_state.process_state==1)//直接返航
    {
      //继续保持执行上一线程设定的指令
    }
    
    flight_altitude_control(land_state.althold_mode,
                            land_state.alt_target_pos,
                            land_state.alt_target_vel);//高度控制
    flight_position_control(land_state.poshold_mode,
                            land_state.target.lng,
                            land_state.target.lat,
                            NUL,NUL,
                            land_state.max_cruising_speed,
                            land_state.target.update_flag,
                            NUL);//位置控制		
  }
  else if(land_state.curr_distance_mode==2&&land_state.last_distance_mode!=2)//距离home较近时，保持当前高度，将Home点作为目标点，飞至home正上方
  {
    //保持当前高度，直接返航
    //这里包含两种情况：1、A->B  2、C->B
    //执行的操作相同，即：保持当前高度，向home靠近
    
    land_state.target.lng=lng;
    land_state.target.lat=lat;	
    land_state.target.update_flag=1;
    //
    land_state.alt_target_pos=curr_height;
    
    land_state.poshold_mode=POSHOLD_SEMI_AUTO_POS_CTRL;//水平方向——位置控制
    land_state.althold_mode=ALTHOLD_AUTO_POS_CTRL;//高度方向——位置控制
    
    land_state.process_state=2;
    land_state.last_distance_mode=2;//确保能顺利进入下一阶段
    
    flight_altitude_control(land_state.althold_mode,
                            land_state.alt_target_pos,
                            land_state.alt_target_vel);//高度控制
    flight_position_control(land_state.poshold_mode,
                            land_state.target.lng,
                            land_state.target.lat,
                            NUL,NUL,
                            land_state.max_cruising_speed,
                            land_state.target.update_flag,
                            NUL);//位置控制		
  }
  else if(land_state.curr_distance_mode==2&&land_state.last_distance_mode==2)//持续处于较近位置，保持上一线程
  {
    //继续保持执行上一线程设定的指令
    flight_altitude_control(land_state.althold_mode,
                            land_state.alt_target_pos,
                            land_state.alt_target_vel);//高度控制
    flight_position_control(land_state.poshold_mode,
                            land_state.target.lng,
                            land_state.target.lat,
                            NUL,NUL,
                            land_state.max_cruising_speed,
                            land_state.target.update_flag,
                            NUL);//位置控制		
  }
  else if(land_state.curr_distance_mode==3&&land_state.last_distance_mode!=3)//首次进入home点正上方，直接原地下降
  {
    land_state.target.lng=lng;
    land_state.target.lat=lat;
    land_state.target.update_flag=1;
    
    //
    land_state.alt_target_time=2000;
    land_state.alt_cnt=0;
    land_state.alt_target_vel=speed_curve_fit(DECLINE_VEL_DEFAULT,
                                              land_state.alt_target_time,
                                              &land_state.alt_cnt);
    
    land_state.poshold_mode=POSHOLD_SEMI_AUTO_POS_CTRL;//水平方向——位置控制
    land_state.althold_mode=ALTHOLD_AUTO_VEL_CTRL;//高度方向——速度控制	
    //继续保持执行上一线程设定的指令
    flight_altitude_control(land_state.althold_mode,
                            land_state.alt_target_pos,
                            land_state.alt_target_vel);//高度控制
    flight_position_control(land_state.poshold_mode,
                            land_state.target.lng,
                            land_state.target.lat,
                            NUL,NUL,
                            land_state.max_cruising_speed,
                            land_state.target.update_flag,
                            NUL);//位置控制	
  }
  else if(land_state.curr_distance_mode==3&&land_state.last_distance_mode==3)//持续处于home点正上方，继续下降
  {
    land_state.alt_target_vel=speed_curve_fit(DECLINE_VEL_DEFAULT,
                                              land_state.alt_target_time,
                                              &land_state.alt_cnt);		
    //继续保持执行上一线程设定的指令
    flight_altitude_control(land_state.althold_mode,
                            land_state.alt_target_pos,
                            land_state.alt_target_vel);//高度控制
    flight_position_control(land_state.poshold_mode,
                            land_state.target.lng,
                            land_state.target.lat,
                            NUL,NUL,
                            land_state.max_cruising_speed,
                            land_state.target.update_flag,
                            NUL);//位置控制		
  }
  
#undef curr_height
#undef curr_lng	
#undef curr_lat	
}
