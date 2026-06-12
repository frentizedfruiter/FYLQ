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
#include "maple_config.h"
#include "schedule.h"
#include "filter.h"
#include "wp_math.h"
#include "pid.h"
#include "rc.h"
#include "drv_tofsense.h"
#include "drv_opticalflow.h"
#include "nclink.h"
#include "reserved_serialport.h"
#include "sins.h"
#include "attitude_ctrl.h"
#include "position_ctrl.h"


vector2f accel_target={0,0},angle_target={0,0},speed_target={0,0};
static vector3f uav_cushion_stop_point;

static float earth_frame_pos_err[3];
static float body_frame_pos_err[3];
static float body_frame_brake_speed[3];
static float body_frame_speed_feedback[3];
static float body_frame_speed_current[3];

float flight_speed_remap(float input,uint16_t input_max,float output_max)
{
  float output_speed=0;
  float temp_scale=(float)(input/input_max);
  temp_scale=constrain_float(temp_scale,-1.0f, 1.0f);
  if(temp_scale>=0) output_speed=(float)(output_max*temp_scale*temp_scale);
  else output_speed=(float)(-output_max*temp_scale*temp_scale); 
  return output_speed;
}


//定点模式下，遥杆回中后，先用水平速度控制刹车，待刹停后再赋值位置选点
uint8_t get_stopping_point_xy(vector3f *stopping_point)
{
  static uint16_t span_cnt=0;
  float vel_total=0;
  vector2f curr_pos,curr_vel;
  curr_pos.x=ins.position[EAST];
  curr_pos.y=ins.position[NORTH];
  curr_vel.x=ins.speed[EAST];
  curr_vel.y=ins.speed[NORTH];
  vel_total=safe_sqrt(sq(curr_vel.x)+sq(curr_vel.y));
  if(vel_total<=POSHOLD_SPEED_0) //合水平速度的小于等于10cm/s
  {
    stopping_point->x = curr_pos.x;
    stopping_point->y = curr_pos.y;
    span_cnt++;
    if(span_cnt>=200/POS_CTRL_PERIOD)//持续200ms满足
    {
      span_cnt=0;
      return 1;
    }
  }
  else span_cnt/=2;		
  return 0;
}

void uav_hoverpoint_xy_update(float a,float b,float *target_a,float *target_b)
{
  *target_a=a;
  *target_b=b;
}

//导航坐标系下向量转到机体Pitch、Roll方向上
void from_enu_to_body_frame(float e,float n,float *right,float *forward)
{
  *right  = e*flymaple.cos_rpy[_YAW]+n*flymaple.sin_rpy[_YAW];
  *forward=-e*flymaple.sin_rpy[_YAW]+n*flymaple.cos_rpy[_YAW];
}

void from_body_to_enu_frame(float right,float forward,float *e,float *n)
{
  *e= right*flymaple.cos_rpy[_YAW]-forward*flymaple.sin_rpy[_YAW];
  *n= right*flymaple.sin_rpy[_YAW]+forward*flymaple.cos_rpy[_YAW];
}


void flight_position_control(uint8_t mode,
                             int32_t target_lng,int32_t target_lat,//目标经、纬度
                             float target_e_vel,float target_n_vel,  //目标巡航速度
                             float max_cruising_speed,
                             uint8_t target_update,															 
                             uint8_t force_fixed_flag)
{
  static uint8_t brake_stage_flag=0;
  static uint8_t miss_fixed_flag=0;
  if(gps_fix_health()==false)//GPS定位状态未锁定，姿态期望直接来源于遥控器给定
  {
    maplepilot.roll_outer_control_output	=rc_data.rc_rpyt[ROLL];
    maplepilot.pitch_outer_control_output=rc_data.rc_rpyt[PITCH];
    //maplepilot.yaw_outer_control_output	=rc_data.rc_rpyt[YAW];
    miss_fixed_flag=1;		
    return ;
  }
  static uint16_t pos_cnt=0,vel_cnt=0;
  static systime poshold_dt;
  get_systime(&poshold_dt);
  //如果本函数运行周期大于控制周期10倍
  if(0.001f*poshold_dt.period>=10*min_ctrl_dt)
  {
    //情况1:初次从其它模式切入本模式
    //情况2:系统调度超时，在系统设计合理情况下，本情况不可能发生
    miss_fixed_flag=1;
  }
  
  if(force_fixed_flag==1//强制更新当前位置为目标点:如初入GPS定点
     ||miss_fixed_flag==1)//之前未满足定位条件,未锁定目标点
  {
    miss_fixed_flag=0;
    uav_hoverpoint_xy_update(ins.position[NORTH],ins.position[EAST],
                             &maple_ctrl.latitude_position_ctrl.expect,&maple_ctrl.longitude_position_ctrl.expect);
    maple_ctrl.latitude_speed_ctrl.expect=0;
    maple_ctrl.longitude_speed_ctrl.expect=0;
    pid_integrate_reset(&maple_ctrl.latitude_position_ctrl);
    pid_integrate_reset(&maple_ctrl.longitude_position_ctrl);
    pid_integrate_reset(&maple_ctrl.latitude_speed_ctrl);
    pid_integrate_reset(&maple_ctrl.longitude_speed_ctrl);
  }
  
  //需要将惯性导航E、N方向的速度估计旋转到以载体俯仰、横滚水平面投影的游动直角坐标系
  from_enu_to_body_frame(ins.speed[EAST],ins.speed[NORTH],&body_frame_speed_current[_RIGHT],&body_frame_speed_current[_FORWARD]);
  
  switch(mode)
  {
  case POSHLOD_MANUAL_CTRL:
    {	
      if(rc_data.rc_rpyt[ROLL]==0&&rc_data.rc_rpyt[PITCH]==0)//水平方向遥控量无任何动作
      {
        pos_cnt++;
        if(pos_cnt>=POS_CTRL_CNT)
        {
          pos_cnt=0;
          //位置期望,经纬、航行速度、高度
          if(maple_ctrl.latitude_position_ctrl.expect==0&&maple_ctrl.longitude_position_ctrl.expect==0)//方向杆回中后，只设置一次
          {
            if(get_stopping_point_xy(&uav_cushion_stop_point)==1)
            {
              maple_ctrl.latitude_position_ctrl.expect=uav_cushion_stop_point.y;
              maple_ctrl.longitude_position_ctrl.expect=uav_cushion_stop_point.x;
              brake_stage_flag=0;
            }
            else
            {
              //速度控制器期望
              maple_ctrl.latitude_speed_ctrl.expect =0;
              maple_ctrl.longitude_speed_ctrl.expect=0;
              brake_stage_flag=1;
            }
          }						
          
          if(brake_stage_flag==0)//飞控不处于刹车阶段时,速度期望来源于位置控制器输出
          {					
            //位置反馈，来源于当前惯导的位置估计
            maple_ctrl.latitude_position_ctrl.feedback = ins.position[NORTH];
            maple_ctrl.longitude_position_ctrl.feedback= ins.position[EAST];
            //导航坐标系下E、N方向上位置偏差
            earth_frame_pos_err[NORTH]=maple_ctrl.latitude_position_ctrl.expect-maple_ctrl.latitude_position_ctrl.feedback;
            earth_frame_pos_err[EAST] =maple_ctrl.longitude_position_ctrl.expect-maple_ctrl.longitude_position_ctrl.feedback;
            
            //导航坐标系下机体Pitch、Roll方向上位置偏差
            from_enu_to_body_frame(earth_frame_pos_err[EAST],earth_frame_pos_err[NORTH],&body_frame_pos_err[_RIGHT],&body_frame_pos_err[_FORWARD]);
            
            //导航坐标系下机体Pitch、Roll方向上期望刹车速度，这里为单比例运算不调用PID_Control()函数
            body_frame_pos_err[_FORWARD]=constrain_float(body_frame_pos_err[_FORWARD],-maple_ctrl.latitude_position_ctrl.err_max, maple_ctrl.latitude_position_ctrl.err_max); //位置偏差限幅，单位cm
            body_frame_pos_err[_RIGHT]  =constrain_float(body_frame_pos_err[_RIGHT]  ,-maple_ctrl.longitude_position_ctrl.err_max,maple_ctrl.longitude_position_ctrl.err_max);//位置偏差限幅，单位cm
            //位置控制器比例运算，直接计算得到速度期望值
            body_frame_brake_speed[_FORWARD]=maple_ctrl.latitude_position_ctrl.kp *body_frame_pos_err[_FORWARD];
            body_frame_brake_speed[_RIGHT]  =maple_ctrl.longitude_position_ctrl.kp*body_frame_pos_err[_RIGHT];
            //更新速度控制器期望
            maple_ctrl.latitude_speed_ctrl.expect =body_frame_brake_speed[_FORWARD];
            maple_ctrl.longitude_speed_ctrl.expect=body_frame_brake_speed[_RIGHT];	
          }							
        }				
      }
      else//水平方向遥控量有动作，直接将遥控器动作位映射成期望运动速度
      {
        maple_ctrl.latitude_speed_ctrl.expect =flight_speed_remap(-rc_data.rc_rpyt[PITCH],Pit_Rol_Max,Max_Horvel);
        maple_ctrl.longitude_speed_ctrl.expect=flight_speed_remap( rc_data.rc_rpyt[ROLL], Pit_Rol_Max,Max_Horvel);
        
        maple_ctrl.latitude_position_ctrl.expect=0;
        maple_ctrl.longitude_position_ctrl.expect=0;
        brake_stage_flag=0;
      }
      vel_cnt++;
      if(vel_cnt>=VEL_CTRL_CNT)
      {
        vel_cnt=0;
        //下面获取导航系下沿飞行器俯仰、横滚方向的速度反馈量，
        //需要将惯性导航E、N方向的速度估计旋转到以载体俯仰、横滚水平面投影的游动直角坐标系				
        body_frame_speed_feedback[_RIGHT]  =body_frame_speed_current[_RIGHT];
        body_frame_speed_feedback[_FORWARD]=body_frame_speed_current[_FORWARD];	
        
        //沿载体游动直角坐标系下的速度反馈量
        maple_ctrl.latitude_speed_ctrl.feedback =body_frame_speed_feedback[_FORWARD];//机头Pitch水平投影方向，Y轴正向
        maple_ctrl.longitude_speed_ctrl.feedback=body_frame_speed_feedback[_RIGHT]; //横滚Roll水平投影方向，X轴正向
        //沿载体方向速度控制器
        pid_ctrl_general(&maple_ctrl.latitude_speed_ctrl,VEL_CTRL_CNT*PERIOD_UNIT_MIN/1000.0f);
        pid_ctrl_general(&maple_ctrl.longitude_speed_ctrl,VEL_CTRL_CNT*PERIOD_UNIT_MIN/1000.0f);
        
        //下面将速度控制器输出————期望水平运动加速度直接映射到期望倾角，计算方法如下：
        accel_target.x=-maple_ctrl.longitude_speed_ctrl.control_output;//横滚水平投影方向下的期望运动加速度
        accel_target.y=-maple_ctrl.latitude_speed_ctrl.control_output;//横滚水平投影方向下的期望运动加速度
        desired_accel_transform_angle(accel_target,&angle_target);//期望运动加速度转期望姿态倾角
        //水平（俯仰、横滚）姿态期望，来源于位置-速度控制器输出，偏航期望仍然来源于遥控器给定
        maplepilot.roll_outer_control_output =angle_target.x;
        maplepilot.pitch_outer_control_output=angle_target.y;
        //maplepilot.yaw_outer_control_output	=rc_data.rc_rpyt[YAW];
      }							
    }
    break;
  case POSHOLD_SEMI_AUTO_POS_CTRL:
    {
      static uint8_t horizontal_position_fixed=0;
      Location target;
      vector2f location_delta={0,0};
      float earth_frame_to_xyz[3]={0,0,0};
      if(target_update==1)//通过目标点的经纬度计算，得到目标点距离home点的正北、正东方向的偏移距离
      {
        target.lng=target_lng;				//更新目标经纬度
        target.lat=target_lat;
        location_delta=location_diff(gps_home_point,target);//根据当前GPS定位信息与Home点位置信息计算正北、正东方向位置偏移
        /***********************************************************************************
        明确下导航系方向，这里正北、正东为正方向:
        沿着正东，经度增加,当无人机相对home点，往正东向移动时，此时GPS_Present.lng>GPS_Home.lng，得到的location_delta.x大于0;
        沿着正北，纬度增加,当无人机相对home点，往正北向移动时，此时GPS_Present.lat>GPS_Home.lat，得到的location_delta.y大于0;
        ******************************************************************************/
        location_delta.x*=100.0f;//沿地理坐标系，正北方向位置偏移,单位为CM
        location_delta.y*=100.0f;//沿地理坐标系，正东方向位置偏移,单位为CM
        earth_frame_to_xyz[EAST] =location_delta.x;//地理系下相对Home点正东位置偏移，单位为CM
        earth_frame_to_xyz[NORTH]=location_delta.y;//地理系下相对Home点正北位置偏移，单位为CM
      }
      
      if(rc_data.rc_rpyt[ROLL]==0&&rc_data.rc_rpyt[PITCH]==0)//水平方向遥控量无任何动作
      {
        pos_cnt++;
        if(pos_cnt>=POS_CTRL_CNT)
        {
          pos_cnt=0;
          if(target_update==1)//更新目标位置
          {
            maple_ctrl.latitude_position_ctrl.expect=earth_frame_to_xyz[NORTH];
            maple_ctrl.longitude_position_ctrl.expect=earth_frame_to_xyz[EAST];
            target_update=0;
          }		
          //速度控制器期望
          if(horizontal_position_fixed==1)//方向杆回中后，只设置一次
          {
            if(get_stopping_point_xy(&uav_cushion_stop_point)==1)
            {
              maple_ctrl.latitude_position_ctrl.expect=uav_cushion_stop_point.y;
              maple_ctrl.longitude_position_ctrl.expect=uav_cushion_stop_point.x;
              
              brake_stage_flag=0;
              horizontal_position_fixed=0;
            }
            else
            {
              //速度控制器期望
              maple_ctrl.latitude_speed_ctrl.expect =0;
              maple_ctrl.longitude_speed_ctrl.expect=0;
              brake_stage_flag=1;
            }
          }
          
          if(brake_stage_flag==0)//飞控不处于刹车阶段时,速度期望来源于位置控制器输出
          {
            //位置反馈，来源于当前惯导的位置估计
            maple_ctrl.latitude_position_ctrl.feedback=ins.position[NORTH];
            maple_ctrl.longitude_position_ctrl.feedback=ins.position[EAST];
            //导航坐标系下E、N方向上位置偏差
            earth_frame_pos_err[NORTH]=maple_ctrl.latitude_position_ctrl.expect-maple_ctrl.latitude_position_ctrl.feedback;
            earth_frame_pos_err[EAST] =maple_ctrl.longitude_position_ctrl.expect-maple_ctrl.longitude_position_ctrl.feedback;
            
            //导航坐标系下机体Pitch、Roll方向上位置偏差			
            from_enu_to_body_frame(earth_frame_pos_err[EAST],earth_frame_pos_err[NORTH],&body_frame_pos_err[_RIGHT],&body_frame_pos_err[_FORWARD]);
            
            //导航坐标系下机体Pitch、Roll方向上期望刹车速度，这里为单比例运算不调用PID_Control()函数
            body_frame_pos_err[_FORWARD]=constrain_float(body_frame_pos_err[_FORWARD],-maple_ctrl.latitude_position_ctrl.err_max, maple_ctrl.latitude_position_ctrl.err_max); //位置偏差限幅，单位cm
            body_frame_pos_err[_RIGHT]  =constrain_float(body_frame_pos_err[_RIGHT] ,-maple_ctrl.longitude_position_ctrl.err_max,maple_ctrl.longitude_position_ctrl.err_max);//位置偏差限幅，单位cm
            //位置控制器比例运算，直接计算得到速度期望值
            body_frame_brake_speed[_FORWARD]=maple_ctrl.latitude_position_ctrl.kp*body_frame_pos_err[_FORWARD];
            body_frame_brake_speed[_RIGHT] =maple_ctrl.longitude_position_ctrl.kp*body_frame_pos_err[_RIGHT];
            
            
            body_frame_brake_speed[_FORWARD]=constrain_float(body_frame_brake_speed[_FORWARD],-max_cruising_speed, max_cruising_speed);//位置偏差限幅，单位cm
            body_frame_brake_speed[_RIGHT]	=constrain_float(body_frame_brake_speed[_RIGHT] ,-max_cruising_speed, max_cruising_speed);//位置偏差限幅，单位cm
            
            //更新速度控制器期望
            maple_ctrl.latitude_speed_ctrl.expect =body_frame_brake_speed[_FORWARD];
            maple_ctrl.longitude_speed_ctrl.expect=body_frame_brake_speed[_RIGHT];						
          }
        }					
      }
      else//水平方向遥控量有动作，直接将遥控器动作位映射成期望运动速度
      {
        maple_ctrl.latitude_speed_ctrl.expect =flight_speed_remap(-rc_data.rc_rpyt[PITCH],Pit_Rol_Max,Max_Horvel);
        maple_ctrl.longitude_speed_ctrl.expect=flight_speed_remap(	rc_data.rc_rpyt[ROLL] ,Pit_Rol_Max,Max_Horvel);
        
        maple_ctrl.latitude_position_ctrl.expect=0;
        maple_ctrl.longitude_position_ctrl.expect=0;
        brake_stage_flag=0;
        horizontal_position_fixed=1;
      }		
      vel_cnt++;
      if(vel_cnt>=VEL_CTRL_CNT)
      {
        vel_cnt=0;
        //下面获取导航系下沿飞行器俯仰、横滚方向的速度反馈量，
        //需要将惯性导航E、N方向的速度估计旋转到以载体俯仰、横滚水平面投影的游动直角坐标系				
        body_frame_speed_feedback[_RIGHT]  =body_frame_speed_current[_RIGHT];
        body_frame_speed_feedback[_FORWARD]=body_frame_speed_current[_FORWARD];	
        
        //沿载体游动直角坐标系下的速度反馈量
        maple_ctrl.latitude_speed_ctrl.feedback =body_frame_speed_feedback[_FORWARD];//机头Pitch水平投影方向，Y轴正向
        maple_ctrl.longitude_speed_ctrl.feedback=body_frame_speed_feedback[_RIGHT]; //横滚Roll水平投影方向，X轴正向
        //沿载体方向速度控制器
        pid_ctrl_general(&maple_ctrl.latitude_speed_ctrl,VEL_CTRL_CNT*PERIOD_UNIT_MIN/1000.0f);
        pid_ctrl_general(&maple_ctrl.longitude_speed_ctrl,VEL_CTRL_CNT*PERIOD_UNIT_MIN/1000.0f);
        
        //下面将速度控制器输出————期望水平运动加速度直接映射到期望倾角，计算方法如下：
        accel_target.x=	-maple_ctrl.longitude_speed_ctrl.control_output;//横滚水平方向投影方向下的期望运动加速度
        accel_target.y=	-maple_ctrl.latitude_speed_ctrl.control_output; //俯仰水平方向投影方向下的期望运动加速度
        desired_accel_transform_angle(accel_target,&angle_target);//期望运动加速度转期望姿态倾角
        //水平（俯仰、横滚）姿态期望，来源于位置-速度控制器输出，偏航期望仍然来源于遥控器给定
        maplepilot.roll_outer_control_output =angle_target.x;
        maplepilot.pitch_outer_control_output=angle_target.y;
        //maplepilot.yaw_outer_control_output	=rc_data.rc_rpyt[YAW];
      }			
    }
    break;
  case POSHOLD_FULL_AUTO_POS_CTRL:
    {
      Location target;
      vector2f location_delta={0,0};
      float earth_frame_to_xyz[3]={0,0,0};
      if(target_update==1)//通过目标点的经纬度计算，得到目标点距离home点的正北、正东方向的偏移距离
      {
        target.lng=target_lng;				//更新目标经纬度
        target.lat=target_lat;
        location_delta=location_diff(gps_home_point,target);//根据当前GPS定位信息与Home点位置信息计算正北、正东方向位置偏移
        /***********************************************************************************
        明确下导航系方向，这里正北、正东为正方向:
        沿着正东，经度增加,当无人机相对home点，往正东向移动时，此时GPS_Present.lng>GPS_Home.lng，得到的location_delta.x大于0;
        沿着正北，纬度增加,当无人机相对home点，往正北向移动时，此时GPS_Present.lat>GPS_Home.lat，得到的location_delta.y大于0;
        ******************************************************************************/
        location_delta.x*=100.0f;//沿地理坐标系，正北方向位置偏移,单位为CM
        location_delta.y*=100.0f;//沿地理坐标系，正东方向位置偏移,单位为CM
        earth_frame_to_xyz[EAST] = location_delta.x;//地理系下相对Home点正东位置偏移，单位为CM
        earth_frame_to_xyz[NORTH]= location_delta.y;//地理系下相对Home点正北位置偏移，单位为CM
      }
      
      pos_cnt++;
      if(pos_cnt>=POS_CTRL_CNT)
      {
        pos_cnt=0;
        if(target_update==1)//更新目标位置
        {
          maple_ctrl.latitude_position_ctrl.expect = earth_frame_to_xyz[NORTH];
          maple_ctrl.longitude_position_ctrl.expect= earth_frame_to_xyz[EAST];
          target_update=0;
        }		
        
        //位置反馈，来源于当前惯导的位置估计
        maple_ctrl.latitude_position_ctrl.feedback =ins.position[NORTH];
        maple_ctrl.longitude_position_ctrl.feedback=ins.position[EAST];
        //导航坐标系下E、N方向上位置偏差
        earth_frame_pos_err[NORTH]=maple_ctrl.latitude_position_ctrl.expect-maple_ctrl.latitude_position_ctrl.feedback;
        earth_frame_pos_err[EAST] =maple_ctrl.longitude_position_ctrl.expect-maple_ctrl.longitude_position_ctrl.feedback;
        
        //导航坐标系下机体Pitch、Roll方向上位置偏差			
        from_enu_to_body_frame(earth_frame_pos_err[EAST],earth_frame_pos_err[NORTH],&body_frame_pos_err[_RIGHT],&body_frame_pos_err[_FORWARD]);
        
        //导航坐标系下机体Pitch、Roll方向上期望刹车速度，这里为单比例运算不调用PID_Control()函数
        body_frame_pos_err[_FORWARD]=constrain_float(body_frame_pos_err[_FORWARD],-maple_ctrl.latitude_position_ctrl.err_max, maple_ctrl.latitude_position_ctrl.err_max); //位置偏差限幅，单位cm
        body_frame_pos_err[_RIGHT]  =constrain_float(body_frame_pos_err[_RIGHT] ,-maple_ctrl.longitude_position_ctrl.err_max,maple_ctrl.longitude_position_ctrl.err_max);//位置偏差限幅，单位cm
        //位置控制器比例运算，直接计算得到速度期望值
        body_frame_brake_speed[_FORWARD]=maple_ctrl.latitude_position_ctrl.kp*body_frame_pos_err[_FORWARD];
        body_frame_brake_speed[_RIGHT] =maple_ctrl.longitude_position_ctrl.kp*body_frame_pos_err[_RIGHT];
        
        
        body_frame_brake_speed[_FORWARD]=constrain_float(body_frame_brake_speed[_FORWARD],-max_cruising_speed, max_cruising_speed);//位置偏差限幅，单位cm
        body_frame_brake_speed[_RIGHT]	=constrain_float(body_frame_brake_speed[_RIGHT] ,-max_cruising_speed, max_cruising_speed);//位置偏差限幅，单位cm
        
        //更新速度控制器期望
        maple_ctrl.latitude_speed_ctrl.expect =body_frame_brake_speed[_FORWARD];
        maple_ctrl.longitude_speed_ctrl.expect=body_frame_brake_speed[_RIGHT];
      }
      
      vel_cnt++;
      if(vel_cnt>=VEL_CTRL_CNT)
      {
        vel_cnt=0;
        //下面获取导航系下沿飞行器俯仰、横滚方向的速度反馈量，
        //需要将惯性导航E、N方向的速度估计旋转到以载体俯仰、横滚水平面投影的游动直角坐标系				
        body_frame_speed_feedback[_RIGHT]  =body_frame_speed_current[_RIGHT];
        body_frame_speed_feedback[_FORWARD]=body_frame_speed_current[_FORWARD];	
        
        //沿载体游动直角坐标系下的速度反馈量
        maple_ctrl.latitude_speed_ctrl.feedback =body_frame_speed_feedback[_FORWARD];//机头Pitch水平投影方向，Y轴正向
        maple_ctrl.longitude_speed_ctrl.feedback=body_frame_speed_feedback[_RIGHT]; //横滚Roll水平投影方向，X轴正向
        //沿载体方向速度控制器
        pid_ctrl_general(&maple_ctrl.latitude_speed_ctrl,VEL_CTRL_CNT*PERIOD_UNIT_MIN/1000.0f);
        pid_ctrl_general(&maple_ctrl.longitude_speed_ctrl,VEL_CTRL_CNT*PERIOD_UNIT_MIN/1000.0f);
        
        //下面将速度控制器输出————期望水平运动加速度直接映射到期望倾角，计算方法如下：
        accel_target.x=-maple_ctrl.longitude_speed_ctrl.control_output;//横滚水平方向投影方向下的期望运动加速度
        accel_target.y=-maple_ctrl.latitude_speed_ctrl.control_output;//俯仰水平方向投影方向下的期望运动加速度
        desired_accel_transform_angle(accel_target,&angle_target);//期望运动加速度转期望姿态倾角
        //水平（俯仰、横滚）姿态期望，来源于位置-速度控制器输出，偏航期望仍然来源于遥控器给定
        maplepilot.roll_outer_control_output =angle_target.x;
        maplepilot.pitch_outer_control_output=angle_target.y;
        //maplepilot.yaw_outer_control_output	=rc_data.rc_rpyt[YAW];
        
      }			
    }break;	
  }
}













//光流定位控制
bool opticalflow_fix(void)
{
  bool fix=false;
  switch(maplepilot.indoor_position_sensor)
  {
  case RPISLAM:
    {
      if(tofsense_data.valid==1&&current_state.valid==1)	fix=true;
    }
    break;
  case OPTICALFLOW0:
  default:
    {
      if(tofsense_data.valid==1&&opt_data.valid==1)	fix=true;
    }
    break;		
  }
  return fix;
}

void opticalflow_speed_ctrl(vector2f target)
{
  if(opticalflow_fix()==false)//定位状态未锁定，姿态期望直接来源于遥控器给定
  {
    maplepilot.roll_outer_control_output	=rc_data.rc_rpyt[ROLL];
    maplepilot.pitch_outer_control_output=rc_data.rc_rpyt[PITCH];
    return ;
  }
  
  //更新期望速度
  maple_ctrl.optical_speed_ctrl_x.expect=target.x;
  maple_ctrl.optical_speed_ctrl_y.expect=target.y;
  //更新反馈速度
  maple_ctrl.optical_speed_ctrl_x.feedback=ins.opt.speed_ctrl.x;
  maple_ctrl.optical_speed_ctrl_y.feedback=ins.opt.speed_ctrl.y;
  
  static uint16_t _cnt=0;_cnt++;	
  if(_cnt>=2)
  {
    pid_ctrl_general(&maple_ctrl.optical_speed_ctrl_x,0.01f);
    pid_ctrl_general(&maple_ctrl.optical_speed_ctrl_y,0.01f);
    
    accel_target.x=-maple_ctrl.optical_speed_ctrl_x.control_output;
    accel_target.y=-maple_ctrl.optical_speed_ctrl_y.control_output;
    
    desired_accel_transform_angle(accel_target,&angle_target);//期望运动加速度转期望姿态倾角
    _cnt=0;
  }
  maplepilot.roll_outer_control_output = angle_target.x;
  maplepilot.pitch_outer_control_output= angle_target.y;
}


void opticalflow_position_ctrl(uint8_t force_brake_flag)
{	
  static uint8_t recode_poshlod=1;
  static uint16_t _cnt=0;_cnt++;
  if(opticalflow_fix()==false)//定位状态未锁定，姿态期望直接来源于遥控器给定
  {
    maplepilot.roll_outer_control_output	=rc_data.rc_rpyt[ROLL];
    maplepilot.pitch_outer_control_output=rc_data.rc_rpyt[PITCH];
    return ;
  }
  
  static systime poshold_dt;
  get_systime(&poshold_dt);
  //如果本函数运行周期大于控制周期10倍
  if(0.001f*poshold_dt.period>=10*min_ctrl_dt)
  {
    //情况1:初次从其它模式切入本模式
    //情况2:系统调度超时，在系统设计合理情况下，本情况不可能发生
    recode_poshlod=1;
  }
  
  if(_cnt>=4)
  {
    _cnt=0;
    if(rc_data.rc_rpyt[ROLL]==0 && rc_data.rc_rpyt[PITCH]==0)//水平方向遥控量无任何动作
    {
      if(recode_poshlod==1||force_brake_flag==1)//更新期望悬停点
      {
        maple_ctrl.optical_position_ctrl_x.expect=ins.opt.position_ctrl.x;
        maple_ctrl.optical_position_ctrl_y.expect=ins.opt.position_ctrl.y;
        recode_poshlod=0;				
      }
      //位置反馈
      maple_ctrl.optical_position_ctrl_x.feedback=ins.opt.position_ctrl.x;
      maple_ctrl.optical_position_ctrl_y.feedback=ins.opt.position_ctrl.y;	
      pid_ctrl_general(&maple_ctrl.optical_position_ctrl_x,0.02f);
      pid_ctrl_general(&maple_ctrl.optical_position_ctrl_y,0.02f);
    }
    else
    {
      //期望速度直接来源于遥感映射值
      maple_ctrl.optical_position_ctrl_x.control_output=flight_speed_remap( rc_data.rc_rpyt[ROLL] ,Pit_Rol_Max,Max_Opticalflow_Speed);
      maple_ctrl.optical_position_ctrl_y.control_output=flight_speed_remap(-rc_data.rc_rpyt[PITCH],Pit_Rol_Max,Max_Opticalflow_Speed);		
      recode_poshlod=1;					
    }
  }
  speed_target.x=maple_ctrl.optical_position_ctrl_x.control_output;//横滚水平投影方向下的期望运动速度
  speed_target.y=maple_ctrl.optical_position_ctrl_y.control_output;//俯仰水平投影方向下的期望运动速度
  opticalflow_speed_ctrl(speed_target);
}

void vioslam_position_ctrl(uint8_t force_brake_flag) 
{	
  static uint8_t recode_poshlod=1;
  static uint16_t _cnt=0;_cnt++;
  if(opticalflow_fix()==false)//定位状态未锁定，姿态期望直接来源于遥控器给定
  {
    maplepilot.roll_outer_control_output	=rc_data.rc_rpyt[ROLL];
    maplepilot.pitch_outer_control_output=rc_data.rc_rpyt[PITCH];
    return ;
  }
  
  static systime poshold_dt;
  get_systime(&poshold_dt);
  //如果本函数运行周期大于控制周期10倍
  if(0.001f*poshold_dt.period>=10*min_ctrl_dt)
  {
    //情况1:初次从其它模式切入本模式
    //情况2:系统调度超时，在系统设计合理情况下，本情况不可能发生
    recode_poshlod=1;
  }
  
  if(_cnt>=4)
  {
    _cnt=0;
    if(rc_data.rc_rpyt[ROLL]==0&&rc_data.rc_rpyt[PITCH]==0)//水平方向遥控量无任何动作
    {
      if(recode_poshlod==1||force_brake_flag==1)//更新期望悬停点
      {
        maple_ctrl.optical_position_ctrl_x.expect=ins.opt.position.x;//等效正东方向位置
        maple_ctrl.optical_position_ctrl_y.expect=ins.opt.position.y;//等效正北方向位置
        recode_poshlod=0;				
      }
      //位置反馈
      maple_ctrl.optical_position_ctrl_x.feedback=ins.opt.position.x;//等效正东方向位置
      maple_ctrl.optical_position_ctrl_y.feedback=ins.opt.position.y;//等效正北方向位置
      pid_ctrl_general(&maple_ctrl.optical_position_ctrl_x,0.02f);
      pid_ctrl_general(&maple_ctrl.optical_position_ctrl_y,0.02f);
      
      from_enu_to_body_frame(maple_ctrl.optical_position_ctrl_x.control_output,	\
        maple_ctrl.optical_position_ctrl_y.control_output,	\
          &speed_target.x,	\
            &speed_target.y);
    }
    else
    {
      //期望速度直接来源于遥感映射值
      speed_target.x=maple_ctrl.optical_position_ctrl_x.control_output=flight_speed_remap( rc_data.rc_rpyt[ROLL] ,Pit_Rol_Max,Max_Opticalflow_Speed);//横滚水平投影方向下的期望运动速度
      speed_target.y=maple_ctrl.optical_position_ctrl_y.control_output=flight_speed_remap(-rc_data.rc_rpyt[PITCH],Pit_Rol_Max,Max_Opticalflow_Speed);//俯仰水平投影方向下的期望运动速度		
      recode_poshlod=1;					
    }
  }
  opticalflow_speed_ctrl(speed_target);
}




/***************************************************************************************************/
void ngs_nav_ctrl_finish_predict(void)
{
  //判断是否到达目标航点位置
  if(ngs_nav_ctrl.ctrl_finish_flag==0)
  {
    if(ngs_nav_ctrl.cnt<10)//持续50ms满足
    {
      ngs_nav_ctrl.dis_cm=pythagorous3(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err,maple_ctrl.height_position_ctrl.err);
      if(ngs_nav_ctrl.dis_cm<=ngs_nav_ctrl.dis_limit_cm)	ngs_nav_ctrl.cnt++;
      else ngs_nav_ctrl.cnt/=2;
    }
    else
    {
      ngs_nav_ctrl.ctrl_finish_flag=1;
      ngs_nav_ctrl.cnt=0;
      nclink_send_check_flag[14]=1;//控制完毕后，返回应答给上位机
      send_check_back=3;//控制完毕后，返回应答给ROS
    }
  }
}

//适用于激光雷达SLAM定位条件下，普通光流（LC307、LC302）定位条件下无效
void Horizontal_Navigation(float x,float y,float z,uint8_t nav_mode,uint8_t frame_id)
{	
  if(nav_mode==RELATIVE_MODE)//相对模式
  {
    switch(frame_id)
    {
    case BODY_FRAME://机体坐标系下
      {
        float map_x=0,map_y=0;
        from_body_to_enu_frame(x,y,&map_x,&map_y);
        maple_ctrl.optical_position_ctrl_x.expect=ins.opt.position.x+map_x;
        maple_ctrl.optical_position_ctrl_y.expect=ins.opt.position.y+map_y;
        maple_ctrl.height_position_ctrl.expect=ins.position_z+z;			
      }
      break;
    case MAP_FRAME://导航坐标系下
      {
        maple_ctrl.optical_position_ctrl_x.expect=ins.opt.position.x+x;
        maple_ctrl.optical_position_ctrl_y.expect=ins.opt.position.y+y;
        maple_ctrl.height_position_ctrl.expect=ins.position_z+z;
      }
      break;		
    }	
  }
  else if(nav_mode==GLOBAL_MODE)//全局模式
  {
    switch(frame_id)
    {
    case MAP_FRAME://导航坐标系下
      {
        maple_ctrl.optical_position_ctrl_x.expect=x;
        maple_ctrl.optical_position_ctrl_y.expect=y;
        maple_ctrl.height_position_ctrl.expect=z;
      }
      break;
    default://原地保持
      {
        maple_ctrl.optical_position_ctrl_x.expect=ins.opt.position.x;
        maple_ctrl.optical_position_ctrl_y.expect=ins.opt.position.y;
        maple_ctrl.height_position_ctrl.expect=ins.position_z;	
      }				
    }	
  }
}


void accurate_position_control(uint8_t force_brake_flag)
{
  /*****************************SDK位置控制开始****************************/						
  if(ngs_sdk.update_flag==true)
  {
    if(ngs_sdk.move_flag.sdk_front_flag==true||ngs_sdk.move_flag.sdk_behind_flag==true)//前进/后退
    {
      maple_ctrl.optical_position_ctrl_x.expect=ins.opt.position_ctrl.x;										//左右保持
      maple_ctrl.optical_position_ctrl_y.expect=ins.opt.position_ctrl.y+ngs_sdk.f_distance;	//前后调整		
      maple_ctrl.height_position_ctrl.expect=ins.position_z;	 				  							    //上下保持																
    }
    
    if(ngs_sdk.move_flag.sdk_left_flag==true||ngs_sdk.move_flag.sdk_right_flag==true)//向左/向右
    {
      maple_ctrl.optical_position_ctrl_x.expect=ins.opt.position_ctrl.x+ngs_sdk.f_distance;  //左右调整
      maple_ctrl.optical_position_ctrl_y.expect=ins.opt.position_ctrl.y;										 //前后保持			
      maple_ctrl.height_position_ctrl.expect=ins.position_z;			 	     									 //上下保持														
    }
    
    if(ngs_sdk.move_flag.sdk_up_flag==true||ngs_sdk.move_flag.sdk_down_flag==true)//上升/下降
    {
      maple_ctrl.optical_position_ctrl_x.expect=ins.opt.position_ctrl.x;					 //前后保持
      maple_ctrl.optical_position_ctrl_y.expect=ins.opt.position_ctrl.y;					 //左右保持
      maple_ctrl.height_position_ctrl.expect=ins.position_z+ngs_sdk.f_distance; //上下调整										
    }
    ngs_sdk.move_flag.sdk_front_flag=false;
    ngs_sdk.move_flag.sdk_behind_flag=false;
    ngs_sdk.move_flag.sdk_left_flag=false;
    ngs_sdk.move_flag.sdk_right_flag=false;
    ngs_sdk.move_flag.sdk_up_flag=false;
    ngs_sdk.move_flag.sdk_down_flag=false;							
    ngs_sdk.update_flag=false;
  }
  
  if(ngs_nav_ctrl.update_flag==1)//地面站/ROS发送串口控制指令
  {
    Horizontal_Navigation(ngs_nav_ctrl.x,
                          ngs_nav_ctrl.y,
                          ngs_nav_ctrl.z,
                          ngs_nav_ctrl.nav_mode,
                          ngs_nav_ctrl.frame_id);
    ngs_nav_ctrl.update_flag=0;
    ngs_nav_ctrl.ctrl_finish_flag=0;
    ngs_nav_ctrl.cnt=0;
  }
  
  switch(ngs_nav_ctrl.nav_mode)
  {
  case RELATIVE_MODE:
  case GLOBAL_MODE://导航位置控制
    {
      vioslam_position_ctrl(force_brake_flag);
      ngs_nav_ctrl_finish_predict();//导航位置控制结束判定	
    }
    break;
  case CMD_VEL_MODE://速度控制模式
    {
      if(ngs_nav_ctrl.x!=0||ngs_nav_ctrl.y!=0)//当水平速度不为0时，速度控制才有效
      {	
        vector2f target;		
        target.x=flight_speed_remap( rc_data.rc_rpyt[ROLL] ,Pit_Rol_Max,Max_Opticalflow_Speed);//横滚水平投影方向下的期望运动速度
        target.y=flight_speed_remap(-rc_data.rc_rpyt[PITCH],Pit_Rol_Max,Max_Opticalflow_Speed);//俯仰水平投影方向下的期望运动速度	
        
        if(rc_data.rc_rpyt[ROLL]==0&&rc_data.rc_rpyt[PITCH]==0)//水平方向遥控量无任何动作
        {
          if(ngs_nav_ctrl.cmd_vel_update==1)//速度控制执行时间控制
          {
            if(ngs_nav_ctrl.cmd_vel_during_cnt>0)
            {
              ngs_nav_ctrl.cmd_vel_during_cnt=ngs_nav_ctrl.cmd_vel_during_cnt-1;
              vector2f target_tmp;
              target_tmp.x=ngs_nav_ctrl.cmd_vel_x;
              target_tmp.y=ngs_nav_ctrl.cmd_vel_y;
              opticalflow_speed_ctrl(target_tmp);
              ngs_nav_ctrl.cmd_vel_suspend_flag=1;
            }
            else	
            {
              ngs_nav_ctrl.cmd_vel_update=0;//速度控制完毕
              ngs_nav_ctrl.nav_mode=TRANSITION_MODE;//控制完毕后，进入过渡模式
              ngs_nav_ctrl.cmd_vel_suspend_flag=0;//正常中止
              nclink_send_check_flag[14]=1;//控制完毕后，返回应答给上位机
              send_check_back=3;//控制完毕后，返回应答给ROS
            }						
          }
          else opticalflow_speed_ctrl(target);
        }
        else 
        {
          opticalflow_speed_ctrl(target);
          //存在手动打杆操作，同样视作任务终止
          ngs_nav_ctrl.cmd_vel_update=0;//只要存在遥控器打杆操作，强制结束本次速度控制
          ngs_nav_ctrl.nav_mode=TRANSITION_MODE;//控制完毕后，进入过渡模式
          ngs_nav_ctrl.cmd_vel_during_cnt=0;
          ngs_nav_ctrl.cmd_vel_suspend_flag=0;//视同为正常中止
        }
      }
      else//否则水平仍然采用位置控制，只响应偏航控制
      {
        if(ngs_nav_ctrl.cmd_vel_update==1)//速度控制执行时间控制
        {
          if(ngs_nav_ctrl.cmd_vel_during_cnt>0)
          {
            ngs_nav_ctrl.cmd_vel_during_cnt=ngs_nav_ctrl.cmd_vel_during_cnt-1;
            //程序在此空转，实际不执行具体任务
            //只在外部控制中，响应偏航角速度控制
            //待计数器归0后，退出速度控制模式
          }
          else	
          {
            ngs_nav_ctrl.cmd_vel_update=0;//速度控制完毕
            ngs_nav_ctrl.nav_mode=TRANSITION_MODE+1;//控制完毕后，进入过渡的下一模式，本任务全程没有刷新水平悬停点
            nclink_send_check_flag[14]=1;//控制完毕后，返回应答给上位机
            send_check_back=3;//控制完毕后，返回应答给ROS
            ngs_nav_ctrl.cmd_vel_suspend_flag=0;//正常中止
          }						
        }
        
        if(ngs_nav_ctrl.cmd_vel_suspend_flag==1)//因后续指令被覆盖，提前终止上条指令的速度控制时，再次进入后会首选刷新悬停点
        {
          vioslam_position_ctrl(1);
          ngs_nav_ctrl.cmd_vel_suspend_flag=0;
        }
        else vioslam_position_ctrl(force_brake_flag);
        
        //存在手动打杆操作，同样视作任务终止
        if(rc_data.rc_rpyt[ROLL]!=0&&rc_data.rc_rpyt[PITCH]!=0)
        {
          ngs_nav_ctrl.cmd_vel_update=0;//速度控制完毕
          ngs_nav_ctrl.nav_mode=TRANSITION_MODE+1;//控制完毕后，进入过渡的下一模式，本任务全程没有刷新水平悬停点
          nclink_send_check_flag[14]=1;//控制完毕后，返回应答给上位机
          send_check_back=3;//控制完毕后，返回应答给ROS
          ngs_nav_ctrl.cmd_vel_suspend_flag=0;//视同为正常中止					
        }
      }
    }
    break;
  case TRANSITION_MODE://安排过渡模式，只会执行一次，用于刷新悬停位置
    {
      vioslam_position_ctrl(1);  //进入过渡模式后，刷新悬停位置期望
      ngs_nav_ctrl.nav_mode++;	 //自加后下次会进入普通光流定点模式
    }
    break;
  default:vioslam_position_ctrl(force_brake_flag);//普通光流控制
  }
}


void indoor_position_control(uint8_t force_brake_flag)
{
  if(maplepilot.indoor_position_sensor==RPISLAM)//定位模式设置为VIO/SLAM
  {
    switch(current_state.slam_sensor)
    {
    case NO_SLAM:
      {
        opticalflow_position_ctrl(force_brake_flag);
      }
      break;
    case RPLIDAR_SLAM:
    case T265_SLAM:
      {
        //vioslam_position_ctrl(force_brake_flag);
        accurate_position_control(force_brake_flag);				
      };
      break;
    default:			
      {
        opticalflow_position_ctrl(force_brake_flag);			
      }		
    }	
  }
  else//定位模式设置为普通光流
  {
    opticalflow_position_ctrl(force_brake_flag);
  }
}
