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
#include "wp_math.h"
#include "attitude_ctrl.h"
#include "altitude_ctrl.h"
#include "position_ctrl.h"
#include "rc.h"
#include "control_output_mix.h"
#include "pid.h"
#include "maple_config.h"
#include "parameter_server.h"
#include "drv_pwm.h"
#include "land_ctrl.h"
#include "Developer_Mode.h"
#include "flymaple_sdk.h"
#include "Subtask_Demo.h"
#include "flymaple_ctrl.h"





void esc_ctrl_hz_init(void)
{
  if(Flight_Params.health[ESC_OUTPUT_FREQUENCY]==false)  maplepilot.esc_output_frequence=f_200hz;
  else maplepilot.esc_output_frequence=Flight_Params.parameter_table[ESC_OUTPUT_FREQUENCY];
}

void main_leading_control(void)
{
  if(maplepilot.init==0)
  {		
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output=0;
    maplepilot.pitch_outer_control_output=0;
    maplepilot.roll_outer_control_output=0;
    maplepilot.init=1;
    if(Flight_Params.health[OPTICAL_TYPE]==false)  maplepilot.indoor_position_sensor=OPTICALFLOW0;
    else maplepilot.indoor_position_sensor=Flight_Params.parameter_table[OPTICAL_TYPE];
  }
  /************************第五通道控制高度模式************************/		
  if(rc_data.height_mode==1)//高度纯手动模式
  {
    throttle_output=rc_data.thr;//油门直接来源于遥控器油门给定
    maplepilot.roll_outer_control_output =rc_data.rc_rpyt[RC_ROLL];
    maplepilot.pitch_outer_control_output=rc_data.rc_rpyt[RC_PITCH];
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output=rc_data.rc_rpyt[RC_YAW];
    return ;
  }
  /*********************以下模式均含有定高****************************/
  
  /*********************根据遥控器切换档位，飞控进入不同模式****************************/
  if(rc_data.gps_mode==4)//GPS定点模式
  {
    flight_position_control(POSHLOD_MANUAL_CTRL,NUL,NUL,NUL,NUL,NUL,NUL,0);;//位置控制
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output=rc_data.rc_rpyt[RC_YAW];
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制		
    return ;		
  }
  else if(rc_data.gps_mode==5)//一键返航/自动降落模式
  {
    land_run();
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
    return ;	
  }		
  
  if(rc_data.sdk_duty_mode==5)//用户SDK开发者自动飞行模式：水平+高度控制
  {
    Auto_Flight_Ctrl(&sdk1_mode_setup);//sdk模式
  }
  else if(rc_data.position_mode==1)//水平姿态自稳+高度控制
  {
    maplepilot.roll_outer_control_output =rc_data.rc_rpyt[RC_ROLL];
    maplepilot.pitch_outer_control_output=rc_data.rc_rpyt[RC_PITCH];
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];		
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    flight_subtask_reset();//sdk任务相关状态量的复位
    offboard_start_flag=0;
  }
  else if(rc_data.position_mode==5)//水平光流定点模式：水平位置+高度控制
  {
    indoor_position_control(0);//普通光流模式		
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    flight_subtask_reset();//sdk任务相关状态量的复位
    offboard_start_flag=0;
  }	
  else 
  {
    maplepilot.roll_outer_control_output =rc_data.rc_rpyt[RC_ROLL];
    maplepilot.pitch_outer_control_output=rc_data.rc_rpyt[RC_PITCH];	
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];		
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    flight_subtask_reset();//sdk任务相关状态量的复位
    offboard_start_flag=0;
  }
}


bool motor_idel_enable=false;
uint16_t motor_idel_cnt=0;
uint16_t motor_idel_value=0;
float motor_idel_period=2;//20
float motor_idel_time=0,motor_idel_last_time=0;
void flymaple_output(void)
{
  if(rc_data.lock_state==UNLOCK)//解锁状态
  {
    //控制器输出来源
    maplepilot.roll_control_output =maple_ctrl.roll_gyro_ctrl.control_output;
    maplepilot.pitch_control_output=maple_ctrl.pitch_gyro_ctrl.control_output;
    maplepilot.yaw_control_output  =maple_ctrl.yaw_gyro_ctrl.control_output;	
    if(rc_data.height_mode==1) maplepilot.throttle_control_output=throttle_angle_compensate(throttle_output);//油门输出来源于定高控制	
    else maplepilot.throttle_control_output=throttle_output;
    
    if(motor_idel_enable==true)//怠速启动过程
    {
      motor_idel_time=get_systime_ms();
      if(motor_idel_time-motor_idel_last_time>motor_idel_period)
      {
        motor_idel_last_time=motor_idel_time;
        motor_idel_cnt++;
      }	  
      motor_idel_value=THR_MIN_OUTPUT+motor_idel_cnt;	
      maplepilot.motor_output[MOTOR1]=maplepilot.motor_output[MOTOR2]=motor_idel_value;
      maplepilot.motor_output[MOTOR3]=maplepilot.motor_output[MOTOR4]=motor_idel_value;
      maplepilot.motor_output[MOTOR5]=maplepilot.motor_output[MOTOR6]=motor_idel_value;
      maplepilot.motor_output[MOTOR7]=maplepilot.motor_output[MOTOR8]=motor_idel_value;		
      
      if(motor_idel_cnt>=(THR_IDEL_OUTPUT-THR_MIN_OUTPUT))
      {
        motor_idel_enable=false;
        motor_idel_cnt=0;
        motor_idel_last_time=motor_idel_time=0;
      }
    }
    else
    {
      if(maplepilot.throttle_control_output>=THR_CONTROL_WORK)//油门控制量大于油门怠速值时,叠加油门+俯仰+横滚控制
      {
        Motor_Control_Rate_Pure(maplepilot.throttle_control_output,
                                maplepilot.roll_control_output,
                                maplepilot.pitch_control_output,
                                maplepilot.yaw_control_output,
                                maplepilot.motor_output);			
      }
      else//油门控制量小于油门怠速值时,只输出油门控制
      {
        Motor_Control_Rate_Pure(maplepilot.throttle_control_output,0,0,0,maplepilot.motor_output);	
        takeoff_ctrl_reset();//清积分
      }			
    }
    
    if(rc_data.rc_return_flag==0)//若已解锁但摇杆没有回中，则会一直清积分
    {
      takeoff_ctrl_reset();//清积分
    }		
    
    //解锁后油门最小输出值为怠速值
    for(uint16_t i=0;i<MOTOR_NUM;i++)
    {
      maplepilot.motor_output[i]=constrain_int16(maplepilot.motor_output[i],THR_IDEL_OUTPUT,THR_MAX_OUTPUT);
    }
  }
  else
  {
    maplepilot.roll_control_output =0;
    maplepilot.pitch_control_output=0;
    maplepilot.yaw_control_output  =0;	
    maplepilot.throttle_control_output=THR_MIN_OUTPUT;
    
    maplepilot.motor_output[MOTOR1]=maplepilot.motor_output[MOTOR2]=maplepilot.throttle_control_output;
    maplepilot.motor_output[MOTOR3]=maplepilot.motor_output[MOTOR4]=maplepilot.throttle_control_output;
    maplepilot.motor_output[MOTOR5]=maplepilot.motor_output[MOTOR6]=maplepilot.throttle_control_output;
    maplepilot.motor_output[MOTOR7]=maplepilot.motor_output[MOTOR8]=maplepilot.throttle_control_output;
    motor_idel_enable=true;
    motor_idel_cnt=0;
    motor_idel_last_time=motor_idel_time=0;	
    takeoff_ctrl_reset();//清积分
  }
  
  for(uint16_t i=0;i<MOTOR_NUM;i++)
  {
    maplepilot.motor_output[i]=constrain_int16(maplepilot.motor_output[i],THR_MIN_OUTPUT,THR_MAX_OUTPUT);
  }
  pwm_output(maplepilot.motor_output[0],
             maplepilot.motor_output[1],
             maplepilot.motor_output[2],
             maplepilot.motor_output[3]);
}

void flymaple_ctrl(void)
{
  /*************主导控制器******************/
  main_leading_control();
  landon_earth_check(rc_data.unwanted_lock_flag);//着陆条件自检
  if(get_landon_state()==1)  rc_data.lock_state=LOCK; //着陆检测
  /*************姿态环控制器*****************/
  attitude_control();
  flymaple_output();	
}


