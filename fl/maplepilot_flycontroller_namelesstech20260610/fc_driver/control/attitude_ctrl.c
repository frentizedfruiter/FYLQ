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
#include "schedule.h"
#include "filter.h"
#include "wp_math.h"
#include "pid.h"
#include "attitude_ctrl.h"


controller_output maplepilot;
nav_planner_cmd planner_cmd;



void angle_control_target(controller_output *_flight_output)//角度环控制
{
  //姿态角度期望
  maple_ctrl.pitch_angle_ctrl.expect=_flight_output->pitch_outer_control_output;
  maple_ctrl.roll_angle_ctrl.expect=_flight_output->roll_outer_control_output;
  
  //姿态角度反馈
  maple_ctrl.roll_angle_ctrl.feedback =(flymaple.rpy_fusion_deg[ROLL]-flymaple.rpy_angle_offset[ROLL]);
  maple_ctrl.pitch_angle_ctrl.feedback=(flymaple.rpy_fusion_deg[PITCH]-flymaple.rpy_angle_offset[PITCH]);
  //运行PID控制器
  pid_ctrl_general(&maple_ctrl.roll_angle_ctrl,min_ctrl_dt);  
  pid_ctrl_general(&maple_ctrl.pitch_angle_ctrl,min_ctrl_dt);
  /***************内环角速度期望****************/
  maple_ctrl.pitch_gyro_ctrl.expect=maple_ctrl.pitch_angle_ctrl.control_output;
  maple_ctrl.roll_gyro_ctrl.expect=maple_ctrl.roll_angle_ctrl.control_output;
  switch(_flight_output->yaw_ctrl_mode)
  {
  case ROTATE:
    {		
      if(_flight_output->yaw_outer_control_output==0)//偏航杆置于中位
      {
        if(maple_ctrl.yaw_angle_ctrl.expect==0)//初次切回中
        {
          maple_ctrl.yaw_angle_ctrl.expect=flymaple.rpy_fusion_deg[YAW];
        }
        maple_ctrl.yaw_angle_ctrl.feedback=flymaple.rpy_fusion_deg[YAW];//偏航角反馈
        pid_ctrl_yaw(&maple_ctrl.yaw_angle_ctrl,min_ctrl_dt);//偏航角度控制
        maple_ctrl.yaw_gyro_ctrl.expect=maple_ctrl.yaw_angle_ctrl.control_output;//偏航角速度环期望，来源于偏航角度控制器输出
      }
      else//波动偏航方向杆后，只进行内环角速度控制
      {
        maple_ctrl.yaw_angle_ctrl.expect=0;//偏航角期望给0,不进行角度控制
        maple_ctrl.yaw_gyro_ctrl.expect=_flight_output->yaw_outer_control_output;//偏航角速度环期望，直接来源于遥控器打杆量
      }
    }
    break;
  case AZIMUTH://绝对航向角度
    {
      if(_flight_output->yaw_ctrl_start==1)//更新偏航角度期望
      {
        float yaw_tmp=_flight_output->yaw_outer_control_output;
        if(yaw_tmp<0) 	yaw_tmp+=360;
        if(yaw_tmp>360) yaw_tmp-=360;
        maple_ctrl.yaw_angle_ctrl.expect=yaw_tmp;
        _flight_output->yaw_ctrl_start=0;
        _flight_output->yaw_ctrl_cnt=0;
        _flight_output->yaw_ctrl_end=0;
        _flight_output->yaw_ctrl_response=1;
      }
      
      if(_flight_output->yaw_ctrl_end==0)//判断偏航角是否控制完毕
      {
        if(FastAbs(maple_ctrl.yaw_angle_ctrl.err)<3.0f) _flight_output->yaw_ctrl_cnt++;
        else _flight_output->yaw_ctrl_cnt/=2;
        
        if(_flight_output->yaw_ctrl_cnt>=200)  _flight_output->yaw_ctrl_end=1;
      }			
      
      maple_ctrl.yaw_angle_ctrl.feedback=flymaple.rpy_fusion_deg[YAW];//偏航角反馈
      pid_ctrl_yaw(&maple_ctrl.yaw_angle_ctrl,min_ctrl_dt);//偏航角度控制
      //对最大偏航角速度进行限制
      float tmp=constrain_float(maple_ctrl.yaw_angle_ctrl.control_output,-YAW_GYRO_CTRL_MAX,YAW_GYRO_CTRL_MAX);
      maple_ctrl.yaw_gyro_ctrl.expect=tmp;//偏航角速度环期望，来源于偏航角度控制器输出
    }
    break;
  case CLOCKWISE://顺时针————相对给定时刻的航向角度
    {
      if(_flight_output->yaw_ctrl_start==1)//更新偏航角度期望
      {
        float yaw_tmp=flymaple.rpy_fusion_deg[YAW]-_flight_output->yaw_outer_control_output;
        if(yaw_tmp<0) 	yaw_tmp+=360;
        if(yaw_tmp>360) yaw_tmp-=360;
        maple_ctrl.yaw_angle_ctrl.expect=yaw_tmp;
        _flight_output->yaw_ctrl_start=0;
        _flight_output->yaw_ctrl_cnt=0;
        _flight_output->yaw_ctrl_end=0;
        _flight_output->yaw_ctrl_response=1;
      }
      
      if(_flight_output->yaw_ctrl_end==0)//判断偏航角是否控制完毕
      {
        if(FastAbs(maple_ctrl.yaw_angle_ctrl.err)<3.0f) _flight_output->yaw_ctrl_cnt++;
        else _flight_output->yaw_ctrl_cnt/=2;
        
        if(_flight_output->yaw_ctrl_cnt>=200)  _flight_output->yaw_ctrl_end=1;
      }
      
      maple_ctrl.yaw_angle_ctrl.feedback=flymaple.rpy_fusion_deg[YAW];//偏航角反馈
      pid_ctrl_yaw(&maple_ctrl.yaw_angle_ctrl,min_ctrl_dt);//偏航角度控制
      //对最大偏航角速度进行限制
      float tmp=constrain_float(maple_ctrl.yaw_angle_ctrl.control_output,-YAW_GYRO_CTRL_MAX,YAW_GYRO_CTRL_MAX);
      maple_ctrl.yaw_gyro_ctrl.expect=tmp;//偏航角速度环期望，来源于偏航角度控制器输出
    }
    break;
  case ANTI_CLOCKWISE://逆时针——相对给定时刻的航向角度
    {
      if(_flight_output->yaw_ctrl_start==1)//更新偏航角度期望
      {
        float yaw_tmp=flymaple.rpy_fusion_deg[YAW]+_flight_output->yaw_outer_control_output;
        if(yaw_tmp<0) 	yaw_tmp+=360;
        if(yaw_tmp>360) yaw_tmp-=360;
        maple_ctrl.yaw_angle_ctrl.expect=yaw_tmp;
        _flight_output->yaw_ctrl_start=0;
        _flight_output->yaw_ctrl_cnt=0;
        _flight_output->yaw_ctrl_end=0;
        _flight_output->yaw_ctrl_response=1;
      }
      
      if(_flight_output->yaw_ctrl_end==0)//判断偏航角是否控制完毕
      {
        if(FastAbs(maple_ctrl.yaw_angle_ctrl.err)<3.0f) _flight_output->yaw_ctrl_cnt++;
        else _flight_output->yaw_ctrl_cnt/=2;
        
        if(_flight_output->yaw_ctrl_cnt>=200)  _flight_output->yaw_ctrl_end=1;
      }
      
      maple_ctrl.yaw_angle_ctrl.feedback=flymaple.rpy_fusion_deg[YAW];//偏航角反馈
      pid_ctrl_yaw(&maple_ctrl.yaw_angle_ctrl,min_ctrl_dt);//偏航角度控制
      //对最大偏航角速度进行限制
      float tmp=constrain_float(maple_ctrl.yaw_angle_ctrl.control_output,-YAW_GYRO_CTRL_MAX,YAW_GYRO_CTRL_MAX);
      maple_ctrl.yaw_gyro_ctrl.expect=tmp;//偏航角速度环期望，来源于偏航角度控制器输出
    }
    break;
  case CLOCKWISE_TURN://以某一角速度顺时针旋转多长时间
    {
      uint32_t curr_time_ms=millis();
      
      if(_flight_output->yaw_ctrl_start==1)//更新偏航角度期望
      {
        //对最大偏航角速度进行限制
        float tmp=constrain_float(-_flight_output->yaw_outer_control_output,-YAW_GYRO_CTRL_MAX,YAW_GYRO_CTRL_MAX);
        maple_ctrl.yaw_gyro_ctrl.expect=tmp;//偏航角速度环期望，来源于偏航角度控制器输出
        _flight_output->yaw_ctrl_start=0;
        _flight_output->yaw_ctrl_cnt=0;
        _flight_output->yaw_ctrl_end=0;
        _flight_output->start_time_ms=curr_time_ms;//记录开始转动的时间	
        _flight_output->yaw_ctrl_response=1;								
      }
      
      if(_flight_output->yaw_ctrl_end==0)//判断偏航角是否控制完毕
      {
        uint32_t tmp=curr_time_ms-_flight_output->start_time_ms;
        if(tmp>=_flight_output->execution_time_ms)
        {					
          _flight_output->yaw_ctrl_end=1;
          //执行完毕后,
          //1、将偏航角速度期望给0,
          //2、停止旋转并锁定当前偏航角，需要退出CLOCKWISE_TURN模式，角度期望才会有效，因为此模式没有对偏航角度进行控制
          maple_ctrl.yaw_gyro_ctrl.expect=0;
          maple_ctrl.yaw_angle_ctrl.expect=flymaple.rpy_fusion_deg[YAW];
        }
      }
      //实时更新角度期望值,以便退出此模式时能锁定当前航向
      maple_ctrl.yaw_angle_ctrl.expect=flymaple.rpy_fusion_deg[YAW];//因为本模式下无航向角度控制,更新期望值并会不会影响到偏航角速度的控制
    }
    break;
  case ANTI_CLOCKWISE_TURN://以某一角速度逆时针旋转多长时间
    {
      uint32_t curr_time_ms=millis();
      
      if(_flight_output->yaw_ctrl_start==1)//更新偏航角度期望
      {
        //对最大偏航角速度进行限制
        float tmp=constrain_float(_flight_output->yaw_outer_control_output,-YAW_GYRO_CTRL_MAX,YAW_GYRO_CTRL_MAX);
        maple_ctrl.yaw_gyro_ctrl.expect=tmp;//偏航角速度环期望，来源于偏航角度控制器输出
        _flight_output->yaw_ctrl_start=0;
        _flight_output->yaw_ctrl_cnt=0;
        _flight_output->yaw_ctrl_end=0;
        _flight_output->start_time_ms=curr_time_ms;//记录开始转动的时间	
        _flight_output->yaw_ctrl_response=1;					
      }
      
      if(_flight_output->yaw_ctrl_end==0)//判断偏航角是否控制完毕
      {
        uint32_t tmp=curr_time_ms-_flight_output->start_time_ms;
        if(tmp>=_flight_output->execution_time_ms)
        {					
          _flight_output->yaw_ctrl_end=1;
          //执行完毕后,
          //1、将偏航角速度期望给0,
          //2、停止旋转并锁定当前偏航角，需要退出CLOCKWISE_TURN模式，角度期望才会有效，因为此模式没有对偏航角度进行控制
          maple_ctrl.yaw_gyro_ctrl.expect=0;
          maple_ctrl.yaw_angle_ctrl.expect=flymaple.rpy_fusion_deg[YAW];					
        }
      }
      //实时更新角度期望值,以便退出此模式时能锁定当前航向
      maple_ctrl.yaw_angle_ctrl.expect=flymaple.rpy_fusion_deg[YAW];//因为本模式下无航向角度控制,更新期望值并会不会影响到偏航角速度的控制			
    }
    break;
  }
  
  //自动复位
  if(_flight_output->yaw_ctrl_end==1)//结束标志位产生后，程序后台1.0S内无响应，复位按键状态
  {
    _flight_output->end_state_lock_time++;
    if(_flight_output->end_state_lock_time>=200)//1s
    {			
      _flight_output->yaw_ctrl_end=0;
      _flight_output->end_state_lock_time=0;
    }
  }
  
  if(_flight_output->yaw_ctrl_response==1)//结束标志位产生后，程序后台1.0S内无响应，复位按键状态
  {
    _flight_output->response_state_lock_time++;
    if(_flight_output->response_state_lock_time>=20)//100ms
    {			
      _flight_output->yaw_ctrl_response=0;
      _flight_output->response_state_lock_time=0;
    }
  }	
  
  
} 


void yaw_fault_check(void)
{
  static uint16_t _cnt=0;
  /*******偏航控制异常情况判断，即偏航控制量很大时，偏航角速度很小，如此时为强外力干扰、已着地等******************************/
  if(FastAbs(maple_ctrl.yaw_gyro_ctrl.control_output)>maple_ctrl.yaw_gyro_ctrl.control_output_limit/2//偏航控制输出相对较大
     &&FastAbs(flymaple.gyro_dps.z)<=30.0f)//偏航角速度相对很小
  {
    _cnt++;
    if(_cnt>=500) _cnt=500;
  }
  else _cnt/=2;//不满足，快速削减至0
  
  if(_cnt>=400)//持续5ms*400=2S,特殊处理
  {
    pid_integrate_reset(&maple_ctrl.yaw_gyro_ctrl); //清空偏航角速度控制的积分
    pid_integrate_reset(&maple_ctrl.yaw_angle_ctrl);//清空偏航角控制的积分
    maple_ctrl.yaw_angle_ctrl.expect=flymaple.rpy_fusion_deg[YAW];	 //将当前偏航角，作为期望偏航角
    _cnt=0;
  }
}



void gyro_control(void)//角速度环控制
{	
  //俯仰、横滚方向姿态内环角速度控制器采用PID控制器
  /***************内环角速度期望****************/
  maple_ctrl.pitch_gyro_ctrl.expect=maple_ctrl.pitch_angle_ctrl.control_output;
  maple_ctrl.roll_gyro_ctrl.expect=maple_ctrl.roll_angle_ctrl.control_output;
  /***************内环角速度反馈****************/
  maple_ctrl.pitch_gyro_ctrl.feedback=flymaple.gyro_dps.x;
  maple_ctrl.roll_gyro_ctrl.feedback =flymaple.gyro_dps.y;
  maple_ctrl.yaw_gyro_ctrl.feedback  =flymaple.gyro_dps.z;
  /***************内环角速度控制：微分参数动态调整****************/
  if(flymaple.player_level==0||flymaple.player_level==1)
  {
    pid_ctrl_div_lpf(&maple_ctrl.pitch_gyro_ctrl,min_ctrl_dt);
    pid_ctrl_div_lpf(&maple_ctrl.roll_gyro_ctrl,min_ctrl_dt);
    pid_ctrl_div_lpf(&maple_ctrl.yaw_gyro_ctrl,min_ctrl_dt);	  
  }
  else if(flymaple.player_level==2)
  {
    pid_ctrl_rpy_gyro(&maple_ctrl.pitch_gyro_ctrl ,min_ctrl_dt,incomplete_diff,first_order_lpf);//微分先行+一阶低通
    pid_ctrl_rpy_gyro(&maple_ctrl.roll_gyro_ctrl  ,min_ctrl_dt,incomplete_diff,first_order_lpf);
    pid_ctrl_rpy_gyro(&maple_ctrl.yaw_gyro_ctrl   ,min_ctrl_dt,incomplete_diff,first_order_lpf);	
  }
  else if(flymaple.player_level==3)
  {
    pid_ctrl_rpy_gyro(&maple_ctrl.pitch_gyro_ctrl ,min_ctrl_dt,incomplete_diff,second_order_lpf);//微分先行+二阶低通
    pid_ctrl_rpy_gyro(&maple_ctrl.roll_gyro_ctrl  ,min_ctrl_dt,incomplete_diff,second_order_lpf);
    pid_ctrl_rpy_gyro(&maple_ctrl.yaw_gyro_ctrl   ,min_ctrl_dt,incomplete_diff,second_order_lpf);	
  }
  else if(flymaple.player_level==4)
  {
    pid_ctrl_rpy_gyro(&maple_ctrl.pitch_gyro_ctrl ,min_ctrl_dt,direct_diff,noneed_lpf);//直接微分+无低通
    pid_ctrl_rpy_gyro(&maple_ctrl.roll_gyro_ctrl  ,min_ctrl_dt,direct_diff,noneed_lpf);
    pid_ctrl_rpy_gyro(&maple_ctrl.yaw_gyro_ctrl   ,min_ctrl_dt,direct_diff,noneed_lpf);		
  }
  /*******偏航控制异常处理******************************/
  yaw_fault_check();
}


/************姿态环控制器：角度+角速度****************/
void attitude_control(void)
{ 
  angle_control_target(&maplepilot);//角度控制	
  gyro_control();//角速度控制
  imu_temperature_ctrl();//IMU恒温控制
}

//将NED坐标系下的期望加速度转换成机体坐标系下的倾斜角度(pitch,roll)
void desired_accel_transform_angle(vector2f _accel_target,vector2f *target_angle)
{
  float accel_right,accel_forward;
  float lean_angle_max = 30.0f;	
  accel_right  =_accel_target.x;//cm/s^2
  accel_forward=_accel_target.y;//cm/s^2
  //update angle targets that will be passed to stabilize controller
  
  target_angle->x=RAD_TO_DEG*atanf(-accel_right/(GRAVITY_MSS*100));													//计算期望横滚角
  target_angle->y=RAD_TO_DEG*atanf( accel_forward*flymaple.cos_rpy[_ROL]/(GRAVITY_MSS*100));//计算期望俯仰角
  
  target_angle->x=constrain_float(target_angle->x,-lean_angle_max,lean_angle_max);//roll
  target_angle->y=constrain_float(target_angle->y,-lean_angle_max,lean_angle_max);//pitch
}

/*************************************************************************************/
void simulation_pwm_init(void)
{
  gpio_init(IMU1_HEATER_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
  gpio_init(IMU2_HEATER_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
  gpio_low(IMU1_HEATER_PIN);
  gpio_low(IMU2_HEATER_PIN);
}


#define Simulation_PWM_Period_MAX  100//100*1ms=0.1S
void simulation_pwm_output(uint16_t width)
{
  uint16_t static cnt=0;
  cnt++;
  if(cnt>=Simulation_PWM_Period_MAX)  cnt=0;
  if(cnt<=width&&flymaple.imu_data_offline_flag==0) 
  {
    gpio_high(IMU1_HEATER_PIN);
    gpio_high(IMU2_HEATER_PIN);
  }
  else 
  {
    gpio_low(IMU1_HEATER_PIN);
    gpio_low(IMU2_HEATER_PIN);
  }
}

void imu_temperature_ctrl(void)
{
  static uint16_t temperature_ctrl_cnt=0;
  temperature_ctrl_cnt++;
  if(temperature_ctrl_cnt>=10)//50ms
  {
    maple_ctrl.imu_temperature_ctrl.expect=50;
    maple_ctrl.imu_temperature_ctrl.feedback=flymaple.t;
    pid_ctrl_div_lpf(&maple_ctrl.imu_temperature_ctrl,0.05f);
    maple_ctrl.imu_temperature_ctrl.control_output=constrain_float(maple_ctrl.imu_temperature_ctrl.control_output,0,100);
    maplepilot.temperature_control_output=(uint16_t)(maple_ctrl.imu_temperature_ctrl.control_output);
    temperature_ctrl_cnt=0;
  }
  //TIM2->CCR3=(uint16_t)(maplepilot.temperature_control_output*24.99f);
  simulation_pwm_output(maplepilot.temperature_control_output);
  temperature_state_check();
}

uint8_t temperature_state_get(void)
{
  return (ABS(maple_ctrl.imu_temperature_ctrl.expect-flymaple.t))<=1.5f?1:0;
}

void temperature_state_check(void)
{
  static uint16_t _cnt=0;
  if(flymaple.imu_data_offline_flag==1)  flymaple.temperature_stable_flag=1;//没有温控的情况下直接置1
  if(temperature_state_get()==1){
    _cnt++;
    if(_cnt>=400) flymaple.temperature_stable_flag=1;
  }
  else  _cnt/=2;
}	

bool imu_temperature_stable(void)
{
  bool ok=false;
  if(flymaple.temperature_stable_flag==1)
    ok=true;
  return ok;
}
