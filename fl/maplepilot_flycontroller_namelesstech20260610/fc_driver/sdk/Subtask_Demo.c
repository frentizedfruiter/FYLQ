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


#include "attitude_ctrl.h"
#include "altitude_ctrl.h"
#include "nclink.h"
#include "reserved_serialport.h"
#include "drv_expand.h"
#include "rc.h"
#include "pid.h"
#include "drv_pwm.h"
#include "position_ctrl.h"
#include "flymaple_sdk.h"
#include "Subtask_Demo.h"


uint16_t flight_subtask_cnt[FLIGHT_SUBTASK_NUM]={0};//飞行任务子线程计数器，可以用于控制每个航点子线程的执行
uint32_t flight_global_cnt[FLIGHT_SUBTASK_NUM]={0}; //飞行任务子线全局计数器，可以结合位置偏差用于判断判断航点是否到达
uint32_t execute_time_ms[FLIGHT_SUBTASK_NUM]={0};		//飞行任务子线执行时间，可以用于设置某个子线程的执行时间
uint32_t flight_global_cnt2[FLIGHT_SUBTASK_NUM]={0};//飞行任务子线全局计数器2
#define flight_subtask_delta 5//5ms
vector3f base_position;//用于记录导航基准原点位置

void flight_subtask_reset(void)
{
  for(uint16_t i=0;i<FLIGHT_SUBTASK_NUM;i++)
  {
    flight_subtask_cnt[i]=0;
    execute_time_ms[i]=0;
    flight_global_cnt[i]=0;
    flight_global_cnt2[i]=0;
  }
}


//基本飞行保障必备子函数——定高、定点
void basic_auto_flight_support(void)
{
  indoor_position_control(0);//SLAM定点控制
  maplepilot.yaw_ctrl_mode=ROTATE;//偏航控制为手动模式
  maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
  flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
}



//基本飞行保障必备子函数——定高、定点、同时加入了偏航指令控制
void ros_flight_support(void)
{
  maplepilot.yaw_ctrl_mode=ROTATE;//偏航控制为手动模式
  if(rc_data.rc_rpyt[RC_YAW]==0)//无水平遥控量给定)
  {
    if(ngs_nav_ctrl.cmd_vel_update==1)	maplepilot.yaw_outer_control_output  =ngs_nav_ctrl.cmd_vel_angular_z;
    else maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];//作用期过后，控制权限交给遥控器
  }
  else
  {
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
    ngs_nav_ctrl.cmd_vel_update=0;//只要存在遥控器打杆操作，强制结束本次速度控制
    ngs_nav_ctrl.cmd_vel_during_cnt=0;		
  }		
  indoor_position_control(0);//SLAM定点控制
  flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
}


/***********************************************************************************************************/
//顺时针转动90度，完成后降落
void flight_subtask_1(void)
{
  static uint8_t n=0;
  if(flight_subtask_cnt[n]==0)
  {
    maplepilot.yaw_ctrl_mode=CLOCKWISE;
    maplepilot.yaw_ctrl_start=1;
    maplepilot.yaw_outer_control_output  =90;//顺时针90度	
    indoor_position_control(0);	
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    flight_subtask_cnt[n]=1;		
  }
  else if(flight_subtask_cnt[n]==1)
  {
    maplepilot.yaw_ctrl_mode=CLOCKWISE;
    maplepilot.yaw_outer_control_output  =0;
    indoor_position_control(0);	
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    if(maplepilot.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//执行完毕后，切换到下一阶段	
  }
  else if(flight_subtask_cnt[n]==2)
  {
    
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
    indoor_position_control(0);
    flight_altitude_control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//高度控制
  }
  else
  {
    basic_auto_flight_support();//基本飞行支持软件
  }
}





/***********************************************************************************************************/
//逆时针转动90度，完成后降落
void flight_subtask_2(void)
{
  static uint8_t n=1;
  if(flight_subtask_cnt[n]==0)
  {
    maplepilot.yaw_ctrl_mode=ANTI_CLOCKWISE;
    maplepilot.yaw_ctrl_start=1;
    maplepilot.yaw_outer_control_output  =90;//逆时针90度	
    indoor_position_control(0);	
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    flight_subtask_cnt[n]=1;		
  }
  else if(flight_subtask_cnt[n]==1)
  {
    maplepilot.yaw_ctrl_mode=ANTI_CLOCKWISE;
    maplepilot.yaw_outer_control_output  =0;
    indoor_position_control(0);	
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制		
    if(maplepilot.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//执行完毕后，切换到下一阶段		
  }
  else if(flight_subtask_cnt[n]==2)
  {
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
    indoor_position_control(0);
    flight_altitude_control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//高度控制
  }
  else
  {
    basic_auto_flight_support();//基本飞行支持软件
  }
}





/***********************************************************************************************************/
//以10deg/s的角速度顺时针转动10000ms，完成后降落
void flight_subtask_3(void)
{
  static uint8_t n=2;
  if(flight_subtask_cnt[n]==0)
  {
    
    maplepilot.yaw_ctrl_mode=CLOCKWISE_TURN;
    maplepilot.yaw_ctrl_start=1;
    maplepilot.yaw_outer_control_output  =10;//以10deg/s的角速度顺时针转动10000ms
    maplepilot.execution_time_ms=10000;//执行时间
    indoor_position_control(0);	
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    flight_subtask_cnt[n]=1;		
  }
  else if(flight_subtask_cnt[n]==1)
  {
    maplepilot.yaw_ctrl_mode=CLOCKWISE_TURN;
    maplepilot.yaw_outer_control_output  =0;
    indoor_position_control(0);	
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    if(maplepilot.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//执行完毕后，切换到下一阶段		
  }
  else if(flight_subtask_cnt[n]==2)
  {
    
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
    
    indoor_position_control(0);
    flight_altitude_control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//高度控制
  }
  else
  {
    basic_auto_flight_support();//基本飞行支持软件
  }
}





/***********************************************************************************************************/
//以10deg/s的角速度逆时针转动10000ms，完成后降落
void flight_subtask_4(void)
{
  static uint8_t n=3;
  if(flight_subtask_cnt[n]==0)
  {	
    maplepilot.yaw_ctrl_mode=ANTI_CLOCKWISE_TURN;
    maplepilot.yaw_ctrl_start=1;
    maplepilot.yaw_outer_control_output  =10;//以10deg/s的角速度顺时针转动10000ms
    maplepilot.execution_time_ms=10000;//执行时间
    indoor_position_control(0);	
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    flight_subtask_cnt[n]=1;		
  }
  else if(flight_subtask_cnt[n]==1)
  {
    maplepilot.yaw_ctrl_mode=ANTI_CLOCKWISE_TURN;
    maplepilot.yaw_outer_control_output  =0;
    indoor_position_control(0);	
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    if(maplepilot.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//执行完毕后，切换到下一阶段		
  }
  else if(flight_subtask_cnt[n]==2)
  {
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
    
    indoor_position_control(0);
    flight_altitude_control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//高度控制
  }
  else
  {
    basic_auto_flight_support();//基本飞行支持软件
  }
}






//本demo适用于激光雷达SLAM定位条件下，普通光流（LC307、LC302）定位条件下无效
//机体坐标系下相对位移
//右前上分别对应XYZ正方向
void flight_subtask_5(void)
{		
  static uint8_t n=4;
  if(flight_subtask_cnt[n]==0)
  {
    basic_auto_flight_support();//基本飞行支持软件
    flight_subtask_cnt[n]=1;
    execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
    //向前移动100cm
    Horizontal_Navigation(0,100,0,RELATIVE_MODE,BODY_FRAME);
  }
  else if(flight_subtask_cnt[n]==1)
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      flight_subtask_cnt[n]=2;
      execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
      //向右移动100cm
      Horizontal_Navigation(100,0,0,RELATIVE_MODE,BODY_FRAME);
    }
  }
  else if(flight_subtask_cnt[n]==2)
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      flight_subtask_cnt[n]=3;
      execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
      //向后移动100cm
      Horizontal_Navigation(0,-100,0,RELATIVE_MODE,BODY_FRAME);
    }
  }
  else if(flight_subtask_cnt[n]==3)
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      flight_subtask_cnt[n]=4;
      execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
      //向左移动100cm
      Horizontal_Navigation(-100,0,0,RELATIVE_MODE,BODY_FRAME);
    }
  }
  else if(flight_subtask_cnt[n]==4)
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      flight_subtask_cnt[n]=5;
      execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
      
      //以下还可以继续增加子任务，比如向上移动50cm
      //Horizontal_Navigation(0,0,50,RELATIVE_FRAME,MAP_FRAME);
    }
  }
  else
  {
    basic_auto_flight_support();//基本飞行支持软件
  }
}

//导航坐标系下相对位移
//东北天分别对应XYZ正方向
void flight_subtask_6(void)
{		
  static uint8_t n=5;
  if(flight_subtask_cnt[n]==0)
  {
    basic_auto_flight_support();//基本飞行支持软件
    flight_subtask_cnt[n]=1;
    execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
    //向前移动100cm
    Horizontal_Navigation(0,100,0,RELATIVE_MODE,MAP_FRAME);
  }
  else if(flight_subtask_cnt[n]==1)
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      flight_subtask_cnt[n]=2;
      execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
      //向右移动100cm
      Horizontal_Navigation(100,0,0,RELATIVE_MODE,MAP_FRAME);
    }
  }
  else if(flight_subtask_cnt[n]==2)
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      flight_subtask_cnt[n]=3;
      execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
      //向后移动100cm
      Horizontal_Navigation(0,-100,0,RELATIVE_MODE,MAP_FRAME);
    }
  }
  else if(flight_subtask_cnt[n]==3)
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      flight_subtask_cnt[n]=4;
      execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
      //向左移动100cm
      Horizontal_Navigation(-100,0,0,RELATIVE_MODE,MAP_FRAME);
    }
  }
  else if(flight_subtask_cnt[n]==4)
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      flight_subtask_cnt[n]=5;
      execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
      
      //以下还可以继续增加子任务，比如向上移动50cm
      //Horizontal_Navigation(0,0,50,RELATIVE_FRAME,MAP_FRAME);
    }
  }
  else
  {
    basic_auto_flight_support();//基本飞行支持软件
  }
}


//导航坐标系下，基于初始点的绝对坐标位移
void flight_subtask_7(void)
{		
  static uint8_t n=6;
  vector3f target_position;
  float x=0,y=0,z=0;
  if(flight_subtask_cnt[n]==0)
  {
    basic_auto_flight_support();//基本飞行支持软件
    //记录下初始起点位置，实际项目中可设置为某一基准原点
    base_position.x=ins.opt.position_ctrl.x;
    base_position.y=ins.opt.position_ctrl.y;
    base_position.z=ins.position_z;
    
    flight_subtask_cnt[n]=1;
    execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
    //向前移动100cm
    x=0;y=100;z=0;
    target_position.x=base_position.x+x;
    target_position.y=base_position.y+y;
    target_position.z=base_position.z+z;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
  }
  else if(flight_subtask_cnt[n]==1)
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      flight_subtask_cnt[n]=2;
      execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
      //向右移动100cm
      x=100;y=100;z=0;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=base_position.z+z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
    }
  }
  else if(flight_subtask_cnt[n]==2)
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      flight_subtask_cnt[n]=3;
      execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
      //向后移动100cm
      x=100;y=0;z=0;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=base_position.z+z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
    }
  }
  else if(flight_subtask_cnt[n]==3)
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      flight_subtask_cnt[n]=4;
      execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
      //向左移动100cm
      x=0;y=0;z=0;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=base_position.z+z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
    }
  }
  else if(flight_subtask_cnt[n]==4)
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      flight_subtask_cnt[n]=5;
      execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
      
      //以下还可以继续增加子任务，比如向上移动50cm
      //Horizontal_Navigation(0,0,50,RELATIVE_FRAME,MAP_FRAME);
    }
  }
  else
  {
    basic_auto_flight_support();//基本飞行支持软件
  }
}


//轨迹圆
#define Waypoint_Radius_CM     50 									//轨迹点半径，单位为cm   100
#define Waypoint_Num           60				 						//轨迹点数量						 60
//改变轨迹圆参数，一般情况下只需要调整半径Waypoint_Radius_CM即可
#define Waypoint_Angle_Delta   (360/Waypoint_Num)		//航点角度增量，单位为deg
#define Waypoint_Ideal_Speed	 20										//理想的巡航速度，用于推算航点发布时间，单位为cm/s
#define Waypoint_Time_Gap			 1000*(2*3.14*Waypoint_Radius_CM/Waypoint_Ideal_Speed)/Waypoint_Num		//航点发布时间间隔，单位为ms
float waypoint_x[Waypoint_Num+1],waypoint_y[Waypoint_Num+1];
void Circle_Waypoint_Generate(uint16_t num,float radius)
{
  for(uint16_t i=0;i<num+1;i++)
  {
    waypoint_x[i]=radius-radius*cos(Waypoint_Angle_Delta*i*DEG2RAD);
    waypoint_y[i]=radius*sin(Waypoint_Angle_Delta*i*DEG2RAD);
  }
}


void flight_subtask_8(void)
{		
  static uint8_t n=7;
  vector3f target_position;
  basic_auto_flight_support();//基本飞行支持软件
  if(flight_subtask_cnt[n]==0)
  {
    Circle_Waypoint_Generate(Waypoint_Num,Waypoint_Radius_CM);
    //记录下初始起点位置，实际项目中可设置为某一基准原点
    base_position.x=ins.opt.position_ctrl.x;
    base_position.y=ins.opt.position_ctrl.y;
    base_position.z=ins.position_z;
    
    execute_time_ms[n]=Waypoint_Time_Gap/flight_subtask_delta;//子任务执行时间
    target_position.x=base_position.x+waypoint_x[0];
    target_position.y=base_position.y+waypoint_y[0];
    target_position.z=base_position.z;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    flight_subtask_cnt[n]=1;
    laser_light_1.reset=1;
    laser_light_1.times=50000;//闪速1次
    laser_light_1.period=40;
    laser_light_1.light_on_percent=0.5;
  }
  else if(flight_subtask_cnt[n]<Waypoint_Num+1)
  {
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      execute_time_ms[n]=Waypoint_Time_Gap/flight_subtask_delta;//子任务执行时间
      
      target_position.x=base_position.x+waypoint_x[flight_subtask_cnt[n]];
      target_position.y=base_position.y+waypoint_y[flight_subtask_cnt[n]];
      target_position.z=base_position.z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]++;
    }
  }
  else//一直重复圆形轨迹，只运行一次可以注释掉下方赋值行
  {
    flight_subtask_cnt[n]=1;
  }
}

/*****************************************************************************************/
const int16_t work_waypoints_table[2][32]={
  {0,1,1,2,2,3,3,4,5,6,7,7,6,5,4,4,5,6,7,7,6,5,4,4,5,6,7,7,6,5,4,0},
  {0,4,5,5,4,4,5,5,5,5,5,4,4,4,4,3,3,3,3,2,2,2,2,1,1,1,1,0,0,0,0,0}
};
float work_waypoints[2][32]={0};
uint16_t work_time_gap[32]={
  3000,
  10000,
  3000,3000,3000,3000,3000,3000,3000,3000,3000,
  3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,
  3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,
  10000
};
#define Scale_Param  50.0f
void work_waypoint_generate(void)
{
  for(uint16_t i=0;i<32;i++)
  {
    work_waypoints[0][i]=Scale_Param*work_waypoints_table[0][i];
    work_waypoints[1][i]=Scale_Param*work_waypoints_table[1][i];
  }
  
}


void Agriculture_UAV_Basic(void)
{		
  static uint8_t n=9;
  vector3f target_position;	
  if(flight_subtask_cnt[n]==0)//起飞点作为第一个悬停点
  {
    basic_auto_flight_support();//基本飞行支持软件
    work_waypoint_generate();
    //记录下初始起点位置，实际项目中可设置为某一基准原点
    base_position.x=ins.opt.position_ctrl.x;
    base_position.y=ins.opt.position_ctrl.y;
    base_position.z=ins.position_z;
    
    execute_time_ms[n]=work_time_gap[0]/flight_subtask_delta;//子任务执行时间
    target_position.x=base_position.x+work_waypoints[0][0];
    target_position.y=base_position.y+work_waypoints[1][0];
    target_position.z=base_position.z;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    flight_subtask_cnt[n]=1;		
  }
  else if(flight_subtask_cnt[n]==1)//起飞点悬停完毕后，飞向目标A——第21号作业区域
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      execute_time_ms[n]=work_time_gap[1]/flight_subtask_delta;//子任务执行时间
      
      target_position.x=base_position.x+work_waypoints[0][1];
      target_position.y=base_position.y+work_waypoints[1][1];
      target_position.z=base_position.z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]++;
    }	
  }	
  else if(flight_subtask_cnt[n]<31)//到达A点所在作业点后，遍历所有航点并检测是否打点
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      if(lookdown_vision.trust_flag==1)//到达目标航电后，判断底部是否为农作物
      {
        laser_light_1.reset=1;
        laser_light_1.times=1;//闪速两次
      }
      
      execute_time_ms[n]=work_time_gap[flight_subtask_cnt[n]]/flight_subtask_delta;//子任务执行时间		
      target_position.x=base_position.x+work_waypoints[0][flight_subtask_cnt[n]];
      target_position.y=base_position.y+work_waypoints[1][flight_subtask_cnt[n]];
      target_position.z=base_position.z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]++;
    }
  }
  else if(flight_subtask_cnt[n]==31)//到达最后一个作业点，检测是否打点，最后返回起飞点
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      if(lookdown_vision.trust_flag==1)//到达目标航点后，判断底部是否为农作物
      {
        laser_light_1.reset=1;
        laser_light_1.times=1;//闪速1次
      }	
      execute_time_ms[n]=work_time_gap[31]/flight_subtask_delta;//子任务执行时间
      target_position.x=base_position.x+work_waypoints[0][31];
      target_position.y=base_position.y+work_waypoints[1][31];
      target_position.z=base_position.z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]++;
    }	
  }
  else if(flight_subtask_cnt[n]==32)//到达起飞点
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      execute_time_ms[n]=2000/flight_subtask_delta;//子任务执行时间
      
      flight_subtask_cnt[n]++;
    }			
  }
  else if(flight_subtask_cnt[n]==33)//原地降落
  {
    
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
    indoor_position_control(0);
    flight_altitude_control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//高度控制
  }
  else
  {
    basic_auto_flight_support();//基本飞行支持软件
  }
}



#define Waypoint_Fix_Cnt1   5 //5
#define Waypoint_Fix_CM1    5//5
//位置kp=400
void Agriculture_UAV_Closeloop(void)
{		
  static uint8_t n=10;
  vector3f target_position;	
  if(flight_subtask_cnt[n]==0)//起飞点作为第一个悬停点
  {
    basic_auto_flight_support();//基本飞行支持软件
    work_waypoint_generate();
    //记录下初始起点位置，实际项目中可设置为某一基准原点
    //base_position.x=ins.opt.position_ctrl.x;
    //base_position.y=ins.opt.position_ctrl.y;
    base_position.z=ins.position_z;
    
    execute_time_ms[n]=work_time_gap[0]/flight_subtask_delta;//子任务执行时间
    target_position.x=base_position.x+work_waypoints[0][0];
    target_position.y=base_position.y+work_waypoints[1][0];
    target_position.z=base_position.z;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    flight_subtask_cnt[n]=1;
    flight_global_cnt[n]=0;
  }
  else if(flight_subtask_cnt[n]==1)//检测起飞点悬停完毕后，飞向目标A——第21号作业区域
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//持续4*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续4*5ms满足，表示到达目标航点位置
    {
      target_position.x=base_position.x+work_waypoints[0][1];
      target_position.y=base_position.y+work_waypoints[1][1];
      target_position.z=base_position.z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;		
      
      //flight_subtask_cnt[n]=30;//留作用户测试用阶段调试用，节省时间			
    }
  }	
  else if(flight_subtask_cnt[n]<31)//到达A点所在作业点后，遍历所有航点并检测是否打点
  {
    basic_auto_flight_support();//基本飞行支持软件
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//持续4*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续4*5ms满足，表示到达目标航点位置
    {
      execute_time_ms[n]=work_time_gap[flight_subtask_cnt[n]]/flight_subtask_delta;//子任务执行时间		
      target_position.x=base_position.x+work_waypoints[0][flight_subtask_cnt[n]];
      target_position.y=base_position.y+work_waypoints[1][flight_subtask_cnt[n]];
      target_position.z=base_position.z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;				
      
      if(lookdown_vision.trust_flag==1)//到达目标航点后，判断底部是否为农作物
      {
        laser_light_1.reset=1;
        laser_light_1.times=1;//闪烁1次
      }		
    }
  }
  else if(flight_subtask_cnt[n]==31)//到达最后一个航点单独处理
  {
    basic_auto_flight_support();//基本飞行支持软件
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//持续4*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续4*5ms满足，表示到达目标航点位置
    {
      execute_time_ms[n]=1000/flight_subtask_delta;//子任务执行时间
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;		
      if(lookdown_vision.trust_flag==1)//到达目标航点后，判断底部是否为农作物
      {
        laser_light_1.reset=1;
        laser_light_1.times=1;//闪烁1次
      }			
    }			
  }
  else if(flight_subtask_cnt[n]==32)//最后返回起飞点
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      execute_time_ms[n]=work_time_gap[31]/flight_subtask_delta;//子任务执行时间
      target_position.x=base_position.x+work_waypoints[0][31];
      target_position.y=base_position.y+work_waypoints[1][31];
      target_position.z=base_position.z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]++;
    }	
  }
  else if(flight_subtask_cnt[n]==33)//到达起飞点
  {
    basic_auto_flight_support();//基本飞行支持软件
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//持续4*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else
    {
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      
      //以下复位操作的作用：空中到地面不同高度，环境分布陈设变化造成的误差
      //不同高度梯度上陈设变化不大时，以下特殊处理部分可以不去掉
      //特殊处理开始
      //			send_check_back=4;//重置slam
      //			ins.opt.position_ctrl.x = 0;
      //			ins.opt.position_ctrl.y= 0;
      //		  OpticalFlow_Pos_Ctrl_Expect.x=0;
      //		  OpticalFlow_Pos_Ctrl_Expect.y=0;
      //特殊处理结束
    }		
  }
  else if(flight_subtask_cnt[n]==34)//原地降落
  {	
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
    indoor_position_control(0);
    flight_altitude_control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//高度控制
    //当巡航高度所在环境与地面环境差异偏大时，建议水平方向只使用速度控制降落，避免位置观测误差造成的降落点偏移
    //OpticalFlow_X_Vel_Control(0);
    //OpticalFlow_Y_Vel_Control(0);
  }
  else
  {
    basic_auto_flight_support();//基本飞行支持软件
  }
}







#define Barcode_Height_CM   135//条形码所在高度
#define Waypoint_Fix_Cnt2   5
#define Waypoint_Fix_CM2    5
uint8_t header_fix_flag=1;//机头对准塔杆标志位
uint16_t barcode_id=0;//条码表示的数字信息
uint8_t barcode_flag=0;//条码识别成功标志位
void Agriculture_UAV_Innovation(void)//发挥部分
{		
  if(ABS(min_dis_cm_backups[0]-min_dis_cm_backups[1])>100.0f)//发现距离存在突变，保持上次检测信息
  {
    min_dis_cm_backups[0]		=min_dis_cm_backups[1];//丢弃当前的距离和角度信息
    min_dis_angle_backups[0]=min_dis_angle_backups[1];//丢弃当前的距离和角度信息
  }
  min_dis_cm	 =min_dis_cm_backups[0];
  min_dis_angle=min_dis_angle_backups[0];	
  
  if(min_dis_angle>180) min_dis_angle=min_dis_angle-360;
  target_yaw_err=min_dis_angle;
  
  static uint8_t n=9;
  vector3f target_position;	
  if(flight_subtask_cnt[n]==0)//起飞点作为第一个悬停点
  {
    basic_auto_flight_support();//基本飞行支持软件
    work_waypoint_generate();
    //记录下初始起点位置，实际项目中可设置为某一基准原点
    base_position.x=ins.opt.position_ctrl.x;
    base_position.y=ins.opt.position_ctrl.y;
    base_position.z=ins.position_z;
    
    execute_time_ms[n]=work_time_gap[0]/flight_subtask_delta;//子任务执行时间
    target_position.x=base_position.x+work_waypoints[0][0];
    target_position.y=base_position.y+work_waypoints[1][0];
    target_position.z=base_position.z;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    flight_subtask_cnt[n]=1;
    flight_global_cnt[n]=0;
    
    //flight_subtask_cnt[n]=30;//留作用户测试用阶段调试用，节省时间
  }
  else if(flight_subtask_cnt[n]==1)//检测起飞点悬停完毕后，飞向目标A——第21号作业区域
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Waypoint_Fix_Cnt2)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Waypoint_Fix_CM2)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续10*5ms满足，表示到达目标航点位置
    {
      target_position.x=base_position.x+work_waypoints[0][1];
      target_position.y=base_position.y+work_waypoints[1][1];
      target_position.z=base_position.z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      //flight_subtask_cnt[n]=30;//单独调试寻杆、靠近、识别条码时可以在此赋值，调过打点步骤
    }
    
  }	
  else if(flight_subtask_cnt[n]<31)//到达A点所在作业点后，遍历所有航点并检测是否打点
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Waypoint_Fix_Cnt2)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Waypoint_Fix_CM2)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续10*5ms满足，表示到达目标航点位置
    {
      execute_time_ms[n]=work_time_gap[flight_subtask_cnt[n]]/flight_subtask_delta;//子任务执行时间		
      target_position.x=base_position.x+work_waypoints[0][flight_subtask_cnt[n]];
      target_position.y=base_position.y+work_waypoints[1][flight_subtask_cnt[n]];
      target_position.z=base_position.z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;				
      
      if(lookdown_vision.trust_flag==1)//到达目标航点后，判断底部是否为农作物
      {
        laser_light_1.reset=1;
        laser_light_1.times=1;//闪烁1次
      }		
    }
  }
  else if(flight_subtask_cnt[n]==31)//到达最后一个航点单独处理
  {
    basic_auto_flight_support();//基本飞行支持软件
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Waypoint_Fix_Cnt2)//持续4*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Waypoint_Fix_CM2)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续4*5ms满足，表示到达目标航点位置
    {
      execute_time_ms[n]=1000/flight_subtask_delta;//子任务执行时间
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;		
      if(lookdown_vision.trust_flag==1)//到达目标航点后，判断底部是否为农作物
      {
        laser_light_1.reset=1;
        laser_light_1.times=1;//闪烁1次
      }			
    }	
  }
  else if(flight_subtask_cnt[n]==32)//首先降低飞行高度到130cm
  {
    basic_auto_flight_support();//基本飞行支持软件
    target_position.x=base_position.x+work_waypoints[0][30];
    target_position.y=base_position.y+work_waypoints[1][30];
    target_position.z=Barcode_Height_CM;//条形码所在高度
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    flight_subtask_cnt[n]++;
    flight_global_cnt[n]=0;		
  }
  else if(flight_subtask_cnt[n]==33)//判断是否到达目标高度
  {
    basic_auto_flight_support();//基本飞行支持软件
    //判断是否到达目标高度
    if(flight_global_cnt[n]<100)//持续100*5ms满足
    {
      if(ABS(maple_ctrl.height_position_ctrl.err)<=10.0f)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续100*5ms满足，表示到达目标高度
    {
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
    }
  }
  else if(flight_subtask_cnt[n]==34)//飞到地图中间14号位置，避免现场人员站立太近，激光雷达扫描误以为是距离最近障碍
  {
    basic_auto_flight_support();//基本飞行支持软件
    target_position.x=base_position.x+4*Scale_Param;
    target_position.y=base_position.y+3*Scale_Param;
    target_position.z=Barcode_Height_CM;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Waypoint_Fix_Cnt2)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<Waypoint_Fix_CM2)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else
    {
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;		
    }
  }
  else if(flight_subtask_cnt[n]==35)//首先控制偏航运动，使得飞机头部对准塔杆
  {
    float expect_yaw_gyro=maple_ctrl.yaw_angle_ctrl.kp*target_yaw_err;//期望偏航角速度	
    
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output=constrain_float(expect_yaw_gyro,-10,10);//以10deg/s的角速度顺时针转动10000ms
    if(ABS(target_yaw_err)<=5.0f)//只有当偏航角度比较小时，才靠近杆
    {
      float dis_err=min_dis_cm-50;//设定杆到无人机中心的距离，非摄像头到杆距离，可根据实际轴距、视觉处理模块安装位置以及识别情况适当调整
      dis_err=constrain_float(dis_err,-20,20);
      
      vector2f speed_target;
      speed_target.x=0;
      speed_target.y=maple_ctrl.optical_position_ctrl_y.kp*dis_err;//俯仰水平投影方向下的期望运动速度
      opticalflow_speed_ctrl(speed_target);
      
      header_fix_flag=1;
    }
    else//否则原地悬停
    {
      indoor_position_control(header_fix_flag);
      header_fix_flag=0;
    }			
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    if(barcode_flag==1)//只要识别到了二维码，提前结束对准塔杆、靠近任务
    {
      flight_subtask_cnt[n]=36;
      flight_global_cnt[n]=0;
      
      //识别到条形码后，重新回退到14区块，避免直接返航过程中撞到杆
      target_position.x=base_position.x+4*Scale_Param;
      target_position.y=base_position.y+3*Scale_Param;
      target_position.z=Barcode_Height_CM;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
    }
  }
  else if(flight_subtask_cnt[n]==36)
  {
    basic_auto_flight_support();//基本飞行支持软件
    laser_light_1.reset=1;
    laser_light_1.times=barcode_id;//闪烁barcode_id次
    execute_time_ms[n]=(barcode_id*1000+3000)/flight_subtask_delta;
    flight_subtask_cnt[n]++;
    flight_global_cnt[n]=0;
    
    //正常情况不会发生，纯属保险起见
    if(execute_time_ms[n]>=20000/flight_subtask_delta)   execute_time_ms[n]=20000/flight_subtask_delta;//最大不超过20S,避免id信息识别错误，造成长时间耽置
  }
  else if(flight_subtask_cnt[n]==37)//间隔3s后再次闪烁
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0) 
    {
      laser_light_1.reset=1;
      laser_light_1.times=barcode_id;//闪烁barcode_id次
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
    }
  }
  else if(flight_subtask_cnt[n]==38)//退回到4号的下方格子
  {
    basic_auto_flight_support();//基本飞行支持软件
    target_position.x=base_position.x+4*Scale_Param;
    target_position.y=base_position.y-1*Scale_Param;
    target_position.z=Barcode_Height_CM;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Waypoint_Fix_Cnt2)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Waypoint_Fix_CM2)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else
    {
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;				
    }
  }
  else if(flight_subtask_cnt[n]==39)//退回到起飞点的下方格子
  {
    basic_auto_flight_support();//基本飞行支持软件
    target_position.x=base_position.x;
    target_position.y=base_position.y-1*Scale_Param;
    target_position.z=Barcode_Height_CM;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Waypoint_Fix_Cnt2)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Waypoint_Fix_CM2)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else
    {
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;				
    }	
  }
  else if(flight_subtask_cnt[n]==40)//开始对准机头，赋值只执行一次
  {
    maplepilot.yaw_ctrl_mode=AZIMUTH;				//直接控制航向角模式
    maplepilot.yaw_ctrl_start=1;						//开始控制使能
    maplepilot.yaw_outer_control_output  =0;//目标角度，等效正北和初始机头朝向一致
    indoor_position_control(0);//SLAM定点控制	
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    flight_subtask_cnt[n]=41;		
  }
  else if(flight_subtask_cnt[n]==41)//执行对准机头过程，并判断是否对准完毕
  {
    maplepilot.yaw_ctrl_mode=AZIMUTH;
    maplepilot.yaw_outer_control_output  =0;
    indoor_position_control(0);	
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    if(maplepilot.yaw_ctrl_end==1)  flight_subtask_cnt[n]=42;//执行完毕后，切换到下一阶段	
  }	
  else if(flight_subtask_cnt[n]==42)//退回到起飞点正下方barcode_id*10处悬停
  {
    basic_auto_flight_support();//基本飞行支持软件
    target_position.x=base_position.x;
    target_position.y=base_position.y-10*barcode_id;
    target_position.z=Barcode_Height_CM;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Waypoint_Fix_Cnt2)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Waypoint_Fix_CM2)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else
    {
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      
      //以下复位操作的作用：空中到地面不同高度，环境分布陈设变化造成的误差
      //不同高度梯度上陈设变化不大时，以下特殊处理部分可以不去掉
      //特殊处理开始
      //			send_check_back=4;//重置slam
      //			ins.opt.position_ctrl.x = 0;
      //			ins.opt.position_ctrl.y= 0;
      //		  OpticalFlow_Pos_Ctrl_Expect.x=0;
      //		  OpticalFlow_Pos_Ctrl_Expect.y=0;
      //特殊处理结束
    }	
  }	
  else if(flight_subtask_cnt[n]==43)//原地降落
  {
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
    indoor_position_control(0);
    flight_altitude_control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//高度控制
  }
  else
  {
    basic_auto_flight_support();//基本飞行支持软件
  }
}





//2022年TI电赛——7月份省赛赛题送货无人机
#define Deliver_Scale_Param   25
#define First_Working_Height  150
#define Second_Working_Height	80
#define Fixed_CM     5.0f
#define Times_Fixed  5

#define deliver_down_time 6000//单位ms————塔吊下降时间
#define deliver_up_time   6100//单位ms————塔吊上升时间

#define deliver_down_pwm_us 2000
#define deliver_stop_pwm_us 1500
#define deliver_up_pwm_us 	1000


const int16_t deliver_work_waypoints_table[4][12]={
  {2 , 8, 11, 14, 14, 11, 5, 5, 2,  8, 8 , 14},//横坐标X
  {11, 5, 8 , -1, 11, 2 , 2, 8, 5, -1, 11,  5} //纵坐标Y
};

const uint16_t deliver_work_feature_table[2][12]={
  {1, 1, 2, 2, 1, 1, 2, 2, 1, 1, 2, 2},//颜色：1红色、2蓝色
  {3, 3, 3, 3, 1, 1, 1, 1, 2, 2, 2, 2} //形状：1圆形、2矩形、3三角形
};

uint16_t template_feature[2]={0,0};//特征：颜色+形状
float xtarget[2],ytarget[2];

//特征总共有6组：
//红色三角形、蓝色三角形、红色圆形
//蓝色圆形  、红色矩形  、蓝色矩形
void deliver_work_waypoint_generate(void)
{
  for(uint16_t i=0;i<12;i++)
  {
    work_waypoints[0][i]=Deliver_Scale_Param*deliver_work_waypoints_table[0][i];
    work_waypoints[1][i]=Deliver_Scale_Param*deliver_work_waypoints_table[1][i];
  }
  
}


//第一部分——手动输入两个目标地点作业
void Deliver_UAV_Basic(void)
{
  static uint8_t n=12;
  vector3f target_position;
  float x=0,y=0,z=0;	
  if(flight_subtask_cnt[n]==0)//起飞点作为第一个悬停点
  {
    basic_auto_flight_support();//基本飞行支持软件
    deliver_work_waypoint_generate();//备用
    
    //记录下初始起点位置，实际项目中可设置为某一基准原点
    //base_position.x=ins.opt.position_ctrl.x;
    //base_position.y=ins.opt.position_ctrl.y;
    base_position.z=First_Working_Height;//第一作业高度
    
    x=base_position.x;
    y=base_position.y;
    z=First_Working_Height;
    target_position.x=x;
    target_position.y=y;
    target_position.z=z;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    flight_subtask_cnt[n]=1;
    flight_global_cnt[n]=0;
    execute_time_ms[n]=5000/flight_subtask_delta;//子任务执行时间
    
    laser_light_1.reset=1;
    laser_light_1.times=50000;//闪速50000次
    laser_light_1.period=200;
    laser_light_1.light_on_percent=0.98;
  }
  else if(flight_subtask_cnt[n]==1)//起飞之后原定悬停5后再执行航点任务
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {
      x=param_value[0]*param_value[1];
      y=param_value[0]*param_value[2];
      z=First_Working_Height;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=0;
    }		
  }
  else if(flight_subtask_cnt[n]==2)//检测起飞点悬停完毕后，飞向第一个目标点
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Times_Fixed)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续10*5ms满足，表示到达目标航点位置，后降低目标高度
    {
      x=param_value[0]*param_value[1];
      y=param_value[0]*param_value[2];
      z=Second_Working_Height;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
    }
  }
  else if(flight_subtask_cnt[n]==3)
  {
    basic_auto_flight_support();//基本飞行支持软件		
    //判断是否到达第二作业高度
    if(flight_global_cnt[n]<400)//持续400*5ms满足
    {
      if(ABS(maple_ctrl.height_position_ctrl.err)<=10.0f)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }		
    else
    {
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=deliver_down_time/flight_subtask_delta;
    }
  }
  else if(flight_subtask_cnt[n]==4)//执行塔吊下方任务
  {
    //basic_auto_flight_support();//基本飞行支持软件
    Color_Block_Control_Pilot();//俯视OPENMV视觉水平追踪		
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output =rc_data.rc_rpyt[RC_YAW];			
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    if(execute_time_ms[n]>0) 
    {
      execute_time_ms[n]--;
      reserved_pwm5_output(deliver_down_pwm_us);//放下塔吊
    }
    
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {
      reserved_pwm5_output(deliver_stop_pwm_us);//停止转动
      //不需要刷新位置期望，和上个线程保持一致
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=5000/flight_subtask_delta;
      
      //蜂鸣器语音提示
      laser_light_1.period=200;//200*5ms
      laser_light_1.light_on_percent=0.5f;
      laser_light_1.reset=1;
      laser_light_1.times=4;//闪烁4次	
    }	
  }
  else if(flight_subtask_cnt[n]==5)//原地悬停5s，后恢复到第一巡航高度
  {
    //basic_auto_flight_support();//基本飞行支持软件
    Color_Block_Control_Pilot();//俯视OPENMV视觉水平追踪		
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output =rc_data.rc_rpyt[RC_YAW];			
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {
      indoor_position_control(1);//强制刷新悬停位置
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=deliver_up_time/flight_subtask_delta;;
    }		
  }
  else if(flight_subtask_cnt[n]==6)//收起塔吊
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) 
    {
      execute_time_ms[n]--;
      reserved_pwm5_output(deliver_up_pwm_us);//收起塔吊
    }
    
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {
      reserved_pwm5_output(deliver_stop_pwm_us);//停止转动
      
      x=param_value[0]*param_value[1];
      y=param_value[0]*param_value[2];
      z=First_Working_Height;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=5000/flight_subtask_delta;;
    }			
  }
  else if(flight_subtask_cnt[n]==7)//第一个航点执行完毕后，恢复到第一飞行高度
  {
    basic_auto_flight_support();//基本飞行支持软件
    //判断是否到达第一作业高度
    if(flight_global_cnt[n]<200)//持续200*5ms满足
    {
      if(ABS(maple_ctrl.height_position_ctrl.err)<=10.0f)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }		
    else
    {
      //达到第一飞行高度后，继续执行第二个航点任务
      x=param_value[0]*param_value[3];
      y=param_value[0]*param_value[4];
      z=First_Working_Height;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=0;
    }	
  }		
  else if(flight_subtask_cnt[n]==8)//检测起飞点悬停完毕后，飞向第二个目标点
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Times_Fixed)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续10*5ms满足，表示到达目标航点位置，后降低目标高度
    {
      x=param_value[0]*param_value[3];
      y=param_value[0]*param_value[4];
      z=Second_Working_Height;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
    }
  }
  else if(flight_subtask_cnt[n]==9)
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //判断是否到达第二作业高度
    if(flight_global_cnt[n]<200)//持续200*5ms满足
    {
      if(ABS(maple_ctrl.height_position_ctrl.err)<=10.0f)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }		
    else//放出吊仓
    {	
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=deliver_down_time/flight_subtask_delta;
    }
  }
  else if(flight_subtask_cnt[n]==10)//执行塔吊下方任务
  {
    //basic_auto_flight_support();//基本飞行支持软件
    Color_Block_Control_Pilot();//俯视OPENMV视觉水平追踪		
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output =rc_data.rc_rpyt[RC_YAW];			
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    if(execute_time_ms[n]>0) 
    {
      execute_time_ms[n]--;
      reserved_pwm5_output(deliver_down_pwm_us);//放下塔吊
    }
    
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {
      reserved_pwm5_output(deliver_stop_pwm_us);//停止转动
      
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=5000/flight_subtask_delta;
      
      //蜂鸣器语音提示
      laser_light_1.period=200;//200*5ms
      laser_light_1.light_on_percent=0.5f;
      laser_light_1.reset=1;
      laser_light_1.times=4;//闪烁4次		
    }	
  }
  else if(flight_subtask_cnt[n]==11)//原地悬停5s，后恢复到第一巡航高度
  {
    //basic_auto_flight_support();//基本飞行支持软件
    Color_Block_Control_Pilot();//俯视OPENMV视觉水平追踪		
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output =rc_data.rc_rpyt[RC_YAW];			
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {
      indoor_position_control(1);//强制刷新悬停位置
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=deliver_up_time/flight_subtask_delta;
    }		
  }
  else if(flight_subtask_cnt[n]==12)//收起塔吊
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) 
    {
      execute_time_ms[n]--;
      reserved_pwm5_output(deliver_up_pwm_us);//收起塔吊
    }
    
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {
      reserved_pwm5_output(deliver_stop_pwm_us);//停止转动
      
      x=param_value[0]*param_value[3];
      y=param_value[0]*param_value[4];
      z=First_Working_Height;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=0;
    }			
  }
  else if(flight_subtask_cnt[n]==13)//第二个航点执行完毕后，恢复到第一飞行高度
  {
    basic_auto_flight_support();//基本飞行支持软件
    //判断是否到达第一作业高度
    if(flight_global_cnt[n]<200)//持续400*5ms满足
    {
      if(ABS(maple_ctrl.height_position_ctrl.err)<=10.0f)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }		
    else
    {
      //达到第一飞行高度后，继续执行返航任务航点任务
      target_position.x=base_position.x;
      target_position.y=base_position.y;
      target_position.z=First_Working_Height;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=0;
    }	
  }	
  else if(flight_subtask_cnt[n]==14)//飞向起飞点正上方
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Times_Fixed)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续10*5ms满足，表示到达目标航点位置
    {
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=0;
      /*___________________________________________________________________
      以下复位操作的作用：空中到地面不同高度，环境分布陈设变化造成的误差
      不同高度梯度上陈设变化不大时，以下特殊处理部分可以去掉
      ___________________________________________________________________*/
      //特殊处理开始
      send_check_back=4;//重置slam
      ins.opt.position_ctrl.x = 0;
      ins.opt.position_ctrl.y= 0;
      
      maple_ctrl.optical_position_ctrl_x.expect=0;
      maple_ctrl.optical_position_ctrl_y.expect=0;
      //特殊处理结束	
    }
  }	
  else if(flight_subtask_cnt[n]==15)//原地下降
  {
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
    indoor_position_control(0);
    flight_altitude_control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//高度控制		
  }
  else
  {
    basic_auto_flight_support();//基本飞行支持软件
  }
}

/************************************************************************************************/
//第二部分——学习某一特征后，在飞行中找到两个目标地点作业
void Deliver_UAV_Innovation(void)
{
  static uint8_t n=12;
  vector3f target_position;
  float x=0,y=0,z=0;
  if(flight_subtask_cnt[n]==0)//起飞点作为第一个悬停点
  {
    deliver_work_waypoint_generate();//备用
    template_feature[0]=param_value[7];//加载模板颜色————颜色：1红色、2蓝色
    template_feature[1]=param_value[8];//加载模板形状————形状：1圆形、2矩形、3三角形
    uint16_t k=0;
    for(uint16_t i=0;i<12;i++)
    {
      if(template_feature[0]==deliver_work_feature_table[0][i]
         &&template_feature[1]==deliver_work_feature_table[1][i])
      {
        //将目标特征所在的位置信息加载到目标数组
        xtarget[k]=work_waypoints[0][i];
        ytarget[k]=work_waypoints[1][i];
        k++;
      }
      if(k>=2) break;//两个航点信息刷新完毕后,提前终止
    }
    
    basic_auto_flight_support();//基本飞行支持软件
    //记录下初始起点位置，实际项目中可设置为某一基准原点
    //base_position.x=ins.opt.position_ctrl.x;
    //base_position.y=ins.opt.position_ctrl.y;
    base_position.z=First_Working_Height;//第一作业高度
    
    x=base_position.x;
    y=base_position.y;
    z=First_Working_Height;
    target_position.x=x;
    target_position.y=y;
    target_position.z=z;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    flight_subtask_cnt[n]=1;
    flight_global_cnt[n]=0;
    execute_time_ms[n]=2000/flight_subtask_delta;//子任务执行时间
    
    laser_light_1.reset=1;
    laser_light_1.times=50000;//闪速50000次
    laser_light_1.period=200;
    laser_light_1.light_on_percent=0.75;
  }
  else if(flight_subtask_cnt[n]==1)//起飞之后原定悬停5后再执行航点任务
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {
      x=xtarget[0];
      y=ytarget[0];
      z=First_Working_Height;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=0;
    }		
  }
  else if(flight_subtask_cnt[n]==2)//检测起飞点悬停完毕后，飞向第一个目标点
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Times_Fixed)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续10*5ms满足，表示到达目标航点位置，后降低目标高度
    {
      x=xtarget[0];
      y=ytarget[0];
      z=Second_Working_Height;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
    }
  }
  else if(flight_subtask_cnt[n]==3)
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //判断是否到达第二作业高度
    if(flight_global_cnt[n]<400)//持续400*5ms满足
    {
      if(ABS(maple_ctrl.height_position_ctrl.err)<=10.0f)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }		
    else
    {
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=deliver_down_time/flight_subtask_delta;
    }
  }
  else if(flight_subtask_cnt[n]==4)//执行塔吊下方任务
  {
    //basic_auto_flight_support();//基本飞行支持软件
    Color_Block_Control_Pilot();//俯视OPENMV视觉水平追踪		
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output =rc_data.rc_rpyt[RC_YAW];			
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    
    if(execute_time_ms[n]>0) 
    {
      execute_time_ms[n]--;
      reserved_pwm5_output(deliver_down_pwm_us);//放下塔吊
    }
    
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {
      reserved_pwm5_output(deliver_stop_pwm_us);//停止转动
      //不需要刷新位置期望，和上个线程保持一致
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=5000/flight_subtask_delta;
      
      //蜂鸣器语音提示
      laser_light_1.period=200;//200*5ms
      laser_light_1.light_on_percent=0.5f;
      laser_light_1.reset=1;
      laser_light_1.times=4;//闪烁4次	
    }	
  }
  else if(flight_subtask_cnt[n]==5)//原地悬停5s，后恢复到第一巡航高度
  {
    //basic_auto_flight_support();//基本飞行支持软件
    Color_Block_Control_Pilot();//俯视OPENMV视觉水平追踪		
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output =rc_data.rc_rpyt[RC_YAW];			
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {		
      indoor_position_control(1);//强制刷新悬停位置
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=deliver_up_time/flight_subtask_delta;;
    }		
  }
  else if(flight_subtask_cnt[n]==6)//收起塔吊
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) 
    {
      execute_time_ms[n]--;
      reserved_pwm5_output(deliver_up_pwm_us);//收起塔吊
    }
    
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {
      reserved_pwm5_output(deliver_stop_pwm_us);//停止转动
      
      x=xtarget[0];
      y=ytarget[0];
      z=First_Working_Height;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=5000/flight_subtask_delta;;
    }			
  }
  else if(flight_subtask_cnt[n]==7)//第一个航点执行完毕后，恢复到第一飞行高度
  {
    basic_auto_flight_support();//基本飞行支持软件
    //判断是否到达第一作业高度
    if(flight_global_cnt[n]<200)//持续200*5ms满足
    {
      if(ABS(maple_ctrl.height_position_ctrl.err)<=10.0f)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }		
    else
    {
      //达到第一飞行高度后，继续执行第二个航点任务
      x=xtarget[1];
      y=ytarget[1];
      z=First_Working_Height;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=0;
    }	
  }		
  else if(flight_subtask_cnt[n]==8)//检测起飞点悬停完毕后，飞向第二个目标点
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Times_Fixed)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续10*5ms满足，表示到达目标航点位置，后降低目标高度
    {
      x=xtarget[1];
      y=ytarget[1];
      z=Second_Working_Height;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
    }
  }
  else if(flight_subtask_cnt[n]==9)
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //判断是否到达第二作业高度
    if(flight_global_cnt[n]<200)//持续200*5ms满足
    {
      if(ABS(maple_ctrl.height_position_ctrl.err)<=10.0f)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }		
    else//放出吊仓
    {	
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=deliver_down_time/flight_subtask_delta;
    }
  }
  else if(flight_subtask_cnt[n]==10)//执行塔吊下方任务
  {
    //basic_auto_flight_support();//基本飞行支持软件
    Color_Block_Control_Pilot();//俯视OPENMV视觉水平追踪		
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output =rc_data.rc_rpyt[RC_YAW];			
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    if(execute_time_ms[n]>0) 
    {
      execute_time_ms[n]--;
      reserved_pwm5_output(deliver_down_pwm_us);//放下塔吊
    }
    
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {
      reserved_pwm5_output(deliver_stop_pwm_us);//停止转动
      
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=5000/flight_subtask_delta;
      
      //蜂鸣器语音提示
      laser_light_1.period=200;//200*5ms
      laser_light_1.light_on_percent=0.5f;
      laser_light_1.reset=1;
      laser_light_1.times=4;//闪烁4次		
    }	
  }
  else if(flight_subtask_cnt[n]==11)//原地悬停5s，后恢复到第一巡航高度
  {
    //basic_auto_flight_support();//基本飞行支持软件
    Color_Block_Control_Pilot();//俯视OPENMV视觉水平追踪		
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output =rc_data.rc_rpyt[RC_YAW];			
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {
      indoor_position_control(1);//强制刷新悬停位置
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=deliver_up_time/flight_subtask_delta;
    }		
  }
  else if(flight_subtask_cnt[n]==12)//收起塔吊
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) 
    {
      execute_time_ms[n]--;
      reserved_pwm5_output(deliver_up_pwm_us);//收起塔吊
    }
    
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {
      reserved_pwm5_output(deliver_stop_pwm_us);//停止转动
      
      x=xtarget[1];
      y=ytarget[1];
      z=First_Working_Height;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=0;
    }			
  }
  else if(flight_subtask_cnt[n]==13)//第二个航点执行完毕后，恢复到第一飞行高度
  {
    basic_auto_flight_support();//基本飞行支持软件
    //判断是否到达第一作业高度
    if(flight_global_cnt[n]<200)//持续400*5ms满足
    {
      if(ABS(maple_ctrl.height_position_ctrl.err)<=10.0f)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }		
    else
    {
      //达到第一飞行高度后，继续执行返航任务航点任务
      target_position.x=base_position.x;
      target_position.y=base_position.y;
      target_position.z=First_Working_Height;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=0;
    }	
  }	
  else if(flight_subtask_cnt[n]==14)//飞向起飞点正上方
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Times_Fixed)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续10*5ms满足，表示到达目标航点位置
    {
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=0;
    }
  }	
  else if(flight_subtask_cnt[n]==15)//原地下降
  {
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
    indoor_position_control(0);
    flight_altitude_control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//高度控制		
  }
  else
  {
    basic_auto_flight_support();//基本飞行支持软件
  }
}

/************************************************************************************************/
//第三部分——穿越圆环
void Deliver_UAV_Hulahoop(void)
{
  static uint8_t n=12;
  static float loop_x_cm=0,loop_y_cm=0,loop_angle=0;
  static float temp_x[2],temp_y[2];
  vector3f target_position;
  float x=0,y=0,z=0;
  if(flight_subtask_cnt[n]==0)//起飞点作为第一个悬停点
  {
    deliver_work_waypoint_generate();//备用
    loop_x_cm=param_value[9];
    loop_y_cm=param_value[10];
    loop_angle=param_value[11];
    
    xtarget[0]=loop_x_cm+110*sinf(DEG2RAD*loop_angle);
    ytarget[0]=loop_y_cm-110*cosf(DEG2RAD*loop_angle);
    xtarget[1]=loop_x_cm-110*sinf(DEG2RAD*loop_angle);
    ytarget[1]=loop_y_cm+110*cosf(DEG2RAD*loop_angle);
    //根据圆环的与X轴夹角来决策搜寻对准点
    if(loop_angle<=90)
    {
      temp_x[0]=0;
      temp_y[0]=-50;
      temp_x[1]=350;
      temp_y[1]=-50;
    }
    else
    {
      temp_x[0]=0;
      temp_y[0]=300;
      temp_x[1]=350;
      temp_y[1]=300;		
    }
    
    basic_auto_flight_support();//基本飞行支持软件
    //记录下初始起点位置，实际项目中可设置为某一基准原点
    //base_position.x=ins.opt.position_ctrl.x;
    //base_position.y=ins.opt.position_ctrl.y;
    base_position.z=First_Working_Height;//第一作业高度
    
    x=base_position.x;
    y=base_position.y;
    z=First_Working_Height;
    target_position.x=x;
    target_position.y=y;
    target_position.z=z;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    flight_subtask_cnt[n]=1;
    flight_global_cnt[n]=0;
    execute_time_ms[n]=2000/flight_subtask_delta;//子任务执行时间	
  }
  else if(flight_subtask_cnt[n]==1)//起飞之后原定悬停5后再执行航点任务
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {
      x=temp_x[0];
      y=temp_y[0];
      z=First_Working_Height;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=0;
    }		
  }
  else if(flight_subtask_cnt[n]==2)//检测起飞点悬停完毕后，飞向第一个目标点
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Times_Fixed)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续10*5ms满足，表示到达目标航点位置，后降低目标高度
    {
      x=temp_x[1];
      y=temp_y[1];
      z=First_Working_Height;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
    }
  }
  else if(flight_subtask_cnt[n]==3)//检测起飞点悬停完毕后，飞向第二个目标点
  {
    basic_auto_flight_support();//基本飞行支持软件
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Times_Fixed)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续10*5ms满足，表示到达目标航点位置，后降低目标高度
    {
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
    }
  }
  else if(flight_subtask_cnt[n]==4)//开始对准机头，赋值只执行一次
  {
    maplepilot.yaw_ctrl_mode=AZIMUTH;				//直接控制航向角模式
    maplepilot.yaw_ctrl_start=1;						//开始控制使能
    maplepilot.yaw_outer_control_output  =loop_angle;//目标角度，等效正北和初始机头朝向一致
    
    indoor_position_control(0);//SLAM定点控制	
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    flight_subtask_cnt[n]++;		
  }
  else if(flight_subtask_cnt[n]==5)//执行对准机头过程，并判断是否对准完毕
  {
    maplepilot.yaw_ctrl_mode=AZIMUTH;
    maplepilot.yaw_outer_control_output  =loop_angle;
    indoor_position_control(0);	
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    if(maplepilot.yaw_ctrl_end==1)  flight_subtask_cnt[n]++;//执行完毕后，切换到下一阶段	
  }
  else if(flight_subtask_cnt[n]==6)
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //设置圆环穿越出发点B
    x=xtarget[0];
    y=ytarget[0];
    z=First_Working_Height;
    target_position.x=base_position.x+x;
    target_position.y=base_position.y+y;
    target_position.z=z;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    flight_subtask_cnt[n]++;
    flight_global_cnt[n]=0;
  }	
  else if(flight_subtask_cnt[n]==7)//检测起飞点悬停完毕后，飞向第二个目标点
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Times_Fixed)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续10*5ms满足，表示到达目标航点位置，后降低目标高度
    {
      //设置圆环穿越结束点C
      x=xtarget[1];
      y=ytarget[1];
      z=First_Working_Height;
      target_position.x=base_position.x+x;
      target_position.y=base_position.y+y;
      target_position.z=z;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
    }
  }
  else if(flight_subtask_cnt[n]==8)//检测起飞点悬停完毕后，飞向第二个目标点
  {
    basic_auto_flight_support();//基本飞行支持软件
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Times_Fixed)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续10*5ms满足，表示到达目标航点位置，后降低目标高度
    {
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
    }
  } 
  else if(flight_subtask_cnt[n]==9)//开始对准机头，赋值只执行一次
  {
    maplepilot.yaw_ctrl_mode=AZIMUTH;				//直接控制航向角模式
    maplepilot.yaw_ctrl_start=1;						//开始控制使能
    maplepilot.yaw_outer_control_output  =0;//目标角度，等效正北和初始机头朝向一致
    indoor_position_control(0);//SLAM定点控制	
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    flight_subtask_cnt[n]++;	
  }
  else if(flight_subtask_cnt[n]==10)//执行对准机头过程，并判断是否对准完毕
  {
    maplepilot.yaw_ctrl_mode=AZIMUTH;
    maplepilot.yaw_outer_control_output  =0;
    indoor_position_control(0);	
    flight_altitude_control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
    
    if(maplepilot.yaw_ctrl_end==1)  
    {
      flight_subtask_cnt[n]++;//执行完毕后，切换到下一阶段
      flight_global_cnt[n]=0;
      
      target_position.x=base_position.x;
      target_position.y=base_position.y;
      target_position.z=First_Working_Height;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);		
    }			
  }
  else if(flight_subtask_cnt[n]==11)//返回到起点
  {
    basic_auto_flight_support();//基本飞行支持软件
    
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<Times_Fixed)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续10*5ms满足，表示到达目标航点位置，后降低目标高度
    {
      flight_subtask_cnt[n]++;
      flight_global_cnt[n]=0;
      
      //以下复位操作的作用：空中到地面不同高度，环境分布陈设变化造成的误差
      //不同高度梯度上陈设变化不大时，以下特殊处理部分可以不去掉
      //特殊处理开始
      send_check_back=4;//重置slam
      ins.opt.position_ctrl.x = 0;
      ins.opt.position_ctrl.y= 0;
      maple_ctrl.optical_position_ctrl_x.expect=0;
      maple_ctrl.optical_position_ctrl_y.expect=0;
      //特殊处理结束			
    }
  }
  else if(flight_subtask_cnt[n]==12)//原地下降
  {
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
    indoor_position_control(0);
    flight_altitude_control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//高度控制		
  }	
  else
  {
    basic_auto_flight_support();//基本飞行支持软件
  }
}




//自动起飞到某一高度
uint8_t Auto_Takeoff(float target)
{
  static uint8_t n=11;
  vector3f target_position;
  basic_auto_flight_support();//基本飞行支持软件	
  if(flight_subtask_cnt[n]==0)
  {
    //不加此行代码，当后续全程无油门上下动作后，飞机最后自动降落到地面不会自动上锁
    rc_data.unwanted_lock_flag=0;//允许飞机自动上锁，原理和手动推油起飞类似
    
    //记录下初始起点位置，实际项目中可设置为某一基准原点
    base_position.x=ins.opt.position_ctrl.x;
    base_position.y=ins.opt.position_ctrl.y;
    base_position.z=ins.position_z;
    
    //execute_time_ms[n]=10000/flight_subtask_delta;//子任务执行时间
    target_position.x=base_position.x;
    target_position.y=base_position.y;
    target_position.z=base_position.z+target;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    flight_subtask_cnt[n]=1;
    return	0;		
  }
  else if(flight_subtask_cnt[n]==1)
  {
    //判断是否起飞到目标高度
    if(flight_global_cnt[n]<400)//持续400*5ms满足
    {
      if(ABS(maple_ctrl.height_position_ctrl.err)<=10.0f)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
      return	0;
    }
    else//持续200*5ms满足，表示到达目标高度
    {
      return	1;
    }
  }
  return	0;	
}



/********************************************************************************************************************/
#define nav_param_base (51-1)      //预留参数数组用于存放航点信息的起始量
#define user_setpoint_max 28   //最大航点数，可根据实际项目需要自己定义:显示屏航点设置页面为4页，每页有7个航点，共28个
#define user_setpoint_fixed_2d_cm  5.0f //5cm
#define user_setpoint_fixed_3d_cm  10.0f//10cm
#define user_setpoint_fixed_times  5//满足次数
int32_t nav_setpoint[user_setpoint_max][3]={0,0,0};//航点坐标数组 0x200066D8 
uint8_t user_setpoint_valid_flag[user_setpoint_max]={0};//航点有效标志位
void user_setpoint_generate(void)
{
  memset(user_setpoint_valid_flag,0,sizeof(char)*user_setpoint_max);
  for(uint16_t i=0;i<user_setpoint_max;i++)
  {
    if((param_value[nav_param_base+3*i+0]==0
        &&param_value[nav_param_base+3*i+1]==0
          &&param_value[nav_param_base+3*i+2]==0)!=1)//通过判断参数组内数据均非0，来判定航点数据是否有效
    {
      user_setpoint_valid_flag[i]=true;
      nav_setpoint[i][0]=param_value[nav_param_base+3*i+0];
      nav_setpoint[i][1]=param_value[nav_param_base+3*i+1];
      nav_setpoint[i][2]=param_value[nav_param_base+3*i+2];
    }
    else user_setpoint_valid_flag[i]=false;
  }
}



//用户通过按键自定义输入三维的航点位置，无人机依次遍历各个航点，最多支持28个航点
void Navigation_User_Setpoint(void)
{
  static uint8_t n=13;
  vector3f target_position;
  float x=0,y=0,z=0;	
  if(flight_subtask_cnt[n]==0)//起飞点作为第一个悬停点
  {
    basic_auto_flight_support();//基本飞行支持软件
    user_setpoint_generate();//生成航点
    
    //记录下初始起点位置，实际项目中可设置为某一基准原点
    //base_position.x=ins.opt.position_ctrl.x;
    //base_position.y=ins.opt.position_ctrl.y;
    base_position.z=First_Working_Height;//第一作业高度
    
    x=base_position.x;
    y=base_position.y;
    z=First_Working_Height;
    target_position.x=x;
    target_position.y=y;
    target_position.z=z;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    flight_subtask_cnt[n]=1;
    flight_global_cnt[n]=0;
    flight_global_cnt2[n]=0;
    execute_time_ms[n]=1000/flight_subtask_delta;//子任务执行时间
  }
  else if(flight_subtask_cnt[n]==1)//起飞之后原定悬停1S后再执行航点任务
  {
    basic_auto_flight_support();//基本飞行支持软件
    if(execute_time_ms[n]>0) execute_time_ms[n]--;
    if(execute_time_ms[n]==0)//悬停时间计数器归零，悬停任务执行完毕 
    {
      //判断所有航点均执行完毕，执行下一任务
      if(flight_global_cnt2[n]>=user_setpoint_max)
      {
        flight_subtask_cnt[n]=3;//结束航点遍历
        flight_global_cnt[n]=0;
        execute_time_ms[n]=0;
        //航点计数器清0
        flight_global_cnt2[n]=0;
        return ;				
      }
      
      uint16_t current_num=constrain_int32(flight_global_cnt2[n],0,user_setpoint_max-1);//限幅防溢出
      if(user_setpoint_valid_flag[current_num]==true)//如果当前的航点有效，就设置目标航点进入下一线程
      {
        x=nav_setpoint[current_num][0];
        y=nav_setpoint[current_num][1];
        z=nav_setpoint[current_num][2];
        target_position.x=base_position.x+x;//水平位置期望为起飞后基准位置+位置X偏移
        target_position.y=base_position.y+y;//水平位置期望为起飞后基准位置+位置Y偏移
        target_position.z=z;
        Horizontal_Navigation(target_position.x,
                              target_position.y,
                              target_position.z,
                              GLOBAL_MODE,
                              MAP_FRAME);
        flight_subtask_cnt[n]=2;
        flight_global_cnt[n]=0;
        execute_time_ms[n]=0;				
      }
      else//如果当前的航点无效，跳过当前航点,继续设置下一航点
      {
        //航点计数器自加
        flight_global_cnt2[n]++;			
      }
    }		
  }
  else if(flight_subtask_cnt[n]==2)//检测起飞点悬停完毕后，飞向下一个目标点
  {
    basic_auto_flight_support();//基本飞行支持软件		
    //判断是否到达目标航点位置
    if(flight_global_cnt[n]<user_setpoint_fixed_times)//持续10*5ms=0.05s满足
    {
      float dis_cm=pythagorous3(maple_ctrl.optical_position_ctrl_x.err,\
        maple_ctrl.optical_position_ctrl_y.err,\
          maple_ctrl.height_position_ctrl.err);
      if(dis_cm<=user_setpoint_fixed_3d_cm)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续10*5ms满足，表示到达目标航点位置，后降低目标高度
    {
      flight_subtask_cnt[n]=1;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=0;	
      //航点计数器自加
      flight_global_cnt2[n]++;
    }
    //判断所有航点均执行完毕，执行下一任务		
    if(flight_global_cnt2[n]>=user_setpoint_max)
    {
      flight_subtask_cnt[n]=3;
      flight_global_cnt[n]=0;
      execute_time_ms[n]=0;
      //航点计数器清0
      flight_global_cnt2[n]=0;			
    }
  }
  else if(flight_subtask_cnt[n]==3)//执行航点完毕后，原地下降，可以根据实际需要，自写更多的飞行任务
  {
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];
    indoor_position_control(0);
    flight_altitude_control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//高度控制		
  }
  else
  {
    basic_auto_flight_support();//基本飞行支持软件
  }
}


void Color_Block_Control_Pilot(void)
{
  static uint8_t miss_cnt=1;
  static uint8_t miss_flag=0;
  static vector2f speed_target;
  static uint16_t _cnt=0;	_cnt++;
  if(_cnt>=20)//100ms
  {	
    _cnt=0;		
    if(lookdown_vision.target_ctrl_enable==1)//目标点检测跟踪
    {			
      miss_cnt=1;
      miss_flag=0;			
      lookdown_vision.target_ctrl_enable=0;
      maple_ctrl.sdk_position_ctrl_x.expect=0;
      maple_ctrl.sdk_position_ctrl_x.feedback=lookdown_vision.sdk_target.x;
      //pid_ctrl_err_lpf(&maple_ctrl.sdk_position_ctrl_x,lookdown_vision.trust_flag,0.1f);
      pid_ctrl_err_lpf(&maple_ctrl.sdk_position_ctrl_x,0.1f);
      
      maple_ctrl.sdk_position_ctrl_y.expect=0;
      maple_ctrl.sdk_position_ctrl_y.feedback=lookdown_vision.sdk_target.y;
      pid_ctrl_err_lpf(&maple_ctrl.sdk_position_ctrl_y,0.1f);
      
      speed_target.x= maple_ctrl.sdk_position_ctrl_x.control_output;//横滚水平投影方向下的期望运动速度 -
      speed_target.y=-maple_ctrl.sdk_position_ctrl_y.control_output;//俯仰水平投影方向下的期望运动速度 -
    }
    else//丢失目标
    {
      miss_flag=1;
    }		
  }
  
  if(miss_flag==1)//目标丢失
  {
    if(miss_cnt==1)//初始丢失跟踪目标后，锁定当前位置后，进行普通光流控制
    {
      miss_cnt=2;
      indoor_position_control(1);//20ms		
    }
    else if(miss_cnt==2)//丢失跟踪目标后，进行普通光流控制
    {
      indoor_position_control(0);//20ms		
    }
  }
  else//目标未丢失,10ms
  {
    opticalflow_speed_ctrl(speed_target);//速度控制周期20ms
  }			
}


void Top_APrilTag_Control_Pilot(void)
{
  static uint8_t miss_cnt=1;
  static uint8_t miss_flag=0;
  static vector2f speed_target;
  static uint16_t _cnt=0;	_cnt++;
  if(_cnt>=20)//100ms
  {	
    _cnt=0;		
    if(lookdown_vision.target_ctrl_enable==1)//目标点检测跟踪
    {			
      miss_cnt=1;
      miss_flag=0;			
      lookdown_vision.target_ctrl_enable=0;
      maple_ctrl.sdk_position_ctrl_x.expect=0;
      maple_ctrl.sdk_position_ctrl_x.feedback=lookdown_vision.sdk_target.x;
      pid_ctrl_err_lpf(&maple_ctrl.sdk_position_ctrl_x,0.1f);
      
      maple_ctrl.sdk_position_ctrl_y.expect=0;
      maple_ctrl.sdk_position_ctrl_y.feedback=lookdown_vision.sdk_target.y;
      pid_ctrl_err_lpf(&maple_ctrl.sdk_position_ctrl_y,0.1f);
      
      speed_target.x= maple_ctrl.sdk_position_ctrl_x.control_output;
      speed_target.y=-maple_ctrl.sdk_position_ctrl_y.control_output;	
    }
    else//丢失目标
    {
      miss_flag=1;
    }		
  }
  
  
  if(miss_flag==1)//目标丢失
  {
    if(miss_cnt==1)//初始丢失跟踪目标后，锁定当前位置后，进行普通光流控制
    {
      miss_cnt=2;
      indoor_position_control(1);//20ms		
    }
    else if(miss_cnt==2)//丢失跟踪目标后，进行普通光流控制
    {
      indoor_position_control(0);//20ms		
    }
  }
  else//目标未丢失,10ms
  {
    opticalflow_speed_ctrl(speed_target);//速度控制周期20ms
  }			
}


void Self_Track_Control_Pilot(void)
{
  static uint8_t miss_cnt=1;
  static uint8_t miss_flag=0;
  static vector2f speed_target;
  static uint16_t _cnt=0;	_cnt++;
  if(_cnt>=20)//100ms
  {	
    _cnt=0;		
    if(lookdown_vision.target_ctrl_enable==1)//目标点检测跟踪
    {			
      miss_cnt=1;
      miss_flag=0;			
      lookdown_vision.target_ctrl_enable=0;
      
      maple_ctrl.sdk_position_ctrl_x.expect=0;
      maple_ctrl.sdk_position_ctrl_x.feedback=lookdown_vision.sdk_target.x;
      pid_ctrl_err_lpf(&maple_ctrl.sdk_position_ctrl_x,0.1f);
      speed_target.x= maple_ctrl.sdk_position_ctrl_x.control_output;
      
      static float target_speed;//5 3 2 1
      if(ABS(lookdown_vision.sdk_angle)<=10)      target_speed=8;//15
      else if(ABS(lookdown_vision.sdk_angle)<=20) target_speed=5;//10
      else if(ABS(lookdown_vision.sdk_angle)<=50) target_speed=3;//5
      else target_speed=1;//5 3 2 1
      
      speed_target.y=target_speed;	
    }
    else//丢失目标
    {
      miss_flag=1;
    }		
  }
  
  
  if(miss_flag==1)//目标丢失
  {
    if(miss_cnt==1)//初始丢失跟踪目标后，锁定当前位置后，进行普通光流控制
    {
      miss_cnt=2;
      speed_target.x=0;
      speed_target.y=0;
      indoor_position_control(1);		
    }
    else if(miss_cnt==2)//丢失跟踪目标后，进行普通光流控制
    {
      indoor_position_control(0);		
      maplepilot.yaw_ctrl_mode=ROTATE;
      maplepilot.yaw_outer_control_output  =rc_data.rc_rpyt[RC_YAW];			
    }
  }
  else//目标未丢失
  {
    opticalflow_speed_ctrl(speed_target);//速度控制周期20ms		
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output  =-3.0f*lookdown_vision.sdk_angle;
  }			
}


void simple_auto_flight(void)
{
  static uint8_t n=14;
  vector3f target_position;
  if(flight_subtask_cnt[n]==0)//记录原点并设定目标高度
  {
    basic_auto_flight_support();//基本飞行支持软件	
    
    //不加此行代码，当后续全程无油门上下动作后，飞机最后自动降落到地面不会自动上锁
    rc_data.unwanted_lock_flag=0;//允许飞机自动上锁，原理和手动推油起飞类似
    
    flight_global_cnt[n]=0;	
    //记录下初始起点位置，实际项目中可设置为某一基准原点
    base_position.x=ins.opt.position_ctrl.x;
    base_position.y=ins.opt.position_ctrl.y;
    base_position.z=ins.position_z;
    
    target_position.x=base_position.x;
    target_position.y=base_position.y;
    target_position.z=base_position.z+100;
    Horizontal_Navigation(target_position.x,
                          target_position.y,
                          target_position.z,
                          GLOBAL_MODE,
                          MAP_FRAME);
    flight_subtask_cnt[n]=1;		
  }
  else if(flight_subtask_cnt[n]==1)//执行飞到目标高度并检测是否到达，之后给定前进目标位置后进入下一线程
  {
    basic_auto_flight_support();//基本飞行支持软件
    //判断是否起飞到目标高度
    if(flight_global_cnt[n]<400)//持续400*5ms满足
    {
      if(ABS(maple_ctrl.height_position_ctrl.err)<=10.0f)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续200*5ms满足，表示到达目标高度
    {
      flight_global_cnt[n]=0;
      target_position.x=base_position.x;
      target_position.y=base_position.y+100;
      target_position.z=base_position.z+100;
      Horizontal_Navigation(target_position.x,
                            target_position.y,
                            target_position.z,
                            GLOBAL_MODE,
                            MAP_FRAME);
      flight_subtask_cnt[n]=2;
    }
  }
  else if(flight_subtask_cnt[n]==2)
  {
    basic_auto_flight_support();//基本飞行支持软件
    //判断是否起飞到目标高度
    if(flight_global_cnt[n]<400)//持续400*5ms满足
    {
      float dis_cm=pythagorous2(maple_ctrl.optical_position_ctrl_x.err,maple_ctrl.optical_position_ctrl_y.err);
      if(dis_cm<=Waypoint_Fix_CM2)	flight_global_cnt[n]++;
      else flight_global_cnt[n]/=2;
    }
    else//持续200*5ms满足，表示到达目标高度
    {
      flight_global_cnt[n]=0;
      flight_subtask_cnt[n]=3;
    }	
  }		
  else if(flight_subtask_cnt[n]==3)
  {
    maplepilot.yaw_ctrl_mode=ROTATE;
    maplepilot.yaw_outer_control_output = rc_data.rc_rpyt[RC_YAW];
    indoor_position_control(0);
    flight_altitude_control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//高度控制		
  }
  else 
  {
    basic_auto_flight_support();//基本飞行支持软件
  }
}



