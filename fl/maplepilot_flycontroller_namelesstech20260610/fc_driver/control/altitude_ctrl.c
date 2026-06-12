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
#include "control_output_mix.h"
#include "altitude_ctrl.h"


uint16_t throttle_output=0;
static uint16_t throttle_hover_value=1500;

// === 高度跳变自适应参数 ===
#define HEIGHT_JUMP_THRESH  10.0f   // 跳变下限(cm), 帧间高度变化超过此值触发跟随
#define HEIGHT_JUMP_MAX     20.0f  // 跳变上限(cm), 超过此值视为无效数据, 忽略
#define HEIGHT_TARGET       110.0f // 默认目标高度(cm)
#define HEIGHT_OVER108_THR  108.0f // 判定"超过108"的阈值
#define HEIGHT_OVER108_CNT  5      // 连续超过108的帧数阈值, 达到后恢复110
// ===========================
void flight_altitude_control(uint8_t mode,float target_alt,float target_vel)
{
  //高度控制周期计数器
  static uint16_t period_cnt=0;
  period_cnt++;
  if((period_cnt%(ALT_POS_CTRL_CNT))==0) period_cnt=0;
  else return;
  
  static uint8_t last_mode=0;	 
  static systime althold_dt;
  uint8_t alt_fixed_flag=0;
  get_systime(&althold_dt);
  throttle_hover_value=THR_HOVER_DEFAULT;
  //如果本函数运行周期大于控制周期10倍
  if(0.001f*althold_dt.period>=10*ALT_POS_CTRL_PERIOD*0.001f)
  {
    //情况1:初次或者从其它模式切入本模式
    //情况2:系统调度超时，在系统设计合理情况下，本情况不可能发生
    alt_fixed_flag=1;
  }
  //高度控制从自动模式切到手动模式	 
  if(last_mode!=mode)  alt_fixed_flag=1;
  last_mode=mode;	
  switch(mode)
  {
  case ALTHOLD_MANUAL_CTRL:
    {		 
      //1、高度位置控制器
      if(rc_data.rc_rpyt[RC_THR]==0)//油门杆位于中位死区内，进行高度保持
      {
        //高度位置环输出给定速度期望
        if(maple_ctrl.height_position_ctrl.expect==0||alt_fixed_flag==1)//油门回中后，将回中时刻的高度设置为期望高度，仅设置1次
        {
          maple_ctrl.height_position_ctrl.expect=ins.position_z;	//油门回中后，更新高度期望
        }
        maple_ctrl.height_position_ctrl.feedback=ins.position_z;	//高度位置反馈
        pid_ctrl_general(&maple_ctrl.height_position_ctrl,0.001f*ALT_POS_CTRL_PERIOD);									//海拔高度位置控制器
        
        maple_ctrl.height_speed_ctrl.expect=maple_ctrl.height_position_ctrl.control_output;
        
      }
      else if(rc_data.rc_rpyt[RC_THR]>0)//给定上升速度期望
      {
        //油门杆上推、给定速度期望
        maple_ctrl.height_speed_ctrl.expect=rc_data.rc_rpyt[RC_THR];//最大上升速度300cm/s
        maple_ctrl.height_position_ctrl.expect=0;//位置环期望置0
      }
      else if(rc_data.rc_rpyt[RC_THR]<0)//给定下降速度期望
      {
        //油门杆下推、给定速度期望
        maple_ctrl.height_speed_ctrl.expect=rc_data.rc_rpyt[RC_THR];//最大下降速度200cm/s
        maple_ctrl.height_position_ctrl.expect=0;//位置环期望置0
      }
      
      //2、高度速度控制器
      maple_ctrl.height_speed_ctrl.feedback=ins.speed_z;//惯导速度估计给到速度反馈
      pid_ctrl_general(&maple_ctrl.height_speed_ctrl,0.001f*ALT_POS_CTRL_PERIOD);//海拔高度速度控制
      /*******************************上升下降过程中期望加速度限幅单独处理*******************************************************************/     
      
      //在上下推杆时对速度控制器输出，对期望上升、下降加速度进行分别限幅，确保快速下降时姿态平稳
      maple_ctrl.height_speed_ctrl.control_output=constrain_float(maple_ctrl.height_speed_ctrl.control_output,-Climb_Down_Acceleration_Max,Climb_Up_Acceleration_Max);                                                                
    }
    break;
    
  case ALTHOLD_AUTO_POS_CTRL:
    {		 
      //1、高度位置控制器
      if(rc_data.rc_rpyt[RC_THR]==0)//油门杆位于中位死区内，进行高度保持
      {
        //高度位置环输出给定速度期望
        maple_ctrl.height_position_ctrl.expect=target_alt;	//油门处于回中后，更新高度期望

        // === 高度跳变自适应: 超过110后，跳变>9则跟随新高度；连续5帧>108则恢复110 ===
        {
            static uint8_t  height_gt110_flag = 0;      // 曾超过110标志
            static float    prev_height = 0;             // 上一帧高度
            static uint8_t  over108_cnt = 0;             // 连续超过108计数
            static uint8_t  jump_locked = 0;             // 跳变锁定(锁定后不再响应跳变)

            // 首次超过110 → 激活, 记录初始高度
            if (!height_gt110_flag && ins.position_z > HEIGHT_TARGET)
            {
                height_gt110_flag = 1;
                prev_height = ins.position_z;
                jump_locked = 0;
            }

            if (height_gt110_flag)
            {
                // 连续超过108计数(用于恢复110)
                if (ins.position_z > HEIGHT_OVER108_THR)
                {
                    over108_cnt++;
                    if (over108_cnt >= HEIGHT_OVER108_CNT)
                    {
                        maple_ctrl.height_position_ctrl.expect = HEIGHT_TARGET;
                        over108_cnt = 0;
                        jump_locked = 0;  // 恢复110后重新允许跳变跟随
                    }
                }
                else
                {
                    over108_cnt = 0;
                }

                // 跳变检测: 高度变化在(9, 30]时跟随新高度; >30视为无效数据忽略
                {
                    float jump = fabsf(ins.position_z - prev_height);
                    if (!jump_locked && jump > HEIGHT_JUMP_THRESH && jump <= HEIGHT_JUMP_MAX)
                    {
                        maple_ctrl.height_position_ctrl.expect = ins.position_z;
                        jump_locked = 1;  // 锁定, 不再响应后续跳变, 直到恢复110
                    }
                }

                prev_height = ins.position_z;
            }
        }
        // ==========================================================
        maple_ctrl.height_position_ctrl.feedback = ins.position_z; // 高度位置反馈
        pid_ctrl_general(&maple_ctrl.height_position_ctrl,0.001f*ALT_POS_CTRL_PERIOD);									//海拔高度位置控制器
        
        maple_ctrl.height_speed_ctrl.expect=maple_ctrl.height_position_ctrl.control_output;
      }
      else if(rc_data.rc_rpyt[RC_THR]>0)//给定上升速度期望
      {
        //油门杆上推、给定速度期望
        maple_ctrl.height_speed_ctrl.expect=rc_data.rc_rpyt[RC_THR];//最大上升速度300cm/s
        maple_ctrl.height_position_ctrl.expect=0;//位置环期望置0
      }
      else if(rc_data.rc_rpyt[RC_THR]<0)//给定下降速度期望
      {
        //油门杆下推、给定速度期望
        maple_ctrl.height_speed_ctrl.expect=rc_data.rc_rpyt[RC_THR];//最大下降速度200cm/s
        maple_ctrl.height_position_ctrl.expect=0;//位置环期望置0
      }
      
      //2、高度速度控制器
      maple_ctrl.height_speed_ctrl.feedback=ins.speed_z;//惯导速度估计给到速度反馈
      pid_ctrl_general(&maple_ctrl.height_speed_ctrl,0.001f*ALT_POS_CTRL_PERIOD);//海拔高度速度控制
      /*******************************上升下降过程中期望加速度限幅单独处理*******************************************************************/     
      //在上下推杆时对速度控制器输出，对期望上升、下降加速度进行分别限幅，确保快速下降时姿态平稳
      maple_ctrl.height_speed_ctrl.control_output=constrain_float(maple_ctrl.height_speed_ctrl.control_output,-Climb_Down_Acceleration_Max,Climb_Up_Acceleration_Max);   
    }
    break;
  case ALTHOLD_AUTO_VEL_CTRL:
    {		 
      //1、高度位置控制器
      if(rc_data.rc_rpyt[RC_THR]==0)//油门杆位于中位死区内，进行高度保持
      {
        //高度位置环输出给定速度期望
        maple_ctrl.height_position_ctrl.expect=0;	//油门处于回中后，更新高度期望					
        maple_ctrl.height_speed_ctrl.expect=target_vel;
      }
      else if(rc_data.rc_rpyt[RC_THR]>0)//给定上升速度期望
      {
        //油门杆上推、给定速度期望
        maple_ctrl.height_speed_ctrl.expect=rc_data.rc_rpyt[RC_THR];//最大上升速度300cm/s
        maple_ctrl.height_position_ctrl.expect=0;//位置环期望置0
      }
      else if(rc_data.rc_rpyt[RC_THR]<0)//给定下降速度期望
      {
        //油门杆下推、给定速度期望
        maple_ctrl.height_speed_ctrl.expect=rc_data.rc_rpyt[RC_THR];//最大下降速度200cm/s
        maple_ctrl.height_position_ctrl.expect=0;//位置环期望置0
      }
      
      //2、高度速度控制器
      maple_ctrl.height_speed_ctrl.feedback=ins.speed_z;//惯导速度估计给到速度反馈
      pid_ctrl_general(&maple_ctrl.height_speed_ctrl,0.001f*ALT_POS_CTRL_PERIOD);//海拔高度速度控制
      /*******************************上升下降过程中期望加速度限幅单独处理*******************************************************************/     
      //在上下推杆时对速度控制器输出，对期望上升、下降加速度进行分别限幅，确保快速下降时姿态平稳
      maple_ctrl.height_speed_ctrl.control_output=constrain_float(maple_ctrl.height_speed_ctrl.control_output,-Climb_Down_Acceleration_Max,Climb_Up_Acceleration_Max);  
    }
    break;
  }	
  //3、高度加速度控制器
  maple_ctrl.height_accel_ctrl.expect=maple_ctrl.height_speed_ctrl.control_output;//加速度期望	
  maple_ctrl.height_accel_ctrl.feedback=ins.accel_feedback[_UP];//加速度反馈accel_feedback
  pid_ctrl_general(&maple_ctrl.height_accel_ctrl,0.001f*ALT_POS_CTRL_PERIOD);//海拔高度加速度控制 pid_ctrl_err_lpf
  /**************************************
  加速度环前馈补偿，引用时请注明出处
  悬停油门 = 加速度环积分值 + 基准悬停油门
  此时输出力 F = mg
  当需要输出a的加速度时，输出力 F1=m(g+a)
  F1/F = 1 + a/g
  因此此时应输出：悬停油门*(1 + a/g)
  **************************************/
  int16_t hover_thr_tmp=(throttle_hover_value+maple_ctrl.height_accel_ctrl.integrate-THR_MOTOR_START)*maple_ctrl.height_accel_ctrl.expect/980.0f;
  float output=maple_ctrl.height_accel_ctrl.control_output+hover_thr_tmp;
  throttle_output=(uint16_t)(throttle_hover_value+output);
  throttle_output=constrain_int16(throttle_output,THR_MIN_OUTPUT,THR_MAX_OUTPUT*0.9f);//THR_MIN_OUTPUT
}





#define Minimal_Thrust_Threshold 1150//着陆检测油门最小值
uint8_t landon_earth_flag=1,last_landon_earth_flag=1;
uint32_t landon_earth_cnt=0;
void landon_earth_check(uint8_t shield)//自检触地进入怠速模式
{
  last_landon_earth_flag=landon_earth_flag;
  //油门控制处于较低行程：//1、姿态模式下，油门杆处于低位
  //2、定高模式下，期望速度向下，单加速度环反馈为角小值，
  //加速度控制输出由于长时间积分，到负的较大值，使得油门控制较低
  if(throttle_output<=Minimal_Thrust_Threshold
     &&flymaple.total_gyro_dps<=50.0f//触地后无旋转，合角速度小于30deg/s
       &&FastAbs(ins.speed_z)<=40.0f//惯导竖直轴速度+-40cm/s
         &&shield==0)
    landon_earth_cnt++;
  else landon_earth_cnt/=2;
  
  if(landon_earth_cnt>=800)  landon_earth_cnt=800;//防止溢出
  if(landon_earth_cnt>=200*1.5)//持续1.5S
  {
    landon_earth_flag=1;//着陆标志位
  }
  else
  {
    landon_earth_flag=0;//着陆标志位
  }
  //只要油门变化率不为0，即清空着陆标志位
}

uint8_t get_landon_state(void)
{
  uint8_t land_trigger_flag=0;
  if(last_landon_earth_flag==0//上次状态为不在地面
     &&landon_earth_flag==1)//本次状态为在地面
    land_trigger_flag=1;
  return land_trigger_flag;
}

void reset_landon_state(void)
{
  last_landon_earth_flag=1;
  landon_earth_flag=1;
}






uint16_t throttle_angle_compensate(uint16_t _throttle)//油门倾角补偿
{
  uint16_t output=0;
  float cPit_cRol=fabs(flymaple.cos_rpy[_PIT]*flymaple.cos_rpy[_ROL]);
  float makeup=0,temp=0;
  if(cPit_cRol>=0.999999f)  cPit_cRol=0.999999f;
  if(cPit_cRol<=0.000001f)  cPit_cRol=0.000001f;
  if(cPit_cRol<=0.50f) 			cPit_cRol=0.50f;//Pitch,Roll约等于30度
  if(_throttle>=THR_MOTOR_START)//大于起转油门量
  {
    temp=(uint16_t)(MAX(ABS(100*flymaple.rpy_fusion_deg[PITCH]),ABS(100*flymaple.rpy_fusion_deg[ROLL])));
    temp=constrain_float(9000-temp,0,3000)/(3000*cPit_cRol);
    makeup=(_throttle-THR_MOTOR_START)*temp;//油门倾角补偿
    output=(uint16_t)(THR_MOTOR_START+makeup);
    output=(uint16_t)(constrain_float(output,THR_MOTOR_START,2000));
  }
  else output=_throttle;
  return output;	
}

