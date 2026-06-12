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
#include "parameter_server.h"
#include "drv_w25qxx.h"
#include "pid.h"
#include "nclink.h"

/***********************************************************/
extern uint16_t pid_param_flag;
extern uint8_t nclink_send_ask_flag[10];
#define PID_USE_NUM  14
static vector3pid pid_parameter[PID_USE_NUM]={0};
static bool pid_parameter_health[PID_USE_NUM]={0};
void pid_init_from_parameter_server(void)
{
  uint16_t pid_fault_cnt=0;
  for(uint16_t i=0;i<PID_USE_NUM;i++)
  {
    ReadFlashParameterThree(PID1_PARAMETER_KP+3*i,&pid_parameter[i].p,&pid_parameter[i].i,&pid_parameter[i].d);
    if(isnan(pid_parameter[i].p)==0&&isnan(pid_parameter[i].i)==0&&isnan(pid_parameter[i].d)==0) pid_parameter_health[i]=true;
  }
  for(uint16_t i=0;i<PID_USE_NUM;i++)
  {
    if(pid_parameter_health[i]==false) pid_fault_cnt++;
  }
  
  if(pid_fault_cnt==0)//eeprom内部数据有效
  {
    pid_init_default();
    maple_ctrl.pitch_gyro_ctrl.kp=pid_parameter[0].p;
    maple_ctrl.pitch_gyro_ctrl.ki=pid_parameter[0].i;
    maple_ctrl.pitch_gyro_ctrl.kd=pid_parameter[0].d;
    
    maple_ctrl.roll_gyro_ctrl.kp=pid_parameter[1].p;
    maple_ctrl.roll_gyro_ctrl.ki=pid_parameter[1].i;
    maple_ctrl.roll_gyro_ctrl.kd=pid_parameter[1].d;
    
    maple_ctrl.yaw_gyro_ctrl.kp=pid_parameter[2].p;
    maple_ctrl.yaw_gyro_ctrl.ki=pid_parameter[2].i;
    maple_ctrl.yaw_gyro_ctrl.kd=pid_parameter[2].d;
    
    maple_ctrl.pitch_angle_ctrl.kp=pid_parameter[3].p;
    maple_ctrl.pitch_angle_ctrl.ki=pid_parameter[3].i;
    maple_ctrl.pitch_angle_ctrl.kd=pid_parameter[3].d;
    
    maple_ctrl.roll_angle_ctrl.kp=pid_parameter[4].p;
    maple_ctrl.roll_angle_ctrl.ki=pid_parameter[4].i;
    maple_ctrl.roll_angle_ctrl.kd=pid_parameter[4].d;
    
    maple_ctrl.yaw_angle_ctrl.kp=pid_parameter[5].p;
    maple_ctrl.yaw_angle_ctrl.ki=pid_parameter[5].i;
    maple_ctrl.yaw_angle_ctrl.kd=pid_parameter[5].d;
    
    maple_ctrl.height_position_ctrl.kp=pid_parameter[6].p;
    maple_ctrl.height_position_ctrl.ki=pid_parameter[6].i;
    maple_ctrl.height_position_ctrl.kd=pid_parameter[6].d;
    
    maple_ctrl.height_speed_ctrl.kp=pid_parameter[7].p;
    maple_ctrl.height_speed_ctrl.ki=pid_parameter[7].i;
    maple_ctrl.height_speed_ctrl.kd=pid_parameter[7].d;
    
    maple_ctrl.height_accel_ctrl.kp=pid_parameter[8].p;
    maple_ctrl.height_accel_ctrl.ki=pid_parameter[8].i;
    maple_ctrl.height_accel_ctrl.kd=pid_parameter[8].d;		
    
    maple_ctrl.latitude_speed_ctrl.kp=pid_parameter[9].p;
    maple_ctrl.latitude_speed_ctrl.ki=pid_parameter[9].i;
    maple_ctrl.latitude_speed_ctrl.kd=pid_parameter[9].d;
    
    maple_ctrl.latitude_position_ctrl.kp=pid_parameter[10].p;
    maple_ctrl.latitude_position_ctrl.ki=pid_parameter[10].i;
    maple_ctrl.latitude_position_ctrl.kd=pid_parameter[10].d;
    /***********************位置控制：位置、速度参数共用一组PID参数**********************************************************/
    maple_ctrl.longitude_speed_ctrl.kp=pid_parameter[9].p;
    maple_ctrl.longitude_speed_ctrl.ki=pid_parameter[9].i;
    maple_ctrl.longitude_speed_ctrl.kd=pid_parameter[9].d;
    
    maple_ctrl.longitude_position_ctrl.kp=pid_parameter[10].p;
    maple_ctrl.longitude_position_ctrl.ki=pid_parameter[10].i;
    maple_ctrl.longitude_position_ctrl.kd=pid_parameter[10].d;
    
    
    maple_ctrl.optical_position_ctrl_x.kp=pid_parameter[11].p;
    maple_ctrl.optical_position_ctrl_x.ki=pid_parameter[11].i;
    maple_ctrl.optical_position_ctrl_x.kd=pid_parameter[11].d;
    maple_ctrl.optical_position_ctrl_y.kp=pid_parameter[11].p;
    maple_ctrl.optical_position_ctrl_y.ki=pid_parameter[11].i;
    maple_ctrl.optical_position_ctrl_y.kd=pid_parameter[11].d;
    
    maple_ctrl.optical_speed_ctrl_x.kp=pid_parameter[12].p;
    maple_ctrl.optical_speed_ctrl_x.ki=pid_parameter[12].i;
    maple_ctrl.optical_speed_ctrl_x.kd=pid_parameter[12].d;
    maple_ctrl.optical_speed_ctrl_y.kp=pid_parameter[12].p;
    maple_ctrl.optical_speed_ctrl_y.ki=pid_parameter[12].i;
    maple_ctrl.optical_speed_ctrl_y.kd=pid_parameter[12].d;		
    
    /***********************共用一组PID参数**********************************************************/    
    maple_ctrl.sdk_position_ctrl_x.kp=pid_parameter[13].p;
    maple_ctrl.sdk_position_ctrl_x.ki=pid_parameter[13].i;
    maple_ctrl.sdk_position_ctrl_x.kd=pid_parameter[13].d;
    maple_ctrl.sdk_position_ctrl_y.kp=pid_parameter[13].p;
    maple_ctrl.sdk_position_ctrl_y.ki=pid_parameter[13].i;
    maple_ctrl.sdk_position_ctrl_y.kd=pid_parameter[13].d;    
  }
  else pid_param_flag=3;   

}








void save_pid_parameter(vector3pid *_pid_parameter)
{
  for(uint16_t i=0;i<PID_USE_NUM;i=i+1)
  {
    WriteFlashParameter_Three(PID1_PARAMETER_KP+i*3,
                              _pid_parameter[i].p,
                              _pid_parameter[i].i,
                              _pid_parameter[i].d,
                              &Flight_Params);
  }
}


void pid_parameter_remap(vector3pid *_pid_parameter)
{
  _pid_parameter[0].p=maple_ctrl.pitch_gyro_ctrl.kp;
  _pid_parameter[0].i=maple_ctrl.pitch_gyro_ctrl.ki;
  _pid_parameter[0].d=maple_ctrl.pitch_gyro_ctrl.kd;
  
  _pid_parameter[1].p=maple_ctrl.roll_gyro_ctrl.kp;
  _pid_parameter[1].i=maple_ctrl.roll_gyro_ctrl.ki;
  _pid_parameter[1].d=maple_ctrl.roll_gyro_ctrl.kd;
  
  _pid_parameter[2].p=maple_ctrl.yaw_gyro_ctrl.kp;
  _pid_parameter[2].i=maple_ctrl.yaw_gyro_ctrl.ki;
  _pid_parameter[2].d=maple_ctrl.yaw_gyro_ctrl.kd;
  
  _pid_parameter[3].p=maple_ctrl.pitch_angle_ctrl.kp;
  _pid_parameter[3].i=maple_ctrl.pitch_angle_ctrl.ki;
  _pid_parameter[3].d=maple_ctrl.pitch_angle_ctrl.kd;
  
  _pid_parameter[4].p=maple_ctrl.roll_angle_ctrl.kp;
  _pid_parameter[4].i=maple_ctrl.roll_angle_ctrl.ki;
  _pid_parameter[4].d=maple_ctrl.roll_angle_ctrl.kd;
  
  _pid_parameter[5].p=maple_ctrl.yaw_angle_ctrl.kp;
  _pid_parameter[5].i=maple_ctrl.yaw_angle_ctrl.ki;
  _pid_parameter[5].d=maple_ctrl.yaw_angle_ctrl.kd;
  
  _pid_parameter[6].p=maple_ctrl.height_position_ctrl.kp;
  _pid_parameter[6].i=maple_ctrl.height_position_ctrl.ki;
  _pid_parameter[6].d=maple_ctrl.height_position_ctrl.kd;
  
  _pid_parameter[7].p=maple_ctrl.height_speed_ctrl.kp;
  _pid_parameter[7].i=maple_ctrl.height_speed_ctrl.ki;
  _pid_parameter[7].d=maple_ctrl.height_speed_ctrl.kd;
  
  _pid_parameter[8].p=maple_ctrl.height_accel_ctrl.kp;
  _pid_parameter[8].i=maple_ctrl.height_accel_ctrl.ki;
  _pid_parameter[8].d=maple_ctrl.height_accel_ctrl.kd;
  
  _pid_parameter[9].p=maple_ctrl.latitude_speed_ctrl.kp;
  _pid_parameter[9].i=maple_ctrl.latitude_speed_ctrl.ki;
  _pid_parameter[9].d=maple_ctrl.latitude_speed_ctrl.kd;
  
  _pid_parameter[10].p=maple_ctrl.latitude_position_ctrl.kp;
  _pid_parameter[10].i=maple_ctrl.latitude_position_ctrl.ki;
  _pid_parameter[10].d=maple_ctrl.latitude_position_ctrl.kd;
  
  _pid_parameter[11].p=maple_ctrl.optical_position_ctrl_x.kp;
  _pid_parameter[11].i=maple_ctrl.optical_position_ctrl_x.ki;
  _pid_parameter[11].d=maple_ctrl.optical_position_ctrl_x.kd;
  
  _pid_parameter[12].p=maple_ctrl.optical_speed_ctrl_x.kp;
  _pid_parameter[12].i=maple_ctrl.optical_speed_ctrl_x.ki;
  _pid_parameter[12].d=maple_ctrl.optical_speed_ctrl_x.kd;
  
  _pid_parameter[13].p=maple_ctrl.sdk_position_ctrl_x.kp;
  _pid_parameter[13].i=maple_ctrl.sdk_position_ctrl_x.ki;
  _pid_parameter[13].d=maple_ctrl.sdk_position_ctrl_x.kd;
}


void pid_parameter_server(void)
{
  if(pid_param_flag==1)//将地面站设置PID参数写入Flash
  {
    pid_parameter_remap(pid_parameter);
    save_pid_parameter(pid_parameter);
    pid_param_flag=0;
  }
  else if(pid_param_flag==2)//将复位PID参数，并写入Flash
  {
    pid_init_default();//将PID参数重置为参数Control_Unit表里面参数
    pid_parameter_remap(pid_parameter);;
    save_pid_parameter(pid_parameter);
    
    pid_param_flag=0;
    
    for(uint16_t i=0;i<6;i++)//恢复默认参数后，将更新的数据发送置地面站
    {
      nclink_send_ask_flag[i]=1;
    }
  }
  else if(pid_param_flag==3)//将复位PID参数，并写入Flash
  {
    pid_init_default();//将PID参数重置为参数Control_Unit表里面参数
    pid_parameter_remap(pid_parameter);;
    save_pid_parameter(pid_parameter);
    pid_param_flag=0;
    
    for(uint16_t i=0;i<6;i++)//恢复默认参数后，将更新的数据发送置地面站
    {
      nclink_send_ask_flag[i]=1;
    }
  }
}


/*****************************************************************/
#include "rc.h"
extern FLIGHT_PARAMETER Flight_Params;
void rc_init_from_parameter_server(void)
{
  rc_init();//先用默认值初始化
  for(uint16_t i=0;i<16;i++)//判断FLASH内数据有效
  {
    if(Flight_Params.health[RC_CH1_MAX+i]==false)  return;
  }
  float _rc_deadband_percent=0;
  for(uint16_t i=0;i<RC_CHL_MAX;i++)
  {	
    if(i==RC_THR_CHANNEL) _rc_deadband_percent=RC_THR_DEADBAND_PERCENT;//油门通道单独处理
    else _rc_deadband_percent=RC_DEADBAND_PERCENT;
    rc_data.cal[i].max_value=Flight_Params.parameter_table[RC_CH1_MAX+2*i];
    rc_data.cal[i].min_value=Flight_Params.parameter_table[RC_CH1_MIN+2*i];
    rc_data.cal[i].deadband	=(rc_data.cal[i].max_value-rc_data.cal[i].min_value)*_rc_deadband_percent;
    rc_data.cal[i].middle_value=(rc_data.cal[i].max_value+rc_data.cal[i].min_value)/2;
    rc_data.cal[i].reverse_flag=RC_REVERSE_FLAG;
    rc_data.cal[i].scale=(rc_data.cal[i].max_value-rc_data.cal[i].min_value-rc_data.cal[i].deadband)*0.5f;
  }
}


extern int8_t imu_sensor_type;
void imu_calibration_init_from_server(void)
{
  uint16_t ACCEL_X_OFFSET=0,ACCEL_X_SCALE=0,HOR_CAL_ACCEL_X=0,GYRO_X_OFFSET=0,PITCH_OFFSET=0,ROLL_OFFSET=0;;
  if(imu_sensor_type==0)
  {
    ACCEL_X_OFFSET=ACCEL_X_OFFSET1;
    ACCEL_X_SCALE=ACCEL_X_SCALE1;
    HOR_CAL_ACCEL_X=HOR_CAL_ACCEL_X1;
    GYRO_X_OFFSET=GYRO_X_OFFSET1;
    PITCH_OFFSET=PITCH_OFFSET1;
    ROLL_OFFSET=ROLL_OFFSET1;
  }
  else if(imu_sensor_type==1) 
  {
    ACCEL_X_OFFSET=ACCEL_X_OFFSET2;
    ACCEL_X_SCALE=ACCEL_X_SCALE2;
    HOR_CAL_ACCEL_X=HOR_CAL_ACCEL_X2;
    GYRO_X_OFFSET=GYRO_X_OFFSET2;
    PITCH_OFFSET=PITCH_OFFSET2;
    ROLL_OFFSET=ROLL_OFFSET2;	
  }
  else if(imu_sensor_type==2) 
  {
    ACCEL_X_OFFSET=ACCEL_X_OFFSET3;
    ACCEL_X_SCALE=ACCEL_X_SCALE3;
    HOR_CAL_ACCEL_X=HOR_CAL_ACCEL_X3;
    GYRO_X_OFFSET=GYRO_X_OFFSET3;
    PITCH_OFFSET=PITCH_OFFSET3;
    ROLL_OFFSET=ROLL_OFFSET3;	
  }
  //陀螺仪校准参数
  if(Flight_Params.health[GYRO_X_OFFSET]==true&&Flight_Params.health[GYRO_X_OFFSET+1]==true&&Flight_Params.health[GYRO_X_OFFSET+2]==true)
  {
    flymaple.gyro_offset.x=Flight_Params.parameter_table[GYRO_X_OFFSET];
    flymaple.gyro_offset.y=Flight_Params.parameter_table[GYRO_X_OFFSET+1];
    flymaple.gyro_offset.z=Flight_Params.parameter_table[GYRO_X_OFFSET+2];
  }
  else
  {
    flymaple.gyro_offset.x=0;
    flymaple.gyro_offset.y=0;
    flymaple.gyro_offset.z=0;
  }	

  if(Flight_Params.health[HOR_CAL_ACCEL_X]==true&&Flight_Params.health[HOR_CAL_ACCEL_X+1]==true&&Flight_Params.health[HOR_CAL_ACCEL_X+2]==true)
  {
    flymaple.accel_hor_offset.x=Flight_Params.parameter_table[HOR_CAL_ACCEL_X];
    flymaple.accel_hor_offset.y=Flight_Params.parameter_table[HOR_CAL_ACCEL_X+1];
    flymaple.accel_hor_offset.z=Flight_Params.parameter_table[HOR_CAL_ACCEL_X+2];
  }
  else
  {
    flymaple.accel_hor_offset.x=0;
    flymaple.accel_hor_offset.y=0;
    flymaple.accel_hor_offset.z=0;  
  }
  
  //读取上次的存储传感器校准方法
  float _accel_simple_mode=0;	
  ReadFlashParameterOne(ACCEL_SIMPLE_MODE,&_accel_simple_mode);
  if(isnan(_accel_simple_mode)==0)   flymaple.accel_calibration_way=_accel_simple_mode;
  if(flymaple.accel_calibration_way==0)//简单校准模式
  {
    flymaple.accel_offset.x=flymaple.accel_hor_offset.x;
    flymaple.accel_offset.y=flymaple.accel_hor_offset.y;
    flymaple.accel_offset.z=flymaple.accel_hor_offset.z;
    flymaple.accel_scale.x=1.0f;
    flymaple.accel_scale.y=1.0f;
    flymaple.accel_scale.z=1.0f;
    flymaple.rpy_angle_offset[PITCH]=0;
    flymaple.rpy_angle_offset[ROLL]=0;
  }
  else//六面校准模式
  {
    //加速度计校准参数
    if(Flight_Params.health[ACCEL_X_OFFSET]==true&&Flight_Params.health[ACCEL_X_OFFSET+1]==true&&Flight_Params.health[ACCEL_X_OFFSET+2]==true 
     &&Flight_Params.health[ACCEL_X_SCALE]==true &&Flight_Params.health[ACCEL_X_SCALE+1]==true &&Flight_Params.health[ACCEL_X_SCALE+2]==true)
    {
      flymaple.accel_offset.x=Flight_Params.parameter_table[ACCEL_X_OFFSET];
      flymaple.accel_offset.y=Flight_Params.parameter_table[ACCEL_X_OFFSET+1];
      flymaple.accel_offset.z=Flight_Params.parameter_table[ACCEL_X_OFFSET+2];
      flymaple.accel_scale.x =Flight_Params.parameter_table[ACCEL_X_SCALE];
      flymaple.accel_scale.y =Flight_Params.parameter_table[ACCEL_X_SCALE+1];
      flymaple.accel_scale.z =Flight_Params.parameter_table[ACCEL_X_SCALE+2];
    }
    else
    {
      flymaple.accel_offset.x=0;
      flymaple.accel_offset.y=0;
      flymaple.accel_offset.z=0;
      flymaple.accel_scale.x=1.0f;
      flymaple.accel_scale.y=1.0f;
      flymaple.accel_scale.z=1.0f;
    }
    //机架水平参数
    if(Flight_Params.health[PITCH_OFFSET]==true&&Flight_Params.health[ROLL_OFFSET]==true)
    {
      flymaple.rpy_angle_offset[PITCH]=Flight_Params.parameter_table[PITCH_OFFSET];
      flymaple.rpy_angle_offset[ROLL] =Flight_Params.parameter_table[ROLL_OFFSET];
    }
    else
    {
      flymaple.rpy_angle_offset[PITCH]=0;
      flymaple.rpy_angle_offset[ROLL]=0;	
    }
  }

  
  if(Flight_Params.health[MAG_X_OFFSET]==true
   &&Flight_Params.health[MAG_Y_OFFSET]==true
   &&Flight_Params.health[MAG_Z_OFFSET]==true )
  {
    flymaple.mag_offset.x=Flight_Params.parameter_table[MAG_X_OFFSET];
    flymaple.mag_offset.y=Flight_Params.parameter_table[MAG_Y_OFFSET];
    flymaple.mag_offset.z=Flight_Params.parameter_table[MAG_Z_OFFSET];
  }
  else
  {
    flymaple.mag_offset.x=0;
    flymaple.mag_offset.y=0;
    flymaple.mag_offset.z=0;	
  }
}



_other_params other_params;
const uint16_t other_parameter_init[12]={Auto_Launch_Target,Nav_Safety_Height,Flight_Safe_Vbat,Flight_Max_Height,Flight_Max_Radius,Climb_Up_Speed_Max,Climb_Down_Speed_Max,Nav_Speed_Max,RESERVED_UART_DEFAULT,Nav_Near_Ground_Height_Default,0,2};
void other_parameter_init_from_server(uint8_t mode)
{
  switch(mode)
  {
  case 1:
    {
      for(uint16_t i=0;i<12;i++)//判断FLASH内数据有效
      {
        other_params.hw[i]=other_parameter_init[i];	
        WriteFlashParameter(TARGET_HEIGHT+i,other_params.hw[i],&Flight_Params);
      }
    }break;
  case 2:
    {
      for(uint16_t i=0;i<12;i++)//判断FLASH内数据有效
      {
        if(Flight_Params.health[TARGET_HEIGHT+i]==false)  other_params.hw[i]=other_parameter_init[i];
        else other_params.hw[i]=Flight_Params.parameter_table[TARGET_HEIGHT+i];
      }			
    }break;
  default:
    {
      for(uint16_t i=0;i<12;i++)//判断FLASH内数据有效
      {
        other_params.hw[i]=other_parameter_init[i];
        WriteFlashParameter(TARGET_HEIGHT+i,other_params.hw[i],&Flight_Params);
      }
    }
  }
}


void reserved_params_init_server(void)
{
  float _param_value[RESERVED_PARAM_NUM];
  for(uint16_t i=0;i<RESERVED_PARAM_NUM;i++)
  {
    ReadFlashParameterOne(RESERVED_PARAM+i,&_param_value[i]);
    if(isnan(_param_value[i])==0) param_value[i]=_param_value[i];
    else param_value[i]=0;
  }
}
