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



#include <stdint.h>
#include <stdbool.h>
#include <wp_math.h>
#include "datatype.h"
#include "schedule.h"
#include "pid.h"
#include "drv_uart.h"
#include "parameter_server.h"
#include "drv_tofsense.h"
#include "drv_opticalflow.h"
#include "drv_gps.h"
#include "drv_notify.h"
#include "altitude_ctrl.h"
#include "attitude_ctrl.h"
#include "Developer_Mode.h"
#include "sensor.h"
#include "sins.h"
#include "rc.h"
#include "drv_ppm.h"
#include "nclink.h"


static uint8_t NCLink_Head[2]={0xFF,0xFC};//数据帧头
static uint8_t NCLink_End[2] ={0xA1,0xA2};//数据帧尾

static uint8_t nclink_databuf[128];//待发送数据缓冲区
static uint8_t nclink_rec_buf[128];//待接收数据缓冲区

uint16_t pid_param_flag=0;
uint8_t nclink_send_ask_flag[10]={0};//飞控接收获取参数命令请求，给地面站发送标志位
uint8_t nclink_send_check_flag[20]={0};//数据解析成功，飞控给地面站发送标志位

uint8_t rc_update_flag=0;//遥控器数据更新标志位
uint8_t unlock_flag=0x03,takeoff_flag=0;//解锁、起飞标志位
ngs_sdk_control ngs_sdk;
nclink_cal_state cal_state;//传感器校准标志位


nclink_guide_gps guide_ctrl;
uint16_t nclink_ppm[10]={0};
uint16_t param_id_cnt=0;
float param_value[RESERVED_PARAM_NUM]={0};
third_party_state current_state;
systime nclink_rec_t;

nav_ctrl ngs_nav_ctrl={
  .ctrl_finish_flag=1,
  .update_flag=0,
  .cnt=0,
  .dis_cm=0,
  //以下为静态成员变量，中途不可修改
  .dis_limit_cm=10.0,			//距离阈值，用于判断是否到达目标的
  .cmd_angular_max=50.0,  //角速度阈值，用于限制速度控制模式的角速度
  .cmd_vel_max=50.0,      //速度阈值，用于限制速度控制模式的线速度
}; 




void Serial_Data_Send(uint8_t *buf, uint32_t cnt)  
{
  uart1_send_bytes(buf,cnt);//用户移植时，重写此串口发送函数
}

void Pilot_Status_Tick(void)  
{
  rgb_notify_set(RED	,TOGGLE	 ,50,500,0);//用户移植时，重写此函数
}

union
{
  unsigned char floatByte[4];
  float floatValue;
}FloatUnion;

void Float2Byte(float *FloatValue, unsigned char *Byte, unsigned char Subscript)
{
  FloatUnion.floatValue = (float)2;
  if(FloatUnion.floatByte[0] == 0)//小端模式
  {
    FloatUnion.floatValue = *FloatValue;
    Byte[Subscript]     = FloatUnion.floatByte[0];
    Byte[Subscript + 1] = FloatUnion.floatByte[1];
    Byte[Subscript + 2] = FloatUnion.floatByte[2];
    Byte[Subscript + 3] = FloatUnion.floatByte[3];
  }
  else//大端模式
  {
    FloatUnion.floatValue = *FloatValue;
    Byte[Subscript]     = FloatUnion.floatByte[3];
    Byte[Subscript + 1] = FloatUnion.floatByte[2];
    Byte[Subscript + 2] = FloatUnion.floatByte[1];
    Byte[Subscript + 3] = FloatUnion.floatByte[0];
  }
}

void Byte2Float(unsigned char *Byte, unsigned char Subscript, float *FloatValue)
{
  FloatUnion.floatByte[0]=Byte[Subscript];
  FloatUnion.floatByte[1]=Byte[Subscript + 1];
  FloatUnion.floatByte[2]=Byte[Subscript + 2];
  FloatUnion.floatByte[3]=Byte[Subscript + 3];
  *FloatValue=FloatUnion.floatValue;
}

void NCLink_Send_Status(float roll,float pitch,float yaw,
                        float roll_gyro,float pitch_gyro,float yaw_gyro,
                        float imu_temp,float vbat,uint8_t fly_model,uint8_t armed)
{
  uint8_t _cnt=0;
  int16_t _temp;
  int32_t _temp1;
  uint8_t sum = 0;
  uint8_t i;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_STATUS;
  nclink_databuf[_cnt++]=0;
  
  _temp = (int)(roll*100);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = (int)(pitch*100);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = (int)(yaw*10);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);  
  _temp1=100*roll_gyro;
  nclink_databuf[_cnt++]=BYTE3(_temp1);
  nclink_databuf[_cnt++]=BYTE2(_temp1);
  nclink_databuf[_cnt++]=BYTE1(_temp1);
  nclink_databuf[_cnt++]=BYTE0(_temp1);
  _temp1=100*pitch_gyro;
  nclink_databuf[_cnt++]=BYTE3(_temp1);
  nclink_databuf[_cnt++]=BYTE2(_temp1);
  nclink_databuf[_cnt++]=BYTE1(_temp1);
  nclink_databuf[_cnt++]=BYTE0(_temp1);
  
  _temp1=100*yaw_gyro;
  nclink_databuf[_cnt++]=BYTE3(_temp1);
  nclink_databuf[_cnt++]=BYTE2(_temp1);
  nclink_databuf[_cnt++]=BYTE1(_temp1);
  nclink_databuf[_cnt++]=BYTE0(_temp1);
  
  _temp = (int16_t)(100*imu_temp);//单位℃
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);	
  
  _temp = (int16_t)(100*vbat);//单位V
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  
  
  nclink_databuf[_cnt++]=fly_model;//飞行模式
  nclink_databuf[_cnt++]=armed;//上锁0、解锁1
  nclink_databuf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i];
  nclink_databuf[_cnt++]=sum;
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf, _cnt);
}


void NCLink_Send_Sensor(int16_t a_x,int16_t a_y,int16_t a_z,
                        int16_t g_x,int16_t g_y,int16_t g_z,
                        int16_t m_x,int16_t m_y,int16_t m_z)
{
  uint8_t _cnt=0;
  int16_t _temp;
  uint8_t sum = 0;
  uint8_t i=0;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_SENSOR;
  nclink_databuf[_cnt++]=0;
  
  _temp = a_x;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = a_y;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = a_z;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  
  _temp = g_x;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = g_y;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = g_z;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  
  _temp = m_x;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = m_y;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = m_z;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  
  nclink_databuf[3] = _cnt-4;
  
  for(i=0;i<_cnt;i++)sum ^= nclink_databuf[i];
  nclink_databuf[_cnt++] = sum;
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf, _cnt);
}

void NCLink_Send_RCData(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4,
                        uint16_t ch5,uint16_t ch6,uint16_t ch7,uint16_t ch8)
{
  uint8_t _cnt=0,i=0,sum = 0;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_RCDATA;
  nclink_databuf[_cnt++]=0;
  nclink_databuf[_cnt++]=BYTE1(ch1);
  nclink_databuf[_cnt++]=BYTE0(ch1);
  nclink_databuf[_cnt++]=BYTE1(ch2);
  nclink_databuf[_cnt++]=BYTE0(ch2);
  nclink_databuf[_cnt++]=BYTE1(ch3);
  nclink_databuf[_cnt++]=BYTE0(ch3);
  nclink_databuf[_cnt++]=BYTE1(ch4);
  nclink_databuf[_cnt++]=BYTE0(ch4);
  nclink_databuf[_cnt++]=BYTE1(ch5);
  nclink_databuf[_cnt++]=BYTE0(ch5);
  nclink_databuf[_cnt++]=BYTE1(ch6);
  nclink_databuf[_cnt++]=BYTE0(ch6);
  nclink_databuf[_cnt++]=BYTE1(ch7);
  nclink_databuf[_cnt++]=BYTE0(ch7);
  nclink_databuf[_cnt++]=BYTE1(ch8);
  nclink_databuf[_cnt++]=BYTE0(ch8);
  nclink_databuf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i];
  nclink_databuf[_cnt++]=sum;
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf, _cnt);
}

void NCLink_Send_GPSData(int32_t lng,int32_t lat,int32_t alt,int16_t pdop,uint8_t fixstate,uint8_t numsv)
{
  uint16_t sum = 0,_cnt=0,i=0;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_GPS;
  nclink_databuf[_cnt++]=0;
  
  nclink_databuf[_cnt++]=BYTE3(lng);
  nclink_databuf[_cnt++]=BYTE2(lng);
  nclink_databuf[_cnt++]=BYTE1(lng);
  nclink_databuf[_cnt++]=BYTE0(lng);
  
  nclink_databuf[_cnt++]=BYTE3(lat);
  nclink_databuf[_cnt++]=BYTE2(lat);
  nclink_databuf[_cnt++]=BYTE1(lat);
  nclink_databuf[_cnt++]=BYTE0(lat);  
  
  nclink_databuf[_cnt++]=BYTE3(alt);
  nclink_databuf[_cnt++]=BYTE2(alt);
  nclink_databuf[_cnt++]=BYTE1(alt);
  nclink_databuf[_cnt++]=BYTE0(alt);
  
  nclink_databuf[_cnt++]=BYTE1(pdop);
  nclink_databuf[_cnt++]=BYTE0(pdop);	
  
  nclink_databuf[_cnt++]=fixstate;
  nclink_databuf[_cnt++]=numsv;
  nclink_databuf[3] = _cnt-4; 
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i];
  nclink_databuf[_cnt++]=sum;
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf, _cnt);
}


void NCLink_Send_Obs_NE(float lat_pos_obs,float lng_pos_obs,
                        float lat_vel_obs,float lng_vel_obs)
{
  uint16_t sum = 0,_cnt=0,i=0;	
  int32_t _temp;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_OBS_NE;
  nclink_databuf[_cnt++]=0;
  
  _temp=100*lat_pos_obs;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  
  _temp=100*lng_pos_obs;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);  
  
  _temp=100*lat_vel_obs;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  
  _temp=100*lng_vel_obs;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  nclink_databuf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i];
  nclink_databuf[_cnt++]=sum;
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf, _cnt);
}

void NCLink_Send_Obs_UOP(float alt_obs_baro,float alt_obs_ult,float opt_vel_p,float opt_vel_r)
{
  uint16_t sum = 0,_cnt=0,i=0;
  int32_t _temp;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_OBS_UOP;
  nclink_databuf[_cnt++]=0;
  
  _temp=100*alt_obs_baro;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  
  _temp=100*alt_obs_ult;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  
  _temp=100*opt_vel_p;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  
  _temp=100*opt_vel_r;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp); 
  nclink_databuf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i];
  nclink_databuf[_cnt++]=sum;
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf, _cnt);
}

void NCLink_Send_Fusion_U(float alt_pos_fus,float alt_vel_fus,float alt_accel_fus)
{
  uint16_t sum = 0,_cnt=0,i=0;
  int32_t _temp;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_FUS_U;
  nclink_databuf[_cnt++]=0;
  _temp=100*alt_pos_fus;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp=100*alt_vel_fus;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp=100*alt_accel_fus;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);	
  nclink_databuf[3] = _cnt-4;  
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i]; 
  nclink_databuf[_cnt++]=sum;
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf, _cnt);
}

void NCLink_Send_Fusion_NE(float lat_pos_fus	,float lng_pos_fus,
                           float lat_vel_fus  ,float lng_vel_fus,
                           float lat_accel_fus,float lng_accel_fus)
{
  uint8_t sum = 0,_cnt=0,i=0;
  int32_t _temp;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_FUS_NE;
  nclink_databuf[_cnt++]=0;
  _temp=100*lat_pos_fus;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp=100*lng_pos_fus;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp=100*lat_vel_fus;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp=100*lng_vel_fus;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);	
  _temp=100*lat_accel_fus;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);	
  _temp=100*lng_accel_fus;
  nclink_databuf[_cnt++]=BYTE3(_temp);
  nclink_databuf[_cnt++]=BYTE2(_temp);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);	
  nclink_databuf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i];
  nclink_databuf[_cnt++]=sum;
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf, _cnt);
}


void NCLink_Send_PID(uint8_t group,float pid1_kp,float pid1_ki,float pid1_kd,
                     float pid2_kp,float pid2_ki,float pid2_kd,
                     float pid3_kp,float pid3_ki,float pid3_kd)
{
  uint8_t _cnt=0,sum = 0,i=0;
  int16_t _temp;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=group;
  nclink_databuf[_cnt++]=0;
  _temp = (int16_t)(pid1_kp * 1000);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = (int16_t)(pid1_ki  * 1000);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = (int16_t)(pid1_kd  * 100);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = (int16_t)(pid2_kp  * 1000);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = (int16_t)(pid2_ki  * 1000);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = (int16_t)(pid2_kd * 100);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = (int16_t)(pid3_kp  * 1000);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = (int16_t)(pid3_ki  * 1000);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = (int16_t)(pid3_kd * 100);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  nclink_databuf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i];
  nclink_databuf[_cnt++]=sum;
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf, _cnt);
}


void NCLink_Send_Parameter(uint16_t targeheight,uint16_t safeheight,uint16_t safevbat,uint16_t maxheight,
                           uint16_t maxradius,uint16_t maxupvel,uint16_t maxdownvel,uint16_t maxhorvel,
                           uint16_t reserveduart,uint16_t neargroundheight,uint16_t uart2_mode,uint16_t avoid_obstacle)
{
  uint8_t _cnt=0,sum = 0,i=0;
  uint16_t _temp;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_SEND_PARA;
  nclink_databuf[_cnt++]=0; 
  _temp = targeheight;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = safeheight;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = safevbat;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = maxheight;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = maxradius;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = maxupvel;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = maxdownvel;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = maxhorvel;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp); 
  
  _temp = reserveduart;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp); 
  
  _temp = neargroundheight;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp); 
  
  _temp = uart2_mode;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp); 
  
  _temp = avoid_obstacle;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp); 
  
  nclink_databuf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i];
  nclink_databuf[_cnt++]=sum;
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf, _cnt);
}


void NCLink_Send_Userdata(float userdata1	 ,float userdata2,
                          float userdata3  ,float userdata4,
                          float userdata5  ,float userdata6)
{
  uint8_t sum=0,_cnt=0,i=0;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_USER;
  nclink_databuf[_cnt++]=0;
  
  Float2Byte(&userdata1,nclink_databuf,_cnt);//4
  _cnt+=4;
  Float2Byte(&userdata2,nclink_databuf,_cnt);//8
  _cnt+=4;
  Float2Byte(&userdata3,nclink_databuf,_cnt);//12
  _cnt+=4;
  Float2Byte(&userdata4,nclink_databuf,_cnt);//16
  _cnt+=4;
  Float2Byte(&userdata5,nclink_databuf,_cnt);//20
  _cnt+=4;
  Float2Byte(&userdata6,nclink_databuf,_cnt);//24
  _cnt+=4;//28
  
  nclink_databuf[3] = _cnt-4;
  
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i]; 
  nclink_databuf[_cnt++]=sum;
  
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf,_cnt);
}


void NCLink_Send_3D_Track(float x,float y,float z,float _q0,float _q1,float _q2,float _q3)
{
  uint8_t sum=0,_cnt=0,i=0;
  int16_t _temp;
  
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_SEND_3D_TRACK;
  nclink_databuf[_cnt++]=0;
  
  Float2Byte(&x,nclink_databuf,_cnt);//4
  _cnt+=4;
  Float2Byte(&y,nclink_databuf,_cnt);//8
  _cnt+=4;
  Float2Byte(&z,nclink_databuf,_cnt);//12
  _cnt+=4;
  
  _temp = (int)(_q0*10000);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = (int)(_q1*10000);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = (int)(_q2*10000);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  _temp = (int)(_q3*10000);
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  
  
  nclink_databuf[3] = _cnt-4;
  
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i]; 
  nclink_databuf[_cnt++]=sum;
  
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf,_cnt);
}

void NCLink_Send_CalRawdata1(uint8_t gyro_auto_cal_flag,
                             float gyro_x_raw,float gyro_y_raw,float gyro_z_raw,
                             float acce_x_raw,float acce_y_raw,float acce_z_raw)
{
  uint8_t sum=0,_cnt=0,i=0;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_SEND_CAL_RAW1;
  nclink_databuf[_cnt++]=0;
  nclink_databuf[_cnt++]=gyro_auto_cal_flag;
  
  Float2Byte(&gyro_x_raw,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&gyro_y_raw,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&gyro_z_raw,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&acce_x_raw,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&acce_y_raw,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&acce_z_raw,nclink_databuf,_cnt);
  _cnt+=4;
  
  nclink_databuf[3] = _cnt-4;	
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i]; 
  nclink_databuf[_cnt++]=sum;
  
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf,_cnt);
}

void NCLink_Send_CalRawdata2(float mag_x_raw ,float mag_y_raw ,float mag_z_raw)
{
  uint8_t sum=0,_cnt=0,i=0;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_SEND_CAL_RAW2;
  nclink_databuf[_cnt++]=0;
  
  Float2Byte(&mag_x_raw,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&mag_y_raw,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&mag_z_raw,nclink_databuf,_cnt);
  _cnt+=4;	
  
  nclink_databuf[3] = _cnt-4;	
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i]; 
  nclink_databuf[_cnt++]=sum;
  
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf,_cnt);
}


void NCLink_Send_CalParadata1(float gyro_x_offset,float gyro_y_offset,float gyro_z_offset,
                              float mag_x_offset ,float mag_y_offset ,float mag_z_offset)
{
  uint8_t sum=0,_cnt=0,i=0;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_SEND_CAL_PARA1;
  nclink_databuf[_cnt++]=0;
  
  Float2Byte(&gyro_x_offset,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&gyro_y_offset,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&gyro_z_offset,nclink_databuf,_cnt);
  _cnt+=4;
  
  
  Float2Byte(&mag_x_offset,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&mag_y_offset,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&mag_z_offset,nclink_databuf,_cnt);
  _cnt+=4;	
  
  nclink_databuf[3] = _cnt-4;	
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i]; 
  nclink_databuf[_cnt++]=sum;
  
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  
  Serial_Data_Send(nclink_databuf,_cnt);	
}

void NCLink_Send_CalParadata2(float acce_x_offset,float acce_y_offset,float acce_z_offset,
                              float acce_x_scale ,float acce_y_scale ,float acce_z_scale)
{
  uint8_t sum=0,_cnt=0,i=0;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_SEND_CAL_PARA2;
  nclink_databuf[_cnt++]=0;
  
  
  Float2Byte(&acce_x_offset,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&acce_y_offset,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&acce_z_offset,nclink_databuf,_cnt);
  _cnt+=4;
  
  Float2Byte(&acce_x_scale,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&acce_y_scale,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&acce_z_scale,nclink_databuf,_cnt);
  _cnt+=4;	
  
  nclink_databuf[3] = _cnt-4;	
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i]; 
  nclink_databuf[_cnt++]=sum;
  
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  
  Serial_Data_Send(nclink_databuf,_cnt);	
}


void NCLink_Send_CalParadata3(float pitch_offset ,float roll_offset)
{
  uint8_t sum=0,_cnt=0,i=0;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_SEND_CAL_PARA3;
  nclink_databuf[_cnt++]=0;
  
  Float2Byte(&pitch_offset,nclink_databuf,_cnt);
  _cnt+=4;
  Float2Byte(&roll_offset,nclink_databuf,_cnt);
  _cnt+=4;	
  
  
  nclink_databuf[3] = _cnt-4;	
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i]; 
  nclink_databuf[_cnt++]=sum;
  
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  
  Serial_Data_Send(nclink_databuf,_cnt);	
}

void NCLink_Send_Parameter_Reserved(uint16_t id,float param)
{
  uint8_t _cnt=0,sum = 0,i=0;
  uint16_t _temp;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_SEND_PARA_RESERVED;
  nclink_databuf[_cnt++]=0; 
  
  _temp = id;
  nclink_databuf[_cnt++]=BYTE1(_temp);
  nclink_databuf[_cnt++]=BYTE0(_temp);
  
  Float2Byte(&param,nclink_databuf,_cnt);
  _cnt+=4;
  
  nclink_databuf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i];
  nclink_databuf[_cnt++]=sum;
  nclink_databuf[_cnt++]=NCLink_End[0];
  nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf, _cnt);
}

static void NCLink_Send_Check(uint8_t response)//地面站应答校验
{
  uint8_t sum = 0,i=0;
  nclink_databuf[0]=NCLink_Head[0];
  nclink_databuf[1]=NCLink_Head[1];
  nclink_databuf[2]=NCLINK_SEND_CHECK;
  nclink_databuf[3]=1;
  nclink_databuf[4]=response;
  for(i=0;i<5;i++) sum ^= nclink_databuf[i];
  nclink_databuf[5]=sum;
  nclink_databuf[6]=NCLink_End[0];
  nclink_databuf[7]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf,8);
}

void NCLink_Data_Prase_Prepare(uint8_t data)//地面站数据解析
{
  
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==NCLink_Head[1])//判断帧头1
  {
    state=1;
    nclink_rec_buf[0]=data;
  }
  else if(state==1&&data==NCLink_Head[0])//判断帧头2
  {
    state=2;
    nclink_rec_buf[1]=data;
  }
  else if(state==2&&data<0XF1)//功能字节
  {
    state=3;
    nclink_rec_buf[2]=data;
  }
  else if(state==3&&data<100)//有效数据长度
  {
    state = 4;
    nclink_rec_buf[3]=data;
    data_len = data;
    data_cnt = 0;
  }
  else if(state==4&&data_len>0)//数据接收
  {
    data_len--;
    nclink_rec_buf[4+data_cnt++]=data;
    if(data_len==0)  state = 5;
  }
  else if(state==5)//异或校验
  {
    state = 6;
    nclink_rec_buf[4+data_cnt++]=data;
  }
  else if(state==6&&data==NCLink_End[0])//帧尾0
  {
    state = 7;
    nclink_rec_buf[4+data_cnt++]=data;
  }
  else if(state==7&&data==NCLink_End[1])//帧尾1
  {
    state = 0;
    nclink_rec_buf[4+data_cnt]=data;
    NCLink_Data_Prase_Process(nclink_rec_buf,data_cnt+5);//数据解析
  }
  else state = 0;
}

void NCLink_Data_Prase_Process(uint8_t *data_buf,uint8_t num)//飞控数据解析进程
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<(num-3);i++)  sum ^= *(data_buf+i);
  if(!(sum==*(data_buf+num-3)))    																					return;//判断sum	
  if(!(*(data_buf)==NCLink_Head[1]&&*(data_buf+1)==NCLink_Head[0]))         return;//判断帧头
  if(!(*(data_buf+num-2)==NCLink_End[0]&&*(data_buf+num-1)==NCLink_End[1])) return;//帧尾校验  
  if(*(data_buf+2)==0X01)//地面站请求状态解析
  {
    if(*(data_buf+4)==0X01)		//地面站发送读取当前PID参数请求
    {
      nclink_send_ask_flag[0]=1;//飞控发送第1组PID参数，请求位置1
      nclink_send_ask_flag[1]=1;//飞控发送第2组PID参数，请求位置1
      nclink_send_ask_flag[2]=1;//飞控发送第3组PID参数，请求位置1
      nclink_send_ask_flag[3]=1;//飞控发送第4组PID参数，请求位置1
      nclink_send_ask_flag[4]=1;//飞控发送第5组PID参数，请求位置1
      nclink_send_ask_flag[5]=1;//飞控发送第6组PID参数，请求位置1
      Pilot_Status_Tick();
    }	
    else if(*(data_buf+4)==0X02)   //地面站发送恢复默认PID参数请求
    {
      pid_param_flag=3;
      Pilot_Status_Tick();
    }
    else if(*(data_buf+4)==0X03)//地面站发送读取当前其它参数请求    
    {
      nclink_send_ask_flag[6]=1;//飞控发送其它参数请求位置1
      Pilot_Status_Tick();
    }
    else if(*(data_buf+4)==0X04)//地面站发送恢复默认其它参数请求  
    {
      nclink_send_ask_flag[6]=1;//恢复默认参数，并将默认参数发送到地面站
      other_parameter_init_from_server(1);//其它参数恢复默认
      Pilot_Status_Tick();
    }
    else if(*(data_buf+4)==0X05)//地面站发送恢复出厂设置
    {
      Resume_Factory_Setting();
      nclink_send_check_flag[11]=1;
      Pilot_Status_Tick();
    }
    else if(*(data_buf+4)==0X06)//地面站发送读取预留设置
    {
      nclink_send_ask_flag[7]=1;
      param_id_cnt=(int16_t)(*(data_buf+5)<<8)|*(data_buf+6);
      Pilot_Status_Tick();
    }
    else if(*(data_buf+4)==0X07)//地面站发送读取全部预留设置
    {
      nclink_send_ask_flag[8]=1;
      //param_id_cnt=(int16_t)(*(data_buf+5)<<8)|*(data_buf+6);
      Pilot_Status_Tick();
    }
  }
  else if(*(data_buf+2)==0X02)                             //接收PID1-3
  {
    maple_ctrl.roll_gyro_ctrl.kp  = 0.001*( (int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
    maple_ctrl.roll_gyro_ctrl.ki  = 0.001*( (int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
    maple_ctrl.roll_gyro_ctrl.kd  = 0.01*( (int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
    maple_ctrl.pitch_gyro_ctrl.kp = 0.001*( (int16_t)(*(data_buf+10)<<8)|*(data_buf+11));
    maple_ctrl.pitch_gyro_ctrl.ki = 0.001*( (int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
    maple_ctrl.pitch_gyro_ctrl.kd = 0.01*( (int16_t)(*(data_buf+14)<<8)|*(data_buf+15));
    maple_ctrl.yaw_gyro_ctrl.kp   = 0.001*( (int16_t)(*(data_buf+16)<<8)|*(data_buf+17));
    maple_ctrl.yaw_gyro_ctrl.ki   = 0.001*( (int16_t)(*(data_buf+18)<<8)|*(data_buf+19));
    maple_ctrl.yaw_gyro_ctrl.kd   = 0.01*( (int16_t)(*(data_buf+20)<<8)|*(data_buf+21));
    nclink_send_check_flag[0]=1;
  }
  else if(*(data_buf+2)==0X03)                             //接收PID4-6
  {
    maple_ctrl.roll_angle_ctrl.kp  = 0.001*( (int16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
    maple_ctrl.roll_angle_ctrl.ki  = 0.001*( (int16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
    maple_ctrl.roll_angle_ctrl.kd  = 0.01*( (int16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
    maple_ctrl.pitch_angle_ctrl.kp = 0.001*( (int16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
    maple_ctrl.pitch_angle_ctrl.ki = 0.001*( (int16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
    maple_ctrl.pitch_angle_ctrl.kd = 0.01*( (int16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
    maple_ctrl.yaw_angle_ctrl.kp   = 0.001*( (int16_t)(*(data_buf+16)<<8)|*(data_buf+17) );
    maple_ctrl.yaw_angle_ctrl.ki   = 0.001*( (int16_t)(*(data_buf+18)<<8)|*(data_buf+19) );
    maple_ctrl.yaw_angle_ctrl.kd   = 0.01*( (int16_t)(*(data_buf+20)<<8)|*(data_buf+21) );
    nclink_send_check_flag[1]=1;
  }
  else if(*(data_buf+2)==0X04)                             //接收PID7-9
  {
    maple_ctrl.height_position_ctrl.kp= 0.001*( (int16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
    maple_ctrl.height_position_ctrl.ki= 0.001*( (int16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
    maple_ctrl.height_position_ctrl.kd= 0.01*( (int16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
    maple_ctrl.height_speed_ctrl.kp   = 0.001*( (int16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
    maple_ctrl.height_speed_ctrl.ki   = 0.001*( (int16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
    maple_ctrl.height_speed_ctrl.kd   = 0.01*( (int16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
    maple_ctrl.height_accel_ctrl.kp		 = 0.001*( (int16_t)(*(data_buf+16)<<8)|*(data_buf+17) );
    maple_ctrl.height_accel_ctrl.ki		 = 0.001*( (int16_t)(*(data_buf+18)<<8)|*(data_buf+19) );
    maple_ctrl.height_accel_ctrl.kd		 = 0.01*( (int16_t)(*(data_buf+20)<<8)|*(data_buf+21) );
    /***********************位置控制：位置、速度参数共用一组PID参数**********************************************************/
    maple_ctrl.longitude_speed_ctrl.kp=maple_ctrl.latitude_speed_ctrl.kp;
    maple_ctrl.longitude_speed_ctrl.ki=maple_ctrl.latitude_speed_ctrl.ki;
    maple_ctrl.longitude_speed_ctrl.kd=maple_ctrl.latitude_speed_ctrl.kd;
    nclink_send_check_flag[2]=1;
  }
  else if(*(data_buf+2)==0X05)                             //接收PID9-11
  {
    maple_ctrl.latitude_position_ctrl.kp= 0.001*( (int16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
    maple_ctrl.latitude_position_ctrl.ki= 0.001*( (int16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
    maple_ctrl.latitude_position_ctrl.kd= 0.01*( (int16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
    maple_ctrl.latitude_speed_ctrl.kp   = 0.001*( (int16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
    maple_ctrl.latitude_speed_ctrl.ki   = 0.001*( (int16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
    maple_ctrl.latitude_speed_ctrl.kd   = 0.01*( (int16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
    maple_ctrl.sdk_position_ctrl_x.kp= 0.001*( (int16_t)(*(data_buf+16)<<8)|*(data_buf+17) );
    maple_ctrl.sdk_position_ctrl_x.ki= 0.001*( (int16_t)(*(data_buf+18)<<8)|*(data_buf+19) );
    maple_ctrl.sdk_position_ctrl_x.kd= 0.01*( (int16_t)(*(data_buf+20)<<8)|*(data_buf+21) );
    /***********************位置控制：位置、速度参数共用一组PID参数**********************************************************/
    maple_ctrl.longitude_position_ctrl.kp=maple_ctrl.latitude_position_ctrl.kp;
    maple_ctrl.longitude_position_ctrl.ki=maple_ctrl.latitude_position_ctrl.ki;
    maple_ctrl.longitude_position_ctrl.kd=maple_ctrl.latitude_position_ctrl.kd;
    maple_ctrl.longitude_speed_ctrl.kp=maple_ctrl.latitude_speed_ctrl.kp;
    maple_ctrl.longitude_speed_ctrl.ki=maple_ctrl.latitude_speed_ctrl.ki;
    maple_ctrl.longitude_speed_ctrl.kd=maple_ctrl.latitude_speed_ctrl.kd;
    
    maple_ctrl.sdk_position_ctrl_y.kp = maple_ctrl.sdk_position_ctrl_x.kp;
    maple_ctrl.sdk_position_ctrl_y.ki = maple_ctrl.sdk_position_ctrl_x.ki;
    maple_ctrl.sdk_position_ctrl_y.kd = maple_ctrl.sdk_position_ctrl_x.kd;
    
    nclink_send_check_flag[3]=1;
  }
  else if(*(data_buf+2)==0X06)                             //接收PID12-15
  {
    maple_ctrl.optical_position_ctrl_x.kp  = 0.001*( (int16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
    maple_ctrl.optical_position_ctrl_x.ki  = 0.001*( (int16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
    maple_ctrl.optical_position_ctrl_x.kd  = 0.01*( (int16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
    maple_ctrl.optical_speed_ctrl_x.kp     = 0.001*( (int16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
    maple_ctrl.optical_speed_ctrl_x.ki     = 0.001*( (int16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
    maple_ctrl.optical_speed_ctrl_x.kd     = 0.01*( (int16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
    
    maple_ctrl.optical_position_ctrl_y.kp=maple_ctrl.optical_position_ctrl_x.kp;
    maple_ctrl.optical_position_ctrl_y.ki=maple_ctrl.optical_position_ctrl_x.ki;
    maple_ctrl.optical_position_ctrl_y.kd=maple_ctrl.optical_position_ctrl_x.kd;
    maple_ctrl.optical_speed_ctrl_y.kp   =maple_ctrl.optical_speed_ctrl_x.kp;
    maple_ctrl.optical_speed_ctrl_y.ki	 =maple_ctrl.optical_speed_ctrl_x.ki;
    maple_ctrl.optical_speed_ctrl_y.kd   =maple_ctrl.optical_speed_ctrl_x.kd;
    
    nclink_send_check_flag[4]=1;		
  }
  else if(*(data_buf+2)==0X07)                             //接收PID16-18
  {
    nclink_send_check_flag[5]=1;
    pid_param_flag=1;	
    Pilot_Status_Tick();
  }
  else if(*(data_buf+2)==0X08)                             //其它参数
  {
    other_params.params.target_height =(int16_t)(*(data_buf+4)<<8)|*(data_buf+5);
    other_params.params.safe_height =(int16_t)(*(data_buf+6)<<8)|*(data_buf+7);
    other_params.params.safe_vbat =(int16_t)(*(data_buf+8)<<8)|*(data_buf+9);
    other_params.params.max_height =(int16_t)(*(data_buf+10)<<8)|*(data_buf+11);
    other_params.params.max_radius =(int16_t)(*(data_buf+12)<<8)|*(data_buf+13);
    other_params.params.max_upvel =(int16_t)(*(data_buf+14)<<8)|*(data_buf+15);
    other_params.params.max_downvel =(int16_t)(*(data_buf+16)<<8)|*(data_buf+17);
    other_params.params.max_horvel =(int16_t)(*(data_buf+18)<<8)|*(data_buf+19);
    other_params.params.reserved_uart =(int16_t)(*(data_buf+20)<<8)|*(data_buf+21);
    other_params.params.near_ground_height =(int16_t)(*(data_buf+22)<<8)|*(data_buf+23);		
    other_params.params.inner_uart=((int16_t)(*(data_buf+24)<<8)|*(data_buf+25));
    other_params.params.opticalcal_type=((int16_t)(*(data_buf+26)<<8)|*(data_buf+27));
    
    nclink_send_check_flag[6]=1;
    Pilot_Status_Tick();
    
    WriteFlashParameter(TARGET_HEIGHT,other_params.params.target_height,&Flight_Params);
    WriteFlashParameter(SAFE_HEIGHT,other_params.params.safe_height,&Flight_Params);
    WriteFlashParameter(SAFE_VBAT,other_params.params.safe_vbat,&Flight_Params);
    WriteFlashParameter(MAX_HEIGHT,other_params.params.max_height,&Flight_Params);
    WriteFlashParameter(MAX_RADIUS,other_params.params.max_radius,&Flight_Params);
    WriteFlashParameter(MAX_UPVEL,other_params.params.max_upvel,&Flight_Params);
    WriteFlashParameter(MAX_DOWNVEL,other_params.params.max_downvel,&Flight_Params);
    WriteFlashParameter(MAX_HORVEL,other_params.params.max_horvel,&Flight_Params);
    WriteFlashParameter(RESERVED_UART_FUNCTION,other_params.params.reserved_uart,&Flight_Params);
    WriteFlashParameter(NEAR_GROUND_HEIGHT,other_params.params.near_ground_height,&Flight_Params);
    WriteFlashParameter(UART5_FUNCTION,other_params.params.inner_uart,&Flight_Params);
    WriteFlashParameter(OPTICALFLOW_TYPE,other_params.params.opticalcal_type,&Flight_Params);		
  }
  else if(*(data_buf+2)==0X09)                             //遥控器参数
  {
    nclink_ppm[0]=(int16_t)(*(data_buf+4)<<8) |*(data_buf+5);
    nclink_ppm[1]=(int16_t)(*(data_buf+6)<<8) |*(data_buf+7);
    nclink_ppm[2]=(int16_t)(*(data_buf+8)<<8) |*(data_buf+9);
    nclink_ppm[3]=(int16_t)(*(data_buf+10)<<8)|*(data_buf+11);
    nclink_ppm[4]=(int16_t)(*(data_buf+12)<<8)|*(data_buf+13);
    nclink_ppm[5]=(int16_t)(*(data_buf+14)<<8)|*(data_buf+15);
    nclink_ppm[6]=(int16_t)(*(data_buf+16)<<8)|*(data_buf+17);
    nclink_ppm[7]=(int16_t)(*(data_buf+18)<<8)|*(data_buf+19);
    nclink_ppm[8]=(int16_t)(*(data_buf+20)<<8)|*(data_buf+21);
    nclink_ppm[9]=(int16_t)(*(data_buf+22)<<8)|*(data_buf+23);
    
    rc_update_flag=1;
    
    unlock_flag=*(data_buf+24);
    takeoff_flag=*(data_buf+25);		
    nclink_send_check_flag[7]=1;
    Pilot_Status_Tick();	
  }
  else if(*(data_buf+2)==0X0A)                             //地面站控制移动数据
  {		
    ngs_sdk.move_mode=*(data_buf+4),
    ngs_sdk.mode_order=*(data_buf+5);
    ngs_sdk.move_distance=(uint16_t)(*(data_buf+6)<<8)|*(data_buf+7);;
    ngs_sdk.update_flag=true;
    
    ngs_sdk.move_flag.sdk_front_flag=false;
    ngs_sdk.move_flag.sdk_behind_flag=false;
    ngs_sdk.move_flag.sdk_left_flag=false;
    ngs_sdk.move_flag.sdk_right_flag=false;
    ngs_sdk.move_flag.sdk_up_flag=false;
    ngs_sdk.move_flag.sdk_down_flag=false;
    
    if(*(data_buf+4)==SDK_FRONT)
    {					
      ngs_sdk.move_flag.sdk_front_flag=true;
      ngs_sdk.f_distance=ngs_sdk.move_distance;
    }
    else if(*(data_buf+4)==SDK_BEHIND) 
    {					
      ngs_sdk.move_flag.sdk_behind_flag=true;
      ngs_sdk.f_distance=-ngs_sdk.move_distance;
    }
    else if(*(data_buf+4)==SDK_LEFT)  
    {			
      ngs_sdk.move_flag.sdk_left_flag=true;
      ngs_sdk.f_distance=ngs_sdk.move_distance;
    }
    else if(*(data_buf+4)==SDK_RIGHT)
    {					
      ngs_sdk.move_flag.sdk_right_flag=true;
      ngs_sdk.f_distance=-ngs_sdk.move_distance;
    }
    else if(*(data_buf+4)==SDK_UP)
    {  				
      ngs_sdk.move_flag.sdk_up_flag=true;
      ngs_sdk.f_distance=ngs_sdk.move_distance;
    }
    else if(*(data_buf+4)==SDK_DOWN) 
    {					
      ngs_sdk.move_flag.sdk_down_flag=true;
      ngs_sdk.f_distance=-ngs_sdk.move_distance;
    }				
    nclink_send_check_flag[8]=1;
    Pilot_Status_Tick();	
  }
  else if(*(data_buf+2)==0X0B)                             //地面站发送校准数据
  {		
    cal_state.cal_flag=*(data_buf+4),
    cal_state.cal_step=*(data_buf+5);
    cal_state.cal_cmd=*(data_buf+6);
    cal_state.update_flag=true;
    if(cal_state.cal_flag==0x00&&cal_state.cal_step==0x00&&cal_state.cal_cmd==0x00)//提前终止当前校准
    {
      cal_state.shutdown_now_cal_flag=1;
    }	
    else
    {
      if(cal_state.cal_cmd==0x01)
      {
        nclink_send_check_flag[10]=1;
        cal_state.cal_cmd=0x00;
      }
    }
    Pilot_Status_Tick();			
  }
  else if(*(data_buf+2)==0X0C)
  {
    guide_ctrl.lng =((int32_t)(*(data_buf+4)<<24)|(*(data_buf+5)<<16)|(*(data_buf+6)<<8)|*(data_buf+7));
    guide_ctrl.lat =((int32_t)(*(data_buf+8)<<24)|(*(data_buf+9)<<16)|(*(data_buf+10)<<8)|*(data_buf+11));
    guide_ctrl.update_flag=1;
    
    Pilot_Status_Tick();
  }
  else if(*(data_buf+2)==0X0D)
  {
    Byte2Float(data_buf,4,&current_state.position_x);
    Byte2Float(data_buf,8,&current_state.position_y);
    Byte2Float(data_buf,12,&current_state.position_z);
    Byte2Float(data_buf,16,&current_state.q[0]);		
    Byte2Float(data_buf,20,&current_state.q[1]);
    Byte2Float(data_buf,24,&current_state.q[2]);
    Byte2Float(data_buf,28,&current_state.q[3]);
    Byte2Float(data_buf,32,&current_state.quality);
    current_state.update_flag=*(data_buf+36);
    get_systime(&nclink_rec_t);
  }
  else if(*(data_buf+2)==0X0E)//预留参数
  {
    
    uint16_t param_id=(int16_t)(*(data_buf+4)<<8) |*(data_buf+5);
    if(param_id_cnt<RESERVED_PARAM_NUM)
    {			
      Byte2Float(data_buf,6,&param_value[param_id]);
      WriteFlashParameter(RESERVED_PARAM+param_id,param_value[param_id],&Flight_Params);
    }
    nclink_send_check_flag[12]=1;
    Pilot_Status_Tick();
  }
  else if(*(data_buf+2)==0X0F)
  {
    ngs_nav_ctrl.number=(uint16_t)(*(data_buf+4)<<8) |*(data_buf+5);
    Byte2Float(data_buf,6, &ngs_nav_ctrl.x);
    Byte2Float(data_buf,10,&ngs_nav_ctrl.y);
    Byte2Float(data_buf,14,&ngs_nav_ctrl.z);
    ngs_nav_ctrl.nav_mode=*(data_buf+18);
    ngs_nav_ctrl.frame_id=*(data_buf+19);
    ngs_nav_ctrl.cmd_vel_during_cnt=(uint32_t)(*(data_buf+20)<<24|*(data_buf+21)<<16|*(data_buf+22)<<8|*(data_buf+23));	//速度控制时的作用时间,单位MS
    if(ngs_nav_ctrl.nav_mode!=CMD_VEL_MODE)	ngs_nav_ctrl.update_flag=1;
    else 
    {
      if(ngs_nav_ctrl.frame_id==0)//数据来源于ROS
      {
        ngs_nav_ctrl.cmd_vel_x=-100*ngs_nav_ctrl.x;					 //转化成cm/s
        ngs_nav_ctrl.cmd_vel_y= 100*ngs_nav_ctrl.y;          //转化成cm/s
        ngs_nav_ctrl.cmd_vel_angular_z=57.3f*ngs_nav_ctrl.z; //转化成deg/s
      }	
      else//数据来源于地面站
      {
        ngs_nav_ctrl.cmd_vel_x=-ngs_nav_ctrl.x;			   //cm/s
        ngs_nav_ctrl.cmd_vel_y= ngs_nav_ctrl.y;        //cm/s
        ngs_nav_ctrl.cmd_vel_angular_z=ngs_nav_ctrl.z; //deg/s			
      }
      //限幅处理避免新手错误操作，输入值过大导致飞机期望过大造成的失控
      ngs_nav_ctrl.cmd_vel_x=constrain_float(ngs_nav_ctrl.cmd_vel_x,-ngs_nav_ctrl.cmd_vel_max,ngs_nav_ctrl.cmd_vel_max);
      ngs_nav_ctrl.cmd_vel_y=constrain_float(ngs_nav_ctrl.cmd_vel_y,-ngs_nav_ctrl.cmd_vel_max,ngs_nav_ctrl.cmd_vel_max);
      ngs_nav_ctrl.cmd_vel_angular_z=constrain_float(ngs_nav_ctrl.cmd_vel_angular_z,-ngs_nav_ctrl.cmd_angular_max,ngs_nav_ctrl.cmd_angular_max);
      ngs_nav_ctrl.cmd_vel_during_cnt/=5;//作用时间200*5=1000ms
      ngs_nav_ctrl.cmd_vel_update=1;
    }
    nclink_send_check_flag[13]=1;
    Pilot_Status_Tick();
  }
}



uint8_t NCLink_Send_Check_Status_Parameter(void)
{
  if(nclink_send_check_flag[0]==1)//飞控解析第1组PID参数成功，返回状态给地面站
  {
    NCLink_Send_Check(NCLINK_SEND_PID1_3);
    nclink_send_check_flag[0]=0;
    return 1;
  }
  else if(nclink_send_check_flag[1]==1)//飞控解析第2组PID参数成功，返回状态给地面站
  {
    NCLink_Send_Check(NCLINK_SEND_PID4_6);
    nclink_send_check_flag[1]=0;
    return 1;
  }
  else if(nclink_send_check_flag[2]==1)//飞控解析第3组PID参数成功，返回状态给地面站
  {
    NCLink_Send_Check(NCLINK_SEND_PID7_9);
    nclink_send_check_flag[2]=0;
    return 1;
  }
  else if(nclink_send_check_flag[3]==1)//飞控解析第4组PID参数成功，返回状态给地面站
  {
    NCLink_Send_Check(NCLINK_SEND_PID10_12);
    nclink_send_check_flag[3]=0;
    return 1;
  }
  else if(nclink_send_check_flag[4]==1)//飞控解析第5组PID参数成功，返回状态给地面站
  {
    NCLink_Send_Check(NCLINK_SEND_PID13_15);
    nclink_send_check_flag[4]=0;
    return 1;
  }
  else if(nclink_send_check_flag[5]==1)//飞控解析第6组PID参数成功，返回状态给地面站
  {
    NCLink_Send_Check(NCLINK_SEND_PID16_18);
    nclink_send_check_flag[5]=0;
    return 1;
  }
  else if(nclink_send_check_flag[6]==1)//飞控解析其它参数成功，返回状态给地面站
  {
    NCLink_Send_Check(NCLINK_SEND_PARA);
    nclink_send_check_flag[6]=0;
    return 1;
  }
  else if(nclink_send_check_flag[7]==1)//飞控解析遥控器数据成功，返回状态给地面站
  {
    NCLink_Send_Check(NCLINK_SEND_RC);	
    nclink_send_check_flag[7]=0;
    return 1;
  }
  else if(nclink_send_check_flag[8]==1)//飞控解析位移数据成功，返回状态给地面站
  {
    NCLink_Send_Check(NCLINK_SEND_DIS);
    nclink_send_check_flag[8]=0;
    return 1;
  }
  else if(nclink_send_check_flag[9]==1)//飞控传感器校准完毕，返回状态给地面站
  {
    NCLink_Send_Check(NCLINK_SEND_CAL);
    nclink_send_check_flag[9]=0;
    
    nclink_send_check_flag[10]=1;//每次校准完毕自动刷新地面站校准数据
    return 1;
  }
  else if(nclink_send_check_flag[10]==1)//飞控传感器校准成功，返回状态给地面站
  {
    static uint16_t cnt=0;
    if(cnt==0)
    {
      NCLink_Send_Check(NCLINK_SEND_CAL_READ);	
      cnt=1;
    }
    else if(cnt==1)
    {
      NCLink_Send_CalParadata1(flymaple.gyro_offset.x,
                               flymaple.gyro_offset.y,
                               flymaple.gyro_offset.z,
                               flymaple.mag_offset.x,
                               flymaple.mag_offset.y,
                               flymaple.mag_offset.z);		 
      cnt=2;
    }
    else if(cnt==2)
    {
      NCLink_Send_CalParadata2(flymaple.accel_offset.x*GRAVITY_MSS,
                               flymaple.accel_offset.y*GRAVITY_MSS,
                               flymaple.accel_offset.z*GRAVITY_MSS,
                               flymaple.accel_scale.x,
                               flymaple.accel_scale.y,
                               flymaple.accel_scale.z);	 
      cnt=3;
    }      
    else if(cnt==3)
    {
      NCLink_Send_CalParadata3(flymaple.rpy_angle_offset[PITCH],flymaple.rpy_angle_offset[ROLL]);
      cnt=0;
      nclink_send_check_flag[10]=0;
    } 
    
    return 1;
  }
  else if(nclink_send_check_flag[11]==1)//飞控恢复出厂设置完毕，返回状态给地面站
  {
    NCLink_Send_Check(NCLINK_SEND_FACTORY);
    nclink_send_check_flag[11]=0;
    return 1;
  }	
  else if(nclink_send_check_flag[12]==1)//飞控接收到预留参数，返回状态给地面站
  {
    NCLink_Send_Check(NCLINK_SEND_PARA_RESERVED);
    nclink_send_check_flag[12]=0;
    return 1;
  }	
  else if(nclink_send_check_flag[13]==1)//飞控接收到导航控制指令，返回状态给地面站
  {
    NCLink_Send_Check(NCLINK_SEND_NAV_CTRL);
    nclink_send_check_flag[13]=0;
    return 1;
  }
  else if(nclink_send_check_flag[14]==1)//飞控导航控制执行完毕，返回状态给地面站
  {
    NCLink_Send_Check(NCLINK_SEND_NAV_CTRL_FINISH);
    nclink_send_check_flag[14]=0;
    return 1;
  }		
  else if(nclink_send_ask_flag[0]==1)//接收到地面站读取PID参数请求，飞控发送第1组PID参数给地面站
  {
    NCLink_Send_PID(NCLINK_SEND_PID1_3,
                    maple_ctrl.roll_gyro_ctrl.kp,
                    maple_ctrl.roll_gyro_ctrl.ki,
                    maple_ctrl.roll_gyro_ctrl.kd,
                    maple_ctrl.pitch_gyro_ctrl.kp,
                    maple_ctrl.pitch_gyro_ctrl.ki,
                    maple_ctrl.pitch_gyro_ctrl.kd,
                    maple_ctrl.yaw_gyro_ctrl.kp,
                    maple_ctrl.yaw_gyro_ctrl.ki,
                    maple_ctrl.yaw_gyro_ctrl.kd);
    nclink_send_ask_flag[0]=0;
    return 1;
  }
  else if(nclink_send_ask_flag[1]==1)//接收到地面站读取PID参数请求，飞控发送第2组PID参数给地面站
  {
    NCLink_Send_PID(NCLINK_SEND_PID4_6,
                    maple_ctrl.roll_angle_ctrl.kp,
                    maple_ctrl.roll_angle_ctrl.ki,
                    maple_ctrl.roll_angle_ctrl.kd,
                    maple_ctrl.pitch_angle_ctrl.kp,
                    maple_ctrl.pitch_angle_ctrl.ki,
                    maple_ctrl.pitch_angle_ctrl.kd,
                    maple_ctrl.yaw_angle_ctrl.kp,
                    maple_ctrl.yaw_angle_ctrl.ki,
                    maple_ctrl.yaw_angle_ctrl.kd);
    nclink_send_ask_flag[1]=0;
    return 1;
  }
  else if(nclink_send_ask_flag[2]==1)//接收到地面站读取PID参数请求，飞控发送第3组PID参数给地面站
  {
    NCLink_Send_PID(NCLINK_SEND_PID7_9,
                    maple_ctrl.height_position_ctrl.kp,
                    maple_ctrl.height_position_ctrl.ki,
                    maple_ctrl.height_position_ctrl.kd,
                    maple_ctrl.height_speed_ctrl.kp,
                    maple_ctrl.height_speed_ctrl.ki,
                    maple_ctrl.height_speed_ctrl.kd,
                    maple_ctrl.height_accel_ctrl.kp,
                    maple_ctrl.height_accel_ctrl.ki,
                    maple_ctrl.height_accel_ctrl.kd);
    nclink_send_ask_flag[2]=0;
    return 1;
  }
  else if(nclink_send_ask_flag[3]==1)//接收到地面站读取PID参数请求，飞控发送第4组PID参数给地面站
  {
    NCLink_Send_PID(NCLINK_SEND_PID10_12,
                    maple_ctrl.latitude_position_ctrl.kp,
                    maple_ctrl.latitude_position_ctrl.ki,
                    maple_ctrl.latitude_position_ctrl.kd,
                    maple_ctrl.latitude_speed_ctrl.kp,
                    maple_ctrl.latitude_speed_ctrl.ki,
                    maple_ctrl.latitude_speed_ctrl.kd,
                    maple_ctrl.sdk_position_ctrl_x.kp,
                    maple_ctrl.sdk_position_ctrl_x.ki,
                    maple_ctrl.sdk_position_ctrl_x.kd); 
    nclink_send_ask_flag[3]=0;
    return 1;
  }
  else if(nclink_send_ask_flag[4]==1)//接收到地面站读取PID参数请求，飞控发送第5一组PID参数给地面站
  {
    NCLink_Send_PID(NCLINK_SEND_PID13_15,
                    maple_ctrl.optical_position_ctrl_x.kp
                      ,maple_ctrl.optical_position_ctrl_x.ki
                        ,maple_ctrl.optical_position_ctrl_x.kd
                          ,maple_ctrl.optical_speed_ctrl_x.kp
                            ,maple_ctrl.optical_speed_ctrl_x.ki
                              ,maple_ctrl.optical_speed_ctrl_x.kd
                                ,0,0,0);
    nclink_send_ask_flag[4]=0;
    return 1;
  }
  else if(nclink_send_ask_flag[5]==1)//接收到地面站读取PID参数请求，飞控发送第6组PID参数给地面站
  {
    NCLink_Send_PID(NCLINK_SEND_PID16_18,
                    0,0,0,
                    0,0,0,
                    0,0,0);
    nclink_send_ask_flag[5]=0;
    return 1;
  }
  else if(nclink_send_ask_flag[6]==1)//接收到地面站读取其它参数请求，飞控发送其它参数给地面站
  {
    NCLink_Send_Parameter(other_params.params.target_height,
                          other_params.params.safe_height,
                          other_params.params.safe_vbat,
                          other_params.params.max_height,
                          other_params.params.max_radius,
                          other_params.params.max_upvel,
                          other_params.params.max_downvel,
                          other_params.params.max_horvel,
                          other_params.params.reserved_uart,
                          other_params.params.near_ground_height,
                          other_params.params.inner_uart,
                          other_params.params.opticalcal_type);
    nclink_send_ask_flag[6]=0;
    return 1;	
  }
  else if(nclink_send_ask_flag[7]==1)//接收到地面站读取预留参数请求，飞控发送其它参数给地面站
  {
    if(param_id_cnt<RESERVED_PARAM_NUM)
    {			 
      NCLink_Send_Parameter_Reserved(param_id_cnt,param_value[param_id_cnt]);
      nclink_send_ask_flag[7]=0;
    }
    return 1;	
  }
  else if(nclink_send_ask_flag[8]==1)//接收到地面站读取全部预留参数请求，飞控发送其它参数给地面站
  { 
    static uint16_t _cnt=0;
    if(_cnt<RESERVED_PARAM_NUM)
    {
      NCLink_Send_Parameter_Reserved(_cnt,param_value[_cnt]);//依次发送
      _cnt++;
    }
    if(_cnt==RESERVED_PARAM_NUM)//发送完毕
    {
      _cnt=0;
      nclink_send_ask_flag[8]=0;
    }
    return 1;	
  }
  else return 0;
}


; 
void NCLink_SEND_StateMachine(void)
{
  static uint16_t nclink_cnt=0;
  if(!NCLink_Send_Check_Status_Parameter())//判断地面站有无请求数据、是否需要发送应答
  {
    if(nclink_cnt==0)//飞控姿态等基本信息
    {
      nclink_cnt++;
      NCLink_Send_Status(flymaple.rpy_fusion_deg[0],flymaple.rpy_fusion_deg[1],flymaple.rpy_fusion_deg[2],
                         flymaple.gyro_dps.x,flymaple.gyro_dps.y,flymaple.gyro_dps.z,
                         flymaple.t,0,rc_data.height_mode,rc_data.lock_state);
    }
    else if(nclink_cnt==1)//飞控传感器原始数据
    {
      nclink_cnt++;
      NCLink_Send_Sensor((int16_t)(flymaple.accel_raw_data[0]),(int16_t)(flymaple.accel_raw_data[1]),(int16_t)(flymaple.accel_raw_data[2]),
                         (int16_t)(flymaple.gyro_raw_data[0]),(int16_t)(flymaple.gyro_raw_data[1]),(int16_t)(flymaple.gyro_raw_data[2]),
                         (int16_t)(1000*flymaple.mag_raw_data_tesla[0]),(int16_t)(1000*flymaple.mag_raw_data_tesla[1]),(int16_t)(1000*flymaple.mag_raw_data_tesla[2]));
    }
    else if(nclink_cnt==2)//飞控接收遥控器数据
    {
      nclink_cnt++;
      NCLink_Send_RCData(ppm_rc.data[0],ppm_rc.data[1],
                         ppm_rc.data[2],ppm_rc.data[3],
                         ppm_rc.data[4],ppm_rc.data[5],
                         ppm_rc.data[6],ppm_rc.data[7]);
    }
    else if(nclink_cnt==3)//飞控解析GPS信息
    {
      nclink_cnt++;
      NCLink_Send_GPSData(gps_data.gps.lon,gps_data.gps.lat,gps_data.gps.height,
                          ins.gps_pdop,gps_data.gps.fixtype,gps_data.gps.numsv);
    }
    else if(nclink_cnt==4)//飞控水平观测位置、速度
    {
      nclink_cnt++;
      NCLink_Send_Obs_NE(ins.gps_obs_pos_enu[NORTH],ins.gps_obs_pos_enu[EAST],
                         ins.gps_obs_vel_enu[NORTH],ins.gps_obs_vel_enu[EAST]);
    }
    else if(nclink_cnt==5)//飞控竖直观测位置、光流速度
    {
      nclink_cnt++;
      NCLink_Send_Obs_UOP(flymaple.baro_height,tofsense_data.distance,opt_data.x,opt_data.y);
    }
    else if(nclink_cnt==6)//飞控竖直状态估计
    {
      nclink_cnt++;
      NCLink_Send_Fusion_U(ins.position_z,
                           ins.speed_z,
                           ins.acceleration_z);
    }
    else if(nclink_cnt==7)//3D轨迹显示
    {
      nclink_cnt++;
      NCLink_Send_3D_Track(ins.opt.position.x,
                           ins.opt.position.y,
                           ins.position_z,
                           flymaple.quaternion[0],
                           flymaple.quaternion[1],
                           flymaple.quaternion[2],
                           flymaple.quaternion[3]);
    }
    else if(nclink_cnt==8)//飞控水平状态估计
    {
      nclink_cnt++;
      NCLink_Send_Fusion_NE(ins.position[_NORTH],
                            ins.position[_EAST],
                            ins.speed[_NORTH],
                            ins.speed[_EAST],
                            ins.acceleration[_NORTH],
                            ins.acceleration[_EAST]);		
    }
    else if(nclink_cnt==9)//用户数据波形显示
    {
      //nclink_cnt++;     
      NCLink_Send_Userdata(vel_z_fusion,
                           tofsense_data.distance,
                           //ins.obs_pos_backups[_UP][2],
                           //ins.pos_backups[_UP][10+ins.alt_obs_sync_ms/10],
                           ins.speed[_UP],
                           ins.position[_UP],
                           flymaple.baro_height,
                           pos_z_fusion);

      //NCLink_Send_Userdata(flymaple.gyro_dps.z,flymaple.gyro_raw_data[2],flymaple.gyro_raw_filter_data.z,flymaple.gyro_offset.z,flymaple.gyro_raw_data[0],0);
      //			NCLink_Send_Userdata(maple_ctrl.pitch_gyro_ctrl.feedback,
      //													 maple_ctrl.roll_gyro_ctrl.feedback,
      //													 maple_ctrl.yaw_gyro_ctrl.feedback,
      //													 maple_ctrl.pitch_gyro_ctrl.dis_err,
      //													 maple_ctrl.pitch_gyro_ctrl.derivative,
      //													 maple_ctrl.pitch_gyro_ctrl.dis_error_history[0]);
      //			NCLink_Send_Userdata(flymaple.accel_raw_data_g[0],
      //													 flymaple.accel_raw_data_g[1],
      //													 flymaple.accel_raw_data_g[2],
      //													 flymaple.gyro_raw_data_dps[0],
      //													 flymaple.gyro_raw_data_dps[1],
      //													 flymaple.gyro_raw_data_dps[2]);
      //			NCLink_Send_Userdata(	ins.opt.position.x,
      //														ins.opt.position.y,
      //														ins.opt.speed.x,
      //														ins.opt.speed.y,
      //														ins.opt.speed_obs.x,
      //														ins.opt.speed_obs.y);
      
//          NCLink_Send_Userdata(opt_data.flow.x,
//                                opt_data.flow_filter.x,	
//                                flymaple.gyro_dps.y*DEG2RAD,
//                                opt_data.gyro_filter.x,
//                                opt_data.flow_correct.x,
//                                0);
  
//          NCLink_Send_Userdata(ins.opt.position.x,
//                                ins.opt.position.y,	
//                                ins.opt.speed.x,
//                                ins.opt.speed.y,
//                                ins.opt.speed_obs.x,
//                                ins.opt.speed_obs.y);  
      //			NCLink_Send_Userdata(opt_data.x,
      //													 opt_data.y,	
      //													 opt_data.r,
      //													 opt_data.ground_distance,
      //													 opt_data.flow_correct.y,
      //													 0);
      //			NCLink_Send_Userdata(flymaple.accel_raw_data_g[2],
      //													 flymaple.accel.z,
      //													 flymaple.accel_fb.z,
      //													 ins.acceleration_initial[_UP],
      //													 ins.accel_feedback[_UP],
      //													 -maple_ctrl.height_accel_ctrl.err_lpf);
      
      //				NCLink_Send_Userdata(maple_ctrl.pitch_gyro_ctrl.expect_div,
      //													   maple_ctrl.pitch_gyro_ctrl.feedback_div,
      //													   maple_ctrl.pitch_gyro_ctrl.dis_err,
      //													   maple_ctrl.pitch_gyro_ctrl.dis_error_history[0],
      //													   maple_ctrl.pitch_gyro_ctrl.derivative,
      //													   maple_ctrl.pitch_gyro_ctrl.combine_div														 
      //														 );
      
      
    }
    else if(nclink_cnt==10)//传感器校准原始数据
    {
      nclink_cnt++;
      NCLink_Send_CalRawdata1(flymaple.imu_gyro_calibration_flag,
                              flymaple.gyro_raw_data[0],
                              flymaple.gyro_raw_data[1],
                              flymaple.gyro_raw_data[2],
                              flymaple.accel_raw_data[0],
                              flymaple.accel_raw_data[1],
                              flymaple.accel_raw_data[2]);
    }
    else if(nclink_cnt==11)//传感器校准原始数据
    {
      nclink_cnt=0;
      NCLink_Send_CalRawdata2(flymaple.mag_raw_data_tesla[0],flymaple.mag_raw_data_tesla[1],flymaple.mag_raw_data_tesla[2]);
    }
    else nclink_cnt=0;
  }
}



/************************************************************************************/


