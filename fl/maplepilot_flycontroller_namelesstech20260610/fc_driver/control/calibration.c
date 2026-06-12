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
#include "wp_math.h"
#include "datatype.h"
#include "routines.h"
#include "rc.h"
#include "nclink.h"
#include "drv_w25qxx.h"
#include "parameter_server.h"
#include "schedule.h"
#include "quaternion.h"
#include "drv_oled.h"
#include "drv_ppm.h"
#include "attitude_ctrl.h"
#include "drv_notify.h"
#include "drv_expand.h"
#include "calibration.h"
/*					
                  左遥杆        右遥杆 
加速度计校准：    左下          右上
磁力计校准  ：    左下          左上
机架水平校准：    左下          右下
陀螺仪校准：      左下          中上
退出当前校准：    左下          中下
*/

flight_cal_flag  flight_calibrate={
  .period_unit=5,
  .gyro_cal_flag=1,//若初始化置1，默认开机需要校准陀螺仪
};


int16_t cal_number=0;


void cal_start_beep(void)
{
  beep.period=20;//20*5ms
  beep.light_on_percent=0.5f;
  beep.reset=1;	
  beep.times=2;
}

void cal_finish_beep(void)
{
  beep.period=40;//40*5ms
  beep.light_on_percent=0.5f;
  beep.reset=1;	
  beep.times=3;
}

void cal_step_beep(void)
{
  beep.period=20;//20*5ms
  beep.light_on_percent=0.5f;
  beep.reset=1;	
  beep.times=1;
}

void rc_accel_cal_check(void);
void rc_range_calibration(rc *data)
{
  if(flight_calibrate.rc_cal_flag==0)  return;//当地面站未发送遥控器校准指令时，直接返回，不记录最大最小值
  for(uint16_t i=0;i<8;i++)//记录遥控器行程的最大最小值
  {
    if(data->rcdata[i]>(data->cal[i].max_value)) data->cal[i].max_value=data->rcdata[i];//最大行程值
    if(data->rcdata[i]<(data->cal[i].min_value)) data->cal[i].min_value=data->rcdata[i];//最小行程值
  }
}

void rc_cal_duty(void)
{
  //遥控器行程校准
  if(cal_state.cal_flag==0x05)
  {
    rc *data=&rc_data;
    for(uint16_t i=0;i<8;i++)//复位最大最小值为中位1500
    {
      data->cal[i].max_value=1500;//最大行程值
      data->cal[i].min_value=1500;//最小行程值
    }
    flight_calibrate.rc_cal_flag=1;
    cal_state.cal_flag=0x00;
    rgb_notify_set(BLUE,TOGGLE,2000,0xffffffff,500);
  }
  //遥控器校准保存
  if(cal_state.cal_flag==0x06)
  {
    rc *data=&rc_data;
    if(flight_calibrate.rc_cal_flag==1)
    {
      for(uint16_t i=0;i<8;i++)
      {
        WriteFlashParameter_Two(RC_CH1_MAX+2*i,
                                data->cal[i].max_value,
                                data->cal[i].min_value,
                                &Flight_Params);		
      }
      flight_calibrate.rc_cal_flag=0;
    }
    cal_state.cal_flag=0x00;
    rgb_notify_set(BLUE,TOGGLE,500,4000,500);
  }
}


extern int8_t imu_sensor_type;
/////////////////////////////////////////////////////////////
/*第一面飞控平放，Z轴正向朝着正上方，Z axis is about 1g,X、Y is about 0g*/
/*第二面飞控平放，X轴正向朝着正上方，X axis is about 1g,Y、Z is about 0g*/
/*第三面飞控平放，X轴正向朝着正下方，X axis is about -1g,Y、Z is about 0g*/
/*第四面飞控平放，Y轴正向朝着正下方，Y axis is about -1g,X、Z is about 0g*/
/*第五面飞控平放，Y轴正向朝着正上方，Y axis is about 1g,X、Z is about 0g*/
/*第六面飞控平放，Z轴正向朝着正下方，Z axis is about -1g,X、Y is about 0g*/
vector3f acce_sample[6]={0};//三行6列，保存6面待矫正数据
vector3f new_offset={0,0,0};
vector3f new_scales={1.0,1.0,1.0,};
void accel_calibration(float gyro_total)
{
  uint16_t ACCEL_X_OFFSET=0,ACCEL_X_SCALE=0;
  if(imu_sensor_type==0)
  {
    ACCEL_X_OFFSET=ACCEL_X_OFFSET1;
    ACCEL_X_SCALE=ACCEL_X_SCALE1;
  }
  else if(imu_sensor_type==1)
  {
    ACCEL_X_OFFSET=ACCEL_X_OFFSET2;
    ACCEL_X_SCALE=ACCEL_X_SCALE2;	
  }
  else if(imu_sensor_type==2)
  {
    ACCEL_X_OFFSET=ACCEL_X_OFFSET3;
    ACCEL_X_SCALE=ACCEL_X_SCALE3;	
  }
  vector3f accel_cal={0,0,0};
  accel_cal.x=flymaple.accel_fb_filter_data.x/flymaple.accel_scale_raw_to_g;
  accel_cal.y=flymaple.accel_fb_filter_data.y/flymaple.accel_scale_raw_to_g;
  accel_cal.z=flymaple.accel_fb_filter_data.z/flymaple.accel_scale_raw_to_g;
  
  if(flight_calibrate.accel_cal_flag==0)  return;//当地面站未发送加速度计校准指令时，直接返回
  rc_accel_cal_check();
  vector3f acce_sample_sum={0,0,0};//加速度和数据
  if(cal_state.cal_step==0x01//飞控水平放置
     ||cal_state.cal_step==0x02//飞机右侧朝上
       ||cal_state.cal_step==0x03//飞机左侧朝上
	 ||cal_state.cal_step==0x04//飞机机头朝下
           ||cal_state.cal_step==0x05//飞机机头朝上
             ||cal_state.cal_step==0x06)//飞机翻面放置  
  {
    uint16_t _cnt=0;
    while(_cnt<200)
    {  
      if(gyro_total<10.0f)
      {
        delay_ms(5);
        acce_sample_sum.x+=accel_cal.x*GRAVITY_MSS;//加速度计转化为m/s^2量程下
        acce_sample_sum.y+=accel_cal.y*GRAVITY_MSS;
        acce_sample_sum.z+=accel_cal.z*GRAVITY_MSS;			
        _cnt++;
      }
    }
    acce_sample[cal_state.cal_step-1].x=acce_sample_sum.x/_cnt;
    acce_sample[cal_state.cal_step-1].y=acce_sample_sum.y/_cnt;
    acce_sample[cal_state.cal_step-1].z=acce_sample_sum.z/_cnt;
    flight_calibrate.accel_cal_finished[cal_state.cal_step-1]=1;//对应面加速度计完成校准标志位
    cal_state.cal_step=0x00;
  }
  
  if(flight_calibrate.accel_cal_finished[0]
     &&flight_calibrate.accel_cal_finished[1]
       &&flight_calibrate.accel_cal_finished[2]
	 &&flight_calibrate.accel_cal_finished[3]
           &&flight_calibrate.accel_cal_finished[4]
             &&flight_calibrate.accel_cal_finished[5])//6面全部校准完毕
  {
    flight_calibrate.accel_cal_total_finished=Calibrate_accel(acce_sample,
                                                              &new_offset,
                                                              &new_scales);//将采集的加速度数据,进行6面校准
    flight_calibrate.accel_cal_flag=0;
    for(uint16_t i=0;i<6;i++)
    {
      flight_calibrate.accel_cal_finished[i]=0;//单面加速度计校准完成标志位
    }
    
    if(flight_calibrate.accel_cal_total_finished==true)//加速度计校准成功
    {
      WriteFlashParameter_Three(ACCEL_X_OFFSET,
                                new_offset.x,
                                new_offset.y,
                                new_offset.z,
                                &Flight_Params);
      WriteFlashParameter_Three(ACCEL_X_SCALE,
                                new_scales.x,
                                new_scales.y,
                                new_scales.z,
                                &Flight_Params);
      rgb_notify_set(BLUE,TOGGLE,2000,4000,500);
      
      flymaple.accel_scale.x=new_scales.x;
      flymaple.accel_scale.y=new_scales.y;
      flymaple.accel_scale.z=new_scales.z;
      flymaple.accel_offset.x=new_offset.x;
      flymaple.accel_offset.y=new_offset.y;
      flymaple.accel_offset.z=new_offset.z;
      nclink_send_check_flag[9]=0x01;
    }
    else
    {
      rgb_notify_set(BLUE,TOGGLE,1000,4000,500);
    }
  }
}
void accel_cal_duty(void)
{
  //加速度计校准开始
  if(cal_state.cal_flag==0x02)
  {
    flight_calibrate.accel_cal_flag=1;
    flight_calibrate.accel_cal_total_finished=0;
    for(uint16_t i=0;i<6;i++)
    {
      flight_calibrate.accel_cal_finished[i]=0;//单面加速度计校准完成标志位
      acce_sample[i].x=0;
      acce_sample[i].y=0;
      acce_sample[i].z=0;
    }
    cal_state.cal_flag=0x00;		
    rgb_notify_set(BLUE,TOGGLE,2000,0xffffffff,500);
  }	
}


void gyro_cal_duty(void)
{
  //陀螺仪校准开始
  if(cal_state.cal_flag==0x01)
  {
    flight_calibrate.gyro_cal_flag=1;
    cal_state.cal_flag=0x00;		
    rgb_notify_set(BLUE,TOGGLE,2000,0xffffffff,500);
    
    flymaple.imu_gyro_calibration_flag=0;//温度校准就位置0，校准完成后置1
  }	
}

void gyro_calibration(uint8_t *finish)
{
  uint16_t GYRO_X_OFFSET=0;
  if(imu_sensor_type==0)
  {
    GYRO_X_OFFSET=GYRO_X_OFFSET1;
  }
  else if(imu_sensor_type==1)
  {
    GYRO_X_OFFSET=GYRO_X_OFFSET2;
  }
  else if(imu_sensor_type==2)
  {
    GYRO_X_OFFSET=GYRO_X_OFFSET3;
  }	
  //低通滤波后再采集校准
  static vector3f pre_gyro,curr_gyro;
  pre_gyro=curr_gyro;
  curr_gyro.x=pre_gyro.x+0.05f*(flymaple.gyro_raw_filter_data.x-pre_gyro.x);
  curr_gyro.y=pre_gyro.y+0.05f*(flymaple.gyro_raw_filter_data.y-pre_gyro.y);
  curr_gyro.z=pre_gyro.z+0.05f*(flymaple.gyro_raw_filter_data.z-pre_gyro.z);
  
  if(flight_calibrate.gyro_cal_flag==0)  return;//当地面站未发送陀螺仪校准指令时，直接返回
  
  static uint16_t cnt=0;
  static float gyro_x_last = 0, gyro_y_last = 0, gyro_z_last = 0;		
  static float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
  
  if(ABS(curr_gyro.x-gyro_x_last)*flymaple.gyro_scale_raw_to_dps<=3.0f
     &&ABS(curr_gyro.y-gyro_y_last)*flymaple.gyro_scale_raw_to_dps<=3.0f
       &&ABS(curr_gyro.z-gyro_z_last)*flymaple.gyro_scale_raw_to_dps<=3.0f
	 &&imu_temperature_stable()==true)
  {
    gyro_x_offset +=curr_gyro.x;
    gyro_y_offset +=curr_gyro.y;
    gyro_z_offset +=curr_gyro.z;
    cnt++;
  }
  else
  {
    gyro_x_offset=0;
    gyro_y_offset=0;
    gyro_z_offset=0;
    cnt=0;
  }	
  gyro_x_last=curr_gyro.x;
  gyro_y_last=curr_gyro.y;
  gyro_z_last=curr_gyro.z;
  
  if(cnt>=200)
  {
    *finish=1;//校准完成
    flymaple.gyro_offset.x =(gyro_x_offset/cnt);//得到标定偏移
    flymaple.gyro_offset.y =(gyro_y_offset/cnt);
    flymaple.gyro_offset.z =(gyro_z_offset/cnt);	
    flight_calibrate.gyro_cal_flag=0;		
    WriteFlashParameter_Three(GYRO_X_OFFSET,
                              flymaple.gyro_offset.x,
                              flymaple.gyro_offset.y,
                              flymaple.gyro_offset.z,
                              &Flight_Params);
    rgb_notify_set(BLUE,TOGGLE,1000,5000,500);
    nclink_send_check_flag[9]=0x01;	
    gyro_x_offset=0;
    gyro_y_offset=0;
    gyro_z_offset=0;
    cnt=0;   
    
    cal_finish_beep();
  }
}

/////////////////////////////////////////////////////////////
const uint16_t mag_sample_point[36]={
  0  ,10 ,20 ,30 ,40 ,50 ,60 ,70 ,80 , 90,100,110,
  120,130,140,150,160,170,180,190,200,210,220,230,
  240,250,260,270,280,290,300,310,320,330,340,350};//磁力计矫正遍历角度，确保数据采集充分
uint8_t mag_sample_flag[3][36],mag_sample_flag_last[3][36];
vector3f new_mag_offset={0,0,0};
float new_mag_radius=0;
void mag_cal_duty(void)
{
  //磁力计校准开始
  if(cal_state.cal_flag==0x03)
  {
    flight_calibrate.mag_cal_flag=1;
    flight_calibrate.mag_cal_total_finished=0;
    for(uint16_t i=0;i<3;i++)
    {
      flight_calibrate.mag_cal_finished[i]=0;
      flight_calibrate.mag_cal_finished_last[i]=0;
    }
    
    for(uint16_t i=0;i<36;i++)
    {
      mag_sample_flag_last[0][i]=mag_sample_flag[0][i]=0;
      mag_sample_flag_last[1][i]=mag_sample_flag[1][i]=0;
      mag_sample_flag_last[2][i]=mag_sample_flag[2][i]=0;		
    }
    cal_state.cal_flag=0x00;		
    rgb_notify_set(BLUE,TOGGLE,2000,0xffffffff,500);
  }	
}

uint16_t  mag_cal_total_cnt=0;
void mag_cal_show(void);
void mag_calibration(void)
{
  uint16_t mag_cal_cnt[3]={0};
  if(flight_calibrate.mag_cal_flag==0)  return;//当地面站未发送磁力计校准指令时，直接返回
  for(uint16_t i=0;i<36;i++)
  {
    mag_sample_flag_last[0][i]=mag_sample_flag[0][i];
    mag_sample_flag_last[1][i]=mag_sample_flag[1][i];
    mag_sample_flag_last[2][i]=mag_sample_flag[2][i];		
  }
  
  if(cal_state.cal_step==0x01) //飞控水平旋转
  {
    for(uint16_t i=0;i<36;i++)
    {
      if(ABS(flymaple.rpy_integral_deg[2]-mag_sample_point[i])<5.0f)
      {
        mag_sample_flag[2][i]=1;
      }
    }
  }
  
  if(cal_state.cal_step==0x02) //飞机机头朝上旋转
  {
    for(uint16_t i=0;i<36;i++)
    {
      if(ABS(flymaple.rpy_integral_deg[0]-mag_sample_point[i])<5.0f)
      {
        mag_sample_flag[0][i]=1;
      }
    }
  }	 
  
  if(cal_state.cal_step==0x03) //飞机右侧朝上旋转
  {
    for(uint16_t i=0;i<36;i++)
    {
      if(ABS(flymaple.rpy_integral_deg[1]-mag_sample_point[i])<5.0f)
      {
        mag_sample_flag[1][i]=1;
      }
    }
  }	 
  
  for(uint16_t i=0;i<36;i++)
  {
    if((mag_sample_flag_last[0][i]==0&&mag_sample_flag[0][i]==1)
       ||(mag_sample_flag_last[1][i]==0&&mag_sample_flag[1][i]==1)
         ||(mag_sample_flag_last[2][i]==0&&mag_sample_flag[2][i]==1))
    {
      LS_Accumulate(&Mag_LS,flymaple.mag_raw_data_tesla[0],
                    flymaple.mag_raw_data_tesla[1],
                    flymaple.mag_raw_data_tesla[2]);
    }
    if(mag_sample_flag[0][i]==1) mag_cal_cnt[0]++;
    if(mag_sample_flag[1][i]==1) mag_cal_cnt[1]++;	
    if(mag_sample_flag[2][i]==1) mag_cal_cnt[2]++;			
  }
  mag_cal_total_cnt=mag_cal_cnt[0]+mag_cal_cnt[1]+mag_cal_cnt[2];
  
  
  flight_calibrate.mag_cal_finished_last[0]=flight_calibrate.mag_cal_finished[0];
  flight_calibrate.mag_cal_finished_last[1]=flight_calibrate.mag_cal_finished[1];
  flight_calibrate.mag_cal_finished_last[2]=flight_calibrate.mag_cal_finished[2];	
  if(mag_cal_cnt[0]==36) flight_calibrate.mag_cal_finished[0]=1;
  if(mag_cal_cnt[1]==36) flight_calibrate.mag_cal_finished[1]=1;
  if(mag_cal_cnt[2]==36) flight_calibrate.mag_cal_finished[2]=1;
  
  
  if(flight_calibrate.mag_cal_finished_last[2]==0&&flight_calibrate.mag_cal_finished[2]==1)//第一面校准完毕
  {
    cal_state.cal_step=0x02;
    rgb_notify_set(BLUE,TOGGLE,500,2000,500);
  }	
  
  
  if(flight_calibrate.mag_cal_finished_last[0]==0&&flight_calibrate.mag_cal_finished[0]==1)//第二面校准完毕
  {
    cal_state.cal_step=0x03;
    rgb_notify_set(BLUE,TOGGLE,500,2000,500);
  }
  
  if(flight_calibrate.mag_cal_finished_last[1]==0&&flight_calibrate.mag_cal_finished[1]==1)//第三面校准完毕
  {
    cal_state.cal_step=0x01;
    rgb_notify_set(BLUE,TOGGLE,500,2000,500);
  }
  
  
  if(flight_calibrate.mag_cal_finished[0]==1
     &&flight_calibrate.mag_cal_finished[1]==1
       &&flight_calibrate.mag_cal_finished[2]==1)//三面数据全部采集完毕，计算磁力计零点
  {
    LS_Calculate(&Mag_LS,mag_cal_total_cnt,0.0f,&new_mag_offset.x,&new_mag_offset.y,&new_mag_offset.z,&new_mag_radius);
    flight_calibrate.mag_cal_total_finished=1;
    flight_calibrate.mag_cal_flag=0;
    WriteFlashParameter_Three(MAG_X_OFFSET,
                              new_mag_offset.x,
                              new_mag_offset.y,
                              new_mag_offset.z,
                              &Flight_Params);
    rgb_notify_set(BLUE,TOGGLE,1000,5000,500);
    
    flymaple.mag_offset.x=new_mag_offset.x;
    flymaple.mag_offset.y=new_mag_offset.y;
    flymaple.mag_offset.z=new_mag_offset.z;
    nclink_send_check_flag[9]=0x01;
    cal_finish_beep();
  }	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void hor_cal_duty(void)
{
  //校准机架水平开始
  if(cal_state.cal_flag==0x04)
  {
    flight_calibrate.hor_cal_flag=1;
    cal_state.cal_flag=0x00;
    rgb_notify_set(BLUE,TOGGLE,2000,0xffffffff,500);
  }	
}

void hor_calibration(void)
{
  uint16_t PITCH_OFFSET=0,ROLL_OFFSET=0,HOR_CAL_ACCEL_X=0,HOR_CAL_ACCEL_Y=0,HOR_CAL_ACCEL_Z=0;
  if(imu_sensor_type==0)
  {
    PITCH_OFFSET=PITCH_OFFSET1;
    ROLL_OFFSET=ROLL_OFFSET1;
    HOR_CAL_ACCEL_X=HOR_CAL_ACCEL_X1;
    HOR_CAL_ACCEL_Y=HOR_CAL_ACCEL_Y1;
    HOR_CAL_ACCEL_Z=HOR_CAL_ACCEL_Z1;
  }
  else if(imu_sensor_type==1)
  {
    PITCH_OFFSET=PITCH_OFFSET2;
    ROLL_OFFSET=ROLL_OFFSET2;	
    HOR_CAL_ACCEL_X=HOR_CAL_ACCEL_X2;
    HOR_CAL_ACCEL_Y=HOR_CAL_ACCEL_Y2;
    HOR_CAL_ACCEL_Z=HOR_CAL_ACCEL_Z2;
  }
  else if(imu_sensor_type==2)
  {
    PITCH_OFFSET=PITCH_OFFSET3;
    ROLL_OFFSET=ROLL_OFFSET3;	
    HOR_CAL_ACCEL_X=HOR_CAL_ACCEL_X3;
    HOR_CAL_ACCEL_Y=HOR_CAL_ACCEL_Y3;
    HOR_CAL_ACCEL_Z=HOR_CAL_ACCEL_Z3;
  }	 
  
  float rp[2]={0};
  float acce_sample_sum[3]={0,0,0};
  if(flight_calibrate.hor_cal_flag==0)  return;//当地面站未发送机架水平校准指令时，直接返回
  uint16_t _cnt=0;
  while(_cnt<200)
  {
    if(flymaple.total_gyro_dps<20.0f)
    {
      delay_ms(5);
      rp[0]+=flymaple.rpy_fusion_deg[ROLL];
      rp[1]+=flymaple.rpy_fusion_deg[PITCH];
      acce_sample_sum[0]+=flymaple.accel_fb_filter_data.x; 
      acce_sample_sum[1]+=flymaple.accel_fb_filter_data.y;
      acce_sample_sum[2]+=flymaple.accel_fb_filter_data.z;
      _cnt++;
    }
  }
  rp[0]/=_cnt;
  rp[1]/=_cnt;		
  WriteFlashParameter(PITCH_OFFSET,rp[1],&Flight_Params);
  WriteFlashParameter(ROLL_OFFSET ,rp[0],&Flight_Params);
  
  flymaple.rpy_angle_offset[ROLL] =rp[0];
  flymaple.rpy_angle_offset[PITCH]=rp[1];
  flight_calibrate.hor_cal_flag=0;

  float RANGE_TO_MSS=GRAVITY_MSS/flymaple.accel_scale_raw_to_g;
  flymaple.accel_hor_offset.x=(acce_sample_sum[0]/_cnt)*RANGE_TO_MSS;
  flymaple.accel_hor_offset.y=(acce_sample_sum[1]/_cnt)*RANGE_TO_MSS;
  flymaple.accel_hor_offset.z=(acce_sample_sum[2]/_cnt-flymaple.accel_scale_raw_to_g)*RANGE_TO_MSS;
  WriteFlashParameter(HOR_CAL_ACCEL_X,flymaple.accel_hor_offset.x,&Flight_Params);
  WriteFlashParameter(HOR_CAL_ACCEL_Y,flymaple.accel_hor_offset.y,&Flight_Params);
  WriteFlashParameter(HOR_CAL_ACCEL_Z,flymaple.accel_hor_offset.z,&Flight_Params);
  
/****************************************************************************/    
  imu_calibration_init_from_server();
/****************************************************************************/    
  rgb_notify_set(BLUE,TOGGLE,1000,5000,500);
  nclink_send_check_flag[9]=0x01;
  float _rpy[3];
  _rpy[1]=DEGTORAD(flymaple.rpy_angle_offset[ROLL]);
  _rpy[0]=DEGTORAD(flymaple.rpy_angle_offset[PITCH]);
  _rpy[2]=0;
  euler_to_quaternion(_rpy,flymaple.quaternion_zero);//零点四元数  
  cal_finish_beep();
}





void quit_all_cal_check(void)
{
  if(cal_state.cal_flag==0
     &&cal_state.cal_step==0
       &&cal_state.cal_cmd==0)//退出所有校准
  {
    flight_calibrate.rc_cal_flag=0;
    flight_calibrate.accel_cal_flag=0;
    flight_calibrate.mag_cal_flag=0;
    flight_calibrate.hor_cal_flag=0;
    flight_calibrate.gyro_cal_flag=0;
    
    flight_calibrate.accel_cal_makesure_cnt=0;
    flight_calibrate.mag_cal_total_finished=0;
    flight_calibrate.accel_cal_total_finished=0;
    flight_calibrate.hor_cal_makesure_cnt=0;
    
    for(uint16_t i=0;i<6;i++)
    {
      flight_calibrate.accel_cal_finished[i]=0;
      flight_calibrate.accel_step_makesure_cnt[i]=0;
    }
    for(uint16_t i=0;i<3;i++)
    {
      flight_calibrate.mag_cal_finished[i]=0;
      flight_calibrate.mag_cal_finished_last[i]=0;
    }
    rgb_notify_set(BLUE,TOGGLE,500,4000,500);
    
    cal_finish_beep();
  }
}


void check_calibration(void)
{
  if(cal_state.update_flag==true)
  {
    quit_all_cal_check();
    rc_cal_duty();
    accel_cal_duty();
    mag_cal_duty();
    hor_cal_duty();
    gyro_cal_duty();
    cal_state.update_flag=false;	
  }
  if(flight_calibrate.rc_cal_flag|flight_calibrate.accel_cal_flag
     |flight_calibrate.mag_cal_flag|flight_calibrate.hor_cal_flag
       |flight_calibrate.gyro_cal_flag)
  {
    flight_calibrate.free=0;
  }
  else flight_calibrate.free=1;
  
  if(rc_data.lock_state!=LOCK) return;	
  
  /**************************************************************************************************************************/	
  //加速度校准进入逻辑
  if(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
     &&rc_data.rc_rpyt[RC_ROLL]>=Pit_Rol_Max*Scale_Pecent_Max
       &&rc_data.rc_rpyt[RC_PITCH]<=-Pit_Rol_Max*Scale_Pecent_Max
	 &&rc_data.rc_rpyt[RC_YAW]>=Yaw_Max*Scale_Pecent_Max)
    flight_calibrate.accel_cal_makesure_cnt++;
  else flight_calibrate.accel_cal_makesure_cnt/=5;
  /**************************************************************************************************************************/	
  if(flight_calibrate.accel_cal_makesure_cnt>2000/flight_calibrate.period_unit)//持续2S
  {
    cal_state.update_flag=true;
    cal_state.cal_flag=0x02;
    flight_calibrate.accel_cal_makesure_cnt=0;
    rgb_notify_set(BLUE,TOGGLE,2000,0xffff,500);
    cal_start_beep();
  }
  /**************************************************************************************************************************/	
  //磁力计校准进入逻辑
  if(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
     &&rc_data.rc_rpyt[RC_ROLL]<-Pit_Rol_Max*Scale_Pecent_Max
       &&rc_data.rc_rpyt[RC_PITCH]<=-Pit_Rol_Max*Scale_Pecent_Max
	 &&rc_data.rc_rpyt[RC_YAW]>=Yaw_Max*Scale_Pecent_Max)
    flight_calibrate.mag_cal_makesure_cnt++;
  else flight_calibrate.mag_cal_makesure_cnt/=5;
  
  if(flight_calibrate.mag_cal_makesure_cnt>2000/flight_calibrate.period_unit)//持续2S
  {
    cal_state.update_flag=true;
    cal_state.cal_flag=0x03;
    cal_state.cal_step=0x01;
    flight_calibrate.mag_cal_makesure_cnt=0;
    rgb_notify_set(BLUE,TOGGLE,2000,0xffff,500);
    cal_start_beep();
  }
  /**************************************************************************************************************************/
  //水平校准进入逻辑
  if(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
     &&rc_data.rc_rpyt[RC_ROLL]>=Pit_Rol_Max*Scale_Pecent_Max
       &&rc_data.rc_rpyt[RC_PITCH]>=Pit_Rol_Max*Scale_Pecent_Max
	 &&rc_data.rc_rpyt[RC_YAW]>=Yaw_Max*Scale_Pecent_Max)
    flight_calibrate.hor_cal_makesure_cnt++;
  else flight_calibrate.hor_cal_makesure_cnt/=5;
  
  if(flight_calibrate.hor_cal_makesure_cnt>2000/flight_calibrate.period_unit)//持续2S
  {
    cal_state.update_flag=true;
    cal_state.cal_flag=0x04;
    flight_calibrate.hor_cal_makesure_cnt=0;
    rgb_notify_set(BLUE,TOGGLE,2000,0xffffffff,500);
    //cal_start_beep();机架水平校准时蜂鸣器震动会影响校准
  }
  /**************************************************************************************************************************/
  //陀螺仪校准进入逻辑
  if(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
     &&rc_data.rc_rpyt[RC_ROLL]==0
       &&rc_data.rc_rpyt[RC_PITCH]<=-Pit_Rol_Max*Scale_Pecent_Max
	 &&rc_data.rc_rpyt[RC_YAW]>=Yaw_Max*Scale_Pecent_Max)
    flight_calibrate.gyro_cal_makesure_cnt++;
  else flight_calibrate.gyro_cal_makesure_cnt/=5;
  
  if(flight_calibrate.gyro_cal_makesure_cnt>2000/flight_calibrate.period_unit)//持续2S
  {
    cal_state.update_flag=true;
    cal_state.cal_flag=0x01;
    flight_calibrate.gyro_cal_makesure_cnt=0;
    rgb_notify_set(BLUE,TOGGLE,2000,0xffffffff,500);
    //cal_start_beep();陀螺仪校准时蜂鸣器震动会影响校准
  }	
  
  
  
  ////////////////////////////////////////////////////////
  //退出所有校准
  if(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
     &&rc_data.rc_rpyt[RC_ROLL]==0
       &&rc_data.rc_rpyt[RC_PITCH]>=Pit_Rol_Max*Scale_Pecent_Max
	 &&rc_data.rc_rpyt[RC_YAW]>=Yaw_Max*Scale_Pecent_Max)
    flight_calibrate.quit_all_cal_cnt++;
  else flight_calibrate.quit_all_cal_cnt/=5;
  if(flight_calibrate.quit_all_cal_cnt>5000/flight_calibrate.period_unit)//持续5000MS
  {
    cal_state.cal_flag=0;
    cal_state.cal_step=0;
    cal_state.cal_cmd=0;
    cal_state.update_flag=1;
    
    flight_calibrate.rc_cal_flag=0;
    flight_calibrate.accel_cal_flag=0;
    flight_calibrate.mag_cal_flag=0;
    flight_calibrate.hor_cal_flag=0;
    flight_calibrate.gyro_cal_flag=0;	
  }	
}	


void rc_accel_cal_check(void)
{
  //第一面
  if(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
     &&rc_data.rc_rpyt[RC_ROLL]==0
       &&rc_data.rc_rpyt[RC_PITCH]==0
	 &&rc_data.rc_rpyt[RC_YAW]<=-Yaw_Max*Scale_Pecent_Max)
    flight_calibrate.accel_step_makesure_cnt[0]++;
  else flight_calibrate.accel_step_makesure_cnt[0]/=5;
  if(flight_calibrate.accel_step_makesure_cnt[0]>100/flight_calibrate.period_unit)//持续100MS
  {
    cal_state.cal_step=0x01;//飞控水平放置
    flight_calibrate.accel_step_makesure_cnt[0]=0;
    rgb_notify_set(RED	,TOGGLE	 ,500,2000,0);
    cal_step_beep();
  }
  //第二面
  if(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
     &&rc_data.rc_rpyt[RC_ROLL]<=-Pit_Rol_Max*Scale_Pecent_Max
       &&rc_data.rc_rpyt[RC_PITCH]==0
	 &&rc_data.rc_rpyt[RC_YAW]==0)
    flight_calibrate.accel_step_makesure_cnt[1]++;
  else flight_calibrate.accel_step_makesure_cnt[1]/=5;
  if(flight_calibrate.accel_step_makesure_cnt[1]>100/flight_calibrate.period_unit)//持续100MS
  {
    cal_state.cal_step=0x02;//飞机右侧朝上
    flight_calibrate.accel_step_makesure_cnt[1]=0;
    rgb_notify_set(RED	,TOGGLE	 ,500,2000,0);
    cal_step_beep();
  }	
  //第三面
  if(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
     &&rc_data.rc_rpyt[RC_ROLL]>=Pit_Rol_Max*Scale_Pecent_Max
       &&rc_data.rc_rpyt[RC_PITCH]==0
	 &&rc_data.rc_rpyt[RC_YAW]==0)
    flight_calibrate.accel_step_makesure_cnt[2]++;
  else flight_calibrate.accel_step_makesure_cnt[2]/=5;
  if(flight_calibrate.accel_step_makesure_cnt[2]>100/flight_calibrate.period_unit)//持续100MS
  {
    cal_state.cal_step=0x03;//飞机左侧朝上
    flight_calibrate.accel_step_makesure_cnt[2]=0;
    rgb_notify_set(RED	,TOGGLE	 ,500,2000,0);
    cal_step_beep();
  }	
  //第四面
  if(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
     &&rc_data.rc_rpyt[RC_ROLL]==0
       &&rc_data.rc_rpyt[RC_PITCH]<=-Pit_Rol_Max*Scale_Pecent_Max
	 &&rc_data.rc_rpyt[RC_YAW]==0)
    flight_calibrate.accel_step_makesure_cnt[3]++;
  else flight_calibrate.accel_step_makesure_cnt[3]/=5;
  if(flight_calibrate.accel_step_makesure_cnt[3]>100/flight_calibrate.period_unit)//持续100MS
  {
    cal_state.cal_step=0x04;//飞机机头朝下
    flight_calibrate.accel_step_makesure_cnt[3]=0;
    rgb_notify_set(RED	,TOGGLE	 ,500,2000,0);
    cal_step_beep();
  }
  //第五面
  if(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
     &&rc_data.rc_rpyt[RC_ROLL]==0
       &&rc_data.rc_rpyt[RC_PITCH]>=Pit_Rol_Max*Scale_Pecent_Max
	 &&rc_data.rc_rpyt[RC_YAW]==0)
    flight_calibrate.accel_step_makesure_cnt[4]++;
  else flight_calibrate.accel_step_makesure_cnt[4]/=5;
  if(flight_calibrate.accel_step_makesure_cnt[4]>100/flight_calibrate.period_unit)//持续100MS
  {
    cal_state.cal_step=0x05;//飞机机头朝上
    flight_calibrate.accel_step_makesure_cnt[4]=0;
    rgb_notify_set(RED	,TOGGLE	 ,500,2000,0);
    cal_step_beep();
  }		
  //第六面
  if(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f
     &&rc_data.rc_rpyt[RC_ROLL]==0
       &&rc_data.rc_rpyt[RC_PITCH]==0
	 &&rc_data.rc_rpyt[RC_YAW]>=Yaw_Max*Scale_Pecent_Max)
    flight_calibrate.accel_step_makesure_cnt[5]++;
  else flight_calibrate.accel_step_makesure_cnt[5]/=5;
  if(flight_calibrate.accel_step_makesure_cnt[5]>100/flight_calibrate.period_unit)//持续100MS
  {
    cal_state.cal_step=0x06;//飞机翻面放置 
    flight_calibrate.accel_step_makesure_cnt[5]=0;
    rgb_notify_set(RED	,TOGGLE	 ,500,2000,0);
    cal_step_beep();
  }
}



void mag_cal_show(void)
{
  LCD_clear_L(0,0);
  LCD_P6x8Str(10,0,(unsigned char *)"Mag_Correct");
  LCD_clear_L(0,1);
  write_6_8_number(0,1, (int16_t)(flymaple.mag_raw_data_tesla[0]));
  write_6_8_number(40,1,(int16_t)(flymaple.mag_raw_data_tesla[1]));
  write_6_8_number(70,1,(int16_t)(flymaple.mag_raw_data_tesla[2]));
  LCD_clear_L(0,2);
  LCD_P6x8Str(0,2,(unsigned char *)"0 To 360");
  write_6_8_number(70,2,cal_state.cal_step);
  LCD_clear_L(0,3);
  LCD_P6x8Str(0,3,(unsigned char *)"Mag Is Okay?");
  write_6_8_number(80,3,flight_calibrate.mag_cal_finished[0]);
  write_6_8_number(90,3,flight_calibrate.mag_cal_finished[1]);
  write_6_8_number(105,3,flight_calibrate.mag_cal_finished[2]);
  LCD_clear_L(0,4);
  for(uint16_t i=0;i<12;i++)
  {
    write_6_8_number(10*i,4,mag_sample_flag[0][3*i]);
  }
  LCD_clear_L(0,5);
  for(uint16_t i=0;i<12;i++)
  {
    write_6_8_number(10*i,5,mag_sample_flag[1][3*i]);
  }
  LCD_clear_L(0,6);
  for(uint16_t i=0;i<12;i++)
  {
    write_6_8_number(10*i,6,mag_sample_flag[2][3*i]);
  }
  LCD_clear_L(0,7);
  if(cal_state.cal_step==1) 		 LCD_P6x8Str(0,7,(unsigned char *)"Make Z+ Upside Sky");
  else if(cal_state.cal_step==2) LCD_P6x8Str(0,7,(unsigned char *)"Make Y+ Upside Sky");
  else if(cal_state.cal_step==3) LCD_P6x8Str(0,7,(unsigned char *)"Make X+ Upside Sky");
  else if(cal_state.cal_step==0) LCD_P6x8Str(0,7,(unsigned char *)"Start With Yaw Move"); 
}

void accel_cal_show(void)
{
  LCD_clear_L(0,0);
  LCD_P6x8Str(10,0,(unsigned char *)"Accel_Correct");
  write_6_8_number(100,0,cal_state.cal_step);
  LCD_clear_L(0,1);
  write_6_8_number(0,1,acce_sample[0].x);
  write_6_8_number(50,1,acce_sample[0].y);
  write_6_8_number(90,1,acce_sample[0].z);
  LCD_clear_L(0,2);
  write_6_8_number(0,2,acce_sample[1].x);
  write_6_8_number(50,2,acce_sample[1].y);
  write_6_8_number(90,2,acce_sample[1].z);
  LCD_clear_L(0,3);
  write_6_8_number(0,3,acce_sample[2].x);
  write_6_8_number(50,3,acce_sample[2].y);
  write_6_8_number(90,3,acce_sample[2].z);
  LCD_clear_L(0,4);
  write_6_8_number(0,4,acce_sample[3].x);
  write_6_8_number(50,4,acce_sample[3].y);
  write_6_8_number(90,4,acce_sample[3].z);
  LCD_clear_L(0,5);
  write_6_8_number(0,5,acce_sample[4].x);
  write_6_8_number(50,5,acce_sample[4].y);
  write_6_8_number(90,5,acce_sample[4].z);
  LCD_clear_L(0,6);
  write_6_8_number(0,6,acce_sample[5].x);
  write_6_8_number(50,6,acce_sample[5].y);
  write_6_8_number(90,6,acce_sample[5].z);
}

void rc_cal_show(void)
{
  LCD_clear_L(0,0);  LCD_P6x8Str(0,0,(unsigned char *)"ch1:");
  write_6_8_number(25,0,ppm_rc.data[0]);
  write_6_8_number(55,0,rc_data.cal[0].max_value);
  write_6_8_number(90,0,rc_data.cal[0].min_value);
  LCD_clear_L(0,1);  LCD_P6x8Str(0,1,(unsigned char *)"ch2:");
  write_6_8_number(25,1,ppm_rc.data[1]);
  write_6_8_number(55,1,rc_data.cal[1].max_value);
  write_6_8_number(90,1,rc_data.cal[1].min_value);
  LCD_clear_L(0,2);  LCD_P6x8Str(0,2,(unsigned char *)"ch3:");
  write_6_8_number(25,2,ppm_rc.data[2]);
  write_6_8_number(55,2,rc_data.cal[2].max_value);
  write_6_8_number(90,2,rc_data.cal[2].min_value);
  LCD_clear_L(0,3);  LCD_P6x8Str(0,3,(unsigned char *)"ch4:");
  write_6_8_number(25,3,ppm_rc.data[3]);
  write_6_8_number(55,3,rc_data.cal[3].max_value);
  write_6_8_number(90,3,rc_data.cal[3].min_value);
  LCD_clear_L(0,4);  LCD_P6x8Str(0,4,(unsigned char *)"ch5:");
  write_6_8_number(25,4,ppm_rc.data[4]);
  write_6_8_number(55,4,rc_data.cal[4].max_value);
  write_6_8_number(90,4,rc_data.cal[4].min_value);
  LCD_clear_L(0,5);  LCD_P6x8Str(0,5,(unsigned char *)"ch6:");
  write_6_8_number(25,5,ppm_rc.data[5]);
  write_6_8_number(55,5,rc_data.cal[5].max_value);
  write_6_8_number(90,5,rc_data.cal[5].min_value);
  LCD_clear_L(0,6);  LCD_P6x8Str(0,6,(unsigned char *)"ch7:");
  write_6_8_number(25,6,ppm_rc.data[6]);
  write_6_8_number(55,6,rc_data.cal[6].max_value);
  write_6_8_number(90,6,rc_data.cal[6].min_value);
  LCD_clear_L(0,7);  LCD_P6x8Str(0,7,(unsigned char *)"ch8:");
  write_6_8_number(25,7,ppm_rc.data[7]);
  write_6_8_number(55,7,rc_data.cal[7].max_value);
  write_6_8_number(90,7,rc_data.cal[7].min_value);
}

uint8_t cal_show_statemachine(void)
{
  if(flight_calibrate.mag_cal_flag==1)
  {
    mag_cal_show();
    return 1;
  }
  else if(flight_calibrate.accel_cal_flag==1)
  {
    accel_cal_show();
    return 1;
  }
  else if(flight_calibrate.rc_cal_flag==1)
  {
    rc_cal_show();
    return 1;
  }
  else
  {
    //LCD_P6x8Str(0,0,(unsigned char *)"no calibration:");
  }
  return 0;
}

