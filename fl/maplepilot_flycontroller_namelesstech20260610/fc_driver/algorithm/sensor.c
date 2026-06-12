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
#include "maple_config.h"
#include "schedule.h"
#include "quaternion.h"
#include "filter.h"
#include "drv_mpu6050.h"
#include "drv_spl06.h"
#include "drv_qmc5883.h"
#include "drv_adc.h"
#include "drv_notify.h"
#include "drv_gpio.h"
#include "rc.h"
#include "sins.h"
#include "maple_ahrs.h"
#include "drv_tofsense.h"
#include "drv_opticalflow.h"
#include "attitude_ctrl.h"
#include "nclink.h"
#include "reserved_serialport.h"
#include "parameter_server.h"
#include "Fusion.h"
#include "sensor.h"


FusionAhrs ahrs;
static FusionOffset offset;
static filter_buffer accel_feedback_filter_buffer[3];
filter_parameter ins_lpf_param,accel_lpf_param,gyro_lpf_param,accel_fb_lpf_param; 
static filter_buffer accel_filter_buffer[3],gyro_filter_buffer[3];
void sensor_filter_init(void)
{
  float tmp_player_level=0;
  ReadFlashParameterOne(DRONE_PLAYER_LEVEL ,&tmp_player_level);
  if(isnan(tmp_player_level)==0)  flymaple.player_level=(uint8_t)(tmp_player_level);
  else flymaple.player_level=player_level_default;
  
  float tmp_gyro_lpf_cf,tmp_accel_lpf_cf,tmp_accel_fb_lpf_param,tmp_ins_lpf_param_cf;
  if(flymaple.player_level==0)//通用机型
  {
    ReadFlashParameterOne(GYRO_LPF_CF ,&tmp_gyro_lpf_cf);
    if(isnan(tmp_gyro_lpf_cf)==0)  gyro_lpf_param.cf=tmp_gyro_lpf_cf;
    else gyro_lpf_param.cf=gyro_lpf_param_default1;
    
    ReadFlashParameterOne(ACCEL_LPF_CF,&tmp_accel_lpf_cf);
    if(isnan(tmp_accel_lpf_cf)==0) accel_lpf_param.cf=tmp_accel_lpf_cf;
    else accel_lpf_param.cf=accel_lpf_param_default1;
    
    ReadFlashParameterOne(INS_LPF_CF,&tmp_ins_lpf_param_cf);
    if(isnan(tmp_ins_lpf_param_cf)==0) ins_lpf_param.cf=tmp_ins_lpf_param_cf;
    else ins_lpf_param.cf=ins_lpf_param_default1;
    
    ReadFlashParameterOne(FB_LPF_CF,&tmp_accel_fb_lpf_param);
    if(isnan(tmp_accel_fb_lpf_param)==0) accel_fb_lpf_param.cf=tmp_accel_fb_lpf_param;
    else accel_fb_lpf_param.cf=accel_fb_lpf_param_default1;
  }
  else if(flymaple.player_level==1)
  {
    ReadFlashParameterOne(GYRO_LPF_CF ,&tmp_gyro_lpf_cf);
    if(isnan(tmp_gyro_lpf_cf)==0)  gyro_lpf_param.cf=tmp_gyro_lpf_cf;
    else gyro_lpf_param.cf=gyro_lpf_param_default2;
    
    ReadFlashParameterOne(ACCEL_LPF_CF,&tmp_accel_lpf_cf);
    if(isnan(tmp_accel_lpf_cf)==0) accel_lpf_param.cf=tmp_accel_lpf_cf;
    else accel_lpf_param.cf=accel_lpf_param_default2;
    
    ReadFlashParameterOne(INS_LPF_CF,&tmp_ins_lpf_param_cf);
    if(isnan(tmp_ins_lpf_param_cf)==0) ins_lpf_param.cf=tmp_ins_lpf_param_cf;
    else ins_lpf_param.cf=ins_lpf_param_default2;
    
    ReadFlashParameterOne(FB_LPF_CF,&tmp_accel_fb_lpf_param);
    if(isnan(tmp_accel_fb_lpf_param)==0) accel_fb_lpf_param.cf=tmp_accel_fb_lpf_param;
    else accel_fb_lpf_param.cf=accel_fb_lpf_param_default2;		
  }
  else
  {
    ReadFlashParameterOne(GYRO_LPF_CF ,&tmp_gyro_lpf_cf);
    if(isnan(tmp_gyro_lpf_cf)==0)  gyro_lpf_param.cf=tmp_gyro_lpf_cf;
    else gyro_lpf_param.cf=gyro_lpf_param_default1;
    
    ReadFlashParameterOne(ACCEL_LPF_CF,&tmp_accel_lpf_cf);
    if(isnan(tmp_accel_lpf_cf)==0) accel_lpf_param.cf=tmp_accel_lpf_cf;
    else accel_lpf_param.cf=accel_lpf_param_default1;
    
    ReadFlashParameterOne(INS_LPF_CF,&tmp_ins_lpf_param_cf);
    if(isnan(tmp_ins_lpf_param_cf)==0) ins_lpf_param.cf=tmp_ins_lpf_param_cf;
    else ins_lpf_param.cf=ins_lpf_param_default1;
    
    ReadFlashParameterOne(FB_LPF_CF,&tmp_accel_fb_lpf_param);
    if(isnan(tmp_accel_fb_lpf_param)==0) accel_fb_lpf_param.cf=tmp_accel_fb_lpf_param;
    else accel_fb_lpf_param.cf=accel_fb_lpf_param_default1;	
  }
  
  set_cutoff_frequency(sampling_frequent,gyro_lpf_param.cf ,&gyro_lpf_param);  //姿态角速度反馈滤波参数
  set_cutoff_frequency(sampling_frequent,accel_lpf_param.cf,&accel_lpf_param);//姿态解算加计修正滤波值
  set_cutoff_frequency(sampling_frequent,ins_lpf_param.cf,&ins_lpf_param);//INS加计滤波值 60
  set_cutoff_frequency(sampling_frequent,accel_fb_lpf_param.cf,&accel_fb_lpf_param);//加速度计反馈  5
}



low_voltage vbat={
  .enable=no_voltage_enable_default,
  .value=0,
  .upper=no_voltage_upper_default,
  .lower=no_voltage_lower_default
};

void battery_voltage_detection(void)
{
  //解锁状态下，低压报警关闭
  if(rc_data.lock_state==UNLOCK) return;//原因是蜂鸣器哔哔会影响IMU数据，飞行过程过程中响可能会失控
  
  static uint16_t _cnt=0;
  _cnt++;
  if(_cnt>=200)//每1S检测一次
  {
    _cnt=0;
    if(vbat.value<vbat.upper&&vbat.value>vbat.lower)	 vbat.low_vbat_cnt++;
    else vbat.low_vbat_cnt/=2;
    if(vbat.low_vbat_cnt>=10)//持续10s满足
    {
      if(vbat.enable==1)
      {
        beep.period=200;//200*5ms
        beep.light_on_percent=0.25f;
        beep.reset=1;	
        beep.times=5;//闪烁5次
      }
      vbat.low_vbat_cnt=0;//清0待下一周期检测			
    }			
  }
}



void compass_fault_check(void)
{ 
  static float last_mag_raw[3]={0,0,0};
  static uint16_t compass_fault_cnt=0;	
  static uint16_t compass_gap_cnt=0;
  compass_gap_cnt++;
  if(compass_gap_cnt>=40)//每200ms检测一次，因为磁力计更新周期大于5ms
  {
    compass_gap_cnt=0;
    if(last_mag_raw[0]==flymaple.mag_raw_data_tesla[0]
       &&last_mag_raw[1]==flymaple.mag_raw_data_tesla[1]
         &&last_mag_raw[2]==flymaple.mag_raw_data_tesla[2])
    {
      compass_fault_cnt++;
      if(compass_fault_cnt>10)  flymaple.compass_health_flag=false;//磁力计数据不健康   
    }
    else
    {
      compass_fault_cnt/=2;
      if(compass_fault_cnt==0)  flymaple.compass_health_flag=true; 
    }
    last_mag_raw[0]=flymaple.mag_raw_data_tesla[0];
    last_mag_raw[1]=flymaple.mag_raw_data_tesla[1];
    last_mag_raw[2]=flymaple.mag_raw_data_tesla[2];
  }
}


void sensor_raw_update(void)
{
  flymaple.vbat =vbat.value=get_battery_valtage();
  get_imu_data(flymaple.accel_raw_data,flymaple.gyro_raw_data,&flymaple._t,&flymaple.imu_acc_scale,&flymaple.imu_gyro_scale,&flymaple.imu_data_offline_flag);
  
  
  spl06_read_data(&flymaple.baro_t,&flymaple.baro_p,&flymaple.baro_update_flag);
  get_mag_data(flymaple.mag_raw_data_tesla,&flymaple.mag_update_flag);
  compass_fault_check();
  //
  switch(flymaple.imu_gyro_scale)
  {
  case SCALE_250: flymaple.gyro_scale_raw_to_dps=GYRO_SCALE_250_DPS; break;
  case SCALE_500: flymaple.gyro_scale_raw_to_dps=GYRO_SCALE_500_DPS; break;
  case SCALE_1000:flymaple.gyro_scale_raw_to_dps=GYRO_SCALE_1000_DPS;break;
  case SCALE_2000:flymaple.gyro_scale_raw_to_dps=GYRO_SCALE_2000_DPS;break;
  }
  switch(flymaple.imu_acc_scale)
  {
  case SCALE_2G: flymaple.accel_scale_raw_to_g=16384.0f;  break;
  case SCALE_4G: flymaple.accel_scale_raw_to_g=8192.0f;   break;
  case SCALE_8G: flymaple.accel_scale_raw_to_g=4096.0f;   break;
  case SCALE_16G:flymaple.accel_scale_raw_to_g=2048.0f;   break;
  case SCALE_3G: flymaple.accel_scale_raw_to_g=10920.0f;   break;
  case SCALE_6G: flymaple.accel_scale_raw_to_g=5460.0f;   break;
  case SCALE_12G: flymaple.accel_scale_raw_to_g=2730.0f;   break;
  case SCALE_24G:flymaple.accel_scale_raw_to_g=1365.0; break;
  }	
  
  //
  vector3f src_accel_rawdata,dst_accel_rawdata;
  src_accel_rawdata.x=flymaple.accel_raw_data[0];
  src_accel_rawdata.y=flymaple.accel_raw_data[1];
  src_accel_rawdata.z=flymaple.accel_raw_data[2];
  
  vector3f_mul_sub_to_rawdata(src_accel_rawdata,flymaple.accel_scale,flymaple.accel_offset,&flymaple.accel_raw_correct,flymaple.accel_scale_raw_to_g);//校准后的加速度计原始数字量
  
  //原始数据滤波处理	
  static float last_t;
  last_t=flymaple.t;
  flymaple.t=last_t+0.1f*(flymaple._t-last_t);
  
  //姿态解算、惯导相关加速度计、陀螺仪滤波
  flymaple.accel_raw_filter_data.x = butterworth(flymaple.accel_raw_correct.x,&accel_filter_buffer[0],&accel_lpf_param);
  flymaple.accel_raw_filter_data.y = butterworth(flymaple.accel_raw_correct.y,&accel_filter_buffer[1],&accel_lpf_param);
  flymaple.accel_raw_filter_data.z = butterworth(flymaple.accel_raw_correct.z,&accel_filter_buffer[2],&accel_lpf_param);
  flymaple.gyro_raw_filter_data.x  = butterworth(flymaple.gyro_raw_data[0] ,&gyro_filter_buffer[0] ,&gyro_lpf_param);
  flymaple.gyro_raw_filter_data.y  = butterworth(flymaple.gyro_raw_data[1] ,&gyro_filter_buffer[1] ,&gyro_lpf_param);
  flymaple.gyro_raw_filter_data.z  = butterworth(flymaple.gyro_raw_data[2] ,&gyro_filter_buffer[2] ,&gyro_lpf_param);	
  
  //原始数字量转化为g
  flymaple.accel_g.x=(flymaple.accel_raw_filter_data.x/flymaple.accel_scale_raw_to_g);
  flymaple.accel_g.y=(flymaple.accel_raw_filter_data.y/flymaple.accel_scale_raw_to_g);
  flymaple.accel_g.z=(flymaple.accel_raw_filter_data.z/flymaple.accel_scale_raw_to_g);	
  
  //修正陀螺仪零偏
  vector3f_sub(flymaple.gyro_raw_filter_data,flymaple.gyro_offset,&flymaple.gyro_correct_raw_data,0);
  
  flymaple.gyro_dps.x=flymaple.gyro_correct_raw_data.x*flymaple.gyro_scale_raw_to_dps;
  flymaple.gyro_dps.y=flymaple.gyro_correct_raw_data.y*flymaple.gyro_scale_raw_to_dps;
  flymaple.gyro_dps.z=flymaple.gyro_correct_raw_data.z*flymaple.gyro_scale_raw_to_dps;
  
  flymaple.gyro_rps.x=flymaple.gyro_dps.x*DEG2RAD;//角速度转换成弧度制
  flymaple.gyro_rps.y=flymaple.gyro_dps.y*DEG2RAD;
  flymaple.gyro_rps.z=flymaple.gyro_dps.z*DEG2RAD;
  
  
  flymaple.accel_fb_filter_data.x=butterworth(flymaple.accel_raw_data[0],&accel_feedback_filter_buffer[0],&accel_fb_lpf_param);//控制器中加速度计反馈滤波
  flymaple.accel_fb_filter_data.y=butterworth(flymaple.accel_raw_data[1],&accel_feedback_filter_buffer[1],&accel_fb_lpf_param);
  flymaple.accel_fb_filter_data.z=butterworth(flymaple.accel_raw_data[2],&accel_feedback_filter_buffer[2],&accel_fb_lpf_param);
  
  //
  float offset_scale=flymaple.accel_scale_raw_to_g/GRAVITY_MSS;
  dst_accel_rawdata.x=flymaple.accel_scale.x*flymaple.accel_fb_filter_data.x-flymaple.accel_offset.x*offset_scale;
  dst_accel_rawdata.y=flymaple.accel_scale.y*flymaple.accel_fb_filter_data.y-flymaple.accel_offset.y*offset_scale;
  dst_accel_rawdata.z=flymaple.accel_scale.z*flymaple.accel_fb_filter_data.z-flymaple.accel_offset.z*offset_scale;
  
  
  //原始数字量转化为g
  flymaple.accel_g_fb.x=(dst_accel_rawdata.x/flymaple.accel_scale_raw_to_g);
  flymaple.accel_g_fb.y=(dst_accel_rawdata.y/flymaple.accel_scale_raw_to_g);
  flymaple.accel_g_fb.z=(dst_accel_rawdata.z/flymaple.accel_scale_raw_to_g);		
  
  
  //角速度备份
  for(uint16_t i=19;i>0;i--)
  {
    flymaple.gyro_backups[i].x=flymaple.gyro_backups[i-1].x;
    flymaple.gyro_backups[i].y=flymaple.gyro_backups[i-1].y;
    flymaple.gyro_backups[i].z=flymaple.gyro_backups[i-1].z;
  }
  flymaple.gyro_backups[0].x=flymaple.gyro_dps.x;
  flymaple.gyro_backups[0].y=flymaple.gyro_dps.y;
  flymaple.gyro_backups[0].z=flymaple.gyro_dps.z;
  
  //{-sinθ cosθsinΦ cosθcosΦ}
  flymaple.yaw_gyro_enu=-flymaple.sin_rpy[_ROL]*flymaple.gyro_dps.x+flymaple.cos_rpy[_ROL]*flymaple.sin_rpy[_PIT]*flymaple.gyro_dps.y+flymaple.cos_rpy[_PIT]*flymaple.cos_rpy[_ROL]*flymaple.gyro_dps.z;
  flymaple.total_gyro_dps=pythagorous3(flymaple.gyro_dps.x,flymaple.gyro_dps.y,flymaple.gyro_dps.z);
   
  flymaple.rpy_integral_deg[0]+=flymaple.gyro_dps.y*min_ctrl_dt;
  flymaple.rpy_integral_deg[1]+=flymaple.gyro_dps.x*min_ctrl_dt;
  flymaple.rpy_integral_deg[2]+=flymaple.gyro_dps.z*min_ctrl_dt;
  for(uint16_t i=0;i<3;i++)
  {
    if(flymaple.rpy_integral_deg[i]<0.0f)   flymaple.rpy_integral_deg[i]+=360.0f;
    if(flymaple.rpy_integral_deg[i]>360.0f) flymaple.rpy_integral_deg[i]-=360.0f;
  }
  
  float ax,ay,az;
  ax=flymaple.accel_raw_filter_data.x;
  ay=flymaple.accel_raw_filter_data.y;
  az=flymaple.accel_raw_filter_data.z;
  
  flymaple.rpy_obs_deg[0]=-57.3f*atan(ax*invSqrt(ay*ay+az*az)); //观测横滚角
  flymaple.rpy_obs_deg[1]= 57.3f*atan(ay*invSqrt(ax*ax+az*az)); //观测俯仰角
  
  flymaple.sin_rpy[_PIT]=sinf(flymaple.rpy_fusion_deg[1]*DEG2RAD);
  flymaple.cos_rpy[_PIT]=cosf(flymaple.rpy_fusion_deg[1]*DEG2RAD);
  flymaple.sin_rpy[_ROL]=sinf(flymaple.rpy_fusion_deg[0]*DEG2RAD);
  flymaple.cos_rpy[_ROL]=cosf(flymaple.rpy_fusion_deg[0]*DEG2RAD);
  flymaple.sin_rpy[_YAW]=sinf(flymaple.rpy_fusion_deg[2]*DEG2RAD);
  flymaple.cos_rpy[_YAW]=cosf(flymaple.rpy_fusion_deg[2]*DEG2RAD);
  
  flymaple.mag.x=flymaple.mag_raw_data_tesla[0]-flymaple.mag_offset.x;
  flymaple.mag.y=flymaple.mag_raw_data_tesla[1]-flymaple.mag_offset.y;
  flymaple.mag.z=flymaple.mag_raw_data_tesla[2]-flymaple.mag_offset.z;
  
  flymaple.mag_intensity=pythagorous3(flymaple.mag.x,flymaple.mag.y,flymaple.mag.z);
  
  float mx,my,mz;
  mx=flymaple.mag.x;	my=flymaple.mag.y;	mz=flymaple.mag.z;
  float norm = invSqrt(mx * mx + my * my + az * az);
  mx *= norm;	
  my *= norm;	
  mz *= norm;
  /************磁力计倾角补偿*****************/
  vector2f magn;	
  magn.x=  mx * flymaple.cos_rpy[_ROL]+my * flymaple.sin_rpy[_ROL] * flymaple.sin_rpy[_PIT]+mz * flymaple.sin_rpy[_ROL] * flymaple.cos_rpy[_PIT];
  magn.y=  my * flymaple.cos_rpy[_PIT]- mz * flymaple.sin_rpy[_PIT];
  
  /***********反正切得到磁力计观测角度*********/
  flymaple.rpy_obs_deg[2]=atan2f(magn.x,magn.y)*57.296f;
  if(flymaple.rpy_obs_deg[2]<0) flymaple.rpy_obs_deg[2]=flymaple.rpy_obs_deg[2]+360;
  
  flymaple.rpy_obs_deg[2]-=get_earth_declination();//home点已设置，获取当地磁偏角，得到地理真北
  flymaple.rpy_obs_deg[2]=constrain_float(flymaple.rpy_obs_deg[2],0.0f,360);
  //计算观测四元数
  euler_to_quaternion(flymaple.rpy_obs_deg,flymaple.quaternion_obs);
  /****************************************************/
  FusionVector gyroscope={0.0f, 0.0f, 0.0f};
  FusionVector accelerometer = {0.0f, 0.0f, 1.0f};
  FusionVector earthacceleration = {0.0f, 0.0f, 0.0f};  
  gyroscope.axis.x=flymaple.gyro_dps.x;
  gyroscope.axis.y=flymaple.gyro_dps.y;
  gyroscope.axis.z=flymaple.gyro_dps.z;
  
  accelerometer.axis.x=flymaple.accel_g.x;
  accelerometer.axis.y=flymaple.accel_g.y;
  accelerometer.axis.z=flymaple.accel_g.z;	
  
  if(flymaple.quaternion_init_ok==1)
  {
    gyroscope = FusionOffsetUpdate(&offset, gyroscope);
    if(current_state.rec_update_flag==1&&maplepilot.indoor_position_sensor==RPISLAM)//偏航角观测量来源于slam
    {
      if(current_state.rec_head_update_flag==1)
      {
        current_state.rec_head_update_flag=0;			
        quad_getangle(current_state.q,current_state.rpy);
        if(current_state.rpy[2]<0.0f)   current_state.rpy[2]+=360.0f;
        if(current_state.rpy[2]>360.0f) current_state.rpy[2]-=360.0f;	
      }
      flymaple.yaw_obs_deg=current_state.rpy[2];
      FusionAhrsUpdateExternalHeading(&ahrs, gyroscope, accelerometer,flymaple.yaw_obs_deg,0.005f);//陀螺仪+加速度计+slam航向观测姿态融合
    } 
    else if(flymaple.compass_health_flag==1)//偏航角观测量来源于磁力计
    {
      flymaple.yaw_obs_deg=flymaple.rpy_obs_deg[2];
      FusionAhrsUpdateExternalHeading(&ahrs, gyroscope, accelerometer,flymaple.yaw_obs_deg,0.005f);//陀螺仪+加速度计+磁力计航向观测姿态融合
    }
    else//只进行加速度计、陀螺仪融合 
    {
      FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer,0.005f);//陀螺仪+加速度计姿态融合
    }
    
    FusionEuler euler=FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    earthacceleration=FusionAhrsGetEarthAcceleration(&ahrs);
    flymaple.rpy_contrast_deg[0]=euler.angle.pitch;
    flymaple.rpy_contrast_deg[1]=euler.angle.roll;
    flymaple.rpy_contrast_deg[2]=euler.angle.yaw;
    //获取导航系系统运动加速度
    flymaple.accel_earth_g.x=earthacceleration.axis.x;
    flymaple.accel_earth_g.y=earthacceleration.axis.y;
    flymaple.accel_earth_g.z=earthacceleration.axis.z;
  }
  
  
  //气压计海拔高度更新
  if(flymaple.baro_update_flag==0) return;
  static systime _t;
  get_systime(&_t);
  flymaple.baro_period_ms=_t.period_int;
  static uint32_t baro_sample_cnt=0;
  if(baro_sample_cnt==0)
  {
    //记录海拔零点
    flymaple.baro_pressure_offset=flymaple.baro_p;
    flymaple.baro_temp_offset=flymaple.baro_t;
    flymaple.baro_cal_finished_flag=1;
    baro_sample_cnt++;
  }
  else if(baro_sample_cnt==1)
  {
    static float last_baro_alt=0,last_baro_vel=0;
    last_baro_alt=flymaple.baro_height;
    last_baro_vel=flymaple.baro_vel;
    float Tempbaro=(float)(flymaple.baro_p/flymaple.baro_pressure_offset)*1.0f;
    flymaple.baro_height = 4433000.0f * (1 - powf((float)(Tempbaro),0.190295f));
    flymaple.baro_vel=(flymaple.baro_height-last_baro_alt)/(0.001f*flymaple.baro_period_ms);
    flymaple.baro_acc=(flymaple.baro_vel-last_baro_vel)/(0.001f*flymaple.baro_period_ms);
    for(int16_t i=20-1;i>0;i--)
    {
      flymaple.baro_height_backups[i]=flymaple.baro_height_backups[i-1];
    }
    flymaple.baro_height_backups[0]=flymaple.baro_height;
  }
}

void sensor_fusion_update(void)
{
  flymaple.rpy_fusion_deg[0] =	flymaple.rpy_contrast_deg[0];
  flymaple.rpy_fusion_deg[1] =	flymaple.rpy_contrast_deg[1];
  flymaple.rpy_fusion_deg[2] =  flymaple.rpy_contrast_deg[2];
  
  flymaple.quaternion[0]=ahrs.quaternion.element.w;
  flymaple.quaternion[1]=ahrs.quaternion.element.x;
  flymaple.quaternion[2]=ahrs.quaternion.element.y;
  flymaple.quaternion[3]=ahrs.quaternion.element.z;
  //euler_to_quaternion(flymaple.rpy_fusion_deg,flymaple.quaternion); //计算含磁力计修正后的四元数
  quaternion_to_cb2n(flymaple.quaternion,flymaple.cb2n);//通过四元数求取旋转矩阵
  
  
  if(flymaple.quaternion_init_ok==0)
  {
    if(flymaple.temperature_stable_flag==1)//温度稳定
    {
      calculate_quaternion_init(flymaple.accel_raw_filter_data,flymaple.mag,flymaple.quaternion_init);
      madgwick_set_quaternion(flymaple.quaternion_init);
      flymaple.quaternion_init_ok	=	1;
      rc_data.fc_ready_flag	=	1;//遥控器允许解锁标志位
      rgb_notify_set(BLUE,TOGGLE,1000,5000,500);
      //AHRS初始化
      FusionOffsetInitialise(&offset, sampling_frequent);
      FusionAhrsInitialise(&ahrs);
      // Set AHRS algorithm settings
      const FusionAhrsSettings settings = {
        .gain = 0.5f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 20.0f,
        .rejectionTimeout = 5 * sampling_frequent, /* 5 seconds */
      };
      FusionAhrsSetSettings(&ahrs, &settings);
    }		
  }
}


void height_sensor_select(void)
{
  flymaple.rangefinder_update_flag=tofsense_data.update_flag;
  if(tofsense_data.valid==1)//使用对地测距传感器
  {
    ins.alt_obs_update=flymaple.rangefinder_update_flag;	
    ins.alt_obs_cm=tofsense_data.distance*(flymaple.cos_rpy[_PIT]*flymaple.cos_rpy[_ROL]);
    ins.alt_obs_update_period=tofsense_data.period_ms;
    flymaple.rangefinder_update_flag=0;
    switch(maplepilot.rangefinder_sensor)
    {
    case TOFSENSE:
      ins.alt_obs_type=1;
      break;
    case VL53L8:
      ins.alt_obs_type=2;
      break;
    case UP_Tx:
      ins.alt_obs_type=3;
      break;
    case MT1:
      ins.alt_obs_type=4;
      break;			
    case GYTOF10M:
      ins.alt_obs_type=5;
      break;
    case TFMINI:
      ins.alt_obs_type=6;
      break;
    default:ins.alt_obs_type=0;
    }
  }
  else//使用气压计传感器
  {
    ins.alt_obs_update=flymaple.baro_update_flag;
    ins.alt_obs_cm=flymaple.baro_height;
    ins.alt_obs_update_period=flymaple.baro_period_ms;
    ins.alt_obs_type=0;
    flymaple.baro_update_flag=0;
  }
}


void flymaple_nav_update(void)
{
  inertial_acceleration_get();//计算惯性导航用的加速度
  if(flymaple.quaternion_init_ok==0)  return;
  //高度观测传感器选择
  height_sensor_select();
  //高度卡尔曼滤波惯导融合
  altitude_kalman_filter(&alt_kf,&ins,ins.acceleration_initial[_UP],ins.alt_obs_cm,flymaple.baro_vel,min_ctrl_dt);
  //高度方向上气压计和惯导三阶互补数据融合
  strapdown_ins_height();
  if(tofsense_data.valid==1)//使用对地测距传感器融合得到得竖直位置、竖直速度、竖直加速度估计值作为反馈量
  {	
    ins.position_z     =ins.position[_UP];
    ins.speed_z        =ins.speed[_UP];
    ins.acceleration_z =ins.acceleration[_UP];	
  }
  else//使用气压计传感器融合得到得竖直位置、竖直速度、竖直加速度估计值作为反馈量
  {	
    ins.position_z     =pos_z_fusion;
    ins.speed_z	       =vel_z_fusion;
    ins.acceleration_z =acc_z_fusion;	
  }
  //GPS水平位置估计
  kalmanfilter_horizontal();
  //光流、SLAM水平位置估计
  indoor_position_fusion();
}


void inertial_acceleration_get(void)
{
  vector3f body_frame,earth_frame;
  //惯导相关加速度计算,转换成cm/s^2	
  ins.acceleration_initial[EAST] =flymaple.accel_earth_g.x*GRAVITY_MSS*100.0f;
  ins.acceleration_initial[NORTH]=flymaple.accel_earth_g.y*GRAVITY_MSS*100.0f;
  ins.acceleration_initial[_UP]  =flymaple.accel_earth_g.z*GRAVITY_MSS*100.0f;		
  
  //控制器反馈相关加速度计算	
  body_frame=flymaple.accel_g_fb;//单位为g
  vector_from_bodyframe2earthframe(&body_frame,&earth_frame,flymaple.cb2n);
  
  ins.accel_feedback[_UP]  =earth_frame.z;
  ins.accel_feedback[EAST] =earth_frame.x;
  ins.accel_feedback[NORTH]=earth_frame.y;
  
  ins.accel_feedback[_UP]  *=GRAVITY_MSS;
  ins.accel_feedback[_UP]  -=GRAVITY_MSS;
  ins.accel_feedback[EAST] *=GRAVITY_MSS;
  ins.accel_feedback[NORTH]*=GRAVITY_MSS;
  
  ins.accel_feedback[_UP]  *=100.0f;
  ins.accel_feedback[EAST] *=100.0f;
  ins.accel_feedback[NORTH]*=100.0f;
  
  //将无人机在导航坐标系下的沿着正东、正北方向的运动加速度旋转到当前航向的运动加速度:机头(俯仰)+横滚
  vector2f _accel;
  _accel.x=ins.acceleration_initial[EAST];  //沿地理坐标系，正东方向运动加速度,单位为CM
  _accel.y=ins.acceleration_initial[NORTH]; //沿地理坐标系，正北方向运动加速度,单位为CM
  
  ins.horizontal_acceleration.x= _accel.x*flymaple.cos_rpy[_YAW]+_accel.y*flymaple.sin_rpy[_YAW];  //横滚正向运动加速度  X轴正向
  ins.horizontal_acceleration.y=-_accel.x*flymaple.sin_rpy[_YAW]+_accel.y*flymaple.cos_rpy[_YAW];  //机头正向运动加速度  Y轴正向
}

void calculate_quaternion_init(vector3f acc,vector3f mag,float *q)
{
  float ax,ay,az,mx,my,mz;
  float tmp_rpy[3],_sin_rpy[2],_cos_rpy[2];
  
  ax= acc.x;	ay= acc.y;	az= acc.z;
  mx= mag.x;	my= mag.y;	mz= mag.z;	
  float norm = invSqrt(mx * mx + my * my + az * az);
  mx *=	norm;
  my *=	norm;
  mz *=	norm;
  
  tmp_rpy[0]=-57.3f*atan(ax*invSqrt(ay*ay+az*az)); //横滚角
  tmp_rpy[1]= 57.3f*atan(ay*invSqrt(ax*ax+az*az)); //俯仰角
  _sin_rpy[_PIT]=sinf(tmp_rpy[1]*DEG2RAD);
  _cos_rpy[_PIT]=cosf(tmp_rpy[1]*DEG2RAD);
  _sin_rpy[_ROL]=sinf(tmp_rpy[0]*DEG2RAD);
  _cos_rpy[_ROL]=cosf(tmp_rpy[0]*DEG2RAD);
  
  /************磁力计倾角补偿*****************/
  vector2f magn;	
  magn.x=  mx * _cos_rpy[_ROL]+my * _sin_rpy[_ROL] * _cos_rpy[_PIT] +mz * _sin_rpy[_ROL] * _cos_rpy[_PIT];
  magn.y=  my * _cos_rpy[_PIT]-mz * _sin_rpy[_PIT];
  
  /***********反正切得到磁力计观测角度*********/
  tmp_rpy[2] = atan2f(magn.x,magn.y)*57.296f;
  if(tmp_rpy[2]<0) tmp_rpy[2] = tmp_rpy[2]+360;
  tmp_rpy[2] = constrain_float(tmp_rpy[2],0.0f,360);
  //
  euler_to_quaternion(tmp_rpy,q);//计算观测四元数
}


