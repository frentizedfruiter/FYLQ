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
#include "rc.h"
#include "calibration.h"
#include "drv_notify.h"
#include "drv_oled.h"
#include "drv_tofsense.h"
#include "drv_opticalflow.h"
#include "drv_gps.h"
#include "pid.h"
#include "drv_w25qxx.h"
#include "parameter_server.h"
#include "calibration.h"
#include "drv_opticalflow.h"
#include "attitude_ctrl.h"
#include "nclink.h"
#include "reserved_serialport.h"
#include "flymaple_sdk.h"
#include "maple_config.h"
#include "ssd1306.h"
#include "drv_button.h"
#include "sensor.h"
#include "drv_expand.h"
#include "drv_mpu6050.h"
#include "nclink.h"
#include "oled_display.h"



extern uint16_t pid_param_flag;


void switch_choose_beep(void)
{
  beep.period=20;//20*5ms
  beep.light_on_percent=0.5f;
  beep.reset=1;	
  beep.times=1;
}



int16_t page_number=0;
int8_t ver_choose=1;
extern uint32_t log_bytes_len;


#define page_number_Max 23-1//20
void display_page_change(void)
{
#define page_last  (rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f \
  &&rc_data.rc_rpyt[RC_ROLL]<-Pit_Rol_Max*Scale_Pecent_Max \
    &&rc_data.rc_rpyt[RC_PITCH]==0  \
      &&rc_data.rc_rpyt[RC_YAW]>=Yaw_Max*Scale_Pecent_Max)

#define page_next  (rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f \
&&rc_data.rc_rpyt[RC_ROLL]>Pit_Rol_Max*Scale_Pecent_Max \
  &&rc_data.rc_rpyt[RC_PITCH]==0 \
    &&rc_data.rc_rpyt[RC_YAW]>=Yaw_Max*Scale_Pecent_Max)
if(rc_data.lock_state!=LOCK) return;	
if(page_last)
{
  delay_ms(10);
  if(page_last)
  {
    page_number--;
    
    if(page_number<0) page_number=page_number_Max;
    OLED_CLS();
    while(page_last);
    switch_choose_beep();		
  }
}
//
if(page_next)
{
  delay_ms(10);
  if(page_next)
  {
    page_number++;
    if(page_number>page_number_Max) page_number=0;
    OLED_CLS();
    while(page_next);		
    switch_choose_beep();	
  }
}	

#undef page_last 
#undef page_next

if(_button.state[IUP].press==SHORT_PRESS)
{
  page_number--;
  if(page_number<0) page_number=page_number_Max;
  _button.state[IUP].press=NO_PRESS;
  OLED_CLS();
}

if(_button.state[IDOWN].press==SHORT_PRESS)
{
  page_number++;
  if(page_number>page_number_Max) page_number=0;
  _button.state[IDOWN].press=NO_PRESS;
  OLED_CLS();
}	


}


extern uint8_t device_id ;
void oled_show_page(void)
{
  if(cal_show_statemachine())  return	;
  display_page_change();
  switch(page_number)
  {
  case 0:
    {
      LCD_clear_L(0,0);  
      LCD_P6x8Str(0,0,"B");          
      write_6_8_number(15,0,flymaple.vbat);
       write_6_8_number(70,0,device_id);
      LCD_clear_L(0,1);  LCD_P6x8Str(0,1,"Y:");       	display_6_8_number_pro(15,1,flymaple.rpy_fusion_deg[YAW]);   display_6_8_number_pro(65,1,flymaple.gyro_dps.z);  
      LCD_clear_L(0,2);  LCD_P6x8Str(0,2,"P:");       	display_6_8_number_pro(15,2,flymaple.rpy_fusion_deg[PITCH]); display_6_8_number_pro(65,2,flymaple.gyro_dps.x);  
      LCD_clear_L(0,3);  LCD_P6x8Str(0,3,"R:");       	display_6_8_number_pro(15,3,flymaple.rpy_fusion_deg[ROLL]);  display_6_8_number_pro(65,3,flymaple.gyro_dps.y);  
      switch(imu_id)
      {
      case WHO_AM_I_MPU6050:display_6_8_string(102,2,"mpu");   break;
      case WHO_AM_I_ICM20689:display_6_8_string(102,2,"689");  break;
      case WHO_AM_I_ICM20608D:display_6_8_string(102,2,"08D"); break;
      case WHO_AM_I_ICM20608G:display_6_8_string(102,2,"08G"); break;
      case WHO_AM_I_ICM20602:display_6_8_string(102,2,"602");  break;
      case WHO_AM_I_BMI088:display_6_8_string(102,2,"BMI");    break;
      case WHO_AM_I_ICM42688:display_6_8_string(102,2,"I426"); break;
      default:display_6_8_string(102,2,"unk");		
      }     
      
      
      LCD_clear_L(0,4);  LCD_P6x8Str(0,4,"Baro:");      write_6_8_number(40,4,flymaple.baro_p);          			 write_6_8_number(90,4,flymaple.baro_height);
      LCD_clear_L(0,5);  LCD_P6x8Str(0,5,"GD_H:");      write_6_8_number(40,5,tofsense_data.distance);         
      write_6_8_number(80,5,ins.position_z);
      LCD_clear_L(0,6);  LCD_P6x8Str(0,6,"S_N:");       write_6_8_number(26,6,gps_data.gps.numsv);
      write_6_8_number(45,6,gps_data.gps.pdop);	
      display_6_8_number_pro(90,6,ins.speed_z);
      LCD_clear_L(0,7);  LCD_P6x8Str(0,7,"T:");
      write_6_8_number(15,7,flymaple.temperature_stable_flag); 			
      write_6_8_number(30,7,flymaple.imu_gyro_calibration_flag); 
      write_6_8_number(45,7,flymaple.t);   
      display_6_8_number_pro(90,7,ins.acceleration_initial[_UP]);
    }break;
  case 1:
    {
      LCD_clear_L(0,0);  LCD_P6x8Str(0,0,"Accel:");
      if(flymaple.accel_calibration_way==0)  LCD_P6x8Str(50,0,"Simple");
      else LCD_P6x8Str(50,0,"6-Side");
      switch(imu_id)
      {
      case WHO_AM_I_MPU6050:display_6_8_string(90,0,"mpu"); break;
      case WHO_AM_I_ICM20689:display_6_8_string(90,0,"689"); break;
      case WHO_AM_I_ICM20608D:display_6_8_string(90,0,"08D"); break;
      case WHO_AM_I_ICM20608G:display_6_8_string(90,0,"08G"); break;
      case WHO_AM_I_ICM20602:display_6_8_string(90,0,"602"); break;
      case WHO_AM_I_BMI088:display_6_8_string(90,0,"BMI");    break;
      case WHO_AM_I_ICM42688:display_6_8_string(90,0,"I426"); break;
      default:display_6_8_string(90,0,"unk");		
      }
      
      LCD_clear_L(0,1);  LCD_P6x8Str(0,1,"X:");              write_6_8_number(12,1,flymaple.accel_scale.x);   write_6_8_number(55,1,flymaple.accel_offset.x);   write_6_8_number(95,1,flymaple.accel_hor_offset.x);
      LCD_clear_L(0,2);  LCD_P6x8Str(0,2,"Y:");              write_6_8_number(12,2,flymaple.accel_scale.y);   write_6_8_number(55,2,flymaple.accel_offset.y);   write_6_8_number(95,2,flymaple.accel_hor_offset.y);
      LCD_clear_L(0,3);  LCD_P6x8Str(0,3,"Z:");              write_6_8_number(12,3,flymaple.accel_scale.z);   write_6_8_number(55,3,flymaple.accel_offset.z);   write_6_8_number(95,3,flymaple.accel_hor_offset.z);
      LCD_clear_L(0,4);  LCD_P6x8Str(0,4,"Mag/T:");          write_6_8_number(40,4,flymaple.mag_intensity);
      LCD_P6x8Str(75,4,"GY/PR_Off:");
      LCD_clear_L(0,5);  LCD_P6x8Str(0,5,"X:");         		 write_6_8_number(12,5,flymaple.mag_offset.x); write_6_8_number(55,5,flymaple.gyro_offset.x);  write_6_8_number(105,5,flymaple.rpy_angle_offset[_PIT]);
      LCD_clear_L(0,6);  LCD_P6x8Str(0,6,"Y:");         		 write_6_8_number(12,6,flymaple.mag_offset.y); write_6_8_number(55,6,flymaple.gyro_offset.y);  write_6_8_number(105,6,flymaple.rpy_angle_offset[_ROL]); 
      LCD_clear_L(0,7);  LCD_P6x8Str(0,7,"Z:");         		 write_6_8_number(12,7,flymaple.mag_offset.z); write_6_8_number(55,7,flymaple.gyro_offset.z); 
      
      if(_button.state[IUP].press==LONG_PRESS)
      {
        _button.state[IUP].press=NO_PRESS;
        if(flymaple.accel_calibration_way==0) flymaple.accel_calibration_way=1;
        else if(flymaple.accel_calibration_way==1) flymaple.accel_calibration_way=0;
        WriteFlashParameter(ACCEL_SIMPLE_MODE,flymaple.accel_calibration_way,&Flight_Params);		
      }	     
    }break;
  case 2:
    {
      LCD_clear_L(0,0);  display_6_8_string(0,0,"RC Data");         display_6_8_number(115,0,page_number+1);
      LCD_clear_L(0,1);  display_6_8_string(0,1,"Yaw:");            display_6_8_number(40,1,rc_data.rc_rpyt[RC_YAW]);            display_6_8_number(90,1,rc_data.rcdata[3]);
      LCD_clear_L(0,2);  display_6_8_string(0,2,"Pitch:");          display_6_8_number(40,2,rc_data.rc_rpyt[RC_PITCH]);          display_6_8_number(90,2,rc_data.rcdata[1]);
      LCD_clear_L(0,3);  display_6_8_string(0,3,"Roll:");           display_6_8_number(40,3,rc_data.rc_rpyt[RC_ROLL]);           display_6_8_number(90,3,rc_data.rcdata[0]);
      LCD_clear_L(0,4);  display_6_8_string(0,4,"Thr:");            display_6_8_number(40,4,rc_data.thr);                				 display_6_8_number(90,4,rc_data.rcdata[2]);
      LCD_clear_L(0,5);  display_6_8_string(0,5,"ch5-6:");          display_6_8_number(40,5,rc_data.rcdata[4]);                  display_6_8_number(90,5,rc_data.rcdata[5]);
      LCD_clear_L(0,6);  display_6_8_string(0,6,"ch7-8:");          display_6_8_number(40,6,rc_data.rcdata[6]);                  display_6_8_number(90,6,rc_data.rcdata[7]);			
    }break;	
  case 3:
    {
      LCD_clear_L(0,0);  display_6_8_string(0,0,"Ctrl Data P  I");  display_6_8_number(115,0,page_number+1);
      LCD_clear_L(0,1);  display_6_8_string(0,1,"Pit_Ang:");        display_6_8_number(50,1,maple_ctrl.pitch_angle_ctrl.kp);           display_6_8_number(90,1,maple_ctrl.pitch_angle_ctrl.ki); 
      LCD_clear_L(0,2);  display_6_8_string(0,2,"Rol_Ang:");        display_6_8_number(50,2,maple_ctrl.roll_angle_ctrl.kp);            display_6_8_number(90,2,maple_ctrl.roll_angle_ctrl.ki);    
      LCD_clear_L(0,3);  display_6_8_string(0,3,"Yaw_Ang:");        display_6_8_number(50,3,maple_ctrl.yaw_angle_ctrl.kp);             display_6_8_number(90,3,maple_ctrl.yaw_angle_ctrl.ki);    
      LCD_clear_L(0,4);  display_6_8_string(0,4,"Pit_Gyr:");        display_6_8_number(50,4,maple_ctrl.pitch_gyro_ctrl.kp);            display_6_8_number(90,4,maple_ctrl.pitch_gyro_ctrl.ki);    
      LCD_clear_L(0,5);  display_6_8_string(0,5,"Rol_Gyr:");        display_6_8_number(50,5,maple_ctrl.roll_gyro_ctrl.kp);             display_6_8_number(90,5,maple_ctrl.roll_gyro_ctrl.ki);    
      LCD_clear_L(0,6);  display_6_8_string(0,6,"Yaw_Gyr:");        display_6_8_number(50,6,maple_ctrl.yaw_gyro_ctrl.kp);              display_6_8_number(90,6,maple_ctrl.yaw_gyro_ctrl.ki);
      LCD_clear_L(0,7);  display_6_8_string(0,7,"Gyro_D:");         display_6_8_number(40,7,maple_ctrl.pitch_gyro_ctrl.kd);            display_6_8_number(70,7,maple_ctrl.roll_gyro_ctrl.kd);   			
    }break;
  case 4:
    {
      LCD_clear_L(0,0);  LCD_P6x8Str(0,0,"Top_View_Openmv"); write_6_8_number(115,0,page_number+1);
      LCD_clear_L(0,1);  LCD_P6x8Str(0,1,"UART3_SDK_Mode:"); write_6_8_number(100,1,sdk1_mode_setup);
      
      LCD_clear_L(0,2); 			
      if(sdk1_mode_setup==0x00)     	 LCD_P6x8Str(0,2,"NCQ_SDK_Run");	//默认模式
      else if(sdk1_mode_setup==0x01)     LCD_P6x8Str(0,2,"Color_Block");	//色块追踪模式
      else if(sdk1_mode_setup==0x02)     LCD_P6x8Str(0,2,"Top_APrilTag");	//AprilTag追踪模式
      else if(sdk1_mode_setup==0x03)     LCD_P6x8Str(0,2,"Track_Control");      //自主循迹
      else 
      {
        LCD_P6x8Str(0,2,"User SDK:");
        write_6_8_number(70,2,sdk1_mode_setup-3);
      }			
      
      LCD_clear_L(0,6);LCD_P6x8Str(0,6,"Now_SDK1_Mode:");write_6_8_number(100,6,lookdown_vision.sdk_mode-0xA0);
      
#define ver_up 	(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f \
      &&rc_data.rc_rpyt[RC_PITCH]>Pit_Rol_Max*Scale_Pecent_Max \
        &&rc_data.rc_rpyt[RC_ROLL]==0 \
          &&rc_data.rc_rpyt[RC_YAW]==0)
#define ver_down 	(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f \
&&rc_data.rc_rpyt[RC_PITCH]<-Pit_Rol_Max*Scale_Pecent_Max \
  &&rc_data.rc_rpyt[RC_ROLL]==0 \
    &&rc_data.rc_rpyt[RC_YAW]==0)
if(ver_up)
{
  delay_ms(10);
  if(ver_up)
  {
    sdk1_mode_setup++;
    if(sdk1_mode_setup>32) sdk1_mode_setup=0;
    while(ver_up);
    switch_choose_beep();
    WriteFlashParameter(SDK1_MODE_DEFAULT,sdk1_mode_setup,&Flight_Params);	
  }
}
if(ver_down)
{
  delay_ms(10);
  if(ver_down)
  {
    sdk1_mode_setup--;
    if(sdk1_mode_setup<0) sdk1_mode_setup=32;
    while(ver_down);	
    switch_choose_beep();	
    WriteFlashParameter(SDK1_MODE_DEFAULT,sdk1_mode_setup,&Flight_Params);
  }
}	
#undef ver_up
#undef ver_down

sdk_send_check(sdk1_mode_setup,UART5_SDK);//初始化俯视opemmv工作模式，默认以上次工作状态配置	
    }break;
  case 5:
    {
      ssd1306_clear_display();
      ssd1306_draw_line(0,32,128,32,WHITE);
      ssd1306_draw_line(64,0,64,64,WHITE);			
      uint16_t x,y;
      y=32-32*constrain_float(0.5*lookdown_vision.height-lookdown_vision.y,-50,50)/50.0f;
      x=64-32*constrain_float(0.5*lookdown_vision.width-lookdown_vision.x,-50,50)/50.0f;
      if(lookdown_vision.flag==1)			ssd1306_fill_circle(x,y,5,WHITE);
      else ssd1306_draw_circle(x,y,5,WHITE);			
      
      ssd1306_display();
      display_6_8_string(0,0,"point_x:");  write_6_8_number(80,0,lookdown_vision.x);		write_6_8_number(105,0,page_number+1);
      display_6_8_string(0,1,"point_y::"); write_6_8_number(80,1,lookdown_vision.y);		write_6_8_number(105,1,lookdown_vision.sdk_mode-0xA0);
      display_6_8_string(0,2,"point_s:");  write_6_8_number(80,2,lookdown_vision.pixel);
      display_6_8_string(0,3,"point_f:");  write_6_8_number(80,3,lookdown_vision.flag);
      display_6_8_string(0,4,"Tar_x:");    write_6_8_number(80,4,lookdown_vision.sdk_target.x);
      display_6_8_string(0,5,"Tar_y:");    write_6_8_number(80,5,lookdown_vision.sdk_target.y); 
      display_6_8_string(0,6,"FPS:");      write_6_8_number(80,6,lookdown_vision.fps);
      display_6_8_string(0,7,"Dis:");      write_6_8_number(30,7,maplepilot.roll_outer_control_output);
      write_6_8_number(80,7,maplepilot.pitch_outer_control_output);		
    }
    break;
  case 6:
    {
      ins.gps_lat_deg_f=(int)( gps_data.gps.lat*0.0000001f);
      ins.gps_lat_min_f=(int)((gps_data.gps.lat*0.0000001f-ins.gps_lat_deg_f)*10000000);
      ins.gps_lng_deg_f=(int)( gps_data.gps.lon*0.0000001f);
      ins.gps_lng_min_f=(int)((gps_data.gps.lon*0.0000001f-ins.gps_lng_deg_f)*10000000);
      
      LCD_clear_L(0,0);  display_6_8_string(0,0,"GPS_Date");  	display_6_8_number(115,0,page_number+1);
      LCD_clear_L(0,1);	 display_6_8_string(0,1,"Lo_deg:");			display_6_8_number(50,1,ins.gps_lng_deg_f);
      LCD_clear_L(0,2);	 display_6_8_string(0,2,"d/100w:");			display_6_8_number(50,2,ins.gps_lng_min_f);
      LCD_clear_L(0,3);	 display_6_8_string(0,3,"La_deg:");			display_6_8_number(50,3,ins.gps_lat_deg_f);
      LCD_clear_L(0,4);	 display_6_8_string(0,4,"d/100w:");			display_6_8_number(50,4,ins.gps_lat_min_f);
      LCD_clear_L(0,5);
      if(gps_data.gps.flags.gnssfixok==0x01)	display_6_8_string(0,5,"fix");
      else display_6_8_string(0,5,"nof");
      if(gps_data.gps.fixtype==0x03)	display_6_8_string(25,5,"3D");
      else if(gps_data.gps.fixtype==0x02)	display_6_8_string(25,5,"2D");
      else display_6_8_string(25,5,"0D");
      display_6_8_number(42,5,gps_data.gps.numsv);
      display_6_8_number(60,5,gps_data.gps.hour+8);display_6_8_string(75,5,":");
      display_6_8_number(84,5,gps_data.gps.min);display_6_8_string(100,5,":");
      display_6_8_number(105,5,gps_data.gps.sec);
      LCD_clear_L(0,6);
      display_6_8_string(0,6,"V:");
      display_6_8_number(10,6,gps_data.gps.veln);
      display_6_8_number(65,6,gps_data.gps.vele);
      LCD_clear_L(0,7);
      display_6_8_number(0,7,gps_data.gps.pdop);
      display_6_8_number(80,7,gps_data.gps.hacc);			
    }break;
  case 7:
    {
      LCD_clear_L(0,0);display_6_8_string(0,0,"GPS_Fusion");  display_6_8_number(115,0,page_number+1);
      LCD_clear_L(0,1);display_6_8_string(0,1,"P_E:");				display_6_8_number(30,1,ins.position[_EAST]);    display_6_8_number(95,1,ins.gps_obs_pos_enu[_EAST]);
      LCD_clear_L(0,2);display_6_8_string(0,2,"P_N:");				display_6_8_number(30,2,ins.position[_NORTH]);   display_6_8_number(95,2,ins.gps_obs_pos_enu[_NORTH]);
      LCD_clear_L(0,3);display_6_8_string(0,3,"V_E:");				display_6_8_number(30,3,ins.speed[_EAST]);    	 display_6_8_number(95,3,ins.gps_obs_vel_enu[_EAST]);
      LCD_clear_L(0,4);display_6_8_string(0,4,"V_N:");				display_6_8_number(30,4,ins.speed[_NORTH]);    	 display_6_8_number(95,4,ins.gps_obs_vel_enu[_NORTH]);		
      LCD_clear_L(0,5);display_6_8_string(0,5,"A_B:");	 			display_6_8_number(30,5,ins.acce_bias[_EAST]);	 display_6_8_number(95,5,ins.gps_home_fixed_flag);  
      LCD_clear_L(0,6);display_6_8_string(0,6,"GPS:"); 			  display_6_8_number(30,6,gps_data.gps.numsv);	  		 display_6_8_number(100,6,ins.gps_hacc_m);
      LCD_clear_L(0,7);display_6_8_string(0,7,"homefix:");    display_6_8_number(60,7,ins.gps_home_fixed_flag);display_6_8_number(90,7,ins.gps_pdop);	
    }break;
  case 8:
    {
      LCD_clear_L(0,1);  display_6_8_string(0,1,"flow_x:");        display_6_8_number_pro(50,1,opt_data.x);     display_6_8_number_pro(90,1,opt_data.flow_filter.x); 
      LCD_clear_L(0,2);  display_6_8_string(0,2,"flow_y:");        display_6_8_number_pro(50,2,opt_data.y);     display_6_8_number_pro(90,2,opt_data.flow_filter.y);    
      LCD_clear_L(0,3);  display_6_8_string(0,3,"time:");       	 display_6_8_number(50,3,opt_data.dt);    		display_6_8_number(90,3,opt_data.ms);    
      LCD_clear_L(0,4);  display_6_8_string(0,4,"qual:");      		 display_6_8_number(50,4,opt_data.qual);            									  
      LCD_clear_L(0,5);  display_6_8_string(0,5,"fxy:");        	 display_6_8_number_pro(30,5,opt_data.flow_filter.x);	display_6_8_number_pro(80,5,opt_data.flow_filter.y);               
      LCD_clear_L(0,6);  display_6_8_string(0,6,"gxy:");        	 display_6_8_number_pro(30,6,opt_data.gyro_filter.x);	display_6_8_number_pro(80,6,opt_data.gyro_filter.y);            
      LCD_clear_L(0,7);  display_6_8_string(0,7,"fopt:");          display_6_8_number_pro(30,7,opt_data.flow_correct.x);display_6_8_number_pro(80,7,opt_data.flow_correct.y);  
    }break;
  case 9:
    {
      LCD_clear_L(0,1);  display_6_8_string(0,1,"p_v_x:");    display_6_8_number(45,1,current_state.position_x); display_6_8_number(100,1,current_state.velocity_x); 
      LCD_clear_L(0,2);  display_6_8_string(0,2,"p_v_y:");    display_6_8_number(45,2,current_state.position_y); display_6_8_number(100,2,current_state.velocity_y);         
      LCD_clear_L(0,3);  display_6_8_string(0,3,"p_v_z:");    display_6_8_number(45,3,current_state.position_z); display_6_8_number(100,3,current_state.velocity_z);   	   
      LCD_clear_L(0,4);  display_6_8_string(0,4,"pitch:");    display_6_8_number(45,4,current_state.rpy[1]);	   display_6_8_number(100,4,min_dis_cm);   									  
      LCD_clear_L(0,5);  display_6_8_string(0,5,"roll:");     display_6_8_number(45,5,current_state.rpy[0]);		 display_6_8_number(100,5,min_dis_angle);
      LCD_clear_L(0,6);  display_6_8_string(0,6,"yaw:");      display_6_8_number(45,6,current_state.rpy[2]);  	 display_6_8_number(100,6,target_yaw_err); 
      LCD_clear_L(0,7);	 display_6_8_string(0,7,"qual_update:");
      display_6_8_number(60,7,current_state.quality);
      display_6_8_number(100,7,current_state.update_flag	);
      
      //上锁状态下才允许设置
      //if(Controler_State==Unlock_Controler) return;
      
      
      //SLAM建图复位  
      //油门低位、俯仰下位、横滚中位、偏航左位
#define reset_slam 	(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f \
      &&rc_data.rc_rpyt[RC_PITCH]>Pit_Rol_Max*Scale_Pecent_Max \
        &&rc_data.rc_rpyt[RC_ROLL]==0 \
          &&rc_data.rc_rpyt[RC_YAW]>Yaw_Max*Scale_Pecent_Max)				
if(reset_slam)
{
  delay_ms(10);
  if(reset_slam)
  {
    send_check_back=4;//重置slam
    while(reset_slam);				
  }
}
#undef reset_slam			
    }break;
  case 10:
    {
      LCD_clear_L(0,0);
      display_6_8_string(10,0,"Sensor Data");   display_6_8_number(115,0,page_number+1);
      LCD_clear_L(0,2);display_6_8_string(0,2,"mag:");
      display_6_8_number(25,2,flymaple.mag_raw_data_tesla[0]);
      display_6_8_number(60,2,flymaple.mag_raw_data_tesla[1]);
      display_6_8_number(95,2,flymaple.mag_raw_data_tesla[2]);
      LCD_clear_L(0,3);display_6_8_string(0,3,"acc:");
      display_6_8_number(25,3,flymaple.accel_raw_data[0]);
      display_6_8_number(60,3,flymaple.accel_raw_data[1]);
      display_6_8_number(95,3,flymaple.accel_raw_data[2]);
      LCD_clear_L(0,4);display_6_8_string(0,4,"gyr:");
      display_6_8_number(25,4,flymaple.gyro_raw_data[0]);
      display_6_8_number(60,4,flymaple.gyro_raw_data[1]);
      display_6_8_number(95,4,flymaple.gyro_raw_data[2]);
      LCD_clear_L(0,5);display_6_8_string(0,5,"bar:");
      display_6_8_number(25,5,flymaple.baro_p);
      display_6_8_number(70,5,flymaple.baro_t);
      LCD_clear_L(0,6);display_6_8_string(0,6,"opt:");
      display_6_8_number(25,6,opt_data.x);
      display_6_8_number(60,6,opt_data.y);
    }
    break;
  case 11://按键键入目标地点
    {
      static uint8_t ver_choose=1;
      LCD_clear_L(0,0); display_6_8_string(0,0,"reserved_params");	display_6_8_number(115,0,page_number+1);
      LCD_clear_L(0,1); display_6_8_string(0,1,"scale:"); display_6_8_number(50,1,param_value[0]);//比例尺度参数A  
      //最终的坐标由A*(x,y)确定，比如25*(8,11)=(200,275)
      
      //第一个坐标x1，y1
      LCD_clear_L(0,2); display_6_8_string(0,2,"x1:");  display_6_8_number(50,2,param_value[1]);   display_6_8_number(90,2,param_value[1]*param_value[0]);
      LCD_clear_L(0,3); display_6_8_string(0,3,"y1:");  display_6_8_number(50,3,param_value[2]);   display_6_8_number(90,3,param_value[2]*param_value[0]);
      //第二个坐标x2，y2																												 
      LCD_clear_L(0,4); display_6_8_string(0,4,"x2:");  display_6_8_number(50,4,param_value[3]);	 display_6_8_number(90,4,param_value[3]*param_value[0]);
      LCD_clear_L(0,5); display_6_8_string(0,5,"y2:");  display_6_8_number(50,5,param_value[4]);   display_6_8_number(90,5,param_value[4]*param_value[0]);
      //第三个坐标x3，y3
      LCD_clear_L(0,6); display_6_8_string(0,6,"x3:");  display_6_8_number(50,6,param_value[5]);	 display_6_8_number(90,6,param_value[5]*param_value[0]);
      LCD_clear_L(0,7); display_6_8_string(0,7,"y3:");  display_6_8_number(50,7,param_value[6]);	 display_6_8_number(90,7,param_value[6]*param_value[0]);
      
      display_6_8_string(40,ver_choose,"*");				
      //通过3D按键来实现换行选中待修改参数
      if(_button.state[UP_3D].press==SHORT_PRESS)
      {
        _button.state[UP_3D].press=NO_PRESS;
        ver_choose--;
        if(ver_choose<1) ver_choose=7;		
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色	
      }
      if(_button.state[DN_3D].press==SHORT_PRESS)
      {
        _button.state[DN_3D].press=NO_PRESS;
        ver_choose++;
        if(ver_choose>7) ver_choose=1;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色					
      }
      
      
      //通过3D按键来实现可以实现选中的参数行自增加调整
      if(_button.state[RT_3D].press==SHORT_PRESS)
      {
        _button.state[RT_3D].press=NO_PRESS;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色	
        switch(ver_choose)
        {
        case 1://修改比例系数
          {
            param_value[0]+=5;
            if(param_value[0]>100) param_value[0]=0;
          }
          break;
        case 2://x1
          {
            param_value[1]+=1;
            if(param_value[1]>20) param_value[1]=-20;
          }
          break;						
        case 3://y1
          {
            param_value[2]+=1;
            if(param_value[2]>20) param_value[2]=-20;
          }
          break;
        case 4://x2
          {
            param_value[3]+=1;
            if(param_value[3]>20) param_value[3]=-20;
          }
          break;						
        case 5://y2
          {
            param_value[4]+=1;
            if(param_value[4]>20) param_value[4]=-20;
          }
          break;	
        case 6://x3
          {
            param_value[5]+=1;
            if(param_value[5]>20) param_value[5]=-20;
          }
          break;						
        case 7://y3
          {
            param_value[6]+=1;
            if(param_value[6]>20) param_value[6]=-20;
          }
          break;								
        }
        //按下后对参数进行保存
        WriteFlashParameter(RESERVED_PARAM,  param_value[0],&Flight_Params);
        WriteFlashParameter(RESERVED_PARAM+1,param_value[1],&Flight_Params);
        WriteFlashParameter(RESERVED_PARAM+2,param_value[2],&Flight_Params);
        WriteFlashParameter(RESERVED_PARAM+3,param_value[3],&Flight_Params);
        WriteFlashParameter(RESERVED_PARAM+4,param_value[4],&Flight_Params);
        WriteFlashParameter(RESERVED_PARAM+5,param_value[5],&Flight_Params);
        WriteFlashParameter(RESERVED_PARAM+6,param_value[6],&Flight_Params);
      }	
      //通过3D按键来实现可以实现选中的参数行减小调整
      if(_button.state[LT_3D].press==SHORT_PRESS)
      {
        _button.state[LT_3D].press=NO_PRESS;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色		
        switch(ver_choose)
        {
        case 1://修改比例系数
          {
            param_value[0]-=5;
            if(param_value[0]<0) param_value[0]=100;
          }
          break;
        case 2://x1
          {
            param_value[1]-=1;
            if(param_value[1]<-20) param_value[1]=20;
          }
          break;						
        case 3://y1
          {
            param_value[2]-=1;
            if(param_value[2]<-20) param_value[2]=20;
          }
          break;
        case 4://x2
          {
            param_value[3]-=1;
            if(param_value[3]<-20) param_value[3]=20;
          }
          break;						
        case 5://y2
          {
            param_value[4]-=1;
            if(param_value[4]<-20) param_value[4]=20;
          }
          break;	
        case 6://x3
          {
            param_value[5]-=1;
            if(param_value[5]<-20) param_value[5]=20;
          }
          break;						
        case 7://y3
          {
            param_value[6]-=1;
            if(param_value[6]<-20) param_value[6]=20;
          }
          break;								
        }
        //按下后对参数进行保存
        WriteFlashParameter(RESERVED_PARAM,  param_value[0],&Flight_Params);
        WriteFlashParameter(RESERVED_PARAM+1,param_value[1],&Flight_Params);
        WriteFlashParameter(RESERVED_PARAM+2,param_value[2],&Flight_Params);
        WriteFlashParameter(RESERVED_PARAM+3,param_value[3],&Flight_Params);
        WriteFlashParameter(RESERVED_PARAM+4,param_value[4],&Flight_Params);
        WriteFlashParameter(RESERVED_PARAM+5,param_value[5],&Flight_Params);
        WriteFlashParameter(RESERVED_PARAM+6,param_value[6],&Flight_Params);
      }				
    }
    break;	
  case 12:
    {
      ssd1306_clear_display();
      ssd1306_draw_line(0,32,128,32,WHITE);
      ssd1306_draw_line(64,0,64,64,WHITE);			
      uint16_t x,y;
      y=32-32*constrain_float(0.5*lookdown_vision.height-lookdown_vision.y,-50,50)/50.0f;
      x=64-32*constrain_float(0.5*lookdown_vision.width-lookdown_vision.x,-50,50)/50.0f;
      if(lookdown_vision.flag==1)			ssd1306_fill_circle(x,y,2,WHITE);
      else ssd1306_draw_circle(x,y,2,WHITE);			
      
      display_6_8_string(0,0,"px:");  write_6_8_number(80,0,lookdown_vision.x);		write_6_8_number(105,0,page_number+1);
      display_6_8_string(0,1,"py:");  write_6_8_number(80,1,lookdown_vision.y);		write_6_8_number(105,1,lookdown_vision.sdk_mode-0xA0);
      display_6_8_string(0,2,"sq:");  write_6_8_number(80,2,lookdown_vision.pixel);
      display_6_8_string(0,3,"fg:");  write_6_8_number(80,3,lookdown_vision.flag);
      display_6_8_string(0,4,"cmx:"); write_6_8_number(60,4,lookdown_vision.sdk_target.x);	write_6_8_number(110,4,lookdown_vision.fps);
      display_6_8_string(0,5,"cmy:"); write_6_8_number(60,5,lookdown_vision.sdk_target.y); 
      display_6_8_string(0,6,"type:");
      
      if(lookdown_vision.reserved3==1) 			display_6_8_string(30,6,"red");
      else if(lookdown_vision.reserved3==2) display_6_8_string(30,6,"blue");
      else display_6_8_string(30,6,"unk");
      
      if(lookdown_vision.reserved4==1) 			
      {
        display_6_8_string(65,6,"circular");
        ssd1306_draw_circle(64,32,16,WHITE);
      }
      else if(lookdown_vision.reserved4==2) 
      {
        display_6_8_string(65,6,"rectangle");
        ssd1306_draw_rect(48,16,32,32,WHITE);	
      }
      else if(lookdown_vision.reserved4==3) 
      {
        display_6_8_string(65,6,"triangle");
        ssd1306_draw_triangle(48,48,80,48,64,21,WHITE);
      }
      else display_6_8_string(65,6,"unk");			
      ssd1306_display();
      
      display_6_8_string(0,7,"color");    
      display_6_8_number(80,7,param_value[7]);	 
      display_6_8_number(110,7,param_value[8]);
      
      static uint8_t hor_choose=1;
      display_6_8_string(40+30*(hor_choose-1),7,"*");
      //通过3D按键来实现换行选中待修改参数
      if(_button.state[LT_3D].press==SHORT_PRESS)
      {
        _button.state[LT_3D].press=NO_PRESS;
        hor_choose--;
        if(hor_choose<1) hor_choose=3;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色				
      }
      if(_button.state[RT_3D].press==SHORT_PRESS)
      {
        _button.state[RT_3D].press=NO_PRESS;
        hor_choose++;
        if(hor_choose>3) hor_choose=1;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色		
      }
      
      
      
      //通过下一页按键长按可以实现选中的参数行自增加调整
      if(_button.state[DN_3D].press==SHORT_PRESS)
      {
        _button.state[DN_3D].press=NO_PRESS;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色
        switch(hor_choose)
        {
        case 1://将OPENMV识别到的模板特征作为待检测目标
          {
            param_value[7]=lookdown_vision.reserved3;//颜色
            param_value[8]=lookdown_vision.reserved4;//形状
          }
          break;								
        case 2://可对模板颜色做进一步修改
          {
            param_value[7]+=1;
            if(param_value[7]>2) param_value[7]=1;
          }
          break;
        case 3://可对模板形状做进一步修改
          {
            param_value[8]+=1;
            if(param_value[8]>3) param_value[8]=1;
          }
          break;	
        }
        //按下后对参数进行保存
        WriteFlashParameter(RESERVED_PARAM+7,param_value[7],&Flight_Params);//颜色
        WriteFlashParameter(RESERVED_PARAM+8,param_value[8],&Flight_Params);//形状
      }
      
      //通过下一页按键长按可以实现选中的参数行自增加调整
      if(_button.state[UP_3D].press==SHORT_PRESS)
      {
        _button.state[UP_3D].press=NO_PRESS;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色
        switch(hor_choose)
        {
        case 1://将OPENMV识别到的模板特征作为待检测目标
          {
            param_value[7]=lookdown_vision.reserved3;//颜色
            param_value[8]=lookdown_vision.reserved4;//形状
          }
          break;								
        case 2://可对模板颜色做进一步修改
          {
            param_value[7]-=1;
            if(param_value[7]<1) param_value[7]=2;
          }
          break;
        case 3://可对模板形状做进一步修改
          {
            param_value[8]-=1;
            if(param_value[8]<1) param_value[8]=3;
          }
          break;	
        }
        //按下后对参数进行保存
        WriteFlashParameter(RESERVED_PARAM+7,param_value[7],&Flight_Params);//颜色
        WriteFlashParameter(RESERVED_PARAM+8,param_value[8],&Flight_Params);//形状
      }				
    }
    break;	
  case 13:
    {
      static uint8_t ver_item=1;
      static uint16_t step=1;
      LCD_clear_L(0,0);display_6_8_string(25,0,"loop_setup");   write_6_8_number(105,0,page_number+1);
      LCD_clear_L(0,1);display_6_8_string(0,1,"step");          write_6_8_number(80,1,step);
      LCD_clear_L(0,2);display_6_8_string(0,2,"A_x");           write_6_8_number(80,2,param_value[9]);
      LCD_clear_L(0,3);display_6_8_string(0,3,"A_y");           write_6_8_number(80,3,param_value[10]);
      LCD_clear_L(0,4);display_6_8_string(0,4,"ang");           write_6_8_number(80,4,param_value[11]);
      LCD_clear_L(0,5);display_6_8_string(0,5,"save");          display_6_8_string(31,5,"press left/right");
      
      display_6_8_string(25,ver_item,"*");
      //通过3D按键来实现换行选中待修改参数
      if(_button.state[UP_3D].press==SHORT_PRESS)
      {
        _button.state[UP_3D].press=NO_PRESS;
        ver_item--;
        if(ver_item<1) ver_item=5;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色				
      }
      if(_button.state[DN_3D].press==SHORT_PRESS)
      {
        _button.state[DN_3D].press=NO_PRESS;
        ver_item++;
        if(ver_item>5) ver_item=1;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色			
      }
      
      
      //通过下一页按键持续长按可以实现选中的参数行自增加调整
      if(_button.state[RT_3D].press==SHORT_PRESS)
      {
        _button.state[RT_3D].press=NO_PRESS;
        switch(ver_item)
        {
        case 1:
          {
            step+=1;
            if(step>100)	step=100;
          }
          break;
        case 2:
          {
            param_value[9]+=step;
            if(param_value[9]>500) param_value[9]=500;
          }
          break;
        case 3:
          {
            param_value[10]+=step;	
            if(param_value[10]>500) param_value[10]=500;
          }
          break;
        case 4:
          {
            param_value[11]+=step;
            if(param_value[11]>180) param_value[11]=180;
          }
          break;
        case 5:
          {
            //按下后对参数进行保存
            WriteFlashParameter(RESERVED_PARAM+9 ,param_value[9],&Flight_Params);
            WriteFlashParameter(RESERVED_PARAM+10,param_value[10],&Flight_Params);
            WriteFlashParameter(RESERVED_PARAM+11,param_value[11],&Flight_Params);
            rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色
          }
          break;
        }
      }
      
      if(_button.state[LT_3D].press==SHORT_PRESS)
      {
        _button.state[LT_3D].press=NO_PRESS;
        switch(ver_item)
        {
        case 1:
          {
            step-=1;
            if(step<1)	step=1;
          }
          break;
        case 2:
          {
            param_value[9]-=step;
            if(param_value[9]<-500) param_value[9]=-500;
          }
          break;
        case 3:
          {
            param_value[10]-=step;	
            if(param_value[10]<-500) param_value[10]=-500;
          }
          break;
        case 4:
          {
            param_value[11]-=step;
            if(param_value[11]<0) param_value[11]=0;
          }
          break;
        case 5:
          {
            //按下后对参数进行保存
            WriteFlashParameter(RESERVED_PARAM+9 ,param_value[9],&Flight_Params);
            WriteFlashParameter(RESERVED_PARAM+10,param_value[10],&Flight_Params);
            WriteFlashParameter(RESERVED_PARAM+11,param_value[11],&Flight_Params);
            rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色	
          }
          break;
        }
      }
    }
    break;
  case 14:
    {
      uint16_t base_item=50;
      LCD_clear_L(0,0);		display_6_8_string(0,0,"nav_point:1/7 e-n-u");  write_6_8_number(115,0,page_number+1);
      LCD_clear_L(0,1);		display_6_8_string(0,1,"p1");                   write_6_8_number(25,1,param_value[base_item+0]);  write_6_8_number(60,1,param_value[base_item+1]);  write_6_8_number(95,1,param_value[base_item+2]);
      LCD_clear_L(0,2);		display_6_8_string(0,2,"p2");                   write_6_8_number(25,2,param_value[base_item+3]);  write_6_8_number(60,2,param_value[base_item+4]);  write_6_8_number(95,2,param_value[base_item+5]);
      LCD_clear_L(0,3);		display_6_8_string(0,3,"p3");                   write_6_8_number(25,3,param_value[base_item+6]);  write_6_8_number(60,3,param_value[base_item+7]);  write_6_8_number(95,3,param_value[base_item+8]);
      LCD_clear_L(0,4);		display_6_8_string(0,4,"p4");                   write_6_8_number(25,4,param_value[base_item+9]);  write_6_8_number(60,4,param_value[base_item+10]); write_6_8_number(95,4,param_value[base_item+11]);
      LCD_clear_L(0,5);		display_6_8_string(0,5,"p5");                   write_6_8_number(25,5,param_value[base_item+12]); write_6_8_number(60,5,param_value[base_item+13]); write_6_8_number(95,5,param_value[base_item+14]);
      LCD_clear_L(0,6);		display_6_8_string(0,6,"p6");                   write_6_8_number(25,6,param_value[base_item+15]); write_6_8_number(60,6,param_value[base_item+16]); write_6_8_number(95,6,param_value[base_item+17]);
      LCD_clear_L(0,7);		display_6_8_string(0,7,"p7");                   write_6_8_number(25,7,param_value[base_item+18]); write_6_8_number(60,7,param_value[base_item+19]);	write_6_8_number(95,7,param_value[base_item+20]);
      
      static uint16_t ver_item=1;	
      if(ver_item==1)	display_6_8_string(18,1,"*"); else if(ver_item==2) display_6_8_string(53,1,"*"); else if(ver_item==3) display_6_8_string(88,1,"*");
      if(ver_item==4)  	display_6_8_string(18,2,"*"); else if(ver_item==5) display_6_8_string(53,2,"*"); else if(ver_item==6) display_6_8_string(88,2,"*");
      if(ver_item==7)	display_6_8_string(18,3,"*"); else if(ver_item==8) display_6_8_string(53,3,"*"); else if(ver_item==9) display_6_8_string(88,3,"*");
      if(ver_item==10)	display_6_8_string(18,4,"*"); else if(ver_item==11)display_6_8_string(53,4,"*"); else if(ver_item==12)display_6_8_string(88,4,"*");
      if(ver_item==13)	display_6_8_string(18,5,"*"); else if(ver_item==14)display_6_8_string(53,5,"*"); else if(ver_item==15)display_6_8_string(88,5,"*");
      if(ver_item==16)	display_6_8_string(18,6,"*"); else if(ver_item==17)display_6_8_string(53,6,"*"); else if(ver_item==18)display_6_8_string(88,6,"*");
      if(ver_item==19)	display_6_8_string(18,7,"*"); else if(ver_item==20)display_6_8_string(53,7,"*"); else if(ver_item==21)display_6_8_string(88,7,"*");
      
      //通过3D按键来实现换行选中待修改参数
      if(_button.state[UP_3D].press==SHORT_PRESS)
      {
        _button.state[UP_3D].press=NO_PRESS;
        ver_item--;
        if(ver_item<1) ver_item=21;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色				
      }
      if(_button.state[DN_3D].press==SHORT_PRESS)
      {
        _button.state[DN_3D].press=NO_PRESS;
        ver_item++;
        if(ver_item>21) ver_item=1;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色			
      }
      
      //通过左按键短按可以实现选中的参数行自减小1调整
      if(_button.state[LT_3D].press==SHORT_PRESS)
      {
        _button.state[LT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]-=1;
      }
      
      //通过右按键短按可以实现选中的参数行自增加1调整
      if(_button.state[RT_3D].press==SHORT_PRESS)
      {
        _button.state[RT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]+=1;
      }			
      
      //通过左按键短按可以实现选中的参数行自减小50调整
      if(_button.state[LT_3D].press==LONG_PRESS)
      {
        _button.state[LT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]-=50;
      }
      
      //通过右按键短按可以实现选中的参数行自增加50调整
      if(_button.state[RT_3D].press==LONG_PRESS)
      {
        _button.state[RT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]+=50;
      }		
      
      //通过中间按键长按可以实现此页设置的所有参数保存
      if(_button.state[ME_3D].press==LONG_PRESS)
      {
        _button.state[ME_3D].press=NO_PRESS;
        //按下后对参数进行保存
        for(uint16_t i=0;i<21;i++)
        {
          WriteFlashParameter(RESERVED_PARAM+base_item+i,param_value[base_item+i],&Flight_Params);
        }
      }				
    }
    break;
  case 15:
    {
      uint16_t base_item=50+21*1;
      LCD_clear_L(0,0);		display_6_8_string(0,0,"nav_point:8/14 e-n-u");write_6_8_number(115,0,page_number+1);
      LCD_clear_L(0,1);		display_6_8_string(0,1,"p8");                   write_6_8_number(25,1,param_value[base_item+0]);  write_6_8_number(60,1,param_value[base_item+1]);  write_6_8_number(95,1,param_value[base_item+2]);
      LCD_clear_L(0,2);		display_6_8_string(0,2,"p9");                   write_6_8_number(25,2,param_value[base_item+3]);  write_6_8_number(60,2,param_value[base_item+4]);  write_6_8_number(95,2,param_value[base_item+5]);
      LCD_clear_L(0,3);		display_6_8_string(0,3,"p10");                  write_6_8_number(25,3,param_value[base_item+6]);  write_6_8_number(60,3,param_value[base_item+7]);  write_6_8_number(95,3,param_value[base_item+8]);
      LCD_clear_L(0,4);		display_6_8_string(0,4,"p11");	                write_6_8_number(25,4,param_value[base_item+9]);  write_6_8_number(60,4,param_value[base_item+10]); write_6_8_number(95,4,param_value[base_item+11]);
      LCD_clear_L(0,5);		display_6_8_string(0,5,"p12");                  write_6_8_number(25,5,param_value[base_item+12]); write_6_8_number(60,5,param_value[base_item+13]); write_6_8_number(95,5,param_value[base_item+14]);
      LCD_clear_L(0,6);		display_6_8_string(0,6,"p13");	                write_6_8_number(25,6,param_value[base_item+15]); write_6_8_number(60,6,param_value[base_item+16]); write_6_8_number(95,6,param_value[base_item+17]);
      LCD_clear_L(0,7);		display_6_8_string(0,7,"p14");	                write_6_8_number(25,7,param_value[base_item+18]); write_6_8_number(60,7,param_value[base_item+19]); write_6_8_number(95,7,param_value[base_item+20]);
      
      static uint16_t ver_item=1;	
      if(ver_item==1)	  display_6_8_string(18,1,"*"); else if(ver_item==2)	display_6_8_string(53,1,"*"); else if(ver_item==3)	display_6_8_string(88,1,"*");
      if(ver_item==4)  	display_6_8_string(18,2,"*"); else if(ver_item==5)	display_6_8_string(53,2,"*"); else if(ver_item==6)	display_6_8_string(88,2,"*");
      if(ver_item==7)	  display_6_8_string(18,3,"*"); else if(ver_item==8)	display_6_8_string(53,3,"*"); else if(ver_item==9)	display_6_8_string(88,3,"*");
      if(ver_item==10)	display_6_8_string(18,4,"*"); else if(ver_item==11)	display_6_8_string(53,4,"*"); else if(ver_item==12)	display_6_8_string(88,4,"*");
      if(ver_item==13)	display_6_8_string(18,5,"*"); else if(ver_item==14)	display_6_8_string(53,5,"*"); else if(ver_item==15)	display_6_8_string(88,5,"*");
      if(ver_item==16)	display_6_8_string(18,6,"*"); else if(ver_item==17)	display_6_8_string(53,6,"*"); else if(ver_item==18)	display_6_8_string(88,6,"*");
      if(ver_item==19)	display_6_8_string(18,7,"*"); else if(ver_item==20)	display_6_8_string(53,7,"*"); else if(ver_item==21)	display_6_8_string(88,7,"*");
      
      //通过3D按键来实现换行选中待修改参数
      if(_button.state[UP_3D].press==SHORT_PRESS)
      {
        _button.state[UP_3D].press=NO_PRESS;
        ver_item--;
        if(ver_item<1) ver_item=21;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色				
      }
      if(_button.state[DN_3D].press==SHORT_PRESS)
      {
        _button.state[DN_3D].press=NO_PRESS;
        ver_item++;
        if(ver_item>21) ver_item=1;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色				
      }
      
      //通过左按键短按可以实现选中的参数行自减小1调整
      if(_button.state[LT_3D].press==SHORT_PRESS)
      {
        _button.state[LT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]-=1;
      }
      
      //通过右按键短按可以实现选中的参数行自增加1调整
      if(_button.state[RT_3D].press==SHORT_PRESS)
      {
        _button.state[RT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]+=1;
      }			
      
      //通过左按键短按可以实现选中的参数行自减小50调整
      if(_button.state[LT_3D].press==LONG_PRESS)
      {
        _button.state[LT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]-=50;
      }
      
      //通过右按键短按可以实现选中的参数行自增加50调整
      if(_button.state[RT_3D].press==LONG_PRESS)
      {
        _button.state[RT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]+=50;
      }		
      
      //通过中间按键长按可以实现此页设置的所有参数保存
      if(_button.state[ME_3D].press==LONG_PRESS)
      {
        _button.state[ME_3D].press=NO_PRESS;
        //按下后对参数进行保存
        for(uint16_t i=0;i<21;i++)
        {
          WriteFlashParameter(RESERVED_PARAM+base_item+i,param_value[base_item+i],&Flight_Params);
        }
      }				
    }
    break;
  case 16:
    {
      uint16_t base_item=50+21*2;
      LCD_clear_L(0,0);		display_6_8_string(0,0,"nav_point:15/21 e-n-u"); write_6_8_number(115,0,page_number+1);
      LCD_clear_L(0,1);		display_6_8_string(0,1,"p15");   		 write_6_8_number(25,1,param_value[base_item+0]);  write_6_8_number(60,1,param_value[base_item+1]);  write_6_8_number(95,1,param_value[base_item+2]);
      LCD_clear_L(0,2);		display_6_8_string(0,2,"p16");  		 write_6_8_number(25,2,param_value[base_item+3]);  write_6_8_number(60,2,param_value[base_item+4]);  write_6_8_number(95,2,param_value[base_item+5]);
      LCD_clear_L(0,3);		display_6_8_string(0,3,"p17");  		 write_6_8_number(25,3,param_value[base_item+6]);  write_6_8_number(60,3,param_value[base_item+7]);  write_6_8_number(95,3,param_value[base_item+8]);
      LCD_clear_L(0,4);		display_6_8_string(0,4,"p18");			 write_6_8_number(25,4,param_value[base_item+9]);  write_6_8_number(60,4,param_value[base_item+10]); write_6_8_number(95,4,param_value[base_item+11]);
      LCD_clear_L(0,5);		display_6_8_string(0,5,"p19");			 write_6_8_number(25,5,param_value[base_item+12]); write_6_8_number(60,5,param_value[base_item+13]); write_6_8_number(95,5,param_value[base_item+14]);
      LCD_clear_L(0,6);		display_6_8_string(0,6,"p20");			 write_6_8_number(25,6,param_value[base_item+15]); write_6_8_number(60,6,param_value[base_item+16]); write_6_8_number(95,6,param_value[base_item+17]);
      LCD_clear_L(0,7);		display_6_8_string(0,7,"p21");			 write_6_8_number(25,7,param_value[base_item+18]); write_6_8_number(60,7,param_value[base_item+19]); write_6_8_number(95,7,param_value[base_item+20]);
      
      static uint16_t ver_item=1;	
      if(ver_item==1)	  display_6_8_string(18,1,"*"); else if(ver_item==2)	display_6_8_string(53,1,"*"); else if(ver_item==3)	display_6_8_string(88,1,"*");
      if(ver_item==4)  	display_6_8_string(18,2,"*"); else if(ver_item==5)	display_6_8_string(53,2,"*"); else if(ver_item==6)	display_6_8_string(88,2,"*");
      if(ver_item==7)	  display_6_8_string(18,3,"*"); else if(ver_item==8)	display_6_8_string(53,3,"*"); else if(ver_item==9)	display_6_8_string(88,3,"*");
      if(ver_item==10)	display_6_8_string(18,4,"*"); else if(ver_item==11)	display_6_8_string(53,4,"*"); else if(ver_item==12)	display_6_8_string(88,4,"*");
      if(ver_item==13)	display_6_8_string(18,5,"*"); else if(ver_item==14)	display_6_8_string(53,5,"*"); else if(ver_item==15)	display_6_8_string(88,5,"*");
      if(ver_item==16)	display_6_8_string(18,6,"*"); else if(ver_item==17)	display_6_8_string(53,6,"*"); else if(ver_item==18)	display_6_8_string(88,6,"*");
      if(ver_item==19)	display_6_8_string(18,7,"*"); else if(ver_item==20)	display_6_8_string(53,7,"*"); else if(ver_item==21)	display_6_8_string(88,7,"*");
      
      //通过3D按键来实现换行选中待修改参数
      if(_button.state[UP_3D].press==SHORT_PRESS)
      {
        _button.state[UP_3D].press=NO_PRESS;
        ver_item--;
        if(ver_item<1) ver_item=21;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色				
      }
      if(_button.state[DN_3D].press==SHORT_PRESS)
      {
        _button.state[DN_3D].press=NO_PRESS;
        ver_item++;
        if(ver_item>21) ver_item=1;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色				
      }
      
      //通过左按键短按可以实现选中的参数行自减小1调整
      if(_button.state[LT_3D].press==SHORT_PRESS)
      {
        _button.state[LT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]-=1;
      }
      
      //通过右按键短按可以实现选中的参数行自增加1调整
      if(_button.state[RT_3D].press==SHORT_PRESS)
      {
        _button.state[RT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]+=1;
      }			
      
      //通过左按键短按可以实现选中的参数行自减小50调整
      if(_button.state[LT_3D].press==LONG_PRESS)
      {
        _button.state[LT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]-=50;
      }
      
      //通过右按键短按可以实现选中的参数行自增加50调整
      if(_button.state[RT_3D].press==LONG_PRESS)
      {
        _button.state[RT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]+=50;
      }		
      
      //通过中间按键长按可以实现此页设置的所有参数保存
      if(_button.state[ME_3D].press==LONG_PRESS)
      {
        _button.state[ME_3D].press=NO_PRESS;
        //按下后对参数进行保存
        for(uint16_t i=0;i<21;i++)
        {
          WriteFlashParameter(RESERVED_PARAM+base_item+i,param_value[base_item+i],&Flight_Params);
        }
      }				
    }
    break;
  case 17:
    {
      uint16_t base_item=50+21*3;
      LCD_clear_L(0,0);		display_6_8_string(0,0,"nav_point:22/28 e-n-u");write_6_8_number(115,0,page_number+1);
      LCD_clear_L(0,1);		display_6_8_string(0,1,"p22");   		write_6_8_number(25,1,param_value[base_item+0]);  write_6_8_number(60,1,param_value[base_item+1]);  write_6_8_number(95,1,param_value[base_item+2]);
      LCD_clear_L(0,2);		display_6_8_string(0,2,"p23");  	        write_6_8_number(25,2,param_value[base_item+3]);  write_6_8_number(60,2,param_value[base_item+4]);  write_6_8_number(95,2,param_value[base_item+5]);
      LCD_clear_L(0,3);		display_6_8_string(0,3,"p24");  	        write_6_8_number(25,3,param_value[base_item+6]);  write_6_8_number(60,3,param_value[base_item+7]);  write_6_8_number(95,3,param_value[base_item+8]);
      LCD_clear_L(0,4);		display_6_8_string(0,4,"p25");		        write_6_8_number(25,4,param_value[base_item+9]);  write_6_8_number(60,4,param_value[base_item+10]); write_6_8_number(95,4,param_value[base_item+11]);
      LCD_clear_L(0,5);		display_6_8_string(0,5,"p26");                  write_6_8_number(25,5,param_value[base_item+12]); write_6_8_number(60,5,param_value[base_item+13]); write_6_8_number(95,5,param_value[base_item+14]);
      LCD_clear_L(0,6);		display_6_8_string(0,6,"p27");		        write_6_8_number(25,6,param_value[base_item+15]); write_6_8_number(60,6,param_value[base_item+16]); write_6_8_number(95,6,param_value[base_item+17]);
      LCD_clear_L(0,7);		display_6_8_string(0,7,"p28");		        write_6_8_number(25,7,param_value[base_item+18]); write_6_8_number(60,7,param_value[base_item+19]); write_6_8_number(95,7,param_value[base_item+20]);
      
      static uint16_t ver_item=1;	
      if(ver_item==1)	  display_6_8_string(18,1,"*"); else if(ver_item==2)	display_6_8_string(53,1,"*"); else if(ver_item==3)	display_6_8_string(88,1,"*");
      if(ver_item==4)  	display_6_8_string(18,2,"*"); else if(ver_item==5)	display_6_8_string(53,2,"*"); else if(ver_item==6)	display_6_8_string(88,2,"*");
      if(ver_item==7)	  display_6_8_string(18,3,"*"); else if(ver_item==8)	display_6_8_string(53,3,"*"); else if(ver_item==9)	display_6_8_string(88,3,"*");
      if(ver_item==10)	display_6_8_string(18,4,"*"); else if(ver_item==11)	display_6_8_string(53,4,"*"); else if(ver_item==12)	display_6_8_string(88,4,"*");
      if(ver_item==13)	display_6_8_string(18,5,"*"); else if(ver_item==14)	display_6_8_string(53,5,"*"); else if(ver_item==15)	display_6_8_string(88,5,"*");
      if(ver_item==16)	display_6_8_string(18,6,"*"); else if(ver_item==17)	display_6_8_string(53,6,"*"); else if(ver_item==18)	display_6_8_string(88,6,"*");
      if(ver_item==19)	display_6_8_string(18,7,"*"); else if(ver_item==20)	display_6_8_string(53,7,"*"); else if(ver_item==21)	display_6_8_string(88,7,"*");
      
      //通过3D按键来实现换行选中待修改参数
      if(_button.state[UP_3D].press==SHORT_PRESS)
      {
        _button.state[UP_3D].press=NO_PRESS;
        ver_item--;
        if(ver_item<1) ver_item=21;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色				
      }
      if(_button.state[DN_3D].press==SHORT_PRESS)
      {
        _button.state[DN_3D].press=NO_PRESS;
        ver_item++;
        if(ver_item>21) ver_item=1;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色				
      }
      
      //通过左按键短按可以实现选中的参数行自减小1调整
      if(_button.state[LT_3D].press==SHORT_PRESS)
      {
        _button.state[LT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]-=1;
      }
      
      //通过右按键短按可以实现选中的参数行自增加1调整
      if(_button.state[RT_3D].press==SHORT_PRESS)
      {
        _button.state[RT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]+=1;
      }			
      
      //通过左按键短按可以实现选中的参数行自减小50调整
      if(_button.state[LT_3D].press==LONG_PRESS)
      {
        _button.state[LT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]-=50;
      }
      
      //通过右按键短按可以实现选中的参数行自增加50调整
      if(_button.state[RT_3D].press==LONG_PRESS)
      {
        _button.state[RT_3D].press=NO_PRESS;
        param_value[base_item+ver_item-1]+=50;
      }		
      
      //通过中间按键长按可以实现此页设置的所有参数保存
      if(_button.state[ME_3D].press==LONG_PRESS)
      {
        _button.state[ME_3D].press=NO_PRESS;
        //按下后对参数进行保存
        for(uint16_t i=0;i<21;i++)
        {
          WriteFlashParameter(RESERVED_PARAM+base_item+i,param_value[base_item+i],&Flight_Params);
        }
      }	
    }
    break;	
  case 18://姿态控制参数调节
    {
      LCD_clear_L(0,0);		display_6_8_string(0,0,"pit_angle_kp:");  write_6_8_number(90,0,maple_ctrl.pitch_angle_ctrl.kp);	
      LCD_clear_L(0,1);		display_6_8_string(0,1,"pit_gyro_kp :");  write_6_8_number(90,1,maple_ctrl.pitch_gyro_ctrl.kp);
      LCD_clear_L(0,2);		display_6_8_string(0,2,"pit_gyro_ki :");  write_6_8_number(90,2,maple_ctrl.pitch_gyro_ctrl.ki);
      LCD_clear_L(0,3);		display_6_8_string(0,3,"pit_gyro_kd :");  write_6_8_number(90,3,maple_ctrl.pitch_gyro_ctrl.kd);
      LCD_clear_L(0,4);		display_6_8_string(0,4,"yaw_angle_kp:");  write_6_8_number(90,4,maple_ctrl.yaw_angle_ctrl.kp);	
      LCD_clear_L(0,5);		display_6_8_string(0,5,"yaw_gyro_kp :");  write_6_8_number(90,5,maple_ctrl.yaw_gyro_ctrl.kp);
      LCD_clear_L(0,6);		display_6_8_string(0,6,"yaw_gyro_ki :");  write_6_8_number(90,6,maple_ctrl.yaw_gyro_ctrl.ki);
      LCD_clear_L(0,7);		display_6_8_string(0,7,"yaw_gyro_kd :");  write_6_8_number(90,7,maple_ctrl.yaw_gyro_ctrl.kd);
      
      static int16_t ver_item=0;
      display_6_8_string(82,ver_item,"*");
      //通过3D按键来实现换行选中待修改参数
      if(_button.state[UP_3D].press==SHORT_PRESS)
      {
        _button.state[UP_3D].press=NO_PRESS;
        ver_item--;
        if(ver_item<0) ver_item=7;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色				
      } 
      if(_button.state[DN_3D].press==SHORT_PRESS)
      {
        _button.state[DN_3D].press=NO_PRESS;
        ver_item++;
        if(ver_item>7) ver_item=0;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色				
      }
      
      if(_button.state[ME_3D].press==LONG_PRESS)//中间按键长按恢复默认参数
      {
        _button.state[ME_3D].press=NO_PRESS;
        pid_param_flag=3;//将复位PID参数，并写入Flash
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色			
      }	
      
      //通过右按键持续短按可以实现选中的参数行自增加调整
      if(_button.state[RT_3D].press==SHORT_PRESS)
      {
        _button.state[RT_3D].press=NO_PRESS;
        switch(ver_item)
        {
        case 0:
          {
            maple_ctrl.pitch_angle_ctrl.kp+=0.10f;
            maple_ctrl.pitch_angle_ctrl.kp=constrain_float(maple_ctrl.pitch_angle_ctrl.kp,0,100);
            maple_ctrl.roll_angle_ctrl.kp=maple_ctrl.pitch_angle_ctrl.kp;
            
            WriteFlashParameter(PID4_PARAMETER_KP,maple_ctrl.pitch_angle_ctrl.kp,&Flight_Params);
            WriteFlashParameter(PID5_PARAMETER_KP,maple_ctrl.roll_angle_ctrl.kp,&Flight_Params);
          }
          break;
        case 1:
          {
            maple_ctrl.pitch_gyro_ctrl.kp+=0.10f;
            maple_ctrl.pitch_gyro_ctrl.kp=constrain_float(maple_ctrl.pitch_gyro_ctrl.kp,0,100);
            maple_ctrl.roll_gyro_ctrl.kp=maple_ctrl.pitch_gyro_ctrl.kp;
            
            WriteFlashParameter(PID1_PARAMETER_KP,maple_ctrl.pitch_gyro_ctrl.kp,&Flight_Params);
            WriteFlashParameter(PID2_PARAMETER_KP,maple_ctrl.roll_gyro_ctrl.kp,&Flight_Params);
          }
          break;
        case 2:
          {
            maple_ctrl.pitch_gyro_ctrl.ki+=0.10f;
            maple_ctrl.pitch_gyro_ctrl.ki=constrain_float(maple_ctrl.pitch_gyro_ctrl.ki,0,100);
            maple_ctrl.roll_gyro_ctrl.ki=maple_ctrl.pitch_gyro_ctrl.ki;
            
            WriteFlashParameter(PID1_PARAMETER_KI,maple_ctrl.pitch_gyro_ctrl.ki,&Flight_Params);
            WriteFlashParameter(PID2_PARAMETER_KI,maple_ctrl.roll_gyro_ctrl.ki,&Flight_Params);
          }
          break;
        case 3:
          {
            maple_ctrl.pitch_gyro_ctrl.kd+=0.10f;
            maple_ctrl.pitch_gyro_ctrl.kd=constrain_float(maple_ctrl.pitch_gyro_ctrl.kd,0,100);
            maple_ctrl.roll_gyro_ctrl.kd=maple_ctrl.pitch_gyro_ctrl.kd;
            
            WriteFlashParameter(PID1_PARAMETER_KD,maple_ctrl.pitch_gyro_ctrl.kd,&Flight_Params);
            WriteFlashParameter(PID2_PARAMETER_KD,maple_ctrl.roll_gyro_ctrl.kd,&Flight_Params);
          }
          break;
        case 4:
          {
            maple_ctrl.yaw_angle_ctrl.kp+=0.10f;
            maple_ctrl.yaw_angle_ctrl.kp=constrain_float(maple_ctrl.yaw_angle_ctrl.kp,0,100);
            WriteFlashParameter(PID6_PARAMETER_KP,maple_ctrl.yaw_angle_ctrl.kp,&Flight_Params);
          }
          break;
        case 5:
          {
            maple_ctrl.yaw_gyro_ctrl.kp+=0.10f;
            maple_ctrl.yaw_gyro_ctrl.kp=constrain_float(maple_ctrl.yaw_gyro_ctrl.kp,0,100);
            WriteFlashParameter(PID3_PARAMETER_KP,maple_ctrl.yaw_gyro_ctrl.kp,&Flight_Params);
          }
          break;
        case 6:
          {
            maple_ctrl.yaw_gyro_ctrl.ki+=0.10f;
            maple_ctrl.yaw_gyro_ctrl.ki=constrain_float(maple_ctrl.yaw_gyro_ctrl.ki,0,100);
            WriteFlashParameter(PID3_PARAMETER_KI,maple_ctrl.yaw_gyro_ctrl.ki,&Flight_Params);
          }
          break;
        case 7:
          {
            maple_ctrl.yaw_gyro_ctrl.kd+=0.10f;
            maple_ctrl.yaw_gyro_ctrl.kd=constrain_float(maple_ctrl.yaw_gyro_ctrl.kd,0,100);
            WriteFlashParameter(PID3_PARAMETER_KD,maple_ctrl.yaw_gyro_ctrl.kd,&Flight_Params);
          }
          break;
        }	
      }	
      //通过左按键持续短按可以实现选中的参数行自减小调整
      if(_button.state[LT_3D].press==SHORT_PRESS)
      {
        _button.state[LT_3D].press=NO_PRESS;
        switch(ver_item)
        {
        case 0:
          {
            maple_ctrl.pitch_angle_ctrl.kp-=0.10f;
            maple_ctrl.pitch_angle_ctrl.kp=constrain_float(maple_ctrl.pitch_angle_ctrl.kp,0,100);
            maple_ctrl.roll_angle_ctrl.kp=maple_ctrl.pitch_angle_ctrl.kp;
            
            WriteFlashParameter(PID4_PARAMETER_KP,maple_ctrl.pitch_angle_ctrl.kp,&Flight_Params);
            WriteFlashParameter(PID5_PARAMETER_KP,maple_ctrl.roll_angle_ctrl.kp,&Flight_Params);
          }
          break;
        case 1:
          {
            maple_ctrl.pitch_gyro_ctrl.kp-=0.10f;
            maple_ctrl.pitch_gyro_ctrl.kp=constrain_float(maple_ctrl.pitch_gyro_ctrl.kp,0,100);
            maple_ctrl.roll_gyro_ctrl.kp=maple_ctrl.pitch_gyro_ctrl.kp;
            
            WriteFlashParameter(PID1_PARAMETER_KP,maple_ctrl.pitch_gyro_ctrl.kp,&Flight_Params);
            WriteFlashParameter(PID2_PARAMETER_KP,maple_ctrl.roll_gyro_ctrl.kp,&Flight_Params);
          }
          break;
        case 2:
          {
            maple_ctrl.pitch_gyro_ctrl.ki-=0.10f;
            maple_ctrl.pitch_gyro_ctrl.ki=constrain_float(maple_ctrl.pitch_gyro_ctrl.ki,0,100);
            maple_ctrl.roll_gyro_ctrl.ki=maple_ctrl.pitch_gyro_ctrl.ki;
            
            WriteFlashParameter(PID1_PARAMETER_KI,maple_ctrl.pitch_gyro_ctrl.ki,&Flight_Params);
            WriteFlashParameter(PID2_PARAMETER_KI,maple_ctrl.roll_gyro_ctrl.ki,&Flight_Params);
          }
          break;
        case 3:
          {
            maple_ctrl.pitch_gyro_ctrl.kd-=0.10f;
            maple_ctrl.pitch_gyro_ctrl.kd=constrain_float(maple_ctrl.pitch_gyro_ctrl.kd,0,100);
            maple_ctrl.roll_gyro_ctrl.kd=maple_ctrl.pitch_gyro_ctrl.kd;
            
            WriteFlashParameter(PID1_PARAMETER_KD,maple_ctrl.pitch_gyro_ctrl.kd,&Flight_Params);
            WriteFlashParameter(PID2_PARAMETER_KD,maple_ctrl.roll_gyro_ctrl.kd,&Flight_Params);
          }
          break;
        case 4:
          {
            maple_ctrl.yaw_angle_ctrl.kp-=0.10f;
            maple_ctrl.yaw_angle_ctrl.kp=constrain_float(maple_ctrl.yaw_angle_ctrl.kp,0,100);
            WriteFlashParameter(PID6_PARAMETER_KP,maple_ctrl.yaw_angle_ctrl.kp,&Flight_Params);
          }
          break;
        case 5:
          {
            maple_ctrl.yaw_gyro_ctrl.kp-=0.10f;
            maple_ctrl.yaw_gyro_ctrl.kp=constrain_float(maple_ctrl.yaw_gyro_ctrl.kp,0,100);
            WriteFlashParameter(PID3_PARAMETER_KP,maple_ctrl.yaw_gyro_ctrl.kp,&Flight_Params);
          }
          break;
        case 6:
          {
            maple_ctrl.yaw_gyro_ctrl.ki-=0.10f;
            maple_ctrl.yaw_gyro_ctrl.ki=constrain_float(maple_ctrl.yaw_gyro_ctrl.ki,0,100);
            WriteFlashParameter(PID3_PARAMETER_KI,maple_ctrl.yaw_gyro_ctrl.ki,&Flight_Params);
          }
          break;
        case 7:
          {
            maple_ctrl.yaw_gyro_ctrl.kd-=0.10f;
            maple_ctrl.yaw_gyro_ctrl.kd=constrain_float(maple_ctrl.yaw_gyro_ctrl.kd,0,100);
            WriteFlashParameter(PID3_PARAMETER_KD,maple_ctrl.yaw_gyro_ctrl.kd,&Flight_Params);
          }
          break;
        }	
      }			
    }
    break;
  case 19://位置控制参数调节
    {
      LCD_clear_L(0,0);		display_6_8_string(0,0,"opt_pos_kp:");	 write_6_8_number(90,0,maple_ctrl.optical_position_ctrl_x.kp);	
      LCD_clear_L(0,1);		display_6_8_string(0,1,"opt_vel_kp :");  write_6_8_number(90,1,maple_ctrl.optical_speed_ctrl_x.kp);
      LCD_clear_L(0,2);		display_6_8_string(0,2,"opt_vel_ki :");  write_6_8_number(90,2,maple_ctrl.optical_speed_ctrl_x.ki);
      LCD_clear_L(0,3);		display_6_8_string(0,3,"sdk_pos_kp :");  write_6_8_number(90,3,maple_ctrl.sdk_position_ctrl_x.kp);
      LCD_clear_L(0,4);		display_6_8_string(0,4,"sdk_pos_kd :");  write_6_8_number(90,4,maple_ctrl.sdk_position_ctrl_x.kd);
      LCD_clear_L(0,5);		display_6_8_string(0,5,"alt_vel_kp :");	 write_6_8_number(90,5,maple_ctrl.height_speed_ctrl.kp);
      LCD_clear_L(0,6);		display_6_8_string(0,6,"alt_acc_kp :");  write_6_8_number(90,6,maple_ctrl.height_accel_ctrl.kp);
      LCD_clear_L(0,7);         display_6_8_string(0,7,"alt_acc_ki :");  write_6_8_number(90,7,maple_ctrl.height_accel_ctrl.ki);
      static int16_t ver_item=0;
      display_6_8_string(82,ver_item,"*");
      //通过3D按键来实现换行选中待修改参数
      if(_button.state[UP_3D].press==SHORT_PRESS)
      {
        _button.state[UP_3D].press=NO_PRESS;
        ver_item--;
        if(ver_item<0) ver_item=7;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色				
      } 
      if(_button.state[DN_3D].press==SHORT_PRESS)
      {
        _button.state[DN_3D].press=NO_PRESS;
        ver_item++;
        if(ver_item>7) ver_item=0;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色				
      }
      
      if(_button.state[ME_3D].press==LONG_PRESS)//中间按键长按恢复默认参数
      {
        _button.state[ME_3D].press=NO_PRESS;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色
	
        pid_params_init(&maple_ctrl.optical_position_ctrl_x,_optical_position_ctrl_x);
        pid_params_init(&maple_ctrl.optical_speed_ctrl_x,_optical_speed_ctrl_x);
        pid_params_init(&maple_ctrl.optical_position_ctrl_y,_optical_position_ctrl_y);
        pid_params_init(&maple_ctrl.optical_speed_ctrl_y,_optical_speed_ctrl_y);
        pid_params_init(&maple_ctrl.sdk_position_ctrl_x,_sdk_position_ctrl_x);
        pid_params_init(&maple_ctrl.sdk_position_ctrl_y,_sdk_position_ctrl_y);	
        pid_params_init(&maple_ctrl.height_speed_ctrl,_height_speed_ctrl);	
        pid_params_init(&maple_ctrl.height_accel_ctrl,_height_accel_ctrl);	        
        
        
        WriteFlashParameter(PID12_PARAMETER_KP,maple_ctrl.optical_position_ctrl_x.kp,&Flight_Params);
        WriteFlashParameter(PID13_PARAMETER_KP,maple_ctrl.optical_speed_ctrl_x.kp,&Flight_Params);
        WriteFlashParameter(PID13_PARAMETER_KI,maple_ctrl.optical_speed_ctrl_x.ki,&Flight_Params);
        WriteFlashParameter(PID14_PARAMETER_KP,maple_ctrl.sdk_position_ctrl_x.kp,&Flight_Params);
        WriteFlashParameter(PID14_PARAMETER_KD,maple_ctrl.sdk_position_ctrl_x.kd,&Flight_Params);
        
        WriteFlashParameter(PID8_PARAMETER_KP,maple_ctrl.height_speed_ctrl.kp,&Flight_Params);
        WriteFlashParameter(PID9_PARAMETER_KP,maple_ctrl.height_accel_ctrl.kp,&Flight_Params);
        WriteFlashParameter(PID9_PARAMETER_KI,maple_ctrl.height_accel_ctrl.ki,&Flight_Params);	
      }
      
      //通过右按键持续短按可以实现选中的参数行自增加调整
      if(_button.state[RT_3D].press==SHORT_PRESS)
      {
        _button.state[RT_3D].press=NO_PRESS;
        switch(ver_item)
        {
        case 0:
          {
            maple_ctrl.optical_position_ctrl_x.kp+=0.01f;
            maple_ctrl.optical_position_ctrl_x.kp=constrain_float(maple_ctrl.optical_position_ctrl_x.kp,0,100);
            maple_ctrl.optical_position_ctrl_y=maple_ctrl.optical_position_ctrl_x;
            WriteFlashParameter(PID12_PARAMETER_KP,maple_ctrl.optical_position_ctrl_x.kp,&Flight_Params);
          }
          break;
        case 1:
          {
            maple_ctrl.optical_speed_ctrl_x.kp+=0.1f;
            maple_ctrl.optical_speed_ctrl_x.kp=constrain_float(maple_ctrl.optical_speed_ctrl_x.kp,0,100);
            maple_ctrl.optical_speed_ctrl_y.kp=maple_ctrl.optical_speed_ctrl_x.kp;
            WriteFlashParameter(PID13_PARAMETER_KP,maple_ctrl.optical_speed_ctrl_x.kp,&Flight_Params);
          }
          break;
        case 2:
          {
            maple_ctrl.optical_speed_ctrl_x.ki+=0.01f;
            maple_ctrl.optical_speed_ctrl_x.ki=constrain_float(maple_ctrl.optical_speed_ctrl_x.ki,0,100);
            maple_ctrl.optical_speed_ctrl_y.ki=maple_ctrl.optical_speed_ctrl_x.ki;
            WriteFlashParameter(PID13_PARAMETER_KI,maple_ctrl.optical_speed_ctrl_x.ki,&Flight_Params);
          }
          break;
        case 3:
          {
            maple_ctrl.sdk_position_ctrl_x.kp+=0.1f;
            maple_ctrl.sdk_position_ctrl_x.kp=constrain_float(maple_ctrl.sdk_position_ctrl_x.kp,0,100);
            maple_ctrl.sdk_position_ctrl_y.kp=maple_ctrl.sdk_position_ctrl_x.kp;
            
            WriteFlashParameter(PID14_PARAMETER_KP,maple_ctrl.sdk_position_ctrl_x.kp,&Flight_Params);
          }
          break;
        case 4:
          {
            maple_ctrl.sdk_position_ctrl_x.kd+=0.1f;
            maple_ctrl.sdk_position_ctrl_x.kd=constrain_float(maple_ctrl.sdk_position_ctrl_x.kd,0,100);					
            maple_ctrl.sdk_position_ctrl_y.kd=maple_ctrl.sdk_position_ctrl_x.kd;
            
            WriteFlashParameter(PID14_PARAMETER_KD,maple_ctrl.sdk_position_ctrl_x.kd,&Flight_Params);
          }
          break;
        case 5:
          {
            maple_ctrl.height_speed_ctrl.kp+=0.1f;
            maple_ctrl.height_speed_ctrl.kp=constrain_float(maple_ctrl.height_speed_ctrl.kp,0,100);
            WriteFlashParameter(PID8_PARAMETER_KP,maple_ctrl.height_speed_ctrl.kp,&Flight_Params);
          }
          break;
        case 6:
          {
            maple_ctrl.height_accel_ctrl.kp+=0.01f;
            maple_ctrl.height_accel_ctrl.kp=constrain_float(maple_ctrl.height_accel_ctrl.kp,0,100);
            WriteFlashParameter(PID9_PARAMETER_KP,maple_ctrl.height_accel_ctrl.kp,&Flight_Params);
          }
          break;
        case 7:
          {
            maple_ctrl.height_accel_ctrl.ki+=0.1f;
            maple_ctrl.height_accel_ctrl.ki=constrain_float(maple_ctrl.height_accel_ctrl.ki,0,100);
            WriteFlashParameter(PID9_PARAMETER_KI,maple_ctrl.height_accel_ctrl.ki,&Flight_Params);
          }
          break;         
          
        }	
      }	
      //通过左按键持续短按可以实现选中的参数行自减小调整
      if(_button.state[LT_3D].press==SHORT_PRESS)
      {
        _button.state[LT_3D].press=NO_PRESS;
        switch(ver_item)
        {
        case 0:
          {
            maple_ctrl.optical_position_ctrl_x.kp-=0.01f;
            maple_ctrl.optical_position_ctrl_x.kp=constrain_float(maple_ctrl.optical_position_ctrl_x.kp,0,100);
            maple_ctrl.optical_position_ctrl_y=maple_ctrl.optical_position_ctrl_x;
            WriteFlashParameter(PID12_PARAMETER_KP,maple_ctrl.optical_position_ctrl_x.kp,&Flight_Params);
          }
          break;
        case 1:
          {
            maple_ctrl.optical_speed_ctrl_x.kp-=0.1f;
            maple_ctrl.optical_speed_ctrl_x.kp=constrain_float(maple_ctrl.optical_speed_ctrl_x.kp,0,100);
            maple_ctrl.optical_speed_ctrl_y.kp=maple_ctrl.optical_speed_ctrl_x.kp;
            
            WriteFlashParameter(PID13_PARAMETER_KP,maple_ctrl.optical_speed_ctrl_x.kp,&Flight_Params);
          }
          break;
        case 2:
          {
            maple_ctrl.optical_speed_ctrl_x.ki-=0.01f;
            maple_ctrl.optical_speed_ctrl_x.ki=constrain_float(maple_ctrl.optical_speed_ctrl_x.ki,0,100);
            maple_ctrl.optical_speed_ctrl_y.ki=maple_ctrl.optical_speed_ctrl_x.ki;
            
            WriteFlashParameter(PID13_PARAMETER_KI,maple_ctrl.optical_speed_ctrl_x.ki,&Flight_Params);
          }
          break;
        case 3:
          {
            maple_ctrl.sdk_position_ctrl_x.kp-=0.1f;
            maple_ctrl.sdk_position_ctrl_x.kp=constrain_float(maple_ctrl.sdk_position_ctrl_x.kp,0,100);
            maple_ctrl.sdk_position_ctrl_y.kp=maple_ctrl.sdk_position_ctrl_x.kp;
            
            WriteFlashParameter(PID14_PARAMETER_KP,maple_ctrl.sdk_position_ctrl_x.kp,&Flight_Params);
          }
          break;
        case 4:
          {
            maple_ctrl.sdk_position_ctrl_x.kd-=0.1f;
            maple_ctrl.sdk_position_ctrl_x.kd=constrain_float(maple_ctrl.sdk_position_ctrl_x.kd,0,100);					
            maple_ctrl.sdk_position_ctrl_y.kd=maple_ctrl.sdk_position_ctrl_x.kd;
            
            WriteFlashParameter(PID14_PARAMETER_KD,maple_ctrl.sdk_position_ctrl_x.kd,&Flight_Params);
          }
          break;
        case 5:
          {
            maple_ctrl.height_speed_ctrl.kp-=0.1f;
            maple_ctrl.height_speed_ctrl.kp=constrain_float(maple_ctrl.height_speed_ctrl.kp,0,100);
            WriteFlashParameter(PID8_PARAMETER_KP,maple_ctrl.height_speed_ctrl.kp,&Flight_Params);
          }
          break;
        case 6:
          {
            maple_ctrl.height_accel_ctrl.kp-=0.01f;
            maple_ctrl.height_accel_ctrl.kp=constrain_float(maple_ctrl.height_accel_ctrl.kp,0,100);
            WriteFlashParameter(PID9_PARAMETER_KP,maple_ctrl.height_accel_ctrl.kp,&Flight_Params);
          }
          break;
        case 7:
          {
            maple_ctrl.height_accel_ctrl.ki-=0.1f;
            maple_ctrl.height_accel_ctrl.ki=constrain_float(maple_ctrl.height_accel_ctrl.ki,0,100);
            WriteFlashParameter(PID9_PARAMETER_KI,maple_ctrl.height_accel_ctrl.ki,&Flight_Params);
          }
          break;
        }	
      }			
    }
    break;
  case 20:
    {
      LCD_clear_L(0,0); display_6_8_string(0,0,"imu_filter_setup");     write_6_8_number(105,0,page_number+1);
      LCD_clear_L(0,1); display_6_8_string(0,1,"player:");      write_6_8_number(105,1,flymaple.player_level);
      if(flymaple.player_level==0)      display_6_8_string(40,1,"general");  //一般玩家
      else if(flymaple.player_level==1) display_6_8_string(40,1,"expert");   //专业玩家
      else      display_6_8_string(40,1,"hardcore"); //硬核玩家
      
      LCD_clear_L(0,2); display_6_8_string(0,2,"ahrs_gyro_lpf:");       write_6_8_number(105,2,gyro_lpf_param.cf);
      LCD_clear_L(0,3); display_6_8_string(0,3,"ahrs_accel_lpf:");      write_6_8_number(105,3,accel_lpf_param.cf);
      LCD_clear_L(0,4); display_6_8_string(0,4,"ins_accel_lpf:");       write_6_8_number(105,4,ins_lpf_param.cf);
      LCD_clear_L(0,5); display_6_8_string(0,5,"fb_accel_lpf:");        write_6_8_number(105,5,accel_fb_lpf_param.cf);
      LCD_clear_L(0,6); display_6_8_string(0,6,"imu_gyro_lpfi:");       write_6_8_number(95,6,flymaple.icm_gyro_inner_lpf_config);
                                                                        write_6_8_number(105,6,icmg_inner_lpf_config.name[flymaple.icm_gyro_inner_lpf_config]);
      LCD_clear_L(0,7); display_6_8_string(0,7,"imu_accel_lpfi:");      write_6_8_number(95,7,flymaple.icm_accel_inner_lpf_config);
                                                                        write_6_8_number(105,7,icma_inner_lpf_config.name[flymaple.icm_accel_inner_lpf_config]);
      
      static uint16_t ver_item=1;
      display_6_8_string(88,ver_item,"*");
      //通过3D按键来实现换行选中待修改参数
      if(_button.state[UP_3D].press==SHORT_PRESS)
      {
        _button.state[UP_3D].press=NO_PRESS;
        ver_item--;
        if(ver_item<1) ver_item=7;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色	
      } 
      if(_button.state[DN_3D].press==SHORT_PRESS)
      {
        _button.state[DN_3D].press=NO_PRESS;
        ver_item++;
        if(ver_item>7) ver_item=1;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色		
      }
      
      //通过3D按键中按键长按恢复默认参数
      if(_button.state[ME_3D].press==LONG_PRESS)
      {
        _button.state[ME_3D].press=NO_PRESS;
        if(flymaple.player_level==0)//通用机型
        {
          gyro_lpf_param.cf=gyro_lpf_param_default1;
          accel_lpf_param.cf=accel_lpf_param_default1;
          ins_lpf_param.cf=ins_lpf_param_default1;
          accel_fb_lpf_param.cf=accel_fb_lpf_param_default1;
        }
        else if(flymaple.player_level==1)
        {
          gyro_lpf_param.cf=gyro_lpf_param_default2;
          accel_lpf_param.cf=accel_lpf_param_default2;
          ins_lpf_param.cf=ins_lpf_param_default2;
          accel_fb_lpf_param.cf=accel_fb_lpf_param_default2;
        }
        else
        {
          gyro_lpf_param.cf=gyro_lpf_param_default1;
          accel_lpf_param.cf=accel_lpf_param_default1;
          ins_lpf_param.cf=ins_lpf_param_default1;
          accel_fb_lpf_param.cf=accel_fb_lpf_param_default1;					
        }
        
        flymaple.icm_gyro_inner_lpf_config=icm_gyro_inner_lpf_config_default;//92hz	
        flymaple.icm_accel_inner_lpf_config=icm_accel_inner_lpf_config_default;//44.8hz         
        
        WriteFlashParameter(GYRO_LPF_CF ,gyro_lpf_param.cf,&Flight_Params);
        WriteFlashParameter(ACCEL_LPF_CF,accel_lpf_param.cf,&Flight_Params);
        WriteFlashParameter(INS_LPF_CF  ,ins_lpf_param.cf,&Flight_Params);
        WriteFlashParameter(FB_LPF_CF   ,accel_fb_lpf_param.cf,&Flight_Params);	
             
        WriteFlashParameter(IMU_GYRO_INNER_LPF,flymaple.icm_gyro_inner_lpf_config,&Flight_Params);
        WriteFlashParameter(IMU_ACCEL_INNER_LPF,flymaple.icm_accel_inner_lpf_config,&Flight_Params);   
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色
      }
      
      //通过右按键持续短按可以实现选中的参数行自增加调整
      if(_button.state[RT_3D].press==SHORT_PRESS)
      {
        _button.state[RT_3D].press=NO_PRESS;
        switch(ver_item)
        {
        case 1:
          {
            flymaple.player_level++;
            if(flymaple.player_level>4)	flymaple.player_level=0;
            WriteFlashParameter(DRONE_PLAYER_LEVEL,flymaple.player_level,&Flight_Params);
          }
          break;
        case 2:
          {
            gyro_lpf_param.cf+=1;
            if(gyro_lpf_param.cf>100)	gyro_lpf_param.cf=100;
            WriteFlashParameter(GYRO_LPF_CF ,gyro_lpf_param.cf,&Flight_Params);
          }
          break;
        case 3:
          {
            accel_lpf_param.cf+=1;
            if(accel_lpf_param.cf>100)	accel_lpf_param.cf=100;
            WriteFlashParameter(ACCEL_LPF_CF,accel_lpf_param.cf,&Flight_Params);
          }
          break;
        case 4:
          {
            ins_lpf_param.cf+=1;
            if(ins_lpf_param.cf>100)	ins_lpf_param.cf=100;
            WriteFlashParameter(INS_LPF_CF  ,ins_lpf_param.cf,&Flight_Params);
          }
          break;
        case 5:
          {
            accel_fb_lpf_param.cf+=1;
            if(accel_fb_lpf_param.cf>100)	accel_fb_lpf_param.cf=100;
            WriteFlashParameter(FB_LPF_CF   ,accel_fb_lpf_param.cf,&Flight_Params);
          }
          break;
        case 6:
          {
            flymaple.icm_gyro_inner_lpf_config+=1;
            if(flymaple.icm_gyro_inner_lpf_config>7)    flymaple.icm_gyro_inner_lpf_config=0;
            WriteFlashParameter(IMU_GYRO_INNER_LPF,flymaple.icm_gyro_inner_lpf_config,&Flight_Params);
          }
          break;
        case 7:
          {
            flymaple.icm_accel_inner_lpf_config+=1;
            if(flymaple.icm_accel_inner_lpf_config>7)    flymaple.icm_accel_inner_lpf_config=0;
            WriteFlashParameter(IMU_ACCEL_INNER_LPF,flymaple.icm_accel_inner_lpf_config,&Flight_Params);
          }
          break;					
        }	
      }			
      //通过左按键持续短按可以实现选中的参数行自减小调整
      if(_button.state[LT_3D].press==SHORT_PRESS)
      {
        _button.state[LT_3D].press=NO_PRESS;
        switch(ver_item)
        {
        case 1:
          {
            flymaple.player_level--;
            if(flymaple.player_level<0)	flymaple.player_level=4;
            WriteFlashParameter(DRONE_PLAYER_LEVEL,flymaple.player_level,&Flight_Params);
          }
          break;
        case 2:
          {
            gyro_lpf_param.cf-=1;
            if(gyro_lpf_param.cf<0)	gyro_lpf_param.cf=0;
            WriteFlashParameter(GYRO_LPF_CF ,gyro_lpf_param.cf,&Flight_Params);
          }
          break;
        case 3:
          {
            accel_lpf_param.cf-=1;
            if(accel_lpf_param.cf<0)	accel_lpf_param.cf=0;
            WriteFlashParameter(ACCEL_LPF_CF,accel_lpf_param.cf,&Flight_Params);
          }
          break;
        case 4:
          {
            ins_lpf_param.cf-=1;
            if(ins_lpf_param.cf<0)	ins_lpf_param.cf=0;
            WriteFlashParameter(INS_LPF_CF  ,ins_lpf_param.cf,&Flight_Params);
          }
          break;
        case 5:
          {
            accel_fb_lpf_param.cf-=1;
            if(accel_fb_lpf_param.cf<0)	accel_fb_lpf_param.cf=0;
            WriteFlashParameter(FB_LPF_CF   ,accel_fb_lpf_param.cf,&Flight_Params);
          }
          break;
        case 6:
          {
            flymaple.icm_gyro_inner_lpf_config-=1;
            if(flymaple.icm_gyro_inner_lpf_config<0)    flymaple.icm_gyro_inner_lpf_config=7;
            WriteFlashParameter(IMU_GYRO_INNER_LPF,flymaple.icm_gyro_inner_lpf_config,&Flight_Params);
          }
          break;
        case 7:
          {
            flymaple.icm_accel_inner_lpf_config-=1;
            if(flymaple.icm_accel_inner_lpf_config<0)    flymaple.icm_accel_inner_lpf_config=7;
            WriteFlashParameter(IMU_ACCEL_INNER_LPF,flymaple.icm_accel_inner_lpf_config,&Flight_Params);
          }
          break;
        }	
      }			
    }
    break;
  case 21:
    {
      LCD_clear_L(0,0);  display_6_8_string(0,0,"params_setup");  		 
      display_6_8_number(115,0,page_number+1);
      LCD_clear_L(0,1);  display_6_8_string(0,1,"indoor_sen:");     
      display_6_8_number(75,1,maplepilot.indoor_position_sensor);
      switch(maplepilot.indoor_position_sensor)
      {
      case 1: display_6_8_string(90,1,"OPTI");  break;
      case 2: display_6_8_string(90,1,"SLAM"); break;
      default:display_6_8_string(90,1,"unkn");		
      }
      
      LCD_clear_L(0,2);	 display_6_8_string(0,2,"range_sen:");      
      display_6_8_number(75,2,maplepilot.rangefinder_sensor);
      switch(maplepilot.rangefinder_sensor)
      {
      case TOFSENSE: display_6_8_string(90,2,"TOFS");  break;
      case VL53L8:    display_6_8_string(90,2,"V53L8"); break;
      case UP_Tx:    display_6_8_string(90,2,"UP_Tx");   break;
      case MT1:      display_6_8_string(90,2,"MT1");   break;
      case GYTOF10M: display_6_8_string(90,2,"GY10");  break;
      case TFMINI:   display_6_8_string(90,2,"TFM");   break;
      default:       display_6_8_string(90,2,"unkn");		
      }
      
      
      
      LCD_clear_L(0,3);  display_6_8_string(0,3,"uart2_mode:");  
      display_6_8_number(75,3,other_params.params.reserved_uart);
      switch(other_params.params.reserved_uart)
      {
      case GPS_M8N: 					display_6_8_string(90,3,"GPS");  break;
      case THIRD_PARTY_STATE: display_6_8_string(90,3,"ROS");   break;
      default:       display_6_8_string(90,3,"unkn");		
      }				
      
      LCD_clear_L(0,4);  display_6_8_string(0,4,"uart5_mode:");  
      display_6_8_number(75,4,other_params.params.inner_uart);
      switch(other_params.params.inner_uart)
      {
      case 0:
      case 1: display_6_8_string(90,4,"VISI");   break;
      case 2: display_6_8_string(90,4,"OFFB");   break;
      default:display_6_8_string(90,4,"unkn");		
      }			
      
      LCD_clear_L(0,5);  display_6_8_string(0,5,"opt_type:"); 
      display_6_8_number(75,5,other_params.params.opticalcal_type);	
      switch(other_params.params.opticalcal_type)
      {
      case 0: display_6_8_string(90,5,"T101p");  break;
      case 1: display_6_8_string(90,5,"T201");   break;
      case 2: display_6_8_string(90,5,"lc307");  break;
      default:display_6_8_string(90,5,"unkn");		
      }	
      
      LCD_clear_L(0,6);  display_6_8_string(0,6,"imu_type:");  
      if(imu_sensor_type==0) display_6_8_string(90,6,"icm");
      else if(imu_sensor_type==1) display_6_8_string(90,6,"bmi:");
      else display_6_8_string(90,6,"I426");	      
      
      static uint8_t factory_reset_flag=0;
      LCD_clear_L(0,7);		
      if(factory_reset_flag==0) display_6_8_string(0,7,"factory:");				  
      else                                               
      {
        static uint8_t _cnt=0;_cnt++;
        if(_cnt<10) display_6_8_string(0,7,"reset_ok");
        else if(_cnt<20)
        {
          display_6_8_string(0,7,"reboot");
        }
        else _cnt=0;
      }
      write_6_8_number(90,7,factory_reset_flag);	
      
#define ver_up 	(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f \
      &&rc_data.rc_rpyt[RC_PITCH]>Pit_Rol_Max*Scale_Pecent_Max \
        &&rc_data.rc_rpyt[RC_ROLL]==0 \
          &&rc_data.rc_rpyt[RC_YAW]==0)
#define ver_down 	(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f \
&&rc_data.rc_rpyt[RC_PITCH]<-Pit_Rol_Max*Scale_Pecent_Max \
  &&rc_data.rc_rpyt[RC_ROLL]==0 \
    &&rc_data.rc_rpyt[RC_YAW]==0)

display_6_8_string(65,ver_choose,"*");
if(ver_up)
{
  delay_ms(10);
  if(ver_up)
  {
    ver_choose++;
    if(ver_choose>7) ver_choose=1;
    while(ver_up);
    switch_choose_beep();				
  }
}
if(ver_down)
{
  delay_ms(10);
  if(ver_down)
  {
    ver_choose--;
    if(ver_choose<1) ver_choose=7;
    while(ver_down);	
    switch_choose_beep();					
  }
}	
#undef ver_up
#undef ver_down

#define ver_left 	(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f \
&&rc_data.rc_rpyt[RC_ROLL]<-Pit_Rol_Max*Scale_Pecent_Max \
  &&rc_data.rc_rpyt[RC_PITCH]==0 \
    &&rc_data.rc_rpyt[RC_YAW]==0)
#define ver_right 	(rc_data.thr<rc_data.cal[RC_THR_CHANNEL].min_value+rc_data.cal[RC_THR_CHANNEL].scale*0.1f \
&&rc_data.rc_rpyt[RC_ROLL]>Pit_Rol_Max*Scale_Pecent_Max \
  &&rc_data.rc_rpyt[RC_PITCH]==0 \
    &&rc_data.rc_rpyt[RC_YAW]==0)
if(ver_left)//参数自减
{
  delay_ms(10);
  if(ver_left)
  {
    switch(ver_choose)
    {
    case 1:
      {
        maplepilot.indoor_position_sensor--;
        if(maplepilot.indoor_position_sensor<1)  maplepilot.indoor_position_sensor=1;
        WriteFlashParameter(OPTICAL_TYPE,maplepilot.indoor_position_sensor,&Flight_Params);							
      }break;
    case 2:
      {
        maplepilot.rangefinder_sensor--;
        if(maplepilot.rangefinder_sensor<1)  maplepilot.rangefinder_sensor=1;
        WriteFlashParameter(GROUND_DISTANCE_DEFAULT,maplepilot.rangefinder_sensor,&Flight_Params);							
      }break;
    case 3:
      {
        other_params.params.reserved_uart--;
        if(other_params.params.reserved_uart<1)  other_params.params.reserved_uart=1;
        WriteFlashParameter(RESERVED_UART_FUNCTION,other_params.params.reserved_uart,&Flight_Params);							
      }break;
    case 4:
      {
        other_params.params.inner_uart--;
        if(other_params.params.inner_uart<1)  other_params.params.inner_uart=1;
        WriteFlashParameter(UART5_FUNCTION,other_params.params.inner_uart,&Flight_Params);							
      }break;
    case 5:
      {
        if(other_params.params.opticalcal_type>0) other_params.params.opticalcal_type--;
        WriteFlashParameter(OPTICALFLOW_TYPE,other_params.params.opticalcal_type,&Flight_Params);							
      }break;
    case 6:
      {
        imu_sensor_type--;
        if(imu_sensor_type<0) imu_sensor_type=1;
        WriteFlashParameter(IMU_SENSE_TYPE,imu_sensor_type,&Flight_Params);
      }break;
    case 7:
      {
        factory_reset_flag=1;
        Resume_Factory_Setting();
        nclink_send_check_flag[11]=1;
      }break;	     
    default:{}
    }
    while(ver_left);
    switch_choose_beep();						
  }
}

if(ver_right)//参数自加
{
  delay_ms(10);
  if(ver_right)
  {
    switch(ver_choose)
    {
    case 1:
      {
        maplepilot.indoor_position_sensor++;
        if(maplepilot.indoor_position_sensor>=2)  maplepilot.indoor_position_sensor=2;
        WriteFlashParameter(OPTICAL_TYPE,maplepilot.indoor_position_sensor,&Flight_Params);
      }break;
    case 2:
      {
        maplepilot.rangefinder_sensor++;
        if(maplepilot.rangefinder_sensor>=3)  maplepilot.rangefinder_sensor=3;
        WriteFlashParameter(GROUND_DISTANCE_DEFAULT,maplepilot.rangefinder_sensor,&Flight_Params);							
      }break;
    case 3:
      {
        other_params.params.reserved_uart++;
        if(other_params.params.reserved_uart>2)  other_params.params.reserved_uart=2;
        WriteFlashParameter(RESERVED_UART_FUNCTION,other_params.params.reserved_uart,&Flight_Params);							
      }break;
    case 4:
      {
        other_params.params.inner_uart++;
        if(other_params.params.inner_uart>2)  other_params.params.inner_uart=2;
        WriteFlashParameter(UART5_FUNCTION,other_params.params.inner_uart,&Flight_Params);							
      }break;
    case 5:
      {
        other_params.params.opticalcal_type++;
        if(other_params.params.opticalcal_type>5)  other_params.params.opticalcal_type=5;
        WriteFlashParameter(OPTICALFLOW_TYPE,other_params.params.opticalcal_type,&Flight_Params);							
      }break;
    case 6:
      {
        imu_sensor_type++;
        if(imu_sensor_type>1) imu_sensor_type=0;
        WriteFlashParameter(IMU_SENSE_TYPE,imu_sensor_type,&Flight_Params);
      }break;
    case 7:
      {
        factory_reset_flag=1;
        Resume_Factory_Setting();
        nclink_send_check_flag[11]=1;
      }break;	 
    default:{}
    }
    while(ver_right);
    switch_choose_beep();						
  }
}	
#undef ver_left
#undef ver_right		
    }
    break;
  case 22:
    {
      LCD_clear_L(0,0); display_6_8_string(0,0,"params_setup");         write_6_8_number(105,0,page_number+1);
      LCD_clear_L(0,1); display_6_8_string(0,1,"esc_output_hz:");       write_6_8_number(105,1,50*maplepilot.esc_output_frequence);
      LCD_clear_L(0,2);
      LCD_clear_L(0,3);
      LCD_clear_L(0,4);
      LCD_clear_L(0,5);
      LCD_clear_L(0,6);
      LCD_clear_L(0,7);
      
      static uint16_t ver_item=1;
      display_6_8_string(92,ver_item,"*");
      //通过3D按键来实现换行选中待修改参数
      if(_button.state[UP_3D].press==SHORT_PRESS)
      {
        _button.state[UP_3D].press=NO_PRESS;
        ver_item--;
        if(ver_item<1) ver_item=6;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色	
      } 
      if(_button.state[DN_3D].press==SHORT_PRESS)
      {
        _button.state[DN_3D].press=NO_PRESS;
        ver_item++;
        if(ver_item>6) ver_item=1;
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色		
      }
      
      //通过3D按键中按键长按恢复默认参数
      if(_button.state[ME_3D].press==LONG_PRESS)
      {
        _button.state[ME_3D].press=NO_PRESS;
        
        rgb_notify_set(BLUE	,TOGGLE	 ,50,500,0);//蓝色
      }
      
      //通过右按键持续短按可以实现选中的参数行自增加调整
      if(_button.state[RT_3D].press==SHORT_PRESS)
      {
        _button.state[RT_3D].press=NO_PRESS;
        switch(ver_item)
        {
        case 1:
          {
            maplepilot.esc_output_frequence*=2;
            if(maplepilot.esc_output_frequence>8)	maplepilot.esc_output_frequence=1;
            WriteFlashParameter(ESC_OUTPUT_FREQUENCY   ,maplepilot.esc_output_frequence,&Flight_Params);
          }
          break;					
        }	
      }			
      //通过左按键持续短按可以实现选中的参数行自减小调整
      if(_button.state[LT_3D].press==SHORT_PRESS)
      {
        _button.state[LT_3D].press=NO_PRESS;
        switch(ver_item)
        {
        case 1:
          {
            maplepilot.esc_output_frequence/=2;
            if(maplepilot.esc_output_frequence<1)	maplepilot.esc_output_frequence=8;
            WriteFlashParameter(ESC_OUTPUT_FREQUENCY   ,maplepilot.esc_output_frequence,&Flight_Params);
          }
          break;
        }	
      }			
    }
    break;
  default:
    {			
      LCD_clear_L(0,0);display_6_8_string(0,0,"page_num");  display_6_8_number(115,0,page_number+1);
    }
  }	
}





//			write_6_8_number(15,0,Flight_Params.parameter_table[LOG_NUM]);
//			write_6_8_number(45,0,get_systick_ms()/1000.0f);
//			write_6_8_number(90,0,log_bytes_len/1024.0f);











