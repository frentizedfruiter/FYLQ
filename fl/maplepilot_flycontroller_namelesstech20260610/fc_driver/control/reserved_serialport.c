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
#include "drv_uart.h"
#include "schedule.h"
#include "quene.h"
#include "parameter_server.h"
#include "maple_config.h"
#include "nclink.h"
#include "drv_gps.h"
#include "drv_expand.h"
#include "reserved_serialport.h"

float min_dis_cm,min_dis_angle,target_yaw_err;
float min_dis_cm_backups[Laser_Min_Info_Num],min_dis_angle_backups[Laser_Min_Info_Num];


void quad_getangle(float *q,float* rpy)
{
  rpy[1]= RADTODEG(FastAtan2(q[0]*q[1] + q[2]*q[3], 0.5f - q[1]*q[1] - q[2]*q[2]));
  rpy[0]= RADTODEG(FastAsin(-2.0f * (q[1]*q[3] - q[0]*q[2])));
  rpy[2]= RADTODEG(FastAtan2(q[1]*q[2] + q[0]*q[3], 0.5f - q[2]*q[2] - q[3]*q[3]));
}


void reserved_serialport_init(void)
{
  float tmp_param=0;
  ReadFlashParameterOne(RESERVED_UART_FUNCTION,&tmp_param);
  if(isnan(tmp_param)==0) other_params.params.reserved_uart=tmp_param;
  else other_params.params.reserved_uart=RESERVED_UART_DEFAULT;
  
  switch(other_params.params.reserved_uart)
  {
  case GPS_M8N:
    gps_uart_init();
    break;
  case THIRD_PARTY_STATE:
    uart2_init(460800);//460800	
    break;
  default:gps_uart_init();
  }
}

static uint8_t NCLink_Head[2]={0xFF,0xFC};//数据帧头
static uint8_t NCLink_End[2] ={0xA1,0xA2};//数据帧尾
static uint8_t nclink_buf[100];//待接收数据缓冲区
static uint8_t send_nclink_buf[100];//待接收数据缓冲区
uint8_t send_check_back=0;
void Reserved_Serialport_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  uart_sendbytes(2,pui8Buffer,ui32Count);
}

void NCLink_Send_IMU_Feedback(float q1	,float q2,float q3  ,float q0,
                              float gx,float gy,float gz,
                              float ax,float ay,float az,
                              float px,float py,float pz,
                              float vx,float vy,float vz)
{
  uint8_t sum=0,_cnt=0,i=0;
  send_nclink_buf[_cnt++]=NCLink_Head[0];
  send_nclink_buf[_cnt++]=NCLink_Head[1];
  send_nclink_buf[_cnt++]=0xE0;
  send_nclink_buf[_cnt++]=0;
  
  Float2Byte(&q1,send_nclink_buf,_cnt);//4
  _cnt+=4;
  Float2Byte(&q2,send_nclink_buf,_cnt);//8
  _cnt+=4;
  Float2Byte(&q3,send_nclink_buf,_cnt);//12
  _cnt+=4;
  Float2Byte(&q0,send_nclink_buf,_cnt);//16
  _cnt+=4;
  Float2Byte(&gx,send_nclink_buf,_cnt);//20
  _cnt+=4;
  Float2Byte(&gy,send_nclink_buf,_cnt);//24
  _cnt+=4;
  Float2Byte(&gz,send_nclink_buf,_cnt);//28
  _cnt+=4;
  Float2Byte(&ax,send_nclink_buf,_cnt);//32
  _cnt+=4;
  Float2Byte(&ay,send_nclink_buf,_cnt);//36
  _cnt+=4;
  Float2Byte(&az,send_nclink_buf,_cnt);//40
  _cnt+=4;
  
  Float2Byte(&px,send_nclink_buf,_cnt);//44
  _cnt+=4;
  Float2Byte(&py,send_nclink_buf,_cnt);//48
  _cnt+=4;
  Float2Byte(&pz,send_nclink_buf,_cnt);//52
  _cnt+=4;
  Float2Byte(&vx,send_nclink_buf,_cnt);//56
  _cnt+=4;
  Float2Byte(&vy,send_nclink_buf,_cnt);//60
  _cnt+=4;
  Float2Byte(&vz,send_nclink_buf,_cnt);//64
  _cnt+=4;
  
  send_nclink_buf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= send_nclink_buf[i]; 
  send_nclink_buf[_cnt++]=sum;
  
  send_nclink_buf[_cnt++]=NCLink_End[0];
  send_nclink_buf[_cnt++]=NCLink_End[1];
  Reserved_Serialport_Send(send_nclink_buf,_cnt);
}

void NCLink_Send_Quaternion_Feedback(float q1	,float q2,float q3  ,float q0,
                                     float gx,float gy,float gz,
                                     float ax,float ay,float az)
{
  uint8_t sum=0,_cnt=0,i=0;
  send_nclink_buf[_cnt++]=NCLink_Head[0];
  send_nclink_buf[_cnt++]=NCLink_Head[1];
  send_nclink_buf[_cnt++]=0xE1;
  send_nclink_buf[_cnt++]=0;
  
  Float2Byte(&q1,send_nclink_buf,_cnt);//4
  _cnt+=4;
  Float2Byte(&q2,send_nclink_buf,_cnt);//8
  _cnt+=4;
  Float2Byte(&q3,send_nclink_buf,_cnt);//12
  _cnt+=4;
  Float2Byte(&q0,send_nclink_buf,_cnt);//16
  _cnt+=4;
  Float2Byte(&gx,send_nclink_buf,_cnt);//20
  _cnt+=4;
  Float2Byte(&gy,send_nclink_buf,_cnt);//24
  _cnt+=4;
  Float2Byte(&gz,send_nclink_buf,_cnt);//28
  _cnt+=4;
  Float2Byte(&ax,send_nclink_buf,_cnt);//32
  _cnt+=4;
  Float2Byte(&ay,send_nclink_buf,_cnt);//36
  _cnt+=4;
  Float2Byte(&az,send_nclink_buf,_cnt);//40
  _cnt+=4;
  
  send_nclink_buf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= send_nclink_buf[i]; 
  send_nclink_buf[_cnt++]=sum;
  
  send_nclink_buf[_cnt++]=NCLink_End[0];
  send_nclink_buf[_cnt++]=NCLink_End[1];
  Reserved_Serialport_Send(send_nclink_buf,_cnt);
}




static void NCLink_Send_Feedback(uint8_t response)//地面站应答校验
{
  uint8_t sum = 0,i=0;
  send_nclink_buf[0]=NCLink_Head[0];
  send_nclink_buf[1]=NCLink_Head[1];
  send_nclink_buf[2]=NCLINK_SEND_CHECK;
  send_nclink_buf[3]=1;
  send_nclink_buf[4]=response;
  for(i=0;i<5;i++) sum ^= send_nclink_buf[i];
  send_nclink_buf[5]=sum;
  send_nclink_buf[6]=NCLink_End[0];
  send_nclink_buf[7]=NCLink_End[1];
  Reserved_Serialport_Send(send_nclink_buf,8);
}

void NCLink_Send_IMU_Feedback_PC(void)//1.54ms
{
  static uint16_t _cnt=0;
  _cnt++;
  if(_cnt>=10)//50ms————20hz
  {
    _cnt=0;
    //温度稳定且陀螺仪零偏校准完毕才发送飞机IMU数据给板载计算机
    //避免激光雷达转动影响陀螺仪标定过程
    if(flymaple.imu_gyro_calibration_flag==1
       &&other_params.params.reserved_uart==THIRD_PARTY_STATE)//设置的定位类型为RPLIDAR/T265
    {			
      NCLink_Send_IMU_Feedback(flymaple.quaternion[1],	\
        flymaple.quaternion[2],	\
          flymaple.quaternion[3],	\
            flymaple.quaternion[0],	\
              flymaple.gyro_rps.x,	\
                flymaple.gyro_rps.y,	\
                  flymaple.gyro_rps.z,	\
                    flymaple.accel_g.x*GRAVITY_MSS,	\
                      flymaple.accel_g.y*GRAVITY_MSS,	\
                        flymaple.accel_g.z*GRAVITY_MSS,	\
                          ins.opt.position.x/100.0f,	\
                            ins.opt.position.y/100.0f,	\
                              ins.position_z/100.0f,	\
                                ins.opt.speed.x/100.0f ,	\
                                  ins.opt.speed.y/100.0f,	\
                                    ins.speed_z/100.0f);
    }
  }
  
  switch(send_check_back)
  {
  case 1:
    {
      NCLink_Send_Feedback(NCLINK_SEND_DIS);
      send_check_back=0;
    }
    break;
  case 2:
    {
      NCLink_Send_Feedback(NCLINK_SEND_NAV_CTRL);
      send_check_back=0;
    }
    break;	
  case 3:
    {
      NCLink_Send_Feedback(NCLINK_SEND_NAV_CTRL_FINISH);
      send_check_back=0;		
    }
    break;
  case 4:
    {
      NCLink_Send_Feedback(NCLINK_SEND_SLAM_SYSTEM_RESET);//复位slam
      send_check_back=0;
      //蜂鸣器提示复位成功
      beep.period=200;//200*5ms
      beep.light_on_percent=0.5f;
      beep.reset=1;	
      beep.times=2;//闪烁2次			
    }
    break;
  case 5:
    {
      NCLink_Send_Feedback(NCLINK_SEND_SLAM_STOP_MOTOR);
      send_check_back=0;		
    }
    break;
  case 6:
    {
      NCLink_Send_Feedback(NCLINK_SEND_SLAM_START_MOTOR);
      send_check_back=0;		
    }
    break;
  }
}

uint32_t nclink_receive_fault_cnt[2];
float nclink_receive_fault_rate=0;
void NCLink_Data_Prase_Prepare_Lite(uint8_t data)//地面站数据解析
{
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==NCLink_Head[1])//判断帧头1
  {
    state=1;
    nclink_buf[0]=data;
  }
  else if(state==1&&data==NCLink_Head[0])//判断帧头2
  {
    state=2;
    nclink_buf[1]=data;
  }
  else if(state==2&&data<0XF1)//功能字节
  {
    state=3;
    nclink_buf[2]=data;
  }
  else if(state==3&&data<100)//有效数据长度
  {
    state = 4;
    nclink_buf[3]=data;
    data_len = data;
    data_cnt = 0;
  }
  else if(state==4&&data_len>0)//数据接收
  {
    data_len--;
    nclink_buf[4+data_cnt++]=data;
    if(data_len==0)  state = 5;
  }
  else if(state==5)//异或校验
  {
    state = 6;
    nclink_buf[4+data_cnt++]=data;
  }
  else if(state==6&&data==NCLink_End[0])//帧尾0
  {
    state = 7;
    nclink_buf[4+data_cnt++]=data;
  }
  else if(state==7&&data==NCLink_End[1])//帧尾1
  {
    state = 0;
    nclink_buf[4+data_cnt]=data;
    NCLink_Data_Prase_Process_Lite(nclink_buf,data_cnt+5);//数据解析
    nclink_receive_fault_cnt[0]+=(data_cnt+5);
    nclink_receive_fault_rate=(float)(nclink_receive_fault_cnt[0])/(nclink_receive_fault_cnt[0]+nclink_receive_fault_cnt[1]);
  }
  else 
  {
    state = 0;
    nclink_receive_fault_cnt[1]++;
  }
}




systime nclink_t;
void NCLink_Data_Prase_Process_Lite(uint8_t *data_buf,uint8_t num)//飞控数据解析进程
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<(num-3);i++)  sum ^= *(data_buf+i);
  if(!(sum==*(data_buf+num-3)))    																					return;//判断sum	
  if(!(*(data_buf)==NCLink_Head[1]&&*(data_buf+1)==NCLink_Head[0]))         return;//判断帧头
  if(!(*(data_buf+num-2)==NCLink_End[0]&&*(data_buf+num-1)==NCLink_End[1])) return;//帧尾校验  
  
  if(*(data_buf+2)==0X0A)                             //地面站控制移动数据
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
    send_check_back=1;
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
    Byte2Float(data_buf,16,&current_state.velocity_x);
    Byte2Float(data_buf,20,&current_state.velocity_y);
    Byte2Float(data_buf,24,&current_state.velocity_z);
    current_state.position_x*=100.0f;
    current_state.position_y*=100.0f;
    current_state.position_z*=100.0f;
    current_state.velocity_x*=100.0f;
    current_state.velocity_y*=100.0f;
    current_state.velocity_z*=100.0f;
    
    
    Byte2Float(data_buf,28,&current_state.q[0]);		
    Byte2Float(data_buf,32,&current_state.q[1]);
    Byte2Float(data_buf,36,&current_state.q[2]);
    Byte2Float(data_buf,40,&current_state.q[3]);
    float _q[4];
    _q[0]=current_state.q[3]*(1.0f);
    _q[1]=current_state.q[0]*(1.0f);
    _q[2]=current_state.q[2]*(-1.0f);
    _q[3]=current_state.q[1]*(1.0f);	
    
    current_state.q[0]=_q[0];
    current_state.q[1]=_q[1];
    current_state.q[2]=_q[2];
    current_state.q[3]=_q[3];
    
    Byte2Float(data_buf,44,&current_state.quality);
    current_state.update_flag=*(data_buf+48);
    current_state.byte[0]=*(data_buf+49);
    current_state.byte[1]=*(data_buf+50);
    current_state.byte[2]=*(data_buf+51);
    current_state.byte[3]=*(data_buf+52);
    current_state.byte[4]=*(data_buf+53);
    current_state.byte[5]=*(data_buf+54);
    current_state.byte[6]=*(data_buf+55);
    current_state.byte[7]=*(data_buf+56);
    
    current_state.rec_update_flag=1;
    current_state.rec_head_update_flag=1;		
    Pilot_Status_Tick();
    get_systime(&nclink_t);
    current_state.slam_sensor=T265_SLAM;
  }
  else if(*(data_buf+2)==0X0E)
  {
    Byte2Float(data_buf,4,&current_state.position_x);
    Byte2Float(data_buf,8,&current_state.position_y);
    Byte2Float(data_buf,12,&current_state.position_z);
    //当为激光雷达时,下面三个数据分别表示最小距离、最小距离所在角度，信号强度
    Byte2Float(data_buf,16,&current_state.velocity_x);//m
    Byte2Float(data_buf,20,&current_state.velocity_y);//deg
    Byte2Float(data_buf,24,&current_state.velocity_z);
    
    current_state.position_x*=100.0f;
    current_state.position_y*=100.0f;
    current_state.position_z*=100.0f;
    
    
    if(current_state.velocity_y<0) current_state.velocity_y+=360;//356.384094
    
    for(uint16_t i=Laser_Min_Info_Num-1;i>0;i--)
    {
      min_dis_cm_backups[i]=min_dis_cm_backups[i-1];
      min_dis_angle_backups[i]=min_dis_angle_backups[i-1];
    }
    min_dis_cm_backups[0]		=current_state.velocity_x*100.0f;//转换成cm
    min_dis_angle_backups[0]=current_state.velocity_y;		
    
    
    Byte2Float(data_buf,28,&current_state.q[0]);		
    Byte2Float(data_buf,32,&current_state.q[1]);
    Byte2Float(data_buf,36,&current_state.q[2]);
    Byte2Float(data_buf,40,&current_state.q[3]);
    float _q[4];
    _q[0]=current_state.q[3]*(1.0f);
    _q[1]=current_state.q[0]*(1.0f);
    _q[2]=current_state.q[1]*(1.0f);
    _q[3]=current_state.q[2]*(1.0f);	
    
    current_state.q[0]=_q[0];
    current_state.q[1]=_q[1];
    current_state.q[2]=_q[2];
    current_state.q[3]=_q[3];
    
    Byte2Float(data_buf,44,&current_state.quality);
    current_state.update_flag=*(data_buf+48);
    current_state.byte[0]=*(data_buf+49);
    current_state.byte[1]=*(data_buf+50);
    current_state.byte[2]=*(data_buf+51);
    current_state.byte[3]=*(data_buf+52);
    current_state.byte[4]=*(data_buf+53);
    current_state.byte[5]=*(data_buf+54);
    current_state.byte[6]=*(data_buf+55);
    current_state.byte[7]=*(data_buf+56);
    
    current_state.rec_update_flag=1;
    current_state.rec_head_update_flag=1;		
    
    Pilot_Status_Tick();
    get_systime(&nclink_t);
    current_state.slam_sensor=RPLIDAR_SLAM;
  }	
  else if(*(data_buf+2)==0X0F)//导航控制指令
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
      else//数据来源于无名创新地面站
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
    Pilot_Status_Tick();	
    send_check_back=2;
  }	
  else if(*(data_buf+2)==0X10)//fastlio
  {
    Byte2Float(data_buf,4,&current_state.position_x);
    Byte2Float(data_buf,8,&current_state.position_y);
    Byte2Float(data_buf,12,&current_state.position_z);
    
    current_state.position_x*=100.0f;
    current_state.position_y*=100.0f;
    current_state.position_z*=100.0f;
    
    Byte2Float(data_buf,28,&current_state.q[0]);		
    Byte2Float(data_buf,32,&current_state.q[1]);
    Byte2Float(data_buf,36,&current_state.q[2]);
    Byte2Float(data_buf,40,&current_state.q[3]);
    float _q[4];
    _q[0]=current_state.q[3]*(1.0f);
    _q[1]=current_state.q[0]*(1.0f);
    _q[2]=current_state.q[1]*(1.0f);
    _q[3]=current_state.q[2]*(1.0f);	
    
    current_state.q[0]=_q[0];
    current_state.q[1]=_q[1];
    current_state.q[2]=_q[2];
    current_state.q[3]=_q[3];
    
    Byte2Float(data_buf,44,&current_state.quality);
    current_state.update_flag=*(data_buf+48);
    current_state.byte[0]=*(data_buf+49);
    current_state.byte[1]=*(data_buf+50);
    current_state.byte[2]=*(data_buf+51);
    current_state.byte[3]=*(data_buf+52);
    current_state.byte[4]=*(data_buf+53);
    current_state.byte[5]=*(data_buf+54);
    current_state.byte[6]=*(data_buf+55);
    current_state.byte[7]=*(data_buf+56);
    
    Pilot_Status_Tick();
    get_systime(&nclink_t);
    //判断loam数据已经正常更新
    if(current_state.position_x==0&&current_state.position_y==0)
    {
      current_state.loam_update_flag=0;
      return ;//数据非正常更新时,提前退出
    }
    else current_state.loam_update_flag=1;
    
    current_state.rec_update_flag=1;
    current_state.rec_head_update_flag=1;		
    current_state.slam_sensor=LOAM;
  }	
}


extern ring_buff_t com2_rx_buf;
void reserved_serialport_prase(void)
{
  if(uart2_rx_idle_flag==0) return;//通过判断串口空闲标志，来决策是否解析数据
  uart2_rx_idle_flag=0;
  uint16_t valid_byte_start=0;
  if(com2_rx_buf.tail<=59)//0-59数据位正在传输
  {
    valid_byte_start=0;
  }
  else//60-119数据位正在传输
  {
    valid_byte_start=60;
  }	
  for(uint16_t i=0;i<60;i++)
  {
    NCLink_Data_Prase_Prepare_Lite(com2_rx_buf.ring_buff[valid_byte_start+i]);
  }
}

