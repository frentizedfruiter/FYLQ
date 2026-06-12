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


#include "schedule.h"
#include "attitude_ctrl.h"
#include "Developer_Mode.h"
#include "nclink.h"
#include "drv_uart.h"
#include "parameter_server.h"
#include "offboard.h"

static uint8_t offboard_send_buf[128];

void pilot_send_to_planner(float px,float py,float pz,
                           float vx,float vy,float vz,
                           float ax,float ay,float az,
                           float gx,float gy,float gz,
                           float _q0,float _q1,float _q2,float _q3,
                           uint8_t ahrs_ok,uint8_t slam_ok)
{
  uint8_t sum=0,_cnt=0,i=0;
  int16_t _temp;
  
  offboard_send_buf[_cnt++]=0xFF;
  offboard_send_buf[_cnt++]=0xFC;
  offboard_send_buf[_cnt++]=NCLINK_SEND_AHRS;
  offboard_send_buf[_cnt++]=0;
  
  Float2Byte(&px,offboard_send_buf,_cnt);//4
  _cnt+=4;
  Float2Byte(&py,offboard_send_buf,_cnt);//8
  _cnt+=4;
  Float2Byte(&pz,offboard_send_buf,_cnt);//12
  _cnt+=4;
  
  Float2Byte(&vx,offboard_send_buf,_cnt);//16
  _cnt+=4;
  Float2Byte(&vy,offboard_send_buf,_cnt);//20
  _cnt+=4;
  Float2Byte(&vz,offboard_send_buf,_cnt);//24
  _cnt+=4;
  
  Float2Byte(&ax,offboard_send_buf,_cnt);//28
  _cnt+=4;
  Float2Byte(&ay,offboard_send_buf,_cnt);//32
  _cnt+=4;
  Float2Byte(&az,offboard_send_buf,_cnt);//36
  _cnt+=4;
  
  Float2Byte(&gx,offboard_send_buf,_cnt);//40
  _cnt+=4;
  Float2Byte(&gy,offboard_send_buf,_cnt);//44
  _cnt+=4;
  Float2Byte(&gz,offboard_send_buf,_cnt);//48
  _cnt+=4;
  
  _temp = (int)(_q0*10000);
  offboard_send_buf[_cnt++]=BYTE1(_temp);
  offboard_send_buf[_cnt++]=BYTE0(_temp);
  _temp = (int)(_q1*10000);
  offboard_send_buf[_cnt++]=BYTE1(_temp);
  offboard_send_buf[_cnt++]=BYTE0(_temp);
  _temp = (int)(_q2*10000);
  offboard_send_buf[_cnt++]=BYTE1(_temp);
  offboard_send_buf[_cnt++]=BYTE0(_temp);
  _temp = (int)(_q3*10000);
  offboard_send_buf[_cnt++]=BYTE1(_temp);
  offboard_send_buf[_cnt++]=BYTE0(_temp);
  
  offboard_send_buf[_cnt++]=BYTE0(ahrs_ok);
  offboard_send_buf[_cnt++]=BYTE0(slam_ok);
  
  offboard_send_buf[_cnt++]=BYTE0(offboard_start_flag);
  offboard_send_buf[_cnt++]=BYTE0(maplepilot.yaw_ctrl_end);
  offboard_send_buf[_cnt++]=BYTE0(maplepilot.yaw_ctrl_response);//偏航命令响应标志位
  
  
  offboard_send_buf[3] = _cnt-4;
  
  for(i=0;i<_cnt;i++) sum ^= offboard_send_buf[i]; 
  offboard_send_buf[_cnt++]=sum;
  
  offboard_send_buf[_cnt++]=0xA1;
  offboard_send_buf[_cnt++]=0xA2;
  uart_sendbytes(5,offboard_send_buf,_cnt);//用户移植时，重写此串口发送函数(offboard_send_buf,_cnt);
}



uint32_t miss_test[2];
uint32_t miss_test1[2];
float miss_rate=0;
systime navcmd_t[2];
float navcmd_dt=0;
void NCLink_Data_Prase_Planner_CMD(uint8_t data)
{
  static uint8_t cmd_buf[100];
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==0xFF)//判断帧头1
  {
    state=1;
    cmd_buf[0]=data;
    get_systime(&navcmd_t[0]);
  }
  else if(state==1&&data==0xFC)//判断帧头2
  {
    state=2;
    cmd_buf[1]=data;
  }
  else if(state==2&&data<0XF1)//功能字节
  {
    state=3;
    cmd_buf[2]=data;
  }
  else if(state==3&&data<100)//有效数据长度
  {
    state = 4;
    cmd_buf[3]=data;
    data_len = data;
    data_cnt = 0;
  }
  else if(state==4&&data_len>0)//数据接收
  {
    data_len--;
    cmd_buf[4+data_cnt++]=data;
    if(data_len==0)  state = 5;
  }
  else if(state==5)//异或校验
  {
    state = 6;
    cmd_buf[4+data_cnt++]=data;
  }
  else if(state==6&&data==0xA1)//帧尾0
  {
    state = 7;
    cmd_buf[4+data_cnt++]=data;
  }
  else if(state==7&&data==0xA2)//帧尾1
  {
    state = 0;
    cmd_buf[4+data_cnt]=data;
    uint16_t _cnt=data_cnt+5;
    uint8_t sum = 0;
    for(uint8_t i=0;i<(_cnt-3);i++)  sum ^= *(cmd_buf+i);
    if(!(sum==*(cmd_buf+_cnt-3)))    												return;//判断sum	
    if(!(*(cmd_buf)==0xFF&&*(cmd_buf+1)==0xFC))             return;//判断帧头
    if(!(*(cmd_buf+_cnt-2)==0xA1&&*(cmd_buf+_cnt-1)==0xA2)) return;//帧尾校验
    if(*(cmd_buf+2)==NCLINK_SEND_AHRS)//数据帧类型判断
    {
      planner_cmd._yaw_ctrl_mode=*(cmd_buf+4);
      planner_cmd._yaw_ctrl_start=*(cmd_buf+5);
      
      Byte2Float(cmd_buf, 6, &planner_cmd._roll_outer_control_output);
      Byte2Float(cmd_buf, 10,&planner_cmd._pitch_outer_control_output);
      Byte2Float(cmd_buf, 14,&planner_cmd._yaw_outer_control_output);
      
      Byte2Float(cmd_buf, 18,&planner_cmd._nav_target_vel.x);
      Byte2Float(cmd_buf, 22,&planner_cmd._nav_target_vel.y);
      Byte2Float(cmd_buf, 26,&planner_cmd._nav_target_vel.z);
      
      planner_cmd._execution_time_ms=(int32_t)(*(cmd_buf+30)<<24 | *(cmd_buf+31)<<16 | *(cmd_buf+32)<<8 | *(cmd_buf+33));
    }
    get_systime(&navcmd_t[1]);
    navcmd_dt=navcmd_t[1].current_time-navcmd_t[0].current_time;
    
    miss_test[0]++;
    miss_test1[0]=37*miss_test[0];		
    miss_test1[1]=miss_test1[0]+miss_test[1];
    miss_rate=((float)miss_test1[0]/miss_test1[1]);
  }
  else 
  {
    state = 0;
    miss_test[1]++;
  }
}



void offboard_uart5_init(void)
{
  if(other_params.params.inner_uart==2)
  {
    uart5_init(921600);//offboard通讯串口
  }
  else
  {
    uart5_init(256000);//OPENMV串口5资源初始化
  }
}









