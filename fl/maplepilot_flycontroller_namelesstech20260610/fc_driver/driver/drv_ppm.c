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
#include "main.h"
#include "datatype.h"
#include "drv_ppm.h"
#include "schedule.h"
#include <string.h>


void ppm_init(void)
{  
  //exti_init(P13_2, EXTI_TRIGGER_RISING);
  //interrupt_set_priority(CPUIntIdx7_IRQn, 1);
  exti_init_intidx(P13_2, EXTI_TRIGGER_RISING,ppm_irq_chl_pre_priority,ppm_irq_chl_sub_priority);
}


ppm ppm_rc;



void _PPM_IRQHandler(void)
{
  static uint32_t ppm_miss_cnt,ppm_work_cnt;  
  static uint16_t _ppm_buf[8]={0};
  static uint32_t last_ppm_time=0,now_ppm_time=0;
  static uint8_t ppm_ready=0,ppm_sample_cnt=0;
  //系统运行时间获取，单位us
  last_ppm_time=now_ppm_time;
  now_ppm_time=micros();//us
  volatile uint32_t ppm_time_delta=now_ppm_time-last_ppm_time;
  
  if(ppm_time_delta<600)   return;//剔除掉异常值
  if(ppm_time_delta>12500) return;//剔除掉PWM
  
  //PPM解析开始
  if(ppm_time_delta>=2500&&ppm_ready==0)//帧结束电平至少2.5ms=2500us
  {
    ppm_ready=1;
    ppm_sample_cnt=0;
    return ;//本次跳出
  }
  
  if(ppm_ready==1)
  {
    if(ppm_time_delta<=800||ppm_time_delta>2200)//根据脉宽判断数据是否异常
    {			
      ppm_ready=0;
      ppm_sample_cnt=0;
      ppm_miss_cnt++;			
      return ;
    }
    
    if(ppm_sample_cnt==0)
    {
      _ppm_buf[0]=ppm_time_delta;//0通道写入缓冲区
      ppm_sample_cnt=1;
    }
    else if(ppm_sample_cnt==1)
    {
      _ppm_buf[1]=ppm_time_delta;//1通道写入缓冲区
      ppm_sample_cnt=2;
    }
    else if(ppm_sample_cnt==2)
    {
      _ppm_buf[2]=ppm_time_delta;//2通道写入缓冲区
      ppm_sample_cnt=3;
    }
    else if(ppm_sample_cnt==3)
    {
      _ppm_buf[3]=ppm_time_delta;//3通道写入缓冲区
      ppm_sample_cnt=4;
    }	
    else if(ppm_sample_cnt==4)
    {
      _ppm_buf[4]=ppm_time_delta;//4通道写入缓冲区
      ppm_sample_cnt=5;
    }	
    else if(ppm_sample_cnt==5)
    {
      _ppm_buf[5]=ppm_time_delta;//5通道写入缓冲区
      ppm_sample_cnt=6;
    }
    else if(ppm_sample_cnt==6)
    {
      _ppm_buf[6]=ppm_time_delta;//6通道写入缓冲区
      ppm_sample_cnt=7;
    }
    else if(ppm_sample_cnt==7)
    {
      _ppm_buf[7]=ppm_time_delta;//7通道写入缓冲区
      memcpy(ppm_rc.data,_ppm_buf,8*sizeof(uint16_t));
      ppm_ready=0;//所有通道数据接收完毕,结束本次接收
      ppm_sample_cnt=0;	
      ppm_rc.update_flag=1;//解析完成标志位
      ppm_work_cnt++;	
    }			
  }
  ppm_rc.accuracy=(float)(ppm_work_cnt)/(ppm_work_cnt+ppm_miss_cnt);
}

