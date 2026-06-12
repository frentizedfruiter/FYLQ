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
#include "datatype.h"
//#include "drv_gps.h"
#include "schedule.h"


void systick_handler(void);
void (*systick_isr_func)() =systick_handler;


#define systick_div  1000
static volatile uint32_t systick_period=0;
uint32_t nvic_get_value=0;
void systickinit(void)
{
  NVIC_SetPriorityGrouping(0x04);//0x03  0x04  0x05 0x06 0x07
  nvic_get_value=NVIC_GetPriorityGrouping();
  
  systick_period=SystemCoreClock/500ul;
  //Cy_SysTick_Set10msCalibration(SystemCoreClock/100);
  //systick_period1=Cy_SysTick_Get10msCalibration();
  NVIC_SetPriority(SysTick_IRQn,NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0,0));
  //NVIC_SetPriority(SysTick_IRQn,0);
  Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU,systick_period-1);
  Cy_SysTick_SetCallback(0, systick_isr_func);  
  Cy_SysTick_Enable();
  Cy_SysTick_EnableInterrupt();
}


volatile uint32_t systickuptime = 0;
void systick_handler(void)
{
  systickuptime++;
}

//返回 us
uint32_t micros(void)
{
  return systickuptime*2000+(2000*(systick_period-SysTick->VAL))/systick_period;
}

//返回 ms
uint32_t millis(void) {
  return micros() / 1000UL;
}

//延时us
void _delayMicroseconds(uint32_t us) {
  uint32_t start = micros();
  while((int32_t)(micros() - start) < us) {
    // Do nothing
  };
}

void _delay(uint32_t ms) {
  _delayMicroseconds(ms * 1000UL);
}


void _delay_ms(uint32_t x)
{
  _delay(x);
}

void _delay_us(uint32_t x)
{
  _delayMicroseconds(x);
}

void _Delay_Ms(uint32_t time)  //延时函数  
{   
  _delay_ms(time);;  
}  

void _Delay_Us(uint32_t time)  //延时函数  
{   
  _delay_us(time); 
}  


void get_systime(systime *sys)
{
  sys->last_time=sys->current_time;
  sys->current_time=micros()/1000.0f;//单位ms
  sys->period=sys->current_time-sys->last_time;
  sys->period_int=(uint16_t)(sys->period+0.5f);//四舍五入
}



float get_systime_ms(void)
{
  return millis();//单位ms
}

uint32_t get_systick_ms(void)
{
  return (uint32_t)(2*systickuptime);//单位ms
}


/*
log_time cur={
.year=1980,//1925
.mon=2,
.day=5,
.hour=0,
.min=0,
.sec=0,
};


log_time get_log_time(void)
{
static log_time lst;
static uint32_t last_ms=0,cur_ms=0;
lst=cur;
last_ms=cur_ms;
cur_ms=get_systick_ms();//获取ms时基
float delta_ms=cur_ms-last_ms;//计算前后两次间隔的ms数

if(gps_data.update==1)
{
cur.year=gps_data.gps.year;
cur.mon=gps_data.gps.month;
cur.day=gps_data.gps.day;
cur.hour=(gps_data.gps.hour+8);
if(cur.hour>=24) cur.hour-=24;//24h==0h

cur.min=gps_data.gps.min;
cur.sec=gps_data.gps.sec;	

uint32_t lst_sec=lst.hour*60*60+lst.min*60+lst.sec;
uint32_t cur_sec=cur.hour*60*60+cur.min*60+cur.sec;
int32_t delta_sec=cur_sec-lst_sec;//计算前后两次间隔秒数
cur.ms=delta_ms-delta_sec*1000;
	}
	else
{
uint32_t sec=(int)(cur_ms/1000);//秒取整
cur.hour=sec/60/60;//时
cur.min=sec/60-cur.hour*60;//分;
cur.sec=sec%60;	//秒
cur.ms=cur_ms-sec*1000;
	}
return cur;
}
*/
