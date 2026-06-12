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
#include "drv_notify.h"
#include "schedule.h"

Bling_Light light_red={
  .pin=P22_0,
};

Bling_Light light_green={
  .pin=P23_3,
};

Bling_Light light_blue={
  .pin=P23_4,
};

void rgb_start_bling(void);

/***************************************
函数名:	void rgb_init(void)
说明: rgb灯所在的IO初始化
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void rgb_notify_init(void)
{
  gpio_init(light_red.pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
  gpio_init(light_green.pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
  gpio_init(light_blue.pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
  
  rgb_start_bling();            //开机LED预显示
}


/***************************************************
函数名: void Bling_Set(Bling_Light *Light,
uint32_t Continue_time,//持续时间
uint16_t Period,//周期100ms~1000ms
float Percent,//0~100%
uint16_t  Cnt,
GPIO_TypeDef* Port,
uint16_t Pin
,uint8_t Flag)
说明:	状态指示灯设置函数
入口:	时间、周期、占空比、端口等
出口:	无
备注:	程序初始化后、始终运行
作者:	无名创新
****************************************************/
void bling_set(Bling_Light *light,
               uint32_t Continue_time,//持续时间
               uint16_t Period,//周期100ms~1000ms
               float Percent,//0~100%
               uint16_t  Cnt,
               uint8_t Flag)
{
  light->contiune_t=(uint16_t)(Continue_time/5);//持续时间
  light->period=Period;//周期
  light->percent=Percent;//占空比
  light->cnt=Cnt;
  //light->port=Port;//端口
  //light->pin=Pin;//引脚
  light->endless_flag=Flag;//无尽模式
}

/***************************************************
函数名: void Bling_Process(Bling_Light *Light)//闪烁运行线程
说明:	状态指示灯实现
入口:	状态灯结构体
出口:	无
备注:	程序初始化后、始终运行
作者:	无名创新
****************************************************/
void bling_process(Bling_Light *light)//闪烁运行线程
{
  if(light->contiune_t>=1)  light->contiune_t--;
  else gpio_high(light->pin);//置高，灭
  if(light->contiune_t!=0//总时间未清0
     ||light->endless_flag==1)//判断无尽模式是否开启
  {
    light->cnt++;
    if(5*light->cnt>=light->period) light->cnt=0;//计满清零
    if(5*light->cnt<=light->period*light->percent)
      gpio_low(light->pin);//置低，亮
    else gpio_high(light->pin);//置高，灭
  }
}



void nGPIO_SetBits(Bling_Light *light)
{
  gpio_high(light->pin);//置高，灭
}

void nGPIO_ResetBits(Bling_Light *light)
{
  gpio_low(light->pin);//置低，亮
}


/***************************************************
函数名: Bling_Working(uint16 bling_mode)
说明:	状态指示灯状态机
入口:	当前模式
出口:	无
备注:	程序初始化后、始终运行
作者:	无名创新
****************************************************/
void rgb_notify_work(void)
{
  bling_process(&light_blue);
  bling_process(&light_red);
  bling_process(&light_green);
}

/***************************************
函数名:	void rgb_start_bling(void)
说明: 开机后初始闪烁
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void rgb_start_bling(void)
{
  bling_set(&light_red  ,1000,200,0.5,0,0);//红色
  bling_set(&light_green,1000,200,0.5,0,0);//绿色
  bling_set(&light_blue ,1000,200,0.5,0,0);//蓝色
}



void rgb_notify_set(rgb_lamp lamp,rgb_mode _mode,uint32_t period,uint32_t time,uint32_t free_t)
{
  switch(lamp)
  {
  case RED:
    {
      bling_set(&light_red,period,time,0.5,0,0);
    }
    break;
  case GREEN:
    {
      bling_set(&light_green,period,time,0.5,0,0);
    }
    break;
  case BLUE:
    {
      bling_set(&light_blue,period,time,0.5,0,0);
    }
    break;
  default:	break;
  }
}


