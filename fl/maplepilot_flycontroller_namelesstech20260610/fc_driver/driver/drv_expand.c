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
#include "schedule.h"
#include "drv_expand.h"



//使能前————务必取下R9/R10，否则飞控PWM输出会不正常，会导致炸鸡

_laser_light laser_light_1,beep;
void reserved_io_init(void)
{
  gpio_init(BEEP_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
  gpio_init(LASERLIGHT_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
  
  //  laser_light_1.port=GPIOD;
  laser_light_1.pin=LASERLIGHT_PIN;
  laser_light_1.period=200;//200*5ms
  laser_light_1.light_on_percent=0.2f;
  laser_light_1.reset=0;
  
  
  //  beep.port=GPIOC;
  beep.pin=BEEP_PIN;
  beep.period=200;//200*5ms
  beep.light_on_percent=0.5f;
  
  gpio_low(BEEP_PIN);
  gpio_low(LASERLIGHT_PIN);
}



void laser_light_work(_laser_light *light)
{
  if(light->reset==1)
  {
    light->reset=0;
    light->cnt=0;
    light->times_cnt=0;//点亮次数计数器
    light->end=0;
  }
  
  if(light->times_cnt==light->times)
  {
    light->end=1;
    return;
  }
  
  light->cnt++;
  if(light->cnt<light->period*light->light_on_percent)
  {
    gpio_high(light->pin);
  }
  else if(light->cnt<light->period)
  {
    gpio_low(light->pin);
  }
  else//完成点亮一次
  {
    light->cnt=0;
    light->times_cnt++;
  }
}


void buzzer_setup(uint32_t _period,float _light_on_percent,uint16_t _times)
{
  beep.period=_period/5;//20*5ms
  beep.light_on_percent=_light_on_percent;
  beep.reset=1;	
  beep.times=_times;
}


