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
#include "drv_button.h"


rc_buttton _button;
void nkey_init(void)
{
  gpio_init(P11_1, GPI, GPIO_HIGH, GPI_PULL_UP);
  gpio_init(P11_2, GPI, GPIO_HIGH, GPI_PULL_UP);
  
  _button.state[IUP].pin=P11_1;
  _button.state[IUP].value=1;
  _button.state[IUP].last_value=1;
  
  _button.state[IDOWN].pin=P11_2;
  _button.state[IDOWN].value=1;
  _button.state[IDOWN].last_value=1;	
  
  /******************************************************************************************/
  _button.state[UP_3D].pin=NC;
  _button.state[UP_3D].value=1;
  _button.state[UP_3D].last_value=1;	
  
  _button.state[DN_3D].pin=NC;
  _button.state[DN_3D].value=1;
  _button.state[DN_3D].last_value=1;	
  
  _button.state[LT_3D].pin=NC;
  _button.state[LT_3D].value=1;
  _button.state[LT_3D].last_value=1;	
  
  _button.state[RT_3D].pin=NC;
  _button.state[RT_3D].value=1;
  _button.state[RT_3D].last_value=1;	
  
  _button.state[ME_3D].pin=NC;
  _button.state[ME_3D].value=1;
  _button.state[ME_3D].last_value=1;		
}

void read_button_state_one(button_state *button)
{
  button->value=((gpio_get_level(button->pin)!=0)?1:0);
  if(button->value==0)
  {
    if(button->last_value!=0)//首次按下
    {
      button->press_time=millis();//记录按下的时间点
      button->in_time=millis();//记录按下的时间点
      button->in_press_cnt=0;
    }
    else
    {
      if(millis()-button->press_time>KEEP_LONG_PRESS_LIMIT)//达到持续长按时间限制，声音提示可以松开了
      {
        //				beep.period=20;//20*5ms
        //				beep.light_on_percent=0.5f;			
        //				beep.reset=1;
        //				beep.times=1;			
      }
      else if(millis()-button->in_time>IN_PRESS_LIMIT)//持续按下
      {
        button->in_time=millis();
        button->press=IN_PRESS;
        if(button->press==IN_PRESS)  button->in_press_cnt++;
      }
      
    }
  }
  else
  {
    if(button->last_value==0)//按下后释放
    {
      button->release_time=millis();//记录释放时的时间
      if(button->release_time-button->press_time>KEEP_LONG_PRESS_LIMIT)//持续长按按键5S
      {
        button->press=KEEP_LONG_PRESS;
        button->state_lock_time=0;
        
        buzzer_setup(1000,0.5f,4);				
      }
      else if(button->release_time-button->press_time>LONG_PRESS_LIMIT)//长按按键1S
      {
        button->press=LONG_PRESS;
        button->state_lock_time=0;//5ms*300=1.5S
        
        buzzer_setup(1000,0.5f,1);
      }
      else
      {
        button->press=SHORT_PRESS;
        button->state_lock_time=0;//5ms*300=1.5S
        
        buzzer_setup(100,0.5f,1);
      }
    }
  }
  button->last_value=button->value;
  
  if(button->press==LONG_PRESS
     ||button->press==SHORT_PRESS)//按下释放后，程序后台1.5S内无响应，复位按键状态
  {
    button->state_lock_time++;
    if(button->state_lock_time>=300)
    {			
      button->press=NO_PRESS;
      button->state_lock_time=0;
    }
  }
}


void read_button_state_all(void)
{
  for(uint16_t i=0;i<2;i++)  
  {
    read_button_state_one(&_button.state[i]);
  }	
}


