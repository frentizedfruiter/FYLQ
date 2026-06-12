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
#include "schedule.h"
#include "drv_button.h"
#include "drv_adc.h"
#include "parameter_server.h"
#include "rc.h"
#include "drv_expand.h"
#include "adc_button.h"

uint8_t adc_key1_ctrl_enable=0;//使能按键控制时设置为1，反之设置为0
uint8_t adc_key2_ctrl_enable=1;
uint8_t wireless_adc_key_ctrl_enable=0;
#define ADC_LONG_PRESS_LIMIT  1500//1500ms
#define ADC_IN_PRESS_LIMIT  	500//250ms


button_state ADC_Button1,ADC_Button2,ADC_Button3,ADC_Button4;
float Button_ADCResult[2];
uint16_t ADC_PPM_Databuf[10]={0};
float tmp_adckey_enable[3]={0};
void adc_button_init(void)
{
  ADC_Button1.pin=NC;
  ADC_Button1.value=1;
  ADC_Button1.last_value=1;
  
  ADC_Button2.pin=NC;
  ADC_Button2.value=1;
  ADC_Button2.last_value=1;
  
  ADC_Button3.pin=NC;
  ADC_Button3.value=1;
  ADC_Button3.last_value=1;
  
  ADC_Button4.pin=NC;
  ADC_Button4.value=1;
  ADC_Button4.last_value=1;
  
  
  ReadFlashParameterOne(ADC_KEY1_ENABLE,&tmp_adckey_enable[0]);
  ReadFlashParameterOne(ADC_KEY2_ENABLE,&tmp_adckey_enable[1]);
  ReadFlashParameterOne(WIRELESS_ADC_KEY_ENABLE,&tmp_adckey_enable[2]);
  
  if(isnan(tmp_adckey_enable[0])==0)   adc_key1_ctrl_enable=tmp_adckey_enable[0];
  else adc_key1_ctrl_enable=0;
  
  if(isnan(tmp_adckey_enable[1])==0)   adc_key2_ctrl_enable=tmp_adckey_enable[1];
  //else adc_key2_ctrl_enable=0;
  
  if(isnan(tmp_adckey_enable[2])==0)   wireless_adc_key_ctrl_enable=tmp_adckey_enable[2];
  else wireless_adc_key_ctrl_enable=0;
}




uint8_t Key_Value[2]={0,0};
void adc_button_read(void)
{		
  //将原始的数字12位ADC数字亮转化成电压,单位V
  if(wireless_adc_key_ctrl_enable==1)//adc电压来源于无线数传传输，如小车
  {
    //Button_ADCResult[0]=wireless_adc_value[0]*3.3f/4095.0f;//PD3
    //Button_ADCResult[1]=wireless_adc_value[1]*3.3f/4095.0f;//PD2
  }
  else//adc电压来源于飞控自身端口采集
  {
    Button_ADCResult[0]=adc_value[0]*3.3f/4095.f;//P06.6
    Button_ADCResult[1]=adc_value[1]*3.3f/4095.f;//P07.2
  }	
  
  if(Button_ADCResult[0]<=0.5f)         Key_Value[0]=0x00;//没有按键按下，下拉5.1K，输入0V
  else if(Button_ADCResult[0]<=1.70f)   Key_Value[0]=0x04;//降落按键按下，	 上接5.1K，输入3.3V*5.1/10.2=1.65V
  else if(Button_ADCResult[0]<=2.10f)   Key_Value[0]=0x02;//上锁按键按下，  上接3.3K，输入3.3V*5.1/8.40=2.00V
  else if(Button_ADCResult[0]<=2.40f)   Key_Value[0]=0x03;//开发按键按下，  上接2.2K，输入3.3V*5.1/7.30=2.31V
  else if(Button_ADCResult[0]<=3.00f)   Key_Value[0]=0x01;//解锁按键按下，  上接1.2K，输入3.3V*5.1/6.30=2.67V
  else Key_Value[0]=0x00;
  
  switch(Key_Value[0])
  {
  case 0://没有按键按下
    {
      ADC_Button1.value=1;
      ADC_Button2.value=1;
      ADC_Button3.value=1;
      ADC_Button4.value=1;
    }
    break;
  case 1://解锁按键按下
    {
      ADC_Button1.value=0;
      ADC_Button2.value=1;
      ADC_Button3.value=1;
      ADC_Button4.value=1;		
    }
    break;		
  case 2://开发按键按下
    {
      ADC_Button1.value=1;
      ADC_Button2.value=0;
      ADC_Button3.value=1;
      ADC_Button4.value=1;		
    }
    break;		
  case 3://上锁按键按下
    {
      ADC_Button1.value=1;
      ADC_Button2.value=1;
      ADC_Button3.value=0;
      ADC_Button4.value=1;		
    }
    break;	
  case 4://降落按键按下
    {
      ADC_Button1.value=1;
      ADC_Button2.value=1;
      ADC_Button3.value=1;
      ADC_Button4.value=0;		
    }
    break;	
  default://没有按键按下
    {
      ADC_Button1.value=1;
      ADC_Button2.value=1;
      ADC_Button3.value=1;
      ADC_Button4.value=1;				
    }
    break;	
  }
  
  
  //
  if(Button_ADCResult[1]<=0.5f)  		   Key_Value[1]=0x00;//没有按键按下，下拉5.1K，输入0V
  else if(Button_ADCResult[1]<=1.35f)  Key_Value[1]=0x05;//上按键按下，	上接10K，输入3.3V*5.1/15.1=1.11V
  else if(Button_ADCResult[1]<=1.70f)  Key_Value[1]=0x04;//右按键按下，	上接5.1K，输入3.3V*5.1/10.2=1.65V
  else if(Button_ADCResult[1]<=2.10f)  Key_Value[1]=0x03;//左按键按下， 上接3.3K，输入3.3V*5.1/8.40=2.00V
  else if(Button_ADCResult[1]<=2.40f)  Key_Value[1]=0x02;//中按键按下， 上接2.2K，输入3.3V*5.1/7.30=2.31V
  else if(Button_ADCResult[1]<=3.00f)  Key_Value[1]=0x01;//下按键按下， 上接1.2K，输入3.3V*5.1/6.30=2.67V
  else Key_Value[1]=0x00;
  
  switch(Key_Value[1])
  {
  case 0://没有按键按下
    {
      _button.state[UP_3D].value=1;
      _button.state[DN_3D].value=1;
      _button.state[LT_3D].value=1;
      _button.state[RT_3D].value=1;
      _button.state[ME_3D].value=1;
    }
    break;
  case 1://下按键按下
    {
      _button.state[UP_3D].value=1;
      _button.state[DN_3D].value=0;
      _button.state[LT_3D].value=1;
      _button.state[RT_3D].value=1;
      _button.state[ME_3D].value=1;	
    }
    break;		
  case 2://中按键按下
    {
      _button.state[UP_3D].value=1;
      _button.state[DN_3D].value=1;
      _button.state[LT_3D].value=1;
      _button.state[RT_3D].value=1;
      _button.state[ME_3D].value=0;	
    }
    break;		
  case 3://左按键按下
    {
      _button.state[UP_3D].value=1;
      _button.state[DN_3D].value=1;
      _button.state[LT_3D].value=0;
      _button.state[RT_3D].value=1;
      _button.state[ME_3D].value=1;	
    }
    break;	
  case 4://右按键按下
    {
      _button.state[UP_3D].value=1;
      _button.state[DN_3D].value=1;
      _button.state[LT_3D].value=1;
      _button.state[RT_3D].value=0;
      _button.state[ME_3D].value=1;	
    }
    break;
  case 5://上按键按下
    {
      _button.state[UP_3D].value=0;
      _button.state[DN_3D].value=1;
      _button.state[LT_3D].value=1;
      _button.state[RT_3D].value=1;
      _button.state[ME_3D].value=1;	
    }
    break;		
  default://没有按键按下
    {
      _button.state[UP_3D].value=1;
      _button.state[DN_3D].value=1;
      _button.state[LT_3D].value=1;
      _button.state[RT_3D].value=1;
      _button.state[ME_3D].value=1;			
    }
    break;	
  }
}



void Read_ADC_Button_State_One(button_state *button)
{
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
      if(millis()-button->in_time>ADC_IN_PRESS_LIMIT)//持续按下
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
      
      if(button->release_time-button->press_time>ADC_LONG_PRESS_LIMIT)
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


void Read_ADC_Button1_State_All(void)
{
  if(adc_key1_ctrl_enable==0)  return ;//ADC按键1未使能时，直接返回
  
  Read_ADC_Button_State_One(&ADC_Button1);
  Read_ADC_Button_State_One(&ADC_Button2);
  Read_ADC_Button_State_One(&ADC_Button3);
  Read_ADC_Button_State_One(&ADC_Button4);
}

void Read_ADC_Button2_State_All(void)
{
  if(adc_key2_ctrl_enable==0)  return ;//ADC按键2未使能时，直接返回
  
  Read_ADC_Button_State_One(&_button.state[UP_3D]);
  Read_ADC_Button_State_One(&_button.state[DN_3D]);
  Read_ADC_Button_State_One(&_button.state[LT_3D]);
  Read_ADC_Button_State_One(&_button.state[RT_3D]);
  Read_ADC_Button_State_One(&_button.state[ME_3D]);
}



uint8_t adc_ppm_update_flag=0;

void adc_button_scan(void)
{
  adc_button_read();
  Read_ADC_Button1_State_All();
  Read_ADC_Button2_State_All();
  
  if(adc_key1_ctrl_enable==0)  return ;//ADC按键1未使能时，直接返回
  
  //以下为解锁按键事件处理逻辑
  if(ADC_Button1.press==IN_PRESS)
  {
    //以下为该按键实现的任务
    adc_ppm_update_flag=1;//控制权限给ADC按键设置的PPM
    ADC_PPM_Databuf[0]=1500;//俯仰杆位于中位
    ADC_PPM_Databuf[1]=1500;//横滚杆位于中位
    ADC_PPM_Databuf[2]=1000;//油门杆位于下侧
    ADC_PPM_Databuf[3]=2000;//偏航杆位于右侧
    
    ADC_PPM_Databuf[4]=2000;//第五通道高位，处于定高模式
    ADC_PPM_Databuf[5]=1000;//第六通道低位
    ADC_PPM_Databuf[6]=1000;//第七通道低位
    ADC_PPM_Databuf[7]=1000;//第八通道低位
    //Bling_Set(&Light_Green,5000,1000,0.5,0,GPIO_PORTF_BASE,BLING_G_PIN,1);
  }
  else if(ADC_Button1.press==LONG_PRESS)//解锁按键长按
  {
    //以下为该按键实现的任务
    adc_ppm_update_flag=1;//控制权限给ADC按键设置的PPM
    if(rc_data.lock_state==UNLOCK)//如果处于解锁状态，遥控器油门回中准备执行SDK
    {
      ADC_PPM_Databuf[0]=1500;//俯仰杆位于中位
      ADC_PPM_Databuf[1]=1500;//横滚杆位于中位
      ADC_PPM_Databuf[2]=1000;//油门杆位于下侧
      ADC_PPM_Databuf[3]=1500;//偏航杆位于中位
      
      ADC_PPM_Databuf[4]=2000;//第五通道高位，处于定高模式
      ADC_PPM_Databuf[5]=1000;//第六通道低位
      ADC_PPM_Databuf[6]=1000;//第七通道低位
      ADC_PPM_Databuf[7]=1000;//第八通道高位
      //Bling_Set(&Light_Green,5000,1000,0.5,0,GPIO_PORTF_BASE,BLING_G_PIN,1);
    }
  }
  else if(ADC_Button1.press==SHORT_PRESS)//解锁按键短按
  {
    //以下为该按键实现的任务
    adc_ppm_update_flag=0;//控制权限给释放给其它渠道获取的PPM：PPM接收机、地面站虚拟遥控器
    //故按键启动后，此时短按下按键，遥控器控制权限瞬间会释放给接收机
    //遥控器接管后，可以手动控制飞机安全降落
    
    //Bling_Set(&Light_Green,5000,1000,0.5,0,GPIO_PORTF_BASE,BLING_G_PIN,0);
  }
  
  
  //以下为上锁按键事件处理逻辑
  if(ADC_Button2.press==IN_PRESS)
  {
    //以下为该按键实现的任务
  }
  else if(ADC_Button2.press==LONG_PRESS)//上锁按键长按
  {
    //以下为该按键实现的任务
    
  }
  else if(ADC_Button2.press==SHORT_PRESS)//上锁按键短按
  {
    //以下为该按键实现的任务
    adc_ppm_update_flag=0;//控制权限给释放给其它渠道获取的PPM：PPM接收机、地面站虚拟遥控器
    //故按键启动后，此时短按下按键，遥控器控制权限瞬间会释放给接收机
    //遥控器接管后，可以手动控制飞机安全降落
    
    if(rc_data.lock_state==UNLOCK)//如果处于解锁状态
    {
      rc_data.lock_state=LOCK;//直接上锁
      //Bling_Set(&Light_Green,5000,1000,0.5,0,GPIO_PORTF_BASE,BLING_G_PIN,0);
    }	
  }
  
  rc_data.lock_state=LOCK;
  
  
  //以下为SDK按键事件处理逻辑
  if(ADC_Button3.press==IN_PRESS)
  {
    //以下为该按键实现的任务
  }
  else if(ADC_Button3.press==LONG_PRESS)//SDK按键长按
  {
    //以下为该按键实现的任务
    adc_ppm_update_flag=1;//控制权限给ADC按键设置的PPM
    if(rc_data.lock_state==UNLOCK)//如果处于解锁状态，执行SDK
    {		
      ADC_PPM_Databuf[0]=1500;//俯仰杆位于中位
      ADC_PPM_Databuf[1]=1500;//横滚杆位于中位
      ADC_PPM_Databuf[2]=1500;//油门杆位于中侧
      ADC_PPM_Databuf[3]=1500;//偏航杆位于中位
      
      ADC_PPM_Databuf[4]=2000;//第五通道高位，处于定高模式
      ADC_PPM_Databuf[5]=2000;//第六通道高位，处于SDK模式
      ADC_PPM_Databuf[6]=1000;//第七通道低位
      ADC_PPM_Databuf[7]=2000;//第八通道高位，处于光流定点模式
      
      //Bling_Set(&Light_Green,5000,1000,0.5,0,GPIO_PORTF_BASE,BLING_G_PIN,1);
    }
  }
  else if(ADC_Button3.press==SHORT_PRESS)//SDK按键短按
  {
    //以下为该按键实现的任务
    adc_ppm_update_flag=1;//控制权限给ADC按键设置的PPM
    if(rc_data.lock_state==UNLOCK)//如果处于解锁状态，退出SDK模式保持原地悬停
    {
      ADC_PPM_Databuf[0]=1500;//俯仰杆位于中位
      ADC_PPM_Databuf[1]=1500;//横滚杆位于中位
      ADC_PPM_Databuf[2]=1500;//油门杆位于中侧
      ADC_PPM_Databuf[3]=1500;//偏航杆位于中位
      
      ADC_PPM_Databuf[4]=2000;//第五通道高位，处于定高模式
      ADC_PPM_Databuf[5]=1000;//第六通道高位，退出SDK模式
      ADC_PPM_Databuf[6]=1000;//第七通道低位
      ADC_PPM_Databuf[7]=2000;//第八通道高位，处于光流定点模式
      //Bling_Set(&Light_Green,5000,1000,0.5,0,GPIO_PORTF_BASE,BLING_G_PIN,1);
    }
  }
  
  
  
  //以下为一键降落事件处理逻辑
  if(ADC_Button4.press==IN_PRESS)
  {
    //以下为该按键实现的任务
  }
  else if(ADC_Button4.press==LONG_PRESS)//一键降落按键长按
  {
    //以下为该按键实现的任务
    
  }
  else if(ADC_Button4.press==SHORT_PRESS)//一键降落按键短按
  {
    //以下为该按键实现的任务
    adc_ppm_update_flag=1;//控制权限给ADC按键设置的PPM
    if(rc_data.lock_state==UNLOCK)//如果处于解锁状态，退出SDK模式保持原地悬停
    {
      ADC_PPM_Databuf[0]=1500;
      ADC_PPM_Databuf[1]=1500;
      ADC_PPM_Databuf[2]=1500;
      ADC_PPM_Databuf[3]=1500;//前四个通道处于中位
      
      ADC_PPM_Databuf[4]=2000;//第五通道高位，处于定高模式
      ADC_PPM_Databuf[5]=1000;//第六通道低位
      ADC_PPM_Databuf[6]=2000;//第七通道高位，处于一键降落模式
      ADC_PPM_Databuf[7]=2000;//第八通道高位，处于光流定点模式
      //Bling_Set(&Light_Green,5000,1000,0.5,0,GPIO_PORTF_BASE,BLING_G_PIN,1);
    }
  }
  
}




