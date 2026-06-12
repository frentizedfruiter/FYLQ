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
#include "main.h"
#include "quene.h"
#include "schedule.h"
#include "wp_math.h"
#include "filter.h"
#include "parameter_server.h"
#include "drv_tofsense.h"
#include "drv_opticalflow.h"


opticalflow opt_data;


systime optical_t;
void opticalflow_receive_anl(uint8_t *data_buf,uint8_t num);
void lc307_opticalflow_prase(uint8_t data)
{
  static uint8_t buf[14];
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==0xfe)//判断帧头1
  {
    state=1;
    buf[0]=data;
  }
  else if(state==1&&data==0x0a)//判断帧头2
  {
    state=2;
    buf[1]=data;
    data_len = 10;
    data_cnt = 0;
  }
  else if(state==2&&data_len>0)//数据接收
  {
    data_len--;
    buf[2+data_cnt++]=data;
    if(data_len==0)  state = 3;
  }
  else if(state==3)//异或校验
  {
    state = 4;
    buf[2+data_cnt++]=data;
  }
  else if(state==4)//判断帧头尾
  {
    static unsigned char last_valid=0;
    static uint32_t start_t,end_t;
    last_valid=opt_data.valid;
    state = 0;
    buf[2+data_cnt++]=data;    
    opticalflow_receive_anl(buf,14);
    get_systime(&optical_t);	
    if(last_valid==1&&opt_data.valid==0)//记录首次失效时刻
    {
      start_t=millis();
      opt_data.ctrl_valid=1;
    }
    else if(last_valid==0&&opt_data.valid==0)//持续记录失效时刻
    {
      end_t=millis();					
      if(end_t-start_t>=1000)//持续失效1000ms，认为光流已故障，放弃光流控制
      {
        opt_data.ctrl_valid=0;
      }
    }
    else//光流数据正常
    {
      opt_data.ctrl_valid=1;
    }
  }
  else state = 0;
}



void opticalflow_receive_anl(uint8_t *buf,uint8_t num)
{
  uint8_t check = *(buf+2);
  for(uint8_t i=3;i<12;i++)  check^=*(buf+i);
  if(!(*(buf)==0xfe&&*(buf+1)==0x0a))	return;//不满足帧头条件
  if(!(*(buf+num-1)==0x55)) 	return;//不满足帧尾条件
  if(!(check==*(buf+num-2))) return;//不满足和校验条件
  
  opt_data.x= (int16_t)(buf[3]<<8|buf[2]);
  opt_data.y=-(int16_t)(buf[5]<<8|buf[4]);
  opt_data.dt=(uint16_t)(buf[7]<<8|buf[6]);
  opt_data.ground_distance=(uint16_t)(buf[9]<<8)|buf[8];
  opt_data.qual=buf[10]; 
  opt_data.version=buf[11];
  
  opt_data.update=1;
  
  if(other_params.params.opticalcal_type==2)  return;
  //
  tofsense_data.distance=opt_data.ground_distance*0.1f;//cm		
  tofsense_data.update_flag=1;
  if(opt_data.version>=50)
  {
    if(tofsense_data.distance<1200&&tofsense_data.distance>1.0f)	tofsense_data.valid=1;
    else tofsense_data.valid=0;
  }
  else tofsense_data.valid=0;
  tofsense_data.period_ms=optical_t.period_int;		
}





float opticalflow_rotate_complementary_filter(float optflow_gyro,float gyro,uint8_t valid)
{
  float optflow_gyro_filter=0;
  //optflow_gyro_filter=optflow_gyro-constrain_float(gyro,-ABS(optflow_gyro),ABS(optflow_gyro));
  optflow_gyro_filter=optflow_gyro-constrain_float(gyro,-8.0f,8.0f);		
  return optflow_gyro_filter;
}


filter_parameter opticalflow_filter_parameter,gyro_sync_filter_parameter;
static filter_buffer opticalflow_filter_buffer[2],gyro_sync_filter_buffer[2];
void opticalflow_pretreat(void)
{
  opt_data.gyro_filter.x=butterworth(-flymaple.gyro_dps.y*DEG2RAD,&gyro_sync_filter_buffer[0],&gyro_sync_filter_parameter);
  opt_data.gyro_filter.y=butterworth( flymaple.gyro_dps.x*DEG2RAD,&gyro_sync_filter_buffer[1],&gyro_sync_filter_parameter);
  
  if(opt_data.update==0) return;
  opt_data.update=0;
  opt_data.flow.x=opt_data.x/200.0f;//光流角速度rad/s
  opt_data.flow.y=opt_data.y/200.0f;//光流角速度rad/s
  opt_data.ms=opt_data.dt*0.001f;
  if(opt_data.qual==0xF5) opt_data.valid=1;
  else opt_data.valid=0;	
  opt_data.flow_filter.x=butterworth(opt_data.flow.x,&opticalflow_filter_buffer[0],&opticalflow_filter_parameter);
  opt_data.flow_filter.y=butterworth(opt_data.flow.y,&opticalflow_filter_buffer[1],&opticalflow_filter_parameter);
  
  //旋转校正
  opt_data.flow_correct.x=opticalflow_rotate_complementary_filter(opt_data.flow_filter.x,opt_data.gyro_filter.x,opt_data.valid);
  opt_data.flow_correct.y=opticalflow_rotate_complementary_filter(opt_data.flow_filter.y,opt_data.gyro_filter.y,opt_data.valid);
  opt_data.is_okay=1;//数据就位标志
}








void uart4_init(uint32_t baud)
{
  //uart_init(UART_4,baud,UART4_TX_P14_1,UART4_RX_P14_0);              
  //uart_rx_interrupt(UART_4, 1);// 使能对应串口接收中断
  //interrupt_set_priority(CPUIntIdx6_IRQn, 2);
  uart_init_intidx(UART_4,baud,UART4_TX_P14_1,UART4_RX_P14_0,uart4_user_isr,uart4_irq_chl_pre_priority,uart4_irq_chl_sub_priority);              
  //uart_rx_interrupt(UART_4, 1);// 使能对应串口接收中断   
}


void uart4_send_byte(unsigned char tx_buf)
{
  Cy_SCB_WriteTxFifo(SCB2,tx_buf);	
  while(Cy_SCB_IsTxComplete(SCB2) == 0);
}


void uart4_send_bytes(unsigned char *buf, int len)
{
  while(len--)
  {
    Cy_SCB_WriteTxFifo(SCB2,*buf);	
    while(Cy_SCB_IsTxComplete(SCB2) == 0);
    buf++;
  }
}


void UART4_IRQHandler(void)
{ 
  if(Cy_SCB_GetRxInterruptMask(SCB2) & CY_SCB_UART_RX_NOT_EMPTY)// 串口接收中断
  {
    if(Cy_SCB_GetNumInRxFifo(SCB2))
    {
      uint8_t ch  = (uint8)Cy_SCB_ReadRxFifo(SCB2);
      lc307_opticalflow_prase(ch);      
    }
    Cy_SCB_ClearRxInterrupt(SCB2, CY_SCB_UART_RX_NOT_EMPTY);              // 清除接收中断标志位
  }
}


uint8_t config_init_uart(void);
void uart4_optical_flow_init(void)
{
  if(other_params.params.opticalcal_type==0)//t101p
  {
    set_cutoff_frequency(50,15,&opticalflow_filter_parameter);//50/20
    set_cutoff_frequency(200,5,&gyro_sync_filter_parameter);//200/6,50/6  
    uart4_init(115200);
    uart_rx_interrupt(UART_4, 1);// 使能对应串口接收中断 
  }
  else if(other_params.params.opticalcal_type==1)//t201
  {
    set_cutoff_frequency(50,15,&opticalflow_filter_parameter);//50/20
    set_cutoff_frequency(200,5,&gyro_sync_filter_parameter);//200/6,50/6
    uart4_init(115200);
    uart_rx_interrupt(UART_4, 1);// 使能对应串口接收中断 
  }
  else if(other_params.params.opticalcal_type==2)//lc307
  {
    set_cutoff_frequency(50,20,&opticalflow_filter_parameter);//50/20
    set_cutoff_frequency(200,6,&gyro_sync_filter_parameter);//200/6,50/6
    uart4_init(19200); 
    config_init_uart(); 
    uart_rx_interrupt(UART_4, 1);// 使能对应串口接收中断 
  }
}

/************************************************************************************/
#define SENSOR_IIC_ADDR 0xdc
const static uint8_t tab_focus[4] = {0x96,0x26,0xbc,0x50};		
const static uint8_t sensor_cfg[]={
  //地址, 数据
  0x12, 0x80, 
  0x11, 0x30, 
  0x1b, 0x06, 
  0x6b, 0x43, 
  0x12, 0x20, 
  0x3a, 0x00, 
  0x15, 0x02, 
  0x62, 0x81, 
  0x08, 0xa0, 
  0x06, 0x68, 
  0x2b, 0x20, 
  0x92, 0x25, 
  0x27, 0x97, 
  0x17, 0x01, 
  0x18, 0x79, 
  0x19, 0x00, 
  0x1a, 0xa0, 
  0x03, 0x00, 
  0x13, 0x00, 
  0x01, 0x13, 
  0x02, 0x20, 
  0x87, 0x16, 
  0x8c, 0x01, 
  0x8d, 0xcc, 
  0x13, 0x07, 
  0x33, 0x10, 
  0x34, 0x1d, 
  0x35, 0x46, 
  0x36, 0x40, 
  0x37, 0xa4, 
  0x38, 0x7c, 
  0x65, 0x46, 
  0x66, 0x46, 
  0x6e, 0x20, 
  0x9b, 0xa4, 
  0x9c, 0x7c, 
  0xbc, 0x0c, 
  0xbd, 0xa4, 
  0xbe, 0x7c, 
  0x20, 0x09, 
  0x09, 0x03, 
  0x72, 0x2f, 
  0x73, 0x2f, 
  0x74, 0xa7, 
  0x75, 0x12, 
  0x79, 0x8d, 
  0x7a, 0x00, 
  0x7e, 0xfa, 
  0x70, 0x0f, 
  0x7c, 0x84, 
  0x7d, 0xba, 
  0x5b, 0xc2, 
  0x76, 0x90, 
  0x7b, 0x55, 
  0x71, 0x46, 
  0x77, 0xdd, 
  0x13, 0x0f, 
  0x8a, 0x10, 
  0x8b, 0x20, 
  0x8e, 0x21, 
  0x8f, 0x40, 
  0x94, 0x41, 
  0x95, 0x7e, 
  0x96, 0x7f, 
  0x97, 0xf3, 
  0x13, 0x07, 
  0x24, 0x58, 
  0x97, 0x48, 
  0x25, 0x08, 
  0x94, 0xb5, 
  0x95, 0xc0, 
  0x80, 0xf4, 
  0x81, 0xe0, 
  0x82, 0x1b, 
  0x83, 0x37, 
  0x84, 0x39, 
  0x85, 0x58, 
  0x86, 0xff, 
  0x89, 0x15, 
  0x8a, 0xb8, 
  0x8b, 0x99, 
  0x39, 0x98, 
  0x3f, 0x98, 
  0x90, 0xa0, 
  0x91, 0xe0, 
  0x40, 0x20, 
  0x41, 0x28, 
  0x42, 0x26, 
  0x43, 0x25, 
  0x44, 0x1f, 
  0x45, 0x1a, 
  0x46, 0x16, 
  0x47, 0x12, 
  0x48, 0x0f, 
  0x49, 0x0d, 
  0x4b, 0x0b, 
  0x4c, 0x0a, 
  0x4e, 0x08, 
  0x4f, 0x06, 
  0x50, 0x06, 
  0x5a, 0x56, 
  0x51, 0x1b, 
  0x52, 0x04, 
  0x53, 0x4a, 
  0x54, 0x26, 
  0x57, 0x75, 
  0x58, 0x2b, 
  0x5a, 0xd6, 
  0x51, 0x28, 
  0x52, 0x1e, 
  0x53, 0x9e, 
  0x54, 0x70, 
  0x57, 0x50, 
  0x58, 0x07, 
  0x5c, 0x28, 
  0xb0, 0xe0, 
  0xb1, 0xc0, 
  0xb2, 0xb0, 
  0xb3, 0x4f, 
  0xb4, 0x63, 
  0xb4, 0xe3, 
  0xb1, 0xf0, 
  0xb2, 0xa0, 
  0x55, 0x00, 
  0x56, 0x40, 
  0x96, 0x50, 
  0x9a, 0x30, 
  0x6a, 0x81, 
  0x23, 0x33, 
  0xa0, 0xd0, 
  0xa1, 0x31, 
  0xa6, 0x04, 
  0xa2, 0x0f, 
  0xa3, 0x2b, 
  0xa4, 0x0f, 
  0xa5, 0x2b, 
  0xa7, 0x9a, 
  0xa8, 0x1c, 
  0xa9, 0x11, 
  0xaa, 0x16, 
  0xab, 0x16, 
  0xac, 0x3c, 
  0xad, 0xf0, 
  0xae, 0x57, 
  0xc6, 0xaa, 
  0xd2, 0x78, 
  0xd0, 0xb4, 
  0xd1, 0x00, 
  0xc8, 0x10, 
  0xc9, 0x12, 
  0xd3, 0x09, 
  0xd4, 0x2a, 
  0xee, 0x4c, 
  0x7e, 0xfa, 
  0x74, 0xa7, 
  0x78, 0x4e, 
  0x60, 0xe7, 
  0x61, 0xc8, 
  0x6d, 0x70, 
  0x1e, 0x39, 
  0x98, 0x1a
};

static void sensor_config_uart(uint8_t dat)
{
  uart4_send_byte(dat);
}

uint16_t lc306_ready_cnt=0;
uint8_t config_init_uart(void)
{
  uint16_t i;
  uint16_t len ;
  uint8_t recv[3];
  int recv_cnt;		
  delay_ms(100);		
  len = sizeof(sensor_cfg);	
  sensor_config_uart(0xAA);//0xAA指令
  sensor_config_uart(0xAB);//0xAB指令		 
  sensor_config_uart(tab_focus[0]);		
  sensor_config_uart(tab_focus[1]);
  sensor_config_uart(tab_focus[2]);
  sensor_config_uart(tab_focus[3]);
  sensor_config_uart(tab_focus[0]^tab_focus[1]^tab_focus[2]^tab_focus[3]);			 		 
  recv_cnt = 0;
  lc306_ready_cnt=65535;
  while(recv_cnt<3)  //如接收不到模块返回的三个数，可延时10ms从0xAA指令开始重新配置
  {
    while(!Cy_SCB_GetNumInRxFifo(SCB2))
    {
      lc306_ready_cnt--;
      if(lc306_ready_cnt==0) return 0;
    };
    recv[recv_cnt++] = (uint8)Cy_SCB_ReadRxFifo(SCB2);							
  }			
  if(((recv[0]^recv[1]) == recv[2]) & (recv[1] == 0x00)) {;}
  //printf("AB Command configuration successconfig succefful\n");		
  
  for(i=0;i<len;i+=2)//0xBB
  {
    sensor_config_uart(0xBB);		 
    sensor_config_uart(SENSOR_IIC_ADDR);		
    sensor_config_uart(sensor_cfg[i]);
    sensor_config_uart(sensor_cfg[i+1]);
    sensor_config_uart(SENSOR_IIC_ADDR^sensor_cfg[i]^sensor_cfg[i+1]);	 
    recv_cnt = 0;
    lc306_ready_cnt=65535;
    while(recv_cnt<3)  //如接收不到模块返回的三个数，可延时1ms重新发送0xBB指令
    {
      while(!Cy_SCB_GetNumInRxFifo(SCB2))
      {
        lc306_ready_cnt--;
        if(lc306_ready_cnt==0) return 0;      
      };
      recv[recv_cnt++] = (uint8)Cy_SCB_ReadRxFifo(SCB2);							
    }			
    if(((recv[0]^recv[1]) == recv[2]) & (recv[1] == 0x00)){;}
    //printf("BB Command configuration successconfig succefful\n");								 
  }	 		 
  sensor_config_uart(0xDD);//0xDD		
  //printf("Configuration success\n");
  return 1;
}	