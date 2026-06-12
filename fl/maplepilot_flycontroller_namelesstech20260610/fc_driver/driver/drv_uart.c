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
#include "stdio.h"
#include "string.h"
#include "datatype.h"
#include "quene.h"
#include "schedule.h"
#include "maple_config.h"
#include "parameter_server.h"
#include "reserved_serialport.h"
#include "flymaple_sdk.h"
#include "main.h"
#include "offboard.h"
#include "drv_uart.h"




void uart1_init(uint32_t baud)
{
  uart_init_intidx(UART_1,baud,UART1_TX_P06_1,UART1_RX_P06_0,uart1_user_isr,uart1_irq_chl_pre_priority,uart1_irq_chl_sub_priority);              
  uart_rx_interrupt(UART_1, 1);// 使能对应串口接收中断
}

void uart1_send_bytes(unsigned char *buf, int len)
{
  while(len--)
  {
    Cy_SCB_WriteTxFifo(SCB4,*buf);	
    while(Cy_SCB_IsTxComplete(SCB4) == 0);
    buf++;
  }
}


extern void NCLink_Data_Prase_Prepare(uint8_t data);
void UART1_IRQHandler(void)
{ 
  if(Cy_SCB_GetRxInterruptMask(SCB4) & CY_SCB_UART_RX_NOT_EMPTY)            // 串口接收中断
  {
    if(Cy_SCB_GetNumInRxFifo(SCB4))
    {
      uint8_t ch  = (uint8)Cy_SCB_ReadRxFifo(SCB4);
      NCLink_Data_Prase_Prepare(ch);
    }
    Cy_SCB_ClearRxInterrupt(SCB4, CY_SCB_UART_RX_NOT_EMPTY);              // 清除接收中断标志位
  }	
}


void wust_sendware(unsigned char *wareaddr, int16_t waresize)//山外发送波形
{
#define CMD_WARE     3
  uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};//帧头
  uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};//帧尾
  uart1_send_bytes(cmdf, sizeof(cmdf));
  uart1_send_bytes(wareaddr, waresize);
  uart1_send_bytes(cmdr, sizeof(cmdr));
}


extern ppm ppm_rc;
void Vcan_Send(void)//山外地面站发送
{
  static float DataBuf[8];	
  DataBuf[0]=ppm_rc.data[0];
  DataBuf[1]=ppm_rc.data[1];
  DataBuf[2]=ppm_rc.data[2];
  DataBuf[3]=ppm_rc.data[3];
  DataBuf[4]=ppm_rc.data[4];
  DataBuf[5]=ppm_rc.data[5];
  DataBuf[6]=ppm_rc.data[6];
  DataBuf[7]=ppm_rc.data[7];
  wust_sendware((unsigned char *)DataBuf,sizeof(DataBuf));
}


/****************************************************************************************************************/
ring_buff_t com2_rx_buf;
void uart2_init(uint32_t baud)
{  
  uart_init_intidx(UART_2,baud,UART2_TX_P07_1,UART2_RX_P07_0,uart2_user_isr,uart2_irq_chl_pre_priority,uart2_irq_chl_sub_priority);              
  uart_rx_interrupt(UART_2, 1);// 使能对应串口接收中断  
}

void uart2_send_bytes(unsigned char *buf, int len)
{
  while(len--)
  {
    Cy_SCB_WriteTxFifo(SCB5,*buf);	
    while(Cy_SCB_IsTxComplete(SCB5) == 0);
    buf++;
  }
}

uint8_t uart2_rx_idle_flag=0;
void UART2_IRQHandler(void)
{ 
  switch(other_params.params.reserved_uart)  
  {
  case THIRD_PARTY_STATE:
    {
      if(Cy_SCB_GetRxInterruptMask(SCB5) & CY_SCB_UART_RX_NOT_EMPTY)            // 串口接收中断
      {
        if(Cy_SCB_GetNumInRxFifo(SCB5))
        {
          uint8_t ch  = (uint8)Cy_SCB_ReadRxFifo(SCB5);
          ringbuf_write(ch,&com2_rx_buf,120);
          if(com2_rx_buf.ring_buff[0]!=0XFC)
          {
            com2_rx_buf.head=1;
            com2_rx_buf.tail=0;
          }
          else if(com2_rx_buf.tail==59||com2_rx_buf.tail==119)//单帧数据接收完毕
          {
            uart2_rx_idle_flag=1;
          }
        }
        Cy_SCB_ClearRxInterrupt(SCB5, CY_SCB_UART_RX_NOT_EMPTY);// 清除接收中断标志位
      }
    }
    break;
  case GPS_M8N:
  default:
    {
      if(Cy_SCB_GetRxInterruptMask(SCB5) & CY_SCB_UART_RX_NOT_EMPTY) 
      {
        uint8_t ch=(uint8)Cy_SCB_ReadRxFifo(SCB5);
        ringbuf_write(ch,&com2_rx_buf,200);//往环形队列里面写数据
        if(com2_rx_buf.ring_buff[0]!=0XB5)
        {
          com2_rx_buf.head=1;
          com2_rx_buf.tail=0;
        }
        else if(com2_rx_buf.tail==99||com2_rx_buf.tail==199)
        {
          flymaple.gps_idle_flag=1;
        } 
        Cy_SCB_ClearRxInterrupt(SCB5, CY_SCB_UART_RX_NOT_EMPTY);// 清除接收中断标志位	
      }			
    }
  }
}


/****************************************************************************************************************/
void uart5_init(uint32_t baud)
{
  //uart_init(UART_5,baud,UART5_TX_P02_1,UART5_RX_P02_0);              
  //uart_rx_interrupt(UART_5, 1);// 使能对应串口接收中断
  //interrupt_set_priority(CPUIntIdx6_IRQn, 2);
  uart_init_intidx(UART_5,baud,UART5_TX_P02_1,UART5_RX_P02_0,uart5_user_isr,uart5_irq_chl_pre_priority,uart5_irq_chl_sub_priority);              
  uart_rx_interrupt(UART_5, 1);// 使能对应串口接收中断   
}

void uart5_send_bytes(unsigned char *buf, int len)
{
  while(len--)
  {
    Cy_SCB_WriteTxFifo(SCB7,*buf);	
    while(Cy_SCB_IsTxComplete(SCB7) == 0);
    buf++;
  }
}

void UART5_IRQHandler(void)
{ 
  if(Cy_SCB_GetRxInterruptMask(SCB7) & CY_SCB_UART_RX_NOT_EMPTY)            // 串口接收中断
  {
    if(Cy_SCB_GetNumInRxFifo(SCB7))
    {
      uint8_t ch  = (uint8)Cy_SCB_ReadRxFifo(SCB7);
      //if(other_params.params.inner_uart==0)	sdk_data_receive_prepare_1(ch);
      //else NCLink_Data_Prase_Planner_CMD(ch);
      sdk_data_receive_prepare_2(ch);
    }
    Cy_SCB_ClearRxInterrupt(SCB7, CY_SCB_UART_RX_NOT_EMPTY);              // 清除接收中断标志位
  }
}




void uart_sendbytes(uint32_t port,uint8_t *ubuf, uint32_t len)
{
  volatile stc_SCB_t* scb =SCB0;
  switch(port)
  {
  case 0:scb=SCB0;break;
  case 1:scb=SCB4;break;
  case 2:scb=SCB5;break;
  case 3:scb=SCB3;break;
  case 4:scb=SCB2;break;
  case 5:scb=SCB7;break;
  case 6:scb=SCB1;break;
  default:scb=SCB0;
  }
  while(len--)
  {
    Cy_SCB_WriteTxFifo(scb,*ubuf);	
    while(Cy_SCB_IsTxComplete(scb) == 0);
    ubuf++;
  }
}


void uart_sendbyte(uint32_t port,uint8_t data)
{
  volatile stc_SCB_t* scb =SCB0;
  switch(port)
  {
  case 0:scb=SCB0;break;
  case 1:scb=SCB4;break;
  case 2:scb=SCB5;break;
  case 3:scb=SCB3;break;
  case 4:scb=SCB2;break;
  case 5:scb=SCB7;break;
  case 6:scb=SCB1;break;
  default:scb=SCB0;
  }
  Cy_SCB_WriteTxFifo(scb,data);	
  while(Cy_SCB_IsTxComplete(scb) == 0);
}






















