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
#include <stdlib.h>
#include "datatype.h"
#include "quene.h"


uint8_t is_full(linear_quene* ptr)//判断队列是否已满
{
  if(ptr->rear==ptr->len-1)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

uint8_t is_empty(linear_quene* ptr)//判断队列是否为空
{
  if(ptr->rear==ptr->front)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void enquene(linear_quene* ptr,qdatatype value)//入列
{
  if(is_full(ptr)==0)//队列数据已满
  {		
    ptr->rear = ptr->rear + 1;
    ptr->data_ptr[ptr->rear] = value;
  }
  else//避免数组溢出
  {
    ptr->rear=-1;
  }
}


qdatatype dequene(linear_quene* ptr)//查询队列中数据
{
  qdatatype value;
  ptr->front = ptr->front + 1;
  value = ptr->data_ptr[ptr->front];
  return value;
}

void initquene(linear_quene *ptr,qdatatype *pdata,uint16_t len)
{
  ptr->data_ptr=pdata;
  ptr->front = -1;
  ptr->rear = -1;
  ptr->len=len;
}

void resetquene(linear_quene *ptr)
{
  ptr->front = -1;
  ptr->rear = -1;
}

/*****************************************************************************/
void ringbuff_init(ring_buff_t *ptr)
{
  //初始化相关信息
  ptr->head = 0;
  ptr->tail = 0;
  ptr->lenght = 0;
}

uint8_t write_ringbuff(uint8_t data,ring_buff_t *ptr)
{
  if(ptr->lenght >= RINGBUFF_LEN) //判断缓冲区是否已满
  {
    return 0;
  }
  ptr->ring_buff[ptr->tail]=data;
  ptr->tail = (ptr->tail+1)%RINGBUFF_LEN;//防止越界非法访问  //ringBuff.Tail++;
  ptr->lenght++;
  return 1;
}

uint8_t read_ringbuff(uint8_t *rdata,ring_buff_t *ptr)
{
  if(ptr->lenght == 0)//判断非空
  {
    return 0;
  }
  *rdata = ptr->ring_buff[ptr->head];//先进先出FIFO，从缓冲区头出
  //ringBuff.Head++;
  ptr->head = (ptr->head+1)%RINGBUFF_LEN;//防止越界非法访问
  ptr->lenght--;
  return 1;
}

/**************************************************************************/
void ringbuf_write(unsigned char data,ring_buff_t *ptr,uint16_t len)
{
  ptr->ring_buff[ptr->tail]=data;//从尾部追加
  if(++ptr->tail>=len)//尾节点偏移
    ptr->tail=0;//大于数组最大长度 归零 形成环形队列  
  if(ptr->tail==ptr->head)//如果尾部节点追到头部节点，则修改头节点偏移位置丢弃早期数据
  {
    if((++ptr->head)>=len)  
      ptr->head=0; 
  }
}

uint8_t ringbuf_read(unsigned char* pData,ring_buff_t *ptr)
{
  if(ptr->head==ptr->tail)  return 1;//如果头尾接触表示缓冲区为空
  else 
  {  
    *pData=ptr->ring_buff[ptr->head];//如果缓冲区非空则取头节点值并偏移头节点
    if((++ptr->head)>=RINGBUFF_LEN)   ptr->head=0;
    return 0;//返回0，表示读取数据成功
  }
}

