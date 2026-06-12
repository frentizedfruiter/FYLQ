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




#include "mt01.h"
#include "schedule.h"
#include "drv_tofsense.h"
/*
说明： 用户使用micolink_decode作为串口数据处理函数即可

距离有效值最小为10(mm),为0说明此时距离值不可用
*/

bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data);

void micolink_decode(uint8_t data)
{
  static MICOLINK_MSG_t msg;
  static systime _t;
  if(micolink_parse_char(&msg, data) == false) return;
  switch(msg.msg_id)
  {
  case MICOLINK_MSG_ID_RANGE_SENSOR:
    {
      MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
      memcpy(&payload, msg.payload, msg.len);
      
      get_systime(&_t);
      tofsense_data.dis        			= payload.distance;
      tofsense_data.signal_strength = payload.strength;
      tofsense_data.range_precision = payload.precision;
      tofsense_data.dis_status    	= payload.tof_status;
      tofsense_data.id          		= payload.id;
      
      tofsense_data.distance=tofsense_data.dis/10.0f;//cm		
      tofsense_data.update_flag=1;
      if(tofsense_data.dis_status==1&&tofsense_data.distance<750) tofsense_data.valid=1;
      else tofsense_data.valid=0;
      tofsense_data.period_ms=_t.period_int;			    
    }
    break;
  default:{;}
  }
}

bool micolink_check_sum(MICOLINK_MSG_t* msg)
{
  uint8_t length = msg->len + 6;
  uint8_t temp[MICOLINK_MAX_LEN];
  uint8_t checksum = 0;
  
  memcpy(temp, msg, length);
  
  for(uint8_t i=0; i<length; i++)
  {
    checksum += temp[i];
  }
  
  if(checksum == msg->checksum)
    return true;
  else
    return false;
}


uint32_t miss_cnt;
bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data)
{
  switch(msg->status)
  {
  case 0:     //帧头
    if(data == MICOLINK_MSG_HEAD)
    {
      msg->head = data;
      msg->status++;
    }
    break;
    
  case 1:     // 设备ID
    msg->dev_id = data;
    msg->status++;
    break;
    
  case 2:     // 系统ID
    msg->sys_id = data;
    msg->status++;
    break;
    
  case 3:     // 消息ID
    msg->msg_id = data;
    msg->status++;
    break;
    
  case 4:     // 包序列
    msg->seq = data;
    msg->status++;
    break;
    
  case 5:     // 负载长度
    msg->len = data;
    if(msg->len == 0)
      msg->status += 2;
    else if(msg->len > MICOLINK_MAX_PAYLOAD_LEN)
      msg->status = 0;
    else
      msg->status++;
    break;
    
  case 6:     // 数据负载接收
    msg->payload[msg->payload_cnt++] = data;
    if(msg->payload_cnt == msg->len)
    {
      msg->payload_cnt = 0;
      msg->status++;
    }
    break;
    
  case 7:     // 帧校验
    msg->checksum = data;
    msg->status = 0;
    if(micolink_check_sum(msg))
    {
      return true;
    }
    
  default:
    msg->status = 0;
    msg->payload_cnt = 0;
    miss_cnt++;
    break;
  }
  
  return false;
}

