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
#include "stdbool.h"
#include "drv_at24cxx.h"
#include "drv_w25qxx.h"
#include "schedule.h"




#define W25QXX_Read_N_Data(ReadAddress,ReadBuf,ReadNum)   EEPROM_Read_N_Data(ReadAddress,ReadBuf,ReadNum)
#define W25QXX_Write_N_Data(WriteAddress,ReadBuf,ReadNum)   EEPROM_Write_N_Data(WriteAddress,ReadBuf,ReadNum)



/**********************************************************************************/

void ReadFlashParameterALL(volatile FLIGHT_PARAMETER *WriteData)
{
  uint32_t ReadAddress = (uint32_t)(PARAMETER_TABLE_STARTADDR_EEPROM);
  W25QXX_Read_N_Data(ReadAddress,(float *)(&WriteData->parameter_table),FLIGHT_PARAMETER_TABLE_NUM);
}

uint8_t ReadFlashParameterOne(uint16_t Label,float *ReadData)
{
  uint32_t ReadAddress = (uint32_t)(PARAMETER_TABLE_STARTADDR_EEPROM+Label*4);
  return W25QXX_Read_N_Data(ReadAddress,(float *)ReadData,1);
}

uint8_t ReadFlashParameterTwo(uint16_t Label,float *ReadData1,float *ReadData2)
{
  uint32_t ReadAddress = (uint32_t)(PARAMETER_TABLE_STARTADDR_EEPROM+Label*4);
  float readtemp[2]; 
  uint8_t flag=W25QXX_Read_N_Data(ReadAddress,(float *)(readtemp),2);
  if(flag==1)
  {
    *ReadData1=readtemp[0];
    *ReadData2=readtemp[1];
  }
  return flag;
}

uint8_t ReadFlashParameterThree(uint16_t Label,float *ReadData1,float *ReadData2,float *ReadData3)
{
  uint32_t ReadAddress = (uint32_t)(PARAMETER_TABLE_STARTADDR_EEPROM+Label*4);
  float readtemp[3]; 
  uint8_t flag=W25QXX_Read_N_Data(ReadAddress,(float *)(readtemp),3);
  if(flag==1)
  {
    *ReadData1=readtemp[0];
    *ReadData2=readtemp[1];
    *ReadData3=readtemp[2];
  }
  return flag;
}

void WriteFlashParameter(uint16_t Label,
                         float WriteData,
                         volatile FLIGHT_PARAMETER *Table)
{
  __set_PRIMASK(1);//关总中断
  Table->parameter_table[Label]=WriteData;//将需要更改的字段赋新值	
  uint32_t WriteAddress = (uint32_t)(PARAMETER_TABLE_STARTADDR_EEPROM+Label*4);
  W25QXX_Write_N_Data(WriteAddress,(float *)(&Table->parameter_table[Label]),1);
  __set_PRIMASK(0);//开总中断
}

void WriteFlashParameter_Two(uint16_t Label,
                             float WriteData1,
                             float WriteData2,
                             volatile FLIGHT_PARAMETER *Table)
{
  __set_PRIMASK(1);//关总中断
  uint32_t WriteAddress = (uint32_t)(PARAMETER_TABLE_STARTADDR_EEPROM+Label*4);
  Table->parameter_table[Label]=WriteData1;//将需要更改的字段赋新值
  Table->parameter_table[Label+1]=WriteData2;//将需要更改的字段赋新值
  W25QXX_Write_N_Data(WriteAddress,(float *)(&Table->parameter_table[Label]),2);
  __set_PRIMASK(0);//开总中断
}

void WriteFlashParameter_Three(uint16_t Label,
                               float WriteData1,
                               float WriteData2,
                               float WriteData3,
                               volatile FLIGHT_PARAMETER *Table)
{
  __set_PRIMASK(1);//关总中断
  uint32_t WriteAddress = (uint32_t)(PARAMETER_TABLE_STARTADDR_EEPROM+Label*4);
  //ReadFlashParameterALL(Table);//先把片区内的所有数据都都出来
  Table->parameter_table[Label]=WriteData1;//将需要更改的字段赋新值
  Table->parameter_table[Label+1]=WriteData2;//将需要更改的字段赋新值
  Table->parameter_table[Label+2]=WriteData3;//将需要更改的字段赋新值
  W25QXX_Write_N_Data(WriteAddress,(float *)(&Table->parameter_table[Label]),3);
  __set_PRIMASK(0);//开总中断
}

FLIGHT_PARAMETER Flight_Params={
  .num=FLIGHT_PARAMETER_TABLE_NUM
};

void flight_read_flash_full(void)    
{
  ReadFlashParameterALL((FLIGHT_PARAMETER *)(&Flight_Params));
  for(uint16_t i=0;i<Flight_Params.num;i++)
  {
    if(isnan(Flight_Params.parameter_table[i])==0)
      Flight_Params.health[i]=true;
  }
}


void Resume_Factory_Setting(void)
{
  AT24CXX_Erase_All();//先全部擦除
  AT24CXX_WriteOneByte(2047, 0X55);
}

