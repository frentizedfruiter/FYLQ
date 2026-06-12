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
#include "drv_at24cxx.h"
#include "schedule.h"

#define at24cxx_scl_pin  AT24CXX_I2C_SCL_PIN
#define at24cxx_sda_pin  AT24CXX_I2C_SDA_PIN

/******************************************************************************************/
//AT24C02的地址是256个字节，地址从0-255即 0x000 到 0xFFH
uint8_t AT24CXX_Check(void);  //检查器件


#define AT24C01 127
#define AT24C02 255
#define AT24C04 511
#define AT24C08 1023
#define AT24C16 2047
#define AT24C32 4095
#define AT24C64 8191
#define AT24C128 16383
#define AT24C256 32767

#define EE_TYPE AT24C16
#define AT24C0X_IIC_ADDRESS  0xA0//1 0 1 0 A2 A1 A0 R/W


#define SOFT_READ_SDA         ((gpio_get_level(at24cxx_sda_pin)!=0)?1:0)
#define SOFT_IIC_SDA_1        gpio_high(at24cxx_sda_pin)//SDA  		 
#define SOFT_IIC_SDA_0        gpio_low(at24cxx_sda_pin)	//SDA
#define SOFT_IIC_SCL_1        gpio_high(at24cxx_scl_pin)//SCL  		
#define SOFT_IIC_SCL_0        gpio_low(at24cxx_scl_pin)	//SCL  	


//IIC延时函数
static void SOFT_IIC_DLY(void)
{
  _delay_us(10);
}


//初始化IIC
static void IIC_Init(void)
{
  gpio_init(at24cxx_scl_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
  gpio_init(at24cxx_sda_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
  
  SOFT_IIC_SDA_1;
  SOFT_IIC_SCL_1;
}

//IO方向设置

static void SOFT_SDA_OUT(void)     
{
  gpio_set_dir(at24cxx_sda_pin, GPO, GPO_PUSH_PULL);
}

static void SOFT_SDA_IN(void)        
{                           
  gpio_set_dir(at24cxx_sda_pin, GPI, GPI_FLOATING_IN);
}


//产生IIC起始信号
static void SOFT_IIC_Start(void)
{
  SOFT_SDA_OUT(); //sda线输出
  /*空闲状态*/
  SOFT_IIC_SDA_1;
  SOFT_IIC_SCL_1;
  /* START信号: 当SCL为高时, SDA从高变成低, 表示起始信号 */
  SOFT_IIC_DLY();
  SOFT_IIC_SDA_0; //START:when CLK is high,DATA change form high to low
  SOFT_IIC_DLY();
  SOFT_IIC_SCL_0; //钳住I2C总线，准备发送或接收数据
  /* 时钟低电平，数据传输无效，持有SDA，钳住数据 */
}

//产生IIC停止信号
static void SOFT_IIC_Stop(void)
{
  SOFT_SDA_OUT(); //sda线输出
  SOFT_IIC_SCL_0;
  SOFT_IIC_SDA_0; //STOP:when CLK is high DATA change form low to high
  SOFT_IIC_DLY();
  SOFT_IIC_SCL_1;
  SOFT_IIC_SDA_1; //发送I2C总线结束信号
  SOFT_IIC_DLY();
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
static uint8_t SOFT_IIC_Wait_Ack(void)
{
  uint8_t ucErrTime = 0;
  SOFT_IIC_SCL_0;//继主机发送一字节后的前低电平1/4时钟
  SOFT_IIC_SDA_1;/*主机释放SDA线(此时外部器件可以拉低SDA线)*/
  SOFT_IIC_DLY();
  SOFT_SDA_IN();//SDA设置为输入
  SOFT_IIC_SCL_1;//SCL=1, 此时从机可以返回ACK
  SOFT_IIC_DLY();
  while (SOFT_READ_SDA)
  {
    ucErrTime++;
    if (ucErrTime > 250)
    {
      SOFT_IIC_Stop();
      return 1;
    }
  }
  SOFT_IIC_SCL_0; //时钟输出0
  return 0;
}


//产生ACK应答
static void SOFT_IIC_Ack(void)
{
  SOFT_IIC_SCL_0;
  SOFT_SDA_OUT();
  SOFT_IIC_SDA_0;
  SOFT_IIC_DLY();
  SOFT_IIC_SCL_1;
  SOFT_IIC_DLY();
  SOFT_IIC_SCL_0;
}
//不产生ACK应答
static void SOFT_IIC_NAck(void)
{
  SOFT_IIC_SCL_0;
  SOFT_SDA_OUT();
  SOFT_IIC_SDA_1;
  SOFT_IIC_DLY();
  SOFT_IIC_SCL_1;
  SOFT_IIC_DLY();
  SOFT_IIC_SCL_0;
}
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
static void SOFT_IIC_Send_Byte(uint8_t txd)
{
  uint8_t t;
  SOFT_SDA_OUT();
  for (t = 0; t < 8; t++)
  {
    SOFT_IIC_SCL_0; //拉低时钟开始数据传输
    //时钟线SCL拉低电平期间，才允许数据线SDA电平变化
    if((txd & 0x80)>>7)  SOFT_IIC_SDA_1;
    else SOFT_IIC_SDA_0;				
    txd <<= 1;//移位操作，先发送高位
    SOFT_IIC_SCL_1;//时钟线SCL拉高，上升延触发SDA数据位发送
    SOFT_IIC_DLY();
    SOFT_IIC_SCL_0;
    SOFT_IIC_DLY();
  }
}


//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
static uint8_t SOFT_IIC_Read_Byte(unsigned char ack)
{
  unsigned char i, receive = 0;
  SOFT_SDA_IN(); //SDA设置为输入
  for (i = 0; i < 8; i++)
  {
    SOFT_IIC_SCL_0;
    SOFT_IIC_DLY();
    SOFT_IIC_SCL_1;
    receive <<= 1;
    if (SOFT_READ_SDA)
      receive++;
    SOFT_IIC_DLY();
  }
  if (!ack)
    SOFT_IIC_NAck(); //发送nACK
  else
    SOFT_IIC_Ack(); //发送ACK
  return receive;
}


//初始化IIC接口
void AT24CXX_Init(void)
{
  IIC_Init();
  while(AT24CXX_Check()) //检测不到24c02
  {
    delay_ms(700);
  }
  //AT24CXX_Erase_All();
}


uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr)
{
  uint8_t temp = 0;
  SOFT_IIC_Start();
  if (EE_TYPE > AT24C16)
  {
    SOFT_IIC_Send_Byte(AT24C0X_IIC_ADDRESS); //发送写命令
    SOFT_IIC_Wait_Ack();
    SOFT_IIC_Send_Byte(ReadAddr >> 8); //发送高地址
    //SOFT_IIC_Wait_Ack();
  }
  else  SOFT_IIC_Send_Byte(AT24C0X_IIC_ADDRESS + ((ReadAddr / 256) << 1)); //发送器件地址0XA0,写数据
  SOFT_IIC_Wait_Ack();
  SOFT_IIC_Send_Byte(ReadAddr % 256); //发送低地址
  SOFT_IIC_Wait_Ack();
  
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(AT24C0X_IIC_ADDRESS+1); //进入接收模式
  SOFT_IIC_Wait_Ack();
  temp = SOFT_IIC_Read_Byte(0); 
  SOFT_IIC_Stop();//产生一个停止条件
  return temp;
}

//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址
//DataToWrite:要写入的数据
void AT24CXX_WriteOneByte(uint16_t WriteAddr, uint8_t DataToWrite)
{
  SOFT_IIC_Start();
  if (EE_TYPE > AT24C16)
  {
    SOFT_IIC_Send_Byte(AT24C0X_IIC_ADDRESS); //发送写命令
    SOFT_IIC_Wait_Ack();
    SOFT_IIC_Send_Byte(WriteAddr >> 8); //发送高地址
  }
  else
  {
    SOFT_IIC_Send_Byte(AT24C0X_IIC_ADDRESS + ((WriteAddr / 256) << 1)); //发送器件地址0XA0,写数据
  }
  SOFT_IIC_Wait_Ack();
  SOFT_IIC_Send_Byte(WriteAddr % 256); //发送低地址
  SOFT_IIC_Wait_Ack();
  SOFT_IIC_Send_Byte(DataToWrite); //发送字节
  SOFT_IIC_Wait_Ack();
  SOFT_IIC_Stop(); //产生一个停止条件
  delay_ms(5);//10
}



void AT24CXX_Erase_All(void)
{
  for(uint16_t i=0;i<=EE_TYPE;i++)
  {
    AT24CXX_WriteOneByte(i,0xff);
  }
}	
//在AT24CXX里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void AT24CXX_WriteLenByte(uint16_t WriteAddr, uint32_t DataToWrite, uint16_t Len)
{
  uint8_t t;
  for (t = 0; t < Len; t++)
  {
    AT24CXX_WriteOneByte(WriteAddr + t, (DataToWrite >> (8 * t)) & 0xff);
  }
}

//在AT24CXX里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址
//返回值     :数据
//Len        :要读出数据的长度2,4
uint32_t AT24CXX_ReadLenByte(uint16_t ReadAddr, uint16_t Len)
{
  uint8_t t;
  uint32_t temp = 0;
  for (t = 0; t < Len; t++)
  {
    temp <<= 8;
    temp += AT24CXX_ReadOneByte(ReadAddr + Len - t - 1);
  }
  return temp;
}

//在AT24CXX里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void AT24CXX_Read(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead)
{
  while (NumToRead)
  {
    *pBuffer++ = AT24CXX_ReadOneByte(ReadAddr++);
    NumToRead--;
  }
}
//在AT24CXX里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址 对24c02为0~255
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
static void AT24CXX_Write(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite)
{
  while (NumToWrite--)
  {
    AT24CXX_WriteOneByte(WriteAddr, *pBuffer);
    WriteAddr++;
    pBuffer++;
  }
}

//检查AT24CXX是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
#define at24cx_check_address 1600//1600  240
#define at24cx_end_address 	 2047//2047  255 
uint8_t AT24CXX_Check(void)
{
  uint8_t temp;
  temp = AT24CXX_ReadOneByte(at24cx_end_address); //避免每次开机都写AT24CXX
  if (temp == 0X55)	return 0;
  else //排除第一次初始化的情况
  {
    //AT24CXX_Erase_All();//先全部擦除
    AT24CXX_WriteOneByte(at24cx_end_address, 0X55);
    temp = AT24CXX_ReadOneByte(at24cx_end_address);
    if (temp == 0X55)	return 0;
  }
  return 1;
}


/////////////////////////////////////////////////////////////////////////////////////////

void AT24CXX_Write_FData(float *data,uint32_t addr)
{
  //  uint8_t tempwr[4];
  //  tempwr[0]=(*(uint32_t *)(data))&0xff;
  //  tempwr[1]=(*(uint32_t *)(data))>>8;
  //  tempwr[2]=(*(uint32_t *)(data))>>16;
  //  tempwr[3]=(*(uint32_t *)(data))>>24;
  //  AT24CXX_Write(addr,tempwr,4);
  FloatByteInt temp;
  temp.Fdata=*data;
  AT24CXX_Write(addr,temp.Bdata,4);
}

void AT24CXX_Read_FData(float *data,uint32_t addr)
{
  FloatByteInt temp;
  AT24CXX_Read(addr,temp.Bdata,4);
  *data=temp.Fdata;
}


void EEPROM_Write_N_Data(uint32_t WriteAddress,float *Buf,int32_t length)
{
  for(uint16_t i=0;i<length;i++)
  {
    AT24CXX_Write_FData((Buf+i),WriteAddress+4*i);
  }
}

int EEPROM_Read_N_Data(uint32_t ReadAddress,float *ReadBuf,int32_t ReadNum)
{
  for(uint16_t i=0;i<ReadNum;i++)
  {
    AT24CXX_Read_FData((ReadBuf+i),ReadAddress+4*i);
  }
  return 1;
}



/*********************************************************************/
void EEPROMRead_One_AT24Cxx(uint32_t *pui32Data,uint32_t ui32Address)
{
  FloatByteInt temp;
  AT24CXX_Read(ui32Address,temp.Bdata,4);
  *pui32Data=temp.Idata;
}

void EEPROMWrite_One_AT24Cxx(uint32_t *pui32Data,uint32_t ui32Address)
{
  FloatByteInt temp;
  temp.Idata=*pui32Data;
  AT24CXX_Write(ui32Address,temp.Bdata,4);
}

void EEPROMRead_AT24Cxx(uint32_t *pui32Data, uint32_t ui32Address, uint32_t ui32Count)
{
  for(uint16_t i=0;i<ui32Count;i++)
  {
    EEPROMRead_One_AT24Cxx((pui32Data+i),4*ui32Address+4*i);
  }
}

void EEPROMProgram_AT24Cxx(uint32_t *pui32Data, uint32_t ui32Address, uint32_t ui32Count)
{
  for(uint16_t i=0;i<ui32Count;i++)
  {
    EEPROMWrite_One_AT24Cxx((pui32Data+i),4*ui32Address+4*i);
  }
}
