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
#include "drv_i2c.h"
#include "schedule.h"
#include <stdbool.h>



#define imu_scl_pin  IMU_I2C_SCL_PIN
#define imu_sda_pin  IMU_I2C_SDA_PIN
/*********************************************************************************/


#define sda_out  gpio_set_dir(imu_sda_pin, GPO, GPO_PUSH_PULL)
#define sda_in   gpio_set_dir(imu_sda_pin, GPI, GPI_FLOATING_IN)


#define i2c_read_sda  ((gpio_get_level(imu_sda_pin)!=0)?1:0)
#define sda_high        gpio_high(imu_sda_pin)//SDA  		 
#define sda_low         gpio_low(imu_sda_pin)	//SDA
#define scl_high        gpio_high(imu_scl_pin)//SCL  		
#define scl_low         gpio_low(imu_scl_pin)	//SCL  	

#define  SCL_H         scl_high
#define  SCL_L         scl_low
#define  SDA_H         sda_high 
#define  SDA_L         sda_low 
#define  SDA_read      i2c_read_sda



void mpu_i2cInit(void);



void i2c_gpio_configuration(void)
{
  gpio_init(imu_scl_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
  gpio_init(imu_sda_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
  
  sda_high;
  scl_high;
}

static void delay(uint32_t n)
{
  n = n * 5;//1~100
  while (n--);
}

static void i2c_start(void)
{
  sda_out;
  sda_high;
  ;//delay(1);
  scl_high;
  delay(1);
  sda_low;
  delay(1);
  scl_low;
  delay(1);
}

static void i2c_stop_sensor(void)
{
  sda_out;
  scl_low;
  sda_low;
  delay(1);
  scl_high;
  delay(1);
  sda_high;
}

static void i2c_send_ask_sensor(uint32_t ack)
{
  sda_out;
  scl_low;
  delay(1);
  if(!ack) sda_high;
  else sda_low;
  delay(1);
  scl_high;
  delay(1);
  scl_low;
  delay(1);
  sda_in; //释放总线
}


static void i2c_recv_ask_sensor(void)
{
  uint16_t cnt=0;
  sda_in;
  scl_high;
  while(i2c_read_sda)
  {
    cnt++;
    if(cnt>5)
    {
      i2c_stop_sensor();
      cnt = 0;
      break;
    }
  }
  scl_low;
  delay(1);
}

static void i2c_sendbyte_sensor(uint32_t dat)
{
  uint8_t i;
  sda_out;
  i = 8;
  //		scl_low;
  //		delay(1);
  while (i--)
  {
    if ((dat & 0x80))
    {
      sda_high;
    }
    else
    {
      sda_low;
    }
    dat <<= 1;
    delay(1);
    scl_high;
    delay(1);
    scl_low;
    delay(1);
  }
  sda_in; //释放总线
  //		sda_high;
  //		i2c_recv_ask_sensor();
}


static uint32_t i2c_recvbyte_sensor()
{
  uint8_t i;
  uint8_t dat = 0;
  sda_in;
  i = 8;
  while (i--)
  {
    //				scl_low;
    //				delay(1);
    scl_high;
    dat <<= 1;
    if (i2c_read_sda)
    {
      dat |= 1;
    }
    else
    {
      dat |= 0;
    }
    scl_low;
    delay(1);
  }
  return dat;
}

static void drv_i2c_write_byte(uint8_t Slaveaddr, uint8_t REG_Address, uint8_t REG_data)
{
  i2c_start();
  i2c_sendbyte_sensor(Slaveaddr);
  i2c_recv_ask_sensor();
  i2c_sendbyte_sensor(REG_Address);
  i2c_recv_ask_sensor();
  i2c_sendbyte_sensor(REG_data);
  i2c_recv_ask_sensor();
  i2c_stop_sensor();
}


static uint8_t drv_i2c_read_byte(uint8_t Slaveaddr, uint8_t REG_Address)
{
  uint32_t REG_data;
  i2c_start();
  i2c_sendbyte_sensor(Slaveaddr);
  i2c_recv_ask_sensor();
  i2c_sendbyte_sensor(REG_Address);
  i2c_recv_ask_sensor();
  i2c_start();
  i2c_sendbyte_sensor(Slaveaddr + 1);
  i2c_recv_ask_sensor();
  REG_data = i2c_recvbyte_sensor();
  i2c_send_ask_sensor(0);
  i2c_stop_sensor();
  return REG_data;
}

static uint8_t drv_i2c_read_bytes(uint8_t Slaveaddr, uint8_t REG_Address, uint8_t *ptr, uint8_t len)
{
  i2c_start();
  i2c_sendbyte_sensor(Slaveaddr);
  i2c_recv_ask_sensor();
  i2c_sendbyte_sensor(REG_Address);
  i2c_recv_ask_sensor();
  i2c_start();
  i2c_sendbyte_sensor(Slaveaddr + 1);
  i2c_recv_ask_sensor();
  while (len)
  {
    if (len == 1)
    {
      *ptr = i2c_recvbyte_sensor();
      i2c_send_ask_sensor(0);
    }
    else
    {
      *ptr = i2c_recvbyte_sensor();
      i2c_send_ask_sensor(1);
    }
    ptr++;
    len--;
  }
  i2c_stop_sensor();
  return 0;
}

/**********************************************************************/



bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

//IIC专用延时函数
static void I2C_delay(void)
{
  //    volatile int i = 7;
  //    while (i)
  //    i--;
}


//IIC接口初始化
void mpu_i2cInit(void)
{
  gpio_init(imu_scl_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);                          // 提取对应IO索引 AF功能编码
  gpio_init(imu_sda_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);                         // 提取对应IO索引 AF功能编码
}



static bool I2C_Start(void)
{
  SDA_H;
  SCL_H;
  I2C_delay();
  if (!SDA_read)
    return false;
  SDA_L;
  I2C_delay();
  if (SDA_read)
    return false;
  SDA_L;
  I2C_delay();
  return true;
}


static void I2C_Stop(void)
{
  SCL_L;
  I2C_delay();
  SDA_L;
  I2C_delay();
  SCL_H;
  I2C_delay();
  SDA_H;
  I2C_delay();
}


static void I2C_Ack(void)
{
  SCL_L;
  I2C_delay();
  SDA_L;
  I2C_delay();
  SCL_H;
  I2C_delay();
  SCL_L;
  I2C_delay();
}


static void I2C_NoAck(void)
{
  SCL_L;
  I2C_delay();
  SDA_H;
  I2C_delay();
  SCL_H;
  I2C_delay();
  SCL_L;
  I2C_delay();
}


static bool I2C_WaitAck(void)
{
  SCL_L;
  I2C_delay();
  SDA_H;
  I2C_delay();
  SCL_H;
  I2C_delay();
  if (SDA_read) 
  {
    SCL_L;
    return false;
  }
  SCL_L;
  return true;
}


static void I2C_SendByte(uint8_t byte)
{
  uint8_t i = 8;
  while (i--) 
  {
    SCL_L;
    I2C_delay();
    if (byte & 0x80)
      SDA_H;
    else
      SDA_L;
    byte <<= 1;
    I2C_delay();
    SCL_H;
    I2C_delay();
  }
  SCL_L;
}


static uint8_t I2C_ReceiveByte(void)
{
  uint8_t i = 8;
  uint8_t byte = 0;
  
  SDA_H;
  while (i--)
  {
    byte <<= 1;
    SCL_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    if (SDA_read) 
    {
      byte |= 0x01;
    }
  }
  SCL_L;
  return byte;
}


bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
  int i;
  if (!I2C_Start())
    return false;
  I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
  if (!I2C_WaitAck()) 
  {
    I2C_Stop();
    return false;
  }
  I2C_SendByte(reg);
  I2C_WaitAck();
  for (i = 0; i < len; i++) 
  {
    I2C_SendByte(data[i]);
    if (!I2C_WaitAck()) 
    {
      I2C_Stop();
      return false;
    }
  }
  I2C_Stop();
  return true;
}


int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
  if(i2cWriteBuffer(addr,reg,len,data))
  {
    return true;
  }
  else
  {
    return false;
  }
  //return FALSE;
}


int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
  if(i2cRead(addr,reg,len,buf))
  {
    return true;
  }
  else
  {
    return false;
  }
  //return FALSE;
}


bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
  if (!I2C_Start())
    return false;
  I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
  if (!I2C_WaitAck()) 
  {
    I2C_Stop();
    return false;
  }
  I2C_SendByte(reg);
  I2C_WaitAck();
  I2C_SendByte(data);
  I2C_WaitAck();
  I2C_Stop();
  return true;
}


bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
  if (!I2C_Start())
    return false;
  I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
  if (!I2C_WaitAck()) 
  {
    I2C_Stop();
    return false;
  }
  I2C_SendByte(reg);
  I2C_WaitAck();
  I2C_Start();
  I2C_SendByte(addr << 1 | I2C_Direction_Receiver);
  I2C_WaitAck();
  while (len) 
  {
    *buf = I2C_ReceiveByte();
    if (len == 1)
      I2C_NoAck();
    else
      I2C_Ack();
    buf++;
    len--;
  }
  I2C_Stop();
  return true;
}


uint16_t i2cGetErrorCounter(void)
{
  // TODO maybe fix this, but since this is test code, doesn't matter.
  return 0;
}




/**********************************************************************/
void single_writei2c(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)
{
  drv_i2c_write_byte(SlaveAddress<<1,REG_Address,REG_data);
  //i2cWrite(SlaveAddress,REG_Address,REG_data);
}

unsigned char single_readi2c(unsigned char SlaveAddress,unsigned char REG_Address)
{
  return drv_i2c_read_byte(SlaveAddress<<1,REG_Address);
  //	unsigned char buf;
  //	i2cRead(SlaveAddress,REG_Address,1,&buf);
  //	return buf;
}

void i2creadnbyte(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length)
{
  drv_i2c_read_bytes(addr<<1, regAddr,data,length);
  //i2cRead(addr,regAddr,length,data);
}

