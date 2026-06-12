/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "main.h"
#include "vl53l8cx_api.h"
#include "platform.h"


#define vl53lc8_addr VL53L8CX_DEFAULT_I2C_ADDRESS

#define vl53lc8_scl_pin  SOFT_I2C_SCL_PIN
#define vl53lc8_sda_pin  SOFT_I2C_SDA_PIN
/*********************************************************************************/
#define vl53lc8_sda_out  gpio_set_dir(vl53lc8_sda_pin, GPO, GPO_PUSH_PULL)
#define vl53lc8_sda_in   gpio_set_dir(vl53lc8_sda_pin, GPI, GPI_FLOATING_IN)

#define SDA_OUT()       vl53lc8_sda_out
#define SDA_IN()        vl53lc8_sda_in
#define READ_SDA()      ((gpio_get_level(vl53lc8_sda_pin)!=0)?1:0)
#define SDA_H()         gpio_high(vl53lc8_sda_pin)//SDA  		 
#define SDA_L()         gpio_low(vl53lc8_sda_pin)//SDA
#define SCL_H()         gpio_high(vl53lc8_scl_pin)//SCL  		
#define SCL_L()         gpio_low(vl53lc8_scl_pin)//SCL  

static void mdelay(uint32_t n)
{
  n = n * 5;//1~100
  while (n--);
}


void Soft_I2C_Init(void)
{
  gpio_init(vl53lc8_scl_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
  gpio_init(vl53lc8_sda_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
  
  SDA_H();
  SCL_H();
}



// 产生I2C起始信号
void Soft_I2C_Start(void)
{
  SDA_OUT();     // 输出模式
  SDA_H();
  SCL_H();
  mdelay(5);
  SDA_L();       // 当SCL为高时，SDA从高变低
  mdelay(5);
  SCL_L();       // 钳住总线，准备发送数据
}

// 产生I2C停止信号
void Soft_I2C_Stop(void)
{
  SDA_OUT();
  SCL_L();
  SDA_L();       // 确保SDA为低
  mdelay(5);
  SCL_H();       // 当SCL为高时，SDA从低变高
  SDA_H();
  mdelay(5);
}

// 等待从机的应答信号
// 返回0: 收到应答(ACK); 返回1: 未收到应答(NACK)
uint8_t Soft_I2C_Wait_Ack(void)
{
  uint8_t retry = 0;
  SDA_IN();      // 设置SDA为输入，准备读取从机应答
  SDA_H();       // 释放SDA，由上拉电阻拉高
  mdelay(1);
  SCL_H();       // 第9个时钟脉冲
  mdelay(1);
  
  while (READ_SDA()) // 等待SDA被拉低 (ACK)
  {
    retry++;
    if (retry > 200) // 超时处理
    {
      Soft_I2C_Stop();
      return 1;    // 返回失败
    }
  }
  
  SCL_L();        // 时钟拉低，结束应答位
  return 0;       // 成功收到ACK
}

// 主机产生ACK应答
void Soft_I2C_Ack(void)
{
  SCL_L();
  SDA_OUT();
  SDA_L();        // 拉低SDA表示应答
  mdelay(2);
  SCL_H();        // 第9个时钟脉冲
  mdelay(5);
  SCL_L();
}

// 主机产生NACK非应答
void Soft_I2C_NAck(void)
{
  SCL_L();
  SDA_OUT();
  SDA_H();        // 释放SDA（高电平）表示非应答
  mdelay(2);
  SCL_H();        // 第9个时钟脉冲
  mdelay(5);
  SCL_L();
}

// 主机发送一个字节 (高位先行)
void Soft_I2C_Send_Byte(uint8_t data)
{
  uint8_t i;
  SDA_OUT();
  SCL_L();        // 拉低时钟，允许改变数据
  for (i = 0; i < 8; i++)
  {
    if (data & 0x80) // 发送高位
      SDA_H();
    else
      SDA_L();
    
    data <<= 1;
    mdelay(2);
    SCL_H();    // 拉高时钟，从机读取数据
    mdelay(5);
    SCL_L();    // 拉低时钟，准备发送下一位
    mdelay(2);
  }
}

// 主机读取一个字节 (高位先行)
uint8_t Soft_I2C_Read_Byte(void)
{
  uint8_t i, data = 0;
  SDA_IN();       // 切换为输入
  for (i = 0; i < 8; i++)
  {
    SCL_L();
    mdelay(2);
    SCL_H();    // 拉高时钟，从机输出数据
    data <<= 1;
    if (READ_SDA())
      data |= 0x01;
    mdelay(2);
  }
  SCL_L();        // 最后拉低时钟
  return data;
}




// VL53L8CX I2C设备地址 (8位地址，包含读写位)
#define VL53L8CX_ADDR        0x52  // 7位地址是0x29，左移一位后为0x52 [citation:2][citation:4]
// 常用的组合：
// 写地址: (VL53L8CX_ADDR & 0xFE) -> 0x50? 不对，需要明确。
// 标准写法：7位地址0x29，左移1位得到8位地址0x52。在发送时，如果是写操作，最低位为0，即0xA4；读操作最低位为1，即0xA5。
// 为了清晰，我们直接定义这两个常用地址：
#define VL53L8CX_WRITE_ADDR  0x52  // (0x52 << 1) | 0  = 0xA4
#define VL53L8CX_READ_ADDR   0x53  // (0x52 << 1) | 1  = 0xA5


uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t reg, uint8_t *data, uint32_t size) {
    uint16_t i;
    Soft_I2C_Start();
    Soft_I2C_Send_Byte(VL53L8CX_WRITE_ADDR);
    if (Soft_I2C_Wait_Ack()) return 1;

    Soft_I2C_Send_Byte((reg >> 8) & 0xFF);
    if (Soft_I2C_Wait_Ack()) return 1;
    Soft_I2C_Send_Byte(reg & 0xFF);
    if (Soft_I2C_Wait_Ack()) return 1;

    for (i = 0; i < size; i++) {
        Soft_I2C_Send_Byte(data[i]);
        if (Soft_I2C_Wait_Ack()) return 1;
    }

    Soft_I2C_Stop();
    return 0;
}

uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t reg, uint8_t data) {
    Soft_I2C_Start();
    // 发送设备地址 + 写位
    Soft_I2C_Send_Byte(VL53L8CX_WRITE_ADDR);
    if (Soft_I2C_Wait_Ack()) return 1;

    // 发送16位寄存器地址 (高8位在前)
    Soft_I2C_Send_Byte((reg >> 8) & 0xFF);
    if (Soft_I2C_Wait_Ack()) return 1;
    Soft_I2C_Send_Byte(reg & 0xFF);
    if (Soft_I2C_Wait_Ack()) return 1;

    // 发送数据
    Soft_I2C_Send_Byte(data);
    if (Soft_I2C_Wait_Ack()) return 1;

    Soft_I2C_Stop();
    return 0;
}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t reg, uint8_t *data, uint32_t size) {
    uint16_t i;
    if (size == 0) return 0;
    // 第一步：伪写入，设置寄存器地址
    Soft_I2C_Start();
    Soft_I2C_Send_Byte(VL53L8CX_WRITE_ADDR);
    if (Soft_I2C_Wait_Ack()) return 1;
    Soft_I2C_Send_Byte((reg >> 8) & 0xFF);
    if (Soft_I2C_Wait_Ack()) return 1;
    Soft_I2C_Send_Byte(reg & 0xFF);
    if (Soft_I2C_Wait_Ack()) return 1;

    // 第二步：重新开始，读取数据
    Soft_I2C_Start(); // 重复起始条件
    Soft_I2C_Send_Byte(VL53L8CX_READ_ADDR);
    if (Soft_I2C_Wait_Ack()) return 1;

    for (i = 0; i < size; i++) {
        data[i] = Soft_I2C_Read_Byte();
        // 如果是最后一个字节，发送NACK，否则发送ACK
        if (i == size - 1) {
            Soft_I2C_NAck();
        } else {
            Soft_I2C_Ack();
        }
    }

    Soft_I2C_Stop();
    return 0;
}

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t reg, uint8_t *data) {
  // 第一步：伪写入，设置寄存器地址
  Soft_I2C_Start();
  Soft_I2C_Send_Byte(VL53L8CX_WRITE_ADDR);
  if (Soft_I2C_Wait_Ack()) return 1;
  Soft_I2C_Send_Byte((reg >> 8) & 0xFF);
  if (Soft_I2C_Wait_Ack()) return 1;
  Soft_I2C_Send_Byte(reg & 0xFF);
  if (Soft_I2C_Wait_Ack()) return 1;

  // 第二步：重新开始，读取数据
  Soft_I2C_Start(); // 重复起始条件
  Soft_I2C_Send_Byte(VL53L8CX_READ_ADDR);
  if (Soft_I2C_Wait_Ack()) return 1;

  *data = Soft_I2C_Read_Byte();
  Soft_I2C_NAck();  // 主机发送NACK，表示读取结束
  Soft_I2C_Stop();

  return 0;
}

uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform* p_platform)
{
//  gpio_set_direction(p_platform->reset_gpio, GPIO_MODE_OUTPUT);
//  gpio_set_level(p_platform->reset_gpio, VL53L8CX_RESET_LEVEL);
//  delay_ms(100);
//  gpio_set_level(p_platform->reset_gpio, !VL53L8CX_RESET_LEVEL);
//  delay_ms(100);
  return 1;
}

void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size) {
    uint32_t i;
    uint8_t tmp[4] = {0};
    for (i = 0; i < size; i = i + 4) {

        tmp[0] = buffer[i + 3];
        tmp[1] = buffer[i + 2];
        tmp[2] = buffer[i + 1];
        tmp[3] = buffer[i];
        memcpy(&(buffer[i]), tmp, 4);
    }
}

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs) {
    delay_ms(TimeMs);
    return 0;
}

VL53L8CX_Configuration 	Dev;/* Sensor configuration */
VL53L8CX_ResultsData 	Results;/* Results data from VL53L8CX */
uint32_t integration_time_ms;
uint8_t isAlive, isReady;
uint8_t vl53l_status[10];
void vl53l8x_start(void)
{
  Soft_I2C_Init();
  VL53L8CX_Reset_Sensor(&(Dev.platform));

  vl53l_status[0] = vl53l8cx_is_alive(&Dev, &isAlive);
  vl53l_status[1] = vl53l8cx_init(&Dev);
  vl53l_status[2] = vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
  vl53l_status[3] = vl53l8cx_set_ranging_mode(&Dev, VL53L8CX_RANGING_MODE_CONTINUOUS);
  vl53l_status[4] = vl53l8cx_set_ranging_frequency_hz(&Dev, 20);
  //vl53l_status[5] = vl53l8cx_set_integration_time_ms(&Dev, 20);
  vl53l_status[6]= vl53l8cx_get_integration_time_ms(&Dev, &integration_time_ms);
  vl53l_status[7] = vl53l8cx_start_ranging(&Dev);
}
