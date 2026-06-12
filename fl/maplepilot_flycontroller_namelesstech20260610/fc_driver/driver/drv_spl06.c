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
#include "datatype.h"
#include "schedule.h"
#include "drv_spl06.h"

struct spl0601_calib_param_t {
  int16_t c0;
  int16_t c1;
  int32_t c00;
  int32_t c10;
  int16_t c01;
  int16_t c11;
  int16_t c20;
  int16_t c21;
  int16_t c30;
};

struct spl0601_t {
  struct spl0601_calib_param_t calib_param;/**<calibration data*/
  uint8_t chip_id; /**<chip id*/
  int32_t i32rawPressure;
  int32_t i32rawTemperature;
  int32_t i32kP;
  int32_t i32kT;
};

static float temperature,pressure;
struct spl0601_t spl0601;
struct spl0601_t *p_spl0601;


void spl0601_write(unsigned char hwadr, unsigned char regadr, unsigned char val)
{
  single_writei2c(hwadr,regadr,val);
}


uint8_t spl0601_read(unsigned char hwadr, unsigned char regadr)
{
  uint8_t reg_data;
  reg_data=single_readi2c(hwadr,regadr);
  return reg_data;
}

void spl0601_read_n_byte(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length)
{
  i2creadnbyte(addr,regAddr,data,length);
}

void spl0601_get_calib_param(void)
{
  uint32_t h;
  uint32_t m;
  uint32_t l;
  h =  spl0601_read(HW_ADR, 0x10);
  l  =  spl0601_read(HW_ADR, 0x11);
  p_spl0601->calib_param.c0 = (int16_t)h<<4 | l>>4;
  p_spl0601->calib_param.c0 = (p_spl0601->calib_param.c0&0x0800)?(0xF000|p_spl0601->calib_param.c0):p_spl0601->calib_param.c0;
  h =  spl0601_read(HW_ADR, 0x11);
  l  =  spl0601_read(HW_ADR, 0x12);
  p_spl0601->calib_param.c1 = (int16_t)(h&0x0F)<<8 | l;
  p_spl0601->calib_param.c1 = (p_spl0601->calib_param.c1&0x0800)?(0xF000|p_spl0601->calib_param.c1):p_spl0601->calib_param.c1;
  h =  spl0601_read(HW_ADR, 0x13);
  m =  spl0601_read(HW_ADR, 0x14);
  l =  spl0601_read(HW_ADR, 0x15);
  p_spl0601->calib_param.c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
  p_spl0601->calib_param.c00 = (p_spl0601->calib_param.c00&0x080000)?(0xFFF00000|p_spl0601->calib_param.c00):p_spl0601->calib_param.c00;
  h =  spl0601_read(HW_ADR, 0x15);
  m =  spl0601_read(HW_ADR, 0x16);
  l =  spl0601_read(HW_ADR, 0x17);
  p_spl0601->calib_param.c10 = (int32_t)h<<16 | (int32_t)m<<8 | l;
  p_spl0601->calib_param.c10 = (p_spl0601->calib_param.c10&0x080000)?(0xFFF00000|p_spl0601->calib_param.c10):p_spl0601->calib_param.c10;
  h =  spl0601_read(HW_ADR, 0x18);
  l  =  spl0601_read(HW_ADR, 0x19);
  p_spl0601->calib_param.c01 = (int16_t)h<<8 | l;
  h =  spl0601_read(HW_ADR, 0x1A);
  l  =  spl0601_read(HW_ADR, 0x1B);
  p_spl0601->calib_param.c11 = (int16_t)h<<8 | l;
  h =  spl0601_read(HW_ADR, 0x1C);
  l  =  spl0601_read(HW_ADR, 0x1D);
  p_spl0601->calib_param.c20 = (int16_t)h<<8 | l;
  h =  spl0601_read(HW_ADR, 0x1E);
  l  =  spl0601_read(HW_ADR, 0x1F);
  p_spl0601->calib_param.c21 = (int16_t)h<<8 | l;
  h =  spl0601_read(HW_ADR, 0x20);
  l  =  spl0601_read(HW_ADR, 0x21);
  p_spl0601->calib_param.c30 = (int16_t)h<<8 | l;
}


void spl0601_init(void)
{
  p_spl0601 = &spl0601; /* read Chip Id */
  p_spl0601->i32rawPressure = 0;
  p_spl0601->i32rawTemperature = 0;
  p_spl0601->chip_id = spl0601_read(HW_ADR, 0x0D);//0x34;
  
  spl0601_get_calib_param();
}


void spl0601_rateset(uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
  uint8_t reg = 0;
  int32_t i32kPkT = 0;
  switch(u8SmplRate)
  {
  case 2:
    reg |= (1<<4);//左移动4位（原5位），原厂提供例程有问题，20180410无名小哥改
    break;
  case 4:
    reg |= (2<<4);
    break;
  case 8:
    reg |= (3<<4);
    break;
  case 16:
    reg |= (4<<4);
    break;
  case 32:
    reg |= (5<<4);
    break;
  case 64:
    reg |= (6<<4);
    break;
  case 128:
    reg |= (7<<4);
    break;
  case 1:
  default:
    break;
  }
  switch(u8OverSmpl)
  {
  case 2:
    reg |= 1;
    i32kPkT = 1572864;
    break;
  case 4:
    reg |= 2;
    i32kPkT = 3670016;
    break;
  case 8:
    reg |= 3;
    i32kPkT = 7864320;
    break;
  case 16:
    i32kPkT = 253952;
    reg |= 4;
    break;
  case 32:
    i32kPkT = 516096;
    reg |= 5;
    break;
  case 64:
    i32kPkT = 1040384;
    reg |= 6;
    break;
  case 128:
    i32kPkT = 2088960;
    reg |= 7;
    break;
  case 1:
  default:
    i32kPkT = 524288;
    break;
  }
  
  if(iSensor == 0)
  {
    p_spl0601->i32kP = i32kPkT;
    spl0601_write(HW_ADR, 0x06, reg);
    if(u8OverSmpl > 8)
    {
      reg = spl0601_read(HW_ADR, 0x09);
      spl0601_write(HW_ADR, 0x09, reg | 0x04);
    }
  }
  if(iSensor == 1)
  {
    p_spl0601->i32kT = i32kPkT;
    spl0601_write(HW_ADR, 0x07, reg|0x80);  //Using mems temperature
    if(u8OverSmpl > 8)
    {
      reg = spl0601_read(HW_ADR, 0x09);
      spl0601_write(HW_ADR, 0x09, reg | 0x08);
    }
  }
}


void spl0601_start_temperature(void)
{
  spl0601_write(HW_ADR, 0x08, 0x02);
}

void spl0601_start_pressure(void)
{
  spl0601_write(HW_ADR, 0x08, 0x01);
}

void spl0601_start_continuous(uint8_t mode)
{
  spl0601_write(HW_ADR, 0x08, mode+4);
}

void spl0601_get_raw_temp(void)
{
  uint8_t h[3] = {0};
  spl0601_read_n_byte(HW_ADR, 0x03,h, 3);
  p_spl0601->i32rawTemperature = (int32_t)h[0]<<16 | (int32_t)h[1]<<8 | (int32_t)h[2];
  p_spl0601->i32rawTemperature= (p_spl0601->i32rawTemperature&0x800000) ? (0xFF000000|p_spl0601->i32rawTemperature) : p_spl0601->i32rawTemperature;
}


void spl0601_get_raw_pressure(void)
{
  uint8_t h[3];
  spl0601_read_n_byte(HW_ADR, 0x00,h, 3);
  p_spl0601->i32rawPressure = (int32_t)h[0]<<16 | (int32_t)h[1]<<8 | (int32_t)h[2];
  p_spl0601->i32rawPressure= (p_spl0601->i32rawPressure&0x800000) ? (0xFF000000|p_spl0601->i32rawPressure) : p_spl0601->i32rawPressure;
}

float spl0601_get_temperature(void)
{
  float fTCompensate;
  float fTsc;
  fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
  fTCompensate =  p_spl0601->calib_param.c0 * 0.5 + p_spl0601->calib_param.c1 * fTsc;
  return fTCompensate;
}


float spl0601_get_pressure(void)
{
  float fTsc, fPsc;
  float qua2, qua3;
  float fPCompensate;
  
  fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
  fPsc = p_spl0601->i32rawPressure / (float)p_spl0601->i32kP;
  qua2 = p_spl0601->calib_param.c10 + fPsc * (p_spl0601->calib_param.c20 + fPsc* p_spl0601->calib_param.c30);
  qua3 = fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);
  //qua3 = 0.9f *fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);
  
  fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;
  //fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + 0.9f *fTsc  * p_spl0601->calib_param.c01 + qua3;
  return fPCompensate;
}

void spl06_init(void)//气压计初始化配置
{
  spl0601_init();
  //spl0601_rateset(PRESSURE_SENSOR,32, 16);//104.4ms
  //spl0601_rateset(PRESSURE_SENSOR,32, 16);//53.2ms
  //spl0601_rateset(TEMPERATURE_SENSOR,128, 2);
  
  spl0601_rateset(PRESSURE_SENSOR,128, 16);//27.6ms
  spl0601_rateset(TEMPERATURE_SENSOR,8, 8);
  //Start background measurement
  spl0601_start_continuous(CONTINUOUS_P_AND_T);
}


uint8_t user_spl0601_get(void)//气压计数据获取状态机
{
  static uint16_t junk_purge_cnt=400;//400*5ms=2S
  uint8_t update=0;
  static uint16_t spl06_cnt=0;
  spl06_cnt++;
  if(spl06_cnt==1)//1
  {
    spl0601_get_raw_temp();
    temperature = spl0601_get_temperature();
  }
  else if(spl06_cnt==10)
  {
    spl0601_get_raw_pressure();
    pressure = spl0601_get_pressure();
    spl06_cnt=0;
    update=1;
  }
  
  if(junk_purge_cnt!=0)//放弃初始化后，前2S内的垃圾数据
  {
    junk_purge_cnt--;
    update=0;
  }
  return update;
}


void spl06_read_data(float *baro_t,float *baro_p,uint8_t *update)//气压计数据采集
{
  *update=user_spl0601_get();
  *baro_p=pressure;
  *baro_t=temperature;
}

