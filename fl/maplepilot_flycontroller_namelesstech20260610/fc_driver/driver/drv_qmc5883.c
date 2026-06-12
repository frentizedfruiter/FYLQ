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
#include "datatype.h"
#include "drv_qmc5883.h"

//磁力计传感器类型
#define compass_type 0//0:qmc5883l  1:qmc5883p


void qmc5883p_init(void);


void QMC5883L_Initialize(_qmc5883l_MODE MODE,_qmc5883l_ODR ODR,_qmc5883l_RNG RNG,_qmc5883l_OSR OSR)
{
  single_writei2c(QMC5883L_RD_ADDRESS,QMC5883L_CONFIG_3,0x01);
  single_writei2c(QMC5883L_RD_ADDRESS,QMC5883L_CONFIG_1,MODE | ODR | RNG | OSR);
}

void QMC5883L_Reset()
{
  single_writei2c(QMC5883L_RD_ADDRESS,QMC5883L_CONFIG_2,0x81);
}


void qmc5883l_init(void)
{
#if compass_type==0
  QMC5883L_Reset();
  Delay_Ms(5);
  QMC5883L_Initialize(MODE_CONTROL_CONTINUOUS,OUTPUT_DATA_RATE_200HZ,FULL_SCALE_8G,OVER_SAMPLE_RATIO_64);
#else  
  qmc5883p_init();
#endif
}



void qmc5883p_init(void)
{
  delay_ms(100);
  // Reset the sensor
  single_writei2c(QMC5883P_ADDR,QMC5883P_REG_CONTROL_2,QMC5883P_CONTROL_2_SOFT_RESET);
  delay_ms(100);
  single_writei2c(QMC5883P_ADDR,QMC5883P_REG_CONTROL_2,0x00);
  delay_ms(100);
  single_readi2c(QMC5883P_ADDR,QMC5883P_REG_CHIP_ID);
  single_writei2c(QMC5883P_ADDR,QMC5883P_REG_CONTROL_2,QMC5883P_CONTROL_2_RNG);
  // Configure the sensor
  uint8_t data = QMC5883P_CONTROL_1_MODE_CONT | QMC5883P_CONTROL_1_ODR | QMC5883P_CONTROL_1_OSR1_2 | QMC5883P_CONTROL_1_OSR2_2;
  single_writei2c(QMC5883P_ADDR,QMC5883P_REG_CONTROL_1,data);
}

void get_mag_data(float *mag,uint8_t *update)
{	
  uint8_t buf[6];
  static uint16_t cnt=0;
  cnt++;
  if(cnt>=10)
  {
#if compass_type==0
    i2creadnbyte(QMC5883L_RD_ADDRESS, QMC5883L_DATA_READ_X_LSB, buf, 6);
    mag[0]= (float)((int16_t)((buf[1]<<8)|buf[0])/QMC5883L_CONVERT_GAUSS_8G);
    mag[1]= (float)((int16_t)((buf[3]<<8)|buf[2])/QMC5883L_CONVERT_GAUSS_8G);
    mag[2]= (float)((int16_t)((buf[5]<<8)|buf[4])/QMC5883L_CONVERT_GAUSS_8G);	
    cnt=0;
    *update=1;
#else  
    i2creadnbyte(QMC5883P_ADDR, QMC5883P_REG_XOUT_L, buf, 6);    
    mag[1]= -(float)((int16_t)((buf[1]<<8)|buf[0])/QMC5883_RNG_SENSITIVITY_8G);
    mag[0]=  (float)((int16_t)((buf[3]<<8)|buf[2])/QMC5883_RNG_SENSITIVITY_8G);
    mag[2]=  (float)((int16_t)((buf[5]<<8)|buf[4])/QMC5883_RNG_SENSITIVITY_8G);	
    cnt=0;
    *update=1;   
#endif
  }
}

