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
#include "wp_math.h"
#include "parameter_server.h"
#include "drv_mpu6050.h"


static uint8_t chipID[2];
uint8_t bmi_read_register[9]={
  //accel configure		
  BMI088_ACC_SOFTRESET_VALUE,
  BMI088_ACC_OSR4 | BMI088_ACC_400_HZ,
  BMI088_ACC_RANGE_12G,	
  BMI088_ACC_PWR_ACTIVE_MODE,	
  BMI088_ACC_ENABLE_ACC_ON,
  //gyro configure	
  BMI088_GYRO_SOFTRESET_VALUE,
  BMI088_GYRO_2000,
  BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set,
  BMI088_GYRO_NORMAL_MODE,
};

uint8_t bmi_read_check[10]={0};
uint8_t bmi088_init(void)
{
  uint8_t BMI088_ACC_I2C_ADDR=BMI088_ACC_I2C_ADDR0;
  uint8_t BMI088_GYR_I2C_ADDR=BMI088_GYR_I2C_ADDR0;
  //check commiunication
  chipID[0]=single_readi2c(BMI088_ACC_I2C_ADDR, BMI088_ACC_CHIP_ID);
  delay_us(BMI088_SHORT_DELAY_TIME); 		
  //accel software reset
  single_writei2c(BMI088_ACC_I2C_ADDR,BMI088_ACC_SOFTRESET,bmi_read_register[0]);
  delay_ms(BMI088_SHORT_DELAY_TIME);
  /* Switch accelerometer on */
  single_writei2c(BMI088_ACC_I2C_ADDR,BMI088_ACC_PWR_CTRL,bmi_read_register[4]);
  delay_ms(BMI088_SHORT_DELAY_TIME);
  
  if(chipID[0]!=BMI088_ACC_CHIP_ID_VALUE)	return 0;
  //ACCELEROMETER
  /*Configure accelerometer LPF bandwidth (OSR4, 1000) and ODR (200 Hz, 1001) --> Actual bandwidth = 20 Hz */
  single_writei2c(BMI088_ACC_I2C_ADDR,BMI088_ACC_CONF,bmi_read_register[1]);//0x89  BMI088_ACC_OSR4 | BMI088_ACC_200_HZ
  delay_ms(BMI088_SHORT_DELAY_TIME);
  /*Accelerometer range (+-12G = 0x01)*/
  delay_ms(BMI088_SHORT_DELAY_TIME);
  single_writei2c(BMI088_ACC_I2C_ADDR,BMI088_ACC_RANGE,bmi_read_register[2]);
  delay_ms(BMI088_SHORT_DELAY_TIME);
  
  /*Set accelerometer to active mode*/
  for(uint8_t i=0;i<3;i++)
  {
    single_writei2c(BMI088_ACC_I2C_ADDR,BMI088_ACC_PWR_CONF,bmi_read_register[3]);
    delay_ms(BMI088_SHORT_DELAY_TIME);
  }
  delay_ms(BMI088_LONG_DELAY_TIME);
  
  //GYROSCOPE
  /*Check chip ID*/
  chipID[1]=single_readi2c(BMI088_GYR_I2C_ADDR, BMI088_GYRO_CHIP_ID);
  delay_ms(BMI088_SHORT_DELAY_TIME);
  if(chipID[1] != 0x0F)	return 0;
  /*Gyro Soft reset*/
  single_writei2c(BMI088_GYR_I2C_ADDR,BMI088_GYRO_SOFTRESET,bmi_read_register[5]);
  delay_ms(BMI088_LONG_DELAY_TIME);
  /*Gyro power mode*/
  single_writei2c(BMI088_GYR_I2C_ADDR, BMI088_GYRO_LPM1,bmi_read_register[8]);
  delay_ms(BMI088_LONG_DELAY_TIME);
  /*Gyro range (+- 1000deg/s)*/
  single_writei2c(BMI088_GYR_I2C_ADDR,BMI088_GYRO_RANGE,bmi_read_register[6]);
  delay_ms(BMI088_SHORT_DELAY_TIME);
  /* Gyro bandwidth/ODR (116Hz / 1000 Hz) */
  single_writei2c(BMI088_GYR_I2C_ADDR,BMI088_GYRO_BANDWIDTH,bmi_read_register[7]);//BMI088_GYRO_1000_116_HZ
  delay_ms(BMI088_SHORT_DELAY_TIME);
  
  //read check
  bmi_read_check[0]=single_readi2c(BMI088_ACC_I2C_ADDR,BMI088_ACC_SOFTRESET);
  bmi_read_check[1]=single_readi2c(BMI088_ACC_I2C_ADDR,BMI088_ACC_CONF);
  bmi_read_check[2]=single_readi2c(BMI088_ACC_I2C_ADDR,BMI088_ACC_RANGE);
  bmi_read_check[3]=single_readi2c(BMI088_ACC_I2C_ADDR,BMI088_ACC_PWR_CONF);
  bmi_read_check[4]=single_readi2c(BMI088_ACC_I2C_ADDR,BMI088_ACC_PWR_CTRL);
  
  bmi_read_check[5]=single_readi2c(BMI088_GYR_I2C_ADDR,BMI088_GYRO_SOFTRESET);
  bmi_read_check[6]=single_readi2c(BMI088_GYR_I2C_ADDR,BMI088_GYRO_RANGE);
  bmi_read_check[7]=single_readi2c(BMI088_GYR_I2C_ADDR,BMI088_GYRO_BANDWIDTH);
  bmi_read_check[8]=single_readi2c(BMI088_GYR_I2C_ADDR,BMI088_GYRO_LPM1);
  bmi_read_check[9]=single_readi2c(BMI088_GYR_I2C_ADDR,BMI088_GYRO_CTRL);
  for(uint8_t i=0;i<9;i++)
  {
    if(i==0||i==5) 
    {
      if(bmi_read_check[i]!=0X00) return 0;
      continue;
    }
    if(bmi_read_check[i]!=bmi_read_register[i]) return 0;
  }
  return 1;
}


uint8_t ICM42688P_Init(void)
{
  uint8_t AODR=AODR_1kHz,GODR=GODR_1kHz;//输出频率
  uint8_t aMode=aMode_LN,gMode=gMode_LN;//工作模式
  uint8_t imu_id_tmp = (_IMU_ID_READ)(single_readi2c(ICM42688_ADDRESS,0x75));
  if(imu_id_tmp!=WHO_AM_I_ICM42688)	return 0;
  
  //single_writei2c(ICM42688_ADDRESS,ICM42688_REG_BANK_SEL,0x00);// select register bank 0
  
  single_writei2c(ICM42688_ADDRESS,ICM42688_PWR_MGMT0, 0x00);
  single_writei2c(ICM42688_ADDRESS,ICM42688_DEVICE_CONFIG, 0x01);//enable reset
  delay_ms(50);
  
  single_writei2c(ICM42688_ADDRESS, ICM42688_INT_SOURCE0, 0x00); // temporary disable interrupts
  single_writei2c(ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
  
  single_writei2c(ICM42688_ADDRESS, ICM42688_PWR_MGMT0, gMode << 2 | aMode); // set accel and gyro modes
  delay_ms(50); // wait >200us (datasheet 14.36)
  single_writei2c(ICM42688_ADDRESS, ICM42688_ACCEL_CONFIG0, AFS_16G << 5 | AODR); // set accel ODR and FS
  single_writei2c(ICM42688_ADDRESS, ICM42688_GYRO_CONFIG0, GFS_2000DPS << 5 | GODR); // set gyro ODR and FS
  
  single_writei2c(ICM42688_ADDRESS, ICM42688_GYRO_ACCEL_CONFIG0, 4 << 4 | 4); // set gyro and accel bandwidth to 92.4hz/nbw:108.9hz
  
  single_writei2c(ICM42688_ADDRESS, ICM42688_GYRO_CONFIG1, 1<<2);//陀螺仪低通滤波阶次2
  single_writei2c(ICM42688_ADDRESS, ICM42688_ACCEL_CONFIG1, 1<<3);//加速度计低通滤波阶次2
  
  delay_ms(50); // 10ms Accel, 30ms Gyro startup
  single_writei2c(ICM42688_ADDRESS, ICM42688_FIFO_CONFIG, 0x00);  // FIFO bypass mode
  single_writei2c(ICM42688_ADDRESS, ICM42688_FSYNC_CONFIG, 0x00); // disable FSYNC
  single_writei2c(ICM42688_ADDRESS, ICM42688_FIFO_CONFIG1, 0x00); // disable FIFO
  //single_writei2c(ICM42688_ADDRESS, ICM42688_FIFO_CONFIG, 1<<6);  // begin FIFO stream
  return 1;
}



int8_t imu_sensor_type=0; //0:ICM206X;	1:BMI088;	2:ICM42688P
_IMU_ID_READ imu_id=WHO_AM_I_NONE;
_IMU_ACCEL_SCALE _imu_accel_scale=SCALE_16G;
_IMU_GYRO_SCALE  _imu_gyro_scale =SCALE_2000;

uint8_t imu_address=ICM20689_ADRESS;
static uint8_t icm_read_register[5]={0x00,0x02,0x18,0x18,0x03};
static uint8_t icm_read_check[5]={0};
_icm_inner_lpf_config icmg_inner_lpf_config=
{
  { 0x00,0x01 ,0x02,0x03,0x04,0x05,0x06,0x07  },
  { 250, 176  ,92  ,41  ,20  ,10  ,5   ,328}
};
_icm_inner_lpf_config icma_inner_lpf_config=
{
  { 0x00,0x01 ,0x02,0x03,0x04,0x05,0x06,0x07  },
  { 218 ,218  ,99  ,44  ,21  ,10  , 5,420}
};
uint8_t icm20608x_init(void)
{
  uint8_t fault=0;
  float tmp_cfg=0;
  ReadFlashParameterOne(IMU_SENSE_TYPE,&tmp_cfg);
  if(isnan(tmp_cfg)==0) imu_sensor_type=(uint8_t)(tmp_cfg);
  
  ReadFlashParameterOne(IMU_GYRO_INNER_LPF,&tmp_cfg);
  if(isnan(tmp_cfg)==0) flymaple.icm_gyro_inner_lpf_config=(uint8_t)(tmp_cfg);	
  else flymaple.icm_gyro_inner_lpf_config=icm_gyro_inner_lpf_config_default;//92hz
  
  ReadFlashParameterOne(IMU_ACCEL_INNER_LPF,&tmp_cfg);
  if(isnan(tmp_cfg)==0) flymaple.icm_accel_inner_lpf_config=(uint8_t)(tmp_cfg);		
  else flymaple.icm_accel_inner_lpf_config=icm_accel_inner_lpf_config_default;//44.8hz 
  
  icm_read_register[1]=flymaple.icm_gyro_inner_lpf_config;
  icm_read_register[4]=flymaple.icm_accel_inner_lpf_config;
  
  icm_read_register[2]=_imu_gyro_scale;
  icm_read_register[3]=_imu_accel_scale;
  
  if(imu_sensor_type==0)
  {  
    single_writei2c(imu_address,PWR_MGMT_1, 0x81);//软件强制复位81
    delay_ms(100);	
    imu_id=(_IMU_ID_READ)(single_readi2c(imu_address,WHO_AM_I));
    switch(imu_id)
    {
    case WHO_AM_I_ICM20608D:
    case WHO_AM_I_ICM20608G:		
      {
        single_writei2c(imu_address,PWR_MGMT_1,0X80);	//复位ICM20608
        delay_ms(100);
        single_writei2c(imu_address,PWR_MGMT_1, 0X01);	//唤醒ICM20608
        for(uint8_t i=0;i<3;i++)
        {
          single_writei2c(imu_address,0x19, icm_read_register[0]);   /* 输出速率是内部采样率 */
          single_writei2c(imu_address,0x1A, icm_read_register[1]);   /* 陀螺仪低通滤波,0x02*/
          //0x00设置陀螺仪、温度内部低通滤波频率范围，陀螺仪250hz，噪声带宽306.6hz，温度4000hz
          //0x01设置陀螺仪、温度内部低通滤波频率范围，陀螺仪176hz，噪声带宽177hz，温度188hz
          //0x02设置陀螺仪、温度内部低通滤波频率范围，陀螺仪92hz，噪声带宽108.6hz，温度98hz
          //0x03设置陀螺仪、温度内部低通滤波频率范围，陀螺仪41hz，噪声带宽59hz，温度42hz		
          //0x04设置陀螺仪、温度内部低通滤波频率范围，陀螺仪20hz，噪声带宽30.5hz，温度20hz
          //0x05设置陀螺仪、温度内部低通滤波频率范围，陀螺仪10hz，噪声带宽15.6hz，温度10hz
          //0x06设置陀螺仪、温度内部低通滤波频率范围，陀螺仪5hz，噪声带宽8.0hz，温度5hz
          //0x07设置陀螺仪、温度内部低通滤波频率范围，陀螺仪3281hz，噪声带宽3451hz，温度4000hz
          single_writei2c(imu_address,0x1B, icm_read_register[2]);   /* 陀螺仪±2000dps量程 */
          single_writei2c(imu_address,0x1C, icm_read_register[3]);  /* 加速度计±16G量程 */
          single_writei2c(imu_address,0x1D, icm_read_register[4]);   /* 加速度计低通滤波,0x02*/
          //0x00设置加速度计内部低通滤波频率范围，加速度218.1hz，噪声带宽235hz		
          //0x01设置加速度计内部低通滤波频率范围，加速度218.1hz，噪声带宽235hz
          //0x02设置加速度计内部低通滤波频率范围，加速度99.0hz，噪声带宽121.3hz		
          //0x03设置加速度计内部低通滤波频率范围，加速度44.8hz，噪声带宽61.5hz
          //0x04设置加速度计内部低通滤波频率范围，加速度21.2hz，噪声带宽31.0hz
          //0x05设置加速度计内部低通滤波频率范围，加速度10.2hz，噪声带宽15.5hz
          //0x06设置加速度计内部低通滤波频率范围，加速度5.1hz，噪声带宽7.8hz
          //0x07设置加速度计内部低通滤波频率范围，加速度420.0hz，噪声带宽441.6hz	
          //0x80设置加速度计内部低通滤波频率范围，加速度1046.0hz，噪声带宽1100.0hz
        }
        single_writei2c(imu_address,0x6C, 0x00);   /* 打开加速度计和陀螺仪所有轴 */
        single_writei2c(imu_address,0x1E, 0x00);   /* 关闭低功耗 */
        single_writei2c(imu_address,0x23, 0x00);   /* 关闭FIFO */ 	
      }
      break;
    default:;		
    }  
    icm_read_check[0]=single_readi2c(imu_address,0x19);
    icm_read_check[1]=single_readi2c(imu_address,0x1A);
    icm_read_check[2]=single_readi2c(imu_address,0x1B);
    icm_read_check[3]=single_readi2c(imu_address,0x1C);
    icm_read_check[4]=single_readi2c(imu_address,0x1D);
    
    _imu_gyro_scale =(_IMU_GYRO_SCALE)(single_readi2c(imu_address,0x1B));
    _imu_accel_scale=(_IMU_ACCEL_SCALE)(single_readi2c(imu_address,0x1C));
    for(uint8_t i=0;i<5;i++)
    {
      if(icm_read_check[i]!=icm_read_register[i]) fault=1;
    }
  }
  else if(imu_sensor_type==1)
  {
    if(bmi088_init())  
    {
      imu_id=WHO_AM_I_BMI088;
    }
  }
  else if(imu_sensor_type==2)
  {
    if(ICM42688P_Init())
    {
      imu_id=WHO_AM_I_ICM42688;
    }
  }  
  else imu_id=WHO_AM_I_NONE;
  
  if(imu_id==WHO_AM_I_NONE) fault=1;
  return fault;  
}





void get_imu_data(float *accel,float *gyro,float *temp,_IMU_ACCEL_SCALE *imu_acc_scale,_IMU_GYRO_SCALE *imu_gyro_scale,uint8_t *offline)
{
  int16_t ax=0,ay=0,az=0,gx=0,gy=0,gz=0,tmp=0;
  uint8_t buf[18];
  if(imu_id==WHO_AM_I_NONE) 
  {
    accel[0]=0;accel[1]=0;accel[2]=0;
    gyro[0]=0; gyro[1]=0; gyro[2]=0;
    *temp=0;    
    *imu_acc_scale =SCALE_2G;
    *imu_gyro_scale=SCALE_2000;
    *offline=1;   
    flymaple.accel_total=0;			
    flymaple.gyro_total=0;
    return;
  }
  if(imu_id==WHO_AM_I_ICM20608D||imu_id==WHO_AM_I_ICM20608G)
  {   
    i2creadnbyte(imu_address, ACCEL_XOUT_H, buf, 14);  
    ay  = (int16_t)((buf[0]<<8)|buf[1]);
    ax  = (int16_t)((buf[2]<<8)|buf[3]);
    az  =-(int16_t)((buf[4]<<8)|buf[5]);	
    tmp = (int16_t)((buf[6]<<8)|buf[7]);
    gy  = (int16_t)((buf[8]<<8)|buf[9]);
    gx  = (int16_t)((buf[10]<<8)|buf[11]);
    gz  =-(int16_t)((buf[12]<<8)|buf[13]);
    
    _IMU_ACCEL_SCALE tmp_imu_accel_scale;
    _IMU_GYRO_SCALE  tmp_imu_gyro_scale;
    tmp_imu_gyro_scale=(_IMU_GYRO_SCALE)(single_readi2c(imu_address,0x1B));
    tmp_imu_accel_scale=(_IMU_ACCEL_SCALE)(single_readi2c(imu_address,0x1C));
    float gyro_scale_dps=0,accel_scale_g=0;	
    switch(tmp_imu_gyro_scale)
    {
    case SCALE_250: gyro_scale_dps=GYRO_SCALE_250_DPS; break;
    case SCALE_500: gyro_scale_dps=GYRO_SCALE_500_DPS; break;
    case SCALE_1000:gyro_scale_dps=GYRO_SCALE_1000_DPS;break;
    case SCALE_2000:gyro_scale_dps=GYRO_SCALE_2000_DPS;break;
    }
    switch(tmp_imu_accel_scale)
    {
    case SCALE_2G: flymaple.accel_scale_raw_to_g=16384.0f;  break;
    case SCALE_4G: flymaple.accel_scale_raw_to_g=8192.0f;   break;
    case SCALE_8G: flymaple.accel_scale_raw_to_g=4096.0f;   break;
    case SCALE_16G:flymaple.accel_scale_raw_to_g=2048.0f;   break;
    case SCALE_3G: flymaple.accel_scale_raw_to_g=10920.0f;   break;
    case SCALE_6G: flymaple.accel_scale_raw_to_g=5460.0f;   break;
    case SCALE_12G:flymaple.accel_scale_raw_to_g=2730.0f;   break;
    case SCALE_24G:flymaple.accel_scale_raw_to_g=1365.0; break;
    }	
    *imu_acc_scale =tmp_imu_accel_scale;
    *imu_gyro_scale=tmp_imu_gyro_scale;
    
    accel[0]=ax;//ax/accel_scale_g;
    accel[1]=ay;//ay/accel_scale_g;
    accel[2]=az;//az/accel_scale_g;
    gyro[0]=gx; //gx*gyro_scale_dps;
    gyro[1]=gy; //gy*gyro_scale_dps;
    gyro[2]=gz; //gz*gyro_scale_dps;
    
    vector3f tmp_accel,dst_accel;
    tmp_accel.x=accel[0];
    tmp_accel.y=accel[1];
    tmp_accel.z=accel[2];
    
    vector3f_mul_sub_to_rawdata(tmp_accel,flymaple.accel_scale,flymaple.accel_offset,&dst_accel,accel_scale_g);//校准后的加速度计原始数字量
    dst_accel.x/=accel_scale_g;
    dst_accel.y/=accel_scale_g;
    dst_accel.z/=accel_scale_g;
    flymaple.accel_total=sq(dst_accel.x)+sq(dst_accel.y)+sq(dst_accel.z);			
    flymaple.gyro_total =sq(gyro[0]*gyro_scale_dps-flymaple.gyro_offset.x)+sq(gyro[1]*gyro_scale_dps-flymaple.gyro_offset.y)+sq(gyro[2]*gyro_scale_dps-flymaple.gyro_offset.z);
    
    switch(imu_id)
    {
    case WHO_AM_I_MPU6050:
      {
        *temp=36.53f+(float)(tmp/340.0f);
      }
      break;
    case WHO_AM_I_ICM20689:
      {
        *temp=25.0f+(double)((tmp-25.0f)/326.8f);
      }
      break;
    case WHO_AM_I_ICM20608D:
    case WHO_AM_I_ICM20608G:
    case WHO_AM_I_ICM20602:	
      {
        *temp=25.0f+(double)((tmp-25.0f)/326.8f);
      }
      break;
    default:
      {
        *temp=36.53f+(float)(tmp/340.0f);
      }
    }
    *offline=0;   
  }
  else if(imu_id==WHO_AM_I_BMI088)
  {
    uint8_t BMI088_ACC_I2C_ADDR=BMI088_ACC_I2C_ADDR0;
    uint8_t BMI088_GYR_I2C_ADDR=BMI088_GYR_I2C_ADDR0; 
    i2creadnbyte(BMI088_ACC_I2C_ADDR,BMI088_ACCEL_XOUT_L,buf,6);
    accel[0]= (int16_t)((buf[1]<<8)|buf[0]);
    accel[1]=-(int16_t)((buf[3]<<8)|buf[2]);
    accel[2]=-(int16_t)((buf[5]<<8)|buf[4]);		
    
    i2creadnbyte(BMI088_ACC_I2C_ADDR,BMI088_TEMP_M,&buf[12],2);//updated every 1.28s
    tmp=(int16_t)((buf[12] << 3) | (buf[13] >> 5));
    if(tmp>1023)	tmp-=2048;
    *temp=tmp*BMI088_TEMP_FACTOR+BMI088_TEMP_OFFSET;	
    i2creadnbyte(BMI088_GYR_I2C_ADDR,BMI088_GYRO_X_L,&buf[6],6);
    gyro[0]= (int16_t)((buf[7]<<8) |buf[6]);
    gyro[1]=-(int16_t)((buf[9]<<8) |buf[8]);
    gyro[2]=-(int16_t)((buf[11]<<8)|buf[10]);
    
    *imu_acc_scale =SCALE_12G;
    *imu_gyro_scale=SCALE_2000;
    *offline=0; 
    float gyro_scale_dps=0,accel_scale_g=0;	
    switch(*imu_gyro_scale)
    {
    case SCALE_250: gyro_scale_dps=GYRO_SCALE_250_DPS; break;
    case SCALE_500: gyro_scale_dps=GYRO_SCALE_500_DPS; break;
    case SCALE_1000:gyro_scale_dps=GYRO_SCALE_1000_DPS;break;
    case SCALE_2000:gyro_scale_dps=GYRO_SCALE_2000_DPS;break;
    }
    switch(*imu_acc_scale)
    {
    case SCALE_2G: flymaple.accel_scale_raw_to_g=16384.0f;  break;
    case SCALE_4G: flymaple.accel_scale_raw_to_g=8192.0f;   break;
    case SCALE_8G: flymaple.accel_scale_raw_to_g=4096.0f;   break;
    case SCALE_16G:flymaple.accel_scale_raw_to_g=2048.0f;   break;
    case SCALE_3G: flymaple.accel_scale_raw_to_g=10920.0f;   break;
    case SCALE_6G: flymaple.accel_scale_raw_to_g=5460.0f;   break;
    case SCALE_12G:flymaple.accel_scale_raw_to_g=2730.0f;   break;
    case SCALE_24G:flymaple.accel_scale_raw_to_g=1365.0; break;
    }	
    
    vector3f tmp_accel,dst_accel;
    tmp_accel.x=accel[0];
    tmp_accel.y=accel[1];
    tmp_accel.z=accel[2];
    
    vector3f_mul_sub_to_rawdata(tmp_accel,flymaple.accel_scale,flymaple.accel_offset,&dst_accel,accel_scale_g);//校准后的加速度计原始数字量
    dst_accel.x/=accel_scale_g;
    dst_accel.y/=accel_scale_g;
    dst_accel.z/=accel_scale_g;
    flymaple.accel_total=sq(dst_accel.x)+sq(dst_accel.y)+sq(dst_accel.z);			
    flymaple.gyro_total =sq(gyro[0]*gyro_scale_dps-flymaple.gyro_offset.x)+sq(gyro[1]*gyro_scale_dps-flymaple.gyro_offset.y)+sq(gyro[2]*gyro_scale_dps-flymaple.gyro_offset.z);
  }
  else
  {
    i2creadnbyte(ICM42688_ADDRESS,0x1D,buf,14);
    int16_t tmp=(int16_t)((buf[0]<<8)|buf[1]);
    accel[0]= (int16_t)((buf[2]<<8)|buf[3]);
    accel[1]=-(int16_t)((buf[4]<<8)|buf[5]);
    accel[2]=-(int16_t)((buf[6]<<8)|buf[7]);	
    gyro[0] = (int16_t)((buf[8]<<8) |buf[9]);
    gyro[1] =-(int16_t)((buf[10]<<8)|buf[11]);
    gyro[2] =-(int16_t)((buf[12]<<8)|buf[13]);
    *temp=tmp/132.48f+25.0f;	
    
    *imu_acc_scale =SCALE_16G;
    *imu_gyro_scale=SCALE_2000;
    *offline=1;//没有温控的情况下置1  
    
    float gyro_scale_dps=0,accel_scale_g=0;	
    switch(*imu_gyro_scale)
    {
    case SCALE_250: gyro_scale_dps=GYRO_SCALE_250_DPS; break;
    case SCALE_500: gyro_scale_dps=GYRO_SCALE_500_DPS; break;
    case SCALE_1000:gyro_scale_dps=GYRO_SCALE_1000_DPS;break;
    case SCALE_2000:gyro_scale_dps=GYRO_SCALE_2000_DPS;break;
    }
    switch(*imu_acc_scale)
    {
    case SCALE_2G: flymaple.accel_scale_raw_to_g=16384.0f;  break;
    case SCALE_4G: flymaple.accel_scale_raw_to_g=8192.0f;   break;
    case SCALE_8G: flymaple.accel_scale_raw_to_g=4096.0f;   break;
    case SCALE_16G:flymaple.accel_scale_raw_to_g=2048.0f;   break;
    case SCALE_3G: flymaple.accel_scale_raw_to_g=10920.0f;   break;
    case SCALE_6G: flymaple.accel_scale_raw_to_g=5460.0f;   break;
    case SCALE_12G:flymaple.accel_scale_raw_to_g=2730.0f;   break;
    case SCALE_24G:flymaple.accel_scale_raw_to_g=1365.0; break;
    }	
    
    vector3f tmp_accel,dst_accel;
    tmp_accel.x=accel[0];
    tmp_accel.y=accel[1];
    tmp_accel.z=accel[2];
    
    vector3f_mul_sub_to_rawdata(tmp_accel,flymaple.accel_scale,flymaple.accel_offset,&dst_accel,accel_scale_g);//校准后的加速度计原始数字量
    dst_accel.x/=accel_scale_g;
    dst_accel.y/=accel_scale_g;
    dst_accel.z/=accel_scale_g;
    flymaple.accel_total=sq(dst_accel.x)+sq(dst_accel.y)+sq(dst_accel.z);			
    flymaple.gyro_total =sq(gyro[0]*gyro_scale_dps-flymaple.gyro_offset.x)+sq(gyro[1]*gyro_scale_dps-flymaple.gyro_offset.y)+sq(gyro[2]*gyro_scale_dps-flymaple.gyro_offset.z);
  }
}
















