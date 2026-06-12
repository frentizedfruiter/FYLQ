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
#include "datatype.h"
#include "main.h"
#include "schedule.h"
#include "filter.h"
#include "quene.h"
#include "wp_math.h"
#include "mt01.h"
#include "parameter_server.h"
#include "attitude_ctrl.h"
#include "drv_opticalflow.h"
#include "maple_config.h"
#include "vl53l8cx_api.h"
#include "platform.h"
#include "drv_tofsense.h"


void uart3_send_bytes(unsigned char *buf, int len);


tofsense tofsense_data;
//linear_quene tofsense_quene;
static systime _t;
uint16_t fj[2];
void tofsense_receive_anl(uint8_t *data_buf,uint8_t num)
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<(num-1);i++)  sum+=*(data_buf+i);
  if(!(sum==*(data_buf+num-1))) 	return;//不满足和校验条件
  if(!(*(data_buf)==0x57 && *(data_buf+1)==0x00))return;//不满足帧头条件
  
  get_systime(&_t);
  tofsense_data.frame_header=data_buf[0];
  tofsense_data.function_mark=data_buf[1];
  tofsense_data.id=data_buf[3];
  tofsense_data.system_time=data_buf[4]|(data_buf[5]<<8)|(data_buf[6]<<16)|(data_buf[7]<<24);
  tofsense_data.dis=(int32_t)(data_buf[8]<< 8|data_buf[9]<<16|data_buf[10]<<24)/256;
  tofsense_data.dis_status=data_buf[11];
  tofsense_data.signal_strength=data_buf[12]|(data_buf[13]<<8);
  tofsense_data.range_precision=data_buf[14];
  
  tofsense_data.distance=tofsense_data.dis/10.0f;//cm		
  tofsense_data.update_flag=1;
  //if(tofsense_data.dis_status==1&&tofsense_data.distance<1200) tofsense_data.valid=1;
  if(tofsense_data.signal_strength!=0&&tofsense_data.distance<1200) tofsense_data.valid=1;
  else tofsense_data.valid=0;
  tofsense_data.period_ms=_t.period_int;
  
  tofsense_data.average=calculate_average(tofsense_data.height_backups,10);//计算均值
  tofsense_data.height_standard_deviation=calculate_standard_deviation(tofsense_data.height_backups,10);
  for(int16_t i=20-1;i>0;i--)//历史位置观测值
  {
    tofsense_data.height_backups[i]=tofsense_data.height_backups[i-1];
  }
  tofsense_data.height_backups[0]=tofsense_data.distance;
}

fault_rate tof_fault_rate=
{
  .period_cnt=200,
};
uint8_t buf[16];
void tofsense_prase(uint8_t data)
{
  
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==0x57)//判断帧头1
  {
    state=1;
    buf[0]=data;
  }
  else if(state==1&&data==0x00)//判断帧头2
  {
    state=2;
    buf[1]=data;
  }
  else if(state==2&&data==0xff)//功能字节
  {
    state=3;
    buf[2]=data;
  }
  else if(state==3&&data<=0xff)//id
  {
    state = 4;
    buf[3]=data;
    data_len = 11;
    data_cnt = 0;
  }
  else if(state==4&&data_len>0)//数据接收
  {
    data_len--;
    buf[4+data_cnt++]=data;
    if(data_len==0)  state = 5;
  }
  else if(state==5)//和校验
  {
    state = 0;
    buf[4+data_cnt++]=data;
    tofsense_receive_anl(buf,16);
    tof_fault_rate.right_cnt+=16;
  }
  else 
  {
    state = 0;
    tof_fault_rate.wrong_cnt++;
  }
  tof_fault_rate.value=(float)tof_fault_rate.right_cnt/(tof_fault_rate.right_cnt+tof_fault_rate.wrong_cnt);
  //	if(tof_fault_rate.right_cnt+tof_fault_rate.wrong_cnt>=tof_fault_rate.period_cnt)
  //	{
  //		tof_fault_rate.value=(float)tof_fault_rate.right_cnt/(tof_fault_rate.right_cnt+tof_fault_rate.wrong_cnt);
  //		tof_fault_rate.right_cnt=0;
  //		tof_fault_rate.wrong_cnt=0;
  //	}
}


void mt01_prase(uint8_t data)
{
  static uint8_t buf[16];
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==0xEF)//判断帧头1
  {
    state=1;
    buf[0]=data;
  }
  else if(state==1&&data==0x0F)//设备id
  {
    state=2;
    buf[1]=data;
  }
  else if(state==2&&data==0x00)//系统id
  {
    state=3;
    buf[2]=data;
  }
  else if(state==3&&data==0x51)//消息id
  {
    state=3;
    buf[2]=data;
  }
  else if(state==3&&data<=0xff)//id
  {
    state = 4;
    buf[3]=data;
    data_len = 11;
    data_cnt = 0;
  }
  else if(state==4&&data_len>0)//数据接收
  {
    data_len--;
    buf[4+data_cnt++]=data;
    if(data_len==0)  state = 5;
  }
  else if(state==5)//和校验
  {
    state = 0;
    buf[4+data_cnt++]=data;
    tofsense_receive_anl(buf,16);
    tof_fault_rate.right_cnt++;
  }
  else 
  {
    state = 0;
    tof_fault_rate.wrong_cnt++;
  }
  
  if(tof_fault_rate.right_cnt+tof_fault_rate.wrong_cnt>=tof_fault_rate.period_cnt)
  {
    tof_fault_rate.value=tof_fault_rate.right_cnt/tof_fault_rate.period_cnt;
    tof_fault_rate.right_cnt=0;
    tof_fault_rate.wrong_cnt=0;
  }
}



void tfmini_prase(uint8_t data)
{
  static uint8_t tfmini_buf[9];
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==0x59)//判断帧头1
  {
    state=1;
    tfmini_buf[0]=data;
  }
  else if(state==1&&data==0x59)//判断帧头2
  {
    state=2;
    tfmini_buf[1]=data;
    data_len = 6;
    data_cnt = 0;
  }
  else if(state==2&&data_len>0)//数据接收
  {
    data_len--;
    tfmini_buf[2+data_cnt++]=data;
    if(data_len==0)  state = 3;
  }
  else if(state==3)//和校验
  {
    state = 0;
    tfmini_buf[2+data_cnt++]=data;
    uint8_t sum=0;
    for(uint8_t i=0;i<8;i++)
    {
      sum+=tfmini_buf[i];
    }
    if(sum==tfmini_buf[8])//满足和校验
    {
      get_systime(&_t);
      tofsense_data.dis        			= tfmini_buf[2] | tfmini_buf[3]<<8;
      tofsense_data.signal_strength = tfmini_buf[4] | tfmini_buf[5]<<8;
      tofsense_data.range_precision = 0;
      tofsense_data.dis_status    	= 0;
      tofsense_data.id          		= 0;
      
      tofsense_data.distance=tofsense_data.dis;//cm		
      tofsense_data.update_flag=1;
      if(tofsense_data.signal_strength!=65535&&tofsense_data.signal_strength>=100)
      {
        if(tofsense_data.distance<1000&&tofsense_data.distance>1.0f)	tofsense_data.valid=1;
        else tofsense_data.valid=0;
      }
      else tofsense_data.valid=0;
      tofsense_data.period_ms=_t.period_int;				
    }
  }
  else 
  {
    state = 0;
  }
}




uint8_t send_buf[18]={'\0'};
//计算检验和
uint8_t countsum(uint8_t *buf)
{
  uint8_t len = 0;
  uint8_t checksum =0;
  len = sizeof(buf)+1;
  while(len --)
  {
    checksum += *buf;
    buf++;
  }
  
  //保留最后两位
  checksum &=0xFF;
  
  return checksum;
}


//停止扫描
void stop_scan(void)
{
  send_buf[0] = 0xAA;
  send_buf[1] = 0x55;
  send_buf[2] = 0x61;
  send_buf[3] = 0x00;
  send_buf[4] = 0x60;
  
  uart3_send_bytes(send_buf,5);
} 

//开始扫描
void start_scan(void)
{
  send_buf[0] = 0xAA;
  send_buf[1] = 0x55;
  send_buf[2] = 0x60;
  send_buf[3] = 0x00;
  send_buf[4] = 0x5F;
  
  uart3_send_bytes(send_buf,5);
}

//设置用标准数据的格式输出
void SMD15_setstandard(void)
{
  send_buf[0] = 0xAA;
  send_buf[1] = 0x55;
  send_buf[2] = 0x67;
  send_buf[3] = 0x01;
  send_buf[4] = 0x00;
  send_buf[5] = 0x67;
  
  uart3_send_bytes(send_buf,6);
}

//设置用pixhawk数据的格式输出-这种模式雷达直接输出测距信息到串口调试助手可以直接显示 
void SMD15_setpixhawk(void)//不使用
{
  send_buf[0] = 0xAA;
  send_buf[1] = 0x55;
  send_buf[2] = 0x67;
  send_buf[3] = 0x01;
  send_buf[4] = 0x01;
  send_buf[5] = 0x68;
  
  uart3_send_bytes(send_buf,6);
}

//设置波特率 
//230400、460800、512000、921600、1500000 分别对应代号 0-4（默认为 460800）
void SMD15_setbaudrate(uint8_t i)
{
  send_buf[0] = 0xAA;
  send_buf[1] = 0x55;
  send_buf[2] = 0x66;
  send_buf[3] = 0x01;
  
  switch(i)
  {
  case 0:send_buf[4] = 0x00;break;
  case 1:send_buf[4] = 0x01;break;
  case 2:send_buf[4] = 0x02;break;
  case 3:send_buf[4] = 0x03;break;
  case 4:send_buf[4] = 0x04;break;
  default :break;
  }
  send_buf[5] = countsum(send_buf);
  uart3_send_bytes(send_buf,6);
}

//设置输出频率 
//10hz、100hz、200hz、500hz、1000hz、1800hz 输出频率，分别对应代号 0-5（默认为100hz）
void SMD15_setScanfHZ(uint8_t hz)
{
  send_buf[0] = 0xAA;
  send_buf[1] = 0x55;
  send_buf[2] = 0x64;
  send_buf[3] = 0x01;
  
  switch(hz)
  {
  case 0:send_buf[4] = 0x00;break;
  case 1:send_buf[4] = 0x01;break;
  case 2:send_buf[4] = 0x02;break;
  case 3:send_buf[4] = 0x03;break;
  case 4:send_buf[4] = 0x04;break;
  case 5:send_buf[4] = 0x05;break;
  default :break;
  }
  send_buf[5] = countsum(send_buf);
  uart3_send_bytes(send_buf,6);
}


uint32_t smd_miss_cnt[2];
float smd_miss_rate=0;
void smd15_prase(uint8_t data)
{
  static uint8_t sdm15_buf[9];
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==0xAA)//判断帧头1
  {
    state=1;
    sdm15_buf[0]=data;
  }
  else if(state==1&&data==0x55)//判断帧头2
  {
    state=2;
    sdm15_buf[1]=data;
    data_cnt = 0;
  }
  else if(state==2&&data==0x60)//命令类型
  {
    state=3;
    sdm15_buf[2]=data;
    data_cnt = 0;
  }
  else if(state==3&&data==0x04)//判断数据长度
  {
    state=4;
    sdm15_buf[3]=data;
    data_len = 4;
    data_cnt = 0;
  }
  else if(state==4&&data_len>0)//数据接收
  {
    data_len--;
    sdm15_buf[4+data_cnt++]=data;
    if(data_len==0)  state = 5;
  }
  else if(state==5)//和校验
  {
    state = 0;
    sdm15_buf[4+data_cnt++]=data;
    uint8_t sum=0;
    for(uint8_t i=0;i<8;i++)
    {
      sum+=sdm15_buf[i];
    }
    if(sum==sdm15_buf[8])//满足和校验
    {
      get_systime(&_t);
      tofsense_data.dis        			= sdm15_buf[4] | sdm15_buf[5]<<8;
      tofsense_data.signal_strength = sdm15_buf[6];
      tofsense_data.range_precision = sdm15_buf[7];
      tofsense_data.dis_status    	= 0;
      tofsense_data.id          		= 0;
      
      tofsense_data.distance=tofsense_data.dis*0.1f;//cm		
      tofsense_data.update_flag=1;
      if(tofsense_data.signal_strength>=50)
      {
        if(tofsense_data.distance<1200&&tofsense_data.distance>1.0f)	tofsense_data.valid=1;
        else tofsense_data.valid=0;
      }
      else tofsense_data.valid=0;
      tofsense_data.period_ms=_t.period_int;		
    }
    memset(sdm15_buf,0,9);//清一下数据
    smd_miss_cnt[0]+=9;
    smd_miss_rate=(float)(smd_miss_cnt[0])/(smd_miss_cnt[0]+smd_miss_cnt[1]);
  }
  else 
  {
    state = 0;
    smd_miss_cnt[1]++;
  }
}


//初始化配置SMD15
void SMD15_init(uint32_t baudrate)//要放在串口2初始化后面
{
  delay_ms(200);//等待串口初始化完成
  //	//停止扫描
  //	stop_scan();
  //	delay_ms(1);
  SMD15_setstandard();//设置用标准数据的格式输出
  SMD15_setScanfHZ(100);//设置扫描频率
  //	delay_ms(5);
  //	uint8_t baud;
  //	//激光雷达初始化 230400、460800、512000、921600、1500000
  //	if     (baudrate ==230400) 	baud = 0;
  //	else if(baudrate ==460800) 	baud = 1;
  //	else if(baudrate ==512000)	baud = 2;
  //	else if(baudrate ==921600) 	baud = 3;//这是stm32最高的波特率，理论是4.5M,但可达到这个9.2MHz
  //	else if(baudrate ==1500000) baud = 4;	
  //	SMD15_setbaudrate(baud);//设置波特率 设置成功后，重启雷达生效 一般不设置
  //	//开始扫描
  start_scan();
}

/************************************************************************/
uint8_t gytof_send_buf[5];
void gytof10m_setBaud(uint32_t baud)
{
  uint8_t baud_cfg=0;
  if     (baud ==2400) 		baud_cfg = 0;//2400
  else if(baud ==4800) 		baud_cfg = 1;//4800
  else if(baud ==9600)		baud_cfg = 2;//9600
  else if(baud ==19200) 	baud_cfg = 3;//19200
  else if(baud ==38400) 	baud_cfg = 4;//38400
  else if(baud ==57600) 	baud_cfg = 5;//57600
  else if(baud ==115200) 	baud_cfg = 6;//115200
  else if(baud ==230400) baud_cfg = 7; //230400	
  else baud_cfg = 2;//9600
  gytof_send_buf[0]=0xA4;
  gytof_send_buf[1]=0x06;
  gytof_send_buf[2]=0x01;
  gytof_send_buf[3]=baud_cfg;
  uint8_t sum=0;
  for(uint16_t i=0;i<4;i++)	sum+=gytof_send_buf[i];
  gytof_send_buf[4]=sum; 
  uart3_send_bytes(gytof_send_buf,5);
}

void gytof10m_setOutputHz(uint32_t hz)
{
  uint8_t hz_cfg=0;
  if     (hz ==1) 	hz_cfg = 0;//1hz
  else if(hz ==10)  hz_cfg = 1;//10hz
  else if(hz ==50)	hz_cfg = 2;//50hz
  else if(hz ==100) hz_cfg = 3;//100hz
  else if(hz ==200) hz_cfg = 4;//200hz
  else hz = 2;//50hz
  gytof_send_buf[0]=0xA4;
  gytof_send_buf[1]=0x06;
  gytof_send_buf[2]=0x02;
  gytof_send_buf[3]=hz_cfg;
  uint8_t sum=0;
  for(uint16_t i=0;i<4;i++)	sum+=gytof_send_buf[i];
  gytof_send_buf[4]=sum; 
  uart3_send_bytes(gytof_send_buf,5);
}

void gytof10m_saveConfig(void)
{
  gytof_send_buf[0]=0xA4;
  gytof_send_buf[1]=0x06;
  gytof_send_buf[2]=0x05;
  gytof_send_buf[3]=0x55;
  gytof_send_buf[4]=0x04; 
  uart3_send_bytes(gytof_send_buf,5);
}

void gytof10m_resetConfig(void)
{
  gytof_send_buf[0]=0xA4;
  gytof_send_buf[1]=0x06;
  gytof_send_buf[2]=0x05;
  gytof_send_buf[3]=0xAA;
  gytof_send_buf[4]=0x59; 
  uart3_send_bytes(gytof_send_buf,5);
}


void gytof10m_prase(uint8_t data)
{
  static uint8_t gytof10m_buf[10];
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==0xA4)//判断帧头1
  {
    state=1;
    gytof10m_buf[0]=data;
  }
  else if(state==1&&data==0x03)//判断帧头2
  {
    state=2;
    gytof10m_buf[1]=data;
    data_cnt = 0;
  }
  else if(state==2&&data==0x08)//命令类型
  {
    state=3;
    gytof10m_buf[2]=data;
    data_cnt = 0;
  }
  else if(state==3&&data==0x05)//判断数据长度
  {
    state=4;
    gytof10m_buf[3]=data;
    data_len = 5;
    data_cnt = 0;
  }
  else if(state==4&&data_len>0)//数据接收
  {
    data_len--;
    gytof10m_buf[4+data_cnt++]=data;
    if(data_len==0)  state = 5;
  }
  else if(state==5)//和校验
  {
    state = 0;
    gytof10m_buf[4+data_cnt++]=data;
    uint8_t sum=0;
    for(uint8_t i=0;i<9;i++)
    {
      sum+=gytof10m_buf[i];
    }
    if(sum==gytof10m_buf[9])//满足和校验
    {
      get_systime(&_t);
      tofsense_data.dis        			= gytof10m_buf[4]<<8 | gytof10m_buf[5];
      tofsense_data.signal_strength = gytof10m_buf[6]<<8 | gytof10m_buf[7];
      tofsense_data.range_precision = gytof10m_buf[8];
      tofsense_data.dis_status    	= 0;
      tofsense_data.id          		= 0;
      
      tofsense_data.distance=tofsense_data.dis*0.1f;//cm		
      tofsense_data.update_flag=1;
      if(tofsense_data.signal_strength>=100)
      {
        if(tofsense_data.distance<900&&tofsense_data.distance>1.0f)	tofsense_data.valid=1;
        else tofsense_data.valid=0;
      }
      else tofsense_data.valid=0;
      tofsense_data.period_ms=_t.period_int;		
    }
    memset(gytof10m_buf,0,10);//清一下数据
  }
  else 
  {
    state = 0;
  }
}



/************************************************************************/
void uart3_init(uint32_t baud)
{
  //uart_init(UART_3,baud,UART3_TX_P13_1,UART3_RX_P13_0);              
  //uart_rx_interrupt(UART_3, 1);// 使能对应串口接收中断
  //interrupt_set_priority(CPUIntIdx6_IRQn, 2);
  uart_init_intidx(UART_3,baud,UART3_TX_P13_1,UART3_RX_P13_0,uart3_user_isr,uart3_irq_chl_pre_priority,uart3_irq_chl_sub_priority);              
  uart_rx_interrupt(UART_3, 1);// 使能对应串口接收中断   
}

void uart3_send_bytes(unsigned char *buf, int len)
{
  while(len--)
  {
    Cy_SCB_WriteTxFifo(SCB3,*buf);	
    while(Cy_SCB_IsTxComplete(SCB3) == 0);
    buf++;
  }
}

void UART3_IRQHandler(void)
{  
  if(Cy_SCB_GetRxInterruptMask(SCB3) & CY_SCB_UART_RX_NOT_EMPTY)// 串口接收中断
  {
    if(Cy_SCB_GetNumInRxFifo(SCB3))
    {
      uint8_t ch  = (uint8)Cy_SCB_ReadRxFifo(SCB3);
      switch(maplepilot.rangefinder_sensor)
      {
      case TOFSENSE:
        tofsense_prase(ch);
        break;
      case VL53L8:
        smd15_prase(ch);
        break;
      case UP_Tx:
        lc307_opticalflow_prase(ch);
        break;
      case MT1:
        micolink_decode(ch);
        break;			
      case GYTOF10M:
        gytof10m_prase(ch);
        break;
      case TFMINI:
        tfmini_prase(ch);
        break;
      default:{}
      }	
    }
    Cy_SCB_ClearRxInterrupt(SCB3, CY_SCB_UART_RX_NOT_EMPTY);              // 清除接收中断标志位
  }
}


void rangefinder_init(void)
{	
  if(Flight_Params.health[GROUND_DISTANCE_DEFAULT]==false)  maplepilot.rangefinder_sensor=TOFSENSE;
  else maplepilot.rangefinder_sensor=Flight_Params.parameter_table[GROUND_DISTANCE_DEFAULT];
  switch(maplepilot.rangefinder_sensor)
  {
  case TOFSENSE:
    {
      uart3_init(921600);//921600
    }
    break;
  case VL53L8:
    {
      vl53l8x_start();
      //uart3_init(460800);
      //SMD15_init(460800);
    }
    break;
  case UP_Tx:
    {
      uart3_init(115200);
    }
    break;	  
  case MT1:
    {
      uart3_init(115200);
    }
    break;
  case GYTOF10M:
    {
      uart3_init(9600);
      gytof10m_setBaud(115200);
      delay_ms(3000);
      //gytof10m_setOutputHz(50);
      gytof10m_saveConfig();
      //USART_DeInit(USART3);
      delay_ms(200);
      uart3_init(115200);
    }
    break;
  case TFMINI:
    {
      uart3_init(115200);
    }
    break;
  default:{}
  }	
}


extern VL53L8CX_Configuration 	Dev;
extern VL53L8CX_ResultsData 	Results;
systime vl53l8cx_duty_dt;
float vl53lcx_distance_cm_4x4[16];
void vl53l8cx_get_distance(void)
{
  if(maplepilot.rangefinder_sensor==VL53L8)
  {
    uint8_t status,isReady;
    status = vl53l8cx_check_data_ready(&Dev, &isReady);
    if(isReady)
    {
       vl53l8cx_get_ranging_data(&Dev, &Results);
       get_systime(&vl53l8cx_duty_dt);
       
      //赋值给观测高度观测量
      float tmp=0;
      uint8_t status=(Results.target_status[5]==0x05)
                    |(Results.target_status[6]==0x05)
                    |(Results.target_status[9]==0x05)
                    |(Results.target_status[10]==0x05)
                    |(Results.target_status[0]==0x05)
                    |(Results.target_status[3]==0x05)
                    |(Results.target_status[12]==0x05)
                    |(Results.target_status[15]==0x05);
                  
      if(status==0) return;//至少有一个数据未超量程

      //简单取最大测量值作为高度观测值
      if(Results.target_status[5]==0x05)  tmp=fmaxf(tmp,Results.distance_mm[5]);
      if(Results.target_status[6]==0x05)  tmp=fmaxf(tmp,Results.distance_mm[6]);
      if(Results.target_status[9]==0x05)  tmp=fmaxf(tmp,Results.distance_mm[9]);
      if(Results.target_status[10]==0x05) tmp=fmaxf(tmp,Results.distance_mm[10]);

      if(Results.target_status[0]==0x05)  tmp=fmaxf(tmp,Results.distance_mm[0]);
      if(Results.target_status[3]==0x05)  tmp=fmaxf(tmp,Results.distance_mm[3]);
      if(Results.target_status[12]==0x05) tmp=fmaxf(tmp,Results.distance_mm[12]);
      if(Results.target_status[15]==0x05) tmp=fmaxf(tmp,Results.distance_mm[15]);

      //
      tofsense_data.distance=0.1f*tmp;//cm	
      tofsense_data.update_flag=1;
      if(tofsense_data.distance<360&&tofsense_data.distance>0.001f)	tofsense_data.valid=1;
      else tofsense_data.valid=0;
      tofsense_data.period_ms=vl53l8cx_duty_dt.period_int;	
    }
  }
}


