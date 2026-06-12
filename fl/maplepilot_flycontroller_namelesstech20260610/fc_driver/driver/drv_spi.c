#include "stm32f10x.h"
#include "main.h"
#include "drv_spi.h"



void spi1_init(uint16_t speed)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef   SPI_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO |RCC_APB2Periph_GPIOA,ENABLE);	//开端口时钟、SPI时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP; //推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //设置SPI为双线双向全双工模式
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	 //主机模式
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//发送、接收8位帧结构
  SPI_InitStructure.SPI_CPOL =SPI_CPOL_High ; //始终悬空高  // SPI_CPOL_Low//始终悬空低 
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//第2个时钟沿捕获 //SPI_CPHA_1Edge第1个时钟沿捕获 
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	 //硬件控制NSS信号（ss） 置成软件时,NSS脚可以他用	  // SPI_NSS_Hard 
  if(speed==2)  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  //预分频值为2
  else if(speed==4) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  //预分频值为4
  else if(speed==8) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //预分频值为8
  else if(speed==16) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;  //预分频值为16
  else if(speed==32) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//预分频值为32
  else if(speed==64) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  //预分频值为64
  else if(speed==128) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;  //预分频值为128
  else if(speed==256) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; //预分频值为256
  else  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  //预分频值为4 
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //数据传输由最高位开始	   //SD卡高位先传送
  SPI_InitStructure.SPI_CRCPolynomial = 7;	 //定义了CRC值计算的多项式为7
  SPI_Init(SPI1, &SPI_InitStructure); 
  SPI_Cmd(SPI1,ENABLE); 
}

uint8_t spi1_sendreceive(uint8_t data)     //SPI1的收发
{		  
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);  //while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); 
	SPI_I2S_SendData(SPI1, data);	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	return  SPI_I2S_ReceiveData(SPI1);
} 

/*********************************************************************/
SPI_InitTypeDef   SPI_InitStructure;
void spi2_init(uint16_t speed)
{ 
  GPIO_InitTypeDef GPIO_InitStructure;//定义GPIO结构体
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);   
  //-----SPI2-----//
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;								//PB13-->SCK,PB14-->MISO,PB15-->MOSI
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;  																			//PB13/14/15复用推挽输出
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&GPIO_InitStructure);
  
		
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;							//设置SPI工作模式:设置为主SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//设置SPI的数据大小:SPI发送接收8位帧结构
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;								//串行同步时钟的空闲状态为低电平
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;							//串行同步时钟的第一个跳变沿（上升或下降）数据被采样
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;									//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制

  if(speed==2)  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  //预分频值为2
  else if(speed==4) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  //预分频值为4
  else if(speed==8) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //预分频值为8
  else if(speed==16) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;  //预分频值为16
  else if(speed==32) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//预分频值为32
  else if(speed==64) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  //预分频值为64
  else if(speed==128) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;  //预分频值为128
  else if(speed==256) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; //预分频值为256
  else  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  //预分频值为4  	
	
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
  SPI_InitStructure.SPI_CRCPolynomial = 7;									//CRC值计算的多项式，在全双工模式中CRC作为最后一个字节发送出去
  SPI_Init(SPI2, &SPI_InitStructure);  											//根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器 
  SPI_Cmd(SPI2, ENABLE); 											   						//使能SPI外设
}

void spi2_setspeed(uint16_t SPI_BaudRatePrescaler)
{	
  if(SPI_BaudRatePrescaler==2)  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  //预分频值为2
  else if(SPI_BaudRatePrescaler==4) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  //预分频值为4
  else if(SPI_BaudRatePrescaler==8) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //预分频值为8
  else if(SPI_BaudRatePrescaler==16) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;  //预分频值为16
  else if(SPI_BaudRatePrescaler==32) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//预分频值为32
  else if(SPI_BaudRatePrescaler==64) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  //预分频值为64
  else if(SPI_BaudRatePrescaler==128) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;  //预分频值为128
  else if(SPI_BaudRatePrescaler==256) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; //预分频值为256
  else  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  //预分频值为4  
	
	SPI_Init(SPI2,&SPI_InitStructure);
	SPI_Cmd(SPI2,ENABLE);
}


uint8_t spi2_sendreceive(uint8_t data)
{
  uint8_t retry = 0;
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
  {
    retry++;
    if (retry > 200)	return 0;//超时
  }
  SPI_I2S_SendData(SPI2, data); 							//通过外设SPIx发送一个数据
  retry = 0;
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
  {
    retry++;
    if(retry > 200)	return 0;	//超时
  }
  return SPI_I2S_ReceiveData(SPI2); 						//返回通过SPIx最近接收的数据
}



/********************************************************************************/
void spi3_init(uint16_t speed)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//DMA_InitTypeDef DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE); //禁止JTAG功能（保留SWD下载口）
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);  

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_15; //PA15 =1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	PAout(15)=1; 

	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	
  if(speed==2)  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  //预分频值为2
  else if(speed==4) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  //预分频值为4
  else if(speed==8) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //预分频值为8
  else if(speed==16) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;  //预分频值为16
  else if(speed==32) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//预分频值为32
  else if(speed==64) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  //预分频值为64
  else if(speed==128) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;  //预分频值为128
  else if(speed==256) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; //预分频值为256
  else  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  //预分频值为4  	
	
	
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI3, &SPI_InitStructure);
	
	
//	//使能DMA发送
//	DMA_DeInit(DMA2_Channel2); 
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI3->DR; 
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; 
//	DMA_InitStructure.DMA_BufferSize = 1024; 
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
//	DMA_InitStructure.DMA_Mode =   DMA_Mode_Normal;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 
//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
//	DMA_Init(DMA2_Channel2, &DMA_InitStructure); 
//	//使能DMA接收
//	DMA_DeInit(DMA2_Channel1);
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI3->DR;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//	DMA_Init(DMA2_Channel1, &DMA_InitStructure);
//	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);
//	SPI_I2S_DMACmd(SPI3,SPI_I2S_DMAReq_Tx,ENABLE);
	
	SPI_Cmd(SPI3, ENABLE);
}

// 速度设置
void spi3_setspeed(uint16_t SPI_BaudRatePrescaler)
{	
  if(SPI_BaudRatePrescaler==2)  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  //预分频值为2
  else if(SPI_BaudRatePrescaler==4) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  //预分频值为4
  else if(SPI_BaudRatePrescaler==8) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //预分频值为8
  else if(SPI_BaudRatePrescaler==16) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;  //预分频值为16
  else if(SPI_BaudRatePrescaler==32) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//预分频值为32
  else if(SPI_BaudRatePrescaler==64) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  //预分频值为64
  else if(SPI_BaudRatePrescaler==128) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;  //预分频值为128
  else if(SPI_BaudRatePrescaler==256) SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; //预分频值为256
  else  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  //预分频值为4  
	SPI_Init(SPI3,&SPI_InitStructure);
	SPI_Cmd(SPI3,ENABLE);
}


//CPU发送
uint8_t spi3_sendreceive(uint8_t data)
{		
	while((SPI3->SR&1<<1)==0){;}
	SPI3->DR=data;	 	  		 
	while((SPI3->SR&1<<0)==0){;}	 
	while(SPI3->SR&(1<<7)){;}
	return SPI3->DR;
}


// SPI3 DMA发送
void dma_spi3_tx(uint8_t *buf,uint16_t len)
{
	DMA2->IFCR |=(0xf<<4);
	DMA2_Channel2->CNDTR=len; //设置要传输的数据长度
	DMA2_Channel2->CMAR=(u32)buf; //设置RAM缓冲区地址
	DMA2_Channel2->CCR|=0x1;
	while(!(DMA2->ISR&(1<<5))){;}
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET){;}
	DMA2_Channel2->CCR &=(uint32_t)~0x1;
}


// SPI3 DMA 接收
void dma_spi3_rx(uint8_t *buf,uint16_t len)
{
	SPI3->CR1|=SPI_Direction_2Lines_RxOnly;
	buf[0] = SPI3->DR;// 读取一次数据
	DMA2->IFCR |=(0xf<<0);
	DMA2_Channel1->CNDTR=len; //设置要传输的数据长度
	DMA2_Channel1->CMAR=(u32)buf; //设置RAM缓冲区地址
	DMA2_Channel1->CCR|=0x1;
	while(!(DMA2->ISR&(1<<1))){;}
	DMA2_Channel1->CCR &=(uint32_t)~0x1;
	SPI3->CR1&=~SPI_Direction_2Lines_RxOnly;
}

