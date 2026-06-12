#include "stm32f10x.h"
#include "drv_gpio.h"


_laser_light beep;

/***************************************
函数名:	void GPIO_Init(void)
说明: GPIO初始化
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void ngpio_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  beep.port=GPIOC;
	beep.pin=GPIO_Pin_14;
	beep.period=20;//200*5ms
	beep.light_on_percent=0.5f;
	beep.reset=0;	
}

/***************************************
函数名:	laser_light_work(_laser_light *light)
说明: gpio驱动状态机
入口:	_laser_light *light-gpio控制结构体
出口:	无
备注:	无
作者:	无名创新
***************************************/
void laser_light_work(_laser_light *light)
{
	if(light->reset==1)
	{
		light->reset=0;
		light->cnt=0;
		light->times_cnt=0;//点亮次数计数器
		light->end=0;
	}
	
	if(light->times_cnt==light->times)
	{
		light->end=1;
		return;
	}

	light->cnt++;
	if(light->cnt<=light->period*light->light_on_percent)
	{
		GPIO_SetBits(light->port,light->pin);
	}
	else if(light->cnt<light->period)
	{
		GPIO_ResetBits(light->port,light->pin);
	}
	else//完成点亮一次
	{
		GPIO_ResetBits(light->port,light->pin);
		light->cnt=0;
		light->times_cnt++;
	}
}

