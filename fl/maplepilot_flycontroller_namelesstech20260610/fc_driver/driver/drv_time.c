#include "stm32f10x.h"
#include "datatype.h"
#include "schedule.h"
#include "nclink.h"
#include "rc.h"
#include "parameter_server.h"
#include "calibration.h"
#include "fattester.h"
#include "offboard.h"
#include "drv_time.h"


extern void maple_duty_200hz(void);
extern void Vcan_Send(void);

systime _t1;
void TIM1_UP_IRQHandler(void)//5ms刷新一次
{
  if( TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET )
  {
		get_systime(&_t1);
		maple_duty_200hz();
    TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
  }
}

extern void rgb_notify_work(void);
systime _t2;
void TIM2_IRQHandler(void)//10ms
{
  if( TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET )
  {
		get_systime(&_t2);
		NCLink_SEND_StateMachine();
		//Vcan_Send();
		accel_calibration(flymaple.total_gyro_dps);
		mag_calibration();
		hor_calibration();
		gyro_calibration(&flymaple.imu_gyro_calibration_flag);		
		pid_parameter_server();
		rgb_notify_work();										//RGB状态指示灯运行
    TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update);
  }
}


systime _t6;
void TIM6_IRQHandler(void)//100ms
{
  if( TIM_GetITStatus(TIM6,TIM_IT_Update)!=RESET )
  {
		get_systime(&_t6);
#if flight_log_enable
		flymaple_logger();//SD卡写飞行日志文件
#endif
    TIM_ClearITPendingBit(TIM6,TIM_FLAG_Update);
  }
}

systime _t7;
void TIM7_IRQHandler(void)//100ms
{
  if( TIM_GetITStatus(TIM7,TIM_IT_Update)!=RESET )
  {
		get_systime(&_t7);
		static uint16_t _cnt=0;	_cnt++;
		if(_cnt>=5&&other_params.params.inner_uart==2)
		{
			pilot_send_to_planner(ins.opt.position.x       ,ins.opt.position.y        ,ins.position_z,
														ins.opt.speed.x          ,ins.opt.speed.y           ,ins.speed_z,
														ins.accel_feedback[EAST] ,ins.accel_feedback[NORTH] ,ins.accel_feedback[_UP],
														flymaple.gyro_dps.y     ,flymaple.gyro_dps.x      ,flymaple.gyro_dps.z,
														flymaple.quaternion[0]       ,flymaple.quaternion[1]        ,flymaple.quaternion[2],			flymaple.quaternion[3],
														flymaple.quaternion_init_ok  ,current_state.rec_update_flag);
			_cnt=0;
		} 	
    TIM_ClearITPendingBit(TIM7,TIM_FLAG_Update);
  }
}   


void timer1_configuration(uint32_t us)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_DeInit(TIM1);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  TIM_TimeBaseStructure.TIM_Period=us-1;
  TIM_TimeBaseStructure.TIM_Prescaler= (72-1);
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);
  TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_Trigger,ENABLE);
  TIM_Cmd(TIM1, ENABLE);
	
	//飞控任务调度定时器
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=timer1_irq_chl_pre_priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=timer1_irq_chl_sub_priority;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


void timer2_configuration(uint32_t us)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_DeInit(TIM2);
  TIM_TimeBaseStructure.TIM_Period = us-1;//10ms
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1; //1us
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=timer2_irq_chl_pre_priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=timer2_irq_chl_sub_priority;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}




void timer6_configuration(uint32_t us)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  TIM_DeInit(TIM6);
  TIM_TimeBaseStructure.TIM_Period = us-1;//10ms
  TIM_TimeBaseStructure.TIM_Prescaler = 7200-1; //1us
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
  
  TIM_ClearFlag(TIM6, TIM_FLAG_Update);
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM6, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=timer6_irq_chl_pre_priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=timer6_irq_chl_sub_priority;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}



void timer7_configuration(uint32_t us)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
  TIM_DeInit(TIM7);
  TIM_TimeBaseStructure.TIM_Period = us-1;
  TIM_TimeBaseStructure.TIM_Prescaler = 7200-1; //100us
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
  
  TIM_ClearFlag(TIM7, TIM_FLAG_Update);
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM7, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=timer7_irq_chl_pre_priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=timer7_irq_chl_sub_priority;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


