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


#include <stdint.h>
#include <math.h>
#include <wp_math.h>
#include "datatype.h"
#include "schedule.h"
#include "filter.h"
#include "pid.h"

maple_controller maple_ctrl;

/*
1偏差限幅标志；  2积分限幅标志；3积分分离标志；   4期望；
5反馈            6偏差；        7上次偏差；       8偏差限幅值；
9积分分离偏差值；10积分值       11积分限幅值；    12控制参数Kp；
13控制参数Ki；   14控制参数Kd； 15控制器总输出；  16上次控制器总输出
17总输出限幅度
*/
const float ctrl_params[20][17]=
{
  /*                                         Kp        Ki        Kd            */
  /*1  2  3  4  5  6   7  8   9   10  11    12        13        14  15  16  17    18*/
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,600 ,0  ,0 , 400,  0.80   ,2.00  ,4.00  ,0  ,0 , 1000 },//Roll_Gyro;偏航角速度
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,600 ,0  ,0 , 400,  0.80   ,2.00  ,4.00  ,0  ,0 , 1000 },//Pitch_Gyro;横滚角速度	
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,300 ,0  ,0 , 300,  1.60   ,1.00  ,1.50  ,0  ,0 , 800  },//Yaw_Gyro;偏航角速度	
  
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,30  ,0  ,0 , 100,  5.00   ,0.00  ,0.00  ,0  ,0 , 500  },//Roll_Angle;偏航角度
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,30  ,0  ,0 , 100,  5.00   ,0.00  ,0.00  ,0  ,0 , 500  },//Pitch_Angle;横滚角
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,45  ,0  ,0 , 200 , 5.00   ,0.00  ,0.00  ,0  ,0 , 500  },//Yaw_Angle;偏航角
  //定高参数
  //高度单项比例控制，有偏差限幅、总输出即为最大攀升、下降速度400cm/s
  //Z轴速度比例+积分控制，无偏差限幅
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,200 ,0  ,0 ,100 ,  1.00     ,0.000   ,0    ,0  ,0 ,500 },//竖直高度位置
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,600 ,0  ,0 ,500 ,  5.00     ,0.000   ,0    ,0  ,0 ,1000},//竖直攀升速度		
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,1000,0  ,0 ,750 ,  0.10     ,2.000   ,0    ,0  ,0 ,1000},//竖直加速度   imax 1000
  
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,180 ,0  ,0 ,8,   0.300    ,0.000    ,0    ,0    ,0 ,300},//Longitude_Position;水平经度位置
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,500 ,0  ,0 ,450, 2.400    ,0.600    ,0    ,0    ,0 ,800},//Longitude_Speed;水平经度速度
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,180 ,0  ,0 ,8,   0.300    ,0.000    ,0    ,0    ,0 ,300},//Latitude_Position;水平纬度位置
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,500 ,0  ,0 ,450, 2.400    ,0.600    ,0    ,0    ,0 ,800},//Latitude_Speed;水平纬度速度
  
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,50    ,30  ,0 ,100, 0.50  ,0.00   ,2.5 ,0   ,0 ,300  },//SDK位置X控制器   2.5  150
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,50    ,30  ,0 ,100, 0.50  ,0.00   ,2.5 ,0   ,0 ,300  },//SDK位置Y控制器   2.5  150
  
  /*************光流位置、速度控制器****************/	
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100   ,15  ,0 ,50,  0.30  ,0.00  ,0   ,0   ,0 ,100 },//光流水平X位置控制器
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100   ,50  ,0 ,200, 2.50  ,0.60  ,0   ,0   ,0 ,450 },//光流水平X速度控制器
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100   ,15  ,0 ,50,  0.30  ,0.00  ,0   ,0   ,0 ,100 },//光流水平Y位置控制器
  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100   ,50  ,0 ,200, 2.50  ,0.60  ,0   ,0   ,0 ,450 },//光流水平Y速度控制器
  //温度控制器	
  {1  ,1 ,1 ,0 ,0 ,0 , 0 ,50    ,30  ,0 ,80,  20.0  ,3.0   ,600.0  ,0    ,0 ,100 }
  /*
  1偏差限幅标志；  2积分限幅标志；3积分分离标志；   4期望；					5反馈            6偏差；        7上次偏差；       8偏差限幅值；
  9积分分离偏差值；10积分值       11积分限幅值；    12控制参数Kp；	13控制参数Ki；   14控制参数Kd； 15控制器总输出；  16上次控制器总输出
  17总输出限幅度； 18变积分控制时的积分增益
  */
};


void set_d_lpf_alpha(int16_t cutoff_frequency, float time_step,float *_d_lpf_alpha)
{    
  // calculate alpha
  float rc = 1/(2*PI*cutoff_frequency);
  *_d_lpf_alpha = time_step / (time_step + rc);
}

void pid_init_default(void)
{
  pid_params_init(&maple_ctrl.roll_angle_ctrl,_roll_angle_ctrl);
  pid_params_init(&maple_ctrl.pitch_gyro_ctrl,_pitch_gyro_ctrl);
  pid_params_init(&maple_ctrl.yaw_angle_ctrl,_yaw_angle_ctrl);
  
  pid_params_init(&maple_ctrl.roll_gyro_ctrl,_roll_gyro_ctrl);
  pid_params_init(&maple_ctrl.pitch_angle_ctrl,_pitch_angle_ctrl);
  pid_params_init(&maple_ctrl.yaw_gyro_ctrl,_yaw_gyro_ctrl);
  
  pid_params_init(&maple_ctrl.height_position_ctrl,_height_position_ctrl);
  pid_params_init(&maple_ctrl.height_speed_ctrl,_height_speed_ctrl);
  pid_params_init(&maple_ctrl.height_accel_ctrl,_height_accel_ctrl);
  
  pid_params_init(&maple_ctrl.longitude_position_ctrl,_longitude_position_ctrl);
  pid_params_init(&maple_ctrl.longitude_speed_ctrl,_longitude_speed_ctrl);
  pid_params_init(&maple_ctrl.latitude_position_ctrl,_latitude_position_ctrl);
  pid_params_init(&maple_ctrl.latitude_speed_ctrl,_latitude_speed_ctrl);
  
  pid_params_init(&maple_ctrl.sdk_position_ctrl_x,_sdk_position_ctrl_x);
  pid_params_init(&maple_ctrl.sdk_position_ctrl_y,_sdk_position_ctrl_y);	
  
  pid_params_init(&maple_ctrl.optical_position_ctrl_x,_optical_position_ctrl_x);
  pid_params_init(&maple_ctrl.optical_speed_ctrl_x,_optical_speed_ctrl_x);
  pid_params_init(&maple_ctrl.optical_position_ctrl_y,_optical_position_ctrl_y);
  pid_params_init(&maple_ctrl.optical_speed_ctrl_y,_optical_speed_ctrl_y);
  
  pid_params_init(&maple_ctrl.imu_temperature_ctrl,_imu_temperature_ctrl);
  
  set_cutoff_frequency(sampling_frequent, 50,&maple_ctrl.pitch_gyro_ctrl.lpf_param);//姿态角速度控制器低通滤波器设计  15 20
  set_cutoff_frequency(sampling_frequent, 50,&maple_ctrl.roll_gyro_ctrl.lpf_param);
  set_cutoff_frequency(sampling_frequent, 50,&maple_ctrl.yaw_gyro_ctrl.lpf_param);
  
  set_d_lpf_alpha(40,1.0f/sampling_frequent,&maple_ctrl.pitch_gyro_ctrl.d_lpf_alpha);
  set_d_lpf_alpha(40,1.0f/sampling_frequent,&maple_ctrl.roll_gyro_ctrl.d_lpf_alpha);
  set_d_lpf_alpha(40,1.0f/sampling_frequent,&maple_ctrl.yaw_gyro_ctrl.d_lpf_alpha);
  
  
  set_cutoff_frequency(sampling_frequent, 5,&maple_ctrl.height_accel_ctrl.lpf_param);//5 10  
  
  set_cutoff_frequency(50, 20,&maple_ctrl.latitude_position_ctrl.lpf_param);
  set_cutoff_frequency(50, 20,&maple_ctrl.longitude_speed_ctrl.lpf_param);
  
  //SDK位置控制器低通滤波器设计
  set_cutoff_frequency(100, 10 ,&maple_ctrl.sdk_position_ctrl_x.lpf_param);
  set_cutoff_frequency(100, 10 ,&maple_ctrl.sdk_position_ctrl_y.lpf_param);	
  //光流速度控制器低通滤波器设计
  set_cutoff_frequency(100, 20,&maple_ctrl.optical_speed_ctrl_x.lpf_param);
  set_cutoff_frequency(100, 20,&maple_ctrl.optical_speed_ctrl_y.lpf_param);
  //IMU温度控制器低通滤波器设计
  set_cutoff_frequency(sampling_frequent, 20,&maple_ctrl.imu_temperature_ctrl.lpf_param);
}


float pid_ctrl_general(pid_ctrl *ctrl,float period_second)
{
  float _dt=0;
  get_systime(&ctrl->_time);
  _dt=ctrl->_time.period/1000.0f;
  
  if(_dt>1.05f*period_second||_dt<0.95f*period_second||isnan(_dt)!=0)   _dt=period_second;
  if(_dt<0.0001f) return 0;
  
  /*******偏差计算*********************/
  ctrl->last_err=ctrl->err;//保存上次偏差
  ctrl->err=ctrl->expect-ctrl->feedback;//期望减去反馈得到偏差
  ctrl->dis_err=ctrl->err-ctrl->last_err;
  
  if(ctrl->err_limit_flag==1)	
    ctrl->err=constrain_float(ctrl->err,-ctrl->err_max,ctrl->err_max);//偏差限幅度标志位
  
  /*******积分计算*********************/
  if(ctrl->integrate_separation_flag==1)//积分分离标志位
  {
    //积分分离--只在偏差较小的时候引入积分，避免系统超调
    if(FastAbs(ctrl->err)<=ctrl->integrate_separation_err)  
      ctrl->integrate+=ctrl->ki*ctrl->err*_dt;
  }
  else
  {
    ctrl->integrate+=ctrl->ki*ctrl->err*_dt;
  }
  /*******积分限幅*********************/
  if(ctrl->integrate_limit_flag==1)	ctrl->integrate=constrain_float(ctrl->integrate,-ctrl->integrate_max,ctrl->integrate_max);//积分总输出限制幅度标志
  /*******总输出计算*********************/
  ctrl->last_control_output=ctrl->control_output;//输出值递推
  ctrl->control_output=	ctrl->kp*ctrl->err//比例
    +ctrl->integrate//积分
      +ctrl->kd*ctrl->dis_err;//微分
  /*******总输出限幅*********************/
  ctrl->control_output=constrain_float(ctrl->control_output,-ctrl->control_output_limit,ctrl->control_output_limit);
  /*******返回总输出*********************/
  return ctrl->control_output;
}





float pid_ctrl_rpy_gyro(pid_ctrl *ctrl,float period_second,diff_mode _diff_mode,lpf_mode _lpf_mode)
{
  uint16_t  i=0;
  float _dt=0;
  get_systime(&ctrl->_time);
  _dt=ctrl->_time.period/1000.0f;
  
  if(_dt>1.05f*period_second||_dt<0.95f*period_second||isnan(_dt)!=0)   _dt=period_second;
  if(_dt<0.0001f) return 0;
  
  /*******偏差计算*********************/
  ctrl->pre_last_err=ctrl->last_err;//上上次偏差
  ctrl->last_err=ctrl->err;//保存上次偏差
  ctrl->err=ctrl->expect-ctrl->feedback;//期望减去反馈得到偏差
  
  ctrl->expect_div   = ctrl->expect     - ctrl->last_expect;   //期望的微分
  ctrl->last_expect	 = ctrl->expect;												   //记录上次期望
  ctrl->feedback_div  = ctrl->feedback   - ctrl->last_feedback;//反馈的微分
  ctrl->last_feedback = ctrl->feedback;												 //记录上次反馈值
  ctrl->combine_div  = ctrl->expect_div - ctrl->feedback_div;  //组合微分
  
  
  if(_diff_mode==interval_diff)//区间微分——对期望和反馈单独微分后作差，其中期望微分的周期可根据角度外环给定来
  {
    ctrl->dis_err=ctrl->combine_div;
  }
  else if(_diff_mode==incomplete_diff)//微分先行——只对反馈信号微分
  {
    ctrl->dis_err=-ctrl->feedback_div;
  }
  else ctrl->dis_err=ctrl->err-ctrl->last_err;//原始微分，也可以用间隔了一次采样的微分
  
  
  ctrl->dis_err=constrain_float(ctrl->dis_err,-50.0f,50.0f);//原始微分项限幅	
  
  for(i=4;i>0;i--)//数字低通后微分项保存
  {
    ctrl->dis_error_history[i]=ctrl->dis_error_history[i-1];
  }
  ctrl->dis_error_history[0]=butterworth(ctrl->dis_err,
                                         &ctrl->lpf_buf,
                                         &ctrl->lpf_param);//巴特沃斯低通后得到的微分项	
  
  //对微分项进行,一阶低通滤波
  ctrl->derivative = ctrl->dis_err;
  ctrl->derivative = ctrl->last_derivative + ctrl->d_lpf_alpha * (ctrl->derivative - ctrl->last_derivative);
  ctrl->last_derivative = ctrl->derivative;
  
  //选择微分信号滤波方式
  switch(_lpf_mode)
  {
  case first_order_lpf://一阶低通
    {
      ctrl->dis_err_lpf=ctrl->derivative;
    }
    break;
  case second_order_lpf://二阶低通	
    {
      ctrl->dis_err_lpf=ctrl->dis_error_history[0];
    }
    break;
  case noneed_lpf://不低通	
    {
      ctrl->dis_err_lpf=ctrl->dis_err;
    }
    break;     
  default:ctrl->dis_err_lpf=ctrl->dis_err;
  }
  
  
  if(ctrl->err_limit_flag==1)	ctrl->err=constrain_float(ctrl->err,-ctrl->err_max,ctrl->err_max);//偏差限幅度标志位
  /*******积分计算*********************/
  if(ctrl->integrate_separation_flag==1)//积分分离标志位
  {
    if(FastAbs(ctrl->err)<=ctrl->integrate_separation_err) 
      ctrl->integrate+=ctrl->ki*ctrl->err*_dt;
  }
  else	ctrl->integrate+=ctrl->ki*ctrl->err*_dt;
  /*******积分限幅*********************/
  if(ctrl->integrate_limit_flag==1)	
    ctrl->integrate=constrain_float(ctrl->integrate,-ctrl->integrate_max,ctrl->integrate_max);//积分总输出限制幅度标志
  
  /*******总输出计算*********************/
  ctrl->last_control_output=ctrl->control_output;//输出值递推
  ctrl->control_output=ctrl->kp*ctrl->err+ctrl->integrate;//比例+积分
  ctrl->control_output+=ctrl->kd*constrain_float(ctrl->dis_err_lpf,-25.0f,25.0f);//叠加微分控制		
  
  /*******总输出限幅*********************/
  ctrl->control_output=constrain_float(ctrl->control_output,-ctrl->control_output_limit,ctrl->control_output_limit);
  /*******返回总输出*********************/
  return ctrl->control_output;
}

float pid_ctrl_yaw(pid_ctrl *ctrl,float period_second)
{
  float _dt=0;
  get_systime(&ctrl->_time);
  _dt=ctrl->_time.period/1000.0f;
  if(_dt>1.05f*period_second||_dt<0.95f*period_second||isnan(_dt)!=0)   _dt=period_second;
  if(_dt<0.0001f) return 0;
  
  /*******偏差计算*********************/
  ctrl->last_err=ctrl->err;//保存上次偏差
  ctrl->err=ctrl->expect-ctrl->feedback;//期望减去反馈得到偏差
  ctrl->dis_err=ctrl->err-ctrl->last_err;
  
  /***********************偏航角偏差超过+-180处理*****************************/
  if(ctrl->err<-180) ctrl->err=ctrl->err+360;
  if(ctrl->err>180)  ctrl->err=ctrl->err-360;
  
  if(ctrl->err_limit_flag==1)//偏差限幅度标志位
    ctrl->err=constrain_float(ctrl->err,-ctrl->err_max,ctrl->err_max);//偏差限幅度标志位
  /*******积分计算*********************/
  if(ctrl->integrate_separation_flag==1)//积分分离标志位
  {
    if(FastAbs(ctrl->err)<=ctrl->integrate_separation_err)
      ctrl->integrate+=ctrl->ki*ctrl->err*_dt;
  }
  else
  {
    ctrl->integrate+=ctrl->ki*ctrl->err*_dt;
  }
  /*******积分限幅*********************/
  if(ctrl->integrate_limit_flag==1)//积分限制幅度标志
    ctrl->integrate=constrain_float(ctrl->integrate,-ctrl->integrate_max,ctrl->integrate_max);//积分总输出限制幅度标志
  
  /*******总输出计算*********************/
  ctrl->last_control_output=ctrl->control_output;//输出值递推
  ctrl->control_output=ctrl->kp*ctrl->err//比例
    +ctrl->integrate//积分
      +ctrl->kd*ctrl->dis_err;//微分
  /*******总输出限幅*********************/
  ctrl->control_output=constrain_float(ctrl->control_output,-ctrl->control_output_limit,ctrl->control_output_limit);
  /*******返回总输出*********************/
  return ctrl->control_output;
}


float pid_ctrl_err_lpf(pid_ctrl *ctrl,float period_second)
{
  float _dt=0;
  get_systime(&ctrl->_time);
  _dt=ctrl->_time.period/1000.0f;
  
  if(_dt>1.05f*period_second||_dt<0.95f*period_second||isnan(_dt)!=0)   _dt=period_second;
  if(_dt<0.0001f) return 0;
  
  /*******偏差计算*********************/
  ctrl->pre_last_err=ctrl->last_err;//上上次偏差
  ctrl->last_err=ctrl->err;//保存上次偏差
  ctrl->err=ctrl->expect - ctrl->feedback;//期望减去反馈得到偏差
  ctrl->dis_err=ctrl->err - ctrl->last_err;//原始微分
  ctrl->last_feedback=ctrl->feedback;//记录上次反馈值
  
  ctrl->last_err_lpf=ctrl->err_lpf;
  ctrl->err_lpf=butterworth( ctrl->err,&ctrl->lpf_buf,&ctrl->lpf_param);
  ctrl->dis_err_lpf=ctrl->err_lpf-ctrl->last_err_lpf;
  
  if(ctrl->err_limit_flag==1)	ctrl->err_lpf=constrain_float(ctrl->err_lpf,-ctrl->err_max,ctrl->err_max);//偏差限幅度标志位
  /*******积分计算*********************/
  if(ctrl->integrate_separation_flag==1)//积分分离标志位
  {
    if(FastAbs(ctrl->err_lpf)<=ctrl->integrate_separation_err) 	ctrl->integrate+=ctrl->ki*ctrl->err_lpf*_dt;
  }
  else	ctrl->integrate+=ctrl->ki*ctrl->err_lpf*_dt;
  /*******积分限幅*********************/
  if(ctrl->integrate_limit_flag==1)	
    ctrl->integrate=constrain_float(ctrl->integrate,-ctrl->integrate_max,ctrl->integrate_max);//积分总输出限制幅度标志
  /*******总输出计算*********************/
  ctrl->last_control_output=ctrl->control_output;//输出值递推
  ctrl->control_output=ctrl->kp*ctrl->err_lpf//比例
    +ctrl->integrate//积分
      +ctrl->kd*ctrl->dis_err_lpf;//微分项来源于巴特沃斯低通滤波器
  /*******总输出限幅*********************/
  ctrl->control_output=constrain_float(ctrl->control_output,-ctrl->control_output_limit,ctrl->control_output_limit);
  /*******返回总输出*********************/
  return ctrl->control_output;
}


float pid_ctrl_div_lpf(pid_ctrl *ctrl,float period_second)
{
  float _dt=0;
  get_systime(&ctrl->_time);
  _dt=ctrl->_time.period/1000.0f;
  
  if(_dt>1.05f*period_second||_dt<0.95f*period_second||isnan(_dt)!=0)   _dt=period_second;
  if(_dt<0.0001f) return 0;
  
  /*******偏差计算*********************/
  ctrl->pre_last_err=ctrl->last_err;//上上次偏差
  ctrl->last_err=ctrl->err;//保存上次偏差
  ctrl->err=ctrl->expect-ctrl->feedback;//期望减去反馈得到偏差
  ctrl->dis_err=ctrl->err-ctrl->last_err;//原始微分 ctrl->dis_err=(ctrl->err-ctrl->pre_last_err);//间隔了一次采样的微分
  ctrl->last_feedback=ctrl->feedback;//记录上次反馈值
  
  for(uint16_t i=4;i>0;i--)//数字低通后微分项保存
  {
    ctrl->dis_error_history[i]=ctrl->dis_error_history[i-1];
  }
  ctrl->dis_error_history[0]=butterworth(ctrl->dis_err,
                                         &ctrl->lpf_buf,
                                         &ctrl->lpf_param);//巴特沃斯低通后得到的微分项
  ctrl->dis_err_lpf=ctrl->dis_error_history[0];
  
  
  if(ctrl->err_limit_flag==1)	
    ctrl->err=constrain_float(ctrl->err,-ctrl->err_max,ctrl->err_max);//偏差限幅度标志位
  /*******积分计算*********************/
  if(ctrl->integrate_separation_flag==1)//积分分离标志位
  {
    if(FastAbs(ctrl->err)<=ctrl->integrate_separation_err) 
      ctrl->integrate+=ctrl->ki*ctrl->err*_dt;
  }
  else	ctrl->integrate+=ctrl->ki*ctrl->err*_dt;
  /*******积分限幅*********************/
  if(ctrl->integrate_limit_flag==1)	
    ctrl->integrate=constrain_float(ctrl->integrate,-ctrl->integrate_max,ctrl->integrate_max);//积分总输出限制幅度标志
  /*******总输出计算*********************/
  ctrl->last_control_output=ctrl->control_output;//输出值递推
  ctrl->control_output=ctrl->kp*ctrl->err//比例
    +ctrl->integrate//积分
      +ctrl->kd*ctrl->dis_err_lpf;//微分项来源于巴特沃斯低通滤波器
  /*******总输出限幅*********************/
  ctrl->control_output=constrain_float(ctrl->control_output,-ctrl->control_output_limit,ctrl->control_output_limit);
  /*******返回总输出*********************/
  return ctrl->control_output;
}

float pid_ctrl_div_gyro_lpf(pid_ctrl *ctrl,float period_second)
{
  float _dt=0;
  get_systime(&ctrl->_time);
  _dt=ctrl->_time.period/1000.0f;
  
  if(_dt>1.05f*period_second||_dt<0.95f*period_second||isnan(_dt)!=0)   _dt=period_second;
  if(_dt<0.0001f) return 0;
  
  /*******偏差计算*********************/
  ctrl->pre_last_err=ctrl->last_err;//上上次偏差
  ctrl->last_err=ctrl->err;//保存上次偏差
  ctrl->err=ctrl->expect-ctrl->feedback;//期望减去反馈得到偏差
  ctrl->dis_err=ctrl->err-ctrl->last_err;//原始微分 ctrl->dis_err=(ctrl->err-ctrl->pre_last_err);//间隔了一次采样的微分
  ctrl->dis_err=constrain_float(ctrl->dis_err,-25.0f,25.0f);
  ctrl->last_feedback=ctrl->feedback;//记录上次反馈值
  
  for(uint16_t i=4;i>0;i--)//数字低通后微分项保存
  {
    ctrl->dis_error_history[i]=ctrl->dis_error_history[i-1];
  }
  ctrl->dis_error_history[0]=butterworth(ctrl->dis_err,
                                         &ctrl->lpf_buf,
                                         &ctrl->lpf_param);//巴特沃斯低通后得到的微分项
  ctrl->dis_error_history[0]=constrain_float(ctrl->dis_error_history[0],-500,500);//微分项限幅
  ctrl->dis_err_lpf=ctrl->dis_error_history[0];
  
  
  if(ctrl->err_limit_flag==1)	
    ctrl->err=constrain_float(ctrl->err,-ctrl->err_max,ctrl->err_max);//偏差限幅度标志位
  /*******积分计算*********************/
  if(ctrl->integrate_separation_flag==1)//积分分离标志位
  {
    if(FastAbs(ctrl->err)<=ctrl->integrate_separation_err) 
      ctrl->integrate+=ctrl->ki*ctrl->err*_dt;
  }
  else	ctrl->integrate+=ctrl->ki*ctrl->err*_dt;
  /*******积分限幅*********************/
  if(ctrl->integrate_limit_flag==1)	
    ctrl->integrate=constrain_float(ctrl->integrate,-ctrl->integrate_max,ctrl->integrate_max);//积分总输出限制幅度标志
  /*******总输出计算*********************/
  ctrl->last_control_output=ctrl->control_output;//输出值递推
  ctrl->control_output	=	ctrl->kp*ctrl->err//比例
    +	ctrl->integrate//积分
      +	ctrl->kd*ctrl->dis_err_lpf;//微分项来源于巴特沃斯低通滤波器
  /*******总输出限幅*********************/
  ctrl->control_output=constrain_float(ctrl->control_output,-ctrl->control_output_limit,ctrl->control_output_limit);
  /*******返回总输出*********************/
  return ctrl->control_output;
}






void pid_params_init(pid_ctrl *ctrl,ctrl_name n)
{
  ctrl->err_limit_flag=(uint8_t)(ctrl_params[n][0]);//1偏差限幅标志
  ctrl->integrate_limit_flag=(uint8_t)(ctrl_params[n][1]);//2积分限幅标志
  ctrl->integrate_separation_flag=(uint8_t)(ctrl_params[n][2]);//3积分分离标志
  ctrl->expect=ctrl_params[n][3];//4期望
  ctrl->feedback=ctrl_params[n][4];//5反馈值
  ctrl->err=ctrl_params[n][5];//6偏差
  ctrl->last_err=ctrl_params[n][6];//7上次偏差
  ctrl->err_max=ctrl_params[n][7];//8偏差限幅值
  ctrl->integrate_separation_err=ctrl_params[n][8];//9积分分离偏差值
  ctrl->integrate=ctrl_params[n][9];//10积分值
  ctrl->integrate_max=ctrl_params[n][10];//11积分限幅值
  ctrl->kp=ctrl_params[n][11];//12控制参数Kp
  ctrl->ki=ctrl_params[n][12];//13控制参数Ki
  ctrl->kd=ctrl_params[n][13];//14控制参数Ki
  ctrl->control_output=ctrl_params[n][14];//15控制器总输出
  ctrl->last_control_output=ctrl_params[n][15];//16上次控制器总输出
  ctrl->control_output_limit=ctrl_params[n][16];//17上次控制器总输出
  ctrl->name=n;
}



void  pid_integrate_reset(pid_ctrl *ctrl)  {ctrl->integrate=0.0f;}


void takeoff_ctrl_reset(void)
{
  pid_integrate_reset(&maple_ctrl.roll_gyro_ctrl);//起飞前屏蔽积分
  pid_integrate_reset(&maple_ctrl.pitch_gyro_ctrl);
  pid_integrate_reset(&maple_ctrl.yaw_gyro_ctrl);
  pid_integrate_reset(&maple_ctrl.roll_angle_ctrl);
  pid_integrate_reset(&maple_ctrl.pitch_angle_ctrl);
  pid_integrate_reset(&maple_ctrl.yaw_angle_ctrl);
  
  pid_integrate_reset(&maple_ctrl.height_position_ctrl);
  //pid_integrate_reset(&maple_ctrl.height_speed_ctrl);
  //pid_integrate_reset(&maple_ctrl.height_accel_ctrl);//单独处理
  
  //水平位置、控制速度环积分项
  pid_integrate_reset(&maple_ctrl.longitude_position_ctrl);
  pid_integrate_reset(&maple_ctrl.longitude_speed_ctrl);
  pid_integrate_reset(&maple_ctrl.latitude_position_ctrl);
  pid_integrate_reset(&maple_ctrl.latitude_speed_ctrl);	
  
  pid_integrate_reset(&maple_ctrl.optical_position_ctrl_x);
  pid_integrate_reset(&maple_ctrl.optical_speed_ctrl_x);
  pid_integrate_reset(&maple_ctrl.optical_position_ctrl_y);
  pid_integrate_reset(&maple_ctrl.optical_speed_ctrl_y);	
  
  pid_integrate_reset(&maple_ctrl.sdk_position_ctrl_x);
  pid_integrate_reset(&maple_ctrl.sdk_position_ctrl_y);	
}



void optical_ctrl_reset(uint8_t fixed)
{
  pid_integrate_reset(&maple_ctrl.optical_position_ctrl_x);
  pid_integrate_reset(&maple_ctrl.optical_position_ctrl_y);
  pid_integrate_reset(&maple_ctrl.optical_speed_ctrl_x);
  pid_integrate_reset(&maple_ctrl.optical_speed_ctrl_y);	
  
  maple_ctrl.optical_speed_ctrl_x.expect=0;
  maple_ctrl.optical_speed_ctrl_y.expect=0;
  if(fixed==0)
  {
    maple_ctrl.optical_position_ctrl_x.expect=0;
    maple_ctrl.optical_position_ctrl_y.expect=0;
  }
  else
  {
    maple_ctrl.optical_position_ctrl_x.expect=ins.opt.position.x;//等效正东方向位置
    maple_ctrl.optical_position_ctrl_y.expect=ins.opt.position.y;//等效正北方向位置	
  }
  
}









