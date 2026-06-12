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
#include "schedule.h"
#include "drv_uart.h"
#include "drv_adc.h"
#include "drv_pwm.h"
#include "drv_i2c.h"

#include "datatype.h"
#include "drv_mpu6050.h"
#include "drv_spl06.h"
#include "drv_qmc5883.h"
#include "drv_ppm.h"
#include "drv_at24cxx.h"
#include "drv_w25qxx.h"
#include "drv_oled.h"
#include "drv_notify.h"
#include "drv_expand.h"
#include "drv_button.h"
#include "oled_display.h"
#include "drv_tofsense.h"
#include "drv_opticalflow.h"
#include "drv_gps.h"
#include "adc_button.h"

#include "sensor.h"
#include "quaternion.h"
#include "maple_ahrs.h"
#include "pid.h"
#include "rc.h"
#include "parameter_server.h"
#include "reserved_serialport.h"
#include "calibration.h"
#include "flymaple_sdk.h"
#include "attitude_ctrl.h"
#include "flymaple_ctrl.h"
#include "offboard.h"
#include "nclink.h"
#include "maple_config.h"
#include "vl53l8cx_api.h"
#include "platform.h"

sins ins;         // 无人机位姿状态数据定义
sensor flymaple = // 无人机传感器数据定义
    {
        .imu_gyro_calibration_flag = 0, // 陀螺仪校准标志位
        .accel_calibration_way = 0,     // 加速度计校准模式
};

systime systime1;
int main(void)
{
  clock_init(SYSTEM_CLOCK_160M); // 时钟配置及系统初始化<务必保留>
  systickinit();                 // 滴答定时器初始化
  simulation_pwm_init();         // 模拟pwm初始化-IMU温度IO
  AT24CXX_Init();                // EEPROM初始化
  flight_read_flash_full();      // 读取外部flash内的参数
  i2c_gpio_configuration();      // I2C资源初始化
  esc_ctrl_hz_init();            // 电调pwm控制信号频率读取
  npwm_init(maplepilot.esc_output_frequence);
  rgb_notify_init();                   // RGB状态指示灯初始化
  ppm_init();                          // 遥控器ppm数据解析初始化
  reserved_io_init();                  // 预留IO初始化aaaaaaaaaaaaaaaaaaaaaaa
  nkey_init();                         // 板载按键初始化
  adc_button_init();                   // ADC按键初始化
  noled_init();                        // OLED显示屏初始化
  uart1_init(460800);                  // 串口1初始化-无名创新地面站通讯串口
  reserved_serialport_init();          // 串口2及相关资源外设初始化
  nadc_init();                         // ADC资源初始化
  icm20608x_init();                    // IMU加速度计陀螺仪初始化
  spl06_init();                        // 气压计spl06初始化
  qmc5883l_init();                     // 磁力计qmc5883初始化
  ahrs_ekf_init();                     // ekf姿态解算初始化
  madgwick_init();                     // madgwick姿态算法初始化
  sensor_filter_init();                // 传感器滤波初始化
  pid_init_from_parameter_server();    // pid参数初始化
  rc_init_from_parameter_server();     // 遥控器校准参数初始化
  imu_calibration_init_from_server();  // IMU准参数初始化
  other_parameter_init_from_server(2); // 其它参数初始化
  reserved_params_init_server();       // 预留参数初始化
  offboard_uart5_init();               // 串口5及相关资源外设初始化
  uart5_init(115200);                  // offboard通讯串口

  uart4_optical_flow_init(); // 串口4初始化
  rangefinder_init();        // 串口3及相关资源外设初始化
  flymaple_sdk_init();       // sdk参数初始化

  pit_ms_init(USER1_PIT_NUM, 5);                                                                                 // 定时器1初始化,周期5ms
  pit_ms_init(USER2_PIT_NUM, 5);                                                                                 // 定时器2初始化,周期5ms
  interrupt_set_priority_EncodePriority(PIT_PRIORITY, timerA_irq_chl_pre_priority, timerA_irq_chl_sub_priority); // 任务调度定时器中断优先级设置
  while (1)
  {
    get_systime(&systime1); // 获取main函数系统时间
    oled_show_page();       // 显示屏ui刷新
  }
}

systime _duty1_t1, _duty1_t2;
float duty1_t_max_200hz = 0;
void maple_duty1_200hz(void)
{
  get_systime(&_duty1_t1);
  sensor_raw_update();    // 加速度计/陀螺仪/磁力计/气压计原始数据更新和低通滤波,计算观测角度、气压高度等
  sensor_fusion_update(); // 融合后的姿态角度输出
  opticalflow_pretreat(); // 光流数据解析
  vl53l8cx_get_distance();
  gps_data_prase(&ins.gps_update_flag); // GPS数据解析
  reserved_serialport_prase();          // ROS通讯数据解析
  flymaple_nav_update();                // 惯性导航相关估计
  rc_prase(&ppm_rc);                    // 遥控器数据解析
  flymaple_ctrl();                      // 总控制器运行
  check_calibration();                  // 校准检测
  battery_voltage_detection();          // 电池电压检测
  rgb_notify_work();                    // RGB状态指示灯运行
  laser_light_work(&beep);              // 蜂鸣器状态机
  read_button_state_all();              // 按键检测
  adc_button_scan();                    // ADC按键检测
  NCLink_Send_IMU_Feedback_PC();        // 飞控给机载计算机发送飞行状态
  // 二次开发调试时,用于测试程序运行时间开销
  get_systime(&_duty1_t2);
  float dt = _duty1_t2.current_time - _duty1_t1.current_time;
  if (dt > duty1_t_max_200hz)
    duty1_t_max_200hz = dt;
}

systime _duty2_t1, _duty2_t2;
float duty2_t_max_200hz = 0;
void maple_duty2_200hz(void)
{
  get_systime(&_duty2_t1);
  NCLink_SEND_StateMachine();                            // 无名创新地面站发送
  accel_calibration(flymaple.total_gyro_dps);            // 加速度计校准
  mag_calibration();                                     // 磁力计校准
  hor_calibration();                                     // 机架水平校准
  gyro_calibration(&flymaple.imu_gyro_calibration_flag); // 陀螺仪校准
  pid_parameter_server();                                // pid参数服务
  get_systime(&_duty2_t2);
  float dt = _duty2_t2.current_time - _duty2_t1.current_time;
  if (dt > duty2_t_max_200hz)
    duty2_t_max_200hz = dt;
}
