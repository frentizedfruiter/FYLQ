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

#include "attitude_ctrl.h"
#include "altitude_ctrl.h"
#include "nclink.h"
#include "drv_expand.h"
#include "rc.h"
#include "pid.h"
#include "position_ctrl.h"
#include "flymaple_sdk.h"
#include "parameter_server.h"
#include "Subtask_Demo.h"
#include "flymaple_sdk.h"
#include "Developer_Mode.h"

#define OneSecond 200
#define MAN 222 // 降落状态

uint8_t offboard_start_flag = 0; // 外部控制模式启动标志位
// 当飞控Pilot端切入SDK后会将此标志位发送给上位机planner端

void SetFlyCar(float vx, float vy, float vyaw)
{
  maplepilot.roll_outer_control_output = vy;         // 右为正
  maplepilot.pitch_outer_control_output = -1 * (vx); // 后为正
  maplepilot.yaw_ctrl_mode = ROTATE;
  maplepilot.yaw_outer_control_output = vyaw; // 顺指针为正
}
extern int16_t SpeedPacket[3]; // 速度数据缓存
float SpeedT[3];
void Auto_Flight_Ctrl(int16_t *mode)
{
  static uint16_t openmv_work_mode = 0;
  static uint16_t StateTime = 0;     // 状态保持时间
  static uint8_t MyState = 0;        // 自定义状态机状态
  static uint16_t altitude_time = 0; // 维持高度时间
  switch (*mode)
  {
  case 0:
  {
    simple_auto_flight();
  }
  break;
  case 1: // 俯视OPENMV视觉追踪色块
  {
    Color_Block_Control_Pilot(); // 俯视OPENMV视觉水平追踪
    maplepilot.yaw_ctrl_mode = ROTATE;
    maplepilot.yaw_outer_control_output = rc_data.rc_rpyt[RC_YAW];
    flight_altitude_control(ALTHOLD_MANUAL_CTRL, NUL, NUL); // 高度控制
  }
  break;
  case 2: // 俯视OPENMV视觉追踪AprilTag，控制逻辑与追踪色块一致
  {
    Top_APrilTag_Control_Pilot(); // 俯视OPENMV视觉水平追踪
    maplepilot.yaw_ctrl_mode = ROTATE;
    maplepilot.yaw_outer_control_output = rc_data.rc_rpyt[RC_YAW];
    flight_altitude_control(ALTHOLD_MANUAL_CTRL, NUL, NUL); // 高度控制
  }
  break;
  case 3: // 俯视OPENMV循迹控制，默认黑线，阈值可在openmv程序中调整
  {
    Self_Track_Control_Pilot();                             // 循迹控制内含有水平控制+偏航控制
    flight_altitude_control(ALTHOLD_MANUAL_CTRL, NUL, NUL); // 高度控制
  }
  break;
  case 4:
  {
    flight_subtask_1(); // 顺时针转动90度，完成后降落
  }
  break;
  case 5:
  {
    flight_subtask_3(); // 以10deg/s的角速度顺时针转动10000ms，完成后降落
  }
  break;
  case 6:
  {
    flight_subtask_5(); // 机体坐标系下相对位移,正方形轨迹
  }
  break;
  case 7:
  {
    flight_subtask_7(); // 导航坐标系下，基于初始点的绝对坐标位移,正方形轨迹
  }
  break;
  case 8:
  {
    flight_subtask_8(); // 航点飞行轨迹圆，半径、航点数量可设置
  }
  break;
  case 9: // 自动起飞到某一高度
  {
    if (Auto_Takeoff(other_params.params.target_height) == 1) // WORK_HEIGHT_CM
    {
      *mode += 1; // 到达目标高度后，切换到下一SDK任务
      //*mode+=4;//到达目标高度后，切换2022年7月省赛第一部分任务
      //*mode+=5;//到达目标高度后，切换2022年7月省赛第二部分任务
      //*mode+=6;//到达目标高度后，切换2022年7月省赛第三部分任务
    }
  }
  break;
  case 10:
  {
    if (openmv_work_mode == 0) // 只配置一次
    {
      openmv_work_mode = 0x07;
      sdk_send_check(openmv_work_mode, UART5_SDK); // 起飞完毕之后，将底部OPENMV设置成检测农作物模式
    }
    // 2021年电子设计竞赛G题植保无人机
    Agriculture_UAV_Closeloop(); // 基础部分
    // Agriculture_UAV_Innovation();//发挥部分
  }
  break;
  case 11:
  {
    ros_flight_support();
  }
  break;
  case 12:
  {
    basic_auto_flight_support();
  }
  break;
  case 13:
  {
    if (openmv_work_mode == 0) // 只配置一次
    {
      openmv_work_mode = 0x10;
      sdk_send_check(openmv_work_mode, UART5_SDK); // 起飞完毕之后，将底部OPENMV设置成色块、形状检测模式
    }
    // 2022年月电子设计竞赛B题送货无人机——第1部分
    Deliver_UAV_Basic();
  }
  break;
  case 14:
  {
    if (openmv_work_mode == 0) // 只配置一次
    {
      openmv_work_mode = 0x10;
      sdk_send_check(openmv_work_mode, UART5_SDK); // 起飞完毕之后，将底部OPENMV设置成色块、形状检测模式
    }
    // 2022年月电子设计竞赛B题送货无人机——第2部分
    Deliver_UAV_Innovation();
  }
  break;
  case 15:
  {
    // 2022年月电子设计竞赛B题送货无人机——第3部分
    Deliver_UAV_Hulahoop();
  }
  break;
  case 16: // 自动起飞到某一高度
  {
    if (Auto_Takeoff(other_params.params.target_height) == 1)
    {
      *mode += 1; // 到达目标高度后，切换到下一SDK任务
    }
  }
  break;
  case 17: // 用户自定义航点飞行-无需二次编程，就可以实现3维空间内的若干航点遍历飞行
  {
    // 用户通过地面站自定义或者按键手动输入三维的航点位置，无人机依次遍历各个航点，当前最多支持28个航点，可以自由扩展
    Navigation_User_Setpoint();
  }
  break;
  case 18:
  {

    StateTime++;                                                                                    // 每5ms自增1
    printf("MyState: %d, StateTime: %d, altitude_time: %d, \n", MyState, StateTime, altitude_time); // 打印当前状态和状态保持时间，方便调试观察
    if (MyState == 0)                                                                               // 起飞
    {
      indoor_position_control(0); // 原地定点状态

      if (ins.position_z >= 50.0f)
      {
        altitude_time++;
      }
      if (altitude_time >= OneSecond) // 当高度达到85cm并维持1s后，进入下一个状态
      {
        MyState = 1;
        StateTime = 0; // 切换状态时，重置状态保持时间
        // printf("MyState:0->1\n");
      }
    }
    else if (MyState == 1) // 起飞后保持当前高度和位置，持续10s
    {
      indoor_position_control(0);     // 原地定点状态
      if (StateTime >= OneSecond * 4) // 维持4s后，进入下一个状态
      {
        MyState = 2;   // 进入旋转状态
        StateTime = 0; // 切换状态时，重置状态保持时间
      }
    }
    else if (MyState == 2)
    {
      SetFlyCar(6.0f, 0, 0.0F);       // 前进
      if (StateTime >= OneSecond * 1) // 维持2s后，进入下一个状态
      {
        MyState = 3;
        StateTime = 0; // 切换状态时，重置状态保持时间
      }
      // indoor_position_control(0); // 原地定点状态
    }
    else if (MyState == 3)
    {
      indoor_position_control(0);     // 原地定点状态
      if (StateTime >= OneSecond * 1) // 维持2s后，进入下一个状态
      {
        MyState = 4;   // 进入旋转状态
        StateTime = 0; // 切换状态时，重置状态保持时间
      }
    }
    else if (MyState == 4)
    {
      SetFlyCar(-6.0f, 0, 0.0f);      // 后退
      if (StateTime >= OneSecond * 1) // 维持2s后，进入下一个状态
      {
        MyState = 5;
        StateTime = 0; // 切换状态时，重置状态保持时间
      }
      // indoor_position_control(0); // 原地定点状态
    }
    else if (MyState == 5)
    {
      indoor_position_control(0);     // 原地定点状态
      if (StateTime >= OneSecond * 1) // 维持2s后，进入下一个状态
      {
        MyState = 6;   // 进入旋转状态
        StateTime = 0; // 切换状态时，重置状态保持时间
      }
    }
    else if (MyState == 6)
    {
      SetFlyCar(0.0f, -6.0f, 0.0f);   // 右移
      if (StateTime >= OneSecond * 1) // 维持2s后，进入下一个状态
      {
        MyState = 7;
        StateTime = 0; // 切换状态时，重置状态保持时间
      }
      // indoor_position_control(0); // 原地定点状态
    }
    else if (MyState == 7)
    {
      indoor_position_control(0);     // 原地定点状态
      if (StateTime >= OneSecond * 1) // 维持2s后，进入下一个状态
      {
        MyState = 8;   // 进入旋转状态
        StateTime = 0; // 切换状态时，重置状态保持时间
      }
    }
    else if (MyState == 8)
    {
      SetFlyCar(0.0f, 6.0f, 0.0f);    // 左移
      if (StateTime >= OneSecond * 1) // 维持2s后，进入下一个状态
      {
        MyState = 9;
        StateTime = 0; // 切换状态时，重置状态保持时间
      }
      // indoor_position_control(0); // 原地定点状态
    }
    else if (MyState == 9)
    {
      indoor_position_control(0);     // 原地定点状态
      if (StateTime >= OneSecond * 1) // 维持2s后，进入下一个状态
      {
        MyState = MAN; // 进入旋转状态
        StateTime = 0; // 切换状态时，重置状态保持时间
      }
    }

    else if (MyState == MAN) // 降落状态
    {
      indoor_position_control(0); // 原地定点状态
      *mode = 28;
    }

    // maplepilot.roll_outer_control_output = rc_data.rc_rpyt[RC_ROLL];//右为正
    // maplepilot.pitch_outer_control_output = rc_data.rc_rpyt[RC_PITCH];//后为正
    maplepilot.yaw_ctrl_mode = ROTATE;
    // maplepilot.yaw_outer_control_output = rc_data.rc_rpyt[RC_YAW];//顺指针为正
    flight_altitude_control(ALTHOLD_AUTO_POS_CTRL, 100, NUL); // 高度控制
    break;
  }
  /*x是前后正为前，负为后。y是左右正左负右。*/
  case 19:
  {
    //printf("SpeedT[0]: %.4f, SpeedT[1]: %.4f, SpeedT[2]: %.4f\n", SpeedT[0], SpeedT[1], SpeedT[2]);
    // printf("SpeedPacket[1]: %.4f\n", SpeedPacket[1]/100.0f);
    // printf("SpeedPacket[2]: %.4f\n", SpeedPacket[2]/100.0f);

    // 用户预留任务，编写后注意加上break跳出

    StateTime++; // 每5ms自增1

    if (MyState == 0) // 起飞
    {
      indoor_position_control(0); // 原地定点状态

      if (ins.position_z >= 50.0f)
      {
        altitude_time++;
      }
      if (altitude_time >= OneSecond * 4) // 当高度达到85cm并维持1s后，进入下一个状态
      {
        MyState = 1;
        StateTime = 0; // 切换状态时，重置状态保持时间
        altitude_time = 0;
        // printf("MyState:0->1\n");
      }
    }
    if (MyState == 1) // 使用CYT4控制
    {
      // indoor_position_control(0);
      SpeedT[0] = SpeedPacket[0] / 100.0f;
      SpeedT[1] = SpeedPacket[1] / 100.0f;
      SpeedT[2] = SpeedPacket[2] / 100.0f;
      SetFlyCar(SpeedT[0], SpeedT[1], SpeedT[2]); // CYT4控制
      printf("%f,%f,%f,%f,%f\n", 0.0f, SpeedT[0], SpeedT[1], SpeedT[2], ins.position_z);
     //printf("SpeedPacket[0]: %d, SpeedPacket[1]: %d, SpeedPacket[2]: %d\n", SpeedPacket[0], SpeedPacket[1], SpeedPacket[2]);
      if ((fabs(SpeedT[0] - 0.00) <= 0.2) && (fabs(SpeedT[1] - 0.00) <= 0.2))
      {
        SpeedT[0] = 0.0f;
        SpeedT[1] = 0.0f;
        SpeedT[2] = 0.0f;
        indoor_position_control(0); // 原地定点
      }

      if (StateTime >= OneSecond * 500) // 维持5s后，进入下一个状态
      {
        
        
        MyState = 2;
        StateTime = 0; // 切换状态时，重置状态保持时间
      }
    }
    if (MyState == 2)
    {
      indoor_position_control(0); // 原地定点状态
      *mode = 28;
    }
    maplepilot.yaw_ctrl_mode = ROTATE;
    // maplepilot.yaw_outer_control_output = rc_data.rc_rpyt[RC_YAW];//顺指针为正
    flight_altitude_control(ALTHOLD_AUTO_POS_CTRL, 120, NUL); // 高度控制
    break;
  }
  case 20:
  {
    // 用户预留任务，编写后注意加上break跳出
  }
  case 21:
  {
    // 用户预留任务，编写后注意加上break跳出
  }
  case 22:
  {
    // 用户预留任务，编写后注意加上break跳出
  }
  case 23:
  {
    // 用户预留任务，编写后注意加上break跳出
  }
  case 24:
  {
    // 用户预留任务，编写后注意加上break跳出
  }
  case 25:
  {
    // 用户预留任务，编写后注意加上break跳出
  }
  case 26:
  {
    offboard_start_flag = 1; // offboard启动标志位
    // 不加此行代码，当后续全程无油门上下动作后，飞机最后自动降落到地面不会自动上锁
    rc_data.unwanted_lock_flag = 0;                                                                      // 允许飞机自动上锁，原理和手动推油起飞类似
    if (rc_data.rc_rpyt[RC_ROLL] == 0 && rc_data.rc_rpyt[RC_PITCH] == 0 && rc_data.rc_rpyt[RC_THR] == 0) // 遥控量无任何给定
    {
      // 偏航角度/角速度控制期望来源于Planner端
      maplepilot.yaw_ctrl_start = planner_cmd._yaw_ctrl_start;
      maplepilot.yaw_ctrl_mode = planner_cmd._yaw_ctrl_mode;
      maplepilot.execution_time_ms = planner_cmd._execution_time_ms;
      maplepilot.yaw_outer_control_output = planner_cmd._yaw_outer_control_output;
      // 俯仰、横滚角度控制期望来源于Planner端
      maplepilot.roll_outer_control_output = planner_cmd._roll_outer_control_output;
      maplepilot.pitch_outer_control_output = planner_cmd._pitch_outer_control_output;
      // 高度位置/速度控制期望来源于Planner端
      flight_altitude_control(ALTHOLD_AUTO_VEL_CTRL, NUL, planner_cmd._nav_target_vel.z); // 高度控制

      // 复位Pilot端的位置、速度控制
      optical_ctrl_reset(1); // 目的是为切出SDK模式后,Pilot直接接管位置/速度控制做准备
    }
    else // 水平方向遥杆有动作后恢复到定高+姿态自稳定模式
    {
      maplepilot.roll_outer_control_output = rc_data.rc_rpyt[RC_ROLL];
      maplepilot.pitch_outer_control_output = rc_data.rc_rpyt[RC_PITCH];
      maplepilot.yaw_ctrl_mode = ROTATE;
      maplepilot.yaw_outer_control_output = rc_data.rc_rpyt[RC_YAW];
      flight_altitude_control(ALTHOLD_MANUAL_CTRL, NUL, NUL); // 高度控制
    }
  }
  break;
  case 27: // 前面预留case不满足情况下执行此情形
  {
    maplepilot.roll_outer_control_output = rc_data.rc_rpyt[RC_ROLL];
    maplepilot.pitch_outer_control_output = rc_data.rc_rpyt[RC_PITCH];
    maplepilot.yaw_ctrl_mode = ROTATE;
    maplepilot.yaw_outer_control_output = rc_data.rc_rpyt[RC_YAW];
    flight_altitude_control(ALTHOLD_MANUAL_CTRL, NUL, NUL); // 高度控制
  }
  break;
  case 28: // SDK模式中原地降落至地面怠速后停桨,用于任务执行完成后降落
  {
    printf("Case 28\n");
    printf("");
    indoor_position_control(0);
    maplepilot.yaw_ctrl_mode = ROTATE;
    maplepilot.yaw_outer_control_output = rc_data.rc_rpyt[RC_YAW];
    flight_altitude_control(ALTHOLD_AUTO_VEL_CTRL, NUL, -30); // 高度控制  -50
    if (ins.position_z <= 50.0f)
    {
      altitude_time++;
    }
    if (altitude_time >= OneSecond * 5) // 当高度达到85cm并维持1s后，进入下一个状态
    {
      rc_data.lock_state = LOCK;
      altitude_time = 0;
      StateTime = 0;
      MyState = 0;
      *mode = 19;
    }
  }
  break;
  default:
  {
    maplepilot.roll_outer_control_output = rc_data.rc_rpyt[RC_ROLL];
    maplepilot.pitch_outer_control_output = rc_data.rc_rpyt[RC_PITCH];
    maplepilot.yaw_ctrl_mode = ROTATE;
    maplepilot.yaw_outer_control_output = rc_data.rc_rpyt[RC_YAW];
    flight_altitude_control(ALTHOLD_MANUAL_CTRL, NUL, NUL); // 高度控制
  }
  }
}
