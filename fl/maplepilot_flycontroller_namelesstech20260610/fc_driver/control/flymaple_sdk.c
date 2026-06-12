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

#include "datatype.h"
#include "schedule.h"
#include "wp_math.h"
#include "schedule.h"
#include "drv_uart.h"
#include "datatype.h"
#include "drv_w25qxx.h"
#include "parameter_server.h"
#include "flymaple_sdk.h"

vision_target_check lookdown_vision;
int16_t sdk1_mode_setup = 0x00;

void sdk_send_data(uint8_t *buf, uint16_t len)
{
  uart5_send_bytes(buf, len);
}

void sdk_send_check(unsigned char mode, COM_SDK com)
{
  if (other_params.params.inner_uart == 2)
    return;
  uint8_t sdk_data_to_send[7];
  sdk_data_to_send[0] = 0xFF;
  sdk_data_to_send[1] = 0xFE;
  sdk_data_to_send[2] = 0xA0;
  sdk_data_to_send[3] = 2;
  sdk_data_to_send[4] = mode;
  sdk_data_to_send[5] = 0;
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 6; i++)
    sum += sdk_data_to_send[i];
  sdk_data_to_send[6] = sum;
  if (com == UART5_SDK)
    uart5_send_bytes(sdk_data_to_send, 7);
  if (com == UART2_SDK)
    uart2_send_bytes(sdk_data_to_send, 7);
}

float _Pixel_Image_View_Angle_X, _Pixel_Image_View_Angle_Y;
void get_camera_wide_angle(float view_angle)
{
  float fh = 5.0f / FastTan(0.5f * view_angle * DEG2RAD);
  _Pixel_Image_View_Angle_X = 2 * RAD2DEG * fast_atan(4 / fh);
  _Pixel_Image_View_Angle_Y = 2 * RAD2DEG * fast_atan(3 / fh);
}

uint16_t _CX = 0, _CY = 0;
float _P1 = 0, _P2 = 0;
float _TX = 0, _TY = 0;
float _DX = 0, _DY = 0;
void sensor_parameter_sort(uint16_t tx, uint16_t ty, float pitch, float roll, float alt)
{
  // get_camera_wide_angle(78);//根据摄像头广角，计算得到X、Y轴视角
  float theta_x_max = 0, theta_y_max = 0;
  theta_x_max = Pixel_Image_View_Angle_X_MV;
  theta_y_max = Pixel_Image_View_Angle_Y_MV;
  _P1 = 0.5f * Pixel_Image_Width_MV / FastTan(theta_x_max * DEG2RAD);
  _P2 = 0.5f * Pixel_Image_Height_MV / FastTan(theta_y_max * DEG2RAD);

  _CX = Pixel_Image_Width_MV / 2;
  _CY = Pixel_Image_Height_MV / 2;

  float tmp_x = 0, tmp_y = 0;
  tmp_x = fast_atan((_CX - tx) / _P1);
  tmp_y = fast_atan((_CY - ty) / _P2);

  _TX = FastTan(tmp_x + roll * DEG2RAD) * _P1;
  _TY = FastTan(tmp_y + pitch * DEG2RAD) * _P2;

  _DX = 0.5f * alt * _TX / _P1; //	_DX=alt*_TX/_P1;
  _DY = 0.5f * alt * _TY / _P2; //	_DY=alt*_TY/_P2;
  // 输出带深度修正后的位置偏移
  lookdown_vision.sdk_target.x = _DX;
  lookdown_vision.sdk_target.y = _DY;
}

void sdk_data_reset(vision_target_check *target)
{
  target->x = 0;
  target->y = 0;
  target->pixel = 0;
  target->flag = 0;
  target->state = 0;
  target->angle = 0;
  target->distance = 0;
  target->apriltag_id = 0;
  target->width = 0;
  target->height = 0;
  target->fps = 0;
  target->reserved1 = 0;
  target->reserved2 = 0;
  target->reserved3 = 0;
  target->reserved4 = 0;
  target->range_sensor1 = 0;
  target->range_sensor2 = 0;
  target->range_sensor3 = 0;
  target->range_sensor4 = 0;
}

void sdk_data_prase_1(uint8_t *data_buf, uint8_t num, vision_target_check *target)
{
  uint8_t sum = 0;
  for (uint8_t i = 0; i < (num - 1); i++)
    sum += *(data_buf + i);
  if (!(sum == *(data_buf + num - 1)))
    return; // 不满足和校验条件
  if (!(*(data_buf) == 0xFF && *(data_buf + 1) == 0xFC))
    return; // 不满足帧头条件
  target->x = *(data_buf + 4) << 8 | *(data_buf + 5);
  target->y = *(data_buf + 6) << 8 | *(data_buf + 7);
  target->pixel = *(data_buf + 8) << 8 | *(data_buf + 9);
  target->flag = *(data_buf + 10);
  target->state = *(data_buf + 11);
  target->angle = *(data_buf + 12) << 8 | *(data_buf + 13);
  target->distance = *(data_buf + 14) << 8 | *(data_buf + 15);
  target->apriltag_id = *(data_buf + 16) << 8 | *(data_buf + 17);
  target->width = *(data_buf + 18) << 8 | *(data_buf + 19);
  target->height = *(data_buf + 20) << 8 | *(data_buf + 21);
  target->fps = *(data_buf + 22);
  target->reserved1 = *(data_buf + 23);
  target->reserved2 = *(data_buf + 24);
  target->reserved3 = *(data_buf + 25);
  target->reserved4 = *(data_buf + 26);
  // 扩展距离传感器
  target->range_sensor1 = *(data_buf + 27) << 8 | *(data_buf + 28);
  target->range_sensor2 = *(data_buf + 29) << 8 | *(data_buf + 30);
  target->range_sensor3 = *(data_buf + 31) << 8 | *(data_buf + 32);
  target->range_sensor4 = *(data_buf + 33) << 8 | *(data_buf + 34);
  target->camera_id = *(data_buf + 35);
  target->reserved1_int32 = *(data_buf + 36) << 24 | *(data_buf + 37) << 16 | *(data_buf + 38) << 8 | *(data_buf + 39);
  target->reserved2_int32 = *(data_buf + 40) << 24 | *(data_buf + 41) << 16 | *(data_buf + 42) << 8 | *(data_buf + 43);
  target->reserved3_int32 = *(data_buf + 44) << 24 | *(data_buf + 45) << 16 | *(data_buf + 46) << 8 | *(data_buf + 47);
  target->reserved4_int32 = *(data_buf + 48) << 24 | *(data_buf + 49) << 16 | *(data_buf + 50) << 8 | *(data_buf + 51);

  target->x_pixel_size = Pixel_Size_MV * (Pixel_Image_Width_MV / target->width);
  target->y_pixel_size = Pixel_Size_MV * (Pixel_Image_Height_MV / target->height);
  target->apriltag_distance = AprilTag_Side_Length * Focal_Length_MV / (target->x_pixel_size * FastSqrt(target->pixel));
  target->sdk_mode = *(data_buf + 2);

  if (target->camera_id == 0x01) // 摄像头id为OPENMV
  {
    switch (target->sdk_mode)
    {
    case 0xA1: // 点检测
    {
      target->target_ctrl_enable = target->flag;
      if (target->flag != 0)
      {
        if (target->trust_cnt < 20)
        {
          target->trust_cnt++;
          target->trust_flag = 0;
        }
        else
          target->trust_flag = 1;
      }
      else
      {
        target->trust_cnt /= 2;
        target->trust_flag = 0;
      }
      target->sdk_target_offset.x = SDK_TARGET_X_OFFSET;
      target->sdk_target_offset.y = SDK_TARGET_Y_OFFSET;
      sensor_parameter_sort(target->x, target->y,
                            flymaple.rpy_fusion_deg[_PIT],
                            flymaple.rpy_fusion_deg[_ROL],
                            ins.position_z);
    }
    break;
    case 0xA2: // AprilTag检测
    {
      target->target_ctrl_enable = target->flag;
      if (target->flag != 0)
      {
        if (target->trust_cnt < 20)
        {
          target->trust_cnt++;
          target->trust_flag = 0;
        }
        else
          target->trust_flag = 1;
      }
      else
      {
        target->trust_cnt /= 2;
        target->trust_flag = 0;
      }
      target->sdk_target_offset.x = SDK_TARGET_X_OFFSET;
      target->sdk_target_offset.y = SDK_TARGET_Y_OFFSET;
      sensor_parameter_sort(target->x, target->y,
                            flymaple.rpy_fusion_deg[_PIT],
                            flymaple.rpy_fusion_deg[_ROL],
                            ins.position_z);
    }
    break;
    case 0xA3:
    {
      target->target_ctrl_enable = target->flag;
      if (target->flag != 0)
      {
        if (target->trust_cnt < 20)
        {
          target->trust_cnt++;
          target->trust_flag = 0;
        }
        else
          target->trust_flag = 1;
      }
      else
      {
        target->trust_cnt /= 2;
        target->trust_flag = 0;
      }
      target->sdk_target_offset.x = SDK_TARGET_X_OFFSET;
      target->sdk_target_offset.y = SDK_TARGET_Y_OFFSET;
      sensor_parameter_sort(target->x, target->y,
                            flymaple.rpy_fusion_deg[_PIT],
                            flymaple.rpy_fusion_deg[_ROL],
                            ins.position_z);
      if (target->angle > 90)
        lookdown_vision.sdk_angle = target->angle - 180;
      else
        lookdown_vision.sdk_angle = target->angle;
    }
    break;
    default:
    {
      target->target_ctrl_enable = 0;
      target->trust_flag = 0;
      target->x = 0;
      target->y = 0;
    }
    }
  }
}

uint8_t sdk_buf[SDK_DATA_LEN];
static uint8_t sdk_state_cnt = {0};
static uint8_t sdk_data_len = 0, sdk_data_cnt = 0;
// volatile

void sdk_data_receive_prepare_1(uint8_t data)
{
  if (sdk_state_cnt == 0 && data == 0xFF) // 帧头1
  {
    sdk_state_cnt = 1;
    sdk_buf[0] = data;
    // printf("1\n");
  }
  else if (sdk_state_cnt == 1 && data == 0xFC) // 帧头2
  {
    sdk_state_cnt = 2;
    sdk_buf[1] = data;
  }
  else if (sdk_state_cnt == 2 && data < 0XFF) // 功能字节
  {
    sdk_state_cnt = 3;
    sdk_buf[2] = data;
  }
  else if (sdk_state_cnt == 3 && data < 50) // 数据长度
  {
    sdk_state_cnt = 4;
    sdk_buf[3] = data;
    sdk_data_len = data;
    sdk_data_cnt = 0;
  }
  else if (sdk_state_cnt == 4 && sdk_data_len > 0) // 有多少数据长度，就存多少个
  {
    sdk_data_len--;
    sdk_buf[4 + sdk_data_cnt++] = data;
    if (sdk_data_len == 0)
      sdk_state_cnt = 5;
  }
  else if (sdk_state_cnt == 5) // 最后接收数据校验和
  {
    sdk_state_cnt = 0;
    sdk_buf[4 + sdk_data_cnt] = data;
    sdk_data_prase_1(sdk_buf, sdk_data_cnt + 5, &lookdown_vision);
  }
  else
    sdk_state_cnt = 0;
}

uint8_t Serial_RxPacket[6];     // 接收数据缓存
volatile uint8_t pRxPacket = 0; // 接收数据计数
int16_t TempSpeedPacket[3];     // 速度数据缓存
int16_t SpeedPacket[3];         // 速度数据缓存

/**
 * @brief  发送端：取 int16_t[3] 每个元素的最后一位（bit0）求和校验
 * @param  arr  3个int16_t成员的数组
 * @return  三个最后一位相加的和（0~3）
 */
uint8_t calc_checksum_last_bit(const int16_t arr[3])
{
  // 取每个数的最低位（最后一位）
  uint8_t bit0 = arr[0] & 0x01;
  uint8_t bit1 = arr[1] & 0x01;
  uint8_t bit2 = arr[2] & 0x01;

  // 三个最后一位相加，得到校验值
  return bit0 + bit1 + bit2;
}

systime vision_duty;

void RxPacket_to_SpeedPacket(void)
{
  // 步骤1：将volatile数组读取到普通数组（减少volatile内存访问次数，保证数据完整）
  uint8_t rx_buf[6];
  for (int i = 0; i < 6; i++)
  {
    rx_buf[i] = Serial_RxPacket[i];
  }

  // 步骤2：小端模式拼接（核心逻辑）
  // 小端规则：数组中靠前的字节 = int16_t的低8位，靠后的字节 = 高8位
  TempSpeedPacket[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]); // 第0-1字节 → 第0个int16
  TempSpeedPacket[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]); // 第2-3字节 → 第1个int16
  TempSpeedPacket[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]); // 第4-5字节 → 第2个int16
}

void sdk_data_receive_prepare_2(uint8_t data)
{
  
  //printf("data: %d\n", data);
  if (sdk_state_cnt == 0 && data == 0xFF) // 帧头1
  {
    sdk_state_cnt = 1;
  }
  else if (sdk_state_cnt == 1 && data == 0xFC) // 帧头2
  {
    sdk_state_cnt = 2;
    sdk_data_len = 6;
    pRxPacket = 0;
  }
  else if (sdk_state_cnt == 2) // 有多少数据长度，就存多少个
  {
    sdk_data_len--;
    Serial_RxPacket[pRxPacket++] = data;
    if (sdk_data_len == 0)
      sdk_state_cnt = 3;
  }
  else if (sdk_state_cnt == 3) // 最后接收数据校验和
  {
    RxPacket_to_SpeedPacket(); // 将接收的字节数组转换为速度数据
    if (data == calc_checksum_last_bit(TempSpeedPacket))
    {
      sdk_state_cnt = 0;
      SpeedPacket[0] = TempSpeedPacket[0];
      SpeedPacket[1] = TempSpeedPacket[1];
      SpeedPacket[2] = TempSpeedPacket[2];

      // ---- 收到正确数据后，通过UART5发送一个true（0x01表示true） ----
     // uint8_t true_byte = 0x01;
      //uart5_send_bytes(&true_byte, 1);
      // ----
    }
    else
    {
      sdk_state_cnt = 0;
      // 校验失败处理（如丢弃数据、记录错误等）
    }
  }
  else
    sdk_state_cnt = 0;
}

void flymaple_sdk_init(void)
{
  float sdk_mode_default = 0;
  sdk_data_reset(&lookdown_vision); // 复位SDK线检测数据
  ReadFlashParameterOne(SDK1_MODE_DEFAULT, &sdk_mode_default);
  if (isnan(sdk_mode_default) == 0)
  {
    sdk1_mode_setup = (uint8_t)(sdk_mode_default); // 俯视
    sdk_send_check(sdk1_mode_setup, UART5_SDK);    // 初始化俯视opemmv工作模式，默认以上次工作状态配置
  }
}
