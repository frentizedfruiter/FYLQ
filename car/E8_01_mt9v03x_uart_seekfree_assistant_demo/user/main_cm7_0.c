#include "zf_common_headfile.h"
#include <math.h>


#define PIT0                             (PIT_CH0 )                             // 使用的周期中断编号

#define ENCODER_QUAD1                    (TC_CH07_ENCODER) //小车接线                   // 编码器接口  
#define ENCODER_QUAD1_PHASE_A            (TC_CH07_ENCODER_CH1_P07_6)            // PHASE_A 对应的引脚                 
#define ENCODER_QUAD1_PHASE_B            (TC_CH07_ENCODER_CH2_P07_7)            // PHASE_B 对应的引脚                   
                                                                                
#define ENCODER_QUAD2                    (TC_CH20_ENCODER)                      // 编码器接口
#define ENCODER_QUAD2_PHASE_A            (TC_CH20_ENCODER_CH1_P08_1)            // PHASE_A 对应的引脚
#define ENCODER_QUAD2_PHASE_B            (TC_CH20_ENCODER_CH2_P08_2)            // PHASE_B 对应的引脚

#define ENCODER_QUAD3                    (TC_CH58_ENCODER)                      
#define ENCODER_QUAD3_PHASE_A            (TC_CH58_ENCODER_CH1_P17_3)                                  
#define ENCODER_QUAD3_PHASE_B            (TC_CH58_ENCODER_CH2_P17_4)                                    
                                                                                
#define ENCODER_QUAD4                    (TC_CH27_ENCODER)                      
#define ENCODER_QUAD4_PHASE_A            (TC_CH27_ENCODER_CH1_P19_2)                              
#define ENCODER_QUAD4_PHASE_B            (TC_CH27_ENCODER_CH2_P19_3)                                
                                                                                
#define AIN1 P03_3 //小车接线
#define AIN2 P04_0
#define BIN1 P03_1
#define BIN2 P02_4
#define CIN1 P04_1
#define CIN2 P00_2
#define DIN1 P05_4
#define DIN2 P02_2


#define PWM_CH1                 (TCPWM_CH02_P06_5)//小车接线,注意核对引脚
#define PWM_CH2                 (TCPWM_CH06_P02_1)
#define PWM_CH3                 (TCPWM_CH09_P05_0)
#define PWM_CH4                 (TCPWM_CH10_P05_1)

#define UART_INDEX              (DEBUG_UART_INDEX   )   // 默认 UART_0
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)   // 默认 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )   // 默认 UART0_TX_P00_1
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )   // 默认 UART0_RX_P00_0

// FIFO相关变量（保留原例程）
uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区
uint8  get_data = 0;                                                            // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数
fifo_struct uart_data_fifo;


volatile uint8_t Serial_RxPacket[6];    // 存储4字节数据包内容
volatile uint8_t RxPacket[6];            
volatile int16_t SpeedPacket[3] = 0; //vx, vy, vw           
volatile uint8_t Serial_RxFlag = 0;     // 数据包接收完成标志

typedef enum {
    RX_STATE_IDLE = 0,    // 空闲状态（等待包头0xA5）
    RX_STATE_DATA,        // 接收数据段（4字节）
    RX_STATE_TAIL         // 等待包尾0x5A
} RxState_TypeDef;

typedef struct {
    float Actual0;        // 当前实际值
    float Actual1;        // 上一次实际值
    float Target;         // 目标值
    float Out0;           // 当前输出值
    float Out1;           // 上一次输出值
    float Error0;         // 当前误差
    float Error1;         // 上一次误差
    float ErrorInt;       // 误差积分
    float Kp;             // 比例系数
    float Ki;             // 积分系数
    float Kd;             // 微分系数
    float MaxErrorInt;    // 积分限幅最大值
    float MaxOut;         // 输出限幅最大值
    float MinOut;         // 新增：输出限幅最小值（原代码只限了上限）
    float SampleTime;     // 新增：采样时间（s），用于标准化Ki/Kd

    float FeedRatio;     // 前馈比例（你的电机趋势：设定值*0.1）
} PIDInformation;

volatile RxState_TypeDef RxState = RX_STATE_IDLE;
volatile uint8_t pRxPacket = 0;         // 数据段接收计数

uint16_t PWMPause1 = 0;
uint16_t PWMPause2 = 0;
uint16_t PWMPause3 = 0;
uint16_t PWMPause4 = 0;

int16_t Speed1 = 0;
int16_t Speed2 = 0;
int16_t Speed3 = 0;
int16_t Speed4 = 0;

unsigned char GetSpeedTime = 0;

float EMA = 1;
uint16_t TuenTime = 0;

int16 encoder_data_quad[4] = {0};

PIDInformation MotorSpeed1 = 
{
  .Actual0 = 0,
  .Actual1 = 0,
  .Target = 0, 
  .Out0 = 0,
  .Out1 = 0,
  .Error0 = 0,
  .Error1 = 0,
  .ErrorInt = 0,
  .Kp = 13.0,
  .Ki = 50.0,
  .Kd = 0.0,
  .MaxErrorInt = 37500,
  .MaxOut = 6000,          //新车模修改：增大限幅，实测空载4000稍快，现限到6000，后面承重再改
  .MinOut = -6000,         // 新增：输出限幅最小值（原代码只限了上限）
  .SampleTime = 0.01,     // 新增：采样时间（s），用于标准化Ki/Kd

  .FeedRatio = 10
};

PIDInformation MotorSpeed2 = 
{
  .Actual0 = 0,
  .Actual1 = 0,
  .Target = 0, 
  .Out0 = 0,
  .Out1 = 0,
  .Error0 = 0,
  .Error1 = 0,
  .ErrorInt = 0,
  .Kp = 13.0,
  .Ki = 50.0,
  .Kd = 0.0,
  .MaxErrorInt = 37500,
  .MaxOut = 6000,          //新车模修改：增大限幅，实测空载4000稍快，现限到6000，后面承重再改
  .MinOut = -6000,         // 新增：输出限幅最小值（原代码只限了上限）
  .SampleTime = 0.01,     // 新增：采样时间（s），用于标准化Ki/Kd

  .FeedRatio = 10
};

PIDInformation MotorSpeed3 = 
{
  .Actual0 = 0,
  .Actual1 = 0,
  .Target = 0, 
  .Out0 = 0,
  .Out1 = 0,
  .Error0 = 0,
  .Error1 = 0,
  .ErrorInt = 0,
  .Kp = 13.0,
  .Ki = 50.0,
  .Kd = 0.0,
  .MaxErrorInt = 37500,
  .MaxOut = 2000,
  .MinOut = -2000,         // 新增：输出限幅最小值（原代码只限了上限）
  .SampleTime = 0.01,     // 新增：采样时间（s），用于标准化Ki/Kd

  .FeedRatio = 10
};

PIDInformation MotorSpeed4 = 
{
  .Actual0 = 0,
  .Actual1 = 0,
  .Target = 0, 
  .Out0 = 0,
  .Out1 = 0,
  .Error0 = 0,
  .Error1 = 0,
  .ErrorInt = 0,
  .Kp = 13.0,
  .Ki = 50.0,
  .Kd = 0.0,
  .MaxErrorInt = 37500,
  .MaxOut = 2000,
  .MinOut = -2000,         // 新增：输出限幅最小值（原代码只限了上限）
  .SampleTime = 0.01,     // 新增：采样时间（s），用于标准化Ki/Kd

  .FeedRatio = 13
};

// 数据包解析函数（逐字节处理，新增）
void uart_parse_packet(uint8_t rx_byte)
{
    switch(RxState)
    {
        case RX_STATE_IDLE:
            // 检测包头0xA5
            if(rx_byte == 0xAB)
            {
                RxState = RX_STATE_DATA;
                pRxPacket = 0;
                memset((uint8_t*)Serial_RxPacket, 0, sizeof(Serial_RxPacket)); // 清空数据缓冲区
            }
            break;

        case RX_STATE_DATA:
            // 接收4字节数据
            Serial_RxPacket[pRxPacket] = rx_byte;
            pRxPacket++;
            if(pRxPacket >= 6)             // 4字节接收完成
            {
                RxState = RX_STATE_TAIL;   // 进入包尾检测状态
            }
            break;

        case RX_STATE_TAIL:
            // 检测包尾0x5A
            if(rx_byte == 0xBA)
            {
                // 解析完成，复制数据到最终缓冲区
                for (int i = 0; i < 6; i++)
                {
                    RxPacket[i] = Serial_RxPacket[i];
                }
                Serial_RxFlag = 1;         // 设置接收完成标志
            }
            // 无论包尾是否正确，都回到空闲状态（容错）
            RxState = RX_STATE_IDLE;
            pRxPacket = 0;
            break;

        default:
            // 异常状态重置
            RxState = RX_STATE_IDLE;
            pRxPacket = 0;
            break;
    }
}

void RxPacket_to_SpeedPacket(void)
{
    // 步骤1：将volatile数组读取到普通数组（减少volatile内存访问次数，保证数据完整）
    uint8_t rx_buf[6];
    for (int i = 0; i < 6; i++) {
        rx_buf[i] = RxPacket[i];
    }

    // 步骤2：小端模式拼接（核心逻辑）
    // 小端规则：数组中靠前的字节 = int16_t的低8位，靠后的字节 = 高8位
    SpeedPacket[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);  // 第0-1字节 → 第0个int16
    SpeedPacket[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);  // 第2-3字节 → 第1个int16
    SpeedPacket[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);  // 第4-5字节 → 第2个int16
}

void Set_PWM1_Pause(uint16_t pause)
{
    pwm_set_duty(PWM_CH1, pause);//0<=pause<=10000
}

void Set_PWM2_Pause(uint16_t pause)
{
    pwm_set_duty(PWM_CH2, pause);//0<=pause<=10000
}

void Set_PWM3_Pause(uint16_t pause)
{
    pwm_set_duty(PWM_CH3, pause);//0<=pause<=10000
}

void Set_PWM4_Pause(uint16_t pause)
{
    pwm_set_duty(PWM_CH4, pause);//0<=pause<=10000
}

int16_t GetSpeed4(void)
{
    encoder_data_quad[0] = encoder_get_count(ENCODER_QUAD1);
    encoder_clear_count(ENCODER_QUAD1);
    return -1*encoder_data_quad[0];
}

int16_t GetSpeed3(void)
{
    encoder_data_quad[1] = encoder_get_count(ENCODER_QUAD2);
    encoder_clear_count(ENCODER_QUAD2);
    return encoder_data_quad[1];
}

int16_t GetSpeed2(void)
{
    encoder_data_quad[2] = encoder_get_count(ENCODER_QUAD3);
    encoder_clear_count(ENCODER_QUAD3);
    return encoder_data_quad[2];
}

int16_t GetSpeed1(void)
{
    encoder_data_quad[3] = encoder_get_count(ENCODER_QUAD4);
    encoder_clear_count(ENCODER_QUAD4);
    return -1*encoder_data_quad[3];
}
void SetMotor1Speed(int16_t speed)
{
  if (speed > 0)
  {
    gpio_set_level(AIN1,1);
    gpio_set_level(AIN2,0);
    Set_PWM1_Pause(speed);
  }
  else if (speed < 0)
  {
    gpio_set_level(AIN1,0);
    gpio_set_level(AIN2,1);
    Set_PWM1_Pause((-1)*speed);
  }
  else
  {
    gpio_set_level(AIN1,0);
    gpio_set_level(AIN2,0);
    Set_PWM1_Pause(0);
  }
}

void SetMotor2Speed(int16_t speed)
{
  if (speed > 0)
  {
    gpio_set_level(BIN1,0);
    gpio_set_level(BIN2,1);
    Set_PWM2_Pause(speed);
  }
  else if (speed < 0)
  {
    gpio_set_level(BIN1,1);
    gpio_set_level(BIN2,0);
    Set_PWM2_Pause((-1)*speed);
  }
  else
  {
    gpio_set_level(BIN1,0);
    gpio_set_level(BIN2,0);
    Set_PWM2_Pause(0);
  }
}

void SetMotor3Speed(int16_t speed)
{
  if (speed > 0)
  {
    gpio_set_level(CIN1,0);
    gpio_set_level(CIN2,1);
    Set_PWM3_Pause(speed);
  }
  else if (speed < 0)
  {
    gpio_set_level(CIN1,1);
    gpio_set_level(CIN2,0);
    Set_PWM3_Pause((-1)*speed);
  }
  else
  {
    gpio_set_level(CIN1,0);
    gpio_set_level(CIN2,0);
    Set_PWM3_Pause(0);
  }
}

void SetMotor4Speed(int16_t speed)
{
  if (speed > 0)
  {
    gpio_set_level(DIN1,1);
    gpio_set_level(DIN2,0);
    Set_PWM4_Pause(speed);
  }
  else if (speed < 0)
  {
    gpio_set_level(DIN1,0);
    gpio_set_level(DIN2,1);
    Set_PWM4_Pause((-1)*speed);
  }
  else
  {
    gpio_set_level(DIN1,0);
    gpio_set_level(DIN2,0);
    Set_PWM4_Pause(0);
  }
}


/**
 * @brief PID计算核心函数（改进版：前馈+基础PID，保留原有所有特性，暂时不考虑EMA）
 * @param PIDObj: PID参数结构体指针（包含前馈比例FeedRatio）
 * @param ActualObj: 本次采集到的电机实际转速值
 * @param EMA: 指数移动平均滤波系数（0~1），本次暂不使用
 */
void PIDCalculate(PIDInformation* PIDObj, float ActualObj, float EMA)
{
    // 1. 空指针检查（健壮性设计，避免程序崩溃）
    if (PIDObj == NULL) {
        return;
    }

    // 2. 保存上一次的实际值和误差（用于微分计算）
    PIDObj->Actual1 = PIDObj->Actual0;  // 上一次实际值 = 更新前的当前实际值
    PIDObj->Error1 = PIDObj->Error0;    // 上一次误差 = 更新前的当前误差

    // 3. 暂不考虑EMA滤波，直接赋值实际值（按要求忽略EMA）
    PIDObj->Actual0 = ActualObj;

    // 4. 计算当前误差（目标转速 - 实际转速）
    PIDObj->Error0 = PIDObj->Target - PIDObj->Actual0;

    // 5. 积分项计算（带积分限幅，防止积分饱和）
    // 积分项累加：误差 * 采样时间（标准化积分，不受采样频率影响）
    PIDObj->ErrorInt += PIDObj->Error0 * PIDObj->SampleTime;
    // 积分限幅：限制在[-MaxErrorInt, MaxErrorInt]区间
    if (PIDObj->ErrorInt > PIDObj->MaxErrorInt) {
        PIDObj->ErrorInt = PIDObj->MaxErrorInt;
    } else if (PIDObj->ErrorInt < -PIDObj->MaxErrorInt) {
        PIDObj->ErrorInt = -PIDObj->MaxErrorInt;
    }

    // 6. 微分项计算（基于误差变化率，避免除以0）
    float errorDerivative = 0.0f;
    if (PIDObj->SampleTime > 0.0001f) {  // 采样时间需大于极小值，防止除0
        errorDerivative = (PIDObj->Error0 - PIDObj->Error1) / PIDObj->SampleTime;
    }

    // 7. 基础PID修正值计算（比例+积分+微分）
    float pidCorrection = PIDObj->Kp * PIDObj->Error0 +  // 比例项：快速响应误差
                          PIDObj->Ki * PIDObj->ErrorInt + // 积分项：消除静态误差
                          PIDObj->Kd * errorDerivative;   // 微分项：抑制超调

    // 8. 前馈值计算（利用FeedRatio适配电机趋势，核心改进）
    // 基础前馈：目标值 * 前馈比例（比如设定值2000*0.1=200，匹配电机趋势）
    float ffOutput = PIDObj->Target * PIDObj->FeedRatio;
    // 前馈值限幅（匹配输出范围，避免超出电机驱动能力）
    if (ffOutput > PIDObj->MaxOut) {
        ffOutput = PIDObj->MaxOut;
    } else if (ffOutput < PIDObj->MinOut) {
        ffOutput = PIDObj->MinOut;
    }

    // 9. 总输出 = 前馈值 + PID修正值（前馈抵消已知趋势，PID修正小偏差）
    float pidOutput = ffOutput + pidCorrection;

    // 10. 输出限幅（匹配电机的实际输出能力，比如PWM范围）
    if (pidOutput > PIDObj->MaxOut) {
        pidOutput = PIDObj->MaxOut;
    } else if (pidOutput < PIDObj->MinOut) {
        pidOutput = PIDObj->MinOut;
    }

    // 11. 更新输出值（保存当前输出，同时记录上一次输出）
    PIDObj->Out1 = PIDObj->Out0;
    PIDObj->Out0 = pidOutput;
}

void SetCarSpeed(int16_t vx, int16_t vy, int16_t vw) //-500 <= vx,vy <=500,-200 <= vw <= 200
{
  MotorSpeed1.Target = vx + vy + vw;
  MotorSpeed2.Target = vx - vy - vw;
  MotorSpeed3.Target = vx + vy - vw;
  MotorSpeed4.Target = vx - vy + vw;
}

void SetWhellSpeed(void)
{
  Speed1 = GetSpeed1();
  Speed2 = GetSpeed2();
  Speed3 = GetSpeed3();
  Speed4 = GetSpeed4();

  PIDCalculate(&MotorSpeed1, Speed1, EMA);
  PIDCalculate(&MotorSpeed2, Speed2, EMA);
  PIDCalculate(&MotorSpeed3, Speed3, EMA);
  PIDCalculate(&MotorSpeed4, Speed4, EMA);

  SetMotor1Speed((int16_t)MotorSpeed1.Out0);
  SetMotor2Speed((int16_t)MotorSpeed2.Out0);
  SetMotor3Speed((int16_t)MotorSpeed3.Out0);
  SetMotor4Speed((int16_t)MotorSpeed4.Out0);
}

uint8 pit_state = 0;
uint8 get_speed_time = 0;

//0<speed<
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                   // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等

    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);
    // 开启串口接收中断（核心：中断触发后执行uart_rx_interrupt_handler）
    uart_rx_interrupt(UART_INDEX, 1);
     // 输出初始化提示
    uart_write_string(UART_INDEX, "UART FIFO Parse Init OK.\r\n");
    uart_write_byte(UART_INDEX, '\r');                                          // 输出回车
    uart_write_byte(UART_INDEX, '\n');   

    pwm_init(PWM_CH1, 17000, 0);//pwm_set_duty(PWM_CH1, duty);0<=duty<=10000
    pwm_init(PWM_CH2, 17000, 0);
    pwm_init(PWM_CH3, 17000, 0);
    pwm_init(PWM_CH4, 17000, 0);

    gpio_init(AIN1, GPO, 0, GPO_PUSH_PULL);
    gpio_init(AIN2, GPO, 0, GPO_PUSH_PULL);
    gpio_init(BIN1, GPO, 0, GPO_PUSH_PULL);
    gpio_init(BIN2, GPO, 0, GPO_PUSH_PULL);
    gpio_init(CIN1, GPO, 0, GPO_PUSH_PULL);
    gpio_init(CIN2, GPO, 0, GPO_PUSH_PULL);
    gpio_init(DIN1, GPO, 0, GPO_PUSH_PULL);
    gpio_init(DIN2, GPO, 0, GPO_PUSH_PULL);

    encoder_quad_init(ENCODER_QUAD1,  ENCODER_QUAD1_PHASE_A, ENCODER_QUAD1_PHASE_B);  // 初始化编码器模块与引脚 正交编码器模式
    encoder_quad_init(ENCODER_QUAD2,  ENCODER_QUAD2_PHASE_A, ENCODER_QUAD2_PHASE_B);  // 初始化编码器模块与引脚 正交编码器模式
    encoder_quad_init(ENCODER_QUAD3,  ENCODER_QUAD3_PHASE_A, ENCODER_QUAD3_PHASE_B);       // 初始化编码器模块与引脚 带方向增量编码器模式
    encoder_quad_init(ENCODER_QUAD4,  ENCODER_QUAD4_PHASE_A, ENCODER_QUAD4_PHASE_B);       // 初始化编码器模块与引脚 带方向增量编码器模式
    pit_ms_init(PIT0, 1);    // 初始化 PIT0 为周期中断 1ms 周

    system_delay_ms(10);
    // ========== 小车运动测试序列 ==========
    // 前走 2s
    //SetCarSpeed(2, 0, 0);
    //SetWhellSpeed();
    //SetMotor1Speed(1000);
    // 后走 2s
    //SetCarSpeed(-2, 0, 0);
    //SetWhellSpeed();
    //system_delay_ms(2000);
    // 左走 2s
    //SetCarSpeed(0, 2, 0);
    //SetWhellSpeed();
    //system_delay_ms(2000);
    // 右走 2s
    //SetCarSpeed(0, -2, 0);
    //SetWhellSpeed();
    //system_delay_ms(2000);
    // 顺时针转 2s
    //SetCarSpeed(0, 0, -2);
    //SetWhellSpeed();
    //system_delay_ms(2000);
    // 逆时针转 2s
    //SetCarSpeed(0, 0, 2);
    //SetWhellSpeed();
    //system_delay_ms(2000);
    // 停止
    //SetCarSpeed(0, 0, 0);
    //SetWhellSpeed();
    // ========== 测试结束 ==========


    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {//SetMotor1Speed(4000);//添加测试项
    SetCarSpeed(10, 0, 0);
      fifo_data_count = fifo_used(&uart_data_fifo); 

       if(fifo_data_count != 0)                                                // 读取到数据了
       {
         fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);
           // 原例程是一次性读出所有数据，改为逐字节读出并解析（核心修改）
           for(uint32_t i=0; i<fifo_data_count; i++)
           {
              // 逐字节读取FIFO（每次读1字节）
              // 解析当前字节（移植你的状态机逻辑）
              get_data = fifo_get_data[i];
              uart_parse_packet(get_data);
           }
       }

       // 检测到完整数据包（新增：处理解析后的数据）
       if(Serial_RxFlag == 1)
       {
           // 1. 清除标志位，防止重复处理
           Serial_RxFlag = 0;
           RxPacket_to_SpeedPacket();
           //SetCarSpeed(SpeedPacket[0], SpeedPacket[1], SpeedPacket[2]);
           // 2. 输出解析结果（替代原例程的“回显所有数据”）
           // uart_write_string(UART_INDEX, "\r\nUART parse data: ");
           // for(int i=0; i<4; i++)
           // {
           //     printf("DATA%d:%d\n", i+1,RxPacket[i]);
           // }
       }

        // 此处编写需要循环执行的代码

       // if (get_speed_time >= 10)
       // {
       //      get_speed_time = 0;
       //      SetWhellSpeed();

       //      // printf("ENCODER_DATA_QUAD2 counter \t\t%d .\r\n", GetSpeed2());      // 输出编码器计数信息
       //      // printf("ENCODER_DATA_QUAD3 counter \t\t%d .\r\n", GetSpeed3());      // 输出编码器计数信息
       //      // printf("ENCODER_DATA_QUAD4 counter \t\t%d .\r\n", GetSpeed4());      // 输出编码器计数信息
       // }


        if(pit_state)
        {
            pit_state = 0;
        }

        
        // 此处编写需要循环执行的代码
    }
}


// ************************ 串口接收中断处理函数（核心修改）************************
// 该函数在UART接收中断中被调用，直接解析每一个接收到的字节
void uart_rx_interrupt_handler (void)
{
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
    if(uart_query_byte(UART_INDEX, &get_data))                                  // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    {
        fifo_write_buffer(&uart_data_fifo, &get_data, 1);                       // 将数据写入 fifo 中
    }
}