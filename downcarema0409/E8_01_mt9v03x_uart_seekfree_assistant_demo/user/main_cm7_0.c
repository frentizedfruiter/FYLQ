#include "zf_common_headfile.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

#define IMAGE_SIZE (MT9V03X_W * MT9V03X_H)

#define BOUNDARY_NUM (MT9V03X_H * 2)

int16 bright_center_x = MT9V03X_W / 2;
int16 bright_center_y = MT9V03X_H / 2;
int16 CenterX = MT9V03X_W / 2;
int16 CenterY = MT9V03X_H / 2;

uint8_t xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8_t xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

int16 PX = 0;
int16 PY = 0;

// 小车长条方向灯中心坐标
int16_t bar_cx = MT9V03X_W / 2;
int16_t bar_cy = MT9V03X_H / 2;

uint8_t is_beacon_detected = 0;
uint8_t is_fly_beacon_detected = 0;
uint8_t beacon_count = 0;  // 信标灯数量标志位，范围0~2

// 图像坐标中心(0,0)为左上角，x向右，y向下
int16_t beacon_cx = -1; // 信标灯中心x坐标 [0,187]
int16_t beacon_cy = -1; // 信标灯中心y坐标 [0,119]

// 转换为视觉坐标系 PX/PY 用于控制
int16_t beacon_PX = 0; // beacon_cx - CenterX 水平偏移
int16_t beacon_PY = 0; // -(beacon_cy - CenterY) 垂直偏移

uint8_t image_copy[MT9V03X_H][MT9V03X_W];

// ===================== 原参数 + 新增信标灯预判断阈值（仅修改识别算法） =====================
#define GRAY_THRESH 110
#define BIN_THRESH 35
#define AREA_MIN 5
#define PX_DEAD1 10
#define PX_DEAD2 4
#define PY_DEAD 10
#define SEARCH_VW 70
#define LED1 P19_0

// 【核心新增】信标灯出现预判断阈值（先判断是否存在，再识别）
#define BEACON_PIXEL_MIN 10      // 信标灯最小有效高亮像素数
#define BEACON_PIXEL_MAX 200     // 信标灯最大有效高亮像素数
// ========================================================================================

// 低通滤波系数 (EMA), 范围 0.0~1.0, 越小越平滑
#define FLY_EMA_ALPHA 0.80f

// ===================== PID 控制器参数 =====================
// 系统: offset ≈ ±60像素, 输出 ±400, 周期 ~50Hz (20ms), 输出最低200
//
// 水平方向 offset_x -> vy 控制 (左右平移)
#define PID_X_KP  1.0f     // P: offset=60→P=300, 留100给I+D凑满400
#define PID_X_KI  0.03f    // I: 20帧×10px→Iaccum=200→I=10, 消除稳态
#define PID_X_KD  5.0f     // D: Δ3px→D=5阻尼抑制速度抖动

// 垂直方向 offset_y -> vx 控制 (前后平移)
#define PID_Y_KP  1.0f     // P: 同水平
#define PID_Y_KI  0.03f    // I: 同水平
#define PID_Y_KD  5.0f     // D: 同水平

#define PID_INTEGRAL_LIMIT  150.0f  // I限幅(防windup), max_I = 0.03*150 = 4.5
#define PID_OUTPUT_LIMIT    400.0f  // 输出限幅 ±400
#define PID_D_ALPHA         0.80f    // D低通滤波α(0~1, 0.8强平滑抑制高频抖)
// ==========================================================

// ===================== 小车 PID 控制器参数 =====================
#define CAR_PID_X_KP  3.0f     // 前后平移 P 增益 (PY→vx)
#define CAR_PID_X_KI  0.02f    // 前后平移 I 增益
#define CAR_PID_X_KD  4.0f     // 前后平移 D 增益
#define CAR_PID_Y_KP  3.0f     // 左右平移 P 增益 (PX→vy)
#define CAR_PID_Y_KI  0.02f    // 左右平移 I 增益
#define CAR_PID_Y_KD  4.0f     // 左右平移 D 增益
#define CAR_PID_W_KP  3.0f     // 旋转 P 增益 (direct_dx→vw)
#define CAR_PID_W_KI  0.01f    // 旋转 I 增益
#define CAR_PID_W_KD  2.0f     // 旋转 D 增益
#define CAR_PID_INTEGRAL_LIMIT  100.0f
#define CAR_PID_OUTPUT_LIMIT    300.0f
// ==========================================================

typedef struct
{
    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float prev_error;
    float prev_derivative;  // 微分项历史值(用于低通滤波)
    float integral_limit;
    float output_limit;
    float min_output;       // 输出最小绝对值(非零时抬升至该值, 0则禁用)
    float deadband;         // 死区阈值(|error|小于此值输出0)
} PID_Controller;

// PID 初始化
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd,
              float setpoint, float integral_limit, float output_limit,
              float min_output, float deadband)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->min_output = min_output;
    pid->deadband = deadband;
}

// PID 更新, 输入当前测量值, 返回控制输出
float PID_Update(PID_Controller *pid, float measurement)
{
    float error = pid->setpoint - measurement;

    // 比例项
    float P = pid->Kp * error;

    // 积分项(带限幅防饱和)
    pid->integral += error;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;
    float I = pid->Ki * pid->integral;

    // 微分项(对误差微分 + 低通滤波平滑)
    float raw_derivative = error - pid->prev_error;
    float D = PID_D_ALPHA * raw_derivative + (1.0f - PID_D_ALPHA) * pid->prev_derivative;
    pid->prev_derivative = D;
    D = pid->Kd * D;
    pid->prev_error = error;

    // 合成输出
    float output = P + I + D;

    // 死区: |error| 小于死区阈值时输出0 (避免抖动)
    if (fabsf(error) < pid->deadband)
    {
        output = 0.0f;
    }

    // 输出限幅
    if (output > pid->output_limit)
        output = pid->output_limit;
    if (output < -pid->output_limit)
        output = -pid->output_limit;

    // 最低输出: 非零时抬升至 ±min_output
    if (output != 0.0f && pid->min_output > 0.0f)
    {
        if (fabsf(output) < pid->min_output)
            output = (output > 0.0f) ? pid->min_output : -pid->min_output;
    }

    return output;
}

// PID 重置(目标丢失时清零积分和微分历史)
void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
}
// ==========================================================

// 8邻域
const int8_t dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
const int8_t dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

typedef enum
{
    BEACON_STATE_LOST,
    BEACON_STATE_ALIGNING,
    BEACON_STATE_TRACKING,
} BeaconState;

// 低通滤波历史值 (无人机 vx/vy)
static float fly_filt_vx = 0.0f;
static float fly_filt_vy = 0.0f;

// PID 控制器实例
static PID_Controller pid_x;  // offset_x -> vy 控制 (飞机)
static PID_Controller pid_y;  // offset_y -> vx 控制 (飞机)

// 小车 PID 控制器实例
static PID_Controller pid_car_x;  // PY → vx (前后平移)
static PID_Controller pid_car_y;  // PX → vy (左右平移)
static PID_Controller pid_car_w;  // direct_dx → vw (旋转)

int16_t direct_dx = 0;

typedef struct
{
    int16_t cx, cy;
    int16_t minx, maxx, miny, maxy;
    uint32_t area;
    float max_ratio;
    uint32_t sum_pixel; // 连通域总亮度（信标灯核心特征）
} Blob;

Blob blobs[8];
uint8_t blob_cnt = 0;

float dir_led_angle = 0.0f;
int16_t dir_top_x, dir_top_y;
int16_t dir_bottom_x, dir_bottom_y;

uint8_t RxPacket[6];
int16_t SpeedPacket[3];

void SpeedPacket_to_RxPacket(void)
{
    int16_t speed_buf[3];
    for (int i = 0; i < 3; i++)
        speed_buf[i] = SpeedPacket[i];
    RxPacket[0] = (uint8_t)(speed_buf[0] & 0xFF);
    RxPacket[1] = (uint8_t)((speed_buf[0] >> 8) & 0xFF);
    RxPacket[2] = (uint8_t)(speed_buf[1] & 0xFF);
    RxPacket[3] = (uint8_t)((speed_buf[1] >> 8) & 0xFF);
    RxPacket[4] = (uint8_t)(speed_buf[2] & 0xFF);
    RxPacket[5] = (uint8_t)((speed_buf[2] >> 8) & 0xFF);
}

void SetCarSpeed(int16_t vx, int16_t vy, int16_t vw)
{
    SpeedPacket[0] = vx;
    SpeedPacket[1] = vy;
    SpeedPacket[2] = vw;
    SpeedPacket_to_RxPacket();
    uart_write_byte(UART_0, 0xAB);
    uart_write_buffer(UART_0, RxPacket, 6);
    uart_write_byte(UART_0, 0xBA);
}

// ===================== 无人机控制 =====================
#define FLY_CONTROL_UART UART_0

static uint8_t FlyTxPacket[9];

static void FlyPacket_Checksum(uint8_t *packet, const int16_t speed[3])
{
    packet[0] = 0xFF;
    packet[1] = 0xFC;
    packet[2] = (uint8_t)(speed[0] & 0xFF);
    packet[3] = (uint8_t)((speed[0] >> 8) & 0xFF);
    packet[4] = (uint8_t)(speed[1] & 0xFF);
    packet[5] = (uint8_t)((speed[1] >> 8) & 0xFF);
    packet[6] = (uint8_t)(speed[2] & 0xFF);
    packet[7] = (uint8_t)((speed[2] >> 8) & 0xFF);
    packet[8] = (speed[0] & 0x01) + (speed[1] & 0x01) + (speed[2] & 0x01);
}

void SetFlySpeed(float vx, float vy, float vw)
{
    int16_t speed_buf[3];
    speed_buf[0] = (int16_t)vx;
    speed_buf[1] = (int16_t)vy;
    speed_buf[2] = (int16_t)vw;
    FlyPacket_Checksum(FlyTxPacket, speed_buf);
    uart_write_buffer(FLY_CONTROL_UART, FlyTxPacket, 9);
}

void UpdateBeaconPos(int16_t x, int16_t y)
{
    if (x < 0)
        x = 0;
    if (x >= MT9V03X_W)
        x = MT9V03X_W - 1;
    if (y < 0)
        y = 0;
    if (y >= MT9V03X_H)
        y = MT9V03X_H - 1;

    bright_center_x = x;
    bright_center_y = y;
    
    PY = -(bright_center_y - beacon_cy);
    PX = bright_center_x -  beacon_cx ;
}

void find_all_blobs(void)
{
    uint8_t vis[MT9V03X_H][MT9V03X_W] = {0};
    blob_cnt = 0;

    for (int y = 0; y < MT9V03X_H; y++)
    {
        for (int x = 0; x < MT9V03X_W; x++)
        {
            if (image_copy[y][x] > BIN_THRESH && !vis[y][x] && blob_cnt < 8)
            {
                int stack_x[1024], stack_y[1024];
                int top = 0;
                stack_x[top] = x;
                stack_y[top] = y;
                top++;
                vis[y][x] = 1;

                int64_t sum_x = 0, sum_y = 0;
                uint32_t area = 0;
                uint32_t sum_pixel = 0;
                int16_t minx = MT9V03X_W, maxx = 0, miny = MT9V03X_H, maxy = 0;

                while (top > 0)
                {
                    top--;
                    int cx = stack_x[top];
                    int cy = stack_y[top];

                    sum_x += cx;
                    sum_y += cy;
                    area++;
                    sum_pixel += image_copy[cy][cx];
                    if (cx < minx)
                        minx = cx;
                    if (cx > maxx)
                        maxx = cx;
                    if (cy < miny)
                        miny = cy;
                    if (cy > maxy)
                        maxy = cy;

                    for (int k = 0; k < 8; k++)
                    {
                        int nx = cx + dx[k];
                        int ny = cy + dy[k];
                        if (nx >= 0 && nx < MT9V03X_W && ny >= 0 && ny < MT9V03X_H && !vis[ny][nx] && image_copy[ny][nx] > BIN_THRESH)
                        {
                            vis[ny][nx] = 1;
                            stack_x[top] = nx;
                            stack_y[top] = ny;
                            top++;
                        }
                    }
                }

                if (area < AREA_MIN)
                    continue;

                int16_t w = maxx - minx + 1;
                int16_t h = maxy - miny + 1;
                float max_val = (w > h) ? w : h;
                float min_val = (w < h) ? w : h;
                float max_ratio = max_val / min_val;

                blobs[blob_cnt].cx = sum_x / area;
                blobs[blob_cnt].cy = sum_y / area;
                blobs[blob_cnt].minx = minx;
                blobs[blob_cnt].maxx = maxx;
                blobs[blob_cnt].miny = miny;
                blobs[blob_cnt].maxy = maxy;
                blobs[blob_cnt].area = area;
                blobs[blob_cnt].max_ratio = max_ratio;
                blobs[blob_cnt].sum_pixel = sum_pixel;
                blob_cnt++;
            }
        }
    }
}

float calculate_vertical_angle(int16_t top_x, int16_t top_y, int16_t bottom_x, int16_t bottom_y)
{
    int16_t dx = bottom_x - top_x;
    int16_t dy = bottom_y - top_y;

    if (dx == 0 && dy == 0)
        return 0.0f;

    float rad = atan2f(dx, dy);
    float deg = rad * 180.0f / PI;

    if (deg > 90.0f)
        deg = 90.0f;
    if (deg < -90.0f)
        deg = -90.0f;

    return deg;
}

uint8_t no_car_led = 0;

// ===================== 核心修改：信标灯识别函数（先判断是否出现，再识别） =====================
void find_bright_center(void)
{
    memset(xy_x2_boundary, 0, sizeof(xy_x2_boundary));
    memset(xy_y2_boundary, 0, sizeof(xy_y2_boundary));
    memset(xy_x3_boundary, 0, sizeof(xy_x3_boundary));
    memset(xy_y3_boundary, 0, sizeof(xy_y3_boundary));

    int cnt_red = 0;
    int cnt_yel = 0;

    dir_led_angle = 0.0f;
    dir_top_x = dir_top_y = dir_bottom_x = dir_bottom_y = -1;

    // 【第一步：预判断信标灯是否出现】核心修改！
    // 统计图像中高亮像素总数，判断是否符合信标灯的特征
    uint32_t bright_pixel_total = 0;
    for (int y = 0; y < MT9V03X_H; y++)
    {
        for (int x = 0; x < MT9V03X_W; x++)
        {
            if (image_copy[y][x] > BIN_THRESH)
                bright_pixel_total++;
        }
    }

    // 高亮像素不在信标灯范围内 → 判定未出现，直接执行无目标逻辑
    if (bright_pixel_total < BEACON_PIXEL_MIN || bright_pixel_total > BEACON_PIXEL_MAX)
    {
        is_beacon_detected = 0;
        is_fly_beacon_detected = 0;
        no_car_led = 1;
        beacon_count = 0;

        // 保留原有的无目标边界绘制
        for (int8_t dy = -1; dy <= 1; dy++)
        {
            for (int8_t dx = -1; dx <= 1; dx++)
            {
                int16_t x = MT9V03X_W / 2 + dx, y = MT9V03X_H / 2 + dy;
                if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_red < BOUNDARY_NUM)
                {
                    xy_x2_boundary[cnt_red] = x;
                    xy_y2_boundary[cnt_red] = y;
                    cnt_red++;
                }
            }
        }
        return;
    }

    // 【第二步：信标灯已出现，执行原有识别逻辑】完全保留原功能
    find_all_blobs();
    is_beacon_detected = 1;
    is_fly_beacon_detected = 0;

    if (blob_cnt == 0)
    {
        no_car_led = 1;
        beacon_count = 0;
        for (int8_t dy = -1; dy <= 1; dy++)
        {
            for (int8_t dx = -1; dx <= 1; dx++)
            {
                int16_t x = MT9V03X_W / 2 + dx, y = MT9V03X_H / 2 + dy;
                if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_red < BOUNDARY_NUM)
                {
                    xy_x2_boundary[cnt_red] = x;
                    xy_y2_boundary[cnt_red] = y;
                    cnt_red++;
                }
            }
        }
        return;
    }
    //printf("是否检测到信标灯：%d ",is_beacon_detected);
    // 以下所有逻辑**完全保留原版**，不做任何修改
    int bar_idx = 0;
    float max_ratio = blobs[0].max_ratio;
    for (int i = 1; i < blob_cnt; i++)
    {
        if (blobs[i].max_ratio > max_ratio)
        {
            max_ratio = blobs[i].max_ratio;
            bar_idx = i;
        }
    }

    // 计算信标灯数量（总连通域数减去方向指示灯），范围限制在0~2
    int16_t raw_count = blob_cnt - 1;
    beacon_count = (raw_count < 0) ? 0 : ((raw_count > 2) ? 2 : (uint8_t)raw_count);
     
    Blob bar_blob = blobs[bar_idx];
    int16_t cx = bar_blob.cx;
    int16_t cy = bar_blob.cy;
    // 更新小车长条方向灯中心坐标
    bar_cx = cx;
    bar_cy = cy;
    int16_t minx = bar_blob.minx, maxx = bar_blob.maxx;
    int16_t miny = bar_blob.miny, maxy = bar_blob.maxy;

    float top_max_dist_sq = -1.0f;
    float bottom_max_dist_sq = -1.0f;
    
    for (int y = miny; y <= maxy; y++)
    {
        for (int x = minx; x <= maxx; x++)
        {
            if (image_copy[y][x] > BIN_THRESH)
            {
                float dist_sq = (x - cx) * (x - cx) + (y - cy) * (y - cy);
                if (y < cy)
                {
                    if (dist_sq > top_max_dist_sq)
                    {
                        top_max_dist_sq = dist_sq;
                        dir_top_x = x;
                        dir_top_y = y;
                    }
                }
                else if (y > cy)
                {
                    if (dist_sq > bottom_max_dist_sq)
                    {
                        bottom_max_dist_sq = dist_sq;
                        dir_bottom_x = x;
                        dir_bottom_y = y;
                    }
                }
            }
        }
    }
    
    direct_dx = dir_top_x - dir_bottom_x;
    no_car_led = 0;
    if (dir_top_x == -1 || dir_bottom_x == -1)
    {
        no_car_led = 1;
    }

    {
        for (int8_t dy = -1; dy <= 1; dy++)
        {
            for (int8_t dx = -1; dx <= 1; dx++)
            {
                int16_t x = cx + dx, y = cy + dy;
                if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                {
                    xy_x3_boundary[cnt_yel] = x;
                    xy_y3_boundary[cnt_yel] = y;
                    cnt_yel++;
                }
            }
        }
        if (dir_top_x != -1)
        {
            for (int8_t dy = -1; dy <= 1; dy++)
            {
                for (int8_t dx = -1; dx <= 1; dx++)
                {
                    int16_t x = dir_top_x + dx, y = dir_top_y + dy;
                    if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                    {
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
        }
        if (dir_bottom_x != -1)
        {
            for (int8_t dy = -1; dy <= 1; dy++)
            {
                for (int8_t dx = -1; dx <= 1; dx++)
                {
                    int16_t x = dir_bottom_x + dx, y = dir_bottom_y + dy;
                    if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                    {
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
        }
        if (bar_blob.maxx - bar_blob.minx > bar_blob.maxy - bar_blob.miny)
        {
            for (int8_t dy = -1; dy <= 1; dy++)
            {
                for (int8_t dx = -1; dx <= 1; dx++)
                {
                    int16_t x = minx + dx, y = cy + dy;
                    if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                    {
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
            for (int8_t dy = -1; dy <= 1; dy++)
            {
                for (int8_t dx = -1; dx <= 1; dx++)
                {
                    int16_t x = maxx + dx, y = cy + dy;
                    if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                    {
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
        }
        else
        {
            for (int8_t dy = -1; dy <= 1; dy++)
            {
                for (int8_t dx = -1; dx <= 1; dx++)
                {
                    int16_t x = cx + dx, y = miny + dy;
                    if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                    {
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
            for (int8_t dy = -1; dy <= 1; dy++)
            {
                for (int8_t dx = -1; dx <= 1; dx++)
                {
                    int16_t x = cx + dx, y = maxy + dy;
                    if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_yel < BOUNDARY_NUM)
                    {
                        xy_x3_boundary[cnt_yel] = x;
                        xy_y3_boundary[cnt_yel] = y;
                        cnt_yel++;
                    }
                }
            }
        }
    }

    beacon_cx = -1;
    beacon_cy = -1;
    {
        int16_t fly_choice_idx = -1;
        uint32_t max_sum_pixel = 0;
        
        for (int i = 0; i < blob_cnt; i++)
        {
            if (i == bar_idx)
            continue;
            if (blobs[i].sum_pixel > max_sum_pixel)
            max_sum_pixel = blobs[i].sum_pixel;
        }
        
        if (max_sum_pixel > 0)
        {
            uint32_t brightness_thresh = (max_sum_pixel * 90) / 100;
            uint32_t min_dist_sq = (uint32_t)-1;
            
            for (int i = 0; i < blob_cnt; i++)
            {
                if (i == bar_idx)
                continue;
                if (blobs[i].sum_pixel >= brightness_thresh)
                {
                    int16_t dx_fly = blobs[i].cx - bar_cx;
                    int16_t dy_fly = blobs[i].cy - bar_cy;
                    uint32_t dist_sq = (uint32_t)(dx_fly * dx_fly + dy_fly * dy_fly);
                    if (dist_sq < min_dist_sq)
                    {
                        min_dist_sq = dist_sq;
                        fly_choice_idx = i;
                    }
                }
            }
        }
        
        if (fly_choice_idx >= 0)
        {
            is_fly_beacon_detected = 1;
            beacon_cx = blobs[fly_choice_idx].cx;
            beacon_cy = blobs[fly_choice_idx].cy;
            beacon_PX = beacon_cx - CenterX;
            beacon_PY = -(beacon_cy - CenterY);
        }
        
        UpdateBeaconPos(cx, cy);
    }
    
    for (int i = 0; i < blob_cnt; i++)
    {
        if (i == bar_idx)
            continue;
        Blob circle_blob = blobs[i];
        int16_t cx_circle = circle_blob.cx;
        int16_t cy_circle = circle_blob.cy;
        for (int8_t dy = -1; dy <= 1; dy++)
        {
            for (int8_t dx = -1; dx <= 1; dx++)
            {
                int16_t x = cx_circle + dx, y = cy_circle + dy;
                if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H && cnt_red < BOUNDARY_NUM)
                {
                    xy_x2_boundary[cnt_red] = x;
                    xy_y2_boundary[cnt_red] = y;
                    cnt_red++;
                }
            }
        }
    }
}

int TrackCar_FollowFly(void)
{
    int16_t vx = 0, vy = 0, vw = 0;

    // 情况3：未检测到小车灯 → 静止，重置所有 PID
    if (no_car_led == 1)
    {
        PID_Reset(&pid_car_x);
        PID_Reset(&pid_car_y);
        PID_Reset(&pid_car_w);
        SetCarSpeed(0, 0, 0);
        return 0;
    }

    // 情况1：只检测到小车灯，未检测到信标灯 → 仅 PID 旋转对齐
    if (beacon_count == 0)
    {
        PID_Reset(&pid_car_x);
        PID_Reset(&pid_car_y);
        vw = (int16_t)PID_Update(&pid_car_w, (float)direct_dx);
        SetCarSpeed(0, 0, vw);
        return 1;
    }

    // 情况2：同时检测到小车灯和信标灯 → 旋转优先，对齐后 PID 平移追信标
    if (abs(direct_dx) > 6)
    {
        // 方向灯未对齐 → 优先旋转，暂停平移
        PID_Reset(&pid_car_x);
        PID_Reset(&pid_car_y);
        vx = 0;
        vy = 0;
        vw = (int16_t)PID_Update(&pid_car_w, (float)direct_dx);
    }
    else
    {
        // 方向灯已对齐 → PID 控制平移追信标 (setpoint=0)
        PID_Reset(&pid_car_w);
        vw = 0;
        vx = (int16_t)PID_Update(&pid_car_x, (float)PY);
        vy = (int16_t)PID_Update(&pid_car_y, (float)PX);
    }
    
    SetCarSpeed(vx, vy, vw);
    return 1;
}

void TrackFly_Beacon(void)
{
    float vx = 0.0f, vy = 0.0f, vw = 0.0f;
    int16_t offset_x = bar_cx - CenterX + 5;  // 水平偏移(正=偏右)
    int16_t offset_y = bar_cy - CenterY;  // 垂直偏移(正=偏下)

    // 移动/静止交替: 移动1秒 → 静止1秒 → 循环 (50fps, 1秒=50帧)
    #define MOVE_FRAMES  50  // 移动持续帧数
    #define STOP_FRAMES  25  // 静止持续帧数
    enum { PHASE_MOVE, PHASE_STOP };
    static uint8_t  motion_phase = PHASE_MOVE;  // 当前阶段
    static uint16_t phase_cnt = 0;              // 当前阶段已过帧数

    // 当检测到小车灯时，使用 PID 控制追着长条方向灯跑
    if (is_beacon_detected && no_car_led == 0)
    {
        // 阶段切换判断
        if (motion_phase == PHASE_MOVE && phase_cnt >= MOVE_FRAMES)
        {
            motion_phase = PHASE_STOP;
            phase_cnt = 0;
            PID_Reset(&pid_x);
            PID_Reset(&pid_y);
        }
        else if (motion_phase == PHASE_STOP && phase_cnt >= STOP_FRAMES)
        {
            motion_phase = PHASE_MOVE;
            phase_cnt = 0;
            PID_Reset(&pid_x);
            PID_Reset(&pid_y);
        }

        if (motion_phase == PHASE_MOVE)
        {
            // offset_x (水平偏移) 通过 PID 产生 vy (左右平移速度)
            vy = PID_Update(&pid_x, -(float)offset_x);
            // offset_y (垂直偏移) 通过 PID 产生 vx (前后平移速度)
            vx = PID_Update(&pid_y, (float)offset_y);
        }
        else  // PHASE_STOP
        {
            vx = 0.0f;
            vy = 0.0f;
        }

        phase_cnt++;
        vw = 0.0f;  // 无人机不旋转
    }
    // 其他情况下无人机静止, 重置PID防止积分饱和
    else
    {
        vx = 0.0f;
        vy = 0.0f;
        vw = 0.0f;
        motion_phase = PHASE_MOVE;
        phase_cnt = 0;
        PID_Reset(&pid_x);
        PID_Reset(&pid_y);
    }

    // 低通滤波 (EMA): filtered = alpha * raw + (1 - alpha) * prev
    fly_filt_vx = FLY_EMA_ALPHA * vx + (1.0f - FLY_EMA_ALPHA) * fly_filt_vx;
    fly_filt_vy = FLY_EMA_ALPHA * vy + (1.0f - FLY_EMA_ALPHA) * fly_filt_vy;

    SetFlySpeed(fly_filt_vx, fly_filt_vy, vw);
    printf("%f,%f,%f,%f,%f\n", 0.0f, fly_filt_vx, fly_filt_vy, (float)offset_x, (float)offset_y);
}

int main(void)
{
 // printf("1");
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
//printf("2");
    while (mt9v03x_init())
    {
      //printf("3");
        gpio_toggle_level(LED1);
        system_delay_ms(500);
    }
//printf("3");
    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X, image_copy[0],
        MT9V03X_W, MT9V03X_H);

    seekfree_assistant_camera_boundary_config(
        XY_BOUNDARY, BOUNDARY_NUM,
        xy_x1_boundary, xy_x2_boundary, xy_x3_boundary,
        xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);

    // 初始化飞机 PID 控制器 (setpoint=15, 飞机偏小车右前)
    PID_Init(&pid_x, PID_X_KP, PID_X_KI, PID_X_KD,
             15.0f, PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT, 0.0f, 3.0f);
    PID_Init(&pid_y, PID_Y_KP, PID_Y_KI, PID_Y_KD,
             15.0f, PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT, 0.0f, 3.0f);

    // 初始化小车 PID 控制器 (setpoint=0, 追信标灯中心)
    PID_Init(&pid_car_x, CAR_PID_X_KP, CAR_PID_X_KI, CAR_PID_X_KD,
             0.0f, CAR_PID_INTEGRAL_LIMIT, CAR_PID_OUTPUT_LIMIT, 0.0f, 5.0f);
    PID_Init(&pid_car_y, CAR_PID_Y_KP, CAR_PID_Y_KI, CAR_PID_Y_KD,
             0.0f, CAR_PID_INTEGRAL_LIMIT, CAR_PID_OUTPUT_LIMIT, 0.0f, 5.0f);
    PID_Init(&pid_car_w, CAR_PID_W_KP, CAR_PID_W_KI, CAR_PID_W_KD,
             0.0f, CAR_PID_INTEGRAL_LIMIT, CAR_PID_OUTPUT_LIMIT, 0.0f, 3.0f);

     // printf("123");
    while (1)
    {   
      //printf("123");
        if (mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;

            for (int y = 0; y < MT9V03X_H; y++)
            {
                for (int x = 0; x < MT9V03X_W; x++)
                {
                    uint8_t pix = mt9v03x_image[y][x];
                    image_copy[y][x] = (pix < GRAY_THRESH) ? 0 : pix;
                }
            }
            find_bright_center();                    // 先解算当前帧的方向指示灯特征点、位置、方向
           //seekfree_assistant_camera_send();        // 再发送图像+当前帧的边界数据到上位机
            TrackFly_Beacon();                        // 无人机追小车灯平移
            TrackCar_FollowFly();
           //printf("信标灯数量： %d\n",beacon_count);
        }
        system_delay_ms(1);
    }
}