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


#include "math.h"
#include "wp_math.h"

void vector3f_mul_sub_to_rawdata(vector3f a,vector3f b,vector3f sub,vector3f *c,float scale)
{
  c->x=a.x*b.x-sub.x*scale/GRAVITY_MSS;
  c->y=a.y*b.y-sub.y*scale/GRAVITY_MSS;
  c->z=a.z*b.z-sub.z*scale/GRAVITY_MSS;
}

void vector3f_sub(vector3f a,vector3f b,vector3f *c,float scale)
{
  c->x=a.x-b.x;
  c->y=a.y-b.y;
  c->z=a.z-b.z;
}


float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;  
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}


// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
  if (isnan(v)) {
    return 0.0;
  }
  if (v >= 1.0f) {
    return PI/2;
  }
  if (v <= -1.0f) {
    return -PI/2;
  }
  return asinf(v);
}


// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
float safe_sqrt(float v)
{
  float ret = sqrtf(v);
  if (isnan(ret)) {
    return 0;
  }
  return ret;
}

// a faster varient of atan.  accurate to 6 decimal places for values between -1 ~ 1 but then diverges quickly
float fast_atan(float v)
{
  float v2 = v*v;
  return (v*(1.6867629106f + v2*0.4378497304f)/(1.6867633134f + v2));
}


#define FAST_ATAN2_PIBY2_FLOAT  1.5707963f
// fast_atan2 - faster version of atan2
//      126 us on AVR cpu vs 199 for regular atan2
//      absolute error is < 0.005 radians or 0.28 degrees
//      origin source: https://gist.github.com/volkansalma/2972237/raw/
float fast_atan2(float y, float x)
{
  if (x == 0.0f) {
    if (y > 0.0f) {
      return FAST_ATAN2_PIBY2_FLOAT;
    }
    if (y == 0.0f) {
      return 0.0f;
    }
    return -FAST_ATAN2_PIBY2_FLOAT;
  }
  float atan;
  float z = y/x;
  if (fabs( z ) < 1.0f) {
    atan = z / (1.0f + 0.28f * z * z);
    if (x < 0.0f) {
      if (y < 0.0f) {
        return atan - PI;
      }
      return atan + PI;
    }
  } else {
    atan = FAST_ATAN2_PIBY2_FLOAT - (z / (z * z + 0.28f));
    if (y < 0.0f) {
      return atan - PI;
    }
  }
  return atan;
}


// constrain a value
float constrain_float(float amt, float low, float high) 
{
  // the check for NaN as a float prevents propogation of
  // floating point errors through any function that uses
  // constrain_float(). The normal float semantics already handle -Inf
  // and +Inf
  if (isnan(amt)) {
    return (low+high)*0.5f;
  }
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// constrain a int16_t value
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// constrain a int32_t value
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high) {
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// degrees -> radians
float radians(float deg) {
  return deg * DEG_TO_RAD;
}

// radians -> degrees
float degrees(float rad) {
  return rad * RAD_TO_DEG;
}

// square
float sq(float v) {
  return v*v;
}

// cube
float cube(float v) {
  return v*v*v;
}


// 2D vector length
float pythagorous2(float a, float b) {
  return sqrtf(sq(a)+sq(b));
}

// 3D vector length
float pythagorous3(float a, float b, float c) {
  return sqrtf(sq(a)+sq(b)+sq(c));
}



//translate from ADI's dsp library.
//////////////////////////////////////////////////////////////////////////
//Get fraction and integer parts of floating point
float Modf(float x, float *i)
{
  float y;
  float fract;
  
  y = x;
  if (x < (float)0.0){
    y = -y;
  }
  
  if (y >= (float)16777216.0f){
    *i = x;
    return (float)0.0f;
  }
  
  if (y < (float)1.0f){
    *i = (float)0.0f;
    return x;
  }
  
  y = (float)((long)(y));
  
  if (x < (float)0.0f){
    y = -y;
  }
  
  fract = x - y;
  *i = y;
  
  return fract;
}

float FastPow(float x,float y)
{
  float tmp;
  float znum, zden, result;
  float g, r, u1, u2, v, z;
  long m, p, negate, y_int, n;
  long mi, pi, iw1;
  float y1, y2, w1, w2, w;
  float *a1, *a2;
  float xtmp;
  long *lPtr = (long *)&xtmp;
  float *fPtr = &xtmp;
  static const long a1l[] =  {0,            /* 0.0 */
  0x3f800000,   /* 1.0 */
  0x3f75257d,   /* 0.9576032757759 */
  0x3f6ac0c6,   /* 0.9170039892197 */
  0x3f60ccde,   /* 0.8781260251999 */
  0x3f5744fc,   /* 0.8408963680267 */
  0x3f4e248c,   /* 0.8052451610565 */
  0x3f45672a,   /* 0.7711054086685 */
  0x3f3d08a3,   /* 0.7384130358696 */
  0x3f3504f3,   /* 0.7071067690849 */
  0x3f2d583e,   /* 0.6771277189255 */
  0x3f25fed6,   /* 0.6484197378159 */
  0x3f1ef532,   /* 0.6209288835526 */
  0x3f1837f0,   /* 0.5946035385132 */
  0x3f11c3d3,   /* 0.5693942904472 */
  0x3f0b95c1,   /* 0.5452538132668 */
  0x3f05aac3,   /* 0.5221368670464 */
  0x3f000000};  /* 0.5 */
  static const long a2l[] =  {0,            /* 0.0 */
  0x31a92436,   /* 4.922664054163e-9 */
  0x336c2a94,   /* 5.498675648141e-8 */
  0x31a8fc24,   /* 4.918108587049e-9 */
  0x331f580c,   /* 3.710015050729e-8 */
  0x336a42a1,   /* 5.454296925222e-8 */
  0x32c12342,   /* 2.248419050943e-8 */
  0x32e75623,   /* 2.693110978669e-8 */
  0x32cf9890};  /* 2.41673490109e-8 */
  
  a1 = (float *)a1l;
  a2 = (float *)a2l; 
  negate = 0;
  
  if (x == (float)0.0){
    if (y == (float)0.0){
      return (float)1.0;
    }
    else if (y > (float)0.0){
      return (float)0.0;
    }
    else{
      return (float)FLT_MAX;
    }
  }
  else if (x < (float)0.0){
    y_int = (long)(y);
    if ((float)(y_int) != y){
      return (float)0.0;
    }
    
    x = -x;
    negate = y_int & 0x1;
  }
  
  xtmp = x;
  m = (*lPtr >> 23);
  m = m - 126;
  
  *lPtr = (*lPtr & 0x807fffff) | (126 << 23);
  g = *fPtr;
  
  p = 1;
  if (g <= a1[9]){
    p = 9;
  }
  if (g <= a1[p + 4]){
    p = p + 4;
  }
  if (g <= a1[p + 2]){
    p = p + 2;
  }
  
  p = p + 1;
  znum = g - a1[p];
  znum = znum - a2[p >> 1];
  
  zden = g + a1[p];
  
  p = p - 1;
  
  z = znum / zden;
  z = z + z;
  
  v = z * z;
  
  r = POWP_COEF2 * v;
  r = r + POWP_COEF1;
  r = r * v;
  r = r * z;
  
  r = r + LOG2E_MINUS1 * r;
  u2 = LOG2E_MINUS1 * z;
  u2 = r + u2;
  u2 = u2 + z;
  
  u1 = (float)((m * 16) - p);
  u1 = u1 * 0.0625f;
  
  Modf(16.0f * y, &(y1));
  y1 = y1 * 0.0625f;
  
  y2 = y - y1;
  
  w = u1 * y2;
  tmp = u2 * y;
  w = tmp + w;
  
  Modf(16.0f * w, &(w1));
  w1 = w1 * 0.0625f;
  
  w2 = w - w1;
  
  w = u1 * y1;
  w = w + w1;
  
  Modf(16.0f * w, &(w1));
  w1 = w1 * 0.0625f;
  
  tmp = w - w1;
  w2 = w2 + tmp;
  
  Modf(16.0f * w2, &(w));
  w = w * 0.0625f;
  
  tmp = w1 + w;
  tmp = 16.0f * tmp;
  iw1 = (long)(tmp);
  
  w2 = w2 - w;
  
  if (iw1 > POW_BIGNUM){
    result = (float)FLT_MAX;
    if (negate == 1){
      result = -result;
    }
    return result;
  }
  
  if (w2 > 0){
    w2 = w2 - 0.0625f;
    iw1++;
  }
  
  if (iw1 < POW_SMALLNUM){
    return (float)0.0;
  }
  
  if (iw1 < 0){
    mi = 0;
  }
  else{
    mi = 1;
  }
  n = iw1 / 16;
  mi = mi + n;
  pi = (mi * 16) - iw1;
  
  z = POWQ_COEF5 * w2;
  z = z + POWQ_COEF4;
  z = z * w2;
  z = z + POWQ_COEF3;
  z = z * w2;
  z = z + POWQ_COEF2;
  z = z * w2;
  z = z + POWQ_COEF1;
  z = z * w2;
  
  z = z * a1[pi + 1];
  z = a1[pi + 1] + z;
  
  fPtr = &z;
  lPtr = (long *)fPtr;
  n = (*lPtr >> 23) & 0xff;
  n = n - 127;
  mi = mi + n;
  mi = mi + 127;
  
  mi = mi & 0xff;
  *lPtr = *lPtr & (0x807fffff);
  *lPtr = *lPtr | mi << 23;
  
  result = *fPtr;
  
  if (negate){
    result = -result;
  }
  
  return result;
}
//
float FastTan(float x)
{
  long n;
  float xn;
  float f, g;
  float x_int, x_fract;
  float result;
  float xnum, xden;
  
  if ((x > (float)X_MAX) || (x < (float)-X_MAX)){
    return (float)0.0;
  }
  
  x_int = (float)((long)(x));
  x_fract = x - x_int;
  
  g = (float)0.5;
  if (x <= (float)0.0){
    g = -g;
  }
  n = (long)(x * (float)INV_PI_2 + g);
  xn = (float)(n);
  
  f = x_int - xn * PI_2_C1;
  f = f + x_fract;
  f = f - xn * PI_2_C2;
  f = f - xn * PI_2_C3;
  
  if (f < (float)0.0){
    g = -f;
  }
  else{
    g = f;
  }
  if (g < (float)EPS_FLOAT){
    if (n & 0x0001){
      result = -1.0f / f;
    }
    else{
      result = f;
    }            
    return result;
  }
  
  g = f * f;
  xnum = g * TANP_COEF2;
  xnum = xnum + TANP_COEF1;
  xnum = xnum * g;
  xnum = xnum * f;
  xnum = xnum + f;
  
  xden = g * TANQ_COEF2;
  xden = xden + TANQ_COEF1;
  xden = xden * g;
  xden = xden + TANQ_COEF0;
  
  if (n & 0x0001){
    result = xnum;
    xnum = -xden;
    xden = result;
  }
  result = xnum / xden;
  return result;
}
//
float FastLn(float x)
{
  union { unsigned int i; float f;} e;
  float xn;
  float	z;
  float	w;
  float	a;
  float	b;
  float	r;
  float	result;
  float znum, zden;
  
  int exponent = (*((int*)&x) & 0x7F800000) >> 23;
  e.i = (*((int*)&x) & 0x3F800000);
  
  if(e.f > ROOT_HALF){
    znum = e.f - 1.0f;
    zden = e.f * 0.5f + 0.5f;
  }
  else{
    exponent -= 1;
    znum = e.f - 0.5f;
    zden = e.f * 0.5f + 0.5f;
  }
  xn = (float)exponent;
  z = znum / zden;
  w = z * z;
  a = (LOGDA_COEF2 * w + LOGDA_COEF1) * w + LOGDA_COEF0;
  b = ((w + LOGDB_COEF2) * w + LOGDB_COEF1) * w + LOGDB_COEF0;
  r = a / b * w * z + z;
  result = xn * LN2_DC1 + r;
  r = xn * LN2_DC2;
  result += r;
  r = xn * LN2_DC3;
  result += r;
  return result;
}

float FastAsin(float x)
{
  float y, g;
  float num, den, result;
  long i;
  float sign = 1.0;
  
  y = x;
  if (y < (float)0.0){
    y = -y;
    sign = -sign;
  }
  
  if (y > (float)0.5){
    i = 1;
    if (y > (float)1.0){
      result = 0.0;
      return result;
    }    
    g = (1.0f - y) * 0.5f;
    y = -2.0f * FastSqrt(g);
  }
  else{
    i = 0;
    if (y < (float)EPS_FLOAT){
      result = y;
      if (sign < (float)0.0){
        result = -result;
      }
      return result;
    }
    g = y * y;
  }
  num = ((ASINP_COEF3 * g + ASINP_COEF2) * g + ASINP_COEF1) * g;
  den = ((g + ASINQ_COEF2) * g + ASINQ_COEF1) * g + ASINQ_COEF0;
  result = num / den;
  result = result * y + y;
  if (i == 1){
    result = result + (float)PI_2;
  }
  if (sign < (float)0.0){
    result = -result;
  }
  return result;
}

float FastAtan2(float y, float x)
{
  float f, g;
  float num, den;
  float result;
  int n;
  
  static const float a[4] = {0, (float)PI_6, (float)PI_2, (float)PI_3};
  
  if (x == (float)0.0){
    if (y == (float)0.0){
      result = 0.0;
      return result;
    }
    
    result = (float)PI_2;
    if (y > (float)0.0){
      return result;
    }
    if (y < (float)0.0){
      result = -result;
      return result;
    }
  }
  n = 0;
  num = y;
  den = x;
  
  if (num < (float)0.0){
    num = -num;
  }
  if (den < (float)0.0){
    den = -den;
  }
  if (num > den){
    f = den;
    den = num;
    num = f;
    n = 2;
  }
  f = num / den;
  
  if (f > (float)TWO_MINUS_ROOT3){
    num = f * (float)SQRT3_MINUS_1 - 1.0f + f;
    den = (float)SQRT3 + f;
    f = num / den;
    n = n + 1;
  }
  
  g = f;
  if (g < (float)0.0){
    g = -g;
  }
  
  if (g < (float)EPS_FLOAT){
    result = f;
  }
  else{
    g = f * f;
    num = (ATANP_COEF1 * g + ATANP_COEF0) * g;
    den = (g + ATANQ_COEF1) * g + ATANQ_COEF0;
    result = num / den;
    result = result * f + f;
  }
  if (n > 1){
    result = -result;
  }
  result = result + a[n];
  
  if (x < (float)0.0){
    result = PI - result;
  }
  if (y < (float)0.0){
    result = -result;
  }
  return result;
}

// Quake inverse square root
float FastSqrtI(float x)
{
  //////////////////////////////////////////////////////////////////////////
  //less accuracy, more faster
  /*
  L2F l2f;
  float xhalf = 0.5f * x;
  l2f.f = x;
  
  l2f.i = 0x5f3759df - (l2f.i >> 1);
  x = l2f.f * (1.5f - xhalf * l2f.f * l2f.f);
  return x;
  */
  //////////////////////////////////////////////////////////////////////////
  union { unsigned int i; float f;} l2f;
  l2f.f = x;
  l2f.i = 0x5F1F1412 - (l2f.i >> 1);
  return l2f.f * (1.69000231f - 0.714158168f * x * l2f.f * l2f.f);
}

float FastSqrt(float x)
{
  return x * FastSqrtI(x);
}

#define FAST_SIN_TABLE_SIZE 512

const float sinTable[FAST_SIN_TABLE_SIZE + 1] = {
  0.00000000f, 0.01227154f, 0.02454123f, 0.03680722f, 0.04906767f, 0.06132074f,
  0.07356456f, 0.08579731f, 0.09801714f, 0.11022221f, 0.12241068f, 0.13458071f,
  0.14673047f, 0.15885814f, 0.17096189f, 0.18303989f, 0.19509032f, 0.20711138f,
  0.21910124f, 0.23105811f, 0.24298018f, 0.25486566f, 0.26671276f, 0.27851969f,
  0.29028468f, 0.30200595f, 0.31368174f, 0.32531029f, 0.33688985f, 0.34841868f,
  0.35989504f, 0.37131719f, 0.38268343f, 0.39399204f, 0.40524131f, 0.41642956f,
  0.42755509f, 0.43861624f, 0.44961133f, 0.46053871f, 0.47139674f, 0.48218377f,
  0.49289819f, 0.50353838f, 0.51410274f, 0.52458968f, 0.53499762f, 0.54532499f,
  0.55557023f, 0.56573181f, 0.57580819f, 0.58579786f, 0.59569930f, 0.60551104f,
  0.61523159f, 0.62485949f, 0.63439328f, 0.64383154f, 0.65317284f, 0.66241578f,
  0.67155895f, 0.68060100f, 0.68954054f, 0.69837625f, 0.70710678f, 0.71573083f,
  0.72424708f, 0.73265427f, 0.74095113f, 0.74913639f, 0.75720885f, 0.76516727f,
  0.77301045f, 0.78073723f, 0.78834643f, 0.79583690f, 0.80320753f, 0.81045720f,
  0.81758481f, 0.82458930f, 0.83146961f, 0.83822471f, 0.84485357f, 0.85135519f,
  0.85772861f, 0.86397286f, 0.87008699f, 0.87607009f, 0.88192126f, 0.88763962f,
  0.89322430f, 0.89867447f, 0.90398929f, 0.90916798f, 0.91420976f, 0.91911385f,
  0.92387953f, 0.92850608f, 0.93299280f, 0.93733901f, 0.94154407f, 0.94560733f,
  0.94952818f, 0.95330604f, 0.95694034f, 0.96043052f, 0.96377607f, 0.96697647f,
  0.97003125f, 0.97293995f, 0.97570213f, 0.97831737f, 0.98078528f, 0.98310549f,
  0.98527764f, 0.98730142f, 0.98917651f, 0.99090264f, 0.99247953f, 0.99390697f,
  0.99518473f, 0.99631261f, 0.99729046f, 0.99811811f, 0.99879546f, 0.99932238f,
  0.99969882f, 0.99992470f, 1.00000000f, 0.99992470f, 0.99969882f, 0.99932238f,
  0.99879546f, 0.99811811f, 0.99729046f, 0.99631261f, 0.99518473f, 0.99390697f,
  0.99247953f, 0.99090264f, 0.98917651f, 0.98730142f, 0.98527764f, 0.98310549f,
  0.98078528f, 0.97831737f, 0.97570213f, 0.97293995f, 0.97003125f, 0.96697647f,
  0.96377607f, 0.96043052f, 0.95694034f, 0.95330604f, 0.94952818f, 0.94560733f,
  0.94154407f, 0.93733901f, 0.93299280f, 0.92850608f, 0.92387953f, 0.91911385f,
  0.91420976f, 0.90916798f, 0.90398929f, 0.89867447f, 0.89322430f, 0.88763962f,
  0.88192126f, 0.87607009f, 0.87008699f, 0.86397286f, 0.85772861f, 0.85135519f,
  0.84485357f, 0.83822471f, 0.83146961f, 0.82458930f, 0.81758481f, 0.81045720f,
  0.80320753f, 0.79583690f, 0.78834643f, 0.78073723f, 0.77301045f, 0.76516727f,
  0.75720885f, 0.74913639f, 0.74095113f, 0.73265427f, 0.72424708f, 0.71573083f,
  0.70710678f, 0.69837625f, 0.68954054f, 0.68060100f, 0.67155895f, 0.66241578f,
  0.65317284f, 0.64383154f, 0.63439328f, 0.62485949f, 0.61523159f, 0.60551104f,
  0.59569930f, 0.58579786f, 0.57580819f, 0.56573181f, 0.55557023f, 0.54532499f,
  0.53499762f, 0.52458968f, 0.51410274f, 0.50353838f, 0.49289819f, 0.48218377f,
  0.47139674f, 0.46053871f, 0.44961133f, 0.43861624f, 0.42755509f, 0.41642956f,
  0.40524131f, 0.39399204f, 0.38268343f, 0.37131719f, 0.35989504f, 0.34841868f,
  0.33688985f, 0.32531029f, 0.31368174f, 0.30200595f, 0.29028468f, 0.27851969f,
  0.26671276f, 0.25486566f, 0.24298018f, 0.23105811f, 0.21910124f, 0.20711138f,
  0.19509032f, 0.18303989f, 0.17096189f, 0.15885814f, 0.14673047f, 0.13458071f,
  0.12241068f, 0.11022221f, 0.09801714f, 0.08579731f, 0.07356456f, 0.06132074f,
  0.04906767f, 0.03680722f, 0.02454123f, 0.01227154f, 0.00000000f, -0.01227154f,
  -0.02454123f, -0.03680722f, -0.04906767f, -0.06132074f, -0.07356456f,
  -0.08579731f, -0.09801714f, -0.11022221f, -0.12241068f, -0.13458071f,
  -0.14673047f, -0.15885814f, -0.17096189f, -0.18303989f, -0.19509032f, 
  -0.20711138f, -0.21910124f, -0.23105811f, -0.24298018f, -0.25486566f, 
  -0.26671276f, -0.27851969f, -0.29028468f, -0.30200595f, -0.31368174f, 
  -0.32531029f, -0.33688985f, -0.34841868f, -0.35989504f, -0.37131719f, 
  -0.38268343f, -0.39399204f, -0.40524131f, -0.41642956f, -0.42755509f, 
  -0.43861624f, -0.44961133f, -0.46053871f, -0.47139674f, -0.48218377f, 
  -0.49289819f, -0.50353838f, -0.51410274f, -0.52458968f, -0.53499762f, 
  -0.54532499f, -0.55557023f, -0.56573181f, -0.57580819f, -0.58579786f, 
  -0.59569930f, -0.60551104f, -0.61523159f, -0.62485949f, -0.63439328f, 
  -0.64383154f, -0.65317284f, -0.66241578f, -0.67155895f, -0.68060100f, 
  -0.68954054f, -0.69837625f, -0.70710678f, -0.71573083f, -0.72424708f, 
  -0.73265427f, -0.74095113f, -0.74913639f, -0.75720885f, -0.76516727f, 
  -0.77301045f, -0.78073723f, -0.78834643f, -0.79583690f, -0.80320753f, 
  -0.81045720f, -0.81758481f, -0.82458930f, -0.83146961f, -0.83822471f, 
  -0.84485357f, -0.85135519f, -0.85772861f, -0.86397286f, -0.87008699f, 
  -0.87607009f, -0.88192126f, -0.88763962f, -0.89322430f, -0.89867447f, 
  -0.90398929f, -0.90916798f, -0.91420976f, -0.91911385f, -0.92387953f, 
  -0.92850608f, -0.93299280f, -0.93733901f, -0.94154407f, -0.94560733f, 
  -0.94952818f, -0.95330604f, -0.95694034f, -0.96043052f, -0.96377607f, 
  -0.96697647f, -0.97003125f, -0.97293995f, -0.97570213f, -0.97831737f, 
  -0.98078528f, -0.98310549f, -0.98527764f, -0.98730142f, -0.98917651f, 
  -0.99090264f, -0.99247953f, -0.99390697f, -0.99518473f, -0.99631261f, 
  -0.99729046f, -0.99811811f, -0.99879546f, -0.99932238f, -0.99969882f, 
  -0.99992470f, -1.00000000f, -0.99992470f, -0.99969882f, -0.99932238f, 
  -0.99879546f, -0.99811811f, -0.99729046f, -0.99631261f, -0.99518473f, 
  -0.99390697f, -0.99247953f, -0.99090264f, -0.98917651f, -0.98730142f, 
  -0.98527764f, -0.98310549f, -0.98078528f, -0.97831737f, -0.97570213f, 
  -0.97293995f, -0.97003125f, -0.96697647f, -0.96377607f, -0.96043052f, 
  -0.95694034f, -0.95330604f, -0.94952818f, -0.94560733f, -0.94154407f, 
  -0.93733901f, -0.93299280f, -0.92850608f, -0.92387953f, -0.91911385f, 
  -0.91420976f, -0.90916798f, -0.90398929f, -0.89867447f, -0.89322430f, 
  -0.88763962f, -0.88192126f, -0.87607009f, -0.87008699f, -0.86397286f, 
  -0.85772861f, -0.85135519f, -0.84485357f, -0.83822471f, -0.83146961f, 
  -0.82458930f, -0.81758481f, -0.81045720f, -0.80320753f, -0.79583690f, 
  -0.78834643f, -0.78073723f, -0.77301045f, -0.76516727f, -0.75720885f, 
  -0.74913639f, -0.74095113f, -0.73265427f, -0.72424708f, -0.71573083f, 
  -0.70710678f, -0.69837625f, -0.68954054f, -0.68060100f, -0.67155895f, 
  -0.66241578f, -0.65317284f, -0.64383154f, -0.63439328f, -0.62485949f, 
  -0.61523159f, -0.60551104f, -0.59569930f, -0.58579786f, -0.57580819f, 
  -0.56573181f, -0.55557023f, -0.54532499f, -0.53499762f, -0.52458968f, 
  -0.51410274f, -0.50353838f, -0.49289819f, -0.48218377f, -0.47139674f, 
  -0.46053871f, -0.44961133f, -0.43861624f, -0.42755509f, -0.41642956f, 
  -0.40524131f, -0.39399204f, -0.38268343f, -0.37131719f, -0.35989504f, 
  -0.34841868f, -0.33688985f, -0.32531029f, -0.31368174f, -0.30200595f, 
  -0.29028468f, -0.27851969f, -0.26671276f, -0.25486566f, -0.24298018f, 
  -0.23105811f, -0.21910124f, -0.20711138f, -0.19509032f, -0.18303989f, 
  -0.17096189f, -0.15885814f, -0.14673047f, -0.13458071f, -0.12241068f, 
  -0.11022221f, -0.09801714f, -0.08579731f, -0.07356456f, -0.06132074f, 
  -0.04906767f, -0.03680722f, -0.02454123f, -0.01227154f, -0.00000000f
};

void FastSinCos(float x, float *sinVal, float *cosVal)
{
  float fract, in; // Temporary variables for input, output
  unsigned short indexS, indexC; // Index variable
  float f1, f2, d1, d2; // Two nearest output values
  int n;
  float findex, Dn, Df, temp;
  
  // input x is in radians
  //Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi, for cosine add 0.25 (pi/2) to read sine table
  in = x * 0.159154943092f;
  
  // Calculation of floor value of input
  n = (int) in;
  
  // Make negative values towards -infinity
  if(in < 0.0f){
    n--;
  }
  // Map input value to [0 1]
  in = in - (float) n;
  
  // Calculation of index of the table
  findex = (float) FAST_SIN_TABLE_SIZE * in;
  indexS = ((unsigned short)findex) & 0x1ff;
  indexC = (indexS + (FAST_SIN_TABLE_SIZE / 4)) & 0x1ff;
  
  // fractional value calculation
  fract = findex - (float) indexS;
  
  // Read two nearest values of input value from the cos & sin tables
  f1 = sinTable[indexC+0];
  f2 = sinTable[indexC+1];
  d1 = -sinTable[indexS+0];
  d2 = -sinTable[indexS+1];
  
  Dn = 0.0122718463030f; // delta between the two points (fixed), in this case 2*pi/FAST_SIN_TABLE_SIZE
  Df = f2 - f1; // delta between the values of the functions
  temp = Dn*(d1 + d2) - 2*Df;
  temp = fract*temp + (3*Df - (d2 + 2*d1)*Dn);
  temp = fract*temp + d1*Dn;
  
  // Calculation of cosine value
  *cosVal = fract*temp + f1;
  
  // Read two nearest values of input value from the cos & sin tables
  f1 = sinTable[indexS+0];
  f2 = sinTable[indexS+1];
  d1 = sinTable[indexC+0];
  d2 = sinTable[indexC+1];
  
  Df = f2 - f1; // delta between the values of the functions
  temp = Dn*(d1 + d2) - 2*Df;
  temp = fract*temp + (3*Df - (d2 + 2*d1)*Dn);
  temp = fract*temp + d1*Dn;
  
  // Calculation of sine value
  *sinVal = fract*temp + f1;
}

float FastSin(float x)
{
  float sinVal, fract, in; // Temporary variables for input, output
  unsigned short index; // Index variable
  float a, b; // Two nearest output values
  int n;
  float findex;
  
  // input x is in radians
  // Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi
  in = x * 0.159154943092f;
  
  // Calculation of floor value of input
  n = (int) in;
  
  // Make negative values towards -infinity
  if(x < 0.0f){
    n--;
  }
  
  // Map input value to [0 1]
  in = in - (float) n;
  
  // Calculation of index of the table
  findex = (float) FAST_SIN_TABLE_SIZE * in;
  index = ((unsigned short)findex) & 0x1ff;
  
  // fractional value calculation
  fract = findex - (float) index;
  
  // Read two nearest values of input value from the sin table
  a = sinTable[index];
  b = sinTable[index+1];
  
  // Linear interpolation process
  sinVal = (1.0f-fract)*a + fract*b;
  
  // Return the output value
  return (sinVal);
}

float FastCos(float x)
{
  float cosVal, fract, in; // Temporary variables for input, output
  unsigned short index; // Index variable
  float a, b; // Two nearest output values
  int n;
  float findex;
  
  // input x is in radians
  // Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi, add 0.25 (pi/2) to read sine table
  in = x * 0.159154943092f + 0.25f;
  
  // Calculation of floor value of input
  n = (int) in;
  
  // Make negative values towards -infinity
  if(in < 0.0f){
    n--;
  }
  
  // Map input value to [0 1]
  in = in - (float) n;
  
  // Calculation of index of the table
  findex = (float) FAST_SIN_TABLE_SIZE * in;
  index = ((unsigned short)findex) & 0x1ff;
  
  // fractional value calculation
  fract = findex - (float) index;
  
  // Read two nearest values of input value from the cos table
  a = sinTable[index];
  b = sinTable[index+1];
  
  // Linear interpolation process
  cosVal = (1.0f-fract)*a + fract*b;
  
  // Return the output value
  return (cosVal);
}

float FastAbs(float x)
{
  union { unsigned int i; float f;} y;
  y.f = x;
  y.i = y.i & 0x7FFFFFFF;
  return (float)y.f;
}

#ifndef M_PI 
#define M_PI 3.14159265358979323846264338327950288f 
#endif 
float sine(float x) 
{
  const float Q = 0.775;
  const float P = 0.225;
  const float B = 4 / M_PI;
  const float C = -4 / (M_PI * M_PI);
  float y = B * x + C * x * fabs(x);
  return (Q * y + P * y * fabs(y));
}

float cosine(float x)
{
  return sine(x + M_PI / 2);
}


float calculate_rmse(float* data1,float *data2,int n)
{
  float fSum = 0;
  for (int i = 0; i < n; ++i)
  {
    fSum += (data1[i] - data2[i]) *(data1[i] - data2[i]);
  }
  return sqrt(fSum / n);
}

float calculate_variance(float* data,int n)
{
  double sum = 0;
  double u=0;
  for (int i = 0; i < n; ++i)
  {
    sum += data[i];
  }
  u=sum/n;
  sum=0;
  for (int i = 0; i < n; ++i)
  {
    sum +=(data[i]-u)*(data[i]-u);
  }
  return (float)(sum/n);
}



float calculate_standard_deviation(float* data,int n)
{
  return FastSqrt(calculate_variance(data,n));
}

float calculate_average(float* data,int n)
{
  double sum = 0,u=0;
  for (int i = 0; i < n; ++i)
  {
    sum += data[i];
  }
  u=sum/n;
  return u;
}

/************************************************************/
/*************************以下计算球面投影距离内容源于APM3.2 AP.Math.c文件******************************/
/***********************************************************
@函数名：longitude_scale
@入口参数：Location loc
@出口参数：无
功能描述：球面投影
@作者：无名小哥
@日期：2019年01月27日
*************************************************************/
float longitude_scale(Location loc)
{
  static int32_t last_lat;
  static float scale = 1.0;
  //比较两次纬度相差值，避免重复运算余弦函数
  if (ABS(last_lat - loc.lat) < 100000) {
    // we are within 0.01 degrees (about 1km) of the
    // same latitude. We can avoid the cos() and return
    // the same scale factor.
    return scale;
  }
  scale = cosf(loc.lat * 1.0e-7f * DEG_TO_RAD);
  scale = constrain_float(scale, 0.01f, 1.0f);
  last_lat = loc.lat;
  return scale;
}
/*
return the distance in meters in North/East plane as a N/E vector
from loc1 to loc2
*/
vector2f location_diff(Location loc1,Location loc2)
{
  vector2f location_delta;
  location_delta.x=(loc2.lng - loc1.lng) * LOCATION_SCALING_FACTOR * longitude_scale(loc2);//距离单位为m
  location_delta.y=(loc2.lat - loc1.lat) * LOCATION_SCALING_FACTOR;//距离单位为m
  return location_delta;
}

// return distance in meters between two locations
float get_distance(Location loc1,Location loc2)
{
  float dlat  = (float)(loc2.lat - loc1.lat);
  float dlong = ((float)(loc2.lng - loc1.lng)) * longitude_scale(loc2);
  return pythagorous2(dlat, dlong)*LOCATION_SCALING_FACTOR;
}


/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
*	Adam M Rivera
*	With direction from: Andrew Tridgell, Jason Short, Justin Beech
*
*	Adapted from: http://www.societyofrobots.com/robotforum/index.php?topic=11855.0
*	Scott Ferguson
*	scottfromscott@gmail.com
*
*/


// 1 byte - 4 bits for value + 1 bit for sign + 3 bits for repeats => 8 bits
typedef struct{
  // Offset has a max value of 15
  uint8_t abs_offset : 4;
  // Sign of the offset, 0 = negative, 1 = positive
  uint8_t offset_sign : 1;
  // The highest repeat is 7
  uint8_t repeats : 3;
}row_value;

// 730 bytes
const uint8_t exceptions[10][73]=
{ 
  {150,145,140,135,130,125,120,115,110,105,100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,5,0,4,9,14,19,24,29,34,39,44,49,54,59,64,69,74,79,84,89,94,99,104,109,114,119,124,129,134,139,144,149,154,159,164,169,174,179,175,170,165,160,155,150}, \
    {143,137,131,126,120,115,110,105,100,95,90,85,80,75,71,66,62,57,53,48,44,39,35,31,27,22,18,14,9,5,1,3,7,11,16,20,25,29,34,38,43,47,52,57,61,66,71,76,81,86,91,96,101,107,112,117,123,128,134,140,146,151,157,163,169,175,178,172,166,160,154,148,143}, \
      {130,124,118,112,107,101,96,92,87,82,78,74,70,65,61,57,54,50,46,42,38,34,31,27,23,19,16,12,8,4,1,2,6,10,14,18,22,26,30,34,38,43,47,51,56,61,65,70,75,79,84,89,94,100,105,111,116,122,128,135,141,148,155,162,170,177,174,166,159,151,144,137,130}, \
        {111,104,99,94,89,85,81,77,73,70,66,63,60,56,53,50,46,43,40,36,33,30,26,23,20,16,13,10,6,3,0,3,6,9,13,16,20,24,28,32,36,40,44,48,52,57,61,65,70,74,79,84,88,93,98,103,109,115,121,128,135,143,152,162,172,176,165,154,144,134,125,118,111}, \
          {85,81,77,74,71,68,65,63,60,58,56,53,51,49,46,43,41,38,35,32,29,26,23,19,16,13,10,7,4,1,1,3,6,9,13,16,19,23,26,30,34,38,42,46,50,54,58,62,66,70,74,78,83,87,91,95,100,105,110,117,124,133,144,159,178,160,141,125,112,103,96,90,85}, \
            {62,60,58,57,55,54,52,51,50,48,47,46,44,42,41,39,36,34,31,28,25,22,19,16,13,10,7,4,2,0,3,5,8,10,13,16,19,22,26,29,33,37,41,45,49,53,56,60,64,67,70,74,77,80,83,86,89,91,94,97,101,105,111,130,109,84,77,74,71,68,66,64,62}, \
              {46,46,45,44,44,43,42,42,41,41,40,39,38,37,36,35,33,31,28,26,23,20,16,13,10,7,4,1,1,3,5,7,9,12,14,16,19,22,26,29,33,36,40,44,48,51,55,58,61,64,66,68,71,72,74,74,75,74,72,68,61,48,25,2,22,33,40,43,45,46,47,46,46}, \
                {6,9,12,15,18,21,23,25,27,28,27,24,17,4,14,34,49,56,60,60,60,58,56,53,50,47,43,40,36,32,28,25,21,17,13,9,5,1,2,6,10,14,17,21,24,28,31,34,37,39,41,42,43,43,41,38,33,25,17,8,0,4,8,10,10,10,8,7,4,2,0,3,6}, \
                  {22,24,26,28,30,32,33,31,23,18,81,96,99,98,95,93,89,86,82,78,74,70,66,62,57,53,49,44,40,36,32,27,23,19,14,10,6,1,2,6,10,15,19,23,27,31,35,38,42,45,49,52,55,57,60,61,63,63,62,61,57,53,47,40,33,28,23,21,19,19,19,20,22}, \
                    {168,173,178,176,171,166,161,156,151,146,141,136,131,126,121,116,111,106,101,96,91,86,81,76,71,66,61,56,51,46,41,36,31,26,21,16,11,6,1,3,8,13,18,23,28,33,38,43,48,53,58,63,68,73,78,83,88,93,98,103,108,113,118,123,128,133,138,143,148,153,158,163,168} \
};

// 100 bytes
const uint8_t exception_signs[10][10] = 
{ 
  {0,0,0,1,255,255,224,0,0,0}, 
  {0,0,0,1,255,255,240,0,0,0}, 
  {0,0,0,1,255,255,248,0,0,0}, 
  {0,0,0,1,255,255,254,0,0,0}, 
  {0,0,0,3,255,255,255,0,0,0}, 
  {0,0,0,3,255,255,255,240,0,0}, 
  {0,0,0,15,255,255,255,254,0,0}, 
  {0,3,255,255,252,0,0,7,252,0}, 
  {0,127,255,255,252,0,0,0,0,0}, 
  {0,0,31,255,254,0,0,0,0,0} 
};

// 76 bytes
const uint8_t declination_keys[2][37]= 
{ 
  // Row start values
  {36,30,25,21,18,16,14,12,11,10,9,9,9,8,8,8,7,6,6,5,4,4,4,3,4,4,4}, 
  // Row length values
  {39,38,33,35,37,35,37,36,39,34,41,42,42,28,39,40,43,51,50,39,37,34,44,51,49,48,55}
};

// 1056 total values @ 1 byte each = 1056 bytes
const row_value declination_values[]= 
{ \
  {0,0,4},{1,1,0},{0,0,2},{1,1,0},{0,0,2},{1,1,3},{2,1,1},{3,1,3},{4,1,1},{3,1,1},{2,1,1},{3,1,0},{2,1,0},{1,1,0},{2,1,1},{1,1,0},{2,1,0},{3,1,4},{4,1,1},{3,1,0},{4,1,0},{3,1,2},{2,1,2},{1,1,1},{0,0,0},{1,0,1},{3,0,0},{4,0,0},{6,0,0},{8,0,0},{11,0,0},{13,0,1},{10,0,0},{9,0,0},{7,0,0},{5,0,0},{4,0,0},{2,0,0},{1,0,2}, \
    {0,0,6},{1,1,0},{0,0,6},{1,1,2},{2,1,0},{3,1,2},{4,1,2},{3,1,3},{2,1,0},{1,1,0},{2,1,0},{1,1,2},{2,1,2},{3,1,3},{4,1,0},{3,1,3},{2,1,1},{1,1,1},{0,0,0},{1,0,1},{2,0,0},{4,0,0},{5,0,0},{6,0,0},{7,0,0},{8,0,0},{9,0,0},{8,0,0},{6,0,0},{7,0,0},{6,0,0},{4,0,1},{3,0,0},{2,0,0},{1,0,0},{2,0,0},{0,0,0},{1,0,0}, \
      {0,0,1},{1,0,0},{0,0,1},{1,1,0},{0,0,6},{1,0,0},{1,1,0},{0,0,0},{1,1,1},{2,1,1},{3,1,0},{4,1,3},{3,1,0},{4,1,0},{3,1,1},{2,1,0},{1,1,7},{2,1,0},{3,1,6},{2,1,0},{1,1,2},{0,0,0},{1,0,0},{2,0,0},{3,0,1},{5,0,1},{6,0,0},{7,0,0},{6,0,2},{4,0,2},{3,0,1},{2,0,2},{1,0,1}, \
        {0,0,0},{1,0,0},{0,0,7},{0,0,5},{1,1,1},{2,1,1},{3,1,0},{4,1,5},{3,1,1},{1,1,0},{2,1,0},{1,1,0},{0,0,0},{1,1,0},{0,0,1},{1,1,0},{0,0,0},{2,1,2},{3,1,1},{2,1,0},{3,1,0},{2,1,1},{1,1,0},{0,0,1},{1,0,0},{2,0,1},{4,0,1},{5,0,4},{4,0,0},{3,0,1},{4,0,0},{2,0,0},{3,0,0},{2,0,2},{1,0,2}, \
          {0,0,0},{1,0,0},{0,0,7},{0,0,5},{1,1,2},{2,1,0},{4,1,0},{3,1,0},{5,1,0},{3,1,0},{5,1,0},{4,1,1},{3,1,0},{2,1,1},{1,1,2},{0,0,2},{1,0,0},{0,0,1},{1,1,0},{2,1,2},{3,1,0},{2,1,1},{1,1,1},{0,0,0},{1,0,0},{2,0,1},{3,0,1},{4,0,0},{5,0,0},{4,0,0},{5,0,0},{4,0,0},{3,0,1},{1,0,0},{3,0,0},{2,0,4},{1,0,3}, \
            {0,0,1},{1,0,0},{0,0,7},{1,1,0},{0,0,4},{1,1,0},{2,1,1},{3,1,0},{4,1,2},{5,1,0},{4,1,0},{3,1,1},{2,1,1},{1,1,1},{0,0,2},{1,0,1},{2,0,0},{1,0,0},{0,0,0},{1,1,1},{2,1,3},{1,1,1},{1,0,2},{2,0,0},{3,0,1},{4,0,2},{3,0,1},{2,0,0},{1,0,0},{2,0,1},{1,0,0},{2,0,1},{1,0,0},{2,0,0},{1,0,3}, \
              {0,0,2},{1,0,0},{0,0,5},{1,1,0},{0,0,4},{1,1,2},{2,1,0},{4,1,0},{3,1,0},{4,1,1},{5,1,0},{4,1,0},{3,1,1},{2,1,0},{1,1,1},{0,0,2},{1,0,0},{2,0,0},{1,0,0},{3,0,0},{2,0,0},{1,0,0},{0,0,1},{2,1,2},{1,1,0},{2,1,0},{0,0,1},{1,0,1},{2,0,1},{3,0,2},{4,0,0},{2,0,1},{1,0,2},{2,0,0},{1,0,1},{2,0,0},{1,0,5}, \
                {0,0,0},{1,0,0},{0,0,7},{0,0,1},{1,1,0},{0,0,2},{1,1,2},{3,1,2},{4,1,3},{3,1,0},{2,1,1},{1,1,0},{0,0,2},{1,0,1},{2,0,0},{3,0,0},{2,0,0},{3,0,0},{2,0,0},{1,0,0},{0,0,0},{1,1,0},{2,1,0},{1,1,0},{2,1,1},{0,0,0},{1,1,0},{1,0,2},{2,0,1},{3,0,1},{2,0,1},{1,0,1},{0,0,0},{1,0,2},{2,0,0},{1,0,5}, \
                  {0,0,4},{1,0,0},{0,0,3},{1,1,0},{0,0,3},{1,1,0},{0,0,0},{1,1,0},{2,1,1},{3,1,1},{4,1,3},{3,1,0},{2,1,0},{1,1,0},{0,0,2},{1,0,0},{2,0,3},{3,0,0},{2,0,0},{3,0,0},{1,0,1},{1,1,1},{2,1,0},{1,1,0},{2,1,0},{1,1,0},{0,0,2},{1,0,0},{2,0,0},{1,0,0},{2,0,0},{3,0,0},{2,0,0},{1,0,0},{0,0,0},{1,0,0},{0,0,0},{1,0,7},{1,0,1}, \
                    {0,0,7},{0,0,5},{1,1,0},{0,0,1},{2,1,0},{1,1,0},{3,1,3},{4,1,1},{3,1,1},{1,1,1},{0,0,1},{1,0,0},{2,0,3},{3,0,0},{2,0,3},{0,0,2},{2,1,0},{1,1,0},{2,1,0},{1,1,0},{0,0,0},{1,1,0},{1,0,0},{0,0,0},{1,0,0},{2,0,0},{1,0,0},{2,0,1},{0,0,0},{1,0,0},{0,0,1},{1,0,0},{0,0,0},{1,0,7}, \
                      {0,0,6},{1,0,0},{0,0,0},{1,1,0},{0,0,4},{1,1,0},{0,0,0},{2,1,0},{1,1,0},{3,1,0},{2,1,0},{4,1,0},{3,1,0},{4,1,1},{2,1,2},{0,0,1},{1,0,0},{2,0,7},{2,0,0},{1,0,1},{0,0,1},{1,1,1},{2,1,0},{1,1,0},{0,0,0},{1,1,0},{0,0,0},{1,0,0},{0,0,0},{1,0,1},{2,0,0},{1,0,0},{0,0,0},{1,0,0},{0,0,2},{1,0,1},{0,0,0},{2,0,0},{1,0,2},{0,0,0},{1,0,0}, \
                        {0,0,7},{0,0,3},{1,1,0},{0,0,2},{1,1,0},{2,1,0},{1,1,0},{3,1,0},{2,1,0},{4,1,0},{3,1,0},{4,1,0},{3,1,0},{2,1,1},{1,1,0},{0,0,0},{1,0,1},{2,0,1},{3,0,0},{2,0,2},{1,0,0},{2,0,0},{1,0,1},{0,0,0},{1,0,0},{0,0,0},{1,1,0},{0,0,0},{2,1,0},{1,1,0},{0,0,0},{1,1,0},{0,0,1},{1,0,0},{0,0,0},{1,0,2},{0,0,3},{1,0,0},{0,0,0},{1,0,6},{0,0,0},{1,0,0}, \
                          {0,0,2},{1,1,0},{0,0,1},{1,0,0},{0,0,3},{1,1,0},{0,0,2},{1,1,2},{2,1,0},{3,1,0},{2,1,0},{3,1,0},{4,1,0},{3,1,1},{2,1,0},{1,1,1},{0,0,0},{1,0,0},{2,0,2},{3,0,0},{2,0,1},{1,0,0},{2,0,0},{1,0,1},{0,0,0},{1,0,0},{0,0,2},{1,1,0},{0,0,0},{1,1,1},{0,0,0},{1,1,0},{0,0,0},{1,0,0},{0,0,0},{1,0,0},{0,0,0},{1,0,0},{0,0,5},{1,0,7},{0,0,0},{1,0,0}, \
                            {0,0,5},{1,0,0},{0,0,4},{1,1,0},{0,0,1},{1,1,1},{2,1,2},{3,1,4},{2,1,0},{1,1,0},{0,0,0},{1,0,1},{2,0,6},{1,0,1},{0,0,0},{1,0,1},{0,0,2},{1,1,1},{0,0,0},{1,1,0},{0,0,1},{1,1,0},{0,0,0},{1,0,0},{0,0,0},{1,0,0},{0,0,7},{1,0,7}, \
                              {0,0,3},{1,0,0},{0,0,7},{1,1,0},{0,0,0},{1,1,0},{2,1,3},{3,1,3},{2,1,0},{1,1,1},{0,0,0},{1,0,1},{2,0,2},{3,0,0},{1,0,0},{2,0,0},{1,0,0},{2,0,0},{0,0,1},{1,0,1},{0,0,2},{1,1,0},{0,0,0},{1,1,0},{0,0,1},{1,1,0},{0,0,3},{1,0,0},{0,0,2},{1,1,0},{0,0,3},{1,0,0},{0,0,0},{1,0,0},{2,0,0},{1,0,1},{2,0,0},{0,0,0},{1,0,0}, \
                                {0,0,1},{1,0,0},{0,0,2},{1,0,0},{0,0,5},{1,1,2},{2,1,1},{3,1,0},{2,1,0},{3,1,2},{2,1,1},{1,1,0},{0,0,1},{1,0,0},{2,0,0},{1,0,0},{2,0,4},{1,0,1},{0,0,0},{1,0,1},{0,0,0},{1,0,0},{0,0,0},{1,1,0},{0,0,0},{1,1,1},{0,0,7},{0,0,0},{1,1,0},{0,0,0},{1,1,0},{0,0,3},{1,0,1},{0,0,0},{1,0,0},{2,0,0},{1,0,0},{2,0,0},{1,0,0},{1,0,0}, \
                                  {0,0,0},{1,0,1},{0,0,1},{1,0,0},{0,0,0},{1,0,0},{0,0,3},{1,1,0},{0,0,0},{1,1,0},{2,1,2},{3,1,0},{2,1,0},{4,1,0},{3,1,0},{2,1,2},{1,1,0},{0,0,0},{1,0,2},{2,0,4},{1,0,0},{2,0,0},{0,0,0},{1,0,0},{0,0,0},{1,0,1},{0,0,2},{1,1,0},{0,0,0},{1,1,0},{0,0,0},{1,1,0},{0,0,5},{1,1,0},{0,0,0},{1,1,1},{0,0,0},{1,1,0},{0,0,1},{1,0,4},{2,0,1},{1,0,0},{1,0,0}, \
                                    {0,0,0},{2,0,0},{1,0,0},{0,0,0},{1,0,1},{0,0,0},{1,0,0},{0,0,3},{1,1,0},{0,0,0},{2,1,2},{3,1,0},{2,1,0},{3,1,0},{4,1,0},{3,1,0},{2,1,1},{1,1,1},{1,0,0},{0,0,0},{2,0,0},{1,0,0},{2,0,1},{1,0,0},{2,0,2},{1,0,0},{0,0,0},{1,0,1},{0,0,0},{1,0,0},{0,0,0},{1,0,0},{1,1,0},{0,0,1},{1,1,0},{0,0,0},{1,1,0},{0,0,1},{1,1,0},{0,0,2},{1,1,3},{0,0,0},{1,1,0},{0,0,2},{1,0,0},{2,0,0},{1,0,1},{2,0,0},{1,0,0},{2,0,0},{1,0,0}, \
                                      {0,0,0},{1,0,1},{2,0,0},{1,0,1},{0,0,0},{1,0,0},{0,0,0},{1,0,0},{0,0,0},{1,1,0},{0,0,0},{2,1,0},{1,1,0},{2,1,0},{3,1,1},{2,1,0},{4,1,1},{3,1,0},{2,1,1},{1,1,0},{0,0,1},{1,0,0},{2,0,0},{1,0,0},{2,0,2},{1,0,0},{2,0,1},{1,0,0},{0,0,0},{1,0,2},{0,0,0},{1,0,0},{0,0,3},{1,1,0},{0,0,1},{1,1,0},{0,0,0},{1,1,0},{0,0,0},{1,1,0},{0,0,0},{1,1,2},{2,1,0},{1,1,1},{0,0,1},{1,0,3},{2,0,0},{1,0,0},{2,0,1},{2,0,0}, \
                                        {0,0,0},{2,0,0},{1,0,0},{2,0,0},{1,0,4},{0,0,1},{1,1,0},{0,0,0},{2,1,0},{1,1,0},{3,1,3},{4,1,1},{3,1,0},{2,1,1},{1,1,0},{0,0,0},{1,0,2},{2,0,0},{1,0,0},{2,0,4},{1,0,0},{0,0,0},{1,0,3},{0,0,0},{1,0,0},{0,0,4},{1,1,0},{0,0,0},{1,1,0},{0,0,0},{1,1,4},{2,1,1},{1,1,1},{0,0,2},{1,0,1},{2,0,2},{1,0,0},{2,0,0},{2,0,0}, \
                                          {0,0,0},{2,0,3},{1,0,3},{0,0,2},{1,1,0},{2,1,2},{4,1,0},{3,1,0},{4,1,2},{3,1,1},{1,1,1},{0,0,0},{1,0,2},{2,0,4},{1,0,0},{2,0,1},{0,0,0},{1,0,0},{2,0,0},{0,0,0},{1,0,2},{0,0,0},{1,0,0},{0,0,3},{1,1,4},{2,1,0},{1,1,0},{2,1,2},{1,1,2},{0,0,1},{1,0,0},{2,0,1},{1,0,0},{3,0,0},{1,0,0},{2,0,0},{2,0,0}, \
                                            {0,0,0},{2,0,4},{1,0,3},{0,0,0},{1,1,2},{3,1,1},{4,1,2},{5,1,0},{4,1,0},{3,1,1},{1,1,1},{0,0,0},{1,0,1},{2,0,0},{1,0,0},{2,0,1},{3,0,0},{2,0,2},{1,0,2},{2,0,0},{1,0,5},{0,0,4},{1,1,1},{2,1,4},{3,1,0},{2,1,1},{1,1,1},{0,0,0},{1,0,2},{2,0,1},{3,0,0},{2,0,0},{1,0,0},{3,0,0}, \
                                              {0,0,0},{2,0,1},{3,0,0},{2,0,1},{1,0,0},{2,0,0},{1,0,0},{0,0,2},{1,1,0},{2,1,0},{3,1,1},{5,1,4},{3,1,1},{1,1,1},{1,0,0},{0,0,0},{2,0,1},{1,0,0},{3,0,0},{2,0,2},{3,0,0},{2,0,1},{1,0,1},{2,0,1},{1,0,0},{2,0,0},{1,0,3},{0,0,0},{1,0,0},{1,1,0},{0,0,0},{1,1,0},{2,1,2},{3,1,0},{2,1,0},{3,1,2},{2,1,0},{1,1,1},{0,0,0},{1,0,2},{2,0,1},{3,0,0},{2,0,1},{3,0,0}, \
                                                {0,0,0},{3,0,1},{2,0,0},{3,0,0},{2,0,0},{1,0,0},{2,0,0},{1,0,1},{0,0,1},{2,1,1},{3,1,0},{4,1,0},{6,1,0},{5,1,0},{7,1,0},{6,1,0},{5,1,0},{3,1,1},{1,1,0},{0,0,1},{1,0,0},{2,0,3},{3,0,0},{2,0,0},{3,0,0},{2,0,0},{3,0,0},{2,0,1},{1,0,0},{2,0,5},{1,0,2},{0,0,2},{1,1,0},{2,1,0},{3,1,2},{4,1,0},{3,1,0},{4,1,0},{3,1,0},{2,1,1},{1,1,0},{1,0,0},{0,0,0},{2,0,0},{1,0,0},{2,0,0},{3,0,0},{2,0,0},{3,0,0},{2,0,0},{2,0,0}, \
                                                  {0,0,0},{2,0,0},{3,0,1},{2,0,0},{3,0,0},{2,0,1},{1,0,1},{0,0,1},{2,1,1},{4,1,0},{6,1,0},{7,1,1},{8,1,0},{7,1,0},{5,1,0},{3,1,0},{2,1,0},{1,1,0},{0,0,0},{1,0,1},{2,0,1},{3,0,0},{2,0,0},{3,0,2},{2,0,0},{3,0,2},{1,0,0},{3,0,0},{2,0,0},{3,0,0},{2,0,4},{1,0,1},{0,0,1},{1,1,0},{2,1,0},{3,1,0},{4,1,0},{5,1,0},{4,1,1},{5,1,0},{4,1,0},{2,1,1},{1,1,0},{0,0,0},{1,0,0},{2,0,3},{3,0,1},{2,0,0},{3,0,0}, \
                                                    {0,0,0},{3,0,2},{2,0,0},{3,0,0},{2,0,2},{1,0,0},{0,0,1},{2,1,0},{3,1,0},{5,1,0},{8,1,0},{9,1,0},{10,1,1},{7,1,0},{5,1,0},{3,1,0},{1,1,0},{0,0,0},{1,0,1},{2,0,0},{3,0,0},{2,0,0},{3,0,3},{4,0,0},{3,0,7},{2,0,0},{3,0,0},{2,0,0},{3,0,0},{2,0,0},{1,0,0},{2,0,0},{0,0,2},{2,1,0},{3,1,0},{4,1,0},{5,1,0},{7,1,0},{5,1,0},{6,1,0},{4,1,1},{2,1,0},{0,0,1},{1,0,1},{2,0,1},{3,0,2},{2,0,0},{3,0,0}, \
                                                      {0,0,0},{3,0,5},{2,0,1},{1,0,0},{0,0,0},{1,1,0},{2,1,0},{5,1,0},{8,1,0},{12,1,0},{14,1,0},{13,1,0},{9,1,0},{6,1,0},{3,1,0},{1,1,0},{0,0,0},{2,0,0},{1,0,0},{3,0,0},{2,0,0},{3,0,0},{4,0,0},{3,0,1},{4,0,0},{3,0,0},{4,0,1},{3,0,0},{4,0,0},{3,0,2},{4,0,0},{3,0,1},{4,0,0},{3,0,0},{2,0,0},{3,0,0},{2,0,2},{0,0,1},{1,1,0},{2,1,0},{4,1,0},{5,1,0},{7,1,0},{8,1,0},{6,1,1},{5,1,0},{3,1,0},{1,1,1},{1,0,1},{2,0,0},{3,0,0},{2,0,0},{3,0,1},{2,0,0},{3,0,0}, \
};

// read something the size of a byte, far version
static inline uint8_t pgm_read_byte_far(const void *s) {return *(const uint8_t *)s;}
static inline void *memcpy_P(void *dest, const uint8_t *src, size_t n){    return memcpy(dest, src, n);}

#define PGM_UINT8(p) pgm_read_byte_far(p)

float get_lookup_value(uint8_t x, uint8_t y)//计算为正数时表示东偏，负数时表示西偏，单位为deg
{
  // return value
  int16_t val = 0;
  // These are exception indicies
  if(x <= 6 || x >= 34)
  {
    // If the x index is in the upper range we need to translate it
    // to match the 10 indicies in the exceptions lookup table
    if(x >= 34) x -= 27;
    // Read the unsigned value from the array
    val = PGM_UINT8(&exceptions[x][y]);
    // Read the 8 bit compressed sign values
    uint8_t sign = PGM_UINT8(&exception_signs[x][y/8]);
    // Check the sign bit for this index
    if(sign & (0x80 >> y%8))
      val = -val;
    return val;
  }
  
  // Because the values were removed from the start of the
  // original array (0-6) to the exception array, all the indicies
  // in this main lookup need to be shifted left 7
  // EX: User enters 7 -> 7 is the first row in this array so it needs to be zero
  if(x >= 7) x -= 7;
  
  // If we are looking for the first value we can just use the
  // row start value from declination_keys
  if(y == 0) return PGM_UINT8(&declination_keys[0][x]);
  // Init vars
  row_value stval;
  int16_t offset = 0;
  // These will never exceed the second dimension length of 73
  uint8_t current_virtual_index = 0, r;
  // This could be the length of the array or less (1075 or less)
  uint16_t start_index = 0, i;
  // Init value to row start
  val = PGM_UINT8(&declination_keys[0][x]);
  // Find the first element in the 1D array
  // that corresponds with the target row
  for(i = 0; i < x; i++) {
    start_index += PGM_UINT8(&declination_keys[1][i]);
  }
  // Traverse the row until we find our value
  for(i = start_index; i < (start_index + PGM_UINT8(&declination_keys[1][x])) && current_virtual_index <= y; i++) {
    
    // Pull out the row_value struct
    memcpy_P((void*) &stval, (const uint8_t *)&declination_values[i], sizeof(row_value));
    // Pull the first offset and determine sign
    offset = stval.abs_offset;
    offset = (stval.offset_sign == 1) ? -offset : offset;
    // Add offset for each repeat
    // This will at least run once for zero repeat
    for(r = 0; r <= stval.repeats && current_virtual_index <= y; r++) {
      val += offset;
      current_virtual_index++;
    }
  }
  return val;
}


float get_declination(float lat, float lon)
{
  int16_t decSW, decSE, decNW, decNE, lonmin, latmin;
  uint8_t latmin_index,lonmin_index;
  float decmin, decmax;
  
  // Constrain to valid inputs
  lat = constrain_float(lat, -90, 90);
  lon = constrain_float(lon, -180, 180);
  
  latmin = (int16_t)((floorf(lat/5)*5));
  lonmin = (int16_t)((floorf(lon/5)*5));
  
  latmin_index= (90+latmin)/5;
  lonmin_index= (180+lonmin)/5;
  
  decSW = (int16_t)(get_lookup_value(latmin_index, lonmin_index));
  decSE = (int16_t)(get_lookup_value(latmin_index, lonmin_index+1));
  decNE = (int16_t)(get_lookup_value(latmin_index+1, lonmin_index+1));
  decNW = (int16_t)(get_lookup_value(latmin_index+1, lonmin_index));
  
  /* approximate declination within the grid using bilinear interpolation */
  decmin = (lon - lonmin) / 5 * (decSE - decSW) + decSW;
  decmax = (lon - lonmin) / 5 * (decNE - decNW) + decNW;
  return (lat - latmin) / 5 * (decmax - decmin) + decmin;
}

