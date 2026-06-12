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



#include "routines.h"
#include <math.h>
#include <string.h>


int sphere_fit_least_squares(const float x[], const float y[], const float z[],
                             unsigned int size, unsigned int max_iterations, float delta, float *sphere_x, float *sphere_y, float *sphere_z, float *sphere_radius)
{
  
  float x_sumplain = 0.0f;
  float x_sumsq = 0.0f;
  float x_sumcube = 0.0f;
  
  float y_sumplain = 0.0f;
  float y_sumsq = 0.0f;
  float y_sumcube = 0.0f;
  
  float z_sumplain = 0.0f;
  float z_sumsq = 0.0f;
  float z_sumcube = 0.0f;
  
  float xy_sum = 0.0f;
  float xz_sum = 0.0f;
  float yz_sum = 0.0f;
  
  float x2y_sum = 0.0f;
  float x2z_sum = 0.0f;
  float y2x_sum = 0.0f;
  float y2z_sum = 0.0f;
  float z2x_sum = 0.0f;
  float z2y_sum = 0.0f;
  
  for (unsigned int i = 0; i < size; i++) {
    
    float x2 = x[i] * x[i];
    float y2 = y[i] * y[i];
    float z2 = z[i] * z[i];
    
    x_sumplain += x[i];
    x_sumsq += x2;
    x_sumcube += x2 * x[i];
    
    y_sumplain += y[i];
    y_sumsq += y2;
    y_sumcube += y2 * y[i];
    
    z_sumplain += z[i];
    z_sumsq += z2;
    z_sumcube += z2 * z[i];
    
    xy_sum += x[i] * y[i];
    xz_sum += x[i] * z[i];
    yz_sum += y[i] * z[i];
    
    x2y_sum += x2 * y[i];
    x2z_sum += x2 * z[i];
    
    y2x_sum += y2 * x[i];
    y2z_sum += y2 * z[i];
    
    z2x_sum += z2 * x[i];
    z2y_sum += z2 * y[i];
  }
  
  //
  //Least Squares Fit a sphere A,B,C with radius squared Rsq to 3D data
  //
  //    P is a structure that has been computed with the data earlier.
  //    P.npoints is the number of elements; the length of X,Y,Z are identical.
  //    P's members are logically named.
  //
  //    X[n] is the x component of point n
  //    Y[n] is the y component of point n
  //    Z[n] is the z component of point n
  //
  //    A is the x coordiante of the sphere
  //    B is the y coordiante of the sphere
  //    C is the z coordiante of the sphere
  //    Rsq is the radius squared of the sphere.
  //
  //This method should converge; maybe 5-100 iterations or more.
  //
  float x_sum = x_sumplain / size;        //sum( X[n] )
  float x_sum2 = x_sumsq / size;    //sum( X[n]^2 )
  float x_sum3 = x_sumcube / size;    //sum( X[n]^3 )
  float y_sum = y_sumplain / size;        //sum( Y[n] )
  float y_sum2 = y_sumsq / size;    //sum( Y[n]^2 )
  float y_sum3 = y_sumcube / size;    //sum( Y[n]^3 )
  float z_sum = z_sumplain / size;        //sum( Z[n] )
  float z_sum2 = z_sumsq / size;    //sum( Z[n]^2 )
  float z_sum3 = z_sumcube / size;    //sum( Z[n]^3 )
  
  float XY = xy_sum / size;        //sum( X[n] * Y[n] )
  float XZ = xz_sum / size;        //sum( X[n] * Z[n] )
  float YZ = yz_sum / size;        //sum( Y[n] * Z[n] )
  float X2Y = x2y_sum / size;    //sum( X[n]^2 * Y[n] )
  float X2Z = x2z_sum / size;    //sum( X[n]^2 * Z[n] )
  float Y2X = y2x_sum / size;    //sum( Y[n]^2 * X[n] )
  float Y2Z = y2z_sum / size;    //sum( Y[n]^2 * Z[n] )
  float Z2X = z2x_sum / size;    //sum( Z[n]^2 * X[n] )
  float Z2Y = z2y_sum / size;    //sum( Z[n]^2 * Y[n] )
  
  //Reduction of multiplications
  float F0 = x_sum2 + y_sum2 + z_sum2;
  float F1 =  0.5f * F0;
  float F2 = -8.0f * (x_sum3 + Y2X + Z2X);
  float F3 = -8.0f * (X2Y + y_sum3 + Z2Y);
  float F4 = -8.0f * (X2Z + Y2Z + z_sum3);
  
  //Set initial conditions:
  float A = x_sum;
  float B = y_sum;
  float C = z_sum;
  
  //First iteration computation:
  float A2 = A * A;
  float B2 = B * B;
  float C2 = C * C;
  float QS = A2 + B2 + C2;
  float QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
  
  //Set initial conditions:
  float Rsq = F0 + QB + QS;
  
  //First iteration computation:
  float Q0 = 0.5f * (QS - Rsq);
  float Q1 = F1 + Q0;
  float Q2 = 8.0f * (QS - Rsq + QB + F0);
  float aA, aB, aC, nA, nB, nC, dA, dB, dC;
  
  //Iterate N times, ignore stop condition.
  unsigned int n = 0;
  
  while (n < max_iterations) {
    n++;
    
    //Compute denominator:
    aA = Q2 + 16.0f * (A2 - 2.0f * A * x_sum + x_sum2);
    aB = Q2 + 16.0f * (B2 - 2.0f * B * y_sum + y_sum2);
    aC = Q2 + 16.0f * (C2 - 2.0f * C * z_sum + z_sum2);
    aA = (aA == 0.0f) ? 1.0f : aA;
    aB = (aB == 0.0f) ? 1.0f : aB;
    aC = (aC == 0.0f) ? 1.0f : aC;
    
    //Compute next iteration
    nA = A - ((F2 + 16.0f * (B * XY + C * XZ + x_sum * (-A2 - Q0) + A * (x_sum2 + Q1 - C * z_sum - B * y_sum))) / aA);
    nB = B - ((F3 + 16.0f * (A * XY + C * YZ + y_sum * (-B2 - Q0) + B * (y_sum2 + Q1 - A * x_sum - C * z_sum))) / aB);
    nC = C - ((F4 + 16.0f * (A * XZ + B * YZ + z_sum * (-C2 - Q0) + C * (z_sum2 + Q1 - A * x_sum - B * y_sum))) / aC);
    
    //Check for stop condition
    dA = (nA - A);
    dB = (nB - B);
    dC = (nC - C);
    
    if ((dA * dA + dB * dB + dC * dC) <= delta) { break; }
    
    //Compute next iteration's values
    A = nA;
    B = nB;
    C = nC;
    A2 = A * A;
    B2 = B * B;
    C2 = C * C;
    QS = A2 + B2 + C2;
    QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
    Rsq = F0 + QB + QS;
    Q0 = 0.5f * (QS - Rsq);
    Q1 = F1 + Q0;
    Q2 = 8.0f * (QS - Rsq + QB + F0);
  }
  
  *sphere_x = A;
  *sphere_y = B;
  *sphere_z = C;
  *sphere_radius = sqrtf(Rsq);
  
  return 0;
}





//original source of：https://github.com/stevenjiaweixie/vrgimbal
//https://github.com/stevenjiaweixie/vrgimbal/blob/master/Firmware/VRGimbal/calibrationRoutines.cpp
//来源于开源云台项目：https://vrgimbal.wordpress.com/
Least_Squares_Intermediate_Variable Mag_LS;
void LS_Init(Least_Squares_Intermediate_Variable * pLSQ)
{
  memset(pLSQ, 0, sizeof(Least_Squares_Intermediate_Variable));
}

unsigned int LS_Accumulate(Least_Squares_Intermediate_Variable * pLSQ, float x, float y, float z)
{
  float x2 = x * x;
  float y2 = y * y;
  float z2 = z * z;
  
  pLSQ->x_sumplain += x;
  pLSQ->x_sumsq += x2;
  pLSQ->x_sumcube += x2 * x;
  
  pLSQ->y_sumplain += y;
  pLSQ->y_sumsq += y2;
  pLSQ->y_sumcube += y2 * y;
  
  pLSQ->z_sumplain += z;
  pLSQ->z_sumsq += z2;
  pLSQ->z_sumcube += z2 * z;
  
  pLSQ->xy_sum += x * y;
  pLSQ->xz_sum += x * z;
  pLSQ->yz_sum += y * z;
  
  pLSQ->x2y_sum += x2 * y;
  pLSQ->x2z_sum += x2 * z;
  
  pLSQ->y2x_sum += y2 * x;
  pLSQ->y2z_sum += y2 * z;
  
  pLSQ->z2x_sum += z2 * x;
  pLSQ->z2y_sum += z2 * y;
  
  pLSQ->size++;
  
  return pLSQ->size;
}


void LS_Calculate(Least_Squares_Intermediate_Variable * pLSQ,
                  unsigned int max_iterations,
                  float delta,
                  float *sphere_x, float *sphere_y, float *sphere_z,
                  float *sphere_radius)
{
  //
  //Least Squares Fit a sphere A,B,C with radius squared Rsq to 3D data
  //
  //    P is a structure that has been computed with the data earlier.
  //    P.npoints is the number of elements; the length of X,Y,Z are identical.
  //    P's members are logically named.
  //
  //    X[n] is the x component of point n
  //    Y[n] is the y component of point n
  //    Z[n] is the z component of point n
  //
  //    A is the x coordiante of the sphere
  //    B is the y coordiante of the sphere
  //    C is the z coordiante of the sphere
  //    Rsq is the radius squared of the sphere.
  //
  //This method should converge; maybe 5-100 iterations or more.
  //
  float x_sum = pLSQ->x_sumplain / pLSQ->size;        //sum( X[n] )
  float x_sum2 = pLSQ->x_sumsq / pLSQ->size;    //sum( X[n]^2 )
  float x_sum3 = pLSQ->x_sumcube / pLSQ->size;    //sum( X[n]^3 )
  float y_sum = pLSQ->y_sumplain / pLSQ->size;        //sum( Y[n] )
  float y_sum2 = pLSQ->y_sumsq / pLSQ->size;    //sum( Y[n]^2 )
  float y_sum3 = pLSQ->y_sumcube / pLSQ->size;    //sum( Y[n]^3 )
  float z_sum = pLSQ->z_sumplain / pLSQ->size;        //sum( Z[n] )
  float z_sum2 = pLSQ->z_sumsq / pLSQ->size;    //sum( Z[n]^2 )
  float z_sum3 = pLSQ->z_sumcube / pLSQ->size;    //sum( Z[n]^3 )
  
  float XY = pLSQ->xy_sum / pLSQ->size;        //sum( X[n] * Y[n] )
  float XZ = pLSQ->xz_sum / pLSQ->size;        //sum( X[n] * Z[n] )
  float YZ = pLSQ->yz_sum / pLSQ->size;        //sum( Y[n] * Z[n] )
  float X2Y = pLSQ->x2y_sum / pLSQ->size;    //sum( X[n]^2 * Y[n] )
  float X2Z = pLSQ->x2z_sum / pLSQ->size;    //sum( X[n]^2 * Z[n] )
  float Y2X = pLSQ->y2x_sum / pLSQ->size;    //sum( Y[n]^2 * X[n] )
  float Y2Z = pLSQ->y2z_sum / pLSQ->size;    //sum( Y[n]^2 * Z[n] )
  float Z2X = pLSQ->z2x_sum / pLSQ->size;    //sum( Z[n]^2 * X[n] )
  float Z2Y = pLSQ->z2y_sum / pLSQ->size;    //sum( Z[n]^2 * Y[n] )
  
  //Reduction of multiplications
  float F0 = x_sum2 + y_sum2 + z_sum2;
  float F1 =  0.5f * F0;
  float F2 = -8.0f * (x_sum3 + Y2X + Z2X);
  float F3 = -8.0f * (X2Y + y_sum3 + Z2Y);
  float F4 = -8.0f * (X2Z + Y2Z + z_sum3);
  
  //Set initial conditions:
  float A = x_sum;
  float B = y_sum;
  float C = z_sum;
  
  //First iteration computation:
  float A2 = A * A;
  float B2 = B * B;
  float C2 = C * C;
  float QS = A2 + B2 + C2;
  float QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
  
  //Set initial conditions:
  float Rsq = F0 + QB + QS;
  
  //First iteration computation:
  float Q0 = 0.5f * (QS - Rsq);
  float Q1 = F1 + Q0;
  float Q2 = 8.0f * (QS - Rsq + QB + F0);
  float aA, aB, aC, nA, nB, nC, dA, dB, dC;
  
  //Iterate N times, ignore stop condition.
  unsigned int n = 0;
  
  while (n < max_iterations) {
    n++;
    
    //Compute denominator:
    aA = Q2 + 16.0f * (A2 - 2.0f * A * x_sum + x_sum2);
    aB = Q2 + 16.0f * (B2 - 2.0f * B * y_sum + y_sum2);
    aC = Q2 + 16.0f * (C2 - 2.0f * C * z_sum + z_sum2);
    aA = (aA == 0.0f) ? 1.0f : aA;
    aB = (aB == 0.0f) ? 1.0f : aB;
    aC = (aC == 0.0f) ? 1.0f : aC;
    
    //Compute next iteration
    nA = A - ((F2 + 16.0f * (B * XY + C * XZ + x_sum * (-A2 - Q0) + A * (x_sum2 + Q1 - C * z_sum - B * y_sum))) / aA);
    nB = B - ((F3 + 16.0f * (A * XY + C * YZ + y_sum * (-B2 - Q0) + B * (y_sum2 + Q1 - A * x_sum - C * z_sum))) / aB);
    nC = C - ((F4 + 16.0f * (A * XZ + B * YZ + z_sum * (-C2 - Q0) + C * (z_sum2 + Q1 - A * x_sum - B * y_sum))) / aC);
    
    //Check for stop condition
    dA = (nA - A);
    dB = (nB - B);
    dC = (nC - C);
    
    if ((dA * dA + dB * dB + dC * dC) <= delta) { break; }
    
    //Compute next iteration's values
    A = nA;
    B = nB;
    C = nC;
    A2 = A * A;
    B2 = B * B;
    C2 = C * C;
    QS = A2 + B2 + C2;
    QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
    Rsq = F0 + QB + QS;
    Q0 = 0.5f * (QS - Rsq);
    Q1 = F1 + Q0;
    Q2 = 8.0f * (QS - Rsq + QB + F0);
  }
  
  *sphere_x = A;
  *sphere_y = B;
  *sphere_z = C;
  *sphere_radius = sqrt(Rsq);
}


/***************加速度计6面矫正，参考APM代码，配合遥控器进行现场矫正**************************/
/***********************************************************
@函数名：Calibrate_Reset_Matrices
@入口参数：float dS[6], float JS[6][6]
@出口参数：无
@功能描述：矩阵数据复位
@作者：无名小哥
@日期：2019年01月27日
*************************************************************/
void Calibrate_Reset_Matrices(float dS[6], float JS[6][6])
{
  int16_t j,k;
  for( j=0; j<6; j++ )
  {
    dS[j] = 0.0f;
    for( k=0; k<6; k++ )
    {
      JS[j][k] = 0.0f;
    }
  }
}


/***********************************************************
@函数名：Calibrate_Find_Delta
@入口参数：float dS[6], float JS[6][6], float delta[6]
@出口参数：无
@功能描述：求解矩阵方程JS*x = dS，第一步把矩阵化上三角阵，
将JS所在的列下方的全部消为0，然后回代得到线性方程的解
@作者：无名小哥
@日期：2019年01月27日
*************************************************************/
void Calibrate_Find_Delta(float dS[6], float JS[6][6], float delta[6])
{
  //Solve 6-d matrix equation JS*x = dS
  //first put in upper triangular form
  int16_t i,j,k;
  float mu;
  //make upper triangular
  for( i=0; i<6; i++ ) {
    //eliminate all nonzero entries below JS[i][i]
    for( j=i+1; j<6; j++ ) {
      mu = JS[i][j]/JS[i][i];
      if( mu != 0.0f ) {
        dS[j] -= mu*dS[i];
        for( k=j; k<6; k++ ) {
          JS[k][j] -= mu*JS[k][i];
        }
      }
    }
  }
  //back-substitute
  for( i=5; i>=0; i-- ) {
    dS[i] /= JS[i][i];
    JS[i][i] = 1.0f;
    
    for( j=0; j<i; j++ ) {
      mu = JS[i][j];
      dS[j] -= mu*dS[i];
      JS[i][j] = 0.0f;
    }
  }
  for( i=0; i<6; i++ ) {
    delta[i] = dS[i];
  }
}


#define N 6
void gaussElimination(float mat_Y[N],float mat_A[N][N],float x[N]) 
{
  double mat[N][N + 1];
  for(uint16_t i=0;i<N;i++)
  {
    for(uint16_t j=0;j<N+1;j++)	
    {
      if(j!=N)  mat[i][j]=mat_A[i][j];
      else mat[i][j]=mat_Y[i];
    }
  }
  
  for (int i = 0; i < N; i++) {
    // Making the diagonal element non-zero
    if (mat[i][i] == 0) {
      for (int k = i + 1; k < N; k++) {
        if (mat[k][i] != 0) {
          for (int j = 0; j <= N; j++) {
            double temp = mat[i][j];
            mat[i][j] = mat[k][j];
            mat[k][j] = temp;
          }
          break;
        }
      }
    }
    
    // Making the elements below the diagonal zero
    for (int k = i + 1; k < N; k++) {
      double factor = mat[k][i] / mat[i][i];
      for (int j = 0; j <= N; j++) {
        mat[k][j] -= factor * mat[i][j];
      }
    }
  }
  // mat是高斯消元后的矩阵
  for (int i = N - 1; i >= 0; i--) {
    x[i] = mat[i][N];
    for (int j = i + 1; j < N; j++) {
      x[i] -= mat[i][j] * x[j];
    }
    x[i] /= mat[i][i];
  }		
}



void Calibrate_Update_Matrices(float dS[6],
                               float JS[6][6],
                               float beta[6],
                               float data[3])
{
  int16_t j, k;
  float dx, b;
  float residual = 1.0;
  float jacobian[6];
  for(j=0;j<3;j++)
  {
    b = beta[3+j];
    dx = (float)data[j] - beta[j];
    residual -= b*b*dx*dx;
    jacobian[j] = 2.0f*b*b*dx;
    jacobian[3+j] = -2.0f*b*dx*dx;
  }
  
  for(j=0;j<6;j++)
  {
    dS[j]+=jacobian[j]*residual;
    for(k=0;k<6;k++)
    {
      JS[j][k]+=jacobian[j]*jacobian[k];
    }
  }
}



uint8_t Calibrate_accel(vector3f accel_sample[6],
                        vector3f *accel_offsets,
                        vector3f *accel_scale)
{
  int16_t i;
  int16_t num_iterations = 0;
  float eps = 0.000000001;
  float change = 100.0;
  float data[3]={0};
  float beta[6]={0};
  float delta[6]={0};
  float ds[6]={0};
  float JS[6][6]={0};
  bool success = true;
  // reset
  beta[0] = beta[1] = beta[2] = 0;
  beta[3] = beta[4] = beta[5] = 1.0f/GRAVITY_MSS;
  while( num_iterations < 20 && change > eps ) {
    num_iterations++;
    Calibrate_Reset_Matrices(ds, JS);
    for( i=0; i<6; i++ ) {
      data[0] = accel_sample[i].x;
      data[1] = accel_sample[i].y;
      data[2] = accel_sample[i].z;
      Calibrate_Update_Matrices(ds, JS, beta, data);
    }
    //Calibrate_Find_Delta(ds, JS, delta);
    gaussElimination(ds, JS, delta);
    change = delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2]
            +delta[3]*delta[3]/(beta[3]*beta[3])
            +delta[4]*delta[4]/(beta[4]*beta[4])
            +delta[5]*delta[5]/(beta[5]*beta[5]);
    for( i=0; i<6; i++ ) {
      beta[i] -= delta[i];
    }
  }
  // copy results out
  accel_scale->x   = beta[3] * GRAVITY_MSS;
  accel_scale->y   = beta[4] * GRAVITY_MSS;
  accel_scale->z   = beta[5] * GRAVITY_MSS;
  accel_offsets->x = beta[0] * accel_scale->x;
  accel_offsets->y = beta[1] * accel_scale->y;
  accel_offsets->z = beta[2] * accel_scale->z;
  
  // sanity check scale
  if(fabsf(accel_scale->x-1.0f) > 0.2f
     ||fabsf(accel_scale->y-1.0f) > 0.2f
       ||fabsf(accel_scale->z-1.0f) > 0.2f )
  {
    success = false;
  }
  // sanity check offsets (3.0 is roughly 3/10th of a G, 5.0 is roughly half a G)
  if(fabsf(accel_offsets->x) > 5.0f
     ||fabsf(accel_offsets->y) > 5.0f
       ||fabsf(accel_offsets->z) > 5.0f )
  {
    success = false;
  }
  // return success or failure
  return success;
}









