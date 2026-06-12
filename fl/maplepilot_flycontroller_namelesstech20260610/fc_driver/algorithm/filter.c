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

#include "wp_math.h"
#include "datatype.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "filter.h"




void set_cutoff_frequency(float sample_frequent, float cutoff_frequent,filter_parameter *lpf)
{
  float fr = sample_frequent / cutoff_frequent;
  float ohm = tanf(M_PI_F / fr);
  float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
  if (cutoff_frequent <= 0.0f) { 
    return;// no filtering
  }
  lpf->b[0] = ohm * ohm / c;
  lpf->b[1] = 2.0f * lpf->b[0];
  lpf->b[2] = lpf->b[0];
  lpf->a[0]=1.0f;
  lpf->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
  lpf->a[2] = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}


float butterworth(float curr_input,filter_buffer *buf,filter_parameter *lpf)
{
  /* 加速度计Butterworth滤波 */
  /* 获取最新x(n) */
  buf->input[2]=curr_input;
  /* Butterworth滤波 */
  buf->output[2]=lpf->b[0] * buf->input[2]	\
    +lpf->b[1] * buf->input[1]	\
      +lpf->b[2] * buf->input[0]	\
        -lpf->a[1] * buf->output[1]	\
          -lpf->a[2] * buf->output[0];
  /* x(n) 序列保存 */
  buf->input[0]=buf->input[1];
  buf->input[1]=buf->input[2];
  /* y(n) 序列保存 */
  buf->output[0]=buf->output[1];
  buf->output[1]=buf->output[2];
  uint16_t i=0;
  for(i=0;i<3;i++)
  {
    if(isnan(buf->output[i])==1) 
      buf->output[i]=curr_input;
    if(isnan(buf->input[i])==1)  
      buf->input[i]=curr_input;
  }
  return buf->output[2];
}