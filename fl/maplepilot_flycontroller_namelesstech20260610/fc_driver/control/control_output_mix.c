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
#include "arm_math.h"
#include "control_output_mix.h"

#define	MOTOR_FRAME_TYPE_X4   0 //X形四轴
#define MOTOR_FRAME_TYPE_H4   1 //H形四轴
#define	MOTOR_FRAME_TYPE_C4   2 //十字形四轴
#define MOTOR_FRAME_TYPE_X6   3 //X形六轴
#define MOTOR_FRAME_TYPE_C6   4 //十字形六轴
#define MOTOR_FRAME_TYPE_X8   5 //X形八轴
#define MOTOR_FRAME_TYPE_C8   6 //十字形八轴
#define MOTOR_FRAME_TYPE_YY3  7 //十字形八轴
#define MOTOR_FRAME_TYPE_XX4  8 //十字形八轴


#define MOTOR_FRAME_TYPE_DEFAULT MOTOR_FRAME_TYPE_X4
#define X_SCALE_ROW 4

#if MOTOR_FRAME_TYPE_DEFAULT==MOTOR_FRAME_TYPE_X4
#define X_SCALE_COL 4
#elif (MOTOR_FRAME_TYPE==MOTOR_FRAME_TYPE_H4)
#define X_SCALE_COL 4
#elif (MOTOR_FRAME_TYPE==MOTOR_FRAME_TYPE_C4)
#define X_SCALE_COL 4
#elif (MOTOR_FRAME_TYPE==MOTOR_FRAME_TYPE_X6)
#define X_SCALE_COL 6
#elif (MOTOR_FRAME_TYPE==MOTOR_FRAME_TYPE_C6)
#define X_SCALE_COL 6
#elif (MOTOR_FRAME_TYPE==MOTOR_FRAME_TYPE_X8)
#define X_SCALE_COL 8
#elif (MOTOR_FRAME_TYPE==MOTOR_FRAME_TYPE_C8)
#define X_SCALE_COL 8
#elif (MOTOR_FRAME_TYPE==MOTOR_FRAME_TYPE_YY3)
#define X_SCALE_COL 6
#elif (MOTOR_FRAME_TYPE==MOTOR_FRAME_TYPE_XX4)
#define X_SCALE_COL 8
#endif

arm_matrix_instance_f32 X_SCALE_Matrix,XI_SCALE_Matrix,XT_SCALE_Matrix;
arm_matrix_instance_f32 XXT_SCALE_Matrix,XXTI_SCALE_Matrix;


float X_SCALE[X_SCALE_ROW*X_SCALE_COL],INV_X_SCALE[X_SCALE_COL*X_SCALE_ROW];
float XT_SCALE[X_SCALE_COL*X_SCALE_ROW],XXT_SCALE[X_SCALE_ROW*X_SCALE_ROW],INV_XXT_SCALE[X_SCALE_ROW*X_SCALE_ROW];

uint8_t uav_motor_frame_type= MOTOR_FRAME_TYPE_DEFAULT;
void motor_control_matrix_generate(uint8_t uav_model)
{
  float *matrix=X_SCALE;
  arm_mat_init_f32(&X_SCALE_Matrix,   X_SCALE_ROW, X_SCALE_COL, X_SCALE);
  arm_mat_init_f32(&XI_SCALE_Matrix,  X_SCALE_COL, X_SCALE_ROW, INV_X_SCALE);
  arm_mat_init_f32(&XT_SCALE_Matrix,  X_SCALE_COL, X_SCALE_ROW, XT_SCALE);
  arm_mat_init_f32(&XXT_SCALE_Matrix, X_SCALE_ROW, X_SCALE_ROW, XXT_SCALE);
  arm_mat_init_f32(&XXTI_SCALE_Matrix,X_SCALE_ROW, X_SCALE_ROW, INV_XXT_SCALE);	
  switch(uav_model)
  {
  case MOTOR_FRAME_TYPE_X4:
  case MOTOR_FRAME_TYPE_H4:
    {
      matrix[0]=1;						matrix[1]=1;						    matrix[2]=1;						  matrix[3]=1;
      matrix[4]=sqrtf(2)/2;	  matrix[5]=-sqrtf(2)/2;	    matrix[6]=-sqrtf(2)/2;	  matrix[7]= sqrtf(2)/2;			
      matrix[8]=sqrtf(2)/2;	  matrix[9]=-sqrtf(2)/2;	    matrix[10]= sqrtf(2)/2;	  matrix[11]=-sqrtf(2)/2;			
      matrix[12]=-1;				  matrix[13]=-1;					    matrix[14]=1;					    matrix[15]=1;
    }
    break;
  case MOTOR_FRAME_TYPE_C4:
    {
      matrix[0]=1;						  matrix[1]=1;						    matrix[2]=1;				  matrix[3]=1;
      matrix[4]=0;	  					matrix[5]=0;	   					  matrix[6]=-1;	 				matrix[7]=1;			
      matrix[8]=1;	            matrix[9]=-1;	              matrix[10]=0;					matrix[11]=0;			
      matrix[12]=-1;					  matrix[13]=-1;					    matrix[14]=1;					matrix[15]=1;	
    }
    break;
  case MOTOR_FRAME_TYPE_X6:
    {
      matrix[0]=1;							matrix[1]=1;		  				matrix[2]=1;						matrix[3]=1;    					matrix[4]=1;			matrix[5]=1;
      matrix[6]=0.5;						matrix[7]=-0.5;						matrix[8]=-0.5;					matrix[9]=0.5;  					matrix[10]=-1;		matrix[11]=1;
      matrix[12]=sqrtf(3)/2;	  matrix[13]=-sqrtf(3)/2;		matrix[14]=sqrtf(3)/2;	matrix[15]=-sqrtf(3)/2;   matrix[16]=0;			matrix[17]=0;	
      matrix[18]=-1;	  				matrix[19]=1;							matrix[20]=1;	  				matrix[21]=-1;  					matrix[22]=-1;		matrix[23]=1;
    }
    break;
  case MOTOR_FRAME_TYPE_C6:
    {
      matrix[0]=1;							matrix[1]=1;		  			  matrix[2]=1;						matrix[3]=1;    				 matrix[4]=1;			          matrix[5]=1;
      matrix[6]=0;						  matrix[7]=0;						  matrix[8]=-sqrtf(3)/2;  matrix[9]=sqrtf(3)/2;  	 matrix[10]=-sqrtf(3)/2;		matrix[11]=sqrtf(3)/2;
      matrix[12]=1;	            matrix[13]=-1;		        matrix[14]=0.5;	        matrix[15]=-0.5;         matrix[16]=-0.5;			      matrix[17]=0.5;	
      matrix[18]=-1;	  				matrix[19]=1;							matrix[20]=1;	  				matrix[21]=-1;  				 matrix[22]=-1;			        matrix[23]=1;
    }
    break;
  case MOTOR_FRAME_TYPE_X8:
    {
      matrix[0]=1;			 matrix[1]=1;		  	matrix[2]=1;			 matrix[3]=1;    	  matrix[4]=1;			   matrix[5]=1; 				matrix[6]=1;			  matrix[7]=1;
      matrix[8]=0.2588;  matrix[9]=-0.2588;	matrix[10]=-0.2588;matrix[11]=0.2588; matrix[12]=-0.9659;  matrix[13]=0.9659; 	matrix[14]=-0.9659; matrix[15]=0.9659;
      matrix[16]=0.9659; matrix[17]=-0.9659;matrix[18]=0.9659; matrix[19]=-0.9659;matrix[20]=0.2588;	 matrix[21]=-0.2588; 	matrix[22]=-0.2588; matrix[23]=0.2588;
      matrix[24]=-1;		 matrix[25]=-1;		  matrix[26]=1;			 matrix[27]=1;    	matrix[28]=-1;			 matrix[29]=-1; 			matrix[30]=1;			  matrix[31]=1;
    }
    break;
  case MOTOR_FRAME_TYPE_C8:
    {
      matrix[0]=1;			 matrix[1]=1;		  	matrix[2]=1;				matrix[3]=1;    	 matrix[4]=1;			    matrix[5]=1; 				matrix[6]=1;			  matrix[7]=1;
      matrix[8]=0; 		   matrix[9]=0;	      matrix[10]=-0.7071; matrix[11]=0.7071; matrix[12]=-1;       matrix[13]=1; 	    matrix[14]=-0.7071; matrix[15]=0.7071;
      matrix[16]=1;      matrix[17]=-1;	    matrix[18]=0.7071;	matrix[19]=-0.7071;matrix[20]=0;	      matrix[21]=0; 	    matrix[22]=-0.7071; matrix[23]=0.7071;
      matrix[24]=-1;		 matrix[25]=-1;		  matrix[26]=1;			  matrix[27]=1;    	 matrix[28]=-1;			  matrix[29]=-1; 			matrix[30]=1;			  matrix[31]=1;		
    }
    break;
  case MOTOR_FRAME_TYPE_YY3:
    {
      matrix[0]=1;							matrix[1]=1;		  					matrix[2]=1;							matrix[3]=1;    				matrix[4]=1;			 matrix[5]=1;
      matrix[6]=0.8660;				  matrix[7]=-0.8660;					matrix[8]=0;					    matrix[9]=0.8660;  			matrix[10]=-0.8660;matrix[11]=0;
      matrix[12]=0.5;	          matrix[13]=0.5;		          matrix[14]=-1;	          matrix[15]=0.5;         matrix[16]=0.5;		 matrix[17]=-1;	
      matrix[18]=-1;	  				matrix[19]=-1;							matrix[20]=-1;	  				matrix[21]=1;  					matrix[22]=1;			 matrix[23]=1;
    }
    break;
  case MOTOR_FRAME_TYPE_XX4:
    {
      matrix[0]=1;							matrix[1]=1;		  					matrix[2]=1;							matrix[3]=1;    					matrix[4]=1;			  matrix[5]=1;           matrix[6]=1;			  matrix[7]=1;
      matrix[8]=0.7071;				  matrix[9]=-0.7071;					matrix[10]=-0.7071;			  matrix[11]=0.7071;  			matrix[12]=0.7071;	matrix[13]=-0.7071;		 matrix[14]=-0.7071;matrix[15]=0.7071;
      matrix[16]=0.7071;				matrix[17]=-0.7071;				  matrix[18]=0.7071;			  matrix[19]=-0.7071;  		  matrix[20]=0.7071;	matrix[21]=-0.7071;		 matrix[22]=0.7071;	matrix[23]=-0.7071;
      matrix[24]=-1;	  				matrix[25]=-1;							matrix[26]=-1;	  				matrix[27]=-1;  					matrix[28]=1;			  matrix[29]=1;			     matrix[30]=1;			matrix[31]=1;
    }
    break;
  default:
    {
      matrix[0]=1;						  matrix[1]=1;						    matrix[2]=1;				      matrix[3]=1;
      matrix[4]=sqrtf(2)/2;	    matrix[5]=-sqrtf(2)/2;	    matrix[6]=-sqrtf(2)/2;	  matrix[7]= sqrtf(2)/2;			
      matrix[8]=sqrtf(2)/2;	    matrix[9]=-sqrtf(2)/2;	    matrix[10]= sqrtf(2)/2;	  matrix[11]=-sqrtf(2)/2;			
      matrix[12]=-1;					  matrix[13]=-1;					    matrix[14]=1;					    matrix[15]=1;
    }break;
  }
  
  //求控制率矩阵的伪逆矩阵——输出矩阵
  arm_mat_trans_f32(&X_SCALE_Matrix,&XT_SCALE_Matrix);
  arm_mat_mult_f32(&X_SCALE_Matrix,&XT_SCALE_Matrix,&XXT_SCALE_Matrix);
  arm_mat_inverse_f32(&XXT_SCALE_Matrix,&XXTI_SCALE_Matrix);
  arm_mat_mult_f32(&XT_SCALE_Matrix,&XXTI_SCALE_Matrix,&XI_SCALE_Matrix);
}




float thrust_scale[X_SCALE_COL],roll_scale[X_SCALE_COL],pitch_scale[X_SCALE_COL],yaw_scale[X_SCALE_COL];
void Motor_Control_Rate_Pure(float _thr,float _rol,float _pit,float _yaw,float *motor_output)
{
  float thr_norm,rol_norm, pit_norm,yaw_norm,_idle_speed;			
  motor_control_matrix_generate(uav_motor_frame_type);
  for(uint16_t i=0;i<X_SCALE_ROW*X_SCALE_COL;i++)
  {
    INV_X_SCALE[i]*=X_SCALE_COL;
  }			
  
  for(uint16_t i=0;i<X_SCALE_COL;i++)
  {
    thrust_scale[i]=INV_X_SCALE[4*i+0];
    roll_scale[i]  =INV_X_SCALE[4*i+1];
    pitch_scale[i] =INV_X_SCALE[4*i+2];
    yaw_scale[i]   =INV_X_SCALE[4*i+3];
  }
  
  _idle_speed=constrain_float((THR_IDEL_OUTPUT-THR_MIN_OUTPUT)*0.001f,0.0f,1.0f);
  //统一将控制量统一量化到-1~1以内
  thr_norm=constrain_float((_thr-THR_MIN_OUTPUT)*0.001f,0.0f,1.0f);
  rol_norm=constrain_float(_rol*(-0.001f),-1.0f,1.0f);
  pit_norm=constrain_float(_pit*0.001f,-1.0f,1.0f);
  yaw_norm=constrain_float(_yaw*0.001f,-1.0f,1.0f);
  
  
  /*将偏航和缩放输出范围映射到怠速-满输出*/
  for (uint8_t  i = 0; i < X_SCALE_COL; i++) {
    motor_output[i] =  rol_norm * roll_scale[i]  +
      pit_norm * pitch_scale[i] +
        yaw_norm * yaw_scale[i]   +
          thr_norm;
    
    motor_output[i] = constrain_float(_idle_speed + (motor_output[i]*(1.0f-_idle_speed)),_idle_speed, 1.0f);
    motor_output[i] = constrain_float(THR_MIN_OUTPUT+motor_output[i]*(THR_MAX_OUTPUT-THR_MIN_OUTPUT),THR_MIN_OUTPUT,THR_MAX_OUTPUT);
  }
}

/*混合策略概述：
1） 混合横摇、俯仰和推力，无偏航。
2） 如果某些输出违反范围[0,1]，则尝试移动所有输出以最小化违反->增加或减少总推力（增压）。推力的总增加或减少是有限的
（最大推力差）。如果移位后某些输出仍然违反边界，则缩放横滚和俯仰。如果在下限和上限存在违规，则尝试移位，以使违规相等两边都有。
3） 如果导致违反限制，则混合偏航和缩放。
4） 将所有输出缩放至范围[怠速，1]
*/


