#include <stdint.h>
#include "pid.h"
#include "Motor.control.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
//#include "motor.pwm.h"


/*------------------------------------------
 				声明变量				
------------------------------------------*/
extern M1TypeDef  M1;
extern M2TypeDef  M2;

PIDTypdDef M1PID = {0};
PIDTypdDef M2PID = {0};
/*------------------------------------------
 函数功能:初始化M1PID结构体参数
 函数说明:			
------------------------------------------*/
void PID_M1_Init(void)
{
    M1PID.LastError  = 0;			//Error[-1]
    M1PID.PrevError  = 0;			//Error[-2]
	M1PID.Proportion = 0;			//比例常数 Proportional Const
    M1PID.Integral   = 0;			//积分常数 Integral Const
    M1PID.Derivative = 0;			//微分常数 Derivative Const
    M1PID.SetPoint   = 0;
	M1PID.SumError   = 0;
}
/*------------------------------------------
 函数功能:初始化M2PID结构体参数
 函数说明:			
------------------------------------------*/
void PID_M2_Init(void)
{
    M2PID.LastError  = 0;			//Error[-1]
    M2PID.PrevError  = 0;			//Error[-2]
	M2PID.Proportion = 0;			//比例常数 Proportional Const
    M2PID.Integral   = 0;			//积分常数 Integral Const
    M2PID.Derivative = 0;			//微分常数 Derivative Const
    M2PID.SetPoint   = 0;
	M2PID.SumError   = 0;
}
/*------------------------------------------
 函数功能:设置M1PID期望值
 函数说明:			
------------------------------------------*/
void PID_M1_SetPoint(float setpoint)
{	
	M1PID.SetPoint = setpoint;	
}
/*------------------------------------------
 函数功能:设置M2期望值
 函数说明:			
------------------------------------------*/
void PID_M2_SetPoint(float setpoint)
{	
	M2PID.SetPoint = setpoint;	
}
/*------------------------------------------
 函数功能:设置M1PID比例系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M1_SetKp(float dKpp)
{	
	M1PID.Proportion = dKpp;	
}
/*------------------------------------------
 函数功能:设置M2比例系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M2_SetKp(float dKpp)
{	
	M2PID.Proportion = dKpp;	
}
/*------------------------------------------
 函数功能:设置M1PID积分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M1_SetKi(float dKii)
{	
	M1PID.Integral = dKii;	
}
/*------------------------------------------
 函数功能:设置M2积分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M2_SetKi(float dKii)
{	
	M2PID.Integral = dKii;	
}
/*------------------------------------------
 函数功能:设置M1PID微分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M1_SetKd(float dKdd)
{	
	M1PID.Derivative = dKdd;
}
/*------------------------------------------
 函数功能:设置M2微分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M2_SetKd(float dKdd)
{	
	M2PID.Derivative = dKdd;
}
/*------------------------------------------
 函数功能:电机1位置式PID计算
 函数说明:		
------------------------------------------*/
int32_t PID_M1_PosLocCalc(float NextPoint)
{
    register float  iError,dError;

	iError = M1PID.SetPoint - NextPoint;        // 偏差
	M1PID.SumError += iError;				    // 积分
	// if(M1PID.SumError > 5000)					//积分限幅2300
	// 	M1PID.SumError = 5000;
	// else if(M1PID.SumError < -5000)
	// 	M1PID.SumError = -5000;	
	dError = iError - M1PID.LastError; 			// 当前微分
	M1PID.LastError = iError;
	
	return(int32_t)(  M1PID.Proportion * iError           	// 比例项
          		    + M1PID.Integral   * M1PID.SumError 		// 积分项
          		    + M1PID.Derivative * dError);
}

/*------------------------------------------
 函数功能:电机1增量式PID计算
 函数说明:		
------------------------------------------*/
int32_t PID_M1_increase_Calc(float NextPoint)
{
    static float Bias1 = 0,PWM1 = 0,Last_bias1 = 0,Last_two_bias1 = 0;
	static float KP1 = 50,KI1 = 0.25,KD1 = 0;
	Bias1 = M1.SetX_angle - NextPoint ;
	PWM1 = KP1 * ((Bias1 - Last_bias1) + KI1 * Bias1 + KD1 * (Bias1 - 2 * Last_bias1 + Last_two_bias1));
	Last_two_bias1 = Last_bias1;
	Last_bias1 = Bias1;
	return (int32_t)PWM1 ;
}

/*------------------------------------------
 函数功能:电机2位置式PID计算
 函数说明:			
------------------------------------------*/
int32_t PID_M2_PosLocCalc(float NextPoint)
{
	register float  iError,dError;

	iError = M2PID.SetPoint - NextPoint;        // 偏差
	M2PID.SumError += iError;
	// if(M2PID.SumError > 2300.0)					//积分限幅
	// 	M2PID.SumError = 2300.0;
	// else if(M2PID.SumError < -2300.0)
	// 	M2PID.SumError = -2300.0;
	dError = iError - M2PID.LastError; 			// 当前微分
	M2PID.LastError = iError;
	
	return(int32_t)(  M2PID.Proportion * iError           	// 比例项
          		    + M2PID.Integral   * M2PID.SumError 		// 积分项
          		    + M2PID.Derivative * dError);
}

/*------------------------------------------
 函数功能:电机2增量式PID计算
 函数说明:		
------------------------------------------*/
int32_t PID_M2_increase_Calc(float NextPoint)
{
    static float Bias2 = 0,PWM2 = 0,Last_bias2 = 0,Last_two_bias2 = 0;
	static float KP2 = 50,KI2 = 0.25,KD2 = 0;
	Bias2 = M2.SetX_angle - NextPoint ;
	PWM2 = KP2 * ((Bias2 - Last_bias2) + KI2 * Bias2 + KD2 * (Bias2 - 2 * Last_bias2 + Last_two_bias2));
	Last_two_bias2 = Last_bias2;
	Last_bias2 = Bias2;
	return (int32_t)PWM2;
}

// /*------------------------------------------
//  函数功能:坐标值转换成角度
//  函数说明:		
// ------------------------------------------*/
// float X_2_degree(float x)
// {
// 	float angle;
// 	angle = atan(x / ALL_HEIGHT) * 180.0 / M_PI;
// 	return angle;
// }
// float degree_2_X(float degree)
// {
// 	float x;
// 	x = tan(degree * M_PI / 180.0) * ALL_HEIGHT;
// 	return x;
// }

/*--------------------------------------------------------------*/
float X_2_degree(float x)
/*--------------------------------------------------------------*
   TableCurve Function: F:\\��ѧ\\J-����\\2022����\\У����\\x2degree.c May 15, 2022 3:17:36 PM 
   x,angle 
   X= x 
   Y= angle 
   Eqn# 1  y=a+bx 
   r2=0.9994929390402112 
   r2adj=0.9993915268482534 
   StdErr=0.2900142436131396 
   Fstat=21682.64410263679 
   a= -0.2987 
   b= 0.6330962637362637 
 *--------------------------------------------------------------*/
{
  double y;
  y=-0.2987000000000000+x*0.6330962637362637;
  return(y);
}

// /*--------------------------------------------------------------*/
// float degree_2_X(float x)
// /*--------------------------------------------------------------*
//    TableCurve Function: F:\\��ѧ\\J-����\\2022����\\У����\\degree_2_X-line.c May 15, 2022 3:21:24 PM 
//    x,angle 
//    X= angle 
//    Y= x 
//    Eqn# 1  y=a+bx 
//    r2=0.9994929390402112 
//    r2adj=0.9993915268482534 
//    StdErr=0.4579725445362531 
//    Fstat=21682.64410263679 
//    a= 0.4715689508723446 
//    b= 1.578737699606109 
//  *--------------------------------------------------------------*/
// {
//   double y;
//   y=0.4715689508723446+x*1.578737699606109;
//   return(y);
// }

/*--------------------------------------------------------------*/
float degree_2_X(float x)
/*--------------------------------------------------------------*
   TableCurve Function: F:\\��ѧ\\J-����\\2022����\\У����\\degree_2_X.c May 15, 2022 3:20:49 PM 
   x,angle 
   X= angle 
   Y= x 
   Eqn# 8002  Exponential(a,b,c) 
   r2=0.9998953089129333 
   r2adj=0.999860411883911 
   StdErr=0.2182531663352514 
   Fstat=47754.55757162906 
   a= -408.0365264242307 
   b= 408.0792364671751 
   c= -258.0322929362569 
   Constraint: c<>0 
 *--------------------------------------------------------------*/
{
  double y;
  y=(-408.0365264242307)+408.0792364671751*exp(-x/(-
    258.0322929362569));
  return(y);
}
 
