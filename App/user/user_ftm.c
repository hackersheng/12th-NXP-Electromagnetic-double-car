/*
* @file			user_ftm.c
* @brief		ftm初始化
* @author		Benson
* @date			20161229
*/

#include "include.h"

#if CAR_NUM==1

//#define steer_center	1723	//舵机中间
#define steer_center	1590	//舵机中间值
//#define steer_center	1752	//舵机中间值
#define steer_l_limit	(int32)(steer_center+steer_limit)	//舵机左极限275
#define steer_r_limit	(int32)(steer_center-steer_limit)	//舵机右极限265

#elif CAR_NUM==2

#define steer_center	1230	//舵机中间值
#define steer_l_limit	(int32)(steer_center+steer_limit)	//舵机左极限 250
#define steer_r_limit	(int32)(steer_center-steer_limit)	//舵机右极限 243

#elif CAR_NUM==3

#define steer_center	1679	//舵机中间值

#else

#endif

int32 steer_limit=250;
extern double UartData[9];
extern int8 Road_Type;



int16 Speed[3];
int32 Motor_Duty=3500;
int32 Steer_Duty[4];

/*
*电机	ftm0 ch0 ch1	PTC1 PTC2
*舵机	ftm2 ch1		PTB19
*编码器	ftm1 ch0 ch1	PTA12 PTA13
*/


/*
*  @brief		ftm初始化
*  @author		Benson
*  @date		20161229
*/

void FTM_Init()
{
	/*电机*/
	ftm_pwm_init(FTM0,FTM_CH0,10*1000,0);
	ftm_pwm_init(FTM0,FTM_CH1,10*1000,0);
	/*舵机*/
	ftm_pwm_init(FTM2,FTM_CH1,250,steer_center);
	/*编码器*/
	ftm_quad_init(FTM1);
}

/*
*  @brief		电机控制
*  @author		Benson
*  @date		20161229
*/

void motor(int32 motor_duty)
{
	if(motor_duty<0)
	{
		ftm_pwm_duty(FTM0,FTM_CH0,0-motor_duty);
		ftm_pwm_duty(FTM0,FTM_CH1,0);
	}
	else if(motor_duty>0)
	{
		ftm_pwm_duty(FTM0,FTM_CH1,motor_duty);
		ftm_pwm_duty(FTM0,FTM_CH0,0);
	}
	else
	{
		ftm_pwm_duty(FTM0,FTM_CH1,0);
		ftm_pwm_duty(FTM0,FTM_CH0,0);
	}
}
/*
*  @brief		编码器采集
*  @author		Benson
*  @date		20161229
*/

void get_speed()
{
	int16 val;
    val =ftm_quad_get(FTM1);

	ftm_quad_clean(FTM1);
	Speed[0]=Speed[1];
	Speed[1]=Speed[2];
	Speed[2]=0-val;
	push(4,0-val);
}

/*
*  @brief		舵机控制
*  @author		Benson
*  @date		20161229
*/

void steer(int32 steer_duty)
{
	if(Road_Type!=5 && Road_Type!=4)
	{
		if(steer_duty+steer_center>steer_l_limit)
			steer_duty=steer_l_limit-steer_center;
		if(steer_duty+steer_center<steer_r_limit)
			steer_duty=steer_r_limit-steer_center;
	}
	else
	{
		if(steer_duty>600)
			steer_duty=600;
		if(steer_duty<-600)
			steer_duty=-600;
	}
		
	ftm_pwm_duty(FTM2,FTM_CH1,(steer_center+steer_duty)*5/2);
}
