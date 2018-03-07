/*
* @file			user_pid.c
* @brief		pid调节与控制
* @author		Benson
* @date			20161229
*/

#include "include.h"

extern uint8 Dis_Coeff;
extern int16 Speed[3];
extern int32 Motor_Duty;
extern int8 Road_Type,Last_Road_Type;
extern int8 Out_Flag;
extern int8 Emergency_Stop;
int16 Set_Speed;
int16 Spd_Err[3];
extern int8 Ramp_Flag;
extern int32 Steer_Duty[4];
extern int16 Dir_Err[4];
extern int32 steer_limit;
TIME time_stall_start;
extern TIME time_launch;
extern TIME sys_time;
int8 stall_flag=0;

/*PID初始值*/
int16 Spd_Kp=700;
int16 Spd_Ki=15000;
int16 Spd_Kd=500;
int16 Dir_Kp;
int16 Dir_Kd;
int32 Spd_Result_P,Spd_Result_I,Spd_Result_D;
extern int16 Cur_Kp,Cur_Ki;
extern double UartData[9];

extern int16 Set_Current;

extern FUZTAB SpeedCtrlTAB;

/*
*  @brief		接收来自上位机的参数
*  @author		Benson
*  @date		20161229
*/

void set_pid()
{
	Spd_Kp=(int16)(UartData[0]*1000);
	Spd_Ki=(int16)(UartData[1]*1000);
//	Spd_Kd=(int16)(UartData[2]*1000);
    Cur_Kp=(int16)(UartData[2]*1000);
    Cur_Ki=(int16)(UartData[3]*1000);
	/*
	if(!FUZZY_FLAG)
	{
		Dir_Kp=(int16)(UartData[3]*1000);
		Dir_Kd=(int16)(UartData[4]*1000);
	}
	*/
}


void fuzzy_speed_control()
{
	int16 error;
	int16 errorC;
	int16 kp=0,ki=0,kd=0;
	error=Spd_Err[2];
	errorC=Spd_Err[2]-Spd_Err[1];
	fuzzyout(&SpeedCtrlTAB, error, errorC, &kp, &ki, &kd);
	//fuzzyout(&SpeedCtrlTAB, error, errorC, &Spd_Kp, &Spd_Ki, &Spd_Kd);
	Spd_Kp=kp/10;
	Spd_Ki=ki/10;
}




/*
*  @brief		速度调节
*  @author		Benson
*  @date		20161229
*/
void speed_adjust()
{
	int16 e;
	Spd_Err[0]=Spd_Err[1];
	Spd_Err[1]=Spd_Err[2];
#if RANK_FLAG==1	//如果启用距离控制，就乘个系数
		Spd_Err[2]=Set_Speed*Dis_Coeff/100-Speed[2];
#else				//不启用距离控制，正常跑
		Spd_Err[2]=Set_Speed-Speed[2];
#endif
		
//	fuzzy_speed_control();
		
	Spd_Result_P=(int32)(Spd_Kp*(Spd_Err[2]-Spd_Err[1]))/1000;
	//push(5,Spd_Result_P);
	Spd_Result_I=(int32)(Spd_Ki*Spd_Err[2])/1000;
	//push(6,Spd_Result_I);
	
//	Spd_Result_P=(int32)(Spd_Kp*Spd_Err[2]/1000);
//	push(5,Spd_Result_P);
//	Spd_Result_I+=(int32)(Spd_Ki*Spd_Err[2])/1000;
////	if(Spd_Result_I>1200)	//积分限幅
////		Spd_Result_I=1200;
////	if(Spd_Result_I<-1200)
////		Spd_Result_I=-1200;
//	push(6,Spd_Result_I);
	
	//防止电机跑飞
//	if((Set_Speed==0)&&(Speed[2]<3)&&(Motor_Duty<0))
//	{
//		Motor_Duty=0;
//	}
	
//	if(abs(Spd_Err[2])>60)	棒棒控制
//	{
//		if(Spd_Err[2]>0)
//			Set_Current=1400+Spd_Result_P;
//		else
//			Set_Current=-700+Spd_Result_P;
//	}
//	else
	Set_Current += (Spd_Result_P+Spd_Result_I);
        
        //Set_Current=-50;
	//push(7,Set_Current);
	//Set_Current=100;
	//push(5,Set_Current);
	
	if(Set_Current>1900)
		Set_Current=1900;
	if(Set_Current<-1900)
		Set_Current=-1900;
		
	
	
//	if(Set_Speed==0&&Spd_Err[2]==0)
//	{
//		Set_Current=0;
//	}
//	
	//push(3,Set_Current);
//	
//	
//	if((Set_Speed<10)&&(Set_Speed!=0))
//	{
//		if(Motor_Duty>2400)
//			Motor_Duty=2400;
//		if(Motor_Duty<0-2400)
//			Motor_Duty=0-2400;
//	}
//	else
//	{
//		if(Motor_Duty>9000)
//			Motor_Duty=9000;
//		if(Motor_Duty<0-9000)
//			Motor_Duty=0-9000;
//	}
//	//限幅
//	
	//堵转保护
    
	//if(stall_flag==0 && (sys_time.ss-time_launch.ss)>2 && Set_Speed*Dis_Coeff/100>10 && Speed[2]<5 && Ramp_Flag==0)
	if(0)
    {
		time_stall_start=sys_time;
		stall_flag=1;
		Motor_Duty=0;
	}
	if(stall_flag==1)
	{
		beep_on();
		Motor_Duty=0;
	}
	
	if(Emergency_Stop==1)
		Set_Current=0;
//	Motor_Duty=3000;
//	motor(Motor_Duty);
	
}


/*
*  @brief		方向调节
*  @author		Benson
*  @date		20161229
*/
void direction_adjust()
{
	int32 Dir_Result_P,Dir_Result_D;

	Dir_Result_P=(int32)(Dir_Kp*Dir_Err[3]/1000);
	Dir_Result_D=(int32)(Dir_Kd*(0.6*(Dir_Err[3]-Dir_Err[2])+0.2*(Dir_Err[2]-Dir_Err[1])+0.2*(Dir_Err[1]-Dir_Err[0]))/1000);
	
//	if(Last_Road_Type==0 && Road_Type!=0)
//		Dir_Result_D=0;
	if(Road_Type!=3 && Last_Road_Type==3)
		Dir_Result_D=0;
	if(Road_Type!=-3 && Last_Road_Type==-3)
		Dir_Result_D=0;
	if(Road_Type==4 && Last_Road_Type==5)
		Dir_Result_D=0;
	Steer_Duty[0]=Steer_Duty[1];
	Steer_Duty[1]=Steer_Duty[2];
	Steer_Duty[2]=Steer_Duty[3];
	Steer_Duty[3]=(Dir_Result_P+Dir_Result_D);//PD调节方向
	
	/*
	if(Out_Flag==1)
	{
		Out_Flag=0;
		Steer_Duty[3]=Steer_Duty[2];
	}
	*/
	
	if(Steer_Duty[3]>steer_limit)
		Steer_Duty[3]=steer_limit;
	if(Steer_Duty[3]<0-steer_limit)
		Steer_Duty[3]=0-steer_limit;
	steer(Steer_Duty[3]);
        push(2,Steer_Duty[3]);
}