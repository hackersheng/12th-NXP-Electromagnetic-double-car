#include "include.h"

int16 AD_Current[3];
int16 Current_Err[3];

int16 Set_Current=0;
//电流的目标值
int16 Current_Zero=2079;
//2079是pwm为0时，电流的采集值,偏差+-5以内

int16 Cur_Kp=80;
int16 Cur_Ki=2200;	//1000倍


extern int16 Set_Speed;
extern int16 Speed[3];
extern int16 Spd_Err[3];
extern int8 stall_flag;
extern int8 Emergency_Stop;
extern int8 Launch_Flag;
extern TIME sys_time;
extern TIME time_launch;
extern TIME time_stall_start;
extern int32 Motor_Duty;		  
extern int8 Road_Type;

void current_init()
{
	adc_init(ADC1_SE14);
}

void get_current()
{
	int8 i=0;
	int16 temp;
	int16 sum;
	int16 ad_current[10];
	
	for(i=0;i<10;i++)
		ad_current[i] = (uint16)adc_once(ADC1_SE14, ADC_12bit);
	
	for(i=0;i<9;i++)
	{
		if(ad_current[i]<ad_current[i+1])
		{
			temp=ad_current[i];
			ad_current[i]=ad_current[i+1];
			ad_current[i+1]=temp;
		}
	}
	ad_current[9]=0;
	for(i=0;i<9;i++)
	{
		if(ad_current[i]>ad_current[i+1])
		{
			temp=ad_current[i];
			ad_current[i]=ad_current[i+1];
			ad_current[i+1]=temp;
		}
	}
	ad_current[9]=0;
	
	for(i=0;i<10;i++)
	{
		sum+=ad_current[i];
	}
	
	AD_Current[0]=AD_Current[1];
	AD_Current[1]=AD_Current[2];
	AD_Current[2]=Current_Zero-sum/8;
	Current_Err[0]=Current_Err[1];
	Current_Err[1]=Current_Err[2];
	Current_Err[2]=Set_Current-AD_Current[2];
	
	//push(8,AD_Current[2]);
}


/*
	初始化 电流校准
*/
void current_regulate()
{
	int8 i=0;
	int16 temp;
	int16 sum;
	int16 ad_current[10];
	
	for(i=0;i<10;i++)
		ad_current[i] = (uint16)adc_once(ADC1_SE14, ADC_12bit);
	
	for(i=0;i<9;i++)
	{
		if(ad_current[i]<ad_current[i+1])
		{
			temp=ad_current[i];
			ad_current[i]=ad_current[i+1];
			ad_current[i+1]=temp;
		}
	}
	ad_current[9]=0;
	for(i=0;i<9;i++)
	{
		if(ad_current[i]>ad_current[i+1])
		{
			temp=ad_current[i];
			ad_current[i]=ad_current[i+1];
			ad_current[i+1]=temp;
		}
	}
	ad_current[9]=0;
	
	for(i=0;i<10;i++)
	{
		sum+=ad_current[i];
	}
	Current_Zero=sum/8;
}


void current_adjust()
{
	int16 Cur_Result_I;
	int16 Cur_Result_P;
	
	Cur_Result_I=Cur_Ki*Current_Err[2]/1000;
	Cur_Result_P=Cur_Kp*(Current_Err[2]-Current_Err[1])/1000;
	
//	Cur_Result_I=Cur_Ki*Current_Err[2]/1000;
//	if(Current_Err[2]>0)
//		Cur_Result_P=Cur_Kp*(Current_Err[2]*Current_Err[2]/1000)/1000;
//	else
//		Cur_Result_P=0-Cur_Kp*(Current_Err[2]*Current_Err[2]/1000)/1000;
	
	Motor_Duty+=(Cur_Result_P+Cur_Result_I);
	
	//保护
	
	if(Set_Current==0&&Current_Err[2]==0)
 	{
		Motor_Duty=0;
	}
	
	if(Motor_Duty>9000)
		Motor_Duty=9000;
	if(Motor_Duty<0-9000)
		Motor_Duty=0-9000;
	
	//堵转保护
	//if(stall_flag==0 && (sys_time.ss-time_launch.ss)>2 && Set_Speed>30 && Speed[2]<4 && Road_Type!=5)	//进入环岛会急减速
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
		Motor_Duty=0;
//	if(Launch_Flag==0)
//		Motor_Duty=0;
	
	//Motor_Duty=2000;
	motor(Motor_Duty);
}
