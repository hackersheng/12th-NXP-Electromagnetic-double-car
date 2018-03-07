/*
* @file			user_pit.c
* @brief		时间处理
* @author		Benson
* @date			20161229
*/

#include "include.h"

int8 RING_WAIT_FLAG=0;	//状态标志位 等待被超车
int8 RING_STOP_FLAG=0;	//环道停车标志位
extern uint8 RING_LOCK_FLAG;
extern uint8 LOCK_Cnt;
extern uint16 Hori_ADMiddle_max,H_middle;
extern int8 AD_MAX_START_FLAG;
extern int8 START_FLAG;
extern int8 Road_Type;
extern uint16 Distance[3];
extern double UartData[9];
/*参数*/
uint8 Timer_Ring = 0;
uint8 Timer_Ring_Stop=0;
uint8 Timer_6ms = 0;	//6ms计时器
uint8 Timer_4ms = 0;	//4ms计时器
uint8 Timer_500ms = 0;	//4ms计时器
uint16 Timer_AD_MAX=0;
uint8 Timer_start=0;
int8 Startline_Flag=0;
uint8 lock_time=85;
extern int8 front_or_after;
extern int8 last_front_or_after;
extern uint8 Dis_Flag;
extern int8 Emergency_Stop;
extern int16 Speed[3];
extern int16 Spd_Err[3];
extern int16 Dir_Err[4];
extern int16 Set_Speed;
extern int16 Motor_Duty;
extern int16 Spd_Kp;
extern int16 Spd_Ki;
extern int16 Spd_Kd;
extern int8 Launch_Flag;
extern int8 Stop_Flag;
extern int16 Set_Speed_Temp;
extern int16 Motor_Duty;
extern int16 Steer_Duty[4];
extern int16 Set_Speed_Temp;
TIME sys_time;
TIME time_start;
TIME time_ring_launch;
extern TIME time_launch;

/*
*  @brief		PIT中断服务函数
*  @author		Benson
*  @date		20161229
*/
void PIT0_IRQHandler()	//2ms
{
	Timer_4ms++;
	Timer_6ms++;
	
	//起跑线
	Startline_Flag=get_startline();	//0触发
//	gpio_set(PTC0, Startline_Flag);
	if(get_timevar_ms(sys_time, time_launch)>10000 && Startline_Flag==0)	//停车线
    {
        gpio_set(PTC0, Startline_Flag);
        //Stop_Flag=1;
    }
	
	if(Timer_6ms>=3)
	{
		Timer_6ms=0;
		/*速度调整周期为6ms*/
		get_speed();
		speed_adjust();
	}
	if(Timer_4ms>=2)
	{
		/*方向调整周期为4ms*/
		//采集电感电压
		Timer_4ms=0;
		get_voltage();
		road_analyze();
		err_analyze();		//偏差分析
		if(FUZZY_FLAG)						//转向模糊
			fuzzy_steer_control();
		speed_control();	//速度控制
		direction_adjust();
	}
  
  	get_current();
	current_adjust();
	PIT_Flag_Clear(PIT0);
}

void PIT1_IRQHandler()	//2ms
{
  //系统时钟
	sys_time.ms+=2;
	if(sys_time.ms>=1000)
	{
		sys_time.ms=0;
		sys_time.ss++;
	}
  
//  if(AD_MAX_START_FLAG)	//如果按键开始标定最大AD值，设定时间10s
//  {
//    Timer_AD_MAX++;
//    if(Timer_AD_MAX==1)
//    {
//      Hori_ADMiddle_max=0;
//    }
//    else
//    {
//      if(Hori_ADMiddle_max<H_middle)
//        Hori_ADMiddle_max=H_middle;
//    }
//    if(Timer_AD_MAX==2000)
//    { 
//      AD_MAX_START_FLAG=0;//触发采集最大电感程序的标志位
//      Timer_AD_MAX=0;     //这个是时间计数标志位
//    }
//  }
  //上面这段程序有一个标志位，一个计数位，一个输出变量。
  
  if(RING_LOCK_FLAG!=0)	//锁死开始计时
  {
    gpio_set(PTD4,1);
    Timer_Ring++;
    if(Timer_Ring>=lock_time)	//锁死时间到
    {
      LOCK_Cnt=1;
      Timer_Ring=0;
      RING_LOCK_FLAG=0;
      gpio_set(PTD4,0);
      //前车进环道停车
      //1车触发环道标志位需要停车，触发环道停车标志位
      if(last_front_or_after==1)
      {
        RING_STOP_FLAG=1;//环道停车标志位
      }
    }
    
  }
  //环道停车标志位触发之后开始计数，定时时间
	if(RING_STOP_FLAG)
	{
		Timer_Ring_Stop++;
		if(Timer_Ring_Stop==200)	//定时停车
		{
			//time_ring_launch=sys_time;
			RING_STOP_FLAG=0;     
			Timer_Ring_Stop=0;
			if (front_or_after == 0 && last_front_or_after == 1)
				RING_WAIT_FLAG = 1;//环道等待标志位
			else
			{
				RING_WAIT_FLAG = 0;
			}
		}
	}
  //环道停车标志位与环道等待标志位只有1车进入环道才会触发
	PIT_Flag_Clear(PIT1);
}

void PIT3_IRQHandler()
{
	get_current();
	current_adjust();	
	PIT_Flag_Clear(PIT3);
}



/*
*  @brief		PIT中断初始化
*  @author		Benson
*  @date		20161229
*/

void PIT_Init()
{
	/*PIT0用作控制周期基数*/
	pit_init_ms(PIT0,2);
	set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler);
	enable_irq(PIT0_IRQn);
  
	/*PIT1用作控制LED和蜂鸣器*/
	pit_init_ms(PIT1,2);
	set_vector_handler(PIT1_VECTORn, PIT1_IRQHandler);
	enable_irq(PIT1_IRQn);
  
//  pit_init_us(PIT3,200);
//  set_vector_handler(PIT3_VECTORn, PIT3_IRQHandler);
//  enable_irq(PIT3_IRQn);
  
	//系统时间初始化
	sys_time.ms=0;
	sys_time.ss=0;
}

TIME get_timevar(TIME time1,TIME time2)
{
	TIME timesoon,timelate,timevar;
	//变量timesoon与timelate都是无关于输入变量
	//判断大小
	if(time1.ss>time2.ss)
	{
	timesoon=time2;
	timelate=time1;
	}
	else if(time1.ss<time2.ss)
	{
	timesoon=time1;
	timelate=time2;
	}
	else	//秒位一样大
	{
		if(time1.ms>time2.ms)
		{
			timesoon=time2;
			timelate=time1;
		}
		else if(time1.ms<time2.ms)
		{
			timesoon=time1;
			timelate=time2;
		}
		else
		{
			timevar.ss=0;
			timevar.ms=0;
		}
	}
  
  //计算时间
	if(timesoon.ss==timelate.ss)
	{
		timevar.ss=0;
		timevar.ms=timelate.ms-timesoon.ms;
	}
	else
	{
		if(timesoon.ms<=timelate.ms)
		{
			timevar.ms=timelate.ms-timesoon.ms;
			timevar.ss=timelate.ss-timesoon.ss;
		}
		else
		{
			timevar.ss=timelate.ss-timesoon.ss-1;
			timevar.ms=1000+timelate.ms-timesoon.ms;
		}
	}
	return timevar;
}


uint16 get_timevar_ms(TIME time1,TIME time2)
{
  TIME timesoon,timelate,timevar;
  //判断大小
  if(time1.ss>time2.ss)
  {
	timesoon=time2;
	timelate=time1;
  }
  else if(time1.ss<time2.ss)
  {
	timesoon=time1;
	timelate=time2;
  }
  else	//秒位一样大
  {
	if(time1.ms>time2.ms)
	{
	  timesoon=time2;
	  timelate=time1;
	}
	else if(time1.ms<time2.ms)
	{
	  timesoon=time1;
	  timelate=time2;
	}
	else
	{
	  timevar.ss=0;
	  timevar.ms=0;
	}
  }
  
  //计算时间
  if(timesoon.ss==timelate.ss)
  {
	timevar.ss=0;
	timevar.ms=timelate.ms-timesoon.ms;
  }
  else
  {
	if(timesoon.ms<=timelate.ms)
	{
	  timevar.ms=timelate.ms-timesoon.ms;
	  timevar.ss=timelate.ss-timesoon.ss;
	}
	else
	{
	  timevar.ss=timelate.ss-timesoon.ss-1;
	  timevar.ms=1000+timelate.ms-timesoon.ms;
	}
  }
  return (timevar.ss*1000+timevar.ms);
}