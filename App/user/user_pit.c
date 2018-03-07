/*
* @file			user_pit.c
* @brief		ʱ�䴦��
* @author		Benson
* @date			20161229
*/

#include "include.h"

int8 RING_WAIT_FLAG=0;	//״̬��־λ �ȴ�������
int8 RING_STOP_FLAG=0;	//����ͣ����־λ
extern uint8 RING_LOCK_FLAG;
extern uint8 LOCK_Cnt;
extern uint16 Hori_ADMiddle_max,H_middle;
extern int8 AD_MAX_START_FLAG;
extern int8 START_FLAG;
extern int8 Road_Type;
extern uint16 Distance[3];
extern double UartData[9];
/*����*/
uint8 Timer_Ring = 0;
uint8 Timer_Ring_Stop=0;
uint8 Timer_6ms = 0;	//6ms��ʱ��
uint8 Timer_4ms = 0;	//4ms��ʱ��
uint8 Timer_500ms = 0;	//4ms��ʱ��
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
*  @brief		PIT�жϷ�����
*  @author		Benson
*  @date		20161229
*/
void PIT0_IRQHandler()	//2ms
{
	Timer_4ms++;
	Timer_6ms++;
	
	//������
	Startline_Flag=get_startline();	//0����
//	gpio_set(PTC0, Startline_Flag);
	if(get_timevar_ms(sys_time, time_launch)>10000 && Startline_Flag==0)	//ͣ����
    {
        gpio_set(PTC0, Startline_Flag);
        //Stop_Flag=1;
    }
	
	if(Timer_6ms>=3)
	{
		Timer_6ms=0;
		/*�ٶȵ�������Ϊ6ms*/
		get_speed();
		speed_adjust();
	}
	if(Timer_4ms>=2)
	{
		/*�����������Ϊ4ms*/
		//�ɼ���е�ѹ
		Timer_4ms=0;
		get_voltage();
		road_analyze();
		err_analyze();		//ƫ�����
		if(FUZZY_FLAG)						//ת��ģ��
			fuzzy_steer_control();
		speed_control();	//�ٶȿ���
		direction_adjust();
	}
  
  	get_current();
	current_adjust();
	PIT_Flag_Clear(PIT0);
}

void PIT1_IRQHandler()	//2ms
{
  //ϵͳʱ��
	sys_time.ms+=2;
	if(sys_time.ms>=1000)
	{
		sys_time.ms=0;
		sys_time.ss++;
	}
  
//  if(AD_MAX_START_FLAG)	//���������ʼ�궨���ADֵ���趨ʱ��10s
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
//      AD_MAX_START_FLAG=0;//�����ɼ�����г���ı�־λ
//      Timer_AD_MAX=0;     //�����ʱ�������־λ
//    }
//  }
  //������γ�����һ����־λ��һ������λ��һ�����������
  
  if(RING_LOCK_FLAG!=0)	//������ʼ��ʱ
  {
    gpio_set(PTD4,1);
    Timer_Ring++;
    if(Timer_Ring>=lock_time)	//����ʱ�䵽
    {
      LOCK_Cnt=1;
      Timer_Ring=0;
      RING_LOCK_FLAG=0;
      gpio_set(PTD4,0);
      //ǰ��������ͣ��
      //1������������־λ��Ҫͣ������������ͣ����־λ
      if(last_front_or_after==1)
      {
        RING_STOP_FLAG=1;//����ͣ����־λ
      }
    }
    
  }
  //����ͣ����־λ����֮��ʼ��������ʱʱ��
	if(RING_STOP_FLAG)
	{
		Timer_Ring_Stop++;
		if(Timer_Ring_Stop==200)	//��ʱͣ��
		{
			//time_ring_launch=sys_time;
			RING_STOP_FLAG=0;     
			Timer_Ring_Stop=0;
			if (front_or_after == 0 && last_front_or_after == 1)
				RING_WAIT_FLAG = 1;//�����ȴ���־λ
			else
			{
				RING_WAIT_FLAG = 0;
			}
		}
	}
  //����ͣ����־λ�뻷���ȴ���־λֻ��1�����뻷���Żᴥ��
	PIT_Flag_Clear(PIT1);
}

void PIT3_IRQHandler()
{
	get_current();
	current_adjust();	
	PIT_Flag_Clear(PIT3);
}



/*
*  @brief		PIT�жϳ�ʼ��
*  @author		Benson
*  @date		20161229
*/

void PIT_Init()
{
	/*PIT0�����������ڻ���*/
	pit_init_ms(PIT0,2);
	set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler);
	enable_irq(PIT0_IRQn);
  
	/*PIT1��������LED�ͷ�����*/
	pit_init_ms(PIT1,2);
	set_vector_handler(PIT1_VECTORn, PIT1_IRQHandler);
	enable_irq(PIT1_IRQn);
  
//  pit_init_us(PIT3,200);
//  set_vector_handler(PIT3_VECTORn, PIT3_IRQHandler);
//  enable_irq(PIT3_IRQn);
  
	//ϵͳʱ���ʼ��
	sys_time.ms=0;
	sys_time.ss=0;
}

TIME get_timevar(TIME time1,TIME time2)
{
	TIME timesoon,timelate,timevar;
	//����timesoon��timelate�����޹����������
	//�жϴ�С
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
	else	//��λһ����
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
  
  //����ʱ��
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
  //�жϴ�С
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
  else	//��λһ����
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
  
  //����ʱ��
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