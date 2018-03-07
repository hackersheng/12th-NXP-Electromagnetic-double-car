#include "include.h"

extern uint8 Receive_Data1[6];
extern int16 Dir_Err[4];
extern int16 Set_Speed;
extern int8 front_or_after;
extern int8 Ramp_Flag;
extern int8 RING_FLAG;
uint8 Dis_Coeff=100;	//一定要初始化为100！
uint16 Distance[3];
int16 Set_Distance=1000;


extern TIME sys_time;
extern TIME time_launch;

extern FUZTAB DisFuzzyTAB;

void distance_speed_fuzzy()
{
    int16 err,dis;
    int16 coeff1,coeff2,coeff3;
    err=abs(Dir_Err[3]);
    dis=Distance[2]-Set_Distance;
    
    fuzzyout(&DisFuzzyTAB, err,dis, &coeff1,&coeff2,&coeff3);
    if(front_or_after==1)
        Dis_Coeff=coeff1/100;
    else if(front_or_after==2)
        Dis_Coeff=coeff2/100;
    else
        Dis_Coeff=100;
}

void sonic_init()
{
	gpio_init(PTA15,GPO,1);		//发送S0
	gpio_set(PTA15,1);
	gpio_init(PTA14,GPI,1);		//发送S1
	//port_init(PTA14, PF | ALT1 );
	
	gpio_init(PTA16,GPI,0);
	port_init(PTA16, IRQ_EITHER | PF | ALT1 | PULLUP);	//超声波 接收D0
	set_vector_handler(PORTA_VECTORn,sonic_handler);
	
    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                          //使能PIT时钟

    PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );    //使能PIT定时器时钟 ，调试模式下继续运行

    PIT_TCTRL(PIT2) &= ~( PIT_TCTRL_TEN_MASK );                     //禁用PIT ，以便设置加载值生效

    PIT_LDVAL(PIT2)  = ~0;                                          //设置溢出中断时间

    PIT_Flag_Clear(PIT2); 
}

void sonic_handler()
{
	uint16 var;
	if(gpio_get(PTA16)==1)	//上升沿触发
	{
		pit_time_start(PIT2);
	}
	else	//下降沿触发
	{
          
                Distance[0]=Distance[1];
                Distance[1]=Distance[2];
                
		var=pit_time_get_us(PIT2);
		var=var*340/1000;
		
		if(front_or_after==2)
		{
			if(var<4500 && var>50)
			{
//				if(abs(var-Distance[2])<220 || sys_time.ss<12 || RING_FLAG)
//					Distance[2]=var;
                              Distance[2]=var*60/100+Distance[1]*30/100+Distance[0]*10/100;
			}
		}
		else if(front_or_after==1)
			Distance[2]=Receive_Data1[0]*100+Receive_Data1[2];
		else
			;
		pit_close(PIT2);
		
	}
	PORTA_ISFR = ~0;
}

void distance_speed()
{
	if(front_or_after!=0)
		//注意这里减法是反的
		//Dis_Coeff=distance_fuzzy(Distance[2]-Set_Distance,front_or_after);
        distance_speed_fuzzy();
	else
		Dis_Coeff=100;
    
    if(Ramp_Flag)
        Dis_Coeff=100;
    
    push(8,Distance[2]);
    /*
	if(Distance<400 && front_or_after==2)	//后车距离太近
		Dis_Coeff=0;
		
	if(Distance>1800 && front_or_after==1)
		Dis_Coeff=0;
    */
}