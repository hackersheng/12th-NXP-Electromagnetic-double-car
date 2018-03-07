#include "include.h"

#define Ring1_flag (gpio_get(PTE0))
#define Ring2_flag (gpio_get(PTE1))
#define Ring3_flag (gpio_get(PTE2))
#define Ring4_flag (gpio_get(PTE3))

uint8 RING_LOCK_FLAG=0;	//�뻷��ʱ����������ı�־λ
uint8 LOCK_Cnt=0;		//ͬһ�������й涨���������������������ظ�����
uint8 RING_FLAG=0;		//��⵽����
uint8 Ring_Cnt = 0;
int16 ring_angle[4];
int8 big_ring_pos=1;
int16 ring_td=200;
int16 ring_th=600;
int16 ring_tl=500;
int16 ring_adh=250;
int16 ring_adl=50;


uint8 Overtake_ctrl_table[]={0,0,0,0};

extern double UartData[9];

void switch_setting()
{
	if (Ring1_flag)
	{
		ring_angle[0] = -700;	//��ת
	}
	else
	{
		ring_angle[0] = 700;	//��ת
	}
	if (Ring2_flag)
	{
		ring_angle[1] = -700;	//��ת
	}  
	else
	{
		ring_angle[1] = 700;	//��ת
	}
	if (Ring3_flag)
	{
		ring_angle[2] = -700;	//��ת
	}  
	else
	{
		ring_angle[2] = 700;	//��ת
	}
	if (Ring4_flag)
	{
		ring_angle[3] = -700;	//��ת
	}  
	else
	{
		ring_angle[3] = 700;	//��ת
	}
	Overtake_ctrl_table[0]=gpio_get(PTD0);
	Overtake_ctrl_table[1]=gpio_get(PTD1);
	Overtake_ctrl_table[2]=gpio_get(PTD2);
	Overtake_ctrl_table[3]=gpio_get(PTD3);
}