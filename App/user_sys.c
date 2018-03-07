/*
* @file       user_sys.c
* @brief      ϵͳ�йغ���
* @author     Benson
*/

#include "include.h"

/*
*  @brief		ϵͳ��ʼ��
*  @author		Benson
*  @date		20161229
*/

void NVIC_Init()
{
	 NVIC_SetPriorityGrouping(4);
	 NVIC_SetPriority(PORTA_IRQn, 1);	//���������ȼ��ڶ���
	 NVIC_EnableIRQ(PORTA_IRQn);
	 
	 NVIC_SetPriority(UART5_RX_TX_IRQn, 2);	//˫������
	 NVIC_EnableIRQ(UART5_RX_TX_IRQn);
	 NVIC_SetPriority(UART4_RX_TX_IRQn, 0);	//��λ���������ȼ����
	 NVIC_EnableIRQ(UART4_RX_TX_IRQn);
	 NVIC_SetPriority(PIT0_IRQn, 3);		//PIT0�ж�
	 NVIC_EnableIRQ(PIT0_IRQn);
	 NVIC_SetPriority(PIT1_IRQn, 4);		//PIT1�ж�
	 NVIC_EnableIRQ(PIT1_IRQn);
}
void beep_init()
{
	gpio_init(PTD4,GPO,0);
	gpio_init(PTD15,GPO,1);
}

void beep_on()
{
	gpio_set(PTD4,1);
}

void beep_off()
{
	gpio_set(PTD4,0);
}
void SYS_Init()
{
	user_key_init();//���뿪��
	beep_init();//��������ʼ������
	sonic_init();//��������ʼ����pit2��ʱ����
	NVIC_Init();//ȷ�����жϵ����ȼ�
	PIT_Init();//pit��ʼ����pit0��pit1��ʱ����
	FTM_Init();
	AD_Init();
	gpio_init(PTD4,GPO,0);
}

void user_key_init()	//���뿪��
{
	gpio_init(PTE0, GPI, 1);
	gpio_init(PTE1, GPI, 1);
	gpio_init(PTE2, GPI, 1);
	gpio_init(PTE3, GPI, 1);
}

//led
void buzzer(uint32 n)
{
	gpio_set(PTD15,1);
	systick_delay_ms(n);
	gpio_set(PTD15,0);
}