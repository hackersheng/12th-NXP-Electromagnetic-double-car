/*
* @file       user_sys.c
* @brief      系统有关函数
* @author     Benson
*/

#include "include.h"

/*
*  @brief		系统初始化
*  @author		Benson
*  @date		20161229
*/

void NVIC_Init()
{
	 NVIC_SetPriorityGrouping(4);
	 NVIC_SetPriority(PORTA_IRQn, 1);	//超声波优先级第二高
	 NVIC_EnableIRQ(PORTA_IRQn);
	 
	 NVIC_SetPriority(UART5_RX_TX_IRQn, 2);	//双车串口
	 NVIC_EnableIRQ(UART5_RX_TX_IRQn);
	 NVIC_SetPriority(UART4_RX_TX_IRQn, 0);	//上位机串口优先级最高
	 NVIC_EnableIRQ(UART4_RX_TX_IRQn);
	 NVIC_SetPriority(PIT0_IRQn, 3);		//PIT0中断
	 NVIC_EnableIRQ(PIT0_IRQn);
	 NVIC_SetPriority(PIT1_IRQn, 4);		//PIT1中断
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
	user_key_init();//拨码开关
	beep_init();//蜂鸣器初始化设置
	sonic_init();//超声波初始化（pit2定时器）
	NVIC_Init();//确定各中断的优先级
	PIT_Init();//pit初始化（pit0、pit1定时器）
	FTM_Init();
	AD_Init();
	gpio_init(PTD4,GPO,0);
}

void user_key_init()	//拨码开关
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