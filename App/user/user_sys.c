/*
* @file       user_sys.c
* @brief      系统有关函数
* @author     Benson
*/

#include "include.h"
double last_data[9];
extern double UartData[9];
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
	 NVIC_SetPriority(PIT3_IRQn, 4);		//PIT1中断
	 NVIC_EnableIRQ(PIT3_IRQn);
}
void beep_init()//蜂鸣器初始化
{
	gpio_init(PTD4,GPO,0);
	gpio_init(PTD15,GPO,1);
	//gpio_init(PTE26,GPO,1);
}

void beep_on()//蜂鸣器打开
{
	gpio_set(PTD4,1);
	//gpio_set(PTE26,0);
}

void beep_off()//蜂鸣器关闭
{
	gpio_set(PTD4,0);
}
void SYS_Init()
{
	user_press_init();//按键初始化
	user_led_init();//led灯初始化
	user_page_init();
	user_key_init();//拨码开关
	beep_init();//蜂鸣器初始化设置
	sonic_init();//超声波初始化（pit2定时器）
	NVIC_Init();//确定各中断的优先级
	PIT_Init();//pit初始化（pit0、pit1定时器）
	FTM_Init();
	user_flash_init();
	AD_Init();
	current_init();
	startline_init();
	gpio_init(PTD4,GPO,0);
}


//起跑线
void startline_init()
{
	gpio_init(PTA25, GPI, 1);
}
uint8 get_startline()
{
	return gpio_get(PTA25);
}
//拨码开关初始化
void user_key_init() 
{
	gpio_init(PTE0, GPI, 1);
	gpio_init(PTE1, GPI, 1);
	gpio_init(PTE2, GPI, 1);
	gpio_init(PTE3, GPI, 1);
	gpio_init(PTD0, GPI, 1);
	gpio_init(PTD1, GPI, 1);
	gpio_init(PTD2, GPI, 1);
	gpio_init(PTD3, GPI, 1);
}

//蜂鸣器初始化
void buzzer(uint32 n)
{
	gpio_set(PTD15,1);
	systick_delay_ms(n);
	gpio_set(PTD15,0);
}

//led灯初始化
void user_led_init(void)
{
  gpio_init(PTA17,GPO,1);
  gpio_init(PTE26,GPO,1);
  gpio_init(PTD15,GPO,1);
  gpio_init(PTC0,GPO,1);
}

//按键初始化
void user_press_init(void)
{
  port_init(PTE4,ALT1|IRQ_FALLING|PULLUP);
  port_init(PTE5,ALT1|IRQ_FALLING|PULLUP);
  port_init(PTE6,ALT1|IRQ_FALLING|PULLUP);
  port_init(PTE7,ALT1|IRQ_FALLING|PULLUP);
  port_init(PTE8,ALT1|IRQ_FALLING|PULLUP);
}

//检测串口数据更新
void data_updata(void)
{
      if(UartData[0]==last_data[0] 
         && UartData[1]==last_data[1] && UartData[2]==last_data[2] 
           && UartData[3]==last_data[3] && UartData[4]==last_data[4] 
             && UartData[5]==last_data[5] && UartData[6]==last_data[6] 
               && UartData[7]==last_data[7] && UartData[8]==last_data[8])
      ;
    else
    {
      beep_on();
      DELAY_MS(10);
      beep_off();
      last_data[0]=UartData[0];
      last_data[1]=UartData[1];
      last_data[2]=UartData[2];
      last_data[3]=UartData[3];
      last_data[4]=UartData[4];
      last_data[5]=UartData[5];
      last_data[6]=UartData[6];
      last_data[7]=UartData[7];
      last_data[8]=UartData[8];
    }
}