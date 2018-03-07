/*
* @file       user_sys.c
* @brief      ϵͳ�йغ���
* @author     Benson
*/

#include "include.h"
double last_data[9];
extern double UartData[9];
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
	 NVIC_SetPriority(PIT3_IRQn, 4);		//PIT1�ж�
	 NVIC_EnableIRQ(PIT3_IRQn);
}
void beep_init()//��������ʼ��
{
	gpio_init(PTD4,GPO,0);
	gpio_init(PTD15,GPO,1);
	//gpio_init(PTE26,GPO,1);
}

void beep_on()//��������
{
	gpio_set(PTD4,1);
	//gpio_set(PTE26,0);
}

void beep_off()//�������ر�
{
	gpio_set(PTD4,0);
}
void SYS_Init()
{
	user_press_init();//������ʼ��
	user_led_init();//led�Ƴ�ʼ��
	user_page_init();
	user_key_init();//���뿪��
	beep_init();//��������ʼ������
	sonic_init();//��������ʼ����pit2��ʱ����
	NVIC_Init();//ȷ�����жϵ����ȼ�
	PIT_Init();//pit��ʼ����pit0��pit1��ʱ����
	FTM_Init();
	user_flash_init();
	AD_Init();
	current_init();
	startline_init();
	gpio_init(PTD4,GPO,0);
}


//������
void startline_init()
{
	gpio_init(PTA25, GPI, 1);
}
uint8 get_startline()
{
	return gpio_get(PTA25);
}
//���뿪�س�ʼ��
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

//��������ʼ��
void buzzer(uint32 n)
{
	gpio_set(PTD15,1);
	systick_delay_ms(n);
	gpio_set(PTD15,0);
}

//led�Ƴ�ʼ��
void user_led_init(void)
{
  gpio_init(PTA17,GPO,1);
  gpio_init(PTE26,GPO,1);
  gpio_init(PTD15,GPO,1);
  gpio_init(PTC0,GPO,1);
}

//������ʼ��
void user_press_init(void)
{
  port_init(PTE4,ALT1|IRQ_FALLING|PULLUP);
  port_init(PTE5,ALT1|IRQ_FALLING|PULLUP);
  port_init(PTE6,ALT1|IRQ_FALLING|PULLUP);
  port_init(PTE7,ALT1|IRQ_FALLING|PULLUP);
  port_init(PTE8,ALT1|IRQ_FALLING|PULLUP);
}

//��⴮�����ݸ���
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