#include "common.h"
#include "include.h"

//freecars��λ��Э������
int8  UART_Recv_Data;
uint8 From_Computer_Command;
uint8 Rec_Date_Flag;
uint8 Rec_UartDate_Flag;
int16 Date_Buff[15];
extern double UartData[9];
//���ӳ�ͨ�Žӿ�
int8 get_link;
int8 AD_MAX_START_FLAG=0;
int8 Emergency_Stop=0;
int8 Stop_Flag=1;
int8 Launch_Flag=0;
int8 Dis_Flag=0;
extern int16 Set_Speed;
extern int16 Set_Speed_Temp;
extern TIME sys_time;
extern TIME time_start;
TIME time_launch;

char reset_info[15];

void UART5_IRQHandler(void);

//freecars��λ��ͨ�ų������ô����ж�
void uart4_handler(void)
{
  //����
  uart_getchar(UART4,&UART_Recv_Data);
  UART4_IRQHandler();//��������ͨ��Э��
}

//�������ӳ��ӿں���
void uart5_handler(void)
{
  //buzzer(20);
  uart_getchar(UART5,&get_link);
#if RANK_FLAG
  receive_data();
  receive_data_process();
#endif
}

void Diagnostic_Reset_Source(void)
{
#ifdef DEBUG_PRINT 
  
#if (defined(MK60DZ10)) 
  /* Determine the last cause(s) of reset */
  if (MC_SRSH & MC_SRSH_SW_MASK)
  {
    printf("Software Reset\n");
    strcpy(reset_info, "software");
  }
  if (MC_SRSH & MC_SRSH_LOCKUP_MASK)
  {
    printf("Core Lockup Event Reset\n");
    strcpy(reset_info, "core lockup");
  }
  if (MC_SRSH & MC_SRSH_JTAG_MASK)
  {
    printf("JTAG Reset\n");
    strcpy(reset_info, "jtag");
  }
  if (MC_SRSL & MC_SRSL_POR_MASK)
  {
    printf("Power-on Reset\n");
    strcpy(reset_info, "power on");
  }
  if (MC_SRSL & MC_SRSL_PIN_MASK)
  {
    printf("External Pin Reset\n");
    strcpy(reset_info, "external pin");
  }
  if (MC_SRSL & MC_SRSL_COP_MASK)
  {
    printf("Watchdog(COP) Reset\n");
    strcpy(reset_info, "watchdog");
  }
  if (MC_SRSL & MC_SRSL_LOC_MASK)
  {
    printf("Loss of Clock Reset\n");
    strcpy(reset_info, "loss of clk");
  }
  if (MC_SRSL & MC_SRSL_LVD_MASK)
  {
    printf("Low-voltage Detect Reset\n");
    strcpy(reset_info, "low voltage");
  }
  if (MC_SRSL & MC_SRSL_WAKEUP_MASK)
  {
    printf("LLWU Reset\n");
    strcpy(reset_info, "llwu");
  }
#endif
  
#endif
}


void main()
{
	DisableInterrupts;
	SYS_Init();  
	LCD_Init();
	Diagnostic_Reset_Source();
//	beep_on();
//	while(1);
	
	//��λ��ͨ�ų�ʼ��
	uart_init(UART4,115200);
	set_vector_handler(UART4_RX_TX_VECTORn,uart4_handler);
	uart_rx_irq_en(UART4);

	//˫��ͨ�ų�ʼ��
	uart_init(UART5,115200);
	set_vector_handler(UART5_RX_TX_VECTORn,uart5_handler);
	uart_rx_irq_en(UART5);

	set_vector_handler(PORTE_VECTORn,IrqHandler_page);
	enable_irq(PORTE_IRQn);//����������־λ

	
	current_regulate();	//�������ĳ�ʼֵУ׼
	user_flash_read();
	EnableInterrupts;
  while(1)
  {    
	  
    data_updata();//���´��ڷ�������
    set_pid();
    switch_setting();
	
#if RANK_FLAG
	sendtoafter();//������һ������������
    //receive_data();//���������յ�������
#endif
    //����Ϊ����λ��������������д���
    if(Rec_Date_Flag==1)
    {
      Rec_Date_Flag=0;
      switch(From_Computer_Command)	//freecars�������
      {
      case 6:	//F6
        break;
      case 7:	//F7
      	break;
       case 8: 	set_pid();//F8
        break;
      case 9: 	//F9
        break;
      case 10: 	//F10
        break;
      case 11:	//F11
        break;
      case 12: 	//F12
        break;
      case 100: 	//Pause
        break;
      case 101: Emergency_Stop=0;Stop_Flag=0;Set_Speed=Set_Speed_Temp;time_launch=sys_time;time_start=sys_time;		//Home
      	break;
      case 102: 	//Page Up
        break;
      case 103: Emergency_Stop=1;		//Page Down
      break;
      case 104: Stop_Flag=1;Set_Speed=0;		//End
      break;
      default:
        ;
      }
    }
	if(display_or_not)
	{
		if(Set_Speed==0)
		{
			user_lcd_display();
    	}
	}
    uSendOnePage();
  }
}



