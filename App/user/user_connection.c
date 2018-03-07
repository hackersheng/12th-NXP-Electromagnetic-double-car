#include "common.h"
#include "include.h"

//ͷУ��λ 0xff,βУ��λ0xfe;
//˫��ͨ�ų�������λ

extern int8 RING_WAIT_FLAG;
extern int8 get_link;
extern int8 Launch_Flag;
extern uint16 Distance[3];//�����Ǳ���Ҫ����
extern TIME time_ring_launch;

#if CAR_NUM==1
int8 front_or_after = 2; //ǰ�����Ǻ�
int8 last_front_or_after = 1;
#elif CAR_NUM== 2
int8 front_or_after=1; //ǰ�����Ǻ�
int8 last_front_or_after = 2;
#elif CAR_NUM==3
int8 front_or_after = 2; //ǰ�����Ǻ�
int8 last_front_or_after = 1;

#else

#endif

extern int16 Set_Speed;//�趨���ٶ�
extern int8 Stop_Flag;
extern int8 Emergency_Stop;

extern int8 Road_Type;
int8 num_send=6;
int8 link_success;
int8 link_flag=0;
uint8 Receive_Data[6];
uint8 Receive_Data1[6];

extern TIME time_launch;
extern TIME sys_time;

void sendtoafter(void)
{	
	uart_putchar(UART5,(char)(127));//ͷ����λ
	uart_putchar(UART5,(char)(Distance[2]/100));//�������λ
	uart_putchar(UART5,(char)(front_or_after));//ǰ�����Ǻ�
	uart_putchar(UART5,(char)(Distance[2]%100));//��һ���������趨���ٶ�
	
	//��ʹ���������Ʒ��������þ��봥��
	//��������������
	//uart_putchar(UART5,(char)(Stop_Flag));
	uart_putchar(UART5,(char)Road_Type);
	uart_putchar(UART5,(char)(126));//βУ��λ  
}
void receive_data(void)
{
  int8 enter_flag=0;//����ifֻ�ܽ���һ����������ʧЧ����
  if(get_link==127)//ͷ����λ���ȼ����
  {
    link_success=127;
    link_flag=0;//�൱�ڼ����������ڼ�������λ
    enter_flag++;
  }
  if((link_success==127)&&(link_flag>0)&&(link_flag<5))
  {
    if(link_flag==1)
    {
      Receive_Data[(int8)(link_flag)-1]=(int8)(get_link);
      enter_flag++;
    }
    else
    {
      Receive_Data[(int8)(link_flag)-1]=(int8)(get_link);
      enter_flag++;
    }
  }
  if(get_link==126)
  {
    link_flag++;
    if(link_flag==6)
    {
      Receive_Data1[0]=Receive_Data[0];
      Receive_Data1[1]=Receive_Data[1];
      Receive_Data1[2]=Receive_Data[2];
      Receive_Data1[3]=Receive_Data[3];
    }
  }
  if((enter_flag!=1)||(link_flag>=5))
  {
    link_success=0;
    link_flag=0;
  }
  else
  {
    link_flag++;
  }
}

void receive_data_process()
{
	//0����/10��1ǰ��2�ٶ��趨ֵ��3Road_Type
	if(RING_WAIT_FLAG==1 && Receive_Data1[1]==1)	//����֮�����ǰ��������������
	{
		RING_WAIT_FLAG=0;
        time_launch=sys_time;//�ı䷢��ʱ�䣬��������������Ϊ�µķ���ʱ�䡣
        time_ring_launch=sys_time;
        
	}
	if(front_or_after==1)
		Distance[2]=Receive_Data1[0]*100+Receive_Data1[2];
//	if(CAR_NUM==1)	//1����2�����ƣ�2������λ������
//	{
//		if (Receive_Data1[3] == 1)
//			Stop_Flag = 1;
//		else
//		{
//			if(Stop_Flag==1)
//				time_launch=sys_time;
//			Stop_Flag = 0;
//			Emergency_Stop = 0;
//                        
//		}
//	}
}
/*
void display_receive(void)
{
  //LCD_Fill(0x00);
  LCD_P8x16Str(0,0,"Dis_H");
  LCD_P8x16Str(0,2,"F_or_A");
  LCD_P8x16Str(0,4,"Dis_L");
  LCD_P8x16Str(0,6,"Stop_F");
  LCD_P8x16Num(75,0,Receive_Data1[0]);
  LCD_P8x16Num(75,2,Receive_Data1[1]);
  LCD_P8x16Num(75,4,Receive_Data1[2]);
  LCD_P8x16Num(75,6,Receive_Data1[3]);  
}
*/