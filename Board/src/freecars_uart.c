/*

���ļ�Ϊ�ƽܾ���λ��ͨѶЭ��,��Ҫ������λ��Ϊ�����ſ��ã������͵����Э�鲻ͬ

*/
#include "include.h"
#include "freecars_uart.h"
#include "common.h"
#include "MK60_uart.h"

uint8 uSendBuf[UartDataNum*2]={0};
uint8 FreeCarsDataNum=UartDataNum*2 ;
SerialPortType SerialPortRx;

#if CAR_NUM==1
double UartData[9] = {0.7,      //Spd_Kp
                      8,      //Spd_Ki
                      0.05,    	//Spd_Kd
                      2.2,    
                      0,    
                      95,   
                      200,   
                      268,  //speed-86-263    	//
                      80       //Set_Speed for F8	//���ٵ�ʱ�������ǻ������ٶ�
                      };

#elif CAR_NUM==2

double UartData[9] = {0.7,      //Spd_Kp    0.7
                       9,      //Spd_Ki 1.  15
                      0.05,
                      2.2,  
                      0,  
                      85, 
                      185,
                      250, //243
                      80       //Set_Speed for F8	//���ٵ�ʱ�������ǻ������ٶ�
                      };

#elif CAR_NUM==3

double UartData[9] = {3,      //Spd_Kp
                      5,      //Spd_Ki
                      0.5,    	//Spd_Kd
                      0,  
                      0,  
                      85, 
                      185,
                      250, //243
                      80       //Set_Speed for F8	//���ٵ�ʱ�������ǻ������ٶ�
                      };
#endif

extern int8 UART_Recv_Data;
extern uint8 From_Computer_Command;
extern uint8 Rec_Date_Flag;
extern uint8 Rec_UartDate_Flag;
extern int16 Date_Buff[15];

/*
int8 UART_Recv_Data;
uint8 From_Computer_Command;
uint8 Rec_Date_Flag;
uint8 Rec_UartDate_Flag;
int16 Date_Buff[15];
*/

void UART4_IRQHandler(void)
{
  int i,b,d1;
  unsigned long int d;
	//uart_getchar(UART4,&SerialPortRx.Data);
	
    SerialPortRx.Data = UART_Recv_Data; 
     
    if( SerialPortRx.Stack < UartRxBufferLen )
    {
      SerialPortRx.Buffer[SerialPortRx.Stack++] = SerialPortRx.Data;
      
      if(   SerialPortRx.Stack >= UartRxDataLen  //UartRxDataLen����Ϊһ֡
         && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen]  ==0xff //У����ͷ
           && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen+1]==0x55
             && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen+2]==0xaa
               && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen+3]==0x10 )
      {   //double data 9��ͨ������У��
        SerialPortRx.Check = 0;
        b = SerialPortRx.Stack - UartRxDataLen; //��ʼλ
        for(i=b; i<SerialPortRx.Stack-1; i++)  //��У��λ���λ����У��
        {
          SerialPortRx.Check += SerialPortRx.Buffer[i];//У��
        }
        
        if( SerialPortRx.Check == SerialPortRx.Buffer[SerialPortRx.Stack-1] )
        {   //У��ɹ����������ݽ���
          for(i = 0; i<9; i++)
          {
            d = SerialPortRx.Buffer[b+i*4+4]*0x1000000L + SerialPortRx.Buffer[b+i*4+5]*0x10000L + SerialPortRx.Buffer[b+i*4+6]*0x100L + SerialPortRx.Buffer[b+i*4+7];
            if(d>0x7FFFFFFF)
            {
              d1 = 0x7FFFFFFF - d;
            }
            else
            {
              d1 = d;
            }
            UartData[i]=d1;
            UartData[i]/=65536.0;
          }
        //  UartDebug();  תȥ�������ܵ������ݸ�������
          Rec_UartDate_Flag=1;
          for(i=2;i<=8;i++)
          {
           Date_Buff[i]=(int16)UartData[i];
          }
          Date_Buff[9]=(int16)UartData[0];
          Date_Buff[11]=(int16)UartData[1];
        }
        SerialPortRx.Stack = 0;
      }
      else if(   SerialPortRx.Stack >= UartRxCmdLen //UartRxDataLen����Ϊһ֡
              && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen]  ==0xff
                && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen+1]==0x55
                  && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen+2]==0xaa
                    && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen+3]==0x77 )//cmd
      {
        SerialPortRx.Check = 0;
        b = SerialPortRx.Stack - UartRxCmdLen; //��ʼλ
        for(i=b; i<SerialPortRx.Stack-1; i++)  //��У��λ���λ����У��
        {
          SerialPortRx.Check += SerialPortRx.Buffer[i];//У��
        }
        if( SerialPortRx.Check == SerialPortRx.Buffer[SerialPortRx.Stack-1] )
        {   //У��ɹ�
          Rec_Date_Flag=1;
          From_Computer_Command=UartCmdData;//������յ����������MCU�������
        }
        SerialPortRx.Stack = 0;
      }
    }
    else
    {
      SerialPortRx.Stack = 0;
    } 
	//printf("get");
}

void uSendOnePage(void)
{
  uint8 i,sum=0; 
  
  uart_putchar(UART4,251);
  uart_putchar(UART4,109);
  uart_putchar(UART4,37);//ʹ����ѯ�ķ�ʽ�������ݣ�������δ���ͣ�����ͣ�ڴ˴�ֱ���������
  sum+=(251);  
  sum+=(109);
  sum+=(37);
  for(i=0;i<FreeCarsDataNum;i++)
  {
    uart_putchar(UART4,uSendBuf[i]);
    sum+=uSendBuf[i];         
  }
  uart_putchar(UART4,sum);
}
/*
��ĳ��ͨ���������
adr��ͨ��
date������-32768~32767
*/
void push(uint8 adr,uint16 date)
{
    uSendBuf[adr*2]=date/256;
    uSendBuf[adr*2+1]=date%256;
}



