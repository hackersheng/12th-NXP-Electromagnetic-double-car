/*

该文件为黄杰军上位机通讯协议,需要配置上位机为电磁组才可用，光电组和电磁组协议不同

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
                      80       //Set_Speed for F8	//变速的时候这里是环道的速度
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
                      80       //Set_Speed for F8	//变速的时候这里是环道的速度
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
                      80       //Set_Speed for F8	//变速的时候这里是环道的速度
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
      
      if(   SerialPortRx.Stack >= UartRxDataLen  //UartRxDataLen个数为一帧
         && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen]  ==0xff //校验字头
           && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen+1]==0x55
             && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen+2]==0xaa
               && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen+3]==0x10 )
      {   //double data 9个通道数据校验
        SerialPortRx.Check = 0;
        b = SerialPortRx.Stack - UartRxDataLen; //起始位
        for(i=b; i<SerialPortRx.Stack-1; i++)  //除校验位外的位进行校验
        {
          SerialPortRx.Check += SerialPortRx.Buffer[i];//校验
        }
        
        if( SerialPortRx.Check == SerialPortRx.Buffer[SerialPortRx.Stack-1] )
        {   //校验成功，进行数据解算
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
        //  UartDebug();  转去处理，把受到的数据付给变量
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
      else if(   SerialPortRx.Stack >= UartRxCmdLen //UartRxDataLen个数为一帧
              && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen]  ==0xff
                && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen+1]==0x55
                  && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen+2]==0xaa
                    && SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen+3]==0x77 )//cmd
      {
        SerialPortRx.Check = 0;
        b = SerialPortRx.Stack - UartRxCmdLen; //起始位
        for(i=b; i<SerialPortRx.Stack-1; i++)  //除校验位外的位进行校验
        {
          SerialPortRx.Check += SerialPortRx.Buffer[i];//校验
        }
        if( SerialPortRx.Check == SerialPortRx.Buffer[SerialPortRx.Stack-1] )
        {   //校验成功
          Rec_Date_Flag=1;
          From_Computer_Command=UartCmdData;//处理接收到的命令，付给MCU命令变量
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
  uart_putchar(UART4,37);//使用轮询的方式发送数据，当数据未发送，程序停在此处直到发送完成
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
向某个通道填充数据
adr：通道
date：数据-32768~32767
*/
void push(uint8 adr,uint16 date)
{
    uSendBuf[adr*2]=date/256;
    uSendBuf[adr*2+1]=date%256;
}



