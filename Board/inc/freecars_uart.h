#ifndef __FREECARS_UART_H__
#define __FREECARS_UART_H__

#define uchar unsigned char
#define UartDataNum      17	     //FreeCars��λ������ͨ������������λ�����øı�

//���²�Ҫ�޸�
#define UartRxBufferLen  100
#define UartRxDataLen    41           //FreeCars��λ�����͸�������MCU���գ���Ҫ��
#define UartRxCmdLen     7	      //FreeCars��λ�������������ݳ��ȣ���Ҫ��

#define UartCmdNum  SerialPortRx.Buffer[SerialPortRx.Stack-3]//�����
#define UartCmdData SerialPortRx.Buffer[SerialPortRx.Stack-2]//��������
//�������ݵĳ���ֻҪ�����鳤��Ϊ26=22+3+1������Ϊ���뷢���ַ���ȡ��ͳһ
//ȡ���ݵĳ������ַ����ĳ�����ȣ������ڷ��������ǻ�෢����һЩ
//��Чλ������Ӱ�첻���
typedef struct 
{
  int Stack;
  uchar Data;
  uchar PreData;
  uchar Buffer[UartRxBufferLen];
  uchar Enable;
  uchar Check;
}SerialPortType;

extern uchar uSendBuf[UartDataNum*2];
extern SerialPortType SerialPortRx;
extern double UartData[9];

void uSendOnePage(void);
void push(uchar adr,unsigned short int date);
extern void UART4_IRQHandler(void);
//void UART4_IRQHandler(void);

#endif 
